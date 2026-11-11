/*
 * A camera image sensor's I2C control interface
 *
 * The register file a sensor driver programs over I2C: the chip ID its
 * detect routine checks, and every other register reading back what was
 * last written to it. The register address is one or two bytes, big
 * endian, and auto-increments across a burst, as on the parts listed.
 * An 8-bit part that banks its registers picks the page with one of them:
 * GalaxyCore's 0xfe, which leaves 0xf0-0xff on every page, OmniVision's
 * 0xfd or PixelPlus's 0xef.
 *
 * The IDs are what the vendor drivers' sensor_detect() reads.
 *
 * It is also the camera: the scene it sees, at "fps" frames a second, is
 * what the SoC's video input takes in, rendered as the ISP puts it out.
 * A board with several sensors gives each its "input", the VIC (or the
 * MIPI switch's port) it feeds: input 1 sees the scene mirrored, 2 upside
 * down and 3 both, so that a stream shows which sensor it came from.
 * With "noise" set the scene's luma carries noise that changes every frame,
 * which no prediction removes: pictures an encoder cannot code small.
 * With "sensor-noise" set every Bayer sample carries a sensor's noise
 * instead, independent from frame to frame: a read-noise floor and a shot
 * noise whose variance grows with the level, both with the analog gain the
 * guest programs, as image_sensor_noise_law() measured them. "read-noise"
 * (the floor's sigma) and "shot-noise" (the variance per level), at 1x gain
 * in the 16-bit samples of image_sensor_render_bayer(), replace the
 * measured ones when set.
 *
 * On the parts whose exposure registers are known the picture follows the
 * exposure in linear light: as rendered at the part's reference (lines at
 * 1x gain, a well-lit room), twice as bright at twice that; the others
 * are taken at their reference. "light" is the scene's brightness in
 * percent of that room's; it can change at any time, and "light-to" ramps
 * it there over "light-secs" seconds of frames (dusk, a light switched
 * off). "kelvin" is the light's colour temperature (6500 as rendered),
 * which the picture shows as the ISP's white balance would have to take
 * out; the model's ISPs apply none.
 *
 * The raw signal a grey gives, which the ISPs' AWB statistics see, is the
 * sensor's own: a table of its raw R/G and B/G by the light's temperature,
 * from the AWB calibration of the sensor's tuning file for the part the guest
 * runs, for the models that have one (image-sensor-raw.c.inc). "raw-r" and
 * "raw-b" give a grey's raw R/G and B/G in 1/256ths, whatever the light, in
 * its place.
 *
 * IR reaches the picture while the IR-cut filter is out: the light's own
 * ("ambient-ir", percent of its visible light) and the IR LEDs' ("ir",
 * percent of the room's light at full power), on the reflectances the
 * picture shows, and without colour. "ircut" names the board's filter
 * pins as thingino.json does (a number, 'o' after it for active low;
 * spaces or commas between, a comma doubled on the command line): one
 * pin's level sets it (high: in), or a pulse on the first of two puts it
 * in and on the second takes it out, latching. "irled" names the LEDs'
 * GPIOs, or pwmN for a PWM channel, its duty the LED's power.
 *
 * With "object" a white square crosses the ramp and back every 4 seconds;
 * "motion" off stands it and the stripe still. With "stamp" the frame's
 * number and the sensor's input are drawn top left as a row of 32 cells,
 * black 0 and white 1, at full contrast whatever the exposure, unmirrored:
 * 1 0, the input's 2 bits, the frame's low 24 bits (high first), the even
 * parity of those 26, then 1 0 1.
 *
 * With "scene" naming a YUV4MPEG2 file (4:2:0, as ffmpeg's -f yuv4mpegpipe
 * -pix_fmt yuv420p writes) or a FIFO the scene is that video instead of the
 * bars: a file loops, the guest's frame n showing the file's frame n
 * modulo its frames; a FIFO gives the latest frame written to it, read by
 * a thread of its own so that nothing waits. Each picture is the frame
 * scaled to the size it is rendered at; the light, IR, noise, input mirror
 * and flip and the stamp apply as they do to the bars, but not "object",
 * which is the bars'.
 *
 * A GC4653 or GC5603 set up by its driver's DOL table runs WDR: two
 * exposures a frame, the long one's integration time where the linear mode
 * keeps it, the short one's at 0x0200 and 0x0201 (a T41LQ's GC5603 reads
 * the exposures in the lines' ratio in its ISP's statistics). The picture
 * is taken at the long one, the short one the light over their ratio, as a
 * T31 asks them of a GC4653 in WDR mode (its ISP's AE0 and AE1); put out,
 * the two are fused: the long exposure's light, the short's where the long
 * saturates, up to the ratio, the brightest of it compressed to full scale
 * (l / (1 + l (k - 1) / k), the model's curve).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/cutils.h"
#include "qemu/thread.h"
#ifndef _WIN32
#include <poll.h>
#endif
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/sensor/image-sensor.h"
#include "trace.h"
#include <math.h>

/*
 * Integration time, as the vendor drivers write it: the sum of up to three
 * registers' fields, (value & mask) << shift, in 1/2^frac_bits lines; and
 * the gain, 1x = 1024. ref is the exposure in lines at 1x that shows the
 * scene as rendered.
 */
typedef struct ImageSensorExposure {
    struct {
        uint16_t reg;
        uint8_t mask;
        uint8_t shift;
    } it[3];
    uint8_t frac_bits;
    uint32_t (*gain)(ImageSensorState *s);
    uint32_t ref;
    /*
     * A shutter in place of an integration time: the frame length's
     * registers (high, low), the integration time that length less the
     * field
     */
    uint16_t vts[2];
} ImageSensorExposure;

/* GC2053: analog gain by the (0xb4, 0xb3) pair of the driver's table */
static uint32_t image_sensor_gc2053_gain(ImageSensorState *s)
{
    static const struct {
        uint16_t reg;
        uint32_t gain;
    } table[] = {
        { 0x000, 1024 }, { 0x010, 1184 }, { 0x020, 1424 }, { 0x030, 1632 },
        { 0x040, 2032 }, { 0x050, 2352 }, { 0x060, 2832 }, { 0x070, 3248 },
        { 0x080, 4160 }, { 0x090, 4800 }, { 0x0a0, 5776 }, { 0x0b0, 6640 },
        { 0x0c0, 8064 }, { 0x05a, 9296 }, { 0x083, 11552 }, { 0x093, 13312 },
        { 0x084, 16432 }, { 0x094, 18912 }, { 0x12c, 22528 },
        { 0x13c, 25936 }, { 0x08c, 31840 }, { 0x09c, 36656 },
        { 0x264, 45600 }, { 0x274, 52512 }, { 0x0c6, 64768 },
        { 0x0dc, 82880 }, { 0x285, 88000 }, { 0x295, 107904 },
        { 0x0ce, 113168 },
    };
    uint16_t reg = (s->regs[0xb4] & 3) << 8 | s->regs[0xb3];
    /* 0xb1:0xb2 is a pre-gain in 1/64ths */
    uint32_t pregain = s->regs[0xb1] << 6 | s->regs[0xb2] >> 2;

    for (int i = 0; i < ARRAY_SIZE(table); i++) {
        if (table[i].reg == reg) {
            return (uint64_t)table[i].gain * pregain / 64;
        }
    }
    return pregain * 16;
}

static const ImageSensorExposure image_sensor_gc2053_exposure = {
    .it = { { 0x03, 0x3f, 8 }, { 0x04, 0xff, 0 } },
    .gain = image_sensor_gc2053_gain,
    .ref = 600,
};

/*
 * SmartSens: analog gain coarse (0x3e08 bits 4:2, ones from the bottom
 * doubling it) times fine (0x3e09 in 1/16ths), digital gain coarse
 * (0x3e06 likewise) times fine (0x3e07 in 1/128ths)
 */
static uint32_t image_sensor_smartsens_gain(ImageSensorState *s)
{
    uint32_t again = (extract32(s->regs[0x3e08], 2, 3) + 1) *
                     s->regs[0x3e09] * 64;
    uint32_t dgain = (extract32(s->regs[0x3e06], 0, 4) + 1) *
                     s->regs[0x3e07] * 8;

    return (uint64_t)again * dgain / 1024;
}

/* integration time in half lines, from bit 4 of 0x3e02 */
static const ImageSensorExposure image_sensor_sc4236_exposure = {
    .it = { { 0x3e00, 0x0f, 16 }, { 0x3e01, 0xff, 8 }, { 0x3e02, 0xf0, 0 } },
    .frac_bits = 5,
    .gain = image_sensor_smartsens_gain,
    .ref = 1000,
};

/* OS02G10: page 1, the gain in 1/16ths */
static uint32_t image_sensor_os02g10_gain(ImageSensorState *s)
{
    return s->regs[0x124] * 64;
}

static const ImageSensorExposure image_sensor_os02g10_exposure = {
    .it = { { 0x103, 0xff, 8 }, { 0x104, 0xff, 0 } },
    .gain = image_sensor_os02g10_gain,
    .ref = 600,
};

/* OS08C10: the gain in 1/128ths, 0x3508 bits 5:0 its high byte */
static uint32_t image_sensor_os08c10_gain(ImageSensorState *s)
{
    return ((s->regs[0x3508] & 0x3f) << 8 | s->regs[0x3509]) * 8;
}

static const ImageSensorExposure image_sensor_os08c10_exposure = {
    .it = { { 0x3501, 0xff, 8 }, { 0x3502, 0xff, 0 } },
    .gain = image_sensor_os08c10_gain,
    .ref = 1250,
};

/* CV2005: analog gain 256 / (256 - 0x3118), to 32x at 0xf8 */
static uint32_t image_sensor_cv2005_gain(ImageSensorState *s)
{
    return 1024 * 256 / (256 - MIN(s->regs[0x3118], 0xf8));
}

/* the shutter at 0x304a:0x3049:0x3048, the frame length at 0x301d:0x301c */
static const ImageSensorExposure image_sensor_cv2005_exposure = {
    .it = { { 0x304a, 0xff, 16 }, { 0x3049, 0xff, 8 }, { 0x3048, 0xff, 0 } },
    .gain = image_sensor_cv2005_gain,
    .ref = 1500,
    .vts = { 0x301d, 0x301c },
};

/*
 * SOI (JX-): the gain at 0x00, a power of two in bits 6:4 times 1 + bits
 * 3:0 / 16; the integration time in lines at 0x02:0x01. Only the JXF22
 * follows it: the T10's, T21's and T32's AE loops do not settle a JXH42 or
 * JXF23 at the scene as rendered within a stream's first seconds
 */
static uint32_t image_sensor_soi_gain(ImageSensorState *s)
{
    return (16 + (s->regs[0x00] & 0xf)) * 64 << extract32(s->regs[0x00], 4, 3);
}

static const ImageSensorExposure image_sensor_soi_exposure = {
    .it = { { 0x02, 0xff, 8 }, { 0x01, 0xff, 0 } },
    .gain = image_sensor_soi_gain,
    .ref = 600,
};

/*
 * A two-exposure (DOL) WDR mode: on while @mode reads @on, which a write
 * of @reset_val to @reset (the part's soft reset) clears; each exposure's
 * integration time in lines, (high & 0x3f) << 8 | low
 */
typedef struct ImageSensorWdr {
    uint16_t mode;
    uint8_t on;
    uint16_t reset;
    uint8_t reset_val;
    uint16_t it_long[2], it_short[2];
} ImageSensorWdr;

/* GC4653 and GC5603: what the DOL table alone writes, 0x0107 */
static const ImageSensorWdr image_sensor_gc_wdr = {
    .mode = 0x0107, .on = 0x89, .reset = 0x03fe, .reset_val = 0xf0,
    .it_long = { 0x0202, 0x0203 }, .it_short = { 0x0200, 0x0201 },
};

#define IMAGE_SENSOR_RAW_POINTS 10

/* A grey's raw R/G and B/G, 1/1024ths, at image_sensor_raw_kelvin[] */
struct ImageSensorRaw {
    uint16_t r[IMAGE_SENSOR_RAW_POINTS];
    uint16_t b[IMAGE_SENSOR_RAW_POINTS];
};

#include "image-sensor-raw.c.inc"

struct ImageSensorModel {
    const char *name;
    uint8_t addr;               /* the 7-bit I2C address it answers */
    uint8_t reg_bytes;          /* register address width */
    uint8_t page_reg;           /* a banked part's page select, else 0 */
    struct {
        uint16_t reg;
        uint8_t val;
    } id[4];
    uint16_t stream_reg;        /* frames flow while these bits are set */
    uint8_t stream_mask;        /* 0: the part always streams */
    bool stream_standby;        /* standby bits: frames flow while clear */
    const ImageSensorExposure *exposure;
    const ImageSensorWdr *wdr;
    /*
     * The mode the T31 driver starts in: the lines of its frame
     * (sensor_attr.total_height) at its nominal rate, from which the line
     * rate follows. A lower frame rate stretches the blanking and leaves it;
     * 0: the driver's mode is not known here.
     */
    uint16_t lines;
    uint8_t lines_fps;
    /*
     * The same for the T21's, T23's, Apical cores' (T30), T40's and T41's
     * drivers, whose mode can differ from the T31's or which have none here;
     * lines 0: the T31's
     */
    struct {
        uint16_t lines;
        uint8_t fps;
    } t21, t23, apical, t40, t41;
};

static const ImageSensorModel image_sensor_models[] = {
    { "jxh42",   0x30, 1, 0,     { { 0x0a, 0xa0 }, { 0x0b, 0x42 },
                                   { 0x09, 0x81 } },
      0x12, 0x40, true },
    { "jxf22",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x22 } },
      .exposure = &image_sensor_soi_exposure },
    { "sc2232h", 0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x07 },
                                   { 0x3109, 0x01 } },
      0x0100, 0x01,
      .t21 = { 1500, 25 } },
    { "sc2336",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x3a } },
      0x0100, 0x01,
      .lines = 1440, .lines_fps = 25 },
    { "sc4236",  0x30, 2, 0,     { { 0x3107, 0x32 }, { 0x3108, 0x35 } },
      0x0100, 0x01, .exposure = &image_sensor_sc4236_exposure,
      .lines = 2000, .lines_fps = 25,
      .apical = { 1920, 25 } },
    { "gc2053",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } },
      .exposure = &image_sensor_gc2053_exposure,
      .lines = 1418, .lines_fps = 30,
      .t21 = { 1350, 25 } },
    /* the T41 driver reads 0x3003 and 0x3002, the T32 one 0x3011 and 0x3138 */
    { "cv2003",  0x35, 2, 0,     { { 0x3002, 0x04 }, { 0x3003, 0x20 },
                                   { 0x3011, 0x20 }, { 0x3138, 0x03 } },
      0x3000, 0x01, true },
    /* the T33 SDK's driver reads 0x3003, then 0x3002 */
    { "cv2005",  0x35, 2, 0,     { { 0x3003, 0x20 }, { 0x3002, 0x05 } },
      0x3000, 0x01, true, .exposure = &image_sensor_cv2005_exposure,
      .lines = 3624, .lines_fps = 15 },
    /* 0x3003 is the high byte: the T32, T40 and T41 drivers agree */
    { "cv8001",  0x35, 2, 0,     { { 0x3002, 0x01 }, { 0x3003, 0x80 } },
      0x3000, 0x01, true },
    { "imx307",  0x1a, 2, 0,     { { 0x3008, 0xa0 }, { 0x301e, 0xb2 } },
      0x3000, 0x01, true,
      .lines = 1125, .lines_fps = 30 },
    { "gc4023",  0x29, 2, 0,     { { 0x03f0, 0x40 }, { 0x03f1, 0x23 } },
      .lines = 1500, .lines_fps = 30,
      .t41 = { 1875, 20 } },
    { "gc5613",  0x31, 2, 0,     { { 0x03f0, 0x56 }, { 0x03f1, 0x03 } },
      0x0100, 0x01,
      .t41 = { 1750, 30 } },
    /* gc2063 and gc4663 answer as gc2053 and gc4653 do */
    { "gc2023",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x23 } },
      0xf2, 0x0f },
    { "gc2063",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } },
      .exposure = &image_sensor_gc2053_exposure,
      .lines = 1418, .lines_fps = 30 },
    { "gc1084",  0x37, 2, 0,     { { 0x03f0, 0x10 }, { 0x03f1, 0x84 } },
      .lines = 750, .lines_fps = 30 },
    /* the drivers stream with 0x023e = 0x99 and stop with 0x00 */
    { "gc2083",  0x37, 2, 0,     { { 0x03f0, 0x20 }, { 0x03f1, 0x83 } },
      0x023e, 0x80,
      .lines = 1350, .lines_fps = 25 },
    { "gc3003a", 0x37, 2, 0,     { { 0x03f0, 0x30 }, { 0x03f1, 0x03 },
                                   { 0x036a, 0x10 } },
      .t23 = { 1340, 30 } },
    { "gc4653",  0x29, 2, 0,     { { 0x03f0, 0x46 }, { 0x03f1, 0x53 } },
      .wdr = &image_sensor_gc_wdr,
      .lines = 1800, .lines_fps = 30,
      .t23 = { 1800, 25 },
      .t40 = { 1920, 25 } },
    { "gc4663",  0x29, 2, 0,     { { 0x03f0, 0x46 }, { 0x03f1, 0x53 } } },
    /* gc5603 answers as gc5613 does */
    { "gc5603",  0x31, 2, 0,     { { 0x03f0, 0x56 }, { 0x03f1, 0x03 } },
      .wdr = &image_sensor_gc_wdr },
    /*
     * 0x3008 reads its reset value, as an imx307 does: older sinfo tables
     * see one
     */
    { "imx327",  0x1a, 2, 0,     { { 0x3008, 0xa0 }, { 0x301e, 0xb2 },
                                   { 0x301f, 0x01 } },
      0x3000, 0x01, true,
      .t23 = { 2781, 25 } },
    { "jxf23",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x23 } },
      .lines = 1350, .lines_fps = 25 },
    { "jxf28",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x28 } } },
    { "jxf37",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x37 } },
      .lines = 1350, .lines_fps = 25 },
    { "jxh62",   0x30, 1, 0,     { { 0x0a, 0xa0 }, { 0x0b, 0x62 } },
      0x12, 0x40, true,
      .lines = 900, .lines_fps = 25 },
    { "jxh63p",  0x40, 1, 0,     { { 0x0a, 0x08 }, { 0x0b, 0x48 } },
      .t23 = { 3000, 30 } },
    { "jxk04",   0x40, 1, 0,     { { 0x0a, 0x04 }, { 0x0b, 0x04 } },
      0x12, 0x40, true },
    { "jxq03",   0x40, 1, 0,     { { 0x0a, 0x05 }, { 0x0b, 0x07 } },
      .lines = 1600, .lines_fps = 25 },
    { "jxq03p",  0x40, 1, 0,     { { 0x0a, 0x08 }, { 0x0b, 0x43 } },
      .lines = 1600, .lines_fps = 25 },
    { "mis2008", 0x30, 2, 0,     { { 0x3000, 0x20 }, { 0x3001, 0x08 } },
      .lines = 1350, .lines_fps = 25 },
    { "mis20c1", 0x30, 2, 0,     { { 0x3000, 0x20 }, { 0x3001, 0xc1 } } },
    /* the T32 drivers check 0x05 as well */
    { "os02b10", 0x3c, 1, 0xfd,  { { 0x02, 0x23 }, { 0x03, 0x08 } },
      .t21 = { 1361, 25 } },
    { "os02g10", 0x3c, 1, 0xfd,  { { 0x02, 0x56 }, { 0x03, 0x02 },
                                   { 0x04, 0x47 } },
      .exposure = &image_sensor_os02g10_exposure,
      .lines = 1330, .lines_fps = 25 },
    { "os02n10", 0x3c, 1, 0xfd,  { { 0x02, 0x53 }, { 0x03, 0x02 },
                                   { 0x04, 0x4e }, { 0x05, 0x10 } },
      .t23 = { 1109, 30 } },
    { "os03b10", 0x3c, 1, 0xfd,  { { 0x02, 0x53 }, { 0x03, 0x03 },
                                   { 0x04, 0x42 } },
      .lines = 1590, .lines_fps = 25 },
    { "os04d10", 0x3c, 1, 0xfd,  { { 0x02, 0x53 }, { 0x03, 0x04 },
                                   { 0x04, 0x44 }, { 0x05, 0x10 } } },
    { "os05a10", 0x36, 2, 0,     { { 0x300a, 0x53 }, { 0x300b, 0x05 },
                                   { 0x300c, 0x41 } },
      0x0100, 0x01 },
    { "os08a10", 0x36, 2, 0,     { { 0x300a, 0x53 }, { 0x300b, 0x08 },
                                   { 0x300c, 0x41 } },
      0x0100, 0x01 },
    { "os08c10", 0x36, 2, 0,     { { 0x300a, 0x53 }, { 0x300b, 0x08 },
                                   { 0x300c, 0x43 } },
      0x0100, 0x01, .exposure = &image_sensor_os08c10_exposure },
    { "ov2710",  0x36, 2, 0,     { { 0x300a, 0x27 }, { 0x300b, 0x10 } },
      0x4202, 0x0f, true },
    /* 0x04 is the revision, which the driver takes as 0x06 or 0x07 */
    { "ov2735b", 0x3c, 1, 0xfd,  { { 0x02, 0x27 }, { 0x03, 0x35 },
                                   { 0x04, 0x07 } },
      0x36, 0x01, true,
      .t21 = { 1595, 25 } },
    { "ov9712",  0x30, 1, 0,     { { 0x0a, 0x97 }, { 0x0b, 0x11 } },
      0x09, 0x10, true },
    { "ov9732",  0x36, 2, 0,     { { 0x300a, 0x97 }, { 0x300b, 0x32 } },
      0x0100, 0x01,
      .t23 = { 974, 25 } },
    { "ov9750",  0x36, 2, 0,     { { 0x300b, 0x97 }, { 0x300c, 0x50 } },
      0x0100, 0x01 },
    { "ps5250",  0x48, 1, 0xef,  { { 0x00, 0x52 }, { 0x01, 0x50 } } },
    { "ps5260",  0x48, 1, 0xef,  { { 0x00, 0x52 }, { 0x01, 0x60 } },
      .t21 = { 1125, 15 },
      .t23 = { 1125, 15 } },
    { "sc1235",  0x30, 2, 0,     { { 0x3107, 0x12 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
    { "sc1a4t",  0x30, 2, 0,     { { 0x3107, 0xda }, { 0x3108, 0x4d } },
      0x0100, 0x01,
      .t23 = { 3000, 15 } },
    { "sc2135",  0x30, 2, 0,     { { 0x3107, 0x21 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
    { "sc2232",  0x30, 2, 0,     { { 0x3107, 0x22 }, { 0x3108, 0x32 },
                                   { 0x3109, 0x01 } },
      0x0100, 0x01,
      .t21 = { 1500, 25 } },
    { "sc2235",  0x30, 2, 0,     { { 0x3107, 0x22 }, { 0x3108, 0x35 } },
      0x0100, 0x01,
      .t21 = { 1350, 25 },
      .t23 = { 1440, 25 } },
    { "sc223a",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x3e } },
      0x0100, 0x01,
      .lines = 1440, .lines_fps = 25,
      .t23 = { 1440, 30 } },
    { "sc2300",  0x30, 2, 0,     { { 0x3107, 0x23 }, { 0x3108, 0x00 } },
      0x0100, 0x01,
      .t21 = { 2250, 25 } },
    { "sc231hai", 0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x6a } },
      0x0100, 0x01 },
    { "sc2310",  0x30, 2, 0,     { { 0x3107, 0x23 }, { 0x3108, 0x11 } },
      0x0100, 0x01,
      .t23 = { 1125, 30 } },
    { "sc2331",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x5c } },
      0x0100, 0x01,
      .t23 = { 1130, 30 } },
    { "sc2332",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x17 } },
      0x0100, 0x01,
      .lines = 1350, .lines_fps = 25 },
    { "sc2336p", 0x30, 2, 0,     { { 0x3107, 0x9b }, { 0x3108, 0x3a } },
      0x0100, 0x01,
      .lines = 2400, .lines_fps = 15 },
    { "sc301iot", 0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x40 } },
      0x0100, 0x01,
      .lines = 1920, .lines_fps = 25 },
    { "sc3235",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x05 } },
      0x0100, 0x01,
      .lines = 1620, .lines_fps = 25 },
    { "sc3332",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x44 } },
      0x0100, 0x01 },
    { "sc3335",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x1a } },
      0x0100, 0x01,
      .lines = 1632, .lines_fps = 25,
      .t23 = { 1620, 25 } },
    /* sc3338 answers as sc3336 does */
    { "sc3336",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x41 } },
      0x0100, 0x01,
      .lines = 1632, .lines_fps = 25 },
    { "sc3338",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x41 } },
      0x0100, 0x01,
      .lines = 1632, .lines_fps = 25 },
    { "sc401ai", 0x30, 2, 0,     { { 0x3107, 0xcd }, { 0x3108, 0x2e } },
      0x0100, 0x01,
      .lines = 1800, .lines_fps = 25 },
    { "sc4336",  0x30, 2, 0,     { { 0x3107, 0xdc }, { 0x3108, 0x42 } },
      0x0100, 0x01,
      .lines = 1500, .lines_fps = 30 },
    { "sc4336p", 0x30, 2, 0,     { { 0x3107, 0x9c }, { 0x3108, 0x42 } },
      0x0100, 0x01,
      .lines = 1500, .lines_fps = 30 },
    { "sc500ai", 0x30, 2, 0,     { { 0x3107, 0xce }, { 0x3108, 0x1f } },
      0x0100, 0x01,
      .lines = 1980, .lines_fps = 25 },
    { "sc5235",  0x30, 2, 0,     { { 0x3107, 0x52 }, { 0x3108, 0x35 } },
      0x0100, 0x01,
      .t23 = { 2666, 15 } },
    { "sc5336",  0x30, 2, 0,     { { 0x3107, 0xce }, { 0x3108, 0x50 } },
      0x0100, 0x01 },
    { "sc8238",  0x30, 2, 0,     { { 0x3107, 0x82 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
    { "sc830ai", 0x30, 2, 0,     { { 0x3107, 0xc1 }, { 0x3108, 0x43 } },
      0x0100, 0x01 },
    { "sc835hai", 0x30, 2, 0,     { { 0x3107, 0xc1 }, { 0x3108, 0x70 } },
      0x0100, 0x01 },
};

/* 75% colour bars, BT.601 limited range: Y, Cb, Cr */
static const uint8_t image_sensor_bars[8][3] = {
    { 180, 128, 128 },          /* white */
    { 162,  44, 142 },          /* yellow */
    { 131, 156,  44 },          /* cyan */
    { 112,  72,  58 },          /* green */
    {  84, 184, 198 },          /* magenta */
    {  65, 100, 212 },          /* red */
    {  35, 212, 114 },          /* blue */
    {  16, 128, 128 },          /* black */
};

#define GC_PAGE_REG     0xfe
#define GC_GLOBAL_REGS  0xf0    /* 0xf0-0xff are on every page */

/* The page select is on every page, as GalaxyCore's 0xf0-0xff are */
static uint16_t image_sensor_index(ImageSensorState *s, uint16_t reg)
{
    uint8_t page_reg = s->model->page_reg;

    if (!page_reg || reg == page_reg ||
        (page_reg == GC_PAGE_REG && reg >= GC_GLOBAL_REGS)) {
        return reg;
    }
    return s->page << 8 | reg;
}

static void image_sensor_advance(ImageSensorState *s)
{
    s->pointer++;
    if (s->model->reg_bytes == 1) {
        s->pointer &= 0xff;
    }
}

static int image_sensor_event(I2CSlave *i2c, enum i2c_event event)
{
    ImageSensorState *s = IMAGE_SENSOR(i2c);

    if (event == I2C_START_SEND) {
        s->addr_bytes = 0;
    }
    return 0;
}

static int image_sensor_send(I2CSlave *i2c, uint8_t data)
{
    ImageSensorState *s = IMAGE_SENSOR(i2c);
    bool streaming = image_sensor_streaming(s);
    uint16_t reg;

    if (s->addr_bytes < s->model->reg_bytes) {
        s->pointer = s->addr_bytes++ ? s->pointer << 8 | data : data;
        return 0;
    }
    if (s->model->page_reg && s->pointer == s->model->page_reg) {
        s->page = data & 7;
    }
    reg = image_sensor_index(s, s->pointer);
    if (s->model->wdr && reg == s->model->wdr->reset &&
        data == s->model->wdr->reset_val) {
        s->regs[s->model->wdr->mode] = 0;
    }
    s->regs[reg] = data;
    trace_image_sensor_write(s->input, reg, data);
    if (image_sensor_streaming(s) != streaming) {
        trace_image_sensor_stream(s->input, !streaming);
    }
    image_sensor_advance(s);
    return 0;
}

static uint8_t image_sensor_recv(I2CSlave *i2c)
{
    ImageSensorState *s = IMAGE_SENSOR(i2c);
    uint16_t reg = image_sensor_index(s, s->pointer);
    uint8_t v = s->regs[reg];

    trace_image_sensor_read(s->input, reg, v);
    image_sensor_advance(s);
    return v;
}

bool image_sensor_streaming(ImageSensorState *s)
{
    const ImageSensorModel *m = s->model;

    if (!m->stream_mask) {
        return true;
    }
    return !(s->regs[m->stream_reg] & m->stream_mask) == m->stream_standby;
}

static uint32_t image_sensor_lines(ImageSensorState *s, const uint16_t *it)
{
    return (s->regs[it[0]] & 0x3f) << 8 | s->regs[it[1]];
}

uint32_t image_sensor_line_rate(ImageSensorState *s, ImageSensorRawPart part)
{
    const ImageSensorModel *m = s->model;
    uint32_t lines = m ? m->lines : 0, fps = m ? m->lines_fps : 0;

    if (m && part == IMAGE_SENSOR_PART_T21 && m->t21.lines) {
        lines = m->t21.lines;
        fps = m->t21.fps;
    } else if (m && part == IMAGE_SENSOR_PART_T23 && m->t23.lines) {
        lines = m->t23.lines;
        fps = m->t23.fps;
    } else if (m && part == IMAGE_SENSOR_PART_APICAL && m->apical.lines) {
        lines = m->apical.lines;
        fps = m->apical.fps;
    } else if (m && part == IMAGE_SENSOR_PART_T40 && m->t40.lines) {
        lines = m->t40.lines;
        fps = m->t40.fps;
    } else if (m && part == IMAGE_SENSOR_PART_T41 && m->t41.lines) {
        lines = m->t41.lines;
        fps = m->t41.fps;
    }
    return lines * fps;
}

bool image_sensor_wdr(ImageSensorState *s)
{
    const ImageSensorWdr *w = s->model->wdr;

    return w && s->regs[w->mode] == w->on &&
           image_sensor_lines(s, w->it_short);
}

/* The long exposure over the short, at least 1 */
static double image_sensor_wdr_ratio(ImageSensorState *s)
{
    const ImageSensorWdr *w = s->model->wdr;

    return MAX((double)image_sensor_lines(s, w->it_long) /
               image_sensor_lines(s, w->it_short), 1.0);
}

/* on I2C buses, which take no hotplug: the set is fixed once realized */
static ImageSensorState *image_sensor_inputs[IMAGE_SENSOR_INPUTS];

ImageSensorState *image_sensor_find(unsigned input)
{
    return input < IMAGE_SENSOR_INPUTS ? image_sensor_inputs[input] : NULL;
}

/* The exposure against the reference, 1.0 showing the scene as rendered */
static double image_sensor_exposure(ImageSensorState *s)
{
    const ImageSensorExposure *e = s->model->exposure;
    uint64_t it = 0;
    uint32_t lines, gain;

    if (!e) {
        return 1.0;
    }
    gain = e->gain(s);
    for (int i = 0; i < ARRAY_SIZE(e->it); i++) {
        uint64_t field = s->regs[e->it[i].reg] & e->it[i].mask;

        it += field << e->it[i].shift;
    }
    if (e->vts[0]) {
        uint64_t vts = s->regs[e->vts[0]] << 8 | s->regs[e->vts[1]];

        it = vts > it ? vts - it : 1;
    }
    lines = it * 16 >> e->frac_bits;
    if (lines != s->traced_lines || gain != s->traced_gain) {
        s->traced_lines = lines;
        s->traced_gain = gain;
        trace_image_sensor_exposure(s->input, lines, gain);
    }
    return (double)it / (1 << e->frac_bits) * gain / 1024 / e->ref;
}

/* The light at frame @frame, a ramp's way along, in percent */
static uint32_t image_sensor_light_at(ImageSensorState *s, uint64_t frame)
{
    uint64_t span = (uint64_t)s->light_secs * s->fps;
    uint64_t gone;

    if (!s->ramping) {
        return s->light;
    }
    gone = frame > s->ramp_frame ? frame - s->ramp_frame : 0;
    if (gone >= span) {
        s->ramping = false;
        s->light = s->light_to;
        return s->light;
    }
    return s->ramp_from + ((int64_t)s->light_to - s->ramp_from) *
           (int64_t)gone / (int64_t)span;
}

/* Whether GPIO @pin is an output driving @level */
static bool image_sensor_pin_drives(ImageSensorState *s, int pin, bool level)
{
    char port = 'a' + pin / 32;
    g_autofree char *intr = g_strdup_printf("int-p%c", port);
    g_autofree char *msk = g_strdup_printf("msk-p%c", port);
    g_autofree char *pat1 = g_strdup_printf("pat1-p%c", port);
    g_autofree char *pat0 = g_strdup_printf("pat0-p%c", port);
    uint32_t bit = BIT(pin % 32);
    uint32_t out = ~object_property_get_uint(s->gpio, intr, NULL) &
                   object_property_get_uint(s->gpio, msk, NULL) &
                   ~object_property_get_uint(s->gpio, pat1, NULL);
    uint32_t hi = object_property_get_uint(s->gpio, pat0, NULL);

    return (out & bit) && !(hi & bit) == !level;
}

/* The IR LEDs' power, 0 to 1 each, summed */
static double image_sensor_irleds(ImageSensorState *s)
{
    double sum = 0;

    for (unsigned i = 0; i < s->irleds; i++) {
        int pin = s->irled_pin[i];
        bool low = s->irled_low[i];

        if (pin >= 0) {
            sum += image_sensor_pin_drives(s, pin, !low);
        } else if (s->pwm) {
            g_autofree char *per = g_strdup_printf("pwm%d-period-ns", -1 - pin);
            g_autofree char *hi = g_strdup_printf("pwm%d-high-ns", -1 - pin);
            g_autofree char *lvl = g_strdup_printf("pwm%d-level", -1 - pin);
            uint64_t period = object_property_get_uint(s->pwm, per, NULL);
            double duty = period ? (double)object_property_get_uint(
                                       s->pwm, hi, NULL) / period :
                          object_property_get_uint(s->pwm, lvl, NULL);

            sum += low ? 1 - duty : duty;
        }
    }
    return sum;
}

/*
 * The visible light and the IR the sensor takes in at frame @frame, as
 * fractions of the reference room's light through the exposure
 */
static void image_sensor_light(ImageSensorState *s, uint64_t frame,
                               double *vis, double *ir)
{
    double e = image_sensor_exposure(s);
    double light = image_sensor_light_at(s, frame) / 100.0;
    double leds;

    if (s->ircut_pins == 1) {
        int pin = s->ircut_pin[0];

        if (image_sensor_pin_drives(s, pin, !s->ircut_low[0])) {
            s->filter_in = true;
        } else if (image_sensor_pin_drives(s, pin, s->ircut_low[0])) {
            s->filter_in = false;
        }
    }
    leds = image_sensor_irleds(s);
    if (s->filter_in != s->traced_filter ||
        lround(leds * 100) != s->traced_leds ||
        lround(light * 100) != s->traced_light) {
        s->traced_filter = s->filter_in;
        s->traced_leds = lround(leds * 100);
        s->traced_light = lround(light * 100);
        trace_image_sensor_ir(s->input, s->filter_in, s->traced_leds,
                              s->traced_light);
    }
    *vis = e * light;
    *ir = s->filter_in ? 0 : e * (light * s->ambient_ir / 100.0 +
                                  leds * s->ir / 100.0);
}

/*
 * Rendered luma is gamma-encoded (2.2 here): the light is scaled before
 * it, and the chroma with the luma, up to the colours as rendered, the
 * share IR has of it taking the colour out; as tables for luma and chroma
 */
static void image_sensor_expose(double vis, double ir, double fuse,
                                uint8_t *lut, uint8_t *clut)
{
    double f = vis + ir;
    double c = f > 0 ? MIN(pow(f, 1 / 2.2), 1.0) * vis / f : 0;
    /* the brightest the fused exposures hold, put out at full scale */
    double k = MAX(MIN(f, fuse), 1.0);

    for (int y = 0; y < 256; y++) {
        double n = MIN(MAX((y - 16) / 219.0, 0.0), 1.0);
        double l = MIN(pow(n, 2.2) * f, fuse ? fuse : 1.0);

        l = fuse ? l / (1 + l * (k - 1) / k) : l;
        lut[y] = y < 16 ? y : 16 + lround(pow(l, 1 / 2.2) * 219);
        clut[y] = 128 + lround((y - 128) * c);
    }
}

/* A colour temperature's R, G and B against 6500K, white's luma kept */
static void image_sensor_tint(uint32_t kelvin, double *gain)
{
    double rgb[2][3];

    for (int k = 0; k < 2; k++) {
        double t = (k ? 6500 : MIN(MAX(kelvin, 1000), 40000)) / 100.0;

        rgb[k][0] = t <= 66 ? 255 : 329.698727446 * pow(t - 60, -0.1332047592);
        rgb[k][1] = t <= 66 ? 99.4708025861 * log(t) - 161.1195681661 :
                    288.1221695283 * pow(t - 60, -0.0755148492);
        rgb[k][2] = t >= 66 ? 255 : t <= 19 ? 0 :
                    138.5177312231 * log(t - 10) - 305.0447927307;
    }
    for (int c = 0; c < 3; c++) {
        gain[c] = MAX(rgb[0][c], 0) / rgb[1][c];
    }
    double y = 0.299 * gain[0] + 0.587 * gain[1] + 0.114 * gain[2];

    for (int c = 0; c < 3; c++) {
        gain[c] /= y;
    }
}

/* A table's value at @kelvin: linear in mired, held beyond the ends */
static double image_sensor_raw_at(const uint16_t *v, uint32_t kelvin)
{
    const uint16_t *k = image_sensor_raw_kelvin;
    int n = IMAGE_SENSOR_RAW_POINTS;

    if (kelvin <= k[0]) {
        return v[0];
    }
    for (int i = 1; i < n; i++) {
        if (kelvin <= k[i]) {
            double m = 1e6 / kelvin, m0 = 1e6 / k[i - 1], m1 = 1e6 / k[i];

            return v[i - 1] + (v[i] - v[i - 1]) * (m0 - m) / (m0 - m1);
        }
    }
    return v[n - 1];
}

/* The table a model has on @part, if any */
static const ImageSensorRaw *image_sensor_raw_of(const ImageSensorState *s,
                                                 ImageSensorRawPart part)
{
    for (int i = 0; i < ARRAY_SIZE(image_sensor_raw_models); i++) {
        if (image_sensor_raw_models[i].part == part &&
            !strcmp(s->model->name, image_sensor_raw_models[i].model)) {
            return image_sensor_raw_models[i].raw;
        }
    }
    return NULL;
}

/* A grey's raw R/G and B/G in the scene's light; false: no table or override */
static bool image_sensor_raw_rg_bg(ImageSensorState *s,
                                   ImageSensorRawPart part, double r[2])
{
    const ImageSensorRaw *raw = image_sensor_raw_of(s, part);
    uint32_t kelvin = s->kelvin ?: 6500;

    if (!raw && !s->raw_r && !s->raw_b) {
        return false;
    }
    r[0] = s->raw_r ? s->raw_r / 256.0 :
           raw ? image_sensor_raw_at(raw->r, kelvin) / 1024 :
           (part == IMAGE_SENSOR_PART_APICAL ? IMAGE_SENSOR_RAW_APICAL_R :
            IMAGE_SENSOR_RAW_R) / 256.0;
    r[1] = s->raw_b ? s->raw_b / 256.0 :
           raw ? image_sensor_raw_at(raw->b, kelvin) / 1024 :
           (part == IMAGE_SENSOR_PART_APICAL ? IMAGE_SENSOR_RAW_APICAL_B :
            IMAGE_SENSOR_RAW_B) / 256.0;
    return true;
}

/*
 * The raw R/G and B/G of a grey are the sensor's for the scene's light, as
 * the tuning file of the part the guest runs has them; the picture shows it
 * tinted by image_sensor_tint(), whose linear-light ratio (the picture's
 * gamma is 2.2) the factors take out.
 */
bool image_sensor_raw_factors(ImageSensorState *s, ImageSensorRawPart part,
                              double f[3])
{
    double r[2], g[3];

    if (!image_sensor_raw_rg_bg(s, part, r)) {
        return false;
    }
    image_sensor_tint(s->kelvin ?: 6500, g);
    f[0] = r[0] / pow(g[0] / g[1], 2.2);
    f[1] = 1;
    f[2] = r[1] / pow(g[2] / g[1], 2.2);
    return true;
}

/* The tint of green in linear light is what the picture's G carries */
bool image_sensor_raw_response(ImageSensorState *s, ImageSensorRawPart part,
                               double f[3])
{
    double r[2], g[3], gg;

    if (!image_sensor_raw_rg_bg(s, part, r)) {
        return false;
    }
    image_sensor_tint(s->kelvin ?: 6500, g);
    gg = pow(g[1], 2.2);
    f[0] = r[0] * gg;
    f[1] = gg;
    f[2] = r[1] * gg;
    return true;
}

/*
 * A T41NQ's GC4023 (Bayer samples of 16 bits, the sensor's 10 at the top,
 * levels above its black): from 10 raw frames of a still scene at each gain
 * and three exposure times, the temporal variance of each sample against its
 * mean level. Read: the variance of a dark frame (a 2 line exposure), less
 * the 1/12 code of the 10-bit rounding the ISP adds back. Shot: the slope of
 * variance over level, from the bins with levels of 1000 to 45000; at 1x and
 * 4x the law holds within 2 to 5 % (rms 1 to 2 %), at 11x within 25 % (rms
 * 12 to 17 %), the sensor's own curve there bending towards a higher floor
 * and a lower slope. The four Bayer channels' laws differ by less than 3 %
 * in the slope; the read variances are their mean. The gains are the
 * register's (1024 is 1x): 4096 and 11472 on the sensor, which showed the
 * scene 3.9 and 13.4 times as bright.
 */
static const struct {
    double gain, read, shot;
} image_sensor_noise_nodes[] = {
    { 1.0, 696, 13.64 }, { 4.0, 2862, 50.51 }, { 11.203, 44335, 137.3 },
};

/* The variance of a sample at @level: read + shot * level */
static double image_sensor_noise_var(const ImageSensorState *s, double gain,
                                     double level)
{
    const int n = ARRAY_SIZE(image_sensor_noise_nodes);
    double read, shot, lg = log2(gain);
    int i = 0;

    /*
     * Between the nodes by the log of the gain; past them the read
     * variance goes with the gain squared and the shot noise with the gain
     */
    while (i < n - 2 && gain > image_sensor_noise_nodes[i + 1].gain) {
        i++;
    }
    if (gain < image_sensor_noise_nodes[0].gain) {
        read = image_sensor_noise_nodes[0].read *
               pow(gain / image_sensor_noise_nodes[0].gain, 2);
        shot = image_sensor_noise_nodes[0].shot *
               gain / image_sensor_noise_nodes[0].gain;
    } else if (gain > image_sensor_noise_nodes[n - 1].gain) {
        read = image_sensor_noise_nodes[n - 1].read *
               pow(gain / image_sensor_noise_nodes[n - 1].gain, 2);
        shot = image_sensor_noise_nodes[n - 1].shot *
               gain / image_sensor_noise_nodes[n - 1].gain;
    } else {
        double l0 = log2(image_sensor_noise_nodes[i].gain);
        double l1 = log2(image_sensor_noise_nodes[i + 1].gain);
        double f = (lg - l0) / (l1 - l0);

        read = image_sensor_noise_nodes[i].read +
               (image_sensor_noise_nodes[i + 1].read -
                image_sensor_noise_nodes[i].read) * f;
        shot = image_sensor_noise_nodes[i].shot +
               (image_sensor_noise_nodes[i + 1].shot -
                image_sensor_noise_nodes[i].shot) * f;
    }
    if (s->read_noise > 0) {
        read = s->read_noise * s->read_noise * gain * gain;
    }
    if (s->shot_noise > 0) {
        shot = s->shot_noise * gain;
    }
    return read + shot * level;
}

/* The analog gain the guest has programmed, 1.0 where the model has none */
static double image_sensor_analog_gain(ImageSensorState *s)
{
    const ImageSensorExposure *e = s->model->exposure;

    return e ? e->gain(s) / 1024.0 : 1.0;
}

static uint64_t image_sensor_mix(uint64_t x)
{
    x = (x ^ x >> 30) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ x >> 27) * 0x94d049bb133111ebull;
    return x ^ x >> 31;
}

/*
 * Two normal variates, N(0, 1), for the pair @i of frame @frame: the same
 * for the YUV and the raw render of a frame, so that a run repeats
 */
static void image_sensor_gauss(const ImageSensorState *s, uint64_t frame,
                               uint64_t i, double z[2])
{
    uint64_t h = image_sensor_mix(image_sensor_mix(frame * 4 + s->input) +
                                  i * 0x9e3779b97f4a7c15ull);
    double u = ((h >> 32) + 0.5) / 4294967296.0;
    double r = sqrt(-2 * log(u));
    double a = 2 * M_PI * (((h & 0xffffffffu) + 0.5) / 4294967296.0);

    z[0] = r * cos(a);
    z[1] = r * sin(a);
}

/* The sensor's noise on the picture's luma, in luma levels */
static void image_sensor_noise_luma(ImageSensorState *s, uint64_t frame,
                                    uint32_t w, uint32_t h, uint8_t *luma)
{
    double gain = image_sensor_analog_gain(s);

    if (gain <= 0) {
        return;
    }
    for (size_t i = 0; i < (size_t)w * h; i += 2) {
        double z[2];

        image_sensor_gauss(s, frame, i / 2, z);
        for (int k = 0; k < 2 && i + k < (size_t)w * h; k++) {
            /* a luma level is 1.164 * 257 of a 16-bit sample */
            double level = MAX(luma[i + k] - 16, 0) * 1.164 * 257;
            double sd = sqrt(image_sensor_noise_var(s, gain, level));

            luma[i + k] = MIN(MAX(lround(luma[i + k] + z[k] * sd /
                                         (1.164 * 257)), 0), 255);
        }
    }
}

static uint8_t image_sensor_clamp(double v)
{
    return MIN(MAX(lround(v), 0), 255);
}

/* A template row (@w luma, then @w of interleaved Cb, Cr) in @kelvin's light */
static void image_sensor_tint_row(uint8_t *row, uint32_t w, const double *g)
{
    for (uint32_t x = 0; x + 1 < w; x += 2) {
        double cb = row[w + x] - 128, cr = row[w + x + 1] - 128;
        double ucb = 0, ucr = 0;

        for (int i = 0; i < 2; i++) {
            double y = 1.164 * (row[x + i] - 16);
            double r = (y + 1.596 * cr) * g[0];
            double gg = (y - 0.392 * cb - 0.813 * cr) * g[1];
            double b = (y + 2.017 * cb) * g[2];

            row[x + i] = image_sensor_clamp(16 + 0.257 * r + 0.504 * gg +
                                            0.098 * b);
            ucb += -0.148 * r - 0.291 * gg + 0.439 * b;
            ucr += 0.439 * r - 0.368 * gg - 0.071 * b;
        }
        row[w + x] = image_sensor_clamp(128 + ucb / 2);
        row[w + x + 1] = image_sensor_clamp(128 + ucr / 2);
    }
}

/* The frame number's cells over the picture, top left (see the header) */
static void image_sensor_stamp(ImageSensorState *s, uint64_t frame, uint32_t w,
                               uint32_t h, uint8_t *luma, uint8_t *uv)
{
    uint32_t cs = MAX(w / 64, 2) & ~1;
    uint32_t bits = (s->input & 3) << 24 | (frame & 0xffffff);
    uint8_t cell[32] = { 1, 0 };

    for (int i = 0; i < 26; i++) {
        cell[2 + i] = extract32(bits, 25 - i, 1);
    }
    cell[28] = ctpop32(bits) & 1;
    cell[29] = 1;
    cell[31] = 1;
    for (uint32_t y = 0; y < MIN(cs, h); y++) {
        for (uint32_t x = 0; x < MIN(32 * cs, w); x++) {
            luma[y * w + x] = cell[x / cs] ? 235 : 16;
        }
    }
    for (uint32_t y = 0; y < MIN(cs, h) / 2; y++) {
        memset(uv + y * w, 128, MIN(32 * cs, w));
    }
}

static void image_sensor_map(uint8_t *p, size_t n, const uint8_t *lut)
{
    for (size_t i = 0; i < n; i++) {
        p[i] = lut[p[i]];
    }
}

/*
 * Bars over a luma ramp, crossed by a white stripe every two seconds so
 * that motion shows in what is encoded
 */
void image_sensor_render_exposure(ImageSensorState *s, uint64_t frame,
                                  uint32_t w, uint32_t h, uint8_t *nv12,
                                  ImageSensorExposureSel exposure)
{
    s->exposure = exposure;
    image_sensor_render(s, frame, w, h, nv12);
    s->exposure = IMAGE_SENSOR_FUSED;
}

bool image_sensor_tinted(const ImageSensorState *s)
{
    return s->kelvin && s->kelvin != 6500;
}

void image_sensor_render_untinted(ImageSensorState *s, uint64_t frame,
                                  uint32_t w, uint32_t h, uint8_t *nv12,
                                  ImageSensorExposureSel exposure)
{
    s->untinted = true;
    image_sensor_render_exposure(s, frame, w, h, nv12, exposure);
    s->untinted = false;
}

void image_sensor_render(ImageSensorState *s, uint64_t frame, uint32_t w,
                         uint32_t h, uint8_t *nv12)
{
    image_sensor_render_planes(s, frame, w, h, nv12, nv12 + w * h);
}

/*
 * The scene from a video: a YUV4MPEG2 file (a loop, frame n of the guest
 * showing the file's n modulo its frames) or a FIFO (the latest frame
 * written to it). Frames are 4:2:0 planar, one "FRAME\n" ahead of each.
 */
enum {
    SCENE_HEADER,       /* the stream's first line */
    SCENE_FRAME_LINE,   /* "FRAME" and its parameters */
    SCENE_BODY,
    SCENE_HUNT,         /* out of step: looking for a header */
};

#define SCENE_MAGIC "YUV4MPEG2 "
#define SCENE_MAX_DIM 16384

static bool image_sensor_scene_header(const char *line, uint32_t *w,
                                      uint32_t *h, Error **errp)
{
    g_auto(GStrv) tok = g_strsplit(line, " ", -1);
    uint64_t v;

    *w = *h = 0;
    if (strcmp(tok[0], "YUV4MPEG2")) {
        error_setg(errp, "not a YUV4MPEG2 stream");
        return false;
    }
    for (int i = 1; tok[i]; i++) {
        switch (tok[i][0]) {
        case 'W':
        case 'H':
            if (qemu_strtou64(tok[i] + 1, NULL, 10, &v) || !v ||
                v > SCENE_MAX_DIM) {
                error_setg(errp, "bad size '%s'", tok[i]);
                return false;
            }
            *(tok[i][0] == 'W' ? w : h) = v;
            break;
        case 'C':
            if (strcmp(tok[i] + 1, "420") && strcmp(tok[i] + 1, "420jpeg") &&
                strcmp(tok[i] + 1, "420mpeg2") &&
                strcmp(tok[i] + 1, "420paldv")) {
                error_setg(errp, "'%s' is not 4:2:0 (ffmpeg -pix_fmt yuv420p)",
                           tok[i]);
                return false;
            }
            break;
        }
    }
    if (!*w || !*h) {
        error_setg(errp, "no frame size in the header");
        return false;
    }
    return true;
}

static void image_sensor_scene_size(ImageSensorState *s, uint32_t w,
                                    uint32_t h)
{
    s->scene_w = w;
    s->scene_h = h;
    s->scene_fsize = (size_t)w * h + 2 * (size_t)((w + 1) / 2) * ((h + 1) / 2);
}

#ifndef _WIN32
static void *image_sensor_scene_thread(void *opaque);
#endif

/* @len bytes of a regular file at @off: false if it holds fewer */
static bool image_sensor_scene_read_at(int fd, void *buf, size_t len,
                                       off_t off)
{
    uint8_t *p = buf;

    if (lseek(fd, off, SEEK_SET) != off) {
        return false;
    }
    while (len) {
        ssize_t n = read(fd, p, len);

        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            return false;
        }
        p += n;
        len -= n;
    }
    return true;
}

static void image_sensor_scene_open(ImageSensorState *s, Error **errp)
{
    Error *local = NULL;
    uint32_t w, h;
    struct stat st;
    char line[256];
    ssize_t n;
    char *nl;

#ifdef _WIN32
    s->scene_fd = qemu_open(s->scene, O_RDONLY | O_BINARY, errp);
#else
    s->scene_fd = qemu_open(s->scene, O_RDONLY | O_NONBLOCK, errp);
#endif
    if (s->scene_fd < 0) {
        return;
    }
    if (fstat(s->scene_fd, &st) < 0) {
        error_setg_errno(errp, errno, "image-sensor: scene '%s'", s->scene);
        return;
    }
    s->scene_stream = !S_ISREG(st.st_mode);
    s->scene_state = SCENE_HEADER;
    if (s->scene_stream) {
#ifdef _WIN32
        error_setg(errp, "image-sensor: scene '%s': only a file here",
                   s->scene);
#else
        qemu_mutex_init(&s->scene_lock);
        s->scene_thread_up = true;
        qemu_thread_create(&s->scene_thread, "image-sensor-scene",
                           image_sensor_scene_thread, s, QEMU_THREAD_JOINABLE);
#endif
        return;
    }
    n = lseek(s->scene_fd, 0, SEEK_SET) ? -1 :
        read(s->scene_fd, line, sizeof(line) - 1);
    nl = n > 0 ? memchr(line, '\n', n) : NULL;
    if (!nl) {
        error_setg(errp, "image-sensor: scene '%s': not a YUV4MPEG2 file",
                   s->scene);
        return;
    }
    *nl = 0;
    if (!image_sensor_scene_header(line, &w, &h, &local)) {
        error_setg(errp, "image-sensor: scene '%s': %s", s->scene,
                   error_get_pretty(local));
        error_free(local);
        return;
    }
    image_sensor_scene_size(s, w, h);
    s->scene_off = nl + 1 - line;
    s->scene_frames = (st.st_size - s->scene_off) / (6 + s->scene_fsize);
    if (!s->scene_frames) {
        error_setg(errp, "image-sensor: scene '%s': no complete %ux%u frame",
                   s->scene, w, h);
        return;
    }
    s->scene_cur = g_malloc(s->scene_fsize);
    s->scene_cur_idx = UINT64_MAX;
}

/* The file's frame @idx into scene_cur; false: keep what it holds */
static bool image_sensor_scene_load(ImageSensorState *s, uint64_t idx)
{
    off_t at = s->scene_off + idx * (6 + s->scene_fsize);
    char mark[6];

    if (!image_sensor_scene_read_at(s->scene_fd, mark, 6, at) ||
        memcmp(mark, "FRAME\n", 6) ||
        !image_sensor_scene_read_at(s->scene_fd, s->scene_cur, s->scene_fsize,
                                    at + 6)) {
        return false;
    }
    s->scene_cur_idx = idx;
    s->scene_serial++;
    s->scene_cur_w = s->scene_w;
    s->scene_cur_h = s->scene_h;
    return true;
}

#ifndef _WIN32
/* A frame complete: it is the one the guest's next frame takes up */
static void image_sensor_scene_publish(ImageSensorState *s)
{
    uint8_t *t;
    bool fits;

    qemu_mutex_lock(&s->scene_lock);
    t = s->scene_ready;
    fits = s->scene_ready_w == s->scene_w && s->scene_ready_h == s->scene_h;
    s->scene_ready = s->scene_acc;
    s->scene_ready_w = s->scene_w;
    s->scene_ready_h = s->scene_h;
    s->scene_ready_new = true;
    qemu_mutex_unlock(&s->scene_lock);
    if (!fits) {
        g_free(t);
        t = NULL;
    }
    s->scene_acc = t ? t : g_malloc(s->scene_fsize);
}

/*
 * Take in what a FIFO holds, in the reader thread, so that the guest never
 * waits and a writer is never held up by a pipe's few KB: a header, then
 * frames, the latest complete one kept. A writer that goes (end of file)
 * starts the next at a header; data out of step is skipped to the next
 * header.
 */
static void image_sensor_scene_pump(ImageSensorState *s)
{
    size_t hunt = 0;

    for (;;) {
        ssize_t n;
        char c;

        if (s->scene_state == SCENE_BODY) {
            n = read(s->scene_fd, s->scene_acc + s->scene_fill,
                     s->scene_fsize - s->scene_fill);
        } else {
            n = read(s->scene_fd, &c, 1);
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0) {
            return;             /* EAGAIN: no more for now */
        }
        if (n == 0) {
            s->scene_state = SCENE_HEADER;
            s->scene_fill = 0;
            return;
        }
        switch (s->scene_state) {
        case SCENE_BODY:
            s->scene_fill += n;
            if (s->scene_fill == s->scene_fsize) {
                image_sensor_scene_publish(s);
                s->scene_state = SCENE_FRAME_LINE;
                s->scene_fill = 0;
            }
            break;
        case SCENE_HUNT:
            hunt = c == SCENE_MAGIC[hunt] ? hunt + 1 : c == 'Y';
            if (hunt == strlen(SCENE_MAGIC)) {
                memcpy(s->scene_line, SCENE_MAGIC, hunt);
                s->scene_fill = hunt;
                s->scene_state = SCENE_HEADER;
            }
            break;
        default:
            if (c != '\n') {
                if (s->scene_fill < sizeof(s->scene_line) - 1) {
                    s->scene_line[s->scene_fill++] = c;
                    break;
                }
                s->scene_state = SCENE_HUNT;
                s->scene_fill = hunt = 0;
                break;
            }
            s->scene_line[s->scene_fill] = 0;
            s->scene_fill = 0;
            if (s->scene_state == SCENE_FRAME_LINE &&
                g_str_has_prefix(s->scene_line, "FRAME")) {
                s->scene_state = s->scene_acc ? SCENE_BODY : SCENE_HUNT;
            } else if (g_str_has_prefix(s->scene_line, SCENE_MAGIC)) {
                uint32_t w, h;

                if (image_sensor_scene_header(s->scene_line, &w, &h, NULL)) {
                    if (w != s->scene_w || h != s->scene_h || !s->scene_acc) {
                        image_sensor_scene_size(s, w, h);
                        g_free(s->scene_acc);
                        s->scene_acc = g_malloc(s->scene_fsize);
                    }
                    s->scene_state = SCENE_FRAME_LINE;
                } else {
                    s->scene_state = SCENE_HUNT;
                }
            } else {
                s->scene_state = SCENE_HUNT;
            }
            hunt = 0;
            break;
        }
    }
}

static void *image_sensor_scene_thread(void *opaque)
{
    ImageSensorState *s = opaque;

    while (!qatomic_read(&s->scene_quit)) {
        struct pollfd pfd = { .fd = s->scene_fd, .events = POLLIN };

        if (poll(&pfd, 1, 50) > 0) {
            image_sensor_scene_pump(s);
            if (pfd.revents & POLLHUP) {
                g_usleep(20000);    /* no writer: HUP until one comes */
            }
        }
    }
    return NULL;
}
#endif

static void image_sensor_scene_close(ImageSensorState *s)
{
    if (s->scene_thread_up) {
        qatomic_set(&s->scene_quit, true);
        qemu_thread_join(&s->scene_thread);
        qemu_mutex_destroy(&s->scene_lock);
        s->scene_thread_up = false;
    }
    if (s->scene_fd >= 0) {
        qemu_close(s->scene_fd);
        s->scene_fd = -1;
    }
    g_clear_pointer(&s->scene_cur, g_free);
    g_clear_pointer(&s->scene_acc, g_free);
    g_clear_pointer(&s->scene_ready, g_free);
    for (int i = 0; i < IMAGE_SENSOR_SCENE_CACHE; i++) {
        g_clear_pointer(&s->scene_cache[i].buf, g_free);
    }
}

/* The picture the scene gives for guest frame @frame; false: none yet */
static bool image_sensor_scene_pick(ImageSensorState *s, uint64_t frame)
{
    if (s->scene_fd < 0) {
        return false;
    }
    if (!s->scene_stream) {
        uint64_t idx = frame % s->scene_frames;

        return idx == s->scene_cur_idx || image_sensor_scene_load(s, idx) ||
               s->scene_cur_idx != UINT64_MAX;
    }
    /* the pictures of one frame (main, sub, raw) come from one frame */
    if (!s->scene_polled_valid || s->scene_polled != frame) {
        s->scene_polled = frame;
        s->scene_polled_valid = true;
        qemu_mutex_lock(&s->scene_lock);
        if (s->scene_ready_new) {
            uint8_t *t = s->scene_cur;
            uint32_t w = s->scene_cur_w, h = s->scene_cur_h;

            s->scene_cur = s->scene_ready;
            s->scene_cur_w = s->scene_ready_w;
            s->scene_cur_h = s->scene_ready_h;
            s->scene_ready = t;
            s->scene_ready_w = w;
            s->scene_ready_h = h;
            s->scene_ready_new = false;
            s->scene_serial++;
        }
        qemu_mutex_unlock(&s->scene_lock);
    }
    return s->scene_cur && s->scene_cur_w;
}

/*
 * One plane of @sw x @sh scaled to @dw x @dh by averaging the source pixels
 * each lands on (the nearest one when enlarging), written @step bytes apart
 * and @pitch a row, mirrored or flipped as asked
 */
static void image_sensor_scale(const uint8_t *src, uint32_t sw, uint32_t sh,
                               uint8_t *dst, uint32_t dw, uint32_t dh,
                               size_t pitch, unsigned step, bool mirror,
                               bool flip)
{
    g_autofree uint32_t *x0 = NULL;
    g_autofree uint32_t *x1 = NULL;
    g_autofree uint32_t *col = NULL;

    if (sw == dw && sh == dh) {
        for (uint32_t y = 0; y < dh; y++) {
            const uint8_t *row = src + (size_t)(flip ? dh - 1 - y : y) * sw;
            uint8_t *out = dst + y * pitch;

            if (step == 1 && !mirror) {
                memcpy(out, row, dw);
                continue;
            }
            if (mirror) {
                row += dw - 1;
            }
            for (uint32_t x = 0; x < dw; x++, out += step) {
                *out = mirror ? *row-- : *row++;
            }
        }
        return;
    }
    if (sw == 2 * dw && sh == 2 * dh) {
        /* half size, as an ISP's statistics take it */
        for (uint32_t y = 0; y < dh; y++) {
            const uint8_t *r0 = src + (size_t)(flip ? dh - 1 - y : y) * 2 * sw;
            const uint8_t *r1 = r0 + sw;
            uint8_t *out = dst + y * pitch;

            for (uint32_t x = 0; x < dw; x++, out += step) {
                uint32_t i = 2 * (mirror ? dw - 1 - x : x);

                *out = (r0[i] + r0[i + 1] + r1[i] + r1[i + 1] + 2) >> 2;
            }
        }
        return;
    }
    x0 = g_new(uint32_t, dw);
    x1 = g_new(uint32_t, dw);
    col = g_new(uint32_t, sw);
    for (uint32_t x = 0; x < dw; x++) {
        uint64_t m = mirror ? dw - 1 - x : x;

        x0[x] = m * sw / dw;
        x1[x] = MAX((m + 1) * sw / dw, x0[x] + 1);
    }
    for (uint32_t y = 0; y < dh; y++) {
        uint64_t m = flip ? dh - 1 - y : y;
        uint32_t y0 = m * sh / dh, y1 = MAX((m + 1) * sh / dh, y0 + 1);
        uint8_t *out = dst + y * pitch;
        uint32_t last = 0, inv = 0;

        /* the rows' column sums first, a pass the compiler vectorizes */
        memset(col, 0, sw * sizeof(*col));
        for (uint32_t yy = y0; yy < y1; yy++) {
            const uint8_t *row = src + (size_t)yy * sw;

            for (uint32_t xx = 0; xx < sw; xx++) {
                col[xx] += row[xx];
            }
        }
        for (uint32_t x = 0; x < dw; x++) {
            uint32_t n = (x1[x] - x0[x]) * (y1 - y0);
            uint32_t sum = 0;

            for (uint32_t xx = x0[x]; xx < x1[x]; xx++) {
                sum += col[xx];
            }
            if (n != last) {
                last = n;
                inv = (65536 + n / 2) / n;      /* the mean as a multiply */
            }
            out[x * step] = n == 1 ? sum : (sum * inv + 32768) >> 16;
        }
    }
}

/*
 * The scene's picture as NV12 planes @w x @h, mirrored and flipped. The
 * renders of one frame (main, sub, raw) that ask the same size and
 * orientation share one scaling: scene_serial says which source frame a
 * cached picture is of.
 */
static void image_sensor_scene_draw(ImageSensorState *s, uint32_t w,
                                    uint32_t h, uint8_t *luma, uint8_t *uv,
                                    bool mirror, bool flip)
{
    uint32_t sw = s->scene_cur_w, sh = s->scene_cur_h;
    uint32_t cw = (sw + 1) / 2, ch = (sh + 1) / 2;
    const uint8_t *cb = s->scene_cur + (size_t)sw * sh;
    size_t lsize = (size_t)w * h, usize = (size_t)w * (h / 2);
    ImageSensorSceneCache *c = NULL;

    bool copy = sw == w && sh == h;     /* no cheaper out of the cache */

    for (int i = 0; !copy && i < IMAGE_SENSOR_SCENE_CACHE; i++) {
        ImageSensorSceneCache *e = &s->scene_cache[i];

        if (e->buf && e->serial == s->scene_serial && e->w == w &&
            e->h == h && e->mirror == mirror && e->flip == flip) {
            memcpy(luma, e->buf, lsize);
            memcpy(uv, e->buf + lsize, usize);
            return;
        }
    }
    image_sensor_scale(s->scene_cur, sw, sh, luma, w, h, w, 1, mirror, flip);
    if (w >= 2 && h >= 2) {
        image_sensor_scale(cb, cw, ch, uv, w / 2, h / 2, w, 2, mirror, flip);
        image_sensor_scale(cb + (size_t)cw * ch, cw, ch, uv + 1, w / 2, h / 2,
                           w, 2, mirror, flip);
    }
    if (copy) {
        return;
    }
    c = &s->scene_cache[s->scene_cache_next++ % IMAGE_SENSOR_SCENE_CACHE];
    c->buf = g_realloc(c->buf, lsize + usize);
    memcpy(c->buf, luma, lsize);
    memcpy(c->buf + lsize, uv, usize);
    c->serial = s->scene_serial;
    c->w = w;
    c->h = h;
    c->mirror = mirror;
    c->flip = flip;
}

/* The light's tint over the scene's rows, 4:2:0 as the bars' rows are */
static void image_sensor_scene_tint(uint32_t w, uint32_t h, uint8_t *luma,
                                    uint8_t *uv, const double *g)
{
    g_autofree uint8_t *a = g_malloc0(w * 2);
    g_autofree uint8_t *b = g_malloc0(w * 2);

    for (uint32_t y = 0; y < h; y += 2) {
        bool chroma = y / 2 < h / 2, second = y + 1 < h;

        memcpy(a, luma + (size_t)y * w, w);
        memset(a + w, 128, w);
        if (chroma) {
            memcpy(a + w, uv + (size_t)(y / 2) * w, w);
        }
        memcpy(b + w, a + w, w);
        if (second) {
            memcpy(b, luma + (size_t)(y + 1) * w, w);
            image_sensor_tint_row(b, w, g);
            memcpy(luma + (size_t)(y + 1) * w, b, w);
        }
        image_sensor_tint_row(a, w, g);
        memcpy(luma + (size_t)y * w, a, w);
        if (chroma) {
            memcpy(uv + (size_t)(y / 2) * w, a + w, w);
        }
    }
}

/* @sensor_noise: the sensor's noise on the luma, if the property is set */
static void image_sensor_draw(ImageSensorState *s, uint64_t frame, uint32_t w,
                              uint32_t h, uint8_t *luma, uint8_t *uv,
                              bool sensor_noise)
{
    g_autofree uint8_t *bar = g_malloc(w * 2);
    g_autofree uint8_t *ramp = g_malloc(w * 2);
    uint64_t t = s->motion ? frame : 0;
    uint32_t stripe = MAX(w / 64, 2);
    uint32_t x0 = t * w / (2 * s->fps) % w;
    uint32_t bars_h = h * 2 / 3;
    uint8_t lut[256], clut[256];
    double vis, ir, fuse = 0;
    bool mirror = s->input & 1, flip = s->input & 2;
    bool expose;

    s->frame = frame;
    image_sensor_light(s, frame, &vis, &ir);
    if (image_sensor_wdr(s)) {
        double r = image_sensor_wdr_ratio(s);

        if (s->exposure == IMAGE_SENSOR_SHORT) {
            vis /= r;
            ir /= r;
        } else if (s->exposure == IMAGE_SENSOR_FUSED) {
            fuse = r;
        }
    }
    expose = s->model->exposure || vis != 1 || ir || fuse;
    if (!s->grey && image_sensor_scene_pick(s, frame)) {
        image_sensor_scene_draw(s, w, h, luma, uv, mirror, flip);
        if (image_sensor_tinted(s) && !s->untinted) {
            double g[3];

            image_sensor_tint(s->kelvin, g);
            image_sensor_scene_tint(w, h, luma, uv, g);
        }
        if (expose) {
            image_sensor_expose(vis, ir, fuse, lut, clut);
        }
        if (expose && !s->noise) {
            image_sensor_map(luma, (size_t)w * h, lut);
            image_sensor_map(uv, (size_t)w * (h / 2), clut);
        }
        goto overlays;
    }
    for (uint32_t x = 0; x < w; x++) {
        uint32_t sx = mirror ? w - 1 - x : x;
        const uint8_t *c = image_sensor_bars[sx * 8 / w];
        bool lit = sx - x0 < stripe;

        bar[x] = lit ? 235 : c[0];
        ramp[x] = lit ? 235 : 16 + sx * 219 / w;
        bar[w + x] = lit ? 128 : c[1 + (x & 1)];
        ramp[w + x] = 128;
    }
    if (s->grey) {
        memset(bar, MIN(s->grey, 255), w);
        memset(ramp, MIN(s->grey, 255), w);
        memset(bar + w, 128, w);
    }
    if (image_sensor_tinted(s) && !s->untinted) {
        double g[3];

        image_sensor_tint(s->kelvin, g);
        image_sensor_tint_row(bar, w, g);
        image_sensor_tint_row(ramp, w, g);
    }
    if (expose) {
        image_sensor_expose(vis, ir, fuse, lut, clut);
    }
    /* without noise every row is one of these: expose them alone */
    if (expose && !s->noise) {
        image_sensor_map(bar, w, lut);
        image_sensor_map(ramp, w, lut);
        image_sensor_map(bar + w, w, clut);
        image_sensor_map(ramp + w, w, clut);
    }
    for (uint32_t y = 0; y < h; y++) {
        uint32_t sy = flip ? h - 1 - y : y;

        memcpy(luma + y * w, sy < bars_h ? bar : ramp, w);
    }
    for (uint32_t y = 0; y < h / 2; y++) {
        uint32_t sy = flip ? h - 2 - y * 2 : y * 2;

        memcpy(uv + y * w, (sy < bars_h ? bar : ramp) + w, w);
    }
    if (s->object) {
        /* across and back every 4 seconds, in the ramp's middle */
        uint32_t side = MAX(h / 6, 4) & ~1;
        uint64_t span = 2 * s->fps, at = t % (2 * span);
        uint32_t ox = (at < span ? at : 2 * span - at) * (w - side) / span;
        uint32_t oy = (bars_h + (h - bars_h - side) / 2) & ~1;
        bool mapped = expose && !s->noise;

        ox = mirror ? w - side - ox : ox;
        oy = flip ? h - side - oy : oy;
        for (uint32_t y = oy; y < MIN(oy + side, h); y++) {
            memset(luma + y * w + ox, mapped ? lut[235] : 235, side);
        }
        for (uint32_t y = oy / 2; y < MIN((oy + side) / 2, h / 2); y++) {
            memset(uv + y * w + ox, mapped ? clut[128] : 128, side);
        }
    }
overlays:
    if (s->noise) {
        for (uint32_t i = 0; i < w * h; i++) {
            uint32_t r = (i + 1) * 2654435761u ^ (uint32_t)frame * 40503u;

            r ^= r >> 15;
            r *= 2246822519u;
            r ^= r >> 13;
            luma[i] = MIN(MAX(luma[i] + (int)(r & 63) - 32, 0), 255);
        }
        if (expose) {
            image_sensor_map(luma, (size_t)w * h, lut);
            image_sensor_map(uv, (size_t)w * (h / 2), clut);
        }
    }
    if (sensor_noise && s->sensor_noise) {
        image_sensor_noise_luma(s, frame, w, h, luma);
    }
    if (s->stamp) {
        image_sensor_stamp(s, frame, w, h, luma, uv);
    }
}

void image_sensor_render_planes(ImageSensorState *s, uint64_t frame,
                                uint32_t w, uint32_t h, uint8_t *luma,
                                uint8_t *uv)
{
    image_sensor_draw(s, frame, w, h, luma, uv, true);
}

/*
 * The picture's limited-range BT.601 YUV back to RGB, each pixel keeping
 * the colour its place in the Bayer pattern gives it
 */
void image_sensor_render_bayer(ImageSensorState *s, uint64_t frame,
                               uint32_t w, uint32_t h, uint16_t *raw)
{
    uint32_t uv_h = MAX(h / 2, 1);
    g_autofree uint8_t *luma = g_malloc((size_t)w * h);
    g_autofree uint8_t *uv = g_malloc((size_t)w * uv_h);

    double gain = image_sensor_analog_gain(s);
    double z[2] = { 0, 0 };

    memset(uv, 128, (size_t)w * uv_h);
    image_sensor_draw(s, frame, w, h, luma, uv, false);
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *c = uv + (size_t)MIN(y / 2, uv_h - 1) * w;

        for (uint32_t x = 0; x < w; x++) {
            double l = 1.164 * (luma[(size_t)y * w + x] - 16);
            double u = c[x & ~1] - 128;
            double v = (x | 1) < w ? c[x | 1] - 128 : 0;
            double rgb;

            switch ((y & 1) << 1 | (x & 1)) {
            case 0:
                rgb = l + 1.596 * v;
                break;
            case 3:
                rgb = l + 2.017 * u;
                break;
            default:
                rgb = l - 0.392 * u - 0.813 * v;
                break;
            }
            double sample = image_sensor_clamp(rgb) * 257;

            if (s->sensor_noise && gain > 0) {
                size_t i = (size_t)y * w + x;

                if (!(i & 1)) {
                    image_sensor_gauss(s, frame, i / 2, z);
                }
                sample += z[i & 1] * sqrt(image_sensor_noise_var(s, gain,
                                                                 sample));
            }
            raw[(size_t)y * w + x] = MIN(MAX(lround(sample), 0), 65535);
        }
    }
}

/* A latching IR-cut's pins: a pulse on the first puts it in, the second out */
static void image_sensor_ircut_pin(void *opaque, int n, int level)
{
    ImageSensorState *s = opaque;

    if (!level == s->ircut_low[n]) {
        s->filter_in = n == 0;
    }
}

/*
 * thingino.json's pin tokens: a GPIO number, 'o' after it for active low
 * ('O' active high), or pwmN for a PWM channel where @pwm is allowed
 */
static int image_sensor_pins(const char *what, const char *spec, bool pwm,
                             int16_t *pin, bool *low, int max, Error **errp)
{
    g_auto(GStrv) tok = g_strsplit_set(spec, " ,", -1);
    int n = 0;

    for (int i = 0; tok[i]; i++) {
        const char *t = tok[i];
        size_t len = strlen(t);
        uint64_t v;

        if (!len) {
            continue;
        }
        if (n == max) {
            error_setg(errp, "image-sensor: %s takes %d pins at most", what,
                       max);
            return -1;
        }
        low[n] = t[len - 1] == 'o';
        if (pwm && g_str_has_prefix(t, "pwm") &&
            !qemu_strtou64(t + 3, NULL, 10, &v) && v < 16) {
            pin[n++] = -1 - v;
            continue;
        }
        if (t[len - 1] == 'o' || t[len - 1] == 'O') {
            len--;
        }
        g_autofree char *num = g_strndup(t, len);

        if (qemu_strtou64(num, NULL, 10, &v) || v >= 32 * 8) {
            error_setg(errp, "image-sensor: %s: bad pin '%s'", what, t);
            return -1;
        }
        pin[n++] = v;
    }
    return n;
}

static bool image_sensor_wire(ImageSensorState *s, Error **errp)
{
    int n;

    if (!s->ircut && !s->irled) {
        return true;
    }
    s->gpio = object_resolve_path_type("", "ingenic-gpio", NULL);
    s->pwm = object_resolve_path_type("", "ingenic-pwm", NULL);
    if (!s->pwm) {
        s->pwm = object_resolve_path_type("", "ingenic-tcu", NULL);
    }
    if (!s->gpio) {
        error_setg(errp, "image-sensor: ircut and irled need the SoC's GPIO");
        return false;
    }
    if (s->irled) {
        n = image_sensor_pins("irled", s->irled, true, s->irled_pin,
                              s->irled_low, ARRAY_SIZE(s->irled_pin), errp);
        if (n < 0) {
            return false;
        }
        s->irleds = n;
    }
    if (!s->ircut) {
        return true;
    }
    n = image_sensor_pins("ircut", s->ircut, false, s->ircut_pin,
                          s->ircut_low, 2, errp);
    if (n < 0) {
        return false;
    }
    s->ircut_pins = n;
    if (n == 2) {
        qdev_init_gpio_in_named(DEVICE(s), image_sensor_ircut_pin, "ircut", 2);
        for (int i = 0; i < 2; i++) {
            DeviceState *gpio = DEVICE(s->gpio);

            if (qdev_get_gpio_out_connector(gpio, "pad", s->ircut_pin[i])) {
                error_setg(errp, "image-sensor: pin %d is wired already",
                           s->ircut_pin[i]);
                return false;
            }
            qdev_connect_gpio_out_named(gpio, "pad", s->ircut_pin[i],
                qdev_get_gpio_in_named(DEVICE(s), "ircut", i));
        }
    }
    return true;
}

static void image_sensor_realize(DeviceState *dev, Error **errp)
{
    ERRP_GUARD();
    ImageSensorState *s = IMAGE_SENSOR(dev);
    I2CSlave *i2c = I2C_SLAVE(dev);

    for (int i = 0; i < ARRAY_SIZE(image_sensor_models); i++) {
        if (s->model_name && !strcmp(s->model_name,
                                     image_sensor_models[i].name)) {
            s->model = &image_sensor_models[i];
        }
    }
    if (!s->model) {
        g_autoptr(GString) names = g_string_new(NULL);

        for (int i = 0; i < ARRAY_SIZE(image_sensor_models); i++) {
            g_string_append_printf(names, "%s%s", i ? ", " : "",
                                   image_sensor_models[i].name);
        }
        error_setg(errp, "image-sensor: model must be one of %s", names->str);
        return;
    }
    if (!i2c->address) {
        i2c->address = s->model->addr;
    }
    if (!s->fps) {
        error_setg(errp, "image-sensor: fps must be at least 1");
        return;
    }
    if (s->input >= IMAGE_SENSOR_INPUTS) {
        error_setg(errp, "image-sensor: input must be below %d",
                   IMAGE_SENSOR_INPUTS);
        return;
    }
    if (image_sensor_inputs[s->input]) {
        error_setg(errp, "image-sensor: input %u has a sensor already; give "
                   "each its own input=", s->input);
        return;
    }
    if (!image_sensor_wire(s, errp)) {
        return;
    }
    if (s->scene) {
        image_sensor_scene_open(s, errp);
        if (*errp) {
            image_sensor_scene_close(s);
            return;
        }
    }
    image_sensor_inputs[s->input] = s;
}

static void image_sensor_unrealize(DeviceState *dev)
{
    ImageSensorState *s = IMAGE_SENSOR(dev);

    if (image_sensor_inputs[s->input] == s) {
        image_sensor_inputs[s->input] = NULL;
    }
    image_sensor_scene_close(s);
}

static void image_sensor_reset_hold(Object *obj, ResetType type)
{
    ImageSensorState *s = IMAGE_SENSOR(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->filter_in = true;
    s->pointer = 0;
    s->addr_bytes = 0;
    s->page = 0;
    /* an empty entry ends the list: PixelPlus keeps its ID in register 0 */
    for (int i = 0; i < ARRAY_SIZE(s->model->id) &&
                    (s->model->id[i].reg || s->model->id[i].val); i++) {
        s->regs[s->model->id[i].reg] = s->model->id[i].val;
    }
}

static void image_sensor_get_light(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    ImageSensorState *s = IMAGE_SENSOR(obj);
    uint32_t light = image_sensor_light_at(s, s->frame);

    visit_type_uint32(v, name, &light, errp);
}

/* "light" sets it now, "light-to" starts a ramp from where it is */
static void image_sensor_set_light(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    ImageSensorState *s = IMAGE_SENSOR(obj);
    uint32_t val;

    if (!visit_type_uint32(v, name, &val, errp)) {
        return;
    }
    if (!strcmp(name, "light")) {
        s->light = val;
        s->ramping = false;
        return;
    }
    s->ramp_from = image_sensor_light_at(s, s->frame);
    s->ramp_frame = s->frame;
    s->light_to = val;
    s->ramping = true;
}

static void image_sensor_get_flag(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    visit_type_bool(v, name, opaque, errp);
}

static void image_sensor_set_flag(Object *obj, Visitor *v, const char *name,
                                  void *opaque, Error **errp)
{
    visit_type_bool(v, name, opaque, errp);
}

static void image_sensor_get_number(Object *obj, Visitor *v, const char *name,
                                    void *opaque, Error **errp)
{
    visit_type_number(v, name, opaque, errp);
}

static void image_sensor_set_number(Object *obj, Visitor *v, const char *name,
                                    void *opaque, Error **errp)
{
    double val;

    if (visit_type_number(v, name, &val, errp)) {
        if (val < 0) {
            error_setg(errp, "image-sensor: %s must not be negative", name);
            return;
        }
        *(double *)opaque = val;
    }
}

static void image_sensor_init(Object *obj)
{
    ImageSensorState *s = IMAGE_SENSOR(obj);

    s->light = 100;
    s->light_to = 100;
    s->light_secs = 10;
    s->ambient_ir = 50;
    s->ir = 100;
    s->motion = true;
    s->scene_fd = -1;
    object_property_add(obj, "light", "uint32", image_sensor_get_light,
                        image_sensor_set_light, NULL, NULL);
    object_property_add(obj, "light-to", "uint32", NULL,
                        image_sensor_set_light, NULL, NULL);
    object_property_add_uint32_ptr(obj, "light-secs", &s->light_secs,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "kelvin", &s->kelvin,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "raw-r", &s->raw_r,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "raw-b", &s->raw_b,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "grey", &s->grey,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ambient-ir", &s->ambient_ir,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "ir", &s->ir,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add(obj, "object", "bool", image_sensor_get_flag,
                        image_sensor_set_flag, NULL, &s->object);
    object_property_add(obj, "motion", "bool", image_sensor_get_flag,
                        image_sensor_set_flag, NULL, &s->motion);
    object_property_add(obj, "stamp", "bool", image_sensor_get_flag,
                        image_sensor_set_flag, NULL, &s->stamp);
    object_property_add(obj, "read-noise", "number", image_sensor_get_number,
                        image_sensor_set_number, NULL, &s->read_noise);
    object_property_add(obj, "shot-noise", "number", image_sensor_get_number,
                        image_sensor_set_number, NULL, &s->shot_noise);
}

static const VMStateDescription vmstate_image_sensor = {
    .name = TYPE_IMAGE_SENSOR,
    .version_id = 4,
    .minimum_version_id = 3,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, ImageSensorState),
        VMSTATE_UINT8_ARRAY(regs, ImageSensorState, 65536),
        VMSTATE_UINT16(pointer, ImageSensorState),
        VMSTATE_UINT8(addr_bytes, ImageSensorState),
        VMSTATE_UINT8(page, ImageSensorState),
        VMSTATE_UINT32(light, ImageSensorState),
        VMSTATE_UINT32(light_to, ImageSensorState),
        VMSTATE_UINT32(light_secs, ImageSensorState),
        VMSTATE_UINT32(kelvin, ImageSensorState),
        VMSTATE_UINT32_V(raw_r, ImageSensorState, 4),
        VMSTATE_UINT32_V(raw_b, ImageSensorState, 4),
        VMSTATE_UINT32(ambient_ir, ImageSensorState),
        VMSTATE_UINT32(ir, ImageSensorState),
        VMSTATE_BOOL(object, ImageSensorState),
        VMSTATE_BOOL(motion, ImageSensorState),
        VMSTATE_BOOL(stamp, ImageSensorState),
        VMSTATE_UINT64(frame, ImageSensorState),
        VMSTATE_UINT64(ramp_frame, ImageSensorState),
        VMSTATE_UINT32(ramp_from, ImageSensorState),
        VMSTATE_BOOL(ramping, ImageSensorState),
        VMSTATE_BOOL(filter_in, ImageSensorState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property image_sensor_props[] = {
    DEFINE_PROP_STRING("model", ImageSensorState, model_name),
    DEFINE_PROP_UINT32("fps", ImageSensorState, fps, 30),
    DEFINE_PROP_UINT32("input", ImageSensorState, input, 0),
    DEFINE_PROP_BOOL("noise", ImageSensorState, noise, false),
    DEFINE_PROP_BOOL("sensor-noise", ImageSensorState, sensor_noise, false),
    DEFINE_PROP_STRING("ircut", ImageSensorState, ircut),
    DEFINE_PROP_STRING("irled", ImageSensorState, irled),
    DEFINE_PROP_STRING("scene", ImageSensorState, scene),
};

static void image_sensor_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Camera image sensor (I2C control)";
    dc->realize = image_sensor_realize;
    dc->unrealize = image_sensor_unrealize;
    dc->vmsd = &vmstate_image_sensor;
    rc->phases.hold = image_sensor_reset_hold;
    device_class_set_props(dc, image_sensor_props);
    k->event = image_sensor_event;
    k->send = image_sensor_send;
    k->recv = image_sensor_recv;
}

static const TypeInfo image_sensor_type_info = {
    .name = TYPE_IMAGE_SENSOR,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ImageSensorState),
    .instance_init = image_sensor_init,
    .class_init = image_sensor_class_init,
};

static void image_sensor_register_types(void)
{
    type_register_static(&image_sensor_type_info);
}

type_init(image_sensor_register_types)
