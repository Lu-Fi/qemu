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
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/cutils.h"
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
};

static const ImageSensorModel image_sensor_models[] = {
    { "jxh42",   0x30, 1, 0,     { { 0x0a, 0xa0 }, { 0x0b, 0x42 },
                                   { 0x09, 0x81 } },
      0x12, 0x40, true },
    { "jxf22",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x22 } },
      .exposure = &image_sensor_soi_exposure },
    { "sc2232h", 0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x07 },
                                   { 0x3109, 0x01 } },
      0x0100, 0x01 },
    { "sc2336",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x3a } },
      0x0100, 0x01 },
    { "sc4236",  0x30, 2, 0,     { { 0x3107, 0x32 }, { 0x3108, 0x35 } },
      0x0100, 0x01, .exposure = &image_sensor_sc4236_exposure },
    { "gc2053",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } },
      .exposure = &image_sensor_gc2053_exposure },
    /* the T41 driver reads 0x3003 and 0x3002, the T32 one 0x3011 and 0x3138 */
    { "cv2003",  0x35, 2, 0,     { { 0x3002, 0x04 }, { 0x3003, 0x20 },
                                   { 0x3011, 0x20 }, { 0x3138, 0x03 } },
      0x3000, 0x01, true },
    /* 0x3003 is the high byte: the T32, T40 and T41 drivers agree */
    { "cv8001",  0x35, 2, 0,     { { 0x3002, 0x01 }, { 0x3003, 0x80 } },
      0x3000, 0x01, true },
    { "imx307",  0x1a, 2, 0,     { { 0x3008, 0xa0 }, { 0x301e, 0xb2 } },
      0x3000, 0x01, true },
    { "gc4023",  0x29, 2, 0,     { { 0x03f0, 0x40 }, { 0x03f1, 0x23 } } },
    { "gc5613",  0x31, 2, 0,     { { 0x03f0, 0x56 }, { 0x03f1, 0x03 } },
      0x0100, 0x01 },
    /* gc2063 and gc4663 answer as gc2053 and gc4653 do */
    { "gc2023",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x23 } },
      0xf2, 0x0f },
    { "gc2063",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } },
      .exposure = &image_sensor_gc2053_exposure },
    { "gc1084",  0x37, 2, 0,     { { 0x03f0, 0x10 }, { 0x03f1, 0x84 } } },
    /* the drivers stream with 0x023e = 0x99 and stop with 0x00 */
    { "gc2083",  0x37, 2, 0,     { { 0x03f0, 0x20 }, { 0x03f1, 0x83 } },
      0x023e, 0x80 },
    { "gc3003a", 0x37, 2, 0,     { { 0x03f0, 0x30 }, { 0x03f1, 0x03 },
                                   { 0x036a, 0x10 } } },
    { "gc4653",  0x29, 2, 0,     { { 0x03f0, 0x46 }, { 0x03f1, 0x53 } } },
    { "gc4663",  0x29, 2, 0,     { { 0x03f0, 0x46 }, { 0x03f1, 0x53 } } },
    /* gc5603 answers as gc5613 does */
    { "gc5603",  0x31, 2, 0,     { { 0x03f0, 0x56 }, { 0x03f1, 0x03 } } },
    /*
     * 0x3008 reads its reset value, as an imx307 does: older sinfo tables
     * see one
     */
    { "imx327",  0x1a, 2, 0,     { { 0x3008, 0xa0 }, { 0x301e, 0xb2 },
                                   { 0x301f, 0x01 } },
      0x3000, 0x01, true },
    { "jxf23",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x23 } } },
    { "jxf28",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x28 } } },
    { "jxf37",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x37 } } },
    { "jxh62",   0x30, 1, 0,     { { 0x0a, 0xa0 }, { 0x0b, 0x62 } },
      0x12, 0x40, true },
    { "jxh63p",  0x40, 1, 0,     { { 0x0a, 0x08 }, { 0x0b, 0x48 } } },
    { "jxk04",   0x40, 1, 0,     { { 0x0a, 0x04 }, { 0x0b, 0x04 } },
      0x12, 0x40, true },
    { "jxq03",   0x40, 1, 0,     { { 0x0a, 0x05 }, { 0x0b, 0x07 } } },
    { "jxq03p",  0x40, 1, 0,     { { 0x0a, 0x08 }, { 0x0b, 0x43 } } },
    { "mis2008", 0x30, 2, 0,     { { 0x3000, 0x20 }, { 0x3001, 0x08 } } },
    { "mis20c1", 0x30, 2, 0,     { { 0x3000, 0x20 }, { 0x3001, 0xc1 } } },
    /* the T32 drivers check 0x05 as well */
    { "os02b10", 0x3c, 1, 0xfd,  { { 0x02, 0x23 }, { 0x03, 0x08 } } },
    { "os02g10", 0x3c, 1, 0xfd,  { { 0x02, 0x56 }, { 0x03, 0x02 },
                                   { 0x04, 0x47 } },
      .exposure = &image_sensor_os02g10_exposure },
    { "os02n10", 0x3c, 1, 0xfd,  { { 0x02, 0x53 }, { 0x03, 0x02 },
                                   { 0x04, 0x4e }, { 0x05, 0x10 } } },
    { "os03b10", 0x3c, 1, 0xfd,  { { 0x02, 0x53 }, { 0x03, 0x03 },
                                   { 0x04, 0x42 } } },
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
      0x36, 0x01, true },
    { "ov9712",  0x30, 1, 0,     { { 0x0a, 0x97 }, { 0x0b, 0x11 } },
      0x09, 0x10, true },
    { "ov9732",  0x36, 2, 0,     { { 0x300a, 0x97 }, { 0x300b, 0x32 } },
      0x0100, 0x01 },
    { "ov9750",  0x36, 2, 0,     { { 0x300b, 0x97 }, { 0x300c, 0x50 } },
      0x0100, 0x01 },
    { "ps5250",  0x48, 1, 0xef,  { { 0x00, 0x52 }, { 0x01, 0x50 } } },
    { "ps5260",  0x48, 1, 0xef,  { { 0x00, 0x52 }, { 0x01, 0x60 } } },
    { "sc1235",  0x30, 2, 0,     { { 0x3107, 0x12 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
    { "sc1a4t",  0x30, 2, 0,     { { 0x3107, 0xda }, { 0x3108, 0x4d } },
      0x0100, 0x01 },
    { "sc2135",  0x30, 2, 0,     { { 0x3107, 0x21 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
    { "sc2232",  0x30, 2, 0,     { { 0x3107, 0x22 }, { 0x3108, 0x32 },
                                   { 0x3109, 0x01 } },
      0x0100, 0x01 },
    { "sc2235",  0x30, 2, 0,     { { 0x3107, 0x22 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
    { "sc223a",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x3e } },
      0x0100, 0x01 },
    { "sc2300",  0x30, 2, 0,     { { 0x3107, 0x23 }, { 0x3108, 0x00 } },
      0x0100, 0x01 },
    { "sc231hai", 0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x6a } },
      0x0100, 0x01 },
    { "sc2310",  0x30, 2, 0,     { { 0x3107, 0x23 }, { 0x3108, 0x11 } },
      0x0100, 0x01 },
    { "sc2331",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x5c } },
      0x0100, 0x01 },
    { "sc2332",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x17 } },
      0x0100, 0x01 },
    { "sc2336p", 0x30, 2, 0,     { { 0x3107, 0x9b }, { 0x3108, 0x3a } },
      0x0100, 0x01 },
    { "sc301iot", 0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x40 } },
      0x0100, 0x01 },
    { "sc3235",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x05 } },
      0x0100, 0x01 },
    { "sc3332",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x44 } },
      0x0100, 0x01 },
    { "sc3335",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x1a } },
      0x0100, 0x01 },
    /* sc3338 answers as sc3336 does */
    { "sc3336",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x41 } },
      0x0100, 0x01 },
    { "sc3338",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x41 } },
      0x0100, 0x01 },
    { "sc401ai", 0x30, 2, 0,     { { 0x3107, 0xcd }, { 0x3108, 0x2e } },
      0x0100, 0x01 },
    { "sc4336",  0x30, 2, 0,     { { 0x3107, 0xdc }, { 0x3108, 0x42 } },
      0x0100, 0x01 },
    { "sc4336p", 0x30, 2, 0,     { { 0x3107, 0x9c }, { 0x3108, 0x42 } },
      0x0100, 0x01 },
    { "sc500ai", 0x30, 2, 0,     { { 0x3107, 0xce }, { 0x3108, 0x1f } },
      0x0100, 0x01 },
    { "sc5235",  0x30, 2, 0,     { { 0x3107, 0x52 }, { 0x3108, 0x35 } },
      0x0100, 0x01 },
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
static void image_sensor_expose(double vis, double ir, uint8_t *lut,
                                uint8_t *clut)
{
    double f = vis + ir;
    double c = f > 0 ? MIN(pow(f, 1 / 2.2), 1.0) * vis / f : 0;

    for (int y = 0; y < 256; y++) {
        double n = MIN(MAX((y - 16) / 219.0, 0.0), 1.0);
        double l = MIN(pow(n, 2.2) * f, 1.0);

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
void image_sensor_render(ImageSensorState *s, uint64_t frame, uint32_t w,
                         uint32_t h, uint8_t *nv12)
{
    image_sensor_render_planes(s, frame, w, h, nv12, nv12 + w * h);
}

void image_sensor_render_planes(ImageSensorState *s, uint64_t frame,
                                uint32_t w, uint32_t h, uint8_t *luma,
                                uint8_t *uv)
{
    g_autofree uint8_t *bar = g_malloc(w * 2);
    g_autofree uint8_t *ramp = g_malloc(w * 2);
    uint64_t t = s->motion ? frame : 0;
    uint32_t stripe = MAX(w / 64, 2);
    uint32_t x0 = t * w / (2 * s->fps) % w;
    uint32_t bars_h = h * 2 / 3;
    uint8_t lut[256], clut[256];
    double vis, ir;
    bool mirror = s->input & 1, flip = s->input & 2;
    bool expose;

    s->frame = frame;
    image_sensor_light(s, frame, &vis, &ir);
    expose = s->model->exposure || vis != 1 || ir;
    for (uint32_t x = 0; x < w; x++) {
        uint32_t sx = mirror ? w - 1 - x : x;
        const uint8_t *c = image_sensor_bars[sx * 8 / w];
        bool lit = sx - x0 < stripe;

        bar[x] = lit ? 235 : c[0];
        ramp[x] = lit ? 235 : 16 + sx * 219 / w;
        bar[w + x] = lit ? 128 : c[1 + (x & 1)];
        ramp[w + x] = 128;
    }
    if (s->kelvin && s->kelvin != 6500) {
        double g[3];

        image_sensor_tint(s->kelvin, g);
        image_sensor_tint_row(bar, w, g);
        image_sensor_tint_row(ramp, w, g);
    }
    if (expose) {
        image_sensor_expose(vis, ir, lut, clut);
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
    if (s->stamp) {
        image_sensor_stamp(s, frame, w, h, luma, uv);
    }
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

    memset(uv, 128, (size_t)w * uv_h);
    image_sensor_render_planes(s, frame, w, h, luma, uv);
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
            raw[(size_t)y * w + x] = image_sensor_clamp(rgb) * 257;
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
    image_sensor_inputs[s->input] = s;
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

static void image_sensor_init(Object *obj)
{
    ImageSensorState *s = IMAGE_SENSOR(obj);

    s->light = 100;
    s->light_to = 100;
    s->light_secs = 10;
    s->ambient_ir = 50;
    s->ir = 100;
    s->motion = true;
    object_property_add(obj, "light", "uint32", image_sensor_get_light,
                        image_sensor_set_light, NULL, NULL);
    object_property_add(obj, "light-to", "uint32", NULL,
                        image_sensor_set_light, NULL, NULL);
    object_property_add_uint32_ptr(obj, "light-secs", &s->light_secs,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "kelvin", &s->kelvin,
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
}

static const VMStateDescription vmstate_image_sensor = {
    .name = TYPE_IMAGE_SENSOR,
    .version_id = 3,
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
    DEFINE_PROP_STRING("ircut", ImageSensorState, ircut),
    DEFINE_PROP_STRING("irled", ImageSensorState, irled),
};

static void image_sensor_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Camera image sensor (I2C control)";
    dc->realize = image_sensor_realize;
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
