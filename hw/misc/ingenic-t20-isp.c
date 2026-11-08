/*
 * Ingenic T10/T20/T30 ISP and VIC: the frame path
 *
 * An Apical ISP core (0x80000 into the window) behind Ingenic's VIC (0x0)
 * and an interrupt glue block (0x20000), on one interrupt, INTC source 29.
 * The tx-isp-t10/t20 driver and the Apical firmware program some thousands
 * of registers, which read back as written; what they wait on is modelled:
 *
 * - VIC control (0x0, the T10's 0x20) reads 0: a command completes at once.
 *   Bit 0 starts frames at the image sensor's rate, bit 2 or 3 stops them;
 *   bit 4 (VIC_RESET) does not stop them, as the T20 driver writes it at
 *   frame-done counts 0, 10 and 20 and never restarts.
 *
 * - Apical interrupts: 16 slots, each taking events from the source ID in
 *   a 6-bit field of 0x80080-0x8009c (slot 2k bits 5:0, 2k + 1 bits
 *   21:16); 0x800a0 latches a bit per slot, and a 0 to 1 transition of a
 *   bit written to 0x800a4 clears it. The glue's status (0x2000c) shows
 *   them in bits 15:0 and the VIC frame done in bit 16, cleared by writing
 *   1 to 0x20004; the interrupt is status & enable (0x20014) & ~mask
 *   (0x2001c). The firmware disables and re-enables bits 15:0 of 0x20014
 *   around its critical sections; events latch meanwhile.
 *
 * - Output: Apical video DMA writers, FR at 0x80b00, DS1 at 0x80c00 and
 *   DS2 at 0x80d00, each a luma writer and a chroma one 0x80 above: +0x0
 *   bits 3:0 the format (13 NV12, 6 RAW16, 7 RAW12), 7:6 the plane (1 UV,
 *   2 VU), bit 9 on;
 *   +0x4 the size (height << 16 | width); +0x8-0x18 five bank addresses
 *   the driver fills; +0x1c the last bank (bits 2:0) and a restart at bank
 *   0 (bit 3); +0x20 the line offset, signed (vflip); +0x24 bits 1:0
 *   control (2 runs, 3 cancels), reading back the current bank (10:8) and
 *   the last (13:11). At each frame a running writer moves to its next
 *   bank and writes the frame there; the driver learns from +0x24 which
 *   bank finished. There is no FIFO and no frame number.
 *
 * - Raw output: with the core's processing bypassed (0x80044 bits 14:13
 *   at 2 and bit 9 on, the DS1 writer fed the raw input by 0x80040 bit
 *   30), a driver gives a writer RAW16 and a line offset of twice the
 *   width (libimp's PIX_FMT_RAW after IMP_ISP_Tuning_SetISPBypass,
 *   sample-Snap-Raw): a T20X's is the DS1 writer, a T10L's the FR writer.
 *   It puts out the sensor's samples as they come, before any black level
 *   or gain: one colour a pixel in the Bayer order of 0x80018 (0 starts
 *   R Gr / Gb B, 1 Gr R / B Gb, 2 Gb B / R Gr, 3 B Gb / Gr R), the
 *   sensor's 10 bits left justified in 16, little-endian, the low 6 bits
 *   0. Lines are +0x20 bytes apart with nothing written between them, and
 *   the writer's size (+0x4) is what it takes of the frame, from its top
 *   left corner. RAW12 takes the 10 bits left justified in 12, two
 *   samples to 3 bytes, little-endian, the first in the low 12 bits. The
 *   events are those of any format: the writer done (DS1 15, FR 14) with
 *   the frame end. The model's samples are the raw picture the AWB
 *   statistics see (t20_isp_raw_ratios()) in linear light; the sensors'
 *   black level and noise are not modelled.
 *
 *   A T30's libimp uses the FR writer, straight to DDR (banks of the
 *   channel's buffers, not the line buffer), but the sensor's samples are
 *   on its DS1 writer: the FR writer runs, writes its lines and
 *   frames, and all of them are zeros (a T30X's snap.raw is all zero).
 *
 * - A frame: frame start (Apical event 24), then half a frame later, the
 *   writes, each channel's writer done (FR 14, DS1 15, DS2 1), frame end
 *   (23), the DS1 and DS2 output ends (50, 53, which re-enable a cancelled
 *   writer), the AE and AWB statistics (11, 9) and the VIC frame done. The
 *   driver fills the banks at frame start: writing at once would put the
 *   first frame after a restart into the last session's buffers.
 *
 * - 0x80134 bit 0 says the input port runs while 0x80130 bits 2:0 ask for
 *   it (1, 3 or 5).
 *
 * - Statistics, written as the AE and AWB events come, from the scene the
 *   image sensor shows (its active size at 0x80010 and 0x80014), as the
 *   Apical firmware reads them and a T20X writes them:
 *   - AE: a histogram of linear luma (the scene's 2.2 gamma undone), 256
 *     bins at 0x90000, each an Apical float (bits 15:12 an exponent e, 11:0
 *     a mantissa m: m, or (m | 0x1000) << (e - 1)). Half the pixels count,
 *     each by the weight of its zone, a byte at 0x81e00 + row * 15 + column
 *     of a grid of 0x80830 bits 7:0 columns and 15:8 rows splitting the
 *     picture evenly; weight 15 counts 16. Its 5 bins, split at the bytes
 *     of 0x80800 to 0x8080c, in 1/65535ths at 0x80820, 0x80824, 0x80828 and
 *     0x8082c (the middle one left out); the pixels at 0x808c8 and those
 *     weighted at 0x808cc. Per zone, two words from 0x88000, the same
 *     bins of all its pixels, unweighted, in 1/65536ths (65535 at most):
 *     bins 0 and 1 in bits 15:0 and 31:16 of the first, 3 and 4 of the
 *     second.
 *   - AWB: per zone of the grid of 0x80870, two words from 0x88740, the
 *     ratios of the sums of its pixels that count, G/R in bits 11:0 and G/B
 *     in 27:16 (R/G and B/G with 0x80868 bit 0 set), 1.0 = 0x100, 0xfff
 *     with none, and their number; the whole picture's at 0x80858,
 *     0x8085c and 0x80860, each zone's pixels counting by its weight, a
 *     byte at 0x81f00 + row * 15 + column in 1/16ths, 15 counting 16. A
 *     pixel counts with R/G and B/G (in 1/256ths) within 0x8084c to
 *     0x80848 and 0x80854 to 0x80850 and its channels, 10 bits, from
 *     0x80844 to 0x80840. Its colours are the raw picture's through the
 *     static white balance (0x80300 R, 0x80304 Gr, 0x80308 Gb, 0x8030c B,
 *     0x100 = 1.0, whichever colour the Bayer order of 0x80018 starts
 *     with), in linear light: the model takes the scene, white-balanced,
 *     back to raw (t20_isp_raw_ratios()) and applies the guest's gains to
 *     it. A switch of 0x80048 to 7 or 0x8004c to 2 turns either off.
 *   - AF (0x80050 bits 1:0 below 2): per zone of the grid of 0x80890, a
 *     word from 0x88e80, of the picture in linear light (16 bits) and its
 *     intensity, a quarter of its mean, at 0x80888 bits 31:16 and 0x8089c
 *     (twice that with 0x80898 bit 2 set; a T20X's, 1.95 to 2.06 times).
 *     Each pixel's contrast to its neighbours, across and down, counts
 *     relative to the noise of their level (its square root). Bits 15:0:
 *     the contrast above the threshold of 0x80884 bits 15:0, squared,
 *     summed over the zone and divided by its pixels and the intensity
 *     squared, times 2 to the 0x8088c; 65535 at most, and with no
 *     intensity. Bits 31:16, by 0x80898 bits 1:0: 0 the intensity, 1 or 2
 *     the zone's contrast unthresholded, 3 nothing (the whole word 0).
 *     0x80880 bits 15:0 is the whole picture's sum as bits 15:0 of a zone
 *     a 16th of the picture, whatever the grid and mode.
 *     The scale is a T20X's in its 15x15 grid, looking at its own test
 *     pattern (0x80040 bit 0 clear, 0x809c0 bit 0 set) with its raw front
 *     end bypassed (0x80040 bits 15:1 set but 11): a step of 0x4000
 *     to 0x5000 reads 16 in bits 31:16, 0.55 of the step over the square
 *     root of its mean level, and a level of 0 reads 65535 everywhere. The
 *     model takes that test pattern, a flat field (pattern 0) or a
 *     rectangle (4), as the picture AF sees; the other patterns not.
 *     Bits 15:0 of single edges come within 20% of a T20X's (a step of
 *     0x800 at 0x4000 reads twice its). Not modelled: the T20X's edges at
 *     the picture's left and top borders, its weaker edges falling to the
 *     right, the threshold it computes when 0x80884 is 0 (0x80888 bits
 *     15:0, reading 0), the alternative metric (0x80880 bits 31:16) and
 *     the scale of grids other than 15x15.
 *   - MVE, the motion vector engine (0x80054 bits 1:0 its tap, 3 off; dump
 *     on, 0x80950 bit 2): per zone of the grid of 0x80908 (columns bits
 *     4:0, rows 12:8, at 0x8090c and 0x80910 from the top left, 0x80914 x
 *     0x80918 pixels each), two words from 0x895c0: a vector, x in bits
 *     7:0 and y in 23:16, signed, then the least cost of its search (bits
 *     15:0) and the mean of all (31:16). A T20X keeps no reference frame:
 *     every cost is of the current frame alone, so the vectors follow
 *     where the picture is dark, not how it moves (a rectangle moved each
 *     frame reads what it reads standing still there). The engine looks
 *     at one point a zone, 0x80914 / 2 - 1 pixels and 0x80918 / 2 - 1
 *     lines into it, moved by each offset of x from -range to range - 1
 *     (0x8091c) and y likewise (0x80920), in a picture of 9 bits smoothed
 *     by 1 2 1 across and down. A cost sums five features, each times its
 *     scale (0x80930 bits 3:0, 11:8, 19:16, 27:24, 0x80934 bits 3:0) and
 *     clipped (0x80938 to 0x80948, 12 bits): the mean of the 8 pixels up
 *     to the point, and four filters running along the line from its
 *     start (the first pixel their start), each of a length L (0x80924
 *     bits 7:0, 23:16, 0x80928 likewise) taking 9 / (8 L) of each new
 *     pixel, read at the point (the first) or the pixel before it, times
 *     8 / L; 4095 at most. The search keeps the first least cost under
 *     4095 (else 0, 0). Search areas a zone cannot hold (2 x range above
 *     its width, or not under its height) read 0 0 and costs 4095 and 0;
 *     no dump keeps the words, 0x80950 bit 1 zeroes them. A flat field
 *     reads in every zone the sum of each scale times 8 / L of its level
 *     >> 7 (the pixel mean's L being 8), and (-range, -range). The model
 *     takes the test pattern when on, else the scene in linear light, at
 *     taps 0 and 1 alike, and the scene at tap 2 (the T20X keeps the words
 *     at tap 2 with its test pattern on). On a T20X's test pattern its
 *     vectors match the model's within a pixel and its costs within about
 *     2 percent; not modelled: the end of the pipeline at tap 0 (its
 *     sharpening's dark halos draw the vectors there), the quarter mean
 *     it reads with 16-pixel zones and ranges under 8, and its writing
 *     only the first 8 zones when the search does not fit.
 *
 * The T30's ISP is the same core, its VIC events in the glue as well, but
 * libimp feeds its frame writer to a line buffer (bank addresses
 * 0xf0000000 and 0xf8000000) on to the noise reducer (ingenic-t30-ncu)
 * and the multi-scaler, which writes the channels' frames: the model
 * writes no frame here and pulses "frame-start" as a frame starts and
 * "frame-out" before the frame end that opens the line buffer.
 *
 * The layout is the vendor drivers' (ingenic-sdk isp/t20, which the T10
 * shares, and isp/t30) and the Apical firmware's register headers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-t20-isp.h"
#include "hw/misc/ingenic-cgu.h"
#include "hw/misc/ingenic-isp.h"
#include "hw/sensor/image-sensor.h"
#include "trace.h"

#define VIC_START           BIT(0)
#define VIC_STOP            (BIT(2) | BIT(3))

#define TOP_CLR             0x20004
#define TOP_CLR_ALL         0x20008
#define TOP_STA             0x2000c
#define TOP_ENABLE          0x20014
#define TOP_MASK            0x2001c
#define TOP_VIC_DONE        BIT(16)
#define TOP_IRQS            0x7fffff

#define CORE                0x80000
#define CORE_SLOT_SRC       (CORE + 0x80)
#define CORE_INT_STA        (CORE + 0xa0)
#define CORE_INT_CLR        (CORE + 0xa4)
#define CORE_PORT_REQ       (CORE + 0x130)
#define CORE_PORT_STA       (CORE + 0x134)
#define CORE_WRITER         (CORE + 0xb00)  /* FR; DS1, DS2 0x100 apart */
#define CORE_WRITER_END     (CORE + 0xe00)
#define CORE_WIDTH          (CORE + 0x10)
#define CORE_HEIGHT         (CORE + 0x14)
#define CORE_BAYER          (CORE + 0x18)
#define CORE_AE_SWITCH      (CORE + 0x48)
#define CORE_AWB_SWITCH     (CORE + 0x4c)
#define CORE_AF_SWITCH      (CORE + 0x50)
#define CORE_BYPASS         (CORE + 0x40)
#define  BYPASS_TEST_GEN    BIT(0)
#define CORE_WB_GAIN        (CORE + 0x300)
#define CORE_AE_WEIGHT      (CORE + 0x1e00)
#define CORE_AWB_WEIGHT     (CORE + 0x1f00)

/* The metering block */
#define MET                 (CORE + 0x800)
#define MET_HIST_THRESH     (MET + 0x00)    /* four, a word each */
#define MET_HIST            (MET + 0x20)    /* bins 0, 1, 3 and 4 */
#define MET_AE_NODES        (MET + 0x30)
#define MET_AWB_WHITE       (MET + 0x40)
#define MET_AWB_BLACK       (MET + 0x44)
#define MET_CR_MAX          (MET + 0x48)
#define MET_CR_MIN          (MET + 0x4c)
#define MET_CB_MAX          (MET + 0x50)
#define MET_CB_MIN          (MET + 0x54)
#define MET_AWB_RG          (MET + 0x58)
#define MET_AWB_BG          (MET + 0x5c)
#define MET_AWB_SUM         (MET + 0x60)
#define MET_AWB_MODE        (MET + 0x68)
#define MET_AWB_NODES       (MET + 0x70)
#define MET_AF_METRIC       (MET + 0x80)
#define MET_AF_THRESH       (MET + 0x84)
#define MET_AF_READ         (MET + 0x88)    /* threshold, intensity << 16 */
#define MET_AF_SHIFT        (MET + 0x8c)
#define MET_AF_NODES        (MET + 0x90)
#define MET_AF_MODE         (MET + 0x98)
#define MET_AF_INTENSITY    (MET + 0x9c)
#define MET_TOTAL           (MET + 0xc8)
#define MET_COUNTED         (MET + 0xcc)
#define MET_AE_ZONES        (CORE + 0x8000) /* in the metering memory */
#define MET_AWB_ZONES       (CORE + 0x8740)
#define MET_AF_ZONES        (CORE + 0x8e80)
#define HIST_MEM            (CORE + 0x10000)
#define STATS_GRID          15

/* The video test generator */
#define TG_CTRL             (CORE + 0x9c0)
#define  TG_ON              BIT(0)
#define TG_PATTERN          (CORE + 0x9c4)
#define  TG_FLAT            0
#define  TG_RECT            4
#define TG_BACK             (CORE + 0x9c8)  /* R, G, B */
#define TG_FORE             (CORE + 0x9d4)
#define TG_TOP              (CORE + 0x9e8)  /* lines and pixels from 1 */
#define TG_BOTTOM           (CORE + 0x9ec)
#define TG_LEFT             (CORE + 0x9f0)
#define TG_RIGHT            (CORE + 0x9f8)

/*
 * AF, fitted to a T20X: a step of S between levels a and b reads
 * T20_AF_HI * S / sqrt((a + b) / 2) in bits 31:16 of the zone (128x72) it
 * crosses; against the threshold its pixels count T20_AF_V * S / sqrt(...),
 * and T20_AF_LO scales bits 15:0
 */
#define T20_AF_HI           0.55
#define T20_AF_V            11.0
#define T20_AF_LO           3.3e6

/* The motion vector engine */
#define CORE_MVE_SWITCH     (CORE + 0x54)
#define  MVE_OFF            3
#define MVE_WIDTH           (CORE + 0x900)
#define MVE_HEIGHT          (CORE + 0x904)
#define MVE_NODES           (CORE + 0x908)
#define MVE_BORDER_X        (CORE + 0x90c)
#define MVE_BORDER_Y        (CORE + 0x910)
#define MVE_ZONE_W          (CORE + 0x914)
#define MVE_ZONE_H          (CORE + 0x918)
#define MVE_RANGE_X         (CORE + 0x91c)
#define MVE_RANGE_Y         (CORE + 0x920)
#define MVE_LENGTHS         (CORE + 0x924)  /* F1, F2; F3, F4 at 0x928 */
#define MVE_SCALES          (CORE + 0x930)  /* F0-F3; F4 at 0x934 */
#define MVE_CLIPS           (CORE + 0x938)  /* F0-F4, a word each */
#define MVE_DUMP            (CORE + 0x950)
#define  MVE_DUMP_RESTART   BIT(1)
#define  MVE_DUMP_ON        BIT(2)
#define MVE_ZONES           (CORE + 0x95c0)
#define MVE_MAX_ZONES       512             /* to the metering memory's end */
#define MVE_COST_MAX        4095

/* An Apical writer's registers */
#define WR_CFG              0x00
#define  WR_FORMAT          0xf
#define  WR_NV12            13
#define  WR_RAW16           6
#define  WR_RAW12           7
#define  WR_PLANE_VU        (2 << 6)
#define  WR_ON              BIT(9)
#define WR_SIZE             0x04
#define WR_BANK             0x08
#define WR_BANKS            0x1c
#define  WR_RESTART         BIT(3)
#define WR_LINE             0x20
#define WR_CTRL             0x24
#define  WR_RUN             2           /* AXI on, no cancel */
#define WR_BASE_CURR        0x28
#define WR_BASE_LAST        0x2c
#define WR_COUNT            0x30

/* Apical event sources */
#define EV_FRAME_START      24
#define EV_FRAME_END        23
#define EV_AE               11
#define EV_AWB              9

#define R(off)              (s->regs[(off) / 4])

/* The bits of a sample of the sensors these parts have (RAW10) */
#define T20_RAW_BITS        10

/* An 8-bit gamma-2.2 value in linear light, 8-bit */
static uint8_t t20_isp_linear[256];
/* The same as a 10-bit sample in 1/64ths, which the AWB sums keep */
static uint32_t t20_isp_linear_fine[256];
/*
 * Temper's raw 12-bit samples of the scene's 8-bit levels, and back in 16ths,
 * by the tone curve the pipeline puts a flat field through: the luma a T20X
 * puts out for a test pattern level of n * 128 (0.5 of a gray level a step
 * near the black and a few at the white)
 */
static const uint8_t t20_isp_tone[] = {
    0, 23, 42, 57, 71, 83, 95, 108, 118, 127, 136, 145, 154, 163, 171, 177,
    185, 190, 197, 202, 207, 212, 216, 219, 223, 227, 230, 233, 236, 240, 242,
    245, 247,
};
static uint16_t t20_isp_temper_raw[256];
static uint16_t t20_isp_temper_level[4096];

static const uint8_t writer_done[3] = { 14, 15, 1 };    /* FR, DS1, DS2 */
static const uint8_t output_end[2] = { 50, 53 };        /* DS1, DS2 */

static const char *const t20_isp_writers[INGENIC_T20_ISP_WRITERS] = {
    "fr.y", "fr.uv", "ds1.y", "ds1.uv", "ds2.y", "ds2.uv",
};

#define T20_ISP_NAME_LEN    24

/* The guest PC behind an access, as of the start of its translation block */
static uint32_t t20_isp_pc(void)
{
    return current_cpu ? CPU_GET_CLASS(current_cpu)->get_pc(current_cpu) : 0;
}

static int t20_isp_writer(hwaddr off, hwaddr *reg);

/*
 * A register's name for the trace: the VIC's, the glue's ("top."), the
 * Apical core's interrupt and port registers ("core.") and its writers'
 * ("ds1.uv.bank2"); "" for the rest of the core
 */
static const char *t20_isp_reg_name(IngenicT20IspState *s, hwaddr off,
                                    char *buf)
{
    static const struct {
        hwaddr off;
        const char *name;
    } regs[] = {
        { TOP_CLR, "top.clr" }, { TOP_CLR_ALL, "top.clr_all" },
        { TOP_STA, "top.sta" }, { TOP_ENABLE, "top.enable" },
        { TOP_MASK, "top.mask" }, { CORE_INT_STA, "core.int_sta" },
        { CORE_INT_CLR, "core.int_clr" }, { CORE_PORT_REQ, "core.port_req" },
        { CORE_PORT_STA, "core.port_sta" },
    };
    static const char *const wr_regs[] = {
        "cfg", "size", "bank0", "bank1", "bank2", "bank3", "bank4", "banks",
        "line", "ctrl", "base_curr", "base_last", "count",
    };
    hwaddr reg;
    int n;

    if (off == INGENIC_T20_ISP_GET_CLASS(s)->vic_ctrl) {
        return "vic.ctrl";
    }
    for (unsigned i = 0; i < ARRAY_SIZE(regs); i++) {
        if (off == regs[i].off) {
            return regs[i].name;
        }
    }
    if (off >= CORE_SLOT_SRC && off < CORE_SLOT_SRC + 8 * 4) {
        unsigned k = (off - CORE_SLOT_SRC) / 4;

        snprintf(buf, T20_ISP_NAME_LEN, "core.slots%u_%u", 2 * k, 2 * k + 1);
        return buf;
    }
    n = t20_isp_writer(off, &reg);
    if (n >= 0 && reg % 4 == 0 && reg / 4 < ARRAY_SIZE(wr_regs)) {
        snprintf(buf, T20_ISP_NAME_LEN, "%s.%s", t20_isp_writers[n],
                 wr_regs[reg / 4]);
        return buf;
    }
    if (n >= 0) {
        snprintf(buf, T20_ISP_NAME_LEN, "%s+0x%02x", t20_isp_writers[n],
                 (unsigned)reg);
        return buf;
    }
    return "";
}

static void t20_isp_update(IngenicT20IspState *s)
{
    uint32_t sta = (s->status & 0xffff) | s->vic_status;
    bool level = sta & R(TOP_ENABLE) & ~R(TOP_MASK) & TOP_IRQS;

    if (level != s->irq_level) {
        s->irq_level = level;
        trace_ingenic_t20_isp_irq(level);
    }
    qemu_set_irq(s->irq, level);
}

/* Apical event @src latches in the slots set to take it */
static void t20_isp_event(IngenicT20IspState *s, unsigned src)
{
    uint32_t slots = 0;

    for (unsigned slot = 0; slot < 16; slot++) {
        uint32_t v = R(CORE_SLOT_SRC + slot / 2 * 4) >> (slot & 1 ? 16 : 0);

        if ((v & 0x3f) == src) {
            slots |= BIT(slot);
        }
    }
    trace_ingenic_t20_isp_event(src, slots);
    s->status |= slots;
}

static int t20_isp_writer(hwaddr off, hwaddr *reg)
{
    if (off < CORE_WRITER || off >= CORE_WRITER_END) {
        return -1;
    }
    *reg = (off - CORE_WRITER) & 0x7f;
    return (off - CORE_WRITER) / 0x80;
}

/* The writer at @n moves to its next bank; whether it writes this frame */
static bool t20_isp_advance(IngenicT20IspState *s, unsigned n)
{
    hwaddr base = CORE_WRITER + n * 0x80;
    IngenicT20IspWriter *w = &s->writer[n];
    uint32_t banks = R(base + WR_BANKS);

    if (!(R(base + WR_CFG) & WR_ON) || (R(base + WR_CTRL) & 3) != WR_RUN) {
        return false;
    }
    w->last = w->curr;
    w->curr = banks & WR_RESTART || w->curr >= MIN(banks & 7, 4) ? 0
                                                                 : w->curr + 1;
    w->base_last = w->base_curr;
    w->base_curr = R(base + WR_BANK + 4 * w->curr);
    w->frames++;
    trace_ingenic_t20_isp_writer(t20_isp_writers[n], w->curr, w->base_curr,
                                 w->frames);
    return true;
}

static void t20_isp_plane(uint32_t addr, int32_t line, const uint8_t *src,
                          uint32_t w, uint32_t rows)
{
    if (!line) {
        line = w;
    }
    for (uint32_t r = 0; r < rows; r++) {
        address_space_write(&address_space_memory, addr + (int64_t)r * line,
                            MEMTXATTRS_UNSPECIFIED, src + r * w, w);
    }
}

/*
 * The factors that take the scene's linear light, per colour, to the raw
 * signal the core sees, in 1/4096ths (G is 4096): the sensor's own response
 * in the scene's light where its model has an Apical one, else a grey's raw
 * R and B as a T20X's (JXF23) measured them in a room lit by a white LED,
 * where its AWB settled at gains R 1.98, B 1.80. The scene is white-balanced.
 */
static void t20_isp_raw_ratios(ImageSensorState *sensor, uint32_t ratio[3])
{
    double f[3] = {
        IMAGE_SENSOR_RAW_APICAL_R / 256.0, 1, IMAGE_SENSOR_RAW_APICAL_B / 256.0,
    };

    image_sensor_raw_factors(sensor, IMAGE_SENSOR_PART_APICAL, f);
    for (unsigned c = 0; c < 3; c++) {
        ratio[c] = lround(f[c] * 4096);
    }
}

/*
 * The sensor's frame as the bypassed core puts it out: the samples in the
 * Bayer order of 0x80018, the colour of each in @order's (y & 1) * 2 + (x &
 * 1), 10 bits in linear light, as the AWB statistics see the raw picture
 */
static void t20_isp_raw_frame(IngenicT20IspState *s, ImageSensorState *sensor,
                              uint32_t w, uint32_t h, uint16_t *raw)
{
    static const uint8_t bayer[4][4] = {
        { 0, 1, 1, 2 }, { 1, 0, 2, 1 }, { 1, 2, 0, 1 }, { 2, 1, 1, 0 },
    };
    const uint8_t *order = bayer[R(CORE_BAYER) & 3];
    uint32_t ratio[3];
    g_autofree uint8_t *luma = g_malloc((size_t)w * h);
    g_autofree uint8_t *uv = g_malloc((size_t)w * (h / 2));

    t20_isp_raw_ratios(sensor, ratio);
    image_sensor_render_planes(sensor, s->frame, w, h, luma, uv);
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *c = uv + (size_t)(y / 2) * w;

        for (uint32_t x = 0; x < w; x++) {
            unsigned col = order[(y & 1) * 2 + (x & 1)];
            int l = (luma[(size_t)y * w + x] - 16) * 298 + 128;
            int u = c[x & ~1] - 128, v = c[x | 1] - 128;
            int rgb = (col == 0 ? l + 409 * v :
                       col == 1 ? l - 100 * u - 208 * v : l + 516 * u) >> 8;

            raw[(size_t)y * w + x] =
                MIN(t20_isp_linear[MIN(MAX(rgb, 0), 255)] * 4 * ratio[col] /
                    4096, 0x3ff);
        }
    }
}

/* A raw channel's frame, RAW16 or RAW12 as the writer is set to */
static void t20_isp_raw_channel(IngenicT20IspState *s,
                                ImageSensorState *sensor, unsigned n,
                                uint32_t w, uint32_t h)
{
    hwaddr y = CORE_WRITER + n * 0x80;
    g_autofree uint16_t *raw = g_new(uint16_t, (size_t)w * h);
    g_autofree uint8_t *out = NULL;
    bool raw16 = (R(y + WR_CFG) & WR_FORMAT) == WR_RAW16;
    uint32_t bytes = raw16 ? w * 2 : w * 3 / 2;

    if (INGENIC_T20_ISP_GET_CLASS(s)->raw_blank & BIT(n / 2)) {
        memset(raw, 0, (size_t)w * h * sizeof(*raw));
    } else {
        t20_isp_raw_frame(s, sensor, w, h, raw);
    }
    out = g_malloc((size_t)bytes * h);
    for (size_t i = 0; i < (size_t)w * h; i += 2) {
        if (raw16) {
            stw_le_p(out + i * 2, raw[i] << (16 - T20_RAW_BITS));
            stw_le_p(out + i * 2 + 2, raw[i + 1] << (16 - T20_RAW_BITS));
        } else {
            uint32_t pair = raw[i] << (12 - T20_RAW_BITS) |
                            raw[i + 1] << (24 - T20_RAW_BITS);

            stw_le_p(out + i * 3 / 2, pair);
            out[i * 3 / 2 + 2] = pair >> 16;
        }
    }
    t20_isp_plane(s->writer[n].base_curr, R(y + WR_LINE), out, bytes, h);
}

/*
 * The picture Temper puts out: a pixel's filtered value for its frame is
 * a different level of the scene's, by the tone curve (the 2.2 gamma the
 * raw picture t20_isp_temper_frame took from the scene went through), in
 * 16ths of a level of the full-range pixel; the luma moves 1 / 1.164 of
 * that, by the mean of its 2 x 2 Bayer cell
 */
static void t20_isp_temper_picture(IngenicT20IspState *s, uint8_t *luma,
                                   uint32_t w, uint32_t h)
{
    uint32_t tw = s->temper_w, th = s->temper_h;
    g_autofree uint32_t *col = g_new(uint32_t, w);

    for (uint32_t x = 0; x < w; x++) {
        col[x] = MIN((uint64_t)x * tw / w & ~1u, tw - 2);
    }
    for (uint32_t y = 0; y < h; y++) {
        uint32_t ty = MIN((uint64_t)y * th / h & ~1u, th - 2);
        const int16_t *r0 = s->temper_delta + (size_t)ty * tw, *r1 = r0 + tw;

        for (uint32_t x = 0; x < w; x++) {
            uint32_t tx = col[x];
            int d = r0[tx] + r0[tx + 1] + r1[tx] + r1[tx + 1];

            luma[(size_t)y * w + x] =
                MIN(MAX(luma[(size_t)y * w + x] + lround(d / 64.0 / 1.164), 0),
                    255);
        }
    }
}

/* A channel's frame: luma writer @n, chroma writer @n + 1 */
static void t20_isp_channel(IngenicT20IspState *s, ImageSensorState *sensor,
                            unsigned n)
{
    hwaddr y = CORE_WRITER + n * 0x80, uv = y + 0x80;
    uint32_t size = R(y + WR_SIZE);
    uint32_t w = size & 0xfffe, h = (size >> 16) & 0xfffe;
    unsigned format = R(y + WR_CFG) & WR_FORMAT;
    g_autofree uint8_t *nv12 = NULL;
    bool chroma;

    if (!t20_isp_advance(s, n)) {
        return;
    }
    chroma = t20_isp_advance(s, n + 1);
    if (w && h && (format == WR_RAW16 || format == WR_RAW12)) {
        t20_isp_raw_channel(s, sensor, n, w, h);
    }
    if (w && h && format == WR_NV12 &&
        !INGENIC_T20_ISP_GET_CLASS(s)->line_buffer) {
        double f[3];

        nv12 = g_malloc(w * h * 3 / 2);
        image_sensor_render(sensor, s->frame, w, h, nv12);
        if (s->temper_delta) {
            t20_isp_temper_picture(s, nv12, w, h);
        }
        if (ingenic_t20_isp_wb_factors(s, sensor, f)) {
            ingenic_isp_wb_picture(nv12, nv12 + w * h, w, h, f);
        }
        t20_isp_plane(s->writer[n].base_curr, R(y + WR_LINE), nv12, w, h);
        if (chroma) {
            uint8_t *c = nv12 + w * h;

            if ((R(uv + WR_CFG) & (3 << 6)) == WR_PLANE_VU) {
                for (uint32_t i = 0; i < w * h / 2; i += 2) {
                    uint8_t t = c[i];

                    c[i] = c[i + 1];
                    c[i + 1] = t;
                }
            }
            t20_isp_plane(s->writer[n + 1].base_curr, R(uv + WR_LINE), c, w,
                          h / 2);
        }
    }
    t20_isp_event(s, writer_done[n / 2]);
}

/* An Apical float: 12 bits of mantissa, an exponent in 15:12 */
static uint32_t t20_isp_float(uint64_t v)
{
    unsigned e;

    if (v < 0x1000) {
        return v;
    }
    e = MIN(63 - clz64(v) - 11, 15);
    return e << 12 | (MIN(v >> (e - 1), 0x1fff) & 0xfff);
}

/* The zones of n along len, evenly: the zone of each position */
static void t20_isp_grid(uint8_t *map, uint32_t len, unsigned n)
{
    for (uint32_t i = 0; i < len; i++) {
        map[i] = n ? (uint64_t)i * n / len : 0;
    }
}

/*
 * The static white balance's gain for colour c (0 R, 1 G, 2 B), 1.0 = 256:
 * 0x80300 R, 0x80304 and 0x80308 Gr and Gb, 0x8030c B, by colour whatever
 * the Bayer order 0x80018 gives (a T20X with a B-first sensor: holding
 * 0x80300 up moves the G/R statistics)
 */
static uint32_t t20_isp_wb_gain(IngenicT20IspState *s, unsigned c)
{
    switch (c) {
    case 0:
        return R(CORE_WB_GAIN) & 0xfff;
    case 1:
        return ((R(CORE_WB_GAIN + 4) & 0xfff) +
                (R(CORE_WB_GAIN + 8) & 0xfff)) / 2;
    default:
        return R(CORE_WB_GAIN + 12) & 0xfff;
    }
}

bool ingenic_t20_isp_wb_factors(IngenicT20IspState *s,
                                ImageSensorState *sensor, double f[3])
{
    uint32_t raw[3];

    t20_isp_raw_ratios(sensor, raw);
    for (unsigned c = 0; c < 3; c++) {
        uint32_t g = t20_isp_wb_gain(s, c);

        if (!g) {
            return false;
        }
        f[c] = g / 256.0 * raw[c] / 4096;
    }
    return true;
}

typedef struct T20IspAwbSums {
    uint64_t rgb[3];                /* in 1/64ths */
    uint32_t n;                     /* pixels */
} T20IspAwbSums;

/* A zone's ratios: G/R | G/B << 16, or R/G and B/G */
static uint32_t t20_isp_awb_ratios(IngenicT20IspState *s,
                                   const T20IspAwbSums *z)
{
    bool cur = R(MET_AWB_MODE) & 1;
    uint64_t a[2] = { z->rgb[0], z->rgb[2] }, g = z->rgb[1];
    uint32_t v[2];

    for (unsigned i = 0; i < 2; i++) {
        uint64_t num = cur ? a[i] : g, den = cur ? g : a[i];

        v[i] = den ? MIN(num * 256 / den, 0xfff) : 0xfff;
    }
    return v[0] | v[1] << 16;
}

/*
 * The picture AF sees, @w x @h, a pixel of each 2x2 in linear light (16
 * bits): the test pattern when on, else the scene of @pic (8-bit luma)
 */
static bool t20_isp_tg_on(IngenicT20IspState *s)
{
    return !(R(CORE_BYPASS) & BYPASS_TEST_GEN) && (R(TG_CTRL) & TG_ON);
}

/*
 * The test pattern's luma (of 16-bit R, G and B) at line @ly and pixel @lx
 * (from 1); false for a pattern not modelled
 */
static bool t20_isp_tg_level(IngenicT20IspState *s, uint32_t ly, uint32_t lx,
                             uint32_t *level)
{
    uint32_t pattern = R(TG_PATTERN) & 0xff;
    bool in = pattern == TG_RECT &&
              ly >= (R(TG_TOP) & 0x3fff) && ly <= (R(TG_BOTTOM) & 0x3fff) &&
              lx >= (R(TG_LEFT) & 0x3fff) && lx <= (R(TG_RIGHT) & 0x3fff);
    hwaddr c = in ? TG_FORE : TG_BACK;

    *level = ((R(c) & 0xffff) + 2 * (R(c + 4) & 0xffff) +
              (R(c + 8) & 0xffff)) / 4;
    return pattern == TG_FLAT || pattern == TG_RECT;
}

static void t20_isp_af_picture(IngenicT20IspState *s, const uint8_t *pic,
                               uint32_t w, uint32_t h, uint16_t *l)
{
    uint32_t level;

    if (!t20_isp_tg_on(s) || !t20_isp_tg_level(s, 1, 1, &level)) {
        for (uint32_t i = 0; i < w * h; i++) {
            l[i] = t20_isp_linear[pic[i]] * 257;
        }
        return;
    }
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            t20_isp_tg_level(s, y * 2 + 1, x * 2 + 1, &level);
            l[y * w + x] = level;
        }
    }
}

/*
 * A pixel's contrast to its neighbours @a and @b, in the units of bits
 * 31:16 (*c) and of the threshold (*v)
 */
static void t20_isp_af_contrast(uint32_t a, uint32_t b, double *c, double *v)
{
    double d = a > b ? a - b : b - a;
    double noise = sqrt(MAX((a + b) / 2, 1u));

    *c = d / noise;
    *v = T20_AF_V * d / noise;
}

/* The AF zones of the picture @pic, @w x @h (half the frame's) */
static void t20_isp_af(IngenicT20IspState *s, const uint8_t *pic, uint32_t w,
                       uint32_t h)
{
    unsigned nh = MIN(extract32(R(MET_AF_NODES), 0, 8), STATS_GRID);
    unsigned nv = MIN(extract32(R(MET_AF_NODES), 8, 8), STATS_GRID);
    unsigned mode = R(MET_AF_MODE) & 7, shift = R(MET_AF_SHIFT) & 0xf;
    double thr = R(MET_AF_THRESH) & 0xffff;
    g_autofree uint16_t *l = g_new(uint16_t, w * h);
    g_autofree uint8_t *col = g_malloc(w);
    g_autofree uint8_t *row = g_malloc(h);
    double con[STATS_GRID * STATS_GRID] = { 0 };
    double sq[STATS_GRID * STATS_GRID] = { 0 };
    uint32_t area[STATS_GRID * STATS_GRID] = { 0 };
    double all = 0, scale, intensity;
    uint64_t sum = 0;

    t20_isp_af_picture(s, pic, w, h, l);
    for (uint32_t i = 0; i < w * h; i++) {
        sum += l[i];
    }
    intensity = (sum / (w * h) >> 2) << (mode & 4 ? 1 : 0);
    R(MET_AF_READ) = (uint32_t)intensity << 16;
    R(MET_AF_INTENSITY) = intensity;
    t20_isp_grid(col, w, nh);
    t20_isp_grid(row, h, nv);
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            unsigned z = row[y] * STATS_GRID + col[x];
            uint32_t i = y * w + x;
            double c, v;

            area[z]++;
            for (unsigned dir = 0; dir < 2; dir++) {
                uint32_t step = dir ? w : 1;

                if (dir ? y == 0 || y == h - 1 : x == 0 || x == w - 1) {
                    continue;
                }
                if (l[i - step] == l[i + step]) {
                    continue;
                }
                t20_isp_af_contrast(l[i - step], l[i + step], &c, &v);
                con[z] += c;
                v = MAX(v - thr, 0);
                sq[z] += v * v;
                all += v * v;
            }
        }
    }
    /*
     * Each pixel here stands for 2x2 of the frame: per pixel of the frame,
     * the sums are half these over the pixels counted here
     */
    scale = intensity ? T20_AF_LO * (1u << shift) / (intensity * intensity)
                      : 0;
    for (unsigned z = 0; z < nh * nv; z++) {
        unsigned zi = z / nh * STATS_GRID + z % nh;
        double n = MAX(area[zi], 1u) * 2.0;
        uint32_t lo = intensity ? MIN(sq[zi] / n * scale, 0xffff) : 0xffff;
        uint32_t hi;

        switch (mode & 3) {
        case 0:
            hi = intensity;
            break;
        case 3:
            R(MET_AF_ZONES + z * 4) = 0;
            continue;
        default:
            hi = MIN(T20_AF_HI * 64 * con[zi] / n, 0xffff);
        }
        R(MET_AF_ZONES + z * 4) = lo | MIN(hi, 0xffff) << 16;
    }
    /* the whole picture's, in a 16th of it */
    R(MET_AF_METRIC) = intensity ? MIN(all / (w * h * 2.0) * 16 * scale,
                                       0xffff)
                                 : 0xffff;
}

/*
 * MVE: the first @n pixels of line @y (clamped) of the picture it sees, @w
 * x @h, 9 bits: the test pattern when @tg, else @pic (the scene, @qw x
 * @qh, 8-bit luma) stretched
 */
static void t20_isp_mve_row(IngenicT20IspState *s, bool tg, const uint8_t *pic,
                            uint32_t qw, uint32_t qh, uint32_t w, uint32_t h,
                            int32_t y, uint32_t n, double *v)
{
    uint32_t ly = MIN(MAX(y, 0), (int32_t)h - 1);
    const uint8_t *row = pic + (uint64_t)ly * qh / h * qw;

    for (uint32_t x = 0; x < n; x++) {
        uint32_t level;

        if (tg) {
            t20_isp_tg_level(s, ly + 1, x + 1, &level);
        } else {
            level = t20_isp_linear[row[(uint64_t)x * qw / w]] * 257;
        }
        v[x] = level >> 7;
    }
}

/* MVE: the cost of the first @n points of line @y into @cost */
static void t20_isp_mve_costs(IngenicT20IspState *s, bool tg,
                              const uint8_t *pic, uint32_t qw, uint32_t qh,
                              uint32_t w, uint32_t h, int32_t y, uint32_t n,
                              uint16_t *cost)
{
    uint32_t m = MIN(n + 1, w);     /* the last point's right neighbour */
    g_autofree double *r = g_new(double, m * 3);
    g_autofree double *l = g_new(double, m);
    unsigned len[4] = {
        extract32(R(MVE_LENGTHS), 0, 8), extract32(R(MVE_LENGTHS), 16, 8),
        extract32(R(MVE_LENGTHS + 4), 0, 8),
        extract32(R(MVE_LENGTHS + 4), 16, 8),
    };
    unsigned scale[5] = {
        extract32(R(MVE_SCALES), 0, 4), extract32(R(MVE_SCALES), 8, 4),
        extract32(R(MVE_SCALES), 16, 4), extract32(R(MVE_SCALES), 24, 4),
        extract32(R(MVE_SCALES + 4), 0, 4),
    };
    double f[4], sum;

    for (int i = 0; i < 3; i++) {
        t20_isp_mve_row(s, tg, pic, qw, qh, w, h, y + i - 1, m, r + i * m);
    }
    /* 1 2 1 down, then across */
    for (uint32_t x = 0; x < m; x++) {
        r[x] = (r[x] + 2 * r[m + x] + r[2 * m + x]) / 4;
    }
    for (uint32_t x = 0; x < n; x++) {
        l[x] = (r[x ? x - 1 : 0] + 2 * r[x] + r[MIN(x + 1, m - 1)]) / 4;
    }
    /* each runs from the line's first pixel, which also pads the mean */
    for (unsigned k = 0; k < 4; k++) {
        len[k] = MAX(len[k], 1);
        f[k] = l[0];
    }
    sum = 8 * l[0];
    for (uint32_t x = 0; x < n; x++) {
        uint32_t c;

        sum += l[x] - (x >= 8 ? l[x - 8] : l[0]);
        c = MIN(scale[0] * (uint32_t)(sum / 8), R(MVE_CLIPS) & 0xfff);
        for (unsigned k = 0; k < 4; k++) {
            double before = f[k];

            f[k] += 9.0 / 8 / len[k] * (l[x] - f[k]);
            /* the first filter is read at the point, the others before it */
            c += MIN(scale[k + 1] * (uint32_t)((k ? before : f[k]) * 8 /
                                                len[k]),
                     R(MVE_CLIPS + 4 * (k + 1)) & 0xfff);
        }
        cost[x] = MIN(c, MVE_COST_MAX);
    }
}

/* MVE: each zone's vector and costs, of the scene @pic (@qw x @qh) */
static void t20_isp_mve(IngenicT20IspState *s, const uint8_t *pic, uint32_t qw,
                        uint32_t qh)
{
    unsigned tap = R(CORE_MVE_SWITCH) & 3;
    unsigned cols = extract32(R(MVE_NODES), 0, 5);
    unsigned rows = extract32(R(MVE_NODES), 8, 5);
    unsigned zones = MIN(cols * rows, MVE_MAX_ZONES);
    uint32_t w = R(MVE_WIDTH) & 0xffff, h = R(MVE_HEIGHT) & 0xffff;
    uint32_t zw = R(MVE_ZONE_W) & 0x1ff, zh = R(MVE_ZONE_H) & 0x1ff;
    int32_t rx = R(MVE_RANGE_X) & 0xff, ry = R(MVE_RANGE_Y) & 0xff;
    bool tg = tap != 2 && t20_isp_tg_on(s);
    g_autofree uint16_t *cost = NULL;
    g_autofree uint32_t *sum = NULL, *best = NULL;
    uint32_t level, n;

    if (R(MVE_DUMP) & MVE_DUMP_RESTART) {
        for (unsigned z = 0; z < zones * 2; z++) {
            R(MVE_ZONES + z * 4) = 0;
        }
        return;
    }
    /* the T20X keeps the words with no range, and at tap 2 on its pattern */
    if (tap == MVE_OFF || !(R(MVE_DUMP) & MVE_DUMP_ON) || !rx || !ry ||
        !w || !h || (tap == 2 && t20_isp_tg_on(s))) {
        return;
    }
    if (2 * rx > zw || 2 * ry >= zh) {
        for (unsigned z = 0; z < zones; z++) {
            R(MVE_ZONES + z * 8) = 0;
            R(MVE_ZONES + z * 8 + 4) = MVE_COST_MAX;
        }
        return;
    }
    tg = tg && t20_isp_tg_level(s, 1, 1, &level);
    /* the filters run from each line's start to the last point searched */
    n = MIN((R(MVE_BORDER_X) & 0xffff) + cols * zw + rx, w);
    cost = g_new(uint16_t, n);
    sum = g_new(uint32_t, cols);
    best = g_new(uint32_t, cols);
    for (unsigned r = 0; r < rows; r++) {
        int32_t sy = (R(MVE_BORDER_Y) & 0xffff) + r * zh + zh / 2 - 1;

        if (r * cols >= zones) {
            break;
        }
        for (unsigned c = 0; c < cols; c++) {
            sum[c] = 0;
            best[c] = MVE_COST_MAX;     /* as 0, 0 */
        }
        for (int32_t dy = -ry; dy < ry; dy++) {
            t20_isp_mve_costs(s, tg, pic, qw, qh, w, h, sy + dy, n, cost);
            for (unsigned c = 0; c < cols; c++) {
                int32_t sx = (R(MVE_BORDER_X) & 0xffff) + c * zw + zw / 2 - 1;

                for (int32_t dx = -rx; dx < rx; dx++) {
                    uint32_t v = cost[MIN(MAX(sx + dx, 0), (int32_t)n - 1)];

                    sum[c] += v;
                    if (v < (best[c] & 0xffff)) {
                        best[c] = v | (dx & 0xff) << 16 | (dy & 0xff) << 24;
                    }
                }
            }
        }
        for (unsigned c = 0; c < cols && r * cols + c < zones; c++) {
            hwaddr z = MVE_ZONES + (r * cols + c) * 8;

            R(z) = extract32(best[c], 16, 8) | extract32(best[c], 24, 8) << 16;
            R(z + 4) = (best[c] & 0xffff) | sum[c] / (4 * rx * ry) << 16;
        }
    }
}

/* The AE, AWB, AF and MVE statistics of the sensor's frame */
/*
 * Temper, the core's temporal noise reducer on the raw picture (0x40 bit
 * 10 bypasses it), keeps a frame in DDR the driver arms at 0xa18 (bank
 * 0xa1c, line offset 0xa24, 0xa28 bit 0 its DMA): a 32-bit word a pixel,
 * the frame in bits 27:16 and its filtered value in 11:0, 12 bits each,
 * each with a 4-bit count above it. As a T20X, a T10L and a T30X run it on
 * the core's own test pattern (flat levels, 0x40 low 16 bits 0xf3fe):
 *
 * - The filtered value's count falls a frame at a time to 2, then goes 3,
 *   2, 3. The value moves to the frame by a share of the difference, 1 /
 *   (1.0667 + 0.39 * (15 - count)) rounded down at the half, but at least
 *   (L + 1) / 16 with the recursion limit L (0x2cc bits 3:0): a step from
 *   1024 to 2048 reads 1984, 2028, 2039, 2043, 2045, 2046, 2047 at L = 2,
 *   and L of 15 takes the frame as it is.
 * - A change restarts the counts. 0x2c4 bits 7:0 are a noise level T, log2
 *   with 4 fractional bits: u = U0 * 2 ^ (T / 16) (U0 0.5594, a T10's
 *   0.614; measured from T = 30 to 140) is the change that restarts a
 *   count at 6, k times u at 4 + 2 k (k to 5, 14), and 14.08 u at 15, at a
 *   pixel level of 2048: u goes with the square root of the level of the
 *   last frame's pixel (a quarter of it at 128). The change is the largest
 *   in the 3 rows by 7 columns around the pixel (a line of one pixel
 *   restarts the pixels 3 columns and 1 row away at the count it gives
 *   its own; a real scene's noise restarts the counts of a patch, not of a
 *   pixel). The upper count (the frame's) goes by the change from the last
 *   frame, the lower (the filtered value's) by the least of that and the
 *   change from the filtered value. A count restarts only above the count
 *   it has (not above where it would fall to): a change that comes back
 *   each frame restarts it on the way up and lets it fall on the way down.
 *   On the restart frame the filtered value moves by a share that grows
 *   with the largest change from the filtered value in the neighbourhood
 *   over u at the new pixel's level, from 3 / 16 (4 / 16 at 3 u) to all of
 *   it at 28.5 u (t20_temper_alpha_at).
 * - 0x2c8 bits 7:0 is the same noise level for the long exposure of a
 *   frame stitched in WDR (0x40 bit 7 clear, 0x240 bits 7:6); with the
 *   stitch bypassed, as a linear sensor runs it, it does nothing.
 *
 * With 0x2c0 bit 0 clear the filtered value is the frame; with the bypass,
 * or the DMA off, the words stay as they are. The pixels are the test
 * pattern's (16 bits, the top 12) or the scene's, the levels the tone curve
 * gives (t20_isp_tone, t20_isp_temper_frame); the picture put out, the
 * scene's, takes each pixel's filtered value for its frame
 * (t20_isp_temper_picture). Not modelled: the lower count catching up
 * with the upper while the filtered value is far off, 0x2c0's other bits,
 * the lower count of a change near 3 u or 5 u, which a T20X restarts one
 * step short of here, and what the frame stitch at 0xa40 does with a WDR
 * sensor's frames (no sensor here has WDR; it keeps the input frame in its
 * bank, t20_isp_stitch, and its registers hold the bits a T20X's do,
 * t20_isp_write_mask).
 *
 * Of the noise of a T20X's scene (a patch of 64 x 16 pixels, 120 frames),
 * the filtered value keeps from 0.99 of the frame to frame change at T =
 * 40 to 0.14 from T = 110; replayed on those frames the words here keep
 * the same, to 0.03.
 */
#define CORE_TEMPER         (CORE + 0x2c0)
#define  TEMPER_ON          BIT(0)
#define CORE_TEMPER_THRESH  (CORE + 0x2c4)
#define CORE_TEMPER_LIMIT   (CORE + 0x2cc)
#define T20_TEMPER_FULL     14.08f
#define T20_TEMPER_RX       3
#define  BYPASS_TEMPER      BIT(10)
#define TEMPER_BANK         (CORE + 0xa18)
#define TEMPER_LINE         (CORE + 0xa24)
#define TEMPER_DMA          (CORE + 0xa28)
#define  BYPASS_STITCH      BIT(7)
#define STITCH_CTRL         (CORE + 0xa40)
#define STITCH_WIDTH        (CORE + 0xa50)
#define STITCH_HEIGHT       (CORE + 0xa54)
#define STITCH_BANK         (CORE + 0xa58)
#define STITCH_LINE         (CORE + 0xa64)
#define STITCH_DMA          (CORE + 0xa68)

/*
 * The share of the difference the filtered value moves on the frame a
 * pixel's count restarts: n / 16 from the size k of the difference, in
 * units of u (see t20_isp_temper) over the square root of the pixel's level
 * in 2048ths, at which it reaches n
 */
static const struct {
    float k;
    uint8_t n;
} t20_temper_alpha_at[] = {
    { 3.0, 4 }, { 4.0, 5 }, { 5.0, 6 }, { 6.0, 7 }, { 7.1, 8 }, { 8.2, 9 },
    { 10.1, 10 }, { 12.2, 11 }, { 14.1, 12 }, { 16.1, 13 }, { 20.2, 14 },
    { 24.0, 15 }, { 28.5, 16 },
};

/* The raw frame Temper takes, 12 bits a pixel */
static void t20_isp_temper_frame(IngenicT20IspState *s,
                                 ImageSensorState *sensor, uint32_t w,
                                 uint32_t h, uint16_t *raw)
{
    uint32_t level;

    if (t20_isp_tg_on(s) && t20_isp_tg_level(s, 1, 1, &level)) {
        for (uint32_t y = 0; y < h; y++) {
            for (uint32_t x = 0; x < w; x++) {
                t20_isp_tg_level(s, y + 1, x + 1, &level);
                raw[(size_t)y * w + x] = level >> 4;
            }
        }
        return;
    }
    /* image_sensor_render_bayer()'s picture, in integers */
    g_autofree uint8_t *luma = g_malloc((size_t)w * h);
    g_autofree uint8_t *uv = g_malloc((size_t)w * MAX(h / 2, 1));

    image_sensor_render_planes(sensor, s->frame, w, h, luma, uv);
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *l = luma + (size_t)y * w;
        const uint8_t *c = uv + (size_t)MIN(y / 2, MAX(h / 2, 1) - 1) * w;
        uint16_t *o = raw + (size_t)y * w;

        for (uint32_t x = 0; x < w; x += 2) {
            int u = c[x] - 128, v = x + 1 < w ? c[x + 1] - 128 : 0;
            int a = y & 1 ? -100 * u - 208 * v : 409 * v;
            int b = y & 1 ? 516 * u : -100 * u - 208 * v;
            int p0 = (298 * (l[x] - 16) + 128 + a) >> 8;

            o[x] = t20_isp_temper_raw[MIN(MAX(p0, 0), 255)];
            if (x + 1 < w) {
                int p1 = (298 * (l[x + 1] - 16) + 128 + b) >> 8;

                o[x + 1] = t20_isp_temper_raw[MIN(MAX(p1, 0), 255)];
            }
        }
    }
}

/*
 * The tables of one frame: by a level, 1 / u at that level (u grows with
 * the square root of the level, as the noise does), and by the size of a
 * change in units of u, the share the filtered value moves by
 */
typedef struct T20TemperTables {
    float by_level[4096];
    uint8_t share[512];             /* n, by the size k in 16ths of a u */
    uint32_t least;                 /* the recursion limit's share, 256ths */
} T20TemperTables;

static void t20_temper_tables(T20TemperTables *tb, double u0, unsigned noise,
                              unsigned limit)
{
    double u = u0 * exp2(noise / 16.0);

    tb->least = (limit + 1) * 16;
    for (unsigned d = 0; d < ARRAY_SIZE(tb->by_level); d++) {
        tb->by_level[d] = 1.0 / (u * sqrt(MAX(d, 1) / 2048.0));
    }
    for (unsigned i = 0; i < ARRAY_SIZE(tb->share); i++) {
        unsigned n = 3;

        for (unsigned j = 0; j < ARRAY_SIZE(t20_temper_alpha_at); j++) {
            if (i / 16.0 >= t20_temper_alpha_at[j].k) {
                n = t20_temper_alpha_at[j].n;
            }
        }
        tb->share[i] = n;
    }
}

/*
 * The count a change of k u restarts at: 0 for none, 16 for a full one,
 * which beats the count 15 it is stored as
 */
static unsigned t20_temper_restart(float k)
{
    if (k < 1) {
        return 0;
    }
    return k >= T20_TEMPER_FULL ? 16 : 4 + 2 * MIN((unsigned)k, 5);
}

static uint32_t t20_temper_alpha(const T20TemperTables *tb, unsigned d,
                                 int cur)
{
    unsigned k = MIN((unsigned)(d * tb->by_level[cur] * 16),
                     ARRAY_SIZE(tb->share) - 1);

    return MAX(tb->share[k] * 16, tb->least);
}

/*
 * One pixel: its word, the frame's value, the largest changes in its
 * neighbourhood (T20_TEMPER_RX, _RY) from the last frame, the least of that
 * and from the filtered value, and from the filtered value; the new word
 */
static uint32_t t20_temper_pixel(const T20TemperTables *tb,
                                 const uint32_t *share, uint32_t old,
                                 int cur, bool on, unsigned nh, unsigned nl,
                                 unsigned nr)
{
    static const uint8_t step[16] = {
        2, 2, 3, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
    };
    int prev = extract32(old, 16, 12), ref = extract32(old, 0, 12);
    unsigned th = old >> 28, tl = extract32(old, 12, 4);
    unsigned dr = abs(cur - ref), c;
    float per_u = tb->by_level[prev];
    unsigned rh = t20_temper_restart(nh * per_u);
    unsigned rl = t20_temper_restart(nl * per_u);

    th = rh > th ? MIN(rh, 15) : step[th];
    if (rl > tl) {
        tl = MIN(rl, 15);
        c = t20_temper_alpha(tb, nr, cur);
    } else {
        tl = step[tl];
        c = share[tl];
    }
    if (on) {
        int q = (dr * c + 127) >> 8;

        ref += cur < ref ? -q : q;
    } else {
        ref = cur;
    }
    return th << 28 | cur << 16 | tl << 12 | ref;
}

/*
 * Temper goes through every pixel of a full-size frame at the sensor's
 * rate: the loops of its window are built to be vectorised. clang
 * vectorises at -O2 and has no optimize attribute.
 */
#ifdef __clang__
#define TEMPER_VECTOR
#else
#define TEMPER_VECTOR       __attribute__((optimize("O3")))
#endif

/* @o: the largest of the three rows of @a, @b and @c, @n entries */
static void TEMPER_VECTOR t20_temper_rows(uint16_t *restrict o,
                                          const uint16_t *restrict a,
                                          const uint16_t *restrict b,
                                          const uint16_t *restrict c,
                                          uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        o[i] = MAX(MAX(a[i], b[i]), c[i]);
    }
}

/* @o: the largest of the 7 entries around each of @n of @p (padded by 3) */
static void TEMPER_VECTOR t20_temper_cols(uint16_t *restrict o,
                                          const uint16_t *restrict p,
                                          uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint16_t m = MAX(MAX(p[i], p[i + 1]), MAX(p[i + 2], p[i + 3]));

        o[i] = MAX(MAX(m, p[i + 4]), MAX(p[i + 5], p[i + 6]));
    }
}

/* Temper runs: armed, its DMA on, not bypassed */
static bool t20_isp_temper_runs(IngenicT20IspState *s, uint32_t w, uint32_t h)
{
    return R(TEMPER_BANK) && (R(TEMPER_DMA) & 1) &&
           !(R(CORE_BYPASS) & BYPASS_TEMPER) && w >= 2 && h >= 2 &&
           R(TEMPER_LINE) >= w * 4;
}

/* The frame stitch writes: not bypassed, its DMA and its writes on */
static bool t20_isp_stitch_runs(IngenicT20IspState *s, uint32_t w, uint32_t h)
{
    return R(STITCH_BANK) && (R(STITCH_DMA) & 1) &&
           (R(STITCH_CTRL) & BIT(16)) && !(R(CORE_BYPASS) & BYPASS_STITCH) &&
           w && h && R(STITCH_LINE) >= w * 4;
}

/*
 * The frame stitch, which no sensor here has WDR to feed, keeps the frame
 * it is given for the next to meet: a word a pixel, the 12-bit sample
 * left-justified in the upper 16 bits (a T10L's, 0x40 bit 7 clear with its
 * writes on, 0xa68 bit 0)
 */
static void t20_isp_stitch(IngenicT20IspState *s, const uint16_t *raw,
                           uint32_t w, uint32_t h)
{
    uint32_t sw = MIN(w, R(STITCH_WIDTH) & 0xffff);
    uint32_t sh = MIN(h, R(STITCH_HEIGHT) & 0xffff);
    g_autofree uint32_t *row = g_new(uint32_t, w);

    for (uint32_t y = 0; y < sh; y++) {
        for (uint32_t x = 0; x < sw; x++) {
            row[x] = cpu_to_le32(raw[(size_t)y * w + x] << 20);
        }
        address_space_write(&address_space_memory,
                            R(STITCH_BANK) + (hwaddr)y * R(STITCH_LINE),
                            MEMTXATTRS_UNSPECIFIED, row, sw * 4);
    }
}

/*
 * Pixels are handled in blocks of this many columns: a block whose changes
 * and whose neighbours' (a window reaches T20_TEMPER_RX columns, one row)
 * are all zero is not windowed
 */
#define T20_TEMPER_BLOCK    32

static uint32_t t20_temper_blocks(uint32_t w)
{
    return DIV_ROUND_UP(w, T20_TEMPER_BLOCK);
}

/*
 * The changes of row @y, and in @act[] whether each block of it has any.
 * No change from the last frame nor from the filtered value is no change at
 * all (the least of them is below both).
 */
static void TEMPER_VECTOR t20_temper_row_changes(uint16_t *restrict nh,
                                                 uint16_t *restrict nl,
                                                 uint16_t *restrict nr,
                                                 const uint32_t *restrict words,
                                                 const uint16_t *restrict raw,
                                                 uint8_t *restrict act,
                                                 uint32_t w)
{
    for (uint32_t b = 0; b < t20_temper_blocks(w); b++) {
        uint32_t x0 = b * T20_TEMPER_BLOCK;
        uint32_t x1 = MIN(x0 + T20_TEMPER_BLOCK, w);
        unsigned any = 0;

        for (uint32_t i = x0; i < x1; i++) {
            uint32_t old = le32_to_cpu(words[i]);
            int cur = raw[i], prev = (old >> 16) & 0xfff, ref = old & 0xfff;
            unsigned df = abs(cur - prev), dr = abs(cur - ref);

            nh[i] = df;
            nl[i] = MIN(df, dr);
            nr[i] = dr;
            any |= df | dr;
        }
        act[b] = any != 0;
    }
}

/*
 * The pixels x0 to x1 of row @y, whose windows may not be all zero: @row
 * holds 6 rows of w + 2 T20_TEMPER_RX, the first 3 the planes' largest
 * changes of 3 rows (offset by T20_TEMPER_RX), the last 3 the windows'
 */
static void t20_temper_span(const T20TemperTables *tb, const uint32_t *share,
                            uint32_t *words, const uint16_t *raw,
                            uint16_t *const *plane, uint16_t *row,
                            int16_t *delta, uint32_t w, uint32_t h,
                            uint32_t y, uint32_t x0, uint32_t x1, bool on)
{
    uint32_t pw = w + 2 * T20_TEMPER_RX;
    uint32_t c0 = MAX(x0, T20_TEMPER_RX) - T20_TEMPER_RX;
    uint32_t c1 = MIN(x1 + T20_TEMPER_RX, w);

    for (unsigned k = 0; k < 3; k++) {
        const uint16_t *p = plane[k];
        uint16_t *pad = row + k * pw;
        uint16_t *o = pad + T20_TEMPER_RX;

        t20_temper_rows(o + c0, p + (size_t)MAX(y, 1) * w - w + c0,
                        p + (size_t)y * w + c0,
                        p + (size_t)MIN(y + 1, h - 1) * w + c0, c1 - c0);
        for (int i = 0; c0 == 0 && i < T20_TEMPER_RX; i++) {
            pad[i] = o[0];
        }
        for (int i = 0; c1 == w && i < T20_TEMPER_RX; i++) {
            pad[T20_TEMPER_RX + w + i] = o[w - 1];
        }
        t20_temper_cols(row + (3 + k) * pw + T20_TEMPER_RX + x0, pad + x0,
                        x1 - x0);
    }
    for (uint32_t x = x0; x < x1; x++) {
        size_t i = (size_t)y * w + x;
        uint32_t new = t20_temper_pixel(tb, share, le32_to_cpu(words[i]),
                                        raw[i], on,
                                        row[3 * pw + T20_TEMPER_RX + x],
                                        row[4 * pw + T20_TEMPER_RX + x],
                                        row[5 * pw + T20_TEMPER_RX + x]);

        delta[i] = t20_isp_temper_level[extract32(new, 0, 12)] -
                   t20_isp_temper_level[extract32(new, 16, 12)];
        words[i] = cpu_to_le32(new);
    }
}

/*
 * One frame of Temper over @words (w x h, as the DDR holds them), from the
 * frame @raw: the new words, and @delta, the filtered value less the
 * frame's in levels. @chg: 3 w x h planes of changes, then 6 rows of
 * w + 2 T20_TEMPER_RX, and the blocks' flags, h of t20_temper_blocks(w).
 */
static void t20_temper_frame(const T20TemperTables *tb, const uint32_t *share,
                             uint32_t *words, const uint16_t *raw,
                             uint16_t *chg, int16_t *delta, uint32_t w,
                             uint32_t h, bool on)
{
    static const uint8_t step[16] = {
        2, 2, 3, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
    };
    size_t n = (size_t)w * h;
    uint32_t nb = t20_temper_blocks(w), pw = w + 2 * T20_TEMPER_RX;
    uint16_t *plane[3] = { chg, chg + n, chg + 2 * n };
    uint16_t *row = chg + 3 * n;
    uint8_t *act = (uint8_t *)(row + 6 * pw);
    g_autofree uint8_t *run = g_new(uint8_t, 2 * nb);
    uint8_t *nearby = run + nb;

    for (uint32_t y = 0; y < h; y++) {
        size_t o = (size_t)y * w;

        t20_temper_row_changes(plane[0] + o, plane[1] + o, plane[2] + o,
                               words + o, raw + o, act + (size_t)y * nb, w);
    }
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *a0 = act + (size_t)MAX(y, 1) * nb - nb;
        const uint8_t *a1 = act + (size_t)y * nb;
        const uint8_t *a2 = act + (size_t)MIN(y + 1, h - 1) * nb;

        for (uint32_t b = 0; b < nb; b++) {
            run[b] = a0[b] | a1[b] | a2[b];
        }
        for (uint32_t b = 0; b < nb; b++) {
            nearby[b] = run[b] || (b && run[b - 1]) ||
                      (b + 1 < nb && run[b + 1]);
        }
        for (uint32_t b = 0; b < nb;) {
            uint32_t e = b;
            uint32_t x0 = b * T20_TEMPER_BLOCK, x1;

            while (e < nb && nearby[e] == nearby[b]) {
                e++;
            }
            x1 = MIN(e * T20_TEMPER_BLOCK, w);
            b = e;
            if (!nearby[e - 1]) {
                for (uint32_t x = x0; x < x1; x++) {
                    size_t i = (size_t)y * w + x;
                    uint32_t old = le32_to_cpu(words[i]);
                    unsigned cur = raw[i];

                    words[i] = cpu_to_le32(step[old >> 28] << 28 | cur << 16 |
                                           step[extract32(old, 12, 4)] << 12 |
                                           cur);
                    delta[i] = 0;
                }
                continue;
            }
            t20_temper_span(tb, share, words, raw, plane, row, delta, w, h, y,
                            x0, x1, on);
        }
    }
}

static void t20_isp_temper(IngenicT20IspState *s, const uint16_t *raw,
                           uint32_t w, uint32_t h)
{
    uint32_t bank = R(TEMPER_BANK), line = R(TEMPER_LINE);
    bool on = R(CORE_TEMPER) & TEMPER_ON;
    size_t n = (size_t)w * h;
    size_t chg = 3 * n + 6 * (w + 2 * T20_TEMPER_RX) +
                 (h * t20_temper_blocks(w) + 1) / 2;
    g_autofree T20TemperTables *tb = NULL;
    unsigned limit = extract32(R(CORE_TEMPER_LIMIT), 0, 4);
    uint32_t share[16];
    uint32_t *words;

    if (s->temper_cap != n || s->temper_chg_cap != chg) {
        g_free(s->temper_words);
        g_free(s->temper_chg);
        g_free(s->temper_buf);
        s->temper_words = g_new(uint32_t, n);
        s->temper_chg = g_new(uint16_t, chg);
        s->temper_buf = g_new(int16_t, n);
        s->temper_cap = n;
        s->temper_chg_cap = chg;
    }
    words = s->temper_words;
    s->temper_delta = s->temper_buf;
    s->temper_w = w;
    s->temper_h = h;
    tb = g_new(T20TemperTables, 1);
    t20_temper_tables(tb, INGENIC_T20_ISP_GET_CLASS(s)->temper_u0,
                      extract32(R(CORE_TEMPER_THRESH), 0, 8), limit);
    for (unsigned t = 0; t < 16; t++) {
        share[t] = MAX(lround(256 / (1.0667 + 0.39 * (15 - t))),
                       (limit + 1) * 16);
    }
    for (uint32_t y = 0; y < h; y++) {
        address_space_read(&address_space_memory, bank + (hwaddr)y * line,
                           MEMTXATTRS_UNSPECIFIED, words + (size_t)y * w,
                           w * 4);
    }
    t20_temper_frame(tb, share, s->temper_words, raw, s->temper_chg,
                     s->temper_delta, w, h, on);
    for (uint32_t y = 0; y < h; y++) {
        address_space_write(&address_space_memory, bank + (hwaddr)y * line,
                            MEMTXATTRS_UNSPECIFIED, words + (size_t)y * w,
                            w * 4);
    }
}

/* The input frame's 12-bit samples to Temper and the frame stitch */
static void t20_isp_input(IngenicT20IspState *s, ImageSensorState *sensor)
{
    uint32_t w = R(CORE_WIDTH) & 0xffff, h = R(CORE_HEIGHT) & 0xffff;
    bool temper = t20_isp_temper_runs(s, w, h);
    bool stitch = t20_isp_stitch_runs(s, w, h);
    g_autofree uint16_t *raw = NULL;

    s->temper_delta = NULL;
    if (!temper && !stitch) {
        return;
    }
    raw = g_new(uint16_t, (size_t)w * h);
    t20_isp_temper_frame(s, sensor, w, h, raw);
    if (stitch) {
        t20_isp_stitch(s, raw, w, h);
    }
    if (temper) {
        t20_isp_temper(s, raw, w, h);
    }
}

static void t20_isp_stats(IngenicT20IspState *s, ImageSensorState *sensor)
{
    uint32_t qw = (R(CORE_WIDTH) & 0xffff) / 2;
    uint32_t qh = (R(CORE_HEIGHT) & 0xffff) / 2;
    bool ae = (R(CORE_AE_SWITCH) & 7) != 7;
    bool awb = (R(CORE_AWB_SWITCH) & 3) != 2;
    bool af = (R(CORE_AF_SWITCH) & 3) < 2;
    bool mve = (R(CORE_MVE_SWITCH) & 3) != MVE_OFF &&
               (R(MVE_DUMP) & (MVE_DUMP_ON | MVE_DUMP_RESTART));
    unsigned ac = MIN(extract32(R(MET_AE_NODES), 0, 8), STATS_GRID);
    unsigned ar = MIN(extract32(R(MET_AE_NODES), 8, 8), STATS_GRID);
    unsigned wc = MIN(extract32(R(MET_AWB_NODES), 0, 8), STATS_GRID);
    unsigned wr = MIN(extract32(R(MET_AWB_NODES), 8, 8), STATS_GRID);
    uint32_t white = R(MET_AWB_WHITE) & 0x3ff;
    uint32_t black = R(MET_AWB_BLACK) & 0x3ff;
    uint32_t gain[3] = {
        t20_isp_wb_gain(s, 0), t20_isp_wb_gain(s, 1), t20_isp_wb_gain(s, 2),
    };
    /* the response on 16 fractional bits */
    uint64_t resp[3];
    double rf[3];
    g_autofree uint8_t *pic = NULL, *acol = NULL, *arow = NULL;
    g_autofree uint8_t *apic = NULL, *gpic = NULL;
    const uint8_t *epic;
    double wf[3];
    g_autofree uint8_t *wcol = NULL, *wrow = NULL;
    uint64_t hist[256] = { 0 }, total = 0, five[5] = { 0 };
    T20IspAwbSums zone[STATS_GRID * STATS_GRID] = { 0 }, all = { 0 };
    uint32_t zbins[STATS_GRID * STATS_GRID][5] = { 0 };
    uint32_t ztotal[STATS_GRID * STATS_GRID] = { 0 };
    uint8_t bin[256];
    uint32_t counted = 0;

    if ((!ae && !awb && !af && !mve) || !qw || !qh) {
        return;
    }
    pic = g_malloc(qw * qh * 3 / 2);
    image_sensor_render(sensor, s->frame, qw, qh, pic);
    if (awb && image_sensor_tinted(sensor) &&
        image_sensor_raw_response(sensor, IMAGE_SENSOR_PART_APICAL, rf)) {
        /* the light's tint rounded into the 8-bit chroma biases a dim grey */
        apic = g_malloc(qw * qh * 3 / 2);
        image_sensor_render_untinted(sensor, s->frame, qw, qh, apic,
                                     IMAGE_SENSOR_FUSED);
    } else {
        rf[0] = IMAGE_SENSOR_RAW_APICAL_R / 256.0;
        rf[1] = 1;
        rf[2] = IMAGE_SENSOR_RAW_APICAL_B / 256.0;
        image_sensor_raw_factors(sensor, IMAGE_SENSOR_PART_APICAL, rf);
    }
    for (unsigned c = 0; c < 3; c++) {
        resp[c] = llround(rf[c] * 65536);
    }
    /*
     * The AE histograms and the AF see the picture after the static white
     * balance (a T10L's AE histogram bins and AF intensity follow the gains
     * held on a flat test pattern); the AWB sums take the gains themselves
     * and the MVE's tap is not known
     */
    epic = pic;
    if ((ae || af) && ingenic_t20_isp_wb_factors(s, sensor, wf)) {
        gpic = g_memdup2(pic, (size_t)qw * qh * 3 / 2);
        ingenic_isp_wb_picture(gpic, gpic + (size_t)qw * qh, qw, qh, wf);
        epic = gpic;
    }
    if (af) {
        t20_isp_af(s, epic, qw, qh);
    }
    if (mve) {
        t20_isp_mve(s, pic, qw, qh);
    }
    if (!ae && !awb) {
        return;
    }
    acol = g_malloc(qw);
    arow = g_malloc(qh);
    wcol = g_malloc(qw);
    wrow = g_malloc(qh);
    t20_isp_grid(acol, qw, ac);
    t20_isp_grid(arow, qh, ar);
    t20_isp_grid(wcol, qw, wc);
    t20_isp_grid(wrow, qh, wr);
    /* the 5-bin histograms' bin of each value: under which threshold */
    for (unsigned i = 0; i < 256; i++) {
        bin[i] = 0;
        while (bin[i] < 4 &&
               i >= (R(MET_HIST_THRESH + bin[i] * 4) & 0xff)) {
            bin[i]++;
        }
    }
    for (uint32_t y = 0; y < qh; y++) {
        const uint8_t *row = pic + y * qw;
        const uint8_t *uv = pic + qw * qh + y / 2 * qw;
        const uint8_t *erow = epic + y * qw;
        const uint8_t *lrow = apic ? apic + y * qw : row;
        const uint8_t *auv = apic ? apic + qw * qh + y / 2 * qw : uv;

        for (uint32_t x = 0; x < qw; x++) {
            /* weight 15 counts 16; each quad's 2 pixels count */
            unsigned at = CORE_AE_WEIGHT + arow[y] * STATS_GRID + acol[x];
            uint32_t w = extract32(R(at & ~3), (at & 3) * 8, 4);
            int u = auv[x & ~1] - 128, v = auv[x | 1] - 128;
            int c = (lrow[x] - 16) * 298 + 128;
            /* Limited-range BT.601, as the scene is rendered */
            int rgb[3] = {
                (c + 409 * v) >> 8, (c - 100 * u - 208 * v) >> 8,
                (c + 516 * u) >> 8,
            };
            uint32_t ch[3], fine[3], lo = 0x3ff, hi = 0;
            unsigned zi = wrow[y] * STATS_GRID + wcol[x];
            T20IspAwbSums *z = &zone[zi];
            uint32_t ww;

            if (ae) {
                unsigned az = arow[y] * ac + acol[x];

                zbins[az][bin[t20_isp_linear[erow[x]]]] += 4;
                ztotal[az] += 4;
            }
            if (ae && w) {
                hist[t20_isp_linear[erow[x]]] += (w == 15 ? 16 : w) * 2;
                counted += 2;
            }
            if (!awb) {
                continue;
            }
            /*
             * 10 bits, in linear light, through the guest's gains; the sums
             * keep 1/64th of a level, which the zone's quads average out
             */
            for (unsigned i = 0; i < 3; i++) {
                uint64_t l = t20_isp_linear_fine[MIN(MAX(rgb[i], 0), 255)];

                fine[i] = MIN(l * resp[i] / 65536 * gain[i] / 256,
                              0x3ff * 64 + 63);
                ch[i] = fine[i] >> 6;
                lo = MIN(lo, ch[i]);
                hi = MAX(hi, ch[i]);
            }
            if (!ch[1] || lo < black || hi > white ||
                ch[0] * 256 / ch[1] < (R(MET_CR_MIN) & 0xfff) ||
                ch[0] * 256 / ch[1] > (R(MET_CR_MAX) & 0xfff) ||
                ch[2] * 256 / ch[1] < (R(MET_CB_MIN) & 0xfff) ||
                ch[2] * 256 / ch[1] > (R(MET_CB_MAX) & 0xfff)) {
                continue;
            }
            /* the whole picture's by the zone's weight, 15 counting 16 */
            ww = extract32(R((CORE_AWB_WEIGHT + zi) & ~3),
                           ((CORE_AWB_WEIGHT + zi) & 3) * 8, 4);
            ww = ww == 15 ? 16 : ww;
            for (unsigned i = 0; i < 3; i++) {
                z->rgb[i] += fine[i] * 4;
                all.rgb[i] += fine[i] * 4 * ww;
            }
            z->n += 4;
            all.n += 4 * ww;
        }
    }
    if (ae) {
        for (unsigned i = 0; i < 256; i++) {
            five[bin[i]] += hist[i];
            total += hist[i];
            R(HIST_MEM + i * 4) = t20_isp_float(hist[i]);
        }
        /* each zone's, unweighted, in 1/65536ths of its pixels */
        for (unsigned z = 0; z < ar * ac; z++) {
            uint32_t v[5];

            for (unsigned k = 0; k < 5; k++) {
                v[k] = ztotal[z] ?
                    MIN((uint64_t)zbins[z][k] * 65536 / ztotal[z], 0xffff) : 0;
            }
            R(MET_AE_ZONES + z * 8) = v[0] | v[1] << 16;
            R(MET_AE_ZONES + z * 8 + 4) = v[3] | v[4] << 16;
        }
        for (unsigned k = 0, r = 0; k < 5; k++) {
            if (k != 2) {
                R(MET_HIST + r++ * 4) = total ? five[k] * 0xffff / total : 0;
            }
        }
        R(MET_TOTAL) = qw * qh * 2;
        R(MET_COUNTED) = counted;
    }
    if (awb) {
        for (unsigned z = 0; z < wr * wc; z++) {
            const T20IspAwbSums *zs = &zone[z / wc * STATS_GRID + z % wc];

            R(MET_AWB_ZONES + z * 8) = t20_isp_awb_ratios(s, zs);
            R(MET_AWB_ZONES + z * 8 + 4) = zs->n;
        }
        R(MET_AWB_RG) = t20_isp_awb_ratios(s, &all) & 0xfff;
        R(MET_AWB_BG) = t20_isp_awb_ratios(s, &all) >> 16;
        R(MET_AWB_SUM) = all.n / 16;
    }
}

/* The frames, in thousandths, that pass at @load: straight between rows */
uint32_t ingenic_t20_isp_keep_pm(const IngenicT20IspKeep *rows, unsigned n,
                                 uint32_t load)
{
    if (!n || load < rows[0].load_pm) {
        return 1000;
    }
    for (unsigned i = 1; i < n; i++) {
        const IngenicT20IspKeep *lo = &rows[i - 1], *hi = &rows[i];

        if (load <= hi->load_pm) {
            return lo->keep_pm + ((int64_t)hi->keep_pm - lo->keep_pm) *
                   (load - lo->load_pm) / (hi->load_pm - lo->load_pm);
        }
    }
    return rows[n - 1].keep_pm;
}

/*
 * Whether the core keeps up with the frame about to start: false for one it
 * drops, silently, with no status raised (a T30X shows no interrupt at all
 * while it drops everything). The load is the active width times the line
 * rate over the clock; the sensor's frame rate does not enter, so no row is
 * kept for one.
 */
static bool t20_isp_keeps(IngenicT20IspState *s, ImageSensorState *sensor)
{
    IngenicT20IspClass *c = INGENIC_T20_ISP_GET_CLASS(s);
    uint64_t hz = s->cpm && c->ispcdr ? ingenic_cgu_hz(s->cpm, c->ispcdr) : 0;
    uint32_t rate = image_sensor_line_rate(sensor, IMAGE_SENSOR_PART_APICAL);

    if (!c->keeps || !hz || !rate) {
        return true;
    }
    s->keep_acc += ingenic_t20_isp_keep_pm(c->keep, c->keeps,
        (uint64_t)(R(CORE_WIDTH) & 0xffff) * rate * 1000 / hz);
    if (s->keep_acc < 1000) {
        return false;
    }
    s->keep_acc -= 1000;
    return true;
}

static void t20_isp_frame(void *opaque)
{
    IngenicT20IspState *s = opaque;
    ImageSensorState *sensor = image_sensor_find(0);
    int64_t half;

    if (!s->running || !sensor) {
        return;
    }
    half = NANOSECONDS_PER_SECOND / MAX(sensor->fps, 1) / 2;
    if (!s->mid_frame && !t20_isp_keeps(s, sensor)) {
        trace_ingenic_t20_isp_frame(s->frame, "dropped");
        timer_mod(s->frame_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 2 * half);
        return;
    }
    if (!s->mid_frame) {
        s->frame++;
        trace_ingenic_t20_isp_frame(s->frame, "starts");
        qemu_irq_pulse(s->frame_start);
        t20_isp_event(s, EV_FRAME_START);
    } else {
        trace_ingenic_t20_isp_frame(s->frame, "out");
        qemu_irq_pulse(s->frame_out);
        t20_isp_input(s, sensor);
        for (unsigned n = 0; n < INGENIC_T20_ISP_WRITERS; n += 2) {
            t20_isp_channel(s, sensor, n);
        }
        t20_isp_event(s, EV_FRAME_END);
        t20_isp_event(s, output_end[0]);
        t20_isp_event(s, output_end[1]);
        t20_isp_stats(s, sensor);
        t20_isp_event(s, EV_AE);
        t20_isp_event(s, EV_AWB);
        s->vic_status |= TOP_VIC_DONE;
    }
    s->mid_frame = !s->mid_frame;
    t20_isp_update(s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + half);
}

static uint32_t t20_isp_reg_read(IngenicT20IspState *s, hwaddr off)
{
    hwaddr reg;
    int n;

    if (off == INGENIC_T20_ISP_GET_CLASS(s)->vic_ctrl) {
        return 0;
    }
    switch (off) {
    case TOP_STA:
        return (s->status & 0xffff) | s->vic_status;
    case CORE_INT_STA:
        return s->status;
    case CORE_PORT_STA:
        switch (R(CORE_PORT_REQ) & 7) {
        case 1:
        case 3:
        case 5:
            return 1;
        }
        return 0;
    }
    n = t20_isp_writer(off, &reg);
    if (n >= 0) {
        IngenicT20IspWriter *w = &s->writer[n];

        switch (reg) {
        case WR_CTRL:
            return (R(off) & 3) | w->curr << 8 | w->last << 11;
        case WR_BASE_CURR:
            return w->base_curr;
        case WR_BASE_LAST:
            return w->base_last;
        case WR_COUNT:
            return (w->frames & 0xffff) * 0x10001;
        }
    }
    return R(off);
}

static uint64_t ingenic_t20_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(opaque);
    uint32_t v = t20_isp_reg_read(s, off);
    char name[T20_ISP_NAME_LEN];

    if (trace_event_get_state_backends(TRACE_INGENIC_T20_ISP_READ)) {
        trace_ingenic_t20_isp_read(off, v, t20_isp_pc(),
                                   t20_isp_reg_name(s, off, name));
    }
    return v;
}

/*
 * The bits of the frame stitch's registers (0x240, the WDR mode word, and
 * 0xa40 to 0xa6c, its frame buffer's: the Temper frame buffer's, at 0xa00,
 * are the same block) that hold what is written, as a T20X's and a T10L's
 * do when written all ones: the format and plane in 0xa40 bits 7:0 (the
 * two cancels in bits 25:24 and the write and read enables in 17:16), the
 * sizes 16 bits, the DMA enable 0xa68 bit 0, and 0xa44, 0xa4c, 0xa60 and
 * 0xa6c nothing
 */
static uint32_t t20_isp_write_mask(hwaddr off)
{
    switch (off - CORE) {
    case 0x240:
        return 0x0003e0e3;
    case 0xa40:
        return 0x030300ff;
    case 0xa44:
    case 0xa4c:
    case 0xa60:
    case 0xa6c:
        return 0;
    case 0xa50:
    case 0xa54:
        return 0xffff;
    case 0xa68:
        return 1;
    }
    return ~0;
}

static void ingenic_t20_isp_write(void *opaque, hwaddr off, uint64_t value,
                                  unsigned size)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(opaque);
    uint32_t v = value, old;
    char name[T20_ISP_NAME_LEN];

    if (trace_event_get_state_backends(TRACE_INGENIC_T20_ISP_WRITE)) {
        trace_ingenic_t20_isp_write(off, v, t20_isp_pc(),
                                    t20_isp_reg_name(s, off, name));
    }
    if (off == INGENIC_T20_ISP_GET_CLASS(s)->vic_ctrl) {
        if (v & VIC_STOP) {
            if (s->running) {
                trace_ingenic_t20_isp_vic("stop");
            }
            s->running = false;
            s->mid_frame = false;
            s->keep_acc = 0;
            timer_del(s->frame_timer);
        } else if (v & VIC_START && !s->running) {
            trace_ingenic_t20_isp_vic(image_sensor_find(0) ? "start"
                                      : "start, no sensor");
            s->running = true;
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        return;
    }
    switch (off) {
    case TOP_CLR:
        s->vic_status &= ~v;
        t20_isp_update(s);
        return;
    case TOP_CLR_ALL:
        s->vic_status = 0;
        t20_isp_update(s);
        return;
    case TOP_STA:
    case CORE_INT_STA:
    case CORE_PORT_STA:
        return;
    case TOP_ENABLE:
    case TOP_MASK:
        R(off) = v;
        t20_isp_update(s);
        return;
    case CORE_INT_CLR:
        old = R(off);
        R(off) = v;
        s->status &= ~(v & ~old & 0xffff);
        t20_isp_update(s);
        return;
    }
    R(off) = v & t20_isp_write_mask(off);
}

static const MemoryRegionOps ingenic_t20_isp_ops = {
    .read = ingenic_t20_isp_read,
    .write = ingenic_t20_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t20_isp_reset_hold(Object *obj, ResetType type)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(obj);

    timer_del(s->frame_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->writer, 0, sizeof(s->writer));
    s->temper_delta = NULL;
    s->status = 0;
    s->vic_status = 0;
    s->frame = 0;
    s->running = false;
    s->mid_frame = false;
}

static void ingenic_t20_isp_reset_exit(Object *obj, ResetType type)
{
    t20_isp_update(INGENIC_T20_ISP(obj));
}

static void ingenic_t20_isp_init(Object *obj)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t20_isp_ops, s,
                          TYPE_INGENIC_T20_ISP, INGENIC_T20_ISP_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->frame_start, "frame-start", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->frame_out, "frame-out", 1);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, t20_isp_frame, s);
}

static const VMStateDescription vmstate_ingenic_t20_isp_writer = {
    .name = TYPE_INGENIC_T20_ISP "/writer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(curr, IngenicT20IspWriter),
        VMSTATE_UINT32(last, IngenicT20IspWriter),
        VMSTATE_UINT32(base_curr, IngenicT20IspWriter),
        VMSTATE_UINT32(base_last, IngenicT20IspWriter),
        VMSTATE_UINT32(frames, IngenicT20IspWriter),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_t20_isp = {
    .name = TYPE_INGENIC_T20_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT20IspState,
                             INGENIC_T20_ISP_IOSIZE / 4),
        VMSTATE_STRUCT_ARRAY(writer, IngenicT20IspState,
                             INGENIC_T20_ISP_WRITERS, 1,
                             vmstate_ingenic_t20_isp_writer,
                             IngenicT20IspWriter),
        VMSTATE_UINT32(status, IngenicT20IspState),
        VMSTATE_UINT32(vic_status, IngenicT20IspState),
        VMSTATE_UINT32(frame, IngenicT20IspState),
        VMSTATE_BOOL(running, IngenicT20IspState),
        VMSTATE_BOOL(mid_frame, IngenicT20IspState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicT20IspState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_t20_isp_properties[] = {
    DEFINE_PROP_LINK("cpm", IngenicT20IspState, cpm, TYPE_INGENIC_CGU,
                     Object *),
};

/*
 * A T20X (Wyze V2, jxf23 1080p at 25 fps, 33750 lines a second), frames
 * counted for 10 s per clock: at 71.4, 66.7 and 62.5 MHz (loads 0.907, 0.972
 * and 1.037) it kept 73 %, 84 % and 52 % of them, all of them at 76.9 MHz
 * (0.843). The rows are those, not monotonic as measured; the load where it
 * starts dropping (0.88) is chosen between the first two. The clock floors
 * at 62.5 MHz. The T10 shares the core and takes the same rows, unmeasured.
 */
static const IngenicT20IspKeep ingenic_t20_isp_keep[] = {
    { 880, 1000 }, { 907, 730 }, { 972, 840 }, { 1037, 510 },
};

/*
 * A T30X (Wyze VDB1, sc4236 at 2048x1536, 48000 lines a second) with every
 * frame lost, silently, once the load passes 0.98: it streamed to 0.885
 * (2048 and 2304 px at 111 and 125 MHz) and stalled from 0.983 (100 and
 * 111 MHz). No clock of its divider fell between, so the half-rate band
 * that a T40 shows between 0.90 and 0.98 is assumed here; the limits are
 * those two fitted across both parts.
 */
static const IngenicT20IspKeep ingenic_t30_isp_keep[] = {
    { 900, 1000 }, { 901, 500 }, { 980, 500 }, { 981, 0 },
};

static void ingenic_t20_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic T20 ISP and VIC";
    dc->vmsd = &vmstate_ingenic_t20_isp;
    device_class_set_props(dc, ingenic_t20_isp_properties);
    INGENIC_T20_ISP_CLASS(oc)->ispcdr = 0x80;
    INGENIC_T20_ISP_CLASS(oc)->keep = ingenic_t20_isp_keep;
    INGENIC_T20_ISP_CLASS(oc)->keeps = ARRAY_SIZE(ingenic_t20_isp_keep);
    rc->phases.hold = ingenic_t20_isp_reset_hold;
    rc->phases.exit = ingenic_t20_isp_reset_exit;
    INGENIC_T20_ISP_CLASS(oc)->vic_ctrl = 0x00;
    INGENIC_T20_ISP_CLASS(oc)->temper_u0 = 0.5594;
    for (unsigned i = 0; i < 256; i++) {
        t20_isp_linear[i] = lrint(255.0 * pow(i / 255.0, 2.2));
        t20_isp_linear_fine[i] = lrint(255.0 * 4 * 64 * pow(i / 255.0, 2.2));
    }
    /* the tone curve, full range, in 16ths of a level, and its inverse */
    for (unsigned i = 0; i < ARRAY_SIZE(t20_isp_temper_level); i++) {
        double at = i / 128.0, y;
        unsigned k = MIN((unsigned)at, ARRAY_SIZE(t20_isp_tone) - 2);

        y = t20_isp_tone[k] + (t20_isp_tone[k + 1] - t20_isp_tone[k]) *
            (at - k);
        t20_isp_temper_level[i] = lrint(16 * y * 255.0 / 247.0);
    }
    for (unsigned i = 0, l = 0; i < ARRAY_SIZE(t20_isp_temper_raw); i++) {
        while (l < 4095 && t20_isp_temper_level[l] < i * 16) {
            l++;
        }
        t20_isp_temper_raw[i] = l;
    }
}

static void ingenic_t10_isp_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->desc = "Ingenic T10 ISP and VIC";
    INGENIC_T20_ISP_CLASS(oc)->vic_ctrl = 0x20;
    INGENIC_T20_ISP_CLASS(oc)->temper_u0 = 0.614;
}

static void ingenic_t30_isp_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->desc = "Ingenic T30 ISP and VIC";
    INGENIC_T20_ISP_CLASS(oc)->line_buffer = true;
    INGENIC_T20_ISP_CLASS(oc)->raw_blank = BIT(0);
    INGENIC_T20_ISP_CLASS(oc)->keep = ingenic_t30_isp_keep;
    INGENIC_T20_ISP_CLASS(oc)->keeps = ARRAY_SIZE(ingenic_t30_isp_keep);
}

static void ingenic_t20_isp_finalize(Object *obj)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(obj);

    g_free(s->temper_words);
    g_free(s->temper_chg);
    g_free(s->temper_buf);
}

static const TypeInfo ingenic_t20_isp_types[] = {
    {
        .name = TYPE_INGENIC_T20_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicT20IspState),
        .instance_init = ingenic_t20_isp_init,
        .instance_finalize = ingenic_t20_isp_finalize,
        .class_size = sizeof(IngenicT20IspClass),
        .class_init = ingenic_t20_isp_class_init,
    }, {
        .name = TYPE_INGENIC_T10_ISP,
        .parent = TYPE_INGENIC_T20_ISP,
        .class_init = ingenic_t10_isp_class_init,
    }, {
        .name = TYPE_INGENIC_T30_ISP,
        .parent = TYPE_INGENIC_T20_ISP,
        .class_init = ingenic_t30_isp_class_init,
    },
};

DEFINE_TYPES(ingenic_t20_isp_types)
