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
 *   bits 3:0 the format (13 NV12), 7:6 the plane (1 UV, 2 VU), bit 9 on;
 *   +0x4 the size (height << 16 | width); +0x8-0x18 five bank addresses
 *   the driver fills; +0x1c the last bank (bits 2:0) and a restart at bank
 *   0 (bit 3); +0x20 the line offset, signed (vflip); +0x24 bits 1:0
 *   control (2 runs, 3 cancels), reading back the current bank (10:8) and
 *   the last (13:11). At each frame a running writer moves to its next
 *   bank and writes the frame there; the driver learns from +0x24 which
 *   bank finished. There is no FIFO and no frame number.
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
 *     weighted at 0x808cc.
 *   - AWB: per zone of the grid of 0x80870, two words from 0x88740, the
 *     ratios of the sums of its pixels that count, G/R in bits 11:0 and G/B
 *     in 27:16 (R/G and B/G with 0x80868 bit 0 set), 1.0 = 0x100, and their
 *     number; the whole picture's at 0x80858, 0x8085c and 0x80860. A pixel
 *     counts with R/G and B/G (in 1/256ths) within 0x8084c to 0x80848 and
 *     0x80854 to 0x80850 and its channels, 10 bits, from 0x80844 to
 *     0x80840. Its colours are the raw picture's through the static white
 *     balance (0x80300 R, 0x80304 Gr, 0x80308 Gb, 0x8030c B, 0x100 = 1.0,
 *     whichever colour the Bayer order of 0x80018 starts with), in linear
 *     light: the model takes the scene, white-balanced, back to raw (T20_RAW_R,
 *     T20_RAW_B) and applies the guest's gains to it. A switch of 0x80048
 *     to 7 or 0x8004c to 2 turns either off.
 *   The rest of the metering memory (AF and the like) is not modelled.
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
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-t20-isp.h"
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
#define CORE_AE_SWITCH      (CORE + 0x48)
#define CORE_AWB_SWITCH     (CORE + 0x4c)
#define CORE_WB_GAIN        (CORE + 0x300)
#define CORE_AE_WEIGHT      (CORE + 0x1e00)

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
#define MET_TOTAL           (MET + 0xc8)
#define MET_COUNTED         (MET + 0xcc)
#define MET_AWB_ZONES       (CORE + 0x8740) /* in the metering memory */
#define HIST_MEM            (CORE + 0x10000)
#define STATS_GRID          15

/* An Apical writer's registers */
#define WR_CFG              0x00
#define  WR_FORMAT          0xf
#define  WR_NV12            13
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

/*
 * A grey's raw R and B in 1/256ths of its G: the scene is white-balanced,
 * the raw picture as a T20X's (JXF23) in a room lit by a white LED strip,
 * where its AWB settles at gains R 1.98, B 1.80. The JXF22's calibration
 * takes that grey (R/G 0.50, B/G 0.55 through its static white balance:
 * 196, 272) for its light source at 211, 268, so its AWB settles there too.
 */
#define T20_RAW_R           129
#define T20_RAW_B           142

/* An 8-bit gamma-2.2 value in linear light, 8-bit */
static uint8_t t20_isp_linear[256];

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

/* A channel's frame: luma writer @n, chroma writer @n + 1 */
static void t20_isp_channel(IngenicT20IspState *s, ImageSensorState *sensor,
                            unsigned n)
{
    hwaddr y = CORE_WRITER + n * 0x80, uv = y + 0x80;
    uint32_t size = R(y + WR_SIZE);
    uint32_t w = size & 0xfffe, h = (size >> 16) & 0xfffe;
    g_autofree uint8_t *nv12 = NULL;
    bool chroma;

    if (!t20_isp_advance(s, n)) {
        return;
    }
    chroma = t20_isp_advance(s, n + 1);
    if (w && h && (R(y + WR_CFG) & WR_FORMAT) == WR_NV12 &&
        !INGENIC_T20_ISP_GET_CLASS(s)->line_buffer) {
        nv12 = g_malloc(w * h * 3 / 2);
        image_sensor_render(sensor, s->frame, w, h, nv12);
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

typedef struct T20IspAwbSums {
    uint64_t rgb[3];
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

        v[i] = den ? MIN(num * 256 / den, 0xfff) : 0;
    }
    return v[0] | v[1] << 16;
}

/* The AE and AWB statistics of the sensor's frame */
static void t20_isp_stats(IngenicT20IspState *s, ImageSensorState *sensor)
{
    uint32_t qw = (R(CORE_WIDTH) & 0xffff) / 2;
    uint32_t qh = (R(CORE_HEIGHT) & 0xffff) / 2;
    bool ae = (R(CORE_AE_SWITCH) & 7) != 7;
    bool awb = (R(CORE_AWB_SWITCH) & 3) != 2;
    unsigned ac = MIN(extract32(R(MET_AE_NODES), 0, 8), STATS_GRID);
    unsigned ar = MIN(extract32(R(MET_AE_NODES), 8, 8), STATS_GRID);
    unsigned wc = MIN(extract32(R(MET_AWB_NODES), 0, 8), STATS_GRID);
    unsigned wr = MIN(extract32(R(MET_AWB_NODES), 8, 8), STATS_GRID);
    uint32_t white = R(MET_AWB_WHITE) & 0x3ff;
    uint32_t black = R(MET_AWB_BLACK) & 0x3ff;
    uint32_t gain[3] = {
        t20_isp_wb_gain(s, 0), t20_isp_wb_gain(s, 1), t20_isp_wb_gain(s, 2),
    };
    g_autofree uint8_t *pic = NULL, *acol = NULL, *arow = NULL;
    g_autofree uint8_t *wcol = NULL, *wrow = NULL;
    uint64_t hist[256] = { 0 }, total = 0, five[5] = { 0 };
    T20IspAwbSums zone[STATS_GRID * STATS_GRID] = { 0 }, all = { 0 };
    uint32_t counted = 0;

    if ((!ae && !awb) || !qw || !qh) {
        return;
    }
    pic = g_malloc(qw * qh * 3 / 2);
    image_sensor_render(sensor, s->frame, qw, qh, pic);
    acol = g_malloc(qw);
    arow = g_malloc(qh);
    wcol = g_malloc(qw);
    wrow = g_malloc(qh);
    t20_isp_grid(acol, qw, ac);
    t20_isp_grid(arow, qh, ar);
    t20_isp_grid(wcol, qw, wc);
    t20_isp_grid(wrow, qh, wr);
    for (uint32_t y = 0; y < qh; y++) {
        const uint8_t *row = pic + y * qw;
        const uint8_t *uv = pic + qw * qh + y / 2 * qw;

        for (uint32_t x = 0; x < qw; x++) {
            /* weight 15 counts 16; each quad's 2 pixels count */
            unsigned at = CORE_AE_WEIGHT + arow[y] * STATS_GRID + acol[x];
            uint32_t w = extract32(R(at & ~3), (at & 3) * 8, 4);
            int u = uv[x & ~1] - 128, v = uv[x | 1] - 128;
            int c = (row[x] - 16) * 298 + 128;
            /* Limited-range BT.601, as the scene is rendered */
            int rgb[3] = {
                (c + 409 * v) >> 8, (c - 100 * u - 208 * v) >> 8,
                (c + 516 * u) >> 8,
            };
            static const uint32_t raw[3] = { T20_RAW_R, 256, T20_RAW_B };
            uint32_t ch[3], lo = 0x3ff, hi = 0;
            T20IspAwbSums *z = &zone[wrow[y] * STATS_GRID + wcol[x]];

            if (ae && w) {
                hist[t20_isp_linear[row[x]]] += (w == 15 ? 16 : w) * 2;
                counted += 2;
            }
            if (!awb) {
                continue;
            }
            /* 10 bits, in linear light, through the guest's gains */
            for (unsigned i = 0; i < 3; i++) {
                uint64_t l = t20_isp_linear[MIN(MAX(rgb[i], 0), 255)];

                ch[i] = MIN(l * 4 * raw[i] / 256 * gain[i] / 256, 0x3ff);
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
            for (unsigned i = 0; i < 3; i++) {
                z->rgb[i] += ch[i] * 4;
                all.rgb[i] += ch[i] * 4;
            }
            z->n += 4;
            all.n += 4;
        }
    }
    if (ae) {
        for (unsigned i = 0; i < 256; i++) {
            unsigned k = 0;

            while (k < 4 && i >= (R(MET_HIST_THRESH + k * 4) & 0xff)) {
                k++;
            }
            five[k] += hist[i];
            total += hist[i];
            R(HIST_MEM + i * 4) = t20_isp_float(hist[i]);
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
        R(MET_AWB_SUM) = all.n;
    }
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
    if (!s->mid_frame) {
        s->frame++;
        trace_ingenic_t20_isp_frame(s->frame, "starts");
        qemu_irq_pulse(s->frame_start);
        t20_isp_event(s, EV_FRAME_START);
    } else {
        trace_ingenic_t20_isp_frame(s->frame, "out");
        qemu_irq_pulse(s->frame_out);
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
    R(off) = v;
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

static void ingenic_t20_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic T20 ISP and VIC";
    dc->vmsd = &vmstate_ingenic_t20_isp;
    rc->phases.hold = ingenic_t20_isp_reset_hold;
    rc->phases.exit = ingenic_t20_isp_reset_exit;
    INGENIC_T20_ISP_CLASS(oc)->vic_ctrl = 0x00;
    for (unsigned i = 0; i < 256; i++) {
        t20_isp_linear[i] = lrint(255.0 * pow(i / 255.0, 2.2));
    }
}

static void ingenic_t10_isp_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->desc = "Ingenic T10 ISP and VIC";
    INGENIC_T20_ISP_CLASS(oc)->vic_ctrl = 0x20;
}

static void ingenic_t30_isp_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->desc = "Ingenic T30 ISP and VIC";
    INGENIC_T20_ISP_CLASS(oc)->line_buffer = true;
}

static const TypeInfo ingenic_t20_isp_types[] = {
    {
        .name = TYPE_INGENIC_T20_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicT20IspState),
        .instance_init = ingenic_t20_isp_init,
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
