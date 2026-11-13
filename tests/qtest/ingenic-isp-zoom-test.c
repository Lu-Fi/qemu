/*
 * QTest testcase for a T31's channel crop and front crop while streaming
 * and for what the T21's, T23's and T31's MSCA keeps up with, and the T20's,
 * T30's, T32's, T40's and T41's core and the T40's and T41's MSCA
 *
 * A channel scaled to +0x0 puts out the window +0x28 and +0x2c give of it. The
 * front crop window in 0x9860 (x << 16 | y) and 0x9864 (w << 16 | h) takes
 * that part of the frame the ISP takes (its size at +0x4) and the channel's
 * scaled size shows it whole: a window the size of the channel is 1:1, a
 * smaller one is enlarged. Both are read as each frame reaches the MSCA, so a
 * write between two frames shows in the next.
 *
 * The MSCA keeps up with so many scaled pixels a line of the frame (the
 * scaled size over the window's height, or the frame's) at the ISP clock the
 * guest set in the CPM, and so much enlargement of a front crop window; past
 * that every channel stops and the core raises status bits 10:9 each frame. A
 * front crop window smaller than channel 0 with its scaler off (bit 8 of the
 * channel enable) stops it at any load.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define BUF             0x00400000
#define UVBUF           0x00410000
#define CH1             (ISP + 0x9a00)
#define FW              128             /* the frame the ISP takes */
#define FH              64

/* Where a part has what the load tests drive */
typedef struct Part {
    const char *machine;
    uint32_t vic;           /* a write of 1 starts the VIC */
    uint32_t ch_en;         /* the channel enable */
    uint32_t ch;            /* channel 0's registers, 0x100 apart */
    uint32_t size, y, y_stride, uv, uv_stride, done, done_y; /* +, in one */
    uint32_t status;        /* the core's status, +4 clears it */
    uint32_t stall;         /* the bits a stall raises */
    unsigned mpll;          /* MHz */
    uint32_t mpll_reg;      /* CPMPCR for it */
} Part;

static const Part t31 = {
    .machine = "ingenic-t31", .vic = 0xe0000, .ch_en = 0x9804, .ch = 0x9900,
    .size = 0x00, .y = 0x6c, .y_stride = 0x80, .uv = 0x84, .uv_stride = 0x98,
    .done = 0x7c, .done_y = 0x74, .status = 0xb4, .stall = 0x600,
    .mpll = 1200, .mpll_reg = 100u << 20 | 1u << 14 | 2u << 11 | 1u << 8 | 9,
};

static const Part t21 = {
    .machine = "ingenic-t21", .vic = 0x10000, .ch_en = 0x2304, .ch = 0x2400,
    .size = 0x00, .y = 0x6c, .y_stride = 0x80, .uv = 0x84, .uv_stride = 0x98,
    .done = 0x7c, .done_y = 0x74, .status = 0x84, .stall = 0x200,
    /* the T21's PLL register: 24 MHz * 2 * (74 + 1) / (3 + 1) */
    .mpll = 900, .mpll_reg = 74u << 20 | 3u << 14 | 1,
};

static const Part t23 = {
    .machine = "ingenic-t23", .vic = 0xe0000, .ch_en = 0xd040, .ch = 0xd100,
    .size = 0x04, .y = 0x30, .y_stride = 0x40, .uv = 0x50, .uv_stride = 0x60,
    .done = 0x3c, .done_y = 0x38, .status = 0xb4, .stall = 0x600,
    .mpll = 1200, .mpll_reg = 100u << 20 | 1u << 14 | 2u << 11 | 1u << 8 | 9,
};

static const Part *part = &t31;

typedef struct Pic {
    uint8_t y[FW * FH], uv[FW * FH / 2];
} Pic;

static uint64_t frame_ns = 40000000;    /* fps=25 */

static QTestState *start_on(const char *model, unsigned fps)
{
    QTestState *qts = qtest_initf("-machine %s -device "
                                  "image-sensor,model=%s,bus=i2c0,fps=%u,"
                                  "motion=off", part->machine, model, fps);

    frame_ns = 1000000000ull / fps;
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + 0x4, FW << 16 | FH);
    qtest_writel(qts, ISP + part->ch_en, 2);
    return qts;
}

static QTestState *start(void)
{
    return start_on("jxq03", 25);
}

/* Channel 1 scaled to w x h, a buffer queued */
static void push(QTestState *qts, uint32_t w, uint32_t h)
{
    qtest_writel(qts, CH1 + 0x00, w << 16 | h);
    qtest_writel(qts, CH1 + 0x80, w);
    qtest_writel(qts, CH1 + 0x98, w);
    qtest_writel(qts, CH1 + 0x6c, BUF);
    qtest_writel(qts, CH1 + 0x84, UVBUF);
}

/* The next frame channel 1 puts out, w x h */
static void frame(QTestState *qts, Pic *p, uint32_t w, uint32_t h, bool first)
{
    if (first) {
        qtest_writel(qts, ISP + 0xe0000, 1);
        if (part == &t31) {
            /* its frame ends a period's 15/16 on, as the next reaches it */
            qtest_clock_step(qts, frame_ns / 16 * 15);
        }
    } else {
        qtest_clock_step(qts, frame_ns);
    }
    g_assert_false(qtest_readl(qts, CH1 + 0x7c) & 1);
    qtest_readl(qts, CH1 + 0x74);
    qtest_memread(qts, BUF, p->y, w * h);
    qtest_memread(qts, UVBUF, p->uv, w * h / 2);
    push(qts, w, h);
}

static void set_fcrop(QTestState *qts, uint32_t x, uint32_t y, uint32_t w,
                      uint32_t h)
{
    qtest_writel(qts, ISP + 0x9860, x << 16 | y);
    qtest_writel(qts, ISP + 0x9864, w << 16 | h);
    qtest_writel(qts, ISP + 0x9804, 0xf0002);
}

static void test_fcrop(void)
{
    QTestState *qts = start();
    Pic full, p;

    push(qts, FW, FH);
    frame(qts, &full, FW, FH, true);

    /* the right half, 1:1, from the next frame on */
    set_fcrop(qts, 64, 0, 64, 64);
    push(qts, 64, 64);
    frame(qts, &p, 64, 64, false);
    for (int y = 0; y < 64; y++) {
        g_assert_cmpmem(p.y + y * 64, 64, full.y + y * FW + 64, 64);
    }

    /* the top left quarter, doubled each way */
    set_fcrop(qts, 0, 0, 64, 32);
    frame(qts, &p, 64, 64, false);
    for (int y = 0; y < 64; y++) {
        for (int x = 0; x < 64; x++) {
            g_assert_cmpuint(p.y[y * 64 + x], ==,
                             full.y[(y / 2) * FW + x]);
        }
    }

    /* the whole frame again */
    set_fcrop(qts, 0, 0, FW, FH);
    push(qts, FW, FH);
    frame(qts, &p, FW, FH, false);
    g_assert_cmpmem(p.y, sizeof(p.y), full.y, sizeof(full.y));
    qtest_quit(qts);
}

static void test_chan_crop(void)
{
    QTestState *qts = start();
    Pic full, p;

    push(qts, FW, FH);
    frame(qts, &full, FW, FH, true);

    /* the channel's window written while streaming: 64x32 at 32,16 */
    qtest_writel(qts, CH1 + 0x28, 32 << 16 | 16);
    qtest_writel(qts, CH1 + 0x2c, 64 << 16 | 32);
    qtest_writel(qts, CH1 + 0x80, 64);
    qtest_writel(qts, CH1 + 0x98, 64);
    qtest_writel(qts, CH1 + 0x6c, BUF);
    qtest_writel(qts, CH1 + 0x84, UVBUF);
    qtest_clock_step(qts, frame_ns);
    qtest_readl(qts, CH1 + 0x74);
    qtest_memread(qts, BUF, p.y, 64 * 32);
    for (int y = 0; y < 32; y++) {
        g_assert_cmpmem(p.y + y * 64, 64, full.y + (16 + y) * FW + 32, 64);
    }

    /* and the front crop under it: the window of the right half */
    set_fcrop(qts, 64, 0, 64, 64);
    qtest_writel(qts, CH1 + 0x00, 64 << 16 | 64);
    qtest_writel(qts, CH1 + 0x28, 0);
    qtest_writel(qts, CH1 + 0x2c, 0);
    qtest_writel(qts, CH1 + 0x80, 64);
    qtest_writel(qts, CH1 + 0x98, 64);
    qtest_writel(qts, CH1 + 0x6c, BUF);
    qtest_writel(qts, CH1 + 0x84, UVBUF);
    qtest_clock_step(qts, frame_ns);
    qtest_readl(qts, CH1 + 0x74);
    qtest_memread(qts, BUF, p.y, 64 * 64);
    for (int y = 0; y < 64; y++) {
        g_assert_cmpmem(p.y + y * 64, 64, full.y + y * FW + 64, 64);
    }
    qtest_quit(qts);
}

#define CPM_CPMPCR      0x10000014
#define CPM_ISPCDR      0x10000080
#define BIG_Y           0x00600000
#define BIG_UV          0x00700000

#define ISP_STATUS      (ISP + part->status)

/* Channel @ch at w x h (@unscaled: channel 0's scaler off): a frame later? */
static bool streams_on(QTestState *qts, unsigned ch, uint32_t w, uint32_t h,
                       bool unscaled)
{
    uint32_t base = ISP + part->ch + ch * 0x100;

    if (part == &t31) {
        /* the buffers left by a frame that stalled: one frame lands a call */
        qtest_writel(qts, base + 0x9c, 1);
        qtest_writel(qts, base + 0xa0, 1);
        qtest_writel(qts, base + 0x9c, 0);
        qtest_writel(qts, base + 0xa0, 0);
    }
    qtest_writel(qts, ISP + part->ch_en, 1u << ch | (unscaled ? 0x100 : 0));
    qtest_writel(qts, ISP_STATUS + 4, 0xffffffff);
    qtest_writel(qts, base + part->size, w << 16 | h);
    qtest_writel(qts, base + part->y_stride, w);
    qtest_writel(qts, base + part->uv_stride, w);
    qtest_writel(qts, base + part->y, BIG_Y);
    qtest_writel(qts, base + part->uv, BIG_UV);
    /* a T31's frame is out a period's 15/16 after it reaches the MSCA */
    qtest_clock_step(qts, part == &t31 ? 2 * frame_ns : frame_ns);
    /*
     * A buffer pushed after a frame started waits for the next frame (a
     * stall shows at once): a frame later
     */
    if ((qtest_readl(qts, base + part->done) & 1) &&
        (qtest_readl(qts, ISP_STATUS) & 0x600) != part->stall) {
        qtest_clock_step(qts, frame_ns / 2);
    }
    if (qtest_readl(qts, base + part->done) & 1) {
        g_assert_cmphex(qtest_readl(qts, ISP_STATUS) & 0x600, ==,
                        part->stall);
        return false;
    }
    qtest_readl(qts, base + part->done_y);
    g_assert_cmphex(qtest_readl(qts, ISP_STATUS) & 0x600, ==, 0);
    return true;
}

static bool streams(QTestState *qts, uint32_t w, uint32_t h, bool unscaled)
{
    return streams_on(qts, unscaled ? 0 : 1, w, h, unscaled);
}

static void set_clock(QTestState *qts, unsigned mhz)
{
    /* the MPLL at the part's rate, which a run with no kernel leaves off */
    qtest_writel(qts, CPM_CPMPCR, part->mpll_reg);
    /* the MPLL over a divider, MPLL (1) as its source, CE */
    qtest_writel(qts, CPM_ISPCDR, 1u << 30 | 1u << 29 |
                 (part->mpll / mhz - 1));
}

static void test_scaler_load(void)
{
    QTestState *qts = start_on("gc2053", 25);

    qtest_writel(qts, ISP + 0xe0000, 1);
    /* 200 MHz, a V3's: AutoZoom's scaled size past 5148 pixels a line */
    g_assert_true(streams(qts, 512, 640, false));
    g_assert_false(streams(qts, 512, 648, false));
    g_assert_true(streams(qts, 512, 640, false));
    /* 120 MHz: 2941 */
    set_clock(qts, 120);
    g_assert_true(streams(qts, 512, 360, false));
    g_assert_false(streams(qts, 512, 368, false));
    /* 240 MHz: 6308 */
    set_clock(qts, 240);
    g_assert_true(streams(qts, 512, 786, false));
    g_assert_false(streams(qts, 512, 790, false));
    /* between the measured clocks, straight: 150 MHz is 3968 */
    set_clock(qts, 150);
    g_assert_true(streams(qts, 512, 496, false));
    g_assert_false(streams(qts, 512, 497, false));
    qtest_quit(qts);
}

static void test_window_load(void)
{
    QTestState *qts = start_on("gc2053", 25);

    qtest_writel(qts, ISP + 0xe0000, 1);
    set_fcrop(qts, 0, 0, FW, 32);
    /* 200 MHz: a window 32 lines high enlarged past 5763 pixels a line */
    g_assert_true(streams(qts, 512, 360, false));
    g_assert_false(streams(qts, 512, 361, false));
    set_clock(qts, 240);
    g_assert_true(streams(qts, 512, 415, false));
    g_assert_false(streams(qts, 512, 417, false));
    set_clock(qts, 120);
    g_assert_true(streams(qts, 512, 195, false));
    g_assert_false(streams(qts, 512, 196, false));
    qtest_quit(qts);
}

/*
 * The limit is a pixel rate: a pixels-a-line one at a GC2053's line rate (1418
 * lines at 30 fps), so a sensor with other lines or rate has its own. A
 * model that gives none has the GC2053's, and the frame rate moves none
 */
static void test_line_rate(void)
{
    /* 25 fps of 1600 lines: 40000 lines a second, 5475 pixels a line at 200 */
    QTestState *qts = start_on("jxq03", 25);

    qtest_writel(qts, ISP + 0xe0000, 1);
    g_assert_true(streams(qts, 512, 684, false));
    g_assert_false(streams(qts, 512, 685, false));
    qtest_quit(qts);

    /* 30 fps of 1125: 33750, 6489 */
    qts = start_on("imx307", 25);
    qtest_writel(qts, ISP + 0xe0000, 1);
    g_assert_true(streams(qts, 512, 811, false));
    g_assert_false(streams(qts, 512, 812, false));
    qtest_quit(qts);

    /* none given: the reference's, 5148 */
    qts = start_on("jxf22", 25);
    qtest_writel(qts, ISP + 0xe0000, 1);
    g_assert_true(streams(qts, 512, 640, false));
    g_assert_false(streams(qts, 512, 648, false));
    qtest_quit(qts);

    /* a GC2053 at 10 fps: the blanking grows, the line rate stays */
    qts = start_on("gc2053", 10);
    qtest_writel(qts, ISP + 0xe0000, 1);
    g_assert_true(streams(qts, 512, 640, false));
    g_assert_false(streams(qts, 512, 648, false));
    qtest_quit(qts);
}

static void test_unscaled(void)
{
    QTestState *qts = start();

    qtest_writel(qts, ISP + 0xe0000, 1);
    /* a window as wide as the channel, the channel unscaled: it goes */
    set_fcrop(qts, 0, 0, 64, 64);
    g_assert_true(streams(qts, 64, 64, true));
    /* a smaller one under an unscaled channel stops it, at a load of 64 */
    g_assert_false(streams(qts, 128, 64, true));
    g_assert_true(streams_on(qts, 0, 128, 64, false));
    /* and no window, none */
    set_fcrop(qts, 0, 0, FW, FH);
    g_assert_true(streams(qts, 128, 64, true));
    qtest_quit(qts);
}

/*
 * The T21's: the vendor driver's ISP at 900 MHz over a divider, scaled sizes
 * past what the clock keeps up with at an ov2735b's 1595 lines at 25 fps; a
 * sensor with another line rate has its own, a frame rate none
 */
static void test_t21_load(void)
{
    QTestState *qts;

    part = &t21;
    qts = start_on("ov2735b", 25);
    qtest_writel(qts, ISP + part->vic, 1);
    set_clock(qts, 180);
    /* 4467 pixels a line of the 64 the frame has: 512 x 558 at most */
    g_assert_true(streams_on(qts, 0, 512, 558, false));
    g_assert_false(streams_on(qts, 0, 512, 559, false));
    set_clock(qts, 90);
    g_assert_true(streams_on(qts, 0, 512, 269, false));
    g_assert_false(streams_on(qts, 0, 512, 270, false));
    set_clock(qts, 225);
    g_assert_true(streams_on(qts, 0, 512, 701, false));
    g_assert_false(streams_on(qts, 0, 512, 702, false));
    /* below the first measured clock, scaled off it: 1822 at 75 MHz */
    set_clock(qts, 75);
    g_assert_true(streams_on(qts, 0, 512, 227, false));
    g_assert_false(streams_on(qts, 0, 512, 228, false));
    qtest_quit(qts);

    /* a jxf23's 1350 lines at 25 fps: 5277 at 180 MHz */
    qts = start_on("jxf23", 10);
    qtest_writel(qts, ISP + part->vic, 1);
    set_clock(qts, 180);
    g_assert_true(streams_on(qts, 0, 512, 659, false));
    g_assert_false(streams_on(qts, 0, 512, 660, false));
    qtest_quit(qts);

    /* an os02b10's 1361 lines at 25 fps: 5235 */
    qts = start_on("os02b10", 25);
    qtest_writel(qts, ISP + part->vic, 1);
    set_clock(qts, 180);
    g_assert_true(streams_on(qts, 0, 512, 654, false));
    g_assert_false(streams_on(qts, 0, 512, 655, false));
    qtest_quit(qts);
    part = &t31;
}

/*
 * The T23's: scaled sizes under the frame source (no AutoZoom: the open
 * driver stalls on the first enlargement) have a limit of their own at the
 * ISP clock, a front crop window in 0xd090 and 0xd094 a lower one, both at
 * an sc2336's 1440 lines at 25 fps, which an os02n10's 1109 lines at 30
 * fps scale to their own
 */
static void test_t23_load(void)
{
    QTestState *qts;

    part = &t23;
    qts = start_on("os02n10", 25);
    qtest_writel(qts, ISP + part->vic, 1);
    set_clock(qts, 200);
    /* 4967 * 36000 / 33270 = 5374 pixels a line of the 64 the frame has */
    g_assert_true(streams_on(qts, 0, 512, 671, false));
    g_assert_false(streams_on(qts, 0, 512, 672, false));
    /* a window 32 lines high: 3494 * 36000 / 33270 = 3780 */
    qtest_writel(qts, ISP + 0xd090, 0);
    qtest_writel(qts, ISP + 0xd094, FW << 16 | 32);
    g_assert_true(streams_on(qts, 0, 512, 236, false));
    g_assert_false(streams_on(qts, 0, 512, 237, false));
    /* 100 MHz: 2503 scaled the same, 2708 */
    qtest_writel(qts, ISP + 0xd094, FW << 16 | FH);
    set_clock(qts, 100);
    g_assert_true(streams_on(qts, 0, 512, 338, false));
    g_assert_false(streams_on(qts, 0, 512, 339, false));
    qtest_quit(qts);
    part = &t31;
}

/*
 * The T40's core: the width of its frame times the sensor's line rate over
 * the ISP clock, a gc4653's 1920 lines at 25 fps (48000 a second) and 2560
 * pixels: every frame while that is within 0.90 of the clock, every other
 * one up to 0.98, none past it, each dropped one an overflow (bit 5) in the
 * error bank at 0x40068, acknowledged at 0x40064; the scaler a limit of its
 * own
 */
#define T40_ERR         (ISP + 0x40068)
#define T40_ERR_CLR     (ISP + 0x40064)

static QTestState *t40_start_as(const char *variant, const char *model,
                                unsigned fps, uint32_t w, uint32_t h)
{
    QTestState *qts = qtest_initf("-machine ingenic-t40 -device "
                                  "image-sensor,model=%s,bus=i2c0,fps=%u,"
                                  "motion=off -global "
                                  "ingenic-t40.soc-variant=%s", model, fps,
                                  variant);

    frame_ns = 1000000000ull / fps;
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + 0x80, w << 16 | h);
    qtest_writel(qts, ISP + 0x16008, 1);
    qtest_writel(qts, ISP + 0x16100 + 0x80, 64);
    qtest_writel(qts, ISP + 0x16100 + 0x98, 64);
    qtest_writel(qts, ISP + 0x80000, 1);
    qtest_clock_step(qts, frame_ns);
    return qts;
}

static QTestState *t40_start(void)
{
    return t40_start_as("t40xp", "gc4653", 25, 2560, 8);
}

/* The MPLL at 1200 MHz, the ISP divider over it set to @div + 1 */
static void t40_clock(QTestState *qts, unsigned div)
{
    qtest_writel(qts, CPM_CPMPCR, 100u << 20 | 1u << 14 | 2u << 11 | 1u << 8 |
                 9);
    qtest_writel(qts, CPM_ISPCDR, 1u << 30 | 1u << 29 | div);
}

/* Of 16 frames at 25 fps, how many channel 0 put out (a buffer each time) */
static unsigned t40_frames(QTestState *qts, uint32_t w, uint32_t h)
{
    uint32_t base = ISP + 0x16100;
    unsigned got = 0;

    qtest_writel(qts, base, w << 16 | h);
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, base + 0x6c, BIG_Y);
        qtest_writel(qts, base + 0x84, BIG_UV);
        qtest_clock_step(qts, frame_ns);
        if (!(qtest_readl(qts, base + 0x7c) & 1)) {
            qtest_readl(qts, base + 0x74);
            got++;
        }
    }
    return got;
}

static void test_t40_core(void)
{
    QTestState *qts = t40_start();

    /* 150 MHz: 0.819 */
    t40_clock(qts, 7);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 16);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0);
    /* 133 MHz: 0.922, every other frame, each dropped one an overflow */
    t40_clock(qts, 8);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 8);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_writel(qts, T40_ERR_CLR, 0x20);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0);
    /* 120 MHz: 1.024, none */
    t40_clock(qts, 9);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_quit(qts);
}

/* The scaler: 4163 pixels a line at 200 MHz, over the frame's 1440 lines */
static void test_t40_scaler(void)
{
    QTestState *qts = t40_start();

    t40_clock(qts, 5);
    qtest_writel(qts, ISP + 0x80, 2560 << 16 | 1440);
    g_assert_cmpuint(t40_frames(qts, 2560, 2300), ==, 16);
    g_assert_cmpuint(t40_frames(qts, 2560, 2400), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_quit(qts);
}

/*
 * The T41's and the T32's: the core on the clock in CPM 0x80, the MSCA on
 * its own in 0xa8 (T41) or 0x90 (T32). A T41NQ's gc4023 has 1875 lines at 20
 * fps and the core keeps every frame up to a load (width times line rate over
 * the clock) of 0.969, none past it, each dropped one an overflow (bit 5) in
 * the error bank at 0x40068; a T32LQ's core keeps all up to 0.997 (a gc4653
 * here, its 1800 lines at 30 fps, the sc4336p's driver standing by), a dropped
 * one raising bit 9 in its VIC's status.
 */
typedef struct Wide {
    const char *machine, *model;
    const char *extra;      /* more command line */
    uint32_t ch_en, ch, size, y, uv, y_stride, uv_stride, done, done_y;
    uint32_t vic_start, vic_stat;
    uint32_t trig;          /* the registers latched at the next frame */
    uint32_t cim;           /* the sensor clock's divider */
    uint32_t mpll;          /* CPMPCR for the clock the dividers share */
    uint32_t sdr;           /* the MSCA's clock divider */
    uint32_t first_ms;      /* the first frame's latency */
} Wide;

static const Wide t41w = {
    .machine = "ingenic-t41", .model = "gc4023", .ch_en = 0xf0008,
    .ch = 0xf0100, .size = 0x00, .y = 0x6c, .uv = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done = 0x7c, .done_y = 0x74, .vic_start = 0x80000,
    .cim = CPM_CIMCDR, .trig = 0xf0010, .mpll = 0x15d0889d,
    .sdr = 0x10000000 + 0xa8, .first_ms = 135,
};

static const Wide t41lqw = {
    .machine = "ingenic-t41", .model = "gc5603",
    .extra = "-global ingenic-t40.soc-variant=t41lq",
    .ch_en = 0xf0008, .ch = 0xf0100, .size = 0x00, .y = 0x6c, .uv = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done = 0x7c, .done_y = 0x74,
    .vic_start = 0x80000, .cim = CPM_CIMCDR, .trig = 0xf0010,
    .mpll = 0x063008bd, .sdr = 0x10000000 + 0xa8, .first_ms = 135,
};

static const Wide t32w = {
    .machine = "ingenic-t32", .model = "gc4653", .ch_en = 0xf0040,
    .ch = 0xf0100, .size = 0x04, .y = 0x30, .uv = 0x50, .y_stride = 0x40,
    .uv_stride = 0x60, .done = 0x3c, .done_y = 0x38, .vic_start = 0x80010,
    .vic_stat = 0x80000 + 0x1e0,
    .cim = 0x1000007c, .mpll = 100u << 20 | 1u << 14 | 2u << 11 | 1u << 8 | 9,
    .sdr = 0x10000000 + 0x90, .first_ms = 0,
};

static QTestState *wide_start(const Wide *w, uint32_t width, uint32_t height)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,model=%s,"
                                  "bus=i2c0,fps=30,motion=off %s", w->machine,
                                  w->model, w->extra ?: "");

    frame_ns = 1000000000ull / 30;
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, w->cim, 0);
    qtest_writel(qts, CPM_CPMPCR, w->mpll);
    qtest_writel(qts, ISP + 0x80, width << 16 | height);
    qtest_writel(qts, ISP + w->ch_en, 1);
    qtest_writel(qts, ISP + w->vic_start, 1);
    qtest_clock_step(qts, frame_ns + w->first_ms * 1000000ull);
    return qts;
}

/* The MPLL (T41NQ 1400 MHz, T32LQ 1200) over @div + 1 on the clock at @reg */
static void wide_clock(QTestState *qts, uint32_t reg, unsigned div)
{
    qtest_writel(qts, reg, 1u << 30 | 1u << 29 | div);
}

/* Of 16 frames, how many channel 0 put out at w x h */
static unsigned wide_frames(QTestState *qts, const Wide *w, uint32_t cw,
                            uint32_t ch)
{
    uint32_t base = ISP + w->ch;
    unsigned got = 0;

    qtest_writel(qts, base + w->size, cw << 16 | ch);
    if (w->trig) {
        qtest_writel(qts, ISP + w->trig, 1);
    }
    qtest_clock_step(qts, frame_ns * 3);
    if (!(qtest_readl(qts, base + w->done) & 1)) {
        qtest_readl(qts, base + w->done_y);
    }
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, base + w->y_stride, cw);
        qtest_writel(qts, base + w->uv_stride, cw);
        qtest_writel(qts, base + w->y, BIG_Y);
        qtest_writel(qts, base + w->uv, BIG_UV);
        qtest_clock_step(qts, frame_ns);
        if (!(qtest_readl(qts, base + w->done) & 1)) {
            qtest_readl(qts, base + w->done_y);
            got++;
        }
    }
    return got;
}

static void test_t41_core(void)
{
    QTestState *qts = wide_start(&t41w, 2560, 1440);
    uint32_t vic = ISP + t41w.vic_start + 0x1e0;

    /* 100 MHz: 0.96 */
    wide_clock(qts, CPM_ISPCDR, 13);
    g_assert_cmpuint(wide_frames(qts, &t41w, 64, 64), ==, 16);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0);
    qtest_quit(qts);

    /* a frame 2610 wide at 100 MHz: 0.979, bit 5 alone */
    qts = wide_start(&t41w, 2610, 1440);
    wide_clock(qts, CPM_ISPCDR, 13);
    g_assert_cmpuint(wide_frames(qts, &t41w, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    g_assert_cmphex(qtest_readl(qts, vic) & 0x200, ==, 0);
    qtest_writel(qts, T40_ERR_CLR, 0x20);
    /* 93.3 MHz: 1.05, "break frame" and the VIC's FIFO overflow instead */
    wide_clock(qts, CPM_ISPCDR, 14);
    g_assert_cmpuint(wide_frames(qts, &t41w, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, vic) & 0x200, ==, 0x200);
    qtest_quit(qts);
}

/*
 * The MSCA on its own clock: 116.7 MHz gives 3150 scaled pixels a line, 2800
 * to an AutoZoom window (channel 0's input window: its position in 0xf00a0
 * and its size the scaling ratios in 0xf0728 and 0xf071c, the window over the
 * scaled size in 1/16384)
 */
static void test_t41_scaler(void)
{
    QTestState *qts = wide_start(&t41w, 2560, 1440);

    wide_clock(qts, t41w.sdr, 11);
    g_assert_cmpuint(wide_frames(qts, &t41w, 2560, 1700), ==, 16);
    g_assert_cmpuint(wide_frames(qts, &t41w, 2560, 1900), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_writel(qts, T40_ERR_CLR, 0x20);

    qtest_writel(qts, ISP + 0xf00a0, 0);
    qtest_writel(qts, ISP + 0xf0728, 1 << 14);
    qtest_writel(qts, ISP + 0xf071c, ((1400 << 14) + 720) / 1440);
    g_assert_cmpuint(wide_frames(qts, &t41w, 2560, 1440), ==, 16);
    qtest_writel(qts, ISP + 0xf071c, ((1300 << 14) + 720) / 1440);
    g_assert_cmpuint(wide_frames(qts, &t41w, 2560, 1440), ==, 0);
    /* a window as big as the frame is none */
    qtest_writel(qts, ISP + 0xf071c, 1 << 14);
    g_assert_cmpuint(wide_frames(qts, &t41w, 2560, 1700), ==, 16);
    qtest_quit(qts);
}

/*
 * A T41LQ's MSCA at 300 MHz (MPLL 1200 over 4): 6063 scaled pixels a line, and
 * 4248 to an AutoZoom window; at 150 MHz the 1:1 frame of 2880 pixels is more
 * than it keeps up with, the core at 240 MHz and at 150 as well
 */
static void test_t41lq_scaler(void)
{
    QTestState *qts = wide_start(&t41lqw, 2880, 1620);

    wide_clock(qts, CPM_ISPCDR, 4);
    wide_clock(qts, t41lqw.sdr, 3);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 2880, 3150), ==, 16);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 2880, 3500), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_writel(qts, T40_ERR_CLR, 0x20);

    qtest_writel(qts, ISP + 0xf00a0, 0);
    qtest_writel(qts, ISP + 0xf0728, 1 << 14);
    qtest_writel(qts, ISP + 0xf071c, ((1150 << 14) + 810) / 1620);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 2880, 1620), ==, 16);
    qtest_writel(qts, ISP + 0xf071c, ((1050 << 14) + 810) / 1620);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 2880, 1620), ==, 0);
    qtest_writel(qts, ISP + 0xf071c, 1 << 14);
    qtest_writel(qts, T40_ERR_CLR, 0x20);

    wide_clock(qts, t41lqw.sdr, 7);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 2880, 1620), ==, 0);
    wide_clock(qts, t41lqw.sdr, 5);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 2880, 1620), ==, 16);

    wide_clock(qts, CPM_ISPCDR, 7);
    g_assert_cmpuint(wide_frames(qts, &t41lqw, 64, 64), ==, 0);
    qtest_quit(qts);
}

static void test_t32_core(void)
{
    QTestState *qts = wide_start(&t32w, 2560, 1440);

    /* 150 MHz: 0.92 of a gc4653's 1800 lines at 30 fps */
    wide_clock(qts, CPM_ISPCDR, 7);
    qtest_writel(qts, ISP + 0x80000 + 0x1f0, 0xffffffff);
    g_assert_cmpuint(wide_frames(qts, &t32w, 64, 64), ==, 16);
    g_assert_cmphex(qtest_readl(qts, ISP + t32w.vic_stat) & 0x200, ==, 0);
    /* 133 MHz: 1.037, each dropped frame an error in the VIC's status */
    wide_clock(qts, CPM_ISPCDR, 8);
    g_assert_cmpuint(wide_frames(qts, &t32w, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + t32w.vic_stat) & 0x200, ==, 0x200);
    qtest_quit(qts);
}

/*
 * A T32LQ's MSCA at 300 MHz: about 6350 scaled pixels a line from the frame
 * source's scaler and 5350 to an AutoZoom window at the sc4336p's 45000
 * lines a second, a gc4653's 54000 here taking 5/6 of that (in the
 * channel's own block the window's position is +0x00, the ratios +0x10
 * vertical and +0x14 horizontal); a stall raises the ISP's status bit 5
 * and no VIC error
 */
static void test_t32_scaler(void)
{
    QTestState *qts = wide_start(&t32w, 2560, 1440);
    uint32_t chan = ISP + t32w.ch;

    wide_clock(qts, t32w.sdr, 3);
    g_assert_cmpuint(wide_frames(qts, &t32w, 3600, 2000), ==, 16);
    g_assert_cmpuint(wide_frames(qts, &t32w, 3840, 2160), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + t32w.vic_stat) & 0x200, ==, 0);

    qtest_writel(qts, chan, 0);
    qtest_writel(qts, chan + 0x14, 1 << 14);
    qtest_writel(qts, chan + 0x10, ((900 << 14) + 720) / 1440);
    g_assert_cmpuint(wide_frames(qts, &t32w, 2560, 1440), ==, 16);
    qtest_writel(qts, chan + 0x10, ((800 << 14) + 720) / 1440);
    g_assert_cmpuint(wide_frames(qts, &t32w, 2560, 1440), ==, 0);
    qtest_quit(qts);
}

/* The T40's AutoZoom windows: 4227 pixels a line at 200 MHz */
static void test_t40_window(void)
{
    QTestState *qts = t40_start();

    t40_clock(qts, 5);
    qtest_writel(qts, ISP + 0x80, 2560 << 16 | 1440);
    qtest_writel(qts, ISP + 0x160a0, 0);
    qtest_writel(qts, ISP + 0x160a4, 2560 << 16 | 880);
    g_assert_cmpuint(t40_frames(qts, 2560, 1440), ==, 16);
    qtest_writel(qts, ISP + 0x160a4, 2560 << 16 | 860);
    g_assert_cmpuint(t40_frames(qts, 2560, 1440), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_quit(qts);
}

/*
 * A T40NN's core (an imx307's 33750 lines a second over 1920 pixels; the
 * MPLL here 1200 MHz): every other frame at 70.6 MHz (0.918), none at 66.7
 * (0.972) with the overflow alone, the VIC's FIFO overflowing too at 63.2
 * (1.025), and at 60 (1.08) nothing from the core but that
 */
static void test_t40nn_core(void)
{
    QTestState *qts = t40_start_as("t40nn", "imx307", 30, 1920, 1080);
    uint32_t vic = ISP + 0x80000 + 0x1e0;

    t40_clock(qts, 16);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 8);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_writel(qts, T40_ERR_CLR, 0x20);
    t40_clock(qts, 17);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    g_assert_cmphex(qtest_readl(qts, vic) & 0x200, ==, 0);
    qtest_writel(qts, T40_ERR_CLR, 0x20);
    t40_clock(qts, 18);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    g_assert_cmphex(qtest_readl(qts, vic) & 0x200, ==, 0x200);
    qtest_writel(qts, T40_ERR_CLR, 0x20);
    t40_clock(qts, 19);
    g_assert_cmpuint(t40_frames(qts, 64, 64), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, vic) & 0x200, ==, 0x200);
    qtest_quit(qts);
}

/*
 * A T40NN's MSCA at 200 MHz (an imx307's 33750 lines a second): 6051 scaled
 * pixels a line from the frame source's scaler, 6867 to an AutoZoom window;
 * at 120 MHz (the qtest's MPLL is 1200) 3745 scaled
 */
static void test_t40nn_scaler(void)
{
    QTestState *qts = t40_start_as("t40nn", "imx307", 30, 1920, 1080);

    t40_clock(qts, 5);
    g_assert_cmpuint(t40_frames(qts, 3360, 1890), ==, 16);
    g_assert_cmpuint(t40_frames(qts, 3456, 1944), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T40_ERR), ==, 0x20);
    qtest_writel(qts, T40_ERR_CLR, 0x20);

    qtest_writel(qts, ISP + 0x160a0, 0);
    qtest_writel(qts, ISP + 0x160a4, 1920 << 16 | 304);
    g_assert_cmpuint(t40_frames(qts, 1920, 1080), ==, 16);
    qtest_writel(qts, ISP + 0x160a4, 1920 << 16 | 300);
    g_assert_cmpuint(t40_frames(qts, 1920, 1080), ==, 0);
    qtest_writel(qts, T40_ERR_CLR, 0x20);

    t40_clock(qts, 9);
    qtest_writel(qts, ISP + 0x160a4, 1920 << 16 | 1080);
    g_assert_cmpuint(t40_frames(qts, 2640, 1485), ==, 16);
    g_assert_cmpuint(t40_frames(qts, 2736, 1539), ==, 0);
    qtest_quit(qts);
}

/*
 * The T30's core drops frames silently, none through an interrupt: a
 * T30X's sc4236 at 48000 lines a second, 2048 pixels wide, kept at 111
 * MHz (0.885) and lost at 100 (0.983); a 1920 wide frame at 100 MHz
 * (0.922) loses every other one. The T20's, a jxf23's 33750 lines a second
 * at 1920 pixels, keeps its frames at 76.9 MHz (0.843) and 73 % of them at
 * 71.4 (0.907)
 */
#define CORE            (ISP + 0x80000)
#define DS1             (CORE + 0xc00)
#define BANK(n)         (0x02000000 + (n) * 0x100000)

static QTestState *apical_start(const char *machine, const char *model,
                                uint32_t width)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,model=%s,"
                                  "bus=i2c0,fps=25,motion=off", machine,
                                  model);

    frame_ns = 40000000;
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CORE + 0x10, width);
    qtest_writel(qts, CORE + 0x14, 4);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, DS1 + 8 + i * 4, BANK(i));
    }
    qtest_writel(qts, DS1 + 0x1c, 2);
    qtest_writel(qts, DS1 + 0x20, width * 2);
    qtest_writel(qts, DS1 + 0x04, 4 << 16 | width);
    qtest_writel(qts, DS1, 6 | 1u << 9);
    qtest_writel(qts, DS1 + 0x24, 2);
    return qts;
}

/* The frames the writer got in 20, the ISP clock MPLL over a divider */
static unsigned apical_frames(QTestState *qts, uint32_t mpll_reg,
                              unsigned div)
{
    qtest_writel(qts, CPM_CPMPCR, mpll_reg);
    qtest_writel(qts, CPM_ISPCDR, 1u << 30 | 1u << 29 | div);
    qtest_writel(qts, ISP, 1);
    qtest_clock_step(qts, frame_ns * 2);
    uint32_t before = qtest_readl(qts, DS1 + 0x30) & 0xffff;

    qtest_clock_step(qts, frame_ns * 20);
    return (qtest_readl(qts, DS1 + 0x30) & 0xffff) - before;
}

/* 24 MHz * 2 * (124 + 1) / (5 + 1): 1000 MHz, the T30's format; the T20's */
#define PLL_1000        (124u << 20 | 5u << 14 | 1)
#define PLL_1000_T20    (125u << 20 | 3u << 14 | 1u << 11 | 1u << 8 | 1)

static void test_t30_core(void)
{
    QTestState *qts = apical_start("ingenic-t30", "sc4236", 2048);

    g_assert_cmpuint(apical_frames(qts, PLL_1000, 8), ==, 20);
    qtest_quit(qts);
    qts = apical_start("ingenic-t30", "sc4236", 2048);
    g_assert_cmpuint(apical_frames(qts, PLL_1000, 9), ==, 0);
    qtest_quit(qts);
    qts = apical_start("ingenic-t30", "sc4236", 1920);
    g_assert_cmpuint(apical_frames(qts, PLL_1000, 9), ==, 10);
    qtest_quit(qts);
}

static void test_t20_core(void)
{
    QTestState *qts = apical_start("ingenic-t20", "jxf23", 1920);

    g_assert_cmpuint(apical_frames(qts, PLL_1000_T20, 12), ==, 20);
    qtest_quit(qts);
    qts = apical_start("ingenic-t20", "jxf23", 1920);
    /* 73 %: 14 or 15 of 20, by where the count starts */
    g_assert_cmpuint(apical_frames(qts, PLL_1000_T20, 13), >=, 14);
    g_assert_cmpuint(apical_frames(qts, PLL_1000_T20, 13), <=, 15);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-isp-zoom/fcrop", test_fcrop);
    qtest_add_func("ingenic-isp-zoom/chan-crop", test_chan_crop);
    qtest_add_func("ingenic-isp-zoom/scaler-load", test_scaler_load);
    qtest_add_func("ingenic-isp-zoom/window-load", test_window_load);
    qtest_add_func("ingenic-isp-zoom/line-rate", test_line_rate);
    qtest_add_func("ingenic-isp-zoom/unscaled", test_unscaled);
    qtest_add_func("ingenic-isp-zoom/t21-load", test_t21_load);
    qtest_add_func("ingenic-isp-zoom/t23-load", test_t23_load);
    qtest_add_func("ingenic-isp-zoom/t40-core", test_t40_core);
    qtest_add_func("ingenic-isp-zoom/t40-scaler", test_t40_scaler);
    qtest_add_func("ingenic-isp-zoom/t40-window", test_t40_window);
    qtest_add_func("ingenic-isp-zoom/t40nn-core", test_t40nn_core);
    qtest_add_func("ingenic-isp-zoom/t40nn-scaler", test_t40nn_scaler);
    qtest_add_func("ingenic-isp-zoom/t41-core", test_t41_core);
    qtest_add_func("ingenic-isp-zoom/t41-scaler", test_t41_scaler);
    qtest_add_func("ingenic-isp-zoom/t41lq-scaler", test_t41lq_scaler);
    qtest_add_func("ingenic-isp-zoom/t32-core", test_t32_core);
    qtest_add_func("ingenic-isp-zoom/t32-scaler", test_t32_scaler);
    qtest_add_func("ingenic-isp-zoom/t30-core", test_t30_core);
    qtest_add_func("ingenic-isp-zoom/t20-core", test_t20_core);
    return g_test_run();
}
