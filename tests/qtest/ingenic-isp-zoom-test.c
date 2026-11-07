/*
 * QTest testcase for a T31's channel crop and front crop while streaming
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

typedef struct Pic {
    uint8_t y[FW * FH], uv[FW * FH / 2];
} Pic;

static uint64_t frame_ns = 40000000;    /* fps=25 */

static QTestState *start_on(const char *model, unsigned fps)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 -device "
                                  "image-sensor,model=%s,bus=i2c0,fps=%u,"
                                  "motion=off", model, fps);

    frame_ns = 1000000000ull / fps;
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + 0x4, FW << 16 | FH);
    qtest_writel(qts, ISP + 0x9804, 2);
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
#define ISP_STATUS      (ISP + 0xb4)

/* Channel @ch at w x h (@unscaled: channel 0's scaler off): a frame later? */
static bool streams_on(QTestState *qts, unsigned ch, uint32_t w, uint32_t h,
                       bool unscaled)
{
    uint32_t base = ISP + 0x9900 + ch * 0x100;

    qtest_writel(qts, ISP + 0x9804, 1u << ch | (unscaled ? 0x100 : 0));
    qtest_writel(qts, ISP_STATUS + 4, 0xffffffff);
    qtest_writel(qts, base + 0x00, w << 16 | h);
    qtest_writel(qts, base + 0x80, w);
    qtest_writel(qts, base + 0x98, w);
    qtest_writel(qts, base + 0x6c, BIG_Y);
    qtest_writel(qts, base + 0x84, BIG_UV);
    qtest_clock_step(qts, frame_ns);
    if (qtest_readl(qts, base + 0x7c) & 1) {
        g_assert_cmphex(qtest_readl(qts, ISP_STATUS) & 0x600, ==, 0x600);
        return false;
    }
    qtest_readl(qts, base + 0x74);
    g_assert_cmphex(qtest_readl(qts, ISP_STATUS) & 0x600, ==, 0);
    return true;
}

static bool streams(QTestState *qts, uint32_t w, uint32_t h, bool unscaled)
{
    return streams_on(qts, unscaled ? 0 : 1, w, h, unscaled);
}

static void set_clock(QTestState *qts, unsigned mhz)
{
    /* the MPLL at 1200 MHz (a T31X's), which a run with no kernel leaves off */
    qtest_writel(qts, CPM_CPMPCR, 100u << 20 | 1u << 14 | 2u << 11 | 1u << 8 |
                 0x9);
    /* the MPLL's 1200 MHz over a divider, MPLL (1) as its source, CE */
    qtest_writel(qts, CPM_ISPCDR, 1u << 30 | 1u << 29 | (1200 / mhz - 1));
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

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-isp-zoom/fcrop", test_fcrop);
    qtest_add_func("ingenic-isp-zoom/chan-crop", test_chan_crop);
    qtest_add_func("ingenic-isp-zoom/scaler-load", test_scaler_load);
    qtest_add_func("ingenic-isp-zoom/window-load", test_window_load);
    qtest_add_func("ingenic-isp-zoom/line-rate", test_line_rate);
    qtest_add_func("ingenic-isp-zoom/unscaled", test_unscaled);
    return g_test_run();
}
