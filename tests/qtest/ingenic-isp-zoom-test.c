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
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define FRAME_NS        40000000        /* fps=25 */
#define BUF             0x00400000
#define UVBUF           0x00410000
#define CH1             (ISP + 0x9a00)
#define FW              128             /* the frame the ISP takes */
#define FH              64

typedef struct Pic {
    uint8_t y[FW * FH], uv[FW * FH / 2];
} Pic;

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 -device "
                                 "image-sensor,model=jxq03,bus=i2c0,fps=25,"
                                 "motion=off");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + 0x4, FW << 16 | FH);
    qtest_writel(qts, ISP + 0x9804, 2);
    return qts;
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
        qtest_clock_step(qts, FRAME_NS);
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
    qtest_clock_step(qts, FRAME_NS);
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
    qtest_clock_step(qts, FRAME_NS);
    qtest_readl(qts, CH1 + 0x74);
    qtest_memread(qts, BUF, p.y, 64 * 64);
    for (int y = 0; y < 64; y++) {
        g_assert_cmpmem(p.y + y * 64, 64, full.y + y * FW + 64, 64);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-isp-zoom/fcrop", test_fcrop);
    qtest_add_func("ingenic-isp-zoom/chan-crop", test_chan_crop);
    return g_test_run();
}
