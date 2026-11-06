/*
 * QTest testcase for the T31's privacy masks
 *
 * As a T31X draws them: channel c's mask m at 0x9938 + c * 0x100 + m * 0xc,
 * position (left << 16 | top), size (width << 16 | height) and colour
 * (Y 23:16, Cr 15:8, Cb 7:0, at once). Bits 5:3 of the channel enable
 * (0x9804) show channels 0-2's, at once; a written position or size reads
 * back and draws only once bit 17 + c of the enable loads it, as the next
 * frame reaches the MSCA.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define CH_EN           (ISP + 0x9804)
#define CH(n)           (ISP + 0x9900 + (n) * 0x100)
#define MASK(n, m)      (CH(n) + 0x38 + (m) * 0xc)
#define FRAME_NS        40000000        /* fps=25 */
#define W               64
#define H               64
#define BUF             0x00400000

static void push(QTestState *qts, unsigned n)
{
    qtest_writel(qts, CH(n), W << 16 | H);
    qtest_writel(qts, CH(n) + 0x80, W);
    qtest_writel(qts, CH(n) + 0x98, W);
    qtest_writel(qts, CH(n) + 0x6c, BUF + n * 0x4000);
    qtest_writel(qts, CH(n) + 0x84, BUF + n * 0x4000 + W * H);
}

/* The next frame, on channel 0 */
static void frame(QTestState *qts)
{
    qtest_clock_step(qts, FRAME_NS);
    g_assert_false(qtest_readl(qts, CH(0) + 0x7c) & 1);
    qtest_readl(qts, CH(0) + 0x74);
    push(qts, 0);
}

static uint8_t luma(QTestState *qts, unsigned x, unsigned y)
{
    return qtest_readb(qts, BUF + y * W + x);
}

/* The chroma at (x, y): Cb << 8 | Cr */
static uint16_t chroma(QTestState *qts, unsigned x, unsigned y)
{
    uint64_t a = BUF + W * H + y / 2 * W + (x & ~1);

    return qtest_readb(qts, a) << 8 | qtest_readb(qts, a + 1);
}

/* Whether mask 8x4 at (16, 8) of colour Y 0x80, Cb 0x20, Cr 0x40 shows */
static bool shown(QTestState *qts)
{
    bool in = luma(qts, 16, 8) == 0x80 && luma(qts, 23, 11) == 0x80 &&
              chroma(qts, 16, 8) == 0x2040 && chroma(qts, 22, 10) == 0x2040;

    if (in) {
        g_assert_cmphex(chroma(qts, 24, 8), !=, 0x2040);
        g_assert_cmphex(chroma(qts, 16, 12), !=, 0x2040);
    }
    return in;
}

static void test_mask(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 -device image-sensor,"
                                 "model=jxq03,bus=i2c0,fps=25");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CH_EN, 1);
    push(qts, 0);
    qtest_writel(qts, ISP + 0xe0000, 1);
    frame(qts);

    /* written but not loaded: reads 0, draws nothing */
    qtest_writel(qts, MASK(0, 1), 16 << 16 | 8);
    qtest_writel(qts, MASK(0, 1) + 4, 8 << 16 | 4);
    qtest_writel(qts, MASK(0, 1) + 8, 0x00804020);
    qtest_writel(qts, CH_EN, 1 | 1 << 3);
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1) + 4), ==, 0);
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1) + 8), ==, 0x00804020);
    frame(qts);
    g_assert_false(shown(qts));

    /* another channel's load leaves it */
    qtest_writel(qts, CH_EN, 1 | 1 << 3 | 1 << 18);
    frame(qts);
    g_assert_false(shown(qts));
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1)), ==, 0);

    /* its own: as the next frame reaches the MSCA */
    qtest_writel(qts, CH_EN, 1 | 1 << 3 | 1 << 17);
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1)), ==, 0);
    frame(qts);
    g_assert_cmphex(qtest_readl(qts, CH_EN), ==, 1 | 1 << 3);
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1)), ==, 16 << 16 | 8);
    g_assert_cmphex(qtest_readl(qts, MASK(0, 1) + 4), ==, 8 << 16 | 4);
    g_assert_true(shown(qts));

    /* the enable, at once */
    qtest_writel(qts, CH_EN, 1);
    frame(qts);
    g_assert_false(shown(qts));
    qtest_writel(qts, CH_EN, 1 | 1 << 3);
    frame(qts);
    g_assert_true(shown(qts));

    /* channel 1's enable is bit 4 */
    qtest_writel(qts, CH_EN, 1 | 1 << 4);
    frame(qts);
    g_assert_false(shown(qts));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-mask/t31", test_mask);
    return g_test_run();
}
