/*
 * QTest testcase for the T32's LDC in its online mode (0x13040000, 0x08 = 1)
 *
 * The ISP puts each core frame through the engine ahead of the channel
 * scaler, with the registers the driver left in the engine. A frame is
 * taken from the channel left at 1:1 with the engine online and checked
 * against an offline job of the same parameters on the same frame taken
 * with the engine off, then through a channel scaled down from it, and the
 * engine's status and interrupt are looked at over the frames.
 *
 * The frame is a synthetic sensor picture, which the sensor model draws the
 * same way in each run.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"

#define ISP             0x13300000
#define LDC             0x13040000
#define LDC_IRQ         38
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define FSIZE           0x80
#define CH_EN           0xf0040
#define CH              0xf0100
#define VIC_START       0x80010
#define W               640
#define H               360
#define LEN             (W * H * 3 / 2)
#define BUF0            0x02000000
#define BUF1            0x03000000
#define OUT             0x04000000

/* The vendor's 640 x 360 settings, with the 0x1f1f line lengths it uses */
static const uint32_t regs[][2] = {
    { 0x10, H << 16 | W }, { 0x14, 71 << 16 | 194 },
    { 0x1c, 2135 << 16 | 2135 }, { 0x20, 8024 }, { 0x50, 0x000c0f0f },
};

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 -device image-sensor,"
                                 "model=gc4653,bus=i2c0,fps=30,motion=off");

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + FSIZE, W << 16 | H);
    return qts;
}

static void channel(QTestState *qts, unsigned n, uint32_t w, uint32_t h,
                    uint32_t y)
{
    uint32_t base = ISP + CH + n * 0x100;

    qtest_writel(qts, base + 0x04, w << 16 | h);
    qtest_writel(qts, base + 0x40, w);
    qtest_writel(qts, base + 0x60, w);
    qtest_writel(qts, base + 0x30, y);
    qtest_writel(qts, base + 0x50, y + w * h);
}

static void engine(QTestState *qts, uint32_t mode)
{
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        qtest_writel(qts, LDC + regs[i][0], regs[i][1]);
    }
    qtest_writel(qts, LDC + 0x70, 0);
    qtest_writel(qts, LDC + 0x08, mode);
}

static void frames(QTestState *qts, unsigned n)
{
    qtest_writel(qts, ISP + VIC_START, 1);
    qtest_clock_step(qts, n * (1000000000ull / 30) + 135000000ull);
}

static void test_online(void)
{
    g_autofree uint8_t *ref = g_malloc(LEN);
    g_autofree uint8_t *want = g_malloc(LEN);
    g_autofree uint8_t *got = g_malloc(LEN);
    QTestState *qts = start();

    /* The core frame, the engine off */
    qtest_writel(qts, ISP + CH_EN, 1);
    channel(qts, 0, W, H, BUF0);
    frames(qts, 2);
    qtest_memread(qts, BUF0, ref, LEN);

    /* The same frame through an offline job */
    engine(qts, 0);
    qtest_writel(qts, LDC + 0x30, BUF0);
    qtest_writel(qts, LDC + 0x34, W);
    qtest_writel(qts, LDC + 0x38, OUT);
    qtest_writel(qts, LDC + 0x3c, W);
    qtest_writel(qts, LDC + 0x40, BUF0 + W * H);
    qtest_writel(qts, LDC + 0x44, W);
    qtest_writel(qts, LDC + 0x48, OUT + W * H);
    qtest_writel(qts, LDC + 0x4c, W);
    qtest_writel(qts, LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, LDC + 0x68), ==, 1);
    qtest_memread(qts, OUT, want, LEN);
    g_assert_cmpint(memcmp(ref, want, LEN), !=, 0);
    qtest_quit(qts);

    /* The engine online: the channel's frame is the corrected one */
    qts = start();
    engine(qts, 1);
    qtest_writel(qts, LDC + 0x04, 1);
    qtest_writel(qts, ISP + CH_EN, 1);
    channel(qts, 0, W, H, BUF0);
    frames(qts, 2);
    qtest_memread(qts, BUF0, got, LEN);
    g_assert_cmpint(memcmp(got, want, LEN), ==, 0);
    /* Online, WORK_EN stays set and each frame ends with the status */
    g_assert_cmphex(qtest_readl(qts, LDC + 0x04), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LDC + 0x68), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-ldc-online/t32/frame", test_online);
    return g_test_run();
}
