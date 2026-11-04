/*
 * QTest testcase for the older ISP statistics: the T20's Apical metering
 * (the T10's and T30's are the same core) and the T21's Tiziano blocks
 *
 * The layouts and counts a T20X and a T21N write for a 1920x1080 frame:
 * the T20's histogram counts half the pixels, each by its zone's weight
 * (15 counting 16), in Apical floats, and an AWB zone of the 15x15 grid
 * counts each of its 128x72 pixels; the T21's histogram is 20-bit bins in
 * pairs tagged with the AE block's frame count, its AWB zones the R, G and
 * B sums and their number, tagged with that count less one, and it leaves
 * the IR half of the histogram buffer alone.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

#define ISP             0x13300000
#define CORE            (ISP + 0x80000)
#define CPM_CLKGR0      0x10000020
#define W               1920
#define H               1080
#define FRAME_NS        100000000   /* a few frames at the sensor's rate */

/* An Apical float's value */
static uint32_t apical_float(uint32_t v)
{
    uint32_t e = extract32(v, 12, 4), m = extract32(v, 0, 12);

    return e ? (m | 0x1000) << (e - 1) : m;
}

static uint64_t t20_hist_sum(QTestState *qts)
{
    uint64_t sum = 0;

    for (int i = 0; i < 256; i++) {
        sum += apical_float(qtest_readl(qts, CORE + 0x10000 + i * 4));
    }
    return sum;
}

static void t20_weights(QTestState *qts, uint8_t w)
{
    for (int i = 0; i < 225; i += 4) {
        qtest_writel(qts, CORE + 0x1e00 + i, w * 0x01010101u);
    }
}

static QTestState *t20_start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t20 "
                                 "-device image-sensor,model=jxq03,bus=i2c0");

    /* as the T20X's firmware sets them up */
    qtest_writel(qts, CORE + 0x10, W);
    qtest_writel(qts, CORE + 0x14, H);
    qtest_writel(qts, CORE + 0x18, 3);              /* B first */
    qtest_writel(qts, CORE + 0x300, 0x1fe);         /* R, whatever the order */
    qtest_writel(qts, CORE + 0x304, 0x100);
    qtest_writel(qts, CORE + 0x308, 0x100);
    qtest_writel(qts, CORE + 0x30c, 0x1ce);         /* B */
    qtest_writel(qts, CORE + 0x800, 13);
    qtest_writel(qts, CORE + 0x804, 64);
    qtest_writel(qts, CORE + 0x808, 144);
    qtest_writel(qts, CORE + 0x80c, 192);
    qtest_writel(qts, CORE + 0x830, 0x0f0f);
    qtest_writel(qts, CORE + 0x840, 0x3ff);
    qtest_writel(qts, CORE + 0x848, 0x1ff);
    qtest_writel(qts, CORE + 0x84c, 0x040);
    qtest_writel(qts, CORE + 0x850, 0x1ff);
    qtest_writel(qts, CORE + 0x854, 0x040);
    qtest_writel(qts, CORE + 0x870, 0x0f0f);
    t20_weights(qts, 15);
    qtest_writel(qts, CORE + 0x80, 11 | 9 << 16);   /* slots: AE, AWB */
    qtest_writel(qts, ISP, 1);                      /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    return qts;
}

static void test_t20_ae(void)
{
    QTestState *qts = t20_start();
    uint64_t sum = t20_hist_sum(qts);
    uint32_t five = 0;

    g_assert_cmphex(qtest_readl(qts, CORE + 0xa0) & 3, ==, 3);
    /* weight 15 counts 16, each of half the pixels; floats round down */
    g_assert_cmpuint(sum, <=, 16ull * W * H / 2);
    g_assert_cmpuint(sum, >, 16ull * W * H / 2 * 999 / 1000);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x8c8), ==, 0xfd200);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x8cc), ==, 0xfd200);
    for (int i = 0; i < 4; i++) {
        five += qtest_readl(qts, CORE + 0x820 + i * 4);
    }
    g_assert_cmpuint(five, <=, 0xffff);

    t20_weights(qts, 1);
    qtest_clock_step(qts, FRAME_NS);
    sum = t20_hist_sum(qts);
    g_assert_cmpuint(sum, <=, W * H / 2);
    g_assert_cmpuint(sum, >, W * H / 2 * 999 / 1000);

    /* one zone of 128x72: 4608 counted, every bin under 4096 exact */
    t20_weights(qts, 0);
    qtest_writel(qts, CORE + 0x1e00, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(t20_hist_sum(qts), ==, 4608);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x8cc), ==, 4608);
    t20_weights(qts, 0);
    qtest_writel(qts, CORE + 0x1ee0, 1);            /* zone 224 */
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(t20_hist_sum(qts), ==, 4608);
    qtest_quit(qts);
}

static void test_t20_awb(void)
{
    QTestState *qts = t20_start();
    uint32_t sum = 0, zone;

    for (int z = 0; z < 225; z++) {
        uint32_t n = qtest_readl(qts, CORE + 0x8744 + z * 8);

        g_assert_cmpuint(n, <=, 128 * 72);
        g_assert_cmpuint(n % 4, ==, 0);
        sum += n;
    }
    g_assert_cmpuint(qtest_readl(qts, CORE + 0x860), ==, sum);
    /* a grey of the bottom row, balanced by the gains: G/R and G/B 1.0 */
    zone = qtest_readl(qts, CORE + 0x8740 + (14 * 15 + 7) * 8);
    g_assert_cmpuint(qtest_readl(qts, CORE + 0x8744 + (14 * 15 + 7) * 8), ==,
                     128 * 72);
    g_assert_cmpint(abs((int)extract32(zone, 0, 12) - 0x100), <=, 4);
    g_assert_cmpint(abs((int)extract32(zone, 16, 12) - 0x100), <=, 4);
    /* R/G and B/G in the current mode; 0x300 is R's on a B-first sensor */
    qtest_writel(qts, CORE + 0x868, 1);
    qtest_writel(qts, CORE + 0x300, 0x100);
    qtest_clock_step(qts, FRAME_NS);
    zone = qtest_readl(qts, CORE + 0x8740 + (14 * 15 + 7) * 8);
    g_assert_cmpint(abs((int)extract32(zone, 0, 12) - 0x81), <=, 4);
    g_assert_cmpint(abs((int)extract32(zone, 16, 12) - 0x100), <=, 4);
    qtest_quit(qts);
}

static void bits_get(const uint32_t *w, unsigned at, unsigned n, uint32_t *v)
{
    *v = 0;
    for (unsigned i = 0; i < n; i++) {
        *v |= ((w[(at + i) / 32] >> ((at + i) % 32)) & 1) << i;
    }
}

#define T21_AE_ZONES    0x01000000
#define T21_AE_HIST     0x01010000
#define T21_AWB         0x01020000

static void test_t21(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21 "
                                 "-device image-sensor,model=jxq03,bus=i2c0");
    uint32_t idx, tag, hist[512], zone[4], v, sum = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    /* 15x15 zones of 64x36 quads, as the T21N's driver sets them */
    for (uint64_t b = ISP + 0x700; b <= ISP + 0x800; b += 0x100) {
        qtest_writel(qts, b + 0x4, 0xf001f001);
        for (int i = 0; i < 4; i++) {
            qtest_writel(qts, b + 0x8 + i * 4, 0x40404040);
            qtest_writel(qts, b + 0x18 + i * 4, 0x24242424);
        }
        qtest_writel(qts, b, 1);
    }
    qtest_writel(qts, ISP + 0x728, 0x0011991e);
    qtest_writel(qts, ISP + 0x72c, T21_AE_ZONES);
    qtest_writel(qts, ISP + 0x73c, T21_AE_HIST);
    qtest_writel(qts, ISP + 0x74c, 0);
    qtest_writel(qts, ISP + 0x828, 0x010b003e);
    qtest_writel(qts, ISP + 0x82c, 0x0104003b);
    qtest_writel(qts, ISP + 0x830, 0x00790100);
    qtest_writel(qts, ISP + 0x834, 0x020f0100);
    qtest_writel(qts, ISP + 0x838, 0x0001f00a);
    qtest_writel(qts, ISP + 0x83c, T21_AWB);
    qtest_writel(qts, ISP + 0x84c, 0);
    qtest_memset(qts, T21_AE_HIST, 0xa5, sizeof(hist));
    qtest_writel(qts, ISP + 0x80, 7 << 16);
    qtest_writel(qts, ISP + 0x10000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS);

    g_assert_cmphex(qtest_readl(qts, ISP + 0x84) & (7 << 16), ==, 7 << 16);
    idx = qtest_readl(qts, ISP + 0x750);
    tag = extract32(idx, 8, 8);
    g_assert_cmpuint(tag, >, 0);
    qtest_memread(qts, T21_AE_HIST, hist, sizeof(hist));
    for (int i = 0; i < 256; i++) {
        uint32_t w = le32_to_cpu(hist[i]);

        g_assert_cmphex(w >> 20, ==, i & 1 ? tag : 0);
        sum += w & 0xfffff;
    }
    g_assert_cmpuint(sum, ==, W * H / 4);
    for (int i = 256; i < 512; i++) {
        g_assert_cmphex(hist[i], ==, 0xa5a5a5a5);
    }
    /* an AE zone: its 64x36 quads, the sums' and counts' fields */
    qtest_memread(qts, T21_AE_ZONES + 112 * 16, zone, sizeof(zone));
    bits_get(zone, 63, 13, &v);
    sum = v;
    bits_get(zone, 76, 13, &v);
    g_assert_cmpuint(sum + v, <=, 64 * 36);
    /* an AWB zone: R, G, B, the count at 63 and the tag at 76 */
    qtest_memread(qts, T21_AWB + 224 * 16, zone, sizeof(zone));
    bits_get(zone, 63, 13, &v);
    g_assert_cmpuint(v, ==, 64 * 36);
    bits_get(zone, 76, 8, &v);
    g_assert_cmpuint(v, ==, (tag - 1) & 0xff);
    bits_get(zone, 21, 21, &v);
    g_assert_cmpuint(v, >, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-stats/t20/ae", test_t20_ae);
    qtest_add_func("/ingenic-isp-stats/t20/awb", test_t20_awb);
    qtest_add_func("/ingenic-isp-stats/t21", test_t21);
    return g_test_run();
}
