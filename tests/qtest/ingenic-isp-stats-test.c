/*
 * QTest testcase for the older ISP statistics: the T20's Apical metering
 * (the T10's and T30's are the same core) and the T21's Tiziano blocks
 *
 * The layouts and counts a T20X and a T21N write for a 1920x1080 frame:
 * the T20's histogram counts half the pixels, each by its zone's weight
 * (15 counting 16), in Apical floats, each AE zone its own 5-bin one of
 * all its pixels, and an AWB zone of the 15x15 grid counts each of its
 * 128x72 pixels, the whole picture's by the zones' weights, and its AF
 * zones read what a T20X does looking at its test pattern; the T21's
 * histogram is 20-bit bins in pairs tagged with the AE block's frame
 * count, its AWB zones the R, G and B sums and their number, tagged with
 * that count less one, and it leaves the IR half of the histogram buffer
 * alone.
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

static void t20_awb_weights(QTestState *qts, uint8_t w)
{
    for (int i = 0; i < 225; i += 4) {
        qtest_writel(qts, CORE + 0x1f00 + i, w * 0x01010101u);
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
    t20_awb_weights(qts, 15);
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

/* Each AE zone's 5-bin histogram, unweighted: words from 0x88000 */
static void test_t20_ae_zones(void)
{
    QTestState *qts = t20_start();

    /* rounded down, but a full bin reads 65535: a T20X's add to 65536 */
    for (int z = 0; z < 225; z++) {
        uint32_t a = qtest_readl(qts, CORE + 0x8000 + z * 8);
        uint32_t b = qtest_readl(qts, CORE + 0x8004 + z * 8);

        g_assert_cmpuint((a & 0xffff) + (a >> 16) + (b & 0xffff) + (b >> 16),
                         <=, 0x10000);
    }
    /* thresholds 0: every pixel in bin 4, whatever the zone's weight */
    t20_weights(qts, 0);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, CORE + 0x800 + i * 4, 0);
    }
    qtest_clock_step(qts, FRAME_NS);
    for (int z = 0; z < 225; z++) {
        g_assert_cmphex(qtest_readl(qts, CORE + 0x8000 + z * 8), ==, 0);
        g_assert_cmphex(qtest_readl(qts, CORE + 0x8004 + z * 8), ==,
                        0xffff0000);
    }
    qtest_quit(qts);
}

/* The AWB weights count in the whole picture's sums, not in the zones' */
static uint32_t t20_awb_zone_sum(QTestState *qts)
{
    uint32_t sum = 0;

    for (int z = 0; z < 225; z++) {
        sum += qtest_readl(qts, CORE + 0x8744 + z * 8);
    }
    return sum;
}

static void test_t20_awb_weights(void)
{
    QTestState *qts = t20_start();

    g_assert_cmpuint(qtest_readl(qts, CORE + 0x860), ==,
                     t20_awb_zone_sum(qts));
    t20_awb_weights(qts, 8);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(qtest_readl(qts, CORE + 0x860), ==,
                     t20_awb_zone_sum(qts) / 2);
    /* none counted: the ratios read 0xfff */
    t20_awb_weights(qts, 0);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(qtest_readl(qts, CORE + 0x860), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x858), ==, 0xfff);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x85c), ==, 0xfff);
    g_assert_cmpuint(t20_awb_zone_sum(qts), >, 0);
    qtest_quit(qts);
}

/*
 * AF of the test pattern: a rectangle of @fg (R = G = B, 16 bits) from
 * line @top and pixel @left to @bottom and @right on @bg
 */
static void t20_pattern(QTestState *qts, uint32_t bg, uint32_t fg,
                        uint32_t top, uint32_t bottom, uint32_t left,
                        uint32_t right)
{
    qtest_writel(qts, CORE + 0x40, 0);
    qtest_writel(qts, CORE + 0x9c0, 1);
    qtest_writel(qts, CORE + 0x9c4, 4);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, CORE + 0x9c8 + i * 4, bg);
        qtest_writel(qts, CORE + 0x9d4 + i * 4, fg);
    }
    qtest_writel(qts, CORE + 0x9e8, top);
    qtest_writel(qts, CORE + 0x9ec, bottom);
    qtest_writel(qts, CORE + 0x9f0, left);
    qtest_writel(qts, CORE + 0x9f8, right);
    qtest_clock_step(qts, FRAME_NS);
}

static uint32_t t20_af(QTestState *qts, int row, int col)
{
    return qtest_readl(qts, CORE + 0x8e80 + (row * 15 + col) * 4);
}

/* @v within @pct percent of @want */
static void assert_near(uint32_t v, uint32_t want, uint32_t pct)
{
    g_assert_cmpuint(v * 100, >=, want * (100 - pct));
    g_assert_cmpuint(v * 100, <=, want * (100 + pct));
}

/*
 * AF zones against a T20X looking at its test pattern, 15x15 zones, shift
 * 3, threshold 1, mode 1: what it read, the model within a few percent
 */
static void test_t20_af(void)
{
    QTestState *qts = t20_start();
    uint32_t v, lo;

    qtest_writel(qts, CORE + 0x884, 1);
    qtest_writel(qts, CORE + 0x88c, 3);
    qtest_writel(qts, CORE + 0x890, 0x0f0f);
    qtest_writel(qts, CORE + 0x898, 1);

    /* flat: no contrast; the intensity a quarter of the level */
    t20_pattern(qts, 0x8000, 0x8000, 1, 1080, 1, 1920);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x89c), ==, 0x2000);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x888) >> 16, ==, 0x2000);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x880) & 0xffff, ==, 0);
    for (int z = 0; z < 225; z++) {
        g_assert_cmphex(t20_af(qts, 0, z), ==, 0);
    }
    /* black: no intensity, every zone's and the whole's metric saturates */
    t20_pattern(qts, 0, 0, 1, 1080, 1, 1920);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x880) & 0xffff, ==, 0xffff);
    for (int z = 0; z < 225; z++) {
        g_assert_cmphex(t20_af(qts, 0, z), ==, 0xffff);
    }
    /* a step from black to 0x1000: 54 in bits 31:16 */
    t20_pattern(qts, 0, 0x1000, 1, 1080, 705, 1216);
    g_assert_cmphex(t20_af(qts, 7, 5) & 0xffff, ==, 0xffff);
    assert_near(t20_af(qts, 7, 5) >> 16, 54, 10);

    /*
     * 0x4000 to 0x5000 at pixel 704, back at 1216: the T20X's 2370 and
     * 16 in the zone of the first, the whole picture's 0x1364
     */
    t20_pattern(qts, 0x4000, 0x5000, 1, 1080, 705, 1216);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x89c), ==, 0x1111);
    v = t20_af(qts, 7, 5);
    lo = v & 0xffff;
    assert_near(lo, 2370, 5);
    g_assert_cmpuint(v >> 16, ==, 16);
    assert_near(t20_af(qts, 7, 9) & 0xffff, 2113, 15);
    g_assert_cmphex(t20_af(qts, 7, 0), ==, 0);
    g_assert_cmphex(t20_af(qts, 7, 7), ==, 0);
    assert_near(qtest_readl(qts, CORE + 0x880) & 0xffff, 0x1364, 5);
    /* the shift doubles bits 15:0 alone */
    qtest_writel(qts, CORE + 0x88c, 5);
    qtest_clock_step(qts, FRAME_NS);
    assert_near(t20_af(qts, 7, 5) & 0xffff, lo * 4, 1);
    g_assert_cmpuint(t20_af(qts, 7, 5) >> 16, ==, 16);
    qtest_writel(qts, CORE + 0x88c, 3);
    /* mode 0: the intensity in bits 31:16; 3: nothing; 5: about twice it */
    qtest_writel(qts, CORE + 0x898, 0);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(t20_af(qts, 7, 5), ==, 0x1111 << 16 | lo);
    g_assert_cmphex(t20_af(qts, 0, 0), ==, 0x1111 << 16);
    qtest_writel(qts, CORE + 0x898, 3);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(t20_af(qts, 7, 5), ==, 0);
    assert_near(qtest_readl(qts, CORE + 0x880) & 0xffff, 0x1364, 5);
    qtest_writel(qts, CORE + 0x898, 5);
    qtest_clock_step(qts, FRAME_NS);
    assert_near(qtest_readl(qts, CORE + 0x89c), 0x2267, 3);
    assert_near(t20_af(qts, 7, 5) & 0xffff, lo / 4, 1);
    g_assert_cmpuint(t20_af(qts, 7, 5) >> 16, ==, 16);
    qtest_writel(qts, CORE + 0x898, 1);

    /* the threshold: 0x48 takes the T20X's 2011 to 1326, 0x180 to 0 */
    t20_pattern(qts, 0x4000, 0x5000, 1, 1080, 1, 704);
    assert_near(t20_af(qts, 7, 5) & 0xffff, 2011, 15);
    qtest_writel(qts, CORE + 0x884, 0x48);
    qtest_clock_step(qts, FRAME_NS);
    assert_near(t20_af(qts, 7, 5) & 0xffff, 1326, 10);
    qtest_writel(qts, CORE + 0x884, 0x180);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(t20_af(qts, 7, 5) & 0xffff, ==, 0);
    qtest_writel(qts, CORE + 0x884, 1);

    /* bigger steps, and a level across (each zone of row 7 crossed) */
    t20_pattern(qts, 0x4000, 0x8000, 1, 1080, 705, 1216);
    assert_near(t20_af(qts, 7, 5) & 0xffff, 22440, 15);
    g_assert_cmpuint(t20_af(qts, 7, 5) >> 16, ==, 57);
    t20_pattern(qts, 0x2000, 0x2800, 1, 1080, 705, 1216);
    assert_near(t20_af(qts, 7, 5) & 0xffff, 5114, 10);
    assert_near(t20_af(qts, 7, 5) >> 16, 12, 10);
    t20_pattern(qts, 0x4000, 0x5000, 541, 1080, 1, 1920);
    assert_near(t20_af(qts, 7, 5) & 0xffff, 3378, 15);
    assert_near(t20_af(qts, 7, 5) >> 16, 26, 15);
    g_assert_cmphex(t20_af(qts, 8, 5), ==, 0);

    /* the scene, once the test pattern is off */
    qtest_writel(qts, CORE + 0x40, 1);
    qtest_clock_step(qts, FRAME_NS);
    v = 0;
    for (int z = 0; z < 225; z++) {
        v |= t20_af(qts, 0, z);
    }
    g_assert_cmphex(v, !=, 0);
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
    qtest_add_func("/ingenic-isp-stats/t20/ae-zones", test_t20_ae_zones);
    qtest_add_func("/ingenic-isp-stats/t20/awb-weights", test_t20_awb_weights);
    qtest_add_func("/ingenic-isp-stats/t20/af", test_t20_af);
    qtest_add_func("/ingenic-isp-stats/t21", test_t21);
    return g_test_run();
}
