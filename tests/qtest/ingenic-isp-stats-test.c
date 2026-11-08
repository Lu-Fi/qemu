/*
 * QTest testcase for the older ISP statistics: the T20's Apical metering
 * (the T10's and T30's are the same core) and the T21's Tiziano blocks
 *
 * The layouts and counts a T20X and a T21N write for a 1920x1080 frame:
 * the T20's histogram counts half the pixels, each by its zone's weight
 * (15 counting 16), in Apical floats, each AE zone its own 5-bin one of
 * all its pixels, and an AWB zone of the 15x15 grid counts each of its
 * 128x72 pixels, the whole picture's by the zones' weights, and its AF
 * and motion vector zones read what a T20X does looking at its test
 * pattern; the T21's histogram is 20-bit bins in pairs tagged with the AE
 * block's frame count, its AWB zones the R, G and B sums and their number,
 * tagged with that count less one, and it leaves the IR half of the
 * histogram buffer alone. The T41's AF, ADR, defog, TMO, LCE and GSM
 * blocks write the layouts a T41NQ's do, with its tags, the RAM ports' reads
 * and the LCE's two banks, and its WDR block as its driver reads it. The
 * T31's and T41's defog and AF sums are of the picture the white-balance
 * gains have been on. The T31's AWB sums are of the raw signal, a grey's
 * R/G and B/G those of the sensor model's tuning file in the light's
 * temperature, and so are the T10's, T20's and T30's, from their Apical
 * tuning files.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "qobject/qdict.h"

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

static QTestState *t20_start_on(const char *machine, uint32_t vic,
                                const char *sensor)
{
    QTestState *qts = qtest_initf("-machine %s -device "
                                 "image-sensor,bus=i2c0,%s", machine, sensor);

    qtest_writel(qts, CPM_CLKGR0, 0);
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
    qtest_writel(qts, ISP + vic, 1);                /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    return qts;
}

static QTestState *t20_start(void)
{
    return t20_start_on("ingenic-t20", 0, "model=jxq03");
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
 * A grey in the AWB statistics is the sensor's own: its Apical table's raw
 * R/G and B/G (1/1024ths) at the light's temperature, which the zone of the
 * ramp reads back as G/R and G/B through unit gains
 */
typedef struct T20Response {
    const char *machine;
    uint32_t vic;                       /* its VIC start register */
    const char *model;
    unsigned kelvin, r, b;
} T20Response;

/* sensors without an exposure model, which show the scene as it is */
static const T20Response t20_responses[] = {
    { "ingenic-t10", 0x20, "jxh42", 6500, 518, 678 },
    { "ingenic-t10", 0x20, "ov9712", 5000, 582, 523 },
    { "ingenic-t20", 0, "sc2135", 6500, 577, 901 },
    { "ingenic-t20", 0, "sc2135", 4000, 812, 630 },
    { "ingenic-t20", 0, "ov2710", 4000, 755, 472 },
    { "ingenic-t30", 0, "jxh62", 5000, 576, 592 },
    { "ingenic-t30", 0, "sc2232", 6500, 596, 739 },
};

static void t20_grey(QTestState *qts, uint32_t *gr, uint32_t *gb)
{
    uint32_t zone;

    qtest_writel(qts, CORE + 0x300, 0x100);
    qtest_writel(qts, CORE + 0x30c, 0x100);
    qtest_clock_step(qts, FRAME_NS);
    zone = qtest_readl(qts, CORE + 0x8740 + (14 * 15 + 7) * 8);
    *gr = extract32(zone, 0, 12);
    *gb = extract32(zone, 16, 12);
}

static void test_t20_awb_response(const void *data)
{
    const T20Response *t = data;
    g_autofree char *props = g_strdup_printf("model=%s,kelvin=%u", t->model,
                                             t->kelvin);
    QTestState *qts = t20_start_on(t->machine, t->vic, props);
    uint32_t gr, gb;

    t20_grey(qts, &gr, &gb);
    /* 256 / (R/G) and 256 / (B/G), the 8-bit scene's rounding allowed for */
    assert_near(gr, 256 * 1024 / t->r, 3);
    assert_near(gb, 256 * 1024 / t->b, 3);
    qtest_quit(qts);
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

/* MVE as the T20X's firmware sets it up, at tap 1 (before demosaic) */
static void t20_mve_setup(QTestState *qts)
{
    static const uint32_t regs[][2] = {
        { 0x54, 1 }, { 0x900, W }, { 0x904, H }, { 0x908, 0x804 },
        { 0x90c, 16 }, { 0x910, 16 }, { 0x914, 32 }, { 0x918, 32 },
        { 0x91c, 12 }, { 0x920, 12 }, { 0x924, 0x100008 },
        { 0x928, 0x400020 }, { 0x930, 0x3020101 }, { 0x934, 4 },
        { 0x938, 0x3ff }, { 0x93c, 0x3ff }, { 0x940, 0x7ff },
        { 0x944, 0xbff }, { 0x948, 0xfff }, { 0x950, 4 },
    };

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        qtest_writel(qts, CORE + regs[i][0], regs[i][1]);
    }
}

/*
 * Zone (@row, @col): the vector a T20X reads, (@x, @y), and its costs,
 * the least and the mean, within 3 percent
 */
static void assert_mve(QTestState *qts, int row, int col, int x, int y,
                       uint32_t least, uint32_t mean)
{
    uint32_t z = CORE + 0x95c0 + (row * 4 + col) * 8;
    uint32_t v = qtest_readl(qts, z), c = qtest_readl(qts, z + 4);

    g_assert_cmphex(v, ==, (uint8_t)x | (uint8_t)y << 16);
    assert_near(c & 0xffff, least, 3);
    assert_near(c >> 16, mean, 3);
}

static void assert_mve_all(QTestState *qts, int x, int y, uint32_t cost)
{
    for (int z = 0; z < 32; z++) {
        g_assert_cmphex(qtest_readl(qts, CORE + 0x95c0 + z * 8), ==,
                        (uint8_t)x | (uint8_t)y << 16);
        g_assert_cmphex(qtest_readl(qts, CORE + 0x95c4 + z * 8), ==,
                        cost | cost << 16);
    }
}

/*
 * The motion vectors against a T20X looking at its test pattern: it keeps
 * no reference frame, its costs those of the current frame alone
 */
static void test_t20_mve(void)
{
    QTestState *qts = t20_start();

    t20_mve_setup(qts);
    /* flat: each feature's level >> 7 times its scale and 8 / length */
    t20_pattern(qts, 0x4000, 0x4000, 1, 1080, 1, 1920);
    assert_mve_all(qts, -12, -12, 544);
    qtest_writel(qts, CORE + 0x930, 0x1010101);
    qtest_writel(qts, CORE + 0x934, 1);
    for (int i = 0; i < 5; i++) {
        qtest_writel(qts, CORE + 0x938 + i * 4, 0xfff);
    }
    t20_pattern(qts, 0xffff, 0xffff, 1, 1080, 1, 1920);
    assert_mve_all(qts, -12, -12, 511 + 511 + 255 + 127 + 63);
    qtest_writel(qts, CORE + 0x930, 0x30000);
    qtest_writel(qts, CORE + 0x934, 0);
    t20_pattern(qts, 0xffff, 0xffff, 1, 1080, 1, 1920);
    assert_mve_all(qts, -12, -12, 3 * 255);
    /* every cost 4095: none taken, (0, 0) */
    qtest_writel(qts, CORE + 0x930, 0xf0f0f0f);
    qtest_writel(qts, CORE + 0x934, 15);
    t20_pattern(qts, 0xffff, 0xffff, 1, 1080, 1, 1920);
    assert_mve_all(qts, 0, 0, 4095);
    /* clipped after the scale */
    qtest_writel(qts, CORE + 0x930, 0x200);
    qtest_writel(qts, CORE + 0x934, 0);
    qtest_writel(qts, CORE + 0x93c, 64);
    t20_pattern(qts, 0x4000, 0x4000, 1, 1080, 1, 1920);
    assert_mve_all(qts, -12, -12, 64);
    t20_mve_setup(qts);

    /* a dark 12x12 square: its right and top edges set the vector */
    t20_pattern(qts, 0x4000, 0x1000, 91, 102, 48, 59);
    assert_mve(qts, 2, 0, -12, -12, 544, 544);
    assert_mve(qts, 2, 1, -5, -4, 289, 469);
    assert_mve(qts, 2, 2, -12, -4, 509, 531);
    assert_mve(qts, 2, 3, -12, -4, 531, 537);
    t20_pattern(qts, 0x4000, 0x1000, 91, 102, 84, 95);
    assert_mve(qts, 2, 2, -1, -4, 289, 467);
    t20_pattern(qts, 0x4000, 0x1000, 60, 71, 59, 70);
    assert_mve(qts, 1, 1, 6, -3, 289, 480);
    t20_pattern(qts, 0x4000, 0x1000, 105, 116, 59, 70);
    assert_mve(qts, 2, 1, 6, 10, 289, 527);
    assert_mve(qts, 3, 1, 5, -12, 352, 538);
    /* a bright one draws no vector, only the mean */
    t20_pattern(qts, 0x4000, 0xc000, 91, 102, 51, 62);
    assert_mve(qts, 2, 1, -12, -12, 544, 737);
    assert_mve(qts, 2, 2, -12, -12, 544, 568);
    /* moved each frame, it reads where it stands, as standing still */
    t20_pattern(qts, 0x4000, 0x1000, 91, 102, 57, 68);
    assert_mve(qts, 2, 1, 4, -4, 289, 475);

    /* other zones: 64x48 from (100, 40) */
    qtest_writel(qts, CORE + 0x90c, 100);
    qtest_writel(qts, CORE + 0x910, 40);
    qtest_writel(qts, CORE + 0x914, 64);
    qtest_writel(qts, CORE + 0x918, 48);
    t20_pattern(qts, 0x4000, 0x1000, 70, 81, 124, 135);
    assert_mve(qts, 0, 0, 3, 7, 289, 508);

    /* a search the zones cannot hold */
    qtest_writel(qts, CORE + 0x914, 16);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x95c0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x95c4), ==, 4095);

    /* no dump keeps the words, a restart zeroes them */
    t20_mve_setup(qts);
    qtest_writel(qts, CORE + 0x950, 0);
    t20_pattern(qts, 0x4000, 0x4000, 1, 1080, 1, 1920);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x95c4), ==, 4095);
    qtest_writel(qts, CORE + 0x950, 6);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, CORE + 0x95c4), ==, 0);
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

static void t21_stats(bool enable)
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
        if (enable) {
            qtest_writel(qts, b, 1);
        }
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

static void test_t21(void)
{
    t21_stats(true);
}

/* A T21N's tx-isp never writes +0x800 and its blocks read 0 meanwhile */
static void test_t21_free(void)
{
    t21_stats(false);
}

/* A T21's AWB zone of the ramp's last row: R, G and B */
static void t21_awb_sums(const char *props, uint64_t sum[3])
{
    QTestState *qts = qtest_initf("-machine ingenic-t21 -device "
                                  "image-sensor,model=jxf37,bus=i2c0,%s",
                                  props);
    uint32_t zone[4], v;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    for (uint64_t b = ISP + 0x700; b <= ISP + 0x800; b += 0x100) {
        qtest_writel(qts, b + 0x4, 0xf001f001);
        for (int i = 0; i < 4; i++) {
            qtest_writel(qts, b + 0x8 + i * 4, 0x40404040);
            qtest_writel(qts, b + 0x18 + i * 4, 0x24242424);
        }
        qtest_writel(qts, b, 1);
    }
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
    qtest_writel(qts, ISP + 0x80, 7 << 16);
    qtest_writel(qts, ISP + 0x10000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, T21_AWB + 224 * 16, zone, sizeof(zone));
    for (int c = 0; c < 3; c++) {
        bits_get(zone, c * 21, 21, &v);
        sum[c] = v;
    }
    qtest_quit(qts);
}

static void test_t21_awb_grey(void)
{
    double want = 0;

    for (unsigned luma = 200; luma >= 80; luma -= 60) {
        g_autofree char *props = g_strdup_printf("kelvin=4000,grey=%u", luma);
        uint64_t sum[3];

        t21_awb_sums(props, sum);
        g_assert_cmpuint(sum[1], >, 0);
        want = want ?: (double)sum[2] / sum[1];
        g_assert_cmpfloat_with_epsilon((double)sum[2] / sum[1], want,
                                       want * 0.01);
    }
}

#define T31_ADR         0x01000000      /* 4 buffers of 4 KiB from each */
#define T31_DEFOG       0x01010000
#define T31_AF          0x01020000
#define T31_AE          0x01030000
#define T31_ADR_HIST    194
#define T31_ADR_END     0x218

static const uint32_t t31_adr_rows[7] = { 0, 66, 270, 540, 810, 1014, H };
static const uint32_t t31_adr_cols[9] = {
    0, 80, 320, 640, 960, 1280, 1600, 1840, W,
};

static void check(QDict *r)
{
    g_assert_false(qdict_haskey(r, "error"));
    qobject_unref(r);
}

static void t31_light(QTestState *qts, unsigned v)
{
    check(qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': { "
                    "'path': '/machine/peripheral/cam', 'property': 'light', "
                    "'value': %u } }", v));
}

static void t31_bufs(QTestState *qts, uint32_t count, uint32_t base)
{
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + count + 4 + i * 4, base + i * 0x1000);
    }
    qtest_writel(qts, ISP + count, 3);
}

/*
 * A T31 streaming 1920x1080 with its ADR, defog and AF blocks set up as
 * its tx-isp firmware sets them (tisp_init, tiziano_adr_init,
 * tiziano_defog_init, tiziano_af_set_hardware_param)
 */
static QTestState *t31_start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 -device "
                                 "image-sensor,model=jxq03,bus=i2c0,id=cam");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    qtest_writel(qts, ISP + 0xb0, 0xffffffff);
    /* AE, for the frames' tag */
    qtest_writel(qts, ISP + 0xa004, 0xf001f001);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0xa008 + i * 4, 0x40404040);
        qtest_writel(qts, ISP + 0xa018 + i * 4, 0x24242424);
    }
    qtest_writel(qts, ISP + 0xa02c, T31_AE);
    qtest_writel(qts, ISP + 0xa03c, T31_AE + 0x1000);
    qtest_writel(qts, ISP + 0xa000, 1);
    /* ADR: margins, the tone map's grid, thresholds */
    qtest_writel(qts, ISP + 0x4010, 270 << 16);
    qtest_writel(qts, ISP + 0x4014, 810 << 16 | 540);
    qtest_writel(qts, ISP + 0x401c, 320 << 16);
    qtest_writel(qts, ISP + 0x4020, 960 << 16 | 640);
    qtest_writel(qts, ISP + 0x4024, 1600 << 16 | 1280);
    qtest_writel(qts, ISP + 0x4454, 1014 << 16 | 66);
    qtest_writel(qts, ISP + 0x4458, 1840 << 16 | 80);
    qtest_writel(qts, ISP + 0x4484, 0x015e009d);
    qtest_writel(qts, ISP + 0x4488, 0x03f90260);
    t31_bufs(qts, 0x4490, T31_ADR);
    /* defog: 10 x 18 zones, 108 rows and 106 or 107 columns */
    for (int i = 0; i <= 10; i += 2) {
        qtest_writel(qts, ISP + 0x5800 + i * 2,
                     i * 108 | (i < 10 ? (i + 1) * 108 << 16 : 0));
    }
    for (int i = 0; i <= 18; i += 2) {
        qtest_writel(qts, ISP + 0x5820 + i * 2,
                     i * W / 18 | (i < 18 ? (i + 1) * W / 18 << 16 : 0));
    }
    qtest_writel(qts, ISP + 0x5b10, 0x00ab0054);
    t31_bufs(qts, 0x5b80, T31_DEFOG);
    /* AF: 8 x 8 zones of 238 x 134 from (1, 3) */
    qtest_writel(qts, ISP + 0xb804, 0x80018003);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0xb808 + i * 4, 0xeeeeeeee);
        qtest_writel(qts, ISP + 0xb818 + i * 4, 0x86868686);
    }
    qtest_writel(qts, ISP + 0xb828, 0xc8f1);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0xb8a8 + i * 4, T31_AF + i * 0x1000);
    }
    qtest_writel(qts, ISP + 0xb8b8, 3);
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    return qts;
}

/* The last buffer a block wrote, by the address or index it left */
static uint32_t t31_last(QTestState *qts, uint32_t reg, uint32_t base,
                         bool index)
{
    uint32_t v = qtest_readl(qts, ISP + reg);

    if (index) {
        g_assert_cmpuint(v, <, 4);
        return base + v * 0x1000;
    }
    g_assert_cmphex(v & ~0x3000, ==, base);
    return v;
}

static uint32_t t31_tag(QTestState *qts)
{
    return extract32(qtest_readl(qts, ISP + 0xa050), 8, 8);
}

static void t31_adr(QTestState *qts, uint32_t *w)
{
    qtest_memread(qts, t31_last(qts, 0x44b0, T31_ADR, false), w, 4096);
    for (int i = 0; i < 1024; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
}

static uint64_t t31_adr_luma(const uint32_t *w)
{
    uint64_t sum = 0;

    for (int z = 0; z < 48; z++) {
        sum += w[2 + z * 4 + 2] & 0xfffffff;
    }
    return sum;
}

static void test_t31_adr(void)
{
    QTestState *qts = t31_start();
    uint32_t *w = g_new(uint32_t, 1024), v, sum = 0;
    uint64_t lit;

    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4) & BIT(18), ==, BIT(18));
    t31_adr(qts, w);
    g_assert_cmpuint(w[0], ==, t31_tag(qts));
    g_assert_cmpuint(w[0], >, 0);
    g_assert_cmpuint(w[1], ==, 0);
    g_assert_cmpuint(w[T31_ADR_END], ==, w[0]);
    /* each zone's 5 bins count its quads, as a T31X's do */
    for (int r = 0; r < 6; r++) {
        for (int c = 0; c < 8; c++) {
            const uint32_t *z = w + 2 + (r * 8 + c) * 4;
            uint32_t n = (z[0] & 0xffff) + (z[0] >> 16) + (z[1] & 0xffff) +
                         (z[1] >> 16) + (z[3] & 0xffff);

            g_assert_cmpuint(n, ==,
                             (t31_adr_rows[r + 1] - t31_adr_rows[r]) / 2 *
                             ((t31_adr_cols[c + 1] - t31_adr_cols[c]) / 2));
            g_assert_cmpuint(z[2] & 0xfffffff, <=, n * 4095);
        }
    }
    /* the histogram: every quad of the frame, 512 bins */
    for (int i = 0; i < 512; i++) {
        bits_get(w + T31_ADR_HIST, i / 3 * 64 + i % 3 * 21, 21, &v);
        sum += v;
    }
    g_assert_cmpuint(sum, ==, W * H / 4);

    /* the thresholds: all at 0, every quad above the last */
    qtest_writel(qts, ISP + 0x4484, 0);
    qtest_writel(qts, ISP + 0x4488, 0);
    qtest_clock_step(qts, FRAME_NS);
    t31_adr(qts, w);
    g_assert_cmphex(w[2 + 27 * 4], ==, 0);
    g_assert_cmphex(w[2 + 27 * 4 + 1], ==, 0);
    g_assert_cmpuint(w[2 + 27 * 4 + 3], ==, 270 / 2 * (320 / 2));

    /* linear light: half the light, half the sums */
    lit = t31_adr_luma(w);
    t31_light(qts, 50);
    qtest_clock_step(qts, FRAME_NS);
    t31_adr(qts, w);
    g_assert_cmpuint(t31_adr_luma(w) * 100, >, lit * 45);
    g_assert_cmpuint(t31_adr_luma(w) * 100, <, lit * 55);
    g_free(w);
    qtest_quit(qts);
}

static void t31_defog(QTestState *qts, uint32_t *w)
{
    qtest_memread(qts, t31_last(qts, 0x5ba4, T31_DEFOG, false), w, 4096);
    for (int i = 0; i < 1024; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
}

static void test_t31_defog(void)
{
    QTestState *qts = t31_start();
    uint32_t *w = g_new(uint32_t, 1024);

    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4) & BIT(20), ==, BIT(20));
    t31_defog(qts, w);
    g_assert_cmpuint(w[0], ==, t31_tag(qts));
    g_assert_cmpuint(w[2 + 180 * 4], ==, w[0]);
    for (int r = 0; r < 10; r++) {
        for (int c = 0; c < 18; c++) {
            const uint32_t *z = w + 2 + (r * 18 + c) * 4;
            uint32_t n = (z[2] & 0xffff) + (z[2] >> 16) + (z[3] & 0xffff);
            uint32_t b = z[0] & 0xffffff;
            uint32_t g = (z[1] & 0xffff) << 8 | z[0] >> 24;
            uint32_t red = extract32(z[3], 16, 12) << 12 |
                           extract32(z[1], 16, 12);

            g_assert_cmpuint(n, ==, 108 * ((c + 1) * W / 18 - c * W / 18));
            g_assert_cmpuint(b, <=, n * 255);
            /* the ramp below the bars is grey */
            if (r == 9) {
                g_assert_cmpuint(abs((int)(red - g)), <, n);
                g_assert_cmpuint(abs((int)(b - g)), <, n);
            }
        }
    }
    /* thresholds at 0: every pixel above both */
    qtest_writel(qts, ISP + 0x5b10, 0);
    qtest_clock_step(qts, FRAME_NS);
    t31_defog(qts, w);
    g_assert_cmphex(w[2 + 2], ==, 0);
    g_assert_cmpuint(w[2 + 3] & 0xffff, ==, 108 * (W / 18));
    g_free(w);
    qtest_quit(qts);
}

/* The R, G and B sums of a defog zone (T31), 24 bits each */
static void t31_defog_rgb(const uint32_t *w, unsigned r, unsigned c,
                          uint32_t rgb[3])
{
    const uint32_t *z = w + 2 + (r * 18 + c) * 4;

    rgb[2] = z[0] & 0xffffff;
    rgb[1] = (z[1] & 0xffff) << 8 | z[0] >> 24;
    rgb[0] = extract32(z[3], 16, 12) << 12 | extract32(z[1], 16, 12);
}

static uint32_t t31_af_ysum(QTestState *qts, unsigned zone)
{
    uint32_t w[256], v;

    qtest_memread(qts, t31_last(qts, 0xb8bc, T31_AF, true), w, sizeof(w));
    for (int i = 0; i < 256; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
    bits_get(w + zone * 4, 88, 23, &v);
    return v;
}

static void test_t31_gains(void)
{
    QTestState *qts = t31_start();
    uint32_t *w = g_new(uint32_t, 1024), rgb[3], ysum;

    /* the ramp is grey; the zones' sums are of the picture put out */
    t31_defog(qts, w);
    t31_defog_rgb(w, 9, 9, rgb);
    g_assert_cmpuint(abs((int)(rgb[0] - rgb[1])), <, rgb[1] / 50);
    g_assert_cmpuint(abs((int)(rgb[2] - rgb[1])), <, rgb[1] / 50);
    ysum = t31_af_ysum(qts, 63);

    /* the AWB's gains at 1.0: the raw R and B, short of a grey's */
    qtest_writel(qts, ISP + 0x1804, 0x04000400);
    qtest_writel(qts, ISP + 0x1808, 0x04000400);
    qtest_clock_step(qts, FRAME_NS);
    t31_defog(qts, w);
    t31_defog_rgb(w, 9, 9, rgb);
    g_assert_cmpuint(rgb[0], <, rgb[1] * 9 / 10);
    g_assert_cmpuint(rgb[2], <, rgb[1] * 9 / 10);
    g_assert_cmpuint(t31_af_ysum(qts, 63), <, ysum * 95 / 100);

    /* and more red than that */
    qtest_writel(qts, ISP + 0x1804, 0x04000800);
    qtest_clock_step(qts, FRAME_NS);
    t31_defog(qts, w);
    t31_defog_rgb(w, 9, 9, rgb);
    g_assert_cmpuint(rgb[0], >, rgb[1]);
    g_free(w);
    qtest_quit(qts);
}

/*
 * How close the sums of a ramp's mid zones are to a table's grey: 8-bit
 * linear light costs a few tenths of a percent, the warm light's clipped
 * red at the bright end about as much
 */
static double awb_tol = 0.015;
static unsigned awb_hi = 359;      /* the white region's R + B over G, /256 */

#define T31_AWB         0x01080000      /* 4 buffers of 4 KiB */

/*
 * The AWB block of a T23 or a T31 set up as a GC2053's tuning file for the
 * T31 has it: 15x15 zones of 64x36 quads, the white region's R/G and B/G
 * between 70 and 269 and 47 and 232 (1/256ths) with R + B between 0.77 and
 * 1.4 of G. The sums of the ramp's zones from the 4th to the 10th of its
 * last row are R, G, B and the quads.
 */
static void tiz_awb_sums(const char *machine, const char *model,
                         const char *props, uint64_t sum[4])
{
    QTestState *qts = qtest_initf("-machine %s -device "
                                  "image-sensor,model=%s,bus=i2c0,id=cam,"
                                  "motion=off,%s", machine, model, props);
    uint32_t w[4], idx;

    memset(sum, 0, 4 * sizeof(*sum));
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    qtest_writel(qts, ISP + 0xb0, 0xffffffff);
    qtest_writel(qts, ISP + 0xb004, 15u << 28 | 1 << 16 | 15 << 12 | 1);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0xb008 + i * 4, 0x40404040);
        qtest_writel(qts, ISP + 0xb018 + i * 4, 0x24242424);
    }
    qtest_writel(qts, ISP + 0xb028, 269 << 16 | 70);
    qtest_writel(qts, ISP + 0xb02c, 232 << 16 | 47);
    qtest_writel(qts, ISP + 0xb030, 197 << 16 | 256);
    qtest_writel(qts, ISP + 0xb034, awb_hi << 16 | 256);
    qtest_writel(qts, ISP + 0xb038, 1 << 16 | 240 << 8 | 9);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0xb03c + i * 4, T31_AWB + i * 0x1000);
    }
    qtest_writel(qts, ISP + 0xb04c, 3);
    qtest_writel(qts, ISP + 0xb000, 1);
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4) & BIT(30), ==, BIT(30));
    idx = qtest_readl(qts, ISP + 0xb050);
    g_assert_cmpuint(idx, <, 4);
    /* the zones where the warm light has not clipped the picture's red */
    for (int z = 14 * 15 + 3; z < 14 * 15 + 10; z++) {
        qtest_memread(qts, T31_AWB + idx * 0x1000 + z * 16, w, sizeof(w));
        for (int i = 0; i < 4; i++) {
            w[i] = le32_to_cpu(w[i]);
        }
        sum[0] += w[0] & 0x1fffff;
        sum[1] += w[0] >> 21 | (w[1] & 0x3ff) << 11;
        sum[2] += w[1] >> 10;
        sum[3] += (w[2] >> 20 | w[3] << 12) & 0x1fff;
    }
    qtest_quit(qts);
}

/* A grey's raw R/G and B/G, against the tuning files' tables */
static void tiz_awb_check(const char *machine, const char *model,
                          const char *props, double rg, double bg)
{
    uint64_t sum[4];

    tiz_awb_sums(machine, model, props, sum);
    g_assert_cmpuint(sum[3], >, 0);
    g_assert_cmpfloat_with_epsilon((double)sum[0] / sum[1], rg, rg * awb_tol);
    g_assert_cmpfloat_with_epsilon((double)sum[2] / sum[1], bg, bg * awb_tol);
}

static void test_t31_awb(void)
{
    uint64_t sum[4];

    /* a JXQ03's and an IMX307's tuning files: R/G and B/G at 6500 K ... */
    tiz_awb_check("ingenic-t31", "jxq03", "kelvin=6500", 556 / 1024.0,
                  751 / 1024.0);
    tiz_awb_check("ingenic-t31", "imx307", "kelvin=6500", 561 / 1024.0,
                  587 / 1024.0);
    /* ... and 3000 K */
    tiz_awb_check("ingenic-t31", "imx307", "kelvin=3000", 903 / 1024.0,
                  347 / 1024.0);
    /* an OS04D10's, steeper, has left the white region by then */
    tiz_awb_sums("ingenic-t31", "gc5603", "kelvin=3000", sum);
    g_assert_cmpuint(sum[3], ==, 0);
    /* a model with no table keeps an OS04D10's, and raw-r and raw-b set it */
    tiz_awb_check("ingenic-t31", "gc5603", "kelvin=6500", 144 / 256.0,
                  105 / 256.0);
    tiz_awb_check("ingenic-t31", "imx307", "kelvin=3000,raw-r=200,raw-b=100",
                  200 / 256.0, 100 / 256.0);
}

/* The T23's AWB block is the T31's: a GC1084's and a JXQ03P's T23 tables */
static void test_t23_awb(void)
{
    uint64_t sum[4];

    tiz_awb_check("ingenic-t23", "gc1084", "kelvin=6500", 564 / 1024.0,
                  573 / 1024.0);
    tiz_awb_check("ingenic-t23", "gc1084", "kelvin=3000", 821 / 1024.0,
                  393 / 1024.0);
    /* a big factor on a small G, and a grey outside the window: widened */
    awb_hi = 400;
    tiz_awb_check("ingenic-t23", "jxq03p", "kelvin=6500", 723 / 1024.0,
                  747 / 1024.0);
    awb_hi = 359;
    /* a model with no table on the T23 (a JXQ03's are the T31's and T40's) */
    tiz_awb_sums("ingenic-t23", "jxq03", "kelvin=3000", sum);
    g_assert_cmpuint(sum[3], ==, 0);
    tiz_awb_check("ingenic-t23", "jxq03", "kelvin=6500", 144 / 256.0,
                  105 / 256.0);
}

static void test_t31_af(void)
{
    QTestState *qts = t31_start();
    uint32_t w[256], v, tag = 0, flat = 0;

    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4) & BIT(31), ==, BIT(31));
    qtest_memread(qts, t31_last(qts, 0xb8bc, T31_AF, true), w, sizeof(w));
    for (int i = 0; i < 256; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
    for (int z = 0; z < 4; z++) {
        bits_get(w + z * 4, 126, 2, &v);
        tag |= v << (z * 2);
    }
    g_assert_cmpuint(tag, ==, t31_tag(qts));
    for (int z = 0; z < 64; z++) {
        uint32_t ysum;

        bits_get(w + z * 4, 88, 23, &ysum);
        g_assert_cmpuint(ysum, >=, 16 * 238 * 134);
        g_assert_cmpuint(ysum, <=, 235 * 238 * 134);
        /* the bars' rows are all alike: no vertical edges in them */
        if (z < 16) {
            bits_get(w + z * 4, 0, 22, &v);
            g_assert_cmpuint(v, ==, 0);
            bits_get(w + z * 4, 22, 22, &v);
            g_assert_cmpuint(v, ==, 0);
        }
        bits_get(w + z * 4, 44, 22, &v);
        flat += !v;
    }
    /* the bars' and the ramp's edges show in the horizontal filters */
    g_assert_cmpuint(flat, <, 64);

    /* bright above 0: every pixel */
    qtest_writel(qts, ISP + 0xb828, 0);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, t31_last(qts, 0xb8bc, T31_AF, true), w, sizeof(w));
    bits_get(w, 111, 15, &v);
    g_assert_cmpuint(v, ==, MIN(238 * 134, 0x7fff));
    qtest_quit(qts);
}

#define T41_W           1280
#define T41_H           720
#define T41_INT         0x40028
#define T41_AF          0x01000000      /* 4 buffers of 4 KiB from each */
#define T41_ADR         0x01010000
#define T41_DEFOG       0x01020000
#define T41_TMO         0x01030000      /* 2 of 16 KiB */
#define T41_ADR_HIST    (2 + 24 * 14)
#define T41_ADR_END     0x2a8
#define T41_SEC         1000000000LL

static void t41_wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, ISP + off, v);
}

static uint32_t t41_rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, ISP + off);
}

static void t41_frame(QTestState *qts)
{
    t41_wr(qts, T41_INT - 4, ~0u);
    qtest_clock_step(qts, T41_SEC);
}

/*
 * A T41 streaming 1280x720 at 1 fps with its statistics blocks set up as
 * its tx-isp firmware sets them for that size (tisp_af_set_hardware_param,
 * tisp_adr_init, tisp_defog_init, tisp_tmo_init, tisp_lce_init)
 */
static QTestState *t41_start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device image-sensor,"
                                 "model=gc4023,bus=i2c0,fps=1,motion=off");

    qtest_writel(qts, CPM_CLKGR0,
                 qtest_readl(qts, CPM_CLKGR0) & ~(1u << 23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~(1u << 27));
    t41_wr(qts, 0x80, T41_W << 16 | T41_H);
    t41_wr(qts, 0x40020, ~0u);
    /*
     * the gains that leave a grey's raw R and B (a GC4023's in a T41's
     * tuning file, 6500 K) as the scene has them
     */
    t41_wr(qts, 0x4004, 0x0400075b);
    t41_wr(qts, 0x4008, 0x04000637);
    /* AF: 15 x 15 zones of 84 x 47 from (1, 3) */
    t41_wr(qts, 0x1a004, 0xf001f003);
    for (int i = 0; i < 8; i++) {
        t41_wr(qts, 0x1a008 + i * 4, 84 << 16 | 84);
    }
    for (int i = 0; i < 4; i++) {
        t41_wr(qts, 0x1a028 + i * 4, 0x2f2f2f2f);
    }
    t41_wr(qts, 0x1a038, 0xc8f1);
    for (int i = 0; i < 4; i++) {
        t41_wr(qts, 0x1a0cc + i * 4, T41_AF + i * 0x1000);
        t41_wr(qts, 0x9514 + i * 4, T41_ADR + i * 0x1000);
        t41_wr(qts, 0xc290 + i * 4, T41_DEFOG + i * 0x800);
    }
    t41_wr(qts, 0x1a0dc, 0x10103);
    /* ADR: 4 x 6 zones */
    t41_wr(qts, 0x9030, 180 << 16);
    t41_wr(qts, 0x9034, 540 << 16 | 360);
    t41_wr(qts, 0x9038, T41_H);
    t41_wr(qts, 0x903c, 214 << 16);
    t41_wr(qts, 0x9040, 640 << 16 | 426);
    t41_wr(qts, 0x9044, 1066 << 16 | 852);
    t41_wr(qts, 0x9048, T41_W);
    t41_wr(qts, 0x9510, 3);
    /* defog: 10 x 18 zones of 72 x 70 */
    t41_wr(qts, 0xc008, 72 << 16 | 70);
    t41_wr(qts, 0xc284, 0);
    t41_wr(qts, 0xc28c, 3);
    /* TMO: 15 x 25 zones of 51 x 48 */
    t41_wr(qts, 0x1e010, T41_TMO);
    t41_wr(qts, 0x1e014, T41_TMO + 0x4000);
    t41_wr(qts, 0x1e080, 51 << 8 | 48);
    t41_wr(qts, 0x1e020, 1);
    /* LCE: 5 x 9 blocks of 142 x 144, the first column 2 wider */
    t41_wr(qts, 0xe000, 144 << 16 | 142);
    t41_wr(qts, 0xe004, 1);
    t41_wr(qts, 0x80000, 1);                    /* VIC start */
    qtest_clock_step(qts, T41_SEC);
    return qts;
}

static void t41_read(QTestState *qts, uint32_t addr, uint32_t *w, int n)
{
    qtest_memread(qts, addr, w, n * 4);
    for (int i = 0; i < n; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
}

static uint32_t bits_at(const uint32_t *w, unsigned at, unsigned n)
{
    uint32_t v;

    bits_get(w, at, n, &v);
    return v;
}

static void test_t41_af(void)
{
    QTestState *qts = t41_start();
    uint32_t idx = t41_rd(qts, 0x1a0e0), w[1024];

    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(6), ==, BIT(6));
    g_assert_cmpuint(idx, <, 4);
    t41_read(qts, T41_AF + idx * 0x1000, w, 1024);
    for (int y = 0; y < 15; y++) {
        /* rows of 15 zones of 17 bytes, padded to 256, the tag last */
        g_assert_cmpuint(bits_at(w + y * 64, 255 * 8, 8), ==,
                         t41_rd(qts, 0x1a120) & 0xff);
        for (int x = 0; x < 15; x++) {
            uint32_t ysum = bits_at(w + y * 64, x * 136 + 92, 24);

            g_assert_cmpuint(ysum, >=, 16 * 84 * 47);
            g_assert_cmpuint(ysum, <=, 235 * 84 * 47);
        }
    }
    /* bright above 0: every pixel */
    t41_wr(qts, 0x1a038, 0);
    t41_frame(qts);
    t41_read(qts, T41_AF + t41_rd(qts, 0x1a0e0) * 0x1000, w, 64);
    g_assert_cmpuint(bits_at(w, 116, 16), ==, 84 * 47);
    qtest_quit(qts);
}

static void test_t41_gains(void)
{
    QTestState *qts = t41_start();
    uint32_t w[512], af[64], z[3], ysum;

    t41_read(qts, t41_rd(qts, 0xc2b4), w, 512);
    for (int i = 0; i < 3; i++) {
        z[i] = bits_at(w + 2, i * 21, 21);
    }
    g_assert_cmpuint(z[0], ==, z[2]);
    t41_read(qts, T41_AF + t41_rd(qts, 0x1a0e0) * 0x1000, af, 64);
    ysum = bits_at(af, 92, 24);

    /* the gains' reset value, 1.0, short of a grey's: blue and luma fall */
    t41_wr(qts, 0x4004, 0x04000400);
    t41_wr(qts, 0x4008, 0x04000400);
    t41_frame(qts);
    t41_read(qts, t41_rd(qts, 0xc2b4), w, 512);
    for (int i = 0; i < 3; i++) {
        z[i] = bits_at(w + 2, i * 21, 21);
    }
    g_assert_cmpuint(z[0], <, z[1] * 9 / 10);
    g_assert_cmpuint(z[2], <, z[1] * 9 / 10);
    t41_read(qts, T41_AF + t41_rd(qts, 0x1a0e0) * 0x1000, af, 64);
    g_assert_cmpuint(bits_at(af, 92, 24), <, ysum * 95 / 100);
    qtest_quit(qts);
}

#define T41_AWB         0x01060000      /* 4 buffers of 32 KiB */

/*
 * The T41's AWB block (tisp_awb_set_hardware_param) set up for 15x15 zones of
 * 42x23 quads and the white region of a GC2053's tuning file for the T31;
 * each zone's four luma bands give a record of all their quads and one of
 * their white ones. The sums of all quads of the ramp's zones from the 4th
 * to the 10th of its last row, over the bands, are R, G and B.
 */
static const char *t41_machine = "ingenic-t41";

static void t41_awb_sums(const char *model, const char *props,
                         uint64_t sum[3])
{
    QTestState *qts = qtest_initf("-machine %s -device "
                                  "image-sensor,model=%s,bus=i2c0,fps=1,"
                                  "motion=off,%s", t41_machine, model, props);
    uint32_t w[8], idx;

    memset(sum, 0, 3 * sizeof(*sum));
    qtest_writel(qts, CPM_CLKGR0,
                 qtest_readl(qts, CPM_CLKGR0) & ~(1u << 23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~(1u << 27));
    t41_wr(qts, 0x80, T41_W << 16 | T41_H);
    t41_wr(qts, 0x40020, ~0u);
    t41_wr(qts, 0x18004, 15u << 28 | 1 << 16 | 15 << 12 | 1);
    for (int i = 0; i < 4; i++) {
        t41_wr(qts, 0x18008 + i * 4, 0x2a2a2a2a);
        t41_wr(qts, 0x18018 + i * 4, 0x17171717);
    }
    t41_wr(qts, 0x18028, 269 << 16 | 70);
    t41_wr(qts, 0x1802c, 232 << 16 | 47);
    t41_wr(qts, 0x18030, 197 << 16 | 256);
    t41_wr(qts, 0x18034, 359 << 16 | 256);
    t41_wr(qts, 0x18038, 230 << 16 | 128 << 8 | 10);
    for (int i = 0; i < 4; i++) {
        t41_wr(qts, 0x1803c + i * 4, T41_AWB + i * 0x8000);
    }
    t41_wr(qts, 0x1804c, 3);
    t41_wr(qts, 0x18000, 1);
    t41_wr(qts, 0x80000, 1);                    /* VIC start */
    t41_frame(qts);
    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(3), ==, BIT(3));
    idx = t41_rd(qts, 0x18050);
    g_assert_cmpuint(idx, <, 4);
    for (int z = 14 * 15 + 3; z < 14 * 15 + 10; z++) {
        for (int band = 0; band < 4; band++) {
            t41_read(qts, T41_AWB + idx * 0x8000 + z * 128 + band * 32, w, 4);
            for (int c = 0; c < 3; c++) {
                sum[c] += bits_at(w, c * 22, 22);
            }
        }
    }
    qtest_quit(qts);
}

static void t41_awb_check(const char *model, const char *props, double rg,
                          double bg)
{
    uint64_t sum[3];

    t41_awb_sums(model, props, sum);
    g_assert_cmpuint(sum[1], >, 0);
    g_assert_cmpfloat_with_epsilon((double)sum[0] / sum[1], rg, rg * awb_tol);
    g_assert_cmpfloat_with_epsilon((double)sum[2] / sum[1], bg, bg * awb_tol);
}

static void test_t41_awb(void)
{
    /* a GC4023's T41 tuning file: R/G and B/G at 6500 K and 3000 K */
    t41_awb_check("gc4023", "kelvin=6500", 557 / 1024.0, 659 / 1024.0);
    t41_awb_check("gc4023", "kelvin=3000", 873 / 1024.0, 453 / 1024.0);
    /* a model with no table on the T41 keeps an OS04D10's */
    t41_awb_check("jxq03", "kelvin=6500", 144 / 256.0, 105 / 256.0);
}

#define T32_AWB         0x01070000      /* 4 buffers of 8 KiB */
#define T32_ROW_WORDS   (3 * 26 + 3 * 6 + 2)

/*
 * The T32's AWB block (tisp_awb_set_hardware_param): 15x15 zones of 42x23
 * quads and the white region of a GC2053's tuning file for the T31. After 8
 * bytes, a row is groups of 4 zones of 6 words and 2 after them, and the 3
 * zones left; the R, G and B sums of all the zone's quads are its first 66
 * bits. The sums of the ramp's zones from the 4th to the 10th of its last
 * row are R, G and B.
 */
static void t32_awb_sums(const char *model, const char *props,
                         uint64_t sum[3])
{
    QTestState *qts = qtest_initf("-machine ingenic-t32 -device "
                                  "image-sensor,model=%s,bus=i2c0,fps=1,"
                                  "motion=off,%s", model, props);
    uint32_t w[T32_ROW_WORDS], idx;

    memset(sum, 0, 3 * sizeof(*sum));
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, 0x10000090, 0);
    t41_wr(qts, 0x80, T41_W << 16 | T41_H);
    t41_wr(qts, 0x160f4, 23 << 16 | 42);
    t41_wr(qts, 0x16064, 269 << 16 | 70);
    t41_wr(qts, 0x16068, 232 << 16 | 47);
    t41_wr(qts, 0x1606c, 197 << 16 | 256);
    t41_wr(qts, 0x16070, 359 << 16 | 256);
    for (int i = 0; i < 4; i++) {
        t41_wr(qts, 0x16078 + i * 4, T32_AWB + i * 0x2000);
    }
    t41_wr(qts, 0x16088, 3);
    t41_wr(qts, 0x16010, 1);
    t41_wr(qts, 0x80010, 1);                    /* VIC start */
    qtest_clock_step(qts, T41_SEC);
    g_assert_cmphex(t41_rd(qts, 0x30028) & BIT(3), ==, BIT(3));
    idx = t41_rd(qts, 0x1608c);
    g_assert_cmpuint(idx, <, 4);
    t41_read(qts, T32_AWB + idx * 0x2000 + 8 + 14 * T32_ROW_WORDS * 4, w,
             T32_ROW_WORDS);
    for (int c = 3; c < 10; c++) {
        const uint32_t *zw = w + c / 4 * 26 + c % 4 * 6;

        for (int f = 0; f < 3; f++) {
            sum[f] += bits_at(zw, f * 22, 22);
        }
    }
    qtest_quit(qts);
}

static void t32_awb_check(const char *model, const char *props, double rg,
                          double bg)
{
    uint64_t sum[3];

    t32_awb_sums(model, props, sum);
    g_assert_cmpuint(sum[1], >, 0);
    g_assert_cmpfloat_with_epsilon((double)sum[0] / sum[1], rg, rg * awb_tol);
    g_assert_cmpfloat_with_epsilon((double)sum[2] / sum[1], bg, bg * awb_tol);
}

static void test_t32_awb(void)
{
    /* a GC4023's T32 tuning file: R/G and B/G at 6500 K and 3000 K */
    t32_awb_check("gc4023", "kelvin=6500", 506 / 1024.0, 815 / 1024.0);
    t32_awb_check("gc4023", "kelvin=3000", 777 / 1024.0, 555 / 1024.0);
    /* a model with no table on the T32 keeps an OS04D10's */
    t32_awb_check("jxq03", "kelvin=6500", 144 / 256.0, 105 / 256.0);
}

static void test_t41_adr(void)
{
    static const uint32_t rows[5] = { 0, 180, 360, 540, T41_H };
    static const uint32_t cols[7] = { 0, 214, 426, 640, 852, 1066, T41_W };
    QTestState *qts = t41_start();
    uint32_t addr = t41_rd(qts, 0x9540), tag = t41_rd(qts, 0x9544);
    uint32_t *w = g_new(uint32_t, 1024), sum = 0;

    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(9), ==, BIT(9));
    g_assert_cmphex(addr & ~0x3000, ==, T41_ADR);
    t41_read(qts, addr, w, 1024);
    g_assert_cmpuint(w[0], ==, tag);
    g_assert_cmpuint(w[1], ==, 0);
    g_assert_cmpuint(w[T41_ADR_END], ==, tag);
    for (int z = 0; z < 24; z++) {
        const uint32_t *zw = w + 2 + z * 14;
        uint32_t quads = (rows[z / 6 + 1] - rows[z / 6]) / 2 *
                         ((cols[z % 6 + 1] - cols[z % 6]) / 2);
        uint32_t n = 0, most = bits_at(zw, 360, 12);
        uint32_t least = bits_at(zw, 372, 12);

        for (int b = 0; b < 20; b++) {
            n += bits_at(zw, b * 18, 18);
        }
        /* those above the last bin are left out */
        g_assert_cmpuint(n, <=, quads);
        g_assert_cmpuint(most, >=, least);
        g_assert_cmpuint(bits_at(zw, 384, 29), >=, (uint64_t)least * quads);
        g_assert_cmpuint(bits_at(zw, 384, 29), <=, (uint64_t)most * quads);
    }
    for (int i = 0; i < 512; i++) {
        sum += bits_at(w + T41_ADR_HIST, i / 3 * 64 + i % 3 * 21, 21);
    }
    g_assert_cmpuint(sum, ==, T41_W * T41_H / 4);
    g_free(w);
    qtest_quit(qts);
}

static void test_t41_defog(void)
{
    QTestState *qts = t41_start();
    uint32_t addr = t41_rd(qts, 0xc2b4), tag = t41_rd(qts, 0xc2b0);
    uint32_t w[512];
    uint64_t z0[3];

    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(10), ==, BIT(10));
    g_assert_cmphex(addr & ~0x1800, ==, T41_DEFOG);
    t41_read(qts, addr, w, 512);
    g_assert_cmpuint(w[0], ==, tag);
    g_assert_cmpuint(w[2 + 360], ==, tag);
    for (int i = 0; i < 3; i++) {
        z0[i] = bits_at(w + 2, i * 21, 21);
        g_assert_cmpuint(z0[i], <=, 255 * 72 * 70);
    }
    /* the first bar is white: R, G and B alike */
    g_assert_cmpuint(z0[0], ==, z0[2]);
    /* shifted by 0xc284 */
    t41_wr(qts, 0xc284, 2);
    t41_frame(qts);
    t41_read(qts, t41_rd(qts, 0xc2b4), w, 4);
    g_assert_cmpuint(bits_at(w + 2, 21, 21), ==, z0[1] >> 2);
    qtest_quit(qts);
}

static void test_t41_tmo(void)
{
    QTestState *qts = t41_start();
    uint32_t i = t41_rd(qts, 0x1e028), *w = g_new(uint32_t, 3750);

    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(17), ==, BIT(17));
    g_assert_cmpuint(i, <, 2);
    g_assert_cmphex(t41_rd(qts, 0x1e02c), ==, T41_TMO + i * 0x4000);
    t41_read(qts, T41_TMO + i * 0x4000, w, 3750);
    for (int y = 0; y < 15; y++) {
        for (int x = 0; x < 25; x++) {
            uint32_t n = 0;

            for (int b = 0; b < 10; b++) {
                uint32_t v = w[(y * 25 + x) * 10 + b];

                n += v >> 16;
                g_assert_cmpuint(v & 0xffff, <=, (v >> 16) * 818 >> 9);
            }
            /* the last column and row reach the frame's edges */
            g_assert_cmpuint(n, ==, (x < 24 ? 51 : T41_W - 24 * 51) *
                                    (y < 14 ? 48 : T41_H - 14 * 48));
        }
    }
    t41_frame(qts);
    g_assert_cmpuint(t41_rd(qts, 0x1e028), ==, !i);
    g_free(w);
    qtest_quit(qts);
}

/* A bank's histograms through its RAM port; the reads clear them */
static uint32_t t41_lce_hist(QTestState *qts, uint32_t port, uint32_t *n)
{
    uint32_t sum = 0;

    t41_wr(qts, port, 0x201);
    for (int b = 0; b < 45; b++) {
        n[b] = 0;
        for (int i = 0; i < 32; i++) {
            n[b] += t41_rd(qts, port + 8);
        }
        sum += n[b];
    }
    t41_wr(qts, port, 0x59f0202);
    return sum;
}

static void test_t41_lce(void)
{
    QTestState *qts = t41_start();
    uint32_t n[45];

    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(8), ==, BIT(8));
    g_assert_cmphex(t41_rd(qts, 0xe014), ==, BIT(0));
    /* every other pixel; the first column 2 wider */
    g_assert_cmpuint(t41_lce_hist(qts, 0x501c0, n), ==, 142 * 9 * 144 * 5 / 2 +
                                                       2 * 144 * 5 / 2);
    g_assert_cmpuint(n[0], ==, 144 * 144 / 2);
    g_assert_cmpuint(n[1], ==, 142 * 144 / 2);
    g_assert_cmpuint(t41_lce_hist(qts, 0x501c0, n), ==, 0);
    /* the sums: of the luma counted */
    t41_wr(qts, 0x50200, 0x201);
    g_assert_cmpuint(t41_rd(qts, 0x50208), >=, 16 * 144 * 144 / 2);
    t41_wr(qts, 0x50200, 0x2c0202);
    /* bank 1 next, bank 0 not again while unread */
    t41_frame(qts);
    g_assert_cmphex(t41_rd(qts, 0xe014), ==, BIT(16) | BIT(8) | BIT(0));
    t41_wr(qts, 0xe010, BIT(16));
    t41_wr(qts, 0xe010, 1);
    g_assert_cmphex(t41_rd(qts, 0xe014), ==, BIT(16));
    g_assert_cmpuint(t41_lce_hist(qts, 0x501d0, n), >, 0);
    t41_frame(qts);
    g_assert_cmphex(t41_rd(qts, 0xe014), ==, BIT(0));
    qtest_quit(qts);
}

static void test_t41_gsm(void)
{
    QTestState *qts = t41_start();
    uint32_t sum[2] = { 0 };

    /* nothing until armed, as the T41NQ's driver leaves it */
    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(7), ==, 0);
    g_assert_cmpuint(t41_rd(qts, 0x1b004), ==, 0);
    t41_wr(qts, 0x1b000, 1);
    t41_wr(qts, 0x1b040, 1);
    t41_frame(qts);
    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(7), ==, BIT(7));
    g_assert_cmpuint(t41_rd(qts, 0x1b004), ==, 1);
    g_assert_cmpuint(t41_rd(qts, 0x1b044), ==, 1);
    for (int h = 0; h < 2; h++) {
        t41_wr(qts, 0x501a0 + h * 0x10, 0x201);
        for (int i = 0; i < 256; i++) {
            sum[h] += t41_rd(qts, 0x501a8 + h * 0x10);
        }
        t41_wr(qts, 0x501a0 + h * 0x10, 0xff0202);
    }
    g_assert_cmpuint(sum[0], ==, T41_W * T41_H / 2);
    g_assert_cmpuint(sum[1], ==, T41_W * T41_H / 2);
    t41_wr(qts, 0x1b000, 1);
    g_assert_cmpuint(t41_rd(qts, 0x1b004), ==, 0);
    qtest_quit(qts);
}

#define T31_WDR         0x01040000      /* 4 buffers of 8 KiB */
#define T31_AE1         0x01060000
#define I2C0            0x10050000
#define I2C_DC          0x10
#define I2C_ENB         0x6c

/* A GC4653 register over I2C0 */
static void gc_write(QTestState *qts, uint16_t reg, uint8_t v)
{
    qtest_writel(qts, I2C0 + I2C_DC, reg >> 8);
    qtest_writel(qts, I2C0 + I2C_DC, reg & 0xff);
    qtest_writel(qts, I2C0 + I2C_DC, v | 0x200);
}

/*
 * A T31 streaming 1920x1080 from a GC4653 in its DOL WDR mode, long
 * exposure 1000 lines, short 250, in a room three times as bright as the
 * long one takes, set up as tisp_init and tiziano_ae_init set AE1 and the
 * WDR block; @wdr 0 leaves the ISP linear
 */
static QTestState *t31_wdr_start(bool wdr)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 -device image-sensor,"
                                 "model=gc4653,bus=i2c0,light=300,"
                                 "motion=off");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, I2C0 + I2C_ENB, 0);
    qtest_writel(qts, I2C0 + 0x04, 0x29);
    qtest_writel(qts, I2C0 + I2C_ENB, 1);
    gc_write(qts, 0x0107, 0x89);
    gc_write(qts, 0x0202, 0x03);
    gc_write(qts, 0x0203, 0xe8);
    gc_write(qts, 0x0200, 0x00);
    gc_write(qts, 0x0201, 0xfa);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    qtest_writel(qts, ISP + 0x10, wdr ? 0x33f : 0x133);
    qtest_writel(qts, ISP + 0xb0, 0xffffffff);
    for (int ae = 0xa000; ae <= 0xa800; ae += 0x800) {
        uint32_t buf = ae == 0xa000 ? T31_AE : T31_AE1;

        qtest_writel(qts, ISP + ae + 0x4, 0xf001f001);
        for (int i = 0; i < 4; i++) {
            qtest_writel(qts, ISP + ae + 0x8 + i * 4, 0x40404040);
            qtest_writel(qts, ISP + ae + 0x18 + i * 4, 0x24242424);
        }
        qtest_writel(qts, ISP + ae + 0x28, 0x11991e);
        qtest_writel(qts, ISP + ae + 0x2c, buf);
        qtest_writel(qts, ISP + ae + 0x3c, buf + 0x1000);
        qtest_writel(qts, ISP + ae, 1);
    }
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0x2010 + i * 4, T31_WDR + i * 0x2000);
    }
    qtest_writel(qts, ISP + 0x2020, 0x400);
    qtest_writel(qts, ISP + 0x2024, 3);
    qtest_writel(qts, ISP + 0x2030, 0x07d00fa0);
    qtest_writel(qts, ISP + 0x2034, 0x00000fa0);
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    return qts;
}

/* An AE block's histogram: its mean bin; *n its quads */
static uint32_t t31_ae_mean(QTestState *qts, uint32_t buf, uint32_t *n)
{
    uint32_t h[256];
    uint64_t sum = 0;

    qtest_memread(qts, buf, h, sizeof(h));
    *n = 0;
    for (int i = 0; i < 256; i++) {
        uint32_t v = le32_to_cpu(h[i]) & 0x1fffff;

        *n += v;
        sum += (uint64_t)i * v;
    }
    return *n ? sum / *n : 0;
}

static void test_t31_wdr(void)
{
    QTestState *qts = t31_wdr_start(true);
    uint32_t addr = qtest_readl(qts, ISP + 0x2680);
    uint32_t *w = g_new(uint32_t, 129 * 8), quads = W * H / 4;
    uint32_t n0, n1, m0, m1, num[2] = { 0 };
    uint64_t mean[2][3] = { { 0 } };

    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4) & (BIT(11) | BIT(28) |
                                                    BIT(29)), ==,
                    BIT(11) | BIT(28) | BIT(29));
    g_assert_cmphex(addr & ~0x6000, ==, T31_WDR);
    /* 8 rows of 129 words, 0x400 bytes apart */
    for (int r = 0; r < 8; r++) {
        qtest_memread(qts, addr + r * 0x400, w + r * 129, 129 * 4);
    }
    for (int i = 0; i < 129 * 8; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
    for (int e = 0; e < 2; e++) {
        for (int c = 0; c < 3; c++) {
            uint32_t n = 0;

            for (int i = 0; i < 256; i++) {
                uint32_t v = bits_at(w + i * 4 + e * 2, c * 21, 21);

                n += v;
                mean[e][c] += (uint64_t)i * v;
            }
            g_assert_cmpuint(n, ==, quads);
        }
        for (int c = 0; c < 3; c++) {
            num[e] += bits_at(w + 1024 + e * 2, c * 21, 21);
        }
    }
    /*
     * the counts leave out the quads with a clipped channel: more of the
     * long exposure's than the short one's
     */
    g_assert_cmpuint(num[0], <, num[1]);
    g_assert_cmpuint(num[1], <=, quads);
    /* the short exposure a quarter of the long one, where that is lit */
    g_assert_cmpuint(mean[1][1], <, mean[0][1] / 2);
    g_assert_cmpuint(w[1028], >, w[1030]);
    /*
     * the luma sum: (R + 2 G + B) / 4 of the channels, 12 bits, whose
     * histograms keep their top 8
     */
    for (int e = 0; e < 2; e++) {
        uint64_t y8 = (mean[e][0] + 2 * mean[e][1] + mean[e][2]) / 4;

        g_assert_cmpuint(w[1028 + e * 2], >=, y8 * 16);
        g_assert_cmpuint(w[1028 + e * 2], <=, y8 * 16 + quads * 16);
    }
    /* AE0 the long exposure, AE1 the short one */
    m0 = t31_ae_mean(qts, T31_AE + 0x1000, &n0);
    m1 = t31_ae_mean(qts, T31_AE1 + 0x1000, &n1);
    g_assert_cmpuint(n0, ==, quads);
    g_assert_cmpuint(n1, ==, quads);
    g_assert_cmpuint(m1, <, m0 / 2);
    g_free(w);
    qtest_quit(qts);

    /* linear: neither */
    qts = t31_wdr_start(false);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4) & (BIT(11) | BIT(28)), ==,
                    0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0x2680), ==, 0);
    qtest_quit(qts);
}

#define T41_WDR         0x01040000      /* 4 buffers of 6 KiB */

/*
 * The T41's WDR block with a GC4653 or GC5603 (@model, answering at @addr)
 * in DOL mode, long exposure 1000 lines, short 250, in a room three times
 * as bright as the long one takes, set up as tisp_wdr_init sets it, its RGB
 * histograms on (0x600c bit 4)
 */
static void t41_wdr(const char *model, uint8_t addr)
{
    g_autofree char *args = g_strdup_printf(
        "-machine ingenic-t41 -device image-sensor,model=%s,bus=i2c0,fps=1,"
        "light=300,motion=off", model);
    QTestState *qts = qtest_init(args);
    uint32_t *w = g_new(uint32_t, 0x1800 / 4), quads = T41_W * T41_H / 4;
    uint64_t lng = 0, shrt = 0;
    uint32_t i, n = 0;

    qtest_writel(qts, CPM_CLKGR0,
                 qtest_readl(qts, CPM_CLKGR0) & ~(1u << 23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~(1u << 27));
    qtest_writel(qts, I2C0 + I2C_ENB, 0);
    qtest_writel(qts, I2C0 + 0x04, addr);
    qtest_writel(qts, I2C0 + I2C_ENB, 1);
    gc_write(qts, 0x0107, 0x89);
    gc_write(qts, 0x0202, 0x03);
    gc_write(qts, 0x0203, 0xe8);
    gc_write(qts, 0x0200, 0x00);
    gc_write(qts, 0x0201, 0xfa);
    t41_wr(qts, 0x80, T41_W << 16 | T41_H);
    t41_wr(qts, 0x40020, ~0u);
    for (int b = 0; b < 4; b++) {
        t41_wr(qts, 0x6010 + b * 4, T41_WDR + b * 0x1800);
    }
    t41_wr(qts, 0x6030, 3);
    t41_wr(qts, 0x600c, BIT(4));
    t41_wr(qts, 0x80000, 1);                    /* VIC start */
    qtest_clock_step(qts, T41_SEC);

    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(11), ==, BIT(11));
    /* the hardware's index and address; 0x6030 stays as written */
    i = t41_rd(qts, 0x6038);
    g_assert_cmpuint(i, <, 4);
    g_assert_cmphex(t41_rd(qts, 0x603c), ==, T41_WDR + i * 0x1800);
    g_assert_cmpuint(t41_rd(qts, 0x6030), ==, 3);
    t41_read(qts, T41_WDR + i * 0x1800, w, 0x1800 / 4);
    /* each block row: 15 of the long exposure, then 15 of the short one */
    for (int r = 0; r < 15; r++) {
        for (int c = 0; c < 15; c++) {
            uint32_t l = bits_at(w, ((r * 2) * 15 + c) * 24, 24);
            uint32_t s = bits_at(w, ((r * 2 + 1) * 15 + c) * 24, 24);

            g_assert_cmpuint(s, <=, l);
            lng += l;
            shrt += s;
        }
    }
    /* the short exposure a quarter of the long one where neither clips */
    g_assert_cmpuint(shrt * 2, <, lng);
    g_assert_cmpuint(shrt * 8, >, lng);
    /* the short one's brightest, 12 bits */
    g_assert_cmpuint(w[0x152] & 0xfff, >, 0);
    g_assert_cmpuint(w[0x153] >> 16, <=, 0xfff);
    /*
     * the long one's pixels below 0x6048 bits 11:0 (160: the black bar) and
     * at or above bits 27:16 (3900: it saturates)
     */
    g_assert_cmpuint(w[0x154], >, 0);
    g_assert_cmpuint(w[0x154] + w[0x155], <, quads * 4);
    g_assert_cmpuint(w[0x155], >, 0);
    /* two counts a quad */
    for (int b = 0; b < 160; b++) {
        n += bits_at(w + 0x156, b * 24, 24);
    }
    g_assert_cmpuint(n, ==, quads * 2);
    /* the RGB histograms: every quad, each exposure */
    for (int e = 0; e < 2; e++) {
        for (int c = 0; c < 3; c++) {
            uint32_t sum = 0;

            for (int b = 0; b < 256; b++) {
                sum += bits_at(w + 0x1ce + e * 0x200 + b * 2, c * 21, 21);
            }
            g_assert_cmpuint(sum, ==, quads);
        }
    }
    /* a pixel counts once: every one is at or above 0, none below 0 */
    t41_wr(qts, 0x6048, 0);
    qtest_clock_step(qts, T41_SEC);
    i = t41_rd(qts, 0x6038);
    t41_read(qts, T41_WDR + i * 0x1800, w, 0x1800 / 4);
    g_assert_cmpuint(w[0x154], ==, 0);
    g_assert_cmpuint(w[0x155], ==, quads * 4);
    /* without stat_en the RGB histograms are left as they were */
    t41_wr(qts, 0x600c, 0);
    qtest_memset(qts, T41_WDR + (i + 1) % 4 * 0x1800, 0xa5, 0x1800);
    qtest_clock_step(qts, T41_SEC);
    g_assert_cmpuint(t41_rd(qts, 0x6038), ==, (i + 1) % 4);
    t41_read(qts, T41_WDR + (i + 1) % 4 * 0x1800, w, 0x1800 / 4);
    g_assert_cmphex(w[0x1ce], ==, 0xa5a5a5a5);
    g_assert_cmphex(w[0x5cd], ==, 0xa5a5a5a5);
    g_assert_cmpuint(w[0x155], ==, quads * 4);
    g_free(w);
    qtest_quit(qts);

    /* a sensor without WDR: nothing */
    qts = t41_start();
    g_assert_cmphex(t41_rd(qts, T41_INT) & BIT(11), ==, 0);
    qtest_quit(qts);
}

static void test_t41_wdr(void)
{
    t41_wdr("gc4653", 0x29);
}

/* the T41LQ's sensor */
static void test_t41_wdr_gc5603(void)
{
    t41_wdr("gc5603", 0x31);
}

/*
 * The T33's and T32 Pro's AE and AWB, 15x15 zones of 64x36 quads, as their
 * tx-isp modules read them (firmware only: no board of either is on the
 * bench): the T32's registers, which a guest's register lists load, here
 * stored, and records of whole fields. A buffer starts with a 16-bit tag in
 * 8 bytes and ends with the tag again, which a driver compares. AWB: a zone
 * is 28 bytes (the R, G and B sums of all its quads and of its white ones,
 * 32 bits each, then their numbers, 16 bits each), a row padded to 8 bytes;
 * the T33 steps four buffers by the index at 0x1608c, the T32 Pro's driver
 * moves the one address itself. AE: a histogram of 256 32-bit bins at the
 * buffer's start and, 0x410 on, a zone buffer of 16 bytes a zone, three
 * luma sums (32 bits) and the numbers below and above the levels (16).
 */
#define FLAT_AWB        0x01100000      /* 4 buffers of 0x7400 bytes */
#define FLAT_AE         0x01200000      /* 4 buffers of 0x1248 bytes */
#define FLAT_GRID       15
#define FLAT_AWB_ROW    424             /* 15 zones of 28 bytes, to 8 */
#define FLAT_AWB_LEN    (8 + FLAT_AWB_ROW * FLAT_GRID + 2)
#define FLAT_AE_LEN     0xe1a           /* 0x410 + this: the zones' tag ends */

typedef struct FlatPart {
    const char *variant;
    uint32_t trig, table;       /* the register lists' trigger and table */
    uint32_t awb_count;         /* the AWB buffers less one */
    const char *model;
    uint32_t kelvin;
    double rg, bg;              /* a grey's raw R/G and B/G (1/256ths) there */
} FlatPart;

/* An OS04D10's (T33) and a GC4653's (T32 Pro) tables in their tuning files */
static const FlatPart flat_t33 = {
    .variant = "t33n", .trig = 0x22010, .table = 0x22020, .awb_count = 3,
    .model = "os04d10", .kelvin = 6500, .rg = 119.8, .bg = 140.5,
};

static const FlatPart flat_t32pro = {
    .variant = "t32prozn", .trig = 0x1c010, .table = 0x1c020, .awb_count = 0,
    .model = "gc4653", .kelvin = 6500, .rg = 123.2, .bg = 144.5,
};

static const FlatPart flat_t33_3000 = {
    .variant = "t33n", .trig = 0x22010, .table = 0x22020, .awb_count = 3,
    .model = "os04d10", .kelvin = 3000, .rg = 192.8, .bg = 71.0,
};

static const FlatPart flat_t32pro_3000 = {
    .variant = "t32prozn", .trig = 0x1c010, .table = 0x1c020, .awb_count = 0,
    .model = "gc4653", .kelvin = 3000, .rg = 194.2, .bg = 88.5,
};

/* a JXQ03 has no table on either part: the constants */
static const FlatPart flat_t33_none = {
    .variant = "t33n", .trig = 0x22010, .table = 0x22020, .awb_count = 3,
    .model = "jxq03", .kelvin = 6500, .rg = 144, .bg = 105,
};

static const FlatPart flat_t32pro_none = {
    .variant = "t32prozn", .trig = 0x1c010, .table = 0x1c020, .awb_count = 0,
    .model = "jxq03", .kelvin = 6500, .rg = 144, .bg = 105,
};

static unsigned flat_grey;      /* a flat grey of this luma, 0: the ramp */

static QTestState *flat_start(const FlatPart *p)
{
    QTestState *qts = qtest_initf("-machine ingenic-t32 -global "
                                  "ingenic-t31.soc-variant=%s -device "
                                  "image-sensor,model=%s,bus=i2c0,id=cam,"
                                  "motion=off,fps=25,kelvin=%u,grey=%u",
                                  p->variant, p->model, p->kelvin, flat_grey);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    /* AWB: the zones, the white region as the GC2053's tuning file has it */
    qtest_writel(qts, ISP + 0x160f4, 36 << 16 | 64);
    qtest_writel(qts, ISP + 0x16064, 269 << 16 | 70);
    qtest_writel(qts, ISP + 0x16068, 232 << 16 | 47);
    qtest_writel(qts, ISP + 0x1606c, 197 << 16 | 256);
    qtest_writel(qts, ISP + 0x16070, 359 << 16 | 256);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0x16078 + i * 4, FLAT_AWB + i * 0x7400);
    }
    qtest_writel(qts, ISP + 0x16088, 0x01010100 | p->awb_count);
    qtest_writel(qts, ISP + 0x16010, 1);
    /* AE: the zones, the dark and bright levels (30 and 170) */
    qtest_writel(qts, ISP + 0x17044, 36 << 16 | 64);
    qtest_writel(qts, ISP + 0x17064, 0x0011aa1e);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP + 0x17068 + i * 4, FLAT_AE + i * 0x1248 + 0x410);
        qtest_writel(qts, ISP + 0x17078 + i * 4, FLAT_AE + i * 0x1248);
    }
    qtest_writel(qts, ISP + 0x17088, 0x10010033);
    qtest_writel(qts, ISP + 0x17010, 1);
    qtest_writel(qts, ISP + 0x80010, 1);                /* VIC start */
    qtest_clock_step(qts, FRAME_NS);
    return qts;
}

/* The buffer AWB wrote last, with its tags checked; the tag */
static uint16_t flat_awb(QTestState *qts, uint8_t *w)
{
    uint32_t idx = qtest_readl(qts, ISP + 0x1608c);
    uint16_t tag;

    g_assert_cmphex(qtest_readl(qts, ISP + 0x30028) & BIT(3), ==, BIT(3));
    g_assert_cmpuint(idx, <, 4);
    qtest_memread(qts, qtest_readl(qts, ISP + 0x16078 + idx * 4), w,
                  FLAT_AWB_LEN);
    tag = lduw_le_p(w);
    g_assert_cmpuint(lduw_le_p(w + FLAT_AWB_LEN - 2), ==, tag);
    return tag;
}

/* A zone's field @f: 0 to 2 the sums of all its quads, 3 to 5 the white */
static uint32_t flat_awb_sum(const uint8_t *w, unsigned row, unsigned col,
                             unsigned f)
{
    return ldl_le_p(w + 8 + row * FLAT_AWB_ROW + col * 28 + f * 4);
}

/* A zone's quads, all of them (0) or the white ones (1) */
static uint32_t flat_awb_n(const uint8_t *w, unsigned row, unsigned col,
                           unsigned white)
{
    return lduw_le_p(w + 8 + row * FLAT_AWB_ROW + col * 28 + 24 + white * 2);
}

static void test_flat_awb(const void *data)
{
    const FlatPart *p = data;
    QTestState *qts = flat_start(p);
    g_autofree uint8_t *w = g_malloc(FLAT_AWB_LEN);
    uint64_t sum[3] = { 0 }, white[3] = { 0 };
    uint16_t tag;

    tag = flat_awb(qts, w);
    /* a T32 Pro has the one buffer */
    g_assert_cmpuint(qtest_readl(qts, ISP + 0x1608c), <=, p->awb_count);
    /* every zone counts its 64x36 quads, and a grey ramp's are white */
    for (unsigned r = 0; r < FLAT_GRID; r++) {
        for (unsigned c = 0; c < FLAT_GRID; c++) {
            g_assert_cmpuint(flat_awb_n(w, r, c, 0), ==, 64 * 36);
        }
    }
    /* the zones of the ramp, the last row's 4th to 10th: the raw R/G, B/G */
    for (unsigned c = 3; c < 10; c++) {
        g_assert_cmpuint(flat_awb_n(w, 14, c, 1), >, 0);
        g_assert_cmpuint(flat_awb_n(w, 14, c, 1), <=, 64 * 36);
        for (unsigned f = 0; f < 3; f++) {
            sum[f] += flat_awb_sum(w, 14, c, f);
            white[f] += flat_awb_sum(w, 14, c, 3 + f);
        }
    }
    for (unsigned f = 0; f < 3; f++) {
        g_assert_cmpuint(white[f], <=, sum[f]);
    }
    g_assert_cmpfloat_with_epsilon((double)sum[0] / sum[1], p->rg / 256,
                                   p->rg / 256 * 0.06);
    g_assert_cmpfloat_with_epsilon((double)sum[2] / sum[1], p->bg / 256,
                                   p->bg / 256 * 0.06);
    g_assert_cmpfloat_with_epsilon((double)white[0] / white[1], p->rg / 256,
                                   p->rg / 256 * 0.06);
    g_assert_cmpfloat_with_epsilon((double)white[2] / white[1], p->bg / 256,
                                   p->bg / 256 * 0.06);
    /* each frame has a tag of its own */
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(flat_awb(qts, w), !=, tag);
    qtest_quit(qts);
}

/* The zones' luma sums add up to the histogram's, the tags agree */
static void test_flat_ae(const void *data)
{
    const FlatPart *p = data;
    QTestState *qts = flat_start(p);
    uint8_t hist[0x40a], zone[FLAT_AE_LEN];
    uint64_t bins = 0, mean = 0, zones = 0;
    uint32_t idx = qtest_readl(qts, ISP + 0x1708c);

    g_assert_cmphex(qtest_readl(qts, ISP + 0x30028) & (BIT(4) | BIT(5)), ==,
                    BIT(4) | BIT(5));
    /* both buffers of the same index, written by one frame */
    g_assert_cmpuint(extract32(idx, 4, 2), ==, extract32(idx, 0, 2));
    idx &= 3;
    qtest_memread(qts, FLAT_AE + idx * 0x1248, hist, sizeof(hist));
    qtest_memread(qts, FLAT_AE + idx * 0x1248 + 0x410, zone, sizeof(zone));
    g_assert_cmpuint(lduw_le_p(hist), ==, lduw_le_p(hist + 0x408));
    g_assert_cmpuint(lduw_le_p(zone), ==, lduw_le_p(zone + 0xe18));
    g_assert_cmpuint(lduw_le_p(zone), ==, lduw_le_p(hist));
    for (unsigned i = 0; i < 256; i++) {
        uint32_t n = ldl_le_p(hist + 8 + i * 4);

        bins += n;
        mean += (uint64_t)n * i;
    }
    /* every quad is in a bin once, and in a zone once */
    g_assert_cmpuint(bins, ==, 960 * 540);
    for (unsigned z = 0; z < FLAT_GRID * FLAT_GRID; z++) {
        const uint8_t *v = zone + 8 + z * 16;

        zones += (uint64_t)ldl_le_p(v) + ldl_le_p(v + 4) + ldl_le_p(v + 8);
    }
    g_assert_cmpuint(zones, ==, mean);
    /* the ramp spans the levels: some quads are dark, some bright */
    for (unsigned f = 12; f < 16; f += 2) {
        uint32_t n = 0;

        for (unsigned z = 0; z < FLAT_GRID * FLAT_GRID; z++) {
            n += lduw_le_p(zone + 8 + z * 16 + f);
        }
        g_assert_cmpuint(n, >, 0);
    }
    qtest_quit(qts);
}

/*
 * The index registers are the hardware's: a register list that carries them
 * (AE's here, 0x1708c) leaves them be
 */
static void test_flat_index(const void *data)
{
    const FlatPart *p = data;
    QTestState *qts = flat_start(p);
    const uint32_t list = 0x00500100;
    uint32_t idx = qtest_readl(qts, ISP + 0x1708c);

    g_assert_cmpuint(idx, !=, 0);
    qtest_writel(qts, ISP + p->table, 0x00500000);
    qtest_writel(qts, 0x00500000 + 4 * 7, list);
    qtest_writel(qts, list, 2);
    qtest_writel(qts, list + 4, 1u << 31 | 0x1708c / 4);
    qtest_writel(qts, list + 8, 0);
    qtest_writel(qts, ISP + p->trig, 1 << 7);
    g_assert_cmpuint(qtest_readl(qts, ISP + 0x1708c), ==, idx);
    qtest_quit(qts);
}

/*
 * A flat grey of any level has the raw colour of its light: the sums of a
 * dim one, a few levels of the 8-bit sample, hold the sensor's ratios as a
 * bright one does. Each part's zones of the last row, R/G and B/G against
 * its table (rg == 0: against the same part's grey of luma 200).
 */
typedef struct GreyPart {
    const char *name;
    const char *machine;
    const char *model;
    unsigned kelvin;
    double rg, bg;
    const FlatPart *flat;
} GreyPart;

static void grey_ratios(const GreyPart *p, unsigned luma, double *rg,
                        double *bg)
{
    g_autofree char *props = g_strdup_printf("kelvin=%u,grey=%u", p->kelvin,
                                             luma);
    uint64_t sum[4] = { 0 };

    if (p->flat) {
        QTestState *qts;
        g_autofree uint8_t *w = g_malloc(FLAT_AWB_LEN);

        flat_grey = luma;
        qts = flat_start(p->flat);
        flat_grey = 0;
        flat_awb(qts, w);
        for (unsigned c = 3; c < 10; c++) {
            for (unsigned f = 0; f < 3; f++) {
                sum[f] += flat_awb_sum(w, 14, c, f);
            }
        }
        qtest_quit(qts);
    } else if (!strcmp(p->name, "t32")) {
        t32_awb_sums(p->model, props, sum);
    } else if (!strcmp(p->name, "t40") || !strcmp(p->name, "t41")) {
        t41_machine = p->machine;
        t41_awb_sums(p->model, props, sum);
        t41_machine = "ingenic-t41";
    } else {
        tiz_awb_sums(p->machine, p->model, props, sum);
    }
    g_assert_cmpuint(sum[1], >, 0);
    *rg = (double)sum[0] / sum[1];
    *bg = (double)sum[2] / sum[1];
}

static void test_awb_grey(const void *data)
{
    const GreyPart *p = data;
    static const unsigned lumas[] = { 200, 110, 80 };
    double rg0, bg0, rg, bg;

    grey_ratios(p, 200, &rg0, &bg0);
    if (p->rg) {
        g_assert_cmpfloat_with_epsilon(rg0, p->rg, p->rg * 0.015);
        g_assert_cmpfloat_with_epsilon(bg0, p->bg, p->bg * 0.015);
    }
    for (int i = 1; i < ARRAY_SIZE(lumas); i++) {
        grey_ratios(p, lumas[i], &rg, &bg);
        g_assert_cmpfloat_with_epsilon(rg, rg0, rg0 * 0.01);
        g_assert_cmpfloat_with_epsilon(bg, bg0, bg0 * 0.01);
    }
}

static const GreyPart grey_t23 = {
    "t23", "ingenic-t23", "gc1084", 3000, 821 / 1024.0, 393 / 1024.0,
};
static const GreyPart grey_t31 = {
    "t31", "ingenic-t31", "imx307", 3000, 903 / 1024.0, 347 / 1024.0,
};
static const GreyPart grey_t32 = {
    "t32", "ingenic-t32", "gc4023", 3000, 777 / 1024.0, 555 / 1024.0,
};
static const GreyPart grey_t32_cv8001 = {
    "t32", "ingenic-t32", "cv8001", 5000, 0, 0,
};
static const GreyPart grey_t40 = {
    "t40", "ingenic-t40", "gc4653", 3000, 0, 0,
};
static const GreyPart grey_t41 = {
    "t41", "ingenic-t41", "gc4023", 3000, 873 / 1024.0, 453 / 1024.0,
};
static const GreyPart grey_t33 = {
    "t33", NULL, NULL, 3000, 192.8 / 256, 71.0 / 256, &flat_t33_3000,
};
static const GreyPart grey_t32pro = {
    "t32pro", NULL, NULL, 3000, 194.2 / 256, 88.5 / 256, &flat_t32pro_3000,
};


int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t23", &grey_t23,
                        test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t31", &grey_t31,
                        test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t32", &grey_t32,
                        test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t32-cv8001",
                        &grey_t32_cv8001, test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t40", &grey_t40,
                        test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t41", &grey_t41,
                        test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t33", &grey_t33,
                        test_awb_grey);
    qtest_add_data_func("/ingenic-isp-stats/awb-grey/t32pro", &grey_t32pro,
                        test_awb_grey);
    qtest_add_func("/ingenic-isp-stats/t20/ae", test_t20_ae);
    qtest_add_func("/ingenic-isp-stats/t20/awb", test_t20_awb);
    qtest_add_func("/ingenic-isp-stats/t20/ae-zones", test_t20_ae_zones);
    qtest_add_func("/ingenic-isp-stats/t20/awb-weights", test_t20_awb_weights);
    for (int i = 0; i < ARRAY_SIZE(t20_responses); i++) {
        const T20Response *t = &t20_responses[i];
        g_autofree char *path = g_strdup_printf(
            "/ingenic-isp-stats/t20/awb-response/%s-%s-%u", t->machine + 8,
            t->model, t->kelvin);

        qtest_add_data_func(path, t, test_t20_awb_response);
    }
    qtest_add_func("/ingenic-isp-stats/t20/af", test_t20_af);
    qtest_add_func("/ingenic-isp-stats/t20/mve", test_t20_mve);
    qtest_add_func("/ingenic-isp-stats/t21", test_t21);
    qtest_add_func("/ingenic-isp-stats/t21/free", test_t21_free);
    qtest_add_func("/ingenic-isp-stats/t21/awb-grey", test_t21_awb_grey);
    qtest_add_func("/ingenic-isp-stats/t31/adr", test_t31_adr);
    qtest_add_func("/ingenic-isp-stats/t31/defog", test_t31_defog);
    qtest_add_func("/ingenic-isp-stats/t31/awb", test_t31_awb);
    qtest_add_func("/ingenic-isp-stats/t23/awb", test_t23_awb);
    qtest_add_func("/ingenic-isp-stats/t31/af", test_t31_af);
    qtest_add_func("/ingenic-isp-stats/t31/gains", test_t31_gains);
    qtest_add_func("/ingenic-isp-stats/t31/wdr", test_t31_wdr);
    qtest_add_func("/ingenic-isp-stats/t41/af", test_t41_af);
    qtest_add_func("/ingenic-isp-stats/t32/awb", test_t32_awb);
    qtest_add_func("/ingenic-isp-stats/t41/awb", test_t41_awb);
    qtest_add_func("/ingenic-isp-stats/t41/gains", test_t41_gains);
    qtest_add_func("/ingenic-isp-stats/t41/adr", test_t41_adr);
    qtest_add_func("/ingenic-isp-stats/t41/defog", test_t41_defog);
    qtest_add_func("/ingenic-isp-stats/t41/tmo", test_t41_tmo);
    qtest_add_func("/ingenic-isp-stats/t41/lce", test_t41_lce);
    qtest_add_func("/ingenic-isp-stats/t41/gsm", test_t41_gsm);
    qtest_add_func("/ingenic-isp-stats/t41/wdr", test_t41_wdr);
    qtest_add_func("/ingenic-isp-stats/t41/wdr-gc5603", test_t41_wdr_gc5603);
    qtest_add_data_func("/ingenic-isp-stats/t33/awb", &flat_t33,
                        test_flat_awb);
    qtest_add_data_func("/ingenic-isp-stats/t33/awb-3000", &flat_t33_3000,
                        test_flat_awb);
    qtest_add_data_func("/ingenic-isp-stats/t33/awb-no-table", &flat_t33_none,
                        test_flat_awb);
    qtest_add_data_func("/ingenic-isp-stats/t33/ae", &flat_t33, test_flat_ae);
    qtest_add_data_func("/ingenic-isp-stats/t33/index", &flat_t33,
                        test_flat_index);
    qtest_add_data_func("/ingenic-isp-stats/t32pro/awb", &flat_t32pro,
                        test_flat_awb);
    qtest_add_data_func("/ingenic-isp-stats/t32pro/awb-3000",
                        &flat_t32pro_3000, test_flat_awb);
    qtest_add_data_func("/ingenic-isp-stats/t32pro/awb-no-table",
                        &flat_t32pro_none, test_flat_awb);
    qtest_add_data_func("/ingenic-isp-stats/t32pro/ae", &flat_t32pro,
                        test_flat_ae);
    qtest_add_data_func("/ingenic-isp-stats/t32pro/index", &flat_t32pro,
                        test_flat_index);
    return g_test_run();
}
