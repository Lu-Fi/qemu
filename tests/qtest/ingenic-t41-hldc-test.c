/*
 * QTest testcase for the T41 ISP's lens correction (HLDC)
 *
 * What a T41NQ showed: its reset values, the bits each register keeps,
 * settings that take effect at a frame start once 0x44 asks, the remap of
 * each row (luma and chroma), and settings that stop the ISP.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/timer.h"
#include "libqtest.h"

#define CPM_CLKGR       0x10000020
#define CPM_CIMCDR      0x10000090
#define ISP             0x13300000
#define ISP_BYPASS      (ISP + 0x40)
#define ISP_SIZE        (ISP + 0x80)
#define ISP_LATCH       (ISP + 0x1000)
#define HLDC            (ISP + 0x15000)
#define VIC_CTRL        (ISP + 0x80000)
#define CH_EN           (ISP + 0xf0008)
#define MSCA_LATCH      (ISP + 0xf0010)
#define CH0             (ISP + 0xf0100)

#define BYPASS_HLDC     (1u << 20)

#define W               640
#define H               360
#define BUF_Y(n)        (0x01000000 + (n) * 0x800000)
#define BUF_UV(n)       (BUF_Y(n) + 0x400000)

typedef struct {
    uint32_t reg, ones;
} Mask;

static const Mask masks[] = {
    { 0x04, 0x03ff03ff }, { 0x08, 0x03ff03ff }, { 0x0c, 0x00000fff },
    { 0x10, 0x00000fff }, { 0x14, 0x00001fff }, { 0x18, 0x00007fff },
    { 0x1c, 0x00ff00ff }, { 0x20, 0x007f007f }, { 0x24, 0x0000ffff },
    { 0x28, 0x0000ffff },
};

/* tisp_hldc_set_attr's for strength 255, centre (300, 200) */
static const uint32_t s255[] = {
    0x006201f0, 0x006201f0, 0x00000a9f, 0x00000a9f, 0x00001b24,
    0x00006d55, 0x0014ffec, 0x000afff6, 0, 0,
};

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, HLDC + reg);
}

static void wr(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, HLDC + reg, v);
}

static void set(QTestState *qts, const uint32_t *v)
{
    for (int i = 0; i < 10; i++) {
        wr(qts, 0x04 + i * 4, v[i]);
    }
    wr(qts, 0x44, 3);
}

/* The bypasses, latched at once as tisp_top_update_pos has them */
static void bypass(QTestState *qts, uint32_t v)
{
    qtest_writel(qts, ISP_BYPASS, v);
    qtest_writel(qts, ISP_LATCH, 1);
    qtest_writel(qts, ISP_LATCH, 0);
}

/* What a T41NQ's block reads out of reset, 0x00 to 0x44 */
static const uint32_t reset[] = {
    0x00010000, 0, 0, 0x800, 0x800, 0, 0, 0, 0, 0x14, 0xa,
    0x00080000, 0x00010001, 0, 0x00080000, 0x00010001, 0, 0,
};

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    for (int i = 0; i < ARRAY_SIZE(reset); i++) {
        g_assert_cmphex(rd(qts, i * 4), ==, reset[i]);
    }
    /* 0x34 and 0x40 show no lines before a frame has gone through */
    qtest_writel(qts, ISP_SIZE, 2560 << 16 | 1440);
    g_assert_cmphex(rd(qts, 0x34), ==, 0);
    g_assert_cmphex(rd(qts, 0x40), ==, 0);

    wr(qts, 0x00, 0xffffffff);
    g_assert_cmphex(rd(qts, 0x00), ==, 0x00010001);
    wr(qts, 0x00, 0);
    g_assert_cmphex(rd(qts, 0x00), ==, 0);

    /* Settings read back those in use: reset's until a frame starts */
    for (int i = 0; i < ARRAY_SIZE(masks); i++) {
        wr(qts, masks[i].reg, 0xffffffff);
        g_assert_cmphex(rd(qts, masks[i].reg), ==, reset[1 + i]);
    }
    wr(qts, 0x44, 0xffffffff);
    g_assert_cmphex(rd(qts, 0x44), ==, 1);

    for (uint32_t r = 0x2c; r <= 0x40; r += 4) {
        uint32_t v = rd(qts, r);

        wr(qts, r, ~v);
        g_assert_cmphex(rd(qts, r), ==, v);
    }
    qtest_quit(qts);
}

/* The model's remap, written from the T41NQ's measurements */
static int64_t ref_pos(const uint32_t *a, int c, int w, int h, int x, int y)
{
    int bits = c ? 7 : 8;
    int64_t k1 = a[c] & 0x3ff, k2 = a[c] >> 16 & 0x3ff;
    int64_t zoom = a[2 + c], norm = a[4 + c];
    int cx = w / 2 + sextract32(a[6 + c], 0, bits);
    int cy = h / 2 + sextract32(a[6 + c], 16, bits);
    int64_t d = x >= cx ? x - cx : cx - 1 - x;
    int64_t e = y >= cy ? y - cy : cy - 1 - y;
    uint64_t r = (uint64_t)(d * d + e * e) * norm & ((1ull << 30) - 1);
    uint64_t rq = r >> 10;
    int64_t f = (INT64_C(1) << 40) - k1 * (int64_t)r +
                k2 * (int64_t)((rq * rq) >> 10);
    int64_t off = d * ((f * zoom) >> 35);

    return x >= cx ? ((int64_t)cx << 16) + off
                   : ((int64_t)(cx - 1) << 16) - off;
}

static void ref_remap(const uint32_t *a, int c, uint8_t *buf, int w, int h,
                      int bpp, int stride)
{
    g_autofree uint8_t *row = g_malloc(w * bpp);

    for (int y = 0; y < h; y++) {
        uint8_t *p = buf + y * stride;

        memcpy(row, p, w * bpp);
        for (int x = 0; x < w; x++) {
            int64_t s = ref_pos(a, c, w, h, x, y);
            int i, j, f;

            s = MIN(MAX(s, 0), (int64_t)(w - 1) << 16);
            i = s >> 16;
            j = MIN(i + 1, w - 1);
            f = (s >> 8) & 0xff;
            for (int b = 0; b < bpp; b++) {
                p[x * bpp + b] = (row[i * bpp + b] * (256 - f) +
                                  row[j * bpp + b] * f) >> 8;
            }
        }
    }
}

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-device image-sensor,model=gc4023,"
                                 "bus=i2c0,fps=1");

    qtest_writel(qts, CPM_CLKGR, qtest_readl(qts, CPM_CLKGR) & ~(1u << 23));
    qtest_writel(qts, CPM_CIMCDR,
                 qtest_readl(qts, CPM_CIMCDR) & ~(1u << 27));
    qtest_writel(qts, ISP_SIZE, W << 16 | H);
    bypass(qts, BYPASS_HLDC);
    qtest_writel(qts, CH0 + 0x00, W << 16 | H);
    qtest_writel(qts, CH0 + 0x2c, W << 16 | H);
    qtest_writel(qts, CH0 + 0x80, W);
    qtest_writel(qts, CH0 + 0x98, W);
    qtest_writel(qts, MSCA_LATCH, 1);
    qtest_writel(qts, CH_EN, 1);
    return qts;
}

/* Queue buffer n, run a frame (the first at the VIC's start), drain it */
static bool frame(QTestState *qts, int n, bool first)
{
    qtest_writel(qts, CH0 + 0x6c, BUF_Y(n));
    qtest_writel(qts, CH0 + 0x84, BUF_UV(n));
    if (first) {
        qtest_writel(qts, VIC_CTRL, 1);
    } else {
        qtest_clock_step(qts, NANOSECONDS_PER_SECOND);
    }
    if (qtest_readl(qts, CH0 + 0x7c) & 1) {
        return false;
    }
    g_assert_cmphex(qtest_readl(qts, CH0 + 0x74), ==, BUF_Y(n));
    return true;
}

/*
 * A frame through the correction against the same scene bypassed (with
 * the sensor at 1 fps every other frame's scene is the same)
 */
static void test_remap(void)
{
    QTestState *qts = start();
    g_autofree uint8_t *want = g_malloc(W * H * 3 / 2);
    g_autofree uint8_t *got = g_malloc(W * H * 3 / 2);

    set(qts, s255);
    g_assert_true(frame(qts, 0, true));
    /* the settings took effect at that frame's start, bypassed or not */
    for (int i = 0; i < 10; i++) {
        g_assert_cmphex(rd(qts, 0x04 + i * 4), ==,
                        s255[i] & masks[i].ones);
    }
    g_assert_cmphex(rd(qts, 0x44), ==, 0);
    /* an idle block's state, 0x34 and 0x40 the lines of its frame */
    g_assert_cmphex(rd(qts, 0x30), ==, 0x00010008);
    g_assert_cmphex(rd(qts, 0x34), ==, H << 16);
    g_assert_cmphex(rd(qts, 0x40), ==, H / 2 << 16);
    g_assert_true(frame(qts, 1, false));
    bypass(qts, 0);
    g_assert_true(frame(qts, 2, false));

    qtest_memread(qts, BUF_Y(0), want, W * H);
    qtest_memread(qts, BUF_UV(0), want + W * H, W * H / 2);
    qtest_memread(qts, BUF_Y(2), got, W * H);
    qtest_memread(qts, BUF_UV(2), got + W * H, W * H / 2);
    g_assert_true(memcmp(want, got, W * H * 3 / 2));
    ref_remap(s255, 0, want, W, H, 1, W);
    ref_remap(s255, 1, want + W * H, W / 2, H / 2, 2, W);
    g_assert_cmpmem(got, W * H * 3 / 2, want, W * H * 3 / 2);

    /* 0x44 not asked: a new setting waits */
    wr(qts, 0x0c, 0x800);
    g_assert_true(frame(qts, 3, false));
    g_assert_cmphex(rd(qts, 0x0c), ==, 0xa9f);
    qtest_quit(qts);
}

/*
 * Settings that take a pixel from outside its row, or from more than 1/8
 * pixel behind, stop the ISP until it is reset, bypassed or not: as a
 * T41NQ ran the zooms 2048 and 2049, and k1 414 and 415 (norm 498, frame
 * 2560 x 1440, chroma at zoom 2048)
 */
static void test_stall(void)
{
    static const struct {
        uint32_t k1, zoom, norm;
        bool runs;
    } cases[] = {
        { 0, 0x800, 498, true }, { 0, 0x801, 498, false },
        { 414, 0x800, 498, true }, { 415, 0x800, 498, false },
    };

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        QTestState *qts = start();
        uint32_t a[10] = {
            cases[i].k1, 0, cases[i].zoom, 0x800, cases[i].norm, 1992,
        };

        qtest_writel(qts, ISP_SIZE, 2560 << 16 | 1440);
        qtest_writel(qts, CH0 + 0x00, 2560 << 16 | 1440);
        qtest_writel(qts, CH0 + 0x2c, 2560 << 16 | 1440);
        qtest_writel(qts, CH0 + 0x80, 2560);
        qtest_writel(qts, CH0 + 0x98, 2560);
        qtest_writel(qts, MSCA_LATCH, 1);
        set(qts, a);
        bypass(qts, 0);
        g_assert_cmpint(frame(qts, 0, true), ==, cases[i].runs);
        bypass(qts, BYPASS_HLDC);
        g_assert_cmpint(frame(qts, 1, false), ==, cases[i].runs);
        qtest_quit(qts);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-t41-hldc/regs", test_regs);
    qtest_add_func("/ingenic-t41-hldc/remap", test_remap);
    qtest_add_func("/ingenic-t41-hldc/stall", test_stall);
    return g_test_run();
}
