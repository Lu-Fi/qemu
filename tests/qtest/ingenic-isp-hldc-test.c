/*
 * QTest testcase for the lens correction (HLDC) of the T21's, T23's, T31's
 * and T40's ISP
 *
 * The block is the T41's (ingenic-t41-hldc-test): the same ten settings at
 * 0x04 to 0x28, taken at a frame start once asked for. Where each part has
 * it, what bypasses it and what asks for the settings come from the vendor
 * tx-isp drivers: the T23's and T31's at ISP 0x9000 (0x9044 asks, bit 18 of
 * 0x0c bypasses), the T21's at 0x2200 (no 0x44: bits 0 and 4 of 0x00, bit
 * 13 of 0x0c), the T40's at 0x15000 and, for its second pipeline, 0x15080
 * (0x44 asks, bit 20 of 0x40 and of 0x6c bypass). The reset values and the
 * bits each register keeps are the T41NQ's, not yet seen on these parts.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/timer.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR       0x10000020
#define CPM_CIMCDR      0x10000090

#define W               640
#define H               360
#define BUF_Y(n)        (0x01000000 + (n) * 0x800000)
#define BUF_UV(n)       (BUF_Y(n) + 0x400000)

typedef struct Part {
    const char *name, *machine;
    uint32_t hldc;          /* the block, in the ISP */
    uint32_t sec;           /* the second pipeline's, if any */
    uint32_t bypass, sec_bypass;    /* register */
    uint32_t bypass_bit;
    uint32_t size;          /* the ISP's frame size */
    bool ctl;               /* the T21: 0x00 asks */
    bool second;            /* frames of the second pipeline, a sensor on it */
    uint32_t vic, ch_en, ch_en_val, ch;
    /* the channel's registers, +: size, buffer and stride of each plane */
    uint32_t csize, y, y_stride, uv, uv_stride, done, done_y;
} Part;

static const Part parts[] = {
    { "t31", "ingenic-t31", 0x9000, 0, 0x0c, 0, BIT(18), 0x04, false, false,
      0xe0000, 0x9804, 2, 0x9a00,
      0x00, 0x6c, 0x80, 0x84, 0x98, 0x7c, 0x74 },
    { "t23", "ingenic-t23", 0x9000, 0, 0x0c, 0, BIT(18), 0x04, false, false,
      0xe0000, 0xd040, 2, 0xd200,
      0x04, 0x30, 0x40, 0x50, 0x60, 0x3c, 0x38 },
    { "t21", "ingenic-t21", 0x2200, 0, 0x0c, 0, BIT(13), 0x04, true, false,
      0x10000, 0x2304, 2, 0x2500,
      0x00, 0x6c, 0x80, 0x84, 0x98, 0x7c, 0x74 },
    { "t40", "ingenic-t40", 0x15000, 0x15080, 0x40, 0x6c, BIT(20), 0x80,
      false, false, 0x80000, 0x16008, 1, 0x16100,
      0x00, 0x6c, 0x80, 0x84, 0x98, 0x7c, 0x74 },
    { "t40-sec", "ingenic-t40", 0x15080, 0, 0x6c, 0, BIT(20), 0x80,
      false, true, 0x90000, 0x17008, 1, 0x17100,
      0x00, 0x6c, 0x80, 0x84, 0x98, 0x7c, 0x74 },
};

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

static uint32_t rd(QTestState *qts, const Part *p, uint32_t base, uint32_t reg)
{
    (void)p;
    return qtest_readl(qts, ISP + base + reg);
}

static void wr(QTestState *qts, uint32_t base, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, ISP + base + reg, v);
}

/* Ask for the settings the way each part's driver does */
static void ask(QTestState *qts, const Part *p, uint32_t base)
{
    if (p->ctl) {
        wr(qts, base, 0x00, 0x00010011);
    } else {
        wr(qts, base, 0x44, 3);
    }
}

static void set(QTestState *qts, const Part *p, uint32_t base,
                const uint32_t *v)
{
    for (int i = 0; i < 10; i++) {
        wr(qts, base, 0x04 + i * 4, v[i]);
    }
    ask(qts, p, base);
}

/* A T41NQ's block out of reset, 0x00 to 0x28 */
static const uint32_t reset[] = {
    0x00010000, 0, 0, 0x800, 0x800, 0, 0, 0, 0, 0x14, 0xa,
};

static void test_regs(const void *data)
{
    const Part *p = data;
    uint32_t bases[2] = { p->hldc, p->sec };
    QTestState *qts = qtest_initf("-machine %s", p->machine);

    qtest_writel(qts, CPM_CLKGR, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    for (int b = 0; b < 2 && bases[b]; b++) {
        uint32_t base = bases[b];
        uint32_t ctrl = p->ctl ? 0x00ffff13 : 0x00010001;

        for (int i = 0; i < ARRAY_SIZE(reset); i++) {
            g_assert_cmphex(rd(qts, p, base, i * 4), ==, reset[i]);
        }
        wr(qts, base, 0x00, 0xffffffff);
        g_assert_cmphex(rd(qts, p, base, 0x00), ==, ctrl);
        wr(qts, base, 0x00, 0);
        g_assert_cmphex(rd(qts, p, base, 0x00), ==, 0);

        /* Settings read back those in use: reset's until a frame starts */
        for (int i = 0; i < ARRAY_SIZE(masks); i++) {
            wr(qts, base, masks[i].reg, 0xffffffff);
            g_assert_cmphex(rd(qts, p, base, masks[i].reg), ==, reset[1 + i]);
        }
        if (!p->ctl) {
            wr(qts, base, 0x44, 0xffffffff);
            g_assert_cmphex(rd(qts, p, base, 0x44), ==, 1);
        }
    }
    /* the two instances are apart */
    if (p->sec) {
        wr(qts, p->hldc, 0x0c, 0x123);
        g_assert_cmphex(rd(qts, p, p->sec, 0x0c), ==, 0x800);
    }
    if (p->ctl) {
        /* the T21's block ends at 0x2228 */
        g_assert_cmphex(rd(qts, p, p->hldc, 0x30), ==, 0);
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

static QTestState *start(const Part *p, uint32_t w, uint32_t h)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=gc4653,bus=i2c0,fps=1,motion=off%s",
                                  p->machine, p->second ?
                                  " -device image-sensor,model=gc4653,"
                                  "bus=i2c1,input=1,fps=1,motion=off" : "");

    qtest_writel(qts, CPM_CLKGR, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + p->size, w << 16 | h);
    qtest_writel(qts, ISP + p->ch + p->csize, w << 16 | h);
    qtest_writel(qts, ISP + p->ch + p->y_stride, w);
    qtest_writel(qts, ISP + p->ch + p->uv_stride, w);
    qtest_writel(qts, ISP + p->ch_en, p->ch_en_val);
    qtest_writel(qts, ISP + p->bypass, p->bypass_bit);
    return qts;
}

/* Queue buffer n, run a frame (the first a frame on from the VIC's start) */
static bool frame(QTestState *qts, const Part *p, int n, bool first)
{
    qtest_writel(qts, ISP + p->ch + p->y, BUF_Y(n));
    qtest_writel(qts, ISP + p->ch + p->uv, BUF_UV(n));
    if (first) {
        qtest_writel(qts, ISP + p->vic, 1);
    }
    qtest_clock_step(qts, NANOSECONDS_PER_SECOND);
    if (qtest_readl(qts, ISP + p->ch + p->done) & 1) {
        return false;
    }
    qtest_readl(qts, ISP + p->ch + p->done_y);
    return true;
}

/*
 * A frame through the correction against the same scene bypassed (with
 * the sensor at 1 fps every other frame's scene is the same)
 */
static void test_remap(const void *data)
{
    const Part *p = data;
    QTestState *qts = start(p, W, H);
    g_autofree uint8_t *want = g_malloc(W * H * 3 / 2);
    g_autofree uint8_t *got = g_malloc(W * H * 3 / 2);

    set(qts, p, p->hldc, s255);
    g_assert_true(frame(qts, p, 0, true));
    /* the settings took effect at that frame's start, bypassed or not */
    for (int i = 0; i < 10; i++) {
        g_assert_cmphex(rd(qts, p, p->hldc, 0x04 + i * 4), ==,
                        s255[i] & masks[i].ones);
    }
    if (!p->ctl) {
        g_assert_cmphex(rd(qts, p, p->hldc, 0x44), ==, 0);
    }
    g_assert_true(frame(qts, p, 1, false));
    qtest_writel(qts, ISP + p->bypass, 0);
    g_assert_true(frame(qts, p, 2, false));

    qtest_memread(qts, BUF_Y(0), want, W * H);
    qtest_memread(qts, BUF_UV(0), want + W * H, W * H / 2);
    qtest_memread(qts, BUF_Y(2), got, W * H);
    qtest_memread(qts, BUF_UV(2), got + W * H, W * H / 2);
    g_assert_true(memcmp(want, got, W * H * 3 / 2));
    ref_remap(s255, 0, want, W, H, 1, W);
    ref_remap(s255, 1, want + W * H, W / 2, H / 2, 2, W);
    g_assert_cmpmem(got, W * H * 3 / 2, want, W * H * 3 / 2);

    /* not asked: a new setting waits (the T21's asks while 0x00 says so) */
    if (!p->ctl) {
        wr(qts, p->hldc, 0x0c, 0x800);
        g_assert_true(frame(qts, p, 3, false));
        g_assert_cmphex(rd(qts, p, p->hldc, 0x0c), ==, 0xa9f);
    } else {
        wr(qts, p->hldc, 0x00, 0x00010000);
        wr(qts, p->hldc, 0x0c, 0x800);
        g_assert_true(frame(qts, p, 3, false));
        g_assert_cmphex(rd(qts, p, p->hldc, 0x0c), ==, 0xa9f);
        wr(qts, p->hldc, 0x00, 0x00010001);
        g_assert_true(frame(qts, p, 4, false));
        g_assert_cmphex(rd(qts, p, p->hldc, 0x0c), ==, 0x800);
    }
    qtest_quit(qts);
}

/*
 * Settings that take a pixel from outside its row stop the ISP until it is
 * reset, bypassed or not: the T41NQ's zoom 2049
 */
static void test_stall(const void *data)
{
    const Part *p = data;
    static const struct {
        uint32_t zoom;
        bool runs;
    } cases[] = { { 0x800, true }, { 0x801, false } };

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        QTestState *qts = start(p, W, H);
        uint32_t a[10] = { 0, 0, cases[i].zoom, 0x800, 498, 1992, };

        set(qts, p, p->hldc, a);
        qtest_writel(qts, ISP + p->bypass, 0);
        g_assert_cmpint(frame(qts, p, 0, true), ==, cases[i].runs);
        qtest_writel(qts, ISP + p->bypass, p->bypass_bit);
        g_assert_cmpint(frame(qts, p, 1, false), ==, cases[i].runs);
        qtest_quit(qts);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-isp-hldc/%s/regs",
                                                parts[i].name);

        if (!parts[i].second) {
            qtest_add_data_func(path, &parts[i], test_regs);
        }
        g_free(path);
        path = g_strdup_printf("/ingenic-isp-hldc/%s/remap", parts[i].name);
        qtest_add_data_func(path, &parts[i], test_remap);
        g_free(path);
        path = g_strdup_printf("/ingenic-isp-hldc/%s/stall", parts[i].name);
        qtest_add_data_func(path, &parts[i], test_stall);
    }
    return g_test_run();
}
