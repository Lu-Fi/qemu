/*
 * QTest testcase for the image sensor's noise mode ("sensor-noise"): the
 * variance of a Bayer sample against its level follows the law measured on a
 * T41NQ's GC4023 (a read floor plus a shot noise growing with the level, both
 * with the analog gain), a still scene's difference between frames is twice
 * that, a run repeats and a frame differs from the next, and the YUV picture
 * of a T20 carries the same noise as the raw samples of a T41. Off, the
 * default, a still scene stays still.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qemu/bswap.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define VIC(n)          (ISP + 0x80000 + (n) * 0x10000)
#define T41_WDMA        (ISP + 0x20000)
#define T41_DMA_EN      (ISP + 0x400b0)
#define I2C0            0x10050000
#define BUF             0x00400000
#define W               64
#define H               48
#define FRAME_NS        40000000        /* fps=25 */
#define T41_FIRST_NS    135000000
#define FRAMES          100
#define RAMP_Y          32              /* the ramp's rows: 32 to 47 */

/* The 10 bits the raw WDMA keeps are cut, not rounded: 1/12 code of variance */
#define CUT_VAR         (64.0 * 64.0 / 12.0)
#define CUT_MEAN        31.5

typedef struct Law {
    double read, shot;
} Law;

/* The model's nodes at 1x and 4x, measured on a GC4023 */
static const Law law_1x = { 696, 13.64 };
static const Law law_4x = { 2862, 50.51 };

static QTestState *t41_start(const char *sensor)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 -device image-sensor,"
                                  "bus=i2c0,fps=25,motion=off,%s", sensor);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, 0x10023000 + 0x0c, 1);        /* DPHY_RSTZ */
    qtest_writel(qts, 0x10023000 + 0x10, 1);        /* CSI2_RESETN */
    qtest_writel(qts, VIC(0) + 0x00c, 2);           /* MIPI */
    qtest_writel(qts, VIC(0) + 0x014, 1);           /* RAW10 */
    qtest_writel(qts, VIC(0) + 0x004, W << 16 | H);
    qtest_writel(qts, T41_DMA_EN, 0x3f);
    qtest_writel(qts, T41_WDMA + 0x14, 0x10020);    /* RAW16 */
    qtest_writel(qts, T41_WDMA + 0x1c, W * 2);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, T41_WDMA + 0x04, 1);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS / 2);
    return qts;
}

/* The next frame into BUF */
static void next_frame(QTestState *qts, uint16_t *raw)
{
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, BUF, raw, W * H * 2);
    for (int i = 0; i < W * H; i++) {
        raw[i] = lduw_le_p(&raw[i]);
    }
}

typedef struct Series {
    uint16_t (*frame)[W * H];
    int n;
} Series;

static Series *capture(QTestState *qts, int n)
{
    Series *s = g_new0(Series, 1);

    s->n = n;
    s->frame = g_malloc0_n(n, sizeof(*s->frame));
    for (int i = 0; i < n; i++) {
        next_frame(qts, s->frame[i]);
    }
    return s;
}

static void series_free(Series *s)
{
    g_free(s->frame);
    g_free(s);
}

/* Four bands of the ramp, columns @x0 to @x0 + 7: levels from 4000 up */
static const int bands[] = { 4, 16, 28, 40 };

/*
 * The measured variance of the ramp's samples over the frames against the
 * law at their measured mean, in each band
 */
static void check_law(const Series *s, Law law, double tol)
{
    for (int b = 0; b < ARRAY_SIZE(bands); b++) {
        double var = 0, want = 0;

        for (int x = bands[b]; x < bands[b] + 8; x++) {
            double sum = 0, sq = 0;
            int n = 0;

            for (int f = 0; f < s->n; f++) {
                for (int y = RAMP_Y; y < H; y++) {
                    double v = s->frame[f][y * W + x];

                    sum += v;
                    sq += v * v;
                    n++;
                }
            }
            var += (sq - sum * sum / n) / (n - 1);
            want += law.read + law.shot * (sum / n + CUT_MEAN) + CUT_VAR;
        }
        g_test_message("band %d: variance %.0f, the law's %.0f", b,
                       var / 8, want / 8);
        g_assert_cmpfloat(fabs(var / want - 1), <, tol);
    }
}

/* A still scene changes by twice the variance from one frame to the next */
static void check_diff(const Series *s, Law law, double tol)
{
    for (int b = 0; b < ARRAY_SIZE(bands); b++) {
        double d2 = 0, want = 0;
        int n = 0;

        for (int x = bands[b]; x < bands[b] + 8; x++) {
            double mean = 0;

            for (int f = 0; f < s->n; f++) {
                for (int y = RAMP_Y; y < H; y++) {
                    mean += s->frame[f][y * W + x];
                }
            }
            mean /= s->n * (H - RAMP_Y);
            for (int f = 1; f < s->n; f++) {
                for (int y = RAMP_Y; y < H; y++) {
                    double d = (double)s->frame[f][y * W + x] -
                               s->frame[f - 1][y * W + x];

                    d2 += d * d;
                    n++;
                }
            }
            want += 2 * (law.read + law.shot * (mean + CUT_MEAN) + CUT_VAR);
        }
        want /= 8;
        g_test_message("band %d: frame difference %.0f, the law's %.0f", b,
                       d2 / n, want);
        g_assert_cmpfloat(fabs(d2 / n / want - 1), <, tol);
    }
}

/* A jxq03 has no gain registers: 1x */
static void test_law(void)
{
    QTestState *qts = t41_start("model=jxq03,sensor-noise=on");
    Series *s = capture(qts, FRAMES);

    check_law(s, law_1x, 0.05);
    check_diff(s, law_1x, 0.05);
    series_free(s);
    qtest_quit(qts);
}

/* Without the mode a still scene is still, with it a frame is not the last */
static void test_still(void)
{
    QTestState *qts = t41_start("model=jxq03");
    uint16_t a[W * H], b[W * H];

    next_frame(qts, a);
    next_frame(qts, b);
    g_assert(!memcmp(a, b, sizeof(a)));
    qtest_quit(qts);

    qts = t41_start("model=jxq03,noise=off,sensor-noise=on");
    next_frame(qts, a);
    next_frame(qts, b);
    g_assert(memcmp(a, b, sizeof(a)));
    qtest_quit(qts);
}

/* Seeded by the frame: a run repeats */
static void test_repeat(void)
{
    uint16_t a[W * H], b[W * H];
    QTestState *qts = t41_start("model=jxq03,sensor-noise=on");

    for (int i = 0; i < 5; i++) {
        next_frame(qts, a);
    }
    qtest_quit(qts);
    qts = t41_start("model=jxq03,sensor-noise=on");
    for (int i = 0; i < 5; i++) {
        next_frame(qts, b);
    }
    qtest_quit(qts);
    g_assert(!memcmp(a, b, sizeof(a)));
}

/* A JXF22's gain register 0x00, and its integration time 0x02:0x01 */
static void jxf22_set(QTestState *qts, uint8_t gain, uint16_t lines)
{
    const uint8_t val[3] = { gain, lines & 0xff, lines >> 8 };

    qtest_writel(qts, I2C0 + 0x6c, 0);
    qtest_writel(qts, I2C0 + 0x04, 0x40);
    qtest_writel(qts, I2C0 + 0x6c, 1);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, I2C0 + 0x10, i);
        qtest_writel(qts, I2C0 + 0x10, 0x200 | val[i]);
    }
}

/*
 * At 4x gain, the integration time a quarter of 1x's for the same
 * picture: the law of the 4x node, a higher floor and a steeper shot noise
 */
static void test_gain(void)
{
    QTestState *qts = t41_start("model=jxf22,sensor-noise=on");
    Series *s;

    jxf22_set(qts, 0x00, 600);
    qtest_clock_step(qts, FRAME_NS * 2);
    s = capture(qts, FRAMES);
    check_law(s, law_1x, 0.05);
    series_free(s);

    jxf22_set(qts, 0x20, 150);
    qtest_clock_step(qts, FRAME_NS * 2);
    s = capture(qts, FRAMES);
    check_law(s, law_4x, 0.05);
    check_diff(s, law_4x, 0.05);
    series_free(s);
    qtest_quit(qts);
}

/* read-noise (sigma) and shot-noise (variance per level) replace the nodes */
static void test_override(void)
{
    QTestState *qts = t41_start("model=jxq03,sensor-noise=on,"
                                "read-noise=150,shot-noise=30");
    Law law = { 150 * 150, 30 };
    Series *s = capture(qts, FRAMES);

    check_law(s, law, 0.05);
    series_free(s);
    qtest_quit(qts);
}

#define T20_CORE        (ISP + 0x80000)
#define T20_WR          (T20_CORE + 0xb00)
#define T20_WR_UV       (T20_WR + 0x80)

/*
 * The T20's NV12 picture: a luma level is 1.164 * 257 of the 16-bit
 * sample, and rounds to a whole one (1/12 of variance)
 */
static void test_yuv(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t20 -device image-sensor,"
                                 "model=jxq03,bus=i2c0,fps=25,motion=off,"
                                 "sensor-noise=on");
    static uint8_t y[FRAMES][64 * 32];
    const int x0 = 36, x1 = 52, rows = 32, y0 = 22;
    double var = 0, want = 0;
    int n = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, 64);
    qtest_writel(qts, T20_CORE + 0x14, rows);
    qtest_writel(qts, T20_WR, 13 | 1 << 9);
    qtest_writel(qts, T20_WR + 4, rows << 16 | 64);
    qtest_writel(qts, T20_WR + 8, BUF);
    qtest_writel(qts, T20_WR + 0x20, 64);
    qtest_writel(qts, T20_WR + 0x24, 2);
    qtest_writel(qts, T20_WR_UV, 1 << 9);
    qtest_writel(qts, T20_WR_UV + 4, rows / 2 << 16 | 64);
    qtest_writel(qts, T20_WR_UV + 8, BUF + 64 * rows);
    qtest_writel(qts, T20_WR_UV + 0x20, 64);
    qtest_writel(qts, T20_WR_UV + 0x24, 2);
    qtest_writel(qts, ISP, 1);
    qtest_clock_step(qts, FRAME_NS * 2);
    for (int f = 0; f < FRAMES; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, BUF, y[f], sizeof(y[f]));
    }
    for (int x = x0; x < x1; x++) {
        double sum = 0, sq = 0;
        int m = 0;

        for (int f = 0; f < FRAMES; f++) {
            for (int r = y0; r < rows; r++) {
                double v = y[f][r * 64 + x];

                sum += v;
                sq += v * v;
                m++;
            }
        }
        var += (sq - sum * sum / m) / (m - 1);
        want += (law_1x.read + law_1x.shot * 1.164 * 257 * (sum / m - 16)) /
                pow(1.164 * 257, 2) + 1.0 / 12;
        n++;
    }
    g_test_message("luma variance %.2f, the law's %.2f", var / n, want / n);
    g_assert_cmpfloat(fabs(var / want - 1), <, 0.1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-sensor-noise/law", test_law);
    qtest_add_func("/ingenic-sensor-noise/still", test_still);
    qtest_add_func("/ingenic-sensor-noise/repeat", test_repeat);
    qtest_add_func("/ingenic-sensor-noise/gain", test_gain);
    qtest_add_func("/ingenic-sensor-noise/override", test_override);
    qtest_add_func("/ingenic-sensor-noise/yuv", test_yuv);
    return g_test_run();
}
