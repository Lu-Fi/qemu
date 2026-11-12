/*
 * QTest testcase for the T23's and T31's 3D noise reducer (MDNS)
 *
 * Its reads as a T23N and a T31AL read them, and its buffers as they fill
 * them with libimp's settings: the reference frame, rewritten each frame,
 * and the block ages in the statistics buffers (two 4-bit ages a byte, the
 * first buffer from its second row), 15 where the scene stands still and
 * 0 where it moves; and the reference's noise between frames falling with
 * the reduction on, and its chroma following a white-balance step in a
 * still scene as fast as its limit on a still block's weight lets it, and
 * its luma's noise rising with a lower limit of its own; and a T31's
 * channels taking not the reference but its smoothing and sharpening.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

#define ISP             0x13300000
#define MDNS            (ISP + 0x7800)
#define CPM_CLKGR0      0x10000020
#define W               256
#define H               128
#define BW              (W / 16)
#define BH              (H / 16)
#define STA_STRIDE      8               /* ((W + 31) >> 5) + 7 & ~7 */
#define STA_ROWS        (BH + 1)
#define REF_Y           0x02000000
#define REF_UV          (REF_Y + W * H)
#define STA0            (REF_UV + W * H / 2)
#define STA_SIZE        (STA_STRIDE * STA_ROWS)
#define FRAME_NS        (1000000000 / 30)
#define CH_BUF          0x03000000
#define T41_FIRST_NS    135000000       /* a T41NQ's first frame */

/* The buffers and settings libimp leaves a T23N streaming with (temper 128) */
static void mdns_setup(QTestState *qts, bool shared)
{
    qtest_writel(qts, MDNS + 0x20, REF_Y);
    qtest_writel(qts, MDNS + 0x24, W);
    qtest_writel(qts, MDNS + 0x28, REF_UV);
    qtest_writel(qts, MDNS + 0x2c, W);
    for (int k = 0; k < 4; k++) {
        static const uint32_t at[4] = { 0x30, 0x40, 0x48, 0x50 };

        qtest_writel(qts, MDNS + at[k], STA0 + (shared ? 0 : k * STA_SIZE));
        qtest_writel(qts, MDNS + at[k] + 4, STA_STRIDE);
    }
    for (int k = 0; k < 4; k++) {
        /* columns, rows: the second and fourth grids a row fewer */
        qtest_writel(qts, MDNS + 0x74 + k * 0x18,
                     BW << 24 | (BH - (k & 1)) << 16 | 0x1010);
        qtest_writel(qts, MDNS + 0x78 + k * 0x18, 0x14f0 | !(k % 3));
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xff0500a4);
    }
    qtest_writel(qts, MDNS + 0x144, 0xff053053);
    qtest_writel(qts, MDNS + 0x148, 0xc8b4a000);
    qtest_writel(qts, MDNS + 0x14c, 0xfafaf0dc);
    qtest_writel(qts, MDNS + 0x150, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x154, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x264, 0xff173141);
    qtest_writel(qts, MDNS + 0x19c, 0x00f300ff);    /* a T31X's */
    qtest_writel(qts, MDNS + 0x26c, 0x07d10aff);
    qtest_writel(qts, MDNS + 0x10, 0x11111111);
    qtest_writel(qts, MDNS + 0x14, 0x00f01100);
    qtest_writel(qts, MDNS + 0x04, 0x111);
}

static QTestState *mdns_start(const char *machine, const char *sensor,
                              bool shared)
{
    QTestState *qts = qtest_initf("-machine %s -device "
                                  "image-sensor,model=jxq03,bus=i2c0,id=cam%s",
                                  machine, sensor);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    mdns_setup(qts, shared);
    return qts;
}

static void mdns_frames(QTestState *qts, int n)
{
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS * n + FRAME_NS / 2);
}

/* A block's age in a statistics buffer whose rows start at row @top */
static unsigned mdns_age(const uint8_t *sta, unsigned top, unsigned bx,
                         unsigned by)
{
    return sta[(by + top) * STA_STRIDE + bx / 2] >> (bx & 1) * 4 & 15;
}

static double ref_noise(QTestState *qts)
{
    g_autofree uint8_t *a = g_malloc(W * H), *b = g_malloc(W * H);
    uint64_t sum = 0;

    qtest_memread(qts, REF_Y, a, W * H);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, REF_Y, b, W * H);
    for (int i = 0; i < W * H; i++) {
        sum += abs(a[i] - b[i]);
    }
    return (double)sum / (W * H);
}

static void test_regs(const char *machine, uint32_t id, uint32_t refresh)
{
    QTestState *qts = qtest_initf("-machine %s", machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, MDNS), ==, id);
    qtest_writel(qts, MDNS, 0);
    g_assert_cmphex(qtest_readl(qts, MDNS), ==, id);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0xc), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x1c), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x304), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x7fc), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x300), ==, 0x4040);
    qtest_writel(qts, MDNS + 0x4, 0x110);
    qtest_writel(qts, MDNS + 0x4, 0x111);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x4), ==, 0x110);
    qtest_writel(qts, MDNS + 0x18, 1);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x18), ==, refresh);
    qtest_writel(qts, MDNS + 0x10, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x10), ==, 0x11111111);
    qtest_quit(qts);
}

static void test_t31_regs(void)
{
    test_regs("ingenic-t31", 0x20151120, 0);
}

static void test_t23_regs(void)
{
    test_regs("ingenic-t23", 0x23021515, 0x20);
}

/* A still scene: every block of every grid 15, the reference the scene */
static void test_still(void)
{
    QTestState *qts = mdns_start("ingenic-t31", ",motion=off", false);
    g_autofree uint8_t *sta = g_malloc(4 * STA_SIZE);
    g_autofree uint8_t *y = g_malloc(W * H);
    bool lit = false;

    mdns_frames(qts, 20);
    qtest_memread(qts, STA0, sta, 4 * STA_SIZE);
    for (int k = 0; k < 4; k++) {
        const uint8_t *s = sta + k * STA_SIZE;
        unsigned top = !k, rows = BH - (k & 1);

        for (unsigned by = 0; by < STA_ROWS; by++) {
            for (unsigned i = 0; i < STA_STRIDE; i++) {
                bool data = by >= top && by < top + rows && i < BW / 2;

                g_assert_cmphex(s[by * STA_STRIDE + i], ==, data ? 0xff : 0);
            }
        }
    }
    qtest_memread(qts, REF_Y, y, W * H);
    for (int i = 0; i < W * H; i++) {
        lit |= y[i] > 16;
    }
    g_assert_true(lit);
    g_assert_cmpfloat(ref_noise(qts), ==, 0);
    qtest_quit(qts);
}

/*
 * Noise: the reference holds still with the reduction on, not with it off.
 * The model's noise (+-32) is well above a sensor's: libimp's thresholds at
 * its strongest (temper 255) take it for still
 */
static void test_noise(void)
{
    QTestState *qts = mdns_start("ingenic-t23", ",motion=off,noise=on", true);
    g_autofree uint8_t *sta = g_malloc(STA_SIZE);
    double on, off;

    for (int k = 0; k < 4; k++) {
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xffc60c64);
    }
    qtest_writel(qts, MDNS + 0x144, 0xffc63633);
    mdns_frames(qts, 20);
    on = ref_noise(qts);
    qtest_memread(qts, STA0, sta, STA_SIZE);
    /* one shared buffer: the first's, its first row 0 */
    for (unsigned bx = 0; bx < BW; bx++) {
        g_assert_cmpuint(sta[bx / 2], ==, 0);
        for (unsigned by = 0; by < BH; by++) {
            g_assert_cmpuint(mdns_age(sta, 1, bx, by), ==, 15);
        }
    }
    qtest_writel(qts, MDNS + 0x10, 0x01110111);     /* no Y or UV filter */
    qtest_clock_step(qts, FRAME_NS);
    off = ref_noise(qts);
    g_assert_cmpfloat(off, >, 1);
    g_assert_cmpfloat(on, <, off / 4);
    qtest_quit(qts);
}

static double ref_v(QTestState *qts)
{
    g_autofree uint8_t *uv = g_malloc(W * H / 2);
    uint64_t sum = 0;

    qtest_memread(qts, REF_UV, uv, W * H / 2);
    for (int i = 1; i < W * H / 2; i += 2) {
        sum += uv[i];
    }
    return (double)sum / (W * H / 4);
}

/*
 * A step in the R gain (0x1804) over a still scene: with a still block's UV
 * weight held to 0xd1 (+0x26c bits 23:16), as a T31X holds it, V closes
 * about a fifth of the gap a frame and arrives; with no limit below the
 * age's 0xfa, rounding keeps the reference's V where it was
 */
static void test_uv_step(const void *data)
{
    uint32_t lim = (uintptr_t)data;
    QTestState *qts = mdns_start("ingenic-t31", ",motion=off", false);
    double v0, v1, v16, vt;

    qtest_writel(qts, MDNS + 0x26c, lim);
    qtest_writel(qts, ISP + 0x1804, 0x04000400);
    qtest_writel(qts, ISP + 0x1808, 0x04000400);
    mdns_frames(qts, 20);
    v0 = ref_v(qts);
    qtest_writel(qts, ISP + 0x1804, 0x04000540);
    qtest_clock_step(qts, FRAME_NS);
    v1 = ref_v(qts);
    qtest_clock_step(qts, FRAME_NS * 15);
    v16 = ref_v(qts);
    qtest_writel(qts, MDNS + 0x10, 0x01111111);     /* UV unfiltered */
    qtest_clock_step(qts, FRAME_NS);
    vt = ref_v(qts);
    g_assert_cmpfloat(vt - v0, >, 4);
    if (lim == 0x07d10aff) {
        g_assert_cmpfloat((v1 - v0) / (vt - v0), >, 0.1);
        g_assert_cmpfloat((v1 - v0) / (vt - v0), <, 0.35);
        g_assert_cmpfloat(fabs(vt - v16), <, 2);
    } else {
        g_assert_cmpfloat(fabs(v16 - v0), <, 1);
    }
    qtest_quit(qts);
}

/*
 * Y's limit on a still block's weight (+0x19c bits 23:16): held to 0x80 the
 * reference keeps more of the noise than at a T31X's 0xf3, as on a T31X
 */
static void test_y_limit(void)
{
    QTestState *qts = mdns_start("ingenic-t23", ",motion=off,noise=on", true);
    double at_f3, at_80;

    for (int k = 0; k < 4; k++) {
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xffc60c64);
    }
    qtest_writel(qts, MDNS + 0x144, 0xffc63633);
    mdns_frames(qts, 20);
    at_f3 = ref_noise(qts);
    qtest_writel(qts, MDNS + 0x19c, 0x008000ff);
    qtest_clock_step(qts, FRAME_NS * 4);
    at_80 = ref_noise(qts);
    g_assert_cmpfloat(at_80, >, at_f3 * 2);
    qtest_quit(qts);
}

/* Channel 0 of a T31, W x H, given its buffers for the next frame */
static void ch_push(QTestState *qts, uint32_t buf)
{
    qtest_writel(qts, ISP + 0x9900, W << 16 | H);
    qtest_writel(qts, ISP + 0x9980, W);
    qtest_writel(qts, ISP + 0x9998, W);
    qtest_writel(qts, ISP + 0x996c, buf);
    qtest_writel(qts, ISP + 0x9984, buf + W * H);
}

static void set_words(QTestState *qts, uint32_t addr, int n, uint32_t v)
{
    for (int k = 0; k < n; k++) {
        qtest_writel(qts, addr + k * 4, v);
    }
}

static void ch_frame(QTestState *qts, uint8_t *ref, uint8_t *out,
                     uint32_t buf)
{
    ch_push(qts, buf);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, REF_Y, ref, W * H);
    qtest_memread(qts, buf, out, W * H);
    qtest_readl(qts, ISP + 0x9974);                 /* the done entry */
}

/*
 * What a T31's channels take is the reference smoothed in space by the
 * weights the driver programs (a T31X's 0x6e for the frame, 0x32 for the
 * reference's) and sharpened by the weights at 0x7048 and 0x7058: with
 * none programmed the reference itself, with the filter its noise a good
 * deal less, and with the sharpening more of it again
 */
static void test_spa_sharp(void)
{
    QTestState *qts = mdns_start("ingenic-t31", ",motion=off,noise=on", false);
    g_autofree uint8_t *ref = g_malloc(W * H), *out = g_malloc(W * H);
    g_autofree uint8_t *ref2 = g_malloc(W * H), *out2 = g_malloc(W * H);
    g_autofree uint8_t *plain = g_malloc(W * H);
    double dref = 0, dout = 0;
    int diff = 0, up = 0, down = 0;

    for (int k = 0; k < 4; k++) {
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xffc60c64);
    }
    qtest_writel(qts, MDNS + 0x144, 0xffc63633);
    qtest_writel(qts, ISP + 0x9804, 1);
    mdns_frames(qts, 20);
    ch_frame(qts, ref, out, CH_BUF);
    g_assert_cmpint(memcmp(ref, out, W * H), ==, 0);

    /* a reference that still changes, as a limit of 0xc0 on its weight does */
    qtest_writel(qts, MDNS + 0x19c, 0x00c000ff);
    mdns_frames(qts, 2);
    set_words(qts, MDNS + 0x1b8, 4, 0x6e6e6e6e);
    set_words(qts, MDNS + 0x1f0, 4, 0x32323232);
    ch_frame(qts, ref, out, CH_BUF);
    mdns_frames(qts, 3);
    ch_frame(qts, ref2, out2, CH_BUF);
    for (int i = 0; i < W * H; i++) {
        dref += abs(ref[i] - ref2[i]);
        dout += abs(out[i] - out2[i]);
    }
    g_assert_cmpfloat(dref / (W * H), >, 1);
    g_assert_cmpfloat(dout, <, dref * 0.9);
    g_assert_cmpfloat(dout, >, dref * 0.1);

    /* the sharpening's weights add to it, up and down, never to a flat part */
    qtest_writel(qts, MDNS + 0x19c, 0xffff00ff);
    mdns_frames(qts, 2);
    ch_frame(qts, ref, plain, CH_BUF);
    set_words(qts, ISP + 0x7048, 8, 0x20202020);
    mdns_frames(qts, 1);
    ch_frame(qts, ref, out, CH_BUF);
    for (int i = 0; i < W * H; i++) {
        diff += out[i] != plain[i];
        up += out[i] > plain[i];
        down += out[i] < plain[i];
    }
    g_assert_cmpint(diff, >, W * H / 100);
    g_assert_cmpint(up, >, 0);
    g_assert_cmpint(down, >, 0);
    qtest_quit(qts);
}

/*
 * The pictures a T31's channel takes, pinned: SHA-256s (the first 16 digits)
 * of the channel's frame, of the reference in memory and of the blocks' ages
 * in the statistics buffers, taken from the model as it stood, so that a
 * change to the filters cannot move them unseen; and a T23's, whose
 * channels take the reference. Sizes of one band and of several (those with
 * 2^17 pixels or more), odd ones among them, and a full 1080p frame;
 * weights as libimp leaves them and others set across their ranges; scenes
 * of a sensor's noise, of an object crossing, and hard edges written into
 * the reference (black and white where the 5 x 5's variance is the
 * sharpening's largest). Each is run with the filters' AVX2 build (where
 * the host has it) and the default.
 */
typedef struct PinScene {
    const char *machine;
    unsigned w, h;
    const char *sensor;
    int pattern;                    /* 0: none, 1: blocks, 2: split, 3: lines */
    uint32_t lim;                   /* +0x19c */
    uint32_t spa, fiir, rise, dip;  /* seeds of the weights in 4 words each */
    unsigned frames;
    const char *want[2];            /* after the third frame, then the last */
} PinScene;

static uint32_t pin_rand(uint32_t *seed)
{
    *seed = *seed * 1664525 + 1013904223;
    return *seed >> 8;
}

static void pin_words(QTestState *qts, uint32_t addr, uint32_t seed,
                      uint32_t mask)
{
    for (int k = 0; k < 4; k++) {
        qtest_writel(qts, addr + k * 4, seed ? pin_rand(&seed) * 0x01010101 &
                     mask : 0);
    }
}

static void pin_pattern(uint8_t *y, unsigned w, unsigned h, int kind)
{
    uint32_t seed = 7;

    memset(y, 255, w * h);
    for (unsigned i = 0; kind == 1 && i < w * h / 1500; i++) {
        unsigned x0 = pin_rand(&seed) % w, y0 = pin_rand(&seed) % h;
        unsigned x1 = MIN(w, x0 + 1 + pin_rand(&seed) % 40);
        unsigned y1 = MIN(h, y0 + 1 + pin_rand(&seed) % 14);
        uint8_t v = pin_rand(&seed) & 1 ? 0 : 128;

        for (unsigned r = y0; r < y1; r++) {
            memset(y + r * w + x0, v, x1 - x0);
        }
    }
    for (unsigned r = 0; kind > 1 && r < h; r++) {
        for (unsigned x = 0; x < w; x++) {
            bool on = kind == 2 ? ((x * 3 + r * 2) / 7) % 5 < 2 : x & 1;

            y[r * w + x] = on ? 255 : 0;
        }
    }
}

static void pin_digest(QTestState *qts, const PinScene *sc, const char *want,
                       uint32_t sta, size_t sta_len)
{
    size_t n = sc->w * sc->h * 3 / 2;
    g_autofree uint8_t *buf = g_malloc(n), *ages = g_malloc(sta_len);
    g_autofree char *got = NULL;
    GString *d = g_string_new(NULL);

    qtest_memread(qts, CH_BUF, buf, n);
    got = g_compute_checksum_for_data(G_CHECKSUM_SHA256, buf, n);
    g_string_append_printf(d, "%.16s", got);
    qtest_memread(qts, REF_Y, buf, n);
    g_free(got);
    got = g_compute_checksum_for_data(G_CHECKSUM_SHA256, buf, n);
    g_string_append_printf(d, " %.16s", got);
    /* the blocks' ages the frames left, in the four statistics buffers */
    g_free(got);
    qtest_memread(qts, sta, ages, sta_len);
    got = g_compute_checksum_for_data(G_CHECKSUM_SHA256, ages, sta_len);
    g_string_append_printf(d, " %.16s", got);
    g_assert_cmpstr(d->str, ==, want);
    g_string_free(d, true);
}

static void pin_run(const PinScene *sc, bool avx2)
{
    unsigned w = sc->w, h = sc->h, bw = DIV_ROUND_UP(w, 16);
    unsigned bh = DIV_ROUND_UP(h, 16);
    unsigned stride = (((w + 31) >> 5) + 7) & ~7;
    uint32_t sta = ROUND_UP(REF_Y + w * h * 3 / 2, 0x100);
    g_autofree uint8_t *pic = g_malloc(w * h);
    size_t sta_len = 4 * ROUND_UP(stride * (bh + 1), 0x100);
    g_autofree char *args = g_strdup_printf("-machine %s -global "
                                            "ingenic-isp.x-mdns-avx2=%s "
                                            "-device image-sensor,model=jxq03,"
                                            "bus=i2c0,id=cam%s", sc->machine,
                                            avx2 ? "on" : "off", sc->sensor);
    QTestState *qts = qtest_init(args);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, w << 16 | h);
    qtest_writel(qts, MDNS + 0x20, REF_Y);
    qtest_writel(qts, MDNS + 0x24, w);
    qtest_writel(qts, MDNS + 0x28, REF_Y + w * h);
    qtest_writel(qts, MDNS + 0x2c, w);
    for (int k = 0; k < 4; k++) {
        static const uint32_t at[4] = { 0x30, 0x40, 0x48, 0x50 };

        qtest_writel(qts, MDNS + at[k],
                     sta + k * ROUND_UP(stride * (bh + 1), 0x100));
        qtest_writel(qts, MDNS + at[k] + 4, stride);
        qtest_writel(qts, MDNS + 0x74 + k * 0x18,
                     bw << 24 | (bh - (k & 1)) << 16 | 0x1010);
        qtest_writel(qts, MDNS + 0x78 + k * 0x18, 0x14f0 | !(k % 3));
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xffc60c64);
    }
    qtest_writel(qts, MDNS + 0x144, 0xffc63633);
    qtest_writel(qts, MDNS + 0x148, 0xc8b4a000);
    qtest_writel(qts, MDNS + 0x14c, 0xfafaf0dc);
    qtest_writel(qts, MDNS + 0x150, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x154, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x264, 0xff173141);
    qtest_writel(qts, MDNS + 0x26c, 0x07d10aff);
    qtest_writel(qts, MDNS + 0x10, 0x11111111);
    qtest_writel(qts, MDNS + 0x14, 0x00f01100);
    qtest_writel(qts, MDNS + 0x04, 0x111);
    qtest_writel(qts, ISP + 0x9804, 1);
    qtest_writel(qts, MDNS + 0x19c, sc->lim);
    pin_words(qts, MDNS + 0x1b8, sc->spa, 0xffffffff);
    pin_words(qts, MDNS + 0x1f0, sc->fiir, 0xffffffff);
    pin_words(qts, ISP + 0x7048, sc->rise, 0x3f3f3f3f);
    pin_words(qts, ISP + 0x7058, sc->dip, 0x3f3f3f3f);
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS / 2);
    if (sc->pattern) {
        pin_pattern(pic, w, h, sc->pattern);
        qtest_memwrite(qts, REF_Y, pic, w * h);
        memset(pic, 128, w * h / 2);
        qtest_memwrite(qts, REF_Y + w * h, pic, w * h / 2);
    }
    for (unsigned f = 0; f < sc->frames; f++) {
        qtest_writel(qts, ISP + 0x9900, w << 16 | h);
        qtest_writel(qts, ISP + 0x9980, w);
        qtest_writel(qts, ISP + 0x9998, w);
        qtest_writel(qts, ISP + 0x996c, CH_BUF);
        qtest_writel(qts, ISP + 0x9984, CH_BUF + w * h);
        qtest_clock_step(qts, FRAME_NS);
        qtest_readl(qts, ISP + 0x9974);             /* the done entry */
        if (f == 2 || f == sc->frames - 1) {
            pin_digest(qts, sc, sc->want[f != 2], sta, sta_len);
        }
    }
    qtest_quit(qts);
}

static const PinScene pin_scenes[] = {
    /* one band: libimp's weights on a sensor's noise */
    { "ingenic-t31", 256, 128, ",motion=off,noise=on", 0, 0x00c000ff,
      1, 1, 1, 1, 6,
      { "5461aece6cea2eb1 094eaf9d6e87725b 4c0cb1f5a9d9d5ab",
        "0e8123c6844324d4 c1f84afec5a338ae 4296bbc84dc3734b" } },
    /* the same, an object crossing, odd weights across their ranges */
    { "ingenic-t31", 253, 131, ",object=on,noise=on,sensor-noise=on", 0,
      0x00f300ff, 11, 12, 13, 14, 8,
      { "0374c2016d92ed33 f7f89c4f39156590 14d1a2d360a185f6",
        "fde7316e19fb971c 66d28935377d63a6 a0ead9b75e2e1a79" } },
    /* several bands, odd sizes: noise; hard edges held; lines on noise */
    { "ingenic-t31", 453, 301, ",motion=off,noise=on", 0, 0x00c000ff,
      21, 22, 23, 24, 6,
      { "b11c5400e449113f 15a474b96dcd9519 223408cad93cec9e",
        "1e4cb1c80c1374f0 2bb8b1f0e5e677d1 3ad6275b3651db76" } },
    { "ingenic-t31", 453, 301, ",motion=off", 1, 0xffff00ff,
      31, 32, 33, 34, 5,
      { "077398a27d6d6c5a 422ff0c3fffdc754 f80791e98da47094",
        "1e17d1d4ef8bb6f3 0f885f204419bfb3 a84fc80d82d887fe" } },
    { "ingenic-t31", 421, 313, ",motion=off", 2, 0xffff00ff,
      41, 42, 43, 44, 5,
      { "8dd7ca9b23e849e9 0b35fdf470f16892 4242239c8fdcce09",
        "54572938ac6e5801 2da5a81e01ad56c0 1e40f42dd46c69dc" } },
    { "ingenic-t31", 421, 313, ",motion=off,noise=on", 3, 0x00c000ff,
      51, 52, 53, 54, 5,
      { "32dbb5f799a45300 8e8a1c4b99860f29 4242239c8fdcce09",
        "fd536bf0aa714bb4 43afe3b346d693fc 1e40f42dd46c69dc" } },
    /* the sharpening off, the smoothing alone, in bands */
    { "ingenic-t31", 453, 301, ",motion=off,noise=on", 0, 0x00f300ff,
      61, 62, 0, 0, 4,
      { "a36872f45aab13bb 3acf55ca8689a8c1 223408cad93cec9e",
        "3791d7c27c88565f 3d7da91dbbf517ea 97672daa045b911b" } },
    /* a full frame, as a 1080p stream runs: the case that costs the host */
    { "ingenic-t31", 1920, 1080, ",motion=off,noise=on", 0, 0x00c000ff,
      71, 72, 73, 74, 4,
      { "aafde41110a0a1d1 cf6c1b6f7e987e05 929aa299b40e8dd3",
        "99d880863e600470 06c9565eda768791 257f93c0091def22" } },
    { "ingenic-t23", 1920, 1080, ",object=on,noise=on", 0, 0x00c000ff,
      71, 72, 73, 74, 4,
      { "0f41e6389208cd25 6df7ac089776c1fe 58980654896a1162",
        "0f41e6389208cd25 f95afacab3fccb06 2bbd157693fd3e07" } },
    /* a T23's */
    { "ingenic-t23", 453, 301, ",motion=off,noise=on", 0, 0x00c000ff,
      21, 22, 23, 24, 6,
      { "737a359d6a8f0a42 15a474b96dcd9519 223408cad93cec9e",
        "737a359d6a8f0a42 2bb8b1f0e5e677d1 3ad6275b3651db76" } },
    { "ingenic-t23", 421, 313, ",object=on,noise=on,sensor-noise=on", 0,
      0x00f300ff, 11, 12, 13, 14, 8,
      { "1e1ad712d5e9f3eb 678d9c3917a7e3f5 bb1c0d1295cb2f7a",
        "1e1ad712d5e9f3eb a86e2b5717e58557 a63d46d7b06f995f" } },
    { "ingenic-t23", 421, 313, ",motion=off", 2, 0xffff00ff,
      41, 42, 43, 44, 5,
      { "1e1ad712d5e9f3eb 0b35fdf470f16892 4242239c8fdcce09",
        "1e1ad712d5e9f3eb 2da5a81e01ad56c0 1e40f42dd46c69dc" } },
};

static void test_pin(const void *data)
{
    pin_run(data, true);
    pin_run(data, false);
}

/* The object crossing the ramp: its blocks 0, the rest still 15 */
static void test_motion(void)
{
    QTestState *qts = mdns_start("ingenic-t31", ",object=on", false);
    g_autofree uint8_t *sta = g_malloc(STA_SIZE);
    unsigned moving = 0, still = 0;

    mdns_frames(qts, 20);
    qtest_memread(qts, STA0, sta, STA_SIZE);
    for (unsigned by = 0; by < BH; by++) {
        for (unsigned bx = 0; bx < BW; bx++) {
            unsigned a = mdns_age(sta, 1, bx, by);

            moving += a == 0;
            still += a == 15;
        }
    }
    g_assert_cmpuint(moving, >, 0);
    g_assert_cmpuint(still, >, BW * BH / 2);
    qtest_quit(qts);
}

/*
 * The T41's buffers as tx_isp_set_mdns_buf lays them (a 640 x 360 frame):
 * a still scene leaves every block 7, two a byte from the first row, and
 * the reference the scene
 */
static void test_t41(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device "
                                 "image-sensor,model=gc4023,bus=i2c0,"
                                 "motion=off");
    enum { TW = 640, TH = 360, TBW = 40, TBH = 22, TSTA = 0x28 };
    const uint32_t y = 0x02000000, uv = y + 0x38400, sta = uv + 0x1c400;
    g_autofree uint8_t *st = g_malloc(TSTA * (TBH + 1));
    g_autofree uint8_t *pic = g_malloc(TW * TH);
    bool lit = false;

    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) & ~BIT(23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~BIT(27));
    qtest_writel(qts, ISP + 0x80, TW << 16 | TH);
    qtest_writel(qts, ISP + 0xf040, y);
    qtest_writel(qts, ISP + 0xf044, TW);
    qtest_writel(qts, ISP + 0xf048, uv);
    qtest_writel(qts, ISP + 0xf04c, TW);
    qtest_writel(qts, ISP + 0xf050, sta);
    qtest_writel(qts, ISP + 0xf054, TSTA);
    qtest_writel(qts, ISP + 0xf10c, TBH << 16 | TBW);
    qtest_writel(qts, ISP + 0x80000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 12);
    qtest_memread(qts, sta, st, TSTA * (TBH + 1));
    for (unsigned by = 0; by < TBH; by++) {
        for (unsigned i = 0; i < TBW / 2; i++) {
            g_assert_cmphex(st[by * TSTA + i], ==, 0x77);
        }
    }
    g_assert_cmphex(st[TBW / 2], ==, 0);
    qtest_memread(qts, y, pic, TW * TH);
    for (int i = 0; i < TW * TH; i++) {
        lit |= pic[i] > 16;
    }
    g_assert_true(lit);
    qtest_quit(qts);
}

/*
 * The T41's blocks follow the guest's size (+0x100) and origin (+0x104):
 * 24 pixels square from (6, 4), 10 across and 5 down in a 256 x 128 frame
 * with the object crossing. A block's age is 0 where its mean difference
 * from the frame before, which the reference holds, is over 5, and the
 * rows are written in whole 8-byte groups: what lies past stays as it was
 */
static void test_t41_blocks(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device "
                                 "image-sensor,model=gc4023,bus=i2c0,"
                                 "object=on");
    enum { BS = 24, GX = 6, GY = 4, NX = 10, NY = 5, TSTA = 0x28 };
    const uint32_t y = 0x02000000, uv = y + W * H * 2, sta = uv + W * H;
    g_autofree uint8_t *st = g_malloc(TSTA * (NY + 2));
    g_autofree uint8_t *cur = g_malloc(W * H), *old = g_malloc(W * H);
    unsigned moved = 0;

    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) & ~BIT(23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~BIT(27));
    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    qtest_writel(qts, ISP + 0xf040, y);
    qtest_writel(qts, ISP + 0xf044, W);
    qtest_writel(qts, ISP + 0xf048, uv);
    qtest_writel(qts, ISP + 0xf04c, W);
    qtest_writel(qts, ISP + 0xf050, sta);
    qtest_writel(qts, ISP + 0xf054, TSTA);
    qtest_writel(qts, ISP + 0xf100, BS);
    qtest_writel(qts, ISP + 0xf104, GY << 8 | GX);
    qtest_writel(qts, ISP + 0xf10c, NY << 16 | NX);
    for (unsigned i = 0; i < TSTA * (NY + 2); i++) {
        qtest_writeb(qts, sta + i, 0xa5);
    }
    qtest_writel(qts, ISP + 0x80000, 1);            /* VIC start */
    qtest_clock_step(qts, T41_FIRST_NS - FRAME_NS / 2);
    qtest_memread(qts, y, old, W * H);
    for (int f = 0; f < 40; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, y, cur, W * H);
        qtest_memread(qts, sta, st, TSTA * (NY + 2));
        for (unsigned by = 0; by < NY; by++) {
            for (unsigned bx = 0; bx < NX; bx++) {
                int sum = 0;
                unsigned age;

                for (unsigned yy = 0; yy < BS; yy++) {
                    for (unsigned xx = 0; xx < BS; xx++) {
                        unsigned i = (GY + by * BS + yy) * W +
                                     GX + bx * BS + xx;

                        sum += cur[i] - old[i];
                    }
                }
                age = st[by * TSTA + bx / 2] >> (bx & 1) * 4 & 15;
                g_assert_cmpint(age == 0, ==, abs(sum) > 5 * BS * BS);
                moved += abs(sum) > 5 * BS * BS;
            }
        }
        for (unsigned by = 0; by < NY + 2; by++) {
            for (unsigned i = NX / 2; i < TSTA; i++) {
                g_assert_cmphex(st[by * TSTA + i], ==,
                                by < NY && i < 8 ? 0 : 0xa5);
            }
        }
        memcpy(old, cur, W * H);
    }
    g_assert_cmpuint(moved, >, 0);
    qtest_quit(qts);
}

/*
 * The T40's, whose reference is a coded stream: its buffers are left as the
 * driver set them, and the ages in the first buffer, two a byte in rows of
 * whole 8-byte groups, follow the frame two frames back: a step in the
 * light holds the blocks it moves at 0 for two frames, as a T40XP's do
 */
static void test_t40(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40 -device "
                                 "image-sensor,model=jxq03,bus=i2c0,id=cam,"
                                 "motion=off");
    enum { BS = 20, NX = 12, NY = 6, TSTA = 0x20, ROWS = NY + 2 };
    const uint32_t y = 0x02000000, uv = y + W * H * 2, sta = uv + W * H * 2;
    uint8_t zero[3][NX * NY], st[TSTA * ROWS], pic[64];
    unsigned held = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf000), ==, 0x20191209);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf00c), ==, 0x40);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf100), ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf10c), ==, 0x00100010);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf800), ==, 0x20191209);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf80c), ==, 0x40);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xfb00), ==, 0x100);
    qtest_writel(qts, ISP + 0xf004, 1);
    qtest_writel(qts, ISP + 0xf008, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf004), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf008), ==, 0);

    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    qtest_writel(qts, ISP + 0xf040, y);
    qtest_writel(qts, ISP + 0xf044, W);
    qtest_writel(qts, ISP + 0xf048, uv);
    qtest_writel(qts, ISP + 0xf04c, W);
    qtest_writel(qts, ISP + 0xf050, sta);
    qtest_writel(qts, ISP + 0xf054, TSTA);
    qtest_writel(qts, ISP + 0xf100, BS);
    qtest_writel(qts, ISP + 0xf104, 3);
    qtest_writel(qts, ISP + 0xf10c, NY << 16 | NX);
    for (unsigned i = 0; i < 64; i += 4) {
        qtest_writel(qts, y + i, 0x5a5a5a5a);
    }
    for (unsigned i = 0; i < TSTA * ROWS; i++) {
        qtest_writeb(qts, sta + i, i / TSTA < NY && i % TSTA < 8 ? 0 : 0xa5);
    }
    qtest_writel(qts, ISP + 0x80000, 1);            /* VIC start: a frame */
    qtest_clock_step(qts, FRAME_NS * 2 + FRAME_NS / 2);
    qtest_memread(qts, sta, st, sizeof(st));
    for (unsigned r = 0; r < ROWS; r++) {
        for (unsigned i = 0; i < TSTA; i++) {
            uint8_t want = r >= NY || i >= 8 ? 0xa5 : i < NX / 2 ? 0x33 : 0;

            g_assert_cmphex(st[r * TSTA + i], ==, want);
        }
    }
    qtest_clock_step(qts, FRAME_NS * 9);
    qtest_memread(qts, sta, st, sizeof(st));
    g_assert_cmphex(st[0], ==, 0x77);
    g_assert_cmphex(st[NX / 2], ==, 0);

    /* a step in the light: the blocks it moves are 0 for two frames */
    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': '/machine/peripheral/cam', "
                             "'property': 'light', 'value': 30}}");
    for (unsigned f = 0; f < 3; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, sta, st, sizeof(st));
        for (unsigned i = 0; i < NX * NY; i++) {
            unsigned a = st[i / NX * TSTA + i % NX / 2] >> (i % 2) * 4 & 15;

            zero[f][i] = a == 0;
            if (f == 2 && zero[0][i]) {
                g_assert_cmpuint(a, ==, 1);
            } else if (f == 2) {
                g_assert_cmpuint(a, >, 1);
            }
            if (f && !zero[0][i]) {
                g_assert_false(zero[f][i]);
            }
        }
    }
    for (unsigned i = 0; i < NX * NY; i++) {
        g_assert_cmpuint(zero[1][i], ==, zero[0][i]);
        held += zero[0][i];
    }
    g_assert_cmpuint(held, >, 0);

    qtest_memread(qts, y, pic, sizeof(pic));
    for (unsigned i = 0; i < sizeof(pic); i++) {
        g_assert_cmphex(pic[i], ==, 0x5a);
    }
    qtest_quit(qts);
}

/*
 * The T32's, whose reference is coded too: the buffers stay as the driver
 * set them (it gives a single one at +0x40 here), and the ages in +0x50's
 * are 5-bit fields, twelve to an 8-byte group, up to 15. A step in the
 * light holds the blocks it moves at 0 for one frame
 */
static void test_t32(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 -global "
                                 "ingenic-t31.soc-variant=t32nq -device "
                                 "image-sensor,model=jxq03,bus=i2c0,id=cam,"
                                 "motion=off");
    enum { BS = 14, NX = 17, NY = 9, TSTA = 0x40, ROWS = NY + 2, ROW = 16 };
    const uint32_t y = 0x02000000, sta = y + W * H * 4;
    uint8_t st[TSTA * ROWS], pic[64], zero[NX * NY];
    unsigned held = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd000), ==, 0x24041703);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd004), ==, 0x24041928);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd044), ==, 0x780);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd04c), ==, 0x780);
    qtest_writel(qts, ISP + 0xd014, 1);
    qtest_writel(qts, ISP + 0xd018, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd014), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd018), ==, 0);

    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    qtest_writel(qts, ISP + 0xd040, y);
    qtest_writel(qts, ISP + 0xd080, W * H / 2 >> 8);
    qtest_writel(qts, ISP + 0xd050, sta);
    qtest_writel(qts, ISP + 0xd054, TSTA);
    qtest_writel(qts, ISP + 0xd100, BS);
    qtest_writel(qts, ISP + 0xd104, 1 << 8 | 3);
    qtest_writel(qts, ISP + 0xd10c, NY << 16 | NX);
    for (unsigned i = 0; i < 64; i += 4) {
        qtest_writel(qts, y + i, 0x5a5a5a5a);
    }
    for (unsigned i = 0; i < TSTA * ROWS; i++) {
        qtest_writeb(qts, sta + i, i / TSTA < NY && i % TSTA < ROW ? 0 : 0xa5);
    }
    qtest_writel(qts, ISP + 0x80000 + 0x10, 1);     /* VIC start: a frame */
    qtest_clock_step(qts, FRAME_NS * 2 + FRAME_NS / 2);
    qtest_memread(qts, sta, st, sizeof(st));
    for (unsigned r = 0; r < ROWS; r++) {
        for (unsigned i = 0; i < TSTA; i++) {
            uint8_t want = 0xa5;

            if (r < NY && i < ROW) {
                /* 3 in each field: twelve of them, then five */
                uint64_t g = 0;

                for (unsigned k = 0; k < (i < 8 ? 12 : NX - 12); k++) {
                    g |= 3ull << (5 * k);
                }
                want = g >> (i % 8 * 8);
            }
            g_assert_cmphex(st[r * TSTA + i], ==, want);
        }
    }
    qtest_clock_step(qts, FRAME_NS * 20);
    qtest_memread(qts, sta, st, sizeof(st));
    for (unsigned bx = 0; bx < NX; bx++) {
        uint64_t g = ldq_le_p(st + bx / 12 * 8);

        g_assert_cmpuint(extract64(g, bx % 12 * 5, 5), ==, 15);
    }

    /* a step in the light: the blocks it moves are 0 for one frame */
    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': '/machine/peripheral/cam', "
                             "'property': 'light', 'value': 30}}");
    for (unsigned f = 0; f < 2; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, sta, st, sizeof(st));
        for (unsigned i = 0; i < NX * NY; i++) {
            uint64_t g = ldq_le_p(st + i / NX * TSTA + i % NX / 12 * 8);
            unsigned a = extract64(g, i % NX % 12 * 5, 5);

            if (!f) {
                zero[i] = a == 0;
                held += zero[i];
                g_assert_true(zero[i] || a == 15);
            } else {
                g_assert_cmpuint(a, ==, zero[i] ? 1 : 15);
            }
        }
    }
    g_assert_cmpuint(held, >, 0);

    qtest_memread(qts, y, pic, sizeof(pic));
    for (unsigned i = 0; i < sizeof(pic); i++) {
        g_assert_cmphex(pic[i], ==, 0x5a);
    }
    qtest_quit(qts);
}

/*
 * The T21's second ring, as a T21N fills it: frame n in slot (n - 1) % 16,
 * its count in the header, a block's age in a word; the first ring, whose
 * measure is not decoded, as the driver left it
 */
static void test_t21(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21 -device "
                                 "image-sensor,model=jxq03,bus=i2c0,"
                                 "motion=off");
    enum { SS = W / 16 * 2, SLOT = SS * (BH + 1) };
    const uint32_t ring0 = STA0, ring1 = STA0 + 20 * SLOT;
    g_autofree uint8_t *slot = g_malloc(SLOT);
    uint32_t last = 0, n = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    qtest_writel(qts, ISP + 0x1b10, REF_Y);
    qtest_writel(qts, ISP + 0x1b14, W);
    qtest_writel(qts, ISP + 0x1b20, REF_UV);
    qtest_writel(qts, ISP + 0x1b24, W);
    qtest_writel(qts, ISP + 0x1b30, ring0);
    qtest_writel(qts, ISP + 0x1b34, SS);
    qtest_writel(qts, ISP + 0x1b38, SLOT);
    qtest_writel(qts, ISP + 0x1b40, ring1);
    qtest_writel(qts, ISP + 0x1b44, SS);
    qtest_writel(qts, ISP + 0x1b48, SLOT);
    qtest_writel(qts, ISP + 0x1b80, 15);
    qtest_writel(qts, ISP + 0x10000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 20 + FRAME_NS / 2);
    for (int k = 0; k < 20; k++) {
        uint32_t c = qtest_readl(qts, ring1 + k * SLOT);

        if (k < 16) {
            g_assert_cmpuint(c, >, 0);
            g_assert_cmpuint((c - 1) % 16, ==, k);
            g_assert_cmpuint(qtest_readl(qts, ring0 + k * SLOT), ==, 0);
            if (c > last) {
                last = c;
                n = k;
            }
        } else {
            g_assert_cmpuint(c, ==, 0);
        }
    }
    g_assert_cmpuint(last, >=, 16);
    /* still: every block 15 once past 15 frames, no difference */
    qtest_memread(qts, ring1 + n * SLOT, slot, SLOT);
    for (unsigned i = 0; i < BW * BH; i++) {
        g_assert_cmpuint(slot[SS + i * 2], ==, 15);
        g_assert_cmpuint(slot[SS + i * 2 + 1], ==, 0);
    }
    qtest_memread(qts, ring0 + n * SLOT, slot, SLOT);
    for (unsigned i = 0; i < BW * BH * 2; i++) {
        g_assert_cmpuint(slot[SS + i], ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-mdns/t31/regs", test_t31_regs);
    qtest_add_func("/ingenic-isp-mdns/t23/regs", test_t23_regs);
    qtest_add_func("/ingenic-isp-mdns/still", test_still);
    qtest_add_func("/ingenic-isp-mdns/noise", test_noise);
    qtest_add_func("/ingenic-isp-mdns/motion", test_motion);
    qtest_add_func("/ingenic-isp-mdns/y-limit", test_y_limit);
    qtest_add_func("/ingenic-isp-mdns/spa-sharp", test_spa_sharp);
    for (unsigned i = 0; i < ARRAY_SIZE(pin_scenes); i++) {
        g_autofree char *name = g_strdup_printf("/ingenic-isp-mdns/pin/%u", i);

        qtest_add_data_func(name, &pin_scenes[i], test_pin);
    }
    qtest_add_data_func("/ingenic-isp-mdns/uv-step", (void *)0x07d10affu,
                        test_uv_step);
    qtest_add_data_func("/ingenic-isp-mdns/uv-step/no-limit",
                        (void *)0x00ff0affu, test_uv_step);
    qtest_add_func("/ingenic-isp-mdns/t41", test_t41);
    qtest_add_func("/ingenic-isp-mdns/t41/blocks", test_t41_blocks);
    qtest_add_func("/ingenic-isp-mdns/t40", test_t40);
    qtest_add_func("/ingenic-isp-mdns/t32", test_t32);
    qtest_add_func("/ingenic-isp-mdns/t21", test_t21);
    return g_test_run();
}
