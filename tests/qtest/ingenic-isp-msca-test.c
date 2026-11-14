/*
 * QTest testcase for the T23's, T32's and T41's MSCA filter, which scales
 * with the coefficients the guest writes
 *
 * A channel at 1:1 is the frame as it is. Another channel scaled to a smaller
 * or larger size takes the frame through the filter the guest programmed: per
 * axis a table of coefficients by distance (the T41's channel 1 a RAM of
 * phases streamed through a port) and a step, the input over the output in
 * Q14, the pixel at x * step the centre of its taps. The expected picture is
 * made here from the frame channel 0 gives, with a synthetic set of
 * coefficients, vertical pass first rounded to 8 bits, chroma with the luma
 * weights in pairs.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define FW              128
#define FH              64
#define BUF0            0x02000000
#define BUF1            0x03000000
#define MAXN            3
#define FIXED           1       /* the channel takes its first taps */
#define LEVEL           2       /* half a box in its weights */

typedef struct Part {
    const char *machine, *model;
    uint32_t fsize;             /* the frame's size */
    uint32_t ch_en, ch, size, y, uv, y_stride, uv_stride, done, done_y;
    uint32_t vic_start, trig;
    uint32_t step_v, step_h, step_ch;   /* 0: not given, sizes */
    unsigned taps, ncoef;
    uint32_t lin[MAXN][2];      /* 0: the T41's RAM ports at ram */
    uint32_t ram, mode, level;
    uint32_t first_ms;
} Part;

static const Part t41 = {
    .machine = "ingenic-t41", .model = "gc4023", .fsize = 0x80,
    .ch_en = 0xf0008, .ch = 0xf0100, .size = 0x00, .y = 0x6c, .uv = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done = 0x7c, .done_y = 0x74,
    .vic_start = 0x80000, .trig = 0xf0010,
    .step_v = 0xf071c, .step_h = 0xf0728, .step_ch = 4,
    .taps = 8, .ncoef = 33,
    .lin = { { 0xf0740, 0xf0790 }, { 0, 0 }, { 0xf0840, 0xf0890 } },
    .ram = 0xf1100, .mode = 0xf0028, .level = 0xf0708, .first_ms = 135,
};

static const Part t32 = {
    .machine = "ingenic-t32", .model = "gc4653", .fsize = 0x80,
    .ch_en = 0xf0040, .ch = 0xf0100, .size = 0x04, .y = 0x30, .uv = 0x50,
    .y_stride = 0x40, .uv_stride = 0x60, .done = 0x3c, .done_y = 0x38,
    .vic_start = 0x80010,
    .step_v = 0xf0110, .step_h = 0xf0114, .step_ch = 0x100,
    .taps = 6, .ncoef = 25,
    .lin = { { 0xf0870, 0xf0820 }, { 0xf0910, 0xf08c0 },
             { 0xf09b0, 0xf0960 } },
};

static const Part t23 = {
    .machine = "ingenic-t23", .model = "jxq03", .fsize = 0x04,
    .ch_en = 0xd040, .ch = 0xd100, .size = 0x04, .y = 0x30, .uv = 0x50,
    .y_stride = 0x40, .uv_stride = 0x60, .done = 0x3c, .done_y = 0x38,
    .vic_start = 0xe0000,
    .taps = 4, .ncoef = 17,
    .lin = { { 0xd800, 0xd830 }, { 0xd860, 0xd890 }, { 0xd8c0, 0xd8f0 } },
};

/*
 * A synthetic set: a falling ramp over the first three quarters of the
 * table's entries and a shallow negative lobe after, the weights of a
 * phase adding up to about 2048
 */
static int coef_at(const Part *p, unsigned axis, unsigned i)
{
    int n = p->ncoef, amp = 2048 / ((n - 1) / 8 * 2 + (n - 1) / 8 + 3) / 4;
    int v = (n - 1 - i) * (axis ? 14 : 12) * amp / 32;

    if (i > n * 3 / 4) {
        v = -(int)(i - n * 3 / 4) * 3;
    }
    return v;
}

/* The T41 channel 1's curve, 257 entries, 64 to a pixel: the same ramp */
static int ram_curve(unsigned axis, unsigned i)
{
    int v = (256 - (int)i) * 2;

    return i > 192 ? -(int)(i - 192) * (axis ? 3 : 2) : v;
}

/* The curve's entry at @i for a channel's axis: its table, or its RAM's */
static int curve_at(const Part *p, unsigned ch, unsigned axis, unsigned i)
{
    return p->lin[ch][0] ? coef_at(p, axis, i) : ram_curve(axis, i);
}

/* As the driver streams the RAM: 33 phases of 8 taps, two to a word */
static void write_ram(QTestState *qts, const Part *p, unsigned axis)
{
    qtest_writel(qts, ISP + p->ram + axis * 8, 1);
    for (unsigned ph = 0; ph <= 32; ph++) {
        for (unsigned t = 0; t < 8; t += 2) {
            uint32_t w[2];

            for (unsigned h = 0; h < 2; h++) {
                w[h] = ram_curve(axis, t + h < 4 ? 256 - (t + h) * 64 - ph :
                                       (t + h - 4) * 64 + ph) & 0x1fff;
            }
            qtest_writel(qts, ISP + p->ram + axis * 8 + 4, w[1] << 16 | w[0]);
        }
    }
}

static void write_tables(QTestState *qts, const Part *p, unsigned ch)
{
    for (unsigned a = 0; a < 2; a++) {
        if (!p->lin[ch][0]) {
            write_ram(qts, p, a);
            continue;
        }
        for (unsigned i = 0; i < p->ncoef; i += 2) {
            uint32_t lo = coef_at(p, a, i) & 0x1fff;
            uint32_t hi = i + 1 < p->ncoef ? coef_at(p, a, i + 1) & 0x1fff : 0;

            qtest_writel(qts, ISP + p->lin[ch][a] + i / 2 * 4, hi << 16 | lo);
        }
    }
}

typedef struct Col {
    int k0;
    int w[8];
} Col;

/* One axis's columns: output x stands at x * step, its taps around it */
static void cols(const Part *p, unsigned ch, unsigned axis, unsigned n,
                 uint64_t step, unsigned flags, Col *c)
{
    bool ram = !p->lin[ch][0];
    unsigned stride = ram ? 64 : 8, sh = ram ? 8 : 11;
    int j0 = 1 - (int)p->taps / 2;

    for (unsigned x = 0; x < n; x++) {
        uint64_t pos = x * step;
        unsigned ph = ((pos & 0x3fff) + 16 + (1 << (sh - 1))) >> sh;

        c[x].k0 = (pos >> 14) + j0;
        for (unsigned t = 0; t < p->taps; t++) {
            unsigned d = abs((j0 + (int)t) * (int)stride - (int)ph);

            c[x].w[t] = d < (ram ? 257 : p->ncoef) ?
                        curve_at(p, ch, axis, d) : 0;
            if (flags & FIXED) {
                c[x].w[t] = ram ? ram_curve(axis, t < 4 ? 256 - t * 64 :
                                            (t - 4) * 64) :
                            coef_at(p, axis, t);
            } else if (flags & LEVEL) {
                c[x].w[t] = (64 * c[x].w[t] + 64 * (2048 / p->taps) + 64) >> 7;
            }
        }
    }
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/*
 * A plane of @nc interleaved components, @sw x @sh with a stride of @stride,
 * to @dw x @dh: vertical pass, rounded to 8 bits, then horizontal. @half
 * takes the luma columns at twice the column and its taps in pairs.
 */
static void scale(const Part *p, unsigned ch, const uint8_t *src, unsigned sw,
                  unsigned sh, unsigned stride, unsigned nc, uint8_t *dst,
                  unsigned dw, unsigned dh, unsigned dstride, uint64_t sx,
                  uint64_t sy, unsigned flags, bool half)
{
    unsigned lw = half ? dw * 2 : dw, lh = half ? dh * 2 : dh;
    Col *vc = g_new(Col, lh), *hc = g_new(Col, lw);
    uint8_t *tmp = g_malloc(sw * nc * dh);

    cols(p, ch, 0, lh, sy, flags, vc);
    cols(p, ch, 1, lw, sx, flags, hc);
    for (unsigned y = 0; y < dh; y++) {
        const Col *c = &vc[half ? 2 * y : y];

        for (unsigned x = 0; x < sw * nc; x++) {
            int acc = 0;

            for (unsigned t = 0; t < p->taps; t++) {
                int r = half ? (c->k0 + (int)t) >> 1 : c->k0 + (int)t;

                acc += src[CLAMP(r, 0, (int)sh - 1) * stride + x] * c->w[t];
            }
            tmp[y * sw * nc + x] = clamp8((acc + 1024) >> 11);
        }
    }
    for (unsigned y = 0; y < dh; y++) {
        for (unsigned x = 0; x < dw; x++) {
            const Col *c = &hc[half ? 2 * x : x];

            for (unsigned k = 0; k < nc; k++) {
                int acc = 0;

                for (unsigned t = 0; t < p->taps; t++) {
                    int r = half ? (c->k0 + (int)t) >> 1 : c->k0 + (int)t;

                    r = CLAMP(r, 0, (int)sw - 1);
                    acc += tmp[y * sw * nc + r * nc + k] * c->w[t];
                }
                dst[y * dstride + x * nc + k] = clamp8((acc + 1024) >> 11);
            }
        }
    }
    g_free(tmp);
    g_free(vc);
    g_free(hc);
}

static void push(QTestState *qts, const Part *p, unsigned ch, uint32_t w,
                 uint32_t h, uint32_t y, uint32_t uv)
{
    uint32_t base = ISP + p->ch + ch * 0x100;

    qtest_writel(qts, base + p->size, w << 16 | h);
    qtest_writel(qts, base + p->y_stride, w);
    qtest_writel(qts, base + p->uv_stride, w);
    qtest_writel(qts, base + p->y, y);
    qtest_writel(qts, base + p->uv, uv);
}

static void test_scale(const Part *p, unsigned ch, unsigned ow, unsigned oh,
                       unsigned flags)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,model=%s,"
                                  "bus=i2c0,fps=30,motion=off", p->machine,
                                  p->model);
    uint64_t frame_ns = 1000000000ull / 30;
    uint64_t sx = (((uint64_t)FW << 14) + ow / 2) / ow;
    uint64_t sy = (((uint64_t)FH << 14) + oh / 2) / oh;
    g_autofree uint8_t *ref = g_malloc(FW * FH * 3 / 2);
    g_autofree uint8_t *got = g_malloc(ow * oh * 3 / 2);
    g_autofree uint8_t *want = g_malloc(ow * oh * 3 / 2);
    unsigned rc = ch ? 0 : 1;           /* the channel left at 1:1 */

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    qtest_writel(qts, ISP + p->fsize, FW << 16 | FH);
    qtest_writel(qts, ISP + p->ch_en, BIT(rc) | BIT(ch));
    write_tables(qts, p, ch);
    if (flags & FIXED) {
        qtest_writel(qts, ISP + p->mode, 0x11 << ch);
    }
    if (flags & LEVEL) {
        qtest_writel(qts, ISP + p->level + ch * 8, 0x4040);
    }
    if (p->step_v) {
        qtest_writel(qts, ISP + p->step_v + ch * p->step_ch, sy);
        qtest_writel(qts, ISP + p->step_h + ch * p->step_ch, sx);
    }
    push(qts, p, rc, FW, FH, BUF0, BUF0 + FW * FH);
    push(qts, p, ch, ow, oh, BUF1, BUF1 + ow * oh);
    if (p->trig) {
        qtest_writel(qts, ISP + p->trig, 1);
    }
    qtest_writel(qts, ISP + p->vic_start, 1);
    qtest_clock_step(qts, frame_ns * 2 + p->first_ms * 1000000ull);
    qtest_memread(qts, BUF0, ref, FW * FH * 3 / 2);
    qtest_memread(qts, BUF1, got, ow * oh * 3 / 2);
    scale(p, ch, ref, FW, FH, FW, 1, want, ow, oh, ow, sx, sy, flags,
          false);
    scale(p, ch, ref + FW * FH, FW / 2, FH / 2, FW, 2, want + ow * oh, ow / 2,
          oh / 2, ow, sx, sy, flags, true);
    for (unsigned i = 0; i < ow * oh * 3 / 2; i++) {
        if (got[i] != want[i]) {
            g_test_message("%s byte %u (%u,%u): %u, expected %u",
                           i < ow * oh ? "luma" : "chroma", i,
                           i % ow, i / ow, got[i], want[i]);
        }
    }
    g_assert_cmpint(memcmp(got, want, ow * oh * 3 / 2), ==, 0);
    qtest_quit(qts);
}

typedef struct Case {
    const Part *part;
    unsigned ch, w, h, flags;
} Case;

static const Case cases[] = {
    { &t41, 1, 96, 48 }, { &t41, 1, 160, 80 }, { &t41, 0, 96, 48 },
    { &t41, 2, 96, 48 }, { &t32, 1, 96, 48 }, { &t32, 1, 160, 80 },
    { &t23, 1, 96, 48 }, { &t41, 0, 96, 48, FIXED }, { &t41, 2, 64, 32, LEVEL },
    { &t41, 1, 96, 48, FIXED }, { &t41, 1, 98, 50 }, { &t23, 1, 66, 38 },
    { &t41, 0, 50, 30, FIXED },
};

static void test_case(const void *data)
{
    const Case *c = data;

    test_scale(c->part, c->ch, c->w, c->h, c->flags);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (unsigned i = 0; i < ARRAY_SIZE(cases); i++) {
        const Case *c = &cases[i];
        g_autofree char *name =
            g_strdup_printf("/ingenic-isp-msca/%s/ch%u/%ux%u/%u",
                            c->part->machine + 8, c->ch, c->w, c->h, c->flags);

        qtest_add_data_func(name, c, test_case);
    }
    return g_test_run();
}
