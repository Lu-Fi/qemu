/*
 * QTest testcase for the T23's, T32's and T41's MSCA filter, which scales
 * with the coefficients the guest writes, and the T40's, whose filter is
 * fixed
 *
 * A channel at 1:1 is the frame as it is. Another channel scaled to a smaller
 * or larger size takes the frame through the filter the guest programmed: per
 * axis a table of coefficients by distance (the T41's channel 1 a RAM of
 * phases streamed through a port) and a step, the input over the output in
 * Q14, the pixel at x * step the centre of its taps. The expected picture is
 * made here from the frame channel 0 gives, with a synthetic set of
 * coefficients, vertical pass first rounded to 8 bits, chroma with the luma
 * weights in pairs. Under a level of 128 each pass is made with the curve
 * and with a box of all the taps, both rounded but not clamped, and mixed.
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
#define LEVEL           2       /* a level under 128 */
#define POLY_T21        1       /* the T21's fixed filter */
#define POLY_T40        2       /* the T40's */

typedef struct Part {
    const char *machine, *model, *extra;
    unsigned fixed;             /* a fixed polyphase filter: POLY_* */
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

static const Part t40 = {
    .machine = "ingenic-t40", .model = "imx307",
    .extra = "-global ingenic-t40.soc-variant=t40xp", .fixed = POLY_T40,
    .fsize = 0x80,
    .ch_en = 0x16008, .ch = 0x16100, .size = 0x00, .y = 0x6c, .uv = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done = 0x7c, .done_y = 0x74,
    .vic_start = 0x80000, .first_ms = 135,
};

static const Part t21 = {
    .machine = "ingenic-t21", .model = "ov2735b", .fixed = POLY_T21,
    .fsize = 0x04,
    .ch_en = 0x2304, .ch = 0x2400, .size = 0x00, .y = 0x6c, .uv = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done = 0x7c, .done_y = 0x74,
    .vic_start = 0x10000, .first_ms = 135,
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
    .step_v = 0xd110, .step_h = 0xd114, .step_ch = 0x100,
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
                  uint64_t sy, unsigned flags, unsigned lv, bool half)
{
    const bool blend = flags & LEVEL;
    unsigned lw = half ? dw * 2 : dw, lh = half ? dh * 2 : dh;
    Col *vc = g_new(Col, lh), *hc = g_new(Col, lw);
    uint8_t *tmp = g_malloc(sw * nc * dh);

    cols(p, ch, 0, lh, sy, flags, vc);
    cols(p, ch, 1, lw, sx, flags, hc);
    for (unsigned y = 0; y < dh; y++) {
        const Col *c = &vc[half ? 2 * y : y];

        for (unsigned x = 0; x < sw * nc; x++) {
            int acc = 0, box = 0;

            for (unsigned t = 0; t < p->taps; t++) {
                int r = half ? (c->k0 + (int)t) >> 1 : c->k0 + (int)t;
                int v = src[CLAMP(r, 0, (int)sh - 1) * stride + x];

                acc += v * c->w[t];
                box += v * (2048 / (int)p->taps);
            }
            acc = (acc + 1024) >> 11;
            if (blend) {
                acc = (lv * acc + (128 - lv) * ((box + 1024) >> 11) + 64) >> 7;
            }
            tmp[y * sw * nc + x] = clamp8(acc);
        }
    }
    for (unsigned y = 0; y < dh; y++) {
        for (unsigned x = 0; x < dw; x++) {
            const Col *c = &hc[half ? 2 * x : x];

            for (unsigned k = 0; k < nc; k++) {
                int acc = 0, box = 0;

                for (unsigned t = 0; t < p->taps; t++) {
                    int r = half ? (c->k0 + (int)t) >> 1 : c->k0 + (int)t;
                    int v;

                    r = CLAMP(r, 0, (int)sw - 1);
                    v = tmp[y * sw * nc + r * nc + k];
                    acc += v * c->w[t];
                    box += v * (2048 / (int)p->taps);
                }
                acc = (acc + 1024) >> 11;
                if (blend) {
                    acc = (lv * acc + (128 - lv) * ((box + 1024) >> 11) +
                           64) >> 7;
                }
                dst[y * dstride + x * nc + k] = clamp8(acc);
            }
        }
    }
    g_free(tmp);
    g_free(vc);
    g_free(hc);
}

/*
 * The T21's and T40's four taps, as the T31's MSCA has them: the weights of
 * a phase in 1/512 in Q9 (the table of the model, measured on a T21N and a
 * T31X: the rows of a half turn, the other half their mirror), the position
 * of output x at x * step, the taps the samples from the integer part minus
 * one, and four samples alike are that sample. A T21 has the T31's step,
 * in * 512 / out, and chroma pairs that take the sample at the integer part;
 * a T40's step is (in * 512 - 1) / out, its chroma pairs filtered as luma,
 * and a box of four on the vertical axis when the horizontal one is scaled
 * too.
 */
static const int16_t rows9[257][4] = {
    { 0, 512, 0, 0 }, { 0, 512, 0, 0 }, { 0, 512, 0, 0 },
    { -2, 512, 1, 0 }, { -2, 512, 2, 0 }, { -3, 512, 2, 0 },
    { -3, 512, 3, 0 }, { -4, 512, 3, -1 }, { -4, 511, 4, 0 },
    { -5, 511, 4, 0 }, { -5, 511, 5, -1 }, { -6, 511, 6, 0 },
    { -6, 511, 6, -1 }, { -6, 511, 7, 0 }, { -7, 511, 7, -1 },
    { -7, 511, 8, -1 }, { -8, 510, 8, -1 }, { -8, 510, 9, -1 },
    { -9, 510, 10, -1 }, { -9, 510, 10, -1 }, { -10, 510, 11, -1 },
    { -10, 510, 12, -1 }, { -10, 509, 12, -1 }, { -11, 509, 13, -1 },
    { -11, 509, 14, -1 }, { -12, 509, 14, -1 }, { -12, 508, 15, -1 },
    { -12, 508, 16, -1 }, { -13, 508, 17, -1 }, { -13, 508, 17, -1 },
    { -14, 507, 18, -1 }, { -14, 507, 19, -1 }, { -15, 507, 19, -1 },
    { -15, 506, 20, -1 }, { -15, 506, 21, -1 }, { -16, 506, 22, -2 },
    { -16, 506, 22, -2 }, { -16, 505, 23, -2 }, { -17, 505, 24, -2 },
    { -17, 505, 25, -2 }, { -17, 504, 25, -2 }, { -18, 504, 26, -2 },
    { -18, 503, 27, -2 }, { -18, 503, 28, -2 }, { -19, 503, 29, -2 },
    { -19, 502, 30, -2 }, { -19, 502, 30, -2 }, { -20, 501, 31, -2 },
    { -20, 501, 32, -3 }, { -21, 501, 33, -3 }, { -21, 500, 34, -3 },
    { -21, 500, 35, -3 }, { -21, 499, 35, -3 }, { -22, 499, 36, -3 },
    { -22, 498, 37, -3 }, { -22, 498, 38, -3 }, { -23, 497, 39, -3 },
    { -23, 497, 40, -3 }, { -23, 496, 41, -3 }, { -24, 496, 42, -4 },
    { -24, 495, 42, -3 }, { -24, 495, 43, -4 }, { -24, 494, 44, -4 },
    { -25, 494, 45, -4 }, { -25, 493, 46, -4 }, { -25, 493, 47, -4 },
    { -25, 492, 48, -4 }, { -26, 491, 49, -4 }, { -26, 491, 50, -4 },
    { -26, 490, 51, -5 }, { -27, 490, 52, -5 }, { -27, 489, 53, -5 },
    { -27, 488, 54, -5 }, { -27, 488, 55, -5 }, { -28, 487, 56, -5 },
    { -28, 487, 57, -5 }, { -28, 486, 58, -5 }, { -28, 485, 59, -5 },
    { -28, 485, 60, -5 }, { -29, 484, 61, -6 }, { -29, 483, 62, -6 },
    { -29, 483, 63, -6 }, { -29, 482, 64, -6 }, { -30, 481, 65, -6 },
    { -30, 481, 66, -6 }, { -30, 480, 67, -6 }, { -30, 479, 68, -6 },
    { -30, 479, 69, -7 }, { -30, 478, 70, -7 }, { -31, 477, 71, -7 },
    { -31, 476, 72, -7 }, { -31, 476, 73, -7 }, { -31, 475, 74, -7 },
    { -32, 474, 75, -7 }, { -32, 473, 76, -7 }, { -32, 472, 77, -7 },
    { -32, 472, 78, -8 }, { -32, 471, 80, -8 }, { -32, 470, 81, -8 },
    { -33, 469, 82, -8 }, { -33, 468, 83, -8 }, { -33, 468, 84, -8 },
    { -33, 467, 85, -8 }, { -33, 466, 86, -9 }, { -33, 465, 87, -9 },
    { -34, 464, 89, -9 }, { -34, 464, 90, -9 }, { -34, 463, 91, -9 },
    { -34, 462, 92, -9 }, { -34, 461, 93, -9 }, { -34, 460, 94, -10 },
    { -34, 459, 95, -10 }, { -35, 458, 96, -10 }, { -35, 458, 98, -10 },
    { -35, 457, 99, -10 }, { -35, 456, 100, -10 }, { -35, 455, 101, -11 },
    { -35, 454, 102, -11 }, { -35, 453, 104, -11 }, { -35, 452, 105, -11 },
    { -35, 451, 106, -11 }, { -36, 450, 107, -11 }, { -36, 449, 108, -11 },
    { -36, 448, 110, -12 }, { -36, 447, 111, -12 }, { -36, 446, 112, -12 },
    { -36, 445, 113, -12 }, { -36, 444, 114, -12 }, { -36, 444, 116, -12 },
    { -36, 443, 117, -12 }, { -36, 442, 118, -13 }, { -37, 441, 119, -13 },
    { -37, 440, 120, -13 }, { -37, 439, 122, -13 }, { -37, 438, 123, -13 },
    { -37, 437, 124, -14 }, { -37, 436, 125, -14 }, { -37, 435, 127, -14 },
    { -37, 434, 128, -14 }, { -37, 433, 129, -14 }, { -37, 432, 131, -14 },
    { -37, 431, 132, -14 }, { -38, 430, 133, -15 }, { -37, 428, 134, -15 },
    { -38, 427, 135, -15 }, { -38, 426, 137, -15 }, { -38, 425, 138, -15 },
    { -38, 424, 139, -15 }, { -38, 423, 141, -16 }, { -38, 422, 142, -16 },
    { -38, 421, 143, -16 }, { -38, 420, 144, -16 }, { -38, 419, 146, -16 },
    { -38, 418, 147, -16 }, { -38, 417, 148, -16 }, { -38, 416, 150, -17 },
    { -38, 414, 151, -17 }, { -38, 413, 152, -17 }, { -38, 412, 154, -17 },
    { -38, 411, 155, -17 }, { -38, 410, 156, -18 }, { -38, 409, 157, -18 },
    { -38, 408, 159, -18 }, { -38, 407, 160, -18 }, { -38, 405, 161, -18 },
    { -38, 404, 163, -18 }, { -38, 403, 164, -18 }, { -38, 402, 165, -19 },
    { -38, 401, 167, -19 }, { -38, 400, 168, -19 }, { -38, 399, 169, -19 },
    { -38, 397, 171, -19 }, { -38, 396, 172, -20 }, { -38, 395, 173, -20 },
    { -38, 394, 175, -20 }, { -38, 393, 176, -20 }, { -38, 391, 177, -20 },
    { -38, 390, 179, -20 }, { -38, 389, 180, -21 }, { -38, 388, 182, -21 },
    { -38, 387, 183, -21 }, { -38, 386, 184, -21 }, { -38, 384, 186, -21 },
    { -38, 383, 187, -21 }, { -38, 382, 188, -21 }, { -38, 381, 190, -22 },
    { -38, 380, 191, -22 }, { -38, 378, 192, -22 }, { -38, 377, 194, -22 },
    { -38, 376, 195, -22 }, { -38, 374, 196, -22 }, { -38, 373, 198, -23 },
    { -38, 372, 199, -23 }, { -38, 371, 200, -23 }, { -38, 370, 202, -23 },
    { -38, 368, 203, -23 }, { -38, 367, 205, -23 }, { -38, 366, 206, -24 },
    { -38, 365, 207, -24 }, { -38, 363, 209, -24 }, { -37, 362, 210, -24 },
    { -37, 361, 211, -24 }, { -38, 360, 213, -25 }, { -37, 358, 214, -25 },
    { -37, 357, 216, -25 }, { -37, 356, 217, -25 }, { -37, 355, 218, -25 },
    { -37, 353, 220, -25 }, { -37, 352, 221, -26 }, { -37, 350, 222, -26 },
    { -37, 349, 224, -26 }, { -37, 348, 225, -26 }, { -37, 347, 227, -26 },
    { -37, 345, 228, -26 }, { -37, 344, 229, -26 }, { -37, 343, 231, -27 },
    { -36, 341, 232, -27 }, { -36, 340, 234, -27 }, { -36, 339, 235, -27 },
    { -36, 338, 236, -27 }, { -36, 336, 238, -27 }, { -36, 335, 239, -28 },
    { -36, 334, 241, -28 }, { -36, 332, 242, -28 }, { -36, 331, 243, -28 },
    { -36, 330, 245, -28 }, { -36, 328, 246, -28 }, { -36, 327, 247, -28 },
    { -35, 326, 249, -28 }, { -35, 324, 250, -29 }, { -35, 323, 252, -29 },
    { -35, 322, 253, -29 }, { -35, 320, 254, -29 }, { -35, 319, 256, -29 },
    { -35, 318, 257, -29 }, { -35, 316, 259, -30 }, { -35, 315, 260, -30 },
    { -34, 313, 261, -30 }, { -34, 312, 263, -30 }, { -34, 311, 264, -30 },
    { -34, 309, 265, -30 }, { -34, 308, 267, -30 }, { -34, 307, 268, -31 },
    { -34, 305, 270, -31 }, { -34, 304, 271, -31 }, { -34, 303, 272, -31 },
    { -34, 301, 274, -31 }, { -33, 300, 275, -31 }, { -33, 299, 277, -31 },
    { -33, 297, 278, -32 }, { -33, 296, 279, -32 }, { -33, 294, 281, -32 },
    { -33, 293, 282, -32 }, { -33, 292, 283, -32 }, { -33, 290, 285, -32 },
    { -32, 289, 286, -32 }, { -32, 288, 288, -32 },
};

static void poly_cols(unsigned kind, unsigned n_in, unsigned n_out,
                      unsigned nc, bool box, int *k0, int w[][4])
{
    unsigned step = kind == POLY_T40 ?
                    (n_in == n_out ? 512 : ((n_in << 9) - 1) / n_out) :
                    (n_in << 9) / n_out;

    for (unsigned x = 0; x < n_out; x++) {
        unsigned pos = x * step, ph = pos & 511;

        k0[x] = (pos >> 9) - 1;
        for (unsigned t = 0; t < 4; t++) {
            w[x][t] = (ph <= 256 ? rows9[ph][t] : rows9[512 - ph][3 - t]) * 8;
            if (box) {
                w[x][t] = 1024;
            } else if (kind == POLY_T21 && nc == 2) {
                w[x][t] = t == 1 ? 4096 : 0;
            }
        }
    }
}

/* Four samples through four weights in Q12, rounded, or one if alike */
static int poly_tap(const int *s, const int *w)
{
    int acc = 2048;

    if (s[0] == s[1] && s[1] == s[2] && s[2] == s[3]) {
        return s[1];
    }
    for (unsigned t = 0; t < 4; t++) {
        acc += s[t] * w[t];
    }
    return clamp8(acc >> 12);
}

static void poly_scale(unsigned kind, const uint8_t *src, unsigned sw,
                       unsigned sh, unsigned stride, unsigned nc,
                       uint8_t *dst, unsigned dw, unsigned dh,
                       unsigned dstride)
{
    bool both = kind == POLY_T40 && sw != dw && sh != dh;
    int *vk = g_new(int, dh), *hk = g_new(int, dw);
    int (*vw)[4] = g_malloc(dh * sizeof(*vw));
    int (*hw)[4] = g_malloc(dw * sizeof(*hw));
    uint8_t *tmp = g_malloc(sw * nc * dh);

    poly_cols(kind, sh, dh, nc, both, vk, vw);
    poly_cols(kind, sw, dw, nc, false, hk, hw);
    for (unsigned y = 0; y < dh; y++) {
        for (unsigned x = 0; x < sw * nc; x++) {
            int s[4];

            for (unsigned t = 0; t < 4; t++) {
                s[t] = src[CLAMP(vk[y] + (int)t, 0, (int)sh - 1) * stride + x];
            }
            tmp[y * sw * nc + x] = poly_tap(s, vw[y]);
        }
    }
    for (unsigned y = 0; y < dh; y++) {
        for (unsigned x = 0; x < dw; x++) {
            for (unsigned k = 0; k < nc; k++) {
                int s[4];

                for (unsigned t = 0; t < 4; t++) {
                    int r = CLAMP(hk[x] + (int)t, 0, (int)sw - 1);

                    s[t] = tmp[y * sw * nc + r * nc + k];
                }
                dst[y * dstride + x * nc + k] = poly_tap(s, hw[x]);
            }
        }
    }
    g_free(tmp);
    g_free(vk);
    g_free(hk);
    g_free(vw);
    g_free(hw);
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
                       unsigned flags, unsigned lv)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,model=%s,"
                                  "bus=i2c0,fps=30,motion=off %s", p->machine,
                                  p->model, p->extra ? p->extra : "");
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
    if (!p->fixed) {
        write_tables(qts, p, ch);
    }
    if (flags & FIXED) {
        qtest_writel(qts, ISP + p->mode, 0x11 << ch);
    }
    if (flags & LEVEL) {
        qtest_writel(qts, ISP + p->level + ch * 8, lv * 0x101);
    }
    if (p->step_v) {
        qtest_writel(qts, ISP + p->step_v + ch * p->step_ch, sy);
        qtest_writel(qts, ISP + p->step_h + ch * p->step_ch, sx);
        if (p == &t23 && !ch) {
            /* channel 1's ratios, which channel 0 must not read */
            qtest_writel(qts, ISP + p->step_v + p->step_ch, sy * 2);
            qtest_writel(qts, ISP + p->step_h + p->step_ch, sx * 2);
        }
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
    if (p->fixed) {
        poly_scale(p->fixed, ref, FW, FH, FW, 1, want, ow, oh, ow);
        poly_scale(p->fixed, ref + FW * FH, FW / 2, FH / 2, FW, 2,
                   want + ow * oh, ow / 2, oh / 2, ow);
    } else {
        scale(p, ch, ref, FW, FH, FW, 1, want, ow, oh, ow, sx, sy, flags, lv,
              false);
        scale(p, ch, ref + FW * FH, FW / 2, FH / 2, FW, 2, want + ow * oh,
              ow / 2, oh / 2, ow, sx, sy, flags, lv, true);
    }
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
    unsigned ch, w, h, flags, lv;
} Case;

static const Case cases[] = {
    { &t41, 1, 96, 48 }, { &t41, 1, 160, 80 }, { &t41, 0, 96, 48 },
    { &t41, 2, 96, 48 }, { &t32, 1, 96, 48 }, { &t32, 1, 160, 80 },
    { &t23, 0, 96, 48 }, { &t23, 1, 96, 48 }, { &t41, 0, 96, 48, FIXED },
    { &t41, 2, 64, 32, LEVEL, 64 },
    { &t41, 1, 96, 48, FIXED }, { &t41, 1, 98, 50 }, { &t23, 1, 66, 38 },
    { &t41, 0, 50, 30, FIXED }, { &t41, 0, 64, 32, LEVEL, 0 },
    { &t41, 1, 64, 32, LEVEL, 100 }, { &t41, 0, 96, 48, LEVEL, 16 },
    { &t40, 1, 96, 48 }, { &t40, 1, 128, 32 }, { &t40, 1, 64, 64 },
    { &t40, 1, 40, 22 }, { &t21, 1, 96, 48 }, { &t21, 1, 128, 32 },
    { &t21, 1, 40, 22 }, { &t21, 1, 32, 16 },
};

static void test_case(const void *data)
{
    const Case *c = data;

    test_scale(c->part, c->ch, c->w, c->h, c->flags, c->lv);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (unsigned i = 0; i < ARRAY_SIZE(cases); i++) {
        const Case *c = &cases[i];
        g_autofree char *name =
            g_strdup_printf("/ingenic-isp-msca/%s/ch%u/%ux%u/%u/%u",
                            c->part->machine + 8, c->ch, c->w, c->h, c->flags,
                            c->lv);

        qtest_add_data_func(name, c, test_case);
    }
    return g_test_run();
}
