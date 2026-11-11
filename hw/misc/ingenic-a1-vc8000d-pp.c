/*
 * Ingenic A1 VC8000D: the post-processor
 *
 * Two output units sit behind the decoder: each takes a window of the
 * picture the core just decoded, scales it to the size libimp's channel
 * attributes ask for and writes it as NV12 or NV21 to its own buffers.
 * libimp's PPSetRegs programs them in the core's register file: unit 0 at
 * word 0x140 and unit 1 at 0x156, a channel using the unit its number
 * differs from.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "ingenic-a1-vc8000d-codec.h"
#include "trace.h"

#define PP_UNITS        2
#define PP_BASE         0x140
#define PP_STRIDE       0x16        /* words between the units */

/* register offsets inside a unit, in words */
#define PP_CTRL         0
#define PP_FMT          1
#define PP_SCALE_V      2
#define PP_SCALE_H      3
#define PP_SCALE_INV    4
#define PP_LUMA         6
#define PP_CHROMA       8
#define PP_PITCH        9
#define PP_CROP_POS     10
#define PP_SRC_SIZE     11
#define PP_OUT_SIZE     12

#define PP_E            BIT(0)
#define PP_CR_FIRST     BIT(1)
#define PP_TILED        BIT(3)

#define SCALE_NONE      0
#define SCALE_UP        1
#define SCALE_DOWN      2

#define MAX_PP_DIM      4096

/* a unit's scaling, one axis */
typedef struct PpAxis {
    unsigned mode;          /* SCALE_* */
    bool cubic;             /* an upscale's horizontal axis */
    unsigned k;             /* whole source samples per output sample */
    unsigned inv;           /* the step's fraction, 16 bits */
    unsigned taps;          /* a downscale's filter length */
    int32_t *coef;          /* 17 phases of stride words, read from guest RAM */
    unsigned stride;
} PpAxis;

/* a plane of samples: d[y * stride + x * step] */
typedef struct PpPlane {
    const uint8_t *d;
    unsigned w, h, stride, step;
} PpPlane;

static uint8_t clamp8(int64_t v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/*
 * Resample n_in samples to n_out, reading in(i) and writing out(x).
 * Upscale: source position x * inv (16.16, inv the register's (in-1) /
 * (out-1)), phase in 32nds; horizontally Catmull-Rom, vertically a
 * blend of two samples by 3f^2 - 2f^3: both read off a bench unit's
 * output, which this matches in 99.7 percent of bytes. Downscale, as
 * measured on a bench A1N: an 8-bit-per-pass separable polyphase filter
 * whose coefficients libimp builds in RAM; output sample x is centred on
 * source position (x + 0.5) * step - 0.5, step being the whole part and
 * the 16-bit fraction the registers hold (in / out when there is no
 * whole part), the fraction of the
 * position in 32nds picks the phase and the table holds 17 of them, used
 * as stored for the upper half of the fraction and reversed for the lower.
 */
static void pp_line(const PpAxis *a, unsigned n_in, unsigned n_out,
                    const uint8_t *in, size_t in_step, uint8_t *out,
                    size_t out_step)
{
    unsigned x, k;

    if (a->mode == SCALE_NONE) {
        for (x = 0; x < n_out; x++) {
            out[x * out_step] = in[MIN(x, n_in - 1) * in_step];
        }
    } else if (a->mode == SCALE_UP) {
        for (x = 0; x < n_out; x++) {
            uint64_t p = (uint64_t)x * a->inv;
            unsigned i = p >> 16;
            double f = ((p >> 11) & 31) / 32.0, v;

            if (a->cubic) {
                /* Catmull-Rom */
                double w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
                double w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
                double w2 = f * (0.5 + f * (2.0 - 1.5 * f));
                double w3 = f * f * (-0.5 + 0.5 * f);

                v = w0 * in[MIN(MAX(i, 1) - 1, n_in - 1) * in_step] +
                    w1 * in[MIN(i, n_in - 1) * in_step] +
                    w2 * in[MIN(i + 1, n_in - 1) * in_step] +
                    w3 * in[MIN(i + 2, n_in - 1) * in_step];
            } else {
                double t = f * f * (3 - 2 * f);

                v = (1 - t) * in[MIN(i, n_in - 1) * in_step] +
                    t * in[MIN(i + 1, n_in - 1) * in_step];
            }
            out[x * out_step] = clamp8(v + 0.5);
        }
    } else {
        unsigned n = a->taps - 1;
        uint64_t step = a->k ? ((uint64_t)a->k << 16) + a->inv :
                        ((uint64_t)n_in << 16) / n_out;

        for (x = 0; x < n_out; x++) {
            int64_t pos = (int64_t)(((2 * (uint64_t)x + 1) * step) >> 1) -
                          32768;
            int64_t i = pos >> 16, acc = 0;
            unsigned q = ((pos & 0xffff) >> 11);
            const int32_t *c = a->coef + (q >= 16 ? q - 16 : 16 - q) *
                               a->stride;

            for (k = 0; k < n; k++) {
                int64_t at = i - (n / 2 - 1) + k;

                at = at < 0 ? 0 : at > (int64_t)n_in - 1 ? n_in - 1 : at;
                acc += (int64_t)(q >= 16 ? c[k] : c[n - 1 - k]) *
                       in[at * in_step];
            }
            out[x * out_step] = clamp8((acc + 32768) >> 16);
        }
    }
}

/* One plane through both axes into dst (ow x oh, dst_stride bytes a row) */
static void pp_plane(const PpAxis *h, const PpAxis *v, const PpPlane *p,
                     unsigned ow, unsigned oh, uint8_t *dst,
                     size_t dst_stride, size_t dst_step)
{
    uint8_t *mid = g_malloc((size_t)ow * p->h);
    uint8_t *col = g_malloc(p->h), *res = g_malloc(oh);
    unsigned x, y;

    for (y = 0; y < p->h; y++) {
        pp_line(h, p->w, ow, p->d + (size_t)y * p->stride, p->step,
                mid + (size_t)y * ow, 1);
    }
    for (x = 0; x < ow; x++) {
        for (y = 0; y < p->h; y++) {
            col[y] = mid[(size_t)y * ow + x];
        }
        pp_line(v, p->h, oh, col, 1, res, 1);
        for (y = 0; y < oh; y++) {
            dst[y * dst_stride + x * dst_step] = res[y];
        }
    }
    g_free(mid);
    g_free(col);
    g_free(res);
}

static bool pp_axis_load(IngenicA1Vc8000dState *s, PpAxis *a, unsigned mode,
                         unsigned k, unsigned inv, unsigned taps,
                         uint32_t base, unsigned skip, bool horizontal)
{
    unsigned i, words;

    a->mode = mode;
    a->cubic = horizontal;
    a->k = k;
    a->inv = inv;
    a->taps = taps;
    a->coef = NULL;
    if (mode != SCALE_DOWN) {
        return true;
    }
    if (taps < 5 || taps > 33 || !(taps & 1)) {
        return false;
    }
    a->stride = 4 * ((taps + 3) / 4);
    words = 17 * a->stride;
    a->coef = g_new(int32_t, words);
    if (address_space_read(s->as, base + skip * 4,
                           MEMTXATTRS_UNSPECIFIED, a->coef,
                           words * 4) != MEMTX_OK) {
        g_free(a->coef);
        a->coef = NULL;
        return false;
    }
    for (i = 0; i < words; i++) {
        a->coef[i] = le32_to_cpu(a->coef[i]);
    }
    return true;
}

static void pp_unit(IngenicA1Vc8000dState *s, unsigned u, const uint8_t *nv12,
                    unsigned w, unsigned h)
{
    const uint32_t *dec = s->dec;
    const uint32_t *r = &dec[PP_BASE + u * PP_STRIDE];
    /* filter registers: tap counts, then the table base two words on */
    const uint32_t *t = &s->dec[u ? 0x1b9 : 0x1b6];
    uint32_t table = t[2];
    unsigned cx = extract32(r[PP_CROP_POS], 16, 13) * 2;
    unsigned cy = extract32(r[PP_CROP_POS], 0, 13) * 2;
    unsigned sw = extract32(r[PP_SRC_SIZE], 16, 16) * 2;
    unsigned sh = extract32(r[PP_SRC_SIZE], 0, 16) * 2;
    unsigned ow = extract32(r[PP_OUT_SIZE], 16, 16);
    unsigned oh = extract32(r[PP_OUT_SIZE], 0, 16);
    unsigned lpitch = extract32(r[PP_PITCH], 16, 16);
    unsigned cpitch = extract32(r[PP_PITCH], 0, 16);
    bool cr_first = r[PP_CTRL] & PP_CR_FIRST;
    PpAxis ah = { 0 }, av = { 0 };
    PpPlane luma, chroma;
    uint8_t *y, *uv;
    unsigned j;

    if (!(r[PP_CTRL] & PP_E)) {
        return;
    }
    if (r[PP_CTRL] & PP_TILED || extract32(r[PP_CTRL], 11, 2)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-a1-vc8000d: PP unit %u output "
                      "format 0x%08x\n", u, r[PP_CTRL]);
        return;
    }
    if (!ow || !oh || ow > MAX_PP_DIM || oh > MAX_PP_DIM) {
        return;
    }
    if (!sw || !sh) {
        sw = MIN(w, ow);
        sh = MIN(h, oh);
    }
    if (cx + sw > w || cy + sh > h) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-a1-vc8000d: PP window "
                      "%ux%u at %u,%u outside the %ux%u picture\n",
                      sw, sh, cx, cy, w, h);
        return;
    }
    if (!pp_axis_load(s, &ah, extract32(r[PP_SCALE_V], 25, 2),
                      extract32(dec[0x1b5], u ? 8 : 24, 8),
                      extract32(r[PP_SCALE_INV], 16, 16),
                      extract32(t[0], 0, 12), table, 0, true) ||
        !pp_axis_load(s, &av, extract32(r[PP_SCALE_V], 23, 2),
                      extract32(dec[0x1b5], u ? 0 : 16, 8),
                      extract32(r[PP_SCALE_INV], 0, 16),
                      extract32(t[0], 12, 12), table,
                      ah.coef ? 17 * ah.stride : 0, false)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-a1-vc8000d: PP unit %u: no "
                      "filter table\n", u);
        g_free(ah.coef);
        g_free(av.coef);
        return;
    }
    luma = (PpPlane){ nv12 + (size_t)cy * w + cx, sw, sh, w, 1 };
    chroma = (PpPlane){ nv12 + (size_t)w * h + (size_t)(cy / 2) * w + cx,
                        sw / 2, sh / 2, w, 2 };
    y = g_malloc0((size_t)oh * MAX(lpitch, ow));
    uv = g_malloc0((size_t)(oh / 2) * MAX(cpitch, ow));
    pp_plane(&ah, &av, &luma, ow, oh, y, lpitch, 1);
    pp_plane(&ah, &av, &chroma, ow / 2, oh / 2, uv + cr_first, cpitch, 2);
    chroma.d++;
    pp_plane(&ah, &av, &chroma, ow / 2, oh / 2, uv + !cr_first, cpitch, 2);
    for (j = 0; j < oh; j++) {
        address_space_write(s->as, r[PP_LUMA] + (size_t)j * lpitch,
                            MEMTXATTRS_UNSPECIFIED, y + (size_t)j * lpitch,
                            ow);
    }
    for (j = 0; j < oh / 2; j++) {
        address_space_write(s->as, r[PP_CHROMA] + (size_t)j * cpitch,
                            MEMTXATTRS_UNSPECIFIED, uv + (size_t)j * cpitch,
                            ow);
    }
    trace_ingenic_a1_vc8000d_pp(u, sw, sh, ow, oh, r[PP_LUMA]);
    g_free(y);
    g_free(uv);
    g_free(ah.coef);
    g_free(av.coef);
}

void vc8000d_pp_run(IngenicA1Vc8000dState *s, const uint8_t *nv12,
                    unsigned w, unsigned h)
{
    unsigned u;

    for (u = 0; u < PP_UNITS; u++) {
        pp_unit(s, u, nv12, w, h);
    }
}
