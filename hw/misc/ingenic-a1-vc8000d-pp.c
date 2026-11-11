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

typedef struct PpPlane {
    const uint8_t *d;
    unsigned w, h, stride;
    unsigned step;          /* bytes between samples: 1 luma, 2 chroma */
} PpPlane;

static uint8_t pp_sample(const PpPlane *p, unsigned x, unsigned y)
{
    return p->d[(size_t)MIN(y, p->h - 1) * p->stride +
                (size_t)MIN(x, p->w - 1) * p->step];
}

/*
 * One output sample at column x of a row. The ratios are the 16.16
 * source-per-output steps libimp derives from the sizes; an upscale
 * interpolates the two neighbouring samples, a downscale averages the
 * source interval an output sample covers.
 */
static uint8_t pp_axis(const PpPlane *p, unsigned ox, unsigned oy,
                       unsigned ow, unsigned oh, unsigned mode_h,
                       unsigned mode_v)
{
    uint64_t sx, sy;
    unsigned x0, y0, fx, fy;
    uint32_t a, b;

    if (mode_h == SCALE_NONE && mode_v == SCALE_NONE) {
        return pp_sample(p, ox, oy);
    }
    sx = ow > 1 ? (uint64_t)ox * (p->w - 1) * 65536 / (ow - 1) : 0;
    sy = oh > 1 ? (uint64_t)oy * (p->h - 1) * 65536 / (oh - 1) : 0;
    if (mode_h == SCALE_DOWN) {
        sx = (uint64_t)ox * p->w * 65536 / ow + p->w * 32768 / ow;
        sx = sx >= 32768 ? sx - 32768 : 0;
    }
    if (mode_v == SCALE_DOWN) {
        sy = (uint64_t)oy * p->h * 65536 / oh + p->h * 32768 / oh;
        sy = sy >= 32768 ? sy - 32768 : 0;
    }
    x0 = sx >> 16;
    y0 = sy >> 16;
    fx = sx & 0xffff;
    fy = sy & 0xffff;
    a = (pp_sample(p, x0, y0) * (65536 - fx) +
         pp_sample(p, x0 + 1, y0) * fx) >> 8;
    b = (pp_sample(p, x0, y0 + 1) * (65536 - fx) +
         pp_sample(p, x0 + 1, y0 + 1) * fx) >> 8;
    return (a * (65536 - fy) + b * fy + (1u << 23)) >> 24;
}

static void pp_unit(IngenicA1Vc8000dState *s, unsigned u, const uint8_t *nv12,
                    unsigned w, unsigned h)
{
    const uint32_t *r = &s->dec[PP_BASE + u * PP_STRIDE];
    unsigned cx = extract32(r[PP_CROP_POS], 16, 13) * 2;
    unsigned cy = extract32(r[PP_CROP_POS], 0, 13) * 2;
    unsigned sw = extract32(r[PP_SRC_SIZE], 16, 16) * 2;
    unsigned sh = extract32(r[PP_SRC_SIZE], 0, 16) * 2;
    unsigned ow = extract32(r[PP_OUT_SIZE], 16, 16);
    unsigned oh = extract32(r[PP_OUT_SIZE], 0, 16);
    unsigned mh = extract32(r[PP_SCALE_V], 25, 2);
    unsigned mv = extract32(r[PP_SCALE_V], 23, 2);
    unsigned lpitch = extract32(r[PP_PITCH], 16, 16);
    unsigned cpitch = extract32(r[PP_PITCH], 0, 16);
    bool cr_first = r[PP_CTRL] & PP_CR_FIRST;
    PpPlane luma, chroma;
    uint8_t *y, *uv;
    unsigned i, j;

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
    luma = (PpPlane){ nv12 + (size_t)cy * w + cx, sw, sh, w, 1 };
    chroma = (PpPlane){ nv12 + (size_t)w * h + (size_t)(cy / 2) * w + cx,
                        sw / 2, sh / 2, w, 2 };
    y = g_malloc((size_t)oh * MAX(lpitch, ow));
    uv = g_malloc((size_t)(oh / 2) * MAX(cpitch, ow));
    for (j = 0; j < oh; j++) {
        for (i = 0; i < ow; i++) {
            y[(size_t)j * lpitch + i] = pp_axis(&luma, i, j, ow, oh, mh, mv);
        }
    }
    for (j = 0; j < oh / 2; j++) {
        for (i = 0; i < ow / 2; i++) {
            PpPlane c = chroma;
            uint8_t cb, cr;

            cb = pp_axis(&c, i, j, ow / 2, oh / 2, mh, mv);
            c.d++;
            cr = pp_axis(&c, i, j, ow / 2, oh / 2, mh, mv);
            uv[(size_t)j * cpitch + 2 * i] = cr_first ? cr : cb;
            uv[(size_t)j * cpitch + 2 * i + 1] = cr_first ? cb : cr;
        }
    }
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
}

void vc8000d_pp_run(IngenicA1Vc8000dState *s, const uint8_t *nv12,
                    unsigned w, unsigned h)
{
    unsigned u;

    for (u = 0; u < PP_UNITS; u++) {
        pp_unit(s, u, nv12, w, h);
    }
}
