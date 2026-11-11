/*
 * Ingenic AIP: the T41's image pre-processor
 *
 * Three units at 0x12b00000, each with its own interrupt and 0x100 bytes
 * of registers, CTRL at +0x00 (bit 1 resets the unit and reads back 0) and
 * the interrupt status at +0x04 (write 1 to clear; bit 2 is a timeout):
 *   T (+0x000): colour conversion, NV12 to 4-byte pixels, in steps.
 *     +0x08 CFG (bit 0 source in ORAM, bit 1 destination in ORAM)
 *     +0x10 TASK_LEN, +0x14/+0x18 SRC_Y/SRC_UV, +0x1c DST,
 *     +0x20 STRIDE (destination << 16 | source), +0x24 SRC_SIZE (h << 16 |
 *     w), +0x28..+0x48 PARAM0-8 (Q20, signed: the B, G and R rows on Y, U,
 *     V), +0x4c OFFSET (order code << 24 | alpha << 16 | UV << 8 | Y).
 *     CTRL bit 0 starts over at the first source line; bit 4 converts the
 *     next TASK_LEN lines to DST.
 *   F (+0x200): scaling, +0x210/+0x214 SCALE_X/Y (16.16), +0x218/+0x21c
 *     TRANS_X/Y, +0x220..+0x22c SRC_Y, SRC_UV, DST_Y, DST_UV, +0x230/+0x234
 *     SRC_SIZE/DST_SIZE (h << 16 | w), +0x238 STRIDE (destination << 16 |
 *     source), +0x23c/+0x240 CHAIN_BASE/CHAIN_SIZE; CFG bits 9:7 the
 *     format. CTRL bit 0 runs the registers as they are, bit 2 a chain of
 *     (value, register address) pairs, each job ended by a write with
 *     address bit 31 set.
 *   P (+0x300): affine and perspective warps of NV12 to 4-byte pixels.
 *     +0x310 MODE (2: NV12 source), +0x314/+0x318 SRC_Y/SRC_UV, +0x31c
 *     SRC_STRIDE, +0x320 DST, +0x324 DST_STRIDE, +0x328/+0x32c DST_SIZE/
 *     SRC_SIZE (h << 16 | w), +0x330 DUMMY (the luma outside the source),
 *     +0x334..+0x36c COEF0-14 (the matrix from destination to source, a
 *     row per 160 bits: 48-, 48- and 59-bit signed fields; libaip puts 16
 *     fraction bits in an affine one and 26 in a perspective one, the
 *     third row dividing), +0x370..+0x390 PARAM0-8 and +0x394 OFFSET as
 *     T's, +0x398/+0x39c CHAIN_BASE/CHAIN_SIZE. CTRL as F's.
 *
 * F's CFG bits 9:7 pick its format: 0 Y, 1 UV pairs, 2 4-byte pixels,
 * 3/4/5 feature maps of 32 channels of 2, 4 or 8 bits (8, 16 or 32 bytes
 * a pixel), 6 16 bytes a pixel, 7 NV12; all but NV12 from SRC_Y to DST_Y.
 * P's MODE: 0 Y, 1 UV pairs, 3 4-byte pixels, all from SRC_Y and passed as
 * they are, 2 NV12 to converted pixels. T converts NV12 only.
 *
 * Each unit sums its output into OSUM (T +0x60, F +0x248, P +0x3a4) as
 * little-endian words; F and P sum into ISUM (+0x244, +0x3a0) each 32-byte
 * burst they fetch, once: F the rows it samples, P the columns its
 * positions use on each row. CFG bits clear them at
 * once (T 8/9, F 10/11, P 9/10: not P's 7/8 the vendor header names), and
 * they otherwise add up across jobs. CTRL reads 0x8 (AXI idle) at rest, T
 * 0x9 while a frame is open, F 0x80000008 after a job.
 *
 * T's arithmetic, its order of lines, F's scaling in every format (down and
 * up, TRANS as libaip sets it to (ratio - 1) / 2 or any other) and P's
 * affine and perspective warps in every mode are a T41NQ's, byte for byte,
 * on the frames Ingenic's personvehiclepetDet sample sends it, on pictures
 * programmed directly and on libaip's own calls; so are the sums where the
 * model computes them. Not modelled: T's ISUM, which a T41NQ fills from a
 * prefetch of its own (the frame once at CTRL bit 0, then again ahead of
 * the steps, in chunks that do not follow the frame's size), and F's
 * re-fetches of source rows over 48 bytes wide, which a T41NQ's two-burst
 * cache misses (it counts some bursts twice). T's chain (+0x50/+0x54)
 * loads its registers but, on a T41NQ, converts nothing and stays
 * chain-busy (CTRL 0xc), a soft reset included; the vendor drivers never
 * use it. TASK_SKIP_BASE (+0x58) is not modelled either: the vendor's
 * driver notes its jump does not work.
 *
 * The A1's driver completes a T step on status bit 3 or 1 (F and P: bit 1
 * or 0), so there "t-done" is the status a T step raises, 0x9: the T41's
 * bit 0 and bit 3. Silicon has not been asked.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-aip.h"

#define AIP_T               0
#define AIP_F               1
#define AIP_P               2

#define REG_CTRL            0x00
#define REG_IQR             0x04
#define REG_CFG             0x08

#define T_TASK_LEN          0x10
#define T_SRC_Y             0x14
#define T_SRC_UV            0x18
#define T_DST               0x1c
#define T_STRIDE            0x20
#define T_SRC_SIZE          0x24
#define T_PARAM0            0x28
#define T_OFFSET            0x4c
#define T_CHAIN_BASE        0x50
#define T_ISUM              0x5c
#define T_OSUM              0x60

#define F_SCALE_X           0x10
#define F_SCALE_Y           0x14
#define F_TRANS_X           0x18
#define F_TRANS_Y           0x1c
#define F_SRC_Y             0x20
#define F_SRC_UV            0x24
#define F_DST_Y             0x28
#define F_DST_UV            0x2c
#define F_SRC_SIZE          0x30
#define F_DST_SIZE          0x34
#define F_STRIDE            0x38
#define F_CHAIN_BASE        0x3c
#define F_CHAIN_SIZE        0x40
#define F_ISUM              0x44
#define F_OSUM              0x48

#define P_MODE              0x10
#define P_SRC_Y             0x14
#define P_SRC_UV            0x18
#define P_SRC_STRIDE        0x1c
#define P_DST               0x20
#define P_DST_STRIDE        0x24
#define P_DST_SIZE          0x28
#define P_SRC_SIZE          0x2c
#define P_DUMMY             0x30
#define P_COEF0             0x34
#define P_PARAM0            0x70
#define P_OFFSET            0x94
#define P_CHAIN_BASE        0x98
#define P_ISUM              0xa0
#define P_OSUM              0xa4
#define P_MODE_Y            0
#define P_MODE_C            1
#define P_MODE_NV12         2
#define P_MODE_BGRA         3

#define CTRL_RUN            BIT(0)
#define CTRL_RESET          BIT(1)
#define CTRL_CHAIN          BIT(2)
#define CTRL_AXI_EMPTY      BIT(3)
#define T_CTRL_STEP         BIT(4)
#define F_CTRL_DONE         BIT(31)
#define IQR_DONE            BIT(0)
#define CHAIN_LAST          BIT(31)

#define F_FMT_NV12          7

#define BURST               32

static const hwaddr unit_base[INGENIC_AIP_UNITS] = { 0x000, 0x200, 0x300 };
static const hwaddr isum_reg[INGENIC_AIP_UNITS] = { T_ISUM, F_ISUM, P_ISUM };
static const hwaddr osum_reg[INGENIC_AIP_UNITS] = { T_OSUM, F_OSUM, P_OSUM };
/* CFG's bits that clear ISUM and OSUM (P's are not where the driver says) */
static const unsigned sum_clr_bit[INGENIC_AIP_UNITS] = { 8, 10, 9 };
/* F's formats: bytes a pixel, bits a field */
static const uint8_t f_bpp[8] = { 1, 2, 4, 8, 16, 32, 16, 1 };
static const uint8_t f_bits[8] = { 8, 8, 8, 2, 4, 8, 8, 8 };

static uint32_t reg(IngenicAipState *s, int unit, hwaddr off)
{
    return s->regs[unit][off / 4];
}

static void aip_update(IngenicAipState *s, int unit)
{
    qemu_set_irq(s->irq[unit], s->regs[unit][REG_IQR / 4] != 0);
}

static void aip_done(IngenicAipState *s, int unit)
{
    s->regs[unit][REG_IQR / 4] |= unit == AIP_T ? s->t_done : IQR_DONE;
    aip_update(s, unit);
}

static void phys_read(hwaddr addr, void *buf, hwaddr len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
}

static void phys_write(hwaddr addr, const void *buf, hwaddr len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

/* bytes added as the little-endian words they sit in */
static uint32_t aip_word_sum(hwaddr addr, const uint8_t *buf, size_t n)
{
    uint32_t sum = 0;

    for (size_t i = 0; i < n; i++) {
        sum += (uint32_t)buf[i] << (8 * ((addr + i) & 3));
    }
    return sum;
}

/* a unit's output, written and added to its OSUM */
static void aip_out(IngenicAipState *s, int unit, hwaddr addr,
                    const uint8_t *buf, hwaddr len)
{
    phys_write(addr, buf, len);
    s->regs[unit][osum_reg[unit] / 4] += aip_word_sum(addr, buf, len);
}

/* the bytes [lo, hi) of each source row a job reads; lo >= hi: none */
typedef struct {
    uint32_t lo, hi;
} AipSpan;

static void aip_span(AipSpan *sp, uint32_t lo, uint32_t hi)
{
    if (sp->lo >= sp->hi) {
        sp->lo = lo;
        sp->hi = hi;
    } else {
        sp->lo = MIN(sp->lo, lo);
        sp->hi = MAX(sp->hi, hi);
    }
}

/* ISUM as F and P fetch: each 32-byte burst under the spans, once */
static void aip_isum_rows(IngenicAipState *s, int unit, hwaddr base,
                          uint32_t stride, const AipSpan *span, uint32_t rows)
{
    int64_t last = -1;
    uint8_t burst[BURST];

    for (uint32_t r = 0; r < rows; r++) {
        hwaddr start = base + (hwaddr)r * stride + span[r].lo;
        hwaddr end = base + (hwaddr)r * stride + span[r].hi;

        if (span[r].lo >= span[r].hi) {
            continue;
        }
        for (int64_t b = start / BURST; b <= (end - 1) / BURST; b++) {
            if (b <= last) {
                continue;
            }
            phys_read(b * BURST, burst, BURST);
            s->regs[unit][isum_reg[unit] / 4] +=
                aip_word_sum(b * BURST, burst, BURST);
            last = b;
        }
    }
}

/*
 * The colour each output byte takes, by the order code: libaip's format
 * number shifted right by one (BGRA 0, GBRA 1, RBGA 2, BRGA 3, GRBA 4,
 * RGBA 5, and the same + 8 with alpha first); 0 is B, 1 G, 2 R
 */
static const uint8_t t_order[6][3] = {
    { 0, 1, 2 }, { 1, 0, 2 }, { 2, 0, 1 }, { 0, 2, 1 }, { 1, 2, 0 },
    { 2, 1, 0 },
};

/* T's and P's colour conversion: PARAM0-8 and OFFSET from base */
typedef struct AipColour {
    int64_t p[9];
    int32_t yoff, uvoff;
    uint8_t alpha;
    unsigned order;
} AipColour;

static void aip_colour_init(IngenicAipState *s, int unit, hwaddr param,
                            hwaddr offreg, AipColour *c)
{
    uint32_t offset = reg(s, unit, offreg);

    for (int i = 0; i < 9; i++) {
        c->p[i] = (int32_t)reg(s, unit, param + 4 * i);
    }
    c->yoff = extract32(offset, 0, 8);
    c->uvoff = extract32(offset, 8, 8);
    c->alpha = extract32(offset, 16, 8);
    c->order = extract32(offset, 24, 4);
    if ((c->order & 7) >= ARRAY_SIZE(t_order)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: order code %u\n", c->order);
        c->order = 5;
    }
}

static void aip_colour(const AipColour *c, int y, int u, int v, uint8_t *o)
{
    int64_t yv = y - c->yoff, uv = u - c->uvoff, vv = v - c->uvoff;
    uint8_t bgr[3];

    for (int k = 0; k < 3; k++) {
        int64_t t = c->p[3 * k] * yv + c->p[3 * k + 1] * uv +
                    c->p[3 * k + 2] * vv;

        bgr[k] = MIN(MAX((t + (1 << 19)) >> 20, 0), 255);
    }
    if (c->order & 8) {
        *o++ = c->alpha;
    }
    for (int k = 0; k < 3; k++) {
        *o++ = bgr[t_order[c->order & 7][k]];
    }
    if (!(c->order & 8)) {
        *o = c->alpha;
    }
}

static void aip_t_step(IngenicAipState *s)
{
    uint32_t lines = reg(s, AIP_T, T_TASK_LEN);
    uint32_t width = extract32(reg(s, AIP_T, T_SRC_SIZE), 0, 16);
    uint32_t height = extract32(reg(s, AIP_T, T_SRC_SIZE), 16, 16);
    uint32_t sstride = extract32(reg(s, AIP_T, T_STRIDE), 0, 16);
    uint32_t dstride = extract32(reg(s, AIP_T, T_STRIDE), 16, 16);
    AipColour c;
    g_autofree uint8_t *y = g_malloc(width);
    g_autofree uint8_t *uv = g_malloc(width + 1);
    g_autofree uint8_t *out = g_malloc(width * 4);

    aip_colour_init(s, AIP_T, T_PARAM0, T_OFFSET, &c);
    for (uint32_t l = 0; l < lines && s->t_row < height; l++, s->t_row++) {
        phys_read(reg(s, AIP_T, T_SRC_Y) + s->t_row * sstride, y, width);
        phys_read(reg(s, AIP_T, T_SRC_UV) + s->t_row / 2 * sstride, uv, width);
        for (uint32_t x = 0; x < width; x++) {
            aip_colour(&c, y[x], uv[x & ~1], uv[x | 1], &out[4 * x]);
        }
        aip_out(s, AIP_T, reg(s, AIP_T, T_DST) + l * dstride, out, width * 4);
    }
    s->regs[AIP_T][REG_CTRL / 4] = CTRL_AXI_EMPTY |
                                   (s->t_row < height ? CTRL_RUN : 0);
    aip_done(s, AIP_T);
}

/*
 * Where output pixel i samples on one axis: the 16.16 position i * SCALE +
 * TRANS (signed), both neighbours clamped into the picture, and an 11-bit
 * weight.
 */
static void aip_f_pos(uint32_t i, uint32_t scale, uint32_t trans, uint32_t n,
                      uint32_t *p0, uint32_t *p1, uint32_t *w)
{
    int64_t f = (int64_t)i * scale + (int32_t)trans;
    int64_t p = f >> 16;

    *p0 = MIN(MAX(p, 0), n - 1);
    *p1 = MIN(MAX(p + 1, 0), n - 1);
    *w = (f & 0xffff) >> 5;
}

/* a, b, d, e blended by 11-bit weights, as F rounds, kept under max */
static uint32_t aip_f_blend(uint32_t a, uint32_t b, uint32_t d, uint32_t e,
                            uint32_t wx, uint32_t wy, uint32_t max)
{
    uint32_t top = (a * (2048 - wx) + b * wx) >> 4;
    uint32_t bot = (d * (2048 - wx) + e * wx) >> 4;

    return MIN(((((uint64_t)top * (2048 - wy)) >> 16) +
                (((uint64_t)bot * wy) >> 16) + 2) >> 2, max);
}

/*
 * One plane of F's job, bilinear in a T41NQ's integer steps: across first,
 * kept to 1/128, then each row's share truncated before the sum is rounded.
 * A pixel's bytes blend apart, and so do the 2- and 4-bit fields of the
 * feature formats (LSB first). Chroma is resampled at the even luma
 * positions, each UV pair standing for a 2x2 block of luma; an odd size
 * keeps its last half pair and line.
 */
static void aip_f_plane(IngenicAipState *s, hwaddr src, hwaddr dst,
                        uint32_t sw, uint32_t sh, uint32_t dw, uint32_t dh,
                        uint32_t sstride, uint32_t dstride, unsigned bpp,
                        unsigned bits, bool chroma)
{
    uint32_t sx = reg(s, AIP_F, F_SCALE_X), sy = reg(s, AIP_F, F_SCALE_Y);
    uint32_t tx = reg(s, AIP_F, F_TRANS_X), ty = reg(s, AIP_F, F_TRANS_Y);
    uint32_t rows = chroma ? DIV_ROUND_UP(sh, 2) : sh, step = chroma ? 2 : 1;
    uint32_t odw = chroma ? dw : dw * bpp, mask = (1u << bits) - 1;
    g_autofree uint8_t *in = g_malloc0((size_t)sstride * rows + 2);
    g_autofree uint8_t *out = g_malloc(odw);
    g_autofree AipSpan *span = g_new0(AipSpan, rows);

    phys_read(src, in, (hwaddr)sstride * rows);
    for (uint32_t j = 0; j < (chroma ? DIV_ROUND_UP(dh, 2) : dh); j++) {
        uint32_t y0, y1, wy;

        aip_f_pos(j * step, sy, ty, sh, &y0, &y1, &wy);
        if (chroma) {
            y0 >>= 1;
            y1 >>= 1;
        }
        aip_span(&span[y0], 0, chroma ? sw : sw * bpp);
        aip_span(&span[y1], 0, chroma ? sw : sw * bpp);
        for (uint32_t o = 0; o < odw; o++) {
            uint32_t x0, x1, wx, k = chroma ? o & 1 : o % bpp;
            uint32_t a, b, d, e, v = 0;

            aip_f_pos(chroma ? o / 2 * 2 : o / bpp, sx, tx, sw, &x0, &x1, &wx);
            if (chroma) {
                x0 = (x0 >> 1) * 2;
                x1 = (x1 >> 1) * 2;
            } else {
                x0 *= bpp;
                x1 *= bpp;
            }
            a = in[y0 * sstride + x0 + k];
            b = in[y0 * sstride + x1 + k];
            d = in[y1 * sstride + x0 + k];
            e = in[y1 * sstride + x1 + k];
            for (unsigned f = 0; f < 8; f += bits) {
                v |= aip_f_blend(a >> f & mask, b >> f & mask, d >> f & mask,
                                 e >> f & mask, wx, wy, mask) << f;
            }
            out[o] = v;
        }
        aip_out(s, AIP_F, dst + j * dstride, out, odw);
    }
    aip_isum_rows(s, AIP_F, src, sstride, span, rows);
}

static void aip_f_job(IngenicAipState *s)
{
    unsigned fmt = extract32(reg(s, AIP_F, REG_CFG), 7, 3);
    uint32_t sw = extract32(reg(s, AIP_F, F_SRC_SIZE), 0, 16);
    uint32_t sh = extract32(reg(s, AIP_F, F_SRC_SIZE), 16, 16);
    uint32_t dw = extract32(reg(s, AIP_F, F_DST_SIZE), 0, 16);
    uint32_t dh = extract32(reg(s, AIP_F, F_DST_SIZE), 16, 16);
    uint32_t sstride = extract32(reg(s, AIP_F, F_STRIDE), 0, 16);
    uint32_t dstride = extract32(reg(s, AIP_F, F_STRIDE), 16, 16);

    if (!sw || !sh || !dw || !dh) {
        return;
    }
    aip_f_plane(s, reg(s, AIP_F, F_SRC_Y), reg(s, AIP_F, F_DST_Y), sw, sh, dw,
                dh, sstride, dstride, f_bpp[fmt], f_bits[fmt], false);
    if (fmt == F_FMT_NV12) {
        aip_f_plane(s, reg(s, AIP_F, F_SRC_UV), reg(s, AIP_F, F_DST_UV),
                    sw, sh, dw, dh, sstride, dstride, 1, 8, true);
    }
}

/* One signed coefficient of P's matrix: row r's field of n bits at bit lo */
static int64_t aip_p_coef(IngenicAipState *s, int r, int lo, int n)
{
    uint64_t v = 0;

    for (int b = 0; b < n; b++) {
        int bit = 160 * r + lo + b;

        v |= (uint64_t)extract32(reg(s, AIP_P, P_COEF0 + bit / 32 * 4),
                                 bit % 32, 1) << b;
    }
    return sextract64(v, 0, n);
}

/*
 * P's warp, as a T41NQ computes it: each output pixel's source position is
 * the matrix's rows (x, y, divisor) at (i, j), truncated toward zero to
 * 1/32 pixel. An NV12 source's pixels are colour converted first; Y, UV
 * pairs and 4-byte pixels pass as they are. The four pixels around the
 * position blend byte by byte; outside the source box a pixel is DUMMY's
 * luma (converted, with zero chroma, from NV12; else DUMMY's low byte and
 * zeros), and so is an output whose position falls a pixel or more
 * outside it. ISUM takes the bursts under the columns the positions use,
 * row by row.
 */
static void aip_p_job(IngenicAipState *s)
{
    uint32_t mode = reg(s, AIP_P, P_MODE);
    uint32_t sw = extract32(reg(s, AIP_P, P_SRC_SIZE), 0, 16);
    uint32_t sh = extract32(reg(s, AIP_P, P_SRC_SIZE), 16, 16);
    uint32_t dw = extract32(reg(s, AIP_P, P_DST_SIZE), 0, 16);
    uint32_t dh = extract32(reg(s, AIP_P, P_DST_SIZE), 16, 16);
    uint32_t sstride = reg(s, AIP_P, P_SRC_STRIDE);
    uint32_t dstride = reg(s, AIP_P, P_DST_STRIDE);
    unsigned bpp = mode == P_MODE_Y ? 1 : mode == P_MODE_C ? 2 : 4;
    int64_t c[9];
    uint8_t dummy[4] = { extract32(reg(s, AIP_P, P_DUMMY), 0, 8) };
    g_autofree uint8_t *line = NULL;
    g_autofree uint8_t *uv = NULL;
    g_autofree uint8_t *px = NULL;
    g_autofree uint8_t *out = NULL;
    g_autofree AipSpan *span = NULL;
    g_autofree AipSpan *span_uv = NULL;

    if (!sw || !sh || !dw || !dh) {
        return;
    }
    if (mode > P_MODE_BGRA) {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: P mode %u\n", mode);
        return;
    }
    for (int r = 0; r < 3; r++) {
        c[3 * r] = aip_p_coef(s, r, 0, 48);
        c[3 * r + 1] = aip_p_coef(s, r, 48, 48);
        c[3 * r + 2] = aip_p_coef(s, r, 96, 59);
    }
    line = g_malloc((size_t)sw * bpp);
    uv = g_malloc(sw + 1);
    px = g_malloc((size_t)sw * sh * bpp);
    out = g_malloc((size_t)dw * bpp);
    span = g_new0(AipSpan, sh);
    span_uv = g_new0(AipSpan, DIV_ROUND_UP(sh, 2));
    if (mode == P_MODE_NV12) {
        AipColour col;

        aip_colour_init(s, AIP_P, P_PARAM0, P_OFFSET, &col);
        aip_colour(&col, dummy[0], 0, 0, dummy);
        for (uint32_t y = 0; y < sh; y++) {
            phys_read(reg(s, AIP_P, P_SRC_Y) + y * sstride, line, sw);
            phys_read(reg(s, AIP_P, P_SRC_UV) + y / 2 * sstride, uv, sw);
            for (uint32_t x = 0; x < sw; x++) {
                aip_colour(&col, line[x], uv[x & ~1], uv[x | 1],
                           &px[((size_t)y * sw + x) * 4]);
            }
        }
    } else {
        for (uint32_t y = 0; y < sh; y++) {
            phys_read(reg(s, AIP_P, P_SRC_Y) + y * sstride,
                      &px[(size_t)y * sw * bpp], sw * bpp);
        }
    }
    for (uint32_t j = 0; j < dh; j++) {
        for (uint32_t i = 0; i < dw; i++) {
            /* wrapping, as nothing sane comes near 64 bits */
            int64_t den = (uint64_t)c[6] * i + (uint64_t)c[7] * j + c[8];
            int64_t nx = ((uint64_t)c[0] * i + (uint64_t)c[1] * j + c[2]) << 5;
            int64_t ny = ((uint64_t)c[3] * i + (uint64_t)c[4] * j + c[5]) << 5;
            int64_t qx, qy, x0, y0;
            uint32_t wx, wy;
            const uint8_t *p4[4];

            if (!den || (den == -1 && (nx == INT64_MIN || ny == INT64_MIN))) {
                memcpy(&out[i * bpp], dummy, bpp);
                continue;
            }
            qx = nx / den;
            qy = ny / den;
            x0 = qx >> 5;
            y0 = qy >> 5;
            wx = qx & 31;
            wy = qy & 31;
            /* the 2x2 pixels' columns, clamped, on their rows */
            for (int64_t y = y0; y <= y0 + 1; y++) {
                int64_t lo = MAX(x0, 0), hi = MIN(x0 + 1, (int64_t)sw - 1);

                if (y >= 0 && y < sh && lo <= hi) {
                    unsigned b = mode == P_MODE_NV12 ? 1 : bpp;

                    aip_span(&span[y], lo * b, (hi + 1) * b);
                    aip_span(&span_uv[y / 2], lo & ~1,
                             MIN(hi | 1, (int64_t)sw - 1) + 1);
                }
            }
            if (x0 < -1 || x0 >= sw || y0 < -1 || y0 >= sh) {
                memcpy(&out[i * bpp], dummy, bpp);
                continue;
            }
            for (int k = 0; k < 4; k++) {
                int64_t x = x0 + (k & 1), y = y0 + (k >> 1);

                p4[k] = x >= 0 && x < sw && y >= 0 && y < sh ?
                        &px[(y * sw + x) * bpp] : dummy;
            }
            for (unsigned b = 0; b < bpp; b++) {
                uint32_t top = p4[0][b] * (32 - wx) + p4[1][b] * wx;
                uint32_t bot = p4[2][b] * (32 - wx) + p4[3][b] * wx;

                out[i * bpp + b] = (top * (32 - wy) + bot * wy + 512) >> 10;
            }
        }
        aip_out(s, AIP_P, reg(s, AIP_P, P_DST) + j * dstride, out,
                (hwaddr)dw * bpp);
    }
    aip_isum_rows(s, AIP_P, reg(s, AIP_P, P_SRC_Y), sstride, span, sh);
    if (mode == P_MODE_NV12) {
        aip_isum_rows(s, AIP_P, reg(s, AIP_P, P_SRC_UV), sstride, span_uv,
                      DIV_ROUND_UP(sh, 2));
    }
}

/*
 * F's and P's chains: (value, register address) pairs; address bit 31 ends
 * a job
 */
static void aip_chain(IngenicAipState *s, int unit, hwaddr base_reg,
                      void (*job)(IngenicAipState *s))
{
    uint32_t base = reg(s, unit, base_reg);
    uint32_t size = reg(s, unit, base_reg + 4);

    for (uint32_t o = 0; o + 8 <= size; o += 8) {
        uint32_t pair[2];
        hwaddr off;

        phys_read(base + o, pair, sizeof(pair));
        off = extract32(le32_to_cpu(pair[1]), 0, 12) - unit_base[unit];
        if (off < INGENIC_AIP_UNIT_REGS * 4 && off != REG_CTRL) {
            s->regs[unit][off / 4] = le32_to_cpu(pair[0]);
        }
        if ((le32_to_cpu(pair[1]) & CHAIN_LAST) && job) {
            job(s);
        }
    }
}

static uint64_t aip_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicAipState *s = opaque;

    for (int u = INGENIC_AIP_UNITS - 1; u >= 0; u--) {
        if (offset >= unit_base[u]) {
            hwaddr off = offset - unit_base[u];

            return off < INGENIC_AIP_UNIT_REGS * 4 ? s->regs[u][off / 4] : 0;
        }
    }
    return 0;
}

static void aip_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    IngenicAipState *s = opaque;
    int u = INGENIC_AIP_UNITS - 1;
    hwaddr off;

    while (offset < unit_base[u]) {
        u--;
    }
    off = offset - unit_base[u];
    if (off >= INGENIC_AIP_UNIT_REGS * 4) {
        return;
    }
    if (off == REG_IQR) {
        s->regs[u][REG_IQR / 4] &= ~value;
        aip_update(s, u);
        return;
    }
    if (off == REG_CFG) {
        /* the sums' clear bits act at once and read back 0 */
        if (value & BIT(sum_clr_bit[u])) {
            s->regs[u][isum_reg[u] / 4] = 0;
        }
        if (value & BIT(sum_clr_bit[u] + 1)) {
            s->regs[u][osum_reg[u] / 4] = 0;
        }
        s->regs[u][off / 4] = value & ~(3u << sum_clr_bit[u]) &
                              (u == AIP_P ? 0x7f : ~0u);
        return;
    }
    if (off != REG_CTRL) {
        s->regs[u][off / 4] = value;
        return;
    }
    if (value & CTRL_RESET) {
        /*
         * the reset is done as it is asked for (libaip polls the bit
         * clear); a T41NQ's stuck T chain outlives it
         */
        uint32_t keep = u == AIP_T ? reg(s, u, REG_CTRL) & CTRL_CHAIN : 0;

        s->regs[u][REG_CTRL / 4] = CTRL_AXI_EMPTY | keep;
        s->regs[u][REG_IQR / 4] = 0;
        aip_update(s, u);
        if (u == AIP_T) {
            s->t_row = 0;
        }
        return;
    }
    switch (u) {
    case AIP_T:
        if (value & CTRL_CHAIN) {
            /* a T41NQ's T chain loads its registers and converts nothing */
            aip_chain(s, AIP_T, T_CHAIN_BASE, NULL);
            s->regs[u][REG_CTRL / 4] = CTRL_CHAIN | CTRL_AXI_EMPTY;
            break;
        }
        if (value & CTRL_RUN) {
            s->t_row = 0;
            s->regs[u][REG_CTRL / 4] = CTRL_RUN | CTRL_AXI_EMPTY;
        }
        if (value & T_CTRL_STEP) {
            aip_t_step(s);
        }
        break;
    case AIP_F:
        if (value & (CTRL_CHAIN | CTRL_RUN)) {
            if (value & CTRL_CHAIN) {
                aip_chain(s, AIP_F, F_CHAIN_BASE, aip_f_job);
            } else {
                aip_f_job(s);
            }
            s->regs[u][REG_CTRL / 4] = F_CTRL_DONE | CTRL_AXI_EMPTY;
            aip_done(s, AIP_F);
        }
        break;
    default:
        if (value & (CTRL_CHAIN | CTRL_RUN)) {
            if (value & CTRL_CHAIN) {
                aip_chain(s, AIP_P, P_CHAIN_BASE, aip_p_job);
            } else {
                aip_p_job(s);
            }
            s->regs[u][REG_CTRL / 4] = CTRL_AXI_EMPTY;
            aip_done(s, AIP_P);
        }
        break;
    }
}

static const MemoryRegionOps aip_ops = {
    .read = aip_read,
    .write = aip_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void aip_reset_hold(Object *obj, ResetType type)
{
    IngenicAipState *s = INGENIC_AIP(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->t_row = 0;
    for (int u = 0; u < INGENIC_AIP_UNITS; u++) {
        s->regs[u][REG_CTRL / 4] = CTRL_AXI_EMPTY;
        aip_update(s, u);
    }
}

static void aip_init(Object *obj)
{
    IngenicAipState *s = INGENIC_AIP(obj);

    memory_region_init_io(&s->iomem, obj, &aip_ops, s, "ingenic-aip",
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    for (int u = 0; u < INGENIC_AIP_UNITS; u++) {
        sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[u]);
    }
}

static const Property aip_props[] = {
    DEFINE_PROP_UINT32("t-done", IngenicAipState, t_done, IQR_DONE),
};

static const VMStateDescription vmstate_aip = {
    .name = "ingenic-aip",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(regs, IngenicAipState, INGENIC_AIP_UNITS,
                               INGENIC_AIP_UNIT_REGS),
        VMSTATE_UINT32(t_row, IngenicAipState),
        VMSTATE_END_OF_LIST()
    }
};

static void aip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_aip;
    device_class_set_props(dc, aip_props);
    rc->phases.hold = aip_reset_hold;
}

static const TypeInfo aip_type_info = {
    .name = TYPE_INGENIC_AIP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAipState),
    .instance_init = aip_init,
    .class_init = aip_class_init,
};

static void aip_register_types(void)
{
    type_register_static(&aip_type_info);
}

type_init(aip_register_types)
