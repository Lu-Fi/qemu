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
 * T's arithmetic, its order of lines, F's NV12 and Y scaling (down and
 * up, TRANS as libaip sets it to (ratio - 1) / 2 or any other) and P's
 * affine and perspective warps from NV12 are a T41NQ's, byte for byte, on
 * the frames Ingenic's personvehiclepetDet sample sends it, on pictures
 * programmed directly and on libaip's own calls. Not modelled yet: F's
 * formats other than NV12 and Y, T's inputs other than NV12, P's sources
 * other than NV12, T's chain, the ISUM/OSUM checksums, and what CTRL reads
 * back after a job (a T41NQ's T 0x9 then 0x8, F 0x80000008).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
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
#define P_MODE_NV12         2

#define CTRL_RUN            BIT(0)
#define CTRL_RESET          BIT(1)
#define CTRL_CHAIN          BIT(2)
#define T_CTRL_STEP         BIT(4)
#define IQR_DONE            BIT(0)
#define CHAIN_LAST          BIT(31)

#define F_FMT_Y             0
#define F_FMT_NV12          7

static const hwaddr unit_base[INGENIC_AIP_UNITS] = { 0x000, 0x200, 0x300 };

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
    s->regs[unit][REG_IQR / 4] |= IQR_DONE;
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
        phys_write(reg(s, AIP_T, T_DST) + l * dstride, out, width * 4);
    }
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

/*
 * One plane of F's job, bilinear in a T41NQ's integer steps: across first,
 * kept to 1/128, then each row's share truncated before the sum is rounded.
 * Chroma is resampled at the even luma positions, each UV pair standing for
 * a 2x2 block of luma.
 */
static void aip_f_plane(IngenicAipState *s, hwaddr src, hwaddr dst,
                        uint32_t sw, uint32_t sh, uint32_t dw, uint32_t dh,
                        uint32_t sstride, uint32_t dstride, bool chroma)
{
    uint32_t sx = reg(s, AIP_F, F_SCALE_X), sy = reg(s, AIP_F, F_SCALE_Y);
    uint32_t tx = reg(s, AIP_F, F_TRANS_X), ty = reg(s, AIP_F, F_TRANS_Y);
    uint32_t rows = chroma ? sh / 2 : sh, step = chroma ? 2 : 1;
    uint32_t odw = chroma ? dw / 2 * 2 : dw;
    g_autofree uint8_t *in = g_malloc((size_t)sstride * rows);
    g_autofree uint8_t *out = g_malloc(odw);

    phys_read(src, in, (hwaddr)sstride * rows);
    for (uint32_t j = 0; j < (chroma ? dh / 2 : dh); j++) {
        uint32_t y0, y1, wy;

        aip_f_pos(j * step, sy, ty, sh, &y0, &y1, &wy);
        if (chroma) {
            y0 >>= 1;
            y1 >>= 1;
        }
        for (uint32_t o = 0; o < odw; o++) {
            uint32_t x0, x1, wx, c = chroma ? o & 1 : 0;
            uint32_t a, b, d, e, top, bot;

            aip_f_pos(o / step * step, sx, tx, sw, &x0, &x1, &wx);
            if (chroma) {
                x0 = (x0 >> 1) * 2 + c;
                x1 = (x1 >> 1) * 2 + c;
            }
            a = in[y0 * sstride + x0];
            b = in[y0 * sstride + x1];
            d = in[y1 * sstride + x0];
            e = in[y1 * sstride + x1];
            top = (a * (2048 - wx) + b * wx) >> 4;
            bot = (d * (2048 - wx) + e * wx) >> 4;
            out[o] = MIN(((((uint64_t)top * (2048 - wy)) >> 16) +
                          (((uint64_t)bot * wy) >> 16) + 2) >> 2, 255);
        }
        phys_write(dst + j * dstride, out, odw);
    }
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
    if (fmt != F_FMT_NV12 && fmt != F_FMT_Y) {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: F format %u\n", fmt);
        return;
    }
    aip_f_plane(s, reg(s, AIP_F, F_SRC_Y), reg(s, AIP_F, F_DST_Y), sw, sh, dw,
                dh, sstride, dstride, false);
    if (fmt == F_FMT_NV12) {
        aip_f_plane(s, reg(s, AIP_F, F_SRC_UV), reg(s, AIP_F, F_DST_UV),
                    sw, sh, dw, dh, sstride, dstride, true);
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
 * 1/32 pixel. Source pixels are colour converted first and the four around
 * the position blended byte by byte; outside the source box a pixel is
 * DUMMY's luma with zero chroma, and so is an output whose position falls
 * a pixel or more outside it.
 */
static void aip_p_job(IngenicAipState *s)
{
    uint32_t sw = extract32(reg(s, AIP_P, P_SRC_SIZE), 0, 16);
    uint32_t sh = extract32(reg(s, AIP_P, P_SRC_SIZE), 16, 16);
    uint32_t dw = extract32(reg(s, AIP_P, P_DST_SIZE), 0, 16);
    uint32_t dh = extract32(reg(s, AIP_P, P_DST_SIZE), 16, 16);
    uint32_t sstride = reg(s, AIP_P, P_SRC_STRIDE);
    uint32_t dstride = reg(s, AIP_P, P_DST_STRIDE);
    int64_t c[9];
    AipColour col;
    uint8_t dummy[4];
    g_autofree uint8_t *line = NULL;
    g_autofree uint8_t *uv = NULL;
    g_autofree uint8_t *rgb = NULL;
    g_autofree uint8_t *out = NULL;

    if (!sw || !sh || !dw || !dh) {
        return;
    }
    if (reg(s, AIP_P, P_MODE) != P_MODE_NV12) {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: P source mode %u\n",
                      reg(s, AIP_P, P_MODE));
        return;
    }
    for (int r = 0; r < 3; r++) {
        c[3 * r] = aip_p_coef(s, r, 0, 48);
        c[3 * r + 1] = aip_p_coef(s, r, 48, 48);
        c[3 * r + 2] = aip_p_coef(s, r, 96, 59);
    }
    aip_colour_init(s, AIP_P, P_PARAM0, P_OFFSET, &col);
    aip_colour(&col, extract32(reg(s, AIP_P, P_DUMMY), 0, 8), 0, 0, dummy);
    line = g_malloc(sw);
    uv = g_malloc(sw + 1);
    rgb = g_malloc((size_t)sw * sh * 4);
    out = g_malloc((size_t)dw * 4);
    for (uint32_t y = 0; y < sh; y++) {
        phys_read(reg(s, AIP_P, P_SRC_Y) + y * sstride, line, sw);
        phys_read(reg(s, AIP_P, P_SRC_UV) + y / 2 * sstride, uv, sw);
        for (uint32_t x = 0; x < sw; x++) {
            aip_colour(&col, line[x], uv[x & ~1], uv[x | 1],
                       &rgb[((size_t)y * sw + x) * 4]);
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
            const uint8_t *px[4];

            if (!den || (den == -1 && (nx == INT64_MIN || ny == INT64_MIN))) {
                memcpy(&out[i * 4], dummy, 4);
                continue;
            }
            qx = nx / den;
            qy = ny / den;
            x0 = qx >> 5;
            y0 = qy >> 5;
            wx = qx & 31;
            wy = qy & 31;
            if (x0 < -1 || x0 >= sw || y0 < -1 || y0 >= sh) {
                memcpy(&out[i * 4], dummy, 4);
                continue;
            }
            for (int k = 0; k < 4; k++) {
                int64_t x = x0 + (k & 1), y = y0 + (k >> 1);

                px[k] = x >= 0 && x < sw && y >= 0 && y < sh ?
                        &rgb[(y * sw + x) * 4] : dummy;
            }
            for (int b = 0; b < 4; b++) {
                uint32_t top = px[0][b] * (32 - wx) + px[1][b] * wx;
                uint32_t bot = px[2][b] * (32 - wx) + px[3][b] * wx;

                out[i * 4 + b] = (top * (32 - wy) + bot * wy + 512) >> 10;
            }
        }
        phys_write(reg(s, AIP_P, P_DST) + j * dstride, out, (hwaddr)dw * 4);
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
        if (le32_to_cpu(pair[1]) & CHAIN_LAST) {
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
    if (off != REG_CTRL) {
        s->regs[u][off / 4] = value;
        return;
    }
    /* the reset is done as it is asked for: libaip polls the bit clear */
    s->regs[u][off / 4] = value & ~CTRL_RESET;
    if (value & CTRL_RESET) {
        s->regs[u][REG_IQR / 4] = 0;
        aip_update(s, u);
        if (u == AIP_T) {
            s->t_row = 0;
        }
    }
    switch (u) {
    case AIP_T:
        if (value & CTRL_RUN) {
            s->t_row = 0;
        }
        if (value & T_CTRL_STEP) {
            aip_t_step(s);
        }
        break;
    case AIP_F:
        if (value & CTRL_CHAIN) {
            aip_chain(s, AIP_F, F_CHAIN_BASE, aip_f_job);
            aip_done(s, AIP_F);
        } else if (value & CTRL_RUN) {
            aip_f_job(s);
            aip_done(s, AIP_F);
        }
        break;
    default:
        if (value & CTRL_CHAIN) {
            aip_chain(s, AIP_P, P_CHAIN_BASE, aip_p_job);
            aip_done(s, AIP_P);
        } else if (value & CTRL_RUN) {
            aip_p_job(s);
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
