/*
 * Ingenic BSCALER: the T40's block scaler
 *
 * One block at 0x13090000 with the LZMA decompressor: while the LZMA's
 * CTRL bit 31 is set, offsets 0x00-0x1f are the LZMA's (ingenic-lzma);
 * otherwise they are FRMC's. Two units, each with its interrupt, share
 * the colour parameters: 0xc0-0xe0 a 3x3 matrix of unsigned 12.20
 * coefficients from Y, U, V to B, G, R (G's U and V terms subtract), and
 * 0xe4 the UV offset (bits 31:24), the Y offset (23:16), the alpha byte
 * (15:8), FRMC's byte order (7:4) and FRMT's (3:0).
 *
 * FRMC (INTC 39) warps tiles. Its registers, loaded directly or from a
 * chain of (value, offset) pairs whose offset bit 31 ends a job: 0x04
 * MODE, 0x08 SRC (the Y plane, or 4-byte pixels), 0x0c the source's UV
 * plane, 0x10 SRC_SIZE (h << 16 | w), 0x14 SRC_STRIDE, 0x18 DST (written
 * once per plane: Y, then UV), 0x1c DST_SIZE, 0x20 DST_STRIDE, 0x40/0x44
 * the source and destination tiles' origins (y << 16 | x) in their
 * pictures, 0x48-0x68 a 3x3 matrix in 16.16 from destination to source.
 * 0x00 bit 0 runs a job (a chain ends each with 0x201 there); 0x30 bit 0
 * runs the chain at 0x34 (0x38 bytes) with bit 5, bit 16 resets; 0x3c is
 * a timeout. A finished run sets 0x00 bit 8 and 0x30 bit 4, which raise
 * the interrupt until cleared. Its soft reset clears only the origins and
 * the matrix (0x68 to 1); FRMT's only its frame.
 *
 * FRMT (INTC 40) converts a frame a few lines at a time: 0x88/0x8c the
 * source's Y and UV planes (or 4-byte pixels at 0x88), 0x90 its size,
 * 0x94 its stride, 0x98/0x9c the step's destination planes, 0xa4 their
 * stride, 0xac the formats (bit 0: a 4-byte source; bits 3:2 the
 * destination: 0 NV12, 1 BGR in the order, 2 YUV). 0x80 bit 0 opens a
 * frame; 0x84 = lines << 16 | 1 converts the next lines to 0x98 on, then
 * sets 0x84 bit 2, which raises the interrupt until cleared; 0x80 bit 0
 * clears after the frame's last line. 0xb0 bit 16 resets FRMT, and bit
 * 17 reads set from a frame's opening to its first step.
 *
 * Both units' pixels are a T40XP's byte for byte:
 *   - NV12 to BGR: (coef * (Y - y offset) ... + 2^19) >> 20, clamped,
 *     with each pixel's own chroma pair; the byte order's bits 2:0 pick
 *     one of B-G-R, G-B-R, R-B-G, B-R-G, G-R-B, R-G-B (6 and 7: zeros),
 *     bit 3 puts alpha first.
 *   - 4-byte pixels pass through with byte 3 (byte 0 under order bit 3)
 *     replaced by the alpha byte; FRMT's YUV output is Y, U, V, alpha,
 *     with byte 0 the alpha too under order bit 3.
 *   - FRMC's source position of each destination pixel is the matrix at
 *     (pixel + destination origin), less the source origin; the samples
 *     blend across with 16-bit weights, (a * (65535 - w) + b * w + 32768)
 *     >> 16, then down the same way. A neighbour outside the source tile
 *     is 0 (Y: the Y offset, UV: the UV offset). Chroma is the UV plane
 *     with each pair spread over its 2x2 pixels, sampled for a pair of
 *     NV12 output at its odd pixel (2i + 1, 2j + 1); an NV12 source is
 *     converted to BGR before it is warped. MODE bit 0 is a 4-byte source,
 *     bit 2 samples (clear: the pad everywhere), bit 3 is a 4-byte BGR
 *     destination, else bit 1 is NV12 and clear packed Y, U, V, alpha.
 *   - Odd widths and heights lose their last column and line in FRMT.
 *
 * Not modelled: FRMC MODE bits 5:4 and 18:16 (other filters), the tile
 * shapes a T40XP declines to write, the port at 0x28/0x2c (16 words
 * libvenus loads, with no effect seen), and the modes that stall a
 * T40XP's bus (FRMC MODE bit 1 without bit 2, FRMT destination format 3,
 * a 4-byte source to NV12 in FRMT). The perspective division is a
 * T40XP's, but with a perspective term a T40XP writes some pixels 0, the
 * same ones run after run: column 0 of the first 12 lines of a tile over
 * 12 lines high, and with a y term most of line 1 and a few more; that
 * reads as its divider's pipeline falling behind, and is left out.
 * 0x80 bits 10/11 move FRMT's source and destination to the ORAM port,
 * where libvenus passes ORAM addresses; they are plain addresses here.
 * 0x80 bits 4:1, which vary from run to run on a T40XP while a frame is
 * open, read 0; so do 0xf0-0xfc, beside the version at 0xec. With MODE
 * bit 2 clear and a 4-byte source, a T40XP writes only the first line.
 * FRMT has a chain base and size at 0xb4/0xb8 that libvenus never uses.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-bscaler.h"

#define FRMC_STAT           0x00
#define FRMC_MODE           0x04
#define FRMC_SRC            0x08
#define FRMC_SRC_UV         0x0c
#define FRMC_SRC_SIZE       0x10
#define FRMC_SRC_STRIDE     0x14
#define FRMC_DST            0x18
#define FRMC_DST_SIZE       0x1c
#define FRMC_DST_STRIDE     0x20
#define FRMC_PORT           0x28
#define FRMC_CTRL           0x30
#define FRMC_CHAIN_BASE     0x34
#define FRMC_CHAIN_SIZE     0x38
#define FRMC_TIMEOUT        0x3c
#define FRMC_SRC_ORIGIN     0x40
#define FRMC_DST_ORIGIN     0x44
#define FRMC_MATRIX         0x48
#define FRMC_MATRIX_LAST    0x68
#define FRMT_STAT           0x80
#define FRMT_STEP           0x84
#define FRMT_SRC            0x88
#define FRMT_SRC_UV         0x8c
#define FRMT_SIZE           0x90
#define FRMT_SRC_STRIDE     0x94
#define FRMT_DST            0x98
#define FRMT_DST_UV         0x9c
#define FRMT_DST_STRIDE     0xa4
#define FRMT_A8             0xa8
#define FRMT_FORMAT         0xac
#define FRMT_CTRL           0xb0
#define FRMT_CHAIN_SIZE     0xb8
#define FRMT_TIMEOUT        0xbc
#define BS_COEF             0xc0
#define BS_PARAM            0xe4

#define LZMA_MODE           BIT(31)
#define RUN                 BIT(0)
#define CHAIN               BIT(5)
#define SOFT_RESET          BIT(16)
#define FRMT_OPENED         BIT(17)
#define STAT_DONE           BIT(8)
#define CTRL_DONE           BIT(4)
#define STEP_DONE           BIT(2)
#define CHAIN_LAST          BIT(31)

#define MODE_SRC4           BIT(0)
#define MODE_NV12           BIT(1)
#define MODE_SAMPLE         BIT(2)
#define MODE_DST4           BIT(3)
#define MODE_UNIMP          0x70030
#define FMT_SRC4            BIT(0)
#define FMT_DST_NV12        0
#define FMT_DST_BGR         1
#define FMT_DST_YUV         2
#define ORDER_ALPHA_FIRST   BIT(3)

#define MAX_SIDE            4096
#define MAX_SOURCE          (64 * MiB)
#define BS_VERSION          0xec
#define BS_VERSION_VALUE    0x20200826

/* a T40XP's writable bits, where not all 32 */
static uint32_t bs_mask(hwaddr off)
{
    switch (off) {
    case FRMC_MODE:
        return 0x7ff7f;
    case FRMC_SRC_STRIDE:
    case FRMC_DST_STRIDE:
    case FRMT_SRC_STRIDE:
        return 0xffff;
    case FRMC_DST_SIZE:
        return 0x01ff01ff;
    case FRMC_PORT:
        return 0x00ff00ff;
    case FRMC_CHAIN_SIZE:
    case FRMT_CHAIN_SIZE:
        return ~7u;
    case FRMT_SIZE:
        return 0xfffefffe;
    case FRMT_FORMAT:
        return 0x777703fd;
    case BS_VERSION ... 0xfc:
        return 0;
    default:
        return ~0u;
    }
}

static void bs_reset_regs(IngenicBscalerState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[FRMC_TIMEOUT / 4] = ~0u;
    s->regs[FRMC_MATRIX_LAST / 4] = 1;
    s->regs[FRMT_A8 / 4] = ~0u;
    s->regs[FRMT_TIMEOUT / 4] = ~0u;
    s->regs[BS_VERSION / 4] = BS_VERSION_VALUE;
}

static uint32_t reg(IngenicBscalerState *s, hwaddr off)
{
    return s->regs[off / 4];
}

static bool lzma_mode(IngenicBscalerState *s)
{
    return s->lzma->regs[0] & LZMA_MODE;
}

static void bscaler_update(IngenicBscalerState *s)
{
    qemu_set_irq(s->irq[0], (reg(s, FRMC_STAT) & STAT_DONE) ||
                            (reg(s, FRMC_CTRL) & CTRL_DONE));
    qemu_set_irq(s->irq[1], reg(s, FRMT_STEP) & STEP_DONE);
}

static uint8_t clamp_u8(int64_t v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* Y, U, V to 4 bytes in the byte order, with the alpha byte */
static void bs_convert(IngenicBscalerState *s, unsigned order,
                       int y, int u, int v, uint8_t *px)
{
    static const uint8_t perm[6][3] = {
        { 0, 1, 2 }, { 1, 0, 2 }, { 2, 0, 1 },
        { 0, 2, 1 }, { 1, 2, 0 }, { 2, 1, 0 },
    };
    uint32_t param = reg(s, BS_PARAM);
    int64_t dy = y - (int)extract32(param, 16, 8);
    int64_t du = u - (int)extract32(param, 24, 8);
    int64_t dv = v - (int)extract32(param, 24, 8);
    const uint32_t *c = &s->regs[BS_COEF / 4];
    int64_t acc[3] = {
        c[0] * dy + c[1] * du + c[2] * dv,
        c[3] * dy - c[4] * du - c[5] * dv,
        c[6] * dy + c[7] * du + c[8] * dv,
    };
    uint8_t *col = px + (order & ORDER_ALPHA_FIRST ? 1 : 0);

    for (int k = 0; k < 3; k++) {
        col[k] = (order & 7) < 6 ?
                 clamp_u8((acc[perm[order & 7][k]] + (1 << 19)) >> 20) : 0;
    }
    px[order & ORDER_ALPHA_FIRST ? 0 : 3] = extract32(param, 8, 8);
}

/* a 4-byte pixel passing through: the alpha byte replaces one end */
static void bs_alpha(IngenicBscalerState *s, unsigned order, uint8_t *px)
{
    px[order & ORDER_ALPHA_FIRST ? 0 : 3] =
        extract32(reg(s, BS_PARAM), 8, 8);
}

/* packed Y, U, V, alpha; alpha first takes Y's byte, not a shift */
static void bs_yuv(IngenicBscalerState *s, unsigned order,
                   int y, int u, int v, uint8_t *px)
{
    px[0] = y;
    px[1] = u;
    px[2] = v;
    px[3] = extract32(reg(s, BS_PARAM), 8, 8);
    if (order & ORDER_ALPHA_FIRST) {
        px[0] = px[3];
    }
}

static void frmt_line(IngenicBscalerState *s, uint32_t line, uint32_t i)
{
    uint32_t w = extract32(reg(s, FRMT_SIZE), 0, 16) & ~1;
    uint32_t sstride = extract32(reg(s, FRMT_SRC_STRIDE), 0, 16);
    uint32_t dstride = reg(s, FRMT_DST_STRIDE);
    uint32_t fmt = reg(s, FRMT_FORMAT);
    unsigned order = extract32(reg(s, BS_PARAM), 0, 4);
    unsigned dst = extract32(fmt, 2, 2);
    hwaddr src = reg(s, FRMT_SRC) + (hwaddr)line * sstride;
    hwaddr out = reg(s, FRMT_DST) + (hwaddr)i * dstride;
    g_autofree uint8_t *y = g_malloc(w * 4);
    g_autofree uint8_t *uv = g_malloc(w);
    g_autofree uint8_t *row = g_malloc(w * 4);

    if (fmt & FMT_SRC4) {
        address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                           row, w * 4);
        for (uint32_t x = 0; x < w; x++) {
            bs_alpha(s, order, &row[x * 4]);
        }
        address_space_write(&address_space_memory, out,
                            MEMTXATTRS_UNSPECIFIED, row, w * 4);
        return;
    }
    address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                       y, w);
    address_space_read(&address_space_memory,
                       reg(s, FRMT_SRC_UV) + (hwaddr)(line / 2) * sstride,
                       MEMTXATTRS_UNSPECIFIED, uv, w);
    if (dst == FMT_DST_NV12) {
        /* the UV plane goes to 0x9c on, from each step's first pair */
        address_space_write(&address_space_memory, out,
                            MEMTXATTRS_UNSPECIFIED, y, w);
        if (!(line & 1)) {
            address_space_write(&address_space_memory,
                                reg(s, FRMT_DST_UV) +
                                (hwaddr)(i / 2) * dstride,
                                MEMTXATTRS_UNSPECIFIED, uv, w);
        }
        return;
    }
    for (uint32_t x = 0; x < w; x++) {
        uint8_t *px = &row[x * 4];

        if (dst == FMT_DST_YUV) {
            bs_yuv(s, order, y[x], uv[x & ~1], uv[x | 1], px);
        } else {
            bs_convert(s, order, y[x], uv[x & ~1], uv[x | 1], px);
        }
    }
    address_space_write(&address_space_memory, out, MEMTXATTRS_UNSPECIFIED,
                        row, w * 4);
}

static void frmt_step(IngenicBscalerState *s, uint32_t v)
{
    uint32_t h = extract32(reg(s, FRMT_SIZE), 16, 16) & ~1;
    uint32_t w = extract32(reg(s, FRMT_SIZE), 0, 16);
    uint32_t fmt = reg(s, FRMT_FORMAT), n = v >> 16;

    s->regs[FRMT_STEP / 4] = v & ~RUN;
    if (!(reg(s, FRMT_STAT) & RUN)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ingenic-bscaler: FRMT step without a frame\n");
        return;
    }
    if (extract32(fmt, 2, 2) == 3 ||
        ((fmt & FMT_SRC4) && extract32(fmt, 2, 2) == FMT_DST_NV12)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-bscaler: FRMT format 0x%x "
                      "stalls a T40XP\n", fmt);
        return;
    }
    s->regs[FRMT_CTRL / 4] &= ~FRMT_OPENED;
    if (w && w <= MAX_SIDE) {
        for (uint32_t i = 0; i < n && s->frmt_line < h; i++) {
            frmt_line(s, s->frmt_line++, i);
        }
    }
    s->regs[FRMT_STEP / 4] |= STEP_DONE;
    if (s->frmt_line >= h) {
        s->regs[FRMT_STAT / 4] &= ~RUN;
    }
}

/* where destination pixel (i, j) samples, in 16.16, inside the source tile */
static void frmc_pos(IngenicBscalerState *s, int64_t i, int64_t j,
                     int64_t *x, int64_t *y)
{
    int64_t m[9], den;

    for (int k = 0; k < 9; k++) {
        m[k] = (int32_t)reg(s, FRMC_MATRIX + 4 * k);
    }
    i += extract32(reg(s, FRMC_DST_ORIGIN), 0, 16);
    j += extract32(reg(s, FRMC_DST_ORIGIN), 16, 16);
    den = m[6] * i + m[7] * j + m[8];
    if (!den) {
        *x = *y = INT64_MIN;
        return;
    }
    *x = ((m[0] * i + m[1] * j + m[2]) << 16) / den -
         ((int64_t)extract32(reg(s, FRMC_SRC_ORIGIN), 0, 16) << 16);
    *y = ((m[3] * i + m[4] * j + m[5]) << 16) / den -
         ((int64_t)extract32(reg(s, FRMC_SRC_ORIGIN), 16, 16) << 16);
}

static unsigned frmc_lerp(unsigned a, unsigned b, unsigned w)
{
    return (a * (65535 - w) + b * w + 32768) >> 16;
}

/* one byte of a source plane; chroma spreads each pair over 2x2 pixels */
typedef struct {
    const uint8_t *buf;
    uint32_t w, h, stride;
    unsigned bpp;
    bool chroma;
    uint8_t pad;
} FrmcPlane;

static unsigned frmc_texel(const FrmcPlane *p, int64_t x, int64_t y, int k)
{
    if (x < 0 || y < 0 || x >= p->w || y >= p->h) {
        return p->pad;
    }
    if (p->chroma) {
        return p->buf[(y >> 1) * p->stride + (x >> 1) * 2 + k];
    }
    return p->buf[y * p->stride + x * p->bpp + k];
}

static unsigned frmc_sample(const FrmcPlane *p, int64_t x, int64_t y, int k)
{
    int64_t x0 = x >> 16, y0 = y >> 16;
    unsigned wx = x & 0xffff, wy = y & 0xffff;

    if (x == INT64_MIN) {
        return p->pad;
    }
    return frmc_lerp(frmc_lerp(frmc_texel(p, x0, y0, k),
                               frmc_texel(p, x0 + 1, y0, k), wx),
                     frmc_lerp(frmc_texel(p, x0, y0 + 1, k),
                               frmc_texel(p, x0 + 1, y0 + 1, k), wx), wy);
}

static void frmc_job(IngenicBscalerState *s)
{
    uint32_t mode = reg(s, FRMC_MODE), param = reg(s, BS_PARAM);
    uint32_t sw = extract32(reg(s, FRMC_SRC_SIZE), 0, 16);
    uint32_t sh = extract32(reg(s, FRMC_SRC_SIZE), 16, 16);
    uint32_t dw = extract32(reg(s, FRMC_DST_SIZE), 0, 16);
    uint32_t dh = extract32(reg(s, FRMC_DST_SIZE), 16, 16);
    uint32_t sstride = reg(s, FRMC_SRC_STRIDE);
    uint32_t dstride = reg(s, FRMC_DST_STRIDE);
    unsigned order = extract32(param, 4, 4), bpp = mode & MODE_SRC4 ? 4 : 1;
    uint8_t ypad = extract32(param, 16, 8), uvpad = extract32(param, 24, 8);
    bool nv12_out = !(mode & MODE_DST4) && (mode & MODE_NV12);
    g_autofree uint8_t *src = NULL;
    g_autofree uint8_t *uv = NULL;
    g_autofree uint8_t *bgr = NULL;
    g_autofree uint8_t *row = NULL;
    FrmcPlane p[3];

    if ((mode & MODE_NV12) && !(mode & MODE_SAMPLE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-bscaler: FRMC mode 0x%x "
                      "stalls a T40XP\n", mode);
        return;
    }
    if (mode & MODE_UNIMP) {
        qemu_log_mask(LOG_UNIMP, "ingenic-bscaler: FRMC mode 0x%x\n", mode);
        return;
    }
    if (!sw || !sh || !dw || !dh || sw > MAX_SIDE || sh > MAX_SIDE ||
        dw > MAX_SIDE || dh > MAX_SIDE || sstride < sw * bpp ||
        (uint64_t)sstride * sh > MAX_SOURCE) {
        return;
    }
    src = g_malloc((size_t)sstride * sh);
    address_space_read(&address_space_memory, reg(s, FRMC_SRC),
                       MEMTXATTRS_UNSPECIFIED, src, (hwaddr)sstride * sh);
    if (mode & MODE_SRC4) {
        /* Y, U, V or B, G, R and alpha, in its bytes */
        p[0] = (FrmcPlane){ src, sw, sh, sstride, 4, false, ypad };
        p[1] = p[2] = (FrmcPlane){ src, sw, sh, sstride, 4, false, uvpad };
    } else {
        /* an odd width's last pair reaches a byte past the plane */
        uv = g_malloc0((size_t)sstride * DIV_ROUND_UP(sh, 2) + 2);
        address_space_read(&address_space_memory, reg(s, FRMC_SRC_UV),
                           MEMTXATTRS_UNSPECIFIED, uv,
                           (hwaddr)sstride * DIV_ROUND_UP(sh, 2));
        p[0] = (FrmcPlane){ src, sw, sh, sstride, 1, false, ypad };
        p[1] = p[2] = (FrmcPlane){ uv, sw, sh, sstride, 2, true, uvpad };
    }
    if ((mode & MODE_DST4) && !(mode & MODE_SRC4)) {
        /* convert first, then warp the BGR picture */
        bgr = g_malloc((size_t)sw * sh * 4);
        for (uint32_t y = 0; y < sh; y++) {
            for (uint32_t x = 0; x < sw; x++) {
                bs_convert(s, order, src[y * sstride + x],
                           uv[(y / 2) * sstride + (x & ~1)],
                           uv[(y / 2) * sstride + (x | 1)],
                           &bgr[(y * sw + x) * 4]);
            }
        }
        p[0] = (FrmcPlane){ bgr, sw, sh, sw * 4, 4, false, 0 };
    }
    if (mode & MODE_DST4) {
        p[0].pad = 0;
    }
    row = g_malloc((size_t)dw * 4);
    for (uint32_t j = 0; j < dh; j++) {
        for (uint32_t i = 0; i < dw; i++) {
            int64_t x, y;
            uint8_t *px = nv12_out ? &row[i] : &row[i * 4];

            frmc_pos(s, i, j, &x, &y);
            if (!(mode & MODE_SAMPLE)) {
                x = y = INT64_MIN;
            }
            if (mode & MODE_DST4) {
                for (int k = 0; k < 4; k++) {
                    px[k] = frmc_sample(&p[0], x, y, k);
                }
                bs_alpha(s, order, px);
                continue;
            }
            if (nv12_out) {
                px[0] = frmc_sample(&p[0], x, y, 0);
                continue;
            }
            /* packed: chroma at the pixel's own position */
            bs_yuv(s, order, frmc_sample(&p[0], x, y, 0),
                   frmc_sample(&p[1], x, y, mode & MODE_SRC4 ? 1 : 0),
                   frmc_sample(&p[2], x, y, mode & MODE_SRC4 ? 2 : 1), px);
        }
        address_space_write(&address_space_memory,
                            s->frmc_dst[0] + (hwaddr)j * dstride,
                            MEMTXATTRS_UNSPECIFIED, row,
                            (hwaddr)dw * (nv12_out ? 1 : 4));
    }
    if (!nv12_out) {
        return;
    }
    /* each UV pair of NV12 output samples at its odd pixel */
    for (uint32_t j = 0; j < dh / 2; j++) {
        for (uint32_t i = 0; i < dw / 2; i++) {
            int64_t x, y;

            frmc_pos(s, 2 * i + 1, 2 * j + 1, &x, &y);
            row[2 * i] = frmc_sample(&p[1], x, y, mode & MODE_SRC4 ? 1 : 0);
            row[2 * i + 1] = frmc_sample(&p[2], x, y,
                                         mode & MODE_SRC4 ? 2 : 1);
        }
        address_space_write(&address_space_memory,
                            s->frmc_dst[1] + (hwaddr)j * dstride,
                            MEMTXATTRS_UNSPECIFIED, row, dw & ~1);
    }
}

static void frmc_write(IngenicBscalerState *s, hwaddr off, uint32_t v);

/* FRMC's chain: (value, offset) pairs, offset bit 31 ending a job */
static void frmc_chain(IngenicBscalerState *s)
{
    uint32_t base = reg(s, FRMC_CHAIN_BASE), size = reg(s, FRMC_CHAIN_SIZE);

    for (uint32_t o = 0; o + 8 <= size; o += 8) {
        uint32_t pair[2];
        hwaddr off;

        address_space_read(&address_space_memory, base + o,
                           MEMTXATTRS_UNSPECIFIED, pair, sizeof(pair));
        off = extract32(le32_to_cpu(pair[1]), 0, 8);
        if (off != FRMC_CTRL) {
            frmc_write(s, off, le32_to_cpu(pair[0]));
        }
    }
}

static void frmc_run(IngenicBscalerState *s)
{
    frmc_job(s);
    s->frmc_ndst = 0;
}

static void frmc_write(IngenicBscalerState *s, hwaddr off, uint32_t v)
{
    switch (off) {
    case FRMC_STAT:
        s->regs[0] = v & ~RUN;
        if (v & RUN) {
            frmc_run(s);
        }
        break;
    case FRMC_DST:
        s->regs[off / 4] = v;
        s->frmc_dst[MIN(s->frmc_ndst, 1)] = v;
        s->frmc_ndst = MIN(s->frmc_ndst + 1, 2);
        break;
    case FRMC_CTRL:
        if (v & SOFT_RESET) {
            /* on a T40XP: the origins and matrix only, 0x68 to 1 */
            memset(&s->regs[FRMC_SRC_ORIGIN / 4], 0,
                   FRMC_MATRIX_LAST - FRMC_SRC_ORIGIN);
            s->regs[FRMC_MATRIX_LAST / 4] = 1;
            s->regs[0] &= ~STAT_DONE;
            s->regs[off / 4] = 0;
            s->frmc_ndst = 0;
            break;
        }
        s->regs[off / 4] = v & ~RUN;
        if ((v & RUN) && (v & CHAIN)) {
            frmc_chain(s);
            s->regs[0] |= STAT_DONE;
            s->regs[off / 4] |= CTRL_DONE;
        } else if (v & RUN) {
            frmc_run(s);
            s->regs[0] |= STAT_DONE;
            s->regs[off / 4] |= CTRL_DONE;
        }
        break;
    case FRMT_STAT:
        if ((v & RUN) && !(reg(s, FRMT_STAT) & RUN)) {
            s->frmt_line = 0;
            s->regs[FRMT_CTRL / 4] |= FRMT_OPENED;
        }
        s->regs[off / 4] = v;
        break;
    case FRMT_STEP:
        if (v & RUN) {
            frmt_step(s, v);
        } else {
            s->regs[off / 4] = v;
        }
        break;
    case FRMT_CTRL:
        if (v & SOFT_RESET) {
            /* its parameters stay, as on a T40XP */
            s->regs[FRMT_STAT / 4] &= ~RUN;
            s->regs[FRMT_STEP / 4] &= ~(RUN | STEP_DONE);
            s->regs[off / 4] = 0;
            s->frmt_line = 0;
            break;
        }
        s->regs[off / 4] = v;
        break;
    default:
        if (off < sizeof(s->regs)) {
            s->regs[off / 4] = (s->regs[off / 4] & ~bs_mask(off)) |
                               (v & bs_mask(off));
        }
        break;
    }
    bscaler_update(s);
}

static uint64_t bscaler_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicBscalerState *s = opaque;
    uint64_t v = 0;

    if (offset < 0x20 && lzma_mode(s)) {
        memory_region_dispatch_read(&s->lzma->iomem, offset, &v, MO_32,
                                    MEMTXATTRS_UNSPECIFIED);
        return v;
    }
    return reg(s, offset);
}

static void bscaler_write(void *opaque, hwaddr offset, uint64_t value,
                          unsigned size)
{
    IngenicBscalerState *s = opaque;

    /* the LZMA's, or the write that sets its mode */
    if (offset < 0x20 && (lzma_mode(s) ||
                          (offset == 0 && (value & LZMA_MODE)))) {
        memory_region_dispatch_write(&s->lzma->iomem, offset, value, MO_32,
                                     MEMTXATTRS_UNSPECIFIED);
        return;
    }
    frmc_write(s, offset, value);
}

static const MemoryRegionOps bscaler_ops = {
    .read = bscaler_read,
    .write = bscaler_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void bscaler_realize(DeviceState *dev, Error **errp)
{
    IngenicBscalerState *s = INGENIC_BSCALER(dev);

    if (!s->lzma) {
        error_setg(errp, "ingenic-bscaler: the lzma link is not set");
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &bscaler_ops, s,
                          TYPE_INGENIC_BSCALER, INGENIC_BSCALER_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    for (int i = 0; i < 2; i++) {
        sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq[i]);
    }
}

static void bscaler_reset_hold(Object *obj, ResetType type)
{
    IngenicBscalerState *s = INGENIC_BSCALER(obj);

    bs_reset_regs(s);
    memset(s->frmc_dst, 0, sizeof(s->frmc_dst));
    s->frmc_ndst = 0;
    s->frmt_line = 0;
    bscaler_update(s);
}

static const VMStateDescription vmstate_bscaler = {
    .name = TYPE_INGENIC_BSCALER,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicBscalerState,
                             INGENIC_BSCALER_IOSIZE / 4),
        VMSTATE_UINT32_ARRAY(frmc_dst, IngenicBscalerState, 2),
        VMSTATE_UINT32(frmc_ndst, IngenicBscalerState),
        VMSTATE_UINT32(frmt_line, IngenicBscalerState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property bscaler_props[] = {
    DEFINE_PROP_LINK("lzma", IngenicBscalerState, lzma, TYPE_INGENIC_LZMA,
                     IngenicLzmaState *),
};

static void bscaler_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_props(dc, bscaler_props);
    dc->realize = bscaler_realize;
    dc->vmsd = &vmstate_bscaler;
    rc->phases.hold = bscaler_reset_hold;
}

static const TypeInfo bscaler_type_info = {
    .name = TYPE_INGENIC_BSCALER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicBscalerState),
    .class_init = bscaler_class_init,
};

static void bscaler_register_types(void)
{
    type_register_static(&bscaler_type_info);
}

type_init(bscaler_register_types)
