/*
 * Ingenic A1 IPU: the descriptor-driven 2D engine at 0x13080000
 *
 * With CPM CLKGR0 bit 29 (gate_ipu) set every register reads 0 and takes no
 * write, as on an A1N under U-Boot (measured).
 *
 * Mapped from the vendor ipu.ko and the libimp that drives it (TDE).
 * Registers:
 *   +0x10 GO (write 1 to run the descriptor at +0x14), +0x14 CHAIN
 *   +0x20 STATUS (bit 0: the chain is done), cleared by writing the bits
 *   to +0x24; +0x2c masks it
 *   +0x00 and +0x1000 read the version 0x20210708; +0x80 reads 0x00140120
 *   +0x40 resets the register file to its reset values (write 1; reads 0
 *   at once); +0x1024 reads 0 and clears nothing
 *   +0x1010 starts the job the registers describe
 *   +0x1020 writing 1 puts the job registers from +0x1040 back to their
 *   reset values; every descriptor starts with it, so none sees what the
 *   one before left
 *   +0x1014 IIP raw status (bits 1, 2 and 3 are errors), cleared by
 *   writing the bits to +0x1018; +0x101c masks it
 *   +0x1040 BK, +0x1060 FG, +0x1080 MK and +0x10a0 OT DMA: SIZE (width <<
 *   16 | height), ADDR, ADDR_UV, STRIDE (signed 16 bits of Y, UV above),
 *   INFO (bits 19:16 the pixel type, 23:20 its channel order, bit 30 a
 *   chroma plane, bit 31 enabled), INFO2, INFO3
 *   +0x10c0 CLIP, +0x10d0 CSC, +0x10f0 colour key, +0x1108 OSD, +0x1110
 *   SCALER, +0x1140 CP_DMA; all plain registers a job reads.
 * A descriptor is four words, the first (low 4 bits the TDE operation,
 * bits 4 and 5 the scaler, bits 31:16 the length of the rest) followed by
 * the next descriptor's address (0xffffffff for none) and the horizontal
 * and vertical scaler tables, then a list of register writes: a word of
 * count << 16 | register offset and that many values. The write of 1 to
 * 0x1010 that ends the list runs the job. The chain's end raises the IRQ
 * with STATUS bit 0.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "qemu/units.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-a1-ipu.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

#define REG_VERSION     0x00
#define REG_GO          0x10
#define REG_CHAIN       0x14
#define REG_STATUS      0x20
#define REG_STATUS_CLR  0x24
#define IPU_GATE_REG    0x30    /* CPM CLKGR0 */
#define IPU_GATE        (1u << 29)
#define REG_STATUS_MASK 0x2c
#define REG_RESET       0x40
#define REG_ID2         0x80
#define REG_VERSION2    0x1000
#define REG_KICK        0x1010
#define REG_IIP_RAW     0x1014
#define REG_IIP_CLR     0x1018
#define REG_IIP_MASK    0x101c
#define REG_RESET2      0x1024
#define REG_JOBRST      0x1020
#define REG_BK          0x1040
#define REG_FG          0x1060
#define REG_MK          0x1080
#define REG_OT          0x10a0
#define REG_CLIP        0x10c0
#define REG_CSC         0x10d0
#define REG_OSD         0x1108
#define REG_SCALER      0x1110

#define STATUS_DONE     0x1
#define VERSION         0x20210708
#define ID2_VALUE       0x00140120
#define STATUS_MASK_RESET 0x3
#define IIP_BADJOB      0x4

#define DESC_NONE       0xffffffffu
#define DESC_HDR        4
#define DESC_MAXWORDS   0x400

#define DMA_SIZE        0
#define DMA_ADDR        1
#define DMA_ADDR_UV     2
#define DMA_STRIDE      3
#define DMA_INFO        4

#define INFO_TYPE(i)    extract32(i, 16, 4)
#define INFO_CHROMA     (1u << 30)
#define INFO_EN         (1u << 31)
#define INFO_CONST      (1u << 25)      /* FG: a constant colour in INFO3 */
#define INFO_MIRROR     (1u << 28)      /* OT: each row turned round */

#define NS_PER_PIXEL    2
#define SCALER_TABLE_WORDS  (513 * 8)
#define SCALER_TABLE_BYTES  (SCALER_TABLE_WORDS * 2)

typedef struct Surface {
    uint32_t w, h;
    uint32_t addr, addr_uv;
    int32_t stride, stride_uv;
    uint32_t info;
    unsigned type;
} Surface;

static uint32_t rd32(hwaddr addr)
{
    uint32_t v = 0;

    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       &v, 4);
    return le32_to_cpu(v);
}

static void mem_read(hwaddr addr, void *buf, hwaddr len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
}

static void mem_write(hwaddr addr, const void *buf, hwaddr len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

static void ipu_update_irq(IngenicA1IpuState *s)
{
    bool on = (s->status & ~s->status_mask) || (s->iip_raw & ~s->iip_mask);

    qemu_set_irq(s->irq, on);
}

/* Bytes of one pixel of a type; 0 for the planar ones and unknown types */
static unsigned type_bpp(unsigned type)
{
    static const uint8_t bpp[8] = { 3, 2, 2, 2, 4, 3, 2, 2 };

    return type < ARRAY_SIZE(bpp) ? bpp[type] : 0;
}

static bool type_planar(unsigned type)
{
    return type >= 8 && type <= 11;
}

static void surface_load(IngenicA1IpuState *s, unsigned base, Surface *sf)
{
    const uint32_t *r = &s->regs[base / 4];

    sf->w = r[DMA_SIZE] >> 16;
    sf->h = r[DMA_SIZE] & 0xffff;
    sf->addr = r[DMA_ADDR];
    sf->addr_uv = r[DMA_ADDR_UV];
    sf->stride = (int16_t)r[DMA_STRIDE];
    sf->stride_uv = (int16_t)(r[DMA_STRIDE] >> 16);
    sf->info = r[DMA_INFO];
    sf->type = INFO_TYPE(sf->info);
}

/* The bytes a row of the surface holds, and how many rows each plane has */
static bool plane_geometry(const Surface *sf, unsigned plane, uint32_t *bytes,
                           uint32_t *rows)
{
    if (!type_planar(sf->type)) {
        if (plane || !type_bpp(sf->type)) {
            return false;
        }
        *bytes = sf->w * type_bpp(sf->type);
        *rows = sf->h;
        return true;
    }
    *bytes = sf->w;
    /* NV12 and NV21 halve the chroma rows, NV16 and NV61 do not */
    *rows = plane && sf->type < 10 ? sf->h / 2 : sf->h;
    return true;
}

typedef struct Job {
    uint32_t header;
    uint32_t tab_h, tab_v;      /* the scaler tables, at the descriptor */
} Job;

/* The unit a mirror turns round in a plane: a pixel, or the UV pair */
static unsigned plane_unit(const Surface *sf, unsigned plane)
{
    if (!type_planar(sf->type)) {
        return type_bpp(sf->type);
    }
    return plane ? 2 : 1;
}

/*
 * Rows from FG to OT without changing the pixels: the surfaces share a
 * type and a size. A negative FG stride reads the rows bottom up (the
 * start address stays the first row's), OT bit 28 turns each row round, the
 * clip box keeps what it holds or all but that, and a constant FG (bit 25)
 * fills with INFO3 in OT's format.
 */
static void job_raw(IngenicA1IpuState *s, const Surface *src,
                    const Surface *dst, bool fill, uint32_t color)
{
    bool mirror = dst->info & INFO_MIRROR;
    uint32_t clip_en = s->regs[REG_CLIP / 4 + 2] >> 31;
    bool inside = s->regs[REG_CLIP / 4 + 2] & 1;
    uint32_t cx = s->regs[REG_CLIP / 4] >> 16;
    uint32_t cy = s->regs[REG_CLIP / 4] & 0xffff;
    uint32_t cw = s->regs[REG_CLIP / 4 + 1] >> 16;
    uint32_t ch = s->regs[REG_CLIP / 4 + 1] & 0xffff;
    g_autofree uint8_t *row = NULL;
    g_autofree uint8_t *tmp = NULL;

    for (unsigned plane = 0; plane < (type_planar(dst->type) ? 2 : 1);
         plane++) {
        uint32_t bytes, rows;
        unsigned unit = plane_unit(dst, plane);
        unsigned sub = plane && dst->type < 10 ? 2 : 1;
        hwaddr sa = plane ? src->addr_uv : src->addr;
        hwaddr da = plane ? dst->addr_uv : dst->addr;
        int32_t ss = plane ? src->stride_uv : src->stride;
        int32_t ds = plane ? dst->stride_uv : dst->stride;

        if (!plane_geometry(dst, plane, &bytes, &rows)) {
            continue;
        }
        row = g_realloc(row, bytes);
        tmp = g_realloc(tmp, bytes);
        for (uint32_t y = 0; y < rows; y++) {
            uint32_t sy = ss < 0 ? rows - 1 - y : y;
            hwaddr base = sa + (int64_t)sy * abs(ss);

            if (fill) {
                for (uint32_t i = 0; i < bytes; i++) {
                    row[i] = color >> (8 * (i % unit));
                }
            } else {
                mem_read(base, row, bytes);
            }
            if (mirror) {
                for (uint32_t i = 0; i < bytes; i += unit) {
                    memcpy(tmp + i, row + bytes - unit - i, unit);
                }
                memcpy(row, tmp, bytes);
            }
            if (clip_en) {
                bool row_in = y * sub >= cy && y * sub < cy + ch;
                uint32_t x0 = MIN(cx / sub * unit, bytes);
                uint32_t x1 = MIN((cx + cw + sub - 1) / sub * unit, bytes);

                if (inside) {
                    if (!row_in) {
                        continue;
                    }
                    mem_write(da + (int64_t)y * ds + x0, row + x0, x1 - x0);
                } else if (!row_in) {
                    mem_write(da + (int64_t)y * ds, row, bytes);
                } else {
                    mem_write(da + (int64_t)y * ds, row, x0);
                    mem_write(da + (int64_t)y * ds + x1, row + x1, bytes - x1);
                }
                continue;
            }
            mem_write(da + (int64_t)y * ds, row, bytes);
        }
    }
}

/*
 * The general path works on 8-bit A, R, G, B pixels. For the planar types
 * R, G and B carry Y, Cb and Cr, the chroma repeated over its 2x2 (or 2x1)
 * block.
 */
typedef struct Img {
    uint32_t w, h;
    uint8_t *px;                /* w * h * 4: A, R, G, B */
} Img;

typedef struct Field {
    uint8_t shift, width;
} Field;

/* The components of a packed type, A, R, G and B, from the order code */
static bool type_fields(unsigned type, unsigned order, Field f[4])
{
    static const uint8_t widths[8][4] = {   /* A, R, G, B */
        { 0, 8, 8, 8 }, { 0, 5, 6, 5 }, { 0, 5, 5, 5 }, { 0, 4, 4, 4 },
        { 8, 8, 8, 8 }, { 8, 5, 6, 5 }, { 1, 5, 5, 5 }, { 4, 4, 4, 4 },
    };
    /* From the top: A R G B, A B G R, R G B A, B G R A (A is 0 and B 3) */
    static const struct {
        uint8_t order, comp[4];
    } layouts[] = {
        { 0, { 0, 1, 2, 3 } }, { 5, { 0, 3, 2, 1 } },
        { 8, { 1, 2, 3, 0 } }, { 13, { 3, 2, 1, 0 } },
    };
    unsigned shift = 0;

    if (type >= 8) {
        return false;
    }
    for (unsigned l = 0; l < ARRAY_SIZE(layouts); l++) {
        if (layouts[l].order != order) {
            continue;
        }
        for (int i = 3; i >= 0; i--) {
            unsigned c = layouts[l].comp[i];

            f[c].shift = shift;
            f[c].width = widths[type][c];
            shift += widths[type][c];
        }
        return true;
    }
    return false;
}

static uint8_t expand(unsigned v, unsigned width)
{
    switch (width) {
    case 4:
        return v * 17;
    case 5:
        return v << 3 | v >> 2;
    case 6:
        return v << 2 | v >> 4;
    default:
        return v;
    }
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static void img_free(Img *im)
{
    g_free(im->px);
    im->px = NULL;
}

/* Row y of a surface: a negative stride reads the rows bottom up */
static hwaddr surf_row(const Surface *sf, int32_t stride, hwaddr base,
                       uint32_t rows, uint32_t y)
{
    return base + (int64_t)(stride < 0 ? rows - 1 - y : y) * abs(stride);
}

static bool img_load(const Surface *sf, Img *im)
{
    Field f[4];
    unsigned bpp = type_bpp(sf->type);
    uint8_t alpha1 = sf->info & 0xff, alpha0 = sf->info >> 8 & 0xff;
    g_autofree uint8_t *row = NULL;
    g_autofree uint8_t *crow = NULL;

    if (!alpha1 && !alpha0) {
        alpha1 = 255;
    }
    im->w = sf->w;
    im->h = sf->h;
    im->px = g_malloc0((size_t)sf->w * sf->h * 4);
    if (type_planar(sf->type)) {
        bool uv_first = sf->type == 9 || sf->type == 11;
        bool half_rows = sf->type < 10;

        row = g_malloc(sf->w);
        crow = g_malloc(sf->w + 2);
        for (uint32_t y = 0; y < sf->h; y++) {
            uint32_t cy = half_rows ? y / 2 : y;

            mem_read(surf_row(sf, sf->stride, sf->addr, sf->h, y), row, sf->w);
            mem_read(surf_row(sf, sf->stride_uv, sf->addr_uv,
                              half_rows ? sf->h / 2 : sf->h, cy), crow, sf->w);
            for (uint32_t x = 0; x < sf->w; x++) {
                uint8_t *p = im->px + ((size_t)y * sf->w + x) * 4;
                uint8_t u = crow[x & ~1], v = crow[(x & ~1) + 1];

                p[0] = 255;
                p[1] = row[x];
                p[2] = uv_first ? v : u;
                p[3] = uv_first ? u : v;
            }
        }
        return true;
    }
    if (!bpp || !type_fields(sf->type, sf->info >> 20 & 0xf, f)) {
        return false;
    }
    row = g_malloc((size_t)sf->w * bpp);
    for (uint32_t y = 0; y < sf->h; y++) {
        mem_read(surf_row(sf, sf->stride, sf->addr, sf->h, y), row,
                 sf->w * bpp);
        for (uint32_t x = 0; x < sf->w; x++) {
            uint32_t v = 0;
            uint8_t *p = im->px + ((size_t)y * sf->w + x) * 4;

            for (unsigned i = 0; i < bpp; i++) {
                v |= row[x * bpp + i] << (8 * i);
            }
            for (unsigned c = 0; c < 4; c++) {
                unsigned raw = extract32(v, f[c].shift, f[c].width);

                if (!f[c].width) {
                    p[c] = 255;
                } else if (c == 0 && f[c].width == 1) {
                    p[c] = raw ? alpha1 : alpha0;
                } else {
                    p[c] = expand(raw, f[c].width);
                }
            }
        }
    }
    return true;
}

/*
 * Write the picture to OT: mirrored if OT asks, and through the clip box.
 * The planar types take the chroma of the first pixel of each block.
 */
static bool img_store(IngenicA1IpuState *s, const Surface *dst, const Img *im)
{
    bool mirror = dst->info & INFO_MIRROR;
    bool clip_en = s->regs[REG_CLIP / 4 + 2] >> 31;
    bool inside = s->regs[REG_CLIP / 4 + 2] & 1;
    uint32_t cx = s->regs[REG_CLIP / 4] >> 16;
    uint32_t cy = s->regs[REG_CLIP / 4] & 0xffff;
    uint32_t cw = s->regs[REG_CLIP / 4 + 1] >> 16;
    uint32_t ch = s->regs[REG_CLIP / 4 + 1] & 0xffff;
    unsigned bpp = type_bpp(dst->type);
    Field f[4];
    g_autofree uint8_t *row = NULL;
    uint32_t w = im->w;

    if (!type_planar(dst->type) &&
        (!bpp || !type_fields(dst->type, dst->info >> 20 & 0xf, f))) {
        return false;
    }
    row = g_malloc((size_t)w * 4);
    for (unsigned plane = 0; plane < (type_planar(dst->type) ? 2 : 1);
         plane++) {
        unsigned sub = plane && dst->type < 10 ? 2 : 1;
        unsigned unit = type_planar(dst->type) ? (plane ? 2 : 1) : bpp;
        uint32_t rows = im->h / sub;
        uint32_t cols = plane ? w / 2 : w;
        hwaddr base = plane ? dst->addr_uv : dst->addr;
        int32_t stride = plane ? dst->stride_uv : dst->stride;
        g_autofree uint8_t *orig = g_malloc((size_t)cols * unit);

        for (uint32_t y = 0; y < rows; y++) {
            hwaddr da = base + (int64_t)y * stride;
            uint32_t ypix = y * sub;

            if (clip_en) {
                mem_read(da, orig, cols * unit);
            }
            for (uint32_t x = 0; x < cols; x++) {
                uint32_t sx = mirror ? cols - 1 - x : x;
                const uint8_t *p = im->px + ((size_t)ypix * w + sx *
                                             (plane ? 2 : 1)) * 4;
                uint8_t *o = row + x * unit;
                bool in = x * (plane ? 2 : 1) >= cx &&
                          x * (plane ? 2 : 1) < cx + cw &&
                          ypix >= cy && ypix < cy + ch;

                if (clip_en && in != inside) {
                    memcpy(o, orig + x * unit, unit);
                    continue;
                }
                if (type_planar(dst->type)) {
                    if (!plane) {
                        o[0] = p[1];
                    } else {
                        bool uv_first = dst->type == 9 || dst->type == 11;

                        o[0] = p[uv_first ? 3 : 2];
                        o[1] = p[uv_first ? 2 : 3];
                    }
                } else {
                    uint32_t v = 0;

                    for (unsigned c = 0; c < 4; c++) {
                        uint32_t q = f[c].width == 1 ?
                                     p[c] >= 0x80 : p[c] >> (8 - f[c].width);

                        v = deposit32(v, f[c].shift, f[c].width, q);
                    }
                    for (unsigned i = 0; i < bpp; i++) {
                        o[i] = v >> (8 * i);
                    }
                }
            }
            mem_write(da, row, cols * unit);
        }
    }
    return true;
}

/* A weight row of the scaler table: 8 signed 14-bit taps for a phase */
static int tap_weight(const uint16_t *tab, unsigned phase, unsigned k)
{
    int v = tab[phase * 8 + k] & 0x3fff;

    return v & 0x2000 ? v - 0x4000 : v;
}

/*
 * One pass of the scaler along rows (or columns): the source position of
 * output n is step * n advanced half a step and back half a pixel, its
 * fraction picks a phase of the 513, and the eight pixels from three
 * before it to four after it, edges repeated, are weighted by that row of
 * the table, which sums to 2048.
 */
static void scale_pass(const uint16_t *tab, uint32_t step, const Img *src,
                       Img *dst, bool horiz)
{
    uint32_t n_out = horiz ? dst->w : dst->h;
    uint32_t n_in = horiz ? src->w : src->h;
    uint32_t lines = horiz ? src->h : src->w;

    for (uint32_t o = 0; o < n_out; o++) {
        int64_t pos = (int64_t)step * o + (step >> 1) - (1 << 13);
        int64_t ix = pos >> 14;
        unsigned phase = MIN(((pos & 0x3fff) + 16) >> 5, 512);

        for (uint32_t l = 0; l < lines; l++) {
            for (unsigned c = 0; c < 4; c++) {
                int acc = 0;

                for (unsigned k = 0; k < 8; k++) {
                    int64_t i = MIN(MAX(ix - 3 + k, 0), (int64_t)n_in - 1);
                    size_t at = horiz ? ((size_t)l * src->w + i)
                                      : ((size_t)i * src->w + l);

                    acc += tap_weight(tab, phase, k) * src->px[at * 4 + c];
                }
                size_t out = horiz ? ((size_t)l * dst->w + o)
                                   : ((size_t)o * dst->w + l);

                dst->px[out * 4 + c] = clamp8((acc + 1024) >> 11);
            }
        }
    }
}

static bool img_scale(const Job *job, const uint32_t *sc, Img *im, uint32_t w,
                      uint32_t h)
{
    g_autofree uint16_t *th = g_malloc(SCALER_TABLE_BYTES);
    g_autofree uint16_t *tv = g_malloc(SCALER_TABLE_BYTES);
    Img mid = { .w = w, .h = im->h };
    Img out = { .w = w, .h = h };

    if (!job->tab_h || !job->tab_v) {
        return false;
    }
    mem_read(job->tab_h, th, SCALER_TABLE_BYTES);
    mem_read(job->tab_v, tv, SCALER_TABLE_BYTES);
    for (unsigned i = 0; i < SCALER_TABLE_WORDS; i++) {
        th[i] = le16_to_cpu(th[i]);
        tv[i] = le16_to_cpu(tv[i]);
    }
    mid.px = g_malloc((size_t)mid.w * mid.h * 4);
    out.px = g_malloc((size_t)out.w * out.h * 4);
    scale_pass(th, sc[1], im, &mid, true);
    scale_pass(tv, sc[2], &mid, &out, false);
    g_free(mid.px);
    g_free(im->px);
    *im = out;
    return true;
}

/*
 * The colour conversion: nine signed 14-bit coefficients, 10 fraction bits,
 * two to a word from +0x10d4 in the order C00 C01 C10 C11 C20 C21 C02 C12
 * C22, rows of (R, G, B) or (Y, Cb, Cr) on the three channels. +0x10d0 holds
 * the mode in bits 1:0 (1 RGB to YUV, 2 YUV to RGB), the luma offset in
 * bits 23:16 and the chroma offset in bits 31:24.
 */
static void img_csc(const uint32_t *r, Img *im)
{
    unsigned mode = r[0] & 3;
    int luma = r[0] >> 16 & 0xff, chroma = r[0] >> 24;
    int c[9];

    if (!mode) {
        return;
    }
    for (unsigned i = 0; i < 9; i++) {
        c[i] = sextract32(r[1 + i / 2] >> (16 * (i & 1)), 0, 14);
    }
    {
        const int m[3][3] = { { c[0], c[1], c[6] }, { c[2], c[3], c[7] },
                              { c[4], c[5], c[8] } };

        for (size_t i = 0; i < (size_t)im->w * im->h; i++) {
            uint8_t *p = im->px + i * 4;
            int in[3] = { p[1], p[2], p[3] };

            if (mode == 2) {
                in[0] -= luma;
                in[1] -= chroma;
                in[2] -= chroma;
            }
            for (unsigned k = 0; k < 3; k++) {
                int v = (m[k][0] * in[0] + m[k][1] * in[1] +
                         m[k][2] * in[2] + 512) >> 10;

                if (mode == 1) {
                    v += k ? chroma : luma;
                }
                p[1 + k] = clamp8(v);
            }
        }
    }
}

/*
 * The blend of FG over BK (OSD at +0x1108: bit 31 on, the command in bits
 * 3:0, the factor modes of BK and FG, the TDE's source 1 and 2, in bits 7:4
 * and 11:8). A surface's alpha is its pixel alpha; INFO2 bits 9:8 of 2 make
 * it the global alpha in INFO2 bits 7:0 and of 3 the two multiplied.
 * Commands other than 0 (use the modes) and 14 (the same) set the factors
 * the Porter-Duff way, on straight alpha.
 */
enum { F_ZERO, F_ONE, F_S2C, F_IS2C, F_S2A, F_IS2A, F_S1C, F_IS1C, F_S1A,
       F_IS1A, F_S2ASAT };

static unsigned alpha_of(uint32_t info2, unsigned pixel)
{
    switch (info2 >> 8 & 3) {
    case 2:
        return info2 & 0xff;
    case 3:
        return (pixel * (info2 & 0xff) + 127) / 255;
    default:
        return pixel;
    }
}

static void blend_modes(unsigned cmd, unsigned m1, unsigned m2,
                        unsigned *f1, unsigned *f2)
{
    static const uint8_t pd[][2] = {
        [1] = { F_ZERO, F_ZERO }, [2] = { F_ZERO, F_ONE },
        [3] = { F_IS2A, F_S2A }, [4] = { F_S1A, F_IS1A },
        [5] = { F_ZERO, F_S1A }, [6] = { F_S2A, F_ZERO },
        [7] = { F_ZERO, F_IS1A }, [8] = { F_IS2A, F_ZERO },
        [9] = { F_IS2A, F_S1A }, [10] = { F_S2A, F_IS1A },
        [11] = { F_ONE, F_ONE }, [12] = { F_IS2A, F_IS1A },
        [13] = { F_ONE, F_ZERO },
    };

    if (cmd >= 1 && cmd <= 13) {
        *f1 = pd[cmd][0];
        *f2 = pd[cmd][1];
    } else {
        *f1 = m1;
        *f2 = m2;
    }
}

/* A factor, 0 to 255, for one channel (0 is the alpha) of BK p1 and FG p2 */
static unsigned blend_factor(unsigned mode, const unsigned *p1,
                             const unsigned *p2, unsigned c)
{
    switch (mode) {
    case F_ONE:
        return 255;
    case F_S2C:
        return p2[c];
    case F_IS2C:
        return 255 - p2[c];
    case F_S2A:
        return p2[0];
    case F_IS2A:
        return 255 - p2[0];
    case F_S1C:
        return p1[c];
    case F_IS1C:
        return 255 - p1[c];
    case F_S1A:
        return p1[0];
    case F_IS1A:
        return 255 - p1[0];
    case F_S2ASAT:
        return c ? MIN(p2[0], 255 - p1[0]) : 255;
    default:
        return 0;
    }
}

static bool img_blend(IngenicA1IpuState *s, Img *fg)
{
    const uint32_t *osd = &s->regs[REG_OSD / 4];
    Surface bk;
    Img bg = { 0 };
    unsigned f1, f2;

    surface_load(s, REG_BK, &bk);
    if (!(bk.info & INFO_EN) || !img_load(&bk, &bg)) {
        return false;
    }
    if (bg.w != fg->w || bg.h != fg->h) {
        img_free(&bg);
        return false;
    }
    blend_modes(*osd & 0xf, *osd >> 4 & 0xf, *osd >> 8 & 0xf, &f1, &f2);
    for (size_t i = 0; i < (size_t)fg->w * fg->h; i++) {
        uint8_t *q = fg->px + i * 4;
        const uint8_t *b = bg.px + i * 4;
        unsigned p1[4], p2[4];

        p1[0] = alpha_of(s->regs[REG_BK / 4 + 5], b[0]);
        p2[0] = alpha_of(s->regs[REG_FG / 4 + 5], q[0]);
        for (unsigned c = 1; c < 4; c++) {
            p1[c] = b[c];
            p2[c] = q[c];
        }
        for (unsigned c = 0; c < 4; c++) {
            unsigned v = (p1[c] * blend_factor(f1, p1, p2, c) +
                          p2[c] * blend_factor(f2, p1, p2, c) + 127) / 255;

            q[c] = MIN(v, 255);
        }
    }
    img_free(&bg);
    return true;
}

/* The job registers' reset values, measured on an A1N */
static const struct {
    uint16_t reg;
    uint32_t val;
} job_reset[] = {
    { 0x1040, 0x028001e0 }, { 0x1050, 0xff }, { 0x1054, 0xff },
    { 0x1060, 0x028001e0 }, { 0x1070, 0xff }, { 0x1074, 0xff },
    { 0x1080, 0x028001e0 }, { 0x10a0, 0x028001e0 }, { 0x10b0, 0xff },
    { 0x10d0, 0x80100000 },
};

/* The bits a register keeps; the others read 0, 0xffffffff if not listed */
static const struct {
    uint16_t reg;
    uint32_t mask;
} job_keep[] = {
    { 0x1050, 0xc2ffffff }, { 0x10c8, 0x80000001 }, { 0x10d0, 0xffff0003 },
    { 0x10d4, 0x3fff3fff }, { 0x1108, 0x80000fff }, { 0x1110, 0x01011111 },
    { 0x1114, 0x001fffff },
};

static uint32_t job_reg_mask(uint32_t reg)
{
    for (unsigned i = 0; i < ARRAY_SIZE(job_keep); i++) {
        if (job_keep[i].reg == reg) {
            return job_keep[i].mask;
        }
    }
    return 0xffffffff;
}

static void ipu_clear_job_regs(IngenicA1IpuState *s)
{
    memset(&s->regs[REG_BK / 4], 0, sizeof(s->regs) - REG_BK);
    for (unsigned i = 0; i < ARRAY_SIZE(job_reset); i++) {
        s->regs[job_reset[i].reg / 4] = job_reset[i].val;
    }
}

static bool ipu_run_job(IngenicA1IpuState *s, const Job *job)
{
    Surface src, dst;
    bool fill;
    const uint32_t *sc = &s->regs[REG_SCALER / 4];
    bool scaling = (job->header & 0x30) && (sc[0] & 0x11);
    bool blending = s->regs[REG_OSD / 4] & INFO_EN;

    surface_load(s, REG_FG, &src);
    surface_load(s, REG_OT, &dst);
    fill = src.info & INFO_CONST;
    if (!(dst.info & INFO_EN) || !(fill || (src.info & INFO_EN))) {
        return false;
    }
    if (fill) {
        src.type = dst.type;
        src.w = dst.w;
        src.h = dst.h;
    }
    if (!scaling && !blending && !(s->regs[REG_CSC / 4] & 3) &&
        src.type == dst.type && src.w == dst.w && src.h == dst.h) {
        job_raw(s, &src, &dst, fill, s->regs[REG_FG / 4 + 6]);
    } else {
        Img im = { 0 };
        bool ok = img_load(&src, &im);

        if (ok && scaling) {
            ok = img_scale(job, sc, &im, dst.w, dst.h);
        }
        if (ok) {
            img_csc(&s->regs[REG_CSC / 4], &im);
        }
        if (ok && blending) {
            ok = img_blend(s, &im);
        }
        ok = ok && im.w == dst.w && im.h == dst.h && img_store(s, &dst, &im);
        img_free(&im);
        if (!ok) {
            qemu_log_mask(LOG_UNIMP, "ingenic-a1-ipu: job (operation %u) "
                          "%ux%u type %u to %ux%u type %u\n", job->header & 0xf,
                          src.w, src.h, src.type, dst.w, dst.h, dst.type);
            return false;
        }
    }
    s->busy_ns += (int64_t)dst.w * dst.h * NS_PER_PIXEL;
    return true;
}

/* Run one descriptor; returns the next one's address */
static uint32_t ipu_run_desc(IngenicA1IpuState *s, uint32_t addr)
{
    Job job = { .header = rd32(addr), .tab_h = rd32(addr + 8),
                .tab_v = rd32(addr + 12) };
    uint32_t hdr = job.header;
    uint32_t next = rd32(addr + 4);
    uint32_t nwords = hdr >> 16;
    uint32_t i = 0;

    trace_ingenic_a1_ipu_desc(addr, hdr, next, nwords, job.tab_h, job.tab_v);
    if (nwords > DESC_MAXWORDS) {
        s->iip_raw |= IIP_BADJOB;
        return DESC_NONE;
    }
    while (i < nwords) {
        uint32_t cmd = rd32(addr + (DESC_HDR + i++) * 4);
        uint32_t reg = cmd & 0xffff;
        uint32_t count = cmd >> 16;

        for (uint32_t k = 0; k < count && i < nwords; k++, i++) {
            uint32_t v = rd32(addr + (DESC_HDR + i) * 4);
            uint32_t r = reg + k * 4;

            trace_ingenic_a1_ipu_write(r, v);
            if (r == REG_JOBRST) {
                if (v & 1) {
                    ipu_clear_job_regs(s);
                }
            } else if (r >= REG_KICK && r < REG_KICK + 4) {
                if (v & 1) {
                    if (!ipu_run_job(s, &job)) {
                        s->iip_raw |= IIP_BADJOB;
                    }
                }
            } else if (r >= REG_KICK && r / 4 < INGENIC_A1_IPU_NREGS) {
                s->regs[r / 4] = v & job_reg_mask(r);
            }
        }
    }
    return next;
}

static void ipu_timer_cb(void *opaque)
{
    IngenicA1IpuState *s = opaque;

    s->running = false;
    s->status |= STATUS_DONE;
    ipu_update_irq(s);
}

static void ipu_go(IngenicA1IpuState *s)
{
    uint32_t addr = s->chain;
    unsigned guard = 0;

    s->busy_ns = 1000;
    while (addr != DESC_NONE && addr && guard++ < 64) {
        addr = ipu_run_desc(s, addr);
    }
    s->running = true;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->busy_ns);
}

static void ipu_reset_regs(IngenicA1IpuState *s)
{
    ipu_clear_job_regs(s);
    s->status = 0;
    s->iip_raw = 0;
    s->iip_mask = 0;
    s->running = false;
    timer_del(s->timer);
    ipu_update_irq(s);
}

static bool ipu_gated(IngenicA1IpuState *s)
{
    return s->cpm && (s->cpm->regs[IPU_GATE_REG / 4] & IPU_GATE);
}

static uint64_t ipu_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicA1IpuState *s = opaque;

    if (ipu_gated(s)) {
        return 0;
    }
    switch (addr) {
    case REG_VERSION:
    case REG_VERSION2:
        return VERSION;
    case REG_ID2:
        return ID2_VALUE;
    case REG_GO:
        return s->running;
    case REG_CHAIN:
        return s->chain;
    case REG_STATUS:
        return s->status;
    case REG_STATUS_MASK:
        return s->status_mask;
    case REG_RESET:
    case REG_RESET2:
        return 0;
    case REG_IIP_RAW:
        return s->iip_raw;
    case REG_IIP_MASK:
        return s->iip_mask;
    }
    if (addr < INGENIC_A1_IPU_NREGS * 4 && addr >= REG_KICK) {
        return s->regs[addr / 4];
    }
    return 0;
}

static void ipu_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IngenicA1IpuState *s = opaque;

    if (ipu_gated(s)) {
        return;
    }
    switch (addr) {
    case REG_GO:
        if ((val & 1) && !s->running) {
            ipu_go(s);
        }
        return;
    case REG_CHAIN:
        s->chain = val;
        return;
    case REG_STATUS_CLR:
        s->status &= ~val;
        break;
    case REG_STATUS_MASK:
        s->status_mask = val;
        break;
    case REG_RESET:
        if (val & 1) {
            ipu_reset_regs(s);
        }
        return;
    case REG_RESET2:
    case REG_VERSION:
    case REG_VERSION2:
    case REG_ID2:
        return;
    case REG_IIP_CLR:
        s->iip_raw &= ~val;
        break;
    case REG_IIP_MASK:
        s->iip_mask = val;
        break;
    case REG_JOBRST:
        if (val & 1) {
            ipu_clear_job_regs(s);
        }
        return;
    case REG_KICK:
        if (val & 1) {
            ipu_run_job(s, &(Job){ 0 });
        }
        return;
    default:
        if (addr >= REG_KICK && addr < INGENIC_A1_IPU_NREGS * 4) {
            s->regs[addr / 4] = val & job_reg_mask(addr);
        }
        return;
    }
    ipu_update_irq(s);
}

static const Property ipu_props[] = {
    DEFINE_PROP_LINK("cpm", IngenicA1IpuState, cpm, TYPE_INGENIC_A1_CPM,
                     IngenicA1CpmState *),
};

static const MemoryRegionOps ipu_ops = {
    .read = ipu_read,
    .write = ipu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ipu_reset_hold(Object *obj, ResetType type)
{
    IngenicA1IpuState *s = INGENIC_A1_IPU(obj);

    ipu_reset_regs(s);
    s->chain = 0;
    s->status_mask = STATUS_MASK_RESET;
    s->iip_mask = 0;
}

static void ipu_realize(DeviceState *dev, Error **errp)
{
    IngenicA1IpuState *s = INGENIC_A1_IPU(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ipu_timer_cb, s);
}

static void ipu_init(Object *obj)
{
    IngenicA1IpuState *s = INGENIC_A1_IPU(obj);

    memory_region_init_io(&s->iomem, obj, &ipu_ops, s, "ingenic-a1-ipu",
                          INGENIC_A1_IPU_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_ipu = {
    .name = "ingenic-a1-ipu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicA1IpuState, INGENIC_A1_IPU_NREGS),
        VMSTATE_UINT32(status, IngenicA1IpuState),
        VMSTATE_UINT32(status_mask, IngenicA1IpuState),
        VMSTATE_UINT32(iip_raw, IngenicA1IpuState),
        VMSTATE_UINT32(iip_mask, IngenicA1IpuState),
        VMSTATE_UINT32(chain, IngenicA1IpuState),
        VMSTATE_BOOL(running, IngenicA1IpuState),
        VMSTATE_TIMER_PTR(timer, IngenicA1IpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ipu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ipu_realize;
    dc->vmsd = &vmstate_ipu;
    device_class_set_props(dc, ipu_props);
    rc->phases.hold = ipu_reset_hold;
}

static const TypeInfo ipu_type_info = {
    .name = TYPE_INGENIC_A1_IPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1IpuState),
    .instance_init = ipu_init,
    .class_init = ipu_class_init,
};

static void ipu_register_types(void)
{
    type_register_static(&ipu_type_info);
}

type_init(ipu_register_types)
