/*
 * Ingenic T33 and T32 Pro ISP: the table-driven LDC and its rotator
 *
 * The block at 0x27000 of the T33's ISP (0x1f000 on the T32 Pro), which
 * those parts' tx-isp drives for libimp's offline LDC and I2D jobs and
 * for its online LDC, I2D and HLDC. Modelled from the vendor drivers
 * alone (tx-isp PRJ008/PRJ009 tisp_offline_ldc_set_attr, libimp's
 * gen_undistort_lut and ldc_param_to_lut); no part has measured it:
 *
 * - 0x40 the job's type (bits 15:0: 0 a table-driven correction, 1 HLDC,
 *   2 a rotation) and the rotation (31:16: 0, 90, 180 or 270 degrees,
 *   mirror, flip), 0x44 the input's w | h << 16, 0x48 the output's, 0x4c
 *   Y in, 0x50 its stride, 0x54 Y out, 0x58 its stride, 0x5c-0x68 the UV
 *   plane's likewise, 0x6c the table, 0x70 its row stride in bytes, 0x74
 *   the block size (8 << n). A write of 1 to 0x24 starts a job; 0x80
 *   reads the status (bit 0 done, 1 an error), 0x84 clears it, 0x88 masks
 *   it, and the interrupt (IRQ_LDC, the line the T32's own LDC uses) is
 *   status & ~mask. The other words keep what is written.
 *
 * - The table: a node per block corner, the last column and row at w - 1
 *   and h - 1, each the x and y it takes the input from as two 16-bit Q4
 *   values. The model interpolates the four nodes around a pixel to
 *   1/1024 pixel and the input there as the 0x13040000 engine does; the
 *   chroma plane takes the luma position of its top-left pixel, halved.
 *   These precisions, the rotation's direction and the mirror and flip
 *   axes are the model's own choice.
 *
 * Not modelled: online jobs (the ISP's own frames; type 1 always is one),
 * the register lists tx-isp also programs the block through, and the
 * time a job takes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/misc/ingenic-isp-ldc.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define R(x)            (l->regs[(x) / 4])
#define LDC_START       0x24
#define LDC_TYPE        0x40
#define LDC_IN_SIZE     0x44
#define LDC_OUT_SIZE    0x48
#define LDC_Y_IN        0x4c
#define LDC_LUT         0x6c
#define LDC_LUT_STRIDE  0x70
#define LDC_BLOCK       0x74
#define LDC_STATUS      0x80
#define LDC_CLEAR       0x84
#define LDC_MASK        0x88

#define ST_DONE         BIT(0)
#define ST_ERROR        BIT(1)

/* The largest plane the model takes */
#define LDC_MAX_PLANE   (32 * MiB)

enum { Y_IN, Y_IN_ST, Y_OUT, Y_OUT_ST, UV_IN, UV_IN_ST, UV_OUT, UV_OUT_ST };

typedef struct LdcPlaneBuf {
    uint8_t *data;
    uint32_t w, h, stride, bpp;
} LdcPlaneBuf;

static uint32_t ldc_dma(const IngenicIspLdc *l, int i)
{
    return l->regs[LDC_Y_IN / 4 + i];
}

static bool ldc_load(const IngenicIspLdc *l, LdcPlaneBuf *p, int addr,
                     uint32_t w, uint32_t h, uint32_t bpp)
{
    uint64_t len;

    p->w = w;
    p->h = h;
    p->bpp = bpp;
    p->stride = ldc_dma(l, addr + 1) & 0xffff;
    len = (uint64_t)(h - 1) * p->stride + (uint64_t)w * bpp;
    if (!w || !h || p->stride < w * bpp || len > LDC_MAX_PLANE) {
        return false;
    }
    p->data = g_malloc(len);
    address_space_read(&address_space_memory, ldc_dma(l, addr),
                       MEMTXATTRS_UNSPECIFIED, p->data, len);
    return true;
}

/* Bilinear sample at x, y (Q10), edges repeated */
static uint8_t ldc_sample(const LdcPlaneBuf *p, int64_t x, int64_t y,
                          unsigned b)
{
    int64_t ix = x >> 10, iy = y >> 10;
    uint32_t fx = x & 1023, fy = y & 1023;
    uint32_t w11 = fx * fy >> 10;
    uint32_t wt[4] = { 1024 - fx - fy + w11, fx - w11, fy - w11, w11 };
    uint32_t v = 0;

    for (int k = 0; k < 4; k++) {
        int64_t sx = MIN(MAX(ix + (k & 1), 0), p->w - 1);
        int64_t sy = MIN(MAX(iy + (k >> 1), 0), p->h - 1);

        v += p->data[sy * p->stride + sx * p->bpp + b] * wt[k];
    }
    return v >> 10;
}

static void ldc_store_row(const IngenicIspLdc *l, int addr, uint32_t y,
                          const uint8_t *row, uint32_t len)
{
    address_space_write(&address_space_memory,
                        ldc_dma(l, addr) +
                        (uint64_t)y * (ldc_dma(l, addr + 1) & 0xffff),
                        MEMTXATTRS_UNSPECIFIED, row, len);
}

/* The table's node c, r: its x and y in Q4 */
static void ldc_node(const uint8_t *lut, uint32_t stride, uint32_t c,
                     uint32_t r, int32_t *x, int32_t *y)
{
    const uint8_t *n = lut + (size_t)r * stride + c * 4;

    *x = (int16_t)lduw_le_p(n);
    *y = (int16_t)lduw_le_p(n + 2);
}

/* Where luma pixel x, y takes the input from, in Q10 */
static void ldc_mesh_pos(const uint8_t *lut, uint32_t stride, uint32_t bs,
                         uint32_t w, uint32_t h, uint32_t x, uint32_t y,
                         int64_t *px, int64_t *py)
{
    uint32_t c = MIN(x / bs, (w - 1) / bs), r = MIN(y / bs, (h - 1) / bs);
    uint32_t x0 = c * bs, y0 = r * bs;
    uint32_t x1 = MIN(x0 + bs, w - 1), y1 = MIN(y0 + bs, h - 1);
    int64_t dx = MAX(x1 - x0, 1), dy = MAX(y1 - y0, 1);
    int64_t u = x - x0, v = y - y0;
    int32_t nx[4], ny[4];
    int64_t sx = 0, sy = 0;

    for (int k = 0; k < 4; k++) {
        ldc_node(lut, stride, c + (k & 1), r + (k >> 1), &nx[k], &ny[k]);
    }
    for (int k = 0; k < 4; k++) {
        int64_t wt = ((k & 1) ? u : dx - u) * ((k >> 1) ? v : dy - v);

        sx += nx[k] * wt;
        sy += ny[k] * wt;
    }
    /* Q4 over dx * dy, to Q10 */
    *px = (sx << 6) / (dx * dy);
    *py = (sy << 6) / (dx * dy);
}

static bool ldc_correct(IngenicIspLdc *l)
{
    uint32_t w = extract32(R(LDC_IN_SIZE), 0, 16);
    uint32_t h = extract32(R(LDC_IN_SIZE), 16, 16);
    uint32_t bs = 8 << (R(LDC_BLOCK) & 3);
    uint32_t stride = R(LDC_LUT_STRIDE);
    uint32_t rows = DIV_ROUND_UP(h, bs) + 1, cols = DIV_ROUND_UP(w, bs) + 1;
    g_autofree uint8_t *lut = NULL;
    g_autofree uint8_t *row = NULL;
    LdcPlaneBuf py = { 0 }, puv = { 0 };
    bool ok = false;

    if (w < 2 || h < 2 || stride < cols * 4 ||
        (uint64_t)rows * stride > LDC_MAX_PLANE) {
        return false;
    }
    lut = g_malloc((size_t)rows * stride);
    address_space_read(&address_space_memory, R(LDC_LUT),
                       MEMTXATTRS_UNSPECIFIED, lut, (size_t)rows * stride);
    if (!ldc_load(l, &py, Y_IN, w, h, 1) ||
        !ldc_load(l, &puv, UV_IN, w / 2, h / 2, 2)) {
        goto out;
    }
    row = g_malloc(w);
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            int64_t sx, sy;

            ldc_mesh_pos(lut, stride, bs, w, h, x, y, &sx, &sy);
            row[x] = ldc_sample(&py, sx, sy, 0);
        }
        ldc_store_row(l, Y_OUT, y, row, w);
    }
    for (uint32_t y = 0; y < h / 2; y++) {
        for (uint32_t x = 0; x < w / 2; x++) {
            int64_t sx, sy;

            ldc_mesh_pos(lut, stride, bs, w, h, 2 * x, 2 * y, &sx, &sy);
            for (unsigned b = 0; b < 2; b++) {
                row[2 * x + b] = ldc_sample(&puv, sx / 2, sy / 2, b);
            }
        }
        ldc_store_row(l, UV_OUT, y, row, w / 2 * 2);
    }
    ok = true;
out:
    g_free(py.data);
    g_free(puv.data);
    return ok;
}

/* One plane turned as I2D turns it: rot 0-3 clockwise, 4 mirror, 5 flip */
static void ldc_turn(IngenicIspLdc *l, const LdcPlaneBuf *p, int out,
                     unsigned rot)
{
    uint32_t ow = rot == 1 || rot == 3 ? p->h : p->w;
    uint32_t oh = rot == 1 || rot == 3 ? p->w : p->h;
    g_autofree uint8_t *row = g_malloc((size_t)ow * p->bpp);

    for (uint32_t dy = 0; dy < oh; dy++) {
        for (uint32_t dx = 0; dx < ow; dx++) {
            uint32_t sx, sy;

            switch (rot) {
            case 1:
                sx = dy;
                sy = p->h - 1 - dx;
                break;
            case 2:
                sx = p->w - 1 - dx;
                sy = p->h - 1 - dy;
                break;
            case 3:
                sx = p->w - 1 - dy;
                sy = dx;
                break;
            case 4:
                sx = p->w - 1 - dx;
                sy = dy;
                break;
            case 5:
                sx = dx;
                sy = p->h - 1 - dy;
                break;
            default:
                sx = dx;
                sy = dy;
                break;
            }
            memcpy(row + dx * p->bpp,
                   p->data + (size_t)sy * p->stride + sx * p->bpp, p->bpp);
        }
        ldc_store_row(l, out, dy, row, ow * p->bpp);
    }
}

static bool ldc_rotate(IngenicIspLdc *l)
{
    uint32_t w = extract32(R(LDC_IN_SIZE), 0, 16);
    uint32_t h = extract32(R(LDC_IN_SIZE), 16, 16);
    unsigned rot = extract32(R(LDC_TYPE), 16, 16);
    LdcPlaneBuf py = { 0 }, puv = { 0 };
    bool ok = false;

    if (rot > 5) {
        return false;
    }
    if (ldc_load(l, &py, Y_IN, w, h, 1) &&
        ldc_load(l, &puv, UV_IN, w / 2, h / 2, 2)) {
        ldc_turn(l, &py, Y_OUT, rot);
        ldc_turn(l, &puv, UV_OUT, rot);
        ok = true;
    }
    g_free(py.data);
    g_free(puv.data);
    return ok;
}

static void ldc_run(IngenicIspLdc *l)
{
    bool ok;

    switch (extract32(R(LDC_TYPE), 0, 16)) {
    case 0:
        ok = ldc_correct(l);
        break;
    case 2:
        ok = ldc_rotate(l);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-isp-ldc: job type %u\n",
                      extract32(R(LDC_TYPE), 0, 16));
        return;
    }
    if (!ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-isp-ldc: bad job\n");
    }
    R(LDC_STATUS) |= ok ? ST_DONE : ST_ERROR;
}

void ingenic_isp_ldc_reset(IngenicIspLdc *l)
{
    memset(l, 0, sizeof(*l));
}

uint32_t ingenic_isp_ldc_read(IngenicIspLdc *l, hwaddr reg)
{
    return reg == LDC_START || reg == LDC_CLEAR ? 0 : l->regs[reg / 4];
}

bool ingenic_isp_ldc_write(IngenicIspLdc *l, hwaddr reg, uint32_t v)
{
    switch (reg) {
    case LDC_START:
        if (v & 1) {
            ldc_run(l);
        }
        return true;
    case LDC_STATUS:
        return false;
    case LDC_CLEAR:
        R(LDC_STATUS) &= ~v;
        return true;
    default:
        l->regs[reg / 4] = v;
        return reg == LDC_MASK;
    }
}

bool ingenic_isp_ldc_irq(const IngenicIspLdc *l)
{
    return R(LDC_STATUS) & ~R(LDC_MASK) & (ST_DONE | ST_ERROR);
}

const VMStateDescription vmstate_ingenic_isp_ldc = {
    .name = "ingenic-isp-ldc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIspLdc, INGENIC_ISP_LDC_SIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};
