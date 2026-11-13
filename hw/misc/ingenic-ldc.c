/*
 * Ingenic LDC, the radial lens distortion correction engine (T30, T32, T33,
 * T41)
 *
 * A job reads an NV12 picture from memory and writes it back corrected,
 * each output pixel taking the input at a point pulled towards or pushed
 * from the picture's centre. As a T32LQ (the 0x13040000 layout, which the
 * vendor kernels give the T32, T33 and T41 alike) and a T30X (its own
 * layout at 0x130a0000) do it, byte for byte:
 *
 * - A pixel d columns right of the centre cx = w / 2 (x - cx; on the left
 *   cx - 1 - x) and e rows from it, likewise, has r = (d * d + e * e) *
 *   R2 mod 2^30 (Q30), f = 2^40 - k1 * r + k2 * (r * r >> 30) (Q40, k1
 *   and k2 Q10) and takes the input d * f * zoom >> 41 (Q10, zoom Q11)
 *   from the centre, mirrored on the left: cx + that or cx - 1 - that,
 *   along each axis with its own zoom. The chroma plane does the same at
 *   half the size with four times R2, U and V alike.
 *
 * - The input there is interpolated from the four pixels around it with
 *   weights w11 = fx * fy >> 10, w01 = fx - w11, w10 = fy - w11 and w00 =
 *   1024 - fx - fy + w11 (fx, fy the position's fractions in Q10), the
 *   sum cut down by 10 bits. The T30 takes pixels outside the plane as
 *   its fill values (0x24: Y bits 23:16, U 15:8, V 7:0; the column just
 *   left of the picture came out otherwise on the T30X). Past the right
 *   end of a row the T32 reads on into the next one as the input address
 *   runs; left of it a T32LQ answers from its line buffer's 128 byte
 *   slots, see ldc_left_addr().
 *
 * - On the T32, a job that would take the input from above row 0 or from
 *   row h - 1 or below (either plane), or whose LEN_COEF read burst lengths
 *   (bits 4:0 and 12:8) are 0 or over 15, never ends: WORK_EN stays set,
 *   no status comes, and TIME_OUT (0x78, at the 400 MHz a thingino T32LQ
 *   clocks it at) ends it with status bits 0 and 1. Columns beyond the
 *   picture do not stop it. On the T30 a job never ends in the full view
 *   mode (CTR bit 4 clear); rows two beyond the picture did not stop it.
 *   The model writes no picture for a job that never ends; the parts write
 *   part of one.
 *
 * - The row shift tables (0x1000 and 0x3000; 0x100 and 0x500 on the T30)
 *   and the write burst lengths only schedule which input lines the
 *   engine holds: tables that do not fit the job corrupt both parts'
 *   output in bursts of 8 pixels, and on the T30X stopped jobs that ran
 *   with zeroed tables. The model takes every table as fitting.
 *
 * T32 layout: 0x04 WORK_EN (bit 0: a write of 1 while idle starts a job,
 * reads 1 until it ends, a write of 0 abandons it), 0x08 bit 0 online
 * (from the ISP: not modelled, a job does nothing), 0x10 h << 16 | w,
 * 0x14 k2 << 16 | k1, 0x1c by << 16 | bx (zooms), 0x20 R2, 0x24 UDIS_R,
 * 0x30/0x34 Y in and stride, 0x38/0x3c Y out, 0x40-0x4c the UV plane's,
 * 0x50 LEN_COEF, 0x60 CFG_BUS, 0x68 status (bit 0 done, 1 timeout), 0x6c
 * write 1 to clear, 0x70 mask (the IRQ is status & ~mask), 0x74, 0x78
 * TIME_OUT. UDIS_R, CFG_BUS and 0x74 change nothing seen. 0x64 (RESET)
 * and 0xf84 read 0; on the part 0xf84 reads a byte that varies with the
 * job's data, which the model does not reproduce. VERSION 0x20240103.
 * A T41LQ's is the same engine, its jobs byte for byte a T32LQ's, but for
 * VERSION 0x20211229 and an 0x08 that keeps nothing.
 *
 * T30 layout: 0x00 CTR (bit 0 starts a job and reads 0; bit 1 resets every
 * register but the tables and reads 0, as the CPM's SRBC1 bit 8 does;
 * bit 2 stops a job and reads 0; bit 3 enables the IRQ, raised while
 * status bit 0 is; bits 5:4 kept: view mode, which must be set, and wait
 * for the bus on stop, which changed nothing seen), 0x04 status (bit 0
 * done, write 1 to clear; bits 2:1 read 1 while a job runs, bit 3 once it
 * was stopped; the rest reads 0x30010000), 0x08 h << 16 | w, 0x0c and
 * 0x10 k2 << 16 | k1 for x and for y, 0x14 UDIS_R, 0x18 LINE_LEN, 0x1c
 * the zooms, 0x20 R2, 0x24 the fill values, 0x28 the job's clock count
 * (the model's reads 0), 0x2c-0x38 Y in, UV in, Y out, UV out and
 * 0x3c-0x48 their strides.
 *
 * Reset values and the bits each register keeps are a T32LQ's and a
 * T30X's.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/host-utils.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-ldc.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

/* The largest picture the model corrects, past which a job is dropped */
#define LDC_MAX_PIXELS      (16 * MiB)
#define LDC_TIMEOUT_HZ      400000000

#define ST_DONE             BIT(0)
#define ST_TIMEOUT          BIT(1)
#define ST_RUN              BIT(1)      /* T30: bits 2:1 = 1 */
#define ST_STOPPED          BIT(3)      /* T30 */
#define T30_ST_IDLE         0x30010000

enum { Y_IN, Y_OUT, UV_IN, UV_OUT };

typedef struct LdcPlane {
    uint32_t w, h;
    int64_t k1[2], k2[2], zoom[2], norm;
} LdcPlane;

/* Where along one axis a pixel takes the input from, in Q10 */
static int64_t ldc_pos(const LdcPlane *p, int ax, int64_t d, int64_t e,
                       bool outer, uint32_t c)
{
    uint64_t r = (uint64_t)(d * d + e * e) * p->norm & MAKE_64BIT_MASK(0, 30);
    int64_t f = (INT64_C(1) << 40) - p->k1[ax] * (int64_t)r +
                p->k2[ax] * (int64_t)((r * r) >> 30);
    uint64_t lo, hi, off;

    mulu64(&lo, &hi, d, (uint64_t)f * p->zoom[ax]);
    off = hi << 23 | lo >> 41;
    return outer ? ((int64_t)c << 10) + off : ((int64_t)(c - 1) << 10) - off;
}

static void ldc_plane_setup(const IngenicLdcState *s, LdcPlane *p,
                            uint32_t w, uint32_t h, bool chroma)
{
    for (int ax = 0; ax < 2; ax++) {
        uint32_t k = s->coef[s->t30 ? ax : 0];

        p->k1[ax] = extract32(k, 0, 9);
        p->k2[ax] = extract32(k, 16, 9);
        p->zoom[ax] = extract32(s->zoom, 16 * ax, 12);
    }
    p->norm = (int64_t)s->r2 << (chroma ? 2 : 0);
    p->w = w;
    p->h = h;
}

/* Positions of every pixel of a plane, x then y, in Q10 */
static void ldc_positions(const LdcPlane *p, int32_t *px, int32_t *py)
{
    uint32_t cx = p->w / 2, cy = p->h / 2;

    for (uint32_t y = 0; y < p->h; y++) {
        int64_t e = y >= cy ? y - cy : cy - 1 - y;

        for (uint32_t x = 0; x < p->w; x++) {
            int64_t d = x >= cx ? x - cx : cx - 1 - x;
            size_t i = (size_t)y * p->w + x;

            px[i] = ldc_pos(p, 0, d, e, x >= cx, cx);
            py[i] = ldc_pos(p, 1, e, d, y >= cy, cy);
        }
    }
}

static bool ldc_rows_ok(const int32_t *py, size_t n, uint32_t h)
{
    for (size_t i = 0; i < n; i++) {
        if (py[i] < 0 || py[i] >= (int32_t)((h - 1) << 10)) {
            return false;
        }
    }
    return true;
}

/* The T30's sample outside the plane: its fill value for byte b */
static uint8_t ldc_fill(const IngenicLdcState *s, unsigned bpp, unsigned b)
{
    return extract32(s->fill, bpp == 1 ? 16 : 8 - 8 * b, 8);
}

/*
 * Where the T32's line buffer answers for a pixel left of the picture
 * (ix < 0): the tap sits in a 128 byte slot of the line buffer, (ix * bpp)
 * & 127, of the row the tap is on, holding that row's byte of the same
 * number; the second tap of the pair (dx 1) follows on in the buffer, and
 * from the last slot it runs into the next line, the row after on the
 * picture's upper half and the row before on its lower half (the engine
 * works outwards from the centre). Measured to 8 columns left of the
 * picture; the rows where a line was loaded twice are not reproduced.
 */
static int64_t ldc_left_addr(int32_t ix, int32_t row, int dx, unsigned bpp,
                             unsigned b, uint32_t st, bool lower)
{
    int64_t o = (((int64_t)ix * bpp) & 127) + dx * bpp + b;

    return (int64_t)(row + (o >> 7) * (lower ? -1 : 1)) * st + (o & 127);
}

/*
 * Correct one plane of w x h units of bpp bytes from addr[in] (stride
 * stride[in]) to addr[out]
 */
static void ldc_remap(IngenicLdcState *s, const LdcPlane *p, int in, int out,
                      unsigned bpp, const int32_t *px, const int32_t *py)
{
    uint32_t w = p->w, h = p->h, st = s->stride[in];
    int64_t lo = INT64_MAX, hi = INT64_MIN;
    size_t n = (size_t)w * h, len;
    g_autofree uint8_t *src = NULL;
    g_autofree uint8_t *row = g_malloc((size_t)w * bpp);

    if (s->t30) {
        lo = 0;
        hi = (int64_t)(h - 1) * st + (int64_t)w * bpp;
    } else {
        for (size_t i = 0; i < n; i++) {
            int64_t a = (int64_t)(py[i] >> 10) * st +
                        (int64_t)(px[i] >> 10) * bpp;

            lo = MIN(lo, a - st);
            hi = MAX(hi, a + st + 2 * bpp);
        }
    }
    len = hi - lo;
    if (len > 64 * MiB) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-ldc: %ux%u input too large\n",
                      w, h);
        return;
    }
    src = g_malloc(len);
    address_space_read(&address_space_memory, s->addr[in] + lo,
                       MEMTXATTRS_UNSPECIFIED, src, len);
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            size_t i = (size_t)y * w + x;
            int32_t ix = px[i] >> 10, iy = py[i] >> 10;
            uint32_t fx = px[i] & 1023, fy = py[i] & 1023;
            uint32_t w11 = fx * fy >> 10;
            uint32_t wt[4] = { 1024 - fx - fy + w11, fx - w11, fy - w11, w11 };

            for (unsigned b = 0; b < bpp; b++) {
                uint32_t v = 0;

                for (int k = 0; k < 4; k++) {
                    int32_t sx = ix + (k & 1), sy = iy + (k >> 1);
                    uint8_t q;

                    if (!s->t30 && ix < 0) {
                        q = src[ldc_left_addr(ix, sy, k & 1, bpp, b,
                                              st, y >= h / 2) - lo];
                    } else if (!s->t30) {
                        q = src[(int64_t)sy * st + (int64_t)sx * bpp + b - lo];
                    } else if (sx < 0 || sx >= (int32_t)w || sy < 0 ||
                               sy >= (int32_t)h) {
                        q = ldc_fill(s, bpp, b);
                    } else {
                        q = src[(int64_t)sy * st + (int64_t)sx * bpp + b];
                    }
                    v += q * wt[k];
                }
                row[x * bpp + b] = v >> 10;
            }
        }
        address_space_write(&address_space_memory,
                            s->addr[out] + (uint64_t)y * s->stride[out],
                            MEMTXATTRS_UNSPECIFIED, row, (size_t)w * bpp);
    }
}

static void ldc_update_irq(IngenicLdcState *s)
{
    bool level;

    if (s->t30) {
        level = (s->status & ST_DONE) && (s->ctrl & BIT(3));
    } else {
        level = s->status & ~s->mask & (ST_DONE | ST_TIMEOUT);
    }
    qemu_set_irq(s->irq, level);
}

static void ldc_end(IngenicLdcState *s, uint32_t status)
{
    s->ctrl &= ~BIT(0);
    s->status |= status;
    ldc_update_irq(s);
}

/* A job that never ends: the T32 layout's TIME_OUT ends it */
static void ldc_hang(IngenicLdcState *s)
{
    if (!s->t30) {
        int64_t ns = muldiv64(s->time_out, NANOSECONDS_PER_SECOND,
                              LDC_TIMEOUT_HZ);

        timer_mod(s->timeout, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    }
}

static void ldc_timeout(void *opaque)
{
    IngenicLdcState *s = opaque;

    if (s->ctrl & BIT(0)) {
        ldc_end(s, ST_DONE | ST_TIMEOUT);
    }
}

static void ldc_run(void *opaque)
{
    IngenicLdcState *s = opaque;
    uint32_t w = extract32(s->res, 0, s->t30 ? 12 : 16);
    uint32_t h = extract32(s->res, 16, s->t30 ? 12 : 16);
    uint32_t rd = s->len;
    g_autofree int32_t *px = NULL, *py = NULL, *cpx = NULL, *cpy = NULL;
    LdcPlane yp, cp;

    if (!(s->ctrl & BIT(0))) {
        return;
    }
    if (!s->t30 && (s->mode & 1)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-ldc: online mode\n");
        return;
    }
    if (w < 2 || h < 2 || (size_t)w * h > LDC_MAX_PIXELS) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-ldc: %ux%u picture\n", w, h);
        ldc_hang(s);
        return;
    }
    if (s->t30 && !(s->ctrl & BIT(4))) {
        ldc_hang(s);
        return;
    }
    for (int i = 0; i < 2 && !s->t30; i++) {
        uint32_t b = extract32(rd, 8 * i, 5);

        if (!b || b > 15) {
            ldc_hang(s);
            return;
        }
    }
    ldc_plane_setup(s, &yp, w, h, false);
    ldc_plane_setup(s, &cp, w / 2, h / 2, true);
    px = g_new(int32_t, (size_t)w * h);
    py = g_new(int32_t, (size_t)w * h);
    cpx = g_new(int32_t, (size_t)(w / 2) * (h / 2));
    cpy = g_new(int32_t, (size_t)(w / 2) * (h / 2));
    ldc_positions(&yp, px, py);
    ldc_positions(&cp, cpx, cpy);
    if (!s->t30 && (!ldc_rows_ok(py, (size_t)w * h, h) ||
                     !ldc_rows_ok(cpy, (size_t)(w / 2) * (h / 2), h / 2))) {
        ldc_hang(s);
        return;
    }
    ldc_remap(s, &yp, Y_IN, Y_OUT, 1, px, py);
    ldc_remap(s, &cp, UV_IN, UV_OUT, 2, cpx, cpy);
    ldc_end(s, ST_DONE);
}

static void ldc_start(IngenicLdcState *s)
{
    s->ctrl |= BIT(0);
    qemu_bh_schedule(s->bh);
}

static void ldc_abandon(IngenicLdcState *s)
{
    s->ctrl &= ~BIT(0);
    timer_del(s->timeout);
}

static void ldc_reset_regs(IngenicLdcState *s)
{
    s->ctrl = s->mode = s->status = s->mask = 0;
    s->coef[0] = s->coef[1] = 0;
    s->udis = s->fill = s->ctl74 = 0;
    memset(s->addr, 0, sizeof(s->addr));
    memset(s->stride, 0, sizeof(s->stride));
    if (s->t30) {
        s->res = 0x02d00500;
        s->zoom = s->r2 = s->len = s->cfg = 0;
        s->time_out = 0;
    } else {
        s->res = 0x04380780;
        s->coef[0] = 0x00660133;
        s->zoom = 0x08910891;
        s->r2 = 0x377;
        s->len = 0x030d1010;
        s->cfg = 0x00140120;
        s->time_out = 0xffffffff;
    }
    timer_del(s->timeout);
}

static uint32_t *ldc_addr_reg(IngenicLdcState *s, hwaddr addr, bool *stride)
{
    if (s->t30) {
        static const int order[] = { Y_IN, UV_IN, Y_OUT, UV_OUT };

        if (addr >= 0x2c && addr <= 0x38) {
            *stride = false;
            return &s->addr[order[(addr - 0x2c) / 4]];
        }
        if (addr >= 0x3c && addr <= 0x48) {
            *stride = true;
            return &s->stride[order[(addr - 0x3c) / 4]];
        }
    } else if (addr >= 0x30 && addr <= 0x4c) {
        static const int order[] = { Y_IN, Y_OUT, UV_IN, UV_OUT };
        unsigned i = (addr - 0x30) / 4;

        *stride = i & 1;
        return *stride ? &s->stride[order[i >> 1]] : &s->addr[order[i >> 1]];
    }
    return NULL;
}

static int ldc_lut_index(IngenicLdcState *s, hwaddr addr, unsigned *i)
{
    if (s->t30) {
        if (addr >= 0x100 && addr < 0x900) {
            *i = (addr - 0x100) / 4 % INGENIC_LDC_LUT;
            return addr >= 0x500;
        }
    } else if ((addr & ~0x2fffULL) == 0x1000) {
        /* Each table's 1 KiB repeats through its 4 KiB */
        *i = (addr & 0x3ff) / 4;
        return addr >= 0x3000;
    }
    return -1;
}

static uint64_t ldc_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicLdcState *s = opaque;
    bool stride;
    uint32_t *r = ldc_addr_reg(s, addr, &stride);
    unsigned i;
    int t = ldc_lut_index(s, addr, &i);

    if (r) {
        return *r;
    }
    if (t >= 0) {
        return s->lut[t][i];
    }
    if (s->t30) {
        switch (addr) {
        case 0x00:
            return s->ctrl & ~BIT(0);
        case 0x04:
            return T30_ST_IDLE | s->status | (s->ctrl & BIT(0) ? ST_RUN : 0);
        case 0x08:
            return s->res;
        case 0x0c:
        case 0x10:
            return s->coef[(addr - 0x0c) / 4];
        case 0x14:
            return s->udis;
        case 0x18:
            return s->len;
        case 0x1c:
            return s->zoom;
        case 0x20:
            return s->r2;
        case 0x24:
            return s->fill;
        }
        return 0;
    }
    switch (addr) {
    case 0x00:
        return s->t41 ? 0x20211229 : 0x20240103;
    case 0x04:
        return s->ctrl;
    case 0x08:
        return s->mode;
    case 0x10:
        return s->res;
    case 0x14:
        return s->coef[0];
    case 0x1c:
        return s->zoom;
    case 0x20:
        return s->r2;
    case 0x24:
        return s->udis;
    case 0x50:
        return s->len;
    case 0x60:
        return s->cfg;
    case 0x68:
        return s->status;
    case 0x70:
        return s->mask;
    case 0x74:
        return s->ctl74;
    case 0x78:
        return s->time_out;
    }
    return 0;
}

static void ldc_write_t30(IngenicLdcState *s, hwaddr addr, uint32_t val)
{
    switch (addr) {
    case 0x00:
        if (val & BIT(1)) {
            ldc_reset_regs(s);
            ldc_update_irq(s);
            break;
        }
        s->ctrl = (s->ctrl & BIT(0)) | (val & 0x38);
        if (val & BIT(2)) {
            if (s->ctrl & BIT(0)) {
                s->status |= ST_STOPPED;
            }
        } else if (val & BIT(0) && !(s->ctrl & BIT(0))) {
            ldc_start(s);
        }
        ldc_update_irq(s);
        break;
    case 0x04:
        s->status &= ~(val & ST_DONE);
        ldc_update_irq(s);
        break;
    case 0x08:
        s->res = val & 0x0fff0fff;
        break;
    case 0x0c:
    case 0x10:
        s->coef[(addr - 0x0c) / 4] = val & 0x01ff01ff;
        break;
    case 0x14:
        s->udis = val & 0x003fffff;
        break;
    case 0x18:
        s->len = val & 0x0f0f0f0f;
        break;
    case 0x1c:
        s->zoom = val & 0x0fff0fff;
        break;
    case 0x20:
        s->r2 = val & 0x1fff;
        break;
    case 0x24:
        s->fill = val & 0x00ffffff;
        break;
    }
}

static void ldc_write_t32(IngenicLdcState *s, hwaddr addr, uint32_t val)
{
    switch (addr) {
    case 0x04:
        if (!(val & 1)) {
            ldc_abandon(s);
        } else if (!(s->ctrl & 1)) {
            ldc_start(s);
        }
        break;
    case 0x08:
        s->mode = s->t41 ? 0 : val & 1;
        break;
    case 0x10:
        s->res = val;
        break;
    case 0x14:
        s->coef[0] = val & 0x01ff01ff;
        break;
    case 0x1c:
        s->zoom = val & 0x0fff0fff;
        break;
    case 0x20:
        s->r2 = val & 0x1fff;
        break;
    case 0x24:
        s->udis = val & 0x003fffff;
        break;
    case 0x50:
        s->len = val & 0x1f1f1f1f;
        break;
    case 0x60:
        s->cfg = val & 0x00ff0137;
        break;
    case 0x6c:
        s->status &= ~val;
        ldc_update_irq(s);
        break;
    case 0x70:
        s->mask = val;
        ldc_update_irq(s);
        break;
    case 0x74:
        s->ctl74 = val & 1;
        break;
    case 0x78:
        s->time_out = val;
        break;
    }
}

static void ldc_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IngenicLdcState *s = opaque;
    bool stride;
    uint32_t *r = ldc_addr_reg(s, addr, &stride);
    unsigned i;
    int t = ldc_lut_index(s, addr, &i);

    if (r) {
        *r = stride ? val & 0xffff : val;
    } else if (t >= 0) {
        s->lut[t][i] = val & (s->t30 ? 0x3ff : 0x7ff);
    } else if (s->t30) {
        ldc_write_t30(s, addr, val);
    } else {
        ldc_write_t32(s, addr, val);
    }
}

static const MemoryRegionOps ldc_ops = {
    .read = ldc_read,
    .write = ldc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ldc_reset_hold(Object *obj, ResetType type)
{
    IngenicLdcState *s = INGENIC_LDC(obj);

    ldc_reset_regs(s);
    memset(s->lut, 0, sizeof(s->lut));
    qemu_set_irq(s->irq, 0);
}

static int ldc_post_load(void *opaque, int version_id)
{
    IngenicLdcState *s = opaque;

    if (s->ctrl & BIT(0)) {
        qemu_bh_schedule(s->bh);
    }
    return 0;
}

static const VMStateDescription vmstate_ldc = {
    .name = TYPE_INGENIC_LDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ldc_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IngenicLdcState),
        VMSTATE_UINT32(mode, IngenicLdcState),
        VMSTATE_UINT32(status, IngenicLdcState),
        VMSTATE_UINT32(mask, IngenicLdcState),
        VMSTATE_UINT32(res, IngenicLdcState),
        VMSTATE_UINT32_ARRAY(coef, IngenicLdcState, 2),
        VMSTATE_UINT32(zoom, IngenicLdcState),
        VMSTATE_UINT32(r2, IngenicLdcState),
        VMSTATE_UINT32(udis, IngenicLdcState),
        VMSTATE_UINT32(len, IngenicLdcState),
        VMSTATE_UINT32(fill, IngenicLdcState),
        VMSTATE_UINT32(cfg, IngenicLdcState),
        VMSTATE_UINT32(ctl74, IngenicLdcState),
        VMSTATE_UINT32(time_out, IngenicLdcState),
        VMSTATE_UINT32_ARRAY(addr, IngenicLdcState, 4),
        VMSTATE_UINT32_ARRAY(stride, IngenicLdcState, 4),
        VMSTATE_UINT32_2DARRAY(lut, IngenicLdcState, 2, INGENIC_LDC_LUT),
        VMSTATE_TIMER_PTR(timeout, IngenicLdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void ldc_realize(DeviceState *dev, Error **errp)
{
    IngenicLdcState *s = INGENIC_LDC(dev);

    if (!s->model || !strcmp(s->model, "t32") || !strcmp(s->model, "t41")) {
        s->t30 = false;
        s->t41 = s->model && !strcmp(s->model, "t41");
    } else if (!strcmp(s->model, "t30")) {
        s->t30 = true;
    } else {
        error_setg(errp, "ingenic-ldc: unknown model '%s'", s->model);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(dev), &ldc_ops, s,
                          TYPE_INGENIC_LDC, s->t30 ? 4 * KiB : 64 * KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

/* The CPM's reset (the T30's SRBC1 bit 8), as CTR bit 1 */
static void ldc_cpm_reset(void *opaque, int n, int level)
{
    IngenicLdcState *s = opaque;

    if (level) {
        ldc_reset_regs(s);
        ldc_update_irq(s);
    }
}

static void ldc_init(Object *obj)
{
    IngenicLdcState *s = INGENIC_LDC(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), ldc_cpm_reset, "reset", 1);
    s->bh = qemu_bh_new(ldc_run, s);
    s->timeout = timer_new_ns(QEMU_CLOCK_VIRTUAL, ldc_timeout, s);
}

static const Property ldc_properties[] = {
    DEFINE_PROP_STRING("model", IngenicLdcState, model),
};

static void ldc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic LDC lens distortion correction engine";
    dc->vmsd = &vmstate_ldc;
    dc->realize = ldc_realize;
    device_class_set_props(dc, ldc_properties);
    rc->phases.hold = ldc_reset_hold;
}

static const TypeInfo ldc_info = {
    .name = TYPE_INGENIC_LDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicLdcState),
    .instance_init = ldc_init,
    .class_init = ldc_class_init,
};

static void ldc_register_types(void)
{
    type_register_static(&ldc_info);
}

type_init(ldc_register_types)
