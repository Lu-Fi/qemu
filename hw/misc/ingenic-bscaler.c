/*
 * Ingenic BSCALER: the T40's block scaler
 *
 * One block at 0x13090000 with the LZMA decompressor: while the LZMA's
 * CTRL bit 31 is set, offsets 0x00-0x1f are the LZMA's (ingenic-lzma);
 * otherwise they are FRMC's. Two units, each with its interrupt:
 *   FRMC (INTC 39): warps BGRA tiles. Its registers, loaded directly or
 *     from a chain of (value, offset) pairs whose offset bit 31 ends a
 *     job: 0x04 MODE (0xf: 4-byte pixels), 0x08 SRC, 0x10 SRC_SIZE
 *     (h << 16 | w), 0x14 SRC_STRIDE, 0x18 DST, 0x1c DST_SIZE, 0x20
 *     DST_STRIDE, 0x40/0x44 the source and destination tiles' origins
 *     (y << 16 | x) in their pictures, 0x48-0x68 a 3x3 matrix in 16.16
 *     from destination to source. 0x00 bit 0 runs a job (a chain ends each
 *     with 0x201 there); 0x30 bit 0 runs the chain at 0x34 (0x38 bytes)
 *     with bit 5, bit 16 resets; 0x3c is a timeout. A finished run sets
 *     0x00 bit 8 and 0x30 bit 4, which raise the interrupt until cleared.
 *   FRMT (INTC 40): 0xb0 its control (bit 16 resets), 0xc0-0xe0 BT.601
 *     parameters, 0xe4 its mode. Not modelled beyond the reset.
 *
 * FRMC's pixels are a T40XP's byte for byte: the source position of each
 * destination pixel is the matrix at (pixel + destination origin), less
 * the source origin; B, G and R are blended across with 16-bit weights,
 * (a * (65535 - w) + b * w + 32768) >> 16, then down the same way; a
 * neighbour outside the source tile is 0, and alpha comes out 0. Not
 * modelled yet: modes other than 0xf, the perspective division's own
 * precision (exact division here; a T40XP's differs in about 1% of the
 * bytes), the tile shapes a T40XP declines to write, FRMT's conversion,
 * and the port at 0x28/0x2c (16 words libvenus loads, with no effect seen
 * in mode 0xf).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-bscaler.h"

#define FRMC_STAT           0x00
#define FRMC_MODE           0x04
#define FRMC_SRC            0x08
#define FRMC_SRC_SIZE       0x10
#define FRMC_SRC_STRIDE     0x14
#define FRMC_DST            0x18
#define FRMC_DST_SIZE       0x1c
#define FRMC_DST_STRIDE     0x20
#define FRMC_CTRL           0x30
#define FRMC_CHAIN_BASE     0x34
#define FRMC_CHAIN_SIZE     0x38
#define FRMC_SRC_ORIGIN     0x40
#define FRMC_DST_ORIGIN     0x44
#define FRMC_MATRIX         0x48
#define FRMT_CTRL           0xb0

#define LZMA_MODE           BIT(31)
#define RUN                 BIT(0)
#define CHAIN               BIT(5)
#define SOFT_RESET          BIT(16)
#define STAT_DONE           BIT(8)
#define CTRL_DONE           BIT(4)
#define CHAIN_LAST          BIT(31)

#define MODE_BGRA           0xf
#define MAX_SIDE            4096

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

static void frmc_job(IngenicBscalerState *s)
{
    uint32_t sw = extract32(reg(s, FRMC_SRC_SIZE), 0, 16);
    uint32_t sh = extract32(reg(s, FRMC_SRC_SIZE), 16, 16);
    uint32_t dw = extract32(reg(s, FRMC_DST_SIZE), 0, 16);
    uint32_t dh = extract32(reg(s, FRMC_DST_SIZE), 16, 16);
    uint32_t sstride = reg(s, FRMC_SRC_STRIDE);
    uint32_t dstride = reg(s, FRMC_DST_STRIDE);
    g_autofree uint8_t *src = NULL;
    g_autofree uint8_t *row = NULL;

    if (reg(s, FRMC_MODE) != MODE_BGRA) {
        qemu_log_mask(LOG_UNIMP, "ingenic-bscaler: FRMC mode 0x%x\n",
                      reg(s, FRMC_MODE));
        return;
    }
    if (!sw || !sh || !dw || !dh || sw > MAX_SIDE || sh > MAX_SIDE ||
        dw > MAX_SIDE || dh > MAX_SIDE || sstride < sw * 4) {
        return;
    }
    src = g_malloc((size_t)sstride * sh);
    row = g_malloc((size_t)dw * 4);
    address_space_read(&address_space_memory, reg(s, FRMC_SRC),
                       MEMTXATTRS_UNSPECIFIED, src, (hwaddr)sstride * sh);
    for (uint32_t j = 0; j < dh; j++) {
        for (uint32_t i = 0; i < dw; i++) {
            int64_t x, y, x0, y0;
            unsigned wx, wy;

            frmc_pos(s, i, j, &x, &y);
            x0 = x >> 16;
            y0 = y >> 16;
            wx = x & 0xffff;
            wy = y & 0xffff;
            for (int c = 0; c < 3; c++) {
                unsigned px[4];

                for (int k = 0; k < 4; k++) {
                    int64_t xx = x0 + (k & 1), yy = y0 + (k >> 1);

                    px[k] = x != INT64_MIN && xx >= 0 && xx < sw && yy >= 0 &&
                            yy < sh ? src[yy * sstride + xx * 4 + c] : 0;
                }
                row[i * 4 + c] = frmc_lerp(frmc_lerp(px[0], px[1], wx),
                                           frmc_lerp(px[2], px[3], wx), wy);
            }
            row[i * 4 + 3] = 0;
        }
        address_space_write(&address_space_memory,
                            reg(s, FRMC_DST) + (hwaddr)j * dstride,
                            MEMTXATTRS_UNSPECIFIED, row, (hwaddr)dw * 4);
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

static void frmc_write(IngenicBscalerState *s, hwaddr off, uint32_t v)
{
    switch (off) {
    case FRMC_STAT:
        s->regs[0] = v & ~RUN;
        if (v & RUN) {
            frmc_job(s);
        }
        break;
    case FRMC_CTRL:
        if (v & SOFT_RESET) {
            uint32_t stat = s->regs[0];  /* a T40XP keeps 0x00 */

            memset(s->regs, 0, FRMT_CTRL);
            s->regs[0] = stat & ~STAT_DONE;
            break;
        }
        s->regs[off / 4] = v & ~RUN;
        if ((v & RUN) && (v & CHAIN)) {
            frmc_chain(s);
            s->regs[0] |= STAT_DONE;
            s->regs[off / 4] |= CTRL_DONE;
        } else if (v & RUN) {
            frmc_job(s);
            s->regs[0] |= STAT_DONE;
            s->regs[off / 4] |= CTRL_DONE;
        }
        break;
    case FRMT_CTRL:
        if (v & SOFT_RESET) {
            memset(&s->regs[FRMT_CTRL / 4], 0,
                   sizeof(s->regs) - FRMT_CTRL);
            break;
        }
        s->regs[off / 4] = v & ~RUN;
        if (v & RUN) {
            qemu_log_mask(LOG_UNIMP, "ingenic-bscaler: FRMT\n");
        }
        break;
    default:
        if (off < sizeof(s->regs)) {
            s->regs[off / 4] = v;
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

    memset(s->regs, 0, sizeof(s->regs));
    bscaler_update(s);
}

static const VMStateDescription vmstate_bscaler = {
    .name = TYPE_INGENIC_BSCALER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicBscalerState,
                             INGENIC_BSCALER_IOSIZE / 4),
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
