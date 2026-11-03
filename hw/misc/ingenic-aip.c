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
 *   P (+0x300): affine and perspective warps.
 *
 * T's arithmetic, its order of lines and F's copy at 1:1 are a T41NQ's,
 * byte for byte, on the frames Ingenic's personvehiclepetDet sample sends
 * it and on pictures programmed directly. Not modelled yet: F at other
 * scales (plain bilinear here; a T41NQ's luma at 0.75 differs by one in
 * 22 of 288 bytes, its chroma more), F's formats other than NV12 and Y,
 * T's inputs other than NV12, the chains of T and P, P's warps (it
 * finishes without output), the ISUM/OSUM checksums, and what CTRL reads
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

static void aip_t_step(IngenicAipState *s)
{
    uint32_t lines = reg(s, AIP_T, T_TASK_LEN);
    uint32_t width = extract32(reg(s, AIP_T, T_SRC_SIZE), 0, 16);
    uint32_t height = extract32(reg(s, AIP_T, T_SRC_SIZE), 16, 16);
    uint32_t sstride = extract32(reg(s, AIP_T, T_STRIDE), 0, 16);
    uint32_t dstride = extract32(reg(s, AIP_T, T_STRIDE), 16, 16);
    uint32_t offset = reg(s, AIP_T, T_OFFSET);
    unsigned order = extract32(offset, 24, 4);
    int32_t yoff = extract32(offset, 0, 8), uvoff = extract32(offset, 8, 8);
    uint8_t alpha = extract32(offset, 16, 8);
    int64_t p[9];
    g_autofree uint8_t *y = g_malloc(width);
    g_autofree uint8_t *uv = g_malloc(width + 1);
    g_autofree uint8_t *out = g_malloc(width * 4);

    if ((order & 7) >= ARRAY_SIZE(t_order)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: T order code %u\n", order);
        order = 5;
    }
    for (int i = 0; i < 9; i++) {
        p[i] = (int32_t)reg(s, AIP_T, T_PARAM0 + 4 * i);
    }
    for (uint32_t l = 0; l < lines && s->t_row < height; l++, s->t_row++) {
        phys_read(reg(s, AIP_T, T_SRC_Y) + s->t_row * sstride, y, width);
        phys_read(reg(s, AIP_T, T_SRC_UV) + s->t_row / 2 * sstride, uv, width);
        for (uint32_t x = 0; x < width; x++) {
            int64_t yv = y[x] - yoff;
            int64_t u = uv[x & ~1] - uvoff, v = uv[x | 1] - uvoff;
            uint8_t bgr[3];
            uint8_t *o = &out[4 * x];

            for (int c = 0; c < 3; c++) {
                int64_t t = p[3 * c] * yv + p[3 * c + 1] * u + p[3 * c + 2] * v;

                bgr[c] = MIN(MAX((t + (1 << 19)) >> 20, 0), 255);
            }
            if (order & 8) {
                *o++ = alpha;
            }
            for (int c = 0; c < 3; c++) {
                *o++ = bgr[t_order[order & 7][c]];
            }
            if (!(order & 8)) {
                *o = alpha;
            }
        }
        phys_write(reg(s, AIP_T, T_DST) + l * dstride, out, width * 4);
    }
    aip_done(s, AIP_T);
}

/* One plane of F's job: 16.16 steps, bilinear (exact at 1:1) */
static void aip_f_plane(IngenicAipState *s, hwaddr src, hwaddr dst,
                        uint32_t sw, uint32_t sh, uint32_t dw, uint32_t dh,
                        uint32_t sstride, uint32_t dstride, int bpp)
{
    uint32_t sx = reg(s, AIP_F, F_SCALE_X), sy = reg(s, AIP_F, F_SCALE_Y);
    g_autofree uint8_t *in = g_malloc((size_t)sstride * sh);
    g_autofree uint8_t *out = g_malloc((size_t)dw * bpp);

    phys_read(src, in, (hwaddr)sstride * sh);
    for (uint32_t j = 0; j < dh; j++) {
        uint64_t fy = (uint64_t)j * sy;
        uint32_t y0 = MIN(fy >> 16, sh - 1), y1 = MIN(y0 + 1, sh - 1);
        uint32_t wy = fy & 0xffff;

        for (uint32_t i = 0; i < dw; i++) {
            uint64_t fx = (uint64_t)i * sx;
            uint32_t x0 = MIN(fx >> 16, sw - 1), x1 = MIN(x0 + 1, sw - 1);
            uint32_t wx = fx & 0xffff;

            for (int c = 0; c < bpp; c++) {
                uint64_t a = in[y0 * sstride + x0 * bpp + c];
                uint64_t b = in[y0 * sstride + x1 * bpp + c];
                uint64_t d = in[y1 * sstride + x0 * bpp + c];
                uint64_t e = in[y1 * sstride + x1 * bpp + c];
                uint64_t top = a * (0x10000 - wx) + b * wx;
                uint64_t bot = d * (0x10000 - wx) + e * wx;

                out[i * bpp + c] = (top * (0x10000 - wy) + bot * wy +
                                    (1ULL << 31)) >> 32;
            }
        }
        phys_write(dst + j * dstride, out, (hwaddr)dw * bpp);
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
    if (reg(s, AIP_F, F_SCALE_X) != 0x10000 ||
        reg(s, AIP_F, F_SCALE_Y) != 0x10000) {
        qemu_log_mask(LOG_UNIMP, "ingenic-aip: F scaled (bilinear, not "
                      "measured)\n");
    }
    aip_f_plane(s, reg(s, AIP_F, F_SRC_Y), reg(s, AIP_F, F_DST_Y), sw, sh, dw,
                dh, sstride, dstride, 1);
    if (fmt == F_FMT_NV12) {
        aip_f_plane(s, reg(s, AIP_F, F_SRC_UV), reg(s, AIP_F, F_DST_UV),
                    sw / 2, sh / 2, dw / 2, dh / 2, sstride, dstride, 2);
    }
}

/* F's chain: (value, register address) pairs; address bit 31 ends a job */
static void aip_f_chain(IngenicAipState *s)
{
    uint32_t base = reg(s, AIP_F, F_CHAIN_BASE);
    uint32_t size = reg(s, AIP_F, F_CHAIN_SIZE);

    for (uint32_t o = 0; o + 8 <= size; o += 8) {
        uint32_t pair[2];
        hwaddr off;

        phys_read(base + o, pair, sizeof(pair));
        off = extract32(le32_to_cpu(pair[1]), 0, 12) - unit_base[AIP_F];
        if (off < INGENIC_AIP_UNIT_REGS * 4 && off != REG_CTRL) {
            s->regs[AIP_F][off / 4] = le32_to_cpu(pair[0]);
        }
        if (le32_to_cpu(pair[1]) & CHAIN_LAST) {
            aip_f_job(s);
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
            aip_f_chain(s);
            aip_done(s, AIP_F);
        } else if (value & CTRL_RUN) {
            aip_f_job(s);
            aip_done(s, AIP_F);
        }
        break;
    default:
        if (value & (CTRL_RUN | CTRL_CHAIN)) {
            qemu_log_mask(LOG_UNIMP, "ingenic-aip: P's warps\n");
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
