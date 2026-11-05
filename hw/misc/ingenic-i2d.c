/*
 * Ingenic I2D, the 2D rotate/mirror/flip engine (T32, T33, T40, T41)
 *
 * Copies a picture from SRC_ADDR to DST_ADDR, turned as IMG_MODE says:
 * bits 5:4 the data type (0 NV12, 1 RAW8, 2 RGB565, 3 ARGB8888), 3:2 the
 * rotation (0, 90, 180 or 270 degrees clockwise), bit 1 mirror and bit 0
 * flip, both applied to the rotated picture. IMG_SIZE holds the source's
 * width in bits 31:16 and height in 15:0, the stride registers each
 * plane's line length in bytes. NV12 turns its UV plane on the half-size
 * grid, one U/V pair at a time; the others move whole pixels of 1, 2 or
 * 4 bytes. Each plane is written where its registers say, so planes the
 * driver places over one another overwrite as on the part.
 *
 * Starting (CTRL bit 0) runs the job, clears SHD_CTRL, sets IRQ_STATE
 * bit 0 and leaves the job's turn in DBG bits 10:8; IRQ_CLEAR clears
 * state bits and IRQ_MASK masks them. CTRL's reset bits (4 and 16) clear
 * nothing a register shows. As a T41NQ, T40XP and T32NQ read them, the
 * same on all three, and as their jobs come out, byte for byte alike;
 * the T33's is unmeasured. VERSION 0x20191031,
 * IMG_SIZE 0x028001e0, TIMEOUT_VALUE 0xffffffff, TIMEOUT_MODE 1,
 * CLK_GATE 1 and DBG 0x7001 out of reset, and the widths each register
 * keeps of a write. Not modelled: the timeout (IRQ_STATE bit 1), what
 * IMG_MODE bit 8 does and the time a job takes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/units.h"
#include "hw/core/irq.h"
#include "hw/misc/ingenic-i2d.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#define I2D_VERSION     0x00
#define I2D_SHD_CTRL    0x08
#define I2D_CTRL        0x10
#define I2D_IMG_SIZE    0x14
#define I2D_IMG_MODE    0x18
#define I2D_SRC_ADDR_Y  0x20
#define I2D_SRC_Y_STRID 0x28
#define I2D_DST_ADDR_Y  0x30
#define I2D_DST_Y_STRID 0x38
#define I2D_IRQ_STATE   0x80
#define I2D_IRQ_CLEAR   0x84
#define I2D_IRQ_MASK    0x88
#define I2D_TIMEOUT_VAL 0x90
#define I2D_TIMEOUT_MOD 0x94
#define I2D_CLK_GATE    0x98
#define I2D_DBG         0xa0

#define CTRL_START      BIT(0)
#define IRQ_DONE        BIT(0)
#define MODE_RW         0x13f
#define DBG_IDLE        0x7001

/* The largest plane the model turns; the T41's 16-bit sizes allow more */
#define I2D_MAX_PLANE   (64 * MiB)

static void i2d_update_irq(IngenicI2dState *s)
{
    qemu_set_irq(s->irq, s->irq_state & ~s->irq_mask & 3);
}

/*
 * Turn one plane of pw x ph units of unit bytes. The source is read
 * whole before the destination is written.
 */
static void i2d_plane(IngenicI2dState *s, int src, int dst, uint32_t pw,
                      uint32_t ph, uint32_t unit)
{
    uint32_t rot = extract32(s->img_mode, 2, 2);
    bool mirr = extract32(s->img_mode, 1, 1);
    bool flip = extract32(s->img_mode, 0, 1);
    uint32_t sstride = s->stride[src], dstride = s->stride[dst];
    uint32_t ow = rot & 1 ? ph : pw, oh = rot & 1 ? pw : ph;
    uint64_t slen = (uint64_t)(ph - 1) * sstride + (uint64_t)pw * unit;
    g_autofree uint8_t *in = NULL;
    g_autofree uint8_t *row = NULL;

    if (!pw || !ph) {
        return;
    }
    if (slen > I2D_MAX_PLANE ||
        (uint64_t)(oh - 1) * dstride + (uint64_t)ow * unit > I2D_MAX_PLANE) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-i2d: %ux%u plane too large\n",
                      pw, ph);
        return;
    }
    in = g_malloc(slen);
    row = g_malloc((size_t)ow * unit);
    address_space_read(&address_space_memory, s->addr[src],
                       MEMTXATTRS_UNSPECIFIED, in, slen);
    for (uint32_t dy = 0; dy < oh; dy++) {
        for (uint32_t dx = 0; dx < ow; dx++) {
            uint32_t ux = mirr ? ow - 1 - dx : dx;
            uint32_t uy = flip ? oh - 1 - dy : dy;
            uint32_t sx, sy;

            switch (rot) {
            case 0:
                sx = ux;
                sy = uy;
                break;
            case 1:
                sx = uy;
                sy = ph - 1 - ux;
                break;
            case 2:
                sx = pw - 1 - ux;
                sy = ph - 1 - uy;
                break;
            default:
                sx = pw - 1 - uy;
                sy = ux;
                break;
            }
            memcpy(row + dx * unit, in + (uint64_t)sy * sstride + sx * unit,
                   unit);
        }
        address_space_write(&address_space_memory,
                            s->addr[dst] + (uint64_t)dy * dstride,
                            MEMTXATTRS_UNSPECIFIED, row, (size_t)ow * unit);
    }
}

static void i2d_run(void *opaque)
{
    IngenicI2dState *s = opaque;
    static const uint8_t unit[] = { 1, 1, 2, 4 };
    static const uint8_t turn[] = { 0, 7, 6, 1 };  /* DBG code per rotation */
    uint32_t fmt = extract32(s->img_mode, 4, 2);
    uint32_t w = extract32(s->img_size, 16, 16);
    uint32_t h = extract32(s->img_size, 0, 16);
    uint32_t code = turn[extract32(s->img_mode, 2, 2)];

    i2d_plane(s, 0, 2, w, h, unit[fmt]);
    if (fmt == 0) {
        i2d_plane(s, 1, 3, w / 2, h / 2, 2);
    }
    code ^= (s->img_mode & 2 ? 4 : 0) ^ (s->img_mode & 1 ? 2 : 0);
    s->dbg = DBG_IDLE | code << 8;
    s->ctrl &= ~CTRL_START;
    s->irq_state |= IRQ_DONE;
    i2d_update_irq(s);
}

static uint64_t i2d_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicI2dState *s = opaque;

    switch (addr) {
    case I2D_VERSION:
        return 0x20191031;
    case I2D_SHD_CTRL:
        return s->shd_ctrl;
    case I2D_CTRL:
        return s->ctrl;
    case I2D_IMG_SIZE:
        return s->img_size;
    case I2D_IMG_MODE:
        return s->img_mode;
    case I2D_SRC_ADDR_Y ... I2D_SRC_ADDR_Y + 4:
        return s->addr[(addr - I2D_SRC_ADDR_Y) / 4];
    case I2D_SRC_Y_STRID ... I2D_SRC_Y_STRID + 4:
        return s->stride[(addr - I2D_SRC_Y_STRID) / 4];
    case I2D_DST_ADDR_Y ... I2D_DST_ADDR_Y + 4:
        return s->addr[2 + (addr - I2D_DST_ADDR_Y) / 4];
    case I2D_DST_Y_STRID ... I2D_DST_Y_STRID + 4:
        return s->stride[2 + (addr - I2D_DST_Y_STRID) / 4];
    case I2D_IRQ_STATE:
        return s->irq_state;
    case I2D_IRQ_MASK:
        return s->irq_mask;
    case I2D_TIMEOUT_VAL:
        return s->timeout_value;
    case I2D_TIMEOUT_MOD:
        return s->timeout_mode;
    case I2D_CLK_GATE:
        return s->clk_gate;
    case I2D_DBG:
        return s->dbg;
    default:
        return 0;
    }
}

static void i2d_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IngenicI2dState *s = opaque;

    switch (addr) {
    case I2D_SHD_CTRL:
        s->shd_ctrl = val & 3;
        break;
    case I2D_CTRL:
        if (val & CTRL_START && !(s->ctrl & CTRL_START)) {
            s->ctrl |= CTRL_START;
            s->shd_ctrl = 0;
            qemu_bh_schedule(s->bh);
        }
        break;
    case I2D_IMG_SIZE:
        s->img_size = val;
        break;
    case I2D_IMG_MODE:
        s->img_mode = val & MODE_RW;
        break;
    case I2D_SRC_ADDR_Y ... I2D_SRC_ADDR_Y + 4:
        s->addr[(addr - I2D_SRC_ADDR_Y) / 4] = val;
        break;
    case I2D_SRC_Y_STRID ... I2D_SRC_Y_STRID + 4:
        s->stride[(addr - I2D_SRC_Y_STRID) / 4] = val & 0xffff;
        break;
    case I2D_DST_ADDR_Y ... I2D_DST_ADDR_Y + 4:
        s->addr[2 + (addr - I2D_DST_ADDR_Y) / 4] = val;
        break;
    case I2D_DST_Y_STRID ... I2D_DST_Y_STRID + 4:
        s->stride[2 + (addr - I2D_DST_Y_STRID) / 4] = val & 0xffff;
        break;
    case I2D_IRQ_CLEAR:
        s->irq_state &= ~val;
        i2d_update_irq(s);
        break;
    case I2D_IRQ_MASK:
        s->irq_mask = val & 3;
        i2d_update_irq(s);
        break;
    case I2D_TIMEOUT_VAL:
        s->timeout_value = val;
        break;
    case I2D_TIMEOUT_MOD:
        s->timeout_mode = val & 1;
        break;
    case I2D_CLK_GATE:
        s->clk_gate = val & 1;
        break;
    case I2D_VERSION:
    case I2D_IRQ_STATE:
    case I2D_DBG:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-i2d: write to 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps i2d_ops = {
    .read = i2d_read,
    .write = i2d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void i2d_reset_hold(Object *obj, ResetType type)
{
    IngenicI2dState *s = INGENIC_I2D(obj);

    s->shd_ctrl = s->ctrl = s->img_mode = 0;
    s->img_size = 0x028001e0;
    memset(s->addr, 0, sizeof(s->addr));
    memset(s->stride, 0, sizeof(s->stride));
    s->irq_state = s->irq_mask = 0;
    s->timeout_value = 0xffffffff;
    s->timeout_mode = 1;
    s->clk_gate = 1;
    s->dbg = DBG_IDLE;
    qemu_set_irq(s->irq, 0);
}

/* A job started and not yet run */
static int i2d_post_load(void *opaque, int version_id)
{
    IngenicI2dState *s = opaque;

    if (s->ctrl & CTRL_START) {
        qemu_bh_schedule(s->bh);
    }
    return 0;
}

static const VMStateDescription vmstate_i2d = {
    .name = TYPE_INGENIC_I2D,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = i2d_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(shd_ctrl, IngenicI2dState),
        VMSTATE_UINT32(ctrl, IngenicI2dState),
        VMSTATE_UINT32(img_size, IngenicI2dState),
        VMSTATE_UINT32(img_mode, IngenicI2dState),
        VMSTATE_UINT32_ARRAY(addr, IngenicI2dState, 4),
        VMSTATE_UINT32_ARRAY(stride, IngenicI2dState, 4),
        VMSTATE_UINT32(irq_state, IngenicI2dState),
        VMSTATE_UINT32(irq_mask, IngenicI2dState),
        VMSTATE_UINT32(timeout_value, IngenicI2dState),
        VMSTATE_UINT32(timeout_mode, IngenicI2dState),
        VMSTATE_UINT32(clk_gate, IngenicI2dState),
        VMSTATE_UINT32(dbg, IngenicI2dState),
        VMSTATE_END_OF_LIST()
    }
};

static void i2d_init(Object *obj)
{
    IngenicI2dState *s = INGENIC_I2D(obj);

    memory_region_init_io(&s->iomem, obj, &i2d_ops, s, TYPE_INGENIC_I2D,
                          0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bh = qemu_bh_new(i2d_run, s);
}

static void i2d_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic I2D rotate/mirror/flip engine";
    dc->vmsd = &vmstate_i2d;
    rc->phases.hold = i2d_reset_hold;
}

static const TypeInfo i2d_info = {
    .name = TYPE_INGENIC_I2D,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicI2dState),
    .instance_init = i2d_init,
    .class_init = i2d_class_init,
};

static void i2d_register_types(void)
{
    type_register_static(&i2d_info);
}

type_init(i2d_register_types)
