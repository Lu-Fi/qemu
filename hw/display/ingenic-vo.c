/*
 * Ingenic VO: the T40, T41 and A1's BT.656/BT.1120 video output
 *
 * One channel sends NV12 frames from memory: VC0_SIZE (height in 27:16,
 * width in 11:0) and VC0_STRIDE (UV in 31:16, Y in 15:0) describe a frame,
 * VC0_VBLK_PAR its lines of blanking after (27:16) and before (11:0) and
 * VC0_HBLK_PAR the clocks of blanking on each line. A frame is h + vfb +
 * vbb lines of 2w + hfb + 8 clocks (+ 16 on the T40 and A1) at the VO
 * clock (the CPM's BT0CDR).
 *
 * Frames come from a queue of four address pairs: a write to VC0_Y_ADDR
 * (which reads 0) waits for one to VC0_UV_ADDR, which queues the pair;
 * ADDR_INFO bit 3 says the queue is full, and later pairs are dropped.
 * With TRAN_EN set the VO takes a pair as it starts, and the next at each
 * frame's end, setting INT_VAL bit 0 each time. If none is queued at a
 * frame's end the T41's sends the frame again, and at the next sets bit 16
 * (read in ADDR_INFO bit 7 too) and stops; the T40's and A1's stop at once
 * and raise nothing. INT_VAL bits held in INT_CLC are cleared and cannot
 * set; the interrupt is INT_VAL & INT_EN.
 *
 * TOP_CON bit 1 resets the channel (TRAN_EN, the queue, INT_VAL). On the
 * T40 and A1 it reads inverted, and while it is set no other register
 * takes a write. Each frame the VO takes is shown on a QEMU console.
 *
 * All as a T41NQ, T40XP and A1N do at U-Boot, the timing to within 0.3%
 * for frames 256 or more wide. Not modelled: ADDR_INFO's other bits while
 * sending, INT_VAL bit 20 (lines too short to fetch), the short frames of
 * very narrow pictures, the blanking and clip values on the wire.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "hw/display/ingenic-vo.h"

#define VO_VER              0x000
#define VO_TOP_CON          0x004
#define VO_TOP_STATUS       0x008
#define VO_INT_EN           0x00c
#define VO_INT_VAL          0x010
#define VO_INT_CLC          0x014
#define VO_SIZE             0x100
#define VO_STRIDE           0x104
#define VO_Y_ADDR           0x108
#define VO_UV_ADDR          0x10c
#define VO_BLK_INFO         0x110
#define VO_CON_PAR          0x114
#define VO_VBLK_PAR         0x118
#define VO_HBLK_PAR         0x11c
#define VO_TRAN_EN          0x120
#define VO_TIME_OUT         0x124
#define VO_ADDR_INFO        0x128
#define VO_YUV_CLIP         0x130

#define TOP_CON_RESET       (1 << 1)
#define INT_TAKEN           (1 << 0)
#define INT_UNDERRUN        (1 << 16)
#define ADDR_FULL           (1 << 3)
#define ADDR_UNDERRUN       (1 << 7)

static bool vo_in_reset(IngenicVoState *s)
{
    return s->top_con & TOP_CON_RESET;
}

static void vo_update_irq(IngenicVoState *s)
{
    qemu_set_irq(s->irq, !!(s->int_val & s->int_en));
}

static void vo_raise(IngenicVoState *s, uint32_t bits)
{
    s->int_val |= bits & ~s->int_clc;
    vo_update_irq(s);
}

/* One frame's length in ns at the VO clock, 0 while it is stopped */
static int64_t vo_frame_ns(IngenicVoState *s)
{
    uint64_t hz = clock_get_hz(s->clk);
    uint64_t w = extract32(s->size, 0, 12), h = extract32(s->size, 16, 12);
    uint64_t lines = h + extract32(s->vblk, 0, 12) + extract32(s->vblk, 16, 12);
    uint64_t clocks = 2 * w + extract32(s->hblk, 0, 12) + (s->t41 ? 8 : 16);

    if (!hz) {
        return 0;
    }
    return MAX(muldiv64(lines * clocks, NANOSECONDS_PER_SECOND, hz), 1);
}

/* The next frame's end, a frame after @from */
static void vo_arm(IngenicVoState *s, int64_t from)
{
    int64_t ns = vo_frame_ns(s);

    if (ns) {
        s->deadline = from + ns;
        timer_mod(s->frame, s->deadline);
    } else {
        timer_del(s->frame);
    }
}

/* The frame at s->cur on the console, NV12 to RGB as BT.601 */
static void vo_show(IngenicVoState *s)
{
    uint32_t w = extract32(s->size, 0, 12), h = extract32(s->size, 16, 12);
    uint32_t ys = extract32(s->stride, 0, 16);
    uint32_t uvs = extract32(s->stride, 16, 16);
    g_autofree uint8_t *yl = NULL, *uvl = NULL;
    DisplaySurface *ds;

    if (!w || !h) {
        return;
    }
    ds = qemu_console_surface(s->con);
    if (!ds || surface_width(ds) != w || surface_height(ds) != h) {
        qemu_console_resize(s->con, w, h);
        ds = qemu_console_surface(s->con);
    }
    if (surface_bits_per_pixel(ds) != 32) {
        return;
    }
    yl = g_malloc(w);
    uvl = g_malloc(w + 1);
    for (uint32_t j = 0; j < h; j++) {
        uint32_t *d = (uint32_t *)((uint8_t *)surface_data(ds) +
                                   j * surface_stride(ds));

        dma_memory_read(&address_space_memory, s->cur.y + (hwaddr)j * ys,
                        yl, w, MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory,
                        s->cur.uv + (hwaddr)(j / 2) * uvs, uvl, w,
                        MEMTXATTRS_UNSPECIFIED);
        for (uint32_t i = 0; i < w; i++) {
            int y = (yl[i] - 16) * 298;
            int u = uvl[i & ~1u] - 128, v = uvl[i | 1] - 128;
            int r = (y + 409 * v + 128) >> 8;
            int g = (y - 100 * u - 208 * v + 128) >> 8;
            int b = (y + 516 * u + 128) >> 8;

            d[i] = (uint32_t)MIN(MAX(r, 0), 255) << 16 |
                   (uint32_t)MIN(MAX(g, 0), 255) << 8 | MIN(MAX(b, 0), 255);
        }
    }
    qemu_console_update(s->con, 0, 0, w, h);
}

static void vo_take(IngenicVoState *s, int64_t from)
{
    s->cur = s->queue[0];
    memmove(s->queue, s->queue + 1, sizeof(s->queue[0]) * (s->qlen - 1));
    s->qlen--;
    s->running = true;
    s->started = true;
    s->dry = 0;
    vo_raise(s, INT_TAKEN);
    vo_arm(s, from);
    vo_show(s);
}

/* A frame's end: the next pair, the frame again, or an underrun */
static void vo_frame_end(void *opaque)
{
    IngenicVoState *s = opaque;

    if (s->qlen) {
        vo_take(s, s->deadline);
    } else if (s->t41 && !s->dry++) {
        vo_arm(s, s->deadline);
    } else {
        s->running = false;
        if (s->t41) {
            s->underrun = true;
            vo_raise(s, INT_UNDERRUN);
        }
    }
}

static void vo_reset_channel(IngenicVoState *s)
{
    timer_del(s->frame);
    s->tran_en = 0;
    s->int_val = 0;
    s->qlen = 0;
    s->running = false;
    s->started = false;
    s->dry = 0;
    s->underrun = false;
    vo_update_irq(s);
}

static uint32_t vo_addr_info(IngenicVoState *s)
{
    uint32_t full = s->qlen == INGENIC_VO_QUEUE ? ADDR_FULL | (1 << 1) : 0;

    if (s->started) {
        return 0x2 | (s->qlen == INGENIC_VO_QUEUE ? ADDR_FULL : 0) |
               (s->int_val & INT_UNDERRUN ? ADDR_UNDERRUN : 0);
    }
    if (!s->qlen) {
        return s->t41 ? 0x15 : 0x25;
    }
    return (s->t41 ? 0x10 : 0x20) | full;
}

static uint64_t ingenic_vo_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicVoState *s = opaque;

    switch (off) {
    case VO_VER:
        return s->t41 ? 0x20211101 : 0x28;
    case VO_TOP_CON:
        if (s->t41) {
            return s->top_con & TOP_CON_RESET ? TOP_CON_RESET : s->top_con;
        }
        return s->top_con & TOP_CON_RESET ? 0 : TOP_CON_RESET | s->top_con;
    case VO_TOP_STATUS:
        return 1;
    case VO_INT_EN:
        return s->int_en;
    case VO_INT_VAL:
        return s->int_val;
    case VO_INT_CLC:
        return s->t41 ? s->int_clc : 0;
    case VO_SIZE:
        return s->size;
    case VO_STRIDE:
        return s->stride;
    case VO_UV_ADDR:
        return s->uv_addr;
    case VO_BLK_INFO:
        return s->blk_info;
    case VO_CON_PAR:
        return s->con_par;
    case VO_VBLK_PAR:
        return s->vblk;
    case VO_HBLK_PAR:
        return s->hblk;
    case VO_TRAN_EN:
        return s->tran_en;
    case VO_TIME_OUT:
        return s->time_out;
    case VO_ADDR_INFO:
        return vo_addr_info(s);
    case VO_YUV_CLIP:
        return s->yuv_clip;
    default:
        return 0;
    }
}

static void ingenic_vo_write(void *opaque, hwaddr off, uint64_t value,
                             unsigned size)
{
    IngenicVoState *s = opaque;
    uint32_t v = value;

    if (off == VO_TOP_CON) {
        s->top_con = v & 3;
        if (vo_in_reset(s)) {
            vo_reset_channel(s);
        }
        return;
    }
    if (!s->t41 && vo_in_reset(s)) {
        return;
    }
    switch (off) {
    case VO_INT_EN:
        s->int_en = v & (s->t41 ? 0x111111 : 0x3);
        vo_update_irq(s);
        break;
    case VO_INT_CLC:
        s->int_clc = v & (s->t41 ? 0x111111 : 0x3);
        s->int_val &= ~s->int_clc;
        vo_update_irq(s);
        break;
    case VO_SIZE:
        s->size = v & 0x0fff0fff;
        break;
    case VO_STRIDE:
        s->stride = v;
        break;
    case VO_Y_ADDR:
        s->y_latch = v;
        break;
    case VO_UV_ADDR:
        s->uv_addr = v;
        if (s->qlen < INGENIC_VO_QUEUE) {
            s->queue[s->qlen++] = (IngenicVoPair) { s->y_latch, v };
            if (s->tran_en && !s->running && !s->underrun) {
                vo_take(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
            }
        }
        break;
    case VO_BLK_INFO:
        s->blk_info = v;
        break;
    case VO_CON_PAR:
        s->con_par = v & (s->t41 ? 0x01ff34ff : 0x01ff0fff);
        break;
    case VO_VBLK_PAR:
        s->vblk = v & 0x0fff0fff;
        break;
    case VO_HBLK_PAR:
        s->hblk = v & 0xfff;
        break;
    case VO_TRAN_EN:
        s->tran_en = v & 1;
        if (!s->tran_en) {
            timer_del(s->frame);
            s->running = false;
            s->started = false;
        } else if (!s->running && s->qlen && !s->underrun) {
            vo_take(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        break;
    case VO_TIME_OUT:
        s->time_out = v;
        break;
    case VO_YUV_CLIP:
        if (s->t41) {
            s->yuv_clip = v;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_vo_ops = {
    .read = ingenic_vo_read,
    .write = ingenic_vo_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_vo_clk_update(void *opaque, ClockEvent event)
{
    IngenicVoState *s = opaque;

    if (s->running) {
        vo_arm(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static const GraphicHwOps ingenic_vo_gfx_ops = {
};

static void ingenic_vo_reset_hold(Object *obj, ResetType type)
{
    IngenicVoState *s = INGENIC_VO(obj);

    s->top_con = 0;
    s->int_en = s->int_clc = 0;
    s->size = 0x04380780;
    s->stride = 0x07800780;
    s->uv_addr = 0;
    s->y_latch = 0;
    s->blk_info = 0x10801080;
    s->con_par = 0x20000;
    s->vblk = 0x100010;
    s->hblk = 0x10;
    s->time_out = 0xffffffff;
    s->yuv_clip = s->t41 ? 0xfe01fe01 : 0;
    vo_reset_channel(s);
}

static void ingenic_vo_realize(DeviceState *dev, Error **errp)
{
    IngenicVoState *s = INGENIC_VO(dev);

    s->con = qemu_graphic_console_create(dev, 0, &ingenic_vo_gfx_ops, s);
    qemu_console_resize(s->con, 1920, 1080);
}

static void ingenic_vo_init(Object *obj)
{
    IngenicVoState *s = INGENIC_VO(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_vo_ops, s, "ingenic-vo",
                          0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", ingenic_vo_clk_update, s,
                                ClockUpdate);
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, vo_frame_end, s);
}

static const VMStateDescription vmstate_ingenic_vo_pair = {
    .name = "ingenic-vo-pair",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(y, IngenicVoPair),
        VMSTATE_UINT32(uv, IngenicVoPair),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_vo = {
    .name = "ingenic-vo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(top_con, IngenicVoState),
        VMSTATE_UINT32(int_en, IngenicVoState),
        VMSTATE_UINT32(int_val, IngenicVoState),
        VMSTATE_UINT32(int_clc, IngenicVoState),
        VMSTATE_UINT32(size, IngenicVoState),
        VMSTATE_UINT32(stride, IngenicVoState),
        VMSTATE_UINT32(uv_addr, IngenicVoState),
        VMSTATE_UINT32(blk_info, IngenicVoState),
        VMSTATE_UINT32(con_par, IngenicVoState),
        VMSTATE_UINT32(vblk, IngenicVoState),
        VMSTATE_UINT32(hblk, IngenicVoState),
        VMSTATE_UINT32(tran_en, IngenicVoState),
        VMSTATE_UINT32(time_out, IngenicVoState),
        VMSTATE_UINT32(yuv_clip, IngenicVoState),
        VMSTATE_UINT32(y_latch, IngenicVoState),
        VMSTATE_STRUCT_ARRAY(queue, IngenicVoState, INGENIC_VO_QUEUE, 1,
                             vmstate_ingenic_vo_pair, IngenicVoPair),
        VMSTATE_UINT32(qlen, IngenicVoState),
        VMSTATE_STRUCT(cur, IngenicVoState, 1, vmstate_ingenic_vo_pair,
                       IngenicVoPair),
        VMSTATE_BOOL(running, IngenicVoState),
        VMSTATE_BOOL(started, IngenicVoState),
        VMSTATE_UINT32(dry, IngenicVoState),
        VMSTATE_BOOL(underrun, IngenicVoState),
        VMSTATE_TIMER_PTR(frame, IngenicVoState),
        VMSTATE_INT64(deadline, IngenicVoState),
        VMSTATE_CLOCK(clk, IngenicVoState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_vo_props[] = {
    DEFINE_PROP_BOOL("t41", IngenicVoState, t41, false),
};

static void ingenic_vo_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_vo_realize;
    dc->vmsd = &vmstate_ingenic_vo;
    device_class_set_props(dc, ingenic_vo_props);
    rc->phases.hold = ingenic_vo_reset_hold;
}

static const TypeInfo ingenic_vo_type_info = {
    .name = TYPE_INGENIC_VO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicVoState),
    .instance_init = ingenic_vo_init,
    .class_init = ingenic_vo_class_init,
};

static void ingenic_vo_register_types(void)
{
    type_register_static(&ingenic_vo_type_info);
}

type_init(ingenic_vo_register_types)
