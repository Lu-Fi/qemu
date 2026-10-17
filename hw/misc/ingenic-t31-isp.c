/*
 * Ingenic T31 ISP (Tiziano) and VIC: the frame path
 *
 * The tx-isp driver programs some thousands of ISP registers, and those
 * read back as written. What it waits on is modelled:
 *
 * - VIC (+0xe0000): a command written to VIC_CTRL completes at once and
 *   the register reads 0; 1 starts the video input and 2 resets it. Once
 *   started it takes frames from the image sensor at the sensor's rate.
 *
 * - The multi-scaler (MSCA) puts each frame out on the channels enabled
 *   in MSCA_CH_EN, as NV12 at the channel's size. Channel N's registers
 *   are at 0x9900 + N * 0x100. The driver queues a buffer by writing its
 *   Y and UV addresses to +0x6c and +0x84; a frame fills the oldest and
 *   moves it to the done FIFO, which the driver drains while +0x7c bit 0
 *   (empty) is clear: +0x74 gives the Y address and pops the entry, +0x8c
 *   and +0x90 its UV address and frame number. With no buffer queued the
 *   channel drops the frame.
 *
 * - Interrupts: ISP_INT_STA bit N is channel N's frame done, 12 the start
 *   of a frame and 13 the end; ISP_INT_EN enables, ISP_INT_CLR clears.
 *   The VIC's status (bit 0: frame done) is masked by VIC_INT_MSK and
 *   cleared through VIC_INT_CLR.
 *
 * The layout is the T31 tx-isp firmware's (the txx-isp-c decompilation).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-t31-isp.h"
#include "hw/sensor/image-sensor.h"

#define ISP_INT_EN          0x0b0
#define ISP_INT_STA         0x0b4
#define ISP_INT_CLR         0x0b8
#define INT_FRAME_START     BIT(12)
#define INT_FRAME_END       BIT(13)

#define MSCA_CH_EN          0x9804
#define MSCA_CH_BUSY        0x9808
#define MSCA_CH(n)          (0x9900 + (n) * 0x100)
#define CH_SIZE             0x00    /* width << 16 | height */
#define CH_Y_PUSH           0x6c
#define CH_DONE_Y           0x74
#define CH_DONE_STA         0x7c
#define CH_Y_STRIDE         0x80
#define CH_UV_PUSH          0x84
#define CH_DONE_UV          0x8c
#define CH_DONE_FRAME       0x90
#define CH_UV_STRIDE        0x98
#define CH_CLR_Y            0x9c    /* write 1 to empty a FIFO */
#define CH_CLR_UV           0xa0
#define CH_CLR_DONE_Y       0xa4
#define CH_CLR_DONE_UV      0xa8

#define VIC_CTRL            0xe0000
#define VIC_INT_STA         0xe01e0
#define VIC_INT_STA1        0xe01e4
#define VIC_INT_MSK         0xe01e8
#define VIC_INT_MSK1        0xe01ec
#define VIC_INT_CLR         0xe01f0
#define VIC_INT_CLR1        0xe01f4
#define VIC_START           BIT(0)
#define VIC_RESET           BIT(1)
#define VIC_INT_FRAME_DONE  BIT(0)

#define R(off)              (s->regs[(off) / 4])

static void isp_push(uint32_t *fifo, uint32_t rd, uint32_t *n, uint32_t v)
{
    if (*n < INGENIC_T31_ISP_FIFO) {
        fifo[(rd + (*n)++) % INGENIC_T31_ISP_FIFO] = v;
    }
}

static uint32_t isp_pop(uint32_t *fifo, uint32_t *rd, uint32_t *n)
{
    uint32_t v;

    if (!*n) {
        return 0;
    }
    v = fifo[*rd];
    *rd = (*rd + 1) % INGENIC_T31_ISP_FIFO;
    (*n)--;
    return v;
}

static void isp_update(IngenicT31IspState *s)
{
    qemu_set_irq(s->isp_irq, !!(R(ISP_INT_STA) & R(ISP_INT_EN)));
    qemu_set_irq(s->vic_irq, !!((R(VIC_INT_STA) & ~R(VIC_INT_MSK)) |
                                (R(VIC_INT_STA1) & ~R(VIC_INT_MSK1))));
}

static void isp_dma_plane(hwaddr addr, const uint8_t *src, uint32_t w,
                          uint32_t h, uint32_t stride)
{
    if (stride <= w) {
        address_space_write(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, src, w * h);
        return;
    }
    for (uint32_t y = 0; y < h; y++) {
        address_space_write(&address_space_memory, addr + y * stride,
                            MEMTXATTRS_UNSPECIFIED, src + y * w, w);
    }
}

static void isp_chan_frame(IngenicT31IspState *s, ImageSensorState *sensor,
                           unsigned n)
{
    IngenicT31IspChan *ch = &s->chan[n];
    uint32_t size = R(MSCA_CH(n) + CH_SIZE);
    uint32_t w = size >> 16, h = size & 0xffff;
    g_autofree uint8_t *nv12 = NULL;
    uint32_t y, uv, i;

    if (!ch->y_n || !ch->uv_n || !w || !h ||
        ch->done_n == INGENIC_T31_ISP_FIFO) {
        return;
    }
    y = isp_pop(ch->y, &ch->y_rd, &ch->y_n);
    uv = isp_pop(ch->uv, &ch->uv_rd, &ch->uv_n);
    nv12 = g_malloc(w * h * 3 / 2);
    image_sensor_render(sensor, s->frame, w, h, nv12);
    isp_dma_plane(y, nv12, w, h, R(MSCA_CH(n) + CH_Y_STRIDE));
    isp_dma_plane(uv, nv12 + w * h, w, h / 2, R(MSCA_CH(n) + CH_UV_STRIDE));
    i = (ch->done_rd + ch->done_n++) % INGENIC_T31_ISP_FIFO;
    ch->done_y[i] = y;
    ch->done_uv[i] = uv;
    ch->done_frame[i] = s->frame;
    R(ISP_INT_STA) |= BIT(n);
}

static void isp_frame(void *opaque)
{
    IngenicT31IspState *s = opaque;
    ImageSensorState *sensor = image_sensor_find();

    if (!s->vic_running || !sensor) {
        return;
    }
    s->frame++;
    R(ISP_INT_STA) |= INT_FRAME_START;
    for (unsigned n = 0; n < INGENIC_T31_ISP_CHANS; n++) {
        if (R(MSCA_CH_EN) & BIT(n)) {
            isp_chan_frame(s, sensor, n);
        }
    }
    R(ISP_INT_STA) |= INT_FRAME_END;
    R(VIC_INT_STA) |= VIC_INT_FRAME_DONE;
    isp_update(s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / sensor->fps);
}

static uint64_t ingenic_t31_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicT31IspState *s = INGENIC_T31_ISP(opaque);

    if (off >= MSCA_CH(0) && off < MSCA_CH(INGENIC_T31_ISP_CHANS)) {
        IngenicT31IspChan *ch = &s->chan[(off - MSCA_CH(0)) / 0x100];
        uint32_t i = ch->done_rd;

        switch (off & 0xff) {
        case CH_DONE_Y:
            if (!ch->done_n) {
                return 0;
            }
            ch->done_rd = (i + 1) % INGENIC_T31_ISP_FIFO;
            ch->done_n--;
            ch->popped_uv = ch->done_uv[i];
            ch->popped_frame = ch->done_frame[i];
            return ch->done_y[i];
        case CH_DONE_STA:
            return !ch->done_n;
        case CH_DONE_UV:
            return ch->popped_uv;
        case CH_DONE_FRAME:
            return ch->popped_frame;
        }
    }
    switch (off) {
    case MSCA_CH_BUSY:
    case VIC_CTRL:
        return 0;
    default:
        return R(off);
    }
}

static void ingenic_t31_isp_write(void *opaque, hwaddr off, uint64_t value,
                                  unsigned size)
{
    IngenicT31IspState *s = INGENIC_T31_ISP(opaque);
    uint32_t v = value;

    if (off >= MSCA_CH(0) && off < MSCA_CH(INGENIC_T31_ISP_CHANS)) {
        IngenicT31IspChan *ch = &s->chan[(off - MSCA_CH(0)) / 0x100];

        switch (off & 0xff) {
        case CH_Y_PUSH:
            isp_push(ch->y, ch->y_rd, &ch->y_n, v);
            return;
        case CH_UV_PUSH:
            isp_push(ch->uv, ch->uv_rd, &ch->uv_n, v);
            return;
        case CH_CLR_Y:
            ch->y_n = 0;
            return;
        case CH_CLR_UV:
            ch->uv_n = 0;
            return;
        case CH_CLR_DONE_Y:
        case CH_CLR_DONE_UV:
            ch->done_n = 0;
            return;
        case CH_DONE_Y:
        case CH_DONE_STA:
        case CH_DONE_UV:
        case CH_DONE_FRAME:
            return;
        }
    }
    switch (off) {
    case ISP_INT_STA:
    case VIC_INT_STA:
    case VIC_INT_STA1:
        return;
    case ISP_INT_CLR:
        R(ISP_INT_STA) &= ~v;
        break;
    case VIC_INT_CLR:
        R(VIC_INT_STA) &= ~v;
        break;
    case VIC_INT_CLR1:
        R(VIC_INT_STA1) &= ~v;
        break;
    case VIC_CTRL:
        if (v & VIC_RESET) {
            s->vic_running = false;
            timer_del(s->frame_timer);
        }
        if ((v & VIC_START) && !s->vic_running) {
            s->vic_running = true;
            isp_frame(s);
        }
        return;
    default:
        R(off) = v;
        break;
    }
    isp_update(s);
}

static const MemoryRegionOps ingenic_t31_isp_ops = {
    .read = ingenic_t31_isp_read,
    .write = ingenic_t31_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_isp_reset_hold(Object *obj, ResetType type)
{
    IngenicT31IspState *s = INGENIC_T31_ISP(obj);

    timer_del(s->frame_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->chan, 0, sizeof(s->chan));
    s->frame = 0;
    s->vic_running = false;
}

static void ingenic_t31_isp_reset_exit(Object *obj, ResetType type)
{
    isp_update(INGENIC_T31_ISP(obj));
}

static void ingenic_t31_isp_init(Object *obj)
{
    IngenicT31IspState *s = INGENIC_T31_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_isp_ops, s,
                          TYPE_INGENIC_T31_ISP, INGENIC_T31_ISP_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->isp_irq);
    sysbus_init_irq(sbd, &s->vic_irq);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, isp_frame, s);
}

static const VMStateDescription vmstate_ingenic_t31_isp_chan = {
    .name = TYPE_INGENIC_T31_ISP "/chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(y, IngenicT31IspChan, INGENIC_T31_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(uv, IngenicT31IspChan, INGENIC_T31_ISP_FIFO),
        VMSTATE_UINT32(y_rd, IngenicT31IspChan),
        VMSTATE_UINT32(y_n, IngenicT31IspChan),
        VMSTATE_UINT32(uv_rd, IngenicT31IspChan),
        VMSTATE_UINT32(uv_n, IngenicT31IspChan),
        VMSTATE_UINT32_ARRAY(done_y, IngenicT31IspChan, INGENIC_T31_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(done_uv, IngenicT31IspChan,
                             INGENIC_T31_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(done_frame, IngenicT31IspChan,
                             INGENIC_T31_ISP_FIFO),
        VMSTATE_UINT32(done_rd, IngenicT31IspChan),
        VMSTATE_UINT32(done_n, IngenicT31IspChan),
        VMSTATE_UINT32(popped_uv, IngenicT31IspChan),
        VMSTATE_UINT32(popped_frame, IngenicT31IspChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_t31_isp = {
    .name = TYPE_INGENIC_T31_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT31IspState,
                             INGENIC_T31_ISP_IOSIZE / 4),
        VMSTATE_STRUCT_ARRAY(chan, IngenicT31IspState, INGENIC_T31_ISP_CHANS,
                             1, vmstate_ingenic_t31_isp_chan,
                             IngenicT31IspChan),
        VMSTATE_UINT32(frame, IngenicT31IspState),
        VMSTATE_BOOL(vic_running, IngenicT31IspState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicT31IspState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t31_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic T31 ISP and VIC";
    dc->vmsd = &vmstate_ingenic_t31_isp;
    rc->phases.hold = ingenic_t31_isp_reset_hold;
    rc->phases.exit = ingenic_t31_isp_reset_exit;
}

static const TypeInfo ingenic_t31_isp_type_info = {
    .name = TYPE_INGENIC_T31_ISP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31IspState),
    .instance_init = ingenic_t31_isp_init,
    .class_init = ingenic_t31_isp_class_init,
};

static void ingenic_t31_isp_register_types(void)
{
    type_register_static(&ingenic_t31_isp_type_info);
}

type_init(ingenic_t31_isp_register_types)
