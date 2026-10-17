/*
 * Ingenic ISP (Tiziano) and VIC: the frame path
 *
 * The tx-isp drivers program some thousands of ISP registers, and those
 * read back as written. What they wait on is modelled:
 *
 * - VIC: a command written to VIC_CTRL completes at once and the register
 *   reads 0; 1 starts the video input and 2 resets it. Once started it
 *   takes frames from the image sensor at the sensor's rate. The T31 has
 *   one VIC, at +0xe0000; the T40 three, at +0x80000 + N * 0x10000, each
 *   with its own interrupt.
 *
 * - The multi-scaler (MSCA) puts each frame out on the channels enabled
 *   in its channel enable register, as NV12 scaled to +0x0 and cropped to
 *   +0x28 (x << 16 | y) and +0x2c (w << 16 | h). The T31 has one MSCA,
 *   its channels at 0x9900 + N * 0x100; the T40 one for each of its two
 *   pipelines, at 0x16100 and 0x17100. The driver queues a buffer by
 *   writing its Y and UV addresses to +0x6c and +0x84; a frame fills the
 *   oldest and moves it to the done FIFO, which the driver drains while
 *   +0x7c bit 0 (empty) is clear (bits 11:8 count the entries): +0x74
 *   gives the Y address and pops the entry, +0x8c and +0x90 its UV address
 *   and frame number. With no buffer queued the channel drops the frame,
 *   and it does while the last is undrained: the T40 driver reports only
 *   the last entry it drains, and a guest can fall a frame behind here
 *   where silicon would not.
 *
 * - Interrupts: a pipeline's status bit N is channel N's frame done, and
 *   the frame start (and on the T31 the frame end) has a bit too; the
 *   enable register gates the ISP interrupt and the clear register clears.
 *   A VIC's status (bit 0: frame done, bit 4 on the T40: frame start) is
 *   masked by VIC_INT_MSK and cleared through VIC_INT_CLR.
 *
 * - The T40 driver requests the ISP idle through 0x200 bit 0 before it
 *   reconfigures, and spins until 0x204 bit 0 says so; frames take no time
 *   here, so the ISP is always idle. The MSCA registers it writes through
 *   the shadow port (0x50324: a value, then the register's offset) take
 *   effect at once.
 *
 * The one image sensor feeds the first pipeline.
 *
 * The layouts are the T31 and T40 tx-isp drivers' (their decompilations).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-isp.h"
#include "hw/sensor/image-sensor.h"

#define CH_SIZE             0x00    /* width << 16 | height */
#define CH_CROP_POS         0x28
#define CH_CROP_SIZE        0x2c
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

#define VIC_CTRL            0x000
#define VIC_INT_STA         0x1e0
#define VIC_INT_STA1        0x1e4
#define VIC_INT_MSK         0x1e8
#define VIC_INT_MSK1        0x1ec
#define VIC_INT_CLR         0x1f0
#define VIC_INT_CLR1        0x1f4
#define VIC_START           BIT(0)
#define VIC_RESET           BIT(1)
#define VIC_INT_FRAME_DONE  BIT(0)

typedef struct IngenicIspPipe {
    hwaddr int_en, int_sta, int_clr;
    hwaddr ch_en, ch;
} IngenicIspPipe;

/* Offsets of 0 are registers a layout lacks */
struct IngenicIspLayout {
    const char *desc;
    unsigned pipes;
    IngenicIspPipe pipe[INGENIC_ISP_PIPES];
    uint32_t int_frame_start, int_frame_end;
    hwaddr ch_busy;                 /* reads 0 */
    unsigned vics;
    hwaddr vic, vic_stride;
    uint32_t vic_frame_start;
    hwaddr idle;                    /* bit 0 reads 1 */
    hwaddr shadow;
};

static const IngenicIspLayout ingenic_t31_isp_layout = {
    .desc = "Ingenic T31 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x0b0, .int_sta = 0x0b4, .int_clr = 0x0b8,
          .ch_en = 0x9804, .ch = 0x9900 },
    },
    .int_frame_start = BIT(12),
    .int_frame_end = BIT(13),
    .ch_busy = 0x9808,
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
};

static const IngenicIspLayout ingenic_t40_isp_layout = {
    .desc = "Ingenic T40 ISP and VICs",
    .pipes = 2,
    .pipe = {
        { .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
          .ch_en = 0x16008, .ch = 0x16100 },
        { .int_en = 0x40030, .int_sta = 0x40038, .int_clr = 0x40034,
          .ch_en = 0x17008, .ch = 0x17100 },
    },
    .int_frame_start = BIT(13),
    .vics = 3,
    .vic = 0x80000,
    .vic_stride = 0x10000,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .shadow = 0x50324,
};

#define R(off)              (s->regs[(off) / 4])

static const IngenicIspLayout *isp_layout(IngenicIspState *s)
{
    return INGENIC_ISP_GET_CLASS(s)->layout;
}

static IngenicIspChan *isp_chan(IngenicIspState *s, hwaddr off)
{
    const IngenicIspLayout *l = isp_layout(s);

    for (unsigned p = 0; p < l->pipes; p++) {
        hwaddr ch = l->pipe[p].ch;

        if (off >= ch && off < ch + INGENIC_ISP_CHANS * 0x100) {
            return &s->chan[p][(off - ch) / 0x100];
        }
    }
    return NULL;
}

/* The VIC an offset falls in, or -1; *reg is the offset within it */
static int isp_vic(IngenicIspState *s, hwaddr off, hwaddr *reg)
{
    const IngenicIspLayout *l = isp_layout(s);

    if (off < l->vic || off >= l->vic + l->vics * l->vic_stride) {
        return -1;
    }
    *reg = (off - l->vic) % l->vic_stride;
    return (off - l->vic) / l->vic_stride;
}

#define VIC(n, reg)         R(l->vic + (n) * l->vic_stride + (reg))

static void isp_push(uint32_t *fifo, uint32_t rd, uint32_t *n, uint32_t v)
{
    if (*n < INGENIC_ISP_FIFO) {
        fifo[(rd + (*n)++) % INGENIC_ISP_FIFO] = v;
    }
}

static uint32_t isp_pop(uint32_t *fifo, uint32_t *rd, uint32_t *n)
{
    uint32_t v;

    if (!*n) {
        return 0;
    }
    v = fifo[*rd];
    *rd = (*rd + 1) % INGENIC_ISP_FIFO;
    (*n)--;
    return v;
}

static void isp_update(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);
    bool isp = false;

    for (unsigned p = 0; p < l->pipes; p++) {
        isp |= R(l->pipe[p].int_sta) & R(l->pipe[p].int_en);
    }
    qemu_set_irq(s->isp_irq, isp);
    for (unsigned n = 0; n < l->vics; n++) {
        qemu_set_irq(s->vic_irq[n],
                     !!((VIC(n, VIC_INT_STA) & ~VIC(n, VIC_INT_MSK)) |
                        (VIC(n, VIC_INT_STA1) & ~VIC(n, VIC_INT_MSK1))));
    }
}

/* The w x h window at (x, y) of a frame src_w wide, out to a plane */
static void isp_dma_plane(hwaddr addr, const uint8_t *src, uint32_t src_w,
                          uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                          uint32_t stride)
{
    src += y * src_w + x;
    if (stride <= w && src_w == w) {
        address_space_write(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, src, w * h);
        return;
    }
    stride = MAX(stride, w);
    for (uint32_t i = 0; i < h; i++) {
        address_space_write(&address_space_memory, addr + i * stride,
                            MEMTXATTRS_UNSPECIFIED, src + i * src_w, w);
    }
}

static void isp_chan_frame(IngenicIspState *s, ImageSensorState *sensor,
                           unsigned p, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr base = l->pipe[p].ch + n * 0x100;
    IngenicIspChan *ch = &s->chan[p][n];
    uint32_t size = R(base + CH_SIZE), crop = R(base + CH_CROP_SIZE);
    uint32_t pos = R(base + CH_CROP_POS);
    uint32_t w = size >> 16, h = size & 0xffff;
    uint32_t cx = pos >> 16, cy = pos & 0xffff;
    uint32_t cw = crop >> 16, ch_h = crop & 0xffff;
    g_autofree uint8_t *nv12 = NULL;
    uint32_t y, uv, i;

    /* NV12 needs even sizes and positions */
    w &= ~1;
    h &= ~1;
    if (!crop || cx + cw > w || cy + ch_h > h) {
        cx = cy = 0;
        cw = w;
        ch_h = h;
    }
    cx &= ~1;
    cy &= ~1;
    cw &= ~1;
    ch_h &= ~1;
    if (!ch->y_n || !ch->uv_n || !cw || !ch_h || ch->done_n) {
        return;
    }
    y = isp_pop(ch->y, &ch->y_rd, &ch->y_n);
    uv = isp_pop(ch->uv, &ch->uv_rd, &ch->uv_n);
    nv12 = g_malloc(w * h * 3 / 2);
    image_sensor_render(sensor, s->frame, w, h, nv12);
    isp_dma_plane(y, nv12, w, cx, cy, cw, ch_h, R(base + CH_Y_STRIDE));
    isp_dma_plane(uv, nv12 + w * h, w, cx, cy / 2, cw, ch_h / 2,
                  R(base + CH_UV_STRIDE));
    i = (ch->done_rd + ch->done_n++) % INGENIC_ISP_FIFO;
    ch->done_y[i] = y;
    ch->done_uv[i] = uv;
    ch->done_frame[i] = s->frame;
    R(l->pipe[p].int_sta) |= BIT(n);
}

static void isp_frame(void *opaque)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspPipe *pipe = &l->pipe[0];
    ImageSensorState *sensor = image_sensor_find();

    if (!s->vic_running || !sensor) {
        return;
    }
    s->frame++;
    R(pipe->int_sta) |= l->int_frame_start;
    for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
        if (R(pipe->ch_en) & BIT(n)) {
            isp_chan_frame(s, sensor, 0, n);
        }
    }
    R(pipe->int_sta) |= l->int_frame_end;
    for (unsigned n = 0; n < l->vics; n++) {
        if (s->vic_running & BIT(n)) {
            VIC(n, VIC_INT_STA) |= l->vic_frame_start | VIC_INT_FRAME_DONE;
        }
    }
    isp_update(s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / sensor->fps);
}

static uint64_t ingenic_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspState *s = INGENIC_ISP(opaque);
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspChan *ch = isp_chan(s, off);
    hwaddr reg;

    if (ch) {
        uint32_t i = ch->done_rd;

        switch (off & 0xff) {
        case CH_DONE_Y:
            if (!ch->done_n) {
                return 0;
            }
            ch->done_rd = (i + 1) % INGENIC_ISP_FIFO;
            ch->done_n--;
            ch->popped_uv = ch->done_uv[i];
            ch->popped_frame = ch->done_frame[i];
            return ch->done_y[i];
        case CH_DONE_STA:
            return ch->done_n << 8 | !ch->done_n;
        case CH_DONE_UV:
            return ch->popped_uv;
        case CH_DONE_FRAME:
            return ch->popped_frame;
        }
    }
    if (isp_vic(s, off, &reg) >= 0 && reg == VIC_CTRL) {
        return 0;
    }
    if (l->ch_busy && off == l->ch_busy) {
        return 0;
    }
    if (l->idle && off == l->idle) {
        return R(off) | 1;
    }
    return R(off);
}

static void ingenic_isp_write(void *opaque, hwaddr off, uint64_t value,
                              unsigned size)
{
    IngenicIspState *s = INGENIC_ISP(opaque);
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspChan *ch = isp_chan(s, off);
    uint32_t v = value;
    hwaddr reg;
    int vic;

    if (ch) {
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

    vic = isp_vic(s, off, &reg);
    switch (vic < 0 ? -1 : reg) {
    case VIC_INT_STA:
    case VIC_INT_STA1:
        return;
    case VIC_INT_CLR:
        VIC(vic, VIC_INT_STA) &= ~v;
        isp_update(s);
        return;
    case VIC_INT_CLR1:
        VIC(vic, VIC_INT_STA1) &= ~v;
        isp_update(s);
        return;
    case VIC_CTRL:
        if (v & VIC_RESET) {
            s->vic_running &= ~BIT(vic);
            if (!s->vic_running) {
                timer_del(s->frame_timer);
            }
        }
        if ((v & VIC_START) && !(s->vic_running & BIT(vic))) {
            bool idle = !s->vic_running;

            s->vic_running |= BIT(vic);
            if (idle) {
                isp_frame(s);
            }
        }
        return;
    }

    if (l->shadow && off == l->shadow) {
        if (!s->shadow_pending) {
            s->shadow_value = v;
        } else if (v != l->shadow && v < INGENIC_ISP_IOSIZE) {
            s->shadow_pending = false;
            ingenic_isp_write(s, v & ~3, s->shadow_value, 4);
            return;
        }
        s->shadow_pending = !s->shadow_pending;
        return;
    }
    for (unsigned p = 0; p < l->pipes; p++) {
        if (off == l->pipe[p].int_sta) {
            return;
        }
        if (off == l->pipe[p].int_clr) {
            R(l->pipe[p].int_sta) &= ~v;
            isp_update(s);
            return;
        }
    }
    R(off) = v;
    isp_update(s);
}

static const MemoryRegionOps ingenic_isp_ops = {
    .read = ingenic_isp_read,
    .write = ingenic_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_isp_reset_hold(Object *obj, ResetType type)
{
    IngenicIspState *s = INGENIC_ISP(obj);

    timer_del(s->frame_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->chan, 0, sizeof(s->chan));
    s->frame = 0;
    s->vic_running = 0;
    s->shadow_value = 0;
    s->shadow_pending = false;
}

static void ingenic_isp_reset_exit(Object *obj, ResetType type)
{
    isp_update(INGENIC_ISP(obj));
}

static void ingenic_isp_init(Object *obj)
{
    IngenicIspState *s = INGENIC_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    const IngenicIspLayout *l = isp_layout(s);

    memory_region_init_io(&s->iomem, obj, &ingenic_isp_ops, s,
                          object_get_typename(obj), INGENIC_ISP_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->isp_irq);
    for (unsigned n = 0; n < l->vics; n++) {
        sysbus_init_irq(sbd, &s->vic_irq[n]);
    }
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, isp_frame, s);
}

static const VMStateDescription vmstate_ingenic_isp_chan = {
    .name = TYPE_INGENIC_ISP "/chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(y, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(uv, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32(y_rd, IngenicIspChan),
        VMSTATE_UINT32(y_n, IngenicIspChan),
        VMSTATE_UINT32(uv_rd, IngenicIspChan),
        VMSTATE_UINT32(uv_n, IngenicIspChan),
        VMSTATE_UINT32_ARRAY(done_y, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(done_uv, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(done_frame, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32(done_rd, IngenicIspChan),
        VMSTATE_UINT32(done_n, IngenicIspChan),
        VMSTATE_UINT32(popped_uv, IngenicIspChan),
        VMSTATE_UINT32(popped_frame, IngenicIspChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_isp = {
    .name = TYPE_INGENIC_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIspState, INGENIC_ISP_IOSIZE / 4),
        VMSTATE_STRUCT_2DARRAY(chan, IngenicIspState, INGENIC_ISP_PIPES,
                               INGENIC_ISP_CHANS, 1, vmstate_ingenic_isp_chan,
                               IngenicIspChan),
        VMSTATE_UINT32(frame, IngenicIspState),
        VMSTATE_UINT32(vic_running, IngenicIspState),
        VMSTATE_UINT32(shadow_value, IngenicIspState),
        VMSTATE_BOOL(shadow_pending, IngenicIspState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_isp;
    rc->phases.hold = ingenic_isp_reset_hold;
    rc->phases.exit = ingenic_isp_reset_exit;
}

static void ingenic_isp_soc_class_init(ObjectClass *oc, const void *data)
{
    const IngenicIspLayout *l = data;

    INGENIC_ISP_CLASS(oc)->layout = l;
    DEVICE_CLASS(oc)->desc = l->desc;
}

static const TypeInfo ingenic_isp_types[] = {
    {
        .name = TYPE_INGENIC_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicIspState),
        .instance_init = ingenic_isp_init,
        .class_size = sizeof(IngenicIspClass),
        .class_init = ingenic_isp_class_init,
        .abstract = true,
    }, {
        .name = TYPE_INGENIC_T31_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t31_isp_layout,
    }, {
        .name = TYPE_INGENIC_T40_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t40_isp_layout,
    },
};

DEFINE_TYPES(ingenic_isp_types)
