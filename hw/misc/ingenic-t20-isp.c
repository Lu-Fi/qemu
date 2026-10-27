/*
 * Ingenic T10/T20/T30 ISP and VIC: the frame path
 *
 * An Apical ISP core (0x80000 into the window) behind Ingenic's VIC (0x0)
 * and an interrupt glue block (0x20000), on one interrupt, INTC source 29.
 * The tx-isp-t10/t20 driver and the Apical firmware program some thousands
 * of registers, which read back as written; what they wait on is modelled:
 *
 * - VIC control (0x0, the T10's 0x20) reads 0: a command completes at once.
 *   Bit 0 starts frames at the image sensor's rate, bit 2 or 3 stops them;
 *   bit 4 (VIC_RESET) does not stop them, as the T20 driver writes it at
 *   frame-done counts 0, 10 and 20 and never restarts.
 *
 * - Apical interrupts: 16 slots, each taking events from the source ID in
 *   a 6-bit field of 0x80080-0x8009c (slot 2k bits 5:0, 2k + 1 bits
 *   21:16); 0x800a0 latches a bit per slot, and a 0 to 1 transition of a
 *   bit written to 0x800a4 clears it. The glue's status (0x2000c) shows
 *   them in bits 15:0 and the VIC frame done in bit 16, cleared by writing
 *   1 to 0x20004; the interrupt is status & enable (0x20014) & ~mask
 *   (0x2001c). The firmware disables and re-enables bits 15:0 of 0x20014
 *   around its critical sections; events latch meanwhile.
 *
 * - Output: Apical video DMA writers, FR at 0x80b00, DS1 at 0x80c00 and
 *   DS2 at 0x80d00, each a luma writer and a chroma one 0x80 above: +0x0
 *   bits 3:0 the format (13 NV12), 7:6 the plane (1 UV, 2 VU), bit 9 on;
 *   +0x4 the size (height << 16 | width); +0x8-0x18 five bank addresses
 *   the driver fills; +0x1c the last bank (bits 2:0) and a restart at bank
 *   0 (bit 3); +0x20 the line offset, signed (vflip); +0x24 bits 1:0
 *   control (2 runs, 3 cancels), reading back the current bank (10:8) and
 *   the last (13:11). At each frame a running writer moves to its next
 *   bank and writes the frame there; the driver learns from +0x24 which
 *   bank finished. There is no FIFO and no frame number.
 *
 * - A frame: frame start (Apical event 24), then half a frame later, the
 *   writes, each channel's writer done (FR 14, DS1 15, DS2 1), frame end
 *   (23), the DS1 and DS2 output ends (50, 53, which re-enable a cancelled
 *   writer), the AE and AWB statistics (11, 9) and the VIC frame done. The
 *   driver fills the banks at frame start: writing at once would put the
 *   first frame after a restart into the last session's buffers.
 *
 * - 0x80134 bit 0 says the input port runs while 0x80130 bits 2:0 ask for
 *   it (1, 3 or 5).
 *
 * The T30's ISP is the same core, its VIC events in the glue as well, but
 * libimp feeds its frame writer to a line buffer (bank addresses
 * 0xf0000000 and 0xf8000000) on to the noise reducer (NCU, 0x13090000,
 * here a 4 KiB window whose 0x4 bit 16 reads idle) and the multi-scaler,
 * which writes the channels' frames (ingenic-t30-mscaler): the model
 * writes no frame here and pulses "frame-out" for the multi-scaler instead,
 * before the frame end that opens the line buffer.
 *
 * The layout is the vendor drivers' (ingenic-sdk isp/t20, which the T10
 * shares, and isp/t30) and the Apical firmware's register headers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-t20-isp.h"
#include "hw/sensor/image-sensor.h"

#define VIC_START           BIT(0)
#define VIC_STOP            (BIT(2) | BIT(3))

#define TOP_CLR             0x20004
#define TOP_CLR_ALL         0x20008
#define TOP_STA             0x2000c
#define TOP_ENABLE          0x20014
#define TOP_MASK            0x2001c
#define TOP_VIC_DONE        BIT(16)
#define TOP_IRQS            0x7fffff

#define CORE                0x80000
#define CORE_SLOT_SRC       (CORE + 0x80)
#define CORE_INT_STA        (CORE + 0xa0)
#define CORE_INT_CLR        (CORE + 0xa4)
#define CORE_PORT_REQ       (CORE + 0x130)
#define CORE_PORT_STA       (CORE + 0x134)
#define CORE_WRITER         (CORE + 0xb00)  /* FR; DS1, DS2 0x100 apart */
#define CORE_WRITER_END     (CORE + 0xe00)

/* An Apical writer's registers */
#define WR_CFG              0x00
#define  WR_FORMAT          0xf
#define  WR_NV12            13
#define  WR_PLANE_VU        (2 << 6)
#define  WR_ON              BIT(9)
#define WR_SIZE             0x04
#define WR_BANK             0x08
#define WR_BANKS            0x1c
#define  WR_RESTART         BIT(3)
#define WR_LINE             0x20
#define WR_CTRL             0x24
#define  WR_RUN             2           /* AXI on, no cancel */
#define WR_BASE_CURR        0x28
#define WR_BASE_LAST        0x2c
#define WR_COUNT            0x30

/* Apical event sources */
#define EV_FRAME_START      24
#define EV_FRAME_END        23
#define EV_AE               11
#define EV_AWB              9

#define NCU_START           0x04
#define  NCU_IDLE           BIT(16)

#define R(off)              (s->regs[(off) / 4])

static const uint8_t writer_done[3] = { 14, 15, 1 };    /* FR, DS1, DS2 */
static const uint8_t output_end[2] = { 50, 53 };        /* DS1, DS2 */

static void t20_isp_update(IngenicT20IspState *s)
{
    uint32_t sta = (s->status & 0xffff) | s->vic_status;

    qemu_set_irq(s->irq, !!(sta & R(TOP_ENABLE) & ~R(TOP_MASK) & TOP_IRQS));
}

static void t20_isp_event(IngenicT20IspState *s, unsigned src)
{
    for (unsigned slot = 0; slot < 16; slot++) {
        uint32_t v = R(CORE_SLOT_SRC + slot / 2 * 4) >> (slot & 1 ? 16 : 0);

        if ((v & 0x3f) == src) {
            s->status |= BIT(slot);
        }
    }
}

static int t20_isp_writer(hwaddr off, hwaddr *reg)
{
    if (off < CORE_WRITER || off >= CORE_WRITER_END) {
        return -1;
    }
    *reg = (off - CORE_WRITER) & 0x7f;
    return (off - CORE_WRITER) / 0x80;
}

/* The writer at @n moves to its next bank; whether it writes this frame */
static bool t20_isp_advance(IngenicT20IspState *s, unsigned n)
{
    hwaddr base = CORE_WRITER + n * 0x80;
    IngenicT20IspWriter *w = &s->writer[n];
    uint32_t banks = R(base + WR_BANKS);

    if (!(R(base + WR_CFG) & WR_ON) || (R(base + WR_CTRL) & 3) != WR_RUN) {
        return false;
    }
    w->last = w->curr;
    w->curr = banks & WR_RESTART || w->curr >= MIN(banks & 7, 4) ? 0
                                                                 : w->curr + 1;
    w->base_last = w->base_curr;
    w->base_curr = R(base + WR_BANK + 4 * w->curr);
    w->frames++;
    return true;
}

static void t20_isp_plane(uint32_t addr, int32_t line, const uint8_t *src,
                          uint32_t w, uint32_t rows)
{
    if (!line) {
        line = w;
    }
    for (uint32_t r = 0; r < rows; r++) {
        address_space_write(&address_space_memory, addr + (int64_t)r * line,
                            MEMTXATTRS_UNSPECIFIED, src + r * w, w);
    }
}

/* A channel's frame: luma writer @n, chroma writer @n + 1 */
static void t20_isp_channel(IngenicT20IspState *s, ImageSensorState *sensor,
                            unsigned n)
{
    hwaddr y = CORE_WRITER + n * 0x80, uv = y + 0x80;
    uint32_t size = R(y + WR_SIZE);
    uint32_t w = size & 0xfffe, h = (size >> 16) & 0xfffe;
    g_autofree uint8_t *nv12 = NULL;
    bool chroma;

    if (!t20_isp_advance(s, n)) {
        return;
    }
    chroma = t20_isp_advance(s, n + 1);
    if (w && h && (R(y + WR_CFG) & WR_FORMAT) == WR_NV12 &&
        !INGENIC_T20_ISP_GET_CLASS(s)->line_buffer) {
        nv12 = g_malloc(w * h * 3 / 2);
        image_sensor_render(sensor, s->frame, w, h, nv12);
        t20_isp_plane(s->writer[n].base_curr, R(y + WR_LINE), nv12, w, h);
        if (chroma) {
            uint8_t *c = nv12 + w * h;

            if ((R(uv + WR_CFG) & (3 << 6)) == WR_PLANE_VU) {
                for (uint32_t i = 0; i < w * h / 2; i += 2) {
                    uint8_t t = c[i];

                    c[i] = c[i + 1];
                    c[i + 1] = t;
                }
            }
            t20_isp_plane(s->writer[n + 1].base_curr, R(uv + WR_LINE), c, w,
                          h / 2);
        }
    }
    t20_isp_event(s, writer_done[n / 2]);
}

static void t20_isp_frame(void *opaque)
{
    IngenicT20IspState *s = opaque;
    ImageSensorState *sensor = image_sensor_find(0);
    int64_t half;

    if (!s->running || !sensor) {
        return;
    }
    half = NANOSECONDS_PER_SECOND / MAX(sensor->fps, 1) / 2;
    if (!s->mid_frame) {
        s->frame++;
        t20_isp_event(s, EV_FRAME_START);
    } else {
        qemu_irq_pulse(s->frame_out);
        for (unsigned n = 0; n < INGENIC_T20_ISP_WRITERS; n += 2) {
            t20_isp_channel(s, sensor, n);
        }
        t20_isp_event(s, EV_FRAME_END);
        t20_isp_event(s, output_end[0]);
        t20_isp_event(s, output_end[1]);
        t20_isp_event(s, EV_AE);
        t20_isp_event(s, EV_AWB);
        s->vic_status |= TOP_VIC_DONE;
    }
    s->mid_frame = !s->mid_frame;
    t20_isp_update(s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + half);
}

static uint64_t ingenic_t20_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(opaque);
    hwaddr reg;
    int n;

    if (off == INGENIC_T20_ISP_GET_CLASS(s)->vic_ctrl) {
        return 0;
    }
    switch (off) {
    case TOP_STA:
        return (s->status & 0xffff) | s->vic_status;
    case CORE_INT_STA:
        return s->status;
    case CORE_PORT_STA:
        switch (R(CORE_PORT_REQ) & 7) {
        case 1:
        case 3:
        case 5:
            return 1;
        }
        return 0;
    }
    n = t20_isp_writer(off, &reg);
    if (n >= 0) {
        IngenicT20IspWriter *w = &s->writer[n];

        switch (reg) {
        case WR_CTRL:
            return (R(off) & 3) | w->curr << 8 | w->last << 11;
        case WR_BASE_CURR:
            return w->base_curr;
        case WR_BASE_LAST:
            return w->base_last;
        case WR_COUNT:
            return (w->frames & 0xffff) * 0x10001;
        }
    }
    return R(off);
}

static void ingenic_t20_isp_write(void *opaque, hwaddr off, uint64_t value,
                                  unsigned size)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(opaque);
    uint32_t v = value, old;

    if (off == INGENIC_T20_ISP_GET_CLASS(s)->vic_ctrl) {
        if (v & VIC_STOP) {
            s->running = false;
            s->mid_frame = false;
            timer_del(s->frame_timer);
        } else if (v & VIC_START && !s->running) {
            s->running = true;
            timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
        return;
    }
    switch (off) {
    case TOP_CLR:
        s->vic_status &= ~v;
        t20_isp_update(s);
        return;
    case TOP_CLR_ALL:
        s->vic_status = 0;
        t20_isp_update(s);
        return;
    case TOP_STA:
    case CORE_INT_STA:
    case CORE_PORT_STA:
        return;
    case TOP_ENABLE:
    case TOP_MASK:
        R(off) = v;
        t20_isp_update(s);
        return;
    case CORE_INT_CLR:
        old = R(off);
        R(off) = v;
        s->status &= ~(v & ~old & 0xffff);
        t20_isp_update(s);
        return;
    }
    R(off) = v;
}

static uint64_t ingenic_t30_ncu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(opaque);

    return s->ncu_regs[off / 4] | (off == NCU_START ? NCU_IDLE : 0);
}

static void ingenic_t30_ncu_write(void *opaque, hwaddr off, uint64_t value,
                                  unsigned size)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(opaque);

    s->ncu_regs[off / 4] = value;
}

static const MemoryRegionOps ingenic_t30_ncu_ops = {
    .read = ingenic_t30_ncu_read,
    .write = ingenic_t30_ncu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps ingenic_t20_isp_ops = {
    .read = ingenic_t20_isp_read,
    .write = ingenic_t20_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t20_isp_reset_hold(Object *obj, ResetType type)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(obj);

    timer_del(s->frame_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->writer, 0, sizeof(s->writer));
    memset(s->ncu_regs, 0, sizeof(s->ncu_regs));
    s->status = 0;
    s->vic_status = 0;
    s->frame = 0;
    s->running = false;
    s->mid_frame = false;
}

static void ingenic_t20_isp_reset_exit(Object *obj, ResetType type)
{
    t20_isp_update(INGENIC_T20_ISP(obj));
}

static void ingenic_t20_isp_init(Object *obj)
{
    IngenicT20IspState *s = INGENIC_T20_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t20_isp_ops, s,
                          TYPE_INGENIC_T20_ISP, INGENIC_T20_ISP_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    if (INGENIC_T20_ISP_GET_CLASS(s)->line_buffer) {
        memory_region_init_io(&s->ncu_iomem, obj, &ingenic_t30_ncu_ops, s,
                              "ingenic-t30-ncu", INGENIC_T30_NCU_IOSIZE);
        sysbus_init_mmio(sbd, &s->ncu_iomem);
    }
    qdev_init_gpio_out_named(DEVICE(obj), &s->frame_out, "frame-out", 1);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, t20_isp_frame, s);
}

static const VMStateDescription vmstate_ingenic_t20_isp_writer = {
    .name = TYPE_INGENIC_T20_ISP "/writer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(curr, IngenicT20IspWriter),
        VMSTATE_UINT32(last, IngenicT20IspWriter),
        VMSTATE_UINT32(base_curr, IngenicT20IspWriter),
        VMSTATE_UINT32(base_last, IngenicT20IspWriter),
        VMSTATE_UINT32(frames, IngenicT20IspWriter),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_t20_isp = {
    .name = TYPE_INGENIC_T20_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT20IspState,
                             INGENIC_T20_ISP_IOSIZE / 4),
        VMSTATE_UINT32_ARRAY(ncu_regs, IngenicT20IspState,
                             INGENIC_T30_NCU_IOSIZE / 4),
        VMSTATE_STRUCT_ARRAY(writer, IngenicT20IspState,
                             INGENIC_T20_ISP_WRITERS, 1,
                             vmstate_ingenic_t20_isp_writer,
                             IngenicT20IspWriter),
        VMSTATE_UINT32(status, IngenicT20IspState),
        VMSTATE_UINT32(vic_status, IngenicT20IspState),
        VMSTATE_UINT32(frame, IngenicT20IspState),
        VMSTATE_BOOL(running, IngenicT20IspState),
        VMSTATE_BOOL(mid_frame, IngenicT20IspState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicT20IspState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t20_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic T20 ISP and VIC";
    dc->vmsd = &vmstate_ingenic_t20_isp;
    rc->phases.hold = ingenic_t20_isp_reset_hold;
    rc->phases.exit = ingenic_t20_isp_reset_exit;
    INGENIC_T20_ISP_CLASS(oc)->vic_ctrl = 0x00;
}

static void ingenic_t10_isp_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->desc = "Ingenic T10 ISP and VIC";
    INGENIC_T20_ISP_CLASS(oc)->vic_ctrl = 0x20;
}

static void ingenic_t30_isp_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->desc = "Ingenic T30 ISP, VIC and noise reducer";
    INGENIC_T20_ISP_CLASS(oc)->line_buffer = true;
}

static const TypeInfo ingenic_t20_isp_types[] = {
    {
        .name = TYPE_INGENIC_T20_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicT20IspState),
        .instance_init = ingenic_t20_isp_init,
        .class_size = sizeof(IngenicT20IspClass),
        .class_init = ingenic_t20_isp_class_init,
    }, {
        .name = TYPE_INGENIC_T10_ISP,
        .parent = TYPE_INGENIC_T20_ISP,
        .class_init = ingenic_t10_isp_class_init,
    }, {
        .name = TYPE_INGENIC_T30_ISP,
        .parent = TYPE_INGENIC_T20_ISP,
        .class_init = ingenic_t30_isp_class_init,
    },
};

DEFINE_TYPES(ingenic_t20_isp_types)
