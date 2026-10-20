/*
 * Ingenic IVDC, the ISP to encoder direct connection (T23, T32, T33, T41)
 *
 * In direct mode (tx-isp's direct_mode) the multi-scaler writes a sensor's
 * channel 0 to no memory: its buffer addresses, Y 0x80000000 and UV
 * 0x81000000 with 4096 bytes a line, are the IVDC's, which passes the
 * lines through a ring in DDR (0x30-0x38) to the encoder as the ISP sends
 * them: HERA or the JPEG engine on the T32 and T33, the Helix on the T23,
 * the EL200 on the T41. Those encoders' lists take the same addresses as
 * their source. Frame channel 0's frames carry no picture
 * then, and on the T32 and T33 their one "done" is this block's interrupt:
 * status 0x44 bit 17, a frame starting, with the sensor in 0xf0 bits 2:0.
 * The T23's and T41's drivers complete them on the ISP's own frame start
 * and only count this block's interrupts.
 *
 * The model takes a pulse on its input "frame" n, which the ISP model
 * sends in place of such a frame, as sensor n's frame start and end (bits
 * 17 and 16). 0x40 enables the status bits, writing ones to 0x54 clears
 * them, and a write to 0x0 with bit 0 or 1 set resets the block, done at
 * once. The encoders' drivers write sensor << 8 | 1 to 0x70 (the video
 * encoder) or 0x88 (JPEG) before a job and after it want that sensor back
 * in bits 6:4 of 0x7c or 0x94, with bits 31:30, the errors, clear.
 *
 * Not modelled: the ring and its overflows, the line threshold and half
 * done events, the SID FIFO at 0x158 that several sensors need.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/misc/ingenic-ivdc.h"

#define IVDC_CTRL           0x000
#define  CTRL_RESET         (BIT(0) | BIT(1))
#define IVDC_INT_EN         0x040
#define IVDC_INT_STA        0x044
#define  INT_FRM_DONE       BIT(16)
#define  INT_FRM_START      BIT(17)
#define IVDC_INT_CLR        0x054
#define IVDC_VPU0_START     0x070
#define IVDC_VPU0_STAT      0x07c
#define IVDC_VPU0_FIFO      0x080
#define IVDC_VPU1_START     0x088
#define IVDC_VPU1_STAT      0x094
#define IVDC_MAX_LINE       0x0ec
#define IVDC_SENSOR         0x0f0
#define IVDC_SID_FIFO       0x158

#define R(off)              (s->regs[(off) / 4])

static void ivdc_update(IngenicIvdcState *s)
{
    qemu_set_irq(s->irq, !!(R(IVDC_INT_STA) & R(IVDC_INT_EN)));
}

static void ivdc_frame_in(void *opaque, int n, int level)
{
    IngenicIvdcState *s = opaque;

    if (!level) {
        return;
    }
    R(IVDC_SENSOR) = n;
    R(IVDC_INT_STA) |= INT_FRM_START | INT_FRM_DONE;
    ivdc_update(s);
}

static uint64_t ingenic_ivdc_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicIvdcState *s = INGENIC_IVDC(opaque);

    if (addr == IVDC_INT_CLR) {
        return 0;
    }
    return R(addr);
}

static void ingenic_ivdc_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    IngenicIvdcState *s = INGENIC_IVDC(opaque);
    uint32_t v = value;

    switch (addr) {
    case IVDC_CTRL:
        if (v & CTRL_RESET) {
            R(IVDC_INT_STA) = 0;
            R(IVDC_VPU0_STAT) = R(IVDC_VPU1_STAT) = 0;
            R(IVDC_VPU0_FIFO) = R(IVDC_SID_FIFO) = R(IVDC_MAX_LINE) = 0;
            ivdc_update(s);
        }
        R(addr) = v & ~CTRL_RESET;
        return;
    case IVDC_INT_EN:
        R(addr) = v;
        ivdc_update(s);
        return;
    case IVDC_INT_STA:
        return;
    case IVDC_INT_CLR:
        R(IVDC_INT_STA) &= ~v;
        ivdc_update(s);
        return;
    case IVDC_VPU0_START:
    case IVDC_VPU1_START:
        R(addr) = v;
        if (v & 1) {
            R(addr == IVDC_VPU0_START ? IVDC_VPU0_STAT : IVDC_VPU1_STAT) =
                extract32(v, 8, 3) << 4;
        }
        return;
    }
    R(addr) = v;
}

static const MemoryRegionOps ingenic_ivdc_ops = {
    .read = ingenic_ivdc_read,
    .write = ingenic_ivdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_ivdc_reset_hold(Object *obj, ResetType type)
{
    IngenicIvdcState *s = INGENIC_IVDC(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void ingenic_ivdc_reset_exit(Object *obj, ResetType type)
{
    ivdc_update(INGENIC_IVDC(obj));
}

static void ingenic_ivdc_init(Object *obj)
{
    IngenicIvdcState *s = INGENIC_IVDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ivdc_ops, s,
                          TYPE_INGENIC_IVDC, INGENIC_IVDC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), ivdc_frame_in, "frame",
                            INGENIC_IVDC_SENSORS);
}

static const VMStateDescription vmstate_ingenic_ivdc = {
    .name = TYPE_INGENIC_IVDC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIvdcState, INGENIC_IVDC_IOSIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ivdc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic IVDC";
    dc->vmsd = &vmstate_ingenic_ivdc;
    rc->phases.hold = ingenic_ivdc_reset_hold;
    rc->phases.exit = ingenic_ivdc_reset_exit;
}

static const TypeInfo ingenic_ivdc_types[] = {
    {
        .name = TYPE_INGENIC_IVDC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicIvdcState),
        .instance_init = ingenic_ivdc_init,
        .class_init = ingenic_ivdc_class_init,
    },
};

DEFINE_TYPES(ingenic_ivdc_types)
