/*
 * Ingenic XBurst2 PWM controller
 *
 * The dedicated PWM block (16 channels) at 0x13460000. PWM output is
 * not observable under emulation, so the channels are modelled as
 * plain configuration registers; only the enable set/clear aliases and
 * the always-idle busy register need active handling so the Linux
 * pwm-ingenic-v2 driver's enable/update sequence completes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-pwm.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

#define PWM_ENS     0x10    /* channel enable set */
#define PWM_ENC     0x14    /* channel enable clear */
#define PWM_EN      0x18    /* channel enable status */
#define PWM_UP      0x20    /* update trigger (self-clearing) */
#define PWM_BUSY    0x24    /* update busy status */
#define PWM_DES     0x100   /* DMA enable set */
#define PWM_DEC     0x104   /* DMA enable clear */
#define PWM_DE      0x108   /* DMA enable status */

#define PWM_IOSIZE  0x1000

static uint64_t pwm_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicPwmState *s = opaque;

    switch (offset) {
    case PWM_BUSY:
        /* updates apply instantly under emulation */
        return 0;
    case PWM_UP:
        return 0;
    default:
        break;
    }

    if (offset / 4 < INGENIC_PWM_NUM_REGS) {
        return s->regs[offset / 4];
    }
    return 0;
}

static void pwm_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicPwmState *s = opaque;
    uint32_t v = val;

    switch (offset) {
    case PWM_ENS:
        s->regs[PWM_EN / 4] |= v;
        return;
    case PWM_ENC:
        s->regs[PWM_EN / 4] &= ~v;
        return;
    case PWM_DES:
        s->regs[PWM_DE / 4] |= v;
        return;
    case PWM_DEC:
        s->regs[PWM_DE / 4] &= ~v;
        return;
    case PWM_EN:
    case PWM_DE:
    case PWM_BUSY:
        /* status mirrors, driven through the set/clear aliases */
        return;
    case PWM_UP:
        return;
    default:
        break;
    }

    if (offset / 4 < INGENIC_PWM_NUM_REGS) {
        s->regs[offset / 4] = v;
    }
}

static const MemoryRegionOps pwm_ops = {
    .read = pwm_read,
    .write = pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void pwm_reset_hold(Object *obj, ResetType type)
{
    IngenicPwmState *s = INGENIC_PWM(obj);

    memset(s->regs, 0, sizeof(s->regs));
    qemu_set_irq(s->irq, 0);
}

static void pwm_init(Object *obj)
{
    IngenicPwmState *s = INGENIC_PWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pwm_ops, s,
                          TYPE_INGENIC_PWM, PWM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_pwm = {
    .name = TYPE_INGENIC_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicPwmState, INGENIC_PWM_NUM_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static void pwm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_pwm;
    rc->phases.hold = pwm_reset_hold;
}

static const TypeInfo pwm_type_info = {
    .name = TYPE_INGENIC_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicPwmState),
    .instance_init = pwm_init,
    .class_init = pwm_class_init,
};

static void pwm_register_types(void)
{
    type_register_static(&pwm_type_info);
}

type_init(pwm_register_types)
