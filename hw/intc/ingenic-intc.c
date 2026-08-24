/*
 * Ingenic T31 Interrupt Controller (INTC) emulation
 *
 * Two banks of 32 IRQ sources at 0x20-byte stride. On XBurst2 SoCs each
 * core has its own register window (0x100 stride) with an independent
 * mask, so an IRQ is delivered to whichever core has it unmasked; the
 * raw source levels are shared. XBurst1 SoCs are single-core (num-cpus
 * defaults to 1). Per-window registers:
 *   0x00 ISR  - source register (level of input from peripheral)
 *   0x04 IMR  - mask register (1 = masked, blocks delivery)
 *   0x08 IMSR - write-1-to-set into IMR
 *   0x0c IMCR - write-1-to-clear into IMR
 *   0x10 IPR  - pending after mask (ISR & ~IMR), read-only
 *
 * Inputs are level-triggered GPIO lines. Each core's output goes high
 * while any (ISR & ~IMR[core]) bit is set, and feeds that core's IP2
 * interrupt path.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/ingenic-intc.h"

#define INTC_CPU_OFF        0x100
#define INTC_BANK_OFF       0x20
#define INTC_ISR            0x00
#define INTC_IMR            0x04
#define INTC_IMSR           0x08
#define INTC_IMCR           0x0c
#define INTC_IPR            0x10

static void ingenic_intc_update(IngenicIntcState *s)
{
    for (unsigned cpu = 0; cpu < s->num_cpus; cpu++) {
        bool active = false;

        for (unsigned i = 0; i < INGENIC_INTC_NR_BANKS; i++) {
            if (s->isr[i] & ~s->imr[cpu][i]) {
                active = true;
                break;
            }
        }
        qemu_set_irq(s->parent_irq[cpu], active);
    }
}

static void ingenic_intc_input(void *opaque, int irq, int level)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);
    unsigned bank = irq / 32;
    uint32_t mask = 1u << (irq % 32);

    if (bank >= INGENIC_INTC_NR_BANKS) {
        return;
    }
    if (level) {
        s->isr[bank] |= mask;
    } else {
        s->isr[bank] &= ~mask;
    }
    ingenic_intc_update(s);
}

/* Resolve the per-CPU window; out-of-range windows fold onto core 0. */
static unsigned intc_cpu(IngenicIntcState *s, hwaddr offset)
{
    unsigned cpu = offset / INTC_CPU_OFF;

    return cpu < s->num_cpus ? cpu : 0;
}

static uint64_t ingenic_intc_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);
    unsigned cpu = intc_cpu(s, offset);
    unsigned bank = (offset % INTC_CPU_OFF) / INTC_BANK_OFF;
    hwaddr off = offset & (INTC_BANK_OFF - 1);

    if (bank >= INGENIC_INTC_NR_BANKS) {
        return 0;
    }

    switch (off) {
    case INTC_ISR:
        return s->isr[bank];
    case INTC_IMR:
    case INTC_IMSR:
    case INTC_IMCR:
        return s->imr[cpu][bank];
    case INTC_IPR:
        return s->isr[bank] & ~s->imr[cpu][bank];
    default:
        return 0;
    }
}

static void ingenic_intc_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);
    unsigned cpu = intc_cpu(s, offset);
    unsigned bank = (offset % INTC_CPU_OFF) / INTC_BANK_OFF;
    hwaddr off = offset & (INTC_BANK_OFF - 1);
    uint32_t v = (uint32_t)value;

    if (bank >= INGENIC_INTC_NR_BANKS) {
        return;
    }

    switch (off) {
    case INTC_ISR:
    case INTC_IPR:
        /* Read-only: ISR is driven by peripheral lines, IPR is derived. */
        break;
    case INTC_IMR:
        s->imr[cpu][bank] = v;
        break;
    case INTC_IMSR:
        s->imr[cpu][bank] |= v;
        break;
    case INTC_IMCR:
        s->imr[cpu][bank] &= ~v;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, offset, v);
        return;
    }
    ingenic_intc_update(s);
}

static const MemoryRegionOps ingenic_intc_ops = {
    .read = ingenic_intc_read,
    .write = ingenic_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_intc_reset_hold(Object *obj, ResetType type)
{
    IngenicIntcState *s = INGENIC_INTC(obj);
    unsigned cpu, i;

    for (i = 0; i < INGENIC_INTC_NR_BANKS; i++) {
        s->isr[i] = 0;
    }
    for (cpu = 0; cpu < INGENIC_INTC_MAX_CPUS; cpu++) {
        for (i = 0; i < INGENIC_INTC_NR_BANKS; i++) {
            s->imr[cpu][i] = 0xffffffff;   /* all sources masked at reset */
        }
        qemu_set_irq(s->parent_irq[cpu], 0);
    }
}

static void ingenic_intc_realize(DeviceState *dev, Error **errp)
{
    IngenicIntcState *s = INGENIC_INTC(dev);

    if (s->num_cpus < 1 || s->num_cpus > INGENIC_INTC_MAX_CPUS) {
        error_setg(errp, "num-cpus must be between 1 and %d",
                   INGENIC_INTC_MAX_CPUS);
    }
}

static void ingenic_intc_init(Object *obj)
{
    IngenicIntcState *s = INGENIC_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_intc_ops, s,
                          TYPE_INGENIC_INTC,
                          INGENIC_INTC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    for (unsigned cpu = 0; cpu < INGENIC_INTC_MAX_CPUS; cpu++) {
        sysbus_init_irq(sbd, &s->parent_irq[cpu]);
    }
    qdev_init_gpio_in(DEVICE(obj), ingenic_intc_input,
                      INGENIC_INTC_NR_IRQS);
}

static const Property ingenic_intc_properties[] = {
    DEFINE_PROP_UINT32("num-cpus", IngenicIntcState, num_cpus, 1),
};

static void ingenic_intc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_intc_realize;
    device_class_set_props(dc, ingenic_intc_properties);
    rc->phases.hold = ingenic_intc_reset_hold;
}

static const TypeInfo ingenic_intc_type_info = {
    .name = TYPE_INGENIC_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIntcState),
    .instance_init = ingenic_intc_init,
    .class_init = ingenic_intc_class_init,
};

static void ingenic_intc_register_types(void)
{
    type_register_static(&ingenic_intc_type_info);
}

type_init(ingenic_intc_register_types)
