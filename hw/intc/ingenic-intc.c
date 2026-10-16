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
 *   0x34 DSR, 0x38 DMR, 0x3c DPR - the same for the PDMA, bank 0, with a
 *   mask of its own; bank 1 at 0x40-0x48
 *
 * Inputs are level-triggered GPIO lines. Each core's output goes high
 * while any (ISR & ~IMR[core]) bit is set, and feeds that core's IP2
 * interrupt path.
 *
 * XBurst2 SoCs also have the APB window at 0x10001000 ("apb-window",
 * INTC4 in the X2000 manual), as measured on the T41: the same sources
 * and its own masks, which no core takes interrupts from. It wakes the
 * chip from sleep and serves the PDMA's MCU, neither modelled.
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
#define INTC_DSR0           0x34
#define INTC_PDMA_STRIDE    0x0c
#define INTC_DSR            0x00
#define INTC_DMR            0x04
#define INTC_DPR            0x08

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

/* The PDMA registers, DSR0 to DPR1: their bank, or -1 */
static int intc_pdma_bank(hwaddr off)
{
    if (off < INTC_DSR0 ||
        off >= INTC_DSR0 + INGENIC_INTC_NR_BANKS * INTC_PDMA_STRIDE) {
        return -1;
    }
    return (off - INTC_DSR0) / INTC_PDMA_STRIDE;
}

static uint64_t intc_window_read(IngenicIntcState *s, unsigned w, hwaddr off)
{
    int pbank = intc_pdma_bank(off);
    unsigned bank = off / INTC_BANK_OFF;

    if (pbank >= 0) {
        switch ((off - INTC_DSR0) % INTC_PDMA_STRIDE) {
        case INTC_DSR:
            return s->isr[pbank];
        case INTC_DMR:
            return s->dmr[w][pbank];
        case INTC_DPR:
            return s->isr[pbank] & ~s->dmr[w][pbank];
        }
        return 0;
    }
    if (bank >= INGENIC_INTC_NR_BANKS) {
        return 0;
    }

    switch (off & (INTC_BANK_OFF - 1)) {
    case INTC_ISR:
        return s->isr[bank];
    case INTC_IMR:
    case INTC_IMSR:
    case INTC_IMCR:
        return s->imr[w][bank];
    case INTC_IPR:
        return s->isr[bank] & ~s->imr[w][bank];
    default:
        return 0;
    }
}

static void intc_window_write(IngenicIntcState *s, unsigned w, hwaddr off,
                              uint32_t v)
{
    int pbank = intc_pdma_bank(off);
    unsigned bank = off / INTC_BANK_OFF;

    if (pbank >= 0) {
        if ((off - INTC_DSR0) % INTC_PDMA_STRIDE == INTC_DMR) {
            s->dmr[w][pbank] = v;
        }
        return;
    }
    if (bank >= INGENIC_INTC_NR_BANKS) {
        return;
    }

    switch (off & (INTC_BANK_OFF - 1)) {
    case INTC_ISR:
    case INTC_IPR:
        /* Read-only: ISR is driven by peripheral lines, IPR is derived. */
        break;
    case INTC_IMR:
        s->imr[w][bank] = v;
        break;
    case INTC_IMSR:
        s->imr[w][bank] |= v;
        break;
    case INTC_IMCR:
        s->imr[w][bank] &= ~v;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, off, v);
        return;
    }
    ingenic_intc_update(s);
}

static uint64_t ingenic_intc_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);

    return intc_window_read(s, intc_cpu(s, offset), offset % INTC_CPU_OFF);
}

static void ingenic_intc_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);

    intc_window_write(s, intc_cpu(s, offset), offset % INTC_CPU_OFF, value);
}

static uint64_t ingenic_intc_apb_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);

    return offset < INTC_CPU_OFF ?
           intc_window_read(s, INGENIC_INTC_APB, offset) : 0;
}

static void ingenic_intc_apb_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicIntcState *s = INGENIC_INTC(opaque);

    if (offset < INTC_CPU_OFF) {
        intc_window_write(s, INGENIC_INTC_APB, offset, value);
    }
}

static const MemoryRegionOps ingenic_intc_ops = {
    .read = ingenic_intc_read,
    .write = ingenic_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static const MemoryRegionOps ingenic_intc_apb_ops = {
    .read = ingenic_intc_apb_read,
    .write = ingenic_intc_apb_write,
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
    for (unsigned w = 0; w < INGENIC_INTC_NR_WINDOWS; w++) {
        for (i = 0; i < INGENIC_INTC_NR_BANKS; i++) {
            s->imr[w][i] = 0xffffffff;   /* all sources masked at reset */
            s->dmr[w][i] = 0xffffffff;
        }
    }
    for (cpu = 0; cpu < INGENIC_INTC_MAX_CPUS; cpu++) {
        qemu_set_irq(s->parent_irq[cpu], 0);
    }
}

static void ingenic_intc_realize(DeviceState *dev, Error **errp)
{
    IngenicIntcState *s = INGENIC_INTC(dev);

    if (s->num_cpus < 1 || s->num_cpus > INGENIC_INTC_MAX_CPUS) {
        error_setg(errp, "num-cpus must be between 1 and %d",
                   INGENIC_INTC_MAX_CPUS);
        return;
    }
    if (s->apb_window) {
        memory_region_init_io(&s->apb, OBJECT(dev), &ingenic_intc_apb_ops, s,
                              TYPE_INGENIC_INTC "-apb",
                              INGENIC_INTC_APB_IOSIZE);
        sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->apb);
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
    DEFINE_PROP_BOOL("apb-window", IngenicIntcState, apb_window, false),
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
