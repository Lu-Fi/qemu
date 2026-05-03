/*
 * Ingenic T31 Interrupt Controller (INTC) emulation
 *
 * Two banks of 32 IRQ sources at 0x20-byte stride. Each bank has:
 *   0x00 ISR  - source register (level of input from peripheral)
 *   0x04 IMR  - mask register (1 = masked, blocks delivery)
 *   0x08 IMSR - write-1-to-set into IMR
 *   0x0c IMCR - write-1-to-clear into IMR
 *   0x10 IPR  - pending after mask (ISR & ~IMR), read-only
 *
 * Inputs are level-triggered GPIO lines. The output goes high when
 * any (ISR & ~IMR) bit is set across either bank, and is wired to
 * MIPS hardware interrupt IP2 (CAUSEF_IP2).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/intc/ingenic-t31-intc.h"

#define INTC_BANK_OFF       0x20
#define INTC_ISR            0x00
#define INTC_IMR            0x04
#define INTC_IMSR           0x08
#define INTC_IMCR           0x0c
#define INTC_IPR            0x10

static void ingenic_t31_intc_update(IngenicT31IntcState *s)
{
    unsigned i;
    bool active = false;

    for (i = 0; i < INGENIC_T31_INTC_NR_BANKS; i++) {
        if (s->isr[i] & ~s->imr[i]) {
            active = true;
            break;
        }
    }
    qemu_set_irq(s->parent_irq, active);
}

static void ingenic_t31_intc_input(void *opaque, int irq, int level)
{
    IngenicT31IntcState *s = INGENIC_T31_INTC(opaque);
    unsigned bank = irq / 32;
    uint32_t mask = 1u << (irq % 32);

    if (bank >= INGENIC_T31_INTC_NR_BANKS) {
        return;
    }
    if (level) {
        s->isr[bank] |= mask;
    } else {
        s->isr[bank] &= ~mask;
    }
    ingenic_t31_intc_update(s);
}

static uint64_t ingenic_t31_intc_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicT31IntcState *s = INGENIC_T31_INTC(opaque);
    unsigned bank = offset / INTC_BANK_OFF;
    hwaddr off = offset & (INTC_BANK_OFF - 1);

    if (bank >= INGENIC_T31_INTC_NR_BANKS) {
        return 0;
    }

    switch (off) {
    case INTC_ISR:
        return s->isr[bank];
    case INTC_IMR:
    case INTC_IMSR:
    case INTC_IMCR:
        return s->imr[bank];
    case INTC_IPR:
        return s->isr[bank] & ~s->imr[bank];
    default:
        return 0;
    }
}

static void ingenic_t31_intc_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicT31IntcState *s = INGENIC_T31_INTC(opaque);
    unsigned bank = offset / INTC_BANK_OFF;
    hwaddr off = offset & (INTC_BANK_OFF - 1);
    uint32_t v = (uint32_t)value;

    if (bank >= INGENIC_T31_INTC_NR_BANKS) {
        return;
    }

    switch (off) {
    case INTC_ISR:
        /* Source register is read-only on real hardware (driven by
         * peripheral input lines). Ignore writes. */
        break;
    case INTC_IMR:
        s->imr[bank] = v;
        break;
    case INTC_IMSR:
        s->imr[bank] |= v;
        break;
    case INTC_IMCR:
        s->imr[bank] &= ~v;
        break;
    case INTC_IPR:
        /* Read-only on real hardware. */
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, offset, v);
        return;
    }
    ingenic_t31_intc_update(s);
}

static const MemoryRegionOps ingenic_t31_intc_ops = {
    .read = ingenic_t31_intc_read,
    .write = ingenic_t31_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_intc_reset_hold(Object *obj, ResetType type)
{
    IngenicT31IntcState *s = INGENIC_T31_INTC(obj);
    unsigned i;

    for (i = 0; i < INGENIC_T31_INTC_NR_BANKS; i++) {
        s->isr[i] = 0;
        s->imr[i] = 0xffffffff;   /* all sources masked at reset */
    }
    qemu_set_irq(s->parent_irq, 0);
}

static void ingenic_t31_intc_init(Object *obj)
{
    IngenicT31IntcState *s = INGENIC_T31_INTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_intc_ops, s,
                          TYPE_INGENIC_T31_INTC,
                          INGENIC_T31_INTC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->parent_irq);
    qdev_init_gpio_in(DEVICE(obj), ingenic_t31_intc_input,
                      INGENIC_T31_INTC_NR_IRQS);
}

static void ingenic_t31_intc_class_init(ObjectClass *oc, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.hold = ingenic_t31_intc_reset_hold;
}

static const TypeInfo ingenic_t31_intc_type_info = {
    .name = TYPE_INGENIC_T31_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31IntcState),
    .instance_init = ingenic_t31_intc_init,
    .class_init = ingenic_t31_intc_class_init,
};

static void ingenic_t31_intc_register_types(void)
{
    type_register_static(&ingenic_t31_intc_type_info);
}

type_init(ingenic_t31_intc_register_types)
