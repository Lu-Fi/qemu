/*
 * Ingenic T31 System OS Timer (standalone OST at 0x12000000)
 *
 * Two channels:
 *   T1 (CH 0): 32-bit periodic. Reaches T1DFR -> sets TFR[0], raises
 *              IRQ to MIPS IP4 if TMR[0] is clear. Used for tick.
 *   T2 (CH 1): 64-bit free-running. Used as clocksource. Reading
 *              T2CNTL latches the upper 32 bits into TCNT2HBUF.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/timer/ingenic-sysost.h"

/* Register offsets */
#define OST_TCCR        0x00
#define OST_TER         0x04
#define OST_TCR         0x08
#define OST_TFR         0x0C
#define OST_TMR         0x10
#define OST_T1DFR       0x14
#define OST_T1CNT       0x18
#define OST_T2CNTH      0x1C
#define OST_T2CNTL      0x20
#define OST_TCNT2HBUF   0x24
#define OST_TESR        0x34
#define OST_TECR        0x38

/*
 * EXTAL/16 -> 1.5 MHz. The kernel computes its own mult/shift from
 * clk_get_rate("ext1") / CLKSOURCE_DIV, which we make match by
 * having ext1 = 24 MHz (CPM default) and the kernel's CLKSOURCE_DIV
 * = 16 hardcoded in timer_sys_ost.c.
 */
#define OST_FREQ        1500000ULL

#define CH_T1           0
#define CH_T2           1
#define BIT_T1          (1u << CH_T1)
#define BIT_T2          (1u << CH_T2)

static uint64_t sysost_t2_count(IngenicSysOstState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->t2_base_ns;

    return (uint64_t)(elapsed * OST_FREQ / NANOSECONDS_PER_SECOND);
}

static uint32_t sysost_t1_count(IngenicSysOstState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->t1_base_ns;
    uint32_t cycles = (uint32_t)(elapsed * OST_FREQ / NANOSECONDS_PER_SECOND);
    uint32_t period = s->t1dfr ? s->t1dfr + 1 : 0xffffffff;

    return s->t1_base_count + (cycles % period);
}

static void sysost_update_irq(IngenicSysOstState *s)
{
    bool active = (s->tfr & ~s->tmr & BIT_T1) != 0;
    qemu_set_irq(s->irq, active);
}

static void sysost_t1_arm(IngenicSysOstState *s)
{
    uint32_t period = s->t1dfr ? s->t1dfr + 1 : 0;
    int64_t now;
    int64_t fire;

    if (!(s->ter & BIT_T1) || period == 0) {
        timer_del(s->t1_timer);
        return;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    fire = s->t1_base_ns +
           (int64_t)period * NANOSECONDS_PER_SECOND / OST_FREQ;
    if (fire <= now) {
        fire = now + 1;
    }
    timer_mod(s->t1_timer, fire);
}

static void sysost_t1_fire(void *opaque)
{
    IngenicSysOstState *s = opaque;

    s->tfr |= BIT_T1;
    s->t1_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->t1_base_count = 0;
    sysost_update_irq(s);
    sysost_t1_arm(s);
}

static uint64_t ingenic_sysost_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    IngenicSysOstState *s = INGENIC_SYSOST(opaque);
    uint64_t cycles;

    switch (offset) {
    case OST_TCCR:
        return s->tccr;
    case OST_TER:
        return s->ter;
    case OST_TFR:
        return s->tfr;
    case OST_TMR:
        return s->tmr;
    case OST_T1DFR:
        return s->t1dfr;
    case OST_T1CNT:
        return sysost_t1_count(s);
    case OST_T2CNTL:
        cycles = sysost_t2_count(s);
        s->t2_high_buf = (uint32_t)(cycles >> 32);
        return (uint32_t)cycles;
    case OST_T2CNTH:
        return (uint32_t)(sysost_t2_count(s) >> 32);
    case OST_TCNT2HBUF:
        return s->t2_high_buf;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }
}

static void ingenic_sysost_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    IngenicSysOstState *s = INGENIC_SYSOST(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case OST_TCCR:
        s->tccr = v;
        break;

    case OST_TESR:
        if ((v & BIT_T1) && !(s->ter & BIT_T1)) {
            s->t1_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->t1_base_count = 0;
        }
        if ((v & BIT_T2) && !(s->ter & BIT_T2)) {
            s->t2_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        s->ter |= v;
        sysost_t1_arm(s);
        break;

    case OST_TECR:
        s->ter &= ~v;
        sysost_t1_arm(s);
        break;

    case OST_TCR:
        if (v & BIT_T1) {
            s->t1_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->t1_base_count = 0;
            sysost_t1_arm(s);
        }
        if (v & BIT_T2) {
            s->t2_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        break;

    case OST_TFR:
        /*
         * Kernel clears flag with `writel(~ctrlbit)`. Treat any
         * write whose bit is 0 as a clear request for that bit.
         */
        s->tfr &= v;
        sysost_update_irq(s);
        break;

    case OST_TMR:
        s->tmr = v;
        sysost_update_irq(s);
        break;

    case OST_T1DFR:
        s->t1dfr = v;
        s->t1_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->t1_base_count = 0;
        sysost_t1_arm(s);
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, offset, v);
        break;
    }
}

static const MemoryRegionOps ingenic_sysost_ops = {
    .read = ingenic_sysost_read,
    .write = ingenic_sysost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_sysost_reset_hold(Object *obj, ResetType type)
{
    IngenicSysOstState *s = INGENIC_SYSOST(obj);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->tccr = 0;
    s->ter = 0;
    s->tfr = 0;
    s->tmr = 0;
    s->t1dfr = 0;
    s->t1_base_ns = now;
    s->t1_base_count = 0;
    s->t2_base_ns = now;
    s->t2_high_buf = 0;
    timer_del(s->t1_timer);
    qemu_set_irq(s->irq, 0);
}

static void ingenic_sysost_realize(DeviceState *dev, Error **errp)
{
    IngenicSysOstState *s = INGENIC_SYSOST(dev);

    s->t1_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sysost_t1_fire, s);
}

static void ingenic_sysost_init(Object *obj)
{
    IngenicSysOstState *s = INGENIC_SYSOST(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_sysost_ops, s,
                          TYPE_INGENIC_SYSOST,
                          INGENIC_SYSOST_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_sysost_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_sysost_realize;
    rc->phases.hold = ingenic_sysost_reset_hold;
}

static const TypeInfo ingenic_sysost_type_info = {
    .name = TYPE_INGENIC_SYSOST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicSysOstState),
    .instance_init = ingenic_sysost_init,
    .class_init = ingenic_sysost_class_init,
};

static void ingenic_sysost_register_types(void)
{
    type_register_static(&ingenic_sysost_type_info);
}

type_init(ingenic_sysost_register_types)
