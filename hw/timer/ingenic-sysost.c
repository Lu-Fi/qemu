/*
 * Ingenic System OS Timer (the standalone OST at 0x12000000), as a T21N
 * reads it
 *
 * Two channels count EXCLK (24 MHz), each through its own prescaler in
 * TCCR (bits 1:0 for T1, 3:2 for T2: 0 is /1, 1 /4, 2 /16 and 3 /1):
 *   T1: 32 bits, from 0 to T1DFR and over again, setting TFR bit 0 as it
 *       reaches T1DFR; unmasked in TMR, that raises the IRQ.
 *   T2: 64 bits, free-running. Reading T2CNTL latches the upper half
 *       into T2CNTHBUF.
 * Both count EXCLK unless the "clk" input is wired: a T31's second OST,
 * at 0x12220000, counts L2CLK (696 MHz from an APLL of 1392 MHz and
 * L2CDIV 1, 464 MHz with L2CDIV 2) through CLKGR0 bit 26. Gated, as a
 * T31X has it, its counts hold, it reads 0 and it keeps no write.
 * TESR (0x34) sets and TECR (0x38) clears the enables TER reads; a
 * stopped count holds and resumes from where it stood, and TCR clears
 * it. TMR resets to 1 and T1DFR to 0xffffffff. The registers repeat every
 * 0x100 over the window ("size"): a T21N's at 0x12000000 over 1 MiB, the
 * second at 0x12220000 over 64 KiB, a T31X's second over 4 KiB.
 *
 * Not modelled: what a write to T1DFR does to a running count (it clears
 * it here, as the kernels always clear it next).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/timer/ingenic-sysost.h"
#include "migration/vmstate.h"

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

#define OST_EXCLK_HZ    24000000ULL

#define CH_T1           0
#define CH_T2           1
#define BIT_T1          (1u << CH_T1)
#define BIT_T2          (1u << CH_T2)

static uint64_t sysost_hz(IngenicSysOstState *s, int ch)
{
    static const unsigned div[] = { 1, 4, 16, 1 };
    uint64_t hz = clock_has_source(s->clk) ? clock_get_hz(s->clk)
                                           : OST_EXCLK_HZ;

    return hz / div[extract32(s->tccr, ch * 2, 2)];
}

/* Wired to a clock that is stopped */
static bool sysost_gated(IngenicSysOstState *s)
{
    return clock_has_source(s->clk) && !clock_get_hz(s->clk);
}

static uint64_t sysost_t1_period(IngenicSysOstState *s)
{
    return (uint64_t)s->t1dfr + 1;
}

/* The count as it now stands, unwrapped */
static uint64_t sysost_raw(IngenicSysOstState *s, int ch)
{
    IngenicSysOstChannel *c = &s->ch[ch];
    int64_t ns;

    if (!(s->ter & (1u << ch))) {
        return c->base;
    }
    ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->base_ns;
    return c->base + muldiv64(ns, sysost_hz(s, ch), NANOSECONDS_PER_SECOND);
}

static uint64_t sysost_count(IngenicSysOstState *s, int ch)
{
    uint64_t raw = sysost_raw(s, ch);

    return ch == CH_T1 ? raw % sysost_t1_period(s) : raw;
}

/* Restart the count's clock at @count, now */
static void sysost_rebase(IngenicSysOstState *s, int ch, uint64_t count)
{
    s->ch[ch].base = count;
    s->ch[ch].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* T2's count now, and its rate (0 while stopped), for a count kept elsewhere */
uint64_t ingenic_sysost_t2(IngenicSysOstState *s, uint64_t *hz)
{
    *hz = s->ter & BIT_T2 ? sysost_hz(s, CH_T2) : 0;
    return sysost_raw(s, CH_T2);
}

void ingenic_sysost_set_notify(IngenicSysOstState *s, void (*fn)(void *),
                               void *opaque)
{
    s->notify = fn;
    s->notify_opaque = opaque;
}

static void sysost_notify(IngenicSysOstState *s)
{
    if (s->notify) {
        s->notify(s->notify_opaque);
    }
}

static void sysost_update_irq(IngenicSysOstState *s)
{
    qemu_set_irq(s->irq, !!(s->tfr & ~s->tmr & BIT_T1));
}

static void sysost_t1_arm(IngenicSysOstState *s)
{
    uint64_t period = sysost_t1_period(s);
    uint64_t left;

    if (!(s->ter & BIT_T1) || sysost_gated(s)) {
        timer_del(s->t1_timer);
        return;
    }
    /* T1DFR is reached as the count goes from T1DFR - 1 to it */
    left = s->t1dfr - sysost_count(s, CH_T1);
    if (!left || left > period) {
        left = period;
    }
    timer_mod(s->t1_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              muldiv64(left, NANOSECONDS_PER_SECOND, sysost_hz(s, CH_T1)) +
              1);
}

static void sysost_t1_fire(void *opaque)
{
    IngenicSysOstState *s = opaque;

    s->tfr |= BIT_T1;
    sysost_update_irq(s);
    sysost_t1_arm(s);
}

static uint64_t ingenic_sysost_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    IngenicSysOstState *s = INGENIC_SYSOST(opaque);
    uint64_t count;

    if (sysost_gated(s)) {
        return 0;
    }
    switch (offset & (INGENIC_SYSOST_IOSIZE - 1)) {
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
        return sysost_count(s, CH_T1);
    case OST_T2CNTL:
        count = sysost_count(s, CH_T2);
        s->t2_high_buf = count >> 32;
        return (uint32_t)count;
    case OST_T2CNTH:
        return sysost_count(s, CH_T2) >> 32;
    case OST_TCNT2HBUF:
        return s->t2_high_buf;
    default:
        return 0;
    }
}

static void ingenic_sysost_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    IngenicSysOstState *s = INGENIC_SYSOST(opaque);
    uint32_t v = value;
    uint32_t change;
    bool t2 = false;            /* T2's rate changes or its count restarts */

    if (sysost_gated(s)) {
        return;
    }
    switch (offset & (INGENIC_SYSOST_IOSIZE - 1)) {
    case OST_TCCR:
        /* A count changing rate goes on from where it stands */
        for (int ch = 0; ch < 2; ch++) {
            if (extract32(s->tccr ^ v, ch * 2, 2)) {
                sysost_rebase(s, ch, sysost_count(s, ch));
            }
        }
        t2 = extract32(s->tccr ^ v, CH_T2 * 2, 2);
        s->tccr = v & 0xf;
        break;
    case OST_TESR:
    case OST_TECR:
        change = (offset & (INGENIC_SYSOST_IOSIZE - 1)) == OST_TESR
                 ? v & ~s->ter : v & s->ter;
        change &= BIT_T1 | BIT_T2;
        /* A stopped count holds; started, it goes on from there */
        for (int ch = 0; ch < 2; ch++) {
            if (change & (1u << ch)) {
                sysost_rebase(s, ch, sysost_count(s, ch));
            }
        }
        s->ter ^= change;
        t2 = change & BIT_T2;
        break;
    case OST_TCR:
        for (int ch = 0; ch < 2; ch++) {
            if (v & (1u << ch)) {
                sysost_rebase(s, ch, 0);
            }
        }
        t2 = v & BIT_T2;
        break;
    case OST_TFR:
        /* The kernels clear the flag with writel(~bit) */
        s->tfr &= v;
        break;
    case OST_TMR:
        s->tmr = v & BIT_T1;
        break;
    case OST_T1DFR:
        s->t1dfr = v;
        sysost_rebase(s, CH_T1, 0);
        break;
    default:
        break;
    }
    sysost_update_irq(s);
    sysost_t1_arm(s);
    if (t2) {
        sysost_notify(s);
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

    s->tccr = 0;
    s->ter = 0;
    s->tfr = 0;
    s->tmr = BIT_T1;
    s->t1dfr = UINT32_MAX;
    s->t2_high_buf = 0;
    for (int ch = 0; ch < 2; ch++) {
        sysost_rebase(s, ch, 0);
    }
    timer_del(s->t1_timer);
    qemu_set_irq(s->irq, 0);
    sysost_notify(s);
}

/* A count changing rate goes on from where it stands */
static void sysost_clk_update(void *opaque, ClockEvent event)
{
    IngenicSysOstState *s = opaque;

    if (event == ClockPreUpdate) {
        for (int ch = 0; ch < 2; ch++) {
            sysost_rebase(s, ch, sysost_count(s, ch));
        }
        return;
    }
    sysost_t1_arm(s);
    sysost_notify(s);
}

static void ingenic_sysost_realize(DeviceState *dev, Error **errp)
{
    IngenicSysOstState *s = INGENIC_SYSOST(dev);

    if (s->size < INGENIC_SYSOST_IOSIZE ||
        s->size % INGENIC_SYSOST_IOSIZE) {
        error_setg(errp, "size must be a multiple of 0x%x",
                   INGENIC_SYSOST_IOSIZE);
        return;
    }
    memory_region_init_io(&s->iomem, OBJECT(dev), &ingenic_sysost_ops, s,
                          TYPE_INGENIC_SYSOST, s->size);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    s->t1_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sysost_t1_fire, s);
}

static void ingenic_sysost_init(Object *obj)
{
    IngenicSysOstState *s = INGENIC_SYSOST(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", sysost_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
}

static int ingenic_sysost_post_load(void *opaque, int version_id)
{
    sysost_notify(opaque);
    return 0;
}

static const VMStateDescription vmstate_ingenic_sysost_channel = {
    .name = TYPE_INGENIC_SYSOST "/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base_ns, IngenicSysOstChannel),
        VMSTATE_UINT64(base, IngenicSysOstChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_sysost = {
    .name = TYPE_INGENIC_SYSOST,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_sysost_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_CLOCK(clk, IngenicSysOstState),
        VMSTATE_UINT32(tccr, IngenicSysOstState),
        VMSTATE_UINT32(ter, IngenicSysOstState),
        VMSTATE_UINT32(tfr, IngenicSysOstState),
        VMSTATE_UINT32(tmr, IngenicSysOstState),
        VMSTATE_UINT32(t1dfr, IngenicSysOstState),
        VMSTATE_STRUCT_ARRAY(ch, IngenicSysOstState, 2, 1,
                             vmstate_ingenic_sysost_channel,
                             IngenicSysOstChannel),
        VMSTATE_TIMER_PTR(t1_timer, IngenicSysOstState),
        VMSTATE_UINT32(t2_high_buf, IngenicSysOstState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_sysost_props[] = {
    DEFINE_PROP_UINT32("size", IngenicSysOstState, size,
                       INGENIC_SYSOST_IOSIZE),
};

static void ingenic_sysost_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_sysost_realize;
    dc->vmsd = &vmstate_ingenic_sysost;
    device_class_set_props(dc, ingenic_sysost_props);
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
