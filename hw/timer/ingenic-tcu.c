/*
 * Ingenic XBurst Timer/Counter Unit (TCU)
 *
 * Models the 8 general-purpose timer/PWM channels and the TCU global
 * control registers. Each channel has a 16-bit up-counter that wraps at
 * its full-match value, raising the shared TCU interrupt on full-match
 * (and, at the same instant, the half-match flag).
 *
 * The watchdog (channel 16) and the legacy OST occupy the same 0x1000
 * page and are modelled elsewhere; this device only owns the channel
 * and global register window at TCU base + 0x10.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/timer/ingenic-tcu.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

/* Register offsets, relative to the MMIO window (TCU base + 0x10) */
#define TCU_TER     0x00    /* timer enable (read) */
#define TCU_TESR    0x04    /* timer enable set */
#define TCU_TECR    0x08    /* timer enable clear */
#define TCU_TSR     0x0c    /* timer stop (read) */
#define TCU_TFR     0x10    /* flags */
#define TCU_TFSR    0x14    /* flag set */
#define TCU_TFCR    0x18    /* flag clear */
#define TCU_TSSR    0x1c    /* timer stop set */
#define TCU_TMR     0x20    /* flag mask */
#define TCU_TMSR    0x24    /* flag mask set */
#define TCU_TMCR    0x28    /* flag mask clear */
#define TCU_TSCR    0x2c    /* timer stop clear */
#define TCU_CH_BASE 0x30    /* per-channel registers start here */
#define TCU_CH_TDFR 0x00
#define TCU_CH_TDHR 0x04
#define TCU_CH_TCNT 0x08
#define TCU_CH_TCSR 0x0c
#define TCU_CH_STRIDE 0x10

/* TCSR bits */
#define TCSR_PCK_EN     (1U << 0)
#define TCSR_RTC_EN     (1U << 1)
#define TCSR_EXT_EN     (1U << 2)
#define TCSR_PRESCALE_SHIFT 3
#define TCSR_PRESCALE_MASK  (7U << TCSR_PRESCALE_SHIFT)

/* Flag bits in TFR/TMR: full-match for channel N at bit N, half at N+16 */
#define TCU_FULL_BIT(n)  (1U << (n))
#define TCU_HALF_BIT(n)  (1U << ((n) + 16))
#define TCU_FLAG_MASK    0x00ff00ffU

/* Clock source frequencies (Hz) */
#define TCU_FREQ_EXT     24000000
#define TCU_FREQ_RTC     32768
#define TCU_FREQ_PCK     24000000

static uint32_t tcu_chn_freq(const IngenicTcuChannel *c)
{
    uint32_t src;
    unsigned prescale;

    if (c->tcsr & TCSR_EXT_EN) {
        src = TCU_FREQ_EXT;
    } else if (c->tcsr & TCSR_RTC_EN) {
        src = TCU_FREQ_RTC;
    } else if (c->tcsr & TCSR_PCK_EN) {
        src = TCU_FREQ_PCK;
    } else {
        return 0;
    }

    prescale = (c->tcsr & TCSR_PRESCALE_MASK) >> TCSR_PRESCALE_SHIFT;
    if (prescale > 5) {
        prescale = 5;
    }
    return src >> (2 * prescale);
}

static bool tcu_chn_running(const IngenicTcuState *s, unsigned n)
{
    return (s->ter & (1U << n)) && !(s->tsr & (1U << n));
}

static uint32_t tcu_chn_period(const IngenicTcuChannel *c)
{
    return (uint32_t)c->tdfr + 1;
}

/* Current counter value for channel n. */
static uint16_t tcu_chn_count(IngenicTcuState *s, unsigned n, int64_t now)
{
    IngenicTcuChannel *c = &s->chn[n];
    uint32_t freq = tcu_chn_freq(c);
    uint32_t period = tcu_chn_period(c);

    if (!tcu_chn_running(s, n) || freq == 0) {
        return c->count_base;
    }

    int64_t elapsed = now - c->base_ns;
    uint64_t ticks = (uint64_t)elapsed * freq / NANOSECONDS_PER_SECOND;
    return (c->count_base + ticks) % period;
}

/* Resample count_base/base_ns so the channel resumes from the right value. */
static void tcu_chn_sync(IngenicTcuState *s, unsigned n, int64_t now)
{
    s->chn[n].count_base = tcu_chn_count(s, n, now);
    s->chn[n].base_ns = now;
}

static void tcu_update_irq(IngenicTcuState *s)
{
    qemu_set_irq(s->irq, !!(s->tfr & ~s->tmr & TCU_FLAG_MASK));
}

static void tcu_chn_reschedule(IngenicTcuState *s, unsigned n)
{
    IngenicTcuChannel *c = &s->chn[n];
    uint32_t freq = tcu_chn_freq(c);
    uint32_t period = tcu_chn_period(c);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!tcu_chn_running(s, n) || freq == 0) {
        timer_del(c->timer);
        return;
    }

    uint32_t cur = tcu_chn_count(s, n, now);
    uint32_t ticks = period - cur;     /* ticks until the next wrap */
    int64_t delay = (int64_t)ticks * NANOSECONDS_PER_SECOND / freq;
    timer_mod(c->timer, now + (delay > 0 ? delay : 1));
}

static void tcu_chn_fire(void *opaque)
{
    IngenicTcuChannel *c = opaque;
    IngenicTcuState *s = c->tcu;

    /* A full period elapsed: raise full-match and half-match flags. */
    s->tfr |= TCU_FULL_BIT(c->index) | TCU_HALF_BIT(c->index);
    tcu_update_irq(s);
    tcu_chn_reschedule(s, c->index);
}

static uint64_t tcu_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicTcuState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    switch (offset) {
    case TCU_TER:
    case TCU_TESR:
        return s->ter;
    case TCU_TSR:
    case TCU_TSSR:
        return s->tsr;
    case TCU_TFR:
    case TCU_TFSR:
        return s->tfr;
    case TCU_TMR:
    case TCU_TMSR:
        return s->tmr;
    default:
        break;
    }

    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + INGENIC_TCU_NUM_CHANNELS * TCU_CH_STRIDE) {
        unsigned n = (offset - TCU_CH_BASE) / TCU_CH_STRIDE;
        unsigned reg = (offset - TCU_CH_BASE) % TCU_CH_STRIDE;

        switch (reg) {
        case TCU_CH_TDFR:
            return s->chn[n].tdfr;
        case TCU_CH_TDHR:
            return s->chn[n].tdhr;
        case TCU_CH_TCNT:
            return tcu_chn_count(s, n, now);
        case TCU_CH_TCSR:
            return s->chn[n].tcsr;
        }
    }
    return 0;
}

static void tcu_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicTcuState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = val;

    switch (offset) {
    case TCU_TESR:
        for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
            if ((v & (1U << n)) && !(s->ter & (1U << n))) {
                s->ter |= (1U << n);
                s->chn[n].base_ns = now;
                tcu_chn_reschedule(s, n);
            }
        }
        return;
    case TCU_TECR:
        for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
            if (v & (1U << n)) {
                tcu_chn_sync(s, n, now);
                s->ter &= ~(1U << n);
                tcu_chn_reschedule(s, n);
            }
        }
        return;
    case TCU_TSSR:
        for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
            if (v & (1U << n)) {
                tcu_chn_sync(s, n, now);
                s->tsr |= (1U << n);
                tcu_chn_reschedule(s, n);
            }
        }
        return;
    case TCU_TSCR:
        for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
            if ((v & (1U << n)) && (s->tsr & (1U << n))) {
                s->tsr &= ~(1U << n);
                s->chn[n].base_ns = now;
                tcu_chn_reschedule(s, n);
            }
        }
        return;
    case TCU_TFSR:
        s->tfr |= v & TCU_FLAG_MASK;
        tcu_update_irq(s);
        return;
    case TCU_TFCR:
        s->tfr &= ~v;
        tcu_update_irq(s);
        return;
    case TCU_TMSR:
        s->tmr |= v & TCU_FLAG_MASK;
        tcu_update_irq(s);
        return;
    case TCU_TMCR:
        s->tmr &= ~v;
        tcu_update_irq(s);
        return;
    case TCU_TER:
    case TCU_TSR:
    case TCU_TFR:
    case TCU_TMR:
        /* read-only mirrors; programmed via the set/clear registers */
        return;
    default:
        break;
    }

    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + INGENIC_TCU_NUM_CHANNELS * TCU_CH_STRIDE) {
        unsigned n = (offset - TCU_CH_BASE) / TCU_CH_STRIDE;
        unsigned reg = (offset - TCU_CH_BASE) % TCU_CH_STRIDE;

        switch (reg) {
        case TCU_CH_TDFR:
            s->chn[n].tdfr = v;
            tcu_chn_reschedule(s, n);
            break;
        case TCU_CH_TDHR:
            s->chn[n].tdhr = v;
            break;
        case TCU_CH_TCNT:
            s->chn[n].count_base = v;
            s->chn[n].base_ns = now;
            tcu_chn_reschedule(s, n);
            break;
        case TCU_CH_TCSR:
            tcu_chn_sync(s, n, now);
            s->chn[n].tcsr = v;
            tcu_chn_reschedule(s, n);
            break;
        }
    }
}

static const MemoryRegionOps tcu_ops = {
    .read = tcu_read,
    .write = tcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void tcu_reset_hold(Object *obj, ResetType type)
{
    IngenicTcuState *s = INGENIC_TCU(obj);

    s->ter = 0;
    s->tsr = 0;
    s->tfr = 0;
    s->tmr = TCU_FLAG_MASK;     /* all flags masked at reset */
    for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
        s->chn[n].tdfr = 0xffff;
        s->chn[n].tdhr = 0;
        s->chn[n].tcsr = 0;
        s->chn[n].count_base = 0;
        s->chn[n].base_ns = 0;
        timer_del(s->chn[n].timer);
    }
    qemu_set_irq(s->irq, 0);
}

static void tcu_init(Object *obj)
{
    IngenicTcuState *s = INGENIC_TCU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tcu_ops, s,
                          TYPE_INGENIC_TCU, INGENIC_TCU_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
        s->chn[n].tcu = s;
        s->chn[n].index = n;
        s->chn[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       tcu_chn_fire, &s->chn[n]);
    }
}

static const VMStateDescription vmstate_tcu_channel = {
    .name = "ingenic-tcu/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(tdfr, IngenicTcuChannel),
        VMSTATE_UINT16(tdhr, IngenicTcuChannel),
        VMSTATE_UINT16(tcsr, IngenicTcuChannel),
        VMSTATE_UINT16(count_base, IngenicTcuChannel),
        VMSTATE_INT64(base_ns, IngenicTcuChannel),
        VMSTATE_TIMER_PTR(timer, IngenicTcuChannel),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_tcu = {
    .name = TYPE_INGENIC_TCU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ter, IngenicTcuState),
        VMSTATE_UINT32(tsr, IngenicTcuState),
        VMSTATE_UINT32(tfr, IngenicTcuState),
        VMSTATE_UINT32(tmr, IngenicTcuState),
        VMSTATE_STRUCT_ARRAY(chn, IngenicTcuState, INGENIC_TCU_NUM_CHANNELS,
                             1, vmstate_tcu_channel, IngenicTcuChannel),
        VMSTATE_END_OF_LIST()
    },
};

static void tcu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_tcu;
    rc->phases.hold = tcu_reset_hold;
}

static const TypeInfo tcu_type_info = {
    .name = TYPE_INGENIC_TCU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicTcuState),
    .instance_init = tcu_init,
    .class_init = tcu_class_init,
};

static void tcu_register_types(void)
{
    type_register_static(&tcu_type_info);
}

type_init(tcu_register_types)
