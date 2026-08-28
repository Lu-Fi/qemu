/*
 * Ingenic XBurst Timer/Counter Unit (TCU)
 *
 * One register page (0x100 bytes) shared by four closely-related
 * functions:
 *   - the watchdog (channel 16) at the low 16 bytes - on full-match it
 *     resets the SoC;
 *   - 8 general-purpose timer/PWM channels (16-bit up-counters that
 *     wrap at a full-match value and raise the shared TCU interrupt);
 *   - the TCU global enable/flag/mask registers driving those channels;
 *   - the embedded OST, a 64-bit free-running counter U-Boot and the
 *     kernel use as a clocksource.
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
#include "system/runstate.h"
#include "system/watchdog.h"

/* Watchdog (channel 16) registers - low 16 bytes of the page */
#define WDT_TDR     0x00    /* full-match (reset) value */
#define WDT_TCER    0x04    /* counter enable, bit 0 */
#define WDT_TCNT    0x08    /* counter */
#define WDT_TCSR    0x0c    /* clock source / prescaler */

#define WDT_TCER_EN          (1U << 0)
#define WDT_TCSR_PCK_EN      (1U << 0)
#define WDT_TCSR_RTC_EN      (1U << 1)
#define WDT_TCSR_EXT_EN      (1U << 2)
#define WDT_TCSR_PRESCALE_SHIFT  3
#define WDT_TCSR_PRESCALE_MASK   (7U << WDT_TCSR_PRESCALE_SHIFT)

/* TCU global registers */
#define TCU_TER     0x10    /* timer enable (read) */
#define TCU_TESR    0x14    /* timer enable set */
#define TCU_TECR    0x18    /* timer enable clear */
#define TCU_TSR     0x1c    /* timer stop (read) */
#define TCU_TFR     0x20    /* flags */
#define TCU_TFSR    0x24    /* flag set */
#define TCU_TFCR    0x28    /* flag clear */
#define TCU_TSSR    0x2c    /* timer stop set */
#define TCU_TMR     0x30    /* flag mask */
#define TCU_TMSR    0x34    /* flag mask set */
#define TCU_TMCR    0x38    /* flag mask clear */
#define TCU_TSCR    0x3c    /* timer stop clear */

/* Per-channel registers: 8 channels of 0x10 bytes starting at 0x40 */
#define TCU_CH_BASE   0x40
#define TCU_CH_STRIDE 0x10
#define CH_TDFR       0x00
#define CH_TDHR       0x04
#define CH_TCNT       0x08
#define CH_TCSR       0x0c

/* Embedded OST registers */
#define TCU_OST_DR      0xe0
#define TCU_OST_CNTL    0xe4
#define TCU_OST_CNTH    0xe8
#define TCU_OST_CSR     0xec
#define TCU_OST_CNTHBUF 0xfc

/* OST strobe registers */
#define TCU_TSTR    0xf0
#define TCU_TSTSR   0xf4
#define TCU_TSTCR   0xf8

#define TCU_IOSIZE  0x100

/* OST clock: EXTAL(24 MHz) / prescale(4) */
#define OST_FREQ    6000000ULL

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

/* Channel clock source frequencies (Hz) */
#define TCU_FREQ_EXT     24000000
#define TCU_FREQ_RTC     32768
#define TCU_FREQ_PCK     24000000

/* ---- Watchdog (channel 16) -------------------------------------------- */

static uint32_t wdt_tick_ns(uint16_t tcsr)
{
    uint64_t rate = TCU_FREQ_RTC;       /* RTC is the reset default */
    if (tcsr & WDT_TCSR_EXT_EN) {
        rate = TCU_FREQ_EXT;
    } else if (tcsr & WDT_TCSR_PCK_EN) {
        rate = 100000000;               /* PCK, approximate */
    }
    static const uint32_t pre[8] = {1, 4, 16, 64, 256, 1024, 1024, 1024};
    uint32_t prescale = pre[(tcsr >> WDT_TCSR_PRESCALE_SHIFT) & 7];
    return (uint32_t)((NANOSECONDS_PER_SECOND * (uint64_t)prescale) / rate);
}

static uint16_t wdt_current_tcnt(IngenicTcuState *s)
{
    if (!(s->wdt_tcer & WDT_TCER_EN)) {
        return s->wdt_tcnt_base;
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->wdt_start_ns;
    uint32_t period = wdt_tick_ns(s->wdt_tcsr);
    if (period == 0) {
        return s->wdt_tcnt_base;
    }
    uint64_t ticks = (uint64_t)elapsed / period;
    uint64_t cnt = (uint64_t)s->wdt_tcnt_base + ticks;
    return cnt > 0xFFFF ? 0xFFFF : (uint16_t)cnt;
}

static void wdt_reschedule(IngenicTcuState *s)
{
    timer_del(s->wdt_timer);
    if (!(s->wdt_tcer & WDT_TCER_EN)) {
        return;
    }
    if (s->wdt_tcnt_base >= s->wdt_tdr) {
        timer_mod(s->wdt_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        return;
    }
    uint32_t period = wdt_tick_ns(s->wdt_tcsr);
    uint64_t remaining = (uint64_t)(s->wdt_tdr - s->wdt_tcnt_base) * period;
    timer_mod(s->wdt_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + remaining);
}

static void wdt_fire(void *opaque)
{
    IngenicTcuState *s = opaque;

    s->wdt_tcer &= ~WDT_TCER_EN;
    watchdog_perform_action();
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void wdt_latch(IngenicTcuState *s)
{
    s->wdt_tcnt_base = wdt_current_tcnt(s);
    s->wdt_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* ---- OST -------------------------------------------------------------- */

static uint64_t tcu_ost_count(IngenicTcuState *s)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->ost_base_ns;

    return (uint64_t)elapsed * OST_FREQ / NANOSECONDS_PER_SECOND;
}

/* ---- TCU channels ----------------------------------------------------- */

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

static void tcu_reschedule_all(IngenicTcuState *s)
{
    for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
        tcu_chn_reschedule(s, n);
    }
}

/* ---- MMIO ------------------------------------------------------------- */

static uint64_t tcu_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicTcuState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t count;

    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + INGENIC_TCU_NUM_CHANNELS * TCU_CH_STRIDE) {
        unsigned n = (offset - TCU_CH_BASE) / TCU_CH_STRIDE;
        switch ((offset - TCU_CH_BASE) % TCU_CH_STRIDE) {
        case CH_TDFR: return s->chn[n].tdfr;
        case CH_TDHR: return s->chn[n].tdhr;
        case CH_TCNT: return tcu_chn_count(s, n, now);
        case CH_TCSR: return s->chn[n].tcsr;
        }
        return 0;
    }

    switch (offset) {
    case WDT_TDR:
        return s->wdt_tdr;
    case WDT_TCER:
        return s->wdt_tcer;
    case WDT_TCNT:
        return wdt_current_tcnt(s);
    case WDT_TCSR:
        return s->wdt_tcsr;
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
    case TCU_TSTR:
        return s->tstr;
    case TCU_OST_CNTL:
        count = tcu_ost_count(s);
        s->ost_cnth_buf = (uint32_t)(count >> 32);
        return (uint32_t)count;
    case TCU_OST_CNTH:
    case TCU_OST_CNTHBUF:
        return s->ost_cnth_buf;
    case TCU_OST_DR:
        return s->ost_data;
    case TCU_OST_CSR:
        return s->ost_csr;
    default:
        return 0;
    }
}

static void tcu_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicTcuState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = val;

    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + INGENIC_TCU_NUM_CHANNELS * TCU_CH_STRIDE) {
        unsigned n = (offset - TCU_CH_BASE) / TCU_CH_STRIDE;
        switch ((offset - TCU_CH_BASE) % TCU_CH_STRIDE) {
        case CH_TDFR:
            s->chn[n].tdfr = v;
            tcu_chn_reschedule(s, n);
            break;
        case CH_TDHR:
            s->chn[n].tdhr = v;
            break;
        case CH_TCNT:
            s->chn[n].count_base = v;
            s->chn[n].base_ns = now;
            tcu_chn_reschedule(s, n);
            break;
        case CH_TCSR:
            tcu_chn_sync(s, n, now);
            s->chn[n].tcsr = v;
            tcu_chn_reschedule(s, n);
            break;
        }
        return;
    }

    switch (offset) {
    case WDT_TDR:
        wdt_latch(s);
        s->wdt_tdr = v;
        wdt_reschedule(s);
        return;
    case WDT_TCER:
        wdt_latch(s);
        s->wdt_tcer = v;
        wdt_reschedule(s);
        return;
    case WDT_TCNT:
        /* Linux's jz4740_wdt_ping writes 0 here to kick the dog. */
        s->wdt_tcnt_base = v;
        s->wdt_start_ns = now;
        wdt_reschedule(s);
        return;
    case WDT_TCSR:
        wdt_latch(s);
        /*
         * Bit 10 is the XBurst2 counter-clear strobe: the vendor
         * kernel's watchdog ping is TCSR |= (1 << 10), not a TCNT
         * write. Self-clearing; zeroes the running count.
         */
        if (v & (1U << 10)) {
            s->wdt_tcnt_base = 0;
            s->wdt_start_ns = now;
        }
        s->wdt_tcsr = v & ~(1U << 10);
        wdt_reschedule(s);
        return;
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
    case TCU_TER:
        s->ter = v;
        tcu_reschedule_all(s);
        return;
    case TCU_TSR:
        s->tsr = v;
        tcu_reschedule_all(s);
        return;
    case TCU_TFR:
        s->tfr = v & TCU_FLAG_MASK;
        tcu_update_irq(s);
        return;
    case TCU_TFSR:
        s->tfr |= v & TCU_FLAG_MASK;
        tcu_update_irq(s);
        return;
    case TCU_TFCR:
        s->tfr &= ~v;
        tcu_update_irq(s);
        return;
    case TCU_TMR:
        s->tmr = v & TCU_FLAG_MASK;
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
    case TCU_TSTR:
        s->tstr = v;
        return;
    case TCU_TSTSR:
        s->tstr |= v;
        return;
    case TCU_TSTCR:
        s->tstr &= ~v;
        return;
    case TCU_OST_DR:
        s->ost_data = v;
        return;
    case TCU_OST_CNTL:
        s->ost_base_ns = now;       /* writing the low word restarts the OST */
        return;
    case TCU_OST_CSR:
        s->ost_csr = v;
        return;
    default:
        return;
    }
}

static const MemoryRegionOps tcu_ops = {
    .read = tcu_read,
    .write = tcu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

static void tcu_reset_hold(Object *obj, ResetType type)
{
    IngenicTcuState *s = INGENIC_TCU(obj);

    s->ost_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->ost_cnth_buf = 0;
    s->ost_data = 0;
    s->ost_csr = 0;

    s->ter = 0;
    s->tsr = 0;
    s->tfr = 0;
    s->tmr = TCU_FLAG_MASK;     /* all flags masked at reset */
    s->tstr = 0;

    s->wdt_tdr = 0;
    s->wdt_tcnt_base = 0;
    s->wdt_tcsr = 0;
    s->wdt_tcer = 0;
    s->wdt_start_ns = 0;
    timer_del(s->wdt_timer);

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
                          TYPE_INGENIC_TCU, TCU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->wdt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_fire, s);

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
        VMSTATE_UINT16(wdt_tdr, IngenicTcuState),
        VMSTATE_UINT16(wdt_tcnt_base, IngenicTcuState),
        VMSTATE_UINT16(wdt_tcsr, IngenicTcuState),
        VMSTATE_UINT8(wdt_tcer, IngenicTcuState),
        VMSTATE_INT64(wdt_start_ns, IngenicTcuState),
        VMSTATE_TIMER_PTR(wdt_timer, IngenicTcuState),
        VMSTATE_INT64(ost_base_ns, IngenicTcuState),
        VMSTATE_UINT32(ost_cnth_buf, IngenicTcuState),
        VMSTATE_UINT32(ost_data, IngenicTcuState),
        VMSTATE_UINT32(ost_csr, IngenicTcuState),
        VMSTATE_UINT32(ter, IngenicTcuState),
        VMSTATE_UINT32(tsr, IngenicTcuState),
        VMSTATE_UINT32(tfr, IngenicTcuState),
        VMSTATE_UINT32(tmr, IngenicTcuState),
        VMSTATE_UINT32(tstr, IngenicTcuState),
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
