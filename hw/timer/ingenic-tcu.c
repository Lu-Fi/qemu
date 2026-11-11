/*
 * Ingenic XBurst Timer/Counter Unit (TCU)
 *
 * One register page (0x100 bytes) shared by four closely-related
 * functions:
 *   - the watchdog (channel 16) at the low 16 bytes - on full-match it
 *     resets the SoC;
 *   - 4 or 8 general-purpose timer/PWM channels (16-bit up-counters that
 *     wrap at a full-match value and raise the shared TCU interrupt);
 *   - the TCU global enable/flag/mask registers driving those channels;
 *   - the embedded OST, a 64-bit counter U-Boot uses as a clocksource:
 *     it counts while TER's bit 15 is set and OSTCSR picks a clock, and
 *     holds otherwise, as a T10L, T20X and T31X keep its bootrom values
 *     under Linux, which leaves it off.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/timer/ingenic-tcu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/log.h"
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
#define TCSR_PWM_EN     (1U << 7)
#define TCSR_INITL      (1U << 8)
#define TCSR_SD         (1U << 9)
#define TCSR_CLOCK      (TCSR_PCK_EN | TCSR_RTC_EN | TCSR_EXT_EN | \
                         TCSR_PRESCALE_MASK)

/* Flag bits in TFR/TMR: full-match for channel N at bit N, half at N+16 */
#define TCU_FULL_BIT(n)  (1U << (n))
#define TCU_HALF_BIT(n)  (1U << ((n) + 16))
#define TCU_CHN_FLAGS    0x00ff00ffU
#define TCU_HFLAGW       (1U << 24)  /* the watchdog's half match (X2000 PM) */
#define TCU_WDTS         (1U << 16)  /* TSR: the watchdog's clock stopped */

/* Channel clock source frequencies (Hz) */
#define TCU_FREQ_EXT     24000000
#define TCU_FREQ_PCK     24000000

/* ---- Watchdog (channel 16) -------------------------------------------- */

/*
 * The input clock TCSR selects, 0 for none: the counter stands. RTC_EN's is
 * EXCLK/512, OPCR.ERCS left 0 (no vendor code sets it), as the T21 manual
 * has it and a T31X's, T33L's, T40N's and T41NQ's count against wall time
 */
static uint32_t wdt_rate(IngenicTcuState *s)
{
    if (s->wdt_tcsr & WDT_TCSR_EXT_EN) {
        return TCU_FREQ_EXT;
    } else if (s->wdt_tcsr & WDT_TCSR_PCK_EN) {
        return 100000000;               /* PCK, approximate */
    } else if (s->wdt_tcsr & WDT_TCSR_RTC_EN) {
        return s->rtc_hz;
    }
    return 0;
}

static uint32_t wdt_prescale(uint16_t tcsr)
{
    static const uint32_t pre[8] = {1, 4, 16, 64, 256, 1024, 1024, 1024};

    return pre[(tcsr >> WDT_TCSR_PRESCALE_SHIFT) & 7];
}

/*
 * It counts while TSR's WDTS has its clock on, TCSR picks one and TCER is
 * set; a T20, T21, T23, T30, T31, T40 or T41 ("wdt-tcer-runs") counts on
 * with TCER clear too, as the T21 manual says and those parts measure, and
 * TCER then only gates the reset.
 *
 * A T41's ("wdt-tdr-resumes") WDTS holds the count only until TDR is
 * written: the count then runs on from where it stood, a clear written
 * while it was held is dropped, and TCNT still reads the held count until
 * TSCR. Without a TDR write the count stays held, and a clear written
 * meanwhile lands as TSCR lets the clock run. Its vendor kernel's restart
 * relies on that: it stops the watchdog, then writes a clear, TDR 0x100 and
 * TCER, and is reset 0x100 counts after its last feed (a T41NQ, 3 ms to 1 s
 * after the feed). From a watchdog stopped long before, a T41NQ is reset
 * about 0.3 s later than this has it (still unexplained)
 */
static bool wdt_running(IngenicTcuState *s)
{
    return ((s->wdt_tcer & WDT_TCER_EN) || s->wdt_tcer_runs) &&
           (!(s->tsr & TCU_WDTS) || s->wdt_run_held) && wdt_rate(s);
}

static uint16_t wdt_current_tcnt(IngenicTcuState *s)
{
    if (!wdt_running(s)) {
        return s->wdt_tcnt_base;
    }
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->wdt_start_ns;
    uint64_t ticks = muldiv64(elapsed, wdt_rate(s), NANOSECONDS_PER_SECOND) /
                     wdt_prescale(s->wdt_tcsr);

    return (uint16_t)(s->wdt_tcnt_base + ticks);
}

/*
 * The reset comes as TCNT counts up to TDR. A T21, T23, T30, T31 or T40 is
 * reset only at that match: a TDR written below the count waits for the
 * 16-bit counter to wrap round to it. A T32, T33 or T41 ("wdt-reset-above")
 * is reset at its next count with TDR at or below it, unless a clear comes
 * first: the T33's vendor kernel writes TDR below the count U-Boot's left
 * and clears it straight after, which a T33 survives
 */
static void wdt_reschedule(IngenicTcuState *s)
{
    uint16_t ticks = s->wdt_tdr - s->wdt_tcnt_base;
    uint32_t rate = wdt_rate(s);
    uint64_t cycles;

    if (s->wdt_reset_above && s->wdt_tcnt_base >= s->wdt_tdr) {
        ticks = 1;
    }
    cycles = (uint64_t)ticks * wdt_prescale(s->wdt_tcsr);

    timer_del(s->wdt_timer);
    if (!wdt_running(s) || !(s->wdt_tcer & WDT_TCER_EN)) {
        return;
    }
    timer_mod(s->wdt_timer, s->wdt_start_ns +
              (cycles * NANOSECONDS_PER_SECOND + rate - 1) / rate);
}

static void wdt_fire(void *opaque)
{
    IngenicTcuState *s = opaque;

    s->wdt_tcer &= ~WDT_TCER_EN;
    /* the CPM's RSR records a watchdog reset */
    if (get_watchdog_action() == WATCHDOG_ACTION_RESET) {
        qemu_irq_pulse(s->wdt_reset);
    }
    /* a reset unless -action watchdog= says otherwise */
    watchdog_perform_action();
}

static void wdt_latch(IngenicTcuState *s)
{
    s->wdt_tcnt_base = wdt_current_tcnt(s);
    s->wdt_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* ---- OST -------------------------------------------------------------- */

#define TCU_OSTEN       (1U << 15)  /* TER: the OST counts */
#define TCU_OSTS        (1U << 15)  /* TSR: its clock stopped */
#define TCU_OSTFLAG     (1U << 15)  /* TFR: OSTCNT = OSTDR */
#define OST_CSR_CLOCKS  7           /* PCK, RTC and EXT enables */

static bool tcu_ost_running(IngenicTcuState *s)
{
    return (s->ter & TCU_OSTEN) && !(s->tsr & TCU_OSTS) &&
           (s->ost_csr & OST_CSR_CLOCKS);
}

static uint64_t tcu_ost_count(IngenicTcuState *s)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->ost_base_ns;

    if (!tcu_ost_running(s)) {
        return s->ost_held;
    }
    return s->ost_held + (uint64_t)elapsed * OST_FREQ / NANOSECONDS_PER_SECOND;
}

/*
 * OSTFLAG, set where the count's low word has passed OSTDR since it was
 * last looked at; on a start or a write of the count, a count already at
 * OSTDR matches (U-Boot leaves OSTDR 0 and starts the count from 0).
 * U-Boot polls the count, so this is checked as time is seen rather than
 * timed.
 */
static void tcu_ost_flag(IngenicTcuState *s, bool start)
{
    uint64_t cur = tcu_ost_count(s);
    uint64_t at = deposit64(s->ost_seen, 0, 32, s->ost_data);

    if (at <= s->ost_seen) {
        at += 1ULL << 32;
    }
    if ((cur > s->ost_seen && at <= cur) ||
        (start && (uint32_t)cur == s->ost_data && tcu_ost_running(s))) {
        s->tfr |= s->flags & TCU_OSTFLAG;
    }
    s->ost_seen = cur;
}

/* The count so far into ost_held, from now on: before it starts or stops */
static void tcu_ost_latch(IngenicTcuState *s, int64_t now)
{
    tcu_ost_flag(s, false);
    s->ost_held = tcu_ost_count(s);
    s->ost_base_ns = now;
}

/* ---- TCU channels ----------------------------------------------------- */

/*
 * A channel counts from 0 to TDFR and wraps, TDFR + 1 ticks a period. Its
 * full match comes as the count leaves TDFR, its half match as it leaves
 * TDHR (with the full one where TDHR is TDFR; never where it is past it).
 * The PWM output is INITL while PWM_EN is clear; set, it holds its level
 * but for the matches: the half one sets it to the other level, the full
 * one back to INITL, so a period is INITL from 0 to TDHR and the other
 * level above. A write to TCNT or INITL moves nothing until a match. TECR
 * stops it at once with SD set; without, it counts on to its full match and
 * halts at 0, while TER already reads clear. TSSR freezes it where it is,
 * and TESR or TSCR carry on from there. TER and TSR drop writes. All as a
 * T31X and a T40XP do at U-Boot.
 *
 * The flags are raised as the count is looked at, and a timer runs only for
 * a match whose interrupt is unmasked or a stop that waits on one.
 */

/* The clock TCSR picks, before its prescaler, 0 for none */
static uint32_t tcu_chn_src(const IngenicTcuChannel *c)
{
    if (c->tcsr & TCSR_EXT_EN) {
        return TCU_FREQ_EXT;
    } else if (c->tcsr & TCSR_RTC_EN) {
        /* the watchdog's RTC_EN clock: a T20X's, T31X's and T40N's count it */
        return c->tcu->rtc_hz;
    } else if (c->tcsr & TCSR_PCK_EN) {
        return TCU_FREQ_PCK;
    }
    return 0;
}

/* The prescaler's divisor: RTC_EN at /1024 is no whole number of Hz */
static uint32_t tcu_chn_div(const IngenicTcuChannel *c)
{
    unsigned prescale = (c->tcsr & TCSR_PRESCALE_MASK) >> TCSR_PRESCALE_SHIFT;

    return 1U << (2 * MIN(prescale, 5));
}

static uint32_t tcu_chn_period(const IngenicTcuChannel *c)
{
    return (uint32_t)c->tdfr + 1;
}

static bool tcu_chn_active(const IngenicTcuState *s, unsigned n)
{
    const IngenicTcuChannel *c = &s->chn[n];

    return ((s->ter & (1U << n)) || c->stopping) && !(s->tsr & (1U << n)) &&
           tcu_chn_src(c);
}

/* The whole ticks counted since base_ns */
static uint64_t tcu_chn_ticks(const IngenicTcuChannel *c, int64_t now)
{
    return now > c->base_ns ? muldiv64(now - c->base_ns, tcu_chn_src(c),
                                       NANOSECONDS_PER_SECOND) /
                              tcu_chn_div(c) : 0;
}

/* When tick k after base_ns lands */
static int64_t tcu_chn_tick_ns(const IngenicTcuChannel *c, uint64_t k)
{
    return c->base_ns + muldiv64_round_up(k * tcu_chn_div(c),
                                          NANOSECONDS_PER_SECOND,
                                          tcu_chn_src(c));
}

static uint32_t tcu_chn_pos(const IngenicTcuChannel *c, uint64_t k)
{
    return (c->count_base + k) % tcu_chn_period(c);
}

/* The first tick from @from on that the count reads @at, if it ever does */
static uint64_t tcu_chn_find(const IngenicTcuChannel *c, uint64_t from,
                             uint32_t at)
{
    uint32_t period = tcu_chn_period(c);

    if (at >= period) {
        return UINT64_MAX;
    }
    return from + (at + period - tcu_chn_pos(c, from)) % period;
}

/* The last tick before @upto, from @from on, that the count reads @at */
static uint64_t tcu_chn_last(const IngenicTcuChannel *c, uint64_t from,
                             uint64_t upto, uint32_t at)
{
    uint32_t period = tcu_chn_period(c);
    uint64_t back;

    if (at >= period || upto <= from) {
        return UINT64_MAX;
    }
    back = (tcu_chn_pos(c, upto - 1) + period - at) % period;
    return back <= upto - 1 - from ? upto - 1 - back : UINT64_MAX;
}

/*
 * Raise the matches the count has passed since it was last looked at, and
 * set the output by the last of them
 */
static void tcu_chn_account(IngenicTcuState *s, unsigned n, int64_t now)
{
    IngenicTcuChannel *c = &s->chn[n];
    uint64_t upto, full, half;

    if (!tcu_chn_active(s, n)) {
        return;
    }
    upto = tcu_chn_ticks(c, now);
    full = tcu_chn_find(c, c->seen, c->tdfr);
    if (c->stopping && full < upto) {
        upto = full + 1;
    }
    half = tcu_chn_last(c, c->seen, upto, c->tdhr);
    if (full < upto) {
        s->tfr |= s->flags & TCU_FULL_BIT(n);
        full = tcu_chn_last(c, c->seen, upto, c->tdfr);
    }
    if (half != UINT64_MAX) {
        s->tfr |= s->flags & TCU_HALF_BIT(n);
    }
    if ((c->tcsr & TCSR_PWM_EN) && (full < upto || half != UINT64_MAX)) {
        c->out = !!(c->tcsr & TCSR_INITL) ^
                 (half != UINT64_MAX && (full >= upto || half > full));
    }
    if (c->stopping && full < upto) {
        c->base_ns = tcu_chn_tick_ns(c, upto);
        c->count_base = 0;
        c->seen = 0;
        c->stopping = false;
        return;
    }
    c->seen = upto;
}

static uint16_t tcu_chn_count(IngenicTcuState *s, unsigned n, int64_t now)
{
    IngenicTcuChannel *c = &s->chn[n];

    tcu_chn_account(s, n, now);
    if (!tcu_chn_active(s, n)) {
        return c->count_base;
    }
    return tcu_chn_pos(c, tcu_chn_ticks(c, now));
}

/* Re-base at the last whole tick, before the count's rules change */
static void tcu_chn_sync(IngenicTcuState *s, unsigned n, int64_t now)
{
    IngenicTcuChannel *c = &s->chn[n];
    uint64_t k;

    tcu_chn_account(s, n, now);
    if (!tcu_chn_active(s, n)) {
        return;
    }
    k = tcu_chn_ticks(c, now);
    c->count_base = tcu_chn_pos(c, k);
    c->base_ns = tcu_chn_tick_ns(c, k);
    c->seen = 0;
}

/* Start counting from count_base, now */
static void tcu_chn_start(IngenicTcuState *s, unsigned n, int64_t now)
{
    s->chn[n].base_ns = now;
    s->chn[n].seen = 0;
}

static void tcu_update_irq(IngenicTcuState *s)
{
    qemu_set_irq(s->irq, !!(s->tfr & ~s->tmr & s->flags & TCU_CHN_FLAGS));
}

static void tcu_chn_reschedule(IngenicTcuState *s, unsigned n)
{
    IngenicTcuChannel *c = &s->chn[n];
    uint32_t want = s->flags & ~s->tmr & ~s->tfr &
                    (TCU_FULL_BIT(n) | TCU_HALF_BIT(n));
    uint64_t at = UINT64_MAX;

    bool back = (c->tcsr & TCSR_PWM_EN) && c->tdhr >= c->tdfr &&
                c->out != !!(c->tcsr & TCSR_INITL);

    if (tcu_chn_active(s, n)) {
        if ((want & TCU_FULL_BIT(n)) || c->stopping || back) {
            at = tcu_chn_find(c, c->seen, c->tdfr);
        }
        if (want & TCU_HALF_BIT(n)) {
            at = MIN(at, tcu_chn_find(c, c->seen, c->tdhr));
        }
    }
    if (at == UINT64_MAX) {
        timer_del(c->timer);
        return;
    }
    timer_mod(c->timer, tcu_chn_tick_ns(c, at + 1));
}

static void tcu_pwm_out(IngenicTcuState *s, unsigned n, IngenicPwmOut *o,
                        IngenicPwmKey *key)
{
    IngenicTcuChannel *c = &s->chn[n];
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t initl = !!(c->tcsr & TCSR_INITL);
    uint32_t src = tcu_chn_src(c), div = tcu_chn_div(c);
    uint64_t k;

    tcu_chn_account(s, n, now);
    memset(o, 0, sizeof(*o));
    memset(key, 0, sizeof(*key));
    o->level = c->tcsr & TCSR_PWM_EN ? c->out : initl;
    if ((c->tcsr & TCSR_PWM_EN) && tcu_chn_active(s, n) &&
        c->tdhr < c->tdfr) {
        k = tcu_chn_ticks(c, now);
        o->period_ns = muldiv64((uint64_t)tcu_chn_period(c) * div,
                                NANOSECONDS_PER_SECOND, src);
        o->high_ns = muldiv64((uint64_t)(initl ? c->tdhr + 1 :
                                         c->tdfr - c->tdhr) * div,
                              NANOSECONDS_PER_SECOND, src);
        o->edge_ns = tcu_chn_tick_ns(c, tcu_chn_find(c, k, c->out == initl ?
                                                     c->tdhr : c->tdfr) + 1);
        key->k[0] = c->tdfr | (uint64_t)c->tdhr << 16 |
                    (uint64_t)initl << 32 | 1ULL << 33;
        key->k[1] = (uint64_t)src << 16 | div;
        key->k[2] = c->base_ns;
        key->k[3] = c->count_base;
        return;
    }
    key->k[0] = o->level;
}

static void tcu_pwm_trace(IngenicTcuState *s, unsigned n)
{
    IngenicPwmOut o;
    IngenicPwmKey key;

    if (!qemu_chr_fe_backend_connected(&s->trace) || n >= s->channels) {
        return;
    }
    tcu_pwm_out(s, n, &o, &key);
    if (memcmp(&key, &s->chn[n].traced, sizeof(key))) {
        s->chn[n].traced = key;
        ingenic_pwm_out_trace(&s->trace, "tcu", n, &o);
    }
}

static void tcu_chn_fire(void *opaque)
{
    IngenicTcuChannel *c = opaque;
    IngenicTcuState *s = c->tcu;

    tcu_chn_account(s, c->index, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    tcu_update_irq(s);
    tcu_pwm_trace(s, c->index);
    tcu_chn_reschedule(s, c->index);
}

static void tcu_account_all(IngenicTcuState *s, int64_t now)
{
    for (unsigned n = 0; n < s->channels; n++) {
        tcu_chn_account(s, n, now);
    }
}

/* After a write that may change any channel's count, flags or output */
static void tcu_update_all(IngenicTcuState *s)
{
    tcu_update_irq(s);
    for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
        tcu_chn_reschedule(s, n);
        tcu_pwm_trace(s, n);
    }
}

/* ---- MMIO ------------------------------------------------------------- */

static uint64_t tcu_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicTcuState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t count;

    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + s->channels * TCU_CH_STRIDE) {
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
        if (s->wdt_tdr_resumes && (s->tsr & TCU_WDTS)) {
            return s->wdt_held;
        }
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
        tcu_ost_flag(s, false);
        tcu_account_all(s, now);
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
        offset < TCU_CH_BASE + s->channels * TCU_CH_STRIDE) {
        unsigned n = (offset - TCU_CH_BASE) / TCU_CH_STRIDE;
        IngenicTcuChannel *c = &s->chn[n];

        switch ((offset - TCU_CH_BASE) % TCU_CH_STRIDE) {
        case CH_TDFR:
            tcu_chn_sync(s, n, now);
            c->tdfr = v;
            break;
        case CH_TDHR:
            tcu_chn_account(s, n, now);
            c->tdhr = v;
            break;
        case CH_TCNT:
            tcu_chn_account(s, n, now);
            c->count_base = v;
            tcu_chn_start(s, n, now);
            break;
        case CH_TCSR:
            if ((v ^ c->tcsr) & TCSR_CLOCK) {
                tcu_chn_sync(s, n, now);
                c->tcsr = v;
                tcu_chn_start(s, n, now);
            } else {
                tcu_chn_account(s, n, now);
                c->tcsr = v;
            }
            if (!(v & TCSR_PWM_EN)) {
                c->out = !!(v & TCSR_INITL);
            }
            break;
        }
        tcu_update_irq(s);
        tcu_chn_reschedule(s, n);
        tcu_pwm_trace(s, n);
        return;
    }

    switch (offset) {
    case WDT_TDR:
        wdt_latch(s);
        s->wdt_tdr = v;
        if (s->wdt_tdr_resumes && (s->tsr & TCU_WDTS) && !s->wdt_run_held) {
            s->wdt_run_held = true;
            s->wdt_clear_pending = false;
            s->wdt_start_ns = now;
        }
        wdt_reschedule(s);
        return;
    case WDT_TCER:
        /* where TCER only gates the reset, the count carries on as it is */
        if (!s->wdt_tcer_runs) {
            wdt_latch(s);
        }
        s->wdt_tcer = v;
        wdt_reschedule(s);
        return;
    case WDT_TCNT:
        /*
         * Linux's jz4740_wdt_ping writes 0 here; a stopped clock drops it,
         * and a T32's or T33's TCNT takes no write at all
         */
        if ((s->tsr & TCU_WDTS) || s->wdt_tcnt_ro) {
            return;
        }
        s->wdt_tcnt_base = v;
        s->wdt_start_ns = now;
        wdt_reschedule(s);
        return;
    case WDT_TCSR:
        wdt_latch(s);
        /*
         * Bit 10 is the XBurst2 counter-clear strobe: the vendor
         * kernel's watchdog ping is TCSR |= (1 << 10), not a TCNT
         * write. Self-clearing; zeroes the running count. Where TFR
         * has HFLAGW (T32, T41, A1) a clear sets it, as a T41NQ's does
         * at each ping and at nothing else its count passes
         */
        /*
         * A T32's or T41's clear lands at its next count, after the compare:
         * with TDR at or below the count it comes too late (a T33's is at
         * once). With WDTS holding a T41's count it waits for the clock
         */
        if ((v & (1U << 10)) && s->wdt_tdr_resumes &&
            (s->tsr & TCU_WDTS) && !s->wdt_run_held) {
            s->wdt_clear_pending = true;
        } else if ((v & (1U << 10)) &&
            !(s->wdt_clear_at_tick && s->wdt_reset_above && wdt_running(s) &&
              s->wdt_tcnt_base >= s->wdt_tdr)) {
            s->wdt_tcnt_base = 0;
            s->wdt_start_ns = now;
            s->tfr |= s->flags & TCU_HFLAGW;
        }
        /* a T32 or T33 has no EXTAL input: bit 2 reads 0 */
        s->wdt_tcsr = v & ~(1U << 10);
        if (s->wdt_no_ext) {
            s->wdt_tcsr &= ~WDT_TCSR_EXT_EN;
        }
        wdt_reschedule(s);
        return;
    case TCU_TESR:
        tcu_ost_latch(s, now);
        s->ter |= v & TCU_OSTEN;
        tcu_ost_flag(s, true);
        for (unsigned n = 0; n < s->channels; n++) {
            if ((v & (1U << n)) && !(s->ter & (1U << n))) {
                tcu_chn_account(s, n, now);
                if (s->chn[n].stopping) {
                    s->chn[n].stopping = false;
                } else {
                    tcu_chn_start(s, n, now);
                }
                s->ter |= (1U << n);
            }
        }
        tcu_update_all(s);
        return;
    case TCU_TECR:
        tcu_ost_latch(s, now);
        s->ter &= ~(v & TCU_OSTEN);
        for (unsigned n = 0; n < s->channels; n++) {
            if ((v & (1U << n)) && (s->ter & (1U << n))) {
                if (s->chn[n].tcsr & TCSR_SD) {
                    tcu_chn_sync(s, n, now);
                } else {
                    tcu_chn_account(s, n, now);
                    s->chn[n].stopping = true;
                }
                s->ter &= ~(1U << n);
            }
        }
        tcu_update_all(s);
        return;
    case TCU_TSSR:
        tcu_ost_latch(s, now);
        wdt_latch(s);
        if (v & TCU_WDTS) {
            s->wdt_held = s->wdt_tcnt_base;
            s->wdt_run_held = false;
        }
        s->tsr |= v & (TCU_OSTS | TCU_WDTS);
        wdt_reschedule(s);
        for (unsigned n = 0; n < s->channels; n++) {
            if (v & (1U << n)) {
                tcu_chn_sync(s, n, now);
                s->tsr |= (1U << n);
            }
        }
        tcu_update_all(s);
        return;
    case TCU_TSCR:
        tcu_ost_latch(s, now);
        wdt_latch(s);
        s->tsr &= ~(v & (TCU_OSTS | TCU_WDTS));
        /* the clear lands at the first count after (here as the clock runs) */
        if (!(s->tsr & TCU_WDTS)) {
            s->wdt_run_held = false;
        }
        if (s->wdt_clear_pending && !(s->tsr & TCU_WDTS)) {
            s->wdt_clear_pending = false;
            s->wdt_tcnt_base = 0;
            s->wdt_start_ns = now;
            s->tfr |= s->flags & TCU_HFLAGW;
        }
        wdt_reschedule(s);
        tcu_ost_flag(s, true);
        for (unsigned n = 0; n < s->channels; n++) {
            if ((v & (1U << n)) && (s->tsr & (1U << n))) {
                s->tsr &= ~(1U << n);
                tcu_chn_start(s, n, now);
            }
        }
        tcu_update_all(s);
        return;
    case TCU_TFR:
        tcu_account_all(s, now);
        s->tfr = v & s->flags;
        tcu_update_all(s);
        return;
    case TCU_TFSR:
        tcu_account_all(s, now);
        s->tfr |= v & s->flags;
        tcu_update_all(s);
        return;
    case TCU_TFCR:
        tcu_account_all(s, now);
        s->tfr &= ~v;
        tcu_update_all(s);
        return;
    case TCU_TMR:
        tcu_account_all(s, now);
        s->tmr = v & s->flags;
        tcu_update_all(s);
        return;
    case TCU_TMSR:
        tcu_account_all(s, now);
        s->tmr |= v & s->flags;
        tcu_update_all(s);
        return;
    case TCU_TMCR:
        tcu_account_all(s, now);
        s->tmr &= ~v;
        tcu_update_all(s);
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
        s->ost_held = deposit64(tcu_ost_count(s), 0, 32, v);
        s->ost_base_ns = now;
        s->ost_seen = s->ost_held;
        tcu_ost_flag(s, true);
        return;
    case TCU_OST_CNTH:
        s->ost_held = deposit64(tcu_ost_count(s), 32, 32, v);
        s->ost_base_ns = now;
        s->ost_seen = s->ost_held;
        tcu_ost_flag(s, true);
        return;
    case TCU_OST_CSR:
        tcu_ost_latch(s, now);
        s->ost_csr = v;
        tcu_ost_flag(s, true);
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
    s->ost_held = 0;
    s->ost_seen = 0;
    s->ost_cnth_buf = 0;
    s->ost_data = 0;
    s->ost_csr = 0;

    s->ter = 0;
    s->tsr = 0;
    s->tfr = 0;
    s->tmr = s->flags;          /* all flags masked at reset */
    s->tstr = 0;

    s->wdt_tdr = 0;
    s->wdt_tcnt_base = 0;
    s->wdt_clear_pending = false;
    s->wdt_run_held = false;
    s->wdt_held = 0;
    s->wdt_tcsr = 0;
    s->wdt_tcer = 0;
    s->wdt_start_ns = 0;
    timer_del(s->wdt_timer);

    for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
        s->chn[n].tdfr = s->tdfr_reset;
        s->chn[n].tdhr = 0;
        s->chn[n].tcsr = 0;
        s->chn[n].count_base = 0;
        s->chn[n].base_ns = 0;
        s->chn[n].seen = 0;
        s->chn[n].stopping = false;
        s->chn[n].out = false;
        timer_del(s->chn[n].timer);
    }
    qemu_set_irq(s->irq, 0);
}

static void tcu_reset_exit(Object *obj, ResetType type)
{
    IngenicTcuState *s = INGENIC_TCU(obj);

    if (qemu_chr_fe_backend_connected(&s->trace)) {
        g_autofree char *js = g_strdup_printf(
            "{\"t\":%" PRId64 ",\"reset\":true,\"channels\":%u}\n",
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), s->channels);

        qemu_chr_fe_write_all(&s->trace, (uint8_t *)js, strlen(js));
        for (unsigned n = 0; n < s->channels; n++) {
            memset(&s->chn[n].traced, 0xff, sizeof(s->chn[n].traced));
            tcu_pwm_trace(s, n);
        }
    }
}

static void tcu_get_pwm(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    IngenicTcuChannel *c = opaque;
    IngenicPwmOut o;
    IngenicPwmKey key;
    uint64_t val;

    tcu_pwm_out(c->tcu, c->index, &o, &key);
    val = g_str_has_suffix(name, "-level") ? o.level :
          g_str_has_suffix(name, "-period-ns") ? o.period_ns : o.high_ns;
    visit_type_uint64(v, name, &val, errp);
}

static void tcu_init(Object *obj)
{
    IngenicTcuState *s = INGENIC_TCU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tcu_ops, s,
                          TYPE_INGENIC_TCU, TCU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(s), &s->wdt_reset, "wdt-reset", 1);

    s->wdt_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_fire, s);

    for (unsigned n = 0; n < INGENIC_TCU_NUM_CHANNELS; n++) {
        s->chn[n].tcu = s;
        s->chn[n].index = n;
        s->chn[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       tcu_chn_fire, &s->chn[n]);
        for (unsigned i = 0; i < 3; i++) {
            static const char *const what[] = {
                "level", "period-ns", "high-ns"
            };
            g_autofree char *name = g_strdup_printf("pwm%u-%s", n, what[i]);

            object_property_add(obj, name, "uint64", tcu_get_pwm, NULL, NULL,
                                &s->chn[n]);
        }
    }
}

static const VMStateDescription vmstate_tcu_channel = {
    .name = "ingenic-tcu/channel",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(tdfr, IngenicTcuChannel),
        VMSTATE_UINT16(tdhr, IngenicTcuChannel),
        VMSTATE_UINT16(tcsr, IngenicTcuChannel),
        VMSTATE_UINT16(count_base, IngenicTcuChannel),
        VMSTATE_INT64(base_ns, IngenicTcuChannel),
        VMSTATE_TIMER_PTR(timer, IngenicTcuChannel),
        VMSTATE_UINT64_V(seen, IngenicTcuChannel, 2),
        VMSTATE_BOOL_V(stopping, IngenicTcuChannel, 2),
        VMSTATE_BOOL_V(out, IngenicTcuChannel, 2),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_tcu = {
    .name = TYPE_INGENIC_TCU,
    .version_id = 4,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(wdt_tdr, IngenicTcuState),
        VMSTATE_UINT16(wdt_tcnt_base, IngenicTcuState),
        VMSTATE_UINT16(wdt_tcsr, IngenicTcuState),
        VMSTATE_UINT8(wdt_tcer, IngenicTcuState),
        VMSTATE_INT64(wdt_start_ns, IngenicTcuState),
        VMSTATE_TIMER_PTR(wdt_timer, IngenicTcuState),
        VMSTATE_INT64(ost_base_ns, IngenicTcuState),
        VMSTATE_UINT64_V(ost_held, IngenicTcuState, 2),
        VMSTATE_UINT64_V(ost_seen, IngenicTcuState, 3),
        VMSTATE_UINT32(ost_cnth_buf, IngenicTcuState),
        VMSTATE_UINT32(ost_data, IngenicTcuState),
        VMSTATE_UINT32(ost_csr, IngenicTcuState),
        VMSTATE_UINT32(ter, IngenicTcuState),
        VMSTATE_UINT32(tsr, IngenicTcuState),
        VMSTATE_UINT32(tfr, IngenicTcuState),
        VMSTATE_UINT32(tmr, IngenicTcuState),
        VMSTATE_UINT32(tstr, IngenicTcuState),
        VMSTATE_BOOL_V(wdt_clear_pending, IngenicTcuState, 4),
        VMSTATE_BOOL_V(wdt_run_held, IngenicTcuState, 4),
        VMSTATE_UINT16_V(wdt_held, IngenicTcuState, 4),
        VMSTATE_STRUCT_ARRAY(chn, IngenicTcuState, INGENIC_TCU_NUM_CHANNELS,
                             1, vmstate_tcu_channel, IngenicTcuChannel),
        VMSTATE_END_OF_LIST()
    },
};

/*
 * The flag bits the part has, all set in TMR out of reset: the channels' full
 * and half flags, OST's (bit 15) and, on the T32, T41 and A1, bit 24. The
 * default is the T21 manual's. The T10, T20, T23 and T31 have channels 0 to 3
 * only; the registers of 4 to 7 read 0 there. The channels' TDFR resets to 0
 * on the T32, T41 and A1, as each reads it at its bootrom; elsewhere it powers
 * on to whatever it held, and 0xffff stands in.
 */
static const Property tcu_properties[] = {
    DEFINE_PROP_UINT32("flags", IngenicTcuState, flags, 0x00ff80ff),
    DEFINE_PROP_UINT32("tdfr-reset", IngenicTcuState, tdfr_reset, 0xffff),
    DEFINE_PROP_UINT32("rtc-hz", IngenicTcuState, rtc_hz, 24000000 / 512),
    DEFINE_PROP_BOOL("wdt-reset-above", IngenicTcuState, wdt_reset_above,
                     false),
    DEFINE_PROP_BOOL("wdt-tdr-resumes", IngenicTcuState, wdt_tdr_resumes,
                     false),
    DEFINE_PROP_BOOL("wdt-clear-at-tick", IngenicTcuState, wdt_clear_at_tick,
                     false),
    DEFINE_PROP_BOOL("wdt-tcnt-ro", IngenicTcuState, wdt_tcnt_ro, false),
    DEFINE_PROP_BOOL("wdt-no-ext", IngenicTcuState, wdt_no_ext, false),
    DEFINE_PROP_BOOL("wdt-tcer-runs", IngenicTcuState, wdt_tcer_runs, false),
    DEFINE_PROP_CHR("trace", IngenicTcuState, trace),
};

static void tcu_realize(DeviceState *dev, Error **errp)
{
    IngenicTcuState *s = INGENIC_TCU(dev);

    s->channels = ctz32(~s->flags);
    if (!s->channels || s->channels > INGENIC_TCU_NUM_CHANNELS ||
        (s->flags & TCU_CHN_FLAGS) != (MAKE_64BIT_MASK(0, s->channels) *
                                       0x10001)) {
        error_setg(errp, "flags 0x%08" PRIx32 ": not 1 to %d channels",
                   s->flags, INGENIC_TCU_NUM_CHANNELS);
    }
}

static void tcu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = tcu_realize;
    dc->vmsd = &vmstate_tcu;
    device_class_set_props(dc, tcu_properties);
    rc->phases.hold = tcu_reset_hold;
    rc->phases.exit = tcu_reset_exit;
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
