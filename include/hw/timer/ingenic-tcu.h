/*
 * Ingenic XBurst Timer/Counter Unit (TCU)
 *
 * Models the one register page shared by the TCU and the OS Timer: 8
 * general-purpose timer/PWM channels, the TCU global control registers,
 * the embedded 64-bit OST free-running counter, and the watchdog
 * (TCU channel 16) at the low 16 bytes. Used by every XBurst1 and
 * XBurst2 SoC.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_INGENIC_TCU_H
#define HW_TIMER_INGENIC_TCU_H

#include "chardev/char-fe.h"
#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-pwm-out.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_TCU "ingenic-tcu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicTcuState, INGENIC_TCU)

#define INGENIC_TCU_NUM_CHANNELS  8

typedef struct IngenicTcuChannel {
    uint16_t tdfr;          /* full-match (period) value */
    uint16_t tdhr;          /* half-match (duty) value */
    uint16_t tcsr;          /* clock source / prescaler / control */
    uint16_t count_base;    /* counter value sampled at base_ns */
    int64_t  base_ns;
    uint64_t seen;          /* ticks since base_ns whose matches TFR holds */
    bool     stopping;      /* disabled, counting on to its full match */
    bool     out;           /* the PWM output, as of seen */
    QEMUTimer *timer;
    IngenicTcuState *tcu;
    unsigned index;
    IngenicPwmKey traced;
} IngenicTcuChannel;

struct IngenicTcuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Watchdog (TCU channel 16) - resets the SoC when TCNT reaches TDR */
    QEMUTimer *wdt_timer;
    qemu_irq wdt_reset;     /* pulsed as the watchdog resets the SoC */
    uint16_t wdt_tdr;
    uint16_t wdt_tcnt_base;
    uint16_t wdt_tcsr;
    uint8_t  wdt_tcer;
    int64_t  wdt_start_ns;

    /* Embedded OST - 64-bit counter, ost_held since ost_base_ns */
    int64_t  ost_base_ns;
    uint64_t ost_held;
    uint64_t ost_seen;      /* the count OSTFLAG was last checked to */
    uint32_t ost_cnth_buf;
    uint32_t ost_data;
    uint32_t ost_csr;

    /* TCU global registers */
    uint32_t ter;           /* channel enable, bit N */
    uint32_t tsr;           /* channel stop, bit N */
    uint32_t tfr;           /* flags: bit N full-match, bit N+16 half-match */
    uint32_t tmr;           /* flag mask, same layout as tfr */
    uint32_t tstr;          /* OST strobe/status */
    bool wdt_clear_pending; /* a TCSR clear waiting for WDTS to let go */
    bool wdt_run_held;      /* a TDR write let the held count run on */
    uint16_t wdt_held;      /* the count TCNT reads while WDTS is set */

    uint32_t flags;         /* the TFR and TMR bits the part has: TMR's reset */
    uint32_t tdfr_reset;    /* the channels' TDFR out of reset */
    uint32_t rtc_hz;        /* RTC_EN's clock, the channels' and watchdog's */
    bool wdt_reset_above;   /* reset with TDR at or below the count */
    bool wdt_clear_at_tick; /* a TCSR clear lands at the next count */
    bool wdt_tcnt_ro;       /* TCNT takes no write */
    bool wdt_no_ext;        /* TCSR has no EXTAL input (EXT_EN reads 0) */
    bool wdt_tcer_runs;     /* the count runs with TCER clear */
    bool wdt_tdr_resumes;   /* a TDR write lets a WDTS-held count run on */
    unsigned channels;      /* those of its channels, from flags */

    CharFrontend trace;
    IngenicTcuChannel chn[INGENIC_TCU_NUM_CHANNELS];
};

#endif /* HW_TIMER_INGENIC_TCU_H */
