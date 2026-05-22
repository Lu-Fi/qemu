/*
 * Ingenic XBurst Timer/Counter Unit (TCU)
 *
 * Models the one register page shared by the TCU and the OS Timer: 8
 * general-purpose timer/PWM channels, the TCU global control registers,
 * and the embedded 64-bit OST free-running counter. Used by every
 * XBurst1 and XBurst2 SoC. The watchdog (channel 16) is modelled
 * separately and overlaps the low 16 bytes of this page.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_INGENIC_TCU_H
#define HW_TIMER_INGENIC_TCU_H

#include "hw/core/sysbus.h"
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
    QEMUTimer *timer;
    IngenicTcuState *tcu;
    unsigned index;
} IngenicTcuChannel;

struct IngenicTcuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Embedded OST - 64-bit free-running counter */
    int64_t  ost_base_ns;
    uint32_t ost_cnth_buf;
    uint32_t ost_data;
    uint32_t ost_csr;

    /* TCU global registers */
    uint32_t ter;           /* channel enable, bit N */
    uint32_t tsr;           /* channel stop, bit N */
    uint32_t tfr;           /* flags: bit N full-match, bit N+16 half-match */
    uint32_t tmr;           /* flag mask, same layout as tfr */
    uint32_t tstr;          /* OST strobe/status */

    IngenicTcuChannel chn[INGENIC_TCU_NUM_CHANNELS];
};

#endif /* HW_TIMER_INGENIC_TCU_H */
