/*
 * Ingenic XBurst Timer/Counter Unit (TCU) - 8 timer/PWM channels
 *
 * Models the TCU's general-purpose channels and global control. The
 * watchdog (channel 16) and the legacy OST live in the same 0x1000
 * page but are modelled separately; this device covers only the
 * channel/global register window at TCU base + 0x10 .. + 0xdf.
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

/* MMIO window: TCU base + 0x10 (TER) .. + 0xdf, just before the OST */
#define INGENIC_TCU_IO_BASE       0x10
#define INGENIC_TCU_IO_SIZE       0xd0

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

    uint32_t ter;           /* channel enable, bit N */
    uint32_t tsr;           /* channel stop, bit N */
    uint32_t tfr;           /* flags: bit N full-match, bit N+16 half-match */
    uint32_t tmr;           /* flag mask, same layout as tfr */

    IngenicTcuChannel chn[INGENIC_TCU_NUM_CHANNELS];
};

#endif /* HW_TIMER_INGENIC_TCU_H */
