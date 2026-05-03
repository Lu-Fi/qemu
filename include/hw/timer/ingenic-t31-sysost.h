/*
 * Ingenic T31 System OS Timer (standalone, at 0x12000000)
 *
 * The system OST is the BSP 3.10.14 kernel's tick + clocksource.
 * Two channels:
 *   T1 (CH 0) - 32-bit periodic, drives the kernel tick via MIPS IP4
 *   T2 (CH 1) - 64-bit free-running, used as the clocksource
 *
 * Distinct from the TCU-internal OST at 0x100020E0 used by U-Boot.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_INGENIC_T31_SYSOST_H
#define HW_TIMER_INGENIC_T31_SYSOST_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_INGENIC_T31_SYSOST "ingenic-t31-sysost"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31SysOstState, INGENIC_T31_SYSOST)

#define INGENIC_T31_SYSOST_IOSIZE   0x100

struct IngenicT31SysOstState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t tccr;
    uint32_t ter;
    uint32_t tfr;
    uint32_t tmr;
    uint32_t t1dfr;

    /* T1 periodic (32-bit). Software-visible counter increments at
     * the configured rate; QEMUTimer schedules period rollover. */
    QEMUTimer *t1_timer;
    int64_t t1_base_ns;
    uint32_t t1_base_count;

    /* T2 free-running (64-bit). Always-on virtual counter latched
     * via the kernel's read pattern (T2CNTL read latches T2HBUF). */
    int64_t t2_base_ns;
    uint32_t t2_high_buf;
};

#endif /* HW_TIMER_INGENIC_T31_SYSOST_H */
