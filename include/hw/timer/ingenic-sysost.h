/*
 * Ingenic System OS Timer (the standalone OST at 0x12000000)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_TIMER_INGENIC_SYSOST_H
#define HW_TIMER_INGENIC_SYSOST_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "qemu/timer.h"

#define TYPE_INGENIC_SYSOST "ingenic-sysost"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSysOstState, INGENIC_SYSOST)

/* The registers' span; a larger "size" repeats them */
#define INGENIC_SYSOST_IOSIZE   0x100

typedef struct IngenicSysOstChannel {
    int64_t base_ns;            /* when the count last stood at base */
    uint64_t base;
} IngenicSysOstChannel;

struct IngenicSysOstState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t size;

    uint32_t tccr;
    uint32_t ter;
    uint32_t tfr;
    uint32_t tmr;
    uint32_t t1dfr;

    IngenicSysOstChannel ch[2]; /* T1 (32-bit, periodic), T2 (64-bit) */
    QEMUTimer *t1_timer;
    uint32_t t2_high_buf;
};

#endif /* HW_TIMER_INGENIC_SYSOST_H */
