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
#include "hw/core/clock.h"

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
    Clock *clk;                 /* EXCLK unless wired */
    uint32_t size;
    void (*notify)(void *);     /* T2's rate or count restarted */
    void *notify_opaque;

    uint32_t tccr;
    uint32_t ter;
    uint32_t tfr;
    uint32_t tmr;
    uint32_t t1dfr;

    IngenicSysOstChannel ch[2]; /* T1 (32-bit, periodic), T2 (64-bit) */
    QEMUTimer *t1_timer;
    uint32_t t2_high_buf;
};

/* T2's count now; *hz its rate, 0 while it is stopped */
uint64_t ingenic_sysost_t2(IngenicSysOstState *s, uint64_t *hz);

/* Call @fn when T2's rate changes or its count restarts */
void ingenic_sysost_set_notify(IngenicSysOstState *s, void (*fn)(void *),
                               void *opaque);

#endif /* HW_TIMER_INGENIC_SYSOST_H */
