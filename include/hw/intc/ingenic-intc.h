/*
 * Ingenic T31 Interrupt Controller (INTC) emulation
 *
 * The INTC sits between SoC peripherals and MIPS hardware interrupt
 * IP2 (CAUSEF_IP2). Two banks of 32 sources each, with a fixed
 * 0x20-byte stride between bank register sets. XBurst2 SoCs give each
 * core its own register window at a 0x100 stride with an independent
 * mask; XBurst1 SoCs are single-core.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_INGENIC_INTC_H
#define HW_INTC_INGENIC_INTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_INTC "ingenic-intc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIntcState, INGENIC_INTC)

#define INGENIC_INTC_NR_BANKS    2
#define INGENIC_INTC_NR_IRQS     (INGENIC_INTC_NR_BANKS * 32)
#define INGENIC_INTC_MAX_CPUS    2
#define INGENIC_INTC_IOSIZE      0x200

struct IngenicIntcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    /*
     * Number of per-CPU register windows (0x100 stride). XBurst2 cores
     * each have their own window and mask; XBurst1 is single-core. Set
     * via the "num-cpus" property, default 1.
     */
    uint32_t num_cpus;

    /* One interrupt output per core (feeds that core's MIPS IP2 path). */
    qemu_irq parent_irq[INGENIC_INTC_MAX_CPUS];

    /* The raw source levels are shared; the mask is per core. */
    uint32_t isr[INGENIC_INTC_NR_BANKS];
    uint32_t imr[INGENIC_INTC_MAX_CPUS][INGENIC_INTC_NR_BANKS];
};

#endif /* HW_INTC_INGENIC_INTC_H */
