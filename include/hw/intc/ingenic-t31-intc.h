/*
 * Ingenic T31 Interrupt Controller (INTC) emulation
 *
 * The INTC sits between SoC peripherals and MIPS hardware interrupt
 * IP2 (CAUSEF_IP2). Two banks of 32 sources each, with a fixed
 * 0x20-byte stride between bank register sets.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_INGENIC_T31_INTC_H
#define HW_INTC_INGENIC_T31_INTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_INTC "ingenic-t31-intc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31IntcState, INGENIC_T31_INTC)

#define INGENIC_T31_INTC_NR_BANKS    2
#define INGENIC_T31_INTC_NR_IRQS     (INGENIC_T31_INTC_NR_BANKS * 32)
#define INGENIC_T31_INTC_IOSIZE      0x200

struct IngenicT31IntcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq parent_irq;

    uint32_t isr[INGENIC_T31_INTC_NR_BANKS];
    uint32_t imr[INGENIC_T31_INTC_NR_BANKS];
};

#endif /* HW_INTC_INGENIC_T31_INTC_H */
