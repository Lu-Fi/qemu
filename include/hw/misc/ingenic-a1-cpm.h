/*
 * Ingenic A1 Clock/Power Manager (CPM) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_CPM_H
#define HW_MISC_INGENIC_A1_CPM_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_CPM "ingenic-a1-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1CpmState, INGENIC_A1_CPM)

#define INGENIC_A1_CPM_IOSIZE  0x200
#define INGENIC_A1_CPM_REGS    (INGENIC_A1_CPM_IOSIZE / 4)

struct IngenicA1CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_A1_CPM_REGS];
    /* Reset values, when the SoC has set them */
    uint32_t boot_regs[INGENIC_A1_CPM_REGS];
    bool boot_saved;
    /* OTG ID change notify pin (level out: 1=device/B, 0=host/A) */
    qemu_irq otg_id_change;
    /* The CPU clock, from CPCCR and the PLLs */
    Clock *cpu_clk;
    Clock *i2s_tx;              /* I2S0's, 256 fs: the AIC plays at it */
    Clock *i2s_rx;              /* ... and records at this one */
};

/* The CPU clock at the rate the boot registers give, before the CPM resets */
Clock *ingenic_a1_cpm_cpu_clock(IngenicA1CpmState *s);

#endif /* HW_MISC_INGENIC_A1_CPM_H */
