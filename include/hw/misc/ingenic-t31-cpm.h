/*
 * Ingenic T31 Clock/Power Manager (CPM) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T31_CPM_H
#define HW_MISC_INGENIC_T31_CPM_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_CPM "ingenic-t31-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31CpmState, INGENIC_T31_CPM)

#define INGENIC_T31_CPM_IOSIZE  0x100
#define INGENIC_T31_CPM_REGS    (INGENIC_T31_CPM_IOSIZE / 4)

struct IngenicT31CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_T31_CPM_REGS];
    /* OTG ID change notify pin (level out: 1=device/B, 0=host/A) */
    qemu_irq otg_id_change;
    /* The VPU's soft reset, SRBC bit 31, which the kernel pulses per job */
    qemu_irq vpu_reset;
    /* The CPU clock, from CPCCR and the PLLs */
    Clock *cpu_clk;

    /* Properties: the clock setup out of reset, and how the PLLs encode */
    uint32_t cpccr_reset;
    uint32_t apll_reset;
    uint32_t apll_frac_reset;
    uint32_t mpll_reset;
    uint32_t clkgr0_reset;
    uint32_t clkgr1_reset;
    uint8_t pll_format;
    bool srbc1;                 /* the T30's second SRBC */
};

/* The CPU clock at its reset rate, for connecting before the CPM resets */
Clock *ingenic_t31_cpm_cpu_clock(IngenicT31CpmState *s);

#endif /* HW_MISC_INGENIC_T31_CPM_H */
