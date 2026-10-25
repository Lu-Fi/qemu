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
    /*
     * The soft resets the kernel pulses before each encode: the VPU's,
     * SRBC bit 31, and the T30's RADIX's, bit 28
     */
    qemu_irq vpu_reset;
    qemu_irq radix_reset;
    qemu_irq riscv_clock;       /* CLKGR0's riscv-gate bit clear */
    /* The ISP's clock, CLKGR0's isp-gate bit clear, and its soft reset */
    qemu_irq isp_clock;
    qemu_irq isp_reset;
    /* The CPU clock, from CPCCR and the PLLs */
    Clock *cpu_clk;
    /* The RISC-V core's, from RISCVHCDR and the PLLs */
    Clock *riscv_clk;

    /* Properties: the clock setup out of reset, and how the PLLs encode */
    uint32_t cpccr_reset;
    uint32_t apll_reset;
    uint32_t apll_frac_reset;
    uint32_t mpll_reset;
    uint32_t clkgr0_reset;
    uint32_t clkgr1_reset;
    uint8_t pll_format;
    uint8_t mpll_format;        /* the MPLL's, where it differs */
    bool srbc1;                 /* the T30's second SRBC */
    uint8_t riscv_gate;         /* the CLKGR0 bit that gates the core */
    uint8_t isp_gate;           /* the CLKGR0 bit that gates the ISP */
};

/* The CPU clock at its reset rate, for connecting before the CPM resets */
Clock *ingenic_t31_cpm_cpu_clock(IngenicT31CpmState *s);

#endif /* HW_MISC_INGENIC_T31_CPM_H */
