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
#define INGENIC_A1_CPM_UARTS   3

struct IngenicA1CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_A1_CPM_REGS];
    uint32_t latch[INGENIC_A1_CPM_REGS];   /* each as its last write with CE */
    /* Reset values, when the SoC has set them */
    uint32_t boot_regs[INGENIC_A1_CPM_REGS];
    bool boot_saved;
    /* OTG ID change notify pin (level out: 1=device/B, 0=host/A) */
    qemu_irq otg_id_change;
    qemu_irq uart_clock[INGENIC_A1_CPM_UARTS];
    qemu_irq helix_reset;       /* SRBC0 bit 22: the JPEG engine's */
    /* The CPU clock, from CPCCR and the PLLs */
    Clock *cpu_clk;
    Clock *i2s_tx;              /* I2S0's, 256 fs: the AIC plays at it */
    Clock *i2s_rx;              /* ... and records at this one */
    Clock *i2s1_tx;             /* I2S1's: AIC1 sends HDMI audio at it */
    Clock *pwm_clk;             /* the PWM block's */
    Clock *vo_clk;              /* the BT output's pixel clock */
};

/* The CPU clock at the rate the boot registers give, before the CPM resets */
Clock *ingenic_a1_cpm_cpu_clock(IngenicA1CpmState *s);

#endif /* HW_MISC_INGENIC_A1_CPM_H */
