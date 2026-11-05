/*
 * Ingenic T40 Clock/Power Manager (CPM) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T40_CPM_H
#define HW_MISC_INGENIC_T40_CPM_H

#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T40_CPM "ingenic-t40-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT40CpmState, INGENIC_T40_CPM)

#define INGENIC_T40_CPM_IOSIZE  0x100
#define INGENIC_T40_CPM_REGS    (INGENIC_T40_CPM_IOSIZE / 4)
#define INGENIC_T40_CPM_UARTS   6       /* the T41's; the T40 has four */

struct IngenicT40CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_T40_CPM_REGS];
    uint32_t latch[INGENIC_T40_CPM_REGS];  /* each as its last write with CE */
    uint32_t boot_regs[INGENIC_T40_CPM_REGS];
    bool boot_saved;
    qemu_irq otg_id_change;
    bool usb_device;            /* USBRDT bit 23 as last written */
    /* The ISP's clock, CLKGR's bit 23 clear, and its soft reset */
    qemu_irq isp_clock;
    qemu_irq isp_reset;
    qemu_irq helix_reset;       /* the T41's JPEG engine */
    /* The sensor's MCLK: a CIM clock not stopped (the T40 has three) */
    qemu_irq cim_clock;
    qemu_irq ivdc_clock;        /* the T41's: ISPACDR not stopped */
    qemu_irq uart_clock[INGENIC_T40_CPM_UARTS];
    /* The CPU clock, from CPCCR and the PLLs */
    Clock *cpu_clk;
    /* The play and record I2S clocks at 256 fs, from 0x70 and 0x84 */
    Clock *i2s_tx;
    Clock *i2s_rx;
    Clock *pwm_clk;             /* the T41's PWM block's */
    Clock *vo_clk;              /* the VO's pixel clock */

    bool has_epll;
    uint8_t pll_format;
    uint8_t cims;
    uint32_t srbc_held;         /* SRBC acks that read 1, no stop asked */
    /* Registers that keep only some bits of a write: offset, mask pairs */
    uint32_t num_wmasks;
    uint32_t *wmasks;
};

/* The CPU clock at the rate the boot registers give, before the CPM resets */
Clock *ingenic_t40_cpm_cpu_clock(IngenicT40CpmState *s);

#endif /* HW_MISC_INGENIC_T40_CPM_H */
