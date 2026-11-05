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
#define INGENIC_T31_CPM_UARTS   3
#define INGENIC_T31_CPM_NO_GATE 0xff     /* a gate property for none */

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
    qemu_irq ncu_reset;         /* T30: SRBC1 bit 2 */
    qemu_irq riscv_clock;       /* CLKGR0's riscv-gate bit clear */
    /* The ISP's clock, CLKGR0's isp-gate bit clear, and its soft reset */
    qemu_irq isp_clock;
    qemu_irq isp_reset;
    qemu_irq cim_clock;         /* the sensor's MCLK: CIMCDR not stopped */
    qemu_irq uart_clock[INGENIC_T31_CPM_UARTS];
    /* The CPU clock, from CPCCR and the PLLs */
    Clock *cpu_clk;
    /* The RISC-V core's, from RISCVHCDR and the PLLs */
    Clock *riscv_clk;
    Clock *riscv_ost_clk;       /* T31: L2CLK through the RISC-V gate */
    /* The play and record I2S clocks at 256 fs, from 0x70 and 0x84 */
    Clock *i2s_tx;
    Clock *i2s_rx;
    Clock *pwm_clk;             /* the T32's and T33's PWM block's */
    Clock *tnpu_clk;            /* the T32's and T33's TNPU's */

    /* Properties: the clock setup out of reset, and how the PLLs encode */
    uint32_t cpccr_reset;
    uint32_t apll_reset;
    uint32_t apll_frac_reset;
    uint32_t mpll_reset;
    uint32_t vpll_reset;
    uint32_t ddrcdr_reset;
    uint32_t clkgr0_reset;
    uint32_t clkgr1_reset;
    uint8_t pll_format;
    uint8_t mpll_format;        /* the MPLL's, where it differs */
    bool srbc1;                 /* the T30's second SRBC */
    bool epll;                  /* the T30's EPLL at 0x58 */
    uint8_t riscv_gate;         /* the CLKGR0 bit that gates the core */
    uint8_t isp_gate;           /* the CLKGR0 bit that gates the ISP */
    uint8_t uart_gate;          /* UART0's CLKGR0 bit; the rest follow */
    uint32_t usbrdt_mask;       /* the USBRDT bits that hold a write */
    uint32_t srbc_held;         /* SRBC acks that read 1, no stop asked */
    bool ssicdr_ce28;           /* SSICDR's change enable at 28, busy at 27 */
    uint8_t i2s_duty;           /* INGENIC_T31_I2S_DUTY_*: see below */
    uint8_t i2s_cdr;            /* INGENIC_T31_I2S_CDR_*: see below */
    uint32_t i2s_mux;           /* INGENIC_T31_I2S_SRC_* per mux value */
    bool usb_device;            /* USBRDT bit 23 as last written */
    /* The family's own values out of reset: offset, value pairs */
    uint32_t num_resets;
    uint32_t *resets;
    /* Registers that keep only some bits of a write: offset, mask pairs */
    uint32_t num_wmasks;
    uint32_t *wmasks;
};

/*
 * Duty registers that read half an I2S divider's N, as a T21N and a T32NQ
 * read them after any divider write: the T21's at 0x70 for its divider at
 * 0x60, the T32's at 0x78 and 0x88 for 0x70 and 0x84
 */
enum {
    INGENIC_T31_I2S_DUTY_NONE,
    INGENIC_T31_I2S_DUTY_T21,
    INGENIC_T31_I2S_DUTY_T32,
};

/*
 * Where the I2S clocks are: the T31 family plays from 0x70 and records from
 * 0x84; up to the T30 one divider at 0x60 clocks both, as a T20X, T21N and
 * T30X play and record at its rate, the T10's and T20's with M at 21:13 and
 * N at 12:0 rather than 28:20 and 19:0
 */
enum {
    INGENIC_T31_I2S_CDR_SPLIT,
    INGENIC_T31_I2S_CDR_ONE,
    INGENIC_T31_I2S_CDR_ONE_M13,
};

/* An I2S clock's sources, as its mux bits pick them per family */
enum {
    INGENIC_T31_I2S_SRC_SCLKA,
    INGENIC_T31_I2S_SRC_MPLL,
    INGENIC_T31_I2S_SRC_VPLL,
    INGENIC_T31_I2S_SRC_EPLL,
    INGENIC_T31_I2S_SRC_NONE,
};

/* The CPU clock at its reset rate, for connecting before the CPM resets */
Clock *ingenic_t31_cpm_cpu_clock(IngenicT31CpmState *s);

#endif /* HW_MISC_INGENIC_T31_CPM_H */
