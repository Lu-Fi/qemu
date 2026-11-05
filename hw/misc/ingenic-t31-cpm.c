/*
 * Ingenic T31 Clock/Power Manager (CPM) emulation
 *
 * The CPM controls PLL configuration, clock gating, power domains,
 * and clock dividers for the T31 SoC. This model implements the
 * registers needed to boot SPL and U-Boot.
 *
 * A divider's value and STOP take effect only on a write with its CE (bit
 * 29, SSICDR's bit 28 up to the T31), as a T31X's SFC clock does; its
 * source (31:30) at once. Every write reads back.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/misc/ingenic-cpm-clk.h"
#include "hw/misc/ingenic-t31-cpm.h"

/* Register offsets */
#define CPM_CPCCR       0x00
#define CPM_LCR         0x04
#define CPM_RSR         0x08
#define CPM_CPPCR       0x0C
#define CPM_CPAPCR      0x10
#define CPM_CPMPCR      0x14
#define CPM_CPAPFR      0x18    /* T10/T20 APLL fraction */
#define CPM_CLKGR0      0x20
#define CPM_OPCR        0x24
#define CPM_CLKGR1      0x28
#define CPM_DDRCDR      0x2C
#define CPM_CPSPR       0x34    /* the scratch pad */
#define CPM_CPSPPR      0x38    /* its lock: 0x5a5a opens it */
#define CPM_USBPCR      0x3C
#define CPM_USBRDT      0x40
#define CPM_USBVBFIL    0x44
#define CPM_USBPCR1     0x48
#define CPM_MACCDR      0x54
#define CPM_CPEPCR      0x58    /* T30: the EPLL; the T32's SFC0CDR */
#define CPM_PWMCDR      0x5C    /* T32, T33: the PWM block clock */
#define CPM_TNPUCDR     0x98    /* T32, T33: the TNPU's clock */
#define CPM_I2SCDR_T21  0x60    /* T21: the I2S clock, its duty at 0x70 */
#define CPM_VPUCDR      0x30    /* T23: the VPU clock, the IVDC's */
#define CPM_MSC0CDR     0x68
#define CPM_I2SCDR      0x70    /* the play (speaker) I2S clock */
#define CPM_CIMCDR      0x7C
#define CPM_I2SCDR1     0x84    /* the record (mic) I2S clock */
#define CPM_SSICDR      0x74
#define CPM_I2SDCDR     0x78    /* T32: the play clock's duty */
#define CPM_I2SDCDR1    0x88    /* T32: the record clock's duty */
#define CPM_RISCVHCDR   0x9C    /* T23, T32, T33: the RISC-V core's clock */
#define CPM_SRBC1       0xAC    /* T30: the T32's SFC1CDR */
#define CPM_INTR        0xB0
#define CPM_SRBC        0xC4
#define CPM_CPCSR       0xD4
#define CPM_CPVPCR      0xE0
#define CPM_GMACPHYC    0xE8

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

/* PLL control register bits */
#define PLL_EN          (1 << 0)
#define PLL_LOCK        (1 << 2)
#define PLL_ON          (1 << 3)

/*
 * SRBC stop requests: each acknowledge is the bit below its request, and
 * with no bus transfer in flight a module stops as soon as it is asked.
 * VPU 30/29, HELIX 27/26 (T32; the T30's RADIX), ISP 21/20, CPU 15/14,
 * AHB1 10/9, AHB2 8/7 and DDR 6/5; SRBC1 is the T30's LDC 7/6, MSCALER 4/3
 * and NCU 1/0, its resets bits 8, 5 and 2.
 */
#define SRBC_ACKS       0x241042a0
#define SRBC1_ACKS      0x00000049
/* The ISP's soft reset (TIZIANO_SR), which tx-isp pulses as it opens it */
#define SRBC_ISP_SR     (1 << 22)

/*
 * MACPHYC's top byte is read-only: bits 30:29 the speed the GMAC runs at,
 * its MAC_CONFIG.FES (10M at reset), as a T21N, T31X and T41NQ read it
 */
#define MACPHYC_RO      0xff000000

/* SSICDR's change enable and busy sit a bit lower than the other dividers' */
#define SSICDR_BUSY     (1u << 27)

/* An I2S duty register: D is N/2 unless DEN is set, as the T21 manual gives */
#define I2SD_DEN        (1u << 30)
#define I2SD_MASK       0xc00fffff

/* A CGU divider's STOP bit, under BUSY (28) and CE (29) */
#define CGU_STOP        (1 << 27)
#define CGU_CE          (1 << 29)
#define CGU_MUX         (3u << 30)
#define CLKGR1_PWM      (1 << 9)    /* T32, T33 */
#define CLKGR1_TNPU     (1 << 11)   /* T32, T33 */
#define I2S_CE          (1 << 29)

/*
 * A divider as its clock runs: the divider and STOP bits as its last write
 * with CE set left them, its source (31:30) as written last. Every write
 * reads back, as on a T31X, whose SFC clock (SSICDR) kept its rate through
 * a divider written without CE.
 */
static uint32_t ingenic_t31_cpm_cdr(IngenicT31CpmState *s, hwaddr cdr)
{
    uint32_t idx = REG_INDEX(cdr);
    uint32_t mux = s->regs[idx] & CGU_MUX;

    return mux | (s->latch[idx] & ~CGU_MUX);
}

static void ingenic_t31_cpm_latch(IngenicT31CpmState *s, hwaddr cdr,
                                  uint32_t ce)
{
    uint32_t idx = REG_INDEX(cdr);

    if (s->regs[idx] & ce) {
        s->latch[idx] = s->regs[idx];
    }
}

/* The CIM's clock, the sensor's MCLK, and the T23's IVDC's, the VPU's */
static void ingenic_t31_cpm_update_stops(IngenicT31CpmState *s)
{
    qemu_set_irq(s->cim_clock,
                 !(ingenic_t31_cpm_cdr(s, CPM_CIMCDR) & CGU_STOP));
    qemu_set_irq(s->ivdc_clock,
                 !(ingenic_t31_cpm_cdr(s, CPM_VPUCDR) & CGU_STOP));
}

static uint32_t ingenic_t31_cpm_srbc(uint32_t v, uint32_t acks)
{
    return (v & ~acks) | (v >> 1 & acks);
}

static uint64_t ingenic_t31_cpm_cpu_hz(IngenicT31CpmState *s, uint32_t cpccr,
                                       uint32_t apll, uint32_t frac,
                                       uint32_t mpll)
{
    return ingenic_cpu_hz(cpccr, ingenic_pll_hz(apll, frac, s->pll_format),
                          ingenic_pll_hz(mpll, 0, s->mpll_format));
}

/* RISCVHCDR: bits 31:30 pick the APLL (0) or MPLL (1), 3:0 divide by +1 */
static uint64_t ingenic_t31_cpm_riscv_hz(IngenicT31CpmState *s)
{
    uint32_t hcdr = s->regs[REG_INDEX(CPM_RISCVHCDR)];
    uint64_t pll = extract32(hcdr, 30, 2) == 1 ?
        ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0, s->mpll_format) :
        ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)],
                       s->regs[REG_INDEX(CPM_CPAPFR)], s->pll_format);

    return pll / (extract32(hcdr, 0, 4) + 1);
}

/* L2CLK through the RISC-V gate, which a T31's RISC-V block's OST counts */
static uint64_t ingenic_t31_cpm_riscv_ost_hz(IngenicT31CpmState *s)
{
    if (extract32(s->regs[REG_INDEX(CPM_CLKGR0)], s->riscv_gate, 1)) {
        return 0;
    }
    return ingenic_l2_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                         ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)],
                                        s->regs[REG_INDEX(CPM_CPAPFR)],
                                        s->pll_format),
                         ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                                        s->mpll_format));
}

/* The source an I2S mux value picks, as the vendor kernels' cgu_i2s routes */
static uint64_t ingenic_t31_cpm_i2s_src(IngenicT31CpmState *s, unsigned mux)
{
    switch (extract32(s->i2s_mux, mux * 4, 4)) {
    case INGENIC_T31_I2S_SRC_SCLKA:
        return ingenic_sclka_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                                ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)],
                                               s->regs[REG_INDEX(CPM_CPAPFR)],
                                               s->pll_format));
    case INGENIC_T31_I2S_SRC_MPLL:
        return ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                              s->mpll_format);
    case INGENIC_T31_I2S_SRC_VPLL:
        return ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPVPCR)], 0,
                              s->pll_format);
    case INGENIC_T31_I2S_SRC_EPLL:
        return ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPEPCR)], 0,
                              s->pll_format);
    default:
        return 0;
    }
}

/*
 * An I2S clock, 256 fs: its source by bits 31:30 of the play register (the
 * record register's read 0, as measured on a T31X), times M over N while CE
 * is set, as the vendor kernels' cgu_i2s
 */
static uint64_t ingenic_t31_cpm_i2s_hz(IngenicT31CpmState *s, bool rx)
{
    bool split = s->i2s_cdr == INGENIC_T31_I2S_CDR_SPLIT;
    bool m13 = s->i2s_cdr == INGENIC_T31_I2S_CDR_ONE_M13;
    hwaddr play = split ? CPM_I2SCDR : CPM_I2SCDR_T21;
    uint32_t v = s->regs[REG_INDEX(split && rx ? CPM_I2SCDR1 : play)];
    uint32_t m = m13 ? extract32(v, 13, 9) : extract32(v, 20, 9);
    uint32_t n = m13 ? extract32(v, 0, 13) : extract32(v, 0, 20);
    uint64_t src;

    src = ingenic_t31_cpm_i2s_src(s,
                                  extract32(s->regs[REG_INDEX(play)], 30, 2));
    return (v & I2S_CE) && n ? src * m / n : 0;
}

/*
 * A T32 or T33 divider clock (the PWM block's, the TNPU's): SCLK_A, MPLL or
 * VPLL by bits 31:30 over bits 7:0 plus one, none while its CLKGR1 bit
 * gates it
 */
static uint64_t ingenic_t31_cpm_div_hz(IngenicT31CpmState *s, hwaddr cdr,
                                       uint32_t gate)
{
    uint32_t v = ingenic_t31_cpm_cdr(s, cdr);
    uint64_t src;

    if ((s->regs[REG_INDEX(CPM_CLKGR1)] & gate) || (v & CGU_STOP)) {
        return 0;
    }
    switch (extract32(v, 30, 2)) {
    case 0:
        src = ingenic_sclka_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                               ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)],
                                              s->regs[REG_INDEX(CPM_CPAPFR)],
                                              s->pll_format));
        break;
    case 1:
        src = ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                             s->mpll_format);
        break;
    case 2:
        src = ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPVPCR)], 0,
                             s->pll_format);
        break;
    default:
        return 0;
    }
    return src / (extract32(v, 0, 8) + 1);
}

static void ingenic_t31_cpm_update_cpu_clk(IngenicT31CpmState *s,
                                           bool propagate)
{
    uint64_t hz = ingenic_t31_cpm_cpu_hz(s, s->regs[REG_INDEX(CPM_CPCCR)],
                                         s->regs[REG_INDEX(CPM_CPAPCR)],
                                         s->regs[REG_INDEX(CPM_CPAPFR)],
                                         s->regs[REG_INDEX(CPM_CPMPCR)]);

    if (propagate) {
        clock_update_hz(s->cpu_clk, hz);
        clock_update_hz(s->riscv_clk, ingenic_t31_cpm_riscv_hz(s));
        clock_update_hz(s->riscv_ost_clk, ingenic_t31_cpm_riscv_ost_hz(s));
        clock_update_hz(s->i2s_tx, ingenic_t31_cpm_i2s_hz(s, false));
        clock_update_hz(s->i2s_rx, ingenic_t31_cpm_i2s_hz(s, true));
        clock_update_hz(s->pwm_clk,
                        ingenic_t31_cpm_div_hz(s, CPM_PWMCDR, CLKGR1_PWM));
        clock_update_hz(s->tnpu_clk,
                        ingenic_t31_cpm_div_hz(s, CPM_TNPUCDR, CLKGR1_TNPU));
    } else {
        clock_set_hz(s->cpu_clk, hz);
        clock_set_hz(s->riscv_clk, ingenic_t31_cpm_riscv_hz(s));
        clock_set_hz(s->riscv_ost_clk, ingenic_t31_cpm_riscv_ost_hz(s));
        clock_set_hz(s->i2s_tx, ingenic_t31_cpm_i2s_hz(s, false));
        clock_set_hz(s->i2s_rx, ingenic_t31_cpm_i2s_hz(s, true));
        clock_set_hz(s->pwm_clk,
                     ingenic_t31_cpm_div_hz(s, CPM_PWMCDR, CLKGR1_PWM));
        clock_set_hz(s->tnpu_clk,
                     ingenic_t31_cpm_div_hz(s, CPM_TNPUCDR, CLKGR1_TNPU));
    }
}

Clock *ingenic_t31_cpm_cpu_clock(IngenicT31CpmState *s)
{
    clock_set_hz(s->cpu_clk, ingenic_t31_cpm_cpu_hz(s, s->cpccr_reset,
                                                    s->apll_reset,
                                                    s->apll_frac_reset,
                                                    s->mpll_reset));
    return s->cpu_clk;
}

/* The bits of a write to @offset that the family's register keeps */
static uint32_t ingenic_t31_cpm_wmask(IngenicT31CpmState *s, hwaddr offset)
{
    for (uint32_t i = 0; i + 1 < s->num_wmasks; i += 2) {
        if (s->wmasks[i] == offset) {
            return s->wmasks[i + 1];
        }
    }
    return UINT32_MAX;
}

/* The divider an I2S duty register follows, or 0 if offset is none */
static hwaddr ingenic_t31_cpm_duty_of(IngenicT31CpmState *s, hwaddr offset)
{
    switch (s->i2s_duty) {
    case INGENIC_T31_I2S_DUTY_T21:
        return offset == CPM_I2SCDR ? CPM_I2SCDR_T21 : 0;
    case INGENIC_T31_I2S_DUTY_T32:
        return offset == CPM_I2SDCDR ? CPM_I2SCDR :
               offset == CPM_I2SDCDR1 ? CPM_I2SCDR1 : 0;
    default:
        return 0;
    }
}

static uint64_t ingenic_t31_cpm_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(opaque);
    uint32_t idx = REG_INDEX(offset);

    if (offset >= INGENIC_T31_CPM_IOSIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds read at 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    if (ingenic_t31_cpm_duty_of(s, offset)) {
        uint32_t v = s->regs[idx];
        uint32_t n = extract32(s->regs[REG_INDEX(
                                   ingenic_t31_cpm_duty_of(s, offset))], 0, 20);

        return v & I2SD_DEN ? v : (v & ~0xfffff) | n / 2;
    }

    switch (offset) {
    case CPM_CPCSR:
        /*
         * Clock process status. T10-T31 SPLs don't poll this, but
         * T32/T33 SPL polls for (val & 0xf0000007) == 0xf0000000
         * after writing CPCCR. Bits 28-31 = PLL-mux-stable flags,
         * bits 0-2 = divider-busy. Return "all stable, not busy".
         */
        return 0xF8000000;     /* as T10 to T32 silicon reads it */

    case CPM_SRBC:
        return ingenic_t31_cpm_srbc(s->regs[idx], SRBC_ACKS) | s->srbc_held;

    case CPM_SRBC1:
        return ingenic_t31_cpm_srbc(s->regs[idx], s->srbc1 ? SRBC1_ACKS : 0);

    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPVPCR:
        return s->regs[idx];

    default:
        return s->regs[idx];
    }
}

/* UART n's clock, CLKGR0 bit uart-gate + n clear, where a part has them */
static void ingenic_t31_cpm_uart_clocks(IngenicT31CpmState *s, uint32_t gr)
{
    if (s->uart_gate == INGENIC_T31_CPM_NO_GATE) {
        return;
    }
    for (int n = 0; n < INGENIC_T31_CPM_UARTS; n++) {
        qemu_set_irq(s->uart_clock[n], !extract32(gr, s->uart_gate + n, 1));
    }
}

static void ingenic_t31_cpm_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(opaque);
    uint32_t idx = REG_INDEX(offset);

    if (offset >= INGENIC_T31_CPM_IOSIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds write at 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }

    if (ingenic_t31_cpm_duty_of(s, offset)) {
        s->regs[idx] = value & I2SD_MASK;
        return;
    }

    switch (offset) {
    case CPM_CPCCR:
        /* Bit 28 is part of the CPU clock select, not a busy bit */
        s->regs[idx] = (uint32_t)value;
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPVPCR:
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON | PLL_LOCK;
        }
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_CPCSR:
    case CPM_INTR:
        break;

    case CPM_LCR:
        /*
         * Only the low-power mode, bits 1:0, keeps a write: no part from the
         * T10 to the A1 answers a power-down request here
         */
        s->regs[idx] = value & 0x3;
        break;

    case CPM_CPSPR:
        if (s->regs[REG_INDEX(CPM_CPSPPR)] == 0x5a5a) {
            s->regs[idx] = value;
        }
        break;

    case CPM_CPSPPR:
        s->regs[idx] = value & 0xffff;
        break;

    case CPM_CLKGR0:
    case CPM_CLKGR1:
        /* Gates have no busy bit: bit 28 of CLKGR0 gates the DES */
        s->regs[idx] = (uint32_t)value;
        if (offset == CPM_CLKGR0) {
            qemu_set_irq(s->riscv_clock,
                         !extract32(value, s->riscv_gate, 1));
            qemu_set_irq(s->isp_clock, !extract32(value, s->isp_gate, 1));
            ingenic_t31_cpm_uart_clocks(s, value);
        }
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_RISCVHCDR:
        s->regs[idx] = value & ~(1u << 28);     /* never busy */
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_I2SCDR:
    case CPM_I2SCDR1:
        /* No busy bit: bit 28 is M's. The record register's mux reads 0 */
        s->regs[idx] = offset == CPM_I2SCDR1 ? value & 0x3fffffff : value;
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_USBPCR:
    case CPM_USBVBFIL:
    case CPM_USBPCR1:
        /* PHY controls, no divider: bit 28 is USBPCR1's, set by the kernel */
        s->regs[idx] = value;
        break;

    case CPM_SSICDR:
        /*
         * Up to the T31 bit 28 is the change enable, as a T31X keeps it, and
         * busy reads 0; the T32's has the other dividers' busy at 28
         */
        s->regs[idx] = value & ~(s->ssicdr_ce28 ? SSICDR_BUSY : 1u << 28);
        ingenic_t31_cpm_latch(s, offset, s->ssicdr_ce28 ? 1u << 28 : CGU_CE);
        break;

    case CPM_USBRDT:
        /*
         * Thingino's /usr/sbin/usb-role uses this register to flip USB
         * role on T20-class SoCs (where bit 23 selects host vs device).
         * On T31 the role is set via DWC2's GOTGCTL.CONID_B directly,
         * and the USBRDT writes that follow only configure clocking/
         * PHY (host writes 0x0b000096; device writes 0x0b000096 then
         * 0x0b000FFF -- bit 23 is *clear* in both cases). So only
         * forward an OTG ID-change IRQ if bit 23 actually transitioned
         * relative to the previous value, otherwise the device-mode
         * sequence (GOTGCTL=device, then two USBRDT writes) ends up
         * snapping back to host mode because bit 23 is 0 each time.
         * From the T21 on only bits 31, 27 and 2:0 hold a write, as a
         * T31X reads them back, so the role is kept apart.
         */
        {
            bool device = value & (1u << 23);

            s->regs[idx] = value & s->usbrdt_mask;
            if (s->otg_id_change && device != s->usb_device) {
                qemu_set_irq(s->otg_id_change, device);
            }
            s->usb_device = device;
        }
        break;

    case CPM_GMACPHYC:
        s->regs[idx] = (s->regs[idx] & MACPHYC_RO) | (value & ~MACPHYC_RO);
        break;

    case CPM_SRBC:
        s->regs[idx] = value & ~(SRBC_ACKS | s->srbc_held);
        qemu_set_irq(s->vpu_reset, extract32(value, 31, 1));
        qemu_set_irq(s->radix_reset, extract32(value, 28, 1));
        qemu_set_irq(s->isp_reset, !!(value & SRBC_ISP_SR));
        break;

    case CPM_CPEPCR:
        if (s->epll) {
            s->regs[idx] = (uint32_t)value;
            if (value & PLL_EN) {
                s->regs[idx] |= PLL_ON | PLL_LOCK;
            }
            break;
        }
        /* fall through */
    case CPM_SRBC1:
        if (offset == CPM_SRBC1 && s->srbc1) {
            s->regs[idx] = value & ~SRBC1_ACKS;
            qemu_set_irq(s->ncu_reset, extract32(value, 2, 1));
            break;
        }
        /* fall through */
    default:
        s->regs[idx] = value & ingenic_t31_cpm_wmask(s, offset);
        if (offset == CPM_I2SCDR_T21 &&
            s->i2s_cdr == INGENIC_T31_I2S_CDR_ONE) {
            ingenic_t31_cpm_update_cpu_clk(s, true);
            break;      /* bit 28 is M's, not a busy bit */
        }
        /*
         * Clock divider registers (DDRCDR, MACCDR, MSC0CDR, etc.)
         * have a 'busy' bit that the SPL polls until clear. On real
         * silicon the clock switch completes in a few cycles and the
         * busy bit self-clears. Clear it immediately so SPL doesn't
         * hang. Busy bits are typically at bit 28 in Ingenic CDR regs.
         */
        if (value & (1u << 28)) {
            s->regs[idx] &= ~(1u << 28);
        }
        ingenic_t31_cpm_latch(s, offset, CGU_CE);
        if (offset == CPM_CPAPFR && s->pll_format == INGENIC_PLL_MNOD_FRAC) {
            ingenic_t31_cpm_update_cpu_clk(s, true);
        }
        if (offset == CPM_CIMCDR || offset == CPM_VPUCDR) {
            ingenic_t31_cpm_update_stops(s);
        }
        if (offset == CPM_PWMCDR || offset == CPM_TNPUCDR) {
            ingenic_t31_cpm_update_cpu_clk(s, true);
        }
        if (offset == CPM_I2SCDR_T21 &&
            s->i2s_cdr == INGENIC_T31_I2S_CDR_ONE_M13) {
            ingenic_t31_cpm_update_cpu_clk(s, true);
        }
        break;
    }
}

static const MemoryRegionOps ingenic_t31_cpm_ops = {
    .read = ingenic_t31_cpm_read,
    .write = ingenic_t31_cpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_cpm_reset_hold(Object *obj, ResetType type)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(obj);
    static const hwaddr plls[] = { CPM_CPAPCR, CPM_CPMPCR, CPM_CPVPCR };
    uint32_t pad = s->regs[REG_INDEX(CPM_CPSPR)];

    memset(s->regs, 0, sizeof(s->regs));
    s->usb_device = false;

    /*
     * Defaults emulate "post-U-Boot" CPM state so a uImage booted
     * directly (-kernel uImage) sees fully-configured PLLs and
     * dividers. The Linux clock framework reads these registers
     * during init_cgu_clk / init_ext_pll; missing/zero fields cause
     * NULL parents (parent=STOP for SELECTOR_A index 0) or
     * divide-by-zero (m=n=od=0 in pll_get_rate).
     *
     * PLL encoding from kernel pll_get_rate():
     *   rate = (parent/4000) * m / n / od1 / od0 * 4000
     * The APLL, MPLL and VPLL come from the properties (the SoC
     * variant's and family's). Bit 0 = enable, bit 3 = on.
     */
    s->regs[REG_INDEX(CPM_CPCCR)]  = s->cpccr_reset;
    s->regs[REG_INDEX(CPM_RSR)]    = 0x00000001;
    /* As every part from T10 to T32 reads them, and the T21 manual gives */
    s->regs[REG_INDEX(CPM_CPPCR)]  = 0x00000020;
    s->regs[REG_INDEX(CPM_OPCR)]   = 0x00001500;
    s->regs[REG_INDEX(CPM_CPSPPR)] = 0x0000a5a5;
    s->regs[REG_INDEX(CPM_CLKGR0)] = s->clkgr0_reset;
    s->regs[REG_INDEX(CPM_CLKGR1)] = s->clkgr1_reset;
    /*
     * A reset locks the scratch pad but keeps what it holds, as a watchdog
     * reset does on silicon; each part powers on with a word of its own
     * there. T10 to T31 kernels write 0x101 into it at boot, and the T10,
     * T20, T21 and T30 libimps read its low byte as the grade (0x10 the Lite)
     */
    s->regs[REG_INDEX(CPM_CPSPR)]  = pad;
    s->regs[REG_INDEX(CPM_CPAPCR)] = s->apll_reset;
    s->regs[REG_INDEX(CPM_CPMPCR)] = s->mpll_reset;
    if (s->pll_format == INGENIC_PLL_MNOD_FRAC) {
        s->regs[REG_INDEX(CPM_CPAPFR)] = s->apll_frac_reset;
    }
    s->regs[REG_INDEX(CPM_CPVPCR)] = s->vpll_reset;
    /* A running PLL reads on and locked, as every part's silicon does */
    for (int i = 0; i < ARRAY_SIZE(plls); i++) {
        if (s->regs[REG_INDEX(plls[i])] & PLL_EN) {
            s->regs[REG_INDEX(plls[i])] |= PLL_ON | PLL_LOCK;
        }
    }
    for (int i = 0; i + 1 < s->num_resets; i += 2) {
        s->regs[REG_INDEX(s->resets[i])] = s->resets[i + 1];
    }

    /*
     * DDRCDR bits 30-31 pick the parent via kernel SELECTOR_A
     * {STOP, SCLKA, MPLL, INVALID}: the default, 10, is the MPLL a -kernel
     * boot needs; the SoC gives -bios boots the power-on value.
     */
    s->regs[REG_INDEX(CPM_DDRCDR)] = s->ddrcdr_reset;

    memcpy(s->latch, s->regs, sizeof(s->latch));
    ingenic_t31_cpm_update_cpu_clk(s, true);
}

static void ingenic_t31_cpm_reset_exit(Object *obj, ResetType type)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(obj);

    qemu_set_irq(s->isp_clock, !extract32(s->clkgr0_reset, s->isp_gate, 1));
    ingenic_t31_cpm_uart_clocks(s, s->clkgr0_reset);
    ingenic_t31_cpm_update_stops(s);
}

static void ingenic_t31_cpm_mac_speed(void *opaque, int n, int level)
{
    IngenicT31CpmState *s = opaque;
    uint32_t *r = &s->regs[REG_INDEX(CPM_GMACPHYC)];

    *r = deposit32(*r, 29, 1, level);
}

static void ingenic_t31_cpm_init(Object *obj)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_cpm_ops, s,
                          TYPE_INGENIC_T31_CPM, INGENIC_T31_CPM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out_named(DEVICE(s), &s->otg_id_change,
                             "otg-id-change", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->vpu_reset, "vpu-reset", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->radix_reset, "radix-reset", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->ncu_reset, "ncu-reset", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->riscv_clock, "riscv-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->isp_clock, "isp-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->isp_reset, "isp-reset", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->cim_clock, "cim-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->ivdc_clock, "ivdc-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), s->uart_clock, "uart-clock",
                             INGENIC_T31_CPM_UARTS);
    qdev_init_gpio_in_named(DEVICE(s), ingenic_t31_cpm_mac_speed,
                            "mac-speed", 1);
    s->cpu_clk = qdev_init_clock_out(DEVICE(s), "cpu-clk");
    s->riscv_clk = qdev_init_clock_out(DEVICE(s), "riscv-clk");
    s->riscv_ost_clk = qdev_init_clock_out(DEVICE(s), "riscv-ost-clk");
    s->i2s_tx = qdev_init_clock_out(DEVICE(s), "i2s-tx");
    s->i2s_rx = qdev_init_clock_out(DEVICE(s), "i2s-rx");
    s->pwm_clk = qdev_init_clock_out(DEVICE(s), "pwm-clk");
    s->tnpu_clk = qdev_init_clock_out(DEVICE(s), "tnpu-clk");
}

/* Defaults: CPU from APLL, APLL 600 MHz (m=25), MPLL 1200 MHz (m=50) */
static const Property ingenic_t31_cpm_properties[] = {
    DEFINE_PROP_UINT32("cpccr", IngenicT31CpmState, cpccr_reset, 0x95800000),
    DEFINE_PROP_UINT32("apll", IngenicT31CpmState, apll_reset,
                       (25 << 20) | (1 << 14) | (1 << 11) | (1 << 8) |
                       PLL_ON | PLL_EN),
    DEFINE_PROP_UINT32("apll-frac", IngenicT31CpmState, apll_frac_reset, 0),
    DEFINE_PROP_UINT32("ddrcdr", IngenicT31CpmState, ddrcdr_reset, 0x80000000),
    /* 480 MHz in MNOD: M 20, N, OD1 and OD0 1 */
    DEFINE_PROP_UINT32("vpll", IngenicT31CpmState, vpll_reset, 0x0140490d),
    DEFINE_PROP_UINT32("clkgr0", IngenicT31CpmState, clkgr0_reset, 0),
    DEFINE_PROP_UINT32("clkgr1", IngenicT31CpmState, clkgr1_reset, 0),
    DEFINE_PROP_UINT32("mpll", IngenicT31CpmState, mpll_reset,
                       (50 << 20) | (1 << 14) | (1 << 11) | (1 << 8) |
                       PLL_ON | PLL_EN),
    DEFINE_PROP_UINT8("pll-format", IngenicT31CpmState, pll_format,
                      INGENIC_PLL_MNOD),
    DEFINE_PROP_UINT8("mpll-format", IngenicT31CpmState, mpll_format,
                      INGENIC_PLL_MNOD),
    DEFINE_PROP_BOOL("srbc1", IngenicT31CpmState, srbc1, false),
    DEFINE_PROP_BOOL("epll", IngenicT31CpmState, epll, false),
    DEFINE_PROP_UINT8("riscv-gate", IngenicT31CpmState, riscv_gate, 26),
    DEFINE_PROP_UINT8("isp-gate", IngenicT31CpmState, isp_gate, 23),
    DEFINE_PROP_UINT8("uart-gate", IngenicT31CpmState, uart_gate,
                      INGENIC_T31_CPM_NO_GATE),
    DEFINE_PROP_UINT32("usbrdt-mask", IngenicT31CpmState, usbrdt_mask,
                       UINT32_MAX),
    DEFINE_PROP_UINT32("srbc-held", IngenicT31CpmState, srbc_held, 0),
    DEFINE_PROP_BOOL("ssicdr-ce28", IngenicT31CpmState, ssicdr_ce28, true),
    DEFINE_PROP_UINT8("i2s-duty", IngenicT31CpmState, i2s_duty,
                      INGENIC_T31_I2S_DUTY_NONE),
    DEFINE_PROP_UINT8("i2s-cdr", IngenicT31CpmState, i2s_cdr,
                      INGENIC_T31_I2S_CDR_SPLIT),
    /* A nibble per mux value, low first: SCLK_A, MPLL, VPLL, none (T31) */
    DEFINE_PROP_UINT32("i2s-mux", IngenicT31CpmState, i2s_mux, 0x4210),
    DEFINE_PROP_ARRAY("resets", IngenicT31CpmState, num_resets, resets,
                      qdev_prop_uint32, uint32_t),
    DEFINE_PROP_ARRAY("wmasks", IngenicT31CpmState, num_wmasks, wmasks,
                      qdev_prop_uint32, uint32_t),
};

static int ingenic_t31_cpm_post_load(void *opaque, int version_id)
{
    IngenicT31CpmState *s = opaque;

    if (version_id < 2) {
        memcpy(s->latch, s->regs, sizeof(s->latch));
    }
    ingenic_t31_cpm_update_cpu_clk(s, false);
    return 0;
}

static const VMStateDescription vmstate_ingenic_t31_cpm = {
    .name = "ingenic-t31-cpm",
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = ingenic_t31_cpm_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT31CpmState,
                             INGENIC_T31_CPM_REGS),
        VMSTATE_UINT32_ARRAY_V(latch, IngenicT31CpmState,
                               INGENIC_T31_CPM_REGS, 2),
        VMSTATE_BOOL_V(usb_device, IngenicT31CpmState, 3),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t31_cpm_realize(DeviceState *dev, Error **errp)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(dev);

    for (int i = 0; i < s->num_resets; i += 2) {
        if (i + 1 == s->num_resets || s->resets[i] & 3 ||
            s->resets[i] >= INGENIC_T31_CPM_IOSIZE) {
            error_setg(errp, "resets: bad offset, value pair at %d", i);
            return;
        }
    }
}

static void ingenic_t31_cpm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_t31_cpm_realize;
    dc->vmsd = &vmstate_ingenic_t31_cpm;
    device_class_set_props(dc, ingenic_t31_cpm_properties);
    rc->phases.hold = ingenic_t31_cpm_reset_hold;
    rc->phases.exit = ingenic_t31_cpm_reset_exit;
}

static const TypeInfo ingenic_t31_cpm_type_info = {
    .name = TYPE_INGENIC_T31_CPM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31CpmState),
    .instance_init = ingenic_t31_cpm_init,
    .class_init = ingenic_t31_cpm_class_init,
};

static void ingenic_t31_cpm_register_types(void)
{
    type_register_static(&ingenic_t31_cpm_type_info);
}

type_init(ingenic_t31_cpm_register_types)
