/*
 * Ingenic T31 Clock/Power Manager (CPM) emulation
 *
 * The CPM controls PLL configuration, clock gating, power domains,
 * and clock dividers for the T31 SoC. This model implements the
 * registers needed to boot SPL and U-Boot.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
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
#define CPM_USBPCR      0x3C
#define CPM_USBRDT      0x40
#define CPM_USBVBFIL    0x44
#define CPM_USBPCR1     0x48
#define CPM_MACCDR      0x54
#define CPM_MSC0CDR     0x68
#define CPM_I2SCDR      0x70    /* the play (speaker) I2S clock */
#define CPM_CIMCDR      0x7C
#define CPM_I2SCDR1     0x84    /* the record (mic) I2S clock */
#define CPM_SSICDR      0x74
#define CPM_RISCVHCDR   0x9C    /* T23, T32, T33: the RISC-V core's clock */
#define CPM_SRBC1       0xAC    /* T30: the T32's SFC1CDR */
#define CPM_SRBC        0xC4
#define CPM_CPCSR       0xD4
#define CPM_CPVPCR      0xE0
#define CPM_GMACPHYC    0xE8

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

/* PLL control register bits */
#define PLL_EN          (1 << 0)
#define PLL_ON          (1 << 3)

/*
 * SRBC stop requests: each acknowledge is the bit below its request, and
 * with no bus transfer in flight a module stops as soon as it is asked.
 * VPU 30/29, HELIX 27/26 (T32; the T30's RADIX), ISP 21/20, CPU 15/14,
 * AHB1 10/9, AHB2 8/7 and DDR 6/5; SRBC1 is the T30's LDC 7/6, MSCALER 4/3
 * and NCU 1/0.
 */
#define SRBC_ACKS       0x241042a0
#define SRBC1_ACKS      0x00000049
/* The ISP's soft reset (TIZIANO_SR), which tx-isp pulses as it opens it */
#define SRBC_ISP_SR     (1 << 22)

/* A CGU divider's STOP bit, under BUSY (28) and CE (29) */
#define CGU_STOP        (1 << 27)
#define I2S_CE          (1 << 29)

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

/*
 * An I2S clock: SCLK_A, MPLL or VPLL by bits 31:30 of the play register
 * (the record register's read 0, as measured on a T31X), times M (28:20)
 * over N (19:0) while CE is set, as the vendor kernels' cgu_i2s_spk/mic
 */
static uint64_t ingenic_t31_cpm_i2s_hz(IngenicT31CpmState *s, hwaddr cdr)
{
    uint32_t v = s->regs[REG_INDEX(cdr)];
    uint32_t n = extract32(v, 0, 20);
    uint64_t src;

    switch (extract32(s->regs[REG_INDEX(CPM_I2SCDR)], 30, 2)) {
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
    return (v & I2S_CE) && n ? src * extract32(v, 20, 9) / n : 0;
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
        clock_update_hz(s->i2s_tx, ingenic_t31_cpm_i2s_hz(s, CPM_I2SCDR));
        clock_update_hz(s->i2s_rx, ingenic_t31_cpm_i2s_hz(s, CPM_I2SCDR1));
    } else {
        clock_set_hz(s->cpu_clk, hz);
        clock_set_hz(s->riscv_clk, ingenic_t31_cpm_riscv_hz(s));
        clock_set_hz(s->i2s_tx, ingenic_t31_cpm_i2s_hz(s, CPM_I2SCDR));
        clock_set_hz(s->i2s_rx, ingenic_t31_cpm_i2s_hz(s, CPM_I2SCDR1));
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

    switch (offset) {
    case CPM_CPCSR:
        /*
         * Clock process status. T10-T31 SPLs don't poll this, but
         * T32/T33 SPL polls for (val & 0xf0000007) == 0xf0000000
         * after writing CPCCR. Bits 28-31 = PLL-mux-stable flags,
         * bits 0-2 = divider-busy. Return "all stable, not busy".
         */
        return 0xF0000000;

    case CPM_SRBC:
        return ingenic_t31_cpm_srbc(s->regs[idx], SRBC_ACKS);

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

    switch (offset) {
    case CPM_CPCCR:
        /* Bit 28 is part of the CPU clock select, not a busy bit */
        s->regs[idx] = (uint32_t)value;
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPVPCR:
    case 0x58: /* CPM_CPEPCR - T30's EPLL */
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON;
        }
        ingenic_t31_cpm_update_cpu_clk(s, true);
        break;

    case CPM_CPCSR:
        break;

    case CPM_CLKGR0:
    case CPM_CLKGR1:
        /* Gates have no busy bit: bit 28 of CLKGR0 gates the DES */
        s->regs[idx] = (uint32_t)value;
        if (offset == CPM_CLKGR0) {
            qemu_set_irq(s->riscv_clock,
                         !extract32(value, s->riscv_gate, 1));
            qemu_set_irq(s->isp_clock, !extract32(value, s->isp_gate, 1));
        }
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
         */
        {
            uint32_t prev = s->regs[idx];
            int prev_mode = !!(prev & (1u << 23));
            int new_mode  = !!(value & (1u << 23));
            s->regs[idx] = (uint32_t)value;
            if (s->otg_id_change && new_mode != prev_mode) {
                qemu_set_irq(s->otg_id_change, new_mode);
            }
        }
        break;

    case CPM_SRBC:
        s->regs[idx] = value & ~SRBC_ACKS;
        qemu_set_irq(s->vpu_reset, extract32(value, 31, 1));
        qemu_set_irq(s->radix_reset, extract32(value, 28, 1));
        qemu_set_irq(s->isp_reset, !!(value & SRBC_ISP_SR));
        break;

    case CPM_SRBC1:
        if (s->srbc1) {
            s->regs[idx] = value & ~SRBC1_ACKS;
            break;
        }
        /* fall through */
    default:
        s->regs[idx] = (uint32_t)value;
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
        if (offset == CPM_CPAPFR && s->pll_format == INGENIC_PLL_MNOD_FRAC) {
            ingenic_t31_cpm_update_cpu_clk(s, true);
        }
        if (offset == CPM_CIMCDR) {
            qemu_set_irq(s->cim_clock, !(value & CGU_STOP));
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

    memset(s->regs, 0, sizeof(s->regs));

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
     * The APLL and MPLL come from the properties (the SoC variant's);
     * VPLL=480M (m=20) with n=od0=od1=1. Bit 0 = enable, bit 3 = on.
     */
    s->regs[REG_INDEX(CPM_CPCCR)]  = s->cpccr_reset;
    s->regs[REG_INDEX(CPM_RSR)]    = 0x00000001;
    s->regs[REG_INDEX(CPM_CLKGR0)] = s->clkgr0_reset;
    s->regs[REG_INDEX(CPM_CLKGR1)] = s->clkgr1_reset;
    s->regs[REG_INDEX(CPM_CPPCR)]  = 0x00000000;
    /*
     * Register 0x34 is CPPSR (CPU Process/Package Status).
     * libimp's get_cpu_id reads cppsr byte 0 to pick a detection
     * path: 0 -> use EFUSE subsoctype1 lookup (our variant table),
     * 1 -> hardcode T31-N, 3 -> hardcode T31-L, etc.
     * Default to 0 so the EFUSE path is taken and -global
     * ingenic-t31.soc-variant= controls which name libimp reports.
     */
    s->regs[REG_INDEX(0x34)]       = 0x00000000;
    s->regs[REG_INDEX(CPM_CPAPCR)] = s->apll_reset;
    s->regs[REG_INDEX(CPM_CPMPCR)] = s->mpll_reset;
    if (s->pll_format == INGENIC_PLL_MNOD_FRAC) {
        s->regs[REG_INDEX(CPM_CPAPFR)] = s->apll_frac_reset;
    }
    s->regs[REG_INDEX(CPM_CPVPCR)] = (20 << 20) | (1 << 14) |
                                     (1 << 11) | (1 << 8) |
                                     PLL_ON | PLL_EN;

    /*
     * DDRCDR bits 30-31 pick the parent via kernel SELECTOR_A
     * {STOP, SCLKA, MPLL, INVALID}. Setting bits = 10 -> MPLL.
     */
    s->regs[REG_INDEX(CPM_DDRCDR)] = 0x80000000;

    ingenic_t31_cpm_update_cpu_clk(s, true);
}

static void ingenic_t31_cpm_reset_exit(Object *obj, ResetType type)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(obj);

    qemu_set_irq(s->isp_clock, !extract32(s->clkgr0_reset, s->isp_gate, 1));
    qemu_set_irq(s->cim_clock,
                 !(s->regs[REG_INDEX(CPM_CIMCDR)] & CGU_STOP));
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
    qdev_init_gpio_out_named(DEVICE(s), &s->riscv_clock, "riscv-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->isp_clock, "isp-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->isp_reset, "isp-reset", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->cim_clock, "cim-clock", 1);
    s->cpu_clk = qdev_init_clock_out(DEVICE(s), "cpu-clk");
    s->riscv_clk = qdev_init_clock_out(DEVICE(s), "riscv-clk");
    s->i2s_tx = qdev_init_clock_out(DEVICE(s), "i2s-tx");
    s->i2s_rx = qdev_init_clock_out(DEVICE(s), "i2s-rx");
}

/* Defaults: CPU from APLL, APLL 600 MHz (m=25), MPLL 1200 MHz (m=50) */
static const Property ingenic_t31_cpm_properties[] = {
    DEFINE_PROP_UINT32("cpccr", IngenicT31CpmState, cpccr_reset, 0x95800000),
    DEFINE_PROP_UINT32("apll", IngenicT31CpmState, apll_reset,
                       (25 << 20) | (1 << 14) | (1 << 11) | (1 << 8) |
                       PLL_ON | PLL_EN),
    DEFINE_PROP_UINT32("apll-frac", IngenicT31CpmState, apll_frac_reset, 0),
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
    DEFINE_PROP_UINT8("riscv-gate", IngenicT31CpmState, riscv_gate, 26),
    DEFINE_PROP_UINT8("isp-gate", IngenicT31CpmState, isp_gate, 23),
};

static int ingenic_t31_cpm_post_load(void *opaque, int version_id)
{
    ingenic_t31_cpm_update_cpu_clk(opaque, false);
    return 0;
}

static const VMStateDescription vmstate_ingenic_t31_cpm = {
    .name = "ingenic-t31-cpm",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_t31_cpm_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT31CpmState,
                             INGENIC_T31_CPM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t31_cpm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

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
