/*
 * Ingenic A1 Clock/Power Manager (CPM) emulation
 *
 * The CPM controls PLL configuration, clock gating, power domains,
 * and clock dividers for the A1 XBurst2 SoC. This model implements
 * the registers needed to boot SPL and U-Boot.
 *
 * The A1 has additional PLLs (EPLL, VPLL) at different offsets than
 * the T31 family, and its clock status register (CPCSR) lives at
 * offset 0xEC; 0xD4, the T31's CPCSR, is the A1's MAC1TXCDR.
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
#include "hw/misc/ingenic-a1-cpm.h"

/* Register offsets */
#define CPM_CPCCR           0x00
#define CPM_LCR             0x04
#define CPM_RSR             0x08
#define CPM_CPPCR           0x0C
#define CPM_CPAPCR          0x10    /* APLL control */
#define CPM_CPMPCR          0x14    /* MPLL control */
#define CPM_CPEPCR          0x18    /* EPLL control (A1-specific) */
#define CPM_CPVPCR          0x1C    /* VPLL control (A1-specific) */
#define CPM_CLKGR0          0x30
#define CPM_OPCR            0x34
#define CPM_CLKGR1          0x38
#define CPM_CPSPR           0x44    /* the scratch pad */
#define CPM_CPSPPR          0x48    /* its lock: 0x5a5a opens it */
#define CPM_USBPCR          0x3C
#define CPM_SOFTAPP         0x40
#define CPM_USB0PCR         0x50    /* USB0 to USB2: PCR, RDT, VBFIL, PCR1 */
#define CPM_USB0RDT         0x54
#define CPM_USB0VBFIL       0x58
#define CPM_USB0PCR1        0x5C
#define CPM_USB_STRIDE      0x10
#define CPM_INTR            0xE0
#define CPM_I2S0TCDR        0xA0
#define CPM_I2S0RCDR        0xA8
#define CPM_I2S1TCDR        0x100
#define CPM_I2S1RCDR        0x108
#define CPM_CPCSR_A1        0xEC    /* Clock status (A1) */
#define CPM_PWMCDR          0xFC    /* the PWM block clock */
#define CLKGR1_PWM          (1 << 15)
#define CLKGR0_UART0        8       /* UART n at 8 + n, as an A1N reads */
#define CPM_BT0CDR          0xF8    /* the BT output's pixel clock */
#define CLKGR1_BT0          (1 << 14)

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

/* PLL control register bits */
#define PLL_EN              (1 << 0)
#define PLL_LOCK            (1 << 2)
#define PLL_ON              (1 << 3)

static void ingenic_a1_cpm_update_cpu_clk(IngenicA1CpmState *s,
                                          bool propagate)
{
    uint64_t hz;

    hz = ingenic_cpu_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                        ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)], 0,
                                       INGENIC_PLL_MNOD),
                        ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                                       INGENIC_PLL_MNOD));
    if (propagate) {
        clock_update_hz(s->cpu_clk, hz);
    } else {
        clock_set_hz(s->cpu_clk, hz);
    }
}

Clock *ingenic_a1_cpm_cpu_clock(IngenicA1CpmState *s)
{
    ingenic_a1_cpm_update_cpu_clk(s, false);
    return s->cpu_clk;
}

/* A divider's source: SCLK_A, MPLL, VPLL or EPLL by its bits 31:30 */
static uint64_t ingenic_a1_cpm_src_hz(IngenicA1CpmState *s, uint32_t v)
{
    switch (extract32(v, 30, 2)) {
    case 0:
        return ingenic_sclka_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                                ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)],
                                               0, INGENIC_PLL_MNOD));
    case 1:
        return ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                              INGENIC_PLL_MNOD);
    case 2:
        return ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPVPCR)], 0,
                              INGENIC_PLL_MNOD);
    default:
        return ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPEPCR)], 0,
                              INGENIC_PLL_MNOD);
    }
}

/*
 * An I2S clock: its source times M (28:20) over N (19:0), a fractional
 * divider with no busy bit
 */
static uint64_t ingenic_a1_cpm_i2s_hz(IngenicA1CpmState *s, hwaddr cdr)
{
    uint32_t v = s->regs[REG_INDEX(cdr)];
    uint32_t n = extract32(v, 0, 20);

    return n ? ingenic_a1_cpm_src_hz(s, v) * extract32(v, 20, 9) / n : 0;
}

/*
 * The PWM block clock: its source over bits 7:0 plus one, none while
 * CLKGR1 bit 15 gates it
 */
static uint64_t ingenic_a1_cpm_pwm_hz(IngenicA1CpmState *s)
{
    uint32_t v = s->regs[REG_INDEX(CPM_PWMCDR)];

    if (s->regs[REG_INDEX(CPM_CLKGR1)] & CLKGR1_PWM) {
        return 0;
    }
    return ingenic_a1_cpm_src_hz(s, v) / (extract32(v, 0, 8) + 1);
}

/* The BT output's clock: its source over bits 7:0 plus one */
static uint64_t ingenic_a1_cpm_vo_hz(IngenicA1CpmState *s)
{
    uint32_t v = s->regs[REG_INDEX(CPM_BT0CDR)];

    if (s->regs[REG_INDEX(CPM_CLKGR1)] & CLKGR1_BT0) {
        return 0;
    }
    return ingenic_a1_cpm_src_hz(s, v) / (extract32(v, 0, 8) + 1);
}

static void ingenic_a1_cpm_update_i2s(IngenicA1CpmState *s, bool propagate)
{
    uint64_t tx = ingenic_a1_cpm_i2s_hz(s, CPM_I2S0TCDR);
    uint64_t rx = ingenic_a1_cpm_i2s_hz(s, CPM_I2S0RCDR);

    if (propagate) {
        clock_update_hz(s->i2s_tx, tx);
        clock_update_hz(s->i2s_rx, rx);
        clock_update_hz(s->pwm_clk, ingenic_a1_cpm_pwm_hz(s));
        clock_update_hz(s->vo_clk, ingenic_a1_cpm_vo_hz(s));
    } else {
        clock_set_hz(s->i2s_tx, tx);
        clock_set_hz(s->i2s_rx, rx);
        clock_set_hz(s->pwm_clk, ingenic_a1_cpm_pwm_hz(s));
        clock_set_hz(s->vo_clk, ingenic_a1_cpm_vo_hz(s));
    }
}

static uint64_t ingenic_a1_cpm_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    IngenicA1CpmState *s = INGENIC_A1_CPM(opaque);
    uint32_t idx = REG_INDEX(offset);

    if (offset >= INGENIC_A1_CPM_IOSIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds read at 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    switch (offset) {
    case CPM_CPCSR_A1:
        /* Never busy (bits 0-3); the rest as the A1N reads it */
        return 0x7C000000;

    default:
        return s->regs[idx];
    }
}

/* UART n's clock, CLKGR0 bit 8 + n clear */
static void ingenic_a1_cpm_uart_clocks(IngenicA1CpmState *s)
{
    for (int n = 0; n < INGENIC_A1_CPM_UARTS; n++) {
        qemu_set_irq(s->uart_clock[n],
                     !extract32(s->regs[REG_INDEX(CPM_CLKGR0)],
                                CLKGR0_UART0 + n, 1));
    }
}

static void ingenic_a1_cpm_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    IngenicA1CpmState *s = INGENIC_A1_CPM(opaque);
    uint32_t idx = REG_INDEX(offset);

    if (offset >= INGENIC_A1_CPM_IOSIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds write at 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }

    switch (offset) {
    case CPM_CPCCR:
        /* Bit 28 is part of the CPU clock select, not a busy bit */
        s->regs[idx] = (uint32_t)value;
        ingenic_a1_cpm_update_cpu_clk(s, true);
        ingenic_a1_cpm_update_i2s(s, true);
        break;

    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPEPCR:
    case CPM_CPVPCR:
        /*
         * PLL control registers. When software sets PLL_EN (bit 0),
         * immediately set PLL_ON (bit 3) to indicate the PLL has
         * locked - instant lock emulation.
         */
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON | PLL_LOCK;
        }
        ingenic_a1_cpm_update_cpu_clk(s, true);
        ingenic_a1_cpm_update_i2s(s, true);
        break;

    case CPM_I2S0TCDR:
    case CPM_I2S0RCDR:
        /* bit 28 is M's top bit here, not a busy bit */
        s->regs[idx] = (uint32_t)value;
        ingenic_a1_cpm_update_i2s(s, true);
        break;

    case CPM_CPCSR_A1:
        /* Clock status is read-only */
        break;

    case CPM_LCR:
        /*
         * Only the low-power mode, bits 1:0, keeps a write: no part from the
         * T10 to the A1 answers a power-down request here
         */
        s->regs[idx] = value & 0x3;
        break;

    case CPM_CLKGR0:
        /* Gates have no busy bit */
        s->regs[idx] = (uint32_t)value;
        ingenic_a1_cpm_uart_clocks(s);
        break;

    case CPM_CLKGR1:
        s->regs[idx] = value & 0xffff;
        ingenic_a1_cpm_update_i2s(s, true);
        break;

    case CPM_SOFTAPP:
        /*
         * A plain register on the A1N. Thingino's usb-role writes its
         * T-series USBRDT value here, so the model takes bit 23 as the OTG0
         * ID. Untested on silicon (the lab A1N has only host ports), where
         * the kernel's header puts the ID override in each PHY's RDT
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

    case CPM_USB0RDT:
    case CPM_USB0RDT + CPM_USB_STRIDE:
    case CPM_USB0RDT + 2 * CPM_USB_STRIDE:
        /* Bits 27 and 2:0 hold a write, as an A1N reads them back */
        s->regs[idx] = value & 0x08000007;
        break;

    case CPM_USB0PCR:
    case CPM_USB0PCR + CPM_USB_STRIDE:
    case CPM_USB0PCR + 2 * CPM_USB_STRIDE:
    case CPM_USB0VBFIL:
    case CPM_USB0VBFIL + CPM_USB_STRIDE:
    case CPM_USB0VBFIL + 2 * CPM_USB_STRIDE:
    case CPM_USB0PCR1:
    case CPM_USB0PCR1 + CPM_USB_STRIDE:
    case CPM_USB0PCR1 + 2 * CPM_USB_STRIDE:
        /* PHY controls, no divider: bit 28 is USB0PCR1's, set by the kernel */
        s->regs[idx] = value;
        break;

    case CPM_CPSPR:
        if (s->regs[REG_INDEX(CPM_CPSPPR)] == 0x5a5a) {
            s->regs[idx] = value;
        }
        break;

    case CPM_CPSPPR:
        s->regs[idx] = value & 0xffff;
        break;

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
        if (offset == CPM_PWMCDR || offset == CPM_BT0CDR) {
            ingenic_a1_cpm_update_i2s(s, true);
        }
        break;
    }
}

static const MemoryRegionOps ingenic_a1_cpm_ops = {
    .read = ingenic_a1_cpm_read,
    .write = ingenic_a1_cpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * As the A1N reads them after U-Boot; the three USB PHYs' controls and INTR
 * as it reads them at its bootrom, USB1's and USB2's, which the bootrom's
 * USB boot leaves alone. The gates as it reads them at its bootrom too: the
 * bootrom only ever clears them (SFC, MSC, OTG, UART1, HASH), so the SFC and
 * MSC gates its USB boot opens may be shut at power-on
 */
static void ingenic_a1_cpm_power_on(IngenicA1CpmState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    for (int i = 0; i < 3; i++) {
        hwaddr usb = i * CPM_USB_STRIDE;

        s->regs[REG_INDEX(CPM_USB0PCR + usb)] = 0x409919b8;
        s->regs[REG_INDEX(CPM_USB0RDT + usb)] = 0x00000004;
        s->regs[REG_INDEX(CPM_USB0VBFIL + usb)] = 0x00ff0080;
        s->regs[REG_INDEX(CPM_USB0PCR1 + usb)] = 0x8a800090;
    }
    s->regs[REG_INDEX(CPM_INTR)] = 0x00000015;
    s->regs[REG_INDEX(CPM_CLKGR0)] = 0xf4b00700;
    s->regs[REG_INDEX(CPM_CLKGR1)] = 0x0000ffff;
    s->regs[REG_INDEX(0x98)] = 0x00108000;             /* MSC0CDR */
    s->regs[REG_INDEX(0x9c)] = 0x00108000;             /* MSC1CDR */
    s->regs[REG_INDEX(0xb0)] = 0x00000800;             /* VDEVCDR */
    s->regs[REG_INDEX(CPM_RSR)] = 0x00000001;
    s->regs[REG_INDEX(CPM_CPPCR)] = 0x00000020;
    s->regs[REG_INDEX(CPM_OPCR)] = 0x00001500;
    s->regs[REG_INDEX(CPM_CPSPPR)] = 0x0000a5a5;
    s->regs[REG_INDEX(CPM_I2S0TCDR)] = 0x00100002;
    s->regs[REG_INDEX(CPM_I2S0RCDR)] = 0x00100002;
    s->regs[REG_INDEX(CPM_I2S1TCDR)] = 0x00100002;
    s->regs[REG_INDEX(CPM_I2S1RCDR)] = 0x00100002;
}

static void ingenic_a1_cpm_reset_hold(Object *obj, ResetType type)
{
    IngenicA1CpmState *s = INGENIC_A1_CPM(obj);
    static const hwaddr plls[] = { CPM_CPAPCR, CPM_CPMPCR, CPM_CPEPCR,
                                   CPM_CPVPCR };
    uint32_t pad = s->regs[REG_INDEX(CPM_CPSPR)];

    if (s->boot_saved) {
        memcpy(s->regs, s->boot_regs, sizeof(s->regs));
    } else {
        ingenic_a1_cpm_power_on(s);
    }
    /* A reset locks the scratch pad but keeps what it holds */
    s->regs[REG_INDEX(CPM_CPSPPR)] = 0x0000a5a5;
    s->regs[REG_INDEX(CPM_CPSPR)] = pad;
    /* A running PLL reads on and locked */
    for (int i = 0; i < ARRAY_SIZE(plls); i++) {
        if (s->regs[REG_INDEX(plls[i])] & PLL_EN) {
            s->regs[REG_INDEX(plls[i])] |= PLL_ON | PLL_LOCK;
        }
    }
    ingenic_a1_cpm_update_cpu_clk(s, true);
    ingenic_a1_cpm_update_i2s(s, true);
    ingenic_a1_cpm_uart_clocks(s);
}

static void ingenic_a1_cpm_init(Object *obj)
{
    IngenicA1CpmState *s = INGENIC_A1_CPM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_a1_cpm_ops, s,
                          TYPE_INGENIC_A1_CPM, INGENIC_A1_CPM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    /* So that the SoC's boot snapshot starts from them */
    ingenic_a1_cpm_power_on(s);
    qdev_init_gpio_out_named(DEVICE(s), &s->otg_id_change,
                             "otg-id-change", 1);
    qdev_init_gpio_out_named(DEVICE(s), s->uart_clock, "uart-clock",
                             INGENIC_A1_CPM_UARTS);
    s->cpu_clk = qdev_init_clock_out(DEVICE(s), "cpu-clk");
    s->i2s_tx = qdev_init_clock_out(DEVICE(s), "i2s-tx");
    s->i2s_rx = qdev_init_clock_out(DEVICE(s), "i2s-rx");
    s->pwm_clk = qdev_init_clock_out(DEVICE(s), "pwm-clk");
    s->vo_clk = qdev_init_clock_out(DEVICE(s), "vo-clk");
}

static int ingenic_a1_cpm_post_load(void *opaque, int version_id)
{
    ingenic_a1_cpm_update_cpu_clk(opaque, false);
    ingenic_a1_cpm_update_i2s(opaque, false);
    return 0;
}

static const VMStateDescription vmstate_ingenic_a1_cpm = {
    .name = "ingenic-a1-cpm",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_a1_cpm_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicA1CpmState,
                             INGENIC_A1_CPM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_a1_cpm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_a1_cpm;
    rc->phases.hold = ingenic_a1_cpm_reset_hold;
}

static const TypeInfo ingenic_a1_cpm_type_info = {
    .name = TYPE_INGENIC_A1_CPM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1CpmState),
    .instance_init = ingenic_a1_cpm_init,
    .class_init = ingenic_a1_cpm_class_init,
};

static void ingenic_a1_cpm_register_types(void)
{
    type_register_static(&ingenic_a1_cpm_type_info);
}

type_init(ingenic_a1_cpm_register_types)
