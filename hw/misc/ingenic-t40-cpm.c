/*
 * Ingenic T40/T41 Clock/Power Manager (CPM) emulation
 *
 * T40/T41 CPM has a different register layout from the A1:
 *   APLL at 0x10, MPLL at 0x14 (same as A1)
 *   EPLL at 0x58 (T40 only; T41 lacks EPLL)
 *   VPLL at 0xE0
 *   CLKGR at 0x20, OPCR at 0x24, CLKGR1 at 0x28
 *   DDRCDR at 0x2C, SFCCDR at 0x60, MACCDR at 0x54
 *   CPCSR at 0xD4
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
#include "hw/misc/ingenic-t40-cpm.h"

#define CPM_CPCCR           0x00
#define CPM_LCR             0x04
#define CPM_CPPCR           0x0C
#define CPM_CPAPCR          0x10
#define CPM_CPMPCR          0x14
#define CPM_CLKGR           0x20
#define CPM_OPCR            0x24
#define CPM_CLKGR1          0x28
#define CPM_DDRCDR          0x2C
#define CPM_USBPCR          0x3C
#define CPM_USBRDT          0x40
#define CPM_MACCDR          0x54
#define CPM_CPEPCR          0x58
#define CPM_SFCCDR          0x60
#define CPM_I2STCDR         0x70    /* the play I2S clock */
#define CPM_I2SRCDR         0x84    /* the record I2S clock */
#define CPM_CIMCDR          0x90    /* the T40's CIM0CDR, then CIM1, CIM2 */
#define CPM_SRBC            0xC4
#define CPM_CPCSR           0xD4
#define CPM_CPVPCR          0xE0
#define CPM_MACPHY          0xE8

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

#define PLL_EN              (1 << 0)
#define PLL_ON              (1 << 3)
#define CGU_STOP            (1 << 27)
#define CLKGR_ISP           23

/*
 * SRBC stop requests: each acknowledge is the bit below its request, and
 * with no bus transfer in flight a module stops as soon as it is asked.
 * VPU 30/29, HELIX or RADIX 27/26, ISP 21/20.
 */
#define SRBC_ACKS           0x24100000
#define SRBC_ISP_SR         (1 << 22)

static uint32_t ingenic_t40_cpm_srbc(uint32_t v)
{
    return (v & ~SRBC_ACKS) | (v >> 1 & SRBC_ACKS);
}

/* The sensor's MCLK runs while any of the CIM clocks does */
static void ingenic_t40_cpm_update_cim(IngenicT40CpmState *s)
{
    bool on = false;

    for (unsigned i = 0; i < s->cims; i++) {
        on |= !(s->regs[REG_INDEX(CPM_CIMCDR) + i] & CGU_STOP);
    }
    qemu_set_irq(s->cim_clock, on);
}

/*
 * An I2S clock: SCLK_A, MPLL or VPLL by bits 31:30 of the play register
 * (the record register's read 0, as measured on a T41NQ), times M (28:20)
 * over N (19:0) while CE (29) is set, as the vendor kernels' div_i2st/r
 */
static uint64_t ingenic_t40_cpm_i2s_hz(IngenicT40CpmState *s, hwaddr cdr)
{
    uint32_t v = s->regs[REG_INDEX(cdr)];
    uint32_t n = extract32(v, 0, 20);
    uint64_t src;

    switch (extract32(s->regs[REG_INDEX(CPM_I2STCDR)], 30, 2)) {
    case 0:
        src = ingenic_sclka_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                               ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)],
                                              0, s->pll_format));
        break;
    case 1:
        src = ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                             s->pll_format);
        break;
    case 2:
        src = ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPVPCR)], 0,
                             s->pll_format);
        break;
    default:
        return 0;
    }
    return (v & (1 << 29)) && n ? src * extract32(v, 20, 9) / n : 0;
}

static void ingenic_t40_cpm_update_cpu_clk(IngenicT40CpmState *s,
                                           bool propagate)
{
    IngenicPllFormat format = s->pll_format;
    uint64_t hz;

    hz = ingenic_cpu_hz(s->regs[REG_INDEX(CPM_CPCCR)],
                        ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPAPCR)], 0,
                                       format),
                        ingenic_pll_hz(s->regs[REG_INDEX(CPM_CPMPCR)], 0,
                                       format));
    if (propagate) {
        clock_update_hz(s->cpu_clk, hz);
        clock_update_hz(s->i2s_tx, ingenic_t40_cpm_i2s_hz(s, CPM_I2STCDR));
        clock_update_hz(s->i2s_rx, ingenic_t40_cpm_i2s_hz(s, CPM_I2SRCDR));
    } else {
        clock_set_hz(s->cpu_clk, hz);
        clock_set_hz(s->i2s_tx, ingenic_t40_cpm_i2s_hz(s, CPM_I2STCDR));
        clock_set_hz(s->i2s_rx, ingenic_t40_cpm_i2s_hz(s, CPM_I2SRCDR));
    }
}

Clock *ingenic_t40_cpm_cpu_clock(IngenicT40CpmState *s)
{
    ingenic_t40_cpm_update_cpu_clk(s, false);
    return s->cpu_clk;
}

static uint64_t ingenic_t40_cpm_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT40CpmState *s = INGENIC_T40_CPM(opaque);
    uint32_t idx = REG_INDEX(offset);

    if (offset >= INGENIC_T40_CPM_IOSIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds read at 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }

    switch (offset) {
    case CPM_CPCSR:
        return 0xF0000000;

    case CPM_SRBC:
        return ingenic_t40_cpm_srbc(s->regs[idx]);

    default:
        return s->regs[idx];
    }
}

static void ingenic_t40_cpm_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT40CpmState *s = INGENIC_T40_CPM(opaque);
    uint32_t idx = REG_INDEX(offset);

    if (offset >= INGENIC_T40_CPM_IOSIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: out-of-bounds write at 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return;
    }

    switch (offset) {
    case CPM_CPCCR:
        /* Bit 28 is part of the CPU clock select, not a busy bit */
        s->regs[idx] = (uint32_t)value;
        ingenic_t40_cpm_update_cpu_clk(s, true);
        break;

    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPEPCR:
    case CPM_CPVPCR:
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON;
        }
        ingenic_t40_cpm_update_cpu_clk(s, true);
        break;

    case CPM_CPCSR:
        break;

    case CPM_CLKGR:
    case CPM_CLKGR1:
        /* Gates have no busy bit: bit 28 of CLKGR gates the DES */
        s->regs[idx] = (uint32_t)value;
        if (offset == CPM_CLKGR) {
            qemu_set_irq(s->isp_clock, !extract32(value, CLKGR_ISP, 1));
        }
        break;

    case CPM_USBRDT:
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

    case CPM_I2STCDR:
    case CPM_I2SRCDR:
        /* No busy bit: bit 28 is M's. The record register's mux reads 0 */
        s->regs[idx] = offset == CPM_I2SRCDR ? value & 0x3fffffff : value;
        ingenic_t40_cpm_update_cpu_clk(s, true);
        break;

    case CPM_SRBC:
        s->regs[idx] = value & ~SRBC_ACKS;
        qemu_set_irq(s->isp_reset, !!(value & SRBC_ISP_SR));
        break;

    default:
        s->regs[idx] = (uint32_t)value;
        if (value & (1u << 28)) {
            s->regs[idx] &= ~(1u << 28);
        }
        if (offset >= CPM_CIMCDR && offset < CPM_CIMCDR + s->cims * 4) {
            ingenic_t40_cpm_update_cim(s);
        }
        break;
    }
}

static const MemoryRegionOps ingenic_t40_cpm_ops = {
    .read = ingenic_t40_cpm_read,
    .write = ingenic_t40_cpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t40_cpm_reset_hold(Object *obj, ResetType type)
{
    IngenicT40CpmState *s = INGENIC_T40_CPM(obj);
    if (s->boot_saved) {
        memcpy(s->regs, s->boot_regs, sizeof(s->regs));
    } else {
        memset(s->regs, 0, sizeof(s->regs));
    }
    ingenic_t40_cpm_update_cpu_clk(s, true);
}

static void ingenic_t40_cpm_reset_exit(Object *obj, ResetType type)
{
    IngenicT40CpmState *s = INGENIC_T40_CPM(obj);

    qemu_set_irq(s->isp_clock,
                 !extract32(s->regs[REG_INDEX(CPM_CLKGR)], CLKGR_ISP, 1));
    ingenic_t40_cpm_update_cim(s);
}

static void ingenic_t40_cpm_init(Object *obj)
{
    IngenicT40CpmState *s = INGENIC_T40_CPM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t40_cpm_ops, s,
                          TYPE_INGENIC_T40_CPM, INGENIC_T40_CPM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out_named(DEVICE(s), &s->otg_id_change,
                             "otg-id-change", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->isp_clock, "isp-clock", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->isp_reset, "isp-reset", 1);
    qdev_init_gpio_out_named(DEVICE(s), &s->cim_clock, "cim-clock", 1);
    s->cpu_clk = qdev_init_clock_out(DEVICE(s), "cpu-clk");
    s->i2s_tx = qdev_init_clock_out(DEVICE(s), "i2s-tx");
    s->i2s_rx = qdev_init_clock_out(DEVICE(s), "i2s-rx");
}

static const Property ingenic_t40_cpm_properties[] = {
    DEFINE_PROP_BOOL("has-epll", IngenicT40CpmState, has_epll, true),
    DEFINE_PROP_UINT8("pll-format", IngenicT40CpmState, pll_format,
                      INGENIC_PLL_MNOD),
    DEFINE_PROP_UINT8("cims", IngenicT40CpmState, cims, 1),
};

static int ingenic_t40_cpm_post_load(void *opaque, int version_id)
{
    ingenic_t40_cpm_update_cpu_clk(opaque, false);
    return 0;
}

static const VMStateDescription vmstate_ingenic_t40_cpm = {
    .name = "ingenic-t40-cpm",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_t40_cpm_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT40CpmState,
                             INGENIC_T40_CPM_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t40_cpm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_t40_cpm;
    device_class_set_props(dc, ingenic_t40_cpm_properties);
    rc->phases.hold = ingenic_t40_cpm_reset_hold;
    rc->phases.exit = ingenic_t40_cpm_reset_exit;
}

static const TypeInfo ingenic_t40_cpm_type_info = {
    .name = TYPE_INGENIC_T40_CPM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT40CpmState),
    .instance_init = ingenic_t40_cpm_init,
    .class_init = ingenic_t40_cpm_class_init,
};

static void ingenic_t40_cpm_register_types(void)
{
    type_register_static(&ingenic_t40_cpm_type_info);
}

type_init(ingenic_t40_cpm_register_types)
