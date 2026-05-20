/*
 * Ingenic A1 Clock/Power Manager (CPM) emulation
 *
 * The CPM controls PLL configuration, clock gating, power domains,
 * and clock dividers for the A1 XBurst2 SoC. This model implements
 * the registers needed to boot SPL and U-Boot.
 *
 * The A1 has additional PLLs (EPLL, VPLL) at different offsets than
 * the T31 family, and its clock status register (CPCSR) lives at
 * offset 0xEC rather than 0xD4.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
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
#define CPM_USBPCR          0x3C
#define CPM_USBRDT          0x40
#define CPM_CPEPCR_T30      0x58    /* EPLL control (T30 offset) */
#define CPM_CPCSR_T31       0xD4    /* Clock status (T31/T32/T33) */
#define CPM_CPVPCR_T31      0xE0    /* VPLL control (T31 offset) */
#define CPM_CPCSR_A1        0xEC    /* Clock status (A1) */

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

/* PLL control register bits */
#define PLL_EN              (1 << 0)
#define PLL_ON              (1 << 3)

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
    case CPM_CPCSR_T31:
    case CPM_CPCSR_A1:
        /*
         * Clock process status. Return "all stable, not busy".
         * Bits 28-31 = PLL-mux-stable flags, bits 0-2 = divider-busy.
         * A1 SPL polls offset 0xEC; T31/T32/T33 SPL polls 0xD4.
         * Both are handled here for compatibility.
         */
        return 0xF0000000;

    default:
        return s->regs[idx];
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
    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPEPCR:
    case CPM_CPVPCR:
    case CPM_CPEPCR_T30:
    case CPM_CPVPCR_T31:
        /*
         * PLL control registers. When software sets PLL_EN (bit 0),
         * immediately set PLL_ON (bit 3) to indicate the PLL has
         * locked - instant lock emulation.
         */
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON;
        }
        break;

    case CPM_CPCSR_T31:
    case CPM_CPCSR_A1:
        /* Clock status is read-only */
        break;

    case CPM_USBRDT:
        /*
         * USB reset detect timer. Bit 23 selects USB host vs device
         * role on some SoCs. Track transitions and fire the OTG ID
         * change GPIO when bit 23 flips.
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
        break;
    }
}

static const MemoryRegionOps ingenic_a1_cpm_ops = {
    .read = ingenic_a1_cpm_read,
    .write = ingenic_a1_cpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_a1_cpm_reset_hold(Object *obj, ResetType type)
{
    IngenicA1CpmState *s = INGENIC_A1_CPM(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void ingenic_a1_cpm_init(Object *obj)
{
    IngenicA1CpmState *s = INGENIC_A1_CPM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_a1_cpm_ops, s,
                          TYPE_INGENIC_A1_CPM, INGENIC_A1_CPM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out_named(DEVICE(s), &s->otg_id_change,
                             "otg-id-change", 1);
}

static const VMStateDescription vmstate_ingenic_a1_cpm = {
    .name = "ingenic-a1-cpm",
    .version_id = 1,
    .minimum_version_id = 1,
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
