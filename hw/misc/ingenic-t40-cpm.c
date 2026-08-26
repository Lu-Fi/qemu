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
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
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
#define CPM_CPCSR           0xD4
#define CPM_CPVPCR          0xE0
#define CPM_MACPHY          0xE8

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

#define PLL_EN              (1 << 0)
#define PLL_ON              (1 << 3)

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
    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPEPCR:
    case CPM_CPVPCR:
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON;
        }
        break;

    case CPM_CPCSR:
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

    default:
        s->regs[idx] = (uint32_t)value;
        if (value & (1u << 28)) {
            s->regs[idx] &= ~(1u << 28);
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
}

static const Property ingenic_t40_cpm_properties[] = {
    DEFINE_PROP_BOOL("has-epll", IngenicT40CpmState, has_epll, true),
};

static const VMStateDescription vmstate_ingenic_t40_cpm = {
    .name = "ingenic-t40-cpm",
    .version_id = 1,
    .minimum_version_id = 1,
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
