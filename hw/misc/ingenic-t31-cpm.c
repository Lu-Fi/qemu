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
#include "migration/vmstate.h"
#include "hw/misc/ingenic-t31-cpm.h"

/* Register offsets */
#define CPM_CPCCR       0x00
#define CPM_LCR         0x04
#define CPM_RSR         0x08
#define CPM_CPPCR       0x0C
#define CPM_CPAPCR      0x10
#define CPM_CPMPCR      0x14
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
#define CPM_SSICDR      0x74
#define CPM_CPCSR       0xD4
#define CPM_CPVPCR      0xE0
#define CPM_GMACPHYC    0xE8

#define REG_INDEX(offset)   ((offset) / sizeof(uint32_t))

/* PLL control register bits */
#define PLL_EN          (1 << 0)
#define PLL_ON          (1 << 3)

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
        return 0;

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
    case CPM_CPAPCR:
    case CPM_CPMPCR:
    case CPM_CPVPCR:
        s->regs[idx] = (uint32_t)value;
        if (value & PLL_EN) {
            s->regs[idx] |= PLL_ON;
        }
        break;

    case CPM_CPCSR:
        break;

    default:
        s->regs[idx] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps ingenic_t31_cpm_ops = {
    .read = ingenic_t31_cpm_read,
    .write = ingenic_t31_cpm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_cpm_reset_hold(Object *obj, ResetType type)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(obj);

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[REG_INDEX(CPM_CPCCR)]  = 0x95800000;
    s->regs[REG_INDEX(CPM_RSR)]    = 0x00000001;
    s->regs[REG_INDEX(CPM_CPPCR)]  = 0x00000000;
    s->regs[REG_INDEX(0x34)]       = 0x00000001;
    s->regs[REG_INDEX(CPM_CPAPCR)] = PLL_EN | PLL_ON;
    s->regs[REG_INDEX(CPM_CPMPCR)] = PLL_EN | PLL_ON;
}

static void ingenic_t31_cpm_init(Object *obj)
{
    IngenicT31CpmState *s = INGENIC_T31_CPM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_cpm_ops, s,
                          TYPE_INGENIC_T31_CPM, INGENIC_T31_CPM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const VMStateDescription vmstate_ingenic_t31_cpm = {
    .name = "ingenic-t31-cpm",
    .version_id = 1,
    .minimum_version_id = 1,
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
    rc->phases.hold = ingenic_t31_cpm_reset_hold;
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
