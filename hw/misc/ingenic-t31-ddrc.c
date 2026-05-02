/*
 * Ingenic T31 DDR Controller and Innosilicon PHY emulation
 *
 * Provides the minimum register behavior needed for SPL DDR init:
 * PLL lock, training completion, write leveling, calibration done,
 * and LMR (Load Mode Register) completion.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/misc/ingenic-t31-ddrc.h"

/* DDRC register offsets (from AHB_BASE 0x134F0000) */
#define DDRC_STATUS     0x00
#define DDRC_CFG        0x04
#define DDRC_CTRL       0x08
#define DDRC_LMR        0x0C

/* PHY register offsets (from PHY_BASE 0x13011000) */
#define PHY_WL_DONE         0xC0
#define PHY_PLL_LOCK        0xC8
#define PHY_CALIB_DONE      0xCC
#define PHY_INIT_COMP       0xD0

/* APB PHY registers (at APB_BASE 0x13012000, offset 0x1000 from PHY_BASE) */
#define APB_PHY_INIT        0x108C
/* DQS delay registers */
#define PHY_DQS_DELAY_L     0x190
#define PHY_DQS_DELAY_H     0x194

#define PHY_REG_INDEX(offset)   ((offset) / sizeof(uint32_t))
#define DDRC_REG_INDEX(offset)  ((offset) / sizeof(uint32_t))

/* ---- DDRC ---- */

static uint64_t ingenic_t31_ddrc_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicT31DdrcState *s = INGENIC_T31_DDRC(opaque);

    if (offset >= INGENIC_T31_DDRC_IOSIZE) {
        return 0;
    }

    switch (offset) {
    case DDRC_LMR:
        return 0;
    default:
        return s->ddrc_regs[DDRC_REG_INDEX(offset)];
    }
}

static void ingenic_t31_ddrc_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicT31DdrcState *s = INGENIC_T31_DDRC(opaque);

    if (offset >= INGENIC_T31_DDRC_IOSIZE) {
        return;
    }

    s->ddrc_regs[DDRC_REG_INDEX(offset)] = (uint32_t)value;
}

static const MemoryRegionOps ingenic_t31_ddrc_ops = {
    .read = ingenic_t31_ddrc_read,
    .write = ingenic_t31_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- DDR PHY ---- */

static uint64_t ingenic_t31_ddrphy_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    IngenicT31DdrcState *s = INGENIC_T31_DDRC(opaque);

    if (offset >= INGENIC_T31_DDRPHY_IOSIZE) {
        return 0;
    }

    switch (offset) {
    case PHY_PLL_LOCK:
        return 0x08;
    case PHY_WL_DONE:
        return 0x03;
    case PHY_CALIB_DONE:
        return 0x03;
    case PHY_INIT_COMP:
        return 0x01;
    case APB_PHY_INIT:
        return 0x07;
    case PHY_DQS_DELAY_L:
    case PHY_DQS_DELAY_H:
        return 0x60;
    default:
        if (offset < INGENIC_T31_DDRPHY_IOSIZE) {
            return s->phy_regs[PHY_REG_INDEX(offset)];
        }
        return 0;
    }
}

static void ingenic_t31_ddrphy_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    IngenicT31DdrcState *s = INGENIC_T31_DDRC(opaque);

    if (offset < INGENIC_T31_DDRPHY_IOSIZE) {
        s->phy_regs[PHY_REG_INDEX(offset)] = (uint32_t)value;
    }
}

static const MemoryRegionOps ingenic_t31_ddrphy_ops = {
    .read = ingenic_t31_ddrphy_read,
    .write = ingenic_t31_ddrphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- Device lifecycle ---- */

static void ingenic_t31_ddrc_reset_hold(Object *obj, ResetType type)
{
    IngenicT31DdrcState *s = INGENIC_T31_DDRC(obj);

    memset(s->ddrc_regs, 0, sizeof(s->ddrc_regs));
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
}

static void ingenic_t31_ddrc_init(Object *obj)
{
    IngenicT31DdrcState *s = INGENIC_T31_DDRC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ddrc_iomem, obj, &ingenic_t31_ddrc_ops, s,
                          "ingenic-t31-ddrc", INGENIC_T31_DDRC_IOSIZE);
    sysbus_init_mmio(sbd, &s->ddrc_iomem);

    memory_region_init_io(&s->phy_iomem, obj, &ingenic_t31_ddrphy_ops, s,
                          "ingenic-t31-ddrphy", INGENIC_T31_DDRPHY_IOSIZE);
    sysbus_init_mmio(sbd, &s->phy_iomem);
}

static const VMStateDescription vmstate_ingenic_t31_ddrc = {
    .name = "ingenic-t31-ddrc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ddrc_regs, IngenicT31DdrcState,
                             INGENIC_T31_DDRC_REGS),
        VMSTATE_UINT32_ARRAY(phy_regs, IngenicT31DdrcState,
                             INGENIC_T31_DDRPHY_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t31_ddrc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_t31_ddrc;
    rc->phases.hold = ingenic_t31_ddrc_reset_hold;
}

static const TypeInfo ingenic_t31_ddrc_type_info = {
    .name = TYPE_INGENIC_T31_DDRC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31DdrcState),
    .instance_init = ingenic_t31_ddrc_init,
    .class_init = ingenic_t31_ddrc_class_init,
};

static void ingenic_t31_ddrc_register_types(void)
{
    type_register_static(&ingenic_t31_ddrc_type_info);
}

type_init(ingenic_t31_ddrc_register_types)
