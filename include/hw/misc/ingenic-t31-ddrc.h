/*
 * Ingenic T31 DDR Controller and Innosilicon PHY emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T31_DDRC_H
#define HW_MISC_INGENIC_T31_DDRC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_DDRC "ingenic-t31-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31DdrcState, INGENIC_T31_DDRC)

#define INGENIC_T31_DDRC_IOSIZE     0x1000
#define INGENIC_T31_DDRC_REGS       (INGENIC_T31_DDRC_IOSIZE / 4)

#define INGENIC_T31_DDRPHY_IOSIZE   0x2000
#define INGENIC_T31_DDRPHY_REGS     (INGENIC_T31_DDRPHY_IOSIZE / 4)

struct IngenicT31DdrcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion ddrc_iomem;
    MemoryRegion phy_iomem;

    uint32_t ddrc_regs[INGENIC_T31_DDRC_REGS];
    uint32_t phy_regs[INGENIC_T31_DDRPHY_REGS];
};

#endif /* HW_MISC_INGENIC_T31_DDRC_H */
