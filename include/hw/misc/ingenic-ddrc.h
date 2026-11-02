/*
 * Ingenic T31 DDR Controller and Innosilicon PHY emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_DDRC_H
#define HW_MISC_INGENIC_DDRC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_DDRC "ingenic-ddrc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDdrcState, INGENIC_DDRC)

#define INGENIC_DDRC_IOSIZE     0x1000
#define INGENIC_DDRC_REGS       (INGENIC_DDRC_IOSIZE / 4)

#define INGENIC_T31_DDRPHY_IOSIZE   0x2000
#define INGENIC_T31_DDRPHY_REGS     (INGENIC_T31_DDRPHY_IOSIZE / 4)

/* the APB page's 64 words, repeating every 0x100 */
#define INGENIC_DDR_APB_REGS        64

struct IngenicDdrcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion ddrc_iomem;
    MemoryRegion phy_iomem;
    MemoryRegion apb_iomem;
    MemoryRegion ddrc1_iomem;

    char *apb_layout;
    const struct DdrApbLayout *apb;

    uint32_t ddrc_regs[INGENIC_DDRC_REGS];
    uint32_t phy_regs[INGENIC_T31_DDRPHY_REGS];
    uint32_t apb_regs[INGENIC_DDR_APB_REGS];
    uint64_t apb_written;
    bool ddr_up;                /* the SPL has written the controller */
};

#endif /* HW_MISC_INGENIC_DDRC_H */
