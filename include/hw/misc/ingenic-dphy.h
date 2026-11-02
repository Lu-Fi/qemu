/*
 * Ingenic MIPI D-PHY (T23, T30, T31, T32, T33, T40 and T41)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_DPHY_H
#define HW_MISC_INGENIC_DPHY_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_DPHY "ingenic-dphy"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDphyState, INGENIC_DPHY)

#define INGENIC_DPHY_REGS       1024    /* byte registers in the 4 KiB */

struct IngenicDphyState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    char *model;
    bool alias;                 /* one KiB repeated across the window */
    const uint8_t *wmask;
    const uint8_t *reset;

    bool held;                  /* the CSI host's DPHY_RSTZ low */
    uint8_t regs[INGENIC_DPHY_REGS];
};

#endif /* HW_MISC_INGENIC_DPHY_H */
