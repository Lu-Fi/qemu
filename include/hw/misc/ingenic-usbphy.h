/*
 * Ingenic USB PHY (Innosilicon USB 2.0), the T20 to the T41 and the A1
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_USBPHY_H
#define HW_MISC_INGENIC_USBPHY_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_USBPHY "ingenic-usbphy"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicUsbPhyState, INGENIC_USBPHY)

#define INGENIC_USBPHY_REGS     256     /* byte registers in a PHY's 1 KiB */
#define INGENIC_USBPHY_MAX      3       /* the A1's three */

struct IngenicUsbPhyState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    char *model;
    uint32_t phys;
    const uint8_t *wmask;
    const uint8_t *reset;

    uint8_t regs[INGENIC_USBPHY_MAX][INGENIC_USBPHY_REGS];
};

#endif /* HW_MISC_INGENIC_USBPHY_H */
