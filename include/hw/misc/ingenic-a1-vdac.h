/*
 * Ingenic A1 video DAC (the VGA output's converter)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_VDAC_H
#define HW_MISC_INGENIC_A1_VDAC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_VDAC "ingenic-a1-vdac"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1VdacState, INGENIC_A1_VDAC)

#define INGENIC_A1_VDAC_IOSIZE  0x1000

struct IngenicA1VdacState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
};

#endif
