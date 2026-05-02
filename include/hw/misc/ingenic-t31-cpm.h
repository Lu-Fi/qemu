/*
 * Ingenic T31 Clock/Power Manager (CPM) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T31_CPM_H
#define HW_MISC_INGENIC_T31_CPM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_CPM "ingenic-t31-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31CpmState, INGENIC_T31_CPM)

#define INGENIC_T31_CPM_IOSIZE  0x100
#define INGENIC_T31_CPM_REGS    (INGENIC_T31_CPM_IOSIZE / 4)

struct IngenicT31CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_T31_CPM_REGS];
};

#endif /* HW_MISC_INGENIC_T31_CPM_H */
