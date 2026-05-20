/*
 * Ingenic A1 Clock/Power Manager (CPM) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_CPM_H
#define HW_MISC_INGENIC_A1_CPM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_CPM "ingenic-a1-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1CpmState, INGENIC_A1_CPM)

#define INGENIC_A1_CPM_IOSIZE  0x200
#define INGENIC_A1_CPM_REGS    (INGENIC_A1_CPM_IOSIZE / 4)

struct IngenicA1CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_A1_CPM_REGS];
    /* OTG ID change notify pin (level out: 1=device/B, 0=host/A) */
    qemu_irq otg_id_change;
};

#endif /* HW_MISC_INGENIC_A1_CPM_H */
