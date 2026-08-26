/*
 * Ingenic T40 Clock/Power Manager (CPM) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T40_CPM_H
#define HW_MISC_INGENIC_T40_CPM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T40_CPM "ingenic-t40-cpm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT40CpmState, INGENIC_T40_CPM)

#define INGENIC_T40_CPM_IOSIZE  0x100
#define INGENIC_T40_CPM_REGS    (INGENIC_T40_CPM_IOSIZE / 4)

struct IngenicT40CpmState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t regs[INGENIC_T40_CPM_REGS];
    uint32_t boot_regs[INGENIC_T40_CPM_REGS];
    bool boot_saved;
    qemu_irq otg_id_change;

    bool has_epll;
};

#endif /* HW_MISC_INGENIC_T40_CPM_H */
