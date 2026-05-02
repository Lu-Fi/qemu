/*
 * Ingenic T31 OS Timer (OST) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T31_OST_H
#define HW_MISC_INGENIC_T31_OST_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "migration/vmstate.h"

#define TYPE_INGENIC_T31_OST "ingenic-t31-ost"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31OstState, INGENIC_T31_OST)

struct IngenicT31OstState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion container;
    MemoryRegion iomem;
    MemoryRegion wdt_iomem;

    int64_t base_time;
    uint32_t cnth_buf;
    uint32_t data_reg;
    uint32_t csr;
    QEMUTimer *wdt_timer;
    bool wdt_fired;
};

static const VMStateDescription vmstate_ingenic_t31_ost = {
    .name = "ingenic-t31-ost",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base_time, IngenicT31OstState),
        VMSTATE_UINT32(cnth_buf, IngenicT31OstState),
        VMSTATE_UINT32(data_reg, IngenicT31OstState),
        VMSTATE_UINT32(csr, IngenicT31OstState),
        VMSTATE_END_OF_LIST()
    }
};

#endif /* HW_MISC_INGENIC_T31_OST_H */
