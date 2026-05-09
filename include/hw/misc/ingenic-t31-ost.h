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

#define TCU_NR_CHANNELS  8

struct IngenicT31TcuChannel {
    uint32_t tdfr;
    uint32_t tdhr;
    uint32_t tcnt;
    uint32_t tcsr;
};

struct IngenicT31OstState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    /* OST (64-bit free-running counter) */
    int64_t base_time;
    uint32_t cnth_buf;
    uint32_t data_reg;
    uint32_t csr;

    /* TCU global registers */
    uint32_t ter;
    uint32_t tsr;
    uint32_t tfr;
    uint32_t tmr;
    uint32_t tstr;

    /* TCU per-channel registers */
    struct IngenicT31TcuChannel ch[TCU_NR_CHANNELS];
};

static const VMStateDescription vmstate_tcu_channel = {
    .name = "ingenic-t31-tcu-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(tdfr, struct IngenicT31TcuChannel),
        VMSTATE_UINT32(tdhr, struct IngenicT31TcuChannel),
        VMSTATE_UINT32(tcnt, struct IngenicT31TcuChannel),
        VMSTATE_UINT32(tcsr, struct IngenicT31TcuChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_t31_ost = {
    .name = "ingenic-t31-ost",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base_time, IngenicT31OstState),
        VMSTATE_UINT32(cnth_buf, IngenicT31OstState),
        VMSTATE_UINT32(data_reg, IngenicT31OstState),
        VMSTATE_UINT32(csr, IngenicT31OstState),
        VMSTATE_UINT32(ter, IngenicT31OstState),
        VMSTATE_UINT32(tsr, IngenicT31OstState),
        VMSTATE_UINT32(tfr, IngenicT31OstState),
        VMSTATE_UINT32(tmr, IngenicT31OstState),
        VMSTATE_UINT32(tstr, IngenicT31OstState),
        VMSTATE_STRUCT_ARRAY(ch, IngenicT31OstState, TCU_NR_CHANNELS, 1,
                             vmstate_tcu_channel, struct IngenicT31TcuChannel),
        VMSTATE_END_OF_LIST()
    }
};

#endif /* HW_MISC_INGENIC_T31_OST_H */
