/*
 * Ingenic A1 RTC emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_RTC_H
#define HW_MISC_INGENIC_A1_RTC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_RTC "ingenic-a1-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1RtcState, INGENIC_A1_RTC)

struct IngenicA1RtcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t clkcnten;
    uint32_t clkcnt;        /* CLKCNTINFO when last started or stopped */
    int64_t clkcnt_ns;      /* and when */
};

#endif
