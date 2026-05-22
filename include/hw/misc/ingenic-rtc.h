/*
 * Ingenic XBurst Real Time Clock
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_RTC_H
#define HW_MISC_INGENIC_RTC_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_RTC "ingenic-rtc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicRtcState, INGENIC_RTC)

#define INGENIC_RTC_NUM_REGS  0x40    /* 0x00 .. 0xff */

struct IngenicRtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *alarm_timer;

    int64_t  offset;      /* RTCSR = host seconds + offset */
    uint32_t regs[INGENIC_RTC_NUM_REGS];
};

#endif /* HW_MISC_INGENIC_RTC_H */
