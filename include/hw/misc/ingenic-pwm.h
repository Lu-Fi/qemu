/*
 * Ingenic XBurst2 PWM controller (the dedicated block at 0x13460000)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_PWM_H
#define HW_MISC_INGENIC_PWM_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_PWM "ingenic-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicPwmState, INGENIC_PWM)

#define INGENIC_PWM_NUM_REGS  0x100   /* registers span 0x000..0x3ff */

struct IngenicPwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[INGENIC_PWM_NUM_REGS];
};

#endif /* HW_MISC_INGENIC_PWM_H */
