/*
 * Ingenic T31 GPIO controller emulation
 *
 * Three ports (A, B, C) at 0x10010000 + i * 0x1000, plus a shadow
 * configuration page at 0x10017000.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_INGENIC_T31_GPIO_H
#define HW_GPIO_INGENIC_T31_GPIO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_GPIO "ingenic-t31-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31GpioState, INGENIC_T31_GPIO)

#define INGENIC_T31_GPIO_NR_PORTS    3
#define INGENIC_T31_GPIO_PORT_OFF    0x1000
#define INGENIC_T31_GPIO_SHADOW_OFF  0x7000
#define INGENIC_T31_GPIO_IOSIZE      0x10000

struct IngenicT31GpioPort {
    uint32_t pin;       /* 0x00 - level (input) */
    uint32_t intr;      /* 0x10 - interrupt enable */
    uint32_t msk;       /* 0x20 - interrupt mask */
    uint32_t pat1;      /* 0x30 - pattern bit 1 */
    uint32_t pat0;      /* 0x40 - pattern bit 0 */
    uint32_t flg;       /* 0x50 - interrupt flag */
    uint32_t puen;      /* 0x110 - pull-up enable */
    uint32_t pden;      /* 0x120 - pull-down enable */
};

struct IngenicT31GpioState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    struct IngenicT31GpioPort port[INGENIC_T31_GPIO_NR_PORTS];
    struct IngenicT31GpioPort shadow;
};

#endif /* HW_GPIO_INGENIC_T31_GPIO_H */
