/*
 * Ingenic I2D, the 2D rotate/mirror/flip engine (T32, T33, T40, T41)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_I2D_H
#define HW_MISC_INGENIC_I2D_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_I2D "ingenic-i2d"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicI2dState, INGENIC_I2D)

struct IngenicI2dState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;

    uint32_t shd_ctrl;
    uint32_t ctrl;
    uint32_t img_size;
    uint32_t img_mode;
    uint32_t addr[4];           /* source Y, UV, destination Y, UV */
    uint32_t stride[4];
    uint32_t irq_state;
    uint32_t irq_mask;
    uint32_t timeout_value;
    uint32_t timeout_mode;
    uint32_t clk_gate;
    uint32_t dbg;
};

#endif /* HW_MISC_INGENIC_I2D_H */
