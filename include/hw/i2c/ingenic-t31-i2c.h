/*
 * Ingenic T31 I2C controller (Synopsys DesignWare derivative)
 *
 * Minimal "auto-NACK" emulation: any addressed transfer reports
 * TX-abort with the address-NACK source so the kernel returns
 * -ENXIO (device not present) and probe ceases. Enough for the
 * I2C platform driver and bus scan to complete.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_INGENIC_T31_I2C_H
#define HW_I2C_INGENIC_T31_I2C_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_I2C "ingenic-t31-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31I2cState, INGENIC_T31_I2C)

#define INGENIC_T31_I2C_IOSIZE      0x1000

struct IngenicT31I2cState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t ctrl;
    uint32_t tar;
    uint32_t intm;
    uint32_t intst;
    uint32_t enb;
    uint32_t txabrt;    /* abort source register */
    bool xfer_active;   /* true between TAR write and STOP */
};

#endif /* HW_I2C_INGENIC_T31_I2C_H */
