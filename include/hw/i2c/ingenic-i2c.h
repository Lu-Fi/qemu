/*
 * Ingenic I2C controller (a Synopsys DesignWare core)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_I2C_INGENIC_I2C_H
#define HW_I2C_INGENIC_I2C_H

#include "hw/core/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/bitbang_i2c.h"
#include "qom/object.h"

#define TYPE_INGENIC_I2C "ingenic-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicI2cState, INGENIC_I2C)

#define INGENIC_I2C_IOSIZE      0x1000
#define INGENIC_I2C_FIFO        64

struct IngenicI2cState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;                /* named after the controller: i2c0, ... */
    bitbang_i2c_interface bitbang;  /* the pads as GPIOs */
    qemu_irq sda;

    uint32_t ctrl, tar, sar, intm, raw, rxtl, txtl, enb, txabrt;
    uint32_t timing[4];         /* SHCNT, SLCNT, FHCNT, FLCNT */
    uint32_t sdahd, sdasu, dmacr, dmatdlr, dmardlr, ackgc, flt;
    uint8_t rx[INGENIC_I2C_FIFO];
    uint32_t rx_rd, rx_n;
    uint8_t state;              /* idle, or the direction of the transfer */
};

#endif /* HW_I2C_INGENIC_I2C_H */
