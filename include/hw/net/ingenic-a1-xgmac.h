/*
 * Ingenic A1 XGMAC Ethernet controller
 *
 * The A1 GMAC is a Synopsys DesignWare dwxgmac2 MAC: MDIO at 0x200,
 * DMA block at 0x3000, version/feature registers at 0x110-0x128.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_INGENIC_A1_XGMAC_H
#define HW_NET_INGENIC_A1_XGMAC_H

#include "hw/core/sysbus.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_XGMAC "ingenic-a1-xgmac"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1XgmacState, INGENIC_A1_XGMAC)

#define INGENIC_A1_XGMAC_IOSIZE  0x4000

struct IngenicA1XgmacState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;

    uint32_t regs[INGENIC_A1_XGMAC_IOSIZE / 4];
    uint32_t rx_idx;
    uint16_t phy_regs[32];
    uint32_t tx_cur;
};

#endif /* HW_NET_INGENIC_A1_XGMAC_H */
