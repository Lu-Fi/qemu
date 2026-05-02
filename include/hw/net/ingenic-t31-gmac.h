/*
 * Ingenic T31 GMAC (Synopsys DesignWare MAC) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_INGENIC_T31_GMAC_H
#define HW_NET_INGENIC_T31_GMAC_H

#include "hw/core/sysbus.h"
#include "net/net.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_GMAC "ingenic-t31-gmac"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31GmacState, INGENIC_T31_GMAC)

#define INGENIC_T31_GMAC_IOSIZE     0x2000
#define INGENIC_T31_GMAC_MAC_REGS   (0x1000 / 4)
#define INGENIC_T31_GMAC_DMA_REGS   (0x1000 / 4)
#define INGENIC_T31_GMAC_PHY_REGS   32

struct IngenicT31GmacState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    NICState *nic;
    NICConf conf;

    uint32_t mac_regs[INGENIC_T31_GMAC_MAC_REGS];
    uint32_t dma_regs[INGENIC_T31_GMAC_DMA_REGS];
    uint16_t phy_regs[INGENIC_T31_GMAC_PHY_REGS];
    QEMUTimer *tx_timer;
};

#endif /* HW_NET_INGENIC_T31_GMAC_H */
