/*
 * Ingenic SSI_SLV, the SPI slave of the T31
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_INGENIC_SSI_SLV_H
#define HW_SSI_INGENIC_SSI_SLV_H

#include "hw/core/sysbus.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"

#define TYPE_INGENIC_SSI_SLV "ingenic-ssi-slv"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSsiSlvState, INGENIC_SSI_SLV)

#define TYPE_INGENIC_SSI_SLV_PORT "ingenic-ssi-slv-port"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSsiSlvPort, INGENIC_SSI_SLV_PORT)

#define INGENIC_SSI_SLV_FIFO    64

struct IngenicSsiSlvState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dreq[2];           /* TX and RX DMA requests */

    uint32_t ctrlr, ssienr, mwcr, txftlr, rxftlr, imr, dmacr;
    uint32_t dmatdlr, dmardlr;
    bool txo, rxu, rxo;         /* raw interrupt flags */
    bool txe;                   /* SR.TXE */

    uint32_t tx[INGENIC_SSI_SLV_FIFO], rx[INGENIC_SSI_SLV_FIFO];
    uint32_t tx_rd, tx_n, rx_rd, rx_n;

    /* The shift registers */
    uint32_t tx_shift, rx_shift, tx_last;
    uint32_t bit;               /* bits of the current frame so far */

    bool selected;              /* the chip select input is active */
    bool ported;                /* a port drives it */
};

struct IngenicSsiSlvPort {
    SSIPeripheral parent_obj;

    IngenicSsiSlvState *slv;
};

#endif
