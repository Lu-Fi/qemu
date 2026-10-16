/*
 * Ingenic SSI, the SPI master of the T and A series
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SSI_INGENIC_SSI_H
#define HW_SSI_INGENIC_SSI_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_SSI "ingenic-ssi"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSsiState, INGENIC_SSI)

#define INGENIC_SSI_FIFO    128

struct IngenicSsiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq cs[2];             /* the CE0 and CE1 pins, as levels */
    qemu_irq dreq[2];           /* TX and RX DMA requests */
    SSIBus *bus;
    QEMUTimer *timer;

    uint32_t cr0, cr1, itr, icr, gr, rcnt;
    uint32_t spen, spcnt;
    bool end, undr, over;
    uint32_t tx[INGENIC_SSI_FIFO], rx[INGENIC_SSI_FIFO];
    uint32_t tx_rd, tx_n, rx_rd, rx_n;
    bool xfer;                  /* a transfer is open: CE asserted, END clear */
    bool stalled;               /* ... held open by UNFIN with nothing to send */
    bool sel;                   /* the selected CE pin is asserted */
    bool cs_level[2];
    uint32_t group;             /* interval mode: frames since the interval */
    uint32_t rcvd;              /* receive-only frames, against RCNT */
    int64_t last_write_ns;

    char *bus_name;
    uint32_t devclk;            /* the CPM's SSI clock, Hz */
    bool dreq_status;           /* SR[31:24]: the DMA requests (T32, T41) */
    bool spcnt_regs;            /* SPEN, SPLOAD, SPCNT (T41) */
    bool fie;                   /* CR0.FIE, and END cleared by writing 0 */
};

#endif /* HW_SSI_INGENIC_SSI_H */
