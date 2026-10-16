/*
 * Ingenic DES engine
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_DES_H
#define HW_MISC_INGENIC_DES_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_DES "ingenic-des"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDesState, INGENIC_DES)

#define INGENIC_DES_FIFO    16

struct IngenicDesState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq dreq[2];           /* DIN and DOUT DMA requests */

    uint32_t cr1, cr2;
    uint32_t key[6], iv[2];     /* as the engine takes them, big-endian */
    uint32_t in[INGENIC_DES_FIFO], out[INGENIC_DES_FIFO];
    uint32_t in_rd, in_n, out_rd, out_n;
    uint32_t hold[2];           /* a finished block waiting for DOUT room */
    bool held;
    uint64_t chain;
};

#endif /* HW_MISC_INGENIC_DES_H */
