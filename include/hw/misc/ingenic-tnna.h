/*
 * Ingenic TNNA: the T32's and T33's neural network accelerator
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_TNNA_H
#define HW_MISC_INGENIC_TNNA_H

#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_TNNA "ingenic-tnna"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicTnnaState, INGENIC_TNNA)

#define INGENIC_TNNA_REGS       0x60        /* the M, D and Q registers */
#define INGENIC_TNNA_WFCW       8
#define INGENIC_TNNA_DMA_REGS   16          /* each channel's */
#define INGENIC_TNNA_ORAM_SIZE  (64 * KiB)
#define INGENIC_TNNA_RAM_SIZE   (384 * KiB)
#define INGENIC_TNNA_CVA_REGS   25

struct IngenicTnnaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion cva_iomem;

    uint32_t greg[8];
    uint32_t regs[INGENIC_TNNA_REGS];
    uint32_t wfcw[INGENIC_TNNA_WFCW];
    uint32_t dma[3][INGENIC_TNNA_DMA_REGS];
    uint8_t oram[INGENIC_TNNA_ORAM_SIZE];
    uint8_t ram[INGENIC_TNNA_RAM_SIZE];     /* what the host side sees */
    uint32_t cva[INGENIC_TNNA_CVA_REGS];
};

#endif /* HW_MISC_INGENIC_TNNA_H */
