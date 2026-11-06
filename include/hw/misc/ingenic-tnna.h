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
#define INGENIC_TNNA_ORAM_SIZE  (128 * KiB)     /* the T32Pro's; the T32's 64 */
#define INGENIC_TNNA_RAM_SIZE   (384 * KiB)
#define INGENIC_TNNA_CVA_REGS   25
#define INGENIC_TNNA_LUT_REGS   32
#define INGENIC_TNNA_FRAM       8192        /* entries of 8 channels x 4 bits */
#define INGENIC_TNNA_WRAM_SIZE  (64 * KiB)      /* the T32Pro's; the T32's 32 */
#define INGENIC_TNNA_BTRAM_SIZE (8 * KiB)       /* the T32Pro's; the T32's 4 */
#define INGENIC_TNNA_LANES      16          /* pixels one NNMAC computes */
#define INGENIC_TNNA_OC         8           /* output channels it computes */

struct IngenicTnnaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion cva_iomem;
    MemoryRegion pro_nna;       /* the T32Pro's windows */
    MemoryRegion pro_dma;
    MemoryRegion pro_oram;
    MemoryRegion pro_lut;
    bool pro;
    uint32_t oram_size;
    uint32_t wram_size;
    uint32_t btram_size;
    uint32_t reg_mask[INGENIC_TNNA_REGS];   /* the bits each register keeps */

    uint32_t greg[8];
    uint32_t regs[INGENIC_TNNA_REGS];
    uint32_t wfcw[INGENIC_TNNA_WFCW];
    uint32_t dma[3][INGENIC_TNNA_DMA_REGS];
    uint8_t oram[INGENIC_TNNA_ORAM_SIZE];
    uint8_t ram[INGENIC_TNNA_RAM_SIZE];     /* what the host side sees */
    uint32_t cva[INGENIC_TNNA_CVA_REGS];
    uint32_t lut[INGENIC_TNNA_LUT_REGS];

    uint8_t fram[INGENIC_TNNA_FRAM][INGENIC_TNNA_OC];
    uint8_t wram[INGENIC_TNNA_WRAM_SIZE];
    uint8_t btram[INGENIC_TNNA_BTRAM_SIZE];
    int64_t sum[INGENIC_TNNA_LANES][INGENIC_TNNA_OC];
    int64_t ocb[INGENIC_TNNA_LANES][INGENIC_TNNA_OC];  /* what CPOC copied */
    int32_t bt[3][INGENIC_TNNA_OC];         /* what RDBT latched: B, T, TN */
    uint32_t wf_line;                       /* NNDWF's next F-RAM line */
    uint32_t or_raddr;                      /* NNDWF's next ORAM column */
    uint32_t wr_wptr;                       /* NNDWW's next W-RAM unit */
    uint32_t bt_wptr;                       /* NNDWBT's next BT-RAM unit */
};

#endif /* HW_MISC_INGENIC_TNNA_H */
