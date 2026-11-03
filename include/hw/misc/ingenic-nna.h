/*
 * Ingenic NNA: the T40's, T41's and A1's neural network accelerator
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_NNA_H
#define HW_MISC_INGENIC_NNA_H

#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_NNA "ingenic-nna"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicNnaState, INGENIC_NNA)

#define INGENIC_NNA_WINDOW      (64 * KiB)  /* NNDMA and its descriptors */
#define INGENIC_NNA_NNDMA_REGS  8

struct IngenicNnaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion oram;

    uint32_t oram_size;
    char *log_path;
    FILE *log;

    uint8_t window[INGENIC_NNA_WINDOW];
    uint32_t nndma[INGENIC_NNA_NNDMA_REGS];
};

/* MXU3's NN instructions land here (wired to MIPSCPU's nna_op) */
void ingenic_nna_op(void *opaque, unsigned op, uint32_t imm, uint8_t *vec);

#endif /* HW_MISC_INGENIC_NNA_H */
