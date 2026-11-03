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
#define INGENIC_NNA_REGS        0x80
#define INGENIC_NNA_WRAM        (576 * KiB) /* a T40's 3x3 W; a T41 uses 256 */
#define INGENIC_NNA_BTRAM       (16 * KiB)
#define INGENIC_NNA_FRAM        (128 * KiB) /* 1024 lines of 128 bytes */
#define INGENIC_NNA_OUT         1024
#define INGENIC_NNA_SLOTS       1024        /* a WFCW program, unrolled */

struct IngenicNnaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion oram;

    uint32_t oram_size;
    uint64_t desc_mask;
    bool t40;                   /* the T40's engine (4-pixel MACs) */
    char *log_path;
    FILE *log;

    uint8_t window[INGENIC_NNA_WINDOW];
    uint32_t nndma[INGENIC_NNA_NNDMA_REGS];

    /* the engine the NN instructions drive */
    uint32_t reg[INGENIC_NNA_REGS];
    uint16_t wfcw[16];
    uint32_t wfcw_ptr;
    uint32_t wr_w;              /* W write entry (256 bytes) */
    uint32_t bt_w;              /* BT write unit (64 bytes) */
    uint32_t wr_r;              /* W read entry */
    uint32_t wr_base;
    uint32_t port_line[2];      /* F write line, per port */
    uint32_t port_half[2];      /* 64-byte halves written into the slot */
    uint32_t port_slot[2];      /* slot of the WFCW program */
    uint32_t fline;             /* T40: the F write line */
    int64_t sums[8 * 32];       /* [pixel][output channel] */
    int64_t ocb[8 * 32];        /* the sums CPOC copied */
    int32_t bt[64];             /* B[32], T[32] that RDBT latched */
    bool bt_valid;
    uint8_t out[INGENIC_NNA_OUT];
    uint32_t out_len, out_rd;
    uint8_t wram[INGENIC_NNA_WRAM];
    uint8_t btram[INGENIC_NNA_BTRAM];
    uint8_t fram[INGENIC_NNA_FRAM];

    /* the WFCW programs unrolled into slots (not migrated: rebuilt) */
    bool slots_dirty;
    uint32_t nslots[2];
    int16_t slot_inc[2][INGENIC_NNA_SLOTS];
    bool slot_jump[2][INGENIC_NNA_SLOTS];
};

/* MXU3's NN instructions land here (wired to MIPSCPU's nna_op) */
void ingenic_nna_op(void *opaque, unsigned op, uint32_t imm, uint8_t *vec);

#endif /* HW_MISC_INGENIC_NNA_H */
