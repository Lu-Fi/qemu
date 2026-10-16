/*
 * Ingenic HASH engine: MD5, SHA-1, SHA-224, SHA-256, SHA-384, SHA-512
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_HASH_H
#define HW_MISC_INGENIC_HASH_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_HASH "ingenic-hash"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicHashState, INGENIC_HASH)

#define INGENIC_HASH_IOSIZE    0x1000

struct IngenicHashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t hscr, hssr, hsintm, hssa, hstc, hsbnum;
    uint32_t h32[8];            /* MD5, SHA-1, SHA-224/256 state */
    uint64_t h64[8];            /* SHA-384/512 state */
    uint32_t blk[32];           /* the block being filled, message words */
    uint32_t blk_n;
    uint32_t out[16];           /* HSDO: the state after the last block */
    uint32_t out_idx;

    bool t33;                   /* T33 register layout */
};

#endif
