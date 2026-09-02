/*
 * Ingenic SHA-256 hardware accelerator emulation
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
#define INGENIC_HASH_FIFO_MAX  (128 * 1024)

struct IngenicHashState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t reg_ctrl;
    uint32_t reg_status;
    uint32_t reg_intmask;
    uint32_t reg_dma_addr;
    uint32_t reg_blkcnt;

    uint8_t  fifo[INGENIC_HASH_FIFO_MAX];
    uint32_t fifo_pos;
    bool     started;

    uint8_t  digest[32];
    bool     digest_valid;
    uint8_t  digest_idx;
};

#endif
