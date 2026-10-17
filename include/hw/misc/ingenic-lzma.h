/*
 * Ingenic hardware LZMA decompressor (JZLZMA)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_LZMA_H
#define HW_MISC_INGENIC_LZMA_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_LZMA "ingenic-lzma"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicLzmaState, INGENIC_LZMA)

#define INGENIC_LZMA_IOSIZE     0x10000

struct IngenicLzmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    char *model;            /* "t32", "t40" or "t41": the register map */
    int kind;
    uint32_t regs[64];
};

#endif
