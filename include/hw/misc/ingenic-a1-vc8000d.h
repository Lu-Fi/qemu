/*
 * Ingenic A1 VC8000D video decoder
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_VC8000D_H
#define HW_MISC_INGENIC_A1_VC8000D_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_VC8000D "ingenic-a1-vc8000d"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1Vc8000dState, INGENIC_A1_VC8000D)

#define INGENIC_A1_VC8000D_IOSIZE   0x100000

#define VC8000D_VCMD_BASE       0x0000
#define VC8000D_VCMD_SIZE       0x70
#define VC8000D_DEC_BASE        0x1000
#define VC8000D_DEC_WORDS       0x300
#define VC8000D_CACHE_BASE      0x2000
#define VC8000D_CACHE_WORDS     0x100

typedef struct Vc8000dCodec Vc8000dCodec;

struct IngenicA1Vc8000dState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    AddressSpace *as;

    uint32_t vcmd[VC8000D_VCMD_SIZE / 4];
    uint32_t dec[VC8000D_DEC_WORDS];
    uint32_t cache[VC8000D_CACHE_WORDS];

    /* properties */
    uint32_t vcmd_version;
    uint32_t dec_id;
    uint32_t build_id;

    Vc8000dCodec *codec;
    bool warned_nodec;
};

#endif
