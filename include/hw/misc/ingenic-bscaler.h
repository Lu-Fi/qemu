/*
 * Ingenic BSCALER: the T40's block scaler
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_BSCALER_H
#define HW_MISC_INGENIC_BSCALER_H

#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-lzma.h"
#include "qom/object.h"

#define TYPE_INGENIC_BSCALER "ingenic-bscaler"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicBscalerState, INGENIC_BSCALER)

#define INGENIC_BSCALER_IOSIZE  0x100

struct IngenicBscalerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[2];            /* FRMC, FRMT */
    IngenicLzmaState *lzma;     /* the LZMA mode of the same block */

    uint32_t regs[INGENIC_BSCALER_IOSIZE / 4];
};

#endif /* HW_MISC_INGENIC_BSCALER_H */
