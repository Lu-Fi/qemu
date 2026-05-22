/*
 * Ingenic XBurst2 Digital True Random Number Generator (DTRNG)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_DTRNG_H
#define HW_MISC_INGENIC_DTRNG_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_DTRNG "ingenic-dtrng"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDtrngState, INGENIC_DTRNG)

struct IngenicDtrngState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t cfg;
};

#endif /* HW_MISC_INGENIC_DTRNG_H */
