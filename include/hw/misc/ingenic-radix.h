/*
 * Ingenic RADIX H.265 encoder (T21, T30)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_RADIX_H
#define HW_MISC_INGENIC_RADIX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_RADIX "ingenic-radix"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicRadixState, INGENIC_RADIX)

#define INGENIC_RADIX_IOSIZE    0x80000     /* 16 modules, 0x8000 apart */

typedef struct IngenicRadixJob IngenicRadixJob;

struct IngenicRadixState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t *regs;             /* the whole window, INGENIC_RADIX_IOSIZE */
    IngenicRadixJob *job;       /* in flight */
};

#endif /* HW_MISC_INGENIC_RADIX_H */
