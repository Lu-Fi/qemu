/*
 * Ingenic HELIX video encoder (the T23's)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_HELIX_H
#define HW_MISC_INGENIC_HELIX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_HELIX "ingenic-helix"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicHelixState, INGENIC_HELIX)

#define INGENIC_HELIX_IOSIZE    0x100000

typedef struct IngenicHelixJob IngenicHelixJob;

struct IngenicHelixState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t *regs;             /* the whole window, INGENIC_HELIX_IOSIZE */
    IngenicHelixJob *job;       /* in flight */
};

#endif /* HW_MISC_INGENIC_HELIX_H */
