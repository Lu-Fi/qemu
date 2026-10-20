/*
 * Ingenic HELIX video encoder (T10, T20, T21, T23, T30)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_HELIX_H
#define HW_MISC_INGENIC_HELIX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_HELIX "ingenic-helix"
OBJECT_DECLARE_TYPE(IngenicHelixState, IngenicHelixClass, INGENIC_HELIX)

#define TYPE_INGENIC_T10_HELIX "ingenic-t10-helix"
#define TYPE_INGENIC_T21_HELIX "ingenic-t21-helix"
#define TYPE_INGENIC_T30_HELIX "ingenic-t30-helix"    /* and the T20's */

#define INGENIC_HELIX_IOSIZE    0x100000

typedef struct IngenicHelixJob IngenicHelixJob;
typedef struct IngenicHelixLayout IngenicHelixLayout;

struct IngenicHelixClass {
    SysBusDeviceClass parent_class;
    const IngenicHelixLayout *layout;
};

struct IngenicHelixState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t *regs;             /* the whole window, INGENIC_HELIX_IOSIZE */
    IngenicHelixJob *job;       /* in flight */
    uint32_t ivdc_frame;        /* scenes taken over IVDC */
};

#endif /* HW_MISC_INGENIC_HELIX_H */
