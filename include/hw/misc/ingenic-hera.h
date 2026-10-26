/*
 * Ingenic HERA video encoder (T32)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_HERA_H
#define HW_MISC_INGENIC_HERA_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/misc/ingenic-isp.h"

#define TYPE_INGENIC_HERA "ingenic-hera"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicHeraState, INGENIC_HERA)

#define INGENIC_HERA_IOSIZE     0xa0000     /* 20 modules, 0x8000 apart */

typedef struct IngenicHeraJob IngenicHeraJob;

#define INGENIC_HERA_CODERS     4

struct IngenicHeraState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t *regs;             /* the whole window, INGENIC_HERA_IOSIZE */
    IngenicHeraJob *job;        /* in flight, or its stream half sent */
    GHashTable *refs;           /* references kept here, by ring */
    IngenicIspState *isp;       /* whose channel 0 comes over IVDC */
    /* H.264 coders idle between jobs, by picture size: host state */
    struct IngenicAvpuH264 *coders[INGENIC_HERA_CODERS];
};

#endif /* HW_MISC_INGENIC_HERA_H */
