/*
 * Ingenic JPEG engine (T32)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_JPEG_H
#define HW_MISC_INGENIC_JPEG_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_JPEG "ingenic-jpeg"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicJpegState, INGENIC_JPEG)

#define INGENIC_JPEG_IOSIZE     0x10000

typedef struct IngenicJpegJob IngenicJpegJob;

struct IngenicJpegState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[INGENIC_JPEG_IOSIZE / 4];
    IngenicJpegJob *job;        /* in flight */
};

#endif /* HW_MISC_INGENIC_JPEG_H */
