/*
 * Ingenic IPU (ipu_v13): OSD blending
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_IPU_H
#define HW_MISC_INGENIC_IPU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_IPU "ingenic-ipu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIpuState, INGENIC_IPU)

#define INGENIC_IPU_IOSIZE          0x1000
#define INGENIC_IPU_OSD_CHANNELS    4

struct IngenicIpuState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t regs[INGENIC_IPU_IOSIZE / 4];
};

#endif /* HW_MISC_INGENIC_IPU_H */
