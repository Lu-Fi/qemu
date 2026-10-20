/*
 * Ingenic IVDC, the ISP to encoder direct connection (T23, T32, T33, T41)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_IVDC_H
#define HW_MISC_INGENIC_IVDC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_IVDC "ingenic-ivdc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIvdcState, INGENIC_IVDC)

#define INGENIC_IVDC_IOSIZE     0x10000
#define INGENIC_IVDC_SENSORS    4

struct IngenicIvdcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[INGENIC_IVDC_IOSIZE / 4];
};

#endif /* HW_MISC_INGENIC_IVDC_H */
