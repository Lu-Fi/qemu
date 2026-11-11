/*
 * Ingenic AIP: the T41's image pre-processor
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_AIP_H
#define HW_MISC_INGENIC_AIP_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_AIP "ingenic-aip"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAipState, INGENIC_AIP)

#define INGENIC_AIP_UNITS       3           /* T, F and P */
#define INGENIC_AIP_UNIT_REGS   64

struct IngenicAipState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq[INGENIC_AIP_UNITS];

    uint32_t regs[INGENIC_AIP_UNITS][INGENIC_AIP_UNIT_REGS];
    bool a1;                        /* an A1N's timeout and T idle */
    uint32_t t_done;                /* the status a T step raises */
    uint32_t t_row;                 /* the next source line T converts */
};

#endif /* HW_MISC_INGENIC_AIP_H */
