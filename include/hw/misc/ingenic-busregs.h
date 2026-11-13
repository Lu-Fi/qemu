/*
 * Ingenic bus windows that only hold registers: the T32's HARB0, the AXI
 * arbiters and the bus monitors
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_BUSREGS_H
#define HW_MISC_INGENIC_BUSREGS_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_BUSREGS "ingenic-busregs"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicBusRegsState, INGENIC_BUSREGS)

/* Words up to the T41 monitor's 0x2400 */
#define INGENIC_BUSREGS_MAX     2304

typedef struct {
    uint16_t offset;
    uint32_t reset;
    uint32_t wmask;
    uint32_t sticky;            /* bits a 1 sets and a 0 leaves */
} IngenicBusReg;

struct IngenicBusRegsState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    char *model;
    uint32_t chip_id;           /* the T32's HARB0 holds it at 0x2c */
    uint32_t period;            /* the registers repeat every period bytes */
    bool gated;                 /* clock gate shut: reads 0, drops writes */

    uint32_t wmask[INGENIC_BUSREGS_MAX];
    uint32_t sticky[INGENIC_BUSREGS_MAX];
    uint32_t regs[INGENIC_BUSREGS_MAX];
};

/* A window of the given model at base; chip_id for the T32's HARB0 */
DeviceState *ingenic_busregs_create(const char *model, hwaddr base,
                                    uint32_t chip_id);

#endif /* HW_MISC_INGENIC_BUSREGS_H */
