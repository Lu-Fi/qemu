/*
 * Ingenic static memory controller (NEMC, NFI on the T20, T21 and T30)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_NEMC_H
#define HW_MISC_INGENIC_NEMC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_NEMC "ingenic-nemc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicNemcState, INGENIC_NEMC)

#define INGENIC_NEMC_BANKS      6
#define INGENIC_NEMC_BANK_BASE  0x16000000  /* CS6; CS1 is 0x1b000000 */
#define INGENIC_NEMC_BANK_SIZE  0x01000000

struct IngenicNemcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion banks;

    uint32_t smcr[INGENIC_NEMC_BANKS];      /* banks 1..6 */
    uint32_t sacr[INGENIC_NEMC_BANKS + 1];  /* banks 0..6 */
    uint32_t nfcsr;
};

/* The registers at base, the bank windows below every other region */
void ingenic_nemc_create(hwaddr base);

#endif /* HW_MISC_INGENIC_NEMC_H */
