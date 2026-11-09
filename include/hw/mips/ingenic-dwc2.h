/*
 * Core configuration of the Ingenic parts' DWC2 OTG controllers
 *
 * GSNPSID and GHWCFG1-4 read with devmem under Linux, the dwc2 driver
 * loaded, on a T10L, T20X, T21N, T23N, T30X, T31X (Wyze V3), T32LQ, T40NN,
 * T40XP, T41NQ, T41LQ and A1N (all three of its cores). The five words
 * are the same on every one of them except for GHWCFG3 bits 11 and 10:
 *
 *   0x0df80ce8  T10L, T20X, T21N, T23N, T30X, T31X, T40NN, T40XP
 *   0x0df804e8  T32LQ, T41NQ, T41LQ, A1N core 1
 *   0x0df800e8  A1N cores 0 and 2
 *
 * A part that was not on the bench takes the value of the nearest one
 * that was; each SoC file says which.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_DWC2_H
#define HW_MIPS_INGENIC_DWC2_H

#include "qapi/error.h"
#include "qom/object.h"

#define INGENIC_DWC2_GHWCFG3_T31    0x0df80ce8
#define INGENIC_DWC2_GHWCFG3_T32    0x0df804e8
#define INGENIC_DWC2_GHWCFG3_A1_0   0x0df800e8

/* Set the core's identification and configuration words before realize. */
static inline void ingenic_dwc2_config(Object *dwc2, uint32_t ghwcfg3)
{
    object_property_set_uint(dwc2, "gsnpsid", 0x4f54300a, &error_abort);
    object_property_set_uint(dwc2, "ghwcfg1", 0x00000000, &error_abort);
    object_property_set_uint(dwc2, "ghwcfg2", 0x228be050, &error_abort);
    object_property_set_uint(dwc2, "ghwcfg3", ghwcfg3, &error_abort);
    object_property_set_uint(dwc2, "ghwcfg4", 0xe3f08020, &error_abort);
}

#endif
