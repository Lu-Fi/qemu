/*
 * Ingenic XBurst2 SRAM: the on-chip SRAM the L2 cache shares with the NNA
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_XBURST2_SRAM_H
#define HW_MISC_INGENIC_XBURST2_SRAM_H

#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_XBURST2_SRAM "ingenic-xburst2-sram"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicXBurst2SramState, INGENIC_XBURST2_SRAM)

#define INGENIC_XBURST2_SRAM_BASE   0x12600000
#define INGENIC_XBURST2_SRAM_WINDOW (1 * MiB)

struct IngenicXBurst2SramState {
    SysBusDevice parent_obj;

    MemoryRegion window;        /* 1 MiB at 0x12600000 */
    MemoryRegion ram;           /* the SRAM itself */
    MemoryRegion oram;          /* the NNA's share, an alias of ram */
    MemoryRegion mirror;        /* the upper half, on a 512 KiB SRAM */
    MemoryRegion l2;            /* the L2's share, as the CPUs see it */

    uint32_t size;              /* 1 MiB on a T40 or A1, 512 KiB on a T41 */
    uint32_t share;             /* the L2's share now, in bytes */
    bool logged;                /* an access to the L2's share was logged */
};

/* MSCR bits 12:10: the L2's share, 0 none to 4 (or more) 1 MiB */
void ingenic_xburst2_sram_set_l2(IngenicXBurst2SramState *s, unsigned field);

/*
 * The NNDMA's own path: @offset from 0x12600000, straight into the SRAM,
 * the L2's share included
 */
void ingenic_xburst2_sram_dma(IngenicXBurst2SramState *s, uint32_t offset,
                              void *buf, uint32_t len, bool is_write);

#endif /* HW_MISC_INGENIC_XBURST2_SRAM_H */
