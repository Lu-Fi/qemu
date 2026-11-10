/*
 * Ingenic XBurst2: the L2's share of the SRAM it splits with the NNA
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_XBURST2_L2_H
#define HW_MIPS_INGENIC_XBURST2_L2_H

#include "qemu/bitops.h"
#include "target/mips/cpu.h"
#include "hw/misc/ingenic-xburst2-sram.h"

/*
 * CCU MSCR bits 12:10 set the L2's share (0 none, 1-4 128 KiB to 1 MiB):
 * Config2's sets per way follow (SS 0, then 2 to 5), and the SRAM's window
 * moves the ORAM up past it, as a T40XP, T41NQ and A1N read
 */
static inline void ingenic_xburst2_l2_split(MIPSCPU **cpu, unsigned ncpu,
                                            IngenicXBurst2SramState *sram,
                                            uint32_t mscr)
{
    unsigned field = MIN(extract32(mscr, 10, 3), 4);

    for (unsigned i = 0; i < ncpu; i++) {
        if (cpu[i]) {
            cpu[i]->env.CP0_Config2 = deposit32(cpu[i]->env.CP0_Config2,
                                                CP0C2_SS, 4,
                                                field ? field + 1 : 0);
        }
    }
    ingenic_xburst2_sram_set_l2(sram, field);
}

#endif /* HW_MIPS_INGENIC_XBURST2_L2_H */
