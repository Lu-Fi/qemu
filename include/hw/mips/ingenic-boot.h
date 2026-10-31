/*
 * Ingenic boards: a direct Linux boot handed over as thingino's U-Boot does
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_BOOT_H
#define HW_MIPS_INGENIC_BOOT_H

#include "exec/hwaddr.h"
#include "hw/core/boards.h"
#include "target/mips/cpu.h"

typedef struct {
    bool camera;    /* thingino reserves rmem for media; not on an NVR */
    bool ispmem;    /* T10 and T20 reserve 8 MiB more, below rmem */
} IngenicBootDefaults;

void ingenic_boot_linux(MachineState *machine, MIPSCPU *cpu,
                        BlockBackend *flash, const IngenicBootDefaults *d,
                        hwaddr kernel_end);

#endif
