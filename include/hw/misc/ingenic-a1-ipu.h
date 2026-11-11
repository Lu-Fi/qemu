/*
 * Ingenic A1 IPU: the descriptor-driven 2D engine
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_IPU_H
#define HW_MISC_INGENIC_A1_IPU_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_IPU "ingenic-a1-ipu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1IpuState, INGENIC_A1_IPU)

#define INGENIC_A1_IPU_IOSIZE   0x10000
#define INGENIC_A1_IPU_NREGS    (0x1200 / 4)

struct IngenicA1IpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t regs[INGENIC_A1_IPU_NREGS];
    uint32_t status;            /* +0x20, cleared through +0x24 */
    uint32_t status_mask;       /* +0x2c */
    uint32_t iip_raw;           /* +0x1014, cleared through +0x1018 */
    uint32_t iip_mask;          /* +0x101c */
    uint32_t chain;             /* +0x14: the descriptor to run at +0x10 */
    int64_t busy_ns;            /* how long the descriptor in flight takes */
    bool running;
};

#endif
