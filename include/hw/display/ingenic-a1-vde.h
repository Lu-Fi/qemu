/*
 * Ingenic A1 VDE: the video display engine (mixer and output timing)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_INGENIC_A1_VDE_H
#define HW_DISPLAY_INGENIC_A1_VDE_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "hw/misc/ingenic-a1-cpm.h"

#define TYPE_INGENIC_A1_VDE "ingenic-a1-vde"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1VdeState, INGENIC_A1_VDE)

#define INGENIC_A1_VDE_SIZE     0x80000
#define INGENIC_A1_VDE_INTP     5

struct IngenicA1VdeState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *frame, *half;
    int64_t frame_ns;
    QemuConsole *con;

    uint32_t regs[INGENIC_A1_VDE_SIZE / 4];
    uint32_t intp[INGENIC_A1_VDE_INTP];    /* the raw status of each */

    uint32_t refresh_hz;        /* 0: from the timing */
    IngenicA1CpmState *cpm;     /* CLKGR0 bit 28 gates the window */
};

#endif /* HW_DISPLAY_INGENIC_A1_VDE_H */
