/*
 * Ingenic DRAW_BOX (T40, T41)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_DRAWBOX_H
#define HW_MISC_INGENIC_DRAWBOX_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_DRAWBOX "ingenic-drawbox"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDrawboxState, INGENIC_DRAWBOX)

#define INGENIC_DRAWBOX_BOXES   255     /* the 8-bit box pointers' reach */

struct IngenicDrawboxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;

    uint32_t ctrl;
    uint32_t yb;
    uint32_t cb;
    uint32_t stride;
    uint32_t img_wh;
    uint32_t color[3];          /* Y, U, V: one byte per colour */
    uint32_t timeout;
    uint32_t ram[2 * INGENIC_DRAWBOX_BOXES];
    uint32_t ram_words;         /* words written since the reset */
};

#endif /* HW_MISC_INGENIC_DRAWBOX_H */
