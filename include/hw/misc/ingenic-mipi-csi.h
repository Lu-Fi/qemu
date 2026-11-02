/*
 * Ingenic MIPI CSI-2 receiver
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_MIPI_CSI_H
#define HW_MISC_INGENIC_MIPI_CSI_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_MIPI_CSI "ingenic-mipi-csi"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMipiCsiState, INGENIC_MIPI_CSI)

#define INGENIC_MIPI_CSI_IOSIZE 0x1000
#define INGENIC_MIPI_CSI_REGS   256     /* one KiB, repeated in the window */

struct IngenicMipiCsiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq reset;                     /* high while it passes no frame */
    qemu_irq phy_reset;                 /* high while DPHY_RSTZ is low */
    char *model;
    unsigned lanes;
    const uint32_t *wmask;
    const uint32_t *reset_values;
    uint32_t regs[INGENIC_MIPI_CSI_REGS];
};

#endif /* HW_MISC_INGENIC_MIPI_CSI_H */
