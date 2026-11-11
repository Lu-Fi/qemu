/*
 * Ingenic A1 HDMI: the transmitter's controller and PHY, with a monitor
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_INGENIC_A1_HDMI_H
#define HW_DISPLAY_INGENIC_A1_HDMI_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_HDMI "ingenic-a1-hdmi"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1HdmiState, INGENIC_A1_HDMI)

#define INGENIC_A1_HDMI_CTL_SIZE    0x20000
#define INGENIC_A1_HDMI_PHY_SIZE    0x1000
#define INGENIC_A1_HDMI_EDID_SIZE   256

struct IngenicA1HdmiState {
    SysBusDevice parent_obj;

    MemoryRegion ctl, phy;
    qemu_irq irq;
    qemu_irq hpd;               /* the HPD pin: high with a monitor */

    uint32_t regs[INGENIC_A1_HDMI_CTL_SIZE / 4];
    uint32_t phy_regs[INGENIC_A1_HDMI_PHY_SIZE / 4];
    uint8_t edid[INGENIC_A1_HDMI_EDID_SIZE];
    uint32_t edid_len;

    /* properties */
    bool connected;
    char *edid_file;
    uint32_t xres, yres;        /* the monitor's preferred mode */
};

#endif /* HW_DISPLAY_INGENIC_A1_HDMI_H */
