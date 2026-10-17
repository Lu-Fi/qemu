/*
 * Ingenic SAR A/D controller (SADC)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_ADC_INGENIC_SADC_H
#define HW_ADC_INGENIC_SADC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_SADC "ingenic-sadc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicSadcState, INGENIC_SADC)

#define INGENIC_SADC_CHANNELS   8

struct IngenicSadcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    uint32_t adena, adcfg, adctrl, adstate, adclk;
    uint32_t aux[INGENIC_SADC_CHANNELS];    /* each channel's sample */
};

#endif
