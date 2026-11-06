/*
 * Ingenic T33 and T32 Pro ISP: the table-driven LDC and its rotator
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_ISP_LDC_H
#define HW_MISC_INGENIC_ISP_LDC_H

#include "exec/hwaddr.h"
#include "migration/vmstate.h"

#define INGENIC_ISP_LDC_SIZE    0x100

typedef struct IngenicIspLdc {
    uint32_t regs[INGENIC_ISP_LDC_SIZE / 4];
} IngenicIspLdc;

extern const VMStateDescription vmstate_ingenic_isp_ldc;

void ingenic_isp_ldc_reset(IngenicIspLdc *l);
uint32_t ingenic_isp_ldc_read(IngenicIspLdc *l, hwaddr reg);

/* A write; returns true when the interrupt's level may have changed */
bool ingenic_isp_ldc_write(IngenicIspLdc *l, hwaddr reg, uint32_t v);
bool ingenic_isp_ldc_irq(const IngenicIspLdc *l);

#endif /* HW_MISC_INGENIC_ISP_LDC_H */
