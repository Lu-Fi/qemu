/*
 * Ingenic LDC, the radial lens distortion correction engine (T30, T32, T33,
 * T41)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_LDC_H
#define HW_MISC_INGENIC_LDC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_LDC "ingenic-ldc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicLdcState, INGENIC_LDC)

#define INGENIC_LDC_LUT 256

struct IngenicLdcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUBH *bh;
    QEMUTimer *timeout;

    char *model;                /* "t32" (0x13040000 layout), "t41" or "t30" */
    bool t30;
    bool t41;                   /* the T32's, its own VERSION, no online bit */

    uint32_t ctrl;              /* T30 CTR; T32 WORK_EN */
    uint32_t mode;              /* T32 0x08 */
    uint32_t status;
    uint32_t mask;              /* T32 INT_MASK */
    uint32_t res;
    uint32_t coef[2];           /* k1 | k2 << 16: T32 one, T30 x and y */
    uint32_t zoom;              /* x | y << 16 */
    uint32_t r2;
    uint32_t udis;
    uint32_t len;
    uint32_t fill;              /* T30 */
    uint32_t cfg;               /* T32 0x60 */
    uint32_t ctl74;             /* T32 0x74 */
    uint32_t time_out;          /* T32 0x78 */
    uint32_t addr[4];           /* Y in, Y out, UV in, UV out */
    uint32_t stride[4];
    uint32_t lut[2][INGENIC_LDC_LUT];
};

#endif /* HW_MISC_INGENIC_LDC_H */
