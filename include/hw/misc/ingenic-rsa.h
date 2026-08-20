/*
 * Ingenic RSA hardware accelerator emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_RSA_H
#define HW_MISC_INGENIC_RSA_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_RSA "ingenic-rsa"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicRsaState, INGENIC_RSA)

#define INGENIC_RSA_IOSIZE    0x1000
#define INGENIC_RSA_MAXWORDS  64

struct IngenicRsaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t reg_ctrl;
    uint32_t reg_exp;

    uint32_t modulus[INGENIC_RSA_MAXWORDS];
    uint32_t message[INGENIC_RSA_MAXWORDS];
    uint32_t result[INGENIC_RSA_MAXWORDS];
    uint32_t mod_pos;
    uint32_t msg_pos;
    uint32_t res_pos;
    uint32_t nwords;
    bool     computed;
};

#endif
