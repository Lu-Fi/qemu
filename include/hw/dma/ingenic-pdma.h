/*
 * Ingenic T31 PDMA (Programmable DMA) controller
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DMA_INGENIC_PDMA_H
#define HW_DMA_INGENIC_PDMA_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_PDMA "ingenic-pdma"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicPdmaState, INGENIC_PDMA)

#define PDMA_NR_CHANNELS  32

struct IngenicT31PdmaChannel {
    uint32_t dsa;
    uint32_t dta;
    uint32_t dtc;
    uint32_t drt;
    uint32_t dcs;
    uint32_t dcm;
    uint32_t dda;
    uint32_t dsd;
};

struct IngenicPdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    struct IngenicT31PdmaChannel ch[PDMA_NR_CHANNELS];

    uint32_t dmac;
    uint32_t dirqp;
    uint32_t ddr;
    uint32_t ddrs;
    uint32_t dmacp;
    uint32_t dsirqp;
    uint32_t dsirqm;
    uint32_t dcirqp;
    uint32_t dcirqm;
    uint32_t dmcs;
    uint32_t dmnmb;
    uint32_t dmsmb;
    uint32_t dmint;
};

#endif
