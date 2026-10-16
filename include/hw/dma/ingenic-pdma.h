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
#define PDMA_NR_REQUESTS  64

struct IngenicT31PdmaChannel {
    uint32_t dsa;
    uint32_t dta;
    uint32_t dtc;
    uint32_t drt;
    uint32_t dcs;
    uint32_t dcm;
    uint32_t dda;
    uint32_t dsd;
    bool loaded;        /* descriptor mode: a descriptor is in the registers */
    uint32_t desc;      /* ... its address */
    uint32_t doa;       /* ... and its link offset */
    uint32_t block;     /* stride mode: data units per sub-block */
    uint32_t req;       /* the request it waits on, as latched */
};

struct IngenicPdmaState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion tcsm;          /* the MCU's memory, 0x2000 in */
    qemu_irq irq;
    qemu_irq dack[PDMA_NR_REQUESTS];    /* a channel serves this request */
    QEMUBH *bh;

    struct IngenicT31PdmaChannel ch[PDMA_NR_CHANNELS];

    uint32_t dmac;
    uint32_t dirqp;
    uint32_t ddr;               /* DDB: doorbells rung, not yet fetched */
    uint32_t dip;
    uint32_t dmacp;
    uint32_t dsirqp;
    uint32_t dsirqm;
    uint32_t dcirqp;
    uint32_t dcirqm;
    uint32_t dmcs;
    uint32_t dmnmb;
    uint32_t dmsmb;
    uint32_t dmint;

    uint64_t dreq;              /* peripheral request levels, by DRT */
    uint64_t dack_level;
    bool busy, again;
    bool kicked;                /* the BH has work for the channels */
};

#endif
