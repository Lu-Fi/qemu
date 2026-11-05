/*
 * Ingenic DPU: the T40's display controller
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_INGENIC_DPU_H
#define HW_DISPLAY_INGENIC_DPU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_DPU "ingenic-dpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDpuState, INGENIC_DPU)

#define INGENIC_DPU_LAYERS      4
#define INGENIC_DPU_FRM_WORDS   11
#define INGENIC_DPU_LAY_WORDS   12
#define INGENIC_DPU_SRD_WORDS   5

/* The display interface registers both blocks share, from 0x8000 */
typedef struct IngenicDpuDisp {
    uint32_t disp_com;
    uint32_t tft[5];            /* HSYNC, VSYNC, HDE, VDE, CFG */
    uint32_t slcd[6];           /* CFG, WR_DUTY, TIMING, FRM_SIZE, SLOW, IF */
} IngenicDpuDisp;

struct IngenicDpuState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;                 /* the pixel clock, the CPM's LPCDR */
    bool gate_open;             /* CLKGR bit 24 clear */
    QEMUTimer *frame;
    QemuConsole *con;

    uint32_t frm_addr, srd_addr;
    uint32_t st, intc, com_cfg;
    uint32_t pcfg[8];           /* 0x2018 to 0x2034 */
    uint32_t tlb_glbc, tlba, tlbv, tlb_stat;
    uint32_t csc[2][4];         /* layer 0's and layer 1's */
    IngenicDpuDisp disp;

    /* The channels */
    bool cmp_on, srd_on;        /* the composer's or simple read's running */
    bool cmp_gen_stop, srd_gen_stop;
    bool to_srd, to_cmp;        /* change at this frame's end */
    bool direct, wb;            /* the composer frame's outputs */
    uint32_t eod_mask;          /* DISP_END's mask in the frame's descriptor */
    uint32_t desc_mask;         /* the descriptor's other interrupt masks */
    uint32_t frm[INGENIC_DPU_FRM_WORDS];
    uint32_t lay[INGENIC_DPU_LAYERS][INGENIC_DPU_LAY_WORDS];
    uint32_t srd[INGENIC_DPU_SRD_WORDS];
    uint32_t frm_cnt, lay_cnt[INGENIC_DPU_LAYERS], srd_cnt;
    uint32_t frm_site, srd_site, wdma_site, rdma_site;
    uint32_t lay_site[INGENIC_DPU_LAYERS];
};

#endif /* HW_DISPLAY_INGENIC_DPU_H */
