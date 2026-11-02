/*
 * Ingenic VO: the T40, T41 and A1's BT.656/BT.1120 video output
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_INGENIC_VO_H
#define HW_DISPLAY_INGENIC_VO_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_VO "ingenic-vo"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicVoState, INGENIC_VO)

#define INGENIC_VO_QUEUE    4

typedef struct IngenicVoPair {
    uint32_t y, uv;
} IngenicVoPair;

struct IngenicVoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    QEMUTimer *frame;
    int64_t deadline;           /* the frame's end, virtual ns */
    QemuConsole *con;
    bool t41;                   /* the T41's VO, else the T40's and A1's */

    uint32_t top_con;           /* as written */
    uint32_t int_en, int_val, int_clc;
    uint32_t size, stride, uv_addr, blk_info, con_par, vblk, hblk;
    uint32_t tran_en, time_out, yuv_clip;
    uint32_t y_latch;           /* Y_ADDR waits here for its UV_ADDR */
    IngenicVoPair queue[INGENIC_VO_QUEUE];
    uint32_t qlen;
    IngenicVoPair cur;
    bool running;               /* a frame is being sent */
    bool started;               /* a pair taken since TRAN_EN or reset */
    uint32_t dry;               /* boundaries met with the queue empty */
    bool underrun;
};

#endif /* HW_DISPLAY_INGENIC_VO_H */
