/*
 * Ingenic camera SoC RISC-V co-processor (Zeratul fast boot), as the
 * MIPS side sees it
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_RISCV_H
#define HW_MISC_INGENIC_RISCV_H

#include "qemu/units.h"
#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-riscv-link.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_INGENIC_RISCV "ingenic-riscv"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicRiscvState, INGENIC_RISCV)

#define INGENIC_RISCV_SRAM_SIZE     (64 * KiB)
#define INGENIC_RISCV_CTRL_SIZE     (8 * KiB)

struct IngenicRiscvState {
    SysBusDevice parent_obj;

    MemoryRegion sram;
    MemoryRegion ctrl;
    QEMUTimer *frame_timer;

    uint32_t regs[INGENIC_RISCV_CTRL_SIZE / 4];
    bool running;
    uint32_t frames;            /* unlinked: the frames counted */

    /* Linked to the core's QEMU */
    CharFrontend link;
    bool hello;                 /* the core has said hello */
    IngenicRiscvMsg rx;
    unsigned rx_len;
    QEMUTimer *tick;            /* the timer's period */
    int64_t count_start;        /* when the timer's count restarted, ns */
    bool mbox_pending;
    uint32_t irq_level;         /* the core's interrupt lines */
    bool clock;                 /* the core's clock runs */

    uint32_t result_addr;   /* DDR address of the results Linux reads */
    uint32_t frame_rate;
    bool night;
};

#endif /* HW_MISC_INGENIC_RISCV_H */
