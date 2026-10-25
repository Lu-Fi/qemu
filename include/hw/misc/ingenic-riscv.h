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
#include "hw/core/clock.h"
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
    uint64_t count_held;        /* T41: the count, still while held */
    bool mbox_pending;
    uint32_t irq_level;         /* the core's interrupt lines */
    bool clock;                 /* the core's clock runs */

    char *model;            /* t31 (default), t23, t32, t41 or a1 */
    bool t23;
    bool t32;               /* the T32's and T33's core */
    bool t41;               /* the T41's core, the A1's too */
    Clock *count_clk;       /* T32: the core's clock, its count's rate */
    uint64_t count_at_hz;   /* T32: the count when that rate last changed */
    uint64_t soc_irqs;      /* T32: the SoC's interrupt lines */
    qemu_irq host_irq;      /* T41, A1: the core's mailbox to the host */
    bool hold;              /* T41: the CCU's CFCR bit 31 */
    QEMUTimer *cmp_timer;   /* T41: the count reaching the compare */
    uint32_t result_addr;   /* DDR address of the results Linux reads */
    uint32_t frame_rate;
    bool night;
};

#endif /* HW_MISC_INGENIC_RISCV_H */
