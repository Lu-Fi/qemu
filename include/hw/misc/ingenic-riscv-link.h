/*
 * Ingenic camera SoC RISC-V co-processor: the link between the SoC's
 * QEMU and the core's
 *
 * The core runs in qemu-system-riscv32 (machine ingenic-t31-riscv) and
 * the rest of the SoC in qemu-system-mipsel, joined by a UNIX socket (a
 * chardev on the SoC's side). The SoC's side passes the core the file
 * descriptors of DDR and of the core's SRAM, which both map; every other
 * access the core makes crosses the link and is done on the SoC's bus,
 * its answer awaited.
 *
 * Messages are four little-endian words. The core opens with HELLO; the
 * SoC answers with a MEM for each memory, then IRQ for each raised line,
 * CLOCK and RUN, and after that sends RUN, CLOCK and IRQ as they change.
 * READ and WRITE are each answered by one DATA, which other messages may
 * precede.
 *
 * A T31 core polls its timer's count in every delay, and some firmware
 * delays forever once Linux has stopped it. The SoC sends a COUNT when a
 * write sets the count running anew, before that write's DATA, and the
 * core then reads the count from its own clock instead of across the link.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_RISCV_LINK_H
#define HW_MISC_INGENIC_RISCV_LINK_H

#define INGENIC_RISCV_LINK_VERSION  2

/* The bus requester ID the SoC's side gives the core's accesses */
#define INGENIC_RISCV_REQUESTER     0x5256

#define INGENIC_RISCV_SRAM_BASE     0x12210000

/* The T31's timer count, at 696 MHz */
#define INGENIC_RISCV_T31_COUNT     0x12220020
#define INGENIC_RISCV_T31_COUNT_HZ  696000000

enum {
    /* core to SoC */
    IRL_HELLO = 1,          /* data: link version */
    IRL_READ,               /* addr, size (1, 2 or 4) */
    IRL_WRITE,              /* addr, size, data */
    /* SoC to core */
    IRL_DATA,               /* data: what a READ read; 0 for a WRITE */
    IRL_MEM,                /* addr, size: a memory, its fd attached */
    IRL_RUN,                /* data: 1 starts the core from reset, 0 holds it */
    IRL_IRQ,                /* addr: interrupt id (0-31), data: level */
    IRL_CLOCK,              /* data: 1 the core's clock runs, 0 it stops */
    IRL_COUNT,              /* addr: wrap period (0: none), data: the count */
};

typedef struct IngenicRiscvMsg {
    uint32_t type;
    uint32_t addr;
    uint32_t size;
    uint32_t data;
} IngenicRiscvMsg;

#endif /* HW_MISC_INGENIC_RISCV_LINK_H */
