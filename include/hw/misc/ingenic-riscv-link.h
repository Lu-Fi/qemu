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
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_RISCV_LINK_H
#define HW_MISC_INGENIC_RISCV_LINK_H

#define INGENIC_RISCV_LINK_VERSION  1

/* The bus requester ID the SoC's side gives the core's accesses */
#define INGENIC_RISCV_REQUESTER     0x5256

#define INGENIC_RISCV_SRAM_BASE     0x12210000

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
};

typedef struct IngenicRiscvMsg {
    uint32_t type;
    uint32_t addr;
    uint32_t size;
    uint32_t data;
} IngenicRiscvMsg;

#endif /* HW_MISC_INGENIC_RISCV_LINK_H */
