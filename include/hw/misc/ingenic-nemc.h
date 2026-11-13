/*
 * Ingenic static memory controller (NEMC, NFI on the T20, T21 and T30)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_NEMC_H
#define HW_MISC_INGENIC_NEMC_H

#include "hw/core/qdev.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_NEMC "ingenic-nemc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicNemcState, INGENIC_NEMC)

#define INGENIC_NEMC_BANKS      6
#define INGENIC_NEMC_BANK_BASE  0x16000000  /* CS6; CS1 is 0x1b000000 */
#define INGENIC_NEMC_BANK_SIZE  0x01000000

#define TYPE_INGENIC_NEMC_NAND "ingenic-nemc-nand"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicNemcNandState, INGENIC_NEMC_NAND)

typedef enum {
    NAND_IDLE, NAND_ID, NAND_STAT, NAND_READ_ADDR, NAND_DATA_OUT,
    NAND_RND_ADDR, NAND_PROG_ADDR, NAND_PROG_DATA, NAND_RND_IN_ADDR,
    NAND_ERASE_ADDR,
} IngenicNemcNandMode;

/* A parallel NAND behind bank "bank", see ingenic-nemc-nand.c */
struct IngenicNemcNandState {
    DeviceState parent_obj;

    BlockBackend *blk;
    qemu_irq rb;
    uint32_t bank;
    uint32_t page_size;
    uint32_t oob_size;
    uint32_t pages_per_block;
    uint64_t id_bytes;

    uint32_t pages;
    uint32_t row_cycles;
    uint32_t buf_size;
    uint8_t *buf;
    uint8_t id[5];
    uint32_t mode;
    uint32_t naddr;
    uint8_t addr[8];
    uint32_t ptr;
    uint8_t id_addr;
    uint8_t status;
};

struct IngenicNemcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion banks;     /* 0x16000000 to 0x1fffffff */
    MemoryRegion gate;
    MemoryRegion *bootrom;

    uint32_t smcr[INGENIC_NEMC_BANKS];      /* banks 1..6 */
    uint32_t sacr[INGENIC_NEMC_BANKS + 1];  /* banks 0..6 */
    uint32_t nfcsr;

    IngenicNemcNandState *nand[INGENIC_NEMC_BANKS + 1];     /* banks 1..6 */
};

/* The chip's three ports, as the bank's window gives them */
void ingenic_nemc_nand_cmd(IngenicNemcNandState *n, uint8_t cmd);
void ingenic_nemc_nand_addr(IngenicNemcNandState *n, uint8_t val);
uint8_t ingenic_nemc_nand_read(IngenicNemcNandState *n);
void ingenic_nemc_nand_write(IngenicNemcNandState *n, uint8_t val);

/* Put a chip behind bank 1..6 of the one NEMC; false if it is not free */
bool ingenic_nemc_attach_nand(IngenicNemcNandState *n, int bank, Error **errp);

/* The registers at base, the bank windows below every other region */
void ingenic_nemc_create(hwaddr base);

/* The bootrom, which bank 0's window repeats; call after the create */
void ingenic_nemc_set_bootrom(MemoryRegion *rom);

#endif /* HW_MISC_INGENIC_NEMC_H */
