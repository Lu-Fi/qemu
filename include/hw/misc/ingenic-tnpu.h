/*
 * Ingenic TNPU: the T32's, T33's and T32Pro's neural engine's RISC-V core
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_TNPU_H
#define HW_MISC_INGENIC_TNPU_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/misc/ingenic-riscv-link.h"
#include "qom/object.h"

#define TYPE_INGENIC_TNPU "ingenic-tnpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicTnpuState, INGENIC_TNPU)

/* The bus requester ID the SoC's side gives the TNPU core's accesses */
#define INGENIC_TNPU_REQUESTER      0x544e

#define INGENIC_TNPU_MISC           19      /* 0x40 to 0x88 */

struct IngenicTnpuState {
    SysBusDevice parent_obj;

    MemoryRegion ccu;
    MemoryRegion dram;
    qemu_irq irq;
    Clock *clk;
    CharFrontend link;
    QEMUTimer *cmp_timer;
    bool pro;                   /* the T32Pro's layout */

    bool hello;
    IngenicRiscvMsg rx;
    uint32_t rx_len;
    uint32_t core_irqs;         /* the lines last sent to the core */

    uint32_t ccsr;              /* its writable bits */
    bool asleep;
    uint32_t crer, fromhost, tohost;
    uint64_t time_base;         /* the count at time_ns */
    int64_t time_ns;
    uint64_t time_cmp;
    uint32_t epc, index, soft_reset, lock, cscr, becr;
    uint32_t misc[INGENIC_TNPU_MISC];
};

#endif /* HW_MISC_INGENIC_TNPU_H */
