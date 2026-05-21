/*
 * Ingenic XBurst2 CCU (Core Control Unit) emulation
 *
 * The CCU is the memory-mapped SMP control block shared by the Ingenic
 * XBurst2 SoCs (A1, T40, T41). It holds per-core software reset, reports
 * per-core sleep state, provides the inter-processor mailbox and masks
 * interrupts per core. See the XBurst2 CPU Core Programming Manual,
 * chapter 8. All registers live in uncached kseg1 space.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_INTC_INGENIC_XBURST2_CCU_H
#define HW_INTC_INGENIC_XBURST2_CCU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_XBURST2_CCU "ingenic-xburst2-ccu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicXBurst2CcuState, INGENIC_XBURST2_CCU)

/* The XBurst2 family scales to 4 physical cores; our SoCs use 2. */
#define INGENIC_XBURST2_CCU_MAX_CORES   4
#define INGENIC_XBURST2_CCU_IOSIZE      0x2000

struct IngenicXBurst2CcuState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    /* Number of logic cores, set by the SoC via the "num-cpus" property. */
    uint32_t num_cpus;

    /* Cores resolved at realize time, indexed by CPU number. */
    CPUState *cpu[INGENIC_XBURST2_CCU_MAX_CORES];

    /* Per-core interrupt outputs to each core's MIPS IP2/IP3/IP4. */
    qemu_irq periph_irq[INGENIC_XBURST2_CCU_MAX_CORES];
    qemu_irq mailbox_irq[INGENIC_XBURST2_CCU_MAX_CORES];
    qemu_irq ost_irq[INGENIC_XBURST2_CCU_MAX_CORES];

    /* Raw interrupt input levels: the INTC line and the per-core OST. */
    bool intc_level;
    bool ost_level[INGENIC_XBURST2_CCU_MAX_CORES];

    /*
     * CCU registers - XBurst2 Core PM chapter 8. CSSR is computed on read
     * from live core state, and CCR/MSIR are constants, so none of those
     * three is stored here.
     */
    uint32_t cscr;      /* 0x000  core sleep control */
    uint32_t csrr;      /* 0x040  core software reset */
    uint32_t mscr;      /* 0x060  memory subsystem control */
    uint32_t pimr;      /* 0x120  peripheral IRQ mask */
    uint32_t mipr;      /* 0x140  mailbox IRQ pending */
    uint32_t mimr;      /* 0x160  mailbox IRQ mask */
    uint32_t oimr;      /* 0x1a0  OST IRQ mask */
    uint32_t dimr;      /* 0x1e0  debug IRQ mask */
    uint32_t rer;       /* 0xf00  reset entry */
    uint32_t cslr;      /* 0xfa0  spin lock */
    uint32_t csar;      /* 0xfa4  spin atomic */
    uint32_t gimr;      /* 0xfc0  global IRQ mask */
    uint32_t cfcr;      /* 0xfe0  CPU feature config */
    uint32_t bcer;      /* 0x1f00 bus exception control */
    uint32_t mbr[INGENIC_XBURST2_CCU_MAX_CORES]; /* 0x1000+N*4 mailbox */
};

#endif /* HW_INTC_INGENIC_XBURST2_CCU_H */
