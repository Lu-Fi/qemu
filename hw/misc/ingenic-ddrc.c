/*
 * Ingenic T31 DDR Controller and Innosilicon PHY emulation
 *
 * Provides the minimum register behavior needed for SPL DDR init:
 * PLL lock, training completion, write leveling, calibration done,
 * and LMR (Load Mode Register) completion.
 *
 * Two more windows are as measured on silicon, at each part's bootrom and
 * again at U-Boot: the page at 0x13010000 the SoC headers call DDRC1,
 * which reads zero and keeps nothing on every part (it is no second
 * controller), and the APB page at 0x13012000 (the T40, T41 and A1's
 * DDRC APB, the inno PHY's init handshake on T21 to T31), 64 words
 * repeating every 0x100. Each word there keeps some bits, reads others as
 * the inverse of the bit below them, and holds the rest read-only, at one
 * value until the SPL sets up DDR (its first write to the controller) and
 * another after.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-ddrc.h"

/* DDRC register offsets (from AHB_BASE 0x134F0000) */
#define DDRC_STATUS     0x00
#define DDRC_CFG        0x04
#define DDRC_CTRL       0x08
#define DDRC_LMR        0x0C

/* Synopsys DWC DDR PHY register offsets (T10/T20/T21/T23/T30) */
#define DWC_PHY_PIR         0x04
#define DWC_PHY_PGCR        0x08
#define DWC_PHY_PGSR        0x0C

/* Innosilicon DDR PHY register offsets (T31) */
#define PHY_WL_DONE         0xC0
#define PHY_PLL_LOCK        0xC8
#define PHY_CALIB_DONE      0xCC
#define PHY_INIT_COMP       0xD0

/* APB PHY registers (at APB_BASE 0x13012000, offset 0x1000 from PHY_BASE) */
#define APB_PHY_INIT        0x108C
/* DQS delay registers */
#define PHY_DQS_DELAY_L     0x190
#define PHY_DQS_DELAY_H     0x194

#define PHY_REG_INDEX(offset)   ((offset) / sizeof(uint32_t))
#define DDRC_REG_INDEX(offset)  ((offset) / sizeof(uint32_t))

/* ---- DDRC ---- */

static uint64_t ingenic_ddrc_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicDdrcState *s = INGENIC_DDRC(opaque);

    if (offset >= INGENIC_DDRC_IOSIZE) {
        return 0;
    }

    switch (offset) {
    case DDRC_LMR:
        return 0;
    default:
        return s->ddrc_regs[DDRC_REG_INDEX(offset)];
    }
}

static void ingenic_ddrc_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicDdrcState *s = INGENIC_DDRC(opaque);

    if (offset >= INGENIC_DDRC_IOSIZE) {
        return;
    }

    s->ddrc_regs[DDRC_REG_INDEX(offset)] = (uint32_t)value;
    s->ddr_up = true;
}

static const MemoryRegionOps ingenic_ddrc_ops = {
    .read = ingenic_ddrc_read,
    .write = ingenic_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- DDR PHY ---- */

static uint64_t ingenic_t31_ddrphy_read(void *opaque, hwaddr offset,
                                        unsigned size)
{
    IngenicDdrcState *s = INGENIC_DDRC(opaque);

    if (offset >= INGENIC_T31_DDRPHY_IOSIZE) {
        return 0;
    }

    switch (offset) {
    /* DWC PHY (T10/T20/T21/T23/T30): PGSR reports all-done */
    case DWC_PHY_PGSR:
        /* IDONE|DLDONE|ZCDONE|DIDONE|DTDONE = 0x1F */
        return 0x1F;
    /*
     * T32/T33 DDRCAPB registers (at PHY_BASE + 0x1000).
     * The DWC uMCTL2 has many status/polling registers. Rather than
     * emulating each one, return sensible defaults for the key ones
     * and let the default path handle the rest via stored regs.
     */
    case 0x11BC: /* DFISTAT: DFI init complete */
        return 0x01;
    case 0x1004: /* STAT: operating mode = normal */
        return 0x01;
    case 0x1324: /* SWSTAT: sw_done_ack */
        return 0x01;
    /* Innosilicon PHY calibration status (T32/T33 training) */
    case 0x158:
        /*
         * T32/T33 training completion status:
         * train_all_step_done (bit 7) = 1
         * train_step1_delay_done (bit 6) = 1
         * train_true_done (bit 0) = 1
         */
        return 0xC1;
    case 0x174:
        /*
         * T32/T33 InnoSilicon PHY training/calibration status:
         * wl_done_byte (bits 24:16) = 0x3   (write leveling done)
         * reg_wl_end (bit 11) = 1           (write leveling end)
         * calib_end (bit 10) = 1            (calibration end)
         * calib_done_byte (bits 8:0) = 0x3  (calibration done)
         */
        return 0x30C03;
    /* Innosilicon PHY (T31) */
    case PHY_PLL_LOCK:
        return 0x08;
    case PHY_WL_DONE:
        return 0x03;
    case PHY_CALIB_DONE:
        return 0x03;
    case PHY_INIT_COMP:
        return 0x01;
    case APB_PHY_INIT:
        return 0x07;
    /* Innosilicon PHY (T40/T41 XBurst2, different offsets from A1) */
    case 0x108: /* DDRP_INNOPHY_PLL_LOCK (T40/T41) */
        return 0x08;
    case 0x10c: /* DDRP_INNOPHY_CALIB_DONE (T40/T41) */
        return 0x0F;
    /* Innosilicon PHY (A1 XBurst2) */
    case 0x180: /* DDRP_INNOPHY_PLL_LOCK */
        return 0x07;
    case 0x184: /* DDRP_INNOPHY_CALIB_DONE */
        /*
         * 16-bit DDR = 0x03 (2 bytes), 32-bit = 0x0F (4 bytes).
         * Return 0x03 as safe default - works for both widths since
         * 32-bit mode checks (val & 0xf) == 0xf with || timeout.
         */
        return 0x03;
    case 0x110: /* DDRP_INNOPHY_INIT_COMP */
        return 0x01;
    case PHY_DQS_DELAY_L:
    case PHY_DQS_DELAY_H:
        return 0x60;
    default:
        if (offset < INGENIC_T31_DDRPHY_IOSIZE) {
            return s->phy_regs[PHY_REG_INDEX(offset)];
        }
        return 0;
    }
}

static void ingenic_t31_ddrphy_write(void *opaque, hwaddr offset,
                                     uint64_t value, unsigned size)
{
    IngenicDdrcState *s = INGENIC_DDRC(opaque);

    if (offset < INGENIC_T31_DDRPHY_IOSIZE) {
        s->phy_regs[PHY_REG_INDEX(offset)] = (uint32_t)value;
    }
}

static const MemoryRegionOps ingenic_t31_ddrphy_ops = {
    .read = ingenic_t31_ddrphy_read,
    .write = ingenic_t31_ddrphy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- DDR APB page ---- */

typedef struct {
    uint8_t first, last;        /* word offsets, inclusive */
    uint32_t rw;                /* bits that keep a write */
    uint32_t inv;               /* bits reading the inverse of the next lower */
    uint32_t clr;               /* bits set at reset that any write clears */
} DdrApbMask;

typedef struct {
    uint8_t first, last;
    uint32_t reset;
    uint32_t post;              /* the read-only bits once DDR is up */
} DdrApbValue;

static const DdrApbMask ddr_apb_dwc[] = {
    { 0x00, 0x00, 0x00000001, 0x00000000, 0x00000000 },
    { 0x20, 0x20, 0x000000ff, 0x00000000, 0x00000000 },
    { 0x28, 0x28, 0x000003ff, 0x00000000, 0x00000000 },
    { 0x34, 0x34, 0x00001113, 0x00002200, 0x00000000 },
    { 0x38, 0x38, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x3c, 0x3c, 0x007fffff, 0x00000000, 0x00000000 },
    { 0x40, 0x40, 0x0000ffff, 0x00000000, 0x00000000 },
    { 0x44, 0x44, 0x0000003f, 0x00000000, 0x00000000 },
    { 0x68, 0x68, 0xf0000fff, 0x00000000, 0x00000000 },
    { 0x80, 0x80, 0x00f00001, 0x00000000, 0x00000000 },
    { 0x84, 0x84, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x88, 0x88, 0x00073f3f, 0x00000000, 0x00000000 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000, 0x00000000 },
};

static const DdrApbMask ddr_apb_inno[] = {
    { 0x00, 0x00, 0x00000001, 0x00000000, 0x00000000 },
    { 0x20, 0x20, 0x000000ff, 0x00000000, 0x00000000 },
    { 0x28, 0x28, 0x000003ff, 0x00000000, 0x00000000 },
    { 0x34, 0x34, 0x00001113, 0x00002200, 0x00000000 },
    { 0x38, 0x38, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x3c, 0x3c, 0x007fffff, 0x00000000, 0x00000000 },
    { 0x40, 0x40, 0x0000ffff, 0x00000000, 0x00000000 },
    { 0x44, 0x44, 0x0000003f, 0x00000000, 0x00000000 },
    { 0x68, 0x68, 0xf0000fff, 0x00000000, 0x00000000 },
    { 0x80, 0x80, 0x00f00001, 0x00000000, 0x00000000 },
    { 0x84, 0x84, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x88, 0x88, 0x00073f3f, 0x00000000, 0x00000000 },
    { 0x8c, 0x8c, 0x00000001, 0x00000002, 0x00000000 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000, 0x00000000 },
};

static const DdrApbMask ddr_apb_xb2[] = {
    { 0x00, 0x00, 0x0000001f, 0x00000000, 0x00000000 },
    { 0x08, 0x18, 0x1f1f1f1f, 0x00000000, 0x00000000 },
    { 0x1c, 0x1c, 0x00001fff, 0x00000000, 0x00000000 },
    { 0x20, 0x48, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x4c, 0x4c, 0x003f003f, 0x00000000, 0x00000000 },
    { 0x50, 0x50, 0x0000fff1, 0x00000000, 0x00000000 },
    { 0x54, 0x58, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x5c, 0x5c, 0x0000000f, 0x00000000, 0x00000000 },
    { 0x60, 0x64, 0xffffffff, 0x00000000, 0x00000000 },
    { 0x68, 0x68, 0xff8001ff, 0x00000000, 0x00000000 },
    { 0x6c, 0x6c, 0x00000001, 0x00000000, 0x00000002 },
};

static const DdrApbValue ddr_apb_t10_values[] = {
    { 0x04, 0x04, 0x0000020e, 0x0000005e },
    { 0x14, 0x14, 0x00000246, 0x00000246 },
    { 0x18, 0x18, 0x00000002, 0x00000001 },
    { 0x1c, 0x1c, 0x00000098, 0x0000015b },
    { 0x20, 0x20, 0x00000001, 0x00000000 },
    { 0x24, 0x24, 0x00000001, 0x00000001 },
    { 0x28, 0x28, 0x00000007, 0x00000000 },
    { 0x2c, 0x30, 0x00000007, 0x00000007 },
    { 0x34, 0x34, 0x00002202, 0x00000000 },
    { 0x38, 0x38, 0x00001870, 0x00000000 },
    { 0x44, 0x44, 0x00000001, 0x00000000 },
    { 0x48, 0x4c, 0x00000001, 0x00000001 },
    { 0x68, 0x68, 0x00000020, 0x00000000 },
    { 0x6c, 0x6c, 0x0000cff0, 0x0000cff0 },
    { 0x84, 0x84, 0x04000400, 0x00000000 },
    { 0x88, 0x88, 0x00000510, 0x00000000 },
    { 0x8c, 0x8c, 0x00000510, 0x00000510 },
    { 0x90, 0xf0, 0x00304600, 0x80300f00 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000 },
};

static const DdrApbValue ddr_apb_t20_values[] = {
    { 0x04, 0x04, 0x000001ab, 0x000001ab },
    { 0x14, 0x14, 0x0000020e, 0x0000020e },
    { 0x18, 0x18, 0x0000009a, 0x00001012 },
    { 0x1c, 0x1c, 0x00000112, 0x00000012 },
    { 0x20, 0x20, 0x00000001, 0x00000000 },
    { 0x24, 0x24, 0x00000001, 0x00000001 },
    { 0x28, 0x28, 0x00000007, 0x00000000 },
    { 0x2c, 0x30, 0x00000007, 0x00000007 },
    { 0x34, 0x34, 0x00002202, 0x00000000 },
    { 0x38, 0x38, 0x00001870, 0x00000000 },
    { 0x44, 0x44, 0x00000001, 0x00000000 },
    { 0x48, 0x4c, 0x00000001, 0x00000001 },
    { 0x68, 0x68, 0x00000020, 0x00000000 },
    { 0x6c, 0x6c, 0x00004ff0, 0x0000cff0 },
    { 0x84, 0x84, 0x04000400, 0x00000000 },
    { 0x88, 0x88, 0x00000510, 0x00000000 },
    { 0x8c, 0x8c, 0x00000510, 0x00000510 },
    { 0x90, 0xf0, 0x00000000, 0x42000000 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000 },
};

static const DdrApbValue ddr_apb_t21_values[] = {
    { 0x04, 0x04, 0x00000053, 0x00000053 },
    { 0x10, 0x10, 0x0000022a, 0x0000000a },
    { 0x18, 0x18, 0x00000087, 0x00000086 },
    { 0x1c, 0x1c, 0x00000083, 0x00000083 },
    { 0x20, 0x20, 0x00000001, 0x00000000 },
    { 0x24, 0x24, 0x00000001, 0x00000001 },
    { 0x28, 0x28, 0x00000007, 0x00000000 },
    { 0x2c, 0x30, 0x00000007, 0x00000007 },
    { 0x34, 0x34, 0x00002202, 0x00000000 },
    { 0x38, 0x38, 0x00001870, 0x00000000 },
    { 0x44, 0x44, 0x00000001, 0x00000000 },
    { 0x48, 0x4c, 0x00000001, 0x00000001 },
    { 0x68, 0x68, 0x00000020, 0x00000000 },
    { 0x6c, 0x6c, 0x00004ff0, 0x00008ff0 },
    { 0x84, 0x84, 0x04000400, 0x00000000 },
    { 0x88, 0x88, 0x00000510, 0x00000000 },
    { 0x8c, 0x8c, 0x00000001, 0x00000004 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000 },
};

static const DdrApbValue ddr_apb_t23_values[] = {
    { 0x04, 0x04, 0x0000020a, 0x0000020a },
    { 0x10, 0x10, 0x00000003, 0x00000003 },
    { 0x14, 0x14, 0x000001ca, 0x000001cb },
    { 0x18, 0x18, 0x000000c3, 0x000000c3 },
    { 0x1c, 0x1c, 0x00000001, 0x00000001 },
    { 0x20, 0x20, 0x00000001, 0x00000000 },
    { 0x24, 0x24, 0x00000001, 0x00000001 },
    { 0x28, 0x28, 0x00000007, 0x00000000 },
    { 0x2c, 0x30, 0x00000007, 0x00000007 },
    { 0x34, 0x34, 0x00002202, 0x00000000 },
    { 0x38, 0x38, 0x00001870, 0x00000000 },
    { 0x44, 0x44, 0x00000001, 0x00000000 },
    { 0x48, 0x4c, 0x00000001, 0x00000001 },
    { 0x68, 0x68, 0x00000020, 0x00000000 },
    { 0x6c, 0x6c, 0x00004ff0, 0x00008ff0 },
    { 0x84, 0x84, 0x04000400, 0x00000000 },
    { 0x88, 0x88, 0x00000510, 0x00000000 },
    { 0x8c, 0x8c, 0x00000001, 0x00000004 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000 },
};

static const DdrApbValue ddr_apb_t30_values[] = {
    { 0x04, 0x04, 0x000002ce, 0x000002ce },
    { 0x10, 0x10, 0x00000280, 0x0000000e },
    { 0x14, 0x14, 0x00000026, 0x000000ae },
    { 0x18, 0x18, 0x00001001, 0x00001001 },
    { 0x1c, 0x1c, 0xfffffcce, 0xfffffcce },
    { 0x20, 0x20, 0x00000001, 0x00000000 },
    { 0x24, 0x24, 0x00000001, 0x00000001 },
    { 0x28, 0x28, 0x00000007, 0x00000000 },
    { 0x2c, 0x30, 0x00000007, 0x00000007 },
    { 0x34, 0x34, 0x00002202, 0x00000000 },
    { 0x38, 0x38, 0x00001870, 0x00000000 },
    { 0x44, 0x44, 0x00000001, 0x00000000 },
    { 0x48, 0x4c, 0x00000001, 0x00000001 },
    { 0x68, 0x68, 0x00000020, 0x00000000 },
    { 0x6c, 0x6c, 0x00004ff0, 0x00008ff0 },
    { 0x84, 0x84, 0x04000400, 0x00000000 },
    { 0x88, 0x88, 0x00000510, 0x00000000 },
    { 0x8c, 0x8c, 0x00000001, 0x00000004 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000 },
};

static const DdrApbValue ddr_apb_t31_values[] = {
    { 0x04, 0x04, 0x000003c7, 0x000003c5 },
    { 0x10, 0x10, 0x00000180, 0x00000222 },
    { 0x14, 0x14, 0x00000076, 0x00000076 },
    { 0x18, 0x18, 0x00001000, 0x00001000 },
    { 0x1c, 0x1c, 0x00000344, 0x00000364 },
    { 0x20, 0x20, 0x00000001, 0x00000000 },
    { 0x24, 0x24, 0x00000001, 0x00000001 },
    { 0x28, 0x28, 0x00000007, 0x00000000 },
    { 0x2c, 0x30, 0x00000007, 0x00000007 },
    { 0x34, 0x34, 0x00002202, 0x00000000 },
    { 0x38, 0x38, 0x00001870, 0x00000000 },
    { 0x44, 0x44, 0x00000001, 0x00000000 },
    { 0x48, 0x4c, 0x00000001, 0x00000001 },
    { 0x68, 0x68, 0x00000020, 0x00000000 },
    { 0x6c, 0x6c, 0x00004ff0, 0x00008ff0 },
    { 0x84, 0x84, 0x04000400, 0x00000000 },
    { 0x88, 0x88, 0x00000510, 0x00000000 },
    { 0x8c, 0x8c, 0x00000001, 0x00000004 },
    { 0xf4, 0xf4, 0x000000ff, 0x00000000 },
    { 0xf8, 0xf8, 0x0000ffff, 0x00000000 },
    { 0xfc, 0xfc, 0xffffffff, 0x00000000 },
};

static const DdrApbValue ddr_apb_t40_values[] = {
    { 0x00, 0x00, 0x00000009, 0x00000000 },
    { 0x04, 0x04, 0x00000000, 0x00000003 },
    { 0x08, 0x08, 0x03020100, 0x00000000 },
    { 0x0c, 0x0c, 0x07060504, 0x00000000 },
    { 0x10, 0x10, 0x0b0a0908, 0x00000000 },
    { 0x14, 0x14, 0x0f0e0d0c, 0x00000000 },
    { 0x18, 0x18, 0x13121110, 0x00000000 },
    { 0x1c, 0x1c, 0x00001550, 0x00000000 },
    { 0x20, 0x3c, 0x88404000, 0x00000000 },
    { 0x40, 0x40, 0x40000000, 0x00000000 },
    { 0x44, 0x44, 0xfef9f9e4, 0x00000000 },
    { 0x48, 0x48, 0x29107460, 0x00000000 },
    { 0x4c, 0x4c, 0x00100020, 0x00000000 },
    { 0x50, 0x50, 0x0000cc82, 0x00000002 },
    { 0x68, 0x68, 0x00800000, 0x00000000 },
    { 0x6c, 0x6c, 0x00000002, 0x00000000 },
};

static const DdrApbValue ddr_apb_t41_values[] = {
    { 0x00, 0x00, 0x00000008, 0x00000000 },
    { 0x04, 0x04, 0x00000000, 0x00000003 },
    { 0x08, 0x08, 0x03020100, 0x00000000 },
    { 0x0c, 0x0c, 0x07060504, 0x00000000 },
    { 0x10, 0x10, 0x0b0a0908, 0x00000000 },
    { 0x14, 0x14, 0x0f0e0d0c, 0x00000000 },
    { 0x18, 0x18, 0x13121110, 0x00000000 },
    { 0x1c, 0x1c, 0x00001550, 0x00000000 },
    { 0x20, 0x3c, 0x88404000, 0x00000000 },
    { 0x40, 0x40, 0x40000000, 0x00000000 },
    { 0x44, 0x44, 0xfef9f9e4, 0x00000000 },
    { 0x48, 0x48, 0x29107460, 0x00000000 },
    { 0x4c, 0x4c, 0x00100020, 0x00000000 },
    { 0x50, 0x50, 0x0000cc82, 0x00000002 },
    { 0x68, 0x68, 0x00800000, 0x00000000 },
    { 0x6c, 0x6c, 0x00000002, 0x00000000 },
};

static const DdrApbValue ddr_apb_a1_values[] = {
    { 0x00, 0x00, 0x00000009, 0x00000000 },
    { 0x04, 0x04, 0x00000000, 0x00000003 },
    { 0x08, 0x08, 0x03020100, 0x00000000 },
    { 0x0c, 0x0c, 0x07060504, 0x00000000 },
    { 0x10, 0x10, 0x0b0a0908, 0x00000000 },
    { 0x14, 0x14, 0x0f0e0d0c, 0x00000000 },
    { 0x18, 0x18, 0x13121110, 0x00000000 },
    { 0x1c, 0x1c, 0x00001550, 0x00000000 },
    { 0x20, 0x3c, 0x88404000, 0x00000000 },
    { 0x40, 0x40, 0x40000000, 0x00000000 },
    { 0x44, 0x44, 0xfef9f9e4, 0x00000000 },
    { 0x48, 0x48, 0x29107460, 0x00000000 },
    { 0x4c, 0x4c, 0x00100020, 0x00000000 },
    { 0x50, 0x50, 0x0000cc82, 0x00000002 },
    { 0x68, 0x68, 0x00800000, 0x00000000 },
    { 0x6c, 0x6c, 0x00000002, 0x00000000 },
};

typedef struct DdrApbLayout {
    const char *name;
    const DdrApbMask *masks;
    size_t nmasks;
    const DdrApbValue *values;
    size_t nvalues;
} DdrApbLayout;

#define DDR_APB(n, m) \
    { #n, m, ARRAY_SIZE(m), ddr_apb_##n##_values, \
      ARRAY_SIZE(ddr_apb_##n##_values) }

static const DdrApbLayout ddr_apb_layouts[] = {
    DDR_APB(t10, ddr_apb_dwc), DDR_APB(t20, ddr_apb_dwc),
    DDR_APB(t21, ddr_apb_inno), DDR_APB(t23, ddr_apb_inno),
    DDR_APB(t30, ddr_apb_inno), DDR_APB(t31, ddr_apb_inno),
    DDR_APB(t40, ddr_apb_xb2), DDR_APB(t41, ddr_apb_xb2),
    DDR_APB(a1, ddr_apb_xb2),
};

static void ddr_apb_word(IngenicDdrcState *s, unsigned off, DdrApbMask *m,
                         DdrApbValue *v)
{
    const DdrApbLayout *l = s->apb;

    *m = (DdrApbMask) { 0 };
    *v = (DdrApbValue) { 0 };
    for (size_t i = 0; i < l->nmasks; i++) {
        if (off >= l->masks[i].first && off <= l->masks[i].last) {
            *m = l->masks[i];
        }
    }
    for (size_t i = 0; i < l->nvalues; i++) {
        if (off >= l->values[i].first && off <= l->values[i].last) {
            *v = l->values[i];
        }
    }
}

static uint64_t ingenic_ddr_apb_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicDdrcState *s = INGENIC_DDRC(opaque);
    unsigned n = (offset & 0xff) >> 2;
    DdrApbMask m;
    DdrApbValue v;
    uint32_t w = s->apb_regs[n];
    uint32_t ro;

    ddr_apb_word(s, n << 2, &m, &v);
    w &= m.rw;
    ro = s->ddr_up ? v.post : v.reset & ~(m.rw | m.inv | m.clr);
    return w | ro | (m.inv & ~(w << 1)) |
           (s->apb_written & BIT_ULL(n) ? 0 : v.reset & m.clr);
}

static void ingenic_ddr_apb_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicDdrcState *s = INGENIC_DDRC(opaque);
    unsigned n = (offset & 0xff) >> 2;

    s->apb_regs[n] = value;
    s->apb_written |= BIT_ULL(n);
}

static const MemoryRegionOps ingenic_ddr_apb_ops = {
    .read = ingenic_ddr_apb_read,
    .write = ingenic_ddr_apb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t ingenic_ddrc1_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    return 0;
}

static void ingenic_ddrc1_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ingenic_ddrc1_ops = {
    .read = ingenic_ddrc1_read,
    .write = ingenic_ddrc1_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---- Device lifecycle ---- */

static void ingenic_ddrc_reset_hold(Object *obj, ResetType type)
{
    IngenicDdrcState *s = INGENIC_DDRC(obj);

    memset(s->ddrc_regs, 0, sizeof(s->ddrc_regs));
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    s->ddr_up = false;
    s->apb_written = 0;
    for (unsigned n = 0; n < INGENIC_DDR_APB_REGS && s->apb; n++) {
        DdrApbMask m;
        DdrApbValue v;

        ddr_apb_word(s, n << 2, &m, &v);
        s->apb_regs[n] = v.reset;
    }
}

static void ingenic_ddrc_realize(DeviceState *dev, Error **errp)
{
    IngenicDdrcState *s = INGENIC_DDRC(dev);

    if (!s->apb_layout) {
        return;
    }
    for (size_t i = 0; i < ARRAY_SIZE(ddr_apb_layouts); i++) {
        if (!strcmp(s->apb_layout, ddr_apb_layouts[i].name)) {
            s->apb = &ddr_apb_layouts[i];
        }
    }
    if (!s->apb) {
        error_setg(errp, "ingenic-ddrc: no DDR APB layout '%s'",
                   s->apb_layout);
    }
}

static void ingenic_ddrc_init(Object *obj)
{
    IngenicDdrcState *s = INGENIC_DDRC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->ddrc_iomem, obj, &ingenic_ddrc_ops, s,
                          "ingenic-ddrc", INGENIC_DDRC_IOSIZE);
    sysbus_init_mmio(sbd, &s->ddrc_iomem);

    memory_region_init_io(&s->phy_iomem, obj, &ingenic_t31_ddrphy_ops, s,
                          "ingenic-t31-ddrphy", INGENIC_T31_DDRPHY_IOSIZE);
    sysbus_init_mmio(sbd, &s->phy_iomem);

    memory_region_init_io(&s->apb_iomem, obj, &ingenic_ddr_apb_ops, s,
                          "ingenic-ddr-apb", 0x1000);
    sysbus_init_mmio(sbd, &s->apb_iomem);

    /* DDRC1: reads zero, keeps nothing */
    memory_region_init_io(&s->ddrc1_iomem, obj, &ingenic_ddrc1_ops, s,
                          "ingenic-ddrc1", 0x1000);
    sysbus_init_mmio(sbd, &s->ddrc1_iomem);
}

static const VMStateDescription vmstate_ingenic_ddrc = {
    .name = "ingenic-ddrc",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ddrc_regs, IngenicDdrcState,
                             INGENIC_DDRC_REGS),
        VMSTATE_UINT32_ARRAY(phy_regs, IngenicDdrcState,
                             INGENIC_T31_DDRPHY_REGS),
        VMSTATE_UINT32_ARRAY(apb_regs, IngenicDdrcState,
                             INGENIC_DDR_APB_REGS),
        VMSTATE_UINT64(apb_written, IngenicDdrcState),
        VMSTATE_BOOL(ddr_up, IngenicDdrcState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_ddrc_props[] = {
    /* the DDR APB page's layout: t10 to t31, t40, t41 or a1 */
    DEFINE_PROP_STRING("apb", IngenicDdrcState, apb_layout),
};

static void ingenic_ddrc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_ddrc;
    dc->realize = ingenic_ddrc_realize;
    device_class_set_props(dc, ingenic_ddrc_props);
    rc->phases.hold = ingenic_ddrc_reset_hold;
}

static const TypeInfo ingenic_ddrc_type_info = {
    .name = TYPE_INGENIC_DDRC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDdrcState),
    .instance_init = ingenic_ddrc_init,
    .class_init = ingenic_ddrc_class_init,
};

static void ingenic_ddrc_register_types(void)
{
    type_register_static(&ingenic_ddrc_type_info);
}

type_init(ingenic_ddrc_register_types)
