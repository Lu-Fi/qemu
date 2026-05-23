/*
 * Ingenic T31 DDR Controller and Innosilicon PHY emulation
 *
 * Provides the minimum register behavior needed for SPL DDR init:
 * PLL lock, training completion, write leveling, calibration done,
 * and LMR (Load Mode Register) completion.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
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
}

static const MemoryRegionOps ingenic_ddrc_ops = {
    .read = ingenic_ddrc_read,
    .write = ingenic_ddrc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
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
    /* T32/T33 DDRCAPB registers (at PHY_BASE + 0x1000).
     * The DWC uMCTL2 has many status/polling registers. Rather than
     * emulating each one, return sensible defaults for the key ones
     * and let the default path handle the rest via stored regs. */
    case 0x11BC: /* DFISTAT: DFI init complete */
        return 0x01;
    case 0x1004: /* STAT: operating mode = normal */
        return 0x01;
    case 0x1324: /* SWSTAT: sw_done_ack */
        return 0x01;
    /* Innosilicon PHY calibration status (T32/T33 training) */
    case 0x158:
        /* T32/T33 training completion status:
         * train_all_step_done (bit 7) = 1
         * train_step1_delay_done (bit 6) = 1
         * train_true_done (bit 0) = 1 */
        return 0xC1;
    case 0x174:
        /* T32/T33 InnoSilicon PHY training/calibration status:
         * wl_done_byte (bits 24:16) = 0x3   (write leveling done)
         * reg_wl_end (bit 11) = 1           (write leveling end)
         * calib_end (bit 10) = 1            (calibration end)
         * calib_done_byte (bits 8:0) = 0x3  (calibration done) */
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
        /* 16-bit DDR = 0x03 (2 bytes), 32-bit = 0x0F (4 bytes).
         * Return 0x03 as safe default - works for both widths since
         * 32-bit mode checks (val & 0xf) == 0xf with || timeout. */
        return 0x03;
    case 0x110: /* DDRP_INNOPHY_INIT_COMP */
        return 0x01;
    /* DDRC APB (A1: aliased from 0x13012000 into PHY region).
     * offset 0x04 = DWSTATUS: DFI init complete (bit 0) */
    case 0x04:
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
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* ---- Device lifecycle ---- */

static void ingenic_ddrc_reset_hold(Object *obj, ResetType type)
{
    IngenicDdrcState *s = INGENIC_DDRC(obj);

    memset(s->ddrc_regs, 0, sizeof(s->ddrc_regs));
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
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
}

static const VMStateDescription vmstate_ingenic_ddrc = {
    .name = "ingenic-ddrc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ddrc_regs, IngenicDdrcState,
                             INGENIC_DDRC_REGS),
        VMSTATE_UINT32_ARRAY(phy_regs, IngenicDdrcState,
                             INGENIC_T31_DDRPHY_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ddrc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_ddrc;
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
