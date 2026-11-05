/*
 * Ingenic static memory controller (NEMC; the T20, T21 and T30 vendor
 * trees call it NFI), the T10 to the T41 and the A1
 *
 * The register file as a T20X, a T30X and a T31X read it at their U-Boot
 * prompts, alike on all three, with nothing on the bus: the IP and its
 * clock gate open (CLKGR0 bit 0 clear) out of reset. 0x00 reads 1 and
 * bank 0's control at 0x10 reads 0x80, neither keeping a write; banks 1 to
 * 6 have a control word each from 0x14 (0x0fff7700, bits 29:6 and 2:0
 * keep a write) and banks 0 to 6 an address word each from 0x30 (base
 * 15:8, mask 7:0: bank 0 at 0x1c mask 0xfc, the boot ROM's bank, then
 * 0x1b down to 0x16, each mask 0xff). NFCSR at 0x50 keeps bits 11:0 (the
 * odd ones, chip enables, were set only on the T31X, whose bonded bank
 * pins were all GPIOs at the time). Everything else in the 64 KiB reads 0
 * and keeps nothing: there is no PN, toggle or BCH block here.
 *
 * The bank windows, 16 MiB each from 0x16000000 (CS6) to 0x1bffffff (CS1),
 * read 0 with no device on the bus; the model gives them that whatever
 * the address words hold, and drops writes, as no bus cycle reaches a
 * pin. Bank 0's window, where the boot ROM repeats every 16 KiB from
 * 0x1c000000, is the boot ROM's mapping, not this device's.
 *
 * Not measured, given the same file: the T10, T21, T23, T32, T33, T40, T41
 * and A1. Their kernels' headers place the six banks the same way; the T21
 * and X2000 manuals list only banks 1 and 2, with masks of 0xfe, which none
 * of the three parts read. Not modelled: bus timing, NOR or SRAM behind a
 * bank, parallel NAND, and the address words' decode.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-nemc.h"
#include "migration/vmstate.h"

#define NEMC_ID         0x00
#define NEMC_SMCR0      0x10
#define NEMC_SMCR1      0x14
#define NEMC_SMCR6      0x28
#define NEMC_SACR0      0x30
#define NEMC_SACR6      0x48
#define NEMC_NFCSR      0x50

#define SMCR0_VALUE     0x00000080
#define SMCR_RESET      0x0fff7700
#define SMCR_WMASK      0x3fffffc7
#define SACR_WMASK      0x0000ffff
#define NFCSR_WMASK     0x00000fff

static uint64_t ingenic_nemc_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicNemcState *s = INGENIC_NEMC(opaque);

    switch (offset) {
    case NEMC_ID:
        return 1;
    case NEMC_SMCR0:
        return SMCR0_VALUE;
    case NEMC_SMCR1 ... NEMC_SMCR6:
        return s->smcr[(offset - NEMC_SMCR1) / 4];
    case NEMC_SACR0 ... NEMC_SACR6:
        return s->sacr[(offset - NEMC_SACR0) / 4];
    case NEMC_NFCSR:
        return s->nfcsr;
    default:
        return 0;
    }
}

static void ingenic_nemc_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned size)
{
    IngenicNemcState *s = INGENIC_NEMC(opaque);

    switch (offset) {
    case NEMC_SMCR1 ... NEMC_SMCR6:
        s->smcr[(offset - NEMC_SMCR1) / 4] = value & SMCR_WMASK;
        break;
    case NEMC_SACR0 ... NEMC_SACR6:
        s->sacr[(offset - NEMC_SACR0) / 4] = value & SACR_WMASK;
        break;
    case NEMC_NFCSR:
        s->nfcsr = value & NFCSR_WMASK;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_nemc_ops = {
    .read = ingenic_nemc_read,
    .write = ingenic_nemc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t ingenic_nemc_bank_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    return 0;
}

static void ingenic_nemc_bank_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ingenic_nemc_bank_ops = {
    .read = ingenic_nemc_bank_read,
    .write = ingenic_nemc_bank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void ingenic_nemc_reset_hold(Object *obj, ResetType type)
{
    IngenicNemcState *s = INGENIC_NEMC(obj);

    for (int i = 0; i < INGENIC_NEMC_BANKS; i++) {
        s->smcr[i] = SMCR_RESET;
    }
    s->sacr[0] = 0x1cfc;
    for (int i = 1; i <= INGENIC_NEMC_BANKS; i++) {
        s->sacr[i] = ((0x1c - i) << 8) | 0xff;
    }
    s->nfcsr = 0;
}

static void ingenic_nemc_init(Object *obj)
{
    IngenicNemcState *s = INGENIC_NEMC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_nemc_ops, s,
                          TYPE_INGENIC_NEMC, 64 * KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->banks, obj, &ingenic_nemc_bank_ops, s,
                          TYPE_INGENIC_NEMC ".banks",
                          INGENIC_NEMC_BANKS * INGENIC_NEMC_BANK_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->banks);
}

static const VMStateDescription vmstate_ingenic_nemc = {
    .name = TYPE_INGENIC_NEMC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(smcr, IngenicNemcState, INGENIC_NEMC_BANKS),
        VMSTATE_UINT32_ARRAY(sacr, IngenicNemcState, INGENIC_NEMC_BANKS + 1),
        VMSTATE_UINT32(nfcsr, IngenicNemcState),
        VMSTATE_END_OF_LIST()
    },
};

static void ingenic_nemc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_ingenic_nemc;
    rc->phases.hold = ingenic_nemc_reset_hold;
}

static const TypeInfo ingenic_nemc_info = {
    .name = TYPE_INGENIC_NEMC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicNemcState),
    .instance_init = ingenic_nemc_init,
    .class_init = ingenic_nemc_class_init,
};

void ingenic_nemc_create(hwaddr base)
{
    DeviceState *dev = qdev_new(TYPE_INGENIC_NEMC);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    /* under a region a board puts in a bank, as the T31's CS2 RAM */
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 1, INGENIC_NEMC_BANK_BASE,
                            -1);
}

static void ingenic_nemc_register_types(void)
{
    type_register_static(&ingenic_nemc_info);
}

type_init(ingenic_nemc_register_types)
