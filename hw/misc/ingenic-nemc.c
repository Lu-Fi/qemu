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
 * and bank 0's from 0x1c000000 to 0x1fffffff, all decode through the
 * address words, as a T31X reads them (the same on the T20X and T30X for
 * the cases run there): a window byte address is in bank n when
 * (addr[31:24] & mask) == base, the base not masked, so a word whose base
 * has a bit set where its mask has none matches nothing, as does a mask of
 * 0 with a base other than 0, while base 0 with mask 0 matches every
 * window. Bank 0 is the bootrom, repeating every ROM size over its whole
 * window including the bootrom's own 0x1fc00000, which reads 0 once
 * bank 0 stops covering 0x1f; any other window reads 0 with no device on
 * the bus, and drops writes. Banks 1 to 6 have no device, so their words
 * decode nothing a read can see: bank 1 over bank 0's window leaves the
 * bootrom there, and over a window bank 0 has left reads 0.
 * A word of 0 in bank 1 to 6 (matching every window) leaves the register
 * file taking junk on every write after the first, on all three parts;
 * that is not modelled.
 *
 * NFCSR's bits are paired per bank, NFE the even and NFCE the odd one. A T31X
 * with CS2's pin, PA16, in its NEMC function read PxPIN low only with both
 * of bank 2's bits (3:2) set and high with either alone or with any other
 * bank's pair, and no bank window read changed with any enables set; the
 * pin level is not modelled. A parallel NAND chip (ingenic-nemc-nand) can
 * sit behind a bank and is selected by the same pair; it follows the
 * vendor and mainline drivers, not silicon.
 *
 * Not measured, given the same file: the T10, T21, T23, T32, T33, T40, T41
 * and A1. Their kernels' headers place the six banks the same way; the T21
 * and X2000 manuals list only banks 1 and 2, with masks of 0xfe, which none
 * of the three parts read. Not modelled: bus timing, and NOR or SRAM
 * behind a bank.
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
#include "system/address-spaces.h"

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
#define INGENIC_NEMC_SPAN   0x0a000000  /* CS6 to the end of bank 0 */

/* Bank n's window covers the 16 MiB window at phys, by the address word */
static bool nemc_covers(IngenicNemcState *s, int bank, uint32_t phys)
{
    uint32_t w = s->sacr[bank];

    return ((phys >> 24) & w & 0xff) == ((w >> 8) & 0xff);
}

/* A chip behind bank n is selected when its NFE and NFCE are both set */
static IngenicNemcNandState *nemc_nand_at(IngenicNemcState *s, uint32_t phys)
{
    for (int n = 1; n <= INGENIC_NEMC_BANKS; n++) {
        if (s->nand[n] && nemc_covers(s, n, phys) &&
            extract32(s->nfcsr, 2 * (n - 1), 2) == 3) {
            return s->nand[n];
        }
    }
    return NULL;
}

static void nemc_sync_gate(IngenicNemcState *s)
{
    if (s->bootrom) {
        memory_region_set_enabled(&s->gate, !nemc_covers(s, 0, 0x1f000000));
    }
}

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
        nemc_sync_gate(s);
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
    IngenicNemcState *s = INGENIC_NEMC(opaque);
    uint32_t phys = INGENIC_NEMC_BANK_BASE + offset;
    IngenicNemcNandState *nand = nemc_nand_at(s, phys);

    if (nand) {
        uint64_t val = 0;

        /* Command and address latches read nothing; the rest is data */
        if (extract32(offset, 22, 2) == 0 || extract32(offset, 22, 2) == 3) {
            for (unsigned i = 0; i < size; i++) {
                val |= (uint64_t)ingenic_nemc_nand_read(nand) << (8 * i);
            }
        }
        return val;
    }
    if (s->bootrom && nemc_covers(s, 0, phys)) {
        uint64_t rom = memory_region_size(s->bootrom);

        return ldn_le_p((uint8_t *)memory_region_get_ram_ptr(s->bootrom) +
                        (phys & (rom - 1)), size);
    }
    return 0;
}

static void ingenic_nemc_bank_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    IngenicNemcState *s = INGENIC_NEMC(opaque);
    IngenicNemcNandState *nand =
        nemc_nand_at(s, INGENIC_NEMC_BANK_BASE + offset);

    if (!nand) {
        return;
    }
    switch (extract32(offset, 22, 2)) {
    case 1:
        ingenic_nemc_nand_cmd(nand, value);
        break;
    case 2:
        ingenic_nemc_nand_addr(nand, value);
        break;
    default:
        for (unsigned i = 0; i < size; i++) {
            ingenic_nemc_nand_write(nand, value >> (8 * i));
        }
        break;
    }
}

static const MemoryRegionOps ingenic_nemc_bank_ops = {
    .read = ingenic_nemc_bank_read,
    .write = ingenic_nemc_bank_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t ingenic_nemc_gate_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    return 0;
}

static void ingenic_nemc_gate_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ingenic_nemc_gate_ops = {
    .read = ingenic_nemc_gate_read,
    .write = ingenic_nemc_gate_write,
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
    nemc_sync_gate(s);
}

static void ingenic_nemc_init(Object *obj)
{
    IngenicNemcState *s = INGENIC_NEMC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_nemc_ops, s,
                          TYPE_INGENIC_NEMC, 64 * KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->banks, obj, &ingenic_nemc_bank_ops, s,
                          TYPE_INGENIC_NEMC ".banks", INGENIC_NEMC_SPAN);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->banks);
    memory_region_init_io(&s->gate, obj, &ingenic_nemc_gate_ops, s,
                          TYPE_INGENIC_NEMC ".gate", 16 * KiB);
}

static int ingenic_nemc_post_load(void *opaque, int version_id)
{
    nemc_sync_gate(INGENIC_NEMC(opaque));
    return 0;
}

static const VMStateDescription vmstate_ingenic_nemc = {
    .name = TYPE_INGENIC_NEMC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_nemc_post_load,
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

bool ingenic_nemc_attach_nand(IngenicNemcNandState *n, int bank, Error **errp)
{
    IngenicNemcState *s = INGENIC_NEMC(
        object_resolve_type_unambiguous(TYPE_INGENIC_NEMC, errp));

    if (!s) {
        return false;
    }
    if (bank < 1 || bank > INGENIC_NEMC_BANKS || s->nand[bank]) {
        error_setg(errp, "NEMC bank %d is not a free bank 1 to 6", bank);
        return false;
    }
    s->nand[bank] = n;
    return true;
}

void ingenic_nemc_set_bootrom(MemoryRegion *rom)
{
    IngenicNemcState *s = INGENIC_NEMC(
        object_resolve_type_unambiguous(TYPE_INGENIC_NEMC, &error_abort));

    s->bootrom = rom;
    /* Over the bootrom proper, which a window not covering 0x1f hides */
    memory_region_set_size(&s->gate, memory_region_size(rom));
    memory_region_add_subregion_overlap(get_system_memory(), 0x1fc00000,
                                        &s->gate, 1);
    nemc_sync_gate(s);
}

static void ingenic_nemc_register_types(void)
{
    type_register_static(&ingenic_nemc_info);
}

type_init(ingenic_nemc_register_types)
