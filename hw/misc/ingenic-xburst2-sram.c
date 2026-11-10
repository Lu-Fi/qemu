/*
 * Ingenic XBurst2 SRAM: the on-chip SRAM the L2 cache shares with the NNA
 *
 * The CPUs' L2 cache and the NNA's ORAM are one SRAM, 1 MiB on a T40 or
 * A1 and 512 KiB on a T41, seen through a 1 MiB window at 0x12600000.
 * CCU MSCR bits 12:10 give the L2 its share from the bottom (0 none, then
 * 128, 256, 512 KiB and 1 MiB), and the ORAM is the rest. As a T40XP,
 * T41NQ and A1N show:
 * - the CPUs reach the ORAM as plain RAM, from 0x12600000 plus the L2's
 *   share;
 * - below that, in the L2's share, a CPU access reaches the SRAM's last
 *   2 KiB at its offset within 2 KiB, reads and writes alike;
 * - a 512 KiB SRAM repeats in the window's upper half, which shows the
 *   L2's share as the SRAM itself, the lines the L2 holds;
 * - SRAM changing hands keeps its bytes;
 * - the NNDMA reaches the SRAM by its offset, the L2's share included,
 *   where on silicon it overwrites the L2's lines.
 * A share larger than the SRAM (1 MiB on a T41) leaves the L2 SRAM that is
 * not there, and a T41 crashes. The L2's contents are not modelled: its
 * share holds what was last written there.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-xburst2-sram.h"
#include "migration/vmstate.h"

#define SRAM_TAIL   (2 * KiB)   /* what the L2's share reaches */

static uint8_t *sram_ptr(IngenicXBurst2SramState *s)
{
    return memory_region_get_ram_ptr(&s->ram);
}

static void sram_l2_log(IngenicXBurst2SramState *s, hwaddr addr, bool write)
{
    if (!s->logged) {
        s->logged = true;
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ingenic-xburst2-sram: %s 0x%" HWADDR_PRIx
                      ", in the L2's %u KiB (MSCR), reaches the SRAM's last"
                      " 2 KiB\n", write ? "write to" : "read of",
                      INGENIC_XBURST2_SRAM_BASE + addr,
                      (unsigned)(s->share / KiB));
    }
}

static uint64_t sram_l2_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicXBurst2SramState *s = opaque;
    uint8_t *tail = sram_ptr(s) + s->size - SRAM_TAIL;
    uint64_t v = 0;

    sram_l2_log(s, addr, false);
    for (unsigned i = 0; i < size; i++) {
        v |= (uint64_t)tail[(addr + i) & (SRAM_TAIL - 1)] << (8 * i);
    }
    return v;
}

static void sram_l2_write(void *opaque, hwaddr addr, uint64_t val,
                          unsigned size)
{
    IngenicXBurst2SramState *s = opaque;
    uint8_t *tail = sram_ptr(s) + s->size - SRAM_TAIL;

    sram_l2_log(s, addr, true);
    for (unsigned i = 0; i < size; i++) {
        uint32_t off = (addr + i) & (SRAM_TAIL - 1);

        tail[off] = val >> (8 * i);
        memory_region_set_dirty(&s->ram, s->size - SRAM_TAIL + off, 1);
    }
}

static const MemoryRegionOps sram_l2_ops = {
    .read = sram_l2_read,
    .write = sram_l2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

static unsigned sram_share_field(uint32_t share)
{
    return share ? ctz32(share / (64 * KiB)) : 0;
}

void ingenic_xburst2_sram_set_l2(IngenicXBurst2SramState *s, unsigned field)
{
    uint32_t share = field ? 64 * KiB << MIN(field, 4) : 0;
    uint32_t lo = MIN(share, s->size);

    if (share > s->size) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ingenic-xburst2-sram: an L2 of %u KiB is more than the"
                      " %u KiB SRAM\n", (unsigned)(share / KiB),
                      (unsigned)(s->size / KiB));
    }
    s->share = share;
    s->logged = false;

    memory_region_transaction_begin();
    memory_region_set_enabled(&s->l2, lo != 0);
    if (lo) {
        memory_region_set_size(&s->l2, lo);
    }
    memory_region_set_enabled(&s->oram, lo < s->size);
    if (lo < s->size) {
        memory_region_set_address(&s->oram, lo);
        memory_region_set_alias_offset(&s->oram, lo);
        memory_region_set_size(&s->oram, s->size - lo);
    }
    memory_region_transaction_commit();
}

void ingenic_xburst2_sram_dma(IngenicXBurst2SramState *s, uint32_t offset,
                              void *buf, uint32_t len, bool is_write)
{
    uint8_t *p = buf;

    while (len) {
        uint32_t off = offset % s->size;
        uint32_t n = MIN(len, s->size - off);

        if (is_write) {
            if (off < s->share) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "ingenic-xburst2-sram: the NNDMA wrote 0x%x,"
                              " in the L2's %u KiB, over its lines\n",
                              INGENIC_XBURST2_SRAM_BASE + off,
                              (unsigned)(s->share / KiB));
            }
            memcpy(sram_ptr(s) + off, p, n);
            memory_region_set_dirty(&s->ram, off, n);
        } else {
            memcpy(p, sram_ptr(s) + off, n);
        }
        offset += n;
        p += n;
        len -= n;
    }
}

static void sram_realize(DeviceState *dev, Error **errp)
{
    IngenicXBurst2SramState *s = INGENIC_XBURST2_SRAM(dev);

    if (s->size != 512 * KiB && s->size != INGENIC_XBURST2_SRAM_WINDOW) {
        error_setg(errp, "ingenic-xburst2-sram: size is 512 KiB or 1 MiB");
        return;
    }
    memory_region_init(&s->window, OBJECT(s), "ingenic-xburst2-sram",
                       INGENIC_XBURST2_SRAM_WINDOW);
    if (!memory_region_init_ram(&s->ram, OBJECT(s), "ingenic-xburst2-sram.ram",
                                s->size, errp)) {
        return;
    }
    memory_region_init_alias(&s->oram, OBJECT(s), "ingenic-xburst2-sram.oram",
                             &s->ram, 0, s->size);
    memory_region_add_subregion(&s->window, 0, &s->oram);
    memory_region_init_io(&s->l2, OBJECT(s), &sram_l2_ops, s,
                          "ingenic-xburst2-sram.l2", s->size);
    memory_region_add_subregion_overlap(&s->window, 0, &s->l2, 1);
    if (s->size < INGENIC_XBURST2_SRAM_WINDOW) {
        memory_region_init_alias(&s->mirror, OBJECT(s),
                                 "ingenic-xburst2-sram.mirror", &s->ram, 0,
                                 s->size);
        memory_region_add_subregion(&s->window, s->size, &s->mirror);
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->window);
    ingenic_xburst2_sram_set_l2(s, 0);
}

static int sram_post_load(void *opaque, int version_id)
{
    IngenicXBurst2SramState *s = opaque;

    ingenic_xburst2_sram_set_l2(s, sram_share_field(s->share));
    return 0;
}

static const VMStateDescription vmstate_ingenic_xburst2_sram = {
    .name = TYPE_INGENIC_XBURST2_SRAM,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = sram_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(share, IngenicXBurst2SramState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property sram_properties[] = {
    DEFINE_PROP_UINT32("size", IngenicXBurst2SramState, size, 1 * MiB),
};

static void sram_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sram_realize;
    dc->vmsd = &vmstate_ingenic_xburst2_sram;
    device_class_set_props(dc, sram_properties);
}

static const TypeInfo sram_info = {
    .name = TYPE_INGENIC_XBURST2_SRAM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicXBurst2SramState),
    .class_init = sram_class_init,
};

static void sram_register_types(void)
{
    type_register_static(&sram_info);
}

type_init(sram_register_types)
