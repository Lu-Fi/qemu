/*
 * Ingenic XBurst2 Digital True Random Number Generator (DTRNG)
 *
 * A small block with three registers: a config register (the generate
 * enable lives in bit 0), a random-number data register, and a status
 * register whose bit 0 signals "data ready". The Linux ingenic-rng
 * hwrng driver polls STATUS and reads RANDOMNUM.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-dtrng.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"

#define DTRNG_CFG        0x00
#define DTRNG_RANDOMNUM  0x04
#define DTRNG_STATUS     0x08

#define DTRNG_CFG_GEN_EN    (1U << 0)
#define DTRNG_CFG_INT_MASK  (1U << 11)
#define DTRNG_CFG_RDY_CLR   (1U << 12)

#define DTRNG_IOSIZE     0x1000

static uint64_t dtrng_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicDtrngState *s = opaque;

    switch (offset) {
    case DTRNG_CFG:
        return s->cfg;
    case DTRNG_RANDOMNUM:
        return g_random_int();
    case DTRNG_STATUS:
        /* Bit 0: a fresh number is always ready once generation is on. */
        return (s->cfg & DTRNG_CFG_GEN_EN) ? 1 : 0;
    default:
        return 0;
    }
}

static void dtrng_write(void *opaque, hwaddr offset, uint64_t val,
                        unsigned size)
{
    IngenicDtrngState *s = opaque;

    if (offset == DTRNG_CFG) {
        /* RDY_CLR is a write-only self-clearing pulse; don't latch it. */
        s->cfg = val & ~DTRNG_CFG_RDY_CLR;
    }
}

static const MemoryRegionOps dtrng_ops = {
    .read = dtrng_read,
    .write = dtrng_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void dtrng_reset_hold(Object *obj, ResetType type)
{
    IngenicDtrngState *s = INGENIC_DTRNG(obj);

    s->cfg = 0;
    qemu_set_irq(s->irq, 0);
}

static void dtrng_init(Object *obj)
{
    IngenicDtrngState *s = INGENIC_DTRNG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dtrng_ops, s,
                          TYPE_INGENIC_DTRNG, DTRNG_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_dtrng = {
    .name = TYPE_INGENIC_DTRNG,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cfg, IngenicDtrngState),
        VMSTATE_END_OF_LIST()
    },
};

static void dtrng_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_dtrng;
    rc->phases.hold = dtrng_reset_hold;
}

static const TypeInfo dtrng_type_info = {
    .name = TYPE_INGENIC_DTRNG,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDtrngState),
    .instance_init = dtrng_init,
    .class_init = dtrng_class_init,
};

static void dtrng_register_types(void)
{
    type_register_static(&dtrng_type_info);
}

type_init(dtrng_register_types)
