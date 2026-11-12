/*
 * Ingenic A1 video DAC
 *
 * The VGA output's converter at 0x10076000. On an A1N running the SDK 1.7.0
 * demo image, with the VO started, 0x10076200 to 0x100762ff read all zero
 * although vo.ko writes +0x288, +0x294, +0x2a0 and others there: the writes
 * are not kept, or the block is not clocked. The model reads zero and drops
 * every write. Not measured: the rest of the page, and any read before the
 * VO runs (assumed zero too).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/misc/ingenic-a1-vdac.h"
#include "trace.h"

static uint64_t vdac_read(void *opaque, hwaddr addr, unsigned size)
{
    return 0;
}

static void vdac_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    trace_ingenic_a1_vdac_write(addr, value);
}

static const MemoryRegionOps vdac_ops = {
    .read = vdac_read,
    .write = vdac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void vdac_realize(DeviceState *dev, Error **errp)
{
    IngenicA1VdacState *s = INGENIC_A1_VDAC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &vdac_ops, s,
                          TYPE_INGENIC_A1_VDAC, INGENIC_A1_VDAC_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void vdac_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = vdac_realize;
}

static const TypeInfo vdac_info = {
    .name = TYPE_INGENIC_A1_VDAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1VdacState),
    .class_init = vdac_class_init,
};

static void vdac_register_types(void)
{
    type_register_static(&vdac_info);
}

type_init(vdac_register_types)
