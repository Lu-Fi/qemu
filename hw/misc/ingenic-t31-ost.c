/*
 * Ingenic T31 OS Timer (OST) emulation
 *
 * The OST is a 64-bit free-running counter clocked from EXTAL/4 (6 MHz).
 * U-Boot uses it as the system timer via get_timer().
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "hw/misc/ingenic-t31-ost.h"

/* TCU register offsets */
#define OST_TESR      0x14
#define TCU_TSCR      0x3C

/* OST register offsets (at TCU base + 0xE0) */
#define OST_DR        0xE0
#define OST_CNTL      0xE4
#define OST_CNTH      0xE8
#define OST_CSR       0xEC
#define OST_CNTHBUF   0xFC

#define OST_IOSIZE    0x100

/* 6 MHz - EXTAL(24MHz) / prescale(4) */
#define OST_FREQ      6000000ULL

static uint64_t ingenic_t31_ost_get_count(IngenicT31OstState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->base_time;

    return (uint64_t)(elapsed * OST_FREQ / NANOSECONDS_PER_SECOND);
}

static uint64_t ingenic_t31_ost_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT31OstState *s = INGENIC_T31_OST(opaque);
    uint64_t count;

    switch (offset) {
    case OST_CNTL:
        count = ingenic_t31_ost_get_count(s);
        s->cnth_buf = (uint32_t)(count >> 32);
        return (uint32_t)count;
    case OST_CNTH:
        return s->cnth_buf;
    case OST_CNTHBUF:
        return s->cnth_buf;
    case OST_DR:
        return s->data_reg;
    case OST_CSR:
        return s->csr;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read (offset 0x%03" HWADDR_PRIx ")\n",
                      __func__, offset);
        return 0;
    }
}

static void ingenic_t31_ost_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT31OstState *s = INGENIC_T31_OST(opaque);

    switch (offset) {
    case TCU_TSCR:
        break;
    case OST_DR:
        s->data_reg = (uint32_t)value;
        break;
    case OST_CNTL:
        s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    case OST_CSR:
        s->csr = (uint32_t)value;
        break;
    case OST_TESR:
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write (offset 0x%03" HWADDR_PRIx
                      ", value 0x%0*" PRIx64 ")\n",
                      __func__, offset, size << 1, value);
        break;
    }
}

static const MemoryRegionOps ingenic_t31_ost_ops = {
    .read = ingenic_t31_ost_read,
    .write = ingenic_t31_ost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 4 },
    .impl  = { .min_access_size = 2, .max_access_size = 4 },
};

static void ingenic_t31_ost_reset_hold(Object *obj, ResetType type)
{
    IngenicT31OstState *s = INGENIC_T31_OST(obj);

    s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->cnth_buf = 0;
    s->data_reg = 0;
    s->csr = 0;
}

static void ingenic_t31_ost_init(Object *obj)
{
    IngenicT31OstState *s = INGENIC_T31_OST(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_ost_ops, s,
                          TYPE_INGENIC_T31_OST, OST_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void ingenic_t31_ost_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_t31_ost;
    rc->phases.hold = ingenic_t31_ost_reset_hold;
}

static const TypeInfo ingenic_t31_ost_type_info = {
    .name = TYPE_INGENIC_T31_OST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31OstState),
    .instance_init = ingenic_t31_ost_init,
    .class_init = ingenic_t31_ost_class_init,
};

static void ingenic_t31_ost_register_types(void)
{
    type_register_static(&ingenic_t31_ost_type_info);
}

type_init(ingenic_t31_ost_register_types)
