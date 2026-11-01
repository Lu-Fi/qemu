/*
 * Ingenic A1 RTC emulation
 *
 * Not the T-series RTC: four registers, as an A1N reads them and the A1
 * kernel's rtc-ingenic-a1 drives them, and the rest of the page reads 0.
 * VERSION reads 0x20210425. RTCSR counts seconds and takes no write: the
 * driver keeps its own base time against it. CLKCNTEN's bit 0 starts
 * CLKCNTINFO, which counts at EXTCLK / 512 (46875 Hz).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/misc/ingenic-a1-rtc.h"
#include "migration/vmstate.h"

#define RTC_VERSION     0x00
#define RTC_RTCSR       0x04
#define RTC_CLKCNTEN    0x10
#define RTC_CLKCNTINFO  0x18

#define A1_RTC_VERSION  0x20210425
#define CLKCNT_HZ       46875       /* 24 MHz / 512 */

static uint32_t a1_rtc_clkcnt(IngenicA1RtcState *s)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->clkcnt_ns;

    if (!(s->clkcnten & 1)) {
        return s->clkcnt;
    }
    return s->clkcnt + muldiv64(ns, CLKCNT_HZ, NANOSECONDS_PER_SECOND);
}

static uint64_t a1_rtc_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicA1RtcState *s = opaque;

    switch (addr) {
    case RTC_VERSION:
        return A1_RTC_VERSION;
    case RTC_RTCSR:
        return (uint32_t)(qemu_clock_get_ns(QEMU_CLOCK_HOST) /
                          NANOSECONDS_PER_SECOND);
    case RTC_CLKCNTEN:
        return s->clkcnten;
    case RTC_CLKCNTINFO:
        return a1_rtc_clkcnt(s);
    default:
        return 0;
    }
}

static void a1_rtc_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    IngenicA1RtcState *s = opaque;

    if (addr == RTC_CLKCNTEN) {
        s->clkcnt = a1_rtc_clkcnt(s);
        s->clkcnt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->clkcnten = value & 1;
    }
}

static const MemoryRegionOps a1_rtc_ops = {
    .read = a1_rtc_read,
    .write = a1_rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void a1_rtc_reset_hold(Object *obj, ResetType type)
{
    IngenicA1RtcState *s = INGENIC_A1_RTC(obj);

    s->clkcnten = 0;
    s->clkcnt = 0;
    s->clkcnt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void a1_rtc_init(Object *obj)
{
    IngenicA1RtcState *s = INGENIC_A1_RTC(obj);

    memory_region_init_io(&s->iomem, obj, &a1_rtc_ops, s,
                          TYPE_INGENIC_A1_RTC, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_a1_rtc = {
    .name = TYPE_INGENIC_A1_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(clkcnten, IngenicA1RtcState),
        VMSTATE_UINT32(clkcnt, IngenicA1RtcState),
        VMSTATE_INT64(clkcnt_ns, IngenicA1RtcState),
        VMSTATE_END_OF_LIST()
    }
};

static void a1_rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_a1_rtc;
    rc->phases.hold = a1_rtc_reset_hold;
}

static const TypeInfo a1_rtc_info = {
    .name = TYPE_INGENIC_A1_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1RtcState),
    .instance_init = a1_rtc_init,
    .class_init = a1_rtc_class_init,
};

static void a1_rtc_register_types(void)
{
    type_register_static(&a1_rtc_info);
}

type_init(a1_rtc_register_types)
