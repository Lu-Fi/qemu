/*
 * Ingenic XBurst Real Time Clock
 *
 * A 32-bit seconds counter (RTCSR) with a control register, a seconds
 * alarm and a set of hibernate/wakeup registers. RTCSR tracks host
 * wall-clock time plus a guest-programmed offset; the alarm raises the
 * RTC interrupt when the counter reaches RTCSAR.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-rtc.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "system/runstate.h"

#define RTC_RTCCR       0x00
#define RTC_RTCSR       0x04
#define RTC_RTCSAR      0x08
#define RTC_CLKCNTINFO  0x18
#define RTC_HCR         0x20
#define RTC_WENR        0x3c

#define HCR_PD      (1U << 0)   /* hibernate: power down */

/* Period of the 32768 Hz RTC clock, in nanoseconds. */
#define RTC_CLK_PERIOD_NS  30518

#define RTCCR_WRDY  (1U << 7)
#define RTCCR_AF    (1U << 4)
#define RTCCR_AIE   (1U << 3)
#define RTCCR_AE    (1U << 2)

#define WENR_WEN    (1U << 31)

#define RTC_IOSIZE  0x1000

static uint32_t rtc_seconds(IngenicRtcState *s)
{
    int64_t host = qemu_clock_get_ns(QEMU_CLOCK_HOST) / NANOSECONDS_PER_SECOND;
    return (uint32_t)(host + s->offset);
}

static void rtc_update_alarm(IngenicRtcState *s)
{
    uint32_t now = rtc_seconds(s);
    uint32_t alarm = s->regs[RTC_RTCSAR / 4];

    if (!(s->regs[RTC_RTCCR / 4] & RTCCR_AE) || alarm <= now) {
        timer_del(s->alarm_timer);
        return;
    }
    timer_mod(s->alarm_timer,
              qemu_clock_get_ns(QEMU_CLOCK_HOST) +
              (int64_t)(alarm - now) * NANOSECONDS_PER_SECOND);
}

static void rtc_alarm_fire(void *opaque)
{
    IngenicRtcState *s = opaque;

    s->regs[RTC_RTCCR / 4] |= RTCCR_AF;
    if (s->regs[RTC_RTCCR / 4] & RTCCR_AIE) {
        qemu_set_irq(s->irq, 1);
    }
}

static uint64_t rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicRtcState *s = opaque;

    switch (offset) {
    case RTC_RTCSR:
        return rtc_seconds(s);
    case RTC_RTCCR:
        /* WRDY is always set - register writes never stall here. */
        return s->regs[RTC_RTCCR / 4] | RTCCR_WRDY;
    case RTC_WENR:
        /* Writes are always permitted under emulation. */
        return s->regs[RTC_WENR / 4] | WENR_WEN;
    case RTC_CLKCNTINFO:
        /*
         * Free-running RTC clock tick counter. The rtc-ingenic-a1
         * driver samples this twice and treats the RTC as present
         * only if the value advanced.
         */
        return qemu_clock_get_ns(QEMU_CLOCK_HOST) / RTC_CLK_PERIOD_NS;
    default:
        if (offset / 4 < INGENIC_RTC_NUM_REGS) {
            return s->regs[offset / 4];
        }
        return 0;
    }
}

static void rtc_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicRtcState *s = opaque;
    uint32_t v = val;

    switch (offset) {
    case RTC_RTCSR: {
        int64_t host = qemu_clock_get_ns(QEMU_CLOCK_HOST) /
                       NANOSECONDS_PER_SECOND;
        s->offset = (int64_t)v - host;
        rtc_update_alarm(s);
        return;
    }
    case RTC_RTCCR:
        /* AF is write-0-to-clear; WRDY is read-only. */
        s->regs[RTC_RTCCR / 4] = v & ~RTCCR_WRDY;
        if (!(v & RTCCR_AF)) {
            qemu_set_irq(s->irq, 0);
        }
        rtc_update_alarm(s);
        return;
    case RTC_RTCSAR:
        s->regs[RTC_RTCSAR / 4] = v;
        rtc_update_alarm(s);
        return;
    case RTC_HCR:
        /*
         * Hibernate control. Writing the power-down bit is how the SoC turns
         * itself off (the bootrom does this after exhausting all boot
         * attempts). Model it as a guest-initiated shutdown so the emulation
         * stops cleanly instead of spinning in the post-power-down wait loop.
         */
        s->regs[RTC_HCR / 4] = v;
        if (v & HCR_PD) {
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        return;
    default:
        if (offset / 4 < INGENIC_RTC_NUM_REGS) {
            s->regs[offset / 4] = v;
        }
        return;
    }
}

static const MemoryRegionOps rtc_ops = {
    .read = rtc_read,
    .write = rtc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void rtc_reset_hold(Object *obj, ResetType type)
{
    IngenicRtcState *s = INGENIC_RTC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->offset = 0;
    s->regs[RTC_RTCCR / 4] = 0x01;       /* RTCE; WRDY added on read */
    s->regs[0x2c / 4] = 0x08;            /* HWCR reset value */
    s->regs[0x48 / 4] = 0x00050064;      /* WKUPPINCR reset value */
    timer_del(s->alarm_timer);
    qemu_set_irq(s->irq, 0);
}

static void rtc_init(Object *obj)
{
    IngenicRtcState *s = INGENIC_RTC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &rtc_ops, s,
                          TYPE_INGENIC_RTC, RTC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->alarm_timer = timer_new_ns(QEMU_CLOCK_HOST, rtc_alarm_fire, s);
}

static const VMStateDescription vmstate_rtc = {
    .name = TYPE_INGENIC_RTC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(offset, IngenicRtcState),
        VMSTATE_UINT32_ARRAY(regs, IngenicRtcState, INGENIC_RTC_NUM_REGS),
        VMSTATE_TIMER_PTR(alarm_timer, IngenicRtcState),
        VMSTATE_END_OF_LIST()
    },
};

static void rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_rtc;
    rc->phases.hold = rtc_reset_hold;
}

static const TypeInfo rtc_type_info = {
    .name = TYPE_INGENIC_RTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicRtcState),
    .instance_init = rtc_init,
    .class_init = rtc_class_init,
};

static void rtc_register_types(void)
{
    type_register_static(&rtc_type_info);
}

type_init(rtc_register_types)
