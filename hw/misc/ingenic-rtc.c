/*
 * Ingenic XBurst Real Time Clock
 *
 * A 32-bit seconds counter (RTCSR) with a control register, a seconds
 * alarm and a set of hibernate/wakeup registers. RTCSR tracks host
 * wall-clock time plus a guest-programmed offset; the alarm raises the
 * RTC interrupt when the counter reaches RTCSAR.
 *
 * As a T30X runs its RTC (on EXCLK / 512, SELEXC): RTCE off stops RTCSR
 * and the 1 Hz flag; each second RTCSR moves on sets RTCCR's 1HZ, which
 * a written 0 clears and a written 1 leaves; the interrupt is AF and AIE
 * or 1HZ and 1HZIE, as a level. A locked RTCCR write (no WENR pattern)
 * sets SELEXC and 1HZIE at once and parks the rest: WRDY reads 0 and
 * RTCCR reads back what was written until the next WENR write, which
 * drops it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-rtc.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "system/runstate.h"

#define RTC_RTCCR       0x00
#define RTC_RTCSR       0x04
#define RTC_RTCSAR      0x08
#define RTC_RTCGR       0x0c
#define RTC_CLKCNTINFO  0x18
#define RTC_HCR         0x20
#define RTC_HWFCR       0x24
#define RTC_HRCR        0x28
#define RTC_HWCR        0x2c
#define RTC_HWRSR       0x30
#define RTC_HSPR        0x34
#define RTC_WENR        0x3c
#define RTC_WKUPPINCR   0x48

#define HCR_PD      (1U << 0)   /* hibernate: power down */
#define HWCR_EALM   (1U << 0)   /* the alarm wakes the SoC */
#define HWRSR_ALM   (1U << 0)   /* woken by the alarm */
#define HWRSR_PIN   (1U << 1)   /* woken by the wakeup pin */
#define HWRSR_PPR   (1U << 4)   /* a power-on reset */
#define HWRSR_HR    (1U << 5)   /* a hibernate reset */

/* Period of the 32768 Hz RTC clock, in nanoseconds. */
#define RTC_CLK_PERIOD_NS  30518

#define RTCCR_WRDY  (1U << 7)
#define RTCCR_1HZ   (1U << 6)
#define RTCCR_HZIE  (1U << 5)
#define RTCCR_AF    (1U << 4)
#define RTCCR_AIE   (1U << 3)
#define RTCCR_AE    (1U << 2)
#define RTCCR_SELEXC (1U << 1)
#define RTCCR_RTCE  (1U << 0)

#define WENR_WEN    (1U << 31)
#define WENR_PAT    0xa55a

#define RTC_IOSIZE  0x1000

static int64_t rtc_host_seconds(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_HOST) / NANOSECONDS_PER_SECOND;
}

static uint32_t rtc_seconds(IngenicRtcState *s)
{
    if (!(s->regs[RTC_RTCCR / 4] & RTCCR_RTCE)) {
        return s->frozen;
    }
    return (uint32_t)(rtc_host_seconds() + s->offset);
}

static void rtc_update_irq(IngenicRtcState *s)
{
    uint32_t cr = s->regs[RTC_RTCCR / 4];

    qemu_set_irq(s->irq, !s->hibernating &&
                 (((cr & RTCCR_AF) && (cr & RTCCR_AIE)) ||
                  ((cr & RTCCR_1HZ) && (cr & RTCCR_HZIE))));
}

/* The next second of RTCSR, while it counts */
static void rtc_hz_arm(IngenicRtcState *s)
{
    if (!(s->regs[RTC_RTCCR / 4] & RTCCR_RTCE)) {
        timer_del(s->hz_timer);
        return;
    }
    timer_mod(s->hz_timer,
              (rtc_host_seconds() + 1) * NANOSECONDS_PER_SECOND);
}

static void rtc_hz_fire(void *opaque)
{
    IngenicRtcState *s = opaque;

    s->regs[RTC_RTCCR / 4] |= RTCCR_1HZ;
    rtc_update_irq(s);
    rtc_hz_arm(s);
}

static void rtc_update_alarm(IngenicRtcState *s)
{
    uint32_t now = rtc_seconds(s);
    uint32_t alarm = s->regs[RTC_RTCSAR / 4];

    if (!(s->regs[RTC_RTCCR / 4] & RTCCR_AE) ||
        !(s->regs[RTC_RTCCR / 4] & RTCCR_RTCE) || alarm <= now) {
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
    if (s->hibernating) {
        qemu_system_wakeup_request(QEMU_WAKEUP_REASON_RTC, NULL);
        return;
    }
    rtc_update_irq(s);
}

/*
 * Power-down leaves only the RTC running. With the alarm armed to wake the
 * SoC (the vendor kernels' hibernate restart) the machine sleeps until the
 * alarm, or until the monitor's system_wakeup, the wakeup pin; anything else
 * (pm_power_off, the bootrom giving up) has nothing to wake it: power off.
 */
static void rtc_hibernate(IngenicRtcState *s)
{
    if ((s->regs[RTC_HWCR / 4] & HWCR_EALM) &&
        (s->regs[RTC_RTCCR / 4] & RTCCR_AE) &&
        s->regs[RTC_RTCSAR / 4] > rtc_seconds(s)) {
        s->hibernating = true;
        qemu_system_suspend_request();
    } else {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
    }
}

/* The machine's wakeup is a cold boot; the RTC keeps the wake reason */
static void rtc_wakeup(Notifier *n, void *data)
{
    IngenicRtcState *s = container_of(n, IngenicRtcState, wakeup);
    WakeupReason *reason = data;

    if (!s->hibernating) {
        return;
    }
    s->hibernating = false;
    s->regs[RTC_HCR / 4] = 0;
    s->regs[RTC_HWCR / 4] &= ~HWCR_EALM;
    s->regs[RTC_HWRSR / 4] |= HWRSR_HR |
        (*reason == QEMU_WAKEUP_REASON_RTC ? HWRSR_ALM : HWRSR_PIN);
}

static uint64_t rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicRtcState *s = opaque;

    switch (offset) {
    case RTC_RTCSR:
        return rtc_seconds(s);
    case RTC_RTCCR:
        /* A parked write reads back as written; others complete at once */
        if (s->parked) {
            return s->parked_val & ~RTCCR_WRDY;
        }
        return s->regs[RTC_RTCCR / 4] | RTCCR_WRDY;
    case RTC_WENR:
        return s->wen ? WENR_WEN : 0;
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

    /*
     * The pattern in WENR opens the registers below for one write, and
     * any other WENR write locks them again (a T41NQ); a WENR write drops
     * a parked RTCCR write (a T30X)
     */
    switch (offset) {
    case RTC_WENR:
        s->wen = (v & 0xffff) == WENR_PAT;
        s->parked = false;
        return;
    case RTC_RTCCR:
        if (!s->wen) {
            uint32_t now = RTCCR_SELEXC | RTCCR_HZIE;

            s->regs[RTC_RTCCR / 4] = (s->regs[RTC_RTCCR / 4] & ~now) |
                                     (v & now);
            s->parked = true;
            s->parked_val = v;
            rtc_update_irq(s);
            return;
        }
        break;
    case RTC_RTCSR:
    case RTC_RTCSAR:
    case RTC_RTCGR:
    case RTC_HCR:
    case RTC_HWFCR:
    case RTC_HRCR:
    case RTC_HWCR:
    case RTC_HWRSR:
    case RTC_HSPR:
    case RTC_WKUPPINCR:
        if (!s->wen) {
            return;
        }
        break;
    default:
        if (offset / 4 < INGENIC_RTC_NUM_REGS) {
            s->regs[offset / 4] = v;
        }
        return;
    }
    s->wen = false;

    switch (offset) {
    case RTC_RTCSR:
        s->frozen = v;
        s->offset = (int64_t)v - rtc_host_seconds();
        rtc_update_alarm(s);
        return;
    case RTC_RTCCR: {
        uint32_t old = s->regs[RTC_RTCCR / 4];
        uint32_t flags = RTCCR_AF | RTCCR_1HZ;  /* a written 0 clears */

        s->parked = false;
        if ((old & RTCCR_RTCE) && !(v & RTCCR_RTCE)) {
            s->frozen = rtc_seconds(s);
        } else if (!(old & RTCCR_RTCE) && (v & RTCCR_RTCE)) {
            s->offset = (int64_t)s->frozen - rtc_host_seconds();
        }
        s->regs[RTC_RTCCR / 4] = (v & ~(RTCCR_WRDY | flags)) |
                                 (old & v & flags);
        rtc_update_alarm(s);
        rtc_hz_arm(s);
        rtc_update_irq(s);
        return;
    }
    case RTC_RTCSAR:
        s->regs[RTC_RTCSAR / 4] = v;
        rtc_update_alarm(s);
        return;
    case RTC_HCR:
        s->regs[RTC_HCR / 4] = v;
        if (v & HCR_PD) {
            rtc_hibernate(s);
        }
        return;
    case RTC_HWRSR:
        s->regs[RTC_HWRSR / 4] &= v;    /* write 0 to clear */
        return;
    default:
        s->regs[offset / 4] = v;
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

/*
 * The RTC is in the always-on domain: its registers and alarm are set at
 * power-on and survive every reset, a wake from hibernation included
 */
static void rtc_realize(DeviceState *dev, Error **errp)
{
    IngenicRtcState *s = INGENIC_RTC(dev);

    s->regs[RTC_RTCCR / 4] = 0x01;       /* RTCE; WRDY added on read */
    /* Power detect on, as every part reads it and the T21 manual gives */
    s->regs[RTC_HWCR / 4] = 0x2d52d2d0;
    s->regs[RTC_HWRSR / 4] = HWRSR_PPR;
    s->regs[RTC_HRCR / 4] = s->hrcr;
    s->regs[RTC_WKUPPINCR / 4] = s->wkuppincr;
    s->wakeup.notify = rtc_wakeup;
    qemu_register_wakeup_notifier(&s->wakeup);
    rtc_hz_arm(s);
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
    s->hz_timer = timer_new_ns(QEMU_CLOCK_HOST, rtc_hz_fire, s);
}

static const VMStateDescription vmstate_rtc = {
    .name = TYPE_INGENIC_RTC,
    .version_id = 4,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(offset, IngenicRtcState),
        VMSTATE_UINT32_ARRAY(regs, IngenicRtcState, INGENIC_RTC_NUM_REGS),
        VMSTATE_TIMER_PTR(alarm_timer, IngenicRtcState),
        VMSTATE_BOOL_V(hibernating, IngenicRtcState, 2),
        VMSTATE_BOOL_V(wen, IngenicRtcState, 3),
        VMSTATE_TIMER_PTR_V(hz_timer, IngenicRtcState, 4),
        VMSTATE_UINT32_V(frozen, IngenicRtcState, 4),
        VMSTATE_BOOL_V(parked, IngenicRtcState, 4),
        VMSTATE_UINT32_V(parked_val, IngenicRtcState, 4),
        VMSTATE_END_OF_LIST()
    },
};

/* The T21 manual's WKUPPINCR, as a T20X, T21N, T30X and T31X read it */
static const Property rtc_properties[] = {
    DEFINE_PROP_UINT32("wkuppincr", IngenicRtcState, wkuppincr, 0x00010064),
    DEFINE_PROP_UINT32("hrcr", IngenicRtcState, hrcr, 0x800),
};

static void rtc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = rtc_realize;
    dc->vmsd = &vmstate_rtc;
    device_class_set_props(dc, rtc_properties);
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
