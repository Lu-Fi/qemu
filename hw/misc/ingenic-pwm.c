/*
 * Ingenic XBurst2 PWM controller
 *
 * The dedicated PWM block: 16 channels on the T41 and A1 at 0x13460000, 4
 * on the T32 at 0x13450000. Its registers repeat every 4 KiB through its
 * 64 KiB window; past those a channel lacks, and in the gaps between them,
 * the window reads 0 and drops writes. The DMA mode is register storage.
 *
 * A channel counts its clock over 2^PRESCALE (CCFG, 4 bits a channel) and
 * runs periods of HIGH + LOW ticks (WCFG): first its init level (PWMLVL bits
 * 15:0) for that level's count, then the other level for the other. With
 * either count 0 it holds the init level, and with both a period lasts
 * 2^17 ticks. ENS starts it with what WCFG,
 * PWMLVL and CCFG hold then; while it runs, they reach it only through
 * PWMUPDATE, at the end of the period it is in, and BUSY stays set until
 * then (on a stopped channel, until it has run a period). ENC clears EN at
 * once, but the channel finishes its period before it rests at its idle
 * level (PWMLVL bits 31:16). PWMOEN changes nothing at the pad. All as a
 * T41NQ and a T32LQ do at U-Boot.
 *
 * Each channel's output is exposed as pwmN-level, pwmN-period-ns and
 * pwmN-high-ns, and with "trace" set to a chardev, written there as a JSON
 * line each time it changes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-pwm.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/host-utils.h"

#define PWM_CCFG0   0x00    /* channels 0-7's prescale, 4 bits each */
#define PWM_CCFG1   0x04    /* channels 8-15's */
#define PWM_ENS     0x10    /* channel enable set */
#define PWM_ENC     0x14    /* channel enable clear */
#define PWM_EN      0x18    /* channel enable status */
#define PWM_UP      0x20    /* WCFG, PWMLVL and CCFG at the period's end */
#define PWM_BUSY    0x24    /* update pending */
#define PWM_LVL     0x30    /* init levels 15:0, idle levels 31:16 */
#define PWM_WCFG    0xb0    /* + 4n: high count 31:16, low count 15:0 */
#define PWM_DES     0x100   /* DMA enable set */
#define PWM_DEC     0x104   /* DMA enable clear */
#define PWM_DE      0x108   /* DMA enable status */
#define PWM_DCR0    0x110
#define PWM_DCR1    0x114
#define PWM_DFER    0x124   /* DMA FIFO empty, one bit a channel */
#define PWM_DINTC   0x138
#define PWM_DAR     0x140   /* + 4n */
#define PWM_DTLR    0x190   /* + 4n */
#define PWM_OEN     0x300

#define PWM_WINDOW  0x1000
#define PWM_IOSIZE  0x10000

static uint32_t pwm_chmask(IngenicPwmState *s)
{
    return MAKE_64BIT_MASK(0, s->channels);
}

/* The bits of a write to @offset its register keeps */
static uint32_t pwm_wmask(IngenicPwmState *s, hwaddr offset)
{
    switch (offset) {
    case PWM_CCFG0:
    case PWM_LVL:
        return UINT32_MAX;
    case PWM_CCFG1:
        return s->channels > 8 ? UINT32_MAX : 0;
    case PWM_DCR0:
    case PWM_DCR1:
    case PWM_DINTC:
    case PWM_OEN:
        return 0xffff;
    }
    if ((offset >= PWM_WCFG && offset < PWM_WCFG + 4 * s->channels) ||
        (offset >= PWM_DAR && offset < PWM_DAR + 4 * s->channels)) {
        return UINT32_MAX;
    }
    if (offset >= PWM_DTLR && offset < PWM_DTLR + 4 * s->channels) {
        return 0xffff;
    }
    return 0;
}

/* HIGH + LOW ticks; with both 0, as long as 65535 each (a 17-bit count) */
static uint32_t pwm_period(const IngenicPwmChannel *c)
{
    return c->high || c->low ? (uint32_t)c->high + c->low : 1U << 17;
}

/* The whole ticks since anchor_ns */
static uint64_t pwm_ticks(const IngenicPwmChannel *c, int64_t now)
{
    if (now <= c->anchor_ns || !c->hz) {
        return 0;
    }
    return muldiv64(now - c->anchor_ns, c->hz, NANOSECONDS_PER_SECOND) >>
           c->prescale;
}

/* When tick k after anchor_ns lands */
static int64_t pwm_tick_ns(const IngenicPwmChannel *c, uint64_t k)
{
    return c->anchor_ns + muldiv64_round_up(k << c->prescale,
                                            NANOSECONDS_PER_SECOND, c->hz);
}

/* Take the waveform from the registers, its periods counted from @now */
static void pwm_latch(IngenicPwmState *s, IngenicPwmChannel *c, int64_t now)
{
    unsigned n = c->index;
    uint32_t wcfg = s->regs[(PWM_WCFG + 4 * n) / 4];

    c->high = wcfg >> 16;
    c->low = wcfg;
    c->init = extract32(s->regs[PWM_LVL / 4], n, 1);
    c->prescale = extract32(s->regs[(n < 8 ? PWM_CCFG0 : PWM_CCFG1) / 4],
                            n % 8 * 4, 4);
    c->hz = clock_get_hz(s->clk);
    c->anchor_ns = now;
}

static void pwm_out(IngenicPwmState *s, unsigned n, IngenicPwmOut *o,
                    IngenicPwmKey *key)
{
    IngenicPwmChannel *c = &s->chn[n];
    uint64_t k, pos, first, period;

    memset(o, 0, sizeof(*o));
    memset(key, 0, sizeof(*key));
    if (!c->running) {
        o->level = extract32(s->regs[PWM_LVL / 4], 16 + n, 1);
    } else if (!c->hz || !c->high || !c->low) {
        o->level = c->init;
    } else {
        period = pwm_period(c);
        first = c->init ? c->high : c->low;
        k = pwm_ticks(c, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        pos = k % period;
        o->level = pos < first ? c->init : !c->init;
        o->period_ns = muldiv64(period << c->prescale, NANOSECONDS_PER_SECOND,
                                c->hz);
        o->high_ns = muldiv64((uint64_t)c->high << c->prescale,
                              NANOSECONDS_PER_SECOND, c->hz);
        o->edge_ns = pwm_tick_ns(c, k - pos + (pos < first ? first : period));
        key->k[0] = c->high | c->low << 16 | (uint64_t)c->init << 32 |
                    (uint64_t)c->prescale << 40 | 1ULL << 48;
        key->k[1] = c->hz;
        key->k[2] = c->anchor_ns;
        return;
    }
    key->k[0] = o->level;
}

static void pwm_trace(IngenicPwmState *s, unsigned n)
{
    IngenicPwmOut o;
    IngenicPwmKey key;

    if (!qemu_chr_fe_backend_connected(&s->trace) || n >= s->channels) {
        return;
    }
    pwm_out(s, n, &o, &key);
    if (memcmp(&key, &s->chn[n].traced, sizeof(key))) {
        s->chn[n].traced = key;
        ingenic_pwm_out_trace(&s->trace, "pwm", n, &o);
    }
}

/* A timer at the period's end, while an update or a stop waits on it */
static void pwm_reschedule(IngenicPwmState *s, IngenicPwmChannel *c)
{
    uint32_t bit = 1U << c->index;
    uint64_t period = pwm_period(c);

    if (!c->running || !c->hz ||
        (!(s->regs[PWM_BUSY / 4] & bit) && (s->regs[PWM_EN / 4] & bit))) {
        timer_del(c->timer);
        return;
    }
    timer_mod(c->timer, pwm_tick_ns(c, (pwm_ticks(c, qemu_clock_get_ns(
                                            QEMU_CLOCK_VIRTUAL)) / period + 1)
                                           * period));
}

static void pwm_period_end(void *opaque)
{
    IngenicPwmChannel *c = opaque;
    IngenicPwmState *s = c->pwm;
    uint32_t bit = 1U << c->index;
    uint64_t period = pwm_period(c);
    int64_t end;

    end = pwm_tick_ns(c, pwm_ticks(c, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL))
                         / period * period);
    if (s->regs[PWM_BUSY / 4] & bit) {
        s->regs[PWM_BUSY / 4] &= ~bit;
        pwm_latch(s, c, end);
    }
    if (!(s->regs[PWM_EN / 4] & bit)) {
        c->running = false;
    }
    pwm_trace(s, c->index);
    pwm_reschedule(s, c);
}

static void pwm_clk_update(void *opaque, ClockEvent event)
{
    IngenicPwmState *s = opaque;
    uint32_t hz = clock_get_hz(s->clk);

    for (unsigned n = 0; n < s->channels; n++) {
        IngenicPwmChannel *c = &s->chn[n];

        if (c->running && c->hz != hz) {
            c->hz = hz;
            c->anchor_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        pwm_trace(s, n);
        pwm_reschedule(s, c);
    }
}

static uint64_t pwm_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicPwmState *s = opaque;

    offset &= PWM_WINDOW - 1;
    if (offset == PWM_DFER) {
        return pwm_chmask(s) & 0xffff;
    }
    return offset < INGENIC_PWM_NUM_REGS * 4 ? s->regs[offset / 4] : 0;
}

static void pwm_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicPwmState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t v = val & pwm_chmask(s);

    offset &= PWM_WINDOW - 1;
    switch (offset) {
    case PWM_ENS:
        for (unsigned n = 0; n < s->channels; n++) {
            if ((v & ~s->regs[PWM_EN / 4]) & (1U << n) && !s->chn[n].running) {
                pwm_latch(s, &s->chn[n], now);
                s->chn[n].running = true;
            }
        }
        s->regs[PWM_EN / 4] |= v;
        break;
    case PWM_ENC:
        s->regs[PWM_EN / 4] &= ~v;
        break;
    case PWM_UP:
        s->regs[PWM_BUSY / 4] |= v;
        break;
    case PWM_DES:
        s->regs[PWM_DE / 4] |= v;
        return;
    case PWM_DEC:
        s->regs[PWM_DE / 4] &= ~v;
        return;
    default:
        /* EN, BUSY, DE and the status registers keep theirs */
        if (pwm_wmask(s, offset)) {
            s->regs[offset / 4] = val & pwm_wmask(s, offset);
        }
        if (offset != PWM_LVL) {
            return;
        }
        break;
    }
    for (unsigned n = 0; n < s->channels; n++) {
        pwm_reschedule(s, &s->chn[n]);
        pwm_trace(s, n);
    }
}

static const MemoryRegionOps pwm_ops = {
    .read = pwm_read,
    .write = pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void pwm_get_out(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    IngenicPwmChannel *c = opaque;
    IngenicPwmOut o;
    IngenicPwmKey key;
    uint64_t val;

    pwm_out(c->pwm, c->index, &o, &key);
    val = g_str_has_suffix(name, "-level") ? o.level :
          g_str_has_suffix(name, "-period-ns") ? o.period_ns : o.high_ns;
    visit_type_uint64(v, name, &val, errp);
}

static void pwm_reset_hold(Object *obj, ResetType type)
{
    IngenicPwmState *s = INGENIC_PWM(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->regs[PWM_DINTC / 4] = 0xffff;
    for (unsigned n = 0; n < INGENIC_PWM_CHANNELS; n++) {
        s->chn[n].running = false;
        timer_del(s->chn[n].timer);
    }
    qemu_set_irq(s->irq, 0);
}

static void pwm_reset_exit(Object *obj, ResetType type)
{
    IngenicPwmState *s = INGENIC_PWM(obj);

    if (qemu_chr_fe_backend_connected(&s->trace)) {
        g_autofree char *js = g_strdup_printf(
            "{\"t\":%" PRId64 ",\"reset\":true,\"channels\":%u}\n",
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL), s->channels);

        qemu_chr_fe_write_all(&s->trace, (uint8_t *)js, strlen(js));
        for (unsigned n = 0; n < s->channels; n++) {
            memset(&s->chn[n].traced, 0xff, sizeof(s->chn[n].traced));
            pwm_trace(s, n);
        }
    }
}

static void pwm_init(Object *obj)
{
    IngenicPwmState *s = INGENIC_PWM(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pwm_ops, s,
                          TYPE_INGENIC_PWM, PWM_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", pwm_clk_update, s,
                                ClockUpdate);

    for (unsigned n = 0; n < INGENIC_PWM_CHANNELS; n++) {
        static const char *const what[] = { "level", "period-ns", "high-ns" };

        s->chn[n].pwm = s;
        s->chn[n].index = n;
        s->chn[n].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pwm_period_end,
                                       &s->chn[n]);
        for (unsigned i = 0; i < ARRAY_SIZE(what); i++) {
            g_autofree char *name = g_strdup_printf("pwm%u-%s", n, what[i]);

            object_property_add(obj, name, "uint64", pwm_get_out, NULL, NULL,
                                &s->chn[n]);
        }
    }
}

static void pwm_realize(DeviceState *dev, Error **errp)
{
    IngenicPwmState *s = INGENIC_PWM(dev);

    if (!s->channels || s->channels > INGENIC_PWM_CHANNELS) {
        error_setg(errp, "channels %u: not 1 to %d", s->channels,
                   INGENIC_PWM_CHANNELS);
    }
}

static const VMStateDescription vmstate_pwm_channel = {
    .name = "ingenic-pwm/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(running, IngenicPwmChannel),
        VMSTATE_UINT16(high, IngenicPwmChannel),
        VMSTATE_UINT16(low, IngenicPwmChannel),
        VMSTATE_UINT8(init, IngenicPwmChannel),
        VMSTATE_UINT8(prescale, IngenicPwmChannel),
        VMSTATE_UINT32(hz, IngenicPwmChannel),
        VMSTATE_INT64(anchor_ns, IngenicPwmChannel),
        VMSTATE_TIMER_PTR(timer, IngenicPwmChannel),
        VMSTATE_END_OF_LIST()
    },
};

static const VMStateDescription vmstate_pwm = {
    .name = TYPE_INGENIC_PWM,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicPwmState, INGENIC_PWM_NUM_REGS),
        VMSTATE_STRUCT_ARRAY(chn, IngenicPwmState, INGENIC_PWM_CHANNELS, 1,
                             vmstate_pwm_channel, IngenicPwmChannel),
        VMSTATE_CLOCK(clk, IngenicPwmState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property pwm_properties[] = {
    DEFINE_PROP_UINT32("channels", IngenicPwmState, channels,
                       INGENIC_PWM_CHANNELS),
    DEFINE_PROP_CHR("trace", IngenicPwmState, trace),
};

static void pwm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = pwm_realize;
    dc->vmsd = &vmstate_pwm;
    device_class_set_props(dc, pwm_properties);
    rc->phases.hold = pwm_reset_hold;
    rc->phases.exit = pwm_reset_exit;
}

static const TypeInfo pwm_type_info = {
    .name = TYPE_INGENIC_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicPwmState),
    .instance_init = pwm_init,
    .class_init = pwm_class_init,
};

static void pwm_register_types(void)
{
    type_register_static(&pwm_type_info);
}

type_init(pwm_register_types)
