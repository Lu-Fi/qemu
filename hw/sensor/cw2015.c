/*
 * CellWise CW2015 battery fuel gauge
 *
 * Eight-bit registers behind a pointer that steps on each byte: VERSION
 * (0x00, 0x6f), VCELL (0x02, 14 bits, 305 uV a step), SOC (0x04, percent
 * and 1/256 percent), RRT_ALRT (0x06, ALRT and 13 bits of minutes),
 * CONFIG (0x08: ATHD, the alert threshold, and UFG), MODE (0x0a) and the
 * 64-byte battery profile (0x10). The part wakes up asleep (MODE 0xc0,
 * CONFIG 0x18) with its registers zero. The datasheet leaves the profile
 * to the vendor, who set UFG after loading it; the model keeps the bytes
 * and UFG as the part keeps them in flash, through a POR.
 *
 * Awake, the first sample comes 10 ms after the wake, a quick start or a
 * POR and the rest every 250 ms of virtual time: VCELL from "voltage-mv"
 * (below 2.5 V the part goes to sleep by itself), SOC from "soc" when it
 * is 0 to 100, else (-1) from the voltage on a typical lithium-ion
 * open-circuit curve, as the part first estimates it. A sleeping part keeps its
 * registers. ALRT is set when the SOC falls to ATHD or below, once until
 * the SOC has been above ATHD again, and only the host clears it; the
 * "alrt" output is the pin, low while ALRT is set. In MODE, sleep (7:6)
 * is 3 to sleep and 0 to wake, a quick start (5:4 = 3) restarts the
 * estimate and a POR (3:0 = 0xf) resets the part as if power were
 * removed, to asleep: neither reads back. RRT is whatever "rrt" says.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/core/irq.h"
#include "qapi/visitor.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_CW2015 "cw2015"
OBJECT_DECLARE_SIMPLE_TYPE(Cw2015State, CW2015)

#define CW_VERSION      0x00
#define CW_VCELL        0x02
#define CW_SOC          0x04
#define CW_RRT_ALRT     0x06
#define CW_CONFIG       0x08
#define CW_MODE         0x0a
#define CW_PROFILE      0x10
#define CW_PROFILE_LEN  64

#define CONFIG_ATHD     0xf8
#define CONFIG_UFG      BIT(1)
#define CONFIG_RESET    0x18
#define MODE_SLEEP      0xc0
#define MODE_QSTRT      0x30
#define MODE_POR        0x0f
#define ALRT            BIT(7)

#define FIRST_SAMPLE_MS 10
#define SAMPLE_MS       250
#define SLEEP_BELOW_UV  2500000

struct Cw2015State {
    I2CSlave parent_obj;

    QEMUTimer *timer;
    qemu_irq alrt_pin;
    uint8_t profile[CW_PROFILE_LEN];
    uint16_t vcell;
    uint16_t soc;
    uint8_t alrt;
    bool armed;
    uint8_t config;
    uint8_t mode;
    uint8_t pointer;
    bool first;                 /* the next byte is the pointer */

    uint32_t voltage_mv;
    int32_t soc_pct;            /* -1: from the voltage */
    uint32_t rrt;
};

/* A typical lithium-ion cell's open-circuit voltage against charge */
static const struct { uint16_t mv; uint8_t pct; } ocv[] = {
    { 3300, 0 }, { 3500, 5 }, { 3600, 12 }, { 3700, 30 }, { 3800, 50 },
    { 3900, 68 }, { 4000, 80 }, { 4100, 92 }, { 4200, 100 },
};

/* 1/256 percent */
static uint16_t cw_estimate(uint32_t mv)
{
    if (mv <= ocv[0].mv) {
        return 0;
    }
    for (int i = 1; i < ARRAY_SIZE(ocv); i++) {
        if (mv < ocv[i].mv) {
            uint32_t span = ocv[i].mv - ocv[i - 1].mv;
            uint32_t pct = ocv[i - 1].pct * 256 +
                           (ocv[i].pct - ocv[i - 1].pct) * 256 *
                           (mv - ocv[i - 1].mv) / span;

            return pct;
        }
    }
    return 100 * 256;
}

static void cw_alrt_update(Cw2015State *s)
{
    qemu_set_irq(s->alrt_pin, !s->alrt);
}

static void cw_sample(void *opaque)
{
    Cw2015State *s = opaque;
    uint32_t uv = s->voltage_mv * 1000;

    s->vcell = MIN(uv / 305, 0x3fff);
    s->soc = s->soc_pct >= 0 ? MIN(s->soc_pct, 100) * 256 :
             cw_estimate(s->voltage_mv);
    if ((s->soc >> 8) <= s->config >> 3) {
        if (s->armed) {
            s->alrt = ALRT;
            s->armed = false;
            cw_alrt_update(s);
        }
    } else {
        s->armed = true;
    }
    if (uv < SLEEP_BELOW_UV) {
        s->mode |= MODE_SLEEP;
        return;
    }
    timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SAMPLE_MS);
}

static void cw_power_up(Cw2015State *s, bool flash)
{
    s->vcell = 0;
    s->soc = 0;
    s->alrt = 0;
    s->armed = true;
    s->mode = MODE_SLEEP;
    if (!flash) {
        s->config = CONFIG_RESET;
        memset(s->profile, 0, sizeof(s->profile));
    }
    timer_del(s->timer);
    cw_alrt_update(s);
}

static uint8_t cw_read(Cw2015State *s, uint8_t reg)
{
    switch (reg) {
    case CW_VERSION:
        return 0x6f;
    case CW_VCELL:
        return s->vcell >> 8;
    case CW_VCELL + 1:
        return s->vcell & 0xff;
    case CW_SOC:
        return s->soc >> 8;
    case CW_SOC + 1:
        return s->soc & 0xff;
    case CW_RRT_ALRT:
        return s->alrt | extract32(s->rrt, 8, 5);
    case CW_RRT_ALRT + 1:
        return s->rrt & 0xff;
    case CW_CONFIG:
        return s->config;
    case CW_MODE:
        return s->mode;
    case CW_PROFILE ... CW_PROFILE + CW_PROFILE_LEN - 1:
        return s->profile[reg - CW_PROFILE];
    default:
        return 0;
    }
}

static void cw_write(Cw2015State *s, uint8_t reg, uint8_t v)
{
    bool woke;

    switch (reg) {
    case CW_RRT_ALRT:
        /* only the host clears ALRT, by writing 0 */
        s->alrt &= v & ALRT;
        cw_alrt_update(s);
        break;
    case CW_CONFIG:
        s->config = v & (CONFIG_ATHD | CONFIG_UFG);
        break;
    case CW_MODE:
        if ((v & MODE_POR) == MODE_POR) {
            cw_power_up(s, true);
            break;
        }
        woke = (s->mode & MODE_SLEEP) == MODE_SLEEP &&
               (v & MODE_SLEEP) != MODE_SLEEP;
        s->mode = v & MODE_SLEEP;
        if ((v & MODE_QSTRT) == MODE_QSTRT) {
            s->soc = 0;
            s->armed = true;
        }
        if ((s->mode & MODE_SLEEP) == MODE_SLEEP) {
            timer_del(s->timer);
        } else if (woke || (v & MODE_QSTRT) == MODE_QSTRT) {
            timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                      FIRST_SAMPLE_MS);
        }
        break;
    case CW_PROFILE ... CW_PROFILE + CW_PROFILE_LEN - 1:
        s->profile[reg - CW_PROFILE] = v;
        break;
    }
}

static int cw_event(I2CSlave *i2c, enum i2c_event event)
{
    Cw2015State *s = CW2015(i2c);

    if (event == I2C_START_SEND) {
        s->first = true;
    }
    return 0;
}

static uint8_t cw_recv(I2CSlave *i2c)
{
    Cw2015State *s = CW2015(i2c);

    return cw_read(s, s->pointer++);
}

static int cw_send(I2CSlave *i2c, uint8_t data)
{
    Cw2015State *s = CW2015(i2c);

    if (s->first) {
        s->pointer = data;
        s->first = false;
    } else {
        cw_write(s, s->pointer++, data);
    }
    return 0;
}

static void cw_reset_hold(Object *obj, ResetType type)
{
    Cw2015State *s = CW2015(obj);

    cw_power_up(s, false);
    s->pointer = 0;
    s->first = true;
}

static void cw_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);

    if (!i2c->address) {
        i2c->address = 0x62;
    }
}

static void cw_get_soc(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    int64_t soc = CW2015(obj)->soc_pct;

    visit_type_int(v, name, &soc, errp);
}

static void cw_set_soc(Object *obj, Visitor *v, const char *name,
                       void *opaque, Error **errp)
{
    int64_t soc;

    if (visit_type_int(v, name, &soc, errp)) {
        CW2015(obj)->soc_pct = soc < 0 || soc > 100 ? -1 : soc;
    }
}

static void cw_init(Object *obj)
{
    Cw2015State *s = CW2015(obj);

    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, cw_sample, s);
    qdev_init_gpio_out_named(DEVICE(obj), &s->alrt_pin, "alrt", 1);
    s->voltage_mv = 4000;
    s->soc_pct = -1;
    object_property_add_uint32_ptr(obj, "voltage-mv", &s->voltage_mv,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add_uint32_ptr(obj, "rrt", &s->rrt,
                                   OBJ_PROP_FLAG_READWRITE);
    object_property_add(obj, "soc", "int", cw_get_soc, cw_set_soc, NULL, s);
}

static void cw_finalize(Object *obj)
{
    timer_free(CW2015(obj)->timer);
}

static const VMStateDescription vmstate_cw = {
    .name = TYPE_CW2015,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Cw2015State),
        VMSTATE_TIMER_PTR(timer, Cw2015State),
        VMSTATE_UINT8_ARRAY(profile, Cw2015State, CW_PROFILE_LEN),
        VMSTATE_UINT16(vcell, Cw2015State),
        VMSTATE_UINT16(soc, Cw2015State),
        VMSTATE_UINT8(alrt, Cw2015State),
        VMSTATE_BOOL(armed, Cw2015State),
        VMSTATE_UINT8(config, Cw2015State),
        VMSTATE_UINT8(mode, Cw2015State),
        VMSTATE_UINT8(pointer, Cw2015State),
        VMSTATE_BOOL(first, Cw2015State),
        VMSTATE_END_OF_LIST()
    }
};

static void cw_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "CellWise CW2015 battery fuel gauge";
    dc->realize = cw_realize;
    dc->vmsd = &vmstate_cw;
    rc->phases.hold = cw_reset_hold;
    k->event = cw_event;
    k->recv = cw_recv;
    k->send = cw_send;
}

static const TypeInfo cw_info = {
    .name = TYPE_CW2015,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Cw2015State),
    .instance_init = cw_init,
    .instance_finalize = cw_finalize,
    .class_init = cw_class_init,
};

static void cw_register_types(void)
{
    type_register_static(&cw_info);
}

type_init(cw_register_types)
