/*
 * Lite-On LTR-308ALS ambient light sensor (0x53)
 *
 * Eight-bit registers behind a pointer that steps on each byte, as the
 * LTR-F216A datasheet (BNS-OD-FC002/A4) lists them: MAIN_CTRL (0x00, ALS
 * enable 1, software reset 4), ALS_MEAS_RES (0x04, reset 0x22: resolution
 * 6:4, 20 to 16 bits in 400 to 25 ms, and rate 2:0, 25, 50, 100, 500 ms),
 * ALS_GAIN (0x05, reset 1: 1, 3, 6, 9, 18x), PART_ID (0x06, 0xb1),
 * MAIN_STATUS (0x07, reset 0x20: power-on 5, interrupt 4, new data 3, all
 * cleared by a read), ALS_DATA (0x0d to 0x0f, 20 bits, little endian),
 * INT_CFG (0x19, 0x10), INT_PST (0x1a), and the upper and lower
 * thresholds (0x21 to 0x26, 0xfffff and 0). The map is the family's; the
 * LTR-308ALS-01 datasheet could not be read, so what it shares with the
 * LTR-F216A is assumed, and the F216A's clear channel (0x0a to 0x0c) reads
 * 0 here.
 *
 * Enabling the ALS starts conversions after 5 ms; one finishes every
 * resolution's conversion time, or the rate if that is longer, and gives
 * the "lux" property as counts: lux * GAIN * INT / 0.45, INT 4 to 0.25
 * for 20 to 16 bits, the datasheet's clear-window formula (0.45 is the
 * F216A's, the LTR-308ALS-01's may differ), up to the resolution's full
 * scale. "counts", when not -1, gives the counts directly. A conversion
 * out of the thresholds INT_PST + 1 times running sets the interrupt flag
 * if the INT_CFG enable (2) is on. While a read is in the registers 0x07
 * to 0x18 a result waits for it to end (the datasheet's lock). The INT pin
 * is not modelled.
 *
 * What the stock eufy T8160's firmware does with it: reads 0x0d, 0x0e and
 * 0x0f through /dev/i2c-2 for a raw count, which it compares with 300000
 * and 500000 for day and night; it writes nothing and never reads the ID,
 * so the part stays in standby (MAIN_CTRL 0) and the counts read 0 unless
 * the "active" property starts the ALS.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_LTR308ALS "ltr308als"
OBJECT_DECLARE_SIMPLE_TYPE(Ltr308AlsState, LTR308ALS)

#define R_MAIN_CTRL     0x00
#define R_MEAS_RES      0x04
#define R_GAIN          0x05
#define R_PART_ID       0x06
#define R_MAIN_STATUS   0x07
#define R_DATA_0        0x0d
#define R_DATA_2        0x0f
#define R_INT_CFG       0x19
#define R_INT_PST       0x1a
#define R_THRES_UP_0    0x21
#define R_THRES_LOW_0   0x24
#define R_THRES_LOW_2   0x26
#define LOCK_FIRST      0x07
#define LOCK_LAST       0x18

#define CTRL_ALS_EN     BIT(1)
#define CTRL_SW_RESET   BIT(4)
#define ST_POWER_ON     BIT(5)
#define ST_INT          BIT(4)
#define ST_NEW_DATA     BIT(3)
#define INT_PIN_EN      BIT(2)

#define PART_ID         0xb1
#define START_MS        5

struct Ltr308AlsState {
    I2CSlave parent_obj;

    QEMUTimer *timer;
    uint8_t ctrl, meas, gain, status, int_cfg, int_pst;
    uint32_t up, low;
    uint32_t data;              /* what the registers show */
    uint32_t pending;           /* a result held back by the lock */
    bool has_pending;
    bool locked;
    uint8_t outside;            /* conversions in a row out of the range */
    uint8_t pointer;
    bool first;

    uint32_t millilux;
    int32_t counts;             /* -1: from the lux */
    bool active;
};

static const uint16_t conv_ms[] = { 400, 200, 100, 50, 25 };
static const uint16_t rate_ms[] = { 25, 50, 100, 500 };
static const uint8_t gains[] = { 1, 3, 6, 9, 18 };

static unsigned res_index(Ltr308AlsState *s)
{
    return MIN((s->meas >> 4) & 7, 4);
}

static uint32_t period_ms(Ltr308AlsState *s)
{
    return MAX(conv_ms[res_index(s)], rate_ms[s->meas & 3]);
}

static uint32_t als_counts(Ltr308AlsState *s)
{
    unsigned r = res_index(s);
    uint32_t full = (1u << (20 - r)) - 1;
    double intg = 4.0 / (1 << r);
    double n;

    if (s->counts >= 0) {
        return MIN(s->counts, full);
    }
    n = s->millilux / 1000.0 * gains[MIN(s->gain & 7, 4)] * intg / 0.45;
    return MIN(n + 0.5, full);
}

static void als_publish(Ltr308AlsState *s, uint32_t v)
{
    s->data = v;
    s->status |= ST_NEW_DATA;
    if (v > s->up || v < s->low) {
        if (s->outside < 0xff) {
            s->outside++;
        }
        if (s->outside > (s->int_pst >> 4) && (s->int_cfg & INT_PIN_EN)) {
            s->status |= ST_INT;
        }
    } else {
        s->outside = 0;
    }
}

static void als_timer(void *opaque)
{
    Ltr308AlsState *s = opaque;
    uint32_t v = als_counts(s);

    if (s->locked) {
        s->pending = v;
        s->has_pending = true;
    } else {
        als_publish(s, v);
    }
    if (s->ctrl & CTRL_ALS_EN) {
        timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                  period_ms(s));
    }
}

static void als_unlock(Ltr308AlsState *s)
{
    s->locked = false;
    if (s->has_pending) {
        s->has_pending = false;
        als_publish(s, s->pending);
    }
}

static void als_restart(Ltr308AlsState *s)
{
    timer_del(s->timer);
    if (s->ctrl & CTRL_ALS_EN) {
        timer_mod(s->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + START_MS +
                  conv_ms[res_index(s)]);
    }
}

static void als_defaults(Ltr308AlsState *s)
{
    s->ctrl = 0;
    s->meas = 0x22;
    s->gain = 1;
    s->status = ST_POWER_ON;
    s->int_cfg = 0x10;
    s->int_pst = 0;
    s->up = 0xfffff;
    s->low = 0;
    s->data = 0;
    s->has_pending = false;
    s->locked = false;
    s->outside = 0;
    timer_del(s->timer);
}

static uint8_t als_read(Ltr308AlsState *s, uint8_t reg)
{
    uint8_t v;

    switch (reg) {
    case R_MAIN_CTRL:
        return s->ctrl;
    case R_MEAS_RES:
        return s->meas;
    case R_GAIN:
        return s->gain;
    case R_PART_ID:
        return PART_ID;
    case R_MAIN_STATUS:
        v = s->status;
        s->status = 0;
        return v;
    case R_DATA_0 ... R_DATA_2:
        return s->data >> (8 * (reg - R_DATA_0));
    case R_INT_CFG:
        return s->int_cfg;
    case R_INT_PST:
        return s->int_pst;
    case R_THRES_UP_0 ... R_THRES_UP_0 + 2:
        return s->up >> (8 * (reg - R_THRES_UP_0));
    case R_THRES_LOW_0 ... R_THRES_LOW_2:
        return s->low >> (8 * (reg - R_THRES_LOW_0));
    default:
        return 0;
    }
}

static void als_write(Ltr308AlsState *s, uint8_t reg, uint8_t v)
{
    uint32_t sh;

    switch (reg) {
    case R_MAIN_CTRL:
        if (v & CTRL_SW_RESET) {
            als_defaults(s);
            break;
        }
        s->ctrl = v & CTRL_ALS_EN;
        als_restart(s);
        break;
    case R_MEAS_RES:
        s->meas = v & 0x77;
        break;
    case R_GAIN:
        s->gain = v & 7;
        break;
    case R_INT_CFG:
        s->int_cfg = v & 0x34;
        break;
    case R_INT_PST:
        s->int_pst = v & 0xf0;
        break;
    case R_THRES_UP_0 ... R_THRES_UP_0 + 2:
        sh = 8 * (reg - R_THRES_UP_0);
        s->up = deposit32(s->up, sh, 8, reg == R_THRES_UP_0 + 2 ? v & 0xf : v);
        break;
    case R_THRES_LOW_0 ... R_THRES_LOW_2:
        sh = 8 * (reg - R_THRES_LOW_0);
        s->low = deposit32(s->low, sh, 8, reg == R_THRES_LOW_2 ? v & 0xf : v);
        break;
    }
}

static int als_event(I2CSlave *i2c, enum i2c_event event)
{
    Ltr308AlsState *s = LTR308ALS(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->first = true;
        break;
    case I2C_FINISH:
        als_unlock(s);
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t als_recv(I2CSlave *i2c)
{
    Ltr308AlsState *s = LTR308ALS(i2c);
    uint8_t reg = s->pointer++;

    if (reg >= LOCK_FIRST && reg <= LOCK_LAST) {
        s->locked = true;
    } else {
        als_unlock(s);
    }
    return als_read(s, reg);
}

static int als_send(I2CSlave *i2c, uint8_t data)
{
    Ltr308AlsState *s = LTR308ALS(i2c);

    if (s->first) {
        s->pointer = data;
        s->first = false;
    } else {
        als_write(s, s->pointer++, data);
    }
    return 0;
}

static void als_reset_hold(Object *obj, ResetType type)
{
    Ltr308AlsState *s = LTR308ALS(obj);

    als_defaults(s);
    s->pointer = 0;
    s->first = true;
    if (s->active) {
        s->ctrl = CTRL_ALS_EN;
        als_restart(s);
    }
}

static void als_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);

    if (!i2c->address) {
        i2c->address = 0x53;
    }
}

static void als_get_lux(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    double lux = LTR308ALS(obj)->millilux / 1000.0;

    visit_type_number(v, name, &lux, errp);
}

static void als_set_lux(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    double lux;

    if (!visit_type_number(v, name, &lux, errp)) {
        return;
    }
    if (!(lux >= 0 && lux <= UINT32_MAX / 1000)) {
        error_setg(errp, "%s: lux must be from 0 to %u", TYPE_LTR308ALS,
                   UINT32_MAX / 1000);
        return;
    }
    LTR308ALS(obj)->millilux = lux * 1000 + 0.5;
}

static void als_get_counts(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    int64_t c = LTR308ALS(obj)->counts;

    visit_type_int(v, name, &c, errp);
}

static void als_set_counts(Object *obj, Visitor *v, const char *name,
                           void *opaque, Error **errp)
{
    int64_t c;

    if (!visit_type_int(v, name, &c, errp)) {
        return;
    }
    if (c < -1 || c > 0xfffff) {
        error_setg(errp, "%s: counts must be -1 or from 0 to 1048575",
                   TYPE_LTR308ALS);
        return;
    }
    LTR308ALS(obj)->counts = c;
}

static const Property als_props[] = {
    DEFINE_PROP_BOOL("active", Ltr308AlsState, active, false),
};

static void als_init(Object *obj)
{
    Ltr308AlsState *s = LTR308ALS(obj);

    s->millilux = 100000;
    s->counts = -1;
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, als_timer, s);
    object_property_add(obj, "lux", "number", als_get_lux, als_set_lux, NULL,
                        NULL);
    object_property_add(obj, "counts", "int", als_get_counts, als_set_counts,
                        NULL, NULL);
}

static void als_finalize(Object *obj)
{
    timer_free(LTR308ALS(obj)->timer);
}

static const VMStateDescription vmstate_als = {
    .name = TYPE_LTR308ALS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Ltr308AlsState),
        VMSTATE_TIMER_PTR(timer, Ltr308AlsState),
        VMSTATE_UINT8(ctrl, Ltr308AlsState),
        VMSTATE_UINT8(meas, Ltr308AlsState),
        VMSTATE_UINT8(gain, Ltr308AlsState),
        VMSTATE_UINT8(status, Ltr308AlsState),
        VMSTATE_UINT8(int_cfg, Ltr308AlsState),
        VMSTATE_UINT8(int_pst, Ltr308AlsState),
        VMSTATE_UINT32(up, Ltr308AlsState),
        VMSTATE_UINT32(low, Ltr308AlsState),
        VMSTATE_UINT32(data, Ltr308AlsState),
        VMSTATE_UINT32(pending, Ltr308AlsState),
        VMSTATE_BOOL(has_pending, Ltr308AlsState),
        VMSTATE_BOOL(locked, Ltr308AlsState),
        VMSTATE_UINT8(outside, Ltr308AlsState),
        VMSTATE_UINT8(pointer, Ltr308AlsState),
        VMSTATE_BOOL(first, Ltr308AlsState),
        VMSTATE_UINT32(millilux, Ltr308AlsState),
        VMSTATE_INT32(counts, Ltr308AlsState),
        VMSTATE_END_OF_LIST()
    }
};

static void als_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Lite-On LTR-308ALS ambient light sensor";
    dc->realize = als_realize;
    dc->vmsd = &vmstate_als;
    device_class_set_props(dc, als_props);
    rc->phases.hold = als_reset_hold;
    k->event = als_event;
    k->recv = als_recv;
    k->send = als_send;
}

static const TypeInfo als_info = {
    .name = TYPE_LTR308ALS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Ltr308AlsState),
    .instance_init = als_init,
    .instance_finalize = als_finalize,
    .class_init = als_class_init,
};

static void als_register_types(void)
{
    type_register_static(&als_info);
}

type_init(als_register_types)
