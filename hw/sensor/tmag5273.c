/*
 * Texas Instruments TMAG5273 3D Hall-effect sensor
 *
 * From the datasheet (SLYS045C): 8-bit registers behind a pointer that
 * steps on each byte, a write's first byte being a trigger bit (7) and the
 * register (6:0). DEVICE_CONFIG_1/2, SENSOR_CONFIG_1/2, the thresholds,
 * T_CONFIG, INT_CONFIG_1, the gain and offsets are plain registers;
 * I2C_ADDRESS (0x0c) moves the part when its update bit is set; DEVICE_ID
 * (VER 1: 40/80 mT, A1; 2: 133/266 mT, A2), MANUFACTURER_ID ("TI",
 * 0x5449), the results and the status are read-only. A conversion in
 * standby comes with the trigger bit, in continuous (OPERATING_MODE 2) or
 * wake-up and sleep mode (3) with every transaction, and fills the
 * enabled channels from the "x", "y", "z" (mT) and "temperature" (deg C)
 * properties: B = code * range / 32768 (eq. 10), T = 25 + (code - 17508) /
 * 58 (eq. 12). Sleep mode drops the results and the next transaction wakes
 * the part to standby (6.4.2). CONV_STATUS counts the data sets (7:5), has
 * POR (4, write 1 to clear) and RESULT_STATUS (0); DEVICE_STATUS reads
 * its reset value.
 *
 * Not from the datasheet, assumed: the angle is atan2 of the second
 * channel over the first (the datasheet gives the 13-bit format, 9 bits of
 * degrees and 4 of 1/16, not the formula), MAGNITUDE_RESULT is the top 8
 * bits of the 16-bit vector length (eq. 15 does not say), and one
 * conversion happens per transaction rather than on the conversion time.
 * Not modelled: the I2C_RD single-byte read frames and CRC (both read as
 * the standard frame), the thresholds and the INT pin, the gain and
 * offset corrections, W&S timing, the 8-bit data format.
 *
 * What the stock eufy T86P2 firmware does with it (from its tmag5273
 * driver): writes DEVICE_CONFIG_1 0x34, DEVICE_CONFIG_2 0x12 (low noise,
 * continuous), SENSOR_CONFIG_1 0x30 (X, Y), SENSOR_CONFIG_2 0x07 (X-Y
 * angle, 80 mT) and INT_CONFIG_1 1 at probe, and reads the angle word
 * (0x19) on demand, taking its top 13 bits >> 4 as degrees. It never reads
 * the ID registers.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/qdev-properties.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"
#include <math.h>

#define TYPE_TMAG5273 "tmag5273"
OBJECT_DECLARE_SIMPLE_TYPE(Tmag5273State, TMAG5273)

#define R_DEVICE_CONFIG_1   0x00
#define R_DEVICE_CONFIG_2   0x01
#define R_SENSOR_CONFIG_1   0x02
#define R_SENSOR_CONFIG_2   0x03
#define R_T_CONFIG          0x07
#define R_I2C_ADDRESS       0x0c
#define R_DEVICE_ID         0x0d
#define R_MANUF_ID_LSB      0x0e
#define R_MANUF_ID_MSB      0x0f
#define R_T_MSB             0x10
#define R_X_MSB             0x12
#define R_Y_MSB             0x14
#define R_Z_MSB             0x16
#define R_CONV_STATUS       0x18
#define R_ANGLE_MSB         0x19
#define R_ANGLE_LSB         0x1a
#define R_MAGNITUDE         0x1b
#define R_DEVICE_STATUS     0x1c
#define NUM_CONFIG          0x0d        /* the registers 0x00 to 0x0c */

#define DC2_MODE            0x03
#define MODE_STANDBY        0
#define MODE_SLEEP          1
#define MODE_CONTINUOUS     2
#define MODE_WAKE_SLEEP     3

#define SC2_ANGLE_EN        0x0c
#define SC2_X_Y_RANGE       BIT(1)
#define SC2_Z_RANGE         BIT(0)
#define T_CH_EN             BIT(0)
#define I2C_ADDRESS_UPDATE  BIT(0)
#define CONV_POR            BIT(4)
#define CONV_RESULT         BIT(0)
#define DEVICE_STATUS_RESET 0x10

#define MANUF_ID            0x5449
#define TADC_T0             17508
#define TADC_RES            58
#define TRIGGER             0x80

struct Tmag5273State {
    I2CSlave parent_obj;

    uint8_t regs[NUM_CONFIG];
    uint8_t conv_status;
    uint16_t t, x, y, z;
    uint16_t angle;
    uint8_t magnitude;
    uint8_t pointer;
    bool first;                 /* the next byte is trigger and pointer */

    uint8_t version;
    int32_t ut[3];              /* the field in microtesla */
    int32_t millic;             /* the temperature in milli degrees C */
};

static int16_t clamp16(double v)
{
    return MAX(MIN(lround(v), INT16_MAX), INT16_MIN);
}

/* The 16-bit code of a field in uT: full scale is the channel's range */
static int16_t field_code(Tmag5273State *s, int axis)
{
    int base = s->version == 2 ? 133 : 40;
    bool wide = axis == 2 ? s->regs[R_SENSOR_CONFIG_2] & SC2_Z_RANGE :
                            s->regs[R_SENSOR_CONFIG_2] & SC2_X_Y_RANGE;

    return clamp16(s->ut[axis] / 1000.0 * 32768 / (base * (1 + wide)));
}

static void tmag_convert(Tmag5273State *s)
{
    uint8_t sc1 = s->regs[R_SENSOR_CONFIG_1];
    int en = sc1 >> 4;
    bool ch[3] = { false };
    int first, second;
    double a, b;

    switch (en) {
    case 0x1 ... 0x7:
        ch[0] = en & 1;
        ch[1] = en & 2;
        ch[2] = en & 4;
        break;
    case 0x8:
    case 0x9:
        ch[0] = ch[1] = true;
        break;
    case 0xa:
        ch[1] = ch[2] = true;
        break;
    case 0xb:
        ch[0] = ch[2] = true;
        break;
    }
    if (ch[0]) {
        s->x = field_code(s, 0);
    }
    if (ch[1]) {
        s->y = field_code(s, 1);
    }
    if (ch[2]) {
        s->z = field_code(s, 2);
    }
    if (s->regs[R_T_CONFIG] & T_CH_EN) {
        s->t = clamp16(TADC_T0 + (s->millic / 1000.0 - 25) * TADC_RES);
    }

    switch ((s->regs[R_SENSOR_CONFIG_2] & SC2_ANGLE_EN) >> 2) {
    case 1:
        first = 0;
        second = 1;
        break;
    case 2:
        first = 1;
        second = 2;
        break;
    case 3:
        first = 0;
        second = 2;
        break;
    default:
        first = -1;
        second = -1;
        break;
    }
    if (first >= 0) {
        a = (int16_t)(first == 0 ? s->x : s->y);
        b = (int16_t)(second == 1 ? s->y : s->z);
        double deg = atan2(b, a) * 180.0 / M_PI;

        if (deg < 0) {
            deg += 360;
        }
        s->angle = (unsigned)lround(deg * 16) % (360 * 16);
        s->magnitude = MIN(hypot(a, b) / 256, 255);
    }
    s->conv_status = ((((s->conv_status >> 5) + 1) & 7) << 5) |
                     (s->conv_status & 0x1f) | CONV_RESULT;
}

static void tmag_wake(Tmag5273State *s)
{
    if ((s->regs[R_DEVICE_CONFIG_2] & DC2_MODE) == MODE_SLEEP) {
        s->regs[R_DEVICE_CONFIG_2] &= ~DC2_MODE;
        s->t = s->x = s->y = s->z = s->angle = 0;
        s->magnitude = 0;
        s->conv_status &= ~CONV_RESULT;
    }
}

static void tmag_power_up(Tmag5273State *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[R_I2C_ADDRESS] = I2C_SLAVE(s)->address << 1;
    s->conv_status = CONV_POR;
    s->t = s->x = s->y = s->z = s->angle = 0;
    s->magnitude = 0;
}

static uint8_t tmag_read(Tmag5273State *s, uint8_t reg)
{
    switch (reg) {
    case 0 ... R_I2C_ADDRESS:
        return s->regs[reg];
    case R_DEVICE_ID:
        return s->version;
    case R_MANUF_ID_LSB:
        return MANUF_ID & 0xff;
    case R_MANUF_ID_MSB:
        return MANUF_ID >> 8;
    case R_T_MSB:
        return s->t >> 8;
    case R_T_MSB + 1:
        return s->t;
    case R_X_MSB:
        return s->x >> 8;
    case R_X_MSB + 1:
        return s->x;
    case R_Y_MSB:
        return s->y >> 8;
    case R_Y_MSB + 1:
        return s->y;
    case R_Z_MSB:
        return s->z >> 8;
    case R_Z_MSB + 1:
        return s->z;
    case R_CONV_STATUS:
        return s->conv_status;
    case R_ANGLE_MSB:
        return s->angle >> 8;
    case R_ANGLE_LSB:
        return s->angle;
    case R_MAGNITUDE:
        return s->magnitude;
    case R_DEVICE_STATUS:
        return DEVICE_STATUS_RESET;
    default:
        return 0;
    }
}

static void tmag_write(Tmag5273State *s, uint8_t reg, uint8_t v)
{
    switch (reg) {
    case 0 ... R_I2C_ADDRESS - 1:
        s->regs[reg] = v;
        if (reg == R_DEVICE_CONFIG_2 &&
            (v & DC2_MODE) == MODE_SLEEP) {
            s->t = s->x = s->y = s->z = s->angle = 0;
            s->magnitude = 0;
            s->conv_status &= ~CONV_RESULT;
        }
        break;
    case R_I2C_ADDRESS:
        s->regs[reg] = v;
        if (v & I2C_ADDRESS_UPDATE) {
            i2c_slave_set_address(I2C_SLAVE(s), v >> 1);
        }
        break;
    case R_CONV_STATUS:
        s->conv_status &= ~(v & CONV_POR);
        break;
    }
}

static int tmag_event(I2CSlave *i2c, enum i2c_event event)
{
    Tmag5273State *s = TMAG5273(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->first = true;
        /* fall through */
    case I2C_START_RECV:
        tmag_wake(s);
        if (s->regs[R_DEVICE_CONFIG_2] & 2) {
            tmag_convert(s);
        }
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t tmag_recv(I2CSlave *i2c)
{
    Tmag5273State *s = TMAG5273(i2c);
    uint8_t reg = s->pointer++;
    uint8_t v = tmag_read(s, reg);

    trace_tmag5273_read(reg, v);
    return v;
}

static int tmag_send(I2CSlave *i2c, uint8_t data)
{
    Tmag5273State *s = TMAG5273(i2c);

    if (s->first) {
        s->first = false;
        s->pointer = data & ~TRIGGER;
        if ((data & TRIGGER) &&
            (s->regs[R_DEVICE_CONFIG_2] & DC2_MODE) == MODE_STANDBY) {
            tmag_convert(s);
        }
    } else {
        trace_tmag5273_write(s->pointer, data);
        tmag_write(s, s->pointer++, data);
    }
    return 0;
}

static void tmag_reset_hold(Object *obj, ResetType type)
{
    Tmag5273State *s = TMAG5273(obj);

    tmag_power_up(s);
    s->pointer = 0;
    s->first = true;
}

static void tmag_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);
    Tmag5273State *s = TMAG5273(dev);

    if (s->version != 1 && s->version != 2) {
        error_setg(errp, "%s: version must be 1 (A1) or 2 (A2)",
                   TYPE_TMAG5273);
        return;
    }
    if (!i2c->address) {
        i2c->address = 0x35;
    }
}

static void tmag_get_ut(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    int32_t *ut = opaque;
    double mt = *ut / 1000.0;

    visit_type_number(v, name, &mt, errp);
}

/* x, y and z in mT, up to the widest range's 266 mT */
static void tmag_set_ut(Object *obj, Visitor *v, const char *name,
                        void *opaque, Error **errp)
{
    int32_t *ut = opaque;
    double mt;

    if (!visit_type_number(v, name, &mt, errp)) {
        return;
    }
    if (!(mt >= -266 && mt <= 266)) {
        error_setg(errp, "%s: %s must be from -266 to 266 mT", TYPE_TMAG5273,
                   name);
        return;
    }
    *ut = lround(mt * 1000);
}

static void tmag_get_temp(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    double c = TMAG5273(obj)->millic / 1000.0;

    visit_type_number(v, name, &c, errp);
}

static void tmag_set_temp(Object *obj, Visitor *v, const char *name,
                          void *opaque, Error **errp)
{
    double c;

    if (!visit_type_number(v, name, &c, errp)) {
        return;
    }
    if (!(c >= -40 && c <= 170)) {
        error_setg(errp, "%s: temperature must be from -40 to 170 deg C",
                   TYPE_TMAG5273);
        return;
    }
    TMAG5273(obj)->millic = lround(c * 1000);
}

static const Property tmag_props[] = {
    DEFINE_PROP_UINT8("version", Tmag5273State, version, 1),
};

static void tmag_init(Object *obj)
{
    Tmag5273State *s = TMAG5273(obj);

    s->millic = 25000;
    object_property_add(obj, "x", "number", tmag_get_ut, tmag_set_ut, NULL,
                        &s->ut[0]);
    object_property_add(obj, "y", "number", tmag_get_ut, tmag_set_ut, NULL,
                        &s->ut[1]);
    object_property_add(obj, "z", "number", tmag_get_ut, tmag_set_ut, NULL,
                        &s->ut[2]);
    object_property_add(obj, "temperature", "number", tmag_get_temp,
                        tmag_set_temp, NULL, NULL);
}

static const VMStateDescription vmstate_tmag = {
    .name = TYPE_TMAG5273,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Tmag5273State),
        VMSTATE_UINT8_ARRAY(regs, Tmag5273State, NUM_CONFIG),
        VMSTATE_UINT8(conv_status, Tmag5273State),
        VMSTATE_UINT16(t, Tmag5273State),
        VMSTATE_UINT16(x, Tmag5273State),
        VMSTATE_UINT16(y, Tmag5273State),
        VMSTATE_UINT16(z, Tmag5273State),
        VMSTATE_UINT16(angle, Tmag5273State),
        VMSTATE_UINT8(magnitude, Tmag5273State),
        VMSTATE_UINT8(pointer, Tmag5273State),
        VMSTATE_BOOL(first, Tmag5273State),
        VMSTATE_INT32_ARRAY(ut, Tmag5273State, 3),
        VMSTATE_INT32(millic, Tmag5273State),
        VMSTATE_END_OF_LIST()
    }
};

static void tmag_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Texas Instruments TMAG5273 3D Hall-effect sensor";
    dc->realize = tmag_realize;
    dc->vmsd = &vmstate_tmag;
    device_class_set_props(dc, tmag_props);
    rc->phases.hold = tmag_reset_hold;
    k->event = tmag_event;
    k->recv = tmag_recv;
    k->send = tmag_send;
}

static const TypeInfo tmag_info = {
    .name = TYPE_TMAG5273,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Tmag5273State),
    .instance_init = tmag_init,
    .class_init = tmag_class_init,
};

static void tmag_register_types(void)
{
    type_register_static(&tmag_info);
}

type_init(tmag_register_types)
