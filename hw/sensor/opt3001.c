/*
 * Texas Instruments OPT3001 ambient light sensor
 *
 * Sixteen-bit registers, big endian, behind a register pointer: the
 * result, the configuration, the low and high limits and the two IDs. A
 * conversion takes 100 or 800 ms (CT) of virtual time, once (M = 01) or
 * over and over (M = 10 or 11), and leaves the light, "lux" (settable at
 * run time), in the result as lux = 0.01 * 2^E * R: E the smallest that
 * fits R in 12 bits in the automatic range (RN 0xc), else RN. Its end sets
 * CRF, and FH or FL when the result is above the high limit or below the
 * low one. A read of the configuration clears CRF and, latched (L), FH and
 * FL; a write that leaves the part converting clears CRF. It answers at
 * 0x44 (ADDR to ground) unless given an address. The INT pin and the
 * fault count are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_OPT3001 "opt3001"
OBJECT_DECLARE_SIMPLE_TYPE(Opt3001State, OPT3001)

#define OPT3001_RESULT      0x00
#define OPT3001_CONFIG      0x01
#define OPT3001_LOW         0x02
#define OPT3001_HIGH        0x03
#define OPT3001_MANUF_ID    0x7e
#define OPT3001_DEVICE_ID   0x7f

#define CONFIG_CT           BIT(11)
#define CONFIG_OVF          BIT(8)
#define CONFIG_CRF          BIT(7)
#define CONFIG_FH           BIT(6)
#define CONFIG_FL           BIT(5)
#define CONFIG_L            BIT(4)
#define CONFIG_FLAGS        (CONFIG_OVF | CONFIG_CRF | CONFIG_FH | CONFIG_FL)
#define CONFIG_RESET        0xc810

struct Opt3001State {
    I2CSlave parent_obj;

    QEMUTimer *timer;
    uint32_t millilux;
    uint16_t result;
    uint16_t config;
    uint16_t low;
    uint16_t high;
    uint16_t value;             /* the register a read sends */
    uint8_t pointer;
    uint8_t data[2];
    uint8_t len;                /* bytes of this transfer so far */
};

static unsigned opt3001_mode(Opt3001State *s)
{
    return extract16(s->config, 9, 2);
}

/* A result or limit in hundredths of a lux */
static uint32_t opt3001_centilux(uint16_t v)
{
    return extract16(v, 0, 12) << extract16(v, 12, 4);
}

static void opt3001_start(Opt3001State *s)
{
    int64_t ms = s->config & CONFIG_CT ? 800 : 100;

    timer_mod(s->timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + ms);
}

static void opt3001_convert(void *opaque)
{
    Opt3001State *s = opaque;
    uint32_t r = s->millilux / 10 + (s->millilux % 10 >= 5);
    unsigned rn = extract16(s->config, 12, 4), e = rn;
    uint32_t res;

    if (rn >= 0xc) {
        for (e = 0; e < 11 && r >> e > 0xfff; e++) {
            continue;
        }
    }
    s->config &= ~CONFIG_OVF;
    if (r >> e > 0xfff) {
        s->config |= CONFIG_OVF;
        r = 0xfff << e;
    }
    s->result = e << 12 | r >> e;

    res = opt3001_centilux(s->result);
    if (!(s->config & CONFIG_L)) {
        s->config &= ~(CONFIG_FH | CONFIG_FL);
    }
    if (res > opt3001_centilux(s->high)) {
        s->config |= CONFIG_FH;
    }
    if (res < opt3001_centilux(s->low)) {
        s->config |= CONFIG_FL;
    }
    s->config |= CONFIG_CRF;

    if (opt3001_mode(s) == 1) {
        s->config = deposit32(s->config, 9, 2, 0);
    } else if (opt3001_mode(s)) {
        opt3001_start(s);
    }
}

static uint16_t opt3001_read(Opt3001State *s, uint8_t reg)
{
    uint16_t v;

    switch (reg) {
    case OPT3001_RESULT:
        return s->result;
    case OPT3001_CONFIG:
        v = s->config;
        s->config &= ~(CONFIG_CRF |
                       (s->config & CONFIG_L ? CONFIG_FH | CONFIG_FL : 0));
        return v;
    case OPT3001_LOW:
        return s->low;
    case OPT3001_HIGH:
        return s->high;
    case OPT3001_MANUF_ID:
        return 0x5449;
    case OPT3001_DEVICE_ID:
        return 0x3001;
    default:
        return 0;
    }
}

static void opt3001_write(Opt3001State *s, uint8_t reg, uint16_t v)
{
    unsigned was = opt3001_mode(s);

    switch (reg) {
    case OPT3001_CONFIG:
        s->config = (s->config & CONFIG_FLAGS) | (v & ~CONFIG_FLAGS);
        if (!opt3001_mode(s)) {
            timer_del(s->timer);
            break;
        }
        s->config &= ~CONFIG_CRF;
        if (!was) {
            opt3001_start(s);
        }
        break;
    case OPT3001_LOW:
        s->low = v;
        break;
    case OPT3001_HIGH:
        s->high = v;
        break;
    default:
        break;
    }
}

static int opt3001_event(I2CSlave *i2c, enum i2c_event event)
{
    Opt3001State *s = OPT3001(i2c);

    switch (event) {
    case I2C_START_SEND:
        s->len = 0;
        break;
    case I2C_START_RECV:
        s->len = 0;
        s->value = opt3001_read(s, s->pointer);
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t opt3001_recv(I2CSlave *i2c)
{
    Opt3001State *s = OPT3001(i2c);

    /* reads go on giving the same register */
    return s->len++ & 1 ? s->value & 0xff : s->value >> 8;
}

static int opt3001_send(I2CSlave *i2c, uint8_t data)
{
    Opt3001State *s = OPT3001(i2c);

    if (s->len == 0) {
        s->pointer = data;
    } else if (s->len <= 2) {
        s->data[s->len - 1] = data;
        if (s->len == 2) {
            opt3001_write(s, s->pointer, s->data[0] << 8 | s->data[1]);
        }
    }
    if (s->len < 3) {
        s->len++;
    }
    return 0;
}

static void opt3001_get_lux(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    double lux = OPT3001(obj)->millilux / 1000.0;

    visit_type_number(v, name, &lux, errp);
}

static void opt3001_set_lux(Object *obj, Visitor *v, const char *name,
                            void *opaque, Error **errp)
{
    double lux;

    if (!visit_type_number(v, name, &lux, errp)) {
        return;
    }
    if (!(lux >= 0 && lux <= UINT32_MAX / 1000)) {
        error_setg(errp, "%s: lux must be from 0 to %u", TYPE_OPT3001,
                   UINT32_MAX / 1000);
        return;
    }
    OPT3001(obj)->millilux = lux * 1000 + 0.5;
}

static void opt3001_reset_hold(Object *obj, ResetType type)
{
    Opt3001State *s = OPT3001(obj);

    timer_del(s->timer);
    s->result = 0;
    s->config = CONFIG_RESET;
    s->low = 0xc000;
    s->high = 0xbfff;
    s->value = 0;
    s->pointer = 0;
    s->len = 0;
}

static void opt3001_realize(DeviceState *dev, Error **errp)
{
    I2CSlave *i2c = I2C_SLAVE(dev);

    if (!i2c->address) {
        i2c->address = 0x44;
    }
}

static void opt3001_init(Object *obj)
{
    Opt3001State *s = OPT3001(obj);

    s->millilux = 100000;
    s->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, opt3001_convert, s);
    object_property_add(obj, "lux", "number", opt3001_get_lux,
                        opt3001_set_lux, NULL, NULL);
}

static const VMStateDescription vmstate_opt3001 = {
    .name = TYPE_OPT3001,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, Opt3001State),
        VMSTATE_TIMER_PTR(timer, Opt3001State),
        VMSTATE_UINT32(millilux, Opt3001State),
        VMSTATE_UINT16(result, Opt3001State),
        VMSTATE_UINT16(config, Opt3001State),
        VMSTATE_UINT16(low, Opt3001State),
        VMSTATE_UINT16(high, Opt3001State),
        VMSTATE_UINT16(value, Opt3001State),
        VMSTATE_UINT8(pointer, Opt3001State),
        VMSTATE_UINT8_ARRAY(data, Opt3001State, 2),
        VMSTATE_UINT8(len, Opt3001State),
        VMSTATE_END_OF_LIST()
    }
};

static void opt3001_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Texas Instruments OPT3001 ambient light sensor";
    dc->realize = opt3001_realize;
    dc->vmsd = &vmstate_opt3001;
    rc->phases.hold = opt3001_reset_hold;
    k->event = opt3001_event;
    k->recv = opt3001_recv;
    k->send = opt3001_send;
}

static const TypeInfo opt3001_info = {
    .name = TYPE_OPT3001,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Opt3001State),
    .instance_init = opt3001_init,
    .class_init = opt3001_class_init,
};

static void opt3001_register_types(void)
{
    type_register_static(&opt3001_info);
}

type_init(opt3001_register_types)
