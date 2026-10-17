/*
 * A camera image sensor's I2C control interface
 *
 * The register file a sensor driver programs over I2C: the chip ID its
 * detect routine checks, and every other register reading back what was
 * last written to it. The register address is one or two bytes, big
 * endian, and auto-increments across a burst, as on the parts listed.
 * GalaxyCore's 8-bit parts bank 0x00-0xef into pages chosen by 0xfe.
 *
 * The IDs are what the vendor drivers' sensor_detect() reads.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/sensor/image-sensor.h"

struct ImageSensorModel {
    const char *name;
    uint8_t addr;               /* the 7-bit I2C address it answers */
    uint8_t reg_bytes;          /* register address width */
    bool paged;
    struct {
        uint16_t reg;
        uint8_t val;
    } id[3];
};

static const ImageSensorModel image_sensor_models[] = {
    { "jxh42",   0x30, 1, false, { { 0x0a, 0xa0 }, { 0x0b, 0x42 },
                                   { 0x09, 0x81 } } },
    { "jxf22",   0x40, 1, false, { { 0x0a, 0x0f }, { 0x0b, 0x22 } } },
    { "sc2232h", 0x30, 2, false, { { 0x3107, 0xcb }, { 0x3108, 0x07 },
                                   { 0x3109, 0x01 } } },
    { "sc2336",  0x30, 2, false, { { 0x3107, 0xcb }, { 0x3108, 0x3a } } },
    { "sc4236",  0x30, 2, false, { { 0x3107, 0x32 }, { 0x3108, 0x35 } } },
    { "gc2053",  0x37, 1, true,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } } },
    /* 0x3003 is the high byte: the T32, T40 and T41 drivers agree */
    { "cv8001",  0x35, 2, false, { { 0x3002, 0x01 }, { 0x3003, 0x80 } } },
    { "imx307",  0x1a, 2, false, { { 0x3008, 0xa0 }, { 0x301e, 0xb2 } } },
    { "gc4023",  0x29, 2, false, { { 0x03f0, 0x40 }, { 0x03f1, 0x23 } } },
};

#define GC_PAGE_REG     0xfe
#define GC_GLOBAL_REGS  0xf0    /* 0xf0-0xff are on every page */

static uint16_t image_sensor_index(ImageSensorState *s, uint16_t reg)
{
    if (s->model->paged && reg < GC_GLOBAL_REGS) {
        return s->page << 8 | reg;
    }
    return reg;
}

static void image_sensor_advance(ImageSensorState *s)
{
    s->pointer++;
    if (s->model->reg_bytes == 1) {
        s->pointer &= 0xff;
    }
}

static int image_sensor_event(I2CSlave *i2c, enum i2c_event event)
{
    ImageSensorState *s = IMAGE_SENSOR(i2c);

    if (event == I2C_START_SEND) {
        s->addr_bytes = 0;
    }
    return 0;
}

static int image_sensor_send(I2CSlave *i2c, uint8_t data)
{
    ImageSensorState *s = IMAGE_SENSOR(i2c);

    if (s->addr_bytes < s->model->reg_bytes) {
        s->pointer = s->addr_bytes++ ? s->pointer << 8 | data : data;
        return 0;
    }
    if (s->model->paged && s->pointer == GC_PAGE_REG) {
        s->page = data & 7;
    }
    s->regs[image_sensor_index(s, s->pointer)] = data;
    image_sensor_advance(s);
    return 0;
}

static uint8_t image_sensor_recv(I2CSlave *i2c)
{
    ImageSensorState *s = IMAGE_SENSOR(i2c);
    uint8_t v = s->regs[image_sensor_index(s, s->pointer)];

    image_sensor_advance(s);
    return v;
}

static void image_sensor_realize(DeviceState *dev, Error **errp)
{
    ImageSensorState *s = IMAGE_SENSOR(dev);
    I2CSlave *i2c = I2C_SLAVE(dev);

    for (int i = 0; i < ARRAY_SIZE(image_sensor_models); i++) {
        if (s->model_name && !strcmp(s->model_name,
                                     image_sensor_models[i].name)) {
            s->model = &image_sensor_models[i];
        }
    }
    if (!s->model) {
        g_autoptr(GString) names = g_string_new(NULL);

        for (int i = 0; i < ARRAY_SIZE(image_sensor_models); i++) {
            g_string_append_printf(names, "%s%s", i ? ", " : "",
                                   image_sensor_models[i].name);
        }
        error_setg(errp, "image-sensor: model must be one of %s", names->str);
        return;
    }
    if (!i2c->address) {
        i2c->address = s->model->addr;
    }
}

static void image_sensor_reset_hold(Object *obj, ResetType type)
{
    ImageSensorState *s = IMAGE_SENSOR(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->pointer = 0;
    s->addr_bytes = 0;
    s->page = 0;
    for (int i = 0; i < ARRAY_SIZE(s->model->id) && s->model->id[i].reg; i++) {
        s->regs[s->model->id[i].reg] = s->model->id[i].val;
    }
}

static const VMStateDescription vmstate_image_sensor = {
    .name = TYPE_IMAGE_SENSOR,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, ImageSensorState),
        VMSTATE_UINT8_ARRAY(regs, ImageSensorState, 65536),
        VMSTATE_UINT16(pointer, ImageSensorState),
        VMSTATE_UINT8(addr_bytes, ImageSensorState),
        VMSTATE_UINT8(page, ImageSensorState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property image_sensor_props[] = {
    DEFINE_PROP_STRING("model", ImageSensorState, model_name),
};

static void image_sensor_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Camera image sensor (I2C control)";
    dc->realize = image_sensor_realize;
    dc->vmsd = &vmstate_image_sensor;
    rc->phases.hold = image_sensor_reset_hold;
    device_class_set_props(dc, image_sensor_props);
    k->event = image_sensor_event;
    k->send = image_sensor_send;
    k->recv = image_sensor_recv;
}

static const TypeInfo image_sensor_type_info = {
    .name = TYPE_IMAGE_SENSOR,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ImageSensorState),
    .class_init = image_sensor_class_init,
};

static void image_sensor_register_types(void)
{
    type_register_static(&image_sensor_type_info);
}

type_init(image_sensor_register_types)
