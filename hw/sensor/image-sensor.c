/*
 * A camera image sensor's I2C control interface
 *
 * The register file a sensor driver programs over I2C: the chip ID its
 * detect routine checks, and every other register reading back what was
 * last written to it. The register address is one or two bytes, big
 * endian, and auto-increments across a burst, as on the parts listed.
 * An 8-bit part that banks its registers picks the page with one of them:
 * GalaxyCore's 0xfe, which leaves 0xf0-0xff on every page, or 0xfd.
 *
 * The IDs are what the vendor drivers' sensor_detect() reads.
 *
 * It is also the camera: the scene it sees, at "fps" frames a second, is
 * what the SoC's video input takes in, rendered as the ISP puts it out.
 * With "noise" set the scene's luma carries noise that changes every frame,
 * which no prediction removes: pictures an encoder cannot code small.
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
    uint8_t page_reg;           /* a banked part's page select, else 0 */
    struct {
        uint16_t reg;
        uint8_t val;
    } id[4];
    uint16_t stream_reg;        /* frames flow while these bits are set */
    uint8_t stream_mask;        /* 0: the part always streams */
};

static const ImageSensorModel image_sensor_models[] = {
    { "jxh42",   0x30, 1, 0,     { { 0x0a, 0xa0 }, { 0x0b, 0x42 },
                                   { 0x09, 0x81 } } },
    { "jxf22",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x22 } } },
    { "sc2232h", 0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x07 },
                                   { 0x3109, 0x01 } } },
    { "sc2336",  0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x3a } } },
    { "sc4236",  0x30, 2, 0,     { { 0x3107, 0x32 }, { 0x3108, 0x35 } } },
    { "gc2053",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } } },
    /* 0x3003 is the high byte: the T32, T40 and T41 drivers agree */
    { "cv8001",  0x35, 2, 0,     { { 0x3002, 0x01 }, { 0x3003, 0x80 } } },
    { "imx307",  0x1a, 2, 0,     { { 0x3008, 0xa0 }, { 0x301e, 0xb2 } } },
    { "gc4023",  0x29, 2, 0,     { { 0x03f0, 0x40 }, { 0x03f1, 0x23 } } },
    { "gc5613",  0x31, 2, 0,     { { 0x03f0, 0x56 }, { 0x03f1, 0x03 } },
      0x0100, 0x01 },
    /* gc2063 and gc4663 answer as gc2053 and gc4653 do */
    { "gc2023",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x23 } } },
    { "gc2063",  0x37, 1, 0xfe,  { { 0xf0, 0x20 }, { 0xf1, 0x53 } } },
    { "gc3003a", 0x37, 2, 0,     { { 0x03f0, 0x30 }, { 0x03f1, 0x03 },
                                   { 0x036a, 0x10 } } },
    { "gc4653",  0x29, 2, 0,     { { 0x03f0, 0x46 }, { 0x03f1, 0x53 } } },
    { "gc4663",  0x29, 2, 0,     { { 0x03f0, 0x46 }, { 0x03f1, 0x53 } } },
    /* 0x3008 reads its reset value, as an imx307 does: older sinfo tables see one */
    { "imx327",  0x1a, 2, 0,     { { 0x3008, 0xa0 }, { 0x301e, 0xb2 },
                                   { 0x301f, 0x01 } } },
    { "jxf23",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x23 } } },
    { "jxf28",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x28 } } },
    { "jxf37",   0x40, 1, 0,     { { 0x0a, 0x0f }, { 0x0b, 0x37 } } },
    { "mis20c1", 0x30, 2, 0,     { { 0x3000, 0x20 }, { 0x3001, 0xc1 } } },
    /* the T32 drivers check 0x05 as well */
    { "os04d10", 0x3c, 1, 0xfd,  { { 0x02, 0x53 }, { 0x03, 0x04 },
                                   { 0x04, 0x44 }, { 0x05, 0x10 } } },
    { "os05a10", 0x36, 2, 0,     { { 0x300a, 0x53 }, { 0x300b, 0x05 },
                                   { 0x300c, 0x41 } } },
    { "os08a10", 0x36, 2, 0,     { { 0x300a, 0x53 }, { 0x300b, 0x08 },
                                   { 0x300c, 0x41 } } },
    { "ov2710",  0x36, 2, 0,     { { 0x300a, 0x27 }, { 0x300b, 0x10 } } },
    { "ov9732",  0x36, 2, 0,     { { 0x300a, 0x97 }, { 0x300b, 0x32 } } },
    { "ov9750",  0x36, 2, 0,     { { 0x300b, 0x97 }, { 0x300c, 0x50 } } },
    { "sc1a4t",  0x30, 2, 0,     { { 0x3107, 0xda }, { 0x3108, 0x4d } } },
    { "sc2135",  0x30, 2, 0,     { { 0x3107, 0x21 }, { 0x3108, 0x35 } } },
    { "sc2232",  0x30, 2, 0,     { { 0x3107, 0x22 }, { 0x3108, 0x32 },
                                   { 0x3109, 0x01 } } },
    { "sc2235",  0x30, 2, 0,     { { 0x3107, 0x22 }, { 0x3108, 0x35 } } },
    { "sc231hai", 0x30, 2, 0,     { { 0x3107, 0xcb }, { 0x3108, 0x6a } } },
    { "sc2310",  0x30, 2, 0,     { { 0x3107, 0x23 }, { 0x3108, 0x11 } } },
    { "sc301iot", 0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x40 } } },
    { "sc3235",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x05 } } },
    { "sc3335",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x1a } } },
    /* sc3338 answers as sc3336 does */
    { "sc3336",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x41 } } },
    { "sc3338",  0x30, 2, 0,     { { 0x3107, 0xcc }, { 0x3108, 0x41 } } },
    { "sc401ai", 0x30, 2, 0,     { { 0x3107, 0xcd }, { 0x3108, 0x2e } } },
    { "sc4336p", 0x30, 2, 0,     { { 0x3107, 0x9c }, { 0x3108, 0x42 } } },
    { "sc8238",  0x30, 2, 0,     { { 0x3107, 0x82 }, { 0x3108, 0x35 } } },
    { "sc835hai", 0x30, 2, 0,     { { 0x3107, 0xc1 }, { 0x3108, 0x70 } } },
};

/* 75% colour bars, BT.601 limited range: Y, Cb, Cr */
static const uint8_t image_sensor_bars[8][3] = {
    { 180, 128, 128 },          /* white */
    { 162,  44, 142 },          /* yellow */
    { 131, 156,  44 },          /* cyan */
    { 112,  72,  58 },          /* green */
    {  84, 184, 198 },          /* magenta */
    {  65, 100, 212 },          /* red */
    {  35, 212, 114 },          /* blue */
    {  16, 128, 128 },          /* black */
};

#define GC_PAGE_REG     0xfe
#define GC_GLOBAL_REGS  0xf0    /* 0xf0-0xff are on every page */

/* The page select is on every page, as GalaxyCore's 0xf0-0xff are */
static uint16_t image_sensor_index(ImageSensorState *s, uint16_t reg)
{
    uint8_t page_reg = s->model->page_reg;

    if (!page_reg || reg == page_reg ||
        (page_reg == GC_PAGE_REG && reg >= GC_GLOBAL_REGS)) {
        return reg;
    }
    return s->page << 8 | reg;
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
    if (s->model->page_reg && s->pointer == s->model->page_reg) {
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

bool image_sensor_streaming(ImageSensorState *s)
{
    const ImageSensorModel *m = s->model;

    return !m->stream_mask || s->regs[m->stream_reg] & m->stream_mask;
}

ImageSensorState *image_sensor_find(void)
{
    Object *o = object_resolve_path_type("", TYPE_IMAGE_SENSOR, NULL);

    return o ? IMAGE_SENSOR(o) : NULL;
}

/*
 * Bars over a luma ramp, crossed by a white stripe every two seconds so
 * that motion shows in what is encoded
 */
void image_sensor_render(ImageSensorState *s, uint64_t frame, uint32_t w,
                         uint32_t h, uint8_t *nv12)
{
    g_autofree uint8_t *bar = g_malloc(w * 2);
    g_autofree uint8_t *ramp = g_malloc(w * 2);
    uint32_t stripe = MAX(w / 64, 2);
    uint32_t x0 = frame * w / (2 * s->fps) % w;
    uint32_t bars_h = h * 2 / 3;
    uint8_t *uv = nv12 + w * h;

    for (uint32_t x = 0; x < w; x++) {
        const uint8_t *c = image_sensor_bars[x * 8 / w];
        bool lit = x - x0 < stripe;

        bar[x] = lit ? 235 : c[0];
        ramp[x] = lit ? 235 : 16 + x * 219 / w;
        bar[w + x] = lit ? 128 : c[1 + (x & 1)];
        ramp[w + x] = 128;
    }
    for (uint32_t y = 0; y < h; y++) {
        memcpy(nv12 + y * w, y < bars_h ? bar : ramp, w);
    }
    for (uint32_t y = 0; y < h / 2; y++) {
        memcpy(uv + y * w, (y * 2 < bars_h ? bar : ramp) + w, w);
    }
    if (s->noise) {
        for (uint32_t i = 0; i < w * h; i++) {
            uint32_t r = (i + 1) * 2654435761u ^ (uint32_t)frame * 40503u;

            r ^= r >> 15;
            r *= 2246822519u;
            r ^= r >> 13;
            nv12[i] = MIN(MAX(nv12[i] + (int)(r & 63) - 32, 0), 255);
        }
    }
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
    if (!s->fps) {
        error_setg(errp, "image-sensor: fps must be at least 1");
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
    DEFINE_PROP_UINT32("fps", ImageSensorState, fps, 30),
    DEFINE_PROP_BOOL("noise", ImageSensorState, noise, false),
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
