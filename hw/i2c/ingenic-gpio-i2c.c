/*
 * I2C bus bit-banged on two pads of an Ingenic GPIO controller
 *
 * What the kernel's i2c-gpio does on plain GPIOs, for boards that put a
 * bus on pads that are no I2C controller's: "sda" and "scl" are the GPIO
 * numbers (port * 32 + pad) of its two lines. The bus is named after the
 * device's id, or gpio-i2c<sda> without one, and takes devices with bus=.
 * SDA reads low while the guest or a device on the bus holds it low; the
 * lines are open drain, so a pad the guest has not made an output low
 * reads high. A device never stretches the clock.
 *
 * The pins go to the SoC's GPIO controller: the one linked as "gpio", or
 * the only one there is.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/ingenic-gpio.h"
#include "hw/i2c/bitbang_i2c.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TYPE_INGENIC_GPIO_I2C "ingenic-gpio-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicGpioI2cState, INGENIC_GPIO_I2C)

struct IngenicGpioI2cState {
    DeviceState parent_obj;

    I2CBus *bus;
    bitbang_i2c_interface bitbang;
    qemu_irq sda_pull;
    DeviceState *gpio;
    int32_t sda, scl;
};

static void gpio_i2c_pads(void *opaque, int line, int level)
{
    IngenicGpioI2cState *s = opaque;

    qemu_set_irq(s->sda_pull, bitbang_i2c_set(&s->bitbang, line, level));
}

static void gpio_i2c_realize(DeviceState *dev, Error **errp)
{
    IngenicGpioI2cState *s = INGENIC_GPIO_I2C(dev);
    g_autofree char *name = NULL;
    bool ambiguous;
    uint32_t pins;

    if (!s->gpio) {
        Object *o = object_resolve_path_type("", TYPE_INGENIC_GPIO,
                                             &ambiguous);

        if (!o || ambiguous) {
            error_setg(errp, "no single Ingenic GPIO controller: "
                       "set the gpio link");
            return;
        }
        s->gpio = DEVICE(o);
    }
    if (!object_dynamic_cast(OBJECT(s->gpio), TYPE_INGENIC_GPIO)) {
        error_setg(errp, "gpio is not an %s", TYPE_INGENIC_GPIO);
        return;
    }
    pins = INGENIC_GPIO(s->gpio)->nr_ports * 32;
    if (s->sda < 0 || s->scl < 0 || s->sda >= pins || s->scl >= pins ||
        s->sda == s->scl) {
        error_setg(errp, "sda and scl must be two different GPIOs under %u",
                   pins);
        return;
    }

    name = dev->id ? g_strdup(dev->id) : g_strdup_printf("gpio-i2c%d", s->sda);
    s->bus = i2c_init_bus(dev, name);
    bitbang_i2c_init(&s->bitbang, s->bus);

    qdev_init_gpio_in_named(dev, gpio_i2c_pads, "pads", 2);
    qdev_connect_gpio_out_named(s->gpio, "pad", s->sda,
                                qdev_get_gpio_in_named(dev, "pads",
                                                       BITBANG_I2C_SDA));
    qdev_connect_gpio_out_named(s->gpio, "pad", s->scl,
                                qdev_get_gpio_in_named(dev, "pads",
                                                       BITBANG_I2C_SCL));
    s->sda_pull = qdev_get_gpio_in_named(s->gpio, "pull", s->sda);
}

static const Property gpio_i2c_props[] = {
    DEFINE_PROP_LINK("gpio", IngenicGpioI2cState, gpio, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_INT32("sda", IngenicGpioI2cState, sda, -1),
    DEFINE_PROP_INT32("scl", IngenicGpioI2cState, scl, -1),
};

/* as the controllers' bit-banging, a transfer in flight is not kept */
static const VMStateDescription vmstate_gpio_i2c = {
    .name = TYPE_INGENIC_GPIO_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_END_OF_LIST()
    },
};

static void gpio_i2c_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "I2C bus on two Ingenic GPIO pads";
    dc->realize = gpio_i2c_realize;
    dc->bus_type = TYPE_SYSTEM_BUS;
    dc->vmsd = &vmstate_gpio_i2c;
    device_class_set_props(dc, gpio_i2c_props);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo gpio_i2c_info = {
    .name = TYPE_INGENIC_GPIO_I2C,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(IngenicGpioI2cState),
    .class_init = gpio_i2c_class_init,
};

static void gpio_i2c_register_types(void)
{
    type_register_static(&gpio_i2c_info);
}

type_init(gpio_i2c_register_types)
