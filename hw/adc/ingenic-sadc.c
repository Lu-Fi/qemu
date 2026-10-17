/*
 * Ingenic SAR A/D controller (SADC)
 *
 * Eight auxiliary channels at 0x10070000. Enabling a channel (ADENA bit
 * N) starts its conversions: its ready bit (ADSTATE bit N) sets a moment
 * later, and again a moment after each write-1-to-clear while the channel
 * stays on. Its 12-bit sample reads at 0x10 + 2N. The block interrupts
 * while an unmasked (ADCTRL bit N clear) channel is ready; the 3.10
 * jz_adc_aux driver waits on that interrupt, the T32's jz_temp polls the
 * ready bit of channel 3.
 *
 * Properties aux0 .. aux7 set the samples, at run time too: a test can
 * darken the room a camera's photoresistor sees. They default to 2025,
 * bright to a photoresistor and 40 degrees C in the T32's temperature
 * table.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/adc/ingenic-sadc.h"

#define ADENA       0x00
#define ADCFG       0x04
#define ADCTRL      0x08
#define ADSTATE     0x0c
#define ADATA       0x10
#define ADCLK       0x20

#define SADC_CONVERT_NS     (100 * SCALE_US)
#define SADC_DEFAULT        2025

static void sadc_update(IngenicSadcState *s)
{
    qemu_set_irq(s->irq, !!(s->adstate & ~s->adctrl & 0xff));
}

static void sadc_convert(void *opaque)
{
    IngenicSadcState *s = opaque;

    s->adstate |= s->adena & 0xff;
    sadc_update(s);
}

static void sadc_arm(IngenicSadcState *s)
{
    if (s->adena & ~s->adstate & 0xff) {
        timer_mod(s->timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SADC_CONVERT_NS);
    }
}

static uint64_t sadc_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicSadcState *s = opaque;

    switch (offset) {
    case ADENA:
        return s->adena;
    case ADCFG:
        return s->adcfg;
    case ADCTRL:
        return s->adctrl;
    case ADSTATE:
        return s->adstate;
    case ADATA ... ADATA + 0xc: {
        unsigned ch = (offset - ADATA) / 2;

        return s->aux[ch] | s->aux[ch + 1] << 16;
    }
    case ADCLK:
        return s->adclk;
    default:
        return 0;
    }
}

static void sadc_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    IngenicSadcState *s = opaque;
    uint32_t v = value;

    switch (offset) {
    case ADENA:
        s->adena = v;
        sadc_arm(s);
        break;
    case ADCFG:
        s->adcfg = v;
        break;
    case ADCTRL:
        s->adctrl = v;
        sadc_update(s);
        break;
    case ADSTATE:
        s->adstate &= ~v;
        sadc_update(s);
        sadc_arm(s);
        break;
    case ADCLK:
        s->adclk = v;
        break;
    }
}

static const MemoryRegionOps sadc_ops = {
    .read = sadc_read,
    .write = sadc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void sadc_aux_get(Object *obj, Visitor *v, const char *name,
                         void *opaque, Error **errp)
{
    uint32_t *aux = opaque;

    visit_type_uint32(v, name, aux, errp);
}

static void sadc_aux_set(Object *obj, Visitor *v, const char *name,
                         void *opaque, Error **errp)
{
    uint32_t *aux = opaque;
    uint32_t value;

    if (!visit_type_uint32(v, name, &value, errp)) {
        return;
    }
    if (value > 0xfff) {
        error_setg(errp, "%s: a sample is 12 bits (0 to 4095)", name);
        return;
    }
    *aux = value;
}

static void sadc_reset_hold(Object *obj, ResetType type)
{
    IngenicSadcState *s = INGENIC_SADC(obj);

    timer_del(s->timer);
    s->adena = s->adcfg = s->adctrl = s->adstate = s->adclk = 0;
}

static void sadc_reset_exit(Object *obj, ResetType type)
{
    sadc_update(INGENIC_SADC(obj));
}

static void sadc_init(Object *obj)
{
    IngenicSadcState *s = INGENIC_SADC(obj);

    memory_region_init_io(&s->iomem, obj, &sadc_ops, s, TYPE_INGENIC_SADC,
                          0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, sadc_convert, s);
    for (unsigned ch = 0; ch < INGENIC_SADC_CHANNELS; ch++) {
        g_autofree char *name = g_strdup_printf("aux%u", ch);

        s->aux[ch] = SADC_DEFAULT;
        object_property_add(obj, name, "uint32", sadc_aux_get, sadc_aux_set,
                            NULL, &s->aux[ch]);
    }
}

static void sadc_finalize(Object *obj)
{
    timer_free(INGENIC_SADC(obj)->timer);
}

static void sadc_class_init(ObjectClass *oc, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.hold = sadc_reset_hold;
    rc->phases.exit = sadc_reset_exit;
}

static const TypeInfo sadc_info = {
    .name = TYPE_INGENIC_SADC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicSadcState),
    .instance_init = sadc_init,
    .instance_finalize = sadc_finalize,
    .class_init = sadc_class_init,
};

static void sadc_register_types(void)
{
    type_register_static(&sadc_info);
}

type_init(sadc_register_types)
