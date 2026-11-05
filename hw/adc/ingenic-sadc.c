/*
 * Ingenic SAR A/D controller (SADC)
 *
 * Eight auxiliary channels at 0x10070000. Setting a channel's ADENA bit N
 * with the converter powered up (ADENA bit 15 clear) converts it once: the
 * bit reads 1 until the sample is taken, then clears as the channel's ready
 * bit (ADSTATE bit N) sets, REPTEN (bit 9) or not. ADENA's upper bits hold
 * what is written. A channel bit set while the converter is powered down
 * never converts, even once it powers up, and no write clears it until
 * reset: a T31X wedges so. The 12-bit sample reads at 0x10 + 2N, 0 until
 * the first conversion. Out of reset the converter is powered down and
 * every channel's interrupt masked, as every part reads them. The block
 * interrupts while an unmasked (ADCTRL bit N clear) channel is ready; the
 * 3.10 jz_adc_aux driver waits on that interrupt, the T32's jz_temp polls
 * the ready bit of channel 3. Both set the channel's bit for every sample.
 *
 * Properties aux0 .. aux7 set the samples, at run time too: a test can
 * darken the room a camera's photoresistor sees. They default to 2025,
 * bright to a photoresistor and 40 degrees C in the T32's temperature
 * table.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/core/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/adc/ingenic-sadc.h"
#include "migration/vmstate.h"

#define ADENA       0x00
#define ADCFG       0x04
#define ADCTRL      0x08
#define ADSTATE     0x0c
#define ADATA       0x10
#define ADCLK       0x20
#define ADSTB       0x24        /* the stable delay before a sample */
#define ADRETM      0x28        /* the repeat time */

#define ADENA_POWER BIT(15)     /* powered down */

#define SADC_CONVERT_NS     (100 * SCALE_US)
#define SADC_DEFAULT        2025

static void sadc_update(IngenicSadcState *s)
{
    qemu_set_irq(s->irq, !!(s->adstate & ~s->adctrl & 0xff));
}

static void sadc_convert(void *opaque)
{
    IngenicSadcState *s = opaque;

    if (s->adena & ADENA_POWER) {
        s->pending = 0;
        return;
    }
    for (unsigned ch = 0; ch < INGENIC_SADC_CHANNELS; ch++) {
        if (s->pending & BIT(ch)) {
            s->data[ch] = s->aux[ch];
        }
    }
    s->adstate |= s->pending;
    s->adena &= ~s->pending;
    s->pending = 0;
    sadc_update(s);
}

static void sadc_arm(IngenicSadcState *s)
{
    if (s->pending) {
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

        return s->data[ch] | s->data[ch + 1] << 16;
    }
    case ADCLK:
        return s->adclk;
    case ADSTB:
        return s->adstb;
    case ADRETM:
        return s->adretm;
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
    case ADENA: {
        uint32_t start = v & ~s->adena & 0xff;

        s->adena = (v & ~0xff) | (s->adena & 0xff) | start;
        if (!(v & ADENA_POWER)) {
            s->pending |= start;
        }
        sadc_arm(s);
        break;
    }
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
        break;
    case ADCLK:
        s->adclk = v;
        break;
    case ADSTB:
        s->adstb = v;
        break;
    case ADRETM:
        s->adretm = v;
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
    s->adena = ADENA_POWER;
    s->adcfg = s->adcfg_reset;
    s->adctrl = 0xff;
    s->adstate = 0;
    s->pending = 0;
    s->adclk = s->adclk_reset;
    s->adstb = s->adstb_reset;
    s->adretm = 0;
    memset(s->data, 0, sizeof(s->data));
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

static const VMStateDescription vmstate_ingenic_sadc = {
    .name = TYPE_INGENIC_SADC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, IngenicSadcState),
        VMSTATE_UINT32(adena, IngenicSadcState),
        VMSTATE_UINT32(adcfg, IngenicSadcState),
        VMSTATE_UINT32(adctrl, IngenicSadcState),
        VMSTATE_UINT32(adstate, IngenicSadcState),
        VMSTATE_UINT32(adclk, IngenicSadcState),
        VMSTATE_UINT32(adstb, IngenicSadcState),
        VMSTATE_UINT32(adretm, IngenicSadcState),
        VMSTATE_UINT32(pending, IngenicSadcState),
        VMSTATE_UINT32_ARRAY(aux, IngenicSadcState, INGENIC_SADC_CHANNELS),
        VMSTATE_UINT16_ARRAY(data, IngenicSadcState, INGENIC_SADC_CHANNELS),
        VMSTATE_END_OF_LIST()
    }
};

/* As every part reads them but the T10 (ADCFG 0) and T32 (ADCLK 0x04021000) */
static const Property sadc_properties[] = {
    DEFINE_PROP_UINT32("adcfg", IngenicSadcState, adcfg_reset, 0x00000001),
    DEFINE_PROP_UINT32("adclk", IngenicSadcState, adclk_reset, 0x00041000),
    DEFINE_PROP_UINT32("adstb", IngenicSadcState, adstb_reset, 0),
};

static void sadc_class_init(ObjectClass *oc, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_props(DEVICE_CLASS(oc), sadc_properties);
    DEVICE_CLASS(oc)->vmsd = &vmstate_ingenic_sadc;
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
