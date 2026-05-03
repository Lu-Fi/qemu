/*
 * Ingenic T31 GPIO controller emulation
 *
 * The kernel ioremap()s a single 64 KiB region at 0x10010000 and
 * treats it as N ports, each at offset i * 0x1000, plus a shadow
 * page at offset 0x7000. Each port exposes paired set/clear
 * registers around a base value (e.g. PXMSK at 0x20, PXMSKS at 0x24
 * sets bits, PXMSKC at 0x28 clears bits).
 *
 * Reads return the current value. Writes to the *S register OR the
 * value into the base; writes to the *C register AND-NOT the value.
 * Writes to the base register replace the value (useful for save/
 * restore paths).
 *
 * Real hardware has more semantics (pin-level capture, edge detect,
 * driving outputs out a physical pad). For boot-time usage the
 * kernel just configures pin functions and reads the registers
 * back, so plain register-bank storage with set/clear handling is
 * enough to advance past the GPIO platform driver init.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/gpio/ingenic-t31-gpio.h"

/* Per-port register offsets within a 0x1000 page */
#define PXPIN       0x000
#define PXINT       0x010
#define PXINTS      0x014
#define PXINTC      0x018
#define PXMSK       0x020
#define PXMSKS      0x024
#define PXMSKC      0x028
#define PXPAT1      0x030
#define PXPAT1S     0x034
#define PXPAT1C     0x038
#define PXPAT0      0x040
#define PXPAT0S     0x044
#define PXPAT0C     0x048
#define PXFLG       0x050
#define PXFLGC      0x058
#define PXPUEN      0x110
#define PXPUENS     0x114
#define PXPUENC     0x118
#define PXPDEN      0x120
#define PXPDENS     0x124
#define PXPDENC     0x128

/* Shadow page register at 0xF0: writing a port index loads the
 * shadow register values into that port's bank. We accept the
 * write but don't actually copy (no observable side-effect for
 * boot-time configuration). */
#define PZGID2LD    0x0F0

static struct IngenicT31GpioPort *
gpio_select_port(IngenicT31GpioState *s, hwaddr offset, hwaddr *port_off)
{
    if (offset >= INGENIC_T31_GPIO_SHADOW_OFF &&
        offset < INGENIC_T31_GPIO_SHADOW_OFF + INGENIC_T31_GPIO_PORT_OFF) {
        *port_off = offset - INGENIC_T31_GPIO_SHADOW_OFF;
        return &s->shadow;
    }
    if (offset < INGENIC_T31_GPIO_NR_PORTS * INGENIC_T31_GPIO_PORT_OFF) {
        unsigned idx = offset / INGENIC_T31_GPIO_PORT_OFF;
        *port_off = offset & (INGENIC_T31_GPIO_PORT_OFF - 1);
        return &s->port[idx];
    }
    return NULL;
}

static uint64_t ingenic_t31_gpio_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicT31GpioState *s = INGENIC_T31_GPIO(opaque);
    struct IngenicT31GpioPort *p;
    hwaddr po;

    p = gpio_select_port(s, offset, &po);
    if (!p) {
        return 0;
    }

    switch (po) {
    case PXPIN:
        /* No external drivers wired. Return the current output level
         * encoded in PAT0 for any pins driven as outputs. Inputs
         * read back as zero. */
        return p->pat0;
    case PXINT: case PXINTS: case PXINTC:
        return p->intr;
    case PXMSK: case PXMSKS: case PXMSKC:
        return p->msk;
    case PXPAT1: case PXPAT1S: case PXPAT1C:
        return p->pat1;
    case PXPAT0: case PXPAT0S: case PXPAT0C:
        return p->pat0;
    case PXFLG: case PXFLGC:
        return p->flg;
    case PXPUEN: case PXPUENS: case PXPUENC:
        return p->puen;
    case PXPDEN: case PXPDENS: case PXPDENC:
        return p->pden;
    default:
        return 0;
    }
}

static void ingenic_t31_gpio_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicT31GpioState *s = INGENIC_T31_GPIO(opaque);
    struct IngenicT31GpioPort *p;
    hwaddr po;
    uint32_t v = (uint32_t)value;

    p = gpio_select_port(s, offset, &po);
    if (!p) {
        return;
    }

    switch (po) {
    case PXPIN:
        /* read-only on real HW; ignore writes */
        break;
    case PXINT:    p->intr = v; break;
    case PXINTS:   p->intr |= v; break;
    case PXINTC:   p->intr &= ~v; break;
    case PXMSK:    p->msk = v; break;
    case PXMSKS:   p->msk |= v; break;
    case PXMSKC:   p->msk &= ~v; break;
    case PXPAT1:   p->pat1 = v; break;
    case PXPAT1S:  p->pat1 |= v; break;
    case PXPAT1C:  p->pat1 &= ~v; break;
    case PXPAT0:   p->pat0 = v; break;
    case PXPAT0S:  p->pat0 |= v; break;
    case PXPAT0C:  p->pat0 &= ~v; break;
    case PXFLG:    p->flg = v; break;
    case PXFLGC:   p->flg &= ~v; break;
    case PXPUEN:   p->puen = v; break;
    case PXPUENS:  p->puen |= v; break;
    case PXPUENC:  p->puen &= ~v; break;
    case PXPDEN:   p->pden = v; break;
    case PXPDENS:  p->pden |= v; break;
    case PXPDENC:  p->pden &= ~v; break;
    case PZGID2LD:
        /* Shadow-load trigger: software writes group ID 0..N to
         * latch shadow values into that port. We accept the write
         * but don't actually propagate (boot path doesn't poll for
         * the effect). */
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, offset, v);
        break;
    }
}

static const MemoryRegionOps ingenic_t31_gpio_ops = {
    .read = ingenic_t31_gpio_read,
    .write = ingenic_t31_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_gpio_reset_hold(Object *obj, ResetType type)
{
    IngenicT31GpioState *s = INGENIC_T31_GPIO(obj);

    memset(s->port, 0, sizeof(s->port));
    memset(&s->shadow, 0, sizeof(s->shadow));
}

static void ingenic_t31_gpio_init(Object *obj)
{
    IngenicT31GpioState *s = INGENIC_T31_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_gpio_ops, s,
                          TYPE_INGENIC_T31_GPIO,
                          INGENIC_T31_GPIO_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void ingenic_t31_gpio_class_init(ObjectClass *oc, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.hold = ingenic_t31_gpio_reset_hold;
}

static const TypeInfo ingenic_t31_gpio_type_info = {
    .name = TYPE_INGENIC_T31_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31GpioState),
    .instance_init = ingenic_t31_gpio_init,
    .class_init = ingenic_t31_gpio_class_init,
};

static void ingenic_t31_gpio_register_types(void)
{
    type_register_static(&ingenic_t31_gpio_type_info);
}

type_init(ingenic_t31_gpio_register_types)
