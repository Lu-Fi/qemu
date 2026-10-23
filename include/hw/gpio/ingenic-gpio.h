/*
 * Ingenic XBurst GPIO controller emulation
 *
 * Up to six ports at base + i * port-stride, plus the PZ shadow page
 * that stages pin-function changes for one port at a time.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_GPIO_INGENIC_GPIO_H
#define HW_GPIO_INGENIC_GPIO_H

#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_INGENIC_GPIO "ingenic-gpio"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicGpioState, INGENIC_GPIO)

#define INGENIC_GPIO_MAX_PORTS   6
#define INGENIC_GPIO_SHADOW_OFF  0x7000
#define INGENIC_GPIO_IOSIZE      0x10000

typedef struct IngenicGpioPort {
    uint32_t intr;      /* 0x10 - interrupt enable */
    uint32_t msk;       /* 0x20 - interrupt mask */
    uint32_t pat1;      /* 0x30 - pattern bit 1 */
    uint32_t pat0;      /* 0x40 - pattern bit 0 */
    uint32_t flg;       /* 0x50 - interrupt flag */
    uint32_t puen;      /* 0x110 - pull-up enable */
    uint32_t pden;      /* 0x120 - pull-down enable */
    uint32_t in;        /* the level outside drives onto each pad */
    uint32_t driven;    /* the pads a test has driven through in-pX */
    uint32_t last;      /* the pad levels at the last update, for edges */
    uint32_t drive;     /* what each pad drives onto its wire: 1 released */
    uint32_t traced[5]; /* INT, MSK, PAT1, PAT0 and level as last traced */
} IngenicGpioPort;

struct IngenicGpioState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t port_stride;
    uint32_t nr_ports;
    uint32_t pinval;        /* the level of a pad nothing drives */

    IngenicGpioPort port[INGENIC_GPIO_MAX_PORTS];
    uint32_t outside[INGENIC_GPIO_MAX_PORTS];   /* in-pX, kept by a reset */
    /* PZ shadow: the bits to set and to clear in INT, MSK, PAT1, PAT0 */
    uint32_t zset[4];
    uint32_t zclr[4];
    qemu_irq irq[INGENIC_GPIO_MAX_PORTS];
    qemu_irq pad[INGENIC_GPIO_MAX_PORTS * 32];
    CharFrontend trace;     /* a JSON line for each pin that changes */
    bool trace_ext;         /* the change comes from outside the guest */
};

#endif /* HW_GPIO_INGENIC_GPIO_H */
