/*
 * Ingenic CPM: the rate of a clock divider, whichever CPM the SoC has
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-cgu.h"

uint64_t ingenic_cgu_hz(Object *cpm, hwaddr cdr)
{
    IngenicCguClass *c = INGENIC_CGU_GET_CLASS(cpm);

    return c->cgu_hz ? c->cgu_hz(cpm, cdr) : 0;
}

static const TypeInfo ingenic_cgu_info = {
    .name = TYPE_INGENIC_CGU,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(IngenicCguClass),
};

static void ingenic_cgu_register_types(void)
{
    type_register_static(&ingenic_cgu_info);
}

type_init(ingenic_cgu_register_types)
