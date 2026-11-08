/*
 * Ingenic CPM: the rate of a clock divider, whichever CPM the SoC has
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_CGU_H
#define HW_MISC_INGENIC_CGU_H

#include "qom/object.h"
#include "exec/hwaddr.h"

#define TYPE_INGENIC_CGU "ingenic-cgu"
typedef struct IngenicCguClass IngenicCguClass;
DECLARE_CLASS_CHECKERS(IngenicCguClass, INGENIC_CGU, TYPE_INGENIC_CGU)

struct IngenicCguClass {
    InterfaceClass parent;

    /* The rate in Hz of the divider at CPM offset @cdr, 0 if it is unset */
    uint64_t (*cgu_hz)(Object *cpm, hwaddr cdr);
};

uint64_t ingenic_cgu_hz(Object *cpm, hwaddr cdr);

#endif /* HW_MISC_INGENIC_CGU_H */
