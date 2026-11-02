/*
 * Ingenic XBurst2 PWM controller (the T32's, T33's, T41's and A1's block)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_PWM_H
#define HW_MISC_INGENIC_PWM_H

#include "chardev/char-fe.h"
#include "hw/core/clock.h"
#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-pwm-out.h"
#include "qom/object.h"

#define TYPE_INGENIC_PWM "ingenic-pwm"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicPwmState, INGENIC_PWM)

#define INGENIC_PWM_NUM_REGS  0x100   /* registers span 0x000..0x3ff */
#define INGENIC_PWM_CHANNELS  16

typedef struct IngenicPwmChannel {
    IngenicPwmState *pwm;
    unsigned index;
    QEMUTimer *timer;       /* the period's end, while BUSY or a stop waits */
    bool running;
    /* what the running periods took from WCFG, PWMLVL and CCFG */
    uint16_t high;
    uint16_t low;
    uint8_t init;
    uint8_t prescale;
    uint32_t hz;            /* the PWM clock they count */
    int64_t anchor_ns;      /* a period starts here, and every period after */
    IngenicPwmKey traced;
} IngenicPwmChannel;

struct IngenicPwmState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    Clock *clk;
    CharFrontend trace;
    uint32_t channels;
    uint32_t regs[INGENIC_PWM_NUM_REGS];
    IngenicPwmChannel chn[INGENIC_PWM_CHANNELS];
};

#endif /* HW_MISC_INGENIC_PWM_H */
