/*
 * A PWM output as the Ingenic timer unit and PWM block compute it
 *
 * The level now and, while it toggles, its period, its high time and its
 * next edge. Both devices expose it as pwmN-level, pwmN-period-ns and
 * pwmN-high-ns, and write it to their "trace" chardev as a JSON line each
 * time it changes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_PWM_OUT_H
#define HW_MISC_INGENIC_PWM_OUT_H

#include "chardev/char-fe.h"
#include "hw/core/cpu.h"
#include "qemu/timer.h"

typedef struct IngenicPwmOut {
    uint64_t period_ns;     /* 0 while the level holds */
    uint64_t high_ns;
    int64_t edge_ns;        /* when the level next changes, while it toggles */
    uint32_t level;
} IngenicPwmOut;

/*
 * What an output's waveform depends on, to trace it only when it changes:
 * the level while it holds, the device's inputs to it while it toggles
 */
typedef struct IngenicPwmKey {
    uint64_t k[4];
} IngenicPwmKey;

static inline void ingenic_pwm_out_trace(CharFrontend *chr, const char *dev,
                                         unsigned ch, const IngenicPwmOut *o)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    g_autoptr(GString) js = g_string_new(NULL);

    g_string_append_printf(js, "{\"t\":%" PRId64 ",\"%s\":%u,\"lvl\":%u",
                           now, dev, ch, o->level);
    if (o->period_ns) {
        g_string_append_printf(js, ",\"period\":%" PRIu64 ",\"high\":%"
                               PRIu64 ",\"edge\":%" PRId64, o->period_ns,
                               o->high_ns, o->edge_ns - now);
    }
    if (current_cpu) {
        g_string_append_printf(js, ",\"pc\":\"0x%08" PRIx64 "\"}\n",
                               (uint64_t)CPU_GET_CLASS(current_cpu)->
                               get_pc(current_cpu));
    } else {
        g_string_append(js, ",\"src\":\"hw\"}\n");
    }
    qemu_chr_fe_write_all(chr, (uint8_t *)js->str, js->len);
}

#endif /* HW_MISC_INGENIC_PWM_OUT_H */
