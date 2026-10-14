/*
 * Ingenic XBurst MXU2/MXU3 SIMD control state
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "internal.h"
#include "fpu/softfloat.h"
#include "fpu_helper.h"

void mxu_reset(CPUMIPSState *env)
{
    float_status *st = &env->active_tc.mxu_fp_status;

    if (!(env->insn_flags & ASE_MXU2)) {
        return;
    }

    /* MIR and MCSR read zero on T20/T31 */
    env->active_tc.mxu_mir = 0;
    env->active_tc.mxu_mcsr = 0;
    restore_mxu_fp_status(env);

    /*
     * The manual describes IEEE 754-2008 behaviour throughout and gives no
     * legacy NaN encoding, so use the 2008 rules, as MSA does.
     */
    set_float_detect_tininess(float_tininess_after_rounding, st);
    set_float_ftz_detection(float_ftz_before_rounding, st);
    set_float_2nan_prop_rule(float_2nan_prop_s_ab, st);
    set_float_3nan_prop_rule(float_3nan_prop_s_cab, st);
    set_default_nan_mode(false, st);
    set_snan_rule(float_snan_bit_is_zero, st);
    set_float_infzeronan_rule(float_infzeronan_dnan_never, st);
    set_float_default_nan_pattern(0b01000000, st);
    set_float_exception_flags(0, st);
}
