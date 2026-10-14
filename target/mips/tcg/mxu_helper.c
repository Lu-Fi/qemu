/*
 * Ingenic XBurst MXU2/MXU3 SIMD helpers, shared parts
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "internal.h"
#include "exec/helper-proto.h"
#include "fpu/softfloat.h"
#include "fpu_helper.h"
#include "tcg-internal.h"
#include "mxu_internal.h"

float_status *mxu_fpst(CPUMIPSState *env)
{
    float_status *st = &env->active_tc.mxu_fp_status;

    set_float_exception_flags(0, st);
    return st;
}

void mxu_fp_end(CPUMIPSState *env, uintptr_t ra)
{
    uint32_t *mcsr = &env->active_tc.mxu_mcsr;
    int ieee = get_float_exception_flags(&env->active_tc.mxu_fp_status);
    int x = 0;

    if (ieee & float_flag_invalid) {
        x |= FP_INVALID;
    }
    if (ieee & float_flag_divbyzero) {
        x |= FP_DIV0;
    }
    if (ieee & float_flag_overflow) {
        x |= FP_OVERFLOW;
    }
    if (ieee & (float_flag_underflow | float_flag_output_denormal_flushed)) {
        x |= FP_UNDERFLOW;
    }
    if (ieee & (float_flag_inexact | float_flag_input_denormal_flushed |
                float_flag_output_denormal_flushed)) {
        x |= FP_INEXACT;
    }

    SET_FP_CAUSE(*mcsr, x);
    if (x & GET_FP_ENABLE(*mcsr)) {
        do_raise_exception(env, EXCP_MXUFPE, ra);
    }
    UPDATE_FP_FLAGS(*mcsr, x);
}

void HELPER(mxu_ctcmxu)(CPUMIPSState *env, uint32_t val)
{
    env->active_tc.mxu_mcsr = val & MXU_MCSR_MASK;
    restore_mxu_fp_status(env);
    if (GET_FP_CAUSE(env->active_tc.mxu_mcsr) &
        GET_FP_ENABLE(env->active_tc.mxu_mcsr)) {
        do_raise_exception(env, EXCP_MXUFPE, GETPC());
    }
}
