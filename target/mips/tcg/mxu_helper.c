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
#include "tcg/tcg-gvec-desc.h"
#include "mxu_internal.h"

/* Keep the operand with the larger or smaller magnitude */
#define DO_MAXA(a, b)   (mxu_umag(a) > mxu_umag(b) ? (a) : (b))
#define DO_MINA(a, b)   (mxu_umag(a) < mxu_umag(b) ? (a) : (b))

MXU_DO_3OP_BHWD(mxu_maxa, , DO_MAXA)
MXU_DO_3OP_BHWD(mxu_mina, , DO_MINA)

/*
 * Shift right by the low log2(esize) bits of the count, then add the most
 * significant bit shifted out.
 */
#define DO_SHR_ROUND(x, n) \
    ((n) ? ((x) >> (n)) + (((x) >> ((n) - 1)) & 1) : (x))

#define DO_SHR_ROUND_3OP(NAME, TYPE, H)                                 \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / sizeof(TYPE); i++) {         \
        int n = t[H(i)] & (sizeof(TYPE) * 8 - 1);                       \
        d[H(i)] = DO_SHR_ROUND(s[H(i)], n);                             \
    }                                                                   \
}

#define DO_SHR_ROUND_IMM(NAME, TYPE, H)                                 \
void HELPER(NAME)(void *vd, void *vs, uint32_t desc)                    \
{                                                                       \
    TYPE *d = vd, *s = vs;                                              \
    int n = simd_data(desc) & (sizeof(TYPE) * 8 - 1);                   \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / sizeof(TYPE); i++) {         \
        d[H(i)] = DO_SHR_ROUND(s[H(i)], n);                             \
    }                                                                   \
}

DO_SHR_ROUND_3OP(mxu_srar_b, int8_t, H1)
DO_SHR_ROUND_3OP(mxu_srar_h, int16_t, H2)
DO_SHR_ROUND_3OP(mxu_srar_w, int32_t, H4)
DO_SHR_ROUND_3OP(mxu_srar_d, int64_t, H8)
DO_SHR_ROUND_3OP(mxu_srlr_b, uint8_t, H1)
DO_SHR_ROUND_3OP(mxu_srlr_h, uint16_t, H2)
DO_SHR_ROUND_3OP(mxu_srlr_w, uint32_t, H4)
DO_SHR_ROUND_3OP(mxu_srlr_d, uint64_t, H8)
DO_SHR_ROUND_IMM(mxu_srari_b, int8_t, H1)
DO_SHR_ROUND_IMM(mxu_srari_h, int16_t, H2)
DO_SHR_ROUND_IMM(mxu_srari_w, int32_t, H4)
DO_SHR_ROUND_IMM(mxu_srari_d, int64_t, H8)
DO_SHR_ROUND_IMM(mxu_srlri_b, uint8_t, H1)
DO_SHR_ROUND_IMM(mxu_srlri_h, uint16_t, H2)
DO_SHR_ROUND_IMM(mxu_srlri_w, uint32_t, H4)
DO_SHR_ROUND_IMM(mxu_srlri_d, uint64_t, H8)

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
