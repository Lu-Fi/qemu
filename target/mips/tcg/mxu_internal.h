/*
 * Ingenic XBurst MXU2/MXU3 SIMD helpers, shared parts
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef TARGET_MIPS_MXU_INTERNAL_H
#define TARGET_MIPS_MXU_INTERNAL_H

#include "fpu/softfloat-types.h"
#include "hw/core/registerfields.h"

/*
 * Vector registers are arrays of host-endian uint64_t, the first holding
 * the least significant bits, so on a big-endian host narrower elements
 * are reversed within each 64-bit unit.
 */
#if HOST_BIG_ENDIAN
#define H1(x)   ((x) ^ 7)
#define H2(x)   ((x) ^ 3)
#define H4(x)   ((x) ^ 1)
#else
#define H1(x)   (x)
#define H2(x)   (x)
#define H4(x)   (x)
#endif
#define H8(x)   (x)

/* Element-wise operations over the whole register, sized by simd_oprsz() */
#define MXU_DO_3OP(NAME, TYPE, H, OP)                                   \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / sizeof(TYPE); i++) {         \
        d[H(i)] = OP(s[H(i)], t[H(i)]);                                 \
    }                                                                   \
}

#define MXU_DO_3OP_BHWD(NAME, SIGN, OP)                                 \
    MXU_DO_3OP(NAME##_b, SIGN##int8_t, H1, OP)                          \
    MXU_DO_3OP(NAME##_h, SIGN##int16_t, H2, OP)                         \
    MXU_DO_3OP(NAME##_w, SIGN##int32_t, H4, OP)                         \
    MXU_DO_3OP(NAME##_d, SIGN##int64_t, H8, OP)

static inline uint64_t mxu_umag(int64_t x)
{
    return x < 0 ? -(uint64_t)x : x;
}

/*
 * MXU3 segmented sums: the helper data gives log2 of the element size in
 * bits, the signedness of the vs and vp elements, log2 of the segment
 * length in words, the word of the segment that accumulates, and the
 * segment of vp to use for every segment of vs (REPL) or the matching one.
 */
FIELD(MXU3_SUM, ESZ, 0, 3)
FIELD(MXU3_SUM, SSIGN, 3, 1)
FIELD(MXU3_SUM, PSIGN, 4, 1)
FIELD(MXU3_SUM, LE, 5, 3)
FIELD(MXU3_SUM, M, 8, 4)
FIELD(MXU3_SUM, N, 12, 4)
FIELD(MXU3_SUM, REPL, 16, 1)

/*
 * Floating point: mxu_fpst() clears the accumulated exception flags and
 * returns the status to compute with; mxu_fp_end() then sets MCSR.Cause,
 * raises the MXU floating-point exception for an enabled cause and
 * otherwise accumulates MCSR.Flags.
 */
float_status *mxu_fpst(CPUMIPSState *env);
void mxu_fp_end(CPUMIPSState *env, uintptr_t ra);

#endif
