/*
 * Ingenic XBurst MXU2 (128-bit SIMD) helpers
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * Reference: XBurst Instruction Set Architecture MXU2 Programming Manual,
 * revision 2.16. Where the manual's pseudo-code and the silicon disagree,
 * these follow the silicon (T20/T31).
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "cpu.h"
#include "internal.h"
#include "exec/helper-proto.h"
#include "fpu/softfloat.h"
#include "fpu_helper.h"
#include "tcg-internal.h"
#include "tcg/tcg-gvec-desc.h"

/*
 * A VPR is two host-endian uint64_t, the first holding bits 63:0, so on a
 * big-endian host narrower elements are reversed within each half.
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

#define MXU2_VLEN 16

/* Element-wise integer operations */

#define DO_3OP(NAME, TYPE, H, OP)                                       \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = OP(s[H(i)], t[H(i)]);                                 \
    }                                                                   \
}

#define DO_3OP_BHWD(NAME, SIGN, OP)                                     \
    DO_3OP(mxu2_##NAME##_b, SIGN##int8_t, H1, OP)                       \
    DO_3OP(mxu2_##NAME##_h, SIGN##int16_t, H2, OP)                      \
    DO_3OP(mxu2_##NAME##_w, SIGN##int32_t, H4, OP)                      \
    DO_3OP(mxu2_##NAME##_d, SIGN##int64_t, H8, OP)

static inline uint64_t umag(int64_t x)
{
    return x < 0 ? -(uint64_t)x : x;
}

#define DO_MAXA(a, b)   (umag(a) > umag(b) ? (a) : (b))
#define DO_MINA(a, b)   (umag(a) < umag(b) ? (a) : (b))
#define DO_ADDA(a, b)   (umag(a) + umag(b))

DO_3OP_BHWD(maxa, , DO_MAXA)
DO_3OP_BHWD(mina, , DO_MINA)
DO_3OP_BHWD(adda, , DO_ADDA)

/* |a - b| without overflow, for signed and unsigned operands */
#define DO_SUBSA(a, b) \
    ((int64_t)(a) >= (int64_t)(b) ? (uint64_t)(a) - (b) : (uint64_t)(b) - (a))
#define DO_SUBUA(a, b) \
    ((uint64_t)(a) >= (uint64_t)(b) ? (uint64_t)(a) - (b) : (uint64_t)(b) - (a))

DO_3OP_BHWD(subsa, , DO_SUBSA)
DO_3OP_BHWD(subua, u, DO_SUBUA)

/* Averages are computed without forming the possibly overflowing sum */
#define DO_AVE(a, b)    (((a) >> 1) + ((b) >> 1) + ((a) & (b) & 1))
#define DO_AVER(a, b)   (((a) >> 1) + ((b) >> 1) + (((a) | (b)) & 1))

DO_3OP_BHWD(aves, , DO_AVE)
DO_3OP_BHWD(avers, , DO_AVER)
DO_3OP_BHWD(aveu, u, DO_AVE)
DO_3OP_BHWD(averu, u, DO_AVER)

/*
 * Division by zero is UNPREDICTABLE; follow the MIPS/MSA convention of
 * all ones (or one, for a negative dividend) and the dividend as remainder.
 */
#define DO_DIVS(a, b)   ((b) == 0 ? ((a) >= 0 ? -1 : 1) :                   \
                         (b) == -1 ? -(uint64_t)(a) : (a) / (b))
#define DO_DIVU(a, b)   ((b) == 0 ? -1 : (a) / (b))
#define DO_MODS(a, b)   ((b) == 0 ? (a) : (b) == -1 ? 0 : (a) % (b))
#define DO_MODU(a, b)   ((b) == 0 ? (a) : (a) % (b))

DO_3OP_BHWD(divs, , DO_DIVS)
DO_3OP_BHWD(divu, u, DO_DIVU)
DO_3OP_BHWD(mods, , DO_MODS)
DO_3OP_BHWD(modu, u, DO_MODU)

/* Signed saturation of a sum of magnitudes, and of an unsigned difference */

#define DO_SAT_3OP(NAME, TYPE, H, OP, MIN, MAX)                         \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = OP(s[H(i)], t[H(i)], MIN, MAX);                       \
    }                                                                   \
}

static inline int64_t addas(int64_t a, int64_t b, int64_t min, int64_t max)
{
    uint64_t ma = umag(a), mb = umag(b), sum = ma + mb;

    return sum < ma || sum > max ? max : sum;
}

static inline int64_t subus(uint64_t a, uint64_t b, int64_t min, int64_t max)
{
    if (a >= b) {
        return a - b > max ? max : a - b;
    }
    return b - a > (uint64_t)max + 1 ? min : -(int64_t)(b - a);
}

DO_SAT_3OP(mxu2_addas_b, int8_t, H1, addas, INT8_MIN, INT8_MAX)
DO_SAT_3OP(mxu2_addas_h, int16_t, H2, addas, INT16_MIN, INT16_MAX)
DO_SAT_3OP(mxu2_addas_w, int32_t, H4, addas, INT32_MIN, INT32_MAX)
DO_SAT_3OP(mxu2_addas_d, int64_t, H8, addas, INT64_MIN, INT64_MAX)
DO_SAT_3OP(mxu2_subus_b, uint8_t, H1, subus, INT8_MIN, INT8_MAX)
DO_SAT_3OP(mxu2_subus_h, uint16_t, H2, subus, INT16_MIN, INT16_MAX)
DO_SAT_3OP(mxu2_subus_w, uint32_t, H4, subus, INT32_MIN, INT32_MAX)
DO_SAT_3OP(mxu2_subus_d, uint64_t, H8, subus, INT64_MIN, INT64_MAX)

/* Multiply-accumulate keeps the low half of the product */

#define DO_MAC(NAME, TYPE, H, OP)                                       \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = OP(d[H(i)], s[H(i)], t[H(i)]);                        \
    }                                                                   \
}

#define DO_MADD(acc, a, b)  ((acc) + (uint64_t)(a) * (b))
#define DO_MSUB(acc, a, b)  ((acc) - (uint64_t)(a) * (b))

DO_MAC(mxu2_madd_b, uint8_t, H1, DO_MADD)
DO_MAC(mxu2_madd_h, uint16_t, H2, DO_MADD)
DO_MAC(mxu2_madd_w, uint32_t, H4, DO_MADD)
DO_MAC(mxu2_madd_d, uint64_t, H8, DO_MADD)
DO_MAC(mxu2_msub_b, uint8_t, H1, DO_MSUB)
DO_MAC(mxu2_msub_h, uint16_t, H2, DO_MSUB)
DO_MAC(mxu2_msub_w, uint32_t, H4, DO_MSUB)
DO_MAC(mxu2_msub_d, uint64_t, H8, DO_MSUB)

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
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
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
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = DO_SHR_ROUND(s[H(i)], n);                             \
    }                                                                   \
}

DO_SHR_ROUND_3OP(mxu2_srar_b, int8_t, H1)
DO_SHR_ROUND_3OP(mxu2_srar_h, int16_t, H2)
DO_SHR_ROUND_3OP(mxu2_srar_w, int32_t, H4)
DO_SHR_ROUND_3OP(mxu2_srar_d, int64_t, H8)
DO_SHR_ROUND_3OP(mxu2_srlr_b, uint8_t, H1)
DO_SHR_ROUND_3OP(mxu2_srlr_h, uint16_t, H2)
DO_SHR_ROUND_3OP(mxu2_srlr_w, uint32_t, H4)
DO_SHR_ROUND_3OP(mxu2_srlr_d, uint64_t, H8)
DO_SHR_ROUND_IMM(mxu2_srari_b, int8_t, H1)
DO_SHR_ROUND_IMM(mxu2_srari_h, int16_t, H2)
DO_SHR_ROUND_IMM(mxu2_srari_w, int32_t, H4)
DO_SHR_ROUND_IMM(mxu2_srari_d, int64_t, H8)
DO_SHR_ROUND_IMM(mxu2_srlri_b, uint8_t, H1)
DO_SHR_ROUND_IMM(mxu2_srlri_h, uint16_t, H2)
DO_SHR_ROUND_IMM(mxu2_srlri_w, uint32_t, H4)
DO_SHR_ROUND_IMM(mxu2_srlri_d, uint64_t, H8)

/* Saturate to imm + 1 bits without changing the element width */

#define DO_SAT_IMM(NAME, TYPE, H, OP)                                   \
void HELPER(NAME)(void *vd, void *vs, uint32_t desc)                    \
{                                                                       \
    TYPE *d = vd, *s = vs;                                              \
    int bits = (simd_data(desc) & (sizeof(TYPE) * 8 - 1)) + 1;          \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = OP(s[H(i)], bits);                                    \
    }                                                                   \
}

static inline int64_t sats(int64_t x, int bits)
{
    int64_t max = bits == 64 ? INT64_MAX : (INT64_C(1) << (bits - 1)) - 1;

    return x > max ? max : x < -max - 1 ? -max - 1 : x;
}

static inline uint64_t satu(uint64_t x, int bits)
{
    uint64_t max = bits == 64 ? UINT64_MAX : (UINT64_C(1) << bits) - 1;

    return x > max ? max : x;
}

DO_SAT_IMM(mxu2_sats_b, int8_t, H1, sats)
DO_SAT_IMM(mxu2_sats_h, int16_t, H2, sats)
DO_SAT_IMM(mxu2_sats_w, int32_t, H4, sats)
DO_SAT_IMM(mxu2_sats_d, int64_t, H8, sats)
DO_SAT_IMM(mxu2_satu_b, uint8_t, H1, satu)
DO_SAT_IMM(mxu2_satu_h, uint16_t, H2, satu)
DO_SAT_IMM(mxu2_satu_w, uint32_t, H4, satu)
DO_SAT_IMM(mxu2_satu_d, uint64_t, H8, satu)

/* Bit counts */

#define DO_2OP(NAME, TYPE, H, OP)                                       \
void HELPER(NAME)(void *vd, void *vs, uint32_t desc)                    \
{                                                                       \
    TYPE *d = vd, *s = vs;                                              \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = OP(s[H(i)]);                                          \
    }                                                                   \
}

#define DO_LZC_B(x)     (clz32(x) - 24)
#define DO_LZC_H(x)     (clz32(x) - 16)
#define DO_LOC_B(x)     (clz32((uint8_t)~(x)) - 24)
#define DO_LOC_H(x)     (clz32((uint16_t)~(x)) - 16)

DO_2OP(mxu2_lzc_b, uint8_t, H1, DO_LZC_B)
DO_2OP(mxu2_lzc_h, uint16_t, H2, DO_LZC_H)
DO_2OP(mxu2_lzc_w, uint32_t, H4, clz32)
DO_2OP(mxu2_lzc_d, uint64_t, H8, clz64)
DO_2OP(mxu2_loc_b, uint8_t, H1, DO_LOC_B)
DO_2OP(mxu2_loc_h, uint16_t, H2, DO_LOC_H)
DO_2OP(mxu2_loc_w, uint32_t, H4, clo32)
DO_2OP(mxu2_loc_d, uint64_t, H8, clo64)
DO_2OP(mxu2_bcnt_b, uint8_t, H1, ctpop8)
DO_2OP(mxu2_bcnt_h, uint16_t, H2, ctpop16)
DO_2OP(mxu2_bcnt_w, uint32_t, H4, ctpop32)
DO_2OP(mxu2_bcnt_d, uint64_t, H8, ctpop64)

/*
 * Dot products: each element is the sum of the products of the two
 * half-width elements it overlaps. Those inputs live in the bytes of the
 * output element, so writing it in place cannot clobber later inputs.
 */

#define DO_DOT(NAME, TYPE, TYPEN, H, HN, PTYPE, OP)                     \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd;                                                       \
    TYPEN *s = vs, *t = vt;                                             \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        TYPE p0 = (PTYPE)s[HN(2 * i)] * t[HN(2 * i)];                   \
        TYPE p1 = (PTYPE)s[HN(2 * i + 1)] * t[HN(2 * i + 1)];           \
        d[H(i)] = OP(d[H(i)], p0 + p1);                                 \
    }                                                                   \
}

#define DO_DOT_HWD(NAME, SIGN, OP)                                      \
    DO_DOT(mxu2_##NAME##_h, uint16_t, SIGN##int8_t, H2, H1,             \
           SIGN##int64_t, OP)                                           \
    DO_DOT(mxu2_##NAME##_w, uint32_t, SIGN##int16_t, H4, H2,            \
           SIGN##int64_t, OP)                                           \
    DO_DOT(mxu2_##NAME##_d, uint64_t, SIGN##int32_t, H8, H4,            \
           SIGN##int64_t, OP)

#define DOT_SET(acc, p)     (p)
#define DOT_ADD(acc, p)     ((acc) + (p))
#define DOT_SUB(acc, p)     ((acc) - (p))

DO_DOT_HWD(dotps, , DOT_SET)
DO_DOT_HWD(dotpu, u, DOT_SET)
DO_DOT_HWD(dadds, , DOT_ADD)
DO_DOT_HWD(daddu, u, DOT_ADD)
DO_DOT_HWD(dsubs, , DOT_SUB)
DO_DOT_HWD(dsubu, u, DOT_SUB)

/*
 * Q15/Q31 fixed-point multiplies. The accumulating forms add the unrounded
 * product to the accumulator scaled up by 2^(esize - 1) and truncate the
 * sum, so MSUBQ yields acc - ceil(a * b / 2^(esize - 1)): it is not
 * acc - MULQ(a, b) (verified on T20).
 */

static inline int64_t q_clamp(int64_t x, int bits)
{
    int64_t max = (INT64_C(1) << (bits - 1)) - 1;

    return x > max ? max : x < -max - 1 ? -max - 1 : x;
}

static inline int64_t q_mul(int64_t a, int64_t b, int bits, bool round)
{
    int64_t r = round ? INT64_C(1) << (bits - 2) : 0;

    return q_clamp((a * b + r) >> (bits - 1), bits);
}

static inline int64_t q_mac(int64_t acc, int64_t a, int64_t b, int bits,
                            bool sub, bool round)
{
    int64_t p = sub ? -(a * b) : a * b;
    int64_t r = round ? INT64_C(1) << (bits - 2) : 0;

    return q_clamp((acc * (INT64_C(1) << (bits - 1)) + p + r) >> (bits - 1),
                   bits);
}

#define DO_QMUL(NAME, TYPE, H, ROUND)                                   \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = q_mul(s[H(i)], t[H(i)], sizeof(TYPE) * 8, ROUND);     \
    }                                                                   \
}

#define DO_QMAC(NAME, TYPE, H, SUB, ROUND)                              \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    TYPE *d = vd, *s = vs, *t = vt;                                     \
                                                                        \
    for (int i = 0; i < MXU2_VLEN / sizeof(TYPE); i++) {                \
        d[H(i)] = q_mac(d[H(i)], s[H(i)], t[H(i)], sizeof(TYPE) * 8,    \
                        SUB, ROUND);                                    \
    }                                                                   \
}

DO_QMUL(mxu2_mulq_h, int16_t, H2, false)
DO_QMUL(mxu2_mulq_w, int32_t, H4, false)
DO_QMUL(mxu2_mulqr_h, int16_t, H2, true)
DO_QMUL(mxu2_mulqr_w, int32_t, H4, true)
DO_QMAC(mxu2_maddq_h, int16_t, H2, false, false)
DO_QMAC(mxu2_maddq_w, int32_t, H4, false, false)
DO_QMAC(mxu2_maddqr_h, int16_t, H2, false, true)
DO_QMAC(mxu2_maddqr_w, int32_t, H4, false, true)
DO_QMAC(mxu2_msubq_h, int16_t, H2, true, false)
DO_QMAC(mxu2_msubq_w, int32_t, H4, true, false)
DO_QMAC(mxu2_msubqr_h, int16_t, H2, true, true)
DO_QMAC(mxu2_msubqr_w, int32_t, H4, true, true)

/*
 * SHUFV: byte i of vd is byte sel % 32 of the interleaving of vt (even
 * bytes) and vs (odd bytes), or zero when bit 7 of the selector is set.
 */
void HELPER(mxu2_shufv)(void *vd, void *vs, void *vt, void *vr, uint32_t desc)
{
    uint8_t *d = vd, *s = vs, *t = vt, *r = vr;
    uint8_t cat[2 * MXU2_VLEN], res[MXU2_VLEN];

    for (int i = 0; i < MXU2_VLEN; i++) {
        cat[2 * i] = t[H1(i)];
        cat[2 * i + 1] = s[H1(i)];
    }
    for (int i = 0; i < MXU2_VLEN; i++) {
        uint8_t sel = r[H1(i)];

        res[i] = sel & 0x80 ? 0 : cat[sel & 31];
    }
    for (int i = 0; i < MXU2_VLEN; i++) {
        d[H1(i)] = res[i];
    }
}

void HELPER(mxu2_ctcmxu)(CPUMIPSState *env, uint32_t val)
{
    env->active_tc.mxu2_mcsr = val & MXU2_MCSR_MASK;
    restore_mxu2_fp_status(env);
    if (GET_FP_CAUSE(env->active_tc.mxu2_mcsr) &
        GET_FP_ENABLE(env->active_tc.mxu2_mcsr)) {
        do_raise_exception(env, EXCP_MXUFPE, GETPC());
    }
}
