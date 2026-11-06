/*
 * Ingenic AIE vector extension (XIngenicAIE) helpers
 *
 * The T32Pro's TNPU cores' vectors are 128 bits: Ingenic's kernels for it
 * step 16 bytes a vector. Predicate bit i enables element i whatever the
 * element's width (one pdec serves every width); pr0 reads all ones, and
 * an element a predicate leaves out keeps the destination's value. The
 * operations are as Ingenic's compiler expresses its builtins (GCC RTL),
 * where it does; the rest are named below.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"
#include "fpu/softfloat.h"
#include "xingenicaie.h"

typedef uint8_t AIEVec[AIE_VLENB];

static uint64_t vget(const uint8_t *v, int esz, int i)
{
    switch (esz) {
    case 0:
        return v[i];
    case 1:
        return lduw_le_p(v + 2 * i);
    case 2:
        return ldl_le_p(v + 4 * i);
    default:
        return ldq_le_p(v + 8 * i);
    }
}

static int64_t vgets(const uint8_t *v, int esz, int i)
{
    int bits = 8 << esz;

    return sextract64(vget(v, esz, i), 0, bits);
}

static void vset(uint8_t *v, int esz, int i, uint64_t x)
{
    switch (esz) {
    case 0:
        v[i] = x;
        break;
    case 1:
        stw_le_p(v + 2 * i, x);
        break;
    case 2:
        stl_le_p(v + 4 * i, x);
        break;
    default:
        stq_le_p(v + 8 * i, x);
        break;
    }
}

static uint32_t aie_pred(CPURISCVState *env, int pg)
{
    return pg ? env->aie_pr[pg] : 0xffff;
}

static void aie_set_pred(CPURISCVState *env, int pd, uint32_t p)
{
    if (pd) {
        env->aie_pr[pd] = p;
    }
}

static void merge(uint8_t *vd, const uint8_t *r, int esz, uint32_t pred)
{
    int n = AIE_VLENB >> esz;

    for (int i = 0; i < n; i++) {
        if (pred & (1u << i)) {
            memcpy(vd + (i << esz), r + (i << esz), 1 << esz);
        }
    }
}

static uint64_t int_op(int op, int esz, uint64_t a, uint64_t b)
{
    int bits = 8 << esz;
    int64_t sa = sextract64(a, 0, bits), sb = sextract64(b, 0, bits);

    switch (op) {
    case AIE_ADD:
        return a + b;
    case AIE_SUB:
        return a - b;
    case AIE_MUL:
        return a * b;
    case AIE_AND:
        return a & b;
    case AIE_OR:
        return a | b;
    case AIE_XOR:
        return a ^ b;
    case AIE_MAXU:
        return MAX(a, b);
    case AIE_MINU:
        return MIN(a, b);
    case AIE_MAXS:
        return MAX(sa, sb);
    case AIE_MINS:
        return MIN(sa, sb);
    case AIE_MAXU4:
        return MAX(a & 0xf, b & 0xf) | MAX(a & 0xf0, b & 0xf0);
    default:
        g_assert_not_reached();
    }
}

static uint32_t fp_op(int op, uint32_t a, uint32_t b, float_status *s)
{
    switch (op) {
    case AIE_FADD:
        return float32_add(a, b, s);
    case AIE_FSUB:
        return float32_sub(a, b, s);
    case AIE_FMUL:
        return float32_mul(a, b, s);
    case AIE_FMAX:
        return float32_maxnum(a, b, s);
    case AIE_FMIN:
        return float32_minnum(a, b, s);
    case AIE_FCLE:
        return float32_le_quiet(a, b, s) ? UINT32_MAX : 0;
    case AIE_FCLT:
        return float32_lt_quiet(a, b, s) ? UINT32_MAX : 0;
    default:
        g_assert_not_reached();
    }
}

void HELPER(aie_vop)(CPURISCVState *env, uint32_t desc, uint32_t imm,
                     target_ulong x)
{
    int op = AIE_DESC_OP(desc), esz = AIE_DESC_ESZ(desc);
    int pg = AIE_DESC_PG(desc), vdn = AIE_DESC_VD(desc);
    uint8_t *vd = env->aie_vp[vdn];
    const uint8_t *s1 = env->aie_vp[AIE_DESC_VS1(desc)];
    const uint8_t *s2 = env->aie_vp[AIE_DESC_VS2(desc)];
    uint32_t pred = aie_pred(env, pg);
    int n = AIE_VLENB >> esz, bits = 8 << esz;
    float_status fs = env->fp_status;
    AIEVec r;
    uint32_t p;

    /* AIE rounds to nearest, whatever frm holds */
    set_float_rounding_mode(float_round_nearest_even, &fs);
    memcpy(r, vd, AIE_VLENB);

    switch (op) {
    case AIE_ADD ... AIE_MAXU4:
        for (int i = 0; i < n; i++) {
            vset(r, esz, i, int_op(op, esz, vget(s1, esz, i),
                                   vget(s2, esz, i)));
        }
        break;
    case AIE_UMULE:
    case AIE_UMULO:
        /* halves 2i (even) or 2i + 1 (odd), widened to words */
        for (int i = 0; i < n; i++) {
            int j = 2 * i + (op == AIE_UMULO);

            vset(r, esz, i, vget(s1, esz - 1, j) * vget(s2, esz - 1, j));
        }
        break;
    case AIE_FADD ... AIE_FCLT:
        for (int i = 0; i < 4; i++) {
            vset(r, 2, i, fp_op(op, vget(s1, 2, i), vget(s2, 2, i), &fs));
        }
        if (op == AIE_FCLE || op == AIE_FCLT) {
            pred = 0xffff;
        }
        break;
    case AIE_ILVE:
    case AIE_ILVO:
        /* vs1's even (odd) elements to the even ones, vs2's to the odd */
        for (int i = 0; i < n / 2; i++) {
            int j = 2 * i + (op == AIE_ILVO);

            vset(r, esz, 2 * i, vget(s1, esz, j));
            vset(r, esz, 2 * i + 1, vget(s2, esz, j));
        }
        break;
    case AIE_GSHUF:
        /*
         * vs1 picks vs2's bytes; bit 7 makes a zero. The vendor's tables
         * index 0 to 15 only: 16 to 31 taken as the old vd's is a guess.
         */
        for (int i = 0; i < AIE_VLENB; i++) {
            uint8_t k = s1[i];

            r[i] = k & 0x80 ? 0 : (k & 0x10 ? vd[k & 0xf] : s2[k & 0xf]);
        }
        break;
    case AIE_BSEL:
        /* vd's set bits take vs2's, its clear bits vs1's */
        for (int i = 0; i < AIE_VLENB; i++) {
            r[i] = (vd[i] & s2[i]) | (~vd[i] & s1[i]);
        }
        break;
    case AIE_ORI:
    case AIE_XORI:
        for (int i = 0; i < AIE_VLENB; i++) {
            r[i] = op == AIE_ORI ? s1[i] | imm : s1[i] ^ imm;
        }
        break;
    case AIE_SLLI:
    case AIE_SRLI:
    case AIE_SRAI:
        for (int i = 0; i < n; i++) {
            int sh = imm % bits;
            uint64_t v = op == AIE_SLLI ? vget(s1, esz, i) << sh :
                         op == AIE_SRLI ? vget(s1, esz, i) >> sh :
                         (uint64_t)(vgets(s1, esz, i) >> sh);

            vset(r, esz, i, v);
        }
        break;
    case AIE_SATUU:
        /* clamped in its own element, imm the bits it is clamped to */
        for (int i = 0; i < n; i++) {
            vset(r, esz, i, MIN(vget(s1, esz, i), MAKE_64BIT_MASK(0, imm)));
        }
        break;
    case AIE_FFSI:
    case AIE_FFUI:
    case AIE_FTSI:
    case AIE_FTUI:
        for (int i = 0; i < 4; i++) {
            uint32_t v = vget(s1, 2, i);

            v = op == AIE_FFSI ? int32_to_float32(v, &fs) :
                op == AIE_FFUI ? uint32_to_float32(v, &fs) :
                op == AIE_FTSI ? float32_to_int32(v, &fs) :
                float32_to_uint32(v, &fs);
            vset(r, 2, i, v);
        }
        break;
    case AIE_EXTUL ... AIE_EXTSH:
        /* the low (high) half's elements, widened to esz */
        for (int i = 0; i < n; i++) {
            int j = i + (op == AIE_EXTUH || op == AIE_EXTSH ? n : 0);
            bool sgn = op == AIE_EXTSL || op == AIE_EXTSH;

            vset(r, esz, i, sgn ? vgets(s2, esz - 1, j) : vget(s2, esz - 1, j));
        }
        break;
    case AIE_EXTU4L:
        /* the low half's nibbles to bytes, each byte's low nibble first */
        for (int i = 0; i < AIE_VLENB; i++) {
            r[i] = (s2[i / 2] >> (4 * (i & 1))) & 0xf;
        }
        break;
    case AIE_GT:
        /*
         * The even elements packed in the low half. The high half's odd
         * elements are a guess: the vendor's kernels use the low half alone.
         */
        for (int i = 0; i < n / 2; i++) {
            vset(r, esz, i, vget(s2, esz, 2 * i));
            vset(r, esz, i + n / 2, vget(s2, esz, 2 * i + 1));
        }
        break;
    case AIE_GT2W:
        /* vd's half S (vs1 field) takes vs2's half I (imm) */
        memcpy(r + AIE_VLENB / 2 * AIE_DESC_VS1(desc),
               s2 + AIE_VLENB / 2 * imm, AIE_VLENB / 2);
        break;
    case AIE_PID:
        for (int i = 0; i < n; i++) {
            vset(r, esz, i, i);
        }
        break;
    case AIE_REPI:
        for (int i = 0; i < n; i++) {
            vset(r, esz, i, vget(s2, esz, imm % n));
        }
        break;
    case AIE_MOVE:
        /* element vs1 (an index) of vd takes element imm of vs2 */
        vset(r, esz, AIE_DESC_VS1(desc) % n, vget(s2, esz, imm % n));
        break;
    case AIE_LI:
        for (int i = 0; i < n; i++) {
            vset(r, esz, i, (int64_t)(int32_t)imm);
        }
        break;
    case AIE_MFCPU:
        for (int i = 0; i < n; i++) {
            vset(r, esz, i, x);
        }
        break;
    case AIE_PDEC:
        p = (int32_t)x <= 0 ? 0 : MAKE_64BIT_MASK(0, MIN((int32_t)x, 16));
        aie_set_pred(env, vdn, p);
        return;
    case AIE_PCLTU:
    case AIE_PCLEU:
        p = env->aie_pr[vdn] & ~pred;
        for (int i = 0; i < n; i++) {
            uint64_t a = vget(s1, esz, i), b = vget(s2, esz, i);

            if ((pred & (1u << i)) && (op == AIE_PCLTU ? a < b : a <= b)) {
                p |= 1u << i;
            }
        }
        aie_set_pred(env, vdn, p);
        return;
    case AIE_PAND:
    case AIE_PORN:
        p = aie_pred(env, AIE_DESC_VS1(desc));
        p = op == AIE_PAND ? p & aie_pred(env, AIE_DESC_VS2(desc)) :
                             p | ~aie_pred(env, AIE_DESC_VS2(desc));
        aie_set_pred(env, vdn, p & 0xffff);
        return;
    default:
        g_assert_not_reached();
    }
    merge(vd, r, esz, pred);
}

/*
 * Element by element, so that an element a predicate leaves out is never
 * touched and a fault leaves the registers as they were.
 */
void HELPER(aie_ld)(CPURISCVState *env, uint32_t desc, uint32_t idx,
                    target_ulong addr)
{
    int op = AIE_DESC_OP(desc), esz = AIE_DESC_ESZ(desc);
    int vdn = AIE_DESC_VD(desc);
    uint32_t pred = aie_pred(env, AIE_DESC_PG(desc));
    uintptr_t ra = GETPC();
    AIEVec r;

    switch (op) {
    case AIE_MEMV:
        if (pred == 0xffff) {
            stq_le_p(r, cpu_ldq_le_data_ra(env, addr, ra));
            stq_le_p(r + 8, cpu_ldq_le_data_ra(env, addr + 8, ra));
            memcpy(env->aie_vp[vdn], r, AIE_VLENB);
            break;
        }
        memcpy(r, env->aie_vp[vdn], AIE_VLENB);
        for (int i = 0; i < AIE_VLENB >> esz; i++) {
            if (pred & (1u << i)) {
                for (int b = 0; b < 1 << esz; b++) {
                    r[(i << esz) + b] =
                        cpu_ldub_data_ra(env, addr + (i << esz) + b, ra);
                }
            }
        }
        memcpy(env->aie_vp[vdn], r, AIE_VLENB);
        break;
    case AIE_MEME:
        idx %= AIE_VLENB >> esz;
        if (pred & (1u << idx)) {
            uint64_t v = esz == 2 ? cpu_ldl_le_data_ra(env, addr, ra) :
                                    cpu_ldq_le_data_ra(env, addr, ra);

            vset(env->aie_vp[vdn], esz, idx, v);
        }
        break;
    case AIE_MEMP:
        aie_set_pred(env, vdn, cpu_lduw_le_data_ra(env, addr, ra));
        break;
    default:
        g_assert_not_reached();
    }
}

void HELPER(aie_st)(CPURISCVState *env, uint32_t desc, uint32_t idx,
                    target_ulong addr)
{
    int op = AIE_DESC_OP(desc), esz = AIE_DESC_ESZ(desc);
    const uint8_t *vs = env->aie_vp[AIE_DESC_VD(desc)];
    uint32_t pred = aie_pred(env, AIE_DESC_PG(desc));
    uintptr_t ra = GETPC();

    switch (op) {
    case AIE_MEMV:
        if (pred == 0xffff) {
            cpu_stq_le_data_ra(env, addr, ldq_le_p(vs), ra);
            cpu_stq_le_data_ra(env, addr + 8, ldq_le_p(vs + 8), ra);
            break;
        }
        for (int i = 0; i < AIE_VLENB >> esz; i++) {
            if (pred & (1u << i)) {
                for (int b = 0; b < 1 << esz; b++) {
                    cpu_stb_data_ra(env, addr + (i << esz) + b,
                                    vs[(i << esz) + b], ra);
                }
            }
        }
        break;
    case AIE_MEME:
        idx %= AIE_VLENB >> esz;
        if (pred & (1u << idx)) {
            uint64_t v = vget(vs, esz, idx);

            if (esz == 2) {
                cpu_stl_le_data_ra(env, addr, v, ra);
            } else {
                cpu_stq_le_data_ra(env, addr, v, ra);
            }
        }
        break;
    case AIE_MEMP:
        cpu_stw_le_data_ra(env, addr, aie_pred(env, AIE_DESC_VD(desc)), ra);
        break;
    default:
        g_assert_not_reached();
    }
}
