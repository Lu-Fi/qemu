/*
 * Ingenic XBurst2 MXU3 (SIMD512) helpers
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "tcg/tcg-gvec-desc.h"
#include "mxu_internal.h"

/*
 * Unsigned a >= b for each w-bit field of a 64-bit unit, as a mask of the
 * fields. msb has the top bit of each field set. Comparing the low bits
 * with that bit set in a and clear in b keeps a field from borrowing from
 * the next one; the top bits then decide wherever they differ.
 */
static uint64_t swar_geu(uint64_t a, uint64_t b, uint64_t msb, int w)
{
    uint64_t low_ge = (a | msb) - (b & ~msb);
    uint64_t ge = ((a & ~b) | (~(a ^ b) & low_ge)) & msb;

    return ge | (ge - (ge >> (w - 1)));
}

#define DO_MINMAX_NBI(NAME, W, MSB, IS_MAX)                             \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    uint64_t *d = vd, *s = vs, *t = vt;                                 \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / 8; i++) {                    \
        uint64_t ge = swar_geu(s[i], t[i], MSB, W);                     \
        uint64_t sel = IS_MAX ? ge : ~ge;                               \
                                                                        \
        d[i] = (s[i] & sel) | (t[i] & ~sel);                            \
    }                                                                   \
}

DO_MINMAX_NBI(mxu3_maxu_2bi, 2, 0xaaaaaaaaaaaaaaaaull, true)
DO_MINMAX_NBI(mxu3_maxu_4bi, 4, 0x8888888888888888ull, true)
DO_MINMAX_NBI(mxu3_minu_2bi, 2, 0xaaaaaaaaaaaaaaaaull, false)
DO_MINMAX_NBI(mxu3_minu_4bi, 4, 0x8888888888888888ull, false)

/*
 * Widening operations: element i of the double-width result takes element
 * i of the low (data 0) or high (data 1) half of vt. vt is copied first,
 * as the result overwrites it when vd is vt.
 */
#define DO_WIDE_HALF(NAME, TW, HW, TN, HN, OP)                          \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    int n = simd_oprsz(desc) / sizeof(TW);                              \
    int half = simd_data(desc) ? n : 0;                                 \
    TW *d = vd, *s = vs;                                                \
    TN t[MXU3_VLEN / sizeof(TN)];                                       \
                                                                        \
    memcpy(t, vt, sizeof(t));                                           \
    for (int i = 0; i < n; i++) {                                       \
        d[HW(i)] = OP(s[HW(i)], (TW)t[HN(half + i)]);                   \
    }                                                                   \
}

#define DO_ADD(a, b)    ((a) + (b))
#define DO_SUB(a, b)    ((a) - (b))
#define DO_MUL(a, b)    ((uint32_t)(a) * (b))

DO_WIDE_HALF(mxu3_waddu_b, uint16_t, H2, uint8_t, H1, DO_ADD)
DO_WIDE_HALF(mxu3_waddu_h, uint32_t, H4, uint16_t, H2, DO_ADD)
DO_WIDE_HALF(mxu3_wadds_b, uint16_t, H2, int8_t, H1, DO_ADD)
DO_WIDE_HALF(mxu3_wadds_h, uint32_t, H4, int16_t, H2, DO_ADD)
DO_WIDE_HALF(mxu3_wsubu_b, uint16_t, H2, uint8_t, H1, DO_SUB)
DO_WIDE_HALF(mxu3_wsubu_h, uint32_t, H4, uint16_t, H2, DO_SUB)
DO_WIDE_HALF(mxu3_wsubs_b, uint16_t, H2, int8_t, H1, DO_SUB)
DO_WIDE_HALF(mxu3_wsubs_h, uint32_t, H4, int16_t, H2, DO_SUB)
DO_WIDE_HALF(mxu3_wumul_b, uint16_t, H2, uint8_t, H1, DO_MUL)
DO_WIDE_HALF(mxu3_wumul_h, uint32_t, H4, uint16_t, H2, DO_MUL)
DO_WIDE_HALF(mxu3_wsmul_b, uint16_t, H2, int8_t, H1, DO_MUL)
DO_WIDE_HALF(mxu3_wsmul_h, uint32_t, H4, int16_t, H2, DO_MUL)

/* Double-width products of the even (data 0) or odd (data 1) elements */
#define DO_MUL_EVEN_ODD(NAME, TW, HW, TN, HN)                           \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    int odd = simd_data(desc);                                          \
    TW *d = vd;                                                         \
    TN *s = vs, *t = vt;                                                \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / sizeof(TW); i++) {           \
        TW a = s[HN(2 * i + odd)], b = t[HN(2 * i + odd)];              \
                                                                        \
        d[HW(i)] = a * b;                                               \
    }                                                                   \
}

DO_MUL_EVEN_ODD(mxu3_umul_b, uint16_t, H2, uint8_t, H1)
DO_MUL_EVEN_ODD(mxu3_umul_h, uint32_t, H4, uint16_t, H2)
DO_MUL_EVEN_ODD(mxu3_smul_b, int16_t, H2, int8_t, H1)
DO_MUL_EVEN_ODD(mxu3_smul_h, int32_t, H4, int16_t, H2)
