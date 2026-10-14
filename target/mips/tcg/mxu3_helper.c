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
