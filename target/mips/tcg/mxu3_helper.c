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
#include "fpu/softfloat.h"
#include "fpu_helper.h"
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

/* Accumulate the products of the even (data 0) or odd (data 1) halfwords */
#define DO_MAC_EVEN_ODD(NAME, OP)                                       \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    int odd = simd_data(desc);                                          \
    uint32_t *d = vd;                                                   \
    int16_t *s = vs, *t = vt;                                           \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {                    \
        uint32_t prod = s[H2(2 * i + odd)] * t[H2(2 * i + odd)];        \
                                                                        \
        d[H4(i)] = OP(d[H4(i)], prod);                                  \
    }                                                                   \
}

DO_MAC_EVEN_ODD(mxu3_smlah, DO_ADD)
DO_MAC_EVEN_ODD(mxu3_smlsh, DO_SUB)

/* Accumulate words times the low (data 0) or high halfwords, sign-extended */
#define DO_MAC_HALF(NAME, OP)                                           \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    int n = simd_oprsz(desc) / 4;                                       \
    int half = simd_data(desc) ? n : 0;                                 \
    uint32_t *d = vd, *s = vs;                                          \
    int16_t *t = vt;                                                    \
                                                                        \
    for (int i = 0; i < n; i++) {                                       \
        uint32_t prod = s[H4(i)] * (uint32_t)t[H2(half + i)];           \
                                                                        \
        d[H4(i)] = OP(d[H4(i)], prod);                                  \
    }                                                                   \
}

DO_MAC_HALF(mxu3_wsmlah, DO_ADD)
DO_MAC_HALF(mxu3_wsmlsh, DO_SUB)

/* Element i of a register, of 2^esz bits */
static int64_t vpr_elem(const uint64_t *v, int i, int esz, bool is_signed)
{
    int bit = i << esz;

    if (is_signed) {
        return sextract64(v[bit / 64], bit % 64, 1 << esz);
    }
    return extract64(v[bit / 64], bit % 64, 1 << esz);
}

static void do_seg_sum(uint64_t *d, const uint64_t *s, const uint64_t *p,
                       uint32_t data)
{
    int esz = FIELD_EX32(data, MXU3_SUM, ESZ);
    bool ssign = FIELD_EX32(data, MXU3_SUM, SSIGN);
    bool psign = FIELD_EX32(data, MXU3_SUM, PSIGN);
    int le = FIELD_EX32(data, MXU3_SUM, LE);
    int m = FIELD_EX32(data, MXU3_SUM, M);
    int nel = (32 << le) >> esz;

    for (int j = 0; j < (MXU3_VLEN * 8) >> (5 + le); j++) {
        int k = FIELD_EX32(data, MXU3_SUM, REPL) ? FIELD_EX32(data, MXU3_SUM, N)
                                                : j;
        int w = m + (j << le);
        uint32_t sum = extract64(d[w / 2], (w % 2) * 32, 32);

        for (int i = 0; i < nel; i++) {
            int64_t x = vpr_elem(s, j * nel + i, esz, ssign);

            sum += p ? x * vpr_elem(p, k * nel + i, esz, psign) : x;
        }
        d[w / 2] = deposit64(d[w / 2], (w % 2) * 32, 32, sum);
    }
}

void HELPER(mxu3_srsum)(void *vd, void *vs, uint32_t desc)
{
    do_seg_sum(vd, vs, NULL, simd_data(desc));
}

void HELPER(mxu3_mac)(void *vd, void *vs, void *vt, uint32_t desc)
{
    do_seg_sum(vd, vs, vt, simd_data(desc));
}

/* Count the elements from element 0 up that have bit 0 set into word 0 */
void HELPER(mxu3_toc)(void *vd, void *vs, uint32_t desc)
{
    int esz = simd_data(desc);
    uint64_t *d = vd;
    uint32_t n = 0;

    while (n < (MXU3_VLEN * 8) >> esz && (vpr_elem(vs, n, esz, false) & 1)) {
        n++;
    }
    d[0] = deposit64(d[0], 0, 32, n);
}

static int64_t sat_elem(int64_t x, uint32_t data)
{
    int64_t lo = 0, hi = MAKE_64BIT_MASK(0, FIELD_EX32(data, MXU3_SAT, BITS));

    if (FIELD_EX32(data, MXU3_SAT, DSIGN)) {
        hi >>= 1;
        lo = -hi - 1;
    }
    return MIN(MAX(x, lo), hi);
}

/* Saturate each element to the range, keeping its width */
#define DO_SAT(NAME, TS, TU, H)                                         \
void HELPER(NAME)(void *vd, void *vs, uint32_t desc)                    \
{                                                                       \
    uint32_t data = simd_data(desc);                                    \
    TU *d = vd, *s = vs;                                                \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / sizeof(TU); i++) {           \
        int64_t x = s[H(i)];                                            \
                                                                        \
        if (FIELD_EX32(data, MXU3_SAT, SSIGN)) {                        \
            x = (TS)s[H(i)];                                            \
        }                                                               \
        d[H(i)] = sat_elem(x, data);                                    \
    }                                                                   \
}

DO_SAT(mxu3_sat_b, int8_t, uint8_t, H1)
DO_SAT(mxu3_sat_h, int16_t, uint16_t, H2)
DO_SAT(mxu3_sat_w, int32_t, uint32_t, H4)

/*
 * Map each element of vt against the one of vs: 3 if vs < 0 or vt >= vs,
 * else 2 if vt >= 0, 1 if vt >= -vs, otherwise 0.
 */
#define DO_PMAP(NAME, T, H)                                             \
void HELPER(NAME)(void *vd, void *vs, void *vt, uint32_t desc)          \
{                                                                       \
    T *d = vd, *s = vs, *t = vt;                                        \
                                                                        \
    for (int i = 0; i < simd_oprsz(desc) / sizeof(T); i++) {            \
        int64_t a = s[H(i)], b = t[H(i)];                               \
                                                                        \
        d[H(i)] = a < 0 || b >= a ? 3 : b >= 0 ? 2 : b >= -a ? 1 : 0;   \
    }                                                                   \
}

DO_PMAP(mxu3_pmap_h, int16_t, H2)
DO_PMAP(mxu3_pmap_w, int32_t, H4)

/* Move the words of vs whose counterpart in vt has the low 4 bits = data */
void HELPER(mxu3_cmvw)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, *s = vs, *t = vt;

    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        if ((t[H4(i)] & 15) == simd_data(desc)) {
            d[H4(i)] = s[H4(i)];
        }
    }
}

/* Shuffles */

static void vpr_set_elem(uint64_t *v, int i, int esz, uint64_t x)
{
    int bit = i << esz;

    v[bit / 64] = deposit64(v[bit / 64], bit % 64, 1 << esz, x);
}

/* Elements wider than 64 bits are whole 64-bit units */
static void vpr_copy_elem(uint64_t *d, int di, const uint64_t *s, int si,
                          int esz)
{
    if (esz > 6) {
        int units = 1 << (esz - 6);

        memcpy(d + di * units, s + si * units, units * 8);
    } else {
        vpr_set_elem(d, di, esz, vpr_elem(s, si, esz, false));
    }
}

/* Each 64-bit unit: even elements to the low word, odd ones to the high */
void HELPER(mxu3_gt)(void *vd, void *vt, uint32_t desc)
{
    int esz = FIELD_EX32(simd_data(desc), MXU3_SHF, ESZ);
    uint64_t *d = vd, *t = vt;

    for (int j = 0; j < simd_oprsz(desc) / 8; j++) {
        uint64_t u = t[j], r = 0;

        for (int i = 0; i < 32 >> esz; i++) {
            r = deposit64(r, i << esz, 1 << esz,
                          extract64(u, (2 * i) << esz, 1 << esz));
            r = deposit64(r, 32 + (i << esz), 1 << esz,
                          extract64(u, (2 * i + 1) << esz, 1 << esz));
        }
        d[j] = r;
    }
}

/* Element POS of each 2^LG-element group, packed into part M of vd */
void HELPER(mxu3_gtn)(void *vd, void *vt, uint32_t desc)
{
    uint32_t data = simd_data(desc);
    int esz = FIELD_EX32(data, MXU3_SHF, ESZ);
    int lg = FIELD_EX32(data, MXU3_SHF, LG);
    int pos = FIELD_EX32(data, MXU3_SHF, POS);
    int cnt = (simd_oprsz(desc) * 8) >> (esz + lg);
    int base = FIELD_EX32(data, MXU3_SHF, M) * cnt;
    uint64_t t[MXU3_VLEN / 8];

    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < cnt; i++) {
        vpr_copy_elem(vd, base + i, t, (i << lg) + pos, esz);
    }
}

/*
 * The elements of a half of vt go to every other element of vd, the odd
 * ones or with LOW the even ones; with ZERO the others are cleared.
 */
void HELPER(mxu3_extuw)(void *vd, void *vt, uint32_t desc)
{
    uint32_t data = simd_data(desc);
    int esz = FIELD_EX32(data, MXU3_SHF, ESZ);
    int low = FIELD_EX32(data, MXU3_SHF, LOW);
    int n = (simd_oprsz(desc) * 8) >> (esz + 1);
    int half = FIELD_EX32(data, MXU3_SHF, HI) ? n : 0;
    uint64_t t[MXU3_VLEN / 8], z[4] = { };

    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < n; i++) {
        if (FIELD_EX32(data, MXU3_SHF, ZERO)) {
            vpr_copy_elem(vd, 2 * i + low, z, 0, esz);
        }
        vpr_copy_elem(vd, 2 * i + 1 - low, t, half + i, esz);
    }
}

/* The elements of a half of vt, extended to double width */
void HELPER(mxu3_ext)(void *vd, void *vt, uint32_t desc)
{
    uint32_t data = simd_data(desc);
    int esz = FIELD_EX32(data, MXU3_SHF, ESZ);
    bool sign = FIELD_EX32(data, MXU3_SHF, SIGN);
    int n = (simd_oprsz(desc) * 8) >> (esz + 1);
    int half = FIELD_EX32(data, MXU3_SHF, HI) ? n : 0;
    uint64_t t[MXU3_VLEN / 8];

    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < n; i++) {
        vpr_set_elem(vd, i, esz + 1, vpr_elem(t, half + i, esz, sign));
    }
}

/* Word i of vd is the zero-extended 24-bit element i of vt */
void HELPER(mxu3_extu3bw)(void *vd, void *vt, uint32_t desc)
{
    uint64_t t[MXU3_VLEN / 8];

    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        int u = 24 * i / 64, ofs = 24 * i % 64;
        uint64_t x = t[u] >> ofs;

        if (ofs > 40) {
            x |= t[u + 1] << (64 - ofs);
        }
        vpr_set_elem(vd, i, 5, extract64(x, 0, 24));
    }
}

/* Element j of each 2^LG-element group comes from group j of vt */
void HELPER(mxu3_shuf)(void *vd, void *vt, uint32_t desc)
{
    uint32_t data = simd_data(desc);
    int esz = FIELD_EX32(data, MXU3_SHF, ESZ);
    int lg = FIELD_EX32(data, MXU3_SHF, LG);
    int groups = (simd_oprsz(desc) * 8) >> (esz + lg);
    uint64_t t[MXU3_VLEN / 8];

    memcpy(t, vt, sizeof(t));
    for (int j = 0; j < groups; j++) {
        for (int i = 0; i < 1 << lg; i++) {
            vpr_copy_elem(vd, (j << lg) + i, t, j + i * groups, esz);
        }
    }
}

/* Interleave the even (or ODD) elements of vs and vt, those of vs in place */
void HELPER(mxu3_ilv)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t data = simd_data(desc);
    int esz = FIELD_EX32(data, MXU3_SHF, ESZ);
    int odd = FIELD_EX32(data, MXU3_SHF, ODD);
    uint64_t s[MXU3_VLEN / 8], t[MXU3_VLEN / 8];

    memcpy(s, vs, sizeof(s));
    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < (simd_oprsz(desc) * 8) >> (esz + 1); i++) {
        vpr_copy_elem(vd, 2 * i + odd, s, 2 * i + odd, esz);
        vpr_copy_elem(vd, 2 * i + 1 - odd, t, 2 * i + odd, esz);
    }
}

/* Shift the whole register by bytes, filling with zeros */
static void do_bsh(uint64_t *d, const uint64_t *t, int n, bool right, int oprsz)
{
    uint8_t b[MXU3_VLEN];

    for (int i = 0; i < oprsz; i++) {
        int k = right ? i + n : i - n;

        b[i] = k >= 0 && k < oprsz ? extract64(t[k / 8], (k % 8) * 8, 8) : 0;
    }
    for (int i = 0; i < oprsz / 8; i++) {
        d[i] = 0;
        for (int k = 0; k < 8; k++) {
            d[i] = deposit64(d[i], k * 8, 8, b[i * 8 + k]);
        }
    }
}

/* data: the byte count, plus 64 to shift right */
void HELPER(mxu3_bshi)(void *vd, void *vt, uint32_t desc)
{
    do_bsh(vd, vt, simd_data(desc) & 63, simd_data(desc) & 64,
           simd_oprsz(desc));
}

/* The byte count is the low 6 bits of vs; data is 1 to shift right */
void HELPER(mxu3_bsh)(void *vd, void *vs, void *vt, uint32_t desc)
{
    do_bsh(vd, vt, *(uint64_t *)vs & 63, simd_data(desc),
           simd_oprsz(desc));
}

/* Generic shuffles */

/*
 * Word sel of vt for a word selector, else 0 or the old word for the
 * zero and keep codes: 0x80 and 0xc0 on MXU3.0, while MXU3.1 decodes
 * only bits 7:6 (data bit 2). Only the low 4 bits index the word.
 */
static uint32_t gshuf_word(const uint32_t *t, uint32_t sel, uint32_t old,
                           bool relaxed)
{
    uint32_t code = sel & (relaxed ? 0xc0 : 0xff);

    if (code == 0x80) {
        return 0;
    }
    if (code == 0xc0) {
        return old;
    }
    return t[H4(sel & 15)];
}

/* Byte b of the word picked by the 2-bit field b of sel */
static uint32_t gshuf_bytes(uint32_t w, uint32_t sel)
{
    uint32_t r = 0;

    for (int b = 0; b < 4; b++) {
        r = deposit32(r, 8 * b, 8, w >> (8 * extract32(sel, 2 * b, 2)));
    }
    return r;
}

/* Halfword h of the word picked by bit h of sel */
static uint32_t gshuf_halves(uint32_t w, uint32_t sel)
{
    return deposit32(w >> (16 * (sel & 1)), 16, 16,
                     w >> (16 * ((sel >> 1) & 1)));
}

/* Byte data[1:0] of each word of vs selects a word of vt */
void HELPER(mxu3_gshufw)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, s[MXU3_VLEN / 4], t[MXU3_VLEN / 4];
    int imm = simd_data(desc) & 3;

    memcpy(s, vs, sizeof(s));
    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        d[H4(i)] = gshuf_word(t, s[H4(i)] >> (8 * imm), d[H4(i)],
                              simd_data(desc) & 4);
    }
}

/*
 * Halfword data[0] of each word of vs: bits 7:0 select a word of vt and
 * bits 15:8 its bytes.
 */
void HELPER(mxu3_gshufwb)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, s[MXU3_VLEN / 4], t[MXU3_VLEN / 4];

    memcpy(s, vs, sizeof(s));
    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        uint32_t ctl = s[H4(i)] >> (16 * (simd_data(desc) & 1));
        uint32_t code = simd_data(desc) & 4 ? ctl & 0xc0 : ctl & 0xff;

        if (code == 0x80 || code == 0xc0) {
            d[H4(i)] = gshuf_word(t, ctl, d[H4(i)], simd_data(desc) & 4);
        } else {
            d[H4(i)] = gshuf_bytes(t[H4(ctl & 15)], ctl >> 8);
        }
    }
}

/* As GSHUFWB, but bits 7:4 of the selector clear bytes 3:0 instead */
void HELPER(mxu3_gshufwb2)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, s[MXU3_VLEN / 4], t[MXU3_VLEN / 4];

    memcpy(s, vs, sizeof(s));
    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        uint32_t ctl = s[H4(i)] >> (16 * simd_data(desc));
        uint32_t r = gshuf_bytes(t[H4(ctl & 15)], ctl >> 8);

        for (int b = 0; b < 4; b++) {
            if (ctl & (0x10 << b)) {
                r = deposit32(r, 8 * b, 8, 0);
            }
        }
        d[H4(i)] = r;
    }
}

/* Bits 7:0 select a word of vt, bits 9:8 its halfwords */
void HELPER(mxu3_gshufwh)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, s[MXU3_VLEN / 4], t[MXU3_VLEN / 4];

    memcpy(s, vs, sizeof(s));
    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        uint32_t ctl = s[H4(i)];
        uint32_t w = gshuf_word(t, ctl, d[H4(i)], false);

        if ((ctl & 0xff) != 0x80 && (ctl & 0xff) != 0xc0) {
            w = gshuf_halves(w, ctl >> 8);
        }
        d[H4(i)] = w;
    }
}

/* The bytes or halfwords of each word of vt, picked by the vs word */
void HELPER(mxu3_gshufb)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, *s = vs, *t = vt;

    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        d[H4(i)] = gshuf_bytes(t[H4(i)], s[H4(i)]);
    }
}

void HELPER(mxu3_gshufh)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint32_t *d = vd, *s = vs, *t = vt;

    for (int i = 0; i < simd_oprsz(desc) / 4; i++) {
        d[H4(i)] = gshuf_halves(t[H4(i)], s[H4(i)]);
    }
}

/* Byte i of vd is byte (vs byte i) of vt, or 0 when that has bit 7 set */
void HELPER(mxu3_gshufvb)(void *vd, void *vs, void *vt, uint32_t desc)
{
    uint8_t *d = vd, s[MXU3_VLEN], t[MXU3_VLEN];

    memcpy(s, vs, sizeof(s));
    memcpy(t, vt, sizeof(t));
    for (int i = 0; i < simd_oprsz(desc); i++) {
        uint8_t idx = s[H1(i)];

        d[H1(i)] = idx & 0x80 ? 0 : t[H1(idx & (simd_oprsz(desc) - 1))];
    }
}

/*
 * Single precision. Results are built in a temporary and stored only once
 * mxu_fp_end() has not raised an enabled exception, so a trapping
 * instruction leaves vd untouched. The operations see the whole sources,
 * as the complex and cross forms pair elements.
 */
#define DO_FP(NAME, OP)                                                 \
void HELPER(NAME)(void *vd, void *vs, void *vt, CPUMIPSState *env,      \
                  uint32_t desc)                                        \
{                                                                       \
    float_status *st = mxu_fpst(env);                                  \
    uint32_t *d = vd, r[MXU3_VLEN / 4];                                 \
                                                                        \
    for (int i = 0; i < MXU3_VLEN / 4; i++) {                           \
        r[i] = OP;                                                      \
    }                                                                   \
    mxu_fp_end(env, GETPC());                                          \
    for (int i = 0; i < MXU3_VLEN / 4; i++) {                           \
        d[H4(i)] = r[i];                                                \
    }                                                                   \
}

#define S_(i)   ((uint32_t *)vs)[H4(i)]
#define T_(i)   ((uint32_t *)vt)[H4(i)]

/*
 * Quiet compares: all ones when the relation is in ok. Flushing a
 * subnormal operand does not make a compare inexact.
 */
static uint32_t fcmp(float32 a, float32 b, int ok, float_status *st)
{
    int rel = float32_compare_quiet(a, b, st);

    set_float_exception_flags(get_float_exception_flags(st) &
                              ~float_flag_input_denormal_flushed, st);
    return ok & (1 << (rel + 1)) ? -1 : 0;
}

#define LT      (1 << (float_relation_less + 1))
#define EQ      (1 << (float_relation_equal + 1))
#define GT      (1 << (float_relation_greater + 1))

DO_FP(mxu3_faddw, float32_add(S_(i), T_(i), st))
DO_FP(mxu3_fsubw, float32_sub(S_(i), T_(i), st))
DO_FP(mxu3_fmulw, float32_mul(S_(i), T_(i), st))
DO_FP(mxu3_fmaxw, float32_maxnum(S_(i), T_(i), st))
DO_FP(mxu3_fmaxaw, float32_maxnummag(S_(i), T_(i), st))
DO_FP(mxu3_fminw, float32_minnum(S_(i), T_(i), st))
DO_FP(mxu3_fminaw, float32_minnummag(S_(i), T_(i), st))
DO_FP(mxu3_fceqw, fcmp(S_(i), T_(i), EQ, st))
DO_FP(mxu3_fclew, fcmp(S_(i), T_(i), LT | EQ, st))
DO_FP(mxu3_fcltw, fcmp(S_(i), T_(i), LT, st))
DO_FP(mxu3_fcorw, fcmp(S_(i), T_(i), LT | EQ | GT, st))
/*
 * Real and imaginary parts are the even and odd elements. The T41 negates
 * the vt operand, not the product, so a NaN from vt changes sign; the
 * same goes for the subtraction in FXAS.
 */
DO_FP(mxu3_fcmulrw, float32_mul(S_(i), T_(i & ~1), st))
DO_FP(mxu3_fcmuliw, i & 1 ? float32_mul(S_(i), float32_chs(T_(i)), st)
                          : float32_mul(S_(i), T_(i + 1), st))
DO_FP(mxu3_fcaddw, float32_add(S_(i), T_(i ^ 1), st))

/* Element i and i + 2^data of each pair of 2^data-word segments of vt */
DO_FP(mxu3_fxas, (i >> simd_data(desc)) & 1
      ? float32_add(float32_chs(T_(i)), T_(i - (1 << simd_data(desc))), st)
      : float32_add(T_(i), T_(i + (1 << simd_data(desc))), st))

/*
 * Conversions, in MCSR.RM unless truncating. A NaN converts to zero, and
 * so does a negative value to unsigned, as invalid even if it would round
 * to zero.
 */
static uint32_t fp_convert(uint32_t a, int sel, float_status *st)
{
    if (sel == 4 || sel == 5) {
        a = float32_squash_input_denormal(a, st);
    }
    if (sel >= 4 && (float32_is_any_nan(a) ||
                     ((sel == 4 || sel == 5) && float32_is_neg(a) &&
                      !float32_is_zero(a)))) {
        float_raise(float_flag_invalid, st);
        return 0;
    }
    switch (sel) {
    case 0:
        return uint32_to_float32(a, st);
    case 1:
        return float32_round_to_int(a, st);
    case 2:
        return int32_to_float32(a, st);
    case 4:
        return float32_to_uint32_round_to_zero(a, st);
    case 5:
        return float32_to_uint32(a, st);
    case 6:
        return float32_to_int32_round_to_zero(a, st);
    default:
        return float32_to_int32(a, st);
    }
}

DO_FP(mxu3_fcvt, fp_convert(S_(i), simd_data(desc), st))

/* The class mask ignores MCSR.FS and raises nothing */
void HELPER(mxu3_fclassw)(void *vd, void *vs, CPUMIPSState *env,
                          uint32_t desc)
{
    uint32_t *d = vd, *s = vs;

    for (int i = 0; i < MXU3_VLEN / 4; i++) {
        d[H4(i)] = float_class_s(s[H4(i)], &env->active_tc.mxu_fp_status);
    }
}
