/*
 * The image sensor's per-pixel kernels, included once for each build of
 * them: the default one and, on a host that has it, one for AVX2. The
 * including file defines ISK() to name a build's functions and ISK_ATTR for
 * the attributes of its kernels. Their loops are written to be vectorised.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/* @n samples of @u and @v, interleaved into @o */
static void ISK_ATTR ISK(interleave)(const uint8_t *restrict u,
                                     const uint8_t *restrict v,
                                     uint8_t *restrict o, size_t n)
{
    for (size_t x = 0; x < n; x++) {
        o[2 * x] = u[x];
        o[2 * x + 1] = v[x];
    }
}

/* @n samples, each the mean (rounded) of 2 x 2 of the rows @r0 and @r1 */
static void ISK_ATTR ISK(half_row)(const uint8_t *restrict r0,
                                   const uint8_t *restrict r1,
                                   uint8_t *restrict o, size_t n)
{
    for (size_t x = 0; x < n; x++) {
        o[x] = ((uint16_t)r0[2 * x] + r0[2 * x + 1] + r1[2 * x] +
                r1[2 * x + 1] + 2) >> 2;
    }
}

/* The same of two planes' rows, @u0 and @u1 and @v0 and @v1, interleaved */
static void ISK_ATTR ISK(half_uv_row)(const uint8_t *restrict u0,
                                      const uint8_t *restrict u1,
                                      const uint8_t *restrict v0,
                                      const uint8_t *restrict v1,
                                      uint8_t *restrict o, size_t n)
{
    for (size_t x = 0; x < n; x++) {
        o[2 * x] = ((uint16_t)u0[2 * x] + u0[2 * x + 1] + u1[2 * x] +
                    u1[2 * x + 1] + 2) >> 2;
        o[2 * x + 1] = ((uint16_t)v0[2 * x] + v0[2 * x + 1] + v1[2 * x] +
                        v1[2 * x + 1] + 2) >> 2;
    }
}

/*
 * @n groups of 16 samples of an enlarged row: group g is the bytes of @row
 * from @base[g] that @idx picks (16 each), in a byte shuffle. A group whose
 * @base is UINT32_MAX is left to the caller; the 16 bytes at a base are
 * always inside the row.
 */
static void ISK_ATTR ISK(shuffle_row)(const uint8_t *restrict row,
                                      uint8_t *restrict o, size_t n,
                                      const uint32_t *restrict base,
                                      const uint8_t *restrict idx)
{
    for (size_t g = 0; g < n; g++) {
        if (base[g] == UINT32_MAX) {
            continue;
        }
#if defined(ISK_AVX2)
        _mm_storeu_si128((__m128i *)(o + 16 * g), _mm_shuffle_epi8(
                         _mm_loadu_si128((const __m128i *)(row + base[g])),
                         _mm_loadu_si128((const __m128i *)(idx + 16 * g))));
#elif defined(__aarch64__)
        vst1q_u8(o + 16 * g, vqtbl1q_u8(vld1q_u8(row + base[g]),
                                        vld1q_u8(idx + 16 * g)));
#else
        for (unsigned i = 0; i < 16; i++) {
            o[16 * g + i] = row[base[g] + idx[16 * g + i]];
        }
#endif
    }
}

/*
 * @n bytes of @src through the 256-byte table @lut into @dst, which may be
 * @src. A table of bytes is 16 shuffles of 16 entries each, picked by the
 * high nibble (AVX2), or 4 lookups of 64 (NEON).
 */
static void ISK_ATTR ISK(lookup)(uint8_t *dst, const uint8_t *src, size_t n,
                                  const uint8_t *lut)
{
    size_t i = 0;

#if defined(ISK_AVX2)
    __m256i t[16];

    for (unsigned h = 0; h < 16; h++) {
        t[h] = _mm256_broadcastsi128_si256(
                   _mm_loadu_si128((const void *)(lut + 16 * h)));
    }
    for (; i + 32 <= n; i += 32) {
        __m256i v = _mm256_loadu_si256((const __m256i *)(src + i));
        __m256i lo = _mm256_and_si256(v, _mm256_set1_epi8(15));
        __m256i hi = _mm256_and_si256(_mm256_srli_epi16(v, 4),
                                      _mm256_set1_epi8(15));
        __m256i r = _mm256_shuffle_epi8(t[0], lo);

        for (unsigned h = 1; h < 16; h++) {
            r = _mm256_blendv_epi8(r, _mm256_shuffle_epi8(t[h], lo),
                                   _mm256_cmpgt_epi8(hi,
                                                     _mm256_set1_epi8(h - 1)));
        }
        _mm256_storeu_si256((__m256i *)(dst + i), r);
    }
#elif defined(__aarch64__)
    uint8x16x4_t t[4];

    for (unsigned q = 0; q < 16; q++) {
        t[q / 4].val[q % 4] = vld1q_u8(lut + 16 * q);
    }
    for (; i + 16 <= n; i += 16) {
        uint8x16_t v = vld1q_u8(src + i);
        uint8x16_t r = vqtbl4q_u8(t[0], v);

        for (unsigned q = 1; q < 4; q++) {
            r = vqtbx4q_u8(r, t[q], vsubq_u8(v, vdupq_n_u8(64 * q)));
        }
        vst1q_u8(dst + i, r);
    }
#endif
    for (; i < n; i++) {
        dst[i] = lut[src[i]];
    }
}
