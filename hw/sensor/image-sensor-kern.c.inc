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
