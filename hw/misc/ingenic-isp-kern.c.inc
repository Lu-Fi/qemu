/*
 * The ISP's per-pixel kernels, included once for each build of them: the
 * default one and, on a host that has it, one for AVX2. The including file
 * defines ISPK() to name a build's functions and ISPK_ATTR for the
 * attributes of its kernels. Their loops are written to be vectorised.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/*
 * One sample through the gains @k: it takes its colours' place in @p and
 * adds to the block's chroma sums
 */
static inline __attribute__((always_inline))
void ISPK(wb_pix)(uint8_t *p, int32_t cr, int32_t cg, int32_t cb,
                  const int32_t *k, int32_t *su, int32_t *sv)
{
    int32_t ly = (*p - 16) * 298;
    int32_t r = ((ly + cr) * k[0]) >> 12;
    int32_t g = ((ly + cg) * k[1]) >> 12;
    int32_t b = ((ly + cb) * k[2]) >> 12;

    r = MIN(MAX(r, -4 * 256), 255 * 256);
    g = MIN(MAX(g, -4 * 256), 255 * 256);
    b = MIN(MAX(b, -4 * 256), 255 * 256);
    *p = MIN(MAX(16 + ((66 * r + 129 * g + 25 * b + (1 << 15)) >> 16), 0),
             255);
    *su += -38 * r - 74 * g + 112 * b;
    *sv += 112 * r - 94 * g - 18 * b;
}

/*
 * @n blocks of 2 x 2 samples of the rows @l0 and @l1 (2 * @n samples each)
 * and the chroma pairs @c, put through the gains @k in 32 bits, which
 * ingenic_isp_wb_picture() uses while no product can pass 2^31. The chroma
 * comes out of a second loop, in chunks, as the vectoriser will not take a
 * loop that reads and writes the pairs; the indices are size_t for it too.
 * @tmp, 2 * ISP_WB_CHUNK words, holds the sums a chunk leaves: a local
 * array would be cleared in the loop by -ftrivial-auto-var-init, which
 * the vectoriser will not take either.
 */
static void ISPK_ATTR ISPK(wb_blocks)(uint8_t *restrict l0,
                                      uint8_t *restrict l1,
                                      uint8_t *restrict c, size_t n,
                                      const int32_t *restrict k,
                                      int32_t *restrict tmp)
{
    int32_t *sus = tmp, *svs = tmp + ISP_WB_CHUNK;

    for (size_t j0 = 0; j0 < n; j0 += ISP_WB_CHUNK) {
        size_t m = MIN(ISP_WB_CHUNK, n - j0);
        uint8_t *a0 = l0 + 2 * j0, *a1 = l1 + 2 * j0, *ac = c + 2 * j0;

        for (size_t j = 0; j < m; j++) {
            int32_t u = ac[2 * j] - 128, v = ac[2 * j + 1] - 128;
            int32_t cr = 409 * v, cg = -100 * u - 208 * v, cb = 516 * u;
            int32_t su = 0, sv = 0;

            ISPK(wb_pix)(a0 + 2 * j, cr, cg, cb, k, &su, &sv);
            ISPK(wb_pix)(a0 + 2 * j + 1, cr, cg, cb, k, &su, &sv);
            ISPK(wb_pix)(a1 + 2 * j, cr, cg, cb, k, &su, &sv);
            ISPK(wb_pix)(a1 + 2 * j + 1, cr, cg, cb, k, &su, &sv);
            sus[j] = su;
            svs[j] = sv;
        }
        for (size_t j = 0; j < m; j++) {
            ac[2 * j] = MIN(MAX(128 + ((sus[j] + (1 << 17)) >> 18), 0), 255);
            ac[2 * j + 1] = MIN(MAX(128 + ((svs[j] + (1 << 17)) >> 18), 0),
                                255);
        }
    }
}
