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

/*
 * AF's sums over the luma of columns @x0 to @x1 of a row of @fw: the luma,
 * the pixels over @bright and the two second differences, whose neighbours
 * a row's ends repeat. @o takes them added to what it holds.
 */
static void ISPK_ATTR ISPK(af_span)(const uint8_t *restrict l, uint32_t x0,
                                    uint32_t x1, uint32_t fw,
                                    uint32_t bright, uint64_t *restrict o)
{
    uint32_t a = MAX(x0, 2), b = MIN(x1, fw > 2 ? fw - 2 : 0);
    uint32_t s0 = 0, s1 = 0, s2 = 0, s3 = 0;

    for (uint32_t x = a; x < b; x++) {
        int v = l[x];

        s0 += v;
        s1 += (uint32_t)v > bright;
        s2 += abs(l[x - 1] - 2 * v + l[x + 1]);
        s3 += abs(l[x - 2] - 2 * v + l[x + 2]);
    }
    for (uint32_t x = x0; x < x1; x++) {
        int v = l[x];

        if (x >= a && x < b) {
            x = b - 1;
            continue;
        }
        o[0] += v;
        o[1] += (uint32_t)v > bright;
        o[2] += abs(l[x ? x - 1 : 0] - 2 * v + l[MIN(x + 1, fw - 1)]);
        o[3] += abs(l[x > 1 ? x - 2 : 0] - 2 * v + l[MIN(x + 2, fw - 1)]);
    }
    o[0] += s0;
    o[1] += s1;
    o[2] += s2;
    o[3] += s3;
}

/* AF's vertical second differences of columns @x0 to @x1 about row @l */
static void ISPK_ATTR ISPK(af_vert)(const uint8_t *restrict l,
                                    const uint8_t *restrict n0,
                                    const uint8_t *restrict n1,
                                    const uint8_t *restrict n2,
                                    const uint8_t *restrict n3, uint32_t x0,
                                    uint32_t x1, uint64_t *restrict o)
{
    uint32_t s4 = 0, s5 = 0;

    for (uint32_t x = x0; x < x1; x++) {
        s4 += abs(n1[x] - 2 * l[x] + n2[x]);
        s5 += abs(n0[x] - 2 * l[x] + n3[x]);
    }
    o[0] += s4;
    o[1] += s5;
}

/* One defog pixel's colours and which of luma's 3 bins it falls in */
static inline __attribute__((always_inline))
void ISPK(defog_pix)(int l, int u, int v, int t0, int t1, uint32_t *rgb,
                     uint32_t *bins)
{
    /* Limited-range BT.601, as the scene is rendered */
    int c = (l - 16) * 298 + 128;
    int r = MIN(MAX((c + 409 * v) >> 8, 0), 255);
    int g = MIN(MAX((c - 100 * u - 208 * v) >> 8, 0), 255);
    int b = MIN(MAX((c + 516 * u) >> 8, 0), 255);
    int y = (r * 77 + g * 150 + b * 29) >> 8;

    rgb[0] += r;
    rgb[1] += g;
    rgb[2] += b;
    bins[0] += y < t0;
    bins[1] += y >= t0 && y < t1;
    bins[2] += y >= t0 && y >= t1;
}

/*
 * Defog's sums over columns @x0 to @x1 of a row's luma @l and chroma @uv:
 * R, G, B, then the pixels in luma's 3 bins, added to @o's six. The loop
 * over the pairs of pixels that share a chroma pair is indexed by size_t,
 * which the vectoriser needs for these interleaved loads.
 */
static void ISPK_ATTR ISPK(defog_span)(const uint8_t *restrict l,
                                       const uint8_t *restrict uv,
                                       uint32_t x0, uint32_t x1, int t0,
                                       int t1, uint64_t *restrict o)
{
    uint32_t rgb[3] = { 0 }, bins[3] = { 0 };
    uint32_t x = x0;
    size_t pairs;
    int lt0 = 0, lt1 = 0;

    if ((x & 1) && x < x1) {
        ISPK(defog_pix)(l[x], uv[x - 1] - 128, uv[x] - 128, t0, t1, rgb, bins);
        x++;
    }
    pairs = x < x1 ? (x1 - x) / 2 : 0;
    for (size_t j = 0; j < pairs; j++) {
        int u = uv[x + 2 * j] - 128, v = uv[x + 2 * j + 1] - 128;
        int cr = 409 * v, cg = -100 * u - 208 * v, cb = 516 * u;

        for (unsigned k = 0; k < 2; k++) {
            int c = (l[x + 2 * j + k] - 16) * 298 + 128;
            int r = MIN(MAX((c + cr) >> 8, 0), 255);
            int g = MIN(MAX((c + cg) >> 8, 0), 255);
            int b = MIN(MAX((c + cb) >> 8, 0), 255);
            int y = (r * 77 + g * 150 + b * 29) >> 8;

            rgb[0] += r;
            rgb[1] += g;
            rgb[2] += b;
            lt0 += y < t0;
            lt1 += y < t1;
        }
    }
    /* below t0 is bin 0, else below t1 bin 1, else bin 2 */
    bins[0] += lt0;
    if (t1 >= t0) {
        bins[1] += lt1 - lt0;
        bins[2] += 2 * pairs - lt1;
    } else {
        bins[2] += 2 * pairs - lt0;
    }
    x += 2 * pairs;
    if (x < x1) {
        ISPK(defog_pix)(l[x], uv[x] - 128, uv[x + 1] - 128, t0, t1, rgb, bins);
    }
    for (unsigned i = 0; i < 3; i++) {
        o[i] += rgb[i];
        o[3 + i] += bins[i];
    }
}

/* @n samples of |cur - ref| summed over x - step, x and x + step */
static void ISPK_ATTR ISPK(mdns_sad3)(const uint8_t *restrict cl,
                                      const uint8_t *restrict rl,
                                      const uint8_t *restrict cm,
                                      const uint8_t *restrict rm,
                                      const uint8_t *restrict cr,
                                      const uint8_t *restrict rr,
                                      uint16_t *restrict o, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) {
        uint8_t a = MAX(cl[i], rl[i]) - MIN(cl[i], rl[i]);
        uint8_t b = MAX(cm[i], rm[i]) - MIN(cm[i], rm[i]);
        uint8_t d = MAX(cr[i], rr[i]) - MIN(cr[i], rr[i]);

        o[i] = (uint16_t)a + b + d;
    }
}

/* @n samples blended: the reference weighs @wt where the 3 x 3 is within */
static void ISPK_ATTR ISPK(mdns_mix)(const uint8_t *restrict c,
                                     const uint8_t *restrict r,
                                     const uint16_t *restrict up,
                                     const uint16_t *restrict mid,
                                     const uint16_t *restrict dn,
                                     const uint8_t *restrict wt,
                                     uint8_t *restrict o, uint8_t *restrict wo,
                                     uint32_t n, uint16_t lim)
{
    for (uint32_t i = 0; i < n; i++) {
        uint16_t sad = up[i] + mid[i] + dn[i];
        int16_t wgt = wt[i] & -(int16_t)(2 * sad <= lim);
        int16_t d = (int16_t)r[i] - (int16_t)c[i];

        o[i] = c[i] + (int16_t)((int16_t)(d * wgt + 128) >> 8);
        wo[i] = wgt;
    }
}

/*
 * The sums of cur - ref and |cur - ref| over each block of a row: @bsz
 * samples from @gx, @nb blocks at most, the last cut at @w. @bs and @ba
 * take them added.
 */
static void ISPK_ATTR ISPK(mdns_blocks)(const uint8_t *restrict c,
                                        const uint8_t *restrict r,
                                        uint32_t gx, uint32_t w, uint32_t bsz,
                                        uint32_t nb, int32_t *restrict bs,
                                        int32_t *restrict ba)
{
    for (uint32_t bx = 0; bx < nb && gx + bx * bsz < w; bx++) {
        const uint8_t *cb = c + gx + bx * bsz, *rb = r + gx + bx * bsz;
        uint32_t n = MIN(bsz, w - (gx + bx * bsz));
        int32_t a = 0, b = 0;

        for (uint32_t i = 0; i < n; i++) {
            a += cb[i] - rb[i];
            b += MAX(cb[i], rb[i]) - MIN(cb[i], rb[i]);
        }
        bs[bx] += a;
        ba[bx] += b;
    }
}
