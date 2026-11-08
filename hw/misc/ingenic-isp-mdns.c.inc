/*
 * The T31's MDNS smoothing and sharpening kernels, included once for each
 * build of them: the default one and, on a host that has it, one for AVX2.
 * The including file defines MDNS_K() to name a build's functions and
 * MDNS_ATTR for the attributes of its kernels.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

/*
 * A row's 5 horizontal taps of @r (@w samples) in @t, its edges repeated.
 * The weights are {36, 58, 68, 58, 36}: a tap row is at most 65280, which
 * the 16-bit sums rely on
 */
static void MDNS_ATTR MDNS_K(taps)(const uint8_t *restrict r,
                                   uint16_t *restrict t, uint32_t w)
{
    static const unsigned tw[5] = { 36, 58, 68, 58, 36 };

    for (uint32_t x = 0; x < MIN(2, w); x++) {
        t[x] = tw[0] * r[x > 1 ? x - 2 : 0] + tw[1] * r[x ? x - 1 : 0] +
               tw[2] * r[x] + tw[3] * r[MIN(x + 1, w - 1)] +
               tw[4] * r[MIN(x + 2, w - 1)];
    }
    for (uint32_t x = 2; x + 2 < w; x++) {
        t[x] = (uint16_t)(36 * (uint16_t)(r[x - 2] + r[x + 2]) +
                          58 * (uint16_t)(r[x - 1] + r[x + 1]) +
                          68 * (uint16_t)r[x]);
    }
    for (uint32_t x = MAX(2, w - 2); x < w; x++) {
        t[x] = tw[0] * r[x - 2] + tw[1] * r[x - 1] + tw[2] * r[x] +
               tw[3] * r[MIN(x + 1, w - 1)] + tw[4] * r[MIN(x + 2, w - 1)];
    }
}

static void MDNS_K(spa_start)(IspMdnsSpa *sp, uint32_t j0)
{
    const IspMdnsJob *p = sp->p;

    sp->j0 = j0;
    for (int k = -2; k <= 2; k++) {
        uint32_t j = MIN(MAX((int)j0 + k, 0), (int)p->h - 1);

        MDNS_K(taps)(p->src + (size_t)j * p->w, sp->ring[j % 5], p->w);
    }
}

/* Row @j (@j0, then each after it) smoothed into @o */
static void MDNS_ATTR MDNS_K(spa_row)(IspMdnsSpa *sp, uint32_t j,
                                      uint8_t *restrict o)
{
    const IspMdnsJob *p = sp->p;
    uint32_t w = p->w, h = p->h;
    const uint8_t *restrict r = p->src + (size_t)j * w;
    const uint8_t *restrict wm = p->wmap + (size_t)j * w;
    const uint16_t *rw[5];
    int32_t *restrict dv = sp->dv;

    if (j > sp->j0 && j + 2 < h) {
        MDNS_K(taps)(p->src + (size_t)(j + 2) * w, sp->ring[(j + 2) % 5], w);
    }
    for (int k = -2; k <= 2; k++) {
        rw[k + 2] = sp->ring[MIN(MAX((int)j + k, 0), (int)h - 1) % 5];
    }
    for (size_t x = 0; x < w; x++) {
        uint32_t t = 36u * (rw[0][x] + rw[4][x]) + 58u * (rw[1][x] + rw[3][x]) +
                     68u * rw[2][x];

        dv[x] = (int32_t)((t + 32768) >> 16) - r[x];
    }
    for (uint32_t x = 0; x < w; x++) {
        int32_t d = dv[x];
        uint32_t kk = p->kk[abs(d)];
        int32_t k = ((kk & 0xffff) * (256 - wm[x]) + (kk >> 16) * wm[x] +
                     128) >> 8;

        o[x] = r[x] + ((k * d + (d < 0 ? 127 : 128)) >> 8);
    }
}

/* Rows of the smoothing, from where @next is, up to row @upto */
static void MDNS_K(spa_to)(IspMdnsSpa *sp, uint8_t *sr, uint32_t *next,
                           uint32_t upto)
{
    for (; *next <= upto && *next < sp->p->h; (*next)++) {
        MDNS_K(spa_row)(sp, *next,
                        sr + (size_t)(*next % MDNS_RING) * sp->p->w);
    }
}

/* Row @y of the picture (its edges repeated) in the ring of smoothed rows */
static inline const uint8_t *MDNS_K(ring)(const uint8_t *sr, int32_t y,
                                          uint32_t w, uint32_t h)
{
    return sr + (size_t)(MIN(MAX(y, 0), (int32_t)h - 1) % MDNS_RING) * w;
}

/*
 * 625 * the variance of a row's 5 x 5 windows in @v, and in @b5 and @b3 the
 * sums of their 5 x 5 and 3 x 3, from the window's column sums
 */
static void MDNS_ATTR MDNS_K(sharp_var)(int32_t *restrict v,
                                        uint16_t *restrict b5,
                                        uint16_t *restrict b3,
                                        const uint16_t *restrict c5,
                                        const uint16_t *restrict c3,
                                        const uint32_t *restrict q5,
                                        uint32_t w)
{
    for (size_t x = 0; x < w; x++) {
        uint16_t s5 = c5[x] + c5[x + 1] + c5[x + 2] + c5[x + 3] + c5[x + 4];
        uint32_t qq = q5[x] + q5[x + 1] + q5[x + 2] + q5[x + 3] + q5[x + 4];

        b5[x] = s5;
        b3[x] = c3[x + 1] + c3[x + 2] + c3[x + 3];
        v[x] = 25 * qq - (uint32_t)s5 * s5;
    }
}

/*
 * One sample of the sharpened row: @c it, @l, @r, @u, @d the 4 around it,
 * @b5 and @b3 the sums of its 5 x 5 and 3 x 3, @v 625 * the 5 x 5's
 * variance, @gu and @gd the gains. Free of branches to be vectorised
 */
static inline QEMU_ALWAYS_INLINE uint8_t MDNS_K(sharp_px)(
    int32_t v, int32_t b5, int32_t b3, int32_t c, int32_t l, int32_t r,
    int32_t u, int32_t d, int32_t gu, int32_t gd)
{
    /*
     * the high-pass in 1/1024ths (taps 228, 106, 17 and -45, summing to
     * 0), by the gain and the gate; the gate is clamped before it is
     * scaled, which would pass 31 bits at a variance of 8.4 million
     */
    int32_t high = 62 * b3 + 89 * (l + r + u + d) + 211 * c - 45 * b5;
    int32_t t = MIN(MAX(v - MDNS_SHARP_FLAT, 0),
                    MDNS_SHARP_FULL - MDNS_SHARP_FLAT);
    int32_t gate = (uint32_t)t * 256 / (MDNS_SHARP_FULL - MDNS_SHARP_FLAT);
    int32_t g = high > 0 ? gu : gd;
    int32_t e = (((high * g + 128) >> 8) * gate + (1 << 17)) >> 18;

    return MIN(MAX(c + e, 0), 255);
}

/* A row's sharpened samples, @v, @b5 and @b3 from isp_mdns_sharp_var() */
static void MDNS_ATTR MDNS_K(sharp_run)(
    uint8_t *restrict o, const int32_t *restrict v,
    const uint16_t *restrict b5, const uint16_t *restrict b3,
    const uint8_t *restrict r, const uint8_t *restrict ru,
    const uint8_t *restrict rd, const int32_t *restrict gu,
    const int32_t *restrict gd, uint32_t w)
{
    for (uint32_t x = 1; x + 1 < w; x++) {
        int32_t g1 = gu[x], g2 = gd[x];

        o[x] = MDNS_K(sharp_px)(v[x], b5[x], b3[x], r[x], r[x - 1], r[x + 1],
                                ru[x], rd[x], g1, g2);
    }
    /* the edge columns repeat their own */
    o[0] = MDNS_K(sharp_px)(v[0], b5[0], b3[0], r[0], r[0], r[1], ru[0],
                            rd[0], gu[0], gd[0]);
    o[w - 1] = MDNS_K(sharp_px)(v[w - 1], b5[w - 1], b3[w - 1], r[w - 1],
                                r[w - 2], r[w - 1], ru[w - 1], rd[w - 1],
                                gu[w - 1], gd[w - 1]);
}

/* A row of the picture into the window's column sums, @in3: also its 3 x 3 */
static void MDNS_ATTR MDNS_K(window_add)(uint16_t *restrict c5,
                                         uint16_t *restrict c3,
                                         uint32_t *restrict q5,
                                         const uint8_t *restrict r,
                                         bool in3, uint32_t w)
{
    for (size_t x = 0; x < w; x++) {
        c5[x + 2] += r[x];
        q5[x + 2] += r[x] * r[x];
        c3[x + 2] += in3 ? r[x] : 0;
    }
}

/* The window one row down: @in5 and @in3 rows in, @out5 and @out3 out */
static void MDNS_ATTR MDNS_K(window_slide)(uint16_t *restrict c5,
                                           uint16_t *restrict c3,
                                           uint32_t *restrict q5,
                                           const uint8_t *restrict in5,
                                           const uint8_t *restrict out5,
                                           const uint8_t *restrict in3,
                                           const uint8_t *restrict out3,
                                           uint32_t w)
{
    for (size_t x = 0; x < w; x++) {
        c5[x + 2] += in5[x] - out5[x];
        q5[x + 2] += in5[x] * in5[x] - out5[x] * out5[x];
        c3[x + 2] += in3[x] - out3[x];
    }
}

/* Rows y0 to y1 of the picture, smoothed and then, if on, sharpened */
static void MDNS_ATTR MDNS_K(band)(const IspMdnsJob *p)
{
    uint32_t w = p->w, h = p->h;
    g_autofree uint16_t *taps = g_malloc((size_t)w * 5 * 2);
    g_autofree int32_t *dv = g_malloc((size_t)w * 4);
    IspMdnsSpa sp = { .p = p, .dv = dv };
    g_autofree uint8_t *sr = NULL;
    g_autofree uint16_t *c5 = NULL, *c3 = NULL;
    g_autofree uint32_t *q5 = NULL;
    g_autofree int32_t *gu = NULL, *gd = NULL, *vr = NULL;
    g_autofree uint16_t *b5 = NULL, *b3 = NULL;
    uint32_t next, by = UINT32_MAX;

    for (unsigned k = 0; k < 5; k++) {
        sp.ring[k] = taps + (size_t)k * w;
    }
    if (!p->sharp) {
        MDNS_K(spa_start)(&sp, p->y0);
        for (uint32_t y = p->y0; y < p->y1; y++) {
            MDNS_K(spa_row)(&sp, y, p->dst + (size_t)y * w);
        }
        return;
    }

    sr = g_malloc((size_t)w * MDNS_RING);
    c5 = g_malloc0((size_t)(w + 6) * 2);
    c3 = g_malloc0((size_t)(w + 6) * 2);
    q5 = g_malloc0((size_t)(w + 6) * 4);
    gu = g_malloc((size_t)w * 4);
    gd = g_malloc((size_t)w * 4);
    vr = g_malloc((size_t)w * 4);
    b5 = g_malloc((size_t)w * 2);
    b3 = g_malloc((size_t)w * 2);
    next = p->y0 > 2 ? p->y0 - 2 : 0;
    MDNS_K(spa_start)(&sp, next);
    MDNS_K(spa_to)(&sp, sr, &next, p->y0 + 2);
    /* the column sums of the rows around the band's first, edges repeated */
    for (int k = -2; k <= 2; k++) {
        MDNS_K(window_add)(c5, c3, q5,
                           MDNS_K(ring)(sr, (int32_t)p->y0 + k, w, h),
                           k >= -1 && k <= 1, w);
    }
    for (uint32_t y = p->y0; y < p->y1; y++) {
        const uint8_t *r = MDNS_K(ring)(sr, y, w, h);
        const uint8_t *ru = MDNS_K(ring)(sr, (int32_t)y - 1, w, h);
        const uint8_t *rd = MDNS_K(ring)(sr, (int32_t)y + 1, w, h);
        uint32_t nby = y < p->gy ? 0 : MIN((y - p->gy) / p->bsz, p->bh - 1);
        uint8_t *o = p->dst + (size_t)y * w;

        MDNS_K(spa_to)(&sp, sr, &next, y + 2);
        if (nby != by) {
            /* a block's samples share their gains */
            const uint8_t *ar = p->age + (size_t)nby * p->bw;

            by = nby;
            for (uint32_t x = 0; x < w; x++) {
                gu[x] = p->gr[ar[p->xb[x]] & 15];
                gd[x] = p->gd[ar[p->xb[x]] & 15];
            }
        }
        c5[0] = c5[1] = c5[2];
        c5[w + 2] = c5[w + 3] = c5[w + 1];
        c3[0] = c3[1] = c3[2];
        c3[w + 2] = c3[w + 3] = c3[w + 1];
        q5[0] = q5[1] = q5[2];
        q5[w + 2] = q5[w + 3] = q5[w + 1];
        MDNS_K(sharp_var)(vr, b5, b3, c5, c3, q5, w);
        MDNS_K(sharp_run)(o, vr, b5, b3, r, ru, rd, gu, gd, w);
        if (y + 1 < p->y1) {
            /* the next row's window: one row in, one out */
            MDNS_K(spa_to)(&sp, sr, &next, y + 3);
            MDNS_K(window_slide)(c5, c3, q5,
                                 MDNS_K(ring)(sr, (int32_t)y + 3, w, h),
                                 MDNS_K(ring)(sr, (int32_t)y - 2, w, h),
                                 MDNS_K(ring)(sr, (int32_t)y + 2, w, h),
                                 ru, w);
        }
    }
}
