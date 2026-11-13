/*
 * Ingenic AVPU JPEG encoder: the per-block kernels (gather, DCT, quantizer)
 *
 * Included twice by ingenic-avpu-jpeg.c, as JK(n) for the host's baseline
 * and for AVX2. The lanes are 8 int32: the compiler lowers them to what the
 * build has. Every product and rounding is the scalar DCT's, bit for bit.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef JK_COMMON
#define JK_COMMON

#define JK_AI       QEMU_ALWAYS_INLINE
#define JK_UNROLL   _Pragma("GCC unroll 8")


typedef int32_t JkV8 __attribute__((vector_size(32)));
typedef uint8_t JkB8 __attribute__((vector_size(8)));
typedef uint8_t JkB16 __attribute__((vector_size(16)));

/* elements of @a then @b, by index: eight of them */
#ifdef __clang__
#define JK_SHUF(a, b, i0, i1, i2, i3, i4, i5, i6, i7) \
    __builtin_shufflevector(a, b, i0, i1, i2, i3, i4, i5, i6, i7)
#else
#define JK_SHUF(a, b, i0, i1, i2, i3, i4, i5, i6, i7) \
    __builtin_shuffle(a, b, (JkV8){ i0, i1, i2, i3, i4, i5, i6, i7 })
#endif

/* the even (@c = 0) or odd bytes of 16 */
static inline JK_AI JkB8 jk_pick(JkB16 b, int c)
{
    JkB8 r;
#ifdef __clang__
    r = c ? __builtin_shufflevector(b, b, 1, 3, 5, 7, 9, 11, 13, 15)
          : __builtin_shufflevector(b, b, 0, 2, 4, 6, 8, 10, 12, 14);
#else
    JkB16 m = c ? __builtin_shuffle(b, (JkB16){ 1, 3, 5, 7, 9, 11, 13, 15,
                                                1, 3, 5, 7, 9, 11, 13, 15 })
                : __builtin_shuffle(b, (JkB16){ 0, 2, 4, 6, 8, 10, 12, 14,
                                                0, 2, 4, 6, 8, 10, 12, 14 });

    memcpy(&r, &m, sizeof(r));
#endif
    return r;
}

static inline JK_AI JkV8 jk_load(const int32_t *p)
{
    JkV8 v;

    memcpy(&v, p, sizeof(v));
    return v;
}

static inline JK_AI void jk_store(int32_t *p, JkV8 v)
{
    memcpy(p, &v, sizeof(v));
}
#endif

/* a vector of one value */
static inline JK_AI JK_ATTR JkV8 JK(bcast)(int32_t x)
{
#ifdef JK_AVX2
    return (JkV8)_mm256_set1_epi32(x);
#else
    return (JkV8){ 0 } + x;
#endif
}

/*
 * The samples less 128 of a block wholly inside the picture: luma, or
 * chroma @c of the interleaved plane (@src to its first byte of the pair;
 * 16 bytes a row are read, and a row of 8 pairs has them)
 */
static inline JK_AI JK_ATTR void JK(gather_y)(int32_t px[64],
                                              const uint8_t *src,
                                              size_t stride)
{
    JK_UNROLL
    for (int j = 0; j < 8; j++) {
#ifdef JK_AVX2
        __m128i b = _mm_loadl_epi64((const __m128i *)(src + j * stride));

        jk_store(px + 8 * j, (JkV8)_mm256_cvtepu8_epi32(b) - 128);
#else
        JkB8 b;

        memcpy(&b, src + j * stride, sizeof(b));
        jk_store(px + 8 * j, __builtin_convertvector(b, JkV8) - 128);
#endif
    }
}

static inline JK_AI JK_ATTR void JK(gather_c)(int32_t px[64],
                                              const uint8_t *src,
                                              size_t stride, int c)
{
    JK_UNROLL
    for (int j = 0; j < 8; j++) {
#ifdef JK_AVX2
        __m128i b = _mm_loadu_si128((const __m128i *)(src + j * stride));
        __m128i m = _mm_set_epi8(-1, -1, -1, -1, -1, -1, -1, -1, 14, 12, 10, 8,
                                 6, 4, 2, 0);

        if (c) {
            m = _mm_add_epi8(m, _mm_set_epi64x(0, 0x0101010101010101));
        }
        b = _mm_shuffle_epi8(b, m);
        jk_store(px + 8 * j, (JkV8)_mm256_cvtepu8_epi32(b) - 128);
#else
        JkB16 b;

        memcpy(&b, src + j * stride, sizeof(b));
        jk_store(px + 8 * j, __builtin_convertvector(jk_pick(b, c), JkV8) -
                             128);
#endif
    }
}

/* A Q11 product rounded half up, 8 lanes */
static inline JK_AI JK_ATTR JkV8 JK(q11)(JkV8 v)
{
    return (v + 1024) >> 11;
}

/* dct8() on 8 lanes: v[i] are the i'th values of 8 independent columns */
static inline JK_AI JK_ATTR void JK(dct8v)(JkV8 *v)
{
    JkV8 s0 = v[0] + v[7], s7 = v[0] - v[7];
    JkV8 s1 = v[1] + v[6], s6 = v[1] - v[6];
    JkV8 s2 = v[2] + v[5], s5 = v[2] - v[5];
    JkV8 s3 = v[3] + v[4], s4 = v[3] - v[4];
    JkV8 t0 = s0 + s3, t3 = s0 - s3, t1 = s1 + s2, t2 = s1 - s2;
    JkV8 t4 = JK(q11)(1703 * s4 + 1138 * s7);
    JkV8 t7 = JK(q11)(1703 * s7 - 1138 * s4);
    JkV8 t5 = JK(q11)(2008 * s5 + 400 * s6);
    JkV8 t6 = JK(q11)(2008 * s6 - 400 * s5);
    JkV8 u4 = t4 + t6, u6 = t4 - t6, u7 = t7 + t5, u5 = t7 - t5;

    v[0] = t0 + t1;
    v[4] = t0 - t1;
    v[2] = JK(q11)(1108 * t2 + 2676 * t3);
    v[6] = JK(q11)(1108 * t3 - 2676 * t2);
    v[1] = u7 + u4;
    v[7] = u7 - u4;
    v[3] = JK(q11)(2896 * u5);
    v[5] = JK(q11)(2896 * u6);
}

/* 8 vectors as the rows of a matrix, to its columns */
static inline JK_AI JK_ATTR void JK(transpose)(JkV8 *v)
{
    JkV8 a0 = JK_SHUF(v[0], v[1], 0, 8, 1, 9, 4, 12, 5, 13);
    JkV8 a1 = JK_SHUF(v[0], v[1], 2, 10, 3, 11, 6, 14, 7, 15);
    JkV8 a2 = JK_SHUF(v[2], v[3], 0, 8, 1, 9, 4, 12, 5, 13);
    JkV8 a3 = JK_SHUF(v[2], v[3], 2, 10, 3, 11, 6, 14, 7, 15);
    JkV8 a4 = JK_SHUF(v[4], v[5], 0, 8, 1, 9, 4, 12, 5, 13);
    JkV8 a5 = JK_SHUF(v[4], v[5], 2, 10, 3, 11, 6, 14, 7, 15);
    JkV8 a6 = JK_SHUF(v[6], v[7], 0, 8, 1, 9, 4, 12, 5, 13);
    JkV8 a7 = JK_SHUF(v[6], v[7], 2, 10, 3, 11, 6, 14, 7, 15);
    JkV8 b0 = JK_SHUF(a0, a2, 0, 1, 8, 9, 4, 5, 12, 13);
    JkV8 b1 = JK_SHUF(a0, a2, 2, 3, 10, 11, 6, 7, 14, 15);
    JkV8 b2 = JK_SHUF(a1, a3, 0, 1, 8, 9, 4, 5, 12, 13);
    JkV8 b3 = JK_SHUF(a1, a3, 2, 3, 10, 11, 6, 7, 14, 15);
    JkV8 b4 = JK_SHUF(a4, a6, 0, 1, 8, 9, 4, 5, 12, 13);
    JkV8 b5 = JK_SHUF(a4, a6, 2, 3, 10, 11, 6, 7, 14, 15);
    JkV8 b6 = JK_SHUF(a5, a7, 0, 1, 8, 9, 4, 5, 12, 13);
    JkV8 b7 = JK_SHUF(a5, a7, 2, 3, 10, 11, 6, 7, 14, 15);

    v[0] = JK_SHUF(b0, b4, 0, 1, 2, 3, 8, 9, 10, 11);
    v[1] = JK_SHUF(b1, b5, 0, 1, 2, 3, 8, 9, 10, 11);
    v[2] = JK_SHUF(b2, b6, 0, 1, 2, 3, 8, 9, 10, 11);
    v[3] = JK_SHUF(b3, b7, 0, 1, 2, 3, 8, 9, 10, 11);
    v[4] = JK_SHUF(b0, b4, 4, 5, 6, 7, 12, 13, 14, 15);
    v[5] = JK_SHUF(b1, b5, 4, 5, 6, 7, 12, 13, 14, 15);
    v[6] = JK_SHUF(b2, b6, 4, 5, 6, 7, 12, 13, 14, 15);
    v[7] = JK_SHUF(b3, b7, 4, 5, 6, 7, 12, 13, 14, 15);
}

/* dct_el200(): rows, then columns, with the lanes the other way across */
static inline JK_AI JK_ATTR void JK(dct_el200)(int32_t px[64])
{
    JkV8 v[8];

    JK_UNROLL
    for (int i = 0; i < 8; i++) {
        v[i] = jk_load(px + 8 * i);
    }
    JK(transpose)(v);
    JK(dct8v)(v);
    JK(transpose)(v);
    JK(dct8v)(v);
    JK_UNROLL
    for (int i = 0; i < 8; i++) {
        jk_store(px + 8 * i, v[i]);
    }
}

/*
 * dct_t32(): rows with each sample's products over the 8 outputs at once
 * (jpeg_t32_abs is |cos| by input, jpeg_t32_neg its sign), then columns
 */
static inline JK_AI JK_ATTR void JK(dct_t32)(int32_t px[64])
{
    JkV8 y[8];

    JK_UNROLL
    for (int r = 0; r < 8; r++) {
        JkV8 acc = JK(bcast)(0);

        JK_UNROLL
        for (int i = 0; i < 8; i++) {
            JkV8 p = (JK(bcast)(px[r * 8 + i]) * jk_load(jpeg_t32_abs[i]) +
                      128) >> 8;
            JkV8 m = jk_load(jpeg_t32_neg[i]);

            acc += (p ^ m) - m;
        }
        y[r] = acc;
    }
    JK_UNROLL
    for (int k = 0; k < 8; k++) {
        JkV8 acc = JK(bcast)(0), m;

        JK_UNROLL
        for (int i = 0; i < 8; i++) {
            int32_t c = t32_cos[k][i];
            JkV8 p = (y[i] * abs(c) + 8192) >> 14;

            acc += c < 0 ? -p : p;
        }
        acc = (acc + 2) >> 2;
        m = acc > 1023;
        acc = (acc & ~m) | (1023 & m);
        m = acc < -1023;
        acc = (acc & ~m) | (-1023 & m);
        jk_store(px + 8 * k, acc);
    }
}

/* which of 8 lanes are not 0, as bits */
static inline JK_AI JK_ATTR unsigned JK(nzbits)(JkV8 v)
{
#ifdef JK_AVX2
    return _mm256_movemask_ps((__m256)(v != 0));
#else
    unsigned bits = 0;

    JK_UNROLL
    for (int i = 0; i < 8; i++) {
        bits |= (v[i] != 0) << i;
    }
    return bits;
#endif
}

/*
 * One block's quantized coefficients in natural order, and which of them
 * but the DC are not 0 as zigzag-order bits, from the samples less 128 in
 * @px (the DCT's working space)
 */
static inline JK_AI JK_ATTR uint64_t JK(block)(int32_t px[64], bool t32,
                                               int16_t *q, const int32_t *mul,
                                               const int32_t *half,
                                               const int32_t *shift)
{
    uint64_t nz = 0;

    if (t32) {
        JK(dct_t32)(px);
    } else {
        JK(dct_el200)(px);
    }
    JK_UNROLL
    for (int i = 0; i < 8; i++) {
        typedef int16_t JkS8 __attribute__((vector_size(16)));
        JkV8 n = (jk_load(px + 8 * i) * jk_load(mul + 8 * i) +
                  jk_load(half + 8 * i)) >> jk_load(shift + 8 * i);
        JkS8 w = __builtin_convertvector(n, JkS8);

        memcpy(q + 8 * i, &w, sizeof(w));
        nz |= jpeg_zz_bits[i][JK(nzbits)(n)];
    }
    return nz & ~1ull;
}

/* the clamped samples of a block that is not wholly inside */
static inline JK_AI JK_ATTR void JK(gather_edge)(int32_t px[64],
                                                 const uint8_t *src,
                                                 size_t stride, size_t step,
                                                 uint32_t x0, uint32_t y0,
                                                 uint32_t w, uint32_t h)
{
    for (uint32_t j = 0; j < 8; j++) {
        uint32_t yy = MIN(y0 + j, h - 1);

        for (uint32_t i = 0; i < 8; i++) {
            uint32_t xx = MIN(x0 + i, w - 1);

            px[j * 8 + i] = src[yy * stride + xx * step] - 128;
        }
    }
}

/* The MCU rows @r0 to @r1 of @j, to its coefficient and mask arrays */
static JK_ATTR void JK(band)(const IngenicAvpuJpegJob *j, uint32_t r0,
                             uint32_t r1)
{
    const uint32_t cw = (j->width + 1) / 2, ch = (j->height + 1) / 2;

    for (uint32_t r = r0; r < r1; r++) {
        for (uint32_t m = 0; m < j->mcux; m++) {
            size_t mcu = (size_t)r * j->mcux + m;
            int16_t *q = j->coef + mcu * 6 * 64;
            uint64_t *nz = j->nz + mcu * 6;
            uint32_t mx = m * 16, my = r * 16;

            for (int b = 0; b < 4; b++) {
                uint32_t x0 = mx + (b & 1) * 8, y0 = my + (b >> 1) * 8;
                int32_t px[64];

                if (x0 >= j->width || y0 >= j->height) {
                    nz[b] = UINT64_MAX;
                    continue;
                }
                if (x0 + 8 <= j->width && y0 + 8 <= j->height) {
                    JK(gather_y)(px, j->y + y0 * j->ystride + x0, j->ystride);
                } else {
                    JK(gather_edge)(px, j->y, j->ystride, 1, x0, y0, j->width,
                                    j->height);
                }
                nz[b] = JK(block)(px, j->t32, q + b * 64, j->qmul[0],
                                  j->qhalf[0], j->qshift[0]);
            }
            for (int c = 0; c < 2; c++) {
                uint32_t x0 = mx / 2, y0 = my / 2;
                const uint8_t *src = j->uv + c;
                int32_t px[64];

                if (x0 + 8 <= cw && y0 + 8 <= ch) {
                    JK(gather_c)(px, j->uv + y0 * j->cstride + x0 * 2,
                                 j->cstride, c);
                } else {
                    JK(gather_edge)(px, src, j->cstride, 2, x0, y0, cw, ch);
                }
                nz[4 + c] = JK(block)(px, j->t32, q + (4 + c) * 64,
                                      j->qmul[1], j->qhalf[1], j->qshift[1]);
            }
        }
    }
}
