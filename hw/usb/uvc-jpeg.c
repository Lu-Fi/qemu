/*
 * A baseline JPEG encoder for the UVC camera's MJPEG frames
 *
 * 4:2:0, the Annex K quantization and Huffman tables, a float AAN forward DCT
 * whose scale factors are folded into the quantizer's reciprocals.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "uvc-jpeg.h"

static const uint8_t zigzag[64] = {
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6, 7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

static const uint8_t qluma[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68, 109, 103, 77,
    24, 35, 55, 64, 81, 104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101,
    72, 92, 95, 98, 112, 100, 103, 99,
};

static const uint8_t qchroma[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
};

typedef struct HuffSpec {
    uint8_t bits[16];
    const uint8_t *vals;
    unsigned nvals;
} HuffSpec;

static const uint8_t dc_vals[12] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static const uint8_t ac_luma_vals[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
    0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
    0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

static const uint8_t ac_chroma_vals[162] = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
    0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
    0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
    0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
    0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
    0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
    0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa,
};

/* DC luma, AC luma, DC chroma, AC chroma */
static const HuffSpec huff_specs[4] = {
    { { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 }, dc_vals, 12 },
    { { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d },
      ac_luma_vals, 162 },
    { { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 }, dc_vals, 12 },
    { { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 },
      ac_chroma_vals, 162 },
};

typedef struct HuffCode {
    uint16_t code[256];
    uint8_t size[256];
} HuffCode;

typedef struct Enc {
    GByteArray *out;
    uint32_t acc;
    unsigned nbits;
    HuffCode huff[4];
    float qrecip[2][64];        /* natural order, DCT scale folded in */
} Enc;

static void huff_build(HuffCode *h, const HuffSpec *s)
{
    unsigned code = 0, k = 0;

    memset(h, 0, sizeof(*h));
    for (unsigned len = 1; len <= 16; len++) {
        for (unsigned i = 0; i < s->bits[len - 1]; i++) {
            h->code[s->vals[k]] = code++;
            h->size[s->vals[k]] = len;
            k++;
        }
        code <<= 1;
    }
    assert(k == s->nvals);
}

static void put_bits(Enc *e, unsigned code, unsigned n)
{
    e->acc = (e->acc << n) | (code & ((1u << n) - 1));
    e->nbits += n;
    while (e->nbits >= 8) {
        uint8_t b = e->acc >> (e->nbits - 8);

        g_byte_array_append(e->out, &b, 1);
        if (b == 0xff) {
            b = 0;
            g_byte_array_append(e->out, &b, 1);
        }
        e->nbits -= 8;
    }
}

static void put_u16(GByteArray *a, unsigned v)
{
    uint8_t b[2] = { v >> 8, v };

    g_byte_array_append(a, b, 2);
}

static void put_marker(GByteArray *a, uint8_t m, unsigned len)
{
    uint8_t b[2] = { 0xff, m };

    g_byte_array_append(a, b, 2);
    if (len) {
        put_u16(a, len + 2);
    }
}

static void fdct_1d(float *d, int stride)
{
    float t0 = d[0] + d[7 * stride], t7 = d[0] - d[7 * stride];
    float t1 = d[stride] + d[6 * stride], t6 = d[stride] - d[6 * stride];
    float t2 = d[2 * stride] + d[5 * stride];
    float t5 = d[2 * stride] - d[5 * stride];
    float t3 = d[3 * stride] + d[4 * stride];
    float t4 = d[3 * stride] - d[4 * stride];
    float t10 = t0 + t3, t13 = t0 - t3, t11 = t1 + t2, t12 = t1 - t2;
    float z1, z2, z3, z4, z5, z11, z13;

    d[0] = t10 + t11;
    d[4 * stride] = t10 - t11;
    z1 = (t12 + t13) * 0.707106781f;
    d[2 * stride] = t13 + z1;
    d[6 * stride] = t13 - z1;

    t10 = t4 + t5;
    t11 = t5 + t6;
    t12 = t6 + t7;
    z5 = (t10 - t12) * 0.382683433f;
    z2 = t10 * 0.541196100f + z5;
    z4 = t12 * 1.306562965f + z5;
    z3 = t11 * 0.707106781f;
    z11 = t7 + z3;
    z13 = t7 - z3;
    d[5 * stride] = z13 + z2;
    d[3 * stride] = z13 - z2;
    d[stride] = z11 + z4;
    d[7 * stride] = z11 - z4;
}

static void put_value(Enc *e, const HuffCode *h, unsigned sym, int v,
                      unsigned cat)
{
    put_bits(e, h->code[sym], h->size[sym]);
    if (cat) {
        put_bits(e, v < 0 ? v - 1 : v, cat);
    }
}

static unsigned category(int v)
{
    unsigned a = v < 0 ? -v : v, n = 0;

    while (a) {
        n++;
        a >>= 1;
    }
    return n;
}

/* @blk: 64 level-shifted samples; @t: 0 luma, 1 chroma */
static void encode_block(Enc *e, float *blk, int t, int *pred)
{
    const HuffCode *dc = &e->huff[t * 2], *ac = &e->huff[t * 2 + 1];
    int q[64], last = 0;

    for (int i = 0; i < 8; i++) {
        fdct_1d(blk + i * 8, 1);
    }
    for (int i = 0; i < 8; i++) {
        fdct_1d(blk + i, 8);
    }
    for (int i = 0; i < 64; i++) {
        float x = blk[zigzag[i]] * e->qrecip[t][zigzag[i]];

        q[i] = (int)(x < 0 ? x - 0.5f : x + 0.5f);
        if (q[i]) {
            last = i;
        }
    }

    int diff = q[0] - *pred;
    unsigned cat = category(diff);

    *pred = q[0];
    put_value(e, dc, cat, diff, cat);

    int run = 0;

    for (int i = 1; i <= last && last; i++) {
        if (!q[i]) {
            run++;
            continue;
        }
        while (run > 15) {
            put_bits(e, ac->code[0xf0], ac->size[0xf0]);
            run -= 16;
        }
        cat = category(q[i]);
        put_value(e, ac, (run << 4) | cat, q[i], cat);
        run = 0;
    }
    if (last < 63) {
        put_bits(e, ac->code[0], ac->size[0]);
    }
}

static void fetch_luma(float *blk, const uint8_t *y, unsigned w, unsigned h,
                       unsigned x0, unsigned y0)
{
    for (unsigned j = 0; j < 8; j++) {
        unsigned yy = MIN(y0 + j, h - 1);

        for (unsigned i = 0; i < 8; i++) {
            blk[j * 8 + i] = y[yy * w + MIN(x0 + i, w - 1)] - 128.0f;
        }
    }
}

static void fetch_chroma(float *blk, const uint8_t *uv, unsigned w,
                         unsigned h, unsigned x0, unsigned y0, unsigned c)
{
    unsigned cw = w / 2, ch = h / 2;

    for (unsigned j = 0; j < 8; j++) {
        unsigned yy = MIN(y0 + j, ch - 1);

        for (unsigned i = 0; i < 8; i++) {
            blk[j * 8 + i] = uv[yy * w + 2 * MIN(x0 + i, cw - 1) + c] - 128.0f;
        }
    }
}

uint8_t *uvc_jpeg_encode(const uint8_t *nv12, unsigned w, unsigned h,
                         unsigned quality, size_t *len)
{
    static const float aasf[8] = {
        1.0f * 2.828427125f, 1.387039845f * 2.828427125f,
        1.306562965f * 2.828427125f, 1.175875602f * 2.828427125f,
        1.0f * 2.828427125f, 0.785694958f * 2.828427125f,
        0.541196100f * 2.828427125f, 0.275899379f * 2.828427125f,
    };
    static const uint8_t *const base[2] = { qluma, qchroma };
    uint8_t qt[2][64];
    Enc e = { .out = g_byte_array_new() };
    const uint8_t *y = nv12, *uv = nv12 + (size_t)w * h;
    unsigned scale;
    int pred[3] = { 0, 0, 0 };
    float blk[64];

    quality = CLAMP(quality, 1, 100);
    scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;
    for (int t = 0; t < 2; t++) {
        for (int k = 0; k < 64; k++) {
            unsigned v = CLAMP((base[t][k] * scale + 50) / 100, 1, 255);

            qt[t][k] = v;
            e.qrecip[t][k] = 1.0f / (v * aasf[k >> 3] * aasf[k & 7]);
        }
    }
    for (int i = 0; i < 4; i++) {
        huff_build(&e.huff[i], &huff_specs[i]);
    }

    put_marker(e.out, 0xd8, 0);
    for (int t = 0; t < 2; t++) {
        uint8_t id = t;

        put_marker(e.out, 0xdb, 65);
        g_byte_array_append(e.out, &id, 1);
        for (int k = 0; k < 64; k++) {
            g_byte_array_append(e.out, &qt[t][zigzag[k]], 1);
        }
    }
    put_marker(e.out, 0xc0, 15);
    {
        uint8_t sof[15] = { 8, h >> 8, h, w >> 8, w, 3,
                            1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1 };

        g_byte_array_append(e.out, sof, sizeof(sof));
    }
    for (int i = 0; i < 4; i++) {
        /* class/id byte: DC0, AC0, DC1, AC1 */
        uint8_t tc = ((i & 1) << 4) | (i >> 1);

        put_marker(e.out, 0xc4, 17 + huff_specs[i].nvals);
        g_byte_array_append(e.out, &tc, 1);
        g_byte_array_append(e.out, huff_specs[i].bits, 16);
        g_byte_array_append(e.out, huff_specs[i].vals, huff_specs[i].nvals);
    }
    put_marker(e.out, 0xda, 10);
    {
        uint8_t sos[10] = { 3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0 };

        g_byte_array_append(e.out, sos, sizeof(sos));
    }

    for (unsigned my = 0; my < h; my += 16) {
        for (unsigned mx = 0; mx < w; mx += 16) {
            for (unsigned b = 0; b < 4; b++) {
                fetch_luma(blk, y, w, h, mx + (b & 1) * 8, my + (b >> 1) * 8);
                encode_block(&e, blk, 0, &pred[0]);
            }
            for (unsigned c = 0; c < 2; c++) {
                fetch_chroma(blk, uv, w, h, mx / 2, my / 2, c);
                encode_block(&e, blk, 1, &pred[1 + c]);
            }
        }
    }
    if (e.nbits) {
        put_bits(&e, 0x7f, 8 - e.nbits);
    }
    put_marker(e.out, 0xd9, 0);

    *len = e.out->len;
    return g_byte_array_free(e.out, false);
}
