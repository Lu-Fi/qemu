/*
 * Ingenic AVPU: the JPEG core's baseline encoder
 *
 * The T31 JPEG core writes the whole JFIF file: SOI, APP0 (JFIF 1.1,
 * aspect 1:1), the luma and chroma DQT from the table buffer, SOF0 (4:2:0),
 * the four Annex K Huffman tables, SOS, the scan and EOI. It pads with COM
 * segments so that each table body starts on 4 bytes, and always before
 * SOS so that the scan starts on 8, as a real T31 lays its snapshots out.
 * It codes a luma block wholly outside the picture (the bottom of a
 * 360-line one) as the previous DC again without AC, and repeats the last
 * row and column into a block the picture covers in part.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-avpu.h"
#include <math.h>

static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* Annex K.3: the bit counts, then the values, of the four tables */
static const uint8_t dc_luma[] = {
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t dc_chroma[] = {
    0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t ac_luma[] = {
    0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d,
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
static const uint8_t ac_chroma[] = {
    0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77,
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

typedef struct Huff {
    uint16_t code[256];
    uint8_t size[256];
} Huff;

typedef struct Writer {
    GByteArray *out;
    uint32_t acc;
    int bits;
    uint32_t symbols;
} Writer;

static void huff_build(const uint8_t *spec, Huff *h)
{
    const uint8_t *vals = spec + 16;
    uint16_t code = 0;
    int k = 0;

    for (int len = 1; len <= 16; len++) {
        for (int i = 0; i < spec[len - 1]; i++) {
            h->code[vals[k]] = code++;
            h->size[vals[k]] = len;
            k++;
        }
        code <<= 1;
    }
}

static void put_byte(GByteArray *out, uint8_t b)
{
    g_byte_array_append(out, &b, 1);
}

static void put_be16(GByteArray *out, uint16_t v)
{
    put_byte(out, v >> 8);
    put_byte(out, v);
}

static void put_bits(Writer *w, uint32_t v, int n)
{
    w->acc = w->acc << n | (v & ((1u << n) - 1));
    w->bits += n;
    while (w->bits >= 8) {
        uint8_t b = w->acc >> (w->bits - 8);

        put_byte(w->out, b);
        if (b == 0xff) {
            put_byte(w->out, 0);        /* byte stuffing */
        }
        w->bits -= 8;
    }
}

static void put_code(Writer *w, const Huff *h, int sym)
{
    put_bits(w, h->code[sym], h->size[sym]);
    w->symbols++;
}

/* A COM segment of @total bytes (at least 4), to align what follows */
static void put_pad(GByteArray *out, int total)
{
    if (total) {
        put_be16(out, 0xfffe);
        put_be16(out, total - 2);
        for (int i = 4; i < total; i++) {
            put_byte(out, 0);
        }
    }
}

/*
 * Pad so that the byte @skip into the next segment lands on @align, with a
 * COM even when it already does if @always
 */
static void pad_to(GByteArray *out, int skip, int align, bool always)
{
    int t = (align - (out->len + skip) % align) % align;

    while ((t || always) && t < 4) {
        t += align;
    }
    put_pad(out, t);
}

static void put_dqt(GByteArray *out, int id, const uint8_t *qt)
{
    pad_to(out, 5, 4, false);
    put_be16(out, 0xffdb);
    put_be16(out, 67);
    put_byte(out, id);
    g_byte_array_append(out, qt, 64);
}

static void put_dht(GByteArray *out, int class_id, const uint8_t *spec)
{
    int n = 0;

    for (int i = 0; i < 16; i++) {
        n += spec[i];
    }
    pad_to(out, 5, 4, false);
    put_be16(out, 0xffc4);
    put_be16(out, 3 + 16 + n);
    put_byte(out, class_id);
    g_byte_array_append(out, spec, 16 + n);
}

static void fdct(const float in[64], float out[64])
{
    static float c[8][8];
    static gsize init;
    float tmp[64];

    if (g_once_init_enter(&init)) {
        for (int u = 0; u < 8; u++) {
            for (int x = 0; x < 8; x++) {
                c[u][x] = (u ? 0.5f : (float)M_SQRT1_2 / 2.0f) *
                          cosf((2 * x + 1) * u * (float)M_PI / 16.0f);
            }
        }
        g_once_init_leave(&init, 1);
    }
    for (int y = 0; y < 8; y++) {
        for (int u = 0; u < 8; u++) {
            float s = 0;

            for (int x = 0; x < 8; x++) {
                s += c[u][x] * in[y * 8 + x];
            }
            tmp[y * 8 + u] = s;
        }
    }
    for (int u = 0; u < 8; u++) {
        for (int v = 0; v < 8; v++) {
            float s = 0;

            for (int y = 0; y < 8; y++) {
                s += c[v][y] * tmp[y * 8 + u];
            }
            out[v * 8 + u] = s;
        }
    }
}

static int magnitude(int v)
{
    int n = 0;

    for (v = abs(v); v; v >>= 1) {
        n++;
    }
    return n;
}

static void put_block(Writer *w, const float px[64], const uint8_t *qt,
                      int *dc_pred, const Huff *dc, const Huff *ac)
{
    float f[64];
    int q[64], run = 0, d;

    fdct(px, f);
    for (int k = 0; k < 64; k++) {
        q[k] = (int)lrintf(f[zigzag[k]] / qt[k]);
    }
    d = q[0] - *dc_pred;
    *dc_pred = q[0];
    put_code(w, dc, magnitude(d));
    put_bits(w, d < 0 ? d - 1 : d, magnitude(d));
    for (int k = 1; k < 64; k++) {
        if (!q[k]) {
            run++;
            continue;
        }
        while (run > 15) {
            put_code(w, ac, 0xf0);
            run -= 16;
        }
        put_code(w, ac, run << 4 | magnitude(q[k]));
        put_bits(w, q[k] < 0 ? q[k] - 1 : q[k], magnitude(q[k]));
        run = 0;
    }
    if (run) {
        put_code(w, ac, 0x00);
    }
}

uint32_t ingenic_avpu_jpeg_encode(const uint8_t *y, const uint8_t *uv,
                                  uint32_t stride, uint32_t width,
                                  uint32_t height, const uint8_t qt[2][64],
                                  GByteArray *out)
{
    Huff hdcl, hdcc, hacl, hacc;
    Writer w = { .out = out };
    int pred[3] = { 0 };

    huff_build(dc_luma, &hdcl);
    huff_build(dc_chroma, &hdcc);
    huff_build(ac_luma, &hacl);
    huff_build(ac_chroma, &hacc);

    put_be16(out, 0xffd8);
    put_be16(out, 0xffe0);
    put_be16(out, 16);
    g_byte_array_append(out, (const uint8_t *)"JFIF\0\1\1\0\0\1\0\1\0\0", 14);
    put_dqt(out, 0, qt[0]);
    put_dqt(out, 1, qt[1]);
    put_be16(out, 0xffc0);
    put_be16(out, 17);
    put_byte(out, 8);
    put_be16(out, height);
    put_be16(out, width);
    put_byte(out, 3);
    g_byte_array_append(out, (const uint8_t *)"\1\x22\0\2\x11\1\3\x11\1", 9);
    put_dht(out, 0x00, dc_luma);
    put_dht(out, 0x10, ac_luma);
    put_dht(out, 0x01, dc_chroma);
    put_dht(out, 0x11, ac_chroma);
    pad_to(out, 14, 8, true);
    put_be16(out, 0xffda);
    put_be16(out, 12);
    g_byte_array_append(out, (const uint8_t *)"\3\1\0\2\x11\3\x11\0\x3f\0", 10);

    for (uint32_t my = 0; my < height; my += 16) {
        for (uint32_t mx = 0; mx < width; mx += 16) {
            float px[64];

            for (int b = 0; b < 4; b++) {
                if (mx + (b & 1) * 8 >= width || my + (b >> 1) * 8 >= height) {
                    put_code(&w, &hdcl, 0);
                    put_code(&w, &hacl, 0x00);
                    continue;
                }
                for (int j = 0; j < 8; j++) {
                    uint32_t yy = MIN(my + (b >> 1) * 8 + j, height - 1);

                    for (int i = 0; i < 8; i++) {
                        uint32_t xx = MIN(mx + (b & 1) * 8 + i, width - 1);

                        px[j * 8 + i] = y[yy * stride + xx] - 128.0f;
                    }
                }
                put_block(&w, px, qt[0], &pred[0], &hdcl, &hacl);
            }
            for (int c = 0; c < 2; c++) {
                for (int j = 0; j < 8; j++) {
                    uint32_t yy = MIN(my / 2 + j, (height + 1) / 2 - 1);

                    for (int i = 0; i < 8; i++) {
                        uint32_t xx = MIN(mx / 2 + i, (width + 1) / 2 - 1);

                        px[j * 8 + i] = uv[yy * stride + xx * 2 + c] - 128.0f;
                    }
                }
                put_block(&w, px, qt[1], &pred[1 + c], &hdcc, &hacc);
            }
        }
    }
    if (w.bits) {
        put_bits(&w, 0x7f, 8 - w.bits);     /* fill with ones */
    }
    put_be16(out, 0xffd9);
    return w.symbols;
}
