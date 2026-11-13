/*
 * Ingenic AVPU: the JPEG core's baseline encoder
 *
 * The JPEG core, the T31's and the T41's alike, writes the whole JFIF
 * file: SOI, APP0 (JFIF 1.1, with the density units and densities the job
 * gives: libimp sets 1:1 on a T31 and leaves 0 on a T41), the luma and
 * chroma DQT from the table buffer, SOF0 (4:2:0), the four Annex K Huffman
 * tables, SOS, the scan and EOI. It pads with COM segments so that each
 * table body starts on 4 bytes, and always before SOS so that the scan
 * starts on 8. Given a restart interval it writes DRI after that COM, which
 * then pads DRI to start on 8, and every interval's MCUs ends the bits
 * with ones, writes the next RSTn and starts the DC predictions over
 * (a T41NQ's EL200). It codes a luma block wholly outside the picture (the
 * bottom of a 360-line one) as the previous DC again without AC, and
 * repeats the last row and column into a block the picture covers in
 * part. With the DCT and quantizer below, the files are a real T31's and
 * T41NQ's byte for byte.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#ifdef CONFIG_AVX2_OPT
#include <immintrin.h>
#endif
#include "hw/misc/ingenic-avpu.h"
#ifdef CONFIG_AVX2_OPT
#include "host/cpuinfo.h"
#endif

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

typedef struct Writer {
    GByteArray *out;
    uint32_t acc;
    int bits;
    uint32_t symbols;
} Writer;

static void huff_build(const uint8_t *spec, IngenicAvpuJpegHuff *h)
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

static void put_code(Writer *w, const IngenicAvpuJpegHuff *h, int sym)
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

static void put_dqt(GByteArray *out, int id, const uint8_t *qt, bool pad)
{
    if (pad) {
        pad_to(out, 5, 4, false);
    }
    put_be16(out, 0xffdb);
    put_be16(out, 67);
    put_byte(out, id);
    g_byte_array_append(out, qt, 64);
}

static void put_dht(GByteArray *out, int class_id, const uint8_t *spec,
                    bool pad)
{
    int n = 0;

    for (int i = 0; i < 16; i++) {
        n += spec[i];
    }
    if (pad) {
        pad_to(out, 5, 4, false);
    }
    put_be16(out, 0xffc4);
    put_be16(out, 3 + 16 + n);
    put_byte(out, class_id);
    g_byte_array_append(out, spec, 16 + n);
}

/* A Q11 product rounded half up */
static int32_t q11(int32_t v)
{
    return (v + 1024) >> 11;
}

/*
 * Loeffler's 8-point DCT (11 multiplications), in sqrt(8) times orthonormal
 * units, on the 8 values @s apart: the rotations by 3 pi / 16, pi / 16 and
 * sqrt(2) 6 pi / 16 and the two sqrt(2) products take Q11 constants and
 * round, which is what the core computes, bit for bit.
 */
static void dct8(int32_t *v, int s)
{
    int32_t s0 = v[0] + v[7 * s], s7 = v[0] - v[7 * s];
    int32_t s1 = v[s] + v[6 * s], s6 = v[s] - v[6 * s];
    int32_t s2 = v[2 * s] + v[5 * s], s5 = v[2 * s] - v[5 * s];
    int32_t s3 = v[3 * s] + v[4 * s], s4 = v[3 * s] - v[4 * s];
    int32_t t0 = s0 + s3, t3 = s0 - s3, t1 = s1 + s2, t2 = s1 - s2;
    int32_t t4 = q11(1703 * s4 + 1138 * s7), t7 = q11(1703 * s7 - 1138 * s4);
    int32_t t5 = q11(2008 * s5 + 400 * s6), t6 = q11(2008 * s6 - 400 * s5);
    int32_t u4 = t4 + t6, u6 = t4 - t6, u7 = t7 + t5, u5 = t7 - t5;

    v[0] = t0 + t1;
    v[4 * s] = t0 - t1;
    v[2 * s] = q11(1108 * t2 + 2676 * t3);
    v[6 * s] = q11(1108 * t3 - 2676 * t2);
    v[s] = u7 + u4;
    v[7 * s] = u7 - u4;
    v[3 * s] = q11(2896 * u5);
    v[5 * s] = q11(2896 * u6);
}

/* Rows, then columns, leave 8 times the coefficients */
static void dct_el200(int32_t px[64])
{
    for (int i = 0; i < 8; i++) {
        dct8(px + i * 8, 1);
    }
    for (int i = 0; i < 8; i++) {
        dct8(px + i, 8);
    }
}

/* 2048 times twice the orthonormal DCT's matrix, by frequency */
static const int16_t t32_cos[8][8] = {
    {  1448,  1448,  1448,  1448,  1448,  1448,  1448,  1448 },
    {  2009,  1703,  1138,   400,  -400, -1138, -1703, -2009 },
    {  1892,   784,  -784, -1892, -1892,  -784,   784,  1892 },
    {  1703,  -400, -2009, -1138,  1138,  2009,   400, -1703 },
    {  1448, -1448, -1448,  1448,  1448, -1448, -1448,  1448 },
    {  1138, -2009,   400,  1703, -1703,  -400,  2009, -1138 },
    {   784, -1892,  1892,  -784,  -784,  1892, -1892,   784 },
    {   400, -1138,  1703, -2009,  2009, -1703,  1138,  -400 },
};

/* The products of @v with frequency @k's row, each rounded half up */
static int32_t t32_row(const int32_t *v, int s, int k, int shift)
{
    int32_t acc = 0;

    for (int i = 0; i < 8; i++) {
        int c = t32_cos[k][i];
        int32_t p = (v[i * s] * abs(c) + (1 << (shift - 1))) >> shift;

        acc += c < 0 ? -p : p;
    }
    return acc;
}

/*
 * A T32's JPEG engine's DCT: no fast algorithm but that matrix, rows then
 * columns, every product rounded half up before its sign goes on (rows
 * >> 8, keeping 3 bits; columns >> 14), then >> 2 half up and clamped to
 * +-1023: the coefficients themselves, a T32NQ's on every probe block
 */
static void dct_t32(int32_t px[64])
{
    int32_t y[64];

    for (int r = 0; r < 8; r++) {
        for (int k = 0; k < 8; k++) {
            y[r * 8 + k] = t32_row(px + r * 8, 1, k, 8);
        }
    }
    for (int c = 0; c < 8; c++) {
        for (int k = 0; k < 8; k++) {
            int32_t f = (t32_row(y + c, 8, k, 14) + 2) >> 2;

            px[k * 8 + c] = MIN(MAX(f, -1023), 1023);
        }
    }
}

/*
 * The same matrix the other way, as a T30's HELIX decodes: columns first,
 * every product rounded half up before its sign goes on (>> 10, keeping a
 * bit), then rows (>> 11), then >> 3 half up, 128 on and clamped; a
 * T30X's samples on every probe block
 */
void ingenic_avpu_jpeg_idct_t32(const int32_t coef[64], uint8_t *out,
                                int stride)
{
    int32_t col[64];

    for (int c = 0; c < 8; c++) {
        for (int y = 0; y < 8; y++) {
            int32_t acc = 0;

            for (int k = 0; k < 8; k++) {
                int t = t32_cos[k][y];
                int32_t p = (coef[k * 8 + c] * abs(t) + 512) >> 10;

                acc += t < 0 ? -p : p;
            }
            col[y * 8 + c] = acc;
        }
    }
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            int32_t acc = 0;

            for (int k = 0; k < 8; k++) {
                int t = t32_cos[k][x];
                int32_t p = (col[y * 8 + k] * abs(t) + 1024) >> 11;

                acc += t < 0 ? -p : p;
            }
            out[y * stride + x] = MIN(MAX(((acc + 4) >> 3) + 128, 0), 255);
        }
    }
}

typedef void JpegDct(int32_t px[64]);

static int magnitude(int v)
{
    int n = 0;

    for (v = abs(v); v; v >>= 1) {
        n++;
    }
    return n;
}

static void put_block(Writer *w, int32_t px[64], JpegDct *dct,
                      const uint32_t *mul, const uint8_t *shift,
                      int *dc_pred, const IngenicAvpuJpegHuff *dc,
                      const IngenicAvpuJpegHuff *ac)
{
    int q[64], run = 0, d;

    dct(px);
    for (int k = 0; k < 64; k++) {
        int64_t c = px[zigzag[k]];
        int sh = shift[zigzag[k]];

        q[k] = (c * mul[zigzag[k]] + (1ll << (sh - 1))) >> sh;
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

void ingenic_avpu_jpeg_annex_k(IngenicAvpuJpegHuff huff[4])
{
    huff_build(dc_luma, &huff[0]);
    huff_build(ac_luma, &huff[1]);
    huff_build(dc_chroma, &huff[2]);
    huff_build(ac_chroma, &huff[3]);
}

static void put_fill(Writer *w)
{
    if (w->bits) {
        put_bits(w, 0x7f, 8 - w->bits);     /* fill with ones */
    }
}

/* The scan a block at a time, as the core does it: the reference */
static uint32_t jpeg_scan_ref(const uint8_t *y, uint32_t ystride,
                              const uint8_t *uv, uint32_t cstride,
                              uint32_t width, uint32_t height, JpegDct *dct,
                              const uint32_t mul[2][64],
                              const uint8_t shift[2][64],
                              const IngenicAvpuJpegHuff huff[4],
                              uint16_t restart, GByteArray *out)
{
    Writer w = { .out = out };
    int pred[3] = { 0 };
    uint32_t mcu = 0;

    for (uint32_t my = 0; my < height; my += 16) {
        for (uint32_t mx = 0; mx < width; mx += 16, mcu++) {
            int32_t px[64];

            if (restart && mcu && !(mcu % restart)) {
                put_fill(&w);
                put_be16(out, 0xffd0 + (mcu / restart - 1) % 8);
                memset(pred, 0, sizeof(pred));
            }

            for (int b = 0; b < 4; b++) {
                if (mx + (b & 1) * 8 >= width || my + (b >> 1) * 8 >= height) {
                    put_code(&w, &huff[0], 0);
                    put_code(&w, &huff[1], 0x00);
                    continue;
                }
                for (int j = 0; j < 8; j++) {
                    uint32_t yy = MIN(my + (b >> 1) * 8 + j, height - 1);

                    for (int i = 0; i < 8; i++) {
                        uint32_t xx = MIN(mx + (b & 1) * 8 + i, width - 1);

                        px[j * 8 + i] = y[yy * ystride + xx] - 128;
                    }
                }
                put_block(&w, px, dct, mul[0], shift[0], &pred[0], &huff[0],
                          &huff[1]);
            }
            for (int c = 0; c < 2; c++) {
                for (int j = 0; j < 8; j++) {
                    uint32_t yy = MIN(my / 2 + j, (height + 1) / 2 - 1);

                    for (int i = 0; i < 8; i++) {
                        uint32_t xx = MIN(mx / 2 + i, (width + 1) / 2 - 1);

                        px[j * 8 + i] = uv[yy * cstride + xx * 2 + c] - 128;
                    }
                }
                put_block(&w, px, dct, mul[1], shift[1], &pred[1 + c], &huff[2],
                          &huff[3]);
            }
        }
    }
    put_fill(&w);
    return w.symbols;
}

/* t32_cos for the kernels: its magnitudes by input, and its signs */
static const int32_t jpeg_t32_abs[8][8] = {
    { 1448, 2009, 1892, 1703, 1448, 1138, 784, 400 },
    { 1448, 1703, 784, 400, 1448, 2009, 1892, 1138 },
    { 1448, 1138, 784, 2009, 1448, 400, 1892, 1703 },
    { 1448, 400, 1892, 1138, 1448, 1703, 784, 2009 },
    { 1448, 400, 1892, 1138, 1448, 1703, 784, 2009 },
    { 1448, 1138, 784, 2009, 1448, 400, 1892, 1703 },
    { 1448, 1703, 784, 400, 1448, 2009, 1892, 1138 },
    { 1448, 2009, 1892, 1703, 1448, 1138, 784, 400 },
};
static const int32_t jpeg_t32_neg[8][8] = {
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, -1, -1, -1, -1, -1 },
    { 0, 0, -1, -1, -1, 0, 0, 0 },
    { 0, 0, -1, -1, 0, 0, -1, -1 },
    { 0, -1, -1, 0, 0, -1, -1, 0 },
    { 0, -1, -1, 0, -1, -1, 0, -1 },
    { 0, -1, 0, 0, -1, 0, -1, 0 },
    { 0, -1, 0, -1, 0, -1, 0, -1 },
};

/*
 * zigzag-order bits by a natural-order byte of a block's non-zero map:
 * [byte of the map][its value]
 */
static uint64_t jpeg_zz_bits[8][256];

static gpointer jpeg_zz_init(gpointer unused)
{
    for (int k = 0; k < 64; k++) {
        int n = zigzag[k];

        for (int v = 0; v < 256; v++) {
            if (v >> (n & 7) & 1) {
                jpeg_zz_bits[n >> 3][v] |= 1ull << k;
            }
        }
    }
    return NULL;
}

/*
 * The fast scan: the blocks' DCT and quantizer in bands of MCU rows, which
 * may run at once, then the Huffman coding over the quantized blocks in
 * order. Where the tables are such that the quantizer's products do not fit
 * 32 bits or a coefficient 16, the reference does it all.
 */
struct IngenicAvpuJpegJob {
    const uint8_t *y, *uv;
    uint32_t ystride, cstride, width, height, mcux, mcuy;
    uint16_t restart;
    bool t32, slow;
    uint32_t mul[2][64];
    uint8_t shift[2][64];
    IngenicAvpuJpegHuff huff[4];
    uint32_t lut[4][256];       /* the codes and sizes: size << 16 | code */
    int32_t qmul[2][64], qhalf[2][64], qshift[2][64];   /* natural order */
    int16_t *coef;              /* 6 blocks of 64, natural order, an MCU */
    /* each block's non-zero ACs as zigzag bits; all ones: outside */
    uint64_t *nz;
};

/* a coefficient's magnitude any DCT gives is under this */
#define JPEG_COEF_MAX   16384

static bool jpeg_quant_fits(const uint32_t mul[64], const uint8_t shift[64])
{
    for (int k = 0; k < 64; k++) {
        uint64_t half;

        if (shift[k] < 1 || shift[k] > 30) {
            return false;
        }
        half = 1ull << (shift[k] - 1);
        if ((uint64_t)JPEG_COEF_MAX * mul[k] + half > INT32_MAX ||
            ((uint64_t)JPEG_COEF_MAX * mul[k] + half) >> shift[k] > INT16_MAX) {
            return false;
        }
    }
    return true;
}

IngenicAvpuJpegJob *ingenic_avpu_jpeg_job_new(const uint8_t *y,
                                              uint32_t ystride,
                                              const uint8_t *uv,
                                              uint32_t cstride,
                                              uint32_t width, uint32_t height,
                                              bool t32,
                                              const uint32_t mul[2][64],
                                              const uint8_t shift[2][64],
                                              const IngenicAvpuJpegHuff huff[4],
                                              uint16_t restart)
{
    static GOnce zz_once = G_ONCE_INIT;
    IngenicAvpuJpegJob *j = g_new0(IngenicAvpuJpegJob, 1);

    g_once(&zz_once, jpeg_zz_init, NULL);
    j->y = y;
    j->uv = uv;
    j->ystride = ystride;
    j->cstride = cstride;
    j->width = width;
    j->height = height;
    j->mcux = DIV_ROUND_UP(width, 16);
    j->mcuy = DIV_ROUND_UP(height, 16);
    j->restart = restart;
    j->t32 = t32;
    memcpy(j->mul, mul, sizeof(j->mul));
    memcpy(j->shift, shift, sizeof(j->shift));
    memcpy(j->huff, huff, sizeof(j->huff));
    for (int t = 0; t < 4; t++) {
        for (int k = 0; k < 256; k++) {
            int size = huff[t].size[k];

            j->lut[t][k] = size << 16 | (huff[t].code[k] & ((1u << size) - 1));
        }
    }
    j->slow = !width || !height ||
              !jpeg_quant_fits(mul[0], shift[0]) ||
              !jpeg_quant_fits(mul[1], shift[1]);
    if (!j->slow) {
        size_t mcus = (size_t)j->mcux * j->mcuy;

        for (int t = 0; t < 2; t++) {
            for (int k = 0; k < 64; k++) {
                j->qmul[t][k] = mul[t][k];
                j->qhalf[t][k] = 1 << (shift[t][k] - 1);
                j->qshift[t][k] = shift[t][k];
            }
        }
        j->coef = g_try_new(int16_t, mcus * 6 * 64);
        j->nz = g_try_new(uint64_t, mcus * 6);
        if (!j->coef || !j->nz) {
            g_free(j->coef);
            g_free(j->nz);
            j->coef = NULL;
            j->nz = NULL;
            j->slow = true;
        }
    }
    return j;
}

#define JK(n)           jpeg_k_##n##_dflt
#define JK_ATTR
#include "ingenic-avpu-jpeg-kern.c.inc"
#undef JK
#undef JK_ATTR

#ifdef CONFIG_AVX2_OPT
#define JK(n)           jpeg_k_##n##_avx2
#define JK_ATTR         __attribute__((target("avx2")))
#define JK_AVX2
#include "ingenic-avpu-jpeg-kern.c.inc"
#undef JK
#undef JK_ATTR
#undef JK_AVX2
#endif

void ingenic_avpu_jpeg_job_band(const IngenicAvpuJpegJob *j, unsigned band,
                                unsigned bands)
{
    uint32_t r0 = (uint64_t)j->mcuy * band / bands;
    uint32_t r1 = (uint64_t)j->mcuy * (band + 1) / bands;

    if (j->slow) {
        return;
    }
#ifdef CONFIG_AVX2_OPT
    if (cpuinfo & CPUINFO_AVX2) {
        jpeg_k_band_avx2(j, r0, r1);
        return;
    }
#endif
    jpeg_k_band_dflt(j, r0, r1);
}

/* The Huffman coder's output: 32 bits at a time, stuffed */
typedef struct BitOut {
    uint8_t *buf, *p;
    size_t cap;
    uint64_t acc;
    int bits;                   /* in @acc, under 32 between calls */
} BitOut;

/* the longest an MCU's bytes can be, stuffed */
#define MCU_ROOM        4096

static void bo_room(BitOut *o)
{
    size_t len = o->p - o->buf;

    if (len + MCU_ROOM > o->cap) {
        o->cap = o->cap * 2 + MCU_ROOM;
        o->buf = g_realloc(o->buf, o->cap);
        o->p = o->buf + len;
    }
}

/* the 4 bytes of @w, each 0xff followed by 0 */
static uint8_t *bo_stuff(uint8_t *p, uint32_t w)
{
    for (int i = 24; i >= 0; i -= 8) {
        *p++ = w >> i;
        if ((w >> i & 0xff) == 0xff) {
            *p++ = 0;
        }
    }
    return p;
}

/* @n bits (at most 32) of @v, which has nothing above them */
static inline void bo_put(BitOut *o, uint32_t v, int n)
{
    o->acc = o->acc << n | v;
    o->bits += n;
    if (o->bits >= 32) {
        uint32_t w = o->acc >> (o->bits - 32);

        o->bits -= 32;
        if (!((~w - 0x01010101u) & w & 0x80808080u)) {
            stl_be_p(o->p, w);
            o->p += 4;
        } else {
            o->p = bo_stuff(o->p, w);
        }
    }
}

/* the last bits ones to a byte, and all of it out */
static void bo_align(BitOut *o)
{
    int pad = -o->bits & 7;

    o->acc = o->acc << pad | ((1u << pad) - 1);
    o->bits += pad;
    while (o->bits) {
        uint8_t b = o->acc >> (o->bits - 8);

        *o->p++ = b;
        if (b == 0xff) {
            *o->p++ = 0;
        }
        o->bits -= 8;
    }
}

/* a Huffman code, and then @n bits of @v: the value bits of a coefficient */
static inline void bo_code(BitOut *o, const uint32_t *lut, int sym,
                           int32_t v, int n)
{
    uint32_t e = lut[sym];

    bo_put(o, (e & 0xffff) << n | ((uint32_t)(v + (v >> 31)) & ((1u << n) - 1)),
           (e >> 16) + n);
}

static inline int nbits(int32_t v)
{
    return v ? 32 - clz32(abs(v)) : 0;
}

static uint32_t jpeg_block_out(BitOut *out, const int16_t *q, uint64_t nz,
                               int *pred, const uint32_t *dc,
                               const uint32_t *ac)
{
    BitOut b = *out, *o = &b;
    int32_t d = q[0] - *pred;
    uint32_t symbols = 1;
    int last = 0;

    *pred = q[0];
    bo_code(o, dc, nbits(d), d, nbits(d));
    for (; nz; nz &= nz - 1) {
        int k = ctz64(nz), run = k - last - 1;
        int32_t v = q[zigzag[k]], n = nbits(v);

        for (; run > 15; run -= 16) {
            bo_code(o, ac, 0xf0, 0, 0);
            symbols++;
        }
        bo_code(o, ac, run << 4 | n, v, n);
        symbols++;
        last = k;
    }
    if (last < 63) {
        bo_code(o, ac, 0x00, 0, 0);
        symbols++;
    }
    *out = b;
    return symbols;
}

uint32_t ingenic_avpu_jpeg_job_finish(IngenicAvpuJpegJob *j, GByteArray *out)
{
    uint32_t symbols = 0;
    BitOut o = { };
    int pred[3] = { 0 };
    uint32_t mcu = 0;

    if (j->slow) {
        symbols = jpeg_scan_ref(j->y, j->ystride, j->uv, j->cstride,
                                j->width, j->height,
                                j->t32 ? dct_t32 : dct_el200, j->mul,
                                j->shift, j->huff, j->restart, out);
        g_free(j);
        return symbols;
    }

    o.cap = (size_t)j->mcux * j->mcuy * 40 + MCU_ROOM;
    o.buf = o.p = g_malloc(o.cap);
    for (uint32_t r = 0; r < j->mcuy; r++) {
        for (uint32_t m = 0; m < j->mcux; m++, mcu++) {
            const int16_t *q = j->coef + (size_t)mcu * 6 * 64;
            const uint64_t *nz = j->nz + (size_t)mcu * 6;

            bo_room(&o);
            if (j->restart && mcu && !(mcu % j->restart)) {
                bo_align(&o);
                *o.p++ = 0xff;
                *o.p++ = 0xd0 + (mcu / j->restart - 1) % 8;
                memset(pred, 0, sizeof(pred));
            }
            for (int b = 0; b < 4; b++) {
                if (nz[b] == UINT64_MAX) {
                    bo_code(&o, j->lut[0], 0, 0, 0);
                    bo_code(&o, j->lut[1], 0x00, 0, 0);
                    symbols += 2;
                    continue;
                }
                symbols += jpeg_block_out(&o, q + b * 64, nz[b], &pred[0],
                                          j->lut[0], j->lut[1]);
            }
            for (int c = 0; c < 2; c++) {
                symbols += jpeg_block_out(&o, q + (4 + c) * 64, nz[4 + c],
                                          &pred[1 + c], j->lut[2],
                                          j->lut[3]);
            }
        }
    }
    bo_align(&o);
    g_byte_array_append(out, o.buf, o.p - o.buf);
    g_free(o.buf);
    ingenic_avpu_jpeg_job_free(j);
    return symbols;
}

void ingenic_avpu_jpeg_job_free(IngenicAvpuJpegJob *j)
{
    g_free(j->coef);
    g_free(j->nz);
    g_free(j);
}

static uint32_t jpeg_scan(const uint8_t *y, uint32_t ystride,
                          const uint8_t *uv, uint32_t cstride,
                          uint32_t width, uint32_t height, bool t32,
                          const uint32_t mul[2][64],
                          const uint8_t shift[2][64],
                          const IngenicAvpuJpegHuff huff[4],
                          uint16_t restart, GByteArray *out)
{
    IngenicAvpuJpegJob *j = ingenic_avpu_jpeg_job_new(y, ystride, uv, cstride,
                                                      width, height, t32, mul,
                                                      shift, huff, restart);

    ingenic_avpu_jpeg_job_band(j, 0, 1);
    return ingenic_avpu_jpeg_job_finish(j, out);
}

uint32_t ingenic_avpu_jpeg_scan(const uint8_t *y, uint32_t ystride,
                                const uint8_t *uv, uint32_t cstride,
                                uint32_t width, uint32_t height,
                                const uint32_t mul[2][64],
                                const uint8_t shift[2][64],
                                const IngenicAvpuJpegHuff huff[4],
                                uint16_t restart, GByteArray *out)
{
    return jpeg_scan(y, ystride, uv, cstride, width, height, false, mul,
                     shift, huff, restart, out);
}

uint32_t ingenic_avpu_jpeg_scan_t32(const uint8_t *y, uint32_t ystride,
                                    const uint8_t *uv, uint32_t cstride,
                                    uint32_t width, uint32_t height,
                                    const uint32_t mul[2][64],
                                    const uint8_t shift[2][64],
                                    const IngenicAvpuJpegHuff huff[4],
                                    uint16_t restart, GByteArray *out)
{
    return jpeg_scan(y, ystride, uv, cstride, width, height, true, mul,
                     shift, huff, restart, out);
}

static void put_sof(GByteArray *out, uint32_t width, uint32_t height)
{
    put_be16(out, 0xffc0);
    put_be16(out, 17);
    put_byte(out, 8);
    put_be16(out, height);
    put_be16(out, width);
    put_byte(out, 3);
    g_byte_array_append(out, (const uint8_t *)"\1\x22\0\2\x11\1\3\x11\1", 9);
}

static void put_tables(GByteArray *out, bool pad)
{
    put_dht(out, 0x00, dc_luma, pad);
    put_dht(out, 0x10, ac_luma, pad);
    put_dht(out, 0x01, dc_chroma, pad);
    put_dht(out, 0x11, ac_chroma, pad);
}

static void put_dri(GByteArray *out, uint16_t restart)
{
    put_be16(out, 0xffdd);
    put_be16(out, 4);
    put_be16(out, restart);
}

/* SOS, the scan and EOI */
static uint32_t put_scan(const uint8_t *y, uint32_t ystride,
                         const uint8_t *uv, uint32_t cstride, uint32_t width,
                         uint32_t height, JpegDct *dct,
                         const uint32_t mul[2][64], const uint8_t shift[2][64],
                         uint16_t restart, GByteArray *out)
{
    IngenicAvpuJpegHuff huff[4];
    uint32_t symbols;

    ingenic_avpu_jpeg_annex_k(huff);
    put_be16(out, 0xffda);
    put_be16(out, 12);
    g_byte_array_append(out, (const uint8_t *)"\3\1\0\2\x11\3\x11\0\x3f\0", 10);
    symbols = jpeg_scan(y, ystride, uv, cstride, width, height,
                        dct == dct_t32, mul, shift, huff, restart, out);
    put_be16(out, 0xffd9);
    return symbols;
}

uint32_t ingenic_avpu_jpeg_encode(const uint8_t *y, uint32_t ystride,
                                  const uint8_t *uv, uint32_t cstride,
                                  uint32_t width, uint32_t height,
                                  const uint8_t qt[2][64],
                                  const uint16_t recip[2][64], uint8_t units,
                                  uint16_t xdensity, uint16_t ydensity,
                                  uint16_t restart, GByteArray *out)
{
    uint32_t mul[2][64];
    uint8_t shift[2][64];

    put_be16(out, 0xffd8);
    put_be16(out, 0xffe0);
    put_be16(out, 16);
    g_byte_array_append(out, (const uint8_t *)"JFIF\0\1\1", 7);
    put_byte(out, units);
    put_be16(out, xdensity);
    put_be16(out, ydensity);
    put_be16(out, 0);                       /* no thumbnail */
    put_dqt(out, 0, qt[0], true);
    put_dqt(out, 1, qt[1], true);
    put_sof(out, width, height);
    put_tables(out, true);
    if (restart) {
        pad_to(out, 0, 8, true);
        put_dri(out, restart);
    } else {
        pad_to(out, 14, 8, true);
    }
    /* libimp's round(65536 / q) - 1, plus one, rounded half up */
    for (int t = 0; t < 2; t++) {
        for (int k = 0; k < 64; k++) {
            mul[t][k] = recip[t][k] + 1;
            shift[t][k] = 19;
        }
    }
    return put_scan(y, ystride, uv, cstride, width, height, dct_el200, mul,
                    shift, restart, out);
}

uint32_t ingenic_avpu_jpeg_encode_bare(const uint8_t *y, uint32_t ystride,
                                       const uint8_t *uv, uint32_t cstride,
                                       uint32_t width, uint32_t height,
                                       const uint8_t qt[2][64],
                                       const uint32_t mul[2][64],
                                       uint16_t restart, GByteArray *out)
{
    uint8_t shift[2][64];

    memset(shift, 16, sizeof(shift));
    put_be16(out, 0xffd8);
    put_dqt(out, 0, qt[0], false);
    put_dqt(out, 1, qt[1], false);
    put_dqt(out, 1, qt[1], false);
    if (restart) {
        put_dri(out, restart);
    }
    put_sof(out, width, height);
    put_tables(out, false);
    return put_scan(y, ystride, uv, cstride, width, height, dct_t32, mul,
                    shift, restart, out);
}
