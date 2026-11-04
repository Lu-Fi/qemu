/*
 * QTest testcase for the T30's and T23's HELIX decoding JPEG (the JPGC
 * alone)
 *
 * libimp's list for a baseline 4:2:0 scan (Annex K tables, as the JPGC
 * holds them for decoding), over scans of LCG coefficients: random blocks
 * with quantizers that clamp, restarts, DC steps onto the prediction's
 * 11-bit edge, a truncated scan. Each case's MCU count, tile counts,
 * bitstream address and tiles' CRC32s are a T30X's for the same scan, as
 * libimp decoded it, and a T23N's where it ran them. Then the JPGC's
 * registers as a T30X reads them, and scans that end early or go wrong
 * (sampling other than 4:2:0, restarts, data past NMCU, runs onto and past
 * 64, an index past HUFE) as a T23N's JPGC ran them from its own list.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define SCH_STAT        0x00034
#define JPGC_TRIG       0xe0000
#define JPGC_GLBI       0xe0004
#define JPGC_STAT       0xe0008
#define JPGC_BSA        0xe000c
#define JPGC_P0A        0xe0010
#define JPGC_P1A        0xe0014
#define JPGC_E20        0xe0020
#define JPGC_GLBI_RD    0xe0024
#define JPGC_NMCU       0xe0028
#define JPGC_NRSM       0xe002c
#define JPGC_CCI        0xe0030
#define JPGC_E60        0xe0060
#define JPGC_MCUS       0xe0064
#define JPGC_HUFB       0xe1200
#define JPGC_HUFM       0xe1300
#define JPGC_QMEM       0xe1400
#define JPGC_HUFE       0xe1800

#define LIST            0x00800000
#define BS              0x00900000
#define TILES_Y         0x00a00000
#define TILES_C         0x00b00000

#define CL_VLD          (1u << 31)
#define CL_TERM         (1u << 30)

static const uint8_t zz[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

/* Annex K: DC luma, AC luma, DC chroma, AC chroma (16 counts, symbols) */
static const uint8_t ak_dc_l[] = {
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t ak_dc_c[] = {
    0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};
static const uint8_t ak_ac_l[] = {
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
static const uint8_t ak_ac_c[] = {
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

typedef struct {
    uint8_t *p;
    uint32_t len;
    uint32_t acc;
    int n;
} Bw;

static void bw_put(Bw *w, uint32_t v, int n)
{
    while (n--) {
        w->acc = w->acc << 1 | ((v >> n) & 1);
        if (++w->n == 8) {
            w->p[w->len++] = w->acc;
            if ((w->acc & 0xff) == 0xff) {
                w->p[w->len++] = 0;
            }
            w->acc = 0;
            w->n = 0;
        }
    }
}

static void bw_flush(Bw *w)
{
    if (w->n) {
        bw_put(w, (1u << (8 - w->n)) - 1, 8 - w->n);
    }
}

/* the code of @sym in table @t (counts then symbols) */
static void bw_sym(Bw *w, const uint8_t *t, int sym)
{
    uint32_t code = 0;
    int k = 16;

    for (int l = 1; l <= 16; l++) {
        for (int i = 0; i < t[l - 1]; i++, k++, code++) {
            if (t[k] == sym) {
                bw_put(w, code, l);
                return;
            }
        }
        code <<= 1;
    }
}

static int mag(int v)
{
    int n = 0;

    for (v = v < 0 ? -v : v; v; v >>= 1) {
        n++;
    }
    return n;
}

static void bw_val(Bw *w, int v, int m)
{
    bw_put(w, v >= 0 ? v : v - 1 + (1 << m), m);
}

/* one block, natural order, the DC as itself (its difference coded) */
static void bw_block(Bw *w, const int *q, int *pred, const uint8_t *dc,
                     const uint8_t *ac)
{
    int d = q[0] - *pred, run = 0;

    *pred = q[0];
    bw_sym(w, dc, mag(d));
    bw_val(w, d, mag(d));
    for (int k = 1; k < 64; k++) {
        int v = q[zz[k]];

        if (!v) {
            run++;
            continue;
        }
        while (run > 15) {
            bw_sym(w, ac, 0xf0);
            run -= 16;
        }
        bw_sym(w, ac, run << 4 | mag(v));
        bw_val(w, v, mag(v));
        run = 0;
    }
    if (run) {
        bw_sym(w, ac, 0);
    }
}

typedef struct {
    uint32_t w, h;
    uint32_t seed;
    int dc, nz, amp, qmax;      /* random blocks */
    uint16_t dri;
    int edge;                   /* DC edge events instead */
    uint32_t cut;               /* scan bytes kept, 0 for all */
} Gen;

static uint32_t lcg(uint32_t *s)
{
    *s = *s * 1103515245 + 12345;
    return *s >> 16;
}

/* the edge events: a start, then a step onto +-1024 and around it */
static const int16_t edge_ev[][2] = {
    { 1023, 1 }, { 1022, 2 }, { 1, 1023 }, { 0, 1024 }, { 0, -1024 },
    { -1, -1023 }, { -1023, -1 }, { -24, -1000 }, { -1, 1025 },
    { -1023, 2047 }, { 1023, -2047 }, { 1, -1025 }, { 1021, 2 },
    { 1023, 3 }, { -1023, -3 }, { 0, 1025 }, { 0, -1025 }, { 970, 97 },
    { 0, 2047 }, { 0, -2047 }, { 600, 424 }, { -1006, -18 },
};

/* qtab: 2x64 natural order; returns the scan's length */
static uint32_t gen_scan(const Gen *g, uint8_t *out, uint8_t qtab[2][64])
{
    uint32_t s = g->seed, mcus = (g->w / 16) * (g->h / 16);
    int pred[3] = { 0 };
    Bw w = { .p = out };

    for (int t = 0; t < 2; t++) {
        for (int k = 0; k < 64; k++) {
            qtab[t][k] = g->edge ? 8 : 1 + lcg(&s) % g->qmax;
        }
    }
    for (uint32_t m = 0; m < mcus; m++) {
        if (g->dri && m && m % g->dri == 0) {
            bw_flush(&w);
            out[w.len++] = 0xff;
            out[w.len++] = 0xd0 + (m / g->dri - 1) % 8;
            pred[0] = pred[1] = pred[2] = 0;
        }
        for (int b = 0; b < 6; b++) {
            int q[64] = { 0 }, c = b < 4 ? 0 : b - 3;

            if (g->edge) {
                const int16_t *e = edge_ev[(m / 2) % ARRAY_SIZE(edge_ev)];
                int read = m % 2 ? 1000 : -1000;

                /* start, step, then back to a visible value, held */
                q[0] = b == 0 ? e[0] : b == 1 ? e[0] + e[1]
                       : b < 4 ? e[0] + e[1] + read : 0;
            } else {
                q[0] = (int)(lcg(&s) % (2 * g->dc + 1)) - g->dc;
                for (int i = 0; i < g->nz; i++) {
                    int k = 1 + lcg(&s) % 63;

                    q[k] = (int)(lcg(&s) % (2 * g->amp + 1)) - g->amp;
                }
            }
            bw_block(&w, q, &pred[c], c ? ak_dc_c : ak_dc_l,
                     c ? ak_ac_c : ak_ac_l);
        }
    }
    bw_flush(&w);
    if (g->cut && g->cut < w.len) {
        w.len = g->cut;
        while (w.len && out[w.len - 1] == 0xff) {
            w.len--;
        }
    }
    out[w.len++] = 0xff;
    out[w.len++] = 0xd9;
    return w.len;
}

typedef struct {
    const char *machine;
    uint32_t helix, taskrg, e60;
} Part;

static const Part t30 = { "ingenic-t30", 0x13200000, 0x10008, 0x001a0600 };
static const Part t23 = { "ingenic-t23", 0x13100000, 0x00084, 0x00250600 };

typedef struct {
    Gen g;
    uint32_t mcus, ytiles, ctiles, bsa, ycrc, ccrc;
    bool t23;                   /* a T23N ran it too */
} Case;

static const Case cases[] = {
    { { .w = 256, .h = 32, .seed = 1, .dc = 400, .nz = 6, .amp = 40,
        .qmax = 4 }, 32, 32, 32, 3840, 0x979472c0, 0xa98422bb, true },
    { { .w = 256, .h = 64, .seed = 2, .dc = 200, .nz = 20, .amp = 25,
        .qmax = 64, .dri = 3 }, 64, 64, 64, 13952, 0x2d7a1cb8, 0xe9e5f73a },
    { { .w = 320, .h = 48, .seed = 3, .dc = 1000, .nz = 3, .amp = 600,
        .qmax = 255 }, 60, 60, 60, 5632, 0xeea137bb, 0x85661c83, true },
    { { .w = 256, .h = 48, .edge = 1, .dri = 1 }, 48, 48, 48, 768,
      0xf17d37f0, 0x1587ed98, true },
    { { .w = 256, .h = 64, .seed = 5, .dc = 300, .nz = 8, .amp = 30,
        .qmax = 8, .cut = 3000 }, 23, 24, 23, 3200, 0x0b298040, 0x05819752 },
};

typedef struct {
    uint32_t v[2048];
    uint32_t n;
} List;

static void put(List *l, uint32_t reg, uint32_t val)
{
    l->v[l->n++] = cpu_to_le32(val);
    l->v[l->n++] = cpu_to_le32(CL_VLD | reg);
}

/*
 * Table @t (AC0, DC0, AC1, DC1) as libimp loads it for decoding: the
 * canonical minima by length into HUFM (1-8 packed at their widths, 9-16
 * by their low bytes), symbol index less minimum into HUFB, the symbols
 * into HUFE from @off (DC0's and DC1's sharing words, by nibble)
 */
static void put_table(List *l, uint32_t *hufe, int t, const uint8_t *spec,
                      int off)
{
    uint32_t min[16], code = 0, w[4] = { 0 };
    uint64_t lo = 0;
    int idx = 0;

    for (int len = 1; len <= 16; len++) {
        min[len - 1] = code;
        put(l, JPGC_HUFB + 64 * t + 4 * (len - 1),
            (off + idx - code) & 0x1ff);
        for (int i = 0; i < spec[len - 1]; i++, idx++) {
            uint8_t s = spec[16 + idx];

            hufe[off + idx] |= t & 1 ? (s & 0xf) << (t == 3 ? 4 : 0) : s;
        }
        code = (code + spec[len - 1]) << 1;
    }
    for (int len = 1; len <= 8; len++) {
        lo = lo << len | (min[len - 1] & ((1u << len) - 1));
    }
    w[3] = lo >> 32;
    w[2] = lo;
    for (int len = 9; len <= 16; len++) {
        int sh = 8 * (3 - (len - 9) % 4);

        w[len <= 12 ? 1 : 0] |= (min[len - 1] & 0xff) << sh;
    }
    for (int i = 0; i < 4; i++) {
        put(l, JPGC_HUFM + 16 * t + 4 * i, w[i]);
    }
}

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

typedef struct {
    const Part *p;
    const Case *c;
} Run;

static void test_case(const void *data)
{
    const Part *p = ((const Run *)data)->p;
    const Case *c = ((const Run *)data)->c;
    QTestState *qts = qtest_initf("-machine %s", p->machine);
    uint32_t helix = p->helix;
    uint32_t mcus = (c->g.w / 16) * (c->g.h / 16), hufe[336] = { 0 };
    g_autofree uint8_t *scan = g_malloc(1 << 20);
    g_autofree uint8_t *y = g_malloc(256 * c->ytiles);
    g_autofree uint8_t *cc = g_malloc(128 * c->ctiles);
    g_autofree List *l = g_new0(List, 1);
    uint8_t qtab[2][64];
    uint32_t len = gen_scan(&c->g, scan, qtab), stat = 0;

    qtest_memwrite(qts, BS, scan, len);
    put(l, 0xc0000, 0);
    put(l, JPGC_GLBI, 1);
    put_table(l, hufe, 0, ak_ac_l, 0);
    put_table(l, hufe, 1, ak_dc_l, 162);
    put_table(l, hufe, 2, ak_ac_c, 174);
    put_table(l, hufe, 3, ak_dc_c, 162);
    for (int i = 0; i < 336; i++) {
        put(l, JPGC_HUFE + 4 * i, hufe[i]);
    }
    for (int i = 0; i < 256; i++) {
        put(l, JPGC_QMEM + 4 * i, i < 128 ? qtab[i / 64][zz[i % 64]] : 0);
    }
    put(l, JPGC_STAT, 0);
    put(l, JPGC_BSA, BS);
    put(l, JPGC_P0A, TILES_Y);
    put(l, JPGC_P1A, TILES_C);
    put(l, JPGC_NMCU, mcus - 1);
    put(l, JPGC_NRSM, c->g.dri ? c->g.dri - 1 : 0);
    put(l, JPGC_CCI, 0x30);
    put(l, JPGC_CCI + 4, 0x07);
    put(l, JPGC_CCI + 8, 0x07);
    put(l, JPGC_CCI + 12, 0xf0);
    put(l, JPGC_GLBI, (c->g.dri ? 0x4 : 0) | 0xa0029);
    put(l, JPGC_TRIG, 7);
    l->v[l->n - 1] |= cpu_to_le32(CL_TERM);
    qtest_memwrite(qts, LIST, l->v, l->n * 4);

    qtest_writel(qts, helix + p->taskrg, LIST | 1);
    for (int i = 0; i < 20000 && !(stat & 1); i++) {
        g_usleep(500);
        stat = qtest_readl(qts, helix + SCH_STAT);
    }
    g_assert_cmphex(stat, ==, 0x11);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_STAT), ==, 0x80000000);
    g_assert_cmpuint(qtest_readl(qts, helix + JPGC_MCUS), ==, c->mcus);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_BSA), ==, BS + c->bsa);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_P0A), ==,
                    TILES_Y + 256 * c->ytiles);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_P1A), ==,
                    TILES_C + 128 * c->ctiles);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_TRIG), ==, 0);
    qtest_memread(qts, TILES_Y, y, 256 * c->ytiles);
    qtest_memread(qts, TILES_C, cc, 128 * c->ctiles);
    g_assert_cmphex(crc32_le(y, 256 * c->ytiles), ==, c->ycrc);
    g_assert_cmphex(crc32_le(cc, 128 * c->ctiles), ==, c->ccrc);
    /* the driver clears the codec's end, and with it 0x34's */
    qtest_writel(qts, helix + JPGC_STAT, 0);
    g_assert_cmphex(qtest_readl(qts, helix + SCH_STAT), ==, 0);
    qtest_quit(qts);
}

/* What the JPGC's registers keep, and what reads back on its own */
static void test_regs(const void *data)
{
    const Part *p = data;
    QTestState *qts = qtest_initf("-machine %s", p->machine);
    uint32_t helix = p->helix;
    static const struct {
        uint32_t reg, ones;
    } masks[] = {
        { JPGC_STAT, 0xc0ffffff }, { JPGC_BSA, 0xffffffff },
        { JPGC_P0A, 0xfffffff8 }, { JPGC_P1A, 0xfffffff8 },
        { JPGC_NMCU, 0x03ffffff }, { JPGC_NRSM, 0x0000ffff },
        { JPGC_CCI, 0xff }, { JPGC_CCI + 12, 0xff },
    };

    qtest_writel(qts, helix + JPGC_E20, 0);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_E20), ==, 0xf0);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_E60), ==, p->e60);
    qtest_writel(qts, helix + JPGC_GLBI, 0xa0029);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI), ==, 0xa0029);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI_RD), ==, 0xa);
    if (p != &t30) {
        /* what a T23N showed; the rest is a T30X's */
        qtest_quit(qts);
        return;
    }
    for (int i = 0; i < ARRAY_SIZE(masks); i++) {
        qtest_writel(qts, helix + masks[i].reg, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, helix + masks[i].reg), ==,
                        masks[i].ones);
    }
    qtest_writel(qts, helix + JPGC_MCUS, 0x1234);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_MCUS), ==, 0);
    /* GLBI's 5:4 and 3:2 read swapped; bit 6 with EN clears them */
    qtest_writel(qts, helix + JPGC_GLBI, 0xa002d);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI), ==, 0xa0039);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI_RD), ==, 0xe);
    qtest_writel(qts, helix + JPGC_GLBI, 0x09);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI), ==, 0x21);
    qtest_writel(qts, helix + JPGC_GLBI, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI), ==, 0xfffff183);
    qtest_writel(qts, helix + JPGC_GLBI, 0x7d);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_GLBI), ==, 0x01);
    qtest_quit(qts);
}

/*
 * Scans that end early or go wrong, as a T23N decoded them: every table
 * Annex K's luma, every quantizer 8, and blocks of DC +1 and an EOB (one
 * byte, 01011010), so each block reads back 128 + its component's count;
 * one block may be other bits (@special), and restart markers stand every
 * @rst blocks.
 */
typedef struct {
    uint32_t nmcu;
    int ncol, dri;
    uint8_t cci[4];
    int blocks, at, rst;
    const char *special;
    uint32_t stat, mcus, bsa, ytiles, ctiles, ycrc, ccrc;   /* crc 0: not */
} BadCase;

static const BadCase bad_cases[] = {
    /* 4:2:2 as libimp sets it up: two luma blocks an MCU */
    { 4, 2, 0, { 0x10, 0x07, 0x07, 0xf0 },
      120, 0, 0, "",
      0x80, 5, 256, 5, 5, 0x23cce387, 0x1fa98d45 },
    /* grayscale: an MCU of one block */
    { 4, 0, 0, { 0x00, 0x07, 0x07, 0xf0 },
      120, 0, 0, "",
      0x80, 5, 256, 5, 5, 0xee0286f3, 0x53014c8e },
    /* four components, seven blocks an MCU */
    { 4, 3, 0, { 0x30, 0x07, 0x07, 0x07 },
      120, 0, 0, "",
      0x80, 5, 256, 5, 5, 0x44f8013f, 0x7180ef15 },
    /* the data end with tile NMCU */
    { 4, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      24, 0, 0, "",
      0x11, 4, 256, 4, 4, 0x59079c1c, 0x16bb62be },
    /* the data end with tile NMCU + 1 */
    { 4, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      30, 0, 0, "",
      0x91, 5, 256, 5, 5, 0xc0a3b92c, 0xb70311b9 },
    /* a block past tile NMCU + 1 */
    { 4, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      31, 0, 0, "",
      0x80, 5, 256, 5, 5, 0xc0a3b92c, 0xb70311b9 },
    /* a restart marker without restarts */
    { 4, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      36, 0, 24, "",
      0x80, 5, 256, 5, 5, 0xc0a3b92c, 0xb70311b9 },
    /* a restart every MCU of three blocks */
    { 4, 2, 1, { 0x00, 0x07, 0x07, 0xf0 },
      24, 0, 3, "",
      0x11, 4, 256, 4, 4, 0x61e99a13, 0x937b72a8 },
    /* a run onto place 64 */
    { 8, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      48, 7, 0,
      "0101111111111111010111111111111110101111111111111101011111111111"
      "111010111010",
      0x80, 9, 256, 9, 9, 0xa54a654c, 0x69ea4eb2 },
    /* a ZRL onto place 64 */
    { 8, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      48, 7, 0,
      "0101111111110011111111100111111111001111111110011010",
      0x80, 9, 256, 9, 9, 0x733f1a55, 0x69ea4eb2 },
    /* a run onto place 64, further in */
    { 100, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      420, 94, 0,
      "0101111111111111010111111111111110101111111111111101011111111111"
      "111010111010",
      0x80, 101, 384, 101, 101, 0x1a6fc132, 0x7880f178 },
    /* a run past 64, into the next block */
    { 8, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      48, 7, 0,
      "0101111111111111010111111111111110101111111111111101011001111111"
      "1111110110111010",
      0x11, 8, 256, 8, 8, 0x067d766e, 0x95fd8b20 },
    /* a ZRL past 64, from a Cb block */
    { 8, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      48, 10, 0,
      "0101111111110011111111100111111111001001111111110011010",
      0x11, 8, 256, 8, 8, 0x69074c6c, 0xa162b524 },
    /* an AC1 symbol index past HUFE's end */
    { 8, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      48, 10, 0, "01011111111111111111",
      0x11, 7, 256, 7, 7, 0x02ac2e8b, 0xdb1bf714 },
    /* the data end with a block */
    { 100, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      17, 0, 0, "",
      0x11, 2, 256, 3, 2, 0x081e4f31, 0xf3e22bc7 },
    /* the data end inside a block's AC */
    { 100, 2, 0, { 0x30, 0x07, 0x07, 0xf0 },
      10, 9, 0,
      "0101001001001001001001001001001001001001001001001001001001001001",
      0x11, 1, 256, 2, 1, 0x0, 0x0 },
};

static void bw_bits(Bw *w, const char *bits)
{
    for (; *bits; bits++) {
        bw_put(w, *bits == '1', 1);
    }
}

static void test_bad(const void *data)
{
    const BadCase *c = data;
    QTestState *qts = qtest_initf("-machine %s", t23.machine);
    uint32_t helix = t23.helix, hufe[336] = { 0 }, stat = 0;
    g_autofree uint8_t *scan = g_malloc(4096);
    g_autofree uint8_t *y = g_malloc(256 * c->ytiles);
    g_autofree uint8_t *cc = g_malloc(128 * c->ctiles);
    g_autofree List *l = g_new0(List, 1);
    Bw w = { .p = scan };

    for (int i = 0; i < c->blocks; i++) {
        if (c->rst && i && i % c->rst == 0) {
            bw_flush(&w);
            scan[w.len++] = 0xff;
            scan[w.len++] = 0xd0 + (i / c->rst - 1) % 8;
        }
        bw_bits(&w, *c->special && i == c->at ? c->special : "01011010");
    }
    bw_flush(&w);
    scan[w.len++] = 0xff;
    scan[w.len++] = 0xd9;
    qtest_memwrite(qts, BS, scan, w.len);

    put(l, 0xc0000, 0);
    put(l, JPGC_GLBI, 1);
    put_table(l, hufe, 0, ak_ac_l, 0);
    put_table(l, hufe, 1, ak_dc_l, 162);
    put_table(l, hufe, 2, ak_ac_l, 174);
    put_table(l, hufe, 3, ak_dc_l, 162);
    for (int i = 0; i < 336; i++) {
        put(l, JPGC_HUFE + 4 * i, hufe[i]);
    }
    for (int i = 0; i < 256; i++) {
        put(l, JPGC_QMEM + 4 * i, i < 128 ? 8 : 0);
    }
    put(l, JPGC_STAT, 0);
    put(l, JPGC_BSA, BS);
    put(l, JPGC_P0A, TILES_Y);
    put(l, JPGC_P1A, TILES_C);
    put(l, JPGC_NMCU, c->nmcu - 1);
    put(l, JPGC_NRSM, c->dri ? c->dri - 1 : 0);
    for (int i = 0; i < 4; i++) {
        put(l, JPGC_CCI + 4 * i, c->cci[i]);
    }
    put(l, JPGC_GLBI, (c->dri ? 0x4 : 0) | 0xa0009 | c->ncol << 4);
    put(l, JPGC_TRIG, 7);
    l->v[l->n - 1] |= cpu_to_le32(CL_TERM);
    qtest_memwrite(qts, LIST, l->v, l->n * 4);

    qtest_writel(qts, helix + t23.taskrg, LIST | 1);
    for (int i = 0; i < 20000 && !(stat & 0x81); i++) {
        g_usleep(500);
        stat = qtest_readl(qts, helix + SCH_STAT);
    }
    /* the end, the error, or both */
    g_assert_cmphex(stat, ==, c->stat);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_STAT), ==,
                    (c->stat & 0x01 ? 0x80000000 : 0) |
                    (c->stat & 0x80 ? 0x40000000 : 0));
    g_assert_cmpuint(qtest_readl(qts, helix + JPGC_MCUS), ==, c->mcus);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_BSA), ==, BS + c->bsa);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_P0A), ==,
                    TILES_Y + 256 * c->ytiles);
    g_assert_cmphex(qtest_readl(qts, helix + JPGC_P1A), ==,
                    TILES_C + 128 * c->ctiles);
    if (c->ycrc) {
        qtest_memread(qts, TILES_Y, y, 256 * c->ytiles);
        qtest_memread(qts, TILES_C, cc, 128 * c->ctiles);
        g_assert_cmphex(crc32_le(y, 256 * c->ytiles), ==, c->ycrc);
        g_assert_cmphex(crc32_le(cc, 128 * c->ctiles), ==, c->ccrc);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static Run runs[2 * ARRAY_SIZE(cases)];
    int n = 0;

    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        for (int t = 0; t < 2; t++) {
            g_autofree char *path = NULL;

            if (t && !cases[i].t23) {
                continue;
            }
            runs[n] = (Run) { t ? &t23 : &t30, &cases[i] };
            path = g_strdup_printf("ingenic/%s-jpeg/%d", t ? "t23" : "t30", i);
            qtest_add_data_func(path, &runs[n++], test_case);
        }
    }
    for (int i = 0; i < ARRAY_SIZE(bad_cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/t23-jpeg/bad/%d", i);

        qtest_add_data_func(path, &bad_cases[i], test_bad);
    }
    qtest_add_data_func("ingenic/t30-jpeg/regs", &t30, test_regs);
    qtest_add_data_func("ingenic/t23-jpeg/regs", &t23, test_regs);
    return g_test_run();
}
