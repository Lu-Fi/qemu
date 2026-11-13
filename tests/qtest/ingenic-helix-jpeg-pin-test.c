/*
 * QTest testcase pinning the HELIX JPEG encoder's bytes
 *
 * One JPEG job at each of several sizes up to 3840x2160, of two pictures
 * (LCG noise, and a smooth gradient with a little noise that has long
 * zero runs and ZRL codes), on the A1's engine (the EL200's DCT) and the
 * T41's (the T32's DCT), libimp's quality 75 quantizers on both. The
 * scan's length and CRC32 are what the model gave when the test was
 * written: a regression pin for speeding the encoder up, not a hardware
 * measurement. With HELIX_PIN_PRINT set, the table is printed instead.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define CL_VLD          (1u << 31)
#define CL_TERM         (1u << 30)

#define SRC_Y           0x01000000
#define SRC_C           0x02000000
#define LIST            0x02800000
#define BS              0x03000000

typedef struct {
    const char *machine;
    uint32_t jpeg, taskrg, efe, jpgc, list_base;
} Part;

static const Part parts[] = {
    { "ingenic-a1", 0x13200000, 0x10008, 0x40000, 0xe0000, 0 },
    { "ingenic-t41", 0x13280000, 0x00084, 0x20000, 0x30000, 0x80000 },
};

#define JPGC_STAT_OFF   0x08
#define JPGC_QMEM_OFF   0x1400
#define JPGC_HUFE_OFF   0x1800

typedef struct {
    int part;
    bool smooth;
    uint32_t w, h, len, crc;
} Case;

static const Case cases[] = {
    { 0, false, 32, 16, 409, 0x13edf38a },
    { 0, false, 640, 368, 188081, 0xa780018d },
    { 0, false, 1920, 1088, 1666574, 0x6399d78a },
    { 0, false, 3840, 2160, 6621656, 0x07d0333e },
    { 0, true, 32, 16, 37, 0x7edf9d63 },
    { 0, true, 640, 368, 12823, 0x13ad2b23 },
    { 0, true, 1920, 1088, 109481, 0x01fda2a4 },
    { 0, true, 3840, 2160, 431555, 0x2d743977 },
    { 1, false, 32, 16, 410, 0x65ac3bec },
    { 1, false, 640, 368, 188150, 0x1093c3d7 },
    { 1, false, 1920, 1088, 1667048, 0x4909a5ed },
    { 1, false, 3840, 2160, 6623867, 0x6669dbf2 },
    { 1, true, 32, 16, 37, 0x869478e7 },
    { 1, true, 640, 368, 12833, 0x1dd9d992 },
    { 1, true, 1920, 1088, 109879, 0x7738291d },
    { 1, true, 3840, 2160, 433256, 0x8c516fea },
};

/* libimp's quantizers for quality 75, luma then chroma, zigzag */
static const uint16_t qmem[128] = {
    0x100, 0x155, 0x155, 0xa49, 0x155, 0xb33, 0x100, 0xa49,
    0xa49, 0xa49, 0x9c7, 0x9c7, 0x100, 0x0cd, 0x955, 0x8cd,
    0x93b, 0x955, 0x12e9, 0x12e9, 0x955, 0x251f, 0x11c7, 0x11af,
    0x911, 0x8cd, 0x1a35, 0x113b, 0x2421, 0x1111, 0x1a35, 0x113b,
    0x1a49, 0x1a49, 0x880, 0x19c7, 0x10b2, 0x10d2, 0x880, 0x10f1,
    0x10ba, 0x10ea, 0x1a49, 0x1a49, 0x10cd, 0x1095, 0x231f, 0x10ba,
    0x1955, 0x229d, 0x193b, 0x193b, 0x193b, 0x2421, 0x10d2, 0x223f,
    0x2219, 0x2249, 0x10a4, 0x1911, 0x10b2, 0x2d05, 0x193b, 0x10a4,
    0x9c7, 0x9c7, 0x9c7, 0x955, 0x12e9, 0x955, 0x1155, 0x93b,
    0x93b, 0x1155, 0x10a4, 0x23e1, 0x1a49, 0x23e1, 0x10a4, 0x10a4,
    0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4,
    0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4,
    0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4,
    0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4,
    0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4,
    0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4, 0x10a4,
};

/* JPEG's Annex K tables: code counts by length, then the symbols */
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


typedef struct {
    uint32_t v[2048];
    uint32_t n;
} List;

static void put(List *l, const Part *p, uint32_t reg, uint32_t val)
{
    l->v[l->n++] = cpu_to_le32(val);
    l->v[l->n++] = cpu_to_le32(CL_VLD | (p->list_base + reg));
}

static void put_huff(List *l, const Part *p, const uint8_t *spec, bool ac,
                     uint32_t base)
{
    uint32_t code = 0;
    const uint8_t *sym = spec + 16;

    for (int len = 1; len <= 16; len++, code <<= 1) {
        for (int i = 0; i < spec[len - 1]; i++, code++, sym++) {
            uint32_t s = *sym, k;

            if (!ac) {
                k = s;
            } else if (s == 0x00 || s == 0xf0) {
                k = s ? 161 : 160;
            } else {
                k = (s >> 4) * 10 + (s & 15) - 1;
            }
            put(l, p, p->jpgc + JPGC_HUFE_OFF + 4 * (base + k),
                (len - 1) << 8 | (code & 0xff));
        }
    }
}

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    static uint32_t table[256];
    uint32_t crc = ~0u;

    if (!table[1]) {
        for (int i = 0; i < 256; i++) {
            uint32_t c = i;

            for (int k = 0; k < 8; k++) {
                c = c >> 1 ^ (c & 1 ? 0xedb88320 : 0);
            }
            table[i] = c;
        }
    }
    while (n--) {
        crc = crc >> 8 ^ table[(crc ^ *p++) & 0xff];
    }
    return ~crc;
}

static void draw(const Case *c, uint8_t *pic)
{
    size_t ny = (size_t)c->w * c->h, n = ny * 3 / 2;
    uint32_t seed = 12345;

    for (size_t i = 0; i < n; i++) {
        seed = seed * 1103515245 + 12345;
        if (!c->smooth) {
            pic[i] = seed >> 16;
        } else if (i < ny) {
            uint32_t x = i % c->w, y = i / c->w;

            pic[i] = (x * 255 / c->w + y * 3 / 2) / 2 + ((seed >> 16) & 3) +
                     (((x >> 5) ^ (y >> 5)) & 1) * 40;
        } else {
            pic[i] = 100 + ((i - ny) % c->w) / 64 + ((seed >> 16) & 1);
        }
    }
}

static void test_case(const void *data)
{
    const Case *c = data;
    const Part *p = &parts[c->part];
    size_t ny = (size_t)c->w * c->h, n = ny * 3 / 2;
    g_autofree uint8_t *pic = g_malloc(n);
    g_autofree uint8_t *out = NULL;
    g_autofree List *l = g_new0(List, 1);
    QTestState *qts = qtest_initf("-machine %s", p->machine);
    uint32_t stat = 0, len;

    draw(c, pic);
    qtest_memwrite(qts, SRC_Y, pic, ny);
    qtest_memwrite(qts, SRC_C, pic + ny, n - ny);
    for (int i = 0; i < 128; i++) {
        put(l, p, p->jpgc + JPGC_QMEM_OFF + 4 * i, qmem[i]);
    }
    put_huff(l, p, ac_luma, true, 0);
    put_huff(l, p, ac_chroma, true, 176);
    put_huff(l, p, dc_luma, false, 352);
    put_huff(l, p, dc_chroma, false, 368);
    put(l, p, p->efe + 0x04, (c->h / 16 - 1) << 16 | (c->w / 16 - 1));
    put(l, p, p->efe + 0x10, SRC_Y);
    put(l, p, p->efe + 0x14, SRC_C);
    put(l, p, p->efe + 0x38, c->w << 16 | c->w);
    put(l, p, p->jpgc + 0x04, 1);
    put(l, p, p->jpgc + 0x0c, BS);
    put(l, p, p->jpgc + 0x28, (c->w / 16) * (c->h / 16) - 1);
    put(l, p, p->efe, 0x400b);
    l->v[l->n - 1] |= cpu_to_le32(CL_TERM);
    qtest_memwrite(qts, LIST, l->v, l->n * 4);

    qtest_writel(qts, p->jpeg + p->taskrg, LIST | 1);
    for (int i = 0; i < 200000 && !(stat & 0x80000000u); i++) {
        g_usleep(500);
        stat = qtest_readl(qts, p->jpeg + p->jpgc + JPGC_STAT_OFF);
    }
    g_assert_true(stat & 0x80000000u);
    len = stat & 0xffffff;
    out = g_malloc(len);
    qtest_memread(qts, BS, out, len);
    if (getenv("HELIX_PIN_PRINT")) {
        printf("    { %d, %s, %u, %u, %u, 0x%08x },\n", c->part,
               c->smooth ? "true" : "false", c->w, c->h, len,
               crc32_le(out, len));
    } else {
        g_assert_cmpuint(len, ==, c->len);
        g_assert_cmphex(crc32_le(out, len), ==, c->crc);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        const Case *c = &cases[i];
        g_autofree char *path = g_strdup_printf("ingenic/helix-jpeg-pin/"
                                                "%s/%s-%ux%u",
                                                parts[c->part].machine,
                                                c->smooth ? "smooth" : "noise",
                                                c->w, c->h);

        qtest_add_data_func(path, c, test_case);
    }
    return g_test_run();
}
