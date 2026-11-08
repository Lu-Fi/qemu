/*
 * QTest testcase for the T41's JPEG engine (HELIX's JPEG path)
 *
 * The command list libimp gives the engine for quality 75, its registers
 * in the T41's unit numbering (the EFE unit 2, JPGC 3, indexed 0x80000
 * above their offsets), over NV12 pictures of LCG bytes; each scan's
 * length and CRC32 are a T41NQ's for the same list and picture, from 32x16
 * to 640x368. Then the status the job leaves. With "capture" set, the scan,
 * the picture and the job's settings as files. The engine is linked to the
 * ISP, which draws the pictures it takes over IVDC.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define JPEG            0x13280000
#define SCH_STAT        0x00034
#define VDMA_TASKRG     0x00084
#define EFE             0x20000
#define JPGC            0x30000
#define JPGC_STAT       (JPGC + 0x08)
#define JPGC_QMEM       (JPGC + 0x1400)
#define JPGC_HUFE       (JPGC + 0x1800)

#define SRC_Y           0x00800000
#define SRC_C           0x00a00000
#define LIST            0x00b00000
#define BS              0x00c00000

#define CL_VLD          (1u << 31)
#define CL_TERM         (1u << 30)

typedef struct {
    uint32_t w, h, len, crc;
} Case;

static const Case cases[] = {
    { 32, 16, 410, 0x65ac3bec },
    { 64, 32, 1622, 0xf06085af },
    { 128, 64, 6512, 0xbfab3da0 },
    { 320, 240, 61309, 0x037c55ee },
    { 640, 368, 188150, 0x1093c3d7 },
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

static void put(List *l, uint32_t reg, uint32_t val)
{
    l->v[l->n++] = cpu_to_le32(val);
    l->v[l->n++] = cpu_to_le32(CL_VLD | (0x80000 + reg));
}

/*
 * A table's codes into HUFE as the engine holds them, (length - 1) << 8 |
 * the code's low byte: AC at run * 10 + size - 1 (EOB 160, ZRL 161) from
 * base, DC by category from base
 */
static void put_huff(List *l, const uint8_t *spec, bool ac, uint32_t base)
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
            put(l, JPGC_HUFE + 4 * (base + k), (len - 1) << 8 | (code & 0xff));
        }
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

/*
 * Encode @c's picture, into @pic and @out, with @opts the machine's options
 * beyond its name; @after, if any, runs with the job over
 */
static void run_case(const Case *c, const char *opts, uint8_t *pic,
                     uint8_t *out, void (*after)(QTestState *, const uint8_t *,
                                                 uint32_t, void *),
                     void *opaque)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 %s", opts ?: "");
    size_t ny = (size_t)c->w * c->h, n = ny * 3 / 2;
    g_autofree List *l = g_new0(List, 1);
    uint32_t seed = 12345, stat = 0;

    for (size_t i = 0; i < n; i++) {
        seed = seed * 1103515245 + 12345;
        pic[i] = seed >> 16;
    }
    qtest_memwrite(qts, SRC_Y, pic, ny);
    qtest_memwrite(qts, SRC_C, pic + ny, n - ny);
    for (int i = 0; i < 128; i++) {
        put(l, JPGC_QMEM + 4 * i, qmem[i]);
    }
    put_huff(l, ac_luma, true, 0);
    put_huff(l, ac_chroma, true, 176);
    put_huff(l, dc_luma, false, 352);
    put_huff(l, dc_chroma, false, 368);
    put(l, EFE + 0x04, (c->h / 16 - 1) << 16 | (c->w / 16 - 1));
    put(l, EFE + 0x10, SRC_Y);
    put(l, EFE + 0x14, SRC_C);
    put(l, EFE + 0x38, c->w << 16 | c->w);
    put(l, JPGC + 0x04, 1);
    put(l, JPGC + 0x0c, BS);
    put(l, JPGC + 0x28, (c->w / 16) * (c->h / 16) - 1);
    put(l, EFE, 0x400b);
    l->v[l->n - 1] |= cpu_to_le32(CL_TERM);
    qtest_memwrite(qts, LIST, l->v, l->n * 4);

    qtest_writel(qts, JPEG + VDMA_TASKRG, LIST | 1);
    for (int i = 0; i < 20000 && !(stat & 0x80000000u); i++) {
        g_usleep(500);
        stat = qtest_readl(qts, JPEG + JPGC_STAT);
    }
    g_assert_cmphex(qtest_readl(qts, JPEG + SCH_STAT), ==, 0x11);
    g_assert_cmpuint(stat & 0xffffff, ==, c->len);
    qtest_memread(qts, BS, out, c->len);
    g_assert_cmphex(crc32_le(out, c->len), ==, c->crc);
    if (after) {
        after(qts, out, c->len, opaque);
    }
    qtest_quit(qts);
}

static void test_case(const void *data)
{
    const Case *c = data;
    g_autofree uint8_t *pic = g_malloc((size_t)c->w * c->h * 3 / 2);
    g_autofree uint8_t *out = g_malloc(c->len);

    run_case(c, NULL, pic, out, NULL, NULL);
}

static void check_file(const char *dir, const char *name, const void *want,
                       size_t len)
{
    g_autofree char *path = g_build_filename(dir, name, NULL);
    g_autofree char *got = NULL;
    size_t n;

    g_assert_true(g_file_get_contents(path, &got, &n, NULL));
    g_assert_cmpuint(n, ==, len);
    g_assert_cmpmem(got, n, want, len);
    unlink(path);
}

/* The job's scan, picture and settings as files */
static void test_capture(void)
{
    const Case *c = &cases[0];
    size_t n = (size_t)c->w * c->h * 3 / 2;
    g_autofree char *dir = g_dir_make_tmp("ingenic-capture-XXXXXX", NULL);
    g_autofree char *txt = g_build_filename(dir, "helix-000000.txt", NULL);
    g_autofree uint8_t *pic = g_malloc(n);
    g_autofree uint8_t *out = g_malloc(c->len);
    g_autofree char *info = NULL;
    g_autofree char *size = g_strdup_printf("width %u\nheight %u\n"
                                            "bytes %u\n", c->w, c->h, c->len);

    g_autofree char *opts = g_strdup_printf("-global "
                                            "ingenic-helix.capture=%s", dir);

    run_case(c, opts, pic, out, NULL, NULL);
    check_file(dir, "helix-000000.jpg", out, c->len);
    check_file(dir, "helix-000000.nv12", pic, n);
    g_assert_true(g_file_get_contents(txt, &info, NULL, NULL));
    g_assert_true(g_str_has_prefix(info, "codec jpeg\n"));
    g_assert_nonnull(strstr(info, size));
    unlink(txt);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

/* libimp's bytes round the scan, built where nothing else is, 16 MiB up */
#define TAP_FILE        0x01000000

static const uint8_t jfif[] = {
    0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00, 0x01, 0x01,
    0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00,
    0xff, 0xda, 0x00, 0x0c, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03, 0x11, 0x00,
    0x3f, 0x00,
};

static void after_tap(QTestState *qts, const uint8_t *scan, uint32_t len,
                      void *opaque)
{
    const char *dir = opaque;
    g_autofree char *path = g_build_filename(dir, "helix-0.mjpg", NULL);
    g_autofree uint8_t *want = g_malloc(sizeof(jfif) + len + 2);
    g_autofree char *got = NULL;
    gsize n;

    memcpy(want, jfif, sizeof(jfif));
    memcpy(want + sizeof(jfif), scan, len);
    want[sizeof(jfif) + len] = 0xff;
    want[sizeof(jfif) + len + 1] = 0xd9;
    /* the core's own window, with zeros before it, is not the stream */
    g_assert_false(g_file_test(path, G_FILE_TEST_EXISTS));
    qtest_memwrite(qts, TAP_FILE, want, sizeof(jfif) + len + 2);
    qtest_clock_step(qts, 300 * SCALE_MS);
    g_assert_true(g_file_get_contents(path, &got, &n, NULL));
    g_assert_cmpmem(got, n, want, sizeof(jfif) + len + 2);
    unlink(path);
}

/* The stream tap finds the scan in memory and joins libimp's bytes to it */
static void test_tap(void)
{
    const Case *c = &cases[0];
    g_autofree char *dir = g_dir_make_tmp("ingenic-tap-XXXXXX", NULL);
    g_autofree char *opts = g_strdup_printf("-global "
                                            "ingenic-helix.x-stream-tap=%s",
                                            dir);
    g_autofree uint8_t *pic = g_malloc((size_t)c->w * c->h * 3 / 2);
    g_autofree uint8_t *out = g_malloc(c->len);

    run_case(c, opts, pic, out, after_tap, dir);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

/* The ISP the engine takes its IVDC pictures from */
static void test_isp_link(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/machine/soc/jpeg', 'property': 'isp' } }");

    g_assert(qdict_haskey(r, "return"));
    g_assert_cmpstr(qdict_get_str(r, "return"), ==, "/machine/soc/isp");
    qobject_unref(r);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic/t41-jpeg/capture", test_capture);
    qtest_add_func("ingenic/t41-jpeg/stream-tap", test_tap);
    qtest_add_func("ingenic/t41-jpeg/isp-link", test_isp_link);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/t41-jpeg/%ux%u",
                                                cases[i].w, cases[i].h);

        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
