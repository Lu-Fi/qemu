/*
 * QTest testcase for the T32's HERA coding an H.265 P picture
 *
 * The six cell picture of ingenic-hera-intra-test (192x128), twice: an I
 * picture at QP 28 and a P picture of the same samples with the motion
 * search on. Injected into a T32LQ tiled over 1080p as an I picture and the
 * P pictures after it (QP 28 fixed, the QP generator on), the camera codes
 * the first P picture of the top left tile with the coding blocks of
 * SILICON_MAP below, one digit for each 8x8: 0 for a block of 8x8, 1 of
 * 16x16, 2 of 32x32. It skips 32x32 blocks of the flat and bar cells and
 * codes the edges and the horizontal bars as intra blocks of 16x16. The
 * model's own picture is pinned by its length, CRC, and the map; it agrees
 * with the camera's in 272 of the 384 8x8s and codes 99 bytes of the
 * camera's 114 for a tile (117 before its mode decisions took the lambdas of
 * the noise ladder, 121 before its structure cells kept to 32x32 blocks,
 * 58 before the QP generator took its classes from
 * the mean deviation and kept the picture's edge blocks at the slice QP; 15
 * before the flat cells' vector search and intra blocks): the camera's intra
 * blocks of flat and bar cells are only partly modelled. The picture decodes
 * to the coder's reconstruction in libavcodec and HM.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define HERA            0x13200000
#define WIDTH           192
#define ROWS            128
#define SRC_Y           0x00800000
#define SRC_C           0x00820000
#define LIST            0x00860000
#define BS              0x00880000
#define HL_VLD          (1u << 31)
#define HL_TERM         (1u << 30)

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t c = ~0u;

    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) {
            c = c >> 1 ^ (c & 1 ? 0xedb88320u : 0);
        }
    }
    return ~c;
}

static uint32_t hash2(uint32_t x, uint32_t y)
{
    uint32_t h = (x * 73856093u) ^ (y * 19349663u);

    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}

/* Cells: 0 ramp, 1 and 2 edges, 3 vertical, 4 horizontal bars, 5 noise */
static int luma(int x, int y)
{
    int cell = x / 64 + 3 * (y / 64), u = x % 64, v = y % 64;
    uint32_t h;

    switch (cell) {
    case 0:
        return 64 + 2 * u + v;
    case 1:
        return u > v ? 190 : 70;
    case 2:
        return v < 2 * u - 30 ? 200 : 60;
    case 3:
        return (u / 8) & 1 ? 200 : 56;
    case 4:
        return (v / 6) & 1 ? 190 : 70;
    default:
        h = hash2(x, y);
        return 128 + (int)(h & 15) + (int)(h >> 8 & 15) - 15;
    }
}


static const char *const SILICON_MAP[ROWS / 8] = {
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222221111110011110011",
    "222222221111110011110011",
    "222222221111111111110011",
    "222222221111111111110011",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "111111112222111122222222",
    "111111112222111122222222",
    "110011112222111122222222",
    "110011112222111122222222",
};

#define WANT_LEN        99
#define WANT_CRC        0xad566167
#define WANT_REC        0x2d4c50cd
#define WANT_INTRA      2
#define AGREE           272

static const char *const WANT_MAP[ROWS / 8] = {
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
    "222222222222222222222222",
};

static void rm_dir(const char *dir)
{
    GDir *d = g_dir_open(dir, 0, NULL);
    const char *f;

    while (d && (f = g_dir_read_name(d))) {
        g_autofree char *p = g_build_filename(dir, f, NULL);

        unlink(p);
    }
    if (d) {
        g_dir_close(d);
    }
    rmdir(dir);
}

static uint32_t job(QTestState *qts, bool intra, const uint8_t *y,
                    const uint8_t *cc, uint8_t *out)
{
    uint32_t pairs[32], n = 0, len;
    const struct {
        uint32_t reg, val;
    } regs[] = {
        { 0x2000c, SRC_Y }, { 0x20010, SRC_C },
        { 0x20014, WIDTH | WIDTH << 16 },
        { 0x68004, WIDTH << 16 | ROWS },
        /* cu_qp_delta to a depth of 2 */
        { 0x68000, (intra ? 2 : 1) << 2 | 1 << 4 | 2 << 18 },
        { 0x60004, 28 << 1 }, { 0x20004, 1 },
    };

    qtest_memwrite(qts, SRC_Y, y, WIDTH * ROWS);
    qtest_memwrite(qts, SRC_C, cc, WIDTH * ROWS / 2);
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        pairs[n++] = cpu_to_le32(regs[i].val);
        pairs[n++] = cpu_to_le32(HL_VLD | (i == ARRAY_SIZE(regs) - 1 ?
                                           HL_TERM : 0) | regs[i].reg);
    }
    qtest_memwrite(qts, LIST, pairs, n * 4);
    qtest_writel(qts, HERA + 0x2c, 0x7ff);
    qtest_writel(qts, HERA + 0x90008, BS);
    qtest_writel(qts, HERA + 0x90068, LIST);
    qtest_writel(qts, HERA + 0x8, 1);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, HERA + 0x2c) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, HERA + 0x2c) & 1, ==, 1);
    len = qtest_readl(qts, HERA + 0x44);
    g_assert_cmpuint(len, <, 0x20000);
    qtest_memread(qts, BS, out, len);
    qtest_writel(qts, HERA + 0x30, 0x7ff);
    return len;
}

static void test_p(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-hera-p-XXXXXX", NULL);
    g_autofree char *args = g_strdup_printf(
        "-machine ingenic-t32 -global ingenic-hera.h265-motion=search "
        "-global ingenic-hera.capture=%s", dir);
    QTestState *qts = qtest_init(args);
    uint8_t y[WIDTH * ROWS], cc[WIDTH * ROWS / 2], out[0x20000];
    g_autofree char *path = NULL, *buf = NULL, *txt = NULL;
    uint32_t len, crc, rec_crc;
    unsigned intra = 0, agree = 0;
    char *p;
    gsize sz;

    for (int i = 0; i < WIDTH * ROWS; i++) {
        y[i] = luma(i % WIDTH, i / WIDTH);
    }
    for (int r = 0; r < ROWS / 2; r++) {
        for (int x = 0; x < WIDTH; x += 2) {
            int yv = luma(x, 2 * r);

            cc[r * WIDTH + x] = 128 + (yv - 128) / 3;
            cc[r * WIDTH + x + 1] = 128 - (yv - 128) / 4;
        }
    }
    job(qts, true, y, cc, out);
    len = job(qts, false, y, cc, out);
    crc = crc32_le(out, len);
    qtest_quit(qts);

    path = g_strdup_printf("%s/hera-000001.rec", dir);
    g_assert_true(g_file_get_contents(path, &buf, &sz, NULL));
    g_assert_cmpuint(sz, ==, WIDTH * ROWS * 3 / 2);
    rec_crc = crc32_le((const uint8_t *)buf, sz);
    g_free(path);
    path = g_strdup_printf("%s/hera-000001.txt", dir);
    g_assert_true(g_file_get_contents(path, &txt, &sz, NULL));
    p = strstr(txt, "luma_modes");
    g_assert_nonnull(p);
    p += strlen("luma_modes");
    for (int i = 0; i < 35; i++) {
        intra += g_ascii_strtoull(p, &p, 10);
    }
    p = strstr(p, "cu_map\n");
    g_assert_nonnull(p);
    p += strlen("cu_map\n");
    for (int r = 0; r < ROWS / 8; r++, p += WIDTH / 8 + 1) {
        for (int x = 0; x < WIDTH / 8; x++) {
            agree += p[x] == SILICON_MAP[r][x];
        }
    }
    g_test_message("P: %u bytes, crc %08x, rec %08x, %u intra blocks, %u of "
                   "%d 8x8s as the camera's", len, crc, rec_crc, intra, agree,
                   WIDTH / 8 * ROWS / 8);
    g_assert_cmpuint(len, ==, WANT_LEN);
    g_assert_cmphex(crc, ==, WANT_CRC);
    g_assert_cmphex(rec_crc, ==, WANT_REC);
    g_assert_cmpuint(intra, ==, WANT_INTRA);
    g_assert_cmpuint(agree, ==, AGREE);
    p = strstr(txt, "cu_map\n") + strlen("cu_map\n");
    for (int r = 0; r < ROWS / 8; r++, p += WIDTH / 8 + 1) {
        g_assert_cmpint(strncmp(p, WANT_MAP[r], WIDTH / 8), ==, 0);
    }
    rm_dir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/hera-p/six-cells", test_p);
    return g_test_run();
}
