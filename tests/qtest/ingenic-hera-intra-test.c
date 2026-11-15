/*
 * QTest testcase for the T32's HERA predicting H.265 intra blocks with all 35
 * modes
 *
 * A picture of six 64x64 cells (a ramp, two edges, two sets of bars and
 * noise) made here, as an I picture with the QP generator on. Injected into
 * a T32LQ, tiled over 1080p, the camera predicts the vertical bars with mode
 * 26 (80% of their area), the horizontal ones with 10 (63%), the noise with
 * DC (95%) and the ramp with 30 and 31, never with 34, and the model's
 * modes are those, here and in the same picture of silicon's size. The
 * model's mode counts are pinned, and the stream and its reconstruction by
 * CRC: libavcodec and the HM reference decoder both decoded the same picture
 * coded with the model's other settings to the reconstruction's samples.
 *
 * The camera's coding blocks of the same picture (the top left tile of its
 * 1080p I picture, which the blocks before it alone decide) are SILICON_MAP
 * below, one digit for each 8x8: 0 for a block of 8x8, 1 of 16x16, 2 of
 * 32x32. The model's own maps are pinned; at QP 28 it agrees with the
 * camera's in 196 of the 384 8x8s (with only 32x32 blocks, in 96).
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

typedef struct {
    int qp;
    unsigned len;
    uint32_t crc, rec_crc;
    unsigned luma[35], chroma[5];
    const char *map[ROWS / 8];
    unsigned agree;             /* 8x8s of the camera's map it has, or 0 */
} IngenicIntraCase;

static const char *const SILICON_MAP[ROWS / 8] = {
    "000011000011222211002222", "000011000011222211002222",
    "000011000000222211002222", "000011000000222211002222",
    "111111000011001111110011", "111111000011001111110011",
    "110011000011110000110000", "110011000011110000110000",
    "000000002222111111112222", "000000002222111111112222",
    "110000002222111111112222", "110000002222111111112222",
    "110011111111111122222222", "110011111111111122222222",
    "110011111111111122222222", "110011111111111122222222",
};

static const IngenicIntraCase cases[] = {
    { 28, 1241, 0x6c047a89, 0xec84bf49,
      {
        20, 40, 15, 0, 0, 0, 0, 0, 0, 7, 47, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 7,
        0, 0, 0, 24, 47, 4, 0, 0, 18, 53, 2, 0, 0
      },
      { 2, 0, 0, 0, 289 },
      {
        "000000000011222200002222",
        "000000000011222200002222",
        "000000000000222211002222",
        "000000000000222211002222",
        "000000000011000011000011",
        "000000000011000011000011",
        "000000000011110000110011",
        "000000000011110000110011",
        "000000000011111100000000",
        "000000000011111100000000",
        "111111000000000000000000",
        "111111000000000000000000",
        "111111000000000000000000",
        "111111000000000000000000",
        "111111000000000000000000",
        "111111000000000000000000",
      },
      196 },
    { 36, 439, 0xecf30a51, 0xcb94046c,
      {
        11, 13, 10, 0, 0, 0, 0, 0, 0, 1, 60, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 7,
        0, 0, 0, 14, 26, 8, 0, 0, 7, 45, 0, 13, 0
      },
      { 6, 1, 0, 1, 214 },
      {
        "000000000011222200002222",
        "000000000011222200002222",
        "110000000000222211002222",
        "110000000000222211002222",
        "000000001111001111110011",
        "000000001111001111110011",
        "000000000011110011110011",
        "000000000011110011110011",
        "000000000000000000002222",
        "000000000000000000002222",
        "111111110000000000112222",
        "111111110000000000112222",
        "222222220000000022222222",
        "222222220000000022222222",
        "222222220000000022222222",
        "222222220000000022222222",
      },
      0 },
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

static void test_intra(const void *data)
{
    const IngenicIntraCase *c = data;
    g_autofree char *dir = g_dir_make_tmp("ingenic-hera-intra-XXXXXX", NULL);
    g_autofree char *args = g_strdup_printf(
        "-machine ingenic-t32 -global ingenic-hera.capture=%s", dir);
    QTestState *qts = qtest_init(args);
    uint32_t pairs[32], n = 0, len, crc, rec_crc;
    uint8_t y[WIDTH * ROWS], cc[WIDTH * ROWS / 2], out[0x20000];
    g_autofree char *path = NULL, *buf = NULL, *txt = NULL;
    unsigned lm[35] = { 0 }, cm[5] = { 0 };
    char *p;
    unsigned agree = 0;
    gsize sz;
    const struct {
        uint32_t reg, val;
    } regs[] = {
        { 0x2000c, SRC_Y }, { 0x20010, SRC_C },
        { 0x20014, WIDTH | WIDTH << 16 },
        { 0x68004, WIDTH << 16 | ROWS },
        /* I, cu_qp_delta to a depth of 2 */
        { 0x68000, 2 << 2 | 1 << 4 | 2 << 18 },
        { 0x60004, c->qp << 1 }, { 0x20004, 1 },
    };

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
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, cc, sizeof(cc));
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
    g_assert_cmpuint(len, <, sizeof(out));
    qtest_memread(qts, BS, out, len);
    crc = crc32_le(out, len);
    qtest_quit(qts);

    path = g_strdup_printf("%s/hera-000000.rec", dir);
    g_assert_true(g_file_get_contents(path, &buf, &sz, NULL));
    g_assert_cmpuint(sz, ==, WIDTH * ROWS * 3 / 2);
    rec_crc = crc32_le((const uint8_t *)buf, sz);
    g_free(path);
    path = g_strdup_printf("%s/hera-000000.txt", dir);
    g_assert_true(g_file_get_contents(path, &txt, &sz, NULL));
    p = strstr(txt, "luma_modes");
    g_assert_nonnull(p);
    p += strlen("luma_modes");
    for (int i = 0; i < 35; i++) {
        lm[i] = g_ascii_strtoull(p, &p, 10);
    }
    p = strstr(p, "chroma_modes") + strlen("chroma_modes");
    for (int i = 0; i < 5; i++) {
        cm[i] = g_ascii_strtoull(p, &p, 10);
    }
    p = strstr(p, "cu_map\n");
    g_assert_nonnull(p);
    p += strlen("cu_map\n");
    for (int r = 0; r < ROWS / 8; r++, p += WIDTH / 8 + 1) {
        g_assert_cmpint(strncmp(p, c->map[r], WIDTH / 8), ==, 0);
        for (int x = 0; x < WIDTH / 8; x++) {
            agree += p[x] == SILICON_MAP[r][x];
        }
    }
    g_test_message("qp %d: %u bytes, crc %08x, rec %08x, %u of %d 8x8s as the "
                   "camera's", c->qp, len, crc, rec_crc, agree,
                   WIDTH / 8 * ROWS / 8);

    g_assert_cmpuint(len, ==, c->len);
    g_assert_cmphex(crc, ==, c->crc);
    g_assert_cmphex(rec_crc, ==, c->rec_crc);
    g_assert_cmpmem(lm, sizeof(lm), c->luma, sizeof(lm));
    g_assert_cmpmem(cm, sizeof(cm), c->chroma, sizeof(cm));
    if (c->agree) {
        g_assert_cmpuint(agree, ==, c->agree);
    }
    g_assert_cmpuint(lm[34], ==, 0);
    rm_dir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *p = g_strdup_printf("ingenic/hera-intra/qp%d",
                                             cases[i].qp);

        qtest_add_data_func(p, &cases[i], test_intra);
    }
    return g_test_run();
}
