/*
 * QTest testcase for the T32's HERA choosing between skip and intra in the
 * noise of a P picture
 *
 * A checkerboard of 32x32 cells, flat 128 between, the others carrying noise
 * of one sigma (new noise in the P picture), at QP 28 with the QP generator
 * on. A T32LQ at QP 28 codes the P picture of cells of sigma 1.5 as skips (12%
 * of their area intra) and of sigma 3 as intra blocks (96%), with the cliff
 * between sigma 1.75 (26%) and 2 (93%); at QP 36 every sigma up to 6 is
 * skipped. The model keeps to that: a lambda of 48% of the picture's for a
 * moving block and 60% for a static one with noise puts its cliff in the
 * same place, where 100% and 20% had intra at 1.5 and skips at 2 to 3.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define HERA            0x13200000
#define WIDTH           320
#define ROWS            192
#define SRC_Y           0x00800000
#define SRC_C           0x00840000
#define LIST            0x00880000
#define BS              0x008a0000
#define HL_VLD          (1u << 31)
#define HL_TERM         (1u << 30)

static uint32_t hash2(uint32_t x, uint32_t y)
{
    uint32_t h = (x * 73856093u) ^ (y * 19349663u);

    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}

/* Noise of a standard deviation near sigma / 10: four uniform bytes, summed */
static int noise(int x, int y, uint32_t seed, int sigma10)
{
    uint32_t h = hash2(x + seed * 7919u, y + seed * 104729u);
    int sum = (int)(h & 255) + (int)(h >> 8 & 255) + (int)(h >> 16 & 255) +
              (int)(h >> 24) - 510;

    /* the sum has a deviation of 147.8 */
    return (sum * sigma10 + (sum < 0 ? -739 : 739)) / 1478;
}

#define WANT_LO 117
#define WANT_LEN_LO 308
#define WANT_HI 47
#define WANT_LEN_HI 1129

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


static unsigned p_intra(int sigma10, uint32_t *len_out)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-hera-noise-XXXXXX", NULL);
    g_autofree char *args = g_strdup_printf(
        "-machine ingenic-t32 -global ingenic-hera.h265-motion=search "
        "-global ingenic-hera.capture=%s", dir);
    QTestState *qts = qtest_init(args);
    uint8_t y[WIDTH * ROWS], cc[WIDTH * ROWS / 2], out[0x20000];
    g_autofree char *path = NULL, *txt = NULL;
    unsigned intra = 0;
    char *p;
    gsize sz;

    memset(cc, 128, sizeof(cc));
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < WIDTH * ROWS; i++) {
            int x = i % WIDTH, r = i / WIDTH, v = 128;

            if (!((x / 32 + r / 32) & 1)) {
                v += noise(x, r, pass + 1, sigma10);
            }
            y[i] = MIN(MAX(v, 0), 255);
        }
        *len_out = job(qts, pass == 0, y, cc, out);
    }
    qtest_quit(qts);

    path = g_strdup_printf("%s/hera-000001.txt", dir);
    g_assert_true(g_file_get_contents(path, &txt, &sz, NULL));
    p = strstr(txt, "luma_modes");
    g_assert_nonnull(p);
    p += strlen("luma_modes");
    for (int i = 0; i < 35; i++) {
        intra += g_ascii_strtoull(p, &p, 10);
    }
    rm_dir(dir);
    return intra;
}

static void test_noise(void)
{
    uint32_t len15, len30;
    unsigned lo = p_intra(15, &len15), hi = p_intra(30, &len30);

    g_test_message("P: sigma 1.5 %u intra blocks, %u bytes; sigma 3 %u, %u",
                   lo, len15, hi, len30);
    g_assert_cmpuint(lo, ==, WANT_LO);
    g_assert_cmpuint(len15, ==, WANT_LEN_LO);
    g_assert_cmpuint(hi, ==, WANT_HI);
    g_assert_cmpuint(len30, ==, WANT_LEN_HI);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/hera-p/noise", test_noise);
    return g_test_run();
}
