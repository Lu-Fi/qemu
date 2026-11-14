/*
 * QTest testcase for the T31 AVPU's H.265 command lists of several slices
 *
 * A synthetic 96x64 picture (3x2 CTBs) as an H.265 intra picture in command
 * lists of several entries. What a T31's core did with the same lists
 * (measured with the synthetic picture injected into the encoder): an entry
 * is a slice of whole rows of CTBs from the row of word 5 to the row of
 * word 6, however far along its first row it starts; it is coded as a
 * picture of its own, so that a slice's data is the same whatever the other
 * entries are; status word 0x45 counts the 8x8 blocks of its rows and 0x4c
 * the scores of their CTBs; one interrupt ends the list. The model's H.265
 * coding decides blocks its own way, so its bytes are not silicon's: what
 * is checked is the structure, the slices that silicon wrote alike are alike
 * here (the CRC of silicon's slice for each is in the case's comment), and
 * the status words.
 *
 * The capture's reconstruction (capture=DIR) of I and P pictures as one slice
 * and two, with the motion search off and on, is pinned by CRC: the pictures
 * libavcodec decodes from the captured streams (parameter sets and slice
 * headers written by hand ahead of the model's slice data) are the same
 * bytes, edges between slices filtered as in one picture, which is what the
 * T31's core does (a P picture coded after a two-slice I picture was
 * decoded by libavcodec to the source at the slice's edge only with the
 * filter across slices on).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define BASE            0x13200000
#define CL_ADDR         0x83e0
#define START           0x83e4
#define IRQ             0x8018

#define W               96
#define H               64
#define CTB_W           3
#define CTB_H           2
#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define LIST            0x00820000
#define EP2             0x00860000
#define BS              0x00840000
#define SLOT            0x2000      /* stream bytes a slice has */
#define HDR             0x220
#define ENTRY           0x200
#define STATUS          0x40
#define MAX_ENTRIES     4

/* An entry's first and last CTB, as words 5 and 6 have them */
typedef struct {
    unsigned x0, y0, x1, y1;
    uint32_t st45, st4c;        /* silicon's status words 0x45 and 0x4c */
} Entry;

typedef struct {
    const char *name;
    int n;
    Entry e[MAX_ENTRIES];
    /* entries whose data is another case's: entry, case, its entry */
    int nsame;
    struct {
        int entry, other, other_entry;
    } same[MAX_ENTRIES];
} Case;

/*
 * Silicon's lists. Each case is a list of the entries below: the rows are
 * coded from the first row's start, wherever the first CTB is.
 */
static const Case cases[] = {
    /* the whole picture, one entry (CRC 0xa20b1c34) */
    { "whole", 1, { { 0, 0, 2, 1, 0x60, 0x22649 } }, 0 },
    /* one row each (0x70ad4f97, 0xe8523ce0) */
    { "rows", 2, { { 0, 0, 2, 0, 0x30, 0x11380 },
                   { 0, 1, 2, 1, 0x30, 0x112c9 } }, 0 },
    /*
     * the second entry starts at the second CTB of the first row: it codes
     * from the row's start, the whole picture's data; the first, a CTB long,
     * codes the whole row (0x70ad4f97, 0xa20b1c34)
     */
    { "mid-row", 2, { { 0, 0, 0, 0, 0x30, 0x11380 },
                      { 1, 0, 2, 1, 0x60, 0x22649 } },
      2, { { 0, 1, 0 }, { 1, 0, 0 } } },
    /*
     * the first entry ends at the first CTB of the second row, the whole
     * picture's data; the second is that row alone (0xa20b1c34, 0xe8523ce0)
     */
    { "to-mid-row", 2, { { 0, 0, 0, 1, 0x60, 0x22649 },
                         { 1, 1, 2, 1, 0x30, 0x112c9 } },
      2, { { 0, 0, 0 }, { 1, 1, 1 } } },
};

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

static int pmod(int v, int m)
{
    return (v % m + m) % m;
}

/* The picture, frame @f of a pattern that moves 2 samples to the right */
static void picture_at(uint8_t *y, uint8_t *c, int f)
{
    for (int r = 0; r < H; r++) {
        for (int x = 0; x < W; x++) {
            int xs = x - 2 * f;
            int v = 64 + pmod(xs * 5 + r * 3, 128) +
                    (((xs >> 4) ^ (r >> 4)) & 1) * 32 +
                    ((xs * 7919 + r * 104729) >> 5 & 15);

            y[r * W + x] = MIN(v, 235);
        }
    }
    for (int r = 0; r < H / 2; r++) {
        for (int x = 0; x < W / 2; x++) {
            c[r * W + 2 * x] = 128 + pmod((x - f) * 3, 64) - 32;
            c[r * W + 2 * x + 1] = 128 + pmod(r * 3, 64) - 32;
        }
    }
}

static void run_list(QTestState *qts, const Entry *e, int n, uint32_t w3,
                     uint32_t w4)
{
    uint32_t list[ENTRY / 4 * MAX_ENTRIES] = { 0 };

    for (int k = 0; k < n; k++) {
        uint32_t *w = list + k * ENTRY / 4;
        unsigned count = (e[k].y1 - e[k].y0 + 1) * CTB_W - e[k].x0;

        w[0] = 0x11700c11 | (k == n - 1 ? 1u << 31 : 0);
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
        w[2] = 0xc010ad50;
        w[3] = w3;
        w[4] = w4;
        w[5] = e[k].x0 | e[k].y0 << 12;
        w[6] = e[k].x1 | e[k].y1 << 12;
        w[7] = 1u << 31 | (CTB_W - 1) | (CTB_H - 1) << 12;
        w[9] = 0xfc048000;
        w[0xb] = w[0x1d] = (CTB_W - 1) << 12;
        w[0x1e] = count - 1;
        w[0x20] = SRC_Y;
        w[0x21] = SRC_C;
        w[0x22] = W;
        w[0x23] = EP2;
        w[0x30] = BS;
        w[0x32] = HDR + k * SLOT;
        w[0x33] = SLOT - HDR;
    }
    for (int i = 0; i < ENTRY / 4 * n; i++) {
        list[i] = cpu_to_le32(list[i]);
    }
    qtest_memwrite(qts, LIST, list, ENTRY * n);
    qtest_writel(qts, BASE + CL_ADDR, LIST);
    qtest_writel(qts, BASE + START, 2);
    for (int i = 0; i < 40000 && !(qtest_readl(qts, BASE + IRQ) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, BASE + IRQ), ==, 1);
    qtest_writel(qts, BASE + IRQ, 1);
}

static void setup(QTestState *qts, int f)
{
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };

    picture_at(y, uv, f);
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
}

/* An entry's slice data, its length returned */
static uint32_t slice_data(QTestState *qts, int k, uint8_t *out)
{
    uint32_t len = qtest_readl(qts, LIST + k * ENTRY + (STATUS + 1) * 4);

    g_assert_cmpuint(len, <=, SLOT - HDR);
    qtest_memread(qts, BS + HDR + k * SLOT, out, len);
    return len;
}

static void test_slices(const void *data)
{
    const Case *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t31");
    uint8_t d[MAX_ENTRIES][SLOT];
    uint32_t len[MAX_ENTRIES];

    setup(qts, 0);
    run_list(qts, c->e, c->n, 0x211e0000, 0x00083f1f);
    for (int k = 0; k < c->n; k++) {
        uint32_t base = LIST + k * ENTRY + STATUS * 4;

        len[k] = slice_data(qts, k, d[k]);
        g_assert_cmpuint(len[k], >, 0);
        g_assert_cmphex(qtest_readl(qts, base + 5 * 4), ==, c->e[k].st45);
        g_assert_cmphex(qtest_readl(qts, base + 12 * 4), ==, c->e[k].st4c);
        g_assert_cmphex(qtest_readl(qts, base + 7 * 4), ==, 0);
    }
    for (int i = 0; i < c->nsame; i++) {
        const Case *o = &cases[c->same[i].other];
        uint8_t od[SLOT];
        uint32_t olen;

        run_list(qts, o->e, o->n, 0x211e0000, 0x00083f1f);
        olen = slice_data(qts, c->same[i].other_entry, od);
        g_assert_cmpuint(olen, ==, len[c->same[i].entry]);
        g_assert_cmpint(memcmp(od, d[c->same[i].entry], olen), ==, 0);
    }
    qtest_quit(qts);
}

/*
 * I and P pictures of a pattern moving 2 samples a picture, the P pictures
 * coded one slice or two, the motion search off or on: the CRC of each
 * picture's reconstruction
 */
typedef struct {
    int n;
    bool search;
    uint32_t crc[3];
} RecCase;

static const RecCase rec_cases[] = {
    { 1, false, { 0x577b62a4, 0x72d67923, 0x5fe0fe34 } },
    { 2, false, { 0x92fa6578, 0x1e037285, 0xcd365b26 } },
    { 1, true, { 0x577b62a4, 0x48a583a3, 0x8593955c } },
    { 2, true, { 0x92fa6578, 0x37e7333b, 0x46ca1fe1 } },
};

static uint32_t rec_crc(const char *dir, unsigned seq, uint32_t *len)
{
    g_autofree char *path = g_strdup_printf("%s/avpu-%06u.rec", dir, seq);
    g_autofree char *buf = NULL;
    gsize n;

    g_assert_true(g_file_get_contents(path, &buf, &n, NULL));
    *len = n;
    return crc32_le((const uint8_t *)buf, n);
}

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

static void test_rec(const void *data)
{
    const RecCase *c = data;
    g_autofree char *dir = g_dir_make_tmp("ingenic-avpu-rec-XXXXXX", NULL);
    g_autofree char *args = g_strdup_printf("-machine ingenic-t31 "
        "-global ingenic-avpu.capture=%s%s", dir,
        c->search ? " -global ingenic-avpu.h265-motion=search" : "");
    QTestState *qts = qtest_init(args);
    Entry e[2] = { { 0, 0, 2, 1 }, };
    uint32_t len;

    if (c->n == 2) {
        e[0] = (Entry){ 0, 0, 2, 0 };
        e[1] = (Entry){ 0, 1, 2, 1 };
    }
    for (int f = 0; f < 3; f++) {
        setup(qts, f);
        run_list(qts, e, c->n, f ? 0x111e0000 : 0x211e0000,
                 0x00083f1f);
        g_assert_cmphex(rec_crc(dir, f, &len), ==, c->crc[f]);
        g_assert_cmpuint(len, ==, W * H * 3 / 2);
    }
    qtest_quit(qts);
    rm_dir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-avpu/hevc-slices/%s",
                                                cases[i].name);

        qtest_add_data_func(path, &cases[i], test_slices);
    }
    for (int i = 0; i < ARRAY_SIZE(rec_cases); i++) {
        g_autofree char *path = g_strdup_printf(
            "/ingenic-avpu/hevc-slices/rec/%d-slices%s", rec_cases[i].n,
            rec_cases[i].search ? "-search" : "");

        qtest_add_data_func(path, &rec_cases[i], test_rec);
    }
    return g_test_run();
}
