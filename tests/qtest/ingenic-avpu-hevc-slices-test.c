/*
 * QTest testcase for the T31 AVPU's H.265 command lists of several slices
 *
 * A synthetic 96x64 picture (3x2 CTBs) as an H.265 intra picture in command
 * lists of several entries, for the T31's EL150 and the T41's EL200 (the
 * EL200's entry: the first and last block in words 0x1a and 0x1b, the blocks
 * less one in word 5, entries 0x1000 apart, each with its status words at
 * 0x5c0). What the cores did with the same lists
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
#define ENTRY200        0x1000
#define STATUS200       0x5c0       /* where the T41's status words go */
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

/* The T31's EL150 and the T41's EL200: where the registers are and the words */
typedef struct {
    const char *name, *machine;
    uint32_t base, cl_addr, start, irq, status;
    bool el200;
    uint32_t entry;             /* bytes between entries */
    unsigned noise_len;         /* the noise picture's slice, and its error */
    uint64_t noise_se;
} Rig;

static const Rig rigs[] = {
    { "el150", "ingenic-t31", 0x13200000, 0x83e0, 0x83e4, 0x8018, 0, false,
      ENTRY, 981, 53368 },
    { "el200", "ingenic-t41", 0x13100000, 0x1000, 0x1018, 0x18, 0x1008, true,
      ENTRY200, 943, 53214 },
};

typedef struct {
    const Case *c;
    const Rig *r;
} CaseRig;

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

static void run_list(QTestState *qts, const Rig *r, const Entry *e, int n,
                     uint32_t w3, uint32_t w4, bool h264)
{
    unsigned uw = h264 ? W / 16 : CTB_W;
    g_autofree uint32_t *list = g_new0(uint32_t, r->entry / 4 * n);

    for (int k = 0; k < n; k++) {
        uint32_t *w = list + k * r->entry / 4;
        unsigned count = (e[k].y1 - e[k].y0 + 1) * uw - e[k].x0;
        uint32_t first = e[k].x0 | e[k].y0 << 12;
        uint32_t last = e[k].x1 | e[k].y1 << 12;

        if (r->el200) {
            w[0] = (h264 ? 0x00700000 : 0x01700c01) |
                   (k == n - 1 ? 1u << 31 : 0);
            w[1] = (W / 8 - 1) | (H / 8 - 1) << 16;
            w[2] = 0x10006;
            w[3] = 0x40000d50;
            w[5] = count - 1;
            w[0x18] = w3;
            w[0x19] = w4;
            w[0x1a] = first;
            w[0x1b] = last;
            w[0x20] = 1u << 31 | last;
            w[0x22] = (uw - 1) << 12;
            w[0x28] = SRC_Y;
            w[0x2a] = SRC_C;
            w[0x34] = W;
            w[0x88] = BS;
            w[0x8b] = 0x22d40;
            w[0x8c] = HDR + k * SLOT;
            w[0x8d] = SLOT - HDR;
            w[0x98] = h264 ? 0x34 : 0xf6;
            w[0x9a] = EP2;
            w[0x203] = (uw - 1) << 12;
            w[0x204] = count - 1;
            continue;
        }
        w[0] = 0x11700c11 | (k == n - 1 ? 1u << 31 : 0);
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
        w[2] = 0xc010ad50;
        w[3] = w3;
        w[4] = w4;
        w[5] = first;
        w[6] = last;
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
    for (int i = 0; i < r->entry / 4 * n; i++) {
        list[i] = cpu_to_le32(list[i]);
    }
    qtest_memwrite(qts, LIST, list, r->entry * n);
    if (r->el200) {
        qtest_writel(qts, r->base + r->status, LIST + STATUS200);
    }
    qtest_writel(qts, r->base + r->cl_addr, LIST);
    qtest_writel(qts, r->base + r->start, 2);
    for (int i = 0; i < 40000 && !(qtest_readl(qts, r->base + r->irq) & 1);
         i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, r->base + r->irq), ==, 1);
    qtest_writel(qts, r->base + r->irq, 1);
}

static void setup(QTestState *qts, int f)
{
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };

    picture_at(y, uv, f);
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
}

/* Status word @i of entry @k: the EL150's 0x40 + i, the EL200's from 0x41 */
static uint32_t status_word(QTestState *qts, const Rig *r, int k, int i)
{
    if (r->el200) {
        /* 0x46 is not there: words 5 on are one the less */
        return qtest_readl(qts, LIST + STATUS200 + k * ENTRY200 +
                           (i > 5 ? i - 2 : i - 1) * 4);
    }
    return qtest_readl(qts, LIST + k * ENTRY + (STATUS + i) * 4);
}

/* An entry's slice data, its length returned */
static uint32_t slice_data(QTestState *qts, const Rig *r, int k, uint8_t *out)
{
    uint32_t len = status_word(qts, r, k, 1);

    g_assert_cmpuint(len, <=, SLOT - HDR);
    qtest_memread(qts, BS + HDR + k * SLOT, out, len);
    return len;
}

static void test_slices(const void *data)
{
    const CaseRig *cr = data;
    const Case *c = cr->c;
    const Rig *r = cr->r;
    QTestState *qts = qtest_initf("-machine %s", r->machine);
    uint8_t d[MAX_ENTRIES][SLOT];
    uint32_t len[MAX_ENTRIES];

    setup(qts, 0);
    run_list(qts, r, c->e, c->n, 0x211e0000, 0x00083f1f, false);
    for (int k = 0; k < c->n; k++) {
        len[k] = slice_data(qts, r, k, d[k]);
        g_assert_cmpuint(len[k], >, 0);
        g_assert_cmphex(status_word(qts, r, k, 5), ==, c->e[k].st45);
        g_assert_cmphex(status_word(qts, r, k, 0xc), ==, c->e[k].st4c);
        g_assert_cmphex(status_word(qts, r, k, 7), ==, 0);
    }
    for (int i = 0; i < c->nsame; i++) {
        const Case *o = &cases[c->same[i].other];
        uint8_t od[SLOT];
        uint32_t olen;

        run_list(qts, r, o->e, o->n, 0x211e0000, 0x00083f1f, false);
        olen = slice_data(qts, r, c->same[i].other_entry, od);
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
    const Rig *r;
    int n;
    bool search;
    bool no_deblock;            /* word 4 bit 18: the reference unfiltered */
    bool h264;                  /* the EL200 only: H.264, rows of 16 */
    uint32_t crc[3];
} RecCase;

static const RecCase rec_cases[] = {
    { &rigs[0], 1, false, false, false,
      { 0xb8119911, 0x72d67923, 0x5fe0fe34 } },
    { &rigs[0], 2, false, false, false,
      { 0x3ea0a86f, 0x1e037285, 0xcd365b26 } },
    { &rigs[0], 1, true, false, false, { 0xb8119911, 0xff012880, 0x9969093a } },
    { &rigs[0], 2, true, false, false, { 0x3ea0a86f, 0xfb1b4093, 0x8641ac09 } },
    { &rigs[0], 1, false, true, false, { 0xdd810c16, 0x267fa93e, 0xbd4bdb0d } },
    { &rigs[0], 2, false, true, false, { 0xf31e987a, 0x1cc6585f, 0x73cc6a86 } },
    { &rigs[0], 1, true, true, false, { 0xdd810c16, 0x3560a774, 0x89cf8015 } },
    { &rigs[0], 2, true, true, false, { 0xf31e987a, 0x8135db2e, 0x6c5d23c5 } },
    { &rigs[1], 1, false, false, false,
      { 0x0ad5075f, 0x72d67923, 0x5fe0fe34 } },
    { &rigs[1], 2, false, false, false,
      { 0x0d904813, 0x1e037285, 0xcd365b26 } },
    { &rigs[1], 1, true, false, false, { 0x0ad5075f, 0x14113ddb, 0xc3b646b9 } },
    { &rigs[1], 2, true, false, false, { 0x0d904813, 0x5fd11dbc, 0x5d348fe5 } },
    { &rigs[1], 2, true, false, true, { 0x874ff146, 0x988ca979, 0x383bddfd } },
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
    g_autofree char *args = g_strdup_printf("-machine %s "
        "-global ingenic-avpu.capture=%s%s%s", c->r->machine, dir,
        c->search && !c->h264 ? " -global ingenic-avpu.h265-motion=search" : "",
        c->h264 ? " -global ingenic-avpu.h264-motion=search" : "");
    QTestState *qts = qtest_init(args);
    Entry e[2] = { { 0, 0, 2, 1 }, };
    uint32_t len;

    if (c->n == 2) {
        e[0] = (Entry){ 0, 0, 2, 0 };
        e[1] = (Entry){ 0, 1, 2, 1 };
    }
    if (c->h264) {
        e[0] = (Entry){ 0, 0, 5, 1 };
        e[1] = (Entry){ 0, 2, 5, 3 };
    }
    for (int f = 0; f < 3; f++) {
        setup(qts, f);
        run_list(qts, c->r, e, c->n, f ? 0x111e0000 : 0x211e0000,
                 0x00083f1f | (c->no_deblock ? 1u << 18 : 0), c->h264);
        g_assert_cmphex(rec_crc(dir, f, &len), ==, c->crc[f]);
        g_assert_cmpuint(len, ==, W * H * 3 / 2);
    }
    qtest_quit(qts);
    rm_dir(dir);
}

/*
 * Noise of a spread of 4 coded at QP 24, an I picture. The EL200 (a T41NQ,
 * 640x352) splits every 8x8 coding block of it into four 4x4 blocks (96% of
 * them) and codes 34.7 KB at 39.6 dB, where 32x32 blocks code 10.7 KB at
 * 37.1 dB. Pinned: the bytes of the slice (943, the camera's 0.154 a sample
 * is 946) and the reconstruction's squared error against the source (38.7
 * dB, the camera's 39.6). The EL150 (a T31, this picture itself injected
 * into its encoder, auto QP on) writes 997 bytes at 39.3 dB (a squared error
 * of 47251); the model without auto QP, 981 and 53368 (38.8 dB).
 */
static void noise_picture(uint8_t *y, uint8_t *c)
{
    uint32_t s = 12345;

    for (int i = 0; i < W * H; i++) {
        int v = 0;

        for (int k = 0; k < 4; k++) {
            s = s * 1664525 + 1013904223;
            v += s >> 29;
        }
        y[i] = 128 + v - 14;
    }
    memset(c, 128, W * H / 2);
}

static void test_noise(const void *data)
{
    const Rig *r = data;
    g_autofree char *dir = g_dir_make_tmp("ingenic-avpu-rec-XXXXXX", NULL);
    g_autofree char *args = g_strdup_printf("-machine %s "
        "-global ingenic-avpu.capture=%s", r->machine, dir);
    QTestState *qts = qtest_init(args);
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };
    g_autofree char *buf = NULL;
    const Entry e = { 0, 0, 2, 1 };
    g_autofree char *path = g_strdup_printf("%s/avpu-%06u.rec", dir, 0);
    uint8_t d[SLOT];
    uint64_t se = 0;
    uint32_t len;
    gsize n;

    noise_picture(y, uv);
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
    run_list(qts, r, &e, 1, 0x21180000, 0x00083f1f, false);
    len = slice_data(qts, r, 0, d);
    g_assert_true(g_file_get_contents(path, &buf, &n, NULL));
    g_assert_cmpuint(n, ==, W * H * 3 / 2);
    for (int i = 0; i < W * H; i++) {
        int dv = (uint8_t)buf[i] - y[i];

        se += dv * dv;
    }
    g_assert_cmpuint(len, ==, r->noise_len);
    g_assert_cmpuint(se, ==, r->noise_se);
    qtest_quit(qts);
    rm_dir(dir);
}

/*
 * P pictures of the EL200 with the motion search on: a picture given twice
 * is skipped throughout (the camera, a T41NQ, codes 9 to 20 bytes of 640x352
 * at QP 24 to 40; 38 here), and the pattern moved 2 samples is one vector
 * of merges and skips, the AMVP only where no merge candidate has it (91).
 * Pinned: the bytes of the P slices
 */
static uint32_t p_bytes(QTestState *qts, const Rig *r, int ref, int cur)
{
    uint8_t d[SLOT];
    const Entry e = { 0, 0, 2, 1 };

    setup(qts, ref);
    run_list(qts, r, &e, 1, 0x211e0000, 0x00083f1f, false);
    setup(qts, cur);
    run_list(qts, r, &e, 1, 0x111e0000, 0x00083f1f, false);
    return slice_data(qts, r, 0, d);
}

static void test_p(const void *data)
{
    const Rig *r = data;
    QTestState *qts = qtest_initf("-machine %s "
                                  "-global ingenic-avpu.h265-motion=search",
                                  r->machine);

    g_assert_cmpuint(p_bytes(qts, r, 0, 0), ==, 38);
    g_assert_cmpuint(p_bytes(qts, r, 0, 1), ==, 91);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int j = 0; j < ARRAY_SIZE(rigs); j++) {
        for (int i = 0; i < ARRAY_SIZE(cases); i++) {
            CaseRig *cr = g_new(CaseRig, 1);
            g_autofree char *path = g_strdup_printf(
                "/ingenic-avpu/hevc-slices/%s/%s", rigs[j].name,
                cases[i].name);

            *cr = (CaseRig){ &cases[i], &rigs[j] };
            qtest_add_data_func(path, cr, test_slices);
        }
    }
    for (int i = 0; i < ARRAY_SIZE(rec_cases); i++) {
        g_autofree char *path = g_strdup_printf(
            "/ingenic-avpu/hevc-slices/rec/%s/%d-slices%s%s",
            rec_cases[i].r->name, rec_cases[i].n,
            rec_cases[i].search ? "-search" : "",
            rec_cases[i].no_deblock ? "-unfiltered" :
            rec_cases[i].h264 ? "-h264" : "");

        qtest_add_data_func(path, &rec_cases[i], test_rec);
    }
    qtest_add_data_func("/ingenic-avpu/hevc-slices/el150/noise-4x4", &rigs[0],
                        test_noise);
    qtest_add_data_func("/ingenic-avpu/hevc-slices/el200/noise-4x4", &rigs[1],
                        test_noise);
    qtest_add_data_func("/ingenic-avpu/hevc-slices/el200/p-static-moved",
                        &rigs[1], test_p);
    return g_test_run();
}
