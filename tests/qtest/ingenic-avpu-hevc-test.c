/*
 * QTest testcase for the T31 AVPU's H.265 motion search against silicon
 *
 * Pictures injected into a T31's encoder (640 wide, the top 96 rows: the
 * coding is causal, so a crop's coding is what the whole picture's starts
 * with) and the motion vector records its core wrote, in
 * ingenic-avpu-hevc-data.inc: an IDR of flat grey, a picture of noise
 * squares (one 16x16 block in each 48x48), the same again, and the same
 * squares moved by a known shift, as the camera had them. With
 * h265-motion=search the model's record of each 16x16 unit is compared with
 * silicon's: how often the unit is inter and not skipped in both, and how
 * often the vectors are equal; they may not fall below what they were when
 * the floors were set. The model's own stream is pinned by CRC, and the
 * same pictures with the search off, which are what the stand-in's
 * coding of them has always been.
 *
 * Where the search looks, with pictures made here: noise squares moved by a
 * shift. On silicon every whole-sample shift from 32 below to 31 above the
 * window's centre is found, along either axis, and one beyond it is not.
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

#define SRC_Y           0x00800000
#define SRC_C           0x00880000
#define LIST            0x00900000
#define BS              0x00910000
#define MV              0x00930000
#define HDR             0x220
#define ROOM            0x10000

#define WIDTH           640
#define ROWS            96
#define UNITS           (WIDTH / 32 * (ROWS / 32) * 4)

typedef struct {
    unsigned len;
    uint32_t crc;
    unsigned cls, both, mv, tgt;    /* agreement with silicon, and squares */
} IngenicHevcJob;

typedef struct {
    const char *name;
    int sx, sy;                 /* the shift, in quarter samples */
    const char *x, *t;          /* the pictures, NV12 base64 */
    /*
     * The pictures in the order of the jobs: F flat grey, X the squares,
     * x the same, t moved; a capital is an IDR (the camera's own second IDR
     * came after the first job or two). Silicon's records of the last P
     * jobs
     */
    const char *seq;
    const char *rec[3];
    IngenicHevcJob job[3];
    struct {
        unsigned len;
        uint32_t crc;
    } off[4];                   /* each job with the search off */
} IngenicHevcCase;

#include "ingenic-avpu-hevc-data.inc"

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

/* The command list words of the camera's IDR and P pictures at QP 30 */
static const uint32_t idr_w[5] = { 0x91700c11, 0xc010ad50, 0x211e0000,
                                   0x00083f1f, 0xf8048000 };
static const uint32_t p_w[5] = { 0x91700c11, 0xc090ad50, 0x111e0000,
                                 0x00083f1f, 0xf8048000 };

/* One job of the picture at @pic (ROWS of NV12): the stream, and the records */
static size_t encode(QTestState *qts, const uint8_t *pic, const uint32_t w5[5],
                     uint8_t *out, uint32_t *mv)
{
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    size_t ysize = (size_t)WIDTH * ROWS;
    uint32_t len;

    qtest_memwrite(qts, SRC_Y, pic, ysize);
    qtest_memwrite(qts, SRC_C, pic + ysize, ysize / 2);
    w[0] = w5[0];
    w[1] = (WIDTH / 8 - 1) | (ROWS / 8 - 1) << 12;
    w[2] = w5[1];
    w[3] = w5[2];
    w[4] = w5[3];
    w[9] = w5[4];
    w[0x20] = SRC_Y;
    w[0x21] = SRC_C;
    w[0x22] = WIDTH;
    w[0x2e] = mv ? MV : 0;
    w[0x30] = BS;
    w[0x32] = HDR;
    w[0x33] = ROOM;
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    qtest_writel(qts, BASE + CL_ADDR, LIST);
    qtest_writel(qts, BASE + START, 2);
    for (int i = 0; i < 40000 && !(qtest_readl(qts, BASE + IRQ) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, BASE + IRQ) & 1, ==, 1);
    qtest_writel(qts, BASE + IRQ, 1);
    len = qtest_readl(qts, LIST + 0x41 * 4);
    g_assert_cmpuint(len, >, 0);
    g_assert_cmpuint(len, <, ROOM);
    qtest_memread(qts, BS + HDR, out, len);
    if (mv) {
        qtest_memread(qts, MV, mv, UNITS * 16);
        for (int i = 0; i < UNITS * 4; i++) {
            mv[i] = le32_to_cpu(mv[i]);
        }
    }
    return len;
}

/* The unit of macroblock row @rr and column @cc: records go by CTB, in Z */
static int unit(int rr, int cc)
{
    return ((rr / 2) * (WIDTH / 32) + cc / 2) * 4 + (rr & 1) * 2 + (cc & 1);
}

static void replay(const IngenicHevcCase *c, bool search)
{
    size_t picsize = WIDTH * ROWS * 3 / 2;
    g_autofree uint8_t *flat = g_malloc(picsize);
    g_autofree uint8_t *x = NULL, *t = NULL;
    uint8_t out[ROOM];
    uint32_t mv[UNITS * 4];
    uint32_t want = (c->sx & 0xffff) | (uint32_t)(c->sy & 0xffff) << 16;
    int jk = 0;
    gsize len;
    QTestState *qts;

    memset(flat, 128, picsize);
    x = g_base64_decode(c->x, &len);
    g_assert_cmpuint(len, ==, picsize);
    t = g_base64_decode(c->t, &len);
    g_assert_cmpuint(len, ==, picsize);
    qts = qtest_init(search ? "-machine ingenic-t31 "
                              "-global ingenic-avpu.h265-motion=search"
                            : "-machine ingenic-t31");
    for (int k = 0; c->seq[k]; k++) {
        char ch = c->seq[k];
        bool idr = g_ascii_isupper(ch);
        const uint8_t *pic = ch == 'F' ? flat : g_ascii_tolower(ch) == 'x' ?
                             x : t;
        bool pin = search && !idr;
        size_t n = encode(qts, pic, idr ? idr_w : p_w, out, pin ? mv : NULL);

        if (!pin) {
            g_test_message("%s %s %d: %u bytes, crc %08x", c->name,
                           search ? "idr" : "off", k, (unsigned)n,
                           crc32_le(out, n));
            g_assert_cmpuint(n, ==, c->off[k].len);
            g_assert_cmphex(crc32_le(out, n), ==, c->off[k].crc);
        } else {
            const IngenicHevcJob *j = &c->job[jk];
            g_autofree uint32_t *sil = NULL;
            unsigned cls = 0, both = 0, ok = 0, tgt = 0;

            sil = (uint32_t *)g_base64_decode(c->rec[jk], &len);
            g_assert_cmpuint(len, ==, UNITS * 16);
            for (int u = 0; u < UNITS; u++) {
                bool mine = mv[u * 4 + 2], theirs = le32_to_cpu(sil[u * 4 + 2]);

                cls += mine == theirs;
                if (mine && theirs) {
                    both++;
                    ok += mv[u * 4] == le32_to_cpu(sil[u * 4]);
                }
            }
            for (int rr = 1; rr < ROWS / 16; rr += 3) {
                for (int cc = 1; cc < WIDTH / 16; cc += 3) {
                    tgt += ch == 't' && mv[unit(rr, cc) * 4] == want;
                }
            }
            g_test_message("%s job %d: %u bytes, crc %08x, class %u/%d, "
                           "vectors %u/%u, squares %u", c->name, jk,
                           (unsigned)n, crc32_le(out, n), cls, UNITS, ok,
                           both, tgt);
            g_assert_cmpuint(n, ==, j->len);
            g_assert_cmphex(crc32_le(out, n), ==, j->crc);
            g_assert_cmpuint(cls, >=, j->cls);
            g_assert_cmpuint(ok, >=, j->mv);
            g_assert_cmpuint(tgt, >=, j->tgt);
            jk++;
        }
    }
    qtest_quit(qts);
}

static void test_replay(const void *data)
{
    replay(data, true);
}

static void test_off(const void *data)
{
    replay(data, false);
}

/*
 * Noise squares at macroblocks (1 + 3 i, 1 + 3 k), 640x96 NV12 around flat
 * grey: @cur has them where they are, @ref has them moved by (sx, sy) with
 * the same samples, so the vector that finds them is (sx, sy)
 */
static uint8_t noise(int x, int y)
{
    uint32_t h = x * 73856093u ^ y * 19349663u;

    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return 88 + (h & 0x7f) % 80;
}

static void lattice_pics(uint8_t *ref, uint8_t *cur, int sx, int sy)
{
    memset(ref, 128, WIDTH * ROWS * 3 / 2);
    memset(cur, 128, WIDTH * ROWS * 3 / 2);
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < 13; i++) {
            for (int y = 0; y < 16; y++) {
                for (int x = 0; x < 16; x++) {
                    int px = (1 + 3 * i) * 16 + x, py = (1 + 3 * k) * 16 + y;

                    cur[py * WIDTH + px] = noise(px, py);
                    if (px + sx >= 0 && px + sx < WIDTH && py + sy >= 0 &&
                        py + sy < ROWS) {
                        ref[(py + sy) * WIDTH + px + sx] = noise(px, py);
                    }
                }
            }
        }
    }
}

typedef struct {
    int sx, sy;                 /* whole samples */
    bool found;                 /* most squares get exactly (sx, sy) */
} IngenicLatticeCase;

static const IngenicLatticeCase lcases[] = {
    { 8, 0, true }, { -24, 0, true }, { 30, 0, true }, { -30, 0, true },
    { 0, 16, true }, { 0, -16, true }, { 6, -4, true }, { 7, 0, true },
    { 50, 0, false }, { -60, 0, false },
};

static void test_lattice(const void *data)
{
    const IngenicLatticeCase *c = data;
    g_autofree uint8_t *ref = g_malloc(WIDTH * ROWS * 3 / 2);
    g_autofree uint8_t *cur = g_malloc(WIDTH * ROWS * 3 / 2);
    uint8_t out[ROOM];
    uint32_t mv[UNITS * 4];
    uint32_t want = (c->sx * 4 & 0xffff) | (uint32_t)(c->sy * 4 & 0xffff) << 16;
    int hit = 0;
    QTestState *qts;

    lattice_pics(ref, cur, c->sx, c->sy);
    qts = qtest_init("-machine ingenic-t31 "
                     "-global ingenic-avpu.h265-motion=search");
    encode(qts, ref, idr_w, out, NULL);
    encode(qts, cur, p_w, out, mv);
    for (int k = 0; k < 2; k++) {
        for (int i = 0; i < 13; i++) {
            hit += mv[unit(1 + 3 * k, 1 + 3 * i) * 4] == want;
        }
    }
    g_test_message("(%d, %d): %d of 26", c->sx, c->sy, hit);
    if (c->found) {
        g_assert_cmpint(hit, >=, 20);
    } else {
        g_assert_cmpint(hit, <=, 2);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(hcases); i++) {
        g_autofree char *p = g_strdup_printf("ingenic/avpu/el150/h265-motion/"
                                             "replay-%s", hcases[i].name);
        g_autofree char *o = g_strdup_printf("ingenic/avpu/el150/h265-motion/"
                                             "off-%s", hcases[i].name);

        qtest_add_data_func(p, &hcases[i], test_replay);
        qtest_add_data_func(o, &hcases[i], test_off);
    }
    for (int i = 0; i < ARRAY_SIZE(lcases); i++) {
        g_autofree char *p = g_strdup_printf("ingenic/avpu/el150/h265-motion/"
                                             "window-%d,%d", lcases[i].sx,
                                             lcases[i].sy);

        qtest_add_data_func(p, &lcases[i], test_lattice);
    }
    return g_test_run();
}
