/*
 * QTest testcase for the T31 AVPU's H.264 encoder against silicon
 *
 * Pictures injected into a T31's encoder (640 wide, the top rows only: the
 * coding is causal, so a crop's slice data is a prefix of the whole
 * picture's) and what its core wrote for them, in ingenic-avpu-h264-data.inc.
 *
 * I pictures: the macroblock kinds of silicon's coding are forced
 * (x-h264-types, which only tests use) and the slice data must be silicon's
 * byte for byte, up to the last bytes of the crop, where silicon went on
 * with the next macroblock; the model's own bytes are pinned by CRC. On its
 * own mode decision the model's kinds must agree with silicon's on at least
 * the macroblocks they did when the floor was set. The CAVLC picture has no
 * silicon behind it: the model's own bytes only.
 *
 * Where the search looks: noise squares, a macroblock each, moved by a known
 * shift between an IDR and a P picture. As measured on a T31, the whole
 * sample candidates are on a quincunx lattice (x + y even) inside 16 below
 * to 15 above the window's centre, so a shift along one axis by an odd
 * number of samples is not found, nor one outside the window.
 *
 * P pictures: an IDR coded with silicon's kinds, then two P pictures with
 * the motion search, as a guest's command lists have them. Silicon's
 * classes (skipped, coded, intra) and vectors, for the macroblocks whose
 * search window lies in the crop, against the capture's: the model's
 * agreement may not fall below what it was, and its bytes are pinned by CRC.
 * Every capture's macroblock kinds come from the capture directory
 * (capture=DIR) the model writes.
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
#define EP2             0x00940000
#define ROOM            0x10000

#define WIDTH           640
#define I_ROWS          64
#define P_ROWS          96
#define P_MBS           (WIDTH / 16 * 4)    /* the rows whose window fits */

typedef struct {
    const char *name;
    const char *pic;            /* NV12, base64 */
    uint32_t w[5];              /* entry words 0, 2, 3, 4, 9 */
    const char *types;          /* silicon's kind of each macroblock */
    const char *sil;            /* silicon's slice data, base64; or NULL */
    unsigned len, prefix;       /* the model's bytes; the match to silicon */
    uint32_t crc;
    unsigned free_kinds, free_len;  /* free running: kinds agreeing, bytes */
    uint32_t free_crc;
} IngenicI264Case;

typedef struct {
    int frame;
    uint32_t w[5];
    const char *vec;            /* silicon's vectors, 4 words a macroblock */
    const char *cls;            /* silicon's class: 0 intra, 3 coded, 4 skip */
    unsigned len;               /* the model's bytes */
    uint32_t crc;
    unsigned class_ok, both, mv_ok;
} IngenicP264Job;

typedef struct {
    const char *name;
    const char *idr_types;
    uint32_t idr[5];
    IngenicP264Job p[2];
} IngenicP264Case;

#include "ingenic-avpu-h264-data.inc"

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

/* The slice data without its emulation prevention bytes */
static size_t unescape(uint8_t *p, size_t n)
{
    size_t o = 0;
    int zeros = 0;

    for (size_t i = 0; i < n; i++) {
        if (zeros >= 2 && p[i] == 3) {
            zeros = 0;
            continue;
        }
        zeros = p[i] ? 0 : zeros + 1;
        p[o++] = p[i];
    }
    return o;
}

static size_t common_prefix(const uint8_t *a, const uint8_t *b, size_t n)
{
    size_t i = 0;

    while (i < n && a[i] == b[i]) {
        i++;
    }
    return i;
}

typedef struct {
    QTestState *qts;
    char *dir;                  /* the capture directory */
    unsigned seq;
} Rig;

static Rig *rig_new(const char *types, bool search)
{
    Rig *r = g_new0(Rig, 1);
    g_autoptr(GError) err = NULL;
    g_autofree char *args = NULL;

    r->dir = g_dir_make_tmp("ingenic-avpu-h264-XXXXXX", &err);
    g_assert_no_error(err);
    args = g_strdup_printf("-machine ingenic-t31 "
                           "-global ingenic-avpu.capture=%s "
                           "%s%s %s%s", r->dir,
                           types ? "-global ingenic-avpu.x-h264-types=" : "",
                           types ?: "",
                           search ? "-global ingenic-avpu.h264-motion=" : "",
                           search ? "search" : "");
    r->qts = qtest_init(args);
    return r;
}

static void rig_free(Rig *r)
{
    GDir *d = g_dir_open(r->dir, 0, NULL);
    const char *f;

    while (d && (f = g_dir_read_name(d))) {
        g_autofree char *p = g_build_filename(r->dir, f, NULL);

        unlink(p);
    }
    if (d) {
        g_dir_close(d);
    }
    rmdir(r->dir);
    qtest_quit(r->qts);
    g_free(r->dir);
    g_free(r);
}

/*
 * One H.264 job of the picture at @pic (@rows rows of NV12, as written to
 * the capture), with the entry words of the capture. Returns the slice data
 * (without emulation prevention) and fills @kinds with the macroblock kinds
 * the model reports and, with @mv, the motion vector buffer.
 */
static size_t encode(Rig *r, const uint8_t *pic, int rows,
                     const uint32_t w5[5], uint8_t *out, uint8_t *kinds,
                     uint32_t *mv)
{
    QTestState *qts = r->qts;
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    uint8_t ep2[0x100] = { [19] = 51 };
    size_t ysize = (size_t)WIDTH * rows, n;
    g_autofree char *path = NULL;
    g_autofree char *txt = NULL;
    const char *k;
    uint32_t len;

    qtest_memwrite(qts, SRC_Y, pic, ysize);
    qtest_memwrite(qts, SRC_C, pic + ysize, ysize / 2);
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
    w[0] = w5[0];
    w[1] = (WIDTH / 8 - 1) | (rows / 8 - 1) << 12;
    w[2] = w5[1];
    w[3] = w5[2];
    w[4] = w5[3];
    w[9] = w5[4];
    w[0x20] = SRC_Y;
    w[0x21] = SRC_C;
    w[0x22] = WIDTH;
    w[0x23] = EP2;
    w[0x2e] = mv ? MV : 0;
    w[0x30] = BS;
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
    qtest_memread(qts, BS, out, len);
    n = unescape(out, len);
    if (mv) {
        qtest_memread(qts, MV, mv, WIDTH / 16 * rows / 16 * 32);
        for (int i = 0; i < WIDTH / 16 * rows / 16 * 8; i++) {
            mv[i] = le32_to_cpu(mv[i]);
        }
    }

    path = g_strdup_printf("%s/avpu-%06u.txt", r->dir, r->seq++);
    g_assert_true(g_file_get_contents(path, &txt, NULL, NULL));
    k = strstr(txt, "\nmb_kind ");
    g_assert_nonnull(k);
    k += strlen("\nmb_kind ");
    for (int i = 0; i < WIDTH / 16 * rows / 16; i++) {
        kinds[i] = g_ascii_xdigit_value(k[i]);
    }
    return n;
}

static void test_intra(const void *data)
{
    const IngenicI264Case *c = data;
    size_t picsize = WIDTH * I_ROWS * 3 / 2, sillen = 0, n;
    int nmb = WIDTH / 16 * I_ROWS / 16;
    g_autofree uint8_t *pic = NULL;
    g_autofree uint8_t *sil = NULL;
    uint8_t out[ROOM], kinds[WIDTH / 16 * I_ROWS / 16];
    gsize len;
    Rig *r;

    pic = g_base64_decode(c->pic, &len);
    g_assert_cmpuint(len, ==, picsize);
    g_assert_cmpuint(strlen(c->types), ==, nmb);

    /* silicon's kinds: its bytes */
    r = rig_new(c->types, false);
    n = encode(r, pic, I_ROWS, c->w, out, kinds, NULL);
    g_assert_cmpuint(n, ==, c->len);
    g_assert_cmphex(crc32_le(out, n), ==, c->crc);
    for (int i = 0; i < nmb; i++) {
        g_assert_cmpuint(kinds[i], ==, c->types[i] - '0');
    }
    if (c->sil) {
        sil = g_base64_decode(c->sil, &sillen);
        g_assert_cmpuint(sillen, >=, n);
        g_assert_cmpuint(common_prefix(out, sil, n), >=, c->prefix);
    }
    rig_free(r);

    if (!c->free_len) {
        return;
    }

    /* its own decision: the model's bytes, and how often silicon's kind */
    r = rig_new(NULL, false);
    n = encode(r, pic, I_ROWS, c->w, out, kinds, NULL);
    g_assert_cmpuint(n, ==, c->free_len);
    g_assert_cmphex(crc32_le(out, n), ==, c->free_crc);
    len = 0;
    for (int i = 0; i < nmb; i++) {
        len += kinds[i] == c->types[i] - '0';
    }
    g_assert_cmpuint(len, >=, c->free_kinds);
    rig_free(r);
}

static void test_inter(const void *data)
{
    const IngenicP264Case *c = data;
    size_t picsize = WIDTH * P_ROWS * 3 / 2;
    int nmb = WIDTH / 16 * P_ROWS / 16;
    g_autofree uint8_t *pic[3] = { NULL };
    uint8_t out[ROOM], kinds[WIDTH / 16 * P_ROWS / 16];
    uint32_t mv[WIDTH / 16 * P_ROWS / 16 * 8];
    gsize len;
    Rig *r;

    for (int i = 0; i < 3; i++) {
        pic[i] = g_base64_decode(frames[i], &len);
        g_assert_cmpuint(len, ==, picsize);
    }
    r = rig_new(c->idr_types, true);
    encode(r, pic[0], P_ROWS, c->idr, out, kinds, mv);

    for (int j = 0; j < 2; j++) {
        const IngenicP264Job *p = &c->p[j];
        g_autofree uint32_t *vec = NULL;
        unsigned cls_ok = 0, both = 0, mv_ok = 0;
        size_t n;

        vec = (uint32_t *)g_base64_decode(p->vec, &len);
        g_assert_cmpuint(len, ==, P_MBS * 16);
        g_assert_cmpuint(strlen(p->cls), ==, P_MBS);
        n = encode(r, pic[p->frame], P_ROWS, p->w, out, kinds, mv);
        g_assert_cmpuint(n, ==, p->len);
        g_assert_cmphex(crc32_le(out, n), ==, p->crc);

        for (int mb = 0; mb < P_MBS; mb++) {
            int k = kinds[mb] & 7;
            int mine = k <= 2 ? 0 : k == 4 ? 4 : 3;
            int theirs = p->cls[mb] - '0';

            cls_ok += mine == theirs;
            if (mine && theirs) {
                both++;
                mv_ok += mv[mb * 8] == le32_to_cpu(vec[mb * 4]) &&
                         mv[mb * 8 + 2] == le32_to_cpu(vec[mb * 4 + 1]) &&
                         mv[mb * 8 + 4] == le32_to_cpu(vec[mb * 4 + 2]) &&
                         mv[mb * 8 + 6] == le32_to_cpu(vec[mb * 4 + 3]);
            }
        }
        g_test_message("P+%d: class %u/%d, vectors %u/%u", p->frame, cls_ok,
                       P_MBS, mv_ok, both);
        g_assert_cmpuint(cls_ok, >=, p->class_ok);
        g_assert_cmpuint(mv_ok, >=, p->mv_ok);
    }
    g_assert_cmpuint(nmb, ==, WIDTH / 16 * P_ROWS / 16);
    rig_free(r);
}

/*
 * Noise squares at macroblocks (3 + 7 i, 2), 640x96 NV12 around flat grey:
 * @cur has them where they are, @ref has them moved by (sx, sy) with the
 * same samples, so the vector that finds them is (sx, sy)
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
    memset(ref, 128, WIDTH * P_ROWS * 3 / 2);
    memset(cur, 128, WIDTH * P_ROWS * 3 / 2);
    for (int i = 0; i < 5; i++) {
        for (int y = 0; y < 16; y++) {
            for (int x = 0; x < 16; x++) {
                int px = (3 + 7 * i) * 16 + x, py = 2 * 16 + y;

                cur[py * WIDTH + px] = noise(px, py);
                ref[(py + sy) * WIDTH + px + sx] = noise(px, py);
            }
        }
    }
}

typedef struct {
    int sx, sy;
    bool found;                 /* all five squares get exactly (sx, sy) */
} IngenicLatticeCase;

static const IngenicLatticeCase lcases[] = {
    { 4, 0, true }, { 14, 0, true }, { -16, 0, true }, { 15, 15, true },
    { 0, -16, true }, { 3, 0, false }, { 0, 5, false }, { 16, 0, false },
    { 0, 16, false }, { -17, 1, false },
};

static void test_lattice(const void *data)
{
    const IngenicLatticeCase *c = data;
    static const uint32_t idr[5] = { 0x80700411, 0x4010ad50, 0x211e0000,
                                     0x00083f1f, 0xf8000000 };
    static const uint32_t pw[5] = { 0x80700411, 0x4010ad50, 0x111e0000,
                                    0x00083f1f, 0xf8000000 };
    g_autofree uint8_t *ref = g_malloc(WIDTH * P_ROWS * 3 / 2);
    g_autofree uint8_t *cur = g_malloc(WIDTH * P_ROWS * 3 / 2);
    g_autofree char *types = g_strnfill(WIDTH / 16 * P_ROWS / 16, '2');
    uint8_t out[ROOM], kinds[WIDTH / 16 * P_ROWS / 16];
    uint32_t mv[WIDTH / 16 * P_ROWS / 16 * 8];
    uint32_t want = (c->sx * 4 & 0x3fff) | (c->sy * 4 & 0xfff) << 14;
    int hit = 0;
    Rig *r;

    lattice_pics(ref, cur, c->sx, c->sy);
    r = rig_new(types, true);
    encode(r, ref, P_ROWS, idr, out, kinds, mv);
    encode(r, cur, P_ROWS, pw, out, kinds, mv);
    for (int i = 0; i < 5; i++) {
        int mb = 2 * (WIDTH / 16) + 3 + 7 * i;

        hit += (kinds[mb] & 7) > 2 && mv[mb * 8] == want &&
               mv[mb * 8 + 6] == want;
    }
    g_test_message("(%d, %d): %d of 5", c->sx, c->sy, hit);
    if (c->found) {
        g_assert_cmpint(hit, ==, 5);
    } else {
        g_assert_cmpint(hit, ==, 0);
    }
    rig_free(r);
}

/*
 * Where the last picture had a noise square and this one is flat grey,
 * silicon codes the square's macroblocks intra, though a vector into the
 * flat grey beside it matches them to the sample (no level at all): the
 * model must try intra there too. A level offset of a macroblock is coded
 * where zero-mean noise on it is not.
 */
static void test_flat(void)
{
    static const uint32_t idr[5] = { 0x80700411, 0x4010ad50, 0x211e0000,
                                     0x00083f1f, 0xf8000000 };
    static const uint32_t pw[5] = { 0x80700411, 0x4010ad50, 0x111e0000,
                                    0x00083f1f, 0xf8000000 };
    g_autofree uint8_t *ref = g_malloc(WIDTH * P_ROWS * 3 / 2);
    g_autofree uint8_t *cur = g_malloc(WIDTH * P_ROWS * 3 / 2);
    g_autofree char *types = g_strnfill(WIDTH / 16 * P_ROWS / 16, '2');
    uint8_t out[ROOM], kinds[WIDTH / 16 * P_ROWS / 16];
    uint32_t mv[WIDTH / 16 * P_ROWS / 16 * 8];
    int coded = 0;
    Rig *r;

    memset(ref, 128, WIDTH * P_ROWS * 3 / 2);
    memset(cur, 128, WIDTH * P_ROWS * 3 / 2);
    for (int y = 16; y < 64; y++) {
        for (int x = 4 * 16; x < 7 * 16; x++) {
            ref[y * WIDTH + x] = noise(x, y);
        }
    }
    /* macroblocks (20, 4) and (21, 4): a level offset and zero-mean noise */
    for (int y = 4 * 16; y < 5 * 16; y++) {
        for (int x = 20 * 16; x < 21 * 16; x++) {
            cur[y * WIDTH + x] = 128 + 6;
        }
        for (int x = 21 * 16; x < 22 * 16; x++) {
            cur[y * WIDTH + x] = 128 + (noise(x, y) & 1 ? 2 : -2);
        }
    }
    r = rig_new(types, true);
    encode(r, ref, P_ROWS, idr, out, kinds, mv);
    encode(r, cur, P_ROWS, pw, out, kinds, mv);
    for (int my = 1; my < 4; my++) {
        for (int mx = 4; mx < 7; mx++) {
            coded += (kinds[my * (WIDTH / 16) + mx] & 7) <= 2;
        }
    }
    g_test_message("square: %d of 9 intra", coded);
    g_assert_cmpint(coded, ==, 9);
    g_assert_cmpint(kinds[4 * (WIDTH / 16) + 20] & 7, !=, 4);
    g_assert_cmpint(kinds[4 * (WIDTH / 16) + 21] & 7, ==, 4);
    rig_free(r);
}

/*
 * A flat macroblock beside a noise block that stays as it was is a few levels
 * off the last picture, where the deblocking of the noise spilled into it.
 * Silicon (a T31X, QP 30) skips it while the picture has coded few
 * intra macroblocks before it, and codes it intra after a row of them (here
 * the row above, flat over a noise row): the model must do the same, and
 * skip it where there is no such row.
 */
static void test_floor(void)
{
    static const uint32_t idr[5] = { 0x80700411, 0x4010ad50, 0x211e0000,
                                     0x00083f1f, 0xf8000000 };
    static const uint32_t pw[5] = { 0x80700411, 0x4010ad50, 0x111e0000,
                                    0x00083f1f, 0xf8000000 };
    static const int cols[4] = { 5, 15, 25, 35 };
    g_autofree uint8_t *ref = g_malloc(WIDTH * P_ROWS * 3 / 2);
    g_autofree uint8_t *cur = g_malloc(WIDTH * P_ROWS * 3 / 2);
    g_autofree char *types = g_strnfill(WIDTH / 16 * P_ROWS / 16, '2');
    uint8_t out[ROOM], kinds[WIDTH / 16 * P_ROWS / 16];
    uint32_t mv[WIDTH / 16 * P_ROWS / 16 * 8];

    for (int row = 0; row < 2; row++) {
        int intra = 0, skip = 0;
        Rig *r;

        memset(ref, 128, WIDTH * P_ROWS * 3 / 2);
        memset(cur, 128, WIDTH * P_ROWS * 3 / 2);
        for (int i = 0; i < 4; i++) {
            for (int y = 3 * 16; y < 4 * 16; y++) {
                for (int x = cols[i] * 16; x < (cols[i] + 1) * 16; x++) {
                    ref[y * WIDTH + x] = cur[y * WIDTH + x] = noise(x, y);
                }
            }
        }
        for (int y = 0; row == 0 && y < 16; y++) {
            for (int x = 0; x < WIDTH; x++) {
                ref[y * WIDTH + x] = noise(x, y);
            }
        }
        r = rig_new(types, true);
        encode(r, ref, P_ROWS, idr, out, kinds, mv);
        encode(r, cur, P_ROWS, pw, out, kinds, mv);
        for (int i = 0; i < 4; i++) {
            int k = kinds[2 * (WIDTH / 16) + cols[i]] & 7;

            intra += k <= 2;
            skip += k == 4;
        }
        g_test_message("%s: %d intra, %d skipped of 4",
                       row ? "no noise row" : "noise row above", intra, skip);
        g_assert_cmpint(row ? skip : intra, ==, 4);
        rig_free(r);
    }
}

/*
 * Four squares of 5x5 macroblocks, one every 9 macroblocks, move by (12, 0)
 * from the first picture to the second, so the vector (12, 0) is the last
 * one at their centres. In the third picture only the centre macroblock of
 * each is noise, the second picture's at 24 samples along: the window round
 * the last vector reaches it, the one round zero does not (the window is 16
 * samples either side of its centre). Silicon (a T31X) scans round zero alone
 * where the macroblock four to the left was skipped, which is so for all but
 * the first square (a macroblock outside the row does not count), so it finds
 * one of the four.
 */
static void test_window(void)
{
    static const uint32_t idr[5] = { 0x80700411, 0x4010ad50, 0x211e0000,
                                     0x00083f1f, 0xf8000000 };
    static const uint32_t pw[5] = { 0x80700411, 0x4010ad50, 0x111e0000,
                                    0x00083f1f, 0xf8000000 };
    size_t size = WIDTH * P_ROWS * 3 / 2;
    g_autofree uint8_t *a = g_malloc(size), *b = g_malloc(size);
    g_autofree uint8_t *c = g_malloc(size);
    g_autofree char *types = g_strnfill(WIDTH / 16 * P_ROWS / 16, '2');
    uint8_t out[ROOM], kinds[WIDTH / 16 * P_ROWS / 16];
    uint32_t mv[WIDTH / 16 * P_ROWS / 16 * 8];
    uint32_t want = 24 * 4;
    int hit[4] = { 0 }, total = 0;
    Rig *r;

    memset(a, 128, size);
    memset(b, 128, size);
    memset(c, 128, size);
    for (int i = 0; i < 4; i++) {
        int x0 = (3 + 9 * i - 2) * 16;

        for (int y = 0; y < 5 * 16; y++) {
            for (int x = x0; x < x0 + 5 * 16; x++) {
                b[y * WIDTH + x] = noise(x, y);
                a[y * WIDTH + x + 12] = noise(x, y);
            }
        }
        for (int y = 2 * 16; y < 3 * 16; y++) {
            for (int x = (3 + 9 * i) * 16; x < (4 + 9 * i) * 16; x++) {
                c[y * WIDTH + x] = noise(x + 24, y);
            }
        }
    }
    r = rig_new(types, true);
    encode(r, a, P_ROWS, idr, out, kinds, mv);
    encode(r, b, P_ROWS, pw, out, kinds, mv);
    encode(r, c, P_ROWS, pw, out, kinds, mv);
    for (int i = 0; i < 4; i++) {
        int mb = 2 * (WIDTH / 16) + 3 + 9 * i;

        hit[i] = (kinds[mb] & 7) > 2 && mv[mb * 8] == want &&
                 mv[mb * 8 + 6] == want;
        total += hit[i];
    }
    g_test_message("found: %d %d %d %d", hit[0], hit[1], hit[2], hit[3]);
    g_assert_cmpint(hit[0], ==, 1);
    g_assert_cmpint(total, ==, 1);
    rig_free(r);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(icases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/avpu/el150/"
                                                "h264-intra/%s",
                                                icases[i].name);

        qtest_add_data_func(path, &icases[i], test_intra);
    }
    for (int i = 0; i < ARRAY_SIZE(pcases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/avpu/el150/"
                                                "h264-inter/%s",
                                                pcases[i].name);

        qtest_add_data_func(path, &pcases[i], test_inter);
    }
    for (int i = 0; i < ARRAY_SIZE(lcases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/avpu/el150/"
                                                "h264-lattice/%d,%d",
                                                lcases[i].sx, lcases[i].sy);

        qtest_add_data_func(path, &lcases[i], test_lattice);
    }
    qtest_add_func("ingenic/avpu/el150/h264-flat/square", test_flat);
    qtest_add_func("ingenic/avpu/el150/h264-flat/floor", test_floor);
    qtest_add_func("ingenic/avpu/el150/h264-window/zero", test_window);
    return g_test_run();
}
