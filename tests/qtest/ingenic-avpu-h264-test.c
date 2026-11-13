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
    return g_test_run();
}
