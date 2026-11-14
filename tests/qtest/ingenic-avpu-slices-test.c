/*
 * QTest testcase for the T31 AVPU's command lists of several slices
 *
 * A synthetic 96x64 picture coded as H.264 intra slices of whole macroblock
 * rows, one command list entry each (words 5 and 6 the first and last
 * block's position, 0x1e its blocks less one, 0x30-0x33 where its data
 * goes), against what a T31's core wrote for the same command lists
 * (ingenic-avpu-slices-data.inc): each slice's data byte for byte, and the
 * status words of its entry. One interrupt ends the list. A slice codes as a
 * picture of its own: it never refers to the rows of another. A P picture of
 * the same picture, searched for motion, is skipped throughout as silicon
 * does and costs each slice a few bytes.
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
#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define LIST            0x00820000
#define EP2             0x00860000
#define BS              0x00840000
#define SLOT            0x1000      /* stream bytes a slice has */
#define HDR             0x220
#define ENTRY           0x200
#define STATUS          0x40

typedef struct {
    unsigned first, rows, len;
    uint32_t crc;
    unsigned tail_bits;     /* of the slice header the core writes first */
    uint32_t tail;
    uint32_t st[8];     /* words 0x42 0x45 0x47 0x48 0x49 0x4c 0x4d 0x4e */
} SliceWant;

typedef struct {
    const char *name;
    uint32_t w[5];      /* entry words 0, 2, 3, 4 and 9 */
    int n;
    SliceWant slice[8];
} SliceCase;

#include "ingenic-avpu-slices-data.inc"

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

static void picture(uint8_t *y, uint8_t *c)
{
    picture_at(y, c, 0);
}

static void run_list(QTestState *qts, const SliceCase *c, uint32_t w3)
{
    g_autofree uint32_t *list = g_new0(uint32_t, ENTRY / 4 * c->n);
    int mbw = W / 16;

    for (int k = 0; k < c->n; k++) {
        uint32_t *w = list + k * ENTRY / 4;
        const SliceWant *s = &c->slice[k];
        unsigned last = s->first + s->rows - 1;

        w[0] = c->w[0] | (k == c->n - 1 ? 1u << 31 : 0);
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
        w[2] = c->w[1];
        w[3] = w3;
        w[4] = c->w[3];
        w[5] = s->first << 12;
        w[6] = (mbw - 1) | last << 12;
        w[7] = 1u << 31 | (mbw - 1) | (H / 16 - 1) << 12;
        w[9] = c->w[4];
        w[0xb] = (mbw - 1) << 12;
        w[0x1e] = s->tail_bits << 24 | (s->rows * mbw - 1);
        w[0x1f] = s->tail;
        w[0x1d] = w[0xb];
        w[0x20] = SRC_Y;
        w[0x21] = SRC_C;
        w[0x22] = W;
        w[0x23] = EP2;
        w[0x30] = BS;
        w[0x32] = HDR + k * SLOT;
        w[0x33] = SLOT - HDR;
    }
    for (int i = 0; i < ENTRY / 4 * c->n; i++) {
        list[i] = cpu_to_le32(list[i]);
    }
    qtest_memwrite(qts, LIST, list, ENTRY * c->n);
    qtest_writel(qts, BASE + CL_ADDR, LIST);
    qtest_writel(qts, BASE + START, 2);
    for (int i = 0; i < 40000 && !(qtest_readl(qts, BASE + IRQ) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, BASE + IRQ) & 1, ==, 1);
    qtest_writel(qts, BASE + IRQ, 1);
}

static void test_slices(const void *data)
{
    const SliceCase *c = data;
    bool p = (c->w[2] >> 28 & 3) == 1;
    QTestState *qts = qtest_init(p ? "-machine ingenic-t31 "
                                     "-global ingenic-avpu.h264-motion=search"
                                   : "-machine ingenic-t31");
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };

    picture(y, uv);
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
    if (p) {
        /* a P picture refers to the one before: the same, as an I picture */
        run_list(qts, c, (c->w[2] & ~(3u << 28)) | 2u << 28);
    }
    run_list(qts, c, c->w[2]);

    for (int k = 0; k < c->n; k++) {
        const SliceWant *s = &c->slice[k];
        uint32_t base = LIST + k * ENTRY + STATUS * 4, len;
        uint8_t out[SLOT];
        static const int word[8] = { 2, 5, 7, 8, 9, 12, 13, 14 };

        len = qtest_readl(qts, base + 4);
        g_assert_cmpuint(len, ==, s->len);
        qtest_memread(qts, BS + HDR + k * SLOT, out, len);
        g_assert_cmphex(crc32_le(out, len), ==, s->crc);
        for (int i = 0; i < 8; i++) {
            g_assert_cmphex(qtest_readl(qts, base + word[i] * 4), ==,
                            s->st[i]);
        }
    }
    qtest_quit(qts);
}

/*
 * The capture's reconstruction (capture=DIR, <name>-<seq>.rec) of a
 * sequence of pictures as one slice and as two: an I picture, then P pictures
 * of the pattern moved by the motion search. The CRCs are those of the
 * pictures an independent decoder (libavcodec, given the stream as a
 * decoder gets it: parameter sets and slice headers written by hand ahead of
 * the model's slice data) puts out for the captured streams, taken from the
 * model's NV12 frame by frame. Pictures of a still pattern, as P pictures
 * skipped throughout, leave the reconstruction as it was.
 */
typedef struct {
    int n;                      /* slices */
    int pics;
    uint32_t crc[3];
} RecCase;

static const RecCase rec_cases[] = {
    { 1, 3, { 0xaf1ba606, 0xb7583081, 0x76dafa76 } },
    { 2, 2, { 0x874ff146, 0x988ca979 } },
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
        "-global ingenic-avpu.capture=%s "
        "-global ingenic-avpu.h264-motion=search", dir);
    QTestState *qts = qtest_init(args);
    SliceCase sc = { .w = { 0x00700011, 0x4010ad50, 0x211e0000, 0x00083f1f,
                            0x38000000 }, .n = c->n };
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };
    uint32_t len;

    for (int i = 0; i < c->n; i++) {
        sc.slice[i].first = i * H / 16 / c->n;
        sc.slice[i].rows = H / 16 / c->n;
    }
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
    for (int f = 0; f < c->pics; f++) {
        picture_at(y, uv, f);
        qtest_memwrite(qts, SRC_Y, y, sizeof(y));
        qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
        run_list(qts, &sc, f ? 0x111e0000 : 0x211e0000);
        g_assert_cmphex(rec_crc(dir, f, &len), ==, c->crc[f]);
        g_assert_cmpuint(len, ==, W * H * 3 / 2);
    }
    qtest_quit(qts);
    rm_dir(dir);
}

static void test_rec_still(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-avpu-rec-XXXXXX", NULL);
    g_autofree char *args = g_strdup_printf("-machine ingenic-t31 "
        "-global ingenic-avpu.capture=%s "
        "-global ingenic-avpu.h264-motion=search", dir);
    QTestState *qts = qtest_init(args);
    SliceCase sc = { .w = { 0x00700011, 0x4010ad50, 0x211e0000, 0x00083f1f,
                            0x38000000 }, .n = 1, .slice = { { 0, 4 } } };
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };
    uint32_t len;

    picture(y, uv);
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
    run_list(qts, &sc, 0x211e0000);
    run_list(qts, &sc, 0x111e0000);
    g_assert_cmphex(rec_crc(dir, 1, &len), ==, rec_crc(dir, 0, &len));
    qtest_quit(qts);
    rm_dir(dir);
}

/*
 * A picture the stream buffer cannot hold: the core writes what fits of the
 * data, zero padded to 32 bytes, in whole 32 byte units; the length word
 * keeps the whole length and nothing flags the overrun. A room or a size
 * under 32 is no limit. What a T31 wrote for this picture's single slice,
 * the room (word 0x33) and the buffer's size (0x31) set as listed.
 */
typedef struct {
    uint32_t room, size, written;
} RoomCase;

static const RoomCase room_cases[] = {
    { 0x22a20, 0x22c40, 1280 }, { 0x500, 0x22c40, 1280 },
    { 0x4f8, 0x22c40, 1248 }, { 0x4e8, 0x22c40, 1248 },
    { 0x4e0, 0x22c40, 1248 }, { 0x3f0, 0x22c40, 992 },
    { 0x400, 0x22c40, 1024 }, { 0x100, 0x22c40, 256 },
    { 0x20, 0x22c40, 32 }, { 0x10, 0x22c40, 1280 }, { 0, 0x22c40, 1280 },
    { 0x22a20, 0x700, 1248 }, { 0x22a20, 0x500, 736 },
    { 0x22a20, 0x10, 1280 }, { 0x22a20, 0, 1280 },
};

#define ROOM_LEN        1256
#define ROOM_CRC        0x1d0ad6ef
#define ROOM_GUARD      0xa5

static void test_room(const void *data)
{
    const RoomCase *r = data;
    QTestState *qts = qtest_init("-machine ingenic-t31");
    uint8_t y[W * H], uv[W * H / 2], ep2[0x100] = { [19] = 51 };
    uint8_t out[0x800], guard[0x800];
    uint32_t w[ENTRY / 4] = { 0 };
    unsigned mbw = W / 16, at = HDR;

    picture(y, uv);
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
    qtest_memwrite(qts, EP2, ep2, sizeof(ep2));
    memset(guard, ROOM_GUARD, sizeof(guard));
    qtest_memwrite(qts, BS + at, guard, sizeof(guard));
    w[0] = 1u << 31 | 0x00700011;
    w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
    w[2] = 0x4010ad50;
    w[3] = 0x211e0000;
    w[4] = 0x00083f1f;
    w[5] = 0;
    w[6] = (mbw - 1) | (H / 16 - 1) << 12;
    w[7] = 1u << 31 | (mbw - 1) | (H / 16 - 1) << 12;
    w[9] = 0x38000000;
    w[0xb] = w[0x1d] = (mbw - 1) << 12;
    w[0x1e] = mbw * H / 16 - 1;
    w[0x20] = SRC_Y;
    w[0x21] = SRC_C;
    w[0x22] = W;
    w[0x23] = EP2;
    w[0x30] = BS;
    w[0x31] = r->size;
    w[0x32] = at;
    w[0x33] = r->room;
    for (int i = 0; i < ENTRY / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, sizeof(w));
    qtest_writel(qts, BASE + CL_ADDR, LIST);
    qtest_writel(qts, BASE + START, 2);
    for (int i = 0; i < 40000 && !(qtest_readl(qts, BASE + IRQ) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, BASE + IRQ), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, LIST + STATUS * 4 + 4), ==, ROOM_LEN);
    qtest_memread(qts, BS + at, out, sizeof(out));
    for (unsigned i = 0; i < sizeof(out); i++) {
        if (i >= r->written) {
            g_assert_cmphex(out[i], ==, ROOM_GUARD);
        } else if (i >= ROOM_LEN) {
            g_assert_cmphex(out[i], ==, 0);
        }
    }
    if (r->written >= ROOM_LEN) {
        g_assert_cmphex(crc32_le(out, ROOM_LEN), ==, ROOM_CRC);
    } else {
        /* a prefix of the whole: the same bytes as the whole's */
        uint8_t whole[ROOM_LEN];

        qtest_writel(qts, LIST + 0x33 * 4, cpu_to_le32(0x22a20));
        qtest_writel(qts, LIST + 0x31 * 4, cpu_to_le32(0x22c40));
        qtest_writel(qts, LIST + 0x32 * 4, cpu_to_le32(SLOT * 6));
        qtest_writel(qts, BASE + CL_ADDR, LIST);
        qtest_writel(qts, BASE + IRQ, 1);
        qtest_writel(qts, BASE + START, 2);
        for (int i = 0; i < 40000 && !(qtest_readl(qts, BASE + IRQ) & 1);
             i++) {
            g_usleep(500);
        }
        qtest_memread(qts, BS + SLOT * 6, whole, sizeof(whole));
        g_assert_cmphex(crc32_le(whole, ROOM_LEN), ==, ROOM_CRC);
        g_assert_cmpint(memcmp(out, whole, r->written), ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(rec_cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-avpu/rec/%d-slices",
                                                rec_cases[i].n);

        qtest_add_data_func(path, &rec_cases[i], test_rec);
    }
    qtest_add_func("/ingenic-avpu/rec/still", test_rec_still);
    for (int i = 0; i < ARRAY_SIZE(room_cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-avpu/room/%x-%x",
                                                room_cases[i].room,
                                                room_cases[i].size);

        qtest_add_data_func(path, &room_cases[i], test_room);
    }
    for (int i = 0; i < ARRAY_SIZE(slice_cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-avpu/slices/%s",
                                                slice_cases[i].name);

        qtest_add_data_func(path, &slice_cases[i], test_slices);
    }
    return g_test_run();
}
