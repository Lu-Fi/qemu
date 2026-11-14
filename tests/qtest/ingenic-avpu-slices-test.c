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

static void picture(uint8_t *y, uint8_t *c)
{
    for (int r = 0; r < H; r++) {
        for (int x = 0; x < W; x++) {
            int v = 64 + (x * 5 + r * 3) % 128 +
                    (((x >> 4) ^ (r >> 4)) & 1) * 32 +
                    ((x * 7919 + r * 104729) >> 5 & 15);

            y[r * W + x] = MIN(v, 235);
        }
    }
    for (int r = 0; r < H / 2; r++) {
        for (int x = 0; x < W / 2; x++) {
            c[r * W + 2 * x] = 128 + (x * 3) % 64 - 32;
            c[r * W + 2 * x + 1] = 128 + (r * 3) % 64 - 32;
        }
    }
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

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(slice_cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-avpu/slices/%s",
                                                slice_cases[i].name);

        qtest_add_data_func(path, &slice_cases[i], test_slices);
    }
    return g_test_run();
}
