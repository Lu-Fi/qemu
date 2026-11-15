/*
 * QTest testcase for the T30's RADIX H.265 encoder: P pictures
 *
 * An I picture and two P pictures of a textured pattern that moves 4
 * samples to the right each picture, as a command list sets them up, the
 * reference the previous reconstruction. Without the motion search, the
 * default, a block is skipped or intra: the three pictures' bytes are the
 * ones the model had before the search existed. With h265-motion=search the
 * inter blocks follow the pattern, the P pictures cost a seventh of the
 * bytes and the reconstruction is as close to the source.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define RADIX           0x13600000
#define W               128
#define H               64

#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define REF_Y           0x00820000
#define REF_C           0x00830000
#define BS              0x00840000
#define LIST            0x00850000
#define REC_Y           0x00860000
#define REC_C           0x00870000

#define CL_VLD          (1u << 31)
#define CL_TERM         (1u << 30)

typedef struct {
    uint32_t len, crc;
} Pic;

typedef struct {
    const char *name;
    const char *args;
    Pic pic[3];
} Case;

static const Case cases[] = {
    { "off", "", { { 2080, 0x766c8b91 }, { 2061, 0x009f3963 },
                   { 2174, 0x5d4eea01 } } },
    { "search", "-global ingenic-radix.h265-motion=search",
      { { 2080, 0x766c8b91 }, { 289, 0x14f407db }, { 220, 0x6788cba8 } } },
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

static void picture(uint8_t *y, uint8_t *uv, int sx)
{
    for (int r = 0; r < H; r++) {
        for (int x = 0; x < W; x++) {
            int xs = x - sx;
            int v = 40 + ((xs * 7919 + r * 104729) >> 4 & 127) +
                    (((xs >> 3) ^ (r >> 3)) & 1) * 40;

            y[r * W + x] = MIN(v, 235);
        }
    }
    for (int r = 0; r < H / 2; r++) {
        for (int x = 0; x < W / 2; x++) {
            int d = x - sx / 2;

            uv[r * W + 2 * x] = 128 + ((d * 3) % 64 + 64) % 64 - 32;
            uv[r * W + 2 * x + 1] = 128 + (r * 3) % 64 - 32;
        }
    }
}

static void test_radix(const void *data)
{
    const Case *c = data;
    g_autofree char *args = g_strdup_printf("-machine ingenic-t30 %s", c->args);
    QTestState *qts = qtest_init(args);
    uint8_t y[W * H], uv[W * H / 2], rec_y[W * H], rec_c[W * H / 2];
    uint8_t out[W * H * 3 / 2];
    uint64_t ssd[3];

    for (int n = 0; n < 3; n++) {
        const uint32_t regs[][2] = {
            { 0x68000, (n ? 1 : 2) << 2 | 30 << 5 | 1 << 4 | 5 << 11 },
            { 0x68008, BS }, { 0x20004, H << 16 | W },
            { 0x2000c, SRC_Y }, { 0x20010, SRC_C },
            { 0x20014, W | W << 16 }, { 0x10014, REC_Y }, { 0x10018, REC_C },
            { 0x50010, 0 }, { 0x30800, REF_Y }, { 0x30804, REF_C },
            { 0x30064, W | W << 16 }, { 0x20018, 1 },
        };
        uint32_t list[2 * ARRAY_SIZE(regs)];
        uint32_t len;

        picture(y, uv, 4 * n);
        qtest_memwrite(qts, SRC_Y, y, sizeof(y));
        qtest_memwrite(qts, SRC_C, uv, sizeof(uv));
        if (n) {
            qtest_memwrite(qts, REF_Y, rec_y, sizeof(rec_y));
            qtest_memwrite(qts, REF_C, rec_c, sizeof(rec_c));
        }
        for (int i = 0; i < ARRAY_SIZE(regs); i++) {
            list[2 * i] = cpu_to_le32(regs[i][1]);
            list[2 * i + 1] = cpu_to_le32(CL_VLD | regs[i][0] |
                                          (i == ARRAY_SIZE(regs) - 1 ?
                                           CL_TERM : 0));
        }
        qtest_memwrite(qts, LIST, list, sizeof(list));
        qtest_writel(qts, RADIX + 0x00, 1u << 24);
        qtest_writel(qts, RADIX + 0x08, LIST | 1);
        for (int i = 0; i < 40000 && !(qtest_readl(qts, RADIX + 0x40) & 0x100);
             i++) {
            g_usleep(500);
        }
        g_assert_cmphex(qtest_readl(qts, RADIX + 0x40) & 0x100, ==, 0x100);
        len = qtest_readl(qts, RADIX + 0x44);
        g_assert_cmpuint(len, ==, c->pic[n].len);
        qtest_memread(qts, BS, out, len);
        g_assert_cmphex(crc32_le(out, len), ==, c->pic[n].crc);
        qtest_memread(qts, REC_Y, rec_y, sizeof(rec_y));
        qtest_memread(qts, REC_C, rec_c, sizeof(rec_c));
        ssd[n] = 0;
        for (int i = 0; i < W * H; i++) {
            ssd[n] += (rec_y[i] - y[i]) * (rec_y[i] - y[i]);
        }
        qtest_writel(qts, RADIX + 0x40, 0);
    }
    if (*c->args) {
        /* the search's pictures are as true to the source */
        g_assert_cmpuint(ssd[1], <, ssd[0] + ssd[0] / 10);
        g_assert_cmpuint(ssd[2], <, ssd[0] + ssd[0] / 10);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-radix/p/%s",
                                                cases[i].name);

        qtest_add_data_func(path, &cases[i], test_radix);
    }
    return g_test_run();
}
