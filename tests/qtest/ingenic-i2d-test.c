/*
 * QTest testcase for the Ingenic I2D
 *
 * Its registers out of reset and the widths they keep of a write, then
 * jobs as a T41NQ ran them through the vendor driver: the same source
 * pattern and registers, and the CRC of 64 KiB of destination, filled with
 * 0xa5 before, as the part left it. A T32NQ, its I2D at 0x13030000, reads
 * and writes the same; the T32 runs one of the jobs. The cases turn NV12,
 * RAW8, RGB565 and ARGB8888 pictures; the driver runs a rotated RGB565 job
 * as NV12 and places a mirrored rotation's UV plane over its Y plane.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2D             0x130b0000
#define T32_I2D         0x13030000
#define SRC             0x00800000
#define DST             0x00900000
#define DST_LEN         0x10000

typedef struct {
    const char *name;
    uint32_t size, mode, src_uv, dst_uv;
    uint32_t stride[4];         /* source Y, UV, destination Y, UV */
    uint32_t crc;               /* of the T41NQ's destination */
} Case;

static const Case cases[] = {
    { "nv12-r90", 0x00600040, 0x04, 0x00801800, 0x00901800,
      { 0x60, 0x60, 0x40, 0x40 }, 0xd4b6318a },
    { "nv12-r90-mirr", 0x00600040, 0x06, 0x00801800, 0x00901000,
      { 0x60, 0x60, 0x40, 0x40 }, 0x11ffbe38 },
    { "nv12-mirr-flip", 0x00600040, 0x03, 0x00801800, 0x00901800,
      { 0x60, 0x60, 0x60, 0x60 }, 0xe1499761 },
    { "raw8-flip", 0x00600040, 0x11, 0x00801800, 0x00901800,
      { 0x60, 0x60, 0x60, 0x60 }, 0x7cd86695 },
    { "rgb565-r90", 0x00300020, 0x04, 0x00800c00, 0x00900c00,
      { 0x60, 0x30, 0x40, 0x20 }, 0x4727b043 },
    { "argb-mirr", 0x00200010, 0x32, 0x00800800, 0x00900800,
      { 0x80, 0x20, 0x80, 0x20 }, 0xd99d66fd },
};

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static void test_resets(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);
    uint32_t i2d = strcmp(machine, "ingenic-t32") ? I2D : T32_I2D;
    static const uint32_t want[][3] = {
        /* offset, reset, kept of all ones */
        { 0x00, 0x20191031, 0x20191031 }, { 0x08, 0, 0x3 },
        { 0x14, 0x028001e0, 0xffffffff }, { 0x18, 0, 0x13f },
        { 0x20, 0, 0xffffffff }, { 0x28, 0, 0xffff }, { 0x3c, 0, 0xffff },
        { 0x84, 0, 0 }, { 0x88, 0, 0x3 }, { 0x90, 0xffffffff, 0xffffffff },
        { 0x94, 1, 1 }, { 0x98, 1, 1 }, { 0xa0, 0x7001, 0x7001 },
    };

    for (int i = 0; i < ARRAY_SIZE(want); i++) {
        g_test_message("0x%02x", want[i][0]);
        g_assert_cmphex(qtest_readl(qts, i2d + want[i][0]), ==, want[i][1]);
        qtest_writel(qts, i2d + want[i][0], 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, i2d + want[i][0]), ==, want[i][2]);
    }
    qtest_quit(qts);
}

typedef struct {
    const Case *c;
    const char *machine;
    uint32_t i2d;
} Job;

static void test_job(const void *data)
{
    const Job *j = data;
    const Case *c = j->c;
    QTestState *qts = qtest_initf("-machine %s", j->machine);
    uint32_t i2d = j->i2d;
    uint32_t w = c->size >> 16, h = c->size & 0xffff;
    uint32_t bpp = c->stride[0] / w;
    uint32_t rows = (h + 15) & ~15;
    uint32_t ylen = w * bpp * rows;
    g_autofree uint8_t *buf = g_malloc(DST_LEN);
    static const uint8_t code[] = { 0, 7, 6, 1 };
    uint32_t dbg;

    /* The jobs' source: Y = column + 2 * row, then a UV-like plane */
    for (uint32_t k = 0; k < 2 * ylen; k++) {
        buf[k] = k < ylen ? k % (w * bpp) + 2 * (k / (w * bpp))
                          : 0x80 + 3 * (k % w) + k / w;
    }
    qtest_memwrite(qts, SRC, buf, 2 * ylen);
    memset(buf, 0xa5, DST_LEN);
    qtest_memwrite(qts, DST, buf, DST_LEN);

    qtest_writel(qts, i2d + 0x18, c->mode);
    qtest_writel(qts, i2d + 0x14, c->size);
    qtest_writel(qts, i2d + 0x20, SRC);
    qtest_writel(qts, i2d + 0x24, c->src_uv);
    qtest_writel(qts, i2d + 0x30, DST);
    qtest_writel(qts, i2d + 0x34, c->dst_uv);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, i2d + 0x28 + (i & 1) * 4 + (i & 2) * 8,
                     c->stride[i]);
    }
    qtest_writel(qts, i2d + 0x08, 0xffffffff);
    qtest_writel(qts, i2d + 0x10, 1);
    qtest_clock_step(qts, 1000);

    g_assert_cmphex(qtest_readl(qts, i2d + 0x10), ==, 0);
    g_assert_cmphex(qtest_readl(qts, i2d + 0x08), ==, 0);
    g_assert_cmphex(qtest_readl(qts, i2d + 0x80), ==, 1);
    dbg = code[(c->mode >> 2) & 3] ^ (c->mode & 2 ? 4 : 0) ^
          (c->mode & 1 ? 2 : 0);
    g_assert_cmphex(qtest_readl(qts, i2d + 0xa0), ==, 0x7001 | dbg << 8);
    qtest_writel(qts, i2d + 0x84, 1);
    g_assert_cmphex(qtest_readl(qts, i2d + 0x80), ==, 0);

    qtest_memread(qts, DST, buf, DST_LEN);
    g_assert_cmphex(crc32_le(buf, DST_LEN), ==, c->crc);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    static Job jobs[ARRAY_SIZE(cases) + 1];

    qtest_add_data_func("ingenic-i2d/resets/t41", "ingenic-t41", test_resets);
    qtest_add_data_func("ingenic-i2d/resets/t32", "ingenic-t32", test_resets);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-i2d/%s",
                                                cases[i].name);
        jobs[i] = (Job) { &cases[i], "ingenic-t41", I2D };
        qtest_add_data_func(path, &jobs[i], test_job);
    }
    jobs[ARRAY_SIZE(cases)] = (Job) { &cases[0], "ingenic-t32", T32_I2D };
    qtest_add_data_func("ingenic-i2d/t32/nv12-r90", &jobs[ARRAY_SIZE(cases)],
                        test_job);
    return g_test_run();
}
