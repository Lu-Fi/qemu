/*
 * QTest testcase for the raw output of the T10's, T20's and T30's Apical
 * core: a DMA writer set to RAW16 or RAW12 puts out the sensor's samples
 *
 * As a T20X (Wyze V2, JXF23) runs sample-Snap-Raw: the core's processing
 * bypassed, the DS1 writer RAW16 with the line offset twice the width, the
 * samples the sensor's 10 bits left justified in 16 in the Bayer order of
 * 0x80018 (B Gb / Gr R), a line to the line offset with nothing written
 * between lines, the writer's size what it takes from the top left; set to
 * RAW12 it takes the 10 bits left justified in 12, two samples to 3 bytes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

#define CORE            0x13380000
#define CPM_CLKGR0      0x10000020
#define VIC             0x13300000
#define FR              (CORE + 0xb00)
#define DS1             (CORE + 0xc00)
#define W               64
#define H               32
#define BANK(n)         (0x02000000 + (n) * 0x100000)
#define FRAME_NS        (1000000000 / 30)

#define RAW16           6
#define RAW12           7
#define ON              BIT(9)
#define SENTINEL        0xa5

/* the white bar, away from the stripe at the top left: a 2 x 2 block */
#define BX              4
#define BY              4

static QTestState *start(const char *machine)
{
    g_autofree char *args = g_strdup_printf(
        "-machine %s -device image-sensor,model=jxq03,bus=i2c0,motion=off",
        machine);
    QTestState *qts = qtest_init(args);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CORE + 0x10, W);
    qtest_writel(qts, CORE + 0x14, H);
    return qts;
}

/* Writer @base: format @cfg, @size, @line apart, three banks; run */
static void writer(QTestState *qts, uint32_t base, uint32_t cfg,
                   uint32_t size, uint32_t line)
{
    for (int i = 0; i < 3; i++) {
        qtest_memset(qts, BANK(i), SENTINEL, 0x10000);
        qtest_writel(qts, base + 8 + i * 4, BANK(i));
    }
    qtest_writel(qts, base + 0x1c, 2);
    qtest_writel(qts, base + 0x20, line);
    qtest_writel(qts, base + 0x04, size);
    qtest_writel(qts, base, cfg | ON);
    qtest_writel(qts, base + 0x24, 2);
}

/* Frames until every bank has had one */
static void run(QTestState *qts)
{
    qtest_writel(qts, VIC, 1);
    qtest_clock_step(qts, FRAME_NS * 5);
}

static uint16_t sample16(QTestState *qts, uint32_t line, int x, int y)
{
    uint8_t v[2];

    qtest_memread(qts, BANK(1) + y * line + x * 2, v, 2);
    return lduw_le_p(v);
}

/* Colour 0 R, 1 G, 2 B of the pixel (x, y) in the Bayer order @pattern */
static int colour(int pattern, int x, int y)
{
    static const int order[4][4] = {
        { 0, 1, 1, 2 }, { 1, 0, 2, 1 }, { 1, 2, 0, 1 }, { 2, 1, 1, 0 },
    };

    return order[pattern][(y & 1) * 2 + (x & 1)];
}

/* The white bar's 2 x 2 block: G, then B, then R, as the sensor sees it */
static void check_block(QTestState *qts, int pattern)
{
    uint32_t v[3] = { 0 };

    for (int y = 0; y < 2; y++) {
        for (int x = 0; x < 2; x++) {
            uint16_t s = sample16(qts, W * 2, BX + x, BY + y);
            int c = colour(pattern, BX + x, BY + y);

            /* 10 bits at the top */
            g_assert_cmphex(s & 0x3f, ==, 0);
            if (v[c]) {
                g_assert_cmpuint(s, ==, v[c]);
            }
            v[c] = s;
        }
    }
    g_assert_cmpuint(v[1], >, v[2]);
    g_assert_cmpuint(v[2], >, v[0]);
    g_assert_cmpuint(v[0] * 256 / v[1], >=, 127);
    g_assert_cmpuint(v[0] * 256 / v[1], <=, 131);
    g_assert_cmpuint(v[2] * 256 / v[1], >=, 140);
    g_assert_cmpuint(v[2] * 256 / v[1], <=, 144);
}

static void test_bayer(const void *data)
{
    int pattern = GPOINTER_TO_INT(data);
    QTestState *qts = start("ingenic-t20");

    qtest_writel(qts, CORE + 0x18, pattern);
    writer(qts, DS1, RAW16, H << 16 | W, W * 2);
    run(qts);
    check_block(qts, pattern);
    qtest_quit(qts);
}

/*
 * The colour bars' red and blue bars (the sixth and seventh of eight): the
 * phase silicon's red lies in is the one that stands out in the red bar and
 * falls away in the blue bar, B the other way round. A T20X (JXF23, BGGR,
 * 0x80018 = 3) put its yellow level's colour on (1, 1) as R and (0, 0) as
 * B, and the register moves nothing in the raw output; the model follows
 * the register, the order a guest writes for its sensor
 */
static void test_red(const void *data)
{
    /* R's phase (x, y) in each order, B's the opposite corner */
    static const int rx[4] = { 0, 1, 0, 1 }, ry[4] = { 0, 0, 1, 1 };
    int pattern = GPOINTER_TO_INT(data);
    QTestState *qts = start("ingenic-t20");
    uint16_t v[2][2][2];

    qtest_writel(qts, CORE + 0x18, pattern);
    writer(qts, DS1, RAW16, H << 16 | W, W * 2);
    run(qts);
    for (int bar = 0; bar < 2; bar++) {
        int x0 = W * (5 + bar) / 8 + 2;

        for (int y = 0; y < 2; y++) {
            for (int x = 0; x < 2; x++) {
                v[bar][y][x] = sample16(qts, W * 2, x0 + x, 4 + y);
            }
        }
    }
    for (int bar = 0; bar < 2; bar++) {
        uint16_t red = v[bar][ry[pattern]][rx[pattern]];
        uint16_t blue = v[bar][!ry[pattern]][!rx[pattern]];

        if (bar == 0) {
            g_assert_cmpuint(red, >, blue * 4);
        } else {
            g_assert_cmpuint(blue, >, red * 4);
        }
    }
    qtest_quit(qts);
}

static void test_line(void)
{
    QTestState *qts = start("ingenic-t20");
    uint8_t row[0x200];

    writer(qts, DS1, RAW16, H << 16 | W, 0x200);
    run(qts);
    /* a line is 128 bytes of samples, the rest of the 512 left alone */
    for (int y = 0; y < H; y++) {
        qtest_memread(qts, BANK(1) + y * 0x200, row, sizeof(row));
        for (int i = 0; i < 0x200; i++) {
            if (i >= W * 2) {
                g_assert_cmphex(row[i], ==, SENTINEL);
            }
        }
    }
    g_assert_cmpuint(sample16(qts, 0x200, BX, BY), !=, 0xa5a5);
    qtest_quit(qts);
}

static void test_size(void)
{
    QTestState *qts = start("ingenic-t20");
    uint8_t row[W * 2];

    /* 16 x 8 from the top left: 8 lines of 32 bytes in 128 */
    writer(qts, DS1, RAW16, 8 << 16 | 16, W * 2);
    run(qts);
    for (int y = 0; y < H; y++) {
        qtest_memread(qts, BANK(1) + y * W * 2, row, sizeof(row));
        for (int i = 0; i < W * 2; i++) {
            if (y >= 8 || i >= 32) {
                g_assert_cmphex(row[i], ==, SENTINEL);
            }
        }
    }
    g_assert_cmpuint(sample16(qts, W * 2, 7, 7), !=, 0xa5a5);
    qtest_quit(qts);
}

/* RAW12 is RAW16 less 4 bits, two samples to 3 bytes, the first low */
static void test_raw12(void)
{
    QTestState *qts = start("ingenic-t20");
    uint16_t want[W];
    uint8_t line[W * 3 / 2];

    writer(qts, DS1, RAW16, H << 16 | W, W * 2);
    run(qts);
    for (int x = 0; x < W; x++) {
        want[x] = sample16(qts, W * 2, x, BY) >> 4;
    }
    qtest_quit(qts);

    qts = start("ingenic-t20");
    writer(qts, DS1, RAW12, H << 16 | W, W * 2);
    run(qts);
    qtest_memread(qts, BANK(1) + BY * W * 2, line, sizeof(line));
    for (int x = 0; x < W; x += 2) {
        uint32_t pair = line[x * 3 / 2] | line[x * 3 / 2 + 1] << 8 |
                        line[x * 3 / 2 + 2] << 16;

        g_assert_cmphex(pair & 0xfff, ==, want[x]);
        g_assert_cmphex(pair >> 12, ==, want[x + 1]);
        g_assert_cmphex(pair & 3, ==, 0);
    }
    /* a line is 96 bytes of the 128 apart */
    qtest_memread(qts, BANK(1) + BY * W * 2 + W * 3 / 2, line, 4);
    g_assert_cmphex(line[0], ==, SENTINEL);
    qtest_quit(qts);
}

/* Any of the three writers takes the format; its done event comes with it */
static void test_events(void)
{
    QTestState *qts = start("ingenic-t20");

    /* slot 6 takes event 15, DS1 writer done; slot 5 event 14, FR */
    qtest_writel(qts, CORE + 0x88, 14 << 16);
    qtest_writel(qts, CORE + 0x8c, 15);
    writer(qts, DS1, RAW16, H << 16 | W, W * 2);
    qtest_writel(qts, CORE + 0xa4, 0xffff);
    qtest_writel(qts, CORE + 0xa4, 0);
    qtest_writel(qts, VIC, 1);
    qtest_clock_step(qts, FRAME_NS * 3 / 4);
    g_assert_cmphex(qtest_readl(qts, CORE + 0xa0) & 0x60, ==, 0x40);
    qtest_clock_step(qts, FRAME_NS * 5);
    g_assert_cmpuint(sample16(qts, W * 2, BX, BY), !=, 0xa5a5);
    qtest_quit(qts);

    qts = start("ingenic-t20");
    /* the FR writer alone */
    writer(qts, FR, RAW16, H << 16 | W, W * 2);
    run(qts);
    g_assert_cmpuint(sample16(qts, W * 2, BX, BY), !=, 0xa5a5);
    qtest_quit(qts);
}

/*
 * A T30's libimp has the FR writer take the raw channel, straight to DDR; the
 * sensor's samples are on its DS1 writer, and FR writes zeros
 */
static void test_t30(void)
{
    QTestState *qts = start("ingenic-t30");
    uint8_t row[W * 2];

    qtest_writel(qts, CORE + 0x18, 3);
    writer(qts, FR, RAW16, H << 16 | W, W * 2);
    run(qts);
    qtest_memread(qts, BANK(1) + BY * W * 2, row, sizeof(row));
    for (int i = 0; i < W * 2; i++) {
        g_assert_cmphex(row[i], ==, 0);
    }
    qtest_memread(qts, BANK(1) + (H - 1) * W * 2, row, sizeof(row));
    g_assert_cmphex(row[W * 2 - 1], ==, 0);
    qtest_memread(qts, BANK(1) + H * W * 2, row, 1);
    g_assert_cmphex(row[0], ==, SENTINEL);
    qtest_quit(qts);

    qts = start("ingenic-t30");
    qtest_writel(qts, CORE + 0x18, 3);
    writer(qts, DS1, RAW16, H << 16 | W, W * 2);
    run(qts);
    check_block(qts, 3);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int p = 0; p < 4; p++) {
        g_autofree char *name = g_strdup_printf("/ingenic-t20-raw/bayer%d", p);

        qtest_add_data_func(name, GINT_TO_POINTER(p), test_bayer);
    }
    for (int p = 0; p < 4; p++) {
        g_autofree char *name = g_strdup_printf("/ingenic-t20-raw/red%d", p);

        qtest_add_data_func(name, GINT_TO_POINTER(p), test_red);
    }
    qtest_add_func("/ingenic-t20-raw/line", test_line);
    qtest_add_func("/ingenic-t20-raw/size", test_size);
    qtest_add_func("/ingenic-t20-raw/raw12", test_raw12);
    qtest_add_func("/ingenic-t20-raw/events", test_events);
    qtest_add_func("/ingenic-t30-raw/fr-blank", test_t30);
    return g_test_run();
}
