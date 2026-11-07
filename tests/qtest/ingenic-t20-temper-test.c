/*
 * QTest testcase for the T20's Temper (the Apical core's temporal noise
 * reducer) and its frame in DDR
 *
 * A word a pixel, the frame and its filtered value, 12 bits each with a
 * count above them: as a T20X (a T10L, a T30X) runs it on the core's flat
 * test pattern, a step from 1024 to 2048 restarts the count at 15 and the
 * filtered value goes 1984, 2028, 2039, 2043, 2045, 2046, 2047 as the
 * count falls; it then goes 3, 2, 3. The recursion limit (0x2cc) is the
 * least share of the difference it moves by, (L + 1) / 16. The change that
 * restarts the count, and at what, goes by the noise level in 0x2c4 (T =
 * 100: 43 restarts it at 6, 86 at 8, 128 at 10, 171 at 12, 213 at 14 and
 * 600 at 15), and 0x2c8, the long exposure's, does nothing without WDR.
 * With Temper off the filtered value is the frame, and bypassed it leaves
 * the words alone; the picture put out follows the filtered value.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include <math.h>

#define CORE            0x13380000
#define CPM_CLKGR0      0x10000020
#define W               64
#define H               32
#define BANK            0x02000000
#define FRAME_NS        (1000000000 / 30)

static void tg_level(QTestState *qts, uint32_t v)
{
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, CORE + 0x9c8 + i * 4, v);
    }
}

/* The machine under test, and where its VIC starts the frames */
static const char *machine = "ingenic-t20";
static uint32_t vic_ctrl;

static QTestState *start_at(const char *sensor, uint32_t level,
                            uint32_t noise, uint32_t limit)
{
    g_autofree char *args = g_strdup_printf(
        "-machine %s -device image-sensor,model=%s,bus=i2c0,id=cam",
        machine, sensor);
    QTestState *qts = qtest_init(args);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CORE + 0x10, W);
    qtest_writel(qts, CORE + 0x14, H);
    /* the test pattern, flat, and the raw front end bypassed but Temper */
    qtest_writel(qts, CORE + 0x40, 0x137cf3fe);
    qtest_writel(qts, CORE + 0x9c0, 1);
    qtest_writel(qts, CORE + 0x9c4, 0);
    tg_level(qts, level);
    /* as the T20X's firmware sets Temper up */
    qtest_writel(qts, CORE + 0x2c0, 0x83);
    qtest_writel(qts, CORE + 0x2c4, noise);
    qtest_writel(qts, CORE + 0x2cc, limit);
    qtest_writel(qts, CORE + 0xa18, BANK);
    qtest_writel(qts, CORE + 0xa1c, BANK);
    qtest_writel(qts, CORE + 0xa24, W * 4);
    qtest_writel(qts, CORE + 0xa28, 1);
    qtest_writel(qts, 0x13300000 + vic_ctrl, 1);    /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 20 + FRAME_NS / 4);
    return qts;
}

static QTestState *start(void)
{
    return start_at("jxf22", 0x4000, 100, 2);
}

static uint32_t word(QTestState *qts, int x, int y)
{
    return qtest_readl(qts, BANK + y * W * 4 + x * 4);
}

static void test_step(void)
{
    static const uint16_t filtered[] = {
        1984, 2028, 2039, 2043, 2045, 2046, 2047, 2047,
    };
    QTestState *qts = start();
    uint32_t v = word(qts, 5, 7), t;

    /* settled a level or two short, as a T20X's is */
    g_assert_cmpuint(extract32(v, 16, 12), ==, 1024);
    g_assert_cmpint(extract32(v, 0, 12), >=, 1022);
    g_assert_cmpint(extract32(v, 0, 12), <=, 1024);
    t = v >> 28;
    g_assert_true(t == 2 || t == 3);
    g_assert_cmpuint(extract32(v, 12, 4), ==, t);

    tg_level(qts, 0x8000);
    for (int i = 0; i < ARRAY_SIZE(filtered); i++) {
        qtest_clock_step(qts, FRAME_NS);
        v = word(qts, 5, 7);
        g_assert_cmpuint(v >> 28, ==, 15 - i);
        g_assert_cmpuint(extract32(v, 16, 12), ==, 2048);
        g_assert_cmpuint(extract32(v, 12, 4), ==, 15 - i);
        g_assert_cmpuint(v & 0xfff, ==, filtered[i]);
        g_assert_cmphex(word(qts, W - 1, H - 1), ==, v);
    }
    qtest_quit(qts);
}

static void test_off(void)
{
    QTestState *qts = start();
    uint32_t v;

    qtest_writel(qts, CORE + 0x2c0, 0x82);
    tg_level(qts, 0x5000);
    qtest_clock_step(qts, FRAME_NS);
    v = word(qts, 0, 0);
    g_assert_cmphex(v & 0x0fff0fff, ==, 1280 << 16 | 1280);

    /* bypassed: the words stay */
    qtest_writel(qts, CORE + 0x40, 0x137cf7fe);
    tg_level(qts, 0x6000);
    qtest_clock_step(qts, FRAME_NS * 3);
    g_assert_cmphex(word(qts, 0, 0), ==, v);
    qtest_quit(qts);
}

/* The recursion limit: the least share of the difference, (L + 1) / 16 */
static void test_limit(void)
{
    static const struct {
        uint32_t limit;
        uint16_t seq[7];
    } t[] = {
        { 8, { 1984, 2028, 2039, 2044, 2046, 2047, 2048 } },
        { 10, { 1984, 2028, 2042, 2046, 2047, 2048, 2048 } },
        { 12, { 1984, 2036, 2046, 2048, 2048, 2048, 2048 } },
        { 15, { 2048, 2048, 2048, 2048, 2048, 2048, 2048 } },
    };

    for (int i = 0; i < ARRAY_SIZE(t); i++) {
        QTestState *qts = start_at("jxf22", 0x4000, 100, t[i].limit);

        g_assert_cmpuint(extract32(word(qts, 3, 3), 0, 12), ==, 1024);
        tg_level(qts, 0x8000);
        for (int f = 0; f < 7; f++) {
            qtest_clock_step(qts, FRAME_NS);
            g_assert_cmpuint(extract32(word(qts, 3, 3), 0, 12), ==,
                             t[i].seq[f]);
        }
        qtest_quit(qts);
    }
}

/*
 * What a change restarts the count at, as a T20X's counts: the change from
 * the last frame (the upper count) in units of u = 0.5594 * 2 ^ (T / 16),
 * T = 0x2c4: u at 6, 2 u at 8, 3 u at 10, 4 u at 12, 5 u at 14, 14.1 u at 15
 */
static void test_restart(void)
{
    static const struct {
        uint32_t noise, change, count;
    } t[] = {
        { 100, 42, 0 }, { 100, 43, 6 }, { 100, 85, 6 }, { 100, 86, 8 },
        { 100, 127, 8 }, { 100, 128, 10 }, { 100, 170, 10 }, { 100, 175, 12 },
        { 100, 212, 12 }, { 100, 215, 14 }, { 100, 599, 14 }, { 100, 600, 15 },
    };
    QTestState *qts = start_at("jxf22", 2048 * 16, 100, 2);

    for (int i = 0; i < ARRAY_SIZE(t); i++) {
        uint32_t v;

        qtest_writel(qts, CORE + 0x2c4, t[i].noise);
        tg_level(qts, 2048 * 16);
        qtest_clock_step(qts, FRAME_NS * 20);
        tg_level(qts, (2048 + t[i].change) * 16);
        qtest_clock_step(qts, FRAME_NS);
        v = word(qts, 9, 5);
        g_assert_cmpuint(extract32(v, 16, 12), ==, 2048 + t[i].change);
        if (t[i].count) {
            g_assert_cmpuint(v >> 28, ==, t[i].count);
        } else {
            g_assert_cmpuint(v >> 28, <=, 3);
        }
    }
    qtest_quit(qts);
}

/* A T10's noise level is 9% up: 47 restarts the count at 6, 94 at 8 (T=100) */
static void test_t10(void)
{
    static const struct {
        uint32_t change, count;
    } t[] = { { 46, 0 }, { 47, 6 }, { 93, 6 }, { 94, 8 } };
    QTestState *qts;

    machine = "ingenic-t10";
    vic_ctrl = 0x20;
    qts = start_at("jxf22", 2048 * 16, 100, 2);
    for (int i = 0; i < ARRAY_SIZE(t); i++) {
        uint32_t v;

        tg_level(qts, 2048 * 16);
        qtest_clock_step(qts, FRAME_NS * 20);
        tg_level(qts, (2048 + t[i].change) * 16);
        qtest_clock_step(qts, FRAME_NS);
        v = word(qts, 9, 5);
        if (t[i].count) {
            g_assert_cmpuint(v >> 28, ==, t[i].count);
        } else {
            g_assert_cmpuint(v >> 28, <=, 3);
        }
    }
    qtest_quit(qts);
    machine = "ingenic-t20";
    vic_ctrl = 0;
}

/*
 * A change restarts the count of every pixel within 3 columns and 1 row of
 * it, at the count it gives the pixel itself (a T20X on a moving rectangle
 * of the test pattern): a line of one pixel, 300 up at T = 100, takes the
 * 3 x 7 pixels around it to 14. Their filtered values stay, for the
 * difference they see from the filtered value is none.
 */
static void test_window(void)
{
    QTestState *qts = start_at("jxf22", 1024 * 16, 100, 2);
    uint32_t before = word(qts, 29, 15);

    qtest_writel(qts, CORE + 0x9c4, 4);
    qtest_writel(qts, CORE + 0x9e8, 16);
    qtest_writel(qts, CORE + 0x9ec, 16);
    qtest_writel(qts, CORE + 0x9f0, 30);
    qtest_writel(qts, CORE + 0x9f8, 30);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, CORE + 0x9d4 + i * 4, 1024 * 16);
    }
    qtest_clock_step(qts, FRAME_NS * 20);
    for (int i = 0; i < 3; i++) {
        qtest_writel(qts, CORE + 0x9d4 + i * 4, 1324 * 16);
    }
    qtest_clock_step(qts, FRAME_NS);
    for (int y = 12; y <= 18; y++) {
        for (int x = 24; x <= 34; x++) {
            uint32_t v = word(qts, x, y);
            bool near = abs(x - 29) <= 3 && abs(y - 15) <= 1;

            if (near) {
                g_assert_cmpuint(v >> 28, ==, 14);
                g_assert_cmpuint(extract32(v, 12, 4), ==, 14);
            } else {
                g_assert_cmpuint(v >> 28, <=, 3);
            }
            if (x != 29 || y != 15) {
                g_assert_cmpuint(extract32(v, 0, 12), ==,
                                 extract32(before, 0, 12));
            }
        }
    }
    g_assert_cmpuint(extract32(word(qts, 29, 15), 16, 12), ==, 1324);
    qtest_quit(qts);
}

/*
 * A count restarts only above the count the pixel has: a light that goes
 * 150 up and down every frame (T = 100) restarts the count at 12 on the
 * way up and lets it fall to 11 on the way down, where the same change
 * would restart it at 12 again
 */
static void test_flicker(void)
{
    static const struct {
        uint32_t change, up, down, lower_up, lower_down;
    } t[] = {
        { 60, 6, 5, 6, 5 }, { 120, 10, 9, 8, 7 }, { 150, 12, 11, 0, 0 },
        { 200, 14, 13, 0, 0 },
    };
    QTestState *qts = start_at("jxf22", 1024 * 16, 100, 2);

    for (int i = 0; i < ARRAY_SIZE(t); i++) {
        for (int f = 0; f < 12; f++) {
            bool up = f % 2 == 0;
            uint32_t v;

            tg_level(qts, (1024 + (up ? t[i].change : 0)) * 16);
            qtest_clock_step(qts, FRAME_NS);
            v = word(qts, 7, 7);
            if (f < 6) {
                continue;
            }
            g_assert_cmpuint(v >> 28, ==, up ? t[i].up : t[i].down);
            if (t[i].lower_up) {
                g_assert_cmpuint(extract32(v, 12, 4), ==,
                                 up ? t[i].lower_up : t[i].lower_down);
            }
        }
        tg_level(qts, 1024 * 16);
        qtest_clock_step(qts, FRAME_NS * 20);
    }
    qtest_quit(qts);
}

/*
 * The noise of a scene is kept less as T rises (a T20X's: 0.99 at T = 40,
 * 0.5 at 70, 0.18 at 100, 0.14 from 110): the mean |change of the filtered
 * value| over the mean |change of the frame|, on the sensor's own noise
 */
static double kept_noise(uint32_t noise)
{
    QTestState *qts = start_at("jxf23,noise=on", 0x4000, noise, 2);
    uint32_t last[W * H], now[W * H];
    double df = 0, dc = 0;

    qtest_writel(qts, CORE + 0x9c0, 0);
    qtest_clock_step(qts, FRAME_NS * 8);
    qtest_memread(qts, BANK, last, sizeof(last));
    for (int f = 0; f < 30; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, BANK, now, sizeof(now));
        for (int i = 0; i < W * H; i++) {
            df += abs((int)extract32(now[i], 0, 12) -
                      (int)extract32(last[i], 0, 12));
            dc += abs((int)extract32(now[i], 16, 12) -
                      (int)extract32(last[i], 16, 12));
        }
        memcpy(last, now, sizeof(last));
    }
    qtest_quit(qts);
    return df / dc;
}

static void test_noise(void)
{
    double k40 = kept_noise(40), k140 = kept_noise(140);
    double k255 = kept_noise(255);

    g_assert_cmpfloat(k40, >, 0.95);
    g_assert_cmpfloat(k140, >, 0.3);
    g_assert_cmpfloat(k140, <, 0.6);
    g_assert_cmpfloat(k255, >, 0.14);
    g_assert_cmpfloat(k255, <, 0.25);
}

/* The long exposure's noise level (0x2c8) is WDR's: a linear sensor's is not */
static void test_long(void)
{
    static const uint32_t level[3] = { 100, 0, 255 };
    uint32_t seen[3];

    for (int i = 0; i < 3; i++) {
        QTestState *qts = start_at("jxf22", 2048 * 16, 100, 2);

        qtest_writel(qts, CORE + 0x2c8, level[i]);
        tg_level(qts, (2048 + 128) * 16);
        qtest_clock_step(qts, FRAME_NS);
        seen[i] = word(qts, 0, 0);
        qtest_quit(qts);
    }
    g_assert_cmphex(seen[1], ==, seen[0]);
    g_assert_cmphex(seen[2], ==, seen[0]);
    g_assert_cmpuint(seen[0] >> 28, ==, 10);
}

/*
 * The frame stitch's registers hold the bits a T20X and a T10L hold
 * (written all ones); the stitch is bypassed (0x40 bit 7) and its DMA
 * (0xa68 bit 0) off in linear mode, so Temper's words are alone in DDR;
 * run on a T10L's linear sensor, it writes the input frame, 12 bits left
 * justified in the upper half of a word, to its bank (0xa58)
 */
static void test_stitch(void)
{
    static const struct {
        uint32_t off, ones;
    } t[] = {
        { 0x240, 0x0003e0e3 }, { 0xa40, 0x030300ff }, { 0xa44, 0 },
        { 0xa48, 0xffffffff }, { 0xa4c, 0 }, { 0xa50, 0xffff },
        { 0xa54, 0xffff }, { 0xa58, 0xffffffff }, { 0xa5c, 0xffffffff },
        { 0xa60, 0 }, { 0xa64, 0xffffffff }, { 0xa68, 1 }, { 0xa6c, 0 },
    };
    QTestState *qts = start();

    for (int i = 0; i < ARRAY_SIZE(t); i++) {
        qtest_writel(qts, CORE + t[i].off, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, CORE + t[i].off), ==, t[i].ones);
        qtest_writel(qts, CORE + t[i].off, 0);
        g_assert_cmphex(qtest_readl(qts, CORE + t[i].off), ==, 0);
    }
    /* bypassed (0x40 bit 7), it leaves the words past Temper's alone */
    qtest_writel(qts, CORE + 0xa50, W);
    qtest_writel(qts, CORE + 0xa54, H);
    qtest_writel(qts, CORE + 0xa58, BANK + W * H * 4);
    qtest_writel(qts, CORE + 0xa5c, BANK + W * H * 4);
    qtest_writel(qts, CORE + 0xa64, W * 4);
    qtest_writel(qts, CORE + 0xa40, 0x30005);
    qtest_writel(qts, CORE + 0xa68, 1);
    qtest_memset(qts, BANK + W * H * 4, 0xa5, 64);
    qtest_clock_step(qts, FRAME_NS * 3);
    g_assert_cmphex(qtest_readl(qts, BANK + W * H * 4), ==, 0xa5a5a5a5);
    /* not bypassed, it writes the frame: a word a pixel, the sample << 20 */
    qtest_writel(qts, CORE + 0x40, 0x137cf37e);
    qtest_clock_step(qts, FRAME_NS * 3);
    g_assert_cmphex(qtest_readl(qts, BANK + W * H * 4), ==, 0x40000000);
    g_assert_cmphex(qtest_readl(qts, BANK + W * H * 4 + 4 * (W + 3)), ==,
                    0x40000000);
    tg_level(qts, 0x8123);
    qtest_clock_step(qts, FRAME_NS * 2);
    g_assert_cmphex(qtest_readl(qts, BANK + W * H * 4 + 8), ==, 0x81200000);
    /* with its writes off, it leaves the words as they were */
    qtest_writel(qts, CORE + 0xa40, 0x00005);
    tg_level(qts, 0x4000);
    qtest_clock_step(qts, FRAME_NS * 2);
    g_assert_cmphex(qtest_readl(qts, BANK + W * H * 4 + 8), ==, 0x81200000);
    qtest_quit(qts);
}

#define DS1             (CORE + 0xc00)
#define OUT(n)          (0x03000000 + (n) * 0x100000)

/* The mean luma of the last frame DS1 wrote */
static double luma_mean(QTestState *qts)
{
    uint8_t row[W];
    unsigned last = extract32(qtest_readl(qts, DS1 + 0x24), 11, 3);
    uint64_t sum = 0;

    for (int y = 0; y < H; y++) {
        qtest_memread(qts, OUT(last) + y * W, row, W);
        for (int x = 0; x < W; x++) {
            sum += row[x];
        }
    }
    return (double)sum / (W * H);
}

/*
 * The picture after Temper: a step in the light is followed by the
 * filtered value, a frame at a time, not at once (T = 255: nothing restarts
 * the count, L = 0: a 16th of the difference at least)
 */
static void picture(bool temper, double *first, double *after)
{
    QTestState *qts = start_at("jxq03,motion=off", 0x4000, 255, 0);

    /* Temper takes the scene, not the test pattern */
    qtest_writel(qts, CORE + 0x9c0, 0);
    qtest_writel(qts, CORE + 0x2c0, temper ? 0x83 : 0x82);
    for (int i = 0; i < 3; i++) {
        qtest_memset(qts, OUT(i), 0, W * H * 2);
        qtest_writel(qts, DS1 + 8 + i * 4, OUT(i));
    }
    qtest_writel(qts, DS1 + 0x1c, 2);
    qtest_writel(qts, DS1 + 0x20, W);
    qtest_writel(qts, DS1 + 0x04, H << 16 | W);
    qtest_writel(qts, DS1, 13 | BIT(9));
    qtest_writel(qts, DS1 + 0x24, 2);
    qtest_clock_step(qts, FRAME_NS * 30);
    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': '/machine/peripheral/cam', "
                             "'property': 'light', 'value': 40}}");
    qtest_clock_step(qts, FRAME_NS * 2);
    *first = luma_mean(qts);
    for (int i = 0; i < 150; i++) {
        qtest_clock_step(qts, FRAME_NS);
    }
    *after = luma_mean(qts);
    qtest_quit(qts);
}

static void test_picture(void)
{
    double on_first, on_after, off_first, off_after;

    picture(false, &off_first, &off_after);
    picture(true, &on_first, &on_after);
    /* Temper off puts the new light out at once; on, it comes in slowly */
    g_assert_cmpfloat(off_first - off_after, <, 1.0);
    g_assert_cmpfloat(on_first - on_after, >, 4.0);
    /* and in the end, the same */
    g_assert_cmpfloat(fabs(on_after - off_after), <, 0.5);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-t20-temper/step", test_step);
    qtest_add_func("/ingenic-t20-temper/off", test_off);
    qtest_add_func("/ingenic-t20-temper/limit", test_limit);
    qtest_add_func("/ingenic-t20-temper/restart", test_restart);
    qtest_add_func("/ingenic-t20-temper/t10", test_t10);
    qtest_add_func("/ingenic-t20-temper/window", test_window);
    qtest_add_func("/ingenic-t20-temper/flicker", test_flicker);
    qtest_add_func("/ingenic-t20-temper/noise", test_noise);
    qtest_add_func("/ingenic-t20-temper/long", test_long);
    qtest_add_func("/ingenic-t20-temper/picture", test_picture);
    qtest_add_func("/ingenic-t20-temper/stitch", test_stitch);
    return g_test_run();
}
