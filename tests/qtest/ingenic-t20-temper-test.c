/*
 * QTest testcase for the T20's Temper (the Apical core's temporal noise
 * reducer) and its frame in DDR
 *
 * A word a pixel, the frame and its filtered value, 12 bits each with a
 * count above them: as a T20X (and a T10L) runs it on the core's flat test
 * pattern, a step from 1024 to 2048 restarts the count at 15 and the
 * filtered value goes 1984, 2028, 2039, 2043, 2045, 2046, 2047 as the
 * count falls; it then goes 3, 2, 3. With Temper off the filtered value is
 * the frame, and bypassed it leaves the words alone.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"

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

static QTestState *start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t20 "
                                 "-device image-sensor,model=jxf22,bus=i2c0");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CORE + 0x10, W);
    qtest_writel(qts, CORE + 0x14, H);
    /* the test pattern, flat, and the raw front end bypassed but Temper */
    qtest_writel(qts, CORE + 0x40, 0x137cf3fe);
    qtest_writel(qts, CORE + 0x9c0, 1);
    qtest_writel(qts, CORE + 0x9c4, 0);
    tg_level(qts, 0x4000);
    /* as the T20X's firmware sets Temper up */
    qtest_writel(qts, CORE + 0x2c0, 0x83);
    qtest_writel(qts, CORE + 0x2c4, 100);
    qtest_writel(qts, CORE + 0xa18, BANK);
    qtest_writel(qts, CORE + 0xa1c, BANK);
    qtest_writel(qts, CORE + 0xa24, W * 4);
    qtest_writel(qts, CORE + 0xa28, 1);
    qtest_writel(qts, 0x13300000, 1);               /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 20 + FRAME_NS / 4);
    return qts;
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

    /* settled a level short, as a T20X's is */
    g_assert_cmphex(v & 0x0fff0fff, ==, 1024 << 16 | 1023);
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

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-t20-temper/step", test_step);
    qtest_add_func("/ingenic-t20-temper/off", test_off);
    return g_test_run();
}
