/*
 * QTest testcase for the Ingenic GPIO's pad registers and power-on values
 *
 * Each port as its part's bootrom leaves it on silicon, read through the
 * bootrom on a T10L, T31X, T32LQ, T41NQ and A1N, with the pins the bootrom
 * puts in a function for flash and cards as inputs again. A pad register
 * (pulls, drive strength, slew rate, ...) holds what is written, with set
 * and clear ports at +4 and +8; the T32's PxUPD, at 0x1e4, has none. An
 * offset a family lacks reads 0.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GPIO            0x10010000

typedef struct GpioReset {
    const char *args;
    uint32_t stride;
    struct {
        unsigned port;
        uint32_t off, value;
    } r[6];
} GpioReset;

static const GpioReset resets[] = {
    { "-machine ingenic-t10 -global ingenic-t31.soc-variant=t10l", 0x100, {
        { 0, 0x20, 0x1fffffff }, { 1, 0x40, 0x01c80000 },
        { 1, 0x60, 0xff06040e }, { 2, 0x30, 0x0007fbff },
    } },
    { "-machine ingenic-t31 -global ingenic-t31.soc-variant=t31x", 0x1000, {
        { 0, 0x20, 0x1fc7ffc0 }, { 0, 0x110, 0x1fc33800 },
        { 0, 0x130, 0xaaaaa000 }, { 0, 0x140, 0x02aaa02a },
        { 2, 0x110, 0x00004309 }, { 2, 0x160, 0x001e0000 },
    } },
    { "-machine ingenic-t32 -global ingenic-t31.soc-variant=t32lq", 0x1000, {
        { 0, 0x110, 0x1f80fc00 }, { 2, 0x20, 0x83fffbff },
        { 2, 0x130, 0x55855565 }, { 2, 0x1e4, 0 },
    } },
    { "-machine ingenic-t41 -global ingenic-t40.soc-variant=t41nq", 0x1000, {
        { 0, 0x20, 0xffffffc0 }, { 0, 0x130, 0x000fffc0 },
        { 0, 0x140, 0xfff00000 }, { 1, 0x20, 0xfe4fefff },
        { 3, 0x130, 0x00001fff },
    } },
    { "-machine ingenic-a1", 0x1000, {
        { 1, 0x110, 0x00000a00 }, { 2, 0x40, 0x000f0000 },
        { 2, 0x110, 0x02400001 }, { 4, 0x130, 0x0001ffff },
    } },
};

static void test_reset(const void *data)
{
    const GpioReset *c = data;
    QTestState *qts = qtest_init(c->args);

    for (unsigned i = 0; i < ARRAY_SIZE(c->r) && c->r[i].off; i++) {
        uint32_t a = GPIO + c->r[i].port * c->stride + c->r[i].off;

        g_assert_cmphex(qtest_readl(qts, a), ==, c->r[i].value);
    }
    qtest_quit(qts);
}

static void test_pads(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 "
                                 "-global ingenic-t31.soc-variant=t32lq");
    uint32_t pc = GPIO + 0x2000;

    /* drive strength 0 on PC: set and clear ports */
    qtest_writel(qts, pc + 0x130, 0);
    qtest_writel(qts, pc + 0x134, 0x00f0000f);
    g_assert_cmphex(qtest_readl(qts, pc + 0x130), ==, 0x00f0000f);
    qtest_writel(qts, pc + 0x138, 0x0000000c);
    g_assert_cmphex(qtest_readl(qts, pc + 0x130), ==, 0x00f00003);

    /* PxUPD holds its write; 0x1e8 is no clear of it */
    qtest_writel(qts, pc + 0x1e4, 1);
    qtest_writel(qts, pc + 0x1e8, 1);
    g_assert_cmphex(qtest_readl(qts, pc + 0x1e4), ==, 1);
    qtest_quit(qts);

    /* the T30 has pulls alone */
    qts = qtest_init("-machine ingenic-t30 "
                     "-global ingenic-t31.soc-variant=t30x");
    qtest_writel(qts, GPIO + 0x130, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, GPIO + 0x130), ==, 0);
    qtest_writel(qts, GPIO + 0x114, 0x1);
    g_assert_cmphex(qtest_readl(qts, GPIO + 0x110), ==, 0x05400001);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const char *const names[] = { "t10", "t31", "t32", "t41", "a1" };

    g_test_init(&argc, &argv, NULL);
    for (unsigned i = 0; i < ARRAY_SIZE(resets); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-gpio/reset/%s",
                                                names[i]);

        qtest_add_data_func(path, &resets[i], test_reset);
    }
    qtest_add_func("ingenic-gpio/pads", test_pads);
    return g_test_run();
}
