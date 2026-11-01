/*
 * QTest testcase for the Ingenic RTC
 *
 * The power-on values as each part reads them, and the write lock as a
 * T41NQ takes it: 0xa55a in WENR opens the protected registers for one
 * write, any other WENR write closes them, and a locked write changes
 * only RTCCR's SELEXC and 1HZIE. The T40 has none to see: a T40XP and a
 * T40NN read 0 at 0x10003000 and at its kernel's RTC_IOBASE, 0x132a0000.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RTC             0x10003000
#define RTCCR           0x00
#define HRCR            0x28
#define HWCR            0x2c
#define HWRSR           0x30
#define HSPR            0x34
#define WENR            0x3c
#define WKUPPINCR       0x48

#define RTCCR_WRDY      0x80
#define RTCCR_HZIE      0x20
#define RTCCR_RTCE      0x01
#define HWRSR_PPR       0x10
#define WENR_WEN        0x80000000u
#define WENR_PAT        0xa55a

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    uint32_t wkuppincr, hrcr;
} RtcCase;

static const RtcCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", 0x00000064, 0x1000 },
    { "t20x", "ingenic-t20", "ingenic-t31", 0x00010064, 0x800 },
    { "t21n", "ingenic-t21", "ingenic-t31", 0x00010064, 0x800 },
    { "t30x", "ingenic-t30", "ingenic-t31", 0x00010064, 0x800 },
    { "t31x", "ingenic-t31", "ingenic-t31", 0x00010064, 0x800 },
    { "t32lq", "ingenic-t32", "ingenic-t31", 0x00210064, 0x800 },
    { "t41nq", "ingenic-t41", "ingenic-t40", 0x00a30064, 0x800 },
};

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, RTC + reg);
}

static void wr(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, RTC + reg, value);
}

static void test_power_on(const void *data)
{
    const RtcCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_WRDY | RTCCR_RTCE);
    g_assert_cmphex(rd(qts, HWCR), ==, 0x2d52d2d0);
    g_assert_cmphex(rd(qts, HWRSR), ==, HWRSR_PPR);
    g_assert_cmphex(rd(qts, WENR), ==, 0);
    g_assert_cmphex(rd(qts, WKUPPINCR), ==, c->wkuppincr);
    g_assert_cmphex(rd(qts, HRCR), ==, c->hrcr);
    qtest_quit(qts);
}

static void test_t40_none(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    for (uint32_t off = 0; off < 0x50; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x132a0000 + off), ==, 0);
        g_assert_cmphex(qtest_readl(qts, 0x10003000 + off), ==, 0);
    }
    qtest_quit(qts);
}

static void test_write_lock(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    wr(qts, HSPR, 0x12345678);
    g_assert_cmphex(rd(qts, HSPR), ==, 0);
    wr(qts, WKUPPINCR, 0x00a30074);
    g_assert_cmphex(rd(qts, WKUPPINCR), ==, 0x00a30064);

    wr(qts, WENR, WENR_PAT);
    g_assert_cmphex(rd(qts, WENR), ==, WENR_WEN);
    wr(qts, HSPR, 0x12345678);
    g_assert_cmphex(rd(qts, HSPR), ==, 0x12345678);
    g_assert_cmphex(rd(qts, WENR), ==, 0);
    wr(qts, HSPR, 0x87654321);
    g_assert_cmphex(rd(qts, HSPR), ==, 0x12345678);

    wr(qts, WENR, WENR_PAT);
    wr(qts, 0x40, 0);
    g_assert_cmphex(rd(qts, WENR), ==, WENR_WEN);
    wr(qts, WENR, 0x1234);
    g_assert_cmphex(rd(qts, WENR), ==, 0);

    wr(qts, RTCCR, RTCCR_HZIE);
    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_WRDY | RTCCR_HZIE | RTCCR_RTCE);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-rtc/power-on/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_power_on);
    }
    qtest_add_func("ingenic-rtc/write-lock", test_write_lock);
    qtest_add_func("ingenic-rtc/t40-none", test_t40_none);
    return g_test_run();
}
