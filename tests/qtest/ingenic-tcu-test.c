/*
 * QTest testcase for the Ingenic TCU's channels and flag mask
 *
 * TMR out of reset as each part reads it, and the channels each part has: a
 * T31X takes writes to channel 3's registers and reads 0 from channel 4's
 * and 7's after writing them, and TMSR and TMCR move only the bits it has.
 * Where TMR has bit 24, HFLAGW, a watchdog ping (TCSR bit 10, the counter
 * clear) sets TFR's bit 24, as a T41NQ's does. The OST counts while TER's
 * bit 15 is set, TSR's clear and OSTCSR picks a clock, and holds
 * otherwise; TFR's bit 15 sets where its count meets OSTDR, as at U-Boot's
 * start of it from 0 with OSTDR 0. The T32's,
 * T41's and A1's channels read TDFR 0 out of reset, as at their bootroms.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "libqtest.h"

#define TCU             0x10002000
#define TCU_TER         0x10
#define TCU_TESR        0x14
#define TCU_TECR        0x18
#define TCU_TFR         0x20
#define TCU_OST_CNTL    0xe4
#define TCU_OST_CSR     0xec
#define TCU_TFCR        0x28
#define TCU_TMR         0x30
#define TCU_TMSR        0x34
#define TCU_TMCR        0x38
#define TCU_TDFR(n)     (0x40 + (n) * 0x10)
#define TCU_TDHR(n)     (0x44 + (n) * 0x10)

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    uint32_t tmr;
} TcuCase;

static const TcuCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", 0x000f800f },
    { "t20x", "ingenic-t20", "ingenic-t31", 0x000f800f },
    { "t21n", "ingenic-t21", "ingenic-t31", 0x00ff80ff },
    { "t23n", "ingenic-t23", "ingenic-t31", 0x000f800f },
    { "t30x", "ingenic-t30", "ingenic-t31", 0x00ff80ff },
    { "t31x", "ingenic-t31", "ingenic-t31", 0x000f800f },
    { "t32lq", "ingenic-t32", "ingenic-t31", 0x01ff80ff },
    { "t40n", "ingenic-t40", "ingenic-t40", 0x00ff80ff },
    { "t41lq", "ingenic-t41", "ingenic-t40", 0x01ff80ff },
    { "a1n", "ingenic-a1", "ingenic-a1", 0x01ff80ff },
};

static void test_case(const void *data)
{
    const TcuCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);
    unsigned channels = ctz32(~c->tmr);

    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, c->tmr);
    qtest_writel(qts, TCU + TCU_TMCR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, 0);
    qtest_writel(qts, TCU + TCU_TMSR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, c->tmr);

    /* TDFR out of reset: 0 on the T32, T41 and A1, which have bit 24 */
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDFR(0)), ==,
                    c->tmr & 1u << 24 ? 0 : 0xffff);
    for (unsigned n = 0; n < 8; n++) {
        uint32_t want = n < channels ? 0x1234 + n : 0;

        qtest_writel(qts, TCU + TCU_TDFR(n), 0x1234 + n);
        qtest_writel(qts, TCU + TCU_TDHR(n), 0x1234 + n);
        g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDFR(n)), ==, want);
        g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDHR(n)), ==, want);
    }
    qtest_writel(qts, TCU + TCU_TESR, 0xff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER), ==, (1u << channels) - 1);
    qtest_writel(qts, TCU + TCU_TECR, 0xff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER), ==, 0);

    qtest_writel(qts, TCU + TCU_TFCR, 0xffffffff);
    qtest_writel(qts, TCU + 0x0c, 0x1a | 1 << 10);     /* WDT TCSR */
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR), ==, c->tmr & 1u << 24);
    qtest_quit(qts);
}

static void test_ost(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");
    uint32_t a, b;

    qtest_clock_step(qts, 1000000);
    a = qtest_readl(qts, TCU + TCU_OST_CNTL);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, a);

    qtest_writel(qts, TCU + TCU_OST_CSR, 0x800c);       /* as U-Boot sets it */
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, a);
    qtest_writel(qts, TCU + TCU_TESR, 1u << 15);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER) & 1u << 15, ==, 1u << 15);
    qtest_clock_step(qts, 1000000);                     /* 1 ms: 6000 ticks */
    b = qtest_readl(qts, TCU + TCU_OST_CNTL);
    g_assert_cmpuint(b - a, ==, 6000);

    qtest_writel(qts, TCU + TCU_TECR, 1u << 15);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, b);

    /* U-Boot's start: count and OSTDR 0, then enable; the flag sets */
    qtest_writel(qts, TCU + TCU_TFCR, 1u << 15);
    qtest_writel(qts, TCU + TCU_OST_CNTL, 0);
    qtest_writel(qts, TCU + TCU_TESR, 1u << 15);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 1u << 15, ==, 1u << 15);
    /* cleared, it stays so until the count meets OSTDR again */
    qtest_writel(qts, TCU + TCU_TFCR, 1u << 15);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 1u << 15, ==, 0);
    qtest_writel(qts, TCU + 0xe0, 9000);                /* OSTDR, 1.5 ms */
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 1u << 15, ==, 1u << 15);
    /* a stopped OST holds */
    qtest_writel(qts, TCU + 0x2c, 1u << 15);           /* TSSR */
    a = qtest_readl(qts, TCU + TCU_OST_CNTL);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, a);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-tcu/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    qtest_add_func("ingenic-tcu/ost", test_ost);
    return g_test_run();
}
