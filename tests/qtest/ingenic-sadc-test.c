/*
 * QTest testcase for the Ingenic SAR A/D controller (SADC)
 *
 * Conversion time and the power-down wedge, as a T30X measures them: a
 * conversion takes ADSTB's STABLE plus 20 A/D clocks of EXTCLK over
 * ADCLK's CLKDIV plus one, and a channel converting when the converter
 * powers down, or set while it is down, holds its bit and never converts.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SADC            0x10070000
#define ADENA           (SADC + 0x00)
#define ADSTATE         (SADC + 0x0c)
#define ADATA           (SADC + 0x10)
#define ADCLK           (SADC + 0x20)
#define ADSTB           (SADC + 0x24)

#define POWER           0x8000
#define SLOW_NS         699242667   /* (0xffff + 20) * 256 / 24 MHz */

static QTestState *sadc_init(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30 "
                                 "-global ingenic-t31.soc-variant=t30x");

    g_assert_cmphex(qtest_readl(qts, ADENA), ==, POWER);
    qtest_writel(qts, ADENA, 0);
    return qts;
}

/* At the reset clocks a conversion is in before the next access */
static void test_reset_clocks(void)
{
    QTestState *qts = sadc_init();

    qtest_writel(qts, ADENA, 1);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ADATA) & 0xfff, ==, 2025);
    qtest_quit(qts);
}

/* STABLE 0xffff at CLKDIV 0xff: 0.70 s */
static void test_slow(void)
{
    QTestState *qts = sadc_init();

    qtest_writel(qts, ADCLK, 0x000410ff);
    qtest_writel(qts, ADSTB, 0xffff);
    qtest_writel(qts, ADENA, 1);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 1);
    qtest_clock_step(qts, SLOW_NS - 1000000);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 1);
    qtest_quit(qts);
}

/* Powered down mid-conversion, the channel holds its bit until reset */
static void test_power_down_mid(void)
{
    QTestState *qts = sadc_init();

    qtest_writel(qts, ADCLK, 0x000410ff);
    qtest_writel(qts, ADSTB, 0xffff);
    qtest_writel(qts, ADENA, 1);
    qtest_writel(qts, ADENA, POWER | 1);
    qtest_writel(qts, ADENA, 1);
    qtest_clock_step(qts, 2 * SLOW_NS);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0);

    /* Neither a clear nor a fresh start moves it */
    qtest_writel(qts, ADENA, 0);
    qtest_writel(qts, ADCLK, 0x00041000);
    qtest_writel(qts, ADSTB, 0);
    qtest_writel(qts, ADENA, 1);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0);

    qtest_system_reset(qts);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, POWER);
    qtest_quit(qts);
}

/* A channel set while the converter is down never converts */
static void test_set_powered_down(void)
{
    QTestState *qts = sadc_init();

    qtest_writel(qts, ADENA, POWER | 1);
    qtest_writel(qts, ADENA, 1);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-sadc/reset-clocks", test_reset_clocks);
    qtest_add_func("ingenic-sadc/slow", test_slow);
    qtest_add_func("ingenic-sadc/power-down-mid", test_power_down_mid);
    qtest_add_func("ingenic-sadc/set-powered-down", test_set_powered_down);
    return g_test_run();
}
