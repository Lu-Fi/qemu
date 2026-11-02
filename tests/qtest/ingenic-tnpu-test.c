/*
 * QTest testcase for the Ingenic TNPU's control unit (CCU) and window
 *
 * The T32's: resets and the bits each register keeps, LOCK and UNLOCK,
 * TOHOST as the SoC's interrupt, a release with no core linked, and TIME
 * at the TNPU clock, as a T32LQ reads them. The T32Pro's ID and window.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM             0x10000000
#define CPM_CLKGR1      0x28
#define CPM_CPCCR       0x00
#define CPM_CPAPCR      0x10
#define CPM_TNPUCDR     0x98
#define CCU             0x12700000
#define DRAM            0x12608000

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, CCU + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, CCU + off, v);
}

/* Written all ones then zero: what each reads, the reset put back after */
static void check(QTestState *qts, uint32_t off, uint32_t reset,
                  uint32_t ones, uint32_t zero)
{
    g_test_message("CCU 0x%03x", off);
    g_assert_cmphex(rd(qts, off), ==, reset);
    wr(qts, off, 0xffffffff);
    g_assert_cmphex(rd(qts, off), ==, ones);
    wr(qts, off, 0);
    g_assert_cmphex(rd(qts, off), ==, zero);
    wr(qts, off, reset);
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    g_assert_cmphex(rd(qts, 0xf0), ==, 0x80000000);
    check(qts, 0x00, 0x1e01, 0x1e21, 0x1e00);
    check(qts, 0x04, 0x12608000, 0xffffffff, 0);
    check(qts, 0x18, 0xffffffff, 0xffffffff, 0);
    check(qts, 0x30, 0x202, 0xffffffff, 0);
    check(qts, 0x38, 0x100, 0x100, 0x100);
    check(qts, 0x58, 0x05ffffff, 0xffffffff, 0);
    check(qts, 0x60, 7, 0x3f, 0);
    check(qts, 0x78, 0x1f, 0x3f, 0);
    check(qts, 0x80, 0x002001fd, 0x81ffffff, 0);
    check(qts, 0x88, 0, 1, 0);
    check(qts, 0xfc, 1, 1, 0);
    check(qts, 0x100, 6, 0xf, 6);
    check(qts, 0x104, 0, 0, 0);
    check(qts, 0x1fc, 0, 0, 0);

    /* LOCK takes a write only while unlocked */
    wr(qts, 0xf4, 1);
    g_assert_cmphex(rd(qts, 0xf4), ==, 0x80000001);
    wr(qts, 0xf4, 0);
    g_assert_cmphex(rd(qts, 0xf4), ==, 0x80000001);
    wr(qts, 0xf8, 1);
    g_assert_cmphex(rd(qts, 0xf4), ==, 1);
    g_assert_cmphex(rd(qts, 0xf8), ==, 0);
    wr(qts, 0xf4, 0);
    g_assert_cmphex(rd(qts, 0xf4), ==, 0x80000000);
    qtest_quit(qts);
}

static void test_irq_and_release(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_irq_intercept_out_named(qts, "/machine/soc/tnpu", "sysbus-irq");
    g_assert_false(qtest_get_irq(qts, 0));
    wr(qts, 0x0c, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(qts, 0x0c, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* FROMHOST keeps its value held; no core linked, it never sleeps */
    wr(qts, 0x08, 1);
    g_assert_cmphex(rd(qts, 0x08), ==, 1);
    wr(qts, 0xf0, 0);
    g_assert_cmphex(rd(qts, 0x00), ==, 0x1c11);
    wr(qts, 0xf0, 0x80000000);
    g_assert_cmphex(rd(qts, 0x00), ==, 0x1e01);
    qtest_quit(qts);
}

/* TIME at SCLK_A (the APLL's 900 MHz, as U-Boot sets them) over TNPUCDR's 2 */
static void test_time(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");
    uint32_t t;

    qtest_writel(qts, CPM + CPM_CPAPCR, 0x04b0510d);
    qtest_writel(qts, CPM + CPM_CPCCR, 0x9a7b5510);
    qtest_writel(qts, CPM + CPM_TNPUCDR, 1u << 29 | 1);
    qtest_writel(qts, CPM + CPM_CLKGR1,
                 qtest_readl(qts, CPM + CPM_CLKGR1) & ~(1u << 11));
    wr(qts, 0x10, 0);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(rd(qts, 0x10), ==, 0);
    wr(qts, 0x00, rd(qts, 0x00) | 1 << 5);
    qtest_clock_step(qts, 1000000);
    t = rd(qts, 0x10);
    g_assert_cmpuint(t, >=, 449999);
    g_assert_cmpuint(t, <=, 450001);
    qtest_quit(qts);
}

static void test_pro(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 "
                                 "-global ingenic-t31.soc-variant=t32prozl");

    g_assert_cmphex(rd(qts, 0x38), ==, 0x150);
    g_assert_cmphex(rd(qts, 0x04), ==, 0x150b0000);
    qtest_writel(qts, 0x150b1ffc, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0x150b1ffc), ==, 0x12345678);
    qtest_quit(qts);
}

/* The T32's window is RAM the driver loads the boot code into */
static void test_window(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_writel(qts, DRAM + 0xffc, 0xfedcba98);
    g_assert_cmphex(qtest_readl(qts, DRAM + 0xffc), ==, 0xfedcba98);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-tnpu/regs", test_regs);
    qtest_add_func("ingenic-tnpu/irq-and-release", test_irq_and_release);
    qtest_add_func("ingenic-tnpu/time", test_time);
    qtest_add_func("ingenic-tnpu/pro", test_pro);
    qtest_add_func("ingenic-tnpu/window", test_window);
    return g_test_run();
}
