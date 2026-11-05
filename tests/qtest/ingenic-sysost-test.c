/*
 * QTest testcase for the Ingenic System OST and the T21's RISC-V window
 *
 * As a T21N reads them: its two System OSTs (0x12000000, repeating over
 * 1 MiB, and 0x12220000, over 64 KiB), their reset values, prescalers
 * (counting EXCLK, 24 MHz), enables and full flag; and the empty window its
 * RISC-V clock gates at 0x12210000. A T31 counts at the 1.5 MHz its kernel
 * sets up.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define OST0            0x12000000
#define OST1            0x12220000
#define RV_WINDOW       0x12210000
#define CPM_CLKGR0      0x10000020
#define CLKGR0_RISCV    (1u << 26)

#define TCCR            0x00
#define TER             0x04
#define TCR             0x08
#define TFR             0x0c
#define TMR             0x10
#define T1DFR           0x14
#define T1CNT           0x18
#define T2CNTH          0x1c
#define T2CNTL          0x20
#define T2CNTHBUF       0x24
#define TESR            0x34
#define TECR            0x38

#define MS              1000000

static void check_reset(QTestState *qts, uint32_t base)
{
    for (unsigned off = 0; off < 0x40; off += 4) {
        uint32_t want = off == TMR ? 1 : off == T1DFR ? 0xffffffff : 0;

        g_assert_cmphex(qtest_readl(qts, base + off), ==, want);
    }
}

static void test_reset(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21");

    check_reset(qts, OST0);
    check_reset(qts, OST1);
    qtest_quit(qts);
}

/* The registers repeat every 0x100; the two timers are apart */
static void test_alias(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21");

    qtest_writel(qts, OST0 + T1DFR, 0xabc);
    g_assert_cmphex(qtest_readl(qts, OST0 + 0x10000 + T1DFR), ==, 0xabc);
    g_assert_cmphex(qtest_readl(qts, OST0 + 0xf0100 + T1DFR), ==, 0xabc);
    g_assert_cmphex(qtest_readl(qts, OST1 + T1DFR), ==, 0xffffffff);
    qtest_writel(qts, OST1 + 0xf000 + T1DFR, 0xdef);
    g_assert_cmphex(qtest_readl(qts, OST1 + 0x1000 + T1DFR), ==, 0xdef);
    g_assert_cmphex(qtest_readl(qts, OST0 + T1DFR), ==, 0xabc);
    /* Where the T31's control page is, the OST again */
    g_assert_cmphex(qtest_readl(qts, OST1 + 0x1010), ==, 1);
    g_assert_cmphex(qtest_readl(qts, OST1 + 0x10000 + TMR), ==, 0);
    qtest_quit(qts);
}

/* Bits 1:0 and 3:2 divide EXCLK for T1 and T2: /1, /4, /16, /1 */
static void test_prescale(void)
{
    static const struct {
        uint32_t tccr;
        uint32_t t1, t2;            /* counts in a millisecond */
    } rates[] = {
        { 0x0, 24000, 24000 },
        { 0x5, 6000, 6000 },
        { 0xa, 1500, 1500 },
        { 0x6, 1500, 6000 },
        { 0xf, 24000, 24000 },
    };
    QTestState *qts = qtest_init("-machine ingenic-t21");

    qtest_writel(qts, OST1 + TCCR, 0xfffffff0);
    g_assert_cmphex(qtest_readl(qts, OST1 + TCCR), ==, 0);
    for (unsigned i = 0; i < ARRAY_SIZE(rates); i++) {
        qtest_writel(qts, OST1 + TECR, 3);
        qtest_writel(qts, OST1 + TCCR, rates[i].tccr);
        qtest_writel(qts, OST1 + TCR, 3);
        qtest_writel(qts, OST1 + TESR, 3);
        g_assert_cmphex(qtest_readl(qts, OST1 + TER), ==, 3);
        qtest_clock_step(qts, MS);
        g_assert_cmpuint(qtest_readl(qts, OST1 + T1CNT), ==, rates[i].t1);
        g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTL), ==, rates[i].t2);
    }
    qtest_quit(qts);
}

/* A stopped count holds and resumes; TCR clears it */
static void test_hold(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21");

    qtest_writel(qts, OST1 + TESR, 2);
    qtest_clock_step(qts, MS);
    qtest_writel(qts, OST1 + TECR, 2);
    g_assert_cmphex(qtest_readl(qts, OST1 + TER), ==, 0);
    qtest_clock_step(qts, MS);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTL), ==, 24000);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T1CNT), ==, 0);
    qtest_writel(qts, OST1 + TESR, 2);
    qtest_clock_step(qts, MS);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTL), ==, 48000);
    qtest_writel(qts, OST1 + TCR, 2);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTL), ==, 0);

    /* 64 bits, the upper half latched by a read of the lower */
    qtest_writel(qts, OST1 + TCR, 2);
    qtest_clock_step(qts, 180 * 1000 * (uint64_t)MS);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTH), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTHBUF), ==, 0);
    qtest_readl(qts, OST1 + T2CNTL);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T2CNTHBUF), ==, 1);
    qtest_quit(qts);
}

/* T1 counts 0 to T1DFR and over, flagging T1DFR */
static void test_full(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21");

    qtest_writel(qts, OST1 + T1DFR, 23999);
    qtest_writel(qts, OST1 + TMR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, OST1 + TMR), ==, 1);
    qtest_writel(qts, OST1 + TMR, 0);
    qtest_writel(qts, OST1 + TESR, 1);
    qtest_clock_step(qts, MS / 2);
    g_assert_cmphex(qtest_readl(qts, OST1 + TFR), ==, 0);
    qtest_clock_step(qts, MS / 2);
    g_assert_cmphex(qtest_readl(qts, OST1 + TFR), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T1CNT), <, 100);
    qtest_writel(qts, OST1 + TFR, 0);
    g_assert_cmphex(qtest_readl(qts, OST1 + TFR), ==, 0);
    qtest_clock_step(qts, MS + MS / 2);
    g_assert_cmphex(qtest_readl(qts, OST1 + TFR), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, OST1 + T1CNT), ==, 12000);
    qtest_quit(qts);
}

/* The kernels set /16 on both channels: 1.5 MHz, as on a T31 */
static void test_t31_kernel_rate(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    check_reset(qts, OST0);
    qtest_writel(qts, OST0 + TCCR, 0xa);
    qtest_writel(qts, OST0 + TCR, 3);
    qtest_writel(qts, OST0 + TESR, 3);
    qtest_clock_step(qts, MS);
    g_assert_cmpuint(qtest_readl(qts, OST0 + T2CNTL), ==, 1500);
    g_assert_cmpuint(qtest_readl(qts, OST0 + T1CNT), ==, 1500);
    qtest_quit(qts);
}

/* The T21's window where the T31's SRAM is: nothing behind it */
static void test_t21_riscv_window(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21");
    uint32_t clkgr0 = qtest_readl(qts, CPM_CLKGR0);

    g_assert_true(clkgr0 & CLKGR0_RISCV);
    qtest_writel(qts, CPM_CLKGR0, clkgr0 & ~CLKGR0_RISCV);
    for (uint32_t off = 0; off < 0x10000; off += 0x1000) {
        qtest_writel(qts, RV_WINDOW + off, 0x11223344);
        g_assert_cmphex(qtest_readl(qts, RV_WINDOW + off), ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-sysost/t21/reset", test_reset);
    qtest_add_func("ingenic-sysost/t21/alias", test_alias);
    qtest_add_func("ingenic-sysost/t21/prescale", test_prescale);
    qtest_add_func("ingenic-sysost/t21/hold", test_hold);
    qtest_add_func("ingenic-sysost/t21/full", test_full);
    qtest_add_func("ingenic-sysost/t31/kernel-rate", test_t31_kernel_rate);
    qtest_add_func("ingenic-sysost/t21/riscv-window", test_t21_riscv_window);
    return g_test_run();
}
