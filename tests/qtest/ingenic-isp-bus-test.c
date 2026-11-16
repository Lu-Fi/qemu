/*
 * QTest testcase for a T41's ISP on the bus clocks that hang it
 *
 * Measured on a T41NQ at U-Boot with CLKGR bit 23 open: ISPMCDR (CPM 0x80)
 * clocks the core's registers (offsets below 0xf0000), ISPSCDR (0xa8) the
 * MSCA's (0xf0000 to 0xfefff). A stopped divider hangs its first access;
 * MPLL undivided (a divider write without CE: the mux follows, the divider
 * stays) hangs the third access to the core and the seventh to the MSCA.
 * SCLKA undivided, MPLL halved and a stopped ISPACDR leave the ISP alone, and
 * a shut CPM gate completes every access.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define CPM_CLKGR   0x10000020
#define CPM_ISPM    0x10000080
#define CPM_ISPS    0x100000a8
#define CPM_ISPA    0x100000ac
#define ISP_CORE    0x13300000
#define ISP_MSCA    0x133f0000

#define CE          (1u << 29)
#define STOP        (1u << 27)
#define MPLL        (1u << 30)

static QTestState *start(bool gate_open)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-global ingenic-t40.soc-variant=t41nq");

    /* the PLLs and CPCCR as a T41NQ's U-Boot leaves them: APLL 1104 MHz */
    /* (SCLKA), MPLL 1400 MHz */
    qtest_writel(qts, 0x10000010, 0x05b008bd);
    qtest_writel(qts, 0x10000014, 0x15d0889d);
    qtest_writel(qts, 0x10000000, 0x9a0b5520);
    if (gate_open) {
        qtest_writel(qts, CPM_CLKGR, qtest_readl(qts, CPM_CLKGR) & ~(1u << 23));
    }
    return qts;
}

static bool hung(QTestState *qts)
{
    QDict *r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                         "{'path': '/machine/soc/isp', "
                         "'property': 'bus-hung'}}");
    bool v = qdict_get_bool(r, "return");

    qobject_unref(r);
    return v;
}

/* @n accesses from @base; the number that completed before a hang, or n */
static int reads_until_hang(QTestState *qts, uint32_t base, int n)
{
    for (int i = 0; i < n; i++) {
        qtest_readl(qts, base + 4 * i);
        if (hung(qts)) {
            return i;
        }
    }
    return n;
}

static void test_stopped(void)
{
    QTestState *qts = start(true);

    qtest_writel(qts, CPM_ISPS, MPLL | CE | 1);
    qtest_writel(qts, CPM_ISPM, MPLL | CE | STOP | 1);
    g_assert_cmpint(reads_until_hang(qts, ISP_MSCA, 8), ==, 8);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 8), ==, 0);
    qtest_quit(qts);

    qts = start(true);
    qtest_writel(qts, CPM_ISPM, MPLL | CE | 1);
    qtest_writel(qts, CPM_ISPS, MPLL | CE | STOP | 1);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 8), ==, 8);
    g_assert_cmpint(reads_until_hang(qts, ISP_MSCA, 8), ==, 0);
    qtest_quit(qts);

    /* a write hangs it as well */
    qts = start(true);
    qtest_writel(qts, CPM_ISPM, MPLL | CE | STOP | 1);
    qtest_writel(qts, ISP_CORE + 0x1000, 0);
    g_assert_true(hung(qts));
    qtest_quit(qts);
}

static void test_stop_without_ce(void)
{
    QTestState *qts = start(true);

    /* STOP without CE does not stop the clock */
    qtest_writel(qts, CPM_ISPM, MPLL | CE | 1);
    qtest_writel(qts, CPM_ISPM, MPLL | STOP | 1);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 16), ==, 16);
    qtest_quit(qts);
}

static void test_too_fast(void)
{
    QTestState *qts = start(true);

    /* the mux follows the write, the divider stays at 0: MPLL undivided */
    qtest_writel(qts, CPM_ISPM, MPLL | (1u << 28) | 3);
    qtest_writel(qts, CPM_ISPS, MPLL | (1u << 28) | 3);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 8), ==, 2);
    qtest_quit(qts);

    qts = start(true);
    qtest_writel(qts, CPM_ISPS, MPLL | (1u << 28) | 3);
    g_assert_cmpint(reads_until_hang(qts, ISP_MSCA, 12), ==, 6);
    /* the core is on SCLKA undivided still: answers */
    qtest_quit(qts);

    qts = start(true);
    qtest_writel(qts, CPM_ISPM, MPLL | (1u << 28) | 3);
    qtest_writel(qts, ISP_CORE, 0);
    qtest_writel(qts, ISP_CORE, 0);
    g_assert_false(hung(qts));
    qtest_writel(qts, ISP_CORE, 0);
    g_assert_true(hung(qts));
    qtest_quit(qts);
}

static void test_runs(void)
{
    QTestState *qts = start(true);

    /* as a driver sets them: through CE, MPLL halved; ISPACDR stopped */
    qtest_writel(qts, CPM_ISPM, MPLL | CE | 1);
    qtest_writel(qts, CPM_ISPS, MPLL | CE | 1);
    qtest_writel(qts, CPM_ISPA, MPLL | CE | STOP | 1);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 32), ==, 32);
    g_assert_cmpint(reads_until_hang(qts, ISP_MSCA, 32), ==, 32);
    qtest_quit(qts);

    /* SCLKA undivided, as the dividers come out of reset */
    qts = start(true);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 32), ==, 32);
    g_assert_cmpint(reads_until_hang(qts, ISP_MSCA, 32), ==, 32);
    qtest_quit(qts);
}

static void test_gate_shut(void)
{
    QTestState *qts = start(false);

    qtest_writel(qts, CPM_ISPM, MPLL | CE | STOP | 1);
    qtest_writel(qts, CPM_ISPS, MPLL | CE | STOP | 1);
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 8), ==, 8);
    g_assert_cmpint(reads_until_hang(qts, ISP_MSCA, 8), ==, 8);
    qtest_quit(qts);
}

static void test_reset(void)
{
    QTestState *qts = start(true);

    qtest_writel(qts, CPM_ISPM, MPLL | CE | STOP | 1);
    qtest_readl(qts, ISP_CORE);
    g_assert_true(hung(qts));
    qtest_system_reset(qts);
    g_assert_false(hung(qts));
    qtest_writel(qts, CPM_CLKGR, qtest_readl(qts, CPM_CLKGR) & ~(1u << 23));
    g_assert_cmpint(reads_until_hang(qts, ISP_CORE, 8), ==, 8);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/isp-bus/stopped", test_stopped);
    qtest_add_func("ingenic/isp-bus/stop-without-ce", test_stop_without_ce);
    qtest_add_func("ingenic/isp-bus/too-fast", test_too_fast);
    qtest_add_func("ingenic/isp-bus/runs", test_runs);
    qtest_add_func("ingenic/isp-bus/gate-shut", test_gate_shut);
    qtest_add_func("ingenic/isp-bus/reset", test_reset);
    return g_test_run();
}
