/*
 * QTest testcase for the Ingenic A1's JPEG engine registers
 *
 * What an A1N reads: JPGC +0x20 0 and +0x60 0x00110800 (a T23's read 0xf0 and
 * 0x00250600), the EFE's +0x0c and +0x18 to +0x30 0x13200000 at reset, and
 * GLBC keeping all 32 bits.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define JPEG            0x13200000
#define GLBC            0x00000
#define EFE             0x40000
#define JPGC            0xe0000

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, JPEG + off);
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(rd(qts, JPGC + 0x20), ==, 0);
    g_assert_cmphex(rd(qts, JPGC + 0x60), ==, 0x00110800);
    g_assert_cmphex(rd(qts, EFE + 0x0c), ==, 0x13200000);
    for (uint32_t off = 0x18; off <= 0x30; off += 4) {
        g_assert_cmphex(rd(qts, EFE + off), ==, 0x13200000);
    }
    g_assert_cmphex(rd(qts, EFE + 0x10), ==, 0);
    g_assert_cmphex(rd(qts, EFE + 0x14), ==, 0);
    g_assert_cmphex(rd(qts, EFE + 0x34), ==, 0);
    qtest_writel(qts, JPEG + GLBC, 0x00010200);
    g_assert_cmphex(rd(qts, GLBC), ==, 0x00010200);
    qtest_writel(qts, JPEG + GLBC, 0xffffffff);
    g_assert_cmphex(rd(qts, GLBC), ==, 0xffffffff);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/a1-jpeg/regs", test_regs);
    return g_test_run();
}
