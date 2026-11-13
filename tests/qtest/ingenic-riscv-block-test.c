/*
 * QTest testcase for the Ingenic RISC-V core's block on the T41 and the A1
 *
 * What the control word, the words that read 0 and the PMA words read and
 * keep, as a T41NQ and an A1N hold them at their U-Boot prompts.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define BLOCK   0x12a00000

/* Reads reset, then all ones written reads reset | mask */
static void check(QTestState *qts, uint32_t off, uint32_t reset,
                  uint32_t mask)
{
    g_test_message("0x%03x", off);
    g_assert_cmphex(qtest_readl(qts, BLOCK + off), ==, reset);
    qtest_writel(qts, BLOCK + off, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, BLOCK + off), ==, reset | mask);
    qtest_writel(qts, BLOCK + off, 0);
    g_assert_cmphex(qtest_readl(qts, BLOCK + off), ==, reset & ~mask);
    qtest_writel(qts, BLOCK + off, reset);
}

static void check_block(QTestState *qts)
{
    check(qts, 0x04, 0, 0xffffffff);
    check(qts, 0x20, 0, 0xffffffff);
    check(qts, 0x24, 0, 0xffffffff);
    /* a hole */
    check(qts, 0x28, 0, 0);
    check(qts, 0x3c, 0, 0);
    check(qts, 0x40, 0x04000000, 0xffffffff);
    check(qts, 0x44, 0x05ffffff, 0xffffffff);
    check(qts, 0x48, 0x1fffffff, 0xffffffff);
    check(qts, 0x4c, 0, 0xffffffff);
    /* the 14 bits a PMA config word keeps */
    check(qts, 0x60, 0x1f01, 0x3fff);
    check(qts, 0x64, 0x2e03, 0x3fff);
    check(qts, 0x68, 0x1f01, 0x3fff);
    check(qts, 0x6c, 0x1e01, 0x3fff);
    check(qts, 0x80, 0x1e01, 0);
    check(qts, 0xfc, 0x1e01, 0);

    /* the PMA words again 0x10 further on, the block every 0x100 */
    qtest_writel(qts, BLOCK + 0x50, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, BLOCK + 0x40), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, BLOCK + 0x150), ==, 0x12345678);
    qtest_writel(qts, BLOCK + 0x40, 0x04000000);
    qtest_writel(qts, BLOCK + 0x70, 0x3333);
    g_assert_cmphex(qtest_readl(qts, BLOCK + 0x60), ==, 0x3333);
    qtest_writel(qts, BLOCK + 0x60, 0x1f01);
}

static void test_t41(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    /* running from reset: bit 4 clear whatever bit 1 says */
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x27);
    qtest_writel(qts, BLOCK, 0);
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x04);
    qtest_writel(qts, BLOCK, 2);
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x06);
    qtest_writel(qts, BLOCK, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x27);
    check_block(qts);
    qtest_quit(qts);
}

static void test_a1(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    /* held by the CCU: bit 4 set */
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x37);
    qtest_writel(qts, BLOCK, 0);
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x14);
    qtest_writel(qts, BLOCK, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, BLOCK), ==, 0x37);
    check_block(qts);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-riscv-block/t41", test_t41);
    qtest_add_func("ingenic-riscv-block/a1", test_a1);
    return g_test_run();
}
