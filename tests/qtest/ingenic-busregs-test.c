/*
 * QTest testcase for the Ingenic bus windows that only hold registers
 *
 * HARB0's chip ID at 0x2c of every 0x40, keeping no write; the T32's
 * HARB0, 32 words repeating every 0x80 that all keep a write. The AXI
 * arbiters' version and configuration, and the word that keeps a write.
 * The bus monitors' two kinds of channel, a T40's and an A1's repeating
 * every 4 KiB, and a T41's channels and control block. Values and the bits
 * kept are as a T31X, T32LQ, T40XP, T41NQ and A1N read them. The SDHCI's
 * DesignWare MSHC areas, the same on every part that has one: pointers at
 * 0xe0, command-queue and vendor registers from 0x180, bits that a 1 sets
 * and a 0 leaves.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define HARB0           0x13000000
#define MONITOR         0x130a0000
#define AXI_ARB1        0x13013000
#define AXI_ARB2        0x13014000

/* Reads reset, then all ones written reads kept | (reset & ~mask) */
static void check(QTestState *qts, uint32_t addr, uint32_t reset,
                  uint32_t mask)
{
    g_test_message("0x%08x", addr);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset);
    qtest_writel(qts, addr, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset | mask);
    qtest_writel(qts, addr, 0);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset & ~mask);
    qtest_writel(qts, addr, reset);
}

static void test_harb0(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    check(qts, HARB0 + 0x2c, 0x10031003, 0);
    check(qts, HARB0 + 0xfec, 0x10031003, 0);
    check(qts, HARB0 + 0x30, 0, 0);
    qtest_quit(qts);
}

static void test_harb0_t32(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    check(qts, HARB0 + 0x2c, 0x10032004, 0xffffffff);
    check(qts, HARB0 + 0x30, 0xee, 0xffffffff);
    check(qts, HARB0 + 0x70, 0xffdd, 0xffffffff);
    check(qts, HARB0 + 0x00, 0, 0);
    qtest_writel(qts, HARB0 + 0x04, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB0 + 0xf84), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB0 + 0x6c), ==, 0);
    qtest_quit(qts);
}

static void test_t40(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    /* A channel, then the other kind at 0x500, repeating every 4 KiB */
    check(qts, MONITOR + 0x000, 0xff00, 0xe007ff07);
    check(qts, MONITOR + 0x608, 0, 0x80ff80ff);
    check(qts, MONITOR + 0x474, 0x7f, 0x7f);
    check(qts, MONITOR + 0x500, 0, 0xf000f003);
    check(qts, MONITOR + 0x574, 3, 3);
    check(qts, MONITOR + 0x700, 0, 0);
    g_assert_cmphex(qtest_readl(qts, MONITOR + 0xf000), ==, 0xff00);
    check(qts, AXI_ARB1 + 0x000, 0x3430302a, 0);
    check(qts, AXI_ARB1 + 0x004, 0x00102001, 0);
    check(qts, AXI_ARB1 + 0x00c, 0, 0xffffffff);
    check(qts, AXI_ARB2 + 0x004, 0x00105001, 0);
    /* Below GPIO port A, as a T40XP reads it: 0, and stores are dropped */
    for (unsigned off = 0; off < 0x100; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x1000f000 + off), ==, 0);
    }
    qtest_writel(qts, 0x1000f018, 0x80000000);
    qtest_writel(qts, 0x1000f044, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, 0x1000f018), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x1000f044), ==, 0);
    qtest_quit(qts);
}

static void test_t41(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    check(qts, MONITOR + 0x0020, 0x200, 0xffff);
    check(qts, MONITOR + 0x1824, 0x200, 0xffff);
    check(qts, MONITOR + 0x1924, 0x10000000, 0xffffffff);
    check(qts, MONITOR + 0x1400, 0, 0xffffffff);
    check(qts, MONITOR + 0x1420, 0, 0);
    check(qts, MONITOR + 0x2300, 0xffffffff, 0xffffffff);
    check(qts, MONITOR + 0x2400, 0, 0);
    qtest_quit(qts);
}

static void test_a1(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    check(qts, MONITOR + 0x700, 0, 0xf000f003);
    check(qts, MONITOR + 0x774, 3, 3);
    check(qts, MONITOR + 0x60c, 0xfff, 0xfff);
    check(qts, AXI_ARB2 + 0x000, 0x3430302a, 0);
    qtest_quit(qts);
}

static void test_mshc(const void *data)
{
    QTestState *qts = qtest_initf("-machine %s", (const char *)data);

    for (uint32_t base = 0x13060000; base <= 0x13070000; base += 0x10000) {
        check(qts, base + 0xe0, 0x0f580f48, 0);
        check(qts, base + 0xe8, 0x01800500, 0);
        check(qts, base + 0x180, 0x510, 0);
        check(qts, base + 0x1c0, 0x11000, 0xfffff);
        check(qts, base + 0x500, 0x3134302a, 0);
        check(qts, base + 0x510, 0x0303000f, 0x0303030f);
        check(qts, base + 0xf58, 0x40000, 0);
        /* 0x19c's low bits stay once set */
        qtest_writel(qts, base + 0x19c, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, base + 0x19c), ==, 0x80001f7f);
        qtest_writel(qts, base + 0x19c, 0);
        g_assert_cmphex(qtest_readl(qts, base + 0x19c), ==, 0x1f7f);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-busregs/harb0", test_harb0);
    qtest_add_func("ingenic-busregs/harb0-t32", test_harb0_t32);
    qtest_add_func("ingenic-busregs/t40", test_t40);
    qtest_add_func("ingenic-busregs/t41", test_t41);
    qtest_add_func("ingenic-busregs/a1", test_a1);
    qtest_add_data_func("ingenic-busregs/mshc/t32", "ingenic-t32", test_mshc);
    qtest_add_data_func("ingenic-busregs/mshc/t41", "ingenic-t41", test_mshc);
    qtest_add_data_func("ingenic-busregs/mshc/a1", "ingenic-a1", test_mshc);
    return g_test_run();
}
