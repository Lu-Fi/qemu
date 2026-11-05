/*
 * QTest testcase for the Ingenic static memory controller (NEMC, NFI)
 *
 * The register file as a T20X, T30X and T31X read it at U-Boot, alike on
 * all three: 0x00 and bank 0's control read-only, banks 1 to 6's control
 * and banks 0 to 6's address words, NFCSR's twelve bits, nothing else in
 * the 64 KiB; the bank windows read 0 with no device on the bus. The other
 * parts carry the same file, unmeasured.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEMC            0x13410000

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
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset);
}

static void test_regs(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);
    /* the T31 family's boards put RAM in CS2 for bootrom work */
    bool cs2_ram = !g_str_equal(machine, "ingenic-t40") &&
                   !g_str_equal(machine, "ingenic-t41") &&
                   !g_str_equal(machine, "ingenic-a1");

    check(qts, NEMC + 0x00, 0x00000001, 0);
    check(qts, NEMC + 0x10, 0x00000080, 0);
    for (int n = 1; n <= 6; n++) {
        check(qts, NEMC + 0x10 + 4 * n, 0x0fff7700, 0x3fffffc7);
    }
    check(qts, NEMC + 0x30, 0x00001cfc, 0x0000ffff);
    for (int n = 1; n <= 6; n++) {
        check(qts, NEMC + 0x30 + 4 * n, ((0x1c - n) << 8) | 0xff, 0x0000ffff);
    }
    check(qts, NEMC + 0x50, 0, 0x00000fff);
    for (uint32_t o = 0x04; o < 0x10; o += 4) {
        check(qts, NEMC + o, 0, 0);
    }
    check(qts, NEMC + 0x2c, 0, 0);
    check(qts, NEMC + 0x4c, 0, 0);
    for (uint32_t o = 0x54; o < 0x10000; o += 0x4c4) {
        check(qts, NEMC + o, 0, 0);
    }
    check(qts, NEMC + 0xfffc, 0, 0);

    for (int n = 1; n <= 6; n++) {
        uint32_t bank = (0x1c - n) << 24;

        if (n == 2 && cs2_ram) {
            continue;
        }
        g_test_message("CS%d", n);
        for (uint32_t o = 0; o < 0x01000000; o += 0x400000) {
            qtest_writel(qts, bank + o, 0x5a5a5a5a);
            g_assert_cmphex(qtest_readl(qts, bank + o), ==, 0);
        }
        g_assert_cmphex(qtest_readl(qts, bank + 0xfffffc), ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const char *const machines[] = {
        "ingenic-t10", "ingenic-t20", "ingenic-t21", "ingenic-t23",
        "ingenic-t30", "ingenic-t31", "ingenic-t32", "ingenic-t33",
        "ingenic-t40", "ingenic-t41", "ingenic-a1",
    };

    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(machines); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-nemc/%s",
                                                machines[i] + 8);

        qtest_add_data_func(path, machines[i], test_regs);
    }
    return g_test_run();
}
