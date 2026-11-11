/*
 * QTest testcase for the Ingenic DDR windows beside the controller
 *
 * DDRC1 at 0x13010000 (and 0x13020000 on the T20) reads zero and keeps
 * nothing. The APB page at 0x13012000 repeats every 0x100: bits that keep
 * a write, bits that read the inverse of the bit below, read-only words
 * that change once the SPL writes the controller, and on the T40, T41 and
 * A1 a bit that any write clears. Values are as a T20X, T31X, T40XP,
 * T41NQ and A1N read them at the bootrom and at U-Boot. The T32Pro's DDR
 * answers at 0x80000000 too (its samples map frames there).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DDRC1           0x13010000
#define APB             0x13012000
#define DDRC            0x134f0000

static void test_ddrc1(const void *data)
{
    QTestState *qts = qtest_init(data);

    for (uint32_t o = 0; o < 0x1000; o += 0x3fc) {
        qtest_writel(qts, DDRC1 + o, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, DDRC1 + o), ==, 0);
    }
    if (strstr(data, "t20")) {
        qtest_writel(qts, 0x13020000, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, 0x13020000), ==, 0);
    }
    qtest_quit(qts);
}

static void test_apb_t31(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    g_assert_cmphex(qtest_readl(qts, APB + 0x04), ==, 0x3c7);
    g_assert_cmphex(qtest_readl(qts, APB + 0x84), ==, 0x04000400);
    g_assert_cmphex(qtest_readl(qts, APB + 0xe84), ==, 0x04000400);

    /* bits 9 and 13 read as the inverse of bits 8 and 12 */
    qtest_writel(qts, APB + 0x34, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, APB + 0x34), ==, 0x1113);
    g_assert_cmphex(qtest_readl(qts, APB + 0x534), ==, 0x1113);
    qtest_writel(qts, APB + 0x34, 0);
    g_assert_cmphex(qtest_readl(qts, APB + 0x34), ==, 0x2200);
    qtest_writel(qts, APB + 0x34, 0x2202);
    g_assert_cmphex(qtest_readl(qts, APB + 0x34), ==, 0x2202);

    /* the PHY init handshake: no PLL lock before the controller is set up */
    g_assert_cmphex(qtest_readl(qts, APB + 0x8c), ==, 1);
    qtest_writel(qts, APB + 0x8c, 0);
    g_assert_cmphex(qtest_readl(qts, APB + 0x8c), ==, 2);
    qtest_writel(qts, DDRC + 0x08, 0);
    g_assert_cmphex(qtest_readl(qts, APB + 0x8c), ==, 6);
    g_assert_cmphex(qtest_readl(qts, APB + 0x04), ==, 0x3c5);
    g_assert_cmphex(qtest_readl(qts, APB + 0x10), ==, 0x222);
    qtest_quit(qts);
}

static void test_apb_t20(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t20");

    /* no inno PHY: 0x8c is read-only */
    qtest_writel(qts, APB + 0x8c, 0);
    g_assert_cmphex(qtest_readl(qts, APB + 0x8c), ==, 0x510);
    g_assert_cmphex(qtest_readl(qts, APB + 0x90), ==, 0);
    qtest_writel(qts, DDRC + 0x08, 0);
    g_assert_cmphex(qtest_readl(qts, APB + 0x90), ==, 0x42000000);
    g_assert_cmphex(qtest_readl(qts, APB + 0xf0), ==, 0x42000000);
    g_assert_cmphex(qtest_readl(qts, APB + 0x18), ==, 0x1012);
    qtest_quit(qts);
}

static void test_apb_xburst2(const void *data)
{
    QTestState *qts = qtest_init(data);
    bool t41 = strstr(data, "t41");

    g_assert_cmphex(qtest_readl(qts, APB + 0x00), ==, t41 ? 8 : 9);
    g_assert_cmphex(qtest_readl(qts, APB + 0x08), ==, 0x03020100);
    g_assert_cmphex(qtest_readl(qts, APB + 0x44), ==, 0xfef9f9e4);
    qtest_writel(qts, APB + 0x08, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, APB + 0x708), ==, 0x1f1f1f1f);

    /* PREGPRO: bit 1 set at reset, gone after any write */
    g_assert_cmphex(qtest_readl(qts, APB + 0x6c), ==, 2);
    qtest_writel(qts, APB + 0x6c, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, APB + 0x6c), ==, 1);
    qtest_writel(qts, APB + 0x6c, 2);
    g_assert_cmphex(qtest_readl(qts, APB + 0x6c), ==, 0);

    /* DWSTATUS: DFI init done once the controller is set up */
    g_assert_cmphex(qtest_readl(qts, APB + 0x04), ==, 0);
    qtest_writel(qts, DDRC + 0x10, 0);
    g_assert_cmphex(qtest_readl(qts, APB + 0x04), ==, 3);
    qtest_quit(qts);
}

/*
 * Above the RAM, as far as the DDR window, reads give 0 and writes go
 * nowhere, without a bus error, as a T23DL (32 MB), T31X and T40N read it
 */
static void test_hole(const void *data)
{
    QTestState *qts = qtest_init(data);
    g_autofree char *mtree = NULL;

    qtest_writel(qts, 0x01f18000, 0x12345678);
    qtest_writel(qts, 0x07f18000, 0x55aa55aa);
    g_assert_cmphex(qtest_readl(qts, 0x07f18000), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x01f18000), ==, 0x12345678);
    mtree = qtest_hmp(qts, "info mtree");
    g_assert(strstr(mtree, "ingenic-ddr-hole"));
    qtest_quit(qts);
}

static void test_t32pro_high(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 -m 128M "
                                 "-global ingenic-t31.soc-variant=t32prozn");

    qtest_writel(qts, 0x84000000, 0x5a5a1234);
    g_assert_cmphex(qtest_readl(qts, 0x04000000), ==, 0x5a5a1234);
    qtest_writel(qts, 0x00100000, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0x80100000), ==, 0x12345678);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    static const char *const machines[] = {
        "-machine ingenic-t20", "-machine ingenic-t31", "-machine ingenic-t32",
        "-machine ingenic-t40", "-machine ingenic-t41", "-machine ingenic-a1",
    };

    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(machines); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-ddr/ddrc1/%s",
                                                machines[i] + 17);

        qtest_add_data_func(path, machines[i], test_ddrc1);
    }
    qtest_add_func("ingenic-ddr/apb/t31", test_apb_t31);
    qtest_add_func("ingenic-ddr/apb/t20", test_apb_t20);
    qtest_add_func("ingenic-ddr/t32pro-high", test_t32pro_high);
    qtest_add_data_func("ingenic-ddr/hole/t23", "-machine ingenic-t23 -m 32M",
                        test_hole);
    qtest_add_data_func("ingenic-ddr/hole/t40", "-machine ingenic-t40 -m 64M",
                        test_hole);
    qtest_add_data_func("ingenic-ddr/apb/t40", "-machine ingenic-t40",
                        test_apb_xburst2);
    qtest_add_data_func("ingenic-ddr/apb/t41", "-machine ingenic-t41",
                        test_apb_xburst2);
    qtest_add_data_func("ingenic-ddr/apb/a1", "-machine ingenic-a1",
                        test_apb_xburst2);
    return g_test_run();
}
