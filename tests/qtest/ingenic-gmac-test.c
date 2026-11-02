/*
 * QTest testcase for the Ingenic GMAC's MDIO bus
 *
 * The PHY at address 0 answers MDIO reads; with "phy" off, as on a board
 * with no PHY on the bus, every read returns all ones and writes go
 * nowhere, as the vendor U-Boot finds no PHY there. With "mdio-hold" too,
 * as on a T23DL's bus, an even register reads 0 and an odd one all ones.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GMAC            0x134b0000
#define MII_ADDR        0x10
#define MII_DATA        0x14
#define MII_BUSY        1
#define MII_WRITE       2
#define MII_PHYSID2     3

static uint32_t mdio_read(QTestState *qts, unsigned reg)
{
    qtest_writel(qts, GMAC + MII_ADDR, reg << 6 | MII_BUSY);
    g_assert_cmphex(qtest_readl(qts, GMAC + MII_ADDR) & MII_BUSY, ==, 0);
    return qtest_readl(qts, GMAC + MII_DATA) & 0xffff;
}

static void test_mdio(const void *data)
{
    bool phy = data != NULL;
    QTestState *qts = qtest_initf("-machine ingenic-t31 %s",
                                  phy ? "" : "-global ingenic-gmac.phy=off");

    g_assert_cmphex(mdio_read(qts, MII_PHYSID2), ==, phy ? 0x0128 : 0xffff);
    qtest_writel(qts, GMAC + MII_DATA, 0x1234);
    qtest_writel(qts, GMAC + MII_ADDR, 4 << 6 | MII_WRITE | MII_BUSY);
    g_assert_cmphex(mdio_read(qts, 4), ==, phy ? 0x1234 : 0xffff);
    qtest_quit(qts);
}

static void test_mdio_hold(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t23 "
                                 "-global ingenic-gmac.phy=off "
                                 "-global ingenic-gmac.mdio-hold=on");

    g_assert_cmphex(mdio_read(qts, 2), ==, 0);
    g_assert_cmphex(mdio_read(qts, MII_PHYSID2), ==, 0xffff);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("ingenic-gmac/mdio/phy", "", test_mdio);
    qtest_add_data_func("ingenic-gmac/mdio/no-phy", NULL, test_mdio);
    qtest_add_func("ingenic-gmac/mdio/hold", test_mdio_hold);
    return g_test_run();
}
