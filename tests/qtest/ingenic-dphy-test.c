/*
 * QTest testcase for the Ingenic MIPI D-PHY
 *
 * The PHY reads 0 and keeps no write while the CSI host's DPHY_RSTZ holds
 * it; let go, its registers read as a T23N, T31X and T32LQ read them, the
 * KiB repeating across the window, and holding it again puts back what was
 * written; PHY_SHUTDOWNZ and CSI2_RESETN do not. The T30's, T40's and T41's
 * CSI hosts are not modelled, so their PHYs stay held.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define PHY             0x10022000
#define PHY_SHUTDOWNZ   0x08
#define DPHY_RSTZ       0x0c
#define CSI2_RESETN     0x10

typedef struct {
    const char *machine;
    uint32_t host;              /* the CSI host; 0: not modelled */
    uint8_t r380;               /* the T23's extra block */
} DphyCase;

static const DphyCase cases[] = {
    { "ingenic-t23", 0x10023000, 0xfd },
    { "ingenic-t31", 0x10023000, 0x00 },
    { "ingenic-t32", 0x10054000, 0x00 },
    { "ingenic-t33", 0x10054000, 0x00 },
    { "ingenic-t30", 0 },
    { "ingenic-t40", 0 },
    { "ingenic-t41", 0 },
};

static void test_dphy(const void *data)
{
    const DphyCase *c = data;
    QTestState *qts = qtest_initf("-machine %s", c->machine);

    /* Held: nothing reads back, nothing is kept */
    qtest_writel(qts, PHY + 0x440, 0x5a);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x040), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x440), ==, 0);
    if (!c->host) {
        qtest_quit(qts);
        return;
    }

    qtest_writel(qts, c->host + DPHY_RSTZ, 1);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x000), ==, 0x01);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x040), ==, 0x03);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x080), ==, 0x1f);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x128), ==, 0x0f);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x380), ==, c->r380);
    g_assert_cmphex(qtest_readb(qts, PHY + 0x041), ==, 0);

    /* The lane status keeps no write */
    qtest_writel(qts, PHY + 0x174, 0);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x174), ==, 0x03);

    /* One KiB of registers behind the whole window */
    qtest_writel(qts, PHY + 0xc40, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x040), ==, 0xff);

    /* PHY_SHUTDOWNZ and CSI2_RESETN leave it; DPHY_RSTZ resets it */
    qtest_writel(qts, c->host + PHY_SHUTDOWNZ, 1);
    qtest_writel(qts, c->host + PHY_SHUTDOWNZ, 0);
    qtest_writel(qts, c->host + CSI2_RESETN, 1);
    qtest_writel(qts, c->host + CSI2_RESETN, 0);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x040), ==, 0xff);
    qtest_writel(qts, c->host + DPHY_RSTZ, 0);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x040), ==, 0);
    qtest_writel(qts, c->host + DPHY_RSTZ, 1);
    g_assert_cmphex(qtest_readl(qts, PHY + 0x040), ==, 0x03);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-dphy/%s",
                                                cases[i].machine + 8);

        qtest_add_data_func(path, &cases[i], test_dphy);
    }
    return g_test_run();
}
