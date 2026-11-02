/*
 * QTest testcase for the Ingenic MIPI CSI-2 receivers
 *
 * Each part's hosts where its drivers find them, out of reset as the part
 * reads them and keeping the bits it keeps of a write, one KiB of registers
 * repeated across the window: the T20's, T30's and T31's plain DesignWare
 * host, the T23's, T41's and the T32's and T40's second hosts with lane
 * maps, and the T32's and T40's four-lane ones. PHY_STATE reads 0x200 at
 * rest; out of DPHY_RSTZ and CSI2_RESETN its clock lane reads in high speed
 * and the lanes past N_LANES stopped. DPHY_RSTZ lets the D-PHY go. The T10
 * has no host.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define N_LANES         0x04
#define DPHY_RSTZ       0x0c
#define CSI2_RESETN     0x10
#define PHY_STATE       0x14
#define MASK1           0x28
#define PHY             0x10022000

typedef struct {
    const char *name;
    const char *machine;
    uint32_t base;
    unsigned lanes;
    uint32_t map[4];            /* 0x110 to 0x11c out of reset */
    bool ext;                   /* the lane maps and 0x100 */
    bool phy;                   /* lets the D-PHY go */
    uint32_t phy_reg;           /* a D-PHY register reading 3 */
} CsiCase;

static const CsiCase cases[] = {
    { "t20", "ingenic-t20", 0x10040000, 2 },
    { "t30", "ingenic-t30", 0x10040000, 2, .phy = true },
    { "t31", "ingenic-t31", 0x10023000, 2, .phy = true },
    { "t23", "ingenic-t23", 0x10023000, 2, { 0, 0, 0xeeaa6622, 0x3377bbff },
      true, true },
    { "t32-4", "ingenic-t32", 0x10054000, 4, .ext = true, .phy = true },
    { "t32-2", "ingenic-t32", 0x10023000, 2,
      { 0, 0, 0xeeaa6622, 0x3377bbff }, true },
    { "t40-4", "ingenic-t40", 0x10054000, 4, .ext = true, .phy = true },
    { "t40-2", "ingenic-t40", 0x10023000, 2,
      { 0xcc884400, 0x115599dd, 0xeeaa6622, 0x3377bbff }, true },
    { "t41", "ingenic-t41", 0x10023000, 2, { 0, 0, 0xeeaa6622, 0x3377bbff },
      true, true, 0x440 },
};

static void test_csi(const void *data)
{
    const CsiCase *c = data;
    QTestState *qts = qtest_initf("-machine %s", c->machine);
    uint32_t b = c->base;
    uint32_t phy = PHY + (c->phy_reg ? c->phy_reg : 0x040);

    g_assert_cmphex(qtest_readl(qts, b), ==, 0x3130322a);
    g_assert_cmphex(qtest_readl(qts, b + N_LANES), ==, c->lanes - 1);
    g_assert_cmphex(qtest_readl(qts, b + PHY_STATE), ==, 0x200);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, b + 0x110 + 4 * i), ==, c->map[i]);
    }

    /* What a write keeps, through the window's last KiB */
    qtest_writel(qts, b + 0xc00 + N_LANES, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, b + N_LANES), ==, c->lanes - 1);
    qtest_writel(qts, b + MASK1, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, b + MASK1), ==,
                    c->lanes == 4 ? 0x1fffffff : 0x1ffffff3);
    qtest_writel(qts, b + 0x100, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, b + 0x100), ==,
                    c->ext ? (c->lanes == 4 ? 0xf : 0x3) : 0);
    qtest_writel(qts, b, 0);
    g_assert_cmphex(qtest_readl(qts, b), ==, 0x3130322a);

    /* Two lanes received, the rest stopped */
    qtest_writel(qts, b + N_LANES, 1);
    qtest_writel(qts, b + DPHY_RSTZ, 1);
    g_assert_cmphex(qtest_readl(qts, b + PHY_STATE), ==, 0x200);
    if (c->phy) {
        g_assert_cmphex(qtest_readl(qts, phy), ==, 0x03);
    }
    qtest_writel(qts, b + CSI2_RESETN, 1);
    g_assert_cmphex(qtest_readl(qts, b + PHY_STATE), ==,
                    c->lanes == 4 ? 0x3c0 : 0x300);
    qtest_writel(qts, b + DPHY_RSTZ, 0);
    g_assert_cmphex(qtest_readl(qts, b + PHY_STATE), ==, 0x200);
    if (c->phy) {
        g_assert_cmphex(qtest_readl(qts, phy), ==, 0);
    }
    qtest_quit(qts);
}

static void test_t10(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t10");

    g_assert_cmphex(qtest_readl(qts, 0x10040000), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-csi/%s",
                                                cases[i].name);

        qtest_add_data_func(path, &cases[i], test_csi);
    }
    qtest_add_func("ingenic-csi/t10", test_t10);
    return g_test_run();
}
