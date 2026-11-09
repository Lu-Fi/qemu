/*
 * QTest testcase for the Ingenic DWC2 core configuration
 *
 * Per machine, GSNPSID and GHWCFG1-4 as the part reads them, and the
 * endpoints the model honours: the device DMA address registers keep a
 * write for the endpoints GHWCFG2 and GHWCFG4 give the core and read 0
 * for the rest.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GSNPSID     0x040
#define DIEPDMA(n)  (0x914 + (n) * 0x20)
#define DOEPDMA(n)  (0xb14 + (n) * 0x20)

#define GHWCFG3_T31 0x0df80ce8
#define GHWCFG3_T32 0x0df804e8
#define GHWCFG3_A1  0x0df800e8

typedef struct {
    const char *machine;
    uint32_t base[3];           /* the cores; a zero ends the list */
    uint32_t ghwcfg3[3];
} CfgCase;

static const CfgCase cases[] = {
    { "ingenic-t10", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t20", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t21", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t23", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t30", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t31", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t32", { 0x13500000 }, { GHWCFG3_T32 } },
    { "ingenic-t40", { 0x13500000 }, { GHWCFG3_T31 } },
    { "ingenic-t41", { 0x13500000 }, { GHWCFG3_T32 } },
    { "ingenic-a1", { 0x13600000, 0x13640000, 0x13680000 },
      { GHWCFG3_A1, GHWCFG3_T32, GHWCFG3_A1 } },
};

static void test_cfg(const void *data)
{
    const CfgCase *c = data;
    QTestState *qts = qtest_initf("-machine %s", c->machine);

    for (int k = 0; k < 3 && c->base[k]; k++) {
        uint32_t base = c->base[k];

        g_assert_cmphex(qtest_readl(qts, base + GSNPSID), ==, 0x4f54300a);
        g_assert_cmphex(qtest_readl(qts, base + GSNPSID + 4), ==, 0);
        g_assert_cmphex(qtest_readl(qts, base + GSNPSID + 8), ==, 0x228be050);
        g_assert_cmphex(qtest_readl(qts, base + GSNPSID + 12), ==,
                        c->ghwcfg3[k]);
        g_assert_cmphex(qtest_readl(qts, base + GSNPSID + 16), ==, 0xe3f08020);

        /* NumDevEps 8 and NumInEps 8: endpoints 0 to 8 in each direction */
        for (int ep = 0; ep < 16; ep++) {
            uint32_t want = ep <= 8 ? 0x1000 + ep * 4 : 0;

            qtest_writel(qts, base + DIEPDMA(ep), 0x1000 + ep * 4);
            qtest_writel(qts, base + DOEPDMA(ep), 0x1000 + ep * 4);
            g_assert_cmphex(qtest_readl(qts, base + DIEPDMA(ep)), ==, want);
            g_assert_cmphex(qtest_readl(qts, base + DOEPDMA(ep)), ==, want);
        }
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-dwc2cfg/%s",
                                                cases[i].machine + 8);

        qtest_add_data_func(path, &cases[i], test_cfg);
    }
    return g_test_run();
}
