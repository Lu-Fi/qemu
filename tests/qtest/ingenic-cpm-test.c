/*
 * QTest testcase for the Ingenic CPM's reset values
 *
 * One case per part: the registers U-Boot leaves alone, as the part named
 * reads them after U-Boot. OPCR reads 0x80 or 0xe0 more there, from the
 * SPL; 0x1500 is its reset in the T21 manual. A running PLL reads on and
 * locked, and CPCSR reads its stable flags. The memory power controls of the
 * T21 and T30 are at 0xf0 to 0xf8.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM             0x10000000
#define CPAPCR          0x10
#define A1_CPCSR        0xec
#define PLL_ON_LOCK_EN  0x0000000d

typedef struct {
    uint32_t offset, value;
} Reg;

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    bool t_series;              /* also reads the T-series registers below */
    Reg regs[8];                /* a zero value ends the list */
} CpmCase;

/* RSR, CPPCR, OPCR, CPSPPR and CPCSR on every part from the T10 to the T41 */
static const Reg t_series[] = {
    { 0x08, 0x00000001 }, { 0x0c, 0x00000020 }, { 0x24, 0x00001500 },
    { 0x38, 0x0000a5a5 }, { 0xd4, 0xf8000000 },
};

static const CpmCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", true,
      { { 0x60, 0x00002002 } } },
    { "t20x", "ingenic-t20", "ingenic-t31", true,
      { { 0x60, 0x00002002 } } },
    { "t21n", "ingenic-t21", "ingenic-t31", true,
      { { 0x60, 0x00100002 }, { 0x70, 0x00000001 },
        { 0xf0, 0x01041041 }, { 0xf4, 0x01041041 }, { 0xf8, 0xffffffff } } },
    { "t23n", "ingenic-t23", "ingenic-t31", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 } } },
    { "t30x", "ingenic-t30", "ingenic-t31", true,
      { { 0x60, 0x00100002 },
        { 0xf0, 0x01041041 }, { 0xf4, 0x01041041 }, { 0xf8, 0xffffffff } } },
    { "t31x", "ingenic-t31", "ingenic-t31", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 } } },
    { "t32lq", "ingenic-t32", "ingenic-t31", true,
      { { 0x70, 0x00100002 }, { 0x78, 0x00000001 },
        { 0x84, 0x00100002 }, { 0x88, 0x00000001 } } },
    { "t40n", "ingenic-t40", "ingenic-t40", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 } } },
    { "t40xp", "ingenic-t40", "ingenic-t40", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 } } },
    { "t41lq", "ingenic-t41", "ingenic-t40", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 } } },
    { "a1n", "ingenic-a1", "ingenic-a1", false,
      { { 0x08, 0x00000001 }, { 0x0c, 0x00000020 }, { 0x34, 0x00001500 },
        { 0x48, 0x0000a5a5 }, { 0xa0, 0x00100002 }, { 0xa8, 0x00100002 },
        { 0x100, 0x00100002 }, { 0x108, 0x00100002 } } },
};

static void check(QTestState *qts, const char *part, const Reg *r, int n)
{
    for (int i = 0; i < n && r[i].value; i++) {
        g_test_message("%s 0x%02x", part, r[i].offset);
        g_assert_cmphex(qtest_readl(qts, CPM + r[i].offset), ==, r[i].value);
    }
}

static void test_case(const void *data)
{
    const CpmCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    if (c->t_series) {
        check(qts, c->part, t_series, ARRAY_SIZE(t_series));
    } else {
        g_assert_cmphex(qtest_readl(qts, CPM + A1_CPCSR), ==, 0x7c000000);
    }
    check(qts, c->part, c->regs, ARRAY_SIZE(c->regs));
    g_assert_cmphex(qtest_readl(qts, CPM + CPAPCR) & PLL_ON_LOCK_EN, ==,
                    PLL_ON_LOCK_EN);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-cpm/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
