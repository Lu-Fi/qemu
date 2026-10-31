/*
 * QTest testcase for the Ingenic TCU's channels and flag mask
 *
 * TMR out of reset as each part reads it, and the channels each part has: a
 * T31X takes writes to channel 3's registers and reads 0 from channel 4's
 * and 7's after writing them, and TMSR and TMCR move only the bits it has.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "libqtest.h"

#define TCU             0x10002000
#define TCU_TER         0x10
#define TCU_TESR        0x14
#define TCU_TECR        0x18
#define TCU_TMR         0x30
#define TCU_TMSR        0x34
#define TCU_TMCR        0x38
#define TCU_TDFR(n)     (0x40 + (n) * 0x10)
#define TCU_TDHR(n)     (0x44 + (n) * 0x10)

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    uint32_t tmr;
} TcuCase;

static const TcuCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", 0x000f800f },
    { "t20x", "ingenic-t20", "ingenic-t31", 0x000f800f },
    { "t21n", "ingenic-t21", "ingenic-t31", 0x00ff80ff },
    { "t23n", "ingenic-t23", "ingenic-t31", 0x000f800f },
    { "t30x", "ingenic-t30", "ingenic-t31", 0x00ff80ff },
    { "t31x", "ingenic-t31", "ingenic-t31", 0x000f800f },
    { "t32lq", "ingenic-t32", "ingenic-t31", 0x01ff80ff },
    { "t40n", "ingenic-t40", "ingenic-t40", 0x00ff80ff },
    { "t41lq", "ingenic-t41", "ingenic-t40", 0x01ff80ff },
    { "a1n", "ingenic-a1", "ingenic-a1", 0x01ff80ff },
};

static void test_case(const void *data)
{
    const TcuCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);
    unsigned channels = ctz32(~c->tmr);

    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, c->tmr);
    qtest_writel(qts, TCU + TCU_TMCR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, 0);
    qtest_writel(qts, TCU + TCU_TMSR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, c->tmr);

    for (unsigned n = 0; n < 8; n++) {
        uint32_t want = n < channels ? 0x1234 + n : 0;

        qtest_writel(qts, TCU + TCU_TDFR(n), 0x1234 + n);
        qtest_writel(qts, TCU + TCU_TDHR(n), 0x1234 + n);
        g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDFR(n)), ==, want);
        g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDHR(n)), ==, want);
    }
    qtest_writel(qts, TCU + TCU_TESR, 0xff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER), ==, (1u << channels) - 1);
    qtest_writel(qts, TCU + TCU_TECR, 0xff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-tcu/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
