/*
 * QTest testcase for what the Ingenic T32's HERA reads back of a picture
 *
 * libimp's rate control takes registers of the core after each H.265
 * picture: they follow the picture, a flat one reading 0 as a flat scene
 * does, one of rows alternately 180 and 84 reading as busy. The test codes
 * a flat picture, then the stripes, then the stripes with the rows swapped
 * as a P picture, and checks the registers that follow the gradients of the
 * picture and its change from the last.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define HERA            0x13200000
#define HW              64
#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define LIST            0x00860000
#define BS              0x00880000
#define HL_VLD          (1u << 31)
#define HL_TERM         (1u << 30)

#define COMPLEXITY      0x20124
#define SAD_SUM         0x20118

static void job(QTestState *qts, bool intra, int row_phase)
{
    uint32_t pairs[32], n = 0;
    uint8_t y[HW * HW], c[HW * HW / 2];
    const struct {
        uint32_t reg, val;
    } regs[] = {
        { 0x2000c, SRC_Y }, { 0x20010, SRC_C }, { 0x20014, HW | HW << 16 },
        { 0x68004, HW << 16 | HW }, { 0x68000, (intra ? 2 : 1) << 2 },
        { 0x60004, 20 << 1 }, { 0x20004, 1 },
    };

    for (int i = 0; i < HW * HW; i++) {
        y[i] = row_phase < 0 ? 128 : ((i / HW) + row_phase) & 1 ? 84 : 180;
    }
    memset(c, 128, sizeof(c));
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        pairs[n++] = cpu_to_le32(regs[i].val);
        pairs[n++] = cpu_to_le32(HL_VLD | (i == ARRAY_SIZE(regs) - 1 ?
                                           HL_TERM : 0) | regs[i].reg);
    }
    qtest_memwrite(qts, LIST, pairs, n * 4);
    qtest_writel(qts, HERA + 0x2c, 0x7ff);
    qtest_writel(qts, HERA + 0x90008, BS);
    qtest_writel(qts, HERA + 0x90068, LIST);
    qtest_writel(qts, HERA + 0x8, 1);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, HERA + 0x2c) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, HERA + 0x2c) & 1, ==, 1);
    qtest_writel(qts, HERA + 0x30, 0x7ff);
}

static void test_stats(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    g_assert_cmpuint(qtest_readl(qts, HERA + COMPLEXITY), ==, 0);
    job(qts, true, -1);
    g_assert_cmpuint(qtest_readl(qts, HERA + COMPLEXITY), ==, 0);

    /* half the gradients of an I picture: 15 rows of 96 in 16 columns */
    job(qts, true, 0);
    g_assert_cmpuint(qtest_readl(qts, HERA + COMPLEXITY), >, 0);
    g_assert_cmpuint(qtest_readl(qts, HERA + 0x203a4), ==, 0);
    /* an intra picture has nothing to differ from */
    g_assert_cmpuint(qtest_readl(qts, HERA + SAD_SUM), ==, 0);

    /* a P picture of the stripes swapped: every sample 96 off */
    job(qts, false, 1);
    g_assert_cmpuint(qtest_readl(qts, HERA + SAD_SUM), >, 0);
    /* the reference is the picture before it, coded, so only about 96 */
    g_assert_cmpuint(qtest_readl(qts, HERA + COMPLEXITY), >, HW * HW * 80);
    g_assert_cmpuint(qtest_readl(qts, HERA + COMPLEXITY), <, HW * HW * 102);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/hera-stats/h265", test_stats);
    return g_test_run();
}
