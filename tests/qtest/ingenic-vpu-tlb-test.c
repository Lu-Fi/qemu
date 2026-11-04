/*
 * QTest testcase for the VPUs' TLB registers: HELIX (T20, T23, T30) and
 * RADIX (T30)
 *
 * The bits each register keeps and which survive the CPM's soft reset, as
 * a T20X, T23N and T30X read them. On those parts setting the TLB
 * translates no address, so its registers are all there is to it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM_SRBC        0x100000c4
#define RADIX           0x13600000

typedef struct {
    uint32_t reg, ones;
} Mask;

typedef struct {
    const char *machine;
    uint32_t helix, glbc;       /* GLBC's bits */
} Part;

static const Part t30 = { "ingenic-t30", 0x13200000, 0xaffffff7 };
static const Part t20 = { "ingenic-t20", 0x13200000, 0xaffffff7 };
static const Part t23 = { "ingenic-t23", 0x13100000, 0xffffffff };

static const Mask helix_masks[] = {
    { 0x30, 0xfffff000 },       /* TLBA */
    { 0x50, 0xfffffff0 },       /* TLBC */
    { 0x54, 0xcfff0fff },       /* TLBV */
};

static const Mask radix_masks[] = {
    { 0x10, 0xfffffffc },       /* TLB_TBA */
    { 0x14, 0xfffffff0 },       /* TLB_CTRL */
    { 0x18, 0x0fff0fff },       /* TLBV */
    { 0x1c, 0x00000000 },       /* TLBE */
};

static void check_masks(QTestState *qts, uint32_t base, const Mask *m, int n)
{
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, base + m[i].reg, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, base + m[i].reg), ==, m[i].ones);
        qtest_writel(qts, base + m[i].reg, 0);
        g_assert_cmphex(qtest_readl(qts, base + m[i].reg), ==, 0);
    }
}

static void soft_reset(QTestState *qts, int bit)
{
    qtest_writel(qts, CPM_SRBC, 1u << bit);
    qtest_writel(qts, CPM_SRBC, 0);
}

/* HELIX: the soft reset keeps TLBA and TLBC, clears GLBC and TLBV */
static void test_helix(const void *data)
{
    const Part *p = data;
    QTestState *qts = qtest_initf("-machine %s", p->machine);
    uint32_t helix = p->helix;
    Mask glbc = { 0x00, p->glbc };

    check_masks(qts, helix, &glbc, 1);
    check_masks(qts, helix, helix_masks, ARRAY_SIZE(helix_masks));
    qtest_writel(qts, helix + 0x00, 0x00e00200);
    qtest_writel(qts, helix + 0x30, 0x12345000);
    qtest_writel(qts, helix + 0x50, 0x55500003);
    qtest_writel(qts, helix + 0x54, 0x0aaa0555);
    g_assert_cmphex(qtest_readl(qts, helix + 0x50), ==, 0x55500000);
    soft_reset(qts, 31);
    g_assert_cmphex(qtest_readl(qts, helix + 0x00), ==, 0);
    g_assert_cmphex(qtest_readl(qts, helix + 0x30), ==, 0x12345000);
    g_assert_cmphex(qtest_readl(qts, helix + 0x50), ==, 0x55500000);
    g_assert_cmphex(qtest_readl(qts, helix + 0x54), ==, 0);
    qtest_quit(qts);
}

/* RADIX: the soft reset keeps TBA, CTRL and TLBV, clears GLB_CTRL */
static void test_radix(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    check_masks(qts, RADIX, radix_masks, ARRAY_SIZE(radix_masks));
    qtest_writel(qts, RADIX + 0x00, 0x0003ffff);
    qtest_writel(qts, RADIX + 0x10, 0x12345000);
    qtest_writel(qts, RADIX + 0x14, 0x5550000f);
    qtest_writel(qts, RADIX + 0x18, 0x0aaa0555);
    qtest_writel(qts, RADIX + 0x1c, 0x3f);
    g_assert_cmphex(qtest_readl(qts, RADIX + 0x00), ==, 0x0003ffff);
    soft_reset(qts, 28);
    g_assert_cmphex(qtest_readl(qts, RADIX + 0x00), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RADIX + 0x10), ==, 0x12345000);
    g_assert_cmphex(qtest_readl(qts, RADIX + 0x14), ==, 0x55500000);
    g_assert_cmphex(qtest_readl(qts, RADIX + 0x18), ==, 0x0aaa0555);
    g_assert_cmphex(qtest_readl(qts, RADIX + 0x1c), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("ingenic/vpu-tlb/t30-helix", &t30, test_helix);
    qtest_add_data_func("ingenic/vpu-tlb/t20-helix", &t20, test_helix);
    qtest_add_data_func("ingenic/vpu-tlb/t23-helix", &t23, test_helix);
    qtest_add_func("ingenic/vpu-tlb/t30-radix", test_radix);
    return g_test_run();
}
