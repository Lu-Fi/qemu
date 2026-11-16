/*
 * QTest testcase for the Ingenic parts' registers against a census
 *
 * Each part's peripheral registers were read on the part itself (U-Boot's
 * md.l, nothing but reads of registers whose reads are free of side
 * effects), and again on the model at reset. The table keeps the registers
 * where the two agreed, so a model change that moves one of them away from
 * what the silicon read fails here. Registers the firmware had programmed
 * by then and the model leaves at reset are not in the table, and neither
 * are the CPM (its own test), the RTC (a clock), the eFuse (unique to each
 * chip) and the PHY blocks. A block the model only stubs as zeroes appears
 * when the silicon also read zero.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

typedef struct {
    uint32_t addr, value;
} CensusReg;

typedef struct {
    const char *name, *machine, *soc, *variant;
    const CensusReg *regs;
    size_t n;
    uint32_t clkgr0_open;   /* CLKGR0 gates the census opened for its reads */
    uint32_t clkgr1_open;   /* CLKGR1 gates the census opened for its reads */
} CensusPart;

#include "ingenic-census-regs.inc"

static void test_part(const void *data)
{
    const CensusPart *p = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  p->machine, p->soc, p->variant);

    if (p->clkgr0_open) {
        qtest_writel(qts, 0x10000020,
                     qtest_readl(qts, 0x10000020) & ~p->clkgr0_open);
    }
    if (p->clkgr1_open) {
        qtest_writel(qts, 0x10000028,
                     qtest_readl(qts, 0x10000028) & ~p->clkgr1_open);
    }
    for (size_t i = 0; i < p->n; i++) {
        g_test_message("%s 0x%08x", p->name, p->regs[i].addr);
        g_assert_cmphex(qtest_readl(qts, p->regs[i].addr), ==,
                        p->regs[i].value);
    }
    qtest_quit(qts);
}

/*
 * A T23N's IVDC window with CLKGR1 bit 3 set, as U-Boot leaves it: every
 * word reads 1 (read at 0x13200000 to 0x13200018 and 0x13201208)
 */
static void test_t23_ivdc_gated(const void *data)
{
    static const uint32_t addrs[] = {
        0x13200000, 0x13200004, 0x13200008, 0x13200010, 0x13200014,
        0x13200018, 0x13201208,
    };
    const CensusPart *p = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  p->machine, p->soc, p->variant);

    g_assert_cmphex(qtest_readl(qts, 0x10000028) & 8, ==, 8);
    for (size_t i = 0; i < ARRAY_SIZE(addrs); i++) {
        g_assert_cmphex(qtest_readl(qts, addrs[i]), ==, 1);
    }
    qtest_writel(qts, 0x10000028, qtest_readl(qts, 0x10000028) & ~8);
    g_assert_cmphex(qtest_readl(qts, 0x13200014), ==, 0);
    qtest_quit(qts);
}

/*
 * The DPU's registers that are not zero at U-Boot, and the bits each keeps:
 * read, written all ones and read, written zero and read, on a T40XP and a
 * T41LQ with CLKGR0 bit 24 open (0x13050000; the registers on the T41 are
 * the T21's smaller controller)
 */
typedef struct {
    uint32_t off, reset, ones;
} DpuKept;

static const DpuKept t40xp_dpu[] = {
    { 0x2014, 0x00cf000c, 0x00ff00fe }, { 0x2018, 0x00000006, 0x00000007 },
    { 0x201c, 0x00000006, 0x00000007 }, { 0x2020, 0x03190842, 0x07ffffff },
    { 0x2024, 0x00c04010, 0x07ffffff }, { 0x2028, 0x0000000b, 0x000007ff },
    { 0x202c, 0x00600400, 0x01ffffff }, { 0x2030, 0x01000400, 0x01ffffff },
    { 0x2034, 0x01e00400, 0x01ffffff }, { 0x3200, 0x066204a8, 0x07ff07ff },
    { 0x3204, 0x03410190, 0x03ff01ff }, { 0x3208, 0x00000812, 0x00000fff },
    { 0x320c, 0x00800000, 0x00ff001f }, { 0x3210, 0x066204a8, 0x07ff07ff },
    { 0x3214, 0x03410190, 0x03ff01ff }, { 0x3218, 0x00000812, 0x00000fff },
    { 0x321c, 0x00800000, 0x00ff001f }, { 0x8000, 0x000000e0, 0x003f00f3 },
    { 0xa000, 0x01540d20, 0x0fff0fff },
};

static const DpuKept t41lq_dpu[] = {
    { 0x0004, 0x00000012, 0x003f003f }, { 0x0100, 0x066204a8, 0x07ff07ff },
    { 0x0104, 0x03410190, 0x03ff01ff }, { 0x0108, 0x00000812, 0x00000fff },
    { 0x010c, 0x00800000, 0x00ff001f }, { 0x1010, 0x03190842, 0x07ffffff },
    { 0x8000, 0x00000001, 0x003f0007 }, { 0xa000, 0x01540d20, 0x8fff0f7f },
};

static void test_dpu_kept(const void *data)
{
    const CensusPart *p = data;
    const DpuKept *k = !strcmp(p->name, "t40xp") ? t40xp_dpu : t41lq_dpu;
    size_t n = !strcmp(p->name, "t40xp") ? ARRAY_SIZE(t40xp_dpu) :
                                           ARRAY_SIZE(t41lq_dpu);
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  p->machine, p->soc, p->variant);

    qtest_writel(qts, 0x10000020, qtest_readl(qts, 0x10000020) & ~(1u << 24));
    for (size_t i = 0; i < n; i++) {
        uint32_t a = 0x13050000 + k[i].off;

        g_test_message("%s 0x%08x", p->name, a);
        g_assert_cmphex(qtest_readl(qts, a), ==, k[i].reset);
        qtest_writel(qts, a, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, a), ==, k[i].ones);
        qtest_writel(qts, a, 0);
        g_assert_cmphex(qtest_readl(qts, a), ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (size_t i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/census/%s",
                                                parts[i].name);

        qtest_add_data_func(path, &parts[i], test_part);
        if (!strcmp(parts[i].name, "t23n")) {
            g_autofree char *gpath = g_strdup_printf(
                "ingenic/census/%s/ivdc-gated", parts[i].name);

            qtest_add_data_func(gpath, &parts[i], test_t23_ivdc_gated);
        }
        if (!strcmp(parts[i].name, "t40xp") ||
            !strcmp(parts[i].name, "t41lq")) {
            g_autofree char *dpath = g_strdup_printf(
                "ingenic/census/%s/dpu-kept", parts[i].name);

            qtest_add_data_func(dpath, &parts[i], test_dpu_kept);
        }
    }
    return g_test_run();
}
