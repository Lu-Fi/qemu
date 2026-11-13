/*
 * QTest testcase for the Ingenic PDMA's memory addresses
 *
 * A no-descriptor auto copy of eight words, as measured on a T30X and a
 * T41NQ at U-Boot: a physical address copies; a KSEG0, KSEG1 or KSEG2
 * address, or one past the RAM, is not translated, so a source there reads
 * junk (zeros here), a target there drops the write, and DCS and DMAC read
 * as after a good run, with no AR. DSA and DTA advance by the bytes moved.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define PDMA            0x13420000
#define DMAC            (PDMA + 0x1000)
#define CH(n, r)        (PDMA + (n) * 0x20 + (r))
#define DSA             0x00
#define DTA             0x04
#define DTC             0x08
#define DRT             0x0c
#define DCS             0x10
#define DCM             0x14

#define DCS_GO          0x80000001
#define DCS_TT          0x00000008
#define DCS_AR          0x00000010
#define DMAC_AR         0x00000004
#define DCM_SAI_DAI     0x00c00000
#define DRT_AUTO        8

#define SRC             0x00600000
#define DST             0x00700000
#define WORDS           8

typedef struct {
    const char *name;
    uint32_t src, dst;
    bool lands;         /* the data reaches DST */
    bool src_junk;      /* the source reads as zeros */
} Case;

static const Case cases[] = {
    { "phys", SRC, DST, true, false },
    { "kseg1-kseg1", SRC | 0xa0000000, DST | 0xa0000000, false, true },
    { "kseg0-kseg0", SRC | 0x80000000, DST | 0x80000000, false, true },
    { "kseg1-src", SRC | 0xa0000000, DST, false, true },
    { "kseg1-dst", SRC, DST | 0xa0000000, false, false },
    { "kseg0-src", SRC | 0x80000000, DST, false, true },
    { "kseg0-dst", SRC, DST | 0x80000000, false, false },
    { "kseg2-src", SRC | 0xc0000000, DST, false, true },
    { "past-ram-src", 0x0c000000, DST, false, true },
    { "past-ram-dst", SRC, 0x0c000000, false, false },
};

static void fill(QTestState *qts)
{
    for (int i = 0; i < WORDS; i++) {
        qtest_writel(qts, SRC + 4 * i, 0x11110100 + i);
        qtest_writel(qts, DST + 4 * i, 0x5a5a5a00 + i);
    }
}

static void test_copy(gconstpointer data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s -m 128", machine);

    for (int n = 0; n < ARRAY_SIZE(cases); n++) {
        const Case *c = &cases[n];

        fill(qts);
        qtest_writel(qts, DMAC, 1);
        qtest_writel(qts, CH(n, DCS), 0);
        qtest_writel(qts, CH(n, DSA), c->src);
        qtest_writel(qts, CH(n, DTA), c->dst);
        qtest_writel(qts, CH(n, DTC), WORDS);
        qtest_writel(qts, CH(n, DRT), DRT_AUTO);
        qtest_writel(qts, CH(n, DCM), DCM_SAI_DAI);
        qtest_writel(qts, CH(n, DCS), DCS_GO);

        g_assert_cmphex(qtest_readl(qts, CH(n, DCS)) & (DCS_TT | DCS_AR), ==,
                        DCS_TT);
        g_assert_cmphex(qtest_readl(qts, DMAC) & DMAC_AR, ==, 0);
        g_assert_cmphex(qtest_readl(qts, CH(n, DTC)), ==, 0);
        g_assert_cmphex(qtest_readl(qts, CH(n, DSA)), ==, c->src + 4 * WORDS);
        g_assert_cmphex(qtest_readl(qts, CH(n, DTA)), ==, c->dst + 4 * WORDS);
        for (int i = 0; i < WORDS; i++) {
            uint32_t want = 0x5a5a5a00 + i;

            if (c->lands) {
                want = 0x11110100 + i;
            } else if (c->src_junk && c->dst == DST) {
                want = 0;
            }
            g_assert_cmphex(qtest_readl(qts, DST + 4 * i), ==, want);
            g_assert_cmphex(qtest_readl(qts, SRC + 4 * i), ==, 0x11110100 + i);
        }
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("ingenic-pdma/t30", "ingenic-t30", test_copy);
    qtest_add_data_func("ingenic-pdma/t31", "ingenic-t31", test_copy);
    qtest_add_data_func("ingenic-pdma/t41", "ingenic-t41", test_copy);
    return g_test_run();
}
