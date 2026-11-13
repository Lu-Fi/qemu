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
} CensusPart;

#include "ingenic-census-regs.inc"

static void test_part(const void *data)
{
    const CensusPart *p = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  p->machine, p->soc, p->variant);

    for (size_t i = 0; i < p->n; i++) {
        g_test_message("%s 0x%08x", p->name, p->regs[i].addr);
        g_assert_cmphex(qtest_readl(qts, p->regs[i].addr), ==,
                        p->regs[i].value);
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
    }
    return g_test_run();
}
