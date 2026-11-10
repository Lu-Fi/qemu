/*
 * QTest testcase for the Ingenic DTRNG
 *
 * The protocol of the jz-dtrng driver, on all eleven machines: the
 * register as every part resets it, status ready only while generation is
 * on, the CPU mode (divider, interrupt masked, enable, poll status bit 0,
 * read RANDOMNUM), RDY_CLR as a pulse that does not latch, and the
 * driver's IRQ mode, which waits for an interrupt on INTC source 34. The
 * numbers are random: the test reads a run of them and requires only that
 * they are not all one value.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "libqtest.h"

#define DTRNG_BASE      0x10072000
#define INTC_ISR1       0x10001020      /* sources 32 to 63 */
#define IRQ_DTRNG       34

#define CFG             0x00
#define RANDOMNUM       0x04
#define STAT            0x08

#define CFG_EN          BIT(0)
#define CFG_DIV(n)      ((n) << 1)
#define CFG_MASK        BIT(11)
#define CFG_RDY_CLR     BIT(12)
#define CFG_RESET       0x03ff0800

static const struct {
    const char *part;
    const char *machine;
} parts[] = {
    { "t10", "ingenic-t10" },
    { "t20", "ingenic-t20" },
    { "t21", "ingenic-t21" },
    { "t23", "ingenic-t23" },
    { "t30", "ingenic-t30" },
    { "t31", "ingenic-t31" },
    { "t32", "ingenic-t32" },
    { "t33", "ingenic-t33" },
    { "t40", "ingenic-t40" },
    { "t41", "ingenic-t41" },
    { "a1", "ingenic-a1" },
};

static uint32_t rd(QTestState *qts, unsigned reg)
{
    return qtest_readl(qts, DTRNG_BASE + reg);
}

static void wr(QTestState *qts, unsigned reg, uint32_t v)
{
    qtest_writel(qts, DTRNG_BASE + reg, v);
}

static void cfg_or(QTestState *qts, uint32_t bits)
{
    wr(qts, CFG, rd(qts, CFG) | bits);
}

static void cfg_clear(QTestState *qts, uint32_t bits)
{
    wr(qts, CFG, rd(qts, CFG) & ~bits);
}

/* dtrng_cpu_get_random: divider 8, interrupt masked, enable, poll, read */
static void test_cpu_mode(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);
    uint32_t first, v;
    bool varied = false;

    g_assert_cmphex(rd(qts, CFG), ==, CFG_RESET);
    g_assert_cmphex(rd(qts, STAT) & 1, ==, 0);

    cfg_or(qts, CFG_DIV(8) | CFG_MASK | CFG_EN);
    g_assert_cmphex(rd(qts, CFG) & (CFG_EN | CFG_MASK), ==, CFG_EN | CFG_MASK);
    g_assert_cmphex(rd(qts, STAT) & 1, ==, 1);
    first = rd(qts, RANDOMNUM);
    for (int i = 0; i < 32; i++) {
        g_assert_cmphex(rd(qts, STAT) & 1, ==, 1);
        v = rd(qts, RANDOMNUM);
        varied |= v != first;
    }
    g_assert_true(varied);

    /* The driver's tail: pulse RDY_CLR, then clear it and disable */
    cfg_or(qts, CFG_RDY_CLR);
    g_assert_cmphex(rd(qts, CFG) & CFG_RDY_CLR, ==, 0);
    cfg_clear(qts, CFG_RDY_CLR);
    cfg_clear(qts, CFG_EN);
    g_assert_cmphex(rd(qts, CFG) & CFG_EN, ==, 0);
    g_assert_cmphex(rd(qts, STAT) & 1, ==, 0);
    qtest_quit(qts);
}

/*
 * dtrng_irqmode_random: unmask, divider 8, enable, then wait for the
 * interrupt (INTC source 34), whose handler reads the number, pulses
 * RDY_CLR, disables and masks, which drops the line.
 */
static void test_irq_mode(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);

    cfg_clear(qts, CFG_MASK);
    cfg_or(qts, CFG_DIV(8) | CFG_EN);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) & BIT(IRQ_DTRNG - 32), ==,
                    BIT(IRQ_DTRNG - 32));
    cfg_or(qts, CFG_RDY_CLR);
    cfg_clear(qts, CFG_EN);
    cfg_or(qts, CFG_MASK);
    cfg_clear(qts, CFG_RDY_CLR);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) & BIT(IRQ_DTRNG - 32), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-dtrng/%s/cpu-mode",
                                                parts[i].part);

        qtest_add_data_func(path, parts[i].machine, test_cpu_mode);
        g_free(path);
        path = g_strdup_printf("/ingenic-dtrng/%s/irq-mode", parts[i].part);
        qtest_add_data_func(path, parts[i].machine, test_irq_mode);
    }
    return g_test_run();
}
