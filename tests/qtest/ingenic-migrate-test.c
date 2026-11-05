/*
 * QTest testcase for saving and loading an Ingenic machine
 *
 * Each machine saves its state to a file and a second QEMU of the same
 * machine loads it: what the guest left in the interrupt controller, the
 * GPIO masks, the OS timer's enables and compare must read back the same
 * there. qtest's virtual clock starts at 0 in each QEMU, so the counts are
 * left to the boots of the regression's snap suite.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define GPIO_PXMSK      0x10010020
#define GPIO_PXMSKC     0x10010028
#define INTC_IMR        0x04
#define INTC_IMCR       0x0c

typedef struct MigrateCase {
    const char *machine;
    uint32_t intc;
    uint32_t ost_dfr;           /* the OS timer's compare */
    uint32_t count;             /* a free-running count, its low word */
    uint32_t count_en;          /* TESR, which starts it, or 0: it runs */
} MigrateCase;

static const MigrateCase cases[] = {
    { "ingenic-t10", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t20", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t21", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t23", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t30", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t31", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t32", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t33", 0x10001000, 0x12000014, 0x12000020, 0x12000034 },
    { "ingenic-t40", 0x10001000, 0x12100014, 0x12000010, 0 },
    { "ingenic-t41", 0x10001000, 0x12100014, 0x12000010, 0 },
    { "ingenic-a1", 0x12300000, 0x12100014, 0x12000010, 0 },
};

static void wait_status(QTestState *qts, const char *key, const char *busy)
{
    for (int i = 0; i < 6000; i++) {
        QDict *rsp = qtest_qmp(qts, "{ 'execute': %s }", key);
        QDict *ret = qdict_get_qdict(rsp, "return");
        const char *st = qdict_get_try_str(ret, "status");
        bool done = st && strcmp(st, busy) && strcmp(st, "setup") &&
                    strcmp(st, "device");

        if (done) {
            g_assert_cmpstr(st, !=, "failed");
            qobject_unref(rsp);
            return;
        }
        qobject_unref(rsp);
        g_usleep(10000);
    }
    g_assert_not_reached();
}

static void test_migrate(const void *data)
{
    const MigrateCase *c = data;
    g_autofree char *path = g_strdup_printf("%s/ingenic-migrate-%d.state",
                                            g_get_tmp_dir(), getpid());
    g_autofree char *uri = g_strdup_printf("file:%s", path);
    QTestState *src, *dst;
    uint32_t count, dfr;

    src = qtest_initf("-machine %s", c->machine);
    if (c->count_en) {
        qtest_writel(src, c->count_en, 1u << 1);      /* T2 */
    }
    qtest_writel(src, c->intc + INTC_IMCR, 0x00010001);
    qtest_writel(src, GPIO_PXMSKC, 0x0000f00f);
    qtest_writel(src, c->ost_dfr, 0x123456);
    dfr = qtest_readl(src, c->ost_dfr);
    qtest_clock_step(src, 50 * 1000 * 1000);
    count = qtest_readl(src, c->count);
    g_assert_cmpuint(count, !=, 0);

    qtest_qmp_assert_success(src, "{ 'execute': 'stop' }");
    qtest_qmp_assert_success(src, "{ 'execute': 'migrate', 'arguments': "
                             "{ 'uri': %s } }", uri);
    wait_status(src, "query-migrate", "active");

    dst = qtest_initf("-machine %s -incoming %s", c->machine, uri);
    wait_status(dst, "query-status", "inmigrate");
    g_assert_cmphex(qtest_readl(dst, c->intc + INTC_IMR), ==,
                    qtest_readl(src, c->intc + INTC_IMR));
    g_assert_cmphex(qtest_readl(dst, c->intc + INTC_IMR) & 0x00010001, ==, 0);
    g_assert_cmphex(qtest_readl(dst, GPIO_PXMSK), ==,
                    qtest_readl(src, GPIO_PXMSK));
    g_assert_cmphex(qtest_readl(dst, c->ost_dfr), ==, dfr);
    if (c->count_en) {
        g_assert_cmphex(qtest_readl(dst, 0x12000004), ==, 1u << 1); /* TER */
    }

    qtest_quit(dst);
    qtest_quit(src);
    unlink(path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *name = g_strdup_printf("ingenic/migrate/%s",
                                                cases[i].machine + 8);

        qtest_add_data_func(name, &cases[i], test_migrate);
    }
    return g_test_run();
}
