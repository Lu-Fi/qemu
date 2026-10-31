/*
 * QTest testcase for the Ingenic AIC's frame clocks
 *
 * Where the CPM's I2S clocks drive the AIC (T23, T31, T40, T41), play and
 * record each run at their own clock, as a T31X and a T41NQ do: with the
 * record divider at twice the play divider's M, the record FIFO reaches
 * its threshold in half the time the play FIFO takes to drain to its own.
 * Where the codec is the master (T20), both run at its rate.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM_I2SCDR      0x10000070
#define CPM_I2SCDR1     0x10000084
#define I2S_CE          (1u << 29)

#define AIC             0x10020000
#define AICFR           0x00
#define AICCR           0x04
#define AICSR           0x14
#define AICDR           0x34

#define AICFR_ENB       (1u << 0)
#define AICCR_EREC      (1u << 0)
#define AICCR_ERPL      (1u << 1)
#define AICSR_TFS       (1u << 3)
#define AICSR_RFS       (1u << 4)

/* 16 one-entry frames each way: TX 64 down to 48, RX up to 16 */
#define AICFR_SETUP     (AICFR_ENB | 24 << 16 | 7 << 24)
#define AICCR_SETUP     (AICCR_EREC | AICCR_ERPL | 1 << 12)

typedef struct {
    const char *machine;
    uint32_t tx_m, rx_m;        /* CPM divider M; 0: the codec drives it */
    unsigned ratio;             /* the play threshold's time over record's */
} RateCase;

static const RateCase cases[] = {
    { "ingenic-t31", 16, 32, 2 },
    { "ingenic-t31", 32, 16, 0 },       /* record slower: half */
    { "ingenic-t41", 16, 32, 2 },
    { "ingenic-t20", 0, 0, 1 },
};

static void test_rates(const void *data)
{
    const RateCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -audio none", c->machine);
    int64_t t_tfs = -1, t_rfs = -1, t;

    if (c->tx_m) {
        qtest_writel(qts, CPM_I2SCDR, I2S_CE | c->tx_m << 20 | 10875);
        qtest_writel(qts, CPM_I2SCDR1, I2S_CE | c->rx_m << 20 | 10875);
    }
    qtest_writel(qts, AIC + AICFR, AICFR_SETUP);
    for (int i = 0; i < 64; i++) {
        qtest_writel(qts, AIC + AICDR, 0);
    }
    t = qtest_clock_step(qts, 1);
    qtest_writel(qts, AIC + AICCR, AICCR_SETUP);
    for (int i = 0; i < 100000 && (t_tfs < 0 || t_rfs < 0); i++) {
        int64_t now = qtest_clock_step_next(qts);
        uint32_t sr = qtest_readl(qts, AIC + AICSR);

        if (t_tfs < 0 && (sr & AICSR_TFS)) {
            t_tfs = now - t;
        }
        if (t_rfs < 0 && (sr & AICSR_RFS)) {
            t_rfs = now - t;
        }
    }
    g_assert_cmpint(t_tfs, >, 0);
    g_assert_cmpint(t_rfs, >, 0);
    if (c->ratio) {
        g_assert_cmpfloat((double)t_tfs / t_rfs, >, c->ratio * 0.95);
        g_assert_cmpfloat((double)t_tfs / t_rfs, <, c->ratio * 1.05);
    } else {
        g_assert_cmpfloat((double)t_rfs / t_tfs, >, 1.9);
        g_assert_cmpfloat((double)t_rfs / t_tfs, <, 2.1);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-aic/rates/%s-%u-%u",
                                                cases[i].machine,
                                                cases[i].tx_m, cases[i].rx_m);
        qtest_add_data_func(path, &cases[i], test_rates);
    }
    return g_test_run();
}
