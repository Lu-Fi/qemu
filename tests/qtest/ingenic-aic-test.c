/*
 * QTest testcase for the Ingenic AIC's frame clocks
 *
 * Where the CPM's I2S clocks drive the AIC (T23, T31, T32, T40, T41), play
 * and record each run at their own clock, as a T31X and a T41NQ do: with the
 * record divider at twice the play divider's M, the record FIFO reaches
 * its threshold in half the time the play FIFO takes to drain to its own.
 * Up to the T30 one divider at 0x60 clocks both ways, as a T20X, T21N and
 * T30X play and record at its rate: twice its M, half the time. RGADW
 * holds an address and RGDATA reads 0 whatever it holds: no part's codec
 * answers there. On the T32 and T41, I2SCR bit 31 sets with the first
 * frame recorded and holds through an AIC reset, record off and a write of
 * 0, as on a T41LQ; playing alone leaves it clear, and a T31 has no such
 * bit. The vendor drivers walk AICFR from its reset value to the internal
 * codec as a slave (ICS set, ICDC clear) one bit at a time, which is not
 * the unmodelled external codec; enabling the AIC with neither set is.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM_CPCCR       0x10000000
#define CPM_I2SCDR_ONE  0x10000060
#define CPM_I2SCDR      0x10000070
#define CPM_I2SCDR1     0x10000084
#define I2S_CE          (1u << 29)
#define CPCCR_EXTCLK    0x55800000      /* SCLK_A and the CPU from EXTCLK */

#define AIC             0x10020000
#define AICFR           0x00
#define AICCR           0x04
#define I2SCR           0x10
#define AICSR           0x14
#define AICDR           0x34
#define RGADW           0xa4
#define RGDATA          0xa8

#define AICFR_ENB       (1u << 0)
#define AICFR_RST       (1u << 3)
#define AICFR_AUSEL     (1u << 4)
#define AICFR_ICS       (1u << 7)
#define I2SCR_RECORDED  (1u << 31)
#define AICCR_EREC      (1u << 0)
#define AICCR_ERPL      (1u << 1)
#define AICSR_TFS       (1u << 3)
#define AICSR_RFS       (1u << 4)

/* 16 one-entry frames each way: TX 64 down to 48, RX up to 16 */
#define AICFR_SETUP     (AICFR_ENB | 24 << 16 | 7 << 24)
#define AICCR_SETUP     (AICCR_EREC | AICCR_ERPL | 1 << 12)

typedef struct {
    const char *machine;
    uint32_t tx_m, rx_m;        /* CPM divider M */
    unsigned ratio;             /* the play threshold's time over record's */
} RateCase;

static const RateCase cases[] = {
    { "ingenic-t31", 16, 32, 2 },
    { "ingenic-t31", 32, 16, 0 },       /* record slower: half */
    { "ingenic-t41", 16, 32, 2 },
};

typedef struct {
    const char *machine;
    unsigned m_shift;           /* M's place in 0x60: 13 on the T10 and T20 */
} OneCase;

static const OneCase one_cases[] = {
    { "ingenic-t10", 13 },
    { "ingenic-t20", 13 },
    { "ingenic-t21", 20 },
    { "ingenic-t30", 20 },
};

/* The time each FIFO takes to reach its threshold once both run */
static void threshold_times(QTestState *qts, int64_t *t_tfs, int64_t *t_rfs)
{
    int64_t t;

    *t_tfs = *t_rfs = -1;
    qtest_writel(qts, AIC + AICFR, AICFR_SETUP);
    for (int i = 0; i < 64; i++) {
        qtest_writel(qts, AIC + AICDR, 0);
    }
    t = qtest_clock_step(qts, 1);
    qtest_writel(qts, AIC + AICCR, AICCR_SETUP);
    for (int i = 0; i < 100000 && (*t_tfs < 0 || *t_rfs < 0); i++) {
        int64_t now = qtest_clock_step_next(qts);
        uint32_t sr = qtest_readl(qts, AIC + AICSR);

        if (*t_tfs < 0 && (sr & AICSR_TFS)) {
            *t_tfs = now - t;
        }
        if (*t_rfs < 0 && (sr & AICSR_RFS)) {
            *t_rfs = now - t;
        }
    }
    g_assert_cmpint(*t_tfs, >, 0);
    g_assert_cmpint(*t_rfs, >, 0);
}

static void test_rates(const void *data)
{
    const RateCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -audio none", c->machine);
    int64_t t_tfs, t_rfs;

    /* SCLK_A from EXTCLK: with nothing to boot a T41's is stopped */
    qtest_writel(qts, CPM_CPCCR, CPCCR_EXTCLK);
    qtest_writel(qts, CPM_I2SCDR, I2S_CE | c->tx_m << 20 | 10875);
    qtest_writel(qts, CPM_I2SCDR1, I2S_CE | c->rx_m << 20 | 10875);
    threshold_times(qts, &t_tfs, &t_rfs);
    if (c->ratio) {
        g_assert_cmpfloat((double)t_tfs / t_rfs, >, c->ratio * 0.95);
        g_assert_cmpfloat((double)t_tfs / t_rfs, <, c->ratio * 1.05);
    } else {
        g_assert_cmpfloat((double)t_rfs / t_tfs, >, 1.9);
        g_assert_cmpfloat((double)t_rfs / t_tfs, <, 2.1);
    }
    qtest_quit(qts);
}

/*
 * The TX FIFO level reads the frames played so far, not the level at the
 * last threshold crossing (an A1N reads 57 and 58 from a playing FIFO that
 * the PDMA keeps full, never a constant 64)
 */
static void test_live_level(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s -audio none", machine);
    int64_t t, wake, frame;
    uint32_t tfl;

    /* The A1's rate is its codec's reset one, without a CPM clock */
    if (!strcmp(machine, "ingenic-t31")) {
        qtest_writel(qts, CPM_CPCCR, CPCCR_EXTCLK);
        qtest_writel(qts, CPM_I2SCDR, I2S_CE | 16 << 20 | 10875);
    }
    qtest_writel(qts, AIC + AICFR, AICFR_ENB | 24 << 16 | 7u << 24);
    for (int i = 0; i < 64; i++) {
        qtest_writel(qts, AIC + AICDR, 0);
    }
    t = qtest_clock_step(qts, 1);
    qtest_writel(qts, AIC + AICCR, AICCR_ERPL | 1 << 12);
    /* The AIC wakes with the level at the threshold, 48: 16 frames */
    do {
        wake = qtest_clock_step_next(qts);
    } while (!(qtest_readl(qts, AIC + AICSR) & AICSR_TFS));
    frame = (wake - t) / 16;
    g_assert_cmpint(frame, >, 0);
    g_assert_cmpuint((qtest_readl(qts, AIC + AICSR) >> 8) & 0x7f, ==, 48);
    qtest_clock_step(qts, 5 * frame);
    tfl = (qtest_readl(qts, AIC + AICSR) >> 8) & 0x7f;
    g_assert_cmpuint(tfl, >=, 42);
    g_assert_cmpuint(tfl, <=, 44);
    qtest_quit(qts);
}

static int64_t one_cdr_time(const OneCase *c, uint32_t m)
{
    QTestState *qts = qtest_initf("-machine %s -audio none", c->machine);
    int64_t t_tfs, t_rfs;

    qtest_writel(qts, CPM_CPCCR, CPCCR_EXTCLK);
    qtest_writel(qts, CPM_I2SCDR_ONE, I2S_CE | m << c->m_shift | 375);
    threshold_times(qts, &t_tfs, &t_rfs);
    g_assert_cmpfloat((double)t_tfs / t_rfs, >, 0.95);
    g_assert_cmpfloat((double)t_tfs / t_rfs, <, 1.05);
    qtest_quit(qts);
    return t_tfs;
}

static void test_one_cdr(const void *data)
{
    double ratio = (double)one_cdr_time(data, 1) / one_cdr_time(data, 2);

    g_assert_cmpfloat(ratio, >, 1.9);
    g_assert_cmpfloat(ratio, <, 2.1);
}

static void test_rgdata(const void *data)
{
    QTestState *qts = qtest_initf("-machine %s -audio none",
                                  (const char *)data);

    for (uint32_t addr = 0; addr < 0x40; addr += 0x11) {
        qtest_writel(qts, AIC + RGADW, addr << 8);
        g_assert_cmphex(qtest_readl(qts, AIC + RGADW), ==, addr << 8);
        g_assert_cmphex(qtest_readl(qts, AIC + RGDATA), ==, 0);
    }
    qtest_quit(qts);
}

static void test_recorded(const void *data)
{
    const char *machine = data;
    bool has = strcmp(machine, "ingenic-t31") != 0;
    QTestState *qts = qtest_initf("-machine %s -audio none", machine);

    qtest_writel(qts, CPM_CPCCR, CPCCR_EXTCLK);
    qtest_writel(qts, CPM_I2SCDR, I2S_CE | 16 << 20 | 10875);
    qtest_writel(qts, CPM_I2SCDR1, I2S_CE | 16 << 20 | 10875);
    g_assert_cmphex(qtest_readl(qts, AIC + I2SCR), ==, 0);
    qtest_writel(qts, AIC + AICFR, AICFR_SETUP);
    qtest_writel(qts, AIC + AICCR, AICCR_ERPL);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AIC + I2SCR), ==, 0);
    qtest_writel(qts, AIC + AICCR, AICCR_EREC);
    qtest_clock_step(qts, 10 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, AIC + I2SCR), ==,
                    has ? I2SCR_RECORDED : 0);
    qtest_writel(qts, AIC + AICCR, 0);
    qtest_writel(qts, AIC + AICFR, AICFR_SETUP | AICFR_RST);
    qtest_writel(qts, AIC + I2SCR, 0);
    g_assert_cmphex(qtest_readl(qts, AIC + I2SCR), ==,
                    has ? I2SCR_RECORDED : 0);
    qtest_quit(qts);
}

static char *codec_log(const char *machine, bool internal, uint32_t base)
{
    g_autofree char *path = NULL;
    char *log = NULL;
    int fd = g_file_open_tmp("ingenic-aic-XXXXXX", &path, NULL);
    QTestState *qts;

    g_assert(fd >= 0);
    close(fd);
    qts = qtest_initf("-machine %s -audio none -d unimp -D %s", machine, path);
    qtest_writel(qts, base + AICFR, 7u << 24 | 8 << 16);
    qtest_writel(qts, base + AICFR, 7u << 24 | 8 << 16 | AICFR_AUSEL);
    if (internal) {
        qtest_writel(qts, base + AICFR,
                     7u << 24 | 8 << 16 | AICFR_AUSEL | AICFR_ICS);
        qtest_writel(qts, base + AICFR,
                     7u << 24 | 8 << 16 | AICFR_AUSEL | AICFR_ICS |
                     AICFR_ENB);
    } else {
        qtest_writel(qts, base + AICFR,
                     7u << 24 | 8 << 16 | AICFR_AUSEL | AICFR_ENB);
    }
    qtest_quit(qts);
    g_assert(g_file_get_contents(path, &log, NULL, NULL));
    unlink(path);
    return log;
}

/*
 * The A1's AIC1 feeds the HDMI transmitter: hdmi_audio.ko enables it with
 * ICDC and ICS clear, and that is no gap; AIC0 with both clear still is
 */
static void test_a1_ext_sink(const void *data)
{
    g_autofree char *hdmi = codec_log(data, false, AIC + 0x1000);
    g_autofree char *aic0 = codec_log(data, false, AIC);

    g_assert_null(strstr(hdmi, "only the internal codec"));
    g_assert_nonnull(strstr(aic0, "only the internal codec"));
}

/* AIC1 as an A1N reads it after hdmi_audio.ko has loaded */
static void test_a1_hdmi_regs(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s -audio none", machine);
    uint32_t aic1 = AIC + 0x1000;

    qtest_writel(qts, aic1 + AICFR, 0x07100015);
    qtest_writel(qts, aic1 + I2SCR, 1u << 17);
    g_assert_cmphex(qtest_readl(qts, aic1 + AICFR), ==, 0x07100015);
    g_assert_cmphex(qtest_readl(qts, aic1 + AICCR), ==, 0x01240000);
    g_assert_cmphex(qtest_readl(qts, aic1 + I2SCR), ==, 0x00020000);
    g_assert_cmphex(qtest_readl(qts, aic1 + AICSR), ==, AICSR_TFS);
    g_assert_cmphex(qtest_readl(qts, aic1 + 0x1c), ==, 0);
    g_assert_cmphex(qtest_readl(qts, aic1 + 0x30), ==, 0x00040004);
    qtest_quit(qts);
}

static void test_codec_log(const void *data)
{
    g_autofree char *internal = codec_log(data, true, AIC);
    g_autofree char *external = codec_log(data, false, AIC);

    g_assert_null(strstr(internal, "only the internal codec"));
    g_assert_nonnull(strstr(external, "only the internal codec"));
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
    for (int i = 0; i < ARRAY_SIZE(one_cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-aic/one-cdr/%s",
                                                one_cases[i].machine);
        qtest_add_data_func(path, &one_cases[i], test_one_cdr);
    }
    qtest_add_data_func("ingenic-aic/live-level/t31", "ingenic-t31",
                        test_live_level);
    qtest_add_data_func("ingenic-aic/live-level/a1", "ingenic-a1",
                        test_live_level);
    qtest_add_data_func("ingenic-aic/rgdata/t31", "ingenic-t31", test_rgdata);
    qtest_add_data_func("ingenic-aic/rgdata/t41", "ingenic-t41", test_rgdata);
    qtest_add_data_func("ingenic-aic/codec-log/t31", "ingenic-t31",
                        test_codec_log);
    qtest_add_data_func("ingenic-aic/codec-log/t41", "ingenic-t41",
                        test_codec_log);
    qtest_add_data_func("ingenic-aic/ext-sink/a1", "ingenic-a1",
                        test_a1_ext_sink);
    qtest_add_data_func("ingenic-aic/hdmi-regs/a1", "ingenic-a1",
                        test_a1_hdmi_regs);
    qtest_add_data_func("ingenic-aic/recorded/t31", "ingenic-t31",
                        test_recorded);
    qtest_add_data_func("ingenic-aic/recorded/t32", "ingenic-t32",
                        test_recorded);
    qtest_add_data_func("ingenic-aic/recorded/t41", "ingenic-t41",
                        test_recorded);
    return g_test_run();
}
