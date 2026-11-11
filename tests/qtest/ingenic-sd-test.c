/*
 * QTest testcase for the Ingenic SD slot's card-detect GPIO
 *
 * With sd-cd naming the board's card-detect GPIO, the pin reads low while
 * the first slot holds a card and high once it is ejected, as a socket's
 * switch drives it, on the MSC (T20, T21) and on the SDHCI (T32, T41).
 * The SDHCI's read-only registers read as a T32LQ, T41NQ or A1N reads
 * them: its capabilities, preset values and version 4.20, with the host
 * regulator voltage stable. No pin reaches the SDHCI's own card-detect
 * input: both slots read a card, stable and the pin high, with or without
 * one, and only the card-detect test bits take it away, which cuts the bus
 * power and raises the removal status, as on a T41LQ and an A1N; power
 * written on after that holds.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GPIO            0x10010000
#define SDHCI           0x13060000

typedef struct {
    const char *machine;
    const char *soc;            /* the device with the sd-cd property */
    uint32_t stride;            /* between GPIO ports */
    int pin;
} SdCase;

static const SdCase cases[] = {
    { "ingenic-t20", "ingenic-t31", 0x100, 43 },
    { "ingenic-t21", "ingenic-t31", 0x1000, 52 },
    { "ingenic-t32", "ingenic-t31", 0x1000, 60 },
    { "ingenic-t41", "ingenic-t40", 0x1000, 59 },
};

static bool pin_high(QTestState *qts, const SdCase *c)
{
    return qtest_readl(qts, GPIO + (c->pin / 32) * c->stride) &
           (1u << (c->pin % 32));
}

static void test_card_detect(const void *data)
{
    const SdCase *c = data;
    g_autofree char *path = NULL;
    int fd = g_file_open_tmp("ingenic-sd-XXXXXX", &path, NULL);
    QTestState *qts;

    g_assert(fd >= 0);
    g_assert(ftruncate(fd, 4 * 1024 * 1024) == 0);
    close(fd);

    qts = qtest_initf("-machine %s -global %s.sd-cd=%d "
                      "-drive file=%s,if=sd,format=raw",
                      c->machine, c->soc, c->pin, path);
    g_assert_false(pin_high(qts, c));
    qtest_qmp_assert_success(qts, "{'execute': 'eject', 'arguments': "
                             "{'device': 'sd0', 'force': true}}");
    g_assert_true(pin_high(qts, c));
    qtest_qmp_assert_success(qts, "{'execute': 'blockdev-change-medium', "
                             "'arguments': {'device': 'sd0', "
                             "'filename': %s, 'format': 'raw'}}", path);
    g_assert_false(pin_high(qts, c));
    qtest_quit(qts);
    unlink(path);
}

static void test_no_card(const void *data)
{
    const SdCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.sd-cd=%d",
                                  c->machine, c->soc, c->pin);

    g_assert_true(pin_high(qts, c));
    qtest_quit(qts);
}

static void test_sdhci_ids(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);
    bool t32 = !strcmp(machine, "ingenic-t32");

    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x40), ==,
                    t32 ? 0x276dc898 : 0x2769fa98);
    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x44), ==, 0x08002077);
    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x60), ==, 0x000401f4);
    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x64), ==, 0x00040002);
    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x68), ==, 0x00010002);
    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x6c), ==, 0x00020000);
    g_assert_cmphex(qtest_readw(qts, SDHCI + 0xfe), ==, 0x0005);
    g_assert_cmphex(qtest_readl(qts, SDHCI + 0x24) & (1u << 25), ==, 1u << 25);
    qtest_quit(qts);
}

static void test_sdhci_log(const void *data)
{
    g_autofree char *path = NULL;
    g_autofree char *log = NULL;
    int fd = g_file_open_tmp("ingenic-sd-log-XXXXXX", &path, NULL);
    QTestState *qts;

    g_assert(fd >= 0);
    close(fd);
    qts = qtest_initf("-machine %s -d unimp,guest_errors -D %s",
                      (const char *)data, path);
    qtest_quit(qts);
    g_assert(g_file_get_contents(path, &log, NULL, NULL));
    g_assert_null(strstr(log, "unknown CAPAB"));
    unlink(path);
}

static void test_cd_tied(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);

    for (int slot = 0; slot < 2; slot++) {
        uint32_t base = SDHCI + slot * 0x10000;

        g_assert_cmphex(qtest_readl(qts, base + 0x24) & 0x70000, ==, 0x70000);
        qtest_writel(qts, base + 0x34, 0xc0);
        qtest_writel(qts, base + 0x28, 0xf00);
        g_assert_cmphex(qtest_readl(qts, base + 0x28), ==, 0xf00);
        g_assert_cmphex(qtest_readl(qts, base + 0x30), ==, 0);
        /* test level 0: no card */
        qtest_writel(qts, base + 0x28, 0xf80);
        g_assert_cmphex(qtest_readl(qts, base + 0x24) & 0x70000, ==, 0x60000);
        g_assert_cmphex(qtest_readl(qts, base + 0x28), ==, 0xe80);
        g_assert_cmphex(qtest_readl(qts, base + 0x30), ==, 0x80);
        qtest_writel(qts, base + 0x30, 0xffff);
        qtest_writel(qts, base + 0x28, 0xf80);
        g_assert_cmphex(qtest_readl(qts, base + 0x28), ==, 0xf80);
        /* test level 1: a card again */
        qtest_writel(qts, base + 0x28, 0xc0);
        g_assert_cmphex(qtest_readl(qts, base + 0x24) & 0x70000, ==, 0x70000);
        g_assert_cmphex(qtest_readl(qts, base + 0x30), ==, 0x40);
        qtest_writel(qts, base + 0x30, 0xffff);
        qtest_writel(qts, base + 0x28, 0);
        g_assert_cmphex(qtest_readl(qts, base + 0x30), ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *with = g_strdup_printf("ingenic-sd/card-detect/%s",
                                                cases[i].machine);
        g_autofree char *without = g_strdup_printf("ingenic-sd/no-card/%s",
                                                   cases[i].machine);

        qtest_add_data_func(with, &cases[i], test_card_detect);
        qtest_add_data_func(without, &cases[i], test_no_card);
    }
    qtest_add_data_func("ingenic-sd/sdhci-ids/t32", "ingenic-t32",
                        test_sdhci_ids);
    qtest_add_data_func("ingenic-sd/sdhci-ids/t41", "ingenic-t41",
                        test_sdhci_ids);
    qtest_add_data_func("ingenic-sd/sdhci-ids/a1", "ingenic-a1",
                        test_sdhci_ids);
    qtest_add_data_func("ingenic-sd/sdhci-log/t32", "ingenic-t32",
                        test_sdhci_log);
    qtest_add_data_func("ingenic-sd/sdhci-log/t41", "ingenic-t41",
                        test_sdhci_log);
    qtest_add_data_func("ingenic-sd/sdhci-log/a1", "ingenic-a1",
                        test_sdhci_log);
    qtest_add_data_func("ingenic-sd/cd-tied/t32", "ingenic-t32",
                        test_cd_tied);
    qtest_add_data_func("ingenic-sd/cd-tied/t41", "ingenic-t41",
                        test_cd_tied);
    qtest_add_data_func("ingenic-sd/cd-tied/a1", "ingenic-a1",
                        test_cd_tied);
    return g_test_run();
}
