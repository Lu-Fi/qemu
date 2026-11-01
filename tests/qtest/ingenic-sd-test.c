/*
 * QTest testcase for the Ingenic SD slot's card-detect GPIO
 *
 * With sd-cd naming the board's card-detect GPIO, the pin reads low while
 * the first slot holds a card and high once it is ejected, as a socket's
 * switch drives it, on the MSC (T20, T21) and on the SDHCI (T32, T41).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define GPIO            0x10010000

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
    return g_test_run();
}
