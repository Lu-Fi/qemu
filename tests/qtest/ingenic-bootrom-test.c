/*
 * QTest testcase for the Ingenic bootrom and its alias over NEMC bank 0
 *
 * A T20X, T23N, T30X and T31X bootrom repeats every 16 KiB, a T40N, T41LQ
 * and A1N one every 32 KiB, from 0x1fc00000 and over the whole of bank 0
 * (0x1c000000 to 0x1fffffff), as each part reads it at its U-Boot. The
 * T10, T21 and T32 are not measured so and keep 32 KiB with no alias.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define BOOTROM         0x1fc00000
#define BANK0           0x1c000000
#define BANK0_END       0x20000000
#define IMAGE_SIZE      0x8000
#define ERET            0x42000018

/* Every word of the image its own, so that the two 16 KiB halves differ */
#define PATTERN(i)      (0xb0070000 | (i))

typedef struct {
    const char *machine;
    uint32_t period;
    bool bank0;
} RomCase;

static const RomCase cases[] = {
    { "ingenic-t20", 0x4000, true },
    { "ingenic-t23", 0x4000, true },
    { "ingenic-t30", 0x4000, true },
    { "ingenic-t31", 0x4000, true },
    { "ingenic-t40", 0x8000, true },
    { "ingenic-t41", 0x8000, true },
    { "ingenic-a1", 0x8000, true },
    { "ingenic-t10", 0x8000, false },
    { "ingenic-t21", 0x8000, false },
    { "ingenic-t32", 0x8000, false },
};

static char *write_image(void)
{
    char *path;
    int fd = g_file_open_tmp("ingenic-bootrom-XXXXXX", &path, NULL);
    uint32_t w[IMAGE_SIZE / 4];

    g_assert_cmpint(fd, >=, 0);
    for (unsigned i = 0; i < ARRAY_SIZE(w); i++) {
        w[i] = cpu_to_le32(PATTERN(i));
    }
    g_assert_cmpint(write(fd, w, sizeof(w)), ==, sizeof(w));
    close(fd);
    return path;
}

/* A 32 KiB dump as -bios: the ROM holds its first period */
static void test_alias(const void *data)
{
    const RomCase *c = data;
    g_autofree char *path = write_image();
    QTestState *qts = qtest_initf("-machine %s -bios %s", c->machine, path);
    static const uint32_t offs[] = { 0, 4, 0x180, 0x1ffc, 0x3ffc, 0x7ffc };
    static const uint32_t bases[] = {
        BANK0, 0x1d000000, 0x1e000000, 0x1f000000, 0x1f800000, 0x1fe00000,
    };

    for (int i = 0; i < ARRAY_SIZE(offs); i++) {
        uint32_t o = offs[i];

        if (o >= c->period) {
            continue;
        }
        g_assert_cmphex(qtest_readl(qts, BOOTROM + o), ==, PATTERN(o / 4));
        if (!c->bank0) {
            continue;
        }
        g_assert_cmphex(qtest_readl(qts, BOOTROM + c->period + o), ==,
                        PATTERN(o / 4));
        g_assert_cmphex(qtest_readl(qts, BANK0_END - c->period + o), ==,
                        PATTERN(o / 4));
        for (int b = 0; b < ARRAY_SIZE(bases); b++) {
            g_assert_cmphex(qtest_readl(qts, bases[b] + o), ==,
                            PATTERN(o / 4));
            g_assert_cmphex(qtest_readl(qts, bases[b] + 3 * c->period + o),
                            ==, PATTERN(o / 4));
        }
    }
    if (c->bank0) {
        /* Narrow reads too, and a write changes nothing */
        g_assert_cmphex(qtest_readb(qts, BANK0 + 0x181), ==,
                        (PATTERN(0x60) >> 8) & 0xff);
        g_assert_cmphex(qtest_readw(qts, BANK0 + 0x182), ==,
                        PATTERN(0x60) >> 16);
        qtest_writel(qts, BANK0 + 0x180, 0);
        g_assert_cmphex(qtest_readl(qts, BANK0 + 0x180), ==, PATTERN(0x60));
        g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x180), ==, PATTERN(0x60));
    } else {
        g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x4000), ==,
                        PATTERN(0x1000));
    }
    qtest_quit(qts);
    unlink(path);
}

/* Without -bios the vectors' ERET reads through the alias too */
static void test_eret(const void *data)
{
    const RomCase *c = data;
    QTestState *qts = qtest_initf("-machine %s", c->machine);

    g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x180), ==, ERET);
    g_assert_cmphex(qtest_readl(qts, BANK0 + 0x180), ==, ERET);
    g_assert_cmphex(qtest_readl(qts, BOOTROM + c->period + 0x380), ==, ERET);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *alias = g_strdup_printf("ingenic-bootrom/alias/%s",
                                                 cases[i].machine);

        qtest_add_data_func(alias, &cases[i], test_alias);
        if (cases[i].bank0) {
            g_autofree char *eret = g_strdup_printf("ingenic-bootrom/eret/%s",
                                                    cases[i].machine);

            qtest_add_data_func(eret, &cases[i], test_eret);
        }
    }
    return g_test_run();
}
