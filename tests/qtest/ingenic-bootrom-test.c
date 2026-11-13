/*
 * QTest testcase for the Ingenic bootrom and its alias over NEMC bank 0
 *
 * A T10L, T20X, T23N, T30X and T31X bootrom repeats every 16 KiB, a T21N,
 * T32LQ, T40N, T41LQ and A1N one every 32 KiB, from 0x1fc00000 and over
 * the whole of bank 0 (0x1c000000 to 0x1fffffff), as each part reads it at
 * its U-Boot. Bank 0's address word (SACR0 at 0x13410030, base 15:8, mask
 * 7:0) places that window: a 16 MiB window is covered when its address bits
 * 31:24 ANDed with the mask equal the base, as a T31X reads it.
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
    { "ingenic-t10", 0x4000, true },
    { "ingenic-t21", 0x8000, true },
    { "ingenic-t32", 0x8000, true },
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

#define SACR0           0x13410030
#define SACR(n)         (SACR0 + 4 * (n))

typedef struct {
    uint16_t word;
    uint16_t covered;   /* bit n: window (0x16 + n) << 24 reads the ROM */
} Decode;

#define WIN(ph)         (1 << ((ph) - 0x16))
#define ALL             0x3ff

/* Each as the T31X read it, 0x16000000 to 0x1fffffff */
static const Decode decodes[] = {
    { 0x1cfc, WIN(0x1c) | WIN(0x1d) | WIN(0x1e) | WIN(0x1f) },
    { 0x1cff, WIN(0x1c) },
    { 0x1dff, WIN(0x1d) },
    { 0x1dfc, 0 },              /* the base is not masked */
    { 0x1cfe, WIN(0x1c) | WIN(0x1d) },
    { 0x1efe, WIN(0x1e) | WIN(0x1f) },
    { 0x1bff, WIN(0x1b) },
    { 0x1bfc, 0 },
    { 0x1afe, WIN(0x1a) | WIN(0x1b) },
    { 0x16ff, WIN(0x16) },
    { 0x16fc, 0 },
    { 0x1c00, 0 },
    { 0x1cf8, 0 },
    { 0x1cc0, 0 },
    { 0xff00, 0 },
    { 0x0000, ALL },            /* every window */
};

static void test_decode(const void *data)
{
    const RomCase *c = data;
    g_autofree char *path = write_image();
    QTestState *qts = qtest_initf("-machine %s -bios %s", c->machine, path);
    /* the T31 family's boards put RAM in CS2 for bootrom work */
    bool cs2_ram = !g_str_equal(c->machine, "ingenic-t40") &&
                   !g_str_equal(c->machine, "ingenic-t41") &&
                   !g_str_equal(c->machine, "ingenic-a1");

    for (int i = 0; i < ARRAY_SIZE(decodes); i++) {
        qtest_writel(qts, SACR(0), decodes[i].word);
        g_assert_cmphex(qtest_readl(qts, SACR(0)), ==, decodes[i].word);
        for (int n = 0; n < 10; n++) {
            uint32_t base = (0x16 + n) << 24;
            uint32_t want = (decodes[i].covered >> n) & 1 ? PATTERN(0x60) : 0;

            if (cs2_ram && base == 0x1a000000) {
                continue;
            }
            g_test_message("SACR0 %04x window %08x", decodes[i].word, base);
            g_assert_cmphex(qtest_readl(qts, base + 0x180), ==, want);
            g_assert_cmphex(qtest_readl(qts, base + c->period + 0x180), ==,
                            want);
        }
        /* The bootrom's own address goes with the 0x1f window */
        g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x180), ==,
                        (decodes[i].covered & WIN(0x1f)) ? PATTERN(0x60) : 0);
    }
    /* Another bank's word, or the same window twice, does not hide it */
    qtest_writel(qts, SACR0, 0x1cfc);
    qtest_writel(qts, SACR(1), 0x1cff);
    g_assert_cmphex(qtest_readl(qts, BANK0 + 0x180), ==, PATTERN(0x60));
    qtest_writel(qts, SACR(1), 0x1bff);
    /* Bank 0 gone from 0x1f and back: the ROM is still there, unaltered */
    qtest_writel(qts, SACR0, 0x1cff);
    g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x180), ==, 0);
    qtest_writel(qts, SACR0, 0x1cfc);
    g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x180), ==, PATTERN(0x60));
    g_assert_cmphex(qtest_readl(qts, BOOTROM + 0x184), ==, PATTERN(0x61));
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

            g_autofree char *dec = g_strdup_printf(
                "ingenic-bootrom/decode/%s", cases[i].machine);

            qtest_add_data_func(eret, &cases[i], test_eret);
            qtest_add_data_func(dec, &cases[i], test_decode);
        }
    }
    return g_test_run();
}
