/*
 * QTest testcase for the XBurst2 SRAM the L2 cache shares with the NNA
 *
 * CCU MSCR bits 12:10 split the SRAM (1 MiB on a T40 or A1, 512 KiB on a
 * T41) between the L2 and the NNA's ORAM. Each case checks a part against
 * what it read on silicon (a T40XP, a T41NQ, an A1N under U-Boot): MSCR
 * from reset, Config2 at each share (read by a program under TCG), the
 * ORAM moving up past the L2's share, an access to the L2's share
 * reaching the SRAM's last 2 KiB, the T41's SRAM repeating in the
 * window's upper half, bytes kept across splits, and the NNDMA writing
 * the L2's own SRAM where the CPUs see the last 2 KiB.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define MSCR            0x12200060
#define SRAM            0x12600000
#define DESRAM          0x12500000
#define NNDMA           0x12508000
#define DDR             0x00800000
#define START           (1u << 31)

static void mscr(QTestState *qts, uint32_t v)
{
    qtest_writel(qts, MSCR, v);
}

/* T40: 512 KiB of L2 from reset, the ORAM from 0x12680000 */
static void test_t40_window(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    g_assert_cmphex(qtest_readl(qts, MSCR), ==, 0x00000c00);

    /* the L2's share reaches the last 2 KiB, both ways */
    qtest_writel(qts, 0x126ff800, 0x11112222);
    g_assert_cmphex(qtest_readl(qts, 0x12634000), ==, 0x11112222);
    qtest_writel(qts, 0x12600804, 0x33334444);
    g_assert_cmphex(qtest_readl(qts, 0x126ff804), ==, 0x33334444);
    qtest_writel(qts, 0x12680000, 0xa0a0a0a0);
    g_assert_cmphex(qtest_readl(qts, 0x12680000), ==, 0xa0a0a0a0);

    /* 128 KiB of L2: the ORAM from 0x12620000 */
    mscr(qts, 0x00000400);
    qtest_writel(qts, 0x12650000, 0x5a5a0001);
    g_assert_cmphex(qtest_readl(qts, 0x12650000), ==, 0x5a5a0001);
    g_assert_cmphex(qtest_readl(qts, 0x1261f800), ==, 0x11112222);

    /* back in the L2's share, then the ORAM's again: the bytes kept */
    mscr(qts, 0x00000c00);
    g_assert_cmphex(qtest_readl(qts, 0x12650000), ==, 0x11112222);
    mscr(qts, 0x00000400);
    g_assert_cmphex(qtest_readl(qts, 0x12650000), ==, 0x5a5a0001);

    /* 5 to 7 read back 4: 1 MiB of L2, no ORAM */
    mscr(qts, 0x00001400);
    g_assert_cmphex(qtest_readl(qts, MSCR), ==, 0x00001000);
    g_assert_cmphex(qtest_readl(qts, 0x126c0000), ==, 0x11112222);
    mscr(qts, 0x00001c00);
    g_assert_cmphex(qtest_readl(qts, MSCR), ==, 0x00001000);

    /* none: the ORAM is the whole SRAM */
    mscr(qts, 0x00000000);
    qtest_writel(qts, 0x12600010, 0x0badcafe);
    g_assert_cmphex(qtest_readl(qts, 0x12600010), ==, 0x0badcafe);
    g_assert_cmphex(qtest_readl(qts, 0x126ff810), !=, 0x0badcafe);
    qtest_quit(qts);
}

/* T41: a 512 KiB SRAM, 256 KiB of L2 and the AIP's clock from reset */
static void test_t41_window(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    g_assert_cmphex(qtest_readl(qts, MSCR), ==, 0x02000800);
    qtest_writel(qts, 0x12640000, 0xb2640000);
    g_assert_cmphex(qtest_readl(qts, 0x126c0000), ==, 0xb2640000);
    qtest_writel(qts, 0x1267f800, 0xb267f800);
    g_assert_cmphex(qtest_readl(qts, 0x12600000), ==, 0xb267f800);
    g_assert_cmphex(qtest_readl(qts, 0x126ff800), ==, 0xb267f800);

    /* the upper half shows the L2's share as the SRAM itself */
    mscr(qts, 0x02000000);
    qtest_writel(qts, 0x12600100, 0xc0ffee00);
    mscr(qts, 0x02000800);
    g_assert_cmphex(qtest_readl(qts, 0x12680100), ==, 0xc0ffee00);
    g_assert_cmphex(qtest_readl(qts, 0x12600100), !=, 0xc0ffee00);
    qtest_quit(qts);
}

static void put_desc(QTestState *qts, unsigned idx, uint32_t ddr,
                     uint32_t offset)
{
    qtest_writel(qts, DESRAM + idx * 8, ddr);
    qtest_writel(qts, DESRAM + idx * 8 + 4, offset / 64);
}

/* A1: 512 KiB of L2 and bit 4 from reset; the NNDMA keeps the T40's bits */
static void test_a1(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(qtest_readl(qts, MSCR), ==, 0x02000c10);
    qtest_writel(qts, DESRAM, 0xffffffff);
    qtest_writel(qts, DESRAM + 4, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DESRAM), ==, 0xffffffc7);
    g_assert_cmphex(qtest_readl(qts, DESRAM + 4), ==, 0xfff03fff);

    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, DDR + 4 * i, 0xa1a10000 + i);
    }
    put_desc(qts, 1, DDR, 0xc3000);
    qtest_writel(qts, NNDMA + 4, 1 | START);
    g_assert_cmphex(qtest_readl(qts, NNDMA + 4), ==, 0x01fff802);
    g_assert_cmphex(qtest_readl(qts, 0x126c3000), ==, 0xa1a10000);
    g_assert_cmphex(qtest_readl(qts, 0x126c303c), ==, 0xa1a1000f);
    qtest_quit(qts);
}

/* T40: the NNDMA writes the L2's own SRAM, which the CPUs do not see */
static void test_t40_nndma_l2(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    qtest_writel(qts, 0x126ff800, 0x11112222);
    for (int i = 0; i < 16; i++) {
        qtest_writel(qts, DDR + 4 * i, 0xd0d00000 + i);
    }
    put_desc(qts, 0, DDR, 0x20000);
    qtest_writel(qts, NNDMA + 4, START);
    g_assert_cmphex(qtest_readl(qts, 0x12620000), ==, 0x11112222);
    mscr(qts, 0x00000400);
    g_assert_cmphex(qtest_readl(qts, 0x12620000), ==, 0xd0d00000);
    g_assert_cmphex(qtest_readl(qts, 0x1262003c), ==, 0xd0d0000f);
    qtest_quit(qts);
}

/* --- Config2, read by the CPU after each MSCR it writes */

#define PROG_LOAD       0x80010000
#define PROG_OUT        0x00200000
#define PROG_DONE       0x00201000

enum { T0 = 8, T1, T2, T3 };

static uint32_t lui(int rt, uint16_t imm)
{
    return 0x3c000000 | rt << 16 | imm;
}

static uint32_t ori(int rt, int rs, uint16_t imm)
{
    return 0x34000000 | rs << 21 | rt << 16 | imm;
}

static uint32_t sw(int rt, int base, uint16_t off)
{
    return 0xac000000 | base << 21 | rt << 16 | off;
}

static uint32_t lw(int rt, int base, uint16_t off)
{
    return 0x8c000000 | base << 21 | rt << 16 | off;
}

/* A uImage that writes each MSCR, stores it read back and Config2 */
static char *config2_image(const uint32_t *v, unsigned n)
{
    size_t size = 4096;
    g_autofree uint8_t *img = g_malloc0(64 + size);
    uint8_t *code = img + 64;
    unsigned k = 0;
    char *path;
    int fd = g_file_open_tmp("ingenic-l2sram-prog-XXXXXX", &path, NULL);

    stl_le_p(&code[4 * k++], lui(T0, 0xb220));
    stl_le_p(&code[4 * k++], lui(T1, 0x8020));
    for (unsigned i = 0; i < n; i++) {
        stl_le_p(&code[4 * k++], lui(T2, v[i] >> 16));
        stl_le_p(&code[4 * k++], ori(T2, T2, v[i]));
        stl_le_p(&code[4 * k++], sw(T2, T0, 0x60));
        stl_le_p(&code[4 * k++], lw(T3, T0, 0x60));
        stl_le_p(&code[4 * k++], sw(T3, T1, 8 * i));
        stl_le_p(&code[4 * k++], 0x40008002 | T3 << 16);   /* mfc0 Config2 */
        stl_le_p(&code[4 * k++], sw(T3, T1, 8 * i + 4));
    }
    stl_le_p(&code[4 * k++], 0x24000000 | T3 << 16 | 0x600d);
    stl_le_p(&code[4 * k++], sw(T3, T1, 0x1000));
    stl_le_p(&code[4 * k++], 0x1000ffff);                  /* b . */
    stl_le_p(&code[4 * k++], 0);
    g_assert(4 * k <= size);

    stl_be_p(img, 0x27051956);
    stl_be_p(img + 12, size);
    stl_be_p(img + 16, PROG_LOAD);
    stl_be_p(img + 20, PROG_LOAD);
    img[28] = 5;                        /* Linux */
    img[29] = 5;                        /* MIPS */
    img[30] = 2;                        /* kernel */
    g_assert(write(fd, img, 64 + size) == 64 + size);
    close(fd);
    return path;
}

/* As a T40XP reads them, the last field 5 (back as 4) */
static void test_t40_config2(void)
{
    static const uint32_t v[] = {
        0x00000000, 0x00000400, 0x00000800, 0x00000c00, 0x00001000,
        0x00001400,
    };
    static const uint32_t want[][2] = {
        { 0x00000000, 0x80002057 }, { 0x00000400, 0x80002257 },
        { 0x00000800, 0x80002357 }, { 0x00000c00, 0x80002457 },
        { 0x00001000, 0x80002557 }, { 0x00001000, 0x80002557 },
    };
    g_autofree char *image = config2_image(v, ARRAY_SIZE(v));
    QTestState *qts = qtest_initf("-machine ingenic-t40 -m 128 -kernel %s"
                                  " -accel tcg", image);
    int i;

    for (i = 0; i < 1000 && qtest_readl(qts, PROG_DONE) != 0x600d; i++) {
        g_usleep(10 * 1000);
    }
    g_assert_cmphex(qtest_readl(qts, PROG_DONE), ==, 0x600d);
    for (i = 0; i < ARRAY_SIZE(v); i++) {
        g_assert_cmphex(qtest_readl(qts, PROG_OUT + 8 * i), ==, want[i][0]);
        g_assert_cmphex(qtest_readl(qts, PROG_OUT + 8 * i + 4), ==,
                        want[i][1]);
    }
    qtest_quit(qts);
    unlink(image);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-l2sram/t40/window", test_t40_window);
    qtest_add_func("ingenic-l2sram/t41/window", test_t41_window);
    qtest_add_func("ingenic-l2sram/a1", test_a1);
    qtest_add_func("ingenic-l2sram/t40/nndma-l2", test_t40_nndma_l2);
    qtest_add_func("ingenic-l2sram/t40/config2", test_t40_config2);
    return g_test_run();
}
