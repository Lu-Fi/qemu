/*
 * QTest testcase for the Ingenic static memory controller (NEMC, NFI)
 *
 * The register file as a T20X, T30X and T31X read it at U-Boot, alike on
 * all three: 0x00 and bank 0's control read-only, banks 1 to 6's control
 * and banks 0 to 6's address words, NFCSR's twelve bits, nothing else in
 * the 64 KiB; the bank windows read 0 with no device on the bus. The other
 * parts carry the same file, unmeasured.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NEMC            0x13410000

/* Reads reset, then all ones written reads kept | (reset & ~mask) */
static void check(QTestState *qts, uint32_t addr, uint32_t reset,
                  uint32_t mask)
{
    g_test_message("0x%08x", addr);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset);
    qtest_writel(qts, addr, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset | mask);
    qtest_writel(qts, addr, 0);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset & ~mask);
    qtest_writel(qts, addr, reset);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset);
}

static void test_regs(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);
    /* the T31 family's boards put RAM in CS2 for bootrom work */
    bool cs2_ram = !g_str_equal(machine, "ingenic-t40") &&
                   !g_str_equal(machine, "ingenic-t41") &&
                   !g_str_equal(machine, "ingenic-a1");

    check(qts, NEMC + 0x00, 0x00000001, 0);
    check(qts, NEMC + 0x10, 0x00000080, 0);
    for (int n = 1; n <= 6; n++) {
        check(qts, NEMC + 0x10 + 4 * n, 0x0fff7700, 0x3fffffc7);
    }
    check(qts, NEMC + 0x30, 0x00001cfc, 0x0000ffff);
    for (int n = 1; n <= 6; n++) {
        check(qts, NEMC + 0x30 + 4 * n, ((0x1c - n) << 8) | 0xff, 0x0000ffff);
    }
    check(qts, NEMC + 0x50, 0, 0x00000fff);
    for (uint32_t o = 0x04; o < 0x10; o += 4) {
        check(qts, NEMC + o, 0, 0);
    }
    check(qts, NEMC + 0x2c, 0, 0);
    check(qts, NEMC + 0x4c, 0, 0);
    for (uint32_t o = 0x54; o < 0x10000; o += 0x4c4) {
        check(qts, NEMC + o, 0, 0);
    }
    check(qts, NEMC + 0xfffc, 0, 0);

    for (int n = 1; n <= 6; n++) {
        uint32_t bank = (0x1c - n) << 24;

        if (n == 2 && cs2_ram) {
            continue;
        }
        g_test_message("CS%d", n);
        for (uint32_t o = 0; o < 0x01000000; o += 0x400000) {
            qtest_writel(qts, bank + o, 0x5a5a5a5a);
            g_assert_cmphex(qtest_readl(qts, bank + o), ==, 0);
        }
        g_assert_cmphex(qtest_readl(qts, bank + 0xfffffc), ==, 0);
    }
    qtest_quit(qts);
}

#define BANK1           0x1b000000
#define CMD             0x400000
#define ADDR            0x800000
#define PAGE            2048
#define OOB             64
#define STRIDE          (PAGE + OOB)
#define BLOCK_PAGES     64
#define IMAGE_SIZE      (2 * BLOCK_PAGES * STRIDE)

static void nand_cmd(QTestState *qts, uint8_t c)
{
    qtest_writeb(qts, BANK1 + CMD, c);
}

static void nand_addr(QTestState *qts, uint8_t a)
{
    qtest_writeb(qts, BANK1 + ADDR, a);
}

/* Command, column 0 and the row, as a 2-cycle-row chip is addressed */
static void nand_page_addr(QTestState *qts, uint32_t col, uint32_t row)
{
    nand_addr(qts, col);
    nand_addr(qts, col >> 8);
    nand_addr(qts, row);
    nand_addr(qts, row >> 8);
}

static void nand_read_page(QTestState *qts, uint32_t row, uint8_t *buf,
                           uint32_t len)
{
    nand_cmd(qts, 0x00);
    nand_page_addr(qts, 0, row);
    nand_cmd(qts, 0x30);
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = qtest_readb(qts, BANK1);
    }
}

static void test_nand(void)
{
    g_autofree char *path = NULL;
    int fd = g_file_open_tmp("ingenic-nand-XXXXXX", &path, NULL);
    uint8_t ff[STRIDE], data[PAGE], back[PAGE];
    QTestState *qts;

    g_assert_cmpint(fd, >=, 0);
    memset(ff, 0xff, sizeof(ff));
    for (int i = 0; i < 2 * BLOCK_PAGES; i++) {
        g_assert_cmpint(write(fd, ff, STRIDE), ==, STRIDE);
    }
    close(fd);
    for (int i = 0; i < PAGE; i++) {
        data[i] = i * 7 + 1;
    }
    /* -blockdev, as the board takes the first if=none -drive for its flash */
    qts = qtest_initf("-machine ingenic-t31 -blockdev driver=file,"
                      "node-name=nd0,filename=%s -device "
                      "ingenic-nemc-nand,bank=1,drive=nd0", path);

    /* Not selected: the window reads 0 whatever the chip holds */
    nand_cmd(qts, 0x90);
    g_assert_cmphex(qtest_readl(qts, BANK1), ==, 0);
    /* NFE alone, or NFCE alone, does not select it */
    qtest_writel(qts, NEMC + 0x50, 1);
    g_assert_cmphex(qtest_readl(qts, BANK1), ==, 0);
    qtest_writel(qts, NEMC + 0x50, 2);
    g_assert_cmphex(qtest_readl(qts, BANK1), ==, 0);
    qtest_writel(qts, NEMC + 0x50, 3);

    nand_cmd(qts, 0xff);
    nand_cmd(qts, 0x70);
    g_assert_cmphex(qtest_readb(qts, BANK1), ==, 0xe0);
    nand_cmd(qts, 0x90);
    nand_addr(qts, 0x00);
    g_assert_cmphex(qtest_readl(qts, BANK1), ==, 0x9500f1ec);
    g_assert_cmphex(qtest_readb(qts, BANK1), ==, 0x40);
    nand_cmd(qts, 0x90);
    nand_addr(qts, 0x20);
    g_assert_cmphex(qtest_readl(qts, BANK1), ==, 0x49464e4f);

    /* An erased page reads all ones, spare area included */
    nand_read_page(qts, 70, back, PAGE);
    g_assert_cmphex(back[0], ==, 0xff);
    g_assert_cmphex(back[PAGE - 1], ==, 0xff);

    /* Program page 70 (the second block), read it back, and part of it */
    nand_cmd(qts, 0x80);
    nand_page_addr(qts, 0, 70);
    for (int i = 0; i < PAGE; i++) {
        qtest_writeb(qts, BANK1, data[i]);
    }
    nand_cmd(qts, 0x10);
    nand_cmd(qts, 0x70);
    g_assert_cmphex(qtest_readb(qts, BANK1) & 1, ==, 0);
    nand_read_page(qts, 70, back, PAGE);
    g_assert(memcmp(back, data, PAGE) == 0);
    nand_cmd(qts, 0x05);
    nand_addr(qts, 0x10);
    nand_addr(qts, 0x00);
    nand_cmd(qts, 0xe0);
    g_assert_cmphex(qtest_readb(qts, BANK1), ==, data[0x10]);
    g_assert_cmphex(qtest_readb(qts, BANK1), ==, data[0x11]);
    nand_read_page(qts, 71, back, 4);
    g_assert_cmphex(back[0], ==, 0xff);

    /* A second program only clears bits: 0x0f over what is there */
    nand_cmd(qts, 0x80);
    nand_page_addr(qts, 0, 70);
    qtest_writeb(qts, BANK1, 0x0f);
    nand_cmd(qts, 0x10);
    nand_read_page(qts, 70, back, 2);
    g_assert_cmphex(back[0], ==, data[0] & 0x0f);
    g_assert_cmphex(back[1], ==, data[1]);

    /* The first block's pages are untouched; erasing the second clears 70 */
    nand_read_page(qts, 3, back, 1);
    g_assert_cmphex(back[0], ==, 0xff);
    nand_cmd(qts, 0x60);
    nand_addr(qts, 64);
    nand_addr(qts, 0);
    nand_cmd(qts, 0xd0);
    nand_read_page(qts, 70, back, PAGE);
    g_assert_cmphex(back[0], ==, 0xff);
    g_assert_cmphex(back[PAGE - 1], ==, 0xff);

    /* The latches are write only, and an address in the next bank is a miss */
    g_assert_cmphex(qtest_readl(qts, BANK1 + CMD), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x1a000000 + 0x2000), ==, 0);
    qtest_writel(qts, NEMC + 0x50, 0);
    g_assert_cmphex(qtest_readl(qts, BANK1), ==, 0);
    qtest_quit(qts);
    unlink(path);
}

int main(int argc, char **argv)
{
    static const char *const machines[] = {
        "ingenic-t10", "ingenic-t20", "ingenic-t21", "ingenic-t23",
        "ingenic-t30", "ingenic-t31", "ingenic-t32", "ingenic-t33",
        "ingenic-t40", "ingenic-t41", "ingenic-a1",
    };

    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(machines); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-nemc/%s",
                                                machines[i] + 8);

        qtest_add_data_func(path, machines[i], test_regs);
    }
    qtest_add_func("ingenic-nemc/nand", test_nand);
    return g_test_run();
}
