/*
 * QTest testcase for the GD25Q256 behind the T40's SFC
 *
 * The opcodes the 4.4 sfc-nor driver issues while it probes the part and
 * that the model used to log as unimplemented: the individual block lock
 * and unlock (0x36, 0x39), the lock read (0x3d), the global lock and
 * unlock (0x7e, 0x98) and the extended address register (0xc5, 0xc8),
 * with the protection they set up: with SR3.WPS the part comes up locked
 * and refuses to erase a locked block.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define SFC             0x13440000
#define SFC_GLB         0x00
#define SFC_TRAN_CONF0  0x14
#define SFC_TRAN_LEN    0x2c
#define SFC_DEV_ADDR0   0x30
#define SFC_TRIG        0x64
#define SFC_SR          0x68
#define SFC_SCR         0x6c
#define SFC_DR          0x1000

#define GLB_TRAN_DIR    (1 << 13)
#define TRAN_DATEEN     (1 << 16)
#define TRIG_START      1
#define SR_END          (1 << 4)

#define FLASH_SIZE      (32 * 1024 * 1024)
#define SR3_ADS         0x01
#define SR3_WPS         0x04

static uint32_t cmd(QTestState *qts, uint8_t op, uint32_t addr, uint32_t len,
                    bool out, uint32_t data)
{
    uint32_t v = 0;

    qtest_writel(qts, SFC + SFC_SCR, 0x1f);
    qtest_writel(qts, SFC + SFC_GLB, out ? GLB_TRAN_DIR : 0);
    qtest_writel(qts, SFC + SFC_TRAN_CONF0, op | (len ? TRAN_DATEEN : 0));
    qtest_writel(qts, SFC + SFC_TRAN_LEN, len);
    qtest_writel(qts, SFC + SFC_DEV_ADDR0, addr);
    qtest_writel(qts, SFC + SFC_TRIG, TRIG_START);
    if (len && out) {
        qtest_writel(qts, SFC + SFC_DR, data);
    } else if (len) {
        v = qtest_readl(qts, SFC + SFC_DR);
    }
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SR) & SR_END, ==, SR_END);
    return v;
}

static void wren(QTestState *qts)
{
    cmd(qts, 0x06, 0, 0, false, 0);
}

static uint8_t read_byte(QTestState *qts, uint8_t op, uint32_t addr)
{
    return cmd(qts, op, addr, 1, false, 0) & 0xff;
}

static void unlock(QTestState *qts, uint32_t addr)
{
    wren(qts);
    cmd(qts, 0x39, addr, 0, false, 0);
}

static void erase_4k(QTestState *qts, uint32_t addr)
{
    wren(qts);
    cmd(qts, 0x20, addr, 0, false, 0);
}

static QTestState *start(const char **path, char **logp)
{
    g_autofree char *img = NULL;
    int fd = g_file_open_tmp("ingenic-nor-XXXXXX", &img, NULL);
    int lfd = g_file_open_tmp("ingenic-nor-log-XXXXXX", logp, NULL);
    g_autofree uint8_t *flash = NULL;
    QTestState *qts;

    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(lfd, >=, 0);
    /* erased, with the SPL header the board looks for at the start */
    flash = g_malloc(FLASH_SIZE);
    memset(flash, 0xff, FLASH_SIZE);
    stl_le_p(flash, 0x03040506);
    stl_le_p(flash + 12, 0x100);
    g_assert_cmpint(write(fd, flash, FLASH_SIZE), ==, FLASH_SIZE);
    close(fd);
    close(lfd);
    *path = g_steal_pointer(&img);
    qts = qtest_initf("-machine ingenic-t40 -drive if=none,format=raw,file=%s "
                      "-d unimp -D %s", *path, *logp);
    return qts;
}

static void test_lock(void)
{
    const char *img;
    char *log = NULL;
    g_autofree char *contents = NULL;
    QTestState *qts = start(&img, &log);

    g_assert_cmphex(cmd(qts, 0x9f, 0, 3, false, 0) & 0xffffff, ==, 0x1940c8);

    /* WPS on: every block is locked from power up */
    wren(qts);
    cmd(qts, 0x11, 0, 1, true, SR3_WPS);
    g_assert_cmphex(read_byte(qts, 0x15, 0), ==, SR3_WPS);
    wren(qts);
    cmd(qts, 0xb7, 0, 0, false, 0);
    g_assert_cmphex(read_byte(qts, 0x15, 0), ==, SR3_WPS | SR3_ADS);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x100000), ==, 0xff);
    g_assert_cmphex(read_byte(qts, 0x3d, FLASH_SIZE - 0x1000), ==, 0xff);

    /* a locked block refuses a program */
    wren(qts);
    cmd(qts, 0x02, 0x100000, 4, true, 0);
    g_assert_cmphex(cmd(qts, 0x03, 0x100000, 4, false, 0), ==, 0xffffffff);

    /* a 64 KiB lock in the middle of the array, a 4 KiB one at the ends */
    unlock(qts, 0x100000);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x100000), ==, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x10f000), ==, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x110000), ==, 0xff);
    unlock(qts, 0x1000);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x1000), ==, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x2000), ==, 0xff);
    unlock(qts, FLASH_SIZE - 0x1000);
    g_assert_cmphex(read_byte(qts, 0x3d, FLASH_SIZE - 0x1000), ==, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, FLASH_SIZE - 0x2000), ==, 0xff);

    /* an unlocked block programs, a locked one does not */
    wren(qts);
    cmd(qts, 0x02, 0x100000, 4, true, 0x12345678);
    g_assert_cmphex(cmd(qts, 0x03, 0x100000, 4, false, 0), ==, 0x12345678);
    wren(qts);
    cmd(qts, 0x02, 0x110000, 4, true, 0x12345678);
    g_assert_cmphex(cmd(qts, 0x03, 0x110000, 4, false, 0), ==, 0xffffffff);

    /* an unlocked block erases */
    erase_4k(qts, 0x100000);
    g_assert_cmphex(cmd(qts, 0x03, 0x100000, 4, false, 0), ==, 0xffffffff);

    /* a locked one keeps what it holds */
    unlock(qts, 0x120000);
    wren(qts);
    cmd(qts, 0x02, 0x120000, 4, true, 0);
    wren(qts);
    cmd(qts, 0x36, 0x120000, 0, false, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x120000), ==, 0xff);
    erase_4k(qts, 0x120000);
    g_assert_cmphex(cmd(qts, 0x03, 0x120000, 4, false, 0), ==, 0);

    /* the global commands, and no protection without WPS */
    wren(qts);
    cmd(qts, 0x98, 0, 0, false, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x500000), ==, 0);
    wren(qts);
    cmd(qts, 0x7e, 0, 0, false, 0);
    g_assert_cmphex(read_byte(qts, 0x3d, 0x500000), ==, 0xff);
    wren(qts);
    cmd(qts, 0x11, 0, 1, true, 0);
    g_assert_cmphex(read_byte(qts, 0x15, 0), ==, SR3_ADS);
    erase_4k(qts, 0x120000);
    g_assert_cmphex(cmd(qts, 0x03, 0x120000, 4, false, 0), ==, 0xffffffff);

    /* the extended address register */
    g_assert_cmphex(read_byte(qts, 0xc8, 0), ==, 0);
    wren(qts);
    cmd(qts, 0xc5, 0, 1, true, 1);
    g_assert_cmphex(read_byte(qts, 0xc8, 0), ==, 1);

    qtest_quit(qts);
    g_assert_true(g_file_get_contents(log, &contents, NULL, NULL));
    g_assert_null(strstr(contents, "SPI NOR opcode"));
    unlink(img);
    unlink(log);
    g_free(log);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/t40-nor/lock", test_lock);
    return g_test_run();
}
