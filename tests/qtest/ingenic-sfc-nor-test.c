/*
 * QTest testcase for the size, the JEDEC ID and the address wrap of the
 * SPI NOR behind the T31's and the T40's SFC
 *
 * An 8 MiB image is an 8 MiB W25Q64 (EF 40 17), a 16 MiB one a W25Q128 and
 * a 32 MiB one a GD25Q256. The part ignores the address bits above its
 * capacity, so a read, a program and an erase beyond the end land at the
 * same offset modulo the size, and a read that runs off the end continues
 * at 0.
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

#define MiB             (1024 * 1024)
#define SPL_MAGIC       0x03040506
#define MARK            0xcafef00d

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

static QTestState *start(const char *machine, unsigned mib, char **img)
{
    g_autofree uint8_t *flash = g_malloc(mib * MiB);
    int fd = g_file_open_tmp("ingenic-sfc-nor-XXXXXX", img, NULL);

    g_assert_cmpint(fd, >=, 0);
    memset(flash, 0xff, mib * MiB);
    stl_le_p(flash, SPL_MAGIC);
    stl_le_p(flash + 12, 0x100);
    stl_le_p(flash + 0x1000, MARK);
    g_assert_cmpint(write(fd, flash, mib * MiB), ==, mib * MiB);
    close(fd);
    return qtest_initf("-machine %s -drive if=none,format=raw,file=%s",
                       machine, *img);
}

static void test_id(const char *machine)
{
    static const struct {
        unsigned mib;
        uint32_t id;
    } parts[] = {
        { 8, 0x1740ef }, { 16, 0x1840ef }, { 32, 0x1940c8 },
    };

    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *img = NULL;
        QTestState *qts = start(machine, parts[i].mib, &img);

        g_assert_cmphex(cmd(qts, 0x9f, 0, 3, false, 0) & 0xffffff, ==,
                        parts[i].id);
        qtest_quit(qts);
        unlink(img);
    }
}

static void test_wrap(const char *machine)
{
    g_autofree char *img = NULL;
    QTestState *qts = start(machine, 8, &img);
    const uint32_t end = 8 * MiB;

    g_assert_cmphex(cmd(qts, 0x03, 0x1000, 4, false, 0), ==, MARK);
    g_assert_cmphex(cmd(qts, 0x03, end + 0x1000, 4, false, 0), ==, MARK);
    g_assert_cmphex(cmd(qts, 0x03, 3 * end + 0x1000, 4, false, 0), ==, MARK);
    g_assert_cmphex(cmd(qts, 0x03, end, 4, false, 0), ==, SPL_MAGIC);

    /* a program and an erase beyond the end hit the low addresses */
    wren(qts);
    cmd(qts, 0x02, end + 0x2000, 4, true, 0x12345678);
    g_assert_cmphex(cmd(qts, 0x03, 0x2000, 4, false, 0), ==, 0x12345678);
    wren(qts);
    cmd(qts, 0x20, end + 0x1000, 0, false, 0);
    g_assert_cmphex(cmd(qts, 0x03, 0x1000, 4, false, 0), ==, 0xffffffff);

    qtest_quit(qts);
    unlink(img);
}

static void t31_id(void)
{
    test_id("ingenic-t31");
}

static void t31_wrap(void)
{
    test_wrap("ingenic-t31");
}

static void t40_id(void)
{
    test_id("ingenic-t40");
}

static void t40_wrap(void)
{
    test_wrap("ingenic-t40");
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/sfc-nor/t31/id", t31_id);
    qtest_add_func("ingenic/sfc-nor/t31/wrap", t31_wrap);
    qtest_add_func("ingenic/sfc-nor/t40/id", t40_id);
    qtest_add_func("ingenic/sfc-nor/t40/wrap", t40_wrap);
    return g_test_run();
}
