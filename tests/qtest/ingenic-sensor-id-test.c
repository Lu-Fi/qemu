/*
 * QTest testcase for the chip IDs a Jooan T31LC's sensors answer with
 *
 * The camera's sinfo module walks its table of sensors, reading each one's
 * ID registers at its address, and loads the driver of the first that
 * answers: a mis2008 (0x30, 16-bit registers 0x3000/0x3001 = 0x20, 0x08)
 * ahead of an os02g10 (0x3c, 8-bit registers 0x02-0x04 = 0x56, 0x02, 0x47,
 * the page register 0xfd first). A sensor answers at its own address only.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2C0            0x10050000
#define TAR             0x04
#define DC              0x10
#define RAWINT          0x34
#define CTXABRT         0x54
#define ENB             0x6c
#define RXFLR           0x78
#define TXABRT          0x80

#define CMD_READ        0x100
#define CMD_STOP        0x200
#define INT_TX_ABRT     (1u << 6)
#define ABRT_7B_NOACK   1u

static QTestState *start(const char *model)
{
    return qtest_initf("-machine ingenic-t31 "
                       "-global ingenic-t31.soc-variant=t31lc "
                       "-device image-sensor,bus=i2c0,model=%s", model);
}

static void select_target(QTestState *qts, unsigned addr)
{
    qtest_writel(qts, I2C0 + ENB, 0);
    qtest_writel(qts, I2C0 + TAR, addr);
    qtest_writel(qts, I2C0 + ENB, 1);
}

/* One register, as sinfo reads it: the address bytes, then a read */
static uint32_t read_reg(QTestState *qts, unsigned bytes, unsigned reg)
{
    qtest_readl(qts, I2C0 + CTXABRT);
    if (bytes == 2) {
        qtest_writel(qts, I2C0 + DC, reg >> 8);
    }
    qtest_writel(qts, I2C0 + DC, reg & 0xff);
    qtest_writel(qts, I2C0 + DC, CMD_READ | CMD_STOP);
    g_assert_cmpuint(qtest_readl(qts, I2C0 + RXFLR), ==, 1);
    return qtest_readl(qts, I2C0 + DC) & 0xff;
}

static void test_mis2008(void)
{
    QTestState *qts = start("mis2008");

    select_target(qts, 0x30);
    g_assert_cmphex(read_reg(qts, 2, 0x3000), ==, 0x20);
    g_assert_cmphex(read_reg(qts, 2, 0x3001), ==, 0x08);
    qtest_quit(qts);
}

static void test_os02g10(void)
{
    QTestState *qts = start("os02g10");

    select_target(qts, 0x3c);
    qtest_writel(qts, I2C0 + DC, 0xfd);
    qtest_writel(qts, I2C0 + DC, CMD_STOP);
    g_assert_cmphex(read_reg(qts, 1, 0x02), ==, 0x56);
    g_assert_cmphex(read_reg(qts, 1, 0x03), ==, 0x02);
    g_assert_cmphex(read_reg(qts, 1, 0x04), ==, 0x47);
    qtest_quit(qts);
}

/* Nothing answers at the other sensor's address */
static void test_noack(const char *model, unsigned other)
{
    QTestState *qts = start(model);

    select_target(qts, other);
    qtest_readl(qts, I2C0 + CTXABRT);
    qtest_writel(qts, I2C0 + DC, 0x02);
    qtest_writel(qts, I2C0 + DC, CMD_READ | CMD_STOP);
    g_assert_cmphex(qtest_readl(qts, I2C0 + RAWINT) & INT_TX_ABRT, ==,
                    INT_TX_ABRT);
    g_assert_cmphex(qtest_readl(qts, I2C0 + TXABRT) & ABRT_7B_NOACK, ==,
                    ABRT_7B_NOACK);
    qtest_quit(qts);
}

static void test_mis2008_noack(void)
{
    test_noack("mis2008", 0x3c);
}

static void test_os02g10_noack(void)
{
    test_noack("os02g10", 0x30);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-sensor-id/mis2008", test_mis2008);
    qtest_add_func("ingenic-sensor-id/os02g10", test_os02g10);
    qtest_add_func("ingenic-sensor-id/mis2008-noack", test_mis2008_noack);
    qtest_add_func("ingenic-sensor-id/os02g10-noack", test_os02g10_noack);
    return g_test_run();
}
