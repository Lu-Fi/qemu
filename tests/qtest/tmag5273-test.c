/*
 * QTest testcase for the TMAG5273 3D Hall-effect sensor
 *
 * Driven over a T31's I2C0 controller. IDs, reset values and the field
 * and temperature codes follow the TMAG5273 datasheet (SLYS045C); the
 * angle's direction and the magnitude's scale are the model's assumption.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define DEV             "/machine/peripheral/hall"
#define I2C0            0x10050000
#define TAR             (I2C0 + 0x04)
#define DC              (I2C0 + 0x10)
#define ENB             (I2C0 + 0x6c)
#define TXABRT          (I2C0 + 0x80)
#define CTXABRT         (I2C0 + 0x54)

static void qom_set(QTestState *qts, const char *prop, double value)
{
    QDict *rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                           "{'path': %s, 'property': %s, 'value': %f}}",
                           DEV, prop, value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

static QTestState *init(const char *extra)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-t31.soc-variant=t31x "
                                  "-device tmag5273,bus=i2c0,id=hall%s",
                                  extra);

    qtest_writel(qts, ENB, 0);
    qtest_writel(qts, TAR, 0x35);
    qtest_writel(qts, ENB, 1);
    return qts;
}

static void wr(QTestState *qts, uint8_t reg, uint8_t v)
{
    qtest_writel(qts, DC, reg);
    qtest_writel(qts, DC, v | 0x200);
}

static void rd(QTestState *qts, uint8_t reg, uint8_t *b, int n)
{
    qtest_writel(qts, DC, reg);
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, DC, 0x100 | (i == n - 1 ? 0x200 : 0));
    }
    for (int i = 0; i < n; i++) {
        b[i] = qtest_readl(qts, DC);
    }
}

static uint8_t rd8(QTestState *qts, uint8_t reg)
{
    uint8_t b;

    rd(qts, reg, &b, 1);
    return b;
}

static uint16_t rd16(QTestState *qts, uint8_t reg)
{
    uint8_t b[2];

    rd(qts, reg, b, 2);
    return b[0] << 8 | b[1];
}

static void test_ids_and_reset(void)
{
    static const uint8_t zero[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xa, 0xb };
    QTestState *qts = init("");

    for (int i = 0; i < sizeof(zero); i++) {
        g_assert_cmphex(rd8(qts, zero[i]), ==, 0);
    }
    g_assert_cmphex(rd8(qts, 0x0c), ==, 0x6a);
    g_assert_cmphex(rd8(qts, 0x0d), ==, 1);
    g_assert_cmphex(rd16(qts, 0x0e) , ==, 0x4954);
    g_assert_cmphex(rd8(qts, 0x18), ==, 0x10);
    g_assert_cmphex(rd8(qts, 0x1c), ==, 0x10);
    g_assert_cmphex(rd16(qts, 0x10), ==, 0);
    g_assert_cmphex(rd16(qts, 0x19), ==, 0);
    qtest_quit(qts);

    qts = init(",version=2");
    g_assert_cmphex(rd8(qts, 0x0d), ==, 2);
    qtest_quit(qts);
}

static void test_nack_elsewhere(void)
{
    QTestState *qts = init("");

    qtest_writel(qts, ENB, 0);
    qtest_writel(qts, TAR, 0x36);
    qtest_writel(qts, ENB, 1);
    qtest_writel(qts, DC, 0x0d | 0x200);
    g_assert_cmphex(qtest_readl(qts, TXABRT) & 1, ==, 1);
    qtest_quit(qts);
}

static void test_poll_and_standby(void)
{
    QTestState *qts = init("");

    /* standby: no conversion until a trigger, then X, Y, T as set */
    wr(qts, 0x02, 0x30);
    wr(qts, 0x07, 0x01);
    qom_set(qts, "x", 10);        /* 10 mT of +-40: 8192 */
    qom_set(qts, "y", -20);
    g_assert_cmphex(rd16(qts, 0x12), ==, 0);
    qtest_writel(qts, DC, 0x80 | 0x18);
    qtest_writel(qts, DC, 0x100 | 0x200);
    qtest_readl(qts, DC);
    g_assert_cmphex(rd16(qts, 0x12), ==, 8192);
    g_assert_cmphex(rd16(qts, 0x14), ==, 0xc000);
    g_assert_cmphex(rd16(qts, 0x16), ==, 0);
    g_assert_cmphex(rd16(qts, 0x10), ==, 17508);
    g_assert_cmphex(rd8(qts, 0x18) & 0x11, ==, 0x11);
    wr(qts, 0x18, 0x10);
    g_assert_cmphex(rd8(qts, 0x18) & 0x10, ==, 0);
    qtest_quit(qts);
}

static void test_continuous(void)
{
    QTestState *qts = init("");

    /* what the stock T86P2 driver writes */
    wr(qts, 0x00, 0x34);
    wr(qts, 0x01, 0x12);
    wr(qts, 0x02, 0x30);
    wr(qts, 0x03, 0x07);
    wr(qts, 0x07, 0x01);
    wr(qts, 0x08, 0x01);
    qom_set(qts, "temperature", 50);
    qom_set(qts, "x", 0);
    qom_set(qts, "y", 40);
    /* 80 mT range: Y 40 mT = 16384; atan2(y, x) = 90 degrees */
    g_assert_cmphex(rd16(qts, 0x14), ==, 16384);
    g_assert_cmpuint(rd16(qts, 0x19), ==, 90 * 16);
    qom_set(qts, "x", -40);
    qom_set(qts, "y", 0);
    g_assert_cmpuint(rd16(qts, 0x19), ==, 180 * 16);
    qom_set(qts, "y", -40);
    g_assert_cmpuint(rd16(qts, 0x19), >, 180 * 16);
    g_assert_cmpuint(rd16(qts, 0x19) >> 4, ==, 225);
    qom_set(qts, "x", 40);
    g_assert_cmpuint(rd16(qts, 0x19) >> 4, ==, 315);
    /* 50 C: 17508 + 25 * 58 */
    g_assert_cmphex(rd16(qts, 0x10), ==, 17508 + 25 * 58);
    g_assert_cmphex(rd8(qts, 0x1b), >, 0);
    /* the data set count rolls on */
    g_assert_cmphex(rd8(qts, 0x18) >> 5, !=, 0);
    qtest_quit(qts);
}

static void test_sleep_and_address(void)
{
    QTestState *qts = init("");

    wr(qts, 0x01, 0x02);
    wr(qts, 0x02, 0x10);
    qom_set(qts, "x", 20);
    g_assert_cmphex(rd16(qts, 0x12), ==, 16384);
    wr(qts, 0x01, 0x01);
    /* sleep drops the results, the next access wakes the part */
    g_assert_cmphex(rd8(qts, 0x01), ==, 0);
    g_assert_cmphex(rd16(qts, 0x12), ==, 0);
    g_assert_cmphex(rd8(qts, 0x18) & 1, ==, 0);

    wr(qts, 0x0c, 0x40 << 1 | 1);
    qtest_writel(qts, ENB, 0);
    qtest_writel(qts, TAR, 0x40);
    qtest_writel(qts, ENB, 1);
    g_assert_cmphex(rd8(qts, 0x0d), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("tmag5273/ids-reset", test_ids_and_reset);
    qtest_add_func("tmag5273/nack", test_nack_elsewhere);
    qtest_add_func("tmag5273/standby", test_poll_and_standby);
    qtest_add_func("tmag5273/continuous", test_continuous);
    qtest_add_func("tmag5273/sleep-address", test_sleep_and_address);
    return g_test_run();
}
