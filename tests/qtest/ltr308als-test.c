/*
 * QTest testcase for the LTR-308ALS ambient light sensor
 *
 * Driven over a T31's I2C0 controller. The register map, reset values
 * and conversion times follow the LTR-F216A datasheet (the LTR-308ALS-01
 * could not be read); the lux factor 0.45 is that datasheet's.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define DEV             "/machine/peripheral/als"
#define I2C0            0x10050000
#define TAR             (I2C0 + 0x04)
#define DC              (I2C0 + 0x10)
#define ENB             (I2C0 + 0x6c)

#define MS              1000000LL

static void qom_set(QTestState *qts, const char *prop, double value)
{
    QDict *rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                           "{'path': %s, 'property': %s, 'value': %f}}",
                           DEV, prop, value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

static void qom_set_int(QTestState *qts, const char *prop, int64_t value)
{
    QDict *rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                           "{'path': %s, 'property': %s, 'value': %" PRId64
                           "}}", DEV, prop, value);

    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
}

static QTestState *init(const char *extra)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 -rtc clock=vm "
                                  "-global ingenic-t31.soc-variant=t31x "
                                  "-device ltr308als,bus=i2c0,id=als%s",
                                  extra);

    qtest_writel(qts, ENB, 0);
    qtest_writel(qts, TAR, 0x53);
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

/* ALS_DATA as the stock firmware reads it: three single-byte reads */
static uint32_t data(QTestState *qts)
{
    return rd8(qts, 0x0d) | rd8(qts, 0x0e) << 8 | rd8(qts, 0x0f) << 16;
}

static void test_reset(void)
{
    static const struct { uint8_t reg, v; } regs[] = {
        { 0x00, 0x00 }, { 0x04, 0x22 }, { 0x05, 0x01 }, { 0x06, 0xb1 },
        { 0x0d, 0 }, { 0x0e, 0 }, { 0x0f, 0 }, { 0x19, 0x10 }, { 0x1a, 0 },
        { 0x21, 0xff }, { 0x22, 0xff }, { 0x23, 0x0f },
        { 0x24, 0 }, { 0x25, 0 }, { 0x26, 0 },
    };
    QTestState *qts = init("");

    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        g_test_message("0x%02x", regs[i].reg);
        g_assert_cmphex(rd8(qts, regs[i].reg), ==, regs[i].v);
    }
    /* the power-on flag reads once */
    g_assert_cmphex(rd8(qts, 0x07), ==, 0x20);
    g_assert_cmphex(rd8(qts, 0x07), ==, 0);
    qtest_quit(qts);
}

static void test_standby_reads_zero(void)
{
    QTestState *qts = init("");

    qom_set_int(qts, "counts", 1234);
    qtest_clock_step(qts, 2000 * MS);
    g_assert_cmpuint(data(qts), ==, 0);
    qtest_quit(qts);
}

static void test_counts_and_lux(void)
{
    QTestState *qts = init("");

    qom_set_int(qts, "counts", 300001);
    wr(qts, 0x00, 0x02);
    /* 18 bits, 100 ms: nothing before the start and a conversion */
    qtest_clock_step(qts, 50 * MS);
    g_assert_cmphex(rd8(qts, 0x07) & 8, ==, 0);
    qtest_clock_step(qts, 100 * MS);
    g_assert_cmphex(rd8(qts, 0x07) & 8, ==, 8);
    g_assert_cmphex(rd8(qts, 0x07) & 8, ==, 0);
    /* 18 bits saturate at 262143 */
    g_assert_cmpuint(data(qts), ==, 262143);

    qom_set_int(qts, "counts", -1);
    qom_set(qts, "lux", 30);
    qtest_clock_step(qts, 200 * MS);
    /* 0.45 counts: lux * 3 (gain) * 1 (18 bit) / 0.45 = 200 */
    g_assert_cmpuint(data(qts), ==, 200);

    wr(qts, 0x05, 0x04);            /* 18x */
    wr(qts, 0x04, 0x00);            /* 20 bit, 400 ms, 25 ms rate */
    qtest_clock_step(qts, 500 * MS);
    /* 30 * 18 * 4 / 0.45 = 4800 */
    g_assert_cmpuint(data(qts), ==, 4800);
    qtest_quit(qts);
}

static void test_active_property_and_reset(void)
{
    QTestState *qts = init(",active=on,counts=777");

    g_assert_cmphex(rd8(qts, 0x00), ==, 2);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmpuint(data(qts), ==, 777);
    wr(qts, 0x00, 0x10);
    g_assert_cmphex(rd8(qts, 0x00), ==, 0);
    g_assert_cmpuint(data(qts), ==, 0);
    qtest_quit(qts);
}

static void test_interrupt(void)
{
    QTestState *qts = init(",active=on,counts=500");

    wr(qts, 0x24, 100);             /* lower 100, upper 0xfffff */
    wr(qts, 0x1a, 0x10);            /* two in a row */
    wr(qts, 0x19, 0x14);
    qtest_clock_step(qts, 300 * MS);
    rd8(qts, 0x07);
    g_assert_cmphex(rd8(qts, 0x07) & 0x10, ==, 0);
    qom_set_int(qts, "counts", 50);
    qtest_clock_step(qts, 110 * MS);
    rd8(qts, 0x07);
    qtest_clock_step(qts, 110 * MS);
    g_assert_cmphex(rd8(qts, 0x07) & 0x10, ==, 0x10);
    g_assert_cmphex(rd8(qts, 0x07) & 0x10, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ltr308als/reset", test_reset);
    qtest_add_func("ltr308als/standby", test_standby_reads_zero);
    qtest_add_func("ltr308als/counts-lux", test_counts_and_lux);
    qtest_add_func("ltr308als/active", test_active_property_and_reset);
    qtest_add_func("ltr308als/interrupt", test_interrupt);
    return g_test_run();
}
