/*
 * QTest testcase for the Ingenic I2C's power-on values
 *
 * Every I2C of a T20X, T21N, T31X, T32LQ and T41LQ reads these before a
 * driver touches it: the DesignWare core's SCL counts, a 10-bit target
 * address flag, and the FIFO thresholds, SDA timing, ACK of general calls
 * and spike filter as Ingenic sets them.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2C(n)          (0x10050000 + (n) * 0x1000)

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    unsigned n;                 /* its I2C controllers */
} I2cCase;

static const I2cCase cases[] = {
    { "t20x", "ingenic-t20", "ingenic-t31", 2 },
    { "t21n", "ingenic-t21", "ingenic-t31", 2 },
    { "t31x", "ingenic-t31", "ingenic-t31", 2 },
    { "t32lq", "ingenic-t32", "ingenic-t31", 3 },
    { "t41lq", "ingenic-t41", "ingenic-t40", 3 },
};

static const struct {
    uint32_t offset, value;
} power_on[] = {
    { 0x00, 0x7d },     /* CTRL */
    { 0x04, 0x1055 },   /* TAR */
    { 0x14, 0x190 },    /* SHCNT */
    { 0x18, 0x1d6 },    /* SLCNT */
    { 0x1c, 0x3c },     /* FHCNT */
    { 0x20, 0x82 },     /* FLCNT */
    { 0x38, 0x1f },     /* RXTL */
    { 0x3c, 0x20 },     /* TXTL */
    { 0x7c, 0x55 },     /* SDAHD */
    { 0x94, 0x64 },     /* SDASU */
    { 0x98, 0x1 },      /* ACKGC */
    { 0xa0, 0x1 },      /* FLT */
};

static void test_case(const void *data)
{
    const I2cCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    for (unsigned n = 0; n < c->n; n++) {
        for (int i = 0; i < ARRAY_SIZE(power_on); i++) {
            g_test_message("%s i2c%u 0x%02x", c->part, n, power_on[i].offset);
            g_assert_cmphex(qtest_readl(qts, I2C(n) + power_on[i].offset),
                            ==, power_on[i].value);
        }
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-i2c/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
