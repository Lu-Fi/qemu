/*
 * QTest testcase for the image sensors' exposure tables: the luma a T20's
 * ISP puts out for the scene's white bar follows the integration time and
 * gain a driver writes to the sensor's registers, as each vendor driver
 * encodes them (SC2336's coarse and fine gain, GC2083's gain table, GC4653's
 * column gain, JXF37's, OV2735B's page 1), against the table's reference
 * exposure: 180 at 1.0, the exposure's 1 / 2.2 power below it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define I2C0            0x10050000
#define T20_CORE        (ISP + 0x80000)
#define T20_WR          (T20_CORE + 0xb00)
#define T20_WR_UV       (T20_WR + 0x80)
#define BUF             0x00400000
#define W               64
#define H               64
#define FRAME_NS        40000000        /* fps=25 */

typedef struct Reg {
    uint16_t reg;
    uint8_t val;
} Reg;

typedef struct Case {
    const char *name;
    const char *model;
    uint8_t addr;
    int reg_bytes;
    Reg stream;                 /* what starts its frames, if anything */
    /* two register sets and the exposure each should give, against the ref */
    Reg a[6], b[6];
    double ea, eb;
} Case;

static QTestState *start(const Case *c)
{
    QTestState *qts = qtest_initf("-machine ingenic-t20 -device image-sensor,"
                                  "bus=i2c0,fps=25,motion=off,model=%s",
                                  c->model);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, W);
    qtest_writel(qts, T20_CORE + 0x14, H);
    qtest_writel(qts, T20_WR, 13 | 1 << 9);             /* NV12, on */
    qtest_writel(qts, T20_WR + 4, H << 16 | W);
    qtest_writel(qts, T20_WR + 8, BUF);
    qtest_writel(qts, T20_WR + 0x20, W);
    qtest_writel(qts, T20_WR + 0x24, 2);
    qtest_writel(qts, T20_WR_UV, 1 << 9);
    qtest_writel(qts, T20_WR_UV + 4, H / 2 << 16 | W);
    qtest_writel(qts, T20_WR_UV + 8, BUF + W * H);
    qtest_writel(qts, T20_WR_UV + 0x20, W);
    qtest_writel(qts, T20_WR_UV + 0x24, 2);
    qtest_writel(qts, ISP, 1);                          /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 2);
    return qts;
}

static void write_regs(QTestState *qts, const Case *c, const Reg *r, int n)
{
    qtest_writel(qts, I2C0 + 0x6c, 0);
    qtest_writel(qts, I2C0 + 0x04, c->addr);
    qtest_writel(qts, I2C0 + 0x6c, 1);
    for (int i = 0; i < n && (r[i].reg || r[i].val); i++) {
        if (c->reg_bytes == 2) {
            qtest_writel(qts, I2C0 + 0x10, r[i].reg >> 8);
        }
        qtest_writel(qts, I2C0 + 0x10, r[i].reg & 0xff);
        qtest_writel(qts, I2C0 + 0x10, 0x200 | r[i].val);
    }
    qtest_clock_step(qts, FRAME_NS * 3);
}

/* The white bar's luma in the next frame */
static int white(QTestState *qts)
{
    uint8_t y;

    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, BUF + 10 * W + 4, &y, 1);
    return y;
}

static void test_case(const void *data)
{
    const Case *c = data;
    QTestState *qts = start(c);
    int want_a = 16 + lround(164 * pow(c->ea, 1 / 2.2));
    int want_b = 16 + lround(164 * pow(c->eb, 1 / 2.2));
    int got_a, got_b;

    write_regs(qts, c, &c->stream, 1);
    write_regs(qts, c, c->a, 6);
    got_a = white(qts);
    write_regs(qts, c, c->b, 6);
    got_b = white(qts);
    g_test_message("%s: %d (%d), %d (%d)", c->name, got_a, want_a, got_b,
                   want_b);
    g_assert_cmpint(abs(got_a - want_a), <=, 1);
    g_assert_cmpint(abs(got_b - want_b), <=, 1);
    qtest_quit(qts);
}

/* SC2336: the half lines of 0x3e00:0x3e01:0x3e02 bits 7:4, ref 700 lines */
static const Case sc2336_it = {
    "sc2336-it", "sc2336", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x25 }, { 0x3e02, 0x80 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e01, 0x4b }, { 0x3e02, 0x00 } },
    300.0 / 700, 600.0 / 700,
};
/* the coarse code 0x09 is 4x */
static const Case sc2336_coarse = {
    "sc2336-coarse", "sc2336p", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x0c }, { 0x3e02, 0x80 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e09, 0x09 } },
    100.0 / 700, 4 * 100.0 / 700,
};
/* the fine gain 0xc0 is 1.5x */
static const Case sc2336_fine = {
    "sc2336-fine", "sc2336", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x0c }, { 0x3e02, 0x80 }, { 0x3e09, 0x08 },
      { 0x3e07, 0x80 } },
    { { 0x3e07, 0xc0 } },
    2 * 100.0 / 700, 3 * 100.0 / 700,
};
/* GC2083: lines at 0x0d03:0x0d04, ref 600; the gain table's rows */
static const Case gc2083_it = {
    "gc2083-it", "gc2083", 0x37, 2, { 0x023e, 0x99 },
    { { 0x0d03, 0x00 }, { 0x0d04, 0xfa }, { 0xd0, 0x00 }, { 0xb8, 0x01 },
      { 0xb9, 0x01 } },
    { { 0x0d03, 0x01 }, { 0x0d04, 0xf4 } },
    250.0 / 600 * (1 + 1 / 64.0), 500.0 / 600 * (1 + 1 / 64.0),
};
static const Case gc2083_gain = {
    "gc2083-gain", "gc2083", 0x37, 2, { 0x023e, 0x99 },
    { { 0x0d03, 0x00 }, { 0x0d04, 0x03 }, { 0xd0, 0x74 }, { 0xb8, 0x35 },
      { 0xb9, 0x17 } },
    { { 0xd0, 0x84 } },
    3 * 54639 / 1024.0 / 600, 3 * 64.0 / 600,
};
/* GC4653 and GC4663: column gain 0x02b8:0x02b9 in 1/64ths, ref 800 */
static const Case gc4653_it = {
    "gc4653-it", "gc4653", 0x29, 2, { 0, 0 },
    { { 0x0202, 0x00 }, { 0x0203, 0xfa }, { 0x02b8, 0x01 },
      { 0x02b9, 0x00 } },
    { { 0x0202, 0x01 }, { 0x0203, 0xf4 } },
    250.0 / 800, 500.0 / 800,
};
static const Case gc4663_gain = {
    "gc4663-gain", "gc4663", 0x29, 2, { 0, 0 },
    { { 0x0202, 0x00 }, { 0x0203, 0x96 }, { 0x02b8, 0x01 },
      { 0x02b9, 0x00 } },
    { { 0x02b8, 0x02 } },
    150.0 / 800, 300.0 / 800,
};
/* JXF37: gain 0x10 is 2x, lines at 0x02:0x01, ref 600 */
static const Case jxf37_gain = {
    "jxf37-gain", "jxf37", 0x40, 1, { 0, 0 },
    { { 0x01, 0x96 }, { 0x02, 0x00 }, { 0x00, 0x00 } },
    { { 0x00, 0x10 } },
    150.0 / 600, 300.0 / 600,
};
static const Case jxf37_it = {
    "jxf37-it", "jxf37", 0x40, 1, { 0, 0 },
    { { 0x01, 0xfa }, { 0x02, 0x00 }, { 0x00, 0x00 } },
    { { 0x01, 0xf4 }, { 0x02, 0x01 } },
    250.0 / 600, 500.0 / 600,
};
/* OV2735B: page 1 (0xfd), lines at 0x03:0x04, gain 0x24 in 1/16ths, ref 600 */
static const Case ov2735b_gain = {
    "ov2735b-gain", "ov2735b", 0x3c, 1, { 0, 0 },
    { { 0xfd, 0x01 }, { 0x03, 0x00 }, { 0x04, 0x96 }, { 0x24, 0x10 } },
    { { 0x24, 0x20 } },
    150.0 / 600, 300.0 / 600,
};
static const Case ov2735b_it = {
    "ov2735b-it", "ov2735b", 0x3c, 1, { 0, 0 },
    { { 0xfd, 0x01 }, { 0x03, 0x00 }, { 0x04, 0xfa }, { 0x24, 0x10 } },
    { { 0x03, 0x01 }, { 0x04, 0xf4 } },
    250.0 / 600, 500.0 / 600,
};

int main(int argc, char **argv)
{
    static const Case *cases[] = {
        &sc2336_it, &sc2336_coarse, &sc2336_fine, &gc2083_it, &gc2083_gain,
        &gc4653_it, &gc4663_gain, &jxf37_gain, &jxf37_it, &ov2735b_gain,
        &ov2735b_it,
    };

    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-sensor-exposure/%s",
                                                cases[i]->name);

        qtest_add_data_func(path, cases[i], test_case);
    }
    return g_test_run();
}
