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
/* GC4653 and GC4663: column gain 0x02b8:0x02b9 in 1/64ths, ref 1100 */
static const Case gc4653_it = {
    "gc4653-it", "gc4653", 0x29, 2, { 0, 0 },
    { { 0x0202, 0x00 }, { 0x0203, 0xfa }, { 0x02b8, 0x01 },
      { 0x02b9, 0x00 } },
    { { 0x0202, 0x01 }, { 0x0203, 0xf4 } },
    250.0 / 1100, 500.0 / 1100,
};
static const Case gc4663_gain = {
    "gc4663-gain", "gc4663", 0x29, 2, { 0, 0 },
    { { 0x0202, 0x00 }, { 0x0203, 0x96 }, { 0x02b8, 0x01 },
      { 0x02b9, 0x00 } },
    { { 0x02b8, 0x02 } },
    150.0 / 1100, 300.0 / 1100,
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
/*
 * OV2735B: page 1 (0xfd), lines at 0x03:0x04, gain 0x24 in 1/16ths, ref
 * 600; a write of 1 to 0x01 is the update that lets the page's changes land
 */
static const Case ov2735b_gain = {
    "ov2735b-gain", "ov2735b", 0x3c, 1, { 0, 0 },
    { { 0xfd, 0x01 }, { 0x03, 0x00 }, { 0x04, 0x96 }, { 0x24, 0x10 },
      { 0x01, 0x01 } },
    { { 0x24, 0x20 }, { 0x01, 0x01 } },
    150.0 / 600, 300.0 / 600,
};
static const Case ov2735b_it = {
    "ov2735b-it", "ov2735b", 0x3c, 1, { 0, 0 },
    { { 0xfd, 0x01 }, { 0x03, 0x00 }, { 0x04, 0xfa }, { 0x24, 0x10 },
      { 0x01, 0x01 } },
    { { 0x03, 0x01 }, { 0x04, 0xf4 }, { 0x01, 0x01 } },
    250.0 / 600, 500.0 / 600,
};

/* SC3336, SC3338: lines from 0x3e00:0x3e01:0x3e02, 16 to the line, ref 735 */
static const Case sc3336_it = {
    "sc3336-it", "sc3336", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x0c }, { 0x3e02, 0x80 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e01, 0x12 }, { 0x3e02, 0xc0 } },
    200.0 / 735, 300.0 / 735,
};
/* the analog code 0x40 is 1.52x, the coarse digital gain 0x01 doubles it */
static const Case sc3338_gain = {
    "sc3338-gain", "sc3338", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x0c }, { 0x3e02, 0x80 }, { 0x3e09, 0x40 },
      { 0x3e07, 0x80 } },
    { { 0x3e06, 0x01 } },
    200.0 * 1556 / 1024 / 735, 400.0 * 1556 / 1024 / 735,
};
/* SC4336 and SC4336P: the SC2336's gain codes, ref 675 */
static const Case sc4336_gain = {
    "sc4336-gain", "sc4336p", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x09 }, { 0x3e02, 0x60 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e09, 0x09 } },
    150.0 / 675, 600.0 / 675,
};
/* SC1A4T: ref 1350, the fine gain 0xc0 is 1.5x */
static const Case sc1a4t_gain = {
    "sc1a4t-gain", "sc1a4t", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x25 }, { 0x3e02, 0x80 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e07, 0xc0 } },
    600.0 / 1350, 900.0 / 1350,
};
/* SC2235: coarse 0x3e08 bits 4:2, fine 0x3e09 at 0x10 per 1x, ref 650 */
static const Case sc2235_gain = {
    "sc2235-gain", "sc2235", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x0c }, { 0x3e02, 0x80 }, { 0x3e08, 0x03 },
      { 0x3e09, 0x10 } },
    { { 0x3e08, 0x07 }, { 0x3e09, 0x18 } },
    200.0 / 650, 600.0 / 650,
};
/* SC2332 and SC3235: fine at 0x20 per 1x, ref 700 */
static const Case sc2332_gain = {
    "sc2332-gain", "sc2332", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x09 }, { 0x3e02, 0x60 }, { 0x3e08, 0x03 },
      { 0x3e09, 0x20 } },
    { { 0x3e08, 0x07 } },
    150.0 / 700, 300.0 / 700,
};
/* SC3335: fine at 0x40 per 1x, ref 735 */
static const Case sc3335_gain = {
    "sc3335-gain", "sc3335", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x09 }, { 0x3e02, 0x60 }, { 0x3e08, 0x03 },
      { 0x3e09, 0x40 } },
    { { 0x3e08, 0x07 }, { 0x3e09, 0x60 } },
    150.0 / 735, 450.0 / 735,
};
/* JXQ03: the SOI's, ref 720 */
static const Case jxq03_gain = {
    "jxq03-gain", "jxq03", 0x40, 1, { 0, 0 },
    { { 0x01, 0x96 }, { 0x02, 0x00 }, { 0x00, 0x00 } },
    { { 0x00, 0x10 } },
    150.0 / 720, 300.0 / 720,
};
/* JXH62: ref 405 */
static const Case jxh62_gain = {
    "jxh62-gain", "jxh62", 0x30, 1, { 0, 0 },
    { { 0x01, 0x96 }, { 0x02, 0x00 }, { 0x00, 0x00 } },
    { { 0x00, 0x10 } },
    150.0 / 405, 300.0 / 405,
};
/* GC4023: the driver's table by 0x0614, 0x0615, 0x00b8 and 0x00b9, ref 675 */
static const Case gc4023_gain = {
    "gc4023-gain", "gc4023", 0x29, 2, { 0, 0 },
    { { 0x0202, 0x00 }, { 0x0203, 0x96 }, { 0x00b8, 0x01 },
      { 0x00b9, 0x00 } },
    { { 0x0614, 0x02 }, { 0x00b8, 0x02 } },
    150.0 / 675, 300.0 / 675,
};
/* IMX307: lines are VMAX 0x3019:0x3018 less SHS1; 0x14 at 0x3014 is 6 dB */
static const Case imx307_gain = {
    "imx307-gain", "imx307", 0x1a, 2, { 0x3000, 0 },
    { { 0x3018, 0x65 }, { 0x3019, 0x04 }, { 0x3020, 0x9d },
      { 0x3021, 0x03 }, { 0x3014, 0x00 } },
    { { 0x3014, 0x14 } },
    200.0 / 500, 200.0 * 1.9953 / 500,
};

/* SC2331: the SC2336's gain codes at 0x3e08, half lines, ref 510 */
static const Case sc2331_gain = {
    "sc2331-gain", "sc2331", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x0c }, { 0x3e02, 0x80 }, { 0x3e08, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e08, 0x09 } },
    100.0 / 510, 4 * 100.0 / 510,
};

/* OS02B10: the OS02G10's page 1 registers, ref 610 */
static const Case os02b10_gain = {
    "os02b10-gain", "os02b10", 0x3c, 1, { 0, 0 },
    { { 0xfd, 0x01 }, { 0x03, 0x00 }, { 0x04, 0x96 }, { 0x24, 0x10 },
      { 0x01, 0x01 } },
    { { 0x24, 0x20 }, { 0x01, 0x01 } },
    150.0 / 610, 300.0 / 610,
};

/* SC3332: lines as the SC3336's, the SC2336's gain codes, ref 1225 */
static const Case sc3332_gain = {
    "sc3332-gain", "sc3332", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x12 }, { 0x3e02, 0xc0 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e09, 0x08 }, { 0x3e07, 0xc0 } },
    300.0 / 1225, 3 * 300.0 / 1225,
};


/* SC2300: 0x23 at 0x3e08 is 2.4x, the fine gain 0x30 is 1.5x, ref 1800 */
static const Case sc2300_gain = {
    "sc2300-gain", "sc2300", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x19 }, { 0x3e02, 0x00 }, { 0x3e08, 0x03 },
      { 0x3e09, 0x20 } },
    { { 0x3e08, 0x23 }, { 0x3e09, 0x30 } },
    400.0 / 1800, 400.0 * 2.4 * 1.5 / 1800,
};

/* JXQ03P: the JXQ03's table, ref 720 */
static const Case jxq03p_gain = {
    "jxq03p-gain", "jxq03p", 0x40, 1, { 0, 0 },
    { { 0x01, 0x96 }, { 0x02, 0x00 }, { 0x00, 0x00 } },
    { { 0x00, 0x10 } },
    150.0 / 720, 300.0 / 720,
};

/*
 * CV2003: lines are the frame length 0x3021:0x3020 less the shutter at
 * 0x304a:0x3049:0x3048, the gain 256 / (256 - 0x3154), ref 2325
 */
static const Case cv2003_gain = {
    "cv2003-gain", "cv2003", 0x35, 2, { 0, 0 },
    { { 0x3021, 0x14 }, { 0x3020, 0x29 }, { 0x3049, 0x11 },
      { 0x3048, 0xd1 }, { 0x3154, 0x00 } },
    { { 0x3154, 0x80 } },
    600.0 / 2325, 1200.0 / 2325,
};

/* SC5336: lines as the SC3336's, the SC2336's gain codes, ref 810 */
static const Case sc5336_gain = {
    "sc5336-gain", "sc5336", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x06 }, { 0x3e02, 0x40 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e09, 0x09 }, { 0x3e07, 0xc0 } },
    100.0 / 810, 6 * 100.0 / 810,
};

/* SC5235: half lines, the SC3235's gain codes, ref 1200 */
static const Case sc5235_gain = {
    "sc5235-gain", "sc5235", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x12 }, { 0x3e02, 0xc0 }, { 0x3e08, 0x03 },
      { 0x3e09, 0x20 } },
    { { 0x3e08, 0x0f }, { 0x3e09, 0x30 } },
    150.0 / 1200, 6 * 150.0 / 1200,
};

/* SC301IOT: its own analog codes, 0x48 is 3.137x, ref 865 */
static const Case sc301iot_gain = {
    "sc301iot-gain", "sc301iot", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x06 }, { 0x3e02, 0x40 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e09, 0x48 }, { 0x3e07, 0xc0 } },
    100.0 / 865, 100.0 * 3.137 * 1.5 / 865,
};

/* SC223A: half lines, 0x40 is 1.81x, ref 650 */
static const Case sc223a_gain = {
    "sc223a-gain", "sc223a", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x12 }, { 0x3e02, 0xc0 }, { 0x3e09, 0x00 },
      { 0x3e07, 0x80 } },
    { { 0x3e09, 0x40 }, { 0x3e07, 0xc0 } },
    150.0 / 650, 150.0 * 1.8096 * 1.5 / 650,
};

/* SC1235: the gain code at 0x3e09 is in 1/16ths, ref 540 */
static const Case sc1235_gain = {
    "sc1235-gain", "sc1235", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x06 }, { 0x3e02, 0x40 }, { 0x3e08, 0x00 },
      { 0x3e09, 0x10 } },
    { { 0x3e09, 0x30 } },
    100.0 / 540, 300.0 / 540,
};

/* JXH63P: the SOI's, ref 1350 */
static const Case jxh63p_gain = {
    "jxh63p-gain", "jxh63p", 0x40, 1, { 0, 0 },
    { { 0x01, 0x2c }, { 0x02, 0x01 }, { 0x00, 0x00 } },
    { { 0x00, 0x10 } },
    300.0 / 1350, 600.0 / 1350,
};

/* IMX327: the IMX307's registers */
static const Case imx327_gain = {
    "imx327-gain", "imx327", 0x1a, 2, { 0x3000, 0 },
    { { 0x3018, 0x65 }, { 0x3019, 0x04 }, { 0x3020, 0x9d },
      { 0x3021, 0x03 }, { 0x3014, 0x00 } },
    { { 0x3014, 0x14 } },
    200.0 / 500, 200.0 * 1.9953 / 500,
};

/* OS03B10: the OS02B10's page 1 registers, ref 715 */
static const Case os03b10_gain = {
    "os03b10-gain", "os03b10", 0x3c, 1, { 0, 0 },
    { { 0xfd, 0x01 }, { 0x03, 0x01 }, { 0x04, 0x2c }, { 0x24, 0x10 },
      { 0x01, 0x01 } },
    { { 0x24, 0x20 }, { 0x01, 0x01 } },
    300.0 / 715, 600.0 / 715,
};

/* MIS20C1: lines at 0x3100, the gain code 0x200 is 2x, ref 505 */
static const Case mis20c1_gain = {
    "mis20c1-gain", "mis20c1", 0x30, 2, { 0, 0 },
    { { 0x3100, 0x00 }, { 0x3101, 0x96 }, { 0x3102, 0x00 },
      { 0x3103, 0x00 }, { 0x3008, 0x01 } },
    { { 0x3102, 0x02 }, { 0x3008, 0x01 } },
    150.0 / 505, 300.0 / 505,
};

/*
 * PS5250: page 1; lines are the driver's 1349 less the offset at
 * 0x0c:0x0d, the gain 0x10 at 0x83 is 2x, ref 600
 */
static const Case ps5250_gain = {
    "ps5250-gain", "ps5250", 0x48, 1, { 0, 0 },
    { { 0xef, 0x01 }, { 0x0c, 0x04 }, { 0x0d, 0xaf }, { 0x83, 0x00 },
      { 0x09, 0x01 } },
    { { 0x83, 0x10 }, { 0x09, 0x01 } },
    150.0 / 600, 300.0 / 600,
};

/*
 * PS5260: lines are the driver's 1124 less the offset at 0x0c:0x0d; clearing
 * bit 0 of 0x18 (the high gain mode) doubles the gain, ref 505
 */
static const Case ps5260_gain = {
    "ps5260-gain", "ps5260", 0x48, 1, { 0, 0 },
    { { 0xef, 0x01 }, { 0x0c, 0x03 }, { 0x0d, 0xce }, { 0x18, 0x01 },
      { 0x83, 0x00 }, { 0x09, 0x01 } },
    { { 0x18, 0x00 }, { 0x09, 0x01 } },
    150.0 / 505, 300.0 / 505,
};

/* JXF23: the SOI's, 0x23 at 0x00 is 4.75x, ref 600 */
static const Case jxf23_gain = {
    "jxf23-gain", "jxf23", 0x40, 1, { 0, 0 },
    { { 0x01, 0x3c }, { 0x02, 0x00 }, { 0x00, 0x00 } },
    { { 0x00, 0x23 } },
    60.0 / 600, 60.0 * 4.75 / 600,
};

/* SC401AI: half lines, the code 0x27 at 0x3e08 is 2.94x, ref 810 */
static const Case sc401ai_gain = {
    "sc401ai-gain", "sc401ai", 0x30, 2, { 0x0100, 1 },
    { { 0x3e01, 0x12 }, { 0x3e02, 0xc0 }, { 0x3e08, 0x03 },
      { 0x3e09, 0x40 } },
    { { 0x3e08, 0x27 }, { 0x3e09, 0x60 } },
    150.0 / 810, 150.0 * 2.9375 * 1.5 / 810,
};

int main(int argc, char **argv)
{
    static const Case *cases[] = {
        &sc2336_it, &sc2336_coarse, &sc2336_fine, &gc2083_it, &gc2083_gain,
        &gc4653_it, &gc4663_gain, &jxf37_gain, &jxf37_it, &ov2735b_gain,
        &ov2735b_it, &sc3336_it, &sc3338_gain, &sc4336_gain, &sc1a4t_gain,
        &sc2235_gain, &sc2332_gain, &sc3335_gain, &jxq03_gain, &jxh62_gain,
        &gc4023_gain, &imx307_gain,
        &sc2331_gain,
        &os02b10_gain,
        &sc3332_gain,
        &sc2300_gain,
        &jxq03p_gain,
        &cv2003_gain,
        &sc5336_gain,
        &sc5235_gain,
        &sc301iot_gain,
        &sc223a_gain,
        &sc1235_gain,
        &jxh63p_gain,
        &imx327_gain,
        &os03b10_gain,
        &mis20c1_gain,
        &ps5250_gain,
        &ps5260_gain,
        &sc401ai_gain,
        &jxf23_gain,
    };

    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-sensor-exposure/%s",
                                                cases[i]->name);

        qtest_add_data_func(path, cases[i], test_case);
    }
    return g_test_run();
}
