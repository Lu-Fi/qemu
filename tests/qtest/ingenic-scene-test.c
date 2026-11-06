/*
 * QTest testcase for the image sensor's test scene, as a T31's ISP puts
 * it out on a 64x64 channel: the frame stamp, the moving square and the
 * stripe stood still, the light's colour temperature, the light and its
 * ramp, and IR through the IR-cut filter from the board's pins. The ISP's
 * white-balance gains (R 0x1804, B 0x1808) on what it puts out, as a T31's
 * AWB sets them: 5000 K's take the ramp back to grey.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/host-utils.h"
#include "qobject/qdict.h"

#define ISP             0x13300000
#define CH_EN           (ISP + 0x9804)
#define CH1             (ISP + 0x9a00)
#define VIC             (ISP + 0xe0000)
#define CPM_CLKGR0      0x10000020
#define GPIO_PB         0x10011000
#define WB_R            (ISP + 0x1804)
#define WB_B            (ISP + 0x1808)
#define FRAME_NS        40000000        /* fps=25 */
#define BUF             0x00400000
#define W               64
#define H               64

typedef struct Frame {
    uint8_t y[W * H], uv[W * H / 2];
} Frame;

static QTestState *start(const char *props)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 -device image-sensor,"
                                  "id=cam,model=jxq03,bus=i2c0,fps=25,%s",
                                  props);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CH_EN, 2);
    return qts;
}

static void push(QTestState *qts)
{
    qtest_writel(qts, CH1, W << 16 | H);
    qtest_writel(qts, CH1 + 0x80, W);
    qtest_writel(qts, CH1 + 0x98, W);
    qtest_writel(qts, CH1 + 0x6c, BUF);
    qtest_writel(qts, CH1 + 0x84, BUF + W * H);
}

/* The next frame channel 1 puts out: the first at the VIC's start */
static void frame(QTestState *qts, Frame *f, bool first)
{
    if (first) {
        push(qts);
        qtest_writel(qts, VIC, 1);
    } else {
        qtest_clock_step(qts, FRAME_NS);
    }
    g_assert_false(qtest_readl(qts, CH1 + 0x7c) & 1);
    qtest_readl(qts, CH1 + 0x74);
    qtest_memread(qts, BUF, f->y, sizeof(f->y));
    qtest_memread(qts, BUF + W * H, f->uv, sizeof(f->uv));
    push(qts);
}

static void check(QDict *r)
{
    g_assert_false(qdict_haskey(r, "error"));
    qobject_unref(r);
}

static void set(QTestState *qts, const char *prop, unsigned v)
{
    check(qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': { "
                    "'path': '/machine/peripheral/cam', 'property': %s, "
                    "'value': %u } }", prop, v));
}

static void set_flag(QTestState *qts, const char *prop, bool v)
{
    check(qtest_qmp(qts, "{ 'execute': 'qom-set', 'arguments': { "
                    "'path': '/machine/peripheral/cam', 'property': %s, "
                    "'value': %i } }", prop, v));
}

/* GPIO PB @pin an output at @level */
static void pin(QTestState *qts, int pin, bool level)
{
    qtest_writel(qts, GPIO_PB + 0x18, 1u << pin);       /* INT */
    qtest_writel(qts, GPIO_PB + 0x24, 1u << pin);       /* MSK */
    qtest_writel(qts, GPIO_PB + 0x38, 1u << pin);       /* PAT1 */
    qtest_writel(qts, GPIO_PB + (level ? 0x44 : 0x48), 1u << pin);
}

/* The stamp's 32 cells, 2 pixels each at this width */
static uint32_t stamp(const Frame *f, uint8_t *cell)
{
    uint32_t bits = 0;

    for (int i = 0; i < 32; i++) {
        cell[i] = f->y[2 * i] > 128;
    }
    for (int i = 2; i < 28; i++) {
        bits = bits << 1 | cell[i];
    }
    return bits;
}

static void test_stamp(void)
{
    QTestState *qts = start("stamp=on");
    uint8_t cell[32];
    uint32_t a, b;
    Frame f;

    frame(qts, &f, true);
    a = stamp(&f, cell);
    g_assert_cmpuint(cell[0], ==, 1);
    g_assert_cmpuint(cell[1], ==, 0);
    g_assert_cmpuint(cell[28], ==, ctpop32(a) & 1);
    g_assert_cmpuint(cell[29] << 2 | cell[30] << 1 | cell[31], ==, 5);
    g_assert_cmphex(a >> 24, ==, 0);
    frame(qts, &f, false);
    b = stamp(&f, cell);
    g_assert_cmphex(b, ==, a + 1);
    /* full contrast in the dark */
    set(qts, "light", 0);
    frame(qts, &f, false);
    g_assert_cmphex(stamp(&f, cell), ==, a + 2);
    g_assert_cmpuint(f.y[0], ==, 235);
    g_assert_cmpuint(f.y[2], ==, 16);
    qtest_quit(qts);
}

static void test_motion(void)
{
    QTestState *qts = start("object=on,motion=off");
    Frame a, b;

    frame(qts, &a, true);
    frame(qts, &b, false);
    g_assert_cmpmem(a.y, sizeof(a.y), b.y, sizeof(b.y));
    /* the square in the ramp's middle, white */
    g_assert_cmpuint(a.y[52 * W + 2], ==, 235);
    set_flag(qts, "motion", true);
    frame(qts, &b, false);
    frame(qts, &b, false);
    g_assert_cmpint(memcmp(a.y, b.y, sizeof(a.y)), !=, 0);
    qtest_quit(qts);
}

static void test_kelvin(void)
{
    QTestState *qts = start("kelvin=6500");
    const uint8_t *ramp_uv;
    Frame f;

    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    g_assert_cmpuint(ramp_uv[0], ==, 128);
    g_assert_cmpuint(ramp_uv[1], ==, 128);
    set(qts, "kelvin", 3000);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[0], <, 128);                /* less blue */
    g_assert_cmpuint(ramp_uv[1], >, 128);                /* more red */
    set(qts, "kelvin", 12000);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[0], >, 128);
    g_assert_cmpuint(ramp_uv[1], <, 128);
    qtest_quit(qts);
}

static void test_wb_gains(void)
{
    QTestState *qts = start("kelvin=5000");
    const uint8_t *ramp_uv;
    Frame f;

    /* not written yet: the scene as it comes */
    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    g_assert_cmpuint(ramp_uv[0], <, 125);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    /* the gains a qemu_t31x guest's AWB settles on in 5000 K */
    qtest_writel(qts, WB_R, 0x0400059c);
    qtest_writel(qts, WB_B, 0x04000c14);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);

    /* more red */
    qtest_writel(qts, WB_R, 0x04000800);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    /* and in 6500 K, the gains it settles on there leave grey grey */
    set(qts, "kelvin", 6500);
    qtest_writel(qts, WB_R, 0x04000728);
    qtest_writel(qts, WB_B, 0x040009dc);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    qtest_quit(qts);
}

static void test_light(void)
{
    QTestState *qts = start("light-secs=1,motion=off");
    uint8_t white;
    Frame f;

    frame(qts, &f, true);
    white = f.y[10 * W + 4];
    g_assert_cmpuint(white, ==, 180);
    set(qts, "light", 25);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], <, white);
    g_assert_cmpuint(f.y[10 * W + 4], >, 16);
    /* down to dark over a second of frames */
    set(qts, "light", 100);
    set(qts, "light-to", 0);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], <, white);
    for (int i = 0; i < 25; i++) {
        frame(qts, &f, false);
    }
    g_assert_cmpuint(f.y[10 * W + 4], ==, 16);
    qtest_quit(qts);
}

/* PB10 the filter's one pin (high: in), PB11 the IR LED */
static void test_ir_single(void)
{
    QTestState *qts = start("light=0,motion=off,ircut=42,irled=43");
    Frame f;

    pin(qts, 11, true);
    frame(qts, &f, true);
    g_assert_cmpuint(f.y[10 * W + 4], ==, 16);           /* filter in */
    pin(qts, 10, false);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], >, 100);           /* lit by IR */
    g_assert_cmpuint(f.uv[5 * W + 18], ==, 128);         /* no colour */
    g_assert_cmpuint(f.uv[5 * W + 19], ==, 128);
    pin(qts, 11, false);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], ==, 16);           /* LED off */
    pin(qts, 11, true);
    pin(qts, 10, true);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], ==, 16);
    qtest_quit(qts);
}

/* PB12 and PB13 a latching filter's pins, pulsed as thingino's ircut does */
static void test_ir_latch(void)
{
    QTestState *qts = start("light=0,motion=off,ircut=44,,45,irled=43");
    Frame f;

    pin(qts, 12, false);
    pin(qts, 13, false);
    pin(qts, 11, true);
    frame(qts, &f, true);
    g_assert_cmpuint(f.y[10 * W + 4], ==, 16);
    pin(qts, 13, true);
    qtest_clock_step(qts, 10000000);
    pin(qts, 13, false);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], >, 100);           /* out, latched */
    pin(qts, 12, true);
    qtest_clock_step(qts, 10000000);
    pin(qts, 12, false);
    frame(qts, &f, false);
    g_assert_cmpuint(f.y[10 * W + 4], ==, 16);           /* in again */
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-scene/stamp", test_stamp);
    qtest_add_func("ingenic-scene/motion", test_motion);
    qtest_add_func("ingenic-scene/kelvin", test_kelvin);
    qtest_add_func("ingenic-scene/wb-gains", test_wb_gains);
    qtest_add_func("ingenic-scene/light", test_light);
    qtest_add_func("ingenic-scene/ir-single", test_ir_single);
    qtest_add_func("ingenic-scene/ir-latch", test_ir_latch);
    return g_test_run();
}
