/*
 * QTest testcase for the image sensor's test scene, as a T31's ISP puts
 * it out on a 64x64 channel: the frame stamp, the moving square and the
 * stripe stood still, the light's colour temperature, the light and its
 * ramp, and IR through the IR-cut filter from the board's pins. The ISP's
 * white-balance gains on what it puts out, as a T31's AWB sets them (R
 * 0x1804, B 0x1808; a T21's 0x604 and 0x608, a T40's and T41's 0x4004 and
 * 0x4008): 5000 K's take the ramp back to grey.
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
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define GPIO_PB         0x10011000
#define FRAME_NS        40000000        /* fps=25 */
#define T41_FIRST_NS    135000000       /* a T41NQ's first frame */
#define BUF             0x00400000
#define W               64
#define H               64

typedef struct Frame {
    uint8_t y[W * H], uv[W * H / 2];
} Frame;

/* An ISP: its channels at ch + n * 0x100, the VIC and the gain words */
typedef struct IspPart {
    const char *machine;
    uint32_t ch_en, ch, vic, vic_size;      /* vic_size: the T41's */
    uint32_t size, y_push, uv_push, y_stride, uv_stride, done_y, done_sta;
    uint32_t wb_r, wb_b, wb_reset;          /* gain words, B's at reset */
    uint32_t bypass, latch;                 /* T41: lens correction off; */
                                            /* its channels' latch */
    bool second;                            /* its sensor is input 1 */
} IspPart;

static const IspPart t21 = {
    .machine = "ingenic-t21", .ch_en = 0x2304, .ch = 0x2400, .vic = 0x10000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x604, .wb_b = 0x608,
};

static const IspPart t31 = {
    .machine = "ingenic-t31", .ch_en = 0x9804, .ch = 0x9900, .vic = 0xe0000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x1804, .wb_b = 0x1808,
};

static const IspPart t40 = {
    .machine = "ingenic-t40", .ch_en = 0x16008, .ch = 0x16100,
    .vic = 0x80000, .size = 0x00, .y_push = 0x6c, .uv_push = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x4004, .wb_b = 0x4008,
};

static const IspPart t41 = {
    .machine = "ingenic-t41", .ch_en = 0xf0008, .ch = 0xf0100,
    .vic = 0x80000, .vic_size = 0x80, .size = 0x00, .y_push = 0x6c,
    .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98, .done_y = 0x74,
    .done_sta = 0x7c, .wb_r = 0x4004, .wb_b = 0x4008,
    .wb_reset = 0x04000400, .bypass = 0x40, .latch = 0xf0010,
};

/* The T40's second pipeline: VIC 1 takes input 1 */
static const IspPart t40_pipe1 = {
    .machine = "ingenic-t40", .ch_en = 0x17008, .ch = 0x17100,
    .vic = 0x90000, .size = 0x00, .y_push = 0x6c, .uv_push = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x5004, .wb_b = 0x5008, .second = true,
};

static const IspPart *part = &t31;

static QTestState *start(const char *props)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,id=cam,"
                                  "model=jxq03,bus=i2c0,fps=25,%s %s",
                                  part->machine, props,
                                  part->second ?
                                  "-device image-sensor,id=cam1,model=jxq03,"
                                  "bus=i2c1,input=1,fps=25,kelvin=5000" : "");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    if (part->bypass) {
        /* a word that waits for the latch (0x1000) */
        qtest_writel(qts, ISP + part->bypass, 1u << 20);
        qtest_writel(qts, ISP + 0x1000, 1);
    }
    qtest_writel(qts, ISP + part->ch_en, 2);
    return qts;
}

/* Channel 1, which these tests read */
static uint32_t ch1(uint32_t reg)
{
    return ISP + part->ch + 0x100 + reg;
}

static void push(QTestState *qts)
{
    qtest_writel(qts, ch1(part->size), W << 16 | H);
    qtest_writel(qts, ch1(part->y_stride), W);
    qtest_writel(qts, ch1(part->uv_stride), W);
    qtest_writel(qts, ch1(part->y_push), BUF);
    qtest_writel(qts, ch1(part->uv_push), BUF + W * H);
    if (part->latch) {
        qtest_writel(qts, ISP + part->latch, 1);
    }
}

/* The next frame channel 1 puts out: the first at the VIC's start */
static void frame(QTestState *qts, Frame *f, bool first)
{
    if (first) {
        push(qts);
        if (part->vic_size) {
            qtest_writel(qts, ISP + part->vic_size, W << 16 | H);
        }
        qtest_writel(qts, ISP + part->vic, 1);
        if (part->vic_size) {
            qtest_clock_step(qts, T41_FIRST_NS);
        }
    } else {
        qtest_clock_step(qts, FRAME_NS);
    }
    g_assert_false(qtest_readl(qts, ch1(part->done_sta)) & 1);
    qtest_readl(qts, ch1(part->done_y));
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
                    "'path': %s, 'property': %s, 'value': %u } }",
                    part->second ? "/machine/peripheral/cam1"
                                 : "/machine/peripheral/cam", prop, v));
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

static void test_wb_gains(const void *data)
{
    const IspPart *p = data;
    const uint32_t wb_r = ISP + p->wb_r, wb_b = ISP + p->wb_b;
    QTestState *qts;
    const uint8_t *ramp_uv;
    Frame f;

    part = p;
    qts = start("kelvin=5000");

    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    g_assert_cmphex(qtest_readl(qts, wb_b), ==, p->wb_reset);
    if (p->wb_reset) {
        /* a T41's, 1.0 out of reset: the raw signal, a grey greenish */
        g_assert_cmpuint(ramp_uv[0], <, 125);
        g_assert_cmpuint(ramp_uv[1], <, 128);
    } else {
        /* not written yet: the scene as it comes */
        g_assert_cmpuint(ramp_uv[0], <, 125);
        g_assert_cmpuint(ramp_uv[1], >, 131);
    }

    /* the gains a qemu_t31x guest's AWB settles on in 5000 K */
    qtest_writel(qts, wb_r, 0x0400059c);
    qtest_writel(qts, wb_b, 0x04000c14);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);

    /* more red */
    qtest_writel(qts, wb_r, 0x04000800);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    /* and in 6500 K, the gains it settles on there leave grey grey */
    set(qts, "kelvin", 6500);
    qtest_writel(qts, wb_r, 0x04000728);
    qtest_writel(qts, wb_b, 0x040009dc);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    qtest_quit(qts);
}

/* Each of the T40's pipelines has its gains: pipeline 1's are 0x5004... */
static void test_wb_pipe1(void)
{
    QTestState *qts;
    const uint8_t *ramp_uv;
    Frame f;

    part = &t40_pipe1;
    qts = start("kelvin=5000");
    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    g_assert_cmpuint(ramp_uv[0], <, 125);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    /* pipeline 0's gains leave it as it was */
    qtest_writel(qts, ISP + t40.wb_r, 0x0400059c);
    qtest_writel(qts, ISP + t40.wb_b, 0x04000c14);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[0], <, 125);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    qtest_writel(qts, ISP + t40_pipe1.wb_r, 0x0400059c);
    qtest_writel(qts, ISP + t40_pipe1.wb_b, 0x04000c14);
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
    qtest_add_data_func("ingenic-scene/wb-gains/t21", &t21, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t31", &t31, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t40", &t40, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t41", &t41, test_wb_gains);
    qtest_add_func("ingenic-scene/wb-gains/t40-pipe1", test_wb_pipe1);
    qtest_add_func("ingenic-scene/light", test_light);
    qtest_add_func("ingenic-scene/ir-single", test_ir_single);
    qtest_add_func("ingenic-scene/ir-latch", test_ir_latch);
    return g_test_run();
}
