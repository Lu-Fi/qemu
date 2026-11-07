/*
 * QTest testcase for the image sensor's test scene, as a T31's ISP puts
 * it out on a 64x64 channel: the frame stamp, the moving square and the
 * stripe stood still, the light's colour temperature, the light and its
 * ramp, and IR through the IR-cut filter from the board's pins. The ISP's
 * white-balance gains on what it puts out, as a T31's AWB sets them (R
 * 0x1804, B 0x1808; a T21's 0x604 and 0x608, a T40's and T41's 0x4004 and
 * 0x4008, a T32's 0x6040 and 0x6044): a part sees a grey as its sensor's own
 * tuning file for that part has it, the sensor's raw response, in the light's
 * colour temperature, and the gains that undo that response, which differ from
 * sensor to sensor and from part to part, leave the ramp grey in every light.
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
    const char *model;                      /* a sensor with a table here */
    uint32_t gain_3000[2], gain_5000[2], gain_6500[2];  /* R's and B's that */
                                            /* settle grey with it */
    uint32_t vic_start;                     /* the word that starts the VIC */
    uint32_t bypass, latch;                 /* T41: lens correction off; */
                                            /* its channels' latch */
    bool second;                            /* its sensor is input 1 */
} IspPart;

static const IspPart t21 = {
    .machine = "ingenic-t21", .ch_en = 0x2304, .ch = 0x2400, .vic = 0x10000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x604, .wb_b = 0x608,
    .model = "jxf23",
    .gain_3000 = { 0x040004fc, 0x04000934 },
    .gain_5000 = { 0x04000707, 0x040006e5 },
    .gain_6500 = { 0x04000842, 0x04000616 },
};

/* The T23: channel 1 on the T32's registers, its own ISP and VIC */
static const IspPart t23 = {
    .machine = "ingenic-t23", .ch_en = 0xd040, .ch = 0xd100, .vic = 0xe0000,
    .size = 0x04, .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40,
    .uv_stride = 0x60, .done_y = 0x38, .done_sta = 0x3c,
    .wb_r = 0x1804, .wb_b = 0x1808,
    .model = "gc1084",
    .gain_3000 = { 0x040004fd, 0x04000a6c },
    .gain_5000 = { 0x040006bd, 0x04000800 },
    .gain_6500 = { 0x04000743, 0x04000726 },
};

static const IspPart t31 = {
    .machine = "ingenic-t31", .ch_en = 0x9804, .ch = 0x9900, .vic = 0xe0000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x1804, .wb_b = 0x1808,
    /* a JXQ03's raw R/G and B/G in 3000, 5000 and 6500 K, as its tuning file */
    .model = "jxq03",
    .gain_3000 = { 0x0400054b, 0x040007be },
    .gain_5000 = { 0x040006c5, 0x0400061f },
    .gain_6500 = { 0x0400075e, 0x04000574 },
};

static const IspPart t40 = {
    .machine = "ingenic-t40", .ch_en = 0x16008, .ch = 0x16100,
    .vic = 0x80000, .size = 0x00, .y_push = 0x6c, .uv_push = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x4004, .wb_b = 0x4008,
    .model = "imx307",
    .gain_3000 = { 0x04000546, 0x04000bb4 },
    .gain_5000 = { 0x040007a1, 0x040006fd },
    .gain_6500 = { 0x040008d4, 0x040005cb },
};

static const IspPart t41 = {
    .machine = "ingenic-t41", .ch_en = 0xf0008, .ch = 0xf0100,
    .vic = 0x80000, .vic_size = 0x80, .size = 0x00, .y_push = 0x6c,
    .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98, .done_y = 0x74,
    .done_sta = 0x7c, .wb_r = 0x4004, .wb_b = 0x4008,
    .wb_reset = 0x04000400, .bypass = 0x40, .latch = 0xf0010,
    .model = "gc4023",
    .gain_3000 = { 0x040004b1, 0x0400090b },
    .gain_5000 = { 0x040006a1, 0x040006f1 },
    .gain_6500 = { 0x0400075b, 0x04000637 },
};

/* The T40's second pipeline: VIC 1 takes input 1 */
static const IspPart t40_pipe1 = {
    .machine = "ingenic-t40", .ch_en = 0x17008, .ch = 0x17100,
    .vic = 0x90000, .size = 0x00, .y_push = 0x6c, .uv_push = 0x84,
    .y_stride = 0x80, .uv_stride = 0x98, .done_y = 0x74, .done_sta = 0x7c,
    .wb_r = 0x5004, .wb_b = 0x5008, .second = true,
};

/* The T32: channel 1 on the T23's registers, VIC 0 started at +0x10 */
static const IspPart t32 = {
    .machine = "ingenic-t32", .ch_en = 0xf0040, .ch = 0xf0100, .vic = 0x80000,
    .vic_start = 0x10, .size = 0x04, .y_push = 0x30, .uv_push = 0x50,
    .y_stride = 0x40, .uv_stride = 0x60, .done_y = 0x38, .done_sta = 0x3c,
    .wb_r = 0x6040, .wb_b = 0x6044,
    .model = "cv8001",
    .gain_3000 = { 0x0400057c, 0x04000c31 },
    .gain_5000 = { 0x040007a4, 0x040008e3 },
    .gain_6500 = { 0x040008d4, 0x04000739 },
};

static const IspPart *part = &t31;
static const char *model = "jxq03";

static QTestState *start(const char *props)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,id=cam,"
                                  "model=%s,bus=i2c0,fps=25,%s %s",
                                  part->machine, model, props,
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
        qtest_writel(qts, ISP + part->vic + part->vic_start, 1);
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
    model = p->model;
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

    /* the gains a guest's AWB settles on in 5000 K */
    qtest_writel(qts, wb_r, p->gain_5000[0]);
    qtest_writel(qts, wb_b, p->gain_5000[1]);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);

    /* more red */
    qtest_writel(qts, wb_r, 0x04000c00);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    /* and in 6500 K and 3000 K, its settled gains there leave grey grey */
    set(qts, "kelvin", 6500);
    qtest_writel(qts, wb_r, p->gain_6500[0]);
    qtest_writel(qts, wb_b, p->gain_6500[1]);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    set(qts, "kelvin", 3000);
    qtest_writel(qts, wb_r, p->gain_3000[0]);
    qtest_writel(qts, wb_b, p->gain_3000[1]);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    qtest_quit(qts);
    model = "jxq03";
}

/* A part's sensor and light, and the gains that undo the sensor's grey there */
typedef struct RawCase {
    const IspPart *part;
    const char *model;
    const char *props;
    uint32_t wb_r, wb_b;
} RawCase;

static void test_raw_response(const void *data)
{
    const RawCase *c = data;
    QTestState *qts;
    const uint8_t *ramp_uv;
    Frame f;

    part = c->part;
    model = c->model;
    qts = start(c->props);
    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    /* gains of 1.0: the raw signal, whose grey is far from grey */
    qtest_writel(qts, ISP + part->wb_r, 0x04000400);
    qtest_writel(qts, ISP + part->wb_b, 0x04000400);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128) + abs(ramp_uv[1] - 128), >, 8);
    qtest_writel(qts, ISP + part->wb_r, c->wb_r);
    qtest_writel(qts, ISP + part->wb_b, c->wb_b);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    qtest_quit(qts);
    model = "jxq03";
}

/* Another sensor's gains tint it: an IMX307 needs more blue than a JXQ03 */
static void test_raw_other_sensor(void)
{
    QTestState *qts;
    const uint8_t *ramp_uv;
    Frame f;

    part = &t31;
    model = "imx307";
    qts = start("kelvin=5000");
    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    qtest_writel(qts, ISP + t31.wb_r, t31.gain_5000[0]);
    qtest_writel(qts, ISP + t31.wb_b, t31.gain_5000[1]);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[0], <, 124);
    qtest_quit(qts);
    model = "jxq03";
}

/*
 * The same sensor on another part: a GC4653's grey is far bluer in a T41's
 * tuning file than in a T32's (B/G 78 and 58 of 256 in 3000 K), so the
 * gains of one tint the other
 */
static void test_raw_other_part(void)
{
    QTestState *qts;
    const uint8_t *ramp_uv;
    Frame f;

    part = &t41;
    model = "gc4653";
    qts = start("kelvin=3000");
    frame(qts, &f, true);
    ramp_uv = f.uv + 28 * W + 32;
    qtest_writel(qts, ISP + t41.wb_r, 0x0400052e);
    qtest_writel(qts, ISP + t41.wb_b, 0x04001194);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[0], >, 132);               /* too blue */
    qtest_writel(qts, ISP + t41.wb_r, 0x04000581);
    qtest_writel(qts, ISP + t41.wb_b, 0x04000d37);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    qtest_quit(qts);
    model = "jxq03";
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
    qtest_writel(qts, ISP + t40.wb_r, 0x040006c2);
    qtest_writel(qts, ISP + t40.wb_b, 0x0400067b);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[0], <, 125);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    qtest_writel(qts, ISP + t40_pipe1.wb_r, 0x040006c2);
    qtest_writel(qts, ISP + t40_pipe1.wb_b, 0x0400067b);
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

/*
 * Gains 1 / raw R/G and 1 / raw B/G, 1.0 at 0x400, of the tuning files'
 * tables; raw-r and raw-b hold a grey's raw response whatever the light, and
 * a model with no table on a part keeps an OS04D10's
 */
static const RawCase raw_imx307_3000 = {
    &t31, "imx307", "kelvin=3000", 0x04000489, 0x04000bce,
};
static const RawCase raw_imx307_5000 = {
    &t31, "imx307", "kelvin=5000", 0x04000679, 0x04000846,
};
static const RawCase raw_imx307_6500 = {
    &t31, "imx307", "kelvin=6500", 0x0400074d, 0x040006fa,
};
static const RawCase raw_gc4023_4000 = {
    &t31, "gc4023", "kelvin=4000", 0x04000554, 0x040007b3,
};
static const RawCase raw_jxq03_3000 = {
    &t31, "jxq03", "kelvin=3000", 0x0400054b, 0x040007be,
};
static const RawCase raw_override = {
    &t31, "gc4023", "kelvin=3000,raw-r=144,raw-b=105", 0x0400071c, 0x040009c0,
};
static const RawCase raw_no_table = {
    &t31, "gc5603", "kelvin=5000", 0x0400059c, 0x04000c14,
};
/* a JXQ03 has tables for the T31 and the T40, not the T41 */
static const RawCase raw_t41_no_table = {
    &t41, "jxq03", "kelvin=5000", 0x0400059c, 0x04000c14,
};
static const RawCase raw_t21_jxf37_4000 = {
    &t21, "jxf37", "kelvin=4000", 0x0400056f, 0x0400075e,
};
static const RawCase raw_t23_os02n10_4000 = {
    &t23, "os02n10", "kelvin=4000", 0x0400060b, 0x04000955,
};
static const RawCase raw_t32_gc4023_4000 = {
    &t32, "gc4023", "kelvin=4000", 0x040005e3, 0x04000699,
};
static const RawCase raw_t40_gc4653_4000 = {
    &t40, "gc4653", "kelvin=4000", 0x040006c2, 0x04000b41,
};
static const RawCase raw_t41_os04d10_4000 = {
    &t41, "os04d10", "kelvin=4000", 0x0400065a, 0x04000b12,
};

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-scene/stamp", test_stamp);
    qtest_add_func("ingenic-scene/motion", test_motion);
    qtest_add_func("ingenic-scene/kelvin", test_kelvin);
    qtest_add_data_func("ingenic-scene/wb-gains/t21", &t21, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t23", &t23, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t31", &t31, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t40", &t40, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t41", &t41, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t32", &t32, test_wb_gains);
    qtest_add_func("ingenic-scene/wb-gains/t40-pipe1", test_wb_pipe1);
    qtest_add_data_func("ingenic-scene/raw/imx307-3000", &raw_imx307_3000,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/imx307-5000", &raw_imx307_5000,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/imx307-6500", &raw_imx307_6500,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/gc4023-4000", &raw_gc4023_4000,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/jxq03-3000", &raw_jxq03_3000,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/override", &raw_override,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/no-table", &raw_no_table,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t41-no-table", &raw_t41_no_table,
                        test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t21-jxf37-4000",
                        &raw_t21_jxf37_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t23-os02n10-4000",
                        &raw_t23_os02n10_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t32-gc4023-4000",
                        &raw_t32_gc4023_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t40-gc4653-4000",
                        &raw_t40_gc4653_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t41-os04d10-4000",
                        &raw_t41_os04d10_4000, test_raw_response);
    qtest_add_func("ingenic-scene/raw/other-sensor", test_raw_other_sensor);
    qtest_add_func("ingenic-scene/raw/other-part", test_raw_other_part);
    qtest_add_func("ingenic-scene/light", test_light);
    qtest_add_func("ingenic-scene/ir-single", test_ir_single);
    qtest_add_func("ingenic-scene/ir-latch", test_ir_latch);
    return g_test_run();
}
