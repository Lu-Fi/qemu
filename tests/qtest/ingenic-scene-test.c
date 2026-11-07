/*
 * QTest testcase for the image sensor's test scene, as a T31's ISP puts
 * it out on a 64x64 channel: the frame stamp, the moving square and the
 * stripe stood still, the light's colour temperature, the light and its
 * ramp, and IR through the IR-cut filter from the board's pins. The ISP's
 * white-balance gains on what it puts out, as a T31's AWB sets them (R
 * 0x1804, B 0x1808; a T21's 0x604 and 0x608, a T40's and T41's 0x4004 and
 * 0x4008, a T32's 0x6040 and 0x6044, which a T33's and T32 Pro's register
 * lists load): a part sees a grey as its sensor's own tuning file for that
 * part has it, the sensor's raw response, in the light's colour temperature,
 * and the gains that undo that response, which differ from sensor to sensor
 * and from part to part, leave the ramp grey in every light. A T10's, T20's
 * and T30's Apical core does the same with its gains by colour (R 0x80300, Gr
 * 0x80304, Gb 0x80308, B 0x8030c), a T30's on its multi-scaler's frames, its
 * sensors' responses from the Apical tuning files.
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
#define LISTS           0x00500000      /* a register list's table */

typedef struct Frame {
    uint8_t y[W * H], uv[W * H / 2];
} Frame;

/* An ISP: its channels at ch + n * 0x100, the VIC and the gain words */
typedef struct IspPart {
    const char *machine;
    const char *variant;                    /* T32 family: the SoC variant */
    uint32_t ch_en, ch, vic, vic_size;      /* vic_size: the T41's */
    uint32_t size, y_push, uv_push, y_stride, uv_stride, done_y, done_sta;
    uint32_t wb_r, wb_b, wb_reset;          /* gain words, B's at reset */
    const char *model;                      /* a sensor with a table here */
    uint32_t gain_3000[2], gain_5000[2], gain_6500[2];  /* R's and B's that */
                                            /* settle grey with it */
    uint32_t vic_start;                     /* the word that starts the VIC */
    uint32_t rcfg_trig, rcfg;               /* T33, T32 Pro: the register */
                                            /* lists' trigger and table */
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

/*
 * The T32 family: channel 1 on the T23's registers, VIC 0 started at +0x10;
 * its AWB puts R and B in 0x6040 and 0x6044 (a T33's and T32 Pro's tx-isp in
 * their module 7's register list, 0x04000000 | R and B alone)
 */
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

static const IspPart t33 = {
    .machine = "ingenic-t32", .variant = "t33n", .ch_en = 0xf0040,
    .ch = 0xf0100, .vic = 0x80000, .vic_start = 0x10, .size = 0x04,
    .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40, .uv_stride = 0x60,
    .done_y = 0x38, .done_sta = 0x3c, .wb_r = 0x6040, .wb_b = 0x6044,
    .rcfg_trig = 0x22010, .rcfg = 0x22020,
    .model = "os04d10",
    .gain_3000 = { 0x04000550, 0x04000e6c },
    .gain_5000 = { 0x0400077d, 0x04000925 },
    .gain_6500 = { 0x0400088d, 0x0400074a },
};

static const IspPart t32pro = {
    .machine = "ingenic-t32", .variant = "t32prozn", .ch_en = 0xf0040,
    .ch = 0xf0100, .vic = 0x80000, .vic_start = 0x10, .size = 0x04,
    .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40, .uv_stride = 0x60,
    .done_y = 0x38, .done_sta = 0x3c, .wb_r = 0x6040, .wb_b = 0x6044,
    .rcfg_trig = 0x1c010, .rcfg = 0x1c020,
    .model = "gc4653",
    .gain_3000 = { 0x04000546, 0x04000b92 },
    .gain_5000 = { 0x04000729, 0x0400084b },
    .gain_6500 = { 0x0400084f, 0x04000716 },
};

static const IspPart *part = &t31;
static const char *model = "jxq03";

static QTestState *start(const char *props)
{
    g_autofree char *soc = part->variant ?
        g_strdup_printf("-global ingenic-t31.soc-variant=%s", part->variant) :
        g_strdup("");
    QTestState *qts = qtest_initf("-machine %s %s -device image-sensor,id=cam,"
                                  "model=%s,bus=i2c0,fps=25,%s %s",
                                  part->machine, soc, model, props,
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

/*
 * The guest's AWB puts R and B in the words: a store, or a register list the
 * core loads (list 7 of the table at LISTS: one command for both words)
 */
static void set_gains(QTestState *qts, uint32_t r, uint32_t b)
{
    const uint32_t list = LISTS + 0x100;

    if (!part->rcfg) {
        qtest_writel(qts, ISP + part->wb_r, r);
        qtest_writel(qts, ISP + part->wb_b, b);
        return;
    }
    qtest_writel(qts, ISP + part->rcfg, LISTS);
    qtest_writel(qts, LISTS + 7 * 4, list);
    qtest_writel(qts, list, 3);
    qtest_writel(qts, list + 4, 1u << 31 | 1 << 18 | part->wb_r / 4);
    qtest_writel(qts, list + 8, r);
    qtest_writel(qts, list + 12, b & 0x3fff);
    qtest_writel(qts, ISP + part->rcfg_trig, 1 << 7);
}

static void test_wb_gains(const void *data)
{
    const IspPart *p = data;
    const uint32_t wb_b = ISP + p->wb_b;
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
    set_gains(qts, p->gain_5000[0], p->gain_5000[1]);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);

    /* more red */
    set_gains(qts, 0x04000c00, p->gain_5000[1]);
    frame(qts, &f, false);
    g_assert_cmpuint(ramp_uv[1], >, 131);

    /* and in 6500 K and 3000 K, its settled gains there leave grey grey */
    set(qts, "kelvin", 6500);
    set_gains(qts, p->gain_6500[0], p->gain_6500[1]);
    frame(qts, &f, false);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    if (p->gain_3000[0]) {
        set(qts, "kelvin", 3000);
        set_gains(qts, p->gain_3000[0], p->gain_3000[1]);
        frame(qts, &f, false);
        g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
        g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);
    }
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

/*
 * A T10 and a T20: the Apical core's frame writer 0 puts the frame in DDR,
 * and its gains are the static white balance's, by colour, 1.0 = 0x100
 */
#define T20_CORE        (ISP + 0x80000)
#define T20_WR          (T20_CORE + 0xb00)
#define T20_WR_UV       (T20_WR + 0x80)

static void t20_frame(QTestState *qts, Frame *f)
{
    qtest_clock_step(qts, FRAME_NS * 2);
    qtest_memread(qts, BUF, f->y, sizeof(f->y));
    qtest_memread(qts, BUF + W * H, f->uv, sizeof(f->uv));
}

typedef struct T20Part {
    const char *machine;
    uint32_t vic;                       /* its VIC start register */
    const char *model;
    unsigned kelvin;
    uint32_t wb_r, wb_b;                /* the gains that take a grey back */
} T20Part;

/* a JXQ03 has no Apical table: the old grey of a T20X's JXF23 */
static const T20Part apical_t10 = { "ingenic-t10", 0x20, "jxq03", 5000,
                                    0x191, 0x23c };
static const T20Part apical_t20 = { "ingenic-t20", 0x00, "jxq03", 5000,
                                    0x191, 0x23c };

/*
 * Each sensor's own: 1 / its table's raw R/G and B/G in the light, in
 * 0x100ths. Sensors without an exposure model, which show the scene as it is
 */
static const T20Part apical_sensors[] = {
    { "ingenic-t10", 0x20, "jxh42", 6500, 0x1fa, 0x183 },
    { "ingenic-t10", 0x20, "ov9712", 5000, 0x1c2, 0x1f5 },
    { "ingenic-t20", 0x00, "sc2135", 6500, 0x1c6, 0x123 },
    { "ingenic-t20", 0x00, "sc2135", 4000, 0x143, 0x1a0 },
};

/* The ramp's chroma: how far from grey, U and V together */
static int t20_tint(const uint8_t *ramp_uv)
{
    return abs(ramp_uv[0] - 128) + abs(ramp_uv[1] - 128);
}

static void test_wb_t20(const void *data)
{
    const T20Part *p = data;
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,id=cam,"
                                  "model=%s,bus=i2c0,fps=25,kelvin=%u",
                                  p->machine, p->model, p->kelvin);
    const uint8_t *ramp_uv;
    uint8_t luma;
    Frame f;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, W);
    qtest_writel(qts, T20_CORE + 0x14, H);
    qtest_writel(qts, T20_WR, 13 | 1 << 9);             /* NV12, on */
    qtest_writel(qts, T20_WR + 4, H << 16 | W);
    qtest_writel(qts, T20_WR + 8, BUF);
    qtest_writel(qts, T20_WR + 0x20, W);
    qtest_writel(qts, T20_WR + 0x24, 2);                /* running */
    qtest_writel(qts, T20_WR_UV, 1 << 9);
    qtest_writel(qts, T20_WR_UV + 4, H / 2 << 16 | W);
    qtest_writel(qts, T20_WR_UV + 8, BUF + W * H);
    qtest_writel(qts, T20_WR_UV + 0x20, W);
    qtest_writel(qts, T20_WR_UV + 0x24, 2);
    qtest_writel(qts, ISP + p->vic, 1);                 /* VIC start */

    /* not set yet: the scene as it comes */
    t20_frame(qts, &f);
    ramp_uv = f.uv + 28 * W + 32;
    if (p->kelvin != 6500) {
        g_assert_cmpuint(ramp_uv[0], <, 125);
        g_assert_cmpuint(ramp_uv[1], >, 131);
    }

    /* gains of 1.0: the raw signal, whose grey is far from grey */
    qtest_writel(qts, T20_CORE + 0x300, 0x100);
    qtest_writel(qts, T20_CORE + 0x304, 0x100);
    qtest_writel(qts, T20_CORE + 0x308, 0x100);
    qtest_writel(qts, T20_CORE + 0x30c, 0x100);
    t20_frame(qts, &f);
    g_assert_cmpint(t20_tint(ramp_uv), >, 8);

    /* the gains that take the light back to grey, from the raw response */
    qtest_writel(qts, T20_CORE + 0x300, p->wb_r);
    qtest_writel(qts, T20_CORE + 0x30c, p->wb_b);
    t20_frame(qts, &f);
    g_assert_cmpint(abs(ramp_uv[0] - 128), <=, 1);
    g_assert_cmpint(abs(ramp_uv[1] - 128), <=, 1);

    /* more red; less green, and the luma falls */
    qtest_writel(qts, T20_CORE + 0x300, 0x300);
    t20_frame(qts, &f);
    g_assert_cmpuint(ramp_uv[1], >, 131);
    qtest_writel(qts, T20_CORE + 0x300, p->wb_r);
    t20_frame(qts, &f);
    luma = f.y[50 * W + 32];
    qtest_writel(qts, T20_CORE + 0x304, 0x80);
    qtest_writel(qts, T20_CORE + 0x308, 0x80);
    t20_frame(qts, &f);
    g_assert_cmpuint(f.y[50 * W + 32], <, luma * 9 / 10);
    qtest_quit(qts);
}

/*
 * The colour bars of the scene (Y, Cb, Cr) the gains an SC2135's AWB settles
 * on, a hair off 1 / its response, leave within a level
 */
static void test_wb_t20_bars(void)
{
    static const uint8_t bars[8][3] = {
        { 180, 128, 128 }, { 162, 44, 142 }, { 131, 156, 44 }, { 112, 72, 58 },
        { 84, 184, 198 }, { 65, 100, 212 }, { 35, 212, 114 }, { 16, 128, 128 },
    };
    QTestState *qts = qtest_init("-machine ingenic-t20 -device "
                                 "image-sensor,id=cam,model=sc2135,bus=i2c0,"
                                 "fps=25,kelvin=6500,motion=off");
    Frame f;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, W);
    qtest_writel(qts, T20_CORE + 0x14, H);
    qtest_writel(qts, T20_WR, 13 | 1 << 9);
    qtest_writel(qts, T20_WR + 4, H << 16 | W);
    qtest_writel(qts, T20_WR + 8, BUF);
    qtest_writel(qts, T20_WR + 0x20, W);
    qtest_writel(qts, T20_WR + 0x24, 2);
    qtest_writel(qts, T20_WR_UV, 1 << 9);
    qtest_writel(qts, T20_WR_UV + 4, H / 2 << 16 | W);
    qtest_writel(qts, T20_WR_UV + 8, BUF + W * H);
    qtest_writel(qts, T20_WR_UV + 0x20, W);
    qtest_writel(qts, T20_WR_UV + 0x24, 2);
    qtest_writel(qts, ISP, 1);
    qtest_writel(qts, T20_CORE + 0x300, 0x1ca);         /* 1 / 1.008 off */
    qtest_writel(qts, T20_CORE + 0x304, 0x100);
    qtest_writel(qts, T20_CORE + 0x308, 0x100);
    qtest_writel(qts, T20_CORE + 0x30c, 0x125);
    t20_frame(qts, &f);
    for (int b = 0; b < 8; b++) {
        int x = b * W / 8 + 4;

        g_assert_cmpint(abs(f.y[10 * W + x] - bars[b][0]), <=, 1);
        g_assert_cmpint(abs(f.uv[5 * W + (x & ~1)] - bars[b][1]), <=, 1);
        g_assert_cmpint(abs(f.uv[5 * W + (x & ~1) + 1] - bars[b][2]), <=, 1);
    }
    qtest_quit(qts);
}

/* Another sensor's gains tint it: an SC2135 needs less blue than an OV2710 */
static void test_wb_t20_other_sensor(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t20 -device "
                                 "image-sensor,id=cam,model=ov2710,bus=i2c0,"
                                 "fps=25,kelvin=6500");
    const uint8_t *ramp_uv;
    Frame f;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, W);
    qtest_writel(qts, T20_CORE + 0x14, H);
    qtest_writel(qts, T20_WR, 13 | 1 << 9);
    qtest_writel(qts, T20_WR + 4, H << 16 | W);
    qtest_writel(qts, T20_WR + 8, BUF);
    qtest_writel(qts, T20_WR + 0x20, W);
    qtest_writel(qts, T20_WR + 0x24, 2);
    qtest_writel(qts, T20_WR_UV, 1 << 9);
    qtest_writel(qts, T20_WR_UV + 4, H / 2 << 16 | W);
    qtest_writel(qts, T20_WR_UV + 8, BUF + W * H);
    qtest_writel(qts, T20_WR_UV + 0x20, W);
    qtest_writel(qts, T20_WR_UV + 0x24, 2);
    qtest_writel(qts, ISP, 1);
    qtest_writel(qts, T20_CORE + 0x300, 0x1c6);         /* SC2135, 6500 K */
    qtest_writel(qts, T20_CORE + 0x304, 0x100);
    qtest_writel(qts, T20_CORE + 0x308, 0x100);
    qtest_writel(qts, T20_CORE + 0x30c, 0x123);
    t20_frame(qts, &f);
    ramp_uv = f.uv + 28 * W + 32;
    g_assert_cmpint(t20_tint(ramp_uv), >, 4);
    qtest_quit(qts);
}

/*
 * A T30: the Apical core's gains scale the picture its multi-scaler puts out,
 * which the core feeds through the noise reducer's line buffer
 */
#define T30_NCU         0x13090000
#define T30_MSC         0x130b0000
#define T30_MSC_CH1     (T30_MSC + 0x200)

static void t30_push(QTestState *qts)
{
    qtest_writel(qts, T30_MSC_CH1, W << 16 | H);
    qtest_writel(qts, T30_MSC_CH1 + 0x80, W);
    qtest_writel(qts, T30_MSC_CH1 + 0x98, W);
    qtest_writel(qts, T30_MSC_CH1 + 0x6c, BUF);
    qtest_writel(qts, T30_MSC_CH1 + 0x84, BUF + W * H);
}

static void t30_frame(QTestState *qts, Frame *f)
{
    qtest_clock_step(qts, FRAME_NS);
    g_assert_false(qtest_readl(qts, T30_MSC_CH1 + 0x7c) & 1);
    qtest_readl(qts, T30_MSC_CH1 + 0x74);
    qtest_memread(qts, BUF, f->y, sizeof(f->y));
    qtest_memread(qts, BUF + W * H, f->uv, sizeof(f->uv));
    t30_push(qts);
}

/* A T30's gains are the T20 part's table, with a T30's responses */
static const T20Part apical_t30 = { "ingenic-t30", 0x00, "jxq03", 5000,
                                    0x191, 0x23c };

static const T20Part apical_t30_sensors[] = {
    { "ingenic-t30", 0x00, "jxh62", 5000, 0x1c7, 0x1bb },
    { "ingenic-t30", 0x00, "sc2232", 6500, 0x1b8, 0x163 },
};

static void test_wb_t30(const void *data)
{
    const T20Part *p = data;
    QTestState *qts = qtest_initf("-machine ingenic-t30 -device "
                                  "image-sensor,id=cam,model=%s,bus=i2c0,"
                                  "fps=25,kelvin=%u", p->model, p->kelvin);
    const uint8_t *ramp_uv;
    Frame f;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, W);
    qtest_writel(qts, T20_CORE + 0x14, H);
    /* the noise reducer passes the line buffer's frames on, loaded */
    qtest_writel(qts, T30_NCU + 0x02c, H << 16 | W);
    qtest_writel(qts, T30_NCU + 0x024, 0x0c000000);
    qtest_writel(qts, T30_NCU + 0x00c, 5);
    qtest_writel(qts, T30_NCU + 0x004, 2);
    qtest_writel(qts, T30_NCU + 0x004, 1);
    qtest_writel(qts, T30_MSC + 0x004, 2);
    t30_push(qts);
    qtest_writel(qts, ISP + p->vic, 1);                 /* VIC start */

    /* not set yet: the scene as it comes */
    qtest_clock_step(qts, FRAME_NS);
    t30_frame(qts, &f);
    ramp_uv = f.uv + 28 * W + 32;
    if (p->kelvin != 6500) {
        g_assert_cmpuint(ramp_uv[0], <, 125);
        g_assert_cmpuint(ramp_uv[1], >, 131);
    }

    /* gains of 1.0: the raw signal, whose grey is far from grey */
    qtest_writel(qts, T20_CORE + 0x300, 0x100);
    qtest_writel(qts, T20_CORE + 0x304, 0x100);
    qtest_writel(qts, T20_CORE + 0x308, 0x100);
    qtest_writel(qts, T20_CORE + 0x30c, 0x100);
    t30_frame(qts, &f);
    g_assert_cmpint(t20_tint(ramp_uv), >, 8);

    /* the gains that take the light back to grey, from the raw response */
    qtest_writel(qts, T20_CORE + 0x300, p->wb_r);
    qtest_writel(qts, T20_CORE + 0x30c, p->wb_b);
    t30_frame(qts, &f);
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
static const RawCase raw_t33_os04d10_4000 = {
    &t33, "os04d10", "kelvin=4000", 0x0400066e, 0x04000b8a,
};
static const RawCase raw_t32pro_gc4653_4000 = {
    &t32pro, "gc4653", "kelvin=4000", 0x0400063c, 0x040009d9,
};
/* a JXQ03 has no table for the T33 or the T32 Pro */
static const RawCase raw_t33_no_table = {
    &t33, "jxq03", "kelvin=5000", 0x0400059c, 0x04000c14,
};
static const RawCase raw_t32pro_no_table = {
    &t32pro, "jxq03", "kelvin=5000", 0x0400059c, 0x04000c14,
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
    qtest_add_data_func("ingenic-scene/wb-gains/t33", &t33, test_wb_gains);
    qtest_add_data_func("ingenic-scene/wb-gains/t32pro", &t32pro,
                        test_wb_gains);
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
    qtest_add_data_func("ingenic-scene/raw/t33-os04d10-4000",
                        &raw_t33_os04d10_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t32pro-gc4653-4000",
                        &raw_t32pro_gc4653_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t33-no-table",
                        &raw_t33_no_table, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t32pro-no-table",
                        &raw_t32pro_no_table, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t40-gc4653-4000",
                        &raw_t40_gc4653_4000, test_raw_response);
    qtest_add_data_func("ingenic-scene/raw/t41-os04d10-4000",
                        &raw_t41_os04d10_4000, test_raw_response);
    qtest_add_func("ingenic-scene/raw/other-sensor", test_raw_other_sensor);
    qtest_add_func("ingenic-scene/raw/other-part", test_raw_other_part);
    qtest_add_data_func("ingenic-scene/wb-gains/t10", &apical_t10,
                        test_wb_t20);
    qtest_add_data_func("ingenic-scene/wb-gains/t20", &apical_t20,
                        test_wb_t20);
    qtest_add_func("ingenic-scene/wb-gains/t20-bars", test_wb_t20_bars);
    qtest_add_func("ingenic-scene/wb-gains/t20-other-sensor",
                   test_wb_t20_other_sensor);
    for (int i = 0; i < ARRAY_SIZE(apical_sensors); i++) {
        const T20Part *p = &apical_sensors[i];
        g_autofree char *path = g_strdup_printf(
            "ingenic-scene/wb-gains/%s-%s-%u", p->machine + 8, p->model,
            p->kelvin);

        qtest_add_data_func(path, p, test_wb_t20);
    }
    qtest_add_data_func("ingenic-scene/wb-gains/t30", &apical_t30,
                        test_wb_t30);
    for (int i = 0; i < ARRAY_SIZE(apical_t30_sensors); i++) {
        const T20Part *p = &apical_t30_sensors[i];
        g_autofree char *path = g_strdup_printf(
            "ingenic-scene/wb-gains/t30-%s-%u", p->model, p->kelvin);

        qtest_add_data_func(path, p, test_wb_t30);
    }
    qtest_add_func("ingenic-scene/light", test_light);
    qtest_add_func("ingenic-scene/ir-single", test_ir_single);
    qtest_add_func("ingenic-scene/ir-latch", test_ir_latch);
    return g_test_run();
}
