/*
 * QTest testcase for the image sensors' latch timing and fault knobs: an
 * integration time or gain written over I2C reaches the picture frames
 * later, at the delay its driver declares, together under a group hold; and
 * the knobs hold a latch for more frames, drop or NACK writes, apply a
 * register late and skip a frame, each counted. The picture's white bar is
 * read from a T20 ISP's buffer frame by frame.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"
#include "qobject/qdict.h"
#include "qobject/qnum.h"

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
#define SENSOR          "/machine/peripheral/sens"

/* SC2336 (a group hold at 0x3812): lines from 0x3e01:0x3e02, ref 700 */
/* 300 lines (A) and 600 (B) */
#define SC_IT_A_HI      { 0x3e01, 0x25 }
#define SC_IT_A_LO      { 0x3e02, 0x80 }
#define SC_IT_B_HI      { 0x3e01, 0x4b }
#define SC_IT_B_LO      { 0x3e02, 0x00 }
#define SC_GAIN_2X      { 0x3e09, 0x08 }

typedef struct Reg {
    uint16_t reg;
    uint8_t val;
} Reg;

static int expect(double e)
{
    return 16 + lround(164 * pow(e, 1 / 2.2));
}

static QTestState *start(const char *model, const char *extra)
{
    QTestState *qts = qtest_initf("-machine ingenic-t20 -device image-sensor,"
                                  "id=sens,bus=i2c0,fps=25,motion=off,"
                                  "model=%s%s", model, extra);

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

static void write_regs(QTestState *qts, uint8_t addr, int reg_bytes,
                       const Reg *r, int n)
{
    qtest_writel(qts, I2C0 + 0x6c, 0);
    qtest_writel(qts, I2C0 + 0x04, addr);
    qtest_writel(qts, I2C0 + 0x6c, 1);
    for (int i = 0; i < n; i++) {
        if (reg_bytes == 2) {
            qtest_writel(qts, I2C0 + 0x10, r[i].reg >> 8);
        }
        qtest_writel(qts, I2C0 + 0x10, r[i].reg & 0xff);
        qtest_writel(qts, I2C0 + 0x10, 0x200 | r[i].val);
    }
}

#define SC(qts, ...) do { \
        const Reg r_[] = { __VA_ARGS__ }; \
        write_regs(qts, 0x30, 2, r_, ARRAY_SIZE(r_)); \
    } while (0)

#define GC(qts, ...) do { \
        const Reg r_[] = { __VA_ARGS__ }; \
        write_regs(qts, 0x29, 2, r_, ARRAY_SIZE(r_)); \
    } while (0)

static uint64_t get(QTestState *qts, const char *prop)
{
    QDict *rsp = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                           "{ 'path': %s, 'property': %s } }", SENSOR, prop);
    uint64_t v = qnum_get_uint(qobject_to(QNum, qdict_get(rsp, "return")));

    qobject_unref(rsp);
    return v;
}

static void set(QTestState *qts, const char *prop, uint32_t v)
{
    qtest_qmp_assert_success(qts, "{ 'execute': 'qom-set', 'arguments': "
                             "{ 'path': %s, 'property': %s, 'value': %u } }",
                             SENSOR, prop, v);
}

/* The next frame's white bar luma: 0 if the sensor sent no frame */
static int frame(QTestState *qts)
{
    uint8_t y;

    qtest_memset(qts, BUF + 10 * W + 4, 0, 1);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, BUF + 10 * W + 4, &y, 1);
    return y;
}

static void frames(QTestState *qts, int *y, int n)
{
    for (int i = 0; i < n; i++) {
        y[i] = frame(qts);
    }
}

/* The index of the first frame that is not @old, or @n */
static int first_not(const int *y, int n, int old)
{
    for (int i = 0; i < n; i++) {
        if (abs(y[i] - old) > 1) {
            return i;
        }
    }
    return n;
}

static void settle(QTestState *qts)
{
    qtest_clock_step(qts, FRAME_NS * 6);
}

static QTestState *sc2336(const char *extra)
{
    QTestState *qts = start("sc2336", extra);

    SC(qts, { 0x0100, 1 });
    SC(qts, SC_IT_A_HI, SC_IT_A_LO, { 0x3e09, 0x00 }, { 0x3e07, 0x80 });
    settle(qts);
    return qts;
}

/* the exposure shows the driver's declared 2 frames after it is written */
static void test_latch(void)
{
    QTestState *qts = sc2336("");
    int a = expect(300.0 / 700), b = expect(600.0 / 700);
    uint64_t l = get(qts, "latches");
    int y[8];

    g_assert_cmpint(abs(frame(qts) - a), <=, 1);
    SC(qts, SC_IT_B_HI, SC_IT_B_LO);
    frames(qts, y, 8);
    g_test_message("latch: %d %d %d %d %d (%d, %d)", y[0], y[1], y[2], y[3],
                   y[4], a, b);
    g_assert_cmpint(first_not(y, 8, a), ==, 2);
    g_assert_cmpint(abs(y[2] - b), <=, 1);
    g_assert_cmpint(get(qts, "latches") - l, ==, 1);
    g_assert_cmpint(get(qts, "writes-late"), ==, 0);
    qtest_quit(qts);
}

/* off: the registers reach the picture at the next frame */
static void test_latch_off(void)
{
    QTestState *qts = sc2336(",latch=off");
    int a = expect(300.0 / 700);
    int y[4];

    g_assert_cmpint(abs(frame(qts) - a), <=, 1);
    SC(qts, SC_IT_B_HI, SC_IT_B_LO);
    frames(qts, y, 4);
    g_assert_cmpint(first_not(y, 4, a), ==, 0);
    g_assert_cmpint(get(qts, "latches"), ==, 0);
    qtest_quit(qts);
}

/* without a group hold an integration time and a gain half a frame apart */
static void test_race(void)
{
    QTestState *qts = start("gc4653", "");
    int a = expect(250.0 / 1100), b = expect(500.0 / 1100);
    uint64_t l;
    int y[8];

    GC(qts, { 0x0202, 0x00 }, { 0x0203, 0xfa }, { 0x02b8, 0x01 },
       { 0x02b9, 0x00 });
    settle(qts);
    l = get(qts, "latches");
    GC(qts, { 0x0202, 0x01 }, { 0x0203, 0xf4 });        /* 500 lines, 1x */
    frame(qts);
    GC(qts, { 0x02b8, 0x02 });                          /* then 2x */
    frames(qts, y, 8);
    g_test_message("race: %d %d %d %d (%d, %d)", y[0], y[1], y[2], y[3], a,
                   b);
    /* the lines land first, the gain a frame after: 500 at 1x between */
    g_assert_cmpint(abs(y[0] - a), <=, 1);
    g_assert_cmpint(abs(y[1] - b), <=, 1);
    g_assert_cmpint(abs(y[2] - expect(1000.0 / 1100)), <=, 1);
    g_assert_cmpint(get(qts, "latches") - l, ==, 2);
    qtest_quit(qts);
}

/* a group hold lands both together, however far apart they are written */
static void test_group(void)
{
    QTestState *qts = sc2336("");
    uint64_t l = get(qts, "latches");
    int y[8];

    SC(qts, { 0x3812, 0x00 }, SC_IT_B_HI, SC_IT_B_LO);
    frame(qts);
    SC(qts, SC_GAIN_2X, { 0x3812, 0x30 });
    frames(qts, y, 8);
    g_test_message("group: %d %d %d %d %d", y[0], y[1], y[2], y[3], y[4]);
    /* 600 lines at 1x is 2 x 300 lines, 600 lines at 2x is 4 x */
    g_assert_cmpint(first_not(y, 8, expect(300.0 / 700)), ==, 2);
    g_assert_cmpint(abs(y[2] - expect(2 * 600.0 / 700)), <=, 1);
    g_assert_cmpint(get(qts, "latches") - l, ==, 1);
    qtest_quit(qts);
}

static void test_hold_nth(void)
{
    QTestState *qts = sc2336(",latch-hold-nth=1,latch-hold-frames=3");
    int a = expect(300.0 / 700), y[10];

    /* the latch of the first write after the settling is the one held */
    g_assert_cmpint(get(qts, "latches-held"), ==, 1);
    set(qts, "latch-hold-nth", 1);
    SC(qts, SC_IT_B_HI, SC_IT_B_LO);
    frames(qts, y, 10);
    g_test_message("hold: %d %d %d %d %d %d", y[0], y[1], y[2], y[3], y[4],
                   y[5]);
    g_assert_cmpint(first_not(y, 10, a), ==, 2 + 3);
    g_assert_cmpint(get(qts, "latches-held"), ==, 2);
    g_assert_cmpint(get(qts, "latch-hold-nth"), ==, 0);
    qtest_quit(qts);
}

static void test_hold_every(void)
{
    QTestState *qts = sc2336("");
    int y[10];

    set(qts, "latch-hold-every", 2);
    set(qts, "latch-hold-frames", 2);
    /* latches so far: 1; the next is the 2nd, held */
    SC(qts, SC_IT_B_HI, SC_IT_B_LO);
    frames(qts, y, 10);
    g_assert_cmpint(first_not(y, 10, expect(300.0 / 700)), ==, 2 + 2);
    g_assert_cmpint(get(qts, "latches-held"), ==, 1);
    SC(qts, SC_IT_A_HI, SC_IT_A_LO);
    frames(qts, y, 10);
    g_assert_cmpint(first_not(y, 10, expect(600.0 / 700)), ==, 2);
    g_assert_cmpint(get(qts, "latches-held"), ==, 1);
    qtest_quit(qts);
}

static void test_drop(void)
{
    QTestState *qts = sc2336(",nack=on");
    int a = expect(300.0 / 700), y[6];
    uint64_t w = get(qts, "writes");

    set(qts, "drop-nth", 2);
    SC(qts, { 0x3e01, 0x4b }, { 0x3e02, 0x00 });
    frames(qts, y, 6);
    /* the low byte was NACKed: the new high byte shows over the old low */
    g_assert_cmpint(get(qts, "writes") - w, ==, 2);
    g_assert_cmpint(get(qts, "writes-nacked"), ==, 1);
    g_assert_cmpint(get(qts, "writes-dropped"), ==, 0);
    g_assert_cmpint(first_not(y, 6, a), ==, 2);
    qtest_quit(qts);
}

static void test_drop_reg(void)
{
    QTestState *qts = sc2336("");
    int a = expect(300.0 / 700), y[6];

    set(qts, "drop-reg", 0x3e09);
    SC(qts, SC_GAIN_2X);
    frames(qts, y, 6);
    g_assert_cmpint(get(qts, "writes-dropped"), ==, 1);
    g_assert_cmpint(get(qts, "writes-nacked"), ==, 0);
    g_assert_cmpint(first_not(y, 6, a), ==, 6);
    set(qts, "drop-reg", 0xffffffff);
    SC(qts, SC_GAIN_2X);
    frames(qts, y, 6);
    g_assert_cmpint(first_not(y, 6, a), ==, 2);
    qtest_quit(qts);
}

static void test_drop_every(void)
{
    QTestState *qts = sc2336("");
    uint64_t w = get(qts, "writes");

    set(qts, "drop-every", 2);
    SC(qts, SC_IT_B_HI, SC_IT_B_LO, SC_GAIN_2X, { 0x3e07, 0x80 });
    g_assert_cmpint(get(qts, "writes") - w, ==, 4);
    g_assert_cmpint(get(qts, "writes-dropped"), ==, (w + 4) / 2 - w / 2);
    qtest_quit(qts);
}

static void test_late(void)
{
    QTestState *qts = sc2336("");
    int y[8];

    set(qts, "late-reg", 0x3e09);
    set(qts, "late-frames", 2);
    /* the lines at 2 frames, the gain a frame after them per the knob 2 */
    SC(qts, SC_IT_B_HI, SC_IT_B_LO, SC_GAIN_2X);
    frames(qts, y, 8);
    g_test_message("late: %d %d %d %d %d", y[0], y[1], y[2], y[3], y[4]);
    g_assert_cmpint(abs(y[2] - expect(600.0 / 700)), <=, 1);
    g_assert_cmpint(abs(y[4] - expect(1200.0 / 700)), <=, 1);
    g_assert_cmpint(get(qts, "writes-late"), ==, 1);
    qtest_quit(qts);
}

static void test_skip(void)
{
    QTestState *qts = sc2336("");
    uint64_t n = get(qts, "frames");
    int y[6];

    set(qts, "skip-nth", 3);
    frames(qts, y, 6);
    g_test_message("skip: %d %d %d %d %d %d", y[0], y[1], y[2], y[3], y[4],
                   y[5]);
    /* one frame is missing: the buffer keeps what the test cleared */
    g_assert_cmpint(!y[0] + !y[1] + !y[2] + !y[3] + !y[4] + !y[5], ==, 1);
    g_assert_cmpint(get(qts, "frames-skipped"), ==, 1);
    g_assert_cmpint(get(qts, "frames") - n, >=, 6);
    qtest_quit(qts);
}

static void wait_status(QTestState *qts, const char *key, const char *busy)
{
    for (int i = 0; i < 6000; i++) {
        QDict *rsp = qtest_qmp(qts, "{ 'execute': %s }", key);
        QDict *ret = qdict_get_qdict(rsp, "return");
        const char *st = qdict_get_try_str(ret, "status");
        bool done = st && strcmp(st, busy) && strcmp(st, "setup") &&
                    strcmp(st, "device");

        if (done) {
            g_assert_cmpstr(st, !=, "failed");
            qobject_unref(rsp);
            return;
        }
        qobject_unref(rsp);
        g_usleep(10000);
    }
    g_assert_not_reached();
}

/* the knobs, the counters and a write still waiting survive a save */
static void test_migrate(void)
{
    g_autofree char *path = g_strdup_printf("%s/ingenic-sensor-fault-%d.state",
                                            g_get_tmp_dir(), getpid());
    g_autofree char *uri = g_strdup_printf("file:%s", path);
    QTestState *src = sc2336(""), *dst;
    static const char *const props[] = {
        "latches", "latches-held", "writes", "writes-dropped", "writes-late",
        "frames", "latch-hold-every", "latch-hold-frames", "drop-every",
        "late-reg", "late-frames", "skip-every", "drop-nth",
    };
    uint64_t want[ARRAY_SIZE(props)];
    int y[8], a = expect(300.0 / 700);

    set(src, "latch-hold-every", 5);
    set(src, "latch-hold-frames", 2);
    set(src, "drop-every", 100);
    set(src, "late-reg", 0x3e09);
    set(src, "skip-every", 1000);
    set(src, "drop-nth", 50);
    SC(src, SC_IT_B_HI, SC_IT_B_LO);           /* waiting for its frame */
    for (int i = 0; i < ARRAY_SIZE(props); i++) {
        want[i] = get(src, props[i]);
    }
    qtest_qmp_assert_success(src, "{ 'execute': 'stop' }");
    qtest_qmp_assert_success(src, "{ 'execute': 'migrate', 'arguments': "
                             "{ 'uri': %s } }", uri);
    wait_status(src, "query-migrate", "active");

    dst = qtest_initf("-machine ingenic-t20 -device image-sensor,id=sens,"
                      "bus=i2c0,fps=25,motion=off,model=sc2336 -incoming %s",
                      uri);
    wait_status(dst, "query-status", "inmigrate");
    for (int i = 0; i < ARRAY_SIZE(props); i++) {
        g_assert_cmpuint(get(dst, props[i]), ==, want[i]);
    }
    qtest_qmp_assert_success(dst, "{ 'execute': 'cont' }");
    frames(dst, y, 8);
    g_assert_cmpint(first_not(y, 8, a), <, 3);
    qtest_quit(dst);
    qtest_quit(src);
    unlink(path);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-sensor-fault/latch", test_latch);
    qtest_add_func("/ingenic-sensor-fault/latch-off", test_latch_off);
    qtest_add_func("/ingenic-sensor-fault/race", test_race);
    qtest_add_func("/ingenic-sensor-fault/group", test_group);
    qtest_add_func("/ingenic-sensor-fault/hold-nth", test_hold_nth);
    qtest_add_func("/ingenic-sensor-fault/hold-every", test_hold_every);
    qtest_add_func("/ingenic-sensor-fault/drop", test_drop);
    qtest_add_func("/ingenic-sensor-fault/drop-reg", test_drop_reg);
    qtest_add_func("/ingenic-sensor-fault/drop-every", test_drop_every);
    qtest_add_func("/ingenic-sensor-fault/late", test_late);
    qtest_add_func("/ingenic-sensor-fault/skip", test_skip);
    qtest_add_func("/ingenic-sensor-fault/migrate", test_migrate);
    return g_test_run();
}
