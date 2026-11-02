/*
 * QTest testcase for the Ingenic PWM block
 *
 * Its registers as a T41NQ, an A1N and a T32LQ read them at U-Boot: which
 * bits each keeps, the T32's 4 channels against 16, the 4 KiB window
 * repeating through 64 KiB. Its waveform as a T41NQ's and a T32LQ's pads
 * show it: HIGH + LOW ticks a period, the init level first, either count
 * at 0 holding it; new counts and levels only through PWMUPDATE, at the end
 * of the period, BUSY set until then; ENC finishing the period before the
 * idle level; both counts at 0 a period of 2^17 ticks. The output traced as
 * JSON lines.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define CPM             0x10000000

#define PWM_CCFG0       0x00
#define PWM_CCFG1       0x04
#define PWM_ENS         0x10
#define PWM_ENC         0x14
#define PWM_EN          0x18
#define PWM_UP          0x20
#define PWM_BUSY        0x24
#define PWM_LVL         0x30
#define PWM_WCFG(n)     (0xb0 + (n) * 4)
#define PWM_DES         0x100
#define PWM_DE          0x108
#define PWM_DCR0        0x110
#define PWM_DFER        0x124
#define PWM_DINTC       0x138
#define PWM_DAR(n)      (0x140 + (n) * 4)
#define PWM_DTLR(n)     (0x190 + (n) * 4)
#define PWM_OEN         0x300

typedef struct {
    const char *machine;
    uint32_t base;
    unsigned channels;
    uint32_t gate;              /* CLKGR1 */
    unsigned gate_bit;
    uint32_t cdr;               /* PWMCDR */
    uint32_t clocks[3];         /* CPCCR, CPAPCR, CPMPCR as U-Boot sets them */
} PwmCase;

static const PwmCase t41 = { "ingenic-t41", 0x13460000, 16, 0x28, 7, 0xfc,
                             { 0x9a0b5520, 0x05b008bd, 0x15d0889d } };
static const PwmCase a1 = { "ingenic-a1", 0x13460000, 16, 0x38, 15, 0xfc,
                            { 0x9a094410, 0x05c0510d, 0x0860510d } };
static const PwmCase t32 = { "ingenic-t32", 0x13450000, 4, 0x28, 9, 0x5c,
                             { 0x9a7b5510, 0x04b0510d, 0x0640510d } };

static uint64_t qom_uint(QTestState *qts, const char *prop)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': '/machine/soc/pwm', 'property': %s } }",
                         prop);
    uint64_t v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_int(r, "return");
    qobject_unref(r);
    return v;
}

static uint64_t out(QTestState *qts, unsigned n, const char *what)
{
    g_autofree char *prop = g_strdup_printf("pwm%u-%s", n, what);

    return qom_uint(qts, prop);
}

static QTestState *start(const PwmCase *c, const char *extra)
{
    QTestState *qts = qtest_initf("-machine %s %s", c->machine, extra);
    uint32_t gr = qtest_readl(qts, CPM + c->gate);

    qtest_writel(qts, CPM + 0x10, c->clocks[1]);
    qtest_writel(qts, CPM + 0x14, c->clocks[2]);
    qtest_writel(qts, CPM, c->clocks[0]);
    qtest_writel(qts, CPM + c->gate, gr & ~(1u << c->gate_bit));
    qtest_writel(qts, CPM + c->cdr, 1u << 29 | 0xff);
    return qts;
}

/* Channel n: prescale 7, the counts, init and idle levels; ENC first */
static void setup(QTestState *qts, const PwmCase *c, unsigned n,
                  uint16_t high, uint16_t low, unsigned init, unsigned idle)
{
    uint32_t b = c->base;
    uint32_t lvl = qtest_readl(qts, b + PWM_LVL) & ~(0x10001u << n);

    qtest_writel(qts, b + PWM_ENC, 1u << n);
    qtest_writel(qts, b + (n < 8 ? PWM_CCFG0 : PWM_CCFG1), 7u << (n % 8 * 4));
    qtest_writel(qts, b + PWM_LVL, lvl | init << n | idle << (n + 16));
    qtest_writel(qts, b + PWM_WCFG(n), (uint32_t)high << 16 | low);
}

/* Equal but for the nanoseconds each rounds away */
static void assert_near(uint64_t a, uint64_t b)
{
    g_assert_cmpuint(a + 4, >=, b);
    g_assert_cmpuint(b + 4, >=, a);
}

static void check_regs(QTestState *qts, const PwmCase *c)
{
    uint32_t b = c->base;
    uint32_t chmask = (1u << c->channels) - 1;
    struct {
        uint32_t off;
        uint32_t keep;
    } rw[] = {
        { PWM_CCFG0, 0xffffffff },
        { PWM_CCFG1, c->channels > 8 ? 0xffffffff : 0 },
        { PWM_LVL, 0xffffffff },
        { PWM_WCFG(3), 0xffffffff },
        { PWM_WCFG(4), c->channels > 4 ? 0xffffffff : 0 },
        { PWM_WCFG(15), c->channels > 15 ? 0xffffffff : 0 },
        { PWM_DCR0, 0xffff },
        { PWM_DINTC, 0xffff },
        { PWM_DAR(3), 0xffffffff },
        { PWM_DAR(4), c->channels > 4 ? 0xffffffff : 0 },
        { PWM_DTLR(3), 0xffff },
        { PWM_OEN, 0xffff },
        { PWM_EN, 0 },
        { PWM_BUSY, 0 },
        { PWM_DE, 0 },
        { 0x08, 0 },
        { 0x2c, 0 },
        { 0x80, 0 },
        { 0x200, 0 },
        { 0x304, 0 },
    };

    g_assert_cmphex(qtest_readl(qts, b + PWM_DINTC), ==, 0xffff);
    g_assert_cmphex(qtest_readl(qts, b + PWM_DFER), ==, chmask);
    for (int i = 0; i < ARRAY_SIZE(rw); i++) {
        qtest_writel(qts, b + rw[i].off, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, b + rw[i].off), ==, rw[i].keep);
        qtest_writel(qts, b + rw[i].off, 0);
    }
    qtest_writel(qts, b + PWM_DES, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, b + PWM_DE), ==, chmask);
    qtest_writel(qts, b + PWM_UP, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, chmask);
    g_assert_cmphex(qtest_readl(qts, b + PWM_UP), ==, 0);

    /* the 4 KiB window repeats through 64 KiB */
    qtest_writel(qts, b + PWM_CCFG0, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, b + 0x1000 + PWM_CCFG0), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, b + 0xf000 + PWM_CCFG0), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, b + 0x400 + PWM_CCFG0), ==, 0);
}

static void test_regs(const void *data)
{
    const PwmCase *c = data;
    QTestState *qts = start(c, "");

    check_regs(qts, c);
    qtest_quit(qts);
}

/* Steps to half a tick into tick @k of a period started now */
static int64_t at(int64_t tick, unsigned k, int64_t *now)
{
    int64_t want = k * tick + tick / 2;
    int64_t step = want - *now;

    *now = want;
    return step;
}

static void test_wave(const void *data)
{
    const PwmCase *c = data;
    QTestState *qts = start(c, "");
    uint32_t b = c->base;
    unsigned n = c->channels - 1;

    for (unsigned init = 0; init < 2; init++) {
        int64_t tick, now = 0;

        setup(qts, c, n, 1, 3, init, !init);
        g_assert_cmpuint(out(qts, n, "level"), ==, !init);
        qtest_writel(qts, b + PWM_ENS, 1u << n);
        tick = out(qts, n, "period-ns") / 4;
        assert_near(out(qts, n, "high-ns"), tick);
        /* init 0: low for its 3 ticks, then high; init 1: high for 1 */
        for (unsigned k = 0; k < 8; k++) {
            unsigned first = init ? 1 : 3;

            qtest_clock_step(qts, at(tick, k, &now));
            g_assert_cmpuint(out(qts, n, "level"), ==,
                             (k % 4 < first) ? init : !init);
        }

        /* ENC: EN clears, the period runs out, then the idle level */
        qtest_clock_step(qts, at(tick, 9, &now));
        qtest_writel(qts, b + PWM_ENC, 1u << n);
        g_assert_cmphex(qtest_readl(qts, b + PWM_EN), ==, 0);
        qtest_clock_step(qts, at(tick, 11, &now));
        g_assert_cmpuint(out(qts, n, "period-ns"), !=, 0);
        qtest_clock_step(qts, at(tick, 12, &now));
        g_assert_cmpuint(out(qts, n, "level"), ==, !init);
        g_assert_cmpuint(out(qts, n, "period-ns"), ==, 0);
    }
    qtest_quit(qts);
}

static void test_update(const void *data)
{
    const PwmCase *c = data;
    QTestState *qts = start(c, "");
    uint32_t b = c->base;
    int64_t tick, now = 0;

    setup(qts, c, 0, 1000, 3000, 0, 0);
    qtest_writel(qts, b + PWM_ENS, 1);
    tick = out(qts, 0, "period-ns") / 4000;
    /* EN reads what ENS and ENC leave it */
    qtest_writel(qts, b + PWM_EN, 0);
    g_assert_cmphex(qtest_readl(qts, b + PWM_EN), ==, 1);
    /* new counts and init level wait for PWMUPDATE */
    qtest_writel(qts, b + PWM_WCFG(0), 3000 << 16 | 1000);
    qtest_writel(qts, b + PWM_LVL, 1);
    qtest_clock_step(qts, at(tick, 4000 + 10, &now));
    assert_near(out(qts, 0, "high-ns"), out(qts, 0, "period-ns") / 4);
    g_assert_cmpuint(out(qts, 0, "level"), ==, 0);
    qtest_writel(qts, b + PWM_UP, 1);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 1);
    qtest_clock_step(qts, at(tick, 7990, &now));
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 1);
    g_assert_cmpuint(out(qts, 0, "level"), ==, 1);
    qtest_clock_step(qts, at(tick, 8010, &now));
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 0);
    g_assert_cmpuint(out(qts, 0, "level"), ==, 1);
    assert_near(out(qts, 0, "high-ns"), out(qts, 0, "period-ns") / 4 * 3);
    qtest_clock_step(qts, at(tick, 8000 + 3010, &now));
    g_assert_cmpuint(out(qts, 0, "level"), ==, 0);

    /* a stopped channel's BUSY waits until it has run a period */
    qtest_writel(qts, b + PWM_UP, 2);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 2);
    setup(qts, c, 1, 2, 2, 0, 0);
    qtest_writel(qts, b + PWM_ENS, 2);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 2);
    qtest_clock_step(qts, 5 * tick);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 0);
    qtest_quit(qts);
}

static void test_zero(const void *data)
{
    const PwmCase *c = data;
    QTestState *qts = start(c, "");
    uint32_t b = c->base;
    static const uint16_t counts[][2] = { { 0, 4 }, { 4, 0 }, { 0, 0 } };
    int64_t tick;

    setup(qts, c, 0, 1, 3, 0, 0);
    qtest_writel(qts, b + PWM_ENS, 1);
    tick = out(qts, 0, "period-ns") / 4;
    qtest_writel(qts, b + PWM_ENC, 1);
    qtest_clock_step(qts, 4 * tick);

    /* either count 0: the init level throughout */
    for (int i = 0; i < ARRAY_SIZE(counts); i++) {
        for (unsigned init = 0; init < 2; init++) {
            setup(qts, c, 1, counts[i][0], counts[i][1], init, !init);
            qtest_writel(qts, b + PWM_ENS, 2);
            for (int k = 0; k < 6; k++) {
                g_assert_cmpuint(out(qts, 1, "level"), ==, init);
                qtest_clock_step(qts, tick);
            }
            g_assert_cmpuint(out(qts, 1, "period-ns"), ==, 0);
            qtest_writel(qts, b + PWM_ENC, 2);
            qtest_clock_step(qts, (1 << 17) * tick);
        }
    }

    /* both 0: a period of 2^17 ticks, as 65535 each */
    setup(qts, c, 1, 0, 0, 0, 0);
    qtest_writel(qts, b + PWM_ENS, 2);
    qtest_writel(qts, b + PWM_UP, 2);
    qtest_clock_step(qts, ((1 << 17) - 10) * tick);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 2);
    qtest_clock_step(qts, 20 * tick);
    g_assert_cmphex(qtest_readl(qts, b + PWM_BUSY), ==, 0);

    /* gated, the clock stops and so does the waveform */
    setup(qts, c, 2, 1, 3, 0, 0);
    qtest_writel(qts, b + PWM_ENS, 4);
    g_assert_cmpuint(out(qts, 2, "period-ns"), !=, 0);
    qtest_writel(qts, CPM + c->gate,
                 qtest_readl(qts, CPM + c->gate) | 1u << c->gate_bit);
    g_assert_cmpuint(out(qts, 2, "period-ns"), ==, 0);
    qtest_quit(qts);
}

static void test_trace(void)
{
    g_autofree char *path = NULL;
    g_autofree char *text = NULL;
    g_autofree char *args = NULL;
    g_auto(GStrv) lines = NULL;
    QTestState *qts;
    int fd = g_file_open_tmp("pwm-trace-XXXXXX", &path, NULL);

    g_assert(fd >= 0);
    close(fd);
    args = g_strdup_printf("-chardev file,id=pt,path=%s "
                           "-global ingenic-pwm.trace=pt", path);
    qts = start(&t41, args);
    setup(qts, &t41, 0, 1, 3, 0, 1);
    qtest_writel(qts, t41.base + PWM_ENS, 1);
    qtest_writel(qts, t41.base + PWM_ENC, 1);
    qtest_clock_step(qts, 1000000);
    qtest_quit(qts);

    g_assert(g_file_get_contents(path, &text, NULL, NULL));
    unlink(path);
    lines = g_strsplit(text, "\n", -1);
    /* the reset, its 16 channels at rest, then channel 0's three changes */
    g_assert_cmpuint(g_strv_length(lines), ==, 1 + 16 + 3 + 1);
    g_assert(strstr(lines[0], "\"reset\":true,\"channels\":16"));
    g_assert(strstr(lines[17], "\"pwm\":0,\"lvl\":1,\"src\":\"hw\""));
    g_assert(strstr(lines[18], "\"pwm\":0,\"lvl\":0,\"period\":"));
    g_assert(strstr(lines[19], "\"pwm\":0,\"lvl\":1,\"src\":\"hw\""));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_data_func("ingenic-pwm/t41/regs", &t41, test_regs);
    qtest_add_data_func("ingenic-pwm/a1/regs", &a1, test_regs);
    qtest_add_data_func("ingenic-pwm/t32/regs", &t32, test_regs);
    qtest_add_data_func("ingenic-pwm/t41/wave", &t41, test_wave);
    qtest_add_data_func("ingenic-pwm/t32/wave", &t32, test_wave);
    qtest_add_data_func("ingenic-pwm/t41/update", &t41, test_update);
    qtest_add_data_func("ingenic-pwm/a1/update", &a1, test_update);
    qtest_add_data_func("ingenic-pwm/t41/zero", &t41, test_zero);
    qtest_add_data_func("ingenic-pwm/t32/zero", &t32, test_zero);
    qtest_add_func("ingenic-pwm/trace", test_trace);
    return g_test_run();
}
