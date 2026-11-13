/*
 * QTest testcase for the Ingenic RTC
 *
 * The power-on values as each part reads them, and the write lock as a
 * T41NQ takes it: 0xa55a in WENR opens the protected registers for one
 * write, any other WENR write closes them, and a locked write changes
 * only RTCCR's SELEXC and 1HZIE. The rest of a locked RTCCR write waits,
 * as a T30X running its RTC on EXCLK / 512 shows: WRDY reads 0 and RTCCR
 * what was written until a WENR write drops it (the T41NQ's RTC has no
 * clock and reads WRDY set). On that T30X too: the 1 Hz flag each second
 * while RTCE is on, cleared by a written 0, kept by a written 1; the RTC
 * interrupt as AF and AIE or 1HZ and 1HZIE; RTCSR stopped by RTCE off.
 * These cases wait on real seconds: the RTC runs on host time.
 * The T40 has none to see: a T40XP and a
 * T40NN read 0 at 0x10003000 and at its kernel's RTC_IOBASE, 0x132a0000.
 * The A1's is its own: VERSION 0x20210425, CLKCNTEN's bit 0 starting
 * CLKCNTINFO at EXTCLK / 512, and the rest of the page 0, as an A1N reads.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define RTC             0x10003000
#define RTCCR           0x00
#define HRCR            0x28
#define HWCR            0x2c
#define HWRSR           0x30
#define HSPR            0x34
#define WENR            0x3c
#define WKUPPINCR       0x48

#define RTCSR           0x04
#define RTCCR_WRDY      0x80
#define RTCCR_1HZ       0x40
#define RTCCR_HZIE      0x20
#define RTCCR_SELEXC    0x02
#define RTCCR_RTCE      0x01
#define ICSR1           0x10001020  /* the RTC is source 32 */
#define HWRSR_PPR       0x10
#define WENR_WEN        0x80000000u
#define WENR_PAT        0xa55a

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    uint32_t wkuppincr, hrcr;
} RtcCase;

static const RtcCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", 0x00000064, 0x1000 },
    { "t20x", "ingenic-t20", "ingenic-t31", 0x00010064, 0x800 },
    { "t21n", "ingenic-t21", "ingenic-t31", 0x00010064, 0x800 },
    { "t30x", "ingenic-t30", "ingenic-t31", 0x00010064, 0x800 },
    { "t31x", "ingenic-t31", "ingenic-t31", 0x00010064, 0x800 },
    { "t32lq", "ingenic-t32", "ingenic-t31", 0x00210064, 0x800 },
    { "t41nq", "ingenic-t41", "ingenic-t40", 0x00a30064, 0x800 },
};

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, RTC + reg);
}

static void wr(QTestState *qts, uint32_t reg, uint32_t value)
{
    qtest_writel(qts, RTC + reg, value);
}

static void test_power_on(const void *data)
{
    const RtcCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    /* 1HZ follows the host's seconds, so one may have passed since realize */
    g_assert_cmphex(rd(qts, RTCCR) & ~RTCCR_1HZ, ==, RTCCR_WRDY | RTCCR_RTCE);
    g_assert_cmphex(rd(qts, HWCR), ==, 0x2d52d2d0);
    g_assert_cmphex(rd(qts, HWRSR), ==, HWRSR_PPR);
    g_assert_cmphex(rd(qts, WENR), ==, 0);
    g_assert_cmphex(rd(qts, WKUPPINCR), ==, c->wkuppincr);
    g_assert_cmphex(rd(qts, HRCR), ==, c->hrcr);
    qtest_quit(qts);
}

static void test_t40_none(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    for (uint32_t off = 0; off < 0x50; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x132a0000 + off), ==, 0);
        g_assert_cmphex(qtest_readl(qts, 0x10003000 + off), ==, 0);
    }
    qtest_quit(qts);
}

static void test_a1(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    uint32_t a;

    g_assert_cmphex(qtest_readl(qts, 0x10003000), ==, 0x20210425);
    qtest_writel(qts, 0x10003000, 0);
    g_assert_cmphex(qtest_readl(qts, 0x10003000), ==, 0x20210425);
    g_assert_cmphex(qtest_readl(qts, 0x10003018), ==, 0);
    qtest_writel(qts, 0x10003010, 3);
    g_assert_cmphex(qtest_readl(qts, 0x10003010), ==, 1);
    a = qtest_readl(qts, 0x10003018);
    qtest_clock_step(qts, 1000000000);
    g_assert_cmpuint(qtest_readl(qts, 0x10003018) - a, ==, 46875);
    qtest_writel(qts, 0x10003010, 0);
    a = qtest_readl(qts, 0x10003018);
    qtest_clock_step(qts, 1000000000);
    g_assert_cmpuint(qtest_readl(qts, 0x10003018), ==, a);
    for (uint32_t off = 0x1c; off < 0x80; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x10003000 + off), ==, 0);
    }
    qtest_quit(qts);
}

static void test_write_lock(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    wr(qts, HSPR, 0x12345678);
    g_assert_cmphex(rd(qts, HSPR), ==, 0);
    wr(qts, WKUPPINCR, 0x00a30074);
    g_assert_cmphex(rd(qts, WKUPPINCR), ==, 0x00a30064);

    wr(qts, WENR, WENR_PAT);
    g_assert_cmphex(rd(qts, WENR), ==, WENR_WEN);
    wr(qts, HSPR, 0x12345678);
    g_assert_cmphex(rd(qts, HSPR), ==, 0x12345678);
    g_assert_cmphex(rd(qts, WENR), ==, 0);
    wr(qts, HSPR, 0x87654321);
    g_assert_cmphex(rd(qts, HSPR), ==, 0x12345678);

    wr(qts, WENR, WENR_PAT);
    wr(qts, 0x40, 0);
    g_assert_cmphex(rd(qts, WENR), ==, WENR_WEN);
    wr(qts, WENR, 0x1234);
    g_assert_cmphex(rd(qts, WENR), ==, 0);

    wr(qts, RTCCR, RTCCR_HZIE);
    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_HZIE);
    wr(qts, WENR, 0);
    g_assert_cmphex(rd(qts, RTCCR) & ~RTCCR_1HZ, ==,
                    RTCCR_WRDY | RTCCR_HZIE | RTCCR_RTCE);
    qtest_quit(qts);
}

/* One protected write, as the drivers make it */
static void wr_open(QTestState *qts, uint32_t reg, uint32_t value)
{
    wr(qts, WENR, WENR_PAT);
    wr(qts, reg, value);
}

static void wait_tick(void)
{
    g_usleep(1200 * 1000);
}

/* Just past a second's tick, so that what follows fits in one second */
static void sync_second(QTestState *qts)
{
    uint32_t sr = rd(qts, RTCSR);

    for (int i = 0; i < 150 && rd(qts, RTCSR) == sr; i++) {
        g_usleep(10 * 1000);
    }
    g_usleep(100 * 1000);
}

static void test_1hz(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint32_t base = RTCCR_SELEXC | RTCCR_RTCE;

    sync_second(qts);
    wr_open(qts, RTCCR, base);
    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_WRDY | base);
    wait_tick();
    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_WRDY | RTCCR_1HZ | base);

    /* a written 1 leaves it, a written 0 clears it */
    sync_second(qts);
    wr_open(qts, RTCCR, base);
    wr_open(qts, RTCCR, RTCCR_1HZ | base);
    g_assert_cmphex(rd(qts, RTCCR) & RTCCR_1HZ, ==, 0);

    /* the interrupt: 1HZ and 1HZIE, as a level */
    wr(qts, RTCCR, RTCCR_HZIE | RTCCR_1HZ | base);
    wr(qts, WENR, 0);
    g_assert_cmphex(qtest_readl(qts, ICSR1) & 1, ==, 0);
    wait_tick();
    g_assert_cmphex(qtest_readl(qts, ICSR1) & 1, ==, 1);
    sync_second(qts);
    wr_open(qts, RTCCR, RTCCR_HZIE | base);
    g_assert_cmphex(qtest_readl(qts, ICSR1) & 1, ==, 0);
    wait_tick();
    g_assert_cmphex(qtest_readl(qts, ICSR1) & 1, ==, 1);
    wr(qts, RTCCR, RTCCR_1HZ | base);       /* locked: 1HZIE off at once */
    g_assert_cmphex(qtest_readl(qts, ICSR1) & 1, ==, 0);
    qtest_quit(qts);
}

static void test_rtce(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint32_t sr;

    wr_open(qts, RTCCR, RTCCR_SELEXC);
    sr = rd(qts, RTCSR);
    wait_tick();
    g_assert_cmphex(rd(qts, RTCSR), ==, sr);
    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_WRDY | RTCCR_SELEXC);
    wr_open(qts, RTCCR, RTCCR_SELEXC | RTCCR_RTCE);
    wait_tick();
    g_assert_cmphex(rd(qts, RTCSR), >, sr);
    g_assert_cmphex(rd(qts, RTCCR) & RTCCR_1HZ, ==, RTCCR_1HZ);
    qtest_quit(qts);
}

/* A locked write waits, reading back as written, and a WENR write drops it */
static void test_parked(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint32_t base = RTCCR_SELEXC | RTCCR_RTCE;

    wr_open(qts, RTCCR, base);
    sync_second(qts);
    wr(qts, RTCCR, RTCCR_WRDY | base);
    g_assert_cmphex(rd(qts, RTCCR), ==, base);
    wait_tick();
    g_assert_cmphex(rd(qts, RTCCR), ==, base);
    wr(qts, WENR, WENR_PAT);
    g_assert_cmphex(rd(qts, RTCCR), ==, RTCCR_WRDY | RTCCR_1HZ | base);
    g_assert_cmphex(rd(qts, WENR), ==, WENR_WEN);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-rtc/power-on/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_power_on);
    }
    qtest_add_func("ingenic-rtc/write-lock", test_write_lock);
    qtest_add_func("ingenic-rtc/1hz", test_1hz);
    qtest_add_func("ingenic-rtc/rtce", test_rtce);
    qtest_add_func("ingenic-rtc/parked", test_parked);
    qtest_add_func("ingenic-rtc/t40-none", test_t40_none);
    qtest_add_func("ingenic-rtc/a1", test_a1);
    return g_test_run();
}
