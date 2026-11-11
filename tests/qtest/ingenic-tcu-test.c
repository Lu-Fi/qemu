/*
 * QTest testcase for the Ingenic TCU's channels and flag mask
 *
 * TMR out of reset as each part reads it, and the channels each part has: a
 * T31X takes writes to channel 3's registers and reads 0 from channel 4's
 * and 7's after writing them, and TMSR and TMCR move only the bits it has.
 * Where TMR has bit 24, HFLAGW, a watchdog ping (TCSR bit 10, the counter
 * clear) sets TFR's bit 24, as a T41NQ's does. The OST counts while TER's
 * bit 15 is set, TSR's clear and OSTCSR picks a clock, and holds
 * otherwise; TFR's bit 15 sets where its count meets OSTDR, as at U-Boot's
 * start of it from 0 with OSTDR 0. The T32's,
 * T41's and A1's channels read TDFR 0 out of reset, as at their bootroms.
 *
 * A channel's PWM output as a T31X's pad shows it at U-Boot: TDFR + 1
 * ticks a period, INITL from 0 to TDHR and the other level above, set by
 * the matches (the half one as the count leaves TDHR) and moved by nothing
 * else; INITL with PWM_EN clear. TECR without SD runs on to the full match
 * and halts at 0, with SD it freezes, as TSSR does. TER and TSR drop
 * writes.
 *
 * The watchdog resets the chip as TCNT counts up to TDR (the T21 manual):
 * on a T21N, T23N, T30X, T31X and T40N at U-Boot a TDR written below the
 * count waits for the 16-bit counter to wrap; a T32LQ, T33L and T41NQ reset
 * at their next count, which on a T33L a clear written straight after
 * prevents and on a T32LQ does not. TSSR's WDTS stops the count, TSCR's
 * starts it again, and a TCSR that picks no clock leaves it standing.
 * RTC_EN counts EXCLK/512 (a T31X, T33L, T40N and T41NQ against wall
 * time). The T32's and T33's TCNT takes no write, and their TCSR has no
 * EXTAL input.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/host-utils.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define TCU             0x10002000
#define CPM_RSR         0x10000008
#define TCU_TER         0x10
#define TCU_TESR        0x14
#define TCU_TECR        0x18
#define TCU_TFR         0x20
#define TCU_OST_CNTL    0xe4
#define TCU_OST_CSR     0xec
#define TCU_TFCR        0x28
#define TCU_TMR         0x30
#define TCU_TMSR        0x34
#define TCU_TMCR        0x38
#define TCU_TSR         0x1c
#define TCU_TSSR        0x2c
#define TCU_TSCR        0x3c
#define TCU_TDFR(n)     (0x40 + (n) * 0x10)
#define TCU_TDHR(n)     (0x44 + (n) * 0x10)
#define TCU_TCNT(n)     (0x48 + (n) * 0x10)
#define TCU_TCSR(n)     (0x4c + (n) * 0x10)

#define TICK            21845334        /* RTC (EXCLK/512) over 1024, up */
/* n of those ticks, to the ns */
#define TICKS_NS(n)     ((n) * 1024ull * 1000000000 / 46875)
#define CSR_SLOW        (1u << 1 | 5u << 3)
#define CSR_PWM_EN      (1u << 7)
#define CSR_INITL       (1u << 8)
#define CSR_SD          (1u << 9)

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    uint32_t tmr;
} TcuCase;

static const TcuCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", 0x000f800f },
    { "t20x", "ingenic-t20", "ingenic-t31", 0x000f800f },
    { "t21n", "ingenic-t21", "ingenic-t31", 0x00ff80ff },
    { "t23n", "ingenic-t23", "ingenic-t31", 0x000f800f },
    { "t30x", "ingenic-t30", "ingenic-t31", 0x00ff80ff },
    { "t31x", "ingenic-t31", "ingenic-t31", 0x000f800f },
    { "t32lq", "ingenic-t32", "ingenic-t31", 0x01ff80ff },
    { "t40n", "ingenic-t40", "ingenic-t40", 0x00ff80ff },
    { "t41lq", "ingenic-t41", "ingenic-t40", 0x01ff80ff },
    { "a1n", "ingenic-a1", "ingenic-a1", 0x01ff80ff },
};

static void test_case(const void *data)
{
    const TcuCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);
    unsigned channels = ctz32(~c->tmr);

    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, c->tmr);
    qtest_writel(qts, TCU + TCU_TMCR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, 0);
    qtest_writel(qts, TCU + TCU_TMSR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TMR), ==, c->tmr);

    /* TDFR out of reset: 0 on the T32, T41 and A1, which have bit 24 */
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDFR(0)), ==,
                    c->tmr & 1u << 24 ? 0 : 0xffff);
    for (unsigned n = 0; n < 8; n++) {
        uint32_t want = n < channels ? 0x1234 + n : 0;

        qtest_writel(qts, TCU + TCU_TDFR(n), 0x1234 + n);
        qtest_writel(qts, TCU + TCU_TDHR(n), 0x1234 + n);
        g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDFR(n)), ==, want);
        g_assert_cmphex(qtest_readl(qts, TCU + TCU_TDHR(n)), ==, want);
    }
    qtest_writel(qts, TCU + TCU_TESR, 0xff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER), ==, (1u << channels) - 1);
    qtest_writel(qts, TCU + TCU_TECR, 0xff);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER), ==, 0);

    qtest_writel(qts, TCU + TCU_TFCR, 0xffffffff);
    qtest_writel(qts, TCU + 0x0c, 0x1a | 1 << 10);     /* WDT TCSR */
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR), ==, c->tmr & 1u << 24);
    qtest_quit(qts);
}

static void test_ost(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");
    uint32_t a, b;

    qtest_clock_step(qts, 1000000);
    a = qtest_readl(qts, TCU + TCU_OST_CNTL);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, a);

    qtest_writel(qts, TCU + TCU_OST_CSR, 0x800c);       /* as U-Boot sets it */
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, a);
    qtest_writel(qts, TCU + TCU_TESR, 1u << 15);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER) & 1u << 15, ==, 1u << 15);
    qtest_clock_step(qts, 1000000);                     /* 1 ms: 6000 ticks */
    b = qtest_readl(qts, TCU + TCU_OST_CNTL);
    g_assert_cmpuint(b - a, ==, 6000);

    qtest_writel(qts, TCU + TCU_TECR, 1u << 15);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, b);

    /* U-Boot's start: count and OSTDR 0, then enable; the flag sets */
    qtest_writel(qts, TCU + TCU_TFCR, 1u << 15);
    qtest_writel(qts, TCU + TCU_OST_CNTL, 0);
    qtest_writel(qts, TCU + TCU_TESR, 1u << 15);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 1u << 15, ==, 1u << 15);
    /* cleared, it stays so until the count meets OSTDR again */
    qtest_writel(qts, TCU + TCU_TFCR, 1u << 15);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 1u << 15, ==, 0);
    qtest_writel(qts, TCU + 0xe0, 9000);                /* OSTDR, 1.5 ms */
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 1u << 15, ==, 1u << 15);
    /* a stopped OST holds */
    qtest_writel(qts, TCU + 0x2c, 1u << 15);           /* TSSR */
    a = qtest_readl(qts, TCU + TCU_OST_CNTL);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_OST_CNTL), ==, a);
    qtest_quit(qts);
}

static uint64_t qom_uint(QTestState *qts, const char *path, const char *prop)
{
    QDict *r = qtest_qmp(qts, "{ 'execute': 'qom-get', 'arguments': "
                         "{ 'path': %s, 'property': %s } }", path, prop);
    uint64_t v;

    g_assert(qdict_haskey(r, "return"));
    v = qdict_get_int(r, "return");
    qobject_unref(r);
    return v;
}

static unsigned level(QTestState *qts)
{
    return qom_uint(qts, "/machine/soc/tcu", "pwm0-level");
}

/* Channel 0 at TDFR 8 and TDHR 3 from 0, half a tick in */
static void run(QTestState *qts, uint32_t csr)
{
    qtest_writel(qts, TCU + TCU_TCSR(0), csr);
    qtest_writel(qts, TCU + TCU_TDFR(0), 8);
    qtest_writel(qts, TCU + TCU_TDHR(0), 3);
    qtest_writel(qts, TCU + TCU_TCNT(0), 0);
    qtest_writel(qts, TCU + TCU_TFCR, 0x10001);
    qtest_writel(qts, TCU + TCU_TCSR(0), csr | CSR_PWM_EN);
    qtest_writel(qts, TCU + TCU_TESR, 1);
    qtest_clock_step(qts, TICK / 2);
}

static void test_pwm(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    for (unsigned initl = 0; initl < 2; initl++) {
        uint32_t csr = CSR_SLOW | (initl ? CSR_INITL : 0);

        qtest_writel(qts, TCU + TCU_TCSR(0), csr);
        g_assert_cmpuint(level(qts), ==, initl);
        run(qts, csr);
        for (unsigned i = 0; i < 18; i++) {
            uint32_t tfr = qtest_readl(qts, TCU + TCU_TFR);

            g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, i % 9);
            g_assert_cmpuint(level(qts), ==, (i % 9 > 3) ^ initl);
            g_assert_cmphex(tfr & 1u << 16, ==, i >= 4 ? 1u << 16 : 0);
            g_assert_cmphex(tfr & 1, ==, i >= 9);
            qtest_clock_step(qts, TICK);
        }
        g_assert_cmpuint(qom_uint(qts, "/machine/soc/tcu", "pwm0-period-ns"),
                         ==, TICKS_NS(9));
        g_assert_cmpuint(qom_uint(qts, "/machine/soc/tcu", "pwm0-high-ns"),
                         ==, TICKS_NS(initl ? 4 : 5));
        qtest_writel(qts, TCU + TCU_TCSR(0), csr | CSR_SD | CSR_PWM_EN);
        qtest_writel(qts, TCU + TCU_TECR, 1);
    }

    /* stopped at a count above TDHR: TCNT and INITL writes move nothing */
    run(qts, CSR_SLOW | CSR_SD);
    qtest_clock_step(qts, 5 * TICK);
    qtest_writel(qts, TCU + TCU_TECR, 1);
    g_assert_cmpuint(level(qts), ==, 1);
    qtest_writel(qts, TCU + TCU_TCNT(0), 1);
    g_assert_cmpuint(level(qts), ==, 1);
    qtest_writel(qts, TCU + TCU_TCSR(0),
                 CSR_SLOW | CSR_SD | CSR_PWM_EN | CSR_INITL);
    g_assert_cmpuint(level(qts), ==, 1);
    /* PWM_EN clear: INITL */
    qtest_writel(qts, TCU + TCU_TCSR(0), CSR_SLOW | CSR_SD);
    g_assert_cmpuint(level(qts), ==, 0);

    /* TDHR past TDFR: INITL throughout, no half match */
    run(qts, CSR_SLOW);
    qtest_writel(qts, TCU + TCU_TDHR(0), 10);
    qtest_writel(qts, TCU + TCU_TFCR, 0x10001);
    for (unsigned i = 0; i < 12; i++) {
        g_assert_cmpuint(level(qts), ==, 0);
        qtest_clock_step(qts, TICK);
    }
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TFR) & 0x10001, ==, 1);
    g_assert_cmpuint(qom_uint(qts, "/machine/soc/tcu", "pwm0-period-ns"), ==,
                     0);
    qtest_quit(qts);
}

static void test_stop(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    /* TECR without SD: on to the full match, then 0 at INITL */
    run(qts, CSR_SLOW);
    qtest_clock_step(qts, 5 * TICK);
    qtest_writel(qts, TCU + TCU_TECR, 1);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER) & 1, ==, 0);
    qtest_clock_step(qts, 3 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 8);
    g_assert_cmpuint(level(qts), ==, 1);
    qtest_clock_step(qts, TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 0);
    g_assert_cmpuint(level(qts), ==, 0);
    qtest_clock_step(qts, 4 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 0);
    qtest_writel(qts, TCU + TCU_TESR, 1);
    qtest_clock_step(qts, 4 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 4);
    g_assert_cmpuint(level(qts), ==, 1);

    /* with SD: frozen at once, and TESR carries on */
    run(qts, CSR_SLOW | CSR_SD);
    qtest_clock_step(qts, 5 * TICK);
    qtest_writel(qts, TCU + TCU_TECR, 1);
    qtest_clock_step(qts, 10 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 5);
    g_assert_cmpuint(level(qts), ==, 1);
    qtest_writel(qts, TCU + TCU_TESR, 1);
    qtest_clock_step(qts, 4 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 0);
    g_assert_cmpuint(level(qts), ==, 0);

    /* TSSR freezes, TSCR carries on; TER and TSR drop writes */
    qtest_clock_step(qts, 6 * TICK);
    qtest_writel(qts, TCU + TCU_TSSR, 1);
    qtest_clock_step(qts, 10 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 6);
    g_assert_cmpuint(level(qts), ==, 1);
    qtest_writel(qts, TCU + TCU_TSR, 0);
    qtest_writel(qts, TCU + TCU_TER, 0);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TSR) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TER) & 1, ==, 1);
    qtest_writel(qts, TCU + TCU_TSCR, 1);
    qtest_clock_step(qts, 3 * TICK);
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==, 0);
    qtest_quit(qts);
}

/* The half match's interrupt, as the count leaves TDHR */
static void test_irq(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    qtest_irq_intercept_out_named(qts, "/machine/soc/tcu", "sysbus-irq");
    qtest_writel(qts, TCU + TCU_TMCR, 1u << 16);
    run(qts, CSR_SLOW);
    qtest_clock_step(qts, 3 * TICK);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, TICK);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_writel(qts, TCU + TCU_TFCR, 1u << 16);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, 8 * TICK);
    g_assert_false(qtest_get_irq(qts, 0));
    qtest_clock_step(qts, TICK);
    g_assert_true(qtest_get_irq(qts, 0));
    qtest_quit(qts);
}

/* A channel at 24 MHz keeps count past the 768 s where ns times Hz wraps */
static void test_long(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    qtest_writel(qts, TCU + TCU_TCSR(0), 1u << 2);
    qtest_writel(qts, TCU + TCU_TDFR(0), 0xffff);
    qtest_writel(qts, TCU + TCU_TCNT(0), 0);
    qtest_writel(qts, TCU + TCU_TESR, 1);
    qtest_clock_step(qts, 1000000000000ll);         /* 1000 s */
    g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(0)), ==,
                     24000000000ull % 0x10000);
    qtest_quit(qts);
}

/* The watchdog, 1 ms on the 24 MHz clock: run to its match */
static QTestState *wdt_expire(const char *extra)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 %s", extra);

    qtest_writel(qts, TCU + 0x0c, 1 << 2);         /* WDT TCSR: EXCLK */
    qtest_writel(qts, TCU + 0x00, 24000);          /* WDT TDR */
    qtest_writel(qts, TCU + 0x04, 1);              /* WDT TCER: on */
    qtest_clock_step(qts, 2000000);
    return qts;
}

/* A reset unless -action says otherwise, which then leaves it running */
static void test_wdt(void)
{
    QTestState *qts = wdt_expire("");

    qtest_qmp_eventwait(qts, "RESET");
    qtest_quit(qts);

    qts = wdt_expire("-action watchdog=none");
    qtest_qmp_eventwait(qts, "WATCHDOG");
    g_assert_cmphex(qtest_readl(qts, TCU + 0x00), ==, 24000);
    qtest_quit(qts);
}

/* The watchdog on EXCLK/1, TDR 0xffff, run for 1 ms: 24000 counts */
static QTestState *wdt_count_on(const char *machine)
{
    QTestState *qts = qtest_initf("-machine %s", machine);

    qtest_writel(qts, TCU + 0x0c, 1 << 2);         /* WDT TCSR: EXCLK */
    qtest_writel(qts, TCU + 0x00, 0xffff);         /* WDT TDR */
    qtest_writel(qts, TCU + 0x04, 1);              /* WDT TCER: on */
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 24000);
    return qts;
}

static QTestState *wdt_count(void)
{
    return wdt_count_on("ingenic-t31");
}

/* The watchdog on RTC_EN/1, TDR 0xffff, run for 10 ms: @counts counts */
static QTestState *wdt_rtc_on(const char *machine, uint32_t counts)
{
    QTestState *qts = qtest_initf("-machine %s", machine);

    qtest_writel(qts, TCU + 0x0c, 1 << 1);         /* WDT TCSR: RTC */
    qtest_writel(qts, TCU + 0x00, 0xffff);         /* WDT TDR */
    qtest_writel(qts, TCU + 0x04, 1);              /* WDT TCER: on */
    qtest_clock_step(qts, 10000000);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, counts);
    return qts;
}

/*
 * A T32's: reset at its next count once TDR is written below it, a clear
 * written straight after too late, as a T32LQ's is at U-Boot
 */
static void test_wdt_above(void)
{
    QTestState *qts = wdt_rtc_on("ingenic-t32", 468);

    qtest_writel(qts, TCU + 0x00, 300);
    qtest_writel(qts, TCU + 0x0c, 1 << 1 | 1 << 10);
    qtest_clock_step(qts, 100000);
    qtest_qmp_eventwait(qts, "RESET");
    qtest_quit(qts);
}

/*
 * A T33's, as a T33L's at U-Boot: reset at its next count once TDR is below
 * it, but a clear written straight after lands first, as its vendor kernel
 * relies on at its start of the watchdog. TCNT takes no write and TCSR has
 * no EXTAL input, as on a T32.
 */
static void test_wdt_t33(void)
{
    QTestState *qts = wdt_rtc_on("ingenic-t33", 468);

    qtest_writel(qts, TCU + 0x08, 0);              /* ignored */
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 468);
    qtest_writel(qts, TCU + 0x00, 300);
    qtest_writel(qts, TCU + 0x0c, 1 << 1 | 1 << 10);
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x04), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 46);
    qtest_writel(qts, TCU + 0x0c, 1 << 2 | 1 << 1);
    g_assert_cmphex(qtest_readl(qts, TCU + 0x0c), ==, 1 << 1);
    qtest_writel(qts, TCU + 0x00, 10);
    qtest_clock_step(qts, 100000);
    qtest_qmp_eventwait(qts, "RESET");
    qtest_quit(qts);
}

/* RTC_EN counts EXCLK/512 on each family: 10 ms of a T31, T33 and T41 */
static void test_wdt_rtc(void)
{
    static const struct {
        const char *machine;
        uint32_t counts;
    } parts[] = {
        { "ingenic-t31", 468 }, { "ingenic-t33", 468 }, { "ingenic-t41", 468 },
    };

    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        QTestState *qts = qtest_initf("-machine %s", parts[i].machine);

        qtest_writel(qts, TCU + 0x0c, 1 << 1);     /* WDT TCSR: RTC */
        qtest_writel(qts, TCU + 0x00, 0xffff);
        qtest_writel(qts, TCU + 0x04, 1);
        qtest_clock_step(qts, 10000000);
        g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, parts[i].counts);
        qtest_quit(qts);
    }
}

/* A channel on RTC_EN counts EXCLK/512, as a T20X's, T31X's and T40N's */
static void test_chn_rtc(void)
{
    static const char *const machines[] = { "ingenic-t31", "ingenic-t40" };

    for (int i = 0; i < ARRAY_SIZE(machines); i++) {
        QTestState *qts = qtest_initf("-machine %s", machines[i]);

        qtest_writel(qts, TCU + TCU_TSCR, 1 << 3);
        qtest_writel(qts, TCU + TCU_TCSR(3), 1 << 1);  /* RTC, /1 */
        qtest_writel(qts, TCU + TCU_TDFR(3), 0xffff);
        qtest_writel(qts, TCU + TCU_TDHR(3), 0xffff);
        qtest_writel(qts, TCU + TCU_TCNT(3), 0);
        qtest_writel(qts, TCU + TCU_TESR, 1 << 3);
        qtest_clock_step(qts, 10000000);
        g_assert_cmpuint(qtest_readl(qts, TCU + TCU_TCNT(3)), ==, 468);
        qtest_quit(qts);
    }
}

/*
 * TCER cleared: a T31's and T40's count runs on (only the reset is off),
 * a T32's stands
 */
static void test_wdt_tcer(void)
{
    static const struct {
        const char *machine;
        uint32_t after;
    } parts[] = {
        { "ingenic-t31", 937 }, { "ingenic-t40", 937 }, { "ingenic-t32", 468 },
    };

    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        QTestState *qts = wdt_rtc_on(parts[i].machine, 468);

        qtest_writel(qts, TCU + 0x04, 0);
        qtest_clock_step(qts, 10000000);
        g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, parts[i].after);
        qtest_quit(qts);
    }
}

/*
 * RSR: 0x1 from power-on; a watchdog reset adds 0x2 and keeps the rest,
 * which stays until written clear, as a T20X, T31X and T40N read it
 */
static void test_wdt_rsr(void)
{
    static const char *const machines[] = { "ingenic-t31", "ingenic-t40" };

    for (int i = 0; i < ARRAY_SIZE(machines); i++) {
        QTestState *qts = qtest_initf("-machine %s", machines[i]);

        g_assert_cmphex(qtest_readl(qts, CPM_RSR), ==, 1);
        for (int k = 0; k < 2; k++) {
            qtest_writel(qts, TCU + 0x0c, 1 << 1);   /* WDT TCSR: RTC */
            qtest_writel(qts, TCU + 0x00, 10);
            qtest_writel(qts, TCU + 0x08, 0);
            qtest_writel(qts, TCU + 0x04, 1);
            qtest_clock_step(qts, 1000000);
            qtest_qmp_eventwait(qts, "RESET");
            g_assert_cmphex(qtest_readl(qts, CPM_RSR), ==, k ? 2 : 3);
            qtest_writel(qts, CPM_RSR, 0);
        }
        qtest_quit(qts);
    }
}

/* A TDR below the count: no reset until the counter wraps round to it */
static void test_wdt_wrap(void)
{
    QTestState *qts = wdt_count();

    qtest_writel(qts, TCU + 0x00, 1000);
    qtest_clock_step(qts, 1600000);                /* to 62400 */
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x04), ==, 1);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 62400);
    qtest_clock_step(qts, 200000);                 /* past 0xffff to 1000 */
    qtest_qmp_eventwait(qts, "RESET");
    qtest_quit(qts);
}

/* WDTS stops the count where it is; TCSR without a clock does too */
static void test_wdt_stop(void)
{
    QTestState *qts = wdt_count();

    qtest_writel(qts, TCU + TCU_TSSR, 1u << 16);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TSR) & 1u << 16, ==, 1u << 16);
    qtest_clock_step(qts, 5000000);                /* past TDR, if it ran */
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 24000);
    qtest_writel(qts, TCU + TCU_TSCR, 1u << 16);
    g_assert_cmphex(qtest_readl(qts, TCU + TCU_TSR) & 1u << 16, ==, 0);
    qtest_clock_step(qts, 1000000);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 48000);
    qtest_writel(qts, TCU + 0x0c, 0);
    qtest_clock_step(qts, 5000000);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x08), ==, 48000);
    g_assert_cmpuint(qtest_readl(qts, TCU + 0x04), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-tcu/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    qtest_add_func("ingenic-tcu/ost", test_ost);
    qtest_add_func("ingenic-tcu/pwm", test_pwm);
    qtest_add_func("ingenic-tcu/stop", test_stop);
    qtest_add_func("ingenic-tcu/irq", test_irq);
    qtest_add_func("ingenic-tcu/long", test_long);
    qtest_add_func("ingenic-tcu/wdt", test_wdt);
    qtest_add_func("ingenic-tcu/wdt-wrap", test_wdt_wrap);
    qtest_add_func("ingenic-tcu/wdt-stop", test_wdt_stop);
    qtest_add_func("ingenic-tcu/wdt-above", test_wdt_above);
    qtest_add_func("ingenic-tcu/wdt-t33", test_wdt_t33);
    qtest_add_func("ingenic-tcu/wdt-rtc", test_wdt_rtc);
    qtest_add_func("ingenic-tcu/chn-rtc", test_chn_rtc);
    qtest_add_func("ingenic-tcu/wdt-tcer", test_wdt_tcer);
    qtest_add_func("ingenic-tcu/wdt-rsr", test_wdt_rsr);
    return g_test_run();
}
