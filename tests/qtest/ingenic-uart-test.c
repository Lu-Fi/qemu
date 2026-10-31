/*
 * QTest testcase for the Ingenic UART
 *
 * Every expected value and time here was measured on a T31X and a T41NQ
 * (UART2, in loopback): the reset state and writable bits on every
 * machine, the frame time for a range of divisor, UMR, UACR and line
 * settings, the FIFO depths and counts, the transmit request and receive
 * trigger levels, the receive timeout and its enable, overrun, the
 * receiver's minimum bit length, and UFCR.UME.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define UART1           0x10031000
#define UART2           0x10032000

#define RBR             0x00
#define IER             0x04
#define IIR             0x08
#define FCR             0x08
#define LCR             0x0c
#define MCR             0x10
#define LSR             0x14
#define MSR             0x18
#define SPR             0x1c
#define ISR             0x20
#define UMR             0x24
#define UACR            0x28
#define URCR            0x40
#define UTCR            0x44

#define IER_RDRIE       0x01
#define IER_TDRIE       0x02
#define IER_RLSIE       0x04
#define IER_RTOIE       0x10

#define FCR_FME         0x01
#define FCR_RESET       0x06
#define FCR_UME         0x10

#define MCR_LOOP        0x10

#define LSR_DR          0x01
#define LSR_OE          0x02
#define LSR_TDRQ        0x20
#define LSR_TEMT        0x40
#define LSR_FIFOE       0x80

/* 115200 8N1 from 24 MHz: divisor 13, 16 cycles a bit */
#define FRAME_NS        86666

typedef struct {
    QTestState *qts;
    uint32_t base;
} Uart;

static uint32_t rd(Uart *u, unsigned off)
{
    return qtest_readl(u->qts, u->base + off);
}

static void wr(Uart *u, unsigned off, uint32_t val)
{
    qtest_writel(u->qts, u->base + off, val);
}

static Uart start(const char *args)
{
    return (Uart) {
        .qts = qtest_initf("-machine ingenic-t31 %s", args),
        .base = UART2,
    };
}

static void setup(Uart *u, unsigned div, unsigned umr, unsigned uacr,
                  unsigned lcr, unsigned fcr, bool loop)
{
    wr(u, LCR, 0x80);
    wr(u, RBR, div & 0xff);
    wr(u, IER, div >> 8);
    wr(u, LCR, lcr);
    wr(u, UMR, umr);
    wr(u, UACR, uacr);
    wr(u, MCR, loop ? MCR_LOOP : 0);
    wr(u, FCR, fcr | FCR_RESET);
}

static void setup_115200(Uart *u, unsigned fcr)
{
    setup(u, 13, 16, 0, 0x03, fcr | FCR_FME | FCR_UME, true);
}

static void send_bytes(Uart *u, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        wr(u, RBR, 0x30 + (i & 0x3f));
    }
}

static const char *machines[] = {
    "ingenic-t10", "ingenic-t20", "ingenic-t21", "ingenic-t23",
    "ingenic-t30", "ingenic-t31", "ingenic-t32", "ingenic-t33",
    "ingenic-t40", "ingenic-t41", "ingenic-a1",
};

static void test_reset(const void *data)
{
    static const struct {
        unsigned off;
        uint32_t reset, mask;
    } regs[] = {
        { IER, 0, 0x1f }, { LCR, 0, 0xff }, { MCR, 0, 0xdf }, { SPR, 0, 0xff },
        { ISR, 0, 0x1f }, { UMR, 0x10, 0x3f }, { UACR, 0, 0xfff },
    };
    Uart u = {
        .qts = qtest_initf("-machine %s", (const char *)data),
        .base = UART1,
    };

    g_assert_cmphex(rd(&u, IIR), ==, 0x01);
    g_assert_cmphex(rd(&u, LSR), ==, 0x60);
    g_assert_cmphex(rd(&u, MSR), ==, 0);
    g_assert_cmphex(rd(&u, URCR), ==, 0);
    g_assert_cmphex(rd(&u, UTCR), ==, 0);
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        g_assert_cmphex(rd(&u, regs[i].off), ==, regs[i].reset);
    }
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        wr(&u, regs[i].off, 0xffffffff);
        g_assert_cmphex(rd(&u, regs[i].off), ==, regs[i].mask);
        wr(&u, regs[i].off, 0);
    }
    qtest_quit(u.qts);
}

typedef struct {
    unsigned div, umr, uacr, lcr;
    int64_t ns;                 /* one frame, as measured */
} FrameCase;

static const FrameCase frames[] = {
    { 13, 16, 0, 0x03, 86666 },
    { 13, 0, 0, 0x03, 346666 },         /* UMR 0 is 64 cycles */
    { 13, 3, 0, 0x03, 16250 },
    { 13, 63, 0, 0x03, 341250 },
    { 13, 8, 0xfff, 0x03, 48750 },
    { 13, 8, 0x00f, 0x03, 45500 },
    { 13, 8, 0x200, 0x03, 43875 },      /* bit 9, the stop bit */
    { 13, 8, 0x400, 0x03, 43333 },      /* past the frame */
    { 13, 8, 0x800, 0x0f, 52541 },      /* the second stop bit of 8E2 */
    { 13, 16, 0, 0x0f, 104000 },
    { 13, 16, 0, 0x00, 60666 },
    { 13, 16, 0, 0x04, 65000 },         /* 1.5 stop bits */
    { 13, 16, 0, 0x0c, 73666 },
};

static void test_frame(const void *data)
{
    const FrameCase *c = data;
    Uart u = start("");

    setup(&u, c->div, c->umr, c->uacr, c->lcr, FCR_FME | FCR_UME, false);
    send_bytes(&u, 3);
    g_assert_cmpuint(rd(&u, UTCR), ==, 2);
    qtest_clock_step(u.qts, c->ns - 1);
    g_assert_cmpuint(rd(&u, UTCR), ==, 2);
    qtest_clock_step(u.qts, 2);
    g_assert_cmpuint(rd(&u, UTCR), ==, 1);
    qtest_clock_step(u.qts, c->ns);
    g_assert_cmpuint(rd(&u, UTCR), ==, 0);
    g_assert_cmphex(rd(&u, LSR) & LSR_TEMT, ==, 0);
    qtest_clock_step(u.qts, c->ns);
    g_assert_cmphex(rd(&u, LSR) & LSR_TEMT, ==, LSR_TEMT);
    qtest_quit(u.qts);
}

/* 127 bytes fit; one more wraps the count and loses them */
static void test_tx_fifo(void)
{
    static const struct {
        unsigned burst, utcr;
    } bursts[] = { { 128, 127 }, { 129, 0 }, { 130, 1 }, { 200, 71 } };

    for (int i = 0; i < ARRAY_SIZE(bursts); i++) {
        Uart u = start("");

        setup_115200(&u, 0);
        send_bytes(&u, bursts[i].burst);
        g_assert_cmpuint(rd(&u, UTCR), ==, bursts[i].utcr);
        qtest_clock_step(u.qts, (bursts[i].utcr + 1) * FRAME_NS + 1);
        g_assert_cmpuint(rd(&u, URCR), ==, MIN(bursts[i].utcr + 1, 64));
        qtest_quit(u.qts);
    }
}

/* TDRQ, and THRI with it, is a level: 32 bytes or fewer waiting */
static void test_tdrq(void)
{
    Uart u = start("");

    qtest_irq_intercept_out_named(u.qts, "/machine/ingenic-t31-uart2",
                                  "sysbus-irq");
    setup_115200(&u, 0);
    wr(&u, IER, IER_TDRIE);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc2);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc2);
    g_assert_true(qtest_get_irq(u.qts, 0));
    send_bytes(&u, 40);
    g_assert_cmphex(rd(&u, LSR) & (LSR_TDRQ | LSR_TEMT), ==, 0);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc1);
    g_assert_false(qtest_get_irq(u.qts, 0));
    qtest_clock_step(u.qts, 6 * FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, UTCR), ==, 33);
    g_assert_cmphex(rd(&u, LSR) & LSR_TDRQ, ==, 0);
    qtest_clock_step(u.qts, FRAME_NS);
    g_assert_cmpuint(rd(&u, UTCR), ==, 32);
    g_assert_cmphex(rd(&u, LSR) & LSR_TDRQ, ==, LSR_TDRQ);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc2);
    g_assert_true(qtest_get_irq(u.qts, 0));
    qtest_quit(u.qts);
}

/* Bytes past 64 are dropped; OVER sets FIFOE and the line status interrupt */
static void test_overrun(void)
{
    Uart u = start("");

    setup_115200(&u, 0);
    wr(&u, IER, IER_RLSIE);
    send_bytes(&u, 70);
    qtest_clock_step(u.qts, 70 * FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, URCR), ==, 64);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc6);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_FIFOE | LSR_TEMT | LSR_TDRQ |
                    LSR_OE | LSR_DR);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ | LSR_DR);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc1);
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0x30 + i);
    }
    g_assert_cmphex(rd(&u, LSR) & LSR_DR, ==, 0);
    qtest_quit(u.qts);
}

/* Receive data ready at 1, 8, 16 and 32 bytes */
static void test_rx_trigger(void)
{
    static const unsigned level[4] = { 1, 8, 16, 32 };

    for (unsigned rdtr = 0; rdtr < 4; rdtr++) {
        Uart u = start("");

        setup_115200(&u, rdtr << 6);
        wr(&u, IER, IER_RDRIE);
        send_bytes(&u, 40);
        qtest_clock_step(u.qts, (level[rdtr] - 1) * FRAME_NS + 1);
        g_assert_cmpuint(rd(&u, URCR), ==, level[rdtr] - 1);
        g_assert_cmphex(rd(&u, IIR), ==, 0xc1);
        qtest_clock_step(u.qts, FRAME_NS);
        g_assert_cmpuint(rd(&u, URCR), ==, level[rdtr]);
        g_assert_cmphex(rd(&u, IIR), ==, 0xc4);
        qtest_quit(u.qts);
    }
}

/* 4 x 8 + 12 bits from the last start bit: 34 bits after the byte lands */
static void test_rx_timeout(void)
{
    const int64_t timeout = 294664;
    Uart u = start("");

    setup_115200(&u, 1 << 6);
    wr(&u, IER, IER_RTOIE);
    send_bytes(&u, 2);
    qtest_clock_step(u.qts, 2 * FRAME_NS + timeout - 1);
    g_assert_cmpuint(rd(&u, URCR), ==, 2);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc1);
    qtest_clock_step(u.qts, 2);
    g_assert_cmphex(rd(&u, IIR), ==, 0xcc);
    rd(&u, RBR);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc1);
    qtest_quit(u.qts);

    /* The data ready enable alone does not give it */
    u = start("");
    setup_115200(&u, 1 << 6);
    wr(&u, IER, IER_RDRIE);
    send_bytes(&u, 2);
    qtest_clock_step(u.qts, 100 * FRAME_NS);
    g_assert_cmphex(rd(&u, IIR), ==, 0xc1);
    qtest_quit(u.qts);
}

/* The transmitter runs at three cycles a bit, the receiver gets nothing */
static void test_rx_min_bit(void)
{
    Uart u = start("");

    setup(&u, 13, 3, 0, 0x03, FCR_FME | FCR_UME, true);
    send_bytes(&u, 5);
    qtest_clock_step(u.qts, 5 * 16250 + 1);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);
    qtest_quit(u.qts);
}

static void test_ume(void)
{
    Uart u = start("");

    setup(&u, 13, 16, 0, 0x03, FCR_FME, true);
    send_bytes(&u, 5);
    qtest_clock_step(u.qts, 10 * FRAME_NS);
    g_assert_cmpuint(rd(&u, UTCR), ==, 5);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TDRQ);
    wr(&u, FCR, FCR_FME | FCR_UME);
    qtest_clock_step(u.qts, 5 * FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, UTCR), ==, 0);
    g_assert_cmpuint(rd(&u, URCR), ==, 5);
    qtest_quit(u.qts);
}

/* In loopback the modem inputs follow DTR, RTS, OUT1 and OUT2 */
static void test_loop_modem(void)
{
    Uart u = start("");

    wr(&u, MCR, 0x1f);
    g_assert_cmphex(rd(&u, MSR), ==, 0xfb);
    g_assert_cmphex(rd(&u, MSR), ==, 0xf0);
    wr(&u, MCR, MCR_LOOP);
    g_assert_cmphex(rd(&u, MSR), ==, 0x0f);
    qtest_quit(u.qts);
}

static void test_no_line_rate(void)
{
    Uart u = start("-global ingenic-uart.line-rate=off");

    setup_115200(&u, 0);
    send_bytes(&u, 10);
    g_assert_cmpuint(rd(&u, UTCR), ==, 0);
    g_assert_cmpuint(rd(&u, URCR), ==, 10);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ | LSR_DR);
    qtest_quit(u.qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(machines); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-uart/reset/%s",
                                                machines[i]);
        qtest_add_data_func(path, machines[i], test_reset);
    }
    for (int i = 0; i < ARRAY_SIZE(frames); i++) {
        const FrameCase *c = &frames[i];
        g_autofree char *path = g_strdup_printf(
            "ingenic-uart/frame/%u-%u-%03x-%02x",
            c->div, c->umr, c->uacr, c->lcr);
        qtest_add_data_func(path, c, test_frame);
    }
    qtest_add_func("ingenic-uart/tx-fifo", test_tx_fifo);
    qtest_add_func("ingenic-uart/tdrq", test_tdrq);
    qtest_add_func("ingenic-uart/overrun", test_overrun);
    qtest_add_func("ingenic-uart/rx-trigger", test_rx_trigger);
    qtest_add_func("ingenic-uart/rx-timeout", test_rx_timeout);
    qtest_add_func("ingenic-uart/rx-min-bit", test_rx_min_bit);
    qtest_add_func("ingenic-uart/ume", test_ume);
    qtest_add_func("ingenic-uart/loop-modem", test_loop_modem);
    qtest_add_func("ingenic-uart/no-line-rate", test_no_line_rate);

    return g_test_run();
}
