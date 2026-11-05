/*
 * QTest testcase for the Ingenic UART
 *
 * Every expected value and time here was measured on a T31X and a T41NQ
 * (UART2, in loopback): the reset state and writable bits on every
 * machine, the frame time for a range of divisor, UMR, UACR and line
 * settings, the FIFO depths and counts, the transmit request and receive
 * trigger levels, the receive timeout and its enable, overrun, the
 * receiver's minimum bit length, and UFCR.UME. The DMA requests follow
 * the manual: a PDMA channel each way through UART2, UART5 and UART0.
 * The CPM clock gates as a T10L, T21N, T23DL, T31X, T32LQ, T40XP, T41LQ
 * and A1N read them: a gated UART reads 0, drops writes and keeps its
 * registers, and the windows past the T10's two UARTs and the T32's three
 * are empty. Offset 0x34 as those parts read it: 0x231c, bits 31:30 and
 * 15:0 kept, and bit 31 stops a looped-back byte reaching the FIFO. As a
 * T31X reads them (UART0, UART2): URBR read from an empty FIFO, and the
 * 0x34 time, latch and data ready sequences; the same as a T23N (UART0,
 * UART2), T40N, A1N (UART2) and T41LQ (UART2, UART5) read them, with their
 * ring entries' error flags, the latched URBR byte and URBR out of FIFO
 * mode. Bytes from the pin under 0x34 bit 31 as a T23N, T31X, T40N, T41LQ
 * and A1N console UART1 reads them at U-Boot.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/sockets.h"
#include "libqtest.h"

#define UART0           0x10030000
#define UART1           0x10031000
#define UART2           0x10032000
#define T32_UART3       0x10033000     /* the T33's; nothing on a T32 */
#define T41_UART5       0x10035000

#define CPM_CLKGR0      0x10000020
#define A1_CLKGR0       0x10000030
#define T32_CLKGR0_ROM  0xf1b9ffe0     /* as a T32LQ's bootrom leaves it */

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
#define R34             0x34
#define URCR            0x40
#define UTCR            0x44

#define IER_RDRIE       0x01
#define IER_TDRIE       0x02
#define IER_RLSIE       0x04
#define IER_RTOIE       0x10

#define FCR_FME         0x01
#define FCR_RESET       0x06
#define FCR_DME         0x08
#define FCR_UME         0x10

#define MCR_LOOP        0x10

#define LSR_DR          0x01
#define LSR_OE          0x02
#define LSR_FE          0x08
#define LSR_BI          0x10
#define LSR_TDRQ        0x20
#define LSR_TEMT        0x40
#define LSR_FIFOE       0x80

#define PDMA            0x13420000
#define PDMA_DMAC       0x1000
#define CH_DSA          0x00
#define CH_DTA          0x04
#define CH_DTC          0x08
#define CH_DRT          0x0c
#define CH_DCS          0x10
#define CH_DCM          0x14
#define DCS_NDES        0x80000000
#define DCS_TT          0x08
#define DCS_CTE         0x01
#define DCM_SAI         (1 << 23)
#define DCM_DAI         (1 << 22)
#define DCM_BYTES       ((1 << 14) | (1 << 12) | (1 << 8))  /* ports, units */

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

typedef struct {
    const char *machine;
    uint32_t clkgr;             /* the CPM register with the UART gates */
    unsigned bit;               /* UART0's gate; UART n's is bit + n */
    unsigned uarts;
} Part;

static const Part parts[] = {
    { "ingenic-t10", CPM_CLKGR0, 14, 2 },
    { "ingenic-t20", CPM_CLKGR0, 14, 3 },
    { "ingenic-t21", CPM_CLKGR0, 14, 3 },
    { "ingenic-t23", CPM_CLKGR0, 14, 3 },
    { "ingenic-t30", CPM_CLKGR0, 14, 3 },
    { "ingenic-t31", CPM_CLKGR0, 14, 3 },
    { "ingenic-t32", CPM_CLKGR0, 11, 3 },
    { "ingenic-t33", CPM_CLKGR0, 11, 3 },
    { "ingenic-t40", CPM_CLKGR0, 14, 4 },
    { "ingenic-t41", CPM_CLKGR0, 14, 6 },
    { "ingenic-a1", A1_CLKGR0, 8, 3 },
};

static const Part *find_part(const char *machine)
{
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        if (!strcmp(parts[i].machine, machine)) {
            return &parts[i];
        }
    }
    g_assert_not_reached();
}

/* Every UART's clock on: the bootroms leave all but UART1's, or all, gated */
static QTestState *start_machine(const char *machine, const char *args)
{
    const Part *p = find_part(machine);
    QTestState *qts = qtest_initf("-machine %s %s", machine, args);
    uint32_t mask = ((1u << p->uarts) - 1) << p->bit;

    qtest_writel(qts, p->clkgr, qtest_readl(qts, p->clkgr) & ~mask);
    return qts;
}

static Uart start(const char *args)
{
    return (Uart) {
        .qts = start_machine("ingenic-t31", args),
        .base = UART2,
    };
}

/* A UART whose 0x34 and receive FIFO were measured, in loopback */
typedef struct {
    const char *name;
    const char *machine;
    uint32_t base;
    bool ring;              /* its flags, latch and non-FIFO reads too */
} Probe;

static const Probe probes[] = {
    { "t23-uart0", "ingenic-t23", UART0, true },
    { "t23-uart2", "ingenic-t23", UART2, true },
    { "t31-uart2", "ingenic-t31", UART2, false },
    { "t40-uart2", "ingenic-t40", UART2, true },
    { "t41-uart2", "ingenic-t41", UART2, true },
    { "t41-uart5", "ingenic-t41", T41_UART5, true },
    { "a1-uart2", "ingenic-a1", UART2, true },
};

static Uart start_probe(const void *data)
{
    const Probe *p = data;

    return (Uart) {
        .qts = start_machine(p->machine, ""),
        .base = p->base,
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

static void test_reset(const void *data)
{
    static const struct {
        unsigned off;
        uint32_t reset, mask;
    } regs[] = {
        { IER, 0, 0x1f }, { LCR, 0, 0xff }, { MCR, 0, 0xdf }, { SPR, 0, 0xff },
        { ISR, 0, 0x1f }, { UMR, 0x10, 0x3f }, { UACR, 0, 0xfff },
        { R34, 0x231c, 0xc000ffff },
    };
    const Part *p = data;
    Uart u = {
        .qts = start_machine(p->machine, ""),
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

/* Switched off while bytes wait, they go at once; back on, they are paced */
static void test_line_rate_switch(void)
{
    Uart u = start("");

    setup_115200(&u, 0);
    send_bytes(&u, 10);
    g_assert_cmpuint(rd(&u, UTCR), ==, 9);
    qtest_qom_set_bool(u.qts, "/machine/ingenic-t31-uart2", "line-rate",
                       false);
    g_assert_cmpuint(rd(&u, UTCR), ==, 0);
    g_assert_cmpuint(rd(&u, URCR), ==, 10);
    g_assert_cmphex(rd(&u, LSR) & LSR_TEMT, ==, LSR_TEMT);
    qtest_qom_set_bool(u.qts, "/machine/ingenic-t31-uart2", "line-rate",
                       true);
    send_bytes(&u, 3);
    g_assert_cmpuint(rd(&u, UTCR), ==, 2);
    qtest_clock_step(u.qts, 3 * FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, URCR), ==, 13);
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

/* Requests only in DMA mode: TX on TDRQ without TDRIE, RX at the trigger */
static void test_dma_dreq(void)
{
    Uart u = start("");

    qtest_irq_intercept_out_named(u.qts, "/machine/ingenic-t31-uart2",
                                  "dreq");
    setup_115200(&u, 2 << 6);
    g_assert_false(qtest_get_irq(u.qts, 0));
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_DME | 2 << 6);
    g_assert_true(qtest_get_irq(u.qts, 0));
    wr(&u, IER, IER_TDRIE);
    g_assert_false(qtest_get_irq(u.qts, 0));
    wr(&u, IER, IER_RLSIE);
    send_bytes(&u, 40);
    g_assert_false(qtest_get_irq(u.qts, 0));
    qtest_clock_step(u.qts, 15 * FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, URCR), ==, 15);
    g_assert_true(qtest_get_irq(u.qts, 0));
    g_assert_false(qtest_get_irq(u.qts, 1));
    qtest_clock_step(u.qts, FRAME_NS);
    g_assert_true(qtest_get_irq(u.qts, 1));

    /* An overrun's FIFOE holds the RX request and raises line status */
    send_bytes(&u, 30);
    qtest_clock_step(u.qts, 60 * FRAME_NS);
    g_assert_cmpuint(rd(&u, URCR), ==, 64);
    g_assert_false(qtest_get_irq(u.qts, 1));
    g_assert_cmphex(rd(&u, IIR), ==, 0xc6);
    g_assert_cmphex(rd(&u, LSR) & LSR_FIFOE, ==, LSR_FIFOE);
    g_assert_true(qtest_get_irq(u.qts, 1));
    qtest_quit(u.qts);
}

typedef struct {
    const char *machine;
    uint32_t base;
    unsigned index;
} DmaCase;

static const DmaCase dma_cases[] = {
    { "ingenic-t31", 0x10032000, 2 },
    { "ingenic-t41", 0x10035000, 5 },
    { "ingenic-a1", 0x10030000, 0 },
};

static void pdma_channel(QTestState *qts, unsigned ch, uint32_t src,
                         uint32_t dst, uint32_t count, uint32_t drt,
                         uint32_t dcm)
{
    uint32_t c = PDMA + ch * 0x20;

    qtest_writel(qts, c + CH_DSA, src);
    qtest_writel(qts, c + CH_DTA, dst);
    qtest_writel(qts, c + CH_DTC, count);
    qtest_writel(qts, c + CH_DRT, drt);
    qtest_writel(qts, c + CH_DCM, dcm);
    qtest_writel(qts, c + CH_DCS, DCS_NDES | DCS_CTE);
}

/*
 * 40 bytes out of RAM into THR and, in loopback, out of RBR into RAM. The
 * RX channel takes a byte while 16 or more wait, so 15 stay for PIO. The
 * PDMA answers a request from a bottom half, so the clock goes a frame at
 * a time to let it run.
 */
static void test_dma(const void *data)
{
    const DmaCase *c = data;
    Uart u = {
        .qts = start_machine(c->machine, ""),
        .base = c->base,
    };
    uint8_t out[40], in[25];

    for (int i = 0; i < sizeof(out); i++) {
        out[i] = 0x40 + i;
    }
    qtest_memwrite(u.qts, 0x100000, out, sizeof(out));
    setup_115200(&u, FCR_DME | 2 << 6);
    qtest_writel(u.qts, PDMA + PDMA_DMAC, 1);
    pdma_channel(u.qts, 1, c->base + RBR, 0x200000, sizeof(out),
                 0x15 - 2 * c->index, DCM_DAI | DCM_BYTES);
    pdma_channel(u.qts, 0, 0x100000, c->base + RBR, sizeof(out),
                 0x14 - 2 * c->index, DCM_SAI | DCM_BYTES);
    for (int i = 0; i < 48; i++) {
        qtest_clock_step(u.qts, FRAME_NS);
    }

    g_assert_cmphex(qtest_readl(u.qts, PDMA + CH_DCS) & DCS_TT, ==, DCS_TT);
    g_assert_cmpuint(rd(&u, URCR), ==, 15);
    g_assert_cmpuint(qtest_readl(u.qts, PDMA + 0x20 + CH_DTC), ==, 15);
    qtest_memread(u.qts, 0x200000, in, sizeof(in));
    g_assert_cmpmem(in, sizeof(in), out, sizeof(in));
    for (int i = sizeof(in); i < sizeof(out); i++) {
        g_assert_cmphex(rd(&u, RBR), ==, out[i]);
    }
    qtest_quit(u.qts);
}

/* Each UART's gate, from the bootrom's: reads 0, drops writes, keeps state */
static void test_gate(const void *data)
{
    const Part *p = data;
    QTestState *qts = qtest_initf("-machine %s", p->machine);

    for (int n = 0; n < p->uarts; n++) {
        Uart u = { .qts = qts, .base = UART0 + n * 0x1000 };
        uint32_t bit = 1u << (p->bit + n);
        uint32_t gr = qtest_readl(qts, p->clkgr) | bit;

        qtest_writel(qts, p->clkgr, gr);
        g_assert_cmphex(rd(&u, IIR), ==, 0);
        g_assert_cmphex(rd(&u, LSR), ==, 0);
        g_assert_cmphex(rd(&u, UMR), ==, 0);
        g_assert_cmphex(rd(&u, R34), ==, 0);
        wr(&u, SPR, 0x5a);                  /* dropped */
        qtest_writel(qts, p->clkgr, gr & ~bit);
        g_assert_cmphex(rd(&u, IIR), ==, 0x01);
        g_assert_cmphex(rd(&u, LSR), ==, 0x60);
        g_assert_cmphex(rd(&u, UMR), ==, 0x10);
        g_assert_cmphex(rd(&u, R34), ==, 0x231c);
        g_assert_cmphex(rd(&u, SPR), ==, 0);
        wr(&u, SPR, 0x5a);
        qtest_writel(qts, p->clkgr, gr);
        g_assert_cmphex(rd(&u, SPR), ==, 0);
        wr(&u, SPR, 0xa5);
        qtest_writel(qts, p->clkgr, gr & ~bit);
        g_assert_cmphex(rd(&u, SPR), ==, 0x5a);  /* kept while gated */
    }
    qtest_quit(qts);
}

/* The bootroms leave UART1 gated; a T32LQ's U-Boot opens it */
static void test_t32_rom_gates(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    g_assert_cmphex(qtest_readl(qts, CPM_CLKGR0), ==, T32_CLKGR0_ROM);
    g_assert_cmphex(qtest_readl(qts, UART1 + UMR), ==, 0);
    qtest_quit(qts);
}

/* Bit 31 of 0x34: a looped-back byte goes out but never lands */
static void test_r34_rx_drop(void)
{
    Uart u = start("");

    setup_115200(&u, 0);
    wr(&u, R34, 0x8000231c);
    send_bytes(&u, 1);
    qtest_clock_step(u.qts, 2 * FRAME_NS);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);
    wr(&u, R34, 0x4000ffff);
    send_bytes(&u, 1);
    qtest_clock_step(u.qts, 2 * FRAME_NS);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ | LSR_DR);
    g_assert_cmpuint(rd(&u, URCR), ==, 1);
    qtest_quit(u.qts);
}

/* An empty FIFO read: the ring's stale byte, URCR wraps from 0 to 127 */
static void test_rx_underflow(const void *data)
{
    Uart u = start_probe(data);

    setup_115200(&u, 0);
    send_bytes(&u, 64);
    qtest_clock_step(u.qts, 64 * FRAME_NS + 1);
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0x30 + i);
    }
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ);
    for (int i = 0; i < 128; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0x30 + (i & 0x3f));
        g_assert_cmpuint(rd(&u, URCR), ==, 127 - i);
        g_assert_cmphex(rd(&u, LSR) & LSR_DR, ==, i == 127 ? 0 : LSR_DR);
    }
    /* From 126, two bytes take the count round to 0 and DR off */
    rd(&u, RBR);
    rd(&u, RBR);
    send_bytes(&u, 1);
    qtest_clock_step(u.qts, FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, URCR), ==, 127);
    send_bytes(&u, 1);
    qtest_clock_step(u.qts, FRAME_NS + 1);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ);
    qtest_quit(u.qts);
}

/* n cycles of the divided clock: 24 MHz over the divisor */
#define CYCLES_NS(div, n)   ((uint64_t)(div) * (n) * 1000 / 24)

/* Send a looped-back byte: did it land? The FIFO is left empty */
static bool lands(Uart *u)
{
    unsigned n;

    send_bytes(u, 1);
    qtest_clock_step(u->qts, 2 * FRAME_NS);
    n = rd(u, URCR);
    for (unsigned i = 0; i < n; i++) {
        rd(u, RBR);
    }
    return n == 1;
}

typedef struct {
    const char *name;
    uint32_t w[4];          /* written in turn, a gap after each */
    uint64_t gap_us[4];
    bool lands;             /* after the last, as the T31X did */
} R34Seq;

#define T  0xc000ffff       /* 35.5 ms at 115200 */
#define A  0x231c

static const R34Seq r34_seqs[] = {
    { "both-early", { T, A }, { 1000 }, false },
    { "both-late", { T, A }, { 50000 }, true },
    { "bit31-early", { 0x8000ffff, A }, { 1000 }, true },
    { "latched-both", { T, A, T, A }, { 1000, 0, 1000 }, true },
    { "latched-bit31-early", { T, A, 0x8000ffff, A }, { 1000, 0, 1000 },
      false },
    { "latched-bit31-late", { T, A, 0x8000ffff, A }, { 1000, 0, 50000 },
      true },
    { "latched-bit30", { T, A, 0x4000ffff, A }, { 1000, 0, 50000 }, false },
    { "bit30-falls", { T, 0x8000ffff, A }, { 20000, 20000 }, false },
    { "bit30-falls-late", { T, 0x8000ffff, A }, { 50000, 1000 }, true },
    { "same-rewrite", { T, T, A }, { 20000, 20000 }, true },
    { "time-rewrite", { T, 0xc000fffe, A }, { 20000, 20000 }, true },
    { "bit30-rises", { 0x8000ffff, T, A }, { 50000, 1000 }, false },
    { "compare-passed", { T, 0xc0000010, A }, { 1000, 1000 }, false },
    { "compare-ahead", { 0xc0001000, 0xc0000800, A }, { 1000, 500 }, true },
    { "bit31-cleared-early", { T, 0x4000ffff, A }, { 1000, 50000 }, false },
};

typedef struct {
    const R34Seq *seq;
    const Probe *probe;
} R34SeqCase;

static void test_r34_seq(const void *data)
{
    const R34SeqCase *rc = data;
    const R34Seq *c = rc->seq;
    Uart u = start_probe(rc->probe);

    setup_115200(&u, 0);
    for (int i = 0; i < 4 && c->w[i]; i++) {
        wr(&u, R34, c->w[i]);
        if (c->gap_us[i]) {
            qtest_clock_step(u.qts, c->gap_us[i] * 1000);
        }
    }
    g_assert_cmpint(lands(&u), ==, c->lands);
    qtest_quit(u.qts);
}

/* 0x1000 cycles of 24 MHz / divisor: 2.22 ms at 13, 8.87 ms at 52 */
static void test_r34_time(const void *data)
{
    static const unsigned divs[] = { 13, 52 };

    for (int i = 0; i < ARRAY_SIZE(divs); i++) {
        for (int late = 0; late < 2; late++) {
            Uart u = start_probe(data);

            setup(&u, divs[i], 16, 0, 0x03, FCR_FME | FCR_UME, true);
            wr(&u, R34, 0xc0001000);
            qtest_clock_step(u.qts, CYCLES_NS(divs[i], 0x1000) +
                             (late ? 1000 : -1000));
            wr(&u, R34, A);
            setup_115200(&u, 0);
            g_assert_cmpint(lands(&u), ==, late);
            qtest_quit(u.qts);
        }
    }
}

/* Latched: DR with bits 31:30 set and nothing stored; UME off ends it */
static void test_r34_latch(const void *data)
{
    Uart u = start_probe(data);

    setup_115200(&u, 0);
    wr(&u, R34, T);
    qtest_clock_step(u.qts, 1000000);
    wr(&u, R34, A);
    g_assert_false(lands(&u));
    g_assert_cmphex(rd(&u, R34), ==, A);
    wr(&u, R34, 0x8000231c);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ);
    qtest_clock_step(u.qts, 50000000);
    wr(&u, R34, A);
    g_assert_true(lands(&u));

    wr(&u, R34, T);
    qtest_clock_step(u.qts, 1000000);
    wr(&u, R34, A);
    g_assert_false(lands(&u));
    wr(&u, R34, T);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ | LSR_DR);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);
    qtest_clock_step(u.qts, 50000000);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ | LSR_DR);
    wr(&u, R34, A);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_TEMT | LSR_TDRQ);
    g_assert_true(lands(&u));

    wr(&u, R34, T);
    qtest_clock_step(u.qts, 1000000);
    wr(&u, R34, A);
    g_assert_false(lands(&u));
    wr(&u, FCR, FCR_FME);
    wr(&u, FCR, FCR_FME | FCR_UME);
    g_assert_true(lands(&u));
    qtest_quit(u.qts);
}

/* One looped-back byte, or a break, landed */
static void send1(Uart *u, uint8_t byte)
{
    wr(u, RBR, byte);
    qtest_clock_step(u->qts, 2 * FRAME_NS);
}

static void send_break(Uart *u)
{
    wr(u, LCR, 0x43);
    qtest_clock_step(u->qts, 3 * FRAME_NS);
    wr(u, LCR, 0x03);
    qtest_clock_step(u->qts, 2 * FRAME_NS);
}

#define LSR_IDLE        (LSR_TEMT | LSR_TDRQ)
#define LSR_BREAK       (LSR_BI | LSR_FE)

/* A break at entry 5 of a full ring: its flags at the head, twice round */
static void test_rx_flags(const void *data)
{
    Uart u = start_probe(data);

    setup_115200(&u, 0);
    for (int i = 0; i < 64; i++) {
        if (i == 5) {
            send_break(&u);
        } else {
            send1(&u, 0x40 + i);
        }
    }
    g_assert_cmphex(rd(&u, LSR), ==, LSR_FIFOE | LSR_IDLE | LSR_DR);
    for (int i = 0; i < 5; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0x40 + i);
    }
    g_assert_cmphex(rd(&u, LSR), ==, LSR_FIFOE | LSR_IDLE | LSR_BREAK |
                    LSR_DR);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_FIFOE | LSR_IDLE | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    for (int i = 6; i < 64; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0x40 + i);
    }
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    /* Round again from the ring: the flags, without FIFOE */
    for (int i = 0; i < 5; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0x40 + i);
    }
    g_assert_cmpuint(rd(&u, URCR), ==, 123);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_BREAK | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);

    /* A flagged entry at the head of an empty FIFO raises nothing */
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_RESET);
    send_break(&u);
    for (int i = 1; i < 64; i++) {
        send1(&u, 0x40 + i);
    }
    for (int i = 0; i < 63; i++) {
        rd(&u, RBR);
    }
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_BREAK | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0x7f);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    g_assert_cmphex(rd(&u, RBR), ==, 0);
    g_assert_cmpuint(rd(&u, URCR), ==, 127);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);

    /* BI and FE stay through the read and a FIFO reset, until ULSR is read */
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_RESET);
    send_break(&u);
    g_assert_cmphex(rd(&u, RBR), ==, 0);
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_RESET);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_BREAK);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    qtest_quit(u.qts);
}

/* A FIFO reset keeps the latched URBR byte; one landing replaces it */
static void test_rx_latch(const void *data)
{
    Uart u = start_probe(data);

    setup_115200(&u, 0);
    for (int i = 0; i < 8; i++) {
        send1(&u, 0xa0 + i);
    }
    for (int i = 0; i < 3; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0xa0 + i);
    }
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_RESET);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa3);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa2);
    g_assert_cmpuint(rd(&u, URCR), ==, 125);
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_RESET);
    send1(&u, 0x55);
    g_assert_cmphex(rd(&u, RBR), ==, 0x55);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    qtest_quit(u.qts);
}

/* Out of FIFO mode: entry 0 takes each byte, empty reads give entry 1 */
static void test_rx_nonfifo(const void *data)
{
    Uart u = start_probe(data);

    setup_115200(&u, 0);
    for (int i = 0; i < 8; i++) {
        send1(&u, 0xa0 + i);
    }
    for (int i = 0; i < 3; i++) {
        rd(&u, RBR);
    }
    wr(&u, FCR, FCR_UME);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    send1(&u, 0x55);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0x55);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    send1(&u, 0x11);
    send1(&u, 0x12);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_OE | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0x11);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    send_break(&u);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_BREAK | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    send1(&u, 0x11);
    rd(&u, RBR);
    wr(&u, FCR, FCR_UME | FCR_RESET);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    /* Back in FIFO mode the latched byte first, then the ring from 1 */
    wr(&u, FCR, FCR_FME | FCR_UME);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    g_assert_cmphex(rd(&u, RBR), ==, 0xa1);
    for (int i = 1; i < 8; i++) {
        g_assert_cmphex(rd(&u, RBR), ==, 0xa0 + i);
    }
    for (int i = 8; i < 64; i++) {
        rd(&u, RBR);
    }
    g_assert_cmphex(rd(&u, RBR), ==, 0x11);

    /* Leaving FIFO mode with bytes left keeps DR, and a byte lands */
    wr(&u, FCR, FCR_FME | FCR_UME | FCR_RESET);
    for (int i = 0; i < 8; i++) {
        send1(&u, 0xa0 + i);
    }
    rd(&u, RBR);
    wr(&u, FCR, FCR_UME | FCR_RESET);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    send1(&u, 0x55);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    g_assert_cmphex(rd(&u, RBR), ==, 0x55);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    qtest_quit(u.qts);
}

/* Bytes from the pin under bit 31 until DR sets */
static void line_bytes(Uart *u, int fd)
{
    wr(u, R34, 0x8000231c);
    g_assert_cmpint(write(fd, "xyz", 3), ==, 3);
    for (int i = 0; i < 100 && !(rd(u, LSR) & LSR_DR); i++) {
        qtest_clock_step(u->qts, FRAME_NS);
        g_usleep(1000);
    }
    g_assert_cmphex(rd(u, LSR), ==, LSR_IDLE | LSR_DR);
    g_assert_cmpuint(rd(u, URCR), ==, 0);
}

/*
 * A byte from the line under bit 31: DR without a byte, until 31 clears,
 * as each part's console UART1 reads it at U-Boot. A URBR read under it
 * takes a stale byte and wraps URCR to 127, which keeps DR.
 */
static void test_r34_line_dr(const void *data)
{
    const char *machine = data;
    g_autofree char *path = g_strdup_printf("%s/ingenic-uart-%d.sock",
                                            g_get_tmp_dir(), getpid());
    g_autofree char *args = g_strdup_printf(
        "-chardev socket,id=s0,path=%s,server=on,wait=off "
        "-serial null -serial chardev:s0", path);
    Uart u = {
        .qts = start_machine(machine, args),
        .base = UART1,
    };
    int fd;

    fd = unix_connect(path, &error_abort);
    setup(&u, 13, 16, 0, 0x03, FCR_FME | FCR_UME, false);
    line_bytes(&u, fd);
    wr(&u, R34, A);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE);
    g_assert_cmpuint(rd(&u, URCR), ==, 0);

    line_bytes(&u, fd);
    rd(&u, RBR);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    g_assert_cmpuint(rd(&u, URCR), ==, 127);
    wr(&u, R34, A);
    g_assert_cmphex(rd(&u, LSR), ==, LSR_IDLE | LSR_DR);
    g_assert_cmpuint(rd(&u, URCR), ==, 127);
    close(fd);
    unlink(path);
    qtest_quit(u.qts);
}

static void test_t10_no_uart2(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t10");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, UART2 + SPR, 0x5a);
    qtest_writel(qts, UART2 + LCR, 0x03);
    for (unsigned off = 0; off <= UTCR; off += 4) {
        g_assert_cmphex(qtest_readl(qts, UART2 + off), ==, 0);
    }
    qtest_quit(qts);
}

static void test_t32_no_uart3(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T32_UART3 + SPR, 0x5a);
    qtest_writel(qts, T32_UART3 + LCR, 0x03);
    for (unsigned off = 0; off <= UTCR; off += 4) {
        g_assert_cmphex(qtest_readl(qts, T32_UART3 + off), ==, 0);
    }
    g_assert_cmphex(qtest_readl(qts, T32_UART3 + 0xffc), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-uart/reset/%s",
                                                parts[i].machine);
        g_autofree char *gate = g_strdup_printf("ingenic-uart/gate/%s",
                                                parts[i].machine);

        qtest_add_data_func(path, &parts[i], test_reset);
        qtest_add_data_func(gate, &parts[i], test_gate);
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
    qtest_add_func("ingenic-uart/line-rate-switch", test_line_rate_switch);
    qtest_add_func("ingenic-uart/dma-dreq", test_dma_dreq);
    qtest_add_func("ingenic-uart/r34-rx-drop", test_r34_rx_drop);
    {
        static const char *const line_parts[] = {
            "ingenic-t23", "ingenic-t31", "ingenic-t40", "ingenic-t41",
            "ingenic-a1",
        };

        for (int i = 0; i < ARRAY_SIZE(line_parts); i++) {
            g_autofree char *path = g_strdup_printf(
                "ingenic-uart/r34-line-dr/%s", line_parts[i] + 8);

            qtest_add_data_func(path, line_parts[i], test_r34_line_dr);
        }
    }
    for (int i = 0; i < ARRAY_SIZE(probes); i++) {
        static const struct {
            const char *name;
            GTestDataFunc fn;
            bool ring;
        } per_probe[] = {
            { "rx-underflow", test_rx_underflow },
            { "rx-flags", test_rx_flags, true },
            { "rx-latch", test_rx_latch, true },
            { "rx-nonfifo", test_rx_nonfifo, true },
            { "r34-time", test_r34_time },
            { "r34-latch", test_r34_latch },
        };
        static R34SeqCase seq_cases[ARRAY_SIZE(probes)][ARRAY_SIZE(r34_seqs)];

        for (int t = 0; t < ARRAY_SIZE(per_probe); t++) {
            g_autofree char *path = g_strdup_printf("ingenic-uart/%s/%s",
                                                    per_probe[t].name,
                                                    probes[i].name);

            if (probes[i].ring || !per_probe[t].ring) {
                qtest_add_data_func(path, &probes[i], per_probe[t].fn);
            }
        }
        for (int k = 0; k < ARRAY_SIZE(r34_seqs); k++) {
            g_autofree char *path = g_strdup_printf(
                "ingenic-uart/r34-seq/%s/%s", probes[i].name,
                r34_seqs[k].name);

            seq_cases[i][k] = (R34SeqCase) { &r34_seqs[k], &probes[i] };
            qtest_add_data_func(path, &seq_cases[i][k], test_r34_seq);
        }
    }
    qtest_add_func("ingenic-uart/t32-rom-gates", test_t32_rom_gates);
    qtest_add_func("ingenic-uart/t32-no-uart3", test_t32_no_uart3);
    qtest_add_func("ingenic-uart/t10-no-uart2", test_t10_no_uart2);
    for (int i = 0; i < ARRAY_SIZE(dma_cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-uart/dma/%s",
                                                dma_cases[i].machine);
        qtest_add_data_func(path, &dma_cases[i], test_dma);
    }

    return g_test_run();
}
