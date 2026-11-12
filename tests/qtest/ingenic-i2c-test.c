/*
 * QTest testcase for the Ingenic I2C's power-on values and DMA requests
 *
 * Every I2C of a T20X, T21N, T31X, T32LQ and T41LQ reads these before a
 * driver touches it: the DesignWare core's SCL counts, a 10-bit target
 * address flag, and the FIFO thresholds, SDA timing, ACK of general calls
 * and spike filter as Ingenic sets them.
 *
 * The DMA requests (PDMA request types 0x24 + 2n TX, 0x25 + 2n RX) as a
 * T31X's I2C0 raises them, reading a GC4653's chip ID through PDMA
 * channels. The SADC's request type 0x09 never moves a channel there.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define I2C(n)          (0x10050000 + (n) * 0x1000)

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    unsigned n;                 /* its I2C controllers */
} I2cCase;

static const I2cCase cases[] = {
    { "t20x", "ingenic-t20", "ingenic-t31", 2 },
    { "t21n", "ingenic-t21", "ingenic-t31", 2 },
    { "t31x", "ingenic-t31", "ingenic-t31", 2 },
    { "t32lq", "ingenic-t32", "ingenic-t31", 3 },
    { "t41lq", "ingenic-t41", "ingenic-t40", 3 },
};

static const struct {
    uint32_t offset, value;
} power_on[] = {
    { 0x00, 0x7d },     /* CTRL */
    { 0x04, 0x1055 },   /* TAR */
    { 0x14, 0x190 },    /* SHCNT */
    { 0x18, 0x1d6 },    /* SLCNT */
    { 0x1c, 0x3c },     /* FHCNT */
    { 0x20, 0x82 },     /* FLCNT */
    { 0x38, 0x1f },     /* RXTL */
    { 0x3c, 0x20 },     /* TXTL */
    { 0x7c, 0x55 },     /* SDAHD */
    { 0x94, 0x64 },     /* SDASU */
    { 0x98, 0x1 },      /* ACKGC */
    { 0xa0, 0x1 },      /* FLT */
};

static void test_case(const void *data)
{
    const I2cCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    for (unsigned n = 0; n < c->n; n++) {
        for (int i = 0; i < ARRAY_SIZE(power_on); i++) {
            g_test_message("%s i2c%u 0x%02x", c->part, n, power_on[i].offset);
            g_assert_cmphex(qtest_readl(qts, I2C(n) + power_on[i].offset),
                            ==, power_on[i].value);
        }
    }
    qtest_quit(qts);
}

#define PDMA            0x13420000
#define CH(n, r)        (PDMA + (n) * 0x20 + (r))
#define DSA             0x00
#define DTA             0x04
#define DTC             0x08
#define DRT             0x0c
#define DCS             0x10
#define DCM             0x14
#define DDA             0x18
#define DMAC            (PDMA + 0x1000)
#define DCS_GO          0x80000001      /* no descriptor, enabled */
#define DCS_TT          0x8
#define DCM_SAI_W       0x00800000      /* source increments, words */
#define DCM_DAI_W       0x00400000      /* target increments, words */

#define I2C0            I2C(0)
#define DC              0x10
#define RINTST          0x34
#define CTXABRT         0x54
#define ENB             0x6c
#define RXFLR           0x78
#define TXABRT          0x80
#define DMACR           0x88
#define DMATDLR         0x8c
#define DMARDLR         0x90

#define CMDS            0x00100000
#define RXBUF           0x00200000

static QTestState *dma_init(uint32_t tdlr, uint32_t rdlr, uint32_t dmacr,
                            uint32_t tar)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 "
                                 "-global ingenic-t31.soc-variant=t31x "
                                 "-device image-sensor,bus=i2c0,"
                                 "model=gc4653");

    qtest_writel(qts, DMAC, 1);
    qtest_writel(qts, I2C0 + ENB, 0);
    qtest_writel(qts, I2C0 + DMATDLR, tdlr);
    qtest_writel(qts, I2C0 + DMARDLR, rdlr);
    qtest_writel(qts, I2C0 + DMACR, dmacr);
    qtest_writel(qts, I2C0 + 0x04, tar);
    qtest_writel(qts, I2C0 + ENB, 1);
    return qts;
}

/* Address 0x03f0, then n reads, the last with STOP */
static uint32_t put_cmds(QTestState *qts, unsigned n)
{
    uint32_t i = 0;

    qtest_writel(qts, CMDS, 0x03);
    qtest_writel(qts, CMDS + 4, 0xf0);
    for (i = 2; i < n + 2; i++) {
        qtest_writel(qts, CMDS + 4 * i, 0x100 | (i == n + 1 ? 0x200 : 0));
    }
    return i;
}

static void ch_start(QTestState *qts, int n, uint32_t dsa, uint32_t dta,
                     uint32_t dtc, uint32_t drt, uint32_t dcm)
{
    qtest_writel(qts, CH(n, DCS), 0);
    qtest_writel(qts, CH(n, DSA), dsa);
    qtest_writel(qts, CH(n, DTA), dta);
    qtest_writel(qts, CH(n, DTC), dtc);
    qtest_writel(qts, CH(n, DRT), drt);
    qtest_writel(qts, CH(n, DCM), dcm);
    qtest_writel(qts, CH(n, DCS), DCS_GO);
}

/* TX: commands go out while TDMAE, the controller enabled; the ID reads */
static void test_dma_tx(void)
{
    QTestState *qts = dma_init(4, 0, 2, 0x29);
    uint32_t n = put_cmds(qts, 3);

    /* Finishing, a no-descriptor run copies DDA bits 11:4 into CDOA */
    qtest_writel(qts, CH(31, DDA), 0x12345ab0);
    ch_start(qts, 31, CMDS, I2C0 + DC, n, 0x24, DCM_SAI_W);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(31, DCS)), ==,
                    DCS_GO | 0xab00 | DCS_TT);
    g_assert_cmphex(qtest_readl(qts, I2C0 + RXFLR), ==, 3);
    g_assert_cmphex(qtest_readl(qts, I2C0 + DC), ==, 0x46);
    g_assert_cmphex(qtest_readl(qts, I2C0 + DC), ==, 0x53);
    qtest_quit(qts);
}

/* No TX request with TDMAE clear, nor with the controller disabled */
static void test_dma_tx_gated(void)
{
    QTestState *qts = dma_init(4, 0, 0, 0x29);
    uint32_t n = put_cmds(qts, 3);

    ch_start(qts, 31, CMDS, I2C0 + DC, n, 0x24, DCM_SAI_W);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, n);
    qtest_writel(qts, I2C0 + ENB, 0);
    qtest_writel(qts, I2C0 + DMACR, 2);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, n);
    qtest_writel(qts, I2C0 + ENB, 1);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, 0);
    qtest_quit(qts);
}

/*
 * An abort holds the TX request down until it is cleared; then the next
 * command goes out, and aborts again
 */
static void test_dma_tx_abort(void)
{
    QTestState *qts = dma_init(0, 0, 2, 0x50);
    uint32_t n = put_cmds(qts, 4), left;

    ch_start(qts, 31, CMDS, I2C0 + DC, n, 0x24, DCM_SAI_W);
    left = qtest_readl(qts, CH(31, DTC));
    g_assert_cmphex(left, <, n);
    g_assert_cmphex(left, >, 0);
    g_assert_cmphex(qtest_readl(qts, I2C0 + TXABRT) & 1, ==, 1);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, left);
    g_assert_cmphex(qtest_readl(qts, I2C0 + CTXABRT), ==, 1);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), <, left);
    g_assert_cmphex(qtest_readl(qts, I2C0 + TXABRT) & 1, ==, 1);
    /* RINTST is read-only: a write (0x58, as a T31X took) changes nothing */
    left = qtest_readl(qts, I2C0 + RINTST);
    qtest_writel(qts, I2C0 + RINTST, 0x58);
    qtest_writel(qts, I2C0 + RINTST, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, I2C0 + RINTST), ==, left);
    qtest_quit(qts);
}

/*
 * RX: the request rises once the FIFO holds more than DMARDLR, and the
 * channel leaves DMARDLR entries behind. A data word reads as its byte.
 */
static void test_dma_rx(void)
{
    QTestState *qts = dma_init(0, 3, 1, 0x29);
    uint32_t n = put_cmds(qts, 10);

    ch_start(qts, 30, I2C0 + DC, RXBUF, 16, 0x25, DCM_DAI_W);
    for (uint32_t i = 0; i < n; i++) {
        uint32_t reads = i < 2 ? 0 : i - 1;

        qtest_writel(qts, I2C0 + DC, qtest_readl(qts, CMDS + 4 * i));
        g_assert_cmphex(qtest_readl(qts, CH(30, DTC)), ==,
                        16 - (reads > 3 ? reads - 3 : 0));
        g_assert_cmphex(qtest_readl(qts, I2C0 + RXFLR), ==, MIN(reads, 3));
    }
    g_assert_cmphex(qtest_readl(qts, RXBUF), ==, 0x46);
    g_assert_cmphex(qtest_readl(qts, RXBUF + 4), ==, 0x53);
    qtest_quit(qts);
}

/* Both ways: more reads than the RX FIFO holds, drained as they come */
static void test_dma_both(void)
{
    QTestState *qts = dma_init(0, 0, 3, 0x29);
    uint32_t n = put_cmds(qts, 70);

    ch_start(qts, 30, I2C0 + DC, RXBUF, 70, 0x25, DCM_DAI_W);
    ch_start(qts, 31, CMDS, I2C0 + DC, n, 0x24, DCM_SAI_W);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, CH(30, DTC)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, I2C0 + RXFLR), ==, 0);
    g_assert_cmphex(qtest_readl(qts, I2C0 + RINTST) & 2, ==, 0); /* RXOF */
    g_assert_cmphex(qtest_readl(qts, RXBUF), ==, 0x46);
    g_assert_cmphex(qtest_readl(qts, RXBUF + 4), ==, 0x53);
    qtest_quit(qts);
}

/* The DMA levels take only levels under the FIFO depth, of bits 7:0 */
static void test_dma_levels(void)
{
    QTestState *qts = dma_init(5, 5, 0, 0x29);

    qtest_writel(qts, I2C0 + DMATDLR, 0x40);
    qtest_writel(qts, I2C0 + DMARDLR, 0xff);
    g_assert_cmphex(qtest_readl(qts, I2C0 + DMATDLR), ==, 5);
    g_assert_cmphex(qtest_readl(qts, I2C0 + DMARDLR), ==, 5);
    qtest_writel(qts, I2C0 + DMATDLR, 0x13f);
    qtest_writel(qts, I2C0 + DMARDLR, 0x10003);
    g_assert_cmphex(qtest_readl(qts, I2C0 + DMATDLR), ==, 0x3f);
    g_assert_cmphex(qtest_readl(qts, I2C0 + DMARDLR), ==, 3);
    qtest_quit(qts);
}

/* The SADC raises no request: a conversion moves nothing */
static void test_sadc_no_request(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 "
                                 "-global ingenic-t31.soc-variant=t31x");

    qtest_writel(qts, DMAC, 1);
    ch_start(qts, 31, 0x10070010, RXBUF, 4, 0x09, DCM_DAI_W);
    qtest_writel(qts, 0x10070000, 0);           /* power up */
    qtest_writel(qts, 0x10070000, 1);           /* convert channel 0 */
    qtest_clock_step(qts, 10000000);
    g_assert_cmphex(qtest_readl(qts, 0x1007000c) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, CH(31, DTC)), ==, 4);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-i2c/dma/tx", test_dma_tx);
    qtest_add_func("ingenic-i2c/dma/tx-gated", test_dma_tx_gated);
    qtest_add_func("ingenic-i2c/dma/tx-abort", test_dma_tx_abort);
    qtest_add_func("ingenic-i2c/dma/rx", test_dma_rx);
    qtest_add_func("ingenic-i2c/dma/both", test_dma_both);
    qtest_add_func("ingenic-i2c/dma/levels", test_dma_levels);
    qtest_add_func("ingenic-i2c/dma/sadc-none", test_sadc_no_request);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-i2c/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
