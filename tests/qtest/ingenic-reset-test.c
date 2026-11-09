/*
 * QTest testcase for Ingenic power-on values: AIC, SADC, SSI, DTRNG, RSA, SFC
 *
 * One case per part: the registers of these blocks as the part named reads
 * them at the U-Boot prompt, before a driver runs them, with each block's
 * clock opened for the read, after a millisecond of virtual time (the T23's
 * IVDC once divided by a line time it had yet to set). RSA's busy bit, set
 * out of reset, clears once an operation starts, as the bootroms and the
 * vendor driver wait on it. A SADC channel bit converts once and clears, as
 * a T31X takes it, and wedges if set with the converter powered down.
 * The T10's DMIC, at 0x10022000, reads its resets; a T20X reads 0 there.
 * The A1's second AIC reads the first's resets and its DES the T40's DESSR.
 * SFC_MEM_ADDR reads back the address written, as a T31X's does after its
 * driver's DMA. On the T32, T41 and A1 it is DES_ADDR too, and a CDT run
 * leaves its entry's command in TRAN_CONF0's third phase, the address in
 * the DEV_ADDR of its kind and its status mask in DEV_STA_MSK, as a
 * T41NQ's after a read.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define AIC             0x10020000
#define SSI0            0x10043000
#define SADC            0x10070000
#define DTRNG           0x10072000
#define RSA             0x134c0000
#define T23_IVDC        0x13200000
#define SFC             0x13440000

#define AIC_FR          (AIC + 0x00)
#define AIC_CR          (AIC + 0x04)
#define I2SSR           (AIC + 0x1c)
#define I2SDIV          (AIC + 0x30)
#define AICTFLR         (AIC + 0x3c)
#define SSI_SR          (SSI0 + 0x0c)
#define ADENA           (SADC + 0x00)
#define ADCFG           (SADC + 0x04)
#define ADCTRL          (SADC + 0x08)
#define ADSTATE         (SADC + 0x0c)
#define ADATA0          (SADC + 0x10)
#define ADCLK           (SADC + 0x20)
#define ADSTB           (SADC + 0x24)
#define DTRNG_CFG       (DTRNG + 0x00)
#define RSAC            (RSA + 0x00)
#define RSA_EXP         (RSA + 0x04)
#define IVDC_DDR_WR     (T23_IVDC + 0x208)
#define T10_DMIC        0x10022000
#define A1_AIC1         0x10021000
#define A1_DES          0x10061000
#define SFC_GLB         (SFC + 0x00)
#define SFC_DEV_CONF    (SFC + 0x04)
#define SFC_MEM_ADDR    (SFC + 0x60)
#define SFC_FSM         (SFC + 0x74)
#define SFC_CGE         (SFC + 0x78)

#define ADENA_REPTEN    0x0200
#define ADENA_POWER     0x8000      /* powered down */

#define RSAC_ENABLE     0x001
#define RSAC_MAIN       0x010
#define RSAC_BUSY       0x200

typedef struct {
    uint32_t addr, value;
} Reg;

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    bool sadc;                  /* the part has one */
    Reg regs[10];               /* an address of 0 ends the list */
} ResetCase;

/* What every part reads alike */
static const Reg common[] = {
    { AIC_FR, 0x07080000 }, { AIC_CR, 0x01240000 },
    { SFC_GLB, 0x8f }, { SFC_DEV_CONF, 0x27 }, { SFC_CGE, 0x2f },
};
static const Reg sadc_common[] = {
    { ADENA, 0x8000 }, { ADCTRL, 0xff }, { ADATA0, 0 },
};

static const ResetCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 0 },
        { ADCFG, 0 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 },
        { SFC_FSM, 0x00010849 },
        { T10_DMIC + 0x00, 0xc8 }, { T10_DMIC + 0x08, 0x3f },
        { T10_DMIC + 0x18, 0x147a } } },
    { "t20x", "ingenic-t20", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 0 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 },
        { SFC_FSM, 0x00010849 }, { T10_DMIC + 0x00, 0 } } },
    { "t21n", "ingenic-t21", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 }, { ADSTB, 0x00100000 },
        { SSI_SR, 0x00000098 }, { DTRNG_CFG, 0x03ff0800 },
        { SFC_FSM, 0x00010849 } } },
    { "t23n", "ingenic-t23", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 },
        { RSAC, 0x00030200 }, { IVDC_DDR_WR, 0 },
        { SFC_FSM, 0x00010849 } } },
    { "t30x", "ingenic-t30", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 0 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 },
        { SFC_FSM, 0x00010849 } } },
    { "t31x", "ingenic-t31", "ingenic-t31", true, {
        { I2SSR, 0x1c }, { I2SDIV, 0x00030003 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 }, { DTRNG_CFG, 0x03ff0800 },
        { RSAC, 0x00030200 },
        { SFC_FSM, 0x00010849 } } },
    { "t32lq", "ingenic-t32", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x04021000 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 },
        { RSAC, 0x00030200 },
        { SFC_FSM, 0x00010111 } } },
    { "t40n", "ingenic-t40", "ingenic-t40", true, {
        { I2SSR, 0x0c }, { I2SDIV, 0x00030003 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 }, { DTRNG_CFG, 0x03ff0800 },
        { RSAC, 0x00030200 },
        { SFC_FSM, 0x00010111 } } },
    { "t41lq", "ingenic-t41", "ingenic-t40", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 },
        { RSAC, 0x00030200 },
        { SFC_FSM, 0x00010111 } } },
    { "a1n", "ingenic-a1", "ingenic-a1", false, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 },
        { SFC_FSM, 0x00010111 },
        { A1_AIC1 + 0x00, 0x07080000 }, { A1_AIC1 + 0x30, 0x00040004 },
        { A1_DES + 0x08, 0x19 } } },
};

static void check(QTestState *qts, const char *part, const Reg *r, int n)
{
    for (int i = 0; i < n && r[i].addr; i++) {
        g_test_message("%s 0x%08x", part, r[i].addr);
        g_assert_cmphex(qtest_readl(qts, r[i].addr), ==, r[i].value);
    }
}

static void test_case(const void *data)
{
    const ResetCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    qtest_clock_step(qts, 1000000);
    check(qts, c->part, common, ARRAY_SIZE(common));
    if (c->sadc) {
        check(qts, c->part, sadc_common, ARRAY_SIZE(sadc_common));
    }
    check(qts, c->part, c->regs, ARRAY_SIZE(c->regs));
    qtest_quit(qts);
}

static void test_rsa_busy(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    g_assert_cmphex(qtest_readl(qts, RSAC) & RSAC_BUSY, ==, RSAC_BUSY);
    qtest_writel(qts, RSA_EXP, 0x10001);
    g_assert_cmphex(qtest_readl(qts, RSAC) & RSAC_BUSY, ==, 0);
    qtest_writel(qts, RSAC, RSAC_ENABLE | RSAC_BUSY);
    g_assert_cmphex(qtest_readl(qts, RSAC) & RSAC_BUSY, ==, RSAC_BUSY);
    qtest_writel(qts, RSAC, RSAC_ENABLE | RSAC_MAIN);
    g_assert_cmphex(qtest_readl(qts, RSAC) & RSAC_BUSY, ==, 0);
    qtest_quit(qts);
}

static void test_sadc_start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    qtest_writel(qts, ADENA, 0);
    /* At the reset clocks the sample is in at once */
    qtest_writel(qts, ADENA, 0x01);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0x01);
    g_assert_cmphex(qtest_readl(qts, ADATA0), ==, 2025);

    /* No second sample, REPTEN or not */
    qtest_writel(qts, ADSTATE, 0xff);
    qtest_writel(qts, ADENA, ADENA_REPTEN | 0x01);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, ADENA_REPTEN);
    qtest_writel(qts, ADSTATE, 0xff);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0);

    qtest_writel(qts, ADENA, 0x03);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0x03);

    qtest_writel(qts, ADSTATE, 0xff);
    qtest_writel(qts, ADENA, ADENA_POWER);
    qtest_writel(qts, ADENA, ADENA_POWER | 0x01);
    qtest_writel(qts, ADENA, 0x01);
    qtest_clock_step(qts, 1000000);
    qtest_writel(qts, ADENA, 0);
    g_assert_cmphex(qtest_readl(qts, ADENA), ==, 0x01);
    g_assert_cmphex(qtest_readl(qts, ADSTATE), ==, 0);
    qtest_quit(qts);
}

static void test_sfc_mem_addr(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    qtest_writel(qts, SFC_MEM_ADDR, 0x004c4000);
    g_assert_cmphex(qtest_readl(qts, SFC_MEM_ADDR), ==, 0x004c4000);
    qtest_quit(qts);
}

static void test_sfc_cdt(const void *data)
{
    QTestState *qts = qtest_initf("-machine %s", (const char *)data);

    qtest_writel(qts, SFC_MEM_ADDR, 0x2229a000);
    g_assert_cmphex(qtest_readl(qts, SFC + 0x90), ==, 0x2229a000);
    qtest_writel(qts, SFC + 0x800, 1);                  /* ROW_ADDR, last */
    qtest_writel(qts, SFC + 0x804, 0x11010003);
    qtest_writel(qts, SFC + 0x80c, 5);
    qtest_writel(qts, SFC + 0x84, 0x605000);
    qtest_writel(qts, SFC_GLB, 1 << 14);                /* CDT_EN */
    qtest_writel(qts, SFC + 0x7c, 0);
    qtest_writel(qts, SFC + 0x64, 1);
    g_assert_cmphex(qtest_readl(qts, SFC + 0x1c), ==, 0x11010003);
    g_assert_cmphex(qtest_readl(qts, SFC + 0x34), ==, 0x605000);
    g_assert_cmphex(qtest_readl(qts, SFC + 0x10), ==, 5);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-reset/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    qtest_add_func("ingenic-reset/rsa-busy", test_rsa_busy);
    qtest_add_func("ingenic-reset/sadc-start", test_sadc_start);
    qtest_add_func("ingenic-reset/sfc-mem-addr", test_sfc_mem_addr);
    qtest_add_data_func("ingenic-reset/sfc-cdt/t32", "ingenic-t32",
                        test_sfc_cdt);
    qtest_add_data_func("ingenic-reset/sfc-cdt/t41", "ingenic-t41",
                        test_sfc_cdt);
    qtest_add_data_func("ingenic-reset/sfc-cdt/a1", "ingenic-a1",
                        test_sfc_cdt);
    return g_test_run();
}
