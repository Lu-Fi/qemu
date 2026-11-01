/*
 * QTest testcase for Ingenic power-on values: AIC, SADC, SSI and DTRNG
 *
 * One case per part: the registers of these blocks as the part named reads
 * them at the U-Boot prompt, before a driver runs them, with each block's
 * clock opened for the read.
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

#define AIC_FR          (AIC + 0x00)
#define AIC_CR          (AIC + 0x04)
#define I2SSR           (AIC + 0x1c)
#define I2SDIV          (AIC + 0x30)
#define AICTFLR         (AIC + 0x3c)
#define SSI_SR          (SSI0 + 0x0c)
#define ADENA           (SADC + 0x00)
#define ADCFG           (SADC + 0x04)
#define ADCTRL          (SADC + 0x08)
#define ADATA0          (SADC + 0x10)
#define ADCLK           (SADC + 0x20)
#define DTRNG_CFG       (DTRNG + 0x00)

typedef struct {
    uint32_t addr, value;
} Reg;

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    bool sadc;                  /* the part has one */
    Reg regs[8];                /* an address of 0 ends the list */
} ResetCase;

/* What every part reads alike */
static const Reg common[] = {
    { AIC_FR, 0x07080000 }, { AIC_CR, 0x01240000 },
};
static const Reg sadc_common[] = {
    { ADENA, 0x8000 }, { ADCTRL, 0xff }, { ADATA0, 0 },
};

static const ResetCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 0 },
        { ADCFG, 0 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 } } },
    { "t20x", "ingenic-t20", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 0 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 } } },
    { "t21n", "ingenic-t21", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 }, { DTRNG_CFG, 0x03ff0800 } } },
    { "t23n", "ingenic-t23", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 } } },
    { "t30x", "ingenic-t30", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00030003 }, { AICTFLR, 0 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 } } },
    { "t31x", "ingenic-t31", "ingenic-t31", true, {
        { I2SSR, 0x1c }, { I2SDIV, 0x00030003 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 }, { DTRNG_CFG, 0x03ff0800 } } },
    { "t32lq", "ingenic-t32", "ingenic-t31", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x04021000 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 } } },
    { "t40n", "ingenic-t40", "ingenic-t40", true, {
        { I2SSR, 0x0c }, { I2SDIV, 0x00030003 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x00000098 }, { DTRNG_CFG, 0x03ff0800 } } },
    { "t41lq", "ingenic-t41", "ingenic-t40", true, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { ADCFG, 1 }, { ADCLK, 0x00041000 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 } } },
    { "a1n", "ingenic-a1", "ingenic-a1", false, {
        { I2SSR, 0 }, { I2SDIV, 0x00040004 }, { AICTFLR, 8 },
        { SSI_SR, 0x36000098 }, { DTRNG_CFG, 0x03ff0800 } } },
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

    check(qts, c->part, common, ARRAY_SIZE(common));
    if (c->sadc) {
        check(qts, c->part, sadc_common, ARRAY_SIZE(sadc_common));
    }
    check(qts, c->part, c->regs, ARRAY_SIZE(c->regs));
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
    return g_test_run();
}
