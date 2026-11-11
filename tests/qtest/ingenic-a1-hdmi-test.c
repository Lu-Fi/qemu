/*
 * QTest testcase for the Ingenic A1 HDMI controller and its monitor
 *
 * The receiver sense and PLL lock status, the DDC master reading the EDID
 * (both blocks, the segment pointer, the NACK of an absent slave), the
 * interrupt status and mask, and the monitor unplugged.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define HDMI            0x13380000
#define HDMI_PHY        0x10075000
#define H_RXSENSE       0x410
#define H_IRQ_STATUS    0x414
#define H_IRQ_MASK      0x614
#define H_DDC_SLAVE     0x1f800
#define H_DDC_OFFSET    0x1f804
#define H_DDC_RDATA     0x1f80c
#define H_DDC_CMD       0x1f810
#define H_DDC_SEGMENT   0x1f828
#define PHY_POWER       0x2c0
#define PHY_SENSE       0x334

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, HDMI + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, HDMI + off, v);
}

/* One DDC byte at SEG:OFF from SLAVE, and the status it left (cleared) */
static uint32_t ddc(QTestState *qts, uint32_t slave, uint32_t seg,
                    uint32_t off, uint32_t *status)
{
    wr(qts, H_DDC_SLAVE, slave);
    wr(qts, H_DDC_OFFSET, off);
    if (seg) {
        wr(qts, H_DDC_SEGMENT, seg);
    }
    wr(qts, H_DDC_CMD, seg ? 2 : 1);
    *status = rd(qts, H_IRQ_STATUS);
    wr(qts, H_IRQ_STATUS, *status);
    g_assert_cmpuint(rd(qts, H_IRQ_STATUS), ==, 0);
    return rd(qts, H_DDC_RDATA);
}

static void test_sense_lock(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0x3c);
    qtest_writel(qts, HDMI_PHY + PHY_POWER, 4);
    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0x3e);
    g_assert_cmphex(qtest_readl(qts, HDMI_PHY + PHY_SENSE), ==, 0x55);
    qtest_writel(qts, HDMI_PHY + PHY_POWER, 0);
    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0x3c);
    g_assert_cmphex(qtest_readl(qts, HDMI_PHY + PHY_SENSE), ==, 0);
    qtest_quit(qts);
}

static void test_edid(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    static const uint8_t head[8] = { 0, 255, 255, 255, 255, 255, 255, 0 };
    uint32_t st, sum = 0;

    for (int i = 0; i < 8; i++) {
        g_assert_cmpuint(ddc(qts, 0x50, 0, i, &st), ==, head[i]);
        g_assert_cmpuint(st, ==, 2);
    }
    for (int i = 0; i < 128; i++) {
        sum += ddc(qts, 0x50, 0, i, &st);
    }
    g_assert_cmpuint(sum & 0xff, ==, 0);
    g_assert_cmpuint(ddc(qts, 0x50, 0, 126, &st), ==, 1);
    /* The second block, at 128 of segment 0, is CEA-861 */
    g_assert_cmpuint(ddc(qts, 0x50, 0, 128, &st), ==, 2);
    g_assert_cmpuint(ddc(qts, 0x50, 0, 129, &st), ==, 3);
    g_assert_cmpuint(ddc(qts, 0x50, 1, 0, &st), ==, 0);
    g_assert_cmpuint(st, ==, 2);
    sum = 0;
    for (int i = 128; i < 256; i++) {
        sum += ddc(qts, 0x50, 0, i, &st);
    }
    g_assert_cmpuint(sum & 0xff, ==, 0);
    qtest_quit(qts);
}

static void test_nack_mask(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    uint32_t st;

    /* The SCDC slave is not there: bit 0 */
    ddc(qts, 0x54, 0, 0, &st);
    g_assert_cmpuint(st, ==, 1);
    /* Masked, the status shows without the interrupt */
    wr(qts, H_IRQ_MASK, 3);
    ddc(qts, 0x50, 0, 0, &st);
    g_assert_cmpuint(st, ==, 2);
    g_assert_cmpuint(rd(qts, H_IRQ_MASK), ==, 3);
    qtest_quit(qts);
}

static void test_unplug(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    QDict *rsp;
    uint32_t st;

    rsp = qtest_qmp(qts, "{'execute': 'qom-set', 'arguments': "
                    "{'path': '/machine/soc/hdmi', 'property': 'connected', "
                    "'value': false}}");
    g_assert(qdict_haskey(rsp, "return"));
    qobject_unref(rsp);
    g_assert_cmphex(rd(qts, H_RXSENSE), ==, 0);
    ddc(qts, 0x50, 0, 0, &st);
    g_assert_cmpuint(st, ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-hdmi/sense-lock", test_sense_lock);
    qtest_add_func("ingenic-a1-hdmi/edid", test_edid);
    qtest_add_func("ingenic-a1-hdmi/nack-mask", test_nack_mask);
    qtest_add_func("ingenic-a1-hdmi/unplug", test_unplug);
    return g_test_run();
}
