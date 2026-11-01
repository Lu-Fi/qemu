/*
 * QTest testcase for the Ingenic CPM's reset values
 *
 * One case per part: the registers U-Boot leaves alone, as the part named
 * reads them after U-Boot. OPCR reads 0x80 or 0xe0 more there, from the
 * SPL; 0x1500 is its reset in the T21 manual. A running PLL reads on and
 * locked, and CPCSR reads its stable flags. The memory power controls of the
 * T21 and T30 are at 0xf0 to 0xf8. USBPCR and USBPCR1 read without the bits
 * the kernel sets in them, and keep bit 28 of a write. From the T21 on,
 * USBRDT holds only bits 31, 27 and 2:0 of a write, as a T31X reads it back;
 * the T20's holds any. The MAC PHY control's top byte is the PHY's, which a
 * write leaves alone. The T21's and T32's I2S duty registers read half the
 * divider's N unless DEN is set, and SSICDR keeps its change enable, bit 28.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM             0x10000000
#define CPAPCR          0x10
#define A1_CPCSR        0xec
#define PLL_ON_LOCK_EN  0x0000000d

typedef struct {
    uint32_t offset, value;
} Reg;

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    bool t_series;              /* also reads the T-series registers below */
    Reg regs[16];               /* a zero value ends the list */
} CpmCase;

/* RSR, CPPCR, OPCR, CPSPPR and CPCSR on every part from the T10 to the T41 */
static const Reg t_series[] = {
    { 0x08, 0x00000001 }, { 0x0c, 0x00000020 }, { 0x24, 0x00001500 },
    { 0x38, 0x0000a5a5 }, { 0xd4, 0xf8000000 },
};

static const CpmCase cases[] = {
    { "t10l", "ingenic-t10", "ingenic-t31", true,
      { { 0x60, 0x00002002 }, { 0x48, 0x8a800090 }, { 0xe8, 0x40000000 },
        { 0xb0, 0x00000003 }, { 0xb8, 0xffffffff } } },
    { "t20x", "ingenic-t20", "ingenic-t31", true,
      { { 0x60, 0x00002002 }, { 0x48, 0x8a800090 }, { 0xe8, 0x40000000 },
        { 0xb0, 0x00000001 } } },
    { "t21n", "ingenic-t21", "ingenic-t31", true,
      { { 0x60, 0x00100002 }, { 0x70, 0x00000001 },
        { 0xf0, 0x01041041 }, { 0xf4, 0x01041041 }, { 0xf8, 0xffffffff },
        { 0x40, 0x00000004 }, { 0x48, 0x8a800090 }, { 0x8c, 0x00000002 },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "t23n", "ingenic-t23", "ingenic-t31", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 },
        { 0x40, 0x00000004 }, { 0x48, 0x8a800090 }, { 0x8c, 0x00000020 },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "t30x", "ingenic-t30", "ingenic-t31", true,
      { { 0x60, 0x00100002 }, { 0x58, 0x06305040 },
        { 0xf0, 0x01041041 }, { 0xf4, 0x01041041 }, { 0xf8, 0xffffffff },
        { 0x40, 0x00000004 }, { 0x48, 0x8a800090 }, { 0x8c, 0x00000002 },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "t31x", "ingenic-t31", "ingenic-t31", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 },
        { 0x40, 0x00000004 }, { 0x48, 0x8a800090 }, { 0x8c, 0x00000020 },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "t32lq", "ingenic-t32", "ingenic-t31", true,
      { { 0x70, 0x00100002 }, { 0x78, 0x00000001 },
        { 0x84, 0x00100002 }, { 0x88, 0x00000001 },
        { 0x40, 0x00000004 }, { 0x48, 0x8a800090 }, { 0x8c, 0x00000020 },
        { 0xe8, 0x80000000 }, { 0x58, 0x00000100 }, { 0xa4, 0x00108000 },
        { 0xac, 0x00000100 }, { 0xd0, 0x00000001 }, { 0xb0, 0x00000001 } } },
    { "t40n", "ingenic-t40", "ingenic-t40", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 },
        { 0x3c, 0x400919b8 }, { 0x40, 0x00000004 }, { 0x48, 0x8a800090 },
        { 0x68, 0x00108000 }, { 0x6c, 0x00108000 }, { 0x8c, 0x00000006 },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "t40xp", "ingenic-t40", "ingenic-t40", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 },
        { 0x3c, 0x400919b8 }, { 0x40, 0x00000004 }, { 0x48, 0x8a800090 },
        { 0x68, 0x00108000 }, { 0x6c, 0x00108000 }, { 0x8c, 0x00000006 },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "t41lq", "ingenic-t41", "ingenic-t40", true,
      { { 0x70, 0x00100002 }, { 0x84, 0x00100002 },
        { 0x3c, 0x400919b8 }, { 0x40, 0x00000004 }, { 0x48, 0x8a800090 },
        { 0x68, 0x00108000 }, { 0x6c, 0x00108000 }, { 0x8c, 0x0000008c },
        { 0xe8, 0x40000000 }, { 0xb0, 0x00000001 } } },
    { "a1n", "ingenic-a1", "ingenic-a1", false,
      { { 0x08, 0x00000001 }, { 0x0c, 0x00000020 }, { 0x34, 0x00001500 },
        { 0x48, 0x0000a5a5 }, { 0xa0, 0x00100002 }, { 0xa8, 0x00100002 },
        { 0x100, 0x00100002 }, { 0x108, 0x00100002 },
        { 0x50, 0x400919b8 }, { 0x5c, 0x8a800090 }, { 0x98, 0x00108000 },
        { 0x9c, 0x00108000 }, { 0xb0, 0x00000800 } } },
};

static void check(QTestState *qts, const char *part, const Reg *r, int n)
{
    for (int i = 0; i < n && r[i].value; i++) {
        g_test_message("%s 0x%02x", part, r[i].offset);
        g_assert_cmphex(qtest_readl(qts, CPM + r[i].offset), ==, r[i].value);
    }
}

static void test_case(const void *data)
{
    const CpmCase *c = data;
    QTestState *qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                                  c->machine, c->soc, c->part);

    if (c->t_series) {
        check(qts, c->part, t_series, ARRAY_SIZE(t_series));
    } else {
        g_assert_cmphex(qtest_readl(qts, CPM + A1_CPCSR), ==, 0x7c000000);
    }
    check(qts, c->part, c->regs, ARRAY_SIZE(c->regs));
    g_assert_cmphex(qtest_readl(qts, CPM + CPAPCR) & PLL_ON_LOCK_EN, ==,
                    PLL_ON_LOCK_EN);
    qtest_quit(qts);
}

static void test_usb_phy(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);
    bool t20 = !strcmp(machine, "ingenic-t20");

    qtest_writel(qts, CPM + 0x40, 0x0a000096);
    g_assert_cmphex(qtest_readl(qts, CPM + 0x40), ==,
                    t20 ? 0x0a000096 : 0x08000006);
    qtest_writel(qts, CPM + 0x40, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CPM + 0x40), ==,
                    t20 ? 0xffffffff : 0x88000007);
    qtest_writel(qts, CPM + 0xe8, 0x00000004);
    g_assert_cmphex(qtest_readl(qts, CPM + 0xe8), ==, 0x40000004);
    qtest_writel(qts, CPM + 0x48, 0x10000000);
    g_assert_cmphex(qtest_readl(qts, CPM + 0x48), ==, 0x10000000);
    qtest_quit(qts);
}

static void test_i2s_duty(const void *data)
{
    const char *machine = data;
    QTestState *qts = qtest_initf("-machine %s", machine);

    if (!strcmp(machine, "ingenic-t21")) {
        qtest_writel(qts, CPM + 0x60, 0x21000d2f);
        g_assert_cmphex(qtest_readl(qts, CPM + 0x60), ==, 0x21000d2f);
        g_assert_cmphex(qtest_readl(qts, CPM + 0x70), ==, 0x697);
        qtest_writel(qts, CPM + 0x70, 0x40000123);
        g_assert_cmphex(qtest_readl(qts, CPM + 0x70), ==, 0x40000123);
        qtest_writel(qts, CPM + 0x74, 0x58000017);
        g_assert_cmphex(qtest_readl(qts, CPM + 0x74), ==, 0x50000017);
    } else {
        qtest_writel(qts, CPM + 0x70, 0x28006ddd);
        qtest_writel(qts, CPM + 0x84, 0x02000777);
        g_assert_cmphex(qtest_readl(qts, CPM + 0x78), ==, 0x36ee);
        g_assert_cmphex(qtest_readl(qts, CPM + 0x88), ==, 0x3bb);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-cpm/%s",
                                                cases[i].part);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    qtest_add_data_func("ingenic-cpm/usb-phy/t20", "ingenic-t20", test_usb_phy);
    qtest_add_data_func("ingenic-cpm/usb-phy/t31", "ingenic-t31", test_usb_phy);
    qtest_add_data_func("ingenic-cpm/usb-phy/t40", "ingenic-t40", test_usb_phy);
    qtest_add_data_func("ingenic-cpm/i2s-duty/t21", "ingenic-t21",
                        test_i2s_duty);
    qtest_add_data_func("ingenic-cpm/i2s-duty/t32", "ingenic-t32",
                        test_i2s_duty);
    return g_test_run();
}
