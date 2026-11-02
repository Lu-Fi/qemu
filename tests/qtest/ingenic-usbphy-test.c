/*
 * QTest testcase for the Ingenic USB PHY
 *
 * Per part, registers that tell its layout apart, as the part reads them
 * with the OTG clock open, and what a write of all ones keeps: a T20X, a
 * T31X (the T21 and T30 alike), a T23N (the T32 alike), a T40XP, a T41NQ
 * and an A1N, whose three PHYs sit 1 KiB apart and whose fourth KiB reads
 * 0; a part with one PHY answers at every KiB of the window. The status
 * registers keep no write. The T10's window reads 0 and keeps nothing.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define PHY             0x10060000

typedef struct {
    uint32_t offset;
    uint8_t reset, kept;        /* out of reset; the bits a write keeps */
} Reg;

typedef struct {
    const char *machine;
    int phys;
    Reg regs[8];                /* a zero offset ends the list */
} PhyCase;

static const PhyCase cases[] = {
    { "ingenic-t10", 0, { { 0x040, 0, 0 }, { 0x140, 0, 0 } } },
    { "ingenic-t20", 1, { { 0x040, 0x61, 0xff }, { 0x048, 0x0e, 0xff },
                          { 0x100, 0x00, 0x1f }, { 0x11c, 0x00, 0x00 },
                          { 0x140, 0x8f, 0x00 }, { 0x144, 0x7f, 0x00 } } },
    { "ingenic-t21", 1, { { 0x040, 0x49, 0xff }, { 0x010, 0x00, 0x7f },
                          { 0x140, 0x80, 0x00 }, { 0x14c, 0x10, 0x00 } } },
    { "ingenic-t30", 1, { { 0x040, 0x49, 0xff }, { 0x11c, 0xb5, 0xff } } },
    { "ingenic-t31", 1, { { 0x040, 0x49, 0xff }, { 0x048, 0x0d, 0xff },
                          { 0x0bc, 0x00, 0x3f }, { 0x11c, 0xb5, 0xff },
                          { 0x140, 0x80, 0x00 }, { 0x180, 0x00, 0x00 } } },
    { "ingenic-t23", 1, { { 0x040, 0x69, 0xff }, { 0x064, 0x80, 0xff },
                          { 0x128, 0x05, 0xff }, { 0x160, 0x00, 0x03 },
                          { 0x180, 0xf2, 0xff }, { 0x164, 0x00, 0x00 } } },
    { "ingenic-t32", 1, { { 0x040, 0x69, 0xff }, { 0x108, 0x10, 0xff } } },
    { "ingenic-t40", 1, { { 0x040, 0x49, 0xff }, { 0x064, 0xe0, 0xff },
                          { 0x164, 0x03, 0x00 }, { 0x180, 0xf2, 0xff } } },
    { "ingenic-t41", 1, { { 0x0bc, 0x00, 0x7f }, { 0x114, 0x00, 0x07 },
                          { 0x160, 0x00, 0x01 }, { 0x190, 0x00, 0x00 },
                          { 0x1c0, 0xdb, 0xff }, { 0x1c4, 0x0c, 0x3f } } },
    { "ingenic-a1", 3, { { 0x048, 0x0e, 0xff }, { 0x108, 0x10, 0xff },
                         { 0x170, 0x03, 0x00 }, { 0x180, 0xf3, 0xff } } },
};

static void test_phy(const void *data)
{
    const PhyCase *c = data;
    QTestState *qts = qtest_initf("-machine %s", c->machine);

    for (int p = 0; p < 4; p++) {
        uint32_t base = PHY + p * 0x400;
        bool here = c->phys == 1 || p < c->phys;

        for (int i = 0; i < ARRAY_SIZE(c->regs) && c->regs[i].offset; i++) {
            const Reg *r = &c->regs[i];

            g_test_message("phy %d 0x%03x", p, r->offset);
            g_assert_cmphex(qtest_readl(qts, base + r->offset), ==,
                            here ? r->reset : 0);
            g_assert_cmphex(qtest_readb(qts, base + r->offset + 1), ==, 0);
            qtest_writel(qts, base + r->offset, 0xffffffff);
            g_assert_cmphex(qtest_readl(qts, base + r->offset), ==,
                            here ? r->kept | r->reset : 0);
            qtest_writeb(qts, base + r->offset, 0);
            g_assert_cmphex(qtest_readb(qts, base + r->offset), ==,
                            here ? r->reset & ~r->kept & 0xff : 0);
            qtest_writel(qts, base + r->offset, r->reset);
        }
    }
    /* One PHY's registers are the same at every KiB */
    if (c->phys == 1) {
        qtest_writel(qts, PHY + 0x4b0, 0x02);
        g_assert_cmphex(qtest_readl(qts, PHY + 0xcb0), ==, 0x02);
    }
    /* A reset puts every PHY back as it powers on */
    qtest_system_reset(qts);
    for (int i = 0; i < ARRAY_SIZE(c->regs) && c->regs[i].offset; i++) {
        g_assert_cmphex(qtest_readl(qts, PHY + c->regs[i].offset), ==,
                        c->phys ? c->regs[i].reset : 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-usbphy/%s",
                                                cases[i].machine + 8);

        qtest_add_data_func(path, &cases[i], test_phy);
    }
    return g_test_run();
}
