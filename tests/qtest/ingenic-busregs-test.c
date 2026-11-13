/*
 * QTest testcase for the Ingenic bus windows that only hold registers
 *
 * HARB0's chip ID at 0x2c of every 0x40, keeping no write; the T32's
 * HARB0, 32 words repeating every 0x80 that all keep a write. The AXI
 * arbiters' version and configuration, and the word that keeps a write.
 * The bus monitors' two kinds of channel, a T40's and an A1's repeating
 * every 4 KiB, and a T41's channels and control block. Values and the bits
 * kept are as a T31X, T32LQ, T40XP, T41NQ and A1N read them. The SDHCI's
 * DesignWare MSHC areas, the same on every part that has one: pointers at
 * 0xe0, command-queue and vendor registers from 0x180, bits that a 1 sets
 * and a 0 leaves.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define HARB0           0x13000000
#define MONITOR         0x130a0000
#define AXI_ARB1        0x13013000
#define AXI_ARB2        0x13014000
#define HARB2           0x13400000

/* Reads reset, then all ones written reads kept | (reset & ~mask) */
static void check(QTestState *qts, uint32_t addr, uint32_t reset,
                  uint32_t mask)
{
    g_test_message("0x%08x", addr);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset);
    qtest_writel(qts, addr, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset | mask);
    qtest_writel(qts, addr, 0);
    g_assert_cmphex(qtest_readl(qts, addr), ==, reset & ~mask);
    qtest_writel(qts, addr, reset);
}

static void test_harb0(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    check(qts, HARB0 + 0x2c, 0x10031003, 0);
    check(qts, HARB0 + 0xfec, 0x10031003, 0);
    check(qts, HARB0 + 0x30, 0, 0);
    qtest_quit(qts);
}

static void test_harb0_t32(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    check(qts, HARB0 + 0x2c, 0x10032004, 0xffffffff);
    check(qts, HARB0 + 0x30, 0xee, 0xffffffff);
    check(qts, HARB0 + 0x70, 0xffdd, 0xffffffff);
    check(qts, HARB0 + 0x00, 0, 0);
    qtest_writel(qts, HARB0 + 0x04, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB0 + 0xf84), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB0 + 0x6c), ==, 0);
    qtest_quit(qts);
}

/*
 * Above the RAM aliases nothing answers on silicon and no bus error comes:
 * the model reads 0 as the stand-in for stale bus data, drops stores, and
 * keeps a T32LQ's mirrors of its RAM on top
 */
static void test_floor(const void *data)
{
    static const uint32_t probes[] = {
        0x30400000, 0x40000000, 0x60000000, 0x80400000, 0x9ff00000,
        0xf0000000, 0xfffffffc,
    };
    QTestState *qts = qtest_init(data);
    g_autofree char *tree = qtest_hmp(qts, "info mtree -f");

    g_assert_nonnull(strstr(tree, "ingenic-bus-floor"));
    for (int i = 0; i < ARRAY_SIZE(probes); i++) {
        g_assert_cmphex(qtest_readl(qts, probes[i]), ==, 0);
        qtest_writel(qts, probes[i], 0x5a5a5a5a);
        g_assert_cmphex(qtest_readl(qts, probes[i]), ==, 0);
    }
    qtest_quit(qts);
}

static void test_floor_t32lq(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 -m 128M "
                                 "-global ingenic-t31.soc-variant=t32lq");

    qtest_writel(qts, 0x00010000, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0x30010000), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0x80010000), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0xa0000000), ==, 0);
    qtest_quit(qts);
}

static void test_t40(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    /* A channel, then the other kind at 0x500, repeating every 4 KiB */
    check(qts, MONITOR + 0x000, 0xff00, 0xe007ff07);
    check(qts, MONITOR + 0x608, 0, 0x80ff80ff);
    check(qts, MONITOR + 0x474, 0x7f, 0x7f);
    check(qts, MONITOR + 0x500, 0, 0xf000f003);
    check(qts, MONITOR + 0x574, 3, 3);
    check(qts, MONITOR + 0x700, 0, 0);
    g_assert_cmphex(qtest_readl(qts, MONITOR + 0xf000), ==, 0xff00);
    check(qts, AXI_ARB1 + 0x000, 0x3430302a, 0);
    check(qts, AXI_ARB1 + 0x004, 0x00102001, 0);
    check(qts, AXI_ARB1 + 0x00c, 0, 0xffffffff);
    check(qts, AXI_ARB2 + 0x004, 0x00105001, 0);
    /* Below GPIO port A, as a T40XP reads it: 0, and stores are dropped */
    for (unsigned off = 0; off < 0x100; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x1000f000 + off), ==, 0);
    }
    qtest_writel(qts, 0x1000f018, 0x80000000);
    qtest_writel(qts, 0x1000f044, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, 0x1000f018), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x1000f044), ==, 0);
    qtest_quit(qts);
}

static void test_t41(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    check(qts, MONITOR + 0x0020, 0x200, 0xffff);
    check(qts, MONITOR + 0x1824, 0x200, 0xffff);
    check(qts, MONITOR + 0x1924, 0x10000000, 0xffffffff);
    check(qts, MONITOR + 0x1400, 0, 0xffffffff);
    check(qts, MONITOR + 0x1420, 0, 0);
    check(qts, MONITOR + 0x2300, 0xffffffff, 0xffffffff);
    check(qts, MONITOR + 0x2400, 0, 0);
    /* Below GPIO port A, as a T41NQ reads it: 0, and stores are dropped */
    for (unsigned off = 0; off < 0x100; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x1000f000 + off), ==, 0);
    }
    qtest_writel(qts, 0x1000f038, 0x5a5a5a5a);
    g_assert_cmphex(qtest_readl(qts, 0x1000f038), ==, 0);
    qtest_quit(qts);
}

static void test_a1(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    check(qts, MONITOR + 0x700, 0, 0xf000f003);
    check(qts, MONITOR + 0x774, 3, 3);
    check(qts, MONITOR + 0x60c, 0xfff, 0xfff);
    check(qts, AXI_ARB2 + 0x000, 0x3430302a, 0);
    qtest_quit(qts);
}

typedef struct {
    const char *args;
    uint32_t r4, r8, m8, r28;
} Harb2Part;

/*
 * HARB2, 16 words repeating every 0x40: reset values as each part reads
 * them at U-Boot, the bits a write keeps
 */
static void test_harb2(const void *data)
{
    const Harb2Part *p = data;
    QTestState *qts = qtest_init(p->args);

    check(qts, HARB2 + 0x00, 0, 0x003fffff);
    check(qts, HARB2 + 0x04, p->r4, 0x00ff7707);
    check(qts, HARB2 + 0x08, p->r8, p->m8);
    check(qts, HARB2 + 0x18, 0, 0xffff0107);
    check(qts, HARB2 + 0x28, p->r28, 0xffffffff);
    check(qts, HARB2 + 0x2c, 0, 0);
    check(qts, HARB2 + 0x30, 0, 0);
    /* the registers repeat: a write at 0x114 shows at 0x14 and 0xf54 */
    qtest_writel(qts, HARB2 + 0x114, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB2 + 0x14), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB2 + 0xf54), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HARB2 + 0x118), ==, 0);
    qtest_quit(qts);
}

/* Below GPIO port A, a T31X reads 0 and drops writes */
static void test_gpio_gap(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");

    for (unsigned off = 0; off < 0x1000; off += 4) {
        g_assert_cmphex(qtest_readl(qts, 0x1000f000 + off), ==, 0);
    }
    qtest_writel(qts, 0x1000f018, 0x80000000);
    qtest_writel(qts, 0x1000f024, 0x80000000);
    qtest_writel(qts, 0x1000f038, 0x80000000);
    g_assert_cmphex(qtest_readl(qts, 0x1000f018), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x1000f038), ==, 0);
    qtest_quit(qts);
}

typedef struct {
    const char *machine;
    uint32_t zero[8];           /* words that read 0 and drop writes */
    bool ssi_slv;
} QuietPart;

/* Windows that read 0 and drop writes, and the SSI slave's two fixed words */
static void test_quiet(const void *data)
{
    const QuietPart *p = data;
    QTestState *qts = qtest_initf("-machine %s", p->machine);

    for (int i = 0; i < ARRAY_SIZE(p->zero) && p->zero[i]; i++) {
        qtest_writel(qts, p->zero[i], 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, p->zero[i]), ==, 0);
    }
    if (p->ssi_slv) {
        check(qts, 0x10040000, 0x40945e80, 0);
        check(qts, 0x1004001c, 0x00800080, 0);
        g_assert_cmphex(qtest_readl(qts, 0x10040f1c), ==, 0x00800080);
        g_assert_cmphex(qtest_readl(qts, 0x10040004), ==, 0);
    }
    qtest_quit(qts);
}

static void test_mshc(const void *data)
{
    QTestState *qts = qtest_initf("-machine %s", (const char *)data);

    for (uint32_t base = 0x13060000; base <= 0x13070000; base += 0x10000) {
        check(qts, base + 0xe0, 0x0f580f48, 0);
        check(qts, base + 0xe8, 0x01800500, 0);
        check(qts, base + 0x180, 0x510, 0);
        check(qts, base + 0x1c0, 0x11000, 0xfffff);
        check(qts, base + 0x500, 0x3134302a, 0);
        check(qts, base + 0x510, 0x0303000f, 0x0303030f);
        check(qts, base + 0xf58, 0x40000, 0);
        /* 0x19c's low bits stay once set */
        qtest_writel(qts, base + 0x19c, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, base + 0x19c), ==, 0x80001f7f);
        qtest_writel(qts, base + 0x19c, 0);
        g_assert_cmphex(qtest_readl(qts, base + 0x19c), ==, 0x1f7f);
    }
    qtest_quit(qts);
}

static void test_a1_sata_vs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    /* What the VGA PHY sample polls on an A1N: 0x17c never loses bits 6:5 */
    g_assert_cmphex(qtest_readl(qts, 0x130d0178), ==, 0x10001110);
    g_assert_cmphex(qtest_readl(qts, 0x130d017c), ==, 0x2020c160);
    qtest_writel(qts, 0x130d017c, 0);
    g_assert_cmphex(qtest_readl(qts, 0x130d017c), ==, 0x2020c160);
    /* assumed: the control word keeps writes */
    qtest_writel(qts, 0x130d0178, 0x10001111);
    g_assert_cmphex(qtest_readl(qts, 0x130d0178), ==, 0x10001111);
    /* the AHCI around it is untouched: port 0 PxSSTS at 0x128 */
    g_assert_cmphex(qtest_readl(qts, 0x130d0128), ==, 0);
    g_assert_cmphex(qtest_readl(qts, 0x130d0174), ==, 0);
    qtest_quit(qts);
}

#define A1_CLKGR1       0x10000038
#define A1_SATA_PHY0    0x10080000
#define A1_SATA_PHY1    0x10090000

static void test_a1_sata_phy(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    /* gate_sata (CLKGR1 bit 11) shut: reads 0, writes dropped */
    g_assert_cmphex(qtest_readl(qts, A1_CLKGR1) & 0x800, ==, 0x800);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 4), ==, 0);
    qtest_writel(qts, A1_SATA_PHY0 + 4, 0x12345);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 4), ==, 0);
    qtest_writel(qts, A1_SATA_PHY1 + 0x3c8, 0xf);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY1 + 0x3c8), ==, 0);

    /* open: the reset values, as the U-Boot reads them with it open */
    qtest_writel(qts, A1_CLKGR1, qtest_readl(qts, A1_CLKGR1) & ~0x800);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 4), ==, 0x2b006);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY1 + 4), ==, 0x2b006);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 0x3c8), ==, 0);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY1 + 0x3c8), ==, 0xe);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 0x7cc), ==, 0xce00ce);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY1 + 0x7cc), ==, 0x9e009e);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 0xc00), ==, 0x18);

    /* 32 bits kept; the low byte of 0xc00; none of the 16 fixed words */
    check(qts, A1_SATA_PHY0, 0x80004080, 0xffffffff);
    check(qts, A1_SATA_PHY1 + 0x138, qtest_readl(qts, A1_SATA_PHY1 + 0x138),
          0xffffffff);
    check(qts, A1_SATA_PHY0 + 0xc00, 0x18, 0xff);
    check(qts, A1_SATA_PHY0 + 0x73c, 0x03fc026a, 0);
    check(qts, A1_SATA_PHY1 + 0x1fcc, 0xc0, 0);
    /* nothing above 0x2000 */
    check(qts, A1_SATA_PHY0 + 0x2000, 0, 0);
    check(qts, A1_SATA_PHY0 + 0xfffc, 0, 0);

    /* what was written comes back when the gate opens again */
    qtest_writel(qts, A1_SATA_PHY0 + 4, 0x777);
    qtest_writel(qts, A1_CLKGR1, qtest_readl(qts, A1_CLKGR1) | 0x800);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 4), ==, 0);
    qtest_writel(qts, A1_SATA_PHY0 + 4, 0x999);
    qtest_writel(qts, A1_CLKGR1, qtest_readl(qts, A1_CLKGR1) & ~0x800);
    g_assert_cmphex(qtest_readl(qts, A1_SATA_PHY0 + 4), ==, 0x777);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-busregs/harb0", test_harb0);
    qtest_add_func("ingenic-busregs/harb0-t32", test_harb0_t32);
    qtest_add_func("ingenic-busregs/t40", test_t40);
    qtest_add_data_func("ingenic-busregs/floor/t31", "-machine ingenic-t31",
                        test_floor);
    qtest_add_data_func("ingenic-busregs/floor/t40", "-machine ingenic-t40",
                        test_floor);
    qtest_add_data_func("ingenic-busregs/floor/t41", "-machine ingenic-t41",
                        test_floor);
    qtest_add_func("ingenic-busregs/floor/t32lq", test_floor_t32lq);
    qtest_add_func("ingenic-busregs/t41", test_t41);
    qtest_add_func("ingenic-busregs/a1", test_a1);
    qtest_add_func("ingenic-busregs/a1-sata-vs", test_a1_sata_vs);
    qtest_add_func("ingenic-busregs/a1-sata-phy", test_a1_sata_phy);
    qtest_add_data_func("ingenic-busregs/mshc/t32", "ingenic-t32", test_mshc);
    qtest_add_data_func("ingenic-busregs/mshc/t41", "ingenic-t41", test_mshc);
    qtest_add_data_func("ingenic-busregs/mshc/a1", "ingenic-a1", test_mshc);
    {
        static const Harb2Part t31 = {
            "-machine ingenic-t31", 0x00ad3100, 0xcc05040b, 0xffffffff,
            0xc0008042
        };
        static const Harb2Part t32 = {
            "-machine ingenic-t32", 0x00155600, 0x01441880, 0x312, 0x30808113
        };
        static const Harb2Part t23 = {
            "-machine ingenic-t31 -global ingenic-t31.soc-variant=t23n",
            0x00477200, 0x050c1203, 0x32c, 0x01030708
        };

        qtest_add_data_func("ingenic-busregs/harb2/t31", &t31, test_harb2);
        qtest_add_data_func("ingenic-busregs/harb2/t32", &t32, test_harb2);
        qtest_add_data_func("ingenic-busregs/harb2/t23", &t23, test_harb2);
    }
    {
        static const Harb2Part t40 = {
            "-machine ingenic-t40", 0x00cc4400, 0xffffffc0, 0xffffffff,
            0x1270b889
        };
        static const Harb2Part t41 = {
            "-machine ingenic-t41", 0x003d3200, 0xdcff7fff, 0xffffffff,
            0xffbff3ff
        };
        static const Harb2Part a1 = {
            "-machine ingenic-a1", 0x00fd7400, 0x5ba36536, 0xffffffff,
            0x7bf367ef
        };

        qtest_add_data_func("ingenic-busregs/harb2/t40", &t40, test_harb2);
        qtest_add_data_func("ingenic-busregs/harb2/t41", &t41, test_harb2);
        qtest_add_data_func("ingenic-busregs/harb2/a1", &a1, test_harb2);
    }
    qtest_add_func("ingenic-busregs/gpio-gap", test_gpio_gap);
    {
        static const QuietPart t40 = {
            "ingenic-t40",
            { 0x10003000, 0x10003ffc, 0x10004000, 0x10004ffc, 0x12210000,
              0x1221fffc, 0x12700000, 0x1270fffc }, true
        };
        static const QuietPart t41 = {
            "ingenic-t41",
            { 0x10004000, 0x10004ffc, 0x13013000, 0x13013ffc, 0x13014000,
              0x13014ffc }, true
        };
        static const QuietPart a1 = {
            "ingenic-a1", { 0x13013000, 0x13013ffc }, false
        };

        qtest_add_data_func("ingenic-busregs/quiet/t40", &t40, test_quiet);
        qtest_add_data_func("ingenic-busregs/quiet/t41", &t41, test_quiet);
        qtest_add_data_func("ingenic-busregs/quiet/a1", &a1, test_quiet);
    }
    return g_test_run();
}
