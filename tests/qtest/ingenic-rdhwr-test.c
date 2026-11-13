/*
 * QTest testcase for RDHWR $4 and HWREna on the Ingenic CPUs
 *
 * As measured at U-Boot on a T30X, a T40XP and a T41NQ, in kernel mode
 * with HWREna all ones: RDHWR $4 raises Reserved Instruction, before any
 * LL and after one (the vendor NNA driver's "ll; rdhwr $4" does not give a
 * physical address). RDHWR $1 reads 0x20 on the XBurst2 parts. HWREna
 * reads 0 after reset and keeps bits 0 to 4 on an XBurst1 (T30X) and bits
 * 0 to 3, 8 to 15 and 29 on an XBurst2 (T40XP, T41NQ).
 *
 * A program run under TCG installs its own exception handler at 0x180,
 * which counts the exceptions, keeps the last Cause and skips the
 * instruction.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define PROG_LOAD       0x80010000
#define PROG_OUT        0x00200000
#define PROG_DONE       0x00201000
#define OUT_COUNT       0x40
#define OUT_CAUSE       0x44

enum { ZERO = 0, T0 = 8, T1, T2, T3, T4, K0 = 26, K1 };

static uint32_t lui(int rt, uint16_t imm)
{
    return 0x3c000000 | rt << 16 | imm;
}

static uint32_t ori(int rt, int rs, uint16_t imm)
{
    return 0x34000000 | rs << 21 | rt << 16 | imm;
}

static uint32_t addiu(int rt, int rs, uint16_t imm)
{
    return 0x24000000 | rs << 21 | rt << 16 | imm;
}

static uint32_t sw(int rt, int base, uint16_t off)
{
    return 0xac000000 | base << 21 | rt << 16 | off;
}

static uint32_t lw(int rt, int base, uint16_t off)
{
    return 0x8c000000 | base << 21 | rt << 16 | off;
}

static uint32_t ll(int rt, int base, uint16_t off)
{
    return 0xc0000000 | base << 21 | rt << 16 | off;
}

static uint32_t mfc0(int rt, int rd)
{
    return 0x40000000 | rt << 16 | rd << 11;
}

static uint32_t mtc0(int rt, int rd)
{
    return 0x40800000 | rt << 16 | rd << 11;
}

static uint32_t rdhwr(int rt, int rd)
{
    return 0x7c00003b | rt << 16 | rd << 11;
}

#define EHB     0x000000c0
#define ERET    0x42000018

static char *image(void)
{
    size_t size = 4096;
    g_autofree uint8_t *img = g_malloc0(64 + size);
    uint8_t *code = img + 64;
    unsigned k = 0, n = 0, i;
    char *path;
    int fd = g_file_open_tmp("ingenic-rdhwr-prog-XXXXXX", &path, NULL);
    const uint32_t handler[] = {
        lui(K1, 0x8020),
        mfc0(K0, 13),
        sw(K0, K1, OUT_CAUSE),
        lw(K0, K1, OUT_COUNT),
        addiu(K0, K0, 1),
        sw(K0, K1, OUT_COUNT),
        mfc0(K0, 14),
        addiu(K0, K0, 4),
        mtc0(K0, 14),
        EHB,
        ERET,
    };

#define I(x) stl_le_p(&code[4 * k++], (x))
    I(lui(T1, 0x8020));
    /* Status: CU0 only, so the exceptions vector to 0x80000180 */
    I(lui(T0, 0x1000));
    I(mtc0(T0, 12));
    I(EHB);
    I(lui(T2, 0x8000));
    I(ori(T2, T2, 0x180));
    for (i = 0; i < ARRAY_SIZE(handler); i++) {
        I(lui(T3, handler[i] >> 16));
        I(ori(T3, T3, handler[i] & 0xffff));
        I(sw(T3, T2, 4 * i));
    }
    /* HWREna after reset, then after writing all ones */
    I(mfc0(T0, 7));
    I(sw(T0, T1, 4 * n++));
    I(addiu(T0, ZERO, 0xffff));
    I(mtc0(T0, 7));
    I(EHB);
    I(mfc0(T0, 7));
    I(sw(T0, T1, 4 * n++));
    /* RDHWR $4 with no LL before it; the marker stays if it raises RI */
    I(lui(T4, 0xdead));
    I(ori(T4, T4, 0xbeef));
    I(rdhwr(T4, 4));
    I(sw(T4, T1, 4 * n++));
    /* and after an LL */
    I(lui(T2, 0x8030));
    I(ll(T3, T2, 0));
    I(lui(T4, 0xdead));
    I(ori(T4, T4, 0xbeef));
    I(rdhwr(T4, 4));
    I(sw(T4, T1, 4 * n++));
    /* RDHWR $1 as the control */
    I(lui(T4, 0xdead));
    I(ori(T4, T4, 0xbeef));
    I(rdhwr(T4, 1));
    I(sw(T4, T1, 4 * n++));
    I(ori(T2, ZERO, 0x600d));
    I(sw(T2, T1, 0x1000));
    I(0x1000ffff);                      /* b . */
    I(0);
#undef I
    g_assert(4 * k <= size);

    stl_be_p(img, 0x27051956);
    stl_be_p(img + 12, size);
    stl_be_p(img + 16, PROG_LOAD);
    stl_be_p(img + 20, PROG_LOAD);
    img[28] = 5;                        /* Linux */
    img[29] = 5;                        /* MIPS */
    img[30] = 2;                        /* kernel */
    g_assert(write(fd, img, 64 + size) == 64 + size);
    close(fd);
    return path;
}

typedef struct {
    const char *machine;
    uint32_t hwrena_ones;
    uint32_t synci;     /* RDHWR $1, 0 where not measured */
} Part;

static void test_rdhwr(const void *data)
{
    const Part *p = data;
    g_autofree char *img = image();
    QTestState *qts = qtest_initf("-machine %s -m 128 -kernel %s -accel tcg",
                                  p->machine, img);
    int i;

    for (i = 0; i < 1000 && qtest_readl(qts, PROG_DONE) != 0x600d; i++) {
        g_usleep(10 * 1000);
    }
    g_assert_cmphex(qtest_readl(qts, PROG_DONE), ==, 0x600d);
    g_assert_cmphex(qtest_readl(qts, PROG_OUT), ==, 0);
    g_assert_cmphex(qtest_readl(qts, PROG_OUT + 4), ==, p->hwrena_ones);
    g_assert_cmphex(qtest_readl(qts, PROG_OUT + 8), ==, 0xdeadbeef);
    g_assert_cmphex(qtest_readl(qts, PROG_OUT + 12), ==, 0xdeadbeef);
    if (p->synci) {
        g_assert_cmphex(qtest_readl(qts, PROG_OUT + 16), ==, p->synci);
    } else {
        g_assert_cmphex(qtest_readl(qts, PROG_OUT + 16), !=, 0xdeadbeef);
    }
    /* the two RDHWR $4 raised Reserved Instruction, the control nothing */
    g_assert_cmpuint(qtest_readl(qts, PROG_OUT + OUT_COUNT), ==, 2);
    g_assert_cmphex(qtest_readl(qts, PROG_OUT + OUT_CAUSE) & 0x7c, ==, 10 << 2);
    qtest_quit(qts);
    unlink(img);
}

int main(int argc, char **argv)
{
    static const Part xburst1 = { "ingenic-t30", 0x0000001f, 0 };
    static const Part t31 = { "ingenic-t31", 0x0000001f, 0 };
    static const Part t40 = { "ingenic-t40", 0x2000ff0f, 0x20 };
    static const Part t41 = { "ingenic-t41", 0x2000ff0f, 0x20 };

    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("ingenic-rdhwr/t30", &xburst1, test_rdhwr);
    qtest_add_data_func("ingenic-rdhwr/t31", &t31, test_rdhwr);
    qtest_add_data_func("ingenic-rdhwr/t40", &t40, test_rdhwr);
    qtest_add_data_func("ingenic-rdhwr/t41", &t41, test_rdhwr);
    return g_test_run();
}
