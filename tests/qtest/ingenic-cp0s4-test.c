/*
 * QTest testcase for CP0 register 9 select 4 of the T32 and T33 core
 *
 * A program run under TCG reads it, writes 0, sets bit 1 and writes all
 * ones, storing what each read returns. A T32 or T33 holds the written bits
 * except bit 1, which reads 0 as the bootroms' write leaves it; the other
 * T-series CPUs do not have the register.
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

enum { T0 = 8, T1, T2 };

static uint32_t lui(int rt, uint16_t imm)
{
    return 0x3c000000 | rt << 16 | imm;
}

static uint32_t ori(int rt, int rs, uint16_t imm)
{
    return 0x34000000 | rs << 21 | rt << 16 | imm;
}

static uint32_t sw(int rt, int base, uint16_t off)
{
    return 0xac000000 | base << 21 | rt << 16 | off;
}

static uint32_t mfc0_9_4(int rt)
{
    return 0x40004804 | rt << 16;
}

static uint32_t mtc0_9_4(int rt)
{
    return 0x40804804 | rt << 16;
}

static char *cp0_image(void)
{
    size_t size = 4096;
    g_autofree uint8_t *img = g_malloc0(64 + size);
    uint8_t *code = img + 64;
    unsigned k = 0, n = 0;
    char *path;
    int fd = g_file_open_tmp("ingenic-cp0s4-prog-XXXXXX", &path, NULL);

#define I(x) stl_le_p(&code[4 * k++], (x))
    I(lui(T1, 0x8020));
    /* the value as it is, then 0, then bit 1, then all ones */
    I(mfc0_9_4(T2));
    I(sw(T2, T1, 4 * n++));
    I(mtc0_9_4(0));
    I(mfc0_9_4(T2));
    I(sw(T2, T1, 4 * n++));
    I(ori(T0, 0, 2));
    I(mtc0_9_4(T0));
    I(mfc0_9_4(T2));
    I(sw(T2, T1, 4 * n++));
    I(0x2408ffff);                      /* li t0, -1 */
    I(mtc0_9_4(T0));
    I(mfc0_9_4(T2));
    I(sw(T2, T1, 4 * n++));
    I(ori(T2, 0, 0x600d));
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

static void run(const char *machine, const uint32_t want[4])
{
    g_autofree char *image = cp0_image();
    QTestState *qts = qtest_initf("-machine %s -m 128 -kernel %s"
                                  " -accel tcg", machine, image);
    int i;

    for (i = 0; i < 1000 && qtest_readl(qts, PROG_DONE) != 0x600d; i++) {
        g_usleep(10 * 1000);
    }
    g_assert_cmphex(qtest_readl(qts, PROG_DONE), ==, 0x600d);
    for (i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(qts, PROG_OUT + 4 * i), ==, want[i]);
    }
    qtest_quit(qts);
    unlink(image);
}

static void test_btbctl(const void *machine)
{
    static const uint32_t want[4] = {
        0x0803c200, 0x00000000, 0x00000000, 0xfffffffd,
    };

    run(machine, want);
}

/* Without the register a read returns all ones and a write is dropped */
static void test_absent(const void *machine)
{
    static const uint32_t want[4] = {
        0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
    };

    run(machine, want);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_data_func("ingenic-cp0s4/t32", "ingenic-t32", test_btbctl);
    qtest_add_data_func("ingenic-cp0s4/t33", "ingenic-t33", test_btbctl);
    qtest_add_data_func("ingenic-cp0s4/t31", "ingenic-t31", test_absent);
    qtest_add_data_func("ingenic-cp0s4/t40", "ingenic-t40", test_absent);
    return g_test_run();
}
