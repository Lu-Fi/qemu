/*
 * QTest testcase for the Ingenic grades the model names
 *
 * Each grade as software identifies it: the soc_id at 0x1300002C and the
 * eFuse word libimp reads the grade from (SUBSOCTYPE1 at 0x238 up to the
 * T32, the byte at 0x21F on the T33, SUBSOCTYPE2 at 0x250 on the T40 and
 * T41), with the codes the vendor libimps decode; and the APLL and MPLL as
 * the vendor U-Boot leaves them for that grade, which a -kernel boot starts
 * from (a 4 KiB uImage here). The T32LQ's are as one reads them.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define SOC_ID          0x1300002c
#define EFUSE_WINDOW    0x13540200
#define CPAPCR          0x10000010
#define CPMPCR          0x10000014

typedef struct {
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    uint32_t soc_id;
    uint32_t grade_off;         /* the eFuse word, from the window */
} Family;

static const Family t21 = { "ingenic-t21", "ingenic-t31", 0x10021003, 0x38 };
static const Family t23 = { "ingenic-t23", "ingenic-t31", 0x10023003, 0x38 };
static const Family t32 = { "ingenic-t32", "ingenic-t31", 0x10032004, 0x38 };
/* the T33's type byte over 0xdd, as a T33L reads its word 7 (0x99dd0000) */
static const Family t33 = { "ingenic-t33", "ingenic-t31", 0x10033004, 0x1c };
static const Family t41 = { "ingenic-t41", "ingenic-t40", 0x10040003, 0x50 };

typedef struct {
    const char *part;
    const Family *f;
    uint32_t grade;
    uint32_t apll, mpll;
} GradeCase;

static const GradeCase cases[] = {
    { "t21x", &t21, 0x22220000, 0x0470484d, 0x04a0484d },
    { "t23dn", &t23, 0x55550000, 0x07d0590d, 0x07d0590d },
    { "t23zx", &t23, 0x66660000, 0x07d0590d, 0x07d0590d },
    { "t32lq", &t32, 0x99991111, 0x04b0510d, 0x0640510d },
    { "t32nq", &t32, 0xaaaa2222, 0x0640510d, 0x04b0510d },
    { "t32zl", &t32, 0x55550000, 0x04b0510d, 0x0640510d },
    { "t32zn", &t32, 0x77770000, 0x04b0510d, 0x0af0590d },
    { "t32zx", &t32, 0x66660000, 0x04b0510d, 0x0af0590d },
    { "t32vl", &t32, 0xcccc0000, 0x04b0510d, 0x0640510d },
    { "t32vn", &t32, 0xdddd0000, 0x04b0510d, 0x0af0590d },
    { "t32vnp", &t32, 0xbbbb0000, 0x04b0510d, 0x0af0590d },
    { "t32vx", &t32, 0xffff0000, 0x04b0510d, 0x0af0590d },
    { "t32a", &t32, 0x44440000, 0x0640510d, 0x04b0510d },
    { "t32dl", &t32, 0x11110000, 0x04b0510d, 0x0640510d },
    { "t32lqh", &t32, 0x88880000, 0x04b0510d, 0x0640510d },
    { "t33n", &t33, 0xaadd0000, 0x1db0e10d, 0x04b0510d },
    { "t33l", &t33, 0x99dd0000, 0x1db0e10d, 0x1450990d },
    { "t33dl", &t33, 0x11dd0000, 0x1db0e10d, 0x1450990d },
    { "t33zl", &t33, 0x55dd0000, 0x1290a10d, 0x1130990d },
    { "t33zn", &t33, 0x77dd0000, 0x1290a10d, 0x0af0590d },
    { "t33vl", &t33, 0xccdd0000, 0x1290a10d, 0x1130990d },
    { "t33vn", &t33, 0xdddd0000, 0x1290a10d, 0x0af0590d },
    { "t33a", &t33, 0x44dd0000, 0x1db0e10d, 0x04b0510d },
    { "t41n", &t41, 0x11110000, 0x05b008bd, 0x1760889d },
    { "t41zn", &t41, 0x77772222, 0x05b008bd, 0x1760889d },
    { "t41l", &t41, 0x33330000, 0x05b008bd, 0x063008bd },
    { "t41zl", &t41, 0x55550000, 0x05b008bd, 0x063008bd },
    { "t41zm", &t41, 0xbbbb0000, 0x05b008bd, 0x15d0889d },
    { "t41zg", &t41, 0xcccc0000, 0x05b008bd, 0x07c0093d },
    { "t41a", &t41, 0x44440000, 0x1440889d, 0x0c70881d },
};

static char *make_uimage(void)
{
    uint8_t img[64 + 4096] = { 0 };
    char *path;
    int fd = g_file_open_tmp("ingenic-grade-kernel-XXXXXX", &path, NULL);

    stl_be_p(img, 0x27051956);
    stl_be_p(img + 12, 4096);
    stl_be_p(img + 16, 0x80010000);
    stl_be_p(img + 20, 0x80010000);
    img[28] = 5;                /* Linux */
    img[29] = 5;                /* MIPS */
    img[30] = 2;                /* kernel */
    g_assert(write(fd, img, sizeof(img)) == sizeof(img));
    close(fd);
    return path;
}

static void test_grade(const void *data)
{
    const GradeCase *c = data;
    g_autofree char *kernel = make_uimage();
    QTestState *qts = qtest_initf("-machine %s -m 256 -kernel %s "
                                  "-global %s.soc-variant=%s",
                                  c->f->machine, kernel, c->f->soc, c->part);

    g_assert_cmphex(qtest_readl(qts, SOC_ID), ==, c->f->soc_id);
    g_assert_cmphex(qtest_readl(qts, EFUSE_WINDOW + c->f->grade_off), ==,
                    c->grade);
    g_assert_cmphex(qtest_readl(qts, CPAPCR), ==, c->apll);
    g_assert_cmphex(qtest_readl(qts, CPMPCR), ==, c->mpll);
    qtest_quit(qts);
    unlink(kernel);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-grade/%s",
                                                cases[i].part);

        qtest_add_data_func(path, &cases[i], test_grade);
    }
    return g_test_run();
}
