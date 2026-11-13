/*
 * QTest testcase for the Ingenic MXU2 (XBurst1 128-bit SIMD) instructions
 *
 * A guest program runs a table of cases on a T31, each one MXU2 word (or a
 * short sequence of them) with its register inputs, and the test compares
 * the result registers or memory with the expected value. The program
 * patches each case's words into three instruction slots, so the table is
 * data only and the instruction words are pre-encoded.
 *
 * The expected values come from the T20 builtin oracle, a program that ran
 * on silicon and printed every MXU2 builtin's result; the words are the
 * ones the compiler emitted for it. The few ops the compiler folded away
 * there, and the quirks measured on a T20 afterwards, carry another source
 * tag in the table, and a failure names it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define PROG_LOAD       0x80010000
#define HANDLER         0x80000180
#define TABLE_VA        0x80400000
#define TABLE_PA        0x00400000
#define HDR_COUNT       0
#define HDR_GO          4
#define HDR_DONE        8
#define HDR_SIZE        16
#define DONE_MAGIC      0x600d

/* one case in the guest's table */
#define R_W             0       /* three words patched into the slots */
#define R_G             16      /* $2 to $7 in */
#define R_F             40      /* $f0 in */
#define R_V             48      /* vr0 to vr3 in */
#define R_MEM           112     /* 64 bytes the base register may point to */
#define R_OV            176     /* vr0 to vr3 out */
#define R_OG            240     /* $2 to $7 out */
#define R_OF            264     /* $f0 out */
#define R_OT            272     /* 1 if a branch was taken, 3 if not */
#define R_CAUSE         276     /* Cause of the last exception, 0 if none */
#define R_SIZE          288

enum { SRC_SILICON, SRC_FINDING, SRC_DEF, SRC_MODEL };
#define FL_MEM16       1       /* K_MEM compares 16 bytes into the memory */

enum { K_VEC, K_GPR, K_FPR32, K_FPR64, K_MEM, K_BR };

typedef struct {
    const char *name;
    uint8_t src;
    uint8_t kind;
    uint8_t res;        /* vector or GPR number the result is in */
    uint8_t nvin;
    uint8_t flags;
    uint32_t w[3];
    uint32_t vin[4][4];         /* vr0 to vr3 */
    uint32_t gpr[6];            /* $2 to $7 */
    uint8_t memreg;             /* GPR that points at the case's memory */
    uint8_t memoff;
    uint64_t fpr;
    uint32_t exp[4];
} Mxu2Case;

static const Mxu2Case cases[] = {
#include "ingenic-mxu2-data.inc"
};

/* the pattern the oracle's g_buf held */
static const uint8_t gbuf[64] = {
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b,
    0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33,
    0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b,
    0x4c, 0x4d, 0x4e, 0x4f,
};

enum { ZERO = 0, T0 = 8, T1, T2, T3, T4, S0 = 16, S1, K0 = 26, K1 };

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

static uint32_t ldc1(int ft, int base, uint16_t off)
{
    return 0xd4000000 | base << 21 | ft << 16 | off;
}

static uint32_t sdc1(int ft, int base, uint16_t off)
{
    return 0xf4000000 | base << 21 | ft << 16 | off;
}

static uint32_t beq(int rs, int rt, int16_t off)
{
    return 0x10000000 | rs << 21 | rt << 16 | (uint16_t)off;
}

static uint32_t mfc0(int rt, int rd)
{
    return 0x40000000 | rt << 16 | rd << 11;
}

static uint32_t mtc0(int rt, int rd)
{
    return 0x40800000 | rt << 16 | rd << 11;
}

static uint32_t lu1q(int base, int vd)
{
    return 0x70000014 | base << 21 | vd << 6;
}

static uint32_t su1q(int base, int vd)
{
    return 0x7000001c | base << 21 | vd << 6;
}

#define EHB     0x000000c0
#define ERET    0x42000018

static char *image(void)
{
    size_t size = 1024;
    g_autofree uint8_t *img = g_malloc0(64 + size);
    uint8_t *code = img + 64;
    unsigned k = 0, i, loop, done, slot[3];
    char *path;
    int fd = g_file_open_tmp("ingenic-mxu2-prog-XXXXXX", &path, NULL);
    const uint32_t handler[] = {
        mfc0(K0, 13),
        sw(K0, S0, R_CAUSE),
        mfc0(K0, 14),
        addiu(K0, K0, 4),
        mtc0(K0, 14),
        EHB,
        ERET,
    };

#define I(x) stl_le_p(&code[4 * k++], (x))
#define LI(r, v) do {                           \
        I(lui(r, (v) >> 16));                   \
        I(ori(r, r, (v) & 0xffff));             \
    } while (0)
    /* Status: CU0, CU1 and CU2, the exceptions vector to 0x80000180 */
    I(lui(T0, 0x7000));
    I(mtc0(T0, 12));
    I(EHB);
    LI(T2, HANDLER);
    for (i = 0; i < ARRAY_SIZE(handler); i++) {
        LI(T3, handler[i]);
        I(sw(T3, T2, 4 * i));
    }
    LI(S0, TABLE_VA);
    /* wait for the host to fill the table */
    I(lw(T0, S0, HDR_GO));
    I(beq(T0, ZERO, -2));
    I(0);
    I(lw(S1, S0, HDR_COUNT));
    I(addiu(S0, S0, HDR_SIZE));

    loop = k;
    I(0);                               /* patched to beq S1, ZERO, done */
    I(0);
    I(sw(ZERO, S0, R_CAUSE));
    for (i = 0; i < 3; i++) {
        I(lw(T0, S0, R_W + 4 * i));
        I(0);                           /* patched to LI, below */
        I(0);
        I(sw(T0, T1, 0));
    }
    /* the jump ends the translation block, so the patched slots are fresh */
    I(beq(ZERO, ZERO, 0));
    I(0);
    I(addiu(T1, S0, R_V));
    for (i = 0; i < 4; i++) {
        I(lu1q(T1, i));
        I(addiu(T1, T1, 16));
    }
    I(ldc1(0, S0, R_F));
    for (i = 0; i < 6; i++) {
        I(lw(2 + i, S0, R_G + 4 * i));
    }
    I(addiu(T4, ZERO, 0));
    for (i = 0; i < 3; i++) {
        slot[i] = k;
        I(0);
    }
    I(addiu(T4, T4, 1));
    I(addiu(T4, T4, 2));
    I(0);
    for (i = 0; i < 6; i++) {
        I(sw(2 + i, S0, R_OG + 4 * i));
    }
    I(sdc1(0, S0, R_OF));
    I(sw(T4, S0, R_OT));
    I(addiu(T1, S0, R_OV));
    for (i = 0; i < 4; i++) {
        I(su1q(T1, i));
        I(addiu(T1, T1, 16));
    }
    I(addiu(S0, S0, R_SIZE));
    I(addiu(S1, S1, -1));
    i = k;
    I(beq(ZERO, ZERO, loop - i - 1));
    I(0);
    done = k;
    LI(T0, DONE_MAGIC);
    LI(T1, TABLE_VA);
    I(sw(T0, T1, HDR_DONE));
    I(beq(ZERO, ZERO, -1));
    I(0);

    stl_le_p(&code[4 * loop], beq(S1, ZERO, done - loop - 1));
    /* the slot patching needs the slots' addresses in T1 */
    for (i = 0; i < 3; i++) {
        unsigned at = loop + 4 + 4 * i, va = PROG_LOAD + 4 * slot[i];

        stl_le_p(&code[4 * at], lui(T1, va >> 16));
        stl_le_p(&code[4 * at + 4], ori(T1, T1, va & 0xffff));
    }
#undef LI
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

static void fill_record(uint8_t *r, const Mxu2Case *c, unsigned idx)
{
    uint32_t g[6];
    unsigned i;

    memcpy(g, c->gpr, sizeof(g));
    if (c->memreg) {
        g[c->memreg - 2] = TABLE_VA + HDR_SIZE + idx * R_SIZE + R_MEM +
                           c->memoff;
    }
    for (i = 0; i < 3; i++) {
        stl_le_p(r + R_W + 4 * i, c->w[i]);
    }
    for (i = 0; i < 6; i++) {
        stl_le_p(r + R_G + 4 * i, g[i]);
    }
    stq_le_p(r + R_F, c->fpr);
    for (i = 0; i < 4 * c->nvin; i++) {
        stl_le_p(r + R_V + 4 * i, c->vin[i / 4][i % 4]);
    }
    memcpy(r + R_MEM, gbuf, sizeof(gbuf));
}

static const char *src_name(int src)
{
    static const char *names[] = { "silicon oracle", "silicon finding",
                                   "definition", "model" };

    return names[src];
}

/* returns an error string, or NULL if the case passed */
static char *check_record(const uint8_t *r, const Mxu2Case *c)
{
    uint32_t got[4] = { 0 }, want[4];
    unsigned i, n = 4;
    uint64_t f;

    memcpy(want, c->exp, sizeof(want));
    switch (c->kind) {
    case K_VEC:
        for (i = 0; i < 4; i++) {
            got[i] = ldl_le_p(r + R_OV + 16 * c->res + 4 * i);
        }
        break;
    case K_GPR:
        got[0] = ldl_le_p(r + R_OG + 4 * (c->res - 2));
        n = 1;
        break;
    case K_FPR32:
        got[0] = ldl_le_p(r + R_OF);
        n = 1;
        break;
    case K_FPR64:
        f = ldq_le_p(r + R_OF);
        got[0] = f;
        got[1] = f >> 32;
        n = 2;
        break;
    case K_MEM:
        for (i = 0; i < 4; i++) {
            got[i] = ldl_le_p(r + R_MEM + 16 * (c->flags & FL_MEM16) + 4 * i);
        }
        break;
    case K_BR:
        got[0] = ldl_le_p(r + R_OT) == 1;
        n = 1;
        break;
    }
    if (ldl_le_p(r + R_CAUSE)) {
        return g_strdup_printf("%s [%s]: exception, Cause 0x%08x", c->name,
                               src_name(c->src), ldl_le_p(r + R_CAUSE));
    }
    for (i = 0; i < n; i++) {
        if (got[i] != want[i]) {
            return g_strdup_printf("%s [%s]: word %u got 0x%08x, want 0x%08x"
                                   " (all: %08x %08x %08x %08x)", c->name,
                                   src_name(c->src), i, got[i], want[i],
                                   got[0], got[1], got[2], got[3]);
        }
    }
    return NULL;
}

static void test_mxu2(const void *data)
{
    g_autofree char *img = image();
    g_autofree uint8_t *tab = g_malloc0(HDR_SIZE + ARRAY_SIZE(cases) * R_SIZE);
    QTestState *qts = qtest_initf("-machine %s -m 128 -kernel %s -accel tcg",
                                  (const char *)data, img);
    unsigned i, bad = 0;
    int t;

    stl_le_p(tab + HDR_COUNT, ARRAY_SIZE(cases));
    for (i = 0; i < ARRAY_SIZE(cases); i++) {
        fill_record(tab + HDR_SIZE + i * R_SIZE, &cases[i], i);
    }
    qtest_memwrite(qts, TABLE_PA, tab, HDR_SIZE + ARRAY_SIZE(cases) * R_SIZE);
    qtest_writel(qts, TABLE_PA + HDR_GO, 1);
    for (t = 0; t < 3000 &&
         qtest_readl(qts, TABLE_PA + HDR_DONE) != DONE_MAGIC; t++) {
        g_usleep(10 * 1000);
    }
    g_assert_cmphex(qtest_readl(qts, TABLE_PA + HDR_DONE), ==, DONE_MAGIC);
    qtest_memread(qts, TABLE_PA, tab, HDR_SIZE + ARRAY_SIZE(cases) * R_SIZE);
    for (i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *err = check_record(tab + HDR_SIZE + i * R_SIZE,
                                            &cases[i]);

        if (err) {
            g_test_message("%s", err);
            bad++;
        }
    }
    g_assert_cmpuint(bad, ==, 0);
    qtest_quit(qts);
    unlink(img);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("ingenic-mxu2/t31", "ingenic-t31", test_mxu2);
    return g_test_run();
}
