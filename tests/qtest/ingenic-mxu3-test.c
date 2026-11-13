/*
 * QTest testcase for the Ingenic MXU3 (XBurst2 512-bit SIMD) instructions
 *
 * A guest program runs a table of cases on a T40 (MXU3.0) and a T41
 * (MXU3.1): each case loads vr1 to vr3, vr31 and vs1, runs one MXU3 word
 * patched into an instruction slot, and stores vr3, vs1 and vr31 back. The
 * test compares them, the base register or the memory the case touched,
 * with the expected value in the table.
 *
 * The words and the C reference models are those of the harnesses that ran
 * on a real T40 and T41 with no mismatch, so a SRC_HWREF row's expected
 * value is what that model gives on those inputs. The floating point rows
 * are SRC_MODEL: the hardware dumps they were diffed against were not kept.
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

#define R_W             0
#define R_MCSR          4
#define R_G8            8
#define R_A             16
#define R_B             80
#define R_C             144
#define R_MEM           208     /* 128 bytes */
#define R_OD            336     /* vr3 out */
#define R_OS            400     /* vs1 out */
#define R_OW            464     /* vr31 out */
#define R_OMCSR         528
#define R_OG8           532
#define R_OT            536     /* 2 if a branch was taken, 3 if not */
#define R_CAUSE         540
#define R_SIZE          544

#define NONE            0xffff

enum { SRC_HWREF, SRC_MODEL };
enum { K3_VEC, K3_BR, K3_RI, K3_MEM, K3_FP };
#define M_D     1
#define M_S     2
#define M_W     4
#define M_MCSR  8

typedef struct {
    uint32_t word;
    uint8_t src;
    uint8_t kind;
    uint8_t ver;                /* 0 both, 30 the T40, 31 the T41 */
    uint8_t mask;
    uint16_t a, b, c, m1;       /* pool indices; m1 is the memory's 2nd half */
    uint8_t off;                /* the base register points at mem + 16 + off */
    int8_t delta;               /* its expected change, or the taken flag */
    uint32_t mcsr_in, mcsr_out;
    uint16_t d, s, w, mem;      /* pool indices of the expected values */
} Mxu3Case;

static const char *const pool[] = {
#include "ingenic-mxu3-pool.inc"
};

static const Mxu3Case cases[] = {
#include "ingenic-mxu3-data.inc"
};

enum { ZERO = 0, T1 = 9, T2, T3, T4, S0 = 16, S1, K0 = 26, K1 };

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

/* 256 bits of vr v at the base $8 + 32 * o, into or out of half n */
static uint32_t lao(int o, int n, int v)
{
    return 0x71001811 | (o & 31) << 16 | n << 14 | v << 6;
}

static uint32_t sao(int o, int v, int n)
{
    return 0x710000d5 | (o & 31) << 16 | v << 11 | n << 9;
}

static uint32_t mtsum(int s, int v)
{
    return 0x4a60001d | v << 16 | s << 6;
}

static uint32_t mfsum(int v, int s)
{
    return 0x4a60000f | s << 11 | v << 6;
}

#define CTCMXU2 (0x70000003 | 2 << 21 | 0x11 << 16 | 31 << 11)
#define CFCMXU2 (0x70100003 | 31 << 11 | 2 << 6)
#define EHB     0x000000c0
#define ERET    0x42000018

static char *image(void)
{
    size_t size = 1024;
    g_autofree uint8_t *img = g_malloc0(64 + size);
    uint8_t *code = img + 64;
    unsigned k = 0, i, loop, done, slot, patch;
    char *path;
    int fd = g_file_open_tmp("ingenic-mxu3-prog-XXXXXX", &path, NULL);
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
#define LOAD(off, v) do {                       \
        I(addiu(8, S0, off));                   \
        I(lao(0, 0, v));                        \
        I(lao(1, 1, v));                        \
    } while (0)
#define STORE(off, v) do {                      \
        I(addiu(8, S0, off));                   \
        I(sao(0, v, 0));                        \
        I(sao(1, v, 1));                        \
    } while (0)
    /* Status: CU0, CU1 and CU2, the exceptions vector to 0x80000180 */
    I(lui(T1, 0x7000));
    I(mtc0(T1, 12));
    I(EHB);
    LI(T2, HANDLER);
    for (i = 0; i < ARRAY_SIZE(handler); i++) {
        LI(T3, handler[i]);
        I(sw(T3, T2, 4 * i));
    }
    LI(S0, TABLE_VA);
    I(lw(T1, S0, HDR_GO));
    I(beq(T1, ZERO, -2));
    I(0);
    I(lw(S1, S0, HDR_COUNT));
    I(addiu(S0, S0, HDR_SIZE));

    loop = k;
    I(0);                               /* patched to beq S1, ZERO, done */
    I(0);
    I(sw(ZERO, S0, R_CAUSE));
    I(lw(T1, S0, R_W));
    patch = k;
    I(0);                               /* patched to LI T2, slot */
    I(0);
    I(sw(T1, T2, 0));
    /* the jump ends the translation block, so the patched slot is fresh */
    I(beq(ZERO, ZERO, 0));
    I(0);
    LOAD(R_A, 1);
    LOAD(R_B, 2);
    LOAD(R_C, 3);
    LOAD(R_C, 31);
    I(mtsum(1, 3));
    I(lw(2, S0, R_MCSR));
    I(CTCMXU2);
    I(lw(8, S0, R_G8));
    I(addiu(T4, ZERO, 0));
    slot = k;
    I(0);
    I(addiu(T4, T4, 2));
    I(addiu(T4, T4, 1));
    I(CFCMXU2);
    I(sw(2, S0, R_OMCSR));
    I(sw(8, S0, R_OG8));
    I(sw(T4, S0, R_OT));
    I(mfsum(5, 1));
    STORE(R_OD, 3);
    STORE(R_OS, 5);
    STORE(R_OW, 31);
    I(addiu(S0, S0, R_SIZE));
    I(addiu(S1, S1, -1));
    i = k;
    I(beq(ZERO, ZERO, loop - i - 1));
    I(0);
    done = k;
    LI(T1, DONE_MAGIC);
    LI(T2, TABLE_VA);
    I(sw(T1, T2, HDR_DONE));
    I(beq(ZERO, ZERO, -1));
    I(0);

    stl_le_p(&code[4 * loop], beq(S1, ZERO, done - loop - 1));
    i = PROG_LOAD + 4 * slot;
    stl_le_p(&code[4 * patch], lui(T2, i >> 16));
    stl_le_p(&code[4 * patch + 4], ori(T2, T2, i & 0xffff));
#undef STORE
#undef LOAD
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

static void put_hex(uint8_t *dst, const char *hex)
{
    size_t i, n = strlen(hex) / 2;

    for (i = 0; i < n; i++) {
        unsigned v;

        g_assert(sscanf(hex + 2 * i, "%2x", &v) == 1);
        dst[i] = v;
    }
}

static const char *src_name(int src)
{
    return src == SRC_HWREF ? "model checked on T40/T41" : "model";
}

static void fill_record(uint8_t *r, const Mxu3Case *c, unsigned idx)
{
    uint32_t g8 = 0;

    stl_le_p(r + R_W, c->word);
    stl_le_p(r + R_MCSR, c->mcsr_in);
    put_hex(r + R_A, pool[c->a]);
    put_hex(r + R_B, pool[c->b]);
    put_hex(r + R_C, pool[c->c]);
    if (c->kind == K3_MEM) {
        put_hex(r + R_MEM, pool[c->a]);
        put_hex(r + R_MEM + 64, pool[c->m1]);
        g8 = TABLE_VA + HDR_SIZE + idx * R_SIZE + R_MEM + 16 + c->off;
        /* vr1 is not an input of the memory cases */
        memset(r + R_A, 0, 64);
    }
    stl_le_p(r + R_G8, g8);
}

static void hexdump(GString *s, const uint8_t *v)
{
    int i;

    for (i = 0; i < 64; i++) {
        g_string_append_printf(s, "%02x", v[i]);
    }
}

static bool cmp_hex(GString *err, const char *what, const uint8_t *got,
                    const char *want)
{
    uint8_t w[128];

    put_hex(w, want);
    if (memcmp(got, w, strlen(want) / 2)) {
        g_string_append_printf(err, " %s got ", what);
        hexdump(err, got);
        g_string_append_printf(err, " want %s", want);
        return false;
    }
    return true;
}

/* returns an error string, or NULL if the case passed */
static char *check_record(const uint8_t *r, const Mxu3Case *c, unsigned idx)
{
    GString *err = g_string_new(NULL);
    uint32_t cause = ldl_le_p(r + R_CAUSE);
    uint8_t w[128];
    bool ok = true;

    if (c->kind == K3_RI) {
        if (((cause >> 2) & 0x1f) != 10) {
            g_string_append_printf(err, " Cause 0x%08x, not Reserved "
                                   "Instruction", cause);
            ok = false;
        }
    } else if (cause) {
        g_string_append_printf(err, " exception, Cause 0x%08x", cause);
        ok = false;
    } else if (c->kind == K3_BR) {
        if ((ldl_le_p(r + R_OT) == 2) != c->delta) {
            g_string_append_printf(err, " branch %s, want %s",
                                   c->delta ? "not taken" : "taken",
                                   c->delta ? "taken" : "not taken");
            ok = false;
        }
    } else {
        if (c->mask & M_D) {
            ok &= cmp_hex(err, "vd", r + R_OD, pool[c->d]);
        }
        if (c->mask & M_S) {
            ok &= cmp_hex(err, "vs1", r + R_OS,
                          pool[c->s == NONE ? c->c : c->s]);
        }
        if (c->mask & M_W) {
            ok &= cmp_hex(err, "vr31", r + R_OW,
                          pool[c->w == NONE ? c->c : c->w]);
        }
        if ((c->mask & M_MCSR) && ldl_le_p(r + R_OMCSR) != c->mcsr_out) {
            g_string_append_printf(err, " MCSR 0x%08x, want 0x%08x",
                                   ldl_le_p(r + R_OMCSR), c->mcsr_out);
            ok = false;
        }
        if (c->kind == K3_MEM) {
            uint32_t base = TABLE_VA + HDR_SIZE + idx * R_SIZE + R_MEM + 16 +
                            c->off;

            if (ldl_le_p(r + R_OG8) != base + c->delta) {
                g_string_append_printf(err, " base 0x%08x, want 0x%08x",
                                       ldl_le_p(r + R_OG8), base + c->delta);
                ok = false;
            }
            put_hex(w, pool[c->a]);
            put_hex(w + 64, pool[c->m1]);
            put_hex(w + 16 + c->off, pool[c->mem]);
            if (memcmp(r + R_MEM, w, 128)) {
                g_string_append(err, " memory differs");
                ok = false;
            }
        }
    }
    if (ok) {
        g_string_free(err, true);
        return NULL;
    }
    g_string_prepend(err, g_strdup_printf("0x%08x [%s]:", c->word,
                                          src_name(c->src)));
    return g_string_free(err, false);
}

static void test_mxu3(const void *data)
{
    const char *machine = data;
    int ver = strcmp(machine, "ingenic-t40") ? 31 : 30;
    g_autofree char *img = image();
    g_autofree uint8_t *tab = g_malloc0(HDR_SIZE + ARRAY_SIZE(cases) * R_SIZE);
    g_autofree unsigned *which = g_new(unsigned, ARRAY_SIZE(cases));
    QTestState *qts = qtest_initf("-machine %s -m 128 -kernel %s -accel tcg",
                                  machine, img);
    unsigned i, n = 0, bad = 0;
    int t;

    for (i = 0; i < ARRAY_SIZE(cases); i++) {
        if (!cases[i].ver || cases[i].ver == ver) {
            fill_record(tab + HDR_SIZE + n * R_SIZE, &cases[i], n);
            which[n++] = i;
        }
    }
    stl_le_p(tab + HDR_COUNT, n);
    qtest_memwrite(qts, TABLE_PA, tab, HDR_SIZE + n * R_SIZE);
    qtest_writel(qts, TABLE_PA + HDR_GO, 1);
    for (t = 0; t < 6000 &&
         qtest_readl(qts, TABLE_PA + HDR_DONE) != DONE_MAGIC; t++) {
        g_usleep(10 * 1000);
    }
    g_assert_cmphex(qtest_readl(qts, TABLE_PA + HDR_DONE), ==, DONE_MAGIC);
    qtest_memread(qts, TABLE_PA, tab, HDR_SIZE + n * R_SIZE);
    for (i = 0; i < n; i++) {
        g_autofree char *err = check_record(tab + HDR_SIZE + i * R_SIZE,
                                            &cases[which[i]], i);

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
    qtest_add_data_func("ingenic-mxu3/t40", "ingenic-t40", test_mxu3);
    qtest_add_data_func("ingenic-mxu3/t41", "ingenic-t41", test_mxu3);
    return g_test_run();
}
