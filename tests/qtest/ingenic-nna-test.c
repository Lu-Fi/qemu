/*
 * QTest testcase for the Ingenic NNA's DMA (T41)
 *
 * Each case lays a chain of descriptors in the NNDMA's descriptor RAM,
 * starts it on one channel and checks where the bytes went and what the
 * channel reads back after, against a T41NQ running the same chains: the
 * three channels to the ORAM and the two to DDR, chains of one and four
 * descriptors, the index wrapping after 2047, a length with bits above
 * the low 12, and an index written without the start bit.
 *
 * The engine cases boot a program (-kernel, under TCG) that runs a stream
 * of NN instructions over generated weights, BT and features, keeping what
 * NNDRD reads: a 1x1 convolution of 8-bit data, and a 3x3 one with
 * padding at the top and left. The CRC32 of what it read is what a T41NQ
 * read running the same instructions.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/units.h"
#include "libqtest.h"

#define DESRAM          0x12500000
#define NNDMA           0x12508000
#define SRAM            0x12600000
#define ORAM            0x12640000  /* with the L2 at its reset 256 KiB */
#define DDR             0x00800000
#define START           (1u << 31)

typedef struct {
    const char *name;
    unsigned reg;               /* channel, from NNDMA */
    unsigned idx;               /* first descriptor */
    unsigned ndesc;
    uint32_t bytes;             /* in all */
    uint32_t status;            /* the T41NQ's, after */
} Case;

static const Case cases[] = {
    { "rd0-1", 0x00, 1000, 1, 4096, 0x01fe03e9 },
    { "rd0-4", 0x00, 1000, 4, 131072, 0x01c003ec },
    { "rd0-wrap", 0x00, 2047, 1, 4096, 0x01fe0000 },
    { "rd1", 0x04, 7, 1, 65536, 0x01e00008 },
    { "rd2", 0x08, 9, 1, 4096, 0x01fe000a },
    { "wr0", 0x10, 100, 1, 131072, 0x01c00065 },
    { "wr1", 0x14, 11, 1, 4096, 0x01fe000c },
};

static void put_desc(QTestState *qts, unsigned idx, uint32_t ddr,
                     uint32_t oram, uint32_t bytes, bool link)
{
    uint32_t n = bytes / 64 - 1;

    qtest_writel(qts, DESRAM + idx * 8, ddr | link | (n >> 12 & 0x1f) << 1);
    qtest_writel(qts, DESRAM + idx * 8 + 4,
                 (n & 0xfff) << 20 | (oram - SRAM) >> 6);
}

static void fill(QTestState *qts, uint64_t addr, uint32_t bytes, uint32_t tag)
{
    g_autofree uint32_t *buf = g_new(uint32_t, bytes / 4);

    for (uint32_t i = 0; i < bytes / 4; i++) {
        buf[i] = cpu_to_le32(tag | i);
    }
    qtest_memwrite(qts, addr, buf, bytes);
}

static void check(QTestState *qts, uint64_t addr, uint32_t bytes, uint32_t tag)
{
    g_autofree uint32_t *buf = g_new(uint32_t, bytes / 4);

    qtest_memread(qts, addr, buf, bytes);
    for (uint32_t i = 0; i < bytes / 4; i++) {
        g_assert_cmphex(le32_to_cpu(buf[i]), ==, tag | i);
    }
}

static void test_case(const void *data)
{
    const Case *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t41");
    bool to_oram = c->reg < 0x10;
    uint32_t per = c->bytes / c->ndesc;

    fill(qts, DDR, c->bytes + 64, 0xd0000000);
    fill(qts, ORAM, c->bytes + 64, 0x0a000000);
    for (unsigned k = 0; k < c->ndesc; k++) {
        put_desc(qts, (c->idx + k) & 2047, DDR + k * per, ORAM + k * per, per,
                 k + 1 < c->ndesc);
    }
    qtest_writel(qts, NNDMA + c->reg, c->idx | START);
    g_assert_cmphex(qtest_readl(qts, NNDMA + c->reg), ==, c->status);
    if (to_oram) {
        check(qts, ORAM, c->bytes, 0xd0000000);
        g_assert_cmphex(qtest_readl(qts, ORAM + c->bytes), ==,
                        0x0a000000 | c->bytes / 4);
    } else {
        check(qts, DDR, c->bytes, 0x0a000000);
        g_assert_cmphex(qtest_readl(qts, DDR + c->bytes), ==,
                        0xd0000000 | c->bytes / 4);
    }
    qtest_quit(qts);
}

/* 4097 units: the length's bits above the low 12 sit in bits 5:1 */
static void test_long(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    uint32_t bytes = 4097 * 64;

    fill(qts, ORAM, 256 * KiB, 0x0a000000);
    put_desc(qts, 300, DDR, ORAM, bytes, false);
    qtest_writel(qts, NNDMA + 0x10, 300 | START);
    g_assert_cmphex(qtest_readl(qts, NNDMA + 0x10), ==, 0x017ff92d);
    check(qts, DDR, 256 * KiB, 0x0a000000);
    qtest_quit(qts);
}

/* without the start bit the index is set and nothing moves */
static void test_index_only(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    fill(qts, DDR, 4096, 0xd0000000);
    fill(qts, ORAM, 4096, 0x0a000000);
    put_desc(qts, 1000, DDR, ORAM, 4096, false);
    qtest_writel(qts, NNDMA, 1000 << 1 | 1);
    g_assert_cmphex(qtest_readl(qts, NNDMA), ==, 0x7d1);
    check(qts, ORAM, 4096, 0x0a000000);
    qtest_quit(qts);
}

/* --- the engine, driven by a program of NN instructions */

#define PROG_LOAD       0x80010000
#define PROG_DATA       0x80020000  /* one 64-byte slot per instruction */
#define PROG_OUT        0x00200000  /* what NNDRD read */
#define PROG_DONE       0x00201000

enum { NN_RWR, NN_RRD, NN_DWR, NN_DRD, NN_MAC, NN_CMD };

typedef struct {
    uint8_t op;
    uint16_t imm;
    uint8_t data[64];
} NnRec;

typedef struct {
    NnRec rec[512];
    unsigned n;
    uint32_t rng;
} NnProg;

static uint32_t rng_next(NnProg *g)
{
    uint32_t x = g->rng;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return g->rng = x;
}

static NnRec *add(NnProg *g, unsigned op, unsigned imm)
{
    NnRec *r = &g->rec[g->n++];

    g_assert(g->n <= ARRAY_SIZE(g->rec));
    memset(r, 0, sizeof(*r));
    r->op = op;
    r->imm = imm;
    return r;
}

static void add_word(NnProg *g, unsigned op, unsigned imm, uint32_t v)
{
    stl_le_p(add(g, op, imm)->data, v);
}

static void add_random(NnProg *g, unsigned op, unsigned imm)
{
    NnRec *r = add(g, op, imm);

    for (int i = 0; i < 64; i++) {
        r->data[i] = rng_next(g);
    }
}

/* registers, W entries, one BT block, F records: as made for the T41NQ */
static void nn_layer(NnProg *g, const uint16_t (*regs)[2], unsigned nregs,
                     unsigned wentries, unsigned nf, uint32_t tlo)
{
    static const uint8_t quarter[4] = { 0x00, 0x02, 0x04, 0x07 };
    uint8_t bt[256];

    add(g, NN_CMD, 0);
    for (unsigned i = 0; i < nregs; i++) {
        add_word(g, NN_RWR, regs[i][0], regs[i][1]);
    }
    add_word(g, NN_RWR, 0x58, 0);
    add_word(g, NN_RWR, 0x59, 0x0801);
    add_word(g, NN_RWR, 0x5a, 0x01);
    add(g, NN_CMD, 0x60);
    add_word(g, NN_RWR, 0x5c, 0);
    for (unsigned e = 0; e < wentries; e++) {
        for (int q = 0; q < 4; q++) {
            add_random(g, NN_DWR, quarter[q]);
        }
    }
    add_word(g, NN_RWR, 0x5e, 0);
    for (int i = 0; i < 32; i++) {
        stl_le_p(&bt[4 * i], (rng_next(g) & 0x7ff) - 0x400);
    }
    for (int i = 0; i < 32; i++) {
        stl_le_p(&bt[128 + 4 * i], tlo + (rng_next(g) & (tlo - 1)));
    }
    for (int i = 0; i < 4; i++) {
        memcpy(add(g, NN_DWR, 0x40)->data, &bt[64 * i], 64);
    }
    add_word(g, NN_RWR, 0x50, 0);
    for (unsigned i = 0; i < nf; i++) {
        add_random(g, NN_DWR, 0x20);
    }
    add_word(g, NN_RWR, 0x30, 0);
}

/* the tail: BT, QSUM, the quantized output, then the sums CPOC copied */
static void nn_out(NnProg *g, unsigned qreads)
{
    add_word(g, NN_RWR, 0x42, 0);
    add(g, NN_CMD, 0x84);
    add(g, NN_CMD, 0x8b);
    for (unsigned i = 0; i < qreads; i++) {
        add(g, NN_DRD, 0x20);
    }
    for (unsigned i = 0; i < 16; i++) {
        add(g, NN_DRD, i);
    }
}

#define T0 8
#define T1 9

static uint32_t nn_insn(unsigned op, unsigned imm)
{
    switch (op) {
    case NN_RRD:
    case NN_DRD:
        return 0x7000003a | op << 21 | (imm & 0x3ff) << 11;
    case NN_CMD:
        return 0x7000003a | op << 21 | (imm & 0x7fff) << 6;
    default:
        return 0x7000003a | op << 21 | (imm >> 5 & 31) << 16 | (imm & 31) << 6;
    }
}

static uint32_t lao(int base, int off, int n, int vr)
{
    return 0x70000011 | base << 21 | (off & 31) << 16 | n << 14 | 3 << 11 |
           vr << 6;
}

static uint32_t sao(int base, int off, int n, int vr)
{
    return 0x70000015 | base << 21 | (off & 31) << 16 | vr << 11 | n << 9 |
           3 << 6;
}

static uint32_t addiu(int rt, int rs, int imm)
{
    return 0x24000000 | rs << 21 | rt << 16 | (imm & 0xffff);
}

/*
 * A uImage of the program: Status.CU2 on, then each instruction with its
 * operand loaded into vr31 word 0 (NNRWR, NNMAC) or vr0 (NNDWR), each read
 * stored to PROG_OUT, and a done mark
 */
static char *nn_image(const NnProg *g)
{
    size_t size = 0x10000 + g->n * 64;
    g_autofree uint8_t *img = g_malloc0(64 + size);
    uint8_t *code = img + 64;
    unsigned k = 0;
    char *path;
    int fd = g_file_open_tmp("ingenic-nna-prog-XXXXXX", &path, NULL);
    static const uint32_t prologue[] = {
        0x400a6000,                     /* mfc0 t2, Status */
        0x3c0b4000,                     /* lui t3, 0x4000 */
        0x014b5025,                     /* or t2, t2, t3 */
        0x408a6000,                     /* mtc0 t2, Status */
        0x000000c0,                     /* ehb */
        0x3c088002,                     /* lui t0, 0x8002: the operands */
        0x3c098020,                     /* lui t1, 0x8020: the reads */
    };
    static const uint32_t epilogue[] = {
        0x3c0a8020,                     /* lui t2, 0x8020 */
        0x240b600d,                     /* li t3, 0x600d */
        0xad4b1000,                     /* sw t3, 0x1000(t2) */
        0x1000ffff,                     /* b . */
        0x00000000,
    };

    for (unsigned i = 0; i < ARRAY_SIZE(prologue); i++) {
        stl_le_p(&code[4 * k++], prologue[i]);
    }
    for (unsigned i = 0; i < g->n; i++) {
        const NnRec *r = &g->rec[i];

        if (r->op == NN_RWR || r->op == NN_MAC) {
            stl_le_p(&code[4 * k++], 0x70000010 | T0 << 21 | 31 << 6);
        } else if (r->op == NN_DWR) {
            stl_le_p(&code[4 * k++], lao(T0, 0, 0, 0));
            stl_le_p(&code[4 * k++], lao(T0, 1, 1, 0));
        }
        stl_le_p(&code[4 * k++], nn_insn(r->op, r->imm));
        if (r->op == NN_RRD || r->op == NN_DRD) {
            stl_le_p(&code[4 * k++], sao(T1, 0, 0, 0));
            stl_le_p(&code[4 * k++], sao(T1, 1, 1, 0));
            stl_le_p(&code[4 * k++], addiu(T1, T1, 64));
        }
        stl_le_p(&code[4 * k++], addiu(T0, T0, 64));
        memcpy(&code[0x10000 + 64 * i], r->data, 64);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(epilogue); i++) {
        stl_le_p(&code[4 * k++], epilogue[i]);
    }
    g_assert(4 * k <= 0x10000);

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

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static void nn_run(const NnProg *g, size_t len, uint32_t crc)
{
    g_autofree char *image = nn_image(g);
    QTestState *qts = qtest_initf("-machine ingenic-t41 -m 128 -kernel %s "
                                  "-accel tcg", image);
    g_autofree uint8_t *out = g_malloc(len);
    int i;

    for (i = 0; i < 1000 && qtest_readl(qts, PROG_DONE) != 0x600d; i++) {
        g_usleep(10 * 1000);
    }
    g_assert_cmphex(qtest_readl(qts, PROG_DONE), ==, 0x600d);
    qtest_memread(qts, PROG_OUT, out, len);
    g_assert_cmphex(crc32_le(out, len), ==, crc);
    qtest_quit(qts);
    unlink(image);
}

/* 8-bit, 1 tap, 8 pixels as 2 rows of 4 */
static void test_engine_1x1(void)
{
    static const uint16_t regs[][2] = {
        { 0x28, 3 }, { 0x46, 3 }, { 0x29, 0 }, { 0x2b, 0 }, { 0x3b, 0x33 },
        { 0x55, 2 }, { 0x44, 3 }, { 0x45, 2 }, { 0x43, 0 }, { 0x3e, 1 },
        { 0x38, 0x12 }, { 0x3c, 0 }, { 0x37, 8 }, { 0x39, 4 },
    };
    g_autofree NnProg *g = g_new0(NnProg, 1);

    g->rng = 0x1234567;
    nn_layer(g, regs, ARRAY_SIZE(regs), 4, 4, 16384);
    add_word(g, NN_MAC, 1, 0);
    nn_out(g, 4);
    nn_run(g, 1280, 0x1d1e99e2);
}

/* 8-bit, 9 taps, rows 12 pixels apart; MACTL_FP_X/Y -1 pad the top-left */
static void test_engine_3x3(void)
{
    static const uint16_t regs[][2] = {
        { 0x28, 3 }, { 0x46, 3 }, { 0x3b, 0x33 }, { 0x55, 2 }, { 0x44, 3 },
        { 0x45, 2 }, { 0x43, 0 }, { 0x3e, 0x1ff }, { 0x38, 0x12 },
        { 0x3c, 0 }, { 0x3d, 0 }, { 0x37, 0x30 }, { 0x39, 0xc }, { 0x2c, 15 },
        { 0x2e, 15 }, { 0x2f, 0 },
    };
    g_autofree NnProg *g = g_new0(NnProg, 1);

    g->rng = 0x89abcdef;
    nn_layer(g, regs, ARRAY_SIZE(regs), 36, 24, 65536);
    add_word(g, NN_MAC, 1, 1);
    nn_out(g, 4);
    nn_run(g, 1280, 0x90f5b996);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/nna/nndma/%s",
                                                cases[i].name);

        qtest_add_data_func(path, &cases[i], test_case);
    }
    qtest_add_func("ingenic/nna/nndma/long", test_long);
    qtest_add_func("ingenic/nna/nndma/index-only", test_index_only);
    qtest_add_func("ingenic/nna/engine/1x1", test_engine_1x1);
    qtest_add_func("ingenic/nna/engine/3x3", test_engine_3x3);
    return g_test_run();
}
