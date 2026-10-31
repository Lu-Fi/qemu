/*
 * QTest testcase for the Ingenic eFuse controller
 *
 * One case per part, run on the machine that models it: the window of fuse
 * words at 0x200, the read protocol (a word index, or a byte address and
 * length, in the control register; the result in the data register), the
 * done bit, and the control bits that read back. Expected values are as the
 * part named reads them, except the chip IDs and the T40's word 19, which
 * are the model's own.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "libqtest.h"

#define EFUSE_BASE      0x13540000
#define EFUSE_CTRL      0x000
#define EFUSE_STATE     0x008
#define EFUSE_DATA      0x00c
#define EFUSE_WINDOW    0x200

#define CTRL_START      BIT(0)
#define CTRL_ADDR_SHIFT 21
#define CTRL_BIT31      BIT(31)     /* kept set by the vendor driver; unknown */
#define STATE_DONE      BIT(0)

/* The chip ID words of every part but the T40 and T41 */
#define ID0             0x725c2516
#define ID1             0x94fe0281
#define ID2             0x12100080
#define T40_ID0         0x40401234
#define T40_ID1         0x56780000
#define T40_ID2         0x12100080

/* Word 15 on a T20X, T30X and T31X; a T21N reads it with bit 24 flipped */
#define MXU_LICENCE     0x5f8040b7

typedef enum {
    BY_WORD,    /* the control register holds a word index */
    BY_BYTE,    /* it holds a byte address and a length */
} Addressing;

typedef struct {
    uint32_t ctrl;
    uint32_t data;
} EfuseRead;

typedef struct {
    const char *part;
    const char *machine;
    const char *soc;            /* the device with the soc-variant property */
    const char *variant;
    Addressing mode;
    unsigned nwords;            /* words in the window, from 0x200 */
    uint32_t window[64];
    uint32_t array[16];         /* BY_BYTE: the 64 bytes the protocol reads */
    uint32_t ctrl_mask;         /* the control bits that read back */
    unsigned addr_bits;         /* BY_WORD: the width of the word index */
    uint32_t state_hi;          /* state bits 15:8, from the security word */
    EfuseRead reads[4];         /* BY_BYTE: longer reads, zero ends the list */
} EfuseCase;

static const EfuseCase cases[] = {
    {
        /* A blank array */
        .part = "t10l", .machine = "ingenic-t10", .soc = "ingenic-t31",
        .variant = "t10l", .mode = BY_BYTE, .nwords = 24,
        .ctrl_mask = 0x8fe30000,
    }, {
        /* The window mirrors only the last two words of the array */
        .part = "t20x", .machine = "ingenic-t20", .soc = "ingenic-t31",
        .variant = "t20x", .mode = BY_BYTE, .nwords = 24,
        .window = { [14] = 0x22220000, [15] = MXU_LICENCE },
        .array = { ID0, ID1, ID2, [14] = 0x22220000, [15] = MXU_LICENCE },
        .ctrl_mask = 0x0fe70000,
        .reads = {
            { 0x80030000, 0x725c2516 },     /* 4 bytes at 0 */
            { 0x00470000, 0x0281725c },     /* 8 bytes at 2: the first 4 */
            { 0x00410000, 0x0000725c },     /* 2 bytes at 2 */
            { 0x10000000, 0x00000016 },     /* address 0x80 wraps to 0 */
        },
    }, {
        .part = "t21n", .machine = "ingenic-t21", .soc = "ingenic-t31",
        .variant = "t21n", .mode = BY_WORD, .nwords = 24,
        .window = { ID0, ID1, ID2, [4] = 0x01000000, [14] = 0x11110000,
                    [15] = MXU_LICENCE ^ BIT(24) },
        .ctrl_mask = 0x07e00000, .addr_bits = 6, .state_hi = 0x100,
    }, {
        .part = "t23n", .machine = "ingenic-t23", .soc = "ingenic-t31",
        .variant = "t23n", .mode = BY_WORD, .nwords = 24,
        .window = { ID0, ID1, ID2, [4] = 0x01000000, [5] = 0x40000000,
                    [14] = 0x11111111 },
        .ctrl_mask = 0x03e00000, .addr_bits = 5, .state_hi = 0x100,
    }, {
        .part = "t23dl", .machine = "ingenic-t23", .soc = "ingenic-t31",
        .variant = "t23dl", .mode = BY_WORD, .nwords = 24,
        .window = { ID0, ID1, ID2, [4] = 0x01000000, [5] = 0x40000000,
                    [14] = 0x33331111 },
        .ctrl_mask = 0x03e00000, .addr_bits = 5, .state_hi = 0x100,
    }, {
        .part = "t30x", .machine = "ingenic-t30", .soc = "ingenic-t31",
        .variant = "t30x", .mode = BY_WORD, .nwords = 24,
        .window = { ID0, ID1, ID2, [4] = 0x01000000, [14] = 0x22221111,
                    [15] = MXU_LICENCE },
        .ctrl_mask = 0x03e00000, .addr_bits = 5, .state_hi = 0x100,
    }, {
        .part = "t31x", .machine = "ingenic-t31", .soc = "ingenic-t31",
        .variant = "t31x", .mode = BY_WORD, .nwords = 24,
        .window = { ID0, ID1, ID2, [4] = 0x01000000, [14] = 0x22221111,
                    [15] = MXU_LICENCE },
        .ctrl_mask = 0x03e00000, .addr_bits = 5, .state_hi = 0x100,
    }, {
        /* Words 4, 5, 6 and 12 as a T32LQ reads them */
        .part = "t32lq", .machine = "ingenic-t32", .soc = "ingenic-t31",
        .variant = "t32lq", .mode = BY_WORD, .nwords = 24,
        .window = { ID0, ID1, ID2, [4] = 0x21000000, [5] = 0x40000000,
                    [6] = 6, [12] = 0x8888, [14] = 0x99991111 },
        .ctrl_mask = 0x0fe00000, .addr_bits = 7, .state_hi = 0x2100,
    }, {
        /* From the T40 on the window is 64 words and words 3-5 repeat 0-2 */
        .part = "t40n", .machine = "ingenic-t40", .soc = "ingenic-t40",
        .variant = "t40n", .mode = BY_WORD, .nwords = 64,
        .window = { T40_ID0, T40_ID1, T40_ID2, T40_ID0, T40_ID1, T40_ID2,
                    [9] = 0x01010000, [11] = 0x4000, [19] = 0xb0d9aeb0,
                    [20] = 0x11111111 },
        .ctrl_mask = 0x07e00000, .addr_bits = 6, .state_hi = 0x100,
    }, {
        .part = "t40xp", .machine = "ingenic-t40", .soc = "ingenic-t40",
        .variant = "t40xp", .mode = BY_WORD, .nwords = 64,
        .window = { T40_ID0, T40_ID1, T40_ID2, T40_ID0, T40_ID1, T40_ID2,
                    [9] = 0x01010000, [11] = 0x4000, [19] = 0xb0d9aeb0,
                    [20] = 0x77772222 },
        .ctrl_mask = 0x07e00000, .addr_bits = 6, .state_hi = 0x100,
    }, {
        /* A T41 reads its security word as 0 */
        .part = "t41nq", .machine = "ingenic-t41", .soc = "ingenic-t40",
        .variant = "t41nq", .mode = BY_WORD, .nwords = 64,
        .window = { T40_ID0, T40_ID1, T40_ID2, T40_ID0, T40_ID1, T40_ID2,
                    [11] = 0x4000, [19] = 0xb0d9aeb0, [20] = 0xaaaa2222 },
        .ctrl_mask = 0x07e00000, .addr_bits = 6,
    }, {
        .part = "t41lq", .machine = "ingenic-t41", .soc = "ingenic-t40",
        .variant = "t41lq", .mode = BY_WORD, .nwords = 64,
        .window = { T40_ID0, T40_ID1, T40_ID2, T40_ID0, T40_ID1, T40_ID2,
                    [11] = 0x4000, [19] = 0xb0d9aeb0, [20] = 0x99991111 },
        .ctrl_mask = 0x07e00000, .addr_bits = 6,
    }, {
        .part = "a1n", .machine = "ingenic-a1", .soc = "ingenic-a1",
        .variant = "a1n", .mode = BY_WORD, .nwords = 64,
        .window = { ID0, ID1, ID2, ID0, ID1, ID2, [6] = 1, [7] = 1,
                    [9] = 0x03030000, [11] = 0x4000, [20] = 0x11112222 },
        .ctrl_mask = 0x07e00000, .addr_bits = 6, .state_hi = 0x300,
    },
};

static uint32_t efuse_readl(QTestState *qts, uint32_t offset)
{
    return qtest_readl(qts, EFUSE_BASE + offset);
}

static void efuse_writel(QTestState *qts, uint32_t offset, uint32_t value)
{
    qtest_writel(qts, EFUSE_BASE + offset, value);
}

/* @what and @idx say where a mismatch is: the assertion has only values */
static void check(const char *what, uint32_t idx, uint32_t got, uint32_t want)
{
    if (got != want) {
        g_printerr("%s 0x%x: got 0x%08x, want 0x%08x\n", what, idx, got, want);
    }
    g_assert_cmphex(got, ==, want);
}

/* A read as firmware runs it; the model has the result at once */
static uint32_t efuse_read(QTestState *qts, uint32_t ctrl)
{
    efuse_writel(qts, EFUSE_STATE, 0);
    efuse_writel(qts, EFUSE_CTRL, ctrl);
    efuse_writel(qts, EFUSE_CTRL, ctrl | CTRL_START);
    g_assert_cmphex(efuse_readl(qts, EFUSE_STATE) & STATE_DONE, ==, STATE_DONE);
    return efuse_readl(qts, EFUSE_DATA);
}

static void test_window(QTestState *qts, const EfuseCase *c)
{
    for (unsigned i = 0; i < c->nwords; i++) {
        uint32_t offset = EFUSE_WINDOW + 4 * i;

        check("window", offset, efuse_readl(qts, offset), c->window[i]);
    }
}

static void test_state(QTestState *qts, const EfuseCase *c)
{
    efuse_writel(qts, EFUSE_STATE, 0);
    check("state cleared", 0, efuse_readl(qts, EFUSE_STATE), c->state_hi);
    efuse_read(qts, 0);
    check("state after a read", 0, efuse_readl(qts, EFUSE_STATE),
          c->state_hi | STATE_DONE);
}

static void test_words(QTestState *qts, const EfuseCase *c)
{
    unsigned n = 1u << c->addr_bits;

    for (unsigned i = 0; i < n; i++) {
        check("word", i, efuse_read(qts, i << CTRL_ADDR_SHIFT),
              i < c->nwords ? c->window[i] : 0);
    }
    /* The index wraps, so the first one past the field reads word 0 */
    if (c->addr_bits < 7) {
        check("word", n, efuse_read(qts, n << CTRL_ADDR_SHIFT), c->window[0]);
    }
}

static void test_bytes(QTestState *qts, const EfuseCase *c)
{
    for (unsigned a = 0; a < 4 * ARRAY_SIZE(c->array); a++) {
        check("byte", a,
              efuse_read(qts, CTRL_BIT31 | (a << CTRL_ADDR_SHIFT)),
              extract32(c->array[a / 4], 8 * (a % 4), 8));
    }
    for (unsigned i = 0; i < ARRAY_SIZE(c->reads) && c->reads[i].ctrl; i++) {
        check("read", c->reads[i].ctrl, efuse_read(qts, c->reads[i].ctrl),
              c->reads[i].data);
    }
}

/* Bit 0 starts a read and bit 1 would blow fuses: neither is written here */
static void test_ctrl(QTestState *qts, const EfuseCase *c)
{
    for (unsigned bit = 2; bit < 32; bit++) {
        efuse_writel(qts, EFUSE_CTRL, BIT(bit));
        check("ctrl bit", bit, efuse_readl(qts, EFUSE_CTRL),
              BIT(bit) & c->ctrl_mask);
    }
    efuse_writel(qts, EFUSE_CTRL, 0xfffffffc);
    check("ctrl all", 0, efuse_readl(qts, EFUSE_CTRL), c->ctrl_mask);
    efuse_writel(qts, EFUSE_CTRL, 0x81c30000);
    check("ctrl byte read", 0, efuse_readl(qts, EFUSE_CTRL),
          0x81c30000 & c->ctrl_mask);
}

static void test_efuse(const void *data)
{
    const EfuseCase *c = data;
    QTestState *qts;

    qts = qtest_initf("-machine %s -global %s.soc-variant=%s",
                      c->machine, c->soc, c->variant);

    test_window(qts, c);
    test_state(qts, c);
    if (c->mode == BY_BYTE) {
        test_bytes(qts, c);
    } else {
        test_words(qts, c);
    }
    /* Bit 8 follows the security word, not the write */
    efuse_writel(qts, EFUSE_STATE, BIT(8) | STATE_DONE);
    check("state kept by a write", 0, efuse_readl(qts, EFUSE_STATE),
          c->state_hi | STATE_DONE);
    test_ctrl(qts, c);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/efuse/%s",
                                                cases[i].part);

        qtest_add_data_func(path, &cases[i], test_efuse);
    }

    return g_test_run();
}
