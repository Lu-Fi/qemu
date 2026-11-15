/*
 * QTest testcase for the T32's HERA giving each 16x16 of a picture a QP of
 * its own
 *
 * With cu_qp_delta on, libimp's H.265 pictures are coded with the core's QP
 * generator, which moves each 16x16's QP from the slice's by the edge strength
 * of the block: a block of little (a flat one, or fine noise) takes 8 to
 * 10 below it, a busy one 0 to 4 above (read from the QPs in the streams of a
 * T32LQ, flat noise of an amplitude of 2 and 4 measured at slice QP 28: 18 and
 * 20). The test codes a picture of fine noise with it on and off, then the
 * same as a P picture over its own reconstruction, and a picture of strong
 * stripes. Streams are pinned by CRC; what the QP generator does is checked
 * by the sizes: a picture of fine noise costs several times what it does at
 * the slice QP, one of stripes costs no more.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define HERA            0x13200000
#define WIDTH           128
#define ROWS            64
#define SRC_Y           0x00800000
#define SRC_C           0x00820000
#define LIST            0x00860000
#define BS              0x00880000
#define HL_VLD          (1u << 31)
#define HL_TERM         (1u << 30)

enum { NOISE, STRIPES };

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t c = ~0u;

    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) {
            c = c >> 1 ^ (c & 1 ? 0xedb88320u : 0);
        }
    }
    return ~c;
}

/* An amplitude of 4 or so, white, the same every time it is made */
static uint8_t noise(int x, int y, int frame)
{
    uint32_t h = (x * 73856093u) ^ (y * 19349663u) ^ (frame * 83492791u);

    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return 120 + (h & 7) + (h >> 8 & 7);
}

static unsigned job(QTestState *qts, int kind, int frame, bool intra, int qp,
                    bool qpg, uint32_t *crc)
{
    uint32_t pairs[32], n = 0, len;
    uint8_t y[WIDTH * ROWS], c[WIDTH * ROWS / 2], out[0x20000];
    const struct {
        uint32_t reg, val;
    } regs[] = {
        { 0x2000c, SRC_Y }, { 0x20010, SRC_C },
        { 0x20014, WIDTH | WIDTH << 16 },
        { 0x68004, WIDTH << 16 | ROWS },
        /* I or P, and cu_qp_delta to a depth of 2 where the QPs follow */
        { 0x68000, (intra ? 2 : 1) << 2 | (qpg ? (1 << 4 | 2 << 18) : 0) },
        { 0x60004, qp << 1 }, { 0x20004, 1 },
    };

    for (int i = 0; i < WIDTH * ROWS; i++) {
        int x = i % WIDTH, r = i / WIDTH;

        y[i] = kind == NOISE ? noise(x, r, frame) :
                               (x / 4 + r / 4) & 1 ? 40 : 216;
    }
    memset(c, 128, sizeof(c));
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        pairs[n++] = cpu_to_le32(regs[i].val);
        pairs[n++] = cpu_to_le32(HL_VLD | (i == ARRAY_SIZE(regs) - 1 ?
                                           HL_TERM : 0) | regs[i].reg);
    }
    qtest_memwrite(qts, LIST, pairs, n * 4);
    qtest_writel(qts, HERA + 0x2c, 0x7ff);
    qtest_writel(qts, HERA + 0x90008, BS);
    qtest_writel(qts, HERA + 0x90068, LIST);
    qtest_writel(qts, HERA + 0x8, 1);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, HERA + 0x2c) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, HERA + 0x2c) & 1, ==, 1);
    len = qtest_readl(qts, HERA + 0x44);
    g_assert_cmpuint(len, <, sizeof(out));
    qtest_memread(qts, BS, out, len);
    *crc = crc32_le(out, len);
    qtest_writel(qts, HERA + 0x30, 0x7ff);
    return len;
}

typedef struct {
    int kind, qp;
    unsigned i_len, p_len;      /* with the QP generator */
    uint32_t i_crc, p_crc;
} IngenicQpgCase;

static const IngenicQpgCase cases[] = {
    { NOISE, 28, 1231, 891, 0x4674f4b8, 0x9da1ee55 },
    { NOISE, 34, 225, 58, 0x76e14cf1, 0x4be96806 },
    { STRIPES, 28, 1276, 3, 0x871c9df6, 0x34e57919 },
};

static void test_qpg(const void *data)
{
    const IngenicQpgCase *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t32");
    uint32_t crc;
    unsigned off_i, on_i, off_p, on_p;

    off_i = job(qts, c->kind, 0, true, c->qp, false, &crc);
    off_p = job(qts, c->kind, 1, false, c->qp, false, &crc);
    on_i = job(qts, c->kind, 0, true, c->qp, true, &crc);
    g_assert_cmphex(crc, ==, c->i_crc);
    on_p = job(qts, c->kind, 1, false, c->qp, true, &crc);
    g_assert_cmphex(crc, ==, c->p_crc);
    g_test_message("kind %d qp %d: I %u -> %u, P %u -> %u", c->kind, c->qp,
                   off_i, on_i, off_p, on_p);
    g_assert_cmpuint(on_i, ==, c->i_len);
    g_assert_cmpuint(on_p, ==, c->p_len);
    if (c->kind == NOISE) {
        g_assert_cmpuint(on_i, >, 3 * off_i);
        g_assert_cmpuint(on_p, >, 3 * off_p);
    } else {
        g_assert_cmpuint(on_i, <=, off_i);
    }
    qtest_quit(qts);
}

static const char *const kinds[] = { "noise", "stripes" };

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        const char *kind = kinds[cases[i].kind];
        g_autofree char *p = g_strdup_printf("ingenic/hera-qpg/%s-qp%d", kind,
                                             cases[i].qp);

        qtest_add_data_func(p, &cases[i], test_qpg);
    }
    return g_test_run();
}
