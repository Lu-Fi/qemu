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
    return g_test_run();
}
