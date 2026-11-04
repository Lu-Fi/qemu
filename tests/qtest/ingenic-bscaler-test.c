/*
 * QTest testcase for the Ingenic BSCALER (T40)
 *
 * FRMC jobs as libvenus chains them, over a BGRA picture of LCG bytes,
 * each output's CRC32 a T40XP's for the same chain: a 2x upscale, a 1.5x
 * downscale with libvenus's half-pixel offset, fractional and uneven
 * scales, a rotation running off the source, and the rotation again with
 * source and destination tile origins. Then what the run leaves in the
 * status registers, the interrupt, and the reset.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define BS              0x13090000
#define SRC             0x00800000
#define DST             0x00900000
#define CHAIN           0x00a00000

typedef struct {
    const char *name;
    uint32_t sw, sh, dw, dh;
    uint32_t m[9];
    uint32_t o40, o44;
    uint32_t crc;
} Case;

static const Case cases[] = {
    { "up2", 32, 16, 64, 32,
      { 0x8000, 0, 0, 0, 0x8000, 0, 0, 0, 0x10000 }, 0, 0, 0x32dc3749 },
    { "down15", 48, 24, 32, 16,
      { 0x18000, 0, 0x4000, 0, 0x18000, 0x4000, 0, 0, 0x10000 }, 0, 0,
      0xbf3b78bb },
    { "frac", 32, 16, 24, 12,
      { 0x10000, 0, 0x4000, 0, 0x10000, 0x8000, 0, 0, 0x10000 }, 0, 0,
      0x39e5015e },
    { "uneven", 40, 20, 27, 14,
      { 0x17b42, 0, 0x2f13, 0, 0x1a3c1, 0x1ab7, 0, 0, 0x10000 }, 0, 0,
      0x4ba80c3b },
    { "rotate", 48, 32, 32, 24,
      { 0xe0a9, 0x7abc, 0x18000, 0xffff8544, 0xe0a9, 0x30000, 0, 0, 0x10000 },
      0, 0, 0x1b218131 },
    { "origins", 48, 32, 32, 24,
      { 0xe0a9, 0x7abc, 0x18000, 0xffff8544, 0xe0a9, 0x30000, 0, 0, 0x10000 },
      0x00020003, 0x00020003, 0x4c8c5036 },
};

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

static void put(uint32_t *v, uint32_t *n, uint32_t val, uint32_t off)
{
    v[(*n)++] = cpu_to_le32(val);
    v[(*n)++] = cpu_to_le32(off);
}

/* the job's chain, in the order libvenus writes it, ending in 0x201 at 0 */
static uint32_t put_chain(QTestState *qts, const Case *c)
{
    uint32_t v[64], n = 0;

    put(v, &n, SRC, 0x08);
    put(v, &n, DST, 0x18);
    put(v, &n, 0xf, 0x04);
    put(v, &n, c->sh << 16 | c->sw, 0x10);
    put(v, &n, c->sw * 4, 0x14);
    put(v, &n, c->dh << 16 | c->dw, 0x1c);
    put(v, &n, c->dw * 4, 0x20);
    put(v, &n, c->o40, 0x40);
    put(v, &n, c->o44, 0x44);
    for (int k = 0; k < 9; k++) {
        put(v, &n, c->m[k], 0x48 + 4 * k);
    }
    put(v, &n, 0x201, 0x80000000);
    qtest_memwrite(qts, CHAIN, v, n * 4);
    return n * 4;
}

static void test_case(const void *data)
{
    const Case *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t40");
    size_t sz = c->sw * c->sh * 4, out = c->dw * c->dh * 4;
    g_autofree uint8_t *src = g_malloc(sz);
    g_autofree uint8_t *dst = g_malloc(out);
    uint32_t seed = 777, len;

    for (size_t i = 0; i < sz; i++) {
        seed = seed * 1103515245 + 12345;
        src[i] = seed >> 16;
    }
    qtest_memwrite(qts, SRC, src, sz);
    len = put_chain(qts, c);
    qtest_writel(qts, BS + 0x30, 0x10000);
    qtest_writel(qts, BS + 0x3c, 0xffffffff);
    qtest_writel(qts, BS + 0x34, CHAIN);
    qtest_writel(qts, BS + 0x38, len);
    qtest_writel(qts, BS + 0x30, 0x21);
    qtest_memread(qts, DST, dst, out);
    g_assert_cmphex(crc32_le(dst, out), ==, c->crc);
    qtest_quit(qts);
}

/* a finished chain sets 0x00 bit 8 and 0x30 bit 4; cleared, 0x200 and 0x20 */
static void test_status(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");
    uint32_t len = put_chain(qts, &cases[0]);

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    qtest_writel(qts, BS + 0x34, CHAIN);
    qtest_writel(qts, BS + 0x38, len);
    qtest_writel(qts, BS + 0x30, 0x21);
    g_assert_true(qtest_get_irq(qts, 39));
    g_assert_cmphex(qtest_readl(qts, BS + 0x00), ==, 0x300);
    g_assert_cmphex(qtest_readl(qts, BS + 0x30), ==, 0x30);
    qtest_writel(qts, BS + 0x00, qtest_readl(qts, BS + 0x00) & ~0x100);
    qtest_writel(qts, BS + 0x30, qtest_readl(qts, BS + 0x30) & ~0x50);
    g_assert_false(qtest_get_irq(qts, 39));
    g_assert_cmphex(qtest_readl(qts, BS + 0x00), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, BS + 0x30), ==, 0x20);
    qtest_writel(qts, BS + 0x30, 0x10000);
    g_assert_cmphex(qtest_readl(qts, BS + 0x30), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BS + 0x00), ==, 0x200);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/bscaler/frmc/%s",
                                                cases[i].name);

        qtest_add_data_func(path, &cases[i], test_case);
    }
    qtest_add_func("ingenic/bscaler/status", test_status);
    return g_test_run();
}
