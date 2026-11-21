/*
 * QTest testcase for the Ingenic LZMA decompressor's interrupt (T32)
 *
 * A 227-byte payload (a repeating 16-byte pattern then a ramp, both
 * generated in the test) packed in the engine's own bit-packed format.
 * The T32's driver starts a run, sees CTRL bit 0 clear, then waits for
 * the interrupt: CTRL bit 3, acknowledged by writing CTRL with it clear.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define SRC         0x00100000
#define DST         0x00200000
#define CTRL        0x00
#define BS_BASE     0x04
#define BS_SIZE     0x08
#define DST_BASE    0x0c
#define OUT_LEN     0x14

static const uint8_t stream[] = {
    0x00, 0x00, 0x02, 0x00, 0xe3, 0x00, 0x00, 0x00, 0xa0, 0x31, 0x50, 0xe5,
    0x25, 0xdf, 0x88, 0x4b, 0x70, 0x98, 0x69, 0xf0, 0x55, 0x64, 0xdc, 0x0d,
    0x44, 0x6c, 0xed, 0x87, 0x0f, 0x00, 0x30, 0x30, 0x90, 0x60, 0xc0, 0x43,
    0x82, 0x0a, 0x03, 0x36, 0x3c, 0x84, 0x48, 0x90, 0xa3, 0x42, 0x8b, 0x01,
    0x33, 0x36, 0x9c, 0x78, 0xf0, 0x13, 0x22, 0x4a, 0x82, 0x34, 0x39, 0x8a,
    0x54, 0xa8, 0xd3, 0xa2, 0xcb, 0x80, 0x31, 0x33, 0x96, 0x6c, 0xd8, 0x73,
    0xe2, 0xca, 0x83, 0x37, 0x3f, 0x81, 0x42, 0x84, 0x8b, 0x12, 0x2b, 0x41,
    0xb2, 0x34, 0x99, 0x72, 0xe4, 0x2b, 0x52, 0xaa, 0x42, 0xb5, 0x3a, 0x8d,
    0x5a, 0xb4, 0xeb, 0xd2, 0x6b, 0xc0, 0xb0, 0x31
};

static void payload(uint8_t *out)
{
    for (int i = 0; i < 160; i++) {
        out[i] = (i % 16) * 37 + 11;
    }
    for (int i = 0; i < 67; i++) {
        out[160 + i] = i * 3;
    }
}

static void run_engine(QTestState *qts, uint32_t base, int irq)
{
    uint8_t want[227], got[227];

    payload(want);
    qtest_memwrite(qts, SRC, stream, sizeof(stream));
    qtest_memset(qts, DST, 0xa5, sizeof(got));
    qtest_writel(qts, base + CTRL, 1u << 31);
    qtest_writel(qts, base + CTRL, 1u << 7);
    qtest_writel(qts, base + CTRL, 1u << 1);
    qtest_writel(qts, base + BS_BASE, SRC);
    qtest_writel(qts, base + BS_SIZE, sizeof(stream));
    qtest_writel(qts, base + DST_BASE, DST);
    g_assert_false(qtest_get_irq(qts, irq));

    qtest_writel(qts, base + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, base + CTRL) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, base + CTRL) & (1 << 3), ==, 1 << 3);
    g_assert_true(qtest_get_irq(qts, irq));
    g_assert_cmpuint(qtest_readl(qts, base + OUT_LEN), ==, sizeof(want));
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert_cmpmem(got, sizeof(got), want, sizeof(want));

    /* The driver's acknowledge: CTRL written with bit 3 clear */
    qtest_writel(qts, base + CTRL, 0);
    g_assert_false(qtest_get_irq(qts, irq));
    g_assert_cmphex(qtest_readl(qts, base + CTRL) & (1 << 3), ==, 0);
}

static void test_irq(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    run_engine(qts, 0x131f0000, 58);
    /* The second engine raises its own line */
    run_engine(qts, 0x131f8000, 53);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/lzma-irq/t32", test_irq);
    return g_test_run();
}
