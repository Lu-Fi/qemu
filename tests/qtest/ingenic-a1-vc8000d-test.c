/*
 * QTest testcase for the Ingenic A1 VC8000D video decoder's register protocol
 *
 * The sequencer's identity, a command buffer run the way the kernel arms
 * it (WREG to the decoder core, its enable bit, RREG of the status back to
 * DRAM, a JMP with no target ready), the status and exe-address registers
 * and the interrupt. Runs with or without libavcodec: no stream is fed.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define VDEC            0x13100000
#define VCMD            (VDEC + 0x0000)
#define DEC             (VDEC + 0x1000)
#define CACHE           (VDEC + 0x2000)
#define RAM             0x01000000
#define STATUS          (RAM + 0x1000)

#define OP_WREG(n, reg)     (0x08000000 | ((n) << 16) | (reg))
#define OP_RREG(n, reg)     (0xb0000000 | ((n) << 16) | (reg))
#define OP_STALL(mask)      (0x48000000 | (mask))
#define OP_JMP              0xc8000000

static void put(QTestState *qts, uint32_t addr, const uint32_t *w, int n)
{
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, addr + 4 * i, w[i]);
    }
}

static void test_identity(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(qtest_readl(qts, VCMD) >> 16, ==, 0x4342);
    g_assert_cmphex(qtest_readl(qts, DEC) >> 16, ==, 0x8001);
    g_assert_cmphex(qtest_readl(qts, DEC + 0x4d4), ==, 0x1f70);
    /* the configuration words and the ID are read-only */
    qtest_writel(qts, DEC, 0);
    qtest_writel(qts, DEC + 0xc8, 0);
    g_assert_cmphex(qtest_readl(qts, DEC) >> 16, ==, 0x8001);
    g_assert_cmphex(qtest_readl(qts, DEC + 0xc8) >> 24, ==, 3);
    /* the rest of the core is plain storage */
    qtest_writel(qts, DEC + 0x20, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, DEC + 0x20), ==, 0x12345678);
    qtest_writel(qts, CACHE + 4, 0x55);
    g_assert_cmphex(qtest_readl(qts, CACHE + 4), ==, 0x55);
    qtest_quit(qts);
}

static void test_chain(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    /*
     * decoder regs 0 and 1 with DEC_E, a stall on the core's interrupt,
     * the status read back, an INT-less JMP whose target is not ready
     */
    const uint32_t buf[] = {
        OP_WREG(2, 0x1000), 0, 0x1,
        0,
        OP_STALL(0x100), 0,
        OP_RREG(2, 0x1000), STATUS, 0, 0,
        OP_JMP, 0, 0, 7,
    };
    uint32_t st;

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    put(qts, RAM, buf, ARRAY_SIZE(buf));
    qtest_writel(qts, VCMD + 0x48, 0xffff00ff);
    qtest_writel(qts, VCMD + 0x40, 0x20);
    qtest_writel(qts, VCMD + 0x50, RAM);
    qtest_writel(qts, VCMD + 0x58, ARRAY_SIZE(buf) / 2);
    qtest_writel(qts, VCMD + 0x64, 0xffffffff);
    qtest_writel(qts, VCMD + 0x40, 0x01);

    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 2);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x50), ==, RAM + 40);
    g_assert_true(qtest_get_irq(qts, 42));
    /* status read back: ID, then a finished core: IRQ and ready, not enabled */
    g_assert_cmphex(qtest_readl(qts, STATUS) >> 16, ==, 0x8001);
    st = qtest_readl(qts, STATUS + 4);
    g_assert_cmphex(st & 0x1101, ==, 0x1100);

    /* write-1-clear, interrupt drops */
    qtest_writel(qts, VCMD + 0x44, 2);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 0);
    g_assert_false(qtest_get_irq(qts, 42));
    qtest_quit(qts);
}

/* A picture job over a stream with nothing to decode: buffer empty, no hang */
static void test_empty_stream(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    const uint32_t buf[] = {
        OP_WREG(1, 0x100c), 0x7800000c, /* H.264 mode, as the library sets it */
        OP_WREG(1, 0x1018), 64,
        OP_WREG(1, 0x12a4), RAM + 0x2000,
        OP_WREG(1, 0x1408), 0x1000,
        OP_WREG(1, 0x140c), 0,
        OP_WREG(1, 0x1004), 1,
        OP_RREG(1, 0x1004), STATUS, 0, 0,
        OP_RREG(1, 0x12a4), STATUS + 4, 0, 0,
        OP_JMP, 0, 0, 3,
    };

    put(qts, RAM, buf, ARRAY_SIZE(buf));
    qtest_writel(qts, VCMD + 0x48, 0xffff00ff);
    qtest_writel(qts, VCMD + 0x50, RAM);
    qtest_writel(qts, VCMD + 0x58, ARRAY_SIZE(buf) / 2);
    qtest_writel(qts, VCMD + 0x40, 0x01);

    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 2);
    /* IRQ and buffer empty, the enable bit self-clears, all 64 bytes read */
    g_assert_cmphex(qtest_readl(qts, STATUS) & 0x5101, ==, 0x4100);
    g_assert_cmphex(qtest_readl(qts, STATUS + 4), ==, RAM + 0x2040);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-vc8000d/identity", test_identity);
    qtest_add_func("ingenic-a1-vc8000d/chain", test_chain);
    qtest_add_func("ingenic-a1-vc8000d/empty-stream", test_empty_stream);
    return g_test_run();
}
