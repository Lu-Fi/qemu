/*
 * QTest testcase for the Ingenic A1 VC8000D video decoder's register protocol
 *
 * The sequencer's identity, a command buffer run the way the kernel arms
 * it (WREG to the decoder core, its enable bit, RREG of the status back to
 * DRAM, a JMP with no target ready), the status and exe-address registers
 * and the interrupt, and the post-processor's output geometry on a picture
 * that fails to decode (mid-grey). Runs with or without libavcodec.
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
    g_assert_cmphex(qtest_readl(qts, DEC + 0x4d4), ==, 0x1f87);
    g_assert_cmphex(qtest_readl(qts, VCMD), ==, 0x43421200);
    /* before the first start (measured on an A1N under U-Boot) */
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x04), ==, 0x20200618);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x08), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x38), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x3c), ==, 0x00400000);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x40), ==, 0x20);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 0);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x4c), ==, 0x80008000);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x5c), ==, 0x00100000);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x6c), ==, 0);
    /* after vdec.ko's init has started the sequencer once */
    qtest_writel(qts, VCMD + 0x40, 0x01);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x0c), ==, 1);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x10), ==, 0xca000000);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x3c), ==, 0x00400073);
    g_assert_cmphex(qtest_readl(qts, CACHE), ==, 0x5000);
    /* the configuration words and the ID are read-only */
    qtest_writel(qts, DEC, 0);
    qtest_writel(qts, DEC + 0xc8, 0);
    g_assert_cmphex(qtest_readl(qts, DEC) >> 16, ==, 0x8001);
    g_assert_cmphex(qtest_readl(qts, DEC + 0xc8), ==, 0x00101000);
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

    /* the stall takes time: nothing past it has run yet */
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 0);
    g_assert_false(qtest_get_irq(qts, 42));
    g_assert_cmphex(qtest_readl(qts, STATUS), ==, 0);
    qtest_clock_step(qts, 1000000);

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

/*
 * The kernel links the next job's buffer onto a running chain and fills it
 * while the sequencer is still in the stall of the job before: the buffer is
 * read when the sequencer gets to it, not when the chain starts. A model
 * that read it at the start would run the empty buffer, or half a job, and
 * write the core's registers to an address the kernel has not set yet.
 */
static void test_chain_filled_late(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    const uint32_t job1[] = {
        OP_STALL(0x100), 0,
        OP_JMP | (1 << 26), RAM + 0x100, 0, 5,
    };
    const uint32_t job2[] = {
        OP_RREG(1, 0x1000), STATUS, 0, 0,
        OP_JMP, 0, 0, 6,
    };

    put(qts, RAM, job1, ARRAY_SIZE(job1));
    qtest_writel(qts, VCMD + 0x48, 0xffff00ff);
    qtest_writel(qts, VCMD + 0x50, RAM);
    qtest_writel(qts, VCMD + 0x58, ARRAY_SIZE(job1) / 2);
    qtest_writel(qts, VCMD + 0x40, 0x01);

    put(qts, RAM + 0x100, job2, ARRAY_SIZE(job2));
    qtest_clock_step(qts, 1000000);

    g_assert_cmphex(qtest_readl(qts, STATUS) >> 16, ==, 0x8001);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 2);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x50), ==, RAM + 0x100 + 16);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x68), ==, 5);
    qtest_quit(qts);
}

/*
 * A status address the kernel has not filled in yet is waited for, not
 * written to: the dump would land on the exception vectors at physical 0.
 * The sequencer runs the command once the address is there, or as it
 * stands after 100 ms.
 */
static void test_unpatched_status(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    const uint32_t buf[] = {
        OP_RREG(1, 0x1000), 0, 0, 0,
        OP_JMP, 0, 0, 4,
    };

    put(qts, RAM, buf, ARRAY_SIZE(buf));
    qtest_writel(qts, 0, 0x12345678);
    qtest_writel(qts, VCMD + 0x48, 0xffff00ff);
    qtest_writel(qts, VCMD + 0x50, RAM);
    qtest_writel(qts, VCMD + 0x58, ARRAY_SIZE(buf) / 2);
    qtest_writel(qts, VCMD + 0x40, 0x01);

    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, 0), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 0);

    qtest_writel(qts, RAM + 4, STATUS);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, 0), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, STATUS) >> 16, ==, 0x8001);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 2);
    qtest_quit(qts);
}

static void test_unpatched_status_gives_up(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    const uint32_t buf[] = {
        OP_RREG(1, 0x1000), 0, 0, 0,
        OP_JMP, 0, 0, 4,
    };

    put(qts, RAM, buf, ARRAY_SIZE(buf));
    qtest_writel(qts, 0, 0x12345678);
    qtest_writel(qts, VCMD + 0x48, 0xffff00ff);
    qtest_writel(qts, VCMD + 0x50, RAM);
    qtest_writel(qts, VCMD + 0x58, ARRAY_SIZE(buf) / 2);
    qtest_writel(qts, VCMD + 0x40, 0x01);

    qtest_clock_step(qts, 99000000);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 0);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(qtest_readl(qts, 0) >> 16, ==, 0x8001);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 2);
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

/*
 * A junk slice that no decoder accepts completes as a mid-grey picture;
 * the post-processor's unit 1 is programmed as libimp does for a table-less
 * downscale and its output buffers are checked for what it writes and, from
 * the sentinels, for what it leaves alone.
 */
#define PP1             (DEC + 4 * 0x156)
#define PP_LUMA_BUF     (RAM + 0x10000)
#define PP_CHROMA_BUF   (RAM + 0x20000)
#define PP_SENTINEL     0xee

static void pp_job(QTestState *qts, uint32_t crop, uint32_t src,
                   uint32_t out, uint32_t pitch, bool nv21)
{
    const uint8_t strm[] = { 0, 0, 0, 1, 0x65, 0x88, 0x84, 0, 0, 0, 0, 0 };
    const uint32_t buf[] = {
        OP_WREG(1, 0x100c), 0x7800000f,
        OP_WREG(1, 0x1010), (8 << 19) | (8 << 6),
        OP_WREG(1, 0x1018), sizeof(strm),
        OP_WREG(1, 0x12a4), RAM + 0x2000,
        OP_WREG(1, 0x1104), RAM + 0x30000,
        OP_WREG(1, 0x118c), RAM + 0x38000,
        OP_WREG(1, 0x1408), 0x1000,
        OP_WREG(1, 0x140c), 0,
        OP_WREG(1, 0x1004), 1,
        OP_JMP, 0, 0, 3,
    };

    qtest_memwrite(qts, RAM + 0x2000, strm, sizeof(strm));
    qtest_memset(qts, PP_LUMA_BUF, PP_SENTINEL, 0x4000);
    qtest_memset(qts, PP_CHROMA_BUF, PP_SENTINEL, 0x4000);
    qtest_writel(qts, PP1 + 0, 1 | (nv21 ? 2 : 0));
    qtest_writel(qts, PP1 + 4 * 2, (2 << 25) | (2 << 23));
    qtest_writel(qts, PP1 + 4 * 6, PP_LUMA_BUF);
    qtest_writel(qts, PP1 + 4 * 8, PP_CHROMA_BUF);
    qtest_writel(qts, PP1 + 4 * 9, pitch);
    qtest_writel(qts, PP1 + 4 * 10, crop);
    qtest_writel(qts, PP1 + 4 * 11, src);
    qtest_writel(qts, PP1 + 4 * 12, out);
    put(qts, RAM, buf, ARRAY_SIZE(buf));
    qtest_writel(qts, VCMD + 0x48, 0xffff00ff);
    qtest_writel(qts, VCMD + 0x50, RAM);
    qtest_writel(qts, VCMD + 0x58, ARRAY_SIZE(buf) / 2);
    qtest_writel(qts, VCMD + 0x40, 0x01);
    g_assert_cmphex(qtest_readl(qts, VCMD + 0x44), ==, 2);
}

/* rows x cols of v at the buffer, pitch bytes a row, and nothing else */
static void pp_check(QTestState *qts, uint32_t addr, unsigned cols,
                     unsigned rows, unsigned pitch, uint8_t v)
{
    uint8_t row[0x100];

    for (unsigned y = 0; y < 0x4000 / pitch; y++) {
        qtest_memread(qts, addr + y * pitch, row, pitch);
        for (unsigned x = 0; x < pitch; x++) {
            g_assert_cmphex(row[x], ==,
                            y < rows && x < cols ? v : PP_SENTINEL);
        }
    }
}

static void test_pp_downscale(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    /* 64x64 to 24x16 whole picture, a non-integer ratio in x */
    pp_job(qts, 0, (32 << 16) | 32, (24 << 16) | 16, (64 << 16) | 64, false);
    pp_check(qts, PP_LUMA_BUF, 24, 16, 64, 0x80);
    pp_check(qts, PP_CHROMA_BUF, 24, 8, 64, 0x80);
    qtest_quit(qts);
}

static void test_pp_crop(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    /* the 32x32 window at 16,16 to 16x16, NV21 order */
    pp_job(qts, (8 << 16) | 8, (16 << 16) | 16, (16 << 16) | 16,
           (32 << 16) | 32, true);
    pp_check(qts, PP_LUMA_BUF, 16, 16, 32, 0x80);
    pp_check(qts, PP_CHROMA_BUF, 16, 8, 32, 0x80);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-vc8000d/identity", test_identity);
    qtest_add_func("ingenic-a1-vc8000d/chain", test_chain);
    qtest_add_func("ingenic-a1-vc8000d/chain-filled-late",
                   test_chain_filled_late);
    qtest_add_func("ingenic-a1-vc8000d/unpatched-status",
                   test_unpatched_status);
    qtest_add_func("ingenic-a1-vc8000d/unpatched-status-gives-up",
                   test_unpatched_status_gives_up);
    qtest_add_func("ingenic-a1-vc8000d/empty-stream", test_empty_stream);
    qtest_add_func("ingenic-a1-vc8000d/pp-downscale", test_pp_downscale);
    qtest_add_func("ingenic-a1-vc8000d/pp-crop", test_pp_crop);
    return g_test_run();
}
