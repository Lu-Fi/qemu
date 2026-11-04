/*
 * QTest testcase for the T30's noise reducer (NCU) and its post-processor
 *
 * The values a T30X reads: reset values and the bits each register keeps,
 * the control words' counters and flags, and frames from memory, which with
 * the functions at reset go through unchanged, to REF0 first and then
 * REF1 and REF0 in turn.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NCU             0x13090000
#define CPM_SRBC1       0x100000ac
#define NCU_IRQ         33
#define PP_IRQ          35

#define W               128
#define H               64
#define YS              (W * H)
#define CUR             0x01000000
#define REF0            0x01100000
#define REF1            0x01200000

typedef struct {
    uint32_t reg, ones;
} Mask;

static const Mask masks[] = {
    { 0x018, 0x0000001f }, { 0x024, 0xffffffff }, { 0x02c, 0x0fff0fff },
    { 0x030, 0xffffffff }, { 0x034, 0x0000ffff }, { 0x060, 0x0000003f },
    { 0x088, 0x0000003f }, { 0x0b8, 0x0fff0fff }, { 0x0bc, 0x00000111 },
    { 0x0c0, 0x00ff00ff }, { 0x0d0, 0x00000000 }, { 0x118, 0x00000000 },
    { 0x180, 0x00000003 }, { 0x18c, 0x7f7f7f00 }, { 0x200, 0x0000ff7f },
    { 0x2a0, 0x1ffff31f }, { 0x340, 0x71ff00f3 }, { 0x3f0, 0xf1ff01ff },
    { 0x3f8, 0x0013ffff }, { 0x404, 0x00000000 }, { 0x518, 0x0000000f },
    { 0x520, 0x00000000 }, { 0x524, 0x0fff0fff }, { 0x558, 0x00000001 },
    { 0x560, 0x001fffff }, { 0x574, 0x000001ff }, { 0x57c, 0x00000111 },
    { 0x580, 0x00000000 }, { 0xffc, 0x00000000 },
};

static const Mask resets[] = {
    { 0x000, 0x00000011 }, { 0x004, 0x00010000 }, { 0x008, 0x00000000 },
    { 0x00c, 0x00000004 }, { 0x010, 0x00000100 }, { 0x014, 0x0a170510 },
    { 0x01c, 0x00000000 }, { 0x020, 0x40000000 }, { 0x024, 0x0c000000 },
    { 0x028, 0xff00ff00 }, { 0x02c, 0x00100010 }, { 0x0b0, 0x02060101 },
    { 0x0b4, 0x02060101 }, { 0x0b8, 0x00200030 }, { 0x2ac, 0x00010001 },
    { 0x2c4, 0x00020002 }, { 0x3e8, 0x00010001 }, { 0x400, 0x00020002 },
    { 0x500, 0x00000011 }, { 0x50c, 0x00000004 }, { 0x510, 0x00000100 },
    { 0x514, 0x0a170510 }, { 0x51c, 0x00000001 }, { 0x524, 0x00100010 },
    { 0x55c, 0x02010201 },
};

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, NCU + reg);
}

static void wr(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, NCU + reg, v);
}

/* As tx-isp does it: ask it to stop, wait for the acknowledge, reset */
static void cpm_reset(QTestState *qts)
{
    qtest_writel(qts, CPM_SRBC1, 1 << 1);
    g_assert_cmphex(qtest_readl(qts, CPM_SRBC1) & 1, ==, 1);
    qtest_writel(qts, CPM_SRBC1, 1 << 2);
    qtest_writel(qts, CPM_SRBC1, 0);
}

static void check(QTestState *qts, const Mask *m, int n)
{
    for (int i = 0; i < n; i++) {
        g_assert_cmphex(rd(qts, m[i].reg), ==, m[i].ones);
    }
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    check(qts, resets, ARRAY_SIZE(resets));
    for (int i = 0; i < ARRAY_SIZE(masks); i++) {
        wr(qts, masks[i].reg, 0xffffffff);
        g_assert_cmphex(rd(qts, masks[i].reg), ==, masks[i].ones);
        wr(qts, masks[i].reg, 0);
        g_assert_cmphex(rd(qts, masks[i].reg), ==, 0);
    }
    wr(qts, 0x018, 0x1f);
    g_assert_cmphex(rd(qts, 0x01c), ==, 0x20170614);
    wr(qts, 0x018, 0x1e);
    g_assert_cmphex(rd(qts, 0x01c), ==, 0x00011212);
    wr(qts, 0x518, 0x2);
    g_assert_cmphex(rd(qts, 0x51c), ==, 0x00008880);
    cpm_reset(qts);
    check(qts, resets, ARRAY_SIZE(resets));
    qtest_quit(qts);
}

/* The control words, as the T30X answered the same writes */
static void test_control(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    static const uint32_t sreg[][2] = {
        { 0x11, 0x10 }, { 0x13, 0x12 }, { 0x17, 0x16 }, { 0x1f, 0x16 },
        { 0x07, 0x07 }, { 0x05, 0x05 }, { 0x08, 0x00 }, { 0x20, 0x00 },
    };

    qtest_irq_intercept_in(qts, "/machine/soc/intc");

    /* Each write with bit 0 clear is a soft reset, counted in 15:8 */
    wr(qts, 0x000, 0);
    g_assert_cmphex(rd(qts, 0x000), ==, 0x111);
    wr(qts, 0x000, 1);
    wr(qts, 0x000, 0xffffffff);
    g_assert_cmphex(rd(qts, 0x000), ==, 0x111);
    wr(qts, 0x000, 0x10);
    g_assert_cmphex(rd(qts, 0x000), ==, 0x211);
    for (int i = 0; i < 254; i++) {
        wr(qts, 0x000, 0);
    }
    g_assert_cmphex(rd(qts, 0x000), ==, 0x011);

    for (int i = 0; i < ARRAY_SIZE(sreg); i++) {
        wr(qts, 0x00c, 0);
        wr(qts, 0x00c, sreg[i][0]);
        g_assert_cmphex(rd(qts, 0x00c), ==, sreg[i][1]);
    }

    /* A stop while idle is done at once, its frame-done interrupt too */
    wr(qts, 0x008, 1);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x103);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x110);
    g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x008, 1);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x103);
    wr(qts, 0x008, 0);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x102);
    wr(qts, 0x010, 1);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x011);
    g_assert_true(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x010, 3);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x101);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x100);
    g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x010, 0);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x100);
    for (int i = 0; i < 5; i++) {
        wr(qts, 0x008, 1);
        wr(qts, 0x008, 0);
    }
    g_assert_cmphex(rd(qts, 0x008), ==, 0x602);
    wr(qts, 0x000, 0x10);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x600);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x110);

    /* START bit 1 marks the next frame the first */
    wr(qts, 0x004, 2);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10001);
    wr(qts, 0x004, 0xfffc);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10001);

    /* 0x20: bit 0 shows the clock count, bit 1 the ping-pong */
    wr(qts, 0x020, 1);
    g_assert_cmphex(rd(qts, 0x020), ==, 0);
    wr(qts, 0x020, 0);
    g_assert_cmphex(rd(qts, 0x020), ==, 0);
    wr(qts, 0x020, 2);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);

    /* The post-processor's */
    wr(qts, 0x500, 1);
    g_assert_cmphex(rd(qts, 0x500), ==, 0x011);
    wr(qts, 0x500, 2);
    g_assert_cmphex(rd(qts, 0x500), ==, 0x111);
    wr(qts, 0x504, 1);
    g_assert_cmphex(rd(qts, 0x504), ==, 0x103);
    g_assert_cmphex(rd(qts, 0x510), ==, 0x110);
    wr(qts, 0x510, 1);
    g_assert_true(qtest_get_irq(qts, PP_IRQ));
    g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x510, 3);
    g_assert_false(qtest_get_irq(qts, PP_IRQ));
    wr(qts, 0x508, 1);
    wr(qts, 0x508, 0);
    g_assert_cmphex(rd(qts, 0x508), ==, 1);
    wr(qts, 0x50c, 0x17);
    g_assert_cmphex(rd(qts, 0x50c), ==, 0x16);

    cpm_reset(qts);
    check(qts, resets, ARRAY_SIZE(resets));
    g_assert_cmphex(rd(qts, 0x508), ==, 0);
    qtest_quit(qts);
}

static void setup(QTestState *qts)
{
    cpm_reset(qts);
    wr(qts, 0x030, CUR);
    wr(qts, 0x040, CUR + YS);
    wr(qts, 0x038, REF0);
    wr(qts, 0x048, REF0 + YS);
    wr(qts, 0x050, REF1);
    wr(qts, 0x058, REF1 + YS);
    for (uint32_t r = 0x034; r <= 0x05c; r += 8) {
        wr(qts, r, W);
    }
    wr(qts, 0x02c, H << 16 | W);
    wr(qts, 0x024, 0x0c000030);
    wr(qts, 0x000, 0);
}

static void frame_data(uint8_t *f, int n)
{
    for (int i = 0; i < YS * 3 / 2; i++) {
        f[i] = (i * 7 + n * 31 + (i >> 7) * 3) & 0xff;
    }
}

static void check_buf(QTestState *qts, uint32_t addr, const uint8_t *want)
{
    g_autofree uint8_t *got = g_malloc(YS * 3 / 2);

    qtest_memread(qts, addr, got, YS * 3 / 2);
    g_assert_cmpmem(got, YS * 3 / 2, want, YS * 3 / 2);
}

/*
 * Frames from memory to memory (FUN_CNTRL bits 4 and 5), the sequence the
 * T30X ran: the first to REF0, then REF1 and REF0 in turn; START's mark
 * shifts up a nibble per frame; a soft reset points the ping-pong at REF0
 */
static void test_frames(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    static const uint32_t start[] = { 0x10010, 0x10100, 0x11000, 0x10000 };
    uint8_t f[4][YS * 3 / 2], fill[YS * 3 / 2];

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    memset(fill, 0x11, sizeof(fill));
    qtest_memwrite(qts, REF0, fill, sizeof(fill));
    memset(fill, 0x22, sizeof(fill));
    qtest_memwrite(qts, REF1, fill, sizeof(fill));
    setup(qts);
    wr(qts, 0x00c, 5);
    wr(qts, 0x004, 2);
    wr(qts, 0x010, 1);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10001);
    for (int n = 0; n < 4; n++) {
        frame_data(f[n], n);
        qtest_memwrite(qts, CUR, f[n], sizeof(f[n]));
        g_assert_cmphex(rd(qts, 0x010), ==, 0x101);
        wr(qts, 0x004, rd(qts, 0x004) | 1);
        g_assert_cmphex(rd(qts, 0x004), ==, start[n]);
        g_assert_cmphex(rd(qts, 0x008), ==, 0x002);
        g_assert_cmphex(rd(qts, 0x010), ==, 0x011);
        g_assert_true(qtest_get_irq(qts, NCU_IRQ));
        g_assert_cmphex(rd(qts, 0x020), ==, n & 1 ? 0x80000000 : 0x40000000);
        g_assert_cmphex(rd(qts, 0x01c), ==, n + 1);
        g_assert_cmphex(rd(qts, 0x00c), ==, 4);
        check_buf(qts, n & 1 ? REF1 : REF0, f[n]);
        if (n) {
            check_buf(qts, n & 1 ? REF0 : REF1, f[n - 1]);
        }
        wr(qts, 0x010, 3);
        g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    }

    /* The last went to REF1; after a soft reset the next does too */
    wr(qts, 0x000, 0);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x80000000);

    /* The clock count view: 12529 on the T30X, the frame's bytes here */
    wr(qts, 0x010, 3);
    wr(qts, 0x020, 1);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x020), ==, YS * 3 / 2);
    wr(qts, 0x020, 2);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);
    qtest_quit(qts);
}

/* A frame started before any load, or while stopped, never ends */
static void test_hang(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    setup(qts);
    wr(qts, 0x004, 2);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0);
    g_assert_cmphex(rd(qts, 0x01c), ==, 1);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x100);
    wr(qts, 0x00c, 0x15);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0);
    g_assert_cmphex(rd(qts, 0x01c), ==, 1);

    setup(qts);
    wr(qts, 0x00c, 5);
    wr(qts, 0x004, 2);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10010);
    wr(qts, 0x010, 2);
    wr(qts, 0x008, 1);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x103);
    wr(qts, 0x010, 2);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x80000000);
    g_assert_cmphex(rd(qts, 0x01c), ==, 2);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x101);
    cpm_reset(qts);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/t30-ncu/regs", test_regs);
    qtest_add_func("ingenic/t30-ncu/control", test_control);
    qtest_add_func("ingenic/t30-ncu/frames", test_frames);
    qtest_add_func("ingenic/t30-ncu/hang", test_hang);
    return g_test_run();
}
