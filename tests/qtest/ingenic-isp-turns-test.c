/*
 * QTest testcase for the T32 ISP with several sensors: each VIC passes its
 * sensor's frames to the core's input cache, and the core takes one frame
 * at a time, the sensors in turn, each once the driver writes the kick
 * (0x2c bit 0; on a T33, its top register list, 33, run at 0x22014 bit 1;
 * on a T32 Pro, list 34 at 0x1c014 bit 2, its frames ending with the LCE
 * statistics bit 8 too, 0x1304 bits 11:8 naming the input it takes next).
 * One sensor's frames go through as they come.
 *
 * A register list's done bit rises as the list runs: lists 0-31 at 0x300c8,
 * 32-63 at 0x300d8, cleared by a write of ones at 0x300c4 and 0x300d4 (the
 * T32 has the second word alone).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define ISP_KICK        (ISP + 0x2c)
#define ISP_TOP_LIST    (ISP + 0x22014)    /* T33: list 33, bit 1 */
#define PRO_TOP_LIST    (ISP + 0x1c014)    /* T32 Pro: list 34, bit 2 */
#define PRO_NEXT        (ISP + 0x1304)     /* bits 11:8: the next input */
#define INT_LCE         (1u << 8)
#define ISP_INT_STA     (ISP + 0x30028)
#define ISP_INT_CLR     (ISP + 0x30024)
#define LIST_EN(k)      (ISP + 0x300c0 + (k) * 0x10)
#define LIST_CLR(k)     (ISP + 0x300c4 + (k) * 0x10)
#define LIST_DONE(k)    (ISP + 0x300c8 + (k) * 0x10)
#define VIC(n)          (ISP + 0x80000 + (n) * 0x1000)
#define VIC_CTRL        0x10
#define VIC_INT_STA     0x1e0
#define VIC_INT_CLR     0x1f0
#define CPM_CLKGR0      0x10000020
#define MS              1000000
#define FRAME_NS        (40 * MS)       /* fps=25 */

#define INT_FRAME_END   (1u << 0)
#define VIC_FRAME_DONE  (1u << 0)

static QTestState *start_on(const char *variant, bool two)
{
    QTestState *qts = qtest_initf("-machine ingenic-t32 "
                                  "-global ingenic-t31.soc-variant=%s "
                                  "-device image-sensor,model=jxq03,bus=i2c0,"
                                  "fps=25,input=0 %s", variant, two ?
                                  "-device image-sensor,model=jxq03,"
                                  "bus=i2c1,fps=25,input=1" : "");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, VIC(0) + VIC_CTRL, 1);
    if (two) {
        qtest_writel(qts, VIC(1) + VIC_CTRL, 1);
    }
    return qts;
}

static QTestState *start(bool two)
{
    return start_on("t32nq", two);
}

/* The core's frame ends since the last call */
static bool core_frame(QTestState *qts)
{
    uint32_t sta = qtest_readl(qts, ISP_INT_STA);

    qtest_writel(qts, ISP_INT_CLR, sta);
    return sta & INT_FRAME_END;
}

static bool vic_frame(QTestState *qts, int n)
{
    uint32_t sta = qtest_readl(qts, VIC(n) + VIC_INT_STA);

    qtest_writel(qts, VIC(n) + VIC_INT_CLR, sta);
    return sta & VIC_FRAME_DONE;
}

static void test_turns(void)
{
    QTestState *qts = start(true);

    /* both VICs pass frames; the core waits for the kick */
    qtest_clock_step(qts, 3 * FRAME_NS);
    g_assert_true(vic_frame(qts, 0));
    g_assert_true(vic_frame(qts, 1));
    core_frame(qts);
    qtest_clock_step(qts, 3 * FRAME_NS);
    g_assert_false(core_frame(qts));

    /* one frame per kick, whichever sensor's turn it is */
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, ISP_KICK, 1);
        qtest_clock_step(qts, MS);
        g_assert_true(core_frame(qts));
        qtest_clock_step(qts, 2 * FRAME_NS);
        g_assert_false(core_frame(qts));
    }
    qtest_quit(qts);
}

static void test_t33(void)
{
    QTestState *qts = start_on("t33n", true);

    qtest_clock_step(qts, 3 * FRAME_NS);
    vic_frame(qts, 0);
    vic_frame(qts, 1);
    core_frame(qts);

    /* 0x2c starts nothing on a T33 */
    qtest_writel(qts, ISP_KICK, 1);
    qtest_clock_step(qts, 3 * FRAME_NS);
    g_assert_false(core_frame(qts));

    for (int i = 0; i < 4; i++) {
        /* the top list's run starts a frame */
        qtest_writel(qts, ISP_TOP_LIST, 2);
        qtest_clock_step(qts, MS);
        g_assert_true(core_frame(qts));
        qtest_clock_step(qts, 3 * FRAME_NS);
        g_assert_false(core_frame(qts));
    }
    qtest_quit(qts);
}

static void test_t32pro(void)
{
    QTestState *qts = start_on("t32prozn", true);

    qtest_clock_step(qts, 3 * FRAME_NS);
    vic_frame(qts, 0);
    vic_frame(qts, 1);
    core_frame(qts);

    qtest_writel(qts, ISP_KICK, 1);
    qtest_clock_step(qts, 3 * FRAME_NS);
    g_assert_false(core_frame(qts));

    for (int i = 0; i < 4; i++) {
        uint32_t sta;

        g_assert_cmpuint(qtest_readl(qts, PRO_NEXT) >> 8 & 0xf, ==, i % 2);
        qtest_writel(qts, PRO_TOP_LIST, 4);
        qtest_clock_step(qts, MS);
        sta = qtest_readl(qts, ISP_INT_STA);
        g_assert_cmphex(sta & (INT_FRAME_END | INT_LCE), ==,
                        INT_FRAME_END | INT_LCE);
        qtest_writel(qts, ISP_INT_CLR, sta);
        qtest_clock_step(qts, 3 * FRAME_NS);
        g_assert_false(core_frame(qts));
    }
    qtest_quit(qts);
}

static void test_one(void)
{
    QTestState *qts = start(false);

    /* one sensor: no turns, every frame goes through */
    qtest_clock_step(qts, 3 * FRAME_NS);
    core_frame(qts);
    qtest_clock_step(qts, FRAME_NS + MS);
    g_assert_true(core_frame(qts));
    qtest_quit(qts);
}

static void test_lists(void)
{
    QTestState *qts = start_on("t32prozn", false);

    qtest_writel(qts, LIST_EN(0), ~0u);
    qtest_writel(qts, LIST_EN(1), ~0u);
    qtest_writel(qts, ISP + 0x1c010, 0x5);
    qtest_writel(qts, PRO_TOP_LIST, 0x4);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(0)), ==, 0x5);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(1)), ==, 0x4);

    /* a write to the word itself changes nothing */
    qtest_writel(qts, LIST_DONE(0), 0);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(0)), ==, 0x5);
    qtest_writel(qts, LIST_CLR(0), 0x1);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(0)), ==, 0x4);
    qtest_writel(qts, LIST_CLR(1), ~0u);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(1)), ==, 0);
    qtest_quit(qts);

    /* the T32: list 48 sets bit 16; lists 0-31 have no word */
    qts = start(false);
    qtest_writel(qts, LIST_EN(1), ~0u);
    qtest_writel(qts, ISP + 0x22010, 0x1);
    qtest_writel(qts, ISP + 0x22014, 0x10000);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(0)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, LIST_DONE(1)), ==, 0x10000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-turns/t32/turns", test_turns);
    qtest_add_func("/ingenic-isp-turns/t32/one", test_one);
    qtest_add_func("/ingenic-isp-turns/t33/turns", test_t33);
    qtest_add_func("/ingenic-isp-turns/t32pro/turns", test_t32pro);
    qtest_add_func("/ingenic-isp-turns/lists", test_lists);
    return g_test_run();
}
