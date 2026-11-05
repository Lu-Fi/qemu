/*
 * QTest testcase for the T32 ISP with several sensors: each VIC passes its
 * sensor's frames to the core's input cache, and the core takes one frame
 * at a time, the sensors in turn, each once the driver writes the kick
 * (0x2c bit 0). One sensor's frames go through as they come.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define ISP_KICK        (ISP + 0x2c)
#define ISP_INT_STA     (ISP + 0x30028)
#define ISP_INT_CLR     (ISP + 0x30024)
#define VIC(n)          (ISP + 0x80000 + (n) * 0x1000)
#define VIC_CTRL        0x10
#define VIC_INT_STA     0x1e0
#define VIC_INT_CLR     0x1f0
#define CPM_CLKGR0      0x10000020
#define MS              1000000
#define FRAME_NS        (40 * MS)       /* fps=25 */

#define INT_FRAME_END   (1u << 0)
#define VIC_FRAME_DONE  (1u << 0)

static QTestState *start(bool two)
{
    QTestState *qts = qtest_initf("-machine ingenic-t32 "
                                  "-device image-sensor,model=jxq03,bus=i2c0,"
                                  "fps=25,input=0 %s", two ?
                                  "-device image-sensor,model=jxq03,"
                                  "bus=i2c1,fps=25,input=1" : "");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, VIC(0) + VIC_CTRL, 1);
    if (two) {
        qtest_writel(qts, VIC(1) + VIC_CTRL, 1);
    }
    return qts;
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

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-turns/t32/turns", test_turns);
    qtest_add_func("/ingenic-isp-turns/t32/one", test_one);
    return g_test_run();
}
