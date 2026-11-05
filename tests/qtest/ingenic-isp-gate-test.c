/*
 * QTest testcase for the ISP's clock gate and the MSCA's channel latch
 *
 * As a T21N, T23DL, T31X and T40XP read it: with CLKGR0 bit 23 set every
 * ISP word reads the last value read before, writes are lost, and the VIC
 * stands still; with the clock back its frame goes on for the time it had
 * left and goes out, and the VIC's status takes bits 4 and 9 (the T21's
 * bit 3).
 *
 * As a T23DL, T31X and T40XP stream: a frame goes out on the channels
 * enabled as it reached the MSCA, so a channel disabled mid-frame still
 * puts it out and one enabled mid-frame waits for the next frame.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define ISP_GATE        (1u << 23)
#define FRAME_NS        40000000        /* fps=25 */
#define IDLE_NS         (FRAME_NS / 16) /* the next frame reaches the MSCA */
#define MS              1000000
#define BUF             0x00400000

typedef struct IspPart {
    const char *machine;
    uint32_t ch_en, ch;                 /* its channels at ch + n * 0x100 */
    uint32_t size, y_push, uv_push, y_stride, uv_stride, done_y, done_sta;
    uint32_t vic, vic_sta;
    uint32_t vic_ungate;
} IspPart;

static const IspPart t21 = {
    .machine = "ingenic-t21", .ch_en = 0x2304, .ch = 0x2400, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74, .done_sta = 0x7c, .vic = 0x10000, .vic_sta = 0x10100,
    .vic_ungate = 1u << 3,
};

static const IspPart t23 = {
    .machine = "ingenic-t23", .ch_en = 0xd040, .ch = 0xd100, .size = 0x04,
    .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40, .uv_stride = 0x60,
    .done_y = 0x38, .done_sta = 0x3c, .vic = 0xe0000, .vic_sta = 0xe01e0,
    .vic_ungate = 0x210,
};

static const IspPart t31 = {
    .machine = "ingenic-t31", .ch_en = 0x9804, .ch = 0x9900, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74, .done_sta = 0x7c, .vic = 0xe0000, .vic_sta = 0xe01e0,
    .vic_ungate = 0x210,
};

static const IspPart t40 = {
    .machine = "ingenic-t40", .ch_en = 0x16008, .ch = 0x16100, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74, .done_sta = 0x7c, .vic = 0x80000, .vic_sta = 0x801e0,
    .vic_ungate = 0x210,
};

static QTestState *start(const IspPart *p)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    return qts;
}

static uint32_t chan(const IspPart *p, unsigned n)
{
    return ISP + p->ch + n * 0x100;
}

/* Give channel @n a 64x64 frame's buffer */
static void push(QTestState *qts, const IspPart *p, unsigned n)
{
    uint32_t c = chan(p, n);

    qtest_writel(qts, c + p->size, 64 << 16 | 64);
    qtest_writel(qts, c + p->y_stride, 64);
    qtest_writel(qts, c + p->uv_stride, 64);
    qtest_writel(qts, c + p->y_push, BUF + n * 0x2000);
    qtest_writel(qts, c + p->uv_push, BUF + n * 0x2000 + 0x1000);
}

/* Whether channel @n put a frame out; if so, drain it and give it back */
static bool took(QTestState *qts, const IspPart *p, unsigned n)
{
    if (qtest_readl(qts, chan(p, n) + p->done_sta) & 1) {
        return false;
    }
    qtest_readl(qts, chan(p, n) + p->done_y);
    push(qts, p, n);
    return true;
}

static void test_gate(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = start(p);
    uint32_t sta;

    qtest_writel(qts, ISP + p->ch_en, 2);
    push(qts, p, 1);
    qtest_writel(qts, ISP + p->vic, 1);     /* its first frame, at once */
    g_assert_true(took(qts, p, 1));

    /* gated mid-frame: the last value read, whatever the word */
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_cmphex(qtest_readl(qts, chan(p, 1) + p->size), ==,
                    64 << 16 | 64);
    qtest_writel(qts, CPM_CLKGR0, ISP_GATE);
    g_assert_cmphex(qtest_readl(qts, ISP + p->ch_en), ==, 64 << 16 | 64);
    g_assert_cmphex(qtest_readl(qts, ISP + p->vic_sta), ==, 64 << 16 | 64);
    g_assert_cmphex(qtest_readl(qts, chan(p, 1) + p->done_sta), ==,
                    64 << 16 | 64);
    /* writes are lost, for good */
    qtest_writel(qts, ISP + p->ch_en, 0);
    qtest_writel(qts, chan(p, 2) + p->size, 0x01000100);
    g_assert_cmphex(qtest_readl(qts, ISP + p->ch_en), ==, 64 << 16 | 64);
    /* and no frame comes */
    qtest_clock_step(qts, FRAME_NS * 2);

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->ch_en), ==, 2);
    g_assert_cmphex(qtest_readl(qts, chan(p, 2) + p->size), ==, 0);
    sta = qtest_readl(qts, ISP + p->vic_sta);
    g_assert_cmphex(sta & p->vic_ungate, ==, p->vic_ungate);
    g_assert_false(took(qts, p, 1));

    /* the frame goes on for the half it had left, and goes out */
    qtest_clock_step(qts, FRAME_NS / 2 - MS);
    g_assert_false(took(qts, p, 1));
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(took(qts, p, 1));
    qtest_clock_step(qts, FRAME_NS - 2 * MS);
    g_assert_false(took(qts, p, 1));
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(took(qts, p, 1));

    /* gated while stopped: the VIC stays stopped and its status as it was */
    qtest_writel(qts, ISP + p->vic, 2);
    sta = qtest_readl(qts, ISP + p->vic_sta);
    qtest_writel(qts, CPM_CLKGR0, ISP_GATE);
    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->vic_sta), ==, sta);
    qtest_clock_step(qts, FRAME_NS * 2);
    g_assert_false(took(qts, p, 1));
    qtest_quit(qts);
}

static void test_latch(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = start(p);

    qtest_writel(qts, ISP + p->ch_en, 3);
    push(qts, p, 0);
    push(qts, p, 1);
    qtest_writel(qts, ISP + p->vic, 1);
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));

    /* channel 1 disabled after the next frame reached the MSCA: still out */
    qtest_clock_step(qts, IDLE_NS * 2);
    qtest_writel(qts, ISP + p->ch_en, 1);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2);
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));

    /* the frame after, it was off as the frame came */
    qtest_clock_step(qts, FRAME_NS);
    g_assert_true(took(qts, p, 0));
    g_assert_false(took(qts, p, 1));

    /* enabled mid-frame: the next frame */
    qtest_clock_step(qts, IDLE_NS * 2);
    qtest_writel(qts, ISP + p->ch_en, 3);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2);
    g_assert_true(took(qts, p, 0));
    g_assert_false(took(qts, p, 1));
    qtest_clock_step(qts, FRAME_NS);
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));

    /* enabled before the frame reached the MSCA: that frame */
    qtest_writel(qts, ISP + p->ch_en, 1);
    qtest_clock_step(qts, FRAME_NS);
    took(qts, p, 1);
    qtest_clock_step(qts, IDLE_NS / 2);
    qtest_writel(qts, ISP + p->ch_en, 3);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS / 2);
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("/ingenic-isp-gate/t21/gate", &t21, test_gate);
    qtest_add_data_func("/ingenic-isp-gate/t23/gate", &t23, test_gate);
    qtest_add_data_func("/ingenic-isp-gate/t31/gate", &t31, test_gate);
    qtest_add_data_func("/ingenic-isp-gate/t40/gate", &t40, test_gate);
    qtest_add_data_func("/ingenic-isp-gate/t23/latch", &t23, test_latch);
    qtest_add_data_func("/ingenic-isp-gate/t31/latch", &t31, test_latch);
    qtest_add_data_func("/ingenic-isp-gate/t40/latch", &t40, test_latch);
    return g_test_run();
}
