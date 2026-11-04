/*
 * QTest testcase for the T23's and T40's ISP shadow ports
 *
 * As a T23DL and a T40XP stream: a run of (value, offset) pairs queued
 * between an open and a close, and a kick that writes its first pairs at
 * once while the MSCA is between frames, else at the end of the frame
 * going through; the run stays for the next kick. The targets are mask
 * blocks no driver had enabled.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define FRAME_NS        40000000        /* fps=25 */
#define IDLE_NS         (FRAME_NS / 16)

typedef struct ShadowPort {
    const char *machine;
    uint32_t open, port, kick, sta;
    uint32_t base;                      /* the pairs' offsets count from */
    unsigned len_shift;
    uint32_t vic;
    uint32_t a, b;                      /* two mask registers */
    uint32_t busy;                      /* kick's bit while a run waits */
    bool done_on_close;
} ShadowPort;

static const ShadowPort t23 = {
    .machine = "ingenic-t23", .open = 0xe000, .port = 0xe004,
    .kick = 0xe100, .sta = 0xe00c, .base = 0xd000, .len_shift = 4,
    .vic = 0xe0000, .a = 0xd438, .b = 0xd434, .done_on_close = true,
};

static const ShadowPort t40 = {
    .machine = "ingenic-t40", .open = 0x50320, .port = 0x50324,
    .kick = 0x50420, .sta = 0x5032c, .len_shift = 24,
    .vic = 0x80000, .a = 0x16164, .b = 0x16160, .busy = 0x100,
};

static QTestState *start(const ShadowPort *p)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    return qts;
}

/* Queue the pairs (@a <- @va, @b <- @vb), as the drivers do */
static void queue2(QTestState *qts, const ShadowPort *p, uint32_t va,
                   uint32_t vb)
{
    qtest_writel(qts, ISP + p->open, 0x101);
    qtest_writel(qts, ISP + p->port, va);
    qtest_writel(qts, ISP + p->port, p->a - p->base);
    qtest_writel(qts, ISP + p->port, vb);
    qtest_writel(qts, ISP + p->port, p->b - p->base);
}

static void kick(QTestState *qts, const ShadowPort *p, uint32_t pairs)
{
    qtest_writel(qts, ISP + p->kick, (pairs - 1) << p->len_shift | 1);
}

static void test_queue(const void *data)
{
    const ShadowPort *p = data;
    QTestState *qts = start(p);

    g_assert_cmphex(qtest_readl(qts, ISP + p->sta), ==, p->done_on_close);
    queue2(qts, p, 0x111111, 0x00100010);
    /* nothing lands before the kick; the port reads 0 */
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->port), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->open), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->sta), ==, 0x10);
    qtest_writel(qts, ISP + p->open, 0x102);
    g_assert_cmphex(qtest_readl(qts, ISP + p->sta), ==, p->done_on_close);

    /* no frames: the MSCA is idle and a kick writes at once */
    kick(qts, p, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x111111);
    g_assert_cmphex(qtest_readl(qts, ISP + p->b), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->sta), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + p->kick), ==, 0);

    /* the run stays: a direct write, then the kick writes it again */
    qtest_writel(qts, ISP + p->a, 0x222222);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x222222);
    kick(qts, p, 2);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x111111);
    g_assert_cmphex(qtest_readl(qts, ISP + p->b), ==, 0x00100010);
    g_assert_cmphex(qtest_readl(qts, ISP + p->kick), ==,
                    p->busy ? 1u << p->len_shift : 0);

    /* a pair outside a run is dropped */
    qtest_writel(qts, ISP + p->port, 0x333333);
    qtest_writel(qts, ISP + p->port, p->a - p->base);
    qtest_writel(qts, ISP + p->a, 0);
    kick(qts, p, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x111111);
    qtest_quit(qts);
}

static void test_latch(const void *data)
{
    const ShadowPort *p = data;
    QTestState *qts = start(p);

    queue2(qts, p, 0x111111, 0x00100010);
    qtest_writel(qts, ISP + p->open, 0x102);
    /* VIC 0 sends its first frame as it starts: the MSCA is between frames */
    qtest_writel(qts, ISP + p->vic, 1);
    qtest_clock_step(qts, IDLE_NS / 2);
    kick(qts, p, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x111111);

    /* mid-frame: the run waits for the frame's end */
    qtest_writel(qts, ISP + p->a, 0x222222);
    qtest_clock_step(qts, IDLE_NS);
    kick(qts, p, 2);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x222222);
    g_assert_cmphex(qtest_readl(qts, ISP + p->b), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + p->kick), ==,
                    p->busy ? p->busy | 1u << p->len_shift : 0);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x222222);
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x111111);
    g_assert_cmphex(qtest_readl(qts, ISP + p->b), ==, 0x00100010);
    g_assert_cmphex(qtest_readl(qts, ISP + p->kick), ==,
                    p->busy ? 1u << p->len_shift : 0);

    /* the VIC stopped, a run is written at once */
    qtest_writel(qts, ISP + p->a, 0);
    qtest_writel(qts, ISP + p->vic, 2);
    qtest_clock_step(qts, IDLE_NS * 2);
    kick(qts, p, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + p->a), ==, 0x111111);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("/ingenic-isp-shadow/t23/queue", &t23, test_queue);
    qtest_add_data_func("/ingenic-isp-shadow/t23/latch", &t23, test_latch);
    qtest_add_data_func("/ingenic-isp-shadow/t40/queue", &t40, test_queue);
    qtest_add_data_func("/ingenic-isp-shadow/t40/latch", &t40, test_latch);
    return g_test_run();
}
