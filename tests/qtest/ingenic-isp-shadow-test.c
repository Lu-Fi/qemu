/*
 * QTest testcase for the T23's and T40's ISP shadow ports and MSCA busy bits
 *
 * As a T23DL and a T40XP stream: a run of (value, offset) pairs queued
 * between an open and a close, and a kick that writes its first pairs at
 * once while the MSCA is between frames, else at the end of the frame
 * going through; the run stays for the next kick. The targets are mask
 * blocks no driver had enabled.
 *
 * The T32's port, as a T32LQ runs it: the T41's RAM of pairs, 256 of them,
 * written out at the frame's end or at once between frames; a pair into
 * the RAM clears the last value written out.
 *
 * The MSCA's busy word, a bit a channel, read-only: the enabled channels'
 * bits set as a frame reaches the MSCA, and a channel's clears as it puts
 * the frame out into a buffer; without one it stays set.
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
    uint32_t ch_en, ch, msca_busy;      /* the MSCA: its channels at ch */
    uint32_t size, y_push, uv_push, y_stride, uv_stride, done_y;
} ShadowPort;

static const ShadowPort t23 = {
    .machine = "ingenic-t23", .open = 0xe000, .port = 0xe004,
    .kick = 0xe100, .sta = 0xe00c, .base = 0xd000, .len_shift = 4,
    .vic = 0xe0000, .a = 0xd438, .b = 0xd434, .done_on_close = true,
    .ch_en = 0xd040, .ch = 0xd100, .msca_busy = 0xd054, .size = 0x04,
    .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40, .uv_stride = 0x60,
    .done_y = 0x38,
};

static const ShadowPort t40 = {
    .machine = "ingenic-t40", .open = 0x50320, .port = 0x50324,
    .kick = 0x50420, .sta = 0x5032c, .len_shift = 24,
    .vic = 0x80000, .a = 0x16164, .b = 0x16160, .busy = 0x100,
    .ch_en = 0x16008, .ch = 0x16100, .msca_busy = 0x160e0, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74,
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

#define BUF     0x00400000

/* Give channel @n a 64x64 frame's buffer */
static void push(QTestState *qts, const ShadowPort *p, unsigned n)
{
    uint32_t c = ISP + p->ch + n * 0x100;

    qtest_writel(qts, c + p->size, 64 << 16 | 64);
    qtest_writel(qts, c + p->y_stride, 64);
    qtest_writel(qts, c + p->uv_stride, 64);
    qtest_writel(qts, c + p->y_push, BUF + n * 0x2000);
    qtest_writel(qts, c + p->uv_push, BUF + n * 0x2000 + 0x1000);
}

static uint32_t busy(QTestState *qts, const ShadowPort *p)
{
    return qtest_readl(qts, ISP + p->msca_busy);
}

static void test_busy(const void *data)
{
    const ShadowPort *p = data;
    QTestState *qts = start(p);

    /* channel 1 has a buffer, 2 none */
    qtest_writel(qts, ISP + p->ch_en, 6);
    push(qts, p, 1);
    g_assert_cmphex(busy(qts, p), ==, 0);

    /* the first frame, as the VIC starts: then the next reaches the MSCA */
    qtest_writel(qts, ISP + p->vic, 1);
    qtest_clock_step(qts, IDLE_NS / 2);
    g_assert_cmphex(busy(qts, p), ==, 0);
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(busy(qts, p), ==, 6);

    /* read-only */
    qtest_writel(qts, ISP + p->msca_busy, 0);
    g_assert_cmphex(busy(qts, p), ==, 6);
    qtest_writel(qts, ISP + p->msca_busy, 0xffffffff);
    g_assert_cmphex(busy(qts, p), ==, 6);

    /* the frame's end: channel 1 puts it out, 2 stays busy */
    qtest_readl(qts, ISP + p->ch + 0x100 + p->done_y);
    push(qts, p, 1);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2);
    g_assert_cmphex(busy(qts, p), ==, 6);
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(busy(qts, p), ==, 4);
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(busy(qts, p), ==, 6);

    /*
     * channel 2 disabled mid-frame, 1 without a buffer: both stay busy
     * through the frame's end, and the next frame sets 1 alone
     */
    qtest_writel(qts, ISP + p->ch_en, 2);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS);
    g_assert_cmphex(busy(qts, p), ==, 6);
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(busy(qts, p), ==, 2);

    /* three channels */
    qtest_writel(qts, ISP + p->ch_en, 7);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(busy(qts, p), ==, 7);

    /* the VIC stopped: no frame reaches the MSCA */
    qtest_readl(qts, ISP + p->ch + 0x100 + p->done_y);
    push(qts, p, 1);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2);
    g_assert_cmphex(busy(qts, p), ==, 7);
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(busy(qts, p), ==, 5);
    qtest_writel(qts, ISP + p->vic, 2);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(busy(qts, p), ==, 5);
    qtest_quit(qts);
}

#define T32_OPEN        0xf8000
#define T32_PAIR        0xf8004
#define T32_LAST        0xf8008
#define T32_STATE       0xf800c
#define T32_RUN         0xf8100
#define T32_A           0xf04b8         /* channel 2's mask words */
#define T32_B           0xf04b4

static void t32_wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, ISP + off, v);
}

static uint32_t t32_rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, ISP + off);
}

static void t32_pair(QTestState *qts, uint32_t v, uint32_t off)
{
    t32_wr(qts, T32_PAIR, v);
    t32_wr(qts, T32_PAIR, off);
}

static void test_t32_ram(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 -device image-sensor,"
                                 "model=jxq03,bus=i2c0,fps=25");

    qtest_writel(qts, CPM_CLKGR0, 0);
    t32_wr(qts, 0x80010, 1);                /* its VIC: a frame at once */
    qtest_clock_step(qts, IDLE_NS * 2);

    /* mid-frame: queued, and written out at the frame's end */
    t32_wr(qts, T32_OPEN, 0x101);
    g_assert_cmphex(t32_rd(qts, T32_STATE), ==, 0x10);
    t32_pair(qts, 0x11, T32_A);
    t32_pair(qts, 0x22, T32_B);
    t32_wr(qts, T32_OPEN, 0x1010102);
    g_assert_cmphex(t32_rd(qts, T32_STATE), ==, 1);
    t32_pair(qts, 0x99, T32_B);             /* closed: dropped */
    t32_wr(qts, T32_RUN, 0x11);
    g_assert_cmphex(t32_rd(qts, T32_RUN), ==, 0);
    g_assert_cmphex(t32_rd(qts, T32_A), ==, 0);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2 - 1000000);
    g_assert_cmphex(t32_rd(qts, T32_A), ==, 0);
    qtest_clock_step(qts, 2000000);
    g_assert_cmphex(t32_rd(qts, T32_A), ==, 0x11);
    g_assert_cmphex(t32_rd(qts, T32_B), ==, 0x22);
    g_assert_cmphex(t32_rd(qts, T32_LAST), ==, 0x22);

    /* between frames: at once, and again */
    t32_wr(qts, T32_A, 0);
    t32_wr(qts, T32_RUN, 0x01);
    g_assert_cmphex(t32_rd(qts, T32_A), ==, 0x11);
    g_assert_cmphex(t32_rd(qts, T32_LAST), ==, 0x11);

    /* a pair into the RAM clears the last value; 257 go round to entry 0 */
    t32_wr(qts, T32_OPEN, 0x101);
    t32_wr(qts, T32_PAIR, 1);
    g_assert_cmphex(t32_rd(qts, T32_LAST), ==, 0x11);
    t32_wr(qts, T32_PAIR, T32_A);
    g_assert_cmphex(t32_rd(qts, T32_LAST), ==, 0);
    for (uint32_t i = 2; i <= 257; i++) {
        t32_pair(qts, i, T32_A);
    }
    t32_wr(qts, T32_OPEN, 0x1010102);
    qtest_clock_step(qts, IDLE_NS * 2);
    t32_wr(qts, T32_RUN, 0x11);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(t32_rd(qts, T32_A), ==, 2);
    t32_wr(qts, T32_RUN, 0x01);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(t32_rd(qts, T32_A), ==, 257);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("/ingenic-isp-shadow/t23/queue", &t23, test_queue);
    qtest_add_data_func("/ingenic-isp-shadow/t23/latch", &t23, test_latch);
    qtest_add_data_func("/ingenic-isp-shadow/t40/queue", &t40, test_queue);
    qtest_add_data_func("/ingenic-isp-shadow/t40/latch", &t40, test_latch);
    qtest_add_data_func("/ingenic-isp-shadow/t23/busy", &t23, test_busy);
    qtest_add_data_func("/ingenic-isp-shadow/t40/busy", &t40, test_busy);
    qtest_add_func("/ingenic-isp-shadow/t32/ram", test_t32_ram);
    return g_test_run();
}
