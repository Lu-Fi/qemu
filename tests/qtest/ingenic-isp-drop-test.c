/*
 * QTest testcase for the MSCA's frame drop
 *
 * Each channel has a window length - 1 (bits 4:0) and a mask: frame i of
 * the window goes out if bit i of the mask is set, else the channel drops
 * it, with no buffer taken, no done entry and its busy bit left set. As a
 * T23DL, T31X and T40XP stream: a write takes effect from the frame after
 * the one in flight (the drop is decided as a frame reaches the MSCA), a
 * window cut short ends at the frame it is at, and on the T31 alone a
 * write of the mask restarts the window. The mask resets to 1 on the T23,
 * T30 and T31, to all ones on the T40 (and T41NQ), the length to 0.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define FRAME_NS        40000000        /* fps=25 */
#define IDLE_NS         (FRAME_NS / 16) /* the next frame reaches the MSCA */
#define T41_FIRST_NS    135000000       /* a T41NQ's first frame */
#define BUF             0x00400000
#define T30_MSCALER     0x130b0100      /* its channel 0 */

typedef struct IspPart {
    const char *machine;
    uint32_t ch_en, ch;                 /* its channels at ch + n * 0x100 */
    uint32_t size, y_push, uv_push, y_stride, uv_stride, done_y, done_sta;
    uint32_t vic, vic_size;             /* T41: the VIC's frame size */
    uint32_t latch;                     /* T41: the MSCA's latch trigger */
    uint32_t drop, mark;                /* frame drop at ch + drop, reset */
    uint32_t busy;                      /* the MSCA's busy word, if any */
    uint32_t pipe1;                     /* a second pipeline's channels */
    bool rephase;
} IspPart;

static const IspPart t23 = {
    .machine = "ingenic-t23", .ch_en = 0xd040, .ch = 0xd100, .size = 0x04,
    .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40, .uv_stride = 0x60,
    .done_y = 0x38, .done_sta = 0x3c, .vic = 0xe0000,
    .drop = 0x20, .mark = 1, .busy = 0xd054,
};

static const IspPart t31 = {
    .machine = "ingenic-t31", .ch_en = 0x9804, .ch = 0x9900, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74, .done_sta = 0x7c, .vic = 0xe0000,
    .drop = 0x30, .mark = 1, .rephase = true,
};

static const IspPart t40 = {
    .machine = "ingenic-t40", .ch_en = 0x16008, .ch = 0x16100, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74, .done_sta = 0x7c, .vic = 0x80000,
    .drop = 0x30, .mark = UINT32_MAX, .busy = 0x160e0, .pipe1 = 0x17100,
};

static const IspPart t41 = {
    .machine = "ingenic-t41", .ch_en = 0xf0008, .ch = 0xf0100, .size = 0x00,
    .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80, .uv_stride = 0x98,
    .done_y = 0x74, .done_sta = 0x7c, .vic = 0x80000, .vic_size = 0x80,
    .latch = 0xf0010,
    .drop = 0x30, .mark = UINT32_MAX,
};

static const IspPart t32 = {
    .machine = "ingenic-t32", .ch = 0xf0100, .drop = 0x20,
    .mark = UINT32_MAX,
};

static QTestState *start(const IspPart *p)
{
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    return qts;
}

static uint32_t chan(const IspPart *p, unsigned n)
{
    return ISP + p->ch + n * 0x100;
}

static void set_drop(QTestState *qts, const IspPart *p, unsigned n,
                     uint32_t lsize, uint32_t mark)
{
    qtest_writel(qts, chan(p, n) + p->drop, lsize);
    qtest_writel(qts, chan(p, n) + p->drop + 4, mark);
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
    if (p->latch) {
        qtest_writel(qts, ISP + p->latch, 1);
    }
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

/* Channels 0 and 1 streaming, their first frame out */
static QTestState *stream(const IspPart *p)
{
    QTestState *qts = start(p);

    qtest_writel(qts, ISP + p->ch_en, 3);
    push(qts, p, 0);
    push(qts, p, 1);
    if (p->vic_size) {
        qtest_writel(qts, ISP + p->vic_size, 64 << 16 | 64);
    }
    qtest_writel(qts, ISP + p->vic, 1);
    if (p->vic_size) {
        qtest_clock_step(qts, T41_FIRST_NS);
    }
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));
    return qts;
}

/* The next @n frames, as a string of channel 1's: 1 out, 0 dropped */
static void frames(QTestState *qts, const IspPart *p, unsigned n,
                   const char *want)
{
    char got[33];

    g_assert_cmpuint(n, <, sizeof(got));
    for (unsigned i = 0; i < n; i++) {
        qtest_clock_step(qts, FRAME_NS);
        g_assert_true(took(qts, p, 0));
        got[i] = took(qts, p, 1) ? '1' : '0';
    }
    got[n] = 0;
    g_assert_cmpstr(got, ==, want);
}

static void test_reset(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = start(p);

    for (unsigned k = 0; k < (p->pipe1 ? 2 : 1); k++) {
        uint32_t ch = ISP + (k ? p->pipe1 : p->ch) + p->drop;

        for (unsigned n = 0; n < 3; n++) {
            g_assert_cmphex(qtest_readl(qts, ch + n * 0x100), ==, 0);
            g_assert_cmphex(qtest_readl(qts, ch + n * 0x100 + 4), ==,
                            p->mark);
        }
    }
    qtest_writel(qts, chan(p, 2) + p->drop, UINT32_MAX);
    qtest_writel(qts, chan(p, 2) + p->drop + 4, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, chan(p, 2) + p->drop), ==, 0x1f);
    g_assert_cmphex(qtest_readl(qts, chan(p, 2) + p->drop + 4), ==,
                    UINT32_MAX);
    qtest_writel(qts, chan(p, 2) + p->drop + 4, 0);
    g_assert_cmphex(qtest_readl(qts, chan(p, 2) + p->drop + 4), ==, 0);
    qtest_quit(qts);
}

static void test_rule(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = stream(p);

    /* the reset values let every frame out */
    frames(qts, p, 4, "1111");

    /* written between frames: from the next frame, bit 0 first */
    set_drop(qts, p, 1, 3, 0x3);
    frames(qts, p, 8, "11001100");
    set_drop(qts, p, 1, 0, 0);
    frames(qts, p, 3, "000");
    set_drop(qts, p, 1, 2, 0x4);
    frames(qts, p, 6, "001001");
    set_drop(qts, p, 1, 0, 1);
    frames(qts, p, 2, "11");
    qtest_quit(qts);
}

/* As a T23DL, T31X and T40XP time it */
static void test_timing(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = stream(p);

    /* mid-frame: the frame in flight keeps the old mask */
    qtest_clock_step(qts, IDLE_NS * 2);
    set_drop(qts, p, 1, 0, 0);
    frames(qts, p, 3, "100");
    qtest_clock_step(qts, IDLE_NS * 2);
    qtest_writel(qts, chan(p, 1) + p->drop + 4, 1);
    frames(qts, p, 2, "01");

    /* 1 of 4, then the mask written again with frame 1 in flight */
    set_drop(qts, p, 1, 3, 1);
    frames(qts, p, 6, "110001");
    qtest_clock_step(qts, IDLE_NS * 2);
    qtest_writel(qts, chan(p, 1) + p->drop + 4, 1);
    frames(qts, p, 6, p->rephase ? "010001" : "000100");

    /* the window cut short to 2 with frame 1 in flight: 2 is its last */
    if (!p->rephase) {
        frames(qts, p, 2, "01");
    }
    qtest_writel(qts, chan(p, 1) + p->drop, 1);
    frames(qts, p, 5, "00101");
    qtest_quit(qts);
}

/* A dropped frame takes no buffer and leaves the channel busy */
static void test_busy(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = stream(p);

    set_drop(qts, p, 1, 1, 1);
    frames(qts, p, 2, "10");
    qtest_clock_step(qts, IDLE_NS * 2);
    g_assert_cmphex(qtest_readl(qts, ISP + p->busy), ==, 3);
    qtest_clock_step(qts, FRAME_NS - IDLE_NS * 2);
    g_assert_cmphex(qtest_readl(qts, ISP + p->busy), ==, 0);
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, ISP + p->busy), ==, 2);
    g_assert_true(took(qts, p, 0));
    g_assert_false(took(qts, p, 1));
    /* the buffer it was given is still queued for the next frame out */
    qtest_clock_step(qts, FRAME_NS);
    g_assert_true(took(qts, p, 0));
    g_assert_true(took(qts, p, 1));
    qtest_quit(qts);
}

/* The T30's multi-scaler, its own block: the T31's registers */
static void test_t30_reset(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    for (unsigned n = 0; n < 3; n++) {
        g_assert_cmphex(qtest_readl(qts, T30_MSCALER + n * 0x100 + 0x30), ==,
                        0);
        g_assert_cmphex(qtest_readl(qts, T30_MSCALER + n * 0x100 + 0x34), ==,
                        1);
    }
    qtest_writel(qts, T30_MSCALER + 0x30, UINT32_MAX);
    g_assert_cmphex(qtest_readl(qts, T30_MSCALER + 0x30), ==, 0x1f);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("/ingenic-isp-drop/t23/reset", &t23, test_reset);
    qtest_add_data_func("/ingenic-isp-drop/t31/reset", &t31, test_reset);
    qtest_add_data_func("/ingenic-isp-drop/t32/reset", &t32, test_reset);
    qtest_add_data_func("/ingenic-isp-drop/t40/reset", &t40, test_reset);
    qtest_add_data_func("/ingenic-isp-drop/t41/reset", &t41, test_reset);
    qtest_add_func("/ingenic-isp-drop/t30/reset", test_t30_reset);
    qtest_add_data_func("/ingenic-isp-drop/t23/rule", &t23, test_rule);
    qtest_add_data_func("/ingenic-isp-drop/t31/rule", &t31, test_rule);
    qtest_add_data_func("/ingenic-isp-drop/t40/rule", &t40, test_rule);
    qtest_add_data_func("/ingenic-isp-drop/t41/rule", &t41, test_rule);
    qtest_add_data_func("/ingenic-isp-drop/t23/timing", &t23, test_timing);
    qtest_add_data_func("/ingenic-isp-drop/t31/timing", &t31, test_timing);
    qtest_add_data_func("/ingenic-isp-drop/t40/timing", &t40, test_timing);
    qtest_add_data_func("/ingenic-isp-drop/t23/busy", &t23, test_busy);
    qtest_add_data_func("/ingenic-isp-drop/t40/busy", &t40, test_busy);
    return g_test_run();
}
