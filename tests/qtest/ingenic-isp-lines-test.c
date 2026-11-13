/*
 * QTest testcase for channel 0's line counter (T31 0x9888, T40 0x1675c and
 * 0x167f4)
 *
 * As a T31X and T40XP read it: line << 16 | pixel of channel 0's frame,
 * from 0 as the frame reaches the MSCA up through its height while it puts
 * the frame out. After it the T31's reads the height (pixel 0) and the
 * T40's 0, until the next frame channel 0 takes; one it drops (frame drop
 * or no buffer) leaves it so. A T31X takes a buffer pushed before the
 * frame reaches the MSCA; one pushed after waits for the next frame.
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
#define IDLE_NS         (FRAME_NS / 16) /* the next frame reaches the MSCA */
#define MS              1000000
#define BUF             0x00400000

typedef struct IspPart {
    const char *machine;
    uint32_t ch_en, ch, vic;
    uint32_t lines[2];
    bool holds;
} IspPart;

static const IspPart t31 = {
    .machine = "ingenic-t31", .ch_en = 0x9804, .ch = 0x9900, .vic = 0xe0000,
    .lines = { 0x9888 }, .holds = true,
};

static const IspPart t40 = {
    .machine = "ingenic-t40", .ch_en = 0x16008, .ch = 0x16100,
    .vic = 0x80000, .lines = { 0x1675c, 0x167f4 },
};

static void push(QTestState *qts, const IspPart *p)
{
    uint32_t c = ISP + p->ch;

    qtest_writel(qts, c, 64 << 16 | 64);
    qtest_writel(qts, c + 0x80, 64);
    qtest_writel(qts, c + 0x98, 64);
    qtest_writel(qts, c + 0x6c, BUF);
    qtest_writel(qts, c + 0x84, BUF + 0x1000);
}

static bool took(QTestState *qts, const IspPart *p)
{
    if (qtest_readl(qts, ISP + p->ch + 0x7c) & 1) {
        return false;
    }
    qtest_readl(qts, ISP + p->ch + 0x74);
    return true;
}

static uint32_t lines(QTestState *qts, const IspPart *p)
{
    uint32_t v = qtest_readl(qts, ISP + p->lines[0]);

    if (p->lines[1]) {
        g_assert_cmphex(qtest_readl(qts, ISP + p->lines[1]), ==, v);
    }
    return v;
}

static void test_lines(const void *data)
{
    const IspPart *p = data;
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);
    uint32_t rest = p->holds ? 64 << 16 : 0, a, b;

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(lines(qts, p), ==, 0);
    qtest_writel(qts, ISP + p->ch_en, 1);
    push(qts, p);
    qtest_writel(qts, ISP + p->vic, 1);
    g_assert_true(took(qts, p));
    g_assert_cmphex(lines(qts, p), ==, rest);
    push(qts, p);

    /* from 0 as the next frame reaches the MSCA, up through the frame */
    qtest_clock_step(qts, IDLE_NS - MS);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, MS);
    g_assert_cmphex(lines(qts, p), ==, 0);
    qtest_clock_step(qts, (FRAME_NS - IDLE_NS) / 2);
    a = lines(qts, p);
    g_assert_cmpuint(a >> 16, >=, 31);
    g_assert_cmpuint(a >> 16, <=, 32);
    g_assert_cmpuint(a & 0xffff, <, 64);
    qtest_clock_step(qts, (FRAME_NS - IDLE_NS) / 2 - MS);
    b = lines(qts, p);
    g_assert_cmpuint(b >> 16, >, 60);
    g_assert_cmpuint(b >> 16, <, 64);
    qtest_clock_step(qts, MS);
    g_assert_true(took(qts, p));
    g_assert_cmphex(lines(qts, p), ==, rest);

    /* a frame it has no buffer for: left as it was */
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_false(took(qts, p));

    /* nor one it drops */
    push(qts, p);
    qtest_writel(qts, ISP + p->ch + 0x30, 1);
    qtest_writel(qts, ISP + p->ch + 0x34, 2);
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_false(took(qts, p));
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_cmphex(lines(qts, p) >> 16, >=, 28);
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_true(took(qts, p));
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_quit(qts);
}

#define SPAN_NS         (FRAME_NS / 16 * 15)

/*
 * A T31X's frame reaches the MSCA with the frame's event, takes the buffer
 * queued then and writes its data from there; its counter runs 0 to the
 * height over the next 15/16 of a period, where its done comes, and holds
 * the height until the next frame
 */
static void test_t31_lines(void)
{
    const IspPart *p = &t31;
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);
    uint32_t rest = 64 << 16, a, b;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_clock_step(qts, MS);
    g_assert_cmphex(lines(qts, p), ==, 0);
    qtest_writel(qts, ISP + p->ch_en, 1);
    push(qts, p);
    qtest_writel(qts, ISP + p->vic, 1);
    g_assert_cmphex(lines(qts, p), ==, 0);
    g_assert_false(took(qts, p));
    push(qts, p);

    qtest_clock_step(qts, SPAN_NS / 2);
    a = lines(qts, p);
    g_assert_cmpuint(a >> 16, >=, 31);
    g_assert_cmpuint(a >> 16, <=, 32);
    g_assert_cmpuint(a & 0xffff, <, 64);
    g_assert_false(took(qts, p));
    qtest_clock_step(qts, SPAN_NS / 2 - MS);
    b = lines(qts, p);
    g_assert_cmpuint(b >> 16, >, 60);
    g_assert_cmpuint(b >> 16, <, 64);
    qtest_clock_step(qts, MS);
    g_assert_true(took(qts, p));
    g_assert_cmphex(lines(qts, p), ==, rest);

    /* held to the next frame, which has its buffer: from 0 again */
    qtest_clock_step(qts, IDLE_NS - MS);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, MS);
    g_assert_cmphex(lines(qts, p), ==, 0);
    qtest_clock_step(qts, SPAN_NS);
    g_assert_true(took(qts, p));

    /* a frame it has no buffer for: left as it was */
    qtest_clock_step(qts, IDLE_NS);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, FRAME_NS / 2);
    g_assert_false(took(qts, p));
    qtest_quit(qts);
}

/*
 * A T31X's channel takes the buffer pushed before a frame reaches the MSCA:
 * pushed after it (frozen consumer, mid-period) the counter stays held until
 * the next frame, which runs from 0 and puts the whole frame out from its
 * start, its done a period on
 */
static void test_latch(void)
{
    const IspPart *p = &t31;
    uint32_t c = ISP + p->ch, rest = 64 << 16;
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_clock_step(qts, MS);
    qtest_writel(qts, ISP + p->ch_en, 1);
    push(qts, p);
    qtest_writel(qts, p->vic + ISP, 1);
    qtest_clock_step(qts, SPAN_NS);
    g_assert_true(took(qts, p));
    g_assert_cmphex(qtest_readl(qts, c + 0x7c), ==, 0x11001);

    /* frame 2 started with no buffer; the push waits */
    qtest_clock_step(qts, IDLE_NS + FRAME_NS / 2);
    qtest_writel(qts, BUF, 0x12345678);
    push(qts, p);
    g_assert_cmphex(qtest_readl(qts, c + 0x6c), ==, BUF | 1);
    g_assert_cmphex(qtest_readl(qts, c + 0x84), ==, (BUF + 0x1000) | 1);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_clock_step(qts, FRAME_NS / 2 - MS);
    g_assert_cmphex(lines(qts, p), ==, rest);
    g_assert_cmphex(qtest_readl(qts, BUF), ==, 0x12345678);

    /* frame 3 starts: from 0, its data in memory, its done a period on */
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmpuint(lines(qts, p) >> 16, <=, 2);
    g_assert_cmphex(qtest_readl(qts, c + 0x7c), ==, 0x11001);
    g_assert_cmphex(qtest_readl(qts, BUF), !=, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, c + 0x6c), ==, BUF | 1);
    qtest_clock_step(qts, SPAN_NS - 3 * MS);
    g_assert_cmpuint(lines(qts, p) >> 16, >=, 58);
    g_assert_cmphex(qtest_readl(qts, c + 0x6c), ==, BUF | 1);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmphex(qtest_readl(qts, c + 0x7c), ==, 0x21100);
    g_assert_cmphex(qtest_readl(qts, c + 0x6c), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, c + 0x84), ==, BUF + 0x1000);
    g_assert_true(took(qts, p));
    g_assert_cmphex(qtest_readl(qts, c + 0x7c), ==, 0x22001);
    g_assert_cmphex(lines(qts, p), ==, rest);
    qtest_quit(qts);
}

/* A reset in a frame's period leaves no done to come */
static void test_t31_reset(void)
{
    const IspPart *p = &t31;
    uint32_t c = ISP + p->ch;
    QTestState *qts = qtest_initf("-machine %s -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25", p->machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_clock_step(qts, MS);
    qtest_writel(qts, ISP + p->ch_en, 1);
    push(qts, p);
    qtest_writel(qts, ISP + p->vic, 1);
    qtest_clock_step(qts, SPAN_NS / 2);
    g_assert_cmpuint(lines(qts, p) >> 16, >, 0);
    qtest_qmp_assert_success(qts, "{'execute': 'system_reset'}");
    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(lines(qts, p), ==, 0);
    qtest_clock_step(qts, FRAME_NS * 3);
    g_assert_cmphex(qtest_readl(qts, c + 0x7c), ==, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xb4), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-lines/t31", test_t31_lines);
    qtest_add_data_func("/ingenic-isp-lines/t40", &t40, test_lines);
    qtest_add_func("/ingenic-isp-lines/t31-latch", test_latch);
    qtest_add_func("/ingenic-isp-lines/t31-reset", test_t31_reset);
    return g_test_run();
}
