/*
 * QTest testcase for the T23's frame pacing: its IVDC holding a frame's
 * lines for its encoder, and a frame behind a MIPI switch waiting for the
 * dual-sensor driver to set the channels up
 *
 * The T23's Helix driver cancels a job that starts more than 500 lines
 * behind the ISP; the model's Helix codes in host time, so on the T23 a
 * frame's lines past the IVDC's RAM (none on a T23) wait for an encoder to
 * take it and go from then. A frame nobody takes overflows 50 ms after its
 * start (when the driver would cancel its job) rather than as the UV ring
 * fills, and the ISP holds the next frame until the guest has cleared the
 * overflow.
 *
 * Behind a MIPI switch the dual-sensor driver enables or disables each
 * channel for the next sensor, writing its Y push FIFO clear, after it
 * queues its buffer: VIC 0 holds the next frame until all three are set
 * up, or ten frame times after the last was served, and the frame goes out
 * on the channels enabled as it goes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define ISP_CH_EN       (ISP + 0xd040)
#define ISP_CH0         (ISP + 0xd100)
#define ISP_VIC         (ISP + 0xe0000)
#define IVDC            0x13200000
#define IVDC_ISP_SIZE   (IVDC + 0x10)
#define IVDC_DDR_LINES  (IVDC + 0x30)
#define IVDC_INT_EN     (IVDC + 0x40)
#define IVDC_INT_STA    (IVDC + 0x44)
#define IVDC_INT_CLR    (IVDC + 0x54)
#define IVDC_VPU0_START (IVDC + 0x70)
#define IVDC_DDR_WR     (IVDC + 0x208)
#define CPM_CLKGR0      0x10000020
#define FRAME_NS        40000000        /* fps=25 */
#define MS              1000000
#define LINES           1080
#define HOLD_NS         50000000        /* the driver's limit for a job */

#define INT_DDR_UV_OVF  (1u << 1)
#define INT_FRM_DONE    (1u << 16)
#define INT_FRM_START   (1u << 17)
#define INT_VPU_ACK_TO  (1u << 24)

/* Give channel 0 the IVDC's addresses for the next frame */
static void push_ivdc(QTestState *qts)
{
    qtest_writel(qts, ISP_CH0 + 0x04, 1920 << 16 | LINES);
    qtest_writel(qts, ISP_CH0 + 0x30, 0x80000000);
    qtest_writel(qts, ISP_CH0 + 0x50, 0x81000000);
}

/* The ring's Y write line */
static uint32_t ring_line(QTestState *qts)
{
    return (qtest_readl(qts, IVDC_DDR_WR) >> 16) & 0x7fff;
}

static uint32_t take_status(QTestState *qts)
{
    uint32_t sta = qtest_readl(qts, IVDC_INT_STA);

    qtest_writel(qts, IVDC_INT_CLR, sta);
    return sta;
}

static void test_hold(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t23 -device image-sensor,"
                                 "model=jxq03,bus=i2c0,fps=25");
    uint32_t line;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, IVDC_ISP_SIZE, LINES << 16);
    qtest_writel(qts, IVDC_DDR_LINES, 480 << 16 | 960);
    qtest_writel(qts, IVDC_INT_EN, 0xffffffff);
    qtest_writel(qts, ISP_CH_EN, 1);
    push_ivdc(qts);
    qtest_writel(qts, ISP_VIC, 1);      /* the first frame, at once */
    g_assert_cmphex(take_status(qts) & INT_FRM_START, ==, INT_FRM_START);
    qtest_readl(qts, ISP_CH0 + 0x38);   /* pop its done entry */

    /* untaken, past where the UV ring would have overflowed: no lines */
    qtest_clock_step(qts, 30 * MS);
    g_assert_cmpuint(ring_line(qts), ==, 0);
    g_assert_cmphex(take_status(qts) & INT_DDR_UV_OVF, ==, 0);

    /* taken: its lines go from now, over three quarters of a frame */
    qtest_writel(qts, IVDC_VPU0_START, 1);
    push_ivdc(qts);
    qtest_clock_step(qts, 5 * MS);
    line = ring_line(qts);
    g_assert_cmpuint(line, >=, 170);
    g_assert_cmpuint(line, <=, 190);
    g_assert_cmphex(take_status(qts) & INT_FRM_DONE, ==, 0);

    /* the next frame comes on time, and the one taken ends as it starts */
    qtest_clock_step(qts, 6 * MS);
    g_assert_cmphex(take_status(qts) & (INT_FRM_DONE | INT_FRM_START), ==,
                    INT_FRM_DONE | INT_FRM_START);
    qtest_readl(qts, ISP_CH0 + 0x38);

    /*
     * One nobody takes overflows 50 ms after its start, and the ISP holds
     * the next frame until the guest has cleared the overflow
     */
    push_ivdc(qts);
    qtest_clock_step(qts, HOLD_NS - 2 * MS);
    g_assert_cmphex(take_status(qts) & (INT_DDR_UV_OVF | INT_FRM_START), ==,
                    0);
    qtest_clock_step(qts, 3 * MS);
    g_assert_cmphex(qtest_readl(qts, IVDC_INT_STA) &
                    (INT_DDR_UV_OVF | INT_VPU_ACK_TO | INT_FRM_START), ==,
                    INT_DDR_UV_OVF | INT_VPU_ACK_TO);
    qtest_clock_step(qts, 10 * MS);
    g_assert_cmphex(qtest_readl(qts, IVDC_INT_STA) & INT_FRM_START, ==, 0);
    take_status(qts);
    qtest_clock_step(qts, 2 * MS);
    g_assert_cmphex(take_status(qts) & (INT_DDR_UV_OVF | INT_FRM_START), ==,
                    INT_FRM_START);
    qtest_quit(qts);
}

/* Serve VIC 0's interrupts and set channel 0 up for the next frame */
static void serve(QTestState *qts, bool set_up)
{
    qtest_writel(qts, ISP_VIC + 0x1f0, 0xffffffff);
    qtest_writel(qts, ISP_CH0 + 0x04, 64 << 16 | 64);
    qtest_writel(qts, ISP_CH0 + 0x40, 64);
    qtest_writel(qts, ISP_CH0 + 0x60, 64);
    qtest_writel(qts, ISP_CH0 + 0x30, 0x00400000);
    qtest_writel(qts, ISP_CH0 + 0x50, 0x00401000);
    if (set_up) {
        for (int n = 0; n < 3; n++) {
            qtest_writel(qts, ISP_CH0 + n * 0x100 + 0x44, 0);
        }
    }
}

/* Whether channel 0 put a frame out; if so, read it */
static bool took(QTestState *qts)
{
    if (qtest_readl(qts, ISP_CH0 + 0x3c) & 1) {
        return false;
    }
    qtest_readl(qts, ISP_CH0 + 0x38);
    return true;
}

static void test_switch(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t23 "
                                 "-global ingenic-t31.mipi-switch=7 "
                                 "-device image-sensor,model=jxq03,bus=i2c0,"
                                 "fps=25 -device image-sensor,model=jxq03,"
                                 "bus=i2c0,address=0x32,input=1,fps=25");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP_CH_EN, 1);
    serve(qts, true);
    qtest_writel(qts, ISP_VIC, 1);      /* its first frame a frame time on */
    qtest_clock_step(qts, FRAME_NS - MS);
    g_assert_false(took(qts));
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(took(qts));

    /* not set up: ten frame times after the serving */
    serve(qts, false);
    qtest_clock_step(qts, 11 * FRAME_NS - 2 * MS);
    g_assert_false(took(qts));
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(took(qts));

    /* set up late: at once */
    serve(qts, false);
    qtest_clock_step(qts, FRAME_NS + 10 * MS);
    g_assert_false(took(qts));
    for (int n = 0; n < 3; n++) {
        qtest_writel(qts, ISP_CH0 + n * 0x100 + 0x44, 0);
    }
    qtest_clock_step(qts, 2 * MS);
    g_assert_true(took(qts));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-ivdc-pace/t23/hold", test_hold);
    qtest_add_func("/ingenic-ivdc-pace/t23/switch", test_switch);
    return g_test_run();
}
