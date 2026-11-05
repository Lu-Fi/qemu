/*
 * QTest testcase for the IVDC's encoder starts and the sensor whose frame
 * each takes
 *
 * An encoder's start names a sensor in bits 10:8. On the T32 and T33,
 * whose drivers run several sensors' main channels over the ring, it waits
 * for a frame of that sensor and takes no other; on the T23 it takes the
 * frame live, whoever sent it. Either way the IVDC's "taken" output names
 * the sensor whose frame each encoder took last, line 4v + n for VPU v;
 * on the T32 as the start names it. The T32's 0x158 holds the frames on
 * the ring: their count in bits 2:0, the oldest one's sensor in 6:4.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define IVDC_PATH       "/machine/soc/ivdc"
#define CPM_CLKGR0      0x10000020
#define MS              1000000
#define LINES           1080

#define ISP_SIZE        0x10
#define INT_EN          0x40
#define INT_STA         0x44
#define INT_CLR         0x54
#define VPU0_START      0x70
#define VPU0_STAT       0x7c
#define VPU1_START      0x88
#define SID_FIFO        0x158

#define INT_FRM_START   (1u << 17)
#define INT_VPU_END(v)  (1u << (18 + (v)))

typedef struct {
    QTestState *qts;
    uint64_t base;
} Ivdc;

static Ivdc ivdc_init(const char *machine, uint64_t base)
{
    Ivdc d = { qtest_initf("-machine %s", machine), base };

    qtest_writel(d.qts, CPM_CLKGR0, 0);
    qtest_writel(d.qts, base + ISP_SIZE, LINES << 16);
    qtest_writel(d.qts, base + INT_EN, 0xffffffff);
    qtest_irq_intercept_out_named(d.qts, IVDC_PATH, "taken");
    return d;
}

/* Sensor @n's frame starts, as the ISP sends it */
static void frame(Ivdc *d, int n)
{
    qtest_set_irq_in(d->qts, IVDC_PATH, "frame", n, 1);
    qtest_set_irq_in(d->qts, IVDC_PATH, "frame", n, 0);
}

static void start(Ivdc *d, unsigned v, unsigned sensor)
{
    qtest_writel(d->qts, d->base + (v ? VPU1_START : VPU0_START),
                 sensor << 8 | 1);
}

static uint32_t take_status(Ivdc *d)
{
    uint32_t sta = qtest_readl(d->qts, d->base + INT_STA);

    qtest_writel(d->qts, d->base + INT_CLR, sta);
    return sta;
}

/* The "taken" lines of VPU @v as a mask of sensors */
static unsigned taken(Ivdc *d, unsigned v)
{
    unsigned mask = 0;

    for (int n = 0; n < 4; n++) {
        mask |= qtest_get_irq(d->qts, v * 4 + n) << n;
    }
    return mask;
}

static void test_t32_match(void)
{
    Ivdc d = ivdc_init("ingenic-t32", 0x13100000);

    /*
     * A start naming sensor 1 leaves sensor 0's frame alone: its encoder,
     * named on "taken" at once, reads the lines of sensor 1's next frame
     */
    frame(&d, 0);
    g_assert_cmphex(qtest_readl(d.qts, d.base + SID_FIFO), ==, 0x01);
    start(&d, 0, 1);
    g_assert_cmphex(qtest_readl(d.qts, d.base + VPU0_STAT) & 0x70, ==, 0x10);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 1);
    qtest_clock_step(d.qts, 35 * MS);
    g_assert_cmphex(take_status(&d) & INT_VPU_END(0), ==, 0);

    /* sensor 0's, untaken, is dropped 50 ms on; sensor 1's is taken */
    qtest_clock_step(d.qts, 20 * MS);
    g_assert_cmphex(qtest_readl(d.qts, d.base + SID_FIFO), ==, 0);
    frame(&d, 1);
    g_assert_cmphex(qtest_readl(d.qts, d.base + SID_FIFO), ==, 0x11);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 1);
    qtest_clock_step(d.qts, 45 * MS);     /* 3/4 of the 55 ms since the last */
    g_assert_cmphex(take_status(&d) & INT_VPU_END(0), ==, INT_VPU_END(0));
    g_assert_cmphex(qtest_readl(d.qts, d.base + SID_FIFO), ==, 0);

    /* live and named: at once; VPU1 has lines of its own */
    frame(&d, 2);
    start(&d, 1, 2);
    g_assert_cmphex(taken(&d, 1), ==, 1 << 2);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 1);
    start(&d, 0, 0);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 0);
    qtest_clock_step(d.qts, 35 * MS);
    frame(&d, 3);
    g_assert_cmphex(take_status(&d) & INT_VPU_END(0), ==, 0);
    qtest_clock_step(d.qts, 55 * MS);
    frame(&d, 0);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 0);
    qtest_quit(d.qts);
}

static void test_t23_live(void)
{
    Ivdc d = ivdc_init("ingenic-t23", 0x13200000);

    /* the frame live, whoever's it is */
    frame(&d, 0);
    g_assert_cmphex(take_status(&d) & INT_FRM_START, ==, INT_FRM_START);
    start(&d, 0, 1);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 0);
    frame(&d, 1);
    start(&d, 0, 0);
    g_assert_cmphex(taken(&d, 0), ==, 1 << 1);
    qtest_quit(d.qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-ivdc-match/t32/match", test_t32_match);
    qtest_add_func("/ingenic-ivdc-match/t23/live", test_t23_live);
    return g_test_run();
}
