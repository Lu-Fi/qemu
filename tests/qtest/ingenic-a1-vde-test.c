/*
 * QTest testcase for the Ingenic A1 VDE (video display engine)
 *
 * The mixer's reset matrix, the shadow latch bits, the frame interrupt
 * at the timing's rate (and the half frame's), INTP clear and enable, and
 * the console: the background, an NV12 window and the G layer's ARGB8888,
 * as the SDK's driver and U-Boot set them.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define VDE             0x13300000
#define V_SHADOW0       0x010
#define V_RST           0x030
#define V_MODULE_EN     0x040
#define V_LAYER_EN      0x044
#define V_V0_FMT        0x080
#define V_G_FMT         0x624
#define V_G_ADDR        0x63c
#define V_G_STRIDE      0x640
#define V_STOP_ON       0x10000
#define V_STOP_STATE    0x1000c
#define V_BG0           0x20008
#define V_CSC           0x202a0
#define V_DSC_ENABLE    0x30000
#define V_DSC_SIZE      0x30010
#define V_DSC_TOTAL     0x30020
#define V_INTP_EN0      0x60020
#define V_INTP_CLR0     0x60024
#define V_INTP_INFO0    0x60028

#define W               128
#define H               64
#define NV12            0x01000000
#define GFB             0x01100000

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, VDE + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, VDE + off, v);
}

/* 720p60's timing out of HDMI, the module on */
static void vde_start(QTestState *qts, uint32_t w, uint32_t h,
                      uint32_t htotal, uint32_t vtotal)
{
    wr(qts, V_DSC_SIZE, h << 16 | w);
    wr(qts, V_DSC_TOTAL, vtotal << 16 | htotal);
    wr(qts, V_MODULE_EN, 1);
    wr(qts, V_DSC_ENABLE, 0x10);
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(rd(qts, V_CSC), ==, 0x400);
    g_assert_cmphex(rd(qts, V_CSC + 4), ==, 0x400059c);
    wr(qts, V_BG0, 0x808000);
    g_assert_cmphex(rd(qts, V_BG0), ==, 0x808000);
    /* DMA_STOP_STATE follows DMA_STOP_ON; TOP_RST clears it and reads 0 */
    g_assert_cmpuint(rd(qts, V_STOP_STATE), ==, 0);
    wr(qts, V_STOP_ON, 1);
    g_assert_cmpuint(rd(qts, V_STOP_STATE), ==, 1);
    wr(qts, V_RST, 1);
    g_assert_cmpuint(rd(qts, V_STOP_STATE), ==, 0);
    g_assert_cmpuint(rd(qts, V_RST), ==, 0);
    /* A word stored at an odd address lands byte by byte */
    qtest_writeb(qts, VDE + V_BG0 + 1, 0x55);
    g_assert_cmphex(rd(qts, V_BG0), ==, 0x805500);
    /* No frame runs: a latch request applies at once */
    wr(qts, V_SHADOW0, 1);
    g_assert_cmpuint(rd(qts, V_SHADOW0), ==, 0);
    qtest_quit(qts);
}

static void test_frames(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    vde_start(qts, 1280, 720, 1650, 750);
    wr(qts, V_INTP_EN0, 0x10000001);
    g_assert_cmphex(rd(qts, V_INTP_INFO0), ==, 0);
    wr(qts, V_SHADOW0, 0x11);
    g_assert_cmphex(rd(qts, V_SHADOW0), ==, 0x11);
    /* Half a frame of 1/60 s: the half-frame bit, not yet the frame's */
    qtest_clock_step(qts, 8400000);
    g_assert_cmphex(rd(qts, V_INTP_INFO0), ==, 0x10000000);
    qtest_clock_step(qts, 8400000);
    g_assert_cmphex(rd(qts, V_INTP_INFO0) & 0xb, ==, 0xb);
    g_assert_cmphex(rd(qts, V_SHADOW0), ==, 0);
    wr(qts, V_INTP_CLR0, 0x1000000b);
    g_assert_cmphex(rd(qts, V_INTP_INFO0), ==, 0);
    /* 2640 by 1125 is taken for 1080p50: 20 ms */
    wr(qts, V_DSC_ENABLE, 0);
    vde_start(qts, 1920, 1080, 2640, 1125);
    qtest_clock_step(qts, 15000000);
    g_assert_cmphex(rd(qts, V_INTP_INFO0) & 1, ==, 0);
    qtest_clock_step(qts, 6000000);
    g_assert_cmphex(rd(qts, V_INTP_INFO0) & 1, ==, 1);
    qtest_quit(qts);
}

static void test_console(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    g_autofree char *path = g_strdup_printf("%s/a1vde-XXXXXX.ppm",
                                            g_get_tmp_dir());
    g_autofree uint8_t *y = g_malloc(32 * 16);
    g_autofree uint8_t *uv = g_malloc(32 * 8);
    uint32_t px[4] = { 0xffff0000, 0xff00ff00, 0xff0000ff, 0xffffffff };
    g_autofree gchar *ppm = NULL;
    gsize len;
    int fd = g_mkstemp(path);
    unsigned pw, ph, max, hdr;
    QDict *rsp;

    g_assert(fd >= 0);
    close(fd);
    vde_start(qts, W, H, 1650, 750);
    /* A 32x16 NV12 window at (8, 4): Y 255, U and V 128 is white */
    memset(y, 255, 32 * 16);
    memset(uv, 128, 32 * 8);
    qtest_memwrite(qts, NV12, y, 32 * 16);
    qtest_memwrite(qts, NV12 + 32 * 16, uv, 32 * 8);
    wr(qts, V_BG0, 0x808000);
    wr(qts, V_V0_FMT, 0x400020);
    wr(qts, 0xf0, 4 << 16 | 8);
    wr(qts, 0xf4, 16 << 16 | 32);
    wr(qts, 0xf8, NV12);
    wr(qts, 0xfc, 32);
    wr(qts, 0x100, NV12 + 32 * 16);
    wr(qts, 0x104, 32);
    /* G: ARGB8888, four pixels at the screen's origin */
    qtest_memwrite(qts, GFB, px, sizeof(px));
    wr(qts, V_G_FMT, 0);
    wr(qts, V_G_ADDR, GFB);
    wr(qts, V_G_STRIDE, W * 4);
    /* The pixel's alpha, as bits 17:16 of 0x202d4 say */
    wr(qts, 0x202d4, 1 << 16);
    wr(qts, V_LAYER_EN, 0x101);
    rsp = qtest_qmp(qts, "{'execute': 'screendump', 'arguments': "
                    "{'filename': %s, 'device': 'vde'}}", path);
    qobject_unref(rsp);
    g_assert(g_file_get_contents(path, &ppm, &len, NULL));
    unlink(path);
    g_assert_cmpint(sscanf(ppm, "P6 %u %u %u %n", &pw, &ph, &max, &hdr), ==,
                    3);
    g_assert_cmpuint(pw, ==, W);
    g_assert_cmpuint(ph, ==, H);
#define PIXEL(x, y, c) ((uint8_t)ppm[hdr + ((y) * W + (x)) * 3 + (c)])
    /* G's pixels, red green blue white */
    g_assert_cmpuint(PIXEL(0, 0, 0), ==, 255);
    g_assert_cmpuint(PIXEL(0, 0, 1), ==, 0);
    g_assert_cmpuint(PIXEL(1, 0, 1), ==, 255);
    g_assert_cmpuint(PIXEL(1, 0, 0), ==, 0);
    g_assert_cmpuint(PIXEL(2, 0, 2), ==, 255);
    g_assert_cmpuint(PIXEL(3, 0, 0), ==, 255);
    /*
     * The window's white, and the black around it (G is transparent where
     * its memory is zero: alpha 0)
     */
    g_assert_cmpuint(PIXEL(8 + 10, 4 + 3, 0), ==, 255);
    g_assert_cmpuint(PIXEL(8 + 10, 4 + 3, 2), ==, 255);
    g_assert_cmpuint(PIXEL(40, 4 + 3, 0), ==, 0);
    g_assert_cmpuint(PIXEL(8 + 10, 30, 1), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-vde/regs", test_regs);
    qtest_add_func("ingenic-a1-vde/frames", test_frames);
    qtest_add_func("ingenic-a1-vde/console", test_console);
    return g_test_run();
}
