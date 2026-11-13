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

#define CPM_CLKGR0      0x10000030
#define GATE_VDE        (1u << 28)

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

/* The VDE's clock gate is shut at power-on; the driver opens it */
static QTestState *vde_init(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    qtest_writel(qts, CPM_CLKGR0,
                 qtest_readl(qts, CPM_CLKGR0) & ~GATE_VDE);
    return qts;
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
    QTestState *qts = vde_init();

    /* The BT.709 matrix, as an A1N reads it */
    g_assert_cmphex(rd(qts, V_CSC), ==, 0x400);
    g_assert_cmphex(rd(qts, V_CSC + 4), ==, 0x0400064d);
    g_assert_cmphex(rd(qts, V_CSC + 8), ==, 0x3e213f40);
    g_assert_cmphex(rd(qts, V_CSC + 12), ==, 0x076c0400);
    wr(qts, V_BG0, 0x808000);
    g_assert_cmphex(rd(qts, V_BG0), ==, 0x808000);
    /* DMA_STOP_STATE reads 1; TOP_RST clears DMA_STOP_ON and reads 0 */
    g_assert_cmpuint(rd(qts, V_STOP_STATE), ==, 1);
    wr(qts, V_STOP_ON, 1);
    wr(qts, V_RST, 1);
    g_assert_cmpuint(rd(qts, V_STOP_ON), ==, 0);
    g_assert_cmpuint(rd(qts, V_STOP_STATE), ==, 1);
    g_assert_cmpuint(rd(qts, V_RST), ==, 0);
    /* A word stored at an odd address lands byte by byte */
    qtest_writeb(qts, VDE + V_BG0 + 1, 0x55);
    g_assert_cmphex(rd(qts, V_BG0), ==, 0x805500);
    /* No frame runs: a latch request applies at once */
    wr(qts, V_SHADOW0, 1);
    g_assert_cmpuint(rd(qts, V_SHADOW0), ==, 0);
    qtest_quit(qts);
}

/* With the clock gate shut every word reads 0 and takes no write */
static void test_gate(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert(qtest_readl(qts, CPM_CLKGR0) & GATE_VDE);
    g_assert_cmphex(rd(qts, 0), ==, 0);
    g_assert_cmphex(rd(qts, V_CSC), ==, 0);
    g_assert_cmphex(rd(qts, V_DSC_TOTAL), ==, 0);
    wr(qts, V_BG0, 0x123456);
    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) & ~GATE_VDE);
    g_assert_cmphex(rd(qts, 0), ==, 0x20210731);
    g_assert_cmphex(rd(qts, 4), ==, 0x11111111);
    g_assert_cmphex(rd(qts, V_BG0), ==, 0x00808000);
    qtest_quit(qts);
}

static void test_reset_values(void)
{
    QTestState *qts = vde_init();
    static const struct {
        uint32_t off, val;
    } rv[] = {
        { 0x050, 0x04380780 }, { 0x054, 0x04380780 }, { 0x058, 0x04380780 },
        { 0x05c, 0x00200020 }, { 0x044, 0 },
        { 0x080, 0x00200001 }, { 0x084, 0x07800000 }, { 0x088, 0x04380000 },
        { 0x800, 0x07800000 }, { 0x804, 0x04380000 }, { 0x80c, 0x20 },
        { 0x818, 0x08000004 }, { 0x820, 0x07800000 }, { 0x824, 0x04380000 },
        { 0x82c, 0x20 }, { 0x838, 0x08000004 }, { 0x844, 0x02001001 },
        { 0x848, 0x100 }, { 0x84c, 0x01000000 },
        { 0x10004, 2 }, { 0x10008, 1 }, { 0x1000c, 1 },
        { 0x20004, 0x14015000 }, { 0x20008, 0x00808000 },
        { 0x2000c, 0x01e00280 }, { 0x20014, 0x01e00280 },
        { 0x20018, 0x40004000 }, { 0x20024, 0x8080 },
        { 0x20040, 0x80fffff9 }, { 0x20044, 0x83fefdd9 },
        { 0x20048, 0x835fdf4b }, { 0x2004c, 0x8378fe5a },
        { 0x202b4, 0x8000 }, { 0x202b8, 0xff00ff00 }, { 0x202bc, 0xff00ff00 },
        { 0x202d0, 0x00ff00ff }, { 0x202d4, 0xff }, { 0x202d8, 0xff },
        { 0x202dc, 0x00ff00ff }, { 0x202e0, 0xff }, { 0x202e4, 0xff },
        { 0x20300, 0x01e00280 }, { 0x20304, 0x40004000 }, { 0x20310, 0x8080 },
        { 0x20330, 0x8123d9b5 }, { 0x20334, 0x819dfa2b },
        { 0x20338, 0x83de0a56 },
        { 0x30000, 0 }, { 0x30010, 0x04380780 }, { 0x30020, 0x04650898 },
        { 0x30024, 0x002d0001 }, { 0x30028, 0x00060001 },
        { 0x3002c, 0x002d0001 }, { 0x30030, 0x00060001 },
        { 0x30034, 0x084100c1 }, { 0x30038, 0x0462002a }, { 0x30040, 0x76c },
        { 0x30044, 0x3025 }, { 0x30050, 0x02dc00da }, { 0x30054, 0x3f8b004a },
        { 0x30058, 0x02003e75 }, { 0x3005c, 0x3e2f0200 }, { 0x30060, 0x3fd1 },
        { 0x30064, 0x80000000 }, { 0x30068, 0xff00ff00 }, { 0x30090, 1 },
        { 0x300a4, 0x210 }, { 0x300ac, 0x210 }, { 0x60000, 0 },
    };

    for (unsigned i = 0; i < ARRAY_SIZE(rv); i++) {
        g_assert_cmphex(rd(qts, rv[i].off), ==, rv[i].val);
    }
    qtest_quit(qts);
}

/* The bits a register keeps of all ones written; the read-only ones keep 0 */
static void test_keep(void)
{
    QTestState *qts = vde_init();
    static const struct {
        uint32_t off, mask;
    } k[] = {
        { 0x050, ~0u }, { 0x05c, ~0u }, { 0x080, 0xffff0007 },
        { 0x084, ~0u }, { 0x088, ~0u }, { 0x10004, 3 }, { 0x10008, ~0u },
        { 0x20004, 0x17055557 }, { 0x20008, ~0u }, { 0x2000c, ~0u },
        { 0x20014, ~0u }, { 0x20018, ~0u }, { 0x20024, 0xf1ffffff },
        { 0x202a0, 0x3fff3fff }, { 0x202a4, 0x3fff3fff },
        { 0x202a8, 0x3fff3fff }, { 0x202ac, 0x3fff3fff },
        { 0x202b4, ~0u }, { 0x202b8, ~0u }, { 0x202bc, ~0u },
        { 0x202d0, 0x11ff11ff }, { 0x202d4, 0x000311ff },
        { 0x30020, ~0u }, { 0x30024, ~0u }, { 0x30034, ~0u },
        { 0x30038, ~0u }, { 0x30064, ~0u }, { 0x30068, ~0u },
    };
    static const uint32_t ro[] = { 0x800, 0x804, 0x844, 0x84c, 0x1000c,
                                   0x20040, 0x20330 };

    for (unsigned i = 0; i < ARRAY_SIZE(k); i++) {
        wr(qts, k[i].off, ~0u);
        g_assert_cmphex(rd(qts, k[i].off), ==, k[i].mask);
        wr(qts, k[i].off, 0);
        g_assert_cmphex(rd(qts, k[i].off), ==, 0);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(ro); i++) {
        uint32_t before = rd(qts, ro[i]);

        wr(qts, ro[i], ~0u);
        g_assert_cmphex(rd(qts, ro[i]), ==, before);
    }
    /* The layer enable keeps nothing until the module is on */
    wr(qts, V_LAYER_EN, ~0u);
    g_assert_cmphex(rd(qts, V_LAYER_EN), ==, 0);
    wr(qts, V_MODULE_EN, 1);
    wr(qts, V_LAYER_EN, 0x1111);
    g_assert_cmphex(rd(qts, V_LAYER_EN), ==, 0x1111);
    qtest_quit(qts);
}

static void test_frames(void)
{
    QTestState *qts = vde_init();

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
    QTestState *qts = vde_init();
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
    /*
     * A second NV12 window at (64, 24), 16x16, Y 128: the first four
     * chroma lines (luma lines 0 to 7) are neutral, the next four V 255,
     * so luma line 6 is grey and line 12 red
     */
    memset(y, 128, 16 * 16);
    memset(uv, 128, 16 * 8);
    for (int i = 16 * 4 + 1; i < 16 * 8; i += 2) {
        uv[i] = 255;
    }
    qtest_memwrite(qts, NV12 + 0x1000, y, 16 * 16);
    qtest_memwrite(qts, NV12 + 0x1000 + 16 * 16, uv, 16 * 8);
    wr(qts, 0x110, 24 << 16 | 64);
    wr(qts, 0x114, 16 << 16 | 16);
    wr(qts, 0x118, NV12 + 0x1000);
    wr(qts, 0x11c, 16);
    wr(qts, 0x120, NV12 + 0x1000 + 16 * 16);
    wr(qts, 0x124, 16);
    wr(qts, V_BG0, 0x808000);
    /* What vo.ko writes for NV12: V0 keeps no bit of the low byte but 2:0 */
    wr(qts, V_V0_FMT, 0x400000);
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
    /* The chroma of luma line 6 is chroma line 3, of line 12 line 6 */
    g_assert_cmpuint(PIXEL(64 + 4, 24 + 6, 0), ==, PIXEL(64 + 4, 24 + 6, 2));
    g_assert_cmpuint(PIXEL(64 + 4, 24 + 12, 0), ==, 255);
    g_assert_cmpuint(PIXEL(64 + 4, 24 + 12, 1), <, 100);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-vde/gate", test_gate);
    qtest_add_func("ingenic-a1-vde/reset-values", test_reset_values);
    qtest_add_func("ingenic-a1-vde/keep", test_keep);
    qtest_add_func("ingenic-a1-vde/regs", test_regs);
    qtest_add_func("ingenic-a1-vde/frames", test_frames);
    qtest_add_func("ingenic-a1-vde/console", test_console);
    return g_test_run();
}
