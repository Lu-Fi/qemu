/*
 * QTest testcase for the Ingenic T40 DPU and the T41 LCDC tiny
 *
 * Resets and the bits each register keeps, the gate, and composed frames
 * written back as a T40N composes them at U-Boot: blending with layer
 * config bit 14 at every alpha, the premultiplied blend, global alpha,
 * Z order, RGB565 and NV12. Then a direct TFT frame on the console, its
 * CMP_START and DISP_END interrupts, a frame of HPE * VPE pixel clocks,
 * and the generic stop. The T41's: resets and write masks, START and
 * SLCD_START, FRM_END and its descriptor mask, DMA_SITE, DES_READ, the
 * SLCD frame length, continuous frames and the generic stop, as a T41LQ
 * runs them.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define CPM             0x10000000
#define CPM_CPMPCR      0x14
#define CPM_CLKGR       0x20
#define CPM_LPCDR       0x64
#define DPU             0x13050000

#define FRM_CFG_ADDR    0x0000
#define FRM_CFG_CTRL    0x0004
#define DC_CTRL         0x2000
#define DC_ST           0x2004
#define DC_CLR_ST       0x2008
#define DC_INTC         0x200c
#define DC_INT_FLAG     0x2010
#define DC_COM_CFG      0x2014
#define DISP_COM        0x8000
#define TFT_HSYNC       0x9000
#define TFT_VSYNC       0x9004
#define TFT_HDE         0x9008
#define TFT_VDE         0x900c
#define TFT_ST          0x9014

#define DESC            0x02000000
#define LAY0            0x02000100
#define LAY1            0x02000200
#define BUF0            0x02100000
#define BUF1            0x02120000
#define UV0             0x02150000
#define WB              0x02200000

/* 1200 MHz MPLL over 12: 100 MHz */
#define PCLK_DIV        12

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, DPU + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, DPU + off, v);
}

static QTestState *dpu_init(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    qtest_writel(qts, CPM + CPM_CPMPCR, 0x0640510d);
    qtest_writel(qts, CPM + CPM_LPCDR, 1u << 30 | 1 << 29 | (PCLK_DIV - 1));
    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 24));
    return qts;
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");
    static const struct {
        uint32_t off, reset, ones;
    } r[] = {
        { 0x0000, 0, 0xffffffff }, { 0x1000, 0, 0xffffffff },
        { 0x200c, 0, 0x8003ffc6 }, { 0x2014, 0x00cf000c, 0x00ff00fe },
        { 0x2018, 6, 7 }, { 0x201c, 6, 7 },
        { 0x2020, 0x03190842, 0x07ffffff }, { 0x2024, 0x00c04010, 0x07ffffff },
        { 0x2028, 0xb, 0x7ff }, { 0x202c, 0x00600400, 0x01ffffff },
        { 0x2030, 0x01000400, 0x01ffffff }, { 0x2034, 0x01e00400, 0x01ffffff },
        { 0x3000, 0, 0xf0f }, { 0x3010, 0, 0xfffff000 },
        { 0x3040, 0, 0x8fff0fff }, { 0x3050, 0, 0xf },
        { 0x3200, 0x066204a8, 0x07ff07ff }, { 0x3204, 0x03410190, 0x03ff01ff },
        { 0x3208, 0x812, 0xfff }, { 0x320c, 0x00800000, 0x00ff001f },
        { 0x3210, 0x066204a8, 0x07ff07ff }, { 0x321c, 0x00800000, 0x00ff001f },
        { 0x8000, 0xe0, 0x3f00f3 }, { 0x9000, 0, 0x0fff0fff },
        { 0x900c, 0, 0x0fff0fff }, { 0x9010, 0, 0x003f0707 },
        { 0x9014, 0, 0 }, { 0xa000, 0x01540d20, 0x0fff0fff },
        { 0xa004, 0, 0xffffffff }, { 0xa010, 0, 0xffff },
        { 0xa014, 0x80000000, 0xe0ffffff }, { 0xa018, 0, 0 },
    };

    /* gated, as U-Boot leaves it: reads 0, drops writes */
    g_assert_cmphex(rd(qts, DC_COM_CFG), ==, 0);
    wr(qts, DC_COM_CFG, 0);
    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 24));
    for (int i = 0; i < ARRAY_SIZE(r); i++) {
        g_assert_cmphex(rd(qts, r[i].off), ==, r[i].reset);
        wr(qts, r[i].off, 0xffffffff);
        g_assert_cmphex(rd(qts, r[i].off), ==, r[i].ones);
        wr(qts, r[i].off, r[i].reset);
    }
    /* DISP_COM answers over 0x8000-0x8fff and from 0xb000; only it writes */
    g_assert_cmphex(rd(qts, 0x8ffc), ==, 0xe0);
    g_assert_cmphex(rd(qts, 0xfffc), ==, 0xe0);
    wr(qts, 0x8004, 0);
    g_assert_cmphex(rd(qts, DISP_COM), ==, 0xe0);
    qtest_quit(qts);
}

static void frame_desc(QTestState *qts, uint32_t ctrl, uint32_t laycfg,
                       uint32_t intmask)
{
    uint32_t d[11] = {
        DESC, 16 << 16 | 16, ctrl, WB, 16, LAY0, LAY1, 0x02000300,
        0x02000400, laycfg, intmask,
    };

    for (int i = 0; i < 11; i++) {
        qtest_writel(qts, DESC + 4 * i, d[i]);
    }
}

static void layer_desc(QTestState *qts, uint32_t at, uint32_t size,
                       uint32_t cfg, uint32_t buf, uint32_t pos,
                       uint32_t stride, uint32_t uv)
{
    qtest_memset(qts, at, 0, 48);
    qtest_writel(qts, at, size);
    qtest_writel(qts, at + 4, cfg);
    qtest_writel(qts, at + 8, buf);
    qtest_writel(qts, at + 24, pos);
    qtest_writel(qts, at + 36, stride);
    qtest_writel(qts, at + 40, uv);
    qtest_writel(qts, at + 44, stride);
}

/* A writeback-only frame: compose at once, the result at WB */
static void compose(QTestState *qts)
{
    wr(qts, DC_CLR_ST, 0xffffffff);
    wr(qts, FRM_CFG_ADDR, DESC);
    wr(qts, FRM_CFG_CTRL, 1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, DC_ST) & 1, ==, 0);
}

static uint32_t wbpix(QTestState *qts, unsigned i)
{
    return qtest_readl(qts, WB + 4 * i);
}

/*
 * Layer 0 opaque 0xff00c0ff, layer 1's pixel i 0xff8037 at alpha i, Z 0
 * and 1, layers 2 and 3 given Z 2 and 3
 */
static void two_layers(QTestState *qts, uint32_t cfg1)
{
    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, 0xff00c0ff);
        qtest_writel(qts, BUF1 + 4 * i, (uint32_t)i << 24 | 0xff8037);
    }
    frame_desc(qts, 3, 0xe430, 0);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);
    layer_desc(qts, LAY1, 0x00100010, cfg1, BUF1, 0, 16, 0);
    compose(qts);
}

static void test_blend(void)
{
    QTestState *qts = dpu_init();
    /* T40N writeback, layer 1 config 0x54000: alpha 0, 1, 2, 3, 0x80 ... */
    static const struct {
        unsigned i;
        uint32_t v;
    } mult[] = {
        { 0, 0xff00c0ff }, { 1, 0xff01bffd }, { 2, 0xff02bffc },
        { 3, 0xff03bffc }, { 0x5b, 0xff5ba9b7 }, { 0x7f, 0xff7fa09b },
        { 0x80, 0xff809f9a }, { 0x81, 0xff809f99 }, { 0xc0, 0xffbf8f68 },
        { 0xfe, 0xfffd8038 }, { 0xff, 0xffff8037 },
    }, premult[] = {
        { 0, 0xffffffff }, { 0x37, 0xfffffffe }, { 0x57, 0xfffffede },
        { 0x7f, 0xffffe0b7 }, { 0x80, 0xffffdfb6 }, { 0xb0, 0xffffbb86 },
        { 0xc0, 0xffffaf76 }, { 0xfe, 0xffff8138 }, { 0xff, 0xffff8037 },
    }, galpha[] = {
        { 0, 0xff00c0ff }, { 1, 0xff00c0ff }, { 2, 0xff01bffd },
        { 0x3c, 0xff1eb8e7 }, { 0x80, 0xff40afcc }, { 0x81, 0xff41afcb },
        { 0xfc, 0xff7ea09c }, { 0xff, 0xff809f9a },
    };

    two_layers(qts, 0x54000);
    for (int i = 0; i < ARRAY_SIZE(mult); i++) {
        g_assert_cmphex(wbpix(qts, mult[i].i), ==, mult[i].v);
    }
    two_layers(qts, 0x50000);
    for (int i = 0; i < ARRAY_SIZE(premult); i++) {
        g_assert_cmphex(wbpix(qts, premult[i].i), ==, premult[i].v);
    }
    two_layers(qts, 0x56080);
    for (int i = 0; i < ARRAY_SIZE(galpha); i++) {
        g_assert_cmphex(wbpix(qts, galpha[i].i), ==, galpha[i].v);
    }
    qtest_quit(qts);
}

/* Layer 0 red at Z 0 under an 8x8 green layer 1 at (4, 4) */
static void test_zorder(void)
{
    QTestState *qts = dpu_init();

    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, 0xffff0000);
    }
    for (int i = 0; i < 64; i++) {
        qtest_writel(qts, BUF1 + 4 * i, 0x0000ff00);
    }
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);
    layer_desc(qts, LAY1, 0x00080008, 0x42080, BUF1, 0x00040004, 8, 0);

    /* layers 2 and 3 at Z 0 too take layer 0's slot: it disappears */
    frame_desc(qts, 3, 0x0430, 0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 0), ==, 0);
    g_assert_cmphex(wbpix(qts, 4 * 16 + 4), ==, 0x8000ff00);

    frame_desc(qts, 3, 0xe430, 0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 0), ==, 0xffff0000);
    g_assert_cmphex(wbpix(qts, 4 * 16 + 4), ==, 0xff7fff00);
    layer_desc(qts, LAY1, 0x00080008, 0x46080, BUF1, 0x00040004, 8, 0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 4 * 16 + 4), ==, 0xff7f8000);
    layer_desc(qts, LAY1, 0x00080008, 0x46000, BUF1, 0x00040004, 8, 0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 4 * 16 + 4), ==, 0xffff0000);
    layer_desc(qts, LAY1, 0x00080008, 0x40080, BUF1, 0x00040004, 8, 0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 4 * 16 + 4), ==, 0xff00ff00);
    qtest_quit(qts);
}

static void test_formats(void)
{
    QTestState *qts = dpu_init();
    static const uint32_t nv12[8] = {
        0xff00ff00, 0xff00ff13, 0xff00e3d5, 0xff15ef13, 0xff48c854,
        0xff7b89ff, 0xffe16d95, 0xffff14ff,
    };
    static const unsigned nv12_at[8][2] = {
        { 0, 0 }, { 0, 4 }, { 0, 10 }, { 2, 4 }, { 4, 6 }, { 6, 12 },
        { 10, 8 }, { 14, 14 },
    };

    /* RGB565 widens by shifting */
    for (int i = 0; i < 256; i++) {
        qtest_writew(qts, BUF0 + 2 * i, i * 0x101);
    }
    frame_desc(qts, 3, 0xe410, 0);
    layer_desc(qts, LAY0, 0x00100010, 0x20000, BUF0, 0, 16, 0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 1), ==, 0xff002008);
    g_assert_cmphex(wbpix(qts, 0x81), ==, 0xff803008);
    g_assert_cmphex(wbpix(qts, 0xff), ==, 0xfff8fcf8);

    /* NV12, Y ramp over neutral chroma, then Y 0x80 under a UV grid */
    for (int i = 0; i < 256; i++) {
        qtest_writeb(qts, BUF0 + i, i);
    }
    qtest_memset(qts, UV0, 0x80, 128);
    layer_desc(qts, LAY0, 0x00100010, 0x80000, BUF0, 0, 16, UV0);
    compose(qts);
    g_assert_cmphex(wbpix(qts, 7), ==, 0xff080808);
    g_assert_cmphex(wbpix(qts, 0xdb), ==, 0xfffefefe);
    g_assert_cmphex(wbpix(qts, 0xdc), ==, 0xffffffff);

    qtest_memset(qts, BUF0, 0x80, 256);
    for (int r = 0; r < 8; r++) {
        for (int k = 0; k < 8; k++) {
            qtest_writeb(qts, UV0 + r * 16 + 2 * k, k * 0x20);
            qtest_writeb(qts, UV0 + r * 16 + 2 * k + 1, r * 0x20 + 0x10);
        }
    }
    compose(qts);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(wbpix(qts, nv12_at[i][0] * 16 + nv12_at[i][1]), ==,
                        nv12[i]);
    }
    qtest_quit(qts);
}

/* A 16x16 TFT frame in a 400 x 100 clock raster */
static void test_tft(void)
{
    QTestState *qts = dpu_init();
    g_autofree char *path = g_strdup_printf("%s/dpu-XXXXXX.ppm",
                                            g_get_tmp_dir());
    int64_t ns = 400LL * 100 * PCLK_DIV * 1000 / 1200;
    g_autofree gchar *ppm = NULL;
    unsigned pw, ph, max, hdr;
    gsize len;
    int fd = g_mkstemp(path);
    QDict *rsp;

    g_assert(fd >= 0);
    close(fd);
    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, i < 128 ? 0xffff0000 : 0xff0000ff);
    }
    /* direct, refreshing; CMP_START and DISP_END unmasked */
    frame_desc(qts, 4, 0xe410, 1 << 17 | 1 << 14);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);
    wr(qts, TFT_HSYNC, 4 << 16 | 400);
    wr(qts, TFT_VSYNC, 2 << 16 | 100);
    wr(qts, TFT_HDE, 12 << 16 | 28);
    wr(qts, TFT_VDE, 6 << 16 | 22);
    wr(qts, DISP_COM, 0xe1);
    wr(qts, DC_INTC, 1 << 17 | 1 << 14);
    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    wr(qts, FRM_CFG_ADDR, DESC);
    wr(qts, FRM_CFG_CTRL, 1);
    g_assert_cmphex(rd(qts, DC_ST), ==, 0x4611);
    g_assert_cmphex(rd(qts, DC_INT_FLAG), ==, 0x4000);
    g_assert_cmphex(rd(qts, TFT_ST), ==, 2);
    g_assert_true(qtest_get_irq(qts, 20));
    wr(qts, DC_CLR_ST, 1 << 14);
    g_assert_false(qtest_get_irq(qts, 20));

    qtest_clock_step(qts, ns - 100);
    g_assert_cmphex(rd(qts, DC_INT_FLAG), ==, 0);
    qtest_clock_step(qts, 100);
    g_assert_cmphex(rd(qts, DC_INT_FLAG), ==, 0x24000);
    g_assert_true(qtest_get_irq(qts, 20));

    rsp = qtest_qmp(qts, "{'execute': 'screendump', 'arguments': "
                    "{'filename': %s, 'device': 'lcd'}}", path);
    g_assert(!qdict_haskey(rsp, "error"));
    qobject_unref(rsp);
    g_assert(g_file_get_contents(path, &ppm, &len, NULL));
    unlink(path);
    g_assert_cmpint(sscanf(ppm, "P6 %u %u %u %n", &pw, &ph, &max, &hdr), ==,
                    3);
    g_assert_cmpuint(pw, ==, 16);
    g_assert_cmpuint(ph, ==, 16);
    g_assert_cmpuint((uint8_t)ppm[hdr], ==, 255);
    g_assert_cmpuint((uint8_t)ppm[hdr + 2], ==, 0);
    g_assert_cmpuint((uint8_t)ppm[hdr + 255 * 3], ==, 0);
    g_assert_cmpuint((uint8_t)ppm[hdr + 255 * 3 + 2], ==, 255);

    /* the generic stop waits for the frame's end, then acknowledges */
    wr(qts, DC_CLR_ST, 0xffffffff);
    wr(qts, DC_CTRL, 1 << 3);
    g_assert_cmphex(rd(qts, DC_ST) & 0x51, ==, 0x11);
    qtest_clock_step(qts, ns);
    g_assert_cmphex(rd(qts, DC_ST) & 0x200d1, ==, 0x20040);
    g_assert_cmphex(rd(qts, TFT_ST), ==, 0);
    qtest_quit(qts);
}

#define LCDC_GLB        0x0004
#define LCDC_CTRL       0x1000
#define LCDC_ST         0x1004
#define LCDC_CSR        0x1008
#define LCDC_INTC       0x100c
#define LCDC_INT_FLAG   0x101c
#define LCDC_SITE       0x2000
#define LCDC_DMA_SITE   0x2004
#define LCDC_DES_READ   0x3000
#define SLCD_CFG        0xa000
#define SLCD_DUTY       0xa004
#define SLCD_SIZE       0xa00c

static void test_lcdc_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    static const struct {
        uint32_t off, reset, ones;
    } r[] = {
        { 0x0000, 0, 0xffffffff }, { 0x0004, 0x12, 0x003f003f },
        { 0x0100, 0x066204a8, 0x07ff07ff }, { 0x010c, 0x00800000, 0x00ff001f },
        { 0x100c, 0, 0x7e }, { 0x1010, 0x03190842, 0x07ffffff },
        { 0x8000, 1, 0x3f0007 }, { 0x9000, 0, 0 }, { 0x9010, 0, 0 },
        { 0xa000, 0x01540d20, 0x8fff0f7f }, { 0xa004, 0, 0xffffffff },
        { 0xa010, 0, 0xffff }, { 0xa014, 0x80000000, 0xe0ffffff },
        { 0xa018, 0, 0 },
    };

    g_assert_cmphex(rd(qts, LCDC_GLB), ==, 0);
    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 24));
    for (int i = 0; i < ARRAY_SIZE(r); i++) {
        g_assert_cmphex(rd(qts, r[i].off), ==, r[i].reset);
        wr(qts, r[i].off, 0xffffffff);
        g_assert_cmphex(rd(qts, r[i].off), ==, r[i].ones);
        wr(qts, r[i].off, r[i].reset);
    }
    g_assert_cmphex(rd(qts, 0xb000), ==, 1);
    qtest_quit(qts);
}

/* 1400 MHz MPLL over 14: 100 MHz */
static QTestState *lcdc_init(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    qtest_writel(qts, CPM + CPM_CPMPCR, 0x15d0889d);
    qtest_writel(qts, CPM + CPM_LPCDR, 1u << 30 | 1 << 29 | 13);
    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 24));
    return qts;
}

static void lcdc_desc(QTestState *qts, uint32_t stride, uint32_t chain,
                      uint32_t intm)
{
    uint32_t d[7] = { DESC, BUF0, stride, chain, intm, UV0, stride };

    for (int i = 0; i < 7; i++) {
        qtest_writel(qts, DESC + 4 * i, d[i]);
    }
    wr(qts, 0, DESC);
}

static void test_lcdc_frame(void)
{
    QTestState *qts = lcdc_init();
    g_autofree char *path = g_strdup_printf("%s/lcdc-XXXXXX.ppm",
                                            g_get_tmp_dir());
    static const uint32_t des[7] = {
        DESC, BUF0, 0x16, 1, 0x7e, UV0, 0x16,
    };
    g_autofree gchar *ppm = NULL;
    unsigned pw, ph, max, hdr;
    gsize len;
    int fd = g_mkstemp(path);
    QDict *rsp;

    g_assert(fd >= 0);
    close(fd);
    for (int i = 0; i < 16 * 0x16; i++) {
        qtest_writel(qts, BUF0 + 4 * i, 0x00ff0000);
    }
    /* 16x16 RGB888 on a 24-bit bus, no TE wait: 2 + 2 clocks a pixel */
    wr(qts, LCDC_GLB, 0x20012);
    wr(qts, SLCD_CFG, 0x05500d20);
    wr(qts, SLCD_SIZE, 16 << 16 | 16);
    wr(qts, LCDC_INTC, 0x7e);
    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    lcdc_desc(qts, 0x16, 1, 2);

    /* the frame fits the FIFO: DMA_END, not WORKING, until SLCD_START */
    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 4);
    g_assert_cmphex(rd(qts, LCDC_SITE), ==, DESC);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 0x10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    g_assert_false(qtest_get_irq(qts, 20));
    qtest_clock_step(qts, 256 * 4 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 6);
    g_assert_cmphex(rd(qts, LCDC_INT_FLAG), ==, 6);
    g_assert_true(qtest_get_irq(qts, 20));
    g_assert_cmphex(rd(qts, LCDC_DMA_SITE), ==, BUF0 + 0x528);
    wr(qts, LCDC_CTRL, 4);
    for (int i = 0; i < 9; i++) {
        g_assert_cmphex(rd(qts, LCDC_DES_READ), ==, des[i % 7]);
    }

    rsp = qtest_qmp(qts, "{'execute': 'screendump', 'arguments': "
                    "{'filename': %s, 'device': 'lcd'}}", path);
    g_assert(!qdict_haskey(rsp, "error"));
    qobject_unref(rsp);
    g_assert(g_file_get_contents(path, &ppm, &len, NULL));
    unlink(path);
    g_assert_cmpint(sscanf(ppm, "P6 %u %u %u %n", &pw, &ph, &max, &hdr), ==,
                    3);
    g_assert_cmpuint(pw, ==, 16);
    g_assert_cmpuint(ph, ==, 16);
    g_assert_cmpuint((uint8_t)ppm[hdr], ==, 255);
    g_assert_cmpuint((uint8_t)ppm[hdr + 1], ==, 0);

    /* without the descriptor's EOF_MSK, FRM_END sets but does not flag */
    wr(qts, LCDC_CSR, 0xffffffff);
    lcdc_desc(qts, 0x16, 1, 0);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 256 * 4 * 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 6);
    g_assert_cmphex(rd(qts, LCDC_INT_FLAG), ==, 4);
    qtest_quit(qts);
}

/* 64x64 on an 8-bit bus, WR_DUTY 1: 3 * (2 + 2) + 2 clocks a pixel */
static void test_lcdc_continuous(void)
{
    QTestState *qts = lcdc_init();
    int64_t ns = 64 * 64 * 14 * 10;

    wr(qts, LCDC_GLB, 0x20012);
    wr(qts, SLCD_CFG, 0x85500d00);
    wr(qts, SLCD_DUTY, 0x01010101);
    wr(qts, SLCD_SIZE, 64 << 16 | 64);
    lcdc_desc(qts, 0x64, 0, 2);

    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    wr(qts, LCDC_CTRL, 0x10);
    for (int i = 0; i < 3; i++) {
        wr(qts, LCDC_CSR, 0xffffffff);
        qtest_clock_step(qts, ns - 10);
        g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
        qtest_clock_step(qts, 10);
        g_assert_cmphex(rd(qts, LCDC_ST), ==, 7);
    }
    g_assert_cmphex(rd(qts, LCDC_DMA_SITE), ==, BUF0 + 0x6330);

    /* the generic stop after the frame: GEN_ACK, and DMA_SITE cleared */
    wr(qts, LCDC_CTRL, 8);
    qtest_clock_step(qts, ns);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 0x26);
    g_assert_cmphex(rd(qts, LCDC_DMA_SITE), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-dpu/regs", test_regs);
    qtest_add_func("ingenic-dpu/blend", test_blend);
    qtest_add_func("ingenic-dpu/zorder", test_zorder);
    qtest_add_func("ingenic-dpu/formats", test_formats);
    qtest_add_func("ingenic-dpu/tft", test_tft);
    qtest_add_func("ingenic-dpu/lcdc/regs", test_lcdc_regs);
    qtest_add_func("ingenic-dpu/lcdc/frame", test_lcdc_frame);
    qtest_add_func("ingenic-dpu/lcdc/continuous", test_lcdc_continuous);
    return g_test_run();
}
