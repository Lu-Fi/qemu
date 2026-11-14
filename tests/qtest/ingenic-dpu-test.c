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
 * runs them. The T21, T23 and T31's, the same block as a T23N and a T31X
 * run it: resets and masks, the pixel clock's change enable, the frame
 * length in clocks, the sizes it keeps, what a frame leaves in ST, the
 * stop acknowledges and the T31X's lost SLCD write.
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

/* Write-back formats of a layer of ARGB pixels, premultiplied over nothing */
static void test_wb_formats(void)
{
    QTestState *qts = dpu_init();
    static const uint32_t src[8] = {
        0x00a54dca, 0x01182530, 0x02bb1d6d, 0x03132cde,
        0x04d6237b, 0x052ed91e, 0x063f721f, 0x07cb1971,
    };
    static const uint32_t xrgb[8] = {
        0xffa54dca, 0xff182530, 0xffbb1d6d, 0xff132cde,
        0xffd6237b, 0xff2ed91e, 0xff3f721f, 0xffcb1971,
    };
    static const uint32_t rgb565[4] = {
        0x1926a279, 0x117bb8ed, 0x2ec3d10f, 0xc8ce3b83,
    };
    static const uint32_t argb1555[4] = {
        0x8c86d139, 0x88bbdc6d, 0x9763e88f, 0xe46e9dc3,
    };
    static const uint32_t yuv422[4] = {
        0x7d1839a5, 0xa51324bb, 0x4c2e7ed6, 0x48cb453f,
    };
    static const uint32_t mono[2] = { 0xde6d30ca, 0x711f1e7b };

    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, i < 8 ? src[i] : 0);
    }
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);

    frame_desc(qts, 3, 0xe410, 0);
    compose(qts);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, src[i]);
    }
    /* RGB888 keeps 0xff in the top byte, whatever the alpha */
    frame_desc(qts, 3 | 6 << 16, 0xe410, 0);
    compose(qts);
    for (int i = 0; i < 8; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, xrgb[i]);
    }
    frame_desc(qts, 3 | 1 << 16, 0xe410, 0);
    compose(qts);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, rgb565[i]);
    }
    /* RGB555 always sets the alpha bit */
    frame_desc(qts, 3 | 2 << 16, 0xe410, 0);
    compose(qts);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, argb1555[i]);
    }
    /* YUV422: R0, the pair's mean G, R1, its mean B */
    frame_desc(qts, 3 | 3 << 16, 0xe410, 0);
    compose(qts);
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, yuv422[i]);
    }
    /* one byte a pixel: the B */
    frame_desc(qts, 3 | 4 << 16, 0xe410, 0);
    compose(qts);
    for (int i = 0; i < 2; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, mono[i]);
    }
    qtest_quit(qts);
}

#define TLB_GLBC        0x3000
#define TLB_TLBA        0x3010
#define TLB_TLBC        0x3020
#define TLB_VPN         0x3030
#define TLB_TLBV        0x3040
#define TLB_STAT        0x3050
#define TLB_TABLE       0x02300000
#define TLB_SECOND      0x02301000
#define TLB_PAGE_A      0x02400000
#define TLB_PAGE_B      0x02510000

static void tlb_check(QTestState *qts, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, 0xff000000 | (i * 0x010203));
    }
}

/* A layer read through the TLB, as a T40NN's: 4 KiB pages in two levels */
static void test_wb_tlb(void)
{
    QTestState *qts = dpu_init();
    uint32_t st;

    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, 0xff000000 | (i * 0x010203));
        qtest_writel(qts, BUF1 + 4 * i, 0xff000000 | (i * 0x030201));
    }
    wr(qts, TLB_TLBV, 0x00010001);
    wr(qts, TLB_TLBA, TLB_TABLE);
    wr(qts, TLB_TLBC, 0xf);
    wr(qts, TLB_GLBC, 1);
    qtest_writel(qts, TLB_TABLE + 4 * 0x0c, TLB_SECOND | 1);
    qtest_writel(qts, TLB_SECOND + 4, BUF0 | 1);
    frame_desc(qts, 3, 0xe410, 0);

    wr(qts, TLB_VPN, 0xffffffff);
    g_assert_cmphex(rd(qts, TLB_VPN), ==, 0xfffff000);
    wr(qts, TLB_VPN, 0);

    /* the page read through the table, not the address itself */
    layer_desc(qts, LAY0, 0x00100010, 0x50000, 0x03001000, 0, 16, 0);
    compose(qts);
    tlb_check(qts, 256);

    /* the top 10 bits index the first table, and the low 12 are ignored */
    qtest_writel(qts, TLB_TABLE + 4 * 0x301, TLB_SECOND | 0xfff);
    qtest_writel(qts, TLB_SECOND + 8, BUF0 | 0xfff);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, 0xc0402000, 0, 16, 0);
    compose(qts);
    tlb_check(qts, 256);

    /* a layer over two pages that are not next to each other */
    for (int i = 0; i < 128; i++) {
        qtest_writel(qts, TLB_PAGE_A + 0xe00 + 4 * i,
                     0xff000000 | (i * 0x010203));
        qtest_writel(qts, TLB_PAGE_B + 4 * i,
                     0xff000000 | ((i + 128) * 0x010203));
    }
    qtest_writel(qts, TLB_SECOND + 12, TLB_PAGE_A | 1);
    qtest_writel(qts, TLB_SECOND + 16, TLB_PAGE_B | 1);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, 0x03003e00, 0, 16, 0);
    compose(qts);
    tlb_check(qts, 256);

    /* translations are cached until invalidated */
    qtest_writel(qts, TLB_SECOND + 4, BUF1 | 1);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, 0x03001000, 0, 16, 0);
    compose(qts);
    tlb_check(qts, 256);
    wr(qts, TLB_TLBC, 0xf);
    compose(qts);
    for (int i = 0; i < 256; i++) {
        g_assert_cmphex(wbpix(qts, i), ==, 0xff000000 | (i * 0x030201));
    }

    /* an entry without its valid bit: the composer waits, STAT and VPN */
    qtest_writel(qts, TLB_SECOND + 4, BUF0);
    wr(qts, TLB_TLBC, 0xf);
    wr(qts, TLB_STAT, 0);
    qtest_memset(qts, WB, 0, 1024);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, 0x03001000, 0, 16, 0);
    wr(qts, DC_CLR_ST, 0xffffffff);
    wr(qts, FRM_CFG_ADDR, DESC);
    wr(qts, FRM_CFG_CTRL, 1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, DC_ST), ==, 0x4021);
    g_assert_cmphex(rd(qts, TLB_STAT), ==, 8);
    g_assert_cmphex(rd(qts, TLB_VPN), ==, 0x03001000);
    g_assert_cmphex(wbpix(qts, 1), ==, 0);

    /* RETRY without a fix changes nothing, with one the frame ends slow */
    wr(qts, TLB_TLBC, 0x10);
    g_assert_cmphex(rd(qts, DC_ST), ==, 0x4021);
    qtest_writel(qts, TLB_SECOND + 4, BUF0 | 1);
    wr(qts, TLB_TLBC, 0x10);
    qtest_clock_step(qts, 1000 * 1000);
    st = rd(qts, DC_ST);
    g_assert_cmphex(st, ==, 0x8000c600);
    tlb_check(qts, 256);
    g_assert_cmphex(rd(qts, TLB_STAT), ==, 8);

    /* channel n flags bit 3 - n of STAT */
    wr(qts, TLB_TLBC, 0xf);
    wr(qts, TLB_STAT, 0);
    wr(qts, TLB_GLBC, 2);
    qtest_writel(qts, TLB_SECOND + 4, 0);
    layer_desc(qts, LAY1, 0x00100010, 0x50000, 0x03001000, 0, 16, 0);
    frame_desc(qts, 3, 0xe420, 0);
    wr(qts, DC_CLR_ST, 0xffffffff);
    wr(qts, FRM_CFG_ADDR, DESC);
    wr(qts, FRM_CFG_CTRL, 1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, DC_ST) & 1, ==, 1);
    g_assert_cmphex(rd(qts, TLB_STAT), ==, 4);
    g_assert_cmphex(rd(qts, TLB_VPN + 4), ==, 0x03001000);

    /* a quick stop leaves it working, until the channel's TLB goes off */
    wr(qts, DC_CLR_ST, 0xffffffff);
    wr(qts, DC_CTRL, 1);
    g_assert_cmphex(rd(qts, DC_ST), ==, 0x8001);
    wr(qts, TLB_TLBC, 0xf);
    g_assert_cmphex(rd(qts, DC_ST) & 1, ==, 1);
    wr(qts, TLB_GLBC, 0);
    g_assert_cmphex(rd(qts, DC_ST) & 1, ==, 0);

    /* RCI: the valid bits are not looked at */
    wr(qts, TLB_TLBV, 0x80010001);
    wr(qts, TLB_TLBC, 0xf);
    wr(qts, TLB_GLBC, 2);
    qtest_writel(qts, TLB_SECOND + 4, BUF1);
    wr(qts, DC_CLR_ST, 0xffffffff);
    wr(qts, FRM_CFG_ADDR, DESC);
    wr(qts, FRM_CFG_CTRL, 1);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, DC_ST) & 1, ==, 0);
    qtest_quit(qts);
}

/* The write-back dither on a flat 0x4c grey: its 4x4 thresholds */
static void test_wb_dither(void)
{
    QTestState *qts = dpu_init();
    static const struct {
        uint32_t ctrl;
        uint16_t px[2];     /* the pairs of a row, then of the next */
        uint16_t odd[2];
    } c[] = {
        /* RGB565, enabled, 4 bits dropped by all, then 3 */
        { 3 | 1 << 16 | 1 << 4 | 0x3f << 20, { 0x528a, 0x4208 },
          { 0x528a, 0x528a } },
        { 3 | 1 << 16 | 1 << 4 | 0x2a << 20, { 0x528a, 0x4a49 },
          { 0x4a49, 0x528a } },
        /* the auto bit alone: what the format drops, 3 bits and 2 for G */
        { 3 | 1 << 16 | 1 << 5, { 0x526a, 0x4a69 }, { 0x4a69, 0x526a } },
        /* RGB555 as an ARGB1555 */
        { 3 | 2 << 16 | 1 << 5, { 0xa94a, 0xa529 }, { 0xa529, 0xa94a } },
    };

    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, 0xff4c4c4c);
    }
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);
    for (int k = 0; k < ARRAY_SIZE(c); k++) {
        frame_desc(qts, c[k].ctrl, 0xe410, 0);
        compose(qts);
        for (int x = 0; x < 4; x++) {
            g_assert_cmphex(qtest_readw(qts, WB + 2 * x), ==, c[k].px[x & 1]);
            g_assert_cmphex(qtest_readw(qts, WB + 32 + 2 * x), ==,
                            c[k].odd[x & 1]);
            g_assert_cmphex(qtest_readw(qts, WB + 64 + 2 * x), ==,
                            c[k].px[x & 1]);
        }
    }
    qtest_quit(qts);
}

/*
 * The dither goes before the conversion to YUV422 and one byte a pixel too:
 * a flat 0x38 0x39 0x3a, 4 bits dropped (the first 8 bytes of each of four
 * rows, as a T40NN writes them back)
 */
static void test_wb_dither_yuv(void)
{
    QTestState *qts = dpu_init();
    static const struct {
        uint32_t fmt, stride;
        uint32_t w[4][2];
    } c[] = {
        { 3, 32, { { 0x38303840, 0x38303840 }, { 0x40404030, 0x38403830 },
                   { 0x38303840, 0x38303840 }, { 0x38403830, 0x40403830 } } },
        { 4, 16, { { 0x30403040, 0x30403040 }, { 0x40304040, 0x40304040 },
                   { 0x30403040, 0x30403040 }, { 0x40404030, 0x40404030 } } },
    };

    for (int i = 0; i < 256; i++) {
        qtest_writel(qts, BUF0 + 4 * i, 0xff38393a);
    }
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);
    for (int k = 0; k < ARRAY_SIZE(c); k++) {
        frame_desc(qts, 3 | c[k].fmt << 16 | 1 << 4 | 0x3f << 20, 0xe410, 0);
        compose(qts);
        for (int y = 0; y < 4; y++) {
            g_assert_cmphex(qtest_readl(qts, WB + y * c[k].stride), ==,
                            c[k].w[y][0]);
            g_assert_cmphex(qtest_readl(qts, WB + y * c[k].stride + 4), ==,
                            c[k].w[y][1]);
        }
        /* the auto bit alone drops nothing from these */
        frame_desc(qts, 3 | c[k].fmt << 16 | 1 << 5, 0xe410, 0);
        compose(qts);
        g_assert_cmphex(qtest_readb(qts, WB), ==, 0x38 + 2 * (c[k].fmt == 4));
    }
    qtest_quit(qts);
}

/* The scaler is a cubic over the source row: 2x a ramp, 1.5x a step */
static void test_wb_scale(void)
{
    QTestState *qts = dpu_init();
    static const uint8_t ramp2x[32] = {
        0, 7, 16, 24, 32, 40, 48, 56, 64, 72, 80, 88, 96, 104, 112, 120,
        128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224,
        233, 240, 241,
    };
    static const uint8_t step15[24] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 73, 254, 255, 255, 255, 255, 255,
        255, 255, 255, 255, 255, 255,
    };

    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
            qtest_writel(qts, BUF0 + 4 * (16 * y + x), 0xff000000 |
                         x * 16 * 0x010101);
            qtest_writel(qts, BUF1 + 4 * (16 * y + x), 0xff000000 |
                         (x < 8 ? 0 : 0xffffff));
        }
    }
    frame_desc(qts, 3, 0xe411, 0);
    qtest_writel(qts, DESC + 4, 32 << 16 | 32);
    qtest_writel(qts, DESC + 16, 32);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF0, 0, 16, 0);
    qtest_writel(qts, LAY0 + 12, 32 << 16 | 32);
    qtest_writel(qts, LAY0 + 28, 256);
    qtest_writel(qts, LAY0 + 32, 256);
    compose(qts);
    for (int i = 0; i < 32; i++) {
        g_assert_cmphex(wbpix(qts, i) & 0xff, ==, ramp2x[i]);
        g_assert_cmphex(wbpix(qts, 32 * 31 + i) & 0xff, ==, ramp2x[i]);
    }

    qtest_writel(qts, DESC + 4, 24 << 16 | 24);
    qtest_writel(qts, DESC + 16, 24);
    layer_desc(qts, LAY0, 0x00100010, 0x50000, BUF1, 0, 16, 0);
    qtest_writel(qts, LAY0 + 12, 24 << 16 | 24);
    qtest_writel(qts, LAY0 + 28, 341);
    qtest_writel(qts, LAY0 + 32, 341);
    compose(qts);
    for (int i = 0; i < 24; i++) {
        g_assert_cmphex(wbpix(qts, 24 * 12 + i) & 0xff, ==, step15[i]);
    }
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
    /* 16x16 RGB888 on a 24-bit bus, no TE wait: 2 + 1 clocks a pixel */
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
    g_assert_cmphex(rd(qts, LCDC_DMA_SITE), ==, BUF0 + 0x528);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 0x10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    g_assert_false(qtest_get_irq(qts, 20));
    qtest_clock_step(qts, 256 * 3 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    qtest_clock_step(qts, 10);
    /* the DMA_END of START was cleared and is not raised again */
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 2);
    g_assert_cmphex(rd(qts, LCDC_INT_FLAG), ==, 2);
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
    qtest_clock_step(qts, 256 * 3 * 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 6);
    g_assert_cmphex(rd(qts, LCDC_INT_FLAG), ==, 4);
    qtest_quit(qts);
}

/* 64x64 on an 8-bit bus, WR_DUTY 1: 3 * (2 + 2) + 1 clocks a pixel */
static void test_lcdc_continuous(void)
{
    QTestState *qts = lcdc_init();
    int64_t ns = 64 * 64 * 13 * 10;

    wr(qts, LCDC_GLB, 0x20012);
    wr(qts, SLCD_CFG, 0x85500d00);
    wr(qts, SLCD_DUTY, 0x01010101);
    wr(qts, SLCD_SIZE, 64 << 16 | 64);
    lcdc_desc(qts, 0x64, 0, 2);

    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    /* the DMA stalls with 30 bursts read: line 7's second */
    g_assert_cmphex(rd(qts, LCDC_DMA_SITE), ==, BUF0 + 0xb30);
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

/*
 * The T41's FIFO, size fields, stop acknowledges and 0xa01c, as a T41LQ and
 * a T41NQ have them (the FIFO's edge is within a few pixels on silicon)
 */
static void test_lcdc_t41(void)
{
    QTestState *qts = lcdc_init();

    wr(qts, 0xa01c, 0xffffffff);
    g_assert_cmphex(rd(qts, 0xa01c), ==, 1);
    wr(qts, LCDC_GLB, 0x20012);
    wr(qts, SLCD_CFG, 0x05500d20);

    /* 1920 bytes of 4-byte pixels fit the FIFO, one more pixel does not */
    wr(qts, SLCD_SIZE, 1 << 16 | 480);
    lcdc_desc(qts, 481, 1, 2);
    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 4);
    wr(qts, LCDC_CTRL, 2);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, SLCD_SIZE, 1 << 16 | 481);
    wr(qts, LCDC_CTRL, 1);
    /* a quick stop's acknowledge comes up again with START */
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 0x41);
    wr(qts, LCDC_CTRL, 2);
    wr(qts, LCDC_CSR, 0xffffffff);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 0);

    /* 10 bits a side: 16 + 1024 wide is 16, a height of 0 is 1024 lines */
    wr(qts, SLCD_SIZE, 1 << 16 | (1024 + 16));
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 16 * 3 * 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 2);
    wr(qts, LCDC_CTRL, 2);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, SLCD_SIZE, 2);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 2 * 1024 * 3 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 2);

    /* a width of 1024 never ends the frame */
    wr(qts, LCDC_CTRL, 2);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, SLCD_SIZE, 1 << 16 | 1024);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 1000000000);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_quit(qts);
}

/* The T21, T23 and T31's LCDC: the T41's block, run as a T23N and T31X do */
#define CPM_LPCDR       0x64

static QTestState *xb1_init(const char *machine, bool clock)
{
    QTestState *qts = qtest_initf("-machine %s", machine);

    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 24));
    if (clock) {
        /* 1200 MHz MPLL over 12: 100 MHz, latched by CE at bit 28 */
        qtest_writel(qts, CPM + CPM_CPMPCR, 0x0640510d);
        qtest_writel(qts, CPM + CPM_LPCDR, 1u << 30 | 1 << 28 | 11);
    }
    return qts;
}

static void test_xb1_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31");
    static const struct {
        uint32_t off, reset, ones;
    } r[] = {
        { 0x0000, 0, 0xffffffff }, { 0x0004, 0x12, 0x003f003f },
        { 0x0100, 0x066204a8, 0x07ff07ff }, { 0x010c, 0x00800000, 0x00ff001f },
        { 0x100c, 0, 0x7e }, { 0x1010, 0x03190842, 0x07ffffff },
        { 0x8000, 1, 0x3f0007 }, { 0x9000, 0, 0 }, { 0x9010, 0, 0 },
        { 0xa000, 0x01540d20, 0x8fff0f7f }, { 0xa004, 0, 0xffffffff },
        { 0xa010, 0, 0xffff }, { 0xa014, 0x80000000, 0xe0ffffff },
        { 0xa018, 0, 0 }, { 0xa01c, 0, 1 },
    };

    g_assert_cmphex(rd(qts, LCDC_GLB), ==, 0);
    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 24));
    /* out of reset the pixel clock is stopped: SLCD_CFG reads 0, no hang */
    g_assert_cmphex(rd(qts, DISP_COM), ==, 1);
    g_assert_cmphex(rd(qts, SLCD_CFG), ==, 0);
    /* LPCDR keeps bits 31:30, 28, 26 and 7:0; busy reads clear */
    qtest_writel(qts, CPM + CPM_LPCDR, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, CPM + CPM_LPCDR), ==, 0xd40000ff);
    qtest_writel(qts, CPM + CPM_CPMPCR, 0x0640510d);
    qtest_writel(qts, CPM + CPM_LPCDR, 1u << 30 | 1 << 28 | 11);
    for (int i = 0; i < ARRAY_SIZE(r); i++) {
        g_assert_cmphex(rd(qts, r[i].off), ==, r[i].reset);
        g_assert_cmphex(rd(qts, 0xa004), ==, 0);
        wr(qts, r[i].off, 0xffffffff);
        g_assert_cmphex(rd(qts, r[i].off), ==, r[i].ones);
        wr(qts, r[i].off, r[i].reset);
        g_assert_cmphex(rd(qts, 0xa004), ==, rd(qts, 0xa004));
    }
    g_assert_cmphex(rd(qts, 0xb000), ==, 1);
    qtest_quit(qts);
}

/* A T31X loses the first SLCD write after a read of the display registers */
static void test_xb1_quirk(void)
{
    QTestState *qts = xb1_init("ingenic-t31", true);

    /* armed out of reset: FMT_EN goes, the rest stays */
    wr(qts, SLCD_CFG, 0x05500d20);
    g_assert_cmphex(rd(qts, SLCD_CFG), ==, 0x01500d20);
    wr(qts, SLCD_DUTY, 0x1234);
    g_assert_cmphex(rd(qts, SLCD_DUTY), ==, 0x1234);
    /* a read of the display registers arms it again: the write is lost */
    g_assert_cmphex(rd(qts, DISP_COM), ==, 1);
    wr(qts, SLCD_DUTY, 0x5678);
    g_assert_cmphex(rd(qts, SLCD_DUTY), ==, 0x1234);
    wr(qts, SLCD_DUTY, 0x5678);
    g_assert_cmphex(rd(qts, SLCD_DUTY), ==, 0x5678);
    /* a read of an SLCD register disarms it, and so does a write */
    g_assert_cmphex(rd(qts, 0xb000), ==, 1);
    g_assert_cmphex(rd(qts, SLCD_CFG), ==, 0x01500d20);
    wr(qts, SLCD_DUTY, 0x9abc);
    g_assert_cmphex(rd(qts, SLCD_DUTY), ==, 0x9abc);
    g_assert_cmphex(rd(qts, 0x9000), ==, 0);
    wr(qts, 0xa01c, 1);
    wr(qts, SLCD_DUTY, 0xdef0);
    g_assert_cmphex(rd(qts, SLCD_DUTY), ==, 0xdef0);
    qtest_quit(qts);

    /* a T23N does not */
    qts = xb1_init("ingenic-t23", true);
    wr(qts, SLCD_CFG, 0x05500d20);
    g_assert_cmphex(rd(qts, SLCD_CFG), ==, 0x05500d20);
    g_assert_cmphex(rd(qts, DISP_COM), ==, 1);
    wr(qts, SLCD_DUTY, 0x1234);
    g_assert_cmphex(rd(qts, SLCD_DUTY), ==, 0x1234);
    qtest_quit(qts);
}

static void xb1_setup(QTestState *qts, uint32_t glb, uint32_t cfg,
                      uint32_t duty, uint32_t size, uint32_t stride)
{
    /* the SLCD registers' first write after a read of them, as the T31X */
    rd(qts, SLCD_CFG);
    wr(qts, LCDC_GLB, glb);
    wr(qts, SLCD_CFG, cfg);
    wr(qts, SLCD_DUTY, duty);
    wr(qts, SLCD_SIZE, size);
    wr(qts, LCDC_INTC, 0x7e);
    lcdc_desc(qts, stride, 1, 2);
}

static void test_xb1_frame(void)
{
    QTestState *qts = xb1_init("ingenic-t31", true);

    /*
     * 16x16 RGB888 on a 24-bit bus: a pixel takes (0 + 0 + 2) + 1 clocks,
     * and its DMA ran at START: DMA_END and DMA_SITE, then FRM_END only
     */
    xb1_setup(qts, 0x20012, 0x05500d20, 0, 16 << 16 | 16, 0x16);
    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 4);
    g_assert_cmphex(rd(qts, LCDC_DMA_SITE), ==, BUF0 + 0x528);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 0x10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    qtest_clock_step(qts, 256 * 3 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 2);

    /* a quick stop's acknowledge comes back with the next START */
    wr(qts, LCDC_CTRL, 2);
    g_assert_cmphex(rd(qts, LCDC_ST) & 0x40, ==, 0x40);
    wr(qts, LCDC_CSR, 0xffffffff);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 0);
    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 0x44);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 8);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 0x24);
    qtest_quit(qts);

    /*
     * 64x64: a frame past the FIFO is WORKING from START, and DMA_END and
     * FRM_END come at its end. An 8-bit bus takes three transfers of 2 + 2
     * clocks for RGB888, two for RGB565, and a 9-bit bus two.
     */
    qts = xb1_init("ingenic-t23", true);
    xb1_setup(qts, 0x20012, 0x05500d00, 0x01010000, 64 << 16 | 64, 0x40);
    wr(qts, LCDC_CTRL, 1);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 4096 * 13 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 1);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(rd(qts, LCDC_ST), ==, 6);

    xb1_setup(qts, 0x10012, 0x01000d00, 0x01010000, 64 << 16 | 64, 0x20);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 4096 * 9 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 2);

    /*
     * the size keeps 10 bits: a height of 0 is 1024 lines, a width of 0 or
     * 1024 never ends
     */
    xb1_setup(qts, 0x20012, 0x05500d20, 0, 16, 0x10);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 16 * 1024 * 3 * 10 - 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_clock_step(qts, 10);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 2);
    wr(qts, SLCD_SIZE, 1024 | 4 << 16);
    wr(qts, LCDC_CSR, 0xffffffff);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 1000 * 1000 * 1000);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_quit(qts);

    /* the pixel clock is stopped until a write latches it with CE */
    qts = xb1_init("ingenic-t31", false);
    xb1_setup(qts, 0x20012, 0x05500d20, 0, 16 << 16 | 16, 0x16);
    wr(qts, LCDC_CTRL, 1);
    wr(qts, LCDC_CTRL, 0x10);
    qtest_clock_step(qts, 1000 * 1000 * 1000);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_writel(qts, CPM + CPM_CPMPCR, 0x0640510d);
    qtest_writel(qts, CPM + CPM_LPCDR, 1u << 30 | 11);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 0);
    qtest_writel(qts, CPM + CPM_LPCDR, 1u << 30 | 1 << 28 | 11);
    qtest_clock_step(qts, 1000 * 1000);
    g_assert_cmphex(rd(qts, LCDC_ST) & 2, ==, 2);
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
    qtest_add_func("ingenic-dpu/wb/formats", test_wb_formats);
    qtest_add_func("ingenic-dpu/wb/dither", test_wb_dither);
    qtest_add_func("ingenic-dpu/wb/dither-yuv", test_wb_dither_yuv);
    qtest_add_func("ingenic-dpu/wb/scale", test_wb_scale);
    qtest_add_func("ingenic-dpu/wb/tlb", test_wb_tlb);
    qtest_add_func("ingenic-dpu/lcdc/regs", test_lcdc_regs);
    qtest_add_func("ingenic-dpu/lcdc/frame", test_lcdc_frame);
    qtest_add_func("ingenic-dpu/lcdc/continuous", test_lcdc_continuous);
    qtest_add_func("ingenic-dpu/lcdc/t41", test_lcdc_t41);
    qtest_add_func("ingenic-dpu/xb1/regs", test_xb1_regs);
    qtest_add_func("ingenic-dpu/xb1/quirk", test_xb1_quirk);
    qtest_add_func("ingenic-dpu/xb1/frame", test_xb1_frame);
    return g_test_run();
}
