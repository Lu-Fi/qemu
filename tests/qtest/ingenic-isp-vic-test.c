/*
 * QTest testcase for the T40's three VICs: their registers out of reset
 * and the bits a write keeps, as a T40XP's read; the raw capture each
 * VIC has (its MDMA), which the driver uses for the third sensor on VIC 2
 * (DVP, no pipeline behind it) and for a sensor's raw frames; and joint
 * mode, where the driver has both pipelines put their channel out into
 * one buffer, side by side or one above the other. The T31's and T32's
 * VICs capture raw as the T40's do (their drivers' raw snapshot), the
 * T32's MIPI data type at +0x24.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define VIC(n)          (ISP + 0x80000 + (n) * 0x10000)
#define VIC_STA2        0x1e4
#define VIC_CLR2        0x1f4
#define MDMA            0x300
#define BUF             0x00400000
#define W               64
#define H               32
#define FRAME_NS        40000000        /* fps=25 */

static QTestState *start(const char *sensors)
{
    QTestState *qts = qtest_initf("-machine ingenic-t40 %s", sensors);

    qtest_writel(qts, CPM_CLKGR0, 0);
    return qts;
}

static void test_regs(void)
{
    QTestState *qts = start("");

    for (unsigned n = 0; n < 3; n++) {
        uint32_t v = VIC(n);

        g_assert_cmphex(qtest_readl(qts, v + 0x004), ==, 0x07800438);
        g_assert_cmphex(qtest_readl(qts, v + 0x0c0), ==, 0x00eb8080);
        g_assert_cmphex(qtest_readl(qts, v + 0x1ac), ==, 0x00004444);
        g_assert_cmphex(qtest_readl(qts, v + 0x304), ==, 0x07800438);
        g_assert_cmphex(qtest_readl(qts, v + 0x310), ==, 0x00000f00);
        g_assert_cmphex(qtest_readl(qts, v + 0x3e0), ==, 0x00000081);
    }
    qtest_writel(qts, VIC(2) + 0x10c, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + 0x10c), ==, 0x03f7ff37);
    qtest_writel(qts, VIC(2) + 0x1c0, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + 0x1c0), ==, 0x13);
    qtest_writel(qts, VIC(2) + 0x1ec, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + 0x1ec), ==, 0xf);
    qtest_writel(qts, VIC(2) + 0x050, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + 0x050), ==, 0);
    /* the raw capture's: all bits but the strides', reset and done */
    qtest_writel(qts, VIC(2) + MDMA, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + MDMA), ==, 0xffffffff);
    qtest_writel(qts, VIC(2) + MDMA, 0);
    qtest_writel(qts, VIC(2) + MDMA + 0x10, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + MDMA + 0x10), ==, 0xffff);
    qtest_writel(qts, VIC(2) + MDMA + 0x18, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + MDMA + 0x18), ==, 0xffffffff);
    qtest_writel(qts, VIC(2) + MDMA + 0x08, 1);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + MDMA + 0x08), ==, 0);
    qtest_writel(qts, VIC(2) + MDMA + 0x80, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + MDMA + 0x80), ==, 0);
    qtest_writel(qts, VIC(2) + MDMA + 0x70, 1);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + MDMA + 0x70), ==, 0);
    qtest_writel(qts, VIC(2) + VIC_STA2, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + VIC_STA2), ==, 0);
    qtest_writel(qts, VIC(2) + 0x3e0, 0);
    g_assert_cmphex(qtest_readl(qts, VIC(2) + 0x3e0), ==, 0x81);
    qtest_quit(qts);
}

static uint16_t raw_at(QTestState *qts, uint32_t base, uint32_t x, uint32_t y)
{
    return qtest_readw(qts, base + y * W * 2 + x * 2);
}

/* The frame stamp's cell @c: 1 0, the input's 2 bits, the frame's 24 */
static bool stamp_cell(unsigned input, uint32_t frame, unsigned c)
{
    uint32_t bits = input << 24 | frame;

    if (c < 2) {
        return c == 0;
    }
    if (c < 28) {
        return bits >> (27 - c) & 1;
    }
    if (c == 28) {
        return __builtin_popcount(bits) & 1;
    }
    return c != 30;
}

/*
 * The third sensor, on VIC 2 through the DVP port: 12 bits a sample, its
 * frames raw into the five slots in turn, 16 bits a pixel, the sample in
 * the high bits, RGGB
 */
static void test_vic2_raw(void)
{
    QTestState *qts = start("-device image-sensor,model=jxq03,bus=i2c0,"
                            "input=2,fps=25,stamp=on,motion=off");
    uint32_t v = VIC(2), frame_size = W * H * 2;

    qtest_writel(qts, v + 0x00c, 3);                /* DVP */
    qtest_writel(qts, v + 0x010, 0x40000);          /* 12 bits */
    qtest_writel(qts, v + 0x004, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x08, 1);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA + 0x14, W * 2);
    for (unsigned i = 0; i < 5; i++) {
        qtest_writel(qts, v + MDMA + 0x18 + 4 * i, BUF + i * frame_size);
    }
    qtest_writel(qts, v + 0x1ec, 1);               /* masked, still read */
    qtest_writel(qts, v + MDMA, 0x80020020);        /* two frames */
    qtest_writel(qts, v, 1);

    /* the first frame, at once */
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, v + MDMA), ==, 0x80010020);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, v + 0x3a4), ==, 1);
    for (unsigned c = 0; c < 32; c++) {
        uint16_t want = stamp_cell(2, 1, c) ? 0xfff0 : 0;

        for (unsigned y = 0; y < 2; y++) {
            g_assert_cmphex(raw_at(qts, BUF, 2 * c, y), ==, want);
            g_assert_cmphex(raw_at(qts, BUF, 2 * c + 1, y), ==, want);
        }
    }
    /* input 2 sees the scene upside down: the grey ramp at the top */
    for (unsigned x = 2; x < W; x++) {
        long g = lround(1.164 * (x * 219 / W));
        uint16_t want = (MIN(g, 255) * 257) & 0xfff0;

        g_assert_cmphex(raw_at(qts, BUF, x, 4), ==, want);
        g_assert_cmphex(raw_at(qts, BUF, x, 5), ==, want);
    }
    /* the bars below: red, green and blue samples apart */
    g_assert_cmphex(raw_at(qts, BUF, 2 * W / 8 + 8, H - 2), !=,
                    raw_at(qts, BUF, 2 * W / 8 + 9, H - 2));
    qtest_writel(qts, v + VIC_CLR2, 1);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 0);

    /* the second into slot 1, then none */
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, BUF + frame_size);
    g_assert_cmphex(qtest_readl(qts, v + MDMA), ==, 0x80000020);
    g_assert_cmphex(raw_at(qts, BUF + frame_size, 52, 0), ==, 0xfff0);
    g_assert_cmphex(raw_at(qts, BUF + frame_size, 54, 0), ==, 0);
    qtest_writel(qts, v + VIC_CLR2, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, v + 0x3a4), ==, 3);

    /* every frame: round the slots, from slot 0 again after a reset */
    qtest_writel(qts, v + MDMA + 0x08, 1);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, 0);
    qtest_writel(qts, v + MDMA, 0x80080020);
    for (unsigned i = 0; i < 6; i++) {
        qtest_clock_step(qts, FRAME_NS);
        g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==,
                        BUF + i % 5 * frame_size);
    }
    g_assert_cmphex(qtest_readl(qts, v + MDMA), ==, 0x80080020);

    /* nothing reaches pipeline 0 */
    g_assert_cmphex(qtest_readl(qts, ISP + 0x40028), ==, 0);
    qtest_quit(qts);
}

/* RAW10 on MIPI keeps the 10 high bits */
static void test_raw10(void)
{
    QTestState *qts = start("-device image-sensor,model=jxq03,bus=i2c0,"
                            "fps=25,stamp=on,motion=off");
    uint32_t v = VIC(0);

    qtest_writel(qts, v + 0x014, 1);
    qtest_writel(qts, v + MDMA + 0x08, 1);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA + 0x18, BUF);
    qtest_writel(qts, v + MDMA, 0x80010020);
    qtest_writel(qts, v, 1);
    g_assert_cmphex(raw_at(qts, BUF, 0, 0), ==, 0xffc0);
    g_assert_cmphex(raw_at(qts, BUF, 2, 0), ==, 0);
    qtest_quit(qts);
}

/* An empty slot (0) takes no frame: the capture waits there, its count kept */
static void test_empty_slot(void)
{
    QTestState *qts = start("-device image-sensor,model=jxq03,bus=i2c0,"
                            "fps=25,stamp=on,motion=off");
    uint32_t v = VIC(0), frame_size = W * H * 2;

    qtest_writel(qts, v + MDMA + 0x08, 1);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA + 0x18, BUF);
    qtest_writel(qts, v + MDMA, 0x80020020);
    qtest_writel(qts, v, 1);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, BUF);
    qtest_writel(qts, v + VIC_CLR2, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 0);
    g_assert_cmphex(qtest_readl(qts, v + MDMA), ==, 0x80010020);
    qtest_writel(qts, v + MDMA + 0x1c, BUF + frame_size);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, BUF + frame_size);
    g_assert_cmphex(qtest_readl(qts, v + MDMA), ==, 0x80000020);
    qtest_quit(qts);
}

static void chan(QTestState *qts, unsigned pipe, uint32_t y, uint32_t uv,
                 uint32_t stride)
{
    uint32_t c = ISP + 0x16100 + pipe * 0x1000;

    qtest_writel(qts, c + 0x00, W << 16 | H);
    qtest_writel(qts, c + 0x80, stride);
    qtest_writel(qts, c + 0x98, stride);
    qtest_writel(qts, c + 0x6c, y);
    qtest_writel(qts, c + 0x84, uv);
    qtest_writel(qts, ISP + 0x16008 + pipe * 0x1000, 1);
}

/* Cell @c of a stamp's top row at @base: white or black */
static bool cell_at(QTestState *qts, uint32_t base, unsigned c)
{
    return qtest_readb(qts, base + 2 * c) > 128;
}

/*
 * Joint mode: the driver gives the second pipeline's channel the same
 * buffer as the first's, offset by the first's width (side by side, both
 * with the joint picture's stride) or by its height's rows (one above the
 * other)
 */
static void test_joint(const void *data)
{
    bool side = (uintptr_t)data;
    QTestState *qts = start("-device image-sensor,model=jxq03,bus=i2c0,"
                            "input=0,fps=25,stamp=on,motion=off "
                            "-device image-sensor,model=jxq03,bus=i2c1,"
                            "input=1,fps=25,stamp=on,motion=off");
    uint32_t stride = side ? 2 * W : W;
    uint32_t y1 = side ? W : W * H, uv = BUF + 0x10000;

    chan(qts, 0, BUF, uv, stride);
    chan(qts, 1, BUF + y1, uv + (side ? W : W * H / 2), stride);
    qtest_writel(qts, VIC(0), 1);
    qtest_writel(qts, VIC(1), 1);
    for (unsigned c = 0; c < 32; c++) {
        g_assert_cmpint(cell_at(qts, BUF, c), ==, stamp_cell(0, 1, c));
        g_assert_cmpint(cell_at(qts, BUF + y1, c), ==, stamp_cell(1, 1, c));
    }
    /* the first sensor's ramp rises left to right, the second's mirrored */
    g_assert_cmpuint(qtest_readb(qts, BUF + (H - 1) * stride + 8), <,
                     qtest_readb(qts, BUF + (H - 1) * stride + W - 8));
    g_assert_cmpuint(qtest_readb(qts, BUF + y1 + (H - 1) * stride + 8), >,
                     qtest_readb(qts, BUF + y1 + (H - 1) * stride + W - 8));
    qtest_quit(qts);
}

typedef struct RawCase {
    const char *machine;
    uint32_t vic, ctrl;                 /* the VIC, its run register */
    uint32_t mode, mode_val;            /* its input */
    uint32_t fmt, fmt_val;              /* its sample format */
    uint16_t keep;                      /* the sample's bits, at the top */
    uint32_t csi[2];                    /* CSI hosts to let out of reset */
} RawCase;

static const RawCase raw_t31 = {
    "-machine ingenic-t31", ISP + 0xe0000, 0x000,
    0x00c, 3, 0x010, 0x40000, 0xfff0,               /* DVP, 12 bits */
};
static const RawCase raw_t32 = {
    "-machine ingenic-t32", ISP + 0x80000, 0x010,
    0x01c, 2, 0x024, 1, 0xffc0,                     /* MIPI, RAW10 */
    { 0x10054000, 0x10023000 },
};

static void test_raw_other(const void *data)
{
    const RawCase *c = data;
    QTestState *qts = qtest_initf("%s -device image-sensor,model=jxq03,"
                                  "bus=i2c0,fps=25,stamp=on,motion=off",
                                  c->machine);
    uint32_t v = c->vic;

    qtest_writel(qts, CPM_CLKGR0, 0);
    for (unsigned i = 0; i < ARRAY_SIZE(c->csi) && c->csi[i]; i++) {
        qtest_writel(qts, c->csi[i] + 0x0c, 1);     /* DPHY_RSTZ */
        qtest_writel(qts, c->csi[i] + 0x10, 1);     /* CSI2_RESETN */
    }
    qtest_writel(qts, v + c->mode, c->mode_val);
    qtest_writel(qts, v + c->fmt, c->fmt_val);
    qtest_writel(qts, v + MDMA + 0x08, 1);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA + 0x18, BUF);
    qtest_writel(qts, v + MDMA, 0x80010020);        /* one frame */
    qtest_writel(qts, v + c->ctrl, 1);
    qtest_clock_step(qts, 2 * FRAME_NS);

    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, v + MDMA), ==, 0x80000020);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 1);
    for (unsigned cell = 0; cell < 32; cell++) {
        uint16_t want = stamp_cell(0, 1, cell) ? c->keep : 0;

        g_assert_cmphex(raw_at(qts, BUF, 2 * cell, 0), ==, want);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-isp-vic/t40/regs", test_regs);
    qtest_add_func("ingenic-isp-vic/t40/vic2-raw", test_vic2_raw);
    qtest_add_func("ingenic-isp-vic/t40/raw10", test_raw10);
    qtest_add_func("ingenic-isp-vic/t40/empty-slot", test_empty_slot);
    qtest_add_data_func("ingenic-isp-vic/t31/raw", &raw_t31, test_raw_other);
    qtest_add_data_func("ingenic-isp-vic/t32/raw", &raw_t32, test_raw_other);
    qtest_add_data_func("ingenic-isp-vic/t40/joint-side", (void *)1,
                        test_joint);
    qtest_add_data_func("ingenic-isp-vic/t40/joint-above", (void *)0,
                        test_joint);
    return g_test_run();
}
