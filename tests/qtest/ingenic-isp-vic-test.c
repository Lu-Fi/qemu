/*
 * QTest testcase for the T40's three VICs: their registers out of reset
 * and the bits a write keeps, as a T40XP's read; the raw capture each
 * VIC has (its MDMA), which the driver uses for the third sensor on VIC 2
 * (DVP, no pipeline behind it) and for a sensor's raw frames; and joint
 * mode, where the driver has both pipelines put their channel out into
 * one buffer, side by side or one above the other. The T31's and T32's
 * VICs capture raw as the T40's do (their drivers' raw snapshot), the
 * T32's MIPI data type at +0x24. The T41's driver leaves the count 0 and
 * queues each buffer with a write of 1 to +0x370: a frame per write. The
 * T41 also has a raw WDMA that takes the VIC's frames before the ISP (and
 * puts out the IR channel's frame, 0 for a sensor with no IR pixels), and
 * an RDMA that feeds raw frames back to the core in place of the VIC's, and
 * whose frames the statistics blocks measure.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/bswap.h"
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
#define T41_FIRST_NS    135000000       /* a T41NQ's first frame */

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

/*
 * The T32 Pro's raw capture takes its buffers from a queue (+0x20), one
 * a frame, and the filled ones come back in order from +0x28, 0 once all
 * are read; a cleared queue takes no frame
 */
static void test_t32pro_raw(void)
{
    const RawCase *c = &raw_t32;
    QTestState *qts = qtest_initf("-machine ingenic-t32 "
                                  "-global ingenic-t31.soc-variant=t32prozn "
                                  "-device image-sensor,model=jxq03,bus=i2c0,"
                                  "fps=25,stamp=on,motion=off");
    uint32_t v = c->vic, buf2 = BUF + 0x10000, buf3 = BUF + 0x20000;

    qtest_writel(qts, CPM_CLKGR0, 0);
    for (unsigned i = 0; i < ARRAY_SIZE(c->csi) && c->csi[i]; i++) {
        qtest_writel(qts, c->csi[i] + 0x0c, 1);     /* DPHY_RSTZ */
        qtest_writel(qts, c->csi[i] + 0x10, 1);     /* CSI2_RESETN */
    }
    qtest_writel(qts, v + c->mode, c->mode_val);
    qtest_writel(qts, v + c->fmt, c->fmt_val);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA + 0x20, BUF);
    qtest_writel(qts, v + MDMA + 0x20, buf2);
    qtest_writel(qts, v + MDMA, 0x80020000);        /* two frames */
    qtest_writel(qts, v + c->ctrl, 1);
    qtest_clock_step(qts, 4 * FRAME_NS);

    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 3, ==, 1);
    for (unsigned cell = 0; cell < 32; cell++) {
        g_assert_cmphex(raw_at(qts, BUF, 2 * cell, 0), ==,
                        stamp_cell(0, 1, cell) ? c->keep : 0);
        g_assert_cmphex(raw_at(qts, buf2, 2 * cell, 0), ==,
                        stamp_cell(0, 2, cell) ? c->keep : 0);
    }
    g_assert_cmphex(qtest_readl(qts, v + MDMA) >> 16 & 15, ==, 0);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x28), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x28), ==, buf2);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x28), ==, 0);

    /* a queue the driver clears takes no frame */
    qtest_writel(qts, v + MDMA + 0x20, buf3);
    qtest_writel(qts, v + MDMA + 0x48, 1);
    qtest_writel(qts, v + MDMA, 0x80010000);
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_cmphex(raw_at(qts, buf3, 0, 0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x28), ==, 0);
    qtest_quit(qts);
}

/* The T41's raw capture, a frame for each buffer the driver queues */
static void test_raw_kick(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device image-sensor,"
                                 "model=jxq03,bus=i2c0,fps=25,stamp=on,"
                                 "motion=off");
    uint32_t v = VIC(0), buf2 = BUF + W * H * 2;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, 0x10023000 + 0x0c, 1);        /* DPHY_RSTZ */
    qtest_writel(qts, 0x10023000 + 0x10, 1);        /* CSI2_RESETN */
    qtest_writel(qts, v + 0x00c, 2);                /* MIPI */
    qtest_writel(qts, v + 0x014, 1);                /* RAW10 */
    qtest_writel(qts, v + MDMA + 0x18, BUF);
    qtest_writel(qts, v + MDMA + 0x1c, buf2);
    qtest_writel(qts, v + MDMA + 0x08, 1);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA, 0x80000020);        /* on, count 0 */
    qtest_writel(qts, v + MDMA + 0x70, 1);          /* one buffer */
    qtest_writel(qts, v + 0x000, 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS);

    /* one frame, into the first slot; then nothing until the next buffer */
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 1);
    g_assert_cmphex(qtest_readw(qts, buf2), ==, 0);
    qtest_writel(qts, v + VIC_CLR2, 1);
    qtest_writel(qts, v + MDMA + 0x70, 1);
    qtest_clock_step(qts, 2 * FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, v + MDMA + 0x80), ==, buf2);
    g_assert_cmphex(qtest_readl(qts, v + VIC_STA2) & 1, ==, 1);
    g_assert_cmphex(raw_at(qts, buf2, W / 2, H / 2), !=, 0);
    qtest_quit(qts);
}

#define T41_WDMA        (ISP + 0x20000)
#define T41_RDMA        (ISP + 0x21000)
#define T41_DMA_EN      (ISP + 0x400b0)
#define T41_DMA_STA     (ISP + 0x400b8)
#define T41_INT_STA     (ISP + 0x40028)
#define T41_VIC_HOLD    (VIC(0) + 0x1a8)

static QTestState *t41_start(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device image-sensor,"
                                 "model=jxq03,bus=i2c0,fps=25,stamp=on,"
                                 "motion=off");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, 0x10023000 + 0x0c, 1);        /* DPHY_RSTZ */
    qtest_writel(qts, 0x10023000 + 0x10, 1);        /* CSI2_RESETN */
    qtest_writel(qts, VIC(0) + 0x00c, 2);           /* MIPI */
    qtest_writel(qts, VIC(0) + 0x014, 1);           /* RAW10 */
    qtest_writel(qts, VIC(0) + 0x004, W << 16 | H);
    qtest_writel(qts, T41_DMA_EN, 0x3f);
    return qts;
}

/* The raw WDMA: a frame into each queued buffer, 16 bits a sample, then done */
static void test_raw_wdma(void)
{
    QTestState *qts = t41_start();
    uint32_t buf2 = BUF + W * H * 2;

    g_assert_cmphex(qtest_readl(qts, T41_WDMA), ==, 0x20211118);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x4001);
    qtest_writel(qts, T41_WDMA + 0x14, 0x10020);    /* RAW16 */
    qtest_writel(qts, T41_WDMA + 0x1c, W * 2);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, T41_WDMA + 0x18, buf2);
    qtest_writel(qts, T41_WDMA + 0x04, 1);
    /* the start reads 0, the queue 2 deep, the next buffer at +0x18 */
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x04), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x2200);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x18), ==, BUF);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS / 2);

    /* the first frame's end: done bit 4, with the core's 0x400a8 and A bits */
    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x30, ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, T41_INT_STA) & 0x40000, ==, 0x40000);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x3100);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x18), ==, buf2);
    for (unsigned c = 0; c < 32; c++) {
        uint16_t want = stamp_cell(0, 1, c) ? 0xffc0 : 0;

        g_assert_cmphex(raw_at(qts, BUF, 2 * c, 0), ==, want);
    }
    g_assert_cmphex(raw_at(qts, buf2, 2, 0), ==, 0);
    /* the done FIFO: the first buffer, then nothing yet */
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x20), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x20), ==, 0);

    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x4001);
    g_assert_cmphex(raw_at(qts, buf2, 0, 0), ==, 0xffc0);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x20), ==, buf2);
    /* none queued: the third frame goes nowhere */
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x18), ==, buf2);
    qtest_quit(qts);
}

/* RAW8 keeps the 8 high bits; a held VIC sends the WDMA nothing */
static void test_raw_wdma8(void)
{
    QTestState *qts = t41_start();

    qtest_writel(qts, T41_WDMA + 0x14, 0x20);
    qtest_writel(qts, T41_WDMA + 0x1c, W + 16);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, T41_WDMA + 0x04, 1);
    qtest_writel(qts, T41_VIC_HOLD, 0x4444);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x3100);
    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x10, ==, 0);

    qtest_writel(qts, T41_VIC_HOLD, 0x4404);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x4001);
    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x10, ==, 0x10);
    /* the stamp's first cell is 2 samples wide; the next row a stride on */
    g_assert_cmphex(qtest_readb(qts, BUF), ==, 0xff);
    g_assert_cmphex(qtest_readb(qts, BUF + 1), ==, 0xff);
    g_assert_cmphex(qtest_readb(qts, BUF + 2), ==, 0);
    g_assert_cmphex(qtest_readb(qts, BUF + W), ==, 0);
    g_assert_cmphex(qtest_readb(qts, BUF + W + 16), ==, 0xff);
    qtest_quit(qts);
}

/* The IR channel's frame: the WDMA, a byte a sample, 0 with no IR pixels */
static void test_ir(void)
{
    QTestState *qts = t41_start();

    qtest_memset(qts, BUF, 0xa5, 2 * W * H);
    qtest_writel(qts, T41_WDMA + 0x14, 0xa2);
    qtest_writel(qts, T41_WDMA + 0x1c, W);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS / 2);

    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x10, ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x20), ==, BUF);
    for (uint32_t i = 0; i < W * H; i += 16) {
        g_assert_cmphex(qtest_readb(qts, BUF + i), ==, 0);
    }
    /* the frame is W x H bytes: what follows it stays */
    g_assert_cmphex(qtest_readb(qts, BUF + W * H), ==, 0xa5);
    qtest_quit(qts);
}

/*
 * The raw loop: the WDMA's frame goes to the guest, which hands a frame of
 * its own to the RDMA; the core takes only that, and its channel shows that
 * picture
 */
static void test_raw_loop(void)
{
    QTestState *qts = t41_start();
    uint32_t y = BUF + 0x10000, uv = BUF + 0x11000, ch = ISP + 0xf0100;
    uint32_t buf2 = BUF + W * H * 2;

    qtest_writel(qts, ch + 0x00, W << 16 | H);
    qtest_writel(qts, ch + 0x80, W);
    qtest_writel(qts, ch + 0x98, W);
    qtest_writel(qts, ch + 0x6c, y);
    qtest_writel(qts, ch + 0x84, uv);
    qtest_writel(qts, ISP + 0xf0008, 1);
    qtest_writel(qts, T41_WDMA + 0x14, 0x10020);
    qtest_writel(qts, T41_WDMA + 0x1c, W * 2);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, T41_WDMA + 0x18, buf2);
    qtest_writel(qts, T41_WDMA + 0x04, 1);
    qtest_writel(qts, T41_RDMA + 0x08, 1);
    qtest_writel(qts, T41_RDMA + 0x14, W * 2);
    qtest_writel(qts, T41_RDMA + 0x24, 0x2a);
    qtest_writel(qts, T41_RDMA + 0x04, 1);
    g_assert_cmphex(qtest_readl(qts, T41_RDMA + 0x04), ==, 0);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS / 2);

    /* the VIC's frame is the WDMA's alone: the channel has none */
    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x30, ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf017c) >> 8, ==, 0);
    g_assert_cmphex(qtest_readb(qts, y), ==, 0);

    /* a flat mid-grey frame in the WDMA's buffer, given to the RDMA */
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x20), ==, BUF);
    for (uint32_t i = 0; i < W * H; i++) {
        qtest_writew(qts, BUF + 2 * i, 0x8000);
    }
    qtest_writel(qts, T41_RDMA + 0x0c, BUF);
    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x20, ==, 0x20);
    g_assert_cmphex(qtest_readl(qts, T41_RDMA + 0x1c), ==, BUF);
    g_assert_cmphex(qtest_readl(qts, T41_RDMA + 0x1c), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf017c) >> 8, ==, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf0174), ==, y);
    /* 128 in each colour: luma 16 + 220 / 2, no chroma */
    for (uint32_t i = 0; i < W * H; i += 37) {
        g_assert_cmphex(qtest_readb(qts, y + i), ==, 126);
    }
    g_assert_cmphex(qtest_readb(qts, uv), ==, 128);
    g_assert_cmphex(qtest_readb(qts, uv + 1), ==, 128);

    /* the guest returns the buffer; the next VIC frame is written there */
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_clock_step(qts, FRAME_NS);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, T41_WDMA + 0x40), ==, 0x4001);
    g_assert_cmphex(raw_at(qts, BUF, 0, 0), ==, 0xffc0);

    /* mode 0 ends it: the VIC's frames reach the core, the stamp's white */
    qtest_writel(qts, T41_RDMA + 0x24, 0);
    qtest_writel(qts, ch + 0x6c, y + 0x2000);
    qtest_writel(qts, ch + 0x84, uv + 0x2000);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf017c) >> 8, ==, 1);
    g_assert_cmphex(qtest_readb(qts, y + 0x2000), >, 200);
    qtest_quit(qts);
}

#define STAT_AE_Z       0x00600000
#define STAT_AE_H       0x00601000
#define STAT_AWB        0x00602000
#define STAT_ADR        0x00610000
#define STAT_AF         0x00611000

static uint32_t bits_at(const uint32_t *w, unsigned at, unsigned n)
{
    uint64_t x = le32_to_cpu(w[at / 32]) |
                 (uint64_t)le32_to_cpu(w[at / 32 + 1]) << 32;

    return (x >> (at % 32)) & ((1ull << n) - 1);
}

/*
 * The T41's statistics blocks on the 64 x 32 frame (32 x 16 quads): AE and
 * AWB in 2 x 2 zones, one ADR and one AF buffer, as tisp_init leaves them
 */
static void t41_stats_setup(QTestState *qts)
{
    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    for (uint32_t b = ISP + 0x18000; b <= ISP + 0x19000; b += 0x1000) {
        qtest_writel(qts, b + 0x04, 2u << 28 | 2 << 12);
        qtest_writel(qts, b + 0x08, 0x1010);
        qtest_writel(qts, b + 0x18, 0x0808);
    }
    qtest_writel(qts, ISP + 0x19028, 0x0011aa1e);       /* 30, 170 */
    qtest_writel(qts, ISP + 0x1902c, STAT_AE_Z);
    qtest_writel(qts, ISP + 0x1903c, STAT_AE_H);
    qtest_writel(qts, ISP + 0x1904c, 0);                /* one buffer each */
    qtest_writel(qts, ISP + 0x1804c, 0);
    qtest_writel(qts, ISP + 0x18038, 0x01e6800a);
    qtest_writel(qts, ISP + 0x1803c, STAT_AWB);
    qtest_writel(qts, ISP + 0x19000, 1);
    qtest_writel(qts, ISP + 0x18000, 1);
    qtest_writel(qts, ISP + 0x9514, STAT_ADR);
    qtest_writel(qts, ISP + 0x1a004, 1u << 28 | 1 << 12);
    qtest_writel(qts, ISP + 0x1a008, W);
    qtest_writel(qts, ISP + 0x1a028, H);
    qtest_writel(qts, ISP + 0x1a0cc, STAT_AF);
    qtest_writel(qts, ISP + 0x1a0dc, 0);
    qtest_writel(qts, ISP + 0x9510, 0);
}

/* The AE histogram: the bins in use, and the quads in them */
static unsigned ae_hist(QTestState *qts, uint32_t *total, uint32_t *peak)
{
    uint32_t h[256], most = 0;
    unsigned bins = 0;

    qtest_memread(qts, STAT_AE_H, h, sizeof(h));
    *total = 0;
    *peak = 0;
    for (unsigned i = 0; i < 256; i++) {
        uint32_t v = le32_to_cpu(h[i]) & 0x1fffff;

        bins += v != 0;
        *total += v;
        if (v > most) {
            most = v;
            *peak = i;
        }
    }
    return bins;
}

/* Zone @z's three luma sums (dark, middle, bright quads) added up */
static uint32_t ae_zone(QTestState *qts, unsigned z)
{
    uint32_t w[16];

    qtest_memread(qts, STAT_AE_Z, w, sizeof(w));
    return bits_at(w, z * 128, 22) + bits_at(w, z * 128 + 22, 22) +
           bits_at(w, z * 128 + 44, 22);
}

/* The AWB records of all quads, bands and zones added up: colour @c, or n */
static uint32_t awb_sum(QTestState *qts, unsigned c)
{
    uint32_t w[4 * 4 * 8], sum = 0;

    qtest_memread(qts, STAT_AWB, w, sizeof(w));
    for (unsigned rec = 0; rec < 16; rec++) {
        sum += c < 3 ? bits_at(w + rec * 8, c * 22, 22) :
                       bits_at(w + rec * 8, 88, 14);
    }
    return sum;
}

static uint32_t adr_bins(QTestState *qts, uint32_t *total)
{
    uint32_t w[1024];
    unsigned bins = 0;

    qtest_memread(qts, STAT_ADR, w, sizeof(w));
    *total = 0;
    for (unsigned i = 0; i < 512; i++) {
        uint32_t v = bits_at(w + 2 + 24 * 14, i / 3 * 64 + i % 3 * 21, 21);

        bins += v != 0;
        *total += v;
    }
    return bins;
}

static uint32_t af_ysum(QTestState *qts)
{
    uint32_t w[8];

    qtest_memread(qts, STAT_AF, w, sizeof(w));
    return bits_at(w, 92, 24);
}

/*
 * A 64 x 32 RGGB frame in RAW16: 0 a flat 0x8000, 1 a flat 0x4000, 2 a colour
 * cast, 3 a gradient
 */
static void raw_frame(QTestState *qts, uint32_t addr, unsigned what)
{
    uint16_t row[W];

    for (unsigned y = 0; y < H; y++) {
        for (unsigned x = 0; x < W; x++) {
            uint16_t v;

            switch (what) {
            case 0:
                v = 0x8000;
                break;
            case 1:
                v = 0x4000;
                break;
            case 2:
                v = (y & 1) ? ((x & 1) ? 0x4000 : 0x8000) :
                              ((x & 1) ? 0x8000 : 0xc000);
                break;
            default:
                v = x * 0xff0 / W << 4;
                break;
            }
            row[x] = cpu_to_le16(v);
        }
        qtest_memwrite(qts, addr + y * W * 2, row, sizeof(row));
    }
}

/*
 * The statistics of a frame the RDMA reads are the buffer's: the sensor's
 * scene before it, then a flat grey, a darker one, a colour cast and a
 * gradient, each seen by AE, AWB, ADR and AF
 */
static void test_raw_stats(void)
{
    QTestState *qts = t41_start();
    uint32_t total, peak, adr, ys;
    uint32_t buf2 = BUF + W * H * 2, quads = W * H / 4, flat_peak, flat_ys;
    uint32_t r, g, b;

    t41_stats_setup(qts);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS / 2);

    /* the VIC's frame reaches the core: the scene's bars and ramp */
    g_assert_cmpuint(ae_hist(qts, &total, &peak), >, 4);
    g_assert_cmpuint(total, ==, quads);
    g_assert_cmpuint(adr_bins(qts, &adr), >, 4);

    qtest_writel(qts, T41_WDMA + 0x14, 0x10020);
    qtest_writel(qts, T41_WDMA + 0x1c, W * 2);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, T41_WDMA + 0x18, buf2);
    qtest_writel(qts, T41_WDMA + 0x04, 1);
    qtest_writel(qts, T41_RDMA + 0x08, 1);
    qtest_writel(qts, T41_RDMA + 0x14, W * 2);
    qtest_writel(qts, T41_RDMA + 0x24, 0x2a);
    qtest_writel(qts, T41_RDMA + 0x04, 1);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, T41_DMA_STA) & 0x30, ==, 0x10);
    /* the WDMA's frame is not the core's: still the scene's */
    g_assert_cmpuint(ae_hist(qts, &total, &peak), >, 4);

    /* a flat grey: one level in each, and no colour in AWB */
    raw_frame(qts, BUF, 0);
    qtest_writel(qts, T41_RDMA + 0x0c, BUF);
    g_assert_cmpuint(ae_hist(qts, &total, &peak), ==, 1);
    g_assert_cmpuint(total, ==, quads);
    flat_peak = peak;
    for (unsigned z = 0; z < 4; z++) {
        g_assert_cmpuint(ae_zone(qts, z), ==, peak * quads / 4);
    }
    r = awb_sum(qts, 0);
    g_assert_cmpuint(awb_sum(qts, 3), ==, quads);
    g_assert_cmpuint(r, ==, awb_sum(qts, 1));
    g_assert_cmpuint(r, ==, awb_sum(qts, 2));
    g_assert_cmpuint(adr_bins(qts, &adr), ==, 1);
    g_assert_cmpuint(adr, ==, quads);
    flat_ys = af_ysum(qts);
    g_assert_cmpuint(flat_ys, >, 0);

    /* a darker one: every statistic lower */
    qtest_writel(qts, T41_RDMA + 0x1c, 0);
    raw_frame(qts, buf2, 1);
    qtest_writel(qts, T41_RDMA + 0x0c, buf2);
    g_assert_cmpuint(ae_hist(qts, &total, &peak), ==, 1);
    g_assert_cmpuint(peak, <, flat_peak);
    g_assert_cmpuint(awb_sum(qts, 1), <, r);
    ys = af_ysum(qts);
    g_assert_cmpuint(ys, <, flat_ys);

    /* a colour cast: red over green over blue, as the samples are */
    raw_frame(qts, BUF, 2);
    qtest_writel(qts, T41_RDMA + 0x0c, BUF);
    r = awb_sum(qts, 0);
    g = awb_sum(qts, 1);
    b = awb_sum(qts, 2);
    g_assert_cmpuint(r, >, 2 * g);
    g_assert_cmpuint(g, >, 3 * b);
    g_assert_cmpuint(awb_sum(qts, 3), ==, quads);
    g_assert_cmpuint(adr_bins(qts, &adr), ==, 1);

    /* a gradient: many levels, and the right zones the brighter */
    raw_frame(qts, buf2, 3);
    qtest_writel(qts, T41_RDMA + 0x0c, buf2);
    g_assert_cmpuint(ae_hist(qts, &total, &peak), >, 8);
    g_assert_cmpuint(ae_zone(qts, 0), <, ae_zone(qts, 1));
    g_assert_cmpuint(ae_zone(qts, 2), <, ae_zone(qts, 3));
    g_assert_cmpuint(ae_zone(qts, 1), >, 2 * ae_zone(qts, 0));
    g_assert_cmpuint(adr_bins(qts, &adr), >, 8);

    /* mode 0 ends it: the VIC's frames are the core's, the scene's again */
    qtest_writel(qts, T41_RDMA + 0x24, 0);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmpuint(ae_hist(qts, &total, &peak), >, 4);
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
    qtest_add_func("ingenic-isp-vic/t32pro/raw", test_t32pro_raw);
    qtest_add_func("ingenic-isp-vic/t41/raw-kick", test_raw_kick);
    qtest_add_func("ingenic-isp-vic/t41/raw-wdma", test_raw_wdma);
    qtest_add_func("ingenic-isp-vic/t41/raw-wdma8", test_raw_wdma8);
    qtest_add_func("ingenic-isp-vic/t41/ir", test_ir);
    qtest_add_func("ingenic-isp-vic/t41/raw-loop", test_raw_loop);
    qtest_add_func("ingenic-isp-vic/t41/raw-stats", test_raw_stats);
    qtest_add_data_func("ingenic-isp-vic/t40/joint-side", (void *)1,
                        test_joint);
    qtest_add_data_func("ingenic-isp-vic/t40/joint-above", (void *)0,
                        test_joint);
    return g_test_run();
}
