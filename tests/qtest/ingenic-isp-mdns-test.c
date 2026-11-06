/*
 * QTest testcase for the T23's and T31's 3D noise reducer (MDNS)
 *
 * Its reads as a T23N and a T31AL read them, and its buffers as they fill
 * them with libimp's settings: the reference frame, rewritten each frame,
 * and the block ages in the statistics buffers (two 4-bit ages a byte, the
 * first buffer from its second row), 15 where the scene stands still and
 * 0 where it moves; and the reference's noise between frames falling with
 * the reduction on.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"

#define ISP             0x13300000
#define MDNS            (ISP + 0x7800)
#define CPM_CLKGR0      0x10000020
#define W               256
#define H               128
#define BW              (W / 16)
#define BH              (H / 16)
#define STA_STRIDE      8               /* ((W + 31) >> 5) + 7 & ~7 */
#define STA_ROWS        (BH + 1)
#define REF_Y           0x02000000
#define REF_UV          (REF_Y + W * H)
#define STA0            (REF_UV + W * H / 2)
#define STA_SIZE        (STA_STRIDE * STA_ROWS)
#define FRAME_NS        (1000000000 / 30)

/* The buffers and settings libimp leaves a T23N streaming with (temper 128) */
static void mdns_setup(QTestState *qts, bool shared)
{
    qtest_writel(qts, MDNS + 0x20, REF_Y);
    qtest_writel(qts, MDNS + 0x24, W);
    qtest_writel(qts, MDNS + 0x28, REF_UV);
    qtest_writel(qts, MDNS + 0x2c, W);
    for (int k = 0; k < 4; k++) {
        static const uint32_t at[4] = { 0x30, 0x40, 0x48, 0x50 };

        qtest_writel(qts, MDNS + at[k], STA0 + (shared ? 0 : k * STA_SIZE));
        qtest_writel(qts, MDNS + at[k] + 4, STA_STRIDE);
    }
    for (int k = 0; k < 4; k++) {
        /* columns, rows: the second and fourth grids a row fewer */
        qtest_writel(qts, MDNS + 0x74 + k * 0x18,
                     BW << 24 | (BH - (k & 1)) << 16 | 0x1010);
        qtest_writel(qts, MDNS + 0x78 + k * 0x18, 0x14f0 | !(k % 3));
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xff0500a4);
    }
    qtest_writel(qts, MDNS + 0x144, 0xff053053);
    qtest_writel(qts, MDNS + 0x148, 0xc8b4a000);
    qtest_writel(qts, MDNS + 0x14c, 0xfafaf0dc);
    qtest_writel(qts, MDNS + 0x150, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x154, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x264, 0xff173141);
    qtest_writel(qts, MDNS + 0x10, 0x11111111);
    qtest_writel(qts, MDNS + 0x14, 0x00f01100);
    qtest_writel(qts, MDNS + 0x04, 0x111);
}

static QTestState *mdns_start(const char *machine, const char *sensor,
                              bool shared)
{
    QTestState *qts = qtest_initf("-machine %s -device "
                                  "image-sensor,model=jxq03,bus=i2c0,id=cam%s",
                                  machine, sensor);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    mdns_setup(qts, shared);
    return qts;
}

static void mdns_frames(QTestState *qts, int n)
{
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS * n + FRAME_NS / 2);
}

/* A block's age in a statistics buffer whose rows start at row @top */
static unsigned mdns_age(const uint8_t *sta, unsigned top, unsigned bx,
                         unsigned by)
{
    return sta[(by + top) * STA_STRIDE + bx / 2] >> (bx & 1) * 4 & 15;
}

static double ref_noise(QTestState *qts)
{
    g_autofree uint8_t *a = g_malloc(W * H), *b = g_malloc(W * H);
    uint64_t sum = 0;

    qtest_memread(qts, REF_Y, a, W * H);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, REF_Y, b, W * H);
    for (int i = 0; i < W * H; i++) {
        sum += abs(a[i] - b[i]);
    }
    return (double)sum / (W * H);
}

static void test_regs(const char *machine, uint32_t id, uint32_t refresh)
{
    QTestState *qts = qtest_initf("-machine %s", machine);

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, MDNS), ==, id);
    qtest_writel(qts, MDNS, 0);
    g_assert_cmphex(qtest_readl(qts, MDNS), ==, id);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0xc), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x1c), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x304), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x7fc), ==, 0x20151120);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x300), ==, 0x4040);
    qtest_writel(qts, MDNS + 0x4, 0x110);
    qtest_writel(qts, MDNS + 0x4, 0x111);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x4), ==, 0x110);
    qtest_writel(qts, MDNS + 0x18, 1);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x18), ==, refresh);
    qtest_writel(qts, MDNS + 0x10, 0x11111111);
    g_assert_cmphex(qtest_readl(qts, MDNS + 0x10), ==, 0x11111111);
    qtest_quit(qts);
}

static void test_t31_regs(void)
{
    test_regs("ingenic-t31", 0x20151120, 0);
}

static void test_t23_regs(void)
{
    test_regs("ingenic-t23", 0x23021515, 0x20);
}

/* A still scene: every block of every grid 15, the reference the scene */
static void test_still(void)
{
    QTestState *qts = mdns_start("ingenic-t31", ",motion=off", false);
    g_autofree uint8_t *sta = g_malloc(4 * STA_SIZE);
    g_autofree uint8_t *y = g_malloc(W * H);
    bool lit = false;

    mdns_frames(qts, 20);
    qtest_memread(qts, STA0, sta, 4 * STA_SIZE);
    for (int k = 0; k < 4; k++) {
        const uint8_t *s = sta + k * STA_SIZE;
        unsigned top = !k, rows = BH - (k & 1);

        for (unsigned by = 0; by < STA_ROWS; by++) {
            for (unsigned i = 0; i < STA_STRIDE; i++) {
                bool data = by >= top && by < top + rows && i < BW / 2;

                g_assert_cmphex(s[by * STA_STRIDE + i], ==, data ? 0xff : 0);
            }
        }
    }
    qtest_memread(qts, REF_Y, y, W * H);
    for (int i = 0; i < W * H; i++) {
        lit |= y[i] > 16;
    }
    g_assert_true(lit);
    g_assert_cmpfloat(ref_noise(qts), ==, 0);
    qtest_quit(qts);
}

/*
 * Noise: the reference holds still with the reduction on, not with it off.
 * The model's noise (+-32) is well above a sensor's: libimp's thresholds at
 * its strongest (temper 255) take it for still
 */
static void test_noise(void)
{
    QTestState *qts = mdns_start("ingenic-t23", ",motion=off,noise=on", true);
    g_autofree uint8_t *sta = g_malloc(STA_SIZE);
    double on, off;

    for (int k = 0; k < 4; k++) {
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xffc60c64);
    }
    qtest_writel(qts, MDNS + 0x144, 0xffc63633);
    mdns_frames(qts, 20);
    on = ref_noise(qts);
    qtest_memread(qts, STA0, sta, STA_SIZE);
    /* one shared buffer: the first's, its first row 0 */
    for (unsigned bx = 0; bx < BW; bx++) {
        g_assert_cmpuint(sta[bx / 2], ==, 0);
        for (unsigned by = 0; by < BH; by++) {
            g_assert_cmpuint(mdns_age(sta, 1, bx, by), ==, 15);
        }
    }
    qtest_writel(qts, MDNS + 0x10, 0x01110111);     /* no Y or UV filter */
    qtest_clock_step(qts, FRAME_NS);
    off = ref_noise(qts);
    g_assert_cmpfloat(off, >, 1);
    g_assert_cmpfloat(on, <, off / 4);
    qtest_quit(qts);
}

/* The object crossing the ramp: its blocks 0, the rest still 15 */
static void test_motion(void)
{
    QTestState *qts = mdns_start("ingenic-t31", ",object=on", false);
    g_autofree uint8_t *sta = g_malloc(STA_SIZE);
    unsigned moving = 0, still = 0;

    mdns_frames(qts, 20);
    qtest_memread(qts, STA0, sta, STA_SIZE);
    for (unsigned by = 0; by < BH; by++) {
        for (unsigned bx = 0; bx < BW; bx++) {
            unsigned a = mdns_age(sta, 1, bx, by);

            moving += a == 0;
            still += a == 15;
        }
    }
    g_assert_cmpuint(moving, >, 0);
    g_assert_cmpuint(still, >, BW * BH / 2);
    qtest_quit(qts);
}

/*
 * The T41's buffers as tx_isp_set_mdns_buf lays them (a 640 x 360 frame):
 * a still scene leaves every block 7, two a byte from the first row, and
 * the reference the scene
 */
static void test_t41(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device "
                                 "image-sensor,model=gc4023,bus=i2c0,"
                                 "motion=off");
    enum { TW = 640, TH = 360, TBW = 40, TBH = 22, TSTA = 0x28 };
    const uint32_t y = 0x02000000, uv = y + 0x38400, sta = uv + 0x1c400;
    g_autofree uint8_t *st = g_malloc(TSTA * (TBH + 1));
    g_autofree uint8_t *pic = g_malloc(TW * TH);
    bool lit = false;

    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) & ~BIT(23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~BIT(27));
    qtest_writel(qts, ISP + 0x80, TW << 16 | TH);
    qtest_writel(qts, ISP + 0xf040, y);
    qtest_writel(qts, ISP + 0xf044, TW);
    qtest_writel(qts, ISP + 0xf048, uv);
    qtest_writel(qts, ISP + 0xf04c, TW);
    qtest_writel(qts, ISP + 0xf050, sta);
    qtest_writel(qts, ISP + 0xf054, TSTA);
    qtest_writel(qts, ISP + 0xf10c, TBH << 16 | TBW);
    qtest_writel(qts, ISP + 0x80000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 12);
    qtest_memread(qts, sta, st, TSTA * (TBH + 1));
    for (unsigned by = 0; by < TBH; by++) {
        for (unsigned i = 0; i < TBW / 2; i++) {
            g_assert_cmphex(st[by * TSTA + i], ==, 0x77);
        }
    }
    g_assert_cmphex(st[TBW / 2], ==, 0);
    qtest_memread(qts, y, pic, TW * TH);
    for (int i = 0; i < TW * TH; i++) {
        lit |= pic[i] > 16;
    }
    g_assert_true(lit);
    qtest_quit(qts);
}

/*
 * The T21's second ring, as a T21N fills it: frame n in slot (n - 1) % 16,
 * its count in the header, a block's age in a word; the first ring, whose
 * measure is not decoded, as the driver left it
 */
static void test_t21(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t21 -device "
                                 "image-sensor,model=jxq03,bus=i2c0,"
                                 "motion=off");
    enum { SS = W / 16 * 2, SLOT = SS * (BH + 1) };
    const uint32_t ring0 = STA0, ring1 = STA0 + 20 * SLOT;
    g_autofree uint8_t *slot = g_malloc(SLOT);
    uint32_t last = 0, n = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, W << 16 | H);
    qtest_writel(qts, ISP + 0x1b10, REF_Y);
    qtest_writel(qts, ISP + 0x1b14, W);
    qtest_writel(qts, ISP + 0x1b20, REF_UV);
    qtest_writel(qts, ISP + 0x1b24, W);
    qtest_writel(qts, ISP + 0x1b30, ring0);
    qtest_writel(qts, ISP + 0x1b34, SS);
    qtest_writel(qts, ISP + 0x1b38, SLOT);
    qtest_writel(qts, ISP + 0x1b40, ring1);
    qtest_writel(qts, ISP + 0x1b44, SS);
    qtest_writel(qts, ISP + 0x1b48, SLOT);
    qtest_writel(qts, ISP + 0x1b80, 15);
    qtest_writel(qts, ISP + 0x10000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS * 20 + FRAME_NS / 2);
    for (int k = 0; k < 20; k++) {
        uint32_t c = qtest_readl(qts, ring1 + k * SLOT);

        if (k < 16) {
            g_assert_cmpuint(c, >, 0);
            g_assert_cmpuint((c - 1) % 16, ==, k);
            g_assert_cmpuint(qtest_readl(qts, ring0 + k * SLOT), ==, 0);
            if (c > last) {
                last = c;
                n = k;
            }
        } else {
            g_assert_cmpuint(c, ==, 0);
        }
    }
    g_assert_cmpuint(last, >=, 16);
    /* still: every block 15 once past 15 frames, no difference */
    qtest_memread(qts, ring1 + n * SLOT, slot, SLOT);
    for (unsigned i = 0; i < BW * BH; i++) {
        g_assert_cmpuint(slot[SS + i * 2], ==, 15);
        g_assert_cmpuint(slot[SS + i * 2 + 1], ==, 0);
    }
    qtest_memread(qts, ring0 + n * SLOT, slot, SLOT);
    for (unsigned i = 0; i < BW * BH * 2; i++) {
        g_assert_cmpuint(slot[SS + i], ==, 0);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-isp-mdns/t31/regs", test_t31_regs);
    qtest_add_func("/ingenic-isp-mdns/t23/regs", test_t23_regs);
    qtest_add_func("/ingenic-isp-mdns/still", test_still);
    qtest_add_func("/ingenic-isp-mdns/noise", test_noise);
    qtest_add_func("/ingenic-isp-mdns/motion", test_motion);
    qtest_add_func("/ingenic-isp-mdns/t41", test_t41);
    qtest_add_func("/ingenic-isp-mdns/t21", test_t21);
    return g_test_run();
}
