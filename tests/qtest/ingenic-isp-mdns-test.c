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
#include "qemu/bswap.h"

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
 * The T41's blocks follow the guest's size (+0x100) and origin (+0x104):
 * 24 pixels square from (6, 4), 10 across and 5 down in a 256 x 128 frame
 * with the object crossing. A block's age is 0 where its mean difference
 * from the frame before, which the reference holds, is over 5, and the
 * rows are written in whole 8-byte groups: what lies past stays as it was
 */
static void test_t41_blocks(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 -device "
                                 "image-sensor,model=gc4023,bus=i2c0,"
                                 "object=on");
    enum { BS = 24, GX = 6, GY = 4, NX = 10, NY = 5, TSTA = 0x28 };
    const uint32_t y = 0x02000000, uv = y + W * H * 2, sta = uv + W * H;
    g_autofree uint8_t *st = g_malloc(TSTA * (NY + 2));
    g_autofree uint8_t *cur = g_malloc(W * H), *old = g_malloc(W * H);
    unsigned moved = 0;

    qtest_writel(qts, CPM_CLKGR0, qtest_readl(qts, CPM_CLKGR0) & ~BIT(23));
    qtest_writel(qts, 0x10000090, qtest_readl(qts, 0x10000090) & ~BIT(27));
    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    qtest_writel(qts, ISP + 0xf040, y);
    qtest_writel(qts, ISP + 0xf044, W);
    qtest_writel(qts, ISP + 0xf048, uv);
    qtest_writel(qts, ISP + 0xf04c, W);
    qtest_writel(qts, ISP + 0xf050, sta);
    qtest_writel(qts, ISP + 0xf054, TSTA);
    qtest_writel(qts, ISP + 0xf100, BS);
    qtest_writel(qts, ISP + 0xf104, GY << 8 | GX);
    qtest_writel(qts, ISP + 0xf10c, NY << 16 | NX);
    for (unsigned i = 0; i < TSTA * (NY + 2); i++) {
        qtest_writeb(qts, sta + i, 0xa5);
    }
    qtest_writel(qts, ISP + 0x80000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS / 2);
    qtest_memread(qts, y, old, W * H);
    for (int f = 0; f < 40; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, y, cur, W * H);
        qtest_memread(qts, sta, st, TSTA * (NY + 2));
        for (unsigned by = 0; by < NY; by++) {
            for (unsigned bx = 0; bx < NX; bx++) {
                int sum = 0;
                unsigned age;

                for (unsigned yy = 0; yy < BS; yy++) {
                    for (unsigned xx = 0; xx < BS; xx++) {
                        unsigned i = (GY + by * BS + yy) * W +
                                     GX + bx * BS + xx;

                        sum += cur[i] - old[i];
                    }
                }
                age = st[by * TSTA + bx / 2] >> (bx & 1) * 4 & 15;
                g_assert_cmpint(age == 0, ==, abs(sum) > 5 * BS * BS);
                moved += abs(sum) > 5 * BS * BS;
            }
        }
        for (unsigned by = 0; by < NY + 2; by++) {
            for (unsigned i = NX / 2; i < TSTA; i++) {
                g_assert_cmphex(st[by * TSTA + i], ==,
                                by < NY && i < 8 ? 0 : 0xa5);
            }
        }
        memcpy(old, cur, W * H);
    }
    g_assert_cmpuint(moved, >, 0);
    qtest_quit(qts);
}

/*
 * The T40's, whose reference is a coded stream: its buffers are left as the
 * driver set them, and the ages in the first buffer, two a byte in rows of
 * whole 8-byte groups, follow the frame two frames back: a step in the
 * light holds the blocks it moves at 0 for two frames, as a T40XP's do
 */
static void test_t40(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40 -device "
                                 "image-sensor,model=jxq03,bus=i2c0,id=cam,"
                                 "motion=off");
    enum { BS = 20, NX = 12, NY = 6, TSTA = 0x20, ROWS = NY + 2 };
    const uint32_t y = 0x02000000, uv = y + W * H * 2, sta = uv + W * H * 2;
    uint8_t zero[3][NX * NY], st[TSTA * ROWS], pic[64];
    unsigned held = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf000), ==, 0x20191209);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf00c), ==, 0x40);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf100), ==, 0x10);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf10c), ==, 0x00100010);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf800), ==, 0x20191209);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf80c), ==, 0x40);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xfb00), ==, 0x100);
    qtest_writel(qts, ISP + 0xf004, 1);
    qtest_writel(qts, ISP + 0xf008, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf004), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xf008), ==, 0);

    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    qtest_writel(qts, ISP + 0xf040, y);
    qtest_writel(qts, ISP + 0xf044, W);
    qtest_writel(qts, ISP + 0xf048, uv);
    qtest_writel(qts, ISP + 0xf04c, W);
    qtest_writel(qts, ISP + 0xf050, sta);
    qtest_writel(qts, ISP + 0xf054, TSTA);
    qtest_writel(qts, ISP + 0xf100, BS);
    qtest_writel(qts, ISP + 0xf104, 3);
    qtest_writel(qts, ISP + 0xf10c, NY << 16 | NX);
    for (unsigned i = 0; i < 64; i += 4) {
        qtest_writel(qts, y + i, 0x5a5a5a5a);
    }
    for (unsigned i = 0; i < TSTA * ROWS; i++) {
        qtest_writeb(qts, sta + i, i / TSTA < NY && i % TSTA < 8 ? 0 : 0xa5);
    }
    qtest_writel(qts, ISP + 0x80000, 1);            /* VIC start: a frame */
    qtest_clock_step(qts, FRAME_NS * 2 + FRAME_NS / 2);
    qtest_memread(qts, sta, st, sizeof(st));
    for (unsigned r = 0; r < ROWS; r++) {
        for (unsigned i = 0; i < TSTA; i++) {
            uint8_t want = r >= NY || i >= 8 ? 0xa5 : i < NX / 2 ? 0x33 : 0;

            g_assert_cmphex(st[r * TSTA + i], ==, want);
        }
    }
    qtest_clock_step(qts, FRAME_NS * 9);
    qtest_memread(qts, sta, st, sizeof(st));
    g_assert_cmphex(st[0], ==, 0x77);
    g_assert_cmphex(st[NX / 2], ==, 0);

    /* a step in the light: the blocks it moves are 0 for two frames */
    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': '/machine/peripheral/cam', "
                             "'property': 'light', 'value': 30}}");
    for (unsigned f = 0; f < 3; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, sta, st, sizeof(st));
        for (unsigned i = 0; i < NX * NY; i++) {
            unsigned a = st[i / NX * TSTA + i % NX / 2] >> (i % 2) * 4 & 15;

            zero[f][i] = a == 0;
            if (f == 2 && zero[0][i]) {
                g_assert_cmpuint(a, ==, 1);
            } else if (f == 2) {
                g_assert_cmpuint(a, >, 1);
            }
            if (f && !zero[0][i]) {
                g_assert_false(zero[f][i]);
            }
        }
    }
    for (unsigned i = 0; i < NX * NY; i++) {
        g_assert_cmpuint(zero[1][i], ==, zero[0][i]);
        held += zero[0][i];
    }
    g_assert_cmpuint(held, >, 0);

    qtest_memread(qts, y, pic, sizeof(pic));
    for (unsigned i = 0; i < sizeof(pic); i++) {
        g_assert_cmphex(pic[i], ==, 0x5a);
    }
    qtest_quit(qts);
}

/*
 * The T32's, whose reference is coded too: the buffers stay as the driver
 * set them (it gives a single one at +0x40 here), and the ages in +0x50's
 * are 5-bit fields, twelve to an 8-byte group, up to 15. A step in the
 * light holds the blocks it moves at 0 for one frame
 */
static void test_t32(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 -global "
                                 "ingenic-t31.soc-variant=t32nq -device "
                                 "image-sensor,model=jxq03,bus=i2c0,id=cam,"
                                 "motion=off");
    enum { BS = 14, NX = 17, NY = 9, TSTA = 0x40, ROWS = NY + 2, ROW = 16 };
    const uint32_t y = 0x02000000, sta = y + W * H * 4;
    uint8_t st[TSTA * ROWS], pic[64], zero[NX * NY];
    unsigned held = 0;

    qtest_writel(qts, CPM_CLKGR0, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd000), ==, 0x24041703);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd004), ==, 0x24041928);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd044), ==, 0x780);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd04c), ==, 0x780);
    qtest_writel(qts, ISP + 0xd014, 1);
    qtest_writel(qts, ISP + 0xd018, 1);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd014), ==, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + 0xd018), ==, 0);

    qtest_writel(qts, ISP + 0x80, W << 16 | H);
    qtest_writel(qts, ISP + 0xd040, y);
    qtest_writel(qts, ISP + 0xd080, W * H / 2 >> 8);
    qtest_writel(qts, ISP + 0xd050, sta);
    qtest_writel(qts, ISP + 0xd054, TSTA);
    qtest_writel(qts, ISP + 0xd100, BS);
    qtest_writel(qts, ISP + 0xd104, 1 << 8 | 3);
    qtest_writel(qts, ISP + 0xd10c, NY << 16 | NX);
    for (unsigned i = 0; i < 64; i += 4) {
        qtest_writel(qts, y + i, 0x5a5a5a5a);
    }
    for (unsigned i = 0; i < TSTA * ROWS; i++) {
        qtest_writeb(qts, sta + i, i / TSTA < NY && i % TSTA < ROW ? 0 : 0xa5);
    }
    qtest_writel(qts, ISP + 0x80000 + 0x10, 1);     /* VIC start: a frame */
    qtest_clock_step(qts, FRAME_NS * 2 + FRAME_NS / 2);
    qtest_memread(qts, sta, st, sizeof(st));
    for (unsigned r = 0; r < ROWS; r++) {
        for (unsigned i = 0; i < TSTA; i++) {
            uint8_t want = 0xa5;

            if (r < NY && i < ROW) {
                /* 3 in each field: twelve of them, then five */
                uint64_t g = 0;

                for (unsigned k = 0; k < (i < 8 ? 12 : NX - 12); k++) {
                    g |= 3ull << (5 * k);
                }
                want = g >> (i % 8 * 8);
            }
            g_assert_cmphex(st[r * TSTA + i], ==, want);
        }
    }
    qtest_clock_step(qts, FRAME_NS * 20);
    qtest_memread(qts, sta, st, sizeof(st));
    for (unsigned bx = 0; bx < NX; bx++) {
        uint64_t g = ldq_le_p(st + bx / 12 * 8);

        g_assert_cmpuint(extract64(g, bx % 12 * 5, 5), ==, 15);
    }

    /* a step in the light: the blocks it moves are 0 for one frame */
    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': '/machine/peripheral/cam', "
                             "'property': 'light', 'value': 30}}");
    for (unsigned f = 0; f < 2; f++) {
        qtest_clock_step(qts, FRAME_NS);
        qtest_memread(qts, sta, st, sizeof(st));
        for (unsigned i = 0; i < NX * NY; i++) {
            uint64_t g = ldq_le_p(st + i / NX * TSTA + i % NX / 12 * 8);
            unsigned a = extract64(g, i % NX % 12 * 5, 5);

            if (!f) {
                zero[i] = a == 0;
                held += zero[i];
                g_assert_true(zero[i] || a == 15);
            } else {
                g_assert_cmpuint(a, ==, zero[i] ? 1 : 15);
            }
        }
    }
    g_assert_cmpuint(held, >, 0);

    qtest_memread(qts, y, pic, sizeof(pic));
    for (unsigned i = 0; i < sizeof(pic); i++) {
        g_assert_cmphex(pic[i], ==, 0x5a);
    }
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
    qtest_add_func("/ingenic-isp-mdns/t41/blocks", test_t41_blocks);
    qtest_add_func("/ingenic-isp-mdns/t40", test_t40);
    qtest_add_func("/ingenic-isp-mdns/t32", test_t32);
    qtest_add_func("/ingenic-isp-mdns/t21", test_t21);
    return g_test_run();
}
