/*
 * QTest testcase for the T30's noise reducer (NCU) and its post-processor
 *
 * The values a T30X reads: reset values and the bits each register keeps,
 * the control words' counters and flags, and frames from memory, which with
 * the functions at reset go through unchanged, to REF0 first and then
 * REF1 and REF0 in turn.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define NCU             0x13090000
#define CPM_SRBC1       0x100000ac
#define NCU_IRQ         33
#define PP_IRQ          35

#define W               128
#define H               64
#define YS              (W * H)
#define CUR             0x01000000
#define REF0            0x01100000
#define REF1            0x01200000

typedef struct {
    uint32_t reg, ones;
} Mask;

static const Mask masks[] = {
    { 0x018, 0x0000001f }, { 0x024, 0xffffffff }, { 0x02c, 0x0fff0fff },
    { 0x030, 0xffffffff }, { 0x034, 0x0000ffff }, { 0x060, 0x0000003f },
    { 0x088, 0x0000003f }, { 0x0b8, 0x0fff0fff }, { 0x0bc, 0x00000111 },
    { 0x0c0, 0x00ff00ff }, { 0x0d0, 0x00000000 }, { 0x118, 0x00000000 },
    { 0x180, 0x00000003 }, { 0x18c, 0x7f7f7f00 }, { 0x200, 0x0000ff7f },
    { 0x2a0, 0x1ffff31f }, { 0x340, 0x71ff00f3 }, { 0x3f0, 0xf1ff01ff },
    { 0x3f8, 0x0013ffff }, { 0x404, 0x00000000 }, { 0x518, 0x0000000f },
    { 0x520, 0x00000000 }, { 0x524, 0x0fff0fff }, { 0x558, 0x00000001 },
    { 0x560, 0x001fffff }, { 0x574, 0x000001ff }, { 0x57c, 0x00000111 },
    { 0x580, 0x00000000 }, { 0xffc, 0x00000000 },
};

static const Mask resets[] = {
    { 0x000, 0x00000011 }, { 0x004, 0x00010000 }, { 0x008, 0x00000000 },
    { 0x00c, 0x00000004 }, { 0x010, 0x00000100 }, { 0x014, 0x0a170510 },
    { 0x01c, 0x00000000 }, { 0x020, 0x40000000 }, { 0x024, 0x0c000000 },
    { 0x028, 0xff00ff00 }, { 0x02c, 0x00100010 }, { 0x0b0, 0x02060101 },
    { 0x0b4, 0x02060101 }, { 0x0b8, 0x00200030 }, { 0x2ac, 0x00010001 },
    { 0x2c4, 0x00020002 }, { 0x3e8, 0x00010001 }, { 0x400, 0x00020002 },
    { 0x500, 0x00000011 }, { 0x50c, 0x00000004 }, { 0x510, 0x00000100 },
    { 0x514, 0x0a170510 }, { 0x51c, 0x00000001 }, { 0x524, 0x00100010 },
    { 0x55c, 0x02010201 },
};

static uint32_t rd(QTestState *qts, uint32_t reg)
{
    return qtest_readl(qts, NCU + reg);
}

static void wr(QTestState *qts, uint32_t reg, uint32_t v)
{
    qtest_writel(qts, NCU + reg, v);
}

/* As tx-isp does it: ask it to stop, wait for the acknowledge, reset */
static void cpm_reset(QTestState *qts)
{
    qtest_writel(qts, CPM_SRBC1, 1 << 1);
    g_assert_cmphex(qtest_readl(qts, CPM_SRBC1) & 1, ==, 1);
    qtest_writel(qts, CPM_SRBC1, 1 << 2);
    qtest_writel(qts, CPM_SRBC1, 0);
}

static void check(QTestState *qts, const Mask *m, int n)
{
    for (int i = 0; i < n; i++) {
        g_assert_cmphex(rd(qts, m[i].reg), ==, m[i].ones);
    }
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    check(qts, resets, ARRAY_SIZE(resets));
    for (int i = 0; i < ARRAY_SIZE(masks); i++) {
        wr(qts, masks[i].reg, 0xffffffff);
        g_assert_cmphex(rd(qts, masks[i].reg), ==, masks[i].ones);
        wr(qts, masks[i].reg, 0);
        g_assert_cmphex(rd(qts, masks[i].reg), ==, 0);
    }
    wr(qts, 0x018, 0x1f);
    g_assert_cmphex(rd(qts, 0x01c), ==, 0x20170614);
    wr(qts, 0x018, 0x1e);
    g_assert_cmphex(rd(qts, 0x01c), ==, 0x00011212);
    wr(qts, 0x518, 0x2);
    g_assert_cmphex(rd(qts, 0x51c), ==, 0x00008880);
    cpm_reset(qts);
    check(qts, resets, ARRAY_SIZE(resets));
    qtest_quit(qts);
}

/* The control words, as the T30X answered the same writes */
static void test_control(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    static const uint32_t sreg[][2] = {
        { 0x11, 0x10 }, { 0x13, 0x12 }, { 0x17, 0x16 }, { 0x1f, 0x16 },
        { 0x07, 0x07 }, { 0x05, 0x05 }, { 0x08, 0x00 }, { 0x20, 0x00 },
    };

    qtest_irq_intercept_in(qts, "/machine/soc/intc");

    /* Each write with bit 0 clear is a soft reset, counted in 15:8 */
    wr(qts, 0x000, 0);
    g_assert_cmphex(rd(qts, 0x000), ==, 0x111);
    wr(qts, 0x000, 1);
    wr(qts, 0x000, 0xffffffff);
    g_assert_cmphex(rd(qts, 0x000), ==, 0x111);
    wr(qts, 0x000, 0x10);
    g_assert_cmphex(rd(qts, 0x000), ==, 0x211);
    for (int i = 0; i < 254; i++) {
        wr(qts, 0x000, 0);
    }
    g_assert_cmphex(rd(qts, 0x000), ==, 0x011);

    for (int i = 0; i < ARRAY_SIZE(sreg); i++) {
        wr(qts, 0x00c, 0);
        wr(qts, 0x00c, sreg[i][0]);
        g_assert_cmphex(rd(qts, 0x00c), ==, sreg[i][1]);
    }

    /* A stop while idle is done at once, its frame-done interrupt too */
    wr(qts, 0x008, 1);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x103);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x110);
    g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x008, 1);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x103);
    wr(qts, 0x008, 0);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x102);
    wr(qts, 0x010, 1);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x011);
    g_assert_true(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x010, 3);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x101);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x100);
    g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x010, 0);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x100);
    for (int i = 0; i < 5; i++) {
        wr(qts, 0x008, 1);
        wr(qts, 0x008, 0);
    }
    g_assert_cmphex(rd(qts, 0x008), ==, 0x602);
    wr(qts, 0x000, 0x10);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x600);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x110);

    /* START bit 1 marks the next frame the first */
    wr(qts, 0x004, 2);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10001);
    wr(qts, 0x004, 0xfffc);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10001);

    /* 0x20: bit 0 shows the clock count, bit 1 the ping-pong */
    wr(qts, 0x020, 1);
    g_assert_cmphex(rd(qts, 0x020), ==, 0);
    wr(qts, 0x020, 0);
    g_assert_cmphex(rd(qts, 0x020), ==, 0);
    wr(qts, 0x020, 2);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);

    /* The post-processor's */
    wr(qts, 0x500, 1);
    g_assert_cmphex(rd(qts, 0x500), ==, 0x011);
    wr(qts, 0x500, 2);
    g_assert_cmphex(rd(qts, 0x500), ==, 0x111);
    wr(qts, 0x504, 1);
    g_assert_cmphex(rd(qts, 0x504), ==, 0x103);
    g_assert_cmphex(rd(qts, 0x510), ==, 0x110);
    wr(qts, 0x510, 1);
    g_assert_true(qtest_get_irq(qts, PP_IRQ));
    g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    wr(qts, 0x510, 3);
    g_assert_false(qtest_get_irq(qts, PP_IRQ));
    wr(qts, 0x508, 1);
    wr(qts, 0x508, 0);
    g_assert_cmphex(rd(qts, 0x508), ==, 1);
    wr(qts, 0x50c, 0x17);
    g_assert_cmphex(rd(qts, 0x50c), ==, 0x16);

    cpm_reset(qts);
    check(qts, resets, ARRAY_SIZE(resets));
    g_assert_cmphex(rd(qts, 0x508), ==, 0);
    qtest_quit(qts);
}

static void setup(QTestState *qts)
{
    cpm_reset(qts);
    wr(qts, 0x030, CUR);
    wr(qts, 0x040, CUR + YS);
    wr(qts, 0x038, REF0);
    wr(qts, 0x048, REF0 + YS);
    wr(qts, 0x050, REF1);
    wr(qts, 0x058, REF1 + YS);
    for (uint32_t r = 0x034; r <= 0x05c; r += 8) {
        wr(qts, r, W);
    }
    wr(qts, 0x02c, H << 16 | W);
    wr(qts, 0x024, 0x0c000030);
    wr(qts, 0x000, 0);
}

static void frame_data(uint8_t *f, int n)
{
    for (int i = 0; i < YS * 3 / 2; i++) {
        f[i] = (i * 7 + n * 31 + (i >> 7) * 3) & 0xff;
    }
}

static void check_buf(QTestState *qts, uint32_t addr, const uint8_t *want)
{
    g_autofree uint8_t *got = g_malloc(YS * 3 / 2);

    qtest_memread(qts, addr, got, YS * 3 / 2);
    g_assert_cmpmem(got, YS * 3 / 2, want, YS * 3 / 2);
}

/*
 * Frames from memory to memory (FUN_CNTRL bits 4 and 5), the sequence the
 * T30X ran: the first to REF0, then REF1 and REF0 in turn; START's mark
 * shifts up a nibble per frame; a soft reset points the ping-pong at REF0
 */
static void test_frames(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    static const uint32_t start[] = { 0x10010, 0x10100, 0x11000, 0x10000 };
    uint8_t f[4][YS * 3 / 2], fill[YS * 3 / 2];

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    memset(fill, 0x11, sizeof(fill));
    qtest_memwrite(qts, REF0, fill, sizeof(fill));
    memset(fill, 0x22, sizeof(fill));
    qtest_memwrite(qts, REF1, fill, sizeof(fill));
    setup(qts);
    wr(qts, 0x00c, 5);
    wr(qts, 0x004, 2);
    wr(qts, 0x010, 1);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10001);
    for (int n = 0; n < 4; n++) {
        frame_data(f[n], n);
        qtest_memwrite(qts, CUR, f[n], sizeof(f[n]));
        g_assert_cmphex(rd(qts, 0x010), ==, 0x101);
        wr(qts, 0x004, rd(qts, 0x004) | 1);
        g_assert_cmphex(rd(qts, 0x004), ==, start[n]);
        g_assert_cmphex(rd(qts, 0x008), ==, 0x002);
        g_assert_cmphex(rd(qts, 0x010), ==, 0x011);
        g_assert_true(qtest_get_irq(qts, NCU_IRQ));
        g_assert_cmphex(rd(qts, 0x020), ==, n & 1 ? 0x80000000 : 0x40000000);
        g_assert_cmphex(rd(qts, 0x01c), ==, n + 1);
        g_assert_cmphex(rd(qts, 0x00c), ==, 4);
        check_buf(qts, n & 1 ? REF1 : REF0, f[n]);
        if (n) {
            check_buf(qts, n & 1 ? REF0 : REF1, f[n - 1]);
        }
        wr(qts, 0x010, 3);
        g_assert_false(qtest_get_irq(qts, NCU_IRQ));
    }

    /* The last went to REF1; after a soft reset the next does too */
    wr(qts, 0x000, 0);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x80000000);

    /* The clock count view: 12529 on the T30X, the frame's bytes here */
    wr(qts, 0x010, 3);
    wr(qts, 0x020, 1);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x020), ==, YS * 3 / 2);
    wr(qts, 0x020, 2);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);
    qtest_quit(qts);
}

/* A frame started before any load, or while stopped, never ends */
static void test_hang(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    setup(qts);
    wr(qts, 0x004, 2);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0);
    g_assert_cmphex(rd(qts, 0x01c), ==, 1);
    g_assert_cmphex(rd(qts, 0x010), ==, 0x100);
    wr(qts, 0x00c, 0x15);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0);
    g_assert_cmphex(rd(qts, 0x01c), ==, 1);

    setup(qts);
    wr(qts, 0x00c, 5);
    wr(qts, 0x004, 2);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10010);
    wr(qts, 0x010, 2);
    wr(qts, 0x008, 1);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x103);
    wr(qts, 0x010, 2);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x40000000);
    wr(qts, 0x004, 1);
    g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0);
    g_assert_cmphex(rd(qts, 0x020), ==, 0x80000000);
    g_assert_cmphex(rd(qts, 0x01c), ==, 2);
    g_assert_cmphex(rd(qts, 0x008), ==, 0x101);
    cpm_reset(qts);
    g_assert_cmphex(rd(qts, 0x004), ==, 0x10000);
    qtest_quit(qts);
}

/*
 * The 3D noise reduction, frames from memory through libimp's settings for
 * a room's light and for its highest gain, against what a T30X made of the
 * same frames (FNV-1a of each output, 0 where not checked) and the
 * statistics it wrote. The reference is updated in place, as libimp has
 * it, and the statistics go to a ring of 64-byte slots of 4 rows of 16
 * bytes (5 x 3 blocks of 28 pixels), as libimp lays them out.
 *
 * The moving scene: a gradient with noise, a block that moves and in the
 * third frame a brighter strip. The still scene: a gradient with a little
 * noise and a strip of 28 pixels that is brighter in frames 2 and 3. The
 * threshold scenes: flat frames, the third against a reference of single
 * pixels (and chroma samples) off by steps of one around a static pixel's
 * threshold, with the statistics read set to age 5, where the T30X's
 * decisions follow the level of the frame as well as the step.
 */
#define NW              128
#define NH              64
#define NSIZE           (NW * NH * 3 / 2)
#define NBX             5
#define NBY             3
#define STA             0x01400000
#define STA_STRIDE      16
#define STA_SLOT        64

typedef struct {
    uint32_t reg, val;
} Reg;

typedef enum {
    NR_MOVING, NR_STILL, NR_THRESHOLD,
} NrScene;

typedef struct {
    const Reg *regs;
    int nregs;
    NrScene scene;
    int level;                  /* the threshold scenes' luma */
    uint32_t frame[5], luma[5];
    bool has_sta;
    uint8_t sta[4][NBY][NBX];   /* frames 1 to 4's ages */
} NrCase;

static const Reg nr_room_regs[] = {
    { 0x024, 0x022acf30 }, { 0x028, 0xff00ff00 }, { 0x088, 0x00000031 },
    { 0x0b0, 0x02060101 }, { 0x0b4, 0x02060101 }, { 0x0b8, 0x00200030 },
    { 0x0e0, 0x00000ee5 }, { 0x1d0, 0x00000203 }, { 0x1d4, 0x1ff0741d },
    { 0x1d8, 0x0007fdff }, { 0x1dc, 0x00250025 }, { 0x1e0, 0x00250025 },
    { 0x1fc, 0x0000ffff }, { 0x200, 0x0000147f }, { 0x204, 0x00000025 },
    { 0x208, 0x0505000a }, { 0x220, 0x6802d428 }, { 0x254, 0x04c804c8 },
    { 0x288, 0x04c804c8 }, { 0x2a0, 0x0e70a10f }, { 0x2a4, 0x002500e7 },
    { 0x2a8, 0x00000003 }, { 0x2ac, 0x00010001 }, { 0x2b0, 0x00000014 },
    { 0x2b4, 0x00360036 }, { 0x2b8, 0x00000028 }, { 0x2bc, 0x000000ff },
    { 0x2c0, 0x00000c0c }, { 0x2d0, 0x0000000b }, { 0x2d4, 0x00000010 },
    { 0x340, 0x00390001 }, { 0x344, 0x00ff00ff }, { 0x348, 0x001f001f },
    { 0x34c, 0x01ff01ff }, { 0x350, 0x00390039 }, { 0x354, 0x00000039 },
    { 0x370, 0x0505000a }, { 0x390, 0x00000080 }, { 0x3ac, 0x00000028 },
    { 0x3b0, 0x00ff00ff }, { 0x3cc, 0x000000b6 }, { 0x3d0, 0x00ff00ff },
    { 0x3e0, 0x0dc0a0f1 }, { 0x3e4, 0x003900dc }, { 0x3ec, 0x00000014 },
    { 0x3f0, 0x00360036 }, { 0x3f4, 0x00000014 }, { 0x3f8, 0x00018006 },
    { 0x3fc, 0x00000e0e },
};

static const Reg nr_gain_regs[] = {
    { 0x024, 0x022acf30 }, { 0x028, 0xff00ff00 }, { 0x088, 0x00000031 },
    { 0x0b0, 0x02060101 }, { 0x0b4, 0x02060101 }, { 0x0b8, 0x00200030 },
    { 0x1d0, 0x00000603 }, { 0x1d4, 0x1ff0781e }, { 0x1d8, 0x0007fdff },
    { 0x1dc, 0x00280028 }, { 0x1e0, 0x00280028 }, { 0x1ec, 0x01020304 },
    { 0x1fc, 0x0000ffff }, { 0x200, 0x0000147f }, { 0x204, 0x00000028 },
    { 0x208, 0x0a0a000a }, { 0x220, 0x0d23e85a }, { 0x254, 0x00ff00ff },
    { 0x288, 0x00ff00ff }, { 0x2a0, 0x0eb3c008 }, { 0x2a4, 0x002800eb },
    { 0x2a8, 0x00000003 }, { 0x2ac, 0x00010001 }, { 0x2b0, 0x00000014 },
    { 0x2b4, 0x00280028 }, { 0x2b8, 0x0000000f }, { 0x2bc, 0x000000ff },
    { 0x2c0, 0x00000c0c }, { 0x2d0, 0x0000000f }, { 0x2d4, 0x00000010 },
    { 0x340, 0x001e0001 }, { 0x344, 0x00ff00ff }, { 0x348, 0x003c003c },
    { 0x34c, 0x01ff01ff }, { 0x350, 0x001e0000 }, { 0x354, 0x0000001e },
    { 0x360, 0x01020304 }, { 0x370, 0x0a0a001e }, { 0x390, 0x000000d2 },
    { 0x3ac, 0x0000005a }, { 0x3b0, 0x00ff00ff }, { 0x3cc, 0x00000064 },
    { 0x3d0, 0x00ff00ff }, { 0x3e0, 0x0eb1e080 }, { 0x3e4, 0x001e00eb },
    { 0x3ec, 0x00000014 }, { 0x3f0, 0x00280028 }, { 0x3f4, 0x00000007 },
    { 0x3f8, 0x00028005 }, { 0x3fc, 0x00000e0e },
};

/*
 * The threshold scenes' settings over the room's: no corners, a 15 x 15
 * window, luma threshold 1, chroma 3
 */
static const Reg nr_threshold_regs[] = {
    { 0x208, 0 }, { 0x370, 0 }, { 0x1d0, 0x00000203 },
    { 0x1d4, 0x1ff07401 }, { 0x348, 0x00030003 },
};

static const NrCase nr_room = {
    nr_room_regs, ARRAY_SIZE(nr_room_regs), NR_MOVING, 0,
    { 0x8513413e, 0xf47625b1, 0xf95b7d29, 0x77d8913a, 0x441d6a4c },
    { 0x94cc17b5, 0x4d04a3c4, 0x2bc08d84, 0xbe2dafd7, 0x08ebaf09 },
    true,
    {
        { { 0, 0, 1, 1, 1 }, { 1, 0, 1, 1, 1 }, { 1, 1, 1, 1, 1 } },
        { { 1, 0, 2, 2, 2 }, { 2, 0, 0, 2, 2 }, { 2, 2, 2, 2, 2 } },
        { { 2, 1, 3, 3, 3 }, { 3, 0, 0, 3, 3 }, { 3, 3, 0, 3, 3 } },
        { { 3, 2, 4, 4, 4 }, { 4, 0, 0, 4, 4 }, { 4, 4, 0, 4, 4 } },
    },
};

static const NrCase nr_gain = {
    nr_gain_regs, ARRAY_SIZE(nr_gain_regs), NR_MOVING, 0,
    { 0x050209ea, 0x631ff0bb, 0xdfff048b, 0x6c346107, 0x8a8b33fb },
    { 0x02c9c903, 0x71e5a03a, 0xe7ce7474, 0x12629a90, 0x923019c0 },
    true,
    {
        { { 0, 0, 1, 1, 1 }, { 0, 0, 1, 1, 1 }, { 1, 0, 1, 1, 1 } },
        { { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 }, { 2, 0, 0, 0, 0 } },
        { { 1, 0, 1, 0, 0 }, { 1, 0, 0, 0, 0 }, { 3, 0, 0, 0, 0 } },
        { { 2, 0, 2, 1, 1 }, { 2, 0, 0, 1, 1 }, { 4, 0, 0, 1, 1 } },
    },
};

static const NrCase nr_still_room = {
    nr_room_regs, ARRAY_SIZE(nr_room_regs), NR_STILL, 0,
    { 0xd2283736, 0xc2b226db, 0x2219a46f, 0x20967674, 0x2b4687d4 },
    { 0x5297599f, 0x7755b7fc, 0x420fbae0, 0xda121e4f, 0xbd700695 },
    true,
    {
        { { 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1 } },
        { { 2, 2, 0, 2, 2 }, { 2, 2, 0, 2, 2 }, { 2, 2, 0, 2, 2 } },
        { { 3, 3, 1, 3, 3 }, { 3, 3, 1, 3, 3 }, { 3, 3, 1, 3, 3 } },
        { { 4, 4, 0, 4, 4 }, { 4, 4, 0, 4, 4 }, { 4, 4, 0, 4, 4 } },
    },
};

static const NrCase nr_still_gain = {
    nr_gain_regs, ARRAY_SIZE(nr_gain_regs), NR_STILL, 0,
    { 0xb4907377, 0x154ede24, 0x184b86da, 0xf87d9f61, 0x43803bf0 },
    { 0x5297599f, 0x4e8e8ecd, 0x582e65f4, 0xc396bc84, 0xff72fb2c },
    true,
    {
        { { 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 1 } },
        { { 2, 0, 0, 0, 2 }, { 2, 2, 0, 0, 2 }, { 2, 2, 0, 0, 2 } },
        { { 3, 1, 1, 1, 3 }, { 3, 3, 1, 1, 3 }, { 3, 3, 1, 1, 3 } },
        { { 4, 0, 0, 0, 4 }, { 4, 4, 0, 0, 4 }, { 4, 4, 0, 0, 4 } },
    },
};

static const NrCase nr_threshold_bright = {
    nr_room_regs, ARRAY_SIZE(nr_room_regs), NR_THRESHOLD, 189,
    { 0, 0, 0x7fa06776 }, { 0, 0, 0x35f13997 },
};

static const NrCase nr_threshold_mid = {
    nr_room_regs, ARRAY_SIZE(nr_room_regs), NR_THRESHOLD, 172,
    { 0, 0, 0x765bda9c }, { 0, 0, 0x8a78599c },
};

static uint32_t lcg(uint32_t *x)
{
    *x = *x * 1103515245 + 12345;
    return (*x >> 16) & 0x7fff;
}

static uint8_t clamp8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

static void nr_moving_frame(uint8_t *f, int n)
{
    uint32_t x = 0x1234 + n;
    int bx = 20 + 9 * n, by = 16 + 3 * n;

    for (int r = 0; r < NH; r++) {
        for (int c = 0; c < NW; c++) {
            int v = 40 + c + r / 2 + (int)(lcg(&x) % 21) - 10;

            if (c >= bx && c < bx + 24 && r >= by && r < by + 20) {
                v += 60;
            }
            if (c >= 96 && n == 2) {
                v += 25;
            }
            f[r * NW + c] = clamp8(v);
        }
    }
    for (int r = 0; r < NH / 2; r++) {
        for (int c = 0; c < NW / 2; c++) {
            int u = 100 + c + (int)(lcg(&x) % 17) - 8;
            int v = 150 - r + (int)(lcg(&x) % 17) - 8;

            if (c >= bx / 2 && c < bx / 2 + 12 && r >= by / 2 &&
                r < by / 2 + 10) {
                u -= 40;
                v += 30;
            }
            f[NW * NH + r * NW + 2 * c] = clamp8(u);
            f[NW * NH + r * NW + 2 * c + 1] = clamp8(v);
        }
    }
}

static void nr_still_frame(uint8_t *f, int n)
{
    uint32_t x = 0x5678 + n;

    for (int r = 0; r < NH; r++) {
        for (int c = 0; c < NW; c++) {
            int v = 60 + c + r / 2 + (int)(lcg(&x) % 5) - 2;

            if (c >= 56 && c < 84 && (n == 2 || n == 3)) {
                v += 40;
            }
            f[r * NW + c] = clamp8(v);
        }
    }
    for (int r = 0; r < NH / 2; r++) {
        for (int c = 0; c < NW / 2; c++) {
            int u = 110 + c / 2 + (int)(lcg(&x) % 5) - 2;
            int v = 140 - r + (int)(lcg(&x) % 5) - 2;

            f[NW * NH + r * NW + 2 * c] = clamp8(u);
            f[NW * NH + r * NW + 2 * c + 1] = clamp8(v);
        }
    }
}

/*
 * A threshold scene's frame (cur flat at the level, chroma at a level off
 * it) or, with ref, its reference: 32 pixels 16 apart and 32 samples of
 * each chroma plane 8 apart, off by 48..63 (luma) and 23..38 (chroma), the
 * first 16 below the frame and the rest above
 */
static void nr_threshold_frame(uint8_t *f, int level, bool ref)
{
    int cl = level - 85;
    uint8_t *uv = f + NW * NH;

    memset(f, level, NW * NH);
    for (int i = 0; i < NW * NH / 2; i += 2) {
        uv[i] = cl;
        uv[i + 1] = 255 - cl;
    }
    if (!ref) {
        return;
    }
    for (int k = 0; k < 32; k++) {
        int d = k < 16 ? 48 + k % 16 : -(48 + k % 16);
        int dc = k < 16 ? 23 + k % 16 : -(23 + k % 16);
        int row = 4 + 8 * (k / 8), col = 4 + 8 * (k % 8);

        f[(8 + 16 * (k / 8)) * NW + 8 + 16 * (k % 8)] = level - d;
        uv[row * NW + 2 * col] = cl - dc;
        uv[row * NW + 2 * col + 1] = 255 - cl + dc;
    }
}

static uint32_t fnv1a(const uint8_t *p, size_t len)
{
    uint32_t h = 0x811c9dc5;

    for (size_t i = 0; i < len; i++) {
        h = (h ^ p[i]) * 0x01000193;
    }
    return h;
}

static void test_nr(const void *data)
{
    const NrCase *nc = data;
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint8_t in[NSIZE], out[NSIZE];
    int frames = nc->scene == NR_THRESHOLD ? 3 : 5;

    cpm_reset(qts);
    for (int i = 0; i < nc->nregs; i++) {
        wr(qts, nc->regs[i].reg, nc->regs[i].val);
    }
    for (int i = 0; nc->scene == NR_THRESHOLD &&
                    i < ARRAY_SIZE(nr_threshold_regs); i++) {
        wr(qts, nr_threshold_regs[i].reg, nr_threshold_regs[i].val);
    }
    wr(qts, 0x030, CUR);
    wr(qts, 0x040, CUR + NW * NH);
    wr(qts, 0x038, REF0);
    wr(qts, 0x048, REF0 + NW * NH);
    wr(qts, 0x050, REF0);
    wr(qts, 0x058, REF0 + NW * NH);
    for (uint32_t r = 0x034; r <= 0x05c; r += 8) {
        wr(qts, r, NW);
    }
    wr(qts, 0x02c, NH << 16 | NW);
    wr(qts, 0x08c, STA);
    wr(qts, 0x090, STA_STRIDE);
    wr(qts, 0x094, STA_SLOT);
    wr(qts, 0x2c4, NBX << 16 | NBY);
    wr(qts, 0x400, NBX << 16 | NBY);
    wr(qts, 0x000, 0);
    wr(qts, 0x00c, 5);
    wr(qts, 0x004, 2);
    for (int n = 0; n < frames; n++) {
        switch (nc->scene) {
        case NR_MOVING:
            nr_moving_frame(in, n);
            break;
        case NR_STILL:
            nr_still_frame(in, n);
            break;
        case NR_THRESHOLD:
            if (n == 2) {
                uint8_t ages[0x1000];

                nr_threshold_frame(in, nc->level, true);
                qtest_memwrite(qts, REF0, in, NSIZE);
                memset(ages, 5, sizeof(ages));
                qtest_memwrite(qts, STA, ages, sizeof(ages));
            }
            nr_threshold_frame(in, nc->level, false);
            break;
        }
        qtest_memwrite(qts, CUR, in, NSIZE);
        wr(qts, 0x004, rd(qts, 0x004) | 1);
        g_assert_cmphex(rd(qts, 0x004) & 0x10000, ==, 0x10000);
        qtest_memread(qts, REF0, out, NSIZE);
        if (nc->luma[n]) {
            g_assert_cmphex(fnv1a(out, NW * NH), ==, nc->luma[n]);
        }
        if (nc->frame[n]) {
            g_assert_cmphex(fnv1a(out, NSIZE), ==, nc->frame[n]);
        }
        if (n && nc->has_sta) {
            uint8_t row[STA_STRIDE];
            uint32_t slot = STA + (n - 1) * STA_SLOT;

            qtest_memread(qts, slot, row, 8);
            g_assert_cmpuint(row[0], ==, n + 1);
            for (int by = 0; by < NBY; by++) {
                qtest_memread(qts, slot + (by + 1) * STA_STRIDE, row, 8);
                for (int bx = 0; bx < NBX; bx++) {
                    g_assert_cmpuint(row[bx], ==, nc->sta[n - 1][by][bx]);
                }
            }
            /* the write slot, then the one read */
            g_assert_cmphex(rd(qts, 0x020) & 0xffff, ==,
                            (n - 1) << 8 | (n > 1 ? n - 2 : 0));
        }
        wr(qts, 0x010, 3);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/t30-ncu/regs", test_regs);
    qtest_add_func("ingenic/t30-ncu/control", test_control);
    qtest_add_func("ingenic/t30-ncu/frames", test_frames);
    qtest_add_func("ingenic/t30-ncu/hang", test_hang);
    qtest_add_data_func("ingenic/t30-ncu/nr-room", &nr_room, test_nr);
    qtest_add_data_func("ingenic/t30-ncu/nr-gain", &nr_gain, test_nr);
    qtest_add_data_func("ingenic/t30-ncu/nr-still-room", &nr_still_room,
                        test_nr);
    qtest_add_data_func("ingenic/t30-ncu/nr-still-gain", &nr_still_gain,
                        test_nr);
    qtest_add_data_func("ingenic/t30-ncu/nr-threshold-bright",
                        &nr_threshold_bright, test_nr);
    qtest_add_data_func("ingenic/t30-ncu/nr-threshold-mid",
                        &nr_threshold_mid, test_nr);
    return g_test_run();
}
