/*
 * QTest testcase for the T41 ISP's registers out of reset
 *
 * What a T41NQ's ISP reads at power-on (under U-Boot, its clock divider
 * set and its gate opened) and again after tx-isp has run and SRBC's bit
 * 22 has reset it: a word from each 256 bytes that hold any, the last of
 * each 4 KiB, and registers that read 0. Then the bits a write keeps, as
 * the T41NQ kept them with its clock up and no frames running, and the
 * words it latched only at a frame or on 0x1000 while it streamed (its live
 * channels' measured with that channel disabled, the MSCA's own with all
 * three), its shadow port and its reset through 0x24. Then the core
 * stopped by the idle request (0x200) or 0x24 bit 2 and restarted by bit 2
 * and a latch (0x1000), as a streaming T41NQ does.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "libqtest.h"

#define CPM_CLKGR       0x10000020
#define CPM_CIMCDR      0x10000090
#define CPM_SRBC        0x100000c4
#define ISP             0x13300000
#define LATCH_NOW       0x01000
#define VIC_CTRL        0x80000
#define MSCA_LATCH      0xf0010
#define SHD_OPEN        0xf8000
#define SHD_PAIR        0xf8004
#define SHD_LAST        0xf8008
#define SHD_STATE       0xf800c
#define CORE_RESET      0x00024
#define SHD_RUN         0xf8100

#define SRBC_ISP_STOP   (1u << 21)
#define SRBC_ISP_SR     (1u << 22)

static const struct {
    uint32_t off, val;
} reset[] = {
    { 0x00000, 0x20220304 }, { 0x00204, 0x00000001 }, { 0x00400, 0x01000000 },
    { 0x00500, 0x01000000 }, { 0x005b8, 0xffffffff }, { 0x01008, 0x000003ce },
    { 0x01138, 0x00010001 }, { 0x02004, 0x00420042 }, { 0x02040, 0x00010001 },
    { 0x03000, 0x00400040 }, { 0x03118, 0x02009944 }, { 0x03130, 0x00010001 },
    { 0x04004, 0x04000400 }, { 0x04010, 0x04000400 }, { 0x05004, 0x04000400 },
    { 0x05010, 0x04000400 }, { 0x06000, 0x20200813 }, { 0x06100, 0x00000ff0 },
    { 0x06200, 0x0ff00ff0 }, { 0x06300, 0x0000017d }, { 0x06400, 0x00680068 },
    { 0x0649c, 0x00000040 }, { 0x07004, 0x00001010 }, { 0x071a0, 0x82828282 },
    { 0x07200, 0x01d4035c }, { 0x07300, 0x04ffffff }, { 0x07308, 0x00000001 },
    { 0x08000, 0x04000400 }, { 0x08060, 0x00010001 }, { 0x09000, 0x20200629 },
    { 0x09100, 0x085408fe }, { 0x09200, 0x00660033 }, { 0x09300, 0x012c00c9 },
    { 0x09400, 0x00ec00d7 }, { 0x09550, 0x01000002 }, { 0x0959c, 0x00010001 },
    { 0x0a004, 0x8000800a }, { 0x0a100, 0x20820820 }, { 0x0a200, 0x00001fff },
    { 0x0a2e8, 0x00050050 }, { 0x0b004, 0x00000400 }, { 0x0b028, 0x00010001 },
    { 0x0c000, 0x20200620 }, { 0x0c100, 0x07060505 }, { 0x0c204, 0x0c060200 },
    { 0x0c300, 0x00000101 }, { 0x0c318, 0x00010001 }, { 0x0d010, 0x06480906 },
    { 0x0d04c, 0x00010001 }, { 0x0e000, 0x00d800d4 }, { 0x0e04c, 0x00010001 },
    { 0x0f000, 0x20191209 }, { 0x0f100, 0x00000010 }, { 0x0f234, 0x0000ff00 },
    { 0x0f300, 0x00800000 }, { 0x0f40c, 0x00010000 }, { 0x0f440, 0x00000100 },
    { 0x10000, 0x20191224 }, { 0x10080, 0x00010001 }, { 0x11000, 0x000003fc },
    { 0x11094, 0x00010001 }, { 0x12008, 0x070f059a }, { 0x12040, 0x00030001 },
    { 0x13000, 0x00060000 }, { 0x13100, 0x03ff03ff }, { 0x13188, 0x7f7f7f7f },
    { 0x14000, 0x00010000 }, { 0x14094, 0x00000006 }, { 0x18004, 0xf001f001 },
    { 0x180a8, 0x06ee04fa }, { 0x19004, 0xf001f001 }, { 0x19118, 0x00400000 },
    { 0x19120, 0x00400000 }, { 0x1a004, 0xf001f003 }, { 0x1a0dc, 0x00000013 },
    { 0x1b020, 0x00004080 }, { 0x1b060, 0x00004080 }, { 0x1c000, 0x20200626 },
    { 0x1c078, 0x00010001 }, { 0x1d004, 0x00001010 }, { 0x1d1a0, 0x82828282 },
    { 0x1d200, 0x01d4035c }, { 0x1d300, 0x04ffffff }, { 0x1d308, 0x00000001 },
    { 0x1e000, 0x20211030 }, { 0x1e100, 0x0bc809c8 }, { 0x1e118, 0x00090909 },
    { 0x1f000, 0x00080000 }, { 0x1f018, 0x10000000 }, { 0x20000, 0x20211118 },
    { 0x20048, 0x00000001 }, { 0x21000, 0x20211210 }, { 0x21074, 0x00004001 },
    { 0x50400, 0x020200ec }, { 0x50414, 0x016b0000 }, { 0x60028, 0x00000200 },
    { 0x60128, 0x00000200 }, { 0x601a8, 0x00000002 }, { 0x70000, 0x00000100 },
    { 0x80004, 0x07800438 }, { 0x80100, 0x000002d0 }, { 0x80304, 0x07800438 },
    { 0x80400, 0x00310001 }, { 0xf002c, 0x00000007 }, { 0xf0100, 0x07800438 },
    { 0xf0200, 0x050002d0 }, { 0xf0300, 0x028001e0 }, { 0xf0704, 0x0384012c },
    { 0xf0840, 0x01000100 }, { 0xf08f8, 0xffffffff }, { 0xf2000, 0x01000000 },
    { 0xf2104, 0x00040000 }, { 0xf21a0, 0x07804000 }, { 0xff004, 0x00000001 },
    { 0xff020, 0x00002200 },
    /* the HLDC's bypass clear, interrupts, a VIC's and the MSCA's enables */
    { 0x00040, 0 }, { 0x40020, 0 }, { 0x40028, 0 }, { 0x80000, 0 },
    { 0xf0008, 0 },
};

static void check(QTestState *qts)
{
    for (int i = 0; i < ARRAY_SIZE(reset); i++) {
        g_assert_cmphex(qtest_readl(qts, ISP + reset[i].off), ==,
                        reset[i].val);
    }
}

static void test_power_on(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    check(qts);
    qtest_quit(qts);
}

/* tx-isp's private_reset_tx_isp_module: stop, wait for the ack, reset */
static void test_soft_reset(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    for (int i = 0; i < ARRAY_SIZE(reset); i++) {
        if (reset[i].off != 0x80000) {
            qtest_writel(qts, ISP + reset[i].off, ~reset[i].val);
        }
    }
    qtest_writel(qts, CPM_SRBC, SRBC_ISP_STOP);
    g_assert_true(qtest_readl(qts, CPM_SRBC) & SRBC_ISP_STOP >> 1);
    qtest_writel(qts, CPM_SRBC, SRBC_ISP_SR);
    qtest_writel(qts, CPM_SRBC, 0);
    check(qts);
    qtest_quit(qts);
}

/* What a T41NQ reads after it writes all ones, then all zeros */
static const struct {
    uint32_t off, ones, zeros;
} masks[] = {
    /* core: the version word, the frame size, bypass sources, a hole */
    { 0x00000, 0x20220304, 0x20220304 }, { 0x00020, 0xfffffff1, 0x00000000 },
    { 0x00060, 0x000007ff, 0x00000000 }, { 0x00080, 0xffffffff, 0x00000000 },
    { 0x00088, 0x0011131f, 0x00000000 }, { 0x00200, 0x03ff1300, 0x00000000 },
    { 0x00a00, 0x00000000, 0x00000000 }, { 0x01020, 0x0003ffff, 0x00000000 },
    /* the tables */
    { 0x02004, 0x0fff0fff, 0x00000000 }, { 0x03008, 0x83ffffff, 0x00000000 },
    { 0x04004, 0x3fff3fff, 0x00000000 }, { 0x0602c, 0x01ff00ff, 0x00000000 },
    { 0x07004, 0x3f3f1f1f, 0x00000000 }, { 0x08000, 0x7fff7fff, 0x00000000 },
    { 0x09008, 0x01111111, 0x00000000 }, { 0x0a004, 0x8000ffff, 0x00000000 },
    { 0x0c008, 0x00ff00ff, 0x00000000 }, { 0x0d004, 0x07ffffff, 0x00000000 },
    { 0x0e004, 0x003f070f, 0x00000000 }, { 0x0f00c, 0x0000007f, 0x00000000 },
    { 0x11000, 0x03ff03ff, 0x00000000 }, { 0x12008, 0x0fff0fff, 0x00000000 },
    { 0x13008, 0x07fcbfff, 0x00000000 }, { 0x1400c, 0x03fe01ff, 0x00000000 },
    { 0x18018, 0x7f7f7f7f, 0x00000000 }, { 0x1a008, 0x01ff01ff, 0x00000000 },
    { 0x1b020, 0x0000ffff, 0x00000000 }, { 0x1c010, 0x0000000f, 0x00000000 },
    { 0x1d004, 0x3f3f1f1f, 0x00000000 }, { 0x1e030, 0x00000111, 0x00000000 },
    { 0x1f010, 0x0007ffff, 0x00000000 }, { 0x16000, 0x00000000, 0x00000000 },
    /* VIC, MSCA, OSD */
    { 0x8000c, 0x00000007, 0x00000000 }, { 0x80014, 0x0000003f, 0x00000000 },
    { 0xf0060, 0x00000333, 0x00000000 }, { 0xf0064, 0x00000f0f, 0x00000000 },
    { 0xf0138, 0x1fff0fff, 0x00000000 }, { 0xf0140, 0x00ffffff, 0x00000000 },
    { 0xf040c, 0x1fff0731, 0x00000000 }, { 0xf2108, 0x00010000, 0x00010000 },
};

static void test_masks(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    for (int i = 0; i < ARRAY_SIZE(masks); i++) {
        uint32_t a = ISP + masks[i].off;

        qtest_writel(qts, a, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, a), ==, masks[i].ones);
        qtest_writel(qts, a, 0);
        g_assert_cmphex(qtest_readl(qts, a), ==, masks[i].zeros);
    }
    qtest_quit(qts);
}

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, ISP + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, ISP + off, v);
}

/* A sensor at 1 fps, the ISP's clock up, its VIC started: the first frame */
static QTestState *stream(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-device image-sensor,model=gc4023,"
                                 "bus=i2c0,fps=1");

    qtest_writel(qts, CPM_CLKGR, qtest_readl(qts, CPM_CLKGR) & ~(1u << 23));
    qtest_writel(qts, CPM_CIMCDR,
                 qtest_readl(qts, CPM_CIMCDR) & ~(1u << 27));
    wr(qts, 0x80, 640 << 16 | 360);
    wr(qts, VIC_CTRL, 1);
    return qts;
}

static void next_frame(QTestState *qts)
{
    qtest_clock_step(qts, NANOSECONDS_PER_SECOND);
}

/*
 * Words a streaming T41NQ latched, what each read after all ones, and the
 * trigger that latched it (0 for 0x40 and 0x44, at every frame)
 */
static const struct {
    uint32_t off, trig, was, ones;
} latched[] = {
    { 0x00040, 0, 0x00000000, 0x03ffdfff },
    { 0x00044, 0, 0x000001ff, 0x000001ff },
    { 0x0a008, 0x0a19c, 0x00000032, 0x0fff0fff },
    { 0x0a0b0, 0x0a19c, 0x08080601, 0x080f8707 },
    { 0x1301c, 0x13080, 0x00019032, 0x0007f8ff },
    { 0x13050, 0x13080, 0x0666000a, 0x7ffeffff },
    { 0x14004, 0x14054, 0x026c007d, 0x0ffc03ff },
    { 0x14140, 0x14054, 0x00000000, 0x1f1f1f1f },
    { 0xf002c, 0xf0010, 0x00000007, 0x3f3fff77 },
    { 0xf0034, 0xf0010, 0x00000000, 0x0fff0fff },
    { 0xf0080, 0xf0010, 0x00000000, 0x1fff0fff },
    { 0xf0084, 0xf0010, 0x07800438, 0x1fff0fff },
    { 0xf0100, 0xf0010, 0x07800438, 0x1fff0fff },
    { 0xf0128, 0xf0010, 0x00000000, 0x1fff0fff },
    { 0xf012c, 0xf0010, 0x07800438, 0x1fff0fff },
    { 0xf01d0, 0xf0010, 0x00000000, 0x01ff01ff },
    { 0xf0200, 0xf0010, 0x050002d0, 0x1fff0fff },
    { 0xf0228, 0xf0010, 0x00000000, 0x1fff0fff },
    { 0xf022c, 0xf0010, 0x050002d0, 0x1fff0fff },
    { 0xf02d0, 0xf0010, 0x00000000, 0x01ff01ff },
    { 0xf0300, 0xf0010, 0x028001e0, 0x1fff0fff },
    { 0xf032c, 0xf0010, 0x028001e0, 0x1fff0fff },
    { 0xf03d0, 0xf0010, 0x00000000, 0x01ff01ff },
    { 0xf0720, 0xf0010, 0x00004000, 0x0007ffff },
    { 0xf0724, 0xf0010, 0x00004000, 0x0007ffff },
    { 0xf0728, 0xf0010, 0x00004000, 0x0007ffff },
    { 0xf072c, 0xf0010, 0x00004000, 0x0007ffff },
    { 0xf08e0, 0xf0010, 0x04000400, 0x07ff07ff },
};

/* With no frames running a write never shows, but 0x1000 latches 0x40/44 */
static void test_latch_idle(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    for (int i = 0; i < ARRAY_SIZE(latched); i++) {
        wr(qts, latched[i].off, 0xffffffff);
        if (latched[i].trig) {
            wr(qts, latched[i].trig, 1);
            g_assert_cmphex(rd(qts, latched[i].trig), ==, 1);
        }
        g_assert_cmphex(rd(qts, latched[i].off), ==, latched[i].was);
    }
    wr(qts, LATCH_NOW, 1);
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 0);
    for (int i = 0; i < ARRAY_SIZE(latched); i++) {
        g_assert_cmphex(rd(qts, latched[i].off), ==,
                        latched[i].trig ? latched[i].was : latched[i].ones);
    }
    qtest_quit(qts);
}

/* While it streams: at the next frame, a trigger's words once it is set */
static void test_latch_frame(void)
{
    QTestState *qts = stream();

    for (int i = 0; i < ARRAY_SIZE(latched); i++) {
        wr(qts, latched[i].off, 0xffffffff);
    }
    next_frame(qts);
    for (int i = 0; i < ARRAY_SIZE(latched); i++) {
        g_assert_cmphex(rd(qts, latched[i].off), ==,
                        latched[i].trig ? latched[i].was : latched[i].ones);
        if (latched[i].trig) {
            wr(qts, latched[i].trig, 1);
        }
    }
    /* the MDNS's and YDNS's triggers too; the CDNS's never clears */
    wr(qts, 0x0f004, 1);
    wr(qts, 0x10004, 1);
    wr(qts, 0x1c004, 1);
    g_assert_cmphex(rd(qts, 0xf0010), ==, 1);
    g_assert_cmphex(rd(qts, latched[2].off), ==, latched[2].was);
    next_frame(qts);
    for (int i = 0; i < ARRAY_SIZE(latched); i++) {
        g_assert_cmphex(rd(qts, latched[i].off), ==, latched[i].ones);
        if (latched[i].trig) {
            g_assert_cmphex(rd(qts, latched[i].trig), ==, 0);
        }
    }
    g_assert_cmphex(rd(qts, 0x0f004), ==, 0);
    g_assert_cmphex(rd(qts, 0x10004), ==, 0);
    g_assert_cmphex(rd(qts, 0x1c004), ==, 1);

    /* 0x1000 set while streaming: it stays, and 0x40 takes writes at once */
    wr(qts, LATCH_NOW, 1);
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 1);
    wr(qts, 0x40, 0x0398004b);
    g_assert_cmphex(rd(qts, 0x40), ==, 0x0398004b);
    wr(qts, LATCH_NOW, 0);
    wr(qts, 0x40, 0x03980049);
    g_assert_cmphex(rd(qts, 0x40), ==, 0x0398004b);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0x40), ==, 0x03980049);
    qtest_quit(qts);
}

static void shd_pairs(QTestState *qts, const uint32_t *pairs, int n)
{
    wr(qts, SHD_OPEN, 0x101);
    g_assert_cmphex(rd(qts, SHD_STATE), ==, 0x10);
    for (int i = 0; i < 2 * n; i++) {
        wr(qts, SHD_PAIR, pairs[i]);
    }
    wr(qts, SHD_OPEN, 0x1010102);
    g_assert_cmphex(rd(qts, SHD_STATE), ==, 1);
}

/* The shadow port: a RAM of pairs the ISP writes out at a frame's end */
static void test_shadow(void)
{
    QTestState *qts = stream();
    static const uint32_t two[] = {
        0x00010001, 0xf0338, 0x00020002, 0xf033c,
    };
    static const uint32_t one[] = { 0x00030003, 0xf0338 };
    static const uint32_t size[] = { 0x02000100, 0xf0300 };

    /* n - 1 in bits 15:4: only the first of the two */
    shd_pairs(qts, two, 2);
    wr(qts, SHD_RUN, 0x01);
    g_assert_cmphex(rd(qts, SHD_RUN), ==, 0);
    g_assert_cmphex(rd(qts, 0xf0338), ==, 0);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0338), ==, 0x00010001);
    g_assert_cmphex(rd(qts, 0xf033c), ==, 0);

    /* once a run; a new opening writes from entry 0, the rest stay */
    wr(qts, 0xf0338, 0);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0338), ==, 0);
    shd_pairs(qts, one, 1);
    wr(qts, SHD_PAIR, 0x00040004);      /* closed: dropped */
    wr(qts, SHD_PAIR, 0xf033c);
    wr(qts, SHD_RUN, 0x11);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0338), ==, 0x00030003);
    g_assert_cmphex(rd(qts, 0xf033c), ==, 0x00020002);

    /* 0 cancels a run */
    shd_pairs(qts, two, 2);
    wr(qts, 0xf0338, 0);
    wr(qts, SHD_RUN, 0x11);
    wr(qts, SHD_RUN, 0);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0338), ==, 0);

    /* a latched word it writes waits for its trigger as the CPU's would */
    shd_pairs(qts, size, 1);
    wr(qts, SHD_RUN, 0x01);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0300), ==, 0x028001e0);
    wr(qts, MSCA_LATCH, 1);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0300), ==, 0x02000100);
    qtest_quit(qts);
}

/*
 * 512 entries: past them the pairs go round to entry 0, and so does a run,
 * whose last value 0xf8008 reads (writes there do not stick)
 */
static void test_shadow_ram(void)
{
    QTestState *qts = stream();
    static const struct {
        uint32_t n, got;
    } run[] = {
        /* entry 0 holds pair 4608 (9 x 512), entry 511 pair 4607 */
        { 1, 0x1201 }, { 2, 0x1202 }, { 256, 0x1300 }, { 512, 0x1200 },
        { 513, 0x1201 }, { 1025, 0x1201 }, { 4096, 0x1200 },
    };

    wr(qts, SHD_LAST, 0x12345678);
    g_assert_cmphex(rd(qts, SHD_LAST), ==, 0);
    wr(qts, SHD_OPEN, 0x101);
    for (uint32_t i = 0; i < 5000; i++) {
        wr(qts, SHD_PAIR, i + 1);
        wr(qts, SHD_PAIR, 0xf0340);
    }
    wr(qts, SHD_OPEN, 0x1010102);
    for (int i = 0; i < ARRAY_SIZE(run); i++) {
        wr(qts, 0xf0340, 0);
        wr(qts, SHD_RUN, (run[i].n - 1) << 4 | 1);
        next_frame(qts);
        g_assert_cmphex(rd(qts, 0xf0340), ==, run[i].got);
        g_assert_cmphex(rd(qts, SHD_LAST), ==, run[i].got);
    }
    qtest_quit(qts);
}

/*
 * 0x24 keeps bits 31:24; bit 0 resets all but the VIC, which still sends
 * frames, and closes the shadow port, whose RAM stays
 */
static void test_core_reset(void)
{
    QTestState *qts = stream();
    static const uint32_t one[] = { 0x00000005, 0xf0340 };

    wr(qts, CORE_RESET, 0xfffffffa);
    g_assert_cmphex(rd(qts, CORE_RESET), ==, 0xff000000);
    wr(qts, CORE_RESET, 0);
    g_assert_cmphex(rd(qts, CORE_RESET), ==, 0);

    shd_pairs(qts, one, 1);
    wr(qts, SHD_RUN, 0x01);
    next_frame(qts);
    g_assert_cmphex(rd(qts, SHD_LAST), ==, 5);
    wr(qts, SHD_OPEN, 0x101);
    wr(qts, SHD_PAIR, 0x77);
    wr(qts, SHD_PAIR, 0xf0340);
    for (int i = 0; i < ARRAY_SIZE(reset); i++) {
        if ((reset[i].off & 0xff000) != 0x80000) {
            wr(qts, reset[i].off, ~reset[i].val);
        }
    }
    wr(qts, 0x80004, 0x0a0005a0);
    wr(qts, CORE_RESET, 0xff000001);
    g_assert_cmphex(rd(qts, CORE_RESET), ==, 0);
    for (int i = 0; i < ARRAY_SIZE(reset); i++) {
        if ((reset[i].off & 0xff000) != 0x80000) {
            g_assert_cmphex(rd(qts, reset[i].off), ==, reset[i].val);
        }
    }
    g_assert_cmphex(rd(qts, 0x80004), ==, 0x0a0005a0);
    g_assert_cmphex(rd(qts, SHD_STATE), ==, 0);
    g_assert_cmphex(rd(qts, SHD_LAST), ==, 5);

    /* a close with the port not open leaves the state at 0 */
    wr(qts, SHD_OPEN, 0x1010102);
    g_assert_cmphex(rd(qts, SHD_STATE), ==, 0);
    wr(qts, SHD_RUN, 0x01);
    next_frame(qts);
    g_assert_cmphex(rd(qts, 0xf0340), ==, 0x77);
    g_assert_cmphex(rd(qts, SHD_LAST), ==, 0x77);
    qtest_quit(qts);
}

#define IDLE_REQ        0x00200
#define IDLE            0x00204
#define CORE_STA        0x40028
#define CORE_CLR        0x40024
#define CORE_START      (1u << 20)
#define VIC_STA         0x801e0
#define VIC_CLR         0x801f0

/* Whether the core and the VIC took the last frame; clears both */
static bool core_took(QTestState *qts)
{
    bool took = rd(qts, CORE_STA) & CORE_START;

    g_assert_true(rd(qts, VIC_STA) & 1);
    wr(qts, CORE_CLR, ~0u);
    wr(qts, VIC_CLR, ~0u);
    return took;
}

static void half_frame(QTestState *qts)
{
    qtest_clock_step(qts, NANOSECONDS_PER_SECOND / 2);
}

static void test_idle(void)
{
    QTestState *qts = stream();
    static const uint32_t one[] = { 0x00000005, 0xf0340 };

    next_frame(qts);
    g_assert_true(core_took(qts));

    /* the request reads back 0 and stops the frame in flight */
    half_frame(qts);
    wr(qts, IDLE_REQ, 1);
    g_assert_cmphex(rd(qts, IDLE_REQ), ==, 0);
    g_assert_cmphex(rd(qts, IDLE), ==, 1);
    half_frame(qts);
    g_assert_false(core_took(qts));

    /* a latch alone does not restart it, nor run the shadow port */
    wr(qts, LATCH_NOW, 1);
    shd_pairs(qts, one, 1);
    wr(qts, SHD_RUN, 0x01);
    next_frame(qts);
    g_assert_false(core_took(qts));
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 1);
    g_assert_cmphex(rd(qts, SHD_LAST), ==, 0);

    /* bit 2 takes the pending latch: the frame in flight lost, the next out */
    half_frame(qts);
    wr(qts, CORE_RESET, 4);
    wr(qts, CORE_RESET, 0);
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 0);
    half_frame(qts);
    g_assert_false(core_took(qts));
    next_frame(qts);
    g_assert_true(core_took(qts));
    g_assert_cmphex(rd(qts, SHD_LAST), ==, 5);

    /* bit 2 alone, no latch pending: stopped until one */
    half_frame(qts);
    wr(qts, CORE_RESET, 4);
    wr(qts, CORE_RESET, 0);
    half_frame(qts);
    g_assert_false(core_took(qts));
    next_frame(qts);
    g_assert_false(core_took(qts));
    half_frame(qts);
    wr(qts, LATCH_NOW, 1);
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 0);
    half_frame(qts);
    g_assert_false(core_took(qts));
    next_frame(qts);
    g_assert_true(core_took(qts));

    /* the driver's order: request, bit 2, then the latch */
    half_frame(qts);
    wr(qts, IDLE_REQ, 1);
    wr(qts, CORE_RESET, 4);
    wr(qts, CORE_RESET, 0);
    wr(qts, 0x01004, 1);
    wr(qts, LATCH_NOW, 1);
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 0);
    half_frame(qts);
    g_assert_false(core_took(qts));
    next_frame(qts);
    g_assert_true(core_took(qts));

    /* bits 1 and 3 to 23: frames go on, a pending latch stays */
    wr(qts, LATCH_NOW, 1);
    half_frame(qts);
    wr(qts, CORE_RESET, 0x00fffffa);
    g_assert_cmphex(rd(qts, LATCH_NOW), ==, 1);
    half_frame(qts);
    g_assert_true(core_took(qts));
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-t41-isp/power-on", test_power_on);
    qtest_add_func("/ingenic-t41-isp/soft-reset", test_soft_reset);
    qtest_add_func("/ingenic-t41-isp/masks", test_masks);
    qtest_add_func("/ingenic-t41-isp/latch-idle", test_latch_idle);
    qtest_add_func("/ingenic-t41-isp/latch-frame", test_latch_frame);
    qtest_add_func("/ingenic-t41-isp/shadow", test_shadow);
    qtest_add_func("/ingenic-t41-isp/shadow-ram", test_shadow_ram);
    qtest_add_func("/ingenic-t41-isp/core-reset", test_core_reset);
    qtest_add_func("/ingenic-t41-isp/idle", test_idle);
    return g_test_run();
}
