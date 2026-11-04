/*
 * QTest testcase for the T41 ISP's registers out of reset
 *
 * What a T41NQ's ISP reads at power-on (under U-Boot, its clock divider
 * set and its gate opened) and again after tx-isp has run and SRBC's bit
 * 22 has reset it: a word from each 256 bytes that hold any, the last of
 * each 4 KiB, and registers that read 0. Then the bits a write keeps, as
 * the T41NQ kept them with its clock up and no frames running.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define CPM_SRBC        0x100000c4
#define ISP             0x13300000

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

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-t41-isp/power-on", test_power_on);
    qtest_add_func("/ingenic-t41-isp/soft-reset", test_soft_reset);
    qtest_add_func("/ingenic-t41-isp/masks", test_masks);
    return g_test_run();
}
