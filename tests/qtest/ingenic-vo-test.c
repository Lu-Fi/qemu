/*
 * QTest testcase for the Ingenic VO (BT.656/BT.1120 output)
 *
 * The T41's VO and the T40's older one: resets, the bits kept, TOP_CON's
 * reset (read inverted on the T40, and holding its registers), the queue
 * of four address pairs, a pair taken as TRAN_EN starts and at each
 * frame's end with INT_VAL bit 0, the frame length of h + vfb + vbb lines
 * of 2w + hfb + 8 (T40: 16) clocks, the T41's underrun after a repeated
 * frame, and the frame on the console. As a T41NQ and a T40XP do at
 * U-Boot.
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
#define CPM_BT0CDR      0xb8
#define VO              0x130c0000
#define VO_VER          0x000
#define VO_TOP_CON      0x004
#define VO_INT_EN       0x00c
#define VO_INT_VAL      0x010
#define VO_INT_CLC      0x014
#define VO_SIZE         0x100
#define VO_STRIDE       0x104
#define VO_Y_ADDR       0x108
#define VO_UV_ADDR      0x10c
#define VO_CON_PAR      0x114
#define VO_VBLK         0x118
#define VO_HBLK         0x11c
#define VO_TRAN_EN      0x120
#define VO_ADDR_INFO    0x128
#define VO_YUV_CLIP     0x130

#define FRAME           0x01000000
#define W               256
#define H               16

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, VO + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, VO + off, v);
}

/*
 * MPLL as U-Boot sets it (T41NQ 1400 MHz, T40XP 1200 MHz), over 22 into the
 * VO, its gate opened
 */
static void vo_clock(QTestState *qts, bool t41)
{
    qtest_writel(qts, CPM + CPM_CPMPCR, t41 ? 0x15d0889d : 0x0640510d);
    qtest_writel(qts, CPM + CPM_BT0CDR, 1 << 30 | 1 << 29 | 21);
    qtest_writel(qts, CPM + CPM_CLKGR,
                 qtest_readl(qts, CPM + CPM_CLKGR) & ~(1u << 26));
}

static void vo_setup(QTestState *qts)
{
    wr(qts, VO_SIZE, H << 16 | W);
    wr(qts, VO_STRIDE, W << 16 | W);
    wr(qts, VO_VBLK, 255 << 16 | 255);
    wr(qts, VO_HBLK, 1023);
}

static void vo_pair(QTestState *qts, uint32_t y, uint32_t uv)
{
    wr(qts, VO_Y_ADDR, y);
    wr(qts, VO_UV_ADDR, uv);
}

/* 526 lines of 2 * 256 + 1023 + 8 (T40: 16) clocks at the MPLL over 22 */
static int64_t frame_ns(bool t41)
{
    return 526LL * (2 * W + 1023 + (t41 ? 8 : 16)) * 22 * 1000 /
           (t41 ? 1400 : 1200);
}

static void test_regs_t41(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");

    g_assert_cmphex(rd(qts, VO_VER), ==, 0x20211101);
    g_assert_cmphex(rd(qts, VO_SIZE), ==, 0x04380780);
    g_assert_cmphex(rd(qts, VO_CON_PAR), ==, 0x20000);
    g_assert_cmphex(rd(qts, VO_YUV_CLIP), ==, 0xfe01fe01);
    g_assert_cmphex(rd(qts, VO_ADDR_INFO), ==, 0x15);
    wr(qts, VO_INT_EN, 0xffffffff);
    g_assert_cmphex(rd(qts, VO_INT_EN), ==, 0x111111);
    wr(qts, VO_CON_PAR, 0xffffffff);
    g_assert_cmphex(rd(qts, VO_CON_PAR), ==, 0x01ff34ff);
    wr(qts, VO_Y_ADDR, 0x12345678);
    g_assert_cmphex(rd(qts, VO_Y_ADDR), ==, 0);
    wr(qts, VO_TOP_CON, 0xffffffff);
    g_assert_cmphex(rd(qts, VO_TOP_CON), ==, 2);
    wr(qts, VO_TOP_CON, 1);
    g_assert_cmphex(rd(qts, VO_TOP_CON), ==, 1);
    qtest_quit(qts);
}

static void test_regs_t40(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    g_assert_cmphex(rd(qts, VO_VER), ==, 0x28);
    g_assert_cmphex(rd(qts, VO_TOP_CON), ==, 2);
    g_assert_cmphex(rd(qts, VO_ADDR_INFO), ==, 0x25);
    g_assert_cmphex(rd(qts, VO_YUV_CLIP), ==, 0);

    /* in reset (bit 1 written, read inverted) nothing takes a write */
    wr(qts, VO_TOP_CON, 2);
    g_assert_cmphex(rd(qts, VO_TOP_CON), ==, 0);
    wr(qts, VO_SIZE, 0);
    g_assert_cmphex(rd(qts, VO_SIZE), ==, 0x04380780);
    wr(qts, VO_TOP_CON, 0);
    wr(qts, VO_INT_EN, 0xffffffff);
    g_assert_cmphex(rd(qts, VO_INT_EN), ==, 0x3);
    wr(qts, VO_CON_PAR, 0xffffffff);
    g_assert_cmphex(rd(qts, VO_CON_PAR), ==, 0x01ff0fff);
    qtest_quit(qts);
}

/* The A1's BT output is the T40's VO at 0x130e0000 */
static void test_regs_a1(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(qtest_readl(qts, 0x130e0000 + VO_VER), ==, 0x28);
    g_assert_cmphex(qtest_readl(qts, 0x130e0000 + VO_TOP_CON), ==, 2);
    g_assert_cmphex(qtest_readl(qts, 0x130e0000 + VO_ADDR_INFO), ==, 0x25);
    qtest_quit(qts);
}

static void test_queue(const void *data)
{
    bool t41 = !strcmp(data, "t41");
    g_autofree char *args = g_strdup_printf("-machine ingenic-%s", t41 ? "t41"
                                                                  : "t40");
    QTestState *qts = qtest_init(args);
    int64_t ns = frame_ns(t41);

    vo_clock(qts, t41);
    vo_setup(qts);
    for (int i = 0; i < 6; i++) {
        vo_pair(qts, FRAME, FRAME + W * H);
        g_assert_cmphex(rd(qts, VO_ADDR_INFO), ==,
                        (t41 ? 0x10 : 0x20) | (i >= 3 ? 0xa : 0));
    }

    /* the first pair goes as TRAN_EN starts, the others a frame apart */
    wr(qts, VO_INT_EN, 1);
    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    wr(qts, VO_TRAN_EN, 1);
    g_assert_cmphex(rd(qts, VO_INT_VAL), ==, 1);
    g_assert_cmphex(rd(qts, VO_ADDR_INFO), ==, 0x2);
    for (int i = 1; i < 4; i++) {
        wr(qts, VO_INT_CLC, 1);
        wr(qts, VO_INT_CLC, 0);
        qtest_clock_step(qts, ns - 1000);
        g_assert_cmphex(rd(qts, VO_INT_VAL), ==, 0);
        qtest_clock_step(qts, 1000);
        g_assert_cmphex(rd(qts, VO_INT_VAL), ==, 1);
    }

    /* dry: the T41 sends the last frame again, then underruns */
    wr(qts, VO_INT_CLC, 1);
    wr(qts, VO_INT_CLC, 0);
    qtest_clock_step(qts, ns);
    g_assert_cmphex(rd(qts, VO_INT_VAL), ==, 0);
    qtest_clock_step(qts, ns);
    g_assert_cmphex(rd(qts, VO_INT_VAL), ==, t41 ? 0x10000 : 0);
    g_assert_cmphex(rd(qts, VO_ADDR_INFO), ==, t41 ? 0x82 : 0x2);

    /* the reset empties the queue and drops TRAN_EN */
    wr(qts, VO_TOP_CON, 2);
    wr(qts, VO_TOP_CON, 0);
    g_assert_cmphex(rd(qts, VO_TRAN_EN), ==, 0);
    g_assert_cmphex(rd(qts, VO_INT_VAL), ==, 0);
    g_assert_cmphex(rd(qts, VO_ADDR_INFO), ==, t41 ? 0x15 : 0x25);
    qtest_quit(qts);
}

/* A white top half over a black bottom one, as the console shows it */
static void test_console(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    g_autofree char *path = g_strdup_printf("%s/vo-XXXXXX.ppm",
                                            g_get_tmp_dir());
    g_autofree uint8_t *img = g_malloc(W * H * 3 / 2);
    g_autofree gchar *ppm = NULL;
    gsize len;
    int fd = g_mkstemp(path);
    unsigned pw, ph, max, hdr;
    QDict *rsp;

    g_assert(fd >= 0);
    close(fd);
    memset(img, 235, W * H / 2);
    memset(img + W * H / 2, 16, W * H / 2);
    memset(img + W * H, 128, W * H / 2);
    qtest_memwrite(qts, FRAME, img, W * H * 3 / 2);
    vo_clock(qts, true);
    vo_setup(qts);
    vo_pair(qts, FRAME, FRAME + W * H);
    wr(qts, VO_TRAN_EN, 1);
    rsp = qtest_qmp(qts, "{'execute': 'screendump', 'arguments': "
                    "{'filename': %s}}", path);
    qobject_unref(rsp);
    g_assert(g_file_get_contents(path, &ppm, &len, NULL));
    unlink(path);
    g_assert_cmpint(sscanf(ppm, "P6 %u %u %u %n", &pw, &ph, &max, &hdr), ==,
                    3);
    g_assert_cmpuint(pw, ==, W);
    g_assert_cmpuint(ph, ==, H);
    g_assert_cmpuint((uint8_t)ppm[hdr], ==, 255);
    g_assert_cmpuint((uint8_t)ppm[hdr + (W * (H - 1)) * 3 + 1], ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-vo/regs/t41", test_regs_t41);
    qtest_add_func("ingenic-vo/regs/t40", test_regs_t40);
    qtest_add_func("ingenic-vo/regs/a1", test_regs_a1);
    qtest_add_data_func("ingenic-vo/queue/t41", "t41", test_queue);
    qtest_add_data_func("ingenic-vo/queue/t40", "t40", test_queue);
    qtest_add_func("ingenic-vo/console", test_console);
    return g_test_run();
}
