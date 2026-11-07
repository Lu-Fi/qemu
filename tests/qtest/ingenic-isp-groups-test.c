/*
 * QTest testcase for the ISP's per-frame module-done and DMA-done words
 * (enable, clear, status at +0x0, +0x4, +0x8), as a T32LQ, T40N and
 * T41NQ raise them each frame: 0x300a8 on the T32 family, 0x400a8 (and
 * the T41's DMA word at 0x400b8) on the T40 and T41. A bit latches only
 * while enabled, and a write of ones to the clear word clears it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define MS              1000000
#define FRAME_NS        (40 * MS)       /* fps=25 */
#define T41_FIRST_NS    (135 * MS)      /* a T41NQ's first frame */

typedef struct GroupPart {
    const char *name;
    const char *args;
    uint32_t vic_ctrl;
    uint32_t mod;                       /* its module-done word's enable */
    uint32_t mod_bits;                  /* what each frame raises there */
    uint32_t dma, dma_bits;             /* its DMA-done word, if any */
} GroupPart;

static const GroupPart parts[] = {
    { "t32", "-machine ingenic-t32 -global ingenic-t31.soc-variant=t32nq",
      0x80010, 0x300a0, 0xfffff9b7 },
    { "t33", "-machine ingenic-t32 -global ingenic-t31.soc-variant=t33n",
      0x80010, 0x300a0, 0xfffffdf7 },
    { "t32pro",
      "-machine ingenic-t32 -global ingenic-t31.soc-variant=t32prozn",
      0x80010, 0x300a0, 0xfffffdf7 },
    { "t40", "-machine ingenic-t40", 0x80000, 0x400a0,
      1u << 25 | 1u << 27 | 1u << 29 },
    { "t41", "-machine ingenic-t41", 0x80000, 0x400a0, 0x7effffff,
      0x400b0, 0xd },
};

static QTestState *start(const GroupPart *p)
{
    QTestState *qts = qtest_initf("%s -device image-sensor,model=jxq03,"
                                  "bus=i2c0,fps=25", p->args);

    qtest_writel(qts, CPM_CLKGR0, 0);
    if (p->dma) {
        qtest_writel(qts, CPM_CIMCDR,
                     qtest_readl(qts, CPM_CIMCDR) & ~(1u << 27));
        qtest_writel(qts, ISP + 0x80080, 640 << 16 | 360);
    }
    return qts;
}

/* The time to the second frame after the VIC starts */
static int64_t two_frames(const GroupPart *p)
{
    return !strcmp(p->name, "t41") ? T41_FIRST_NS + FRAME_NS : FRAME_NS * 2;
}

/* Clear a word's status, run a frame with @en enabled, read it back */
static uint32_t frame_with(QTestState *qts, uint32_t word, uint32_t en)
{
    qtest_writel(qts, ISP + word + 4, ~0u);
    g_assert_cmphex(qtest_readl(qts, ISP + word + 8), ==, 0);
    qtest_writel(qts, ISP + word, en);
    qtest_clock_step(qts, FRAME_NS);
    return qtest_readl(qts, ISP + word + 8);
}

static void test_groups(const void *data)
{
    const GroupPart *p = data;
    QTestState *qts = start(p);

    qtest_writel(qts, ISP + p->vic_ctrl, 1);
    qtest_clock_step(qts, two_frames(p));

    g_assert_cmphex(frame_with(qts, p->mod, ~0u), ==, p->mod_bits);
    g_assert_cmphex(frame_with(qts, p->mod, 0), ==, 0);
    /* enabling a bit raises nothing until the next frame */
    qtest_writel(qts, ISP + p->mod, ~0u);
    g_assert_cmphex(qtest_readl(qts, ISP + p->mod + 8), ==, 0);
    g_assert_cmphex(frame_with(qts, p->mod, 1u << 29 | 1u << 1), ==,
                    p->mod_bits & (1u << 29 | 1u << 1));
    /* a clear takes only the bits written */
    qtest_writel(qts, ISP + p->mod, ~0u);
    qtest_clock_step(qts, FRAME_NS);
    qtest_writel(qts, ISP + p->mod + 4, 1u << 29);
    g_assert_cmphex(qtest_readl(qts, ISP + p->mod + 8), ==,
                    p->mod_bits & ~(1u << 29));

    if (p->dma) {
        g_assert_cmphex(frame_with(qts, p->dma, 0x3f), ==, p->dma_bits);
        g_assert_cmphex(frame_with(qts, p->dma, 0x30), ==, 0);
    }
    qtest_quit(qts);
}

/*
 * The main interrupt word's bit 13: a T40N's with channel 0's frame, on
 * each pipeline, and a T41NQ's never (its frame start is bit 20)
 */
static void test_core_a(const void *data)
{
    const GroupPart *p = data;
    bool t40 = !strcmp(p->name, "t40");
    uint32_t ch_en = t40 ? 0x16008 : 0xf0008;
    QTestState *qts = start(p);

    qtest_writel(qts, ISP + p->vic_ctrl, 1);
    qtest_writel(qts, ISP + ch_en, 1);
    qtest_clock_step(qts, two_frames(p));

    qtest_writel(qts, ISP + 0x40024, ~0u);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, ISP + 0x40028) & (1u << 13 | 1u << 20),
                    ==, t40 ? 1u << 13 : 1u << 20);

    qtest_writel(qts, ISP + ch_en, 0);
    qtest_clock_step(qts, FRAME_NS);
    qtest_writel(qts, ISP + 0x40024, ~0u);
    qtest_clock_step(qts, FRAME_NS);
    g_assert_cmphex(qtest_readl(qts, ISP + 0x40028) & (1u << 13 | 1u << 20),
                    ==, t40 ? 0 : 1u << 20);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (unsigned i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-isp/groups/%s",
                                                parts[i].name);

        qtest_add_data_func(path, &parts[i], test_groups);
        if (!strcmp(parts[i].name, "t40") || !strcmp(parts[i].name, "t41")) {
            g_autofree char *a = g_strdup_printf("/ingenic-isp/core-a/%s",
                                                 parts[i].name);

            qtest_add_data_func(a, &parts[i], test_core_a);
        }
    }
    return g_test_run();
}
