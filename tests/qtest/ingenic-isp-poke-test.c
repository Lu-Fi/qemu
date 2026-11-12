/*
 * QTest testcase for a T31 frame whose passes run without the BQL
 *
 * The ISP's registers are written as a guest's would be while a pass runs
 * (the model's x-poke does it, with the BQL released, in each pass): the
 * writes wait, so that the frame comes out as it does with no writes
 * meanwhile, byte for byte, and they take effect as it ends. Frames of
 * several bands, with a second channel scaled down, as the noise reducer's
 * pins run them.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define ISP             0x13300000
#define MDNS            (ISP + 0x7800)
#define CPM_CLKGR0      0x10000020
#define FRAME_NS        (1000000000 / 30)
#define REF_Y           0x02000000
#define CH_BUF          0x03000000
#define CH1_BUF         0x04000000
#define FRAMES          4

typedef struct Scene {
    unsigned w, h;
    unsigned scaled_w, scaled_h;
    const char *sensor;
} Scene;

/* What the poke writes, and what the test puts back before each frame */
typedef struct Poke {
    uint32_t reg;
    uint32_t val;
    bool back;                  /* reads back the value after the frame */
                                /* (no buffer pushes: they queue) */
} Poke;

static const Poke pokes[] = {
    { MDNS + 0x19c, 0x00ff00ff, true },
    { MDNS + 0x1b8, 0xffffffff, true },
    { MDNS + 0x1f0, 0x00000000, true },
    { ISP + 0x7048, 0x3f3f3f3f, true },
    { ISP + 0x7058, 0x00000000, true },
    { ISP + 0x1804, 0x00000300, true },
    { ISP + 0x1808, 0x00000100, true },
    { ISP + 0x9900, 0x00100010, true },
    { ISP + 0x9998, 0x00000040, true },
};

static void run(const Scene *sc, bool poked, GString *out)
{
    unsigned w = sc->w, h = sc->h, bw = DIV_ROUND_UP(w, 16);
    unsigned bh = DIV_ROUND_UP(h, 16);
    unsigned stride = (((w + 31) >> 5) + 7) & ~7;
    uint32_t sta = ROUND_UP(REF_Y + w * h * 3 / 2, 0x100);
    size_t sta_len = 4 * ROUND_UP(stride * (bh + 1), 0x100);
    size_t n = w * h * 3 / 2;
    g_autofree uint8_t *buf = g_malloc(MAX(n, sta_len));
    g_autoptr(GString) args = g_string_new(NULL);
    uint32_t orig[ARRAY_SIZE(pokes)];
    QTestState *qts;

    g_string_append_printf(args, "-machine ingenic-t31 -device "
                           "image-sensor,model=jxq03,bus=i2c0,id=cam%s",
                           sc->sensor);
    if (poked) {
        g_string_append(args, " -global ingenic-isp.x-poke=");
        for (unsigned i = 0; i < ARRAY_SIZE(pokes); i++) {
            g_string_append_printf(args, "%s0x%x=0x%x", i ? "," : "",
                                   pokes[i].reg - ISP, pokes[i].val);
        }
    }
    qts = qtest_init(args->str);
    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, ISP + 0x4, w << 16 | h);
    qtest_writel(qts, MDNS + 0x20, REF_Y);
    qtest_writel(qts, MDNS + 0x24, w);
    qtest_writel(qts, MDNS + 0x28, REF_Y + w * h);
    qtest_writel(qts, MDNS + 0x2c, w);
    for (int k = 0; k < 4; k++) {
        static const uint32_t at[4] = { 0x30, 0x40, 0x48, 0x50 };

        qtest_writel(qts, MDNS + at[k],
                     sta + k * ROUND_UP(stride * (bh + 1), 0x100));
        qtest_writel(qts, MDNS + at[k] + 4, stride);
        qtest_writel(qts, MDNS + 0x74 + k * 0x18,
                     bw << 24 | (bh - (k & 1)) << 16 | 0x1010);
        qtest_writel(qts, MDNS + 0x78 + k * 0x18, 0x14f0 | !(k % 3));
        qtest_writel(qts, MDNS + 0x7c + k * 0x18, 0xffc60c64);
    }
    qtest_writel(qts, MDNS + 0x144, 0xffc63633);
    qtest_writel(qts, MDNS + 0x148, 0xc8b4a000);
    qtest_writel(qts, MDNS + 0x14c, 0xfafaf0dc);
    qtest_writel(qts, MDNS + 0x150, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x154, 0xfafafafa);
    qtest_writel(qts, MDNS + 0x264, 0xff173141);
    qtest_writel(qts, MDNS + 0x26c, 0x07d10aff);
    qtest_writel(qts, MDNS + 0x10, 0x11111111);
    qtest_writel(qts, MDNS + 0x14, 0x00f01100);
    qtest_writel(qts, MDNS + 0x04, 0x111);
    qtest_writel(qts, ISP + 0x9804, sc->scaled_w ? 0xf0003 : 1);
    qtest_writel(qts, MDNS + 0x19c, 0x00c000ff);
    qtest_writel(qts, MDNS + 0x1b8, 0x01010101);
    qtest_writel(qts, MDNS + 0x1f0, 0x01010101);
    qtest_writel(qts, ISP + 0x7048, 0x01010101);
    qtest_writel(qts, ISP + 0x7058, 0x01010101);
    /* the VIC's start runs the first frame, the writes in its passes too */
    for (unsigned i = 0; i < ARRAY_SIZE(pokes); i++) {
        orig[i] = qtest_readl(qts, pokes[i].reg);
    }
    qtest_writel(qts, ISP + 0xe0000, 1);            /* VIC start */
    qtest_clock_step(qts, FRAME_NS / 2);
    for (unsigned f = 0; f < FRAMES; f++) {
        for (unsigned i = 0; i < ARRAY_SIZE(pokes); i++) {
            qtest_writel(qts, pokes[i].reg, orig[i]);
        }
        qtest_writel(qts, ISP + 0x9900, w << 16 | h);
        qtest_writel(qts, ISP + 0x9980, w);
        qtest_writel(qts, ISP + 0x9998, w);
        qtest_writel(qts, ISP + 0x996c, CH_BUF);
        qtest_writel(qts, ISP + 0x9984, CH_BUF + w * h);
        if (sc->scaled_w) {
            qtest_writel(qts, ISP + 0x9a00, sc->scaled_w << 16 | sc->scaled_h);
            qtest_writel(qts, ISP + 0x9a80, sc->scaled_w);
            qtest_writel(qts, ISP + 0x9a98, sc->scaled_w);
            qtest_writel(qts, ISP + 0x9a6c, CH1_BUF);
            qtest_writel(qts, ISP + 0x9a84,
                         CH1_BUF + sc->scaled_w * sc->scaled_h);
        }
        qtest_clock_step(qts, FRAME_NS);
        for (unsigned i = 0; poked && i < ARRAY_SIZE(pokes); i++) {
            /* the writes made during the passes took effect at its end */
            if (pokes[i].back) {
                g_assert_cmphex(qtest_readl(qts, pokes[i].reg), ==,
                                pokes[i].val);
            }
        }
        qtest_readl(qts, ISP + 0x9974);             /* the done entry */
        if (sc->scaled_w) {
            qtest_readl(qts, ISP + 0x9a74);
        }
        qtest_memread(qts, CH_BUF, buf, n);
        g_string_append_printf(out, "%.16s", g_compute_checksum_for_data(
                               G_CHECKSUM_SHA256, buf, n));
        qtest_memread(qts, REF_Y, buf, n);
        g_string_append_printf(out, " %.16s", g_compute_checksum_for_data(
                               G_CHECKSUM_SHA256, buf, n));
        qtest_memread(qts, sta, buf, sta_len);
        g_string_append_printf(out, " %.16s", g_compute_checksum_for_data(
                               G_CHECKSUM_SHA256, buf, sta_len));
        if (sc->scaled_w) {
            size_t m = sc->scaled_w * sc->scaled_h * 3 / 2;

            qtest_memread(qts, CH1_BUF, buf, m);
            g_string_append_printf(out, " %.16s", g_compute_checksum_for_data(
                                   G_CHECKSUM_SHA256, buf, m));
        }
        g_string_append_c(out, '\n');
    }
    if (poked) {
        QDict *r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                             "{'path': '/machine/soc/isp', "
                             "'property': 'x-poke-passes'}}");
        int64_t passes = qdict_get_int(r, "return");

        qobject_unref(r);
        r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                      "{'path': '/machine/soc/isp', "
                      "'property': 'x-poke-misses'}}");
        g_assert_cmpint(passes, >, 0);
        g_assert_cmpint(qdict_get_int(r, "return"), ==, 0);
        qobject_unref(r);
    }
    qtest_quit(qts);
}

static void test_poke(const void *data)
{
    const Scene *sc = data;
    g_autoptr(GString) quiet = g_string_new(NULL);
    g_autoptr(GString) poked = g_string_new(NULL);

    run(sc, false, quiet);
    run(sc, true, poked);
    g_assert_cmpstr(poked->str, ==, quiet->str);
}

static const Scene scenes[] = {
    /* several bands (2^17 pixels and over), odd sizes, a channel scaled down */
    { 421, 313, 0, 0, ",motion=off,noise=on" },
    { 453, 301, 322, 190, ",object=on,noise=on" },
    /* a full frame, as a 1080p stream runs */
    { 1920, 1080, 1280, 720, ",motion=off,noise=on" },
};

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (unsigned i = 0; i < ARRAY_SIZE(scenes); i++) {
        g_autofree char *name = g_strdup_printf("/ingenic-isp/poke/%ux%u",
                                                scenes[i].w, scenes[i].h);

        qtest_add_data_func(name, &scenes[i], test_poke);
    }
    return g_test_run();
}
