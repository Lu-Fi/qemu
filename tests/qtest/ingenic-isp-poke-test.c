/*
 * QTest testcase for a frame whose passes run without the BQL, on each ISP
 * layout that does so
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

/*
 * An ISP: its channels at ch + n * 0x100 with the T31's or the T32's
 * register offsets, its VIC, and the gain words
 */
typedef struct Lay {
    const char *name, *machine, *variant, *model;
    uint32_t ch_en, ch, vic, vic_start, vic_size;
    uint32_t size, y_push, uv_push, y_stride, uv_stride, done_y;
    uint32_t wb_r, wb_b;
    uint32_t bypass, latch;             /* T41: lens correction off; */
                                        /* its channels' latch */
    bool mdns;                          /* the T31's noise reducer registers */
    int64_t first_ns;                   /* the first frame, if later than */
                                        /* half a period */
} Lay;

typedef struct Scene {
    unsigned w, h;
    unsigned scaled_w, scaled_h;
    const char *sensor;
    const Lay *lay;
} Scene;

static const Lay lay_t31 = {
    .name = "t31", .machine = "ingenic-t31", .model = "jxq03",
    .ch_en = 0x9804, .ch = 0x9900, .vic = 0xe0000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .wb_r = 0x1804, .wb_b = 0x1808,
    .mdns = true,
};

static const Lay lay_t23 = {
    .name = "t23", .machine = "ingenic-t23", .model = "jxq03",
    .ch_en = 0xd040, .ch = 0xd100, .vic = 0xe0000,
    .size = 0x04, .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40,
    .uv_stride = 0x60, .done_y = 0x38, .wb_r = 0x1804, .wb_b = 0x1808,
    .mdns = true,
};

static const Lay lay_t21 = {
    .name = "t21", .machine = "ingenic-t21", .model = "jxq03",
    .ch_en = 0x2304, .ch = 0x2400, .vic = 0x10000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .wb_r = 0x604, .wb_b = 0x608,
};

#define T32_LAY \
    .machine = "ingenic-t32", .model = "jxq03", .ch_en = 0xf0040, \
    .ch = 0xf0100, .vic = 0x80000, .vic_start = 0x10, .size = 0x04, \
    .y_push = 0x30, .uv_push = 0x50, .y_stride = 0x40, .uv_stride = 0x60, \
    .done_y = 0x38, .wb_r = 0x6040, .wb_b = 0x6044

static const Lay lay_t32 = { .name = "t32", .variant = "t32nq", T32_LAY };
static const Lay lay_t33 = { .name = "t33", .variant = "t33n", T32_LAY };
static const Lay lay_t32pro = { .name = "t32pro", .variant = "t32prozn",
                                T32_LAY };

static const Lay lay_t40 = {
    .name = "t40", .machine = "ingenic-t40", .model = "jxq03",
    .ch_en = 0x16008, .ch = 0x16100, .vic = 0x80000,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .wb_r = 0x4004, .wb_b = 0x4008,
};

static const Lay lay_t41 = {
    .name = "t41", .machine = "ingenic-t41", .model = "gc4023",
    .ch_en = 0xf0008, .ch = 0xf0100, .vic = 0x80000, .vic_size = 0x80,
    .size = 0x00, .y_push = 0x6c, .uv_push = 0x84, .y_stride = 0x80,
    .uv_stride = 0x98, .done_y = 0x74, .wb_r = 0x4004, .wb_b = 0x4008,
    .bypass = 0x40, .latch = 0xf0010, .first_ns = 135000000,
};

#define CHR(sc, n, r)   (ISP + (sc)->lay->ch + (n) * 0x100 + (sc)->lay->r)

/* What the poke writes, and what the test puts back before each frame */
typedef struct Poke {
    uint32_t reg;
    uint32_t val;
    bool back;                  /* reads back the value after the frame */
                                /* (no buffer pushes: they queue) */
} Poke;

#define MAX_POKES 9

/* What the writes are, for a layout */
static unsigned pokes_of(const Scene *sc, Poke *p)
{
    unsigned n = 0;

    if (sc->lay->mdns) {
        p[n++] = (Poke){ MDNS + 0x19c, 0x00ff00ff, true };
        p[n++] = (Poke){ MDNS + 0x1b8, 0xffffffff, true };
        p[n++] = (Poke){ MDNS + 0x1f0, 0x00000000, true };
        p[n++] = (Poke){ ISP + 0x7048, 0x3f3f3f3f, true };
        p[n++] = (Poke){ ISP + 0x7058, 0x00000000, true };
    }
    p[n++] = (Poke){ ISP + sc->lay->wb_r, 0x00000300, true };
    p[n++] = (Poke){ ISP + sc->lay->wb_b, 0x00000100, true };
    /* a T41's channel words wait for its latch: nothing to read back */
    p[n++] = (Poke){ CHR(sc, 0, size), 0x00100010, !sc->lay->latch };
    p[n++] = (Poke){ CHR(sc, 0, uv_stride), 0x00000040, !sc->lay->latch };
    return n;
}

static QTestState *start(const Scene *sc, const char *extra)
{
    const Lay *l = sc->lay;
    g_autofree char *soc = l->variant ?
        g_strdup_printf("-global ingenic-t31.soc-variant=%s", l->variant) :
        g_strdup("");
    g_autofree char *args = g_strdup_printf("-machine %s %s -device "
                                            "image-sensor,model=%s,"
                                            "bus=i2c0,id=cam%s %s",
                                            l->machine, soc, l->model,
                                            sc->sensor, extra);
    g_autofree char *more = g_strdup_printf("%s %s", args,
                                            g_getenv("ISP_POKE_EXTRA") ?: "");

    return qtest_init(more);
}

static uint32_t sta_at(const Scene *sc)
{
    return ROUND_UP(REF_Y + sc->w * sc->h * 3 / 2, 0x100);
}

/* The settings libimp leaves a T31 streaming with, but for the VIC start */
static void setup(QTestState *qts, const Scene *sc)
{
    unsigned w = sc->w, h = sc->h, bw = DIV_ROUND_UP(w, 16);
    unsigned bh = DIV_ROUND_UP(h, 16);
    unsigned stride = (((w + 31) >> 5) + 7) & ~7;
    uint32_t sta = sta_at(sc);

    qtest_writel(qts, CPM_CLKGR0, 0);
    if (sc->lay->bypass) {
        /* a word that waits for the latch (0x1000) */
        qtest_writel(qts, ISP + sc->lay->bypass, 1u << 20);
        qtest_writel(qts, ISP + 0x1000, 1);
    }
    qtest_writel(qts, ISP + sc->lay->ch_en, sc->scaled_w ? 3 : 1);
    if (sc->lay->vic_size) {
        qtest_writel(qts, ISP + sc->lay->vic_size, w << 16 | h);
    }
    if (!sc->lay->mdns) {
        return;
    }
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
    qtest_writel(qts, ISP + sc->lay->ch_en,
                 (sc->scaled_w ? 3 : 1) | (sc->lay == &lay_t31 ? 0xf0000 : 0));
    qtest_writel(qts, MDNS + 0x19c, 0x00c000ff);
    qtest_writel(qts, MDNS + 0x1b8, 0x01010101);
    qtest_writel(qts, MDNS + 0x1f0, 0x01010101);
    qtest_writel(qts, ISP + 0x7048, 0x01010101);
    qtest_writel(qts, ISP + 0x7058, 0x01010101);
}

/* The channels' buffers, as the driver gives them before each frame */
static void push(QTestState *qts, const Scene *sc)
{
    const Lay *l = sc->lay;
    unsigned w = sc->w, h = sc->h;

    qtest_writel(qts, CHR(sc, 0, size), w << 16 | h);
    qtest_writel(qts, CHR(sc, 0, y_stride), w);
    qtest_writel(qts, CHR(sc, 0, uv_stride), w);
    qtest_writel(qts, CHR(sc, 0, y_push), CH_BUF);
    qtest_writel(qts, CHR(sc, 0, uv_push), CH_BUF + w * h);
    if (sc->scaled_w) {
        qtest_writel(qts, CHR(sc, 1, size), sc->scaled_w << 16 | sc->scaled_h);
        qtest_writel(qts, CHR(sc, 1, y_stride), sc->scaled_w);
        qtest_writel(qts, CHR(sc, 1, uv_stride), sc->scaled_w);
        qtest_writel(qts, CHR(sc, 1, y_push), CH1_BUF);
        qtest_writel(qts, CHR(sc, 1, uv_push),
                     CH1_BUF + sc->scaled_w * sc->scaled_h);
    }
    if (l->latch) {
        qtest_writel(qts, ISP + l->latch, 1);
    }
}

static void digest_of(GString *out, const uint8_t *buf, size_t n, bool first)
{
    g_autofree char *d = g_compute_checksum_for_data(G_CHECKSUM_SHA256, buf, n);

    g_string_append_printf(out, "%s%.16s", first ? "" : " ", d);
}

/* The frame the channels, the reference, the ages and channel 1 hold */
static void digest(QTestState *qts, const Scene *sc, GString *out)
{
    unsigned stride = (((sc->w + 31) >> 5) + 7) & ~7;
    size_t n = sc->w * sc->h * 3 / 2;
    unsigned bh = DIV_ROUND_UP(sc->h, 16);
    size_t sta_len = 4 * ROUND_UP(stride * (bh + 1), 0x100);
    g_autofree uint8_t *buf = g_malloc(MAX(n, sta_len));

    qtest_readl(qts, CHR(sc, 0, done_y));           /* the done entry */
    if (sc->scaled_w) {
        qtest_readl(qts, CHR(sc, 1, done_y));
    }
    qtest_memread(qts, CH_BUF, buf, n);
    digest_of(out, buf, n, true);
    qtest_memread(qts, REF_Y, buf, n);
    digest_of(out, buf, n, false);
    qtest_memread(qts, sta_at(sc), buf, sta_len);
    digest_of(out, buf, sta_len, false);
    if (sc->scaled_w) {
        size_t m = sc->scaled_w * sc->scaled_h * 3 / 2;

        qtest_memread(qts, CH1_BUF, buf, m);
        digest_of(out, buf, m, false);
    }
    g_string_append_c(out, '\n');
}

static void vic_start(QTestState *qts, const Scene *sc)
{
    qtest_writel(qts, ISP + sc->lay->vic + sc->lay->vic_start, 1);
}

/* Up to the middle of the first frame's period, or where it comes later */
static int64_t first_frame(QTestState *qts, const Scene *sc)
{
    return qtest_clock_step(qts, sc->lay->first_ns ?: FRAME_NS / 2);
}

static int64_t prop(QTestState *qts, const char *name)
{
    QDict *r = qtest_qmp(qts, "{'execute': 'qom-get', 'arguments': "
                         "{'path': '/machine/soc/isp', 'property': %s}}",
                         name);
    int64_t v = qdict_get_int(r, "return");

    qobject_unref(r);
    return v;
}

/* Frames run from a bottom half are done when the device says it is idle */
static void settle(QTestState *qts)
{
    for (int i = 0; i < 20000; i++) {
        if (!prop(qts, "x-due") && !prop(qts, "x-txn")) {
            return;
        }
        g_usleep(1000);
    }
    g_assert_not_reached();
}

static void run(const Scene *sc, const char *extra, bool poked, GString *out)
{
    g_autoptr(GString) args = g_string_new(extra);
    Poke pokes[MAX_POKES];
    unsigned npokes = pokes_of(sc, pokes);
    uint32_t orig[MAX_POKES];
    QTestState *qts;

    if (poked) {
        g_string_append(args, " -global ingenic-isp.x-poke=");
        for (unsigned i = 0; i < npokes; i++) {
            g_string_append_printf(args, "%s0x%x=0x%x", i ? "," : "",
                                   pokes[i].reg - ISP, pokes[i].val);
        }
    }
    qts = start(sc, args->str);
    setup(qts, sc);
    /* the VIC's start runs the first frame, the writes in its passes too */
    for (unsigned i = 0; i < npokes; i++) {
        orig[i] = qtest_readl(qts, pokes[i].reg);
    }
    vic_start(qts, sc);
    first_frame(qts, sc);
    for (unsigned f = 0; f < FRAMES; f++) {
        for (unsigned i = 0; i < npokes; i++) {
            qtest_writel(qts, pokes[i].reg, orig[i]);
        }
        push(qts, sc);
        qtest_clock_step(qts, FRAME_NS);
        for (unsigned i = 0; poked && i < npokes; i++) {
            /* the writes made during the passes took effect at its end */
            if (pokes[i].back) {
                g_assert_cmphex(qtest_readl(qts, pokes[i].reg), ==,
                                pokes[i].val);
            }
        }
        digest(qts, sc, out);
    }
    if (strstr(extra, "x-act=")) {
        /* the other thread waited for the frame, and found it whole */
        for (int i = 0; i < 100 && prop(qts, "x-act-done") < 1; i++) {
            g_usleep(50000);
        }
        g_assert_cmpint(prop(qts, "x-act-done"), ==, 1);
        g_assert_cmpint(prop(qts, "x-act-bad"), ==, 0);
        g_assert_cmpint(prop(qts, "x-quiesce-waits"), >, 0);
    }
    if (poked) {
        g_assert_cmpint(prop(qts, "x-poke-passes"), >, 0);
        g_assert_cmpint(prop(qts, "x-poke-misses"), ==, 0);
    }
    qtest_quit(qts);
}

static void test_poke(const void *data)
{
    const Scene *sc = data;
    g_autoptr(GString) quiet = g_string_new(NULL);
    g_autoptr(GString) poked = g_string_new(NULL);

    run(sc, "", false, quiet);
    run(sc, "", true, poked);
    g_assert_cmpstr(poked->str, ==, quiet->str);
}

/* A frame comes out the same in however many bands its passes are worked */
static void test_bands(const void *data)
{
    const Scene *sc = data;
    g_autoptr(GString) base = g_string_new(NULL);

    run(sc, "-global ingenic-isp.x-bands=1", false, base);
    for (int n = 2; n <= 4; n++) {
        g_autoptr(GString) out = g_string_new(NULL);
        g_autofree char *extra = g_strdup_printf("-global "
                                                 "ingenic-isp.x-bands=%d", n);

        run(sc, extra, false, out);
        g_assert_cmpstr(out->str, ==, base->str);
    }
}

/*
 * Another thread saves the device as the first frame's first pass runs (a
 * migration thread would): it waits for the frame to end, finds nothing
 * half done, and the frames that follow are as with no save
 */
static void test_save(const void *data)
{
    const Scene *sc = data;
    g_autoptr(GString) quiet = g_string_new(NULL);
    g_autoptr(GString) saved = g_string_new(NULL);

    run(sc, "", false, quiet);
    run(sc, "-global ingenic-isp.x-act=save "
        "-global ingenic-isp.x-pass-delay-ms=20", false, saved);
    g_assert_cmpstr(saved->str, ==, quiet->str);
}

/* The same with a reset: the frame ends first, then the device is reset */
static void test_reset(const void *data)
{
    const Scene *sc = data;
    g_autoptr(GString) quiet = g_string_new(NULL);
    g_autoptr(GString) reset = g_string_new(NULL);
    QTestState *qts = start(sc, "-global ingenic-isp.x-act=reset "
                            "-global ingenic-isp.x-pass-delay-ms=20");
    QTestState *ref = start(sc, "");

    setup(ref, sc);
    push(ref, sc);
    vic_start(ref, sc);
    first_frame(ref, sc);
    digest(ref, sc, quiet);
    setup(qts, sc);
    push(qts, sc);
    vic_start(qts, sc);                             /* the first frame */
    first_frame(qts, sc);
    digest(qts, sc, reset);
    /* the reference the frame wrote is whole, whenever the reset came */
    g_assert_cmpstr(reset->str, ==, quiet->str);
    for (int i = 0; i < 100 && prop(qts, "x-act-done") < 1; i++) {
        g_usleep(50000);
        qtest_clock_step(qts, 1);
    }
    g_assert_cmpint(prop(qts, "x-act-done"), ==, 1);
    g_assert_cmpint(prop(qts, "x-act-bad"), ==, 0);
    g_assert_cmpint(prop(qts, "x-quiesce-waits"), >, 0);
    g_assert_cmphex(qtest_readl(qts, ISP + sc->lay->ch_en), ==, 0);
    qtest_quit(ref);
    qtest_quit(qts);
}

static void migrate_ref(const Scene *sc, unsigned frames, GString *ref)
{
    QTestState *src = start(sc, "");

    setup(src, sc);
    if (sc->lay->mdns) {
        qtest_writel(src, MDNS + 0x20, 0);
    }
    vic_start(src, sc);
    first_frame(src, sc);
    for (unsigned f = 0; f < frames; f++) {
        push(src, sc);
        qtest_clock_step(src, FRAME_NS);
        digest(src, sc, ref);
    }
    qtest_quit(src);
}

/*
 * A migration while frames run, passes long enough that the end of it is
 * likely to come in one: the machine loaded in a fresh QEMU goes on with
 * the frames an uninterrupted run has. The noise reducer is off: the frames
 * it kept to work from are not part of the saved state.
 */
static void test_migrate(const void *data)
{
    const Scene *sc = data;
    g_autoptr(GString) ref = g_string_new(NULL);
    g_autoptr(GString) after = g_string_new(NULL);
    g_autofree char *sock = g_strdup_printf("/tmp/isp-poke-%d.sock", getpid());
    g_autofree char *uri = g_strdup_printf("unix:%s", sock);
    g_autofree char *dargs = NULL;
    QTestState *src, *dst;
    int64_t now = 0;
    QDict *r;

    src = start(sc, "-global ingenic-isp.x-pass-delay-ms=15 "
                "-global ingenic-isp.x-frame-bh=on");
    dargs = g_strdup_printf("-global ingenic-isp.x-frame-bh=on -incoming %s",
                            uri);
    dst = start(sc, dargs);
    setup(src, sc);
    if (sc->lay->mdns) {
        qtest_writel(src, MDNS + 0x20, 0);
    }
    vic_start(src, sc);
    now += first_frame(src, sc) - now;
    for (unsigned f = 0; f < 2; f++) {
        push(src, sc);
        now = qtest_clock_step(src, FRAME_NS);
        settle(src);
    }
    qtest_qmp_assert_success(src, "{'execute': 'migrate', 'arguments': "
                             "{'uri': %s}}", uri);
    for (int i = 0; i < 4000; i++) {
        const char *st;

        r = qtest_qmp(src, "{'execute': 'query-migrate'}");
        st = qdict_get_str(qdict_get_qdict(r, "return"), "status");
        if (!strcmp(st, "completed")) {
            qobject_unref(r);
            break;
        }
        g_assert_cmpstr(st, !=, "failed");
        qobject_unref(r);
        now = qtest_clock_step(src, FRAME_NS / 8);
        g_usleep(1000);
    }
    g_test_message("quiesce waits: %" PRId64, prop(src, "x-quiesce-waits"));
    qtest_quit(src);
    /* the timers came across at the source's time: catch up to it */
    qtest_clock_step(dst, now);
    settle(dst);
    for (unsigned f = 0; f < 4; f++) {
        push(dst, sc);
        qtest_clock_step(dst, FRAME_NS);
        settle(dst);
        digest(dst, sc, after);
    }
    qtest_quit(dst);
    /*
     * The frames of a run that is not migrated, as many as the other got
     * to: how long a migration takes is the host's, not the model's
     */
    migrate_ref(sc, now / FRAME_NS + FRAMES + 8, ref);
    /*
     * What it put out is frames the reference run has, each channel's and
     * the reference's: their noise differs from frame to frame, so a frame
     * of another number, or one half of two, would not be found. The clock
     * of the new QEMU starts at 0, so that frames may be skipped.
     */
    {
        g_auto(GStrv) rl = g_strsplit(ref->str, "\n", -1);
        g_auto(GStrv) al = g_strsplit(after->str, "\n", -1);
        unsigned found = 0;

        for (unsigned i = 0; al[i] && *al[i]; i++) {
            g_auto(GStrv) ac = g_strsplit(al[i], " ", -1);

            for (unsigned c = 0; ac[c]; c++) {
                bool seen = false;

                for (unsigned j = 0; rl[j] && *rl[j]; j++) {
                    g_auto(GStrv) rc = g_strsplit(rl[j], " ", -1);

                    seen |= rc[c] && !strcmp(rc[c], ac[c]);
                }
                if (!seen) {
                    g_test_message("frame %u column %u is not in the reference",
                                   i, c);
                    g_test_message("%s", ref->str);
                    g_test_message("%s", after->str);
                    g_test_fail();
                }
            }
            found++;
        }
        g_assert_cmpuint(found, ==, 4);
    }
}

static const Scene scenes[] = {
    /* several bands (2^17 pixels and over), odd sizes, a channel scaled down */
    { 421, 313, 0, 0, ",motion=off,noise=on", &lay_t31 },
    { 453, 301, 322, 190, ",object=on,noise=on", &lay_t31 },
    /* a full frame, as a 1080p stream runs */
    { 1920, 1080, 1280, 720, ",motion=off,noise=on", &lay_t31 },
};

/* The T23's shares the noise reducer's registers */
static const Scene scenes_t23[] = {
    { 453, 301, 322, 190, ",object=on,noise=on", &lay_t23 },
    { 1920, 1080, 1280, 720, ",motion=off,noise=on", &lay_t23 },
};

/* The others: a main channel and one scaled down, the statistics and DMA */
#define OTHER(l) { 1280, 720, 640, 360, ",motion=off,noise=on", &l }

static const Scene scenes_other[] = {
    OTHER(lay_t21), OTHER(lay_t32), OTHER(lay_t33), OTHER(lay_t32pro),
    OTHER(lay_t40), OTHER(lay_t41),
};

static void add_layout(const Scene *sc, const char *tag)
{
    g_autofree char *name = g_strdup_printf("/ingenic-isp/%s/poke/%ux%u",
                                            sc->lay->name, sc->w, sc->h);

    qtest_add_data_func(name, sc, test_poke);
    if (tag) {
        g_autofree char *b = g_strdup_printf("/ingenic-isp/%s/bands",
                                             sc->lay->name);
        g_autofree char *s = g_strdup_printf("/ingenic-isp/%s/save",
                                             sc->lay->name);
        g_autofree char *r = g_strdup_printf("/ingenic-isp/%s/reset",
                                             sc->lay->name);
        g_autofree char *m = g_strdup_printf("/ingenic-isp/%s/migrate",
                                             sc->lay->name);

        qtest_add_data_func(b, sc, test_bands);
        qtest_add_data_func(s, sc, test_save);
        qtest_add_data_func(r, sc, test_reset);
        qtest_add_data_func(m, sc, test_migrate);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (unsigned i = 0; i < ARRAY_SIZE(scenes); i++) {
        add_layout(&scenes[i], i == 1 ? "t31" : NULL);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(scenes_t23); i++) {
        add_layout(&scenes_t23[i], i ? NULL : "t23");
    }
    for (unsigned i = 0; i < ARRAY_SIZE(scenes_other); i++) {
        add_layout(&scenes_other[i], "other");
    }
    return g_test_run();
}
