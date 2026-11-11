/*
 * QTest testcase for the Ingenic encoders' stream tap on the AVPU and HERA
 *
 * The T31's EL150 codes small pictures for three libimp channels with
 * "x-stream-tap" set. The test plays libimp: it puts the parameter sets and
 * the slice header in front of the slice data in the stream buffer, as
 * libimp's H.264 does before the job, stale bytes of an older picture and
 * all, and checks what each stream's file holds: the channels apart, each
 * picture libimp's bytes then the core's, the parameter sets only with an
 * IDR picture, and nothing from a stream whose first picture has no
 * parameter sets. An H.265 picture's headers come after the job, the stale
 * ones first: the tap waits for them.
 *
 * The T32's HERA codes a picture whose slice data libimp escapes and copies
 * into a stream of its own, after the header; the T33's asks the core for
 * the 03 bytes and libimp copies what the core wrote. The test plays libimp
 * for both, with a picture that needs an 03 byte, and checks the file holds
 * the copy, escaped once.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "libqtest.h"

#define AVPU            0x13200000
#define CL_ADDR         0x83e0
#define START           0x83e4
#define IRQ             0x8018

#define W               32
#define H               16
#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define LIST            0x00820000
#define BS              0x00840000
#define OFFSET          0x220
#define ROOM            0x10000
#define CHAN            0x00900000

#define HERA            0x13200000
#define HW              256
#define HERA_LIST       0x00860000
#define HERA_BS         0x00880000
#define HERA_COPY       0x00a00000
#define HL_VLD          (1u << 31)
#define HL_TERM         (1u << 30)

#define CL_LAST         (1u << 31)
#define CABAC           (1u << 10)

static const uint8_t sps[] = { 0, 0, 0, 1, 0x67, 0x64, 0x00, 0x1e, 0xac, 0xd9 };
static const uint8_t pps[] = { 0, 0, 0, 1, 0x68, 0xee, 0x3c, 0x80 };
static const uint8_t idr[] = { 0, 0, 0, 1, 0x65, 0x88, 0x84, 0x00, 0x3f };
static const uint8_t pic[] = { 0, 0, 0, 1, 0x41, 0x9a, 0x00, 0x2f };
static const uint8_t vps5[] = { 0, 0, 0, 1, 0x40, 0x01, 0x0c, 0x01, 0xff };
static const uint8_t sps5[] = { 0, 0, 0, 1, 0x42, 0x01, 0x01, 0x01, 0x60 };
static const uint8_t pps5[] = { 0, 0, 0, 1, 0x44, 0x01, 0xc1, 0x73 };
static const uint8_t idr5[] = { 0, 0, 0, 1, 0x26, 0x01, 0xaf, 0x08 };
static const uint8_t idr5b[] = { 0, 0, 0, 1, 0x26, 0x01, 0xbb, 0x42 };

static GByteArray *cat(const uint8_t *a, size_t an, ...)
{
    GByteArray *g = g_byte_array_new();
    va_list ap;

    va_start(ap, an);
    while (a) {
        g_byte_array_append(g, a, an);
        a = va_arg(ap, const uint8_t *);
        an = a ? va_arg(ap, size_t) : 0;
    }
    va_end(ap);
    return g;
}

/* Code a picture of stream @chan; the slice data it wrote, in @out */
static uint32_t job(QTestState *qts, uint32_t chan, bool hevc, bool intra,
                    uint8_t *out)
{
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    uint8_t y[W * H], c[W * H / 2];
    uint32_t len;

    for (int i = 0; i < W * H; i++) {
        y[i] = (i / W) & 1 ? 84 : 180;
    }
    memset(c, 128, sizeof(c));
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));
    w[0] = CL_LAST | (hevc ? 1u << 28 : 0);
    w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
    w[2] = CABAC;
    w[3] = (intra ? 2u : 0) << 28 | 27 << 16;
    w[0x20] = SRC_Y;
    w[0x21] = SRC_C;
    w[0x22] = W;
    w[0x23] = chan;
    w[0x30] = BS;
    w[0x32] = OFFSET;
    w[0x33] = ROOM;
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    qtest_writel(qts, AVPU + IRQ, 1);
    qtest_writel(qts, AVPU + CL_ADDR, LIST);
    qtest_writel(qts, AVPU + START, 2);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, AVPU + IRQ) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, AVPU + IRQ) & 1, ==, 1);
    len = qtest_readl(qts, LIST + 0x41 * 4);
    g_assert_cmpuint(len, >, 0);
    qtest_memread(qts, BS + OFFSET, out, len);
    return len;
}

static void put(QTestState *qts, const GByteArray *hdr)
{
    qtest_memwrite(qts, BS + OFFSET - hdr->len, hdr->data, hdr->len);
}

static GByteArray *file(const char *dir, const char *name)
{
    g_autofree char *path = g_build_filename(dir, name, NULL);
    g_autofree char *got = NULL;
    gsize n;

    if (!g_file_get_contents(path, &got, &n, NULL)) {
        return NULL;
    }
    unlink(path);
    return g_byte_array_append(g_byte_array_new(), (guint8 *)got, n);
}

static void want(GByteArray *got, GByteArray *a, const uint8_t *d, size_t n)
{
    GByteArray *w = g_byte_array_append(g_byte_array_new(), a->data, a->len);

    g_byte_array_append(w, d, n);
    g_assert_nonnull(got);
    g_assert_cmpmem(got->data, got->len, w->data, w->len);
    g_byte_array_unref(w);
}

static void test_h264(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-tap-XXXXXX", NULL);
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-avpu.x-stream-tap=%s", dir);
    g_autofree uint8_t *d = g_malloc(ROOM);
    uint8_t zeros[16] = { 0 };
    GByteArray *h, *f;
    uint32_t n;

    /* a stream whose first picture has no parameter sets writes nothing */
    h = cat(idr, sizeof(idr), NULL);
    put(qts, h);
    job(qts, CHAN + 0x100, false, true, d);
    g_byte_array_unref(h);

    /*
     * stream 0: an IDR picture, after the stale parameter sets of an older
     * one and the zeros between; a P picture, after the stale ones again
     */
    h = cat(sps, sizeof(sps), pps, sizeof(pps), idr, sizeof(idr), NULL);
    qtest_memwrite(qts, BS + OFFSET - 400, sps, sizeof(sps));
    qtest_memwrite(qts, BS + OFFSET - 400 + sizeof(sps), pps, sizeof(pps));
    qtest_memwrite(qts, BS + OFFSET - 300, zeros, sizeof(zeros));
    put(qts, h);
    n = job(qts, CHAN, false, true, d);
    f = h;
    g_byte_array_append(f, d, n);

    h = cat(pic, sizeof(pic), NULL);
    put(qts, h);
    n = job(qts, CHAN, false, false, d);
    g_byte_array_append(f, h->data, h->len);
    g_byte_array_append(f, d, n);
    g_byte_array_unref(h);

    /* stream 1, the first to end an IDR picture with parameter sets */
    h = cat(sps, sizeof(sps), pps, sizeof(pps), idr, sizeof(idr), NULL);
    put(qts, h);
    n = job(qts, CHAN + 0x200, false, true, d);

    /* the files: streams are numbered by their first picture to be written */
    {
        GByteArray *s0 = file(dir, "avpu-0.h264");
        GByteArray *s1 = file(dir, "avpu-1.h264");

        g_assert_nonnull(s0);
        g_assert_cmpmem(s0->data, s0->len, f->data, f->len);
        want(s1, h, d, n);
        g_assert_null(file(dir, "avpu-2.h264"));
        g_byte_array_unref(s0);
        g_byte_array_unref(s1);
    }
    g_byte_array_unref(h);
    g_byte_array_unref(f);
    qtest_quit(qts);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

/*
 * libimp writes an IDR picture's PPS extension byte after the job: a look
 * before it is written finds a cut PPS, which is waited out
 */
static void test_h264_pps_late(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-tap-XXXXXX", NULL);
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-avpu.x-stream-tap=%s", dir);
    g_autofree uint8_t *d = g_malloc(ROOM);
    GByteArray *whole, *cut, *got;
    uint32_t n;

    whole = cat(sps, sizeof(sps), pps, sizeof(pps), idr, sizeof(idr), NULL);
    cut = cat(sps, sizeof(sps), pps, sizeof(pps) - 1, idr, sizeof(idr), NULL);

    put(qts, cut);
    n = job(qts, CHAN, false, true, d);
    qtest_clock_step(qts, 100 * SCALE_MS);
    g_assert_null(file(dir, "avpu-0.h264"));
    put(qts, whole);
    qtest_clock_step(qts, 10 * SCALE_MS);
    got = file(dir, "avpu-0.h264");
    want(got, whole, d, n);
    g_byte_array_unref(got);
    g_byte_array_unref(whole);
    g_byte_array_unref(cut);
    qtest_quit(qts);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

/* H.265: libimp writes the headers after the job; the stale ones stand first */
static void test_h265_after(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-tap-XXXXXX", NULL);
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-avpu.x-stream-tap=%s", dir);
    g_autofree uint8_t *d = g_malloc(ROOM);
    GByteArray *stale = cat(vps5, sizeof(vps5), sps5, sizeof(sps5),
                            pps5, sizeof(pps5), idr5b, sizeof(idr5b), NULL);
    GByteArray *fresh = cat(vps5, sizeof(vps5), sps5, sizeof(sps5),
                            pps5, sizeof(pps5), idr5, sizeof(idr5), NULL);
    GByteArray *got;
    uint32_t n;

    put(qts, stale);
    n = job(qts, CHAN, true, true, d);
    /* the job is over and the stale headers are all there is */
    qtest_clock_step(qts, 5 * SCALE_MS);
    g_assert_null(file(dir, "avpu-0.h265"));
    put(qts, fresh);
    qtest_clock_step(qts, 10 * SCALE_MS);
    got = file(dir, "avpu-0.h265");
    want(got, fresh, d, n);
    g_byte_array_unref(got);
    g_byte_array_unref(stale);
    g_byte_array_unref(fresh);
    qtest_quit(qts);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

/*
 * A guest that a busy host starves hands libimp's headers over late: a
 * picture is still written when they come seconds after the job, and is
 * dropped when they never do
 */
static void test_h265_late(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-tap-XXXXXX", NULL);
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-avpu.x-stream-tap=%s", dir);
    g_autofree uint8_t *d = g_malloc(ROOM);
    GByteArray *stale = cat(vps5, sizeof(vps5), sps5, sizeof(sps5),
                            pps5, sizeof(pps5), idr5b, sizeof(idr5b), NULL);
    GByteArray *fresh = cat(vps5, sizeof(vps5), sps5, sizeof(sps5),
                            pps5, sizeof(pps5), idr5, sizeof(idr5), NULL);
    GByteArray *got;
    uint32_t n;

    put(qts, stale);
    n = job(qts, CHAN, true, true, d);
    qtest_clock_step(qts, 3 * NANOSECONDS_PER_SECOND);
    g_assert_null(file(dir, "avpu-0.h265"));
    put(qts, fresh);
    qtest_clock_step(qts, 10 * SCALE_MS);
    got = file(dir, "avpu-0.h265");
    want(got, fresh, d, n);
    g_byte_array_unref(got);

    put(qts, stale);
    job(qts, CHAN, true, true, d);
    qtest_clock_step(qts, 12 * NANOSECONDS_PER_SECOND);
    put(qts, fresh);
    qtest_clock_step(qts, 10 * SCALE_MS);
    g_assert_null(file(dir, "avpu-0.h265"));
    g_byte_array_unref(stale);
    g_byte_array_unref(fresh);
    qtest_quit(qts);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

/* Code a HERA picture; its slice data, as the core wrote it, in @out */
static uint32_t hera_job(QTestState *qts, bool escape, uint8_t *out)
{
    uint32_t pairs[32], n = 0, len;
    uint8_t y[HW * HW], c[HW * HW / 2];
    const struct {
        uint32_t reg, val;
    } regs[] = {
        { 0x2000c, SRC_Y }, { 0x20010, SRC_C }, { 0x20014, HW | HW << 16 },
        { 0x68004, HW << 16 | HW },
        { 0x68000, 2 << 2 | (escape ? 1u << 28 : 0) },
        { 0x60004, 20 << 1 },
        { 0x20004, 1 },
    };

    /* a flat picture: its coded bytes hold the zero runs that need an 03 */
    memset(y, 128, sizeof(y));
    memset(c, 128, sizeof(c));
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));
    for (int i = 0; i < ARRAY_SIZE(regs); i++) {
        pairs[n++] = cpu_to_le32(regs[i].val);
        pairs[n++] = cpu_to_le32(HL_VLD | (i == ARRAY_SIZE(regs) - 1 ?
                                           HL_TERM : 0) | regs[i].reg);
    }
    qtest_memwrite(qts, HERA_LIST, pairs, n * 4);
    qtest_writel(qts, HERA + 0x90008, HERA_BS);
    qtest_writel(qts, HERA + 0x90068, HERA_LIST);
    qtest_writel(qts, HERA + 0x8, 1);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, HERA + 0x2c) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, HERA + 0x2c) & 1, ==, 1);
    len = qtest_readl(qts, HERA + 0x44);
    g_assert_cmpuint(len, >, 0);
    qtest_memread(qts, HERA_BS, out, len);
    return len;
}

/* @d with the 03 bytes libimp adds */
static GByteArray *escaped(const uint8_t *d, size_t n)
{
    static const uint8_t epb = 3;
    GByteArray *g = g_byte_array_new();
    int zeros = 0;

    for (size_t i = 0; i < n; i++) {
        if (zeros >= 2 && d[i] <= 3) {
            g_byte_array_append(g, &epb, 1);
            zeros = 0;
        }
        g_byte_array_append(g, &d[i], 1);
        zeros = d[i] ? 0 : zeros + 1;
    }
    return g;
}

/*
 * One HERA picture. libimp's copy goes where no core wrote, the headers
 * right before it: the core's bytes escaped by libimp (@escape false) or as
 * they are
 */
static void hera_case(bool escape)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-tap-XXXXXX", NULL);
    QTestState *qts = qtest_initf("-machine ingenic-t32 "
                                  "-global ingenic-hera.x-stream-tap=%s", dir);
    g_autofree uint8_t *d = g_malloc(ROOM);
    GByteArray *hdr = cat(vps5, sizeof(vps5), sps5, sizeof(sps5),
                          pps5, sizeof(pps5), idr5, sizeof(idr5), NULL);
    GByteArray *copy, *got;
    uint32_t n = hera_job(qts, escape, d);

    /* escaped again, the data grows: or the two cases are the same */
    copy = escaped(d, n);
    g_assert_cmpuint(copy->len, >, n);
    if (escape) {
        g_byte_array_set_size(copy, 0);
        g_byte_array_append(copy, d, n);
    }
    qtest_memwrite(qts, HERA_COPY - hdr->len, hdr->data, hdr->len);
    qtest_memwrite(qts, HERA_COPY, copy->data, copy->len);
    qtest_clock_step(qts, 10 * SCALE_MS);
    got = file(dir, "hera-0.h265");
    want(got, hdr, copy->data, copy->len);
    g_byte_array_unref(got);
    g_byte_array_unref(copy);
    g_byte_array_unref(hdr);
    qtest_quit(qts);
    g_assert_cmpint(rmdir(dir), ==, 0);
}

static void test_hera_t32(void)
{
    hera_case(false);
}

static void test_hera_t33(void)
{
    hera_case(true);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic/enc-tap/avpu/h264", test_h264);
    qtest_add_func("ingenic/enc-tap/avpu/h264-pps-late", test_h264_pps_late);
    qtest_add_func("ingenic/enc-tap/avpu/h265-headers-after", test_h265_after);
    qtest_add_func("ingenic/enc-tap/avpu/h265-headers-late", test_h265_late);
    qtest_add_func("ingenic/enc-tap/hera/h265-escaped-by-libimp",
                   test_hera_t32);
    qtest_add_func("ingenic/enc-tap/hera/h265-escaped-by-core",
                   test_hera_t33);
    return g_test_run();
}
