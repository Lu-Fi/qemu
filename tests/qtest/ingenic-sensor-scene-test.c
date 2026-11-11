/*
 * QTest testcase for the image sensor's "scene" property: a YUV4MPEG2 file
 * is parsed and loops with the guest's frame count, any size of it is
 * scaled to the size asked, a FIFO gives
 * the latest frame written to it (a partial frame waits, a writer that
 * goes and comes back is taken up again), and a file that is no 4:2:0
 * YUV4MPEG2 stream is refused when the device is realized.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define VIC(n)          (ISP + 0x80000 + (n) * 0x10000)
#define T41_WDMA        (ISP + 0x20000)
#define T41_DMA_EN      (ISP + 0x400b0)
#define BUF             0x00400000
#define W               64
#define H               48
#define FRAME_NS        40000000        /* fps=25 */
#define T41_FIRST_NS    135000000

static char *tmpdir;

static QTestState *t41_start(const char *scene, const char *extra)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 -device image-sensor,"
                                  "model=jxq03,bus=i2c0,fps=25,motion=off,"
                                  "scene=%s%s", scene, extra);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, 0x10023000 + 0x0c, 1);        /* DPHY_RSTZ */
    qtest_writel(qts, 0x10023000 + 0x10, 1);        /* CSI2_RESETN */
    qtest_writel(qts, VIC(0) + 0x00c, 2);           /* MIPI */
    qtest_writel(qts, VIC(0) + 0x014, 1);           /* RAW10 */
    qtest_writel(qts, VIC(0) + 0x004, W << 16 | H);
    qtest_writel(qts, T41_DMA_EN, 0x3f);
    qtest_writel(qts, T41_WDMA + 0x14, 0x10020);    /* RAW16 */
    qtest_writel(qts, T41_WDMA + 0x1c, W * 2);
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_writel(qts, T41_WDMA + 0x04, 1);
    qtest_writel(qts, VIC(0), 1);
    qtest_clock_step(qts, T41_FIRST_NS + FRAME_NS / 2);
    return qts;
}

static void next_frame(QTestState *qts, uint16_t *raw)
{
    qtest_writel(qts, T41_WDMA + 0x18, BUF);
    qtest_clock_step(qts, FRAME_NS);
    qtest_memread(qts, BUF, raw, W * H * 2);
    for (int i = 0; i < W * H; i++) {
        raw[i] = lduw_le_p(&raw[i]);
    }
}

/* The mean sample of the columns @x0 to @x1 */
static double mean(const uint16_t *raw, int x0, int x1)
{
    double sum = 0;

    for (int y = 0; y < H; y++) {
        for (int x = x0; x < x1; x++) {
            sum += raw[y * W + x];
        }
    }
    return sum / (H * (x1 - x0));
}

static GByteArray *header(int w, int h, const char *chroma)
{
    GByteArray *a = g_byte_array_new();
    g_autofree char *line = g_strdup_printf("YUV4MPEG2 W%d H%d F25:1 Ip A1:1 "
                                            "%s\n", w, h, chroma);

    g_byte_array_append(a, (const guint8 *)line, strlen(line));
    return a;
}

/* A frame, its columns left of @split at luma @l0, the others at @l1 */
static void add_frame(GByteArray *a, int w, int h, int split, int l0, int l1)
{
    int cw = (w + 1) / 2, ch = (h + 1) / 2;
    g_autofree uint8_t *p = g_malloc(w * h + 2 * cw * ch);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            p[y * w + x] = x < split ? l0 : l1;
        }
    }
    memset(p + w * h, 128, 2 * cw * ch);
    g_byte_array_append(a, (const guint8 *)"FRAME\n", 6);
    g_byte_array_append(a, p, w * h + 2 * cw * ch);
}

static char *write_file(const char *name, const GByteArray *a)
{
    char *path = g_build_filename(tmpdir, name, NULL);

    g_assert(g_file_set_contents(path, (const char *)a->data, a->len, NULL));
    return path;
}

/* Three frames of three levels: the guest's frame n shows n modulo 3 */
static void test_loop(void)
{
    GByteArray *a = header(32, 24, "C420jpeg");
    char *path;
    QTestState *qts;
    uint16_t f[7][W * H];

    add_frame(a, 32, 24, 32, 60, 60);
    add_frame(a, 32, 24, 32, 120, 120);
    add_frame(a, 32, 24, 32, 200, 200);
    path = write_file("loop.y4m", a);
    qts = t41_start(path, "");
    for (int i = 0; i < 7; i++) {
        next_frame(qts, f[i]);
    }
    for (int i = 0; i + 3 < 7; i++) {
        g_assert(!memcmp(f[i], f[i + 3], sizeof(f[i])));
        g_assert(memcmp(f[i], f[i + 1], sizeof(f[i])));
    }
    qtest_quit(qts);

    /* and a second run is the same */
    qts = t41_start(path, "");
    for (int i = 0; i < 7; i++) {
        uint16_t g[W * H];

        next_frame(qts, g);
        g_assert(!memcmp(f[i], g, sizeof(g)));
    }
    qtest_quit(qts);
    g_byte_array_free(a, TRUE);
    g_free(path);
}

/* A left half and a right half at any size: the same picture at 64x48 */
static void test_scale(void)
{
    static const int sizes[][2] = { { 16, 12 }, { 64, 48 }, { 128, 96 },
                                    { 99, 77 } };
    uint16_t ref[W * H], f[W * H];

    for (int i = 0; i < ARRAY_SIZE(sizes); i++) {
        GByteArray *a = header(sizes[i][0], sizes[i][1], "C420mpeg2");
        g_autofree char *name = g_strdup_printf("s%d.y4m", i);
        char *path;
        QTestState *qts;

        add_frame(a, sizes[i][0], sizes[i][1], sizes[i][0] / 2, 40, 200);
        path = write_file(name, a);
        qts = t41_start(path, "");
        next_frame(qts, f);
        g_assert_cmpfloat(mean(f, 0, W / 2 - 2), <, mean(f, W / 2 + 2, W) / 2);
        if (i == 0) {
            memcpy(ref, f, sizeof(ref));
        } else if (i < 3) {
            g_assert(!memcmp(ref, f, sizeof(ref)));
        }
        qtest_quit(qts);
        g_byte_array_free(a, TRUE);
        g_free(path);
    }
}

/* The model's reader thread takes what is written in its own time */
static void feed(int fd, const GByteArray *a, size_t from, size_t to)
{
    g_assert_cmpint(write(fd, a->data + from, to - from), ==, to - from);
    g_usleep(300000);
}

/* The latest frame, a partial one waiting, a writer gone and back */
#ifndef _WIN32
static void test_fifo(void)
{
    g_autofree char *path = g_build_filename(tmpdir, "live", NULL);
    GByteArray *a = header(16, 12, "C420jpeg");
    size_t hdr = a->len, frame;
    QTestState *qts;
    uint16_t f[W * H], g[W * H];
    int fd;

    g_assert_cmpint(mkfifo(path, 0600), ==, 0);
    qts = t41_start(path, "");
    /* no writer yet: the guest runs on */
    next_frame(qts, f);

    add_frame(a, 16, 12, 16, 60, 60);
    frame = a->len - hdr;
    add_frame(a, 16, 12, 16, 200, 200);
    fd = open(path, O_WRONLY | O_NONBLOCK);
    g_assert_cmpint(fd, >=, 0);
    feed(fd, a, 0, hdr + frame);
    next_frame(qts, f);
    g_assert_cmpfloat(mean(f, 0, W), <, 20000);
    feed(fd, a, hdr + frame, hdr + frame + 100);
    next_frame(qts, g);
    g_assert(!memcmp(f, g, sizeof(f)));         /* a partial frame waits */
    feed(fd, a, hdr + frame + 100, a->len);
    next_frame(qts, g);
    g_assert_cmpfloat(mean(g, 0, W), >, mean(f, 0, W) * 1.5);
    close(fd);
    next_frame(qts, f);
    g_assert(!memcmp(f, g, sizeof(f)));         /* the writer is gone */
    next_frame(qts, f);
    g_assert(!memcmp(f, g, sizeof(f)));

    /* back, with another size */
    fd = open(path, O_WRONLY | O_NONBLOCK);
    g_assert_cmpint(fd, >=, 0);
    g_byte_array_free(a, TRUE);
    a = header(32, 24, "C420jpeg");
    add_frame(a, 32, 24, 32, 120, 120);
    feed(fd, a, 0, a->len);
    next_frame(qts, f);
    g_assert_cmpfloat(mean(f, 0, W), >, mean(g, 0, W) / 3);
    g_assert_cmpfloat(mean(f, 0, W), <, mean(g, 0, W) * 0.9);
    close(fd);
    qtest_quit(qts);
    g_byte_array_free(a, TRUE);
    unlink(path);
}

static void refused(const char *name, const GByteArray *a, const char *why)
{
    g_autofree char *path = a ? write_file(name, a) :
                            g_build_filename(tmpdir, name, NULL);
    g_autofree char *dev = g_strdup_printf("image-sensor,model=jxq03,"
                                           "bus=i2c0,scene=%s", path);
    const char *args[] = { qtest_qemu_binary(NULL), "-machine", "ingenic-t41",
                           "-display", "none", "-nodefaults", "-S",
                           "-audio", "none", "-accel", "qtest", "-device",
                           dev, NULL };
    g_autofree char *err = NULL;
    int status;

    g_assert(g_spawn_sync(NULL, (char **)args, NULL, G_SPAWN_STDOUT_TO_DEV_NULL,
                          NULL, NULL, NULL, &err, &status, NULL));
    g_assert(!WIFEXITED(status) || WEXITSTATUS(status));
    g_assert(strstr(err, why));
}
#endif

static void test_bad(void)
{
    GByteArray *a = g_byte_array_new();

    g_byte_array_append(a, (const guint8 *)"hello\n", 6);
    refused("text.y4m", a, "YUV4MPEG2");
    g_byte_array_free(a, TRUE);

    a = header(16, 12, "C444");
    add_frame(a, 16, 12, 8, 1, 2);
    refused("c444.y4m", a, "4:2:0");
    g_byte_array_free(a, TRUE);

    a = header(16, 12, "C420jpeg");
    g_byte_array_append(a, (const guint8 *)"FRAME\nshort", 11);
    refused("short.y4m", a, "no complete");
    g_byte_array_free(a, TRUE);

    refused("none.y4m", NULL, "none.y4m");
}

int main(int argc, char **argv)
{
    int ret;

    g_test_init(&argc, &argv, NULL);
    tmpdir = g_dir_make_tmp("scene-test-XXXXXX", NULL);
    g_assert(tmpdir);
    qtest_add_func("/ingenic-sensor-scene/loop", test_loop);
    qtest_add_func("/ingenic-sensor-scene/scale", test_scale);
#ifndef _WIN32
    qtest_add_func("/ingenic-sensor-scene/fifo", test_fifo);
#endif
    qtest_add_func("/ingenic-sensor-scene/bad", test_bad);
    ret = g_test_run();
    g_rmdir(tmpdir);
    return ret;
}
