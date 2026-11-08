/*
 * QTest testcase for the ISPs' frame taps
 *
 * With "x-frame-tap" set, an ISP writes each channel's picture as a YUV4MPEG2
 * stream and the raw frame of each VIC as 16-bit Bayer samples. The test
 * checks the files against what the guest reads from its own buffers: a T20's
 * NV12 writers and its raw writer, a T23's channel and the same channel
 * taken over IVDC (no buffer, the picture only in the tap), a T40's raw
 * capture; that a FIFO made beforehand gets the stream; and that with the
 * tap off no file is made and the guest's buffers are the same.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

#define W               64
#define H               32
#define FRAME_NS        40000000        /* fps=25 */
#define SENSOR          "-device image-sensor,model=jxq03,bus=i2c0,fps=25," \
                        "motion=off"
#define CPM_CLKGR0      0x10000020
#define CPM_CIMCDR      0x10000090
#define YUV             (W * H * 3 / 2)
#define Y4M_HDR         "YUV4MPEG2 W64 H32 F25:1 Ip A1:1 C420mpeg2\n"

/* The T20's */
#define T20_CORE        0x13380000
#define T20_VIC         0x13300000
#define FR              (T20_CORE + 0xb00)
#define DS1             (T20_CORE + 0xc00)
#define BANK(n)         (0x02000000 + (n) * 0x100000)
#define ON              BIT(9)
#define NV12            13
#define UV_PLANE        (1 << 6)
#define RAW16           6

/* The T23's, T40's */
#define ISP             0x13300000
#define BUF             0x00400000
#define T23_CH_EN       0xd040
#define T23_CH          0xd100
#define T23_VIC         0xe0000
#define T40_VIC(n)      (ISP + 0x80000 + (n) * 0x10000)
#define MDMA            0x300

static char *tmpdir(void)
{
    g_autofree char *tpl = g_build_filename(g_get_tmp_dir(),
                                            "ingenic-frame-tap-XXXXXX", NULL);

    g_assert_nonnull(g_mkdtemp(tpl));
    return g_steal_pointer(&tpl);
}

static void rmdir_files(const char *dir)
{
    GDir *d = g_dir_open(dir, 0, NULL);
    const char *n;

    while (d && (n = g_dir_read_name(d))) {
        g_autofree char *p = g_build_filename(dir, n, NULL);

        unlink(p);
    }
    if (d) {
        g_dir_close(d);
    }
    rmdir(dir);
}

static uint8_t *slurp(const char *dir, const char *name, size_t *len)
{
    g_autofree char *p = g_build_filename(dir, name, NULL);
    gchar *data = NULL;

    g_assert_true(g_file_get_contents(p, &data, len, NULL));
    return (uint8_t *)data;
}

/* Frame @i of a W x H stream: Y at [0], U at [W * H], V after */
static const uint8_t *y4m_frame(const uint8_t *file, size_t len, unsigned i,
                                unsigned frame_bytes)
{
    size_t hdr = strlen(Y4M_HDR), per = 6 + frame_bytes;

    g_assert_cmpuint(len, >=, hdr + (i + 1) * per);
    g_assert_cmpmem(file, hdr, Y4M_HDR, hdr);
    g_assert_cmpmem(file + hdr + i * per, 6, "FRAME\n", 6);
    return file + hdr + i * per + 6;
}

/* The NV12 at @y, @uv in the guest as I420 */
static void guest_i420(QTestState *qts, uint32_t y, uint32_t uv, uint8_t *out)
{
    uint8_t c[W * H / 2];

    qtest_memread(qts, y, out, W * H);
    qtest_memread(qts, uv, c, sizeof(c));
    for (unsigned i = 0; i < W * H / 4; i++) {
        out[W * H + i] = c[2 * i];
        out[W * H * 5 / 4 + i] = c[2 * i + 1];
    }
}

static void t20_writer(QTestState *qts, uint32_t base, uint32_t cfg,
                       uint32_t size, uint32_t line, unsigned bank0)
{
    for (int i = 0; i < 3; i++) {
        qtest_memset(qts, BANK(bank0 + i), 0xa5, 0x10000);
        qtest_writel(qts, base + 8 + i * 4, BANK(bank0 + i));
    }
    qtest_writel(qts, base + 0x1c, 2);
    qtest_writel(qts, base + 0x20, line);
    qtest_writel(qts, base + 0x04, size);
    qtest_writel(qts, base, cfg | ON);
    qtest_writel(qts, base + 0x24, 2);
}

/* A T20 with the FR writer on NV12 and DS1 on RAW16 (bayer order @bayer) */
static QTestState *t20_run(const char *dir, unsigned bayer)
{
    g_autofree char *args = g_strdup_printf(
        "-machine ingenic-t20 " SENSOR " %s%s", dir ? "-global "
        "ingenic-t20-isp.x-frame-tap=" : "", dir ? dir : "");
    QTestState *qts = qtest_init(args);

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, T20_CORE + 0x10, W);
    qtest_writel(qts, T20_CORE + 0x14, H);
    qtest_writel(qts, T20_CORE + 0x18, bayer);
    t20_writer(qts, FR, NV12, H << 16 | W, W, 0);
    t20_writer(qts, FR + 0x80, NV12 | UV_PLANE, H << 16 | W, W, 4);
    t20_writer(qts, DS1, RAW16, H << 16 | W, W * 2, 8);
    qtest_writel(qts, T20_VIC, 1);
    qtest_clock_step(qts, FRAME_NS * 5);
    return qts;
}

static void test_t20(void)
{
    g_autofree char *dir = tmpdir();
    const char *raw_name = "raw-0-64x32-bayer_bggr16le.raw";
    QTestState *qts = t20_run(dir, 3);
    uint8_t want[YUV], mem[W * H * 2], bare[W * H * 2];
    size_t len, rlen;
    uint8_t *y4m, *raw;

    /* the writers' banks hold frame n at bank n % 3; compare the first */
    guest_i420(qts, BANK(1), BANK(5), want);
    qtest_memread(qts, BANK(9), mem, sizeof(mem));
    qtest_quit(qts);

    y4m = slurp(dir, "isp-0-0.y4m", &len);
    g_assert_cmpmem(y4m_frame(y4m, len, 1, YUV), YUV, want, YUV);
    raw = slurp(dir, raw_name, &rlen);
    g_assert_cmpuint(rlen, >=, 2 * sizeof(mem));
    g_assert_cmpmem(raw + sizeof(mem), sizeof(mem), mem, sizeof(mem));
    /* 10 bits at the top */
    for (unsigned i = 0; i < sizeof(mem); i += 2) {
        g_assert_cmphex(lduw_le_p(raw + i) & 0x3f, ==, 0);
    }
    g_free(y4m);
    g_free(raw);

    /* off: no file is made, and the banks are the same */
    rmdir_files(dir);
    qts = t20_run(NULL, 3);
    guest_i420(qts, BANK(1), BANK(5), want);
    qtest_memread(qts, BANK(9), bare, sizeof(bare));
    qtest_quit(qts);
    g_assert_cmpmem(bare, sizeof(bare), mem, sizeof(mem));
    g_assert_false(g_file_test(dir, G_FILE_TEST_EXISTS));
}

static void t23_push(QTestState *qts, bool ivdc)
{
    uint32_t c = ISP + T23_CH;

    qtest_writel(qts, ISP + T23_CH_EN, 1);
    qtest_writel(qts, c + 0x04, W << 16 | H);
    qtest_writel(qts, c + 0x40, W);
    qtest_writel(qts, c + 0x60, W);
    qtest_writel(qts, c + 0x30, ivdc ? 0x80000000 : BUF);
    qtest_writel(qts, c + 0x50, ivdc ? 0x81000000 : BUF + 0x1000);
    qtest_writel(qts, ISP + T23_VIC, 1);
    qtest_clock_step(qts, FRAME_NS);
}

static QTestState *t23_start(const char *dir)
{
    QTestState *qts = qtest_initf("-machine ingenic-t23 " SENSOR " %s%s",
                                  dir ? "-global ingenic-isp.x-frame-tap=" : "",
                                  dir ? dir : "");

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, CPM_CIMCDR, 0);
    return qts;
}

/*
 * A channel's picture in the tap is what it writes to the guest; over IVDC
 * it is the same, with nothing written
 */
static void test_t23(void)
{
    g_autofree char *dir = tmpdir();
    g_autofree char *dir2 = tmpdir();
    uint8_t want[YUV], bare[YUV];
    size_t len;
    uint8_t *y4m;
    QTestState *qts = t23_start(dir);

    t23_push(qts, false);
    guest_i420(qts, BUF, BUF + 0x1000, want);
    qtest_quit(qts);
    y4m = slurp(dir, "isp-0-0.y4m", &len);
    g_assert_cmpuint(len, ==, strlen(Y4M_HDR) + 6 + YUV);
    g_assert_cmpmem(y4m_frame(y4m, len, 0, YUV), YUV, want, YUV);
    g_free(y4m);

    qts = t23_start(dir2);
    qtest_memset(qts, BUF, 0x5a, 0x2000);
    t23_push(qts, true);
    qtest_memread(qts, BUF, bare, 4);
    g_assert_cmphex(bare[0], ==, 0x5a);
    qtest_quit(qts);
    y4m = slurp(dir2, "isp-0-0.y4m", &len);
    g_assert_cmpuint(len, ==, strlen(Y4M_HDR) + 6 + YUV);
    g_assert_cmpmem(y4m_frame(y4m, len, 0, YUV), YUV, want, YUV);
    g_free(y4m);

    /* off: the same buffer, no file */
    qts = t23_start(NULL);
    t23_push(qts, false);
    guest_i420(qts, BUF, BUF + 0x1000, bare);
    qtest_quit(qts);
    g_assert_cmpmem(bare, YUV, want, YUV);

    rmdir_files(dir);
    rmdir_files(dir2);
}

/* The T40's raw frame, as its VIC's capture writes it */
static void test_t40_raw(void)
{
    g_autofree char *dir = tmpdir();
    QTestState *qts = qtest_initf("-machine ingenic-t40 " SENSOR
                                  " -global ingenic-isp.x-frame-tap=%s", dir);
    uint32_t v = T40_VIC(0);
    uint8_t mem[W * H * 2];
    size_t len;
    uint8_t *raw;

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_writel(qts, v + 0x004, W << 16 | H);
    qtest_writel(qts, v + 0x014, 1);                /* RAW10 */
    qtest_writel(qts, v + MDMA + 0x08, 1);
    qtest_writel(qts, v + MDMA + 0x04, W << 16 | H);
    qtest_writel(qts, v + MDMA + 0x10, W * 2);
    qtest_writel(qts, v + MDMA + 0x18, BUF);
    qtest_writel(qts, v + MDMA, 0x80010020);        /* one frame */
    qtest_writel(qts, v, 1);
    qtest_memread(qts, BUF, mem, sizeof(mem));
    qtest_clock_step(qts, FRAME_NS * 3);
    qtest_quit(qts);

    raw = slurp(dir, "raw-0-64x32-bayer_rggb16le.raw", &len);
    g_assert_cmpuint(len, >=, 3 * sizeof(mem));
    g_assert_cmpuint(len % sizeof(mem), ==, 0);
    g_assert_cmpmem(raw, sizeof(mem), mem, sizeof(mem));
    for (unsigned i = 0; i < sizeof(mem); i += 2) {
        g_assert_cmphex(lduw_le_p(raw + i) & 0x3f, ==, 0);      /* 10 bits */
    }
    g_assert_cmpuint(lduw_le_p(raw), !=, 0);
    g_free(raw);
    rmdir_files(dir);
}

#ifndef _WIN32
/* A FIFO made beforehand gets the header and then the frames */
static void test_fifo(void)
{
    g_autofree char *dir = tmpdir();
    g_autofree char *path = g_build_filename(dir, "isp-0-0.y4m", NULL);
    uint8_t buf[4096];
    GByteArray *got = g_byte_array_new();
    QTestState *qts;
    int fd;

    g_assert_cmpint(mkfifo(path, 0600), ==, 0);
    qts = t23_start(dir);
    /* a reader appears; the first frame after it goes in */
    fd = open(path, O_RDONLY | O_NONBLOCK);
    g_assert_cmpint(fd, >=, 0);
    t23_push(qts, false);
    for (;;) {
        ssize_t r = read(fd, buf, sizeof(buf));

        if (r <= 0) {
            break;
        }
        g_byte_array_append(got, buf, r);
    }
    close(fd);
    qtest_quit(qts);
    g_assert_cmpuint(got->len, ==, strlen(Y4M_HDR) + 6 + YUV);
    g_assert_cmpmem(got->data, strlen(Y4M_HDR), Y4M_HDR, strlen(Y4M_HDR));
    g_byte_array_unref(got);
    unlink(path);
    rmdir_files(dir);
}
#endif

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("/ingenic-frame-tap/t20", test_t20);
    qtest_add_func("/ingenic-frame-tap/t23", test_t23);
    qtest_add_func("/ingenic-frame-tap/t40-raw", test_t40_raw);
#ifndef _WIN32
    qtest_add_func("/ingenic-frame-tap/fifo", test_fifo);
#endif
    return g_test_run();
}
