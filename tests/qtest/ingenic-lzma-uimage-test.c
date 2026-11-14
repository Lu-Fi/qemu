/*
 * QTest testcase for LZMA-compressed uImages given with -kernel
 *
 * A 512-byte payload packed as an "lzma alone" stream, the way thingino's
 * kernels are (the size field all ones and an end marker), must land at the
 * load address the header names, also with the size field filled in. A
 * truncated stream, a bad property byte, a bad first byte and a size that
 * disagrees with the stream must stop the machine with an error instead of
 * starting it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define LOAD_ADDR   0x80010000
#define PHYS_ADDR   0x00010000
#define PAYLOAD_LEN 512
#define IH_COMP_LZMA 3

/* The payload below, from lzma.compress(FORMAT_ALONE) with a 64 KiB window */
static const uint8_t packed[] = {
    0x5d, 0x00, 0x00, 0x01, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0x00, 0x63, 0x1f, 0x8c, 0x26, 0xb2, 0x5b, 0x76, 0x3e, 0x4e, 0xde,
    0xea, 0x16, 0x09, 0xd3, 0xc4, 0x87, 0x24, 0x60, 0xbe, 0xfe, 0x15, 0x19,
    0x98, 0xb3, 0x52, 0xc8, 0x5f, 0x58, 0xa8, 0x37, 0x16, 0x4b, 0xc0, 0x50,
    0xb0, 0x48, 0xf5, 0x0f, 0x14, 0x59, 0xb1, 0xd0, 0x68, 0x3e, 0x20, 0xd4,
    0xc7, 0x46, 0xbe, 0x58, 0x40, 0xa5, 0x67, 0x30, 0xe9, 0xd7, 0x98, 0xc6,
    0xbb, 0x94, 0xde, 0xce, 0x22, 0x1e, 0xb4, 0x9b, 0xfe, 0x85, 0x29, 0x50,
    0xca, 0x02, 0x00, 0x63, 0xe1, 0x83, 0xa1, 0x19, 0x48, 0x25, 0xd1, 0xd4,
    0x18, 0xa1, 0x06, 0xa5, 0xab, 0xfd, 0x48, 0xba, 0x76, 0xae, 0xf8, 0xa5,
    0x01, 0xb2, 0xfc, 0x2e, 0x35, 0x51, 0x90, 0x6d, 0x44, 0xab, 0x04, 0xc8,
    0x3d, 0xad, 0x96, 0xb1, 0xe8, 0x93, 0x79, 0x3e, 0x29, 0xd8, 0x1b, 0xa8,
    0x6a, 0xfd, 0x7c, 0x84, 0xb7, 0x20, 0xa7, 0x60, 0x1c, 0x36, 0x8d, 0xcc,
    0x00, 0x4e, 0x41, 0x16, 0xf8, 0xbc, 0x7f, 0x75, 0xcb, 0x2b, 0x30, 0x48,
    0x78, 0x13, 0xae, 0xd6, 0x89, 0x0c, 0x36, 0x9d, 0x82, 0xf1, 0x3d, 0xda,
    0x79, 0xbd, 0xaa, 0x8c, 0xaa, 0x7c, 0xd9, 0xf1, 0x5d, 0xda, 0x7a, 0x75,
    0xe5, 0x90, 0x63, 0x31, 0x6a, 0xd2, 0xfa, 0x19, 0x66, 0xb2, 0x0b, 0xb6,
    0x73, 0x3c, 0x74, 0x82, 0x48, 0x81, 0xea, 0x48, 0x32, 0x79, 0xed, 0x6e,
    0x9d, 0x3d, 0xbf, 0xb5, 0xcc, 0x72, 0x5b, 0xd5, 0x3b, 0xde, 0xba, 0x26,
    0x4c, 0xd6, 0x61, 0x89, 0xeb, 0xe8, 0x7f, 0xcf, 0x19, 0x89, 0x64, 0x33,
    0xf5, 0x16, 0xa8, 0x99, 0xca, 0x97, 0x64, 0x8f, 0xc2, 0xc3, 0x52, 0x7c,
    0x1e, 0x69, 0x0a, 0x1b, 0x2c, 0xd7, 0x20, 0xe2, 0xa1, 0x71, 0x62, 0xe2,
    0xc0, 0x0a, 0x05, 0x19, 0xda, 0xf4, 0x87, 0xe8, 0x44, 0x11, 0xf9, 0x06,
    0x51, 0x26, 0xbd, 0x9d, 0xec, 0x0e, 0x7c, 0xf2, 0x31, 0xc7, 0x28, 0xd0,
    0xf2, 0xd6, 0xcf, 0xbc, 0xe6, 0x43, 0xbe, 0x2c, 0x6f, 0x6d, 0xa2, 0x81,
    0x6a, 0xff, 0xff, 0xfd, 0x76, 0x06, 0xa0
};

/* 256 bytes of a linear congruential generator, twice */
static void payload(uint8_t *out)
{
    uint32_t x = 1;

    for (int i = 0; i < 256; i++) {
        x = x * 1103515245 + 12345;
        out[i] = out[256 + i] = x >> 16;
    }
}

static char *make_uimage(const uint8_t *data, size_t len)
{
    g_autofree uint8_t *img = g_malloc0(64 + len);
    char *path;
    int fd = g_file_open_tmp("ingenic-lzma-XXXXXX", &path, NULL);

    stl_be_p(img, 0x27051956);
    stl_be_p(img + 12, len);
    stl_be_p(img + 16, LOAD_ADDR);
    stl_be_p(img + 20, LOAD_ADDR);
    img[28] = 5;                /* Linux */
    img[29] = 5;                /* MIPS */
    img[30] = 2;                /* kernel */
    img[31] = IH_COMP_LZMA;
    memcpy(img + 64, data, len);
    g_assert(write(fd, img, 64 + len) == 64 + len);
    close(fd);
    return path;
}

static void check_loaded(const uint8_t *stream, size_t len)
{
    uint8_t want[PAYLOAD_LEN], got[PAYLOAD_LEN + 16];
    g_autofree char *kernel = make_uimage(stream, len);
    QTestState *qts = qtest_initf("-machine ingenic-t31 -m 64 -kernel %s",
                                  kernel);

    payload(want);
    qtest_memread(qts, PHYS_ADDR, got, sizeof(got));
    g_assert_cmpmem(got, PAYLOAD_LEN, want, PAYLOAD_LEN);
    /* nothing is written past the unpacked size */
    for (int i = PAYLOAD_LEN; i < sizeof(got); i++) {
        g_assert_cmphex(got[i], ==, 0);
    }
    qtest_quit(qts);
    unlink(kernel);
}

static void test_unknown_size(void)
{
    check_loaded(packed, sizeof(packed));
}

static void test_known_size(void)
{
    uint8_t s[sizeof(packed)];

    memcpy(s, packed, sizeof(s));
    stq_le_p(s + 5, PAYLOAD_LEN);
    check_loaded(s, sizeof(s));
}

static void check_refused(const uint8_t *stream, size_t len)
{
    g_autofree char *kernel = make_uimage(stream, len);
    g_autofree char *err = NULL;
    const char *argv[] = {
        qtest_qemu_binary(NULL), "-machine", "ingenic-t31", "-m", "64",
        "-display", "none", "-monitor", "none", "-S", "-kernel", kernel, NULL
    };
    int status = 0;

    g_assert(g_spawn_sync(NULL, (char **)argv, NULL, G_SPAWN_STDOUT_TO_DEV_NULL,
                          NULL, NULL, NULL, &err, &status, NULL));
    g_assert_true(WIFEXITED(status));
    g_assert_cmpint(WEXITSTATUS(status), ==, 1);
    g_assert_nonnull(strstr(err, "lzma"));
    unlink(kernel);
}

static void test_truncated(void)
{
    check_refused(packed, sizeof(packed) - 40);
}

static void test_header_only(void)
{
    check_refused(packed, 8);
}

static void test_bad_props(void)
{
    uint8_t s[sizeof(packed)];

    memcpy(s, packed, sizeof(s));
    s[0] = 225;
    check_refused(s, sizeof(s));
}

static void test_bad_first_byte(void)
{
    uint8_t s[sizeof(packed)];

    memcpy(s, packed, sizeof(s));
    s[13] = 0x80;
    check_refused(s, sizeof(s));
}

static void test_size_mismatch(void)
{
    uint8_t s[sizeof(packed)];

    memcpy(s, packed, sizeof(s));
    stq_le_p(s + 5, PAYLOAD_LEN + 100);
    check_refused(s, sizeof(s));
    stq_le_p(s + 5, PAYLOAD_LEN - 100);
    check_refused(s, sizeof(s));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic/lzma-uimage/unknown-size", test_unknown_size);
    qtest_add_func("ingenic/lzma-uimage/known-size", test_known_size);
    qtest_add_func("ingenic/lzma-uimage/truncated", test_truncated);
    qtest_add_func("ingenic/lzma-uimage/header-only", test_header_only);
    qtest_add_func("ingenic/lzma-uimage/bad-props", test_bad_props);
    qtest_add_func("ingenic/lzma-uimage/bad-first-byte", test_bad_first_byte);
    qtest_add_func("ingenic/lzma-uimage/size-mismatch", test_size_mismatch);
    return g_test_run();
}
