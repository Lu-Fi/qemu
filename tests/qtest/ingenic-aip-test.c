/*
 * QTest testcase for the Ingenic AIP (T41)
 *
 * The T and F units are programmed directly over a 64x16 NV12 picture of
 * known values, as a T41NQ was, and each output's CRC32 is that T41NQ's:
 * T's colour conversion in two steps of eight lines with Ingenic's BT.601
 * parameters, to RGBA and to alpha-first BGR, and F's copy at 1:1 and at
 * half size. F's source stride (128) and destination stride (64) differ,
 * so its lines past the eighth read beyond the picture: those are not
 * checked.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define AIP             0x12b00000
#define AIP_T           (AIP + 0x000)
#define AIP_F           (AIP + 0x200)
#define CTRL            0x00
#define IQR             0x04
#define CFG             0x08

#define SRC_Y           0x00800000
#define SRC_UV          0x00801000
#define DST             0x00810000
#define DST_UV          0x00820000
#define W               64
#define H               16
#define T_STRIDE        (W * 4 + 64)

static const uint32_t bt601[9] = {
    0x00129fbe, 0x002049ba, 0, 0x00129fbe, 0xfff9be77, 0xfff2fdf4,
    0x00129fbe, 0, 0x00198937,
};

static QTestState *setup(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    uint8_t y[W * H], uv[W * H];
    g_autofree uint8_t *fill = g_malloc(0x20000);

    for (int r = 0; r < H; r++) {
        for (int x = 0; x < W; x++) {
            y[r * W + x] = x * 7 + r * 13;
            uv[r * W + x] = x * 5 + r * 11 + 0x40;
        }
    }
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_UV, uv, sizeof(uv));
    memset(fill, 0x5a, 0x20000);
    qtest_memwrite(qts, DST, fill, 0x20000);
    return qts;
}

static uint32_t crc(QTestState *qts, uint64_t addr, size_t len)
{
    g_autofree uint8_t *buf = g_malloc(len);
    uint32_t crc = ~0u;

    qtest_memread(qts, addr, buf, len);
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int k = 0; k < 8; k++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static void t_run(QTestState *qts, uint32_t offset)
{
    qtest_writel(qts, AIP_T + CFG, 0x43c);
    qtest_writel(qts, AIP_T + 0x10, 8);
    qtest_writel(qts, AIP_T + 0x14, SRC_Y);
    qtest_writel(qts, AIP_T + 0x18, SRC_UV);
    qtest_writel(qts, AIP_T + 0x20, T_STRIDE << 16 | W);
    qtest_writel(qts, AIP_T + 0x24, H << 16 | W);
    for (int i = 0; i < 9; i++) {
        qtest_writel(qts, AIP_T + 0x28 + 4 * i, bt601[i]);
    }
    qtest_writel(qts, AIP_T + 0x4c, offset);
    qtest_writel(qts, AIP_T + CTRL, 1);
    for (int step = 0; step < 2; step++) {
        qtest_writel(qts, AIP_T + 0x1c, DST + step * 8 * T_STRIDE);
        qtest_writel(qts, AIP_T + CTRL, 0x10);
        g_assert_cmphex(qtest_readl(qts, AIP_T + IQR) & 1, ==, 1);
        qtest_writel(qts, AIP_T + IQR, 1);
    }
}

static void test_t_rgba(void)
{
    QTestState *qts = setup();

    t_run(qts, 0x05008010);
    g_assert_cmphex(crc(qts, DST, H * T_STRIDE), ==, 0x97e35a9c);
    qtest_quit(qts);
}

static void test_t_abgr(void)
{
    QTestState *qts = setup();

    t_run(qts, 0x08ff8010);
    g_assert_cmphex(crc(qts, DST, H * T_STRIDE), ==, 0xc7370d2d);
    qtest_quit(qts);
}

static void f_run(QTestState *qts, uint32_t dw, uint32_t dh, uint32_t scale)
{
    qtest_writel(qts, AIP_F + CFG, 0x394);
    qtest_writel(qts, AIP_F + 0x10, scale);
    qtest_writel(qts, AIP_F + 0x14, scale);
    qtest_writel(qts, AIP_F + 0x18, 0x10);
    qtest_writel(qts, AIP_F + 0x1c, 0x10);
    qtest_writel(qts, AIP_F + 0x20, SRC_Y);
    qtest_writel(qts, AIP_F + 0x24, SRC_UV);
    qtest_writel(qts, AIP_F + 0x28, DST);
    qtest_writel(qts, AIP_F + 0x2c, DST_UV);
    qtest_writel(qts, AIP_F + 0x30, H << 16 | W);
    qtest_writel(qts, AIP_F + 0x34, dh << 16 | dw);
    qtest_writel(qts, AIP_F + 0x38, W << 16 | 128);
    qtest_writel(qts, AIP_F + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_F + IQR) & 1, ==, 1);
}

static void test_f_copy(void)
{
    QTestState *qts = setup();

    f_run(qts, W, H, 0x10000);
    g_assert_cmphex(crc(qts, DST, 8 * W), ==, 0xc6dc2cee);
    g_assert_cmphex(crc(qts, DST_UV, 4 * W), ==, 0x333817de);
    qtest_quit(qts);
}

static void test_f_half(void)
{
    QTestState *qts = setup();

    f_run(qts, W / 2, H / 2, 0x20000);
    g_assert_cmphex(crc(qts, DST, 4 * W), ==, 0x99045a9b);
    g_assert_cmphex(crc(qts, DST_UV, 2 * W), ==, 0x24d4dcf4);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic/aip/t-rgba", test_t_rgba);
    qtest_add_func("ingenic/aip/t-abgr", test_t_abgr);
    qtest_add_func("ingenic/aip/f-copy", test_f_copy);
    qtest_add_func("ingenic/aip/f-half", test_f_half);
    return g_test_run();
}
