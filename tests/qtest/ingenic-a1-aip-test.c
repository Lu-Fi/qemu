/*
 * QTest testcase for the Ingenic AIP on the A1
 *
 * The A1 has the T41's AIP at 0x13090000 (T at +0x000, F at +0x200, P at
 * +0x300), with the interrupts the A1's tree gives it: T 40, F 39, P 38 of
 * the INTC. Each unit runs a job over a 64x16 NV12 picture of known values
 * (T and F) or a 64x32 one of pseudo-random bytes (P), and the outputs are
 * the T41NQ's, as ingenic-aip-test checks them: T's colour conversion in
 * two steps of eight lines, F's copy at half size, and P's rotation.
 * A T step raises status 0x9 (the A1 driver waits for bit 3 or 1). Then each
 * unit's interrupt shows in the INTC's source register while its
 * status holds and goes when it is cleared.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define AIP             0x13090000
#define AIP_T           (AIP + 0x000)
#define AIP_F           (AIP + 0x200)
#define AIP_P           (AIP + 0x300)
#define INTC_ISR1       0x12300020      /* sources 32 to 63 */
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
    QTestState *qts = qtest_init("-machine ingenic-a1");
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

static uint32_t crc_rows(QTestState *qts, uint64_t addr, int rows,
                         size_t stride, size_t len)
{
    g_autofree uint8_t *buf = g_malloc(len);
    uint32_t crc = ~0u;

    for (int r = 0; r < rows; r++) {
        qtest_memread(qts, addr + r * stride, buf, len);
        for (size_t i = 0; i < len; i++) {
            crc ^= buf[i];
            for (int k = 0; k < 8; k++) {
                crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
            }
        }
    }
    return ~crc;
}

static uint32_t crc(QTestState *qts, uint64_t addr, size_t len)
{
    return crc_rows(qts, addr, 1, len, len);
}

static void test_t_rgba(void)
{
    QTestState *qts = setup();

    qtest_writel(qts, AIP_T + CFG, 0x43c);
    qtest_writel(qts, AIP_T + 0x10, 8);
    qtest_writel(qts, AIP_T + 0x14, SRC_Y);
    qtest_writel(qts, AIP_T + 0x18, SRC_UV);
    qtest_writel(qts, AIP_T + 0x20, T_STRIDE << 16 | W);
    qtest_writel(qts, AIP_T + 0x24, H << 16 | W);
    for (int i = 0; i < 9; i++) {
        qtest_writel(qts, AIP_T + 0x28 + 4 * i, bt601[i]);
    }
    qtest_writel(qts, AIP_T + 0x4c, 0x05008010);
    qtest_writel(qts, AIP_T + CTRL, 1);
    for (int step = 0; step < 2; step++) {
        qtest_writel(qts, AIP_T + 0x1c, DST + step * 8 * T_STRIDE);
        qtest_writel(qts, AIP_T + CTRL, 0x10);
        g_assert_cmphex(qtest_readl(qts, AIP_T + IQR), ==, 9);
        g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 8 & 1, ==, 1);
        qtest_writel(qts, AIP_T + IQR, 9);
        g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 8 & 1, ==, 0);
    }
    g_assert_cmphex(crc(qts, DST, H * T_STRIDE), ==, 0x97e35a9c);
    qtest_quit(qts);
}

static void test_f_half(void)
{
    QTestState *qts = setup();

    qtest_writel(qts, AIP_F + CFG, 0x394);
    qtest_writel(qts, AIP_F + 0x10, 0x20000);
    qtest_writel(qts, AIP_F + 0x14, 0x20000);
    qtest_writel(qts, AIP_F + 0x18, 0x10);
    qtest_writel(qts, AIP_F + 0x1c, 0x10);
    qtest_writel(qts, AIP_F + 0x20, SRC_Y);
    qtest_writel(qts, AIP_F + 0x24, SRC_UV);
    qtest_writel(qts, AIP_F + 0x28, DST);
    qtest_writel(qts, AIP_F + 0x2c, DST_UV);
    qtest_writel(qts, AIP_F + 0x30, H << 16 | W);
    qtest_writel(qts, AIP_F + 0x34, H / 2 << 16 | W / 2);
    qtest_writel(qts, AIP_F + 0x38, W << 16 | 128);
    qtest_writel(qts, AIP_F + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_F + IQR) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 7 & 1, ==, 1);
    g_assert_cmphex(crc(qts, DST, 4 * W), ==, 0x99045a9b);
    g_assert_cmphex(crc(qts, DST_UV, 2 * W), ==, 0x24d4dcf4);
    qtest_writel(qts, AIP_F + IQR, 1);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 7 & 1, ==, 0);
    qtest_quit(qts);
}

/* P's matrix: per row 48, 48 and 59 bits, rows 160 bits apart */
static void p_coefs(QTestState *qts, const int64_t *c)
{
    uint32_t w[15] = { 0 };
    static const int lo[3] = { 0, 48, 96 }, n[3] = { 48, 48, 59 };

    for (int k = 0; k < 9; k++) {
        for (int b = 0; b < n[k % 3]; b++) {
            int bit = 160 * (k / 3) + lo[k % 3] + b;

            if ((c[k] >> b) & 1) {
                w[bit / 32] |= 1u << (bit % 32);
            }
        }
    }
    for (int i = 0; i < 15; i++) {
        qtest_writel(qts, AIP_P + 0x34 + 4 * i, w[i]);
    }
}

static void test_p_warp(void)
{
    static const int64_t rot[9] = {
        57513, 31420, -993167, -31420, 57513, 915959, 0, 0, 65536,
    };
    QTestState *qts = qtest_init("-machine ingenic-a1");
    uint8_t y[64 * 32], uv[64 * 16];
    uint32_t seed = 12345;

    for (int i = 0; i < 64 * 32; i++) {
        seed = seed * 1103515245 + 12345;
        y[i] = seed >> 16;
        if (i < 64 * 16) {
            uv[i] = seed >> 24;
        }
    }
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_UV, uv, sizeof(uv));
    qtest_writel(qts, AIP_P + CFG, 0x14);
    qtest_writel(qts, AIP_P + 0x10, 2);
    qtest_writel(qts, AIP_P + 0x14, SRC_Y);
    qtest_writel(qts, AIP_P + 0x18, SRC_UV);
    qtest_writel(qts, AIP_P + 0x1c, 64);
    qtest_writel(qts, AIP_P + 0x20, DST);
    qtest_writel(qts, AIP_P + 0x24, 48 * 4);
    qtest_writel(qts, AIP_P + 0x28, 32 << 16 | 48);
    qtest_writel(qts, AIP_P + 0x2c, 32 << 16 | 64);
    qtest_writel(qts, AIP_P + 0x30, 0x11);
    p_coefs(qts, rot);
    for (int k = 0; k < 9; k++) {
        qtest_writel(qts, AIP_P + 0x70 + 4 * k, k % 4 ? 0 : 1 << 20);
    }
    qtest_writel(qts, AIP_P + 0x94, 0x00770000);
    qtest_writel(qts, AIP_P + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_P + IQR) & 1, ==, 1);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 6 & 1, ==, 1);
    g_assert_cmphex(crc(qts, DST, 48 * 4 * 32), ==, 0x6c4af574);
    qtest_writel(qts, AIP_P + IQR, 1);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 6 & 1, ==, 0);
    qtest_quit(qts);
}

/* Reset values, T's CFG without bit 10, and F's TIMEOUT run down by a job */
static void test_a1n_regs(void)
{
    QTestState *qts = setup();
    static const uint32_t unit[] = { AIP_T, AIP_F, AIP_P };

    for (int i = 0; i < 3; i++) {
        g_assert_cmphex(qtest_readl(qts, unit[i] + CTRL), ==, 8);
        g_assert_cmphex(qtest_readl(qts, unit[i] + IQR), ==, 0);
        g_assert_cmphex(qtest_readl(qts, unit[i] + CFG), ==, 0);
        g_assert_cmphex(qtest_readl(qts, unit[i] + 0x0c), ==, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, unit[i] + 0x10), ==,
                        i ? 0 : 0x00020002);
        g_assert_cmphex(qtest_readl(qts, unit[i] + 0x14), ==, 0);
    }
    qtest_writel(qts, AIP_T + CFG, 0x43c);
    g_assert_cmphex(qtest_readl(qts, AIP_T + CFG), ==, 0x3c);

    /* F: a 2x2 NV12 copy with TIMEOUT 0xf: done, CTRL bit 31, 0xd4 cycles */
    qtest_writel(qts, AIP_F + 0x0c, 0xf);
    qtest_writel(qts, AIP_F + CFG, 0x380);
    qtest_writel(qts, AIP_F + 0x10, 0x10000);
    qtest_writel(qts, AIP_F + 0x14, 0x10000);
    qtest_writel(qts, AIP_F + 0x20, SRC_Y);
    qtest_writel(qts, AIP_F + 0x24, SRC_UV);
    qtest_writel(qts, AIP_F + 0x28, DST);
    qtest_writel(qts, AIP_F + 0x2c, DST_UV);
    qtest_writel(qts, AIP_F + 0x30, 2 << 16 | 2);
    qtest_writel(qts, AIP_F + 0x34, 2 << 16 | 2);
    qtest_writel(qts, AIP_F + 0x38, W << 16 | W);
    qtest_writel(qts, AIP_F + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_F + IQR), ==, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_F + CTRL), ==, 0x80000008);
    g_assert_cmphex(qtest_readl(qts, AIP_F + 0x0c), ==, 0xffffff3b);
    qtest_quit(qts);
}

/* A T step with no job loaded leaves CTRL 0x9 and IQR 0x8, nothing written */
static void test_t_empty(void)
{
    QTestState *qts = setup();
    uint8_t fill[64];

    qtest_writel(qts, AIP_T + CTRL, 1);
    qtest_writel(qts, AIP_T + 0x1c, DST);
    qtest_writel(qts, AIP_T + CTRL, 0x10);
    g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, 9);
    g_assert_cmphex(qtest_readl(qts, AIP_T + IQR), ==, 8);
    qtest_memread(qts, DST, fill, sizeof(fill));
    for (unsigned i = 0; i < sizeof(fill); i++) {
        g_assert_cmphex(fill[i], ==, 0x5a);
    }
    qtest_writel(qts, AIP_T + CTRL, 2);
    g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, 8);
    g_assert_cmphex(qtest_readl(qts, AIP_T + IQR), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic/a1-aip/t-rgba", test_t_rgba);
    qtest_add_func("ingenic/a1-aip/f-half", test_f_half);
    qtest_add_func("ingenic/a1-aip/p-warp", test_p_warp);
    qtest_add_func("ingenic/a1-aip/a1n-regs", test_a1n_regs);
    qtest_add_func("ingenic/a1-aip/t-empty", test_t_empty);
    return g_test_run();
}
