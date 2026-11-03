/*
 * QTest testcase for the Ingenic AIP (T41)
 *
 * The T and F units are programmed directly over a 64x16 NV12 picture of
 * known values, as a T41NQ was, and each output's CRC32 is that T41NQ's:
 * T's colour conversion in two steps of eight lines with Ingenic's BT.601
 * parameters, to RGBA and to alpha-first BGR, and F's copy at 1:1 and at
 * half size. F's source stride (128) and destination stride (64) differ,
 * so its lines past the eighth read beyond the picture: those are not
 * checked. F's scaling is checked again over a 64x32 picture of
 * pseudo-random bytes, down and up, with TRANS as libaip sets it and
 * negative, and P's warps over the same picture: a rotation, a
 * perspective and a scaled shear running off the picture, with the colour
 * left as sampled (B = Y, G = U, R = V), and the rotation again with the
 * registers libaip set for BT.601 to alpha-first RGB.
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
#define AIP_P           (AIP + 0x300)
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

/* CRC32 of rows lines of len bytes, stride bytes apart */
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

static void lcg_picture(QTestState *qts)
{
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
}

/*
 * F over a 64x32 picture from the LCG the T41NQ's probe used: source
 * stride 64, destination stride 128.
 */
static void test_f_scaled(void)
{
    static const struct {
        uint32_t dw, dh, sx, sy, tx, ty, crc_y, crc_uv;
    } cases[] = {
        { 48, 24, 0x15555, 0x15555, 0x2aaa, 0x2aaa, 0x18e56348, 0x3fd65fce },
        { 37, 19, 0x1bacf, 0x1af28, 0x5d67, 0x5794, 0x9493fd22, 0x8b52102e },
        { 100, 50, 0xa3d7, 0xa3d7, 0xffffd1ec, 0xffffd1ec, 0xbe50c941,
          0x8cee1c54 },
        { 48, 24, 0x15555, 0x15555, 0xffffc000, 0xffff8000, 0x8a6be764,
          0x01066eea },
    };
    QTestState *qts = qtest_init("-machine ingenic-t41");

    lcg_picture(qts);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        qtest_writel(qts, AIP_F + CTRL, 2);
        qtest_writel(qts, AIP_F + IQR, 0xffffffff);
        qtest_writel(qts, AIP_F + CFG, 0x394);
        qtest_writel(qts, AIP_F + 0x10, cases[i].sx);
        qtest_writel(qts, AIP_F + 0x14, cases[i].sy);
        qtest_writel(qts, AIP_F + 0x18, cases[i].tx);
        qtest_writel(qts, AIP_F + 0x1c, cases[i].ty);
        qtest_writel(qts, AIP_F + 0x20, SRC_Y);
        qtest_writel(qts, AIP_F + 0x24, SRC_UV);
        qtest_writel(qts, AIP_F + 0x28, DST);
        qtest_writel(qts, AIP_F + 0x2c, DST_UV);
        qtest_writel(qts, AIP_F + 0x30, 32 << 16 | 64);
        qtest_writel(qts, AIP_F + 0x34, cases[i].dh << 16 | cases[i].dw);
        qtest_writel(qts, AIP_F + 0x38, 128 << 16 | 64);
        qtest_writel(qts, AIP_F + CTRL, 1);
        g_assert_cmphex(qtest_readl(qts, AIP_F + IQR) & 1, ==, 1);
        g_assert_cmphex(crc_rows(qts, DST, cases[i].dh, 128, cases[i].dw),
                        ==, cases[i].crc_y);
        g_assert_cmphex(crc_rows(qts, DST_UV, cases[i].dh / 2, 128,
                                 cases[i].dw & ~1), ==, cases[i].crc_uv);
    }
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
    static const struct {
        int64_t c[9];
        uint32_t dw, dh, dummy, crc;
    } cases[] = {
        { { 57513, 31420, -993167, -31420, 57513, 915959, 0, 0, 65536 },
          48, 32, 0x11, 0x6c4af574 },
        { { 67446096, -6744610, 0, -3372305, 67446096, 0, -124775, -188849,
            67108864 }, 48, 32, 0x11, 0x185276a7 },
        { { 98304, 6000, -40000, -5000, 70000, 30000, 0, 0, 65536 },
          64, 32, 0xa5, 0x75fac8ab },
    };
    QTestState *qts = qtest_init("-machine ingenic-t41");

    lcg_picture(qts);
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        qtest_writel(qts, AIP_P + CTRL, 2);
        qtest_writel(qts, AIP_P + IQR, 0xffffffff);
        qtest_writel(qts, AIP_P + CFG, 0x14);
        qtest_writel(qts, AIP_P + 0x10, 2);
        qtest_writel(qts, AIP_P + 0x14, SRC_Y);
        qtest_writel(qts, AIP_P + 0x18, SRC_UV);
        qtest_writel(qts, AIP_P + 0x1c, 64);
        qtest_writel(qts, AIP_P + 0x20, DST);
        qtest_writel(qts, AIP_P + 0x24, cases[i].dw * 4);
        qtest_writel(qts, AIP_P + 0x28, cases[i].dh << 16 | cases[i].dw);
        qtest_writel(qts, AIP_P + 0x2c, 32 << 16 | 64);
        qtest_writel(qts, AIP_P + 0x30, cases[i].dummy);
        p_coefs(qts, cases[i].c);
        for (int k = 0; k < 9; k++) {
            qtest_writel(qts, AIP_P + 0x70 + 4 * k, k % 4 ? 0 : 1 << 20);
        }
        qtest_writel(qts, AIP_P + 0x94, 0x00770000);
        qtest_writel(qts, AIP_P + CTRL, 1);
        g_assert_cmphex(qtest_readl(qts, AIP_P + IQR) & 1, ==, 1);
        g_assert_cmphex(crc(qts, DST, cases[i].dw * 4 * cases[i].dh), ==,
                        cases[i].crc);
    }
    qtest_quit(qts);
}

/* the rotation with what libaip programs for BT.601 to alpha-first RGB */
static void test_p_colour(void)
{
    static const int64_t rot[9] = {
        57513, 31420, -993167, -31420, 57513, 915959, 0, 0, 65536,
    };
    static const uint32_t param[9] = {
        0x00129fbe, 0x002049ba, 0, 0x00129fbe, 0x00064189, 0x000d020c,
        0x00129fbe, 0, 0x00198937,
    };
    QTestState *qts = qtest_init("-machine ingenic-t41");

    lcg_picture(qts);
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
        qtest_writel(qts, AIP_P + 0x70 + 4 * k, param[k]);
    }
    qtest_writel(qts, AIP_P + 0x94, 0x0d13c010);
    qtest_writel(qts, AIP_P + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_P + IQR) & 1, ==, 1);
    g_assert_cmphex(crc(qts, DST, 48 * 4 * 32), ==, 0x4fa18087);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic/aip/t-rgba", test_t_rgba);
    qtest_add_func("ingenic/aip/t-abgr", test_t_abgr);
    qtest_add_func("ingenic/aip/f-copy", test_f_copy);
    qtest_add_func("ingenic/aip/f-half", test_f_half);
    qtest_add_func("ingenic/aip/f-scaled", test_f_scaled);
    qtest_add_func("ingenic/aip/p-warp", test_p_warp);
    qtest_add_func("ingenic/aip/p-colour", test_p_colour);
    return g_test_run();
}
