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
#include "qemu/bswap.h"
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

/*
 * The formats, sums and status: a T41NQ's outputs over 256 KiB of
 * pseudo-random bytes at B_SRC and the same bytes xor 0x55 at B_SRC_C,
 * destinations filled with 0x5a first
 */
#define B_SRC           0x00a00000
#define B_SRC_C         0x00a40000
#define B_DST           0x00a80000
#define B_DST_C         0x00ac0000
#define B_CHAIN         0x00af0000

static QTestState *setup_bytes(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    g_autofree uint8_t *a = g_malloc(0x40000);
    g_autofree uint8_t *b = g_malloc(0x40000);
    uint32_t seed = 12345;

    for (int i = 0; i < 0x40000; i++) {
        seed = seed * 1103515245 + 12345;
        a[i] = seed >> 16;
        b[i] = a[i] ^ 0x55;
    }
    qtest_memwrite(qts, B_SRC, a, 0x40000);
    qtest_memwrite(qts, B_SRC_C, b, 0x40000);
    memset(a, 0x5a, 0x40000);
    qtest_memwrite(qts, B_DST, a, 0x40000);
    qtest_memwrite(qts, B_DST_C, a, 0x40000);
    return qts;
}

typedef struct {
    uint32_t fmt, sw, sh, ss, dw, dh, ds;
    uint32_t sx, sy, tx, ty;
    uint32_t isum, osum, crc;   /* isum 0: not checked */
} FCase;

/* bytes a pixel of F's formats */
static const uint32_t f_bpp[8] = { 1, 2, 4, 8, 16, 32, 16, 1 };

static void f_case(QTestState *qts, const FCase *c)
{
    uint32_t bpp = f_bpp[c->fmt];

    qtest_writel(qts, AIP_F + CTRL, 2);
    qtest_writel(qts, AIP_F + IQR, 0xffffffff);
    qtest_writel(qts, AIP_F + CFG, c->fmt << 7 | 0x14 | 0xc00);
    qtest_writel(qts, AIP_F + 0x0c, 0xffffffff);
    qtest_writel(qts, AIP_F + 0x10, c->sx);
    qtest_writel(qts, AIP_F + 0x14, c->sy);
    qtest_writel(qts, AIP_F + 0x18, c->tx);
    qtest_writel(qts, AIP_F + 0x1c, c->ty);
    qtest_writel(qts, AIP_F + 0x20, B_SRC);
    qtest_writel(qts, AIP_F + 0x24, B_SRC_C);
    qtest_writel(qts, AIP_F + 0x28, B_DST);
    qtest_writel(qts, AIP_F + 0x2c, B_DST_C);
    qtest_writel(qts, AIP_F + 0x30, c->sh << 16 | c->sw);
    qtest_writel(qts, AIP_F + 0x34, c->dh << 16 | c->dw);
    qtest_writel(qts, AIP_F + 0x38, c->ds << 16 | c->ss);
    qtest_writel(qts, AIP_F + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_F + CTRL), ==, 0x80000008);
    if (c->fmt == 7) {
        uint32_t crc32 = crc_rows(qts, B_DST, c->dh, c->ds, c->dw);
        g_autofree uint8_t *b = g_malloc(c->dw);

        /* the chroma rows go on the same CRC */
        crc32 = ~crc32;
        for (int r = 0; r < (c->dh + 1) / 2; r++) {
            qtest_memread(qts, B_DST_C + r * c->ds, b, c->dw);
            for (int i = 0; i < c->dw; i++) {
                crc32 ^= b[i];
                for (int k = 0; k < 8; k++) {
                    crc32 = crc32 >> 1 ^ (crc32 & 1 ? 0xedb88320 : 0);
                }
            }
        }
        g_assert_cmphex(~crc32, ==, c->crc);
    } else {
        g_assert_cmphex(crc_rows(qts, B_DST, c->dh, c->ds, c->dw * bpp), ==,
                        c->crc);
    }
    if (c->isum) {
        g_assert_cmphex(qtest_readl(qts, AIP_F + 0x44), ==, c->isum);
    }
    if (c->osum) {
        g_assert_cmphex(qtest_readl(qts, AIP_F + 0x48), ==, c->osum);
    }
}

/* F's formats from 16x8: UV pairs, BGRA, 2-, 4- and 8-bit features, 16 B */
static void test_f_formats(void)
{
    static const FCase cases[] = {
        { 1, 16, 8, 32, 12, 6, 24, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0, 0, 0x37692ac0 },
        { 1, 16, 8, 32, 24, 12, 48, 0xaaaa, 0xaaaa, 0xffffd555, 0xffffd555,
          0, 0, 0x94957c94 },
        { 2, 16, 8, 64, 12, 6, 48, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0, 0, 0xe05c9551 },
        { 2, 16, 8, 64, 24, 12, 96, 0xaaaa, 0xaaaa, 0xffffd555, 0xffffd555,
          0, 0, 0x2e3fb83a },
        { 3, 16, 8, 128, 12, 6, 96, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0, 0, 0x0a931a49 },
        { 3, 16, 8, 128, 24, 12, 192, 0xaaaa, 0xaaaa, 0xffffd555, 0xffffd555,
          0, 0, 0xc71a4945 },
        { 4, 16, 8, 256, 12, 6, 192, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0, 0, 0xc50480bd },
        { 4, 16, 8, 256, 24, 12, 384, 0xaaaa, 0xaaaa, 0xffffd555, 0xffffd555,
          0, 0, 0x5bcefd65 },
        { 5, 16, 8, 512, 12, 6, 384, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0, 0, 0x78d9a0ce },
        { 5, 16, 8, 512, 24, 12, 768, 0xaaaa, 0xaaaa, 0xffffd555, 0xffffd555,
          0, 0, 0x27e68c95 },
        { 6, 16, 8, 256, 12, 6, 192, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0, 0, 0x4c5a3788 },
        { 6, 16, 8, 256, 24, 12, 384, 0xaaaa, 0xaaaa, 0xffffd555, 0xffffd555,
          0, 0, 0xd48a3fe5 },
    };
    QTestState *qts = setup_bytes();

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        f_case(qts, &cases[i]);
    }
    qtest_quit(qts);
}

/*
 * F's sums from cleared, and odd NV12 sizes, which keep their last chroma
 * half pair and line; ISUM is not checked where the source rows pass 48
 * bytes (a T41NQ fetches some bursts twice)
 */
static void test_f_sums(void)
{
    static const FCase cases[] = {
        { 2, 16, 4, 64, 16, 2, 64, 0x10000, 0x20000, 0, 0,
          0xf129ddd7, 0xd007a1d7, 0xc19f43a1 },
        { 0, 16, 4, 16, 12, 3, 16, 0x15555, 0x15555, 0x2aaa, 0x2aaa,
          0x4b7be2ed, 0x591a4df7, 0x60997757 },
        { 0, 16, 4, 16, 13, 3, 16, 0x13b13, 0x15555, 0, 0,
          0x4b7be2ed, 0xe79f82fd, 0x59ff5346 },
        { 7, 16, 4, 16, 13, 3, 16, 0x13b13, 0x15555, 0, 0,
          0x3f4f1600, 0x1c267d31, 0xe85b9ad8 },
        { 4, 4, 4, 64, 3, 3, 48, 0x15555, 0x15555, 0, 0,
          0, 0x8b13f883, 0xd956fa35 },
    };
    QTestState *qts = setup_bytes();

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        f_case(qts, &cases[i]);
    }
    qtest_quit(qts);
}

typedef struct {
    uint32_t mode, sw, sh, ss, dw, dh, ds, dummy;
    int64_t c[9];
    uint32_t isum, osum, crc;   /* isum 0: not checked */
} PCase;

#define P_ID    { 65536, 0, 0, 0, 65536, 0, 0, 0, 65536 }
#define P_ROT   { 57513, 31420, -993167, -31420, 57513, 915959, 0, 0, 65536 }
#define P_PERSP { 67446096, -6744610, 0, -3372305, 67446096, 0, -124775, \
                  -188849, 67108864 }

/* P's Y, UV pair and 4-byte modes, passed as they are, and their sums */
static void test_p_modes(void)
{
    static const PCase cases[] = {
        { 0, 32, 16, 128, 24, 12, 128, 0xa5, P_ID, 0, 0, 0x4eaa7583 },
        { 0, 32, 16, 128, 24, 12, 128, 0xa5, P_ROT, 0x00b96753, 0x4439fff0,
          0x51208caf },
        { 0, 32, 16, 128, 24, 12, 128, 0xa5, P_PERSP, 0, 0, 0x7439d512 },
        { 1, 32, 16, 128, 24, 12, 128, 0xa5, P_ID, 0, 0, 0x3a868870 },
        { 1, 32, 16, 128, 24, 12, 128, 0xa5, P_ROT, 0, 0, 0xd5d0b980 },
        { 1, 32, 16, 128, 24, 12, 128, 0xa5, P_PERSP, 0x7bee3d37, 0x90582f71,
          0xa43e0fdb },
        { 3, 32, 16, 128, 24, 12, 128, 0xa5, P_ID, 0, 0, 0x0a76e57b },
        { 3, 32, 16, 128, 24, 12, 128, 0xa5, P_ROT, 0, 0, 0x7c43261f },
        { 3, 32, 16, 128, 24, 12, 128, 0xa5, P_PERSP, 0, 0, 0x94bc2fea },
        { 3, 16, 4, 64, 16, 4, 64, 0xa5, P_ID, 0xf129ddd7, 0xf129ddd7,
          0xd2924c90 },
        { 2, 16, 4, 16, 16, 4, 64, 0x11, P_ID, 0x3f4f1600, 0xd89ba982,
          0xbd21f16f },
    };
    QTestState *qts = setup_bytes();

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        const PCase *c = &cases[i];
        uint32_t bpp = c->mode == 0 ? 1 : c->mode == 1 ? 2 : 4;

        qtest_writel(qts, AIP_P + CTRL, 2);
        qtest_writel(qts, AIP_P + IQR, 0xffffffff);
        qtest_writel(qts, AIP_P + CFG, 0x14 | 0x600);
        qtest_writel(qts, AIP_P + CFG, 0x14);
        qtest_writel(qts, AIP_P + 0x10, c->mode);
        qtest_writel(qts, AIP_P + 0x14, B_SRC);
        qtest_writel(qts, AIP_P + 0x18, B_SRC_C);
        qtest_writel(qts, AIP_P + 0x1c, c->ss);
        qtest_writel(qts, AIP_P + 0x20, B_DST);
        qtest_writel(qts, AIP_P + 0x24, c->ds);
        qtest_writel(qts, AIP_P + 0x28, c->dh << 16 | c->dw);
        qtest_writel(qts, AIP_P + 0x2c, c->sh << 16 | c->sw);
        qtest_writel(qts, AIP_P + 0x30, c->dummy);
        p_coefs(qts, c->c);
        for (int k = 0; k < 9; k++) {
            qtest_writel(qts, AIP_P + 0x70 + 4 * k, k % 4 ? 0 : 1 << 20);
        }
        qtest_writel(qts, AIP_P + 0x94, 0x00770000);
        qtest_writel(qts, AIP_P + CTRL, 1);
        g_assert_cmphex(qtest_readl(qts, AIP_P + CTRL), ==, 8);
        g_assert_cmphex(crc_rows(qts, B_DST, c->dh, c->ds, c->dw * bpp), ==,
                        c->crc);
        if (c->isum) {
            g_assert_cmphex(qtest_readl(qts, AIP_P + 0xa0), ==, c->isum);
            g_assert_cmphex(qtest_readl(qts, AIP_P + 0xa4), ==, c->osum);
        }
    }
    qtest_quit(qts);
}

/*
 * T in two steps of four lines: CTRL 0x9 while the frame is open, 0x8 at
 * its end, OSUM after each step; then a chain, which on a T41NQ converts
 * nothing and stays chain-busy past a soft reset
 */
static void test_t_status(void)
{
    static const uint32_t par[9] = {
        0x129fbe, 0x2049ba, 0, 0x129fbe, 0x64189, 0xd020c, 0x129fbe, 0,
        0x198937,
    };
    static const uint32_t osum[2] = { 0xdb130881, 0xbe24e0ae };
    uint32_t node[18] = {
        0xffffffff, 0x12b0000c, B_SRC, 0x12b00014, B_SRC_C, 0x12b00018,
        B_DST + 0x10000, 0x12b0001c, 8 << 16 | 16, 0x12b00024,
        64 << 16 | 16, 0x12b00020, 8, 0x12b00010, 0, 0, 1, 0x92b00000,
    };
    QTestState *qts = setup_bytes();

    g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, 8);
    g_assert_cmphex(qtest_readl(qts, AIP_F + CTRL), ==, 8);
    g_assert_cmphex(qtest_readl(qts, AIP_P + CTRL), ==, 8);
    qtest_writel(qts, AIP_T + CTRL, 2);
    qtest_writel(qts, AIP_T + CFG, 0x43c | 0x300);
    qtest_writel(qts, AIP_T + CFG, 0x43c);
    for (int i = 0; i < 9; i++) {
        qtest_writel(qts, AIP_T + 0x28 + 4 * i, par[i]);
    }
    qtest_writel(qts, AIP_T + 0x4c, 0x05ab8010);
    qtest_writel(qts, AIP_T + 0x14, B_SRC);
    qtest_writel(qts, AIP_T + 0x18, B_SRC_C);
    qtest_writel(qts, AIP_T + 0x20, 64 << 16 | 16);
    qtest_writel(qts, AIP_T + 0x24, 8 << 16 | 16);
    qtest_writel(qts, AIP_T + 0x10, 4);
    qtest_writel(qts, AIP_T + CTRL, 1);
    g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, 9);
    for (int step = 0; step < 2; step++) {
        qtest_writel(qts, AIP_T + 0x1c, B_DST + step * 4 * 64);
        qtest_writel(qts, AIP_T + CTRL, 0x10);
        g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, step ? 8 : 9);
        g_assert_cmphex(qtest_readl(qts, AIP_T + 0x60), ==, osum[step]);
    }
    g_assert_cmphex(crc_rows(qts, B_DST, 8, 64, 64), ==, 0xcd78a3da);

    for (int i = 0; i < 18; i++) {
        node[i] = cpu_to_le32(node[i]);
    }
    qtest_memwrite(qts, B_CHAIN, node, sizeof(node));
    qtest_writel(qts, AIP_T + 0x50, B_CHAIN);
    qtest_writel(qts, AIP_T + 0x54, sizeof(node));
    qtest_writel(qts, AIP_T + CTRL, 4);
    g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, 0xc);
    g_assert_cmphex(qtest_readl(qts, AIP_T + 0x14), ==, B_SRC);
    g_assert_cmphex(qtest_readb(qts, B_DST + 0x10000), ==, 0x5a);
    qtest_writel(qts, AIP_T + CTRL, 2);
    g_assert_cmphex(qtest_readl(qts, AIP_T + CTRL), ==, 0xc);
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
    qtest_add_func("ingenic/aip/f-formats", test_f_formats);
    qtest_add_func("ingenic/aip/f-sums", test_f_sums);
    qtest_add_func("ingenic/aip/p-modes", test_p_modes);
    qtest_add_func("ingenic/aip/t-status", test_t_status);
    return g_test_run();
}
