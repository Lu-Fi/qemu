/*
 * QTest testcase for the Ingenic BSCALER (T40)
 *
 * FRMC jobs as libvenus chains them, over a BGRA picture of LCG bytes,
 * each output's CRC32 a T40XP's for the same chain: a 2x upscale, a 1.5x
 * downscale with libvenus's half-pixel offset, fractional and uneven
 * scales, a rotation running off the source, and the rotation again with
 * source and destination tile origins. Then the other formats: NV12 to
 * NV12 (chroma sampled at each pair's odd pixel), with the pads following
 * 0xe4's offsets, NV12 to RGBA and ARGB (converted first), 4-byte pixels
 * to NV12, 4-byte pixels with an alpha byte, and NV12 to packed YUV.
 * FRMT frames over an NV12 picture of LCG bytes, a few lines a step:
 * RGBA as libvenus converts it, other byte orders, offsets and
 * coefficients, YUV out, an NV12 copy whose UV plane restarts at 0x9c
 * each step, and an odd size, whose last column and line stay unwritten.
 * Then what the runs leave in the status registers, the interrupts, and
 * the resets.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define BS              0x13090000
#define SRC             0x00800000
#define SRC_UV          0x00880000
#define DST             0x00900000
#define DST_UV          0x00980000
#define CHAIN           0x00a00000
#define FRMC_IRQ        39
#define FRMT_IRQ        40

typedef struct {
    const char *name;
    uint32_t sw, sh, dw, dh;
    uint32_t m[9];
    uint32_t o40, o44;
    uint32_t crc;
} Case;

static const Case cases[] = {
    { "up2", 32, 16, 64, 32,
      { 0x8000, 0, 0, 0, 0x8000, 0, 0, 0, 0x10000 }, 0, 0, 0x32dc3749 },
    { "down15", 48, 24, 32, 16,
      { 0x18000, 0, 0x4000, 0, 0x18000, 0x4000, 0, 0, 0x10000 }, 0, 0,
      0xbf3b78bb },
    { "frac", 32, 16, 24, 12,
      { 0x10000, 0, 0x4000, 0, 0x10000, 0x8000, 0, 0, 0x10000 }, 0, 0,
      0x39e5015e },
    { "uneven", 40, 20, 27, 14,
      { 0x17b42, 0, 0x2f13, 0, 0x1a3c1, 0x1ab7, 0, 0, 0x10000 }, 0, 0,
      0x4ba80c3b },
    { "rotate", 48, 32, 32, 24,
      { 0xe0a9, 0x7abc, 0x18000, 0xffff8544, 0xe0a9, 0x30000, 0, 0, 0x10000 },
      0, 0, 0x1b218131 },
    { "origins", 48, 32, 32, 24,
      { 0xe0a9, 0x7abc, 0x18000, 0xffff8544, 0xe0a9, 0x30000, 0, 0, 0x10000 },
      0x00020003, 0x00020003, 0x4c8c5036 },
};

/* NV12 to BGR, as libvenus loads 0xc0-0xe0: BT.601 in 12.20 */
static const uint32_t bt601[9] = {
    0x129fbe, 0x2049ba, 0, 0x129fbe, 0x64189, 0xd020c, 0x129fbe, 0, 0x198937,
};

typedef struct {
    const char *name;
    uint32_t mode, param;
    uint32_t sw, sh, dw, dh;
    uint32_t m[9];
    uint32_t ds;                /* 0: the format's own */
    uint32_t rows;              /* of output in the CRC, 0: all */
    uint32_t crc;
} FmtCase;

#define M_DOWN15    { 0x18000, 0, 0x4000, 0, 0x18000, 0x4000, 0, 0, 0x10000 }
#define M_ROTATE    { 0xe0a9, 0x7abc, 0x18000, 0xffff8544, 0xe0a9, 0x30000, \
                      0, 0, 0x10000 }
#define M_ID        { 0x10000, 0, 0, 0, 0x10000, 0, 0, 0, 0x10000 }

static const FmtCase fmt_cases[] = {
    { "nv12", 0x6, 0x80100050, 48, 24, 32, 16, M_DOWN15, 0, 0, 0xea2a0cdc },
    { "nv12-pads", 0x6, 0x40200050, 48, 32, 32, 24, M_ROTATE, 0, 0,
      0xcb133417 },
    { "nv12-rgba", 0xe, 0x80100050, 48, 32, 32, 24, M_ROTATE, 0, 0,
      0x74544fbb },
    { "nv12-argb", 0xe, 0x801000d0, 16, 8, 16, 8, M_ID, 0, 0, 0xda92d191 },
    { "bgra-nv12", 0x7, 0x40200050, 48, 32, 32, 24, M_ROTATE, 0, 0,
      0xcbe972c2 },
    { "bgra-alpha", 0xf, 0x8010ab50, 48, 32, 32, 24, M_ROTATE, 0, 0,
      0x15ba2dc2 },
    { "nv12-yuv", 0x4, 0x8010ab80, 48, 24, 32, 16, M_DOWN15, 0x80, 4,
      0xc7013d44 },
};

typedef struct {
    const char *name;
    uint32_t w, h, param, format, ds;
    const uint32_t *coef;       /* NULL: BT.601 */
    uint8_t steps[4];           /* lines a step, then the rest */
    uint32_t uv_rows;           /* of an NV12 copy's UV plane in the CRC */
    uint32_t crc;
} FrmtCase;

static const uint32_t coef_g_v[9] = { 0, 0, 0, 0, 0, 0x100000, 0, 0, 0 };
static const uint32_t coef_r_v16[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0x1000000 };

static const FrmtCase frmt_cases[] = {
    { "rgba", 64, 16, 0x80100005, 0x4, 256, NULL, { 6, 4, 4, 2 }, 0,
      0x9f56d203 },
    { "rbga-alpha-first", 64, 16, 0x8010000a, 0x4, 256, NULL, { 0 }, 0,
      0x21683f11 },
    { "offsets", 64, 16, 0x40200005, 0x4, 256, NULL, { 0 }, 0, 0xa8fc50aa },
    { "g-minus-v", 64, 16, 0x80100005, 0x4, 256, coef_g_v, { 0 }, 0,
      0x2739cdfe },
    { "r-16v", 64, 16, 0x80100005, 0x4, 256, coef_r_v16, { 0 }, 0,
      0x328c406b },
    { "yuv", 64, 16, 0x8010ab08, 0x8, 256, NULL, { 0 }, 0, 0x5d6b3a40 },
    { "nv12", 64, 16, 0x80100005, 0x0, 256, NULL, { 4, 4, 8 }, 4,
      0x59e2053a },
    { "odd", 17, 5, 0x80100005, 0x4, 68, NULL, { 0 }, 0, 0x0ff2560d },
};

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static void lcg_fill(QTestState *qts, uint64_t addr, size_t n, uint32_t *seed)
{
    g_autofree uint8_t *b = g_malloc(n);

    for (size_t i = 0; i < n; i++) {
        *seed = *seed * 1103515245 + 12345;
        b[i] = *seed >> 16;
    }
    qtest_memwrite(qts, addr, b, n);
}

static void crc_add(QTestState *qts, uint64_t addr, size_t n, uint32_t *crc)
{
    g_autofree uint8_t *b = g_malloc(n);
    uint32_t c = ~*crc;

    qtest_memread(qts, addr, b, n);
    for (size_t i = 0; i < n; i++) {
        c ^= b[i];
        for (int k = 0; k < 8; k++) {
            c = c >> 1 ^ (c & 1 ? 0xedb88320 : 0);
        }
    }
    *crc = ~c;
}

static void put(uint32_t *v, uint32_t *n, uint32_t val, uint32_t off)
{
    v[(*n)++] = cpu_to_le32(val);
    v[(*n)++] = cpu_to_le32(off);
}

/* the job's chain, in the order libvenus writes it, ending in 0x201 at 0 */
static uint32_t put_chain(QTestState *qts, const Case *c)
{
    uint32_t v[64], n = 0;

    put(v, &n, SRC, 0x08);
    put(v, &n, DST, 0x18);
    put(v, &n, 0xf, 0x04);
    put(v, &n, c->sh << 16 | c->sw, 0x10);
    put(v, &n, c->sw * 4, 0x14);
    put(v, &n, c->dh << 16 | c->dw, 0x1c);
    put(v, &n, c->dw * 4, 0x20);
    put(v, &n, c->o40, 0x40);
    put(v, &n, c->o44, 0x44);
    for (int k = 0; k < 9; k++) {
        put(v, &n, c->m[k], 0x48 + 4 * k);
    }
    put(v, &n, 0x201, 0x80000000);
    qtest_memwrite(qts, CHAIN, v, n * 4);
    return n * 4;
}

static void test_case(const void *data)
{
    const Case *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t40");
    size_t sz = c->sw * c->sh * 4, out = c->dw * c->dh * 4;
    g_autofree uint8_t *src = g_malloc(sz);
    g_autofree uint8_t *dst = g_malloc(out);
    uint32_t seed = 777, len;

    for (size_t i = 0; i < sz; i++) {
        seed = seed * 1103515245 + 12345;
        src[i] = seed >> 16;
    }
    qtest_memwrite(qts, SRC, src, sz);
    len = put_chain(qts, c);
    qtest_writel(qts, BS + 0x30, 0x10000);
    qtest_writel(qts, BS + 0x3c, 0xffffffff);
    qtest_writel(qts, BS + 0x34, CHAIN);
    qtest_writel(qts, BS + 0x38, len);
    qtest_writel(qts, BS + 0x30, 0x21);
    qtest_memread(qts, DST, dst, out);
    g_assert_cmphex(crc32_le(dst, out), ==, c->crc);
    qtest_quit(qts);
}

static void bs_params(QTestState *qts, uint32_t param, const uint32_t *coef)
{
    for (int k = 0; k < 9; k++) {
        qtest_writel(qts, BS + 0xc0 + 4 * k, coef ? coef[k] : bt601[k]);
    }
    qtest_writel(qts, BS + 0xe4, param);
}

static void test_fmt(const void *data)
{
    const FmtCase *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t40");
    bool s4 = c->mode & 1, d4 = c->mode & 8, nv12 = !d4 && (c->mode & 2);
    uint32_t ss = s4 ? c->sw * 4 : c->sw;
    uint32_t ds = c->ds ? c->ds : d4 ? c->dw * 4 : c->dw;
    uint32_t seed = 777, v[64], n = 0, crc = 0;
    g_autofree uint8_t *fill = g_malloc0(0x40000);

    lcg_fill(qts, SRC, ss * c->sh, &seed);
    if (!s4) {
        lcg_fill(qts, SRC_UV, c->sw * ((c->sh + 1) / 2), &seed);
    }
    memset(fill, 0x5a, 0x40000);
    qtest_memwrite(qts, DST, fill, 0x40000);
    qtest_memwrite(qts, DST_UV, fill, 0x40000);
    put(v, &n, SRC, 0x08);
    put(v, &n, SRC_UV, 0x0c);
    put(v, &n, DST, 0x18);
    if (nv12) {
        put(v, &n, DST_UV, 0x18);
    }
    put(v, &n, c->mode, 0x04);
    put(v, &n, c->sh << 16 | c->sw, 0x10);
    put(v, &n, ss, 0x14);
    put(v, &n, c->dh << 16 | c->dw, 0x1c);
    put(v, &n, ds, 0x20);
    for (int k = 0; k < 9; k++) {
        put(v, &n, c->m[k], 0x48 + 4 * k);
    }
    put(v, &n, 0x201, 0x80000000);
    qtest_memwrite(qts, CHAIN, v, n * 4);
    bs_params(qts, c->param, NULL);
    qtest_writel(qts, BS + 0x34, CHAIN);
    qtest_writel(qts, BS + 0x38, n * 4);
    qtest_writel(qts, BS + 0x30, 0x21);
    crc_add(qts, DST, (size_t)ds * (c->rows ? c->rows : c->dh), &crc);
    if (nv12) {
        crc_add(qts, DST_UV, (size_t)ds * (c->dh / 2), &crc);
    }
    g_assert_cmphex(crc, ==, c->crc);
    qtest_quit(qts);
}

/* one FRMT frame, a step at a time, as libvenus's bs_covert_step_start */
static void frmt_frame(QTestState *qts, const FrmtCase *c)
{
    uint32_t line = 0;

    qtest_writel(qts, BS + 0xb0, 0x10000);
    bs_params(qts, c->param, c->coef);
    qtest_writel(qts, BS + 0x88, SRC);
    qtest_writel(qts, BS + 0x8c, SRC + c->w * c->h);
    qtest_writel(qts, BS + 0x98, DST);
    qtest_writel(qts, BS + 0x9c, DST_UV);
    qtest_writel(qts, BS + 0x90, c->h << 16 | c->w);
    qtest_writel(qts, BS + 0x94, c->w);
    qtest_writel(qts, BS + 0xa4, c->ds);
    qtest_writel(qts, BS + 0xac, c->format);
    qtest_writel(qts, BS + 0x80, 0x201);
    for (int k = 0; line < c->h; k++) {
        uint32_t n = k < 4 && c->steps[k] ? c->steps[k] : c->h - line;

        qtest_writel(qts, BS + 0x98, DST + line * c->ds);
        qtest_writel(qts, BS + 0x84, n << 16 | 1);
        qtest_writel(qts, BS + 0x84, qtest_readl(qts, BS + 0x84) & ~4);
        line += n;
    }
}

static void test_frmt(const void *data)
{
    const FrmtCase *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t40");
    g_autofree uint8_t *fill = g_malloc(0x20000);
    uint32_t seed = 777, crc = 0;

    lcg_fill(qts, SRC, c->w * c->h * 3 / 2, &seed);
    memset(fill, 0x5a, 0x20000);
    qtest_memwrite(qts, DST, fill, 0x20000);
    qtest_memwrite(qts, DST_UV, fill, 0x20000);
    frmt_frame(qts, c);
    crc_add(qts, DST, (size_t)c->ds * c->h, &crc);
    crc_add(qts, DST_UV, (size_t)c->ds * c->uv_rows, &crc);
    g_assert_cmphex(crc, ==, c->crc);
    qtest_quit(qts);
}

/*
 * each FRMT step sets 0x84 bit 2, which raises its interrupt until cleared;
 * 0x80 bit 0 stays set to the frame's last line, 0xb0 bit 17 to its first
 * step
 */
static void test_frmt_status(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    qtest_writel(qts, BS + 0x90, 8 << 16 | 16);
    qtest_writel(qts, BS + 0x94, 16);
    qtest_writel(qts, BS + 0xa4, 64);
    qtest_writel(qts, BS + 0xac, 4);
    qtest_writel(qts, BS + 0x80, 0x201);
    g_assert_cmphex(qtest_readl(qts, BS + 0xb0), ==, 0x20000);
    qtest_writel(qts, BS + 0x84, 4 << 16 | 1);
    g_assert_true(qtest_get_irq(qts, FRMT_IRQ));
    g_assert_false(qtest_get_irq(qts, FRMC_IRQ));
    g_assert_cmphex(qtest_readl(qts, BS + 0x84), ==, 0x40004);
    g_assert_cmphex(qtest_readl(qts, BS + 0xb0), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BS + 0x80) & 1, ==, 1);
    qtest_writel(qts, BS + 0x84, qtest_readl(qts, BS + 0x84) & ~4);
    g_assert_false(qtest_get_irq(qts, FRMT_IRQ));
    qtest_writel(qts, BS + 0x84, 4 << 16 | 1);
    g_assert_true(qtest_get_irq(qts, FRMT_IRQ));
    g_assert_cmphex(qtest_readl(qts, BS + 0x80), ==, 0x200);
    qtest_writel(qts, BS + 0xb0, 0x10000);
    g_assert_false(qtest_get_irq(qts, FRMT_IRQ));
    g_assert_cmphex(qtest_readl(qts, BS + 0xac), ==, 4);
    g_assert_cmphex(qtest_readl(qts, BS + 0xec), ==, 0x20200826);
    qtest_quit(qts);
}

/* a finished chain sets 0x00 bit 8 and 0x30 bit 4; cleared, 0x200 and 0x20 */
static void test_status(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40");
    uint32_t len = put_chain(qts, &cases[0]);

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    qtest_writel(qts, BS + 0x34, CHAIN);
    qtest_writel(qts, BS + 0x38, len);
    qtest_writel(qts, BS + 0x30, 0x21);
    g_assert_true(qtest_get_irq(qts, FRMC_IRQ));
    g_assert_cmphex(qtest_readl(qts, BS + 0x00), ==, 0x300);
    g_assert_cmphex(qtest_readl(qts, BS + 0x30), ==, 0x30);
    qtest_writel(qts, BS + 0x00, qtest_readl(qts, BS + 0x00) & ~0x100);
    qtest_writel(qts, BS + 0x30, qtest_readl(qts, BS + 0x30) & ~0x50);
    g_assert_false(qtest_get_irq(qts, FRMC_IRQ));
    g_assert_cmphex(qtest_readl(qts, BS + 0x00), ==, 0x200);
    g_assert_cmphex(qtest_readl(qts, BS + 0x30), ==, 0x20);
    qtest_writel(qts, BS + 0x30, 0x10000);
    g_assert_cmphex(qtest_readl(qts, BS + 0x30), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BS + 0x00), ==, 0x200);
    /* the reset takes the origins and matrix only */
    g_assert_cmphex(qtest_readl(qts, BS + 0x10), ==, 16 << 16 | 32);
    g_assert_cmphex(qtest_readl(qts, BS + 0x48), ==, 0);
    g_assert_cmphex(qtest_readl(qts, BS + 0x68), ==, 1);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/bscaler/frmc/%s",
                                                cases[i].name);

        qtest_add_data_func(path, &cases[i], test_case);
    }
    for (int i = 0; i < ARRAY_SIZE(fmt_cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/bscaler/frmc/%s",
                                                fmt_cases[i].name);

        qtest_add_data_func(path, &fmt_cases[i], test_fmt);
    }
    for (int i = 0; i < ARRAY_SIZE(frmt_cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic/bscaler/frmt/%s",
                                                frmt_cases[i].name);

        qtest_add_data_func(path, &frmt_cases[i], test_frmt);
    }
    qtest_add_func("ingenic/bscaler/status", test_status);
    qtest_add_func("ingenic/bscaler/frmt-status", test_frmt_status);
    return g_test_run();
}
