/*
 * QTest testcase for the Ingenic A1 IPU
 *
 * Descriptors are built in RAM the way the vendor libimp builds them: four
 * header words, then register writes as a (count << 16 | register) word and
 * its values, ended by a write of 1 to 0x1010. The chain is started through
 * +0x14 and +0x10; its end sets STATUS bit 0 and INTC source 46.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define IPU             0x13080000
#define INTC_ISR1       0x12300020      /* sources 32 to 63 */
#define GO              0x10
#define CHAIN           0x14
#define STATUS          0x20
#define STATUS_CLR      0x24
#define RESET           0x40
#define IIP_RAW         0x1014
#define IIP_CLR         0x1018
#define IIP_MASK        0x101c
#define STATUS_MASK     0x2c
#define RESET2          0x1024
#define REG_FG          0x1060
#define REG_OT          0x10a0

#define DESC            0x00700000
#define SRC             0x00800000
#define SRC_UV          0x00810000
#define DST             0x00900000
#define DST_UV          0x00910000

#define T_ARGB8888      4
#define T_ARGB1555      6
#define T_NV12          8
#define INFO(type, order) (1u << 31 | (order) << 20 | (type) << 16 | 0xff)

typedef struct Desc {
    uint32_t words[512];
    unsigned n;
} Desc;

static void desc_begin(Desc *d, uint32_t op)
{
    d->n = 4;
    d->words[0] = op;
    d->words[1] = 0xffffffff;
    d->words[2] = 0;
    d->words[3] = 0;
}

static void desc_regs(Desc *d, uint32_t reg, unsigned count,
                      const uint32_t *v)
{
    d->words[d->n++] = count << 16 | reg;
    for (unsigned i = 0; i < count; i++) {
        d->words[d->n++] = v[i];
    }
}

static void desc_kick(Desc *d)
{
    uint32_t one = 1;

    desc_regs(d, 0x1010, 1, &one);
}

static void desc_put(Desc *d, QTestState *qts, uint32_t addr)
{
    d->words[0] |= (d->n - 4) << 16;
    qtest_memwrite(qts, addr, d->words, d->n * 4);
}

/* The driver unmasks the interrupt; the reset value of +0x2c is 3 */
static QTestState *ipu_init(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    qtest_writel(qts, IPU + STATUS_MASK, 0);
    return qts;
}

static void ipu_go(QTestState *qts, uint32_t addr)
{
    qtest_writel(qts, IPU + CHAIN, addr);
    qtest_writel(qts, IPU + GO, 1);
}

static void ipu_wait_done(QTestState *qts)
{
    g_assert_cmphex(qtest_readl(qts, IPU + STATUS), ==, 0);
    qtest_clock_step(qts, 100000000);
    g_assert_cmphex(qtest_readl(qts, IPU + STATUS), ==, 1);
}

static void ipu_ack(QTestState *qts)
{
    qtest_writel(qts, IPU + STATUS_CLR, 1);
    g_assert_cmphex(qtest_readl(qts, IPU + STATUS), ==, 0);
}

/* A chain of one job: FG and OT surfaces, no scaler */
static void job_fg_ot(Desc *d, uint32_t op, uint32_t w, uint32_t h,
                      uint32_t src, uint32_t sstride, uint32_t sinfo,
                      uint32_t dst, uint32_t dstride, uint32_t dinfo)
{
    uint32_t fg[7] = { w << 16 | h, src, 0, sstride, sinfo, 0, 0 };
    uint32_t ot[5] = { w << 16 | h, dst, 0, dstride, dinfo };

    desc_begin(d, op);
    desc_regs(d, REG_FG, 7, fg);
    desc_regs(d, REG_OT, 5, ot);
    desc_kick(d);
}

static void test_reset(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    static const struct {
        uint32_t reg, val;
    } rv[] = {
        { 0x00, 0x20210708 }, { 0x2c, 3 }, { 0x80, 0x00140120 },
        { 0x84, 0 }, { 0xfc, 0 }, { 0x1000, 0x20210708 },
        { 0x1040, 0x028001e0 }, { 0x1044, 0 }, { 0x1050, 0xff },
        { 0x1054, 0xff }, { 0x1060, 0x028001e0 }, { 0x1070, 0xff },
        { 0x1074, 0xff }, { 0x1080, 0x028001e0 }, { 0x10a0, 0x028001e0 },
        { 0x10b0, 0xff }, { 0x10d0, 0x80100000 }, { 0x10d4, 0 },
        { 0x10fc, 0 },
    };

    for (unsigned i = 0; i < ARRAY_SIZE(rv); i++) {
        g_assert_cmphex(qtest_readl(qts, IPU + rv[i].reg), ==, rv[i].val);
    }
    qtest_writel(qts, IPU + RESET2, 1);
    qtest_writel(qts, IPU + RESET, 1);
    g_assert_cmphex(qtest_readl(qts, IPU + RESET), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IPU + RESET2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IPU + STATUS), ==, 0);
    g_assert_cmphex(qtest_readl(qts, IPU + IIP_RAW), ==, 0);
    qtest_writel(qts, IPU + IIP_MASK, 0xf);
    g_assert_cmphex(qtest_readl(qts, IPU + IIP_MASK), ==, 0xf);
    qtest_quit(qts);
}

/* The CPM words the IPU's driver sets: IPUMCDR stores, SRBC0's stop acks */
static void test_cpm(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");

    g_assert_cmphex(qtest_readl(qts, 0x100000bc), ==, 0);
    qtest_writel(qts, 0x100000bc, 0x80000001);
    g_assert_cmphex(qtest_readl(qts, 0x100000bc), ==, 0x80000001);
    g_assert_cmphex(qtest_readl(qts, 0x100000f0), ==, 0);
    qtest_writel(qts, 0x100000f0, 0x00200000);
    g_assert_cmphex(qtest_readl(qts, 0x100000f0), ==, 0x00300000);
    qtest_writel(qts, 0x100000f0, 0);
    qtest_writel(qts, 0x100000f0, 0x00400000);
    g_assert_cmphex(qtest_readl(qts, 0x100000f0), ==, 0x00400000);
    qtest_quit(qts);
}

/* Write all ones, read what a register keeps, then zero */
static void test_keep(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    static const struct {
        uint32_t reg, mask;
    } k[] = {
        { 0x14, ~0u }, { 0x2c, ~0u }, { 0x101c, ~0u }, { 0x1040, ~0u },
        { 0x1044, ~0u }, { 0x104c, ~0u }, { 0x1060, ~0u }, { 0x10a0, ~0u },
        { 0x10c0, ~0u }, { 0x10c4, ~0u }, { 0x1050, 0xc2ffffff },
        { 0x10c8, 0x80000001 }, { 0x10d0, 0xffff0003 },
        { 0x10d4, 0x3fff3fff }, { 0x1108, 0x80000fff },
        { 0x1110, 0x01011111 }, { 0x1114, 0x001fffff },
    };

    for (unsigned i = 0; i < ARRAY_SIZE(k); i++) {
        qtest_writel(qts, IPU + k[i].reg, ~0u);
        g_assert_cmphex(qtest_readl(qts, IPU + k[i].reg), ==, k[i].mask);
        qtest_writel(qts, IPU + k[i].reg, 0);
        g_assert_cmphex(qtest_readl(qts, IPU + k[i].reg), ==, 0);
    }
    qtest_quit(qts);
}

/* +0x1024 clears nothing; +0x40 and +0x1020 restore the reset values */
static void test_resets(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
    static const uint32_t regs[] = { 0x1040, 0x1060 };

    for (unsigned r = 0; r < 3; r++) {
        uint32_t reg = r == 0 ? RESET2 : r == 1 ? RESET : 0x1020;

        for (unsigned i = 0; i < ARRAY_SIZE(regs); i++) {
            qtest_writel(qts, IPU + regs[i], 0x12345678);
        }
        qtest_writel(qts, IPU + reg, 1);
        g_assert_cmphex(qtest_readl(qts, IPU + reg), ==, 0);
        for (unsigned i = 0; i < ARRAY_SIZE(regs); i++) {
            g_assert_cmphex(qtest_readl(qts, IPU + regs[i]), ==,
                            r == 0 ? 0x12345678 : 0x028001e0);
        }
        g_assert_cmphex(qtest_readl(qts, IPU + 0x1000), ==, 0x20210708);
    }
    qtest_quit(qts);
}

static void test_copy_argb(void)
{
    QTestState *qts = ipu_init();
    enum { W = 13, H = 9, SS = 64, DS = 80 };
    uint8_t src[SS * H], dst[DS * H], want[DS * H], got[DS * H];
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 7 + 3;
    }
    memset(dst, 0xa5, sizeof(dst));
    memcpy(want, dst, sizeof(want));
    for (int y = 0; y < H; y++) {
        memcpy(want + y * DS, src + y * SS, W * 4);
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    qtest_memwrite(qts, DST, dst, sizeof(dst));
    job_fg_ot(&d, 2, W, H, SRC, SS, INFO(T_ARGB8888, 0),
              DST, DS, INFO(T_ARGB8888, 0));
    desc_put(&d, qts, DESC);
    ipu_go(qts, DESC);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 14 & 1, ==, 0);
    ipu_wait_done(qts);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 14 & 1, ==, 1);
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert(!memcmp(got, want, sizeof(got)));
    ipu_ack(qts);
    g_assert_cmphex(qtest_readl(qts, INTC_ISR1) >> 14 & 1, ==, 0);
    qtest_quit(qts);
}

/* A negative FG stride reads the rows bottom up: a vertical flip */
static void test_copy_flip(void)
{
    QTestState *qts = ipu_init();
    enum { W = 10, H = 6, S = 32 };
    uint8_t src[S * H], got[S * H], want[S * H];
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 11 + 5;
    }
    memset(want, 0, sizeof(want));
    for (int y = 0; y < H; y++) {
        memcpy(want + y * S, src + (H - 1 - y) * S, W * 2);
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    memset(got, 0, sizeof(got));
    qtest_memwrite(qts, DST, got, sizeof(got));
    job_fg_ot(&d, 5, W, H, SRC, (uint16_t)-S,
              INFO(T_ARGB1555, 0), DST, S, INFO(T_ARGB1555, 0));
    desc_put(&d, qts, DESC);
    ipu_go(qts, DESC);
    ipu_wait_done(qts);
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert(!memcmp(got, want, sizeof(got)));
    qtest_quit(qts);
}

static void test_copy_nv12(void)
{
    QTestState *qts = ipu_init();
    enum { W = 16, H = 8, S = 32 };
    uint8_t y[S * H], uv[S * H / 2], gy[S * H], guv[S * H / 2];
    Desc d;
    uint32_t fg[7] = { W << 16 | H, SRC, SRC_UV, (S << 16) | S,
                       INFO(T_NV12, 0) | 1u << 30, 0, 0 };
    uint32_t ot[5] = { W << 16 | H, DST, DST_UV, (S << 16) | S,
                       INFO(T_NV12, 0) | 1u << 30 };

    for (unsigned i = 0; i < sizeof(y); i++) {
        y[i] = i * 3;
    }
    for (unsigned i = 0; i < sizeof(uv); i++) {
        uv[i] = i * 5 + 1;
    }
    memset(gy, 0, sizeof(gy));
    memset(guv, 0, sizeof(guv));
    qtest_memwrite(qts, SRC, y, sizeof(y));
    qtest_memwrite(qts, SRC_UV, uv, sizeof(uv));
    qtest_memwrite(qts, DST, gy, sizeof(gy));
    qtest_memwrite(qts, DST_UV, guv, sizeof(guv));
    desc_begin(&d, 2);
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_kick(&d);
    desc_put(&d, qts, DESC);
    ipu_go(qts, DESC);
    ipu_wait_done(qts);
    qtest_memread(qts, DST, gy, sizeof(gy));
    qtest_memread(qts, DST_UV, guv, sizeof(guv));
    for (int r = 0; r < H; r++) {
        g_assert(!memcmp(gy + r * S, y + r * S, W));
    }
    for (int r = 0; r < H / 2; r++) {
        g_assert(!memcmp(guv + r * S, uv + r * S, W));
    }
    g_assert_cmphex(guv[S * H / 2 - 1], ==, 0);
    qtest_quit(qts);
}


static void run_job(QTestState *qts, Desc *d)
{
    desc_put(d, qts, DESC);
    ipu_go(qts, DESC);
    ipu_wait_done(qts);
}

/* A constant FG (INFO bit 25) fills OT's rectangle with INFO3 */
static void test_fill(void)
{
    QTestState *qts = ipu_init();
    enum { W = 12, H = 5, S = 64 };
    uint8_t got[S * H], want[S * H];
    uint32_t fg[7] = { 0, 0, 0, 0, INFO(3, 0) | 1u << 25, 0x100, 0xfd00 };
    uint32_t ot[5] = { W << 16 | H, DST, 0, S, INFO(T_ARGB1555, 0) };
    Desc d;

    memset(want, 0x55, sizeof(want));
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            want[y * S + x * 2] = 0x00;
            want[y * S + x * 2 + 1] = 0xfd;
        }
    }
    memset(got, 0x55, sizeof(got));
    qtest_memwrite(qts, DST, got, sizeof(got));
    desc_begin(&d, 3);
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert(!memcmp(got, want, sizeof(got)));
    qtest_quit(qts);
}

/* OT bit 28 turns each row round, a pixel at a time */
static void test_mirror(void)
{
    QTestState *qts = ipu_init();
    enum { W = 9, H = 4, S = 48 };
    uint8_t src[S * H], got[S * H], want[S * H];
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 13 + 7;
    }
    memset(want, 0, sizeof(want));
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            memcpy(want + y * S + x * 4, src + y * S + (W - 1 - x) * 4, 4);
        }
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    memset(got, 0, sizeof(got));
    qtest_memwrite(qts, DST, got, sizeof(got));
    job_fg_ot(&d, 6, W, H, SRC, S, INFO(T_ARGB8888, 0), DST, S,
              INFO(T_ARGB8888, 0) | 1u << 28);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert(!memcmp(got, want, sizeof(got)));
    qtest_quit(qts);
}

/* CLIP: INFO bit 0 keeps the box, clear keeps all but it */
static void clip_case(bool inside)
{
    QTestState *qts = ipu_init();
    enum { W = 16, H = 10, S = 32, CX = 4, CY = 2, CW = 6, CH = 5 };
    uint8_t src[S * H], got[S * H], want[S * H];
    uint32_t clip[3] = { CX << 16 | CY, CW << 16 | CH, 1u << 31 | inside };
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 3 + 1;
    }
    memset(want, 0x77, sizeof(want));
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            bool in = x >= CX && x < CX + CW && y >= CY && y < CY + CH;

            if (in == inside) {
                memcpy(want + y * S + x * 2, src + y * S + x * 2, 2);
            }
        }
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    memset(got, 0x77, sizeof(got));
    qtest_memwrite(qts, DST, got, sizeof(got));
    job_fg_ot(&d, 7, W, H, SRC, S, INFO(T_ARGB1555, 0), DST, S,
              INFO(T_ARGB1555, 0));
    d.n--;
    d.n--;
    desc_regs(&d, 0x10c0, 3, clip);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert(!memcmp(got, want, sizeof(got)));
    qtest_quit(qts);
}

static void test_clip_inside(void)
{
    clip_case(true);
}

static void test_clip_outside(void)
{
    clip_case(false);
}

/* The scaler's table: bilinear, 513 phases of eight taps, each summing 2048 */
static void put_bilinear_table(QTestState *qts, uint32_t addr)
{
    uint16_t tab[513 * 8] = { 0 };

    for (int p = 0; p <= 512; p++) {
        tab[p * 8 + 3] = 2048 - 4 * p;
        tab[p * 8 + 4] = 4 * p;
    }
    qtest_memwrite(qts, addr, tab, sizeof(tab));
}

/* The bilinear scaler, as the model's header describes it, in plain C */
static int ref_scale_pos(unsigned step, unsigned o, int *i0, unsigned *phase)
{
    int64_t pos = (int64_t)step * o + (step >> 1) - (1 << 13);

    *i0 = pos >> 14;
    *phase = MIN(((pos & 0x3fff) + 16) >> 5, 512);
    return 0;
}

static uint8_t ref_tap(const uint8_t *line, int n, int stride, int i0,
                       unsigned phase)
{
    int a = MIN(MAX(i0, 0), n - 1), b = MIN(MAX(i0 + 1, 0), n - 1);

    return ((2048 - 4 * phase) * line[a * stride] +
            4 * phase * line[b * stride] + 1024) >> 11;
}

#define TAB_H   (DESC + 0x1000)
#define TAB_V   (DESC + 0x4000)

static void resize_case(unsigned sw, unsigned sh, unsigned dw, unsigned dh)
{
    QTestState *qts = ipu_init();
    uint8_t *src = g_malloc(sw * sh * 4), *want = g_malloc(dw * dh * 4);
    uint8_t *mid = g_malloc(dw * sh * 4), *got = g_malloc(dw * dh * 4);
    unsigned hs = (sw << 14) / dw, vs = (sh << 14) / dh;
    uint32_t sc[6] = { 0x10011, hs, vs, 0, 0, 0 };
    uint32_t fg[7] = { sw << 16 | sh, SRC, 0, sw * 4, INFO(T_ARGB8888, 0),
                       0, 0 };
    uint32_t ot[5] = { dw << 16 | dh, DST, 0, dw * 4, INFO(T_ARGB8888, 0) };
    Desc d;

    for (unsigned i = 0; i < sw * sh * 4; i++) {
        src[i] = i * 37 + (i >> 3) * 5;
    }
    for (unsigned y = 0; y < sh; y++) {
        for (unsigned x = 0; x < dw; x++) {
            int i0;
            unsigned ph;

            ref_scale_pos(hs, x, &i0, &ph);
            for (int c = 0; c < 4; c++) {
                mid[(y * dw + x) * 4 + c] =
                    ref_tap(src + y * sw * 4 + c, sw, 4, i0, ph);
            }
        }
    }
    for (unsigned y = 0; y < dh; y++) {
        int i0;
        unsigned ph;

        ref_scale_pos(vs, y, &i0, &ph);
        for (unsigned x = 0; x < dw * 4; x++) {
            want[y * dw * 4 + x] = ref_tap(mid + x, sh, dw * 4, i0, ph);
        }
    }
    qtest_memwrite(qts, SRC, src, sw * sh * 4);
    put_bilinear_table(qts, TAB_H);
    put_bilinear_table(qts, TAB_V);
    desc_begin(&d, 4 | 0x30);
    d.words[2] = TAB_H;
    d.words[3] = TAB_V;
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_regs(&d, 0x1110, 6, sc);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, dw * dh * 4);
    g_assert(!memcmp(got, want, dw * dh * 4));
    qtest_quit(qts);
    g_free(src);
    g_free(want);
    g_free(mid);
    g_free(got);
}

static void test_resize_half(void)
{
    resize_case(32, 16, 16, 8);
}

static void test_resize_up(void)
{
    resize_case(13, 7, 29, 17);
}

static void test_resize_odd(void)
{
    resize_case(40, 30, 23, 11);
}

/* A half-size resize of two bilinear passes is the mean of 2x2 pixels */
static void test_resize_mean(void)
{
    QTestState *qts = ipu_init();
    uint8_t src[8 * 4 * 4], got[4 * 2 * 4];
    uint32_t sc[6] = { 0x10011, 0x8000, 0x8000, 0, 0, 0 };
    uint32_t fg[7] = { 8 << 16 | 4, SRC, 0, 32, INFO(T_ARGB8888, 0), 0, 0 };
    uint32_t ot[5] = { 4 << 16 | 2, DST, 0, 16, INFO(T_ARGB8888, 0) };
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 29;
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    put_bilinear_table(qts, TAB_H);
    put_bilinear_table(qts, TAB_V);
    desc_begin(&d, 4 | 0x30);
    d.words[2] = TAB_H;
    d.words[3] = TAB_V;
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_regs(&d, 0x1110, 6, sc);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, sizeof(got));
    for (int y = 0; y < 2; y++) {
        for (int x = 0; x < 4; x++) {
            for (int c = 0; c < 4; c++) {
                int a = src[(2 * y * 8 + 2 * x) * 4 + c];
                int b = src[(2 * y * 8 + 2 * x + 1) * 4 + c];
                int e = src[((2 * y + 1) * 8 + 2 * x) * 4 + c];
                int f = src[((2 * y + 1) * 8 + 2 * x + 1) * 4 + c];
                int top = (a + b + 1) >> 1, bot = (e + f + 1) >> 1;

                g_assert_cmpint(got[(y * 4 + x) * 4 + c], ==,
                                (top + bot + 1) >> 1);
            }
        }
    }
    qtest_quit(qts);
}

/* The default conversions libimp programs: BT.709, full range */
static const uint32_t csc_yuv2rgb[6] = {
    0x80000002, 0x00000400, 0x3f400400, 0x076c0400, 0x3e21064d, 0x00000000,
};
static const uint32_t csc_rgb2yuv[6] = {
    0x80000001, 0x02dc00da, 0x3e753f8b, 0x3e2f0200, 0x0200004a, 0x00003fd1,
};

static uint8_t clamp_u8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* NV12 to ARGB1555 through the YUV to RGB matrix */
static void test_csc_nv12_rgb(void)
{
    QTestState *qts = ipu_init();
    enum { W = 16, H = 8 };
    uint8_t y[W * H], uv[W * H / 2];
    uint16_t got[W * H];
    uint32_t fg[7] = { W << 16 | H, SRC, SRC_UV, W << 16 | W,
                       INFO(T_NV12, 0) | 1u << 30, 0, 0 };
    uint32_t ot[5] = { W << 16 | H, DST, 0, W * 2, INFO(T_ARGB1555, 0) };
    Desc d;

    for (unsigned i = 0; i < sizeof(y); i++) {
        y[i] = i * 7 + 20;
    }
    for (unsigned i = 0; i < sizeof(uv); i++) {
        uv[i] = i * 11 + 60;
    }
    qtest_memwrite(qts, SRC, y, sizeof(y));
    qtest_memwrite(qts, SRC_UV, uv, sizeof(uv));
    desc_begin(&d, 8);
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_regs(&d, 0x10d0, 6, csc_yuv2rgb);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, sizeof(got));
    for (int r = 0; r < H; r++) {
        for (int x = 0; x < W; x++) {
            int yy = y[r * W + x];
            int cb = uv[(r / 2) * W + (x & ~1)] - 128;
            int cr = uv[(r / 2) * W + (x & ~1) + 1] - 128;
            int rr = clamp_u8(yy + ((1613 * cr + 512) >> 10));
            int gg = clamp_u8(yy + ((-192 * cb - 479 * cr + 512) >> 10));
            int bb = clamp_u8(yy + ((1900 * cb + 512) >> 10));
            uint16_t want = 0x8000 | (rr >> 3) << 10 | (gg >> 3) << 5 | bb >> 3;

            g_assert_cmphex(GUINT16_FROM_LE(got[r * W + x]), ==, want | 0x8000);
        }
    }
    qtest_quit(qts);
}

/* ARGB8888 to NV12: the Y plane and the chroma of each block's first pixel */
static void test_csc_rgb_nv12(void)
{
    QTestState *qts = ipu_init();
    enum { W = 8, H = 4 };
    uint8_t src[W * H * 4], y[W * H], uv[W * H / 2];
    uint32_t fg[7] = { W << 16 | H, SRC, 0, W * 4, INFO(T_ARGB8888, 0), 0, 0 };
    uint32_t ot[5] = { W << 16 | H, DST, DST_UV, W << 16 | W,
                       INFO(T_NV12, 0) | 1u << 30 };
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 19 + 3;
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    desc_begin(&d, 8);
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_regs(&d, 0x10d0, 6, csc_rgb2yuv);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, y, sizeof(y));
    qtest_memread(qts, DST_UV, uv, sizeof(uv));
    for (int r = 0; r < H; r++) {
        for (int x = 0; x < W; x++) {
            const uint8_t *p = src + (r * W + x) * 4;
            int rr = p[2], gg = p[1], bb = p[0];
            int yy = clamp_u8((218 * rr + 732 * gg + 74 * bb + 512) >> 10);

            g_assert_cmpint(y[r * W + x], ==, yy);
            if (!(r & 1) && !(x & 1)) {
                int cb = clamp_u8(((-117 * rr - 395 * gg + 512 * bb + 512)
                                   >> 10) + 128);
                int cr = clamp_u8(((512 * rr - 465 * gg - 47 * bb + 512)
                                   >> 10) + 128);

                g_assert_cmpint(uv[(r / 2) * W + x], ==, cb);
                g_assert_cmpint(uv[(r / 2) * W + x + 1], ==, cr);
            }
        }
    }
    qtest_quit(qts);
}

#define REG_BK          0x1040
#define REG_OSD         0x1108

/* FG over BK: SRCOVER with the global alpha of FG, and the explicit modes */
static void blend_case(uint32_t cmd, uint32_t m1, uint32_t m2, int alpha)
{
    QTestState *qts = ipu_init();
    enum { W = 8, H = 4 };
    uint8_t fgp[W * H * 4], bgp[W * H * 4], got[W * H * 4];
    uint32_t bk[7] = { W << 16 | H, SRC + 0x1000, 0, W * 4,
                       INFO(T_ARGB8888, 0), 0, 0 };
    uint32_t fg[7] = { W << 16 | H, SRC, 0, W * 4, INFO(T_ARGB8888, 0),
                       2 << 8 | alpha, 0 };
    uint32_t ot[5] = { W << 16 | H, DST, 0, W * 4, INFO(T_ARGB8888, 0) };
    uint32_t osd = 1u << 31 | m2 << 8 | m1 << 4 | cmd;
    Desc d;

    for (unsigned i = 0; i < sizeof(fgp); i++) {
        fgp[i] = i * 23 + 9;
        bgp[i] = i * 41 + 200;
    }
    qtest_memwrite(qts, SRC, fgp, sizeof(fgp));
    qtest_memwrite(qts, SRC + 0x1000, bgp, sizeof(bgp));
    desc_begin(&d, 10);
    desc_regs(&d, REG_BK, 7, bk);
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_regs(&d, REG_OSD, 1, &osd);
    desc_kick(&d);
    run_job(qts, &d);
    qtest_memread(qts, DST, got, sizeof(got));
    for (unsigned i = 0; i < sizeof(got); i += 4) {
        for (int c = 0; c < 4; c++) {
            /* byte 3 is A, 2 R, 1 G, 0 B */
            int a2 = alpha;
            int p1 = bgp[i + c], p2 = c == 3 ? a2 : fgp[i + c];
            int f1 = cmd == 3 ? 255 - a2 : m1 == 1 ? 255 : 0;
            int f2 = cmd == 3 ? a2 : m2 == 1 ? 255 : 0;
            int want = MIN((p1 * f1 + p2 * f2 + 127) / 255, 255);

            g_assert_cmpint(got[i + c], ==, want);
        }
    }
    qtest_quit(qts);
}

static void test_blend_over(void)
{
    blend_case(3, 0, 0, 0x80);
}

static void test_blend_add(void)
{
    blend_case(0, 1, 1, 0xff);
}

/* Each descriptor starts by clearing the job registers: no stale clip box */
static void test_regs_cleared(void)
{
    QTestState *qts = ipu_init();
    enum { W = 16, H = 4, S = 32 };
    uint8_t src[S * H], got[S * H];
    uint32_t clip[3] = { 4 << 16 | 1, 3 << 16 | 2, 1u << 31 | 1 };
    uint32_t one = 1;
    Desc d;

    for (unsigned i = 0; i < sizeof(src); i++) {
        src[i] = i * 5 + 3;
    }
    qtest_memwrite(qts, SRC, src, sizeof(src));
    memset(got, 0x77, sizeof(got));
    qtest_memwrite(qts, DST, got, sizeof(got));
    job_fg_ot(&d, 7, W, H, SRC, S, INFO(T_ARGB1555, 0), DST, S,
              INFO(T_ARGB1555, 0));
    d.n -= 2;
    desc_regs(&d, 0x10c0, 3, clip);
    desc_kick(&d);
    d.words[1] = DESC + 0x400;
    desc_put(&d, qts, DESC);
    desc_begin(&d, 2);
    desc_regs(&d, 0x1020, 1, &one);
    {
        uint32_t fg[7] = { W << 16 | H, SRC, 0, S, INFO(T_ARGB1555, 0), 0, 0 };
        uint32_t ot[5] = { W << 16 | H, DST + 0x1000, 0, S,
                           INFO(T_ARGB1555, 0) };

        desc_regs(&d, REG_FG, 7, fg);
        desc_regs(&d, REG_OT, 5, ot);
    }
    desc_kick(&d);
    desc_put(&d, qts, DESC + 0x400);
    ipu_go(qts, DESC);
    ipu_wait_done(qts);
    qtest_memread(qts, DST + 0x1000, got, sizeof(got));
    g_assert(!memcmp(got, src, sizeof(got)));
    qtest_quit(qts);
}

/* Two descriptors linked by the next address run one after the other */
static void test_chain(void)
{
    QTestState *qts = ipu_init();
    uint8_t a[64], b[64], got[64];
    Desc d;

    memset(a, 0x11, sizeof(a));
    memset(b, 0x22, sizeof(b));
    qtest_memwrite(qts, SRC, a, sizeof(a));
    qtest_memwrite(qts, SRC + 0x1000, b, sizeof(b));
    job_fg_ot(&d, 2, 8, 2, SRC, 32, INFO(T_ARGB8888, 0), DST, 32,
              INFO(T_ARGB8888, 0));
    d.words[1] = DESC + 0x400;
    desc_put(&d, qts, DESC);
    job_fg_ot(&d, 2, 8, 2, SRC + 0x1000, 32, INFO(T_ARGB8888, 0),
              DST + 0x1000, 32, INFO(T_ARGB8888, 0));
    desc_put(&d, qts, DESC + 0x400);
    ipu_go(qts, DESC);
    ipu_wait_done(qts);
    qtest_memread(qts, DST, got, sizeof(got));
    g_assert_cmphex(got[0], ==, 0x11);
    qtest_memread(qts, DST + 0x1000, got, sizeof(got));
    g_assert_cmphex(got[0], ==, 0x22);
    qtest_quit(qts);
}

/* A job the model does not run (a mismatch of sizes) raises an error bit */
static void test_bad_job(void)
{
    QTestState *qts = ipu_init();
    Desc d;
    uint32_t fg[7] = { 8 << 16 | 8, SRC, 0, 64, INFO(T_ARGB8888, 0), 0, 0 };
    uint32_t ot[5] = { 4 << 16 | 4, DST, 0, 64, INFO(T_ARGB1555, 0) };

    desc_begin(&d, 2);
    desc_regs(&d, REG_FG, 7, fg);
    desc_regs(&d, REG_OT, 5, ot);
    desc_kick(&d);
    desc_put(&d, qts, DESC);
    ipu_go(qts, DESC);
    ipu_wait_done(qts);
    g_assert_cmphex(qtest_readl(qts, IPU + IIP_RAW), ==, 4);
    qtest_writel(qts, IPU + IIP_CLR, 4);
    g_assert_cmphex(qtest_readl(qts, IPU + IIP_RAW), ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic/a1-ipu/reset", test_reset);
    qtest_add_func("ingenic/a1-ipu/cpm", test_cpm);
    qtest_add_func("ingenic/a1-ipu/keep", test_keep);
    qtest_add_func("ingenic/a1-ipu/resets", test_resets);
    qtest_add_func("ingenic/a1-ipu/copy-argb", test_copy_argb);
    qtest_add_func("ingenic/a1-ipu/copy-flip", test_copy_flip);
    qtest_add_func("ingenic/a1-ipu/copy-nv12", test_copy_nv12);
    qtest_add_func("ingenic/a1-ipu/fill", test_fill);
    qtest_add_func("ingenic/a1-ipu/mirror", test_mirror);
    qtest_add_func("ingenic/a1-ipu/clip-inside", test_clip_inside);
    qtest_add_func("ingenic/a1-ipu/clip-outside", test_clip_outside);
    qtest_add_func("ingenic/a1-ipu/resize-mean", test_resize_mean);
    qtest_add_func("ingenic/a1-ipu/resize-half", test_resize_half);
    qtest_add_func("ingenic/a1-ipu/resize-up", test_resize_up);
    qtest_add_func("ingenic/a1-ipu/resize-odd", test_resize_odd);
    qtest_add_func("ingenic/a1-ipu/csc-nv12-rgb", test_csc_nv12_rgb);
    qtest_add_func("ingenic/a1-ipu/csc-rgb-nv12", test_csc_rgb_nv12);
    qtest_add_func("ingenic/a1-ipu/blend-over", test_blend_over);
    qtest_add_func("ingenic/a1-ipu/blend-add", test_blend_add);
    qtest_add_func("ingenic/a1-ipu/regs-cleared", test_regs_cleared);
    qtest_add_func("ingenic/a1-ipu/chain", test_chain);
    qtest_add_func("ingenic/a1-ipu/bad-job", test_bad_job);
    return g_test_run();
}
