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

static void test_copy_argb(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
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

/* A negative stride walks the rows upward: a vertical flip */
static void test_copy_flip(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
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
    job_fg_ot(&d, 2, W, H, SRC + (H - 1) * S, (uint16_t)-S,
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
    QTestState *qts = qtest_init("-machine ingenic-a1");
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

/* Two descriptors linked by the next address run one after the other */
static void test_chain(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1");
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
    QTestState *qts = qtest_init("-machine ingenic-a1");
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
    qtest_add_func("ingenic/a1-ipu/copy-argb", test_copy_argb);
    qtest_add_func("ingenic/a1-ipu/copy-flip", test_copy_flip);
    qtest_add_func("ingenic/a1-ipu/copy-nv12", test_copy_nv12);
    qtest_add_func("ingenic/a1-ipu/chain", test_chain);
    qtest_add_func("ingenic/a1-ipu/bad-job", test_bad_job);
    return g_test_run();
}
