/*
 * QTest testcase for the Ingenic LDC (lens distortion correction)
 *
 * The T32's engine (0x13040000) as a T32LQ answered: its registers out of
 * reset and the bits they keep, two jobs on a 640 x 360 noise picture
 * (xorshift32 bytes, as the probe tool made them on the part) checked by
 * the CRC of the part's output, the status, mask and clear, and a job that
 * never ends until TIME_OUT ends it. The T30's (0x130a0000) as a T30X
 * answered: its reset values, kept bits and control bits, and two 1280 x
 * 720 jobs as its driver programs them, one with the y axis's own
 * coefficients.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define T32_LDC         0x13040000
#define T32_LDC_IRQ     38
#define T30_LDC         0x130a0000
#define T30_LDC_IRQ     20
#define SRC             0x00800000
#define DST             0x00a00000
#define W               640
#define H               360
#define LEN             (W * H * 3 / 2)

typedef struct {
    const char *name;
    uint32_t seed, k1, k2, bx, by, r2;
    uint32_t crc;               /* of the part's output picture */
} Job;

static const Job t32_jobs[] = {
    /* The vendor's 640 x 360 table */
    { "sample", 0x1234, 194, 71, 2135, 2135, 8024, 0x9fb3fd4e },
    { "zooms", 0x4242, 150, 40, 2100, 2090, 8024, 0x28f34422 },
};

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

static void noise(uint8_t *p, size_t n, uint32_t x)
{
    x |= 1;
    for (size_t i = 0; i < n; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        p[i] = x;
    }
}

static void check_regs(QTestState *qts, uint32_t base,
                       const uint32_t (*want)[3], size_t n)
{
    for (size_t i = 0; i < n; i++) {
        g_test_message("0x%03x", want[i][0]);
        g_assert_cmphex(qtest_readl(qts, base + want[i][0]), ==, want[i][1]);
        qtest_writel(qts, base + want[i][0], 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, base + want[i][0]), ==, want[i][2]);
        qtest_writel(qts, base + want[i][0], want[i][1]);
    }
}

static void test_t32_resets(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");
    static const uint32_t want[][3] = {
        /* offset, reset, kept of all ones */
        { 0x00, 0x20240103, 0x20240103 }, { 0x08, 0, 1 },
        { 0x0c, 0, 0 }, { 0x10, 0x04380780, 0xffffffff },
        { 0x14, 0x00660133, 0x01ff01ff }, { 0x18, 0, 0 },
        { 0x1c, 0x08910891, 0x0fff0fff }, { 0x20, 0x377, 0x1fff },
        { 0x24, 0, 0x003fffff }, { 0x30, 0, 0xffffffff },
        { 0x34, 0, 0xffff }, { 0x38, 0, 0xffffffff }, { 0x3c, 0, 0xffff },
        { 0x40, 0, 0xffffffff }, { 0x44, 0, 0xffff },
        { 0x48, 0, 0xffffffff }, { 0x4c, 0, 0xffff },
        { 0x50, 0x030d1010, 0x1f1f1f1f }, { 0x60, 0x00140120, 0x00ff0137 },
        { 0x68, 0, 0 }, { 0x70, 0, 0xffffffff }, { 0x74, 0, 1 },
        { 0x78, 0xffffffff, 0xffffffff }, { 0x7c, 0, 0 }, { 0xf84, 0, 0 },
        { 0x1000, 0, 0x7ff }, { 0x13fc, 0, 0x7ff }, { 0x3000, 0, 0x7ff },
        { 0x2000, 0, 0 },
    };

    check_regs(qts, T32_LDC, want, ARRAY_SIZE(want));
    /* The tables' 1 KiB windows repeat */
    qtest_writel(qts, T32_LDC + 0x1004, 0x123);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x1404), ==, 0x123);
    qtest_quit(qts);
}

static void t32_setup(QTestState *qts, const Job *j)
{
    g_autofree uint8_t *buf = g_malloc0(LEN);

    noise(buf, LEN, j->seed);
    qtest_memwrite(qts, SRC, buf, LEN);
    memset(buf, 0, LEN);
    qtest_memwrite(qts, DST, buf, LEN);
    qtest_writel(qts, T32_LDC + 0x10, H << 16 | W);
    qtest_writel(qts, T32_LDC + 0x14, j->k2 << 16 | j->k1);
    qtest_writel(qts, T32_LDC + 0x1c, j->by << 16 | j->bx);
    qtest_writel(qts, T32_LDC + 0x20, j->r2);
    qtest_writel(qts, T32_LDC + 0x30, SRC);
    qtest_writel(qts, T32_LDC + 0x34, W);
    qtest_writel(qts, T32_LDC + 0x38, DST);
    qtest_writel(qts, T32_LDC + 0x3c, W);
    qtest_writel(qts, T32_LDC + 0x40, SRC + W * H);
    qtest_writel(qts, T32_LDC + 0x44, W);
    qtest_writel(qts, T32_LDC + 0x48, DST + W * H);
    qtest_writel(qts, T32_LDC + 0x4c, W);
    qtest_writel(qts, T32_LDC + 0x50, 0x000c0f0f);
}

static void test_t32_job(const void *data)
{
    const Job *j = data;
    QTestState *qts = qtest_init("-machine ingenic-t32");
    g_autofree uint8_t *buf = g_malloc(LEN);

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    t32_setup(qts, j);
    qtest_writel(qts, T32_LDC + 0x70, 0xffffffff);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);

    /* Done, the status raised under the mask, no interrupt */
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x04), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
    g_assert_false(qtest_get_irq(qts, T32_LDC_IRQ));
    qtest_writel(qts, T32_LDC + 0x70, 0xfffffffe);
    g_assert_true(qtest_get_irq(qts, T32_LDC_IRQ));
    qtest_writel(qts, T32_LDC + 0x6c, 2);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
    qtest_writel(qts, T32_LDC + 0x6c, 1);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 0);
    g_assert_false(qtest_get_irq(qts, T32_LDC_IRQ));

    qtest_memread(qts, DST, buf, LEN);
    g_assert_cmphex(crc32_le(buf, LEN), ==, j->crc);
    qtest_quit(qts);
}

/*
 * Zoom 1 with no correction takes the bottom row from row h - 1 exactly:
 * the job never ends, and TIME_OUT (0x1000000 clocks, 42 ms on the part)
 * ends it with status 3
 */
static void test_t32_hang(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");
    Job j = { "hang", 1, 0, 0, 2048, 2048, 8024, 0 };

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    t32_setup(qts, &j);
    qtest_writel(qts, T32_LDC + 0x78, 0x1000000);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 40 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x04), ==, 1);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 0);
    qtest_clock_step(qts, 2 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x04), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 3);
    g_assert_true(qtest_get_irq(qts, T32_LDC_IRQ));

    /* Zoom 2048 along x alone runs: columns do not stop it */
    qtest_writel(qts, T32_LDC + 0x6c, 3);
    qtest_writel(qts, T32_LDC + 0x1c, 2047 << 16 | 2048);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);

    /* A read burst length of 0 hangs it; writing 0 abandons the job */
    qtest_writel(qts, T32_LDC + 0x6c, 3);
    qtest_writel(qts, T32_LDC + 0x50, 0x000c0f00);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x04), ==, 1);
    qtest_writel(qts, T32_LDC + 0x04, 0);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x04), ==, 0);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 0);
    qtest_quit(qts);
}

typedef struct {
    const char *name;
    uint32_t seed, ky, zoom, crc;
} T30Job;

/* The driver's 1280 x 720 set, and one with y's k1 200, k2 60, zoom 2100 */
static const T30Job t30_jobs[] = {
    { "720p", 0x2468, 139 << 16 | 352, 2212 << 16 | 2212, 0xfe6b8520 },
    { "y-axis", 0x5555, 60 << 16 | 200, 2100 << 16 | 2212, 0x786b05de },
};

static void test_t30_resets(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    static const uint32_t want[][3] = {
        { 0x08, 0x02d00500, 0x0fff0fff }, { 0x0c, 0, 0x01ff01ff },
        { 0x10, 0, 0x01ff01ff }, { 0x14, 0, 0x003fffff },
        { 0x18, 0, 0x0f0f0f0f }, { 0x1c, 0, 0x0fff0fff },
        { 0x20, 0, 0x1fff }, { 0x24, 0, 0x00ffffff }, { 0x28, 0, 0 },
        { 0x2c, 0, 0xffffffff }, { 0x38, 0, 0xffffffff },
        { 0x3c, 0, 0xffff }, { 0x48, 0, 0xffff }, { 0x4c, 0, 0 },
        { 0x100, 0, 0x3ff }, { 0x4fc, 0, 0x3ff }, { 0x500, 0, 0x3ff },
        { 0x8fc, 0, 0x3ff }, { 0x900, 0, 0 },
    };

    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010000);
    check_regs(qts, T30_LDC, want, ARRAY_SIZE(want));
    /* CTR keeps bits 5:3; bit 1 resets the registers, not the tables */
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x00), ==, 0x38);
    qtest_writel(qts, T30_LDC + 0x14, 0x123);
    qtest_writel(qts, T30_LDC + 0x100, 0x77);
    qtest_writel(qts, T30_LDC + 0x00, 2);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x00), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x14), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x08), ==, 0x02d00500);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x100), ==, 0x77);
    /* So does the CPM's SRBC1 bit 8 */
    qtest_writel(qts, T30_LDC + 0x14, 0x123);
    qtest_writel(qts, 0x100000ac, 0x100);
    qtest_writel(qts, 0x100000ac, 0);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x14), ==, 0);
    qtest_quit(qts);
}

static void t30_setup(QTestState *qts, const T30Job *j, uint32_t w,
                      uint32_t h)
{
    size_t len = w * h * 3 / 2;
    g_autofree uint8_t *buf = g_malloc0(len);
    static const uint16_t ylut[] = { 0, 0, 0, 0, 221, 0, 285 };
    static const uint16_t uvlut[] = { 0, 0, 0, 0, 109, 0, 144 };

    noise(buf, len, j->seed);
    qtest_memwrite(qts, SRC, buf, len);
    memset(buf, 0, len);
    qtest_memwrite(qts, DST, buf, len);
    qtest_writel(qts, T30_LDC + 0x00, 2);
    for (int i = 0; i < ARRAY_SIZE(ylut); i++) {
        qtest_writel(qts, T30_LDC + 0x100 + 4 * i, ylut[i]);
        qtest_writel(qts, T30_LDC + 0x500 + 4 * i, uvlut[i]);
    }
    qtest_writel(qts, T30_LDC + 0x0c, 139 << 16 | 352);
    qtest_writel(qts, T30_LDC + 0x10, j->ky);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | 12);
    qtest_writel(qts, T30_LDC + 0x1c, j->zoom);
    qtest_writel(qts, T30_LDC + 0x20, 1991);
    qtest_writel(qts, T30_LDC + 0x24, 255 << 16 | 128 << 8 | 128);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, w);
    }
    qtest_writel(qts, T30_LDC + 0x08, h << 16 | w);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + w * h);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + w * h);
}

static void test_t30_job(const void *data)
{
    const T30Job *j = data;
    QTestState *qts = qtest_init("-machine ingenic-t30");
    size_t len = 1280 * 720 * 3 / 2;
    g_autofree uint8_t *buf = g_malloc(len);

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    t30_setup(qts, j, 1280, 720);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 1000);

    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x00), ==, 0x38);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010001);
    g_assert_true(qtest_get_irq(qts, T30_LDC_IRQ));
    qtest_writel(qts, T30_LDC + 0x04, 2);
    g_assert_true(qtest_get_irq(qts, T30_LDC_IRQ));
    qtest_writel(qts, T30_LDC + 0x04, 1);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010000);
    g_assert_false(qtest_get_irq(qts, T30_LDC_IRQ));

    qtest_memread(qts, DST, buf, len);
    g_assert_cmphex(crc32_le(buf, len), ==, j->crc);
    qtest_quit(qts);
}

/* The full view mode (CTR bit 4 clear) never ends; stop, then reset */
static void test_t30_hang(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");

    t30_setup(qts, &t30_jobs[0], 1280, 720);
    qtest_writel(qts, T30_LDC + 0x00, 0x08);
    qtest_writel(qts, T30_LDC + 0x00, 0x09);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010002);
    qtest_writel(qts, T30_LDC + 0x00, 0x04);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x00), ==, 0);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x3001000a);
    qtest_writel(qts, T30_LDC + 0x00, 0x02);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010000);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-ldc/t32/resets", test_t32_resets);
    for (int i = 0; i < ARRAY_SIZE(t32_jobs); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-ldc/t32/%s",
                                                t32_jobs[i].name);
        qtest_add_data_func(path, &t32_jobs[i], test_t32_job);
    }
    qtest_add_func("ingenic-ldc/t32/hang", test_t32_hang);
    qtest_add_func("ingenic-ldc/t30/resets", test_t30_resets);
    for (int i = 0; i < ARRAY_SIZE(t30_jobs); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-ldc/t30/%s",
                                                t30_jobs[i].name);
        qtest_add_data_func(path, &t30_jobs[i], test_t30_job);
    }
    qtest_add_func("ingenic-ldc/t30/hang", test_t30_hang);
    return g_test_run();
}
