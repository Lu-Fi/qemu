/*
 * QTest testcase for the Ingenic LDC (lens distortion correction)
 *
 * The T32's engine (0x13040000) as a T32LQ answered: its registers out of
 * reset and the bits they keep, two jobs on a 640 x 360 noise picture
 * (xorshift32 bytes, as the probe tool made them on the part) checked by
 * the CRC of the part's output, then gradients (zoom alone, k1 alone, and
 * the vendor's table) the same way, the status, mask and clear, and a job that
 * never ends until TIME_OUT ends it, and the columns left of the picture
 * that a zoom reads, which the part answers from its line buffer. The
 * T30's (0x130a0000) as a T30X answered: its reset values, kept bits and
 * control bits, and two 1280 x 720 jobs as its driver programs them, one
 * with the y axis's own coefficients. The T33's and T32 Pro's table-driven
 * engine in their ISPs,
 * modelled from their drivers alone: an identity table, a shift, a
 * rotation and the status and interrupt.
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
    const int *ramp;            /* a gradient instead of noise, see above */
} Job;

/*
 * Gradients as the probe tool wrote them: byte (ax * x + ay * y + c) of
 * each plane, the chroma's pairs from the pair's own x and the chroma row;
 * the nine numbers are Y's ax, ay, c, then U's and V's
 */
static const int ramp_x16[] = { 16, 0, 0, 16, 0, 0, 16, 0, 0 };
static const int ramp_x16o[] = { 16, 0, 128, 16, 0, 128, 16, 0, 128 };
static const int ramp_x64[] = { 64, 0, 0, 64, 0, 0, 64, 0, 0 };
static const int ramp_x64o[] = { 64, 0, 128, 64, 0, 128, 64, 0, 128 };
static const int ramp_y16[] = { 0, 16, 0, 0, 16, 0, 0, 16, 128 };
static const int ramp_y64[] = { 0, 64, 0, 0, 64, 0, 0, 64, 128 };

static const Job t32_jobs[] = {
    /* The vendor's 640 x 360 table */
    { "sample", 0x1234, 194, 71, 2135, 2135, 8024, 0x9fb3fd4e },
    { "zooms", 0x4242, 150, 40, 2100, 2090, 8024, 0x28f34422 },
    /* Gradients through the part: zoom alone, k1 alone, and both k1 and k2 */
    { "zoom-x16", 0, 0, 0, 2040, 2040, 8024, 0x5fe7c246, ramp_x16 },
    { "zoom-x16o", 0, 0, 0, 2040, 2040, 8024, 0x3340870c, ramp_x16o },
    { "zoom-x64", 0, 0, 0, 2040, 2040, 8024, 0x4ab9ee55, ramp_x64 },
    { "zoom-x64o", 0, 0, 0, 2040, 2040, 8024, 0xbe424acc, ramp_x64o },
    { "k1-x16", 0, 194, 0, 2135, 2135, 8024, 0x897501a9, ramp_x16 },
    { "k1-x16o", 0, 194, 0, 2135, 2135, 8024, 0x0ca6ce86, ramp_x16o },
    { "k1-x64", 0, 194, 0, 2135, 2135, 8024, 0x5e3fd79a, ramp_x64 },
    { "k1-x64o", 0, 194, 0, 2135, 2135, 8024, 0xbe228dec, ramp_x64o },
    { "sample-y16", 0, 194, 71, 2135, 2135, 8024, 0xaee13f15, ramp_y16 },
    { "sample-y64", 0, 194, 71, 2135, 2135, 8024, 0x555d87bc, ramp_y64 },
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

static void ramp(uint8_t *p, const int *r)
{
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            p[y * W + x] = r[0] * x + r[1] * y + r[2];
        }
    }
    for (int y = 0; y < H / 2; y++) {
        for (int x = 0; x < W / 2; x++) {
            uint8_t *uv = p + W * H + y * W + 2 * x;

            uv[0] = r[3] * x + r[4] * y + r[5];
            uv[1] = r[6] * x + r[7] * y + r[8];
        }
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

    if (j->ramp) {
        ramp(buf, j->ramp);
    } else {
        noise(buf, LEN, j->seed);
    }
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
 * k2 alone (zoom 1900 on both axes) on the 640 x 360 noise picture. The
 * line buffer decides the top and bottom rows (lines it did not fetch show
 * what an earlier job left, so those differ from run to run on the part)
 * and, across the middle, one column on each side of the centre; the rest
 * is the part's output, checked by CRC32 of the Y rows and chroma rows
 * between.
 */
static const struct {
    uint32_t k2, crc;
    int y[2][2], uv[2][2];
} k2_sets[] = {
    { 71, 0x2dbdef1d, { { 8, 127 }, { 233, 353 } },
                      { { 4, 63 }, { 117, 176 } } },
    { 40, 0x4a17f046, { { 4, 91 }, { 269, 356 } },
                      { { 2, 45 }, { 135, 178 } } },
};

static uint32_t k2_rows_crc(const uint8_t *buf, int set)
{
    g_autofree uint8_t *rows = g_malloc(LEN);
    size_t n = 0;

    for (int k = 0; k < 2; k++) {
        size_t len = (k2_sets[set].y[k][1] - k2_sets[set].y[k][0]) * W;

        memcpy(rows + n, buf + k2_sets[set].y[k][0] * W, len);
        n += len;
    }
    for (int k = 0; k < 2; k++) {
        size_t len = (k2_sets[set].uv[k][1] - k2_sets[set].uv[k][0]) * W;

        memcpy(rows + n, buf + W * H + k2_sets[set].uv[k][0] * W, len);
        n += len;
    }
    return crc32_le(rows, n);
}

static void test_t32_k2(void)
{
    for (int i = 0; i < ARRAY_SIZE(k2_sets); i++) {
        QTestState *qts = qtest_init("-machine ingenic-t32");
        Job j = { "k2", 0x1234, 0, k2_sets[i].k2, 1900, 1900, 8024, 0 };
        g_autofree uint8_t *buf = g_malloc(LEN);

        t32_setup(qts, &j);
        qtest_writel(qts, T32_LDC + 0x04, 1);
        qtest_clock_step(qts, 1000);
        g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
        qtest_memread(qts, DST, buf, LEN);
        g_assert_cmphex(k2_rows_crc(buf, i), ==, k2_sets[i].crc);
        qtest_quit(qts);
    }
}

/*
 * k2 alone at zoom 1900 x 1900 with bx 1892 on the noise picture: the pixels
 * 112 from the centre on each side (output columns 207 and 432) take their
 * source column from a line window fixed at the top row, so in the rows
 * where the column differs the taps wrap inside the 128 byte line and the
 * second one reads the next line's first byte. The part's output at those two
 * columns over rows 12 to 347, but for the rows 128, 129, 180 and 181 where
 * the buffer had not loaded the next line, is pinned by CRC32.
 */
static void test_t32_window(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");
    Job j = { "window", 0x1234, 0, 71, 1892, 1900, 8024, 0 };
    g_autofree uint8_t *buf = g_malloc(LEN);
    g_autofree uint8_t *cols = g_malloc(2 * H);
    size_t n = 0;

    t32_setup(qts, &j);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
    qtest_memread(qts, DST, buf, LEN);
    for (int y = 12; y < 348; y++) {
        if (y == 128 || y == 129 || y == 180 || y == 181) {
            continue;
        }
        cols[n++] = buf[y * W + 207];
        cols[n++] = buf[y * W + 432];
    }
    g_assert_cmphex(crc32_le(cols, n), ==, 0xe49858e6);
    qtest_quit(qts);
}

/*
 * Two jobs back to back, an earlier one on other data and then the k2
 * job on an output buffer full of noise. On a T32LQ the corner rows of the
 * second differ with the first because the line buffer keeps its lines
 * (and not because the engine skips those bytes: the noise was gone from
 * every byte, the corners included), and neither the soft reset (0x64) nor
 * the clock gate cleared them. The rows between are the same whatever came
 * before, and the model leaves none of the noise.
 */
static void test_t32_two_jobs(void)
{
    uint32_t seeds[] = { 0x77, 0x5a5a, 0x1234 };

    for (int a = 0; a < ARRAY_SIZE(seeds); a++) {
        QTestState *qts = qtest_init("-machine ingenic-t32");
        Job first = { "first", seeds[a], 0, 71, 1900, 1900, 8024, 0 };
        Job j = { "k2", 0x1234, 0, 71, 1900, 1900, 8024, 0 };
        g_autofree uint8_t *buf = g_malloc(LEN);
        g_autofree uint8_t *pre = g_malloc(LEN);
        size_t same = 0;

        t32_setup(qts, &first);
        qtest_writel(qts, T32_LDC + 0x04, 1);
        qtest_clock_step(qts, 1000);
        g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
        qtest_writel(qts, T32_LDC + 0x6c, 1);

        t32_setup(qts, &j);
        noise(pre, LEN, 999);
        qtest_memwrite(qts, DST, pre, LEN);
        qtest_writel(qts, T32_LDC + 0x64, 1);
        qtest_writel(qts, T32_LDC + 0x04, 1);
        qtest_clock_step(qts, 1000);
        g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
        qtest_memread(qts, DST, buf, LEN);
        g_assert_cmphex(k2_rows_crc(buf, 0), ==, k2_sets[0].crc);
        for (size_t i = 0; i < LEN; i++) {
            same += buf[i] == pre[i];
        }
        /* a byte in 256 matches by chance */
        g_assert_cmpuint(same, <, LEN / 128);
        qtest_quit(qts);
    }
}

/* A T41LQ's: the T32's jobs to the byte, its own VERSION, no online bit */
static void test_t41_job(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t41");
    g_autofree uint8_t *buf = g_malloc(LEN);

    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    g_assert_cmphex(qtest_readl(qts, T32_LDC), ==, 0x20211229);
    qtest_writel(qts, T32_LDC + 0x08, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x08), ==, 0);
    t32_setup(qts, &t32_jobs[0]);
    qtest_writel(qts, T32_LDC + 0x70, 0);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
    g_assert_true(qtest_get_irq(qts, 35));
    qtest_memread(qts, DST, buf, LEN);
    g_assert_cmphex(crc32_le(buf, LEN), ==, t32_jobs[0].crc);
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

/*
 * The T30X's picture edges, k and the tables left out: a zoom that reads
 * left of the picture takes the second tap of the pixel at x = -1 from
 * byte 8 of the row, and on a 640 byte row the centre's first burst
 * (wr 12: 104 bytes, counted from the edge in) ends at byte 8, so the 8
 * bytes after it are the output 8 bytes further out of the first rows of
 * that half of the picture. Both checked by the CRC of the part's output,
 * and the clock count FRM_CLK by the part's, which the model fits.
 */
static void t30_edge_job(uint32_t w, uint32_t h, uint32_t seed,
                         uint32_t zoom, uint32_t r2, uint32_t crc,
                         uint32_t clk)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    size_t len = w * h * 3 / 2;
    g_autofree uint8_t *buf = g_malloc0(len);

    noise(buf, len, seed);
    qtest_memwrite(qts, SRC, buf, len);
    qtest_writel(qts, T30_LDC + 0x00, 2);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | 12);
    qtest_writel(qts, T30_LDC + 0x1c, zoom);
    qtest_writel(qts, T30_LDC + 0x20, r2);
    qtest_writel(qts, T30_LDC + 0x24, 0x11 << 16 | 0x22 << 8 | 0x33);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, w);
    }
    qtest_writel(qts, T30_LDC + 0x08, h << 16 | w);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + w * h);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + w * h);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010001);
    qtest_memread(qts, DST, buf, len);
    g_assert_cmphex(crc32_le(buf, len), ==, crc);
    /* FRM_CLK is a fit, within 5% of the part's count */
    g_assert_cmpuint(qtest_readl(qts, T30_LDC + 0x28), >, clk * 95 / 100);
    g_assert_cmpuint(qtest_readl(qts, T30_LDC + 0x28), <, clk * 105 / 100);
    qtest_quit(qts);
}

static void test_t30_edges(void)
{
    t30_edge_job(640, 360, 0x1234, 2047 << 16 | 2097, 8024, 0x6f4227d8,
                 298409);
    t30_edge_job(1280, 720, 0x6666, 2040 << 16 | 2070, 1991, 0xebe15199,
                 1046826);
}

/*
 * Two identity jobs of 640 x 360 noise into the same output, LINE_LEN
 * wr 12 / wrs 1 then wr 3 / wrs 1: the second's first burst at the centre
 * (B = 32 bytes) is longer than the rows' own (A = 16), the 16 bytes
 * between are not written, and keep what the first job's output held
 * there. Checked by the CRC of the T30X's output after the second job.
 */
static void t30_run_job(QTestState *qts, uint32_t seed, uint32_t wr)
{
    g_autofree uint8_t *buf = g_malloc0(LEN);

    noise(buf, LEN, seed);
    qtest_memwrite(qts, SRC, buf, LEN);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | wr);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010001);
    qtest_writel(qts, T30_LDC + 0x04, 1);
}

static void test_t30_band_kept(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    g_autofree uint8_t *buf = g_malloc0(LEN);

    qtest_memwrite(qts, DST, buf, LEN);
    qtest_writel(qts, T30_LDC + 0x00, 2);
    qtest_writel(qts, T30_LDC + 0x1c, 2048 << 16 | 2048);
    qtest_writel(qts, T30_LDC + 0x20, 8024);
    qtest_writel(qts, T30_LDC + 0x24, 0x11 << 16 | 0x22 << 8 | 0x33);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, W);
    }
    qtest_writel(qts, T30_LDC + 0x08, H << 16 | W);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + W * H);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + W * H);
    t30_run_job(qts, 0x1111, 12);
    t30_run_job(qts, 0x2222, 3);
    qtest_memread(qts, DST, buf, LEN);
    g_assert_cmphex(crc32_le(buf, LEN), ==, 0xf0a580df);
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

/*
 * A T30X's Y table entry (one per burst of the half row, from the edge in)
 * on 640 x 360 jobs without x distortion: the job ends when the entry is
 * within the lag the y axis builds up at that column, and never otherwise.
 * Every case here is a T30X's answer (job done, or WORK still set).
 */
static uint32_t t30_table_job(uint32_t py, uint32_t ky, int idx, uint32_t v)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint32_t st;

    qtest_writel(qts, T30_LDC + 0x00, 2);
    qtest_writel(qts, T30_LDC + 0x100 + 4 * idx, v);
    qtest_writel(qts, T30_LDC + 0x10, ky);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | 12);
    qtest_writel(qts, T30_LDC + 0x1c, py << 16 | 2048);
    qtest_writel(qts, T30_LDC + 0x20, 8024);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, W);
    }
    qtest_writel(qts, T30_LDC + 0x08, H << 16 | W);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + W * H);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + W * H);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    st = qtest_readl(qts, T30_LDC + 0x04);
    qtest_quit(qts);
    return st;
}

static void test_t30_table(void)
{
    /* no distortion: an entry below h / 2 hangs, 0 and h / 2 do not */
    g_assert_cmphex(t30_table_job(2048, 0, 2, 50), ==, 0x30010002);
    g_assert_cmphex(t30_table_job(2048, 0, 2, 179), ==, 0x30010002);
    g_assert_cmphex(t30_table_job(2048, 0, 2, 180), ==, 0x30010001);
    g_assert_cmphex(t30_table_job(2048, 0, 2, 0), ==, 0x30010001);
    /* a zoom of 2048 or more shifts no line the right way */
    g_assert_cmphex(t30_table_job(2100, 0, 0, 1), ==, 0x30010002);
    /* a zoom out of 1950 loses 8 lines by the last row of a half, any entry */
    g_assert_cmphex(t30_table_job(1950, 0, 1, 8), ==, 0x30010001);
    g_assert_cmphex(t30_table_job(1990, 0, 3, 5), ==, 0x30010001);
    g_assert_cmphex(t30_table_job(1950, 0, 1, 9), ==, 0x30010002);
    g_assert_cmphex(t30_table_job(1990, 0, 0, 5), ==, 0x30010001);
    g_assert_cmphex(t30_table_job(1990, 0, 0, 6), ==, 0x30010002);
    /* entry 4 is past the 4 bursts of a 640 wide job */
    g_assert_cmphex(t30_table_job(2048, 0, 4, 50), ==, 0x30010001);
    /* k1 on y alone: 20 lines by the edge column of burst 0, 5 at the centre */
    g_assert_cmphex(t30_table_job(2048, 200, 0, 20), ==, 0x30010001);
    g_assert_cmphex(t30_table_job(2048, 200, 0, 21), ==, 0x30010002);
    g_assert_cmphex(t30_table_job(2048, 100, 3, 4), ==, 0x30010001);
    g_assert_cmphex(t30_table_job(2048, 100, 3, 5), ==, 0x30010002);
}

/*
 * The same on the vendor's 1280 x 720 job (k1 352, k2 139 on both axes,
 * zoom 2212, R2 1991, wr 12), whose x axis crosses columns into the next
 * burst by the last rows: entries 3, 4 and 5 hang above the y lag until
 * the n rows (80, 22, 2) the source has crossed for have been passed.
 */
static uint32_t t30_vendor_job2(int idx, uint32_t v, int idx2, uint32_t v2)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint32_t st;

    qtest_writel(qts, T30_LDC + 0x00, 2);
    qtest_writel(qts, T30_LDC + 0x100 + 4 * idx, v);
    if (idx2 >= 0) {
        qtest_writel(qts, T30_LDC + 0x100 + 4 * idx2, v2);
    }
    qtest_writel(qts, T30_LDC + 0x0c, 0x008b0160);
    qtest_writel(qts, T30_LDC + 0x10, 0x008b0160);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | 12);
    qtest_writel(qts, T30_LDC + 0x1c, 2212 << 16 | 2212);
    qtest_writel(qts, T30_LDC + 0x20, 1991);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, 1280);
    }
    qtest_writel(qts, T30_LDC + 0x08, 720 << 16 | 1280);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + 1280 * 720);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + 1280 * 720);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    st = qtest_readl(qts, T30_LDC + 0x04);
    qtest_quit(qts);
    return st;
}

static uint32_t t30_vendor_job(int idx, uint32_t v)
{
    return t30_vendor_job2(idx, v, -1, 0);
}

static void test_t30_table_x(void)
{
    /* entry 5 (column 16): no line lag, hangs for v 1 and 2, ends from 3 */
    g_assert_cmphex(t30_vendor_job(5, 1), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(5, 2), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(5, 3), ==, 0x30010001);
    /* entry 4 (column 120): ends for v 1 and 2, hangs for 3 to 22 */
    g_assert_cmphex(t30_vendor_job(4, 2), ==, 0x30010001);
    g_assert_cmphex(t30_vendor_job(4, 3), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(4, 22), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(4, 23), ==, 0x30010001);
    /* entry 3 (column 224): ends up to 9, hangs for 10 to 80 */
    g_assert_cmphex(t30_vendor_job(3, 9), ==, 0x30010001);
    g_assert_cmphex(t30_vendor_job(3, 10), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(3, 80), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(3, 81), ==, 0x30010001);
    /* entry 6 (column 1) never crosses: it hangs alone, whatever v */
    g_assert_cmphex(t30_vendor_job(6, 1), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(6, 285), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job(6, 359), ==, 0x30010002);
    /* beside entry 4 above its window it ends, beside one under the lag not */
    g_assert_cmphex(t30_vendor_job2(4, 23, 6, 285), ==, 0x30010001);
    g_assert_cmphex(t30_vendor_job2(4, 221, 6, 285), ==, 0x30010001);
    g_assert_cmphex(t30_vendor_job2(4, 1, 6, 285), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job2(4, 2, 6, 285), ==, 0x30010002);
    g_assert_cmphex(t30_vendor_job2(4, 3, 6, 285), ==, 0x30010002);
}

/*
 * A T30X's job on 960 x 360 with an empty table: a k on the y axis hangs
 * it once R2 makes r = (d^2 + e^2) * R2 reach 2^30 at the corner pixel
 * (0x100a is the last R2 that does not), a k1 on x alone does not.
 */
static uint32_t t30_wrap_job(uint32_t kx, uint32_t ky, uint32_t r2)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    uint32_t st;

    qtest_writel(qts, T30_LDC + 0x00, 2);
    qtest_writel(qts, T30_LDC + 0x0c, kx);
    qtest_writel(qts, T30_LDC + 0x10, ky);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | 12);
    qtest_writel(qts, T30_LDC + 0x1c, 2048 << 16 | 2048);
    qtest_writel(qts, T30_LDC + 0x20, r2);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, 960);
    }
    qtest_writel(qts, T30_LDC + 0x08, 360 << 16 | 960);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + 960 * 360);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + 960 * 360);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 100 * 1000 * 1000);
    st = qtest_readl(qts, T30_LDC + 0x04);
    qtest_quit(qts);
    return st;
}

static void test_t30_r_wrap(void)
{
    g_assert_cmphex(t30_wrap_job(0, 100, 0x100a), ==, 0x30010001);
    g_assert_cmphex(t30_wrap_job(0, 100, 0x100b), ==, 0x30010002);
    g_assert_cmphex(t30_wrap_job(0, 100 << 16, 0x100a), ==, 0x30010001);
    g_assert_cmphex(t30_wrap_job(0, 100 << 16, 0x100b), ==, 0x30010002);
    g_assert_cmphex(t30_wrap_job(0, 100, 0x1100), ==, 0x30010002);
    g_assert_cmphex(t30_wrap_job(0, 0, 0x1100), ==, 0x30010001);
    g_assert_cmphex(t30_wrap_job(100, 0, 0x1100), ==, 0x30010001);
    g_assert_cmphex(t30_wrap_job(100, 100, 0x1100), ==, 0x30010002);
}

/*
 * A job of w x 64 noise (seed 0x2222) into an output holding noise 0x3333,
 * wr 15 (128 byte bursts), wrs 1, no zoom or distortion: the damaged burst
 * ends of the part's output, checked by its CRC.
 */
static void t30_wr15_job(uint32_t w, uint32_t crc)
{
    QTestState *qts = qtest_init("-machine ingenic-t30");
    size_t len = w * 64 * 3 / 2;
    g_autofree uint8_t *buf = g_malloc0(len);

    noise(buf, len, 0x3333);
    qtest_memwrite(qts, DST, buf, len);
    noise(buf, len, 0x2222);
    qtest_memwrite(qts, SRC, buf, len);
    qtest_writel(qts, T30_LDC + 0x00, 2);
    qtest_writel(qts, T30_LDC + 0x18, 15 << 24 | 15 << 16 | 1 << 8 | 15);
    qtest_writel(qts, T30_LDC + 0x1c, 2048 << 16 | 2048);
    qtest_writel(qts, T30_LDC + 0x20, 8024);
    qtest_writel(qts, T30_LDC + 0x24, 0x11 << 16 | 0x22 << 8 | 0x33);
    for (int i = 0; i < 4; i++) {
        qtest_writel(qts, T30_LDC + 0x3c + 4 * i, w);
    }
    qtest_writel(qts, T30_LDC + 0x08, 64 << 16 | w);
    qtest_writel(qts, T30_LDC + 0x2c, SRC);
    qtest_writel(qts, T30_LDC + 0x30, SRC + w * 64);
    qtest_writel(qts, T30_LDC + 0x34, DST);
    qtest_writel(qts, T30_LDC + 0x38, DST + w * 64);
    qtest_writel(qts, T30_LDC + 0x00, 0x38);
    qtest_writel(qts, T30_LDC + 0x00, 0x39);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==, 0x30010001);
    qtest_memread(qts, DST, buf, len);
    g_assert_cmphex(crc32_le(buf, len), ==, crc);
    qtest_quit(qts);
}

static void test_t30_burst_edges(void)
{
    t30_wr15_job(448, 0xa2f39aa1);
    t30_wr15_job(640, 0x55b60931);
    t30_wr15_job(960, 0x4561853f);
    t30_wr15_job(1280, 0x3e3dbccc);
}

/*
 * LINE_LEN bits 19:16 and 27:24 on the T30X: the job ends only for 1 <=
 * bits 27:24 <= bits 19:16 + 1 and bits 19:16 >= 1 (every one of the 256
 * pairs on 640 x 360 identity jobs), and hangs otherwise.
 */
static void test_t30_line_len(void)
{
    static const int cases[][2] = {
        { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 }, { 1, 2 }, { 1, 3 },
        { 8, 9 }, { 8, 10 }, { 13, 15 }, { 14, 15 }, { 15, 0 }, { 15, 15 },
    };
    QTestState *qts = qtest_init("-machine ingenic-t30");

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        int rd = cases[i][0], rds = cases[i][1];
        bool ends = rd >= 1 && rds >= 1 && rds <= rd + 1;

        qtest_writel(qts, T30_LDC + 0x00, 2);
        qtest_writel(qts, T30_LDC + 0x18, rds << 24 | rd << 16 | 1 << 8 | 12);
        qtest_writel(qts, T30_LDC + 0x1c, 2048 << 16 | 2048);
        qtest_writel(qts, T30_LDC + 0x20, 8024);
        for (int k = 0; k < 4; k++) {
            qtest_writel(qts, T30_LDC + 0x3c + 4 * k, 64);
        }
        qtest_writel(qts, T30_LDC + 0x08, 32 << 16 | 64);
        qtest_writel(qts, T30_LDC + 0x2c, SRC);
        qtest_writel(qts, T30_LDC + 0x30, SRC + 64 * 32);
        qtest_writel(qts, T30_LDC + 0x34, DST);
        qtest_writel(qts, T30_LDC + 0x38, DST + 64 * 32);
        qtest_writel(qts, T30_LDC + 0x00, 0x38);
        qtest_writel(qts, T30_LDC + 0x00, 0x39);
        qtest_clock_step(qts, 100 * 1000 * 1000);
        g_assert_cmphex(qtest_readl(qts, T30_LDC + 0x04), ==,
                        ends ? 0x30010001 : 0x30010002);
    }
    qtest_quit(qts);
}

#define ISP             0x13300000
#define CPM_CLKGR0      0x10000020
#define TW              64
#define TH              32
#define TLEN            (TW * TH * 3 / 2)
#define LUT             0x00c00000

/* Program an offline job of a 64 x 32 picture, block 16 */
static void isp_ldc_job(QTestState *qts, uint32_t base, uint32_t type)
{
    qtest_writel(qts, base + 0x40, type);
    qtest_writel(qts, base + 0x44, TH << 16 | TW);
    qtest_writel(qts, base + 0x48, TH << 16 | TW);
    qtest_writel(qts, base + 0x4c, SRC);
    qtest_writel(qts, base + 0x50, TW);
    qtest_writel(qts, base + 0x54, DST);
    qtest_writel(qts, base + 0x58, (type >> 16) & 1 ? TH : TW);
    qtest_writel(qts, base + 0x5c, SRC + TW * TH);
    qtest_writel(qts, base + 0x60, TW);
    qtest_writel(qts, base + 0x64, DST + TW * TH);
    qtest_writel(qts, base + 0x68, (type >> 16) & 1 ? TH : TW);
    qtest_writel(qts, base + 0x6c, LUT);
    qtest_writel(qts, base + 0x70, 24);
    qtest_writel(qts, base + 0x74, 1);
    qtest_writel(qts, base + 0x30, 1);
    qtest_writel(qts, base + 0x20, 0xffffffff);
    qtest_writel(qts, base + 0x24, 1);
}

/* A table whose nodes take the input dx / 16 pixel to their right */
static void isp_ldc_table(QTestState *qts, int dx)
{
    for (int r = 0; r <= TH / 16; r++) {
        for (int c = 0; c <= TW / 16; c++) {
            int x = MIN(c * 16, TW - 1), y = MIN(r * 16, TH - 1);

            qtest_writew(qts, LUT + r * 24 + c * 4, x * 16 + dx);
            qtest_writew(qts, LUT + r * 24 + c * 4 + 2, y * 16);
        }
    }
}

static void test_isp_ldc(const void *data)
{
    const char *variant = data;
    QTestState *qts = qtest_initf("-machine %s%s%s",
                                  strcmp(variant, "t33") ? "ingenic-t32"
                                                         : "ingenic-t33",
                                  strcmp(variant, "t33") ?
                                  " -global ingenic-t31.soc-variant=" : "",
                                  strcmp(variant, "t33") ? variant : "");
    uint32_t base = ISP + (strcmp(variant, "t33") ? 0x1f000 : 0x27000);
    uint8_t in[TLEN], out[TLEN];

    qtest_writel(qts, CPM_CLKGR0, 0);
    qtest_irq_intercept_in(qts, "/machine/soc/intc");
    for (int i = 0; i < TLEN; i++) {
        in[i] = i < TW * TH ? 3 * (i % TW) + i / TW : 0x40 + i % 7;
    }
    qtest_memwrite(qts, SRC, in, TLEN);

    /* The identity: the picture as it was, done, the interrupt raised */
    isp_ldc_table(qts, 0);
    isp_ldc_job(qts, base, 0);
    g_assert_cmphex(qtest_readl(qts, base + 0x80), ==, 1);
    g_assert_true(qtest_get_irq(qts, T32_LDC_IRQ));
    qtest_memread(qts, DST, out, TLEN);
    g_assert_cmpmem(out, TLEN, in, TLEN);
    qtest_writel(qts, base + 0x84, 1);
    g_assert_cmphex(qtest_readl(qts, base + 0x80), ==, 0);
    g_assert_false(qtest_get_irq(qts, T32_LDC_IRQ));

    /* Half a pixel right: luma Y = 3x + y gives 3x + y + 1 (1.5 cut) */
    qtest_writel(qts, base + 0x88, 0xffffffff);
    isp_ldc_table(qts, 8);
    isp_ldc_job(qts, base, 0);
    g_assert_cmphex(qtest_readl(qts, base + 0x80), ==, 1);
    g_assert_false(qtest_get_irq(qts, T32_LDC_IRQ));
    qtest_memread(qts, DST, out, TLEN);
    g_assert_cmpuint(out[5 * TW + 10], ==, 3 * 10 + 5 + 1);
    qtest_writel(qts, base + 0x84, 1);

    /* A 90 degree turn: row 0 of the output is column 0 bottom up */
    isp_ldc_job(qts, base, 1 << 16 | 2);
    qtest_memread(qts, DST, out, TW * TH);
    for (int x = 0; x < TH; x++) {
        g_assert_cmpuint(out[x], ==, in[(TH - 1 - x) * TW]);
    }
    qtest_quit(qts);
}

/*
 * A noise job whose zoom of 2100 reads up to nine columns left of the
 * picture, which the part answers from its line buffer. The columns it
 * answers as the model does (Y 0 to 6, the chroma pairs' first 6 bytes) are
 * checked by CRC; a few rows of the next column and of the chroma's
 * others are not reproduced.
 */
static void test_t32_left_edge(void)
{
    static const Job j = { "left", 0x777, 0, 0, 2100, 2047, 8024 };
    QTestState *qts = qtest_init("-machine ingenic-t32");
    g_autofree uint8_t *buf = g_malloc(LEN);
    g_autofree uint8_t *cols = g_malloc(H * 7);
    size_t n = 0;

    t32_setup(qts, &j);
    qtest_writel(qts, T32_LDC + 0x70, 0xffffffff);
    qtest_writel(qts, T32_LDC + 0x04, 1);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, T32_LDC + 0x68), ==, 1);
    qtest_memread(qts, DST, buf, LEN);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < 7; x++) {
            cols[n++] = buf[y * W + x];
        }
    }
    g_assert_cmphex(crc32_le(cols, n), ==, 0xea8f59fa);
    n = 0;
    for (int y = 0; y < H / 2; y++) {
        for (int x = 0; x < 6; x++) {
            cols[n++] = buf[W * H + y * W + x];
        }
    }
    g_assert_cmphex(crc32_le(cols, n), ==, 0xe3fa8cb0);
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
    qtest_add_func("ingenic-ldc/t32/left-edge", test_t32_left_edge);
    qtest_add_func("ingenic-ldc/t32/k2", test_t32_k2);
    qtest_add_func("ingenic-ldc/t32/two-jobs", test_t32_two_jobs);
    qtest_add_func("ingenic-ldc/t32/window", test_t32_window);
    qtest_add_func("ingenic-ldc/t32/hang", test_t32_hang);
    qtest_add_func("ingenic-ldc/t41/job", test_t41_job);
    qtest_add_func("ingenic-ldc/t30/resets", test_t30_resets);
    for (int i = 0; i < ARRAY_SIZE(t30_jobs); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-ldc/t30/%s",
                                                t30_jobs[i].name);
        qtest_add_data_func(path, &t30_jobs[i], test_t30_job);
    }
    qtest_add_func("ingenic-ldc/t30/hang", test_t30_hang);
    qtest_add_func("ingenic-ldc/t30/r-wrap", test_t30_r_wrap);
    qtest_add_func("ingenic-ldc/t30/edges", test_t30_edges);
    qtest_add_func("ingenic-ldc/t30/band-kept", test_t30_band_kept);
    qtest_add_func("ingenic-ldc/t30/table", test_t30_table);
    qtest_add_func("ingenic-ldc/t30/table-x", test_t30_table_x);
    qtest_add_func("ingenic-ldc/t30/burst-edges", test_t30_burst_edges);
    qtest_add_func("ingenic-ldc/t30/line-len", test_t30_line_len);
    qtest_add_data_func("ingenic-ldc/t33/isp", "t33", test_isp_ldc);
    qtest_add_data_func("ingenic-ldc/t32pro/isp", "t32prozn", test_isp_ldc);
    return g_test_run();
}
