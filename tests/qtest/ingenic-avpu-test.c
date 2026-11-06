/*
 * QTest testcase for the Ingenic AVPU's H.265 transform skip
 *
 * One intra command list entry, H.265, a 32x8 picture of alternating rows:
 * 8x8 coding blocks whose 4x4 chroma blocks have levels, through the T31's
 * EL150 and the T41's EL200. transform_skip_enabled_flag comes with the
 * entry (EL150 word 2 bit 31, EL200 word 3 bit 31); without it the slice
 * data must not carry transform_skip_flag. Both layouts code the same
 * slice data for the same picture and settings.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define W               32
#define H               8
#define QP              27

#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define LIST            0x00820000
#define STATUS          0x00830000
#define BS              0x00840000
#define ROOM            0x10000

#define CL_LAST         (1u << 31)
#define TRANSFORM_SKIP  (1u << 31)
#define CABAC           (1u << 10)

typedef struct {
    const char *machine;
    uint32_t base;          /* the AVPU's registers */
    uint32_t cl_addr, start, irq, status;
    bool el200;
} Avpu;

static const Avpu el150 = {
    .machine = "ingenic-t31",
    .base = 0x13200000,
    .cl_addr = 0x83e0,
    .start = 0x83e4,
    .irq = 0x8018,
};

static const Avpu el200 = {
    .machine = "ingenic-t41",
    .base = 0x13100000,
    .cl_addr = 0x1000,
    .start = 0x1018,
    .irq = 0x18,
    .status = 0x1008,
    .el200 = true,
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

/* Encode the picture with or without transform skip; the slice data's length */
static uint32_t encode(const Avpu *a, bool skip, uint8_t *out)
{
    QTestState *qts = qtest_initf("-machine %s", a->machine);
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    uint8_t y[W * H], c[W * H / 2];
    uint32_t len;

    /* rows of 180 and 84, chroma 128 and 184: levels in every block */
    for (int i = 0; i < W * H; i++) {
        y[i] = (i / W) & 1 ? 84 : 180;
    }
    for (int i = 0; i < W * H / 2; i++) {
        c[i] = (i / W) & 1 ? 184 : 128;
    }
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));

    if (a->el200) {
        w[0] = CL_LAST | 1;                         /* H.265 */
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 16;
        w[3] = CABAC | (skip ? TRANSFORM_SKIP : 0);
        w[24] = 2u << 28 | QP << 16;                /* intra */
        w[40] = SRC_Y;
        w[42] = SRC_C;
        w[52] = W;
        w[136] = BS;
        w[141] = ROOM;
    } else {
        w[0] = CL_LAST | 1u << 28;                  /* H.265 */
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
        w[2] = CABAC | (skip ? TRANSFORM_SKIP : 0);
        w[3] = 2u << 28 | QP << 16;                 /* intra */
        w[0x20] = SRC_Y;
        w[0x21] = SRC_C;
        w[0x22] = W;
        w[0x30] = BS;
        w[0x33] = ROOM;
    }
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    if (a->status) {
        qtest_writel(qts, a->base + a->status, STATUS);
    }
    qtest_writel(qts, a->base + a->cl_addr, LIST);
    qtest_writel(qts, a->base + a->start, 2);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, a->base + a->irq) & 1);
         i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, a->base + a->irq) & 1, ==, 1);
    /* status word 0x41, the bytes written; the EL200's first word */
    len = qtest_readl(qts, a->status ? STATUS : LIST + 0x41 * 4);
    g_assert_cmpuint(len, >, 0);
    g_assert_cmpuint(len, <, ROOM);
    qtest_memread(qts, BS, out, len);
    qtest_quit(qts);
    return len;
}

static void test_transform_skip(const void *data)
{
    const Avpu *a = data;
    g_autofree uint8_t *on = g_malloc(ROOM), *off = g_malloc(ROOM);
    g_autofree uint8_t *ref = g_malloc(ROOM);
    uint32_t n_on = encode(a, true, on), n_off = encode(a, false, off);
    uint32_t n_ref;

    /* the flag's bins go, and the rest stays as coded */
    g_assert_cmpuint(n_on, ==, 75);
    g_assert_cmphex(crc32_le(on, n_on), ==, 0x86f35b82);
    g_assert_cmpuint(n_off, ==, 74);
    g_assert_cmphex(crc32_le(off, n_off), ==, 0xee17f728);

    /* the other layout codes the same slice data */
    n_ref = encode(a->el200 ? &el150 : &el200, false, ref);
    g_assert_cmpmem(off, n_off, ref, n_ref);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_data_func("ingenic/avpu/el150/transform-skip", &el150,
                        test_transform_skip);
    qtest_add_data_func("ingenic/avpu/el200/transform-skip", &el200,
                        test_transform_skip);
    return g_test_run();
}
