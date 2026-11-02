/*
 * QTest testcase for the Ingenic IPU (ipu_v13)
 *
 * Each case programs the IPU as the vendor driver's ipu_start does, over a
 * 32x8 NV12 frame with an 8x4 OSD picture at (8, 2), and checks the CRC32
 * of the frame (an OSD blend, in place) or of the output (the CSC command)
 * against a T31X's for the same job at U-Boot: pictures taken as AYUV and
 * through PARA bit 25's RGB to YUV, byte orders, alpha per pixel, global,
 * multiplied and premultiplied, NV12 and NV21 pictures, the 16-bit pictures
 * of type 4; the CSC command to NV21, ARGB and HSV, each plane one row
 * longer than the frame, read from past it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define IPU             0x13080000
#define IPU_STATUS      0x004
#define IPU_D_FMT       0x008
#define IPU_Y_ADDR      0x00c
#define IPU_U_ADDR      0x010
#define IPU_IN_FM_GS    0x018
#define IPU_Y_STRIDE    0x01c
#define IPU_UV_STRIDE   0x020
#define IPU_OUT_ADDR    0x024
#define IPU_OUT_STRIDE  0x02c
#define IPU_OUT_V_ADDR  0x030
#define IPU_OUT_V_STRIDE 0x034
#define IPU_REG_CTRL    0x064
#define IPU_TRIGGER     0x074
#define IPU_NV_OUT_ADDR 0x0b4
#define IPU_NV_OUT_STRIDE 0x0b8
#define IPU_OSD_Y_ADDR  0x0bc
#define IPU_OSD_Y_STRIDE 0x0c0
#define IPU_OSD_UV_ADDR 0x0c4
#define IPU_OSD_UV_STRIDE 0x0c8
#define IPU_OSD_GS      0x0fc
#define IPU_OSD_POS     0x10c
#define IPU_OSD_PARA    0x11c
#define IPU_CH_BK_PARA  0x12c
#define IPU_OSD_BAK     0x130
#define IPU_CH_B_BAK    0x140
#define IPU_CSC(i)      (0x148 + (i) * 4)

#define BG              0x01000000
#define OSD             0x01100000
#define OUT             0x01200000
#define W               32

enum { PIC_RAMP, PIC_RAMP_PREM, PIC_COLOURS, PIC_COLOURS_BGRA, PIC_NV12,
       PIC_1555, PIC_ARGB };

typedef struct {
    const char *name;
    int pic;
    uint32_t para;              /* an OSD job, or */
    const char *out;            /* a CSC command to this */
    unsigned h, len;
    uint32_t crc;               /* of the T31X's frame or output */
    bool dfmt0;                 /* an OSD job with D_FMT 0, as older libimp */
} Case;

static const Case cases[] = {
    { "ayuv", PIC_RAMP, 0x000007f9, NULL, 8, 384, 0xb4baa983 },
    { "csc-colours", PIC_COLOURS, 0x020007f9, NULL, 8, 384, 0xe8f99756 },
    { "csc-order-bgra", PIC_COLOURS_BGRA, 0x020347f9, NULL, 8, 384,
      0xe8f99756 },
    { "csc-ramp", PIC_RAMP, 0x020007f9, NULL, 8, 384, 0x29dd3310 },
    { "csc-prem", PIC_RAMP_PREM, 0x020407f9, NULL, 8, 384, 0x9159be19 },
    { "csc-mul80", PIC_RAMP, 0x02000405, NULL, 8, 384, 0x010cf6ac },
    { "csc-global80", PIC_RAMP, 0x02000403, NULL, 8, 384, 0xb60ff3ad },
    { "nv12-ga40", PIC_NV12, 0x00001203, NULL, 8, 384, 0x94b4b982 },
    { "nv12-ga81", PIC_NV12, 0x0000140b, NULL, 8, 384, 0x75b77018 },
    { "1555-raw-o3", PIC_1555, 0x0000e7f9, NULL, 8, 384, 0x009b3f15 },
    { "1555-csc-o5", PIC_1555, 0x020167f9, NULL, 8, 384, 0x4ae31c1b },
    { "1555-csc-mul80", PIC_1555, 0x02002405, NULL, 8, 384, 0xfd7949f0 },
    { "dfmt0", PIC_ARGB, 0x000007f9, NULL, 8, 384, 0xc5571e54, true },
    { "cmd-nv21", 0, 0, "nv21", 8, 640, 0xe8b3562c },
    { "cmd-argb", 0, 0, "argb", 6, 1024, 0x8d6f7b35 },
    { "cmd-hsv", 0, 0, "hsv", 8, 1280, 0xff0c065f },
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

/* The NV12 frame, then 256 bytes of a known pattern past it */
static size_t frame(uint8_t *p, unsigned h)
{
    size_t n = 0;

    for (unsigned y = 0; y < h; y++) {
        for (unsigned x = 0; x < W; x++) {
            p[n++] = x * 7 + y * 23 + 16;
        }
    }
    for (unsigned y = 0; y < h / 2; y++) {
        for (unsigned x = 0; x < W; x++) {
            p[n++] = x & 1 ? x * 3 + y * 17 + 200 : x * 11 + y * 5 + 40;
        }
    }
    for (unsigned i = 0; i < 256; i++) {
        p[n + i] = i * 13 + 7;
    }
    return n;
}

/* The 8x4 OSD picture; its size in bytes */
static size_t picture(uint8_t *p, int kind)
{
    static const uint8_t first[8][3] = {
        { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 255 },
        { 0, 0, 0 }, { 128, 128, 128 }, { 255, 255, 0 }, { 0, 255, 255 },
    };

    for (int i = 0; i < 32; i++) {
        int a = MIN(255, i * 8 + (i == 31) * 7);
        uint8_t rgb[3] = { 200, 60, 30 };

        switch (kind) {
        case PIC_RAMP:
        case PIC_RAMP_PREM:
            for (int c = 0; c < 3 && kind == PIC_RAMP_PREM; c++) {
                rgb[c] = (rgb[c] * a + 127) / 255;
            }
            p[4 * i] = a;
            memcpy(p + 4 * i + 1, rgb, 3);
            break;
        case PIC_COLOURS:
        case PIC_COLOURS_BGRA:
            if (i < 8) {
                memcpy(rgb, first[i], 3);
            } else {
                rgb[0] = (i - 8) * 53 + 11;
                rgb[1] = (i - 8) * 97 + 31;
                rgb[2] = (i - 8) * 151 + 71;
            }
            if (kind == PIC_COLOURS) {
                p[4 * i] = 255;
                memcpy(p + 4 * i + 1, rgb, 3);
            } else {
                p[4 * i] = rgb[2];
                p[4 * i + 1] = rgb[1];
                p[4 * i + 2] = rgb[0];
                p[4 * i + 3] = 255;
            }
            break;
        case PIC_NV12:
            p[i] = i * 29 + 5;
            if (i < 16) {
                p[32 + i] = i * 61 + 77;
            }
            break;
        case PIC_ARGB:
            p[4 * i] = i * 255 / 31;
            p[4 * i + 1] = i * 37 + 20;
            p[4 * i + 2] = i * 59 + 90;
            p[4 * i + 3] = i * 83 + 160;
            break;
        case PIC_1555: {
            uint16_t v = (i % 3 != 0) << 15 | ((i * 3) & 31) << 10 |
                         ((i * 7 + 4) & 31) << 5 | ((i * 11 + 9) & 31);

            p[2 * i] = v;
            p[2 * i + 1] = v >> 8;
            break;
        }
        }
    }
    return kind == PIC_NV12 ? 48 : kind == PIC_1555 ? 64 : 128;
}

static void w32(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, IPU + off, v);
}

static void test_case(const void *data)
{
    const Case *c = data;
    QTestState *qts = qtest_init("-machine ingenic-t31");
    unsigned h = c->h;
    uint8_t mem[1024];
    g_autofree uint8_t *buf = g_malloc(c->len);
    size_t n = frame(mem, h);

    qtest_memwrite(qts, BG, mem, n + 256);
    w32(qts, IPU_TRIGGER, 2);
    w32(qts, IPU_TRIGGER, 8);
    w32(qts, IPU_IN_FM_GS, W << 16 | h);
    w32(qts, IPU_Y_STRIDE, W);
    w32(qts, IPU_UV_STRIDE, W << 16 | W);
    w32(qts, IPU_Y_ADDR, BG);
    w32(qts, IPU_U_ADDR, BG + W * h);
    if (c->out) {
        static const uint32_t yuv2rgb[6] = {
            0x4ad, 0x669 | 0x4ad << 20, 0x193 | 0x344 << 20,
            0x4ad | 0x81a << 20, 0, 0x10 | 0x80 << 16,
        };

        memset(buf, 0x5a, c->len);
        qtest_memwrite(qts, OUT, buf, c->len);
        if (!strcmp(c->out, "nv21")) {
            w32(qts, IPU_D_FMT, 0x3);
            w32(qts, IPU_OUT_STRIDE, W);
            w32(qts, IPU_NV_OUT_STRIDE, W);
            w32(qts, IPU_OUT_ADDR, OUT);
            w32(qts, IPU_NV_OUT_ADDR, OUT + W * h);
        } else if (!strcmp(c->out, "hsv")) {
            w32(qts, IPU_D_FMT, 0x4);
            w32(qts, IPU_OUT_STRIDE, W);
            w32(qts, IPU_NV_OUT_STRIDE, 2 * W);
            w32(qts, IPU_OUT_V_STRIDE, W);
            w32(qts, IPU_OUT_ADDR, OUT + 2 * W * h);
            w32(qts, IPU_NV_OUT_ADDR, OUT);
            w32(qts, IPU_OUT_V_ADDR, OUT + 3 * W * h);
        } else {
            w32(qts, IPU_D_FMT, 0x0);
            w32(qts, IPU_OUT_STRIDE, 4 * W);
            w32(qts, IPU_OUT_ADDR, OUT);
        }
        w32(qts, IPU_OSD_PARA, 0x010417fa);
        w32(qts, IPU_CH_BK_PARA, 0x141000);
        for (int i = 0; i < 6; i++) {
            w32(qts, IPU_CSC(i), yuv2rgb[i]);
        }
    } else {
        static const uint32_t rgb2yuv[6] = {
            0x133 | 0x25a << 20, 0x75 | 0x97 << 20, 0x128 | 0x1bf << 20,
            0x276 | 0x210 << 20, 0x67, 0x10 | 0x80 << 16,
        };
        static const uint32_t yuv[6] = {
            0x400, 0x490 | 0x400 << 20, 0x190 | 0x252 << 20,
            0x400 | 0x81f << 20, 0, 0x10 | 0x80 << 16,
        };
        uint8_t pic[128];
        size_t len = picture(pic, c->pic);
        unsigned bpp = c->pic == PIC_NV12 ? 1 : c->pic == PIC_1555 ? 2 : 4;

        qtest_memwrite(qts, OSD, pic, len);
        w32(qts, IPU_CH_B_BAK, 0xffaa7733);
        w32(qts, IPU_OUT_STRIDE, W);
        w32(qts, IPU_NV_OUT_STRIDE, W);
        w32(qts, IPU_CH_BK_PARA, 2 << 11 | 1 << 18 | 1);
        w32(qts, IPU_D_FMT, c->dfmt0 ? 0 : 0x2);
        w32(qts, IPU_OUT_ADDR, BG);
        w32(qts, IPU_NV_OUT_ADDR, BG + W * h);
        w32(qts, IPU_OSD_Y_STRIDE, 8 * bpp);
        if (bpp == 1) {
            w32(qts, IPU_OSD_UV_STRIDE, 8);
        }
        w32(qts, IPU_OSD_GS, (8 * bpp) << 16 | 4);
        w32(qts, IPU_OSD_POS, 8 << 16 | 2);
        for (int i = 0; i < 6; i++) {
            w32(qts, IPU_CSC(i), bpp > 1 ? rgb2yuv[i] : yuv[i]);
        }
        w32(qts, IPU_OSD_BAK, 0);
        w32(qts, IPU_OSD_PARA, c->para);
        w32(qts, IPU_OSD_Y_ADDR, OSD);
        w32(qts, IPU_OSD_UV_ADDR, OSD + 32);
    }
    w32(qts, IPU_STATUS, 0);
    w32(qts, IPU_REG_CTRL, 0xffffffff);
    w32(qts, IPU_TRIGGER, 1);
    g_assert_cmphex(qtest_readl(qts, IPU + IPU_STATUS) & 1, ==, 1);
    qtest_memread(qts, c->out ? OUT : BG, buf, c->len);
    g_assert_cmphex(crc32_le(buf, c->len), ==, c->crc);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-ipu/%s",
                                                cases[i].name);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
