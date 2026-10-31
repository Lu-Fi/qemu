/*
 * QTest testcase for the Ingenic DRAW_BOX (T40, T41)
 *
 * Each case programs the block as the vendor driver does, over a picture
 * filled with a known pattern, and checks the picture's CRC32 against the
 * one a T41NQ produced from the same boxes: every mode, line widths 0 to
 * 7, the four colours, corners shorter than the line, and a 1280x720
 * picture.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define DBOX            0x130d0000
#define DBOX_CTRL       0x00
#define DBOX_YB         0x04
#define DBOX_CB         0x08
#define DBOX_STRIDE     0x0c
#define DBOX_IMG_WH     0x10
#define DBOX_COLOR_Y    0x14
#define DBOX_COLOR_U    0x18
#define DBOX_COLOR_V    0x1c
#define DBOX_RAM        0x20

#define CTRL_START      (1u << 0)
#define CTRL_RESET      (1u << 1)
#define CTRL_IRQ_MASK   (1u << 3)
#define CTRL_DONE       (1u << 4)

#define PICTURE         0x800000

typedef struct {
    unsigned x, y, w, h, lw, ll, mode, colour;
} Box;

typedef struct {
    const char *name;
    const char *machine;
    unsigned w, h;
    uint32_t crc;               /* of the T41NQ's picture */
    unsigned n;
    Box b[8];
} Case;

static const Case cases[] = {
    { "rect1", "ingenic-t41", 640, 360, 0x8e145408, 1,
      { { 64, 64, 128, 64, 1, 0, 0, 0 } } },
    { "widths", "ingenic-t41", 640, 360, 0x24510c82, 8,
      { { 16, 16, 64, 48, 0, 0, 0, 0 }, { 96, 16, 64, 48, 1, 0, 0, 1 },
        { 176, 16, 64, 48, 2, 0, 0, 2 }, { 256, 16, 64, 48, 3, 0, 0, 3 },
        { 336, 16, 64, 48, 4, 0, 0, 0 }, { 416, 16, 64, 48, 5, 0, 0, 1 },
        { 496, 16, 64, 48, 6, 0, 0, 2 }, { 16, 100, 128, 96, 7, 0, 0, 3 } } },
    { "corners", "ingenic-t41", 640, 360, 0x59220930, 4,
      { { 32, 32, 160, 120, 2, 16, 1, 0 }, { 224, 32, 160, 120, 2, 32, 1, 2 },
        { 416, 32, 160, 120, 4, 8, 1, 3 },
        { 32, 200, 160, 120, 1, 60, 1, 1 } } },
    { "lines", "ingenic-t41", 640, 360, 0xa853a1be, 4,
      { { 32, 40, 300, 8, 2, 0, 2, 0 }, { 32, 80, 300, 8, 4, 0, 2, 2 },
        { 400, 40, 8, 280, 2, 0, 3, 3 }, { 480, 40, 8, 280, 5, 0, 3, 1 } } },
    { "fit", "ingenic-t41", 640, 360, 0x70157689, 4,
      { { 8, 8, 624, 344, 2, 0, 0, 0 }, { 536, 256, 96, 96, 2, 0, 0, 2 },
        { 8, 200, 64, 64, 3, 0, 0, 3 }, { 552, 8, 80, 40, 1, 0, 0, 1 } } },
    { "wide", "ingenic-t41", 1280, 720, 0x5c05eb4e, 3,
      { { 256, 128, 512, 384, 2, 0, 0, 0 }, { 800, 400, 400, 300, 3, 40, 1, 2 },
        { 4, 4, 1272, 712, 1, 0, 0, 3 } } },
    { "rect1-t40", "ingenic-t40", 640, 360, 0x8e145408, 1,
      { { 64, 64, 128, 64, 1, 0, 0, 0 } } },
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

static void test_case(const void *data)
{
    const Case *c = data;
    QTestState *qts = qtest_initf("-machine %s", c->machine);
    size_t len = c->w * c->h * 3 / 2;
    g_autofree uint8_t *pic = g_malloc(len);
    uint32_t ctrl;

    for (unsigned y = 0; y < c->h; y++) {
        for (unsigned x = 0; x < c->w; x++) {
            pic[y * c->w + x] = x + 2 * y;
        }
    }
    for (unsigned y = 0; y < c->h / 2; y++) {
        for (unsigned x = 0; x < c->w; x++) {
            pic[c->w * c->h + y * c->w + x] = x * 3 + y;
        }
    }
    qtest_memwrite(qts, PICTURE, pic, len);

    qtest_writel(qts, DBOX + DBOX_CTRL, CTRL_RESET | CTRL_IRQ_MASK);
    g_assert_cmphex(qtest_readl(qts, DBOX + DBOX_CTRL) & CTRL_RESET, ==, 0);
    qtest_writel(qts, DBOX + DBOX_IMG_WH, c->w | c->h << 16);
    qtest_writel(qts, DBOX + DBOX_STRIDE, c->w | c->w << 16);
    qtest_writel(qts, DBOX + DBOX_COLOR_Y, 0xe195004c);
    qtest_writel(qts, DBOX + DBOX_COLOR_U, 0x002b8054);
    qtest_writel(qts, DBOX + DBOX_COLOR_V, 0x941580ff);
    qtest_writel(qts, DBOX + DBOX_YB, PICTURE);
    qtest_writel(qts, DBOX + DBOX_CB, PICTURE + c->w * c->h);
    for (unsigned i = 0; i < c->n; i++) {
        const Box *b = &c->b[i];

        qtest_writel(qts, DBOX + DBOX_RAM, b->x | b->y << 12 | b->w << 24);
        qtest_writel(qts, DBOX + DBOX_RAM, b->w >> 8 | b->h << 4 |
                     b->colour << 16 | b->mode << 18 | b->lw << 20 |
                     b->ll << 23);
    }
    qtest_writel(qts, DBOX + DBOX_CTRL, CTRL_START);
    qtest_clock_step(qts, 1000);
    ctrl = qtest_readl(qts, DBOX + DBOX_CTRL);
    g_assert_cmphex(ctrl & CTRL_DONE, ==, CTRL_DONE);
    g_assert_cmphex(ctrl & 0x1fffe01, ==, c->n << 17 | c->n << 9);

    qtest_memread(qts, PICTURE, pic, len);
    g_assert_cmphex(crc32_le(pic, len), ==, c->crc);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        g_autofree char *path = g_strdup_printf("ingenic-drawbox/%s",
                                                cases[i].name);
        qtest_add_data_func(path, &cases[i], test_case);
    }
    return g_test_run();
}
