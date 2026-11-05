/*
 * Ingenic DRAW_BOX (T40, T41)
 *
 * Draws up to 255 boxes into an NV12 or (CTRL bit 6) BGRA picture in
 * memory. Each box is two words written to the RAM register: x (11:0), y
 * (23:12) and width bits 7:0 (31:24); width bits 11:8 (3:0), height
 * (15:4), colour (17:16), mode (19:18), line width (22:20) and line length
 * (31:23). CTRL bits 16:9 count the boxes written since a reset, bits 24:17
 * the boxes drawn. Starting (CTRL bit 0) draws them all, then sets bit 4
 * unless bit 3 masks the interrupt; bit 4 also reads back what software
 * writes. Reset (bit 1) empties the RAM, done at once.
 *
 * As a T41NQ draws, byte for byte, through its driver and programmed
 * directly: a box covers x..x+w-1, y..y+h-1 and its lines are 2 * (line
 * width + 1) pixels thick, inward. Mode 0 draws the left and right sides,
 * then the top and bottom, each kept to the box and each row of those two
 * laid as two halves split in its middle byte; mode 1 an arm of
 * line-length pixels along both sides from each corner, no thicker than
 * that length, and nothing at all with a length past the box's width or
 * height; mode 2 a horizontal line of the thickness from (x, y), w long,
 * however tall the box; mode 3 a vertical one, h long. In NV12 a drawn
 * pixel takes the colour's byte from COLOR_Y, and the chroma rows under
 * every other row of a part (unless the UV mask, bit 2, is set) take its U
 * and V by turns on the same columns, U first. In BGRA a pixel is COLOR_Y
 * ORed with COLOR_U for colour 1 or COLOR_V for colour 2, the stride is in
 * pixels, and a half's bytes start the word over, so an odd-width
 * rectangle's second halves are two bytes on. Boxes are drawn in order.
 *
 * Silicon does not keep a box to the picture: one past its edge, and mode
 * 1 with a line length of 0 (whose arms run on through memory), wrote far
 * outside the buffer and corrupted the kernel. The model logs those and
 * draws only what lies in the picture, and nothing for the second.
 * Not modelled: the timeout and the time a job takes.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "hw/core/irq.h"
#include "hw/misc/ingenic-drawbox.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"

#define DBOX_CTRL       0x00
#define DBOX_YB         0x04
#define DBOX_CB         0x08
#define DBOX_STRIDE     0x0c
#define DBOX_IMG_WH     0x10
#define DBOX_COLOR_Y    0x14
#define DBOX_COLOR_U    0x18
#define DBOX_COLOR_V    0x1c
#define DBOX_RAM        0x20
#define DBOX_TIMEOUT    0x24

#define CTRL_START      BIT(0)
#define CTRL_RESET      BIT(1)
#define CTRL_UV_MASK    BIT(2)
#define CTRL_IRQ_MASK   BIT(3)
#define CTRL_DONE       BIT(4)
#define CTRL_BGRA       BIT(6)
#define CTRL_RW         0x1ec   /* bits 2, 3, 5 to 8 hold what is written */

static void dbox_update_irq(IngenicDrawboxState *s)
{
    qemu_set_irq(s->irq, (s->ctrl & CTRL_DONE) && !(s->ctrl & CTRL_IRQ_MASK));
}

/* @n bytes at @addr, @pat (@plen bytes) laid from its first byte on */
static void dbox_bytes(uint64_t addr, int n, const uint8_t *pat, int plen)
{
    g_autofree uint8_t *buf = g_malloc(n);

    for (int i = 0; i < n; i++) {
        buf[i] = pat[i % plen];
    }
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, n);
}

/* A run of @n bytes, or two halves of it, each from the pattern's start */
static void dbox_run_bytes(uint64_t addr, int n, const uint8_t *pat, int plen,
                           bool split)
{
    if (split) {
        dbox_bytes(addr, n / 2, pat, plen);
        dbox_bytes(addr + n / 2, n - n / 2, pat, plen);
    } else {
        dbox_bytes(addr, n, pat, plen);
    }
}

/*
 * A filled rectangle of the picture, row by row; with @split each row is
 * laid as two halves, each from the colour's first byte. In NV12 its luma,
 * and unless the UV mask is set chroma along each other row from its first
 * (row / 2), on the same columns, U and V by turns from the run's start,
 * odd or even. In BGRA a word a pixel, COLOR_Y ORed with the colour's U or
 * V word (colour 1 U, 2 V; 0 and 3 COLOR_Y alone), the stride counted in
 * pixels, so a split row of odd width puts its second half's words two
 * bytes on.
 */
static void dbox_rect(IngenicDrawboxState *s, int x, int y, int w, int h,
                      unsigned colour, bool split)
{
    int pw = extract32(s->img_wh, 0, 16), ph = extract32(s->img_wh, 16, 16);
    uint32_t ystride = extract32(s->stride, 0, 16);
    uint32_t cstride = extract32(s->stride, 16, 16);
    uint8_t pat[4];

    if (x < 0 || y < 0 || x + w > pw || y + h > ph) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-drawbox: %dx%d at %d,%d is "
                      "outside the %dx%d picture\n", w, h, x, y, pw, ph);
        w = MIN(x + w, pw) - MAX(x, 0);
        h = MIN(y + h, ph) - MAX(y, 0);
        x = MAX(x, 0);
        y = MAX(y, 0);
    }
    if (w <= 0 || h <= 0) {
        return;
    }
    if (s->ctrl & CTRL_BGRA) {
        stl_le_p(pat, s->color[0] | (colour == 1 ? s->color[1] :
                                     colour == 2 ? s->color[2] : 0));
        for (int r = y; r < y + h; r++) {
            dbox_run_bytes(s->yb + 4 * ((uint64_t)r * ystride + x), 4 * w,
                           pat, 4, split);
        }
        return;
    }
    pat[0] = extract32(s->color[0], 8 * colour, 8);
    for (int r = y; r < y + h; r++) {
        dbox_run_bytes(s->yb + (uint64_t)r * ystride + x, w, pat, 1, false);
    }
    if (s->ctrl & CTRL_UV_MASK) {
        return;
    }
    pat[0] = extract32(s->color[1], 8 * colour, 8);
    pat[1] = extract32(s->color[2], 8 * colour, 8);
    for (int r = y; r < y + h; r += 2) {
        dbox_run_bytes(s->cb + (uint64_t)(r / 2) * cstride + x, w, pat, 2,
                       split);
    }
}

/* A part of a rectangle, kept to the box */
static void dbox_part(IngenicDrawboxState *s, int bx, int by, int bw, int bh,
                      int x, int y, int w, int h, unsigned colour, bool split)
{
    int x1 = MIN(x + w, bx + bw), y1 = MIN(y + h, by + bh);

    x = MAX(x, bx);
    y = MAX(y, by);
    dbox_rect(s, x, y, x1 - x, y1 - y, colour, split);
}

static void dbox_draw(IngenicDrawboxState *s, uint32_t w0, uint32_t w1)
{
    int x = extract32(w0, 0, 12), y = extract32(w0, 12, 12);
    int w = extract32(w0, 24, 8) | extract32(w1, 0, 4) << 8;
    int h = extract32(w1, 4, 12);
    unsigned colour = extract32(w1, 16, 2), mode = extract32(w1, 18, 2);
    int t = 2 * (extract32(w1, 20, 3) + 1), len = extract32(w1, 23, 9);
    int a = MIN(t, len);

    switch (mode) {
    case 0:
        /* the sides, then the top and bottom, each laid in two halves */
        dbox_part(s, x, y, w, h, x, y, t, h, colour, false);
        dbox_part(s, x, y, w, h, x + w - t, y, t, h, colour, false);
        dbox_part(s, x, y, w, h, x, y, w, t, colour, true);
        dbox_part(s, x, y, w, h, x, y + h - t, w, t, colour, true);
        break;
    case 1:
        if (!len) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-drawbox: corners with a "
                          "line length of 0, which silicon draws through "
                          "memory\n");
            break;
        }
        if (len > w || len > h) {
            break;
        }
        dbox_rect(s, x, y, len, a, colour, false);
        dbox_rect(s, x, y, a, len, colour, false);
        dbox_rect(s, x + w - len, y, len, a, colour, false);
        dbox_rect(s, x + w - a, y, a, len, colour, false);
        dbox_rect(s, x, y + h - a, len, a, colour, false);
        dbox_rect(s, x, y + h - len, a, len, colour, false);
        dbox_rect(s, x + w - len, y + h - a, len, a, colour, false);
        dbox_rect(s, x + w - a, y + h - len, a, len, colour, false);
        break;
    case 2:
        dbox_rect(s, x, y, w, t, colour, false);
        break;
    default:
        dbox_rect(s, x, y, t, h, colour, false);
        break;
    }
}

static void dbox_run(void *opaque)
{
    IngenicDrawboxState *s = opaque;
    unsigned boxes = s->ram_words / 2;

    if (!(s->ctrl & CTRL_START)) {
        return;
    }
    for (unsigned i = 0; i < boxes; i++) {
        dbox_draw(s, s->ram[2 * i], s->ram[2 * i + 1]);
    }
    s->ctrl = deposit32(s->ctrl, 17, 8, boxes) & ~CTRL_START;
    if (!(s->ctrl & CTRL_IRQ_MASK)) {
        s->ctrl |= CTRL_DONE;
    }
    dbox_update_irq(s);
}

static uint64_t dbox_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicDrawboxState *s = opaque;

    switch (addr) {
    case DBOX_CTRL:
        return deposit32(s->ctrl, 9, 8, s->ram_words / 2);
    case DBOX_YB:
        return s->yb;
    case DBOX_CB:
        return s->cb;
    case DBOX_STRIDE:
        return s->stride;
    case DBOX_IMG_WH:
        return s->img_wh;
    case DBOX_COLOR_Y:
    case DBOX_COLOR_U:
    case DBOX_COLOR_V:
        return s->color[(addr - DBOX_COLOR_Y) / 4];
    case DBOX_TIMEOUT:
        return s->timeout;
    default:
        return 0;
    }
}

static void dbox_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IngenicDrawboxState *s = opaque;

    switch (addr) {
    case DBOX_CTRL:
        if (val & CTRL_RESET) {
            s->ram_words = 0;
            s->ctrl &= ~(CTRL_START | CTRL_DONE | MAKE_64BIT_MASK(17, 8));
        }
        s->ctrl = (s->ctrl & ~(CTRL_RW | CTRL_DONE)) |
                  (val & (CTRL_RW | CTRL_DONE));
        if (val & CTRL_START) {
            s->ctrl |= CTRL_START;
            qemu_bh_schedule(s->bh);
        }
        dbox_update_irq(s);
        break;
    case DBOX_YB:
        s->yb = val;
        break;
    case DBOX_CB:
        s->cb = val;
        break;
    case DBOX_STRIDE:
        s->stride = val;
        break;
    case DBOX_IMG_WH:
        s->img_wh = val;
        break;
    case DBOX_COLOR_Y:
    case DBOX_COLOR_U:
    case DBOX_COLOR_V:
        s->color[(addr - DBOX_COLOR_Y) / 4] = val;
        break;
    case DBOX_RAM:
        if (s->ram_words == ARRAY_SIZE(s->ram)) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-drawbox: box RAM full\n");
            break;
        }
        s->ram[s->ram_words++] = val;
        break;
    case DBOX_TIMEOUT:
        s->timeout = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-drawbox: write to 0x%"
                      HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps dbox_ops = {
    .read = dbox_read,
    .write = dbox_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void dbox_reset_hold(Object *obj, ResetType type)
{
    IngenicDrawboxState *s = INGENIC_DRAWBOX(obj);

    s->ctrl = 0;
    s->yb = s->cb = s->stride = s->img_wh = s->timeout = 0;
    memset(s->color, 0, sizeof(s->color));
    s->ram_words = 0;
    qemu_set_irq(s->irq, 0);
}

/* A list started and not yet drawn */
static int dbox_post_load(void *opaque, int version_id)
{
    IngenicDrawboxState *s = opaque;

    if (s->ctrl & CTRL_START) {
        qemu_bh_schedule(s->bh);
    }
    return 0;
}

static const VMStateDescription vmstate_dbox = {
    .name = TYPE_INGENIC_DRAWBOX,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dbox_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IngenicDrawboxState),
        VMSTATE_UINT32(yb, IngenicDrawboxState),
        VMSTATE_UINT32(cb, IngenicDrawboxState),
        VMSTATE_UINT32(stride, IngenicDrawboxState),
        VMSTATE_UINT32(img_wh, IngenicDrawboxState),
        VMSTATE_UINT32_ARRAY(color, IngenicDrawboxState, 3),
        VMSTATE_UINT32(timeout, IngenicDrawboxState),
        VMSTATE_UINT32_ARRAY(ram, IngenicDrawboxState,
                             2 * INGENIC_DRAWBOX_BOXES),
        VMSTATE_UINT32(ram_words, IngenicDrawboxState),
        VMSTATE_END_OF_LIST()
    }
};

static void dbox_init(Object *obj)
{
    IngenicDrawboxState *s = INGENIC_DRAWBOX(obj);

    memory_region_init_io(&s->iomem, obj, &dbox_ops, s, TYPE_INGENIC_DRAWBOX,
                          0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bh = qemu_bh_new(dbox_run, s);
}

static void dbox_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->vmsd = &vmstate_dbox;
    rc->phases.hold = dbox_reset_hold;
}

static const TypeInfo dbox_info = {
    .name = TYPE_INGENIC_DRAWBOX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDrawboxState),
    .instance_init = dbox_init,
    .class_init = dbox_class_init,
};

static void dbox_register_types(void)
{
    type_register_static(&dbox_info);
}

type_init(dbox_register_types)
