/*
 * Ingenic DRAW_BOX (T40, T41)
 *
 * Draws up to 255 boxes into an NV12 picture in memory. Each box is two
 * words written to the RAM register: x (11:0), y (23:12) and width bits
 * 7:0 (31:24); width bits 11:8 (3:0), height (15:4), colour (17:16),
 * mode (19:18), line width (22:20) and line length (31:23). CTRL bits
 * 16:9 count the boxes written since a reset, bits 24:17 the boxes drawn.
 * Starting (CTRL bit 0) draws them all, then sets bit 4, which raises the
 * interrupt unless bit 3 masks it; writing bit 4 as 0 clears it. Reset
 * (bit 1) empties the RAM, done at once.
 *
 * As a T41NQ draws, byte for byte: a box covers x..x+w-1, y..y+h-1 and
 * its lines are 2 * (line width + 1) pixels thick, inward. Mode 0 draws
 * the four sides; mode 1 an arm of line-length pixels along both sides
 * from each corner, no thicker than that length; mode 2 a horizontal line
 * of the thickness from (x, y), w long, however tall the box; mode 3 a
 * vertical one, h long. A drawn pixel takes the colour's byte from
 * COLOR_Y and its 2x2 block takes the colour's U and V. Boxes are drawn
 * in order.
 *
 * Silicon does not keep a box to the picture: one past its edge, and mode
 * 1 with a line length of 0 (whose arms run on through memory), wrote far
 * outside the buffer and corrupted the kernel. The model logs those and
 * draws only what lies in the picture, and nothing for the second.
 * Not modelled: BGRA pictures (CTRL bit 6; silicon writes words this
 * model cannot yet explain), the UV mask (bit 2), the timeout and the
 * time a job takes.
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

static void dbox_fill_rows(uint32_t base, uint32_t stride, int x0, int y0,
                           int w, int h, const uint8_t *pat)
{
    g_autofree uint8_t *row = g_malloc(w);

    for (int i = 0; i < w; i++) {
        row[i] = pat[i & 1];
    }
    for (int y = y0; y < y0 + h; y++) {
        address_space_write(&address_space_memory,
                            base + (uint64_t)y * stride + x0,
                            MEMTXATTRS_UNSPECIFIED, row, w);
    }
}

/* A filled rectangle of the picture, its luma and the chroma under it */
static void dbox_rect(IngenicDrawboxState *s, int x, int y, int w, int h,
                      unsigned colour)
{
    int pw = extract32(s->img_wh, 0, 16), ph = extract32(s->img_wh, 16, 16);
    uint8_t luma[2], chroma[2];
    int ux0, ux1;

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
    luma[0] = luma[1] = extract32(s->color[0], 8 * colour, 8);
    chroma[0] = extract32(s->color[1], 8 * colour, 8);
    chroma[1] = extract32(s->color[2], 8 * colour, 8);
    dbox_fill_rows(s->yb, extract32(s->stride, 0, 16), x, y, w, h, luma);
    ux0 = x & ~1;
    ux1 = ((x + w - 1) & ~1) + 2;
    for (int r = y / 2; r <= (y + h - 1) / 2; r++) {
        dbox_fill_rows(s->cb, extract32(s->stride, 16, 16), ux0, r,
                       ux1 - ux0, 1, chroma);
    }
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
        dbox_rect(s, x, y, w, t, colour);
        dbox_rect(s, x, y + h - t, w, t, colour);
        dbox_rect(s, x, y, t, h, colour);
        dbox_rect(s, x + w - t, y, t, h, colour);
        break;
    case 1:
        if (!len) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-drawbox: corners with a "
                          "line length of 0, which silicon draws through "
                          "memory\n");
            break;
        }
        dbox_rect(s, x, y, len, a, colour);
        dbox_rect(s, x, y, a, len, colour);
        dbox_rect(s, x + w - len, y, len, a, colour);
        dbox_rect(s, x + w - a, y, a, len, colour);
        dbox_rect(s, x, y + h - a, len, a, colour);
        dbox_rect(s, x, y + h - len, a, len, colour);
        dbox_rect(s, x + w - len, y + h - a, len, a, colour);
        dbox_rect(s, x + w - a, y + h - len, a, len, colour);
        break;
    case 2:
        dbox_rect(s, x, y, w, t, colour);
        break;
    default:
        dbox_rect(s, x, y, t, h, colour);
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
    if (s->ctrl & (CTRL_BGRA | CTRL_UV_MASK)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-drawbox: BGRA pictures and the UV "
                      "mask are not modelled\n");
    } else {
        for (unsigned i = 0; i < boxes; i++) {
            dbox_draw(s, s->ram[2 * i], s->ram[2 * i + 1]);
        }
    }
    s->ctrl = deposit32(s->ctrl, 17, 8, boxes);
    s->ctrl = (s->ctrl & ~CTRL_START) | CTRL_DONE;
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
        s->ctrl = (s->ctrl & ~CTRL_RW) | (val & CTRL_RW);
        if (!(val & CTRL_DONE)) {
            s->ctrl &= ~CTRL_DONE;
        }
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

static const VMStateDescription vmstate_dbox = {
    .name = TYPE_INGENIC_DRAWBOX,
    .version_id = 1,
    .minimum_version_id = 1,
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
