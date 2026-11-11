/*
 * Ingenic ISP: the horizontal lens distortion correction (HLDC)
 *
 * The block at 0x15000 of the T41's ISP moves each picture row's pixels
 * along the row, by a factor that falls with the distance from a centre:
 * a barrel correction that keeps every row in its place. As a T41NQ takes
 * it, the ISP's 0x40 bit 20 bypassing it (tx-isp sets it):
 *
 * - 0x04 and 0x08: the luma and the chroma plane's k1 (bits 9:0) and k2
 *   (25:16), unsigned Q10; 0x0c and 0x10: their zoom, 12 bits Q11; 0x14
 *   and 0x18: their norm (13 and 15 bits), 2^30 over the squared distance
 *   to the far corner; 0x1c and 0x20: their centre's offset from the
 *   plane's middle, x in the low half and y in the high, signed 8 and 7
 *   bits; 0x24 and 0x28: 16 bits of their own (low values change nothing
 *   seen, bit 15 set in both stopped the ISP; the model stops it for bit
 *   15 in either). These read back the settings in use: writes take effect
 *   at the next frame start once 0x44 bit 0 (luma) or bit 1 (chroma) asks
 *   for them. Bit 0 reads back set until then, bit 1 reads 0; a write of 0
 *   at once after read 0, which the model takes as a request withdrawn.
 *   0x00 keeps bits 16 and 0 (tx-isp sets both) and changes nothing seen;
 *   0x2c to 0x40 are read-only state: out of reset, 0x30 and 0x3c read
 *   0x00010001 and 0x34 and 0x40 0; once frames have gone through, the
 *   model's are an idle block's.
 *
 * - Out of reset (at power-on and through the ISP's soft reset alike) 0x00
 *   reads 0x00010000, the zooms 0x800, 0x24 0x14 and 0x28 0xa, the rest of
 *   the settings 0: no correction.
 *
 * - A pixel d columns right of the centre (x - cx; on the left cx - 1 - x,
 *   both sides counting from 0) and e rows from it, likewise, takes the
 *   picture at d * zoom * (1 - k1 * r + k2 * r * r) further out on its
 *   side, r = (d * d + e * e) * norm / 2^30: as tisp_hldc_calc_para works
 *   its zoom out, and as positions fitted to a T41NQ's frames put it to
 *   within 0.005 pixel. r keeps only its fraction: past the distance where
 *   it reaches 1 it starts again from 0, as the T41NQ showed with a norm
 *   too large for its frame (tx-isp's own for 1080p, on a 2560 x 1440
 *   frame, stops it so). The chroma plane does the same at half size, U
 *   and V alike. Between two pixels the result is interpolated and cut
 *   down to an integer: a T41NQ's frames come out half a level darker on
 *   average with the stage on, and unchanged when every pixel takes its
 *   own. The model works positions to 1/65536 pixel and the weights to
 *   1/256, its own choice where averages of noisy frames show no step.
 *
 * - A setting that takes a pixel from outside its row, or from more than
 *   1/8 pixel behind one already taken along it (a T41NQ ran 0.10 pixel
 *   back and stopped at 0.13), stops the ISP until it is reset; on the
 *   T41NQ writes to its bypass did not restart it.
 *
 * - The T21's, T23's, T31's and T40's ISPs have the same registers, from
 *   tx-isp's use of them: the T23 and T31 at 0x9000 and the T40 at 0x15000
 *   (and 0x15080 for its second pipeline) ask for new settings through 0x44
 *   as the T41 does, the T21 at 0x2200 (no 0x44) through bits 0 (luma) and 4
 *   (chroma) of 0x00, which also holds the fields tx-isp packs from bit 8.
 *   Measured on a T31X and a T40N: the settings read the active copy, a
 *   write shows only once 0x44 asks and a frame starts, which clears 0x44
 *   (the T31 reads both its bits back, the T40 and T41 bit 0); 0x00 keeps
 *   0x00010001; the T40's second block resets as the T41's; the T31's state
 *   words differ from the T41's. Not measured: the T21 and T23, and the
 *   T21's latch, taken from the drivers.
 *
 * Not modelled: what 0x00, 0x24 and 0x28 do, and the state 0x2c to 0x40
 * shows while a frame is going through.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "hw/misc/ingenic-isp-hldc.h"

#define HLDC_CTRL           0x00
#define HLDC_FIRST          0x04
#define HLDC_LAST           0x28
#define HLDC_LATCH          0x44

/*
 * The T21's 0x00 holds what the others' 0x44 asks for: bits 0 and 4 as the
 * luma's and the chroma's, and tx-isp's four 4-bit fields from bit 8
 */
#define HLDC_T21_CTRL       0x00ffff13

#define HLDC_BACK           (1 << 13)   /* 1/8 pixel, in Q16 */

static const uint32_t hldc_mask[2][INGENIC_HLDC_FIELDS] = {
    { 0x03ff03ff, 0x00000fff, 0x00001fff, 0x00ff00ff, 0x0000ffff },
    { 0x03ff03ff, 0x00000fff, 0x00007fff, 0x007f007f, 0x0000ffff },
};

/* The read-only state an idle T41NQ's block shows, from 0x2c */
static const uint32_t hldc_state[] = { 0x00080000, 0x00010008 };

/* Its settings out of reset: k, zoom, norm, centre, 0x24 (luma; chroma) */
static const uint32_t hldc_reset[2][INGENIC_HLDC_FIELDS] = {
    { 0, 0x800, 0, 0, 0x14 },
    { 0, 0x800, 0, 0, 0x0a },
};

typedef struct HldcPlane {
    int64_t k1, k2, zoom, norm;
    int32_t cx, cy;
    uint32_t w, h;
} HldcPlane;

static void hldc_plane(const IngenicIspHldc *h, unsigned c, uint32_t w,
                       uint32_t ht, HldcPlane *p)
{
    const uint32_t *a = h->act[c];
    unsigned bits = c ? 7 : 8;

    p->k1 = extract32(a[0], 0, 10);
    p->k2 = extract32(a[0], 16, 10);
    p->zoom = a[1];
    p->norm = a[2];
    p->w = w;
    p->h = ht;
    p->cx = (int32_t)(w / 2) + sextract32(a[3], 0, bits);
    p->cy = (int32_t)(ht / 2) + sextract32(a[3], 16, bits);
}

/* Where along row y pixel x takes the picture from, in Q16 */
static int64_t hldc_pos(const HldcPlane *p, int32_t x, int32_t y)
{
    int64_t d = x >= p->cx ? x - p->cx : p->cx - 1 - x;
    int64_t e = y >= p->cy ? y - p->cy : p->cy - 1 - y;
    /* Q30, its integer bits dropped */
    uint64_t r = (uint64_t)(d * d + e * e) * p->norm & MAKE_64BIT_MASK(0, 30);
    uint64_t rq = r >> 10;
    int64_t f, off;

    /* Q40 */
    f = (INT64_C(1) << 40) - p->k1 * (int64_t)r +
        p->k2 * (int64_t)((rq * rq) >> 10);
    off = d * ((f * p->zoom) >> 35);
    return x >= p->cx ? ((int64_t)p->cx << 16) + off
                      : ((int64_t)(p->cx - 1) << 16) - off;
}

/*
 * A plane's source positions (Q8, clamped into the row) for one set of
 * settings and one size, worked once and reused by every frame after: the
 * positions depend on nothing in the picture. @stalls is what the unclamped
 * positions say; @ident[y] marks a row that takes every pixel from itself.
 */
typedef struct HldcTable {
    HldcPlane plane;
    bool stalls;
    uint8_t *ident;
    uint32_t *pos;
} HldcTable;

static void hldc_table_free(HldcTable *t)
{
    if (t) {
        g_free(t->ident);
        g_free(t->pos);
        g_free(t);
    }
}

static HldcTable *hldc_table_build(const HldcPlane *p)
{
    HldcTable *t = g_new0(HldcTable, 1);
    int64_t last = (int64_t)(p->w - 1) << 16;

    t->plane = *p;
    t->pos = g_new(uint32_t, (size_t)p->w * p->h);
    t->ident = g_new(uint8_t, p->h);
    for (uint32_t y = 0; y < p->h; y++) {
        uint32_t *row = t->pos + (size_t)y * p->w;
        int64_t furthest = 0;
        bool ident = true;

        for (uint32_t x = 0; x < p->w; x++) {
            int64_t s = hldc_pos(p, x, y);

            if (s < 0 || s > last || s < furthest - HLDC_BACK) {
                t->stalls = true;
            }
            furthest = MAX(furthest, s);
            row[x] = MIN(MAX(s, 0), last) >> 8;
            ident = ident && row[x] == x << 8;
        }
        t->ident[y] = ident;
    }
    return t;
}

/* The table for @p; the last few are kept, a frame can use several sizes */
static HldcTable *hldc_table(IngenicIspHldc *h, const HldcPlane *p)
{
    for (unsigned i = 0; i < INGENIC_HLDC_TABLES; i++) {
        HldcTable *t = h->tab[i];

        if (t && !memcmp(&t->plane, p, sizeof(*p))) {
            return t;
        }
    }
    unsigned at = h->tab_next;

    hldc_table_free(h->tab[at]);
    h->tab[at] = hldc_table_build(p);
    h->tab_next = (at + 1) % INGENIC_HLDC_TABLES;
    return h->tab[at];
}

/*
 * The row is padded by one pixel so the next one can be read without a
 * test: it carries weight 0 where the position is the last pixel's.
 */
static void hldc_plane_remap(const HldcTable *t, uint8_t *buf,
                             uint32_t stride, unsigned bpp)
{
    const HldcPlane *p = &t->plane;
    g_autofree uint8_t *row = NULL;

    if (!p->w) {
        return;
    }
    row = g_malloc((p->w + 1) * bpp);

    for (uint32_t y = 0; y < p->h; y++) {
        uint8_t *dst = buf + (size_t)y * stride;
        const uint32_t *pos = t->pos + (size_t)y * p->w;

        if (t->ident[y]) {
            continue;
        }
        memcpy(row, dst, p->w * bpp);
        memcpy(row + p->w * bpp, row + (p->w - 1) * bpp, bpp);
        if (bpp == 1) {
            for (uint32_t x = 0; x < p->w; x++) {
                uint32_t i = pos[x] >> 8, f = pos[x] & 0xff;

                dst[x] = ((row[i] << 8) + (row[i + 1] - row[i]) * (int)f) >> 8;
            }
        } else {
            for (uint32_t x = 0; x < p->w; x++) {
                uint32_t i = (pos[x] >> 8) * 2, f = pos[x] & 0xff;

                dst[2 * x] = ((row[i] << 8) +
                              (row[i + 2] - row[i]) * (int)f) >> 8;
                dst[2 * x + 1] = ((row[i + 1] << 8) +
                                  (row[i + 3] - row[i + 1]) * (int)f) >> 8;
            }
        }
    }
}

void ingenic_hldc_free(IngenicIspHldc *h)
{
    for (unsigned i = 0; i < INGENIC_HLDC_TABLES; i++) {
        hldc_table_free(h->tab[i]);
        h->tab[i] = NULL;
    }
}

void ingenic_hldc_reset(IngenicIspHldc *h, bool ctl_latch, bool wide)
{
    ingenic_hldc_free(h);
    memset(h, 0, sizeof(*h));
    h->ctl_latch = ctl_latch;
    h->wide = wide;
    h->ctrl = 0x00010000;
    memcpy(h->pend, hldc_reset, sizeof(h->pend));
    memcpy(h->act, hldc_reset, sizeof(h->act));
}

uint32_t ingenic_hldc_read(IngenicIspHldc *h, hwaddr reg, uint32_t w,
                           uint32_t ht)
{
    if (reg == HLDC_CTRL) {
        return h->ctrl;
    }
    if (reg >= HLDC_FIRST && reg <= HLDC_LAST) {
        unsigned i = (reg - HLDC_FIRST) / 4;

        return h->act[i & 1][i >> 1];
    }
    if (h->wide) {
        /* a T31X's: out of reset as read in U-Boot (uncertain), then idle */
        switch (reg) {
        case 0x30:
        case 0x3c:
            return 0x01000800;
        case 0x34:
        case 0x40:
            return h->ran ? 8 : 1;
        case 0x38:
            return h->ran ? 0x08000000 : 0;
        case HLDC_LATCH:
            return h->latch;
        }
        return 0;
    }
    switch (reg) {
    case 0x2c:
    case 0x38:
        return hldc_state[0];
    case 0x30:
    case 0x3c:
        return h->ran ? hldc_state[1] : 0x00010001;
    case 0x34:
        return h->ran ? ht << 16 : 0;
    case 0x40:
        return h->ran ? ht / 2 << 16 : 0;
    case HLDC_LATCH:
        return h->latch & 1;
    }
    return 0;
}

void ingenic_hldc_write(IngenicIspHldc *h, hwaddr reg, uint32_t v)
{
    if (reg == HLDC_CTRL) {
        h->ctrl = v & (h->ctl_latch ? HLDC_T21_CTRL : 0x00010001);
    } else if (reg >= HLDC_FIRST && reg <= HLDC_LAST) {
        unsigned i = (reg - HLDC_FIRST) / 4;

        h->pend[i & 1][i >> 1] = v & hldc_mask[i & 1][i >> 1];
    } else if (reg == HLDC_LATCH) {
        h->latch = v & 3;
    }
}

void ingenic_hldc_frame_start(IngenicIspHldc *h)
{
    uint32_t asked = h->ctl_latch ? (h->ctrl & 1) | (h->ctrl >> 4 & 1) << 1
                                  : h->latch;

    for (unsigned c = 0; c < 2; c++) {
        if (asked & BIT(c)) {
            memcpy(h->act[c], h->pend[c], sizeof(h->act[c]));
        }
    }
    h->latch = 0;
    h->ran = true;
}

bool ingenic_hldc_stalls(IngenicIspHldc *h, uint32_t w, uint32_t ht)
{
    HldcPlane p;
    bool stalls;

    if (h->stuck) {
        return true;
    }
    if ((h->act[0][4] | h->act[1][4]) & BIT(15)) {
        h->stuck = true;
        return true;
    }
    hldc_plane(h, 0, w, ht, &p);
    stalls = hldc_table(h, &p)->stalls;
    if (!stalls) {
        hldc_plane(h, 1, w / 2, ht / 2, &p);
        stalls = hldc_table(h, &p)->stalls;
    }
    h->stuck = stalls;
    return h->stuck;
}

void ingenic_hldc_remap(IngenicIspHldc *h, uint8_t *luma, uint8_t *uv,
                        uint32_t w, uint32_t ht)
{
    HldcPlane p;

    hldc_plane(h, 0, w, ht, &p);
    hldc_plane_remap(hldc_table(h, &p), luma, w, 1);
    hldc_plane(h, 1, w / 2, ht / 2, &p);
    hldc_plane_remap(hldc_table(h, &p), uv, w, 2);
}

const VMStateDescription vmstate_ingenic_isp_hldc = {
    .name = "ingenic-isp-hldc",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IngenicIspHldc),
        VMSTATE_UINT32_2DARRAY(pend, IngenicIspHldc, 2, INGENIC_HLDC_FIELDS),
        VMSTATE_UINT32_2DARRAY(act, IngenicIspHldc, 2, INGENIC_HLDC_FIELDS),
        VMSTATE_UINT32(latch, IngenicIspHldc),
        VMSTATE_BOOL(stuck, IngenicIspHldc),
        VMSTATE_BOOL(ran, IngenicIspHldc),
        VMSTATE_END_OF_LIST()
    }
};
