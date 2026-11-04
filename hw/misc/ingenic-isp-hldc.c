/*
 * Ingenic T41 ISP: the horizontal lens distortion correction (HLDC)
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
 *   0x2c to 0x40 are read-only state, the model's an idle block's.
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
 * Not modelled: what 0x00, 0x24 and 0x28 do, the state 0x2c to 0x40 shows
 * while a frame is going through, and the reset values, which a T41NQ
 * with tx-isp unloaded did not let us read (the model's are 0).
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

#define HLDC_BACK           (1 << 13)   /* 1/8 pixel, in Q16 */

static const uint32_t hldc_mask[2][INGENIC_HLDC_FIELDS] = {
    { 0x03ff03ff, 0x00000fff, 0x00001fff, 0x00ff00ff, 0x0000ffff },
    { 0x03ff03ff, 0x00000fff, 0x00007fff, 0x007f007f, 0x0000ffff },
};

/* The read-only state an idle T41NQ's block shows, from 0x2c */
static const uint32_t hldc_state[] = { 0x00080000, 0x00010008 };

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

static bool hldc_plane_stalls(const HldcPlane *p)
{
    int64_t last = (int64_t)(p->w - 1) << 16;

    for (uint32_t y = 0; y < p->h; y++) {
        int64_t far = 0;

        for (uint32_t x = 0; x < p->w; x++) {
            int64_t s = hldc_pos(p, x, y);

            if (s < 0 || s > last || s < far - HLDC_BACK) {
                return true;
            }
            far = MAX(far, s);
        }
    }
    return false;
}

static void hldc_plane_remap(const HldcPlane *p, uint8_t *buf,
                             uint32_t stride, unsigned bpp)
{
    g_autofree uint8_t *row = g_malloc(p->w * bpp);
    int64_t last = (int64_t)(p->w - 1) << 16;

    for (uint32_t y = 0; y < p->h; y++) {
        uint8_t *dst = buf + (size_t)y * stride;

        memcpy(row, dst, p->w * bpp);
        for (uint32_t x = 0; x < p->w; x++) {
            int64_t s = MIN(MAX(hldc_pos(p, x, y), 0), last);
            uint32_t i = s >> 16, j = MIN(i + 1, p->w - 1);
            unsigned f = (s >> 8) & 0xff;

            for (unsigned b = 0; b < bpp; b++) {
                dst[x * bpp + b] = (row[i * bpp + b] * (256 - f) +
                                    row[j * bpp + b] * f) >> 8;
            }
        }
    }
}

void ingenic_hldc_reset(IngenicIspHldc *h)
{
    memset(h, 0, sizeof(*h));
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
    switch (reg) {
    case 0x2c:
    case 0x38:
        return hldc_state[0];
    case 0x30:
    case 0x3c:
        return hldc_state[1];
    case 0x34:
        return ht << 16;
    case 0x40:
        return ht / 2 << 16;
    case HLDC_LATCH:
        return h->latch & 1;
    }
    return 0;
}

void ingenic_hldc_write(IngenicIspHldc *h, hwaddr reg, uint32_t v)
{
    if (reg == HLDC_CTRL) {
        h->ctrl = v & 0x00010001;
    } else if (reg >= HLDC_FIRST && reg <= HLDC_LAST) {
        unsigned i = (reg - HLDC_FIRST) / 4;

        h->pend[i & 1][i >> 1] = v & hldc_mask[i & 1][i >> 1];
    } else if (reg == HLDC_LATCH) {
        h->latch = v & 3;
    }
}

void ingenic_hldc_frame_start(IngenicIspHldc *h)
{
    for (unsigned c = 0; c < 2; c++) {
        if (h->latch & BIT(c)) {
            memcpy(h->act[c], h->pend[c], sizeof(h->act[c]));
        }
    }
    h->latch = 0;
}

bool ingenic_hldc_stalls(IngenicIspHldc *h, uint32_t w, uint32_t ht)
{
    HldcPlane p;

    if (h->stuck) {
        return true;
    }
    if ((h->act[0][4] | h->act[1][4]) & BIT(15)) {
        h->stuck = true;
        return true;
    }
    if (!h->cache_valid || h->cache_w != w || h->cache_h != ht ||
        memcmp(h->cache_act, h->act, sizeof(h->act))) {
        hldc_plane(h, 0, w, ht, &p);
        h->cache_stalls = hldc_plane_stalls(&p);
        if (!h->cache_stalls) {
            hldc_plane(h, 1, w / 2, ht / 2, &p);
            h->cache_stalls = hldc_plane_stalls(&p);
        }
        memcpy(h->cache_act, h->act, sizeof(h->act));
        h->cache_w = w;
        h->cache_h = ht;
        h->cache_valid = true;
    }
    h->stuck = h->cache_stalls;
    return h->stuck;
}

void ingenic_hldc_remap(const IngenicIspHldc *h, uint8_t *luma, uint8_t *uv,
                        uint32_t w, uint32_t ht)
{
    HldcPlane p;

    hldc_plane(h, 0, w, ht, &p);
    hldc_plane_remap(&p, luma, w, 1);
    hldc_plane(h, 1, w / 2, ht / 2, &p);
    hldc_plane_remap(&p, uv, w, 2);
}

const VMStateDescription vmstate_ingenic_isp_hldc = {
    .name = "ingenic-isp-hldc",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IngenicIspHldc),
        VMSTATE_UINT32_2DARRAY(pend, IngenicIspHldc, 2, INGENIC_HLDC_FIELDS),
        VMSTATE_UINT32_2DARRAY(act, IngenicIspHldc, 2, INGENIC_HLDC_FIELDS),
        VMSTATE_UINT32(latch, IngenicIspHldc),
        VMSTATE_BOOL(stuck, IngenicIspHldc),
        VMSTATE_END_OF_LIST()
    }
};
