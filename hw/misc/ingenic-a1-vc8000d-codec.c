/*
 * Ingenic A1 VC8000D: the picture decoding behind the register model
 *
 * The software stack (VeriSilicon's DWL, in libimp) programs one picture
 * per run: the stream buffer and the bit the data starts at, the
 * picture's size and its output and reference addresses. The hardware
 * walks the NAL units from there, decodes every slice of the picture,
 * writes it to the output address and leaves its stream position in the
 * stream address register. This file does the same with libavcodec when
 * it is built in, and with a mid-grey picture when it is not.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "migration/blocker.h"
#include <zlib.h>
#include "system/address-spaces.h"
#include "ingenic-a1-vc8000d-codec.h"
#ifdef CONFIG_INGENIC_VDEC_AVCODEC
#include "ingenic-a1-vc8000d-avc.h"
#endif
#include "trace.h"

/* decoding modes, swreg3[31:27] */
#define MODE_H264       0x0f
#define MODE_HEVC       0x0c

/* swreg1 completion flags */
#define DEC_IRQ         BIT(8)
#define DEC_RDY         BIT(12)
#define DEC_BUF_EMPTY   BIT(14)

/* H.264 job registers, in words */
#define R_MODE          3
#define R_PICSIZE       4       /* [31:19] width/8, [18:6] height/8 */
#define R_STRM_BIT      5       /* [31:25] start bit */
#define R_STRM_LEN      6
#define R_OUT           0x41    /* the picture's luma base */
#define R_OUT_CHROMA    0x63
#define R_STRM_BASE     0xa9    /* the ring's base, then the read position */
#define R_RING_SIZE     0x102
#define R_RING_OFFSET   0x103

#define MAX_STREAM      (8 * MiB)
#define MAX_DIM         4096

/* A stream's decoder state; the stream is found by its ring's address */
typedef struct StreamCtx {
    uint32_t base;
#ifdef CONFIG_INGENIC_VDEC_AVCODEC
    Vc8000dAvc *avc;
#endif
    GByteArray *ps[3];  /* VPS (HEVC), SPS, PPS */
    bool hevc;
    uint64_t used;
} StreamCtx;

#define MAX_STREAMS     32
#define LOOKBACK        8192

struct Vc8000dCodec {
    IngenicA1Vc8000dState *s;
    StreamCtx ctx[MAX_STREAMS];
    uint64_t clock;
    Error *blocker;     /* libavcodec holds the pictures, not guest RAM */
};

typedef struct Nal {
    size_t start;       /* of the 00 00 [00] 01 prefix */
    size_t data;        /* first byte after the prefix: the NAL header */
    size_t end;         /* where the next prefix starts, or the buffer's end */
} Nal;

Vc8000dCodec *vc8000d_codec_new(IngenicA1Vc8000dState *s)
{
    Vc8000dCodec *c = g_new0(Vc8000dCodec, 1);

    c->s = s;
    return c;
}

void vc8000d_codec_reset(Vc8000dCodec *c)
{
    unsigned i, j;

    for (i = 0; i < MAX_STREAMS; i++) {
        StreamCtx *x = &c->ctx[i];

#ifdef CONFIG_INGENIC_VDEC_AVCODEC
        vc8000d_avc_close(x->avc);
        x->avc = NULL;
#endif
        for (j = 0; j < 3; j++) {
            if (x->ps[j]) {
                g_byte_array_set_size(x->ps[j], 0);
            }
        }
        x->used = 0;
    }
    if (c->blocker) {
        migrate_del_blocker(&c->blocker);
    }
}

static StreamCtx *ctx_get(Vc8000dCodec *c, uint32_t base)
{
    StreamCtx *x, *old = &c->ctx[0];
    unsigned i;

    for (i = 0; i < MAX_STREAMS; i++) {
        x = &c->ctx[i];
        if (x->ps[0] && x->base == base) {
            x->used = ++c->clock;
            return x;
        }
        if (x->used < old->used) {
            old = x;
        }
    }
    x = old;
#ifdef CONFIG_INGENIC_VDEC_AVCODEC
    vc8000d_avc_close(x->avc);
    x->avc = NULL;
#endif
    for (i = 0; i < 3; i++) {
        if (!x->ps[i]) {
            x->ps[i] = g_byte_array_new();
        }
        g_byte_array_set_size(x->ps[i], 0);
    }
    x->base = base;
    x->used = ++c->clock;
    return x;
}

/* n bytes of the ring from offset off, wrapping at its end */
static void ring_read(AddressSpace *as, uint32_t base, uint32_t ring,
                      uint32_t off, uint8_t *dst, size_t n)
{
    if (!ring) {
        address_space_read(as, base + off, MEMTXATTRS_UNSPECIFIED, dst, n);
        return;
    }
    off %= ring;
    if (off + n > ring) {
        uint32_t first = ring - off;

        address_space_read(as, base + off, MEMTXATTRS_UNSPECIFIED, dst, first);
        address_space_read(as, base, MEMTXATTRS_UNSPECIFIED, dst + first,
                           n - first);
    } else {
        address_space_read(as, base + off, MEMTXATTRS_UNSPECIFIED, dst, n);
    }
}

/* The next NAL's prefix at or after from; false when there is none */
static bool nal_next(const uint8_t *b, size_t len, size_t from, Nal *n)
{
    size_t i;

    for (i = from; i + 3 <= len; i++) {
        if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 1) {
            size_t j, st = i;

            while (st > from && b[st - 1] == 0) {
                st--;
            }
            n->start = st;
            n->data = i + 3;
            for (j = n->data; j + 3 <= len; j++) {
                if (b[j] == 0 && b[j + 1] == 0 && b[j + 2] == 1) {
                    break;
                }
            }
            n->end = j + 3 <= len ? j : len;
            while (n->end > n->data && b[n->end - 1] == 0 && j + 3 <= len) {
                n->end--;
            }
            n->end = n->end < n->data ? n->data : n->end;
            return true;
        }
    }
    return false;
}

/* first_mb_in_slice, ue(v), of the slice whose NAL header is at p */
static unsigned first_mb(const uint8_t *p, size_t avail)
{
    unsigned zeros = 0, v = 0, i;
    size_t bit = 0, nbits = avail * 8;

    while (bit < nbits && !((p[bit >> 3] >> (7 - (bit & 7))) & 1)) {
        zeros++;
        bit++;
    }
    if (zeros > 24 || bit + zeros >= nbits) {
        return 0;
    }
    bit++;
    for (i = 0; i < zeros; i++, bit++) {
        v = (v << 1) | ((p[bit >> 3] >> (7 - (bit & 7))) & 1);
    }
    return (1u << zeros) - 1 + v;
}

static void put_nal(GByteArray *a, const uint8_t *b, const Nal *n)
{
    static const uint8_t sc[4] = { 0, 0, 0, 1 };

    g_byte_array_append(a, sc, 4);
    g_byte_array_append(a, b + n->data, n->end - n->data);
}

static void fill_grey(IngenicA1Vc8000dState *s, uint32_t out,
                      uint32_t out_uv, unsigned w, unsigned h)
{
    size_t n = (size_t)w * h;
    uint8_t *g = g_malloc(n * 3 / 2);

    memset(g, 0x80, n * 3 / 2);
    address_space_write(s->as, out, MEMTXATTRS_UNSPECIFIED, g, n);
    address_space_write(s->as, out_uv, MEMTXATTRS_UNSPECIFIED, g, n / 2);
    vc8000d_pp_run(s, g, w, h);
    g_free(g);
}

#ifdef CONFIG_INGENIC_VDEC_AVCODEC
/* The picture to the output address, NV12, w x h of coded macroblocks */
static void write_nv12(IngenicA1Vc8000dState *s, uint32_t out,
                       uint32_t out_uv, unsigned w, unsigned h,
                       const Vc8000dPicture *pic)
{
    uint8_t *img = g_malloc0((size_t)w * h * 3 / 2);
    uint8_t *uv = img + (size_t)w * h;
    unsigned x, y, cw = MIN(w, pic->width), ch = MIN(h, pic->height);

    for (y = 0; y < ch; y++) {
        memcpy(img + (size_t)y * w, pic->data[0] + (size_t)y * pic->linesize[0],
               cw);
    }
    for (y = 0; y < ch / 2; y++) {
        const uint8_t *cb = pic->data[1] + (size_t)y * pic->linesize[1];
        const uint8_t *cr = pic->data[2] + (size_t)y * pic->linesize[2];

        for (x = 0; x < cw / 2; x++) {
            uv[(size_t)y * w + 2 * x] = cb[x];
            uv[(size_t)y * w + 2 * x + 1] = cr[x];
        }
    }
    address_space_write(s->as, out, MEMTXATTRS_UNSPECIFIED, img,
                        (size_t)w * h);
    address_space_write(s->as, out_uv, MEMTXATTRS_UNSPECIFIED, uv,
                        (size_t)w * h / 2);
    trace_ingenic_a1_vc8000d_pic(out, w, h, crc32(0, img, (size_t)w * h),
                                 crc32(0, uv, (size_t)w * h / 2));
    vc8000d_pp_run(s, img, w, h);
    g_free(img);
}
#endif

/* What differs between the two bitstreams' NAL units */
typedef struct NalKind {
    unsigned hdr;                               /* NAL header bytes */
    unsigned (*type)(const uint8_t *nal);
    bool (*is_slice)(unsigned type);
    int (*param)(unsigned type);                /* index into ps[], or -1 */
    bool (*first)(const uint8_t *nal, size_t avail);    /* starts a picture */
} NalKind;

static unsigned h264_type(const uint8_t *nal)
{
    return nal[0] & 0x1f;
}

static bool h264_is_slice(unsigned type)
{
    return type == 1 || type == 5;
}

static int h264_param(unsigned type)
{
    return type == 7 ? 1 : type == 8 ? 2 : -1;
}

static bool h264_first(const uint8_t *nal, size_t avail)
{
    return first_mb(nal + 1, avail - 1) == 0;
}

static unsigned hevc_type(const uint8_t *nal)
{
    return (nal[0] >> 1) & 0x3f;
}

static bool hevc_is_slice(unsigned type)
{
    return type < 32;
}

static int hevc_param(unsigned type)
{
    return type >= 32 && type <= 34 ? type - 32 : -1;
}

/* first_slice_segment_in_pic_flag, the first bit after the NAL header */
static bool hevc_first(const uint8_t *nal, size_t avail)
{
    return avail > 2 && (nal[2] & 0x80);
}

static const NalKind h264_kind = {
    1, h264_type, h264_is_slice, h264_param, h264_first,
};

static const NalKind hevc_kind = {
    2, hevc_type, hevc_is_slice, hevc_param, hevc_first,
};

/* The parameter sets of the AU the slices at off belong to */
static void learn_params(Vc8000dCodec *c, StreamCtx *x, const NalKind *k,
                         uint32_t base, uint32_t ring, uint32_t off)
{
    uint32_t back = ring ? MIN(LOOKBACK, ring) : MIN(LOOKBACK, off);
    uint8_t *b = g_malloc(back + 4);
    size_t cur = 0;
    Nal n;

    ring_read(c->s->as, base, ring, ring ? off + ring - back : off - back, b,
              back);
    while (nal_next(b, back, cur, &n)) {
        int p = n.data + k->hdr <= n.end ? k->param(k->type(b + n.data)) : -1;

        if (p >= 0) {
            g_byte_array_set_size(x->ps[p], 0);
            put_nal(x->ps[p], b, &n);
        }
        cur = n.end;
    }
    g_free(b);
}

static void run_picture(Vc8000dCodec *c, bool hevc)
{
    const NalKind *k = hevc ? &hevc_kind : &h264_kind;
    IngenicA1Vc8000dState *s = c->s;
    uint32_t *r = s->dec;
    uint32_t base = r[R_STRM_BASE];
    uint32_t ring = r[R_RING_SIZE];
    unsigned skip = extract32(r[R_STRM_BIT], 25, 7) / 8;
    uint32_t off = ring ? (r[R_RING_OFFSET] + skip) % ring : skip;
    uint32_t len = MIN(r[R_STRM_LEN], MAX_STREAM);
    unsigned w = extract32(r[R_PICSIZE], 19, 13) * 8;
    unsigned h = extract32(r[R_PICSIZE], 6, 13) * 8;
    uint32_t out = r[R_OUT];
    StreamCtx *x = ctx_get(c, base);
    GByteArray *pkt = g_byte_array_new();
    uint8_t *b;
    size_t cur = 0, end;
    unsigned nslices = 0, i;
    Nal n;

    if (x->hevc != hevc) {
#ifdef CONFIG_INGENIC_VDEC_AVCODEC
        vc8000d_avc_close(x->avc);
        x->avc = NULL;
#endif
        x->hevc = hevc;
    }
    len = len > skip ? len - skip : 0;
    if (w > MAX_DIM || h > MAX_DIM) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-a1-vc8000d: %ux%u picture\n",
                      w, h);
        r[1] |= DEC_BUF_EMPTY;
        return;
    }
    b = g_malloc(len + 4);
    ring_read(s->as, base, ring, off, b, len);
    learn_params(c, x, k, base, ring, off);
    end = len;
    while (nal_next(b, len, cur, &n)) {
        unsigned type;
        int p;

        if (n.data + k->hdr > n.end) {
            cur = n.end;
            continue;
        }
        type = k->type(b + n.data);
        p = k->param(type);
        trace_ingenic_a1_vc8000d_nal(type, n.start, n.data, n.end);
        if (p >= 0) {
            g_byte_array_set_size(x->ps[p], 0);
            put_nal(x->ps[p], b, &n);
        }
        if (k->is_slice(type)) {
            if (nslices && k->first(b + n.data, n.end - n.data)) {
                end = n.start;
                break;
            }
            put_nal(pkt, b, &n);
            nslices++;
        } else if (nslices && (p >= 0 || type == (hevc ? 35 : 9) ||
                               type == (hevc ? 39 : 6))) {
            end = n.start;
            break;
        }
        cur = n.end;
    }
    trace_ingenic_a1_vc8000d_job(hevc ? MODE_HEVC : MODE_H264, base + off, len,
                                 out);

    if (nslices) {
        GByteArray *all = g_byte_array_new();
        bool ok = false;

        for (i = 0; i < 3; i++) {
            g_byte_array_append(all, x->ps[i]->data, x->ps[i]->len);
        }
        g_byte_array_append(all, pkt->data, pkt->len);
#ifdef CONFIG_INGENIC_VDEC_AVCODEC
        if (!x->avc) {
            x->avc = vc8000d_avc_open(hevc);
            if (x->avc && !c->blocker) {
                error_setg(&c->blocker, "ingenic-a1-vc8000d: a decoder holds "
                           "the reference pictures");
                if (migrate_add_blocker(&c->blocker, NULL) < 0) {
                    error_free(c->blocker);
                    c->blocker = NULL;
                }
            }
        }
        if (x->avc) {
            Vc8000dPicture pic;

            if (vc8000d_avc_decode(x->avc, all->data, all->len, &pic)) {
                write_nv12(s, out, r[R_OUT_CHROMA], w, h, &pic);
                ok = true;
            }
        }
#endif
        if (!ok) {
            if (!s->warned_nodec) {
                qemu_log_mask(LOG_UNIMP, "ingenic-a1-vc8000d: no libavcodec, "
                              "pictures are grey\n");
                s->warned_nodec = true;
            }
            fill_grey(s, out, r[R_OUT_CHROMA], w, h);
        }
        g_byte_array_free(all, true);
        r[1] |= DEC_RDY;
    } else {
        r[1] |= DEC_BUF_EMPTY;
    }
    if (ring) {
        r[R_STRM_BASE] = base + (off + (end & ~7u)) % ring;
    } else {
        r[R_STRM_BASE] = (base + off + end) & ~7u;
    }
    g_byte_array_free(pkt, true);
    g_free(b);
}

void vc8000d_codec_run(IngenicA1Vc8000dState *s)
{
    Vc8000dCodec *c = s->codec;

    s->dec[1] &= ~(DEC_RDY | DEC_BUF_EMPTY);
    switch (s->dec[R_MODE] >> 27) {
    case MODE_H264:
        run_picture(c, false);
        break;
    case MODE_HEVC:
        run_picture(c, true);
        break;
    default:
        {
            unsigned i;

            for (i = 0; i < VC8000D_DEC_WORDS; i++) {
                if (s->dec[i]) {
                    trace_ingenic_a1_vc8000d_reg(i, s->dec[i]);
                }
            }
        }
        qemu_log_mask(LOG_UNIMP, "ingenic-a1-vc8000d: decoding mode 0x%x\n",
                      s->dec[R_MODE] >> 27);
        s->dec[1] |= DEC_RDY;
    }
}
