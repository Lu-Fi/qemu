/*
 * Ingenic A1 VC8000D: bitstream decoding with libavcodec
 *
 * The hardware decodes in coding order and writes each picture when it
 * finishes it; libavcodec's low-delay mode gives the same for H.264: the
 * picture of the packet just sent. Its HEVC decoder holds a picture back by
 * the SPS's reorder depth whatever the flags, so that decoder is fed an SPS
 * that asks for none.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include "ingenic-a1-vc8000d-avc.h"

struct Vc8000dAvc {
    bool hevc;
    GByteArray *tmp;
    AVCodecContext *ctx;
    AVPacket *pkt;
    AVFrame *frame;
};

Vc8000dAvc *vc8000d_avc_open(bool hevc)
{
    const AVCodec *codec = avcodec_find_decoder(hevc ? AV_CODEC_ID_HEVC :
                                                       AV_CODEC_ID_H264);
    Vc8000dAvc *a;

    if (!codec) {
        return NULL;
    }
    a = g_new0(Vc8000dAvc, 1);
    a->hevc = hevc;
    a->tmp = g_byte_array_new();
    a->ctx = avcodec_alloc_context3(codec);
    a->pkt = av_packet_alloc();
    a->frame = av_frame_alloc();
    if (!a->ctx || !a->pkt || !a->frame) {
        vc8000d_avc_close(a);
        return NULL;
    }
    a->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY | AV_CODEC_FLAG_OUTPUT_CORRUPT;
    a->ctx->thread_count = 1;
    a->ctx->apply_cropping = 0;
    if (avcodec_open2(a->ctx, codec, NULL) < 0) {
        vc8000d_avc_close(a);
        return NULL;
    }
    return a;
}

void vc8000d_avc_close(Vc8000dAvc *a)
{
    if (!a) {
        return;
    }
    g_byte_array_free(a->tmp, true);
    av_frame_free(&a->frame);
    av_packet_free(&a->pkt);
    avcodec_free_context(&a->ctx);
    g_free(a);
}

/* RBSP bit reader and writer, for rewriting an SPS */
typedef struct Bits {
    const uint8_t *d;
    size_t nbits, pos;
} Bits;

static unsigned bits_get(Bits *b, unsigned n)
{
    unsigned v = 0;

    while (n--) {
        v = (v << 1) | (b->pos < b->nbits ?
                        (b->d[b->pos >> 3] >> (7 - (b->pos & 7))) & 1 : 0);
        b->pos++;
    }
    return v;
}

static unsigned bits_ue(Bits *b)
{
    unsigned zeros = 0;

    while (b->pos < b->nbits && !bits_get(b, 1) && zeros < 32) {
        zeros++;
    }
    return zeros ? (1u << zeros) - 1 + bits_get(b, zeros) : 0;
}

static void put_bit(GByteArray *w, size_t *n, unsigned bit)
{
    if (!(*n & 7)) {
        guint8 z = 0;

        g_byte_array_append(w, &z, 1);
    }
    w->data[*n >> 3] |= bit << (7 - (*n & 7));
    (*n)++;
}

static void copy_bits(GByteArray *w, size_t *n, const uint8_t *d, size_t from,
                      size_t to)
{
    for (; from < to; from++) {
        put_bit(w, n, (d[from >> 3] >> (7 - (from & 7))) & 1);
    }
}

/*
 * Append the SPS NAL at nal (header included) to out with every
 * sps_max_num_reorder_pics set to 0. Returns false, leaving out as it was,
 * when the SPS is not one this parses.
 */
static bool hevc_sps_no_reorder(GByteArray *out, const uint8_t *nal,
                                size_t len)
{
    GByteArray *rbsp = g_byte_array_new(), *w = g_byte_array_new();
    size_t i, n = 0, last, size = out->len;
    unsigned sub, prof = 0, lvl = 0, flag;
    bool ok = false;
    unsigned zeros = 0;
    Bits b;

    for (i = 2; i < len; i++) {
        if (i >= 4 && nal[i] == 3 && !nal[i - 1] && !nal[i - 2]) {
            continue;
        }
        g_byte_array_append(rbsp, &nal[i], 1);
    }
    for (last = rbsp->len * 8; last > 0; last--) {
        if ((rbsp->data[(last - 1) >> 3] >> (7 - ((last - 1) & 7))) & 1) {
            break;
        }
    }
    b = (Bits){ rbsp->data, last };
    bits_get(&b, 4);
    sub = bits_get(&b, 3);
    bits_get(&b, 1 + 96);
    for (i = 0; i < sub; i++) {
        prof |= bits_get(&b, 1) << i;
        lvl |= bits_get(&b, 1) << i;
    }
    if (sub) {
        bits_get(&b, 2 * (8 - sub));
    }
    for (i = 0; i < sub; i++) {
        bits_get(&b, ((prof >> i) & 1) * 88 + ((lvl >> i) & 1) * 8);
    }
    bits_ue(&b);
    if (bits_ue(&b) == 3) {
        bits_get(&b, 1);
    }
    bits_ue(&b);
    bits_ue(&b);
    if (bits_get(&b, 1)) {
        for (i = 0; i < 4; i++) {
            bits_ue(&b);
        }
    }
    bits_ue(&b);
    bits_ue(&b);
    bits_ue(&b);
    flag = bits_get(&b, 1);
    copy_bits(w, &n, rbsp->data, 0, b.pos);
    for (i = flag ? 0 : sub; i <= sub; i++) {
        size_t from = b.pos;

        bits_ue(&b);
        copy_bits(w, &n, rbsp->data, from, b.pos);
        bits_ue(&b);
        put_bit(w, &n, 1);
        from = b.pos;
        bits_ue(&b);
        copy_bits(w, &n, rbsp->data, from, b.pos);
    }
    if (b.pos >= last) {
        goto out;
    }
    copy_bits(w, &n, rbsp->data, b.pos, last);
    g_byte_array_append(out, nal, 2);
    for (i = 0; i < w->len; i++) {
        if (zeros >= 2 && w->data[i] <= 3) {
            guint8 e = 3;

            g_byte_array_append(out, &e, 1);
            zeros = 0;
        }
        g_byte_array_append(out, &w->data[i], 1);
        zeros = w->data[i] ? 0 : zeros + 1;
    }
    ok = true;
out:
    if (!ok) {
        g_byte_array_set_size(out, size);
    }
    g_byte_array_free(rbsp, true);
    g_byte_array_free(w, true);
    return ok;
}

/* data with every SPS rewritten by hevc_sps_no_reorder() */
static void hevc_no_reorder(GByteArray *out, const uint8_t *data, size_t len)
{
    size_t i = 0, cur = 0;

    g_byte_array_set_size(out, 0);
    while (i + 3 <= len) {
        size_t j;

        if (data[i] || data[i + 1] || data[i + 2] != 1) {
            i++;
            continue;
        }
        for (j = i + 3; j + 3 <= len; j++) {
            if (!data[j] && !data[j + 1] && data[j + 2] <= 1) {
                break;
            }
        }
        if (j + 3 > len) {
            j = len;
        }
        if (j > i + 5 && ((data[i + 3] >> 1) & 0x3f) == 33) {
            g_byte_array_append(out, data + cur, i + 3 - cur);
            if (!hevc_sps_no_reorder(out, data + i + 3, j - i - 3)) {
                g_byte_array_append(out, data + i + 3, j - i - 3);
            }
            cur = j;
        }
        i = j;
    }
    g_byte_array_append(out, data + cur, len - cur);
}

bool vc8000d_avc_decode(Vc8000dAvc *a, const uint8_t *data, size_t len,
                        Vc8000dPicture *pic)
{
    int i;

    if (a->hevc) {
        hevc_no_reorder(a->tmp, data, len);
        data = a->tmp->data;
        len = a->tmp->len;
    }
    av_frame_unref(a->frame);
    a->pkt->data = (uint8_t *)data;
    a->pkt->size = len;
    if (avcodec_send_packet(a->ctx, a->pkt) < 0) {
        return false;
    }
    if (avcodec_receive_frame(a->ctx, a->frame) < 0) {
        return false;
    }
    if (a->frame->format != AV_PIX_FMT_YUV420P &&
        a->frame->format != AV_PIX_FMT_YUVJ420P) {
        qemu_log_mask(LOG_UNIMP, "ingenic-a1-vc8000d: pixel format %d\n",
                      a->frame->format);
        return false;
    }
    pic->width = a->frame->width;
    pic->height = a->frame->height;
    for (i = 0; i < 3; i++) {
        pic->data[i] = a->frame->data[i];
        pic->linesize[i] = a->frame->linesize[i];
    }
    return true;
}
