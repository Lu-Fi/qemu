/*
 * Ingenic A1 VC8000D: bitstream decoding with libavcodec
 *
 * The hardware decodes in coding order and writes each picture when it
 * finishes it; libavcodec's low-delay mode gives the same: the picture of
 * the packet just sent.
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
    av_frame_free(&a->frame);
    av_packet_free(&a->pkt);
    avcodec_free_context(&a->ctx);
    g_free(a);
}

bool vc8000d_avc_decode(Vc8000dAvc *a, const uint8_t *data, size_t len,
                        Vc8000dPicture *pic)
{
    int i;

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
