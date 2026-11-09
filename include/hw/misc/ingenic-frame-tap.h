/*
 * Ingenic ISPs' frame taps: the raw frame and each channel's picture as files
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_FRAME_TAP_H
#define HW_MISC_INGENIC_FRAME_TAP_H

#include "hw/core/qdev-properties.h"

/*
 * With "x-frame-tap" naming a directory, an ISP writes there, with nothing in
 * the guest changed:
 *
 *   isp-<pipe>-<chan>.y4m   each ISP channel's picture, as the ISP puts it
 *                           out (NV12 in the guest's buffer, I420 in a
 *                           YUV4MPEG2 stream); a channel an IVDC takes
 *                           straight to the encoder has its picture
 *                           drawn the same way
 *   raw-<n>-<W>x<H>-<pixfmt>.raw
 *                           the raw frame as VIC <n> takes it: the Bayer
 *                           samples at the sensor's bits, left justified in
 *                           16, little-endian, frames back to back
 *
 * A file is created when its first frame is due and grows with each frame:
 *
 *   ffplay -follow 1 -i file:isp-0-0.y4m
 *   ffplay -follow 1 -framerate 25 -f rawvideo -video_size 1920x1080 \
 *          -pixel_format bayer_rggb16le -i file:raw-0-....raw
 *
 * A channel or VIC whose size changes goes on in a new file (-1, -2 ... added
 * to a .y4m's name, the new size in a .raw's). A FIFO made beforehand at
 * isp-<pipe>-<chan>.y4m or raw-<n>.raw is used instead of a file: it gets
 * frames once a reader has it open (a .y4m with its header first), drops
 * every frame that finds the last not yet read, and never makes the guest
 * wait.
 *
 * Off, an ISP does nothing for the tap and renders nothing extra. On, a
 * channel an IVDC takes is drawn for the tap as well, and the raw frame is
 * rendered at each VIC frame.
 */
typedef struct IngenicFrameTap {
    char *dir;                  /* the property; NULL: off */
    GList *sinks;
    int64_t next_ns;            /* host time before which a render waits */
    int64_t began_ns;
} IngenicFrameTap;

#define DEFINE_INGENIC_FRAME_TAP_PROPERTY(_state, _field)                   \
    DEFINE_PROP_STRING("x-frame-tap", _state, _field.dir)

static inline bool ingenic_frame_tap_on(const IngenicFrameTap *t)
{
    return t->dir && *t->dir;
}

/*
 * Whether the stream @key ("isp-<pipe>-<chan>", "raw-<n>") can take a frame
 * of @w x @h now: its file is made or its FIFO has a reader that is keeping
 * up. @pixfmt is the raw frame's pixel format, NULL for a channel's. A frame
 * the tap alone needs (the raw frame, an IVDC channel's) is rendered only
 * after ingenic_frame_tap_ready() says yes, which asks that and also keeps
 * the tap from starving the guest: at 1080p a rendering can take longer than
 * the guest's frame time, so one that took over 10 ms of host time (chosen)
 * is followed by a pause as long, in which those frames are skipped, so the
 * tap takes half the host at most. done() ends a rendering begun after a
 * true ready().
 */
bool ingenic_frame_tap_wants(IngenicFrameTap *t, const char *key, uint32_t w,
                             uint32_t h, const char *pixfmt);
bool ingenic_frame_tap_ready(IngenicFrameTap *t, const char *key, uint32_t w,
                             uint32_t h, const char *pixfmt);
void ingenic_frame_tap_done(IngenicFrameTap *t);

/*
 * A channel's picture: the @w x @h window at (@x, @y) of the NV12 frame @fw x
 * @fh (its chroma plane after the luma, @fw apart). @name is "isp-<pipe>-
 * <chan>", @fps the rate a player shows it at.
 */
void ingenic_frame_tap_nv12(IngenicFrameTap *t, const char *name, unsigned fps,
                            const uint8_t *nv12, uint32_t fw, uint32_t fh,
                            uint32_t x, uint32_t y, uint32_t w, uint32_t h);

/*
 * VIC @n's raw frame, @w x @h samples of 16 bits in host order; @pixfmt is
 * ffmpeg's name for the pattern ("bayer_rggb16le")
 */
void ingenic_frame_tap_raw(IngenicFrameTap *t, unsigned n, uint32_t w,
                           uint32_t h, const char *pixfmt,
                           const uint16_t *raw);

void ingenic_frame_tap_free(IngenicFrameTap *t);

#endif /* HW_MISC_INGENIC_FRAME_TAP_H */
