/*
 * Ingenic A1 VC8000D: bitstream decoding with libavcodec
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_VC8000D_AVC_H
#define HW_MISC_INGENIC_A1_VC8000D_AVC_H

typedef struct Vc8000dAvc Vc8000dAvc;

/* A decoded picture, 8-bit 4:2:0 */
typedef struct Vc8000dPicture {
    unsigned width, height;
    const uint8_t *data[3];     /* Y, Cb, Cr */
    int linesize[3];
} Vc8000dPicture;

Vc8000dAvc *vc8000d_avc_open(bool hevc);
void vc8000d_avc_close(Vc8000dAvc *a);

/*
 * Decode one picture's worth of Annex B bytes (parameter sets first, if
 * the stream has not shown them to this decoder). The picture is valid
 * until the next call. Returns false when no picture came out.
 */
bool vc8000d_avc_decode(Vc8000dAvc *a, const uint8_t *data, size_t len,
                        Vc8000dPicture *pic);

#endif
