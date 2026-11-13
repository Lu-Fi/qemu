/*
 * A baseline JPEG encoder for the UVC camera's MJPEG frames
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_USB_UVC_JPEG_H
#define HW_USB_UVC_JPEG_H

/*
 * Encode an NV12 picture of @w x @h (both even, the chroma plane @w bytes a
 * line) as a 4:2:0 (4:2:2 if @yuv422) baseline JPEG of quality @quality
 * (1-100), with the standard Huffman tables in its header. The result is
 * g_malloc()ed, its size in @len.
 */
uint8_t *uvc_jpeg_encode(const uint8_t *nv12, unsigned w, unsigned h,
                         unsigned quality, bool yuv422, size_t *len);

#endif
