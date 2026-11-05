/*
 * Ingenic encoders' capture: each job's output, source frame and settings
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_ENC_CAPTURE_H
#define HW_MISC_INGENIC_ENC_CAPTURE_H

#include "hw/core/qdev-properties.h"

typedef struct IngenicAvpuPic IngenicAvpuPic;

/*
 * With "capture" naming a directory, every job an encoder finishes leaves
 * there <name>-<seq>.<ext>, the bytes it wrote out; <name>-<seq>.nv12, the
 * frame it coded, its Y then its UV plane, packed to its width; and
 * <name>-<seq>.txt, its settings, "key value" a line. <seq> counts the
 * device's jobs from 0. A 1080p stream leaves 3 MiB of frame a job.
 */
typedef struct IngenicEncCapture {
    char *dir;                  /* the property; NULL: off */
    uint32_t seq;
} IngenicEncCapture;

#define DEFINE_INGENIC_ENC_CAPTURE_PROPERTY(_state, _field)                 \
    DEFINE_PROP_STRING("capture", _state, _field.dir)

static inline bool ingenic_enc_capture_on(const IngenicEncCapture *c)
{
    return c->dir && *c->dir;
}

/*
 * The frame a job codes, @width x @height NV12 from its planes, packed for
 * ingenic_enc_capture(); NULL with capture off. An encoder that overwrites
 * its source takes this before it codes.
 */
uint8_t *ingenic_enc_capture_frame(const IngenicEncCapture *c,
                                   const uint8_t *y, uint32_t stride,
                                   const uint8_t *uv, uint32_t cstride,
                                   uint32_t width, uint32_t height);

/*
 * Write the job's files, @frame the one ingenic_enc_capture_frame() gave
 * (or NULL), @info the settings so far, to which the frame's size is added
 */
void ingenic_enc_capture(IngenicEncCapture *c, const char *name,
                         const char *ext, const void *out, size_t len,
                         const uint8_t *frame, uint32_t width,
                         uint32_t height, GString *info);

/* Append an encode's settings, and the nonzero words of @regs */
void ingenic_enc_capture_pic(GString *info, const IngenicAvpuPic *pic);
void ingenic_enc_capture_regs(GString *info, const char *key,
                              const uint32_t *regs, size_t n);

#endif /* HW_MISC_INGENIC_ENC_CAPTURE_H */
