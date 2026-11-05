/*
 * Ingenic encoders' capture: each job's output, source frame and settings
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "hw/misc/ingenic-avpu.h"
#include "hw/misc/ingenic-enc-capture.h"

static void capture_write(const char *dir, const char *name, uint32_t seq,
                          const char *ext, const void *p, size_t len)
{
    g_autofree char *base = g_strdup_printf("%s-%06u.%s", name, seq, ext);
    g_autofree char *path = g_build_filename(dir, base, NULL);
    g_autoptr(GError) err = NULL;

    if (!g_file_set_contents(path, p, len, &err)) {
        warn_report("%s: capture: %s", name, err->message);
    }
}

uint8_t *ingenic_enc_capture_frame(const IngenicEncCapture *c,
                                   const uint8_t *y, uint32_t stride,
                                   const uint8_t *uv, uint32_t cstride,
                                   uint32_t width, uint32_t height)
{
    size_t ysize = (size_t)width * height;
    uint8_t *frame;

    if (!ingenic_enc_capture_on(c) || !y || !uv) {
        return NULL;
    }
    frame = g_malloc(ysize + ysize / 2);
    for (uint32_t r = 0; r < height; r++) {
        memcpy(frame + (size_t)r * width, y + (size_t)r * stride, width);
    }
    for (uint32_t r = 0; r < height / 2; r++) {
        memcpy(frame + ysize + (size_t)r * width, uv + (size_t)r * cstride,
               width);
    }
    return frame;
}

void ingenic_enc_capture(IngenicEncCapture *c, const char *name,
                         const char *ext, const void *out, size_t len,
                         const uint8_t *frame, uint32_t width,
                         uint32_t height, GString *info)
{
    size_t ysize = (size_t)width * height;

    if (!ingenic_enc_capture_on(c)) {
        return;
    }
    capture_write(c->dir, name, c->seq, ext, out, len);
    if (frame) {
        capture_write(c->dir, name, c->seq, "nv12", frame, ysize + ysize / 2);
    }
    g_string_append_printf(info, "width %u\nheight %u\nbytes %zu\n",
                           width, height, len);
    capture_write(c->dir, name, c->seq, "txt", info->str, info->len);
    c->seq++;
}

void ingenic_enc_capture_pic(GString *info, const IngenicAvpuPic *pic)
{
    g_string_append_printf(info,
                           "codec %s\nintra %d\nqp %d\nauto_qp %d\ncabac %d\n"
                           "transform_8x8 %d\nscaling_list %d\n"
                           "cabac_init_idc %d\nno_deblock %d\n"
                           "alpha_c0_div2 %d\nbeta_div2 %d\n"
                           "chroma_qp_offset %d\ncr_qp_offset %d\n",
                           pic->hevc ? "h265" : "h264", pic->intra, pic->qp,
                           pic->auto_qp, pic->cabac, pic->transform_8x8,
                           pic->scaling_list, pic->cabac_init_idc,
                           pic->no_deblock, pic->alpha_c0_div2,
                           pic->beta_div2, pic->chroma_qp_offset,
                           pic->cr_qp_offset);
}

void ingenic_enc_capture_regs(GString *info, const char *key,
                              const uint32_t *regs, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        if (regs[i]) {
            g_string_append_printf(info, "%s 0x%04zx 0x%08x\n", key, i * 4,
                                   regs[i]);
        }
    }
}
