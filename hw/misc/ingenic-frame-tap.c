/*
 * Ingenic ISPs' frame taps: the raw frame and each channel's picture as files
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "hw/misc/ingenic-frame-tap.h"
#include "trace.h"

typedef struct TapSink {
    char *key;                  /* "isp-0-1", "raw-0" */
    bool y4m;
    uint32_t w, h;              /* the stream in the file or FIFO */
    int fd;
    bool fifo;
    bool fresh;                 /* just opened: its first frame has a header */
    int made;                   /* files made for the key */
    GByteArray *pend;           /* a FIFO's bytes the pipe has not taken */
    size_t off;
} TapSink;

static void tap_sink_close(TapSink *s)
{
    if (s->fd >= 0) {
#ifndef _WIN32
        if (s->fifo) {
            qemu_set_fd_handler(s->fd, NULL, NULL, NULL);
        }
#endif
        close(s->fd);
        s->fd = -1;
    }
    if (s->pend) {
        g_byte_array_set_size(s->pend, 0);
    }
    s->off = 0;
}

static void tap_sink_free(TapSink *s)
{
    tap_sink_close(s);
    if (s->pend) {
        g_byte_array_unref(s->pend);
    }
    g_free(s->key);
    g_free(s);
}

void ingenic_frame_tap_free(IngenicFrameTap *t)
{
    g_list_free_full(t->sinks, (GDestroyNotify)tap_sink_free);
    t->sinks = NULL;
}

#ifndef _WIN32
/* Give the pipe what it takes of a FIFO sink's pending bytes */
static void tap_sink_flush(TapSink *s)
{
    while (s->pend && s->off < s->pend->len) {
        ssize_t r = write(s->fd, s->pend->data + s->off,
                          s->pend->len - s->off);

        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno != EAGAIN) {
                /* the reader left: the next one starts at a header */
                tap_sink_close(s);
            }
            return;
        }
        s->off += r;
    }
    if (s->pend) {
        g_byte_array_set_size(s->pend, 0);
        s->off = 0;
    }
    qemu_set_fd_handler(s->fd, NULL, NULL, NULL);
}

static void tap_sink_writable(void *opaque)
{
    tap_sink_flush(opaque);
}
#endif

static TapSink *tap_sink(IngenicFrameTap *t, const char *key, bool y4m)
{
    for (GList *l = t->sinks; l; l = l->next) {
        TapSink *s = l->data;

        if (!strcmp(s->key, key)) {
            return s;
        }
    }
    TapSink *s = g_new0(TapSink, 1);

    s->key = g_strdup(key);
    s->y4m = y4m;
    s->fd = -1;
    t->sinks = g_list_append(t->sinks, s);
    return s;
}

/* Open the sink's FIFO or file for a stream of @w x @h; false if it cannot */
static bool tap_sink_open(IngenicFrameTap *t, TapSink *s, uint32_t w,
                          uint32_t h, const char *pixfmt)
{
    g_autofree char *base = NULL;
    g_autofree char *path = NULL;
#ifndef _WIN32
    struct stat st;

    /* a FIFO made beforehand (Windows has none) */
    base = g_strdup_printf("%s.%s", s->key, s->y4m ? "y4m" : "raw");
    path = g_build_filename(t->dir, base, NULL);
    if (!stat(path, &st) && S_ISFIFO(st.st_mode)) {
        s->fd = open(path, O_WRONLY | O_NONBLOCK);
        if (s->fd < 0) {
            return false;       /* ENXIO: no reader yet */
        }
#ifdef CONFIG_LINUX
        fcntl(s->fd, F_SETPIPE_SZ, 1 * MiB);
#endif
        s->fifo = true;
        s->w = w;
        s->h = h;
        return true;
    }
    g_free(base);
    g_free(path);
#endif
    if (s->y4m) {
        base = s->made ? g_strdup_printf("%s-%d.y4m", s->key, s->made) :
                         g_strdup_printf("%s.y4m", s->key);
    } else {
        base = g_strdup_printf("%s-%ux%u-%s.raw", s->key, w, h, pixfmt);
    }
    path = g_build_filename(t->dir, base, NULL);
    g_mkdir_with_parents(t->dir, 0755);
    s->fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, 0644);
    if (s->fd < 0) {
        warn_report("frame tap: %s: %s", path, strerror(errno));
        return false;
    }
    s->made++;
    s->fifo = false;
    s->w = w;
    s->h = h;
    return true;
}

static void tap_write_all(TapSink *s, const uint8_t *p, size_t n)
{
    while (n) {
        ssize_t r = write(s->fd, p, n);

        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            warn_report("frame tap: %s: %s", s->key, strerror(errno));
            tap_sink_close(s);
            return;
        }
        p += r;
        n -= r;
    }
}

/* Whether the sink can take a frame of @w x @h; it is opened if it is not */
static bool tap_sink_ready(IngenicFrameTap *t, TapSink *s, uint32_t w,
                           uint32_t h, const char *pixfmt)
{
    if (s->fd >= 0 && (s->w != w || s->h != h)) {
        tap_sink_close(s);
    }
    if (s->fd < 0) {
        if (!tap_sink_open(t, s, w, h, pixfmt)) {
            return false;
        }
        s->fresh = true;
    }
#ifndef _WIN32
    if (s->fifo && s->pend && s->pend->len) {
        trace_ingenic_frame_tap_drop(s->key);   /* the last one is not read */
        return false;
    }
#endif
    return true;
}

/* Put one frame (@frame, consumed) in a ready sink; @hdr starts a stream */
static void tap_put(TapSink *s, uint32_t w, uint32_t h, const char *hdr,
                    GByteArray *frame)
{
    if (s->fresh && hdr) {
        g_byte_array_prepend(frame, (const uint8_t *)hdr, strlen(hdr));
    }
    s->fresh = false;
    trace_ingenic_frame_tap_put(s->key, w, h, frame->len, s->fifo);
#ifndef _WIN32
    if (s->fifo) {
        if (!s->pend) {
            s->pend = g_byte_array_new();
        }
        g_byte_array_append(s->pend, frame->data, frame->len);
        qemu_set_fd_handler(s->fd, NULL, tap_sink_writable, s);
        tap_sink_flush(s);
        g_byte_array_unref(frame);
        return;
    }
#endif
    tap_write_all(s, frame->data, frame->len);
    g_byte_array_unref(frame);
}

bool ingenic_frame_tap_wants(IngenicFrameTap *t, const char *key, uint32_t w,
                             uint32_t h, const char *pixfmt)
{
    return tap_sink_ready(t, tap_sink(t, key, !pixfmt), w, h, pixfmt);
}

bool ingenic_frame_tap_ready(IngenicFrameTap *t, const char *key, uint32_t w,
                             uint32_t h, const char *pixfmt)
{
    int64_t now = get_clock();

    if (now < t->next_ns || !ingenic_frame_tap_wants(t, key, w, h, pixfmt)) {
        return false;
    }
    t->began_ns = now;
    return true;
}

void ingenic_frame_tap_done(IngenicFrameTap *t)
{
    int64_t now = get_clock();

    t->next_ns = now - t->began_ns > 10 * SCALE_MS ? now + (now - t->began_ns)
                                                    : 0;
}

void ingenic_frame_tap_nv12(IngenicFrameTap *t, const char *name, unsigned fps,
                            const uint8_t *nv12, uint32_t fw, uint32_t fh,
                            uint32_t x, uint32_t y, uint32_t w, uint32_t h)
{
    static const char frame_tag[] = "FRAME\n";
    g_autofree char *hdr = NULL;
    TapSink *s = tap_sink(t, name, true);
    size_t cw = w / 2, ch = h / 2;
    GByteArray *fr;
    uint8_t *p;
    const uint8_t *uv = nv12 + (size_t)fw * fh + (size_t)(y / 2) * fw + x;

    if (!tap_sink_ready(t, s, w, h, NULL)) {
        return;
    }
    hdr = g_strdup_printf("YUV4MPEG2 W%u H%u F%u:1 Ip A1:1 C420mpeg2\n", w, h,
                          MAX(fps, 1));
    fr = g_byte_array_sized_new(sizeof(frame_tag) - 1 + (size_t)w * h * 3 / 2);
    g_byte_array_append(fr, (const uint8_t *)frame_tag, sizeof(frame_tag) - 1);
    g_byte_array_set_size(fr, fr->len + (size_t)w * h * 3 / 2);
    p = fr->data + sizeof(frame_tag) - 1;
    for (uint32_t r = 0; r < h; r++) {
        memcpy(p + (size_t)r * w, nv12 + (size_t)(y + r) * fw + x, w);
    }
    /* NV12's interleaved UV to I420's U plane, then V */
    for (size_t r = 0; r < ch; r++) {
        const uint8_t *c = uv + r * fw;
        uint8_t *u = p + (size_t)w * h + r * cw;
        uint8_t *v = u + cw * ch;

        for (size_t i = 0; i < cw; i++) {
            u[i] = c[2 * i];
            v[i] = c[2 * i + 1];
        }
    }
    tap_put(s, w, h, hdr, fr);
}

void ingenic_frame_tap_raw(IngenicFrameTap *t, unsigned n, uint32_t w,
                           uint32_t h, const char *pixfmt,
                           const uint16_t *raw)
{
    g_autofree char *key = g_strdup_printf("raw-%u", n);
    TapSink *s = tap_sink(t, key, false);
    GByteArray *fr;
    uint8_t *p;

    if (!tap_sink_ready(t, s, w, h, pixfmt)) {
        return;
    }
    fr = g_byte_array_sized_new((size_t)w * h * 2);
    g_byte_array_set_size(fr, (size_t)w * h * 2);
    p = fr->data;
    for (size_t i = 0; i < (size_t)w * h; i++) {
        stw_le_p(p + 2 * i, raw[i]);
    }
    tap_put(s, w, h, NULL, fr);
}
