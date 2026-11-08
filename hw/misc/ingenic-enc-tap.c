/*
 * Ingenic encoders' stream tap: each encoded stream as a file a player follows
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#ifdef CONFIG_LINUX
#include <sys/ioctl.h>
#endif
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/misc/ingenic-enc-tap.h"
#include "trace.h"

/* The bytes of libimp's headers looked at before the data: a chosen bound */
#define TAP_PRE             1024
/* The header of a slice itself is short; farther is not its start code */
#define TAP_SLICE_HDR_MAX   160
/* The slice data's bytes searched for, chosen */
#define TAP_PATTERN         24
/* Guest time between looks at memory, and to a dropped picture: chosen */
#define TAP_POLL_NS         (2 * SCALE_MS)
#define TAP_DROP_NS         (1000 * SCALE_MS)
/* A header unchanged this long after the job is taken as is: chosen */
#define TAP_GRACE_NS        (50 * SCALE_MS)
/* The grain of the written-memory bitmap, and places kept: chosen */
#define TAP_PAGE            4096
#define TAP_CANDS           16
/* Pictures of one stream waiting for libimp, at most: chosen */
#define TAP_QUEUE_MAX       32
/* Looks at a copy that is not whole before it is passed by: chosen */
#define TAP_SHORT_LOOKS     4

/* A place in memory where a picture's data may be */
typedef struct TapCand {
    hwaddr at;
    uint32_t hash;              /* of the headers at the last look */
    bool seen;
    int short_looks;            /* looks with the data not whole */
} TapCand;

typedef struct TapAu {
    char *name;
    uint64_t stream;
    IngenicEncTapCodec codec;
    uint32_t width, height;
    GByteArray *data;           /* the core's */
    bool rbsp, header_after;
    bool known;                 /* its place is the device's: no search */
    int64_t t0;                 /* the job ended */
    GArray *cands;              /* places it may be, TapCand */
    uint8_t *snap;              /* the bytes before the data at the job's end */
    size_t snaplen;
    uint8_t pattern[TAP_PATTERN];
    size_t plen;
    int why;
    uint32_t poff;              /* the pattern's offset in the data */
} TapAu;

typedef struct TapStream {
    char *name;
    uint64_t key;
    IngenicEncTapCodec codec;
    uint32_t width, height;
    int idx;
    int fd;
    bool fifo, synced, started;
} TapStream;

static void tap_au_free(TapAu *a)
{
    g_free(a->name);
    g_free(a->snap);
    g_array_free(a->cands, true);
    g_byte_array_unref(a->data);
    g_free(a);
}

/*
 * Add libimp's 03 bytes to slice data; @zeros are the zeros that end what
 * precedes
 */
static void tap_escape(GByteArray *out, const uint8_t *p, size_t n, int zeros)
{
    static const uint8_t epb = 3;

    for (size_t i = 0; i < n; i++) {
        if (zeros >= 2 && p[i] <= 3) {
            g_byte_array_append(out, &epb, 1);
            zeros = 0;
        }
        g_byte_array_append(out, &p[i], 1);
        zeros = p[i] ? 0 : zeros + 1;
    }
}

static const uint8_t *tap_find_bytes(const uint8_t *h, size_t hl,
                                     const uint8_t *n, size_t nl)
{
    while (hl >= nl) {
        const uint8_t *p = memchr(h, n[0], hl - nl + 1);

        if (!p) {
            return NULL;
        }
        if (!memcmp(p, n, nl)) {
            return p;
        }
        hl -= p + 1 - h;
        h = p + 1;
    }
    return NULL;
}

/*
 * The runs of RAM written since the last look, in bytes from its start, each
 * widened by what a pattern can straddle
 */
typedef struct TapRun {
    hwaddr at;
    size_t len;
} TapRun;

static void tap_dirty(IngenicEncTap *t)
{
    MemoryRegion *ram = current_machine->ram;
    hwaddr size = current_machine->ram_size;
    DirtyBitmapSnapshot *snap = memory_region_snapshot_and_clear_dirty(
        ram, 0, size, DIRTY_MEMORY_VGA);
    hwaddr start = 0;
    bool in_run = false;

    if (!t->runs) {
        t->runs = g_array_new(false, false, sizeof(TapRun));
    }
    g_array_set_size(t->runs, 0);
    for (hwaddr at = 0; at <= size; at += TAP_PAGE) {
        bool dirty = at < size &&
                     memory_region_snapshot_get_dirty(ram, snap, at, TAP_PAGE);

        if (dirty && !in_run) {
            start = at;
        }
        if (!dirty && in_run) {
            TapRun r = { .at = start, .len = at - start };

            g_array_append_val(t->runs, r);
        }
        in_run = dirty;
    }
    g_free(snap);
}

/* Add to @a's candidates the places in the written runs holding its pattern */
static void tap_collect(const IngenicEncTap *t, TapAu *a)
{
    const uint8_t *ram = memory_region_get_ram_ptr(current_machine->ram);
    hwaddr size = current_machine->ram_size;

    for (guint i = 0; i < t->runs->len && a->cands->len < TAP_CANDS; i++) {
        const TapRun *r = &g_array_index(t->runs, TapRun, i);
        hwaddr from = r->at >= a->plen ? r->at - a->plen + 1 : 0;
        hwaddr to = MIN(r->at + r->len + a->plen - 1, size);
        const uint8_t *h = ram + from, *hit;

        while (to - from >= a->plen &&
               (hit = tap_find_bytes(h, to - from, a->pattern, a->plen))) {
            TapCand c = { .at = hit - ram - a->poff };

            bool have = false;

            for (guint k = 0; k < a->cands->len; k++) {
                have |= g_array_index(a->cands, TapCand, k).at == c.at;
            }
            if (c.at < size && !have) {
                g_array_append_val(a->cands, c);
            }
            from = hit - ram + 1;
            h = ram + from;
        }
    }
}

static int tap_nal_type(IngenicEncTapCodec c, const uint8_t *p, size_t left)
{
    if (c == INGENIC_TAP_H264) {
        return left >= 1 && !(p[0] & 0x80) ? p[0] & 0x1f : -1;
    }
    return left >= 2 && !(p[0] & 0x81) && !(p[1] & 0xf8) && (p[1] & 7)
           ? (p[0] >> 1) & 0x3f : -1;
}

static bool tap_is_slice(IngenicEncTapCodec c, int t)
{
    return c == INGENIC_TAP_H264 ? t == 1 || t == 5
                                 : (t >= 0 && t <= 9) || (t >= 16 && t <= 21);
}

static bool tap_is_idr(IngenicEncTapCodec c, int t)
{
    return c == INGENIC_TAP_H264 ? t == 5 : t >= 16 && t <= 21;
}

static bool tap_is_params(IngenicEncTapCodec c, int t)
{
    return c == INGENIC_TAP_H264 ? t == 7 || t == 8 : t >= 32 && t <= 34;
}

/* The last start code in pre[0..n) with a NAL header after it, or -1 */
static ssize_t tap_last_sc(const uint8_t *pre, ssize_t n)
{
    for (ssize_t p = n - 4; p >= 0; p--) {
        if (pre[p + 2] == 1 && !pre[p + 1] && !pre[p]) {
            return p;
        }
    }
    return -1;
}

/* A NAL has no three zeros in a row: such a gap is the buffer's, not its */
static bool tap_zero_gap(const uint8_t *p, size_t n)
{
    for (size_t i = 0; i + 2 < n; i++) {
        if (!p[i] && !p[i + 1] && !p[i + 2]) {
            return true;
        }
    }
    return false;
}

/*
 * Where libimp's bytes begin in pre[0..n), the data following: its slice
 * NAL's start code, and for an IDR slice the parameter sets before it. -1
 * when what precedes the data is not a slice header.
 */
static ssize_t tap_header_start(IngenicEncTapCodec c, const uint8_t *pre,
                                size_t n, bool *idr, bool *params)
{
    ssize_t p = tap_last_sc(pre, n);
    int t;

    if (p < 0 || n - p > TAP_SLICE_HDR_MAX) {
        return -1;
    }
    t = tap_nal_type(c, pre + p + 3, n - p - 3);
    if (!tap_is_slice(c, t)) {
        return -1;
    }
    *idr = tap_is_idr(c, t);
    *params = false;
    while (*idr) {
        ssize_t q = tap_last_sc(pre, p);

        if (q < 0 || !tap_is_params(c, tap_nal_type(c, pre + q + 3,
                                                    p - q - 3)) ||
            tap_zero_gap(pre + q + 3, p - q - 3)) {
            break;
        }
        *params = true;
        p = q;
    }
    return p > 0 && !pre[p - 1] ? p - 1 : p;
}

/* A JPEG file's start: the SOI before the SOS that ends the bytes ahead */
static ssize_t tap_jpeg_start(const uint8_t *pre, size_t n)
{
    ssize_t sos = -1;

    for (ssize_t p = n - 2; p >= 0 && p >= (ssize_t)n - 32; p--) {
        if (pre[p] == 0xff && pre[p + 1] == 0xda) {
            sos = p;
            break;
        }
    }
    if (sos < 0) {
        return -1;
    }
    for (ssize_t p = sos - 2; p >= 0; p--) {
        if (pre[p] == 0xff && pre[p + 1] == 0xd8 && pre[p + 2] == 0xff) {
            return p;
        }
    }
    return -1;
}

static TapStream *tap_stream(IngenicEncTap *t, const TapAu *a)
{
    for (GList *l = t->streams; l; l = l->next) {
        TapStream *s = l->data;

        if (s->key == a->stream && s->codec == a->codec &&
            s->width == a->width && s->height == a->height &&
            !strcmp(s->name, a->name)) {
            return s;
        }
    }
    TapStream *s = g_new0(TapStream, 1);

    s->name = g_strdup(a->name);
    s->key = a->stream;
    s->codec = a->codec;
    s->width = a->width;
    s->height = a->height;
    s->idx = -1;
    s->fd = -1;
    t->streams = g_list_append(t->streams, s);
    return s;
}

static void tap_open(IngenicEncTap *t, TapStream *s)
{
    static const char *const ext[] = { "h264", "h265", "mjpg", "mjpg" };
    g_autofree char *base = NULL;
    g_autofree char *path = NULL;
#ifndef _WIN32
    struct stat st;
#endif
    int n = 0;

    for (GList *l = t->streams; l; l = l->next) {
        const TapStream *o = l->data;

        n += o != s && o->idx >= 0 && !strcmp(o->name, s->name);
    }
    s->idx = n;
    base = g_strdup_printf("%s-%d.%s", s->name, s->idx, ext[s->codec]);
    path = g_build_filename(t->dir, base, NULL);
#ifndef _WIN32
    /* a FIFO made beforehand (Windows has none) */
    if (!stat(path, &st) && S_ISFIFO(st.st_mode)) {
        s->fd = open(path, O_WRONLY | O_NONBLOCK);
        s->fifo = true;
        if (s->fd >= 0) {
#ifdef CONFIG_LINUX
            fcntl(s->fd, F_SETPIPE_SZ, 1 * MiB);
#endif
            s->synced = false;
        }
        return;
    }
#endif
    g_mkdir_with_parents(t->dir, 0755);
    s->fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, 0644);
    if (s->fd < 0) {
        warn_report("%s: stream tap: %s: %s", s->name, path, strerror(errno));
    }
    s->fifo = false;
}

static void tap_write(IngenicEncTap *t, TapStream *s, const uint8_t *p,
                      size_t n, bool key)
{
    if (s->fd < 0) {
        tap_open(t, s);
        if (s->fd < 0) {
            return;
        }
    }
    if (s->fifo) {
        if (!key && !s->synced) {
            return;
        }
#ifdef CONFIG_LINUX
        /* a full pipe would split the picture: drop it, resume at an IDR */
        int unread = 0;
        int room = fcntl(s->fd, F_GETPIPE_SZ);

        if (room > 0 && !ioctl(s->fd, FIONREAD, &unread) &&
            n > (size_t)(room - unread)) {
            s->synced = false;
            trace_ingenic_enc_tap_drop(s->name, s->idx, n, 0);
            return;
        }
#endif
    }
    while (n) {
        ssize_t r = write(s->fd, p, n);

        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (s->fifo && errno == EPIPE) {
                close(s->fd);
                s->fd = -1;
            }
            s->synced = false;
            return;
        }
        p += r;
        n -= r;
    }
    if (s->fifo) {
        s->synced = true;
    }
}

/* Write @a's picture with the headers @hdr, @data after them */
static void tap_emit(IngenicEncTap *t, TapAu *a, const uint8_t *hdr,
                     size_t hlen, const uint8_t *data, size_t dlen,
                     const uint8_t *tail, size_t tlen, bool key)
{
    TapStream *s = tap_stream(t, a);
    g_autofree uint8_t *buf = g_malloc(hlen + dlen + tlen);

    memcpy(buf, hdr, hlen);
    memcpy(buf + hlen, data, dlen);
    memcpy(buf + hlen + dlen, tail, tlen);
    trace_ingenic_enc_tap_emit(a->name, s->idx, hlen, dlen + tlen, key);
    tap_write(t, s, buf, hlen + dlen + tlen, key);
}

enum { TAP_DONE, TAP_WAIT, TAP_DROP };

/* Why a picture is not out yet, for the drop trace */
enum {
    WHY_NOT_FOUND = 1,      /* its data is nowhere in memory */
    WHY_MOVED,              /* what was found there changed */
    WHY_NO_HEADER,          /* no slice NAL's start code before it */
    WHY_STALE_HEADER,       /* the headers are the last picture's */
    WHY_UNSTABLE,           /* the headers changed between looks */
    WHY_SHORT,              /* memory holds only part of the data */
    WHY_OVERTAKEN,          /* a later picture of the stream is there */
};

static int tap_late(const TapAu *a, int64_t now)
{
    return now - a->t0 > TAP_DROP_NS ? TAP_DROP : TAP_WAIT;
}

/* FNV-1a: enough to tell two looks at libimp's headers apart */
static uint32_t tap_hash(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;

    while (n--) {
        h = (h ^ *p++) * 16777619u;
    }
    return h;
}

enum { CAND_DEAD, CAND_WAIT, CAND_READY };

/* Whether the bytes at one of @a's places are its picture, with headers */
static int tap_eval(IngenicEncTap *t, TapAu *a, TapCand *c, int64_t now)
{
    g_autoptr(GByteArray) esc = g_byte_array_new();
    g_autofree uint8_t *mem = NULL;
    uint8_t pre[TAP_PRE], got[TAP_PATTERN];
    TapStream *s = tap_stream(t, a);
    bool video = a->codec == INGENIC_TAP_H264 || a->codec == INGENIC_TAP_H265;
    size_t n, elen;
    ssize_t start;
    bool idr = false, params = false;
    uint32_t hash;
    int zeros = 0;

    address_space_read(&address_space_memory, c->at + a->poff,
                       MEMTXATTRS_UNSPECIFIED, got, a->plen);
    if (memcmp(got, a->pattern, a->plen)) {
        a->why = WHY_MOVED;
        return a->known ? CAND_WAIT : CAND_DEAD;
    }
    n = MIN(c->at, TAP_PRE);
    address_space_read(&address_space_memory, c->at - n,
                       MEMTXATTRS_UNSPECIFIED, pre, n);
    start = video ? tap_header_start(a->codec, pre, n, &idr, &params)
                  : tap_jpeg_start(pre, n);
    if (start < 0) {
        /* libimp's bytes are not there yet, or this is another copy */
        a->why = WHY_NO_HEADER;
        return CAND_WAIT;
    }
    if (a->header_after && a->snap &&
        !memcmp(a->snap, pre, MIN(n, a->snaplen)) &&
        now - a->t0 < TAP_GRACE_NS) {
        a->why = WHY_STALE_HEADER;
        return CAND_WAIT;
    }
    hash = tap_hash(pre + start, n - start);
    if (!a->known && !(c->seen && c->hash == hash)) {
        /* libimp may still be writing: wait for two looks alike */
        c->seen = true;
        c->hash = hash;
        a->why = WHY_UNSTABLE;
        return CAND_WAIT;
    }
    elen = a->data->len;
    if (video && a->rbsp) {
        for (size_t i = n; i > 0 && zeros < 2 && !pre[i - 1]; i--) {
            zeros++;
        }
        tap_escape(esc, a->data->data, a->data->len, zeros);
        elen = esc->len;
    } else {
        g_byte_array_append(esc, a->data->data, a->data->len);
    }
    mem = g_malloc(elen + 2);
    address_space_read(&address_space_memory, c->at, MEMTXATTRS_UNSPECIFIED,
                       mem, elen + 2);
    if (memcmp(mem, esc->data, elen)) {
        /*
         * libimp copies the data in pieces: what is not yet there is waited
         * for a few looks; a copy that stays short (a ring of slots too
         * small for the picture) is not the stream's
         */
        a->why = WHY_SHORT;
        return ++c->short_looks > TAP_SHORT_LOOKS && !a->known ? CAND_DEAD
                                                               : CAND_WAIT;
    }
    if (a->codec == INGENIC_TAP_JPEG_SCAN) {
        static const uint8_t eoi[2] = { 0xff, 0xd9 };

        if (memcmp(mem + elen, eoi, 2) && now - a->t0 < TAP_GRACE_NS) {
            return CAND_WAIT;
        }
        tap_emit(t, a, pre + start, n - start, esc->data, elen, eoi, 2, true);
    } else {
        /* the first picture a player can start on carries the parameter sets */
        if (video && !s->started && !params) {
            return CAND_READY;
        }
        s->started = true;
        tap_emit(t, a, pre + start, n - start, esc->data, elen, NULL, 0,
                 !video || idr);
    }
    return CAND_READY;
}

/* Try to complete a picture from guest memory */
static int tap_try(IngenicEncTap *t, TapAu *a, int64_t now)
{
    if (a->codec == INGENIC_TAP_JPEG) {
        tap_emit(t, a, NULL, 0, a->data->data, a->data->len, NULL, 0, true);
        return TAP_DONE;
    }
    a->why = WHY_NOT_FOUND;
    for (guint i = 0; i < a->cands->len; ) {
        TapCand *c = &g_array_index(a->cands, TapCand, i);

        switch (tap_eval(t, a, c, now)) {
        case CAND_READY:
            return TAP_DONE;
        case CAND_DEAD:
            g_array_remove_index(a->cands, i);
            break;
        default:
            i++;
        }
    }
    return tap_late(a, now);
}

static void tap_drop(IngenicEncTap *t, const TapAu *a, const TapStream *s)
{
    if (t->drops++ < 8 || !(t->drops % 64)) {
        warn_report("%s: stream tap: libimp's bytes for a picture never "
                    "showed up in memory (%u dropped)", a->name, t->drops);
    }
    trace_ingenic_enc_tap_drop(a->name, s->idx, a->data->len, a->why);
}

static void tap_poll(void *opaque)
{
    IngenicEncTap *t = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    GList *l = g_queue_peek_head_link(t->queue);

    /* every picture looks in what the guest wrote since the last poll */
    tap_dirty(t);
    for (GList *k = l; k; k = k->next) {
        TapAu *a = k->data;

        if (!a->known) {
            tap_collect(t, a);
        }
    }
    while (l) {
        TapAu *a = l->data;
        GList *next = l->next;
        TapStream *s = tap_stream(t, a);
        int r = tap_try(t, a, now);

        if (r == TAP_WAIT) {
            l = next;
            continue;
        }
        if (r == TAP_DONE) {
            /* libimp copies in order: older pictures not there never will be */
            for (GList *k = g_queue_peek_head_link(t->queue); k != l; ) {
                TapAu *o = k->data;
                GList *kn = k->next;

                if (tap_stream(t, o) == s) {
                    o->why = WHY_OVERTAKEN;
                    tap_drop(t, o, s);
                    g_queue_delete_link(t->queue, k);
                    tap_au_free(o);
                }
                k = kn;
            }
        } else {
            tap_drop(t, a, s);
        }
        g_queue_delete_link(t->queue, l);
        tap_au_free(a);
        l = next;
    }
    if (!g_queue_is_empty(t->queue)) {
        timer_mod(t->timer, now + TAP_POLL_NS);
    }
}

/* Keep a stream nobody takes from piling up: the oldest picture goes */
static void tap_trim(IngenicEncTap *t, const TapAu *n)
{
    guint same = 0;
    GList *oldest = NULL;

    for (GList *l = g_queue_peek_head_link(t->queue); l; l = l->next) {
        const TapAu *a = l->data;

        if (a->stream == n->stream && a->codec == n->codec &&
            a->width == n->width && a->height == n->height &&
            !strcmp(a->name, n->name)) {
            oldest = oldest ?: l;
            same++;
        }
    }
    if (same >= TAP_QUEUE_MAX) {
        TapAu *a = oldest->data;

        trace_ingenic_enc_tap_drop(a->name, -1, a->data->len, WHY_NOT_FOUND);
        t->drops++;
        g_queue_delete_link(t->queue, oldest);
        tap_au_free(a);
    }
}

void ingenic_enc_tap_au(IngenicEncTap *t, const IngenicEncTapAu *au)
{
    TapAu *a;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    bool video = au->codec == INGENIC_TAP_H264 || au->codec == INGENIC_TAP_H265;

    if (!ingenic_enc_tap_on(t) || !au->len) {
        return;
    }
    if (!t->queue) {
        /* RAM tells which of its pages are written only if asked to */
        memory_region_set_log(current_machine->ram, true, DIRTY_MEMORY_VGA);
        t->queue = g_queue_new();
        t->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tap_poll, t);
    }
    a = g_new0(TapAu, 1);
    a->name = g_strdup(au->name);
    a->stream = au->stream;
    a->codec = au->codec;
    a->width = au->width;
    a->height = au->height;
    a->data = g_byte_array_append(g_byte_array_new(), au->data, au->len);
    a->rbsp = au->rbsp;
    a->header_after = au->header_after;
    a->t0 = now;
    a->cands = g_array_new(false, false, sizeof(TapCand));
    if (au->codec != INGENIC_TAP_JPEG) {
        g_autoptr(GByteArray) e = g_byte_array_new();
        size_t n = MIN(au->len, 2 * TAP_PATTERN);

        if (video && au->rbsp) {
            tap_escape(e, au->data, n, 0);
        } else {
            g_byte_array_append(e, au->data, n);
        }
        /* a run of 0 or ff matches everywhere in memory: start past it */
        while (a->poff + TAP_PATTERN < e->len &&
               (e->data[a->poff] == 0 || e->data[a->poff] == 0xff)) {
            a->poff++;
        }
        a->plen = MIN(TAP_PATTERN, e->len - a->poff);
        memcpy(a->pattern, e->data + a->poff, a->plen);
        if (au->at != INGENIC_TAP_FIND) {
            TapCand c = { .at = au->at, .seen = true };

            a->known = true;
            g_array_append_val(a->cands, c);
        }
        if (au->at != INGENIC_TAP_FIND && au->header_after) {
            a->snaplen = MIN((hwaddr)TAP_PRE, au->at);
            a->snap = g_malloc(a->snaplen);
            address_space_read(&address_space_memory, au->at - a->snaplen,
                               MEMTXATTRS_UNSPECIFIED, a->snap, a->snaplen);
        }
    }
    if (g_queue_is_empty(t->queue)) {
        /* what was written while no picture waited is of no use */
        tap_dirty(t);
    }
    tap_trim(t, a);
    g_queue_push_tail(t->queue, a);
    tap_poll(t);
}
