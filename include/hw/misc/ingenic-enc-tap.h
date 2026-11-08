/*
 * Ingenic encoders' stream tap: each encoded stream as a file a player follows
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_ENC_TAP_H
#define HW_MISC_INGENIC_ENC_TAP_H

#include "hw/core/qdev-properties.h"
#include "system/memory.h"
#include "qemu/timer.h"

/*
 * With "x-stream-tap" naming a directory, an encoder writes there every
 * stream its guest codes, as the guest itself delivers it to its
 * application, with nothing in the guest changed:
 *
 *   <name>-<n>.h264, .h265  Annex B, parameter sets ahead of each IDR picture
 *   <name>-<n>.mjpg         JPEG files back to back (a snapshot is one)
 *
 * <name> is the device's ("avpu", "helix", ...), <n> counts its streams in
 * the order their first picture ended, from 0. A stream is told apart by
 * what the encoder gives for it (a libimp channel's buffer). A file is
 * created with the stream's first picture and grows with each one:
 *
 *   ffplay -f h264 -follow 1 -i file:avpu-0.h264
 *
 * A FIFO made beforehand at a stream's path is used instead of a file. It
 * gets pictures once a reader has it open, from the next IDR picture (a
 * JPEG file at once) on, and drops pictures, again to resume at an IDR,
 * while it is full: the guest never waits for a player.
 *
 * What a core writes of an H.264 or H.265 picture is the slice data. The
 * parameter sets and the slice header are libimp's, and where libimp puts
 * them is per encoder family: the AVPU's goes in front of the slice data in
 * the stream buffer, ahead of the job (H.264) or after it (H.265). Where
 * libimp builds the stream apart from the core's window (HELIX, HERA,
 * RADIX), the tap finds the slice data in guest memory (escaped as libimp
 * escapes it) and takes the headers from just before it. In either case a
 * picture is written once the bytes before the data are a slice NAL's
 * start code and header, with the parameter sets that go with an IDR
 * picture, and the tap does not guess libimp's header.
 *
 * The search is through the pages the guest (and its devices) wrote since the
 * last look, which the RAM region tracks for the tap while it is on (the
 * VGA dirty client; RAM is taken to start at physical 0, as it does on every
 * Ingenic board). libimp copies in order, so when a picture is whole in
 * memory, older pictures of its stream that are not are dropped.
 *
 * Off, an encoder does nothing for the tap. On, it copies each picture and
 * polls the guest's memory every 2 ms of guest time till libimp's bytes are
 * in place; a picture that does not show up within a second is dropped, with
 * a warning.
 */
typedef enum IngenicEncTapCodec {
    INGENIC_TAP_H264,
    INGENIC_TAP_H265,
    INGENIC_TAP_JPEG,       /* the core wrote the whole file */
    INGENIC_TAP_JPEG_SCAN,  /* the core wrote the scan, libimp the rest */
} IngenicEncTapCodec;

#define INGENIC_TAP_FIND    HWADDR_MAX

typedef struct IngenicEncTap {
    char *dir;                  /* the property; NULL: off */
    GQueue *queue;              /* pictures not yet whole in memory */
    GList *streams;
    QEMUTimer *timer;
    uint32_t drops;
    GArray *runs;               /* RAM written since the last poll */
} IngenicEncTap;

#define DEFINE_INGENIC_ENC_TAP_PROPERTY(_state, _field)                     \
    DEFINE_PROP_STRING("x-stream-tap", _state, _field.dir)

static inline bool ingenic_enc_tap_on(const IngenicEncTap *t)
{
    return t->dir && *t->dir;
}

/* A picture a job coded */
typedef struct IngenicEncTapAu {
    const char *name;           /* the device's */
    uint64_t stream;            /* what tells the job's stream apart */
    IngenicEncTapCodec codec;
    uint32_t width, height;
    const uint8_t *data;        /* what the core wrote */
    size_t len;
    bool rbsp;                  /* libimp escapes it (adds 03 bytes) */
    hwaddr at;                  /* where it wrote it, or INGENIC_TAP_FIND */
    bool header_after;          /* libimp writes the headers after the job */
} IngenicEncTapAu;

/* Take the picture of a job that has ended; its data is copied */
void ingenic_enc_tap_au(IngenicEncTap *t, const IngenicEncTapAu *au);

#endif /* HW_MISC_INGENIC_ENC_TAP_H */
