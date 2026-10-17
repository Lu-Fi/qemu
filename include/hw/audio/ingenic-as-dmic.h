/*
 * Ingenic AudioSystem DMIC capture, the T32, T33 and T41's digital
 * microphone path to memory
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_INGENIC_AS_DMIC_H
#define HW_AUDIO_INGENIC_AS_DMIC_H

#include "hw/core/sysbus.h"
#include "hw/audio/ingenic-mic.h"
#include "qom/object.h"

#define TYPE_INGENIC_AS_DMIC "ingenic-as-dmic"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAsDmicState, INGENIC_AS_DMIC)

#define INGENIC_AS_CHANNELS     10
#define INGENIC_AS_FIFO_BYTES   (4096 * 4)  /* the most a channel can have */

typedef struct IngenicAsChannel {
    /* DMA */
    uint32_t dba, dtc, dcr, dsr, dcm, dda;
    uint32_t flags;             /* word 0 of the descriptor being run */
    uint32_t next;              /* ... and its next descriptor offset */
    bool busy;

    /* FIFO, a byte queue of FAS words */
    uint32_t fas, fcr, ffr, fsr;
    uint8_t fifo[INGENIC_AS_FIFO_BYTES];
    uint32_t rd, n;

    /* Format converter */
    uint32_t dfcr;
} IngenicAsChannel;

struct IngenicAsDmicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion dma_iomem, fifo_iomem, fmt_iomem, dbus_iomem, dmic_iomem;
    qemu_irq irq;
    QEMUTimer *timer;

    AudioBackend *audio_be;
    char *mic;                  /* "tone[:Hz]", a WAV file, or the audiodev */
    IngenicMic mic_src;

    IngenicAsChannel ch[INGENIC_AS_CHANNELS];
    uint32_t dger, aeer, aesr;

    /* The audio bus */
    uint32_t btsr, bfsr, bfcr1, bfcr2, bst[3], btt[2];

    /* The DMIC */
    uint32_t cr0, gcr, ier, icr, tcr;

    /* The frame clock: frames since t0, at rate Hz */
    bool running;
    uint32_t rate;
    int64_t t0;
    uint64_t frames;
};

#endif
