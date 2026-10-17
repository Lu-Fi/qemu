/*
 * Ingenic DMIC, the digital microphone controller of the T30, T31 and T40
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_INGENIC_DMIC_H
#define HW_AUDIO_INGENIC_DMIC_H

#include "hw/core/sysbus.h"
#include "hw/dma/ingenic-pdma.h"
#include "hw/audio/ingenic-mic.h"
#include "qom/object.h"

#define TYPE_INGENIC_DMIC "ingenic-dmic"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicDmicState, INGENIC_DMIC)

#define INGENIC_DMIC_FIFO   64          /* 32-bit words */

struct IngenicDmicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dreq;              /* to the PDMA, the RX request */
    IngenicPdmaState *pdma;     /* served between frames of a late batch */
    QEMUTimer *timer;

    AudioBackend *audio_be;
    char *mic;                  /* "tone[:Hz]", a WAV file, or the audiodev */
    IngenicMic mic_src;

    uint32_t cr0, gcr, imr, intcr, tricr, thrh, thrl, trimmax, trinmax;
    uint32_t fthr, cgdis;
    uint32_t fifo[INGENIC_DMIC_FIFO], fifo_rd, fifo_n;
    uint32_t dr;                /* the word last read, read again when empty */
    uint32_t half;              /* a packed sample waiting for its pair */
    bool half_full;

    /* The frame clock: frames since t0, at rate Hz */
    bool running;
    uint32_t rate;
    int64_t t0;
    uint64_t frames;
};

#endif
