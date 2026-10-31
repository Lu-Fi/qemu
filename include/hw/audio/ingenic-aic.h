/*
 * Ingenic AIC (the I2S audio controller) with the SoC's internal codec
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_INGENIC_AIC_H
#define HW_AUDIO_INGENIC_AIC_H

#include "hw/core/sysbus.h"
#include "hw/dma/ingenic-pdma.h"
#include "hw/audio/ingenic-mic.h"
#include "qemu/audio.h"
#include "qom/object.h"

#define TYPE_INGENIC_AIC "ingenic-aic"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAicState, INGENIC_AIC)

typedef struct IngenicAicCodec IngenicAicCodec;

#define INGENIC_AIC_TX_DEPTH    64
#define INGENIC_AIC_RX_DEPTH    32
#define INGENIC_AIC_LOOP_DEPTH  32
#define INGENIC_AIC_CODEC_REGS  0x80        /* 32-bit words, 0x000..0x1fc */
#define INGENIC_AIC_RING        8192        /* frames queued to/from the host */

/* The "dreq" outputs, for the PDMA's request types */
enum {
    INGENIC_AIC_DREQ_TX,
    INGENIC_AIC_DREQ_RX,
    INGENIC_AIC_DREQ_LOOP,      /* the TX FIFO loop, read for echo cancelling */
    INGENIC_AIC_NR_DREQ,
};

struct IngenicAicState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    MemoryRegion codec_iomem;
    qemu_irq irq;
    qemu_irq dreq[INGENIC_AIC_NR_DREQ];
    Clock *tx_clk;              /* I2S at 256 fs from the CPM, where wired */
    Clock *rx_clk;
    IngenicPdmaState *pdma;     /* served between frames of a late batch */
    QEMUTimer *timer;

    AudioBackend *audio_be;
    SWVoiceOut *voice_out;
    char *mic;                  /* "tone[:Hz]", a WAV file, or the audiodev */
    IngenicMic mic_src;
    char *codec_name;           /* which internal codec: t31, t41, ... */
    const IngenicAicCodec *cdc;

    uint32_t aicfr, aiccr, i2scr, aicsr, i2sdiv, aictflr, ckcfg, rgadw;
    uint32_t tx[INGENIC_AIC_TX_DEPTH], tx_rd, tx_n;
    uint32_t rx[INGENIC_AIC_RX_DEPTH], rx_rd, rx_n;
    uint32_t loop[INGENIC_AIC_LOOP_DEPTH], loop_rd, loop_n;
    int16_t last[2];            /* the frame last sent, replayed by LSMP */
    uint32_t last_raw[2];       /* ... and the FIFO entries it came from */
    uint32_t codec[INGENIC_AIC_CODEC_REGS];

    /*
     * The frame clocks: play frames since t0 at rate Hz, record frames
     * since rx_t0 at rx_rate Hz (the same clock where the codec is master)
     */
    bool running;
    bool ran;                   /* since reset; I2SSR shows it */
    uint32_t rate;
    int64_t t0;
    uint64_t frames;
    uint32_t rx_rate;
    int64_t rx_t0;
    uint64_t rx_frames;
    int16_t played[2];          /* the frame on the line, for loopback */
    uint32_t played_raw[2];     /* ... and its FIFO entries, for the loop */

    /* Host side, in S16 stereo frames */
    int16_t out_ring[INGENIC_AIC_RING][2];
    uint32_t out_rd, out_n;
    uint32_t voice_rate;
};

#endif
