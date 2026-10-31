/*
 * Ingenic AIC (the I2S audio controller) with the SoC's internal codec
 *
 * The AIC moves samples between memory and the codec through a 64-entry
 * TX FIFO and a 32-entry RX FIFO, one sample per entry (two in PACK16),
 * asking the PDMA for a refill when TX holds TFTH * 2 samples or fewer
 * and for a drain when RX holds (RFTH + 1) * 2 or more. With the TFIFO
 * loop on (ETFL), each frame recorded also queues the frame playing (zero
 * when nothing plays), as written to TX and with MONOCTR's channels, for
 * the CPU or DMA to read back from AICLR, (TFLTH + 1) * 2 at a time; the
 * vendor driver feeds it to its echo canceller beside the mic samples.
 *
 * As read on a T31X, unlike the T21 manual: AICFR resets with TFTH 8 and
 * AICTFLR with 8; I2SSR reads BSY, RBSY and TBSY from reset, then BSY and
 * CHBSY for good once the AIC has run, plus RBSY and TBSY while EREC and
 * ERPL are set; an empty FIFO reads its stale entry at the read pointer;
 * the loop fills with RX, TLDMS or not, and not while RX is full. Every
 * codec bit the vendor driver writes reads back.
 *
 * Where the CPM's I2S clocks are wired (T23, T31, T40, T41 and A1), the
 * AIC is the I2S master: play runs at the transmit clock and record at
 * the receive clock, each at its own rate, as a T31X and a T41NQ do.
 * Elsewhere the internal codec is the master of both, at its sample rate
 * (CSRR). Frames are played and recorded in virtual time; between FIFO
 * thresholds nothing changes that software can see, so each clock only
 * wakes at its next threshold crossing, an underrun or an overrun.
 *
 * Played frames go to the audiodev. The mic is the audiodev's input, a
 * sine ("tone[:<Hz>[:<dBFS>]]", 1 kHz at -20 dBFS by default) or a looped
 * 16-bit PCM WAV file, set by the "mic" property. The codec's analog
 * settings (gains, mutes, routing) are registers only: samples pass
 * through unscaled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/audio/ingenic-aic.h"

#define AICFR       0x00
#define AICCR       0x04
#define I2SCR       0x10
#define AICSR       0x14
#define I2SSR       0x1c
#define I2SDIV      0x30
#define AICDR       0x34
#define AICLR       0x38
#define AICTFLR     0x3c
#define CKCFG       0xa0
#define RGADW       0xa4
#define RGDATA      0xa8

#define AICFR_ENB       BIT(0)
#define AICFR_RST       BIT(3)
#define AICFR_ICDC      BIT(5)
#define AICFR_LSMP      BIT(6)
#define AICFR_MASK      0x8f1f1ff7
#define AICFR_TFTH(v)   extract32(v, 16, 5)
#define AICFR_RFTH(v)   extract32(v, 24, 4)

#define AICCR_EREC      BIT(0)
#define AICCR_ERPL      BIT(1)
#define AICCR_ENLBF     BIT(2)
#define AICCR_ETFS      BIT(3)
#define AICCR_ERFS      BIT(4)
#define AICCR_ETUR      BIT(5)
#define AICCR_EROR      BIT(6)
#define AICCR_RFLUSH    BIT(7)
#define AICCR_TFLUSH    BIT(8)
#define AICCR_ASVTSU    BIT(9)
#define AICCR_ENDSW     BIT(10)
#define AICCR_RDMS      BIT(15)
#define AICCR_TDMS      BIT(14)
#define AICCR_TLDMS     BIT(22)
#define AICCR_ETFL      BIT(23)
#define AICCR_PACK16    BIT(28)
#define AICCR_ETFLS     BIT(30)
#define AICCR_ETFLOR    BIT(31)
#define AICCR_MASK      0xd7fffe7f
#define AICCR_MONOCTR(v)    extract32(v, 12, 2)
#define AICCR_ISS(v)        extract32(v, 16, 3)
#define AICCR_OSS(v)        extract32(v, 19, 3)
#define AICCR_CHANNEL(v)    extract32(v, 24, 3)

#define I2SCR_SWLH      BIT(16)
#define I2SCR_RFIRST    BIT(17)
#define I2SCR_MASK      0x00033011

#define AICSR_TFLS      BIT(1)
#define AICSR_TFLOR     BIT(2)
#define AICSR_TFS       BIT(3)
#define AICSR_RFS       BIT(4)
#define AICSR_TUR       BIT(5)
#define AICSR_ROR       BIT(6)
#define AICSR_STICKY    (AICSR_TFLOR | AICSR_TUR | AICSR_ROR)

#define I2SSR_BSY       BIT(2)
#define I2SSR_RBSY      BIT(3)
#define I2SSR_TBSY      BIT(4)
#define I2SSR_CHBSY     BIT(5)

#define RGADW_WR        BIT(16)

/* The T31 codec's registers behind its ALC gain readout */
#define T31_CMICCR      0x23
#define CMICCR_ALC      BIT(4)
#define T31_CALCGR      0x25    /* the ALC gain, bits 4:0 */

/* Sample sizes by ISS/OSS; 5..7 are reserved */
static const uint8_t aic_bits[8] = { 8, 16, 18, 20, 24, 16, 16, 16 };
static const uint32_t codec_rates[8] = {
    96000, 48000, 44100, 32000, 24000, 16000, 12000, 8000,
};

/* The T31 codec after reset, as read on a T31X, by register number */
static const uint8_t codec_reset_t31[INGENIC_AIC_CODEC_REGS] = {
    [0x00] = 0x03, [0x01] = 0x86, [0x02] = 0x50, [0x03] = 0x0e,
    [0x04] = 0x50, [0x05] = 0x0e, [0x06] = 0x22, [0x07] = 0x01,
    [0x08] = 0xc3, [0x09] = 0x0c, [0x0a] = 0xf1, [0x0b] = 0xf1,
    [0x20] = 0x30, [0x25] = 0x0c,
    [0x41] = 0x46, [0x42] = 0x41, [0x43] = 0x2c, [0x45] = 0x26,
    [0x46] = 0x40, [0x47] = 0x36, [0x48] = 0x20, [0x49] = 0x38,
    [0x4b] = 0x10,
};

/* The T40's, as read on a T40NN; the DAC's bank at 0x50 starts as the ADC's */
static const uint8_t codec_reset_t40[INGENIC_AIC_CODEC_REGS] = {
    [0x00] = 0x03, [0x01] = 0x56, [0x02] = 0x50, [0x03] = 0x0e,
    [0x04] = 0x50, [0x05] = 0x0e, [0x06] = 0xea, [0x07] = 0x01,
    [0x08] = 0xc3, [0x09] = 0xc3, [0x0a] = 0x0c, [0x0b] = 0xf1,
    [0x0c] = 0xf1, [0x0d] = 0x03, [0x0e] = 0xc3,
    [0x20] = 0xc0, [0x26] = 0x0c, [0x27] = 0x0c, [0x2a] = 0x05,
    [0x41] = 0x46, [0x42] = 0x41, [0x43] = 0x2c, [0x44] = 0xf0,
    [0x45] = 0x26, [0x46] = 0x40, [0x47] = 0x36, [0x48] = 0x20,
    [0x49] = 0x38, [0x4b] = 0x01, [0x4c] = 0x10, [0x4d] = 0x0f,
    [0x51] = 0x46, [0x52] = 0x41, [0x53] = 0x2c, [0x54] = 0xf0,
    [0x55] = 0x26, [0x56] = 0x40, [0x57] = 0x36, [0x58] = 0x20,
    [0x59] = 0x38, [0x5b] = 0x01, [0x5c] = 0x10, [0x5d] = 0x0f,
};

/* The T41's, as read on a T41NQ */
static const uint8_t codec_reset_t41[INGENIC_AIC_CODEC_REGS] = {
    [0x00] = 0x03, [0x01] = 0x56, [0x02] = 0x50, [0x03] = 0x0e,
    [0x04] = 0x50, [0x05] = 0x0e, [0x06] = 0xf1, [0x07] = 0x01,
    [0x08] = 0xc3, [0x09] = 0xc3, [0x0a] = 0x80, [0x0b] = 0xf1,
    [0x0c] = 0xf1, [0x0d] = 0x03, [0x0e] = 0xc3,
    [0x20] = 0xc7, [0x24] = 0x07, [0x26] = 0x0c, [0x27] = 0x0c,
    [0x29] = 0x10, [0x2a] = 0x08, [0x2c] = 0x10, [0x2d] = 0x08,
    [0x41] = 0x46, [0x42] = 0x41, [0x43] = 0x2c, [0x45] = 0x26,
    [0x46] = 0x40, [0x47] = 0x36, [0x48] = 0x20, [0x49] = 0x38,
    [0x4a] = 0xf0, [0x4b] = 0x01, [0x4c] = 0x10, [0x4d] = 0x0f,
};

/*
 * The internal codecs, by what of them the model needs, and the AIC that
 * goes with each where it differs. Register N sits at byte 4N. Their vendor
 * drivers write, read back and wait: no register reports a state, and the
 * frame rate is the rate code (bits 2:0) the driver programs. A mono ADC
 * sends the mic in one slot and zero in the other; where a codec has no
 * mono bit, the mic is in both.
 */
struct IngenicAicCodec {
    const char *name;
    uint8_t rate_reg;
    int8_t mono_reg;            /* bit 0 makes the ADC mono; -1, no such bit */
    uint8_t mono_slot;          /* ... and the mic's slot then: 0 L, 1 R */
    int8_t agc_gain_reg;        /* read only: the ALC gain now; -1, none */
    const uint8_t *reset;       /* after reset; NULL, all zero */
    uint8_t i2ssr_idle;         /* I2SSR until the AIC first runs */
    uint32_t i2sdiv;            /* I2SDIV after reset */
};

/* As read on a T31X; a T41NQ and an A1N read 0 and 0x00040004 */
#define I2SSR_IDLE  (I2SSR_BSY | I2SSR_RBSY | I2SSR_TBSY)

static const IngenicAicCodec aic_codecs[] = {
    /* T10, T20, T21, T30 */
    { "t10", 0x44, 0x02, 1, -1, NULL, I2SSR_IDLE, 0x00030003 },
    { "t31", 0x44, 0x02, 1, 0x4c, codec_reset_t31, I2SSR_IDLE, 0x00030003 },
    { "t23", 0x44, 0x02, 1, -1, NULL, I2SSR_IDLE, 0x00030003 },
    /* T32, T33 */
    { "t32", 0x4a, -1, 0, -1, NULL, I2SSR_IDLE, 0x00030003 },
    { "t40", 0x44, 0x02, 0, -1, codec_reset_t40, I2SSR_IDLE, 0x00030003 },
    { "t41", 0x44, -1, 0, -1, codec_reset_t41, 0, 0x00040004 },
    { "a1", 0x44, 0x02, 0, -1, NULL, 0, 0x00040004 },
};

static void aic_schedule(IngenicAicState *s);

static unsigned aic_tx_thr(IngenicAicState *s)
{
    return AICFR_TFTH(s->aicfr) * 2;
}

static unsigned aic_rx_thr(IngenicAicState *s)
{
    return (AICFR_RFTH(s->aicfr) + 1) * 2;
}

static unsigned aic_loop_thr(IngenicAicState *s)
{
    return (extract32(s->aictflr, 0, 4) + 1) * 2;
}

/* TX FIFO entries a frame takes: one in mono or packed stereo */
static unsigned aic_tx_spf(IngenicAicState *s)
{
    if (AICCR_CHANNEL(s->aiccr) == 0) {
        return 1;
    }
    return (s->aiccr & AICCR_PACK16) && aic_bits[AICCR_OSS(s->aiccr)] == 16
           ? 1 : 2;
}

/* RX FIFO entries a frame gives: MONOCTR keeps one channel */
static unsigned aic_rx_spf(IngenicAicState *s)
{
    unsigned m = AICCR_MONOCTR(s->aiccr);

    return m == 1 || m == 2 ? 1 : 2;
}

static void aic_update(IngenicAicState *s)
{
    bool enb = s->aicfr & AICFR_ENB;
    bool tfs = s->tx_n <= aic_tx_thr(s);
    bool rfs = s->rx_n >= aic_rx_thr(s);
    bool tfls = s->loop_n >= aic_loop_thr(s);
    uint32_t sr = s->aicsr & AICSR_STICKY;
    uint32_t irq = 0;

    sr = deposit32(sr, 24, 6, s->rx_n);
    sr = deposit32(sr, 15, 6, s->loop_n);
    sr = deposit32(sr, 8, 7, s->tx_n);
    sr |= (tfs ? AICSR_TFS : 0) | (rfs ? AICSR_RFS : 0) |
          (tfls ? AICSR_TFLS : 0);
    s->aicsr = sr;

    qemu_set_irq(s->dreq[INGENIC_AIC_DREQ_TX],
                 enb && (s->aiccr & AICCR_TDMS) && tfs);
    qemu_set_irq(s->dreq[INGENIC_AIC_DREQ_RX],
                 enb && (s->aiccr & AICCR_RDMS) && rfs);
    qemu_set_irq(s->dreq[INGENIC_AIC_DREQ_LOOP],
                 enb && (s->aiccr & AICCR_TLDMS) && tfls);

    irq |= (s->aiccr & AICCR_ETFS) && tfs;
    irq |= (s->aiccr & AICCR_ERFS) && rfs;
    irq |= (s->aiccr & AICCR_ETFLS) && tfls;
    irq |= (s->aiccr & AICCR_ETUR) && (sr & AICSR_TUR);
    irq |= (s->aiccr & AICCR_EROR) && (sr & AICSR_ROR);
    irq |= (s->aiccr & AICCR_ETFLOR) && (sr & AICSR_TFLOR);
    qemu_set_irq(s->irq, irq);
}

/* --- Samples ------------------------------------------------------------ */

/* A FIFO entry of the given size to S16, as the codec hears it */
static int16_t aic_out_sample(IngenicAicState *s, uint32_t v, unsigned bits)
{
    int32_t x;

    if (s->aiccr & AICCR_ASVTSU) {
        v ^= 1u << (bits - 1);
    }
    if (bits == 16 && (s->aiccr & AICCR_ENDSW)) {
        v = bswap16(v);
    }
    x = sextract32(v, 0, bits);
    return bits >= 16 ? x >> (bits - 16) : x << (16 - bits);
}

/* An S16 sample to a FIFO entry of the given size: no sign extension */
static uint32_t aic_in_sample(IngenicAicState *s, int16_t x, unsigned bits)
{
    int32_t v = bits >= 16 ? (int32_t)x * (1 << (bits - 16)) : x >> (16 - bits);
    uint32_t e = extract32(v, 0, bits);

    if (s->aiccr & AICCR_ASVTSU) {
        e ^= 1u << (bits - 1);
    }
    return e;
}

static void aic_push(uint32_t *fifo, unsigned depth, uint32_t *rd,
                     uint32_t *n, uint32_t v)
{
    fifo[(*rd + *n) % depth] = v;
    (*n)++;
}

static uint32_t aic_pop(uint32_t *fifo, unsigned depth, uint32_t *rd,
                        uint32_t *n)
{
    uint32_t v = fifo[*rd];

    *rd = (*rd + 1) % depth;
    (*n)--;
    return v;
}

/*
 * The frame the codec plays next, as S16 and as the FIFO entry each
 * channel came from. An underrun plays zero or, with LSMP, the last frame
 * again.
 */
static void aic_play_frame(IngenicAicState *s, int16_t out[2], uint32_t raw[2])
{
    unsigned bits = aic_bits[AICCR_OSS(s->aiccr)];
    unsigned spf = aic_tx_spf(s);

    if (s->tx_n < spf) {
        bool lsmp = s->aicfr & AICFR_LSMP;

        s->aicsr |= AICSR_TUR;
        for (int c = 0; c < 2; c++) {
            out[c] = lsmp ? s->last[c] : 0;
            raw[c] = lsmp ? s->last_raw[c] : 0;
        }
        return;
    }
    if (AICCR_CHANNEL(s->aiccr) == 0) {
        raw[0] = raw[1] = aic_pop(s->tx, INGENIC_AIC_TX_DEPTH, &s->tx_rd,
                                  &s->tx_n);
    } else if (spf == 1) {
        uint32_t v = aic_pop(s->tx, INGENIC_AIC_TX_DEPTH, &s->tx_rd, &s->tx_n);
        int lo = !!(s->i2scr & I2SCR_SWLH);

        raw[lo] = v & 0xffff;
        raw[!lo] = v >> 16;
        bits = 16;
    } else {
        int first = !!(s->i2scr & I2SCR_RFIRST);

        raw[first] = aic_pop(s->tx, INGENIC_AIC_TX_DEPTH, &s->tx_rd, &s->tx_n);
        raw[!first] = aic_pop(s->tx, INGENIC_AIC_TX_DEPTH, &s->tx_rd,
                              &s->tx_n);
    }
    for (int c = 0; c < 2; c++) {
        out[c] = aic_out_sample(s, raw[c], bits);
        s->last[c] = out[c];
        s->last_raw[c] = raw[c];
    }
}

/* Queue a frame for reading: both channels, or the one MONOCTR keeps */
static void aic_queue(IngenicAicState *s, uint32_t *fifo, unsigned depth,
                      uint32_t *rd, uint32_t *n, const uint32_t e[2])
{
    unsigned m = AICCR_MONOCTR(s->aiccr);

    if (m != 1) {
        aic_push(fifo, depth, rd, n, e[0]);
    }
    if (m != 2) {
        aic_push(fifo, depth, rd, n, e[1]);
    }
}

/* A play clock frame: the codec plays one, silence when not playing */
static void aic_tx_frame(IngenicAicState *s)
{
    int16_t out[2] = { 0, 0 };
    uint32_t raw[2] = { 0, 0 };

    if (s->aiccr & AICCR_ERPL) {
        aic_play_frame(s, out, raw);
    }
    if (s->out_n == INGENIC_AIC_RING) {
        /* The host fell behind: drop the oldest frame */
        s->out_rd = (s->out_rd + 1) % INGENIC_AIC_RING;
        s->out_n--;
    }
    s->out_ring[(s->out_rd + s->out_n) % INGENIC_AIC_RING][0] = out[0];
    s->out_ring[(s->out_rd + s->out_n) % INGENIC_AIC_RING][1] = out[1];
    s->out_n++;
    memcpy(s->played, out, sizeof(out));
    memcpy(s->played_raw, raw, sizeof(raw));
}

/* A record clock frame: the mic's, or with loopback the frame playing */
static void aic_rx_frame(IngenicAicState *s)
{
    int16_t mic[2];

    if (!(s->aiccr & AICCR_EREC)) {
        return;
    }
    if (s->rx_n + aic_rx_spf(s) > INGENIC_AIC_RX_DEPTH) {
        s->aicsr |= AICSR_ROR;
        return;
    }
    if (s->aiccr & AICCR_ENLBF) {
        mic[0] = s->played[0];
        mic[1] = s->played[1];
    } else {
        ingenic_mic_frame(&s->mic_src, s->rx_rate, mic);
        /*
         * The SDK's oss2 driver on the T10, T20 and T30 adds the two slots
         * of a mono ADC: the silent one must be zero
         */
        if (s->cdc->mono_reg >= 0 && (s->codec[s->cdc->mono_reg] & 1)) {
            mic[!s->cdc->mono_slot] = 0;
        }
    }
    {
        unsigned bits = aic_bits[AICCR_ISS(s->aiccr)];
        uint32_t e[2] = {
            aic_in_sample(s, mic[0], bits), aic_in_sample(s, mic[1], bits),
        };

        aic_queue(s, s->rx, INGENIC_AIC_RX_DEPTH, &s->rx_rd, &s->rx_n, e);
    }
    /*
     * The TFIFO loop records what is playing beside it, as written. TFLOR
     * comes up as it fills, not on reading it empty as the manual says.
     */
    if ((s->aiccr & AICCR_ETFL) &&
        s->loop_n + aic_rx_spf(s) <= INGENIC_AIC_LOOP_DEPTH) {
        aic_queue(s, s->loop, INGENIC_AIC_LOOP_DEPTH, &s->loop_rd,
                  &s->loop_n, s->played_raw);
        if (s->loop_n == INGENIC_AIC_LOOP_DEPTH) {
            s->aicsr |= AICSR_TFLOR;
        }
    }
}

/* --- The frame clock ---------------------------------------------------- */

/*
 * A direction's rate: its CPM clock at 256 fs where one is wired and
 * running (the A1's driver leaves the codec's rate code alone), else the
 * codec's rate code
 */
static uint32_t aic_clk_rate(IngenicAicState *s, Clock *clk)
{
    uint32_t rate = clock_has_source(clk) ? clock_get_hz(clk) / 256 : 0;

    return rate ? rate : codec_rates[s->codec[s->cdc->rate_reg] & 7];
}

static void aic_out_cb(void *opaque, int avail);

/* The host voices run at the guest's rates; the audiodev resamples */
static void aic_open_voices(IngenicAicState *s)
{
    struct audsettings as = {
        .freq = s->rate,
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = HOST_BIG_ENDIAN,
    };

    if (!s->audio_be) {
        return;
    }
    ingenic_mic_open(&s->mic_src, s->audio_be, s->rx_rate);
    if (s->voice_rate == s->rate) {
        return;
    }
    s->voice_rate = s->rate;
    s->out_rd = s->out_n = 0;
    s->voice_out = audio_be_open_out(s->audio_be, s->voice_out,
                                     "ingenic-aic.out", s, aic_out_cb, &as);
}

static void aic_clock_update(IngenicAicState *s)
{
    bool run = (s->aicfr & AICFR_ENB) &&
               (s->aiccr & (AICCR_ERPL | AICCR_EREC));
    uint32_t rate = aic_clk_rate(s, s->tx_clk);
    uint32_t rx_rate = aic_clk_rate(s, s->rx_clk);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (run && (!s->running || rate != s->rate)) {
        s->t0 = now;
        s->frames = 0;
        s->rate = rate;
    }
    if (run && (!s->running || rx_rate != s->rx_rate)) {
        s->rx_t0 = now;
        s->rx_frames = 0;
        s->rx_rate = rx_rate;
    }
    if (run) {
        aic_open_voices(s);
    }
    s->running = run;
    s->ran |= run;
    if (s->voice_out) {
        audio_be_set_active_out(s->audio_be, s->voice_out, run);
    }
    /* The host mic only listens while the guest records */
    ingenic_mic_set_active(&s->mic_src, run && (s->aiccr & AICCR_EREC));
    if (run) {
        aic_schedule(s);
    } else {
        timer_del(s->timer);
    }
}

static bool aic_dma_wanted(IngenicAicState *s)
{
    return (s->aicfr & AICFR_ENB) &&
           (((s->aiccr & AICCR_TDMS) && s->tx_n <= aic_tx_thr(s)) ||
            ((s->aiccr & AICCR_RDMS) && s->rx_n >= aic_rx_thr(s)) ||
            ((s->aiccr & AICCR_TLDMS) && s->loop_n >= aic_loop_thr(s)));
}

static int64_t aic_frame_ns(int64_t t0, uint64_t frame, uint32_t rate)
{
    return t0 + muldiv64(frame, NANOSECONDS_PER_SECOND, rate);
}

static void aic_tick(void *opaque)
{
    IngenicAicState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t tx_due, rx_due;

    if (!s->running) {
        return;
    }
    tx_due = muldiv64(now - s->t0, s->rate, NANOSECONDS_PER_SECOND);
    rx_due = muldiv64(now - s->rx_t0, s->rx_rate, NANOSECONDS_PER_SECOND);
    while (s->frames < tx_due || s->rx_frames < rx_due) {
        /* The earlier frame first; at the same time, play before record */
        bool tx = s->frames < tx_due &&
                  (s->rx_frames >= rx_due ||
                   aic_frame_ns(s->t0, s->frames, s->rate) <=
                   aic_frame_ns(s->rx_t0, s->rx_frames, s->rx_rate));

        if (tx) {
            aic_tx_frame(s);
            s->frames++;
        } else {
            aic_rx_frame(s);
            s->rx_frames++;
        }
        /*
         * A late timer owes more frames than the FIFOs hold. The silicon's
         * DMA keeps up between frames, so let the PDMA run here too rather
         * than overrun RX or underrun TX on the guest.
         */
        if ((s->frames < tx_due || s->rx_frames < rx_due) && s->pdma &&
            aic_dma_wanted(s)) {
            aic_update(s);
            ingenic_pdma_kick(s->pdma);
        }
    }
    aic_update(s);
    aic_schedule(s);
}

/*
 * Wake when software could see something change: a FIFO crossing its
 * request threshold, or with a request standing, running dry or full.
 * At least every 10 ms, to keep the host side flowing.
 */
static void aic_schedule(IngenicAicState *s)
{
    uint64_t k = MAX(s->rate / 100, 1), rk = MAX(s->rx_rate / 100, 1);

    if (s->aiccr & AICCR_ERPL) {
        unsigned spf = aic_tx_spf(s), thr = aic_tx_thr(s);

        k = MIN(k, s->tx_n > thr ? DIV_ROUND_UP(s->tx_n - thr, spf)
                                 : s->tx_n / spf + 1);
    }
    if (s->aiccr & AICCR_EREC) {
        unsigned spf = aic_rx_spf(s), thr = aic_rx_thr(s);

        rk = MIN(rk, s->rx_n < thr ? DIV_ROUND_UP(thr - s->rx_n, spf)
                     : (INGENIC_AIC_RX_DEPTH - s->rx_n) / spf + 1);
    }
    if ((s->aiccr & (AICCR_ETFL | AICCR_EREC)) ==
        (AICCR_ETFL | AICCR_EREC)) {
        unsigned spf = aic_rx_spf(s), thr = aic_loop_thr(s);

        rk = MIN(rk, s->loop_n < thr ? DIV_ROUND_UP(thr - s->loop_n, spf)
                     : (INGENIC_AIC_LOOP_DEPTH - s->loop_n) / spf + 1);
    }
    timer_mod(s->timer,
              MIN(aic_frame_ns(s->t0, s->frames + MAX(k, 1), s->rate),
                  aic_frame_ns(s->rx_t0, s->rx_frames + MAX(rk, 1),
                               s->rx_rate)) + 1);
}

/* --- Host audio ------------------------------------------------------- */

static void aic_out_cb(void *opaque, int avail)
{
    IngenicAicState *s = opaque;
    uint32_t frames = MIN((uint32_t)avail / 4, s->out_n);

    while (frames) {
        uint32_t chunk = MIN(frames, INGENIC_AIC_RING - s->out_rd);
        size_t done = audio_be_write(s->audio_be, s->voice_out,
                                     s->out_ring[s->out_rd], chunk * 4) / 4;

        s->out_rd = (s->out_rd + done) % INGENIC_AIC_RING;
        s->out_n -= done;
        frames -= done;
        if (done < chunk) {
            break;
        }
    }
}

/* --- Registers ---------------------------------------------------------- */

static void aic_reset_regs(IngenicAicState *s)
{
    /* What AICFR.RST resets: all but AICFR and I2SDIV */
    s->aiccr = 0x01240000;
    s->i2scr = 0;
    s->aicsr = 0;
    s->aictflr = 8;
    s->tx_rd = s->tx_n = 0;
    s->rx_rd = s->rx_n = 0;
    s->loop_rd = s->loop_n = 0;
}

static uint64_t aic_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAicState *s = opaque;
    uint32_t v;

    switch (addr) {
    case AICFR:
        return s->aicfr;
    case AICCR:
        return s->aiccr;
    case I2SCR:
        return s->i2scr;
    case AICSR:
        return s->aicsr;
    case I2SSR:
        /*
         * As read on a T31X: BSY, RBSY and TBSY from reset (0 on a T41NQ
         * and an A1N); once the AIC has run, BSY and CHBSY for good, RBSY
         * and TBSY with EREC and ERPL
         */
        if (!s->ran) {
            return s->cdc->i2ssr_idle;
        }
        v = I2SSR_BSY | I2SSR_CHBSY;
        v |= s->aiccr & AICCR_EREC ? I2SSR_RBSY : 0;
        v |= s->aiccr & AICCR_ERPL ? I2SSR_TBSY : 0;
        return v;
    case I2SDIV:
        return s->i2sdiv;
    case AICDR:
        /* Empty, it reads the stale entry at the read pointer */
        if (!s->rx_n) {
            return s->rx[s->rx_rd];
        }
        v = aic_pop(s->rx, INGENIC_AIC_RX_DEPTH, &s->rx_rd, &s->rx_n);
        aic_update(s);
        if (s->rx_n + 1 == aic_rx_thr(s) && s->running) {
            aic_schedule(s);
        }
        return v;
    case AICLR:
        if (!s->loop_n) {
            return s->loop[s->loop_rd];
        }
        v = aic_pop(s->loop, INGENIC_AIC_LOOP_DEPTH, &s->loop_rd, &s->loop_n);
        aic_update(s);
        if (s->loop_n + 1 == aic_loop_thr(s) && s->running) {
            aic_schedule(s);
        }
        return v;
    case AICTFLR:
        return s->aictflr;
    case CKCFG:
        return s->ckcfg;
    case RGADW:
        return s->rgadw;
    case RGDATA:
        return s->codec[extract32(s->rgadw, 8, 7) % INGENIC_AIC_CODEC_REGS];
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aic: read of 0x%" HWADDR_PRIx
                      "\n", addr);
        return 0;
    }
}

static void aic_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    IngenicAicState *s = opaque;

    switch (addr) {
    case AICFR:
        if (value & AICFR_RST) {
            aic_reset_regs(s);
        }
        s->aicfr = value & AICFR_MASK;
        if (!(s->aicfr & AICFR_ICDC)) {
            qemu_log_mask(LOG_UNIMP, "ingenic-aic: only the internal codec "
                          "is modelled\n");
        }
        break;
    case AICCR:
        if (value & AICCR_TFLUSH) {
            s->tx_rd = s->tx_n = 0;
        }
        if (value & AICCR_RFLUSH) {
            s->rx_rd = s->rx_n = 0;
        }
        s->aiccr = value & AICCR_MASK;
        break;
    case I2SCR:
        s->i2scr = value & I2SCR_MASK;
        break;
    case AICSR:
        /* ROR, TUR and TFLOR clear when written 0 */
        s->aicsr &= ~(~value & AICSR_STICKY);
        break;
    case I2SDIV:
        s->i2sdiv = value & 0x01ff01ff;
        break;
    case AICDR:
        if (s->tx_n == INGENIC_AIC_TX_DEPTH) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aic: TX FIFO overflow\n");
            return;
        }
        aic_push(s->tx, INGENIC_AIC_TX_DEPTH, &s->tx_rd, &s->tx_n,
                 value & 0xffffffff);
        aic_update(s);
        if (s->tx_n == aic_tx_thr(s) + 1 && s->running) {
            aic_schedule(s);
        }
        return;
    case AICTFLR:
        s->aictflr = value & 0xf;
        break;
    case CKCFG:
        s->ckcfg = value;
        break;
    case RGADW:
        /* A write completes at once: drivers spin until RGWR reads 0 */
        s->rgadw = value & 0x7fff;
        if (value & RGADW_WR) {
            s->codec[extract32(value, 8, 7) % INGENIC_AIC_CODEC_REGS] =
                value & 0xff;
            aic_clock_update(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aic: write of 0x%" PRIx64
                      " to 0x%" HWADDR_PRIx "\n", value, addr);
        return;
    }
    aic_update(s);
    aic_clock_update(s);
}

static const MemoryRegionOps aic_ops = {
    .read = aic_read,
    .write = aic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static uint64_t codec_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAicState *s = opaque;

    if (s->cdc->agc_gain_reg >= 0 && addr >> 2 == s->cdc->agc_gain_reg) {
        /*
         * The gain the ALC applies now, which with a steady mic is the one
         * CALCGR sets (0x0c on a T31X while recording)
         */
        return s->codec[T31_CMICCR] & CMICCR_ALC
               ? s->codec[T31_CALCGR] & 0x1f : 0;
    }
    return s->codec[addr >> 2];
}

static void codec_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    IngenicAicState *s = opaque;

    if (s->cdc->agc_gain_reg >= 0 && addr >> 2 == s->cdc->agc_gain_reg) {
        return;
    }
    s->codec[addr >> 2] = value & 0xff;
    if (addr >> 2 == s->cdc->rate_reg) {
        aic_clock_update(s);
    }
}

static const MemoryRegionOps codec_ops = {
    .read = codec_read,
    .write = codec_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/* --- Lifecycle ---------------------------------------------------------- */

static void aic_realize(DeviceState *dev, Error **errp)
{
    IngenicAicState *s = INGENIC_AIC(dev);
    Error *err = NULL;

    for (int i = 0; i < ARRAY_SIZE(aic_codecs); i++) {
        if (!strcmp(s->codec_name ?: "t31", aic_codecs[i].name)) {
            s->cdc = &aic_codecs[i];
        }
    }
    if (!s->cdc) {
        error_setg(errp, "ingenic-aic: no codec '%s'", s->codec_name);
        return;
    }

    if (!ingenic_mic_init(&s->mic_src, TYPE_INGENIC_AIC, s->mic, errp)) {
        return;
    }
    /*
     * Every T-series SoC model realizes the AIC, mapped or not, so an
     * audiodev that cannot start costs the host side only. The voices
     * open when the clock first runs.
     */
    if (!audio_be_check(&s->audio_be, &err)) {
        warn_report_err(err);
    }
}

static void aic_unrealize(DeviceState *dev)
{
    IngenicAicState *s = INGENIC_AIC(dev);

    ingenic_mic_cleanup(&s->mic_src);
}

static void aic_reset_hold(Object *obj, ResetType type)
{
    IngenicAicState *s = INGENIC_AIC(obj);

    timer_del(s->timer);
    s->running = false;
    s->ran = false;
    s->aicfr = 0x07080000;      /* TFTH 8 on a T31X; the T21 manual says 16 */
    s->i2sdiv = s->cdc->i2sdiv;
    s->ckcfg = 0;
    s->rgadw = 0;
    aic_reset_regs(s);
    memset(s->last, 0, sizeof(s->last));
    memset(s->last_raw, 0, sizeof(s->last_raw));
    memset(s->played, 0, sizeof(s->played));
    memset(s->played_raw, 0, sizeof(s->played_raw));
    for (int i = 0; i < INGENIC_AIC_CODEC_REGS; i++) {
        s->codec[i] = s->cdc->reset ? s->cdc->reset[i] : 0;
    }
}

static void aic_reset_exit(Object *obj, ResetType type)
{
    aic_update(INGENIC_AIC(obj));
}

static void aic_clk_changed(void *opaque, ClockEvent event)
{
    aic_clock_update(opaque);
}

static void aic_init(Object *obj)
{
    IngenicAicState *s = INGENIC_AIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &aic_ops, s, "ingenic-aic", 0x1000);
    memory_region_init_io(&s->codec_iomem, obj, &codec_ops, s,
                          "ingenic-aic.codec", 4 * INGENIC_AIC_CODEC_REGS);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_mmio(sbd, &s->codec_iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->dreq, "dreq",
                             INGENIC_AIC_NR_DREQ);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, aic_tick, s);
    s->tx_clk = qdev_init_clock_in(DEVICE(obj), "i2s-tx", aic_clk_changed, s,
                                   ClockUpdate);
    s->rx_clk = qdev_init_clock_in(DEVICE(obj), "i2s-rx", aic_clk_changed, s,
                                   ClockUpdate);
}

static int aic_post_load(void *opaque, int version_id)
{
    IngenicAicState *s = opaque;

    if (version_id < 3) {
        /* One clock for both ways before */
        s->rx_rate = s->rate;
        s->rx_t0 = s->t0;
        s->rx_frames = s->frames;
    }
    if (s->running) {
        aic_open_voices(s);
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_aic = {
    .name = TYPE_INGENIC_AIC,
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = aic_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(aicfr, IngenicAicState),
        VMSTATE_UINT32(aiccr, IngenicAicState),
        VMSTATE_UINT32(i2scr, IngenicAicState),
        VMSTATE_UINT32(aicsr, IngenicAicState),
        VMSTATE_UINT32(i2sdiv, IngenicAicState),
        VMSTATE_UINT32(aictflr, IngenicAicState),
        VMSTATE_UINT32(ckcfg, IngenicAicState),
        VMSTATE_UINT32(rgadw, IngenicAicState),
        VMSTATE_UINT32_ARRAY(tx, IngenicAicState, INGENIC_AIC_TX_DEPTH),
        VMSTATE_UINT32(tx_rd, IngenicAicState),
        VMSTATE_UINT32(tx_n, IngenicAicState),
        VMSTATE_UINT32_ARRAY(rx, IngenicAicState, INGENIC_AIC_RX_DEPTH),
        VMSTATE_UINT32(rx_rd, IngenicAicState),
        VMSTATE_UINT32(rx_n, IngenicAicState),
        VMSTATE_UINT32_ARRAY(loop, IngenicAicState, INGENIC_AIC_LOOP_DEPTH),
        VMSTATE_UINT32(loop_rd, IngenicAicState),
        VMSTATE_UINT32(loop_n, IngenicAicState),
        VMSTATE_INT16_ARRAY(last, IngenicAicState, 2),
        VMSTATE_UINT32_ARRAY(last_raw, IngenicAicState, 2),
        VMSTATE_BOOL(ran, IngenicAicState),
        VMSTATE_UINT32_ARRAY(codec, IngenicAicState, INGENIC_AIC_CODEC_REGS),
        VMSTATE_BOOL(running, IngenicAicState),
        VMSTATE_UINT32(rate, IngenicAicState),
        VMSTATE_INT64(t0, IngenicAicState),
        VMSTATE_UINT64(frames, IngenicAicState),
        VMSTATE_TIMER_PTR(timer, IngenicAicState),
        VMSTATE_CLOCK_V(tx_clk, IngenicAicState, 2),
        VMSTATE_CLOCK_V(rx_clk, IngenicAicState, 2),
        VMSTATE_UINT32_V(rx_rate, IngenicAicState, 3),
        VMSTATE_INT64_V(rx_t0, IngenicAicState, 3),
        VMSTATE_UINT64_V(rx_frames, IngenicAicState, 3),
        VMSTATE_INT16_ARRAY_V(played, IngenicAicState, 2, 3),
        VMSTATE_UINT32_ARRAY_V(played_raw, IngenicAicState, 2, 3),
        VMSTATE_END_OF_LIST()
    }
};

static const Property aic_props[] = {
    DEFINE_AUDIO_PROPERTIES(IngenicAicState, audio_be),
    DEFINE_PROP_STRING("mic", IngenicAicState, mic),
    DEFINE_PROP_STRING("codec", IngenicAicState, codec_name),
    DEFINE_PROP_LINK("pdma", IngenicAicState, pdma, TYPE_INGENIC_PDMA,
                     IngenicPdmaState *),
};

static void aic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = aic_realize;
    dc->unrealize = aic_unrealize;
    dc->vmsd = &vmstate_ingenic_aic;
    rc->phases.hold = aic_reset_hold;
    rc->phases.exit = aic_reset_exit;
    device_class_set_props(dc, aic_props);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo aic_type_info = {
    .name = TYPE_INGENIC_AIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAicState),
    .instance_init = aic_init,
    .class_init = aic_class_init,
};

static void aic_register_types(void)
{
    type_register_static(&aic_type_info);
}

type_init(aic_register_types)
