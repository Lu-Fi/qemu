/*
 * The microphone behind an Ingenic audio input: a sine, a looped WAV file
 * or an audiodev's input
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_AUDIO_INGENIC_MIC_H
#define HW_AUDIO_INGENIC_MIC_H

#include "qemu/audio.h"

#define INGENIC_MIC_RING    8192        /* audiodev frames held for the guest */

typedef struct IngenicMic {
    const char *owner;          /* names errors and the audiodev voice */

    double tone_hz;
    double tone_level;
    double phase;

    int16_t (*wav)[2];
    uint32_t wav_frames, wav_rate;
    double wav_pos;

    AudioBackend *audio_be;
    char *voice_name;
    SWVoiceIn *voice;
    uint32_t voice_rate;
    int16_t ring[INGENIC_MIC_RING][2];
    uint32_t rd, n;
} IngenicMic;

/* "tone[:<Hz>[:<dBFS>]]", a 16-bit PCM WAV file, or NULL for the audiodev */
bool ingenic_mic_init(IngenicMic *m, const char *owner, const char *spec,
                      Error **errp);

/* Without a tone or a WAV, listen on the audiodev at this rate */
void ingenic_mic_open(IngenicMic *m, AudioBackend *be, uint32_t rate);
void ingenic_mic_set_active(IngenicMic *m, bool on);

/* The next stereo frame at this rate; silence when nothing is heard */
void ingenic_mic_frame(IngenicMic *m, uint32_t rate, int16_t f[2]);

void ingenic_mic_cleanup(IngenicMic *m);

#endif
