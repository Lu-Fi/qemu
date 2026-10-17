/*
 * The microphone behind an Ingenic audio input: a sine, a looped WAV file
 * or an audiodev's input
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/audio/ingenic-mic.h"

#define TONE_DBFS       -20     /* the vendor libimp adds 10 dB by default */

static bool mic_load_wav(IngenicMic *m, const char *path, Error **errp)
{
    g_autofree uint8_t *buf = NULL;
    gsize len, off = 12;
    unsigned ch = 0, bits = 0;
    const uint8_t *data = NULL;
    uint32_t data_len = 0;

    if (!g_file_get_contents(path, (gchar **)&buf, &len, NULL) || len < 12 ||
        memcmp(buf, "RIFF", 4) || memcmp(buf + 8, "WAVE", 4)) {
        error_setg(errp, "%s: mic: %s is not a WAV file", m->owner, path);
        return false;
    }
    while (off + 8 <= len) {
        uint32_t clen = ldl_le_p(buf + off + 4);

        if (!memcmp(buf + off, "fmt ", 4) && clen >= 16 && off + 24 <= len) {
            if (lduw_le_p(buf + off + 8) != 1) {
                break;
            }
            ch = lduw_le_p(buf + off + 10);
            m->wav_rate = ldl_le_p(buf + off + 12);
            bits = lduw_le_p(buf + off + 22);
        } else if (!memcmp(buf + off, "data", 4)) {
            data = buf + off + 8;
            data_len = MIN(clen, len - off - 8);
        }
        off += 8 + clen + (clen & 1);
    }
    if (bits != 16 || (ch != 1 && ch != 2) || !m->wav_rate || !data) {
        error_setg(errp, "%s: mic: %s is not 16-bit mono or stereo PCM",
                   m->owner, path);
        return false;
    }
    m->wav_frames = data_len / (2 * ch);
    if (!m->wav_frames) {
        error_setg(errp, "%s: mic: %s has no samples", m->owner, path);
        return false;
    }
    m->wav = g_malloc_n(m->wav_frames, sizeof(*m->wav));
    for (uint32_t i = 0; i < m->wav_frames; i++) {
        m->wav[i][0] = lduw_le_p(data + 2 * ch * i);
        m->wav[i][1] = lduw_le_p(data + 2 * ch * i + 2 * (ch - 1));
    }
    return true;
}

bool ingenic_mic_init(IngenicMic *m, const char *owner, const char *spec,
                      Error **errp)
{
    m->owner = owner;
    m->voice_name = g_strdup_printf("%s.in", owner);
    if (spec && (!strcmp(spec, "tone") || g_str_has_prefix(spec, "tone:"))) {
        g_auto(GStrv) arg = g_strsplit(spec, ":", 3);
        double dbfs = arg[1] && arg[2] ? g_ascii_strtod(arg[2], NULL)
                                       : TONE_DBFS;

        m->tone_hz = arg[1] ? g_ascii_strtod(arg[1], NULL) : 1000;
        if (!(m->tone_hz > 0) || !(dbfs <= 0)) {
            error_setg(errp, "%s: mic: want tone[:<Hz>[:<dBFS>]]", owner);
            return false;
        }
        m->tone_level = 32767 * pow(10, dbfs / 20);
        return true;
    }
    return !spec || !*spec || mic_load_wav(m, spec, errp);
}

static void mic_in_cb(void *opaque, int avail)
{
    IngenicMic *m = opaque;
    uint32_t frames = (uint32_t)avail / 4;

    while (frames) {
        uint32_t wr = (m->rd + m->n) % INGENIC_MIC_RING;
        uint32_t chunk = MIN(frames, INGENIC_MIC_RING - wr);
        size_t done;

        if (m->n + chunk > INGENIC_MIC_RING) {
            /* The guest is not recording: keep the newest */
            uint32_t drop = m->n + chunk - INGENIC_MIC_RING;

            m->rd = (m->rd + drop) % INGENIC_MIC_RING;
            m->n -= drop;
        }
        done = audio_be_read(m->audio_be, m->voice, m->ring[wr], chunk * 4) / 4;
        m->n += done;
        frames -= done;
        if (done < chunk) {
            break;
        }
    }
}

void ingenic_mic_open(IngenicMic *m, AudioBackend *be, uint32_t rate)
{
    struct audsettings as = {
        .freq = rate,
        .nchannels = 2,
        .fmt = AUDIO_FORMAT_S16,
        .big_endian = HOST_BIG_ENDIAN,
    };

    if (!be || m->tone_hz > 0 || m->wav || m->voice_rate == rate) {
        return;
    }
    m->audio_be = be;
    m->voice_rate = rate;
    m->rd = m->n = 0;
    m->voice = audio_be_open_in(be, m->voice, m->voice_name, m, mic_in_cb,
                                &as);
}

void ingenic_mic_set_active(IngenicMic *m, bool on)
{
    if (m->voice) {
        audio_be_set_active_in(m->audio_be, m->voice, on);
    }
}

void ingenic_mic_frame(IngenicMic *m, uint32_t rate, int16_t f[2])
{
    if (m->tone_hz > 0) {
        f[0] = f[1] = lrint(m->tone_level * sin(m->phase));
        m->phase += 2 * M_PI * m->tone_hz / rate;
        if (m->phase >= 2 * M_PI) {
            m->phase -= 2 * M_PI;
        }
    } else if (m->wav) {
        uint32_t i = m->wav_pos;
        uint32_t j = (i + 1) % m->wav_frames;
        double a = m->wav_pos - i;

        for (int c = 0; c < 2; c++) {
            f[c] = lrint(m->wav[i][c] * (1 - a) + m->wav[j][c] * a);
        }
        m->wav_pos += (double)m->wav_rate / rate;
        while (m->wav_pos >= m->wav_frames) {
            m->wav_pos -= m->wav_frames;
        }
    } else if (m->n) {
        f[0] = m->ring[m->rd][0];
        f[1] = m->ring[m->rd][1];
        m->rd = (m->rd + 1) % INGENIC_MIC_RING;
        m->n--;
    } else {
        f[0] = f[1] = 0;
    }
}

void ingenic_mic_cleanup(IngenicMic *m)
{
    g_free(m->wav);
    m->wav = NULL;
    g_free(m->voice_name);
    m->voice_name = NULL;
}
