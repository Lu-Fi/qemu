/*
 * CCMP as 802.11 frames carry it: AES-CCM with an 8-byte MIC and a 2-byte
 * length field, over the nonce and AAD that 802.11 builds from the header
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "crypto/aes.h"
#include "wifi-ccmp.h"
#include "wifi-frame.h"

#define FC_RETRY                BIT(11)
#define FC_PWRMGT               BIT(12)
#define FC_MOREDATA             BIT(13)
#define FC_ORDER                BIT(15)

/* B0: AAD present, M = 8 ((8 - 2) / 2 << 3), L = 2 (2 - 1) */
#define CCM_B0_FLAGS            0x59
/* A_i: L = 2 */
#define CCM_A_FLAGS             0x01

size_t wifi_hdr_len(uint16_t fc)
{
    size_t n = WIFI_HDR_LEN;

    if (WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA) {
        if ((fc & WIFI_FC_TODS) && (fc & WIFI_FC_FROMDS)) {
            n += 6;
        }
        if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
            n += 2;
        }
    }
    return n;
}

/* The nonce and AAD 802.11 builds from a protected frame's header */
static size_t wifi_ccmp_aad(const uint8_t *f, size_t hdr, uint8_t *nonce,
                            uint8_t *aad)
{
    uint16_t fc = lduw_le_p(f);
    bool data = WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA;
    bool qos = data && (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS);
    bool a4 = data && (fc & WIFI_FC_TODS) && (fc & WIFI_FC_FROMDS);
    const uint8_t *iv = f + hdr;
    uint8_t *p = aad;

    nonce[0] = qos ? f[hdr - 2] & 0x0f : 0;
    memcpy(nonce + 1, f + 10, 6);
    nonce[7] = iv[7];
    nonce[8] = iv[6];
    nonce[9] = iv[5];
    nonce[10] = iv[4];
    nonce[11] = iv[1];
    nonce[12] = iv[0];

    fc &= ~(FC_RETRY | FC_PWRMGT | FC_MOREDATA);
    if (data) {
        fc &= ~0x0070;
    }
    if (qos) {
        fc &= ~FC_ORDER;
    }
    stw_le_p(p, fc | WIFI_FC_PROTECTED);
    p += 2;
    memcpy(p, f + 4, 18);
    p += 18;
    /* the fragment number stays, the sequence number goes */
    stw_le_p(p, lduw_le_p(f + 22) & 0x000f);
    p += 2;
    if (a4) {
        memcpy(p, f + 24, 6);
        p += 6;
    }
    if (qos) {
        *p++ = f[hdr - 2] & 0x0f;
        *p++ = 0;
    }
    return p - aad;
}

/* One CBC-MAC step over up to a block, zero padded */
static void wifi_ccm_mac(const AES_KEY *key, uint8_t *x, const uint8_t *in,
                         size_t len)
{
    for (size_t i = 0; i < len; i++) {
        x[i] ^= in[i];
    }
    AES_encrypt(x, x, key);
}

/*
 * The payload's keystream XORed in, from counter A_1, and the MIC: the
 * CBC-MAC over B_0, the AAD and the plaintext, under A_0's keystream
 */
static void wifi_ccmp_crypt(const AES_KEY *key, const uint8_t *nonce,
                            const uint8_t *aad, size_t aad_len,
                            const uint8_t *in, uint8_t *out, size_t n,
                            bool decrypting, uint8_t *mic)
{
    uint8_t b[16], x[16], a[16], s[16];
    const uint8_t *plain = decrypting ? out : in;

    a[0] = CCM_A_FLAGS;
    memcpy(a + 1, nonce, 13);
    for (size_t i = 0; i < n; i += 16) {
        stw_be_p(a + 14, i / 16 + 1);
        AES_encrypt(a, s, key);
        for (size_t k = 0; k < 16 && i + k < n; k++) {
            out[i + k] = in[i + k] ^ s[k];
        }
    }

    b[0] = CCM_B0_FLAGS;
    memcpy(b + 1, nonce, 13);
    stw_be_p(b + 14, n);
    memset(x, 0, sizeof(x));
    wifi_ccm_mac(key, x, b, 16);
    memset(b, 0, sizeof(b));
    stw_be_p(b, aad_len);
    memcpy(b + 2, aad, MIN(aad_len, 14));
    wifi_ccm_mac(key, x, b, 16);
    if (aad_len > 14) {
        wifi_ccm_mac(key, x, aad + 14, aad_len - 14);
    }
    for (size_t i = 0; i < n; i += 16) {
        wifi_ccm_mac(key, x, plain + i, MIN(n - i, 16));
    }
    stw_be_p(a + 14, 0);
    AES_encrypt(a, s, key);
    for (size_t k = 0; k < WIFI_CCMP_MIC_LEN; k++) {
        mic[k] = x[k] ^ s[k];
    }
}

size_t wifi_ccmp_encrypt(const uint8_t *tk, uint64_t pn, unsigned key_id,
                         const uint8_t *f, size_t len, uint8_t *out)
{
    uint16_t fc = lduw_le_p(f);
    size_t hdr = MIN(wifi_hdr_len(fc), len), n = len - hdr, aad_len;
    uint8_t nonce[13], aad[32];
    uint8_t *iv = out + hdr;
    AES_KEY key;

    memcpy(out, f, hdr);
    stw_le_p(out, fc | WIFI_FC_PROTECTED);
    iv[0] = pn;
    iv[1] = pn >> 8;
    iv[2] = 0;
    /* ExtIV, then the key id */
    iv[3] = 0x20 | (key_id & 3) << 6;
    stl_le_p(iv + 4, pn >> 16);
    aad_len = wifi_ccmp_aad(out, hdr, nonce, aad);
    AES_set_encrypt_key(tk, 128, &key);
    wifi_ccmp_crypt(&key, nonce, aad, aad_len, f + hdr,
                    out + hdr + WIFI_CCMP_HDR_LEN, n, false,
                    out + hdr + WIFI_CCMP_HDR_LEN + n);
    return hdr + WIFI_CCMP_HDR_LEN + n + WIFI_CCMP_MIC_LEN;
}

bool wifi_ccmp_decrypt(const uint8_t *tk, uint8_t *f, size_t *len)
{
    uint16_t fc;
    size_t hdr, n, aad_len;
    uint8_t nonce[13], aad[32], mic[WIFI_CCMP_MIC_LEN];
    g_autofree uint8_t *plain = NULL;
    const uint8_t *c;
    AES_KEY key;

    if (*len < WIFI_HDR_LEN) {
        return false;
    }
    fc = lduw_le_p(f);
    hdr = wifi_hdr_len(fc);
    if (!(fc & WIFI_FC_PROTECTED) ||
        *len < hdr + WIFI_CCMP_HDR_LEN + WIFI_CCMP_MIC_LEN ||
        !(f[hdr + 3] & 0x20)) {
        return false;
    }
    n = *len - hdr - WIFI_CCMP_HDR_LEN - WIFI_CCMP_MIC_LEN;
    c = f + hdr + WIFI_CCMP_HDR_LEN;
    aad_len = wifi_ccmp_aad(f, hdr, nonce, aad);
    AES_set_encrypt_key(tk, 128, &key);
    plain = g_malloc(n);
    wifi_ccmp_crypt(&key, nonce, aad, aad_len, c, plain, n, true, mic);
    if (memcmp(mic, c + n, sizeof(mic))) {
        return false;
    }

    stw_le_p(f, fc & ~WIFI_FC_PROTECTED);
    memcpy(f + hdr, plain, n);
    *len = hdr + n;
    return true;
}
