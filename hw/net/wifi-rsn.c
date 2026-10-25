/*
 * WPA2-PSK keys and the 4-way handshake's integrity and wrapping, for the
 * access point and the client both
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "crypto/aes.h"
#include "crypto/hmac.h"
#include "wifi-rsn.h"

#define SHA1_LEN                20

const uint8_t wifi_rsn_ie[22] = {
    0x30, 20, 1, 0,
    0x00, 0x0f, 0xac, 4,
    1, 0, 0x00, 0x0f, 0xac, 4,
    1, 0, 0x00, 0x0f, 0xac, 2,
    0, 0,
};

static void hmac_sha1(const uint8_t *key, size_t nkey,
                      const struct iovec *iov, size_t niov, uint8_t *out)
{
    g_autoptr(QCryptoHmac) h = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA1,
                                                key, nkey, &error_abort);
    size_t len = SHA1_LEN;

    qcrypto_hmac_bytesv(h, iov, niov, &out, &len, &error_abort);
}

void wifi_rsn_pmk(const char *psk, const uint8_t *ssid, size_t ssid_len,
                  uint8_t *pmk)
{
    size_t np = strlen(psk);
    uint8_t cnt[4], u[SHA1_LEN], t[SHA1_LEN];

    for (int blk = 0; blk < 2; blk++) {
        struct iovec salt[2] = {
            { (void *)ssid, ssid_len }, { cnt, sizeof(cnt) },
        };

        stl_be_p(cnt, blk + 1);
        hmac_sha1((const uint8_t *)psk, np, salt, 2, u);
        memcpy(t, u, sizeof(t));
        for (int i = 1; i < 4096; i++) {
            struct iovec iov = { u, sizeof(u) };
            uint8_t next[SHA1_LEN];

            hmac_sha1((const uint8_t *)psk, np, &iov, 1, next);
            memcpy(u, next, sizeof(u));
            for (int k = 0; k < SHA1_LEN; k++) {
                t[k] ^= u[k];
            }
        }
        memcpy(pmk + blk * SHA1_LEN, t, blk ? 12 : SHA1_LEN);
    }
}

void wifi_rsn_ptk(const uint8_t *pmk, const uint8_t *aa, const uint8_t *spa,
                  const uint8_t *anonce, const uint8_t *snonce, uint8_t *ptk)
{
    static const char label[] = "Pairwise key expansion";
    uint8_t data[6 + 6 + 2 * WIFI_NONCE_LEN], out[3 * SHA1_LEN];
    bool sta_first = memcmp(spa, aa, 6) < 0;
    bool snonce_first = memcmp(snonce, anonce, WIFI_NONCE_LEN) < 0;

    memcpy(data, sta_first ? spa : aa, 6);
    memcpy(data + 6, sta_first ? aa : spa, 6);
    memcpy(data + 12, snonce_first ? snonce : anonce, WIFI_NONCE_LEN);
    memcpy(data + 44, snonce_first ? anonce : snonce, WIFI_NONCE_LEN);
    for (uint8_t i = 0; i < 3; i++) {
        /* the label goes in with its NUL, the separator the PRF wants */
        struct iovec iov[3] = {
            { (void *)label, sizeof(label) }, { data, sizeof(data) },
            { &i, 1 },
        };

        hmac_sha1(pmk, WIFI_PMK_LEN, iov, 3, out + i * SHA1_LEN);
    }
    memcpy(ptk, out, WIFI_PTK_LEN);
}

void wifi_rsn_mic(const uint8_t *ptk, const uint8_t *eapol, size_t len,
                  uint8_t *mic)
{
    g_autofree uint8_t *copy = g_memdup2(eapol, len);
    struct iovec iov = { copy, len };
    uint8_t out[SHA1_LEN];

    memset(copy + WIFI_KEY_MIC, 0, 16);
    hmac_sha1(ptk, 16, &iov, 1, out);
    memcpy(mic, out, 16);
}

void wifi_rsn_wrap(const uint8_t *ptk, const uint8_t *in, size_t len,
                   uint8_t *out)
{
    size_t n = len / 8;
    AES_KEY key;
    uint8_t b[16];

    AES_set_encrypt_key(ptk + WIFI_PTK_KEK, 128, &key);
    memset(out, 0xa6, 8);
    memcpy(out + 8, in, len);
    for (unsigned j = 0; j < 6; j++) {
        for (size_t i = 1; i <= n; i++) {
            memcpy(b, out, 8);
            memcpy(b + 8, out + 8 * i, 8);
            AES_encrypt(b, b, &key);
            stq_be_p(out, ldq_be_p(b) ^ (n * j + i));
            memcpy(out + 8 * i, b + 8, 8);
        }
    }
}

bool wifi_rsn_unwrap(const uint8_t *ptk, const uint8_t *in, size_t len,
                     uint8_t *out)
{
    size_t n;
    uint64_t a;
    AES_KEY key;
    uint8_t b[16];

    if (len < 16 || len % 8) {
        return false;
    }
    n = len / 8 - 1;
    a = ldq_be_p(in);
    AES_set_decrypt_key(ptk + WIFI_PTK_KEK, 128, &key);
    memcpy(out, in + 8, len - 8);
    for (int j = 5; j >= 0; j--) {
        for (size_t i = n; i >= 1; i--) {
            stq_be_p(b, a ^ (n * j + i));
            memcpy(b + 8, out + 8 * (i - 1), 8);
            AES_decrypt(b, b, &key);
            a = ldq_be_p(b);
            memcpy(out + 8 * (i - 1), b + 8, 8);
        }
    }
    return a == 0xa6a6a6a6a6a6a6a6ULL;
}
