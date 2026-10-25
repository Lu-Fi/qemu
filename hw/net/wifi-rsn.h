/*
 * WPA2-PSK as both ends of the 4-way handshake work it: the PMK from the
 * passphrase, the PTK from the PMK and both nonces, the EAPOL-Key MIC,
 * and the key wrap that carries the group key
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_WIFI_RSN_H
#define HW_NET_WIFI_RSN_H

#define WIFI_ETH_P_EAPOL        0x888e

/* An EAPOL-Key frame's fields, from its EAPOL header, as 802.11 has them */
#define WIFI_EAPOL_KEY          3
#define WIFI_KEY_DESC_RSN       2
#define WIFI_KEY_DESC           4
#define WIFI_KEY_INFO           5
#define WIFI_KEY_LEN            7
#define WIFI_KEY_REPLAY         9
#define WIFI_KEY_NONCE          17
#define WIFI_KEY_MIC            81
#define WIFI_KEY_DATA_LEN       97
#define WIFI_KEY_DATA           99

#define WIFI_KI_HMAC_SHA1_AES   2
#define WIFI_KI_PAIRWISE        BIT(3)
#define WIFI_KI_INSTALL         BIT(6)
#define WIFI_KI_ACK             BIT(7)
#define WIFI_KI_MIC             BIT(8)
#define WIFI_KI_SECURE          BIT(9)
#define WIFI_KI_ENC_DATA        BIT(12)

#define WIFI_PMK_LEN            32
#define WIFI_PTK_LEN            48      /* KCK, KEK, TK */
#define WIFI_PTK_KEK            16
#define WIFI_PTK_TK             32
#define WIFI_NONCE_LEN          32
#define WIFI_GTK_LEN            16

/* WPA2: CCMP for both groups of traffic, PSK key management */
extern const uint8_t wifi_rsn_ie[22];

/* PBKDF2-HMAC-SHA1 of @psk over the SSID, 4096 rounds */
void wifi_rsn_pmk(const char *psk, const uint8_t *ssid, size_t ssid_len,
                  uint8_t *pmk);

/* PRF-384 over both addresses and both nonces, each pair in order */
void wifi_rsn_ptk(const uint8_t *pmk, const uint8_t *aa, const uint8_t *spa,
                  const uint8_t *anonce, const uint8_t *snonce,
                  uint8_t *ptk);

/*
 * The EAPOL-Key MIC, keyed with the KCK, over a frame whose MIC is zero:
 * @len takes in the fixed fields, WIFI_KEY_DATA bytes at least
 */
void wifi_rsn_mic(const uint8_t *ptk, const uint8_t *eapol, size_t len,
                  uint8_t *mic);

/* RFC 3394 key wrap with the KEK; @out takes len + 8 bytes */
void wifi_rsn_wrap(const uint8_t *ptk, const uint8_t *in, size_t len,
                   uint8_t *out);

/* And its unwrap, @len the wrapped length: false if the check fails */
bool wifi_rsn_unwrap(const uint8_t *ptk, const uint8_t *in, size_t len,
                     uint8_t *out);

#endif
