/*
 * A Wi-Fi access point for a station model to join
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/ctype.h"
#include "qemu/guest-random.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "crypto/aes.h"
#include "crypto/hmac.h"
#include "hw/net/wifi-ap.h"
#include "net/eth.h"
#include "trace.h"

#define FC_TYPE(fc)         (((fc) >> 2) & 3)
#define FC_STYPE(fc)        (((fc) >> 4) & 0xf)
#define FC_TODS             BIT(8)
#define FC_FROMDS           BIT(9)

#define TYPE_MGMT           0
#define TYPE_DATA           2

#define STYPE_ASSOC_REQ     0
#define STYPE_REASSOC_REQ   2
#define STYPE_PROBE_REQ     4
#define STYPE_PROBE_RESP    5
#define STYPE_BEACON        8
#define STYPE_DISASSOC      10
#define STYPE_AUTH          11
#define STYPE_DEAUTH        12

#define FC_MGMT(stype)      ((stype) << 4)
#define FC_DATA             0x0008

#define HDR_LEN             24
#define ETH_P_EAPOL         0x888e

#define CAP_ESS             BIT(0)
#define CAP_PRIVACY         BIT(4)
#define CAP_SHORT_PREAMBLE  BIT(5)
#define CAP_SHORT_SLOT      BIT(10)

#define REASON_4WAY_TIMEOUT 15
#define STATUS_FAILURE      1
#define STATUS_BAD_ALG      13

#define EAPOL_KEY           3
#define DESC_RSN            2
/* EAPOL-Key fields, from the start of the EAPOL header */
#define KEY_DESC            4
#define KEY_INFO            5
#define KEY_LEN             7
#define KEY_REPLAY          9
#define KEY_NONCE           17
#define KEY_MIC             81
#define KEY_DATA_LEN        97
#define KEY_DATA            99

#define KI_HMAC_SHA1_AES    2
#define KI_PAIRWISE         BIT(3)
#define KI_INSTALL          BIT(6)
#define KI_ACK              BIT(7)
#define KI_MIC              BIT(8)
#define KI_SECURE           BIT(9)
#define KI_ENC_DATA         BIT(12)

#define SHA1_LEN            20
#define EAPOL_RETRY_MS      1000
#define EAPOL_TRIES         4

static const uint8_t rfc1042[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };

/* WPA2: CCMP for both groups of traffic, PSK key management */
static const uint8_t rsn_ie[] = {
    0x30, 20, 1, 0,
    0x00, 0x0f, 0xac, 4,
    1, 0, 0x00, 0x0f, 0xac, 4,
    1, 0, 0x00, 0x0f, 0xac, 2,
    0, 0,
};

static const MACAddr default_bssid = { .a = { 0x52, 0x54, 0x00, 0x77,
                                              0x69, 0x01 } };

static void hmac_sha1(const uint8_t *key, size_t nkey,
                      const struct iovec *iov, size_t niov, uint8_t *out)
{
    g_autoptr(QCryptoHmac) h = qcrypto_hmac_new(QCRYPTO_HASH_ALGO_SHA1,
                                                key, nkey, &error_abort);
    size_t len = SHA1_LEN;

    qcrypto_hmac_bytesv(h, iov, niov, &out, &len, &error_abort);
}

/* PBKDF2-HMAC-SHA1 of the passphrase over the SSID, 4096 rounds */
static void wifi_ap_pmk(WifiAP *ap)
{
    size_t np = strlen(ap->psk);
    uint8_t cnt[4], u[SHA1_LEN], t[SHA1_LEN];

    for (int blk = 0; blk < 2; blk++) {
        struct iovec salt[2] = {
            { ap->ssid, strlen(ap->ssid) }, { cnt, sizeof(cnt) },
        };

        stl_be_p(cnt, blk + 1);
        hmac_sha1((uint8_t *)ap->psk, np, salt, 2, u);
        memcpy(t, u, sizeof(t));
        for (int i = 1; i < 4096; i++) {
            struct iovec iov = { u, sizeof(u) };
            uint8_t next[SHA1_LEN];

            hmac_sha1((uint8_t *)ap->psk, np, &iov, 1, next);
            memcpy(u, next, sizeof(u));
            for (int k = 0; k < SHA1_LEN; k++) {
                t[k] ^= u[k];
            }
        }
        memcpy(ap->pmk + blk * SHA1_LEN, t, blk ? 12 : SHA1_LEN);
    }
}

/* PRF-384 over both addresses and both nonces, each pair in order */
static void wifi_ap_ptk(WifiAP *ap, const uint8_t *snonce)
{
    static const char label[] = "Pairwise key expansion";
    const uint8_t *aa = ap->bssid.a, *spa = ap->sta.a;
    uint8_t data[6 + 6 + 32 + 32], out[3 * SHA1_LEN];
    bool sta_first = memcmp(spa, aa, 6) < 0;
    bool snonce_first = memcmp(snonce, ap->anonce, 32) < 0;

    memcpy(data, sta_first ? spa : aa, 6);
    memcpy(data + 6, sta_first ? aa : spa, 6);
    memcpy(data + 12, snonce_first ? snonce : ap->anonce, 32);
    memcpy(data + 44, snonce_first ? ap->anonce : snonce, 32);
    for (uint8_t i = 0; i < 3; i++) {
        /* the label goes in with its NUL, the separator the PRF wants */
        struct iovec iov[3] = {
            { (void *)label, sizeof(label) }, { data, sizeof(data) },
            { &i, 1 },
        };

        hmac_sha1(ap->pmk, sizeof(ap->pmk), iov, 3, out + i * SHA1_LEN);
    }
    memcpy(ap->ptk, out, sizeof(ap->ptk));
}

/* The EAPOL-Key MIC, keyed with the KCK, over a frame whose MIC is zero */
static void wifi_ap_mic(WifiAP *ap, const uint8_t *eapol, size_t len,
                        uint8_t *mic)
{
    g_autofree uint8_t *copy = g_memdup2(eapol, len);
    struct iovec iov = { copy, len };
    uint8_t out[SHA1_LEN];

    memset(copy + KEY_MIC, 0, 16);
    hmac_sha1(ap->ptk, 16, &iov, 1, out);
    memcpy(mic, out, 16);
}

/* RFC 3394 key wrap with the KEK; @out takes len + 8 bytes */
static void wifi_ap_wrap(WifiAP *ap, const uint8_t *in, size_t len,
                         uint8_t *out)
{
    size_t n = len / 8;
    AES_KEY key;
    uint8_t b[16];

    AES_set_encrypt_key(ap->ptk + 16, 128, &key);
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

static size_t wifi_ap_hdr(WifiAP *ap, uint8_t *f, uint16_t fc,
                          const uint8_t *da, const uint8_t *sa)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, da, 6);
    memcpy(f + 10, ap->bssid.a, 6);
    memcpy(f + 16, sa, 6);
    stw_le_p(f + 22, (ap->seq++ & 0xfff) << 4);
    return HDR_LEN;
}

static void wifi_ap_send(WifiAP *ap, const uint8_t *f, size_t len,
                         bool protected)
{
    trace_wifi_ap_send(lduw_le_p(f), len, protected);
    ap->deliver(ap->opaque, f, len, protected);
}

/* SSID, 802.11b/g rates, channel, then what WPA2 needs */
static size_t wifi_ap_ies(WifiAP *ap, uint8_t *p, bool beacon)
{
    static const uint8_t rates[] = {
        1, 8, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
    };
    static const uint8_t xrates[] = { 50, 4, 0x30, 0x48, 0x60, 0x6c };
    static const uint8_t tim[] = { 5, 4, 0, 1, 0, 0 };
    static const uint8_t erp[] = { 42, 1, 0 };
    size_t n = strlen(ap->ssid);
    uint8_t *start = p;

    *p++ = 0;
    *p++ = n;
    memcpy(p, ap->ssid, n);
    p += n;
    memcpy(p, rates, sizeof(rates));
    p += sizeof(rates);
    *p++ = 3;
    *p++ = 1;
    *p++ = ap->channel;
    if (beacon) {
        memcpy(p, tim, sizeof(tim));
        p += sizeof(tim);
    }
    memcpy(p, erp, sizeof(erp));
    p += sizeof(erp);
    memcpy(p, xrates, sizeof(xrates));
    p += sizeof(xrates);
    if (ap->psk) {
        memcpy(p, rsn_ie, sizeof(rsn_ie));
        p += sizeof(rsn_ie);
    }
    return p - start;
}

static uint16_t wifi_ap_capability(WifiAP *ap)
{
    return CAP_ESS | CAP_SHORT_PREAMBLE | CAP_SHORT_SLOT |
           (ap->psk ? CAP_PRIVACY : 0);
}

static size_t wifi_ap_announce(WifiAP *ap, uint8_t *f, unsigned stype,
                               const uint8_t *da)
{
    size_t len = wifi_ap_hdr(ap, f, FC_MGMT(stype), da, ap->bssid.a);

    stq_le_p(f + len, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(f + len + 8, 100);
    stw_le_p(f + len + 10, wifi_ap_capability(ap));
    len += 12;
    return len + wifi_ap_ies(ap, f + len, stype == STYPE_BEACON);
}

size_t wifi_ap_beacon(WifiAP *ap, uint8_t *buf)
{
    static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

    return wifi_ap_announce(ap, buf, STYPE_BEACON, bcast);
}

static void wifi_ap_set_state(WifiAP *ap, WifiAPState state)
{
    trace_wifi_ap_state(ap->state, state);
    ap->state = state;
    if (state < WIFI_AP_ASSOCIATED || state == WIFI_AP_CONNECTED) {
        timer_del(ap->eapol_timer);
    }
}

static void wifi_ap_deauth(WifiAP *ap, uint16_t reason)
{
    uint8_t f[HDR_LEN + 2];

    wifi_ap_hdr(ap, f, FC_MGMT(STYPE_DEAUTH), ap->sta.a, ap->bssid.a);
    stw_le_p(f + HDR_LEN, reason);
    wifi_ap_send(ap, f, sizeof(f), false);
    wifi_ap_set_state(ap, WIFI_AP_IDLE);
}

/* An LLC/SNAP data frame from the distribution system to the station */
static void wifi_ap_data(WifiAP *ap, const uint8_t *da, const uint8_t *sa,
                         uint16_t proto, const uint8_t *payload, size_t len,
                         bool protected)
{
    g_autofree uint8_t *f = g_malloc(HDR_LEN + 8 + len);

    wifi_ap_hdr(ap, f, FC_DATA | FC_FROMDS, da, sa);
    memcpy(f + HDR_LEN, rfc1042, sizeof(rfc1042));
    stw_be_p(f + HDR_LEN + 6, proto);
    memcpy(f + HDR_LEN + 8, payload, len);
    wifi_ap_send(ap, f, HDR_LEN + 8 + len, protected);
}

/* Message 1 or 3 of the 4-way handshake, the latter carrying the GTK */
static void wifi_ap_eapol_send(WifiAP *ap)
{
    bool msg3 = ap->state == WIFI_AP_PTK_NEGOTIATING;
    uint8_t f[KEY_DATA + 64] = {};
    uint16_t info = KI_HMAC_SHA1_AES | KI_PAIRWISE | KI_ACK;
    size_t data_len = 0;

    ap->replay++;
    if (msg3) {
        uint8_t kd[48] = {}, *p = kd;

        info |= KI_INSTALL | KI_MIC | KI_SECURE | KI_ENC_DATA;
        memcpy(p, rsn_ie, sizeof(rsn_ie));
        p += sizeof(rsn_ie);
        /* the GTK KDE: key id 1, then padding to the wrap's 8 bytes */
        *p++ = 0xdd;
        *p++ = 22;
        *p++ = 0x00;
        *p++ = 0x0f;
        *p++ = 0xac;
        *p++ = 1;
        *p++ = 1;
        *p++ = 0;
        memcpy(p, ap->gtk, sizeof(ap->gtk));
        p += sizeof(ap->gtk);
        *p = 0xdd;
        data_len = sizeof(kd) + 8;
        wifi_ap_wrap(ap, kd, sizeof(kd), f + KEY_DATA);
    }
    f[0] = 2;
    f[1] = EAPOL_KEY;
    stw_be_p(f + 2, KEY_DATA - 4 + data_len);
    f[KEY_DESC] = DESC_RSN;
    stw_be_p(f + KEY_INFO, info);
    stw_be_p(f + KEY_LEN, 16);
    stq_be_p(f + KEY_REPLAY, ap->replay);
    memcpy(f + KEY_NONCE, ap->anonce, sizeof(ap->anonce));
    stw_be_p(f + KEY_DATA_LEN, data_len);
    if (msg3) {
        wifi_ap_mic(ap, f, KEY_DATA + data_len, f + KEY_MIC);
    }
    trace_wifi_ap_eapol_send(msg3 ? 3 : 1, ap->replay);
    wifi_ap_data(ap, ap->sta.a, ap->bssid.a, ETH_P_EAPOL, f,
                 KEY_DATA + data_len, false);
    ap->tries++;
    timer_mod(ap->eapol_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + EAPOL_RETRY_MS);
}

static void wifi_ap_eapol_timer(void *opaque)
{
    WifiAP *ap = opaque;

    switch (ap->state) {
    case WIFI_AP_ASSOCIATED:
        qemu_guest_getrandom_nofail(ap->anonce, sizeof(ap->anonce));
        ap->tries = 0;
        wifi_ap_set_state(ap, WIFI_AP_PTK_START);
        wifi_ap_eapol_send(ap);
        break;
    case WIFI_AP_PTK_START:
    case WIFI_AP_PTK_NEGOTIATING:
        if (ap->tries >= EAPOL_TRIES) {
            wifi_ap_deauth(ap, REASON_4WAY_TIMEOUT);
            break;
        }
        wifi_ap_eapol_send(ap);
        break;
    default:
        break;
    }
}

/* Message 2 or 4 of the 4-way handshake */
static void wifi_ap_eapol(WifiAP *ap, const uint8_t *p, size_t len)
{
    uint16_t info;
    uint8_t mic[16];
    size_t elen;

    if (len < KEY_DATA || p[1] != EAPOL_KEY || p[KEY_DESC] != DESC_RSN) {
        return;
    }
    elen = MIN(len, 4 + lduw_be_p(p + 2));
    info = lduw_be_p(p + KEY_INFO);
    if (!(info & KI_MIC) || ldq_be_p(p + KEY_REPLAY) != ap->replay) {
        return;
    }
    if (ap->state == WIFI_AP_PTK_START && !(info & KI_SECURE)) {
        wifi_ap_ptk(ap, p + KEY_NONCE);
        wifi_ap_mic(ap, p, elen, mic);
        trace_wifi_ap_eapol_recv(2, !memcmp(mic, p + KEY_MIC, 16));
        /* a wrong passphrase: leave it to the retries to give up */
        if (memcmp(mic, p + KEY_MIC, 16)) {
            return;
        }
        ap->tries = 0;
        wifi_ap_set_state(ap, WIFI_AP_PTK_NEGOTIATING);
        wifi_ap_eapol_send(ap);
    } else if (ap->state == WIFI_AP_PTK_NEGOTIATING && (info & KI_SECURE)) {
        wifi_ap_mic(ap, p, elen, mic);
        trace_wifi_ap_eapol_recv(4, !memcmp(mic, p + KEY_MIC, 16));
        if (!memcmp(mic, p + KEY_MIC, 16)) {
            wifi_ap_set_state(ap, WIFI_AP_CONNECTED);
        }
    }
}

static const uint8_t *wifi_ap_find_ie(const uint8_t *p, size_t len,
                                      uint8_t id)
{
    while (len >= 2 && len >= 2u + p[1]) {
        if (p[0] == id) {
            return p;
        }
        len -= 2 + p[1];
        p += 2 + p[1];
    }
    return NULL;
}

static bool wifi_ap_ssid_is(WifiAP *ap, const uint8_t *ie, bool wildcard)
{
    size_t n = strlen(ap->ssid);

    if (!ie) {
        return false;
    }
    if (!ie[1]) {
        return wildcard;
    }
    return ie[1] == n && !memcmp(ie + 2, ap->ssid, n);
}

static void wifi_ap_mgmt(WifiAP *ap, uint16_t fc, const uint8_t *f,
                         size_t len)
{
    const uint8_t *sa = f + 10, *body = f + HDR_LEN;
    size_t blen = len - HDR_LEN;
    uint8_t r[WIFI_AP_BEACON_MAX];
    size_t n, ies;
    uint16_t status;

    switch (FC_STYPE(fc)) {
    case STYPE_PROBE_REQ:
        if (wifi_ap_ssid_is(ap, wifi_ap_find_ie(body, blen, 0), true)) {
            n = wifi_ap_announce(ap, r, STYPE_PROBE_RESP, sa);
            wifi_ap_send(ap, r, n, false);
        }
        return;
    case STYPE_AUTH:
        if (blen < 6 || lduw_le_p(body + 2) != 1) {
            return;
        }
        status = lduw_le_p(body) ? STATUS_BAD_ALG : 0;
        n = wifi_ap_hdr(ap, r, FC_MGMT(STYPE_AUTH), sa, ap->bssid.a);
        stw_le_p(r + n, lduw_le_p(body));
        stw_le_p(r + n + 2, 2);
        stw_le_p(r + n + 4, status);
        memcpy(ap->sta.a, sa, 6);
        wifi_ap_set_state(ap, status ? WIFI_AP_IDLE : WIFI_AP_AUTHENTICATED);
        wifi_ap_send(ap, r, n + 6, false);
        return;
    case STYPE_ASSOC_REQ:
    case STYPE_REASSOC_REQ:
        if (memcmp(sa, ap->sta.a, 6) || ap->state < WIFI_AP_AUTHENTICATED) {
            return;
        }
        ies = FC_STYPE(fc) == STYPE_REASSOC_REQ ? 10 : 4;
        if (blen < ies) {
            return;
        }
        status = wifi_ap_ssid_is(ap, wifi_ap_find_ie(body + ies, blen - ies,
                                                     0), false)
                 ? 0 : STATUS_FAILURE;
        n = wifi_ap_hdr(ap, r, FC_MGMT(FC_STYPE(fc) + 1), sa, ap->bssid.a);
        stw_le_p(r + n, wifi_ap_capability(ap));
        stw_le_p(r + n + 2, status);
        /* AID 1, with the two bits 802.11 sets in the field */
        stw_le_p(r + n + 4, 0xc001);
        n += 6;
        r[n++] = 1;
        r[n++] = 8;
        memcpy(r + n, "\x82\x84\x8b\x96\x0c\x12\x18\x24", 8);
        n += 8;
        memcpy(r + n, "\x32\x04\x30\x48\x60\x6c", 6);
        n += 6;
        if (status) {
            wifi_ap_set_state(ap, WIFI_AP_IDLE);
        } else if (ap->psk) {
            /* message 1 of 4 once the station has taken the response in */
            wifi_ap_set_state(ap, WIFI_AP_ASSOCIATED);
            timer_mod(ap->eapol_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
        } else {
            wifi_ap_set_state(ap, WIFI_AP_CONNECTED);
        }
        wifi_ap_send(ap, r, n, false);
        return;
    case STYPE_DEAUTH:
    case STYPE_DISASSOC:
        if (!memcmp(sa, ap->sta.a, 6)) {
            wifi_ap_set_state(ap, WIFI_AP_IDLE);
        }
        return;
    }
}

static void wifi_ap_data_in(WifiAP *ap, uint16_t fc, const uint8_t *f,
                            size_t len)
{
    size_t hdr = HDR_LEN;
    g_autofree uint8_t *eth = NULL;
    uint16_t proto;

    /* null data carries no frame body; 4-address frames are not ours */
    if ((FC_STYPE(fc) & 4) || (fc & FC_FROMDS) || !(fc & FC_TODS) ||
        memcmp(f + 10, ap->sta.a, 6) || ap->state < WIFI_AP_ASSOCIATED) {
        return;
    }
    if (FC_STYPE(fc) & 8) {
        hdr += 2;
    }
    if (len < hdr + 8 || memcmp(f + hdr, rfc1042, sizeof(rfc1042))) {
        return;
    }
    proto = lduw_be_p(f + hdr + 6);
    if (proto == ETH_P_EAPOL) {
        wifi_ap_eapol(ap, f + hdr + 8, len - hdr - 8);
        return;
    }
    if (ap->state != WIFI_AP_CONNECTED || !ap->nic) {
        return;
    }
    /* Ethernet: the destination in address 3, the source in address 2 */
    len -= hdr + 8;
    eth = g_malloc(14 + len);
    memcpy(eth, f + 16, 6);
    memcpy(eth + 6, f + 10, 6);
    stw_be_p(eth + 12, proto);
    memcpy(eth + 14, f + hdr + 8, len);
    qemu_send_packet(qemu_get_queue(ap->nic), eth, 14 + len);
}

void wifi_ap_receive(WifiAP *ap, const uint8_t *f, size_t len)
{
    static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    uint16_t fc;

    if (len < HDR_LEN) {
        return;
    }
    fc = lduw_le_p(f);
    trace_wifi_ap_receive(fc, len);
    if (memcmp(f + 4, ap->bssid.a, 6) &&
        (FC_TYPE(fc) != TYPE_MGMT || memcmp(f + 4, bcast, 6))) {
        return;
    }
    switch (FC_TYPE(fc)) {
    case TYPE_MGMT:
        wifi_ap_mgmt(ap, fc, f, len);
        break;
    case TYPE_DATA:
        wifi_ap_data_in(ap, fc, f, len);
        break;
    }
}

void wifi_ap_net_receive(WifiAP *ap, const uint8_t *buf, size_t size)
{
    uint16_t proto;

    if (ap->state != WIFI_AP_CONNECTED || size < 14) {
        return;
    }
    /* unicast goes to the one station there is, or nowhere */
    if (!is_multicast_ether_addr(buf) && memcmp(buf, ap->sta.a, 6)) {
        return;
    }
    proto = lduw_be_p(buf + 12);
    if (proto < 0x0600) {
        return;
    }
    wifi_ap_data(ap, buf, buf + 6, proto, buf + 14, size - 14, !!ap->psk);
}

void wifi_ap_reset(WifiAP *ap)
{
    wifi_ap_set_state(ap, WIFI_AP_IDLE);
    memset(&ap->sta, 0, sizeof(ap->sta));
    ap->replay = 0;
}

bool wifi_ap_realize(WifiAP *ap, NICState *nic, WifiAPDeliverFn deliver,
                     void *opaque, Error **errp)
{
    size_t n;

    if (!ap->ssid) {
        ap->ssid = g_strdup("qemu");
    }
    if (!*ap->ssid || strlen(ap->ssid) > 32) {
        error_setg(errp, "ssid must be 1 to 32 bytes long");
        return false;
    }
    if (ap->channel < 1 || ap->channel > 13) {
        error_setg(errp, "channel must be 1 to 13");
        return false;
    }
    if (ap->psk) {
        n = strlen(ap->psk);
        if (n == 64) {
            /* 64 hex digits are the PMK itself */
            for (size_t i = 0; i < 32; i++) {
                if (!qemu_isxdigit(ap->psk[2 * i]) ||
                    !qemu_isxdigit(ap->psk[2 * i + 1])) {
                    error_setg(errp, "a 64 character psk must be hex");
                    return false;
                }
                ap->pmk[i] = g_ascii_xdigit_value(ap->psk[2 * i]) << 4 |
                             g_ascii_xdigit_value(ap->psk[2 * i + 1]);
            }
        } else if (n >= 8 && n <= 63) {
            if (!qcrypto_hmac_supports(QCRYPTO_HASH_ALGO_SHA1)) {
                error_setg(errp, "WPA2 needs HMAC-SHA1");
                return false;
            }
            wifi_ap_pmk(ap);
        } else {
            error_setg(errp, "psk must be 8 to 63 characters, or 64 hex");
            return false;
        }
        qemu_guest_getrandom_nofail(ap->gtk, sizeof(ap->gtk));
    }
    if (!memcmp(&ap->bssid, &(MACAddr){}, sizeof(ap->bssid))) {
        ap->bssid = default_bssid;
    }
    ap->nic = nic;
    ap->deliver = deliver;
    ap->opaque = opaque;
    ap->eapol_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, wifi_ap_eapol_timer,
                                   ap);
    return true;
}

void wifi_ap_unrealize(WifiAP *ap)
{
    timer_free(ap->eapol_timer);
    ap->eapol_timer = NULL;
}
