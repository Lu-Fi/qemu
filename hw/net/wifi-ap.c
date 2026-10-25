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
#include "crypto/hmac.h"
#include "hw/net/wifi-ap.h"
#include "net/eth.h"
#include "trace.h"
#include "wifi-frame.h"
#include "wifi-rsn.h"

#define REASON_4WAY_TIMEOUT 15
#define STATUS_FAILURE      1
#define STATUS_BAD_ALG      13

#define EAPOL_RETRY_MS      1000
#define EAPOL_TRIES         4

static const MACAddr default_bssid = { .a = { 0x52, 0x54, 0x00, 0x77,
                                              0x69, 0x01 } };

static size_t wifi_ap_hdr(WifiAP *ap, uint8_t *f, uint16_t fc,
                          const uint8_t *da, const uint8_t *sa)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, da, 6);
    memcpy(f + 10, ap->bssid.a, 6);
    memcpy(f + 16, sa, 6);
    stw_le_p(f + 22, (ap->seq++ & 0xfff) << 4);
    return WIFI_HDR_LEN;
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
    static const uint8_t tim[] = { 5, 4, 0, 1, 0, 0 };
    static const uint8_t erp[] = { 42, 1, 0 };
    size_t n = strlen(ap->ssid);
    uint8_t *start = p;

    *p++ = 0;
    *p++ = n;
    memcpy(p, ap->ssid, n);
    p += n;
    memcpy(p, WIFI_RATES_IE, sizeof(WIFI_RATES_IE));
    p += sizeof(WIFI_RATES_IE);
    *p++ = 3;
    *p++ = 1;
    *p++ = ap->channel;
    if (beacon) {
        memcpy(p, tim, sizeof(tim));
        p += sizeof(tim);
    }
    memcpy(p, erp, sizeof(erp));
    p += sizeof(erp);
    memcpy(p, WIFI_XRATES_IE, sizeof(WIFI_XRATES_IE));
    p += sizeof(WIFI_XRATES_IE);
    if (ap->psk) {
        memcpy(p, wifi_rsn_ie, sizeof(wifi_rsn_ie));
        p += sizeof(wifi_rsn_ie);
    }
    return p - start;
}

static uint16_t wifi_ap_capability(WifiAP *ap)
{
    return WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE | WIFI_CAP_SHORT_SLOT |
           (ap->psk ? WIFI_CAP_PRIVACY : 0);
}

static size_t wifi_ap_announce(WifiAP *ap, uint8_t *f, unsigned stype,
                               const uint8_t *da)
{
    size_t len = wifi_ap_hdr(ap, f, WIFI_FC_MGMT(stype), da, ap->bssid.a);

    stq_le_p(f + len, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    stw_le_p(f + len + 8, 100);
    stw_le_p(f + len + 10, wifi_ap_capability(ap));
    len += 12;
    return len + wifi_ap_ies(ap, f + len, stype == WIFI_STYPE_BEACON);
}

size_t wifi_ap_beacon(WifiAP *ap, uint8_t *buf)
{
    static const uint8_t bcast[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

    return wifi_ap_announce(ap, buf, WIFI_STYPE_BEACON, bcast);
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
    uint8_t f[WIFI_HDR_LEN + 2];

    wifi_ap_hdr(ap, f, WIFI_FC_MGMT(WIFI_STYPE_DEAUTH), ap->sta.a, ap->bssid.a);
    stw_le_p(f + WIFI_HDR_LEN, reason);
    wifi_ap_send(ap, f, sizeof(f), false);
    wifi_ap_set_state(ap, WIFI_AP_IDLE);
}

/* An LLC/SNAP data frame from the distribution system to the station */
static void wifi_ap_data(WifiAP *ap, const uint8_t *da, const uint8_t *sa,
                         uint16_t proto, const uint8_t *payload, size_t len,
                         bool protected)
{
    g_autofree uint8_t *f = g_malloc(WIFI_HDR_LEN + 8 + len);

    wifi_ap_hdr(ap, f, WIFI_FC_DATA | WIFI_FC_FROMDS, da, sa);
    memcpy(f + WIFI_HDR_LEN, WIFI_RFC1042, sizeof(WIFI_RFC1042));
    stw_be_p(f + WIFI_HDR_LEN + 6, proto);
    memcpy(f + WIFI_HDR_LEN + 8, payload, len);
    wifi_ap_send(ap, f, WIFI_HDR_LEN + 8 + len, protected);
}

/* Message 1 or 3 of the 4-way handshake, the latter carrying the GTK */
static void wifi_ap_eapol_send(WifiAP *ap)
{
    bool msg3 = ap->state == WIFI_AP_PTK_NEGOTIATING;
    uint8_t f[WIFI_KEY_DATA + 64] = {};
    uint16_t info = WIFI_KI_HMAC_SHA1_AES | WIFI_KI_PAIRWISE | WIFI_KI_ACK;
    size_t data_len = 0;

    ap->replay++;
    if (msg3) {
        uint8_t kd[48] = {}, *p = kd;

        info |= WIFI_KI_INSTALL | WIFI_KI_MIC | WIFI_KI_SECURE |
                WIFI_KI_ENC_DATA;
        memcpy(p, wifi_rsn_ie, sizeof(wifi_rsn_ie));
        p += sizeof(wifi_rsn_ie);
        /* the GTK KDE: the key id, then padding to the wrap's 8 bytes */
        *p++ = 0xdd;
        *p++ = 22;
        *p++ = 0x00;
        *p++ = 0x0f;
        *p++ = 0xac;
        *p++ = 1;
        *p++ = WIFI_AP_GTK_ID;
        *p++ = 0;
        memcpy(p, ap->gtk, sizeof(ap->gtk));
        p += sizeof(ap->gtk);
        *p = 0xdd;
        data_len = sizeof(kd) + 8;
        wifi_rsn_wrap(ap->ptk, kd, sizeof(kd), f + WIFI_KEY_DATA);
    }
    f[0] = 2;
    f[1] = WIFI_EAPOL_KEY;
    stw_be_p(f + 2, WIFI_KEY_DATA - 4 + data_len);
    f[WIFI_KEY_DESC] = WIFI_KEY_DESC_RSN;
    stw_be_p(f + WIFI_KEY_INFO, info);
    stw_be_p(f + WIFI_KEY_LEN, 16);
    stq_be_p(f + WIFI_KEY_REPLAY, ap->replay);
    memcpy(f + WIFI_KEY_NONCE, ap->anonce, sizeof(ap->anonce));
    stw_be_p(f + WIFI_KEY_DATA_LEN, data_len);
    if (msg3) {
        wifi_rsn_mic(ap->ptk, f, WIFI_KEY_DATA + data_len, f + WIFI_KEY_MIC);
    }
    trace_wifi_ap_eapol_send(msg3 ? 3 : 1, ap->replay);
    wifi_ap_data(ap, ap->sta.a, ap->bssid.a, WIFI_ETH_P_EAPOL, f,
                 WIFI_KEY_DATA + data_len, false);
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

    if (len < WIFI_KEY_DATA || p[1] != WIFI_EAPOL_KEY ||
        p[WIFI_KEY_DESC] != WIFI_KEY_DESC_RSN) {
        return;
    }
    /* the MIC covers the length the frame gives, its fixed fields at least */
    elen = MIN(len, 4 + lduw_be_p(p + 2));
    info = lduw_be_p(p + WIFI_KEY_INFO);
    if (elen < WIFI_KEY_DATA || !(info & WIFI_KI_MIC) ||
        ldq_be_p(p + WIFI_KEY_REPLAY) != ap->replay) {
        return;
    }
    if (ap->state == WIFI_AP_PTK_START && !(info & WIFI_KI_SECURE)) {
        wifi_rsn_ptk(ap->pmk, ap->bssid.a, ap->sta.a, ap->anonce,
                     p + WIFI_KEY_NONCE, ap->ptk);
        wifi_rsn_mic(ap->ptk, p, elen, mic);
        trace_wifi_ap_eapol_recv(2, !memcmp(mic, p + WIFI_KEY_MIC, 16));
        /* a wrong passphrase: leave it to the retries to give up */
        if (memcmp(mic, p + WIFI_KEY_MIC, 16)) {
            return;
        }
        ap->tries = 0;
        wifi_ap_set_state(ap, WIFI_AP_PTK_NEGOTIATING);
        wifi_ap_eapol_send(ap);
    } else if (ap->state == WIFI_AP_PTK_NEGOTIATING &&
               (info & WIFI_KI_SECURE)) {
        wifi_rsn_mic(ap->ptk, p, elen, mic);
        trace_wifi_ap_eapol_recv(4, !memcmp(mic, p + WIFI_KEY_MIC, 16));
        if (!memcmp(mic, p + WIFI_KEY_MIC, 16)) {
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
    const uint8_t *sa = f + 10, *body = f + WIFI_HDR_LEN;
    size_t blen = len - WIFI_HDR_LEN;
    uint8_t r[WIFI_AP_BEACON_MAX];
    size_t n, ies;
    uint16_t status;

    switch (WIFI_FC_STYPE(fc)) {
    case WIFI_STYPE_PROBE_REQ:
        if (wifi_ap_ssid_is(ap, wifi_ap_find_ie(body, blen, WIFI_EID_SSID),
                            true)) {
            n = wifi_ap_announce(ap, r, WIFI_STYPE_PROBE_RESP, sa);
            wifi_ap_send(ap, r, n, false);
        }
        return;
    case WIFI_STYPE_AUTH:
        if (blen < 6 || lduw_le_p(body + 2) != 1) {
            return;
        }
        status = lduw_le_p(body) ? STATUS_BAD_ALG : 0;
        n = wifi_ap_hdr(ap, r, WIFI_FC_MGMT(WIFI_STYPE_AUTH), sa, ap->bssid.a);
        stw_le_p(r + n, lduw_le_p(body));
        stw_le_p(r + n + 2, 2);
        stw_le_p(r + n + 4, status);
        memcpy(ap->sta.a, sa, 6);
        wifi_ap_set_state(ap, status ? WIFI_AP_IDLE : WIFI_AP_AUTHENTICATED);
        wifi_ap_send(ap, r, n + 6, false);
        return;
    case WIFI_STYPE_ASSOC_REQ:
    case WIFI_STYPE_REASSOC_REQ:
        if (memcmp(sa, ap->sta.a, 6) || ap->state < WIFI_AP_AUTHENTICATED) {
            return;
        }
        ies = WIFI_FC_STYPE(fc) == WIFI_STYPE_REASSOC_REQ ? 10 : 4;
        if (blen < ies) {
            return;
        }
        status = wifi_ap_ssid_is(ap, wifi_ap_find_ie(body + ies, blen - ies,
                                                     WIFI_EID_SSID), false)
                 ? 0 : STATUS_FAILURE;
        n = wifi_ap_hdr(ap, r, WIFI_FC_MGMT(WIFI_FC_STYPE(fc) + 1), sa,
                        ap->bssid.a);
        stw_le_p(r + n, wifi_ap_capability(ap));
        stw_le_p(r + n + 2, status);
        /* AID 1, with the two bits 802.11 sets in the field */
        stw_le_p(r + n + 4, 0xc001);
        n += 6;
        memcpy(r + n, WIFI_RATES_IE, sizeof(WIFI_RATES_IE));
        n += sizeof(WIFI_RATES_IE);
        memcpy(r + n, WIFI_XRATES_IE, sizeof(WIFI_XRATES_IE));
        n += sizeof(WIFI_XRATES_IE);
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
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (!memcmp(sa, ap->sta.a, 6)) {
            wifi_ap_set_state(ap, WIFI_AP_IDLE);
        }
        return;
    }
}

static void wifi_ap_data_in(WifiAP *ap, uint16_t fc, const uint8_t *f,
                            size_t len)
{
    size_t hdr = WIFI_HDR_LEN;
    g_autofree uint8_t *eth = NULL;
    uint16_t proto;

    /* null data carries no frame body; 4-address frames are not ours */
    if ((WIFI_FC_STYPE(fc) & WIFI_STYPE_NODATA) || (fc & WIFI_FC_FROMDS) ||
        !(fc & WIFI_FC_TODS) || memcmp(f + 10, ap->sta.a, 6) ||
        ap->state < WIFI_AP_ASSOCIATED) {
        return;
    }
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        hdr += 2;
    }
    if (len < hdr + 8 || memcmp(f + hdr, WIFI_RFC1042, sizeof(WIFI_RFC1042))) {
        return;
    }
    proto = lduw_be_p(f + hdr + 6);
    if (proto == WIFI_ETH_P_EAPOL) {
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

    if (len < WIFI_HDR_LEN) {
        return;
    }
    fc = lduw_le_p(f);
    trace_wifi_ap_receive(fc, len);
    if (memcmp(f + 4, ap->bssid.a, 6) &&
        (WIFI_FC_TYPE(fc) != WIFI_TYPE_MGMT || memcmp(f + 4, bcast, 6))) {
        return;
    }
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        wifi_ap_mgmt(ap, fc, f, len);
        break;
    case WIFI_TYPE_DATA:
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
            wifi_rsn_pmk(ap->psk, (const uint8_t *)ap->ssid, strlen(ap->ssid),
                         ap->pmk);
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
