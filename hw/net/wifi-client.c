/*
 * A Wi-Fi client for an access point model to take in
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/guest-random.h"
#include "qemu/timer.h"
#include "hw/net/wifi-client.h"
#include "net/eth.h"
#include "trace.h"
#include "wifi-frame.h"
#include "wifi-rsn.h"

/* what a phone takes to notice a network, and to give one another go */
#define JOIN_DELAY_MS       200
#define RETRY_MS            500
#define BACKOFF_MS          2000
#define TRIES               5
/* how long the access point may take through the 4-way handshake */
#define KEYING_MS           5000

#define LISTEN_INTERVAL     10

static void wifi_client_set_state(WifiClient *cl, WifiClientState state)
{
    if (cl->state != state) {
        trace_wifi_client_state(cl->state, state);
    }
    cl->state = state;
}

static void wifi_client_send(WifiClient *cl, const uint8_t *f, size_t len,
                             bool protected)
{
    trace_wifi_client_send(lduw_le_p(f), len);
    cl->deliver(cl->opaque, f, len, protected);
}

/* To the access point from the client, on to @da */
static size_t wifi_client_hdr(WifiClient *cl, uint8_t *f, uint16_t fc,
                              const uint8_t *da)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, cl->bssid.a, 6);
    memcpy(f + 10, cl->mac.a, 6);
    memcpy(f + 16, da, 6);
    stw_le_p(f + 22, (cl->seq++ & 0xfff) << 4);
    return WIFI_HDR_LEN;
}

/* Turned away, or no answer: start over in a while */
static void wifi_client_backoff(WifiClient *cl)
{
    wifi_client_set_state(cl, WIFI_CLIENT_IDLE);
    cl->tries = 0;
    timer_mod(cl->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BACKOFF_MS);
}

/* Open authentication, then association, each sent until answered */
static void wifi_client_join(WifiClient *cl)
{
    uint8_t f[WIFI_HDR_LEN + 4 + 2 + sizeof(cl->ssid) +
              sizeof(WIFI_RATES_IE) + sizeof(WIFI_XRATES_IE) +
              sizeof(cl->rsn_ie)];
    size_t n;

    if (cl->tries++ >= TRIES) {
        wifi_client_backoff(cl);
        return;
    }
    if (cl->state == WIFI_CLIENT_ASSOCIATING) {
        n = wifi_client_hdr(cl, f, WIFI_FC_MGMT(WIFI_STYPE_ASSOC_REQ),
                            cl->bssid.a);
        stw_le_p(f + n, WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE |
                        WIFI_CAP_SHORT_SLOT);
        stw_le_p(f + n + 2, LISTEN_INTERVAL);
        n += 4;
        f[n++] = WIFI_EID_SSID;
        f[n++] = cl->ssid_len;
        memcpy(f + n, cl->ssid, cl->ssid_len);
        n += cl->ssid_len;
        memcpy(f + n, WIFI_RATES_IE, sizeof(WIFI_RATES_IE));
        n += sizeof(WIFI_RATES_IE);
        memcpy(f + n, WIFI_XRATES_IE, sizeof(WIFI_XRATES_IE));
        n += sizeof(WIFI_XRATES_IE);
        if (cl->rsn) {
            memcpy(f + n, cl->rsn_ie, sizeof(cl->rsn_ie));
            n += sizeof(cl->rsn_ie);
        }
    } else {
        wifi_client_set_state(cl, WIFI_CLIENT_AUTHENTICATING);
        n = wifi_client_hdr(cl, f, WIFI_FC_MGMT(WIFI_STYPE_AUTH),
                            cl->bssid.a);
        /* open system, transaction 1 */
        stw_le_p(f + n, 0);
        stw_le_p(f + n + 2, 1);
        stw_le_p(f + n + 4, 0);
        n += 6;
    }
    wifi_client_send(cl, f, n, false);
    timer_mod(cl->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + RETRY_MS);
}

static void wifi_client_timer(void *opaque)
{
    WifiClient *cl = opaque;

    if (cl->state == WIFI_CLIENT_KEYING) {
        wifi_client_backoff(cl);
    } else if (cl->bss && cl->state != WIFI_CLIENT_CONNECTED) {
        wifi_client_join(cl);
    }
}

/* Keys in hand, or none needed: the backend's frames may go */
static void wifi_client_connected(WifiClient *cl)
{
    timer_del(cl->timer);
    wifi_client_set_state(cl, WIFI_CLIENT_CONNECTED);
    qemu_flush_queued_packets(qemu_get_queue(cl->nic));
}

static void wifi_client_mgmt(WifiClient *cl, uint16_t fc, const uint8_t *f,
                             size_t len)
{
    const uint8_t *body = f + WIFI_HDR_LEN;
    size_t blen = len - WIFI_HDR_LEN;

    switch (WIFI_FC_STYPE(fc)) {
    case WIFI_STYPE_AUTH:
        /* algorithm, transaction, status */
        if (cl->state != WIFI_CLIENT_AUTHENTICATING || blen < 6 ||
            lduw_le_p(body) != 0 || lduw_le_p(body + 2) != 2) {
            return;
        }
        if (lduw_le_p(body + 4)) {
            wifi_client_backoff(cl);
            return;
        }
        wifi_client_set_state(cl, WIFI_CLIENT_ASSOCIATING);
        cl->tries = 0;
        wifi_client_join(cl);
        return;
    case WIFI_STYPE_ASSOC_RESP:
    case WIFI_STYPE_REASSOC_RESP:
        /* capability, status, association id */
        if (cl->state != WIFI_CLIENT_ASSOCIATING || blen < 6) {
            return;
        }
        if (lduw_le_p(body + 2)) {
            trace_wifi_client_refused(lduw_le_p(body + 2));
            wifi_client_backoff(cl);
            return;
        }
        if (!cl->rsn) {
            wifi_client_connected(cl);
            return;
        }
        wifi_client_set_state(cl, WIFI_CLIENT_KEYING);
        cl->replay = 0;
        timer_mod(cl->timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + KEYING_MS);
        return;
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (cl->state != WIFI_CLIENT_IDLE) {
            trace_wifi_client_dropped(blen >= 2 ? lduw_le_p(body) : 0);
            wifi_client_backoff(cl);
        }
        return;
    }
}

/* An LLC/SNAP data frame to the distribution system, on to @da */
static void wifi_client_data(WifiClient *cl, const uint8_t *da,
                             uint16_t proto, const uint8_t *payload,
                             size_t len, bool protected)
{
    g_autofree uint8_t *f = g_malloc(WIFI_HDR_LEN + 8 + len);
    size_t n = wifi_client_hdr(cl, f, WIFI_FC_DATA | WIFI_FC_TODS, da);

    memcpy(f + n, WIFI_RFC1042, sizeof(WIFI_RFC1042));
    stw_be_p(f + n + 6, proto);
    memcpy(f + n + 8, payload, len);
    wifi_client_send(cl, f, n + 8 + len, protected);
}

/* The GTK KDE among the key data of message 3 */
static bool wifi_client_gtk(WifiClient *cl, const uint8_t *kd, size_t len)
{
    static const uint8_t gtk_kde[] = { 0x00, 0x0f, 0xac, 1 };

    for (size_t i = 0; i + 2 <= len && kd[i + 1]; i += 2 + kd[i + 1]) {
        size_t n = kd[i + 1];

        if (kd[i] == 0xdd && n >= 6 + 16 && i + 2 + n <= len &&
            !memcmp(kd + i + 2, gtk_kde, sizeof(gtk_kde))) {
            /* key id and a reserved byte, then the key */
            memset(cl->gtk, 0, sizeof(cl->gtk));
            memcpy(cl->gtk, kd + i + 8, MIN(n - 6, sizeof(cl->gtk)));
            return true;
        }
    }
    return false;
}

/*
 * Messages 1 and 3 of the 4-way handshake, answered with 2 and 4: the PTK
 * from both nonces, then the GTK out of its wrap
 */
static void wifi_client_eapol(WifiClient *cl, const uint8_t *p, size_t len)
{
    uint8_t f[WIFI_KEY_DATA + sizeof(cl->rsn_ie)] = {};
    uint16_t info;
    size_t elen, data_len = 0;
    int msg;

    if (cl->state != WIFI_CLIENT_KEYING || len < WIFI_KEY_DATA ||
        p[1] != WIFI_EAPOL_KEY || p[WIFI_KEY_DESC] != WIFI_KEY_DESC_RSN) {
        return;
    }
    /* the MIC covers the length the frame gives, its fixed fields at least */
    elen = MIN(len, 4 + lduw_be_p(p + 2));
    info = lduw_be_p(p + WIFI_KEY_INFO);
    if (elen < WIFI_KEY_DATA || !(info & WIFI_KI_ACK) ||
        !(info & WIFI_KI_PAIRWISE) ||
        ldq_be_p(p + WIFI_KEY_REPLAY) < cl->replay) {
        return;
    }
    if (!(info & WIFI_KI_MIC)) {
        memcpy(cl->anonce, p + WIFI_KEY_NONCE, sizeof(cl->anonce));
        qemu_guest_getrandom_nofail(cl->snonce, sizeof(cl->snonce));
        wifi_rsn_ptk(cl->pmk, cl->bssid.a, cl->mac.a, cl->anonce,
                     cl->snonce, cl->ptk);
        info = WIFI_KI_HMAC_SHA1_AES | WIFI_KI_PAIRWISE | WIFI_KI_MIC;
        memcpy(f + WIFI_KEY_NONCE, cl->snonce, sizeof(cl->snonce));
        memcpy(f + WIFI_KEY_DATA, cl->rsn_ie, sizeof(cl->rsn_ie));
        data_len = sizeof(cl->rsn_ie);
        msg = 2;
    } else {
        size_t wrapped = lduw_be_p(p + WIFI_KEY_DATA_LEN);
        uint8_t mic[16], kd[128];
        bool ok;

        wifi_rsn_mic(cl->ptk, p, elen, mic);
        ok = !memcmp(mic, p + WIFI_KEY_MIC, sizeof(mic)) &&
             (info & WIFI_KI_ENC_DATA) && wrapped <= sizeof(kd) + 8 &&
             WIFI_KEY_DATA + wrapped <= elen &&
             wifi_rsn_unwrap(cl->ptk, p + WIFI_KEY_DATA, wrapped, kd) &&
             wifi_client_gtk(cl, kd, wrapped - 8);
        trace_wifi_client_eapol(3, ok);
        /* a wrong passphrase: leave it to the timeout to start over */
        if (!ok) {
            return;
        }
        info = WIFI_KI_HMAC_SHA1_AES | WIFI_KI_PAIRWISE | WIFI_KI_MIC |
               WIFI_KI_SECURE;
        msg = 4;
    }
    cl->replay = ldq_be_p(p + WIFI_KEY_REPLAY);
    f[0] = p[0];
    f[1] = WIFI_EAPOL_KEY;
    stw_be_p(f + 2, WIFI_KEY_DATA - 4 + data_len);
    f[WIFI_KEY_DESC] = WIFI_KEY_DESC_RSN;
    stw_be_p(f + WIFI_KEY_INFO, info);
    stq_be_p(f + WIFI_KEY_REPLAY, cl->replay);
    stw_be_p(f + WIFI_KEY_DATA_LEN, data_len);
    wifi_rsn_mic(cl->ptk, f, WIFI_KEY_DATA + data_len, f + WIFI_KEY_MIC);
    trace_wifi_client_eapol(msg, true);
    wifi_client_data(cl, cl->bssid.a, WIFI_ETH_P_EAPOL, f,
                     WIFI_KEY_DATA + data_len, false);
    if (msg == 4) {
        wifi_client_connected(cl);
    }
}

/* From the distribution system: destination in address 1, source in 3 */
static void wifi_client_data_in(WifiClient *cl, uint16_t fc,
                                const uint8_t *f, size_t len)
{
    size_t hdr = WIFI_HDR_LEN;
    g_autofree uint8_t *eth = NULL;

    if ((cl->state != WIFI_CLIENT_CONNECTED &&
         cl->state != WIFI_CLIENT_KEYING) ||
        (WIFI_FC_STYPE(fc) & WIFI_STYPE_NODATA) ||
        !(fc & WIFI_FC_FROMDS) || (fc & WIFI_FC_TODS)) {
        return;
    }
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        hdr += 2;
    }
    if (len < hdr + 8 ||
        memcmp(f + hdr, WIFI_RFC1042, sizeof(WIFI_RFC1042))) {
        return;
    }
    if (lduw_be_p(f + hdr + 6) == WIFI_ETH_P_EAPOL) {
        wifi_client_eapol(cl, f + hdr + 8, len - hdr - 8);
        return;
    }
    if (cl->state != WIFI_CLIENT_CONNECTED) {
        return;
    }
    /* the protocol and what follows it, behind both addresses */
    len -= hdr + sizeof(WIFI_RFC1042);
    eth = g_malloc(12 + len);
    memcpy(eth, f + 4, 6);
    memcpy(eth + 6, f + 16, 6);
    memcpy(eth + 12, f + hdr + sizeof(WIFI_RFC1042), len);
    qemu_send_packet(qemu_get_queue(cl->nic), eth, 12 + len);
}

void wifi_client_receive(WifiClient *cl, const uint8_t *f, size_t len)
{
    uint16_t fc;

    if (!cl->bss || len < WIFI_HDR_LEN) {
        return;
    }
    /* from the access point, to the client or to everyone */
    if (memcmp(f + 10, cl->bssid.a, 6) ||
        (!is_multicast_ether_addr(f + 4) && memcmp(f + 4, cl->mac.a, 6))) {
        return;
    }
    fc = lduw_le_p(f);
    trace_wifi_client_receive(fc, len);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        wifi_client_mgmt(cl, fc, f, len);
        break;
    case WIFI_TYPE_DATA:
        wifi_client_data_in(cl, fc, f, len);
        break;
    }
}

bool wifi_client_acks(WifiClient *cl, const uint8_t *ra)
{
    return cl->bss && !memcmp(ra, cl->mac.a, 6);
}

bool wifi_client_can_receive(WifiClient *cl)
{
    return cl->state == WIFI_CLIENT_CONNECTED;
}

void wifi_client_net_receive(WifiClient *cl, const uint8_t *buf, size_t size)
{
    /* Ethernet II only: the protocol is what LLC/SNAP carries */
    if (cl->state != WIFI_CLIENT_CONNECTED || size < 14 ||
        lduw_be_p(buf + 12) < 0x0600) {
        return;
    }
    wifi_client_data(cl, buf, lduw_be_p(buf + 12), buf + 14, size - 14,
                     cl->rsn);
}

/*
 * The RSN element the client associates with, as a phone picks it from the
 * BSS's: the BSS's group cipher, CCMP for itself where offered, PSK
 */
static bool wifi_client_pick_rsn(WifiClient *cl, const uint8_t *rsn)
{
    static const uint8_t ccmp[4] = { 0x00, 0x0f, 0xac, 4 };
    const uint8_t *pairwise = rsn + 10;
    size_t len = rsn[1], n;

    /* version, group cipher, a pairwise count and at least one cipher */
    if (len < 12 || lduw_le_p(rsn + 2) != 1) {
        return false;
    }
    n = lduw_le_p(rsn + 8);
    for (size_t i = 0; i < n && 12 + 4 * i <= len; i++) {
        if (!memcmp(rsn + 10 + 4 * i, ccmp, sizeof(ccmp))) {
            pairwise = ccmp;
        }
    }
    memcpy(cl->rsn_ie, wifi_rsn_ie, sizeof(cl->rsn_ie));
    memcpy(cl->rsn_ie + 4, rsn + 4, 4);
    memcpy(cl->rsn_ie + 10, pairwise, 4);
    trace_wifi_client_rsn(rsn[7], pairwise[3]);
    return true;
}

void wifi_client_bss(WifiClient *cl, const uint8_t *bssid,
                     const uint8_t *ssid, size_t ssid_len,
                     const uint8_t *rsn)
{
    ssid_len = MIN(ssid_len, sizeof(cl->ssid));
    /* the access point announcing itself again changes nothing */
    if (bssid && cl->bss && !memcmp(bssid, cl->bssid.a, 6) &&
        ssid_len == cl->ssid_len && !memcmp(ssid, cl->ssid, ssid_len) &&
        cl->rsn == (rsn && cl->psk)) {
        return;
    }
    timer_del(cl->timer);
    wifi_client_set_state(cl, WIFI_CLIENT_IDLE);
    cl->tries = 0;
    cl->bss = bssid != NULL;
    if (!bssid) {
        return;
    }
    memcpy(cl->bssid.a, bssid, 6);
    memcpy(cl->ssid, ssid, ssid_len);
    cl->ssid_len = ssid_len;
    cl->rsn = rsn && cl->psk && wifi_client_pick_rsn(cl, rsn);
    if (cl->rsn) {
        wifi_rsn_pmk(cl->psk, cl->ssid, cl->ssid_len, cl->pmk);
    }
    timer_mod(cl->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                         JOIN_DELAY_MS);
}

void wifi_client_init(WifiClient *cl, NICState *nic, const MACAddr *mac,
                      WifiClientDeliverFn deliver, void *opaque)
{
    cl->nic = nic;
    cl->mac = *mac;
    cl->deliver = deliver;
    cl->opaque = opaque;
    cl->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, wifi_client_timer, cl);
}

void wifi_client_cleanup(WifiClient *cl)
{
    timer_free(cl->timer);
    cl->timer = NULL;
}
