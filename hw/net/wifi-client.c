/*
 * A Wi-Fi client for an access point model to take in
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "hw/net/wifi-client.h"
#include "net/eth.h"
#include "trace.h"
#include "wifi-frame.h"

/* what a phone takes to notice a network, and to give one another go */
#define JOIN_DELAY_MS       200
#define RETRY_MS            500
#define BACKOFF_MS          2000
#define TRIES               5

#define LISTEN_INTERVAL     10

static void wifi_client_set_state(WifiClient *cl, WifiClientState state)
{
    if (cl->state != state) {
        trace_wifi_client_state(cl->state, state);
    }
    cl->state = state;
}

static void wifi_client_send(WifiClient *cl, const uint8_t *f, size_t len)
{
    trace_wifi_client_send(lduw_le_p(f), len);
    cl->deliver(cl->opaque, f, len);
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
              sizeof(WIFI_RATES_IE) + sizeof(WIFI_XRATES_IE)];
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
    wifi_client_send(cl, f, n);
    timer_mod(cl->timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + RETRY_MS);
}

static void wifi_client_timer(void *opaque)
{
    WifiClient *cl = opaque;

    if (cl->bss && cl->state != WIFI_CLIENT_CONNECTED) {
        wifi_client_join(cl);
    }
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
            wifi_client_backoff(cl);
            return;
        }
        timer_del(cl->timer);
        wifi_client_set_state(cl, WIFI_CLIENT_CONNECTED);
        qemu_flush_queued_packets(qemu_get_queue(cl->nic));
        return;
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (cl->state != WIFI_CLIENT_IDLE) {
            wifi_client_backoff(cl);
        }
        return;
    }
}

/* From the distribution system: destination in address 1, source in 3 */
static void wifi_client_data_in(WifiClient *cl, uint16_t fc,
                                const uint8_t *f, size_t len)
{
    size_t hdr = WIFI_HDR_LEN;
    g_autofree uint8_t *eth = NULL;

    if (cl->state != WIFI_CLIENT_CONNECTED ||
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
    g_autofree uint8_t *f = NULL;
    size_t n;

    /* Ethernet II only: the protocol is what LLC/SNAP carries */
    if (cl->state != WIFI_CLIENT_CONNECTED || size < 14 ||
        lduw_be_p(buf + 12) < 0x0600) {
        return;
    }
    f = g_malloc(WIFI_HDR_LEN + sizeof(WIFI_RFC1042) + size - 12);
    n = wifi_client_hdr(cl, f, WIFI_FC_DATA | WIFI_FC_TODS, buf);
    memcpy(f + n, WIFI_RFC1042, sizeof(WIFI_RFC1042));
    n += sizeof(WIFI_RFC1042);
    memcpy(f + n, buf + 12, size - 12);
    wifi_client_send(cl, f, n + size - 12);
}

void wifi_client_bss(WifiClient *cl, const uint8_t *bssid,
                     const uint8_t *ssid, size_t ssid_len)
{
    ssid_len = MIN(ssid_len, sizeof(cl->ssid));
    /* the access point announcing itself again changes nothing */
    if (bssid && cl->bss && !memcmp(bssid, cl->bssid.a, 6) &&
        ssid_len == cl->ssid_len && !memcmp(ssid, cl->ssid, ssid_len)) {
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
