/*
 * Broadcom FullMAC Wi-Fi: the dongle firmware the bus front ends share
 *
 * The host's bcmdhd driver talks to the firmware rather than to the MAC:
 * BCDC ioctls on the control channel, events back, Ethernet frames for
 * data. This model is that firmware. It scans for and joins the access
 * point (wifi-ap.c) itself, leaving the 4-way handshake to the host's
 * supplicant, and runs an access point of its own for a client
 * (wifi-client.c) to join.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev.h"
#include "net/eth.h"
#include "bcm-wifi.h"
#include "trace.h"
#include "wifi-frame.h"

/* cdc_ioctl_t: command, buffer length, flags, status; then the buffer */
#define CDC_HDR_LEN             16
#define CDCF_IOC_ERROR          BIT(0)
#define CDCF_IOC_SET            BIT(1)
/* the most the host takes back */
#define CDC_BUF_MAX             8192

/* BDC: flags with the version, priority, interface, then 4-byte words */
#define BDC_HDR_LEN             4
#define BDC_PROTO_VER           0x20

#define WLC_GET_VERSION         1
#define WLC_UP                  2
#define WLC_DOWN                3
#define WLC_SET_PROMISC         10
#define WLC_GET_RATE            12
#define WLC_SET_INFRA           20
#define WLC_GET_BSSID           23
#define WLC_GET_SSID            25
#define WLC_SET_SSID            26
#define WLC_GET_CHANNEL         29
#define WLC_SET_CHANNEL         30
#define WLC_SET_PASSIVE_SCAN    49
#define WLC_SCAN                50
#define WLC_DISASSOC            52
#define WLC_SET_ROAM_TRIGGER    55
#define WLC_SET_ROAM_DELTA      57
#define WLC_SET_ROAM_SCAN_PERIOD 59
#define WLC_SET_BCNPRD          76
#define WLC_GET_DTIMPRD         77
#define WLC_SET_DTIMPRD         78
#define WLC_GET_PM              85
#define WLC_SET_PM              86
#define WLC_GET_REVINFO         98
#define WLC_GET_AP              117
#define WLC_SET_AP              118
#define WLC_SCB_AUTHORIZE       121
#define WLC_SCB_DEAUTHORIZE     122
#define WLC_GET_RSSI            127
#define WLC_GET_BSS_INFO        136
#define WLC_GET_PKTCNTS         137
#define WLC_GET_BANDLIST        140
#define WLC_GET_BAND            141
#define WLC_SET_BAND            142
#define WLC_GET_ASSOCLIST       159
#define WLC_SET_SCAN_CHANNEL_TIME 185
#define WLC_SET_SCAN_UNASSOC_TIME 187
#define WLC_SCB_DEAUTHENTICATE_FOR_REASON 201
#define WLC_GET_VALID_CHANNELS  217
#define WLC_SET_KEY_PRIMARY     236
#define WLC_SET_SCAN_PASSIVE_TIME 258
#define WLC_GET_VAR             262
#define WLC_SET_VAR             263

/* what WLC_GET_VERSION answers: chanspecs in the 802.11ac layout */
#define WLC_IOCTL_VERSION       2
#define WLC_BAND_2G             2

#define BCME_OK                 0
#define BCME_BADARG             (-2)
#define BCME_NOTASSOCIATED      (-17)
#define BCME_UNSUPPORTED        (-23)
#define BCME_NOTFOUND           (-30)

#define WLC_E_SET_SSID          0
#define WLC_E_AUTH              3
#define WLC_E_AUTH_IND          4
#define WLC_E_DEAUTH_IND        6
#define WLC_E_ASSOC             7
#define WLC_E_ASSOC_IND         8
#define WLC_E_DISASSOC          11
#define WLC_E_DISASSOC_IND      12
#define WLC_E_LINK              16
#define WLC_E_ESCAN_RESULT      69

#define WLC_E_STATUS_SUCCESS    0
#define WLC_E_STATUS_FAIL       1
#define WLC_E_STATUS_NO_NETWORKS 3
#define WLC_E_STATUS_ABORT      4
#define WLC_E_STATUS_PARTIAL    8
#define WLC_EVENT_MSG_LINK      BIT(0)

/*
 * bcm_event_t: an Ethernet header, Broadcom's vendor header, then
 * wl_event_msg_t, whose fields are the only big-endian ones on the link
 */
#define ETH_P_BRCM              0x886c
#define BCM_EVENT_LEN           72
#define BCMILCP_SUBTYPE_VENDOR  0x8001
#define BCMILCP_BCM_EVENT       1
#define WL_EVENT_MSG_VERSION    2

/* wl_bss_info_t version 109: the fixed part, then the IEs */
#define BSS_INFO_VERSION        109
#define BSS_INFO_LEN            128
#define BSS_FROM_BEACON         0x01
#define BSS_RSSI_ONCHANNEL      0x04
/* wl_escan_result_t: buffer length, version, sync id, BSS count */
#define ESCAN_RESULT_LEN        12
#define ESCAN_ACTION_START      1
#define ESCAN_ACTION_ABORT      3

/* a 20 MHz 2.4 GHz chanspec, the channel in the low byte */
#define CHANSPEC_2G_20(ch)      (0x1000 | (ch))
#define CHANNELS_2G             11

/* sta_info_t flags: authenticated, associated, authorized */
#define WL_STA_AUTHE            0x08
#define WL_STA_ASSOC            0x10
#define WL_STA_AUTHO            0x20

#define RSSI_DBM                (-40)
#define NOISE_DBM               (-90)
#define RATE_54M                108
#define LISTEN_INTERVAL         10
/* AID 1, with the two bits 802.11 sets in the field */
#define PEER_AID                0xc001

#define OUT_QUEUE_MAX           256
#define SCAN_MS                 300
#define JOIN_MS                 20
#define RETRY_MS                300
#define TRIES                   3

static void bcm_queue(BcmCore *c, unsigned chan, GByteArray *data)
{
    BcmMsg *m = g_new(BcmMsg, 1);

    m->chan = chan;
    m->data = data;
    g_queue_push_tail(&c->out, m);
    c->notify(c);
}

BcmMsg *bcm_core_peek(BcmCore *c)
{
    return g_queue_peek_head(&c->out);
}

BcmMsg *bcm_core_pop(BcmCore *c)
{
    BcmMsg *m = g_queue_pop_head(&c->out);

    if (m && g_queue_get_length(&c->out) == OUT_QUEUE_MAX - 1) {
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        if (c->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(c->client_nic));
        }
    }
    return m;
}

void bcm_msg_free(BcmMsg *m)
{
    g_byte_array_unref(m->data);
    g_free(m);
}

static void bcm_flush(BcmCore *c)
{
    BcmMsg *m;

    while ((m = g_queue_pop_head(&c->out))) {
        bcm_msg_free(m);
    }
}

static void bcm_event(BcmCore *c, uint32_t type, uint32_t status,
                      uint32_t reason, const uint8_t *addr, uint16_t flags,
                      const uint8_t *data, size_t dlen)
{
    GByteArray *e = g_byte_array_sized_new(BDC_HDR_LEN + BCM_EVENT_LEN +
                                           dlen);
    uint8_t *p;

    trace_bcm_event(type, status, reason);
    g_byte_array_set_size(e, BDC_HDR_LEN + BCM_EVENT_LEN);
    memset(e->data, 0, e->len);
    e->data[0] = BDC_PROTO_VER;
    p = e->data + BDC_HDR_LEN;
    memcpy(p, c->mac.a, 6);
    memcpy(p + 6, c->mac.a, 6);
    stw_be_p(p + 12, ETH_P_BRCM);
    stw_be_p(p + 14, BCMILCP_SUBTYPE_VENDOR);
    stw_be_p(p + 16, BCM_EVENT_LEN - 16 + dlen);
    /* Broadcom's OUI */
    p[19] = 0x00;
    p[20] = 0x10;
    p[21] = 0x18;
    stw_be_p(p + 22, BCMILCP_BCM_EVENT);
    stw_be_p(p + 24, WL_EVENT_MSG_VERSION);
    stw_be_p(p + 26, flags);
    stl_be_p(p + 28, type);
    stl_be_p(p + 32, status);
    stl_be_p(p + 36, reason);
    stl_be_p(p + 44, dlen);
    if (addr) {
        memcpy(p + 48, addr, 6);
    }
    if (dlen) {
        g_byte_array_append(e, data, dlen);
    }
    bcm_queue(c, BCM_CHAN_EVENT, e);
}

/* An Ethernet frame for the host, on the data channel */
static void bcm_to_host(BcmCore *c, const uint8_t *da, const uint8_t *sa,
                        uint16_t proto, const uint8_t *payload, size_t len)
{
    uint8_t h[BDC_HDR_LEN + ETH_HLEN] = { BDC_PROTO_VER };
    GByteArray *d;

    if (g_queue_get_length(&c->out) >= OUT_QUEUE_MAX) {
        return;
    }
    memcpy(h + BDC_HDR_LEN, da, 6);
    memcpy(h + BDC_HDR_LEN + 6, sa, 6);
    stw_be_p(h + BDC_HDR_LEN + 12, proto);
    d = g_byte_array_sized_new(sizeof(h) + len);
    g_byte_array_append(d, h, sizeof(h));
    g_byte_array_append(d, payload, len);
    bcm_queue(c, BCM_CHAN_DATA, d);
}

/*
 * The Ethernet frame an 802.11 data frame carries, LLC/SNAP and all: its
 * protocol, and where its payload starts. False for anything else.
 */
static bool bcm_unwrap(uint16_t fc, const uint8_t *f, size_t len,
                       uint16_t *proto, size_t *off)
{
    size_t hdr = WIFI_HDR_LEN;

    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_NODATA) {
        return false;
    }
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        hdr += 2;
    }
    if (len < hdr + 8 ||
        memcmp(f + hdr, WIFI_RFC1042, sizeof(WIFI_RFC1042))) {
        return false;
    }
    *proto = lduw_be_p(f + hdr + 6);
    *off = hdr + 8;
    return true;
}

/* An 802.11 data frame around an Ethernet II one, LLC/SNAP and all */
static size_t bcm_wrap(uint8_t *f, uint16_t fc, const uint8_t *a1,
                       const uint8_t *a2, const uint8_t *a3, uint16_t seq,
                       const uint8_t *eth, size_t len)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, a1, 6);
    memcpy(f + 10, a2, 6);
    memcpy(f + 16, a3, 6);
    stw_le_p(f + 22, (seq & 0xfff) << 4);
    memcpy(f + WIFI_HDR_LEN, WIFI_RFC1042, sizeof(WIFI_RFC1042));
    memcpy(f + WIFI_HDR_LEN + 6, eth + 12, len - 12);
    return WIFI_HDR_LEN + 6 + len - 12;
}

static const uint8_t *bcm_find_ie(const uint8_t *p, size_t len, uint8_t id)
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

/*
 * wl_bss_info_t for the access point as its beacon has it now, into
 * @bi of BSS_INFO_LEN + WIFI_AP_BEACON_MAX bytes
 */
static size_t bcm_bss_info(BcmCore *c, uint8_t *bi)
{
    uint8_t b[WIFI_AP_BEACON_MAX];
    size_t n = wifi_ap_beacon(&c->ap, b);
    const uint8_t *body = b + WIFI_HDR_LEN;
    const uint8_t *ies = body + 12, *ie;
    size_t ielen = n - WIFI_HDR_LEN - 12;
    static const uint8_t rate_ies[] = { 1, 50 };
    unsigned rates = 0;

    memset(bi, 0, BSS_INFO_LEN);
    stl_le_p(bi, BSS_INFO_VERSION);
    stl_le_p(bi + 4, BSS_INFO_LEN + ielen);
    memcpy(bi + 8, b + 16, 6);
    /* the beacon interval and capability, after the timestamp */
    stw_le_p(bi + 14, lduw_le_p(body + 8));
    stw_le_p(bi + 16, lduw_le_p(body + 10));
    ie = bcm_find_ie(ies, ielen, WIFI_EID_SSID);
    if (ie) {
        bi[18] = MIN(ie[1], 32);
        memcpy(bi + 19, ie + 2, bi[18]);
    }
    for (size_t r = 0; r < ARRAY_SIZE(rate_ies); r++) {
        ie = bcm_find_ie(ies, ielen, rate_ies[r]);
        for (unsigned i = 0; ie && i < ie[1] && rates < 16; i++) {
            bi[56 + rates++] = ie[2 + i];
        }
    }
    stl_le_p(bi + 52, rates);
    stw_le_p(bi + 72, CHANSPEC_2G_20(c->ap.channel));
    bi[76] = 1;
    stw_le_p(bi + 78, RSSI_DBM);
    bi[80] = NOISE_DBM;
    bi[88] = c->ap.channel;
    bi[96] = BSS_FROM_BEACON | BSS_RSSI_ONCHANNEL;
    stw_le_p(bi + 116, BSS_INFO_LEN);
    stl_le_p(bi + 120, ielen);
    stw_le_p(bi + 124, RSSI_DBM - NOISE_DBM);
    memcpy(bi + BSS_INFO_LEN, ies, ielen);
    return BSS_INFO_LEN + ielen;
}

static void bcm_scan_done(void *opaque)
{
    BcmCore *c = opaque;
    uint8_t r[ESCAN_RESULT_LEN + BSS_INFO_LEN + WIFI_AP_BEACON_MAX] = {};
    size_t n = bcm_bss_info(c, r + ESCAN_RESULT_LEN);

    stl_le_p(r, ESCAN_RESULT_LEN + n);
    stl_le_p(r + 4, BSS_INFO_VERSION);
    stw_le_p(r + 8, c->scan_sync_id);
    stw_le_p(r + 10, 1);
    bcm_event(c, WLC_E_ESCAN_RESULT, WLC_E_STATUS_PARTIAL, 0, NULL, 0, r,
              ESCAN_RESULT_LEN + n);
    memset(r, 0, ESCAN_RESULT_LEN);
    stl_le_p(r, ESCAN_RESULT_LEN);
    stl_le_p(r + 4, BSS_INFO_VERSION);
    stw_le_p(r + 8, c->scan_sync_id);
    bcm_event(c, WLC_E_ESCAN_RESULT, WLC_E_STATUS_SUCCESS, 0, NULL, 0, r,
              ESCAN_RESULT_LEN);
}

/*
 * wl_escan_params_t: version, action, sync id, then wl_scan_params_t with
 * its channel count at 68 and the chanspecs at 72. The access point is
 * heard when its channel is among them, or none are named.
 */
static int bcm_escan(BcmCore *c, const uint8_t *p, size_t len)
{
    bool heard;
    unsigned nch;

    if (len < 72) {
        return BCME_BADARG;
    }
    if (lduw_le_p(p + 4) == ESCAN_ACTION_ABORT) {
        timer_del(c->scan_timer);
        return BCME_OK;
    }
    c->scan_sync_id = lduw_le_p(p + 6);
    nch = MIN(ldl_le_p(p + 68) & 0xffff, (len - 72) / 2);
    heard = !nch;
    for (unsigned i = 0; i < nch; i++) {
        heard |= (lduw_le_p(p + 72 + 2 * i) & 0xff) == c->ap.channel;
    }
    if (heard) {
        timer_mod(c->scan_timer,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SCAN_MS);
    } else {
        uint8_t r[ESCAN_RESULT_LEN] = {};

        stl_le_p(r, ESCAN_RESULT_LEN);
        stl_le_p(r + 4, BSS_INFO_VERSION);
        stw_le_p(r + 8, c->scan_sync_id);
        bcm_event(c, WLC_E_ESCAN_RESULT, WLC_E_STATUS_SUCCESS, 0, NULL, 0,
                  r, sizeof(r));
    }
    return BCME_OK;
}

/* Abort a scan under way, as WLC_SCAN with no channels asks */
static void bcm_scan_abort(BcmCore *c)
{
    uint8_t r[ESCAN_RESULT_LEN] = {};

    if (!timer_pending(c->scan_timer)) {
        return;
    }
    timer_del(c->scan_timer);
    stl_le_p(r, ESCAN_RESULT_LEN);
    stl_le_p(r + 4, BSS_INFO_VERSION);
    stw_le_p(r + 8, c->scan_sync_id);
    bcm_event(c, WLC_E_ESCAN_RESULT, WLC_E_STATUS_ABORT, 0, NULL, 0, r,
              sizeof(r));
}

static void bcm_set_sta(BcmCore *c, BcmStaState state)
{
    if (c->sta != state) {
        trace_bcm_sta(c->sta, state);
    }
    c->sta = state;
}

/* To the BSS joined, from the dongle */
static size_t bcm_sta_hdr(BcmCore *c, uint8_t *f, uint16_t fc)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, c->bssid.a, 6);
    memcpy(f + 10, c->mac.a, 6);
    memcpy(f + 16, c->bssid.a, 6);
    stw_le_p(f + 22, (c->seq++ & 0xfff) << 4);
    return WIFI_HDR_LEN;
}

/* Open authentication, then association, each sent until answered */
static void bcm_sta_send(BcmCore *c)
{
    uint8_t f[WIFI_HDR_LEN + 4 + BCM_IES_MAX];
    size_t n;
    uint8_t *ies;

    if (c->sta == BCM_STA_AUTHENTICATING) {
        n = bcm_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_AUTH));
        /* open system, transaction 1 */
        stw_le_p(f + n, 0);
        stw_le_p(f + n + 2, 1);
        stw_le_p(f + n + 4, 0);
        n += 6;
    } else {
        n = bcm_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_ASSOC_REQ));
        stw_le_p(f + n, WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE |
                        WIFI_CAP_SHORT_SLOT |
                        (c->wsec ? WIFI_CAP_PRIVACY : 0));
        stw_le_p(f + n + 2, LISTEN_INTERVAL);
        n += 4;
        ies = f + n;
        f[n++] = WIFI_EID_SSID;
        f[n++] = c->ssid_len;
        memcpy(f + n, c->ssid, c->ssid_len);
        n += c->ssid_len;
        memcpy(f + n, WIFI_RATES_IE, sizeof(WIFI_RATES_IE));
        n += sizeof(WIFI_RATES_IE);
        memcpy(f + n, WIFI_XRATES_IE, sizeof(WIFI_XRATES_IE));
        n += sizeof(WIFI_XRATES_IE);
        memcpy(f + n, c->wpaie, c->wpaie_len);
        n += c->wpaie_len;
        c->req_ies_len = f + n - ies;
        memcpy(c->req_ies, ies, c->req_ies_len);
    }
    timer_mod(c->join_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                             RETRY_MS);
    wifi_ap_receive(&c->ap, f, n);
}

/* The join failed: say so as the firmware does, with the last status */
static void bcm_join_fail(BcmCore *c, uint32_t event, uint16_t code)
{
    timer_del(c->join_timer);
    bcm_set_sta(c, BCM_STA_IDLE);
    if (event != WLC_E_SET_SSID) {
        bcm_event(c, event, WLC_E_STATUS_FAIL, code, c->bssid.a, 0, NULL, 0);
    }
    bcm_event(c, WLC_E_SET_SSID, event == WLC_E_SET_SSID ?
              WLC_E_STATUS_NO_NETWORKS : WLC_E_STATUS_FAIL, 0, NULL, 0,
              NULL, 0);
}

/*
 * Start joining the BSS named by a wlc_ssid_t, and by a BSSID unless
 * that is broadcast; the access point is the only one there is
 */
static int bcm_join(BcmCore *c, const uint8_t *ssid, size_t len,
                    const uint8_t *bssid)
{
    uint32_t n;

    if (len < 36) {
        return BCME_BADARG;
    }
    n = MIN(ldl_le_p(ssid), 32);
    timer_del(c->scan_timer);
    c->ssid_len = n;
    memcpy(c->ssid, ssid + 4, n);
    if (bssid && !is_broadcast_ether_addr(bssid)) {
        memcpy(c->bssid.a, bssid, 6);
    } else {
        memcpy(c->bssid.a, c->ap.bssid.a, 6);
    }
    c->tries = 0;
    bcm_set_sta(c, BCM_STA_JOINING);
    timer_mod(c->join_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                             JOIN_MS);
    return BCME_OK;
}

/* A join under way: start it, or send again what went unanswered */
static void bcm_join_tick(void *opaque)
{
    BcmCore *c = opaque;
    size_t n = strlen(c->ap.ssid);

    switch (c->sta) {
    case BCM_STA_JOINING:
        if (memcmp(c->bssid.a, c->ap.bssid.a, 6) || c->ssid_len != n ||
            memcmp(c->ssid, c->ap.ssid, n)) {
            bcm_join_fail(c, WLC_E_SET_SSID, 0);
            return;
        }
        bcm_set_sta(c, BCM_STA_AUTHENTICATING);
        bcm_sta_send(c);
        break;
    case BCM_STA_AUTHENTICATING:
    case BCM_STA_ASSOCIATING:
        if (++c->tries >= TRIES) {
            bcm_join_fail(c, c->sta == BCM_STA_AUTHENTICATING ?
                          WLC_E_AUTH : WLC_E_ASSOC, 0);
            return;
        }
        bcm_sta_send(c);
        break;
    default:
        break;
    }
}

/* The BSS is gone, by the host's wish or the access point's */
static void bcm_sta_leave(BcmCore *c, bool tell_ap, uint32_t event,
                          uint16_t reason)
{
    uint8_t f[WIFI_HDR_LEN + 2];
    size_t n;

    if (c->sta != BCM_STA_ASSOCIATED) {
        bcm_set_sta(c, BCM_STA_IDLE);
        return;
    }
    if (tell_ap) {
        n = bcm_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_DISASSOC));
        stw_le_p(f + n, reason);
        wifi_ap_receive(&c->ap, f, n + 2);
    }
    bcm_set_sta(c, BCM_STA_IDLE);
    c->ssid_len = 0;
    bcm_event(c, event, WLC_E_STATUS_SUCCESS, reason, c->bssid.a, 0, NULL,
              0);
    bcm_event(c, WLC_E_LINK, WLC_E_STATUS_SUCCESS, reason, c->bssid.a, 0,
              NULL, 0);
}

static void bcm_sta_mgmt(BcmCore *c, uint16_t fc, const uint8_t *f,
                         size_t len)
{
    const uint8_t *body = f + WIFI_HDR_LEN;
    size_t blen = len - WIFI_HDR_LEN;

    switch (WIFI_FC_STYPE(fc)) {
    case WIFI_STYPE_AUTH:
        /* algorithm, transaction, status */
        if (c->sta != BCM_STA_AUTHENTICATING || blen < 6 ||
            lduw_le_p(body + 2) != 2) {
            return;
        }
        if (lduw_le_p(body + 4)) {
            bcm_join_fail(c, WLC_E_AUTH, lduw_le_p(body + 4));
            return;
        }
        bcm_set_sta(c, BCM_STA_ASSOCIATING);
        c->tries = 0;
        bcm_sta_send(c);
        return;
    case WIFI_STYPE_ASSOC_RESP:
    case WIFI_STYPE_REASSOC_RESP:
        /* capability, status, association id */
        if (c->sta != BCM_STA_ASSOCIATING || blen < 6) {
            return;
        }
        if (lduw_le_p(body + 2)) {
            bcm_join_fail(c, WLC_E_ASSOC, lduw_le_p(body + 2));
            return;
        }
        timer_del(c->join_timer);
        c->resp_ies_len = MIN(blen - 6, sizeof(c->resp_ies));
        memcpy(c->resp_ies, body + 6, c->resp_ies_len);
        bcm_set_sta(c, BCM_STA_ASSOCIATED);
        bcm_event(c, WLC_E_AUTH, WLC_E_STATUS_SUCCESS, 0, c->bssid.a, 0,
                  NULL, 0);
        bcm_event(c, WLC_E_ASSOC, WLC_E_STATUS_SUCCESS, 0, c->bssid.a, 0,
                  NULL, 0);
        bcm_event(c, WLC_E_LINK, WLC_E_STATUS_SUCCESS, 0, c->bssid.a,
                  WLC_EVENT_MSG_LINK, NULL, 0);
        bcm_event(c, WLC_E_SET_SSID, WLC_E_STATUS_SUCCESS, 0, c->bssid.a,
                  0, NULL, 0);
        return;
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (blen >= 2) {
            bcm_sta_leave(c, false,
                          WIFI_FC_STYPE(fc) == WIFI_STYPE_DEAUTH ?
                          WLC_E_DEAUTH_IND : WLC_E_DISASSOC_IND,
                          lduw_le_p(body));
        }
        return;
    }
}

/* A frame from the access point, its cipher taken off */
static void bcm_ap_deliver(void *opaque, const uint8_t *f, size_t len,
                           bool protected)
{
    BcmCore *c = opaque;
    uint16_t fc, proto;
    size_t off;

    if (len < WIFI_HDR_LEN || c->sta < BCM_STA_AUTHENTICATING ||
        memcmp(f + 10, c->bssid.a, 6) ||
        (!is_multicast_ether_addr(f + 4) && memcmp(f + 4, c->mac.a, 6))) {
        return;
    }
    fc = lduw_le_p(f);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        bcm_sta_mgmt(c, fc, f, len);
        break;
    case WIFI_TYPE_DATA:
        /* from the distribution system: destination 1, source 3 */
        if (c->sta == BCM_STA_ASSOCIATED && (fc & WIFI_FC_FROMDS) &&
            !(fc & WIFI_FC_TODS) && bcm_unwrap(fc, f, len, &proto, &off)) {
            bcm_to_host(c, f + 4, f + 16, proto, f + off, len - off);
        }
        break;
    }
}

/* From the dongle's access point to the client */
static size_t bcm_bss_hdr(BcmCore *c, uint8_t *f, uint16_t fc,
                          const uint8_t *da)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, da, 6);
    memcpy(f + 10, c->mac.a, 6);
    memcpy(f + 16, c->mac.a, 6);
    stw_le_p(f + 22, (c->seq++ & 0xfff) << 4);
    return WIFI_HDR_LEN;
}

/* Tell the client whether there is an access point to join */
static void bcm_bss_update(BcmCore *c)
{
    if (!c->bss_up) {
        c->peer_assoc = false;
    }
    if (!c->client_nic) {
        return;
    }
    if (c->bss_up) {
        wifi_client_bss(&c->client, c->mac.a, c->ap_ssid, c->ap_ssid_len,
                        NULL);
    } else {
        wifi_client_bss(&c->client, NULL, NULL, 0, NULL);
    }
}

static void bcm_set_bss(BcmCore *c, bool up)
{
    if (c->bss_up != up) {
        trace_bcm_bss(up);
        c->bss_up = up;
        bcm_bss_update(c);
    }
}

/* What the firmware's own access point answers a station with */
static void bcm_peer_mgmt(BcmCore *c, uint16_t fc, const uint8_t *f,
                          size_t len)
{
    const uint8_t *sa = f + 10, *body = f + WIFI_HDR_LEN;
    size_t blen = len - WIFI_HDR_LEN;
    uint8_t r[WIFI_HDR_LEN + 6 + sizeof(WIFI_RATES_IE) +
              sizeof(WIFI_XRATES_IE)];
    size_t n;

    switch (WIFI_FC_STYPE(fc)) {
    case WIFI_STYPE_AUTH:
        if (blen < 6 || lduw_le_p(body) || lduw_le_p(body + 2) != 1) {
            return;
        }
        memcpy(c->peer.a, sa, 6);
        c->peer_assoc = false;
        n = bcm_bss_hdr(c, r, WIFI_FC_MGMT(WIFI_STYPE_AUTH), sa);
        stw_le_p(r + n, 0);
        stw_le_p(r + n + 2, 2);
        stw_le_p(r + n + 4, 0);
        bcm_event(c, WLC_E_AUTH_IND, WLC_E_STATUS_SUCCESS, 0, sa, 0, NULL,
                  0);
        wifi_client_receive(&c->client, r, n + 6);
        return;
    case WIFI_STYPE_ASSOC_REQ:
        /* capability and listen interval, then the IEs the host wants */
        if (blen < 4 || memcmp(sa, c->peer.a, 6)) {
            return;
        }
        c->peer_ies_len = MIN(blen - 4, sizeof(c->peer_ies));
        memcpy(c->peer_ies, body + 4, c->peer_ies_len);
        c->peer_assoc = true;
        n = bcm_bss_hdr(c, r, WIFI_FC_MGMT(WIFI_STYPE_ASSOC_RESP), sa);
        stw_le_p(r + n, WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE |
                        WIFI_CAP_SHORT_SLOT);
        stw_le_p(r + n + 2, 0);
        stw_le_p(r + n + 4, PEER_AID);
        n += 6;
        memcpy(r + n, WIFI_RATES_IE, sizeof(WIFI_RATES_IE));
        n += sizeof(WIFI_RATES_IE);
        memcpy(r + n, WIFI_XRATES_IE, sizeof(WIFI_XRATES_IE));
        n += sizeof(WIFI_XRATES_IE);
        trace_bcm_peer(true);
        bcm_event(c, WLC_E_ASSOC_IND, WLC_E_STATUS_SUCCESS, 0, sa, 0,
                  c->peer_ies, c->peer_ies_len);
        wifi_client_receive(&c->client, r, n);
        return;
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (!c->peer_assoc || memcmp(sa, c->peer.a, 6) || blen < 2) {
            return;
        }
        c->peer_assoc = false;
        trace_bcm_peer(false);
        bcm_event(c, WIFI_FC_STYPE(fc) == WIFI_STYPE_DEAUTH ?
                  WLC_E_DEAUTH_IND : WLC_E_DISASSOC_IND,
                  WLC_E_STATUS_SUCCESS, lduw_le_p(body), sa, 0, NULL, 0);
        return;
    }
}

/* A frame from the client, to the dongle's access point */
static void bcm_client_deliver(void *opaque, const uint8_t *f, size_t len,
                               bool protected)
{
    BcmCore *c = opaque;
    uint16_t fc, proto;
    size_t off;

    if (!c->bss_up || len < WIFI_HDR_LEN || memcmp(f + 4, c->mac.a, 6)) {
        return;
    }
    fc = lduw_le_p(f);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        bcm_peer_mgmt(c, fc, f, len);
        break;
    case WIFI_TYPE_DATA:
        /* to the distribution system: destination 3, source 2 */
        if (c->peer_assoc && !memcmp(f + 10, c->peer.a, 6) &&
            (fc & WIFI_FC_TODS) && !(fc & WIFI_FC_FROMDS) &&
            bcm_unwrap(fc, f, len, &proto, &off)) {
            bcm_to_host(c, f + 16, f + 10, proto, f + off, len - off);
        }
        break;
    }
}

/* An Ethernet frame from the host, for whichever BSS the dongle is in */
static void bcm_host_data(BcmCore *c, const uint8_t *buf, size_t len)
{
    g_autofree uint8_t *f = NULL;
    const uint8_t *eth;
    size_t off, n;

    if (len < BDC_HDR_LEN) {
        return;
    }
    off = BDC_HDR_LEN + 4 * buf[3];
    if (len < off + ETH_HLEN || lduw_be_p(buf + off + 12) < 0x0600) {
        return;
    }
    eth = buf + off;
    len -= off;
    f = g_malloc(WIFI_HDR_LEN + 6 + len);
    if (c->bss_up) {
        if (!c->peer_assoc || (!is_multicast_ether_addr(eth) &&
                               memcmp(eth, c->peer.a, 6))) {
            return;
        }
        n = bcm_wrap(f, WIFI_FC_DATA | WIFI_FC_FROMDS, eth, c->mac.a,
                     eth + 6, c->seq++, eth, len);
        wifi_client_receive(&c->client, f, n);
    } else if (c->sta == BCM_STA_ASSOCIATED) {
        n = bcm_wrap(f, WIFI_FC_DATA | WIFI_FC_TODS, c->bssid.a, c->mac.a,
                     eth, c->seq++, eth, len);
        wifi_ap_receive(&c->ap, f, n);
    }
}

static int bcm_get_int(uint8_t *buf, size_t len, uint32_t val)
{
    if (len < 4) {
        return BCME_BADARG;
    }
    stl_le_p(buf, val);
    return BCME_OK;
}

static int bcm_set_int(const uint8_t *buf, size_t len, uint32_t *val)
{
    if (len < 4) {
        return BCME_BADARG;
    }
    *val = ldl_le_p(buf);
    return BCME_OK;
}

/* the iovars that only tune what the model leaves out: taken, ignored */
static const char *const bcm_iovars_ignored[] = {
    "mpc", "assoc_listen", "roam_off", "fullroamperiod", "bcn_timeout",
    "assoc_retry_max", "buf_key_b4_m4", "bus:txglomalign", "arpoe",
    "arp_ol", "bw_cap", "ap_isolate", "closednet", "pmkid_info",
    "tdls_enable", "tdls_auto_op", "vndr_ie", "wsec_key", "country",
    "mcast_list", "allmulti",
};

/*
 * An iovar: @val holds what a SET brings, or what a GET names after the
 * name; a GET's answer goes over the start of @buf, @len long
 */
static int bcm_iovar(BcmCore *c, const char *name, bool set,
                     const uint8_t *val, size_t vlen, uint8_t *buf,
                     size_t len)
{
    static const char ver[] = "wl0: QEMU model of the BCM4343x firmware "
                              "version 7.45.96\n";
    static const char cap[] = "ap sta wme 11n 11d 11h";
    uint32_t v;

    if (!strcmp(name, "cur_etheraddr")) {
        if (set) {
            if (vlen < 6) {
                return BCME_BADARG;
            }
            memcpy(c->mac.a, val, 6);
        } else if (len >= 6) {
            memcpy(buf, c->mac.a, 6);
        }
        return BCME_OK;
    }
    if (!strcmp(name, "event_msgs")) {
        if (set) {
            memcpy(c->event_mask, val, MIN(vlen, BCM_EVENT_MASK_LEN));
        } else {
            memcpy(buf, c->event_mask, MIN(len, BCM_EVENT_MASK_LEN));
        }
        return BCME_OK;
    }
    if (!strcmp(name, "wsec")) {
        return set ? bcm_set_int(val, vlen, &c->wsec)
                   : bcm_get_int(buf, len, c->wsec);
    }
    if (!strcmp(name, "wpa_auth")) {
        return set ? bcm_set_int(val, vlen, &c->wpa_auth)
                   : bcm_get_int(buf, len, c->wpa_auth);
    }
    if (!strcmp(name, "auth")) {
        return set ? bcm_set_int(val, vlen, &c->auth)
                   : bcm_get_int(buf, len, c->auth);
    }
    if (!strcmp(name, "chanspec")) {
        return set ? bcm_set_int(val, vlen, &c->chanspec)
                   : bcm_get_int(buf, len, c->chanspec);
    }
    if (!strcmp(name, "wpaie") && set) {
        c->wpaie_len = MIN(vlen, sizeof(c->wpaie));
        memcpy(c->wpaie, val, c->wpaie_len);
        return BCME_OK;
    }
    if (!strcmp(name, "escan") && set) {
        return bcm_escan(c, val, vlen);
    }
    if (!strcmp(name, "join") && set) {
        /* wl_extjoin_params_t: the SSID, scan parameters, BSSID at 56 */
        return bcm_join(c, val, vlen, vlen >= 62 ? val + 56 : NULL);
    }
    if (!strcmp(name, "bss")) {
        /* the bsscfg index, then on a SET whether it is to be up */
        if (set) {
            if (vlen < 8) {
                return BCME_BADARG;
            }
            bcm_set_bss(c, ldl_le_p(val + 4) && c->ap_mode &&
                        c->ap_ssid_len);
            return BCME_OK;
        }
        return bcm_get_int(buf, len, c->bss_up);
    }
    if (set) {
        for (size_t i = 0; i < ARRAY_SIZE(bcm_iovars_ignored); i++) {
            if (!strcmp(name, bcm_iovars_ignored[i])) {
                return BCME_OK;
            }
        }
        return BCME_UNSUPPORTED;
    }
    if (!strcmp(name, "ver") || !strcmp(name, "cap")) {
        pstrcpy((char *)buf, len, name[0] == 'v' ? ver : cap);
        return BCME_OK;
    }
    if (!strcmp(name, "country")) {
        /* wl_country_t: abbreviation, revision, code */
        if (len < 12) {
            return BCME_BADARG;
        }
        memset(buf, 0, 12);
        memcpy(buf, "US", 2);
        memcpy(buf + 8, "US", 2);
        return BCME_OK;
    }
    if (!strcmp(name, "nmode") || !strcmp(name, "mimo_bw_cap") ||
        !strcmp(name, "per_chan_info")) {
        return bcm_get_int(buf, len, 0);
    }
    if (!strcmp(name, "chanspecs")) {
        if (len < 4 * (1 + CHANNELS_2G)) {
            return BCME_BADARG;
        }
        stl_le_p(buf, CHANNELS_2G);
        for (v = 1; v <= CHANNELS_2G; v++) {
            stl_le_p(buf + 4 * v, CHANSPEC_2G_20(v));
        }
        return BCME_OK;
    }
    if (!strcmp(name, "assoc_info")) {
        /* wl_assoc_info_t: request and response lengths, then fields */
        if (len < 28) {
            return BCME_BADARG;
        }
        memset(buf, 0, 28);
        stl_le_p(buf, 4 + c->req_ies_len);
        stl_le_p(buf + 4, 6 + c->resp_ies_len);
        stw_le_p(buf + 12, WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE |
                           WIFI_CAP_SHORT_SLOT |
                           (c->wsec ? WIFI_CAP_PRIVACY : 0));
        stw_le_p(buf + 14, LISTEN_INTERVAL);
        stw_le_p(buf + 26, PEER_AID);
        return BCME_OK;
    }
    if (!strcmp(name, "assoc_req_ies")) {
        memcpy(buf, c->req_ies, MIN(len, c->req_ies_len));
        return BCME_OK;
    }
    if (!strcmp(name, "assoc_resp_ies")) {
        memcpy(buf, c->resp_ies, MIN(len, c->resp_ies_len));
        return BCME_OK;
    }
    if (!strcmp(name, "sta_info")) {
        /* sta_info_t: version, length, capability, flags, idle, address */
        if (vlen < 6 || len < 48 || !c->peer_assoc ||
            memcmp(val, c->peer.a, 6)) {
            return BCME_NOTFOUND;
        }
        memset(buf, 0, 48);
        stw_le_p(buf, 4);
        stw_le_p(buf + 2, 48);
        stw_le_p(buf + 4, WIFI_CAP_ESS);
        stl_le_p(buf + 8, WL_STA_AUTHE | WL_STA_ASSOC | WL_STA_AUTHO);
        memcpy(buf + 16, c->peer.a, 6);
        return BCME_OK;
    }
    return BCME_UNSUPPORTED;
}

/* WLC_GET_VAR and WLC_SET_VAR: "name\0", then a value or parameters */
static int bcm_iovar_cmd(BcmCore *c, bool set, uint8_t *buf, size_t len)
{
    size_t n = strnlen((char *)buf, len);
    g_autofree char *name = g_strndup((char *)buf, n);
    g_autofree uint8_t *val = NULL;
    int err;

    n = MIN(n + 1, len);
    val = g_memdup2(buf + n, len - n);
    err = bcm_iovar(c, name, set, val, len - n, buf, len);
    trace_bcm_iovar(name, set, err);
    return err;
}

static int bcm_ioctl(BcmCore *c, uint32_t cmd, bool set, uint8_t *buf,
                     size_t len)
{
    uint32_t v;

    switch (cmd) {
    case WLC_GET_VAR:
    case WLC_SET_VAR:
        return bcm_iovar_cmd(c, cmd == WLC_SET_VAR, buf, len);
    case WLC_GET_VERSION:
        return bcm_get_int(buf, len, WLC_IOCTL_VERSION);
    case WLC_DOWN:
        bcm_set_bss(c, false);
        return BCME_OK;
    case WLC_GET_RATE:
        return bcm_get_int(buf, len, RATE_54M);
    case WLC_GET_BSSID:
        if (c->bss_up) {
            memcpy(buf, c->mac.a, MIN(len, 6));
            return BCME_OK;
        }
        if (c->sta != BCM_STA_ASSOCIATED) {
            return BCME_NOTASSOCIATED;
        }
        memcpy(buf, c->bssid.a, MIN(len, 6));
        return BCME_OK;
    case WLC_GET_SSID:
        /* wlc_ssid_t: the length, then 32 bytes */
        if (len < 36) {
            return BCME_BADARG;
        }
        memset(buf, 0, 36);
        if (c->bss_up) {
            stl_le_p(buf, c->ap_ssid_len);
            memcpy(buf + 4, c->ap_ssid, c->ap_ssid_len);
        } else if (c->sta == BCM_STA_ASSOCIATED) {
            stl_le_p(buf, c->ssid_len);
            memcpy(buf + 4, c->ssid, c->ssid_len);
        }
        return BCME_OK;
    case WLC_SET_SSID:
        if (!c->ap_mode) {
            /* wl_join_params_t: the SSID, then the BSSID at 36 */
            return bcm_join(c, buf, len, len >= 42 ? buf + 36 : NULL);
        }
        if (len < 36) {
            return BCME_BADARG;
        }
        c->ap_ssid_len = MIN(ldl_le_p(buf), 32);
        memcpy(c->ap_ssid, buf + 4, c->ap_ssid_len);
        bcm_set_bss(c, false);
        bcm_set_bss(c, c->ap_ssid_len);
        return BCME_OK;
    case WLC_GET_CHANNEL:
        /* channel_info_t: hardware, target and scan channels */
        if (len < 12) {
            return BCME_BADARG;
        }
        v = c->bss_up ? c->chanspec & 0xff : c->ap.channel;
        stl_le_p(buf, v);
        stl_le_p(buf + 4, v);
        stl_le_p(buf + 8, 0);
        return BCME_OK;
    case WLC_SET_CHANNEL:
        if (len >= 4) {
            c->chanspec = CHANSPEC_2G_20(ldl_le_p(buf) & 0xff);
        }
        return BCME_OK;
    case WLC_SCAN:
        bcm_scan_abort(c);
        return BCME_OK;
    case WLC_DISASSOC:
        /* scb_val_t: the reason, then an address */
        bcm_sta_leave(c, true, WLC_E_DISASSOC, len >= 4 ? ldl_le_p(buf) : 0);
        return BCME_OK;
    case WLC_GET_DTIMPRD:
        return bcm_get_int(buf, len, 1);
    case WLC_GET_PM:
    case WLC_GET_BAND:
        return bcm_get_int(buf, len, 0);
    case WLC_GET_AP:
        return bcm_get_int(buf, len, c->ap_mode);
    case WLC_SET_AP:
        if (len < 4) {
            return BCME_BADARG;
        }
        c->ap_mode = ldl_le_p(buf);
        if (!c->ap_mode) {
            bcm_set_bss(c, false);
        }
        return BCME_OK;
    case WLC_GET_RSSI:
        return bcm_get_int(buf, len, RSSI_DBM);
    case WLC_GET_BSS_INFO:
        /* the buffer's length, then wl_bss_info_t */
        if (len < 4 + BSS_INFO_LEN + WIFI_AP_BEACON_MAX) {
            return BCME_BADARG;
        }
        if (c->sta != BCM_STA_ASSOCIATED) {
            return BCME_NOTASSOCIATED;
        }
        bcm_bss_info(c, buf + 4);
        return BCME_OK;
    case WLC_GET_REVINFO:
    case WLC_GET_PKTCNTS:
        memset(buf, 0, len);
        return BCME_OK;
    case WLC_GET_BANDLIST:
        /* the count, then the bands */
        if (len < 8) {
            return BCME_BADARG;
        }
        stl_le_p(buf, 1);
        stl_le_p(buf + 4, WLC_BAND_2G);
        return BCME_OK;
    case WLC_GET_ASSOCLIST:
        /* the count, then the addresses */
        if (len < 4 + 6) {
            return BCME_BADARG;
        }
        stl_le_p(buf, c->peer_assoc);
        memcpy(buf + 4, c->peer.a, 6);
        return BCME_OK;
    case WLC_SCB_DEAUTHENTICATE_FOR_REASON:
        if (len >= 10 && c->peer_assoc && !memcmp(buf + 4, c->peer.a, 6)) {
            uint8_t f[WIFI_HDR_LEN + 2];
            size_t hn = bcm_bss_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_DEAUTH),
                                    c->peer.a);

            stw_le_p(f + hn, ldl_le_p(buf));
            c->peer_assoc = false;
            trace_bcm_peer(false);
            wifi_client_receive(&c->client, f, hn + 2);
        }
        return BCME_OK;
    case WLC_GET_VALID_CHANNELS:
        if (len < 4 * (1 + CHANNELS_2G)) {
            return BCME_BADARG;
        }
        stl_le_p(buf, CHANNELS_2G);
        for (v = 1; v <= CHANNELS_2G; v++) {
            stl_le_p(buf + 4 * v, v);
        }
        return BCME_OK;
    case WLC_UP:
    case WLC_SET_PROMISC:
    case WLC_SET_INFRA:
    case WLC_SET_PASSIVE_SCAN:
    case WLC_SET_ROAM_TRIGGER:
    case WLC_SET_ROAM_DELTA:
    case WLC_SET_ROAM_SCAN_PERIOD:
    case WLC_SET_BCNPRD:
    case WLC_SET_DTIMPRD:
    case WLC_SET_PM:
    case WLC_SCB_AUTHORIZE:
    case WLC_SCB_DEAUTHORIZE:
    case WLC_SET_BAND:
    case WLC_SET_SCAN_CHANNEL_TIME:
    case WLC_SET_SCAN_UNASSOC_TIME:
    case WLC_SET_KEY_PRIMARY:
    case WLC_SET_SCAN_PASSIVE_TIME:
        return BCME_OK;
    }
    return BCME_UNSUPPORTED;
}

/*
 * A BCDC ioctl: the header, then as much of the buffer as the host sent.
 * The answer is the whole buffer, ahead of anything the ioctl sets off.
 */
static void bcm_ioctl_msg(BcmCore *c, const uint8_t *msg, size_t len)
{
    uint32_t cmd, blen, flags;
    GByteArray *r;
    int err;

    if (len < CDC_HDR_LEN) {
        return;
    }
    cmd = ldl_le_p(msg);
    blen = MIN(ldl_le_p(msg + 4), CDC_BUF_MAX);
    flags = ldl_le_p(msg + 8);
    r = g_byte_array_sized_new(CDC_HDR_LEN + blen);
    g_byte_array_set_size(r, CDC_HDR_LEN + blen);
    memset(r->data, 0, r->len);
    memcpy(r->data + CDC_HDR_LEN, msg + CDC_HDR_LEN,
           MIN(blen, len - CDC_HDR_LEN));
    bcm_queue(c, BCM_CHAN_CONTROL, r);
    err = bcm_ioctl(c, cmd, flags & CDCF_IOC_SET, r->data + CDC_HDR_LEN,
                    blen);
    trace_bcm_ioctl(cmd, !!(flags & CDCF_IOC_SET), blen, err);
    stl_le_p(r->data, cmd);
    stl_le_p(r->data + 4, blen);
    stl_le_p(r->data + 8, (flags & ~CDCF_IOC_ERROR) |
                          (err ? CDCF_IOC_ERROR : 0));
    stl_le_p(r->data + 12, err);
}

void bcm_core_host(BcmCore *c, unsigned chan, const uint8_t *buf,
                   size_t len)
{
    switch (chan) {
    case BCM_CHAN_CONTROL:
        bcm_ioctl_msg(c, buf, len);
        break;
    case BCM_CHAN_DATA:
        bcm_host_data(c, buf, len);
        break;
    }
}

void bcm_core_reset(BcmCore *c)
{
    timer_del(c->scan_timer);
    timer_del(c->join_timer);
    bcm_flush(c);
    c->mac = c->conf.macaddr;
    memset(c->event_mask, 0, sizeof(c->event_mask));
    c->wsec = 0;
    c->wpa_auth = 0;
    c->auth = 0;
    c->chanspec = CHANSPEC_2G_20(1);
    c->wpaie_len = 0;
    c->sta = BCM_STA_IDLE;
    c->ssid_len = 0;
    c->req_ies_len = 0;
    c->resp_ies_len = 0;
    c->ap_mode = false;
    c->ap_ssid_len = 0;
    c->peer_assoc = false;
    c->bss_up = false;
    bcm_bss_update(c);
}

static bool bcm_can_receive(NetClientState *nc)
{
    BcmCore *c = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&c->out) < OUT_QUEUE_MAX;
}

static ssize_t bcm_receive(NetClientState *nc, const uint8_t *buf,
                           size_t size)
{
    BcmCore *c = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&c->ap, buf, size);
    return size;
}

static NetClientInfo bcm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = bcm_can_receive,
    .receive = bcm_receive,
};

static bool bcm_client_can_receive(NetClientState *nc)
{
    BcmCore *c = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&c->client) &&
           g_queue_get_length(&c->out) < OUT_QUEUE_MAX;
}

static ssize_t bcm_client_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    BcmCore *c = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&c->client, buf, size);
    return size;
}

static NetClientInfo bcm_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = bcm_client_can_receive,
    .receive = bcm_client_receive,
};

void bcm_core_init(BcmCore *c)
{
    c->scan_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, bcm_scan_done, c);
    c->join_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, bcm_join_tick, c);
    g_queue_init(&c->out);
}

bool bcm_core_realize(BcmCore *c, Error **errp)
{
    DeviceState *dev = c->dev;

    qemu_macaddr_default_if_unset(&c->conf.macaddr);
    c->nic = qemu_new_nic(&bcm_net_info, &c->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, c);
    if (!wifi_ap_realize(&c->ap, c->nic, bcm_ap_deliver, c, errp)) {
        qemu_del_nic(c->nic);
        c->nic = NULL;
        return false;
    }
    if (c->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&c->client_conf.macaddr);
        c->client_nic = qemu_new_nic(&bcm_client_net_info, &c->client_conf,
                                     object_get_typename(OBJECT(dev)), NULL,
                                     &dev->mem_reentrancy_guard, c);
        wifi_client_init(&c->client, c->client_nic, &c->client_conf.macaddr,
                         bcm_client_deliver, c);
    }
    bcm_core_reset(c);
    return true;
}

void bcm_core_unrealize(BcmCore *c)
{
    timer_del(c->scan_timer);
    timer_del(c->join_timer);
    if (c->client_nic) {
        wifi_client_cleanup(&c->client);
        qemu_del_nic(c->client_nic);
        c->client_nic = NULL;
    }
    wifi_ap_unrealize(&c->ap);
    qemu_del_nic(c->nic);
}

void bcm_core_finalize(BcmCore *c)
{
    timer_free(c->scan_timer);
    timer_free(c->join_timer);
    bcm_flush(c);
}
