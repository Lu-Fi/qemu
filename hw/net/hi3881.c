/*
 * HiSilicon Hi3881 Wi-Fi on SDIO
 *
 * The chip as its vendor driver (hi3881) sees it. Function 1 has a FIFO,
 * an interrupt status, a transfer count, message registers and an extend
 * area the driver reads in one go. Before the firmware runs, the FIFO
 * carries a text protocol (VERSION, FILES, CONFIG, QUIT) that downloads
 * it; the firmware is taken and dropped. Then it carries HCC packets: the
 * host's MAC (hmac) talks to the chip's (dmac), which this model plays,
 * with an access point (wifi-ap.c) on the air and a client (wifi-client.c)
 * for the driver's own.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/sd/sdio.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "wifi-frame.h"
#include "wifi-rsn.h"
#include "trace.h"

#define TYPE_HI3881 "hi3881"
OBJECT_DECLARE_SIMPLE_TYPE(Hi3881State, HI3881)

/* function 1 */
#define REG_FIFO                0x00
#define REG_INT_STATUS          0x08
#define REG_INT_ENABLE          0x09
#define REG_XFER_COUNT          0x0c
#define REG_WRITE_MSG           0x24
#define REG_MSG_FROM_DEV        0x28
#define REG_MSG_ACK             0x2b
/* int_stat, msg_stat, xfer_count, credit_info, then a length per buffer */
#define REG_EXTEND              0x30
#define EXTEND_BUFS             64
#define EXTEND_LEN              (16 + EXTEND_BUFS)
#define INT_DREADY              BIT(0)
#define INT_MFARM               BIT(2)
#define BLOCK_SIZE              512

/* device to host messages */
#define D2H_WLAN_READY          0

/*
 * Free buffers the extend area reports: short, large, management and
 * descriptors; the host holds data back while fewer than 12 descriptors
 * are free.
 */
#define CREDIT_INFO             (32 | 32 << 8 | 8 << 16 | 32 << 24)

/*
 * HCC: a 64-byte header area (the 4-byte header, padding, the extend
 * header and what the event adds), then the payload. A host transfer is
 * a 64-byte descriptor, then the packets the previous descriptor named
 * (a length / 8 each; none named: one packet); a transfer of only the
 * descriptor resets that.
 */
#define HCC_HDR_TOTAL           64
#define HCC_DESCR_LEN           64
#define HCC_PAD_HDR(w0)         extract32(w0, 7, 6)
#define HCC_PAY_LEN(w1)         extract32(w1, 4, 12)
#define HCC_EXT_LEN             4
#define D2H_ALIGN               32
#define HCC_SUB_DATA            1

/*
 * After the extend header, a frame's packet has a 20-byte control block
 * and 36 bytes for its MAC header, which starts the room; its body is the
 * payload. The host skips the same 56 bytes in every packet it gets.
 */
#define HCC_CB                  (4 + HCC_EXT_LEN)
#define CB_LEN                  20
#define MAC_HDR_ROOM            36

/* frw events and the sub-types this model takes or gives */
#define EV_HOST_CRX             1
#define EV_HOST_DRX             2
#define EV_DMAC_TO_HMAC_CFG     4
#define EV_WLAN_CRX             5
#define EV_WLAN_DRX             6
#define EV_WLAN_CTX             7
#define HOST_CREATE_CFG_VAP     1
#define SYN_CFG                 2
#define CTX_ADD_USER            1
#define CTX_SCAN_REQ            6
#define CTX_MGMT                8
#define CRX_INIT                0
#define CRX_RX                  1
#define CRX_SCAN_RESULT         3
#define CRX_SCAN_COMP           4
#define DRX_RX_AP               0
#define DRX_RX_STA              1

/* mac_scan_req_stru: the channel list, its length and the cookie */
#define SCAN_REQ_LEN            160
#define SCAN_REQ_CHANNEL        91
#define SCAN_REQ_CHANNELS       14
#define SCAN_REQ_NUMS           148
#define SCAN_REQ_COOKIE         152
#define SCAN_MS                 200

/*
 * A configuration message is its cfgid, its length and its body. The
 * dmac keeps what a VAP's beacon needs and answers hmac's station query
 * with a dmac_query_station_info_response_event; it reports the fate of
 * a management frame wpa_supplicant sent.
 */
#define CFGID_STATION_ID        0x000
#define CFGID_SSID              0x005
#define CFGID_CHANNEL           0x052
#define CFGID_ADD_VAP           0x064
#define CFGID_START_VAP         0x065
#define CFGID_DEL_VAP           0x066
#define CFGID_DOWN_VAP          0x067
#define CFGID_CONFIG_BEACON     0x127
#define CFGID_QUERY_STATION     0x1e3
#define CFGID_MGMT_TX_STATUS    0x1f3
#define STATION_INFO_LEN        44
#define QUERY_STATION_INFO      1
#define TX_SUCC                 1
#define TX_FAIL                 2

/* hmac's VAPs: its configuration VAP, then wlan0 and ap0 */
#define VAPS                    4
#define VAP_MODE_AP             2
#define BEACON_WPA2             BIT(4)

/* tx control block: wpa_supplicant waits for a frame with need_rsp */
#define TX_CB_NEED_RSP(cb)      ((cb)[4] & BIT(4))
#define TX_CB_FRAME_ID(cb)      extract32((cb)[5], 3, 4)

#define RX_RSSI                 (-40)
#define TX_RATE                 540     /* 100 kb/s */

typedef struct Hi3881Vap {
    uint8_t mode;
    uint8_t user;               /* hmac's user for the peer */
    uint8_t channel;
    bool up;
    bool rsn;
    MACAddr mac;
    uint8_t ssid[32];
    uint8_t ssid_len;
} Hi3881Vap;

typedef struct Hi3881State {
    SDIOCard parent_obj;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;

    bool running;               /* the firmware has booted */
    bool files_data;            /* the next FIFO write is FILES data */
    uint8_t int_stat;
    uint8_t int_enable;
    uint32_t msg_stat;
    uint8_t descr[HCC_DESCR_LEN];
    bool descr_valid;

    /* what the FIFO gives: a download answer, or the announced transfer */
    GByteArray *out;
    size_t out_off;
    uint32_t xfer_count;
    uint8_t comm_reg[EXTEND_BUFS];
    GQueue d2h;                 /* HCC packets for the host */
    uint8_t seq;

    QEMUTimer *scan_timer;
    bool scan_heard;
    uint8_t scan_vap;
    uint64_t scan_cookie;
    Hi3881Vap vap[VAPS];
    uint8_t sta_vap;            /* the last VAP to send the access point */
    uint8_t bss_vap;            /* the access point VAP the client sees */
    uint32_t rx_packets, tx_packets, rx_bytes, tx_bytes;
} Hi3881State;

static void hi_update(Hi3881State *s)
{
    /* the download polls the status; only the firmware raises the line */
    sdio_card_set_irq(SDIO_CARD(s), 1,
                      s->running && (s->int_stat & s->int_enable));
}

static void hi_answer(Hi3881State *s, const char *text)
{
    trace_hi3881_answer(text);
    g_clear_pointer(&s->out, g_byte_array_unref);
    s->out = g_byte_array_new();
    g_byte_array_append(s->out, (const uint8_t *)text, strlen(text) + 1);
    s->out_off = 0;
    s->xfer_count = s->out->len;
    s->int_stat |= INT_DREADY;
    hi_update(s);
}

/* A download command, as text; FILES data follows its header */
static void hi_download(Hi3881State *s, const uint8_t *buf, size_t len)
{
    g_autofree char *cmd = NULL;

    if (s->files_data) {
        s->files_data = false;
        hi_answer(s, "FILES OK");
        return;
    }
    cmd = g_strndup((const char *)buf, len);
    trace_hi3881_download(cmd);
    if (g_str_has_prefix(cmd, "VERSION")) {
        hi_answer(s, "Hi3881V100");
    } else if (g_str_has_prefix(cmd, "FILES")) {
        s->files_data = true;
        hi_answer(s, "READY");
    } else if (g_str_has_prefix(cmd, "CONFIG")) {
        hi_answer(s, "CONFIG OK");
    } else if (g_str_has_prefix(cmd, "WRITEM")) {
        hi_answer(s, "WRITEM OK");
    } else if (g_str_has_prefix(cmd, "JUMP")) {
        hi_answer(s, "JUMP OK");
    } else if (g_str_has_prefix(cmd, "QUIT")) {
        /* the firmware boots and says so */
        s->running = true;
        s->descr_valid = false;
        s->msg_stat |= BIT(D2H_WLAN_READY);
        s->int_stat |= INT_MFARM;
        hi_update(s);
    }
}

/* The next transfer for the host: the queued packets, a buffer each */
static void hi_stage(Hi3881State *s)
{
    GByteArray *u;
    unsigned i = 0;

    if (!s->running || s->out || g_queue_is_empty(&s->d2h)) {
        return;
    }
    s->out = g_byte_array_new();
    s->out_off = 0;
    memset(s->comm_reg, 0, sizeof(s->comm_reg));
    while (i < EXTEND_BUFS && (u = g_queue_pop_head(&s->d2h))) {
        size_t n = ROUND_UP(u->len, D2H_ALIGN);
        size_t at = s->out->len;

        g_byte_array_set_size(s->out, at + n);
        memset(s->out->data + at, 0, n);
        memcpy(s->out->data + at, u->data, u->len);
        s->comm_reg[i++] = n / D2H_ALIGN;
        g_byte_array_unref(u);
    }
    s->xfer_count = s->out->len < BLOCK_SIZE ? ROUND_UP(s->out->len, 4) :
                    ROUND_UP(s->out->len, BLOCK_SIZE);
    g_byte_array_set_size(s->out, s->xfer_count);
    s->int_stat |= INT_DREADY;
    hi_update(s);
}

/* An event for hmac, its body zeroed */
static GByteArray *hi_d2h_new(Hi3881State *s, uint8_t type, uint8_t sub,
                              uint8_t vap, size_t len)
{
    GByteArray *u = g_byte_array_new();

    g_byte_array_set_size(u, HCC_HDR_TOTAL + len);
    memset(u->data, 0, u->len);
    stw_le_p(u->data, HCC_SUB_DATA);
    stw_le_p(u->data + 2, (s->seq++ & 0xf) | len << 4);
    u->data[4] = type;
    u->data[5] = sub;
    u->data[6] = vap << 4;
    return u;
}

static void hi_d2h_send(Hi3881State *s, GByteArray *u)
{
    trace_hi3881_d2h(u->data[4], u->data[5], u->data[6] >> 4,
                     u->len - HCC_HDR_TOTAL);
    g_queue_push_tail(&s->d2h, u);
    hi_stage(s);
}

/* A frame as the dmac received it, with what it adds after the body */
static void hi_d2h_frame(Hi3881State *s, uint8_t type, uint8_t sub,
                         uint8_t vap, const uint8_t *f, size_t len,
                         size_t hdr, const uint8_t *tail, size_t tail_len)
{
    GByteArray *u = hi_d2h_new(s, type, sub, vap, len - hdr + tail_len);
    uint8_t *cb = u->data + HCC_CB;
    uint16_t fc = lduw_le_p(f);

    /*
     * First buffer of one, for the VAP it came to (hmac drops data for
     * the configuration VAP); the transmitter is hmac's user for the peer.
     */
    cb[0] = vap | BIT(6);
    cb[1] = 1;
    cb[2] = (s->vap[vap].user & 0xf) << 4;
    cb[3] = hdr | (WIFI_FC_TYPE(fc) == WIFI_TYPE_MGMT &&
                   WIFI_FC_STYPE(fc) == WIFI_STYPE_BEACON ? BIT(6) : 0);
    stw_le_p(cb + 4, len);
    cb[6] = vap | 1 << 4;
    cb[7] = vap == s->bss_vap ? s->vap[vap].channel : s->ap.channel;
    cb[12] = (uint8_t)RX_RSSI;
    memcpy(cb + CB_LEN, f, hdr);
    memcpy(u->data + HCC_HDR_TOTAL, f + hdr, len - hdr);
    if (tail_len) {
        memcpy(u->data + HCC_HDR_TOTAL + len - hdr, tail, tail_len);
    }
    hi_d2h_send(s, u);
}

/*
 * The chip's rate table (802.11b/g, rate in 500 kb/s units and Mb/s with
 * 5.5 as 5), then its MAC address: hmac matches every rate it hears
 * against the table. The host copies the chip's own PHY codes unread.
 */
static void hi_d2h_init(Hi3881State *s)
{
    static const uint8_t rates[][2] = {
        { 0x02, 1 }, { 0x04, 2 }, { 0x0b, 5 }, { 0x0c, 6 }, { 0x12, 9 },
        { 0x16, 11 }, { 0x18, 12 }, { 0x24, 18 }, { 0x30, 24 },
        { 0x48, 36 }, { 0x60, 48 }, { 0x6c, 54 },
    };
    GByteArray *u = hi_d2h_new(s, EV_WLAN_CRX, CRX_INIT, 0,
                               sizeof(rates) * 2 + 6);
    uint8_t *p = u->data + HCC_HDR_TOTAL;

    for (unsigned i = 0; i < ARRAY_SIZE(rates); i++, p += 4) {
        p[0] = rates[i][0];
        p[2] = rates[i][1];
    }
    memcpy(p, s->conf.macaddr.a, 6);
    hi_d2h_send(s, u);
}

static void hi_scan_done(void *opaque)
{
    Hi3881State *s = opaque;
    uint8_t f[WIFI_AP_BEACON_MAX];
    uint8_t ext[8] = { 0 };
    GByteArray *u;
    size_t n;

    if (s->scan_heard) {
        /* the dmac's notes on a BSS: RSSI, type (infrastructure), channel */
        n = wifi_ap_beacon(&s->ap, f);
        stl_le_p(ext, (uint32_t)RX_RSSI);
        ext[4] = 1;
        ext[5] = s->ap.channel;
        hi_d2h_frame(s, EV_WLAN_CRX, CRX_SCAN_RESULT, s->scan_vap, f, n,
                     WIFI_HDR_LEN, ext, sizeof(ext));
    }
    /* mac_scan_rsp_stru: success, then the cookie hmac waits for */
    u = hi_d2h_new(s, EV_WLAN_CRX, CRX_SCAN_COMP, s->scan_vap, 16);
    stq_le_p(u->data + HCC_HDR_TOTAL + 8, s->scan_cookie);
    hi_d2h_send(s, u);
}

/* A configuration message for hmac */
static GByteArray *hi_d2h_config(Hi3881State *s, uint8_t vap, uint16_t id,
                                 size_t len)
{
    GByteArray *u = hi_d2h_new(s, EV_DMAC_TO_HMAC_CFG, SYN_CFG, vap,
                               4 + len);

    stw_le_p(u->data + HCC_HDR_TOTAL, id);
    stw_le_p(u->data + HCC_HDR_TOTAL + 2, len);
    return u;
}

/* The client sees the first access point VAP up with an SSID */
static void hi_bss_update(Hi3881State *s)
{
    uint8_t bss = 0;
    Hi3881Vap *v;

    for (unsigned i = 1; i < VAPS && !bss; i++) {
        v = &s->vap[i];
        if (v->mode == VAP_MODE_AP && v->up && v->ssid_len) {
            bss = i;
        }
    }
    if (bss == s->bss_vap) {
        return;
    }
    trace_hi3881_bss(bss);
    s->bss_vap = bss;
    if (!s->client_nic) {
        return;
    }
    if (bss) {
        v = &s->vap[bss];
        wifi_client_bss(&s->client, v->mac.a, v->ssid, v->ssid_len,
                        v->rsn ? wifi_rsn_ie : NULL);
    } else {
        wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
    }
}

/* hmac waits for the answer to a station query: signal, counters, rate */
static void hi_query_station(Hi3881State *s, uint8_t vap)
{
    GByteArray *u = hi_d2h_config(s, vap, CFGID_QUERY_STATION,
                                  STATION_INFO_LEN);
    uint8_t *p = u->data + HCC_HDR_TOTAL + 4;

    stl_le_p(p, (uint32_t)RX_RSSI);
    stl_le_p(p + 4, s->rx_packets);
    stl_le_p(p + 8, s->tx_packets);
    stl_le_p(p + 12, s->rx_bytes);
    stl_le_p(p + 16, s->tx_bytes);
    /* struct rate_info at 36: flags, mcs, then the legacy rate */
    stw_le_p(p + 38, TX_RATE);
    p[42] = QUERY_STATION_INFO;
    hi_d2h_send(s, u);
}

static void hi_config(Hi3881State *s, uint8_t vap, const uint8_t *msg,
                      size_t len)
{
    const uint8_t *body = msg + 4;
    Hi3881Vap *v;
    size_t n;

    if (len < 4 || vap >= VAPS) {
        return;
    }
    v = &s->vap[vap];
    n = MIN(lduw_le_p(msg + 2), len - 4);
    switch (lduw_le_p(msg)) {
    case CFGID_ADD_VAP:
        /* mac_cfg_add_vap_param_stru: the mode first */
        if (n >= 1) {
            *v = (Hi3881Vap) { .mode = body[0] };
        }
        break;
    case CFGID_STATION_ID:
        if (n >= 6) {
            memcpy(v->mac.a, body, 6);
        }
        break;
    case CFGID_SSID:
        /* mac_cfg_ssid_param_stru: the length, two spare bytes, the SSID */
        if (n >= 3) {
            v->ssid_len = MIN(body[0], MIN(sizeof(v->ssid), n - 3));
            memcpy(v->ssid, body + 3, v->ssid_len);
        }
        break;
    case CFGID_CHANNEL:
        if (n >= 1) {
            v->channel = body[0];
        }
        break;
    case CFGID_CONFIG_BEACON:
        /* mac_beacon_param_stru: privacy at 8, the WPA versions at 9 */
        if (n >= 10) {
            v->rsn = body[8] && (body[9] & BEACON_WPA2);
        }
        break;
    case CFGID_START_VAP:
        v->up = true;
        break;
    case CFGID_DOWN_VAP:
    case CFGID_DEL_VAP:
        v->up = false;
        break;
    case CFGID_QUERY_STATION:
        hi_query_station(s, vap);
        return;
    default:
        return;
    }
    hi_bss_update(s);
}

/* The dmac scans; the access point is heard if its channel is on the list */
static void hi_scan(Hi3881State *s, uint8_t vap, const uint8_t *req,
                    size_t len)
{
    unsigned n;

    if (len < SCAN_REQ_LEN || vap >= VAPS) {
        return;
    }
    n = MIN(req[SCAN_REQ_NUMS], SCAN_REQ_CHANNELS);
    s->scan_heard = false;
    for (unsigned i = 0; i < n; i++) {
        s->scan_heard |= req[SCAN_REQ_CHANNEL + i * 4] == s->ap.channel;
    }
    s->scan_vap = vap;
    s->scan_cookie = ldq_le_p(req + SCAN_REQ_COOKIE);
    trace_hi3881_scan(vap, n, s->scan_heard);
    timer_mod(s->scan_timer,
              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SCAN_MS);
}

/*
 * A frame for the air: the control block names its MAC header's length,
 * the header follows the block and the body is the payload. Frames from
 * the access point VAP go to the client, the rest to the access point.
 */
static void hi_tx_frame(Hi3881State *s, uint8_t vap, const uint8_t *p,
                        size_t pay, bool mgmt)
{
    const uint8_t *cb = p + HCC_CB;
    size_t hdr = cb[1] & 0x3f;
    g_autofree uint8_t *f = NULL;
    bool acked;
    GByteArray *u;

    if (HCC_PAD_HDR(lduw_le_p(p)) || hdr < WIFI_HDR_LEN ||
        hdr > MAC_HDR_ROOM || vap >= VAPS) {
        return;
    }
    f = g_malloc(hdr + pay);
    memcpy(f, cb + CB_LEN, hdr);
    memcpy(f + hdr, p + HCC_HDR_TOTAL, pay);
    if (vap && vap == s->bss_vap) {
        acked = s->client_nic && wifi_client_acks(&s->client, f + 4);
        if (s->client_nic) {
            wifi_client_receive(&s->client, f, hdr + pay);
        }
    } else {
        acked = !memcmp(f + 4, s->ap.bssid.a, 6);
        s->sta_vap = vap;
        wifi_ap_receive(&s->ap, f, hdr + pay);
    }
    if (mgmt && TX_CB_NEED_RSP(cb)) {
        /* dmac_crx_mgmt_tx_status_stru: status, frame id, user */
        u = hi_d2h_config(s, vap, CFGID_MGMT_TX_STATUS, 4);
        u->data[HCC_HDR_TOTAL + 4] = acked ? TX_SUCC : TX_FAIL;
        u->data[HCC_HDR_TOTAL + 5] = TX_CB_FRAME_ID(cb);
        u->data[HCC_HDR_TOTAL + 6] = s->vap[vap].user;
        hi_d2h_send(s, u);
    }
}

/* One HCC packet from the host: its header area, then the payload */
static void hi_hcc_packet(Hi3881State *s, const uint8_t *p, size_t len)
{
    uint16_t w0, w1;
    size_t pad, pay;
    const uint8_t *ext, *body = p + HCC_HDR_TOTAL;
    uint8_t vap;

    if (len < HCC_HDR_TOTAL) {
        return;
    }
    w0 = lduw_le_p(p);
    w1 = lduw_le_p(p + 2);
    pad = HCC_PAD_HDR(w0);
    pay = MIN(HCC_PAY_LEN(w1), len - HCC_HDR_TOTAL);
    if (4 + pad + HCC_EXT_LEN > HCC_HDR_TOTAL) {
        return;
    }
    ext = p + 4 + pad;
    vap = extract32(ext[2], 4, 4);
    trace_hi3881_h2d(extract32(w0, 4, 3), extract32(w0, 0, 4), ext[0],
                     ext[1], vap, pay, pay >= 2 ? lduw_le_p(body) : 0);
    switch (ext[0]) {
    case EV_HOST_CRX:
        /* the dmac makes its configuration VAP and reports the device */
        if (ext[1] == HOST_CREATE_CFG_VAP) {
            hi_d2h_init(s);
        } else if (ext[1] == SYN_CFG) {
            hi_config(s, vap, body, pay);
        }
        break;
    case EV_WLAN_CTX:
        switch (ext[1]) {
        case CTX_ADD_USER:
            if (pay >= 1 && vap < VAPS) {
                s->vap[vap].user = body[0];
            }
            break;
        case CTX_SCAN_REQ:
            hi_scan(s, vap, body, pay);
            break;
        case CTX_MGMT:
            hi_tx_frame(s, vap, p, pay, true);
            break;
        }
        break;
    case EV_HOST_DRX:
        s->tx_packets++;
        s->tx_bytes += pay;
        hi_tx_frame(s, vap, p, pay, false);
        break;
    }
}

static void hi_hcc_transfer(Hi3881State *s, const uint8_t *buf, size_t len)
{
    size_t p = HCC_DESCR_LEN;

    if (len < HCC_DESCR_LEN) {
        return;
    }
    if (len == HCC_DESCR_LEN) {
        trace_hi3881_descr(ldl_le_p(buf));
        s->descr_valid = false;
        return;
    }
    if (!s->descr_valid || !s->descr[0]) {
        hi_hcc_packet(s, buf + p, len - p);
    } else {
        for (unsigned i = 0; i < HCC_DESCR_LEN && s->descr[i]; i++) {
            size_t n = (size_t)s->descr[i] << 3;

            if (p + n > len) {
                break;
            }
            hi_hcc_packet(s, buf + p, n);
            p += n;
        }
    }
    memcpy(s->descr, buf, HCC_DESCR_LEN);
    s->descr_valid = true;
}

static void hi_fifo_read(Hi3881State *s, uint8_t *buf, size_t len)
{
    size_t n = 0;

    if (s->out) {
        n = MIN(len, s->out->len - s->out_off);
        memcpy(buf, s->out->data + s->out_off, n);
        s->out_off += n;
        if (s->out_off >= s->out->len) {
            g_clear_pointer(&s->out, g_byte_array_unref);
            s->int_stat &= ~INT_DREADY;
            hi_stage(s);
        }
    }
    memset(buf + n, 0, len - n);
    hi_update(s);
}

static uint8_t hi_reg_read(Hi3881State *s, uint32_t a)
{
    uint8_t ext[EXTEND_LEN];

    switch (a) {
    case REG_INT_STATUS:
        return s->int_stat;
    case REG_INT_ENABLE:
        return s->int_enable;
    case REG_XFER_COUNT ... REG_XFER_COUNT + 3:
        return extract32(s->xfer_count, (a - REG_XFER_COUNT) * 8, 8);
    case REG_MSG_FROM_DEV ... REG_MSG_FROM_DEV + 3:
        return extract32(s->msg_stat, (a - REG_MSG_FROM_DEV) * 8, 8);
    }
    if (a >= REG_EXTEND && a < REG_EXTEND + EXTEND_LEN) {
        stl_le_p(ext, s->int_stat);
        stl_le_p(ext + 4, s->msg_stat);
        stl_le_p(ext + 8, s->xfer_count);
        stl_le_p(ext + 12, CREDIT_INFO);
        memcpy(ext + 16, s->comm_reg, EXTEND_BUFS);
        return ext[a - REG_EXTEND];
    }
    return 0;
}

static void hi_read(SDIOCard *card, unsigned fn, uint32_t addr, bool incr,
                    uint8_t *buf, size_t len)
{
    Hi3881State *s = HI3881(card);

    if (addr == REG_FIFO) {
        hi_fifo_read(s, buf, len);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = hi_reg_read(s, addr + (incr ? i : 0));
    }
    if (addr == REG_MSG_ACK) {
        trace_hi3881_msg_ack(s->msg_stat);
        s->msg_stat = 0;
    }
    /* the ISR reads the whole extend area: that takes the interrupt */
    if (s->running && addr == REG_EXTEND && len >= 4) {
        s->int_stat &= ~INT_MFARM;
        if (!s->out) {
            s->int_stat &= ~INT_DREADY;
        }
    }
    hi_update(s);
}

static void hi_write(SDIOCard *card, unsigned fn, uint32_t addr, bool incr,
                     const uint8_t *buf, size_t len)
{
    Hi3881State *s = HI3881(card);

    if (addr == REG_FIFO) {
        if (s->running) {
            hi_hcc_transfer(s, buf, len);
        } else {
            hi_download(s, buf, len);
        }
        return;
    }
    for (size_t i = 0; i < len; i++) {
        uint32_t a = addr + (incr ? i : 0);

        switch (a) {
        case REG_INT_STATUS:
            s->int_stat &= ~buf[i];
            break;
        case REG_INT_ENABLE:
            s->int_enable = buf[i];
            break;
        }
    }
    if (addr == REG_WRITE_MSG && len >= 4) {
        trace_hi3881_h2d_msg(ldl_le_p(buf));
    }
    hi_update(s);
}

static void hi_reset(Hi3881State *s)
{
    GByteArray *u;

    s->running = false;
    s->files_data = false;
    s->int_stat = 0;
    s->int_enable = 0;
    s->msg_stat = 0;
    s->descr_valid = false;
    s->xfer_count = 0;
    memset(s->comm_reg, 0, sizeof(s->comm_reg));
    timer_del(s->scan_timer);
    memset(s->vap, 0, sizeof(s->vap));
    s->sta_vap = 0;
    hi_bss_update(s);
    s->rx_packets = s->tx_packets = s->rx_bytes = s->tx_bytes = 0;
    g_clear_pointer(&s->out, g_byte_array_unref);
    while ((u = g_queue_pop_head(&s->d2h))) {
        g_byte_array_unref(u);
    }
}

static void hi_io_reset(SDIOCard *card)
{
    hi_reset(HI3881(card));
    hi_update(HI3881(card));
}

static bool hi_can_receive(NetClientState *nc)
{
    return true;
}

static ssize_t hi_receive(NetClientState *nc, const uint8_t *buf,
                          size_t size)
{
    Hi3881State *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
    return size;
}

static NetClientInfo hi_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = hi_can_receive,
    .receive = hi_receive,
};

/* What the access point sends the station, as the dmac passes it on */
static void hi_deliver(void *opaque, const uint8_t *frame, size_t len,
                       bool protected)
{
    Hi3881State *s = opaque;
    uint16_t fc = lduw_le_p(frame);
    size_t hdr = WIFI_HDR_LEN;

    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        hi_d2h_frame(s, EV_WLAN_CRX, CRX_RX, s->sta_vap, frame, len, hdr,
                     NULL, 0);
        break;
    case WIFI_TYPE_DATA:
        if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
            hdr += 2;
        }
        s->rx_packets++;
        s->rx_bytes += len;
        hi_d2h_frame(s, EV_WLAN_DRX, DRX_RX_STA, s->sta_vap, frame, len,
                     hdr, NULL, 0);
        break;
    }
}

/* What the client sends the access point VAP */
static void hi_client_deliver(void *opaque, const uint8_t *frame,
                              size_t len, bool protected)
{
    Hi3881State *s = opaque;
    uint16_t fc = lduw_le_p(frame);
    size_t hdr = WIFI_HDR_LEN;

    if (!s->bss_vap) {
        return;
    }
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        hi_d2h_frame(s, EV_WLAN_CRX, CRX_RX, s->bss_vap, frame, len, hdr,
                     NULL, 0);
        break;
    case WIFI_TYPE_DATA:
        if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
            hdr += 2;
        }
        hi_d2h_frame(s, EV_WLAN_DRX, DRX_RX_AP, s->bss_vap, frame, len,
                     hdr, NULL, 0);
        break;
    }
}

static bool hi_client_can_receive(NetClientState *nc)
{
    Hi3881State *s = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&s->client);
}

static ssize_t hi_client_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size)
{
    Hi3881State *s = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&s->client, buf, size);
    return size;
}

static NetClientInfo hi_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = hi_client_can_receive,
    .receive = hi_client_receive,
};

static void hi_realize(DeviceState *dev, Error **errp)
{
    Hi3881State *s = HI3881(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&hi_net_info, &s->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, s);
    if (!wifi_ap_realize(&s->ap, s->nic, hi_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
        return;
    }
    if (s->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&s->client_conf.macaddr);
        s->client_nic = qemu_new_nic(&hi_client_net_info, &s->client_conf,
                                     object_get_typename(OBJECT(dev)), NULL,
                                     &dev->mem_reentrancy_guard, s);
        wifi_client_init(&s->client, s->client_nic,
                         &s->client_conf.macaddr, hi_client_deliver, s);
    }
    hi_reset(s);
}

static void hi_unrealize(DeviceState *dev)
{
    Hi3881State *s = HI3881(dev);

    if (s->client_nic) {
        wifi_client_cleanup(&s->client);
        qemu_del_nic(s->client_nic);
        s->client_nic = NULL;
    }
    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void hi_init(Object *obj)
{
    Hi3881State *s = HI3881(obj);

    g_queue_init(&s->d2h);
    s->scan_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, hi_scan_done, s);
}

static void hi_finalize(Object *obj)
{
    Hi3881State *s = HI3881(obj);

    hi_reset(s);
    timer_free(s->scan_timer);
}

static const Property hi_props[] = {
    DEFINE_NIC_PROPERTIES(Hi3881State, conf),
    DEFINE_PROP_STRING("ssid", Hi3881State, ap.ssid),
    DEFINE_PROP_STRING("psk", Hi3881State, ap.psk),
    DEFINE_PROP_UINT8("channel", Hi3881State, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", Hi3881State, ap.bssid),
    DEFINE_PROP_NETDEV("client-netdev", Hi3881State, client_conf.peers),
    DEFINE_PROP_MACADDR("client-mac", Hi3881State, client_conf.macaddr),
    DEFINE_PROP_STRING("client-psk", Hi3881State, client.psk),
};

/* the firmware's state and the queued packets are not migrated */
static const VMStateDescription hi_vmstate = {
    .name = TYPE_HI3881,
    .unmigratable = 1,
};

static void hi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = hi_realize;
    dc->unrealize = hi_unrealize;
    dc->vmsd = &hi_vmstate;
    dc->desc = "HiSilicon Hi3881 SDIO Wi-Fi";
    device_class_set_props(dc, hi_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x0296;
    sc->device = 0x5347;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = BLOCK_SIZE;
    sc->read = hi_read;
    sc->write = hi_write;
    sc->io_reset = hi_io_reset;
}

static const TypeInfo hi_info = {
    .name           = TYPE_HI3881,
    .parent         = TYPE_SDIO_CARD,
    .instance_size  = sizeof(Hi3881State),
    .instance_init  = hi_init,
    .instance_finalize = hi_finalize,
    .class_init     = hi_class_init,
};

static void hi_register_types(void)
{
    type_register_static(&hi_info);
}

type_init(hi_register_types)
