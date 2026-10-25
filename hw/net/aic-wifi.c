/*
 * AICSemi AIC8800 Wi-Fi: the chip and firmware the bus front ends share
 *
 * The aic8800 driver comes in two modules. aic_load_fw talks to the boot
 * ROM with debug messages, reading the chip's identity, writing the
 * firmware and its patches into memory and starting it; the chip then
 * comes back as another device. aic8800_fdrv is a RivieraWaves FullMAC
 * driver: it sends request messages and gets confirmations and
 * indications back, the firmware scans and joins, and the host's
 * supplicant runs the 4-way handshake. Frames go to the chip as Ethernet
 * payloads behind a descriptor and come back as 802.11 frames behind an
 * RX header, the host turning them into Ethernet itself. For an access
 * point of its own the host answers the management frames.
 *
 * The DC and DW have no boot ROM to load: they run their firmware from
 * power on, and aic8800_fdrv patches it with the same debug messages and
 * calls into it, waiting on each call's confirmation.
 *
 * Every packet starts with a word of length and type; a message follows
 * it in the RivieraWaves layout, a frame its descriptor.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "net/eth.h"
#include "aic-wifi.h"
#include "trace.h"
#include "wifi-frame.h"

/*
 * A packet's word: a 12-bit length, then the type. A message is
 * configuration with a command or response; a TX confirmation is
 * configuration too; a frame is data.
 */
#define PKT_HDR_LEN             4
#define TYPE_DATA               0x01
#define TYPE_CFG_CMD_RSP        0x11
#define TYPE_CFG_DATA_CFM       0x12

/* to the chip: the word, a spare one, then id, destination, source, length */
#define A2E_HDR_LEN             (PKT_HDR_LEN + 4 + 8)
/* to the host: id, destination, source, length, a pattern, the parameters */
#define E2A_HDR_LEN             12
#define E2A_PATTERN             0xaddede2a
#define TASK_API                13

/* the host descriptor ahead of a frame for the chip */
#define HD_LEN                  28
#define HD_PACKET_LEN           0
#define HD_STATUS_DESC          4
#define HD_ETH_DEST             8
#define HD_ETH_SRC              14
#define HD_ETHERTYPE            20
#define HD_VIF                  24
#define HD_FLAGS                26
#define TXU_CNTRL_MGMT          BIT(3)
/* the host wants a TX confirmation, at this index */
#define STATUS_DESC_CFM         BIT(31)
#define TX_STATUS_DONE          BIT(0)
#define TX_STATUS_ACKED         BIT(3)

/*
 * The RX header ahead of a frame for the host: the vector, whose first
 * word is the packet's (the frame's length, type data), the channel, the
 * flags, and two bytes to put the frame at 60
 */
#define RX_HDR_LEN              60
#define RXH_RSSI1               17
#define RXH_PRIM20              42
#define RXH_CENTER1             44
#define RXH_FLAGS               48
#define RX_FLAGS_MPDU           BIT(1)
#define RX_FLAGS_UPLOAD         BIT(6)
#define RXH_VIF                 49
#define RXH_STA                 50
#define RXH_DST                 51
#define RX_RSSI                 (-45)

/* message ids: the task above 10 bits of index */
#define MM_RESET_REQ            0x0000
#define MM_VERSION_REQ          0x0004
#define MM_ADD_IF_REQ           0x0006
#define MM_REMOVE_IF_REQ        0x0008
#define MM_SET_CHANNEL_REQ      0x0010
#define MM_KEY_ADD_REQ          0x0024
#define MM_SET_POWER_REQ        0x002e
#define MM_CHANNEL_SWITCH_IND   0x0044
#define MM_REMAIN_ON_CHANNEL_REQ 0x0046
#define MM_SET_P2P_NOA_REQ      0x0051
#define MM_SET_P2P_OPPPS_REQ    0x0052
#define MM_CFG_RSSI_REQ         0x0056
#define MM_CFG_RSSI_CFM         0x0086
#define MM_SET_RF_CALIB_REQ     0x0069
#define MM_GET_MAC_ADDR_REQ     0x0073
#define MM_GET_STA_INFO_REQ     0x0075
#define MM_SET_STACK_START_REQ  0x007b
#define MM_SET_VENDOR_HWCONFIG_REQ 0x007e
#define MM_GET_FW_VERSION_REQ   0x0080
#define MM_SET_VENDOR_SWCONFIG_REQ 0x0087
#define DBG_MEM_READ_REQ        0x0400
#define DBG_MEM_WRITE_REQ       0x0402
#define DBG_GET_SYS_STAT_REQ    0x0409
#define DBG_START_APP_REQ       0x040d
#define DBG_MEM_MASK_WRITE_REQ  0x0411
#define DBG_GPIO_READ_REQ       0x041d
#define SCANU_START_REQ         0x1000
#define SCANU_START_CFM         0x1001
#define SCANU_RESULT_IND        0x1004
#define SCANU_START_CFM_ADDTIONAL 0x1009
#define SCANU_CANCEL_REQ        0x100a
#define ME_STA_ADD_REQ          0x1407
#define ME_STA_DEL_REQ          0x1409
#define ME_RC_STATS_REQ         0x140e
#define ME_CONFIG_MONITOR_REQ   0x1411
#define SM_CONNECT_REQ          0x1800
#define SM_CONNECT_IND          0x1802
#define SM_DISCONNECT_REQ       0x1803
#define SM_DISCONNECT_IND       0x1805
#define SM_EXT_AUTH_RSP         0x1807
#define SM_EXT_AUTH_RSP_CFM     0x180c
#define APM_START_REQ           0x1c00
#define APM_STOP_REQ            0x1c02
#define APM_SET_BEACON_IE_REQ   0x1c08

/* the start that sends the chip back to its boot ROM */
#define BOOT_REBOOT             3

/*
 * MM_VERSION_CFM: a Karst PHY of one stream (an ELMA one would cost the
 * 2.4 GHz band), beaconing (the driver keeps a broadcast queue) and the
 * UMAC
 */
#define VERSION_CFM_LEN         28
#define FW_LMAC_VERSION         0x0b040000
#define PHY_KARST_1SS           (2 << 16 | 1 << 8)
#define MM_FEAT_BCN             BIT(0)
#define MM_FEAT_UMAC            BIT(15)
#define REMOTE_STA_MAX          32

/* the parameters' lengths and fields the driver lays out */
#define SCANU_REQ_LEN           376
#define SCANU_CHANS             42
#define SCANU_CHAN_LEN          6
#define SCANU_SSIDS             252
#define SCANU_SSID_MAX          3
#define SCANU_VIF               366
#define SCANU_CHAN_CNT          367
#define SCANU_SSID_CNT          368
#define SCANU_RESULT_LEN        12
#define SM_CONNECT_REQ_IES      64
#define SM_CONNECT_IND_LEN      852
#define SM_CONNECT_IND_IES      20
#define SM_ASSOC_IE_LEN         800
#define ME_STA_ADD_REQ_LEN      136
#define APM_START_REQ_LEN       52
#define APM_START_FREQ          14
#define BCN_IE_REQ_LEN          4
#define VIF_AP                  2

/*
 * The station table: the access point joined, the client of the host's
 * own, then one for each interface's broadcast
 */
#define STA_AP                  0
#define STA_PEER                1
#define STA_BCMC(vif)           (REMOTE_STA_MAX + (vif))
#define STA_NONE                0xff

#define SCAN_MS                 100
#define JOIN_MS                 50
#define RETRY_MS                500
#define TRIES                   5
#define LISTEN_INTERVAL         10
#define STATUS_FAILURE          1
#define REASON_LEAVING          3
#define OUT_QUEUE_MAX           64

enum {
    AIC_STA_IDLE,
    AIC_STA_JOINING,
    AIC_STA_AUTHENTICATING,
    AIC_STA_ASSOCIATING,
    AIC_STA_ASSOCIATED,
};

static uint32_t aic_mem(AicCore *c, uint32_t addr)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(c->mem,
                            GUINT_TO_POINTER(addr)));
}

/* The centre frequency of a 2.4 GHz channel, in MHz */
static uint16_t aic_freq(uint8_t channel)
{
    return channel == 14 ? 2484 : 2407 + 5 * channel;
}

/*
 * A packet for the host. A message is padded to a word; a frame is not: a
 * driver without RX aggregation (the DC's and the 8801's) takes the whole
 * transfer for its frame
 */
static void aic_queue(AicCore *c, GQueue *q, const uint8_t *hdr,
                      size_t hlen, const void *data, size_t len)
{
    static const uint8_t zero[4];
    GByteArray *p = g_byte_array_sized_new(hlen + len + 3);

    g_byte_array_append(p, hdr, hlen);
    if (len) {
        g_byte_array_append(p, data, len);
    }
    if (q == &c->msgs) {
        g_byte_array_append(p, zero, -p->len & 3);
    }
    g_queue_push_tail(q, p);
    c->notify(c);
}

/* A confirmation or indication */
static void aic_msg_up(AicCore *c, uint16_t id, const void *param,
                       size_t len)
{
    uint8_t h[PKT_HDR_LEN + E2A_HDR_LEN] = {};

    stw_le_p(h, E2A_HDR_LEN + len);
    h[2] = TYPE_CFG_CMD_RSP;
    stw_le_p(h + 4, id);
    stw_le_p(h + 6, TASK_API);
    stw_le_p(h + 8, id >> 10);
    stw_le_p(h + 10, len);
    stl_le_p(h + 12, E2A_PATTERN);
    trace_aic_ind(id, len);
    aic_queue(c, &c->msgs, h, sizeof(h), param, len);
}

/* A frame went out: whether it was acked, at the index the host gave */
static void aic_tx_cfm(AicCore *c, uint32_t status_desc, bool acked)
{
    uint8_t h[PKT_HDR_LEN + 8] = {};

    stw_le_p(h, 8);
    h[2] = TYPE_CFG_DATA_CFM;
    stl_le_p(h + 4, TX_STATUS_DONE | (acked ? TX_STATUS_ACKED : 0));
    stl_le_p(h + 8, status_desc & ~STATUS_DESC_CFM);
    aic_queue(c, &c->msgs, h, sizeof(h), NULL, 0);
}

static bool aic_vif_is(AicCore *c, uint8_t vif, int type)
{
    return vif < AIC_VIFS && c->vif_type[vif] == type;
}

/* An 802.11 frame for the host, from station @sta on interface @vif */
static void aic_frame_up(AicCore *c, uint8_t vif, uint8_t sta, bool mpdu,
                         const uint8_t *f, size_t len)
{
    uint8_t h[RX_HDR_LEN] = {};
    uint16_t freq = aic_vif_is(c, vif, VIF_AP) ? c->ap_freq :
                    aic_freq(c->ap.channel);

    if (g_queue_get_length(&c->frames) >= OUT_QUEUE_MAX) {
        return;
    }
    stw_le_p(h, len);
    h[RXH_RSSI1] = RX_RSSI;
    stw_le_p(h + RXH_PRIM20, freq);
    stw_le_p(h + RXH_CENTER1, freq);
    h[RXH_FLAGS] = RX_FLAGS_UPLOAD | (mpdu ? RX_FLAGS_MPDU : 0);
    h[RXH_VIF] = vif;
    h[RXH_STA] = sta;
    h[RXH_DST] = STA_NONE;
    trace_aic_frame(true, lduw_le_p(f), len);
    aic_queue(c, &c->frames, h, sizeof(h), f, len);
}

/* An 802.11 data frame around an Ethernet payload, LLC/SNAP and all */
static size_t aic_wrap(uint8_t *f, uint16_t fc, const uint8_t *a1,
                       const uint8_t *a2, const uint8_t *a3, uint16_t seq,
                       const uint8_t *proto, const uint8_t *payload,
                       size_t len)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, a1, 6);
    memcpy(f + 10, a2, 6);
    memcpy(f + 16, a3, 6);
    stw_le_p(f + 22, (seq & 0xfff) << 4);
    memcpy(f + WIFI_HDR_LEN, WIFI_RFC1042, sizeof(WIFI_RFC1042));
    memcpy(f + WIFI_HDR_LEN + 6, proto, 2);
    memcpy(f + WIFI_HDR_LEN + 8, payload, len);
    return WIFI_HDR_LEN + 8 + len;
}

static void aic_set_sta(AicCore *c, int state)
{
    if (c->sta != state) {
        trace_aic_sta(c->sta, state);
    }
    c->sta = state;
}

/* To the BSS joined, from the joining interface */
static size_t aic_sta_hdr(AicCore *c, uint8_t *f, uint16_t fc)
{
    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, c->bssid.a, 6);
    memcpy(f + 10, c->vif_mac[c->sta_vif].a, 6);
    memcpy(f + 16, c->bssid.a, 6);
    stw_le_p(f + 22, (c->seq++ & 0xfff) << 4);
    return WIFI_HDR_LEN;
}

/* The channel context the station or the access point runs on is up */
static void aic_chan_switch(AicCore *c, uint8_t vif)
{
    uint8_t ind[4] = { 0, 0, vif, 0 };

    aic_msg_up(c, MM_CHANNEL_SWITCH_IND, ind, sizeof(ind));
}

/* The join's outcome: success with both sides' IEs, or a status */
static void aic_connect_ind(AicCore *c, uint16_t status)
{
    g_autofree uint8_t *p = g_malloc0(SM_CONNECT_IND_LEN);
    uint16_t freq = aic_freq(c->ap.channel);
    size_t req = 0, resp = 0;

    stw_le_p(p, status);
    memcpy(p + 2, c->bssid.a, 6);
    p[9] = c->sta_vif;
    p[10] = STA_AP;
    if (!status) {
        req = MIN(c->req_ies_len, SM_ASSOC_IE_LEN);
        resp = MIN(c->resp_ies_len, SM_ASSOC_IE_LEN - req);
        memcpy(p + SM_CONNECT_IND_IES, c->req_ies, req);
        memcpy(p + SM_CONNECT_IND_IES + req, c->resp_ies, resp);
    }
    stw_le_p(p + 14, req);
    stw_le_p(p + 16, resp);
    stw_le_p(p + 820, c->aid);
    /* 2.4 GHz, 20 MHz wide */
    stw_le_p(p + 824, freq);
    stl_le_p(p + 828, freq);
    aic_msg_up(c, SM_CONNECT_IND, p, SM_CONNECT_IND_LEN);
    if (!status) {
        aic_chan_switch(c, c->sta_vif);
    }
}

/* The join failed: the host hears it with the last status */
static void aic_join_fail(AicCore *c, uint16_t status)
{
    timer_del(c->join);
    aic_set_sta(c, AIC_STA_IDLE);
    aic_connect_ind(c, status ? status : STATUS_FAILURE);
}

/* Open authentication, then association, each sent until answered */
static void aic_sta_send(AicCore *c)
{
    uint8_t f[WIFI_HDR_LEN + 4 + sizeof(c->req_ies)];
    uint8_t *ies;
    size_t n;

    if (c->sta == AIC_STA_AUTHENTICATING) {
        n = aic_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_AUTH));
        /* open system, transaction 1 */
        stw_le_p(f + n, 0);
        stw_le_p(f + n + 2, 1);
        stw_le_p(f + n + 4, 0);
        n += 6;
    } else {
        n = aic_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_ASSOC_REQ));
        stw_le_p(f + n, WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE |
                        WIFI_CAP_SHORT_SLOT |
                        (c->ap.psk ? WIFI_CAP_PRIVACY : 0));
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
        memcpy(f + n, c->ies, c->ies_len);
        n += c->ies_len;
        c->req_ies_len = f + n - ies;
        memcpy(c->req_ies, ies, c->req_ies_len);
    }
    timer_mod(c->join, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + RETRY_MS);
    wifi_ap_receive(&c->ap, f, n);
}

/* A join under way: start it, or send again what went unanswered */
static void aic_join_tick(void *opaque)
{
    AicCore *c = opaque;
    size_t n = strlen(c->ap.ssid);

    switch (c->sta) {
    case AIC_STA_JOINING:
        if (memcmp(c->bssid.a, c->ap.bssid.a, 6) || c->ssid_len != n ||
            memcmp(c->ssid, c->ap.ssid, n)) {
            aic_join_fail(c, STATUS_FAILURE);
            return;
        }
        c->tries = 0;
        aic_set_sta(c, AIC_STA_AUTHENTICATING);
        aic_sta_send(c);
        break;
    case AIC_STA_AUTHENTICATING:
    case AIC_STA_ASSOCIATING:
        if (++c->tries >= TRIES) {
            aic_join_fail(c, STATUS_FAILURE);
            return;
        }
        aic_sta_send(c);
        break;
    }
}

/*
 * SM_CONNECT_REQ: the SSID, the BSSID (broadcast for any), and the IEs
 * the host wants in the association request
 */
static void aic_connect(AicCore *c, const uint8_t *p, size_t len)
{
    size_t ies;

    if (len < SM_CONNECT_REQ_IES) {
        return;
    }
    c->ssid_len = MIN(p[0], 32);
    memcpy(c->ssid, p + 1, c->ssid_len);
    if (is_broadcast_ether_addr(p + 34) || buffer_is_zero(p + 34, 6)) {
        memcpy(c->bssid.a, c->ap.bssid.a, 6);
    } else {
        memcpy(c->bssid.a, p + 34, 6);
    }
    c->sta_vif = p[61];
    ies = MIN(MIN(lduw_le_p(p + 54), len - SM_CONNECT_REQ_IES),
              sizeof(c->ies));
    memcpy(c->ies, p + SM_CONNECT_REQ_IES, ies);
    c->ies_len = ies;
    c->resp_ies_len = 0;
    aic_set_sta(c, AIC_STA_JOINING);
    timer_mod(c->join, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + JOIN_MS);
}

/* Out of the BSS: the access point told when it is the host's wish */
static void aic_sta_leave(AicCore *c, bool tell_ap, uint16_t reason)
{
    uint8_t f[WIFI_HDR_LEN + 2];
    uint8_t ind[6] = {};
    size_t n;

    timer_del(c->join);
    if (c->sta != AIC_STA_ASSOCIATED) {
        if (c->sta != AIC_STA_IDLE) {
            aic_join_fail(c, STATUS_FAILURE);
        }
        return;
    }
    if (tell_ap) {
        n = aic_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_DEAUTH));
        stw_le_p(f + n, reason);
        wifi_ap_receive(&c->ap, f, n + 2);
    }
    aic_set_sta(c, AIC_STA_IDLE);
    stw_le_p(ind, reason);
    ind[2] = c->sta_vif;
    aic_msg_up(c, SM_DISCONNECT_IND, ind, sizeof(ind));
}

static void aic_sta_mgmt(AicCore *c, uint16_t fc, const uint8_t *f,
                         size_t len)
{
    const uint8_t *body = f + WIFI_HDR_LEN;
    size_t blen = len - WIFI_HDR_LEN;

    switch (WIFI_FC_STYPE(fc)) {
    case WIFI_STYPE_AUTH:
        /* algorithm, transaction, status */
        if (c->sta != AIC_STA_AUTHENTICATING || blen < 6 ||
            lduw_le_p(body + 2) != 2) {
            return;
        }
        if (lduw_le_p(body + 4)) {
            aic_join_fail(c, lduw_le_p(body + 4));
            return;
        }
        c->tries = 0;
        aic_set_sta(c, AIC_STA_ASSOCIATING);
        aic_sta_send(c);
        return;
    case WIFI_STYPE_ASSOC_RESP:
    case WIFI_STYPE_REASSOC_RESP:
        /* capability, status, association id */
        if (c->sta != AIC_STA_ASSOCIATING || blen < 6) {
            return;
        }
        if (lduw_le_p(body + 2)) {
            aic_join_fail(c, lduw_le_p(body + 2));
            return;
        }
        timer_del(c->join);
        c->aid = lduw_le_p(body + 4) & 0x3fff;
        c->resp_ies_len = MIN(blen - 6, sizeof(c->resp_ies));
        memcpy(c->resp_ies, body + 6, c->resp_ies_len);
        aic_set_sta(c, AIC_STA_ASSOCIATED);
        aic_connect_ind(c, 0);
        return;
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (blen >= 2) {
            aic_sta_leave(c, false, lduw_le_p(body));
        }
        return;
    }
}

/* A frame from the access point, its cipher taken off */
static void aic_deliver(void *opaque, const uint8_t *f, size_t len,
                        bool protected)
{
    AicCore *c = opaque;
    uint16_t fc;

    if (len < WIFI_HDR_LEN || c->sta < AIC_STA_AUTHENTICATING ||
        memcmp(f + 10, c->bssid.a, 6) ||
        (!is_multicast_ether_addr(f + 4) &&
         memcmp(f + 4, c->vif_mac[c->sta_vif].a, 6))) {
        return;
    }
    fc = lduw_le_p(f);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        aic_sta_mgmt(c, fc, f, len);
        break;
    case WIFI_TYPE_DATA:
        /* the host takes the frame apart; it wants it from the DS */
        if (c->sta == AIC_STA_ASSOCIATED && (fc & WIFI_FC_FROMDS) &&
            !(fc & WIFI_FC_TODS)) {
            aic_frame_up(c, c->sta_vif, STA_AP, false, f, len);
        }
        break;
    }
}

/*
 * A frame from the client of the host's access point: management goes up
 * whole for the host to answer, data once the host has added the station
 */
static void aic_client_deliver(void *opaque, const uint8_t *f, size_t len,
                               bool protected)
{
    AicCore *c = opaque;
    uint16_t fc;

    if (!c->bss_up || len < WIFI_HDR_LEN ||
        memcmp(f + 4, c->ap_bssid.a, 6)) {
        return;
    }
    fc = lduw_le_p(f);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        aic_frame_up(c, c->ap_vif, STA_NONE, true, f, len);
        break;
    case WIFI_TYPE_DATA:
        if (c->peer_added && (fc & WIFI_FC_TODS) &&
            !(fc & WIFI_FC_FROMDS)) {
            aic_frame_up(c, c->ap_vif, STA_PEER, false, f, len);
        }
        break;
    }
}

/* Tell the client whether there is an access point to join */
static void aic_set_bss(AicCore *c, bool up)
{
    if (c->bss_up == up) {
        return;
    }
    trace_aic_bss(up);
    c->bss_up = up;
    if (!up) {
        c->peer_added = false;
    }
    if (!c->client_nic) {
        return;
    }
    if (up) {
        wifi_client_bss(&c->client, c->ap_bssid.a, c->ap_ssid,
                        c->ap_ssid_len, NULL);
    } else {
        wifi_client_bss(&c->client, NULL, NULL, 0, NULL);
    }
}

/* The host's beacon for its access point: the BSSID and the SSID */
static void aic_beacon(AicCore *c, const uint8_t *f, size_t len)
{
    const size_t ie = WIFI_HDR_LEN + 12;

    if (len < ie + 2 || lduw_le_p(f) != WIFI_FC_MGMT(WIFI_STYPE_BEACON) ||
        f[ie] != WIFI_EID_SSID || ie + 2 + f[ie + 1] > len) {
        return;
    }
    memcpy(c->ap_bssid.a, f + 16, 6);
    c->ap_ssid_len = MIN(f[ie + 1], sizeof(c->ap_ssid));
    memcpy(c->ap_ssid, f + ie + 2, c->ap_ssid_len);
}

/* SCANU_START_REQ: the access point is found on its channel, by name */
static void aic_scan(AicCore *c, const uint8_t *p, size_t len)
{
    size_t n = strlen(c->ap.ssid);
    unsigned chans, ssids;
    bool chan = false, ssid = false;

    if (len < SCANU_REQ_LEN) {
        return;
    }
    c->scan_vif = p[SCANU_VIF];
    chans = p[SCANU_CHAN_CNT];
    ssids = p[SCANU_SSID_CNT];
    for (unsigned i = 0; i < MIN(chans, SCANU_CHANS); i++) {
        chan |= lduw_le_p(p + i * SCANU_CHAN_LEN) == aic_freq(c->ap.channel);
    }
    for (unsigned i = 0; i < MIN(ssids, SCANU_SSID_MAX); i++) {
        const uint8_t *s = p + SCANU_SSIDS + i * 33;

        ssid |= !s[0] || (s[0] == n && !memcmp(s + 1, c->ap.ssid, n));
    }
    c->scan_hit = (chan || !chans) && (ssid || !ssids);
    timer_mod(c->scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SCAN_MS);
}

/* The scan's end: the access point's beacon if it was heard */
static void aic_scan_done(void *opaque)
{
    AicCore *c = opaque;
    uint8_t r[SCANU_RESULT_LEN + WIFI_AP_BEACON_MAX] = {};
    uint8_t cfm[3] = { c->scan_vif };
    size_t n;

    if (c->scan_hit) {
        n = wifi_ap_beacon(&c->ap, r + SCANU_RESULT_LEN);
        stw_le_p(r, n);
        stw_le_p(r + 2, lduw_le_p(r + SCANU_RESULT_LEN));
        stw_le_p(r + 4, aic_freq(c->ap.channel));
        r[7] = STA_NONE;
        r[8] = c->scan_vif;
        r[9] = RX_RSSI;
        aic_msg_up(c, SCANU_RESULT_IND, r, SCANU_RESULT_LEN + n);
        cfm[2] = 1;
    }
    aic_msg_up(c, SCANU_START_CFM, cfm, sizeof(cfm));
}

/*
 * An Ethernet payload from the host, its addresses in the descriptor, for
 * whichever BSS its interface is in; the host may want to hear it went
 */
static void aic_data_tx(AicCore *c, const uint8_t *hd, const uint8_t *p,
                        size_t len)
{
    g_autofree uint8_t *f = g_malloc(WIFI_HDR_LEN + 8 + len);
    uint8_t vif = hd[HD_VIF];
    uint32_t cfm = ldl_le_p(hd + HD_STATUS_DESC);
    bool sent = false;
    size_t n;

    trace_aic_frame(false, lduw_be_p(hd + HD_ETHERTYPE), len);
    if (aic_vif_is(c, vif, VIF_AP)) {
        if (c->bss_up && c->peer_added) {
            n = aic_wrap(f, WIFI_FC_DATA | WIFI_FC_FROMDS, hd + HD_ETH_DEST,
                         c->ap_bssid.a, hd + HD_ETH_SRC, c->seq++,
                         hd + HD_ETHERTYPE, p, len);
            wifi_client_receive(&c->client, f, n);
            sent = true;
        }
    } else if (c->sta == AIC_STA_ASSOCIATED && vif == c->sta_vif) {
        n = aic_wrap(f, WIFI_FC_DATA | WIFI_FC_TODS, c->bssid.a,
                     c->vif_mac[vif].a, hd + HD_ETH_DEST, c->seq++,
                     hd + HD_ETHERTYPE, p, len);
        wifi_ap_receive(&c->ap, f, n);
        sent = true;
    }
    if (cfm & STATUS_DESC_CFM) {
        aic_tx_cfm(c, cfm, sent);
    }
}

/*
 * A management frame the host built: to the client of its access point,
 * or the one joined
 */
static void aic_mgmt_tx(AicCore *c, const uint8_t *hd, const uint8_t *f,
                        size_t len)
{
    uint32_t cfm = ldl_le_p(hd + HD_STATUS_DESC);
    bool acked = false;

    if (len < WIFI_HDR_LEN) {
        return;
    }
    trace_aic_frame(false, lduw_le_p(f), len);
    if (aic_vif_is(c, hd[HD_VIF], VIF_AP)) {
        if (c->bss_up && c->client_nic) {
            acked = wifi_client_acks(&c->client, f + 4);
            wifi_client_receive(&c->client, f, len);
        }
    } else {
        acked = c->sta == AIC_STA_ASSOCIATED;
        wifi_ap_receive(&c->ap, f, len);
    }
    if (cfm & STATUS_DESC_CFM) {
        aic_tx_cfm(c, cfm, acked);
    }
}

/* MM_ADD_IF_REQ: the type and the address; the index is the firmware's */
static void aic_add_if(AicCore *c, const uint8_t *p, size_t len,
                       uint8_t *cfm)
{
    unsigned i;

    cfm[0] = STATUS_FAILURE;
    if (len < 8) {
        return;
    }
    for (i = 0; i < AIC_VIFS && c->vif_type[i] >= 0; i++) {
        continue;
    }
    if (i == AIC_VIFS) {
        return;
    }
    c->vif_type[i] = p[0];
    memcpy(c->vif_mac[i].a, p + 2, 6);
    cfm[0] = 0;
    cfm[1] = i;
}

static void aic_remove_if(AicCore *c, uint8_t vif)
{
    if (vif >= AIC_VIFS) {
        return;
    }
    if (vif == c->sta_vif && c->sta != AIC_STA_IDLE) {
        aic_sta_leave(c, true, REASON_LEAVING);
    }
    if (vif == c->ap_vif) {
        aic_set_bss(c, false);
    }
    c->vif_type[vif] = -1;
}

/* What the boot ROM and the firmware read back */
static uint32_t aic_mem_read(AicCore *c, uint32_t addr)
{
    for (const AicWord *w = c->words; w && w->addr; w++) {
        if (w->addr == addr) {
            return w->value;
        }
    }
    return aic_mem(c, addr);
}

/* A debug request, the boot ROM's and the firmware's both */
static bool aic_dbg(AicCore *c, uint16_t id, const uint8_t *p, size_t len,
                    uint8_t *cfm, size_t *n)
{
    uint32_t addr = len >= 4 ? ldl_le_p(p) : 0;

    switch (id) {
    case DBG_MEM_READ_REQ:
        stl_le_p(cfm, addr);
        stl_le_p(cfm + 4, aic_mem_read(c, addr));
        *n = 8;
        return true;
    case DBG_MEM_WRITE_REQ:
        if (len >= 8) {
            g_hash_table_insert(c->mem, GUINT_TO_POINTER(addr),
                                GUINT_TO_POINTER(ldl_le_p(p + 4)));
        }
        return true;
    case DBG_MEM_MASK_WRITE_REQ:
        if (len >= 12) {
            uint32_t mask = ldl_le_p(p + 4);

            g_hash_table_insert(c->mem, GUINT_TO_POINTER(addr),
                                GUINT_TO_POINTER((aic_mem(c, addr) & ~mask) |
                                                 (ldl_le_p(p + 8) & mask)));
        }
        return true;
    }
    return false;
}

/* A request, and its confirmation: the next id, but for a few */
static void aic_msg(AicCore *c, uint16_t id, const uint8_t *p, size_t len)
{
    uint8_t cfm[64] = {};
    uint16_t cfm_id = id + 1;
    size_t n = 0;

    trace_aic_req(id, len);
    if (aic_dbg(c, id, p, len, cfm, &n)) {
        aic_msg_up(c, cfm_id, cfm, n);
        return;
    }
    switch (id) {
    case DBG_START_APP_REQ: {
        /* the chip restarts, into the firmware or back into the boot ROM */
        bool app = len < 8 || ldl_le_p(p + 4) != BOOT_REBOOT;

        if (!c->rom) {
            /* its own driver calls into the firmware, and waits on it */
            aic_msg_up(c, cfm_id, cfm, 4);
            if (!app) {
                aic_core_reset(c);
                c->replug(c);
            }
            return;
        }
        if (app != c->app) {
            if (!app) {
                aic_core_reset(c);
            }
            c->app = app;
            c->replug(c);
        }
        if (c->start_cfm) {
            aic_msg_up(c, cfm_id, cfm, 4);
        }
        return;
    }
    case MM_RESET_REQ:
        aic_sta_leave(c, true, REASON_LEAVING);
        break;
    case MM_VERSION_REQ:
        stl_le_p(cfm, FW_LMAC_VERSION);
        stl_le_p(cfm + 12, PHY_KARST_1SS);
        stl_le_p(cfm + 20, MM_FEAT_BCN | MM_FEAT_UMAC);
        stw_le_p(cfm + 24, REMOTE_STA_MAX);
        cfm[26] = AIC_VIFS;
        n = VERSION_CFM_LEN;
        break;
    case MM_SET_STACK_START_REQ:
        /* no 5 GHz band, no vendor information */
        n = 2;
        break;
    case MM_GET_FW_VERSION_REQ: {
        static const char ver[] = "qemu";

        cfm[0] = sizeof(ver) - 1;
        memcpy(cfm + 1, ver, sizeof(ver) - 1);
        n = 64;
        break;
    }
    case MM_GET_MAC_ADDR_REQ:
        memcpy(cfm, c->conf.macaddr.a, 6);
        n = 6;
        break;
    case MM_SET_RF_CALIB_REQ:
        n = 16;
        break;
    case MM_GET_STA_INFO_REQ:
        cfm[8] = RX_RSSI;
        n = 12;
        break;
    case MM_SET_VENDOR_HWCONFIG_REQ:
    case MM_SET_VENDOR_SWCONFIG_REQ:
        /* the configuration's id, echoed */
        if (len >= 4) {
            memcpy(cfm, p, 4);
        }
        n = 8;
        break;
    case MM_ADD_IF_REQ:
        aic_add_if(c, p, len, cfm);
        n = 2;
        break;
    case MM_REMOVE_IF_REQ:
        if (len >= 1) {
            aic_remove_if(c, p[0]);
        }
        break;
    case MM_SET_CHANNEL_REQ:
    case MM_SET_POWER_REQ:
        /* the radio, and the power it was given */
        cfm[1] = 20;
        n = 2;
        break;
    case MM_KEY_ADD_REQ:
        /* the hardware slot: the key's own, or one per station */
        if (len >= 44) {
            cfm[1] = p[43] ? 8 + p[1] : p[0];
        }
        n = 2;
        break;
    case MM_REMAIN_ON_CHANNEL_REQ:
        n = 3;
        break;
    case MM_SET_P2P_NOA_REQ:
    case MM_SET_P2P_OPPPS_REQ:
        cfm_id = id + 2;
        n = 1;
        break;
    case MM_CFG_RSSI_REQ:
        cfm_id = MM_CFG_RSSI_CFM;
        break;
    case DBG_GET_SYS_STAT_REQ:
    case ME_CONFIG_MONITOR_REQ:
        n = 12;
        break;
    case DBG_GPIO_READ_REQ:
        n = 2;
        break;
    case SCANU_START_REQ:
        /* a word now, SCANU_START_CFM when the scan is over */
        aic_scan(c, p, len);
        cfm_id = SCANU_START_CFM_ADDTIONAL;
        break;
    case SCANU_CANCEL_REQ:
        n = 1;
        break;
    case ME_STA_ADD_REQ:
        if (len >= ME_STA_ADD_REQ_LEN && c->client_nic &&
            !memcmp(p, c->client_conf.macaddr.a, 6)) {
            c->peer_added = true;
        }
        cfm[0] = STA_PEER;
        n = 3;
        break;
    case ME_STA_DEL_REQ:
        if (len >= 1 && p[0] == STA_PEER) {
            c->peer_added = false;
        }
        break;
    case ME_RC_STATS_REQ: {
        uint8_t stats[200] = {};

        aic_msg_up(c, cfm_id, stats, sizeof(stats));
        return;
    }
    case SM_CONNECT_REQ:
        aic_connect(c, p, len);
        n = 1;
        break;
    case SM_DISCONNECT_REQ:
        aic_msg_up(c, cfm_id, NULL, 0);
        aic_sta_leave(c, true, len >= 2 ? lduw_le_p(p) : REASON_LEAVING);
        return;
    case SM_EXT_AUTH_RSP:
        cfm_id = SM_EXT_AUTH_RSP_CFM;
        break;
    case APM_SET_BEACON_IE_REQ:
        if (len >= BCN_IE_REQ_LEN) {
            aic_beacon(c, p + BCN_IE_REQ_LEN,
                       MIN(lduw_le_p(p + 2), len - BCN_IE_REQ_LEN));
        }
        break;
    case APM_START_REQ:
        if (len < APM_START_REQ_LEN) {
            cfm[0] = STATUS_FAILURE;
        } else {
            c->ap_vif = p[51];
            c->ap_freq = lduw_le_p(p + APM_START_FREQ);
            cfm[1] = c->ap_vif;
            cfm[3] = STA_BCMC(c->ap_vif);
        }
        aic_msg_up(c, cfm_id, cfm, 4);
        if (!cfm[0]) {
            aic_chan_switch(c, c->ap_vif);
            aic_set_bss(c, true);
        }
        return;
    case APM_STOP_REQ:
        aic_set_bss(c, false);
        break;
    }
    aic_msg_up(c, cfm_id, cfm, n);
}

void aic_core_host(AicCore *c, const uint8_t *buf, size_t len)
{
    const uint8_t *hd = buf + PKT_HDR_LEN;
    size_t plen;

    if (len < PKT_HDR_LEN) {
        return;
    }
    len = MIN(len, lduw_le_p(buf) & 0xfff);
    switch (buf[2]) {
    case TYPE_CFG_CMD_RSP:
        /* the length counts the message and four bytes more */
        if (len >= A2E_HDR_LEN - 4) {
            plen = MIN(lduw_le_p(buf + 14), len + 4 - A2E_HDR_LEN);
            aic_msg(c, lduw_le_p(buf + 8), buf + A2E_HDR_LEN, plen);
        }
        break;
    case TYPE_DATA:
        if (len < PKT_HDR_LEN + HD_LEN) {
            return;
        }
        plen = MIN(lduw_le_p(hd + HD_PACKET_LEN), len - PKT_HDR_LEN - HD_LEN);
        if (lduw_le_p(hd + HD_FLAGS) & TXU_CNTRL_MGMT) {
            aic_mgmt_tx(c, hd, hd + HD_LEN, plen);
        } else {
            aic_data_tx(c, hd, hd + HD_LEN, plen);
        }
        break;
    }
}

static GByteArray *aic_pop(AicCore *c, GQueue *q)
{
    GByteArray *p = g_queue_pop_head(q);

    if (p && q == &c->frames &&
        g_queue_get_length(q) == OUT_QUEUE_MAX - 1) {
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        if (c->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(c->client_nic));
        }
    }
    return p;
}

GByteArray *aic_core_pop_msg(AicCore *c)
{
    return aic_pop(c, &c->msgs);
}

GByteArray *aic_core_pop_frame(AicCore *c)
{
    return aic_pop(c, &c->frames);
}

static void aic_flush(GQueue *q)
{
    GByteArray *p;

    while ((p = g_queue_pop_head(q))) {
        g_byte_array_unref(p);
    }
}

void aic_core_reset(AicCore *c)
{
    timer_del(c->scan);
    timer_del(c->join);
    aic_flush(&c->msgs);
    aic_flush(&c->frames);
    g_hash_table_remove_all(c->mem);
    memset(c->vif_type, -1, sizeof(c->vif_type));
    c->app = !c->rom;
    c->sta = AIC_STA_IDLE;
    if (c->ap.eapol_timer) {
        wifi_ap_reset(&c->ap);
    }
    c->sta_vif = 0;
    c->ap_vif = 0;
    aic_set_bss(c, false);
}

static bool aic_can_receive(NetClientState *nc)
{
    AicCore *c = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&c->frames) < OUT_QUEUE_MAX;
}

static ssize_t aic_receive(NetClientState *nc, const uint8_t *buf,
                           size_t size)
{
    AicCore *c = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&c->ap, buf, size);
    return size;
}

static NetClientInfo aic_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = aic_can_receive,
    .receive = aic_receive,
};

static bool aic_client_can_receive(NetClientState *nc)
{
    AicCore *c = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&c->client) &&
           g_queue_get_length(&c->frames) < OUT_QUEUE_MAX;
}

static ssize_t aic_client_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    AicCore *c = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&c->client, buf, size);
    return size;
}

static NetClientInfo aic_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = aic_client_can_receive,
    .receive = aic_client_receive,
};

void aic_core_init(AicCore *c)
{
    c->mem = g_hash_table_new(g_direct_hash, g_direct_equal);
    c->scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, aic_scan_done, c);
    c->join = timer_new_ms(QEMU_CLOCK_VIRTUAL, aic_join_tick, c);
    g_queue_init(&c->msgs);
    g_queue_init(&c->frames);
}

bool aic_core_realize(AicCore *c, Error **errp)
{
    DeviceState *dev = c->dev;

    qemu_macaddr_default_if_unset(&c->conf.macaddr);
    c->nic = qemu_new_nic(&aic_net_info, &c->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, c);
    if (!wifi_ap_realize(&c->ap, c->nic, aic_deliver, c, errp)) {
        qemu_del_nic(c->nic);
        c->nic = NULL;
        return false;
    }
    if (c->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&c->client_conf.macaddr);
        c->client_nic = qemu_new_nic(&aic_client_net_info, &c->client_conf,
                                     object_get_typename(OBJECT(dev)), NULL,
                                     &dev->mem_reentrancy_guard, c);
        wifi_client_init(&c->client, c->client_nic, &c->client_conf.macaddr,
                         aic_client_deliver, c);
    }
    aic_core_reset(c);
    return true;
}

void aic_core_unrealize(AicCore *c)
{
    timer_del(c->scan);
    timer_del(c->join);
    if (c->client_nic) {
        wifi_client_cleanup(&c->client);
        qemu_del_nic(c->client_nic);
        c->client_nic = NULL;
    }
    wifi_ap_unrealize(&c->ap);
    qemu_del_nic(c->nic);
}

void aic_core_finalize(AicCore *c)
{
    timer_free(c->scan);
    timer_free(c->join);
    aic_flush(&c->msgs);
    aic_flush(&c->frames);
    g_hash_table_destroy(c->mem);
}
