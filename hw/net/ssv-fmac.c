/*
 * iComm-semi SSV6355 Wi-Fi: the FullMAC firmware of its SSV6030
 *
 * The pid_6011 ssv6x5x driver leaves the 802.11 MAC to the firmware, as
 * the RivieraWaves design it comes from does: the host sends request
 * messages and gets confirmations and indications back, the firmware
 * scans and joins, and frames cross as Ethernet with a descriptor naming
 * the interface and the station. The host's supplicant runs the 4-way
 * handshake. For an access point of its own the host answers the
 * management frames, the firmware sending its beacon.
 *
 * Every transfer to the chip starts with a BMU descriptor, an SDIO header
 * with the content type and the host descriptor; every unit to the host
 * with a word of type and length, the RX information and a reorder
 * header. The host aggregates what it sends when the HCI control
 * register says so; the chip always does.
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
#include "ssv-wifi.h"
#include "trace.h"
#include "wifi-frame.h"

/* bit 0: the host aggregates what it sends */
#define REG_HCI_CTRL            0x08c10008
#define HCI_TX_AGGR             BIT(0)
/* free TX pages, in [23:16] those the host may fill: always plenty */
#define REG_TX_BMU_FREE         0x08c10050
#define TX_BMU_FREE_ALL         0x00ff01ff
/* the firmware's version, which the driver must match */
#define REG_FW_VERSION          0x08d00804
#define FW_VERSION              11204
/* a count the firmware keeps moving; stalled, the driver reloads it */
#define REG_FW_STATUS           0x08d010f8
/* the eFuse's MAC: bytes 2 to 5 in word 3, 0 and 1 in word 4 */
#define REG_EFUSE_MAC_LO        0x0850000c
#define REG_EFUSE_MAC_HI        0x08500010

/* to the chip: BMU descriptor, SDIO header, host descriptor, content */
#define BMU_LEN                 32
#define SDIO_HDR_LEN            8
#define HD_LEN                  44
#define TX_HDR_LEN              (BMU_LEN + SDIO_HDR_LEN + HD_LEN)
/* an aggregate unit: its length twice, a spare word; a zero word ends */
#define AGG_HDR_LEN             8

/* the content types, in the SDIO header and in a unit for the host */
#define IPC_MSG                 0
#define IPC_DATA                3
#define IPC_PRIV_MSG            15

/* the host descriptor */
#define HD_PACKET_LEN           4
#define HD_VIF                  33
#define HD_FLAGS                36
#define HD_SW_ACK               38
#define TXU_CNTRL_MGMT          BIT(3)
/* the host wants the frame's fate; the answer's bit 3: it was acked */
#define SW_ACK_WANTED           BIT(0)
#define SW_ACK_ACKED            BIT(3)

/* to the host: type and length, RX information, reorder header */
#define RX_HWIF_LEN             4
#define RX_INFO_LEN             84
#define RX_REORDER_LEN          24
#define RX_HDR_LEN              (RX_HWIF_LEN + RX_INFO_LEN + RX_REORDER_LEN)
#define RXI_FRMLEN              0
#define RXI_RSSI_LEG            14
#define RXI_RSSI1               17
#define RXI_PHY_INFO            40
#define RXI_FLAGS               48
#define RX_FLAGS_MPDU           BIT(1)
#define RX_FLAGS(vif, sta)      ((vif) << 8 | (sta) << 16)
#define RX_RSSI                 (-45)
/* the most the host takes in a transfer */
#define AGG_MAX                 (20 * 1024)

/*
 * A message to the chip: a spare word, the id, destination and source
 * tasks, the parameters' length; one to the host leaves the spare out.
 * A message's id is its task above 10 bits of index, and a request's
 * confirmation the next id.
 */
#define A2E_HDR_LEN             12
#define E2A_HDR_LEN             8
#define TASK_API                13

#define MM_RESET_REQ            0x0000
#define MM_VERSION_REQ          0x0004
#define MM_ADD_IF_REQ           0x0006
#define MM_REMOVE_IF_REQ        0x0008
#define MM_SET_CHANNEL_REQ      0x0010
#define MM_KEY_ADD_REQ          0x0024
#define MM_KEY_DEL_REQ          0x0026
#define MM_SET_POWER_REQ        0x002e
#define MM_BCN_CHANGE_REQ       0x003f
#define MM_TIM_UPDATE_REQ       0x0041
#define SCANU_START_REQ         0x1000
#define SCANU_START_CFM         0x1001
#define SCANU_RESULT_IND        0x1004
#define ME_STA_ADD_REQ          0x1407
#define ME_STA_DEL_REQ          0x1409
#define ME_TRAFFIC_IND_REQ      0x140c
#define ME_RC_STATS_REQ         0x140e
#define ME_RC_SET_RATE_REQ      0x1410
#define SM_CONNECT_REQ          0x1800
#define SM_CONNECT_IND          0x1802
#define SM_DISCONNECT_REQ       0x1803
#define SM_DISCONNECT_IND       0x1805
#define APM_START_REQ           0x1c00
#define APM_STOP_REQ            0x1c02

#define PRIV_SW_ACK_IND         6

/*
 * MM_VERSION_CFM: a Karst PHY of one stream (an ELMA would cost the
 * 2.4 GHz band), and the UMAC present
 */
#define VERSION_CFM_LEN         27
#define FW_LMAC_VERSION         0x0b040000
#define PHY_KARST_1SS           (2 << 16 | 1 << 8)
#define MM_FEAT_UMAC            BIT(8)
#define REMOTE_STA_MAX          11

/* the parameters' lengths and fields the driver lays out */
#define SCANU_REQ_LEN           296
#define SCANU_SSIDS             210
#define SCANU_VIF               288
#define SCANU_CHAN_CNT          289
#define SCANU_SSID_CNT          290
#define SM_CONNECT_REQ_IES      58
#define SM_CONNECT_IND_LEN      846
#define SM_CONNECT_IND_IES      46
#define SM_ASSOC_IE_LEN         800
#define ME_STA_ADD_REQ_LEN      124
#define ME_RC_STATS_CFM_LEN     197
#define APM_START_REQ_LEN       41
#define MM_BCN_CHANGE_REQ_LEN   12
#define VIF_STA                 0
#define VIF_AP                  2

/*
 * The station table: the access point joined, the client of the host's
 * own, then one for each interface's broadcast
 */
#define STA_AP                  0
#define STA_PEER                1
#define STA_BCMC(vif)           (REMOTE_STA_MAX + (vif))
#define STA_NONE                0xff

/* how long a scan takes, a join takes to start, and its tries */
#define SCAN_MS                 100
#define JOIN_MS                 50
#define RETRY_MS                500
#define TRIES                   5
#define LISTEN_INTERVAL         10
#define STATUS_FAILURE          1

enum {
    FMAC_STA_IDLE,
    FMAC_STA_JOINING,
    FMAC_STA_AUTHENTICATING,
    FMAC_STA_ASSOCIATING,
    FMAC_STA_ASSOCIATED,
};

static uint32_t fmac_reg(SsvCore *c, uint32_t addr)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(c->regs,
                            GUINT_TO_POINTER(addr)));
}

/* The centre frequency of a 2.4 GHz channel, in MHz */
static uint16_t fmac_freq(uint8_t channel)
{
    return channel == 14 ? 2484 : 2407 + 5 * channel;
}

/* A unit for the host: its type, RX information and the content */
static void fmac_up(SsvCore *c, uint8_t type, uint32_t flags,
                    const uint8_t *data, size_t len)
{
    uint8_t h[RX_HDR_LEN] = {};
    uint8_t *ri = h + RX_HWIF_LEN;
    uint16_t freq = fmac_freq(c->ap.channel);
    GByteArray *u;

    stl_le_p(h, type | len << 8);
    stw_le_p(ri + RXI_FRMLEN, len);
    ri[RXI_RSSI_LEG] = RX_RSSI;
    ri[RXI_RSSI1] = RX_RSSI;
    stl_le_p(ri + RXI_PHY_INFO, freq << 16);
    stl_le_p(ri + RXI_PHY_INFO + 4, freq);
    stl_le_p(ri + RXI_FLAGS, flags);
    u = g_byte_array_sized_new(sizeof(h) + len);
    g_byte_array_append(u, h, sizeof(h));
    g_byte_array_append(u, data, len);
    g_queue_push_tail(&c->out, u);
    c->notify(c);
}

/* A confirmation or indication */
static void fmac_msg_up(SsvCore *c, uint16_t id, const void *param,
                        size_t len)
{
    g_autofree uint8_t *m = g_malloc0(E2A_HDR_LEN + len);

    stw_le_p(m, id);
    stw_le_p(m + 2, TASK_API);
    stw_le_p(m + 4, id >> 10);
    stw_le_p(m + 6, len);
    if (len) {
        memcpy(m + E2A_HDR_LEN, param, len);
    }
    trace_ssv_fmac_ind(id, len);
    fmac_up(c, IPC_MSG, 0, m, E2A_HDR_LEN + len);
}

/* An Ethernet frame for the host, from station @sta on interface @vif */
static void fmac_eth_up(SsvCore *c, uint8_t vif, uint8_t sta,
                        const uint8_t *da, const uint8_t *sa, uint16_t proto,
                        const uint8_t *payload, size_t len)
{
    g_autofree uint8_t *e = NULL;

    if (g_queue_get_length(&c->out) >= SSV_OUT_QUEUE_MAX) {
        return;
    }
    e = g_malloc(ETH_HLEN + len);
    memcpy(e, da, 6);
    memcpy(e + 6, sa, 6);
    stw_be_p(e + 12, proto);
    memcpy(e + ETH_HLEN, payload, len);
    trace_ssv_fmac_data(true, ETH_HLEN + len);
    fmac_up(c, IPC_DATA, RX_FLAGS(vif, sta), e, ETH_HLEN + len);
}

/*
 * The Ethernet frame an 802.11 data frame carries, LLC/SNAP and all: its
 * protocol, and where its payload starts. False for anything else.
 */
static bool fmac_unwrap(uint16_t fc, const uint8_t *f, size_t len,
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
static size_t fmac_wrap(uint8_t *f, uint16_t fc, const uint8_t *a1,
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

static bool fmac_vif_is(SsvCore *c, uint8_t vif, int type)
{
    return vif < SSV_FMAC_VIFS && c->fmac.vif_type[vif] == type;
}

static void fmac_set_sta(SsvCore *c, int state)
{
    if (c->fmac.sta != state) {
        trace_ssv_fmac_sta(c->fmac.sta, state);
    }
    c->fmac.sta = state;
}

/* To the BSS joined, from the joining interface */
static size_t fmac_sta_hdr(SsvCore *c, uint8_t *f, uint16_t fc)
{
    SsvFmac *m = &c->fmac;

    stw_le_p(f, fc);
    stw_le_p(f + 2, 0);
    memcpy(f + 4, m->bssid.a, 6);
    memcpy(f + 10, m->vif_mac[m->sta_vif].a, 6);
    memcpy(f + 16, m->bssid.a, 6);
    stw_le_p(f + 22, (m->seq++ & 0xfff) << 4);
    return WIFI_HDR_LEN;
}

/* The join's outcome: success with both sides' IEs, or a status */
static void fmac_connect_ind(SsvCore *c, uint16_t status)
{
    SsvFmac *m = &c->fmac;
    g_autofree uint8_t *p = g_malloc0(SM_CONNECT_IND_LEN);
    uint16_t freq = fmac_freq(c->ap.channel);
    size_t req = 0, resp = 0;

    stw_le_p(p, status);
    memcpy(p + 2, m->bssid.a, 6);
    p[9] = m->sta_vif;
    p[10] = STA_AP;
    if (!status) {
        req = MIN(m->req_ies_len, SM_ASSOC_IE_LEN);
        resp = MIN(m->resp_ies_len, SM_ASSOC_IE_LEN - req);
        memcpy(p + SM_CONNECT_IND_IES, m->req_ies, req);
        memcpy(p + SM_CONNECT_IND_IES + req, m->resp_ies, resp);
    }
    stw_le_p(p + 14, req);
    stw_le_p(p + 16, resp);
    stw_le_p(p + 18, m->aid);
    /* the operating channel: 2.4 GHz, 20 MHz wide, full power */
    stw_le_p(p + 22, freq);
    stw_le_p(p + 24, freq);
    p[28] = 20;
    fmac_msg_up(c, SM_CONNECT_IND, p, SM_CONNECT_IND_LEN);
}

/* The join failed: the host hears it with the last status */
static void fmac_join_fail(SsvCore *c, uint16_t status)
{
    timer_del(c->fmac.join);
    fmac_set_sta(c, FMAC_STA_IDLE);
    fmac_connect_ind(c, status ? status : STATUS_FAILURE);
}

/* Open authentication, then association, each sent until answered */
static void fmac_sta_send(SsvCore *c)
{
    SsvFmac *m = &c->fmac;
    uint8_t f[WIFI_HDR_LEN + 4 + sizeof(m->req_ies)];
    uint8_t *ies;
    size_t n;

    if (m->sta == FMAC_STA_AUTHENTICATING) {
        n = fmac_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_AUTH));
        /* open system, transaction 1 */
        stw_le_p(f + n, 0);
        stw_le_p(f + n + 2, 1);
        stw_le_p(f + n + 4, 0);
        n += 6;
    } else {
        n = fmac_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_ASSOC_REQ));
        stw_le_p(f + n, WIFI_CAP_ESS | WIFI_CAP_SHORT_PREAMBLE |
                        WIFI_CAP_SHORT_SLOT |
                        (c->ap.psk ? WIFI_CAP_PRIVACY : 0));
        stw_le_p(f + n + 2, LISTEN_INTERVAL);
        n += 4;
        ies = f + n;
        f[n++] = WIFI_EID_SSID;
        f[n++] = m->ssid_len;
        memcpy(f + n, m->ssid, m->ssid_len);
        n += m->ssid_len;
        memcpy(f + n, WIFI_RATES_IE, sizeof(WIFI_RATES_IE));
        n += sizeof(WIFI_RATES_IE);
        memcpy(f + n, WIFI_XRATES_IE, sizeof(WIFI_XRATES_IE));
        n += sizeof(WIFI_XRATES_IE);
        memcpy(f + n, m->ies, m->ies_len);
        n += m->ies_len;
        m->req_ies_len = f + n - ies;
        memcpy(m->req_ies, ies, m->req_ies_len);
    }
    timer_mod(m->join, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + RETRY_MS);
    wifi_ap_receive(&c->ap, f, n);
}

/* A join under way: start it, or send again what went unanswered */
static void fmac_join_tick(void *opaque)
{
    SsvCore *c = opaque;
    SsvFmac *m = &c->fmac;
    size_t n = strlen(c->ap.ssid);

    switch (m->sta) {
    case FMAC_STA_JOINING:
        if (memcmp(m->bssid.a, c->ap.bssid.a, 6) || m->ssid_len != n ||
            memcmp(m->ssid, c->ap.ssid, n)) {
            fmac_join_fail(c, STATUS_FAILURE);
            return;
        }
        m->tries = 0;
        fmac_set_sta(c, FMAC_STA_AUTHENTICATING);
        fmac_sta_send(c);
        break;
    case FMAC_STA_AUTHENTICATING:
    case FMAC_STA_ASSOCIATING:
        if (++m->tries >= TRIES) {
            fmac_join_fail(c, STATUS_FAILURE);
            return;
        }
        fmac_sta_send(c);
        break;
    }
}

/*
 * SM_CONNECT_REQ: the SSID, the BSSID (broadcast for any), and the IEs
 * the host wants in the association request
 */
static void fmac_connect(SsvCore *c, const uint8_t *p, size_t len)
{
    SsvFmac *m = &c->fmac;
    size_t ies;

    if (len < SM_CONNECT_REQ_IES) {
        return;
    }
    m->ssid_len = MIN(p[0], 32);
    memcpy(m->ssid, p + 1, m->ssid_len);
    if (is_broadcast_ether_addr(p + 33) || buffer_is_zero(p + 33, 6)) {
        memcpy(m->bssid.a, c->ap.bssid.a, 6);
    } else {
        memcpy(m->bssid.a, p + 33, 6);
    }
    m->sta_vif = p[55];
    ies = MIN(MIN(lduw_le_p(p + 56), len - SM_CONNECT_REQ_IES),
              sizeof(m->ies));
    memcpy(m->ies, p + SM_CONNECT_REQ_IES, ies);
    m->ies_len = ies;
    m->resp_ies_len = 0;
    fmac_set_sta(c, FMAC_STA_JOINING);
    timer_mod(m->join, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + JOIN_MS);
}

/* Out of the BSS: the access point told when it is the host's wish */
static void fmac_sta_leave(SsvCore *c, bool tell_ap, uint16_t reason)
{
    SsvFmac *m = &c->fmac;
    uint8_t f[WIFI_HDR_LEN + 2];
    uint8_t ind[4] = {};
    size_t n;

    timer_del(m->join);
    if (m->sta != FMAC_STA_ASSOCIATED) {
        if (m->sta != FMAC_STA_IDLE) {
            fmac_join_fail(c, STATUS_FAILURE);
        }
        return;
    }
    if (tell_ap) {
        n = fmac_sta_hdr(c, f, WIFI_FC_MGMT(WIFI_STYPE_DEAUTH));
        stw_le_p(f + n, reason);
        wifi_ap_receive(&c->ap, f, n + 2);
    }
    fmac_set_sta(c, FMAC_STA_IDLE);
    stw_le_p(ind, reason);
    ind[2] = m->sta_vif;
    fmac_msg_up(c, SM_DISCONNECT_IND, ind, sizeof(ind));
}

static void fmac_sta_mgmt(SsvCore *c, uint16_t fc, const uint8_t *f,
                          size_t len)
{
    SsvFmac *m = &c->fmac;
    const uint8_t *body = f + WIFI_HDR_LEN;
    size_t blen = len - WIFI_HDR_LEN;

    switch (WIFI_FC_STYPE(fc)) {
    case WIFI_STYPE_AUTH:
        /* algorithm, transaction, status */
        if (m->sta != FMAC_STA_AUTHENTICATING || blen < 6 ||
            lduw_le_p(body + 2) != 2) {
            return;
        }
        if (lduw_le_p(body + 4)) {
            fmac_join_fail(c, lduw_le_p(body + 4));
            return;
        }
        m->tries = 0;
        fmac_set_sta(c, FMAC_STA_ASSOCIATING);
        fmac_sta_send(c);
        return;
    case WIFI_STYPE_ASSOC_RESP:
    case WIFI_STYPE_REASSOC_RESP:
        /* capability, status, association id */
        if (m->sta != FMAC_STA_ASSOCIATING || blen < 6) {
            return;
        }
        if (lduw_le_p(body + 2)) {
            fmac_join_fail(c, lduw_le_p(body + 2));
            return;
        }
        timer_del(m->join);
        m->aid = lduw_le_p(body + 4) & 0x3fff;
        m->resp_ies_len = MIN(blen - 6, sizeof(m->resp_ies));
        memcpy(m->resp_ies, body + 6, m->resp_ies_len);
        fmac_set_sta(c, FMAC_STA_ASSOCIATED);
        fmac_connect_ind(c, 0);
        return;
    case WIFI_STYPE_DEAUTH:
    case WIFI_STYPE_DISASSOC:
        if (blen >= 2) {
            fmac_sta_leave(c, false, lduw_le_p(body));
        }
        return;
    }
}

/* A frame from the access point, its cipher taken off */
void ssv_fmac_deliver(void *opaque, const uint8_t *f, size_t len,
                      bool protected)
{
    SsvCore *c = opaque;
    SsvFmac *m = &c->fmac;
    uint16_t fc, proto;
    size_t off;

    if (len < WIFI_HDR_LEN || m->sta < FMAC_STA_AUTHENTICATING ||
        memcmp(f + 10, m->bssid.a, 6) ||
        (!is_multicast_ether_addr(f + 4) &&
         memcmp(f + 4, m->vif_mac[m->sta_vif].a, 6))) {
        return;
    }
    fc = lduw_le_p(f);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        fmac_sta_mgmt(c, fc, f, len);
        break;
    case WIFI_TYPE_DATA:
        /* from the distribution system: destination 1, source 3 */
        if (m->sta == FMAC_STA_ASSOCIATED && (fc & WIFI_FC_FROMDS) &&
            !(fc & WIFI_FC_TODS) && fmac_unwrap(fc, f, len, &proto, &off)) {
            fmac_eth_up(c, m->sta_vif, STA_AP, f + 4, f + 16, proto,
                        f + off, len - off);
        }
        break;
    }
}

/*
 * A frame from the client of the host's access point: management goes up
 * whole for the host to answer, data once the host has added the station
 */
void ssv_fmac_client_deliver(void *opaque, const uint8_t *f, size_t len,
                             bool protected)
{
    SsvCore *c = opaque;
    SsvFmac *m = &c->fmac;
    uint16_t fc, proto;
    size_t off;

    if (!c->bss_up || len < WIFI_HDR_LEN || memcmp(f + 4, c->bssid.a, 6)) {
        return;
    }
    fc = lduw_le_p(f);
    switch (WIFI_FC_TYPE(fc)) {
    case WIFI_TYPE_MGMT:
        if (g_queue_get_length(&c->out) < SSV_OUT_QUEUE_MAX) {
            fmac_up(c, IPC_DATA,
                    RX_FLAGS_MPDU | RX_FLAGS(m->ap_vif, STA_NONE), f, len);
        }
        break;
    case WIFI_TYPE_DATA:
        /* to the distribution system: destination 3, source 2 */
        if (m->peer_added && (fc & WIFI_FC_TODS) &&
            !(fc & WIFI_FC_FROMDS) && fmac_unwrap(fc, f, len, &proto, &off)) {
            fmac_eth_up(c, m->ap_vif, STA_PEER, f + 16, f + 10, proto,
                        f + off, len - off);
        }
        break;
    }
}

/* SCANU_START_REQ: the access point is found on its channel, by name */
static void fmac_scan(SsvCore *c, const uint8_t *p, size_t len)
{
    SsvFmac *m = &c->fmac;
    size_t n = strlen(c->ap.ssid);
    unsigned chans, ssids;
    bool chan = false, ssid = false;

    if (len < SCANU_REQ_LEN) {
        return;
    }
    m->scan_vif = p[SCANU_VIF];
    chans = p[SCANU_CHAN_CNT];
    ssids = p[SCANU_SSID_CNT];
    for (unsigned i = 0; i < MIN(chans, SCANU_SSIDS / 5); i++) {
        chan |= lduw_le_p(p + i * 5) == fmac_freq(c->ap.channel);
    }
    for (unsigned i = 0; i < MIN(ssids, 2); i++) {
        const uint8_t *s = p + SCANU_SSIDS + i * 33;

        ssid |= !s[0] || (s[0] == n && !memcmp(s + 1, c->ap.ssid, n));
    }
    m->scan_hit = (chan || !chans) && (ssid || !ssids);
    timer_mod(m->scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SCAN_MS);
}

/* The scan's end: the access point's beacon if it was heard */
static void fmac_scan_done(void *opaque)
{
    SsvCore *c = opaque;
    SsvFmac *m = &c->fmac;
    uint8_t r[10 + WIFI_AP_BEACON_MAX] = {};
    uint8_t cfm[3] = { m->scan_vif };
    size_t n;

    if (m->scan_hit) {
        n = wifi_ap_beacon(&c->ap, r + 10);
        stw_le_p(r, n);
        stw_le_p(r + 2, lduw_le_p(r + 10));
        stw_le_p(r + 4, fmac_freq(c->ap.channel));
        r[7] = STA_NONE;
        r[8] = m->scan_vif;
        r[9] = RX_RSSI;
        fmac_msg_up(c, SCANU_RESULT_IND, r, 10 + n);
        cfm[2] = 1;
    }
    fmac_msg_up(c, SCANU_START_CFM, cfm, sizeof(cfm));
}

/* An Ethernet frame from the host, for whichever BSS its interface is in */
static void fmac_data_tx(SsvCore *c, const uint8_t *hd, const uint8_t *eth,
                         size_t len)
{
    SsvFmac *m = &c->fmac;
    g_autofree uint8_t *f = NULL;
    uint8_t vif = hd[HD_VIF];
    size_t n;

    if (len < ETH_HLEN || lduw_be_p(eth + 12) < 0x0600) {
        return;
    }
    trace_ssv_fmac_data(false, len);
    f = g_malloc(WIFI_HDR_LEN + 6 + len);
    if (fmac_vif_is(c, vif, VIF_AP)) {
        if (!c->bss_up || !m->peer_added) {
            return;
        }
        n = fmac_wrap(f, WIFI_FC_DATA | WIFI_FC_FROMDS, eth, c->bssid.a,
                      eth + 6, m->seq++, eth, len);
        wifi_client_receive(&c->client, f, n);
    } else if (m->sta == FMAC_STA_ASSOCIATED && vif == m->sta_vif) {
        n = fmac_wrap(f, WIFI_FC_DATA | WIFI_FC_TODS, m->bssid.a,
                      m->vif_mac[vif].a, eth, m->seq++, eth, len);
        wifi_ap_receive(&c->ap, f, n);
    }
}

/*
 * A management frame the host built: to the client of its access point,
 * or the one joined; the host hears it was acked when it asks to
 */
static void fmac_mgmt_tx(SsvCore *c, const uint8_t *hd, const uint8_t *f,
                         size_t len)
{
    uint8_t ack[8 + HD_LEN];
    uint8_t vif = hd[HD_VIF];
    bool acked = false;

    if (len < WIFI_HDR_LEN) {
        return;
    }
    if (fmac_vif_is(c, vif, VIF_AP)) {
        if (c->bss_up && c->client_nic) {
            acked = wifi_client_acks(&c->client, f + 4);
            wifi_client_receive(&c->client, f, len);
        }
    } else {
        acked = c->fmac.sta == FMAC_STA_ASSOCIATED;
        wifi_ap_receive(&c->ap, f, len);
    }
    if (lduw_le_p(hd + HD_SW_ACK) & SW_ACK_WANTED) {
        stl_le_p(ack, PRIV_SW_ACK_IND);
        stl_le_p(ack + 4, HD_LEN);
        memcpy(ack + 8, hd, HD_LEN);
        if (acked) {
            stw_le_p(ack + 8 + HD_SW_ACK,
                     lduw_le_p(hd + HD_SW_ACK) | SW_ACK_ACKED);
        }
        fmac_up(c, IPC_PRIV_MSG, 0, ack, sizeof(ack));
    }
}

/* MM_ADD_IF_REQ: the type and the address; the index is the firmware's */
static void fmac_add_if(SsvCore *c, const uint8_t *p, size_t len,
                        uint8_t *cfm)
{
    SsvFmac *m = &c->fmac;
    unsigned i;

    cfm[0] = STATUS_FAILURE;
    if (len < 7) {
        return;
    }
    for (i = 0; i < SSV_FMAC_VIFS && m->vif_type[i] >= 0; i++) {
        continue;
    }
    if (i == SSV_FMAC_VIFS) {
        return;
    }
    m->vif_type[i] = p[0];
    memcpy(m->vif_mac[i].a, p + 1, 6);
    cfm[0] = 0;
    cfm[1] = i;
}

static void fmac_remove_if(SsvCore *c, uint8_t vif)
{
    SsvFmac *m = &c->fmac;

    if (vif >= SSV_FMAC_VIFS) {
        return;
    }
    if (vif == m->sta_vif && m->sta != FMAC_STA_IDLE) {
        fmac_sta_leave(c, true, 3);
    }
    if (vif == m->ap_vif && c->bss_up) {
        m->peer_added = false;
        ssv_core_set_bss(c, false);
    }
    m->vif_type[vif] = -1;
}

/* A request, and its confirmation */
static void fmac_msg(SsvCore *c, uint16_t id, const uint8_t *p, size_t len)
{
    SsvFmac *m = &c->fmac;
    uint8_t cfm[VERSION_CFM_LEN] = {};
    size_t n = 0;

    trace_ssv_fmac_req(id, len);
    switch (id) {
    case MM_RESET_REQ:
        fmac_sta_leave(c, true, 3);
        break;
    case MM_VERSION_REQ:
        stl_le_p(cfm, FW_LMAC_VERSION);
        stl_le_p(cfm + 12, PHY_KARST_1SS);
        stl_le_p(cfm + 20, MM_FEAT_UMAC);
        stw_le_p(cfm + 24, REMOTE_STA_MAX);
        cfm[26] = SSV_FMAC_VIFS;
        n = VERSION_CFM_LEN;
        break;
    case MM_ADD_IF_REQ:
        fmac_add_if(c, p, len, cfm);
        n = 2;
        break;
    case MM_REMOVE_IF_REQ:
        if (len >= 1) {
            fmac_remove_if(c, p[0]);
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
        if (len >= 39) {
            cfm[1] = p[38] ? 4 + p[1] : p[0];
        }
        n = 2;
        break;
    case ME_STA_ADD_REQ:
        if (len >= ME_STA_ADD_REQ_LEN && c->client_nic &&
            !memcmp(p, c->client_conf.macaddr.a, 6)) {
            m->peer_added = true;
        }
        cfm[0] = STA_PEER;
        n = 3;
        break;
    case ME_STA_DEL_REQ:
        if (len >= 1 && p[0] == STA_PEER) {
            m->peer_added = false;
        }
        break;
    case ME_RC_STATS_REQ: {
        uint8_t stats[ME_RC_STATS_CFM_LEN] = {};

        fmac_msg_up(c, id + 1, stats, sizeof(stats));
        return;
    }
    case SCANU_START_REQ:
        /* SCANU_START_CFM ends the scan instead */
        fmac_scan(c, p, len);
        return;
    case SM_CONNECT_REQ:
        fmac_connect(c, p, len);
        n = 1;
        break;
    case SM_DISCONNECT_REQ:
        fmac_msg_up(c, id + 1, NULL, 0);
        fmac_sta_leave(c, true, len >= 2 ? lduw_le_p(p) : 3);
        return;
    case APM_START_REQ:
        if (len < APM_START_REQ_LEN) {
            cfm[0] = STATUS_FAILURE;
        } else {
            m->ap_vif = p[40];
            ssv_core_beacon(c, p + APM_START_REQ_LEN,
                            MIN(lduw_le_p(p + 27), len - APM_START_REQ_LEN));
            cfm[1] = m->ap_vif;
            cfm[3] = STA_BCMC(m->ap_vif);
        }
        n = 4;
        fmac_msg_up(c, id + 1, cfm, n);
        if (!cfm[0]) {
            ssv_core_set_bss(c, c->bcn_valid);
        }
        return;
    case APM_STOP_REQ:
        m->peer_added = false;
        ssv_core_set_bss(c, false);
        break;
    case MM_BCN_CHANGE_REQ:
        if (len >= MM_BCN_CHANGE_REQ_LEN) {
            ssv_core_beacon(c, p + MM_BCN_CHANGE_REQ_LEN,
                            MIN(lduw_le_p(p + 4),
                                len - MM_BCN_CHANGE_REQ_LEN));
        }
        break;
    case MM_KEY_DEL_REQ:
    case MM_TIM_UPDATE_REQ:
    case ME_TRAFFIC_IND_REQ:
    case ME_RC_SET_RATE_REQ:
        /* these the host does not wait on */
        if (id != MM_KEY_DEL_REQ) {
            return;
        }
        break;
    }
    fmac_msg_up(c, id + 1, cfm, n);
}

/* One message or frame from the host, its headers and all */
static void fmac_tx(SsvCore *c, const uint8_t *b, size_t len)
{
    const uint8_t *hd = b + BMU_LEN + SDIO_HDR_LEN;
    const uint8_t *p = b + TX_HDR_LEN;
    size_t plen;

    if (len < TX_HDR_LEN) {
        return;
    }
    len = MIN(len, lduw_le_p(b + BMU_LEN));
    if (len < TX_HDR_LEN) {
        return;
    }
    plen = len - TX_HDR_LEN;
    switch (lduw_le_p(b + BMU_LEN + 2)) {
    case IPC_MSG:
        if (plen >= A2E_HDR_LEN) {
            fmac_msg(c, lduw_le_p(p + 4), p + A2E_HDR_LEN,
                     MIN(lduw_le_p(p + 10), plen - A2E_HDR_LEN));
        }
        break;
    case IPC_DATA:
        plen = MIN(plen, lduw_le_p(hd + HD_PACKET_LEN));
        if (lduw_le_p(hd + HD_FLAGS) & TXU_CNTRL_MGMT) {
            fmac_mgmt_tx(c, hd, p, plen);
        } else {
            fmac_data_tx(c, hd, p, plen);
        }
        break;
    }
}

void ssv_fmac_host(SsvCore *c, const uint8_t *buf, size_t len)
{
    size_t p = 0;

    if (!(fmac_reg(c, REG_HCI_CTRL) & HCI_TX_AGGR)) {
        fmac_tx(c, buf, len);
        return;
    }
    while (p + AGG_HDR_LEN <= len) {
        size_t unit = ldl_le_p(buf + p) & 0xffff;

        if (unit <= AGG_HDR_LEN || p + unit > len) {
            break;
        }
        fmac_tx(c, buf + p + AGG_HDR_LEN, unit - AGG_HDR_LEN);
        p += unit;
    }
}

/*
 * What is queued, aggregated into one transfer: units of a header and
 * the content out to a word, then a zero word. The first unit's spare
 * word has the total on top; one that fills whole 512-byte packets gets
 * another word, which the host drops.
 */
GByteArray *ssv_fmac_pop(SsvCore *c)
{
    static const uint8_t zero[4];
    bool was_full = g_queue_get_length(&c->out) >= SSV_OUT_QUEUE_MAX;
    GByteArray *t, *m;
    uint8_t h[AGG_HDR_LEN] = {};
    size_t total;

    if (g_queue_is_empty(&c->out)) {
        return NULL;
    }
    t = g_byte_array_new();
    while ((m = g_queue_peek_head(&c->out))) {
        size_t unit = AGG_HDR_LEN + ROUND_UP(m->len, 4);

        if (t->len && t->len + unit + sizeof(zero) > AGG_MAX) {
            break;
        }
        g_queue_pop_head(&c->out);
        stw_le_p(h, unit);
        stw_le_p(h + 2, unit);
        g_byte_array_append(t, h, sizeof(h));
        g_byte_array_append(t, m->data, m->len);
        g_byte_array_append(t, zero, -m->len & 3);
        g_byte_array_unref(m);
    }
    total = t->len + sizeof(zero);
    stl_le_p(t->data + 4, total << 16);
    g_byte_array_append(t, zero, sizeof(zero));
    if (total % 512 == 0) {
        g_byte_array_append(t, zero, sizeof(zero));
    }
    if (was_full) {
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        if (c->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(c->client_nic));
        }
    }
    return t;
}

uint32_t ssv_fmac_reg_read(SsvCore *c, uint32_t addr)
{
    const uint8_t *mac = c->conf.macaddr.a;

    switch (addr) {
    case REG_FW_VERSION:
        return FW_VERSION;
    case REG_FW_STATUS:
        return qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) / 10;
    case REG_TX_BMU_FREE:
        return TX_BMU_FREE_ALL;
    case REG_EFUSE_MAC_LO:
        return ldl_be_p(mac + 2);
    case REG_EFUSE_MAC_HI:
        return lduw_be_p(mac);
    }
    return fmac_reg(c, addr);
}

void ssv_fmac_reg_write(SsvCore *c, uint32_t addr, uint32_t val)
{
    g_hash_table_insert(c->regs, GUINT_TO_POINTER(addr),
                        GUINT_TO_POINTER(val));
}

void ssv_fmac_reset(SsvCore *c)
{
    SsvFmac *m = &c->fmac;

    timer_del(m->scan);
    timer_del(m->join);
    memset(m->vif_type, -1, sizeof(m->vif_type));
    m->sta = FMAC_STA_IDLE;
    m->sta_vif = 0;
    m->ap_vif = 0;
    m->peer_added = false;
}

void ssv_fmac_init(SsvCore *c)
{
    c->fmac.scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, fmac_scan_done, c);
    c->fmac.join = timer_new_ms(QEMU_CLOCK_VIRTUAL, fmac_join_tick, c);
}

void ssv_fmac_finalize(SsvCore *c)
{
    timer_free(c->fmac.scan);
    timer_free(c->fmac.join);
}
