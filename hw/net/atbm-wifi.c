/*
 * AltoBeam ATBM60xx Wi-Fi: the firmware model the bus front ends share
 *
 * The chip as its Linux driver (atbm-wifi hal_apollo, derived from
 * ST-Ericsson's cw1200) sees it once past the bus: a few registers of the
 * chip, and the WSM messages its firmware trades with the host. The
 * firmware the driver uploads is taken and dropped; this model answers in
 * its place, with an access point on the air (wifi-ap.c) to join and, when
 * the chip runs one of its own, a client (wifi-client.c) to take in.
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
#include "atbm-wifi.h"
#include "trace.h"
#include "wifi-frame.h"

#define CHIP_VERSION_ADDR       0x0acc017c
/* the Wi-Fi 6 drivers wait for it to read 5 before anything else */
#define BOOT_STATE_ADDR         0x16101028
#define BOOT_STATE_READY        5
#define EFUSE_INDEX_ADDR        0x16b00000
#define EFUSE_DATA_ADDR         0x16b00004
/* bit 8 holds the CPU, on the chips that start it this way (USB) */
#define CPU_CTRL_ADDR           0x16101000
#define CPU_HOLD                BIT(8)
/* the firmware's count of host messages taken, which paces SDIO */
#define XMITED_ADDR             0x09000100
/* ITCM and DTCM, where the firmware goes and nothing is read back */
#define FIRMWARE_END            0x01000000

#define WSM_CNF                 0x0400
#define WSM_FIRMWARE_CHECK      0x0002
#define WSM_GENERIC_REQ         0x0003
#define WSM_TX_REQ              0x0004
#define WSM_READ_MIB_REQ        0x0005
#define WSM_WRITE_MIB_REQ       0x0006
#define WSM_START_SCAN_REQ      0x0007
#define WSM_STOP_SCAN_REQ       0x0008
#define WSM_CONFIGURATION_REQ   0x0009
#define WSM_RESET_REQ           0x000a
#define WSM_JOIN_REQ            0x000b
#define WSM_ADD_KEY_REQ         0x000c
#define WSM_REMOVE_KEY_REQ      0x000d
#define WSM_SET_PM_REQ          0x0010
#define WSM_START_REQ           0x0017
#define WSM_MAP_LINK_REQ        0x001c
#define WSM_STARTUP_IND         0x0801
#define WSM_RECEIVE_IND         0x0804
#define WSM_SCAN_COMPLETE_IND   0x0806
#define WSM_SET_PM_COMPLETE_IND 0x0809

#define WSM_STATUS_FAILURE      1
#define WSM_STATUS_RETRY_EXCEEDED 6
#define WSM_STATUS_NO_KEY_FOUND 9
#define WSM_STATUS_NOEFFECT     14

/*
 * struct wsm_tx, after the message header: packet id, rate, queue, more,
 * flags, then the HT parameters and the frame. The Wi-Fi 6 drivers put a
 * rate set after the packet id.
 */
#define TX_RATE(c)              ((c)->wifi6 ? 8 : 4)
#define TX_FLAGS(c)             ((c)->wifi6 ? 11 : 7)
#define TX_HT_PARAMS(c)         ((c)->wifi6 ? 20 : 16)
#define TX_FRAME(c)             ((c)->wifi6 ? 24 : 20)
#define TX_2BYTES_SHIFT         BIT(7)
#define TX_NEED_CONFIRM         BIT(8)

/* struct wsm_rx, before the frame */
#define RX_DESC                 16
#define RX_STATUS_AES           3
#define RX_STATUS_BEACON        BIT(7)
#define RX_STATUS_GROUP_KEY     BIT(19)
/* the link the driver gave the sending station, if any */
#define RX_STATUS_LINK_SHIFT    25

#define START_MODE_AP           0
#define TEMPLATE_BEACON         1

#define RATE_1                  0
#define RATE_54                 13
#define RSSI                    (-40)

#define KEY_TYPE_AES_GROUP      4
#define KEY_TYPE_AES_PAIRWISE   5

#define CCMP_HDR                8
#define CCMP_MIC                8

/* indications the driver has not read yet before the network waits */
#define OUT_QUEUE_MAX           64

#define MIB_TEMPLATE_FRAME      0x1002
#define MIB_MAC_FROM_EFUSE      0x103a
#define MIB_EFUSE_DATA          0x103c

#define CAP_NO_CONFIRM          BIT(3)
#define CAP_VIFADDR_LOCAL_BIT   BIT(18)
/* the Wi-Fi 6 dies carry Bluetooth LE, which their drivers may insist on */
#define CAP_WIFI6_BLE           BIT(13)

#define NUM_INPUT_BUFS          16
#define INPUT_BUF_SIZE          1632

/* A message for the host: its length and id, then the body */
static void atbm_send(AtbmCore *c, uint16_t id, const void *body, size_t len)
{
    GByteArray *m = g_byte_array_sized_new(4 + len);
    uint8_t hdr[4];

    id |= (c->seq & 7) << 13;
    c->seq++;
    stw_le_p(&hdr[0], 4 + len);
    stw_le_p(&hdr[2], id);
    g_byte_array_append(m, hdr, sizeof(hdr));
    g_byte_array_append(m, body, len);
    trace_atbm_wsm_send(id, 4 + len);
    g_queue_push_tail(&c->out, m);
    c->notify(c);
}

GByteArray *atbm_core_peek(AtbmCore *c)
{
    return g_queue_peek_head(&c->out);
}

GByteArray *atbm_core_pop(AtbmCore *c)
{
    GByteArray *m = g_queue_pop_head(&c->out);

    if (m && g_queue_get_length(&c->out) == OUT_QUEUE_MAX - 1) {
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        if (c->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(c->client_nic));
        }
    }
    return m;
}

uint32_t atbm_core_read32(AtbmCore *c, uint32_t addr)
{
    switch (addr) {
    case CHIP_VERSION_ADDR:
        return c->chip_id;
    case BOOT_STATE_ADDR:
        return BOOT_STATE_READY;
    case EFUSE_DATA_ADDR:
        return c->efuse_byte < sizeof(c->efuse) ? c->efuse[c->efuse_byte] : 0;
    case XMITED_ADDR:
        return c->xmited;
    }
    return GPOINTER_TO_UINT(g_hash_table_lookup(c->mem,
                                                GUINT_TO_POINTER(addr)));
}

void atbm_core_write32(AtbmCore *c, uint32_t addr, uint32_t val)
{
    uint32_t old;

    if (addr < FIRMWARE_END) {
        return;
    }
    old = atbm_core_read32(c, addr);
    g_hash_table_insert(c->mem, GUINT_TO_POINTER(addr),
                        GUINT_TO_POINTER(val));
    switch (addr) {
    case EFUSE_INDEX_ADDR:
        c->efuse_byte = val >> 8;
        break;
    case CPU_CTRL_ADDR:
        if (!(old & CPU_HOLD) && (val & CPU_HOLD)) {
            atbm_core_hold(c);
        } else if ((old & CPU_HOLD) && !(val & CPU_HOLD)) {
            atbm_core_run(c);
        }
        break;
    }
}

static void atbm_startup(void *opaque)
{
    AtbmCore *c = opaque;
    uint8_t ind[20 + 128 + 16 + 8] = {};
    uint32_t cap = CAP_NO_CONFIRM | CAP_VIFADDR_LOCAL_BIT |
                   (c->wifi6 ? CAP_WIFI6_BLE : 0);
    uint8_t *p = ind;

    stw_le_p(p, NUM_INPUT_BUFS);
    stw_le_p(p + 2, INPUT_BUF_SIZE);
    /* hardware id and sub id, then status 0 */
    stw_le_p(p + 10, cap);
    /* firmware type WFM, API version, build, version */
    stw_le_p(p + 12, 1);
    stw_le_p(p + 14, 0x0100);
    stw_le_p(p + 16, 1);
    stw_le_p(p + 18, 15000);
    p += 20;
    pstrcpy((char *)p, 128, "QEMU model of the ATBM60xx firmware");
    p += 128;
    /* 8 stations, 8 aggregates each way, 2 interfaces (16 bits on Wi-Fi 6) */
    stl_le_p(p, 8 | 8 << 8 | 2 << 16 | (c->wifi6 ? 0 : 8u << 24));
    stl_le_p(p + 12, XMITED_ADDR);
    p += 16;
    stw_le_p(p, cap >> 16);
    atbm_send(c, WSM_STARTUP_IND, ind, sizeof(ind));
}

/* Tell the client whether the chip has an access point up to join */
static void atbm_ap_update(AtbmCore *c)
{
    int i = c->ap_if;

    if (!c->client_nic) {
        return;
    }
    if (i >= 0 && c->beacon[i]) {
        wifi_client_bss(&c->client, c->beacon_bssid[i].a, c->ap_ssid,
                        c->ap_ssid_len, NULL);
    } else {
        wifi_client_bss(&c->client, NULL, NULL, 0, NULL);
    }
}

static void atbm_ap_down(AtbmCore *c)
{
    c->ap_if = -1;
    memset(c->link, 0, sizeof(c->link));
    atbm_ap_update(c);
}

void atbm_core_hold(AtbmCore *c)
{
    GByteArray *m;

    timer_del(c->boot);
    timer_del(c->scan);
    while ((m = g_queue_pop_head(&c->out))) {
        g_byte_array_unref(m);
    }
    c->xmited = 0;
    c->seq = 0;
    c->scan_hit = false;
    c->ptk_set = false;
    c->gtk_set = false;
    if (c->ap.eapol_timer) {
        wifi_ap_reset(&c->ap);
    }
    memset(c->beacon, 0, sizeof(c->beacon));
    atbm_ap_down(c);
    c->notify(c);
}

void atbm_core_run(AtbmCore *c)
{
    timer_mod(c->boot, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 20);
}

static void atbm_confirm(AtbmCore *c, uint16_t cnf, uint32_t status,
                         const void *data, size_t len)
{
    g_autofree uint8_t *body = g_malloc0(4 + len);

    stl_le_p(body, status);
    if (len) {
        memcpy(body + 4, data, len);
    }
    atbm_send(c, cnf, body, 4 + len);
}

static void atbm_read_mib(AtbmCore *c, uint16_t cnf, uint16_t mib)
{
    uint8_t mibcnf[4 + 64] = {};
    const uint8_t *mac = c->conf.macaddr.a;

    stw_le_p(&mibcnf[0], mib);
    stw_le_p(&mibcnf[2], 64);
    switch (mib) {
    case MIB_MAC_FROM_EFUSE:
        memcpy(&mibcnf[4], mac, 6);
        break;
    case MIB_EFUSE_DATA:
        /* struct efuse_headr: nine bytes of trims, then the MAC */
        memcpy(&mibcnf[4 + 9], mac, 6);
        break;
    }
    atbm_confirm(c, cnf, 0, mibcnf, sizeof(mibcnf));
}

/*
 * A frame off the air, in the shape the hardware leaves it: one that came
 * encrypted keeps its CCMP header and MIC, now over the plaintext
 */
static void atbm_rx(AtbmCore *c, uint8_t if_id, const uint8_t *frame,
                    size_t len, bool protected, unsigned link)
{
    g_autofree uint8_t *ind = g_malloc0(RX_DESC + len + CCMP_HDR + CCMP_MIC);
    uint16_t fc = lduw_le_p(frame);
    uint8_t *f = ind + RX_DESC;
    uint32_t status = 0, flags = 0;

    if (protected) {
        bool group = is_multicast_ether_addr(frame + 4);
        uint64_t pn = group ? ++c->gtk_pn : ++c->ptk_pn;

        if (!(group ? c->gtk_set : c->ptk_set)) {
            status = WSM_STATUS_NO_KEY_FOUND;
        }
        memcpy(f, frame, WIFI_HDR_LEN);
        stw_le_p(f, fc | WIFI_FC_PROTECTED);
        f[WIFI_HDR_LEN] = pn;
        f[WIFI_HDR_LEN + 1] = pn >> 8;
        /* ExtIV, then the key id */
        f[WIFI_HDR_LEN + 3] = 0x20 | (group ? c->gtk_id : 0) << 6;
        stl_le_p(f + WIFI_HDR_LEN + 4, pn >> 16);
        memcpy(f + WIFI_HDR_LEN + CCMP_HDR, frame + WIFI_HDR_LEN,
               len - WIFI_HDR_LEN);
        len += CCMP_HDR + CCMP_MIC;
        flags = RX_STATUS_AES | (group ? RX_STATUS_GROUP_KEY : 0);
    } else {
        memcpy(f, frame, len);
        if (fc == WIFI_FC_MGMT(WIFI_STYPE_BEACON)) {
            flags = RX_STATUS_BEACON;
        }
    }
    stl_le_p(ind, status);
    stw_le_p(ind + 4, if_id == c->ap_if ? c->ap_channel : c->ap.channel);
    ind[6] = WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA ? RATE_54 : RATE_1;
    ind[7] = RSSI;
    stl_le_p(ind + 8, flags | link << RX_STATUS_LINK_SHIFT);
    atbm_send(c, WSM_RECEIVE_IND | if_id << 6, ind, RX_DESC + len);
}

static void atbm_deliver(void *opaque, const uint8_t *frame, size_t len,
                         bool protected)
{
    AtbmCore *c = opaque;

    atbm_rx(c, c->join_if, frame, len, protected, 0);
}

static unsigned atbm_link_of(AtbmCore *c, const uint8_t *addr)
{
    for (unsigned i = 0; i < ATBM_LINKS; i++) {
        if (!memcmp(c->link[i].a, addr, 6)) {
            return i + 1;
        }
    }
    return 0;
}

static void atbm_client_deliver(void *opaque, const uint8_t *frame,
                                size_t len, bool protected)
{
    AtbmCore *c = opaque;

    if (c->ap_if >= 0) {
        atbm_rx(c, c->ap_if, frame, len, protected,
                atbm_link_of(c, frame + 10));
    }
}

/*
 * A frame for the air: from the chip's own access point to its client,
 * else to the access point it joined. The driver wants to hear back about
 * those it marks, and whatever it wants encrypted comes with room for the
 * CCMP header and MIC, which go before the frame is passed on.
 */
static void atbm_tx(AtbmCore *c, uint16_t id, const uint8_t *body,
                    size_t len)
{
    bool from_ap = (int)((id >> 6) & 0xf) == c->ap_if;
    size_t off;
    const uint8_t *frame;
    g_autofree uint8_t *plain = NULL;
    size_t flen;
    uint32_t status = 0;
    uint16_t fc;
    size_t hdr;

    if (len < TX_FRAME(c)) {
        return;
    }
    off = TX_FRAME(c) + (body[TX_FLAGS(c)] & TX_2BYTES_SHIFT ? 2 : 0);
    frame = body + off;
    flen = len > off ? len - off : 0;
    if (flen < WIFI_HDR_LEN) {
        return;
    }
    fc = lduw_le_p(frame);
    trace_atbm_tx(ldl_le_p(body), fc, flen);
    /* no one else is out there to acknowledge it */
    if (!is_multicast_ether_addr(frame + 4) &&
        !(from_ap ? wifi_client_acks(&c->client, frame + 4)
                  : !memcmp(frame + 4, c->ap.bssid.a, 6))) {
        status = WSM_STATUS_RETRY_EXCEEDED;
    }
    if (ldl_le_p(body + TX_HT_PARAMS(c)) & TX_NEED_CONFIRM) {
        uint8_t cnf[20] = {};

        stl_le_p(cnf, ldl_le_p(body));
        stl_le_p(cnf + 4, status);
        cnf[8] = body[TX_RATE(c)];
        atbm_send(c, (id & 0x03ff) | WSM_CNF, cnf, sizeof(cnf));
    }
    if (status) {
        return;
    }
    if (fc & WIFI_FC_PROTECTED) {
        hdr = WIFI_HDR_LEN;
        if (WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA &&
            (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS)) {
            hdr += 2;
        }
        if (flen < hdr + CCMP_HDR + CCMP_MIC) {
            return;
        }
        flen -= CCMP_HDR + CCMP_MIC;
        plain = g_malloc(flen);
        memcpy(plain, frame, hdr);
        stw_le_p(plain, fc & ~WIFI_FC_PROTECTED);
        memcpy(plain + hdr, frame + hdr + CCMP_HDR, flen - hdr);
        frame = plain;
    }
    if (from_ap) {
        wifi_client_receive(&c->client, frame, flen);
    } else {
        wifi_ap_receive(&c->ap, frame, flen);
    }
}

static void atbm_scan_done(void *opaque)
{
    AtbmCore *c = opaque;
    /* status, power save mode, channels done, then busy ratios */
    uint8_t ind[4 + 2 + 14] = {};

    if (c->scan_hit) {
        uint8_t b[WIFI_AP_BEACON_MAX];

        atbm_rx(c, c->scan_if, b, wifi_ap_beacon(&c->ap, b), false, 0);
    }
    ind[5] = c->scan_channels;
    atbm_send(c, WSM_SCAN_COMPLETE_IND, ind, sizeof(ind));
}

/*
 * The band, then the channel list after 12 bytes of parameters, 16 bytes a
 * channel; the access point is on 2.4 GHz
 */
static void atbm_scan(AtbmCore *c, uint16_t id, const uint8_t *b, size_t len)
{
    unsigned n = len >= 12 ? b[9] : 0;

    c->scan_if = (id >> 6) & 0xf;
    c->scan_channels = n;
    c->scan_hit = false;
    for (unsigned i = 0; i < n && 12 + 16 * (i + 1) <= len; i++) {
        if (b[0] == 0 && lduw_le_p(b + 12 + 16 * i) == c->ap.channel) {
            c->scan_hit = true;
        }
    }
    timer_mod(c->scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                       10 * MAX(n, 1));
}

/* struct wsm_add_key: type, index, two reserved, then by type */
static void atbm_key(AtbmCore *c, bool add, const uint8_t *b, size_t len)
{
    if (len < 4) {
        return;
    }
    switch (b[0]) {
    case KEY_TYPE_AES_PAIRWISE:
        c->ptk_set = add;
        c->ptk_pn = 0;
        break;
    case KEY_TYPE_AES_GROUP:
        c->gtk_set = add;
        c->gtk_id = len > 20 ? b[20] & 3 : 0;
        c->gtk_pn = 0;
        break;
    }
}

/*
 * struct wsm_start: mode, band, channel, CT window, beacon interval, DTIM
 * period, preamble, probe delay, then the SSID's length and 32 bytes
 */
static void atbm_start(AtbmCore *c, unsigned if_id, const uint8_t *b,
                       size_t len)
{
    if (if_id >= ATBM_IFS || len < 48 || b[0] != START_MODE_AP || b[1]) {
        if ((int)if_id == c->ap_if) {
            atbm_ap_down(c);
        }
        return;
    }
    c->ap_if = if_id;
    c->ap_channel = lduw_le_p(b + 2);
    c->ap_ssid_len = MIN(b[15], sizeof(c->ap_ssid));
    memcpy(c->ap_ssid, b + 16, c->ap_ssid_len);
    atbm_ap_update(c);
}

/*
 * The MIB id and length, then a template's type, rate and length ahead of
 * the frame; the beacon's tells the BSSID, and an empty one withdraws it
 */
static void atbm_write_mib(AtbmCore *c, unsigned if_id, const uint8_t *b,
                           size_t len)
{
    size_t flen;

    if (if_id >= ATBM_IFS || len < 8 ||
        lduw_le_p(b) != MIB_TEMPLATE_FRAME || b[4] != TEMPLATE_BEACON) {
        return;
    }
    flen = MIN(lduw_le_p(b + 6), len - 8);
    c->beacon[if_id] = flen >= WIFI_HDR_LEN;
    if (c->beacon[if_id]) {
        memcpy(c->beacon_bssid[if_id].a, b + 8 + 16, 6);
    }
    if ((int)if_id == c->ap_if) {
        atbm_ap_update(c);
    }
}

/* struct wsm_map_link: the station's address, unmap, then the link id */
static void atbm_map_link(AtbmCore *c, const uint8_t *b, size_t len)
{
    if (len < 8 || b[7] < 1 || b[7] > ATBM_LINKS) {
        return;
    }
    if (b[6]) {
        memset(&c->link[b[7] - 1], 0, sizeof(c->link[0]));
    } else {
        memcpy(c->link[b[7] - 1].a, b, 6);
    }
}

void atbm_core_wsm(AtbmCore *c, uint16_t id, const uint8_t *body, size_t len)
{
    uint16_t cnf = (id & 0x03ff) | WSM_CNF;
    unsigned if_id = (id >> 6) & 0xf;
    uint8_t zeros[256] = {};
    uint8_t b[40] = {};
    bool ok;

    trace_atbm_wsm_take(id, len);
    c->xmited++;
    switch (id & 0x3f) {
    case WSM_FIRMWARE_CHECK:
        /* the driver expects nothing back */
        return;
    case WSM_TX_REQ:
        atbm_tx(c, id, body, len);
        return;
    case WSM_START_SCAN_REQ:
        atbm_confirm(c, cnf, 0, NULL, 0);
        atbm_scan(c, id, body, len);
        return;
    case WSM_STOP_SCAN_REQ:
        if (!timer_pending(c->scan)) {
            atbm_confirm(c, cnf, WSM_STATUS_NOEFFECT, NULL, 0);
            return;
        }
        atbm_confirm(c, cnf, 0, NULL, 0);
        timer_del(c->scan);
        c->scan_hit = false;
        atbm_scan_done(c);
        return;
    case WSM_JOIN_REQ:
        /* band, channel, BSSID; the power range in 0.1 dBm */
        ok = len >= 10 && body[1] == 0 &&
             lduw_le_p(body + 2) == c->ap.channel &&
             !memcmp(body + 4, c->ap.bssid.a, 6);
        c->join_if = if_id;
        if ((int)if_id == c->ap_if) {
            atbm_ap_down(c);
        }
        stl_le_p(&b[4], 200);
        atbm_confirm(c, cnf, ok ? 0 : WSM_STATUS_FAILURE, b, 8);
        return;
    case WSM_SET_PM_REQ:
        /* the mode is in once the confirm is out */
        atbm_confirm(c, cnf, 0, NULL, 0);
        b[0] = len ? body[0] : 0;
        atbm_send(c, WSM_SET_PM_COMPLETE_IND, b, 4);
        return;
    case WSM_ADD_KEY_REQ:
    case WSM_REMOVE_KEY_REQ:
        atbm_key(c, (id & 0x3f) == WSM_ADD_KEY_REQ, body, len);
        break;
    case WSM_READ_MIB_REQ:
        atbm_read_mib(c, cnf, len >= 2 ? lduw_le_p(body) : 0);
        return;
    case WSM_WRITE_MIB_REQ:
        atbm_write_mib(c, if_id, body, len);
        break;
    case WSM_START_REQ:
        atbm_start(c, if_id, body, len);
        break;
    case WSM_MAP_LINK_REQ:
        atbm_map_link(c, body, len);
        break;
    case WSM_RESET_REQ:
        /* the driver resets link 0 alone: the whole interface */
        if ((int)if_id == c->ap_if) {
            atbm_ap_down(c);
        }
        break;
    case WSM_CONFIGURATION_REQ:
        /* the MAC, 2.4 GHz only, the 802.11b/g rates, 0-20 dBm */
        memcpy(&b[0], c->conf.macaddr.a, 6);
        b[6] = 1;
        stl_le_p(&b[8], 0x3fff);
        stl_le_p(&b[16], 200);
        stl_le_p(&b[20], 10);
        stl_le_p(&b[28], 200);
        stl_le_p(&b[32], 10);
        atbm_confirm(c, cnf, 0, b, 36);
        return;
    case WSM_GENERIC_REQ:
        /* echo the request id; status and payload follow */
        memcpy(zeros, body, MIN(len, 4));
        atbm_send(c, cnf, zeros, sizeof(zeros));
        return;
    }
    atbm_confirm(c, cnf, 0, zeros, 64);
}

static bool atbm_can_receive(NetClientState *nc)
{
    AtbmCore *c = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&c->out) < OUT_QUEUE_MAX;
}

static ssize_t atbm_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    AtbmCore *c = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&c->ap, buf, size);
    return size;
}

static NetClientInfo atbm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = atbm_can_receive,
    .receive = atbm_receive,
};

static bool atbm_client_can_receive(NetClientState *nc)
{
    AtbmCore *c = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&c->client) &&
           g_queue_get_length(&c->out) < OUT_QUEUE_MAX;
}

static ssize_t atbm_client_receive(NetClientState *nc, const uint8_t *buf,
                                   size_t size)
{
    AtbmCore *c = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&c->client, buf, size);
    return size;
}

static NetClientInfo atbm_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = atbm_client_can_receive,
    .receive = atbm_client_receive,
};

void atbm_core_init(AtbmCore *c)
{
    c->boot = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_startup, c);
    c->scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_scan_done, c);
    c->mem = g_hash_table_new(g_direct_hash, g_direct_equal);
    g_queue_init(&c->out);
    c->ap_if = -1;
}

bool atbm_core_realize(AtbmCore *c, Error **errp)
{
    DeviceState *dev = c->dev;

    qemu_macaddr_default_if_unset(&c->conf.macaddr);
    c->nic = qemu_new_nic(&atbm_net_info, &c->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, c);
    if (!wifi_ap_realize(&c->ap, c->nic, atbm_deliver, c, errp)) {
        qemu_del_nic(c->nic);
        c->nic = NULL;
        return false;
    }
    if (c->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&c->client_conf.macaddr);
        c->client_nic = qemu_new_nic(&atbm_client_net_info, &c->client_conf,
                                     object_get_typename(OBJECT(dev)), NULL,
                                     &dev->mem_reentrancy_guard, c);
        wifi_client_init(&c->client, c->client_nic, &c->client_conf.macaddr,
                         atbm_client_deliver, c);
    }
    return true;
}

void atbm_core_unrealize(AtbmCore *c)
{
    if (c->client_nic) {
        wifi_client_cleanup(&c->client);
        qemu_del_nic(c->client_nic);
        c->client_nic = NULL;
    }
    wifi_ap_unrealize(&c->ap);
    qemu_del_nic(c->nic);
}

void atbm_core_finalize(AtbmCore *c)
{
    GByteArray *m;

    timer_free(c->boot);
    timer_free(c->scan);
    while ((m = g_queue_pop_head(&c->out))) {
        g_byte_array_unref(m);
    }
    g_hash_table_destroy(c->mem);
}
