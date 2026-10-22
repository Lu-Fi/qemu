/*
 * AltoBeam ATBM60xx Wi-Fi: the firmware model the bus front ends share
 *
 * The chip as its Linux driver (atbm-wifi hal_apollo, derived from
 * ST-Ericsson's cw1200) sees it once past the bus: a few registers of the
 * chip, and the WSM messages its firmware trades with the host. The
 * firmware the driver uploads is taken and dropped; this model answers in
 * its place, with an access point on the air (wifi-ap.c) to join.
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
#define WSM_START_SCAN_REQ      0x0007
#define WSM_STOP_SCAN_REQ       0x0008
#define WSM_CONFIGURATION_REQ   0x0009
#define WSM_JOIN_REQ            0x000b
#define WSM_ADD_KEY_REQ         0x000c
#define WSM_REMOVE_KEY_REQ      0x000d
#define WSM_SET_PM_REQ          0x0010
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

#define RATE_1                  0
#define RATE_54                 13
#define RSSI                    (-40)

#define KEY_TYPE_AES_GROUP      4
#define KEY_TYPE_AES_PAIRWISE   5

#define FC_PROTECTED            BIT(14)
#define CCMP_HDR                8
#define CCMP_MIC                8

/* indications the driver has not read yet before the network waits */
#define OUT_QUEUE_MAX           64

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
                    size_t len, bool protected)
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
        memcpy(f, frame, 24);
        stw_le_p(f, fc | FC_PROTECTED);
        f[24] = pn;
        f[25] = pn >> 8;
        /* ExtIV, then the key id */
        f[27] = 0x20 | (group ? c->gtk_id : 0) << 6;
        stl_le_p(f + 28, pn >> 16);
        memcpy(f + 24 + CCMP_HDR, frame + 24, len - 24);
        len += CCMP_HDR + CCMP_MIC;
        flags = RX_STATUS_AES | (group ? RX_STATUS_GROUP_KEY : 0);
    } else {
        memcpy(f, frame, len);
        if (fc == 0x0080) {
            flags = RX_STATUS_BEACON;
        }
    }
    stl_le_p(ind, status);
    stw_le_p(ind + 4, c->ap.channel);
    ind[6] = (fc & 0x0c) == 0x08 ? RATE_54 : RATE_1;
    ind[7] = RSSI;
    stl_le_p(ind + 8, flags);
    atbm_send(c, WSM_RECEIVE_IND | if_id << 6, ind, RX_DESC + len);
}

static void atbm_deliver(void *opaque, const uint8_t *frame, size_t len,
                         bool protected)
{
    AtbmCore *c = opaque;

    atbm_rx(c, c->join_if, frame, len, protected);
}

/*
 * A frame for the air. The driver wants to hear back about those it marks,
 * and whatever it wants encrypted comes with room for the CCMP header and
 * MIC, which go before the frame reaches the access point.
 */
static void atbm_tx(AtbmCore *c, uint16_t id, const uint8_t *body,
                    size_t len)
{
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
    if (flen < 24) {
        return;
    }
    fc = lduw_le_p(frame);
    trace_atbm_tx(ldl_le_p(body), fc, flen);
    /* no one else is out there to acknowledge it */
    if (!is_multicast_ether_addr(frame + 4) &&
        memcmp(frame + 4, c->ap.bssid.a, 6)) {
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
    if (fc & FC_PROTECTED) {
        hdr = 24 + ((fc & 0x8c) == 0x88 ? 2 : 0);
        if (flen < hdr + CCMP_HDR + CCMP_MIC) {
            return;
        }
        flen -= CCMP_HDR + CCMP_MIC;
        plain = g_malloc(flen);
        memcpy(plain, frame, hdr);
        stw_le_p(plain, fc & ~FC_PROTECTED);
        memcpy(plain + hdr, frame + hdr + CCMP_HDR, flen - hdr);
        frame = plain;
    }
    wifi_ap_receive(&c->ap, frame, flen);
}

static void atbm_scan_done(void *opaque)
{
    AtbmCore *c = opaque;
    /* status, power save mode, channels done, then busy ratios */
    uint8_t ind[4 + 2 + 14] = {};

    if (c->scan_hit) {
        uint8_t b[WIFI_AP_BEACON_MAX];

        atbm_rx(c, c->scan_if, b, wifi_ap_beacon(&c->ap, b), false);
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

void atbm_core_wsm(AtbmCore *c, uint16_t id, const uint8_t *body, size_t len)
{
    uint16_t cnf = (id & 0x03ff) | WSM_CNF;
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
        c->join_if = (id >> 6) & 0xf;
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

void atbm_core_init(AtbmCore *c)
{
    c->boot = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_startup, c);
    c->scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_scan_done, c);
    c->mem = g_hash_table_new(g_direct_hash, g_direct_equal);
    g_queue_init(&c->out);
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
    return true;
}

void atbm_core_unrealize(AtbmCore *c)
{
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
