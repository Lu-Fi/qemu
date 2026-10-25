/*
 * WuQi WQ9001 Wi-Fi on USB
 *
 * The chip as its vendor driver (wq9001) sees it: a RISC-V CPU running
 * firmware the driver downloads, which this model takes, drops and
 * answers in place of. Bulk OUT 1 carries the firmware, then frames; OUT 3
 * the driver's commands and frames, OUT 4 and 5 frames. Everything back
 * comes on bulk IN 1: command answers, events and received frames, each
 * behind a 12-byte IPC header. The host runs the 802.11 stack (a net80211
 * port) and hands the chip whole frames, which the chip encrypts and
 * decrypts. An access point (wifi-ap.c) is on the air to join, and a
 * client (wifi-client.c) joins the driver's own.
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
#include "hw/usb/usb.h"
#include "hw/usb/desc.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "wifi-ccmp.h"
#include "wifi-frame.h"
#include "trace.h"

#define TYPE_WQ9001 "wq9001"
OBJECT_DECLARE_SIMPLE_TYPE(Wq9001State, WQ9001)

/* vendor requests */
#define VREQ_FW_DL              0xaa
#define VREQ_FW_DL_COMP         0xab
#define VREQ_GET_DSTATE         0xba
#define VREQ_GET_ROM_VER        0xca
#define VREQ_SET_RESET          0xda
#define DSTATE_TRANSFER         0
#define DSTATE_READY            1

/* bulk endpoints: OUT 1, 3, 4, 5 and IN 1; the interrupt pair 2 is idle */
#define EP_DATA                 1
#define EP_INT                  2
#define EP_OUTS                 4
static const uint8_t wq_ep_out[EP_OUTS] = { 1, 3, 4, 5 };

/* a firmware chunk is 10232 bytes and a 4-byte tag */
#define OUT_MAX                 (16 * 1024)

/* the IPC header ahead of every unit either way */
#define IPC_HDR_LEN             12
#define IPC_TYPE(f)             extract32(f, 0, 2)
#define IPC_RCV(f)              extract32(f, 2, 2)
#define IPC_SEQ(f)              extract32(f, 16, 16)
#define IPC_TYPE_CMD            1
#define IPC_TYPE_EVT            2
#define IPC_TYPE_PKT            3
#define EVT_FWREADY             0
#define EVT_ACK_RETURN          4

/* a command: the message descriptor, then its struct */
#define MSG_DESC_LEN            8
#define MSG_REG_READ            0
#define MSG_REG_WRITE           1
#define MSG_BCN_UPDATE          3
#define MSG_CH_BW_NOTIFY        7
#define MSG_NODE_NOTIFY         8
#define MSG_MVAP_NOTIFY         10
#define MSG_FW_INFO             12
#define MSG_MP_GET_NOTIFY       16
#define MSG_AMPDU_AGGR_NOTIFY   17
/* an answer: the confirm descriptor, then its struct */
#define CFM_DESC_LEN            4
#define CFM_SUCC                0
#define NODE_ALLOC              0
#define NODE_FREE               1
#define MVAP_FREE               1
#define AMPDU_AGG_GET           0
#define NODE_INVALID            0xff
#define NODES                   8
/* the IPC and hardware versions FW_INFO gives: IPC 0.1 */
#define FW_IPC_VER              1
#define FW_HW_VER               3

/* struct beacon_info: macid, mnid, length, the frame */
#define BCN_DATA                4
#define BCN_MAX                 470

/* struct mp_info as MIPS32 lays it out: the answer length at 30 */
#define MP_RSP_LEN              30

/* the data descriptor ahead of each frame, 24 bytes */
#define DATA_DESC_LEN           24
#define DESC_STA_IDX            0
#define DESC_LEN                2
#define DESC_INFO0              8
#define DESC_INFO1              12
#define TX_IS_PROTECT           BIT(18)

#define RX_QUEUE_MAX            64
#define BEACON_MS               102
/* RSSI -40 dBm over a -95 dBm noise floor */
#define RX_RSSI                 -40
#define RX_NF                   -95

struct Wq9001State {
    USBDevice parent_obj;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;
    QEMUTimer *beacon;

    bool fw_running;
    GHashTable *regs;
    uint8_t nodes;              /* the node ids handed out, a bit each */
    uint8_t channel;
    uint16_t pkt_seq;
    uint64_t ptk_pn, gtk_pn;    /* the AP's packet numbers, as sent */

    /* the driver's own BSS, as its beacon names it */
    bool bcn;
    uint8_t bcn_macid;
    MACAddr bcn_bssid;
    uint8_t bcn_ssid[32];
    uint8_t bcn_ssid_len;

    GByteArray *out[EP_OUTS];
    GQueue in;                  /* units for bulk IN 1 */
    GByteArray *cur;            /* the one it is giving */
    size_t cur_off;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings wq_strings = {
    [STR_MANUFACTURER]  = "WuQi",
    [STR_PRODUCT]       = "WQ9001",
    [STR_SERIALNUMBER]  = "1.0",
};

/* in the order the driver's probe checks them */
#define WQ_EPS(_max)                                                        \
    (USBDescEndpoint[]) {                                                   \
        {                                                                   \
            .bEndpointAddress   = USB_DIR_IN | EP_DATA,                     \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_OUT | EP_DATA,                    \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_IN | EP_INT,                      \
            .bmAttributes       = USB_ENDPOINT_XFER_INT,                    \
            .wMaxPacketSize     = 64,                                       \
            .bInterval          = 1,                                        \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_OUT | EP_INT,                     \
            .bmAttributes       = USB_ENDPOINT_XFER_INT,                    \
            .wMaxPacketSize     = 64,                                       \
            .bInterval          = 1,                                        \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_OUT | 3,                          \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_OUT | 4,                          \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_OUT | 5,                          \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        },                                                                  \
    }

static const USBDescIface wq_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 7,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = WQ_EPS(64),
};

static const USBDescIface wq_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 7,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = WQ_EPS(512),
};

static const USBDescDevice wq_device_full = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 250,
            .nif = 1,
            .ifs = &wq_iface_full,
        },
    },
};

static const USBDescDevice wq_device_high = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 250,
            .nif = 1,
            .ifs = &wq_iface_high,
        },
    },
};

static const USBDesc wq_desc = {
    .id = {
        .idVendor          = 0x0ffe,
        .idProduct         = 0x0001,
        .bcdDevice         = 0x0100,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &wq_device_full,
    .high = &wq_device_high,
    .str  = wq_strings,
};

/* A unit for bulk IN 1: the IPC header, then @len bytes of @data */
static GByteArray *wq_unit(unsigned type, unsigned rcv, unsigned seq,
                           uint32_t id, const uint8_t *data, size_t len)
{
    GByteArray *u = g_byte_array_sized_new(IPC_HDR_LEN + len);
    uint8_t h[IPC_HDR_LEN];

    stl_le_p(h, type | rcv << 2 | seq << 16);
    stl_le_p(h + 4, id);
    stl_le_p(h + 8, len);
    g_byte_array_append(u, h, sizeof(h));
    if (len) {
        g_byte_array_append(u, data, len);
    }
    return u;
}

static void wq_event(Wq9001State *s, uint32_t id, const uint8_t *data,
                     size_t len)
{
    trace_wq9001_event(id, len);
    g_queue_push_tail(&s->in, wq_unit(IPC_TYPE_EVT, 0, 0, id, data, len));
}

/*
 * A frame off the air, as the chip hands it over: one that came encrypted
 * has been decrypted, keeping its CCMP header and a MIC's room behind it,
 * which net80211 strips once it has checked the packet number
 */
static void wq_rx_frame(Wq9001State *s, const uint8_t *f, size_t len,
                        bool protected)
{
    size_t hdr, mpdu = len + (protected ? WIFI_CCMP_HDR_LEN +
                                          WIFI_CCMP_MIC_LEN : 0);
    g_autofree uint8_t *d = NULL;
    uint8_t *m;
    uint16_t fc;

    if (len < WIFI_HDR_LEN - 6 ||
        g_queue_get_length(&s->in) >= RX_QUEUE_MAX) {
        return;
    }
    fc = lduw_le_p(f);
    hdr = MIN(wifi_hdr_len(fc), len);
    d = g_malloc0(DATA_DESC_LEN + mpdu);
    stw_le_p(d + DESC_LEN, mpdu);
    stl_le_p(d + DESC_INFO0, (uint16_t)RX_NF | (uint16_t)RX_RSSI << 16);
    m = d + DATA_DESC_LEN;
    if (protected) {
        bool group = is_multicast_ether_addr(f + 4);
        uint64_t pn = group ? ++s->gtk_pn : ++s->ptk_pn;
        uint8_t *ccmp = m + hdr;

        memcpy(m, f, hdr);
        stw_le_p(m, fc | WIFI_FC_PROTECTED);
        ccmp[0] = pn;
        ccmp[1] = pn >> 8;
        /* ExtIV, then the key id */
        ccmp[3] = 0x20 | (group ? WIFI_AP_GTK_ID : 0) << 6;
        stl_le_p(ccmp + 4, pn >> 16);
        memcpy(ccmp + WIFI_CCMP_HDR_LEN, f + hdr, len - hdr);
    } else {
        memcpy(m, f, len);
    }
    trace_wq9001_rx(fc, len);
    g_queue_push_tail(&s->in, wq_unit(IPC_TYPE_PKT, 0, ++s->pkt_seq, 0, d,
                                      DATA_DESC_LEN + mpdu));
}

/* From the access point, heard on its channel */
static void wq_deliver(void *opaque, const uint8_t *frame, size_t len,
                       bool protected)
{
    Wq9001State *s = opaque;

    if (s->fw_running && s->channel == s->ap.channel) {
        wq_rx_frame(s, frame, len, protected);
    }
}

static void wq_beacon_tick(void *opaque)
{
    Wq9001State *s = opaque;
    uint8_t b[WIFI_AP_BEACON_MAX];

    wq_deliver(s, b, wifi_ap_beacon(&s->ap, b), false);
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

/*
 * The driver's own BSS from the beacon it hands the chip: the client joins
 * what it names
 */
static void wq_bcn_update(Wq9001State *s, const uint8_t *p, size_t len)
{
    const uint8_t *f = p + BCN_DATA;
    size_t n;
    /* the SSID leads the elements, after timestamp, interval, capability */
    const size_t ie = WIFI_HDR_LEN + 12;
    const uint8_t *rsn = NULL;

    if (len < BCN_DATA || !s->client_nic) {
        return;
    }
    n = MIN(lduw_le_p(p + 2), MIN(len - BCN_DATA, BCN_MAX));
    if (n < ie + 2 || lduw_le_p(f) != WIFI_FC_MGMT(WIFI_STYPE_BEACON) ||
        f[ie] != WIFI_EID_SSID || ie + 2 + f[ie + 1] > n) {
        return;
    }
    if (s->bcn && !memcmp(s->bcn_bssid.a, f + 16, 6) &&
        s->bcn_ssid_len == MIN(f[ie + 1], sizeof(s->bcn_ssid)) &&
        !memcmp(s->bcn_ssid, f + ie + 2, s->bcn_ssid_len)) {
        return;
    }
    s->bcn = true;
    s->bcn_macid = p[0];
    memcpy(s->bcn_bssid.a, f + 16, 6);
    s->bcn_ssid_len = MIN(f[ie + 1], sizeof(s->bcn_ssid));
    memcpy(s->bcn_ssid, f + ie + 2, s->bcn_ssid_len);
    for (size_t i = ie; i + 2 <= n && i + 2 + f[i + 1] <= n;
         i += 2 + f[i + 1]) {
        if (f[i] == WIFI_EID_RSN) {
            rsn = f + i;
            break;
        }
    }
    wifi_client_bss(&s->client, s->bcn_bssid.a, s->bcn_ssid,
                    s->bcn_ssid_len, rsn);
}

static void wq_bss_down(Wq9001State *s)
{
    if (s->bcn) {
        s->bcn = false;
        wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
    }
}

static uint32_t wq_reg(Wq9001State *s, uint32_t addr)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->regs,
                                                GUINT_TO_POINTER(addr)));
}

/*
 * A command for the firmware, answered at once: its confirm, where the
 * driver waits for one, sized as the driver's buffer for it
 */
static void wq_cmd(Wq9001State *s, unsigned rcv, unsigned seq,
                   const uint8_t *p, size_t len)
{
    unsigned type;
    size_t n;
    uint8_t r[CFM_DESC_LEN + 8 + 256] = {};
    size_t rlen = 0;
    uint8_t *c = r + CFM_DESC_LEN;
    bool dwell = false;

    if (len < MSG_DESC_LEN) {
        return;
    }
    type = lduw_le_p(p);
    n = MIN(lduw_le_p(p + 2), len - MSG_DESC_LEN);
    p += MSG_DESC_LEN;
    trace_wq9001_cmd(type, seq, n);
    switch (type) {
    case MSG_REG_READ:
        if (n >= 12) {
            uint32_t addr = ldl_le_p(p), cnt = MIN(ldl_le_p(p + 8), 256);

            c[1] = cnt;
            for (uint32_t i = 0; i < cnt; i += 4) {
                uint8_t w[4];

                stl_le_p(w, wq_reg(s, addr + i));
                memcpy(c + 2 + i, w, MIN(4, cnt - i));
            }
            trace_wq9001_reg_read(addr, wq_reg(s, addr));
            rlen = 2 + cnt;
        }
        break;
    case MSG_REG_WRITE:
        if (n >= 8) {
            trace_wq9001_reg_write(ldl_le_p(p), ldl_le_p(p + 4));
            g_hash_table_insert(s->regs, GUINT_TO_POINTER(ldl_le_p(p)),
                                GUINT_TO_POINTER(ldl_le_p(p + 4)));
        }
        break;
    case MSG_FW_INFO:
        c[0] = FW_IPC_VER;
        c[1] = FW_HW_VER;
        rlen = 2;
        break;
    case MSG_NODE_NOTIFY:
        /* struct node_cfm: result, mnid, the firmware's node pointer */
        if (n >= 2 && p[0] == NODE_ALLOC) {
            c[1] = NODE_INVALID;
            for (unsigned i = 0; i < NODES; i++) {
                if (!(s->nodes & BIT(i))) {
                    s->nodes |= BIT(i);
                    c[1] = i;
                    break;
                }
            }
            rlen = 8;
        } else if (n >= 2 && p[0] == NODE_FREE && p[1] < NODES) {
            s->nodes &= ~BIT(p[1]);
        }
        break;
    case MSG_CH_BW_NOTIFY:
        /* struct ch_bw_info: frequency, channel; the channel comes back */
        if (n >= 3) {
            if (p[2] != s->channel) {
                trace_wq9001_channel(s->channel, p[2]);
                s->channel = p[2];
                dwell = s->channel == s->ap.channel;
            }
            c[1] = p[2];
            rlen = 2;
        }
        break;
    case MSG_BCN_UPDATE:
        wq_bcn_update(s, p, n);
        break;
    case MSG_MVAP_NOTIFY:
        if (n >= 2 && p[0] == MVAP_FREE && s->bcn && p[1] == s->bcn_macid) {
            wq_bss_down(s);
        }
        break;
    case MSG_MP_GET_NOTIFY:
        /* the result, then the answer, all zero */
        if (n >= MP_RSP_LEN + 2) {
            rlen = 1 + MIN(lduw_le_p(p + MP_RSP_LEN), 256);
        }
        break;
    case MSG_AMPDU_AGGR_NOTIFY:
        if (n >= 1 && p[0] == AMPDU_AGG_GET) {
            c[1] = 1;
            rlen = 2;
        }
        break;
    }
    if (rlen) {
        rlen += CFM_DESC_LEN;
        stw_le_p(r + 2, rlen - CFM_DESC_LEN);
    }
    g_queue_push_tail(&s->in, wq_unit(IPC_TYPE_CMD, rcv, seq, 0, r, rlen));
    /* a dwell on the channel catches a beacon, once the switch is done */
    if (dwell) {
        wq_beacon_tick(s);
    }
}

/*
 * A frame to send, behind its descriptor; the chip encrypts one marked
 * protected, so the air carries what is behind its CCMP header. Each goes
 * acknowledged: the driver counts the frames the firmware still holds.
 */
static void wq_pkt_out(Wq9001State *s, const uint8_t *p, size_t len)
{
    g_autofree uint8_t *f = NULL;
    /* struct ack_payload: an inactivity reload, sent, for the node */
    uint8_t ack[3] = { 0, 1, 0 };
    size_t n, hdr;
    uint16_t fc;

    if (len < DATA_DESC_LEN) {
        return;
    }
    n = MIN(lduw_le_p(p + DESC_LEN), len - DATA_DESC_LEN);
    ack[2] = p[DESC_STA_IDX];
    if (n >= WIFI_HDR_LEN - 6) {
        f = g_memdup2(p + DATA_DESC_LEN, n);
        fc = lduw_le_p(f);
        trace_wq9001_tx(fc, n);
        if ((ldl_le_p(p + DESC_INFO1) & TX_IS_PROTECT) &&
            n >= wifi_hdr_len(fc) + WIFI_CCMP_HDR_LEN) {
            hdr = wifi_hdr_len(fc);
            memmove(f + hdr, f + hdr + WIFI_CCMP_HDR_LEN,
                    n - hdr - WIFI_CCMP_HDR_LEN);
            n -= WIFI_CCMP_HDR_LEN;
        }
        stw_le_p(f, fc & ~WIFI_FC_PROTECTED);
        if (s->bcn && !memcmp(f + 10, s->bcn_bssid.a, 6)) {
            wifi_client_receive(&s->client, f, n);
        } else if (s->channel == s->ap.channel) {
            wifi_ap_receive(&s->ap, f, n);
        }
    }
    wq_event(s, EVT_ACK_RETURN, ack, sizeof(ack));
}

static void wq_unit_out(Wq9001State *s, const uint8_t *u, size_t len)
{
    uint32_t flags;

    if (len < IPC_HDR_LEN) {
        return;
    }
    flags = ldl_le_p(u);
    switch (IPC_TYPE(flags)) {
    case IPC_TYPE_CMD:
        wq_cmd(s, IPC_RCV(flags), IPC_SEQ(flags), u + IPC_HDR_LEN,
               MIN(ldl_le_p(u + 8), len - IPC_HDR_LEN));
        break;
    case IPC_TYPE_PKT:
        wq_pkt_out(s, u + IPC_HDR_LEN, MIN(ldl_le_p(u + 8),
                                           len - IPC_HDR_LEN));
        break;
    }
}

static void wq_handle_control(USBDevice *dev, USBPacket *p, int request,
                              int value, int index, int length,
                              uint8_t *data)
{
    Wq9001State *s = WQ9001(dev);

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }
    switch (request) {
    case VendorDeviceRequest | VREQ_GET_ROM_VER:
        /* ROM 1.0, built 00:00 */
        memset(data, 0, MIN(length, 4));
        if (length >= 1) {
            data[0] = 1;
        }
        p->actual_length = MIN(length, 4);
        return;
    case VendorDeviceOutRequest | VREQ_FW_DL:
        trace_wq9001_fw(value << 16 | index);
        s->fw_running = false;
        return;
    case VendorDeviceOutRequest | VREQ_FW_DL_COMP:
        /* the firmware boots and says so on bulk IN 1 */
        s->fw_running = true;
        wq_event(s, EVT_FWREADY, NULL, 0);
        return;
    case VendorDeviceRequest | VREQ_GET_DSTATE:
        if (length >= 1) {
            data[0] = s->fw_running ? DSTATE_READY : DSTATE_TRANSFER;
            p->actual_length = 1;
        }
        return;
    case VendorDeviceOutRequest | VREQ_SET_RESET:
        return;
    }
    p->status = USB_RET_STALL;
}

/* What the host sends, until a short packet ends the transfer */
static void wq_take(Wq9001State *s, USBPacket *p, unsigned i)
{
    GByteArray *out = s->out[i];
    size_t len = out->len;

    if (len + p->iov.size > OUT_MAX) {
        g_byte_array_set_size(out, 0);
        return;
    }
    g_byte_array_set_size(out, len + p->iov.size);
    usb_packet_copy(p, out->data + len, p->iov.size);
    if (p->iov.size % p->ep->max_packet_size == 0 && p->iov.size) {
        return;
    }
    /* before the firmware runs, OUT 1 brings it, and it is dropped */
    if (s->fw_running) {
        wq_unit_out(s, out->data, out->len);
    }
    g_byte_array_set_size(out, 0);
}

/* One unit a transfer, the zero-length end left to QEMU */
static void wq_give(Wq9001State *s, USBPacket *p)
{
    size_t n;

    if (!s->cur) {
        s->cur = g_queue_pop_head(&s->in);
        s->cur_off = 0;
        if (!s->cur) {
            p->status = USB_RET_NAK;
            return;
        }
        /* room again: what the netdevs held back may come */
        if (g_queue_get_length(&s->in) == RX_QUEUE_MAX - 1) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
            if (s->client_nic) {
                qemu_flush_queued_packets(qemu_get_queue(s->client_nic));
            }
        }
    }
    n = MIN(p->iov.size, s->cur->len - s->cur_off);
    usb_packet_copy(p, s->cur->data + s->cur_off, n);
    s->cur_off += n;
    if (s->cur_off == s->cur->len) {
        g_clear_pointer(&s->cur, g_byte_array_unref);
    }
}

static void wq_handle_data(USBDevice *dev, USBPacket *p)
{
    Wq9001State *s = WQ9001(dev);

    if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_DATA) {
        wq_give(s, p);
        return;
    }
    if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_INT) {
        p->status = USB_RET_NAK;
        return;
    }
    if (p->pid == USB_TOKEN_OUT) {
        for (unsigned i = 0; i < EP_OUTS; i++) {
            if (p->ep->nr == wq_ep_out[i]) {
                wq_take(s, p, i);
                return;
            }
        }
    }
    p->status = USB_RET_STALL;
}

static void wq_flush(Wq9001State *s)
{
    GByteArray *u;

    for (unsigned i = 0; i < EP_OUTS; i++) {
        g_byte_array_set_size(s->out[i], 0);
    }
    g_clear_pointer(&s->cur, g_byte_array_unref);
    while ((u = g_queue_pop_head(&s->in))) {
        g_byte_array_unref(u);
    }
}

/* Power on: the CPU waiting in its ROM for firmware */
static void wq_handle_reset(USBDevice *dev)
{
    Wq9001State *s = WQ9001(dev);

    wq_flush(s);
    g_hash_table_remove_all(s->regs);
    s->fw_running = false;
    s->nodes = 0;
    s->channel = 1;
    s->pkt_seq = 0;
    s->ptk_pn = s->gtk_pn = 0;
    wq_bss_down(s);
}

static bool wq_can_receive(NetClientState *nc)
{
    Wq9001State *s = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&s->in) < RX_QUEUE_MAX;
}

static ssize_t wq_receive(NetClientState *nc, const uint8_t *buf,
                          size_t size)
{
    Wq9001State *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
    return size;
}

static NetClientInfo wq_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = wq_can_receive,
    .receive = wq_receive,
};

/* From the client, on the channel of the driver's own BSS */
static void wq_client_deliver(void *opaque, const uint8_t *frame,
                              size_t len, bool protected)
{
    Wq9001State *s = opaque;

    if (s->bcn) {
        wq_rx_frame(s, frame, len, protected);
    }
}

static bool wq_client_can_receive(NetClientState *nc)
{
    Wq9001State *s = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&s->client) &&
           g_queue_get_length(&s->in) < RX_QUEUE_MAX;
}

static ssize_t wq_client_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size)
{
    Wq9001State *s = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&s->client, buf, size);
    return size;
}

static NetClientInfo wq_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = wq_client_can_receive,
    .receive = wq_client_receive,
};

static void wq_realize(USBDevice *dev, Error **errp)
{
    Wq9001State *s = WQ9001(dev);
    DeviceState *d = DEVICE(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&wq_net_info, &s->conf,
                          object_get_typename(OBJECT(d)), d->id,
                          &d->mem_reentrancy_guard, s);
    if (!wifi_ap_realize(&s->ap, s->nic, wq_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
        return;
    }
    if (s->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&s->client_conf.macaddr);
        s->client_nic = qemu_new_nic(&wq_client_net_info, &s->client_conf,
                                     object_get_typename(OBJECT(d)), NULL,
                                     &d->mem_reentrancy_guard, s);
        wifi_client_init(&s->client, s->client_nic,
                         &s->client_conf.macaddr, wq_client_deliver, s);
    }
    wq_handle_reset(dev);
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

static void wq_unrealize(USBDevice *dev)
{
    Wq9001State *s = WQ9001(dev);

    timer_del(s->beacon);
    if (s->client_nic) {
        wifi_client_cleanup(&s->client);
        qemu_del_nic(s->client_nic);
        s->client_nic = NULL;
    }
    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void wq_init(Object *obj)
{
    Wq9001State *s = WQ9001(obj);

    for (unsigned i = 0; i < EP_OUTS; i++) {
        s->out[i] = g_byte_array_new();
    }
    g_queue_init(&s->in);
    s->regs = g_hash_table_new(g_direct_hash, g_direct_equal);
    s->beacon = timer_new_ms(QEMU_CLOCK_VIRTUAL, wq_beacon_tick, s);
}

static void wq_finalize(Object *obj)
{
    Wq9001State *s = WQ9001(obj);

    wq_flush(s);
    for (unsigned i = 0; i < EP_OUTS; i++) {
        g_byte_array_unref(s->out[i]);
    }
    g_hash_table_destroy(s->regs);
    timer_free(s->beacon);
}

static const Property wq_props[] = {
    DEFINE_NIC_PROPERTIES(Wq9001State, conf),
    DEFINE_PROP_STRING("ssid", Wq9001State, ap.ssid),
    DEFINE_PROP_STRING("psk", Wq9001State, ap.psk),
    DEFINE_PROP_UINT8("channel", Wq9001State, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", Wq9001State, ap.bssid),
    DEFINE_PROP_NETDEV("client-netdev", Wq9001State, client_conf.peers),
    DEFINE_PROP_MACADDR("client-mac", Wq9001State, client_conf.macaddr),
};

/* the firmware's state and the queued units are not migrated */
static const VMStateDescription wq_vmstate = {
    .name = TYPE_WQ9001,
    .unmigratable = 1,
};

static void wq_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->product_desc = "WuQi WQ9001 USB Wi-Fi";
    uc->usb_desc = &wq_desc;
    uc->realize = wq_realize;
    uc->unrealize = wq_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = wq_handle_reset;
    uc->handle_control = wq_handle_control;
    uc->handle_data = wq_handle_data;
    dc->desc = "WuQi WQ9001 USB Wi-Fi";
    dc->vmsd = &wq_vmstate;
    device_class_set_props(dc, wq_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo wq_info = {
    .name          = TYPE_WQ9001,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(Wq9001State),
    .instance_init = wq_init,
    .instance_finalize = wq_finalize,
    .class_init    = wq_class_init,
};

static void wq_register_types(void)
{
    type_register_static(&wq_info);
}

type_init(wq_register_types)
