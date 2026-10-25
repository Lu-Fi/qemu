/*
 * Huge-IC TXW901U Wi-Fi on USB
 *
 * The chip as its vendor driver (hgics, packaged as txw901u) sees it. Its
 * boot ROM takes the firmware over the bulk pipes in 2 KB pieces and runs
 * it with a reset, after which the chip is back on USB with the firmware
 * running; the firmware is taken and dropped, and this model answers in
 * its place. Every unit either way starts with an 8-byte header: commands
 * and their answers, frames, and the beacons of the driver's own BSS. The
 * host runs mac80211 and hands the chip whole frames, which the chip
 * encrypts and decrypts. An access point (wifi-ap.c) is on the air to
 * join, and a client (wifi-client.c) joins the driver's own.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/usb/usb.h"
#include "hw/usb/desc.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "wifi-frame.h"
#include "trace.h"

#define TYPE_TXW901U "txw901u"
OBJECT_DECLARE_SIMPLE_TYPE(Txw901uState, TXW901U)

#define EP_IN                   1
#define EP_OUT                  2
#define OUT_MAX                 (16 * 1024)

/* struct hgic_hdr: magic, type, interface and flags, length, cookie */
#define HDR_LEN                 8
#define MAGIC_TX                0x1a2b
#define MAGIC_RX                0x2b1a
#define HDR_TYPE                2
#define HDR_IF                  3
#define HDR_LENGTH              4
#define HDR_COOKIE              6
#define TYPE_FRM                2
#define TYPE_CMD                3
#define TYPE_BOOTDL             7
#define TYPE_CMD2               13
#define TYPE_BEACON             17

/*
 * A frame's header has 24 more bytes, tx_info or rx_info. The flags
 * nibble counts padding: after the header on the way out, after the frame
 * on the way in.
 */
#define FRM_HDR_LEN             32
#define FRM_PAD(p)              ((p)[HDR_IF] >> 4)
#define RX_SIGNAL               11
#define RX_FREQ                 12

/* a command: its id at 8, the answer's status (a length) at 10, then data */
#define CTRL_HDR_LEN            12
#define CMD_SET_CENTER_FREQ     8
#define CMD_GET_FW_INFO         43
#define CMD_SET_BEACON_START    143
/* struct hgic_fw_info: firmware 2.6.0.1, its svn build, chip, then MAC */
#define FW_INFO_LEN             28
#define FW_VERSION              0x02060001
#define FW_SVN                  30594
#define FW_CHIP_ID              0x8000

/* the boot ROM's commands: id, address and length; answer, check byte */
#define BOOT_CMD                8
#define BOOT_ARG_ADDR           11
#define BOOT_ARG_LEN            15
#define BOOT_CMD_LEN            20
#define BOOT_RSP_LEN            16
#define BOOT_ENTER              0
#define BOOT_WRITE_MEM          2
#define BOOT_RUN                4
#define BOOT_ERR_CMD            1
#define BOOT_IN_FW              0xff
#define BOOT_CHECK              0xfd
#define RUN_RESET               BIT(29)
/* after a RUN with reset the chip leaves the bus, then comes back */
#define RESET_MS                20
#define REATTACH_MS             200

#define RX_QUEUE_MAX            64
#define BEACON_MS               102
#define RX_RSSI                 -40

struct Txw901uState {
    USBDevice parent_obj;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;
    QEMUTimer *beacon;
    QEMUTimer *reboot;

    bool running;               /* the firmware, which a USB reset keeps */
    bool fw_data;               /* the next unit is a WRITE_MEM's data */
    uint16_t fw_cookie;
    uint16_t freq;              /* MHz */
    uint16_t rx_cookie;

    /* the driver's own BSS, as its beacon names it */
    bool bcn;
    MACAddr bcn_bssid;
    uint8_t bcn_ssid[32];
    uint8_t bcn_ssid_len;

    GByteArray *out;
    GQueue in;                  /* units for bulk IN */
    GByteArray *cur;            /* the one it is giving */
    size_t cur_off;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings txw_strings = {
    [STR_MANUFACTURER]  = "HUGE-IC",
    [STR_PRODUCT]       = "TXW901U",
    [STR_SERIALNUMBER]  = "1.0",
};

#define TXW_EPS(_max)                                                       \
    (USBDescEndpoint[]) {                                                   \
        {                                                                   \
            .bEndpointAddress   = USB_DIR_IN | EP_IN,                       \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        }, {                                                                \
            .bEndpointAddress   = USB_DIR_OUT | EP_OUT,                     \
            .bmAttributes       = USB_ENDPOINT_XFER_BULK,                   \
            .wMaxPacketSize     = (_max),                                   \
        },                                                                  \
    }

static const USBDescIface txw_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = TXW_EPS(64),
};

static const USBDescIface txw_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = TXW_EPS(512),
};

static const USBDescDevice txw_device_full = {
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
            .ifs = &txw_iface_full,
        },
    },
};

static const USBDescDevice txw_device_high = {
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
            .ifs = &txw_iface_high,
        },
    },
};

/* the chip id of the firmware it runs, which the driver matches */
static const USBDesc txw_desc = {
    .id = {
        .idVendor          = 0xa012,
        .idProduct         = FW_CHIP_ID,
        .bcdDevice         = 0x0100,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &txw_device_full,
    .high = &txw_device_high,
    .str  = txw_strings,
};

/* A unit for bulk IN, its body zeroed */
static GByteArray *txw_unit(uint8_t type, uint8_t ifidx, uint16_t cookie,
                            size_t len)
{
    GByteArray *u = g_byte_array_new();

    g_byte_array_set_size(u, len);
    memset(u->data, 0, len);
    stw_le_p(u->data, MAGIC_RX);
    u->data[HDR_TYPE] = type;
    u->data[HDR_IF] = ifidx;
    stw_le_p(u->data + HDR_LENGTH, len);
    stw_le_p(u->data + HDR_COOKIE, cookie);
    return u;
}

static void txw_send(Txw901uState *s, GByteArray *u)
{
    trace_txw901u_in(u->data[HDR_TYPE], u->len);
    g_queue_push_tail(&s->in, u);
}

static void txw_flush(Txw901uState *s)
{
    GByteArray *u;

    g_byte_array_set_size(s->out, 0);
    g_clear_pointer(&s->cur, g_byte_array_unref);
    while ((u = g_queue_pop_head(&s->in))) {
        g_byte_array_unref(u);
    }
    s->fw_data = false;
}

static void txw_boot_answer(Txw901uState *s, uint16_t cookie, uint8_t cmd,
                            uint8_t rsp)
{
    GByteArray *u = txw_unit(TYPE_BOOTDL, 0, cookie, BOOT_RSP_LEN);

    u->data[BOOT_CMD] = cmd;
    u->data[BOOT_CMD + 1] = rsp;
    u->data[BOOT_RSP_LEN - 1] = BOOT_CHECK;
    txw_send(s, u);
}

/* The boot ROM's commands; the firmware answers them all "in firmware" */
static void txw_boot(Txw901uState *s, const uint8_t *p, size_t len)
{
    uint16_t cookie = lduw_le_p(p + HDR_COOKIE);
    uint8_t cmd;

    if (len < BOOT_CMD_LEN) {
        return;
    }
    cmd = p[BOOT_CMD];
    trace_txw901u_boot(cmd, ldl_le_p(p + BOOT_ARG_ADDR),
                       ldl_le_p(p + BOOT_ARG_LEN), s->running);
    if (s->running) {
        txw_boot_answer(s, cookie, cmd, BOOT_IN_FW);
        return;
    }
    switch (cmd) {
    case BOOT_ENTER:
        txw_boot_answer(s, cookie, cmd, 0);
        break;
    case BOOT_WRITE_MEM:
        s->fw_data = true;
        s->fw_cookie = cookie;
        txw_boot_answer(s, cookie, cmd, 0);
        break;
    case BOOT_RUN:
        txw_boot_answer(s, cookie, cmd, 0);
        if (ldl_le_p(p + BOOT_ARG_LEN) & RUN_RESET) {
            timer_mod(s->reboot,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + RESET_MS);
        } else {
            s->running = true;
        }
        break;
    default:
        txw_boot_answer(s, cookie, cmd, BOOT_ERR_CMD);
        break;
    }
}

/* The reset RUN asked for: off the bus, then back with the firmware */
static void txw_reboot(void *opaque)
{
    Txw901uState *s = opaque;
    USBDevice *dev = USB_DEVICE(s);
    Error *err = NULL;

    trace_txw901u_reset(dev->attached);
    if (dev->attached) {
        usb_device_detach(dev);
        txw_flush(s);
        s->running = true;
        timer_mod(s->reboot,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + REATTACH_MS);
        return;
    }
    usb_device_attach(dev, &err);
    if (err) {
        warn_report_err(err);
    }
}

static void txw_answer(Txw901uState *s, const uint8_t *req, int16_t status,
                       const uint8_t *data, size_t len)
{
    GByteArray *u = txw_unit(req[HDR_TYPE], req[HDR_IF] & 0xf,
                             lduw_le_p(req + HDR_COOKIE),
                             CTRL_HDR_LEN + len);

    /* the command's id, whichever width */
    memcpy(u->data + HDR_LEN, req + HDR_LEN, 2);
    stw_le_p(u->data + HDR_LEN + 2, status);
    if (len) {
        memcpy(u->data + CTRL_HDR_LEN, data, len);
    }
    txw_send(s, u);
}

static void txw_bss_down(Txw901uState *s)
{
    if (s->bcn) {
        s->bcn = false;
        if (s->client_nic) {
            wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
        }
    }
}

/* A command for the firmware, answered at once; one is a query */
static void txw_cmd(Txw901uState *s, const uint8_t *p, size_t len)
{
    uint8_t info[FW_INFO_LEN] = { 0 };
    const uint8_t *arg = p + CTRL_HDR_LEN;
    unsigned id;
    size_t n;

    if (len < CTRL_HDR_LEN || !s->running) {
        return;
    }
    id = p[HDR_TYPE] == TYPE_CMD2 ? lduw_le_p(p + HDR_LEN) : p[HDR_LEN];
    n = len - CTRL_HDR_LEN;
    trace_txw901u_cmd(id, p[HDR_IF] & 0xf, n);
    switch (id) {
    case CMD_GET_FW_INFO:
        stl_le_p(info, FW_VERSION);
        stl_le_p(info + 4, FW_SVN);
        stw_le_p(info + 8, FW_CHIP_ID);
        memcpy(info + 12, s->conf.macaddr.a, 6);
        txw_answer(s, p, sizeof(info), info, sizeof(info));
        return;
    case CMD_SET_CENTER_FREQ:
        if (n >= 4) {
            s->freq = ldl_le_p(arg);
        }
        break;
    case CMD_SET_BEACON_START:
        if (n >= 1 && !arg[0]) {
            txw_bss_down(s);
        }
        break;
    }
    txw_answer(s, p, 0, NULL, 0);
}

static uint16_t txw_ap_freq(Txw901uState *s)
{
    return s->ap.channel == 14 ? 2484 : 2407 + 5 * s->ap.channel;
}

/*
 * A frame off the air, as the chip hands it over: one that came encrypted
 * has been decrypted and stripped of its CCMP header and MIC, keeping the
 * Protected bit
 */
static void txw_rx_frame(Txw901uState *s, const uint8_t *f, size_t len,
                         bool protected)
{
    GByteArray *u;
    uint16_t fc;

    if (len < 10 || g_queue_get_length(&s->in) >= RX_QUEUE_MAX) {
        return;
    }
    fc = lduw_le_p(f) | (protected ? WIFI_FC_PROTECTED : 0);
    u = txw_unit(TYPE_FRM, 0, ++s->rx_cookie, FRM_HDR_LEN + len);
    u->data[RX_SIGNAL] = (uint8_t)RX_RSSI;
    stw_le_p(u->data + RX_FREQ, s->freq);
    memcpy(u->data + FRM_HDR_LEN, f, len);
    stw_le_p(u->data + FRM_HDR_LEN, fc);
    trace_txw901u_rx(fc, len);
    txw_send(s, u);
}

/* From the access point, heard on its channel */
static void txw_deliver(void *opaque, const uint8_t *frame, size_t len,
                        bool protected)
{
    Txw901uState *s = opaque;

    if (s->running && s->freq == txw_ap_freq(s)) {
        txw_rx_frame(s, frame, len, protected);
    }
}

static void txw_beacon_tick(void *opaque)
{
    Txw901uState *s = opaque;
    uint8_t b[WIFI_AP_BEACON_MAX];

    if (USB_DEVICE(s)->attached) {
        txw_deliver(s, b, wifi_ap_beacon(&s->ap, b), false);
    }
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

/* A frame the driver sends: the client's if from the driver's own BSS */
static void txw_tx_frame(Txw901uState *s, const uint8_t *p, size_t len)
{
    size_t off = FRM_HDR_LEN + FRM_PAD(p), n;
    g_autofree uint8_t *f = NULL;
    uint16_t fc;

    if (len < off + 16) {
        return;
    }
    n = len - off;
    f = g_memdup2(p + off, n);
    fc = lduw_le_p(f);
    trace_txw901u_tx(fc, n);
    stw_le_p(f, fc & ~WIFI_FC_PROTECTED);
    if (s->bcn && !memcmp(f + 10, s->bcn_bssid.a, 6)) {
        if (s->client_nic) {
            wifi_client_receive(&s->client, f, n);
        }
    } else if (s->freq == txw_ap_freq(s)) {
        wifi_ap_receive(&s->ap, f, n);
    }
}

/* The driver's own BSS from the beacons it hands the chip */
static void txw_beacon(Txw901uState *s, const uint8_t *p, size_t len)
{
    size_t off = FRM_HDR_LEN + FRM_PAD(p), n;
    const uint8_t *f = p + off, *rsn = NULL;
    /* the SSID leads the elements, after timestamp, interval, capability */
    const size_t ie = WIFI_HDR_LEN + 12;

    if (len < off || !s->client_nic) {
        return;
    }
    n = len - off;
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
    trace_txw901u_bss(s->bcn_ssid_len);
    wifi_client_bss(&s->client, s->bcn_bssid.a, s->bcn_ssid,
                    s->bcn_ssid_len, rsn);
}

/*
 * A unit from the host: a piece of firmware right after its WRITE_MEM,
 * else one header's worth
 */
static void txw_unit_out(Txw901uState *s, const uint8_t *u, size_t len)
{
    size_t n;

    if (s->fw_data) {
        s->fw_data = false;
        trace_txw901u_fw(len);
        txw_boot_answer(s, s->fw_cookie, BOOT_WRITE_MEM, 0);
        return;
    }
    if (len < HDR_LEN || lduw_le_p(u) != MAGIC_TX) {
        return;
    }
    n = MIN(lduw_le_p(u + HDR_LENGTH), len);
    switch (u[HDR_TYPE]) {
    case TYPE_BOOTDL:
        txw_boot(s, u, n);
        break;
    case TYPE_CMD:
    case TYPE_CMD2:
        txw_cmd(s, u, n);
        break;
    case TYPE_FRM:
        if (s->running) {
            txw_tx_frame(s, u, n);
        }
        break;
    case TYPE_BEACON:
        if (s->running) {
            txw_beacon(s, u, n);
        }
        break;
    }
}

/* What the host sends, until a short packet ends the transfer */
static void txw_take(Txw901uState *s, USBPacket *p)
{
    size_t len = s->out->len;

    if (len + p->iov.size > OUT_MAX) {
        g_byte_array_set_size(s->out, 0);
        return;
    }
    g_byte_array_set_size(s->out, len + p->iov.size);
    usb_packet_copy(p, s->out->data + len, p->iov.size);
    if (p->iov.size % p->ep->max_packet_size == 0 && p->iov.size) {
        return;
    }
    txw_unit_out(s, s->out->data, s->out->len);
    g_byte_array_set_size(s->out, 0);
}

/* One unit a transfer: the driver takes a single header from each */
static void txw_give(Txw901uState *s, USBPacket *p)
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

static void txw_handle_data(USBDevice *dev, USBPacket *p)
{
    Txw901uState *s = TXW901U(dev);

    if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_IN) {
        txw_give(s, p);
        return;
    }
    if (p->pid == USB_TOKEN_OUT && p->ep->nr == EP_OUT) {
        txw_take(s, p);
        return;
    }
    p->status = USB_RET_STALL;
}

static void txw_handle_control(USBDevice *dev, USBPacket *p, int request,
                               int value, int index, int length,
                               uint8_t *data)
{
    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) < 0) {
        p->status = USB_RET_STALL;
    }
}

/* A bus reset leaves the CPU, and whatever it runs, alone */
static void txw_handle_reset(USBDevice *dev)
{
    txw_flush(TXW901U(dev));
}

/* Power on: the CPU waiting in its ROM, the chip on the bus */
static void txw_power_on(DeviceState *d)
{
    Txw901uState *s = TXW901U(d);

    txw_flush(s);
    s->running = false;
    s->freq = 0;
    s->rx_cookie = 0;
    txw_bss_down(s);
    timer_del(s->reboot);
    if (!USB_DEVICE(s)->attached) {
        timer_mod(s->reboot, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    }
}

static bool txw_can_receive(NetClientState *nc)
{
    Txw901uState *s = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&s->in) < RX_QUEUE_MAX;
}

static ssize_t txw_receive(NetClientState *nc, const uint8_t *buf,
                           size_t size)
{
    Txw901uState *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
    return size;
}

static NetClientInfo txw_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = txw_can_receive,
    .receive = txw_receive,
};

/* From the client, on the channel of the driver's own BSS */
static void txw_client_deliver(void *opaque, const uint8_t *frame,
                               size_t len, bool protected)
{
    Txw901uState *s = opaque;

    if (s->bcn) {
        txw_rx_frame(s, frame, len, protected);
    }
}

static bool txw_client_can_receive(NetClientState *nc)
{
    Txw901uState *s = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&s->client) &&
           g_queue_get_length(&s->in) < RX_QUEUE_MAX;
}

static ssize_t txw_client_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    Txw901uState *s = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&s->client, buf, size);
    return size;
}

static NetClientInfo txw_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = txw_client_can_receive,
    .receive = txw_client_receive,
};

static void txw_realize(USBDevice *dev, Error **errp)
{
    Txw901uState *s = TXW901U(dev);
    DeviceState *d = DEVICE(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&txw_net_info, &s->conf,
                          object_get_typename(OBJECT(d)), d->id,
                          &d->mem_reentrancy_guard, s);
    if (!wifi_ap_realize(&s->ap, s->nic, txw_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
        return;
    }
    if (s->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&s->client_conf.macaddr);
        s->client_nic = qemu_new_nic(&txw_client_net_info, &s->client_conf,
                                     object_get_typename(OBJECT(d)), NULL,
                                     &d->mem_reentrancy_guard, s);
        wifi_client_init(&s->client, s->client_nic,
                         &s->client_conf.macaddr, txw_client_deliver, s);
    }
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

static void txw_unrealize(USBDevice *dev)
{
    Txw901uState *s = TXW901U(dev);

    timer_del(s->beacon);
    timer_del(s->reboot);
    if (s->client_nic) {
        wifi_client_cleanup(&s->client);
        qemu_del_nic(s->client_nic);
        s->client_nic = NULL;
    }
    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void txw_init(Object *obj)
{
    Txw901uState *s = TXW901U(obj);

    s->out = g_byte_array_new();
    g_queue_init(&s->in);
    s->beacon = timer_new_ms(QEMU_CLOCK_VIRTUAL, txw_beacon_tick, s);
    s->reboot = timer_new_ms(QEMU_CLOCK_VIRTUAL, txw_reboot, s);
}

static void txw_finalize(Object *obj)
{
    Txw901uState *s = TXW901U(obj);

    txw_flush(s);
    g_byte_array_unref(s->out);
    timer_free(s->beacon);
    timer_free(s->reboot);
}

static const Property txw_props[] = {
    DEFINE_NIC_PROPERTIES(Txw901uState, conf),
    DEFINE_PROP_STRING("ssid", Txw901uState, ap.ssid),
    DEFINE_PROP_STRING("psk", Txw901uState, ap.psk),
    DEFINE_PROP_UINT8("channel", Txw901uState, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", Txw901uState, ap.bssid),
    DEFINE_PROP_NETDEV("client-netdev", Txw901uState, client_conf.peers),
    DEFINE_PROP_MACADDR("client-mac", Txw901uState, client_conf.macaddr),
};

/* the firmware's state and the queued units are not migrated */
static const VMStateDescription txw_vmstate = {
    .name = TYPE_TXW901U,
    .unmigratable = 1,
};

static void txw_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->product_desc = "Huge-IC TXW901U USB Wi-Fi";
    uc->usb_desc = &txw_desc;
    uc->realize = txw_realize;
    uc->unrealize = txw_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = txw_handle_reset;
    uc->handle_control = txw_handle_control;
    uc->handle_data = txw_handle_data;
    dc->desc = "Huge-IC TXW901U USB Wi-Fi";
    dc->vmsd = &txw_vmstate;
    device_class_set_legacy_reset(dc, txw_power_on);
    device_class_set_props(dc, txw_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo txw_info = {
    .name          = TYPE_TXW901U,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(Txw901uState),
    .instance_init = txw_init,
    .instance_finalize = txw_finalize,
    .class_init    = txw_class_init,
};

static void txw_register_types(void)
{
    type_register_static(&txw_info);
}

type_init(txw_register_types)
