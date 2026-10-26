/*
 * Seekwave SWT6621S (SV6160LITE) Wi-Fi on USB
 *
 * The chip as its vendor driver (skw_usb_lite and swt6621s_wifi) sees it.
 * Its boot ROM takes the firmware in framed packets on two bulk pipes and,
 * told to run it, leaves the bus to come back with the firmware's ports:
 * one for commands, one that carries data both ways and everything the chip
 * says, a text side channel and a log. The firmware is taken and dropped,
 * and this model answers in its place. The host runs the SME of a station
 * and of an access point; the firmware scans, sends the authentication and
 * association frames the host describes, and carries data as Ethernet
 * frames with their descriptors. An access point (wifi-ap.c) is on the air
 * to join, and a client (wifi-client.c) joins the driver's own.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
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
#include "net/checksum.h"
#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "wifi-frame.h"
#include "trace.h"

#define TYPE_SWT6621S "swt6621s"
OBJECT_DECLARE_SIMPLE_TYPE(Swt6621sState, SWT6621S)

/* the interfaces, by number and endpoint, and what the driver calls them */
#define EP_CMD                  1
#define EP_DATA                 2
#define EP_LOOP                 3
#define EP_LOG                  4
#define PORT_MPS                512

/* the boot ROM: 0x7e7e7e7e, length, type, CRC16, then the body */
#define BOOT_MAGIC              0x7e7e7e7e
#define BOOT_HDR_LEN            12
#define BOOT_CONNECT            0x00
#define BOOT_START              0x01
#define BOOT_DATA               0x02
#define BOOT_EXEC               0x04
#define BOOT_ACK                0x80
#define BOOT_VERSION            0x81
#define BOOT_PACKET             0x1000
#define BOOT_OUT_MAX            (BOOT_HDR_LEN + BOOT_PACKET + 64)
#define LEAVE_MS                20
#define REATTACH_MS             200

/*
 * The chip id the boot ROM gives, which names the core's platform device;
 * its last byte is the revision that names the calibration file
 * (SWT6621S_SEEKWAVE_R04001.bin)
 */
#define CHIP_ID                 "SV6160LITE"
#define CHIP_ID_LEN             16
#define CHIP_REVISION           1

/* the side channel's lines: the firmware's version, then WiFi up */
#define LOOP_VERSION            "trunk_W25.34.6.1"
#define LOOP_WIFI_READY         "WIFIREADY"

/*
 * vendor control requests on the device; the packet count bounds a
 * transfer up, past which the driver drops the rest
 */
#define VENDOR_SERVICE          0xa6
#define VENDOR_PACKET_COUNT     0xa7
#define PACKET_COUNT            20

/*
 * Host to device, each message or frame after a word: the port in its top
 * byte, then the length that follows from bit 7 (an end-of-frame at 23)
 */
#define XHDR_LEN                4
#define XHDR_LEN_SHIFT          7
#define XHDR_LEN_MASK           0xffff
#define OUT_MAX                 (64 * 1024)

/*
 * Device to host on the data port: packets of a 12-byte header and a
 * payload, the third word holding the packet's whole length on top and
 * whether it is a message (else data, with its MAC); the first packet's
 * first word numbers the transfer and counts its packets
 */
#define PKT_HDR_LEN             12
#define PKT_W0                  0
#define PKT_W2                  8
#define PKT_IS_MSG              BIT(1)
#define PKT_MAX                 2048
#define XFER_VALID              BIT(31)
#define XFER_MAX                (16 * 1024)
#define D2H_QUEUE_MAX           128

/* struct skw_msg: inst_id:4 type:4, id, seq, total_len, two spare bytes */
#define MSG_HDR_LEN             8
#define MSG_TYPE_INST           0
#define MSG_ID                  1
#define MSG_SEQ                 2
#define MSG_TOTAL_LEN           4
#define MSG_ACK                 1
#define MSG_EVENT               2

/* enum SKW_CMD_ID, those the model does more than acknowledge */
#define CMD_GET_INFO            1
#define CMD_SYN_VERSION         2
#define CMD_OPEN_DEV            3
#define CMD_CLOSE_DEV           4
#define CMD_START_SCAN          5
#define CMD_STOP_SCAN           6
#define CMD_JOIN                9
#define CMD_AUTH                10
#define CMD_ASSOC               11
#define CMD_ADD_KEY             12
#define CMD_TX_MGMT             14
#define CMD_TX_DATA_FRAME       15
#define CMD_DISCONNECT          17
#define CMD_START_AP            19
#define CMD_STOP_AP             20
#define CMD_ADD_STA             21
#define CMD_DEL_STA             22
#define CMD_SET_MAC             24

/* enum SKW_EVENT_ID */
#define EVENT_SCAN_CMPL         0
#define EVENT_RX_MGMT           4
#define EVENT_SCAN_REPORT       11
#define EVENT_MGMT_TX_STATUS    12
#define EVENT_CREDIT_UPDATE     16

/* struct skw_version_info: a version for each command and event */
#define VERSION_INFO_LEN        512

/* struct skw_chip_info, packed */
#define CI_LEN                  234
#define CI_MAX_STA              18
#define CI_MAX_MC               19
#define CI_HT_CAPA              20
#define CI_HT_AMPDU             24
#define CI_HT_TX_MCS            26
#define CI_HT_RX_MCS            30
#define CI_MAX_SCAN_SSIDS       42
#define CI_MAC                  64
#define CI_BW_CAPA              86
#define CI_PRIV                 90
#define CI_BUILD_TIME           94
#define CI_PLAT_VER             126
#define CI_WIFI_VER             142
#define CI_CHIP_TYPE            178
#define CI_PRIV_2G_ONLY         BIT(5)
#define CI_PRIV_NR_HW_MAC       24
#define CHIP_TYPE_SWT6621S      0x300
#define BW_CAP_2G_20M           BIT(0)

/* the commands' parameters, packed (skw_cfg80211.h, skw_iface.h) */
#define OPEN_MODE               0
#define OPEN_MAC                4
#define MODE_STA                1
#define MODE_AP                 2
#define SCAN_NR_CHAN            8
#define SCAN_CHAN_OFFSET        12
#define SCAN_CHAN_LEN           3
#define JOIN_CHAN               0
#define JOIN_CAPABILITY         7
#define JOIN_BSSID              11
#define JOIN_IE_OFFSET          19
#define JOIN_IE_LEN             21
#define JOIN_LEN                25
#define AUTH_ALG                0
#define AUTH_DATA_OFFSET        6
#define AUTH_IE_OFFSET          10
#define AUTH_LEN                14
#define ASSOC_BSSID             38
#define ASSOC_IE_OFFSET         50
#define ASSOC_LEN               54
#define KEY_MAC                 0
#define KEY_TYPE                6
#define KEY_TYPE_PTK            0
#define MGMT_TX_COOKIE          4
#define MGMT_TX_FRAME_LEN       15
#define MGMT_TX_LEN             17
#define AP_SSID_LEN             11
#define AP_SSID                 12
#define AP_HEAD_OFFSET          44
#define AP_HEAD_LEN             46
#define AP_TAIL_OFFSET          48
#define AP_TAIL_LEN             50
#define AP_LEN                  64
#define DISCONNECT_LOCAL        1
#define DEL_STA_REASON          6
#define DEL_STA_TX_FRAME        8
#define DEL_STA_LEN             9

/* struct skw_mgmt_hdr, ahead of a frame in an event */
#define MGMT_HDR_LEN            8
#define RX_SIGNAL               -40
#define BAND_2GHZ               0

/*
 * A data packet up: the header, PHY words the driver skips, struct
 * skw_rx_desc at 52, and the frame as Ethernet after it
 */
#define RXD                     52
#define RXD_FLAGS               (RXD + 0)
#define RXD_MSDU_LEN            (RXD + 2)
#define RXD_QOS                 (RXD + 4)
#define RXD_SN                  (RXD + 8)
#define RXD_PEER                (RXD + 10)
#define RXD_PN                  (RXD + 12)
#define RXD_MSDU_OFFSET         (RXD + 18)
#define RXD_LEN                 20
#define RX_MSDU                 (RXD + RXD_LEN)
#define RXF_EAPOL               BIT(14)
#define RXF_SNAP_MATCH          BIT(2)
#define RXF_QOS                 BIT(4)
#define RXP_INST_VALID          BIT(2)
#define RXP_PEER_SHIFT          4
#define RXP_PEER_VALID          BIT(9)
#define RXP_MC                  BIT(10)
#define RXP_TID_SHIFT           12
/* the 802.11 MSDU's length counts the LLC/SNAP, Ethernet's header 6 more */
#define SNAP_LEN                8
#define ETH_HDR_LEN             14

/* struct skw_tx_desc_hdr, then padding, skw_tx_desc_conf and the frame */
#define TXD_HDR_LEN             6
#define TXD_CONF_LEN            2
#define TXD_GAP_MASK            0x3
#define TXD_INST_SHIFT          2
#define TXC_L4_MASK             0x3ff
#define TXC_CSUM                BIT(10)
#define TXC_TCP                 BIT(11)

#define NR_INST                 4
#define PEERS_MAX               32
#define CREDITS                 32
#define SCAN_MS                 120
#define FRAME_MAX               2304

typedef struct Swt6621sPeer {
    bool valid;
    uint8_t inst;
    MACAddr mac;
    uint64_t pn;
} Swt6621sPeer;

struct Swt6621sState {
    USBDevice parent_obj;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;
    QEMUTimer *replug;
    QEMUTimer *scan;

    bool running;               /* the firmware: the four-interface device */
    bool exec;                  /* the boot ROM told to run it */
    GByteArray *boot_out;
    GQueue boot_in;             /* the boot ROM's answers */

    GByteArray *cmd_out;        /* the command port, a message at a time */
    GByteArray *data_out;       /* the data port, a frame at a time */
    GQueue d2h;                 /* packets for the data port's IN */
    GQueue loop_in;             /* side channel lines */
    uint16_t xfer_seq;
    uint16_t event_seq;
    uint8_t mac_users;          /* instances joined or beaconing */

    /* the host's interfaces, by instance */
    struct {
        uint8_t mode;
        MACAddr mac;
        MACAddr bssid;          /* a station's, as it joins */
        uint8_t peer;           /* a station's access point */
        uint8_t chan;
        uint16_t capability;
        GByteArray *bss_ies;
    } inst[NR_INST];
    Swt6621sPeer peers[PEERS_MAX];
    uint8_t scan_inst;
    bool scanning;

    /* the driver's own BSS, as START_AP gives it */
    bool bcn;
    uint8_t bcn_inst;
    MACAddr bcn_bssid;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
    STR_BOOT,
    STR_CMD,
    STR_DATA,
    STR_LOOP,
    STR_LOG,
};

static const USBDescStrings swt_strings = {
    [STR_MANUFACTURER]  = "Seekwave",
    [STR_PRODUCT]       = CHIP_ID,
    [STR_SERIALNUMBER]  = "1",
    [STR_BOOT]          = "Boot",
    [STR_CMD]           = "WIFITCMD",
    [STR_DATA]          = "WIFIDATA",
    [STR_LOOP]          = "LOOP",
    [STR_LOG]           = "LOG",
};

/* each port a pair of bulk pipes; the driver tells them by name */
#define SWT_EPS(_ep)                                                        \
    (USBDescEndpoint[]) {                                                   \
        {                                                                   \
            .bEndpointAddress      = USB_DIR_IN | (_ep),                    \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = PORT_MPS,                              \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | (_ep),                   \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = PORT_MPS,                              \
        },                                                                  \
    }

#define SWT_IFACE(_n, _str, _ep)                                            \
    {                                                                       \
        .bInterfaceNumber          = (_n),                                  \
        .bNumEndpoints             = 2,                                     \
        .bInterfaceClass           = USB_CLASS_VENDOR_SPEC,                 \
        .iInterface                = (_str),                                \
        .eps                       = SWT_EPS(_ep),                          \
    }

static const USBDescIface swt_iface_boot[] = {
    SWT_IFACE(0, STR_BOOT, EP_CMD),
};

static const USBDescIface swt_iface_fw[] = {
    SWT_IFACE(0, STR_CMD, EP_CMD),
    SWT_IFACE(1, STR_DATA, EP_DATA),
    SWT_IFACE(2, STR_LOOP, EP_LOOP),
    SWT_IFACE(3, STR_LOG, EP_LOG),
};

#define SWT_DEVICE(_ifaces)                                                 \
    (const USBDescDevice) {                                                 \
        .bcdUSB                    = 0x0200,                                \
        .bMaxPacketSize0           = 64,                                    \
        .bNumConfigurations        = 1,                                     \
        .confs = (USBDescConfig[]) {                                        \
            {                                                               \
                .bNumInterfaces    = ARRAY_SIZE(_ifaces),                   \
                .bConfigurationValue = 1,                                   \
                .bmAttributes      = USB_CFG_ATT_ONE,                       \
                .bMaxPower         = 250,                                   \
                .nif = ARRAY_SIZE(_ifaces),                                 \
                .ifs = (_ifaces),                                           \
            },                                                              \
        },                                                                  \
    }

static const USBDescDevice swt_device_boot = SWT_DEVICE(swt_iface_boot);
static const USBDescDevice swt_device_fw = SWT_DEVICE(swt_iface_fw);

#define SWT_DESC(_dev)                                                      \
    (const USBDesc) {                                                       \
        .id = {                                                             \
            .idVendor              = 0x3607,                                \
            .idProduct             = 0x6621,                                \
            .bcdDevice             = 0x0100,                                \
            .iManufacturer         = STR_MANUFACTURER,                      \
            .iProduct              = STR_PRODUCT,                           \
            .iSerialNumber         = STR_SERIALNUMBER,                      \
        },                                                                  \
        .high = (_dev),                                                     \
        .str  = swt_strings,                                                \
    }

static const USBDesc swt_desc_boot = SWT_DESC(&swt_device_boot);
static const USBDesc swt_desc_fw = SWT_DESC(&swt_device_fw);

static void swt_flush_queue(GQueue *q)
{
    GByteArray *u;

    while ((u = g_queue_pop_head(q))) {
        g_byte_array_unref(u);
    }
}

static void swt_flush(Swt6621sState *s)
{
    g_byte_array_set_size(s->boot_out, 0);
    g_byte_array_set_size(s->cmd_out, 0);
    g_byte_array_set_size(s->data_out, 0);
    swt_flush_queue(&s->boot_in);
    swt_flush_queue(&s->d2h);
    swt_flush_queue(&s->loop_in);
}

static void swt_queue(GQueue *q, const void *data, size_t len)
{
    GByteArray *u = g_byte_array_new();

    g_byte_array_append(u, data, len);
    g_queue_push_tail(q, u);
}

/* The boot ROM's answer: its header, and a body for some */
static void swt_boot_answer(Swt6621sState *s, uint16_t type,
                            const void *body, size_t len)
{
    uint8_t a[BOOT_HDR_LEN + 32] = { 0 };

    len = MIN(len, sizeof(a) - BOOT_HDR_LEN);
    stl_le_p(a, BOOT_MAGIC);
    stl_le_p(a + 4, len);
    stw_le_p(a + 8, type);
    if (len) {
        memcpy(a + BOOT_HDR_LEN, body, len);
    }
    swt_queue(&s->boot_in, a, BOOT_HDR_LEN + len);
}

/*
 * A packet the boot ROM was sent: the magic alone asks its version, a
 * connect its chip and the size of the data packets it takes
 */
static void swt_boot(Swt6621sState *s, const uint8_t *p, size_t len)
{
    uint8_t body[32] = { 0 };
    uint16_t type;

    if (len < 4 || ldl_le_p(p) != BOOT_MAGIC) {
        return;
    }
    if (len < BOOT_HDR_LEN) {
        trace_swt_boot(BOOT_VERSION, len);
        swt_boot_answer(s, BOOT_VERSION, "Boot Loader version 1.0", 24);
        return;
    }
    type = lduw_le_p(p + 8);
    trace_swt_boot(type, ldl_le_p(p + 4));
    switch (type) {
    case BOOT_CONNECT:
        /* struct connect_ack: packet size, flags, chip id */
        stl_le_p(body, BOOT_PACKET);
        stl_le_p(body + 4, 2);
        memcpy(body + 8, CHIP_ID, strlen(CHIP_ID));
        body[8 + CHIP_ID_LEN - 1] = CHIP_REVISION;
        swt_boot_answer(s, BOOT_ACK, body, 8 + CHIP_ID_LEN);
        break;
    case BOOT_START:
    case BOOT_DATA:
        swt_boot_answer(s, BOOT_ACK, NULL, 0);
        break;
    case BOOT_EXEC:
        swt_boot_answer(s, BOOT_ACK, NULL, 0);
        s->exec = true;
        break;
    }
}

/* Off the bus, then back on it running the firmware */
static void swt_replug(void *opaque)
{
    Swt6621sState *s = opaque;
    USBDevice *dev = USB_DEVICE(s);
    Error *err = NULL;

    trace_swt_replug(dev->attached);
    if (dev->attached) {
        usb_device_detach(dev);
        swt_flush(s);
        s->running = true;
        s->exec = false;
        dev->usb_desc = &swt_desc_fw;
        timer_mod(s->replug,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + REATTACH_MS);
        return;
    }
    usb_device_attach(dev, &err);
    if (err) {
        warn_report_err(err);
    }
}

/* A packet for the host: a message, or data for one of the MACs */
static GByteArray *swt_packet(size_t len, bool msg)
{
    GByteArray *u = g_byte_array_new();
    size_t n = PKT_HDR_LEN + len;

    g_byte_array_set_size(u, ROUND_UP(n, 4));
    memset(u->data, 0, u->len);
    stl_le_p(u->data + PKT_W2, n << 16 | (msg ? PKT_IS_MSG : 0));
    return u;
}

static void swt_send(Swt6621sState *s, GByteArray *u)
{
    if (g_queue_get_length(&s->d2h) >= D2H_QUEUE_MAX) {
        g_byte_array_unref(u);
        return;
    }
    g_queue_push_tail(&s->d2h, u);
}

static uint8_t *swt_msg(GByteArray *u)
{
    return u->data + PKT_HDR_LEN;
}

/* A command done: its id and number, a status, and what it gives back */
static void swt_ack(Swt6621sState *s, const uint8_t *cmd, const void *rsp,
                    size_t len)
{
    GByteArray *u = swt_packet(MSG_HDR_LEN + 2 + len, true);
    uint8_t *m = swt_msg(u);

    m[MSG_TYPE_INST] = MSG_ACK << 4 | (cmd[MSG_TYPE_INST] & 0xf);
    m[MSG_ID] = cmd[MSG_ID];
    stw_le_p(m + MSG_SEQ, lduw_le_p(cmd + MSG_SEQ));
    stw_le_p(m + MSG_TOTAL_LEN, MSG_HDR_LEN + 2 + len);
    if (len) {
        memcpy(m + MSG_HDR_LEN + 2, rsp, len);
    }
    swt_send(s, u);
}

/* An event, numbered: the driver asserts the chip on a gap */
static void swt_event(Swt6621sState *s, uint8_t id, uint8_t inst,
                      const void *data, size_t len)
{
    GByteArray *u = swt_packet(MSG_HDR_LEN + len, true);
    uint8_t *m = swt_msg(u);

    m[MSG_TYPE_INST] = MSG_EVENT << 4 | (inst & 0xf);
    m[MSG_ID] = id;
    stw_le_p(m + MSG_SEQ, ++s->event_seq);
    stw_le_p(m + MSG_TOTAL_LEN, MSG_HDR_LEN + len);
    if (len) {
        memcpy(m + MSG_HDR_LEN, data, len);
    }
    trace_swt_event(id, inst, s->event_seq, len);
    swt_send(s, u);
}

/* A frame off the air in an event: a scan's, or one the host takes in */
static void swt_mgmt_event(Swt6621sState *s, uint8_t id, uint8_t inst,
                           uint8_t chan, const uint8_t *f, size_t len)
{
    g_autofree uint8_t *e = g_malloc0(MGMT_HDR_LEN + len);

    e[0] = chan;
    e[1] = BAND_2GHZ;
    stw_le_p(e + 2, (uint16_t)RX_SIGNAL);
    stw_le_p(e + 4, len);
    memcpy(e + MGMT_HDR_LEN, f, len);
    swt_event(s, id, inst, e, MGMT_HDR_LEN + len);
}

/* Data credit back to the host for the frames the firmware took */
static void swt_credit(Swt6621sState *s, unsigned n)
{
    uint8_t c[4] = { 0 };

    stw_le_p(c, n);
    swt_event(s, EVENT_CREDIT_UPDATE, 0, c, sizeof(c));
}

/*
 * An instance that joins or starts a BSS takes the MAC. The host drops
 * the MAC's credit once no instance is left on it, so the first one in
 * gets the whole window again
 */
static void swt_mac_bind(Swt6621sState *s, uint8_t inst)
{
    bool idle = !s->mac_users;

    s->mac_users |= BIT(inst);
    if (idle) {
        swt_credit(s, CREDITS);
    }
}

static int swt_peer_add(Swt6621sState *s, uint8_t inst, const uint8_t *mac)
{
    int i, free = -1;

    for (i = 0; i < PEERS_MAX; i++) {
        if (s->peers[i].valid && s->peers[i].inst == inst &&
            !memcmp(s->peers[i].mac.a, mac, 6)) {
            return i;
        }
        if (!s->peers[i].valid && free < 0) {
            free = i;
        }
    }
    if (free >= 0) {
        s->peers[free].valid = true;
        s->peers[free].inst = inst;
        memcpy(s->peers[free].mac.a, mac, 6);
        s->peers[free].pn = 0;
    }
    return free;
}

static int swt_peer_find(Swt6621sState *s, uint8_t inst, const uint8_t *mac)
{
    for (int i = 0; i < PEERS_MAX; i++) {
        if (s->peers[i].valid && s->peers[i].inst == inst &&
            !memcmp(s->peers[i].mac.a, mac, 6)) {
            return i;
        }
    }
    return -1;
}

static void swt_peer_del(Swt6621sState *s, uint8_t inst, const uint8_t *mac)
{
    int i = swt_peer_find(s, inst, mac);

    if (i >= 0) {
        s->peers[i].valid = false;
    }
}

static void swt_peers_del(Swt6621sState *s, uint8_t inst)
{
    for (int i = 0; i < PEERS_MAX; i++) {
        if (s->peers[i].inst == inst) {
            s->peers[i].valid = false;
        }
    }
}

static bool swt_is_sta(Swt6621sState *s, uint8_t inst)
{
    return inst < NR_INST && s->inst[inst].mode == MODE_STA;
}

/* the station joined to the access point on the air, if one is */
static int swt_sta_inst(Swt6621sState *s)
{
    for (int i = 0; i < NR_INST; i++) {
        if (swt_is_sta(s, i) &&
            !memcmp(s->inst[i].bssid.a, s->ap.bssid.a, 6)) {
            return i;
        }
    }
    return -1;
}

/*
 * A data frame off the air for the host, as Ethernet after its
 * descriptor; one that came encrypted has been decrypted, and numbered
 */
static void swt_rx_data(Swt6621sState *s, uint8_t inst, int peer,
                        const uint8_t *da, const uint8_t *sa,
                        const uint8_t *f, size_t hl, size_t len,
                        bool protected)
{
    size_t body = len - hl;
    uint16_t fc = lduw_le_p(f), proto, flags = 0, pf;
    uint8_t tid = 0;
    GByteArray *u;
    uint8_t *p;
    bool mc = da[0] & 1;

    if (body < SNAP_LEN || memcmp(f + hl, WIFI_RFC1042, 6)) {
        return;
    }
    proto = lduw_be_p(f + hl + 6);
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        tid = f[hl - 2] & 0xf;
    }
    u = swt_packet(RX_MSDU - PKT_HDR_LEN + ETH_HDR_LEN + body - SNAP_LEN,
                   false);
    p = u->data;
    if (proto == 0x888e) {
        flags |= RXF_EAPOL;
    }
    stw_le_p(p + RXD_FLAGS, flags);
    stw_le_p(p + RXD_MSDU_LEN, body);
    /* an LLC/SNAP header found: else the driver takes it for 802.3 */
    p[RXD_QOS] = RXF_SNAP_MATCH;
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        p[RXD_QOS] |= RXF_QOS;
    }
    stw_le_p(p + RXD_SN, lduw_le_p(f + 22) >> 4);
    pf = (inst & 3) | RXP_INST_VALID | peer << RXP_PEER_SHIFT |
         RXP_PEER_VALID | tid << RXP_TID_SHIFT | (mc ? RXP_MC : 0);
    stw_le_p(p + RXD_PEER, pf);
    if (protected && !mc) {
        uint64_t pn = ++s->peers[peer].pn;

        for (int i = 0; i < 6; i++) {
            p[RXD_PN + i] = pn >> (8 * i);
        }
    }
    p[RXD_MSDU_OFFSET] = RX_MSDU;
    memcpy(p + RX_MSDU, da, 6);
    memcpy(p + RX_MSDU + 6, sa, 6);
    stw_be_p(p + RX_MSDU + 12, proto);
    memcpy(p + RX_MSDU + ETH_HDR_LEN, f + hl + SNAP_LEN, body - SNAP_LEN);
    trace_swt_rx(fc, len, inst, peer);
    swt_send(s, u);
}

/* From the access point on the air */
static void swt_deliver(void *opaque, const uint8_t *f, size_t len,
                        bool protected)
{
    Swt6621sState *s = opaque;
    int inst = swt_sta_inst(s);
    uint16_t fc;
    size_t hl = WIFI_HDR_LEN;

    if (len < WIFI_HDR_LEN) {
        return;
    }
    fc = lduw_le_p(f);
    if (WIFI_FC_TYPE(fc) == WIFI_TYPE_MGMT) {
        if (WIFI_FC_STYPE(fc) == WIFI_STYPE_PROBE_RESP ||
            WIFI_FC_STYPE(fc) == WIFI_STYPE_BEACON) {
            if (s->scanning) {
                swt_mgmt_event(s, EVENT_SCAN_REPORT, s->scan_inst,
                               s->ap.channel, f, len);
            }
            return;
        }
        if (inst >= 0 && !memcmp(f + 4, s->inst[inst].mac.a, 6)) {
            trace_swt_rx(fc, len, inst, s->inst[inst].peer);
            swt_mgmt_event(s, EVENT_RX_MGMT, inst, s->ap.channel, f, len);
        }
        return;
    }
    if (inst < 0 || WIFI_FC_TYPE(fc) != WIFI_TYPE_DATA ||
        (WIFI_FC_STYPE(fc) & WIFI_STYPE_NODATA)) {
        return;
    }
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        hl += 2;
    }
    if (len < hl || (memcmp(f + 4, s->inst[inst].mac.a, 6) && !(f[4] & 1))) {
        return;
    }
    swt_rx_data(s, inst, s->inst[inst].peer, f + 4, f + 16, f, hl, len,
                protected);
}

/* From the client, to the driver's own BSS */
static void swt_client_deliver(void *opaque, const uint8_t *f, size_t len,
                               bool protected)
{
    Swt6621sState *s = opaque;
    uint16_t fc;
    size_t hl = WIFI_HDR_LEN;
    int peer;

    if (!s->bcn || len < WIFI_HDR_LEN) {
        return;
    }
    fc = lduw_le_p(f);
    if (WIFI_FC_TYPE(fc) == WIFI_TYPE_MGMT) {
        trace_swt_rx(fc, len, s->bcn_inst, PEERS_MAX);
        swt_mgmt_event(s, EVENT_RX_MGMT, s->bcn_inst,
                       s->inst[s->bcn_inst].chan, f, len);
        return;
    }
    if (WIFI_FC_TYPE(fc) != WIFI_TYPE_DATA ||
        (WIFI_FC_STYPE(fc) & WIFI_STYPE_NODATA)) {
        return;
    }
    if (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS) {
        hl += 2;
    }
    peer = swt_peer_find(s, s->bcn_inst, f + 10);
    if (len < hl || peer < 0) {
        return;
    }
    swt_rx_data(s, s->bcn_inst, peer, f + 16, f + 10, f, hl, len,
                protected);
}

/* A frame the host has sent: to the access point, or to the client */
static void swt_air(Swt6621sState *s, uint8_t inst, uint8_t *f, size_t len)
{
    uint16_t fc = lduw_le_p(f) & ~WIFI_FC_PROTECTED;

    stw_le_p(f, fc);
    trace_swt_tx(fc, len, inst);
    if (s->bcn && inst == s->bcn_inst) {
        if (s->client_nic) {
            wifi_client_receive(&s->client, f, len);
        }
    } else if (swt_is_sta(s, inst)) {
        wifi_ap_receive(&s->ap, f, len);
    }
}

/* whether a frame the host has sent is acknowledged on the air */
static bool swt_acked(Swt6621sState *s, uint8_t inst, const uint8_t *f)
{
    if (f[4] & 1) {
        return true;
    }
    if (s->bcn && inst == s->bcn_inst) {
        return s->client_nic && wifi_client_acks(&s->client, f + 4);
    }
    return !memcmp(f + 4, s->ap.bssid.a, 6);
}

/* A station the host drops from its BSS, told so when the host asks */
static void swt_ap_deauth(Swt6621sState *s, uint8_t inst, const uint8_t *mac,
                          uint16_t reason)
{
    uint8_t f[WIFI_HDR_LEN + 2] = { 0 };

    if (!s->bcn || inst != s->bcn_inst) {
        return;
    }
    stw_le_p(f, WIFI_FC_MGMT(WIFI_STYPE_DEAUTH));
    memcpy(f + 4, mac, 6);
    memcpy(f + 10, s->bcn_bssid.a, 6);
    memcpy(f + 16, s->bcn_bssid.a, 6);
    stw_le_p(f + WIFI_HDR_LEN, reason);
    swt_air(s, inst, f, sizeof(f));
}

/*
 * A TCP or UDP checksum the stack left to the chip (NETIF_F_IP_CSUM),
 * done as the stack would: summed from the L4 header over its field,
 * which holds the pseudo-header's sum
 */
static void swt_csum(uint8_t *eth, size_t n, uint16_t conf)
{
    size_t l4 = conf & TXC_L4_MASK;
    bool tcp = conf & TXC_TCP;
    size_t at = l4 + (tcp ? 16 : 6);
    uint16_t sum;

    if (at + 2 > n) {
        return;
    }
    sum = net_raw_checksum(eth + l4, n - l4);
    stw_be_p(eth + at, tcp || sum ? sum : 0xffff);
}

/*
 * A data frame from the host, Ethernet after its descriptors: sent on
 * as 802.11, to the access point of a station or from the driver's own
 */
static void swt_tx_data(Swt6621sState *s, const uint8_t *d, size_t len)
{
    uint16_t w, conf;
    uint8_t inst;
    size_t off, n;
    g_autofree uint8_t *f = NULL;
    g_autofree uint8_t *eth = NULL;

    if (len < TXD_HDR_LEN + TXD_CONF_LEN + ETH_HDR_LEN) {
        return;
    }
    w = lduw_le_p(d);
    inst = (w >> TXD_INST_SHIFT) & 3;
    off = TXD_HDR_LEN + (w & TXD_GAP_MASK) + TXD_CONF_LEN;
    n = lduw_le_p(d + 2) & 0xfff;
    if (n < ETH_HDR_LEN || off + n > len) {
        return;
    }
    conf = lduw_le_p(d + off - TXD_CONF_LEN);
    eth = g_memdup2(d + off, n);
    if (conf & TXC_CSUM) {
        swt_csum(eth, n, conf);
    }
    trace_swt_data_out(inst, (w >> 8) & 0x1f, n);
    f = g_malloc0(WIFI_HDR_LEN + SNAP_LEN + n - ETH_HDR_LEN);
    if (s->bcn && inst == s->bcn_inst) {
        stw_le_p(f, WIFI_FC_DATA | WIFI_FC_FROMDS);
        memcpy(f + 4, eth, 6);
        memcpy(f + 10, s->bcn_bssid.a, 6);
        memcpy(f + 16, eth + 6, 6);
    } else if (swt_is_sta(s, inst)) {
        stw_le_p(f, WIFI_FC_DATA | WIFI_FC_TODS);
        memcpy(f + 4, s->inst[inst].bssid.a, 6);
        memcpy(f + 10, eth + 6, 6);
        memcpy(f + 16, eth, 6);
    } else {
        return;
    }
    memcpy(f + WIFI_HDR_LEN, WIFI_RFC1042, 6);
    memcpy(f + WIFI_HDR_LEN + 6, eth + 12, 2);
    memcpy(f + WIFI_HDR_LEN + SNAP_LEN, eth + ETH_HDR_LEN, n - ETH_HDR_LEN);
    swt_air(s, inst, f, WIFI_HDR_LEN + SNAP_LEN + n - ETH_HDR_LEN);
}

/* A station's authentication or association, as the firmware builds it */
static void swt_sta_frame(Swt6621sState *s, uint8_t inst, uint8_t stype,
                          const uint8_t *fixed, size_t fixed_len,
                          const uint8_t *ies, size_t ies_len)
{
    g_autofree uint8_t *f = g_malloc0(WIFI_HDR_LEN + fixed_len + ies_len);

    stw_le_p(f, WIFI_FC_MGMT(stype));
    memcpy(f + 4, s->inst[inst].bssid.a, 6);
    memcpy(f + 10, s->inst[inst].mac.a, 6);
    memcpy(f + 16, s->inst[inst].bssid.a, 6);
    memcpy(f + WIFI_HDR_LEN, fixed, fixed_len);
    memcpy(f + WIFI_HDR_LEN + fixed_len, ies, ies_len);
    swt_air(s, inst, f, WIFI_HDR_LEN + fixed_len + ies_len);
}

/* an element of the BSS the station joined, for its association */
static void swt_bss_ie(Swt6621sState *s, uint8_t inst, uint8_t eid,
                       GByteArray *out)
{
    GByteArray *b = s->inst[inst].bss_ies;

    for (size_t i = 0; b && i + 2 <= b->len &&
         i + 2 + b->data[i + 1] <= b->len; i += 2 + b->data[i + 1]) {
        if (b->data[i] == eid) {
            g_byte_array_append(out, b->data + i, 2 + b->data[i + 1]);
            return;
        }
    }
}

static const uint8_t *swt_span(const uint8_t *d, size_t n, size_t off_at,
                               size_t len_at, bool len32, size_t *len)
{
    size_t off = lduw_le_p(d + off_at);

    *len = len32 ? ldl_le_p(d + len_at) : lduw_le_p(d + len_at);
    if (!*len || off > n || *len > n - off) {
        *len = 0;
        return NULL;
    }
    return d + off;
}

static void swt_scan_done(void *opaque)
{
    Swt6621sState *s = opaque;

    if (s->scanning) {
        s->scanning = false;
        swt_event(s, EVENT_SCAN_CMPL, s->scan_inst, NULL, 0);
    }
}

/* A scan: the access point found where it is, by its beacon */
static void swt_scan(Swt6621sState *s, uint8_t inst, const uint8_t *d,
                     size_t n)
{
    uint32_t nr, off;
    uint8_t b[WIFI_AP_BEACON_MAX];
    bool hit = false;

    if (n < SCAN_CHAN_OFFSET + 4) {
        return;
    }
    nr = ldl_le_p(d + SCAN_NR_CHAN);
    off = ldl_le_p(d + SCAN_CHAN_OFFSET);
    for (uint32_t i = 0; i < nr && off + (i + 1) * SCAN_CHAN_LEN <= n; i++) {
        if (d[off + i * SCAN_CHAN_LEN] == s->ap.channel) {
            hit = true;
        }
    }
    s->scanning = true;
    s->scan_inst = inst;
    if (hit) {
        swt_mgmt_event(s, EVENT_SCAN_REPORT, inst, s->ap.channel, b,
                       wifi_ap_beacon(&s->ap, b));
    }
    timer_mod(s->scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SCAN_MS);
}

/* The driver's own BSS: its address and SSID from the beacon it gives */
static void swt_start_ap(Swt6621sState *s, uint8_t inst, const uint8_t *d,
                         size_t n)
{
    const uint8_t *head, *tail, *rsn = NULL;
    size_t hlen, tlen, ssid_len;

    if (n < AP_LEN || !s->client_nic) {
        return;
    }
    head = swt_span(d, n, AP_HEAD_OFFSET, AP_HEAD_LEN, false, &hlen);
    tail = swt_span(d, n, AP_TAIL_OFFSET, AP_TAIL_LEN, false, &tlen);
    if (!head || hlen < WIFI_HDR_LEN) {
        return;
    }
    for (size_t i = 0; tail && i + 2 <= tlen &&
         i + 2 + tail[i + 1] <= tlen; i += 2 + tail[i + 1]) {
        if (tail[i] == WIFI_EID_RSN) {
            rsn = tail + i;
            break;
        }
    }
    ssid_len = MIN(d[AP_SSID_LEN], 32);
    s->bcn = true;
    s->bcn_inst = inst;
    s->inst[inst].chan = d[6];
    memcpy(s->bcn_bssid.a, head + 16, 6);
    trace_swt_bss(ssid_len);
    wifi_client_bss(&s->client, s->bcn_bssid.a, d + AP_SSID, ssid_len, rsn);
}

static void swt_stop_ap(Swt6621sState *s, uint8_t inst)
{
    if (s->bcn && inst == s->bcn_inst) {
        s->bcn = false;
        trace_swt_bss(0);
        wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
        swt_peers_del(s, inst);
    }
}

static void swt_chip_info(Swt6621sState *s, uint8_t *ci)
{
    memset(ci, 0, CI_LEN);
    ci[CI_MAX_STA] = 8;
    ci[CI_MAX_MC] = 8;
    /* HT: SM power save off, short GI at 20 MHz; one stream of MCS 0-7 */
    stw_le_p(ci + CI_HT_CAPA, 0x002c);
    stw_le_p(ci + CI_HT_AMPDU, 0x17);
    stl_le_p(ci + CI_HT_TX_MCS, 0xff);
    stl_le_p(ci + CI_HT_RX_MCS, 0xff);
    ci[CI_MAX_SCAN_SSIDS] = 9;
    memcpy(ci + CI_MAC, s->conf.macaddr.a, 6);
    stl_le_p(ci + CI_BW_CAPA, BW_CAP_2G_20M);
    stl_le_p(ci + CI_PRIV, CI_PRIV_2G_ONLY | 1u << CI_PRIV_NR_HW_MAC);
    pstrcpy((char *)ci + CI_BUILD_TIME, 32, "QEMU");
    pstrcpy((char *)ci + CI_PLAT_VER, 16, "H25.34.7.1");
    pstrcpy((char *)ci + CI_WIFI_VER, 16, "F25.34.6.1");
    stl_le_p(ci + CI_CHIP_TYPE, CHIP_TYPE_SWT6621S);
}

/* A command from the host, answered at once; some go on the air */
static void swt_cmd(Swt6621sState *s, const uint8_t *m, size_t len)
{
    uint8_t inst = m[MSG_TYPE_INST] & 0xf, id = m[MSG_ID];
    const uint8_t *d = m + MSG_HDR_LEN, *ies;
    size_t n, ies_len;
    uint8_t rsp[CI_LEN];
    int peer, mc;

    if (len < MSG_HDR_LEN) {
        return;
    }
    n = MIN(lduw_le_p(m + MSG_TOTAL_LEN), len);
    n = n > MSG_HDR_LEN ? n - MSG_HDR_LEN : 0;
    trace_swt_cmd(id, inst, lduw_le_p(m + MSG_SEQ), n);
    if (inst >= NR_INST) {
        swt_ack(s, m, NULL, 0);
        return;
    }
    switch (id) {
    case CMD_GET_INFO:
        swt_chip_info(s, rsp);
        swt_ack(s, m, rsp, CI_LEN);
        return;
    case CMD_SYN_VERSION: {
        g_autofree uint8_t *v = g_malloc0(VERSION_INFO_LEN);

        swt_ack(s, m, v, VERSION_INFO_LEN);
        return;
    }
    case CMD_OPEN_DEV:
        if (n >= OPEN_MAC + 6) {
            s->inst[inst].mode = lduw_le_p(d + OPEN_MODE);
            memcpy(s->inst[inst].mac.a, d + OPEN_MAC, 6);
        }
        break;
    case CMD_SET_MAC:
        /* SKW_CMD_RANDOM_MAC: the interface's new address, as set */
        if (n >= 6) {
            memcpy(s->inst[inst].mac.a, d, 6);
        }
        break;
    case CMD_CLOSE_DEV:
        swt_stop_ap(s, inst);
        swt_peers_del(s, inst);
        s->mac_users &= ~BIT(inst);
        s->inst[inst].mode = 0;
        memset(s->inst[inst].bssid.a, 0, 6);
        break;
    case CMD_START_SCAN:
        swt_ack(s, m, NULL, 0);
        swt_scan(s, inst, d, n);
        return;
    case CMD_STOP_SCAN:
        swt_ack(s, m, NULL, 0);
        timer_del(s->scan);
        swt_scan_done(s);
        return;
    case CMD_JOIN:
        if (n < JOIN_LEN) {
            break;
        }
        memcpy(s->inst[inst].bssid.a, d + JOIN_BSSID, 6);
        s->inst[inst].chan = d[JOIN_CHAN];
        s->inst[inst].capability = lduw_le_p(d + JOIN_CAPABILITY);
        ies = swt_span(d, n, JOIN_IE_OFFSET, JOIN_IE_LEN, true, &ies_len);
        g_byte_array_set_size(s->inst[inst].bss_ies, 0);
        if (ies) {
            g_byte_array_append(s->inst[inst].bss_ies, ies, ies_len);
        }
        swt_peers_del(s, inst);
        peer = swt_peer_add(s, inst, d + JOIN_BSSID);
        mc = swt_peer_add(s, inst, (const uint8_t[6]) { 0xff, 0xff, 0xff,
                                                        0xff, 0xff, 0xff });
        s->inst[inst].peer = MAX(peer, 0);
        /* struct skw_join_resp: peer, MAC, instance, multicast peer */
        rsp[0] = MAX(peer, 0);
        rsp[1] = 0;
        rsp[2] = inst;
        rsp[3] = MAX(mc, 0);
        swt_ack(s, m, rsp, 4);
        swt_mac_bind(s, inst);
        return;
    case CMD_AUTH:
        swt_ack(s, m, NULL, 0);
        if (n >= AUTH_LEN && swt_is_sta(s, inst)) {
            g_autoptr(GByteArray) e = g_byte_array_new();
            uint8_t fixed[6];

            stw_le_p(fixed, lduw_le_p(d + AUTH_ALG));
            stw_le_p(fixed + 2, 1);
            stw_le_p(fixed + 4, 0);
            ies = swt_span(d, n, AUTH_DATA_OFFSET, AUTH_DATA_OFFSET + 2,
                           false, &ies_len);
            if (ies) {
                g_byte_array_append(e, ies, ies_len);
            }
            ies = swt_span(d, n, AUTH_IE_OFFSET, AUTH_IE_OFFSET + 2, false,
                           &ies_len);
            if (ies) {
                g_byte_array_append(e, ies, ies_len);
            }
            swt_sta_frame(s, inst, WIFI_STYPE_AUTH, fixed, sizeof(fixed),
                          e->data, e->len);
        }
        return;
    case CMD_ASSOC:
        swt_ack(s, m, NULL, 0);
        if (n >= ASSOC_LEN && swt_is_sta(s, inst)) {
            g_autoptr(GByteArray) e = g_byte_array_new();
            uint8_t fixed[4];

            stw_le_p(fixed, s->inst[inst].capability);
            stw_le_p(fixed + 2, 10);
            /* what the host leaves to the firmware: SSID and rates */
            swt_bss_ie(s, inst, WIFI_EID_SSID, e);
            swt_bss_ie(s, inst, 1, e);
            swt_bss_ie(s, inst, 50, e);
            ies = swt_span(d, n, ASSOC_IE_OFFSET, ASSOC_IE_OFFSET + 2,
                           false, &ies_len);
            if (ies) {
                g_byte_array_append(e, ies, ies_len);
            }
            swt_sta_frame(s, inst, WIFI_STYPE_ASSOC_REQ, fixed,
                          sizeof(fixed), e->data, e->len);
        }
        return;
    case CMD_TX_MGMT:
        swt_ack(s, m, NULL, 0);
        if (n >= MGMT_TX_LEN) {
            size_t fl = MIN(lduw_le_p(d + MGMT_TX_FRAME_LEN),
                            n - MGMT_TX_LEN);
            g_autofree uint8_t *st = g_malloc0(12 + fl);
            g_autofree uint8_t *f = g_memdup2(d + MGMT_TX_LEN, fl);

            /*
             * struct skw_tx_mgmt_status: cookie, acked, the frame. The
             * ACK comes before any answer to the frame: hostapd takes a
             * station as authenticated at the status of its response
             */
            memcpy(st, d + MGMT_TX_COOKIE, 8);
            stw_le_p(st + 8, fl >= WIFI_HDR_LEN && swt_acked(s, inst, f));
            stw_le_p(st + 10, fl);
            memcpy(st + 12, d + MGMT_TX_LEN, fl);
            swt_event(s, EVENT_MGMT_TX_STATUS, inst, st, 12 + fl);
            if (fl >= WIFI_HDR_LEN) {
                swt_air(s, inst, f, fl);
            }
        }
        return;
    case CMD_TX_DATA_FRAME:
        swt_ack(s, m, NULL, 0);
        swt_tx_data(s, d, n);
        return;
    case CMD_DISCONNECT:
        swt_ack(s, m, NULL, 0);
        if (swt_is_sta(s, inst)) {
            if (n >= 4 && !d[DISCONNECT_LOCAL]) {
                uint8_t reason[2];

                stw_le_p(reason, lduw_le_p(d + 2));
                swt_sta_frame(s, inst, WIFI_STYPE_DEAUTH, reason,
                              sizeof(reason), NULL, 0);
            }
            swt_peers_del(s, inst);
            memset(s->inst[inst].bssid.a, 0, 6);
        }
        s->mac_users &= ~BIT(inst);
        return;
    case CMD_START_AP:
        swt_peers_del(s, inst);
        swt_start_ap(s, inst, d, n);
        mc = swt_peer_add(s, inst, (const uint8_t[6]) { 0xff, 0xff, 0xff,
                                                        0xff, 0xff, 0xff });
        /* struct skw_startap_resp: MAC, instance, multicast peer */
        rsp[0] = 0;
        rsp[1] = inst;
        rsp[2] = MAX(mc, 0);
        swt_ack(s, m, rsp, 3);
        swt_mac_bind(s, inst);
        return;
    case CMD_STOP_AP:
        swt_stop_ap(s, inst);
        s->mac_users &= ~BIT(inst);
        break;
    case CMD_ADD_STA:
        rsp[0] = n >= 6 ? MAX(swt_peer_add(s, inst, d), 0) : 0;
        swt_ack(s, m, rsp, 1);
        return;
    case CMD_DEL_STA:
        if (n >= DEL_STA_LEN) {
            swt_peer_del(s, inst, d);
            if (d[DEL_STA_TX_FRAME]) {
                swt_ap_deauth(s, inst, d, lduw_le_p(d + DEL_STA_REASON));
            }
        }
        break;
    }
    swt_ack(s, m, NULL, 0);
}

/*
 * What the host sends a port: messages or frames, each after the word
 * with its length; a transfer may hold several, or end inside one
 */
static void swt_take(Swt6621sState *s, GByteArray *b, USBPacket *p,
                     bool data)
{
    size_t len = b->len, used = 0, n;
    unsigned frames = 0;

    if (len + p->iov.size > OUT_MAX) {
        g_byte_array_set_size(b, 0);
        return;
    }
    g_byte_array_set_size(b, len + p->iov.size);
    usb_packet_copy(p, b->data + len, p->iov.size);
    while (b->len - used >= XHDR_LEN) {
        n = (ldl_le_p(b->data + used) >> XHDR_LEN_SHIFT) & XHDR_LEN_MASK;
        if (!n) {
            /* padding the host's alignment left */
            used = b->len;
            break;
        }
        if (b->len - used < XHDR_LEN + n) {
            break;
        }
        if (data) {
            swt_tx_data(s, b->data + used + XHDR_LEN, n);
            frames++;
        } else {
            swt_cmd(s, b->data + used + XHDR_LEN, n);
        }
        used += XHDR_LEN + n;
    }
    g_byte_array_remove_range(b, 0, used);
    if (frames) {
        swt_credit(s, frames);
    }
}

/*
 * The data port's IN: the packets queued, messages and data in transfers
 * of their own kind, the first packet numbering the transfer
 */
static void swt_give(Swt6621sState *s, USBPacket *p)
{
    g_autoptr(GByteArray) x = g_byte_array_new();
    GByteArray *u;
    uint32_t kind;
    unsigned count = 0;

    u = g_queue_peek_head(&s->d2h);
    if (!u) {
        p->status = USB_RET_NAK;
        return;
    }
    kind = ldl_le_p(u->data + PKT_W2) & PKT_IS_MSG;
    while ((u = g_queue_peek_head(&s->d2h)) && count < PACKET_COUNT &&
           (ldl_le_p(u->data + PKT_W2) & PKT_IS_MSG) == kind &&
           x->len + u->len <= MIN(p->iov.size, XFER_MAX) &&
           u->len <= PKT_MAX) {
        g_queue_pop_head(&s->d2h);
        g_byte_array_append(x, u->data, u->len);
        g_byte_array_unref(u);
        count++;
    }
    if (!count) {
        /* larger than the host takes: dropped, not stuck in the queue */
        g_byte_array_unref(g_queue_pop_head(&s->d2h));
        p->status = USB_RET_NAK;
        return;
    }
    stl_le_p(x->data + PKT_W0, XFER_VALID | count << 16 | s->xfer_seq++);
    trace_swt_d2h(kind, count, x->len);
    usb_packet_copy(p, x->data, x->len);
    /* room again: what the netdevs held back may come */
    qemu_flush_queued_packets(qemu_get_queue(s->nic));
    if (s->client_nic) {
        qemu_flush_queued_packets(qemu_get_queue(s->client_nic));
    }
}

static void swt_give_text(GQueue *q, USBPacket *p)
{
    GByteArray *u = g_queue_pop_head(q);

    if (!u) {
        p->status = USB_RET_NAK;
        return;
    }
    usb_packet_copy(p, u->data, MIN(u->len, p->iov.size));
    g_byte_array_unref(u);
}

/* The boot ROM: a packet per transfer, to its short end */
static void swt_boot_take(Swt6621sState *s, USBPacket *p)
{
    size_t len = s->boot_out->len;

    if (len + p->iov.size > BOOT_OUT_MAX) {
        g_byte_array_set_size(s->boot_out, 0);
        return;
    }
    g_byte_array_set_size(s->boot_out, len + p->iov.size);
    usb_packet_copy(p, s->boot_out->data + len, p->iov.size);
    if (p->iov.size % PORT_MPS == 0 && p->iov.size) {
        return;
    }
    swt_boot(s, s->boot_out->data, s->boot_out->len);
    g_byte_array_set_size(s->boot_out, 0);
}

static void swt_boot_give(Swt6621sState *s, USBPacket *p)
{
    swt_give_text(&s->boot_in, p);
    /* told to run the firmware, and its answer read: off the bus */
    if (s->exec && g_queue_is_empty(&s->boot_in)) {
        timer_mod(s->replug, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                             LEAVE_MS);
    }
}

static void swt_handle_data(USBDevice *dev, USBPacket *p)
{
    Swt6621sState *s = SWT6621S(dev);
    uint8_t ep = p->ep->nr | (p->pid == USB_TOKEN_IN ? USB_DIR_IN : 0);

    if (!s->running) {
        if (ep == (USB_DIR_IN | EP_CMD)) {
            swt_boot_give(s, p);
        } else if (ep == EP_CMD) {
            swt_boot_take(s, p);
        } else {
            p->status = USB_RET_STALL;
        }
        return;
    }
    switch (ep) {
    case EP_CMD:
        swt_take(s, s->cmd_out, p, false);
        return;
    case EP_DATA:
        swt_take(s, s->data_out, p, true);
        return;
    case USB_DIR_IN | EP_DATA:
        swt_give(s, p);
        return;
    case USB_DIR_IN | EP_LOOP:
        swt_give_text(&s->loop_in, p);
        return;
    case EP_LOOP:
    case EP_LOG:
        /* the host's answers on the side channel, and the log's switch */
        usb_packet_skip(p, p->iov.size);
        return;
    case USB_DIR_IN | EP_CMD:
    case USB_DIR_IN | EP_LOG:
        p->status = USB_RET_NAK;
        return;
    }
    p->status = USB_RET_STALL;
}

static void swt_loop_say(Swt6621sState *s, const char *line)
{
    trace_swt_loop(line);
    swt_queue(&s->loop_in, line, strlen(line));
}

static void swt_handle_control(USBDevice *dev, USBPacket *p, int request,
                               int value, int index, int length,
                               uint8_t *data)
{
    Swt6621sState *s = SWT6621S(dev);

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        /*
         * Configured, the firmware gives its version on the side
         * channel, which has the host add the WiFi device, and WiFi up
         */
        if (request == (DeviceOutRequest | USB_REQ_SET_CONFIGURATION) &&
            value && s->running) {
            swt_loop_say(s, LOOP_VERSION);
            swt_loop_say(s, LOOP_WIFI_READY);
        }
        return;
    }
    trace_swt_ctrl(request, value, index);
    switch (request) {
    case VendorDeviceOutRequest | VENDOR_SERVICE:
        if (s->running && !(index & 1)) {
            swt_loop_say(s, LOOP_WIFI_READY);
        }
        return;
    case VendorDeviceRequest | VENDOR_PACKET_COUNT:
        if (length >= 2) {
            stw_le_p(data, PACKET_COUNT);
            p->actual_length = 2;
        }
        return;
    }
    if ((request & ~0xff) == VendorDeviceOutRequest) {
        return;
    }
    p->status = USB_RET_STALL;
}

/* A bus reset leaves the CPU, and whatever it runs, alone */
static void swt_handle_reset(USBDevice *dev)
{
    Swt6621sState *s = SWT6621S(dev);

    g_byte_array_set_size(s->boot_out, 0);
    g_byte_array_set_size(s->cmd_out, 0);
    g_byte_array_set_size(s->data_out, 0);
    swt_flush_queue(&s->d2h);
}

/* Power on: the CPU waiting in its ROM, the chip on the bus */
static void swt_power_on(DeviceState *d)
{
    Swt6621sState *s = SWT6621S(d);
    USBDevice *dev = USB_DEVICE(d);

    swt_flush(s);
    s->running = false;
    s->exec = false;
    s->xfer_seq = 0;
    s->event_seq = 0;
    s->mac_users = 0;
    s->scanning = false;
    s->bcn = false;
    memset(s->peers, 0, sizeof(s->peers));
    for (int i = 0; i < NR_INST; i++) {
        s->inst[i].mode = 0;
        memset(s->inst[i].bssid.a, 0, 6);
        g_byte_array_set_size(s->inst[i].bss_ies, 0);
    }
    timer_del(s->replug);
    timer_del(s->scan);
    dev->usb_desc = NULL;
    if (dev->attached) {
        usb_desc_attach(dev);
    } else {
        timer_mod(s->replug, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    }
}

static bool swt_can_receive(NetClientState *nc)
{
    Swt6621sState *s = qemu_get_nic_opaque(nc);

    return s->ap.state == WIFI_AP_CONNECTED &&
           g_queue_get_length(&s->d2h) < D2H_QUEUE_MAX / 2;
}

static ssize_t swt_receive(NetClientState *nc, const uint8_t *buf,
                           size_t size)
{
    Swt6621sState *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
    return size;
}

static NetClientInfo swt_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = swt_can_receive,
    .receive = swt_receive,
};

static bool swt_client_can_receive(NetClientState *nc)
{
    Swt6621sState *s = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&s->client) &&
           g_queue_get_length(&s->d2h) < D2H_QUEUE_MAX / 2;
}

static ssize_t swt_client_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    Swt6621sState *s = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&s->client, buf, size);
    return size;
}

static NetClientInfo swt_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = swt_client_can_receive,
    .receive = swt_client_receive,
};

static void swt_realize(USBDevice *dev, Error **errp)
{
    Swt6621sState *s = SWT6621S(dev);
    DeviceState *d = DEVICE(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&swt_net_info, &s->conf,
                          object_get_typename(OBJECT(d)), d->id,
                          &d->mem_reentrancy_guard, s);
    if (!wifi_ap_realize(&s->ap, s->nic, swt_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
        return;
    }
    if (s->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&s->client_conf.macaddr);
        s->client_nic = qemu_new_nic(&swt_client_net_info, &s->client_conf,
                                     object_get_typename(OBJECT(d)), NULL,
                                     &d->mem_reentrancy_guard, s);
        wifi_client_init(&s->client, s->client_nic,
                         &s->client_conf.macaddr, swt_client_deliver, s);
    }
}

static void swt_unrealize(USBDevice *dev)
{
    Swt6621sState *s = SWT6621S(dev);

    timer_del(s->replug);
    timer_del(s->scan);
    if (s->client_nic) {
        wifi_client_cleanup(&s->client);
        qemu_del_nic(s->client_nic);
        s->client_nic = NULL;
    }
    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void swt_init(Object *obj)
{
    Swt6621sState *s = SWT6621S(obj);

    s->boot_out = g_byte_array_new();
    s->cmd_out = g_byte_array_new();
    s->data_out = g_byte_array_new();
    g_queue_init(&s->boot_in);
    g_queue_init(&s->d2h);
    g_queue_init(&s->loop_in);
    for (int i = 0; i < NR_INST; i++) {
        s->inst[i].bss_ies = g_byte_array_new();
    }
    s->replug = timer_new_ms(QEMU_CLOCK_VIRTUAL, swt_replug, s);
    s->scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, swt_scan_done, s);
}

static void swt_finalize(Object *obj)
{
    Swt6621sState *s = SWT6621S(obj);

    swt_flush(s);
    g_byte_array_unref(s->boot_out);
    g_byte_array_unref(s->cmd_out);
    g_byte_array_unref(s->data_out);
    for (int i = 0; i < NR_INST; i++) {
        g_byte_array_unref(s->inst[i].bss_ies);
    }
    timer_free(s->replug);
    timer_free(s->scan);
}

static const Property swt_props[] = {
    DEFINE_NIC_PROPERTIES(Swt6621sState, conf),
    DEFINE_PROP_STRING("ssid", Swt6621sState, ap.ssid),
    DEFINE_PROP_STRING("psk", Swt6621sState, ap.psk),
    DEFINE_PROP_UINT8("channel", Swt6621sState, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", Swt6621sState, ap.bssid),
    DEFINE_PROP_NETDEV("client-netdev", Swt6621sState, client_conf.peers),
    DEFINE_PROP_MACADDR("client-mac", Swt6621sState, client_conf.macaddr),
};

/* the firmware's state and the queued packets are not migrated */
static const VMStateDescription swt_vmstate = {
    .name = TYPE_SWT6621S,
    .unmigratable = 1,
};

static void swt_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->product_desc = "Seekwave SWT6621S USB Wi-Fi";
    uc->usb_desc = &swt_desc_boot;
    uc->realize = swt_realize;
    uc->unrealize = swt_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = swt_handle_reset;
    uc->handle_control = swt_handle_control;
    uc->handle_data = swt_handle_data;
    dc->desc = "Seekwave SWT6621S USB Wi-Fi";
    dc->vmsd = &swt_vmstate;
    device_class_set_legacy_reset(dc, swt_power_on);
    device_class_set_props(dc, swt_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo swt_info = {
    .name          = TYPE_SWT6621S,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(Swt6621sState),
    .instance_init = swt_init,
    .instance_finalize = swt_finalize,
    .class_init    = swt_class_init,
};

static void swt_register_types(void)
{
    type_register_static(&swt_info);
}

type_init(swt_register_types)
