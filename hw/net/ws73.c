/*
 * HiSilicon WS73V100 Wi-Fi on USB
 *
 * The chip as its vendor driver (ws73v100_plat and ws73v100) sees it. Its
 * boot ROM takes the firmware as text commands on two bulk pipes, and on
 * QUIT leaves the bus to come back running it, with five endpoints; the
 * firmware is taken and dropped, and this model answers in its place. The
 * host runs hmac and most of the HAL, and talks to dmac in HCC packets:
 * the model answers its configuration messages, raises the interrupt
 * pipe's notifications, and keeps the registers the register pipe and
 * those messages reach. Frames go both ways with their control blocks; an
 * access point (wifi-ap.c) is on the air to join, and a client
 * (wifi-client.c) joins the driver's own.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/guest-random.h"
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

#define TYPE_WS73 "ws73v100"
OBJECT_DECLARE_SIMPLE_TYPE(Ws73State, WS73)

#define EP_BULK                 1
#define EP_BULK_OUT             2
#define EP_INT                  3
#define EP_REG_OUT              4
#define EP_REG_IN               5
#define OUT_MAX                 (64 * 1024)

/* after QUIT the chip leaves the bus, then comes back running */
#define LEAVE_MS                20
#define REATTACH_MS             200

/* struct usb_package: what a transfer either way starts with */
#define PKG_TYPE                0
#define PKG_LEN                 4
#define PKG_RESERVE             8
#define PKG_SINGLE              0
#define PKG_AGGR                1
#define PKG_D2H                 2
#define PKG_CONTROL             3
/* host to device: the package and its descriptor, then packets of >= 64 */
#define H2D_HDR_LEN             64
#define H2D_MIN_PKT             64
/* device to host: the package, a length for each packet, then the packets */
#define D2H_HDR_LEN             92
#define D2H_AGGR                12
#define D2H_AGGR_MAX            24
#define D2H_XFER_MAX            (20 * 1024)

/* struct usb_dev_notification on the interrupt pipe */
#define NOTIFY_LEN              8
#define D2H_BSP_READY           0
#define D2H_WLAN_READY          1
#define H2D_WLAN_OPEN           0

/* class requests to the interface */
#define REQ_SEND_MSG            0
#define REQ_GET_CREDIT          5
#define CREDIT                  64

/* the register pipe: a setup, then the value either way */
#define REG_SETUP_LEN           16
#define REG_SETUP_OFF           8
#define REG_SETUP_CTRL          12
#define REG_DIR_WRITE           BIT(0)
/* the eFuse controller: always booted, idle, and a read done */
#define REG_EFUSE_ST            0x8018
#define EFUSE_ST_READY          (BIT(1) | BIT(2))
/*
 * where dmac publishes its free buffers, the host's credit for data:
 * none reserved, 64 large, 32 short, 16 for management frames
 */
#define REG_GP0                 0x40019380
#define GP0_FREE                (64 << 8 | 32 << 16 | 16 << 22)

/* hcc_header: service and sub-type, queue, the packet's whole length */
#define HCC_HDR_LEN             4
#define HCC_TYPE                0
#define HCC_QUEUE               1
#define HCC_LEN                 2
#define SRV_OAM                 1
#define SRV_CUSTOMIZE           4
#define SRV_WIFI                6
#define OAM_PLAT_MSG            9
#define CUSTOM_CFGID_MSG        4
#define CTRL_QUEUE              0
#define DATA_HI_QUEUE           1
#define DATA_LO_QUEUE           2
#define INTERNAL_QUEUE          9
#define HCC_ALIGN               32

/*
 * hcc_plat_cfg_msg_header after it for the platform's messages. The
 * TRNG's answer: a result, the count, the bytes, all within 88 bytes
 */
#define PLAT_MSG_ID             4
#define PLAT_LEN                6
#define PLAT_DATA               8
#define PLAT_H2D_TRNG           5
#define PLAT_D2H_TRNG           0
#define TRNG_HEAD               8
#define TRNG_MAX                80

/* frw_hcc_extend_hdr after it, then frw_msg_adapt_comm for a message */
#define EXT_LEN                 8
#define EXT_MSG_ID              0
#define EXT_VAP_TYPE            2
#define EXT_DATA_LEN            3
#define MSG_CFG                 0
#define MSG_RSP                 1
#define MSG_MGMT                2
#define MSG_DATA                3
/* the netbuf messages to hmac */
#define NETBUF_MGMT             0
#define NETBUF_DATA             1
#define AD_LEN                  28
#define AD_MSG_ID               0
#define AD_VAP                  2
#define AD_FLAGS                3
#define AD_CB_RET               4
#define AD_SEQ                  8
#define AD_DATA_LEN             20
#define AD_RSP_BUF_LEN          22
#define AD_RSP_LEN              24
#define AD_CFG_FLAGS            26
#define AD_SYNC                 BIT(0)
#define MSG_HDR_LEN             (HCC_HDR_LEN + EXT_LEN + AD_LEN)

/*
 * A frame either way: the headers, its control block, then the 802.11
 * header at the start of a slot of its own and the body after the slot
 */
#define FRM_CB_LEN              32
#define FRM_HDR_SLOT            36
#define FRM_LEN                 (HCC_HDR_LEN + EXT_LEN + FRM_CB_LEN + \
                                 FRM_HDR_SLOT)
#define TXC_HDR_LEN             5
#define RXC_VAP                 0
#define RXC_MSDUS               1
#define RXC_USER                2
#define RXC_USER_MASK           0x1f
/* past hmac's users: a transmitter dmac knows no user for */
#define RXC_USER_NONE           RXC_USER_MASK
#define RXC_HDR_LEN             3
#define RXC_BEACON              BIT(6)
#define RXC_LAST                BIT(7)
#define RXC_FRAME_LEN           4
#define RXC_MAC_VAP             6
#define RXC_CHANNEL             7
#define RXC_STATUS              20
#define RX_OK_PLAIN             (1 << 4 | 3)
#define RX_OK_CCMP              (1 << 4 | 2)
#define RXC_RSSI                24
#define RX_RSSI                 -40

#define D2H_QUEUE_MAX           64
#define BEACON_MS               102

/* dmac's configuration messages the model acts on */
#define MSG_ADD_VAP             82
#define ADD_VAP_ID              4
#define ADD_VAP_HAL_ID          5
#define MSG_DEL_VAP             84
#define MSG_SYNC_VAP            85
#define VAP_STATE_INIT          0
#define MSG_ADD_USER            91
#define MSG_DEL_USER            92
#define MSG_SEND_READY          131
#define MSG_REG_WRITE           134
#define MSG_REG_READ            135
#define MSG_REG_WRITE_BITS      136
#define MSG_SWITCH_CHANNEL      172
#define MSG_SEND_BEACON         133
#define BEACON_LEN              768
#define USERS_MAX               8

struct Ws73State {
    USBDevice parent_obj;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;
    QEMUTimer *replug;
    QEMUTimer *beacon;

    bool running;               /* the firmware: the five-endpoint device */
    uint32_t files_left;        /* bytes of a FILES piece still to come */
    GByteArray *out;
    GQueue boot_in;             /* the boot ROM's answers */

    GQueue d2h;                 /* HCC packets for bulk IN */
    uint32_t notify;            /* D2H_MSG bits for the interrupt pipe */
    bool reg_data;              /* the next register OUT is a value */
    bool reg_read;              /* the next register IN answers a read */
    uint16_t reg_addr;
    GHashTable *regs;           /* SoC, MAC and PHY registers by address */

    uint8_t vap;                /* hmac's VAP the frames are for */
    uint8_t hal_vap[16];        /* the MAC's VAP for each of hmac's */
    uint8_t chan;               /* the channel hmac last switched to */
    struct {
        bool valid;
        uint16_t idx;
        MACAddr mac;
    } users[USERS_MAX];         /* hmac's users, by what dmac was told */

    /* the driver's own BSS, as its beacon names it */
    bool bcn;
    uint8_t bcn_vap;
    MACAddr bcn_bssid;
    uint8_t bcn_ssid[32];
    uint8_t bcn_ssid_len;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings ws_strings = {
    [STR_MANUFACTURER]  = "HiSilicon",
    [STR_PRODUCT]       = "WS73V100",
    [STR_SERIALNUMBER]  = "1",
};

/* the driver takes endpoints by their place: bulk IN first, then OUT */
static const USBDescIface ws_iface_boot = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | EP_BULK,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | EP_BULK_OUT,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        },
    },
};

static const USBDescIface ws_iface_fw = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 5,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | EP_BULK,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | EP_BULK_OUT,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_IN | EP_INT,
            .bmAttributes          = USB_ENDPOINT_XFER_INT,
            .wMaxPacketSize        = NOTIFY_LEN,
            .bInterval             = 4,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | EP_REG_OUT,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        }, {
            .bEndpointAddress      = USB_DIR_IN | EP_REG_IN,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 512,
        },
    },
};

#define WS_DEVICE(_iface)                                                   \
    (const USBDescDevice) {                                                 \
        .bcdUSB                    = 0x0200,                                \
        .bMaxPacketSize0           = 64,                                    \
        .bNumConfigurations        = 1,                                     \
        .confs = (USBDescConfig[]) {                                        \
            {                                                               \
                .bNumInterfaces    = 1,                                     \
                .bConfigurationValue = 1,                                   \
                .bmAttributes      = USB_CFG_ATT_ONE,                       \
                .bMaxPower         = 250,                                   \
                .nif = 1,                                                   \
                .ifs = (_iface),                                            \
            },                                                              \
        },                                                                  \
    }

static const USBDescDevice ws_device_boot = WS_DEVICE(&ws_iface_boot);
static const USBDescDevice ws_device_fw = WS_DEVICE(&ws_iface_fw);

#define WS_DESC(_dev)                                                       \
    (const USBDesc) {                                                       \
        .id = {                                                             \
            .idVendor              = 0xffff,                                \
            .idProduct             = 0x3733,                                \
            .bcdDevice             = 0x0100,                                \
            .iManufacturer         = STR_MANUFACTURER,                      \
            .iProduct              = STR_PRODUCT,                           \
            .iSerialNumber         = STR_SERIALNUMBER,                      \
        },                                                                  \
        .high = (_dev),                                                     \
        .str  = ws_strings,                                                 \
    }

static const USBDesc ws_desc_boot = WS_DESC(&ws_device_boot);
static const USBDesc ws_desc_fw = WS_DESC(&ws_device_fw);

/* the register pipe reaches only the MAC block */
#define REG_PIPE_BASE           0x40010000

static uint32_t ws_reg_get(Ws73State *s, uint32_t addr)
{
    if (addr == REG_PIPE_BASE + REG_EFUSE_ST) {
        return EFUSE_ST_READY;
    }
    if (addr == REG_GP0) {
        return GP0_FREE;
    }
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->regs,
                                                GUINT_TO_POINTER(addr & ~3)));
}

static void ws_reg_set(Ws73State *s, uint32_t addr, uint32_t val)
{
    g_hash_table_insert(s->regs, GUINT_TO_POINTER(addr & ~3),
                        GUINT_TO_POINTER(val));
}

static void ws_flush(Ws73State *s)
{
    GByteArray *u;

    g_byte_array_set_size(s->out, 0);
    while ((u = g_queue_pop_head(&s->boot_in))) {
        g_byte_array_unref(u);
    }
    while ((u = g_queue_pop_head(&s->d2h))) {
        g_byte_array_unref(u);
    }
    s->files_left = 0;
    s->notify = 0;
    s->reg_data = false;
    s->reg_read = false;
}

static void ws_boot_answer(Ws73State *s, const char *text)
{
    GByteArray *u = g_byte_array_new();

    g_byte_array_append(u, (const uint8_t *)text, strlen(text) + 1);
    g_queue_push_tail(&s->boot_in, u);
}

/* The boot ROM's text commands; a FILES piece follows its command */
static void ws_boot(Ws73State *s, const uint8_t *buf, size_t len)
{
    g_autofree char *cmd = g_strndup((const char *)buf, len);
    unsigned long count, addr, size;

    trace_ws73_boot(cmd);
    if (g_str_has_prefix(cmd, "VERSION")) {
        ws_boot_answer(s, "WS73V100");
    } else if (g_str_has_prefix(cmd, "FILES")) {
        if (sscanf(cmd + 5, " %lu 0x%lx 0x%lx", &count, &addr, &size) == 3) {
            s->files_left = size;
        }
        ws_boot_answer(s, "READY");
    } else if (g_str_has_prefix(cmd, "WRITEM")) {
        ws_boot_answer(s, "WRITEM OK");
    } else if (g_str_has_prefix(cmd, "JUMP")) {
        ws_boot_answer(s, "JUMP OK");
    } else if (g_str_has_prefix(cmd, "SET")) {
        ws_boot_answer(s, "SET OK");
    } else if (g_str_has_prefix(cmd, "QUIT")) {
        timer_mod(s->replug,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + LEAVE_MS);
    }
}

/* Off the bus, then back on it with the firmware's endpoints */
static void ws_replug(void *opaque)
{
    Ws73State *s = opaque;
    USBDevice *dev = USB_DEVICE(s);
    Error *err = NULL;

    trace_ws73_replug(dev->attached);
    if (dev->attached) {
        usb_device_detach(dev);
        ws_flush(s);
        s->running = true;
        dev->usb_desc = &ws_desc_fw;
        timer_mod(s->replug,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + REATTACH_MS);
        return;
    }
    usb_device_attach(dev, &err);
    if (err) {
        warn_report_err(err);
    }
}

/* A packet for the host; its HCC header names its length */
static GByteArray *ws_packet(uint8_t service, uint8_t sub, uint8_t queue,
                             size_t len)
{
    GByteArray *u = g_byte_array_new();
    size_t n = ROUND_UP(len, HCC_ALIGN);

    g_byte_array_set_size(u, n);
    memset(u->data, 0, n);
    u->data[HCC_TYPE] = service << 4 | sub;
    u->data[HCC_QUEUE] = queue;
    stw_le_p(u->data + HCC_LEN, n);
    return u;
}

static void ws_send(Ws73State *s, GByteArray *u)
{
    trace_ws73_d2h(u->data[HCC_TYPE], u->data[HCC_QUEUE], u->len);
    if (g_queue_get_length(&s->d2h) >= D2H_QUEUE_MAX) {
        g_byte_array_unref(u);
        return;
    }
    g_queue_push_tail(&s->d2h, u);
}

/* dmac's answer to a configuration message: what was asked, done */
static void ws_cfg_answer(Ws73State *s, const uint8_t *req,
                          const uint8_t *rsp, size_t rsp_len)
{
    const uint8_t *ad = req + HCC_HDR_LEN + EXT_LEN;
    uint16_t buf_len = lduw_le_p(ad + AD_RSP_BUF_LEN);
    GByteArray *u;
    uint8_t *ext, *a;

    rsp_len = MIN(rsp_len, buf_len);
    u = ws_packet(SRV_CUSTOMIZE, CUSTOM_CFGID_MSG, CTRL_QUEUE,
                  MSG_HDR_LEN + rsp_len);
    ext = u->data + HCC_HDR_LEN;
    a = ext + EXT_LEN;
    memcpy(ext, req + HCC_HDR_LEN, EXT_LEN);
    ext[EXT_VAP_TYPE] = (ext[EXT_VAP_TYPE] & 0xf) | MSG_RSP << 4;
    stw_le_p(ext + EXT_DATA_LEN, AD_LEN + rsp_len);
    memcpy(a, ad, AD_LEN);
    a[AD_FLAGS] = (a[AD_FLAGS] & ~0xe) | MSG_RSP << 1;
    stl_le_p(a + AD_CB_RET, 0);
    stw_le_p(a + AD_DATA_LEN, 0);
    stw_le_p(a + AD_RSP_LEN, rsp_len);
    if (rsp_len) {
        memcpy(a + AD_LEN, rsp, rsp_len);
    }
    ws_send(s, u);
}

/* The users dmac is told of: whose frames are whose */
static void ws_user(Ws73State *s, const uint8_t *d, size_t n, bool add)
{
    unsigned i, free = USERS_MAX;
    uint16_t idx;

    if (n < 8) {
        return;
    }
    idx = lduw_le_p(d);
    for (i = 0; i < USERS_MAX; i++) {
        if (s->users[i].valid && s->users[i].idx == idx) {
            s->users[i].valid = false;
        }
        if (!s->users[i].valid && free == USERS_MAX) {
            free = i;
        }
    }
    if (add && free < USERS_MAX) {
        s->users[free].valid = true;
        s->users[free].idx = idx;
        memcpy(s->users[free].mac.a, d + 2, 6);
    }
}

/*
 * Registers by message: struct reg_info {addr, width, val}, or for a
 * field reg_bit_info {offset, bits, irq_lock, resv, addr, val}
 */
static uint32_t ws_cfg_reg(Ws73State *s, uint16_t msg_id, const uint8_t *d,
                           size_t n)
{
    uint32_t addr, mask;

    if (n < 12) {
        return 0;
    }
    switch (msg_id) {
    case MSG_REG_WRITE:
        ws_reg_set(s, ldl_le_p(d), ldl_le_p(d + 8));
        break;
    case MSG_REG_WRITE_BITS:
        addr = ldl_le_p(d + 4);
        mask = (d[1] >= 32 ? ~0u : (1u << d[1]) - 1) << (d[0] & 31);
        ws_reg_set(s, addr, (ws_reg_get(s, addr) & ~mask) |
                   ((ldl_le_p(d + 8) << (d[0] & 31)) & mask));
        break;
    case MSG_REG_READ:
        return ws_reg_get(s, ldl_le_p(d));
    }
    return 0;
}

/*
 * The beacon hmac has dmac send for its access point (struct
 * tx_beacon_data: the frame, then its length after the TX parameters)
 */
static void ws_beacon(Ws73State *s, const uint8_t *f, size_t n)
{
    const uint8_t *rsn = NULL;
    /* the SSID leads the elements, after timestamp, interval, capability */
    const size_t ie = WIFI_HDR_LEN + 12;

    n = MIN(n, BEACON_LEN);
    if (!s->client_nic || n < ie + 2 ||
        lduw_le_p(f) != WIFI_FC_MGMT(WIFI_STYPE_BEACON) ||
        f[ie] != WIFI_EID_SSID || ie + 2 + f[ie + 1] > n) {
        return;
    }
    if (s->bcn && !memcmp(s->bcn_bssid.a, f + 16, 6) &&
        s->bcn_ssid_len == MIN(f[ie + 1], sizeof(s->bcn_ssid)) &&
        !memcmp(s->bcn_ssid, f + ie + 2, s->bcn_ssid_len)) {
        return;
    }
    s->bcn = true;
    s->bcn_vap = s->vap;
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
    trace_ws73_bss(s->bcn_ssid_len);
    wifi_client_bss(&s->client, s->bcn_bssid.a, s->bcn_ssid,
                    s->bcn_ssid_len, rsn);
}

/* The access point stopped: no more beacons, the phone loses it */
static void ws_bss_down(Ws73State *s)
{
    if (s->bcn) {
        s->bcn = false;
        trace_ws73_bss(0);
        wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
    }
}

/* A configuration message for dmac; one waiting on it gets an answer */
static void ws_cfg(Ws73State *s, const uint8_t *p, size_t len)
{
    const uint8_t *ad = p + HCC_HDR_LEN + EXT_LEN, *d = ad + AD_LEN;
    uint16_t msg_id;
    size_t n;
    uint8_t rsp[4];

    if (len < MSG_HDR_LEN) {
        return;
    }
    msg_id = lduw_le_p(ad + AD_MSG_ID);
    n = MIN(lduw_le_p(ad + AD_DATA_LEN), len - MSG_HDR_LEN);
    trace_ws73_cfg(msg_id, ad[AD_VAP], ad[AD_FLAGS] & AD_SYNC,
                   ldl_le_p(ad + AD_SEQ), lduw_le_p(ad + AD_DATA_LEN),
                   lduw_le_p(ad + AD_RSP_BUF_LEN),
                   len >= MSG_HDR_LEN + 4 ? ldl_le_p(ad + AD_LEN) : 0,
                   len >= MSG_HDR_LEN + 8 ? ldl_le_p(ad + AD_LEN + 4) : 0,
                   len >= MSG_HDR_LEN + 12 ? ldl_le_p(ad + AD_LEN + 8) : 0);
    switch (msg_id) {
    case MSG_ADD_USER:
    case MSG_DEL_USER:
        ws_user(s, d, n, msg_id == MSG_ADD_USER);
        break;
    case MSG_SWITCH_CHANNEL:
        if (n >= 1) {
            s->chan = d[0];
        }
        break;
    case MSG_SEND_READY:
        /* calibrated: every open waits on this second ready */
        s->notify |= BIT(D2H_WLAN_READY);
        break;
    case MSG_ADD_VAP:
        /*
         * mac_cfg_add_vap_param: the MAC's VAP is fixed by role (STA 0,
         * P2P 1, AP 2), and hmac drops a frame for none of them
         */
        if (n > ADD_VAP_HAL_ID) {
            s->hal_vap[d[ADD_VAP_ID] & 0xf] = d[ADD_VAP_HAL_ID];
        }
        break;
    case MSG_DEL_VAP:
    case MSG_SYNC_VAP:
        /* the beaconing VAP deleted or back to INIT (mac_update_dmac_vap) */
        if (ad[AD_VAP] == s->bcn_vap &&
            (msg_id == MSG_DEL_VAP || (n >= 1 && d[0] == VAP_STATE_INIT))) {
            ws_bss_down(s);
        }
        break;
    case MSG_SEND_BEACON:
        /* the access point's VAP: the phone's frames are for it */
        s->vap = ad[AD_VAP];
        ws_beacon(s, d, n);
        break;
    case MSG_REG_READ:
        stl_le_p(rsp, ws_cfg_reg(s, msg_id, d, n));
        if (ad[AD_FLAGS] & AD_SYNC) {
            ws_cfg_answer(s, p, rsp, sizeof(rsp));
        }
        return;
    case MSG_REG_WRITE:
    case MSG_REG_WRITE_BITS:
        ws_cfg_reg(s, msg_id, d, n);
        break;
    }
    if (ad[AD_FLAGS] & AD_SYNC) {
        ws_cfg_answer(s, p, NULL, 0);
    }
}

/*
 * A frame off the air for hmac; one that came encrypted has been
 * decrypted
 */
static void ws_rx_frame(Ws73State *s, const uint8_t *f, size_t len,
                        bool protected)
{
    uint16_t fc;
    size_t hl = WIFI_HDR_LEN, body;
    bool mgmt;
    GByteArray *u;
    uint8_t *ext, *cb;

    if (len < WIFI_HDR_LEN || g_queue_get_length(&s->d2h) >= D2H_QUEUE_MAX) {
        return;
    }
    fc = lduw_le_p(f) & ~WIFI_FC_PROTECTED;
    mgmt = WIFI_FC_TYPE(fc) == WIFI_TYPE_MGMT;
    if (!mgmt && (WIFI_FC_STYPE(fc) & WIFI_STYPE_QOS)) {
        hl += 2;
    }
    if (len < hl) {
        return;
    }
    body = len - hl;
    u = ws_packet(SRV_WIFI, 0, mgmt ? DATA_HI_QUEUE : DATA_LO_QUEUE,
                  FRM_LEN + body);
    ext = u->data + HCC_HDR_LEN;
    cb = ext + EXT_LEN;
    stw_le_p(ext + EXT_MSG_ID, mgmt ? NETBUF_MGMT : NETBUF_DATA);
    ext[EXT_VAP_TYPE] = s->vap | (mgmt ? MSG_MGMT : MSG_DATA) << 4;
    stw_le_p(ext + EXT_DATA_LEN, body);
    cb[RXC_VAP] = s->hal_vap[s->vap & 0xf];
    cb[RXC_MSDUS] = 1;
    cb[RXC_USER] = RXC_USER_NONE;
    for (unsigned i = 0; i < USERS_MAX; i++) {
        if (s->users[i].valid && !memcmp(s->users[i].mac.a, f + 10, 6)) {
            cb[RXC_USER] = s->users[i].idx & RXC_USER_MASK;
        }
    }
    cb[RXC_HDR_LEN] = hl | RXC_LAST |
        (fc == WIFI_FC_MGMT(WIFI_STYPE_BEACON) ? RXC_BEACON : 0);
    stw_le_p(cb + RXC_FRAME_LEN, len);
    cb[RXC_MAC_VAP] = s->vap | 1 << 4;
    cb[RXC_CHANNEL] = s->chan;
    cb[RXC_STATUS] = protected ? RX_OK_CCMP : RX_OK_PLAIN;
    cb[RXC_RSSI] = (uint8_t)RX_RSSI;
    memcpy(cb + FRM_CB_LEN, f, hl);
    /*
     * Decrypted, a frame keeps its Protected bit: hmac drops unicast
     * data without it once the user has a key
     */
    stw_le_p(cb + FRM_CB_LEN, fc | (protected ? WIFI_FC_PROTECTED : 0));
    memcpy(cb + FRM_CB_LEN + FRM_HDR_SLOT, f + hl, body);
    trace_ws73_rx(fc, len, s->vap, cb[RXC_VAP], cb[RXC_USER]);
    ws_send(s, u);
}

/* A frame hmac sends: the client's if from the driver's own BSS */
static void ws_tx_frame(Ws73State *s, const uint8_t *p, size_t len)
{
    const uint8_t *ext = p + HCC_HDR_LEN, *cb = ext + EXT_LEN;
    g_autofree uint8_t *f = NULL;
    size_t hl, body;
    uint16_t fc;

    if (len < FRM_LEN) {
        return;
    }
    hl = cb[TXC_HDR_LEN] & 0x3f;
    body = MIN(lduw_le_p(ext + EXT_DATA_LEN), len - FRM_LEN);
    if (hl < 16 || hl > FRM_HDR_SLOT) {
        return;
    }
    f = g_malloc(hl + body);
    memcpy(f, cb + FRM_CB_LEN, hl);
    memcpy(f + hl, cb + FRM_CB_LEN + FRM_HDR_SLOT, body);
    fc = lduw_le_p(f) & ~WIFI_FC_PROTECTED;
    stw_le_p(f, fc);
    s->vap = ext[EXT_VAP_TYPE] & 0xf;
    trace_ws73_tx(fc, hl + body, s->vap);
    if (s->bcn && !memcmp(f + 10, s->bcn_bssid.a, 6)) {
        if (s->client_nic) {
            wifi_client_receive(&s->client, f, hl + body);
        }
    } else {
        wifi_ap_receive(&s->ap, f, hl + body);
    }
}

/*
 * A platform message: of these only the TRNG's request gets an answer,
 * random bytes the driver feeds to the kernel at each authentication
 */
static void ws_plat(Ws73State *s, const uint8_t *p, size_t len)
{
    GByteArray *u;
    uint32_t n;

    if (len < PLAT_DATA + 4 || lduw_le_p(p + PLAT_MSG_ID) != PLAT_H2D_TRNG) {
        return;
    }
    /* the driver asks in words and loops on the count until done */
    n = MIN(ldl_le_p(p + PLAT_DATA), TRNG_MAX) & ~3u;
    if (!n) {
        return;
    }
    u = ws_packet(SRV_OAM, OAM_PLAT_MSG, INTERNAL_QUEUE,
                  PLAT_DATA + TRNG_HEAD + n);
    stw_le_p(u->data + PLAT_MSG_ID, PLAT_D2H_TRNG);
    stw_le_p(u->data + PLAT_LEN, TRNG_HEAD + n);
    stl_le_p(u->data + PLAT_DATA + 4, n);
    qemu_guest_getrandom_nofail(u->data + PLAT_DATA + TRNG_HEAD, n);
    ws_send(s, u);
}

static void ws_hcc_packet(Ws73State *s, const uint8_t *p, size_t len)
{
    uint8_t service = p[HCC_TYPE] >> 4, sub = p[HCC_TYPE] & 0xf;

    trace_ws73_h2d(service, sub, p[HCC_QUEUE], len);
    if (service == SRV_CUSTOMIZE && sub == CUSTOM_CFGID_MSG) {
        ws_cfg(s, p, len);
    } else if (service == SRV_OAM && sub == OAM_PLAT_MSG) {
        ws_plat(s, p, len);
    } else if (service == SRV_WIFI) {
        ws_tx_frame(s, p, len);
    }
}

/* A transfer from the host: packets after the package and descriptor */
static void ws_h2d(Ws73State *s, const uint8_t *buf, size_t len)
{
    uint32_t type;
    size_t off, n;

    if (len < H2D_HDR_LEN) {
        return;
    }
    type = ldl_le_p(buf + PKG_TYPE);
    if (type != PKG_SINGLE && type != PKG_AGGR) {
        return;
    }
    len = MIN(len, ldl_le_p(buf + PKG_LEN));
    for (off = H2D_HDR_LEN; off + HCC_HDR_LEN <= len;
         off += MAX(n, H2D_MIN_PKT)) {
        n = lduw_le_p(buf + off + HCC_LEN);
        if (n <= HCC_HDR_LEN || off + n > len) {
            break;
        }
        ws_hcc_packet(s, buf + off, n);
    }
}

/* What the host sends: a FILES piece by its length, else to a short packet */
static void ws_take(Ws73State *s, USBPacket *p)
{
    size_t len = s->out->len;

    if (len + p->iov.size > OUT_MAX) {
        g_byte_array_set_size(s->out, 0);
        return;
    }
    g_byte_array_set_size(s->out, len + p->iov.size);
    usb_packet_copy(p, s->out->data + len, p->iov.size);
    if (!s->running && s->files_left) {
        if (s->out->len < s->files_left) {
            return;
        }
        trace_ws73_fw(s->out->len);
        s->files_left = 0;
        ws_boot_answer(s, "FILES OK");
    } else if (p->iov.size % p->ep->max_packet_size == 0 && p->iov.size) {
        return;
    } else if (s->running) {
        ws_h2d(s, s->out->data, s->out->len);
    } else {
        ws_boot(s, s->out->data, s->out->len);
    }
    g_byte_array_set_size(s->out, 0);
}

/* A boot ROM answer, or the packets of one queue with their lengths */
static void ws_give(Ws73State *s, USBPacket *p)
{
    GByteArray *u, *x;
    size_t n = 0, total = D2H_HDR_LEN;
    uint8_t queue;
    bool full;

    if (!s->running) {
        u = g_queue_pop_head(&s->boot_in);
        if (!u) {
            p->status = USB_RET_NAK;
            return;
        }
        usb_packet_copy(p, u->data, MIN(u->len, p->iov.size));
        g_byte_array_unref(u);
        return;
    }
    u = g_queue_peek_head(&s->d2h);
    if (!u) {
        p->status = USB_RET_NAK;
        return;
    }
    full = g_queue_get_length(&s->d2h) >= D2H_QUEUE_MAX;
    x = g_byte_array_new();
    g_byte_array_set_size(x, D2H_HDR_LEN);
    memset(x->data, 0, D2H_HDR_LEN);
    queue = u->data[HCC_QUEUE];
    while ((u = g_queue_peek_head(&s->d2h)) && n < D2H_AGGR_MAX &&
           u->data[HCC_QUEUE] == queue &&
           total + u->len <= MIN(p->iov.size, D2H_XFER_MAX)) {
        g_queue_pop_head(&s->d2h);
        stw_le_p(x->data + D2H_AGGR + 2 * n++, u->len);
        g_byte_array_append(x, u->data, u->len);
        total += u->len;
        g_byte_array_unref(u);
    }
    stl_le_p(x->data + PKG_TYPE, PKG_D2H);
    stl_le_p(x->data + PKG_LEN, x->len);
    stl_le_p(x->data + PKG_RESERVE, queue);
    usb_packet_copy(p, x->data, MIN(x->len, p->iov.size));
    g_byte_array_unref(x);
    /* room again: what the netdevs held back may come */
    if (full) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        if (s->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->client_nic));
        }
    }
}

static void ws_give_notify(Ws73State *s, USBPacket *p)
{
    uint8_t n[NOTIFY_LEN] = { 0 };

    if (!s->notify) {
        p->status = USB_RET_NAK;
        return;
    }
    stl_le_p(n, s->notify);
    trace_ws73_notify(s->notify);
    s->notify = 0;
    usb_packet_copy(p, n, MIN(sizeof(n), p->iov.size));
}

static void ws_reg_out(Ws73State *s, USBPacket *p)
{
    uint8_t b[REG_SETUP_LEN] = { 0 };
    size_t n = MIN(p->iov.size, sizeof(b));

    usb_packet_copy(p, b, n);
    if (s->reg_data && n == 4) {
        s->reg_data = false;
        ws_reg_set(s, REG_PIPE_BASE + s->reg_addr, ldl_le_p(b));
        trace_ws73_reg(true, s->reg_addr, ldl_le_p(b));
        return;
    }
    if (n < REG_SETUP_LEN) {
        return;
    }
    s->reg_addr = lduw_le_p(b + REG_SETUP_OFF);
    s->reg_data = b[REG_SETUP_CTRL] & REG_DIR_WRITE;
    s->reg_read = !s->reg_data;
}

static void ws_reg_in(Ws73State *s, USBPacket *p)
{
    uint8_t v[4];

    if (!s->reg_read) {
        p->status = USB_RET_NAK;
        return;
    }
    s->reg_read = false;
    stl_le_p(v, ws_reg_get(s, REG_PIPE_BASE + s->reg_addr));
    trace_ws73_reg(false, s->reg_addr, ldl_le_p(v));
    usb_packet_copy(p, v, MIN(sizeof(v), p->iov.size));
}

static void ws_handle_data(USBDevice *dev, USBPacket *p)
{
    Ws73State *s = WS73(dev);

    switch (p->pid == USB_TOKEN_IN ? p->ep->nr | USB_DIR_IN : p->ep->nr) {
    case USB_DIR_IN | EP_BULK:
        ws_give(s, p);
        return;
    case EP_BULK_OUT:
        ws_take(s, p);
        return;
    case USB_DIR_IN | EP_INT:
        if (s->running) {
            ws_give_notify(s, p);
            return;
        }
        break;
    case EP_REG_OUT:
        if (s->running) {
            ws_reg_out(s, p);
            return;
        }
        break;
    case USB_DIR_IN | EP_REG_IN:
        if (s->running) {
            ws_reg_in(s, p);
            return;
        }
        break;
    }
    p->status = USB_RET_STALL;
}

/* The firmware's messages: an open is answered with its ready */
static void ws_h2d_msg(Ws73State *s, uint32_t msg)
{
    trace_ws73_msg(msg);
    if (msg == H2D_WLAN_OPEN) {
        s->notify |= BIT(D2H_WLAN_READY);
    }
}

static void ws_handle_control(USBDevice *dev, USBPacket *p, int request,
                              int value, int index, int length,
                              uint8_t *data)
{
    Ws73State *s = WS73(dev);

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        /*
         * The firmware says it booted once configured: the bus resets
         * of the enumeration before would lose it
         */
        if (request == (DeviceOutRequest | USB_REQ_SET_CONFIGURATION) &&
            value && s->running) {
            s->notify |= BIT(D2H_BSP_READY);
        }
        return;
    }
    if (request == (ClassInterfaceOutRequest | REQ_SEND_MSG)) {
        if (length >= 4) {
            ws_h2d_msg(s, ldl_le_p(data));
        }
    } else if (request == (ClassInterfaceRequest | REQ_GET_CREDIT)) {
        if (length >= 4) {
            stl_le_p(data, CREDIT);
            p->actual_length = 4;
        }
    } else if ((request & ~0xff) == ClassInterfaceRequest) {
        memset(data, 0, length);
        p->actual_length = length;
    } else if ((request & ~0xff) != ClassInterfaceOutRequest) {
        p->status = USB_RET_STALL;
    }
}

/* A bus reset leaves the CPU, and whatever it runs, alone */
static void ws_handle_reset(USBDevice *dev)
{
    ws_flush(WS73(dev));
}

/* Power on: the CPU waiting in its ROM, the chip on the bus */
static void ws_power_on(DeviceState *d)
{
    Ws73State *s = WS73(d);
    USBDevice *dev = USB_DEVICE(d);

    ws_flush(s);
    s->running = false;
    g_hash_table_remove_all(s->regs);
    memset(s->users, 0, sizeof(s->users));
    memset(s->hal_vap, 0, sizeof(s->hal_vap));
    s->chan = 0;
    timer_del(s->replug);
    dev->usb_desc = NULL;
    if (dev->attached) {
        usb_desc_attach(dev);
    } else {
        timer_mod(s->replug, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
    }
}

/* From the access point */
static void ws_deliver(void *opaque, const uint8_t *frame, size_t len,
                       bool protected)
{
    Ws73State *s = opaque;

    if (s->running && s->chan == s->ap.channel) {
        ws_rx_frame(s, frame, len, protected);
    }
}

static void ws_beacon_tick(void *opaque)
{
    Ws73State *s = opaque;
    uint8_t b[WIFI_AP_BEACON_MAX];

    if (USB_DEVICE(s)->attached) {
        ws_deliver(s, b, wifi_ap_beacon(&s->ap, b), false);
    }
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

static bool ws_can_receive(NetClientState *nc)
{
    Ws73State *s = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&s->d2h) < D2H_QUEUE_MAX;
}

static ssize_t ws_receive(NetClientState *nc, const uint8_t *buf,
                          size_t size)
{
    Ws73State *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
    return size;
}

static NetClientInfo ws_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ws_can_receive,
    .receive = ws_receive,
};

static void ws_client_deliver(void *opaque, const uint8_t *frame,
                              size_t len, bool protected)
{
    Ws73State *s = opaque;

    if (s->bcn) {
        ws_rx_frame(s, frame, len, protected);
    }
}

static bool ws_client_can_receive(NetClientState *nc)
{
    Ws73State *s = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&s->client) &&
           g_queue_get_length(&s->d2h) < D2H_QUEUE_MAX;
}

static ssize_t ws_client_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size)
{
    Ws73State *s = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&s->client, buf, size);
    return size;
}

static NetClientInfo ws_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ws_client_can_receive,
    .receive = ws_client_receive,
};

static void ws_realize(USBDevice *dev, Error **errp)
{
    Ws73State *s = WS73(dev);
    DeviceState *d = DEVICE(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ws_net_info, &s->conf,
                          object_get_typename(OBJECT(d)), d->id,
                          &d->mem_reentrancy_guard, s);
    if (!wifi_ap_realize(&s->ap, s->nic, ws_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
        return;
    }
    if (s->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&s->client_conf.macaddr);
        s->client_nic = qemu_new_nic(&ws_client_net_info, &s->client_conf,
                                     object_get_typename(OBJECT(d)), NULL,
                                     &d->mem_reentrancy_guard, s);
        wifi_client_init(&s->client, s->client_nic,
                         &s->client_conf.macaddr, ws_client_deliver, s);
    }
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

static void ws_unrealize(USBDevice *dev)
{
    Ws73State *s = WS73(dev);

    timer_del(s->replug);
    timer_del(s->beacon);
    if (s->client_nic) {
        wifi_client_cleanup(&s->client);
        qemu_del_nic(s->client_nic);
        s->client_nic = NULL;
    }
    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void ws_init(Object *obj)
{
    Ws73State *s = WS73(obj);

    s->out = g_byte_array_new();
    g_queue_init(&s->boot_in);
    g_queue_init(&s->d2h);
    s->regs = g_hash_table_new(NULL, NULL);
    s->replug = timer_new_ms(QEMU_CLOCK_VIRTUAL, ws_replug, s);
    s->beacon = timer_new_ms(QEMU_CLOCK_VIRTUAL, ws_beacon_tick, s);
}

static void ws_finalize(Object *obj)
{
    Ws73State *s = WS73(obj);

    ws_flush(s);
    g_byte_array_unref(s->out);
    g_hash_table_destroy(s->regs);
    timer_free(s->replug);
    timer_free(s->beacon);
}

static const Property ws_props[] = {
    DEFINE_NIC_PROPERTIES(Ws73State, conf),
    DEFINE_PROP_STRING("ssid", Ws73State, ap.ssid),
    DEFINE_PROP_STRING("psk", Ws73State, ap.psk),
    DEFINE_PROP_UINT8("channel", Ws73State, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", Ws73State, ap.bssid),
    DEFINE_PROP_NETDEV("client-netdev", Ws73State, client_conf.peers),
    DEFINE_PROP_MACADDR("client-mac", Ws73State, client_conf.macaddr),
};

/* the firmware's state and the queued packets are not migrated */
static const VMStateDescription ws_vmstate = {
    .name = TYPE_WS73,
    .unmigratable = 1,
};

static void ws_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->product_desc = "HiSilicon WS73V100 USB Wi-Fi";
    uc->usb_desc = &ws_desc_boot;
    uc->realize = ws_realize;
    uc->unrealize = ws_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = ws_handle_reset;
    uc->handle_control = ws_handle_control;
    uc->handle_data = ws_handle_data;
    dc->desc = "HiSilicon WS73V100 USB Wi-Fi";
    dc->vmsd = &ws_vmstate;
    device_class_set_legacy_reset(dc, ws_power_on);
    device_class_set_props(dc, ws_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo ws_info = {
    .name          = TYPE_WS73,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(Ws73State),
    .instance_init = ws_init,
    .instance_finalize = ws_finalize,
    .class_init    = ws_class_init,
};

static void ws_register_types(void)
{
    type_register_static(&ws_info);
}

type_init(ws_register_types)
