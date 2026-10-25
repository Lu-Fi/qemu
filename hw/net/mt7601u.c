/*
 * MediaTek MT7601U Wi-Fi on USB
 *
 * The chip as MediaTek's vendor STA driver (mt7601sta) sees it: vendor
 * requests on endpoint 0 read and write the MAC registers, the first
 * bulk OUT endpoint carries the firmware and the commands for the Andes
 * MCU, the other five the WMM queues' frames, and bulk IN 4 and 5 bring
 * received frames and the MCU's answers. BBP registers sit behind a port
 * at 0x101c, RF banks behind one at 0x500; the MCU reaches both, and the
 * MAC registers at 0x410000 up, for the commands that batch them. The
 * firmware the driver downloads is taken and dropped; this model answers
 * in its place, and puts an access point (wifi-ap.c) on the air.
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

#define TYPE_MT7601U "mt7601u"
OBJECT_DECLARE_SIMPLE_TYPE(Mt7601uState, MT7601U)

/* vendor requests */
#define VEND_DEV_MODE           0x01
#define VEND_WRITE              0x02
#define VEND_MULTI_WRITE        0x06
#define VEND_MULTI_READ         0x07
#define VEND_READ_EEPROM        0x09
#define VEND_WRITE_HIGH         0x10
#define VEND_WRITE_FCE          0x42

/* the device mode that takes the 64-byte interrupt vector the MCU starts */
#define DEV_MODE_LOAD_IVB       0x12

/* endpoints, in the order the descriptor lists them and the driver maps */
#define EP_IN_DATA              4
#define EP_IN_CMD               5
#define EP_OUT_CMD              8
#define EP_OUTS                 6
static const uint8_t mt_ep_out[EP_OUTS] = { EP_OUT_CMD, 4, 5, 6, 7, 9 };

/* a firmware chunk is 14336 bytes behind its TXINFO */
#define OUT_MAX                 (16 * 1024)

#define MT_REG_SIZE             0x10000

#define MT_ASIC_VERSION         0x0000
#define MT_CMB_CTRL             0x0020
#define CMB_CTRL_XTAL_RDY       BIT(22)
#define CMB_CTRL_PLL_LD         BIT(23)
#define MT_EFUSE_CTRL           0x0024
#define EFUSE_CTRL_AOUT         0x3f
#define EFUSE_CTRL_AIN(v)       extract32(v, 16, 10)
#define EFUSE_CTRL_KICK         BIT(30)
#define EFUSE_CTRL_SEL          BIT(31)
#define MT_EFUSE_DATA0          0x0028
#define MT_RF_CSR_CFG           0x0500
#define RF_CSR_WR               BIT(30)
#define RF_CSR_KICK             BIT(31)
#define MT_COM_REG0             0x0730
#define MT_COM_REG1             0x0734
#define COM_REG1_DDONE          BIT(31)
#define MT_MAC_CSR0             0x1000
#define MT_MAC_ADDR_DW0         0x1008
#define MT_MAC_BSSID_DW0        0x1010
#define MT_BBP_CSR_CFG          0x101c
#define BBP_CSR_READ            BIT(16)
#define BBP_CSR_BUSY            BIT(17)
#define MT_MAC_STATUS_CFG       0x1200
#define MT_BCN_TIME_CFG         0x1114
#define BCN_TIME_CFG_BEACON_GEN BIT(20)
/* BSS0's beacon: its TXWI, then the frame */
#define MT_BEACON0              0xc000
#define BEACON_BUF              0x200

/* ASIC and MAC versions as MT7601U silicon reads them */
#define MT7601_ASIC_VERSION     0x76010001
#define MT7601_MAC_VERSION      0x76010500

/*
 * The eFuse's logical map; its usage map at 0x1e0-0x1fc (a byte a block,
 * 0 free) reads every block used, so the driver keeps to the eFuse
 */
#define MT_EFUSE_SIZE           0x200
#define EFUSE_USAGE_MAP         0x1e0

/* TXINFO, ahead of a frame or a command; RXFCE_INFO, after one */
#define INFO_LEN(v)             extract32(v, 0, 16)
#define INFO_SEQ(v)             extract32(v, 16, 4)
#define INFO_CMD(v)             extract32(v, 20, 7)
#define INFO_TYPE(v)            extract32(v, 30, 2)
#define INFO_TYPE_CMD           1
#define PORT_CPU_RX             1

/* Andes MCU commands, the addresses they take, and the events back */
#define CMD_BURST_WRITE         8
#define CMD_READ_MODIFY_WRITE   9
#define CMD_RANDOM_READ         10
#define CMD_BURST_READ          11
#define CMD_RANDOM_WRITE        12
#define MCU_MAC_BASE            0x410000
#define MCU_ADDR_RF             BIT(31)
#define MCU_ADDR_BBP            BIT(30)
#define EVT_CMD_DONE            0
#define CMD_ACK_LEN             8

/* a received frame: its DMA length, RXINFO, RXWI, the MPDU, RXFCE_INFO */
#define RXINFO_DATA             BIT(1)
#define RXINFO_U2ME             BIT(4)
#define RXINFO_MCAST            BIT(5)
#define RXINFO_BCAST            BIT(6)
#define RXINFO_MYBSS            BIT(7)
#define RXINFO_DECRYPTED        BIT(16)
#define RXINFO_PN_LEN(dwords)   ((dwords) << 19)
#define RXWI_LEN                24
#define RXFCE_80211             BIT(19)
#define RXWI_WCID_NONE          0xff
#define RXWI_KEY_IDX(i)         ((i) << 8)

/* the transmitters the hardware knows, by WCID; BSS0's group key modes */
#define MT_WCID_TABLE           0x1800
#define WCID_ENTRY              8
#define WCIDS                   256
#define MT_SHARED_KEY_MODE      0xb000

/* a frame to send: TXINFO, TXWI (20 bytes), the MPDU */
#define TXWI_LEN                20
#define TXWI_MPDU_LEN(w1)       extract32(w1, 16, 12)

/* RF registers, 64 to a bank; bank 0 R17-R20 tune the synthesizer */
#define RF_BANKS                16
#define RF_REGS                 64

#define BEACON_MS               102
#define RX_QUEUE_MAX            64

struct Mt7601uState {
    USBDevice parent_obj;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;
    QEMUTimer *beacon;

    uint8_t regs[MT_REG_SIZE];
    uint8_t bbp[256];
    uint8_t rf[RF_BANKS][RF_REGS];
    uint8_t efuse[MT_EFUSE_SIZE];
    uint8_t channel;
    uint64_t ptk_pn, gtk_pn;    /* the AP's packet numbers, as sent */

    /* the chip's own BSS, as its beacon names it */
    bool bcn;
    MACAddr bcn_bssid;
    uint8_t bcn_ssid[32];
    uint8_t bcn_ssid_len;

    GByteArray *out[EP_OUTS];
    GQueue rx;                  /* frames for bulk IN 4 */
    GQueue rsp;                 /* MCU answers for bulk IN 5 */
    GByteArray *in[2];          /* the transfer each IN endpoint is giving */
    size_t in_off[2];
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings mt_strings = {
    [STR_MANUFACTURER]  = "MediaTek",
    [STR_PRODUCT]       = "802.11 n WLAN",
    [STR_SERIALNUMBER]  = "1.0",
};

#define MT_EP(_addr, _max)                                                  \
    {                                                                       \
        .bEndpointAddress      = (_addr),                                   \
        .bmAttributes          = USB_ENDPOINT_XFER_BULK,                    \
        .wMaxPacketSize        = (_max),                                    \
    }

#define MT_EPS(_max)                                                        \
    (USBDescEndpoint[]) {                                                   \
        MT_EP(USB_DIR_IN | EP_IN_DATA, _max),                               \
        MT_EP(USB_DIR_IN | EP_IN_CMD, _max),                                \
        MT_EP(USB_DIR_OUT | EP_OUT_CMD, _max),                              \
        MT_EP(USB_DIR_OUT | 4, _max),                                       \
        MT_EP(USB_DIR_OUT | 5, _max),                                       \
        MT_EP(USB_DIR_OUT | 6, _max),                                       \
        MT_EP(USB_DIR_OUT | 7, _max),                                       \
        MT_EP(USB_DIR_OUT | 9, _max),                                       \
    }

static const USBDescIface mt_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2 + EP_OUTS,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = MT_EPS(64),
};

static const USBDescIface mt_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2 + EP_OUTS,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = MT_EPS(512),
};

static const USBDescDevice mt_device_full = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 160,
            .nif = 1,
            .ifs = &mt_iface_full,
        },
    },
};

static const USBDescDevice mt_device_high = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 160,
            .nif = 1,
            .ifs = &mt_iface_high,
        },
    },
};

static const USBDesc mt_desc = {
    .id = {
        .idVendor          = 0x148f,
        .idProduct         = 0x7601,
        .bcdDevice         = 0x0000,
        .iManufacturer     = STR_MANUFACTURER,
        .iProduct          = STR_PRODUCT,
        .iSerialNumber     = STR_SERIALNUMBER,
    },
    .full = &mt_device_full,
    .high = &mt_device_high,
    .str  = mt_strings,
};

static uint32_t mt_reg(Mt7601uState *s, uint32_t off)
{
    return ldl_le_p(s->regs + off);
}

static void mt_set_reg(Mt7601uState *s, uint32_t off, uint32_t v)
{
    stl_le_p(s->regs + off, v);
}

/*
 * The eFuse's logical blocks the driver reads: id, version 13 (what the
 * driver takes), MAC, one antenna on a USB board, region 5 (channels 1 to
 * 14), no frequency offset, flat TX power. What is not set reads 0xff.
 */
static void mt_efuse_program(Mt7601uState *s)
{
    static const uint8_t power[14] = {
        0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a,
        0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a,
    };
    uint8_t *e = s->efuse;

    memset(e, 0xff, MT_EFUSE_SIZE);
    stw_le_p(e + 0x00, 0x7601);
    stw_le_p(e + 0x02, 0x000d);
    memcpy(e + 0x04, s->conf.macaddr.a, 6);
    stw_le_p(e + 0x34, 0x1011);
    stw_le_p(e + 0x36, 0x0000);
    stw_le_p(e + 0x38, 0xff05);
    stw_le_p(e + 0x3a, 0xff00);
    memcpy(e + 0x52, power, sizeof(power));
    memset(e + EFUSE_USAGE_MAP, 0x01, MT_EFUSE_SIZE - EFUSE_USAGE_MAP);
}

/*
 * An eFuse read the driver kicks: a 16-byte block into EFUSE_DATA0-3, and
 * in AOUT where it sits, or 0x3f for one past the fuses, never burned
 */
static void mt_efuse_kick(Mt7601uState *s, uint32_t ctrl)
{
    uint32_t off = EFUSE_CTRL_AIN(ctrl) & ~0xf;
    uint32_t aout = EFUSE_CTRL_AOUT;

    if (off < MT_EFUSE_SIZE) {
        memcpy(s->regs + MT_EFUSE_DATA0, s->efuse + off, 16);
        aout = off >> 4;
    } else {
        memset(s->regs + MT_EFUSE_DATA0, 0xff, 16);
    }
    ctrl &= ~(EFUSE_CTRL_KICK | EFUSE_CTRL_AOUT);
    mt_set_reg(s, MT_EFUSE_CTRL, ctrl | EFUSE_CTRL_SEL | aout);
}

/* The WCID the hardware gives a transmitter it has in its table */
static unsigned mt_wcid(Mt7601uState *s, const uint8_t *ta)
{
    for (unsigned i = 0; i < WCIDS; i++) {
        if (!memcmp(s->regs + MT_WCID_TABLE + i * WCID_ENTRY, ta, 6)) {
            return i;
        }
    }
    return RXWI_WCID_NONE;
}

/* The group key BSS0 has a cipher for */
static unsigned mt_gtk_id(Mt7601uState *s)
{
    uint32_t modes = mt_reg(s, MT_SHARED_KEY_MODE);

    for (unsigned i = 0; i < 4; i++) {
        if (extract32(modes, i * 4, 4)) {
            return i;
        }
    }
    return 0;
}

/*
 * A frame off the air, as the hardware hands it over. One that came
 * encrypted to the chip's BSS is decrypted: it keeps its CCMP header,
 * which pn_len counts, and loses its MIC. Any other comes as the air
 * carried it, for the driver to decrypt.
 */
static void mt_rx_frame(Mt7601uState *s, const uint8_t *f, size_t len,
                        bool protected)
{
    const uint8_t *me = s->regs + MT_MAC_ADDR_DW0;
    size_t hdr, mpdu, body;
    GByteArray *u;
    uint8_t *d, *m;
    uint32_t rxinfo = 0, key = 0;
    unsigned wcid = RXWI_WCID_NONE;
    bool group = len >= 10 && is_multicast_ether_addr(f + 4);
    uint16_t fc;

    if (len < WIFI_HDR_LEN - 6 ||
        g_queue_get_length(&s->rx) >= RX_QUEUE_MAX) {
        return;
    }
    fc = lduw_le_p(f);
    hdr = MIN(wifi_hdr_len(fc), len);
    if (len < WIFI_HDR_LEN) {
        protected = false;
    }
    if (WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA) {
        rxinfo |= RXINFO_DATA;
    }
    if (!memcmp(f + 4, me, 6)) {
        rxinfo |= RXINFO_U2ME;
    } else if (group) {
        rxinfo |= is_broadcast_ether_addr(f + 4) ? RXINFO_BCAST
                                                 : RXINFO_MCAST;
    }
    if (len >= WIFI_HDR_LEN) {
        /* the BSSID is the address the DS bits leave to it */
        const uint8_t *bssid = fc & WIFI_FC_FROMDS ? f + 10 :
                               fc & WIFI_FC_TODS ? f + 4 : f + 16;

        wcid = mt_wcid(s, f + 10);
        /*
         * My BSS: the BSSID registers' own, or, past them, a frame to the
         * station from a peer it has a WCID for. The driver's P2P side
         * puts the chip's own address there and says as much (sta/
         * rtmp_data.c): broadcasts from the joined AP then come unmarked.
         */
        if (!memcmp(bssid, s->regs + MT_MAC_BSSID_DW0, 6) ||
            (rxinfo & RXINFO_U2ME && wcid != RXWI_WCID_NONE)) {
            rxinfo |= RXINFO_MYBSS;
        }
    }

    mpdu = len + (protected ? WIFI_CCMP_HDR_LEN : 0);
    if (protected && !(rxinfo & RXINFO_MYBSS)) {
        mpdu += WIFI_CCMP_MIC_LEN;
    }
    body = ROUND_UP(RXWI_LEN + mpdu, 4);
    u = g_byte_array_sized_new(8 + 4 + body);
    g_byte_array_set_size(u, 8 + 4 + body);
    d = u->data;
    memset(d, 0, u->len);
    m = d + 8 + RXWI_LEN;
    if (protected && !(rxinfo & RXINFO_MYBSS)) {
        /* the AP's own encryption, with the key the air used */
        const uint8_t *tk = group ? s->ap.gtk : s->ap.ptk + 32;
        uint64_t pn = group ? ++s->gtk_pn : ++s->ptk_pn;

        wifi_ccmp_encrypt(tk, pn, group ? WIFI_AP_GTK_ID : 0, f, len, m);
    } else if (protected) {
        uint64_t pn = group ? ++s->gtk_pn : ++s->ptk_pn;
        uint8_t *ccmp = m + hdr;

        key = group ? mt_gtk_id(s) : 0;
        memcpy(m, f, hdr);
        stw_le_p(m, fc | WIFI_FC_PROTECTED);
        ccmp[0] = pn;
        ccmp[1] = pn >> 8;
        /* ExtIV, then the key id */
        ccmp[3] = 0x20 | key << 6;
        stl_le_p(ccmp + 4, pn >> 16);
        memcpy(ccmp + WIFI_CCMP_HDR_LEN, f + hdr, len - hdr);
        rxinfo |= RXINFO_DECRYPTED | RXINFO_PN_LEN(WIFI_CCMP_HDR_LEN / 4);
    } else {
        memcpy(m, f, len);
    }
    stl_le_p(d, 4 + body);
    stl_le_p(d + 4, rxinfo);
    stl_le_p(d + 8, wcid | RXWI_KEY_IDX(key) | mpdu << 16);
    /* RSSI -40 dBm on the one path, SNR 30 dB */
    d[16] = -40;
    d[20] = 30;
    stl_le_p(d + 8 + body, (4 + body) | RXFCE_80211);
    trace_mt7601u_rx(fc, len, wcid);
    g_queue_push_tail(&s->rx, u);
}

/* From the access point, heard on its channel */
static void mt_deliver(void *opaque, const uint8_t *frame, size_t len,
                       bool protected)
{
    Mt7601uState *s = opaque;

    if (s->channel == s->ap.channel) {
        mt_rx_frame(s, frame, len, protected);
    }
}

static void mt_beacon(Mt7601uState *s)
{
    uint8_t b[WIFI_AP_BEACON_MAX];

    mt_deliver(s, b, wifi_ap_beacon(&s->ap, b), false);
}

/* The synthesizer's frequency, from RF bank 0: its 2.4 GHz channel */
static void mt_retune(Mt7601uState *s)
{
    const uint8_t *r = s->rf[0];
    uint64_t k = r[19] << 16 | r[18] << 8 | r[17];
    int mhz = (30 * (((uint64_t)r[20] << 18) + k) >> 18) - 60;
    uint8_t ch = mhz == 2484 ? 14 : (mhz - 2407 + 2) / 5;

    if (ch < 1 || ch > 14 || ch == s->channel) {
        return;
    }
    trace_mt7601u_channel(s->channel, ch);
    s->channel = ch;
    /* a dwell on the channel catches a beacon */
    mt_beacon(s);
}

static void mt_rf_write(Mt7601uState *s, unsigned bank, unsigned reg,
                        uint8_t v)
{
    if (bank >= RF_BANKS || reg >= RF_REGS) {
        return;
    }
    s->rf[bank][reg] = v;
    if (!bank && reg >= 17 && reg <= 20) {
        mt_retune(s);
    }
}

static uint8_t mt_rf_read(Mt7601uState *s, unsigned bank, unsigned reg)
{
    return bank < RF_BANKS && reg < RF_REGS ? s->rf[bank][reg] : 0;
}

/* The RF port: a kick reads or writes at once */
static void mt_rf_csr(Mt7601uState *s, uint32_t v)
{
    unsigned bank = extract32(v, 14, 4), reg = extract32(v, 8, 6);

    if (!(v & RF_CSR_KICK)) {
        return;
    }
    if (v & RF_CSR_WR) {
        mt_rf_write(s, bank, reg, v);
    } else {
        v = deposit32(v, 0, 8, mt_rf_read(s, bank, reg));
    }
    mt_set_reg(s, MT_RF_CSR_CFG, v & ~RF_CSR_KICK);
}

/* The BBP port: busy for no time */
static void mt_bbp_csr(Mt7601uState *s, uint32_t v)
{
    uint8_t reg = extract32(v, 8, 8);

    if (!(v & BBP_CSR_BUSY)) {
        return;
    }
    if (v & BBP_CSR_READ) {
        v = deposit32(v, 0, 8, s->bbp[reg]);
    } else {
        s->bbp[reg] = v;
    }
    mt_set_reg(s, MT_BBP_CSR_CFG, v & ~BBP_CSR_BUSY);
}

static void mt_ap_update(Mt7601uState *s);

/* A MAC register write and what the hardware does with it */
static void mt_write32(Mt7601uState *s, uint32_t off, uint32_t v)
{
    if (off + 4 > MT_REG_SIZE) {
        return;
    }
    switch (off) {
    case MT_ASIC_VERSION:
    case MT_MAC_CSR0:
        return;
    case MT_EFUSE_CTRL:
        if (v & EFUSE_CTRL_KICK) {
            mt_efuse_kick(s, v);
        } else {
            mt_set_reg(s, off, v | EFUSE_CTRL_SEL);
        }
        return;
    case MT_RF_CSR_CFG:
        mt_set_reg(s, off, v);
        mt_rf_csr(s, v);
        return;
    case MT_BBP_CSR_CFG:
        mt_set_reg(s, off, v);
        mt_bbp_csr(s, v);
        return;
    }
    mt_set_reg(s, off, v);
    if (off == MT_BCN_TIME_CFG) {
        mt_ap_update(s);
    }
}

/* A register as read: what reads differently from what was written */
static uint32_t mt_read32(Mt7601uState *s, uint32_t off)
{
    if (off + 4 > MT_REG_SIZE) {
        return 0;
    }
    switch (off) {
    case MT_CMB_CTRL:
        return mt_reg(s, off) | CMB_CTRL_XTAL_RDY | CMB_CTRL_PLL_LD;
    case MT_MAC_STATUS_CFG:
        return 0;
    }
    return mt_reg(s, off);
}

/* A byte run, as the multi-read and multi-write requests move them */
static void mt_read(Mt7601uState *s, uint32_t off, uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i += 4) {
        uint8_t w[4];

        stl_le_p(w, mt_read32(s, (off + i) & ~3));
        memcpy(buf + i, w, MIN(4, len - i));
    }
}

static void mt_write(Mt7601uState *s, uint32_t off, const uint8_t *buf,
                     size_t len)
{
    for (size_t i = 0; i + 4 <= len; i += 4) {
        mt_write32(s, off + i, ldl_le_p(buf + i));
    }
}

/* The MCU's view: MAC registers at 0x410000, BBP and RF by tag */
static uint32_t mt_mcu_read(Mt7601uState *s, uint32_t addr)
{
    if (addr & MCU_ADDR_RF) {
        return mt_rf_read(s, extract32(addr, 16, 8), extract32(addr, 0, 8));
    }
    if (addr & MCU_ADDR_BBP) {
        return s->bbp[addr & 0xff];
    }
    return mt_read32(s, (addr - MCU_MAC_BASE) & (MT_REG_SIZE - 4));
}

static void mt_mcu_write(Mt7601uState *s, uint32_t addr, uint32_t v)
{
    if (addr & MCU_ADDR_RF) {
        mt_rf_write(s, extract32(addr, 16, 8), extract32(addr, 0, 8), v);
    } else if (addr & MCU_ADDR_BBP) {
        s->bbp[addr & 0xff] = v;
    } else {
        mt_write32(s, (addr - MCU_MAC_BASE) & (MT_REG_SIZE - 4), v);
    }
}

/* An answer on bulk IN 5: RXFCE_INFO with the command's sequence */
static void mt_respond(Mt7601uState *s, unsigned seq, const uint8_t *data,
                       size_t len)
{
    GByteArray *r = g_byte_array_sized_new(4 + len);
    uint8_t info[4];

    stl_le_p(info, len | seq << 16 | EVT_CMD_DONE << 20 |
                   PORT_CPU_RX << 27 | INFO_TYPE_CMD << 30);
    g_byte_array_append(r, info, sizeof(info));
    g_byte_array_append(r, data, len);
    g_queue_push_tail(&s->rsp, r);
}

/* A command for the MCU; a sequence number asks for an answer */
static void mt_mcu_cmd(Mt7601uState *s, unsigned cmd, unsigned seq,
                       uint8_t *p, size_t len)
{
    static const uint8_t ack[CMD_ACK_LEN];

    trace_mt7601u_cmd(cmd, seq, len);
    switch (cmd) {
    case CMD_RANDOM_WRITE:
        for (size_t i = 0; i + 8 <= len; i += 8) {
            mt_mcu_write(s, ldl_le_p(p + i), ldl_le_p(p + i + 4));
        }
        break;
    case CMD_READ_MODIFY_WRITE:
        for (size_t i = 0; i + 12 <= len; i += 12) {
            uint32_t addr = ldl_le_p(p + i);
            uint32_t v = mt_mcu_read(s, addr) & ~ldl_le_p(p + i + 4);

            mt_mcu_write(s, addr, v | ldl_le_p(p + i + 8));
        }
        break;
    case CMD_BURST_WRITE:
        for (size_t i = 4; i + 4 <= len; i += 4) {
            mt_mcu_write(s, ldl_le_p(p) + i - 4, ldl_le_p(p + i));
        }
        break;
    case CMD_RANDOM_READ:
        for (size_t i = 0; i + 8 <= len; i += 8) {
            stl_le_p(p + i + 4, mt_mcu_read(s, ldl_le_p(p + i)));
        }
        if (seq) {
            mt_respond(s, seq, p, len);
        }
        return;
    case CMD_BURST_READ:
        if (len >= 8) {
            uint32_t addr = ldl_le_p(p), n = MIN(ldl_le_p(p + 4), 64);
            g_autofree uint8_t *r = g_malloc(4 + n * 4);

            stl_le_p(r, addr);
            for (uint32_t i = 0; i < n; i++) {
                stl_le_p(r + 4 + i * 4, mt_mcu_read(s, addr + i * 4));
            }
            if (seq) {
                mt_respond(s, seq, r, 4 + n * 4);
            }
            return;
        }
        break;
    }
    if (seq) {
        mt_respond(s, seq, ack, sizeof(ack));
    }
}

/*
 * What comes on the command endpoint: before the MCU runs, firmware for
 * its memory (each chunk done at once); then commands.
 */
static void mt_cmd_out(Mt7601uState *s, uint8_t *buf, size_t len)
{
    uint32_t info;

    if (len < 4) {
        return;
    }
    info = ldl_le_p(buf);
    if (INFO_TYPE(info) != INFO_TYPE_CMD) {
        return;
    }
    if (mt_reg(s, MT_COM_REG0) != 1) {
        mt_set_reg(s, MT_COM_REG1, mt_reg(s, MT_COM_REG1) | COM_REG1_DDONE);
        return;
    }
    mt_mcu_cmd(s, INFO_CMD(info), INFO_SEQ(info), buf + 4,
               MIN(INFO_LEN(info), len - 4));
}

/* Frames to send: each TXINFO, TXWI and MPDU, until a zero TXINFO */
static void mt_data_out(Mt7601uState *s, const uint8_t *buf, size_t len)
{
    size_t p = 0;

    while (p + 4 + TXWI_LEN <= len) {
        uint32_t info = ldl_le_p(buf + p);
        size_t n = INFO_LEN(info);
        size_t mpdu = TXWI_MPDU_LEN(ldl_le_p(buf + p + 8));

        if (!n || p + 4 + n > len) {
            return;
        }
        if (mpdu >= WIFI_HDR_LEN - 6 && TXWI_LEN + mpdu <= n) {
            g_autofree uint8_t *f = g_memdup2(buf + p + 4 + TXWI_LEN, mpdu);
            uint16_t fc = lduw_le_p(f);

            /* the hardware encrypts with the WCID's key: heard in clear */
            stw_le_p(f, fc & ~WIFI_FC_PROTECTED);
            trace_mt7601u_tx(fc, mpdu);
            if (s->bcn && !memcmp(f + 10, s->bcn_bssid.a, 6)) {
                wifi_client_receive(&s->client, f, mpdu);
            } else if (s->channel == s->ap.channel) {
                wifi_ap_receive(&s->ap, f, mpdu);
            }
        }
        p += ROUND_UP(4 + n, 4);
    }
}

/*
 * The access point the chip runs: beacons on, and a beacon in BSS0's
 * buffer naming the chip as its BSS. Without one the client has nothing.
 */
static void mt_ap_update(Mt7601uState *s)
{
    const uint8_t *f = s->regs + MT_BEACON0 + TXWI_LEN;
    size_t len = TXWI_MPDU_LEN(mt_reg(s, MT_BEACON0 + 4));
    /* the SSID leads the elements, after timestamp, interval, capability */
    const size_t ie = WIFI_HDR_LEN + 12;
    bool up = (mt_reg(s, MT_BCN_TIME_CFG) & BCN_TIME_CFG_BEACON_GEN) &&
              len >= ie + 2 && len <= BEACON_BUF - TXWI_LEN &&
              lduw_le_p(f) == WIFI_FC_MGMT(WIFI_STYPE_BEACON) &&
              !memcmp(f + 10, f + 16, 6) && f[ie] == WIFI_EID_SSID &&
              ie + 2 + f[ie + 1] <= len;

    if (!s->client_nic) {
        return;
    }
    if (up && (!s->bcn || memcmp(s->bcn_bssid.a, f + 16, 6) ||
               s->bcn_ssid_len != MIN(f[ie + 1], sizeof(s->bcn_ssid)) ||
               memcmp(s->bcn_ssid, f + ie + 2, s->bcn_ssid_len))) {
        const uint8_t *rsn = NULL;

        memcpy(s->bcn_bssid.a, f + 16, 6);
        s->bcn_ssid_len = MIN(f[ie + 1], sizeof(s->bcn_ssid));
        memcpy(s->bcn_ssid, f + ie + 2, s->bcn_ssid_len);
        for (size_t i = ie; i + 2 <= len && i + 2 + f[i + 1] <= len;
             i += 2 + f[i + 1]) {
            if (f[i] == WIFI_EID_RSN) {
                rsn = f + i;
                break;
            }
        }
        wifi_client_bss(&s->client, s->bcn_bssid.a, s->bcn_ssid,
                        s->bcn_ssid_len, rsn);
    } else if (!up && s->bcn) {
        wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
    }
    s->bcn = up;
}

/* From the client, on the channel of the chip's own BSS */
static void mt_client_deliver(void *opaque, const uint8_t *frame,
                              size_t len, bool protected)
{
    Mt7601uState *s = opaque;

    if (s->bcn) {
        mt_rx_frame(s, frame, len, protected);
    }
}

static void mt_beacon_tick(void *opaque)
{
    Mt7601uState *s = opaque;

    mt_beacon(s);
    mt_ap_update(s);
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

static void mt_handle_control(USBDevice *dev, USBPacket *p, int request,
                              int value, int index, int length,
                              uint8_t *data)
{
    Mt7601uState *s = MT7601U(dev);

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }
    switch (request) {
    case VendorDeviceOutRequest | VEND_DEV_MODE:
        trace_mt7601u_dev_mode(value);
        if (value == DEV_MODE_LOAD_IVB) {
            mt_set_reg(s, MT_COM_REG0, 1);
        }
        return;
    case VendorDeviceOutRequest | VEND_WRITE:
    case VendorDeviceOutRequest | VEND_WRITE_HIGH:
        if (index + 2 <= MT_REG_SIZE) {
            uint32_t w = index & ~3, sh = (index & 2) * 8;

            trace_mt7601u_write(index, value);
            mt_write32(s, w, deposit32(mt_reg(s, w), sh, 16, value));
        }
        return;
    case VendorDeviceOutRequest | VEND_MULTI_WRITE:
        trace_mt7601u_write(index, length >= 4 ? ldl_le_p(data) : 0);
        mt_write(s, index, data, length);
        return;
    case VendorDeviceRequest | VEND_MULTI_READ:
        mt_read(s, index, data, length);
        trace_mt7601u_read(index, length >= 4 ? ldl_le_p(data) : 0);
        p->actual_length = length;
        return;
    case VendorDeviceRequest | VEND_READ_EEPROM:
        for (int i = 0; i < length; i++) {
            data[i] = s->efuse[(index + i) % MT_EFUSE_SIZE];
        }
        p->actual_length = length;
        return;
    case VendorDeviceOutRequest | VEND_WRITE_FCE:
        return;
    }
    p->status = USB_RET_STALL;
}

/* What the host sends, until a short packet ends the transfer */
static void mt_take(Mt7601uState *s, USBPacket *p, unsigned i)
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
    if (i) {
        mt_data_out(s, out->data, out->len);
    } else {
        mt_cmd_out(s, out->data, out->len);
    }
    g_byte_array_set_size(out, 0);
}

/* One queued unit a transfer, the zero-length end left to QEMU */
static void mt_give(Mt7601uState *s, USBPacket *p, unsigned i, GQueue *q)
{
    size_t n;

    if (!s->in[i]) {
        s->in[i] = g_queue_pop_head(q);
        s->in_off[i] = 0;
        if (!s->in[i]) {
            p->status = USB_RET_NAK;
            return;
        }
        /* room again: what the netdevs held back may come */
        if (q == &s->rx && g_queue_get_length(q) == RX_QUEUE_MAX - 1) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
            if (s->client_nic) {
                qemu_flush_queued_packets(qemu_get_queue(s->client_nic));
            }
        }
    }
    n = MIN(p->iov.size, s->in[i]->len - s->in_off[i]);
    usb_packet_copy(p, s->in[i]->data + s->in_off[i], n);
    s->in_off[i] += n;
    if (s->in_off[i] == s->in[i]->len) {
        g_clear_pointer(&s->in[i], g_byte_array_unref);
    }
}

static void mt_handle_data(USBDevice *dev, USBPacket *p)
{
    Mt7601uState *s = MT7601U(dev);

    if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_IN_DATA) {
        mt_give(s, p, 0, &s->rx);
        return;
    }
    if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_IN_CMD) {
        mt_give(s, p, 1, &s->rsp);
        return;
    }
    if (p->pid == USB_TOKEN_OUT) {
        for (unsigned i = 0; i < EP_OUTS; i++) {
            if (p->ep->nr == mt_ep_out[i]) {
                mt_take(s, p, i);
                return;
            }
        }
    }
    p->status = USB_RET_STALL;
}

static void mt_flush(Mt7601uState *s)
{
    GByteArray *u;

    for (unsigned i = 0; i < EP_OUTS; i++) {
        g_byte_array_set_size(s->out[i], 0);
    }
    for (unsigned i = 0; i < 2; i++) {
        g_clear_pointer(&s->in[i], g_byte_array_unref);
    }
    while ((u = g_queue_pop_head(&s->rx))) {
        g_byte_array_unref(u);
    }
    while ((u = g_queue_pop_head(&s->rsp))) {
        g_byte_array_unref(u);
    }
}

/* Power on: registers cleared, the MCU waiting for its firmware */
static void mt_handle_reset(USBDevice *dev)
{
    Mt7601uState *s = MT7601U(dev);

    mt_flush(s);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->bbp, 0, sizeof(s->bbp));
    memset(s->rf, 0, sizeof(s->rf));
    mt_set_reg(s, MT_ASIC_VERSION, MT7601_ASIC_VERSION);
    mt_set_reg(s, MT_MAC_CSR0, MT7601_MAC_VERSION);
    mt_set_reg(s, MT_EFUSE_CTRL, EFUSE_CTRL_SEL);
    s->channel = 1;
    s->ptk_pn = s->gtk_pn = 0;
    if (s->bcn) {
        s->bcn = false;
        wifi_client_bss(&s->client, NULL, NULL, 0, NULL);
    }
}

static bool mt_can_receive(NetClientState *nc)
{
    Mt7601uState *s = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&s->rx) < RX_QUEUE_MAX;
}

static ssize_t mt_receive(NetClientState *nc, const uint8_t *buf,
                          size_t size)
{
    Mt7601uState *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
    return size;
}

static NetClientInfo mt_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = mt_can_receive,
    .receive = mt_receive,
};

static bool mt_client_can_receive(NetClientState *nc)
{
    Mt7601uState *s = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&s->client) &&
           g_queue_get_length(&s->rx) < RX_QUEUE_MAX;
}

static ssize_t mt_client_receive(NetClientState *nc, const uint8_t *buf,
                                 size_t size)
{
    Mt7601uState *s = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&s->client, buf, size);
    return size;
}

static NetClientInfo mt_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = mt_client_can_receive,
    .receive = mt_client_receive,
};

static void mt_realize(USBDevice *dev, Error **errp)
{
    Mt7601uState *s = MT7601U(dev);
    DeviceState *d = DEVICE(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    mt_efuse_program(s);
    s->nic = qemu_new_nic(&mt_net_info, &s->conf,
                          object_get_typename(OBJECT(d)), d->id,
                          &d->mem_reentrancy_guard, s);
    if (!wifi_ap_realize(&s->ap, s->nic, mt_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
        return;
    }
    if (s->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&s->client_conf.macaddr);
        s->client_nic = qemu_new_nic(&mt_client_net_info, &s->client_conf,
                                     object_get_typename(OBJECT(d)), NULL,
                                     &d->mem_reentrancy_guard, s);
        wifi_client_init(&s->client, s->client_nic,
                         &s->client_conf.macaddr, mt_client_deliver, s);
    }
    mt_handle_reset(dev);
    timer_mod(s->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

static void mt_unrealize(USBDevice *dev)
{
    Mt7601uState *s = MT7601U(dev);

    timer_del(s->beacon);
    if (s->client_nic) {
        wifi_client_cleanup(&s->client);
        qemu_del_nic(s->client_nic);
        s->client_nic = NULL;
    }
    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void mt_init(Object *obj)
{
    Mt7601uState *s = MT7601U(obj);

    for (unsigned i = 0; i < EP_OUTS; i++) {
        s->out[i] = g_byte_array_new();
    }
    g_queue_init(&s->rx);
    g_queue_init(&s->rsp);
    s->beacon = timer_new_ms(QEMU_CLOCK_VIRTUAL, mt_beacon_tick, s);
}

static void mt_finalize(Object *obj)
{
    Mt7601uState *s = MT7601U(obj);

    mt_flush(s);
    for (unsigned i = 0; i < EP_OUTS; i++) {
        g_byte_array_unref(s->out[i]);
    }
    timer_free(s->beacon);
}

static const Property mt_props[] = {
    DEFINE_NIC_PROPERTIES(Mt7601uState, conf),
    DEFINE_PROP_STRING("ssid", Mt7601uState, ap.ssid),
    DEFINE_PROP_STRING("psk", Mt7601uState, ap.psk),
    DEFINE_PROP_UINT8("channel", Mt7601uState, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", Mt7601uState, ap.bssid),
    DEFINE_PROP_NETDEV("client-netdev", Mt7601uState, client_conf.peers),
    DEFINE_PROP_MACADDR("client-mac", Mt7601uState, client_conf.macaddr),
    /* its portal takes a passphrase: the driver runs no open AP */
    DEFINE_PROP_STRING("client-psk", Mt7601uState, client.psk),
};

/* the chip's registers and the queued units are not migrated */
static const VMStateDescription mt_vmstate = {
    .name = TYPE_MT7601U,
    .unmigratable = 1,
};

static void mt_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->product_desc = "MediaTek MT7601U USB Wi-Fi";
    uc->usb_desc = &mt_desc;
    uc->realize = mt_realize;
    uc->unrealize = mt_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = mt_handle_reset;
    uc->handle_control = mt_handle_control;
    uc->handle_data = mt_handle_data;
    dc->desc = "MediaTek MT7601U USB Wi-Fi";
    dc->vmsd = &mt_vmstate;
    device_class_set_props(dc, mt_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo mt_info = {
    .name          = TYPE_MT7601U,
    .parent        = TYPE_USB_DEVICE,
    .instance_size = sizeof(Mt7601uState),
    .instance_init = mt_init,
    .instance_finalize = mt_finalize,
    .class_init    = mt_class_init,
};

static void mt_register_types(void)
{
    type_register_static(&mt_info);
}

type_init(mt_register_types)
