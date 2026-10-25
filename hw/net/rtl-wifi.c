/*
 * Realtek RTL8188E, RTL8188F, RTL8192F and RTL8733B Wi-Fi: the chip model
 * the bus front ends share
 *
 * The MAC as its Linux driver (Realtek's rtl8188e/rtl8188f/rtl8192f HAL,
 * or halmac for the 8733B) sees it past the bus: the WLAN registers with
 * the handshakes the driver waits on, the security CAM, and the RX and TX
 * descriptors. The firmware the driver downloads is taken and dropped;
 * this model answers what the driver asks of it, and puts an access point
 * (wifi-ap.c) on the air, and a client (wifi-client.c) for an access point
 * the chip runs itself.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev.h"
#include "net/eth.h"
#include "rtl-wifi.h"
#include "trace.h"
#include "wifi-ccmp.h"
#include "wifi-frame.h"

/* REG_APS_FSMCO: the MAC power on and off requests clear once done */
#define REG_APS_FSMCO_1         0x0005
#define APFM_ONMAC              BIT(0)
#define APFM_OFF                BIT(1)
#define REG_APS_FSMCO_2         0x0006
#define POWER_READY             BIT(1)

/* the byte offset, then in the top byte the flag: read ready, write busy */
#define REG_EFUSE_CTRL          0x0030
#define EFUSE_FLAG              BIT(7)

/* EEPROM_EN: the efuse holds a map, which the driver then reads */
#define REG_9346CR              0x000a
#define EEPROM_EN               BIT(5)

/*
 * The logical map of a programmed efuse: its ID, and on the 8192F the RF
 * front end type the driver refuses to run without, and the SDIO MAC
 */
#define EEPROM_ID               0x0000
#define EEPROM_RFE_OPTION_8192F 0x00ca
#define EEPROM_MAC_ADDR_8192FS  0x011a
#define RFE_TYPE_8192F          0
/* the 8733B's: tpt_mode, antennas, RFE and country from 0xc8, then USB */
#define EEPROM_BOARD_8733B      0x00c8
#define EEPROM_USB_IDS_8733BU   0x0100
#define EEPROM_MAC_ADDR_8733BU  0x0108

/*
 * Firmware download: the checksum reports in once the download window
 * closes, and the firmware is up as soon as the driver hands over to it
 */
#define REG_MCUFWDL             0x0080
#define MCUFWDL_EN              BIT(0)
#define MCUFWDL_RDY             BIT(1)
#define FWDL_CHKSUM_RPT         BIT(2)
#define WINTINI_RDY             BIT(6)
#define RAM_DL_SEL              BIT(7)

/*
 * The 8733B (halmac) instead loads each 4 KiB chunk the driver parks in
 * the TX buffer with its DDMA engine, done the moment it is started and
 * with a good checksum; the firmware is up once the CPU starts with the
 * download closed (0x80 bit 14 set, bit 0 clear), and says so in bit 15
 */
#define REG_DDMA_CTRL           0x1208
#define DDMA_OWN                BIT(31)
#define DDMA_CHKSUM_STS         BIT(27)
#define REG_SYS_FUNC_EN_1       0x0003
#define CPU_EN                  BIT(2)
#define FWDL_EN_8733B           BIT(0)
#define FW_DW_RDY_8733B         BIT(14)
#define FW_INIT_RDY_8733B       BIT(15)

/*
 * What the 8733B reads of itself: its id and cut (D, which spares the
 * driver its retries for older ones), WLAN function on, non-secure
 * firmware, every TX queue empty, RX DMA idle; and in 0xf5 bit 0 the
 * powered-off state halmac checks before it powers the MAC on
 */
#define REG_SYS_CFG1_8733B      0x00f0
#define CUT_D_8733B             0x00003000
#define REG_CHIP_ID_8733B       0x00fc
#define CHIP_ID_8733B           0x16
#define REG_SYS_STATUS1_8733B   0x00f5
#define PWR_OFF_8733B           BIT(0)
#define REG_WL_FUNC_8733B       0x0068
#define WL_FUNC_EN_8733B        BIT(2)
#define REG_FW_SECURE_CTRL      0x0014
#define FW_NON_SECURE           0x03
#define REG_TXPKT_EMPTY         0x041a
#define REG_TXDMA_STATUS        0x0210
#define REG_RXDMA_CTRL_2        0x0286
#define RXDMA_IDLE              BIT(1)

/*
 * Its RF registers are memory in BB space, a path's at 4 bytes apiece: the
 * channel in RF_CHNLBW's low byte, and RF 0xc5 bit 15 the lock the driver
 * waits on after each channel, band or bandwidth switch
 */
#define REG_RF_A_8733B          0x3c00
#define REG_RF_B_8733B          0x4c00
#define RF_LOCK_8733B           (0xc5 * 4 + 1)
#define RF_LOCKED               BIT(7)

/*
 * The 8733B's calibrations wait on the BB: IQK on 0x2d9c reading 0x55 once
 * 0x1b00 bit 0 starts one (0x1b10 written first resets it), DPK and TSSI
 * on 0x2de0 counting 20 frames sent
 */
#define REG_IQK_CTRL_8733B      0x1b00
#define REG_IQK_CLR_8733B       0x1b10
#define REG_IQK_DONE_8733B      0x2d9c
#define IQK_DONE                0x55
#define REG_HWTX_CNT_8733B      0x2de0
#define HWTX_DONE               20

/*
 * The 8188E's IO offload: a command bit in [3:0] runs, then clears; an
 * error would show as the same bit in [7:4]
 */
#define REG_HMEBOX_E0           0x0088
#define IOL_CMD_MASK            0x0f

/*
 * The firmware's report mailbox: an id, a sequence byte, then the report.
 * A driver that leaves C2H_DEFEATURE_RSVD in it asks the firmware to post
 * the MAC hidden report there once it boots: what the part is fused for.
 */
#define REG_C2HEVT_MSG_NORMAL   0x01a0
#define C2H_MAC_HIDDEN_RPT      0x19
#define C2H_DEFEATURE_RSVD      0xfd

/* the network type of each port, two bits apiece: 3 runs an access point */
#define REG_MSR                 0x0102
#define MSR_NETTYPE(msr, port)  (((msr) >> (2 * (port))) & 3)
#define NT_AS_AP                3

/* the H2C boxes the firmware has not taken yet; it takes them at once */
#define REG_HMETFR              0x01cc

/* the LLT builds itself on request, clearing the request once built */
#define REG_AUTO_LLT_2          0x0226
#define AUTO_INIT_LLT           BIT(0)
/* or, on the 8188E, entry by entry: the operation in [31:30] runs at once */
#define REG_LLT_INIT_3          0x01e3
#define LLT_OP_MASK             0xc0

/*
 * The TX buffer's head and tail: the driver waits for them to meet, its
 * queues drained, before it leaves a channel
 */
#define REG_RQPN                0x0200
#define REG_TXBUF_TAIL          0x0204

/* a beacon page has arrived: write 1 to clear, for ports 0 and 1 */
#define REG_TDECTRL_2           0x020a
#define REG_DWBCN1_CTRL_2       0x022a
#define BCN_VALID               BIT(0)

/* REG_CAMCMD: polling bit, clear all, write, then the dword address */
#define REG_CAMCMD              0x0670
#define REG_CAMWRITE            0x0674
#define REG_CAMREAD             0x0678
#define CAM_POLLING             BIT(31)
#define CAM_CLR                 BIT(30)
#define CAM_WRITE               BIT(16)
#define CAM_CTRL_VALID          BIT(15)
#define CAM_CTRL_GK             BIT(6)

#define REG_RCR                 0x0608
#define RCR_APPFCS              BIT(31)
#define RCR_APP_PHYST_RXFF      BIT(28)
/* RXFLTMAP2: the data subtypes taken, none while scanning */
#define REG_RXFLTMAP2           0x06a4

/*
 * RF registers go through the BB's 3-wire port, one per path: the register
 * in [27:20], 20 bits of data below; RF_CHNLBW keeps the channel in its low
 * byte. The 8192F reads them back from a window per path, a dword each.
 */
#define REG_LSSI_PARAMETER      0x0840
#define REG_LSSI_PARAMETER_B    0x0844
#define REG_RF_WINDOW           0x2800
#define REG_RF_WINDOW_B         0x2c00
#define RF_CHNLBW               0x18

/* the RX descriptor, then its PHY status in 8-byte units */
#define RXDESC_SIZE             24
#define RXD_DRVINFO(n)          ((n) << 16)
#define RXD_SECURITY(n)         ((n) << 20)
#define RXD_PHY_STATUS          BIT(26)
#define RXD_LS                  BIT(28)
#define RXD_FS                  BIT(29)
/* the 8188F marks a C2H packet in DW2, the 8188E a TX report in DW3 */
#define RXD_RPT_SEL_C2H         BIT(28)
#define RXD_RPT_SEL_TXRPT1      (1 << 14)
#define DRVINFO_SIZE            32
#define SEC_AES                 4
#define FCS_LEN                 4

/* RX rates: 6 Mb/s for management, 54 Mb/s for data, both OFDM */
#define RX_RATE_6M              4
#define RX_RATE_54M             11

/* the TX descriptor: length, its own size, then queue, cipher and report */
#define TXD_QSEL(dw1)           (((dw1) >> 8) & 0x1f)
#define TXD_SEC_TYPE(dw1)       (((dw1) >> 22) & 3)
/*
 * 8-byte blocks a USB driver leaves between the descriptor and the frame,
 * so no transfer fills whole packets
 */
#define TXD_PKT_OFFSET(dw1)     (((dw1) >> 24) & 0x1f)
#define TXD_PKT_OFFSET_88E(dw1) (((dw1) >> 26) & 0x1f)
#define TXD_REPORT              BIT(19)
#define QSEL_BEACON             0x10
#define SEC_TYPE_AES            3

#define C2H_CCX_TX_RPT          0x03
#define CCX_RETRY_OVER          BIT(7)
#define TXRPT_INT_CCX           BIT(7)
#define TXRPT_PKT_OK            BIT(6)
/* the 8733B's: an extended C2H, its state in the second dword's top bits */
#define C2H_EXTEND              0xff
#define C2H_EXTEND_CCX_RPT      0x0f
#define CCX_TX_STATE_FAIL       (1u << 30)

/* indications the driver has not read yet before the network waits */
#define RX_QUEUE_MAX            64
#define BEACON_MS               102

/*
 * The hidden report: SDIO, every function and bandwidth (the driver keeps
 * what the chip has), the antennas in byte 6 [7:5], 802.11b/g/n; then a
 * cut version of 0xf, which changes none
 */
#define HIDDEN_RPT_ANT_SHIFT    5
static const uint8_t rtl_hidden_rpt[8 + 5] = {
    0, 0, 0, 0, 0x01, 0x0f, 0x02, 2 << 2,
    0x0f, 0, 0, 0, 0,
};

/*
 * The 8733B's layout: every function, 40 MHz (code 6) and one antenna in
 * byte 6, 802.11b/g/n, then the 5 GHz band (RPT_2 bit 1) and no Bluetooth
 */
static const uint8_t rtl_hidden_rpt_8733b[8 + 5] = {
    0, 0, 0, 0, 0, 0x0f, 1 << 5 | 6, 2 << 2,
    0, 0, 0, 0, 0x02,
};

/* What the MAC holds out of reset, where drivers check it */
static const struct {
    uint16_t reg;
    uint8_t size;
    uint32_t val;
} rtl_reset_regs[] = {
    /* the SIFS timings */
    { 0x0428, 2, 0x100a },
    { 0x0514, 4, 0x0e0a0e0a },
    { 0x063a, 2, 0x100a },
    { 0x063c, 4, 0x0e0e0a0a },
};

/*
 * A strong OFDM signal, -40 dBm: AGC gain, then the power in byte 4, the
 * EVM and SNR further on
 */
static const uint8_t rtl_phy_status[DRVINFO_SIZE] = {
    [0] = 0x23, [4] = 0x8c, [13] = 0xbe, [15] = 0x70,
};

/*
 * The same in the 8192F's layout: page 1 for OFDM, each path's power 110
 * dB up, then the EVM and SNR in half dBs
 */
static const uint8_t rtl_phy_status_8192f[DRVINFO_SIZE] = {
    [0] = 1, [1] = 70, [2] = 70, [16] = 0xc4, [17] = 0xc4, [24] = 60,
    [25] = 60,
};

static void rtl_queue(RtlCore *c, GByteArray *unit)
{
    trace_rtl_rx(unit->len);
    g_queue_push_tail(&c->rx, unit);
    c->notify(c);
}

GByteArray *rtl_core_rx_peek(RtlCore *c)
{
    return g_queue_peek_head(&c->rx);
}

GByteArray *rtl_core_rx_pop(RtlCore *c)
{
    GByteArray *m = g_queue_pop_head(&c->rx);

    if (m && g_queue_get_length(&c->rx) == RX_QUEUE_MAX - 1) {
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        if (c->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(c->client_nic));
        }
    }
    return m;
}

static void rtl_flush(RtlCore *c)
{
    GByteArray *m;

    while ((m = g_queue_pop_head(&c->rx))) {
        g_byte_array_unref(m);
    }
    c->notify(c);
}

/* An RX unit: the descriptor, the PHY status if asked for, the frame */
static GByteArray *rtl_unit(RtlCore *c, uint32_t dw0, uint32_t dw2,
                            uint32_t dw3, bool phy_status)
{
    GByteArray *u = g_byte_array_sized_new(RXDESC_SIZE + DRVINFO_SIZE);
    uint8_t d[RXDESC_SIZE] = {};

    if (phy_status) {
        dw0 |= RXD_DRVINFO(DRVINFO_SIZE / 8) | RXD_PHY_STATUS;
    }
    /* bit 28 is the 8733B's firmware status flag, not a last segment */
    if (c->family != RTL_FAMILY_8733B) {
        dw0 |= RXD_LS | RXD_FS;
    }
    stl_le_p(d, dw0);
    stl_le_p(d + 8, dw2);
    stl_le_p(d + 12, dw3);
    stl_le_p(d + 20, qemu_clock_get_us(QEMU_CLOCK_VIRTUAL));
    g_byte_array_append(u, d, sizeof(d));
    if (phy_status) {
        g_byte_array_append(u, c->family == RTL_FAMILY_8192F ||
                            c->family == RTL_FAMILY_8733B ?
                            rtl_phy_status_8192f : rtl_phy_status,
                            DRVINFO_SIZE);
    }
    return u;
}

/* The key id of the group key, as the CAM holds it */
static unsigned rtl_gtk_id(RtlCore *c)
{
    for (unsigned e = 0; e < RTL_CAM_WORDS / 8; e++) {
        uint32_t ctrl = c->cam[e * 8];

        if ((ctrl & CAM_CTRL_VALID) && (ctrl & CAM_CTRL_GK)) {
            return ctrl & 3;
        }
    }
    return 1;
}

/*
 * A frame off the air, as the hardware hands it over once decrypted: one
 * that came encrypted keeps its CCMP header, with room for the MIC after
 */
static void rtl_rx_frame(RtlCore *c, const uint8_t *frame, size_t len,
                         bool protected)
{
    uint32_t rcr = ldl_le_p(c->ioreg + REG_RCR);
    uint16_t fc;
    size_t hdr, on_air;
    uint8_t ccmp[WIFI_CCMP_HDR_LEN] = {}, tail[WIFI_CCMP_MIC_LEN + FCS_LEN];
    bool data;
    GByteArray *u;

    if (len < WIFI_HDR_LEN || !c->powered ||
        g_queue_get_length(&c->rx) >= RX_QUEUE_MAX) {
        return;
    }
    fc = lduw_le_p(frame);
    data = WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA;
    if (data && !lduw_le_p(c->ioreg + REG_RXFLTMAP2)) {
        return;
    }
    hdr = wifi_hdr_len(fc);
    on_air = len + (protected ? WIFI_CCMP_HDR_LEN + WIFI_CCMP_MIC_LEN : 0);
    if (protected) {
        bool group = is_multicast_ether_addr(frame + 4);
        uint64_t pn = group ? ++c->gtk_pn : ++c->ptk_pn;

        ccmp[0] = pn;
        ccmp[1] = pn >> 8;
        /* ExtIV, then the key id */
        ccmp[3] = 0x20 | (group ? rtl_gtk_id(c) : 0) << 6;
        stl_le_p(ccmp + 4, pn >> 16);
    }
    u = rtl_unit(c, (on_air + (rcr & RCR_APPFCS ? FCS_LEN : 0)) |
                    (protected ? RXD_SECURITY(SEC_AES) : 0),
                 0, data ? RX_RATE_54M : RX_RATE_6M,
                 rcr & RCR_APP_PHYST_RXFF);
    if (protected) {
        uint8_t h[32];

        memcpy(h, frame, MIN(hdr, sizeof(h)));
        stw_le_p(h, fc | WIFI_FC_PROTECTED);
        g_byte_array_append(u, h, MIN(hdr, sizeof(h)));
        g_byte_array_append(u, ccmp, sizeof(ccmp));
        g_byte_array_append(u, frame + hdr, len - hdr);
    } else {
        g_byte_array_append(u, frame, len);
    }
    /* the MIC the hardware checked, and the FCS, neither of them looked at */
    memset(tail, 0, sizeof(tail));
    g_byte_array_append(u, tail, (protected ? WIFI_CCMP_MIC_LEN : 0) +
                                 (rcr & RCR_APPFCS ? FCS_LEN : 0));
    rtl_queue(c, u);
}

/* From the access point, heard on its channel */
static void rtl_deliver(void *opaque, const uint8_t *frame, size_t len,
                        bool protected)
{
    RtlCore *c = opaque;

    if (c->channel == c->ap.channel) {
        rtl_rx_frame(c, frame, len, protected);
    }
}

/* Whether the chip runs an access point, one with a beacon to go by */
static bool rtl_ap_up(RtlCore *c)
{
    uint8_t msr = c->ioreg[REG_MSR];

    return c->powered && c->bcn && (MSR_NETTYPE(msr, 0) == NT_AS_AP ||
                                    MSR_NETTYPE(msr, 1) == NT_AS_AP);
}

/* From the client, on the channel of the chip's own BSS */
static void rtl_client_deliver(void *opaque, const uint8_t *frame,
                               size_t len, bool protected)
{
    RtlCore *c = opaque;

    if (rtl_ap_up(c)) {
        rtl_rx_frame(c, frame, len, protected);
    }
}

/* Tell the client whether there is an access point to join */
static void rtl_ap_update(RtlCore *c)
{
    if (!c->client_nic) {
        return;
    }
    if (rtl_ap_up(c)) {
        wifi_client_bss(&c->client, c->bcn_bssid.a, c->bcn_ssid,
                        c->bcn_ssid_len, NULL);
    } else {
        wifi_client_bss(&c->client, NULL, NULL, 0, NULL);
    }
}

/*
 * A beacon for the hardware to send. An access point's names the chip as
 * the BSS; the template a station leaves the firmware names the one it
 * joined, and changes nothing.
 */
static void rtl_ap_beacon(RtlCore *c, const uint8_t *f, size_t len)
{
    /* the SSID leads the elements, after timestamp, interval, capability */
    const size_t ie = WIFI_HDR_LEN + 12;

    if (len < ie + 2 || lduw_le_p(f) != WIFI_FC_MGMT(WIFI_STYPE_BEACON) ||
        memcmp(f + 10, f + 16, 6) || f[ie] != WIFI_EID_SSID ||
        ie + 2 + f[ie + 1] > len) {
        return;
    }
    c->bcn = true;
    memcpy(c->bcn_bssid.a, f + 16, 6);
    c->bcn_ssid_len = MIN(f[ie + 1], sizeof(c->bcn_ssid));
    memcpy(c->bcn_ssid, f + ie + 2, c->bcn_ssid_len);
    rtl_ap_update(c);
}

static void rtl_beacon(RtlCore *c)
{
    uint8_t b[WIFI_AP_BEACON_MAX];

    rtl_deliver(c, b, wifi_ap_beacon(&c->ap, b), false);
}

static void rtl_beacon_tick(void *opaque)
{
    RtlCore *c = opaque;

    rtl_beacon(c);
    timer_mod(c->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

/* The TX report a frame asked for: a C2H packet, but in band on the 8188E */
static void rtl_tx_report(RtlCore *c, const uint8_t *desc, bool acked)
{
    GByteArray *u;
    uint8_t r[16] = {};

    if (c->family == RTL_FAMILY_8733B) {
        r[0] = C2H_EXTEND;
        r[1] = c->c2h_seq++;
        r[2] = C2H_EXTEND_CCX_RPT;
        r[3] = sizeof(r) - 4;
        stl_le_p(r + 8, desc[24] << 16 | (acked ? 0 : CCX_TX_STATE_FAIL));
        u = rtl_unit(c, sizeof(r), RXD_RPT_SEL_C2H, 0, false);
        g_byte_array_append(u, r, sizeof(r));
    } else if (c->family != RTL_FAMILY_8188E) {
        r[0] = C2H_CCX_TX_RPT;
        r[1] = c->c2h_seq++;
        r[2] = acked ? 0 : CCX_RETRY_OVER;
        /* the tag the driver put in the descriptor's SW_DEFINE */
        r[8] = desc[24];
        u = rtl_unit(c, 10, RXD_RPT_SEL_C2H, 0, false);
        g_byte_array_append(u, r, 10);
    } else {
        r[0] = TXRPT_INT_CCX;
        r[1] = acked ? TXRPT_PKT_OK : 0;
        u = rtl_unit(c, 8, 0, RXD_RPT_SEL_TXRPT1, false);
        g_byte_array_append(u, r, 8);
    }
    rtl_queue(c, u);
}

/* The pairwise key the driver wrote for @addr, if any */
static bool rtl_ptk(RtlCore *c, const uint8_t *addr, uint8_t *tk)
{
    for (unsigned e = 0; e < RTL_CAM_WORDS / 8; e++) {
        const uint32_t *w = &c->cam[e * 8];
        uint8_t mac[6];

        stw_le_p(mac, w[0] >> 16);
        stl_le_p(mac + 2, w[1]);
        if ((w[0] & CAM_CTRL_VALID) && !(w[0] & CAM_CTRL_GK) &&
            !memcmp(mac, addr, 6)) {
            for (unsigned i = 0; i < 4; i++) {
                stl_le_p(tk + 4 * i, w[2 + i]);
            }
            return true;
        }
    }
    return false;
}

/*
 * A frame for the air. The client hears what the chip's own BSS sends, the
 * access point the rest on its channel; one the hardware is to encrypt
 * comes with its CCMP header and no MIC, one the driver encrypted itself
 * is decrypted again here.
 */
static void rtl_tx_frame(RtlCore *c, const uint8_t *desc,
                         const uint8_t *frame, size_t len)
{
    uint32_t dw1 = ldl_le_p(desc + 4);
    g_autofree uint8_t *f = NULL;
    uint8_t tk[16];
    uint16_t fc;
    size_t hdr;
    bool own, heard;

    if (TXD_QSEL(dw1) == QSEL_BEACON) {
        c->ioreg[REG_TDECTRL_2] |= BCN_VALID;
        c->ioreg[REG_DWBCN1_CTRL_2] |= BCN_VALID;
        /* the 8733B's firmware comes in as beacon pages too */
        if (c->family != RTL_FAMILY_8733B ||
            !(c->ioreg[REG_MCUFWDL] & FWDL_EN_8733B)) {
            rtl_ap_beacon(c, frame, len);
        }
        return;
    }
    if (len < WIFI_HDR_LEN) {
        return;
    }
    fc = lduw_le_p(frame);
    trace_rtl_tx(fc, len, TXD_SEC_TYPE(dw1));
    own = rtl_ap_up(c) && !memcmp(frame + 10, c->bcn_bssid.a, 6);
    if (own) {
        heard = c->client_nic &&
                (is_multicast_ether_addr(frame + 4) ||
                 wifi_client_acks(&c->client, frame + 4));
    } else {
        heard = c->channel == c->ap.channel &&
                (is_multicast_ether_addr(frame + 4) ||
                 !memcmp(frame + 4, c->ap.bssid.a, 6));
    }
    if (ldl_le_p(desc + 8) & TXD_REPORT) {
        rtl_tx_report(c, desc, heard);
    }
    if (!heard) {
        return;
    }
    f = g_memdup2(frame, len);
    if (fc & WIFI_FC_PROTECTED) {
        hdr = wifi_hdr_len(fc);
        if (TXD_SEC_TYPE(dw1) == SEC_TYPE_AES) {
            if (len < hdr + WIFI_CCMP_HDR_LEN) {
                return;
            }
            memmove(f + hdr, f + hdr + WIFI_CCMP_HDR_LEN,
                    len - hdr - WIFI_CCMP_HDR_LEN);
            len -= WIFI_CCMP_HDR_LEN;
            stw_le_p(f, fc & ~WIFI_FC_PROTECTED);
        } else if (!rtl_ptk(c, frame + 4, tk) ||
                   !wifi_ccmp_decrypt(tk, f, &len)) {
            return;
        }
    }
    if (own) {
        wifi_client_receive(&c->client, f, len);
    } else {
        wifi_ap_receive(&c->ap, f, len);
    }
}

void rtl_core_tx(RtlCore *c, const uint8_t *buf, size_t len)
{
    size_t p = 0;

    while (p + 8 <= len) {
        uint32_t dw1 = ldl_le_p(buf + p + 4);
        size_t pkt = lduw_le_p(buf + p), off = buf[p + 2];

        off += 8 * (c->family == RTL_FAMILY_8188E ? TXD_PKT_OFFSET_88E(dw1)
                                                   : TXD_PKT_OFFSET(dw1));

        if (!pkt || off < 8 || p + off + pkt > len) {
            break;
        }
        rtl_tx_frame(c, buf + p, buf + p + off, pkt);
        p = ROUND_UP(p + off + pkt, 8);
    }
}

/*
 * Clearing the flag starts a read of the byte at the offset; setting it
 * programs one, which the blank efuse here takes without changing
 */
static void rtl_efuse(RtlCore *c)
{
    uint8_t *r = c->ioreg + REG_EFUSE_CTRL;
    unsigned addr = (r[2] & 3) << 8 | r[1];

    if (r[3] & EFUSE_FLAG) {
        r[3] &= ~EFUSE_FLAG;
    } else {
        r[0] = addr < RTL_EFUSE_SIZE ? c->efuse[addr] : 0xff;
        r[3] |= EFUSE_FLAG;
    }
}

/*
 * One packet of the physical efuse: the words of @data at @offset, within
 * one 8-byte section, behind a header naming the section and the words
 */
static uint8_t *rtl_efuse_put(uint8_t *p, uint16_t offset,
                              const uint8_t *data, size_t len)
{
    unsigned section = offset / 8;
    unsigned words = ((1 << (len / 2)) - 1) << (offset % 8 / 2);
    uint8_t wden = 0x0f & ~words;

    if (section < 16) {
        *p++ = section << 4 | wden;
    } else {
        *p++ = (section & 0x07) << 5 | 0x0f;
        *p++ = (section & 0x78) << 1 | wden;
    }
    memcpy(p, data, len);
    return p + len;
}

/*
 * The 8192F's and 8733B's drivers will not run on a blank efuse, so they
 * get a map. The 8733B's: TX power from the thermal meter (tpt_mode 0),
 * one antenna on path A, RFE 0, then the USB IDs and the MAC.
 */
static void rtl_efuse_program(RtlCore *c)
{
    static const uint8_t id[2] = { 0x29, 0x81 };
    static const uint8_t rfe[2] = { RFE_TYPE_8192F, 0xff };
    static const uint8_t board_8733b[4] = { 0x00, 0x11, 0x00, 0xff };
    static const uint8_t usb_8733b[4] = { 0xda, 0x0b, 0x2b, 0xf7 };
    uint8_t *p = c->efuse;

    switch (c->family) {
    case RTL_FAMILY_8192F:
        p = rtl_efuse_put(p, EEPROM_ID, id, sizeof(id));
        p = rtl_efuse_put(p, EEPROM_RFE_OPTION_8192F, rfe, sizeof(rfe));
        rtl_efuse_put(p, EEPROM_MAC_ADDR_8192FS, c->conf.macaddr.a, 6);
        break;
    case RTL_FAMILY_8733B:
        p = rtl_efuse_put(p, EEPROM_ID, id, sizeof(id));
        p = rtl_efuse_put(p, EEPROM_BOARD_8733B, board_8733b,
                          sizeof(board_8733b));
        p = rtl_efuse_put(p, EEPROM_USB_IDS_8733BU, usb_8733b,
                          sizeof(usb_8733b));
        rtl_efuse_put(p, EEPROM_MAC_ADDR_8733BU, c->conf.macaddr.a, 6);
        break;
    default:
        break;
    }
}

/* What the firmware does as it comes up */
static void rtl_fw_boot(RtlCore *c)
{
    uint8_t *c2h = &c->ioreg[REG_C2HEVT_MSG_NORMAL];

    if (c2h[0] != C2H_DEFEATURE_RSVD) {
        return;
    }
    c2h[0] = C2H_MAC_HIDDEN_RPT;
    if (c->family == RTL_FAMILY_8733B) {
        memcpy(c2h + 2, rtl_hidden_rpt_8733b, sizeof(rtl_hidden_rpt_8733b));
        return;
    }
    memcpy(c2h + 2, rtl_hidden_rpt, sizeof(rtl_hidden_rpt));
    c2h[2 + 6] |= (c->family == RTL_FAMILY_8192F ? 2 : 1)
                  << HIDDEN_RPT_ANT_SHIFT;
}

/* A CAM command: one dword in or out by address, or every entry cleared */
static void rtl_cam(RtlCore *c)
{
    uint32_t cmd = ldl_le_p(c->ioreg + REG_CAMCMD);
    unsigned addr = cmd & (RTL_CAM_WORDS - 1);

    if (!(cmd & CAM_POLLING)) {
        return;
    }
    if (cmd & CAM_CLR) {
        memset(c->cam, 0, sizeof(c->cam));
    } else if (cmd & CAM_WRITE) {
        c->cam[addr] = ldl_le_p(c->ioreg + REG_CAMWRITE);
    } else {
        stl_le_p(c->ioreg + REG_CAMREAD, c->cam[addr]);
    }
    stl_le_p(c->ioreg + REG_CAMCMD, cmd & ~CAM_POLLING);
}

static bool rtl_covers(uint32_t off, size_t len, uint32_t reg)
{
    return off <= reg && off + len > reg;
}

/* The radio retuned: a dwell on the new channel catches a beacon */
static void rtl_channel(RtlCore *c, uint8_t ch)
{
    if (ch == c->channel) {
        return;
    }
    trace_rtl_channel(c->channel, ch);
    c->channel = ch;
    rtl_beacon(c);
}

/* An RF register write through a path's 3-wire port: follow the channel */
static void rtl_lssi(RtlCore *c, unsigned path)
{
    static const uint16_t port[] = {
        REG_LSSI_PARAMETER, REG_LSSI_PARAMETER_B,
    };
    static const uint16_t window[] = { REG_RF_WINDOW, REG_RF_WINDOW_B };
    uint32_t v = ldl_le_p(c->ioreg + port[path]);
    uint8_t reg = (v >> 20) & 0xff;
    uint8_t ch = v & 0xff;

    /* phydm changes RF_CHNLBW by reading it back, so it has to hold */
    if (c->family == RTL_FAMILY_8192F) {
        stl_le_p(c->ioreg + window[path] + reg * 4, v & 0xfffff);
    }
    if (!path && reg == RF_CHNLBW) {
        rtl_channel(c, ch);
    }
}

/* What the 8733B keeps reading the same, whatever the driver writes */
static void rtl_hold_8733b(RtlCore *c)
{
    uint8_t *r = c->ioreg;

    r[REG_TXPKT_EMPTY] = 0xff;
    r[REG_TXPKT_EMPTY + 1] = 0x0f;
    r[REG_RXDMA_CTRL_2] |= RXDMA_IDLE;
    r[REG_RF_A_8733B + RF_LOCK_8733B] |= RF_LOCKED;
    r[REG_RF_B_8733B + RF_LOCK_8733B] |= RF_LOCKED;
    stw_le_p(r + REG_HWTX_CNT_8733B, HWTX_DONE);
}

/*
 * The 8733B's handshakes after a write at @off: TXDMA_STATUS clears where a
 * 1 is written (@txdma held it before), a DDMA copy is done at once, the
 * firmware comes up with the CPU, the channel follows RF_CHNLBW, IQK ends
 */
static void rtl_write_8733b(RtlCore *c, uint32_t off, size_t len,
                            const uint8_t *txdma)
{
    uint8_t *r = c->ioreg;
    uint16_t fw = lduw_le_p(r + REG_MCUFWDL);

    for (unsigned i = 0; i < 4; i++) {
        if (rtl_covers(off, len, REG_TXDMA_STATUS + i)) {
            r[REG_TXDMA_STATUS + i] = txdma[i] & ~r[REG_TXDMA_STATUS + i];
        }
    }
    if (rtl_covers(off, len, REG_DDMA_CTRL + 3)) {
        stl_le_p(r + REG_DDMA_CTRL, ldl_le_p(r + REG_DDMA_CTRL) &
                                    ~(DDMA_OWN | DDMA_CHKSUM_STS));
    }
    if ((rtl_covers(off, len, REG_SYS_FUNC_EN_1) ||
         rtl_covers(off, len, REG_MCUFWDL) ||
         rtl_covers(off, len, REG_MCUFWDL + 1)) &&
        (r[REG_SYS_FUNC_EN_1] & CPU_EN) && (fw & FW_DW_RDY_8733B) &&
        !(fw & (FWDL_EN_8733B | FW_INIT_RDY_8733B))) {
        stw_le_p(r + REG_MCUFWDL, fw | FW_INIT_RDY_8733B);
        rtl_fw_boot(c);
    }
    if (rtl_covers(off, len, REG_RF_A_8733B + RF_CHNLBW * 4)) {
        rtl_channel(c, r[REG_RF_A_8733B + RF_CHNLBW * 4]);
    }
    if (rtl_covers(off, len, REG_IQK_CLR_8733B)) {
        r[REG_IQK_DONE_8733B] = 0;
    }
    if (rtl_covers(off, len, REG_IQK_CTRL_8733B) &&
        (r[REG_IQK_CTRL_8733B] & BIT(0))) {
        r[REG_IQK_DONE_8733B] = IQK_DONE;
    }
    rtl_hold_8733b(c);
}

void rtl_core_read(RtlCore *c, uint32_t off, uint8_t *buf, size_t len)
{
    memset(buf, 0, len);
    if (off < RTL_IOREG_SIZE) {
        memcpy(buf, c->ioreg + off, MIN(len, RTL_IOREG_SIZE - off));
    }
}

void rtl_core_write(RtlCore *c, uint32_t off, const uint8_t *buf, size_t len)
{
    uint8_t *fwdl = &c->ioreg[REG_MCUFWDL];
    uint8_t old_bcn[2] = {
        c->ioreg[REG_TDECTRL_2], c->ioreg[REG_DWBCN1_CTRL_2],
    };
    uint8_t txdma[4];
    bool b8733 = c->family == RTL_FAMILY_8733B;

    if (off >= RTL_IOREG_SIZE) {
        return;
    }
    len = MIN(len, RTL_IOREG_SIZE - off);
    memcpy(txdma, c->ioreg + REG_TXDMA_STATUS, sizeof(txdma));
    memcpy(c->ioreg + off, buf, len);

    if (rtl_covers(off, len, REG_APS_FSMCO_1)) {
        uint8_t v = c->ioreg[REG_APS_FSMCO_1];

        if (v & APFM_ONMAC) {
            c->powered = true;
        }
        if (v & APFM_OFF) {
            c->powered = false;
            rtl_flush(c);
            /* halmac's powered-off state, and the firmware gone with it */
            if (b8733) {
                c->ioreg[REG_SYS_STATUS1_8733B] |= PWR_OFF_8733B;
                stw_le_p(c->ioreg + REG_MCUFWDL, 0);
            }
        }
        c->ioreg[REG_APS_FSMCO_1] = v & ~(APFM_ONMAC | APFM_OFF);
        rtl_ap_update(c);
    }
    if (rtl_covers(off, len, REG_MSR)) {
        rtl_ap_update(c);
    }
    if (rtl_covers(off, len, REG_APS_FSMCO_2)) {
        c->ioreg[REG_APS_FSMCO_2] |= POWER_READY;
    }
    if (rtl_covers(off, len, REG_EFUSE_CTRL + 3)) {
        rtl_efuse(c);
    }
    if (rtl_covers(off, len, REG_HMEBOX_E0)) {
        c->ioreg[REG_HMEBOX_E0] &= ~IOL_CMD_MASK;
    }
    if (rtl_covers(off, len, REG_HMETFR)) {
        c->ioreg[REG_HMETFR] = 0;
    }
    if (rtl_covers(off, len, REG_AUTO_LLT_2)) {
        c->ioreg[REG_AUTO_LLT_2] &= ~AUTO_INIT_LLT;
    }
    if (rtl_covers(off, len, REG_LLT_INIT_3)) {
        c->ioreg[REG_LLT_INIT_3] &= ~LLT_OP_MASK;
    }
    if (!b8733 && off < REG_RQPN + 4 && off + len > REG_RQPN) {
        memcpy(c->ioreg + REG_TXBUF_TAIL, c->ioreg + REG_RQPN, 4);
    }
    /* BCN_VALID clears where the driver writes a 1 */
    if (rtl_covers(off, len, REG_TDECTRL_2)) {
        c->ioreg[REG_TDECTRL_2] = (c->ioreg[REG_TDECTRL_2] & ~BCN_VALID) |
                                  (old_bcn[0] & ~c->ioreg[REG_TDECTRL_2] &
                                   BCN_VALID);
    }
    if (rtl_covers(off, len, REG_DWBCN1_CTRL_2)) {
        c->ioreg[REG_DWBCN1_CTRL_2] =
            (c->ioreg[REG_DWBCN1_CTRL_2] & ~BCN_VALID) |
            (old_bcn[1] & ~c->ioreg[REG_DWBCN1_CTRL_2] & BCN_VALID);
    }
    if (rtl_covers(off, len, REG_CAMCMD + 3)) {
        rtl_cam(c);
    }
    if (b8733) {
        rtl_write_8733b(c, off, len, txdma);
        return;
    }
    if (rtl_covers(off, len, REG_LSSI_PARAMETER + 3)) {
        rtl_lssi(c, 0);
    }
    if (rtl_covers(off, len, REG_LSSI_PARAMETER_B + 3)) {
        rtl_lssi(c, 1);
    }
    if (rtl_covers(off, len, REG_MCUFWDL)) {
        if (!(*fwdl & MCUFWDL_EN)) {
            *fwdl |= FWDL_CHKSUM_RPT;
        }
        if (*fwdl & MCUFWDL_RDY) {
            *fwdl |= WINTINI_RDY | RAM_DL_SEL;
            rtl_fw_boot(c);
        }
    }
}

void rtl_core_reset(RtlCore *c)
{
    memset(c->ioreg, 0, sizeof(c->ioreg));
    memset(c->cam, 0, sizeof(c->cam));
    c->ioreg[REG_APS_FSMCO_2] = POWER_READY;
    if (c->efuse[0] != 0xff) {
        c->ioreg[REG_9346CR] = EEPROM_EN;
    }
    for (size_t i = 0; i < ARRAY_SIZE(rtl_reset_regs); i++) {
        stn_le_p(c->ioreg + rtl_reset_regs[i].reg, rtl_reset_regs[i].size,
                 rtl_reset_regs[i].val);
    }
    if (c->family == RTL_FAMILY_8733B) {
        stl_le_p(c->ioreg + REG_SYS_CFG1_8733B, CUT_D_8733B);
        c->ioreg[REG_CHIP_ID_8733B] = CHIP_ID_8733B;
        c->ioreg[REG_SYS_STATUS1_8733B] = PWR_OFF_8733B;
        stl_le_p(c->ioreg + REG_WL_FUNC_8733B, WL_FUNC_EN_8733B);
        c->ioreg[REG_FW_SECURE_CTRL] = FW_NON_SECURE;
        rtl_hold_8733b(c);
    }
    c->powered = false;
    /* the hal init starts out on channel 1 */
    c->channel = 1;
    c->c2h_seq = 0;
    c->ptk_pn = 0;
    c->gtk_pn = 0;
    c->bcn = false;
    rtl_ap_update(c);
    rtl_flush(c);
}

static bool rtl_can_receive(NetClientState *nc)
{
    RtlCore *c = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&c->rx) < RX_QUEUE_MAX;
}

static ssize_t rtl_receive(NetClientState *nc, const uint8_t *buf,
                           size_t size)
{
    RtlCore *c = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&c->ap, buf, size);
    return size;
}

static NetClientInfo rtl_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = rtl_can_receive,
    .receive = rtl_receive,
};

static bool rtl_client_can_receive(NetClientState *nc)
{
    RtlCore *c = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&c->client) &&
           g_queue_get_length(&c->rx) < RX_QUEUE_MAX;
}

static ssize_t rtl_client_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    RtlCore *c = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&c->client, buf, size);
    return size;
}

static NetClientInfo rtl_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = rtl_client_can_receive,
    .receive = rtl_client_receive,
};

void rtl_core_init(RtlCore *c)
{
    memset(c->efuse, 0xff, sizeof(c->efuse));
    c->beacon = timer_new_ms(QEMU_CLOCK_VIRTUAL, rtl_beacon_tick, c);
    g_queue_init(&c->rx);
}

bool rtl_core_realize(RtlCore *c, Error **errp)
{
    DeviceState *dev = c->dev;

    qemu_macaddr_default_if_unset(&c->conf.macaddr);
    rtl_efuse_program(c);
    c->nic = qemu_new_nic(&rtl_net_info, &c->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, c);
    if (!wifi_ap_realize(&c->ap, c->nic, rtl_deliver, c, errp)) {
        qemu_del_nic(c->nic);
        c->nic = NULL;
        return false;
    }
    if (c->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&c->client_conf.macaddr);
        c->client_nic = qemu_new_nic(&rtl_client_net_info, &c->client_conf,
                                     object_get_typename(OBJECT(dev)), NULL,
                                     &dev->mem_reentrancy_guard, c);
        wifi_client_init(&c->client, c->client_nic, &c->client_conf.macaddr,
                         rtl_client_deliver, c);
    }
    rtl_core_reset(c);
    timer_mod(c->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
    return true;
}

void rtl_core_unrealize(RtlCore *c)
{
    timer_del(c->beacon);
    if (c->client_nic) {
        wifi_client_cleanup(&c->client);
        qemu_del_nic(c->client_nic);
        c->client_nic = NULL;
    }
    wifi_ap_unrealize(&c->ap);
    qemu_del_nic(c->nic);
}

void rtl_core_finalize(RtlCore *c)
{
    GByteArray *m;

    timer_free(c->beacon);
    while ((m = g_queue_pop_head(&c->rx))) {
        g_byte_array_unref(m);
    }
}
