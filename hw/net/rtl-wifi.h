/*
 * Realtek RTL8188E, RTL8188F, RTL8192F and RTL8733B Wi-Fi: the chip the bus
 * front ends share
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_RTL_WIFI_H
#define HW_NET_RTL_WIFI_H

#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "net/net.h"

#define RTL_IOREG_SIZE          0x8000
#define RTL_EFUSE_SIZE          512
/* 32 entries of 8 dwords */
#define RTL_CAM_WORDS           256

typedef enum {
    RTL_FAMILY_8188E,
    RTL_FAMILY_8188F,
    RTL_FAMILY_8192F,
    RTL_FAMILY_8733B,
} RtlFamily;

typedef struct RtlCore RtlCore;

struct RtlCore {
    /* filled in by the front end before rtl_core_realize() */
    DeviceState *dev;
    void (*notify)(RtlCore *c);     /* the RX queue changed */
    RtlFamily family;

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    /* a station for the chip's own access point, given a backend */
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;

    QEMUTimer *beacon;
    uint8_t ioreg[RTL_IOREG_SIZE];
    uint8_t efuse[RTL_EFUSE_SIZE];
    uint32_t cam[RTL_CAM_WORDS];
    GQueue rx;                      /* GByteArrays, one RX unit each */
    bool powered;
    uint8_t channel;
    uint8_t c2h_seq;
    uint64_t ptk_pn;
    uint64_t gtk_pn;
    /* the BSS of the last access point beacon the driver handed over */
    bool bcn;
    MACAddr bcn_bssid;
    uint8_t bcn_ssid[32];
    uint8_t bcn_ssid_len;
};

/*
 * the access point on the air, and the network behind it; then the client
 * for the chip's own access point, and the network behind that
 */
#define DEFINE_RTL_PROPERTIES(_state, _core)                                \
    DEFINE_NIC_PROPERTIES(_state, _core.conf),                              \
    DEFINE_PROP_STRING("ssid", _state, _core.ap.ssid),                      \
    DEFINE_PROP_STRING("psk", _state, _core.ap.psk),                        \
    DEFINE_PROP_UINT8("channel", _state, _core.ap.channel, 6),              \
    DEFINE_PROP_MACADDR("bssid", _state, _core.ap.bssid),                   \
    DEFINE_PROP_NETDEV("client-netdev", _state, _core.client_conf.peers),   \
    DEFINE_PROP_MACADDR("client-mac", _state, _core.client_conf.macaddr)

void rtl_core_init(RtlCore *c);
bool rtl_core_realize(RtlCore *c, Error **errp);
void rtl_core_unrealize(RtlCore *c);
void rtl_core_finalize(RtlCore *c);
/* Power on: the MAC registers as reset leaves them, nothing queued */
void rtl_core_reset(RtlCore *c);

/* The WLAN register space */
void rtl_core_read(RtlCore *c, uint32_t off, uint8_t *buf, size_t len);
void rtl_core_write(RtlCore *c, uint32_t off, const uint8_t *buf, size_t len);

/* What a TX queue was handed: TX descriptors, each followed by its frame */
void rtl_core_tx(RtlCore *c, const uint8_t *buf, size_t len);

/* RX units for the host: an RX descriptor, then a PHY status and frame */
GByteArray *rtl_core_rx_peek(RtlCore *c);
GByteArray *rtl_core_rx_pop(RtlCore *c);

#endif
