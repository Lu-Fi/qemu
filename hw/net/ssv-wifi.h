/*
 * iComm-semi SSV6006C Wi-Fi: the chip and firmware the bus front ends share
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_SSV_WIFI_H
#define HW_NET_SSV_WIFI_H

#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "net/net.h"

typedef struct SsvCore SsvCore;

struct SsvCore {
    /* filled in by the front end before ssv_core_realize() */
    DeviceState *dev;
    void (*notify)(SsvCore *c);     /* a message for the host is queued */
    uint32_t efuse_id;              /* what the eFuse says the part is */

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    /* a station for the access point the chip runs, given a backend */
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;

    GHashTable *regs;               /* the registers that are only storage */
    GQueue out;                     /* GByteArrays, for the host */
    QEMUTimer *beacon;
    QEMUTimer *heartbeat;

    /* the firmware download's checksum, and whether it runs */
    uint32_t fw_sum;
    bool fw_booting;
    bool fw_up;
    uint32_t fw_status;
    uint32_t heartbeats;

    bool rf_on;
    uint8_t channel;
    /* the access point the host runs, from the beacon it hands over */
    bool bcn_valid;
    bool bss_up;
    MACAddr bssid;
    uint8_t bss_ssid[32];
    uint8_t bss_ssid_len;
};

/*
 * the access point on the air, and the network behind it; then the client
 * for the chip's own access point, and the network behind that
 */
#define DEFINE_SSV_PROPERTIES(_state, _core)                                \
    DEFINE_NIC_PROPERTIES(_state, _core.conf),                              \
    DEFINE_PROP_STRING("ssid", _state, _core.ap.ssid),                      \
    DEFINE_PROP_STRING("psk", _state, _core.ap.psk),                        \
    DEFINE_PROP_UINT8("channel", _state, _core.ap.channel, 6),              \
    DEFINE_PROP_MACADDR("bssid", _state, _core.ap.bssid),                   \
    DEFINE_PROP_NETDEV("client-netdev", _state, _core.client_conf.peers),   \
    DEFINE_PROP_MACADDR("client-mac", _state, _core.client_conf.macaddr)

void ssv_core_init(SsvCore *c);
bool ssv_core_realize(SsvCore *c, Error **errp);
void ssv_core_unrealize(SsvCore *c);
void ssv_core_finalize(SsvCore *c);
/* Power on: the boot ROM runs, no firmware, nothing queued */
void ssv_core_reset(SsvCore *c);

uint32_t ssv_core_reg_read(SsvCore *c, uint32_t addr);
void ssv_core_reg_write(SsvCore *c, uint32_t addr, uint32_t val);
/* Firmware going into SRAM at @addr */
void ssv_core_fw_write(SsvCore *c, uint32_t addr, const uint8_t *buf,
                       size_t len);

/* A frame or a host command, with its descriptor or header */
void ssv_core_host(SsvCore *c, const uint8_t *buf, size_t len);

/* What the chip has for the host: frames and events, one each */
GByteArray *ssv_core_peek(SsvCore *c);
GByteArray *ssv_core_pop(SsvCore *c);

#endif
