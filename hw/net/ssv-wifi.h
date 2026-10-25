/*
 * iComm-semi SSV6006C, SSV6020 and SSV6030 Wi-Fi: the chip and firmware
 * the bus front ends share
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

typedef enum {
    SSV_CHIP_6006C,
    SSV_CHIP_6020,
    /* the SSV6355's: a FullMAC, its firmware in ssv-fmac.c */
    SSV_CHIP_6030,
} SsvChip;

/* what the core queues for the host before it stops taking frames */
#define SSV_OUT_QUEUE_MAX       64

#define SSV_FMAC_VIFS           2
#define SSV_FMAC_IES_MAX        384

/* The SSV6030 firmware's interfaces, and the BSSes it is in */
typedef struct SsvFmac {
    QEMUTimer *scan;
    QEMUTimer *join;
    /* the interfaces the host added: their type, or -1, and address */
    int8_t vif_type[SSV_FMAC_VIFS];
    MACAddr vif_mac[SSV_FMAC_VIFS];
    uint8_t scan_vif;
    bool scan_hit;

    /* the access point joined, by interface sta_vif */
    int sta;
    uint8_t sta_vif;
    MACAddr bssid;
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint16_t aid;
    uint16_t seq;
    unsigned tries;
    /* what the host adds to the association request, and both sides' */
    uint8_t ies[SSV_FMAC_IES_MAX];
    uint16_t ies_len;
    uint8_t req_ies[SSV_FMAC_IES_MAX + 64];
    uint16_t req_ies_len;
    uint8_t resp_ies[256];
    uint16_t resp_ies_len;

    /* the host's own access point, on interface ap_vif */
    uint8_t ap_vif;
    bool peer_added;
} SsvFmac;

typedef struct SsvCore SsvCore;

struct SsvCore {
    /* filled in by the front end before ssv_core_realize() */
    DeviceState *dev;
    void (*notify)(SsvCore *c);     /* a message for the host is queued */
    SsvChip chip;
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
    uint8_t pbuf_id;                /* the last packet buffer handed out */

    bool rf_on;
    uint8_t channel;
    /* the access point the host runs, from the beacon it hands over */
    bool bcn_valid;
    bool bss_up;
    MACAddr bssid;
    uint8_t bss_ssid[32];
    uint8_t bss_ssid_len;

    SsvFmac fmac;
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

/*
 * What the chip has for the host: frames and events, one each; the
 * SSV6030 aggregates them into a transfer
 */
GByteArray *ssv_core_peek(SsvCore *c);
GByteArray *ssv_core_pop(SsvCore *c);

/* The access point the host runs: up or down, and its beacon */
void ssv_core_set_bss(SsvCore *c, bool up);
void ssv_core_beacon(SsvCore *c, const uint8_t *frame, size_t len);

/* ssv-fmac.c: the SSV6030's firmware, behind the core's calls */
void ssv_fmac_init(SsvCore *c);
void ssv_fmac_finalize(SsvCore *c);
void ssv_fmac_reset(SsvCore *c);
uint32_t ssv_fmac_reg_read(SsvCore *c, uint32_t addr);
void ssv_fmac_reg_write(SsvCore *c, uint32_t addr, uint32_t val);
void ssv_fmac_host(SsvCore *c, const uint8_t *buf, size_t len);
GByteArray *ssv_fmac_pop(SsvCore *c);
void ssv_fmac_deliver(void *opaque, const uint8_t *frame, size_t len,
                      bool protected);
void ssv_fmac_client_deliver(void *opaque, const uint8_t *frame,
                             size_t len, bool protected);

#endif
