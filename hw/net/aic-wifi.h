/*
 * AICSemi AIC8800 Wi-Fi: the chip and firmware the bus front ends share
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_AIC_WIFI_H
#define HW_NET_AIC_WIFI_H

#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "net/net.h"

#define AIC_VIFS                4
#define AIC_IES_MAX             256

typedef struct AicCore AicCore;

/* A word the chip reads back whatever the host writes: its identity */
typedef struct AicWord {
    uint32_t addr;
    uint32_t value;
} AicWord;

struct AicCore {
    /* filled in by the front end before aic_core_realize() */
    DeviceState *dev;
    void (*notify)(AicCore *c);     /* something is queued for the host */
    void (*replug)(AicCore *c);     /* the chip restarts, as app says */
    /*
     * The chip boots a ROM the host loads its firmware through, or runs
     * its firmware from power on (the DC and DW, whose driver patches it)
     */
    bool rom;
    /* the chip stays on the bus, its loader waiting on DBG_START_APP */
    bool start_cfm;
    const AicWord *words;           /* up to one with address 0 */

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    /* a station for the access point the host runs, given a backend */
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;

    bool app;                       /* the firmware runs, not the ROM */
    GHashTable *mem;                /* words the host wrote */
    GQueue msgs;                    /* confirmations and indications */
    GQueue frames;                  /* frames off the air */
    QEMUTimer *scan;
    QEMUTimer *join;

    /* the interfaces the host added: their type, or -1, and address */
    int8_t vif_type[AIC_VIFS];
    MACAddr vif_mac[AIC_VIFS];
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
    uint8_t ies[AIC_IES_MAX];
    uint16_t ies_len;
    uint8_t req_ies[AIC_IES_MAX + 64];
    uint16_t req_ies_len;
    uint8_t resp_ies[256];
    uint16_t resp_ies_len;

    /* the host's own access point, on interface ap_vif */
    bool bss_up;
    uint8_t ap_vif;
    uint16_t ap_freq;
    MACAddr ap_bssid;
    uint8_t ap_ssid[32];
    uint8_t ap_ssid_len;
    bool peer_added;
};

/*
 * the access point on the air, and the network behind it; then the client
 * for the chip's own access point, and the network behind that
 */
#define DEFINE_AIC_PROPERTIES(_state, _core)                                \
    DEFINE_NIC_PROPERTIES(_state, _core.conf),                              \
    DEFINE_PROP_STRING("ssid", _state, _core.ap.ssid),                      \
    DEFINE_PROP_STRING("psk", _state, _core.ap.psk),                        \
    DEFINE_PROP_UINT8("channel", _state, _core.ap.channel, 6),              \
    DEFINE_PROP_MACADDR("bssid", _state, _core.ap.bssid),                   \
    DEFINE_PROP_NETDEV("client-netdev", _state, _core.client_conf.peers),   \
    DEFINE_PROP_MACADDR("client-mac", _state, _core.client_conf.macaddr)

void aic_core_init(AicCore *c);
bool aic_core_realize(AicCore *c, Error **errp);
void aic_core_unrealize(AicCore *c);
void aic_core_finalize(AicCore *c);
/* Power on: the boot ROM runs, nothing queued */
void aic_core_reset(AicCore *c);

/* A transfer from the host: a message or a frame, with its header */
void aic_core_host(AicCore *c, const uint8_t *buf, size_t len);

/* What the chip has for the host, a packet a transfer */
GByteArray *aic_core_pop_msg(AicCore *c);
GByteArray *aic_core_pop_frame(AicCore *c);

#endif
