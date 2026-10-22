/*
 * Broadcom FullMAC Wi-Fi: the dongle firmware the bus front ends share
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_BCM_WIFI_H
#define HW_NET_BCM_WIFI_H

#include "hw/net/wifi-ap.h"
#include "hw/net/wifi-client.h"
#include "net/net.h"

/* the SDPCM channels: BCDC ioctls, events, then Ethernet frames */
#define BCM_CHAN_CONTROL        0
#define BCM_CHAN_EVENT          1
#define BCM_CHAN_DATA           2

#define BCM_EVENT_MASK_LEN      16
#define BCM_IES_MAX             256

typedef struct BcmMsg {
    unsigned chan;
    GByteArray *data;
} BcmMsg;

typedef enum {
    BCM_STA_IDLE,
    BCM_STA_JOINING,                /* asked to, once the ioctl is answered */
    BCM_STA_AUTHENTICATING,
    BCM_STA_ASSOCIATING,
    BCM_STA_ASSOCIATED,
} BcmStaState;

typedef struct BcmCore BcmCore;

struct BcmCore {
    /* filled in by the front end before bcm_core_realize() */
    DeviceState *dev;
    void (*notify)(BcmCore *c);     /* a message for the host is queued */

    NICConf conf;
    NICState *nic;
    WifiAP ap;
    /* a station for the access point the dongle runs, given a backend */
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;

    QEMUTimer *scan_timer;
    QEMUTimer *join_timer;
    GQueue out;                     /* BcmMsg *, for the host */
    MACAddr mac;                    /* cur_etheraddr */
    uint8_t event_mask[BCM_EVENT_MASK_LEN];
    uint32_t wsec;
    uint32_t wpa_auth;
    uint32_t auth;
    uint32_t chanspec;
    uint8_t wpaie[BCM_IES_MAX - 64];
    size_t wpaie_len;
    uint16_t scan_sync_id;

    /* the BSS joined as a station */
    BcmStaState sta;
    MACAddr bssid;
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint16_t seq;
    unsigned tries;
    uint8_t req_ies[BCM_IES_MAX];
    size_t req_ies_len;
    uint8_t resp_ies[BCM_IES_MAX];
    size_t resp_ies_len;

    /* the BSS run as an access point, and the one station it takes */
    bool ap_mode;
    bool bss_up;
    uint8_t ap_ssid[32];
    uint8_t ap_ssid_len;
    bool peer_assoc;
    MACAddr peer;
    uint8_t peer_ies[BCM_IES_MAX];
    size_t peer_ies_len;
};

/*
 * the access point on the air, and the network behind it; then the client
 * for the dongle's own access point, and the network behind that
 */
#define DEFINE_BCM_PROPERTIES(_state, _core)                                \
    DEFINE_NIC_PROPERTIES(_state, _core.conf),                              \
    DEFINE_PROP_STRING("ssid", _state, _core.ap.ssid),                      \
    DEFINE_PROP_STRING("psk", _state, _core.ap.psk),                        \
    DEFINE_PROP_UINT8("channel", _state, _core.ap.channel, 6),              \
    DEFINE_PROP_MACADDR("bssid", _state, _core.ap.bssid),                   \
    DEFINE_PROP_NETDEV("client-netdev", _state, _core.client_conf.peers),   \
    DEFINE_PROP_MACADDR("client-mac", _state, _core.client_conf.macaddr)

void bcm_core_init(BcmCore *c);
bool bcm_core_realize(BcmCore *c, Error **errp);
void bcm_core_unrealize(BcmCore *c);
void bcm_core_finalize(BcmCore *c);
/* The dongle stops: what the firmware knew is gone, nothing is queued */
void bcm_core_reset(BcmCore *c);

/* A message from the host on @chan: a BCDC ioctl, or a BDC data frame */
void bcm_core_host(BcmCore *c, unsigned chan, const uint8_t *buf,
                   size_t len);

/* Messages for the host, in order */
BcmMsg *bcm_core_peek(BcmCore *c);
BcmMsg *bcm_core_pop(BcmCore *c);
void bcm_msg_free(BcmMsg *m);

#endif
