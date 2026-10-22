/*
 * AltoBeam ATBM60xx Wi-Fi: the firmware model the bus front ends share
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_ATBM_WIFI_H
#define HW_NET_ATBM_WIFI_H

#include "hw/net/wifi-ap.h"
#include "net/net.h"

typedef struct AtbmCore AtbmCore;

struct AtbmCore {
    /* filled in by the front end before atbm_core_realize() */
    DeviceState *dev;
    void (*notify)(AtbmCore *c);    /* the queue for the host changed */
    uint8_t chip_id;
    uint8_t efuse[32];
    /* the Wi-Fi 6 drivers' dialect (atbm-606x branches) */
    bool wifi6;

    NICConf conf;
    NICState *nic;
    WifiAP ap;

    QEMUTimer *boot;
    QEMUTimer *scan;
    GHashTable *mem;
    GQueue out;
    uint32_t xmited;
    uint8_t seq;
    uint8_t efuse_byte;

    unsigned scan_channels;
    bool scan_hit;
    uint8_t scan_if;
    uint8_t join_if;
    bool ptk_set;
    bool gtk_set;
    uint8_t gtk_id;
    uint64_t ptk_pn;
    uint64_t gtk_pn;
};

/* the access point on the air, and the network behind it */
#define DEFINE_ATBM_PROPERTIES(_state, _core)                               \
    DEFINE_NIC_PROPERTIES(_state, _core.conf),                              \
    DEFINE_PROP_STRING("ssid", _state, _core.ap.ssid),                      \
    DEFINE_PROP_STRING("psk", _state, _core.ap.psk),                        \
    DEFINE_PROP_UINT8("channel", _state, _core.ap.channel, 6),              \
    DEFINE_PROP_MACADDR("bssid", _state, _core.ap.bssid)

void atbm_core_init(AtbmCore *c);
bool atbm_core_realize(AtbmCore *c, Error **errp);
void atbm_core_unrealize(AtbmCore *c);
void atbm_core_finalize(AtbmCore *c);

/* The CPU held: the firmware's state and whatever it had queued are gone */
void atbm_core_hold(AtbmCore *c);
/* The CPU let go: the firmware reports in shortly */
void atbm_core_run(AtbmCore *c);

uint32_t atbm_core_read32(AtbmCore *c, uint32_t addr);
void atbm_core_write32(AtbmCore *c, uint32_t addr, uint32_t val);

/* A message from the host, past the header its bus puts in front */
void atbm_core_wsm(AtbmCore *c, uint16_t id, const uint8_t *body, size_t len);

/* Messages for the host: {le16 len; le16 id | seq << 13; body} */
GByteArray *atbm_core_peek(AtbmCore *c);
GByteArray *atbm_core_pop(AtbmCore *c);

#endif
