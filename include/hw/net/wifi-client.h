/*
 * A Wi-Fi client for an access point model to take in
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_WIFI_CLIENT_H
#define HW_NET_WIFI_CLIENT_H

#include "net/net.h"

/*
 * A station on the air of a chip model that runs an access point, as a
 * phone would be: once the chip has an open BSS up it joins, through open
 * authentication and association, then carries a network backend's
 * Ethernet frames to the access point and back as 802.11 data frames.
 *
 * Three-address frames leave no room for any source but the station, so
 * the client's address has to be the one its backend sends from: for a
 * user backend, 52:55 followed by the backend's host address.
 */

typedef void (*WifiClientDeliverFn)(void *opaque, const uint8_t *frame,
                                    size_t len);

typedef enum {
    WIFI_CLIENT_IDLE,
    WIFI_CLIENT_AUTHENTICATING,
    WIFI_CLIENT_ASSOCIATING,
    WIFI_CLIENT_CONNECTED,
} WifiClientState;

typedef struct WifiClient {
    NICState *nic;
    MACAddr mac;
    WifiClientDeliverFn deliver;
    void *opaque;
    QEMUTimer *timer;

    WifiClientState state;
    bool bss;
    MACAddr bssid;
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint16_t seq;
    unsigned tries;
} WifiClient;

void wifi_client_init(WifiClient *cl, NICState *nic, const MACAddr *mac,
                      WifiClientDeliverFn deliver, void *opaque);
void wifi_client_cleanup(WifiClient *cl);

/* An open BSS to join is up, or with @bssid NULL, gone */
void wifi_client_bss(WifiClient *cl, const uint8_t *bssid,
                     const uint8_t *ssid, size_t ssid_len);

/* Whether the client acknowledges a unicast frame sent to @ra */
bool wifi_client_acks(WifiClient *cl, const uint8_t *ra);

/* A frame from the access point, in the clear */
void wifi_client_receive(WifiClient *cl, const uint8_t *frame, size_t len);

/* An Ethernet frame from the network backend */
bool wifi_client_can_receive(WifiClient *cl);
void wifi_client_net_receive(WifiClient *cl, const uint8_t *buf,
                             size_t size);

#endif
