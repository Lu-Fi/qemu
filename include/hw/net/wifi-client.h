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
 * phone would be: once the chip has a BSS up it joins, through open
 * authentication and association and, given a passphrase, the WPA2-PSK
 * 4-way handshake as the supplicant, then carries a network backend's
 * Ethernet frames to the access point and back as 802.11 data frames.
 * Ciphers stay on the chip's side, as for the access point model: frames
 * cross in the clear, the client saying which the air carried encrypted.
 *
 * Three-address frames leave no room for any source but the station, so
 * the client's address has to be the one its backend sends from: for a
 * user backend, 52:55 followed by the backend's host address.
 */

typedef void (*WifiClientDeliverFn)(void *opaque, const uint8_t *frame,
                                    size_t len, bool protected);

typedef enum {
    WIFI_CLIENT_IDLE,
    WIFI_CLIENT_AUTHENTICATING,
    WIFI_CLIENT_ASSOCIATING,
    WIFI_CLIENT_KEYING,         /* in the 4-way handshake */
    WIFI_CLIENT_CONNECTED,
} WifiClientState;

typedef struct WifiClient {
    /* a property of the owner: a WPA2-PSK BSS's passphrase, or none */
    char *psk;

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

    /* WPA2-PSK: a passphrase and an RSN element from the BSS */
    bool rsn;
    uint8_t rsn_ie[22];         /* the client's own, as it associates */
    uint64_t replay;
    uint8_t pmk[32];
    uint8_t anonce[32];
    uint8_t snonce[32];
    uint8_t ptk[48];
    uint8_t gtk[32];            /* CCMP's or TKIP's */
} WifiClient;

void wifi_client_init(WifiClient *cl, NICState *nic, const MACAddr *mac,
                      WifiClientDeliverFn deliver, void *opaque);
void wifi_client_cleanup(WifiClient *cl);

/*
 * A BSS to join is up, or with @bssid NULL, gone; @rsn its RSN element
 * (from the ID on), NULL for an open BSS
 */
void wifi_client_bss(WifiClient *cl, const uint8_t *bssid,
                     const uint8_t *ssid, size_t ssid_len,
                     const uint8_t *rsn);

/* Whether the client acknowledges a unicast frame sent to @ra */
bool wifi_client_acks(WifiClient *cl, const uint8_t *ra);

/* A frame from the access point, in the clear */
void wifi_client_receive(WifiClient *cl, const uint8_t *frame, size_t len);

/* An Ethernet frame from the network backend */
bool wifi_client_can_receive(WifiClient *cl);
void wifi_client_net_receive(WifiClient *cl, const uint8_t *buf,
                             size_t size);

#endif
