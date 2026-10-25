/*
 * A Wi-Fi access point for a station model to join
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_WIFI_AP_H
#define HW_NET_WIFI_AP_H

#include "net/net.h"

/*
 * One BSS with room for one station: it beacons, takes the station through
 * open authentication, association and, given a passphrase, the WPA2-PSK
 * 4-way handshake, then bridges the station's data to a network backend as
 * Ethernet frames.
 *
 * Frames cross between the access point and the chip model as 802.11 MPDUs
 * without an FCS. Ciphers stay on the chip's side: the model takes off what
 * its hardware would encrypt before passing a frame in, and for a frame
 * passed out it is told whether the air carried it encrypted.
 */

#define WIFI_AP_BEACON_MAX  160

/* the key id the group key goes out under */
#define WIFI_AP_GTK_ID      1

typedef void (*WifiAPDeliverFn)(void *opaque, const uint8_t *frame,
                                size_t len, bool protected);

typedef enum {
    WIFI_AP_IDLE,
    WIFI_AP_AUTHENTICATED,
    WIFI_AP_ASSOCIATED,
    WIFI_AP_PTK_START,          /* sent message 1 of 4 */
    WIFI_AP_PTK_NEGOTIATING,    /* sent message 3 of 4 */
    WIFI_AP_CONNECTED,
} WifiAPState;

typedef struct WifiAP {
    /* properties of the owner */
    char *ssid;
    char *psk;
    uint8_t channel;
    MACAddr bssid;

    NICState *nic;
    WifiAPDeliverFn deliver;
    void *opaque;
    QEMUTimer *eapol_timer;

    WifiAPState state;
    MACAddr sta;
    uint16_t seq;
    unsigned tries;
    uint64_t replay;
    uint8_t pmk[32];
    uint8_t anonce[32];
    uint8_t ptk[48];
    uint8_t gtk[16];
} WifiAP;

bool wifi_ap_realize(WifiAP *ap, NICState *nic, WifiAPDeliverFn deliver,
                     void *opaque, Error **errp);
void wifi_ap_unrealize(WifiAP *ap);
void wifi_ap_reset(WifiAP *ap);

/* A beacon as heard now, into a buffer of WIFI_AP_BEACON_MAX bytes */
size_t wifi_ap_beacon(WifiAP *ap, uint8_t *buf);

/* A frame from the station, with whatever cipher the air used taken off */
void wifi_ap_receive(WifiAP *ap, const uint8_t *frame, size_t len);

/* An Ethernet frame from the network backend */
void wifi_ap_net_receive(WifiAP *ap, const uint8_t *buf, size_t size);

#endif
