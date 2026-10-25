/*
 * The parts of 802.11 frames the Wi-Fi models build and take apart
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_WIFI_FRAME_H
#define HW_NET_WIFI_FRAME_H

#include "qemu/bitops.h"

#define WIFI_FC_TYPE(fc)        (((fc) >> 2) & 3)
#define WIFI_FC_STYPE(fc)       (((fc) >> 4) & 0xf)
#define WIFI_FC_TODS            BIT(8)
#define WIFI_FC_FROMDS          BIT(9)
#define WIFI_FC_PROTECTED       BIT(14)

#define WIFI_TYPE_MGMT          0
#define WIFI_TYPE_DATA          2

#define WIFI_STYPE_ASSOC_REQ    0
#define WIFI_STYPE_ASSOC_RESP   1
#define WIFI_STYPE_REASSOC_REQ  2
#define WIFI_STYPE_REASSOC_RESP 3
#define WIFI_STYPE_PROBE_REQ    4
#define WIFI_STYPE_PROBE_RESP   5
#define WIFI_STYPE_BEACON       8
#define WIFI_STYPE_DISASSOC     10
#define WIFI_STYPE_AUTH         11
#define WIFI_STYPE_DEAUTH       12

/* data subtypes: bit 3 adds the QoS control field, bit 2 drops the body */
#define WIFI_STYPE_QOS          BIT(3)
#define WIFI_STYPE_NODATA       BIT(2)

#define WIFI_FC_MGMT(stype)     ((stype) << 4)
#define WIFI_FC_DATA            0x0008

/* three addresses, no QoS control */
#define WIFI_HDR_LEN            24

#define WIFI_CAP_ESS            BIT(0)
#define WIFI_CAP_PRIVACY        BIT(4)
#define WIFI_CAP_SHORT_PREAMBLE BIT(5)
#define WIFI_CAP_SHORT_SLOT     BIT(10)

#define WIFI_EID_SSID           0
#define WIFI_EID_RSN            48

/* LLC/SNAP before an Ethernet payload: the protocol follows */
#define WIFI_RFC1042            ((const uint8_t[]) { 0xaa, 0xaa, 0x03, \
                                                     0x00, 0x00, 0x00 })

/* the 802.11b/g rates, the b ones basic */
#define WIFI_RATES_IE           ((const uint8_t[]) { 1, 8, 0x82, 0x84, \
                                                     0x8b, 0x96, 0x0c, \
                                                     0x12, 0x18, 0x24 })
#define WIFI_XRATES_IE          ((const uint8_t[]) { 50, 4, 0x30, 0x48, \
                                                     0x60, 0x6c })

#endif
