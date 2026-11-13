/*
 * Migration helpers for the Wi-Fi chip models: the message queues and
 * sparse register files their firmware models keep
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_WIFI_VMSTATE_H
#define HW_NET_WIFI_VMSTATE_H

#include "migration/vmstate.h"

/* a GQueue of GByteArray messages, in order */
extern const VMStateInfo vmstate_info_wifi_msgq;
/* a GByteArray pointer, NULL or not */
extern const VMStateInfo vmstate_info_wifi_bytes;
/* a GHashTable of 32-bit address to 32-bit value, as GUINT_TO_POINTER */
extern const VMStateInfo vmstate_info_wifi_u32map;

/* a MACAddr, for VMSTATE_STRUCT_ARRAY() */
extern const VMStateDescription vmstate_wifi_mac;

#define VMSTATE_WIFI_MAC_ARRAY(_f, _s, _n)                              \
    VMSTATE_STRUCT_ARRAY(_f, _s, _n, 0, vmstate_wifi_mac, MACAddr)

#define VMSTATE_WIFI_MSGQ(_f, _s)                                       \
    VMSTATE_SINGLE(_f, _s, 0, vmstate_info_wifi_msgq, GQueue)

#define VMSTATE_WIFI_BYTES(_f, _s)                                      \
    VMSTATE_SINGLE(_f, _s, 0, vmstate_info_wifi_bytes, GByteArray *)

#define VMSTATE_WIFI_U32MAP(_f, _s)                                     \
    VMSTATE_SINGLE(_f, _s, 0, vmstate_info_wifi_u32map, GHashTable *)

#endif
