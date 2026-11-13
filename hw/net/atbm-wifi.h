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
#include "hw/net/wifi-client.h"
#include "chardev/char-fe.h"
#include "hw/core/irq.h"
#include "net/net.h"
#include "migration/vmstate.h"

/* the interfaces the driver runs, and the links it numbers stations by */
#define ATBM_IFS                4
#define ATBM_LINKS              15

typedef struct AtbmCore AtbmCore;

/* 13549 is where the IoT firmware's drivers move to the new sleep */
#define IOT_FW_VERSION          14000
#define IOT_JUNK_MSGS           4

/*
 * The state of the IoT firmware that the Hera parts (ATBM6041, 6441) run
 * from their own flash: its own supplicant joins, and the host only gets
 * the outcome
 */
typedef struct AtbmIot {
    QEMUTimer *scan;
    QEMUTimer *join;
    bool scanning;
    bool connected;
    uint8_t ssid[32];
    uint8_t ssid_len;
    uint8_t keymgmt;
    uint8_t key_len;
    uint8_t key[64];
    uint8_t ps_mode;
    /* the sleep the host asked for, and what may wake it: 1 << reason */
    bool asleep;
    uint32_t wake_mask;
    /* the host's check on the firmware: its period (s) and timeouts */
    QEMUTimer *alive;
    uint32_t alive_period;
    uint32_t alive_tmo;
    uint32_t alive_missed;
    /* what a sleeping host's server says to wake it, from the heartbeat */
    char wake_key[64];
} AtbmIot;

/*
 * The ATBM6461's RTOS: the low-power firmware a camera talks to through
 * the driver's general command, and what its commands set
 */
typedef struct AtbmRtos {
    uint32_t upgrade_interval;
    uint32_t detect_range;
    uint32_t inc_interval;
    uint32_t pir_type;
    uint32_t rtc_mode;
    bool pir;
    bool wdt;
    bool ble;
    char fw_date[16];
} AtbmRtos;

struct AtbmCore {
    /* filled in by the front end before atbm_core_realize() */
    DeviceState *dev;
    void (*notify)(AtbmCore *c);    /* the queue for the host changed */
    uint8_t chip_id;
    uint8_t efuse[32];
    /* the startup indication's extended capabilities; bit 1 is Wi-Fi 6 */
    uint32_t excap;
    /* struct efuse_headr's bytes before the MAC, as the efuse MIB gives them */
    uint8_t trims[9];
    /* the Wi-Fi 6 drivers' dialect (atbm-606x branches) */
    bool wifi6;
    /*
     * The host's messages carry the Wi-Fi 6 header and transmit layout: the
     * Wi-Fi 6 dies, and the ATBM6461's driver on an IoT part
     */
    bool wide_hdr;
    /*
     * The ATBM6461's driver takes the buffers it may still fill as the
     * firmware's count of messages taken, plus the buffers there are,
     * less what it sent: the count it reads starts at the number of
     * buffers. The others' counts start at zero.
     */
    bool xmit_offset;
    /*
     * The firmware a driver loads decides the startup indication's config
     * word, which its sections do not say: older firmware gives stations
     * and interfaces as 16-bit halves
     */
    bool legacy_config;
    /*
     * A Hera part: the host may download firmware as it does to the other
     * chips (the 6041 with its 60xx driver), or find the IoT firmware
     * already running, which a driver that downloads nothing shows
     */
    bool hera;
    /* the part has no RAM firmware to download: the 6441 */
    bool iot_only;
    /* the host wrote to the firmware's memory since the CPU was held */
    bool fw_written;
    /* the IoT firmware's dialect is the one on the wire */
    bool iot;
    AtbmIot iot_state;
    /* the part runs the RTOS beside the Wi-Fi firmware: the 6461 */
    bool rtos;
    AtbmRtos rtos_state;

    /*
     * The 6441's low-power side: a UART to the SoC, the line that wakes it
     * and the lines in (the PIR, the charger)
     */
    bool mcu;
    CharFrontend uart;
    qemu_irq wake;
    uint8_t battery;
    uint16_t battery_mv;
    bool charging;
    bool pir;
    QEMUTimer *pir_timer;
    uint8_t uart_frame[132];
    unsigned uart_len;

    NICConf conf;
    NICState *nic;
    WifiAP ap;

    /* a phone for the chip's own access point to take in */
    NICConf client_conf;
    NICState *client_nic;
    WifiClient client;

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

    /* the interface running an access point, or -1, and its BSS */
    int ap_if;
    uint16_t ap_channel;
    uint8_t ap_ssid[32];
    uint8_t ap_ssid_len;
    bool beacon[ATBM_IFS];
    MACAddr beacon_bssid[ATBM_IFS];
    MACAddr link[ATBM_LINKS];
};

/*
 * The access point on the air and the network behind it; the client that
 * joins the chip's own access point, and the network behind that
 */
#define DEFINE_ATBM_PROPERTIES(_state, _core)                               \
    DEFINE_NIC_PROPERTIES(_state, _core.conf),                              \
    DEFINE_PROP_STRING("ssid", _state, _core.ap.ssid),                      \
    DEFINE_PROP_STRING("psk", _state, _core.ap.psk),                        \
    DEFINE_PROP_UINT8("channel", _state, _core.ap.channel, 6),              \
    DEFINE_PROP_MACADDR("bssid", _state, _core.ap.bssid),                   \
    DEFINE_PROP_NETDEV("client-netdev", _state, _core.client_conf.peers),   \
    DEFINE_PROP_MACADDR("client-mac", _state, _core.client_conf.macaddr),   \
    DEFINE_PROP_BOOL("legacy-config", _state, _core.legacy_config, false)

extern const VMStateDescription vmstate_atbm_core;

#define VMSTATE_ATBM_CORE(_f, _s)                                       \
    VMSTATE_STRUCT(_f, _s, 0, vmstate_atbm_core, AtbmCore)

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

/* A message for the host, and the answer to a request of its */
void atbm_core_send(AtbmCore *c, uint16_t id, const void *body, size_t len);
void atbm_core_confirm(AtbmCore *c, uint16_t cnf, uint32_t status,
                       const void *data, size_t len);

/* The IoT firmware's dialect, in atbm-iot.c */
extern const VMStateDescription vmstate_atbm_iot;
extern const VMStateDescription vmstate_atbm_rtos;

void atbm_iot_connect(AtbmCore *c, const uint8_t *ssid, size_t ssid_len,
                      uint8_t keymgmt, const uint8_t *key, size_t key_len);
void atbm_iot_init(AtbmCore *c);
void atbm_iot_finalize(AtbmCore *c);
void atbm_iot_reset(AtbmCore *c);
void atbm_iot_wsm(AtbmCore *c, uint16_t id, const uint8_t *body, size_t len);
/* The reasons the firmware wakes a sleeping host for */
enum {
    ATBM_WAKEUP_IO,
    ATBM_WAKEUP_NETWORK,
    ATBM_WAKEUP_CONNECT,
    ATBM_WAKEUP_KEEPALIVE,
};
/* A data frame came in for the host: if it is asleep it may be what wakes it */
void atbm_iot_net_rx(AtbmCore *c, const uint8_t *frame, size_t len);
/* The host sleeps, and if it asked to hear of this reason it wakes */
void atbm_iot_wake(AtbmCore *c, unsigned reason);

/* The RTOS's commands, in atbm-rtos.c: fills the answer to a request */
void atbm_rtos_cmd(AtbmCore *c, const uint8_t *req, size_t len,
                   uint8_t *resp, size_t resp_len);

/* The MCU, in atbm-mcu.c */
void atbm_mcu_init(AtbmCore *c);
void atbm_mcu_realize(AtbmCore *c);
void atbm_mcu_unrealize(AtbmCore *c);
void atbm_mcu_finalize(AtbmCore *c);

/* A message from the host, past the header its bus puts in front */
void atbm_core_wsm(AtbmCore *c, uint16_t id, const uint8_t *body, size_t len);

/* Messages for the host: {le16 len; le16 id | seq << 13; body} */
GByteArray *atbm_core_peek(AtbmCore *c);
GByteArray *atbm_core_pop(AtbmCore *c);

#endif
