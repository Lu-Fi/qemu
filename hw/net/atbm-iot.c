/*
 * AltoBeam ATBM6x41 Wi-Fi: the IoT firmware's dialect
 *
 * The Hera parts run firmware of their own from flash, and the drivers
 * that know that (the atbm6041_wifi_sdio and atbm6441_wifi_sdio modules)
 * download nothing. The firmware keeps the supplicant: the host hands it
 * an SSID and a key in a join request and hears the outcome in an event;
 * scans, the keep-alive and the sleep settings are requests of the IoT
 * firmware's too. Its request ids overlap the ones of the firmware the
 * other chips download (a join is 0x0003, a scan 0x000a), so it is a
 * dialect apart. Data frames still cross as WSM transmit and receive
 * messages, handled with the others' in atbm-wifi.c.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "net/eth.h"
#include "wifi-frame.h"
#include "atbm-wifi.h"
#include "trace.h"

#define IOT_CONFIG_REQ          0x0002
#define IOT_JOIN_REQ            0x0003
#define IOT_PSMODE_REQ          0x0008
#define IOT_AT_CMD_REQ          0x0023
#define IOT_SCAN_REQ            0x000a
#define IOT_SCAN_INFO_REQ       0x000b
#define IOT_GET_VER_REQ         0x001e
#define IOT_GET_SDK_VER_REQ     0x001f
#define IOT_FW_SLEEP_REQ        0x0026
#define IOT_SCAN_STATE_REQ      0x0029
#define IOT_RSSI_REQ            0x002a
#define IOT_SDK_VERSION         "0.3.8.4"
#define IOT_WAKEUP_EVENT_REQ    0x0037
#define IOT_CHECK_ALIVE_REQ     0x003a

#define WSM_EVENT_IND           0x0805
/* what the host reads of an event: its id, then this many bytes */
#define EVENT_BODY              504
#define EVENT_BSS_CONNECT       1
#define EVENT_HOST_WAKEUP       12
#define EVENT_HOST_CONN_ERR     14

/* powersave_mode: no sleep, modem sleep, light sleep, deep sleep */
#define PS_LIGHT_SLEEP          2

#define KEY_MGMT_NONE           0
#define CONNECT_NOT_FOUND_SSID  0
#define CONNECT_JOIN_FAIL       1
#define CIPHER_CCMP             4
#define MODE_11BG               3

/* the host's check on the firmware, and the firmware's answer to it */
#define ALIVE_FROM_FW           2121

#define SCAN_MS                 100
#define JOIN_MS                 50
#define XMITED_ADDR             0x09000100

/*
 * struct wsm_scan_result_info: SSID, BSSID and length, the channel, the
 * flag bits (HT, WPA, RSN, WPS, P2P, 40 MHz, encrypted), the RSSI
 */
#define SCAN_INFO_SIZE          47
#define SCAN_INFO_HDR           8
#define SCAN_INFO_MAX           30
#define SCAN_FLAG_RSN           BIT(2)
#define SCAN_FLAG_ENCRYPT       BIT(6)

/* the struct wsm_sdio_getconfig_cnf and the connect event inside it */
#define CFG_CONNECT_EVENT       32
#define CONN_EVENT_SIZE         64
#define CFG_BSS                 (CFG_CONNECT_EVENT + CONN_EVENT_SIZE)

/* struct wsm_join, which has no status: flags, BSSID, SSID, then the key */
#define JOIN_SSID_LEN           7
#define JOIN_SSID               8
#define JOIN_KEYMGMT            40
#define JOIN_KEY_LEN            41
#define JOIN_KEY                44
#define JOIN_SIZE               108

static void atbm_iot_event(AtbmCore *c, uint32_t id, const void *data,
                           size_t len)
{
    uint8_t ev[4 + EVENT_BODY] = {};

    stl_le_p(ev, id);
    memcpy(ev + 4, data, MIN(len, EVENT_BODY));
    atbm_core_send(c, WSM_EVENT_IND, ev, sizeof(ev));
}

/* The firmware's view of the connection, as events and the config show it */
static void atbm_iot_conn_event(AtbmCore *c, uint8_t *e)
{
    AtbmIot *iot = &c->iot_state;

    memcpy(e, iot->ssid, iot->ssid_len);
    e[32] = iot->ssid_len;
    memcpy(e + 33, c->ap.bssid.a, 6);
    if (iot->keymgmt != KEY_MGMT_NONE) {
        e[39] = CIPHER_CCMP;
        e[40] = CIPHER_CCMP;
    }
    e[41] = iot->keymgmt;
    e[42] = MODE_11BG;
}

/*
 * A host asleep is told why the chip woke it, over the event and the wake
 * line, if it asked to hear of that reason
 */
void atbm_iot_wake(AtbmCore *c, unsigned reason)
{
    AtbmIot *iot = &c->iot_state;
    uint8_t why = reason;

    if (!c->iot || !iot->asleep || !(iot->wake_mask & BIT(reason))) {
        return;
    }
    trace_atbm_iot_wake(reason);
    iot->asleep = false;
    atbm_iot_event(c, EVENT_HOST_WAKEUP, &why, 1);
    if (c->mcu) {
        qemu_set_irq(c->wake, 1);
    }
}

/*
 * The keep-alive: a null frame to the access point each period, and a
 * sleeping host woken when the link has gone unanswered as often as it
 * allowed
 */
static void atbm_iot_alive(void *opaque)
{
    AtbmCore *c = opaque;
    AtbmIot *iot = &c->iot_state;
    uint8_t f[WIFI_HDR_LEN] = {};

    if (iot->connected && c->ap.state == WIFI_AP_CONNECTED) {
        stw_le_p(f, WIFI_FC_DATA | WIFI_FC_TODS |
                    (WIFI_STYPE_NODATA << 4));
        memcpy(f + 4, c->ap.bssid.a, 6);
        memcpy(f + 10, c->conf.macaddr.a, 6);
        memcpy(f + 16, c->ap.bssid.a, 6);
        wifi_ap_receive(&c->ap, f, sizeof(f));
        iot->alive_missed = 0;
    } else {
        iot->alive_missed++;
    }
    trace_atbm_iot_keepalive(iot->alive_missed);
    if (iot->alive_missed >= iot->alive_tmo) {
        atbm_iot_wake(c, ATBM_WAKEUP_KEEPALIVE);
    }
    timer_mod(iot->alive, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                          iot->alive_period * 1000);
}

/*
 * The heartbeat to the cloud server that keeps the host's wake up: the
 * period it sends at, and the key that the server's answer carries when the
 * host is wanted. The rest of an AT command is the firmware's own business.
 */
static void atbm_iot_at(AtbmCore *c, const char *cmd)
{
    AtbmIot *iot = &c->iot_state;
    const char *p;

    if (!g_str_has_prefix(cmd, "AT+WIFI_HEART_PKT")) {
        return;
    }
    p = strstr(cmd, "WAKEUPKEY \"");
    if (p) {
        const char *e = strchr(p + 11, '"');

        g_strlcpy(iot->wake_key, p + 11,
                  MIN(sizeof(iot->wake_key), e ? e - (p + 11) + 1 : 1));
    }
    p = strstr(cmd, "PERIOD ");
    if (p && !iot->alive_period) {
        iot->alive_period = g_ascii_strtoull(p + 7, NULL, 10);
        iot->alive_tmo = 3;
        if (iot->alive_period) {
            timer_mod(iot->alive, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                                  iot->alive_period * 1000);
        }
    }
}

void atbm_iot_net_rx(AtbmCore *c, const uint8_t *frame, size_t len)
{
    const char *key = c->iot_state.wake_key;
    size_t hdr = WIFI_HDR_LEN;

    if (len < hdr || WIFI_FC_TYPE(lduw_le_p(frame)) != WIFI_TYPE_DATA ||
        (WIFI_FC_STYPE(lduw_le_p(frame)) & WIFI_STYPE_NODATA)) {
        return;
    }
    if (WIFI_FC_STYPE(lduw_le_p(frame)) & WIFI_STYPE_QOS) {
        hdr += 2;
    }
    if (!*key ||
        (len > hdr && memmem(frame + hdr, len - hdr, key, strlen(key)))) {
        atbm_iot_wake(c, ATBM_WAKEUP_NETWORK);
    }
}

static bool atbm_iot_key_ok(AtbmCore *c)
{
    AtbmIot *iot = &c->iot_state;

    if (!c->ap.psk) {
        return iot->keymgmt == KEY_MGMT_NONE;
    }
    return iot->keymgmt != KEY_MGMT_NONE &&
           iot->key_len == strlen(c->ap.psk) &&
           !memcmp(iot->key, c->ap.psk, iot->key_len);
}

/*
 * The join runs in the firmware: the SSID must be the access point's and
 * the key its passphrase. What the host sees is an event either way. The
 * handshake itself stays inside, with the keys: from here on the chip
 * encrypts what the host sends and decrypts what arrives.
 */
static void atbm_iot_joined(void *opaque)
{
    AtbmCore *c = opaque;
    AtbmIot *iot = &c->iot_state;
    uint8_t e[CONN_EVENT_SIZE] = {};
    bool ssid_ok = c->ap.ssid && iot->ssid_len == strlen(c->ap.ssid) &&
                   !memcmp(iot->ssid, c->ap.ssid, iot->ssid_len);
    uint8_t why = CONNECT_NOT_FOUND_SSID;

    if (ssid_ok && atbm_iot_key_ok(c)) {
        wifi_ap_admit(&c->ap, c->conf.macaddr.a);
        c->ptk_set = true;
        c->gtk_set = true;
        c->ptk_pn = 0;
        c->gtk_pn = 0;
        c->join_if = 0;
        iot->connected = true;
        atbm_iot_conn_event(c, e);
        atbm_iot_event(c, EVENT_BSS_CONNECT, e, sizeof(e));
        return;
    }
    if (ssid_ok) {
        why = CONNECT_JOIN_FAIL;
    }
    atbm_iot_event(c, EVENT_HOST_CONN_ERR, &why, 1);
}

static void atbm_iot_join(AtbmCore *c, const uint8_t *b, size_t len)
{
    AtbmIot *iot = &c->iot_state;

    if (len < JOIN_SIZE) {
        return;
    }
    wifi_ap_reset(&c->ap);
    iot->connected = false;
    iot->ssid_len = MIN(b[JOIN_SSID_LEN], sizeof(iot->ssid));
    memcpy(iot->ssid, b + JOIN_SSID, iot->ssid_len);
    iot->keymgmt = b[JOIN_KEYMGMT];
    iot->key_len = MIN(b[JOIN_KEY_LEN], sizeof(iot->key));
    memcpy(iot->key, b + JOIN_KEY, iot->key_len);
    timer_mod(iot->join, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + JOIN_MS);
}

static void atbm_iot_scan_done(void *opaque)
{
    AtbmCore *c = opaque;

    c->iot_state.scanning = false;
}

/* struct wsm_scan_result_req: status, count, remaining, offset, entries */
static void atbm_iot_scan_info(AtbmCore *c, uint8_t *r)
{
    uint8_t *i = r + SCAN_INFO_HDR;

    if (c->iot_state.scanning || !c->ap.ssid) {
        stl_le_p(r, c->iot_state.scanning ? 1 : 0);
        return;
    }
    stw_le_p(r + 4, 1);
    pstrcpy((char *)i, 33, c->ap.ssid);
    memcpy(i + 33, c->ap.bssid.a, 6);
    i[39] = strlen(c->ap.ssid);
    i[40] = c->ap.channel;
    i[41] = c->ap.psk ? SCAN_FLAG_RSN | SCAN_FLAG_ENCRYPT : 0;
    i[42] = -40;
}

/* struct wsm_sdio_getconfig_cnf: the MAC, the mode, and the link if up */
static void atbm_iot_config(AtbmCore *c, uint8_t *r)
{
    memcpy(r + 4, c->conf.macaddr.a, 6);
    r[10] = c->iot_state.connected;
    r[16] = 16;
    stl_le_p(r + 20, XMITED_ADDR);
    stl_le_p(r + 24, IOT_FW_VERSION);
    if (c->iot_state.connected) {
        atbm_iot_conn_event(c, r + CFG_CONNECT_EVENT);
        memcpy(r + CFG_BSS + 3, c->ap.bssid.a, 6);
        stw_le_p(r + CFG_BSS, c->ap.channel);
    }
}

void atbm_iot_wsm(AtbmCore *c, uint16_t id, const uint8_t *body, size_t len)
{
    AtbmIot *iot = &c->iot_state;
    uint16_t cnf = (id & 0x03ff) | 0x0400;
    /*
     * The host sized its buffer for the answer: a get request is only that
     * size, a set request carries a struct of it
     */
    size_t n = MAX(len == 2 ? lduw_le_p(body) : len, 4);
    g_autofree uint8_t *r = g_malloc0(n);

    /* a host that talks is awake */
    iot->asleep = false;
    if (c->mcu) {
        qemu_set_irq(c->wake, 0);
    }
    switch (id & 0x3f) {
    case IOT_JOIN_REQ:
        atbm_iot_join(c, body, len);
        break;
    case IOT_SCAN_REQ:
        iot->scanning = true;
        timer_mod(iot->scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + SCAN_MS);
        break;
    case IOT_SCAN_STATE_REQ:
        r[4] = iot->scanning;
        break;
    case IOT_SCAN_INFO_REQ:
        if (n >= SCAN_INFO_HDR + SCAN_INFO_SIZE * SCAN_INFO_MAX) {
            atbm_iot_scan_info(c, r);
        }
        break;
    case IOT_CONFIG_REQ:
        if (n >= CFG_BSS) {
            atbm_iot_config(c, r);
        }
        break;
    case IOT_PSMODE_REQ:
        iot->ps_mode = len > 8 ? body[8] : 0;
        iot->asleep = iot->ps_mode >= PS_LIGHT_SLEEP;
        break;
    case IOT_AT_CMD_REQ:
        if (len > 4) {
            g_autofree char *cmd = g_strndup((char *)body + 4,
                                             MIN(len - 4, 255));

            atbm_iot_at(c, cmd);
        }
        break;
    case IOT_FW_SLEEP_REQ:
        iot->asleep = true;
        break;
    case IOT_WAKEUP_EVENT_REQ:
        iot->wake_mask = len >= 8 ? ldl_le_p(body + 4) : 0;
        break;
    case IOT_GET_VER_REQ:
        pstrcpy((char *)r + 4, 64, "ATBM6x41");
        break;
    case IOT_GET_SDK_VER_REQ:
        pstrcpy((char *)r + 4 + 64, 64, IOT_SDK_VERSION);
        break;
    case IOT_RSSI_REQ:
        stl_le_p(r + 4, -40);
        break;
    case IOT_CHECK_ALIVE_REQ:
        if (n >= 8) {
            stl_le_p(r + 4, ALIVE_FROM_FW);
        }
        if (len >= 16) {
            iot->alive_period = ldl_le_p(body + 8);
            iot->alive_tmo = ldl_le_p(body + 12);
            iot->alive_missed = 0;
            if (iot->alive_period) {
                timer_mod(iot->alive, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                                      iot->alive_period * 1000);
            } else {
                timer_del(iot->alive);
            }
        }
        break;
    }
    atbm_core_send(c, cnf, r, n);
}

void atbm_iot_reset(AtbmCore *c)
{
    AtbmIot *iot = &c->iot_state;

    timer_del(iot->scan);
    timer_del(iot->join);
    timer_del(iot->alive);
    iot->scanning = false;
    iot->connected = false;
    iot->ps_mode = 0;
    iot->asleep = false;
    iot->wake_mask = ~0u;
    iot->alive_period = 0;
}

void atbm_iot_init(AtbmCore *c)
{
    c->iot_state.scan = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                     atbm_iot_scan_done, c);
    c->iot_state.join = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_iot_joined, c);
    c->iot_state.alive = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_iot_alive, c);
}

void atbm_iot_finalize(AtbmCore *c)
{
    timer_free(c->iot_state.scan);
    timer_free(c->iot_state.join);
    timer_free(c->iot_state.alive);
}
