/*
 * AltoBeam ATBM6461: the RTOS beside the Wi-Fi firmware
 *
 * The 6461 is a Hera Wi-Fi part with a low-power core in it, which a
 * battery camera leaves running while the SoC sleeps. The SoC reaches it
 * through the driver's general command (a WSM request, id 0x38, carrying
 * 532 bytes the driver does not look into) and the librtos library of the
 * camera's firmware wraps that in rtos_cmd_send(). A request is a magic,
 * the command id, a CRC of the payload, the payload's length and the
 * payload; the answer is the magic, the id, a CRC, a return code, the
 * payload's length and the payload. The CRC is the zlib one without its
 * final inversion. The command ids and what they carry are read from the
 * camera's mcu_test and ajcloud programs; what the real core answers to
 * each, and what its return codes mean, is assumed (zero for success).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include <zlib.h>
#include "atbm-wifi.h"
#include "trace.h"

#define RTOS_MAGIC              0xacaccacaU
#define REQ_ID                  4
#define REQ_CRC                 8
#define REQ_LEN                 12
#define REQ_DATA                16
#define RESP_RET                12
#define RESP_LEN                16
#define RESP_DATA               20

#define CMD_CONNECT             0x02
#define CMD_NET_INFO            0x03
#define CMD_PIR_ENABLE          0x09
#define CMD_PIR_DISABLE         0x0a
#define CMD_SET_DETECT_RANGE    0x0d
#define CMD_WDT_ENABLE          0x12
#define CMD_WDT_DISABLE         0x13
#define CMD_SET_UPGRADE_ITV     0x18
#define CMD_GET_BATTERY         0x1a
#define CMD_GET_VOLTAGE         0x1b
#define CMD_GET_VERSION         0x24
#define CMD_SET_FW_DATE         0x2e
#define CMD_GET_FW_DATE         0x2f
#define CMD_FACTORY_RESET       0x34
#define CMD_SET_RTC_MODE        0x3b
#define CMD_GET_INC_INTERVAL    0x44
#define CMD_SET_INC_INTERVAL    0x45
#define CMD_BLE_START           0x46
#define CMD_BLE_STOP            0x47
#define CMD_SET_PIR_TYPE        0x51

/* wifi_connect: a flag, the SSID in 36 bytes, the key in 68 */
#define CONNECT_SSID            4
#define CONNECT_SSID_MAX        36
#define CONNECT_KEY             40
#define CONNECT_KEY_MAX         68
/* assumed: the join request's key management for a WPA2 key */
#define KEY_MGMT_WPA2           2

#define MCU_VERSION             "qemu"

static uint32_t rtos_crc(const uint8_t *p, size_t len)
{
    return ~crc32(0, p, len);
}

/* The payload of an answer, copied in as far as the 532 bytes allow */
static size_t rtos_reply(uint8_t *resp, size_t resp_len, const void *data,
                         size_t len)
{
    len = MIN(len, resp_len - RESP_DATA);
    memcpy(resp + RESP_DATA, data, len);
    return len;
}

void atbm_rtos_cmd(AtbmCore *c, const uint8_t *req, size_t len,
                   uint8_t *resp, size_t resp_len)
{
    AtbmRtos *r = &c->rtos_state;
    uint32_t id, plen, val = 0;
    const uint8_t *p = req + REQ_DATA;
    size_t out = 0;
    uint32_t word[3];

    if (len < REQ_DATA || resp_len < RESP_DATA ||
        ldl_le_p(req) != RTOS_MAGIC) {
        return;
    }
    id = ldl_le_p(req + REQ_ID);
    plen = MIN(ldl_le_p(req + REQ_LEN), len - REQ_DATA);
    if (plen >= 4) {
        val = ldl_le_p(p);
    }
    trace_atbm_rtos_cmd(id, plen, val,
                        ldl_le_p(req + REQ_CRC) == rtos_crc(p, plen));

    switch (id) {
    case CMD_CONNECT:
        if (plen >= CONNECT_KEY + 1) {
            const uint8_t *ssid = p + CONNECT_SSID;
            const uint8_t *key = p + CONNECT_KEY;
            size_t slen = strnlen((const char *)ssid, CONNECT_SSID_MAX);
            size_t klen = MIN(strnlen((const char *)key, CONNECT_KEY_MAX),
                              plen - CONNECT_KEY);

            atbm_iot_connect(c, ssid, slen, klen ? KEY_MGMT_WPA2 : 0,
                             key, klen);
        }
        break;
    case CMD_PIR_ENABLE:
    case CMD_PIR_DISABLE:
        r->pir = id == CMD_PIR_ENABLE;
        break;
    case CMD_SET_DETECT_RANGE:
        r->detect_range = val;
        break;
    case CMD_WDT_ENABLE:
    case CMD_WDT_DISABLE:
        r->wdt = id == CMD_WDT_ENABLE;
        break;
    case CMD_SET_UPGRADE_ITV:
        r->upgrade_interval = val;
        break;
    case CMD_GET_BATTERY:
        /* the capacity, the voltage and the charge status, in that order */
        stl_le_p(&word[0], c->battery);
        stl_le_p(&word[1], c->battery_mv);
        stl_le_p(&word[2], c->charging);
        out = rtos_reply(resp, resp_len, word, sizeof(word));
        break;
    case CMD_GET_VOLTAGE:
        stl_le_p(&word[0], c->battery_mv);
        out = rtos_reply(resp, resp_len, word, 4);
        break;
    case CMD_GET_VERSION:
        out = rtos_reply(resp, resp_len, MCU_VERSION, sizeof(MCU_VERSION));
        break;
    case CMD_SET_FW_DATE:
        pstrcpy(r->fw_date, sizeof(r->fw_date),
                strnlen((const char *)p, plen) < plen ? (const char *)p : "");
        break;
    case CMD_GET_FW_DATE:
        out = rtos_reply(resp, resp_len, r->fw_date, strlen(r->fw_date) + 1);
        break;
    case CMD_SET_RTC_MODE:
        r->rtc_mode = val;
        break;
    case CMD_GET_INC_INTERVAL:
        stl_le_p(&word[0], r->inc_interval);
        out = rtos_reply(resp, resp_len, word, 4);
        break;
    case CMD_SET_INC_INTERVAL:
        r->inc_interval = val;
        break;
    case CMD_BLE_START:
    case CMD_BLE_STOP:
        r->ble = id == CMD_BLE_START;
        break;
    case CMD_SET_PIR_TYPE:
        r->pir_type = val;
        break;
    case CMD_NET_INFO:
    case CMD_FACTORY_RESET:
    default:
        break;
    }

    trace_atbm_rtos_reply(id, out, ldl_le_p(resp + RESP_DATA));
    stl_le_p(resp, RTOS_MAGIC);
    stl_le_p(resp + REQ_ID, id);
    stl_le_p(resp + REQ_CRC, rtos_crc(resp + RESP_DATA, out));
    stl_le_p(resp + RESP_RET, 0);
    stl_le_p(resp + RESP_LEN, out);
}

const VMStateDescription vmstate_atbm_rtos = {
    .name = "atbm-rtos",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(upgrade_interval, AtbmRtos),
        VMSTATE_UINT32(detect_range, AtbmRtos),
        VMSTATE_UINT32(inc_interval, AtbmRtos),
        VMSTATE_UINT32(pir_type, AtbmRtos),
        VMSTATE_UINT32(rtc_mode, AtbmRtos),
        VMSTATE_BOOL(pir, AtbmRtos),
        VMSTATE_BOOL(wdt, AtbmRtos),
        VMSTATE_BOOL(ble, AtbmRtos),
        VMSTATE_BUFFER_UNSAFE(fw_date, AtbmRtos, 0, 16),
        VMSTATE_END_OF_LIST()
    },
};
