/*
 * AltoBeam ATBM6441: the MCU's side of a battery camera
 *
 * The 6441 is a Wi-Fi chip with a microcontroller in it, and a battery
 * camera leaves it running while the SoC sleeps. What the SoC sees of that
 * side, besides the SDIO function: a UART the MCU reports on and takes
 * requests over, a line it wakes the SoC with, and the lines in (the PIR
 * sensor, the USB charger). The UART frames are 0x7b, the frame's length,
 * a type, the payload and a checksum, the complement of the XOR of all
 * before it. The reasons it wakes the SoC for, and the keep-alive it keeps
 * up for it, are the IoT firmware's (atbm-iot.c).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "hw/core/qdev.h"
#include "atbm-wifi.h"
#include "trace.h"

#define FRAME_START             0x7b
#define FRAME_MIN               4
#define FRAME_MAX               132

#define TYPE_PIR                0x03
#define TYPE_CHARGER_IN         0x04
#define TYPE_CHARGER_OUT        0x05
#define TYPE_POWER_INFO         0x45

/* the reply to a power info request: how it started, the battery, the rest */
#define POWER_INFO_LEN          12

/* the PIR frame repeats for as long as the sensor sees motion */
#define PIR_REPEAT_MS           200

static uint8_t atbm_mcu_sum(const uint8_t *f, size_t len)
{
    uint8_t x = 0;

    for (size_t i = 0; i < len; i++) {
        x ^= f[i];
    }
    return ~x;
}

static void atbm_mcu_send(AtbmCore *c, uint8_t type, const uint8_t *payload,
                          size_t len)
{
    uint8_t f[FRAME_MAX];

    f[0] = FRAME_START;
    f[1] = FRAME_MIN + len;
    f[2] = type;
    memcpy(f + 3, payload, len);
    f[3 + len] = atbm_mcu_sum(f, 3 + len);
    trace_atbm_mcu_tx(type, len);
    qemu_chr_fe_write_all(&c->uart, f, FRAME_MIN + len);
}

static void atbm_mcu_power_info(AtbmCore *c)
{
    uint8_t p[POWER_INFO_LEN] = {};

    p[1] = c->battery;
    p[2] = c->charging;
    stw_le_p(p + 4, c->battery_mv);
    atbm_mcu_send(c, TYPE_POWER_INFO, p, sizeof(p));
}

static void atbm_mcu_frame(AtbmCore *c, const uint8_t *f, size_t len)
{
    trace_atbm_mcu_rx(f[2]);
    if (f[2] == TYPE_POWER_INFO) {
        atbm_mcu_power_info(c);
    }
}

static int atbm_mcu_can_receive(void *opaque)
{
    return FRAME_MAX;
}

/* Frames from the SoC, found by their start byte and their length */
static void atbm_mcu_receive(void *opaque, const uint8_t *buf, int size)
{
    AtbmCore *c = opaque;

    for (int i = 0; i < size; i++) {
        uint8_t *f = c->uart_frame;

        if (!c->uart_len && buf[i] != FRAME_START) {
            continue;
        }
        f[c->uart_len++] = buf[i];
        if (c->uart_len == 2 && (f[1] < FRAME_MIN || f[1] > FRAME_MAX)) {
            c->uart_len = 0;
        } else if (c->uart_len >= 2 && c->uart_len == f[1]) {
            if (f[c->uart_len - 1] == atbm_mcu_sum(f, c->uart_len - 1)) {
                atbm_mcu_frame(c, f, c->uart_len);
            }
            c->uart_len = 0;
        }
    }
}

static void atbm_mcu_pir_tick(void *opaque)
{
    AtbmCore *c = opaque;

    atbm_mcu_send(c, TYPE_PIR, NULL, 0);
    timer_mod(c->pir_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                            PIR_REPEAT_MS);
}

/* The PIR line: motion wakes a sleeping SoC and is reported while it lasts */
static void atbm_mcu_pir(void *opaque, int line, int level)
{
    AtbmCore *c = opaque;

    c->pir = level;
    if (level) {
        atbm_iot_wake(c, ATBM_WAKEUP_IO);
        atbm_mcu_pir_tick(c);
    } else {
        timer_del(c->pir_timer);
    }
}

/* The charger line: high with the USB charger plugged in */
static void atbm_mcu_charger(void *opaque, int line, int level)
{
    AtbmCore *c = opaque;

    c->charging = level;
    atbm_mcu_send(c, level ? TYPE_CHARGER_IN : TYPE_CHARGER_OUT, NULL, 0);
}

void atbm_mcu_init(AtbmCore *c)
{
    DeviceState *dev = c->dev;

    c->mcu = true;
    qdev_init_gpio_out_named(dev, &c->wake, "wake", 1);
    qdev_init_gpio_in_named_with_opaque(dev, atbm_mcu_pir, c, "pir", 1);
    qdev_init_gpio_in_named_with_opaque(dev, atbm_mcu_charger, c, "charger",
                                        1);
    c->pir_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_mcu_pir_tick, c);
}

void atbm_mcu_realize(AtbmCore *c)
{
    qemu_chr_fe_set_handlers(&c->uart, atbm_mcu_can_receive,
                             atbm_mcu_receive, NULL, NULL, c, NULL, true);
}

void atbm_mcu_unrealize(AtbmCore *c)
{
    qemu_chr_fe_deinit(&c->uart, false);
}

void atbm_mcu_finalize(AtbmCore *c)
{
    timer_free(c->pir_timer);
}
