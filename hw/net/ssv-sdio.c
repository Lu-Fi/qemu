/*
 * iComm-semi SSV6158 Wi-Fi on SDIO
 *
 * Function 1 has a page of byte registers for CMD52, two of which name the
 * ports CMD53 goes through: the register port, where a 4-byte address
 * then a read, or the address and a value, reach one chip register; and
 * the data port, which carries frames and host commands to the chip,
 * frames and events back, and with download mode on the firmware into
 * SRAM. Both ways can aggregate several messages in a transfer, as
 * HCI_TRX_MODE says. The chip behind them is ssv-wifi.c.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/core/qdev-properties.h"
#include "hw/sd/sdio.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "ssv-wifi.h"
#include "trace.h"

#define TYPE_SSV_SDIO "ssv6158"
OBJECT_DECLARE_SIMPLE_TYPE(SsvSdioState, SSV_SDIO)

/* the byte registers, and where the driver finds the two ports */
#define LOCAL_SIZE              0x100
#define REG_DATA_IO_PORT        0x00
#define REG_INT_MASK            0x04
#define REG_INT_STATUS          0x08
#define REG_FN1_STATUS          0x0c
#define REG_CARD_PKT_LEN        0x10
#define REG_REG_IO_PORT         0x70
#define INT_RX                  BIT(0)
#define FN1_SRAM_DOWNLOAD       BIT(1)

#define DATA_PORT               0x10000
#define REG_PORT                0x10020

/*
 * The byte registers again in the chip's map, where the driver reads the
 * RX length through the register port and sets the SRAM download address
 */
#define SD_REG_BASE             0xc0000800
#define SD_REG_PKT_LEN          (SD_REG_BASE + REG_CARD_PKT_LEN)
#define SD_REG_DMA_ADDR         (SD_REG_BASE + 0x60)

/* HCI_TRX_MODE: TX aggregation, and the aggregated RX format */
#define REG_HCI_TRX_MODE        0xc1000004
#define HCI_TX_AGG_EN           BIT(0)
#define HCI_RX_FORM_1           BIT(30)

/*
 * An RX aggregate: units of a 12-byte header (this unit's length, the
 * aggregate's in the top half; the TX resources and EDCA counts after it
 * left at 0) and a message, ended by a zero word. A TX aggregate's units
 * have an 8-byte header, the length twice then a zero word.
 */
#define RX_AGGR_HDR_LEN         12
#define TX_AGGR_HDR_LEN         8

#define BLOCK_SIZE              512

struct SsvSdioState {
    SDIOCard parent_obj;

    SsvCore core;
    uint8_t local[LOCAL_SIZE];
    uint32_t reg_addr;              /* latched by a 4-byte register write */
    GByteArray *rx;                 /* the transfer the host is reading */
    size_t rx_off;
};

static void ssv_sdio_update(SsvSdioState *s)
{
    if (s->rx || ssv_core_peek(&s->core)) {
        s->local[REG_INT_STATUS] |= INT_RX;
    } else {
        s->local[REG_INT_STATUS] &= ~INT_RX;
    }
    sdio_card_set_irq(SDIO_CARD(s), 1,
                      s->local[REG_INT_STATUS] & ~s->local[REG_INT_MASK]);
}

static void ssv_sdio_notify(SsvCore *c)
{
    ssv_sdio_update(container_of(c, SsvSdioState, core));
}

static bool ssv_sdio_rx_aggr(SsvSdioState *s)
{
    return ssv_core_reg_read(&s->core, REG_HCI_TRX_MODE) & HCI_RX_FORM_1;
}

/* The next transfer for the host: a message, or one in an aggregate */
static void ssv_sdio_next(SsvSdioState *s)
{
    GByteArray *m = ssv_core_pop(&s->core);
    uint8_t h[RX_AGGR_HDR_LEN] = {};
    uint32_t unit;

    if (!m) {
        return;
    }
    s->rx_off = 0;
    if (!ssv_sdio_rx_aggr(s)) {
        s->rx = m;
        return;
    }
    unit = ROUND_UP(RX_AGGR_HDR_LEN + m->len, 4);
    stl_le_p(h, unit | (unit + 4) << 16);
    s->rx = g_byte_array_sized_new(unit + 4);
    g_byte_array_append(s->rx, h, sizeof(h));
    g_byte_array_append(s->rx, m->data, m->len);
    g_byte_array_set_size(s->rx, unit + 4);
    memset(s->rx->data + RX_AGGR_HDR_LEN + m->len, 0,
           unit + 4 - RX_AGGR_HDR_LEN - m->len);
    g_byte_array_unref(m);
}

/*
 * The data port read: 4 bytes ahead of an aggregate only look at its
 * header, for the length; otherwise the transfer, then zeros
 */
static void ssv_sdio_rx_read(SsvSdioState *s, uint8_t *buf, size_t len)
{
    size_t n = 0;

    if (!s->rx) {
        ssv_sdio_next(s);
    }
    if (s->rx && len == 4 && !s->rx_off && ssv_sdio_rx_aggr(s)) {
        memcpy(buf, s->rx->data, 4);
        return;
    }
    if (s->rx) {
        n = MIN(len, s->rx->len - s->rx_off);
        memcpy(buf, s->rx->data + s->rx_off, n);
        s->rx_off += n;
        if (s->rx_off == s->rx->len) {
            g_clear_pointer(&s->rx, g_byte_array_unref);
        }
    }
    memset(buf + n, 0, len - n);
    ssv_sdio_update(s);
}

/* What the host sends: messages one by one, or aggregated */
static void ssv_sdio_tx_write(SsvSdioState *s, const uint8_t *buf,
                              size_t len)
{
    size_t p = 0;

    if (!(ssv_core_reg_read(&s->core, REG_HCI_TRX_MODE) & HCI_TX_AGG_EN)) {
        ssv_core_host(&s->core, buf, len);
        return;
    }
    while (p + TX_AGGR_HDR_LEN <= len) {
        size_t unit = ldl_le_p(buf + p) & 0xffff;

        if (unit <= TX_AGGR_HDR_LEN || p + unit > len) {
            break;
        }
        ssv_core_host(&s->core, buf + p + TX_AGGR_HDR_LEN,
                      unit - TX_AGGR_HDR_LEN);
        p += unit;
    }
}

/* A chip register through the register port, or one of the byte ones */
static uint32_t ssv_sdio_reg_read(SsvSdioState *s, uint32_t addr)
{
    if (addr == SD_REG_PKT_LEN) {
        if (!s->rx) {
            ssv_sdio_next(s);
        }
        return s->rx ? s->rx->len - s->rx_off : 0;
    }
    if (addr >= SD_REG_BASE && addr < SD_REG_BASE + LOCAL_SIZE &&
        addr != SD_REG_DMA_ADDR) {
        return ldl_le_p(s->local + (addr - SD_REG_BASE) / 4 * 4);
    }
    return ssv_core_reg_read(&s->core, addr);
}

static void ssv_sdio_read(SDIOCard *card, unsigned fn, uint32_t addr,
                          bool incr, uint8_t *buf, size_t len)
{
    SsvSdioState *s = SSV_SDIO(card);
    uint32_t v;

    if (addr == DATA_PORT) {
        ssv_sdio_rx_read(s, buf, len);
        return;
    }
    if (addr == REG_PORT) {
        v = ssv_sdio_reg_read(s, s->reg_addr);
        trace_ssv_sdio_reg_read(s->reg_addr, v);
        memset(buf, 0, len);
        stl_le_p(buf, v);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        uint32_t a = addr + (incr ? i : 0);

        buf[i] = a < LOCAL_SIZE ? s->local[a] : 0;
    }
}

static void ssv_sdio_write(SDIOCard *card, unsigned fn, uint32_t addr,
                           bool incr, const uint8_t *buf, size_t len)
{
    SsvSdioState *s = SSV_SDIO(card);
    uint32_t v;

    if (addr == DATA_PORT) {
        if (s->local[REG_FN1_STATUS] & FN1_SRAM_DOWNLOAD) {
            v = ssv_core_reg_read(&s->core, SD_REG_DMA_ADDR);
            ssv_core_fw_write(&s->core, v, buf, len);
            ssv_core_reg_write(&s->core, SD_REG_DMA_ADDR, v + len);
        } else {
            ssv_sdio_tx_write(s, buf, len);
        }
        return;
    }
    if (addr == REG_PORT) {
        if (len < 4) {
            return;
        }
        s->reg_addr = ldl_le_p(buf);
        if (len >= 8) {
            v = ldl_le_p(buf + 4);
            trace_ssv_sdio_reg_write(s->reg_addr, v);
            ssv_core_reg_write(&s->core, s->reg_addr, v);
        }
        return;
    }
    for (size_t i = 0; i < len; i++) {
        uint32_t a = addr + (incr ? i : 0);

        /* the status and the ports' addresses are the chip's to say */
        if (a >= LOCAL_SIZE || a == REG_INT_STATUS ||
            a - REG_DATA_IO_PORT < 3 || a - REG_REG_IO_PORT < 3) {
            continue;
        }
        s->local[a] = buf[i];
    }
    ssv_sdio_update(s);
}

static void ssv_sdio_local_reset(SsvSdioState *s)
{
    memset(s->local, 0, sizeof(s->local));
    s->local[REG_DATA_IO_PORT] = DATA_PORT & 0xff;
    s->local[REG_DATA_IO_PORT + 1] = (DATA_PORT >> 8) & 0xff;
    s->local[REG_DATA_IO_PORT + 2] = DATA_PORT >> 16;
    s->local[REG_REG_IO_PORT] = REG_PORT & 0xff;
    s->local[REG_REG_IO_PORT + 1] = (REG_PORT >> 8) & 0xff;
    s->local[REG_REG_IO_PORT + 2] = REG_PORT >> 16;
    /* every interrupt masked until the driver asks for one */
    s->local[REG_INT_MASK] = 0xff;
    s->reg_addr = 0;
    g_clear_pointer(&s->rx, g_byte_array_unref);
}

static void ssv_sdio_realize(DeviceState *dev, Error **errp)
{
    SsvSdioState *s = SSV_SDIO(dev);

    ssv_sdio_local_reset(s);
    ssv_core_realize(&s->core, errp);
}

static void ssv_sdio_unrealize(DeviceState *dev)
{
    ssv_core_unrealize(&SSV_SDIO(dev)->core);
}

static void ssv_sdio_init(Object *obj)
{
    SsvSdioState *s = SSV_SDIO(obj);

    s->core.dev = DEVICE(obj);
    s->core.notify = ssv_sdio_notify;
    s->core.chip = SSV_CHIP_6020;
    s->core.efuse_id = 0;
    ssv_core_init(&s->core);
}

static void ssv_sdio_finalize(Object *obj)
{
    SsvSdioState *s = SSV_SDIO(obj);

    g_clear_pointer(&s->rx, g_byte_array_unref);
    ssv_core_finalize(&s->core);
}

static const Property ssv_sdio_props[] = {
    DEFINE_SSV_PROPERTIES(SsvSdioState, core),
};

/* the registers, the queued messages and the firmware are not migrated */
static const VMStateDescription ssv_sdio_vmstate = {
    .name = TYPE_SSV_SDIO,
    .unmigratable = 1,
};

static void ssv_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = ssv_sdio_realize;
    dc->unrealize = ssv_sdio_unrealize;
    dc->vmsd = &ssv_sdio_vmstate;
    dc->desc = "iComm-semi SSV6158 SDIO Wi-Fi";
    device_class_set_props(dc, ssv_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x5653;
    sc->device = 0x2060;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = BLOCK_SIZE;
    sc->read = ssv_sdio_read;
    sc->write = ssv_sdio_write;
}

static const TypeInfo ssv_sdio_info = {
    .name           = TYPE_SSV_SDIO,
    .parent         = TYPE_SDIO_CARD,
    .instance_size  = sizeof(SsvSdioState),
    .instance_init  = ssv_sdio_init,
    .instance_finalize = ssv_sdio_finalize,
    .class_init     = ssv_sdio_class_init,
};

static void ssv_sdio_register_types(void)
{
    type_register_static(&ssv_sdio_info);
}

type_init(ssv_sdio_register_types)
