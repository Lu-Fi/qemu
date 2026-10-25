/*
 * AICSemi AIC8800D80 Wi-Fi on SDIO
 *
 * Two functions of the WLAN class answer as C8A1:0082: aic8800_bsp probes
 * the second, loads the firmware through the first and waits on its start,
 * then aic8800_fdrv takes the first over. Function 1 has a page of byte
 * registers: 0x00 the interrupt enable, 0x01 the pending ones (bit 4: the
 * chip is awake), 0x02 for the chip (bit 0 wakes it and reads back clear
 * once it is), 0x03 the 1536-byte buffers the host may fill, 0x04 the
 * 512-byte blocks waiting for the host, 0x07 byte mode (1: off), and the
 * read and write FIFOs at 0x0f and 0x10.
 *
 * Both ways a transfer is whole blocks of packets, each behind its word of
 * length, type and a CRC-8 of the three, aligned to a word, the last one
 * followed by a zero word. The host gathers frames several to a transfer,
 * a message goes alone; the chip sends a packet a transfer. A frame's
 * length counts what follows its word, where a USB frame's counts the word
 * too. The chip behind them is aic-wifi.c.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/resettable.h"
#include "hw/sd/sdio.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "aic-wifi.h"
#include "trace.h"

#define TYPE_AIC_SDIO "aic8800d80-sdio"
OBJECT_DECLARE_TYPE(AicSdioState, AicSdioClass, AIC_SDIO)

#define REG_INT_ENABLE          0x00
#define REG_INT_PENDING         0x01
#define REG_TO_DEVICE           0x02
#define REG_FLOW_CTRL           0x03
#define REG_BLOCKS              0x04
#define REG_RD_FIFO             0x0f
#define REG_WR_FIFO             0x10
#define LOCAL_SIZE              0x20

#define INT_PENDING_AWAKE       BIT(4)
#define TO_DEVICE_WAKE          BIT(0)

/* every buffer free: a packet is taken in as it comes */
#define FREE_BUFFERS            32

#define BLOCK_SIZE              512
#define PKT_HDR_LEN             4
#define TYPE_DATA               0x01

#define REG_CHIP_ID             0x40500000

/* an 8800D80 of the H variant, U02, as on USB */
static const AicWord aic_sdio_words[] = {
    { REG_CHIP_ID, 0x02c30000 },
    { }
};

struct AicSdioClass {
    SDIOCardClass parent_class;
    ResettablePhases parent_phases;
};

struct AicSdioState {
    SDIOCard parent_obj;

    AicCore core;
    uint8_t local[LOCAL_SIZE];
    uint8_t vendor_cccr[16];
    GByteArray *rx;                 /* the transfer the host is reading */
    size_t rx_off;
};

static void aic_sdio_update(AicSdioState *s)
{
    AicCore *c = &s->core;
    bool waiting = s->rx || !g_queue_is_empty(&c->msgs) ||
                   !g_queue_is_empty(&c->frames);

    sdio_card_set_irq(SDIO_CARD(s), 1, waiting && s->local[REG_INT_ENABLE]);
}

static void aic_sdio_notify(AicCore *c)
{
    aic_sdio_update(container_of(c, AicSdioState, core));
}

/* The chip starts its firmware or its ROM again, still on the bus */
static void aic_sdio_replug(AicCore *c)
{
    AicSdioState *s = container_of(c, AicSdioState, core);

    g_clear_pointer(&s->rx, g_byte_array_unref);
    aic_sdio_update(s);
}

/* The next transfer for the host: a packet, then a zero word, in blocks */
static void aic_sdio_next(AicSdioState *s)
{
    GByteArray *p;
    size_t len;

    if (s->rx) {
        return;
    }
    p = aic_core_pop_msg(&s->core) ?: aic_core_pop_frame(&s->core);
    if (!p) {
        return;
    }
    len = p->len;
    g_byte_array_set_size(p, ROUND_UP(ROUND_UP(len, 4) + 4, BLOCK_SIZE));
    memset(p->data + len, 0, p->len - len);
    trace_aic_sdio_rx(len, p->len / BLOCK_SIZE);
    s->rx = p;
    s->rx_off = 0;
}

static void aic_sdio_rx_read(AicSdioState *s, uint8_t *buf, size_t len)
{
    size_t n = 0;

    aic_sdio_next(s);
    if (s->rx) {
        n = MIN(len, s->rx->len - s->rx_off);
        memcpy(buf, s->rx->data + s->rx_off, n);
        s->rx_off += n;
        if (s->rx_off == s->rx->len) {
            g_clear_pointer(&s->rx, g_byte_array_unref);
        }
    }
    memset(buf + n, 0, len - n);
    aic_sdio_update(s);
}

/* What the host sends: its packets, up to the zero word */
static void aic_sdio_tx_write(AicSdioState *s, const uint8_t *buf, size_t len)
{
    size_t p = 0;

    trace_aic_sdio_tx(len);
    while (p + PKT_HDR_LEN <= len) {
        size_t n = lduw_le_p(buf + p) & 0xfff;
        size_t plen = MIN(PKT_HDR_LEN + n, len - p);

        if (!n) {
            break;
        }
        if (buf[p + 2] == TYPE_DATA) {
            g_autofree uint8_t *f = g_memdup2(buf + p, plen);

            stw_le_p(f, plen);
            aic_core_host(&s->core, f, plen);
        } else {
            aic_core_host(&s->core, buf + p, plen);
        }
        p += ROUND_UP(PKT_HDR_LEN + n, 4);
    }
}

static uint8_t aic_sdio_reg_read(AicSdioState *s, uint32_t addr)
{
    switch (addr) {
    case REG_FLOW_CTRL:
        return FREE_BUFFERS;
    case REG_BLOCKS:
        aic_sdio_next(s);
        return s->rx ? DIV_ROUND_UP(s->rx->len - s->rx_off, BLOCK_SIZE) : 0;
    }
    return addr < LOCAL_SIZE ? s->local[addr] : 0;
}

static void aic_sdio_reg_write(AicSdioState *s, uint32_t addr, uint8_t val)
{
    switch (addr) {
    case REG_TO_DEVICE:
        if (val & TO_DEVICE_WAKE) {
            s->local[REG_INT_PENDING] |= INT_PENDING_AWAKE;
        }
        s->local[addr] = val & ~TO_DEVICE_WAKE;
        return;
    case REG_FLOW_CTRL:
    case REG_BLOCKS:
        return;
    }
    if (addr < LOCAL_SIZE) {
        s->local[addr] = val;
    }
}

static void aic_sdio_read(SDIOCard *card, unsigned fn, uint32_t addr,
                          bool incr, uint8_t *buf, size_t len)
{
    AicSdioState *s = AIC_SDIO(card);

    if (fn != 1) {
        memset(buf, 0, len);
        return;
    }
    if (addr == REG_RD_FIFO) {
        aic_sdio_rx_read(s, buf, len);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = aic_sdio_reg_read(s, addr + (incr ? i : 0));
    }
}

static void aic_sdio_write(SDIOCard *card, unsigned fn, uint32_t addr,
                           bool incr, const uint8_t *buf, size_t len)
{
    AicSdioState *s = AIC_SDIO(card);

    if (fn != 1) {
        return;
    }
    if (addr == REG_WR_FIFO) {
        aic_sdio_tx_write(s, buf, len);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        aic_sdio_reg_write(s, addr + (incr ? i : 0), buf[i]);
    }
    aic_sdio_update(s);
}

static uint8_t aic_sdio_vendor_read(SDIOCard *card, uint32_t addr)
{
    return AIC_SDIO(card)->vendor_cccr[addr & 0xf];
}

static void aic_sdio_vendor_write(SDIOCard *card, uint32_t addr, uint8_t val)
{
    AIC_SDIO(card)->vendor_cccr[addr & 0xf] = val;
}

static void aic_sdio_local_reset(AicSdioState *s)
{
    memset(s->local, 0, sizeof(s->local));
    memset(s->vendor_cccr, 0, sizeof(s->vendor_cccr));
    g_clear_pointer(&s->rx, g_byte_array_unref);
}

static void aic_sdio_io_reset(SDIOCard *card)
{
    AicSdioState *s = AIC_SDIO(card);

    aic_sdio_local_reset(s);
    aic_sdio_update(s);
}

/* Power on: the boot ROM, nothing queued */
static void aic_sdio_reset_hold(Object *obj, ResetType type)
{
    AicSdioClass *ac = AIC_SDIO_GET_CLASS(obj);
    AicSdioState *s = AIC_SDIO(obj);

    if (ac->parent_phases.hold) {
        ac->parent_phases.hold(obj, type);
    }
    aic_sdio_local_reset(s);
    aic_core_reset(&s->core);
}

static void aic_sdio_realize(DeviceState *dev, Error **errp)
{
    AicSdioState *s = AIC_SDIO(dev);

    aic_sdio_local_reset(s);
    aic_core_realize(&s->core, errp);
}

static void aic_sdio_unrealize(DeviceState *dev)
{
    aic_core_unrealize(&AIC_SDIO(dev)->core);
}

static void aic_sdio_init(Object *obj)
{
    AicSdioState *s = AIC_SDIO(obj);

    s->core.dev = DEVICE(obj);
    s->core.rom = true;
    s->core.start_cfm = true;
    s->core.words = aic_sdio_words;
    s->core.notify = aic_sdio_notify;
    s->core.replug = aic_sdio_replug;
    aic_core_init(&s->core);
}

static void aic_sdio_finalize(Object *obj)
{
    AicSdioState *s = AIC_SDIO(obj);

    g_clear_pointer(&s->rx, g_byte_array_unref);
    aic_core_finalize(&s->core);
}

static const Property aic_sdio_props[] = {
    DEFINE_AIC_PROPERTIES(AicSdioState, core),
};

/* the chip's memory, the queued messages and the joins are not migrated */
static const VMStateDescription aic_sdio_vmstate = {
    .name = TYPE_AIC_SDIO,
    .unmigratable = 1,
};

static void aic_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = aic_sdio_realize;
    dc->unrealize = aic_sdio_unrealize;
    resettable_class_set_parent_phases(rc, NULL, aic_sdio_reset_hold, NULL,
                                       &AIC_SDIO_CLASS(oc)->parent_phases);
    dc->vmsd = &aic_sdio_vmstate;
    dc->desc = "AICSemi AIC8800D80 SDIO Wi-Fi";
    device_class_set_props(dc, aic_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0xc8a1;
    sc->device = 0x0082;
    sc->num_funcs = 2;
    sc->func_class = 0x07;
    sc->max_blksz = BLOCK_SIZE;
    sc->read = aic_sdio_read;
    sc->write = aic_sdio_write;
    sc->io_reset = aic_sdio_io_reset;
    sc->vendor_read = aic_sdio_vendor_read;
    sc->vendor_write = aic_sdio_vendor_write;
}

static const TypeInfo aic_sdio_info = {
    .name           = TYPE_AIC_SDIO,
    .parent         = TYPE_SDIO_CARD,
    .instance_size  = sizeof(AicSdioState),
    .instance_init  = aic_sdio_init,
    .instance_finalize = aic_sdio_finalize,
    .class_size     = sizeof(AicSdioClass),
    .class_init     = aic_sdio_class_init,
};

static void aic_sdio_register_types(void)
{
    type_register_static(&aic_sdio_info);
}

type_init(aic_sdio_register_types)
