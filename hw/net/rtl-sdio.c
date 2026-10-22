/*
 * Realtek RTL8189FS and RTL8189ES Wi-Fi on SDIO
 *
 * Function 1 is split by the top address bits into what the drivers call
 * devices: the SDIO local registers, the TX FIFOs of each queue, the RX
 * FIFO, and with bit 16 set the chip's WLAN register space.
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
#include "rtl-wifi.h"
#include "trace.h"

#define TYPE_RTL_SDIO "rtl-sdio"
OBJECT_DECLARE_TYPE(RtlSdioState, RtlSdioClass, RTL_SDIO)

#define DEV_SDIO_LOCAL          0
#define DEV_TX_EXQ              3
#define DEV_TX_LOQ              6
#define DEV_RX0FF               7
#define DEV_WLAN_IOREG          8

#define SDIO_LOCAL_SIZE         0x1000
/* the TX FIFO address carries the length of what follows, in dwords */
#define TX_FIFO_DWORDS          0x1fff

/*
 * The host interrupt mask and status, then the length of what the RX
 * FIFO holds: the driver reads all three in one go. RX_REQUEST is a level
 * that falls as the FIFO empties.
 */
#define SDIO_REG_HIMR           0x0014
#define SDIO_REG_HISR           0x0018
#define SDIO_REG_RX0_REQ_LEN    0x001c
#define HISR_RX_REQUEST         BIT(0)

/*
 * Free TX pages per queue, and the OQT slots: the 8188F reads them as
 * 16-bit counts from 0x20 and its OQT at 0x2a, the 8188E as bytes from
 * 0x20 and at 0x25. Every page is back as soon as a write is taken.
 */
#define SDIO_REG_FREE_TXPG      0x0020
#define SDIO_REG_FREE_TXPG_END  0x0030

/*
 * Indirect access to the WLAN registers, which the 8192F's driver uses for
 * page 0: an address, then the size in [1:0] and a write or read trigger;
 * the access is done, and ready set, at once
 */
#define SDIO_REG_INDIRECT_CFG   0x0040
#define SDIO_REG_INDIRECT_CMD   0x0042
#define SDIO_REG_INDIRECT_DATA  0x0044
#define INDIRECT_SIZE_MASK      3
#define INDIRECT_WRITE          BIT(2)
#define INDIRECT_READ           BIT(3)
#define INDIRECT_RDY            BIT(4)

/* HCI suspend: bit 0 asks, bit 1 says the resumed power is ready */
#define SDIO_REG_HSUS_CTRL      0x0086
#define HCI_SUS_REQ             BIT(0)
#define HCI_RESUME_PWR_RDY      BIT(1)

typedef struct RtlSdioChip {
    const char *name;
    const char *desc;
    uint16_t device;
    RtlFamily family;
} RtlSdioChip;

struct RtlSdioClass {
    SDIOCardClass parent_class;

    const RtlSdioChip *chip;
};

static const RtlSdioChip rtl_sdio_chips[] = {
    { "rtl8189fs", "Realtek RTL8189FS SDIO Wi-Fi", 0xf179, RTL_FAMILY_8188F },
    { "rtl8189es", "Realtek RTL8189ES SDIO Wi-Fi", 0x8179, RTL_FAMILY_8188E },
};

struct RtlSdioState {
    SDIOCard parent_obj;

    RtlCore core;
    uint8_t local[SDIO_LOCAL_SIZE];
};

/* Bit 16 picks the WLAN registers, else bits 15-13 the device */
static unsigned rtl_sdio_device(uint32_t addr)
{
    return addr & BIT(16) ? DEV_WLAN_IOREG : (addr >> 13) & 7;
}

/* What the RX FIFO holds, in HISR and RX0_REQ_LEN, and the line it drives */
static void rtl_sdio_update(RtlCore *c)
{
    RtlSdioState *s = container_of(c, RtlSdioState, core);
    GByteArray *head = rtl_core_rx_peek(c);
    uint32_t hisr = head ? HISR_RX_REQUEST : 0;

    stl_le_p(s->local + SDIO_REG_HISR, hisr);
    stw_le_p(s->local + SDIO_REG_RX0_REQ_LEN, head ? head->len : 0);
    sdio_card_set_irq(SDIO_CARD(s), 1,
                      hisr & ldl_le_p(s->local + SDIO_REG_HIMR));
}

/* One read of the FIFO takes one unit, padded out with zeros */
static void rtl_sdio_rx(RtlSdioState *s, uint8_t *buf, size_t len)
{
    g_autoptr(GByteArray) u = rtl_core_rx_pop(&s->core);

    if (u) {
        memcpy(buf, u->data, MIN(len, u->len));
    }
    rtl_sdio_update(&s->core);
}

static void rtl_sdio_read(SDIOCard *card, unsigned fn, uint32_t addr,
                          bool incr, uint8_t *buf, size_t len)
{
    RtlSdioState *s = RTL_SDIO(card);
    unsigned dev = rtl_sdio_device(addr);
    uint32_t off = addr & (RTL_IOREG_SIZE - 1);

    memset(buf, 0, len);
    switch (dev) {
    case DEV_SDIO_LOCAL:
        off = addr & (SDIO_LOCAL_SIZE - 1);
        memcpy(buf, s->local + off, MIN(len, SDIO_LOCAL_SIZE - off));
        break;
    case DEV_RX0FF:
        rtl_sdio_rx(s, buf, len);
        break;
    case DEV_WLAN_IOREG:
        rtl_core_read(&s->core, off, buf, len);
        break;
    }
    trace_rtl_sdio_read(dev, off, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void rtl_sdio_indirect(RtlSdioState *s)
{
    uint8_t *cmd = &s->local[SDIO_REG_INDIRECT_CMD];
    uint32_t addr = lduw_le_p(s->local + SDIO_REG_INDIRECT_CFG);
    size_t size = 1 << MIN(*cmd & INDIRECT_SIZE_MASK, 2);

    if (*cmd & INDIRECT_WRITE) {
        rtl_core_write(&s->core, addr, s->local + SDIO_REG_INDIRECT_DATA,
                       size);
    } else if (*cmd & INDIRECT_READ) {
        rtl_core_read(&s->core, addr, s->local + SDIO_REG_INDIRECT_DATA,
                      size);
    } else {
        return;
    }
    *cmd = (*cmd & ~(INDIRECT_WRITE | INDIRECT_READ)) | INDIRECT_RDY;
}

static void rtl_sdio_local_write(RtlSdioState *s, uint32_t off,
                                 const uint8_t *buf, size_t len)
{
    uint8_t *hsus = &s->local[SDIO_REG_HSUS_CTRL];

    len = MIN(len, SDIO_LOCAL_SIZE - off);
    for (size_t i = 0; i < len; i++) {
        uint32_t reg = off + i;

        /* status, lengths and page counts are the card's to set */
        if (reg >= SDIO_REG_HISR && reg < SDIO_REG_FREE_TXPG_END) {
            continue;
        }
        s->local[reg] = buf[i];
    }
    if (off <= SDIO_REG_HSUS_CTRL && off + len > SDIO_REG_HSUS_CTRL) {
        *hsus = *hsus & HCI_SUS_REQ ? *hsus & ~HCI_RESUME_PWR_RDY
                                    : *hsus | HCI_RESUME_PWR_RDY;
    }
    if (off <= SDIO_REG_INDIRECT_CMD && off + len > SDIO_REG_INDIRECT_CMD) {
        rtl_sdio_indirect(s);
    }
    rtl_sdio_update(&s->core);
}

static void rtl_sdio_write(SDIOCard *card, unsigned fn, uint32_t addr,
                           bool incr, const uint8_t *buf, size_t len)
{
    RtlSdioState *s = RTL_SDIO(card);
    unsigned dev = rtl_sdio_device(addr);
    uint32_t off = addr & (RTL_IOREG_SIZE - 1);

    switch (dev) {
    case DEV_SDIO_LOCAL:
        off = addr & (SDIO_LOCAL_SIZE - 1);
        rtl_sdio_local_write(s, off, buf, len);
        break;
    case DEV_TX_EXQ ... DEV_TX_LOQ:
        off = addr & TX_FIFO_DWORDS;
        rtl_core_tx(&s->core, buf, MIN(len, off * 4));
        break;
    case DEV_WLAN_IOREG:
        rtl_core_write(&s->core, off, buf, len);
        break;
    }
    trace_rtl_sdio_write(dev, off, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void rtl_sdio_local_reset(RtlSdioState *s)
{
    memset(s->local, 0, sizeof(s->local));
    memset(s->local + SDIO_REG_FREE_TXPG, 0xff,
           SDIO_REG_FREE_TXPG_END - SDIO_REG_FREE_TXPG);
    s->local[SDIO_REG_HSUS_CTRL] = HCI_RESUME_PWR_RDY;
}

static void rtl_sdio_io_reset(SDIOCard *card)
{
    RtlSdioState *s = RTL_SDIO(card);

    rtl_sdio_local_reset(s);
    rtl_core_reset(&s->core);
}

static void rtl_sdio_realize(DeviceState *dev, Error **errp)
{
    RtlSdioState *s = RTL_SDIO(dev);

    rtl_sdio_local_reset(s);
    rtl_core_realize(&s->core, errp);
}

static void rtl_sdio_unrealize(DeviceState *dev)
{
    RtlSdioState *s = RTL_SDIO(dev);

    rtl_core_unrealize(&s->core);
}

static void rtl_sdio_init(Object *obj)
{
    RtlSdioState *s = RTL_SDIO(obj);
    const RtlSdioChip *chip = RTL_SDIO_GET_CLASS(obj)->chip;

    s->core.dev = DEVICE(obj);
    s->core.notify = rtl_sdio_update;
    s->core.family = chip->family;
    rtl_core_init(&s->core);
}

static void rtl_sdio_finalize(Object *obj)
{
    RtlSdioState *s = RTL_SDIO(obj);

    rtl_core_finalize(&s->core);
}

static const Property rtl_sdio_props[] = {
    DEFINE_RTL_PROPERTIES(RtlSdioState, core),
};

/* the queued frames and the chip state are not migrated */
static const VMStateDescription rtl_sdio_vmstate = {
    .name = TYPE_RTL_SDIO,
    .unmigratable = 1,
};

static void rtl_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = rtl_sdio_realize;
    dc->unrealize = rtl_sdio_unrealize;
    dc->vmsd = &rtl_sdio_vmstate;
    device_class_set_props(dc, rtl_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x024c;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = 512;
    sc->read = rtl_sdio_read;
    sc->write = rtl_sdio_write;
    sc->io_reset = rtl_sdio_io_reset;
}

static void rtl_sdio_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);
    const RtlSdioChip *chip = data;

    RTL_SDIO_CLASS(oc)->chip = chip;
    dc->desc = chip->desc;
    sc->device = chip->device;
}

static void rtl_sdio_register_types(void)
{
    static const TypeInfo rtl_sdio_info = {
        .name           = TYPE_RTL_SDIO,
        .parent         = TYPE_SDIO_CARD,
        .instance_size  = sizeof(RtlSdioState),
        .instance_init  = rtl_sdio_init,
        .instance_finalize = rtl_sdio_finalize,
        .class_size     = sizeof(RtlSdioClass),
        .class_init     = rtl_sdio_class_init,
        .abstract       = true,
    };

    type_register_static(&rtl_sdio_info);
    for (size_t i = 0; i < ARRAY_SIZE(rtl_sdio_chips); i++) {
        const TypeInfo ti = {
            .name       = rtl_sdio_chips[i].name,
            .parent     = TYPE_RTL_SDIO,
            .class_init = rtl_sdio_chip_class_init,
            .class_data = &rtl_sdio_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(rtl_sdio_register_types)
