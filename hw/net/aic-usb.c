/*
 * AICSemi AIC8800, AIC8800D80, AIC8800DC and AIC8800DW Wi-Fi on USB
 *
 * The D80 comes up as its boot ROM, A69C:8D80, taking messages on bulk
 * endpoint 1 and answering on the same endpoint. Once the host has
 * written the firmware and started it, the chip drops off the bus and
 * comes back as A69C:8D81, with two Bluetooth interfaces besides the
 * Wi-Fi one, whose endpoint 1 carries frames and endpoint 2 messages,
 * each way. The DC and the DW come up running that firmware, both as
 * A69C:88DC: the DC with its two Bluetooth interfaces, the DW with the
 * Wi-Fi one alone, which is how their driver tells them apart (it turns
 * down an 88DD without the Bluetooth ones). The AIC8800 boots as the D80
 * does, from A69C:8800 to A69C:8801, but its driver reads no second IN
 * endpoint: its messages come on endpoint 1 with the frames. The chip
 * behind them is aic-wifi.c.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/resettable.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "hw/usb/desc.h"
#include "aic-wifi.h"
#include "trace.h"

#define TYPE_AIC_USB "aic8800-usb"
OBJECT_DECLARE_TYPE(AicUsbState, AicUsbClass, AIC_USB)

#define AIC_VENDOR              0xa69c

#define REG_CHIP_ID             0x40500000
#define REG_CHIP_SUB_ID         0x00000020

typedef struct AicUsbChip {
    const char *desc;
    const USBDesc *rom;             /* the boot ROM, or none */
    const USBDesc *app;             /* the firmware */
    const AicWord *words;
    bool msgs_on_data;              /* the firmware's on endpoint 1 too */
} AicUsbChip;

struct AicUsbClass {
    USBDeviceClass parent_class;
    const AicUsbChip *chip;
};

#define USB_CLASS_WIRELESS      0xe0

#define EP_DATA                 1
#define EP_MSG                  2

/* a packet's length has 12 bits */
#define TX_MAX                  (8 * 1024)

/* how long the chip stays off the bus when it restarts */
#define REPLUG_MS               100

struct AicUsbState {
    USBDevice parent_obj;

    AicCore core;
    QEMUTimer *replug;
    /* by endpoint, less one: what the host sends, what it is being sent */
    GByteArray *tx[2];
    GByteArray *in[2];
    size_t in_off[2];
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

#define AIC_USB_STRINGS(_product)                                           \
    {                                                                       \
        [STR_MANUFACTURER]  = "AICSemi",                                    \
        [STR_PRODUCT]       = (_product),                                   \
        [STR_SERIALNUMBER]  = "1",                                          \
    }

static const USBDescStrings aic_usb_8800_strings = AIC_USB_STRINGS("AIC8800");
static const USBDescStrings aic_usb_d80_strings = AIC_USB_STRINGS("AIC8800D80");
static const USBDescStrings aic_usb_dc_strings = AIC_USB_STRINGS("AIC8800DC");
static const USBDescStrings aic_usb_dw_strings = AIC_USB_STRINGS("AIC8800DW");

/* the driver takes the first IN and OUT for frames, the second for messages */
#define AIC_USB_EPS(_max)                                                   \
    (USBDescEndpoint[]) {                                                   \
        {                                                                   \
            .bEndpointAddress      = USB_DIR_IN | EP_DATA,                  \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | EP_DATA,                 \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_IN | EP_MSG,                   \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | EP_MSG,                  \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        },                                                                  \
    }

#define AIC_USB_WLAN(_max)                                                  \
    {                                                                       \
        .bInterfaceNumber          = 0,                                     \
        .bNumEndpoints             = 4,                                     \
        .bInterfaceClass           = USB_CLASS_VENDOR_SPEC,                 \
        .bInterfaceSubClass        = 0xff,                                  \
        .bInterfaceProtocol        = 0xff,                                  \
        .eps = AIC_USB_EPS(_max),                                           \
    }

/* nothing drives the Bluetooth side; its interfaces are there to count */
#define AIC_USB_BT(_nr)                                                     \
    {                                                                       \
        .bInterfaceNumber          = (_nr),                                 \
        .bInterfaceClass           = USB_CLASS_WIRELESS,                    \
        .bInterfaceSubClass        = 0x01,                                  \
        .bInterfaceProtocol        = 0x01,                                  \
    }

#define AIC_USB_DEVICE(_nif, _ifs)                                          \
    (const USBDescDevice) {                                                 \
        .bcdUSB                    = 0x0200,                                \
        .bMaxPacketSize0           = 64,                                    \
        .bNumConfigurations        = 1,                                     \
        .confs = (USBDescConfig[]) {                                        \
            {                                                               \
                .bNumInterfaces    = (_nif),                                \
                .bConfigurationValue = 1,                                   \
                .bmAttributes      = USB_CFG_ATT_ONE,                       \
                .bMaxPower         = 250,                                   \
                .nif = (_nif),                                              \
                .ifs = (_ifs),                                              \
            },                                                              \
        },                                                                  \
    }

/* the Wi-Fi interface alone, or with the Bluetooth ones */
static const USBDescIface aic_usb_wifi_full[] = { AIC_USB_WLAN(64) };
static const USBDescIface aic_usb_wifi_high[] = { AIC_USB_WLAN(512) };
static const USBDescIface aic_usb_combo_full[] = {
    AIC_USB_WLAN(64), AIC_USB_BT(1), AIC_USB_BT(2),
};
static const USBDescIface aic_usb_combo_high[] = {
    AIC_USB_WLAN(512), AIC_USB_BT(1), AIC_USB_BT(2),
};

static const USBDescDevice aic_usb_wifi_device_full =
    AIC_USB_DEVICE(1, aic_usb_wifi_full);
static const USBDescDevice aic_usb_wifi_device_high =
    AIC_USB_DEVICE(1, aic_usb_wifi_high);
static const USBDescDevice aic_usb_combo_device_full =
    AIC_USB_DEVICE(3, aic_usb_combo_full);
static const USBDescDevice aic_usb_combo_device_high =
    AIC_USB_DEVICE(3, aic_usb_combo_high);

#define AIC_USB_DESC(_product, _str, _dev)                                  \
    {                                                                       \
        .id = {                                                             \
            .idVendor           = AIC_VENDOR,                               \
            .idProduct          = (_product),                               \
            .bcdDevice          = 0x0100,                                   \
            .iManufacturer      = STR_MANUFACTURER,                         \
            .iProduct           = STR_PRODUCT,                              \
            .iSerialNumber      = STR_SERIALNUMBER,                         \
        },                                                                  \
        .full = &aic_usb_##_dev##_device_full,                              \
        .high = &aic_usb_##_dev##_device_high,                              \
        .str  = (_str),                                                     \
    }

static const USBDesc aic_usb_8800_rom =
    AIC_USB_DESC(0x8800, aic_usb_8800_strings, wifi);
static const USBDesc aic_usb_8800_app =
    AIC_USB_DESC(0x8801, aic_usb_8800_strings, combo);
static const USBDesc aic_usb_d80_rom = AIC_USB_DESC(0x8d80, aic_usb_d80_strings,
                                                    wifi);
static const USBDesc aic_usb_d80_app = AIC_USB_DESC(0x8d81, aic_usb_d80_strings,
                                                    combo);
static const USBDesc aic_usb_dc_app = AIC_USB_DESC(0x88dc, aic_usb_dc_strings,
                                                   combo);
static const USBDesc aic_usb_dw_app = AIC_USB_DESC(0x88dc, aic_usb_dw_strings,
                                                   wifi);

/*
 * What the drivers read of the chips: the revision in bits 23:16 (3, an
 * AIC8800's U02, and the D80's and DC's H-bonded ones, bits 7:6 set), and
 * the DC's and DW's ROM, 2: an H part's U02, whose patches are the ones
 * thingino ships
 */
static const AicWord aic_8800_words[] = {
    { REG_CHIP_ID, 0x00030000 },
    { }
};

static const AicWord aic_d80_words[] = {
    { REG_CHIP_ID, 0x02c30000 },
    { }
};

static const AicWord aic_dc_words[] = {
    { REG_CHIP_ID, 0x02c10000 },
    { REG_CHIP_SUB_ID, 2 },
    { }
};

static const AicUsbChip aic_usb_8800 = {
    .desc = "AICSemi AIC8800 USB Wi-Fi",
    .rom = &aic_usb_8800_rom,
    .app = &aic_usb_8800_app,
    .words = aic_8800_words,
    .msgs_on_data = true,
};

static const AicUsbChip aic_usb_d80 = {
    .desc = "AICSemi AIC8800D80 USB Wi-Fi",
    .rom = &aic_usb_d80_rom,
    .app = &aic_usb_d80_app,
    .words = aic_d80_words,
};

static const AicUsbChip aic_usb_dc = {
    .desc = "AICSemi AIC8800DC USB Wi-Fi",
    .app = &aic_usb_dc_app,
    .words = aic_dc_words,
};

static const AicUsbChip aic_usb_dw = {
    .desc = "AICSemi AIC8800DW USB Wi-Fi",
    .app = &aic_usb_dw_app,
    .words = aic_dc_words,
};

static void aic_usb_notify(AicCore *c)
{
    /* the host polls bulk IN; nothing to wake */
}

static void aic_usb_drop(AicUsbState *s)
{
    for (int i = 0; i < 2; i++) {
        g_byte_array_set_size(s->tx[i], 0);
        g_clear_pointer(&s->in[i], g_byte_array_unref);
    }
}

/* Off the bus, then back on it as whatever the chip now runs */
static void aic_usb_replug(void *opaque)
{
    AicUsbState *s = opaque;
    USBDevice *dev = USB_DEVICE(s);

    if (dev->attached) {
        trace_aic_usb_replug(s->core.app);
        usb_device_detach(dev);
        aic_usb_drop(s);
        dev->usb_desc = s->core.app ? AIC_USB_GET_CLASS(s)->chip->app : NULL;
        timer_mod(s->replug,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + REPLUG_MS);
        return;
    }
    usb_device_attach(dev, &error_abort);
}

static void aic_usb_restart(AicCore *c)
{
    AicUsbState *s = container_of(c, AicUsbState, core);

    /* the transfer that started it completes first */
    timer_mod(s->replug, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
}

static void aic_usb_handle_control(USBDevice *dev, USBPacket *p,
                                   int request, int value, int index,
                                   int length, uint8_t *data)
{
    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) < 0) {
        p->status = USB_RET_STALL;
    }
}

static void aic_usb_handle_reset(USBDevice *dev)
{
    /* a bus reset leaves the chip running whatever it runs */
    aic_usb_drop(AIC_USB(dev));
}

/* What the host sends, until a short packet ends the transfer */
static void aic_usb_take(AicUsbState *s, int i, USBPacket *p)
{
    GByteArray *tx = s->tx[i];
    size_t len = tx->len;

    if (len + p->iov.size > TX_MAX) {
        g_byte_array_set_size(tx, 0);
        return;
    }
    g_byte_array_set_size(tx, len + p->iov.size);
    usb_packet_copy(p, tx->data + len, p->iov.size);
    if (p->iov.size % p->ep->max_packet_size == 0 && p->iov.size) {
        return;
    }
    if (tx->len) {
        aic_core_host(&s->core, tx->data, tx->len);
    }
    g_byte_array_set_size(tx, 0);
}

/* One packet a transfer, NAKed while there is none */
static void aic_usb_give(AicUsbState *s, int i, USBPacket *p,
                         GByteArray *(*pop)(AicCore *c))
{
    size_t n;

    if (!s->in[i]) {
        s->in[i] = pop(&s->core);
        s->in_off[i] = 0;
        if (!s->in[i]) {
            p->status = USB_RET_NAK;
            return;
        }
    }
    n = MIN(p->iov.size, s->in[i]->len - s->in_off[i]);
    usb_packet_copy(p, s->in[i]->data + s->in_off[i], n);
    s->in_off[i] += n;
    if (s->in_off[i] == s->in[i]->len) {
        g_clear_pointer(&s->in[i], g_byte_array_unref);
    }
}

/* A message ahead of a frame */
static GByteArray *aic_usb_pop_any(AicCore *c)
{
    return aic_core_pop_msg(c) ?: aic_core_pop_frame(c);
}

/*
 * The boot ROM answers where it was asked; the firmware sends frames on
 * one endpoint and messages on the other, or on the first as well
 */
static void aic_usb_handle_data(USBDevice *dev, USBPacket *p)
{
    AicUsbState *s = AIC_USB(dev);
    int i = p->ep->nr - 1;

    if (p->ep->nr != EP_DATA && p->ep->nr != EP_MSG) {
        p->status = USB_RET_STALL;
        return;
    }
    if (p->pid == USB_TOKEN_OUT) {
        aic_usb_take(s, i, p);
    } else if (!s->core.app || AIC_USB_GET_CLASS(s)->chip->msgs_on_data) {
        if (p->ep->nr == EP_DATA) {
            aic_usb_give(s, i, p, s->core.app ? aic_usb_pop_any :
                                                aic_core_pop_msg);
        } else {
            p->status = USB_RET_NAK;
        }
    } else {
        aic_usb_give(s, i, p, p->ep->nr == EP_DATA ? aic_core_pop_frame :
                                                     aic_core_pop_msg);
    }
}

static void aic_usb_realize(USBDevice *dev, Error **errp)
{
    AicUsbState *s = AIC_USB(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    aic_core_realize(&s->core, errp);
}

static void aic_usb_unrealize(USBDevice *dev)
{
    AicUsbState *s = AIC_USB(dev);

    timer_del(s->replug);
    aic_core_unrealize(&s->core);
}

/* Power on: the boot ROM again, which the host enumerates after its reset */
static void aic_usb_reset_hold(Object *obj, ResetType type)
{
    AicUsbState *s = AIC_USB(obj);
    USBDevice *dev = USB_DEVICE(obj);

    aic_usb_drop(s);
    aic_core_reset(&s->core);
    dev->usb_desc = NULL;
    if (dev->attached) {
        timer_del(s->replug);
        usb_desc_attach(dev);
    }
}

static void aic_usb_init(Object *obj)
{
    AicUsbState *s = AIC_USB(obj);
    const AicUsbChip *chip = AIC_USB_GET_CLASS(obj)->chip;

    s->core.dev = DEVICE(obj);
    s->core.rom = chip->rom;
    s->core.words = chip->words;
    s->core.notify = aic_usb_notify;
    s->core.replug = aic_usb_restart;
    s->replug = timer_new_ms(QEMU_CLOCK_VIRTUAL, aic_usb_replug, s);
    for (int i = 0; i < 2; i++) {
        s->tx[i] = g_byte_array_new();
    }
    aic_core_init(&s->core);
}

static void aic_usb_finalize(Object *obj)
{
    AicUsbState *s = AIC_USB(obj);

    aic_usb_drop(s);
    for (int i = 0; i < 2; i++) {
        g_byte_array_unref(s->tx[i]);
    }
    timer_free(s->replug);
    aic_core_finalize(&s->core);
}

static const Property aic_usb_props[] = {
    DEFINE_AIC_PROPERTIES(AicUsbState, core),
};

/* the chip's memory, the queued messages and the joins are not migrated */
static const VMStateDescription aic_usb_vmstate = {
    .name = TYPE_AIC_USB,
    .unmigratable = 1,
};

static void aic_usb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);
    const AicUsbChip *chip = data;

    AIC_USB_CLASS(oc)->chip = chip;

    uc->realize = aic_usb_realize;
    uc->unrealize = aic_usb_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = aic_usb_handle_reset;
    uc->handle_control = aic_usb_handle_control;
    uc->handle_data = aic_usb_handle_data;
    uc->usb_desc = chip->rom ?: chip->app;
    uc->product_desc = chip->desc;
    rc->phases.hold = aic_usb_reset_hold;
    dc->desc = chip->desc;
    dc->vmsd = &aic_usb_vmstate;
    device_class_set_props(dc, aic_usb_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

#define AIC_USB_TYPE(_name, _chip)                                          \
    {                                                                       \
        .name           = (_name),                                          \
        .parent         = TYPE_AIC_USB,                                     \
        .class_init     = aic_usb_class_init,                               \
        .class_data     = &(_chip),                                         \
    }

static const TypeInfo aic_usb_types[] = {
    {
        .name           = TYPE_AIC_USB,
        .parent         = TYPE_USB_DEVICE,
        .instance_size  = sizeof(AicUsbState),
        .instance_init  = aic_usb_init,
        .instance_finalize = aic_usb_finalize,
        .class_size     = sizeof(AicUsbClass),
        .abstract       = true,
    },
    AIC_USB_TYPE("aic8800", aic_usb_8800),
    AIC_USB_TYPE("aic8800d80", aic_usb_d80),
    AIC_USB_TYPE("aic8800dc", aic_usb_dc),
    AIC_USB_TYPE("aic8800dw", aic_usb_dw),
};

DEFINE_TYPES(aic_usb_types)
