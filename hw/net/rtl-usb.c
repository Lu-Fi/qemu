/*
 * Realtek RTL8188FU Wi-Fi on USB
 *
 * Vendor request 5 on endpoint 0 reads and writes the WLAN registers,
 * wValue the address. Bulk IN carries one RX unit a transfer, as the
 * driver takes them; each bulk OUT transfer, which a short packet ends,
 * holds TX descriptors with their frames. The chip behind them is
 * rtl-wifi.c.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "hw/usb/desc.h"
#include "rtl-wifi.h"
#include "trace.h"

#define TYPE_RTL_USB "rtl-usb"
OBJECT_DECLARE_TYPE(RtlUsbState, RtlUsbClass, RTL_USB)

#define VENDOR_REG              5

/* the high, normal and low priority queues, in the order the driver maps */
#define EP_IN                   1
#define EP_OUT_FIRST            2
#define EP_OUTS                 3

/* the most the driver puts in one transfer is 20 KiB */
#define OUT_MAX                 (32 * 1024)

typedef struct RtlUsbChip {
    const char *name;
    const char *desc;
    uint16_t product;
    RtlFamily family;
} RtlUsbChip;

struct RtlUsbClass {
    USBDeviceClass parent_class;

    const RtlUsbChip *chip;
};

struct RtlUsbState {
    USBDevice parent_obj;

    RtlCore core;
    GByteArray *out[EP_OUTS];
    GByteArray *in;
    size_t in_off;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings rtl_usb_strings = {
    [STR_MANUFACTURER]  = "Realtek",
    [STR_PRODUCT]       = "802.11n WLAN Adapter",
    [STR_SERIALNUMBER]  = "1",
};

#define RTL_USB_EPS(_max)                                                   \
    (USBDescEndpoint[]) {                                                   \
        {                                                                   \
            .bEndpointAddress      = USB_DIR_IN | EP_IN,                    \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | EP_OUT_FIRST,            \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | (EP_OUT_FIRST + 1),      \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | (EP_OUT_FIRST + 2),      \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        },                                                                  \
    }

static const USBDescIface rtl_usb_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 1 + EP_OUTS,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = RTL_USB_EPS(64),
};

static const USBDescIface rtl_usb_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 1 + EP_OUTS,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = RTL_USB_EPS(512),
};

static const USBDescDevice rtl_usb_device_full = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 250,
            .nif = 1,
            .ifs = &rtl_usb_iface_full,
        },
    },
};

static const USBDescDevice rtl_usb_device_high = {
    .bcdUSB                        = 0x0200,
    .bMaxPacketSize0               = 64,
    .bNumConfigurations            = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces        = 1,
            .bConfigurationValue   = 1,
            .bmAttributes          = USB_CFG_ATT_ONE,
            .bMaxPower             = 250,
            .nif = 1,
            .ifs = &rtl_usb_iface_high,
        },
    },
};

static const RtlUsbChip rtl_usb_chips[] = {
    { "rtl8188eu", "Realtek RTL8188EUS USB Wi-Fi", 0x8179, RTL_FAMILY_8188E },
    { "rtl8733bu", "Realtek RTL8733BU USB Wi-Fi", 0xf72b, RTL_FAMILY_8733B },
    { "rtl8188fu", "Realtek RTL8188FU USB Wi-Fi", 0xf179, RTL_FAMILY_8188F },
};

static void rtl_usb_notify(RtlCore *c)
{
    /* the host polls bulk IN; nothing to wake */
}

static void rtl_usb_drop(RtlUsbState *s)
{
    for (int i = 0; i < EP_OUTS; i++) {
        g_byte_array_set_size(s->out[i], 0);
    }
    g_clear_pointer(&s->in, g_byte_array_unref);
}

static void rtl_usb_handle_reset(USBDevice *dev)
{
    RtlUsbState *s = RTL_USB(dev);

    rtl_usb_drop(s);
    rtl_core_reset(&s->core);
}

/*
 * A register read or write. The driver counts a failed request against
 * the chip and gives up on it after four in a row, so none fails.
 */
static void rtl_usb_handle_control(USBDevice *dev, USBPacket *p,
                                   int request, int value, int index,
                                   int length, uint8_t *data)
{
    RtlUsbState *s = RTL_USB(dev);
    uint16_t addr = value;

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }
    switch (request) {
    case VendorDeviceRequest | VENDOR_REG:
        rtl_core_read(&s->core, addr, data, length);
        trace_rtl_usb_read(addr, length,
                           length >= 4 ? ldl_le_p(data) : data[0]);
        p->actual_length = length;
        return;
    case VendorDeviceOutRequest | VENDOR_REG:
        trace_rtl_usb_write(addr, length,
                            length >= 4 ? ldl_le_p(data) : data[0]);
        rtl_core_write(&s->core, addr, data, length);
        return;
    }
    p->status = USB_RET_STALL;
}

/* What the host sends, until a short packet ends the transfer */
static void rtl_usb_take(RtlUsbState *s, USBPacket *p)
{
    GByteArray *out = s->out[p->ep->nr - EP_OUT_FIRST];
    size_t len = out->len;

    if (len + p->iov.size > OUT_MAX) {
        g_byte_array_set_size(out, 0);
        return;
    }
    g_byte_array_set_size(out, len + p->iov.size);
    usb_packet_copy(p, out->data + len, p->iov.size);
    if (p->iov.size % p->ep->max_packet_size == 0 && p->iov.size) {
        return;
    }
    rtl_core_tx(&s->core, out->data, out->len);
    g_byte_array_set_size(out, 0);
}

/*
 * An RX unit for the host. One that fills whole packets ends its transfer
 * with the zero-length packet the chip would send, which in QEMU is the
 * transfer coming back short of what was asked.
 */
static void rtl_usb_give(RtlUsbState *s, USBPacket *p)
{
    size_t n;

    if (!s->in) {
        s->in = rtl_core_rx_pop(&s->core);
        s->in_off = 0;
        if (!s->in) {
            p->status = USB_RET_NAK;
            return;
        }
    }
    n = MIN(p->iov.size, s->in->len - s->in_off);
    usb_packet_copy(p, s->in->data + s->in_off, n);
    s->in_off += n;
    if (s->in_off == s->in->len) {
        g_clear_pointer(&s->in, g_byte_array_unref);
    }
}

static void rtl_usb_handle_data(USBDevice *dev, USBPacket *p)
{
    RtlUsbState *s = RTL_USB(dev);

    if (p->pid == USB_TOKEN_OUT && p->ep->nr >= EP_OUT_FIRST &&
        p->ep->nr < EP_OUT_FIRST + EP_OUTS) {
        rtl_usb_take(s, p);
    } else if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_IN) {
        rtl_usb_give(s, p);
    } else {
        p->status = USB_RET_STALL;
    }
}

static void rtl_usb_realize(USBDevice *dev, Error **errp)
{
    RtlUsbState *s = RTL_USB(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    rtl_core_realize(&s->core, errp);
}

static void rtl_usb_unrealize(USBDevice *dev)
{
    RtlUsbState *s = RTL_USB(dev);

    rtl_core_unrealize(&s->core);
}

static void rtl_usb_init(Object *obj)
{
    RtlUsbState *s = RTL_USB(obj);
    const RtlUsbChip *chip = RTL_USB_GET_CLASS(obj)->chip;

    s->core.dev = DEVICE(obj);
    s->core.notify = rtl_usb_notify;
    s->core.family = chip->family;
    for (int i = 0; i < EP_OUTS; i++) {
        s->out[i] = g_byte_array_new();
    }
    rtl_core_init(&s->core);
}

static void rtl_usb_finalize(Object *obj)
{
    RtlUsbState *s = RTL_USB(obj);

    for (int i = 0; i < EP_OUTS; i++) {
        g_byte_array_unref(s->out[i]);
    }
    g_clear_pointer(&s->in, g_byte_array_unref);
    rtl_core_finalize(&s->core);
}

static const Property rtl_usb_props[] = {
    DEFINE_RTL_PROPERTIES(RtlUsbState, core),
};

/* the queued units and the chip's registers are not migrated */
static const VMStateDescription rtl_usb_vmstate = {
    .name = TYPE_RTL_USB,
    .unmigratable = 1,
};

static void rtl_usb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->realize = rtl_usb_realize;
    uc->unrealize = rtl_usb_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = rtl_usb_handle_reset;
    uc->handle_control = rtl_usb_handle_control;
    uc->handle_data = rtl_usb_handle_data;
    dc->vmsd = &rtl_usb_vmstate;
    device_class_set_props(dc, rtl_usb_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void rtl_usb_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);
    RtlUsbClass *rc = RTL_USB_CLASS(oc);
    const RtlUsbChip *chip = data;
    USBDesc *desc = g_new0(USBDesc, 1);

    desc->id.idVendor = 0x0bda;
    desc->id.idProduct = chip->product;
    desc->id.bcdDevice = 0x0200;
    desc->id.iManufacturer = STR_MANUFACTURER;
    desc->id.iProduct = STR_PRODUCT;
    desc->id.iSerialNumber = STR_SERIALNUMBER;
    desc->full = &rtl_usb_device_full;
    desc->high = &rtl_usb_device_high;
    desc->str = rtl_usb_strings;

    rc->chip = chip;
    uc->usb_desc = desc;
    uc->product_desc = chip->desc;
    dc->desc = chip->desc;
}

static void rtl_usb_register_types(void)
{
    static const TypeInfo rtl_usb_info = {
        .name           = TYPE_RTL_USB,
        .parent         = TYPE_USB_DEVICE,
        .instance_size  = sizeof(RtlUsbState),
        .instance_init  = rtl_usb_init,
        .instance_finalize = rtl_usb_finalize,
        .class_size     = sizeof(RtlUsbClass),
        .class_init     = rtl_usb_class_init,
        .abstract       = true,
    };

    type_register_static(&rtl_usb_info);
    for (size_t i = 0; i < ARRAY_SIZE(rtl_usb_chips); i++) {
        const TypeInfo ti = {
            .name       = rtl_usb_chips[i].name,
            .parent     = TYPE_RTL_USB,
            .class_init = rtl_usb_chip_class_init,
            .class_data = &rtl_usb_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(rtl_usb_register_types)
