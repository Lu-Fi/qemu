/*
 * AltoBeam ATBM60xx Wi-Fi on USB
 *
 * Vendor requests on endpoint 0 read and write the chip's registers and
 * memory, a 32-bit address split between wValue (low half) and wIndex.
 * Bulk endpoints carry one WSM message a transfer each way. The firmware
 * behind them is atbm-wifi.c.
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
#include "atbm-wifi.h"
#include "trace.h"

#define TYPE_ATBM_USB "atbm-usb"
OBJECT_DECLARE_TYPE(AtbmUsbState, AtbmUsbClass, ATBM_USB)

#define VENDOR_HW_READ          0
#define VENDOR_HW_WRITE         1
#define VENDOR_HW_RESET         6
#define VENDOR_EP0_CMD          7

/* VENDOR_HW_RESET: what to reset, in the low byte of wValue */
#define HW_HOLD_CPU             BIT(0)
#define HW_RESET_HIF            BIT(1)

#define EP_IN                   1
#define EP_OUT                  2

/*
 * The header of a message from the host: flag, usb_len, usb_id, len, id;
 * the Wi-Fi 6 drivers widen it, keeping usb_len and ending in len and id
 */
#define OUT_HDR(s)              ((s)->core.wifi6 ? 32 : 12)
#define OUT_MAX                 2048
#define HS_PACKET               512

typedef struct AtbmUsbChip {
    const char *name;
    const char *desc;
    uint16_t product;
    uint8_t chip_id;
    /* the efuse byte the driver tells this part from its siblings by */
    uint8_t efuse_index;
    uint8_t efuse;
    bool wifi6;
} AtbmUsbChip;

struct AtbmUsbClass {
    USBDeviceClass parent_class;

    const AtbmUsbChip *chip;
};

struct AtbmUsbState {
    USBDevice parent_obj;

    AtbmCore core;
    uint8_t out[OUT_MAX];
    size_t out_len;
    GByteArray *in;
    size_t in_off;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings atbm_usb_strings = {
    [STR_MANUFACTURER]  = "QEMU",
    [STR_PRODUCT]       = "ATBM60xx Wi-Fi",
    [STR_SERIALNUMBER]  = "1",
};

static const USBDescIface atbm_usb_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | EP_IN,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 64,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | EP_OUT,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = 64,
        },
    },
};

static const USBDescIface atbm_usb_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 2,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = (USBDescEndpoint[]) {
        {
            .bEndpointAddress      = USB_DIR_IN | EP_IN,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = HS_PACKET,
        }, {
            .bEndpointAddress      = USB_DIR_OUT | EP_OUT,
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,
            .wMaxPacketSize        = HS_PACKET,
        },
    },
};

static const USBDescDevice atbm_usb_device_full = {
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
            .ifs = &atbm_usb_iface_full,
        },
    },
};

static const USBDescDevice atbm_usb_device_high = {
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
            .ifs = &atbm_usb_iface_high,
        },
    },
};

/*
 * Chip ids and efuse as the drivers decode them: 0x49 is the AresB die,
 * which efuse bits 152 and 154 make a 6012B; 0x4a is AsmLite, whose bits 8
 * and 10 without 3 make a 6012B-X; 0xa0 is Mercurius; 0x5a is Cronus; 0x5c
 * is Cronus Lite, the 6062-C; 0x30 is Oceanus, which efuse bit 16 makes a
 * 6132-C without Wi-Fi 6. The 6132CU ships on USB id 0x6055, and on 0x6162
 * as the part paired with Bluetooth.
 */
static const AtbmUsbChip atbm_usb_chips[] = {
    { "atbm6012b",  "AltoBeam ATBM6012B USB Wi-Fi",  0x8888, 0x49, 19, 0x05 },
    { "atbm6012bx", "AltoBeam ATBM6012B-X USB Wi-Fi", 0x888b, 0x4a, 1, 0x05 },
    { "atbm6032",   "AltoBeam ATBM6032 USB Wi-Fi",   0x8888, 0x49, 0, 0 },
    { "atbm6132u",  "AltoBeam ATBM6132U USB Wi-Fi",  0x8890, 0xa0, 0, 0 },
    { "atbm6062u",  "AltoBeam ATBM6062U USB Wi-Fi",  0x6052, 0x5a, 0, 0, true },
    { "atbm6062cu", "AltoBeam ATBM6062CU USB Wi-Fi", 0x6055, 0x5c, 0, 0, true },
    { "atbm6132cu", "AltoBeam ATBM6132CU USB Wi-Fi", 0x6055, 0x30, 2, 0x01,
      true },
    { "atbm6132cu-6162", "AltoBeam ATBM6132CU USB Wi-Fi, Bluetooth part",
      0x6162, 0x30, 2, 0x01, true },
};

static void atbm_usb_notify(AtbmCore *c)
{
    /* the host polls bulk IN; nothing to wake */
}

static void atbm_usb_handle_reset(USBDevice *dev)
{
    AtbmUsbState *s = ATBM_USB(dev);

    s->out_len = 0;
    g_clear_pointer(&s->in, g_byte_array_unref);
    atbm_core_hold(&s->core);
}

static void atbm_usb_handle_control(USBDevice *dev, USBPacket *p,
                                    int request, int value, int index,
                                    int length, uint8_t *data)
{
    AtbmUsbState *s = ATBM_USB(dev);
    uint32_t addr = (uint16_t)value | (uint32_t)(uint16_t)index << 16;
    uint32_t val;

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }
    switch (request) {
    case VendorDeviceRequest | VENDOR_HW_READ:
        if (length < 4) {
            break;
        }
        val = atbm_core_read32(&s->core, addr);
        trace_atbm_usb_read(addr, val);
        stl_le_p(data, val);
        p->actual_length = 4;
        return;
    case VendorDeviceOutRequest | VENDOR_HW_WRITE:
        /* registers are words; longer writes are firmware going in */
        trace_atbm_usb_write(addr, length, length >= 4 ? ldl_le_p(data) : 0);
        if (length == 4) {
            atbm_core_write32(&s->core, addr, ldl_le_p(data));
        }
        return;
    case VendorDeviceOutRequest | VENDOR_HW_RESET:
        trace_atbm_usb_reset(value);
        if (value & HW_HOLD_CPU) {
            atbm_core_hold(&s->core);
        }
        if (value & HW_RESET_HIF) {
            s->out_len = 0;
            g_clear_pointer(&s->in, g_byte_array_unref);
        }
        return;
    case VendorDeviceOutRequest | VENDOR_EP0_CMD:
        /* the mailbox it rings is not in the startup indication */
        return;
    }
    p->status = USB_RET_STALL;
}

/*
 * A message from the host: usb_len bytes, ended early by a short packet,
 * of which the first len are the message
 */
static void atbm_usb_take(AtbmUsbState *s, USBPacket *p)
{
    size_t n = MIN(p->iov.size, OUT_MAX - s->out_len);
    size_t hdr = OUT_HDR(s);
    size_t len, want;

    usb_packet_copy(p, s->out + s->out_len, n);
    s->out_len += n;
    if (s->out_len < hdr) {
        return;
    }
    want = MIN(lduw_le_p(s->out + 4), OUT_MAX);
    if (s->out_len < want && p->iov.size % HS_PACKET == 0 &&
        s->out_len < OUT_MAX) {
        return;
    }
    len = lduw_le_p(s->out + hdr - 4);
    if (len >= hdr && len <= s->out_len) {
        atbm_core_wsm(&s->core, lduw_le_p(s->out + hdr - 2), s->out + hdr,
                      len - hdr);
    }
    s->out_len = 0;
}

/*
 * A message for the host. One that fills whole packets gets a byte more,
 * as the firmware sends it, so its transfer ends on a short packet.
 */
static void atbm_usb_give(AtbmUsbState *s, USBPacket *p)
{
    size_t n;

    if (!s->in) {
        s->in = atbm_core_pop(&s->core);
        s->in_off = 0;
        if (!s->in) {
            p->status = USB_RET_NAK;
            return;
        }
        if (s->in->len % HS_PACKET == 0) {
            g_byte_array_set_size(s->in, s->in->len + 1);
        }
    }
    n = MIN(p->iov.size, s->in->len - s->in_off);
    usb_packet_copy(p, s->in->data + s->in_off, n);
    s->in_off += n;
    if (s->in_off == s->in->len) {
        g_clear_pointer(&s->in, g_byte_array_unref);
    }
}

static void atbm_usb_handle_data(USBDevice *dev, USBPacket *p)
{
    AtbmUsbState *s = ATBM_USB(dev);

    if (p->pid == USB_TOKEN_OUT && p->ep->nr == EP_OUT) {
        atbm_usb_take(s, p);
    } else if (p->pid == USB_TOKEN_IN && p->ep->nr == EP_IN) {
        atbm_usb_give(s, p);
    } else {
        p->status = USB_RET_STALL;
    }
}

static void atbm_usb_realize(USBDevice *dev, Error **errp)
{
    AtbmUsbState *s = ATBM_USB(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    atbm_core_realize(&s->core, errp);
}

static void atbm_usb_unrealize(USBDevice *dev)
{
    AtbmUsbState *s = ATBM_USB(dev);

    atbm_core_unrealize(&s->core);
}

static void atbm_usb_init(Object *obj)
{
    AtbmUsbState *s = ATBM_USB(obj);
    const AtbmUsbChip *chip = ATBM_USB_GET_CLASS(obj)->chip;

    s->core.dev = DEVICE(obj);
    s->core.notify = atbm_usb_notify;
    s->core.chip_id = chip->chip_id;
    s->core.efuse[chip->efuse_index] = chip->efuse;
    s->core.wifi6 = chip->wifi6;
    atbm_core_init(&s->core);
}

static void atbm_usb_finalize(Object *obj)
{
    AtbmUsbState *s = ATBM_USB(obj);

    g_clear_pointer(&s->in, g_byte_array_unref);
    atbm_core_finalize(&s->core);
}

static const Property atbm_usb_props[] = {
    DEFINE_ATBM_PROPERTIES(AtbmUsbState, core),
};

/* the queued messages and the chip memory are not migrated */
static const VMStateDescription atbm_usb_vmstate = {
    .name = TYPE_ATBM_USB,
    .unmigratable = 1,
};

static void atbm_usb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->realize = atbm_usb_realize;
    uc->unrealize = atbm_usb_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = atbm_usb_handle_reset;
    uc->handle_control = atbm_usb_handle_control;
    uc->handle_data = atbm_usb_handle_data;
    dc->vmsd = &atbm_usb_vmstate;
    device_class_set_props(dc, atbm_usb_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void atbm_usb_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);
    AtbmUsbClass *ac = ATBM_USB_CLASS(oc);
    const AtbmUsbChip *chip = data;
    USBDesc *desc = g_new0(USBDesc, 1);

    desc->id.idVendor = 0x007a;
    desc->id.idProduct = chip->product;
    desc->id.bcdDevice = 0x0001;
    desc->id.iManufacturer = STR_MANUFACTURER;
    desc->id.iProduct = STR_PRODUCT;
    desc->id.iSerialNumber = STR_SERIALNUMBER;
    desc->full = &atbm_usb_device_full;
    desc->high = &atbm_usb_device_high;
    desc->str = atbm_usb_strings;

    ac->chip = chip;
    uc->usb_desc = desc;
    uc->product_desc = chip->desc;
    dc->desc = chip->desc;
}

static void atbm_usb_register_types(void)
{
    static const TypeInfo atbm_usb_info = {
        .name           = TYPE_ATBM_USB,
        .parent         = TYPE_USB_DEVICE,
        .instance_size  = sizeof(AtbmUsbState),
        .instance_init  = atbm_usb_init,
        .instance_finalize = atbm_usb_finalize,
        .class_size     = sizeof(AtbmUsbClass),
        .class_init     = atbm_usb_class_init,
        .abstract       = true,
    };

    type_register_static(&atbm_usb_info);
    for (size_t i = 0; i < ARRAY_SIZE(atbm_usb_chips); i++) {
        const TypeInfo ti = {
            .name       = atbm_usb_chips[i].name,
            .parent     = TYPE_ATBM_USB,
            .class_init = atbm_usb_chip_class_init,
            .class_data = &atbm_usb_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(atbm_usb_register_types)
