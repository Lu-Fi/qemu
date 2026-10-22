/*
 * iComm-semi SSV6155P Wi-Fi on USB
 *
 * Registers go through bulk endpoint 1 as {length, command, sequence,
 * address, value}, the answer coming back on endpoint 2. Endpoint 3
 * carries frames and host commands to the chip and endpoint 4 frames and
 * events back, one a transfer. The firmware is written to SRAM with
 * vendor request 0xf0 on endpoint 0. The chip behind them is ssv-wifi.c.
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
#include "ssv-wifi.h"
#include "trace.h"

#define TYPE_SSV_USB "ssv-usb"
OBJECT_DECLARE_TYPE(SsvUsbState, SsvUsbClass, SSV_USB)

#define VENDOR_FW_WRITE         0xf0

#define EP_REG_OUT              1
#define EP_REG_IN               2
#define EP_TX                   3
#define EP_RX                   4

/* a register command: length, command, sequence, address, value */
#define REG_CMD_LEN             12
#define REG_CMD_WRITE           1
#define REG_CMD_READ            2
#define REG_RSP_LEN             8

/* the most a frame or command takes */
#define TX_MAX                  (8 * 1024)

typedef struct SsvUsbChip {
    const char *name;
    const char *desc;
    uint32_t efuse_id;
} SsvUsbChip;

struct SsvUsbClass {
    USBDeviceClass parent_class;

    const SsvUsbChip *chip;
};

struct SsvUsbState {
    USBDevice parent_obj;

    SsvCore core;
    GQueue rsp;                     /* answers to register commands */
    GByteArray *tx;
    GByteArray *in;
    size_t in_off;
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIALNUMBER,
};

static const USBDescStrings ssv_usb_strings = {
    [STR_MANUFACTURER]  = "iComm-semi",
    [STR_PRODUCT]       = "SSV6X5X",
    [STR_SERIALNUMBER]  = "1",
};

#define SSV_USB_EPS(_max)                                                   \
    (USBDescEndpoint[]) {                                                   \
        {                                                                   \
            .bEndpointAddress      = USB_DIR_OUT | EP_REG_OUT,              \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_IN | EP_REG_IN,                \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_OUT | EP_TX,                   \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        }, {                                                                \
            .bEndpointAddress      = USB_DIR_IN | EP_RX,                    \
            .bmAttributes          = USB_ENDPOINT_XFER_BULK,                \
            .wMaxPacketSize        = (_max),                                \
        },                                                                  \
    }

static const USBDescIface ssv_usb_iface_full = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 4,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = SSV_USB_EPS(64),
};

static const USBDescIface ssv_usb_iface_high = {
    .bInterfaceNumber              = 0,
    .bNumEndpoints                 = 4,
    .bInterfaceClass               = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass            = 0xff,
    .bInterfaceProtocol            = 0xff,
    .eps = SSV_USB_EPS(512),
};

static const USBDescDevice ssv_usb_device_full = {
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
            .ifs = &ssv_usb_iface_full,
        },
    },
};

static const USBDescDevice ssv_usb_device_high = {
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
            .ifs = &ssv_usb_iface_high,
        },
    },
};

/* the eFuse identity tells the parts on the one USB id apart */
static const SsvUsbChip ssv_usb_chips[] = {
    { "ssv6155p", "iComm-semi SSV6155P USB Wi-Fi", 0x70000000 },
};

static void ssv_usb_notify(SsvCore *c)
{
    /* the host polls bulk IN; nothing to wake */
}

static void ssv_usb_drop(SsvUsbState *s)
{
    GByteArray *m;

    while ((m = g_queue_pop_head(&s->rsp))) {
        g_byte_array_unref(m);
    }
    g_byte_array_set_size(s->tx, 0);
    g_clear_pointer(&s->in, g_byte_array_unref);
}

static void ssv_usb_handle_reset(USBDevice *dev)
{
    SsvUsbState *s = SSV_USB(dev);

    ssv_usb_drop(s);
    ssv_core_reset(&s->core);
}

static void ssv_usb_handle_control(USBDevice *dev, USBPacket *p,
                                   int request, int value, int index,
                                   int length, uint8_t *data)
{
    SsvUsbState *s = SSV_USB(dev);

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }
    if (request == (VendorDeviceOutRequest | VENDOR_FW_WRITE)) {
        ssv_core_fw_write(&s->core, (uint16_t)value | (uint32_t)index << 16,
                          data, length);
        return;
    }
    p->status = USB_RET_STALL;
}

/* A register command, answered at once */
static void ssv_usb_reg_cmd(SsvUsbState *s, USBPacket *p)
{
    uint8_t c[REG_CMD_LEN] = {};
    uint8_t *r;
    GByteArray *rsp;
    uint32_t addr, val = 0;

    usb_packet_copy(p, c, MIN(p->iov.size, sizeof(c)));
    addr = ldl_le_p(c + 4);
    switch (c[1]) {
    case REG_CMD_WRITE:
        trace_ssv_usb_reg_write(addr, ldl_le_p(c + 8));
        ssv_core_reg_write(&s->core, addr, ldl_le_p(c + 8));
        break;
    case REG_CMD_READ:
        val = ssv_core_reg_read(&s->core, addr);
        trace_ssv_usb_reg_read(addr, val);
        break;
    default:
        return;
    }
    rsp = g_byte_array_sized_new(REG_RSP_LEN);
    g_byte_array_set_size(rsp, REG_RSP_LEN);
    r = rsp->data;
    r[0] = 4;
    r[1] = c[1];
    r[2] = c[2];
    r[3] = c[3];
    stl_le_p(r + 4, val);
    g_queue_push_tail(&s->rsp, rsp);
}

/* What the host sends, until a short packet ends the transfer */
static void ssv_usb_take(SsvUsbState *s, USBPacket *p)
{
    size_t len = s->tx->len;

    if (len + p->iov.size > TX_MAX) {
        g_byte_array_set_size(s->tx, 0);
        return;
    }
    g_byte_array_set_size(s->tx, len + p->iov.size);
    usb_packet_copy(p, s->tx->data + len, p->iov.size);
    if (p->iov.size % p->ep->max_packet_size == 0 && p->iov.size) {
        return;
    }
    ssv_core_host(&s->core, s->tx->data, s->tx->len);
    g_byte_array_set_size(s->tx, 0);
}

/* One message a transfer, NAKed while there is none */
static void ssv_usb_give(SsvUsbState *s, USBPacket *p)
{
    size_t n;

    if (!s->in) {
        s->in = ssv_core_pop(&s->core);
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

static void ssv_usb_handle_data(USBDevice *dev, USBPacket *p)
{
    SsvUsbState *s = SSV_USB(dev);
    GByteArray *m;

    switch (p->ep->nr) {
    case EP_REG_OUT:
        if (p->pid == USB_TOKEN_OUT) {
            ssv_usb_reg_cmd(s, p);
            return;
        }
        break;
    case EP_REG_IN:
        if (p->pid == USB_TOKEN_IN) {
            m = g_queue_pop_head(&s->rsp);
            if (!m) {
                p->status = USB_RET_NAK;
                return;
            }
            usb_packet_copy(p, m->data, MIN(p->iov.size, m->len));
            g_byte_array_unref(m);
            return;
        }
        break;
    case EP_TX:
        if (p->pid == USB_TOKEN_OUT) {
            ssv_usb_take(s, p);
            return;
        }
        break;
    case EP_RX:
        if (p->pid == USB_TOKEN_IN) {
            ssv_usb_give(s, p);
            return;
        }
        break;
    }
    p->status = USB_RET_STALL;
}

static void ssv_usb_realize(USBDevice *dev, Error **errp)
{
    SsvUsbState *s = SSV_USB(dev);

    usb_desc_create_serial(dev);
    usb_desc_init(dev);
    ssv_core_realize(&s->core, errp);
}

static void ssv_usb_unrealize(USBDevice *dev)
{
    SsvUsbState *s = SSV_USB(dev);

    ssv_core_unrealize(&s->core);
}

static void ssv_usb_init(Object *obj)
{
    SsvUsbState *s = SSV_USB(obj);
    const SsvUsbChip *chip = SSV_USB_GET_CLASS(obj)->chip;

    s->core.dev = DEVICE(obj);
    s->core.notify = ssv_usb_notify;
    s->core.efuse_id = chip->efuse_id;
    g_queue_init(&s->rsp);
    s->tx = g_byte_array_new();
    ssv_core_init(&s->core);
}

static void ssv_usb_finalize(Object *obj)
{
    SsvUsbState *s = SSV_USB(obj);

    ssv_usb_drop(s);
    g_byte_array_unref(s->tx);
    ssv_core_finalize(&s->core);
}

static const Property ssv_usb_props[] = {
    DEFINE_SSV_PROPERTIES(SsvUsbState, core),
};

/* the registers, the queued messages and the firmware are not migrated */
static const VMStateDescription ssv_usb_vmstate = {
    .name = TYPE_SSV_USB,
    .unmigratable = 1,
};

static void ssv_usb_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);

    uc->realize = ssv_usb_realize;
    uc->unrealize = ssv_usb_unrealize;
    uc->handle_attach = usb_desc_attach;
    uc->handle_reset = ssv_usb_handle_reset;
    uc->handle_control = ssv_usb_handle_control;
    uc->handle_data = ssv_usb_handle_data;
    dc->vmsd = &ssv_usb_vmstate;
    device_class_set_props(dc, ssv_usb_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static void ssv_usb_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    USBDeviceClass *uc = USB_DEVICE_CLASS(oc);
    SsvUsbClass *sc = SSV_USB_CLASS(oc);
    const SsvUsbChip *chip = data;
    USBDesc *desc = g_new0(USBDesc, 1);

    desc->id.idVendor = 0x8065;
    desc->id.idProduct = 0x6000;
    desc->id.bcdDevice = 0x0100;
    desc->id.iManufacturer = STR_MANUFACTURER;
    desc->id.iProduct = STR_PRODUCT;
    desc->id.iSerialNumber = STR_SERIALNUMBER;
    desc->full = &ssv_usb_device_full;
    desc->high = &ssv_usb_device_high;
    desc->str = ssv_usb_strings;

    sc->chip = chip;
    uc->usb_desc = desc;
    uc->product_desc = chip->desc;
    dc->desc = chip->desc;
}

static void ssv_usb_register_types(void)
{
    static const TypeInfo ssv_usb_info = {
        .name           = TYPE_SSV_USB,
        .parent         = TYPE_USB_DEVICE,
        .instance_size  = sizeof(SsvUsbState),
        .instance_init  = ssv_usb_init,
        .instance_finalize = ssv_usb_finalize,
        .class_size     = sizeof(SsvUsbClass),
        .class_init     = ssv_usb_class_init,
        .abstract       = true,
    };

    type_register_static(&ssv_usb_info);
    for (size_t i = 0; i < ARRAY_SIZE(ssv_usb_chips); i++) {
        const TypeInfo ti = {
            .name       = ssv_usb_chips[i].name,
            .parent     = TYPE_SSV_USB,
            .class_init = ssv_usb_chip_class_init,
            .class_data = &ssv_usb_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(ssv_usb_register_types)
