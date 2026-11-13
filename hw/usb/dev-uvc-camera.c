/*
 * USB Video Class 1.1 camera
 *
 * A webcam for the guest's USB host: a Linux uvcvideo driver binds to it and
 * a V4L2 program gets frames of MJPEG, YUYV or NV12 from it. Streaming is
 * bulk. The pictures are a moving test pattern with the frame number in its
 * top left corner, or the scene of an image-sensor device given as "sensor".
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
#include "hw/usb/usb.h"
#include "hw/sensor/image-sensor.h"
#include "migration/vmstate.h"
#include "desc.h"
#include "uvc-jpeg.h"

#define TYPE_USB_UVC_CAMERA "usb-uvc-camera"
OBJECT_DECLARE_SIMPLE_TYPE(UVCCamera, USB_UVC_CAMERA)

#define UVC_CLOCK_HZ        30000000
#define UVC_HDR_LEN         12
#define UVC_CTRL_LEN        34      /* a UVC 1.1 probe/commit block */

#define UVC_VC_IF           0
#define UVC_VS_IF           1
#define UVC_ID_CAMERA       1
#define UVC_ID_PU           2
#define UVC_ID_OUTPUT       3

/* class requests */
#define UVC_SET_CUR         0x01
#define UVC_GET_CUR         0x81
#define UVC_GET_MIN         0x82
#define UVC_GET_MAX         0x83
#define UVC_GET_RES         0x84
#define UVC_GET_LEN         0x85
#define UVC_GET_INFO        0x86
#define UVC_GET_DEF         0x87

#define UVC_VS_PROBE        1
#define UVC_VS_COMMIT       2
#define UVC_VC_ERROR_CODE   2
#define UVC_PU_BRIGHTNESS   2
#define UVC_PU_CONTRAST     3

enum {
    UVC_STR_MANUFACTURER = 1,
    UVC_STR_PRODUCT,
    UVC_STR_SERIAL,
};

typedef enum UVCFormat {
    UVC_MJPEG = 1,
    UVC_YUYV,
    UVC_NV12,
    UVC_NFORMATS = UVC_NV12,
} UVCFormat;

typedef struct UVCSize {
    uint16_t w, h;
} UVCSize;

static const UVCSize mjpeg_sizes[] = {
    { 1920, 1080 }, { 1280, 720 }, { 640, 480 }, { 640, 360 }, { 320, 240 },
};
static const UVCSize raw_sizes[] = {
    { 640, 480 }, { 640, 360 }, { 320, 240 },
};
/* 100 ns units: 30, 15 and 1 frames a second */
static const uint32_t intervals[] = { 333333, 666666, 10000000 };

#define UVC_NSIZES(f) ((f) == UVC_MJPEG ? ARRAY_SIZE(mjpeg_sizes) : \
                       ARRAY_SIZE(raw_sizes))
#define UVC_SIZE(f, i) ((f) == UVC_MJPEG ? mjpeg_sizes[(i)] : raw_sizes[(i)])

#define UVC_NDESC_VS (1 + UVC_NFORMATS + \
                      ARRAY_SIZE(mjpeg_sizes) + 2 * ARRAY_SIZE(raw_sizes))

typedef struct UVCStream {
    UVCFormat fmt;
    unsigned frame;                 /* 1-based, as the descriptors */
    uint32_t interval;
} UVCStream;

struct UVCCamera {
    USBDevice dev;

    uint32_t quality;               /* of the MJPEG frames */
    uint32_t payload;               /* dwMaxPayloadTransferSize */
    bool stamp;
    ImageSensorState *sensor;

    uint8_t probe[UVC_CTRL_LEN];
    uint8_t commit[UVC_CTRL_LEN];
    int16_t brightness;
    int16_t contrast;
    uint8_t error;

    bool streaming;
    UVCStream cur;
    uint8_t *frame;                 /* the one being sent, or NULL */
    size_t frame_len, pos;
    size_t pl_sent;                 /* of the payload being sent */
    bool pl_eof;
    bool zlp;                       /* a short payload ended on a packet */
    bool fid;
    uint64_t frame_no;
    uint32_t pts;
    int64_t frame_start, next_start;
    USBPacket *pending;
    QEMUTimer *timer;

    USBDesc desc;
    USBDescDevice dev_hs, dev_fs;
    USBDescConfig conf_hs, conf_fs;
    USBDescIfaceAssoc iad_hs, iad_fs;
    USBDescIface vc, vs_hs, vs_fs;
    USBDescEndpoint ep_hs, ep_fs;
    USBDescOther vc_descs[4];
    USBDescOther vs_descs[UVC_NDESC_VS];
    USBDescIface ifs_hs[2], ifs_fs[2];
    GPtrArray *blobs;
};

static const USBDescStrings uvc_strings = {
    [UVC_STR_MANUFACTURER] = "QEMU",
    [UVC_STR_PRODUCT]      = "QEMU USB UVC Camera",
    [UVC_STR_SERIAL]       = "1",
};

/* descriptors */

static void uvc_put(GByteArray *a, const void *p, size_t n)
{
    g_byte_array_append(a, p, n);
}

static void uvc_put8(GByteArray *a, unsigned v)
{
    uint8_t b = v;

    uvc_put(a, &b, 1);
}

static void uvc_put16(GByteArray *a, unsigned v)
{
    uvc_put8(a, v);
    uvc_put8(a, v >> 8);
}

static void uvc_put32(GByteArray *a, uint32_t v)
{
    uvc_put16(a, v);
    uvc_put16(a, v >> 16);
}

/* Keep @a as one class-specific descriptor of @s */
static void uvc_desc_add(UVCCamera *s, USBDescOther *o, GByteArray *a)
{
    assert(a->len < 256);
    a->data[0] = a->len;
    o->length = a->len;
    o->data = a->data;
    g_ptr_array_add(s->blobs, g_byte_array_free(a, false));
}

static GByteArray *uvc_cs(unsigned subtype)
{
    GByteArray *a = g_byte_array_new();

    uvc_put8(a, 0);
    uvc_put8(a, 0x24);              /* CS_INTERFACE */
    uvc_put8(a, subtype);
    return a;
}

static unsigned uvc_frame_buffer(UVCFormat fmt, UVCSize sz)
{
    switch (fmt) {
    case UVC_MJPEG:
        return sz.w * sz.h;
    case UVC_NV12:
        return sz.w * sz.h * 3 / 2;
    default:
        return sz.w * sz.h * 2;
    }
}

static void uvc_build_vc(UVCCamera *s)
{
    GByteArray *a;

    a = uvc_cs(1);                  /* VC_HEADER */
    uvc_put16(a, 0x0110);
    uvc_put16(a, 13 + 18 + 12 + 9);
    uvc_put32(a, UVC_CLOCK_HZ);
    uvc_put8(a, 1);
    uvc_put8(a, UVC_VS_IF);
    assert(a->len == 13);
    uvc_desc_add(s, &s->vc_descs[0], a);

    a = uvc_cs(2);                  /* VC_INPUT_TERMINAL: camera */
    uvc_put8(a, UVC_ID_CAMERA);
    uvc_put16(a, 0x0201);
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    uvc_put16(a, 0);
    uvc_put16(a, 0);
    uvc_put16(a, 0);
    uvc_put8(a, 3);
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    assert(a->len == 18);
    uvc_desc_add(s, &s->vc_descs[1], a);

    a = uvc_cs(5);                  /* VC_PROCESSING_UNIT */
    uvc_put8(a, UVC_ID_PU);
    uvc_put8(a, UVC_ID_CAMERA);
    uvc_put16(a, 0);
    uvc_put8(a, 2);
    uvc_put8(a, 0x03);              /* brightness, contrast */
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    assert(a->len == 12);
    uvc_desc_add(s, &s->vc_descs[2], a);

    a = uvc_cs(3);                  /* VC_OUTPUT_TERMINAL: USB streaming */
    uvc_put8(a, UVC_ID_OUTPUT);
    uvc_put16(a, 0x0101);
    uvc_put8(a, 0);
    uvc_put8(a, UVC_ID_PU);
    uvc_put8(a, 0);
    assert(a->len == 9);
    uvc_desc_add(s, &s->vc_descs[3], a);
}

static void uvc_build_vs(UVCCamera *s)
{
    GByteArray *a;
    unsigned n = 0, total = 12 + UVC_NFORMATS;
    static const uint8_t yuy2[16] = {
        'Y', 'U', 'Y', '2', 0x00, 0x00, 0x10, 0x00,
        0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
    };
    static const uint8_t nv12[16] = {
        'N', 'V', '1', '2', 0x00, 0x00, 0x10, 0x00,
        0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71,
    };

    for (UVCFormat f = UVC_MJPEG; f <= UVC_NFORMATS; f++) {
        total += (f == UVC_MJPEG ? 11 : 27) +
                 UVC_NSIZES(f) * (26 + 4 * ARRAY_SIZE(intervals));
    }

    a = uvc_cs(1);                  /* VS_INPUT_HEADER */
    uvc_put8(a, UVC_NFORMATS);
    uvc_put16(a, total);
    uvc_put8(a, 0x81);
    uvc_put8(a, 0);
    uvc_put8(a, UVC_ID_OUTPUT);
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    uvc_put8(a, 0);
    uvc_put8(a, 1);
    for (int i = 0; i < UVC_NFORMATS; i++) {
        uvc_put8(a, 0);
    }
    uvc_desc_add(s, &s->vs_descs[n++], a);

    for (UVCFormat f = UVC_MJPEG; f <= UVC_NFORMATS; f++) {
        a = uvc_cs(f == UVC_MJPEG ? 6 : 4);
        uvc_put8(a, f);
        uvc_put8(a, UVC_NSIZES(f));
        if (f == UVC_MJPEG) {
            uvc_put8(a, 0);             /* bmFlags */
        } else {
            uvc_put(a, f == UVC_YUYV ? yuy2 : nv12, 16);
            uvc_put8(a, f == UVC_YUYV ? 16 : 12);
        }
        uvc_put8(a, 1);
        uvc_put8(a, 0);
        uvc_put8(a, 0);
        uvc_put8(a, 0);
        uvc_put8(a, 0);
        uvc_desc_add(s, &s->vs_descs[n++], a);

        for (unsigned i = 0; i < UVC_NSIZES(f); i++) {
            UVCSize sz = UVC_SIZE(f, i);
            unsigned fb = uvc_frame_buffer(f, sz);

            a = uvc_cs(f == UVC_MJPEG ? 7 : 5);
            uvc_put8(a, i + 1);
            uvc_put8(a, 0);
            uvc_put16(a, sz.w);
            uvc_put16(a, sz.h);
            uvc_put32(a, fb * 8 * 1);
            uvc_put32(a, fb * 8 * 30);
            uvc_put32(a, fb);
            uvc_put32(a, intervals[0]);
            uvc_put8(a, ARRAY_SIZE(intervals));
            for (int k = 0; k < ARRAY_SIZE(intervals); k++) {
                uvc_put32(a, intervals[k]);
            }
            uvc_desc_add(s, &s->vs_descs[n++], a);
        }
    }
    assert(n == UVC_NDESC_VS);
}

static void uvc_build_desc(UVCCamera *s)
{
    s->blobs = g_ptr_array_new_with_free_func(g_free);
    uvc_build_vc(s);
    uvc_build_vs(s);

    s->vc = (USBDescIface) {
        .bInterfaceNumber = UVC_VC_IF,
        .bInterfaceClass = 0x0e,
        .bInterfaceSubClass = 1,
        .ndesc = ARRAY_SIZE(s->vc_descs),
        .descs = s->vc_descs,
    };
    s->ep_hs = (USBDescEndpoint) {
        .bEndpointAddress = USB_DIR_IN | 1,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize = 512,
    };
    s->ep_fs = s->ep_hs;
    s->ep_fs.wMaxPacketSize = 64;
    s->vs_hs = (USBDescIface) {
        .bInterfaceNumber = UVC_VS_IF,
        .bNumEndpoints = 1,
        .bInterfaceClass = 0x0e,
        .bInterfaceSubClass = 2,
        .ndesc = ARRAY_SIZE(s->vs_descs),
        .descs = s->vs_descs,
        .eps = &s->ep_hs,
    };
    s->vs_fs = s->vs_hs;
    s->vs_fs.eps = &s->ep_fs;
    s->ifs_hs[0] = s->vc;
    s->ifs_hs[1] = s->vs_hs;
    s->ifs_fs[0] = s->vc;
    s->ifs_fs[1] = s->vs_fs;

    s->iad_hs = (USBDescIfaceAssoc) {
        .bFirstInterface = UVC_VC_IF,
        .bInterfaceCount = 2,
        .bFunctionClass = 0x0e,
        .bFunctionSubClass = 3,
        .nif = 2,
        .ifs = s->ifs_hs,
    };
    s->iad_fs = s->iad_hs;
    s->iad_fs.ifs = s->ifs_fs;

    s->conf_hs = (USBDescConfig) {
        .bNumInterfaces = 2,
        .bConfigurationValue = 1,
        .bmAttributes = USB_CFG_ATT_ONE,
        .bMaxPower = 250,
        .nif_groups = 1,
        .if_groups = &s->iad_hs,
    };
    s->conf_fs = s->conf_hs;
    s->conf_fs.if_groups = &s->iad_fs;

    s->dev_hs = (USBDescDevice) {
        .bcdUSB = 0x0200,
        .bDeviceClass = 0xef,
        .bDeviceSubClass = 2,
        .bDeviceProtocol = 1,
        .bMaxPacketSize0 = 64,
        .bNumConfigurations = 1,
        .confs = &s->conf_hs,
    };
    s->dev_fs = s->dev_hs;
    s->dev_fs.confs = &s->conf_fs;

    s->desc = (USBDesc) {
        .id = {
            .idVendor = 0x46f4,
            .idProduct = 0x0007,
            .bcdDevice = 0,
            .iManufacturer = UVC_STR_MANUFACTURER,
            .iProduct = UVC_STR_PRODUCT,
            .iSerialNumber = UVC_STR_SERIAL,
        },
        .full = &s->dev_fs,
        .high = &s->dev_hs,
        .str = uvc_strings,
    };
}

/* pictures */

static void uvc_rect(uint8_t *nv, unsigned w, unsigned h, unsigned x,
                     unsigned y, unsigned rw, unsigned rh,
                     uint8_t yy, uint8_t u, uint8_t v)
{
    x &= ~1u;
    y &= ~1u;
    rw = MIN(rw, w - x) & ~1u;
    rh = MIN(rh, h - y) & ~1u;
    for (unsigned j = 0; j < rh; j++) {
        memset(nv + (size_t)(y + j) * w + x, yy, rw);
    }
    for (unsigned j = 0; j < rh / 2; j++) {
        uint8_t *c = nv + (size_t)w * h + (size_t)(y / 2 + j) * w + x;

        for (unsigned i = 0; i < rw / 2; i++) {
            c[2 * i] = u;
            c[2 * i + 1] = v;
        }
    }
}

/* A position that runs 0..range and back as @t grows */
static unsigned uvc_bounce(uint64_t t, unsigned range)
{
    uint64_t p = t % (2 * (uint64_t)range);

    return p < range ? p : 2 * range - p;
}

static void uvc_pattern(uint8_t *nv, unsigned w, unsigned h, uint64_t n)
{
    static const uint8_t bars[8][3] = {
        { 180, 128, 128 }, { 162, 44, 142 }, { 131, 156, 44 }, { 112, 72, 58 },
        { 84, 184, 198 }, { 65, 100, 212 }, { 35, 212, 114 }, { 16, 128, 128 },
    };
    uint8_t *yrow = nv, *uv = nv + (size_t)w * h;
    unsigned shift = (n * 6) % w, box = (h / 6) & ~1u;

    for (unsigned x = 0; x < w; x++) {
        const uint8_t *c = bars[((x + shift) % w) * 8 / w];

        yrow[x] = c[0];
        if (!(x & 1)) {
            uv[x] = c[1];
            uv[x + 1] = c[2];
        }
    }
    for (unsigned y = 1; y < h; y++) {
        memcpy(nv + (size_t)y * w, yrow, w);
    }
    for (unsigned y = 1; y < h / 2; y++) {
        memcpy(uv + (size_t)y * w, uv, w);
    }
    uvc_rect(nv, w, h, uvc_bounce(n * 7, w - box), uvc_bounce(n * 5, h - box),
             box, box, 82, 90, 240);
}

static void uvc_stamp(uint8_t *nv, unsigned w, unsigned h, uint64_t n)
{
    static const char *const font[10] = {
        "111101101101111", "010110010010111", "111001111100111",
        "111001111001111", "101101111001001", "111100111001111",
        "111100111101111", "111001001001001", "111101111101111",
        "111101111001111",
    };
    unsigned c = MAX(8, (h / 36) & ~1u), sc = c / 2;

    /* 16 cells, the low 16 bits' most significant first, white for a 1 */
    for (unsigned i = 0; i < 16; i++) {
        bool one = (n >> (15 - i)) & 1;

        uvc_rect(nv, w, h, c + i * c, c, c, c, one ? 235 : 16, 128, 128);
    }
    for (unsigned d = 0; d < 6; d++) {
        const char *g;
        unsigned div = 1;

        for (unsigned k = 0; k < 5 - d; k++) {
            div *= 10;
        }
        g = font[(n / div) % 10];
        uvc_rect(nv, w, h, c + d * 4 * sc, 3 * c, 3 * sc + sc, 5 * sc + sc,
                 16, 128, 128);
        for (unsigned r = 0; r < 5; r++) {
            for (unsigned q = 0; q < 3; q++) {
                if (g[r * 3 + q] == '1') {
                    uvc_rect(nv, w, h, c + d * 4 * sc + q * sc,
                             3 * c + r * sc, sc, sc, 235, 128, 128);
                }
            }
        }
    }
}

static void uvc_adjust(UVCCamera *s, uint8_t *nv, unsigned w, unsigned h)
{
    uint8_t lut[256];

    if (s->brightness == 0 && s->contrast == 100) {
        return;
    }
    for (int i = 0; i < 256; i++) {
        lut[i] = CLAMP((i - 128) * s->contrast / 100 + 128 + s->brightness,
                       0, 255);
    }
    for (size_t i = 0; i < (size_t)w * h; i++) {
        nv[i] = lut[nv[i]];
    }
}

static void uvc_next_frame(UVCCamera *s, int64_t now)
{
    UVCSize sz = UVC_SIZE(s->cur.fmt, s->cur.frame - 1);
    unsigned w = sz.w, h = sz.h;
    uint8_t *nv = g_malloc((size_t)w * h * 3 / 2);
    uint64_t n = s->frame_no++;

    if (s->sensor) {
        image_sensor_render(s->sensor, n, w, h, nv);
    } else {
        uvc_pattern(nv, w, h, n);
    }
    uvc_adjust(s, nv, w, h);
    if (s->stamp) {
        uvc_stamp(nv, w, h, n);
    }

    switch (s->cur.fmt) {
    case UVC_MJPEG: {
        unsigned q = s->quality;

        /* the probe promised w * h bytes at most */
        do {
            g_free(s->frame);
            s->frame = uvc_jpeg_encode(nv, w, h, q, &s->frame_len);
            q = q > 10 ? q - 10 : 1;
        } while (s->frame_len > (size_t)w * h && q > 1);
        g_free(nv);
        break;
    }
    case UVC_YUYV:
        s->frame_len = (size_t)w * h * 2;
        s->frame = g_malloc(s->frame_len);
        for (unsigned y = 0; y < h; y++) {
            const uint8_t *yp = nv + (size_t)y * w;
            const uint8_t *cp = nv + (size_t)w * h + (size_t)(y / 2) * w;
            uint8_t *o = s->frame + (size_t)y * w * 2;

            for (unsigned x = 0; x < w; x += 2) {
                o[2 * x] = yp[x];
                o[2 * x + 1] = cp[x];
                o[2 * x + 2] = yp[x + 1];
                o[2 * x + 3] = cp[x + 1];
            }
        }
        g_free(nv);
        break;
    default:
        s->frame = nv;
        s->frame_len = (size_t)w * h * 3 / 2;
        break;
    }
    s->pos = 0;
    s->pl_sent = 0;
    s->fid = !s->fid;
    s->frame_start = now;
    s->pts = (uint32_t)(n * s->cur.interval * (UVC_CLOCK_HZ / 10000000));
}

/* streaming */

static void uvc_stream_stop(UVCCamera *s)
{
    s->streaming = false;
    g_free(s->frame);
    s->frame = NULL;
    s->pl_sent = 0;
    s->zlp = false;
    if (s->pending) {
        USBPacket *p = s->pending;

        s->pending = NULL;
        p->status = USB_RET_IOERROR;
        usb_packet_complete(&s->dev, p);
    }
    timer_del(s->timer);
}

/* Put the next piece of the frame in @p; false: no frame is due yet */
static bool uvc_fill(UVCCamera *s, USBPacket *p)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    size_t room, n;

    if (!s->frame) {
        if (now < s->next_start) {
            timer_mod(s->timer, s->next_start);
            return false;
        }
        uvc_next_frame(s, now);
    }

    room = MIN(p->iov.size, s->payload - s->pl_sent);
    if (room < UVC_HDR_LEN + 1) {
        p->status = USB_RET_NAK;
        return true;
    }
    if (s->pl_sent == 0) {
        uint8_t hdr[UVC_HDR_LEN];
        uint64_t stc = (uint64_t)now * (UVC_CLOCK_HZ / 1000000) / 1000;

        s->pl_eof = s->frame_len - s->pos <= s->payload - UVC_HDR_LEN;
        hdr[0] = UVC_HDR_LEN;
        hdr[1] = 0x80 | 0x08 | 0x04 | (s->pl_eof ? 2 : 0) | s->fid;
        stl_le_p(hdr + 2, s->pts);
        stl_le_p(hdr + 6, stc);
        stw_le_p(hdr + 10, (now / 1000000) & 0x7ff);
        usb_packet_copy(p, hdr, UVC_HDR_LEN);
        room -= UVC_HDR_LEN;
        s->pl_sent = UVC_HDR_LEN;
    }
    n = MIN(room, s->frame_len - s->pos);
    usb_packet_copy(p, s->frame + s->pos, n);
    s->pos += n;
    s->pl_sent += n;

    if (s->pos == s->frame_len && s->pl_sent < s->payload) {
        /* a payload that fills the packets ends with a zero-length one */
        USBEndpoint *ep = usb_ep_get(&s->dev, USB_TOKEN_IN, 1);

        s->zlp = s->pl_sent % ep->max_packet_size == 0;
    }
    if (s->pos == s->frame_len) {
        int64_t iv = (int64_t)s->cur.interval * 100;

        g_free(s->frame);
        s->frame = NULL;
        s->pl_sent = 0;
        s->next_start = s->frame_start + iv;
        if (s->next_start < now - iv) {
            s->next_start = now;
        }
    } else if (s->pl_sent >= s->payload) {
        s->pl_sent = 0;
    }
    return true;
}

static void uvc_timer(void *opaque)
{
    UVCCamera *s = opaque;
    USBPacket *p = s->pending;

    if (p && uvc_fill(s, p)) {
        s->pending = NULL;
        p->status = USB_RET_SUCCESS;
        usb_packet_complete(&s->dev, p);
    }
}

static void uvc_handle_data(USBDevice *dev, USBPacket *p)
{
    UVCCamera *s = USB_UVC_CAMERA(dev);

    if (p->pid != USB_TOKEN_IN || p->ep->nr != 1) {
        p->status = USB_RET_STALL;
        return;
    }
    if (s->streaming && s->zlp) {
        s->zlp = false;
        return;
    }
    if (!s->streaming || s->pending) {
        p->status = USB_RET_NAK;
        return;
    }
    if (!uvc_fill(s, p)) {
        s->pending = p;
        p->status = USB_RET_ASYNC;
    }
}

static void uvc_cancel_packet(USBDevice *dev, USBPacket *p)
{
    UVCCamera *s = USB_UVC_CAMERA(dev);

    if (s->pending == p) {
        s->pending = NULL;
    }
}

/* control requests */

static unsigned uvc_default_frame(UVCFormat fmt)
{
    return fmt == UVC_MJPEG ? 3 : 1;    /* 640x480 */
}

/* Settle a probe or commit block on what the camera can do */
static void uvc_negotiate(UVCCamera *s, uint8_t *blk)
{
    UVCFormat fmt = blk[2];
    unsigned frame = blk[3], best = 0;
    uint32_t iv = ldl_le_p(blk + 4);
    UVCSize sz;
    uint32_t dist = UINT32_MAX;

    if (fmt < UVC_MJPEG || fmt > UVC_NFORMATS) {
        fmt = UVC_MJPEG;
        frame = 0;
    }
    if (frame < 1 || frame > UVC_NSIZES(fmt)) {
        frame = uvc_default_frame(fmt);
    }
    for (unsigned i = 0; i < ARRAY_SIZE(intervals); i++) {
        uint32_t d = abs((int)intervals[i] - (int)iv);

        if (d < dist) {
            dist = d;
            best = i;
        }
    }
    sz = UVC_SIZE(fmt, frame - 1);

    memset(blk, 0, UVC_CTRL_LEN);
    stw_le_p(blk, 1);                       /* the interval is fixed */
    blk[2] = fmt;
    blk[3] = frame;
    stl_le_p(blk + 4, intervals[best]);
    stl_le_p(blk + 18, uvc_frame_buffer(fmt, sz));
    stl_le_p(blk + 22, s->payload);
    stl_le_p(blk + 26, UVC_CLOCK_HZ);
    blk[30] = 3;                            /* FID and EOF in headers */
}

/* @data: the setup's buffer; returns the bytes for the host or -1 to stall */
static int uvc_vs_request(UVCCamera *s, unsigned req, unsigned cs,
                          uint8_t *data, int len)
{
    uint8_t *blk;

    if (cs != UVC_VS_PROBE && cs != UVC_VS_COMMIT) {
        return -1;
    }
    blk = cs == UVC_VS_PROBE ? s->probe : s->commit;

    switch (req) {
    case UVC_SET_CUR:
        memset(blk, 0, UVC_CTRL_LEN);
        memcpy(blk, data, MIN(len, UVC_CTRL_LEN));
        uvc_negotiate(s, blk);
        if (cs == UVC_VS_COMMIT) {
            uvc_stream_stop(s);
            s->cur.fmt = blk[2];
            s->cur.frame = blk[3];
            s->cur.interval = ldl_le_p(blk + 4);
            s->next_start = 0;
            s->streaming = true;
        }
        return 0;
    case UVC_GET_CUR:
    case UVC_GET_MIN:
    case UVC_GET_MAX:
    case UVC_GET_DEF:
    case UVC_GET_RES:
        if (req == UVC_GET_RES) {
            memset(data, 0, UVC_CTRL_LEN);
        } else {
            memcpy(data, blk, UVC_CTRL_LEN);
        }
        return MIN(len, UVC_CTRL_LEN);
    case UVC_GET_LEN:
        stw_le_p(data, UVC_CTRL_LEN);
        return 2;
    case UVC_GET_INFO:
        data[0] = 3;
        return 1;
    default:
        return -1;
    }
}

static int uvc_pu_request(UVCCamera *s, unsigned req, unsigned cs,
                          uint8_t *data, int len)
{
    int16_t *cur, min, max, def;

    switch (cs) {
    case UVC_PU_BRIGHTNESS:
        cur = &s->brightness;
        min = -64;
        max = 64;
        def = 0;
        break;
    case UVC_PU_CONTRAST:
        cur = &s->contrast;
        min = 0;
        max = 200;
        def = 100;
        break;
    default:
        return -1;
    }

    switch (req) {
    case UVC_SET_CUR:
        if (len < 2) {
            return -1;
        }
        *cur = CLAMP((int16_t)lduw_le_p(data), min, max);
        return 0;
    case UVC_GET_CUR:
        stw_le_p(data, *cur);
        return 2;
    case UVC_GET_MIN:
        stw_le_p(data, min);
        return 2;
    case UVC_GET_MAX:
        stw_le_p(data, max);
        return 2;
    case UVC_GET_RES:
        stw_le_p(data, 1);
        return 2;
    case UVC_GET_DEF:
        stw_le_p(data, def);
        return 2;
    case UVC_GET_LEN:
        stw_le_p(data, 2);
        return 2;
    case UVC_GET_INFO:
        data[0] = 3;
        return 1;
    default:
        return -1;
    }
}

static void uvc_handle_control(USBDevice *dev, USBPacket *p, int request,
                               int value, int index, int length,
                               uint8_t *data)
{
    UVCCamera *s = USB_UVC_CAMERA(dev);
    unsigned req = request & 0xff, cs = value >> 8;
    unsigned ifn = index & 0xff, unit = index >> 8;
    int ret;

    ret = usb_desc_handle_control(dev, p, request, value, index, length, data);
    if (ret >= 0) {
        return;
    }

    switch (request & ~0xff) {
    case ClassInterfaceRequest:
    case ClassInterfaceOutRequest:
        break;
    case EndpointOutRequest:
        if (req == USB_REQ_CLEAR_FEATURE) {
            uvc_stream_stop(s);
            return;
        }
        p->status = USB_RET_STALL;
        return;
    default:
        p->status = USB_RET_STALL;
        return;
    }

    if (ifn == UVC_VS_IF && unit == 0) {
        ret = uvc_vs_request(s, req, cs, data, length);
    } else if (ifn == UVC_VC_IF && unit == 0 && cs == UVC_VC_ERROR_CODE &&
               req == UVC_GET_CUR) {
        data[0] = s->error;
        s->error = 0;
        ret = 1;
    } else if (ifn == UVC_VC_IF && unit == UVC_ID_PU) {
        ret = uvc_pu_request(s, req, cs, data, length);
    } else {
        ret = -1;
    }

    if (ret < 0) {
        s->error = 6;       /* invalid control */
        p->status = USB_RET_STALL;
        return;
    }
    p->actual_length = ret;
}

static void uvc_handle_reset(USBDevice *dev)
{
    uvc_stream_stop(USB_UVC_CAMERA(dev));
}

static void uvc_realize(USBDevice *dev, Error **errp)
{
    UVCCamera *s = USB_UVC_CAMERA(dev);

    if (s->payload < 1024 || s->payload > 65536 || s->payload % 512) {
        error_setg(errp, "payload must be a multiple of 512 from 1024 to "
                   "65536");
        return;
    }
    uvc_build_desc(s);
    dev->usb_desc = &s->desc;
    usb_desc_create_serial(dev);
    usb_desc_init(dev);

    s->brightness = 0;
    s->contrast = 100;
    s->probe[2] = UVC_MJPEG;
    s->probe[3] = uvc_default_frame(UVC_MJPEG);
    stl_le_p(s->probe + 4, intervals[0]);
    uvc_negotiate(s, s->probe);
    memcpy(s->commit, s->probe, sizeof(s->commit));
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uvc_timer, s);
}

static void uvc_unrealize(USBDevice *dev)
{
    UVCCamera *s = USB_UVC_CAMERA(dev);

    uvc_stream_stop(s);
    timer_free(s->timer);
    g_ptr_array_free(s->blobs, true);
}

static const VMStateDescription vmstate_uvc = {
    .name = TYPE_USB_UVC_CAMERA,
    .unmigratable = 1,
};

static const Property uvc_properties[] = {
    DEFINE_PROP_UINT32("quality", UVCCamera, quality, 85),
    DEFINE_PROP_UINT32("payload", UVCCamera, payload, 16384),
    DEFINE_PROP_BOOL("stamp", UVCCamera, stamp, true),
    DEFINE_PROP_LINK("sensor", UVCCamera, sensor, TYPE_IMAGE_SENSOR,
                     ImageSensorState *),
};

static void uvc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    USBDeviceClass *k = USB_DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_uvc;
    device_class_set_props(dc, uvc_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    k->product_desc = "QEMU USB UVC Camera";
    k->realize = uvc_realize;
    k->unrealize = uvc_unrealize;
    k->handle_reset = uvc_handle_reset;
    k->handle_control = uvc_handle_control;
    k->handle_data = uvc_handle_data;
    k->cancel_packet = uvc_cancel_packet;
}

static const TypeInfo uvc_info = {
    .name = TYPE_USB_UVC_CAMERA,
    .parent = TYPE_USB_DEVICE,
    .instance_size = sizeof(UVCCamera),
    .class_init = uvc_class_init,
};

static void uvc_register_types(void)
{
    type_register_static(&uvc_info);
}

type_init(uvc_register_types)
