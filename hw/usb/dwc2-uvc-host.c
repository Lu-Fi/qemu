/*
 * USB host with a UVC class driver, cabled to the DWC2 device port
 *
 * Enumerates the gadget on the DWC2's device side the way a PC would,
 * negotiates a video format with VS_PROBE_CONTROL and VS_COMMIT_CONTROL,
 * takes the bulk stream and saves the frames it carries, so a guest
 * running a UVC gadget (g_webcam) can be checked end to end:
 *
 *   -device dwc2-uvc-host,x-out=/tmp/uvc,x-format=mjpeg,x-frames=10
 *
 * x-format is a list joined by +. The host streams each format in turn for
 * x-frames frames, committing the next one on the live connection, and
 * writes x-out/uvc-<n>.mjpg or .h264 for the n-th. The files hold the frames
 * as the gadget received them from its application, back to back, with the
 * UVC payload headers stripped.
 *
 * Only bulk streaming is modelled: the gadget's streaming interface must
 * carry its endpoint in alternate setting 0.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/cutils.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hcd-dwc2.h"
#include "trace.h"

#define TYPE_DWC2_UVC_HOST "dwc2-uvc-host"
OBJECT_DECLARE_SIMPLE_TYPE(DWC2UVCHost, DWC2_UVC_HOST)

/* Bus timing (virtual time) */
#define UVC_TICK_NS             (1 * SCALE_MS)  /* one pass over the device */
#define UVC_DEBOUNCE_NS         (100 * SCALE_MS)
#define UVC_RESET_NS            (10 * SCALE_MS)
#define UVC_RECOVERY_NS         (10 * SCALE_MS)
#define UVC_SET_ADDRESS_NS      (2 * SCALE_MS)
#define UVC_CONTROL_TIMEOUT_NS  (5 * NANOSECONDS_PER_SECOND)
#define UVC_ATTEMPTS            5
/*
 * The gadget answers class requests through its application, which may start
 * after the cable is connected (rwc does). Chosen: a minute of virtual time
 * in all, at one timeout per try.
 */
#define UVC_NEGOTIATE_RETRIES   12

#define UVC_ADDRESS             1
#define UVC_CONFIG_MAX          1024
#define UVC_CTL_MAX             64
#define UVC_PROBE_LEN           26      /* UVC 1.0 streaming control */
#define UVC_MAX_FORMATS         8
#define UVC_MAX_SEGMENTS        4
/*
 * A high-speed bus polls the bulk endpoint every microframe, and fits 13
 * packets of 512 bytes in one. The gadget re-arms its endpoint after every
 * packet, so a coarser poll would throttle it below what rwc waits for.
 */
#define UVC_STREAM_TICK_NS      (125 * SCALE_US)
#define UVC_POLL_PACKETS        13
/* Frames reported when dropped, to show what the gadget sent. */
#define UVC_DROP_REPORTS        8
#define UVC_PAYLOAD_MAX        (128 * 1024)
#define UVC_FRAME_MAX          (16 * 1024 * 1024)

#define USB_CLASS_VIDEO_V       0x0e
#define UVC_SC_VIDEOSTREAMING   2
#define UVC_VS_FORMAT_MJPEG     0x06
#define UVC_VS_FRAME_MJPEG      0x07
#define UVC_VS_FORMAT_FRAMEBASED 0x10
#define UVC_VS_FRAME_FRAMEBASED 0x11
#define UVC_SET_CUR             0x01
#define UVC_GET_CUR             0x81
#define UVC_VS_PROBE            0x01
#define UVC_VS_COMMIT           0x02

#define UVC_BH_FID              0x01
#define UVC_BH_EOF              0x02
#define UVC_BH_ERR              0x40

typedef enum {
    UVC_IDLE,           /* nothing on the port */
    UVC_DEBOUNCE,       /* pull-up seen, let it settle */
    UVC_RESET,          /* driving bus reset */
    UVC_RECOVERY,       /* reset recovery */
    UVC_ENUM,           /* control transfers */
    UVC_STREAMING,      /* reading the bulk endpoint */
    UVC_DONE,           /* every format captured */
    UVC_FAILED,         /* gave up until the next attach */
} UVCState;

static const char *const uvc_state_name[] = {
    [UVC_IDLE] = "idle",
    [UVC_DEBOUNCE] = "debounce",
    [UVC_RESET] = "reset",
    [UVC_RECOVERY] = "recovery",
    [UVC_ENUM] = "enumerating",
    [UVC_STREAMING] = "streaming",
    [UVC_DONE] = "done",
    [UVC_FAILED] = "failed",
};

typedef enum {
    STEP_DEVICE_EP0,    /* GET_DESCRIPTOR(DEVICE, 64) for bMaxPacketSize0 */
    STEP_SET_ADDRESS,
    STEP_DEVICE,
    STEP_CONFIG_HEAD,   /* first 9 bytes, for wTotalLength */
    STEP_CONFIG,
    STEP_SET_CONFIG,
    STEP_PROBE_SET,     /* the formats loop from here */
    STEP_PROBE_GET,
    STEP_COMMIT_SET,
    STEP_STREAM,
} UVCStep;

typedef enum {
    CTL_SETUP,
    CTL_DATA,
    CTL_STATUS,
} UVCStage;

typedef enum {
    FMT_MJPEG,
    FMT_H264,
} UVCFormat;

/* A format the gadget offers, with the frame descriptor chosen for it. */
typedef struct UVCOffer {
    UVCFormat kind;
    uint8_t format_index;
    bool has_frame;
    uint16_t width, height;
    uint32_t interval;
} UVCOffer;

struct DWC2UVCHost {
    DeviceState parent_obj;

    DWC2State *dwc2;
    QEMUTimer *timer;
    char *out_dir;
    char *format_list;
    uint32_t frames_per_format;
    uint8_t frame_index;

    UVCState state;
    int64_t wait_until;
    int attempts;
    int negotiate_retries;

    /* The control transfer in flight */
    UVCStep step;
    UVCStage stage;
    uint8_t setup[8];
    uint8_t ctl_buf[UVC_CONFIG_MAX + 64];
    int ctl_len;
    int ctl_pos;
    int64_t ctl_deadline;

    /* From the descriptors */
    int ep0_mps;
    uint16_t vid, pid;
    uint16_t cfg_total;
    uint8_t cfg_value;
    int vs_intf;
    int ep_in, mps_in;
    bool iso_only;
    UVCOffer offers[UVC_MAX_FORMATS];
    int n_offers;

    /* What to stream, and the negotiation */
    UVCFormat wanted[UVC_MAX_SEGMENTS];
    int n_wanted;
    int segment;
    const UVCOffer *offer;
    uint8_t probe[UVC_PROBE_LEN];
    uint32_t max_payload;

    /* The stream */
    uint8_t payload[UVC_PAYLOAD_MAX];
    int payload_len;
    GByteArray *frame;
    int fid;
    bool sync;                  /* a frame boundary has been seen */
    int out_fd;
    uint32_t seg_frames;
    uint64_t seg_bytes;
    uint32_t total_frames;
    uint32_t dropped;           /* cut frames and ones of another format */
    uint32_t bad_payloads;
    uint32_t payloads;
    uint32_t eof_less;          /* frames ended by the frame bit alone */
};

static void uvc_set_state(DWC2UVCHost *s, UVCState state)
{
    if (s->state != state) {
        s->state = state;
        trace_dwc2_uvc_state(uvc_state_name[state]);
    }
}

static void uvc_setup(DWC2UVCHost *s, uint8_t type, uint8_t req,
                      uint16_t value, uint16_t index, uint16_t len)
{
    s->setup[0] = type;
    s->setup[1] = req;
    stw_le_p(s->setup + 2, value);
    stw_le_p(s->setup + 4, index);
    stw_le_p(s->setup + 6, len);
    s->ctl_len = len;
}

static void uvc_start_step(DWC2UVCHost *s, int64_t now)
{
    switch (s->step) {
    case STEP_DEVICE_EP0:
        uvc_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8,
                  0, 64);
        break;
    case STEP_SET_ADDRESS:
        uvc_setup(s, 0, USB_REQ_SET_ADDRESS, UVC_ADDRESS, 0, 0);
        break;
    case STEP_DEVICE:
        uvc_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8,
                  0, 18);
        break;
    case STEP_CONFIG_HEAD:
        uvc_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIG << 8,
                  0, 9);
        break;
    case STEP_CONFIG:
        uvc_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIG << 8,
                  0, s->cfg_total);
        break;
    case STEP_SET_CONFIG:
        uvc_setup(s, 0, USB_REQ_SET_CONFIGURATION, s->cfg_value, 0, 0);
        break;
    case STEP_PROBE_SET:
        /* bmHint 1: the frame interval is the field to hold fixed. */
        memset(s->ctl_buf, 0, UVC_PROBE_LEN);
        stw_le_p(s->ctl_buf, 1);
        s->ctl_buf[2] = s->offer->format_index;
        s->ctl_buf[3] = s->frame_index;
        stl_le_p(s->ctl_buf + 4, s->offer->interval);
        uvc_setup(s, USB_TYPE_CLASS | USB_RECIP_INTERFACE, UVC_SET_CUR,
                  UVC_VS_PROBE << 8, s->vs_intf, UVC_PROBE_LEN);
        break;
    case STEP_PROBE_GET:
        uvc_setup(s, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                  UVC_GET_CUR, UVC_VS_PROBE << 8, s->vs_intf, UVC_PROBE_LEN);
        break;
    case STEP_COMMIT_SET:
        memcpy(s->ctl_buf, s->probe, UVC_PROBE_LEN);
        uvc_setup(s, USB_TYPE_CLASS | USB_RECIP_INTERFACE, UVC_SET_CUR,
                  UVC_VS_COMMIT << 8, s->vs_intf, UVC_PROBE_LEN);
        break;
    default:
        g_assert_not_reached();
    }
    s->stage = CTL_SETUP;
    s->ctl_pos = 0;
    s->ctl_deadline = now + UVC_CONTROL_TIMEOUT_NS;
}

/*
 * Run the control transfer as far as the device lets it: 1 when done,
 * 0 while the device NAKs, a negative handshake when it stalls.
 */
static int uvc_control(DWC2UVCHost *s)
{
    bool in = s->setup[0] & USB_DIR_IN;
    uint8_t zlp[64];
    int n, r;

    for (;;) {
        switch (s->stage) {
        case CTL_SETUP:
            r = dwc2_dev_setup(s->dwc2, s->setup);
            if (r < 0) {
                return r == DWC2_DEV_NAK ? 0 : r;
            }
            s->stage = s->ctl_len ? CTL_DATA : CTL_STATUS;
            break;

        case CTL_DATA:
            if (in) {
                r = dwc2_dev_in(s->dwc2, 0, s->ctl_buf + s->ctl_pos,
                                s->ep0_mps);
                if (r < 0) {
                    return r == DWC2_DEV_NAK ? 0 : r;
                }
                s->ctl_pos += r;
                if (r < s->ep0_mps || s->ctl_pos >= s->ctl_len) {
                    s->ctl_pos = MIN(s->ctl_pos, s->ctl_len);
                    s->stage = CTL_STATUS;
                }
            } else {
                n = MIN(s->ep0_mps, s->ctl_len - s->ctl_pos);
                r = dwc2_dev_out(s->dwc2, 0, s->ctl_buf + s->ctl_pos, n);
                if (r < 0) {
                    return r == DWC2_DEV_NAK ? 0 : r;
                }
                s->ctl_pos += n;
                if (s->ctl_pos >= s->ctl_len) {
                    s->stage = CTL_STATUS;
                }
            }
            break;

        case CTL_STATUS:
            /* The status stage runs opposite to the data stage. */
            if (in && s->ctl_len) {
                r = dwc2_dev_out(s->dwc2, 0, NULL, 0);
            } else {
                r = dwc2_dev_in(s->dwc2, 0, zlp, sizeof(zlp));
            }
            if (r < 0) {
                return r == DWC2_DEV_NAK ? 0 : r;
            }
            return 1;
        }
    }
}

static bool uvc_is_h264_guid(const uint8_t *guid)
{
    return !memcmp(guid, "H264", 4);
}

/*
 * Collect the formats, the frame descriptor wanted for each, and the
 * streaming endpoint from the configuration descriptor.
 */
static bool uvc_parse_config(DWC2UVCHost *s, const uint8_t *d, int len)
{
    int intf = -1, alt = 0, class = 0, subclass = 0, i;
    UVCOffer *cur = NULL;

    s->cfg_value = d[5];
    s->vs_intf = -1;
    s->ep_in = s->mps_in = 0;
    s->iso_only = false;
    s->n_offers = 0;

    for (i = 0; i + 2 <= len && d[i] >= 2 && i + d[i] <= len; i += d[i]) {
        const uint8_t *p = d + i;

        if (p[1] == USB_DT_INTERFACE && p[0] >= 9) {
            intf = p[2];
            alt = p[3];
            class = p[5];
            subclass = p[6];
            if (class == USB_CLASS_VIDEO_V && subclass == UVC_SC_VIDEOSTREAMING
                && s->vs_intf < 0) {
                s->vs_intf = intf;
            }
        } else if (p[1] == USB_DT_CS_INTERFACE && p[0] >= 4 &&
                   intf == s->vs_intf && class == USB_CLASS_VIDEO_V) {
            switch (p[2]) {
            case UVC_VS_FORMAT_MJPEG:
            case UVC_VS_FORMAT_FRAMEBASED:
                cur = NULL;
                if (s->n_offers == UVC_MAX_FORMATS) {
                    break;
                }
                if (p[2] == UVC_VS_FORMAT_FRAMEBASED &&
                    (p[0] < 21 || !uvc_is_h264_guid(p + 5))) {
                    break;
                }
                cur = &s->offers[s->n_offers++];
                memset(cur, 0, sizeof(*cur));
                cur->kind = p[2] == UVC_VS_FORMAT_MJPEG ? FMT_MJPEG : FMT_H264;
                cur->format_index = p[3];
                break;
            case UVC_VS_FRAME_MJPEG:
            case UVC_VS_FRAME_FRAMEBASED:
                if (cur && p[0] >= 26 && p[3] == s->frame_index) {
                    cur->has_frame = true;
                    cur->width = lduw_le_p(p + 5);
                    cur->height = lduw_le_p(p + 7);
                    /* dwDefaultFrameInterval sits 4 bytes apart in the two */
                    cur->interval = ldl_le_p(p + (p[2] == UVC_VS_FRAME_MJPEG ?
                                                  21 : 17));
                }
                break;
            }
        } else if (p[1] == USB_DT_ENDPOINT && p[0] >= 7 &&
                   intf == s->vs_intf && class == USB_CLASS_VIDEO_V &&
                   (p[2] & USB_DIR_IN)) {
            if (alt == 0 && (p[3] & 3) == USB_ENDPOINT_XFER_BULK) {
                s->ep_in = p[2] & 0xf;
                s->mps_in = lduw_le_p(p + 4) & 0x7ff;
            } else if ((p[3] & 3) == USB_ENDPOINT_XFER_ISOC) {
                s->iso_only = true;
            }
        }
    }
    return s->vs_intf >= 0 && s->ep_in && s->mps_in && s->n_offers;
}

static const UVCOffer *uvc_find_offer(DWC2UVCHost *s, UVCFormat kind)
{
    int i;

    for (i = 0; i < s->n_offers; i++) {
        if (s->offers[i].kind == kind && s->offers[i].has_frame) {
            return &s->offers[i];
        }
    }
    return NULL;
}

static const char *uvc_ext(UVCFormat kind)
{
    return kind == FMT_MJPEG ? "mjpg" : "h264";
}

static void uvc_close_segment(DWC2UVCHost *s)
{
    if (s->out_fd >= 0) {
        close(s->out_fd);
        s->out_fd = -1;
    }
}

static bool uvc_open_segment(DWC2UVCHost *s)
{
    UVCFormat kind = s->offer->kind;

    s->seg_frames = 0;
    s->seg_bytes = 0;
    s->payload_len = 0;
    g_byte_array_set_size(s->frame, 0);
    s->sync = false;
    s->fid = -1;
    if (!s->out_dir) {
        return true;
    }
    g_autofree char *path = g_strdup_printf("%s/uvc-%d.%s", s->out_dir,
                                            s->segment, uvc_ext(kind));

    s->out_fd = qemu_open_old(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
                              0644);
    if (s->out_fd < 0) {
        warn_report("%s: cannot create %s: %s", TYPE_DWC2_UVC_HOST, path,
                    strerror(errno));
        return false;
    }
    return true;
}

/* Start over from a bus reset, a few times before giving up. */
static void uvc_retry(DWC2UVCHost *s, int64_t now, const char *why)
{
    if (++s->attempts >= UVC_ATTEMPTS) {
        warn_report("%s: no UVC gadget after %d attempts (%s at step %d)",
                    TYPE_DWC2_UVC_HOST, s->attempts, why, s->step);
        uvc_set_state(s, UVC_FAILED);
        return;
    }
    dwc2_dev_bus_reset(s->dwc2);
    uvc_set_state(s, UVC_RESET);
    s->wait_until = now + UVC_RESET_NS;
}

static void uvc_fail(DWC2UVCHost *s, const char *why)
{
    warn_report("%s: %04x:%04x: %s (step %d, request 0x%02x)",
                TYPE_DWC2_UVC_HOST, s->vid, s->pid, why, s->step,
                s->setup[1]);
    uvc_close_segment(s);
    uvc_set_state(s, UVC_FAILED);
}

/*
 * A request nobody answered may have left the gadget's control endpoint
 * waiting for its application: reset the bus and enumerate again.
 */
static bool uvc_negotiate_retry(DWC2UVCHost *s, int64_t now)
{
    if (++s->negotiate_retries > UVC_NEGOTIATE_RETRIES) {
        return false;
    }
    dwc2_dev_bus_reset(s->dwc2);
    uvc_set_state(s, UVC_RESET);
    s->wait_until = now + UVC_RESET_NS;
    return true;
}

/* The negotiation for the current segment is committed: read the stream. */
static void uvc_start_stream(DWC2UVCHost *s)
{
    s->negotiate_retries = 0;
    if (!uvc_open_segment(s)) {
        uvc_set_state(s, UVC_FAILED);
        return;
    }
    info_report("%s: streaming %s %ux%u, %u frames",
                TYPE_DWC2_UVC_HOST, uvc_ext(s->offer->kind), s->offer->width,
                s->offer->height, s->frames_per_format);
    s->step = STEP_STREAM;
    uvc_set_state(s, UVC_STREAMING);
}

/* The control transfer for s->step finished with handshake r. */
static void uvc_step_done(DWC2UVCHost *s, int64_t now, int r)
{
    const uint8_t *b = s->ctl_buf;
    int len = s->ctl_pos;

    trace_dwc2_uvc_control(s->setup[0], s->setup[1], lduw_le_p(s->setup + 2),
                           lduw_le_p(s->setup + 4), s->ctl_len, r);

    if (r < 0) {
        if (s->step < STEP_PROBE_SET) {
            uvc_retry(s, now, "stall");
        } else if (!uvc_negotiate_retry(s, now)) {
            uvc_fail(s, "the gadget refused the stream negotiation");
        }
        return;
    }

    switch (s->step) {
    case STEP_DEVICE_EP0:
        if (len < 8 || b[1] != USB_DT_DEVICE ||
            (b[7] != 8 && b[7] != 16 && b[7] != 32 && b[7] != 64)) {
            uvc_retry(s, now, "device descriptor");
            return;
        }
        s->ep0_mps = b[7];
        break;
    case STEP_SET_ADDRESS:
        s->wait_until = now + UVC_SET_ADDRESS_NS;
        break;
    case STEP_DEVICE:
        if (len < 18) {
            uvc_retry(s, now, "device descriptor");
            return;
        }
        s->vid = lduw_le_p(b + 8);
        s->pid = lduw_le_p(b + 10);
        break;
    case STEP_CONFIG_HEAD:
        if (len < 9 || b[1] != USB_DT_CONFIG) {
            uvc_retry(s, now, "configuration descriptor");
            return;
        }
        s->cfg_total = MIN(lduw_le_p(b + 2), UVC_CONFIG_MAX);
        break;
    case STEP_CONFIG:
        if (!uvc_parse_config(s, b, len)) {
            uvc_fail(s, s->iso_only ? "isochronous UVC streaming is not "
                     "supported" : "not a UVC gadget with a bulk stream");
            return;
        }
        break;
    case STEP_PROBE_GET:
        if (len < UVC_PROBE_LEN || b[2] != s->offer->format_index ||
            b[3] != s->frame_index) {
            uvc_fail(s, "the gadget changed the format in its probe reply");
            return;
        }
        memcpy(s->probe, b, UVC_PROBE_LEN);
        s->max_payload = ldl_le_p(b + 22);
        break;
    default:
        break;
    }

    if (s->step == STEP_SET_CONFIG) {
        s->offer = uvc_find_offer(s, s->wanted[s->segment]);
        if (!s->offer) {
            uvc_fail(s, "the gadget offers no such format and frame");
            return;
        }
    }
    s->step++;
    if (s->step == STEP_STREAM) {
        uvc_start_stream(s);
        return;
    }
    uvc_start_step(s, now);
}

static bool uvc_has_idr(const uint8_t *b, size_t len)
{
    size_t i;

    for (i = 0; i + 3 < len; i++) {
        if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 1 &&
            ((b[i + 3] & 0x1f) == 5 || (b[i + 3] & 0x1f) == 7)) {
            return true;
        }
    }
    return false;
}

static void uvc_next_segment(DWC2UVCHost *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    info_report("%s: segment %d: %u %s frames, %" PRIu64 " bytes, %u dropped",
                TYPE_DWC2_UVC_HOST, s->segment, s->seg_frames,
                uvc_ext(s->offer->kind), s->seg_bytes, s->dropped);
    uvc_close_segment(s);
    if (++s->segment >= s->n_wanted) {
        uvc_set_state(s, UVC_DONE);
        return;
    }
    s->offer = uvc_find_offer(s, s->wanted[s->segment]);
    if (!s->offer) {
        uvc_fail(s, "the gadget offers no such format and frame");
        return;
    }
    s->step = STEP_PROBE_SET;
    uvc_start_step(s, now);
    uvc_set_state(s, UVC_ENUM);
}

/* The frame is whole: keep it if it is the segment's format. */
static void uvc_frame_done(DWC2UVCHost *s)
{
    const uint8_t *b = s->frame->data;
    size_t len = s->frame->len;
    bool ok;

    if (s->seg_frames >= s->frames_per_format) {
        /* A frame ended in the same payload that completed the count. */
        g_byte_array_set_size(s->frame, 0);
        return;
    }

    if (s->offer->kind == FMT_MJPEG) {
        ok = len > 2 && b[0] == 0xff && b[1] == 0xd8;
    } else {
        /* Start at the first picture a decoder can start from. */
        ok = len > 4 && b[0] == 0 && b[1] == 0 &&
             (s->seg_frames || uvc_has_idr(b, len));
    }
    if (!ok) {
        if (s->dropped < UVC_DROP_REPORTS) {
            warn_report("%s: dropped a %zu byte frame starting %02x %02x %02x "
                        "%02x", TYPE_DWC2_UVC_HOST, len, len > 0 ? b[0] : 0,
                        len > 1 ? b[1] : 0, len > 2 ? b[2] : 0,
                        len > 3 ? b[3] : 0);
        }
        s->dropped++;
    } else {
        if (s->out_fd >= 0 && qemu_write_full(s->out_fd, b, len) != len) {
            warn_report("%s: write failed: %s", TYPE_DWC2_UVC_HOST,
                        strerror(errno));
        }
        s->seg_frames++;
        s->seg_bytes += len;
        s->total_frames++;
        trace_dwc2_uvc_frame(s->segment, s->seg_frames, len);
    }
    g_byte_array_set_size(s->frame, 0);
}

/* One payload: its header, then picture data up to the end-of-frame bit. */
static void uvc_payload(DWC2UVCHost *s)
{
    const uint8_t *p = s->payload;
    int len = s->payload_len, hlen;

    if (!len) {
        return;
    }
    s->payloads++;
    hlen = p[0];
    trace_dwc2_uvc_payload(len, p[0], len > 1 ? p[1] : 0);
    if (len < 2 || hlen < 2 || hlen > 12 || hlen > len) {
        s->bad_payloads++;
        trace_dwc2_uvc_bad_payload(len, p[0], len > 1 ? p[1] : 0);
        g_byte_array_set_size(s->frame, 0);
        return;
    }
    if (p[1] & UVC_BH_ERR) {
        s->dropped++;
        g_byte_array_set_size(s->frame, 0);
        s->sync = false;
        return;
    }
    if (s->fid >= 0 && (p[1] & UVC_BH_FID) != s->fid) {
        /*
         * The frame bit toggled: a new frame begins. Like the Linux host
         * driver, take it as the end of the last one even without its
         * end-of-frame bit, which the 3.10 gadget leaves out of bulk frames.
         */
        if (s->sync && s->frame->len) {
            s->eof_less++;
            uvc_frame_done(s);
        }
        g_byte_array_set_size(s->frame, 0);
        s->sync = true;
    }
    s->fid = p[1] & UVC_BH_FID;

    if (s->sync && s->frame->len + len - hlen <= UVC_FRAME_MAX) {
        g_byte_array_append(s->frame, p + hlen, len - hlen);
    }
    if (p[1] & UVC_BH_EOF) {
        /*
         * The stream is taken from the first frame boundary on, so a frame
         * the gadget was part way through when the host connected, or of the
         * format before this one, is not mistaken for a picture.
         */
        if (s->sync) {
            uvc_frame_done(s);
        }
        s->sync = true;
        g_byte_array_set_size(s->frame, 0);
    }
}

static void uvc_poll(DWC2UVCHost *s)
{
    uint8_t pkt[1024];
    int budget, r;

    for (budget = UVC_POLL_PACKETS; budget > 0; budget--) {
        r = dwc2_dev_in(s->dwc2, s->ep_in, pkt, MIN(s->mps_in, sizeof(pkt)));
        if (r < 0) {
            break;
        }
        if (s->payload_len + r > UVC_PAYLOAD_MAX) {
            s->bad_payloads++;
            s->payload_len = 0;
        }
        memcpy(s->payload + s->payload_len, pkt, r);
        s->payload_len += r;

        /*
         * A payload ends on a short packet, or where the committed maximum
         * says it must.
         */
        if (r < s->mps_in || (s->max_payload &&
                              s->payload_len >= s->max_payload)) {
            uvc_payload(s);
            s->payload_len = 0;
            if (s->seg_frames >= s->frames_per_format) {
                uvc_next_segment(s);
                return;
            }
        }
    }
}

static void uvc_tick(void *opaque)
{
    DWC2UVCHost *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int r;

    switch (s->state) {
    case UVC_IDLE:
    case UVC_FAILED:
    case UVC_DONE:
        return;

    case UVC_DEBOUNCE:
        if (now >= s->wait_until) {
            dwc2_dev_bus_reset(s->dwc2);
            uvc_set_state(s, UVC_RESET);
            s->wait_until = now + UVC_RESET_NS;
        }
        break;

    case UVC_RESET:
        if (now >= s->wait_until) {
            if (dwc2_dev_reset_done(s->dwc2) < 0) {
                uvc_set_state(s, UVC_IDLE);
                return;
            }
            uvc_set_state(s, UVC_RECOVERY);
            s->wait_until = now + UVC_RECOVERY_NS;
        }
        break;

    case UVC_RECOVERY:
        if (now >= s->wait_until) {
            s->ep0_mps = 64;
            s->step = STEP_DEVICE_EP0;
            uvc_start_step(s, now);
            uvc_set_state(s, UVC_ENUM);
        }
        break;

    case UVC_ENUM:
        if (now < s->wait_until) {
            break;
        }
        r = uvc_control(s);
        if (!r) {
            if (now > s->ctl_deadline) {
                if (s->step < STEP_PROBE_SET) {
                    uvc_retry(s, now, "timeout");
                } else if (!uvc_negotiate_retry(s, now)) {
                    uvc_fail(s, "timeout in the stream negotiation");
                }
            }
            break;
        }
        uvc_step_done(s, now, r < 0 ? r : 0);
        break;

    case UVC_STREAMING:
        uvc_poll(s);
        break;
    }

    if (s->state != UVC_FAILED && s->state != UVC_DONE) {
        timer_mod(s->timer, now + (s->state == UVC_STREAMING ?
                                   UVC_STREAM_TICK_NS : UVC_TICK_NS));
    }
}

static void uvc_connect(void *opaque, bool on)
{
    DWC2UVCHost *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    uvc_close_segment(s);
    s->payload_len = 0;
    if (!on) {
        timer_del(s->timer);
        uvc_set_state(s, UVC_IDLE);
        return;
    }
    s->attempts = 0;
    s->negotiate_retries = 0;
    s->segment = 0;
    uvc_set_state(s, UVC_DEBOUNCE);
    s->wait_until = now + UVC_DEBOUNCE_NS;
    timer_mod(s->timer, now + UVC_TICK_NS);
}

static const DWC2PeerOps uvc_peer_ops = {
    .connect = uvc_connect,
};

static char *uvc_get_state(Object *obj, Error **errp)
{
    return g_strdup(uvc_state_name[DWC2_UVC_HOST(obj)->state]);
}

static int uvc_first_dwc2(Object *obj, void *opaque)
{
    Object **best = opaque;
    g_autofree char *path = NULL;
    g_autofree char *best_path = NULL;

    if (!object_dynamic_cast(obj, TYPE_DWC2_USB)) {
        return 0;
    }
    if (*best) {
        path = object_get_canonical_path(obj);
        best_path = object_get_canonical_path(*best);
        if (strcmp(path, best_path) >= 0) {
            return 0;
        }
    }
    *best = obj;
    return 0;
}

static bool uvc_parse_formats(DWC2UVCHost *s, Error **errp)
{
    g_auto(GStrv) names = g_strsplit(s->format_list, "+", -1);
    int i;

    for (i = 0; names[i]; i++) {
        if (i == UVC_MAX_SEGMENTS) {
            error_setg(errp, "x-format: at most %d formats", UVC_MAX_SEGMENTS);
            return false;
        }
        if (!strcmp(names[i], "mjpeg")) {
            s->wanted[i] = FMT_MJPEG;
        } else if (!strcmp(names[i], "h264")) {
            s->wanted[i] = FMT_H264;
        } else {
            error_setg(errp, "x-format: '%s' is not mjpeg or h264", names[i]);
            return false;
        }
    }
    s->n_wanted = i;
    return true;
}

static void dwc2_uvc_host_realize(DeviceState *dev, Error **errp)
{
    DWC2UVCHost *s = DWC2_UVC_HOST(dev);

    if (!s->format_list) {
        s->format_list = g_strdup("mjpeg");
    }
    if (!uvc_parse_formats(s, errp)) {
        return;
    }
    if (!s->frames_per_format || !s->frame_index) {
        error_setg(errp, "x-frames and x-frame must be at least 1");
        return;
    }

    /* Without 'controller', the first DWC2 by QOM path (OTG0 on the A1). */
    if (!s->dwc2) {
        Object *o = NULL;

        object_child_foreach_recursive(object_get_root(), uvc_first_dwc2, &o);
        if (!o) {
            error_setg(errp, "%s: no %s to attach to", TYPE_DWC2_UVC_HOST,
                       TYPE_DWC2_USB);
            return;
        }
        s->dwc2 = DWC2_USB(o);
    }

    object_property_add_uint32_ptr(OBJECT(dev), "frames", &s->total_frames,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(dev), "dropped", &s->dropped,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(dev), "bad-payloads",
                                   &s->bad_payloads, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(dev), "payloads", &s->payloads,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(OBJECT(dev), "eof-less-frames",
                                   &s->eof_less, OBJ_PROP_FLAG_READ);
    s->frame = g_byte_array_new();
    s->out_fd = -1;
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uvc_tick, s);
    s->state = UVC_IDLE;
    dwc2_dev_set_peer(s->dwc2, &uvc_peer_ops, s);
}

static void dwc2_uvc_host_unrealize(DeviceState *dev)
{
    DWC2UVCHost *s = DWC2_UVC_HOST(dev);

    dwc2_dev_set_peer(s->dwc2, NULL, NULL);
    timer_free(s->timer);
    uvc_close_segment(s);
    g_byte_array_free(s->frame, true);
}

static const VMStateDescription vmstate_dwc2_uvc_host = {
    .name = TYPE_DWC2_UVC_HOST,
    .unmigratable = 1,
};

static const Property dwc2_uvc_host_properties[] = {
    DEFINE_PROP_LINK("controller", DWC2UVCHost, dwc2, TYPE_DWC2_USB,
                     DWC2State *),
    DEFINE_PROP_STRING("x-out", DWC2UVCHost, out_dir),
    DEFINE_PROP_STRING("x-format", DWC2UVCHost, format_list),
    DEFINE_PROP_UINT32("x-frames", DWC2UVCHost, frames_per_format, 5),
    DEFINE_PROP_UINT8("x-frame", DWC2UVCHost, frame_index, 1),
};

static void dwc2_uvc_host_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "USB host with a UVC driver on the DWC2 device port";
    dc->realize = dwc2_uvc_host_realize;
    dc->unrealize = dwc2_uvc_host_unrealize;
    dc->vmsd = &vmstate_dwc2_uvc_host;
    device_class_set_props(dc, dwc2_uvc_host_properties);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    object_class_property_add_str(klass, "state", uvc_get_state, NULL);
}

static const TypeInfo dwc2_uvc_host_info = {
    .name          = TYPE_DWC2_UVC_HOST,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(DWC2UVCHost),
    .class_init    = dwc2_uvc_host_class_init,
};

static void dwc2_uvc_host_register_types(void)
{
    type_register_static(&dwc2_uvc_host_info);
}

type_init(dwc2_uvc_host_register_types)
