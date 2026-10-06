/*
 * USB host with a CDC-NCM class driver, cabled to the DWC2 device port
 *
 * Enumerates the gadget on the DWC2's device side the way a PC would and
 * passes NCM datagrams to and from a QEMU network backend, so a guest
 * running a USB network gadget (g_ncm) can be reached through -netdev as
 * if its USB port were plugged into that machine:
 *
 *   -netdev user,id=u0 -device dwc2-ncm-host,netdev=u0
 *
 * The data path is a plain wire: frames from the gadget go to the backend
 * as they are, and each frame from the backend goes out as one NTB16.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "hcd-dwc2.h"
#include "trace.h"

#define TYPE_DWC2_NCM_HOST "dwc2-ncm-host"
OBJECT_DECLARE_SIMPLE_TYPE(DWC2NCMHost, DWC2_NCM_HOST)

/* Bus timing (virtual time) */
#define NCM_TICK_NS             (1 * SCALE_MS)  /* one pass over the device */
#define NCM_DEBOUNCE_NS         (100 * SCALE_MS)
#define NCM_RESET_NS            (10 * SCALE_MS)
#define NCM_RECOVERY_NS         (10 * SCALE_MS)
#define NCM_SET_ADDRESS_NS      (2 * SCALE_MS)
#define NCM_CONTROL_TIMEOUT_NS  (5 * NANOSECONDS_PER_SECOND)
#define NCM_ATTEMPTS            5

#define NCM_ADDRESS             1
#define NCM_CONFIG_MAX          512
#define NCM_NTB_IN_MAX          16384
#define NCM_NTB_OUT_MAX         2048
#define NCM_NOTIFY_TICKS        8

#define USB_CDC_SUBCLASS_NCM    0x0d
#define USB_CDC_GET_NTB_PARAMETERS  0x80
#define USB_CDC_SET_NTB_FORMAT      0x84
#define USB_CDC_SET_NTB_INPUT_SIZE  0x86
#define USB_CDC_SET_CRC_MODE        0x8a
#define USB_DT_CS_INTERFACE         0x24
#define USB_CDC_NCM_TYPE            0x1a
#define USB_CDC_NCM_NCAP_CRC_MODE   0x10
#define USB_CDC_NCM_NTB32_SUPPORTED 0x02

#define NCM_NTH16_SIGN          0x484d434e      /* "NCMH" */
#define NCM_NDP16_NOCRC_SIGN    0x304d434e      /* "NCM0" */
#define NCM_NDP16_CRC_SIGN      0x314d434e      /* "NCM1" */
#define NCM_NTH16_LEN           12
#define NCM_NDP16_LEN           16              /* header, one entry, end */

typedef enum {
    NCM_IDLE,           /* nothing on the port */
    NCM_DEBOUNCE,       /* pull-up seen, let it settle */
    NCM_RESET,          /* driving bus reset */
    NCM_RECOVERY,       /* reset recovery */
    NCM_ENUM,           /* control transfers */
    NCM_RUNNING,        /* data interface active */
    NCM_FAILED,         /* gave up until the next attach */
} NCMState;

static const char *const ncm_state_name[] = {
    [NCM_IDLE] = "idle",
    [NCM_DEBOUNCE] = "debounce",
    [NCM_RESET] = "reset",
    [NCM_RECOVERY] = "recovery",
    [NCM_ENUM] = "enumerating",
    [NCM_RUNNING] = "running",
    [NCM_FAILED] = "failed",
};

/* The enumeration, one control transfer per step. */
typedef enum {
    STEP_DEVICE_EP0,    /* GET_DESCRIPTOR(DEVICE, 64) for bMaxPacketSize0 */
    STEP_SET_ADDRESS,
    STEP_DEVICE,
    STEP_CONFIG_HEAD,   /* first 9 bytes, for wTotalLength */
    STEP_CONFIG,
    STEP_SET_CONFIG,
    STEP_NTB_PARAMS,
    STEP_CRC_MODE,      /* CRC off, sent when the function can do CRC */
    STEP_NTB_FORMAT,    /* NTB16, sent when NTB32 is also offered */
    STEP_NTB_INPUT,     /* SET_NTB_INPUT_SIZE, a control write with data */
    STEP_SET_ALT,       /* data interface to the altsetting with the pipes */
    STEP_DONE,
} NCMStep;

typedef enum {
    CTL_SETUP,
    CTL_DATA,
    CTL_STATUS,
} NCMStage;

struct DWC2NCMHost {
    DeviceState parent_obj;

    DWC2State *dwc2;
    NICConf conf;
    NICState *nic;
    QEMUTimer *timer;

    NCMState state;
    int64_t wait_until;
    int attempts;

    /* The control transfer in flight */
    NCMStep step;
    NCMStage stage;
    uint8_t setup[8];
    uint8_t ctl_buf[NCM_CONFIG_MAX + 64];
    int ctl_len;
    int ctl_pos;
    int64_t ctl_deadline;

    /* From the descriptors and the NTB parameters */
    int ep0_mps;
    uint16_t vid, pid;
    uint16_t cfg_total;
    uint8_t cfg_value;
    int ctrl_intf, data_intf, data_alt;
    uint8_t ncm_caps;           /* bmNetworkCapabilities */
    uint16_t ntb_formats;       /* bmNtbFormatsSupported */
    int ep_in, ep_out, ep_notify;
    int mps_in, mps_out, mps_notify;
    uint32_t ntb_in_max;
    uint32_t ntb_out_max;
    uint16_t out_div, out_rem, out_align;

    /* Data path */
    uint8_t rx[NCM_NTB_IN_MAX + 1024];
    int rx_len;
    uint8_t tx[NCM_NTB_OUT_MAX];
    int tx_len;
    int tx_pos;
    bool tx_zlp;
    uint16_t tx_seq;
    int notify_ticks;
};

static void ncm_set_state(DWC2NCMHost *s, NCMState state)
{
    if (s->state != state) {
        s->state = state;
        trace_dwc2_ncm_state(ncm_state_name[state]);
    }
}

static void ncm_setup(DWC2NCMHost *s, uint8_t type, uint8_t req,
                      uint16_t value, uint16_t index, uint16_t len)
{
    s->setup[0] = type;
    s->setup[1] = req;
    stw_le_p(s->setup + 2, value);
    stw_le_p(s->setup + 4, index);
    stw_le_p(s->setup + 6, len);
    s->ctl_len = len;
}

static void ncm_start_step(DWC2NCMHost *s, int64_t now)
{
    switch (s->step) {
    case STEP_DEVICE_EP0:
        ncm_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8,
                  0, 64);
        break;
    case STEP_SET_ADDRESS:
        ncm_setup(s, 0, USB_REQ_SET_ADDRESS, NCM_ADDRESS, 0, 0);
        break;
    case STEP_DEVICE:
        ncm_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8,
                  0, 18);
        break;
    case STEP_CONFIG_HEAD:
        ncm_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIG << 8,
                  0, 9);
        break;
    case STEP_CONFIG:
        ncm_setup(s, USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIG << 8,
                  0, s->cfg_total);
        break;
    case STEP_SET_CONFIG:
        ncm_setup(s, 0, USB_REQ_SET_CONFIGURATION, s->cfg_value, 0, 0);
        break;
    case STEP_NTB_PARAMS:
        ncm_setup(s, USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                  USB_CDC_GET_NTB_PARAMETERS, 0, s->ctrl_intf, 28);
        break;
    case STEP_CRC_MODE:
        ncm_setup(s, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                  USB_CDC_SET_CRC_MODE, 0, s->ctrl_intf, 0);
        break;
    case STEP_NTB_FORMAT:
        ncm_setup(s, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                  USB_CDC_SET_NTB_FORMAT, 0, s->ctrl_intf, 0);
        break;
    case STEP_NTB_INPUT:
        ncm_setup(s, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
                  USB_CDC_SET_NTB_INPUT_SIZE, 0, s->ctrl_intf, 4);
        stl_le_p(s->ctl_buf, s->ntb_in_max);
        break;
    case STEP_SET_ALT:
        ncm_setup(s, USB_RECIP_INTERFACE, USB_REQ_SET_INTERFACE, s->data_alt,
                  s->data_intf, 0);
        break;
    default:
        g_assert_not_reached();
    }
    s->stage = CTL_SETUP;
    s->ctl_pos = 0;
    s->ctl_deadline = now + NCM_CONTROL_TIMEOUT_NS;
}

/*
 * Run the control transfer as far as the device lets it: 1 when done,
 * 0 while the device NAKs, a negative handshake when it stalls.
 */
static int ncm_control(DWC2NCMHost *s)
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

static bool ncm_parse_config(DWC2NCMHost *s, const uint8_t *d, int len)
{
    int intf = -1, alt = 0, class = 0, i;

    s->cfg_value = d[5];
    s->ctrl_intf = s->data_intf = -1;
    s->ep_in = s->ep_out = s->ep_notify = 0;
    s->ncm_caps = 0;

    for (i = 0; i + 2 <= len && d[i] >= 2 && i + d[i] <= len; i += d[i]) {
        const uint8_t *p = d + i;

        if (p[1] == USB_DT_INTERFACE && p[0] >= 9) {
            intf = p[2];
            alt = p[3];
            class = p[5];
            if (class == USB_CLASS_COMM && p[6] == USB_CDC_SUBCLASS_NCM &&
                s->ctrl_intf < 0) {
                s->ctrl_intf = intf;
            }
            if (class == USB_CLASS_CDC_DATA && alt && s->data_intf < 0) {
                s->data_intf = intf;
                s->data_alt = alt;
            }
        } else if (p[1] == USB_DT_CS_INTERFACE && p[0] >= 6 &&
                   p[2] == USB_CDC_NCM_TYPE && intf == s->ctrl_intf) {
            s->ncm_caps = p[5];
        } else if (p[1] == USB_DT_ENDPOINT && p[0] >= 7) {
            int addr = p[2];
            int type = p[3] & 3;
            int mps = lduw_le_p(p + 4) & 0x7ff;

            if (intf == s->ctrl_intf && class == USB_CLASS_COMM &&
                type == USB_ENDPOINT_XFER_INT && (addr & USB_DIR_IN)) {
                s->ep_notify = addr & 0xf;
                s->mps_notify = mps;
            }
            if (intf == s->data_intf && alt == s->data_alt &&
                type == USB_ENDPOINT_XFER_BULK) {
                if (addr & USB_DIR_IN) {
                    s->ep_in = addr & 0xf;
                    s->mps_in = mps;
                } else {
                    s->ep_out = addr & 0xf;
                    s->mps_out = mps;
                }
            }
        }
    }
    return s->ctrl_intf >= 0 && s->data_intf >= 0 && s->ep_in && s->ep_out &&
           s->mps_in && s->mps_out;
}

static void ncm_ntb_params(DWC2NCMHost *s, const uint8_t *p, int len)
{
    s->ntb_in_max = NCM_NTB_IN_MAX;
    s->ntb_out_max = NCM_NTB_OUT_MAX;
    s->out_div = 4;
    s->out_rem = 0;
    s->out_align = 4;
    s->ntb_formats = 1;
    if (len < 28) {
        return;
    }
    s->ntb_formats = lduw_le_p(p + 2);
    s->ntb_in_max = MIN(ldl_le_p(p + 4), NCM_NTB_IN_MAX);
    s->ntb_out_max = MIN(ldl_le_p(p + 16), NCM_NTB_OUT_MAX);
    if (lduw_le_p(p + 20)) {
        s->out_div = lduw_le_p(p + 20);
        s->out_rem = lduw_le_p(p + 22) % s->out_div;
    }
    if (lduw_le_p(p + 24) >= 4) {
        s->out_align = lduw_le_p(p + 24);
    }
}

/* Start over from a bus reset, a few times before giving up. */
static void ncm_retry(DWC2NCMHost *s, int64_t now, const char *why)
{
    if (++s->attempts >= NCM_ATTEMPTS) {
        warn_report("%s: no NCM gadget after %d attempts (%s at step %d)",
                    TYPE_DWC2_NCM_HOST, s->attempts, why, s->step);
        ncm_set_state(s, NCM_FAILED);
        return;
    }
    dwc2_dev_bus_reset(s->dwc2);
    ncm_set_state(s, NCM_RESET);
    s->wait_until = now + NCM_RESET_NS;
}

/* The control transfer for s->step finished with handshake r. */
static void ncm_step_done(DWC2NCMHost *s, int64_t now, int r)
{
    const uint8_t *b = s->ctl_buf;
    int len = s->ctl_pos;

    trace_dwc2_ncm_control(s->setup[0], s->setup[1], lduw_le_p(s->setup + 2),
                           lduw_le_p(s->setup + 4), s->ctl_len, r);

    /* The NTB and CRC requests may be refused, nothing else. */
    if (r < 0 && (s->step < STEP_NTB_PARAMS || s->step == STEP_SET_ALT)) {
        ncm_retry(s, now, "stall");
        return;
    }

    switch (s->step) {
    case STEP_DEVICE_EP0:
        if (len < 8 || b[1] != USB_DT_DEVICE ||
            (b[7] != 8 && b[7] != 16 && b[7] != 32 && b[7] != 64)) {
            ncm_retry(s, now, "device descriptor");
            return;
        }
        s->ep0_mps = b[7];
        break;
    case STEP_SET_ADDRESS:
        s->wait_until = now + NCM_SET_ADDRESS_NS;
        break;
    case STEP_DEVICE:
        if (len < 18) {
            ncm_retry(s, now, "device descriptor");
            return;
        }
        s->vid = lduw_le_p(b + 8);
        s->pid = lduw_le_p(b + 10);
        break;
    case STEP_CONFIG_HEAD:
        if (len < 9 || b[1] != USB_DT_CONFIG) {
            ncm_retry(s, now, "configuration descriptor");
            return;
        }
        s->cfg_total = MIN(lduw_le_p(b + 2), NCM_CONFIG_MAX);
        break;
    case STEP_CONFIG:
        if (!ncm_parse_config(s, b, len)) {
            warn_report("%s: %04x:%04x is not an NCM device",
                        TYPE_DWC2_NCM_HOST, s->vid, s->pid);
            ncm_set_state(s, NCM_FAILED);
            return;
        }
        break;
    case STEP_NTB_PARAMS:
        ncm_ntb_params(s, b, r < 0 ? 0 : len);
        break;
    default:
        break;
    }

    /*
     * Requests a Linux host makes only for what the function offers. The
     * 3.10 f_ncm sets its NDP signature only on SET_CRC_MODE, so without
     * that request it sends and expects a zero signature.
     */
    do {
        s->step++;
    } while ((s->step == STEP_CRC_MODE &&
              !(s->ncm_caps & USB_CDC_NCM_NCAP_CRC_MODE)) ||
             (s->step == STEP_NTB_FORMAT &&
              !(s->ntb_formats & USB_CDC_NCM_NTB32_SUPPORTED)));

    if (s->step == STEP_DONE) {
        info_report("%s: %04x:%04x configured, NCM link up",
                    TYPE_DWC2_NCM_HOST, s->vid, s->pid);
        s->rx_len = 0;
        s->tx_len = s->tx_pos = 0;
        s->notify_ticks = 0;
        ncm_set_state(s, NCM_RUNNING);
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
        return;
    }
    ncm_start_step(s, now);
}

static void ncm_rx_ntb(DWC2NCMHost *s, const uint8_t *b, int len)
{
    int ndp, frames = 0, guard;

    if (len < NCM_NTH16_LEN || ldl_le_p(b) != NCM_NTH16_SIGN ||
        lduw_le_p(b + 4) != NCM_NTH16_LEN) {
        trace_dwc2_ncm_rx_bad("NTH16", len, ldl_le_p(b), ldl_le_p(b + 4),
                              ldl_le_p(b + 8));
        return;
    }
    len = MIN(len, lduw_le_p(b + 8));
    ndp = lduw_le_p(b + 10);

    for (guard = 0; ndp && guard < 16; guard++) {
        uint32_t sig;
        int end, i;

        if (ndp + 8 > len) {
            break;
        }
        sig = ldl_le_p(b + ndp);
        if (sig != NCM_NDP16_NOCRC_SIGN && sig != NCM_NDP16_CRC_SIGN) {
            trace_dwc2_ncm_rx_bad("NDP16", ndp, sig, ldl_le_p(b + ndp + 4),
                                  ldl_le_p(b + ndp + 8));
            break;
        }
        end = MIN(ndp + lduw_le_p(b + ndp + 4), len);
        for (i = ndp + 8; i + 4 <= end; i += 4) {
            int idx = lduw_le_p(b + i);
            int dlen = lduw_le_p(b + i + 2);

            if (!idx || !dlen) {
                break;
            }
            if (sig == NCM_NDP16_CRC_SIGN) {
                dlen -= 4;
            }
            if (dlen >= 14 && idx + dlen <= len) {
                qemu_send_packet(qemu_get_queue(s->nic), b + idx, dlen);
                frames++;
            } else {
                trace_dwc2_ncm_rx_bad("datagram", len, idx, dlen, i);
            }
        }
        ndp = lduw_le_p(b + ndp + 6);
    }
    trace_dwc2_ncm_rx(len, frames);
}

static void ncm_poll(DWC2NCMHost *s)
{
    int budget, n, r;

    /*
     * Device to host: an IN transfer ends on a short packet or at the size
     * agreed with SET_NTB_INPUT_SIZE (an NTB of exactly that size has no
     * terminating ZLP).
     */
    for (budget = 64; budget > 0; budget--) {
        r = dwc2_dev_in(s->dwc2, s->ep_in, s->rx + s->rx_len, s->mps_in);
        if (r < 0) {
            break;
        }
        s->rx_len += r;
        if (r < s->mps_in || s->rx_len >= s->ntb_in_max) {
            ncm_rx_ntb(s, s->rx, s->rx_len);
            s->rx_len = 0;
        } else if (s->rx_len + s->mps_in > sizeof(s->rx)) {
            s->rx_len = 0;
        }
    }

    /*
     * Host to device: one NTB at a time, ended by a ZLP when it fills its
     * last packet.
     */
    while (s->tx_len) {
        n = MIN(s->mps_out, s->tx_len - s->tx_pos);
        r = dwc2_dev_out(s->dwc2, s->ep_out, s->tx + s->tx_pos, n);
        if (r < 0) {
            break;
        }
        s->tx_pos += n;
        if (!n) {
            s->tx_zlp = false;
        }
        if (s->tx_pos == s->tx_len && !s->tx_zlp) {
            s->tx_len = s->tx_pos = 0;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
    }

    /* Drain the notifications so the function's notify state machine runs. */
    if (s->ep_notify && ++s->notify_ticks >= NCM_NOTIFY_TICKS) {
        uint8_t note[64];

        s->notify_ticks = 0;
        dwc2_dev_in(s->dwc2, s->ep_notify, note,
                    MIN(sizeof(note), s->mps_notify));
    }
}

static void ncm_tick(void *opaque)
{
    DWC2NCMHost *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int r;

    switch (s->state) {
    case NCM_IDLE:
    case NCM_FAILED:
        return;

    case NCM_DEBOUNCE:
        if (now >= s->wait_until) {
            dwc2_dev_bus_reset(s->dwc2);
            ncm_set_state(s, NCM_RESET);
            s->wait_until = now + NCM_RESET_NS;
        }
        break;

    case NCM_RESET:
        if (now >= s->wait_until) {
            if (dwc2_dev_reset_done(s->dwc2) < 0) {
                ncm_set_state(s, NCM_IDLE);
                return;
            }
            ncm_set_state(s, NCM_RECOVERY);
            s->wait_until = now + NCM_RECOVERY_NS;
        }
        break;

    case NCM_RECOVERY:
        if (now >= s->wait_until) {
            s->ep0_mps = 64;
            s->step = STEP_DEVICE_EP0;
            ncm_start_step(s, now);
            ncm_set_state(s, NCM_ENUM);
        }
        break;

    case NCM_ENUM:
        if (now < s->wait_until) {
            break;
        }
        r = ncm_control(s);
        if (!r) {
            if (now > s->ctl_deadline) {
                ncm_retry(s, now, "timeout");
            }
            break;
        }
        ncm_step_done(s, now, r < 0 ? r : 0);
        break;

    case NCM_RUNNING:
        ncm_poll(s);
        break;
    }

    if (s->state != NCM_FAILED) {
        timer_mod(s->timer, now + NCM_TICK_NS);
    }
}

static void ncm_connect(void *opaque, bool on)
{
    DWC2NCMHost *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->rx_len = 0;
    s->tx_len = s->tx_pos = 0;
    if (!on) {
        timer_del(s->timer);
        ncm_set_state(s, NCM_IDLE);
        return;
    }
    s->attempts = 0;
    ncm_set_state(s, NCM_DEBOUNCE);
    s->wait_until = now + NCM_DEBOUNCE_NS;
    timer_mod(s->timer, now + NCM_TICK_NS);
}

static const DWC2PeerOps ncm_peer_ops = {
    .connect = ncm_connect,
};

static bool ncm_can_receive(NetClientState *nc)
{
    DWC2NCMHost *s = qemu_get_nic_opaque(nc);

    return s->state != NCM_RUNNING || !s->tx_len;
}

static ssize_t ncm_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    DWC2NCMHost *s = qemu_get_nic_opaque(nc);
    int off, ndp, total;

    if (s->state != NCM_RUNNING) {
        return size;
    }
    if (s->tx_len) {
        return 0;
    }

    /* The datagram goes where wNdpOutDivisor and the remainder want it. */
    for (off = NCM_NTH16_LEN; off % s->out_div != s->out_rem; off++) {
    }
    ndp = QEMU_ALIGN_UP(off + size, s->out_align);
    total = ndp + NCM_NDP16_LEN;
    if (size < 14 || total > s->ntb_out_max) {
        return size;
    }

    memset(s->tx, 0, total);
    stl_le_p(s->tx, NCM_NTH16_SIGN);
    stw_le_p(s->tx + 4, NCM_NTH16_LEN);
    stw_le_p(s->tx + 6, s->tx_seq++);
    stw_le_p(s->tx + 8, total);
    stw_le_p(s->tx + 10, ndp);
    memcpy(s->tx + off, buf, size);
    stl_le_p(s->tx + ndp, NCM_NDP16_NOCRC_SIGN);
    stw_le_p(s->tx + ndp + 4, NCM_NDP16_LEN);
    stw_le_p(s->tx + ndp + 8, off);
    stw_le_p(s->tx + ndp + 10, size);

    s->tx_len = total;
    s->tx_pos = 0;
    s->tx_zlp = !(total % s->mps_out);
    trace_dwc2_ncm_tx(size);
    return size;
}

static NetClientInfo ncm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ncm_can_receive,
    .receive = ncm_receive,
};

static char *ncm_get_state(Object *obj, Error **errp)
{
    return g_strdup(ncm_state_name[DWC2_NCM_HOST(obj)->state]);
}

static int ncm_first_dwc2(Object *obj, void *opaque)
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

static void dwc2_ncm_host_realize(DeviceState *dev, Error **errp)
{
    DWC2NCMHost *s = DWC2_NCM_HOST(dev);

    /* Without 'controller', the first DWC2 by QOM path (OTG0 on the A1). */
    if (!s->dwc2) {
        Object *o = NULL;

        object_child_foreach_recursive(object_get_root(), ncm_first_dwc2, &o);
        if (!o) {
            error_setg(errp, "%s: no %s to attach to", TYPE_DWC2_NCM_HOST,
                       TYPE_DWC2_USB);
            return;
        }
        s->dwc2 = DWC2_USB(o);
    }

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&ncm_net_info, &s->conf, TYPE_DWC2_NCM_HOST,
                          dev->id, &dev->mem_reentrancy_guard, s);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ncm_tick, s);
    s->state = NCM_IDLE;
    dwc2_dev_set_peer(s->dwc2, &ncm_peer_ops, s);
}

static void dwc2_ncm_host_unrealize(DeviceState *dev)
{
    DWC2NCMHost *s = DWC2_NCM_HOST(dev);

    dwc2_dev_set_peer(s->dwc2, NULL, NULL);
    timer_free(s->timer);
    qemu_del_nic(s->nic);
}

static const VMStateDescription vmstate_dwc2_ncm_host = {
    .name = TYPE_DWC2_NCM_HOST,
    .unmigratable = 1,
};

static const Property dwc2_ncm_host_properties[] = {
    DEFINE_NIC_PROPERTIES(DWC2NCMHost, conf),
    DEFINE_PROP_LINK("controller", DWC2NCMHost, dwc2, TYPE_DWC2_USB,
                     DWC2State *),
};

static void dwc2_ncm_host_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "USB host with a CDC-NCM driver on the DWC2 device port";
    dc->realize = dwc2_ncm_host_realize;
    dc->unrealize = dwc2_ncm_host_unrealize;
    dc->vmsd = &vmstate_dwc2_ncm_host;
    device_class_set_props(dc, dwc2_ncm_host_properties);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    object_class_property_add_str(klass, "state", ncm_get_state, NULL);
}

static const TypeInfo dwc2_ncm_host_info = {
    .name          = TYPE_DWC2_NCM_HOST,
    .parent        = TYPE_DEVICE,
    .instance_size = sizeof(DWC2NCMHost),
    .class_init    = dwc2_ncm_host_class_init,
};

static void dwc2_ncm_host_register_types(void)
{
    type_register_static(&dwc2_ncm_host_info);
}

type_init(dwc2_ncm_host_register_types)
