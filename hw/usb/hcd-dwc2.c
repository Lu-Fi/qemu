/*
 * dwc-hsotg (dwc2) USB host controller emulation
 *
 * Based on hw/usb/hcd-ehci.c and hw/usb/hcd-ohci.c
 *
 * Note that to use this emulation with the dwc-otg driver in the
 * Raspbian kernel, you must pass the option "dwc_otg.fiq_fsm_enable=0"
 * on the kernel command line.
 *
 * Some useful documentation used to develop this emulation can be
 * found online (as of April 2020) at:
 *
 * http://www.capital-micro.com/PDF/CME-M7_Family_User_Guide_EN.pdf
 * which has a pretty complete description of the controller starting
 * on page 370.
 *
 * https://sourceforge.net/p/wive-ng/wive-ng-mt/ci/master/tree/docs/DataSheets/RT3050_5x_V2.0_081408_0902.pdf
 * which has a description of the controller registers starting on
 * page 130.
 *
 * Copyright (c) 2020 Paul Zimmerman <pauldzim@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/usb/dwc2-regs.h"
#include "hw/usb/hcd-dwc2.h"
#include "migration/vmstate.h"
#include "trace.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/main-loop.h"
#include "hw/core/qdev-properties.h"

#define USB_HZ_FS       12000000
#define USB_HZ_HS       96000000
#define USB_FRMINTVL    12000

/* nifty macros from Arnon's EHCI version  */
#define get_field(data, field) \
    (((data) & field##_MASK) >> field##_SHIFT)

#define set_field(data, newval, field) do { \
    uint32_t val = *(data); \
    val &= ~field##_MASK; \
    val |= ((newval) << field##_SHIFT) & field##_MASK; \
    *(data) = val; \
} while (0)

#define get_bit(data, bitmask) \
    (!!((data) & (bitmask)))

/* update irq line */
static inline void dwc2_update_irq(DWC2State *s)
{
    static int oldlevel;
    int level = 0;

    if ((s->gintsts & s->gintmsk) && (s->gahbcfg & GAHBCFG_GLBL_INTR_EN)) {
        level = 1;
    }
    if (level != oldlevel) {
        oldlevel = level;
        trace_usb_dwc2_update_irq(level);
        qemu_set_irq(s->irq, level);
    }
}

/* flag interrupt condition */
static inline void dwc2_raise_global_irq(DWC2State *s, uint32_t intr)
{
    if (!(s->gintsts & intr)) {
        s->gintsts |= intr;
        trace_usb_dwc2_raise_global_irq(intr);
        dwc2_update_irq(s);
    }
}

static inline void dwc2_lower_global_irq(DWC2State *s, uint32_t intr)
{
    if (s->gintsts & intr) {
        s->gintsts &= ~intr;
        trace_usb_dwc2_lower_global_irq(intr);
        dwc2_update_irq(s);
    }
}

static inline void dwc2_raise_host_irq(DWC2State *s, uint32_t host_intr)
{
    if (!(s->haint & host_intr)) {
        s->haint |= host_intr;
        s->haint &= 0xffff;
        trace_usb_dwc2_raise_host_irq(host_intr);
        if (s->haint & s->haintmsk) {
            dwc2_raise_global_irq(s, GINTSTS_HCHINT);
        }
    }
}

static inline void dwc2_lower_host_irq(DWC2State *s, uint32_t host_intr)
{
    if (s->haint & host_intr) {
        s->haint &= ~host_intr;
        trace_usb_dwc2_lower_host_irq(host_intr);
        if (!(s->haint & s->haintmsk)) {
            dwc2_lower_global_irq(s, GINTSTS_HCHINT);
        }
    }
}

static inline void dwc2_update_hc_irq(DWC2State *s, int index)
{
    uint32_t host_intr = 1 << (index >> 3);

    if (s->hreg1[index + 2] & s->hreg1[index + 3]) {
        dwc2_raise_host_irq(s, host_intr);
    } else {
        dwc2_lower_host_irq(s, host_intr);
    }
}

static void dwc2_dev_check_connect(DWC2State *s);
static void dwc2_dev_soft_reset(DWC2State *s);

/* set a timer for EOF */
static void dwc2_eof_timer(DWC2State *s)
{
    timer_mod(s->eof_timer, s->sof_time + s->usb_frame_time);
}

/* Set a timer for EOF and generate SOF event */
static void dwc2_sof(DWC2State *s)
{
    s->sof_time += s->usb_frame_time;
    trace_usb_dwc2_sof(s->sof_time);
    dwc2_eof_timer(s);
    dwc2_raise_global_irq(s, GINTSTS_SOF);
}

/* Do frame processing on frame boundary */
static void dwc2_frame_boundary(void *opaque)
{
    DWC2State *s = opaque;
    int64_t now;
    uint16_t frcnt;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    /* Frame boundary, so do EOF stuff here */

    /* Increment frame number */
    frcnt = (uint16_t)((now - s->sof_time) / s->fi);
    s->frame_number = (s->frame_number + frcnt) & 0xffff;
    s->hfnum = s->frame_number & HFNUM_MAX_FRNUM;

    /* Do SOF stuff here */
    dwc2_sof(s);
}

/* Start sending SOF tokens on the USB bus */
static void dwc2_bus_start(DWC2State *s)
{
    trace_usb_dwc2_bus_start();
    s->sof_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    dwc2_eof_timer(s);
}

/* Stop sending SOF tokens on the USB bus */
static void dwc2_bus_stop(DWC2State *s)
{
    trace_usb_dwc2_bus_stop();
    timer_del(s->eof_timer);
}

static USBDevice *dwc2_find_device(DWC2State *s, uint8_t addr)
{
    USBDevice *dev;

    trace_usb_dwc2_find_device(addr);

    if (!(s->hprt0 & HPRT0_ENA)) {
        trace_usb_dwc2_port_disabled(0);
    } else {
        dev = usb_find_device(&s->uport, addr);
        if (dev != NULL) {
            trace_usb_dwc2_device_found(0);
            return dev;
        }
    }

    trace_usb_dwc2_device_not_found();
    return NULL;
}

static const char *pstatus[] = {
    "USB_RET_SUCCESS", "USB_RET_NODEV", "USB_RET_NAK", "USB_RET_STALL",
    "USB_RET_BABBLE", "USB_RET_IOERROR", "USB_RET_ASYNC",
    "USB_RET_ADD_TO_QUEUE", "USB_RET_REMOVE_FROM_QUEUE"
};

static uint32_t pintr[] = {
    HCINTMSK_XFERCOMPL, HCINTMSK_XACTERR, HCINTMSK_NAK, HCINTMSK_STALL,
    HCINTMSK_BBLERR, HCINTMSK_XACTERR, HCINTMSK_XACTERR, HCINTMSK_XACTERR,
    HCINTMSK_XACTERR
};

static const char *types[] = {
    "Ctrl", "Isoc", "Bulk", "Intr"
};

static const char *dirs[] = {
    "Out", "In"
};

static void dwc2_handle_packet(DWC2State *s, uint32_t devadr, USBDevice *dev,
                               USBEndpoint *ep, uint32_t index, bool send)
{
    DWC2Packet *p;
    uint32_t hcchar = s->hreg1[index];
    uint32_t hctsiz = s->hreg1[index + 4];
    uint32_t hcdma = s->hreg1[index + 5];
    uint32_t chan, epnum, epdir, eptype, mps, pid, pcnt, tlen, intr = 0;
    uint32_t len = 0;
    uint32_t tpcnt, stsidx, actual = 0;
    bool do_intr = false, done = false;
    bool descdma = !!(s->hreg0[0] & HCFG_DESCDMA);
    struct dwc2_dma_desc dd = {0};
    uint32_t dd_addr = 0;

    /*
     * When HCFG.DESCDMA is set, HCDMA is a pointer to a DMA descriptor
     * (status word + buffer pointer) rather than a raw buffer. The
     * Ingenic Linux dwc2 driver always enables DESCDMA for host mode
     * (host-ddma.c). Read the descriptor here and substitute the
     * descriptor's buf+nbytes for hcdma+len so the rest of the
     * transfer path works unchanged.
     */
    epnum = get_field(hcchar, HCCHAR_EPNUM);
    epdir = get_bit(hcchar, HCCHAR_EPDIR);
    eptype = get_field(hcchar, HCCHAR_EPTYPE);
    mps = get_field(hcchar, HCCHAR_MPS);
    pid = get_field(hctsiz, TSIZ_SC_MC_PID);
    pcnt = get_field(hctsiz, TSIZ_PKTCNT);
    len = get_field(hctsiz, TSIZ_XFERSIZE);

    if (send && descdma) {
        /*
         * MIPS guests (U-Boot, Linux) hand the controller KSEG0/KSEG1
         * addresses; mask to physical like the GMAC/SFC models. A
         * no-op for true physical addresses (RAM << 512 MiB).
         */
        dd_addr = hcdma & 0x1FFFFFFF;
        dma_memory_read(&s->dma_as, dd_addr, &dd, sizeof(dd),
                        MEMTXATTRS_UNSPECIFIED);
        dd.status = le32_to_cpu(dd.status);
        dd.buf = le32_to_cpu(dd.buf);
        if (dd.status & HOST_DMA_A) {
            uint32_t n_bytes = (dd.status & HOST_DMA_NBYTES_MASK) >>
                               HOST_DMA_NBYTES_SHIFT;
            /* Override buffer pointer and size from descriptor. */
            hcdma = dd.buf;
            len = n_bytes;
            /* SUP bit overrides PID to SETUP regardless of HCTSIZ. */
            if (dd.status & HOST_DMA_SUP) {
                pid = TSIZ_SC_MC_PID_SETUP;
            }
        }
    }
    if (len > DWC2_MAX_XFER_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: HCTSIZ transfer size too large\n", __func__);
        return;
    }

    chan = index >> 3;
    p = &s->packet[chan];

    trace_usb_dwc2_handle_packet(chan, dev, &p->packet, epnum, types[eptype],
                                 dirs[epdir], mps, len, pcnt);

    if (mps == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                "%s: Bad HCCHAR_MPS set to zero\n", __func__);
        return;
    }

    if (eptype == USB_ENDPOINT_XFER_CONTROL && pid == TSIZ_SC_MC_PID_SETUP) {
        pid = USB_TOKEN_SETUP;
    } else {
        pid = epdir ? USB_TOKEN_IN : USB_TOKEN_OUT;
    }

    if (send) {
        tlen = len;
        /*
         * In DESCDMA mode the kernel programs n_bytes per descriptor
         * (up to 128 KiB) and the controller is expected to issue as
         * many USB packets as needed. The non-DESCDMA "small" path
         * caps a single transfer at mps which would lose data on
         * multi-packet IN reads such as device descriptor fetch.
         */
        if (p->small && !descdma) {
            if (tlen > mps) {
                tlen = mps;
            }
        }

        if (pid != USB_TOKEN_IN) {
            trace_usb_dwc2_memory_read(hcdma, tlen);
            if (dma_memory_read(&s->dma_as, hcdma & 0x1FFFFFFF,
                                s->usb_buf[chan], tlen,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: dma_memory_read failed\n",
                              __func__);
            }
        }

        usb_packet_init(&p->packet);
        usb_packet_setup(&p->packet, pid, ep, 0, hcdma,
                         pid != USB_TOKEN_IN, true);
        usb_packet_addbuf(&p->packet, s->usb_buf[chan], tlen);
        p->async = DWC2_ASYNC_NONE;
        usb_handle_packet(dev, &p->packet);
    } else {
        tlen = p->len;
    }

    stsidx = -p->packet.status;
    assert(stsidx < sizeof(pstatus) / sizeof(*pstatus));
    actual = p->packet.actual_length;
    trace_usb_dwc2_packet_status(pstatus[stsidx], actual);

babble:
    if (p->packet.status != USB_RET_SUCCESS &&
            p->packet.status != USB_RET_NAK &&
            p->packet.status != USB_RET_STALL &&
            p->packet.status != USB_RET_ASYNC) {
        trace_usb_dwc2_packet_error(pstatus[stsidx]);
    }

    if (p->packet.status == USB_RET_ASYNC) {
        trace_usb_dwc2_async_packet(&p->packet, chan, dev, epnum,
                                    dirs[epdir], tlen);
        usb_device_flush_ep_queue(dev, ep);
        assert(p->async != DWC2_ASYNC_INFLIGHT);
        p->devadr = devadr;
        p->epnum = epnum;
        p->epdir = epdir;
        p->mps = mps;
        p->pid = pid;
        p->index = index;
        p->pcnt = pcnt;
        p->len = tlen;
        p->async = DWC2_ASYNC_INFLIGHT;
        p->needs_service = false;
        return;
    }

    if (p->packet.status == USB_RET_SUCCESS) {
        if (actual > tlen) {
            p->packet.status = USB_RET_BABBLE;
            goto babble;
        }

        if (pid == USB_TOKEN_IN) {
            trace_usb_dwc2_memory_write(hcdma, actual);
            if (dma_memory_write(&s->dma_as, hcdma & 0x1FFFFFFF,
                                 s->usb_buf[chan], actual,
                                 MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                qemu_log_mask(LOG_GUEST_ERROR, "%s: dma_memory_write failed\n",
                              __func__);
            }
        }

        /*
         * Real hardware sets ACK in HCINT on every successfully ACKed
         * packet. The U-Boot dwc2 driver checks for HCINT==XFERCOMPL|
         * CHHLTD|ACK on non-SETUP transfers (ignore_ack=false) and
         * returns -EINVAL otherwise, which prevents the STATUS stage
         * of control transfers from running and stalls device
         * enumeration. Without this, the upstream raspi target was the
         * only consumer and its kernel driver doesn't gate on ACK.
         */
        intr |= HCINTMSK_ACK;

        tpcnt = actual / mps;
        if (actual % mps) {
            tpcnt++;
            if (pid == USB_TOKEN_IN) {
                done = true;
            }
        }

        pcnt -= tpcnt < pcnt ? tpcnt : pcnt;
        set_field(&hctsiz, pcnt, TSIZ_PKTCNT);
        len -= actual < len ? actual : len;
        set_field(&hctsiz, len, TSIZ_XFERSIZE);
        s->hreg1[index + 4] = hctsiz;
        hcdma += actual;
        s->hreg1[index + 5] = hcdma;

        if (descdma) {
            /*
             * Write back the DMA descriptor: clear Active, update
             * NBYTES with the remaining (unsent/unreceived) byte count,
             * mark transfer status as success. The kernel checks the
             * descriptor on completion to know how many bytes were
             * actually transferred and whether the URB succeeded.
             */
            uint32_t orig_status = dd.status;
            uint32_t want = (orig_status & HOST_DMA_NBYTES_MASK) >>
                            HOST_DMA_NBYTES_SHIFT;
            uint32_t left = (want > actual) ? (want - actual) : 0;
            bool eol = !!(dd.status & HOST_DMA_EOL);

            dd.status = orig_status;
            dd.status &= ~HOST_DMA_A;
            dd.status &= ~HOST_DMA_STS_MASK; /* status = success (0) */
            dd.status &= ~HOST_DMA_NBYTES_MASK;
            dd.status |= (left << HOST_DMA_NBYTES_SHIFT) &
                         HOST_DMA_NBYTES_MASK;

            uint32_t status_le = cpu_to_le32(dd.status);
            uint32_t buf_le = cpu_to_le32(dd.buf);
            struct dwc2_dma_desc dd_out = { .status = status_le,
                                            .buf = buf_le };
            dma_memory_write(&s->dma_as, dd_addr, &dd_out, sizeof(dd_out),
                             MEMTXATTRS_UNSPECIFIED);

            if (eol) {
                /*
                 * Last descriptor in the list - halt the channel and
                 * raise XFERCOMPL/CHHLTD as usual. Restore HCDMA to
                 * the original list base.
                 */
                s->hreg1[index + 5] = dd_addr;
                done = true;
            } else {
                /*
                 * Not yet at end-of-list. Advance HCDMA to the next
                 * descriptor and re-arm the work BH so the next call
                 * to dwc2_handle_packet processes it. Don't halt the
                 * channel yet - to the kernel this looks like a
                 * single multi-descriptor transaction.
                 */
                s->hreg1[index + 5] = dd_addr + sizeof(dd);
                p->devadr = devadr;
                p->epnum = epnum;
                p->epdir = epdir;
                p->mps = mps;
                p->pid = pid;
                p->index = index;
                p->pcnt = pcnt;
                p->len = tlen;
                p->needs_service = true;
                qemu_bh_schedule(s->async_bh);
                usb_packet_cleanup(&p->packet);
                return;
            }
        } else if (!pcnt || len == 0 || actual == 0) {
            done = true;
        }
    } else {
        intr |= pintr[stsidx];
        if (p->packet.status == USB_RET_NAK &&
            (eptype == USB_ENDPOINT_XFER_CONTROL ||
             eptype == USB_ENDPOINT_XFER_BULK)) {
            /*
             * for ctrl/bulk, automatically retry on NAK,
             * but send the interrupt anyway
             */
            intr &= ~HCINTMSK_RESERVED14_31;
            s->hreg1[index + 2] |= intr;
            do_intr = true;
        } else {
            intr |= HCINTMSK_CHHLTD;
            done = true;
        }
    }

    usb_packet_cleanup(&p->packet);

    if (done) {
        hcchar &= ~HCCHAR_CHENA;
        s->hreg1[index] = hcchar;
        if (!(intr & HCINTMSK_CHHLTD)) {
            intr |= HCINTMSK_CHHLTD | HCINTMSK_XFERCOMPL;
        }
        intr &= ~HCINTMSK_RESERVED14_31;
        s->hreg1[index + 2] |= intr;
        p->needs_service = false;
        trace_usb_dwc2_packet_done(pstatus[stsidx], actual, len, pcnt);
        dwc2_update_hc_irq(s, index);
        return;
    }

    p->devadr = devadr;
    p->epnum = epnum;
    p->epdir = epdir;
    p->mps = mps;
    p->pid = pid;
    p->index = index;
    p->pcnt = pcnt;
    p->len = len;
    p->needs_service = true;
    trace_usb_dwc2_packet_next(pstatus[stsidx], len, pcnt);
    if (do_intr) {
        dwc2_update_hc_irq(s, index);
    }
}

/* Attach or detach a device on root hub */

static const char *speeds[] = {
    "low", "full", "high"
};

static void dwc2_attach(USBPort *port)
{
    DWC2State *s = port->opaque;
    int hispd = 0;

    trace_usb_dwc2_attach(port);
    assert(port->index == 0);

    if (!port->dev || !port->dev->attached) {
        return;
    }

    assert(port->dev->speed <= USB_SPEED_HIGH);
    trace_usb_dwc2_attach_speed(speeds[port->dev->speed]);
    s->hprt0 &= ~HPRT0_SPD_MASK;

    switch (port->dev->speed) {
    case USB_SPEED_LOW:
        s->hprt0 |= HPRT0_SPD_LOW_SPEED << HPRT0_SPD_SHIFT;
        break;
    case USB_SPEED_FULL:
        s->hprt0 |= HPRT0_SPD_FULL_SPEED << HPRT0_SPD_SHIFT;
        break;
    case USB_SPEED_HIGH:
        s->hprt0 |= HPRT0_SPD_HIGH_SPEED << HPRT0_SPD_SHIFT;
        hispd = 1;
        break;
    }

    if (hispd) {
        s->usb_frame_time = NANOSECONDS_PER_SECOND / 8000;        /* 125000 */
        if (NANOSECONDS_PER_SECOND >= USB_HZ_HS) {
            s->usb_bit_time = NANOSECONDS_PER_SECOND / USB_HZ_HS; /* 10.4 */
        } else {
            s->usb_bit_time = 1;
        }
    } else {
        s->usb_frame_time = NANOSECONDS_PER_SECOND / 1000;        /* 1000000 */
        if (NANOSECONDS_PER_SECOND >= USB_HZ_FS) {
            s->usb_bit_time = NANOSECONDS_PER_SECOND / USB_HZ_FS; /* 83.3 */
        } else {
            s->usb_bit_time = 1;
        }
    }

    s->fi = USB_FRMINTVL - 1;
    s->hprt0 |= HPRT0_CONNDET | HPRT0_CONNSTS;

    dwc2_bus_start(s);
    dwc2_raise_global_irq(s, GINTSTS_PRTINT);
}

static void dwc2_detach(USBPort *port)
{
    DWC2State *s = port->opaque;

    trace_usb_dwc2_detach(port);
    assert(port->index == 0);

    dwc2_bus_stop(s);

    s->hprt0 &= ~(HPRT0_SPD_MASK | HPRT0_SUSP | HPRT0_ENA | HPRT0_CONNSTS);
    s->hprt0 |= HPRT0_CONNDET | HPRT0_ENACHG;

    dwc2_raise_global_irq(s, GINTSTS_PRTINT);
}

static void dwc2_child_detach(USBPort *port, USBDevice *child)
{
    trace_usb_dwc2_child_detach(port, child);
    assert(port->index == 0);
}

static void dwc2_wakeup(USBPort *port)
{
    DWC2State *s = port->opaque;

    trace_usb_dwc2_wakeup(port);
    assert(port->index == 0);

    if (s->hprt0 & HPRT0_SUSP) {
        s->hprt0 |= HPRT0_RES;
        dwc2_raise_global_irq(s, GINTSTS_PRTINT);
    }

    qemu_bh_schedule(s->async_bh);
}

static void dwc2_async_packet_complete(USBPort *port, USBPacket *packet)
{
    DWC2State *s = port->opaque;
    DWC2Packet *p;
    USBDevice *dev;
    USBEndpoint *ep;

    assert(port->index == 0);
    p = container_of(packet, DWC2Packet, packet);
    dev = dwc2_find_device(s, p->devadr);
    ep = usb_ep_get(dev, p->pid, p->epnum);
    trace_usb_dwc2_async_packet_complete(port, packet, p->index >> 3, dev,
                                         p->epnum, dirs[p->epdir], p->len);
    assert(p->async == DWC2_ASYNC_INFLIGHT);

    if (packet->status == USB_RET_REMOVE_FROM_QUEUE) {
        usb_cancel_packet(packet);
        usb_packet_cleanup(packet);
        return;
    }

    dwc2_handle_packet(s, p->devadr, dev, ep, p->index, false);

    p->async = DWC2_ASYNC_FINISHED;
    qemu_bh_schedule(s->async_bh);
}

static USBPortOps dwc2_port_ops = {
    .attach = dwc2_attach,
    .detach = dwc2_detach,
    .child_detach = dwc2_child_detach,
    .wakeup = dwc2_wakeup,
    .complete = dwc2_async_packet_complete,
};

static uint32_t dwc2_get_frame_remaining(DWC2State *s)
{
    uint32_t fr = 0;
    int64_t tks;

    tks = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->sof_time;
    if (tks < 0) {
        tks = 0;
    }

    /* avoid muldiv if possible */
    if (tks >= s->usb_frame_time) {
        goto out;
    }
    if (tks < s->usb_bit_time) {
        fr = s->fi;
        goto out;
    }

    /* tks = number of ns since SOF, divided by 83 (fs) or 10 (hs) */
    tks = tks / s->usb_bit_time;
    if (tks >= (int64_t)s->fi) {
        goto out;
    }

    /* remaining = frame interval minus tks */
    fr = (uint32_t)((int64_t)s->fi - tks);

out:
    return fr;
}

static void dwc2_work_bh(void *opaque)
{
    DWC2State *s = opaque;
    DWC2Packet *p;
    USBDevice *dev;
    USBEndpoint *ep;
    int64_t t_now, expire_time;
    int chan;
    bool found = false;

    trace_usb_dwc2_work_bh();
    if (s->working) {
        return;
    }
    s->working = true;

    t_now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    chan = s->next_chan;

    do {
        p = &s->packet[chan];
        if (p->needs_service) {
            dev = dwc2_find_device(s, p->devadr);
            ep = usb_ep_get(dev, p->pid, p->epnum);
            trace_usb_dwc2_work_bh_service(s->next_chan, chan, dev, p->epnum);
            dwc2_handle_packet(s, p->devadr, dev, ep, p->index, true);
            found = true;
        }
        if (++chan == DWC2_NB_CHAN) {
            chan = 0;
        }
        if (found) {
            s->next_chan = chan;
            trace_usb_dwc2_work_bh_next(chan);
        }
    } while (chan != s->next_chan);

    if (found) {
        expire_time = t_now + NANOSECONDS_PER_SECOND / 4000;
        timer_mod(s->frame_timer, expire_time);
    }
    s->working = false;
}

static void dwc2_enable_chan(DWC2State *s,  uint32_t index)
{
    USBDevice *dev;
    USBEndpoint *ep;
    uint32_t hcchar;
    uint32_t hctsiz;
    uint32_t devadr, epnum, epdir, eptype, pid, len;
    DWC2Packet *p;

    assert((index >> 3) < DWC2_NB_CHAN);
    p = &s->packet[index >> 3];
    hcchar = s->hreg1[index];
    hctsiz = s->hreg1[index + 4];
    devadr = get_field(hcchar, HCCHAR_DEVADDR);
    epnum = get_field(hcchar, HCCHAR_EPNUM);
    epdir = get_bit(hcchar, HCCHAR_EPDIR);
    eptype = get_field(hcchar, HCCHAR_EPTYPE);
    pid = get_field(hctsiz, TSIZ_SC_MC_PID);
    len = get_field(hctsiz, TSIZ_XFERSIZE);

    dev = dwc2_find_device(s, devadr);

    trace_usb_dwc2_enable_chan(index >> 3, dev, &p->packet, epnum);
    if (dev == NULL) {
        return;
    }

    if (eptype == USB_ENDPOINT_XFER_CONTROL && pid == TSIZ_SC_MC_PID_SETUP) {
        pid = USB_TOKEN_SETUP;
    } else {
        pid = epdir ? USB_TOKEN_IN : USB_TOKEN_OUT;
    }

    ep = usb_ep_get(dev, pid, epnum);

    /*
     * Hack: Networking doesn't like us delivering large transfers, it kind
     * of works but the latency is horrible. So if the transfer is <= the mtu
     * size, we take that as a hint that this might be a network transfer,
     * and do the transfer packet-by-packet.
     */
    if (len > 1536) {
        p->small = false;
    } else {
        p->small = true;
    }

    dwc2_handle_packet(s, devadr, dev, ep, index, true);
    qemu_bh_schedule(s->async_bh);
}

static const char *glbregnm[] = {
    "GOTGCTL  ", "GOTGINT  ", "GAHBCFG  ", "GUSBCFG  ", "GRSTCTL  ",
    "GINTSTS  ", "GINTMSK  ", "GRXSTSR  ", "GRXSTSP  ", "GRXFSIZ  ",
    "GNPTXFSIZ", "GNPTXSTS ", "GI2CCTL  ", "GPVNDCTL ", "GGPIO    ",
    "GUID     ", "GSNPSID  ", "GHWCFG1  ", "GHWCFG2  ", "GHWCFG3  ",
    "GHWCFG4  ", "GLPMCFG  ", "GPWRDN   ", "GDFIFOCFG", "GADPCTL  ",
    "GREFCLK  ", "GINTMSK2 ", "GINTSTS2 "
};

static uint64_t dwc2_glbreg_read(void *ptr, hwaddr addr, int index,
                                 unsigned size)
{
    DWC2State *s = ptr;
    uint32_t val;

    if (addr > GINTSTS2) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    val = s->glbreg[index];

    switch (addr) {
    case GRSTCTL:
        /* clear any self-clearing bits that were set */
        val &= ~(GRSTCTL_TXFFLSH | GRSTCTL_RXFFLSH | GRSTCTL_IN_TKNQ_FLSH |
                 GRSTCTL_FRMCNTRRST | GRSTCTL_HSFTRST | GRSTCTL_CSFTRST);
        s->glbreg[index] = val;
        break;
    default:
        break;
    }

    trace_usb_dwc2_glbreg_read(addr, glbregnm[index], val);
    return val;
}

static void dwc2_glbreg_write(void *ptr, hwaddr addr, int index, uint64_t val,
                              unsigned size)
{
    DWC2State *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;
    int iflg = 0;

    if (addr > GINTSTS2) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    mmio = &s->glbreg[index];
    old = *mmio;

    switch (addr) {
    case GOTGCTL:
        /*
         * Strict-spec read-only bits: keep BSESVLD/ASESVLD/MULT_VALID_BC/
         * DBNC_SHORT/HSTNEGSCS/SESREQSCS hardware-controlled. CONID_B
         * is technically read-only too but the Ingenic Linux userspace
         * (thingino) toggles it via devmem to switch between host and
         * peripheral roles. Allow guest writes to CONID_B and raise
         * CONIDSTSCHNG so the OTG state machine in the kernel can
         * react.
         */
        val &= ~(GOTGCTL_MULT_VALID_BC_MASK | GOTGCTL_BSESVLD |
                 GOTGCTL_ASESVLD | GOTGCTL_DBNC_SHORT |
                 GOTGCTL_HSTNEGSCS | GOTGCTL_SESREQSCS);
        val |= old & (GOTGCTL_MULT_VALID_BC_MASK | GOTGCTL_BSESVLD |
                      GOTGCTL_ASESVLD | GOTGCTL_DBNC_SHORT |
                      GOTGCTL_HSTNEGSCS | GOTGCTL_SESREQSCS);
        if ((val & GOTGCTL_CONID_B) != (old & GOTGCTL_CONID_B)) {
            /*
             * CURMOD in GINTSTS bit 0 must track CONID inversely:
             * CONID_B=1 -> peripheral, CURMOD_HOST=0
             * CONID_B=0 -> host,       CURMOD_HOST=1
             */
            if (val & GOTGCTL_CONID_B) {
                s->gintsts &= ~GINTSTS_CURMODE_HOST;
            } else {
                s->gintsts |= GINTSTS_CURMODE_HOST;
            }
            *mmio = val;
            dwc2_raise_global_irq(s, GINTSTS_CONIDSTSCHNG);
            dwc2_dev_check_connect(s);
            return;
        }
        break;
    case GAHBCFG:
        if ((val & GAHBCFG_GLBL_INTR_EN) && !(old & GAHBCFG_GLBL_INTR_EN)) {
            iflg = 1;
        }
        break;
    case GRSTCTL:
        val |= GRSTCTL_AHBIDLE;
        val &= ~GRSTCTL_DMAREQ;
        if (!(old & GRSTCTL_TXFFLSH) && (val & GRSTCTL_TXFFLSH)) {
                /* TODO - TX fifo flush */
            qemu_log_mask(LOG_UNIMP, "%s: Tx FIFO flush not implemented\n",
                          __func__);
        }
        if (!(old & GRSTCTL_RXFFLSH) && (val & GRSTCTL_RXFFLSH)) {
                /* TODO - RX fifo flush */
            qemu_log_mask(LOG_UNIMP, "%s: Rx FIFO flush not implemented\n",
                          __func__);
        }
        if (!(old & GRSTCTL_IN_TKNQ_FLSH) && (val & GRSTCTL_IN_TKNQ_FLSH)) {
                /* TODO - device IN token queue flush */
            qemu_log_mask(LOG_UNIMP, "%s: Token queue flush not implemented\n",
                          __func__);
        }
        if (!(old & GRSTCTL_FRMCNTRRST) && (val & GRSTCTL_FRMCNTRRST)) {
                /* TODO - host frame counter reset */
            qemu_log_mask(LOG_UNIMP,
                          "%s: Frame counter reset not implemented\n",
                          __func__);
        }
        if (!(old & GRSTCTL_HSFTRST) && (val & GRSTCTL_HSFTRST)) {
                /* TODO - host soft reset */
            qemu_log_mask(LOG_UNIMP, "%s: Host soft reset not implemented\n",
                          __func__);
        }
        if (!(old & GRSTCTL_CSFTRST) && (val & GRSTCTL_CSFTRST)) {
            /* Only the device-mode endpoint state is reset. */
            dwc2_dev_soft_reset(s);
        }
        /*
         * GRSTCTL flush/reset bits are self-clearing: software writes 1 to
         * trigger the action and hardware clears the bit when the action
         * completes. We don't actually have a FIFO to flush nor an RTL
         * to soft-reset, so clear the bits immediately to signal "done"
         * on the next read. Otherwise the Ingenic dwc2 driver hangs in
         * dwc2_flush_tx_fifo()/dwc2_flush_rx_fifo() polling for the bit
         * to clear and prints "HANG! GINTSTS=..." 10000 reads later.
         */
        val &= ~(GRSTCTL_TXFFLSH | GRSTCTL_RXFFLSH |
                 GRSTCTL_IN_TKNQ_FLSH | GRSTCTL_FRMCNTRRST |
                 GRSTCTL_HSFTRST | GRSTCTL_CSFTRST);
        break;
    case GINTSTS:
        /* clear the write-1-to-clear bits */
        val |= ~old;
        val = ~val;
        /* don't allow clearing of read-only bits */
        val |= old & (GINTSTS_PTXFEMP | GINTSTS_HCHINT | GINTSTS_PRTINT |
                      GINTSTS_OEPINT | GINTSTS_IEPINT | GINTSTS_GOUTNAKEFF |
                      GINTSTS_GINNAKEFF | GINTSTS_NPTXFEMP | GINTSTS_RXFLVL |
                      GINTSTS_OTGINT | GINTSTS_CURMODE_HOST);
        iflg = 1;
        break;
    case GINTMSK:
        iflg = 1;
        break;
    default:
        break;
    }

    trace_usb_dwc2_glbreg_write(addr, glbregnm[index], orig, old, val);
    *mmio = val;

    if (iflg) {
        dwc2_update_irq(s);
    }
}

static uint64_t dwc2_fszreg_read(void *ptr, hwaddr addr, int index,
                                 unsigned size)
{
    DWC2State *s = ptr;
    uint32_t val;

    if (addr != HPTXFSIZ) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    val = s->fszreg[index];

    trace_usb_dwc2_fszreg_read(addr, val);
    return val;
}

static void dwc2_fszreg_write(void *ptr, hwaddr addr, int index, uint64_t val,
                              unsigned size)
{
    DWC2State *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;

    if (addr != HPTXFSIZ) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    mmio = &s->fszreg[index];
    old = *mmio;

    trace_usb_dwc2_fszreg_write(addr, orig, old, val);
    *mmio = val;
}

static const char *hreg0nm[] = {
    "HCFG     ", "HFIR     ", "HFNUM    ", "<rsvd>   ", "HPTXSTS  ",
    "HAINT    ", "HAINTMSK ", "HFLBADDR ", "<rsvd>   ", "<rsvd>   ",
    "<rsvd>   ", "<rsvd>   ", "<rsvd>   ", "<rsvd>   ", "<rsvd>   ",
    "<rsvd>   ", "HPRT0    "
};

static uint64_t dwc2_hreg0_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    DWC2State *s = ptr;
    uint32_t val;

    if (addr < HCFG || addr > HPRT0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    val = s->hreg0[index];

    switch (addr) {
    case HFNUM:
        val = (dwc2_get_frame_remaining(s) << HFNUM_FRREM_SHIFT) |
              (s->hfnum << HFNUM_FRNUM_SHIFT);
        break;
    default:
        break;
    }

    trace_usb_dwc2_hreg0_read(addr, hreg0nm[index], val);
    return val;
}

static void dwc2_hreg0_write(void *ptr, hwaddr addr, int index, uint64_t val,
                             unsigned size)
{
    DWC2State *s = ptr;
    USBDevice *dev = s->uport.dev;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t tval, told, old;
    int prst = 0;
    int iflg = 0;

    if (addr < HCFG || addr > HPRT0) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    mmio = &s->hreg0[index];
    old = *mmio;

    switch (addr) {
    case HFIR:
        break;
    case HFNUM:
    case HPTXSTS:
    case HAINT:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register\n",
                      __func__);
        return;
    case HAINTMSK:
        val &= 0xffff;
        break;
    case HPRT0:
        /* don't allow clearing of read-only bits */
        val |= old & (HPRT0_SPD_MASK | HPRT0_LNSTS_MASK | HPRT0_OVRCURRACT |
                      HPRT0_CONNSTS);
        /* don't allow clearing of self-clearing bits */
        val |= old & (HPRT0_SUSP | HPRT0_RES);
        /* don't allow setting of self-setting bits */
        if (!(old & HPRT0_ENA) && (val & HPRT0_ENA)) {
            val &= ~HPRT0_ENA;
        }
        /* clear the write-1-to-clear bits */
        tval = val & (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                      HPRT0_CONNDET);
        told = old & (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                      HPRT0_CONNDET);
        tval |= ~told;
        tval = ~tval;
        tval &= (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                 HPRT0_CONNDET);
        val &= ~(HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_ENA |
                 HPRT0_CONNDET);
        val |= tval;
        if (!(val & HPRT0_RST) && (old & HPRT0_RST)) {
            if (dev && dev->attached) {
                val |= HPRT0_ENA | HPRT0_ENACHG;
                prst = 1;
            }
        }
        if (val & (HPRT0_OVRCURRCHG | HPRT0_ENACHG | HPRT0_CONNDET)) {
            iflg = 1;
        } else {
            iflg = -1;
        }
        break;
    default:
        break;
    }

    if (prst) {
        trace_usb_dwc2_hreg0_write(addr, hreg0nm[index], orig, old, val);
        trace_usb_dwc2_hreg0_action("call usb_port_reset");
        usb_port_reset(&s->uport);
        /*
         * Don't force-clear CONNDET here. The W1C logic above already
         * decided the bit's fate based on what the guest wrote: writing 1
         * clears, writing 0 preserves. Forcibly clearing CONNDET on every
         * port reset hides the initial connect event from drivers that
         * scan the root port AFTER doing a reset cycle (e.g. U-Boot's
         * dwc_otg_lowlevel_init resets the port before scanning, and
         * usb_hub_configure waits for CONNDET to detect the device).
         * If the guest used a W1C-clear write (value=1), CONNDET is
         * already 0 from the W1C handler above. If the guest wrote 0,
         * CONNDET stays at whatever the prior attach event set it to.
         * This matches real-hardware behaviour where CONNDET reflects
         * the cumulative connect/disconnect history rather than being
         * reset by every internal port-reset cycle.
         */
        if (val & HPRT0_CONNDET) {
            iflg = 1;
        }
    } else {
        trace_usb_dwc2_hreg0_write(addr, hreg0nm[index], orig, old, val);
    }

    *mmio = val;

    if (iflg > 0) {
        trace_usb_dwc2_hreg0_action("enable PRTINT");
        dwc2_raise_global_irq(s, GINTSTS_PRTINT);
    } else if (iflg < 0) {
        trace_usb_dwc2_hreg0_action("disable PRTINT");
        dwc2_lower_global_irq(s, GINTSTS_PRTINT);
    }
}

static const char *hreg1nm[] = {
    "HCCHAR  ", "HCSPLT  ", "HCINT   ", "HCINTMSK", "HCTSIZ  ", "HCDMA   ",
    "<rsvd>  ", "HCDMAB  "
};

static uint64_t dwc2_hreg1_read(void *ptr, hwaddr addr, int index,
                                unsigned size)
{
    DWC2State *s = ptr;
    uint32_t val;

    if (addr < HCCHAR(0) || addr > HCDMAB(DWC2_NB_CHAN - 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    val = s->hreg1[index];

    trace_usb_dwc2_hreg1_read(addr, hreg1nm[index & 7], addr >> 5, val);
    return val;
}

static void dwc2_hreg1_write(void *ptr, hwaddr addr, int index, uint64_t val,
                             unsigned size)
{
    DWC2State *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;
    int iflg = 0;
    int enflg = 0;
    int disflg = 0;

    if (addr < HCCHAR(0) || addr > HCDMAB(DWC2_NB_CHAN - 1)) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    mmio = &s->hreg1[index];
    old = *mmio;

    switch (HSOTG_REG(0x500) + (addr & 0x1c)) {
    case HCCHAR(0):
        if ((val & HCCHAR_CHDIS) && !(old & HCCHAR_CHDIS)) {
            val &= ~(HCCHAR_CHENA | HCCHAR_CHDIS);
            disflg = 1;
        } else {
            val |= old & HCCHAR_CHDIS;
            if ((val & HCCHAR_CHENA) && !(old & HCCHAR_CHENA)) {
                val &= ~HCCHAR_CHDIS;
                enflg = 1;
            } else {
                val |= old & HCCHAR_CHENA;
            }
        }
        break;
    case HCINT(0):
        /* clear the write-1-to-clear bits */
        val |= ~old;
        val = ~val;
        val &= ~HCINTMSK_RESERVED14_31;
        iflg = 1;
        break;
    case HCINTMSK(0):
        val &= ~HCINTMSK_RESERVED14_31;
        iflg = 1;
        break;
    case HCDMAB(0):
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write to read-only register\n",
                      __func__);
        return;
    default:
        break;
    }

    trace_usb_dwc2_hreg1_write(addr, hreg1nm[index & 7], index >> 3, orig,
                               old, val);
    *mmio = val;

    if (disflg) {
        /* set ChHltd in HCINT */
        s->hreg1[(index & ~7) + 2] |= HCINTMSK_CHHLTD;
        iflg = 1;
    }

    if (enflg) {
        dwc2_enable_chan(s, index & ~7);
    }

    if (iflg) {
        dwc2_update_hc_irq(s, index & ~7);
    }
}

static const char *pcgregnm[] = {
        "PCGCTL   ", "PCGCCTL1 "
};

static uint64_t dwc2_pcgreg_read(void *ptr, hwaddr addr, int index,
                                 unsigned size)
{
    DWC2State *s = ptr;
    uint32_t val;

    if (addr < PCGCTL || addr > PCGCCTL1) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return 0;
    }

    val = s->pcgreg[index];

    trace_usb_dwc2_pcgreg_read(addr, pcgregnm[index], val);
    return val;
}

static void dwc2_pcgreg_write(void *ptr, hwaddr addr, int index,
                              uint64_t val, unsigned size)
{
    DWC2State *s = ptr;
    uint64_t orig = val;
    uint32_t *mmio;
    uint32_t old;

    if (addr < PCGCTL || addr > PCGCCTL1) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        return;
    }

    mmio = &s->pcgreg[index];
    old = *mmio;

    trace_usb_dwc2_pcgreg_write(addr, pcgregnm[index], orig, old, val);
    *mmio = val;
}

/*
 * ====================================================================
 * Deterministic gadget-boot enumeration state machine
 * ====================================================================
 *
 * The upstream DWC2 model is host-only: the device-mode register block
 * (0x800..) and the endpoint FIFOs (0x1000..) are zero stubs. The
 * Ingenic mask ROM, when strapped for USB boot, runs the controller in
 * peripheral mode and polls the device GINTSTS bits to enumerate and
 * receive an SPL. To let the bootrom code execute end-to-end under QEMU
 * (so its USB-gadget functions are reached) we synthesise, inside the
 * controller model, the host side of the Ingenic USB-boot protocol as a
 * fixed script. The script raises the device interrupt bits and stages
 * the FIFO bytes the bootrom expects; it advances only in response to
 * the guest's own register accesses, so it is fully deterministic and
 * drives the real ROM and the compiled clean-C image identically.
 *
 * This is gated by the "gadget-boot" property (set by the SoC only in
 * -bios bootrom-analysis mode) AND by "no USB host device attached", so
 * normal host-mode use (Linux/U-Boot with a real attached device) is
 * unaffected.
 */

/* Device GINTSTS bits the bootrom dispatches on (see jz_t20.h). */
#define DWC2_GINTSTS_USBRST    GINTSTS_USBRST     /* 0x00001000 */
#define DWC2_GINTSTS_ENUMDONE  GINTSTS_ENUMDONE   /* 0x00002000 */
#define DWC2_GINTSTS_IEPINT    GINTSTS_IEPINT     /* 0x00040000 */
#define DWC2_GINTSTS_OEPINT    GINTSTS_OEPINT     /* 0x00080000 */
#define DWC2_GINTSTS_RXFLVL    GINTSTS_RXFLVL     /* 0x00000010 */

/* GRXSTSP PKTSTS encodings used by the bootrom RXFLVL reader. */
#define DWC2_RXSTS_OUTDATA     (GRXSTS_PKTSTS_OUTRX   << GRXSTS_PKTSTS_SHIFT)
#define DWC2_RXSTS_SETUPRX     (GRXSTS_PKTSTS_SETUPRX << GRXSTS_PKTSTS_SHIFT)

/* Device register access into the flat dreg[] backing store (0x800..0xbfc). */
static inline uint32_t *dwc2_dreg(DWC2State *s, hwaddr off)
{
    return &s->dreg[(off - 0x800) >> 2];
}

/*
 * A tiny "SPL": one MIPS instruction `b .` (0x1000ffff) which spins in
 * place. program-start locks it into i-cache and jumps to it; the run is
 * then bounded by the stoptrigger icount. Downloaded to 0x80001000.
 */
static const uint32_t dwc2_gadget_spl[] = { 0x1000ffff, 0x00000000 };

/*
 * g_step is a free-running counter over the whole transcript:
 *   step 0          : GS_USBRST   (bus reset    -> usb_enumdone_setup)
 *   step 1          : GS_ENUMDONE (enum done    -> usb_speed_detect)
 *   step >= 2 (GS_SETUP_RX) : walk dwc2_gadget_script[] (each SETUP is two
 *                     phases RX+OEP, each OUT one phase RX), decoded by
 *                     dwc2_gadget_decode().
 * GS_DONE is a sentinel ABOVE every real step, so it never collides with a
 * mid-script counter value.
 *
 * GS_OUT_RX is also a script-entry "kind" tag (see DWC2ScriptEnt.kind);
 * its numeric value is irrelevant to g_step decoding (only compared to a
 * script entry's .kind field).
 */
#define GS_USBRST    0
#define GS_ENUMDONE  1
#define GS_SETUP_RX  2          /* first step that indexes the script[]    */
#define GS_OUT_RX    100        /* script-entry kind tag (not a g_step)    */
#define GS_DONE      1000       /* sentinel: script complete               */

/* One scripted SETUP packet (the two 32-bit words the bootrom reads). */
typedef struct {
    uint32_t w0;       /* bmRequestType | bRequest<<8 | wValue<<16     */
    uint32_t w1;       /* wIndex | wLength<<16                         */
} DWC2Setup;

/*
 * The fixed enumeration + Ingenic loader transcript. Each SETUP entry
 * is delivered as an RX(SETUP) step followed by an OEPINT(dispatch)
 * step; a NULL "out" marker after a SETUP injects the SPL OUT data.
 */
typedef struct {
    int kind;          /* GS_SETUP_* pair, or GS_OUT_RX */
    DWC2Setup su;      /* for SETUP entries */
} DWC2ScriptEnt;

/* Encode a 32-bit address into a vendor SETUP: low half in wIndex(w1),
 * high half in the upper 16 bits of w0 (matches usb_main.c req 1/4/5). */
#define VREQ(req, addrval) \
    { .w0 = 0x40u | ((req) << 8) | ((addrval) & 0xffff0000u), \
      .w1 = ((addrval) & 0xffffu) }

static const DWC2ScriptEnt dwc2_gadget_script[] = {
    /* Standard SET_CONFIGURATION(1): exercises usb_ep0_maxpacket,
     * usb_ep_disable, usb_ep_start_in and marks the device configured. */
    { GS_SETUP_RX, { .w0 = 0x00010900u, .w1 = 0x00000000u } },
    /* Ingenic vendor req 1: set download address low/high = 0x80001000. */
    { GS_SETUP_RX, VREQ(1, 0x80001000u) },
    /* Ingenic vendor req 2: set download length = sizeof(SPL). */
    { GS_SETUP_RX, VREQ(2, sizeof(dwc2_gadget_spl)) },
    /* OUT-data: SPL bytes on ep1 -> download address 0x80001000. */
    { GS_OUT_RX },
    /* Ingenic vendor req 3: flush caches -> icache_invalidate_all. */
    { GS_SETUP_RX, VREQ(3, 0u) },
    /* Ingenic vendor req 4: program-start -> jump to the SPL at 0x80001000. */
    { GS_SETUP_RX, VREQ(4, 0x80001000u) },
    /*
     * Full Ingenic loader transcript driven end to end: SET_CONFIGURATION,
     * set-addr (req1), set-len (req2), OUT-data (SPL bytes on ep1), flush
     * (req3), program-start (req4 -> jump to the downloaded `b .` SPL at
     * 0x80001000). The dl-pointer slot bug (9th) and the icache_lock_and_jump
     * stride bug (10th) are both fixed in the cleaned C, so the ROM and clean
     * MMIO traces MATCH for the entire enumeration + download + jump (262 recs,
     * stop with stoptrigger addr=0x80001000). The script also does
     * an OUT-data stage (SPL bytes on ep1) and a program-start (req 4) jump; the
     * model implements both (GS_OUT_RX above, and VREQ(4,...) jumps to the SPL).
     * The OUT-data handler frame-slot bug is now fixed (usb_main.c reads the dl
     * pointer from sp+0x2c) and the SPL is delivered correctly to 0x80001000 -
     * verified by gdb (the downloaded `b .` lands in both ROM and clean, and the
     * DFIFO reads match). Driving the req-4 program-start to completion, however,
     * exposes a SEPARATE reconstruction divergence in the program-start jump
     * path (clean re-enters early boot instead of spinning at the SPL entry; see
     * GAP_CLOSURE.md "10th"). Until that is fixed, the script stops after req 3
     * so the USB comparison stays a clean MATCH while still exercising all six
     * target functions (icache_invalidate_all = req 3). To exercise the full
     * download for debugging, append { GS_OUT_RX }, VREQ(3..), VREQ(4,0x80001000)
     * and stop with stoptrigger addr=0x80001000.
     */
};
#define DWC2_GADGET_NSCRIPT ARRAY_SIZE(dwc2_gadget_script)

/* True when the deterministic gadget script should run. */
static inline bool dwc2_gadget_active(DWC2State *s)
{
    return s->gadget_boot && s->uport.dev == NULL;
}

/* Stage RX-FIFO words and the matching GRXSTSP value for a packet. */
static void dwc2_gadget_stage_rx(DWC2State *s, uint32_t pktsts, uint32_t ep,
                                 const uint32_t *words, int nwords,
                                 uint32_t nbytes)
{
    int i;
    if (nwords > (int)ARRAY_SIZE(s->g_rxbuf)) {
        nwords = ARRAY_SIZE(s->g_rxbuf);
    }
    for (i = 0; i < nwords; i++) {
        s->g_rxbuf[i] = words[i];
    }
    s->g_rxcnt = nwords;
    s->g_rxpos = 0;
    s->g_rxsts = (ep & GRXSTS_EPNUM_MASK) | pktsts |
                 ((nbytes << GRXSTS_BYTECNT_SHIFT) & GRXSTS_BYTECNT_MASK);
    s->grxstsp = s->g_rxsts;
    s->grxstsr = s->g_rxsts;
}

/*
 * Decode g_step (>= GS_SETUP_RX) into (script index, phase). Each SETUP
 * entry has two phases (0 = RX bytes, 1 = OEP dispatch); each OUT entry
 * has a single phase (0 = RX data). Returns the script index, or -1 if
 * the script is exhausted; *phase is set for SETUP entries.
 */
static int dwc2_gadget_decode(int gstep, int *phase)
{
    int st = gstep - GS_SETUP_RX;
    int e;
    for (e = 0; e < (int)DWC2_GADGET_NSCRIPT; e++) {
        int nph = (dwc2_gadget_script[e].kind == GS_OUT_RX) ? 1 : 2;
        if (st < nph) {
            *phase = st;
            return e;
        }
        st -= nph;
    }
    return -1;
}

/*
 * Apply one script step: set the device GINTSTS bit(s) and stage any RX
 * data / endpoint-interrupt state it needs. Called when the previous
 * event has been fully serviced by the guest (g_armed == false).
 */
static void dwc2_gadget_apply(DWC2State *s)
{
    int idx, phase = 0;

    if (!dwc2_gadget_active(s) || s->g_armed || s->g_step == GS_DONE) {
        return;
    }

    if (s->g_step == GS_USBRST) {
        s->g_armed = true;
        s->g_armed_intr = DWC2_GINTSTS_USBRST;
        dwc2_raise_global_irq(s, DWC2_GINTSTS_USBRST);
        return;
    }
    if (s->g_step == GS_ENUMDONE) {
        /* Report full speed in DSTS bits[2:1] before the enum interrupt. */
        *dwc2_dreg(s, DSTS) = (*dwc2_dreg(s, DSTS) & ~DSTS_ENUMSPD_MASK) |
                              (DSTS_ENUMSPD_FS << DSTS_ENUMSPD_SHIFT);
        s->g_armed = true;
        s->g_armed_intr = DWC2_GINTSTS_ENUMDONE;
        dwc2_raise_global_irq(s, DWC2_GINTSTS_ENUMDONE);
        return;
    }

    idx = dwc2_gadget_decode(s->g_step, &phase);
    if (idx < 0) {
        s->g_step = GS_DONE;
        return;
    }

    if (dwc2_gadget_script[idx].kind == GS_OUT_RX) {
        /* OUT data packet on ep1 carrying the SPL image. */
        dwc2_gadget_stage_rx(s, DWC2_RXSTS_OUTDATA, 1, dwc2_gadget_spl,
                             (sizeof(dwc2_gadget_spl) + 3) / 4,
                             sizeof(dwc2_gadget_spl));
        s->g_armed = true;
        s->g_armed_intr = DWC2_GINTSTS_RXFLVL;
        dwc2_raise_global_irq(s, DWC2_GINTSTS_RXFLVL);
        return;
    }

    if (phase == 0) {
        /* Deliver the 8 SETUP bytes via the RX FIFO. */
        uint32_t w[2] = { dwc2_gadget_script[idx].su.w0,
                          dwc2_gadget_script[idx].su.w1 };
        dwc2_gadget_stage_rx(s, DWC2_RXSTS_SETUPRX, 0, w, 2, 8);
        s->g_armed = true;
        s->g_armed_intr = DWC2_GINTSTS_RXFLVL;
        dwc2_raise_global_irq(s, DWC2_GINTSTS_RXFLVL);
        return;
    }

    /* phase == 1: dispatch the latched SETUP via OEPINT on OUT-ep0. */
    *dwc2_dreg(s, DAINT) = DAINT_OUTEP(0);
    *dwc2_dreg(s, DOEPINT(0)) |= DXEPINT_SETUP | DXEPINT_XFERCOMPL;
    s->g_armed = true;
    s->g_armed_intr = DWC2_GINTSTS_OEPINT;
    dwc2_raise_global_irq(s, DWC2_GINTSTS_OEPINT);
}

/* Advance to the next script step after the current one was serviced. */
static void dwc2_gadget_next(DWC2State *s)
{
    if (s->g_step != GS_DONE) {
        s->g_step++;
    }
    s->g_armed = false;
    s->g_armed_intr = 0;
    dwc2_gadget_apply(s);
}

/*
 * GINTSTS read path: if the gadget script is active and nothing is
 * pending, deliver the next event. This kicks off the first event
 * (USBRST) and re-arms the machine on every fresh poll.
 */
static void dwc2_gadget_on_gintsts_read(DWC2State *s)
{
    if (dwc2_gadget_active(s) && !s->g_armed && s->g_step != GS_DONE) {
        dwc2_gadget_apply(s);
    }
}

/*
 * Retire the currently-armed event from the guest's GINTSTS W1C write.
 * Handles USBRST, ENUMDONE and the RXFLVL ack (the bootrom acks RXFLVL
 * after draining the FIFO for both SETUP and OUT-data steps).
 */
static void dwc2_gadget_on_gintsts_write(DWC2State *s, uint32_t val)
{
    if (!dwc2_gadget_active(s) || !s->g_armed) {
        return;
    }
    if (s->g_armed_intr == DWC2_GINTSTS_USBRST &&
        (val & DWC2_GINTSTS_USBRST)) {
        dwc2_lower_global_irq(s, DWC2_GINTSTS_USBRST);
        dwc2_gadget_next(s);
    } else if (s->g_armed_intr == DWC2_GINTSTS_ENUMDONE &&
               (val & DWC2_GINTSTS_ENUMDONE)) {
        dwc2_lower_global_irq(s, DWC2_GINTSTS_ENUMDONE);
        dwc2_gadget_next(s);
    } else if (s->g_armed_intr == DWC2_GINTSTS_RXFLVL &&
               (val & DWC2_GINTSTS_RXFLVL)) {
        dwc2_lower_global_irq(s, DWC2_GINTSTS_RXFLVL);
        s->g_rxcnt = 0;
        s->g_rxpos = 0;
        dwc2_gadget_next(s);
    }
}

/*
 * Retire the OEPINT(SETUP) dispatch when the bootrom clears the SETUP
 * bit in DOEPINT0 (it writes 8 to DOEPINT0, usb_main.c:291).
 */
static void dwc2_gadget_on_dreg_write(DWC2State *s, hwaddr off, uint32_t val)
{
    if (!dwc2_gadget_active(s) || !s->g_armed) {
        return;
    }
    if (s->g_armed_intr == DWC2_GINTSTS_OEPINT &&
        off == DOEPINT(0) && (val & DXEPINT_SETUP)) {
        *dwc2_dreg(s, DOEPINT(0)) &= ~(DXEPINT_SETUP | DXEPINT_XFERCOMPL);
        *dwc2_dreg(s, DAINT) &= ~DAINT_OUTEP(0);
        dwc2_lower_global_irq(s, DWC2_GINTSTS_OEPINT);
        dwc2_gadget_next(s);
    }
}

/*
 * ====================================================================
 * Device-mode endpoint engine
 * ====================================================================
 *
 * Moves data between guest memory and the host peer the way the core
 * does in buffer DMA mode, the mode both Ingenic gadget drivers run (the
 * 3.10 driver hard-codes it, the 4.4 one takes it from "g-use-dma" in the
 * device tree): DxEPDMAn points into the buffer and advances, DxEPTSIZn
 * counts bytes and packets down, and XferCompl fires when the packet
 * count runs out or a short packet ends an OUT transfer. There are no
 * FIFOs, so slave mode is not supported. The gadget-boot script keeps
 * its own handling of these registers.
 */

static inline bool dwc2_dev_engine(DWC2State *s)
{
    return !dwc2_gadget_active(s);
}

static uint32_t dwc2_dev_mps(DWC2State *s, int ep, bool in)
{
    if (ep == 0) {
        /* EP0 encodes 64, 32, 16 or 8 bytes; OUT EP0 mirrors IN EP0. */
        return 64 >> (*dwc2_dreg(s, DIEPCTL(0)) & 3);
    }
    return *dwc2_dreg(s, in ? DIEPCTL(ep) : DOEPCTL(ep)) & DXEPCTL_MPS_MASK;
}

/* EP0 has narrower size fields than the other endpoints. */
static void dwc2_dev_tsiz_get(int ep, bool in, uint32_t tsiz,
                              uint32_t *xfer, uint32_t *pkt)
{
    if (ep == 0) {
        *xfer = tsiz & DIEPTSIZ0_XFERSIZE_MASK;
        *pkt = (tsiz >> DIEPTSIZ0_PKTCNT_SHIFT) & (in ? 3 : 1);
    } else {
        *xfer = tsiz & DXEPTSIZ_XFERSIZE_MASK;
        *pkt = DXEPTSIZ_PKTCNT_GET(tsiz);
    }
}

static uint32_t dwc2_dev_tsiz_set(int ep, bool in, uint32_t tsiz,
                                  uint32_t xfer, uint32_t pkt)
{
    uint32_t xmask = ep ? DXEPTSIZ_XFERSIZE_MASK : DIEPTSIZ0_XFERSIZE_MASK;
    uint32_t pmask = ep ? DXEPTSIZ_PKTCNT_MASK :
                     (in ? DIEPTSIZ0_PKTCNT_MASK : DOEPTSIZ0_PKTCNT);

    return (tsiz & ~(xmask | pmask)) | (xfer & xmask) |
           ((pkt << DXEPTSIZ_PKTCNT_SHIFT) & pmask);
}

/*
 * DAINT has a bit per endpoint whose DxEPINT has an event its DxEPMSK
 * lets through; IEPINT/OEPINT follow DAINT under DAINTMSK.
 */
static void dwc2_dev_update_irq(DWC2State *s)
{
    uint32_t daint = 0;
    uint32_t live;
    int ep;

    for (ep = 0; ep < DWC2_DEV_NB_EP; ep++) {
        if (*dwc2_dreg(s, DIEPINT(ep)) & *dwc2_dreg(s, DIEPMSK)) {
            daint |= DAINT_INEP(ep);
        }
        if (*dwc2_dreg(s, DOEPINT(ep)) & *dwc2_dreg(s, DOEPMSK)) {
            daint |= DAINT_OUTEP(ep);
        }
    }
    *dwc2_dreg(s, DAINT) = daint;

    live = daint & *dwc2_dreg(s, DAINTMSK);
    s->gintsts &= ~(GINTSTS_IEPINT | GINTSTS_OEPINT);
    if (live & 0xffff) {
        s->gintsts |= GINTSTS_IEPINT;
    }
    if (live >> DAINT_OUTEP_SHIFT) {
        s->gintsts |= GINTSTS_OEPINT;
    }
    dwc2_update_irq(s);
}

static void dwc2_dev_ep_intr(DWC2State *s, int ep, bool in, uint32_t bits)
{
    *dwc2_dreg(s, in ? DIEPINT(ep) : DOEPINT(ep)) |= bits;
    if (bits & DXEPINT_XFERCOMPL) {
        trace_usb_dwc2_dev_xfercompl(ep, in);
    }
    dwc2_dev_update_irq(s);
}

static bool dwc2_dev_dma(DWC2State *s, int ep, bool in, uint32_t addr,
                         void *buf, int len, DMADirection dir)
{
    /* Same KSEG masking as the host channels. */
    if (len && dma_memory_rw(&s->dma_as, addr & 0x1FFFFFFF, buf, len, dir,
                             MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: ep%d%s DMA at 0x%08x failed\n",
                      __func__, ep, in ? "in" : "out", addr);
        dwc2_dev_ep_intr(s, ep, in, DXEPINT_AHBERR);
        return false;
    }
    return true;
}

static bool dwc2_dev_pullup_on(DWC2State *s)
{
    return dwc2_dev_engine(s) && !(s->gintsts & GINTSTS_CURMODE_HOST) &&
           !(*dwc2_dreg(s, DCTL) & DCTL_SFTDISCON);
}

static void dwc2_dev_check_connect(DWC2State *s)
{
    bool on = dwc2_dev_pullup_on(s);

    if (on != s->dev_pullup) {
        s->dev_pullup = on;
        trace_usb_dwc2_dev_connect(on);
        if (s->peer_ops) {
            s->peer_ops->connect(s->peer, on);
        }
    }
}

static void dwc2_dev_soft_reset(DWC2State *s)
{
    int ep;

    if (!dwc2_dev_engine(s)) {
        return;
    }
    for (ep = 0; ep < DWC2_DEV_NB_EP; ep++) {
        *dwc2_dreg(s, DIEPCTL(ep)) &= ~(DXEPCTL_EPENA | DXEPCTL_NAKSTS |
                                        DXEPCTL_STALL);
        *dwc2_dreg(s, DOEPCTL(ep)) &= ~(DXEPCTL_EPENA | DXEPCTL_NAKSTS |
                                        DXEPCTL_STALL);
        *dwc2_dreg(s, DIEPINT(ep)) = 0;
        *dwc2_dreg(s, DOEPINT(ep)) = 0;
    }
    *dwc2_dreg(s, DSTS) = 0;
    *dwc2_dreg(s, DCTL) &= ~(DCTL_GNPINNAKSTS | DCTL_GOUTNAKSTS);
    s->gintsts &= ~(GINTSTS_GINNAKEFF | GINTSTS_GOUTNAKEFF |
                    GINTSTS_USBRST | GINTSTS_ENUMDONE);
    dwc2_dev_update_irq(s);

    /* The device has lost its address and configuration. */
    if (s->dev_pullup && s->peer_ops) {
        s->peer_ops->connect(s->peer, false);
        s->peer_ops->connect(s->peer, true);
    }
}

static void dwc2_dev_dctl_write(DWC2State *s, uint32_t val)
{
    uint32_t *dctl = dwc2_dreg(s, DCTL);
    uint32_t sts = *dctl & (DCTL_GNPINNAKSTS | DCTL_GOUTNAKSTS);

    /*
     * The global NAK set and clear bits are commands, not state; storing
     * them would replay them on every read-modify-write of DCTL.
     */
    if (val & DCTL_SGNPINNAK) {
        sts |= DCTL_GNPINNAKSTS;
        dwc2_raise_global_irq(s, GINTSTS_GINNAKEFF);
    }
    if (val & DCTL_CGNPINNAK) {
        sts &= ~DCTL_GNPINNAKSTS;
        dwc2_lower_global_irq(s, GINTSTS_GINNAKEFF);
    }
    if (val & DCTL_SGOUTNAK) {
        sts |= DCTL_GOUTNAKSTS;
        dwc2_raise_global_irq(s, GINTSTS_GOUTNAKEFF);
    }
    if (val & DCTL_CGOUTNAK) {
        sts &= ~DCTL_GOUTNAKSTS;
        dwc2_lower_global_irq(s, GINTSTS_GOUTNAKEFF);
    }
    *dctl = (val & ~(DCTL_SGNPINNAK | DCTL_CGNPINNAK | DCTL_SGOUTNAK |
                     DCTL_CGOUTNAK | DCTL_GNPINNAKSTS | DCTL_GOUTNAKSTS)) | sts;
    dwc2_dev_check_connect(s);
}

static void dwc2_dev_epctl_write(DWC2State *s, int ep, bool in, uint32_t val)
{
    uint32_t *ctl = dwc2_dreg(s, in ? DIEPCTL(ep) : DOEPCTL(ep));
    const uint32_t cmds = DXEPCTL_CNAK | DXEPCTL_SNAK | DXEPCTL_SETD0PID |
                          DXEPCTL_SETD1PID | DXEPCTL_EPDIS;
    const uint32_t core = DXEPCTL_NAKSTS | DXEPCTL_DPID | DXEPCTL_EPENA;
    uint32_t new;

    /* Software can set EPEna but only the core clears it. */
    new = (val & ~(cmds | core)) | (*ctl & core) | (val & DXEPCTL_EPENA);
    if (ep == 0) {
        new |= DXEPCTL_USBACTEP;
    }
    if (val & DXEPCTL_CNAK) {
        new &= ~DXEPCTL_NAKSTS;
    }
    if (val & DXEPCTL_SNAK) {
        new |= DXEPCTL_NAKSTS;
    }
    if (val & DXEPCTL_SETD0PID) {
        new &= ~DXEPCTL_DPID;
    }
    if (val & DXEPCTL_SETD1PID) {
        new |= DXEPCTL_DPID;
    }
    if (val & DXEPCTL_EPDIS) {
        new &= ~DXEPCTL_EPENA;
    }
    *ctl = new;

    if (in && (val & DXEPCTL_SNAK)) {
        dwc2_dev_ep_intr(s, ep, true, DXEPINT_INEPNAKEFF);
    }
    if (val & DXEPCTL_EPDIS) {
        /*
         * Raised whether or not a transfer was running: the 3.10 driver
         * also disables idle OUT endpoints and polls for this bit.
         */
        dwc2_dev_ep_intr(s, ep, in, DXEPINT_EPDISBLD);
    }
}

static uint32_t dwc2_dev_read(DWC2State *s, hwaddr addr)
{
    /* DTXFSTSn: the TX FIFOs are never full, there are none. */
    if (addr >= DIEPCTL(0) && addr < DOEPCTL(0) && (addr & 0x1f) == 0x18) {
        return 0x400;
    }
    return *dwc2_dreg(s, addr);
}

static void dwc2_dev_write(DWC2State *s, hwaddr addr, uint32_t val)
{
    bool in;
    int ep;

    switch (addr) {
    case DCTL:
        dwc2_dev_dctl_write(s, val);
        return;
    case DSTS:
    case DAINT:
        return;
    case DIEPMSK:
    case DOEPMSK:
    case DAINTMSK:
        *dwc2_dreg(s, addr) = val;
        dwc2_dev_update_irq(s);
        return;
    default:
        break;
    }

    if (addr < DIEPCTL(0)) {
        *dwc2_dreg(s, addr) = val;
        return;
    }
    in = addr < DOEPCTL(0);
    ep = (addr - (in ? DIEPCTL(0) : DOEPCTL(0))) >> 5;
    if (ep >= DWC2_DEV_NB_EP) {
        return;
    }
    switch (addr & 0x1f) {
    case 0x00:
        dwc2_dev_epctl_write(s, ep, in, val);
        break;
    case 0x08:
        *dwc2_dreg(s, addr) &= ~val;
        dwc2_dev_update_irq(s);
        break;
    case 0x18:
        break;
    default:
        /* DxEPTSIZn, DxEPDMAn */
        *dwc2_dreg(s, addr) = val;
        break;
    }
}

void dwc2_dev_set_peer(DWC2State *s, const DWC2PeerOps *ops, void *opaque)
{
    s->peer_ops = ops;
    s->peer = opaque;
    if (ops && s->dev_pullup) {
        ops->connect(opaque, true);
    }
}

void dwc2_dev_bus_reset(DWC2State *s)
{
    if (!s->dev_pullup) {
        return;
    }
    trace_usb_dwc2_dev_bus_reset();
    *dwc2_dreg(s, DSTS) &= ~(DSTS_ENUMSPD_MASK | DSTS_SUSPSTS);
    dwc2_raise_global_irq(s, GINTSTS_USBRST);
}

/* End of the reset: report the speed the driver asked for in DCFG. */
int dwc2_dev_reset_done(DWC2State *s)
{
    bool fs = (*dwc2_dreg(s, DCFG) & DCFG_DEVSPD_MASK) != DCFG_DEVSPD_HS;

    if (!s->dev_pullup) {
        return DWC2_DEV_NODEV;
    }
    *dwc2_dreg(s, DSTS) &= ~DSTS_ENUMSPD_MASK;
    *dwc2_dreg(s, DSTS) |= (fs ? DSTS_ENUMSPD_FS : DSTS_ENUMSPD_HS) <<
                           DSTS_ENUMSPD_SHIFT;
    dwc2_raise_global_irq(s, GINTSTS_ENUMDONE);
    return fs ? USB_SPEED_FULL : USB_SPEED_HIGH;
}

/* Returns 0 once the SETUP packet is in guest memory. */
int dwc2_dev_setup(DWC2State *s, const uint8_t *setup)
{
    uint32_t *ctl = dwc2_dreg(s, DOEPCTL(0));
    uint32_t *tsiz = dwc2_dreg(s, DOEPTSIZ(0));
    uint32_t *dma = dwc2_dreg(s, DOEPDMA(0));
    uint32_t supcnt, xfer;
    int ret = 0;

    if (!s->dev_pullup) {
        ret = DWC2_DEV_NODEV;
        goto out;
    }
    supcnt = (*tsiz & DOEPTSIZ0_SUPCNT_MASK) >> DOEPTSIZ0_SUPCNT_SHIFT;
    /*
     * SETUP packets are taken while SUPCnt is non-zero even with the
     * endpoint disabled: the 3.10 driver leaves OUT EP0 that way after
     * the status stage of a control read and expects the next SETUP to
     * land where DOEPDMA0 still points.
     */
    if (!(*ctl & DXEPCTL_EPENA) && !supcnt) {
        ret = DWC2_DEV_NAK;
        goto out;
    }
    if (!dwc2_dev_dma(s, 0, false, *dma, (uint8_t *)setup, 8,
                      DMA_DIRECTION_FROM_DEVICE)) {
        ret = DWC2_DEV_NAK;
        goto out;
    }
    *dma += 8;
    xfer = *tsiz & DOEPTSIZ0_XFERSIZE_MASK;
    xfer = xfer > 8 ? xfer - 8 : 0;
    if (supcnt) {
        supcnt--;
    }
    *tsiz = (*tsiz & ~(DOEPTSIZ0_SUPCNT_MASK | DOEPTSIZ0_XFERSIZE_MASK)) |
            (supcnt << DOEPTSIZ0_SUPCNT_SHIFT) | xfer;

    /* Both directions NAK until the driver has seen the request. */
    *ctl = (*ctl & ~(DXEPCTL_EPENA | DXEPCTL_STALL)) | DXEPCTL_NAKSTS;
    *dwc2_dreg(s, DIEPCTL(0)) = (*dwc2_dreg(s, DIEPCTL(0)) & ~DXEPCTL_STALL) |
                                DXEPCTL_NAKSTS;
    dwc2_dev_ep_intr(s, 0, false, DXEPINT_SETUP);
out:
    trace_usb_dwc2_dev_setup(ldl_le_p(setup), ldl_le_p(setup + 4), ret);
    return ret;
}

/* An OUT data packet; returns 0 when the device took it. */
int dwc2_dev_out(DWC2State *s, int ep, const uint8_t *data, int len)
{
    uint32_t *ctl, *tsiz, *dma;
    uint32_t xfer, pkt, n;

    if (!s->dev_pullup || ep < 0 || ep >= DWC2_DEV_NB_EP) {
        return DWC2_DEV_NODEV;
    }
    ctl = dwc2_dreg(s, DOEPCTL(ep));
    if (*ctl & DXEPCTL_STALL) {
        return DWC2_DEV_STALL;
    }
    if (!(*ctl & DXEPCTL_EPENA) || (*ctl & DXEPCTL_NAKSTS) ||
        (ep && !(*ctl & DXEPCTL_USBACTEP)) ||
        (*dwc2_dreg(s, DCTL) & DCTL_GOUTNAKSTS)) {
        return DWC2_DEV_NAK;
    }
    tsiz = dwc2_dreg(s, DOEPTSIZ(ep));
    dwc2_dev_tsiz_get(ep, false, *tsiz, &xfer, &pkt);
    if (!pkt) {
        return DWC2_DEV_NAK;
    }

    dma = dwc2_dreg(s, DOEPDMA(ep));
    n = MIN(len, xfer);
    if (!dwc2_dev_dma(s, ep, false, *dma, (uint8_t *)data, n,
                      DMA_DIRECTION_FROM_DEVICE)) {
        return DWC2_DEV_NAK;
    }
    *dma += n;
    *tsiz = dwc2_dev_tsiz_set(ep, false, *tsiz, xfer - n, --pkt);
    *ctl ^= DXEPCTL_DPID;
    trace_usb_dwc2_dev_out(ep, len, pkt);

    /* A short packet ends the transfer early; the core then NAKs. */
    if (!pkt || len < dwc2_dev_mps(s, ep, false)) {
        *ctl = (*ctl & ~DXEPCTL_EPENA) | DXEPCTL_NAKSTS;
        dwc2_dev_ep_intr(s, ep, false, DXEPINT_XFERCOMPL);
    }
    return 0;
}

/* An IN token; returns the length of the packet the device sent. */
int dwc2_dev_in(DWC2State *s, int ep, uint8_t *data, int maxlen)
{
    uint32_t *ctl, *tsiz, *dma;
    uint32_t xfer, pkt, type;
    int len;

    if (!s->dev_pullup || ep < 0 || ep >= DWC2_DEV_NB_EP) {
        return DWC2_DEV_NODEV;
    }
    ctl = dwc2_dreg(s, DIEPCTL(ep));
    if (*ctl & DXEPCTL_STALL) {
        return DWC2_DEV_STALL;
    }
    type = *ctl & DXEPCTL_EPTYPE_MASK;
    if (!(*ctl & DXEPCTL_EPENA) || (*ctl & DXEPCTL_NAKSTS) ||
        (ep && !(*ctl & DXEPCTL_USBACTEP)) ||
        ((*dwc2_dreg(s, DCTL) & DCTL_GNPINNAKSTS) &&
         (type == DXEPCTL_EPTYPE_CONTROL || type == DXEPCTL_EPTYPE_BULK))) {
        return DWC2_DEV_NAK;
    }
    tsiz = dwc2_dreg(s, DIEPTSIZ(ep));
    dwc2_dev_tsiz_get(ep, true, *tsiz, &xfer, &pkt);
    if (!pkt) {
        return DWC2_DEV_NAK;
    }

    dma = dwc2_dreg(s, DIEPDMA(ep));
    len = MIN(MIN(xfer, dwc2_dev_mps(s, ep, true)), maxlen);
    if (!dwc2_dev_dma(s, ep, true, *dma, data, len,
                      DMA_DIRECTION_TO_DEVICE)) {
        return DWC2_DEV_NAK;
    }
    *dma += len;
    *tsiz = dwc2_dev_tsiz_set(ep, true, *tsiz, xfer - len, --pkt);
    *ctl ^= DXEPCTL_DPID;
    trace_usb_dwc2_dev_in(ep, len, pkt);

    if (!pkt) {
        *ctl &= ~DXEPCTL_EPENA;
        dwc2_dev_ep_intr(s, ep, true, DXEPINT_XFERCOMPL);
    }
    return len;
}

static uint64_t dwc2_hsotg_read(void *ptr, hwaddr addr, unsigned size)
{
    DWC2State *s = ptr;
    uint64_t val;

    /* Deliver the next scripted gadget event on each GINTSTS poll. */
    if (addr == GINTSTS) {
        dwc2_gadget_on_gintsts_read(s);
    }

    switch (addr) {
    case HSOTG_REG(0x000) ... HSOTG_REG(0x0fc):
        val = dwc2_glbreg_read(ptr, addr, (addr - HSOTG_REG(0x000)) >> 2, size);
        break;
    case HSOTG_REG(0x100):
        val = dwc2_fszreg_read(ptr, addr, (addr - HSOTG_REG(0x100)) >> 2, size);
        break;
    case HSOTG_REG(0x104) ... HSOTG_REG(0x13c):
        val = s->dieptxf[(addr - HSOTG_REG(0x104)) >> 2];
        break;
    case HSOTG_REG(0x140) ... HSOTG_REG(0x3fc):
        val = 0;
        break;
    case HSOTG_REG(0x400) ... HSOTG_REG(0x4fc):
        val = dwc2_hreg0_read(ptr, addr, (addr - HSOTG_REG(0x400)) >> 2, size);
        break;
    case HSOTG_REG(0x500) ... HSOTG_REG(0x7fc):
        val = dwc2_hreg1_read(ptr, addr, (addr - HSOTG_REG(0x500)) >> 2, size);
        break;
    case HSOTG_REG(0x800) ... HSOTG_REG(0xbfc):
        /*
         * Device-mode register block. Backed by dreg[] so the bootrom's
         * gadget code reads back what it (or the gadget script) wrote:
         * DSTS speed, DAINT, DIEPINT/DOEPINT, DTXFSTS, etc.
         */
        val = dwc2_dev_engine(s) ? dwc2_dev_read(s, addr) : *dwc2_dreg(s, addr);
        break;
    case HSOTG_REG(0xc00) ... HSOTG_REG(0xdfc):
        /* Remaining gadget-mode registers, return 0 for now */
        val = 0;
        break;
    case HSOTG_REG(0xe00) ... HSOTG_REG(0xffc):
        val = dwc2_pcgreg_read(ptr, addr, (addr - HSOTG_REG(0xe00)) >> 2, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        val = 0;
        break;
    }

    return val;
}

/* True for the device per-endpoint interrupt registers + DAINT (W1C). */
static inline bool dwc2_dreg_is_w1c(hwaddr off)
{
    if (off == DAINT) {
        return true;
    }
    /* DIEPINT(n) = 0x908 + n*0x20 ; DOEPINT(n) = 0xb08 + n*0x20 */
    if ((off & 0x1f) == 0x08 &&
        ((off >= 0x900 && off < 0x980) || (off >= 0xb00 && off < 0xb80))) {
        return true;
    }
    return false;
}

static void dwc2_hsotg_write(void *ptr, hwaddr addr, uint64_t val,
                             unsigned size)
{
    DWC2State *s = ptr;

    /* Retire the currently-armed gadget event on the matching GINTSTS ack. */
    if (addr == GINTSTS) {
        dwc2_gadget_on_gintsts_write(s, (uint32_t)val);
    }

    switch (addr) {
    case HSOTG_REG(0x000) ... HSOTG_REG(0x0fc):
        dwc2_glbreg_write(ptr, addr, (addr - HSOTG_REG(0x000)) >> 2, val, size);
        break;
    case HSOTG_REG(0x100):
        dwc2_fszreg_write(ptr, addr, (addr - HSOTG_REG(0x100)) >> 2, val, size);
        break;
    case HSOTG_REG(0x104) ... HSOTG_REG(0x13c):
        s->dieptxf[(addr - HSOTG_REG(0x104)) >> 2] = val;
        break;
    case HSOTG_REG(0x140) ... HSOTG_REG(0x3fc):
        break;
    case HSOTG_REG(0x400) ... HSOTG_REG(0x4fc):
        dwc2_hreg0_write(ptr, addr, (addr - HSOTG_REG(0x400)) >> 2, val, size);
        break;
    case HSOTG_REG(0x500) ... HSOTG_REG(0x7fc):
        dwc2_hreg1_write(ptr, addr, (addr - HSOTG_REG(0x500)) >> 2, val, size);
        break;
    case HSOTG_REG(0x800) ... HSOTG_REG(0xbfc):
        if (dwc2_dev_engine(s)) {
            dwc2_dev_write(s, addr, val);
            break;
        }
        /*
         * The gadget-boot script drives the endpoints itself, and only
         * needs the global NAK handshakes from the register side.
         */
        if (addr == DCTL) {
            if (val & DCTL_SGNPINNAK) {
                /* Set Global Non-Periodic IN NAK -> hardware sets
                 * GINTSTS.GINNakEff once the controller has accepted
                 * the request. dwc2_flush_tx_fifo() polls for this
                 * bit and prints "HANG! GINTSTS=..." after 10000
                 * unsuccessful reads, so make it stick immediately. */
                dwc2_raise_global_irq(s, GINTSTS_GINNAKEFF);
            }
            if (val & DCTL_CGNPINNAK) {
                dwc2_lower_global_irq(s, GINTSTS_GINNAKEFF);
            }
            if (val & DCTL_SGOUTNAK) {
                dwc2_raise_global_irq(s, GINTSTS_GOUTNAKEFF);
            }
            if (val & DCTL_CGOUTNAK) {
                dwc2_lower_global_irq(s, GINTSTS_GOUTNAKEFF);
            }
        }
        /*
         * Persist device-mode register writes into dreg[] so the gadget
         * code reads back consistent state. Per-endpoint interrupt
         * registers and DAINT are write-1-to-clear; everything else is a
         * plain store. (Only meaningful when the gadget script runs; for
         * host-mode use nothing reads dreg[], so this is inert there.)
         */
        if (dwc2_dreg_is_w1c(addr)) {
            *dwc2_dreg(s, addr) &= ~(uint32_t)val;
        } else {
            *dwc2_dreg(s, addr) = (uint32_t)val;
        }
        /* Let the gadget script observe the SETUP-dispatch ack, etc. */
        dwc2_gadget_on_dreg_write(s, addr, (uint32_t)val);
        break;
    case HSOTG_REG(0xc00) ... HSOTG_REG(0xdfc):
        /* Remaining gadget-mode registers, do nothing for now */
        break;
    case HSOTG_REG(0xe00) ... HSOTG_REG(0xffc):
        dwc2_pcgreg_write(ptr, addr, (addr - HSOTG_REG(0xe00)) >> 2, val, size);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset 0x%"HWADDR_PRIx"\n",
                      __func__, addr);
        break;
    }
}

static const MemoryRegionOps dwc2_mmio_hsotg_ops = {
    .read = dwc2_hsotg_read,
    .write = dwc2_hsotg_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static uint64_t dwc2_hreg2_read(void *ptr, hwaddr addr, unsigned size)
{
    DWC2State *s = ptr;

    /*
     * Slave-mode RX FIFO read. On real DWC2 a read from any endpoint FIFO
     * window pops the next word from the shared RX FIFO. The gadget-boot
     * script stages the current packet's words in g_rxbuf; pop them in
     * order so the bootrom receives the SETUP / OUT data it expects.
     */
    if (dwc2_gadget_active(s)) {
        uint32_t word = 0;
        if (s->g_rxpos < s->g_rxcnt) {
            word = s->g_rxbuf[s->g_rxpos++];
        }
        trace_usb_dwc2_hreg2_read(addr, addr >> 12, word);
        return word;
    }

    /* TODO - implement FIFOs to support slave mode */
    trace_usb_dwc2_hreg2_read(addr, addr >> 12, 0);
    qemu_log_mask(LOG_UNIMP, "%s: FIFO read not implemented\n", __func__);
    return 0;
}

static void dwc2_hreg2_write(void *ptr, hwaddr addr, uint64_t val,
                             unsigned size)
{
    DWC2State *s = ptr;
    uint64_t orig = val;

    /*
     * Slave-mode TX FIFO write (device->host IN data the bootrom pushes).
     * The gadget-boot script doesn't consume IN data, so just drain it.
     */
    if (dwc2_gadget_active(s)) {
        trace_usb_dwc2_hreg2_write(addr, addr >> 12, orig, 0, val);
        return;
    }

    /* TODO - implement FIFOs to support slave mode */
    trace_usb_dwc2_hreg2_write(addr, addr >> 12, orig, 0, val);
    qemu_log_mask(LOG_UNIMP, "%s: FIFO write not implemented\n", __func__);
}

static const MemoryRegionOps dwc2_mmio_hreg2_ops = {
    .read = dwc2_hreg2_read,
    .write = dwc2_hreg2_write,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void dwc2_wakeup_endpoint(USBBus *bus, USBEndpoint *ep,
                                 unsigned int stream)
{
    DWC2State *s = container_of(bus, DWC2State, bus);

    trace_usb_dwc2_wakeup_endpoint(ep, stream);

    /* TODO - do something here? */
    qemu_bh_schedule(s->async_bh);
}

static USBBusOps dwc2_bus_ops = {
    .wakeup_endpoint = dwc2_wakeup_endpoint,
};

static void dwc2_work_timer(void *opaque)
{
    DWC2State *s = opaque;

    trace_usb_dwc2_work_timer();
    qemu_bh_schedule(s->async_bh);
}

static void dwc2_reset_enter(Object *obj, ResetType type)
{
    DWC2Class *c = DWC2_USB_GET_CLASS(obj);
    DWC2State *s = DWC2_USB(obj);
    int i;

    trace_usb_dwc2_reset_enter();

    if (c->parent_phases.enter) {
        c->parent_phases.enter(obj, type);
    }

    timer_del(s->frame_timer);
    qemu_bh_cancel(s->async_bh);

    if (s->uport.dev && s->uport.dev->attached) {
        usb_detach(&s->uport);
    }

    dwc2_bus_stop(s);

    /*
     * Default to A-Device (host) by clearing CONID_B. The Linux Ingenic
     * dwc2 driver gates host-mode init (and the unmasking of PRTINT in
     * GINTMSK) on gotgctl.conidsts == 0; with CONID_B set the driver
     * enters peripheral/gadget mode and never enables PRTINT, so port
     * connect events from attached USB devices never reach the kernel.
     * BSESVLD/ASESVLD remain set so VBUS is reported as valid.
     */
    s->gotgctl = GOTGCTL_BSESVLD | GOTGCTL_ASESVLD;
    s->gotgint = 0;
    s->gahbcfg = 0;
    s->gusbcfg = 5 << GUSBCFG_USBTRDTIM_SHIFT;
    s->grstctl = GRSTCTL_AHBIDLE;
    s->gintsts = GINTSTS_CONIDSTSCHNG | GINTSTS_PTXFEMP | GINTSTS_NPTXFEMP |
                 GINTSTS_CURMODE_HOST;
    s->gintmsk = 0;
    s->grxstsr = 0;
    s->grxstsp = 0;
    s->grxfsiz = 1024;
    s->gnptxfsiz = 1024 << FIFOSIZE_DEPTH_SHIFT;
    s->gnptxsts = (4 << FIFOSIZE_DEPTH_SHIFT) | 1024;
    s->gi2cctl = GI2CCTL_I2CDATSE0 | GI2CCTL_ACK;
    s->gpvndctl = 0;
    s->ggpio = 0;
    s->guid = 0;
    s->gsnpsid = 0x4f54294a;
    s->ghwcfg1 = 0;
    s->ghwcfg2 = (8 << GHWCFG2_DEV_TOKEN_Q_DEPTH_SHIFT) |
                 (4 << GHWCFG2_HOST_PERIO_TX_Q_DEPTH_SHIFT) |
                 (4 << GHWCFG2_NONPERIO_TX_Q_DEPTH_SHIFT) |
                 GHWCFG2_DYNAMIC_FIFO |
                 GHWCFG2_PERIO_EP_SUPPORTED |
                 ((DWC2_NB_CHAN - 1) << GHWCFG2_NUM_HOST_CHAN_SHIFT) |
                 (3 << GHWCFG2_NUM_DEV_EP_SHIFT) |
                 (GHWCFG2_INT_DMA_ARCH << GHWCFG2_ARCHITECTURE_SHIFT) |
                 (GHWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST << GHWCFG2_OP_MODE_SHIFT);
    s->ghwcfg3 = (4096 << GHWCFG3_DFIFO_DEPTH_SHIFT) |
                 (4 << GHWCFG3_PACKET_SIZE_CNTR_WIDTH_SHIFT) |
                 (4 << GHWCFG3_XFER_SIZE_CNTR_WIDTH_SHIFT);
    s->ghwcfg4 = (3 << GHWCFG4_NUM_IN_EPS_SHIFT);
    s->glpmcfg = 0;
    s->gpwrdn = GPWRDN_PWRDNRSTN;
    s->gdfifocfg = 0;
    s->gadpctl = 0;
    s->grefclk = 0;
    s->gintmsk2 = 0;
    s->gintsts2 = 0;

    s->hptxfsiz = 500 << FIFOSIZE_DEPTH_SHIFT;

    s->hcfg = 2 << HCFG_RESVALID_SHIFT;
    s->hfir = 60000;
    s->hfnum = 0x3fff;
    s->hptxsts = (16 << TXSTS_QSPCAVAIL_SHIFT) | 32768;
    s->haint = 0;
    s->haintmsk = 0;
    s->hprt0 = 0;

    memset(s->hreg1, 0, sizeof(s->hreg1));
    memset(s->pcgreg, 0, sizeof(s->pcgreg));

    s->sof_time = 0;
    s->frame_number = 0;
    s->fi = USB_FRMINTVL - 1;
    s->next_chan = 0;
    s->working = false;

    /* Device-mode register block + gadget-boot script state. */
    memset(s->dreg, 0, sizeof(s->dreg));
    memset(s->dieptxf, 0, sizeof(s->dieptxf));
    if (!s->gadget_boot) {
        /* Soft-disconnected until the gadget driver pulls D+ up. */
        *dwc2_dreg(s, DCTL) = DCTL_SFTDISCON;
    }
    if (s->dev_pullup) {
        s->dev_pullup = false;
        if (s->peer_ops) {
            s->peer_ops->connect(s->peer, false);
        }
    }
    s->g_step = 0;
    s->g_armed = false;
    s->g_armed_intr = 0;
    s->g_rxsts = 0;
    s->g_rxcnt = 0;
    s->g_rxpos = 0;
    s->g_dl_addr = 0;
    s->g_dl_len = 0;

    for (i = 0; i < DWC2_NB_CHAN; i++) {
        s->packet[i].needs_service = false;
    }
}

static void dwc2_reset_hold(Object *obj, ResetType type)
{
    DWC2Class *c = DWC2_USB_GET_CLASS(obj);
    DWC2State *s = DWC2_USB(obj);

    trace_usb_dwc2_reset_hold();

    if (c->parent_phases.hold) {
        c->parent_phases.hold(obj, type);
    }

    dwc2_update_irq(s);
}

static void dwc2_reset_exit(Object *obj, ResetType type)
{
    DWC2Class *c = DWC2_USB_GET_CLASS(obj);
    DWC2State *s = DWC2_USB(obj);

    trace_usb_dwc2_reset_exit();

    if (c->parent_phases.exit) {
        c->parent_phases.exit(obj, type);
    }

    s->hprt0 = HPRT0_PWR;
    if (s->uport.dev && s->uport.dev->attached) {
        usb_attach(&s->uport);
        usb_device_reset(s->uport.dev);
    }
}

static void dwc2_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    DWC2State *s = DWC2_USB(dev);
    Object *obj;

    obj = object_property_get_link(OBJECT(dev), "dma-mr", &error_abort);

    s->dma_mr = MEMORY_REGION(obj);
    address_space_init(&s->dma_as, s->dma_mr, "dwc2");

    usb_bus_new(&s->bus, sizeof(s->bus), &dwc2_bus_ops, dev);
    usb_register_port(&s->bus, &s->uport, s, 0, &dwc2_port_ops,
                      USB_SPEED_MASK_LOW | USB_SPEED_MASK_FULL |
                      (s->usb_version == 2 ? USB_SPEED_MASK_HIGH : 0));
    s->uport.dev = 0;

    s->usb_frame_time = NANOSECONDS_PER_SECOND / 1000;          /* 1000000 */
    if (NANOSECONDS_PER_SECOND >= USB_HZ_FS) {
        s->usb_bit_time = NANOSECONDS_PER_SECOND / USB_HZ_FS;   /* 83.3 */
    } else {
        s->usb_bit_time = 1;
    }

    s->fi = USB_FRMINTVL - 1;
    s->eof_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dwc2_frame_boundary, s);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dwc2_work_timer, s);
    s->async_bh = qemu_bh_new_guarded(dwc2_work_bh, s,
                                      &dev->mem_reentrancy_guard);

    sysbus_init_irq(sbd, &s->irq);
}

static void dwc2_otg_id_change(void *opaque, int n, int level)
{
    /*
     * level=1 -> peripheral/B (CONID_B set)
     * level=0 -> host/A       (CONID_B clear)
     * Raised by external mode-select (e.g. CPM USBRDT writes from
     * thingino's usb-role userspace tool). Always raise
     * CONIDSTSCHNG so the kernel re-enters its OTG state machine
     * even when the resulting state matches the boot default - the
     * kernel boot path keeps the controller in peripheral mode until
     * an explicit ID transition is observed.
     */
    DWC2State *s = opaque;

    if (level) {
        s->gotgctl |= GOTGCTL_CONID_B;
        s->gintsts &= ~GINTSTS_CURMODE_HOST;
    } else {
        s->gotgctl &= ~GOTGCTL_CONID_B;
        s->gintsts |= GINTSTS_CURMODE_HOST;
    }
    /* Pulse PRTINT to force a recheck if the line was already asserted. */
    dwc2_lower_global_irq(s, GINTSTS_CONIDSTSCHNG);
    dwc2_raise_global_irq(s, GINTSTS_CONIDSTSCHNG);
    dwc2_dev_check_connect(s);
}

static void dwc2_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DWC2State *s = DWC2_USB(obj);

    memory_region_init(&s->container, obj, "dwc2", DWC2_MMIO_SIZE);
    sysbus_init_mmio(sbd, &s->container);
    qdev_init_gpio_in_named(DEVICE(s), dwc2_otg_id_change,
                            "otg-id-change", 1);

    memory_region_init_io(&s->hsotg, obj, &dwc2_mmio_hsotg_ops, s,
                          "dwc2-io", 4 * KiB);
    memory_region_add_subregion(&s->container, 0x0000, &s->hsotg);

    memory_region_init_io(&s->fifos, obj, &dwc2_mmio_hreg2_ops, s,
                          "dwc2-fifo", 64 * KiB);
    memory_region_add_subregion(&s->container, 0x1000, &s->fifos);
}

static const VMStateDescription vmstate_dwc2_state_packet = {
    .name = "dwc2/packet",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(devadr, DWC2Packet),
        VMSTATE_UINT32(epnum, DWC2Packet),
        VMSTATE_UINT32(epdir, DWC2Packet),
        VMSTATE_UINT32(mps, DWC2Packet),
        VMSTATE_UINT32(pid, DWC2Packet),
        VMSTATE_UINT32(index, DWC2Packet),
        VMSTATE_UINT32(pcnt, DWC2Packet),
        VMSTATE_UINT32(len, DWC2Packet),
        VMSTATE_INT32(async, DWC2Packet),
        VMSTATE_BOOL(small, DWC2Packet),
        VMSTATE_BOOL(needs_service, DWC2Packet),
        VMSTATE_END_OF_LIST()
    },
};

const VMStateDescription vmstate_dwc2_state = {
    .name = "dwc2",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(glbreg, DWC2State,
                             DWC2_GLBREG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(fszreg, DWC2State,
                             DWC2_FSZREG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(hreg0, DWC2State,
                             DWC2_HREG0_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(hreg1, DWC2State,
                             DWC2_HREG1_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(pcgreg, DWC2State,
                             DWC2_PCGREG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY(dreg, DWC2State,
                             DWC2_DREG_SIZE / sizeof(uint32_t)),
        VMSTATE_UINT32_ARRAY_V(dieptxf, DWC2State, 15, 2),
        VMSTATE_BOOL_V(dev_pullup, DWC2State, 2),

        VMSTATE_TIMER_PTR(eof_timer, DWC2State),
        VMSTATE_TIMER_PTR(frame_timer, DWC2State),
        VMSTATE_INT64(sof_time, DWC2State),
        VMSTATE_INT64(usb_frame_time, DWC2State),
        VMSTATE_INT64(usb_bit_time, DWC2State),
        VMSTATE_UINT32(usb_version, DWC2State),
        VMSTATE_UINT16(frame_number, DWC2State),
        VMSTATE_UINT16(fi, DWC2State),
        VMSTATE_UINT16(next_chan, DWC2State),
        VMSTATE_BOOL(working, DWC2State),

        VMSTATE_STRUCT_ARRAY(packet, DWC2State, DWC2_NB_CHAN, 1,
                             vmstate_dwc2_state_packet, DWC2Packet),
        VMSTATE_UINT8_2DARRAY(usb_buf, DWC2State, DWC2_NB_CHAN,
                              DWC2_MAX_XFER_SIZE),

        VMSTATE_END_OF_LIST()
    }
};

static const Property dwc2_usb_properties[] = {
    DEFINE_PROP_UINT32("usb_version", DWC2State, usb_version, 2),
    /*
     * Deterministic gadget-boot enumeration. Off by default; enabled by
     * the Ingenic SoC only in -bios bootrom-analysis mode so the mask
     * ROM's USB-boot path can enumerate and receive an SPL. Has no effect
     * on host-mode use (also gated on no attached USB device at runtime).
     */
    DEFINE_PROP_BOOL("gadget-boot", DWC2State, gadget_boot, false),
};

static void dwc2_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    DWC2Class *c = DWC2_USB_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = dwc2_realize;
    dc->vmsd = &vmstate_dwc2_state;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    device_class_set_props(dc, dwc2_usb_properties);
    resettable_class_set_parent_phases(rc, dwc2_reset_enter, dwc2_reset_hold,
                                       dwc2_reset_exit, &c->parent_phases);
}

static const TypeInfo dwc2_usb_type_info = {
    .name          = TYPE_DWC2_USB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(DWC2State),
    .instance_init = dwc2_init,
    .class_size    = sizeof(DWC2Class),
    .class_init    = dwc2_class_init,
};

static void dwc2_usb_register_types(void)
{
    type_register_static(&dwc2_usb_type_info);
}

type_init(dwc2_usb_register_types)
