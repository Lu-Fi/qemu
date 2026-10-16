/*
 * Ingenic SSI, the SPI master of the T and A series
 *
 * Two 128-word FIFOs feeding a shift register, one frame of FLEN + 2 bits
 * per word. Frames go out on the SSI bus a byte at a time when the frame
 * is whole bytes, else as one transfer; LOOP turns the frame straight
 * around and leaves the pins alone. CE0 and CE1 are the "cs" outputs, at
 * pin level; a peripheral given cs=0 or cs=1 on the bus is wired to them
 * at reset.
 *
 * The CPU is taken to be much faster than the SPI clock, as on the boards:
 * words written to DR queue up, and the shift register works through them
 * before the CPU next reads an SSI register, after it writes any other
 * one, or once it has left the SSI alone for a while. A burst of writes
 * therefore goes out back to back under one chip select, whatever the
 * host's timing (a QEMU timer of a few microseconds would fire between
 * two of the guest's writes).
 *
 * As measured on the T31 and T41 (LOOP mode):
 *  - TFHE is TX count < TTRG * 8 (< 1 for TTRG 0), not "at or below" as
 *    the manuals have it; RFHF is RX count >= RTRG * 8 (>= 1 for 0).
 *  - With UNFIN set, an empty TX FIFO holds the transfer open and sets
 *    UNDR, which comes back if cleared while the FIFO is still empty.
 *    Nothing moves while UNDR is set: words written wait, and clearing
 *    UNFIN does not end the transfer. Clearing UNDR resumes it, or ends it
 *    if UNFIN is clear. With EACLRUN the write that makes the TX FIFO
 *    non-empty clears UNDR.
 *  - Clearing SSIE ends a transfer at once (END set) and keeps the FIFOs
 *    and UNDR.
 *  - The 129th word into a full RX FIFO sets OVER and is lost. Reading the
 *    empty RX FIFO returns the FIFO RAM at the read pointer, unpopped.
 *  - Receive-only (RFINE, UNFIN clear) sends zeros, RCNT frames with VRCNT.
 *    Here it also waits for room in the RX FIFO rather than overrun it.
 *  - XBurst2: SR[31:24], documented reserved, holds 0x30 while the TX DMA
 *    request is idle and 0x06 while the RX one is.
 * The T32/T33 driver also waits on END through CR0.FIE and clears END by
 * writing 0 to it; the "fie" property gives that pair.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/ingenic-ssi.h"
#include "migration/vmstate.h"

#define SSI_DR          0x00
#define SSI_CR0         0x04
#define SSI_CR1         0x08
#define SSI_SR          0x0c
#define SSI_ITR         0x10
#define SSI_ICR         0x14
#define SSI_GR          0x18
#define SSI_RCNT        0x1c
#define SSI_SPEN        0x20
#define SSI_SPLOAD      0x24
#define SSI_SPCNT       0x28
#define SSI_IOSIZE      0x1000

#define CR0_FIE         BIT(20)
#define CR0_TENDIAN(v)  extract32(v, 18, 2)
#define CR0_RENDIAN(v)  extract32(v, 16, 2)
#define CR0_SSIE        BIT(15)
#define CR0_TIE         BIT(14)
#define CR0_RIE         BIT(13)
#define CR0_TEIE        BIT(12)
#define CR0_REIE        BIT(11)
#define CR0_LOOP        BIT(10)
#define CR0_RFINE       BIT(9)
#define CR0_RFINC       BIT(8)
#define CR0_EACLRUN     BIT(7)
#define CR0_FSEL        BIT(6)
#define CR0_VRCNT       BIT(4)
#define CR0_TFLUSH      BIT(2)
#define CR0_RFLUSH      BIT(1)
#define CR0_DISREV      BIT(0)
#define CR0_MASK        0x000fffd9

#define CR1_FRMHL(n)    BIT(30 + (n))
#define CR1_ITFRM       BIT(24)
#define CR1_UNFIN       BIT(23)
#define CR1_FMAT(v)     extract32(v, 20, 2)
#define CR1_TTRG(v)     extract32(v, 16, 4)
#define CR1_RTRG(v)     extract32(v, 8, 4)
#define CR1_FLEN(v)     extract32(v, 3, 5)
#define CR1_MASK        0xfdbffffb
#define CR1_RESET       0x00087830

#define SR_END          BIT(7)
#define SR_BUSY         BIT(6)
#define SR_TFF          BIT(5)
#define SR_RFE          BIT(4)
#define SR_TFHE         BIT(3)
#define SR_RFHF         BIT(2)
#define SR_UNDR         BIT(1)
#define SR_OVER         BIT(0)
#define SR_TX_IDLE      0x30000000      /* XBurst2: no TX DMA request */
#define SR_RX_IDLE      0x06000000      /* ... no RX one */

#define ITR_IVLTM(v)    extract32(v, 0, 15)

#define FMAT_SPI        0
#define FMAT_SSP        1

/* Left alone this long after a DR write, the SSI catches up by itself */
#define SSI_QUIET_NS    (1000 * SCALE_US)

static bool ssi_tfhe(IngenicSsiState *s)
{
    unsigned t = CR1_TTRG(s->cr1);

    return s->tx_n < (t ? t * 8 : 1);
}

static bool ssi_rfhf(IngenicSsiState *s)
{
    unsigned r = CR1_RTRG(s->cr1);

    return s->rx_n >= (r ? r * 8 : 1);
}

static bool ssi_tdreq(IngenicSsiState *s)
{
    return (s->cr0 & (CR0_SSIE | CR0_TIE)) == CR0_SSIE && ssi_tfhe(s);
}

static bool ssi_rdreq(IngenicSsiState *s)
{
    return (s->cr0 & (CR0_SSIE | CR0_RIE)) == CR0_SSIE && ssi_rfhf(s);
}

static void ssi_update(IngenicSsiState *s)
{
    bool irq = ((s->cr0 & CR0_TIE) && ssi_tfhe(s)) ||
               ((s->cr0 & CR0_RIE) && ssi_rfhf(s)) ||
               ((s->cr0 & CR0_TEIE) && s->undr) ||
               ((s->cr0 & CR0_REIE) && s->over) ||
               ((s->cr0 & CR0_FIE) && s->end);

    qemu_set_irq(s->irq, irq);
    qemu_set_irq(s->dreq[0], ssi_tdreq(s));
    qemu_set_irq(s->dreq[1], ssi_rdreq(s));
}

/* Drive CE0/CE1: FSEL picks the pin, FRMHL its active level */
static void ssi_drive_cs(IngenicSsiState *s)
{
    for (int n = 0; n < 2; n++) {
        bool active = s->sel && !(s->cr0 & CR0_LOOP) &&
                      !!(s->cr0 & CR0_FSEL) == n;
        bool level = active == !!(s->cr1 & CR1_FRMHL(n));

        if (level != s->cs_level[n]) {
            s->cs_level[n] = level;
            qemu_set_irq(s->cs[n], level);
        }
    }
}

static void ssi_select(IngenicSsiState *s, bool sel)
{
    s->sel = sel;
    ssi_drive_cs(s);
}

static int ssi_frame_bits(IngenicSsiState *s)
{
    return MIN(CR1_FLEN(s->cr1) + 2, 32);
}

static int64_t ssi_frame_ns(IngenicSsiState *s)
{
    return muldiv64(2 * (s->gr + 1) * ssi_frame_bits(s),
                    NANOSECONDS_PER_SECOND, s->devclk);
}

/*
 * The order a frame's bits take on the wire, as measured: the frame is
 * cut into byte lanes from bit 0 up, the top lane holding what is left
 * over; endian 0 and 1 send the top lane first, 2 and 3 the bottom one,
 * and 1 and 2 send each lane LSB first. The wire value holds the first
 * bit at its top.
 */
static int ssi_wire_bit(int k, int nlanes, int n, int endian, int i)
{
    int lane = endian < 2 ? nlanes - 1 - k : k;
    int lo = lane * 8, w = MIN(8, n - lo);

    return endian == 1 || endian == 2 ? lo + i : lo + w - 1 - i;
}

static uint32_t ssi_to_wire(uint32_t val, int n, int endian)
{
    int nlanes = DIV_ROUND_UP(n, 8);
    uint32_t wire = 0;

    for (int k = 0; k < nlanes; k++) {
        int lane = endian < 2 ? nlanes - 1 - k : k;

        for (int i = 0; i < MIN(8, n - lane * 8); i++) {
            wire = wire << 1 | extract32(val, ssi_wire_bit(k, nlanes, n,
                                                           endian, i), 1);
        }
    }
    return wire;
}

static uint32_t ssi_from_wire(uint32_t wire, int n, int endian)
{
    int nlanes = DIV_ROUND_UP(n, 8), bit = n;
    uint32_t val = 0;

    for (int k = 0; k < nlanes; k++) {
        int lane = endian < 2 ? nlanes - 1 - k : k;

        for (int i = 0; i < MIN(8, n - lane * 8); i++) {
            val |= extract32(wire, --bit, 1)
                   << ssi_wire_bit(k, nlanes, n, endian, i);
        }
    }
    return val;
}

static uint32_t ssi_bus_frame(IngenicSsiState *s, uint32_t wire, int n)
{
    uint32_t rx = 0;

    if (n % 8) {
        return ssi_transfer(s->bus, wire) & MAKE_64BIT_MASK(0, n);
    }
    for (int i = n - 8; i >= 0; i -= 8) {
        rx = rx << 8 | (ssi_transfer(s->bus, extract32(wire, i, 8)) & 0xff);
    }
    return rx;
}

/* One frame out of the TX FIFO (zeros when receiving only) and one in */
static void ssi_frame(IngenicSsiState *s)
{
    int n = ssi_frame_bits(s);
    uint32_t tx = 0, wire;

    if (s->tx_n) {
        tx = s->tx[s->tx_rd];
        s->tx_rd = (s->tx_rd + 1) % INGENIC_SSI_FIFO;
        s->tx_n--;
    } else {
        s->rcvd++;
    }
    wire = ssi_to_wire(tx & MAKE_64BIT_MASK(0, n), n, CR0_TENDIAN(s->cr0));
    if (!(s->cr0 & CR0_LOOP)) {
        if (CR1_FMAT(s->cr1) == FMAT_SSP) {
            /* The SSP frame pulse selects the peripheral for one frame */
            ssi_select(s, true);
            wire = ssi_bus_frame(s, wire, n);
            ssi_select(s, false);
        } else {
            wire = ssi_bus_frame(s, wire, n);
        }
    }
    if (s->cr0 & CR0_DISREV) {
        return;
    }
    if (s->rx_n == INGENIC_SSI_FIFO) {
        s->over = true;
        return;
    }
    s->rx[(s->rx_rd + s->rx_n) % INGENIC_SSI_FIFO] =
        ssi_from_wire(wire, n, CR0_RENDIAN(s->cr0));
    s->rx_n++;
}

/* Something to clock: TX data, or a receive-only frame with room for it */
static bool ssi_have_frame(IngenicSsiState *s)
{
    if (s->tx_n) {
        return true;
    }
    return (s->cr0 & (CR0_RFINE | CR0_RFINC | CR0_DISREV)) == CR0_RFINE &&
           !(s->cr1 & CR1_UNFIN) && s->rx_n < INGENIC_SSI_FIFO &&
           (!(s->cr0 & CR0_VRCNT) || s->rcvd < s->rcnt);
}

static void ssi_finish(IngenicSsiState *s)
{
    s->xfer = false;
    s->stalled = false;
    s->end = true;
    ssi_select(s, false);
}

/* Run the shift register until it waits on the CPU */
static void ssi_settle(IngenicSsiState *s)
{
    bool interval = ITR_IVLTM(s->itr) && CR1_FMAT(s->cr1) == FMAT_SPI;

    if (!(s->cr0 & CR0_SSIE)) {
        return;
    }
    while (!(s->stalled && s->undr)) {
        if (!ssi_have_frame(s)) {
            if (!s->xfer) {
                return;
            }
            if (s->cr1 & CR1_UNFIN) {
                s->stalled = true;
                s->undr = true;
            } else {
                ssi_finish(s);
            }
            return;
        }
        if (!s->xfer) {
            s->xfer = true;
            s->end = false;
            s->group = 0;
            if (CR1_FMAT(s->cr1) >= 2) {
                qemu_log_mask(LOG_UNIMP, "ingenic-ssi: Microwire frames are "
                              "sent as SPI ones\n");
            }
        } else if (interval && !s->group && !(s->cr1 & CR1_ITFRM)) {
            /* CE drops for the interval between frame groups */
            ssi_select(s, false);
        }
        s->stalled = false;
        if (CR1_FMAT(s->cr1) != FMAT_SSP) {
            ssi_select(s, true);
        }
        ssi_frame(s);
        if (interval) {
            s->group = (s->group + 1) % (s->icr + 1);
        }
    }
}

static bool ssi_pending(IngenicSsiState *s)
{
    return (s->cr0 & CR0_SSIE) && !(s->stalled && s->undr) &&
           (ssi_have_frame(s) || s->stalled);
}

static void ssi_tick(void *opaque)
{
    IngenicSsiState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t due = s->last_write_ns +
                  MAX(SSI_QUIET_NS, ssi_frame_ns(s) * (s->tx_n + 1));

    if (now < due) {
        timer_mod(s->timer, due);
        return;
    }
    ssi_settle(s);
    ssi_update(s);
}

/* After a DR write: catch up later if the CPU does not come back first */
static void ssi_wrote(IngenicSsiState *s)
{
    s->last_write_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (ssi_pending(s) && !timer_pending(s->timer)) {
        timer_mod(s->timer, s->last_write_ns + SSI_QUIET_NS);
    }
}

static uint32_t ssi_sr(IngenicSsiState *s)
{
    uint32_t sr = s->tx_n << 16 | s->rx_n << 8;

    sr |= s->end ? SR_END : 0;
    sr |= s->stalled ? SR_BUSY : 0;
    sr |= s->tx_n == INGENIC_SSI_FIFO ? SR_TFF : 0;
    sr |= s->rx_n ? 0 : SR_RFE;
    sr |= ssi_tfhe(s) ? SR_TFHE : 0;
    sr |= ssi_rfhf(s) ? SR_RFHF : 0;
    sr |= s->undr ? SR_UNDR : 0;
    sr |= s->over ? SR_OVER : 0;
    if (s->xburst2) {
        sr |= ssi_tdreq(s) ? 0 : SR_TX_IDLE;
        sr |= ssi_rdreq(s) ? 0 : SR_RX_IDLE;
    }
    return sr;
}

/* A register as read, without side effects; false if there is none */
static bool ssi_reg(IngenicSsiState *s, hwaddr reg, uint32_t *v)
{
    switch (reg) {
    case SSI_CR0:
        *v = s->cr0;
        return true;
    case SSI_CR1:
        *v = s->cr1;
        return true;
    case SSI_SR:
        *v = ssi_sr(s);
        return true;
    case SSI_ITR:
        *v = s->itr;
        return true;
    case SSI_ICR:
        *v = s->icr;
        return true;
    case SSI_GR:
        *v = s->gr;
        return true;
    case SSI_RCNT:
        *v = s->rcnt;
        return true;
    case SSI_SPEN:
        *v = s->xburst2 ? s->spen : 0;
        return true;
    case SSI_SPLOAD:
        *v = 0;
        return true;
    case SSI_SPCNT:
        *v = s->xburst2 ? s->spcnt : 0;
        return true;
    }
    return false;
}

static uint64_t ingenic_ssi_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicSsiState *s = opaque;
    uint32_t v = 0;

    ssi_settle(s);
    if (offset < 4) {
        /* Only an access at the register's own address reaches the FIFO */
        if (!offset) {
            v = s->rx[s->rx_rd];
            if (s->rx_n) {
                s->rx_rd = (s->rx_rd + 1) % INGENIC_SSI_FIFO;
                s->rx_n--;
            }
        }
    } else if (ssi_reg(s, offset & ~3, &v)) {
        v >>= (offset & 3) * 8;
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-ssi: read of 0x%"
                      HWADDR_PRIx "\n", offset);
    }
    ssi_update(s);
    return v;
}

static void ingenic_ssi_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IngenicSsiState *s = opaque;
    uint32_t v = value, cur;

    if (offset < 4) {
        if (!offset && s->tx_n < INGENIC_SSI_FIFO) {
            if (!s->tx_n && (s->cr0 & CR0_EACLRUN)) {
                s->undr = false;
            }
            s->tx[(s->tx_rd + s->tx_n) % INGENIC_SSI_FIFO] = v;
            s->tx_n++;
        }
        ssi_wrote(s);
        ssi_update(s);
        return;
    }
    if (size < 4 && ssi_reg(s, offset & ~3, &cur)) {
        /* Narrow writes update their bytes of the register */
        v = deposit32(cur, (offset & 3) * 8, size * 8, v);
        offset &= ~3;
    }
    switch (offset) {
    case SSI_CR0:
        if (!(v & CR0_SSIE) && s->xfer) {
            ssi_finish(s);
        } else if ((v & CR0_SSIE) && !(s->cr0 & CR0_SSIE)) {
            s->rcvd = 0;
        }
        s->cr0 = v & (CR0_MASK | (s->fie ? CR0_FIE : 0));
        if (v & CR0_TFLUSH) {
            s->tx_rd = s->tx_n = 0;
        }
        if (v & CR0_RFLUSH) {
            s->rx_rd = s->rx_n = 0;
        }
        ssi_drive_cs(s);
        break;
    case SSI_CR1:
        s->cr1 = v & CR1_MASK;
        ssi_drive_cs(s);
        break;
    case SSI_SR:
        if (!(v & SR_UNDR)) {
            s->undr = false;
        }
        if (!(v & SR_OVER)) {
            s->over = false;
        }
        if (s->fie && !(v & SR_END)) {
            s->end = false;
        }
        break;
    case SSI_ITR:
        s->itr = v & 0xffff;
        break;
    case SSI_ICR:
        s->icr = v & 7;
        break;
    case SSI_GR:
        s->gr = v & 0xff;
        break;
    case SSI_RCNT:
        s->rcnt = v & 0xffff;
        s->rcvd = 0;
        break;
    case SSI_SPEN:
        s->spen = s->xburst2 ? v & 0x11 : 0;
        break;
    case SSI_SPLOAD:
        break;
    case SSI_SPCNT:
        s->spcnt = s->xburst2 ? v : 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-ssi: write of 0x%x to 0x%"
                      HWADDR_PRIx "\n", v, offset);
        return;
    }
    ssi_settle(s);
    ssi_update(s);
}

static const MemoryRegionOps ingenic_ssi_ops = {
    .read = ingenic_ssi_read,
    .write = ingenic_ssi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void ingenic_ssi_reset_hold(Object *obj, ResetType type)
{
    IngenicSsiState *s = INGENIC_SSI(obj);

    timer_del(s->timer);
    s->cr0 = 0;
    s->cr1 = CR1_RESET;
    s->itr = s->icr = s->gr = s->rcnt = 0;
    s->spen = s->spcnt = 0;
    s->end = true;
    s->undr = s->over = false;
    s->tx_rd = s->tx_n = s->rx_rd = s->rx_n = 0;
    s->xfer = s->stalled = s->sel = false;
    s->group = s->rcvd = 0;
}

static void ingenic_ssi_reset_exit(Object *obj, ResetType type)
{
    IngenicSsiState *s = INGENIC_SSI(obj);
    DeviceState *dev = DEVICE(obj);

    /* Peripherals added with -device find their CE pin here */
    for (int n = 0; n < 2; n++) {
        DeviceState *p = ssi_get_cs(s->bus, n);

        if (p && !qdev_get_gpio_out_connector(dev, "cs", n) &&
            object_property_find(OBJECT(p), SSI_GPIO_CS "[0]")) {
            qdev_connect_gpio_out_named(dev, "cs", n,
                                        qdev_get_gpio_in_named(p, SSI_GPIO_CS,
                                                               0));
        }
        s->cs_level[n] = true;
        qemu_set_irq(s->cs[n], true);
    }
    ssi_update(s);
}

static void ingenic_ssi_realize(DeviceState *dev, Error **errp)
{
    IngenicSsiState *s = INGENIC_SSI(dev);

    if (!s->devclk) {
        error_setg(errp, "ingenic-ssi: devclk must not be 0");
        return;
    }
    s->bus = ssi_create_bus(dev, s->bus_name);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ssi_tick, s);
}

static void ingenic_ssi_init(Object *obj)
{
    IngenicSsiState *s = INGENIC_SSI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ssi_ops, s,
                          TYPE_INGENIC_SSI, SSI_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->cs, "cs", 2);
    qdev_init_gpio_out_named(DEVICE(obj), s->dreq, "dreq", 2);
}

static const VMStateDescription vmstate_ingenic_ssi = {
    .name = TYPE_INGENIC_SSI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr0, IngenicSsiState),
        VMSTATE_UINT32(cr1, IngenicSsiState),
        VMSTATE_UINT32(itr, IngenicSsiState),
        VMSTATE_UINT32(icr, IngenicSsiState),
        VMSTATE_UINT32(gr, IngenicSsiState),
        VMSTATE_UINT32(rcnt, IngenicSsiState),
        VMSTATE_UINT32(spen, IngenicSsiState),
        VMSTATE_UINT32(spcnt, IngenicSsiState),
        VMSTATE_BOOL(end, IngenicSsiState),
        VMSTATE_BOOL(undr, IngenicSsiState),
        VMSTATE_BOOL(over, IngenicSsiState),
        VMSTATE_UINT32_ARRAY(tx, IngenicSsiState, INGENIC_SSI_FIFO),
        VMSTATE_UINT32_ARRAY(rx, IngenicSsiState, INGENIC_SSI_FIFO),
        VMSTATE_UINT32(tx_rd, IngenicSsiState),
        VMSTATE_UINT32(tx_n, IngenicSsiState),
        VMSTATE_UINT32(rx_rd, IngenicSsiState),
        VMSTATE_UINT32(rx_n, IngenicSsiState),
        VMSTATE_BOOL(xfer, IngenicSsiState),
        VMSTATE_BOOL(stalled, IngenicSsiState),
        VMSTATE_BOOL(sel, IngenicSsiState),
        VMSTATE_BOOL_ARRAY(cs_level, IngenicSsiState, 2),
        VMSTATE_UINT32(group, IngenicSsiState),
        VMSTATE_UINT32(rcvd, IngenicSsiState),
        VMSTATE_INT64(last_write_ns, IngenicSsiState),
        VMSTATE_TIMER_PTR(timer, IngenicSsiState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_ssi_props[] = {
    DEFINE_PROP_STRING("bus-name", IngenicSsiState, bus_name),
    DEFINE_PROP_UINT32("devclk", IngenicSsiState, devclk, 50000000),
    DEFINE_PROP_BOOL("xburst2", IngenicSsiState, xburst2, false),
    DEFINE_PROP_BOOL("fie", IngenicSsiState, fie, false),
};

static void ingenic_ssi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_ssi_realize;
    dc->vmsd = &vmstate_ingenic_ssi;
    device_class_set_props(dc, ingenic_ssi_props);
    rc->phases.hold = ingenic_ssi_reset_hold;
    rc->phases.exit = ingenic_ssi_reset_exit;
}

static const TypeInfo ingenic_ssi_info = {
    .name          = TYPE_INGENIC_SSI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicSsiState),
    .instance_init = ingenic_ssi_init,
    .class_init    = ingenic_ssi_class_init,
};

static void ingenic_ssi_register_types(void)
{
    type_register_static(&ingenic_ssi_info);
}

type_init(ingenic_ssi_register_types)
