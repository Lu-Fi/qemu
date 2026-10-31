/*
 * Ingenic UART
 *
 * A 16550 at a 4-byte register stride with Ingenic's changes, as measured
 * on a T31X and a T41NQ:
 *  - a 127-byte transmit FIFO counted by UTCR (without the byte on the
 *    line) and a 64-byte receive FIFO counted by URCR;
 *  - the transmit data request is a level, set while 32 bytes or fewer
 *    wait, and reading UIIR does not clear it;
 *  - receive triggers of 1, 8, 16 and 32 bytes, and a receive timeout of
 *    4 x word length + 12 bit times from the last start bit, enabled by
 *    UIER.RTOIE rather than the data-ready enable;
 *  - a bit takes UMR (64 if 0) cycles of the divided 24 MHz clock, one
 *    more where UACR has the bit's position set; the receiver needs four;
 *  - ULSR.FIFOE flags an overrun, or a break byte still in the FIFO;
 *  - UFCR.UME gates both directions.
 * In DMA mode (UFCR.DME) the "dreq" outputs request a transfer: TX while
 * TDRQ is set and UIER.TDRIE clear, RX at the trigger level unless FIFOE
 * is set, which raises the line status interrupt instead (the manual's
 * account; not measured).
 * Bytes take the line rate unless the "line-rate" property is off.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/char/ingenic-uart.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "chardev/char-serial.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

#define UART_RBR        0x00    /* THR on write; DLL with LCR.DLAB */
#define UART_IER        0x04    /* DLH with LCR.DLAB */
#define UART_IIR        0x08    /* FCR on write */
#define UART_LCR        0x0c
#define UART_MCR        0x10
#define UART_LSR        0x14
#define UART_MSR        0x18
#define UART_SPR        0x1c
#define UART_ISR        0x20
#define UART_UMR        0x24
#define UART_UACR       0x28
#define UART_RCR        0x40
#define UART_TCR        0x44

#define IER_RDRIE       0x01
#define IER_TDRIE       0x02
#define IER_RLSIE       0x04
#define IER_MSIE        0x08
#define IER_RTOIE       0x10

#define IIR_NO_INT      0x01
#define IIR_MSI         0x00
#define IIR_THRI        0x02
#define IIR_RDI         0x04
#define IIR_RLSI        0x06
#define IIR_CTI         0x0c
#define IIR_FIFO        0xc0

#define FCR_FME         0x01
#define FCR_RFRT        0x02
#define FCR_TFRT        0x04
#define FCR_DME         0x08
#define FCR_UME         0x10
#define FCR_RDTR        0xc0

#define LCR_WLS         0x03
#define LCR_STOP        0x04
#define LCR_PARE        0x08
#define LCR_PARM        0x10
#define LCR_SBK         0x40
#define LCR_DLAB        0x80

#define MCR_MASK        0xdf
#define MCR_LOOP        0x10

#define LSR_DR          0x01
#define LSR_OE          0x02
#define LSR_BI          0x10
#define LSR_TDRQ        0x20
#define LSR_TEMT        0x40
#define LSR_FIFOE       0x80
#define LSR_ERRORS      0x1e

#define MSR_DELTAS      0x0f
#define MSR_TERI        0x04
#define MSR_RI          0x40

#define TX_FIFO_SIZE    127
#define RX_FIFO_SIZE    64
#define TDRQ_LEVEL      32

static const uint8_t rx_trigger[4] = { 1, 8, 16, 32 };

static void uart_tx_kick(IngenicUartState *s);

static bool uart_fifo_mode(IngenicUartState *s)
{
    return s->fcr & FCR_FME;
}

/* The divided clock runs; without line-rate pacing nothing waits for it */
static bool uart_clock_runs(IngenicUartState *s)
{
    return (s->fcr & FCR_UME) && (s->divider || !s->line_rate);
}

static bool uart_rx_samples(IngenicUartState *s)
{
    return uart_clock_runs(s) && (s->umr == 0 || s->umr >= 4);
}

static bool uart_rx_listens(IngenicUartState *s)
{
    return uart_rx_samples(s) && !(s->mcr & MCR_LOOP);
}

static unsigned uart_word_length(IngenicUartState *s)
{
    return 5 + (s->lcr & LCR_WLS);
}

/* Start, data, parity and stop bits, in half bits (5-bit words stop at 1.5) */
static unsigned uart_frame_halfbits(IngenicUartState *s)
{
    unsigned wl = uart_word_length(s);
    unsigned h = 2 * (1 + wl + !!(s->lcr & LCR_PARE));

    if (s->lcr & LCR_STOP) {
        return h + (wl == 5 ? 3 : 4);
    }
    return h + 2;
}

static int64_t uart_frame_ns(IngenicUartState *s)
{
    unsigned m = s->umr ? s->umr : 64;
    unsigned h = uart_frame_halfbits(s);
    uint32_t extra = s->uacr & MAKE_64BIT_MASK(0, DIV_ROUND_UP(h, 2));
    uint64_t halfcycles = (uint64_t)m * h + 2 * ctpop32(extra);

    return muldiv64(halfcycles * s->divider, NANOSECONDS_PER_SECOND,
                    2ULL * s->clock_freq);
}

/* Byte landed to timeout: 4 x word length + 12 bits after its start bit */
static int64_t uart_timeout_ns(IngenicUartState *s)
{
    unsigned h = uart_frame_halfbits(s);
    unsigned t = 2 * (4 * uart_word_length(s) + 12);

    return muldiv64(uart_frame_ns(s), t - h, h);
}

static bool uart_tdrq(IngenicUartState *s)
{
    if (uart_fifo_mode(s)) {
        return fifo8_num_used(&s->tx_fifo) <= TDRQ_LEVEL;
    }
    return fifo8_is_empty(&s->tx_fifo);
}

static uint8_t uart_lsr(IngenicUartState *s)
{
    uint8_t lsr = s->lsr;

    if (!fifo8_is_empty(&s->rx_fifo)) {
        lsr |= LSR_DR;
    }
    if (uart_tdrq(s)) {
        lsr |= LSR_TDRQ;
    }
    if (fifo8_is_empty(&s->tx_fifo) && !s->tx_busy) {
        lsr |= LSR_TEMT;
    }
    if (uart_fifo_mode(s) && (s->rx_err_pos || (s->lsr & LSR_OE))) {
        lsr |= LSR_FIFOE;
    }
    return lsr;
}

static bool uart_dma_mode(IngenicUartState *s)
{
    return (s->fcr & (FCR_FME | FCR_DME)) == (FCR_FME | FCR_DME);
}

static bool uart_rx_triggered(IngenicUartState *s)
{
    unsigned rx = fifo8_num_used(&s->rx_fifo);

    return rx && (!uart_fifo_mode(s) || rx >= rx_trigger[s->fcr >> 6]);
}

static uint8_t uart_iir(IngenicUartState *s)
{
    uint8_t lsr = uart_lsr(s);
    uint8_t err = LSR_ERRORS | (uart_dma_mode(s) ? LSR_FIFOE : 0);

    if ((s->ier & IER_RLSIE) && (lsr & err)) {
        return IIR_RLSI;
    }
    if ((s->ier & IER_RTOIE) && s->timeout_pending) {
        return IIR_CTI;
    }
    if ((s->ier & IER_RDRIE) && uart_rx_triggered(s)) {
        return IIR_RDI;
    }
    if ((s->ier & IER_TDRIE) && (lsr & LSR_TDRQ)) {
        return IIR_THRI;
    }
    if ((s->ier & IER_MSIE) && (s->msr & MSR_DELTAS)) {
        return IIR_MSI;
    }
    return IIR_NO_INT;
}

static void uart_update_irq(IngenicUartState *s)
{
    bool dma = uart_dma_mode(s);

    qemu_set_irq(s->irq, uart_iir(s) != IIR_NO_INT);
    qemu_set_irq(s->dreq[0], dma && !(s->ier & IER_TDRIE) && uart_tdrq(s));
    qemu_set_irq(s->dreq[1], dma && uart_rx_triggered(s) &&
                 !(uart_lsr(s) & LSR_FIFOE));
}

static void uart_update_params(IngenicUartState *s)
{
    QEMUSerialSetParams ssp = {
        .data_bits = uart_word_length(s),
        .stop_bits = s->lcr & LCR_STOP ? 2 : 1,
        .parity = !(s->lcr & LCR_PARE) ? 'N' : s->lcr & LCR_PARM ? 'E' : 'O',
    };
    int64_t ns = s->divider ? uart_frame_ns(s) : 0;

    if (ns) {
        ssp.speed = NANOSECONDS_PER_SECOND * uart_frame_halfbits(s) / (2 * ns);
        qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
    }
}

/* In loopback the modem inputs follow the outputs: DTR, RTS, OUT1, OUT2 */
static void uart_loop_modem(IngenicUartState *s)
{
    uint8_t in = 0, delta;

    if (s->mcr & MCR_LOOP) {
        in = ((s->mcr & 0x0c) << 4) | ((s->mcr & 0x02) << 3) |
             ((s->mcr & 0x01) << 5);
    }
    delta = (in ^ s->msr) >> 4;
    if ((delta & (MSR_RI >> 4)) && (in & MSR_RI)) {
        delta &= ~MSR_TERI;     /* trailing edge only */
    }
    s->msr = in | ((s->msr | delta) & MSR_DELTAS);
}

static void uart_rx_land(IngenicUartState *s, uint8_t byte, bool brk)
{
    if (!uart_fifo_mode(s) && !fifo8_is_empty(&s->rx_fifo)) {
        fifo8_reset(&s->rx_fifo);
        s->lsr |= LSR_OE;
    }
    if (fifo8_num_used(&s->rx_fifo) >= RX_FIFO_SIZE) {
        s->lsr |= LSR_OE;
    } else {
        fifo8_push(&s->rx_fifo, byte);
        if (brk) {
            s->rx_err_pos = fifo8_num_used(&s->rx_fifo);
            s->lsr |= LSR_BI;
        }
    }
    s->timeout_pending = false;
    if (uart_fifo_mode(s)) {
        timer_mod(s->timeout_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + uart_timeout_ns(s));
    }
}

static gboolean uart_watch_cb(void *do_not_use, GIOCondition cond,
                              void *opaque);

/* The stop bit has gone: pass the byte on, or hold it for a full backend */
static void uart_tx_shift_out(IngenicUartState *s)
{
    if (s->mcr & MCR_LOOP) {
        if (uart_rx_samples(s)) {
            uart_rx_land(s, s->tsr, false);
        }
    } else {
        int rc = qemu_chr_fe_write(&s->chr, &s->tsr, 1);

        if (rc == 0 || (rc < 0 && errno == EAGAIN)) {
            s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                                 uart_watch_cb, s);
            if (s->watch_tag) {
                return;
            }
        }
    }
    s->tx_busy = false;
}

static gboolean uart_watch_cb(void *do_not_use, GIOCondition cond,
                              void *opaque)
{
    IngenicUartState *s = opaque;

    s->watch_tag = 0;
    uart_tx_shift_out(s);
    uart_tx_kick(s);
    return G_SOURCE_REMOVE;
}

static void uart_tx_kick(IngenicUartState *s)
{
    while (!s->tx_busy && uart_clock_runs(s) && !fifo8_is_empty(&s->tx_fifo)) {
        s->tsr = fifo8_pop(&s->tx_fifo);
        s->tx_busy = true;
        if (s->line_rate) {
            timer_mod(s->tx_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + uart_frame_ns(s));
            break;
        }
        uart_tx_shift_out(s);
    }
    uart_update_irq(s);
}

static void uart_tx_timer_cb(void *opaque)
{
    IngenicUartState *s = opaque;

    uart_tx_shift_out(s);
    uart_tx_kick(s);
}

static void uart_rx_timer_cb(void *opaque)
{
    IngenicUartState *s = opaque;

    s->rx_busy = false;
    uart_rx_land(s, s->rsr, false);
    uart_update_irq(s);
    qemu_chr_fe_accept_input(&s->chr);
}

static void uart_timeout_cb(void *opaque)
{
    IngenicUartState *s = opaque;

    if (!fifo8_is_empty(&s->rx_fifo)) {
        s->timeout_pending = true;
        uart_update_irq(s);
    }
}

static int uart_can_receive(void *opaque)
{
    IngenicUartState *s = opaque;
    unsigned room = (uart_fifo_mode(s) ? RX_FIFO_SIZE : 1) -
                    MIN(fifo8_num_used(&s->rx_fifo), RX_FIFO_SIZE);

    /* A full FIFO holds the backend off rather than overrunning */
    if (!uart_rx_listens(s) || s->rx_busy) {
        return 0;
    }
    return s->line_rate ? !!room : room;
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    IngenicUartState *s = opaque;

    if (s->line_rate) {
        s->rsr = buf[0];
        s->rx_busy = true;
        timer_mod(s->rx_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + uart_frame_ns(s));
        return;
    }
    for (int i = 0; i < size; i++) {
        uart_rx_land(s, buf[i], false);
    }
    uart_update_irq(s);
}

static void uart_event(void *opaque, QEMUChrEvent event)
{
    IngenicUartState *s = opaque;

    if (event == CHR_EVENT_BREAK && uart_rx_listens(s)) {
        uart_rx_land(s, 0, true);
        uart_update_irq(s);
    }
}

static uint8_t uart_read_rbr(IngenicUartState *s)
{
    uint8_t ret = 0;

    if (!fifo8_is_empty(&s->rx_fifo)) {
        ret = fifo8_pop(&s->rx_fifo);
        if (s->rx_err_pos) {
            s->rx_err_pos--;
        }
    }
    s->timeout_pending = false;
    if (fifo8_is_empty(&s->rx_fifo)) {
        s->lsr &= ~LSR_BI;
        timer_del(s->timeout_timer);
    } else if (uart_fifo_mode(s)) {
        timer_mod(s->timeout_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + uart_timeout_ns(s));
    }
    uart_update_irq(s);
    qemu_chr_fe_accept_input(&s->chr);
    return ret;
}

static uint64_t uart_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicUartState *s = opaque;
    uint32_t ret = 0;

    switch (addr) {
    case UART_RBR:
        ret = s->lcr & LCR_DLAB ? s->divider & 0xff : uart_read_rbr(s);
        break;
    case UART_IER:
        ret = s->lcr & LCR_DLAB ? s->divider >> 8 : s->ier;
        break;
    case UART_IIR:
        ret = uart_iir(s) | (uart_fifo_mode(s) ? IIR_FIFO : 0);
        break;
    case UART_LCR:
        ret = s->lcr;
        break;
    case UART_MCR:
        ret = s->mcr;
        break;
    case UART_LSR:
        ret = uart_lsr(s);
        if (s->lsr) {
            s->lsr = 0;
            uart_update_irq(s);
        }
        break;
    case UART_MSR:
        ret = s->msr;
        if (s->msr & MSR_DELTAS) {
            s->msr &= ~MSR_DELTAS;
            uart_update_irq(s);
        }
        break;
    case UART_SPR:
        ret = s->spr;
        break;
    case UART_ISR:
        ret = s->isr;
        break;
    case UART_UMR:
        ret = s->umr;
        break;
    case UART_UACR:
        ret = s->uacr;
        break;
    case UART_RCR:
        ret = fifo8_num_used(&s->rx_fifo);
        break;
    case UART_TCR:
        ret = uart_fifo_mode(s) ? fifo8_num_used(&s->tx_fifo) : 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of unknown offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
    trace_ingenic_uart_read(addr, ret);
    return ret;
}

static void uart_write_thr(IngenicUartState *s, uint8_t val)
{
    if (!uart_fifo_mode(s)) {
        fifo8_reset(&s->tx_fifo);
    } else if (fifo8_is_full(&s->tx_fifo)) {
        /* The ring's count wraps to zero: what was queued is lost */
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-uart: TX FIFO overfilled\n");
        fifo8_reset(&s->tx_fifo);
        uart_update_irq(s);
        return;
    }
    fifo8_push(&s->tx_fifo, val);
    uart_tx_kick(s);
}

static void uart_write_fcr(IngenicUartState *s, uint8_t val)
{
    if ((val ^ s->fcr) & FCR_FME) {
        val |= FCR_RFRT | FCR_TFRT;
    }
    if (val & FCR_RFRT) {
        fifo8_reset(&s->rx_fifo);
        s->rx_err_pos = 0;
        s->lsr &= ~LSR_BI;
        s->timeout_pending = false;
        timer_del(s->timeout_timer);
    }
    if (val & FCR_TFRT) {
        fifo8_reset(&s->tx_fifo);
    }
    s->fcr = val & (FCR_FME | FCR_DME | FCR_UME | FCR_RDTR);
    uart_tx_kick(s);
    qemu_chr_fe_accept_input(&s->chr);
}

static void uart_write(void *opaque, hwaddr addr, uint64_t val64,
                       unsigned size)
{
    IngenicUartState *s = opaque;
    uint32_t val = val64;

    trace_ingenic_uart_write(addr, val);
    switch (addr) {
    case UART_RBR:
        if (s->lcr & LCR_DLAB) {
            s->divider = deposit32(s->divider, 0, 8, val);
            uart_update_params(s);
            uart_tx_kick(s);
            qemu_chr_fe_accept_input(&s->chr);
        } else {
            uart_write_thr(s, val);
        }
        break;
    case UART_IER:
        if (s->lcr & LCR_DLAB) {
            s->divider = deposit32(s->divider, 8, 8, val);
            uart_update_params(s);
            uart_tx_kick(s);
            qemu_chr_fe_accept_input(&s->chr);
        } else {
            s->ier = val & 0x1f;
            uart_update_irq(s);
        }
        break;
    case UART_IIR:
        uart_write_fcr(s, val);
        break;
    case UART_LCR:
        if ((val ^ s->lcr) & LCR_SBK) {
            int brk = !!(val & LCR_SBK);

            qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_BREAK, &brk);
        }
        s->lcr = val;
        uart_update_params(s);
        break;
    case UART_MCR:
        s->mcr = val & MCR_MASK;
        uart_loop_modem(s);
        uart_update_irq(s);
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case UART_SPR:
        s->spr = val;
        break;
    case UART_ISR:
        s->isr = val & 0x1f;
        break;
    case UART_UMR:
        s->umr = val & 0x3f;
        uart_update_params(s);
        qemu_chr_fe_accept_input(&s->chr);
        break;
    case UART_UACR:
        s->uacr = val & 0xfff;
        uart_update_params(s);
        break;
    case UART_LSR:
    case UART_MSR:
    case UART_RCR:
    case UART_TCR:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of unknown offset 0x%"
                      HWADDR_PRIx "\n", __func__, addr);
        break;
    }
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static void uart_reset_hold(Object *obj, ResetType type)
{
    IngenicUartState *s = INGENIC_UART(obj);

    s->divider = 0;
    s->ier = 0;
    s->fcr = 0;
    s->lcr = 0;
    s->mcr = 0;
    s->lsr = 0;
    s->msr = 0;
    s->spr = 0;
    s->isr = 0;
    s->umr = 0x10;
    s->uacr = 0;
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    s->tx_busy = false;
    s->rx_busy = false;
    s->timeout_pending = false;
    s->rx_err_pos = 0;
    timer_del(s->tx_timer);
    timer_del(s->rx_timer);
    timer_del(s->timeout_timer);
    g_clear_handle_id(&s->watch_tag, g_source_remove);
    qemu_set_irq(s->irq, 0);
    qemu_set_irq(s->dreq[0], 0);
    qemu_set_irq(s->dreq[1], 0);
}

static int uart_post_load(void *opaque, int version_id)
{
    IngenicUartState *s = opaque;

    /* A byte held for a full backend: try it again */
    if (s->tx_busy && !timer_pending(s->tx_timer)) {
        timer_mod(s->tx_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_uart = {
    .name = TYPE_INGENIC_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = uart_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(divider, IngenicUartState),
        VMSTATE_UINT8(ier, IngenicUartState),
        VMSTATE_UINT8(fcr, IngenicUartState),
        VMSTATE_UINT8(lcr, IngenicUartState),
        VMSTATE_UINT8(mcr, IngenicUartState),
        VMSTATE_UINT8(lsr, IngenicUartState),
        VMSTATE_UINT8(msr, IngenicUartState),
        VMSTATE_UINT8(spr, IngenicUartState),
        VMSTATE_UINT8(isr, IngenicUartState),
        VMSTATE_UINT8(umr, IngenicUartState),
        VMSTATE_UINT16(uacr, IngenicUartState),
        VMSTATE_FIFO8(tx_fifo, IngenicUartState),
        VMSTATE_FIFO8(rx_fifo, IngenicUartState),
        VMSTATE_UINT8(tsr, IngenicUartState),
        VMSTATE_UINT8(rsr, IngenicUartState),
        VMSTATE_BOOL(tx_busy, IngenicUartState),
        VMSTATE_BOOL(rx_busy, IngenicUartState),
        VMSTATE_BOOL(timeout_pending, IngenicUartState),
        VMSTATE_UINT8(rx_err_pos, IngenicUartState),
        VMSTATE_TIMER_PTR(tx_timer, IngenicUartState),
        VMSTATE_TIMER_PTR(rx_timer, IngenicUartState),
        VMSTATE_TIMER_PTR(timeout_timer, IngenicUartState),
        VMSTATE_END_OF_LIST()
    }
};

static void uart_realize(DeviceState *dev, Error **errp)
{
    IngenicUartState *s = INGENIC_UART(dev);

    if (!s->clock_freq) {
        error_setg(errp, "clock-frequency must be set");
        return;
    }
    fifo8_create(&s->tx_fifo, TX_FIFO_SIZE);
    fifo8_create(&s->rx_fifo, RX_FIFO_SIZE);
    s->tx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_tx_timer_cb, s);
    s->rx_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_rx_timer_cb, s);
    s->timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, uart_timeout_cb, s);
    qemu_chr_fe_set_handlers(&s->chr, uart_can_receive, uart_receive,
                             uart_event, NULL, s, NULL, true);
}

static void uart_unrealize(DeviceState *dev)
{
    IngenicUartState *s = INGENIC_UART(dev);

    qemu_chr_fe_deinit(&s->chr, false);
    g_clear_handle_id(&s->watch_tag, g_source_remove);
    timer_free(s->tx_timer);
    timer_free(s->rx_timer);
    timer_free(s->timeout_timer);
    fifo8_destroy(&s->tx_fifo);
    fifo8_destroy(&s->rx_fifo);
}

static void uart_init(Object *obj)
{
    IngenicUartState *s = INGENIC_UART(obj);

    memory_region_init_io(&s->mmio, obj, &uart_ops, s, TYPE_INGENIC_UART,
                          4 * KiB);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->dreq, "dreq", 2);
}

static const Property uart_properties[] = {
    DEFINE_PROP_CHR("chardev", IngenicUartState, chr),
    DEFINE_PROP_UINT32("clock-frequency", IngenicUartState, clock_freq,
                       24000000),
    DEFINE_PROP_BOOL("line-rate", IngenicUartState, line_rate, true),
};

static void uart_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = uart_realize;
    dc->unrealize = uart_unrealize;
    dc->vmsd = &vmstate_ingenic_uart;
    rc->phases.hold = uart_reset_hold;
    device_class_set_props(dc, uart_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo uart_info = {
    .name = TYPE_INGENIC_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicUartState),
    .instance_init = uart_init,
    .class_init = uart_class_init,
};

static void uart_register_types(void)
{
    type_register_static(&uart_info);
}

type_init(uart_register_types)

/* Every part numbers UART n's DMA requests 0x14 - 2n (TX) and 0x15 - 2n (RX) */
void ingenic_uart_create(MemoryRegion *mem, hwaddr base, qemu_irq irq,
                         Chardev *chr, const char *name, DeviceState *pdma,
                         unsigned index)
{
    DeviceState *dev = qdev_new(TYPE_INGENIC_UART);

    qdev_prop_set_chr(dev, "chardev", chr);
    object_property_add_child(qdev_get_machine(), name, OBJECT(dev));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    for (int d = 0; d < 2; d++) {
        qdev_connect_gpio_out_named(dev, "dreq", d,
            qdev_get_gpio_in_named(pdma, "dreq", 0x14 - 2 * index + d));
    }
    memory_region_add_subregion(mem, base,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));
}

/* The UART as U-Boot leaves it: 115200 8N1 from 24 MHz, FIFOs on */
void ingenic_uart_boot_setup(hwaddr base)
{
    static const struct {
        uint8_t off, val;
    } seq[] = {
        { UART_LCR, LCR_DLAB }, { UART_RBR, 13 }, { UART_IER, 0 },
        { UART_LCR, 0x03 },
        { UART_IIR, FCR_UME | FCR_FME | FCR_RFRT | FCR_TFRT },
    };

    for (int i = 0; i < ARRAY_SIZE(seq); i++) {
        address_space_stl_le(&address_space_memory, base + seq[i].off,
                             seq[i].val, MEMTXATTRS_UNSPECIFIED, NULL);
    }
}
