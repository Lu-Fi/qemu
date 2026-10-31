/*
 * Ingenic UART
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_INGENIC_UART_H
#define HW_CHAR_INGENIC_UART_H

#include "exec/hwaddr.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_UART "ingenic-uart"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicUartState, INGENIC_UART)

struct IngenicUartState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    qemu_irq irq;
    qemu_irq dreq[2];       /* TX, RX DMA requests */
    CharFrontend chr;

    uint16_t divider;
    uint8_t ier;
    uint8_t fcr;
    uint8_t lcr;
    uint8_t mcr;
    uint8_t lsr;            /* the sticky error bits only */
    uint8_t msr;
    uint8_t spr;
    uint8_t isr;
    uint8_t umr;
    uint16_t uacr;

    Fifo8 tx_fifo;
    Fifo8 rx_fifo;
    uint8_t tsr;            /* the byte on the TX line */
    uint8_t rsr;            /* the byte coming in on the RX line */
    bool tx_busy;
    bool rx_busy;
    bool timeout_pending;
    uint8_t rx_err_pos;     /* bytes to read until no break byte is left */
    QEMUTimer *tx_timer;
    QEMUTimer *rx_timer;
    QEMUTimer *timeout_timer;
    guint watch_tag;

    uint32_t clock_freq;
    bool line_rate;
};

void ingenic_uart_create(MemoryRegion *mem, hwaddr base, qemu_irq irq,
                         Chardev *chr, const char *name, DeviceState *pdma,
                         unsigned index);
void ingenic_uart_boot_setup(hwaddr base);

#endif
