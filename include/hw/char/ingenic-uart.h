/*
 * Ingenic UART
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_CHAR_INGENIC_UART_H
#define HW_CHAR_INGENIC_UART_H

#include "exec/hwaddr.h"
#include "hw/core/irq.h"
#include "chardev/char.h"

void ingenic_uart_create(MemoryRegion *mem, hwaddr base, qemu_irq irq,
                         Chardev *chr, const char *name);

#endif
