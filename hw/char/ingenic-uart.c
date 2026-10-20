/*
 * Ingenic UART
 *
 * A 16550 at a 4-byte register stride, except that the transmit data
 * request interrupt is a level: it stays up while the transmit FIFO is
 * at least half empty, and reading UIIR does not clear it. ULSR.FIFOE
 * stays set until the break bytes have been read out of the receive FIFO.
 * The registers from 0x20 on (UMR, UACR, URCR, ...) are not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/char/ingenic-uart.h"
#include "hw/char/serial-mm.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "hw/misc/unimp.h"

void ingenic_uart_create(MemoryRegion *mem, hwaddr base, qemu_irq irq,
                         Chardev *chr, const char *name)
{
    DeviceState *dev = qdev_new(TYPE_SERIAL_MM);
    g_autofree char *ext = g_strdup_printf("%s-ext", name);

    qdev_prop_set_uint8(dev, "regshift", 2);
    qdev_prop_set_uint32(dev, "baudbase", 115200);
    qdev_prop_set_chr(dev, "chardev", chr);
    qdev_prop_set_uint8(dev, "endianness", DEVICE_LITTLE_ENDIAN);
    qdev_prop_set_bit(dev, "level-tx-irq", true);
    qdev_prop_set_bit(dev, "fifo-error-bit", true);
    qdev_set_legacy_instance_id(dev, base, 2);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    memory_region_add_subregion(mem, base,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));
    create_unimplemented_device(ext, base + 0x20, 4 * KiB - 0x20);
}
