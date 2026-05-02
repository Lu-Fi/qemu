/*
 * Ingenic T31 System on Chip emulation
 *
 * The Ingenic T31 is a MIPS32r2 XBurst1 SoC used in IP cameras and IoT
 * devices. It features DDR2/DDR3 memory, SPI flash, SD/MMC, USB OTG,
 * Ethernet, and hardware video encoding.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/char/serial-mm.h"
#include "hw/misc/unimp.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "hw/mips/ingenic-t31.h"

/* Memory map - physical addresses (KSEG1 = phys | 0xA0000000) */
const hwaddr ingenic_t31_memmap[] = {
    /* APB bus */
    [INGENIC_T31_DEV_CPM]       = 0x10000000,
    [INGENIC_T31_DEV_INTC]      = 0x10001000,
    [INGENIC_T31_DEV_TCU]       = 0x10002000,
    [INGENIC_T31_DEV_RTC]       = 0x10003000,
    [INGENIC_T31_DEV_GPIO]      = 0x10010000,
    [INGENIC_T31_DEV_AIC]       = 0x10020000,
    [INGENIC_T31_DEV_CODEC]     = 0x10021000,
    [INGENIC_T31_DEV_UART0]     = 0x10030000,
    [INGENIC_T31_DEV_UART1]     = 0x10031000,
    [INGENIC_T31_DEV_UART2]     = 0x10032000,
    [INGENIC_T31_DEV_DMIC]      = 0x10034000,
    [INGENIC_T31_DEV_SSI0]      = 0x10043000,
    [INGENIC_T31_DEV_SSI1]      = 0x10044000,
    [INGENIC_T31_DEV_I2C0]      = 0x10050000,
    [INGENIC_T31_DEV_I2C1]      = 0x10051000,
    [INGENIC_T31_DEV_USBPHY]    = 0x10060000,
    [INGENIC_T31_DEV_DES]       = 0x10061000,
    [INGENIC_T31_DEV_SADC]      = 0x10070000,
    [INGENIC_T31_DEV_DTRNG]     = 0x10072000,

    /* Standalone */
    [INGENIC_T31_DEV_OST]       = 0x12000000,

    /* AHB0 bus */
    [INGENIC_T31_DEV_HARB0]     = 0x13000000,
    [INGENIC_T31_DEV_DDR_PHY]   = 0x13010000,
    [INGENIC_T31_DEV_LCDC]      = 0x13050000,
    [INGENIC_T31_DEV_IPU]       = 0x13080000,
    [INGENIC_T31_DEV_DDRC]      = 0x134f0000,

    /* AHB1 bus (video) */
    [INGENIC_T31_DEV_SCH]       = 0x13200000,
    [INGENIC_T31_DEV_VDMA]      = 0x13210000,
    [INGENIC_T31_DEV_EFE]       = 0x13240000,
    [INGENIC_T31_DEV_MCE]       = 0x13250000,
    [INGENIC_T31_DEV_DBLK]      = 0x13270000,
    [INGENIC_T31_DEV_VMAU]      = 0x13280000,
    [INGENIC_T31_DEV_SDE]       = 0x13290000,
    [INGENIC_T31_DEV_AUX]       = 0x132a0000,
    [INGENIC_T31_DEV_TCSM]      = 0x132c0000,
    [INGENIC_T31_DEV_JPGC]      = 0x132e0000,
    [INGENIC_T31_DEV_SRAM]      = 0x132f0000,

    /* AHB2 bus */
    [INGENIC_T31_DEV_HARB2]     = 0x13400000,
    [INGENIC_T31_DEV_NEMC]      = 0x13410000,
    [INGENIC_T31_DEV_PDMA]      = 0x13420000,
    [INGENIC_T31_DEV_AES]       = 0x13430000,
    [INGENIC_T31_DEV_SFC]       = 0x13440000,
    [INGENIC_T31_DEV_MSC0]      = 0x13450000,
    [INGENIC_T31_DEV_MSC1]      = 0x13460000,
    [INGENIC_T31_DEV_HASH]      = 0x13480000,
    [INGENIC_T31_DEV_GMAC]      = 0x134b0000,
    [INGENIC_T31_DEV_OTG]       = 0x13500000,
    [INGENIC_T31_DEV_EFUSE]     = 0x13540000,

    /* Memory */
    [INGENIC_T31_DEV_SDRAM]     = 0x00000000,
};

/* Unimplemented devices - stubbed with LOG_UNIMP on access */
static const struct {
    const char *name;
    hwaddr base;
    hwaddr size;
} ingenic_t31_unimp[] = {
    { "ingenic-t31-cpm",    0x10000000, 4 * KiB },
    { "ingenic-t31-intc",   0x10001000, 4 * KiB },
    { "ingenic-t31-tcu",    0x10002000, 4 * KiB },
    { "ingenic-t31-rtc",    0x10003000, 4 * KiB },
    { "ingenic-t31-gpio",   0x10010000, 16 * KiB },
    { "ingenic-t31-aic",    0x10020000, 4 * KiB },
    { "ingenic-t31-codec",  0x10021000, 4 * KiB },
    { "ingenic-t31-dmic",   0x10034000, 4 * KiB },
    { "ingenic-t31-ssi0",   0x10043000, 4 * KiB },
    { "ingenic-t31-ssi1",   0x10044000, 4 * KiB },
    { "ingenic-t31-i2c0",   0x10050000, 4 * KiB },
    { "ingenic-t31-i2c1",   0x10051000, 4 * KiB },
    { "ingenic-t31-usbphy", 0x10060000, 4 * KiB },
    { "ingenic-t31-des",    0x10061000, 4 * KiB },
    { "ingenic-t31-sadc",   0x10070000, 4 * KiB },
    { "ingenic-t31-dtrng",  0x10072000, 4 * KiB },
    /* OST is a real device model, not stubbed */
    { "ingenic-t31-harb0",  0x13000000, 4 * KiB },
    { "ingenic-t31-ddrphy", 0x13010000, 4 * KiB },
    { "ingenic-t31-lcdc",   0x13050000, 4 * KiB },
    { "ingenic-t31-ipu",    0x13080000, 4 * KiB },
    { "ingenic-t31-ddrc",   0x134f0000, 4 * KiB },
    { "ingenic-t31-sch",    0x13200000, 4 * KiB },
    { "ingenic-t31-vdma",   0x13210000, 4 * KiB },
    { "ingenic-t31-efe",    0x13240000, 4 * KiB },
    { "ingenic-t31-mce",    0x13250000, 4 * KiB },
    { "ingenic-t31-dblk",   0x13270000, 4 * KiB },
    { "ingenic-t31-vmau",   0x13280000, 4 * KiB },
    { "ingenic-t31-sde",    0x13290000, 4 * KiB },
    { "ingenic-t31-aux",    0x132a0000, 4 * KiB },
    { "ingenic-t31-jpgc",   0x132e0000, 4 * KiB },
    { "ingenic-t31-harb2",  0x13400000, 4 * KiB },
    { "ingenic-t31-nemc",   0x13410000, 4 * KiB },
    { "ingenic-t31-pdma",   0x13420000, 4 * KiB },
    { "ingenic-t31-aes",    0x13430000, 4 * KiB },
    { "ingenic-t31-sfc",    0x13440000, 8 * KiB },
    { "ingenic-t31-msc0",   0x13450000, 4 * KiB },
    { "ingenic-t31-msc1",   0x13460000, 4 * KiB },
    { "ingenic-t31-hash",   0x13480000, 4 * KiB },
    { "ingenic-t31-gmac",   0x134b0000, 4 * KiB },
    { "ingenic-t31-otg",    0x13500000, 68 * KiB },
    { "ingenic-t31-efuse",  0x13540000, 4 * KiB },
};

static void ingenic_t31_init(Object *obj)
{
    IngenicT31State *s = INGENIC_T31(obj);

    s->memmap = ingenic_t31_memmap;

    object_initialize_child(obj, "ost", &s->ost, TYPE_INGENIC_T31_OST);
}

static void ingenic_t31_realize(DeviceState *dev, Error **errp)
{
    IngenicT31State *s = INGENIC_T31(dev);
    unsigned i;

    /* OS Timer (mapped within TCU address space at offset 0x00) */
    sysbus_realize(SYS_BUS_DEVICE(&s->ost), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ost), 0,
                    s->memmap[INGENIC_T31_DEV_TCU]);

    /* TCSM - tightly coupled scratchpad memory (SPL runs from here) */
    memory_region_init_ram(&s->tcsm, OBJECT(dev), "ingenic-t31.tcsm",
                           INGENIC_T31_TCSM_SIZE, &error_abort);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_TCSM], &s->tcsm);

    /* SRAM */
    memory_region_init_ram(&s->sram, OBJECT(dev), "ingenic-t31.sram",
                           INGENIC_T31_SRAM_SIZE, &error_abort);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_SRAM], &s->sram);

    /* UARTs - 16550 compatible, register shift 2 (4-byte aligned) */
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART0], 2,
                   NULL, 115200, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART1], 2,
                   NULL, 115200, serial_hd(1), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART2], 2,
                   NULL, 115200, serial_hd(2), DEVICE_LITTLE_ENDIAN);

    /* Unimplemented device stubs */
    for (i = 0; i < ARRAY_SIZE(ingenic_t31_unimp); i++) {
        create_unimplemented_device(ingenic_t31_unimp[i].name,
                                    ingenic_t31_unimp[i].base,
                                    ingenic_t31_unimp[i].size);
    }
}

static void ingenic_t31_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ingenic_t31_realize;
    dc->user_creatable = false;
}

static const TypeInfo ingenic_t31_type_info = {
    .name = TYPE_INGENIC_T31,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(IngenicT31State),
    .instance_init = ingenic_t31_init,
    .class_init = ingenic_t31_class_init,
};

static void ingenic_t31_register_types(void)
{
    type_register_static(&ingenic_t31_type_info);
}

type_init(ingenic_t31_register_types)
