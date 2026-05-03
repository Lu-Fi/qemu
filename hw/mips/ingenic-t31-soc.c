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
#include "system/watchdog.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "net/net.h"
#include "hw/mips/ingenic-t31.h"

/*
 * WDT (Watchdog Timer) - writing TCER enable bit triggers system reset
 * via QEMU's watchdog subsystem (default action: reset).
 * Separate from OST to avoid memory region re-entrancy during reset.
 */
#define WDT_TDR_OFF     0x00
#define WDT_TCER_OFF    0x04
#define WDT_TCSR_OFF    0x0C
#define WDT_TCER_EN     (1 << 0)

static QEMUTimer *wdt_timer;
static bool wdt_configured;

static void ingenic_t31_wdt_fire(void *opaque)
{
    wdt_configured = false;
    watchdog_perform_action();
}

static uint64_t ingenic_t31_wdt_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    return 0;
}

static void ingenic_t31_wdt_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    switch (offset) {
    case WDT_TCSR_OFF:
        wdt_configured = true;
        break;
    case WDT_TCER_OFF:
        if ((value & WDT_TCER_EN) && wdt_configured) {
            timer_mod(wdt_timer,
                      qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
        }
        break;
    }
}

static const MemoryRegionOps ingenic_t31_wdt_ops = {
    .read = ingenic_t31_wdt_read,
    .write = ingenic_t31_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 4 },
    .impl  = { .min_access_size = 2, .max_access_size = 4 },
};

/*
 * HARB0 (AHB bus controller) stub - returns SoC ID at offset 0x2C.
 * The cpu_id field encodes the SoC variant: (soc_id >> 12) & 0xFFFF.
 * T31X = 0x00031000.
 */
#define HARB0_SOC_ID    0x2C
#define T31X_SOC_ID     0x00031000

static uint64_t ingenic_t31_harb0_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    if (offset == HARB0_SOC_ID) {
        return T31X_SOC_ID;
    }
    return 0;
}

static void ingenic_t31_harb0_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ingenic_t31_harb0_ops = {
    .read = ingenic_t31_harb0_read,
    .write = ingenic_t31_harb0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * EFUSE stub - returns SoC variant identification and serial numbers.
 * T31X: subsoctype1 = 0x22220000, subsoctype2 = 0x00000000.
 */
#define EFUSE_SERIAL0       0x200
#define EFUSE_SERIAL1       0x204
#define EFUSE_SERIAL2       0x208
#define EFUSE_SUBREMARK     0x231
#define EFUSE_SUBSOCTYPE1   0x238
#define EFUSE_SERIAL3       0x23C
#define EFUSE_SUBSOCTYPE2   0x250

static uint64_t ingenic_t31_efuse_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    switch (offset) {
    case EFUSE_SERIAL0:
        return 0x51454D55;
    case EFUSE_SERIAL1:
        return 0x00000000;
    case EFUSE_SERIAL2:
        return 0x00000000;
    case EFUSE_SERIAL3:
        return 0x00000000;
    case EFUSE_SUBREMARK:
        return 0x00000000;
    case EFUSE_SUBSOCTYPE1:
        return 0xEE000000;
    case EFUSE_SUBSOCTYPE2:
        return 0x00000000;
    default:
        return 0;
    }
}

static void ingenic_t31_efuse_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ingenic_t31_efuse_ops = {
    .read = ingenic_t31_efuse_read,
    .write = ingenic_t31_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

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
    [INGENIC_T31_DEV_DDR_PHY]   = 0x13011000,
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
    /* CPM is a real device model, not stubbed */
    /* INTC is a real device model, not stubbed */
    { "ingenic-t31-tcu",    0x10002000, 4 * KiB },
    { "ingenic-t31-rtc",    0x10003000, 4 * KiB },
    /* GPIO is a real device model, not stubbed */
    { "ingenic-t31-aic",    0x10020000, 4 * KiB },
    { "ingenic-t31-codec",  0x10021000, 4 * KiB },
    { "ingenic-t31-dmic",   0x10034000, 4 * KiB },
    { "ingenic-t31-ssi0",   0x10043000, 4 * KiB },
    { "ingenic-t31-ssi1",   0x10044000, 4 * KiB },
    /* I2C0/I2C1 are real device models, not stubbed */
    { "ingenic-t31-usbphy", 0x10060000, 4 * KiB },
    { "ingenic-t31-des",    0x10061000, 4 * KiB },
    { "ingenic-t31-sadc",   0x10070000, 4 * KiB },
    { "ingenic-t31-dtrng",  0x10072000, 4 * KiB },
    /* OST is a real device model, not stubbed */
    /* HARB0 uses SoC ID stub */
    /* DDR PHY is a real device model, not stubbed */
    { "ingenic-t31-lcdc",   0x13050000, 4 * KiB },
    { "ingenic-t31-ipu",    0x13080000, 4 * KiB },
    /* DDRC is a real device model, not stubbed */
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
    { "ingenic-t31-pdma",   0x13420000, 64 * KiB },
    { "ingenic-t31-aes",    0x13430000, 4 * KiB },
    /* SFC is a real device model, not stubbed */
    /* MSC0/MSC1 are real device models, not stubbed */
    { "ingenic-t31-hash",   0x13480000, 4 * KiB },
    /* GMAC is a real device model, not stubbed */
    { "ingenic-t31-otg",    0x13500000, 68 * KiB },
    /* EFUSE uses SoC variant stub */
};

static void ingenic_t31_init(Object *obj)
{
    IngenicT31State *s = INGENIC_T31(obj);

    s->memmap = ingenic_t31_memmap;

    object_initialize_child(obj, "cpm", &s->cpm, TYPE_INGENIC_T31_CPM);
    object_initialize_child(obj, "ddrc", &s->ddrc, TYPE_INGENIC_T31_DDRC);
    object_initialize_child(obj, "sfc", &s->sfc, TYPE_INGENIC_T31_SFC);
    object_initialize_child(obj, "gmac", &s->gmac, TYPE_INGENIC_T31_GMAC);
    object_initialize_child(obj, "ost", &s->ost, TYPE_INGENIC_T31_OST);
    object_initialize_child(obj, "sysost", &s->sysost, TYPE_INGENIC_T31_SYSOST);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_INGENIC_T31_GPIO);
    object_initialize_child(obj, "intc", &s->intc, TYPE_INGENIC_T31_INTC);
    object_initialize_child(obj, "i2c0", &s->i2c[0], TYPE_INGENIC_T31_I2C);
    object_initialize_child(obj, "i2c1", &s->i2c[1], TYPE_INGENIC_T31_I2C);
    object_initialize_child(obj, "msc0", &s->msc[0], TYPE_INGENIC_T31_MSC);
    object_initialize_child(obj, "msc1", &s->msc[1], TYPE_INGENIC_T31_MSC);
}

static void ingenic_t31_realize(DeviceState *dev, Error **errp)
{
    IngenicT31State *s = INGENIC_T31(dev);
    unsigned i;

    /* CPM */
    sysbus_realize(SYS_BUS_DEVICE(&s->cpm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->cpm), 0,
                    s->memmap[INGENIC_T31_DEV_CPM]);

    /* DDR Controller at 0x134F0000, PHY at 0x13011000 */
    sysbus_realize(SYS_BUS_DEVICE(&s->ddrc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 0,
                    s->memmap[INGENIC_T31_DEV_DDRC]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 1,
                    s->memmap[INGENIC_T31_DEV_DDR_PHY]);

    /* SFC: IRQ -> INTC source 7 (IRQ_SFC). */
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc), 0,
                    s->memmap[INGENIC_T31_DEV_SFC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 7));

    /* GMAC */
    {
        bool matched = qemu_configure_nic_device(DEVICE(&s->gmac), true,
                                                  NULL);
        if (!matched) {
            matched = qemu_configure_nic_device(DEVICE(&s->gmac), false,
                                                 "ingenic-t31-gmac");
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gmac), 0,
                    s->memmap[INGENIC_T31_DEV_GMAC]);

    /* WDT at TCU base, overlapping OST with higher priority */
    memory_region_init_io(&s->wdt, OBJECT(dev), &ingenic_t31_wdt_ops,
                          NULL, "ingenic-t31-wdt", 0x10);
    memory_region_add_subregion_overlap(get_system_memory(),
                                        s->memmap[INGENIC_T31_DEV_TCU],
                                        &s->wdt, 1);
    if (!wdt_timer) {
        wdt_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL,
                                 ingenic_t31_wdt_fire, NULL);
    }

    /* OS Timer (mapped within TCU address space) - U-Boot uses this. */
    sysbus_realize(SYS_BUS_DEVICE(&s->ost), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ost), 0,
                    s->memmap[INGENIC_T31_DEV_TCU]);

    /*
     * Standalone System OS Timer at 0x12000000 - BSP 3.10.14 kernel
     * uses this for the tick (T1, IRQ to MIPS IP4) and clocksource
     * (T2 64-bit free counter). IRQ wiring deferred until after the
     * kernel reaches a point where the tick is actually required.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->sysost), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sysost), 0,
                    s->memmap[INGENIC_T31_DEV_OST]);

    /* GPIO controller: 3 ports + shadow page in a 64 KiB region. */
    sysbus_realize(SYS_BUS_DEVICE(&s->gpio), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0,
                    s->memmap[INGENIC_T31_DEV_GPIO]);

    /* INTC: 64 sources -> MIPS IP2. Wired to CPU in board init. */
    sysbus_realize(SYS_BUS_DEVICE(&s->intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0,
                    s->memmap[INGENIC_T31_DEV_INTC]);

    /*
     * I2C0/I2C1: minimal "auto-NACK" controllers so the kernel's
     * bus scan completes. INTC source numbers: I2C0 = 60 (bank 1
     * bit 28), I2C1 = 59 (bank 1 bit 27).
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[0]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[0]), 0,
                    s->memmap[INGENIC_T31_DEV_I2C0]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[0]), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 60));
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[1]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[1]), 0,
                    s->memmap[INGENIC_T31_DEV_I2C1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[1]), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 59));

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

    /*
     * Boot ROM region at physical 0x1FC00000 (KSEG1 0xBFC00000).
     * Populated with ERET at exception vector offsets so that stray
     * exceptions (e.g. from MMIO to unimplemented devices) return
     * cleanly instead of looping in unmapped memory.
     * A real bootrom image can be loaded via -bios to replace this.
     */
    memory_region_init_rom(&s->bootrom, OBJECT(dev), "ingenic-t31.bootrom",
                           32 * KiB, &error_abort);
    memory_region_add_subregion(get_system_memory(), 0x1fc00000, &s->bootrom);
    {
        /* ERET opcode: 0x42000018 */
        const uint32_t eret = 0x42000018;
        /* Exception vector offsets from BFC00000 */
        static const uint32_t vectors[] = {
            0x000, 0x080, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380
        };
        unsigned v;
        for (v = 0; v < ARRAY_SIZE(vectors); v++) {
            rom_add_blob_fixed("ingenic-t31.eret", &eret, 4,
                               0x1fc00000 + vectors[v]);
        }
    }

    /*
     * UARTs - 16550 compatible with Ingenic SIRCR/UMR extensions.
     * Register shift 2 = 4-byte aligned registers. The 16550 model
     * covers registers 0-7 (offsets 0x00-0x1C). The Ingenic UART has
     * SIRCR at 0x20 and UMR at 0x24, so we add stubs for each UART
     * to absorb writes to the extended registers.
     */
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART0], 2,
                   NULL, 115200, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    create_unimplemented_device("ingenic-t31-uart0-ext",
                                s->memmap[INGENIC_T31_DEV_UART0] + 0x20,
                                4 * KiB - 0x20);
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART1], 2,
                   NULL, 115200, serial_hd(1), DEVICE_LITTLE_ENDIAN);
    create_unimplemented_device("ingenic-t31-uart1-ext",
                                s->memmap[INGENIC_T31_DEV_UART1] + 0x20,
                                4 * KiB - 0x20);
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART2], 2,
                   NULL, 115200, serial_hd(2), DEVICE_LITTLE_ENDIAN);
    create_unimplemented_device("ingenic-t31-uart2-ext",
                                s->memmap[INGENIC_T31_DEV_UART2] + 0x20,
                                4 * KiB - 0x20);

    /* HARB0 - SoC ID at offset 0x2C */
    memory_region_init_io(&s->harb0, OBJECT(dev), &ingenic_t31_harb0_ops,
                          NULL, "ingenic-t31-harb0", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_HARB0], &s->harb0);

    /* EFUSE - SoC variant and serial numbers */
    memory_region_init_io(&s->efuse, OBJECT(dev), &ingenic_t31_efuse_ops,
                          NULL, "ingenic-t31-efuse", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_EFUSE], &s->efuse);

    /* MSC0/MSC1 - SD/MMC controllers */
    sysbus_realize(SYS_BUS_DEVICE(&s->msc[0]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[0]),
                    0, s->memmap[INGENIC_T31_DEV_MSC0]);
    sysbus_realize(SYS_BUS_DEVICE(&s->msc[1]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[1]),
                    0, s->memmap[INGENIC_T31_DEV_MSC1]);

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
