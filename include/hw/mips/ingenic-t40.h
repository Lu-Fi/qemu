/*
 * Ingenic T40/T41 System on Chip emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_T40_H
#define HW_MIPS_INGENIC_T40_H

#include "qom/object.h"
#include "hw/mips/mips.h"
#include "target/mips/cpu.h"
#include "hw/misc/ingenic-t40-cpm.h"
#include "hw/misc/ingenic-ddrc.h"
#include "hw/misc/ingenic-a1-sfc.h"
#include "hw/net/ingenic-gmac.h"
#include "hw/gpio/ingenic-gpio.h"
#include "hw/intc/ingenic-intc.h"
#include "hw/i2c/ingenic-i2c.h"
#include "hw/sd/ingenic-msc.h"
#include "hw/sd/sdhci.h"
#include "hw/usb/hcd-dwc2.h"
#include "hw/dma/ingenic-pdma.h"
#include "hw/intc/ingenic-xburst2-ccu.h"
#include "hw/timer/ingenic-tcu.h"
#include "hw/misc/ingenic-dtrng.h"
#include "hw/misc/ingenic-pwm.h"
#include "hw/misc/ingenic-rtc.h"
#include "net/net.h"

enum {
    /* APB bus */
    INGENIC_T40_DEV_CPM,
    INGENIC_T40_DEV_INTC,
    INGENIC_T40_DEV_TCU,
    INGENIC_T40_DEV_RTC,
    INGENIC_T40_DEV_GPIO,
    INGENIC_T40_DEV_UART0,
    INGENIC_T40_DEV_UART1,
    INGENIC_T40_DEV_UART2,
    INGENIC_T40_DEV_UART3,
    INGENIC_T40_DEV_SSI_SLV,
    INGENIC_T40_DEV_SSI0,
    INGENIC_T40_DEV_I2C0,
    INGENIC_T40_DEV_I2C1,
    INGENIC_T40_DEV_I2C2,
    INGENIC_T40_DEV_I2C3,
    INGENIC_T40_DEV_USBPHY,
    INGENIC_T40_DEV_SADC,
    INGENIC_T40_DEV_DMIC,
    INGENIC_T40_DEV_MIPI_PHY,
    INGENIC_T40_DEV_MIPI,

    /* Core region */
    INGENIC_T40_DEV_G_OST,
    INGENIC_T40_DEV_N_OST,
    INGENIC_T40_DEV_CCU,
    INGENIC_T40_DEV_INTCN,
    INGENIC_T40_DEV_SRAM,
    INGENIC_T40_DEV_NNDMA,
    INGENIC_T40_DEV_RISCV,

    /* AHB bus */
    INGENIC_T40_DEV_HARB0,
    INGENIC_T40_DEV_DDR_PHY,
    INGENIC_T40_DEV_DDRC,
    INGENIC_T40_DEV_LCDC,
    INGENIC_T40_DEV_MSC0,
    INGENIC_T40_DEV_MSC1,
    INGENIC_T40_DEV_IPU,
    INGENIC_T40_DEV_BSCALER,
    INGENIC_T40_DEV_MONITOR,
    INGENIC_T40_DEV_I2D,
    INGENIC_T40_DEV_VO,
    INGENIC_T40_DEV_DRAWBOX,
    INGENIC_T40_DEV_RADIX,
    INGENIC_T40_DEV_EL150,
    INGENIC_T40_DEV_ISP,
    INGENIC_T40_DEV_NEMC,
    INGENIC_T40_DEV_PDMA,
    INGENIC_T40_DEV_AES,
    INGENIC_T40_DEV_SFC,
    INGENIC_T40_DEV_GMAC,
    INGENIC_T40_DEV_RSA,
    INGENIC_T40_DEV_HASH,
    INGENIC_T40_DEV_OTG,
    INGENIC_T40_DEV_EFUSE,

    /* Memory */
    INGENIC_T40_DEV_SDRAM,
};

/* Boot SRAM: large enough to hold the SPL (up to ~29 KB) plus its
 * stack; also the size of the low-kseg0 alias window used during boot. */
#define INGENIC_T40_SRAM_SIZE    (128 * 1024)

#define TYPE_INGENIC_T40 "ingenic-t40"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT40State, INGENIC_T40)

struct IngenicT40State {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    const hwaddr *memmap;

    IngenicT40CpmState cpm;
    IngenicDdrcState ddrc;
    IngenicA1SfcState sfc;
    IngenicGmacState gmac;
    IngenicTcuState tcu;
    IngenicDtrngState dtrng;
    IngenicPwmState pwm;
    IngenicRtcState rtc;
    IngenicGpioState gpio;
    IngenicIntcState intc;
    IngenicXBurst2CcuState ccu;
    IngenicI2cState i2c[4];
    IngenicMscState msc[2];
    DWC2State dwc2;
    IngenicPdmaState pdma;

    MemoryRegion sram;
    MemoryRegion cpu1_sram;
    MemoryRegion bootrom;
    MemoryRegion harb0;
    MemoryRegion efuse;
    MemoryRegion gost;
    MemoryRegion cost;

    /* Per-CPU state (dual-core XBurst2) */
    MIPSCPU *cpu[2];
    int num_cpus;
    qemu_irq cost_irq[2];
    QEMUTimer *cost_timer[2];

    char *soc_variant;
    uint32_t efuse_subsoctype2;
    uint32_t harb0_cpuid;
    const void *variant;
};

extern const hwaddr ingenic_t40_memmap[];

#endif /* HW_MIPS_INGENIC_T40_H */
