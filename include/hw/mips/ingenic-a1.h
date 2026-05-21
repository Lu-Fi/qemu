/*
 * Ingenic A1 System on Chip emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_A1_H
#define HW_MIPS_INGENIC_A1_H

#include "qom/object.h"
#include "hw/mips/mips.h"
#include "target/mips/cpu.h"
#include "hw/misc/ingenic-a1-cpm.h"
#include "hw/misc/ingenic-t31-ddrc.h"
#include "hw/misc/ingenic-a1-sfc.h"
#include "hw/net/ingenic-t31-gmac.h"
#include "hw/misc/ingenic-t31-ost.h"
#include "hw/timer/ingenic-t31-sysost.h"
#include "hw/gpio/ingenic-t31-gpio.h"
#include "hw/intc/ingenic-t31-intc.h"
#include "hw/i2c/ingenic-t31-i2c.h"
#include "hw/sd/ingenic-t31-msc.h"
#include "hw/sd/sdhci.h"
#include "hw/usb/hcd-dwc2.h"
#include "hw/dma/ingenic-t31-pdma.h"
#include "hw/intc/ingenic-xburst2-ccu.h"
#include "net/net.h"

enum {
    /* APB bus */
    INGENIC_A1_DEV_CPM,
    INGENIC_A1_DEV_TCU,
    INGENIC_A1_DEV_RTC,
    INGENIC_A1_DEV_GPIO,
    INGENIC_A1_DEV_AIC0,
    INGENIC_A1_DEV_AIC1,
    INGENIC_A1_DEV_CODEC,
    INGENIC_A1_DEV_UART0,
    INGENIC_A1_DEV_UART1,
    INGENIC_A1_DEV_UART2,
    INGENIC_A1_DEV_SSI0,
    INGENIC_A1_DEV_SSI1,
    INGENIC_A1_DEV_I2C0,
    INGENIC_A1_DEV_I2C1,
    INGENIC_A1_DEV_USBPHY,
    INGENIC_A1_DEV_DES,
    INGENIC_A1_DEV_DTRNG,
    INGENIC_A1_DEV_HDMI_PHY,
    INGENIC_A1_DEV_VDAC,
    INGENIC_A1_DEV_SATA_PHY0,
    INGENIC_A1_DEV_SATA_PHY1,

    /* Core region */
    INGENIC_A1_DEV_G_OST,
    INGENIC_A1_DEV_N_OST,
    INGENIC_A1_DEV_CCU,
    INGENIC_A1_DEV_INTCN,
    INGENIC_A1_DEV_SRAM,

    /* AHB0 bus */
    INGENIC_A1_DEV_HARB0,
    INGENIC_A1_DEV_DDR_PHY,
    INGENIC_A1_DEV_DDRC,
    INGENIC_A1_DEV_MSC0,
    INGENIC_A1_DEV_MSC1,
    INGENIC_A1_DEV_IPU,
    INGENIC_A1_DEV_AIP,
    INGENIC_A1_DEV_MONITOR,
    INGENIC_A1_DEV_GMAC0,
    INGENIC_A1_DEV_GMAC1,
    INGENIC_A1_DEV_SATA,
    INGENIC_A1_DEV_VO,
    INGENIC_A1_DEV_VDE,
    INGENIC_A1_DEV_HDMI,

    /* AHB1 bus (video) */
    INGENIC_A1_DEV_VC8000D,
    INGENIC_A1_DEV_JPEG,

    /* AHB2 bus */
    INGENIC_A1_DEV_HARB2,
    INGENIC_A1_DEV_NEMC,
    INGENIC_A1_DEV_PDMA,
    INGENIC_A1_DEV_AES,
    INGENIC_A1_DEV_SFC0,
    INGENIC_A1_DEV_SFC1,
    INGENIC_A1_DEV_PWM,
    INGENIC_A1_DEV_HASH,
    INGENIC_A1_DEV_EFUSE,
    INGENIC_A1_DEV_OTG0,
    INGENIC_A1_DEV_OTG1,
    INGENIC_A1_DEV_OTG2,

    /* Memory */
    INGENIC_A1_DEV_SDRAM,
};

/* Boot SRAM: large enough to hold the SPL (up to ~29 KB) plus its
 * stack; also the size of the low-kseg0 alias window used during boot. */
#define INGENIC_A1_SRAM_SIZE    (128 * 1024)

#define TYPE_INGENIC_A1 "ingenic-a1"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1State, INGENIC_A1)

struct IngenicA1State {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    const hwaddr *memmap;

    IngenicA1CpmState cpm;
    IngenicT31DdrcState ddrc;
    IngenicA1SfcState sfc;
    IngenicT31GmacState gmac;
    IngenicT31OstState ost;
    IngenicT31SysOstState sysost;
    IngenicT31GpioState gpio;
    IngenicT31IntcState intc;
    IngenicXBurst2CcuState ccu;
    IngenicT31I2cState i2c[2];
    IngenicT31MscState msc[2];
    DWC2State dwc2;
    IngenicT31PdmaState pdma;

    MemoryRegion sram;
    MemoryRegion bootrom;
    MemoryRegion harb0;
    MemoryRegion efuse;
    MemoryRegion wdt;
    MemoryRegion gost;
    MemoryRegion cost;
    MemoryRegion xgmac;
    NICState *xgmac_nic;
    NICConf xgmac_nic_conf;

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

extern const hwaddr ingenic_a1_memmap[];

#endif /* HW_MIPS_INGENIC_A1_H */
