/*
 * Ingenic T31 System on Chip emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_T31_H
#define HW_MIPS_INGENIC_T31_H

#include "qom/object.h"
#include "hw/mips/mips.h"
#include "hw/misc/ingenic-t31-cpm.h"
#include "hw/misc/ingenic-ddrc.h"
#include "hw/misc/ingenic-t31-sfc.h"
#include "hw/net/ingenic-gmac.h"
#include "hw/timer/ingenic-tcu.h"
#include "hw/timer/ingenic-sysost.h"
#include "hw/gpio/ingenic-gpio.h"
#include "hw/intc/ingenic-intc.h"
#include "hw/i2c/ingenic-i2c.h"
#include "hw/sd/ingenic-msc.h"
#include "hw/sd/sdhci.h"
#include "hw/usb/hcd-dwc2.h"
#include "hw/dma/ingenic-pdma.h"
#include "hw/misc/ingenic-rtc.h"
#include "hw/misc/ingenic-dtrng.h"
#include "hw/misc/ingenic-pwm.h"
#include "hw/misc/ingenic-hash.h"
#include "hw/misc/ingenic-rsa.h"

/**
 * Ingenic T31 device list
 *
 * This enumeration is used to refer to a particular device in the
 * Ingenic T31 SoC. The physical memory base address for each device
 * can be found in the ingenic_t31_memmap array using the device enum
 * value as index.
 */
enum {
    /* APB bus */
    INGENIC_T31_DEV_CPM,
    INGENIC_T31_DEV_INTC,
    INGENIC_T31_DEV_TCU,
    INGENIC_T31_DEV_RTC,
    INGENIC_T31_DEV_GPIO,
    INGENIC_T31_DEV_AIC,
    INGENIC_T31_DEV_CODEC,
    INGENIC_T31_DEV_UART0,
    INGENIC_T31_DEV_UART1,
    INGENIC_T31_DEV_UART2,
    INGENIC_T31_DEV_DMIC,
    INGENIC_T31_DEV_SSI0,
    INGENIC_T31_DEV_SSI1,
    INGENIC_T31_DEV_I2C0,
    INGENIC_T31_DEV_I2C1,
    INGENIC_T31_DEV_USBPHY,
    INGENIC_T31_DEV_DES,
    INGENIC_T31_DEV_SADC,
    INGENIC_T31_DEV_DTRNG,

    /* Standalone */
    INGENIC_T31_DEV_OST,

    /* AHB0 bus */
    INGENIC_T31_DEV_HARB0,
    INGENIC_T31_DEV_DDR_PHY,
    INGENIC_T31_DEV_LCDC,
    INGENIC_T31_DEV_IPU,
    INGENIC_T31_DEV_DDRC,

    /* AHB1 bus (video) */
    INGENIC_T31_DEV_SCH,
    INGENIC_T31_DEV_VDMA,
    INGENIC_T31_DEV_EFE,
    INGENIC_T31_DEV_MCE,
    INGENIC_T31_DEV_DBLK,
    INGENIC_T31_DEV_VMAU,
    INGENIC_T31_DEV_SDE,
    INGENIC_T31_DEV_AUX,
    INGENIC_T31_DEV_TCSM,
    INGENIC_T31_DEV_JPGC,
    INGENIC_T31_DEV_SRAM,

    /* AHB2 bus */
    INGENIC_T31_DEV_HARB2,
    INGENIC_T31_DEV_NEMC,
    INGENIC_T31_DEV_PDMA,
    INGENIC_T31_DEV_AES,
    INGENIC_T31_DEV_SFC,
    INGENIC_T31_DEV_MSC0,
    INGENIC_T31_DEV_MSC1,
    INGENIC_T31_DEV_HASH,
    INGENIC_T31_DEV_RSA,
    INGENIC_T31_DEV_GMAC,
    INGENIC_T31_DEV_OTG,
    INGENIC_T31_DEV_EFUSE,

    /* Memory */
    INGENIC_T31_DEV_SDRAM,
};

#define INGENIC_T31_TCSM_SIZE       (16 * 1024)
#define INGENIC_T31_SRAM_SIZE       (4 * 1024)

#define TYPE_INGENIC_T31 "ingenic-t31"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31State, INGENIC_T31)

struct IngenicT31State {
    /*< private >*/
    DeviceState parent_obj;
    /*< public >*/

    const hwaddr *memmap;

    IngenicT31CpmState cpm;
    IngenicDdrcState ddrc;
    IngenicT31SfcState sfc;
    IngenicGmacState gmac;
    IngenicTcuState tcu;
    IngenicSysOstState sysost;
    IngenicGpioState gpio;
    IngenicIntcState intc;
    IngenicI2cState i2c[2];
    IngenicMscState msc[2];
    SDHCIState sdhci[2];
    DWC2State dwc2;
    IngenicPdmaState pdma;
    IngenicRtcState rtc;
    IngenicDtrngState dtrng;
    IngenicPwmState pwm;
    IngenicHashState hash;
    IngenicRsaState rsa;

    MemoryRegion tcsm;
    MemoryRegion sram;
    MemoryRegion bootrom;
    MemoryRegion harb0;
    MemoryRegion efuse;

    char *soc_variant;
    char *efuse_keyhash_hex;
    uint32_t efuse_subsoctype1;
    uint32_t efuse_security;
    uint32_t efuse_keyhash[8];
    uint32_t harb0_cpuid;
    const void *variant;
};

extern const hwaddr ingenic_t31_memmap[];

#endif /* HW_MIPS_INGENIC_T31_H */
