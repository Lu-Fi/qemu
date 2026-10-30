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
#include "hw/adc/ingenic-sadc.h"
#include "hw/intc/ingenic-intc.h"
#include "hw/i2c/ingenic-i2c.h"
#include "hw/ssi/ingenic-ssi.h"
#include "hw/ssi/ingenic-ssi-slv.h"
#include "hw/sd/ingenic-msc.h"
#include "hw/sd/sdhci.h"
#include "hw/usb/hcd-dwc2.h"
#include "hw/dma/ingenic-pdma.h"
#include "hw/misc/ingenic-rtc.h"
#include "hw/misc/ingenic-dtrng.h"
#include "hw/misc/ingenic-riscv.h"
#include "hw/core/split-irq.h"
#include "hw/misc/ingenic-pwm.h"
#include "hw/misc/ingenic-hash.h"
#include "hw/misc/ingenic-rsa.h"
#include "hw/misc/ingenic-lzma.h"
#include "hw/misc/ingenic-aes.h"
#include "hw/misc/ingenic-des.h"
#include "hw/audio/ingenic-aic.h"
#include "hw/audio/ingenic-dmic.h"
#include "hw/audio/ingenic-as-dmic.h"
#include "hw/misc/ingenic-isp.h"
#include "hw/misc/ingenic-mipi-csi.h"
#include "hw/misc/ingenic-t20-isp.h"
#include "hw/misc/ingenic-avpu.h"
#include "hw/misc/ingenic-helix.h"
#include "hw/misc/ingenic-radix.h"
#include "hw/misc/ingenic-hera.h"
#include "hw/misc/ingenic-jpeg.h"
#include "hw/misc/ingenic-ivdc.h"
#include "hw/misc/ingenic-ipu.h"

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
    INGENIC_T31_DEV_I2C2,
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
    INGENIC_T31_DEV_ISP,

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
    IngenicT31SfcState sfc1;    /* T32, T33 */
    IngenicGmacState gmac;
    IngenicTcuState tcu;
    IngenicSysOstState sysost;
    IngenicGpioState gpio;
    IngenicSadcState sadc;
    IngenicIntcState intc;
    IngenicI2cState i2c[3];
    IngenicSsiState ssi[2];     /* SSI1: T21, T30, T31 */
    IngenicSsiSlvState ssi_slv; /* T31 */
    IngenicMscState msc[2];
    SDHCIState sdhci[2];
    DWC2State dwc2;
    IngenicPdmaState pdma;
    IngenicRtcState rtc;
    IngenicDtrngState dtrng;
    IngenicRiscvState riscv;    /* T23, T31, T32, T33 */
    SplitIRQ isp_irq[2];        /* T32/T33: to the INTC and the core */
    IngenicPwmState pwm;
    IngenicHashState hash;
    IngenicRsaState rsa;
    IngenicLzmaState lzma[2];   /* T32/T33 two, T23 one */
    IngenicAesState aes;
    IngenicDesState des;        /* not the T33 */
    IngenicAicState aic;
    IngenicDmicState dmic;      /* T30, T31 */
    IngenicAsDmicState as_dmic; /* T32, T33 */
    IngenicIspState isp;        /* T21, T23, T31, T32, T33 */
    IngenicMipiCsiState csi;    /* T23, T31 */
    IngenicT20IspState t20_isp; /* T10, T20, T30 */
    IngenicIspState mscaler;    /* T30 */
    IngenicAvpuState avpu;      /* T31 */
    IngenicHelixState helix;    /* T10, T20, T21, T23, T30 */
    IngenicRadixState radix;    /* T30 */
    IngenicHeraState hera;      /* T32, T33 */
    IngenicJpegState jpeg;      /* T32, T33 */
    IngenicIvdcState ivdc;      /* T23, T32, T33 */
    IngenicIpuState ipu;

    MemoryRegion tcsm;
    MemoryRegion sram;
    MemoryRegion bootrom;
    MemoryRegion harb0;
    MemoryRegion efuse;
    MemoryRegion tnpu_id;       /* T32Pro */
    MemoryRegion tnpu_sram;     /* T32Pro */

    char *soc_variant;
    char *efuse_keyhash_hex;
    uint32_t efuse_subsoctype1;
    uint32_t efuse_t33_variant;
    uint32_t efuse_security;
    uint32_t efuse_keyhash[8];
    uint32_t efuse_ctrl;   /* reg 0x000, its bits that read back */
    uint32_t efuse_state;  /* reg 0x008 bit 0: a read is done */
    uint32_t efuse_data;   /* reg 0x00c, from the last read */
    uint32_t efuse_word13;
    int32_t mipi_switch[2];     /* the GPIOs of a MIPI switch, or -1 */
    uint32_t harb0_cpuid;
    const void *variant;
};

extern const hwaddr ingenic_t31_memmap[];

/*
 * Set what differs between the variants' CPUs - PRId, cache sizes and
 * the MXU2 unit - on @cpu, before it is realized.
 */
void ingenic_t31_cpu_props(IngenicT31State *s, DeviceState *cpu);

/* Warn when @ram_size is less than the variant's DDR, before realize */
void ingenic_t31_check_ram(IngenicT31State *s, uint64_t ram_size);

/*
 * The CPU clock, which the CPM derives from CPCCR and the PLLs, set to the
 * variant's rate; for the CPU's "clk-in", before it is realized.
 */
Clock *ingenic_t31_cpu_clock(IngenicT31State *s);

#endif /* HW_MIPS_INGENIC_T31_H */
