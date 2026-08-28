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
#include "system/runstate.h"
#include "system/reset.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "net/net.h"
#include "hw/core/boards.h"
#include "hw/mips/ingenic-t31.h"

/*
 * HARB0 (AHB bus controller) stub - returns SoC ID at offset 0x2C.
 * The cpu_id field encodes the SoC variant: (soc_id >> 12) & 0xFFFF.
 * T31X = 0x00031000.
 */
#define HARB0_SOC_ID    0x2C

static uint64_t ingenic_t31_harb0_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    IngenicT31State *s = opaque;

    if (offset == HARB0_SOC_ID) {
        return s->harb0_cpuid;
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
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * EFUSE stub - returns SoC variant identification and serial numbers.
 * T31X: subsoctype1 = 0x22220000, subsoctype2 = 0x00000000.
 */
#define EFUSE_SERIAL0       0x200
#define EFUSE_SERIAL1       0x204
#define EFUSE_SERIAL2       0x208
#define EFUSE_T33_VARIANT   0x21C
#define EFUSE_SUBREMARK     0x231
#define EFUSE_SUBSOCTYPE1   0x238
#define EFUSE_SERIAL3       0x23C
#define EFUSE_SUBSOCTYPE2   0x250

/*
 * Map SoC variant names to EFUSE SUBSOCTYPE1 values (upper 16 bits).
 * libimp's get_cpu_id() and thingino's /usr/sbin/soc both read this
 * register at 0x13540238 to identify the exact chip variant.
 */
/*
 * T31 variant table: maps sub-model name to EFUSE id, PLL register
 * values, and RAM size. PLL MNOD encoding from U-Boot isvp_common.h:
 *   (nf << 20) | (nr << 14) | (od1 << 11) | (od0 << 8)
 * VPLL is always 1200 MHz on T31.
 */
#define PLL_ON_EN  ((1 << 3) | (1 << 0))
#define MNOD(nf, nr, od1, od0)  (((nf) << 20) | ((nr) << 14) | \
                                 ((od1) << 11) | ((od0) << 8) | PLL_ON_EN)

typedef struct {
    const char *name;
    uint32_t cpuid;      /* HARB0+0x2C: bits[27:12]=family, bit28=1 */
    uint32_t type1;      /* EFUSE SUBSOCTYPE1 */
    uint32_t t33var;     /* EFUSE T33 variant word at 0x21C (byte 3 = variant) */
    uint32_t apll;       /* CPAPCR register value */
    uint32_t mpll;       /* CPMPCR register value */
    uint32_t ram_mb;
} T31Variant;

static const T31Variant t31_variants[] = {
    /* T10 family (cpuid bits[27:12] = 0x0005) */
    { "t10",   0x10005000, 0x00000000, 0, MNOD( 71,2,1,1), MNOD(100,1,2,1),  64 },  /* 860/600 */
    { "t10l",  0x10005000, 0x00000000, 0, MNOD( 59,2,1,1), MNOD(100,1,2,1),  64 },  /* 712/600 */
    /* T20 family (cpuid bits[27:12] = 0x2000) */
    { "t20n",  0x12000000, 0x11110000, 0, MNOD( 71,2,1,1), MNOD(125,3,1,1),  64 },  /* 860/500 */
    { "t20x",  0x12000000, 0x22220000, 0, MNOD( 71,2,1,1), MNOD(100,1,2,1), 128 },  /* 860/600 */
    { "t20l",  0x12000000, 0x33330000, 0, MNOD( 59,2,1,1), MNOD(125,3,1,1),  64 },  /* 712/500 */
    /* T21 family (cpuid bits[27:12] = 0x0021, InnoSilicon DDR PHY like T31) */
    { "t21n",  0x10021000, 0x11110000, 0, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    { "t21l",  0x10021000, 0x33330000, 0, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    { "t21z",  0x10021000, 0x55550000, 0, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    /* T30 family (cpuid bits[27:12] = 0x0030, InnoSilicon DDR PHY,
     * PLL od0 at bit 5 not bit 8 - use raw MNOD values from isvp_common.h) */
    { "t30n",  0x10030000, 0x11110000, 0, (( 74<<20)|(1<<14)|(1<<11)|(2<<5))|PLL_ON_EN, MNOD(125,1,3,1),  64 },  /*  900/500 */
    { "t30x",  0x10030000, 0x22220000, 0, (( 74<<20)|(1<<14)|(1<<11)|(2<<5))|PLL_ON_EN, MNOD(100,1,2,1), 128 },  /*  900/600 */
    { "t30l",  0x10030000, 0x33330000, 0, ((124<<20)|(1<<14)|(2<<11)|(2<<5))|PLL_ON_EN, MNOD(125,1,3,1),  64 },  /*  750/500 */
    { "t30a",  0x10030000, 0x44440000, 0, ((149<<20)|(2<<14)|(1<<11)|(1<<5))|PLL_ON_EN, MNOD(100,1,2,1), 128 },  /* 1200/600 */
    { "t30z",  0x10030000, 0x55550000, 0, (( 74<<20)|(1<<14)|(1<<11)|(2<<5))|PLL_ON_EN, MNOD(125,1,3,1),  64 },  /*  900/500 */
    /* T23 family (cpuid bits[27:12] = 0x0023, InnoSilicon DDR PHY like T31) */
    { "t23n",  0x10023000, 0x11110000, 0, MNOD(297,3,2,1), MNOD(100,1,2,1),  64 },  /* 1188/600 */
    { "t23x",  0x10023000, 0x22220000, 0, MNOD(297,3,2,1), MNOD(100,1,2,1),  64 },  /* 1188/600 */
    { "t23dl", 0x10023000, 0x33330000, 0, MNOD(297,3,2,1), MNOD(100,1,2,1),  32 },  /* 1188/600 */
    { "t23zn", 0x10023000, 0x77770000, 0, MNOD(125,1,3,1), MNOD(125,1,3,1),  64 },  /* 1000/500 */
    /* T31 family (cpuid bits[27:12] = 0x0031) */
    { "t31n",  0x10031000, 0x11110000, 0, MNOD(117,1,2,1), MNOD(125,1,3,1),  64 },  /* 1404/500 */
    { "t31x",  0x10031000, 0x22220000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31l",  0x10031000, 0x33330000, 0, MNOD( 84,1,2,1), MNOD(125,1,3,1),  64 },  /* 1008/500 */
    { "t31a",  0x10031000, 0x44440000, 0, MNOD(125,1,2,1), MNOD(125,1,2,1), 128 },  /* 1500/750 */
    { "t31zl", 0x10031000, 0x55550000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31zx", 0x10031000, 0x66660000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31al", 0x10031000, 0xCCCC0000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31zc", 0x10031000, 0xDDDD0000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31lc", 0x10031000, 0xEEEE0000, 0, MNOD( 92,1,2,1), MNOD(125,1,3,1),  64 },  /* 1104/500 */
    /* T32 family (cpuid bits[27:12] = 0x0032, same DDR/GPIO as T31,
     * low nibble = 4 is the revision code expected by vendor SPL) */
    { "t32nq", 0x10032004, 0xAAAA0000, 0, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128 },  /* 864/600 */
    { "t32lq", 0x10032004, 0xBBBB0000, 0, MNOD( 72,1,2,1), MNOD(100,1,2,1),  64 },  /* 864/600 */
    { "t32xq", 0x10032004, 0x22220000, 0, MNOD( 72,1,2,1), MNOD(100,1,2,1), 256 },  /* 864/600 */
    { "t32zn", 0x10032004, 0x55550000, 0, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    /* T33 family (cpuid bits[27:12] = 0x0033, variant byte at efuse 0x21C>>24) */
    { "t33n",      0x10033004, 0x00000000, 0xAA000000, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128 },
    { "qemu-t33",  0x10033004, 0x00000000, 0xEE000000, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128 },
    /* Default */
    { "qemu",  0x10031000, 0xEE000000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { NULL, 0, 0, 0, 0, 0, 0 }
};

static const T31Variant *ingenic_t31_find_variant(const char *name)
{
    if (name && name[0]) {
        for (int i = 0; t31_variants[i].name; i++) {
            if (g_ascii_strcasecmp(name, t31_variants[i].name) == 0) {
                return &t31_variants[i];
            }
        }
    }
    /* default: last entry before NULL sentinel = "qemu" */
    return &t31_variants[ARRAY_SIZE(t31_variants) - 2];
}

static uint64_t ingenic_t31_efuse_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    IngenicT31State *s = opaque;

    switch (offset) {
    case EFUSE_SERIAL0:
        return 0x725C2516;
    case EFUSE_SERIAL1:
        return 0x94FE0281;
    case EFUSE_SERIAL2:
        return 0x12100080;
    case EFUSE_SERIAL3:
        return 0x00000000;
    case 0x210:
        return s->efuse_security ? s->efuse_security : 0x21000000;
    case 0x230:
        return 0x00008888;
    case EFUSE_SUBREMARK:
        return 0x00000000;
    case 0x008:
        return 0x01;
    case EFUSE_T33_VARIANT: /* 0x21C */
        return s->efuse_t33_variant;
    case 0x21F:
        return 0x00000000;
    case EFUSE_SUBSOCTYPE1: /* 0x238 */
        return s->efuse_subsoctype1 ? s->efuse_subsoctype1 : 0x99991111;
    case EFUSE_SUBSOCTYPE2:
        if (s->efuse_keyhash[4]) {
            return s->efuse_keyhash[(offset - 0x240) / 4];
        }
        return 0x00000000;
    default:
        if (offset >= 0x240 && offset < 0x260) {
            unsigned idx = (offset - 0x240) / 4;
            fprintf(stderr, "EFUSE: R 0x%03x keyhash[%u] = 0x%08x\n",
                    (unsigned)offset, idx, s->efuse_keyhash[idx]);
            return s->efuse_keyhash[idx];
        }
        fprintf(stderr, "EFUSE: R 0x%03x = 0 (unhandled)\n", (unsigned)offset);
        return 0;
    }
}

static void ingenic_t31_efuse_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    fprintf(stderr, "EFUSE: W 0x%03x = 0x%08x\n",
            (unsigned)offset, (unsigned)value);
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
    [INGENIC_T31_DEV_RSA]       = 0x134c0000,
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
    /* OST is a real device model, not stubbed */
    /* HARB0 uses SoC ID stub */
    /* DDR PHY is a real device model, not stubbed */
    { "ingenic-t31-i2d",    0x13030000, 4 * KiB },
    /* T30 places its video encoder block here (T31 has nothing at this
     * address); the T30 SDK module probes it at open time and a bus
     * error there oopses the open with misc_mtx held. */
    { "ingenic-t30-vpu",    0x130b0000, 64 * KiB },
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
    { "ingenic-t31-isp",    0x13300000, 1024 * KiB },
    { "ingenic-t31-harb2",  0x13400000, 4 * KiB },
    { "ingenic-t31-nemc",   0x13410000, 4 * KiB },
    /* PDMA is a real device now, not unimplemented */
    { "ingenic-t31-aes",    0x13430000, 4 * KiB },
    /* SFC is a real device model, not stubbed */
    /* MSC0/MSC1 are real device models, not stubbed */
    /* HASH and RSA are real device models */
    /* GMAC is a real device model, not stubbed */
    { "ingenic-t31-otg",    0x13500000, 68 * KiB },
    /* EFUSE uses SoC variant stub */
};

static void ingenic_t31_init(Object *obj)
{
    IngenicT31State *s = INGENIC_T31(obj);

    s->memmap = ingenic_t31_memmap;

    object_initialize_child(obj, "cpm", &s->cpm, TYPE_INGENIC_T31_CPM);
    object_initialize_child(obj, "ddrc", &s->ddrc, TYPE_INGENIC_DDRC);
    object_initialize_child(obj, "sfc", &s->sfc, TYPE_INGENIC_T31_SFC);
    object_initialize_child(obj, "gmac", &s->gmac, TYPE_INGENIC_GMAC);
    object_initialize_child(obj, "tcu", &s->tcu, TYPE_INGENIC_TCU);
    object_initialize_child(obj, "sysost", &s->sysost, TYPE_INGENIC_SYSOST);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_INGENIC_GPIO);
    object_initialize_child(obj, "intc", &s->intc, TYPE_INGENIC_INTC);
    object_initialize_child(obj, "i2c0", &s->i2c[0], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c1", &s->i2c[1], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "msc0", &s->msc[0], TYPE_INGENIC_MSC);
    object_initialize_child(obj, "msc1", &s->msc[1], TYPE_INGENIC_MSC);
    object_initialize_child(obj, "sdhci0", &s->sdhci[0], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "sdhci1", &s->sdhci[1], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "dwc2", &s->dwc2, TYPE_DWC2_USB);
    object_initialize_child(obj, "pdma", &s->pdma, TYPE_INGENIC_PDMA);
    object_initialize_child(obj, "rtc", &s->rtc, TYPE_INGENIC_RTC);
    object_initialize_child(obj, "dtrng", &s->dtrng, TYPE_INGENIC_DTRNG);
    object_initialize_child(obj, "pwm", &s->pwm, TYPE_INGENIC_PWM);
    object_initialize_child(obj, "hash", &s->hash, TYPE_INGENIC_HASH);
    object_initialize_child(obj, "rsa", &s->rsa, TYPE_INGENIC_RSA);
    object_property_add_const_link(OBJECT(&s->dwc2), "dma-mr",
                                   OBJECT(get_system_memory()));
}

static void ingenic_t31_realize(DeviceState *dev, Error **errp)
{
    IngenicT31State *s = INGENIC_T31(dev);
    unsigned i;

    {
        const T31Variant *v = ingenic_t31_find_variant(s->soc_variant);
        s->efuse_subsoctype1 = v->type1;
        s->efuse_t33_variant = v->t33var;
        s->harb0_cpuid = v->cpuid;
        s->variant = v;
    }

    if (s->efuse_keyhash_hex) {
        const char *p = s->efuse_keyhash_hex;
        for (i = 0; i < 8 && *p; i++) {
            unsigned long v = 0;
            for (int j = 0; j < 8 && *p; j++, p++) {
                unsigned d = (*p >= 'a') ? *p - 'a' + 10 :
                             (*p >= 'A') ? *p - 'A' + 10 : *p - '0';
                v = (v << 4) | (d & 0xf);
            }
            s->efuse_keyhash[i] = (uint32_t)v;
        }
    }

    /* CPM */
    sysbus_realize(SYS_BUS_DEVICE(&s->cpm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->cpm), 0,
                    s->memmap[INGENIC_T31_DEV_CPM]);

    /* Override PLL defaults based on SoC variant so that direct
     * -kernel boots (without SPL reprogramming) see correct clocks. */
    {
        const T31Variant *v = s->variant;
        s->cpm.regs[0x10 / 4] = v->apll;   /* CPM_CPAPCR */
        s->cpm.regs[0x14 / 4] = v->mpll;   /* CPM_CPMPCR */
    }

    /*
     * DDR Controller: T31 at 0x134F0000 + PHY at 0x13011000.
     * T10/T20 use DDRC at 0x13020000 + PHY at 0x13010000 instead.
     * Map both sets so the same model serves all SoC families.
     * The DDRC model is a lightweight stub that returns PHY-ready
     * bits, so overlapping coverage is harmless.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->ddrc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 0,
                    s->memmap[INGENIC_T31_DEV_DDRC]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 1,
                    s->memmap[INGENIC_T31_DEV_DDR_PHY]);
    /* T10/T20 DDRC aliases */
    {
        static MemoryRegion ddrc_t20, ddrphy_t20;
        memory_region_init_alias(&ddrc_t20, OBJECT(dev), "ddrc-t20",
                                 &s->ddrc.ddrc_iomem, 0, 4 * KiB);
        memory_region_add_subregion(get_system_memory(), 0x13020000,
                                    &ddrc_t20);
        memory_region_init_alias(&ddrphy_t20, OBJECT(dev), "ddrphy-t20",
                                 &s->ddrc.phy_iomem, 0, 4 * KiB);
        memory_region_add_subregion(get_system_memory(), 0x13010000,
                                    &ddrphy_t20);
    }

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
                                                 "ingenic-gmac");
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gmac), 0,
                    s->memmap[INGENIC_T31_DEV_GMAC]);
    /* GMAC IRQ -> INTC source 55 (bank 1 bit 23) */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gmac), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 55));

    /*
     * USB OTG (DWC2) at 0x13500000 -> INTC source 21 (IRQ_OTG).
     * In bootrom-analysis mode (-bios), enable the deterministic
     * gadget-boot enumeration so the mask ROM's USB-boot path can
     * enumerate and receive an SPL. The DWC2 model still gates this on
     * "no USB host device attached", so normal host-mode use is intact.
     */
    {
        MachineState *ms = MACHINE(qdev_get_machine());
        if (ms && ms->firmware) {
            object_property_set_bool(OBJECT(&s->dwc2), "gadget-boot", true,
                                     &error_abort);
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->dwc2), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dwc2), 0,
                    s->memmap[INGENIC_T31_DEV_OTG]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dwc2), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 21));
    /*
     * Thingino's userspace usb-role tool toggles the OTG role by
     * writing CPM_USBRDT (0x10000040). Wire CPM's otg-id-change pin
     * to the DWC2 otg-id-change input so role changes update GOTGCTL
     * and fire CONIDSTSCHNG into the kernel.
     */
    qdev_connect_gpio_out_named(DEVICE(&s->cpm), "otg-id-change", 0,
                                qdev_get_gpio_in_named(DEVICE(&s->dwc2),
                                                       "otg-id-change", 0));

    /* PDMA at 0x13420000, IRQ -> INTC source 0 (IRQ_PDMA) */
    sysbus_realize(SYS_BUS_DEVICE(&s->pdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pdma), 0,
                    s->memmap[INGENIC_T31_DEV_PDMA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pdma), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 0));

    /*
     * TCU - 8 timer/PWM channels + the global registers + the embedded
     * OST (U-Boot's clocksource), all on one page. The watchdog overlaps
     * the low 16 bytes at higher priority. IRQ -> INTC source 27.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->tcu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->tcu), 0,
                    s->memmap[INGENIC_T31_DEV_TCU]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->tcu), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 27));

    /*
     * Standalone System OS Timer at 0x12000000 - BSP 3.10.14 kernel
     * uses this for the tick (T1, IRQ to MIPS IP4) and clocksource
     * (T2 64-bit free counter). IRQ wiring deferred until after the
     * kernel reaches a point where the tick is actually required.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->sysost), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sysost), 0,
                    s->memmap[INGENIC_T31_DEV_OST]);

    /* RTC at 0x10003000, IRQ -> INTC source 32 */
    sysbus_realize(SYS_BUS_DEVICE(&s->rtc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0,
                    s->memmap[INGENIC_T31_DEV_RTC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 32));

    /* DTRNG at 0x10072000, IRQ -> INTC source 34 */
    sysbus_realize(SYS_BUS_DEVICE(&s->dtrng), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dtrng), 0,
                    s->memmap[INGENIC_T31_DEV_DTRNG]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dtrng), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 34));

    /*
     * Dedicated PWM controller - only the T32/T33 carry it, at
     * 0x13450000; the device is realized for all variants but mapped
     * only where the hardware exists. IRQ -> INTC source 56.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->pwm), &error_fatal);
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;
        if (cpufam == 0x0032 || cpufam == 0x0033) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->pwm), 0, 0x13450000);
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->pwm), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 56));
        }
    }

    /* HASH (SHA-256) accelerator: IRQ -> INTC source 22 */
    sysbus_realize(SYS_BUS_DEVICE(&s->hash), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hash), 0,
                    s->memmap[INGENIC_T31_DEV_HASH]);

    /* RSA accelerator: IRQ -> INTC source 24 */
    sysbus_realize(SYS_BUS_DEVICE(&s->rsa), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rsa), 0,
                    s->memmap[INGENIC_T31_DEV_RSA]);

    /* GPIO controller: 3 ports + shadow page in a 64 KiB region. */
    /* T10/T20 use 0x100 GPIO port stride, T31+ use 0x1000 */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;
        if (cpufam == 0x0005 || cpufam == 0x2000) {
            qdev_prop_set_uint32(DEVICE(&s->gpio), "port-stride", 0x100);
        }
    }
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
     * NEMC chip-select window (parallel NOR) at phys 0x1A000000 (KSEG1
     * 0xBA000000). Boot mode 2 (nemc_boot) pin-muxes the bus then executes the
     * parallel NOR in place by jumping to 0xBA000004. Backed RAM so a NEMC
     * image (-device loader,addr=0x1a000000) can be run for bootrom analysis.
     */
    {
        MemoryRegion *nemc_cs = g_new0(MemoryRegion, 1);
        memory_region_init_ram(nemc_cs, OBJECT(dev), "ingenic-t31.nemc-cs",
                               16 * MiB, &error_abort);
        memory_region_add_subregion(get_system_memory(), 0x1a000000, nemc_cs);
    }

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
        MachineState *ms = MACHINE(qdev_get_machine());
        if (ms && ms->firmware) {
            /*
             * Bootrom analysis mode: run a real 32 KiB mask-ROM image
             * from the reset vector. Load -bios over the bootrom region
             * instead of the ERET stubs; the board starts the CPU at
             * 0xBFC00000 like real silicon.
             */
            if (load_image_mr(ms->firmware, &s->bootrom) < 0) {
                error_setg(errp, "ingenic: failed to load -bios '%s'",
                           ms->firmware);
                return;
            }
        } else {
            /* ERET opcode 0x42000018 at each exception vector offset so
             * stray exceptions return cleanly instead of looping in
             * unmapped memory. */
            const uint32_t eret = 0x42000018;
            static const uint32_t vectors[] = {
                0x000, 0x080, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380
            };
            unsigned v;
            for (v = 0; v < ARRAY_SIZE(vectors); v++) {
                rom_add_blob_fixed("ingenic-t31.eret", &eret, 4,
                                   0x1fc00000 + vectors[v]);
            }
        }
    }

    /*
     * UARTs - 16550 compatible with Ingenic SIRCR/UMR extensions.
     * Register shift 2 = 4-byte aligned registers. The 16550 model
     * covers registers 0-7 (offsets 0x00-0x1C). The Ingenic UART has
     * SIRCR at 0x20 and UMR at 0x24, so we add stubs for each UART
     * to absorb writes to the extended registers.
     *
     * INTC source numbers (from BSP arch/mips/xburst/soc-t31/include/
     * soc/irq.h): UART0=51, UART1=50, UART2=49. Wired so the tty
     * layer's IRQ-driven TX path completes - otherwise userspace
     * writes to /dev/console block forever in tty_write.
     */
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART0], 2,
                   qdev_get_gpio_in(DEVICE(&s->intc), 51),
                   115200, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    create_unimplemented_device("ingenic-t31-uart0-ext",
                                s->memmap[INGENIC_T31_DEV_UART0] + 0x20,
                                4 * KiB - 0x20);
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART1], 2,
                   qdev_get_gpio_in(DEVICE(&s->intc), 50),
                   115200, serial_hd(1), DEVICE_LITTLE_ENDIAN);
    create_unimplemented_device("ingenic-t31-uart1-ext",
                                s->memmap[INGENIC_T31_DEV_UART1] + 0x20,
                                4 * KiB - 0x20);
    serial_mm_init(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART2], 2,
                   qdev_get_gpio_in(DEVICE(&s->intc), 49),
                   115200, serial_hd(2), DEVICE_LITTLE_ENDIAN);
    create_unimplemented_device("ingenic-t31-uart2-ext",
                                s->memmap[INGENIC_T31_DEV_UART2] + 0x20,
                                4 * KiB - 0x20);

    /* HARB0 - SoC ID at offset 0x2C */
    memory_region_init_io(&s->harb0, OBJECT(dev), &ingenic_t31_harb0_ops,
                          s, "ingenic-t31-harb0", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_HARB0], &s->harb0);

    /* EFUSE - SoC variant and serial numbers */
    memory_region_init_io(&s->efuse, OBJECT(dev), &ingenic_t31_efuse_ops,
                          s, "ingenic-t31-efuse", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_EFUSE], &s->efuse);

    /* MSC0/MSC1 - SD/MMC controllers.
     * T10-T31 use the old Ingenic MSC register interface at 0x13450000.
     * T32/T33 switched to standard SDHCI at 0x13060000/0x13070000.
     * Both device types must be realized (QEMU asserts all children are
     * realized), but only the active variant gets memory-mapped. */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;
        MachineState *ms = MACHINE(qdev_get_machine());
        bool bootrom_mode = ms && ms->firmware;
        bool use_sdhci = !bootrom_mode &&
                         (cpufam == 0x0032 || cpufam == 0x0033);

        sysbus_realize(SYS_BUS_DEVICE(&s->msc[0]), &error_fatal);
        sysbus_realize(SYS_BUS_DEVICE(&s->msc[1]), &error_fatal);
        for (int j = 0; j < 2; j++) {
            object_property_set_uint(OBJECT(&s->sdhci[j]),
                                     "sd-spec-version", 3, &error_abort);
            object_property_set_uint(OBJECT(&s->sdhci[j]),
                                     "capareg",
                                     (1ULL << 26) |  /* 3.3V */
                                     (1ULL << 25) |  /* 3.0V */
                                     (1ULL << 24) |  /* 1.8V */
                                     (1ULL << 21) |  /* high-speed */
                                     (25 << 8),      /* base clock 25 MHz */
                                     &error_abort);
            sysbus_realize(SYS_BUS_DEVICE(&s->sdhci[j]), &error_fatal);
        }

        if (use_sdhci) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[0]), 0, 0x13060000);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[1]), 0, 0x13070000);
        } else if (bootrom_mode && (cpufam == 0x0032 || cpufam == 0x0033)) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[0]), 0, 0x13060000);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[1]), 0, 0x13070000);
        } else {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[0]),
                            0, s->memmap[INGENIC_T31_DEV_MSC0]);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[1]),
                            0, s->memmap[INGENIC_T31_DEV_MSC1]);
        }
    }

    /* Unimplemented device stubs */
    for (i = 0; i < ARRAY_SIZE(ingenic_t31_unimp); i++) {
        create_unimplemented_device(ingenic_t31_unimp[i].name,
                                    ingenic_t31_unimp[i].base,
                                    ingenic_t31_unimp[i].size);
    }
}

static const Property ingenic_t31_props[] = {
    DEFINE_PROP_STRING("soc-variant", IngenicT31State, soc_variant),
    DEFINE_PROP_UINT32("efuse-security", IngenicT31State, efuse_security, 0),
    DEFINE_PROP_STRING("efuse-keyhash", IngenicT31State, efuse_keyhash_hex),
};

static void ingenic_t31_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ingenic_t31_realize;
    dc->user_creatable = false;
    device_class_set_props(dc, ingenic_t31_props);
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
