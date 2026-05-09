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
#include "hw/mips/ingenic-t31.h"

/*
 * WDT (Watchdog Timer) - counts up from TCNT toward TDR using the
 * configured clock source. When TCNT reaches TDR with the counter
 * enabled, the WDT triggers a SoC reset. Both U-Boot's reset command
 * and Linux's reboot path arm a short timeout (~4ms) and busy-wait.
 * The userspace watchdog daemon kicks via the kernel which writes
 * TCNT=0, restarting the count.
 */
#define WDT_TDR_OFF     0x00
#define WDT_TCER_OFF    0x04
#define WDT_TCNT_OFF    0x08
#define WDT_TCSR_OFF    0x0C
#define WDT_TCER_EN     (1 << 0)
#define WDT_TCSR_PCK_EN (1 << 0)
#define WDT_TCSR_RTC_EN (1 << 1)
#define WDT_TCSR_EXT_EN (1 << 2)
#define WDT_TCSR_PRESCALE_SHIFT 3
#define WDT_TCSR_PRESCALE_MASK  (7 << WDT_TCSR_PRESCALE_SHIFT)

static struct {
    QEMUTimer *timer;
    uint16_t tdr;
    uint16_t tcnt_base;     /* TCNT value at start_ns */
    uint16_t tcsr;
    uint8_t  tcer;
    int64_t  start_ns;      /* when current count started */
} wdt_state;

static uint32_t wdt_tick_ns(uint16_t tcsr)
{
    /* Pick clock: prefer RTC, then EXT, then PCK. */
    uint64_t rate = 32768;          /* RTC default */
    if (tcsr & WDT_TCSR_EXT_EN) {
        rate = 24000000;            /* EXTAL */
    } else if (tcsr & WDT_TCSR_PCK_EN) {
        rate = 100000000;           /* PCK approx */
    }
    static const uint32_t pre[8] = {1, 4, 16, 64, 256, 1024, 1024, 1024};
    uint32_t prescale = pre[(tcsr >> WDT_TCSR_PRESCALE_SHIFT) & 7];
    /* Period of one count tick in ns. */
    return (uint32_t)((1000000000ULL * prescale) / rate);
}

static uint16_t wdt_current_tcnt(void)
{
    if (!(wdt_state.tcer & WDT_TCER_EN)) {
        return wdt_state.tcnt_base;
    }
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - wdt_state.start_ns;
    uint32_t period = wdt_tick_ns(wdt_state.tcsr);
    if (period == 0) {
        return wdt_state.tcnt_base;
    }
    uint64_t ticks = (uint64_t)elapsed / period;
    uint64_t cnt = (uint64_t)wdt_state.tcnt_base + ticks;
    return cnt > 0xFFFF ? 0xFFFF : (uint16_t)cnt;
}

static void wdt_reschedule(void)
{
    timer_del(wdt_state.timer);
    if (!(wdt_state.tcer & WDT_TCER_EN)) {
        return;
    }
    if (wdt_state.tcnt_base >= wdt_state.tdr) {
        /* Already past TDR - fire next tick. */
        timer_mod(wdt_state.timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
        return;
    }
    uint32_t period = wdt_tick_ns(wdt_state.tcsr);
    uint64_t remaining = (uint64_t)(wdt_state.tdr - wdt_state.tcnt_base) *
                         period;
    timer_mod(wdt_state.timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + remaining);
}

static void ingenic_t31_wdt_fire(void *opaque)
{
    /*
     * On real hardware the WDT pulses the SoC reset signal. Request
     * a guest reset directly so 'reset' (U-Boot) and 'reboot' (Linux)
     * actually restart the VM. watchdog_perform_action() is also
     * called so users can override with -action watchdog=pause/none.
     */
    wdt_state.tcer &= ~WDT_TCER_EN;
    watchdog_perform_action();
    qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
}

static void ingenic_t31_wdt_reset(void *opaque)
{
    /*
     * On system reset (including the one we just triggered) the WDT
     * registers come back to their default state. Without this, leftover
     * tcer/tdr/start_ns from the previous boot can cause spurious fires
     * after the next firmware load and break sequential reboots.
     */
    timer_del(wdt_state.timer);
    wdt_state.tdr = 0;
    wdt_state.tcer = 0;
    wdt_state.tcsr = 0;
    wdt_state.tcnt_base = 0;
    wdt_state.start_ns = 0;
}

static uint64_t ingenic_t31_wdt_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    switch (offset) {
    case WDT_TDR_OFF:  return wdt_state.tdr;
    case WDT_TCER_OFF: return wdt_state.tcer;
    case WDT_TCNT_OFF: return wdt_current_tcnt();
    case WDT_TCSR_OFF: return wdt_state.tcsr;
    }
    return 0;
}

static void ingenic_t31_wdt_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    /*
     * Snapshot the current count before changing state - any latch
     * to TDR/TCSR/TCER mid-count needs the live counter as the new
     * base or the timer math drifts.
     */
    uint16_t live = wdt_current_tcnt();

    switch (offset) {
    case WDT_TDR_OFF:
        wdt_state.tdr = (uint16_t)value;
        wdt_state.tcnt_base = live;
        wdt_state.start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        wdt_reschedule();
        break;
    case WDT_TCER_OFF:
        wdt_state.tcer = (uint8_t)value;
        wdt_state.tcnt_base = live;
        wdt_state.start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        wdt_reschedule();
        break;
    case WDT_TCNT_OFF:
        /* Linux's jz4740_wdt_ping writes 0 here to kick the dog. */
        wdt_state.tcnt_base = (uint16_t)value;
        wdt_state.start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        wdt_reschedule();
        break;
    case WDT_TCSR_OFF:
        wdt_state.tcsr = (uint16_t)value;
        wdt_state.tcnt_base = live;
        wdt_state.start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        wdt_reschedule();
        break;
    }
}

static const MemoryRegionOps ingenic_t31_wdt_ops = {
    .read = ingenic_t31_wdt_read,
    .write = ingenic_t31_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

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
    uint32_t apll;       /* CPAPCR register value */
    uint32_t mpll;       /* CPMPCR register value */
    uint32_t ram_mb;
} T31Variant;

static const T31Variant t31_variants[] = {
    /* T10 family (cpuid bits[27:12] = 0x0005) */
    { "t10",   0x10005000, 0x00000000, MNOD( 71,2,1,1), MNOD(100,1,2,1),  32 },  /* 860/600 */
    { "t10l",  0x10005000, 0x00000000, MNOD( 59,2,1,1), MNOD(100,1,2,1),  32 },  /* 712/600 */
    /* T20 family (cpuid bits[27:12] = 0x2000) */
    { "t20n",  0x12000000, 0x11110000, MNOD( 71,2,1,1), MNOD(125,3,1,1),  64 },  /* 860/500 */
    { "t20x",  0x12000000, 0x22220000, MNOD( 71,2,1,1), MNOD(100,1,2,1), 128 },  /* 860/600 */
    { "t20l",  0x12000000, 0x33330000, MNOD( 59,2,1,1), MNOD(125,3,1,1),  64 },  /* 712/500 */
    /* T21 family (cpuid bits[27:12] = 0x0021, InnoSilicon DDR PHY like T31) */
    { "t21n",  0x10021000, 0x11110000, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    { "t21l",  0x10021000, 0x33330000, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    { "t21z",  0x10021000, 0x55550000, MNOD( 72,1,2,1), MNOD(125,1,3,1),  64 },  /* 864/500 */
    /* T31 family (cpuid bits[27:12] = 0x0031) */
    { "t31n",  0x10031000, 0x11110000, MNOD(117,1,2,1), MNOD(125,1,3,1),  64 },  /* 1404/500 */
    { "t31x",  0x10031000, 0x22220000, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31l",  0x10031000, 0x33330000, MNOD( 84,1,2,1), MNOD(125,1,3,1),  64 },  /* 1008/500 */
    { "t31a",  0x10031000, 0x44440000, MNOD(125,1,2,1), MNOD(125,1,2,1), 128 },  /* 1500/750 */
    { "t31zl", 0x10031000, 0x55550000, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31zx", 0x10031000, 0x66660000, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31al", 0x10031000, 0xCCCC0000, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31zc", 0x10031000, 0xDDDD0000, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { "t31lc", 0x10031000, 0xEEEE0000, MNOD( 92,1,2,1), MNOD(125,1,3,1),  64 },  /* 1104/500 */
    { "qemu",  0x10031000, 0xEE000000, MNOD(116,1,2,1), MNOD(100,1,2,1), 128 },  /* 1392/600 */
    { NULL, 0, 0, 0, 0, 0 }
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
        return 0x51454D55;
    case EFUSE_SERIAL1:
    case EFUSE_SERIAL2:
    case EFUSE_SERIAL3:
        return 0x00000000;
    case EFUSE_SUBREMARK:
        return 0x00000000;
    case EFUSE_SUBSOCTYPE1:
        return s->efuse_subsoctype1;
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
    /* PDMA is a real device now, not unimplemented */
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
    object_initialize_child(obj, "dwc2", &s->dwc2, TYPE_DWC2_USB);
    object_initialize_child(obj, "pdma", &s->pdma, TYPE_INGENIC_T31_PDMA);
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
        s->harb0_cpuid = v->cpuid;
        s->variant = v;
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
                                                 "ingenic-t31-gmac");
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gmac), 0,
                    s->memmap[INGENIC_T31_DEV_GMAC]);
    /* GMAC IRQ -> INTC source 55 (bank 1 bit 23) */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gmac), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 55));

    /* USB OTG (DWC2) at 0x13500000 -> INTC source 21 (IRQ_OTG). */
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

    /* WDT at TCU base, overlapping OST with higher priority */
    memory_region_init_io(&s->wdt, OBJECT(dev), &ingenic_t31_wdt_ops,
                          NULL, "ingenic-t31-wdt", 0x10);
    memory_region_add_subregion_overlap(get_system_memory(),
                                        s->memmap[INGENIC_T31_DEV_TCU],
                                        &s->wdt, 1);
    if (!wdt_state.timer) {
        wdt_state.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       ingenic_t31_wdt_fire, NULL);
        qemu_register_reset(ingenic_t31_wdt_reset, NULL);
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

static const Property ingenic_t31_props[] = {
    DEFINE_PROP_STRING("soc-variant", IngenicT31State, soc_variant),
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
