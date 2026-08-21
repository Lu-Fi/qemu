/*
 * Ingenic T40/T41 System on Chip emulation
 *
 * The T40 and T41 are MIPS32r2 XBurst2 SoCs targeting IP cameras.
 * They share the same CPM register layout, Innophy DDR PHY (with
 * T40-specific offsets), Synopsys GMAC, and peripheral addresses.
 * T41 lacks the EPLL that T40 has.
 *
 * Key differences from the A1:
 *  - CPM register offsets are reshuffled (EPLL=0x58, VPLL=0xE0, etc.)
 *  - Synopsys GMAC at 0x134b0000 (not XGMAC)
 *  - Single USB OTG at 0x13500000
 *  - INTC APB at 0x10001000
 *  - DDR Innophy with T40-specific PHY offsets
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
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "net/net.h"
#include "exec/cpu-common.h"
#include "hw/mips/ingenic-t40.h"


/* HARB0 stub - SoC CPU ID at offset 0x2C */
#define HARB0_SOC_ID    0x2C

static uint64_t t40_harb0_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicT40State *s = opaque;
    if (offset == HARB0_SOC_ID) {
        return s->harb0_cpuid;
    }
    return 0;
}

static void t40_harb0_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
}

static const MemoryRegionOps t40_harb0_ops = {
    .read = t40_harb0_read,
    .write = t40_harb0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Global OST - same register layout as A1.
 * G_OSTCCR(0x00), G_OSTER(0x04), G_OSTCR(0x08),
 * G_OSTCNTH(0x0C), G_OSTCNTL(0x10), G_OSTCNTB(0x14).
 *
 * The kernel ingenic_core_ost driver registers both the Global OST
 * clocksource and the Core OST clockevent at the same rate:
 * ext_clk / CLK_DIV = 24 MHz / 1 = 24 MHz (it writes CSRDIV(1)=0 to
 * G_OSTCCR for divide-by-1). The Global OST must run at 24 MHz to
 * match COST_FREQ - any mismatch makes the clockevent fire early
 * relative to clocksource time, causing a timer re-arm storm that
 * only shows up under SMP (UP uses the per-CPU CP0 Count clocksource).
 */
#define G_OST_FREQ  24000000ULL

static struct {
    int64_t base_ns;
    uint32_t high_buf;
    uint32_t ccr;
    uint32_t enabled;
} t40_gost_state;

static uint64_t t40_gost_count(void)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - t40_gost_state.base_ns;
    return (uint64_t)(elapsed * G_OST_FREQ / 1000000000ULL);
}

static uint64_t t40_gost_read(void *opaque, hwaddr offset, unsigned size)
{
    uint64_t cnt;

    switch (offset) {
    case 0x00:
        return t40_gost_state.ccr;
    case 0x04:
        return t40_gost_state.enabled;
    case 0x0C:
        return (uint32_t)(t40_gost_count() >> 32);
    case 0x10:
        cnt = t40_gost_count();
        t40_gost_state.high_buf = (uint32_t)(cnt >> 32);
        return (uint32_t)cnt;
    case 0x14:
        return t40_gost_state.high_buf;
    default:
        return 0;
    }
}

static void t40_gost_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    switch (offset) {
    case 0x00:
        t40_gost_state.ccr = (uint32_t)value;
        break;
    case 0x04:
        if ((value & 1) && !t40_gost_state.enabled) {
            t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        t40_gost_state.enabled = (uint32_t)value;
        break;
    case 0x08:
        if (value & 1) {
            t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        break;
    }
}

static const MemoryRegionOps t40_gost_ops = {
    .read = t40_gost_read,
    .write = t40_gost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void t40_gost_reset(void *opaque)
{
    t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    t40_gost_state.high_buf = 0;
    t40_gost_state.ccr = 0;
    t40_gost_state.enabled = 1;
}

/*
 * Core OST - per-CPU clockevent timer at 0x12100000.
 * Same register layout as A1: OSTDFR compare, OSTER enable, IRQ to IP4.
 */
#define COST_FREQ       24000000ULL
#define COST_OSTCCR     0x00
#define COST_OSTER      0x04
#define COST_OSTCR      0x08
#define COST_OSTFR      0x0C
#define COST_OSTMR      0x10
#define COST_OSTDFR     0x14
#define COST_OSTCNT     0x18

/*
 * Per-CPU Core OST state. CPU0 at N_OST+0x000, CPU1 at N_OST+0x100.
 */
#define T40_MAX_CPUS 2

static struct {
    int64_t base_ns;
    uint32_t dfr;
    uint32_t enabled;
    uint32_t flag;
    uint32_t mask;
} t40_cost_state[T40_MAX_CPUS];

typedef struct {
    IngenicT40State *soc;
    int cpu_id;
} T40CostCtx;

static T40CostCtx t40_cost_ctx[T40_MAX_CPUS];

static void t40_cost_fire(void *opaque)
{
    T40CostCtx *ctx = opaque;
    IngenicT40State *s = ctx->soc;
    int id = ctx->cpu_id;
    t40_cost_state[id].flag = 1;
    if (!t40_cost_state[id].mask && s->cost_irq[id]) {
        qemu_irq_raise(s->cost_irq[id]);
    }
}

static void t40_cost_arm(IngenicT40State *s, int id)
{
    if (!t40_cost_state[id].enabled || !t40_cost_state[id].dfr) {
        timer_del(s->cost_timer[id]);
        return;
    }
    int64_t period_ns = (int64_t)t40_cost_state[id].dfr *
                        NANOSECONDS_PER_SECOND / COST_FREQ;
    int64_t fire = t40_cost_state[id].base_ns + period_ns;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (fire <= now) {
        fire = now + 1;
    }
    timer_mod(s->cost_timer[id], fire);
}

static uint64_t t40_cost_read(void *opaque, hwaddr offset, unsigned size)
{
    int id = (offset >> 8) & 1;

    switch (offset & 0xFF) {
    case COST_OSTER:
        return t40_cost_state[id].enabled;
    case COST_OSTFR:
        return t40_cost_state[id].flag;
    case COST_OSTMR:
        return t40_cost_state[id].mask;
    case COST_OSTDFR:
        return t40_cost_state[id].dfr;
    case COST_OSTCNT: {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t elapsed = now - t40_cost_state[id].base_ns;
        return (uint32_t)(elapsed * COST_FREQ / NANOSECONDS_PER_SECOND);
    }
    default:
        return 0;
    }
}

static void t40_cost_write(void *opaque, hwaddr offset,
                           uint64_t value, unsigned size)
{
    IngenicT40State *s = opaque;
    int id = (offset >> 8) & 1;

    switch (offset & 0xFF) {
    case COST_OSTCCR:
        break;
    case COST_OSTER:
        if ((value & 1) && !t40_cost_state[id].enabled) {
            t40_cost_state[id].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        t40_cost_state[id].enabled = (uint32_t)value & 1;
        t40_cost_arm(s, id);
        break;
    case COST_OSTCR:
        if (value & 1) {
            t40_cost_state[id].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            t40_cost_arm(s, id);
        }
        break;
    case COST_OSTFR:
        if (!(value & 1)) {
            t40_cost_state[id].flag = 0;
            if (s->cost_irq[id]) {
                qemu_irq_lower(s->cost_irq[id]);
            }
        }
        t40_cost_state[id].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        t40_cost_arm(s, id);
        break;
    case COST_OSTMR:
        t40_cost_state[id].mask = (uint32_t)value & 1;
        break;
    case COST_OSTDFR:
        t40_cost_state[id].dfr = (uint32_t)value;
        t40_cost_state[id].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        t40_cost_arm(s, id);
        break;
    }
}

static const MemoryRegionOps t40_cost_ops = {
    .read = t40_cost_read,
    .write = t40_cost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* EFUSE stub - same layout as A1 (SUBSOCTYPE2 at 0x250) */
#define T40_EFUSE_SERIAL0     0x200
#define T40_EFUSE_SERIAL1     0x204
#define T40_EFUSE_SERIAL2     0x208
#define T40_EFUSE_SECURITY    0x224
#define T40_EFUSE_BOOTCFG     0x22C
#define T40_EFUSE_SUBSOCTYPE1 0x238
#define T40_EFUSE_CHIPID3     0x23C
#define T40_EFUSE_SUBSOCTYPE2 0x250

static uint64_t t40_efuse_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicT40State *s = opaque;

    switch (offset) {
    case T40_EFUSE_SERIAL0:
        return 0x40401234;
    case T40_EFUSE_SERIAL1:
        return 0x56780000;
    case T40_EFUSE_SERIAL2:
        return 0x12100080;
    case T40_EFUSE_SECURITY:
        return s->efuse_security;
    case T40_EFUSE_BOOTCFG:
        return s->efuse_bootcfg;
    case T40_EFUSE_CHIPID3:
        return 0x00000000;
    case T40_EFUSE_SUBSOCTYPE1:
        return 0x00000000;
    case T40_EFUSE_SUBSOCTYPE2:
        return s->efuse_subsoctype2;
    default:
        /* Key hash at 0x280-0x29F (8 words) */
        if (offset >= 0x280 && offset < 0x2A0) {
            unsigned idx = (offset - 0x280) / 4;
            return s->efuse_keyhash[idx];
        }
        return 0;
    }
}

static void t40_efuse_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
}

static const MemoryRegionOps t40_efuse_ops = {
    .read = t40_efuse_read,
    .write = t40_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/* Variant table */
#define PLL_ON_EN  ((1 << 3) | (1 << 0))
#define MNOD(nf, nr, od1, od0)  (((nf) << 20) | ((nr) << 14) | \
                                 ((od1) << 11) | ((od0) << 8) | PLL_ON_EN)

typedef struct {
    const char *name;
    uint32_t cpuid;
    uint32_t type2;
    uint32_t apll;
    uint32_t mpll;
    uint32_t ram_mb;
    bool has_epll;
} T40Variant;

static const T40Variant t40_variants[] = {
    /* T40 family: socId = (cpuid >> 12) & 0xFFFF */
    { "t40n",  0x00040000, 0x11110000, MNOD(67,1,2,1), MNOD(134,1,2,1), 256, true  },
    { "t40nn", 0x00040000, 0x11110000, MNOD(67,1,2,1), MNOD(134,1,2,1), 128, true  },
    { "t40xp", 0x00040000, 0x22220000, MNOD(67,1,2,1), MNOD(134,1,2,1), 128, true  },
    { "t40a",  0x00040000, 0x44440000, MNOD(67,1,2,1), MNOD(134,1,2,1), 256, true  },
    /* T41 family: no EPLL */
    { "t41nq", 0x00040100, 0xAAAA0000, MNOD(67,1,2,1), MNOD(125,1,3,1), 256, false },
    { "t41lq", 0x00040100, 0xBBBB0000, MNOD(67,1,2,1), MNOD(125,1,3,1), 128, false },
    { "t41zx", 0x00040100, 0xCCCC0000, MNOD(67,1,2,1), MNOD(125,1,3,1), 256, false },
    /* Default */
    { "qemu",  0x00040000, 0xEE000000, MNOD(67,1,2,1), MNOD(134,1,2,1), 256, true  },
    { NULL, 0, 0, 0, 0, 0, false }
};

static const T40Variant *ingenic_t40_find_variant(const char *name)
{
    if (name && name[0]) {
        for (int i = 0; t40_variants[i].name; i++) {
            if (g_ascii_strcasecmp(name, t40_variants[i].name) == 0) {
                return &t40_variants[i];
            }
        }
    }
    return &t40_variants[ARRAY_SIZE(t40_variants) - 2];
}

/* T40/T41 physical memory map */
const hwaddr ingenic_t40_memmap[] = {
    /* APB bus */
    [INGENIC_T40_DEV_CPM]        = 0x10000000,
    [INGENIC_T40_DEV_INTC]       = 0x10001000,
    [INGENIC_T40_DEV_TCU]        = 0x10002000,
    [INGENIC_T40_DEV_RTC]        = 0x132a0000,
    [INGENIC_T40_DEV_GPIO]       = 0x10010000,
    [INGENIC_T40_DEV_UART0]      = 0x10030000,
    [INGENIC_T40_DEV_UART1]      = 0x10031000,
    [INGENIC_T40_DEV_UART2]      = 0x10032000,
    [INGENIC_T40_DEV_UART3]      = 0x10033000,
    [INGENIC_T40_DEV_SSI_SLV]    = 0x10040000,
    [INGENIC_T40_DEV_SSI0]       = 0x10043000,
    [INGENIC_T40_DEV_I2C0]       = 0x10050000,
    [INGENIC_T40_DEV_I2C1]       = 0x10051000,
    [INGENIC_T40_DEV_I2C2]       = 0x10053000,
    [INGENIC_T40_DEV_I2C3]       = 0x10051000,
    [INGENIC_T40_DEV_USBPHY]     = 0x10060000,
    [INGENIC_T40_DEV_SADC]       = 0x10070000,
    [INGENIC_T40_DEV_DMIC]       = 0x10034000,
    [INGENIC_T40_DEV_MIPI_PHY]   = 0x10022000,
    [INGENIC_T40_DEV_MIPI]       = 0x10023000,

    /* Core region */
    [INGENIC_T40_DEV_G_OST]      = 0x12000000,
    [INGENIC_T40_DEV_N_OST]      = 0x12100000,
    [INGENIC_T40_DEV_CCU]        = 0x12200000,
    [INGENIC_T40_DEV_INTCN]      = 0x12300000,
    [INGENIC_T40_DEV_SRAM]       = 0x12400000,
    [INGENIC_T40_DEV_NNDMA]      = 0x12500000,
    [INGENIC_T40_DEV_RISCV]      = 0x12210000,

    /* AHB bus */
    [INGENIC_T40_DEV_HARB0]      = 0x13000000,
    [INGENIC_T40_DEV_DDR_PHY]    = 0x13011000,
    [INGENIC_T40_DEV_DDRC]       = 0x134f0000,
    [INGENIC_T40_DEV_LCDC]       = 0x13050000,
    [INGENIC_T40_DEV_MSC0]       = 0x13060000,
    [INGENIC_T40_DEV_MSC1]       = 0x13070000,
    [INGENIC_T40_DEV_IPU]        = 0x13080000,
    [INGENIC_T40_DEV_BSCALER]    = 0x13090000,
    [INGENIC_T40_DEV_MONITOR]    = 0x130a0000,
    [INGENIC_T40_DEV_I2D]        = 0x130b0000,
    [INGENIC_T40_DEV_VO]         = 0x130c0000,
    [INGENIC_T40_DEV_DRAWBOX]    = 0x130d0000,
    [INGENIC_T40_DEV_RADIX]      = 0x13100000,
    [INGENIC_T40_DEV_EL150]      = 0x13200000,
    [INGENIC_T40_DEV_ISP]        = 0x13300000,
    [INGENIC_T40_DEV_NEMC]       = 0x13410000,
    [INGENIC_T40_DEV_PDMA]       = 0x13420000,
    [INGENIC_T40_DEV_AES]        = 0x13430000,
    [INGENIC_T40_DEV_SFC]        = 0x13440000,
    [INGENIC_T40_DEV_GMAC]       = 0x134b0000,
    [INGENIC_T40_DEV_RSA]        = 0x134c0000,
    [INGENIC_T40_DEV_HASH]       = 0x13480000,
    [INGENIC_T40_DEV_OTG]        = 0x13500000,
    [INGENIC_T40_DEV_EFUSE]      = 0x13540000,

    /* Memory */
    [INGENIC_T40_DEV_SDRAM]      = 0x00000000,
};

static const struct {
    const char *name;
    hwaddr base;
    hwaddr size;
} ingenic_t40_unimp[] = {
    { "ingenic-t40-rtc",       0x132a0000, 4 * KiB },
    { "ingenic-t40-ssi-slv",   0x10040000, 4 * KiB },
    { "ingenic-t40-ssi0",      0x10043000, 4 * KiB },
    { "ingenic-t40-usbphy",    0x10060000, 4 * KiB },
    { "ingenic-t40-sadc",      0x10070000, 4 * KiB },
    { "ingenic-t40-dmic",      0x10034000, 4 * KiB },
    { "ingenic-t40-mipiphy",   0x10022000, 4 * KiB },
    { "ingenic-t40-mipi",      0x10023000, 4 * KiB },
    { "ingenic-t40-riscv",     0x12210000, 64 * KiB },
    { "ingenic-t40-nndma",     0x12500000, 64 * KiB },
    { "ingenic-t40-lcdc",      0x13050000, 64 * KiB },
    { "ingenic-t40-ipu",       0x13080000, 64 * KiB },
    { "ingenic-t40-bscaler",   0x13090000, 64 * KiB },
    { "ingenic-t40-monitor",   0x130a0000, 64 * KiB },
    { "ingenic-t40-i2d",       0x130b0000, 64 * KiB },
    { "ingenic-t40-vo",        0x130c0000, 64 * KiB },
    { "ingenic-t40-drawbox",   0x130d0000, 64 * KiB },
    { "ingenic-t40-radix",     0x13100000, 64 * KiB },
    { "ingenic-t40-el150",     0x13200000, 64 * KiB },
    { "ingenic-t40-isp",       0x13300000, 1024 * KiB },
    { "ingenic-t40-nemc",      0x13410000, 64 * KiB },
    { "ingenic-t40-aes",       0x13430000, 4 * KiB },
    /* HASH and RSA are real device models */
};

static void ingenic_t40_init(Object *obj)
{
    IngenicT40State *s = INGENIC_T40(obj);

    s->memmap = ingenic_t40_memmap;

    object_initialize_child(obj, "cpm", &s->cpm, TYPE_INGENIC_T40_CPM);
    object_initialize_child(obj, "ddrc", &s->ddrc, TYPE_INGENIC_DDRC);
    object_initialize_child(obj, "sfc", &s->sfc, TYPE_INGENIC_A1_SFC);
    object_initialize_child(obj, "gmac", &s->gmac, TYPE_INGENIC_GMAC);
    object_initialize_child(obj, "tcu", &s->tcu, TYPE_INGENIC_TCU);
    object_initialize_child(obj, "dtrng", &s->dtrng, TYPE_INGENIC_DTRNG);
    object_initialize_child(obj, "pwm", &s->pwm, TYPE_INGENIC_PWM);
    object_initialize_child(obj, "rtc", &s->rtc, TYPE_INGENIC_RTC);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_INGENIC_GPIO);
    object_initialize_child(obj, "intc", &s->intc, TYPE_INGENIC_INTC);
    object_initialize_child(obj, "ccu", &s->ccu, TYPE_INGENIC_XBURST2_CCU);
    object_initialize_child(obj, "i2c0", &s->i2c[0], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c1", &s->i2c[1], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c2", &s->i2c[2], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c3", &s->i2c[3], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "sdhci0", &s->sdhci[0], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "sdhci1", &s->sdhci[1], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "msc0", &s->msc[0], TYPE_INGENIC_MSC);
    object_initialize_child(obj, "msc1", &s->msc[1], TYPE_INGENIC_MSC);
    object_initialize_child(obj, "dwc2", &s->dwc2, TYPE_DWC2_USB);
    object_initialize_child(obj, "pdma", &s->pdma, TYPE_INGENIC_PDMA);
    object_initialize_child(obj, "hash", &s->hash, TYPE_INGENIC_HASH);
    object_initialize_child(obj, "rsa", &s->rsa, TYPE_INGENIC_RSA);
    object_property_add_const_link(OBJECT(&s->dwc2), "dma-mr",
                                   OBJECT(get_system_memory()));
}

static void ingenic_t40_realize(DeviceState *dev, Error **errp)
{
    IngenicT40State *s = INGENIC_T40(dev);
    unsigned i;

    {
        const T40Variant *v = ingenic_t40_find_variant(s->soc_variant);
        s->efuse_subsoctype2 = v->type2;
        s->harb0_cpuid = v->cpuid;
        s->variant = v;

        qdev_prop_set_bit(DEVICE(&s->cpm), "has-epll", v->has_epll);
    }

    /* CPM */
    sysbus_realize(SYS_BUS_DEVICE(&s->cpm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->cpm), 0,
                    s->memmap[INGENIC_T40_DEV_CPM]);
    {
        const T40Variant *v = s->variant;
        s->cpm.regs[0x10 / 4] = v->apll;
        s->cpm.regs[0x14 / 4] = v->mpll;
        if (v->has_epll) {
            s->cpm.regs[0x58 / 4] = MNOD(125, 1, 2, 1) | PLL_ON_EN;
        }
        s->cpm.regs[0xE0 / 4] = MNOD(100, 1, 2, 1) | PLL_ON_EN;
    }

    /* DDR Controller + PHY */
    sysbus_realize(SYS_BUS_DEVICE(&s->ddrc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 0,
                    s->memmap[INGENIC_T40_DEV_DDRC]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 1,
                    s->memmap[INGENIC_T40_DEV_DDR_PHY]);

    /* SFC V2 (same IP as A1), IRQ -> INTC source 7 */
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc), 0,
                    s->memmap[INGENIC_T40_DEV_SFC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 7));

    /*
     * Synopsys GMAC (same IP as T31). Bind it to the host network
     * backend: a -nic via qemu_configure_nic_device, otherwise a bare
     * -netdev id=n0 (the convention used by this board's run scripts).
     */
    if (!qemu_configure_nic_device(DEVICE(&s->gmac), true, NULL)) {
        NetClientState *nc = qemu_find_netdev("n0");
        if (nc) {
            qdev_prop_set_netdev(DEVICE(&s->gmac), "netdev", nc);
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gmac), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gmac), 0,
                    s->memmap[INGENIC_T40_DEV_GMAC]);

    /*
     * TCU - 8 timer/PWM channels + the global registers + the embedded
     * OST, all on one page. The watchdog overlaps the low 16 bytes at
     * higher priority. IRQ -> INTC source 27.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->tcu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->tcu), 0,
                    s->memmap[INGENIC_T40_DEV_TCU]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->tcu), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 27));

    /* DTRNG - true random number generator, IRQ -> INTC source 34 */
    sysbus_realize(SYS_BUS_DEVICE(&s->dtrng), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dtrng), 0, 0x10072000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dtrng), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 34));

    /* PWM controller at 0x13460000, IRQ -> INTC source 31 */
    sysbus_realize(SYS_BUS_DEVICE(&s->pwm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pwm), 0, 0x13460000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pwm), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 31));

    /* RTC, IRQ -> INTC source 3 */
    sysbus_realize(SYS_BUS_DEVICE(&s->rtc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0,
                    s->memmap[INGENIC_T40_DEV_RTC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 3));

    /* GPIO (4 ports, same stride) */
    sysbus_realize(SYS_BUS_DEVICE(&s->gpio), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0,
                    s->memmap[INGENIC_T40_DEV_GPIO]);

    /* INTC (core INTC) - one register window per core */
    qdev_prop_set_uint32(DEVICE(&s->intc), "num-cpus", s->num_cpus);
    sysbus_realize(SYS_BUS_DEVICE(&s->intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0,
                    s->memmap[INGENIC_T40_DEV_INTCN]);

    /* I2C (4 channels) */
    for (i = 0; i < 4; i++) {
        sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), &error_fatal);
        hwaddr base;
        switch (i) {
        case 0: base = s->memmap[INGENIC_T40_DEV_I2C0]; break;
        case 1: base = s->memmap[INGENIC_T40_DEV_I2C1]; break;
        case 2: base = s->memmap[INGENIC_T40_DEV_I2C2]; break;
        default: base = s->memmap[INGENIC_T40_DEV_I2C3]; break;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0, base);
    }

    /* MSC (2 channels).
     * In bootrom mode (-bios), map the Ingenic MSC controller which
     * matches the register layout the ROM's SD driver expects.
     * Otherwise map SDHCI for U-Boot/Linux compatibility.
     * Both are always realized to satisfy QOM. */
    {
        MachineState *ms = MACHINE(qdev_get_machine());
        bool bootrom_mode = ms && ms->firmware;

        for (i = 0; i < 2; i++) {
            sysbus_realize(SYS_BUS_DEVICE(&s->msc[i]), &error_fatal);
            object_property_set_uint(OBJECT(&s->sdhci[i]),
                                     "sd-spec-version", 3, &error_abort);
            object_property_set_uint(OBJECT(&s->sdhci[i]),
                                     "capareg",
                                     (1ULL << 26) | (1ULL << 25) |
                                     (1ULL << 24) | (1ULL << 21) | (25 << 8),
                                     &error_abort);
            sysbus_realize(SYS_BUS_DEVICE(&s->sdhci[i]), &error_fatal);

            if (bootrom_mode) {
                sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[i]), 0,
                                s->memmap[INGENIC_T40_DEV_MSC0 + i]);
            } else {
                sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[i]), 0,
                                s->memmap[INGENIC_T40_DEV_MSC0 + i]);
            }
        }
    }

    /* USB DWC2 OTG (single) -> INTC source 21 */
    sysbus_realize(SYS_BUS_DEVICE(&s->dwc2), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dwc2), 0,
                    s->memmap[INGENIC_T40_DEV_OTG]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dwc2), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 21));

    /* PDMA */
    sysbus_realize(SYS_BUS_DEVICE(&s->pdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pdma), 0,
                    s->memmap[INGENIC_T40_DEV_PDMA]);

    /* HASH (SHA-256) accelerator */
    sysbus_realize(SYS_BUS_DEVICE(&s->hash), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hash), 0,
                    s->memmap[INGENIC_T40_DEV_HASH]);

    /* RSA accelerator */
    sysbus_realize(SYS_BUS_DEVICE(&s->rsa), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rsa), 0,
                    s->memmap[INGENIC_T40_DEV_RSA]);

    /* UARTs (4 channels, IRQs: UART0=51, UART1=50, UART2=49, UART3=48) */
    for (i = 0; i < 4; i++) {
        hwaddr uart_base = s->memmap[INGENIC_T40_DEV_UART0] + i * 0x1000;
        serial_mm_init(get_system_memory(), uart_base, 2,
                       qdev_get_gpio_in(DEVICE(&s->intc), 51 - i),
                       115200, serial_hd(i),
                       DEVICE_LITTLE_ENDIAN);
        char name[32];
        snprintf(name, sizeof(name), "ingenic-t40-uart%u-ext", i);
        create_unimplemented_device(name, uart_base + 0x20,
                                    4 * KiB - 0x20);
    }

    /* HARB0 (inline, SoC ID at offset 0x2C) */
    memory_region_init_io(&s->harb0, OBJECT(s), &t40_harb0_ops, s,
                          "ingenic-t40-harb0", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T40_DEV_HARB0], &s->harb0);

    /* EFUSE (inline) */
    memory_region_init_io(&s->efuse, OBJECT(s), &t40_efuse_ops, s,
                          "ingenic-t40-efuse", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T40_DEV_EFUSE], &s->efuse);

    /* Bootrom at 0x1FC00000 (kseg1 0xBFC00000). 32 KiB, same as T31.
     * Populated with ERET at exception vectors by default; a real ROM
     * image can be loaded via -bios to replace this. */
    memory_region_init_rom(&s->bootrom, OBJECT(dev), "ingenic-t40.bootrom",
                           32 * KiB, &error_abort);
    memory_region_add_subregion(get_system_memory(), 0x1fc00000, &s->bootrom);
    {
        MachineState *ms = MACHINE(qdev_get_machine());
        if (ms && ms->firmware) {
            if (load_image_mr(ms->firmware, &s->bootrom) < 0) {
                error_setg(errp, "ingenic-t40: failed to load -bios '%s'",
                           ms->firmware);
                return;
            }
        } else {
            const uint32_t eret = 0x42000018;
            static const uint32_t vectors[] = {
                0x000, 0x080, 0x100, 0x180, 0x200, 0x280, 0x300, 0x380
            };
            unsigned v;
            for (v = 0; v < ARRAY_SIZE(vectors); v++) {
                rom_add_blob_fixed("ingenic-t40.eret", &eret, 4,
                                   0x1fc00000 + vectors[v]);
            }
        }
    }

    /* Global OST (inline) */
    memory_region_init_io(&s->gost, OBJECT(s), &t40_gost_ops, s,
                          "ingenic-t40-gost", 0x20);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T40_DEV_G_OST], &s->gost);
    qemu_register_reset(t40_gost_reset, NULL);

    /* Core OST (per-CPU: CPU0 at +0x000, CPU1 at +0x100) */
    for (i = 0; i < T40_MAX_CPUS; i++) {
        t40_cost_ctx[i].soc = s;
        t40_cost_ctx[i].cpu_id = i;
        s->cost_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        t40_cost_fire, &t40_cost_ctx[i]);
    }
    memory_region_init_io(&s->cost, OBJECT(s), &t40_cost_ops, s,
                          "ingenic-t40-cost", 0x200);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T40_DEV_N_OST], &s->cost);

    /* CCU - XBurst2 SMP control unit */
    qdev_prop_set_uint32(DEVICE(&s->ccu), "num-cpus", s->num_cpus);
    sysbus_realize(SYS_BUS_DEVICE(&s->ccu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccu), 0,
                    s->memmap[INGENIC_T40_DEV_CCU]);

    /* SRAM */
    memory_region_init_ram(&s->sram, OBJECT(s), "ingenic-t40.sram",
                           INGENIC_T40_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T40_DEV_SRAM], &s->sram);

    /*
     * Secondary-core boot SRAM at 0x12660000. The SPL copies a small
     * "wait" stub here and points the CCU reset-entry register at it
     * when parking CPU1; without backing RAM that copy bus-errors.
     */
    memory_region_init_ram(&s->cpu1_sram, OBJECT(s), "ingenic-t40.cpu1-sram",
                           64 * KiB, &error_fatal);
    memory_region_add_subregion(get_system_memory(), 0x12660000,
                                &s->cpu1_sram);

    /* Unimplemented peripherals */
    for (i = 0; i < ARRAY_SIZE(ingenic_t40_unimp); i++) {
        create_unimplemented_device(ingenic_t40_unimp[i].name,
                                    ingenic_t40_unimp[i].base,
                                    ingenic_t40_unimp[i].size);
    }
}

static const Property ingenic_t40_properties[] = {
    DEFINE_PROP_STRING("soc-variant", IngenicT40State, soc_variant),
    DEFINE_PROP_UINT32("efuse-security", IngenicT40State, efuse_security, 0),
    DEFINE_PROP_UINT32("efuse-bootcfg", IngenicT40State, efuse_bootcfg, 0),
};

static void ingenic_t40_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ingenic_t40_realize;
    device_class_set_props(dc, ingenic_t40_properties);
    dc->user_creatable = false;
}

static const TypeInfo ingenic_t40_type_info = {
    .name = TYPE_INGENIC_T40,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(IngenicT40State),
    .instance_init = ingenic_t40_init,
    .class_init = ingenic_t40_class_init,
};

static void ingenic_t40_register_types(void)
{
    type_register_static(&ingenic_t40_type_info);
}

type_init(ingenic_t40_register_types)
