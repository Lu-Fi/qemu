/*
 * Ingenic A1 System on Chip emulation
 *
 * The Ingenic A1 is a dual-core MIPS32r2 XBurst2 SoC featuring DDR3
 * memory, dual SPI flash controllers, dual Gigabit Ethernet, triple
 * USB OTG, HDMI, SATA, and hardware video encode/decode. It targets
 * NVR and smart-display applications.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/char/ingenic-uart.h"
#include "hw/misc/unimp.h"
#include "hw/misc/ingenic-busregs.h"
#include "hw/misc/ingenic-nemc.h"
#include "system/watchdog.h"
#include "system/runstate.h"
#include "system/reset.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/mips/ingenic-boot.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "net/net.h"
#include "system/physmem.h"
#include "hw/mips/ingenic-a1.h"
#include "hw/mips/ingenic-xburst2-l2.h"
#include "hw/misc/ingenic-efuse-id.h"
#include "hw/mips/ingenic-dwc2.h"


/*
 * HARB0 stub - returns SoC CPU ID at offset 0x2C.
 * A1: (cpuid >> 12) & 0xFFFF identifies the SoC family.
 */
#define HARB0_SOC_ID    0x2C

static uint64_t ingenic_a1_harb0_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicA1State *s = opaque;
    /* The chip ID at 0x2c of every 0x40, as the part reads its HARB0 */
    if (offset % 0x40 == HARB0_SOC_ID) {
        return s->harb0_cpuid;
    }
    return 0;
}

static void ingenic_a1_harb0_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
}

static const MemoryRegionOps ingenic_a1_harb0_ops = {
    .read = ingenic_a1_harb0_read,
    .write = ingenic_a1_harb0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * Global OST - 64-bit free-running counter at 24 MHz.
 * A1/XBurst2 layout: G_OSTCCR(0x00), G_OSTER(0x04), G_OSTCR(0x08),
 * G_OSTCNTH(0x0C), G_OSTCNTL(0x10), G_OSTCNTB(0x14).
 * This differs from the T31 sysost layout where 0x0C/0x10 are TFR/TMR.
 *
 * The kernel ingenic_core_ost driver registers the Global OST
 * clocksource and the Core OST clockevent at the same rate
 * (ext_clk / CLK_DIV = 24 MHz / 1; it writes CSRDIV(1)=0 to G_OSTCCR
 * for divide-by-1). The Global OST must run at 24 MHz to match
 * COST_FREQ - a mismatch makes the clockevent fire early relative to
 * clocksource time, causing an SMP timer re-arm storm. U-Boot divides
 * by 4 and times its delays at 6 MHz.
 */
#define G_OST_FREQ  24000000ULL

/* G_OSTCCR divides EXTCLK by 1, 4 or 16 */
static uint64_t a1_gost_count(IngenicA1State *s)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                      s->gost_state.base_ns;

    return s->gost_state.base_count +
           muldiv64(elapsed, G_OST_FREQ >> (2 * (s->gost_state.ccr & 3)),
                    NANOSECONDS_PER_SECOND);
}

static uint64_t ingenic_a1_gost_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicA1State *s = opaque;
    uint64_t cnt;

    switch (offset) {
    case 0x00: /* G_OSTCCR */
        return s->gost_state.ccr;
    case 0x04: /* G_OSTER */
        return s->gost_state.enabled;
    case 0x0C: /* G_OSTCNTH */
        return (uint32_t)(a1_gost_count(s) >> 32);
    case 0x10: /* G_OSTCNTL */
        cnt = a1_gost_count(s);
        s->gost_state.high_buf = (uint32_t)(cnt >> 32);
        return (uint32_t)cnt;
    case 0x14: /* G_OSTCNTB */
        return s->gost_state.high_buf;
    default:
        return 0;
    }
}

static void ingenic_a1_gost_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicA1State *s = opaque;

    switch (offset) {
    case 0x00: /* G_OSTCCR */
        s->gost_state.base_count = a1_gost_count(s);
        s->gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->gost_state.ccr = (uint32_t)value;
        break;
    case 0x04: /* G_OSTER */
        if ((value & 1) && !s->gost_state.enabled) {
            s->gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->gost_state.base_count = 0;
        }
        s->gost_state.enabled = (uint32_t)value;
        break;
    case 0x08: /* G_OSTCR */
        if (value & 1) {
            s->gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            s->gost_state.base_count = 0;
        }
        break;
    }
}

static const MemoryRegionOps ingenic_a1_gost_ops = {
    .read = ingenic_a1_gost_read,
    .write = ingenic_a1_gost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_a1_gost_reset(void *opaque)
{
    IngenicA1State *s = opaque;

    s->gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->gost_state.base_count = 0;
    s->gost_state.high_buf = 0;
    s->gost_state.ccr = 0;
    s->gost_state.enabled = 1;
}

/*
 * Core OST - per-CPU clockevent timer at 0x12100000.
 * The kernel programs OSTDFR with a compare value, enables OSTER,
 * and expects an IRQ (MIPS IP4) when the counter reaches OSTDFR.
 * 24 MHz clock (prescale 1).
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
#define A1_MAX_CPUS 2

typedef struct {
    IngenicA1State *soc;
    int cpu_id;
} A1CostCtx;

static A1CostCtx a1_cost_ctx[A1_MAX_CPUS];

static void a1_cost_reset(void *opaque)
{
    IngenicA1State *s = opaque;

    for (int i = 0; i < A1_MAX_CPUS; i++) {
        s->cost_state[i].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->cost_state[i].dfr = 0;
        s->cost_state[i].enabled = 0;
        s->cost_state[i].flag = 0;
        s->cost_state[i].mask = 0;
        if (s && s->cost_timer[i]) {
            timer_del(s->cost_timer[i]);
        }
        if (s && s->cost_irq[i]) {
            qemu_irq_lower(s->cost_irq[i]);
        }
    }
}

static void a1_cost_fire(void *opaque)
{
    A1CostCtx *ctx = opaque;
    IngenicA1State *s = ctx->soc;
    int id = ctx->cpu_id;
    s->cost_state[id].flag = 1;
    if (!s->cost_state[id].mask && s->cost_irq[id]) {
        qemu_irq_raise(s->cost_irq[id]);
    }
}

static void a1_cost_arm(IngenicA1State *s, int id)
{
    if (!s->cost_state[id].enabled || !s->cost_state[id].dfr) {
        timer_del(s->cost_timer[id]);
        return;
    }
    int64_t period_ns = (int64_t)s->cost_state[id].dfr *
                        NANOSECONDS_PER_SECOND / COST_FREQ;
    int64_t fire = s->cost_state[id].base_ns + period_ns;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (fire <= now) {
        fire = now + 1;
    }
    timer_mod(s->cost_timer[id], fire);
}

static uint64_t ingenic_a1_cost_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicA1State *s = opaque;
    int id = (offset >> 8) & 1;

    switch (offset & 0xFF) {
    case COST_OSTER:
        return s->cost_state[id].enabled;
    case COST_OSTFR:
        return s->cost_state[id].flag;
    case COST_OSTMR:
        return s->cost_state[id].mask;
    case COST_OSTDFR:
        return s->cost_state[id].dfr;
    case COST_OSTCNT: {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t elapsed = now - s->cost_state[id].base_ns;
        return (uint32_t)(elapsed * COST_FREQ / NANOSECONDS_PER_SECOND);
    }
    default:
        return 0;
    }
}

static void ingenic_a1_cost_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicA1State *s = opaque;
    int id = (offset >> 8) & 1;

    switch (offset & 0xFF) {
    case COST_OSTCCR:
        break;
    case COST_OSTER:
        if ((value & 1) && !s->cost_state[id].enabled) {
            s->cost_state[id].base_ns =
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        }
        s->cost_state[id].enabled = (uint32_t)value & 1;
        a1_cost_arm(s, id);
        break;
    case COST_OSTCR:
        if (value & 1) {
            s->cost_state[id].base_ns =
                qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            a1_cost_arm(s, id);
        }
        break;
    case COST_OSTFR:
        if (!(value & 1)) {
            s->cost_state[id].flag = 0;
            if (s->cost_irq[id]) {
                qemu_irq_lower(s->cost_irq[id]);
            }
        }
        s->cost_state[id].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        a1_cost_arm(s, id);
        break;
    case COST_OSTMR:
        s->cost_state[id].mask = (uint32_t)value & 1;
        /*
         * A fire during a masked window only sets the flag. Deliver it
         * on unmask, or the guest's oneshot clockevent chain breaks and
         * that CPU sleeps forever in wait-irqoff.
         */
        if (!s->cost_state[id].mask && s->cost_state[id].flag &&
            s->cost_irq[id]) {
            qemu_irq_raise(s->cost_irq[id]);
        }
        break;
    case COST_OSTDFR:
        s->cost_state[id].dfr = (uint32_t)value;
        s->cost_state[id].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        a1_cost_arm(s, id);
        break;
    }
}

static const MemoryRegionOps ingenic_a1_cost_ops = {
    .read = ingenic_a1_cost_read,
    .write = ingenic_a1_cost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * eFuse, as an A1N reads it with the read protocol driven from U-Boot:
 * clear the state at 0x008, latch the word index in bits [26:21] of
 * 0x000 and set bit 0, wait for bit 0 of 0x008, read 0x00c. Word n is
 * the one mirrored at 0x200 + 4n. The grade is SUBSOCTYPE2 at 0x250, not
 * SUBSOCTYPE1 at 0x238 as on the T31.
 */
#define A1_EFUSE_CTRL        0x000
#define A1_EFUSE_STATE       0x008
#define A1_EFUSE_DATA        0x00C
#define A1_EFUSE_SERIAL0     0x200
#define A1_EFUSE_SERIAL1     0x204
#define A1_EFUSE_SERIAL2     0x208
#define A1_EFUSE_SECURITY    0x224
#define A1_EFUSE_BOOTCFG     0x22C
#define A1_EFUSE_CHIPID3     0x23C
#define A1_EFUSE_SUBSOCTYPE1 0x238
#define A1_EFUSE_SUBSOCTYPE2 0x250

static uint32_t ingenic_a1_efuse_word(IngenicA1State *s, hwaddr offset)
{
    if (s->efuse_dump_words && offset >= 0x200 && !(offset & 3) &&
        (offset - 0x200) / 4 < s->efuse_dump_words) {
        return s->efuse_dump[(offset - 0x200) / 4];
    }
    switch (offset) {
    case A1_EFUSE_SERIAL0:
    case A1_EFUSE_SERIAL1:
    case A1_EFUSE_SERIAL2:
        return s->efuse_chipid[(offset - A1_EFUSE_SERIAL0) / 4];
    case A1_EFUSE_SERIAL0 + 12:     /* words 3-5 repeat the chip ID */
    case A1_EFUSE_SERIAL1 + 12:
    case A1_EFUSE_SERIAL2 + 12:
        return s->efuse_chipid[(offset - A1_EFUSE_SERIAL0 - 12) / 4];
    case 0x218:
    case 0x21C:
        return 0x00000001;
    case A1_EFUSE_CHIPID3:
        return 0x00000000;
    case A1_EFUSE_SUBSOCTYPE1:
        return 0x00000000;
    case A1_EFUSE_SUBSOCTYPE2:
        return s->efuse_subsoctype2;
    case A1_EFUSE_SECURITY:
        return s->efuse_security;
    case A1_EFUSE_BOOTCFG:
        return s->efuse_bootcfg;
    default:
        return 0;
    }
}

static uint64_t ingenic_a1_efuse_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicA1State *s = opaque;

    switch (offset) {
    case A1_EFUSE_CTRL:
        return s->efuse_ctrl;
    case A1_EFUSE_STATE:
        /* Bits 15:8 show the top byte of the security word */
        return s->efuse_state | (s->efuse_security >> 16 & 0xff00);
    case A1_EFUSE_DATA:
        return s->efuse_data;
    default:
        return ingenic_a1_efuse_word(s, offset);
    }
}

static void ingenic_a1_efuse_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicA1State *s = opaque;

    switch (offset) {
    case A1_EFUSE_CTRL:
        /* Bit 0 starts a read, done at once; bit 1 would blow fuses */
        if (value & 2) {
            qemu_log_mask(LOG_UNIMP, "%s: fuse programming\n", __func__);
        }
        s->efuse_ctrl = value & 0x07e00000;
        if (value & 1) {
            s->efuse_data = ingenic_a1_efuse_word(s, 0x200 + 4 *
                                                  extract32(value, 21, 6));
            s->efuse_state |= 1;
        }
        break;
    case A1_EFUSE_STATE:
        /* Writing 0 clears done; nothing sets it but a read */
        s->efuse_state &= value;
        break;
    }
}

static const MemoryRegionOps ingenic_a1_efuse_ops = {
    .read = ingenic_a1_efuse_read,
    .write = ingenic_a1_efuse_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

/*
 * A1 variant table. PLL encoding matches T31 convention:
 *   (nf << 20) | (nr << 14) | (od1 << 11) | (od0 << 8)
 * A1 has 4 PLLs (APLL, MPLL, EPLL, VPLL) but we only store APLL/MPLL
 * as defaults - EPLL/VPLL are set in realize.
 */
#define PLL_ON_EN  ((1 << 3) | (1 << 0))
#define MNOD(nf, nr, od1, od0)  (((nf) << 20) | ((nr) << 14) | \
                                 ((od1) << 11) | ((od0) << 8) | PLL_ON_EN)

typedef struct {
    const char *name;
    uint32_t cpuid;
    uint32_t type2;      /* EFUSE SUBSOCTYPE2 (upper 16 bits = variant) */
    uint32_t apll;
    uint32_t mpll;
    uint32_t ram_mb;
} A1Variant;

static const A1Variant a1_variants[] = {
    /*
     * A1 family: socId = (cpuid >> 12) & 0xFFFF = 0x001; cpuid and type2
     * as measured on an A1N. The vendor libimp (1.7.0) names 0x1111 A1-N,
     * 0x2222 A1-X, 0x3333 A1-L and 0x5555 A1-NT; it has no name for 0x4444.
     */
    { "a1n", 0x50001002, 0x11112222, MNOD(92, 1, 2, 1), MNOD(134, 1, 2, 1),
      256 },
    { "a1l", 0x50001002, 0x33330000, MNOD(92, 1, 2, 1), MNOD(134, 1, 2, 1),
      128 },
    { "a1x", 0x50001002, 0x22220000, MNOD(92, 1, 2, 1), MNOD(117, 1, 2, 1),
      512 },
    { "a1a", 0x50001002, 0x44440000, MNOD(100, 1, 2, 1), MNOD(134, 1, 2, 1),
      1024 },
    { "a1nt", 0x50001002, 0x55550000, MNOD(92, 1, 2, 1), MNOD(117, 1, 2, 1),
      256 },
    /* Default */
    { "qemu", 0x50001002, 0xEE000000, MNOD(92, 1, 2, 1), MNOD(134, 1, 2, 1),
      256 },
    { NULL, 0, 0, 0, 0, 0 }
};

static const A1Variant *ingenic_a1_find_variant(const char *name)
{
    if (name && name[0]) {
        for (int i = 0; a1_variants[i].name; i++) {
            if (g_ascii_strcasecmp(name, a1_variants[i].name) == 0) {
                return &a1_variants[i];
            }
        }
    }
    return &a1_variants[ARRAY_SIZE(a1_variants) - 2];
}

void ingenic_a1_check_ram(IngenicA1State *s, uint64_t ram_size)
{
    const A1Variant *v = ingenic_a1_find_variant(s->soc_variant);

    if (ram_size < (uint64_t)v->ram_mb * MiB) {
        warn_report("the %s has %" PRIu32 " MiB of RAM but -m gives %" PRIu64
                    " MiB: firmware built for it may not start",
                    v->name, v->ram_mb, ram_size / MiB);
    }
}

Clock *ingenic_a1_cpu_clock(IngenicA1State *s)
{
    const A1Variant *v = ingenic_a1_find_variant(s->soc_variant);
    IngenicA1CpmState *cpm = &s->cpm;

    /* The bootrom runs from EXTCLK, CPCCR still 0 */
    if (!MACHINE(qdev_get_machine())->kernel_filename ||
        MACHINE(qdev_get_machine())->firmware) {
        /*
         * Every PLL off, as an A1N's bootrom finds those it leaves alone and
         * leaves them for an SPL from flash
         */
        for (int i = 0x10; i <= 0x1c; i += 4) {
            cpm->regs[i / 4] = 0x010049c0;
        }
        cpm->regs[0x3c / 4] = 0x40000000;               /* DDRCDR */
    } else {
        /* As the vendor U-Boot leaves it; EPLL 1500 MHz, VPLL 1200 MHz */
        cpm->regs[0x00 / 4] = 0x9a7f7520;
        cpm->regs[0x10 / 4] = v->apll;
        cpm->regs[0x14 / 4] = v->mpll;
        cpm->regs[0x18 / 4] = MNOD(125, 1, 2, 1) | PLL_ON_EN;
        cpm->regs[0x1c / 4] = MNOD(100, 1, 2, 1) | PLL_ON_EN;
        cpm->regs[0x30 / 4] = 0xe0b00500;               /* CLKGR0 */
        cpm->regs[0x38 / 4] = 0x0000daff;               /* CLKGR1 */
    }
    memcpy(cpm->boot_regs, cpm->regs, sizeof(cpm->regs));
    cpm->boot_saved = true;
    return ingenic_a1_cpm_cpu_clock(cpm);
}

/* Physical memory map */
const hwaddr ingenic_a1_memmap[] = {
    /* APB bus */
    [INGENIC_A1_DEV_CPM]        = 0x10000000,
    [INGENIC_A1_DEV_TCU]        = 0x10002000,
    [INGENIC_A1_DEV_RTC]        = 0x10003000,
    [INGENIC_A1_DEV_GPIO]       = 0x10010000,
    [INGENIC_A1_DEV_AIC0]       = 0x10020000,
    [INGENIC_A1_DEV_AIC1]       = 0x10021000,
    [INGENIC_A1_DEV_CODEC]      = 0x10022000,
    [INGENIC_A1_DEV_UART0]      = 0x10030000,
    [INGENIC_A1_DEV_UART1]      = 0x10031000,
    [INGENIC_A1_DEV_UART2]      = 0x10032000,
    [INGENIC_A1_DEV_SSI0]       = 0x10043000,
    [INGENIC_A1_DEV_SSI1]       = 0x10044000,
    [INGENIC_A1_DEV_I2C0]       = 0x10050000,
    [INGENIC_A1_DEV_I2C1]       = 0x10051000,
    [INGENIC_A1_DEV_USBPHY]     = 0x10060000,
    [INGENIC_A1_DEV_DES]        = 0x10061000,
    [INGENIC_A1_DEV_DTRNG]      = 0x10072000,
    [INGENIC_A1_DEV_HDMI_PHY]   = 0x10075000,
    [INGENIC_A1_DEV_VDAC]       = 0x10076000,
    [INGENIC_A1_DEV_SATA_PHY0]  = 0x10080000,
    [INGENIC_A1_DEV_SATA_PHY1]  = 0x10090000,

    /* Core region */
    [INGENIC_A1_DEV_G_OST]      = 0x12000000,
    [INGENIC_A1_DEV_N_OST]      = 0x12100000,
    [INGENIC_A1_DEV_CCU]        = 0x12200000,
    [INGENIC_A1_DEV_INTCN]      = 0x12300000,
    [INGENIC_A1_DEV_SRAM]       = 0x12400000,

    /* AHB0 bus */
    [INGENIC_A1_DEV_HARB0]      = 0x13000000,
    [INGENIC_A1_DEV_DDR_PHY]    = 0x13011000,
    [INGENIC_A1_DEV_MSC0]       = 0x13060000,
    [INGENIC_A1_DEV_MSC1]       = 0x13070000,
    [INGENIC_A1_DEV_IPU]        = 0x13080000,
    [INGENIC_A1_DEV_AIP]        = 0x13090000,
    [INGENIC_A1_DEV_MONITOR]    = 0x130a0000,
    [INGENIC_A1_DEV_GMAC0]      = 0x130b0000,
    [INGENIC_A1_DEV_GMAC1]      = 0x130c0000,
    [INGENIC_A1_DEV_SATA]       = 0x130d0000,
    [INGENIC_A1_DEV_VO]         = 0x130e0000,
    [INGENIC_A1_DEV_DDRC]       = 0x134f0000,
    [INGENIC_A1_DEV_VDE]        = 0x13300000,
    [INGENIC_A1_DEV_HDMI]       = 0x13380000,

    /* AHB1 bus (video) */
    [INGENIC_A1_DEV_VC8000D]    = 0x13100000,
    [INGENIC_A1_DEV_JPEG]       = 0x13200000,

    /* AHB2 bus */
    [INGENIC_A1_DEV_HARB2]      = 0x13400000,
    [INGENIC_A1_DEV_NEMC]       = 0x13410000,
    [INGENIC_A1_DEV_PDMA]       = 0x13420000,
    [INGENIC_A1_DEV_AES]        = 0x13430000,
    [INGENIC_A1_DEV_SFC0]       = 0x13440000,
    [INGENIC_A1_DEV_SFC1]       = 0x13450000,
    [INGENIC_A1_DEV_PWM]        = 0x13460000,
    [INGENIC_A1_DEV_HASH]       = 0x13480000,
    [INGENIC_A1_DEV_RSA]        = 0x134c0000,
    [INGENIC_A1_DEV_EFUSE]      = 0x13540000,
    [INGENIC_A1_DEV_OTG0]       = 0x13600000,
    [INGENIC_A1_DEV_OTG1]       = 0x13640000,
    [INGENIC_A1_DEV_OTG2]       = 0x13680000,

    /* Memory */
    [INGENIC_A1_DEV_SDRAM]      = 0x00000000,
};

static const struct {
    const char *name;
    hwaddr base;
    hwaddr size;
} ingenic_a1_unimp[] = {
    { "ingenic-a1-vdac",      0x10076000, 4 * KiB },
    /* Core OST at 0x12100000 is a real device now */
    { "ingenic-a1-lepccu",    0x12a00000, 64 * KiB },
    /* the DDR APB at 0x13012000 is the DDR PHY model's second page */
    { "ingenic-a1-axi-arb1",  0x13013000, 4 * KiB },
    /* CCU stub at 0x12200000 absorbs SMP boot writes */
    { "ingenic-a1-ipu",       0x13080000, 64 * KiB },
    { "ingenic-a1-aip",       0x13090000, 64 * KiB },
    { "ingenic-a1-vc8000d",   0x13100000, 1024 * KiB },
    { "ingenic-a1-jpeg",      0x13200000, 1024 * KiB },
    { "ingenic-a1-vde",       0x13300000, 512 * KiB },
    { "ingenic-a1-harb2",     0x13400000, 4 * KiB },
    /* HASH and RSA are real device models */
};

static void ingenic_a1_init(Object *obj)
{
    IngenicA1State *s = INGENIC_A1(obj);

    s->memmap = ingenic_a1_memmap;

    object_initialize_child(obj, "cpm", &s->cpm, TYPE_INGENIC_A1_CPM);
    object_initialize_child(obj, "ddrc", &s->ddrc, TYPE_INGENIC_DDRC);
    object_initialize_child(obj, "sfc", &s->sfc, TYPE_INGENIC_A1_SFC);
    object_initialize_child(obj, "sfc1", &s->sfc1, TYPE_INGENIC_A1_SFC);
    object_initialize_child(obj, "gmac0", &s->gmac0, TYPE_INGENIC_A1_XGMAC);
    object_initialize_child(obj, "gmac1", &s->gmac1, TYPE_INGENIC_A1_XGMAC);
    object_initialize_child(obj, "tcu", &s->tcu, TYPE_INGENIC_TCU);
    object_initialize_child(obj, "dtrng", &s->dtrng, TYPE_INGENIC_DTRNG);
    object_initialize_child(obj, "pwm", &s->pwm, TYPE_INGENIC_PWM);
    object_initialize_child(obj, "vo", &s->vo, TYPE_INGENIC_VO);
    object_initialize_child(obj, "hdmi", &s->hdmi, TYPE_INGENIC_A1_HDMI);
    object_initialize_child(obj, "rtc", &s->rtc, TYPE_INGENIC_A1_RTC);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_INGENIC_GPIO);
    object_initialize_child(obj, "intc", &s->intc, TYPE_INGENIC_INTC);
    object_initialize_child(obj, "ccu", &s->ccu, TYPE_INGENIC_XBURST2_CCU);
    object_initialize_child(obj, "l2sram", &s->l2sram,
                            TYPE_INGENIC_XBURST2_SRAM);
    object_initialize_child(obj, "nna", &s->nna, TYPE_INGENIC_NNA);
    object_initialize_child(obj, "riscv", &s->riscv, TYPE_INGENIC_RISCV);
    object_initialize_child(obj, "i2c0", &s->i2c[0], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c1", &s->i2c[1], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "ssi0", &s->ssi[0], TYPE_INGENIC_SSI);
    object_initialize_child(obj, "ssi1", &s->ssi[1], TYPE_INGENIC_SSI);
    object_initialize_child(obj, "aes", &s->aes, TYPE_INGENIC_AES);
    object_initialize_child(obj, "sdhci0", &s->sdhci[0], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "sdhci1", &s->sdhci[1], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "dwc2-0", &s->dwc2[0], TYPE_DWC2_USB);
    object_initialize_child(obj, "dwc2-1", &s->dwc2[1], TYPE_DWC2_USB);
    object_initialize_child(obj, "dwc2-2", &s->dwc2[2], TYPE_DWC2_USB);
    object_initialize_child(obj, "pdma", &s->pdma, TYPE_INGENIC_PDMA);
    object_initialize_child(obj, "aic", &s->aic, TYPE_INGENIC_AIC);
    object_initialize_child(obj, "aic1", &s->aic1, TYPE_INGENIC_AIC);
    object_initialize_child(obj, "des", &s->des, TYPE_INGENIC_DES);
    object_initialize_child(obj, "usbphy", &s->usbphy, TYPE_INGENIC_USBPHY);
    object_initialize_child(obj, "hash", &s->hash, TYPE_INGENIC_HASH);
    object_initialize_child(obj, "rsa", &s->rsa, TYPE_INGENIC_RSA);
    object_initialize_child(obj, "sata", &s->sata, TYPE_SYSBUS_AHCI);
    for (unsigned i = 0; i < 3; i++) {
        object_property_add_const_link(OBJECT(&s->dwc2[i]), "dma-mr",
                                       OBJECT(get_system_memory()));
    }
}

/* MSCR: the L2's share of the SRAM, in each core's Config2 and the window */
static void ingenic_a1_mscr_changed(void *opaque, uint32_t mscr)
{
    IngenicA1State *s = opaque;

    ingenic_xburst2_l2_split(s->cpu, s->num_cpus, &s->l2sram, mscr);
}

static void ingenic_a1_realize(DeviceState *dev, Error **errp)
{
    IngenicA1State *s = INGENIC_A1(dev);
    unsigned i;

    /* The model's chip ID unless given; a dump overrides every word */
    s->efuse_chipid[0] = 0x725C2516;
    s->efuse_chipid[1] = 0x94FE0281;
    s->efuse_chipid[2] = 0x12100080;
    if (s->efuse_chipid_hex &&
        !ingenic_efuse_parse_id(s->efuse_chipid_hex, s->efuse_chipid, errp)) {
        return;
    }
    if (s->efuse_file &&
        !ingenic_efuse_load(s->efuse_file, s->efuse_dump,
                            ARRAY_SIZE(s->efuse_dump), &s->efuse_dump_words,
                            errp)) {
        return;
    }

    {
        const A1Variant *v = ingenic_a1_find_variant(s->soc_variant);
        s->efuse_subsoctype2 = v->type2;
        s->harb0_cpuid = v->cpuid;
        s->variant = v;
        /*
         * As an A1N reads them: the bootrom stays off the UART with bit
         * 14 of the boot config set
         */
        if (!s->efuse_security) {
            s->efuse_security = 0x03030000;
        }
        if (!s->efuse_bootcfg) {
            s->efuse_bootcfg = 0x00004000;
        }
        /* As the bootrom leaves it, after its own reads */
        s->efuse_state = 1;
    }

    /* CPM */
    sysbus_realize(SYS_BUS_DEVICE(&s->cpm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->cpm), 0,
                    s->memmap[INGENIC_A1_DEV_CPM]);

    /* DDR controller, PHY, the APB page over its second half, DDRC1 */
    qdev_prop_set_string(DEVICE(&s->ddrc), "apb", "a1");
    sysbus_realize(SYS_BUS_DEVICE(&s->ddrc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 0,
                    s->memmap[INGENIC_A1_DEV_DDRC]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 1,
                    s->memmap[INGENIC_A1_DEV_DDR_PHY]);
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->ddrc), 2, 0x13012000, 1);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 3, 0x13010000);

    /* SFC0: IRQ -> INTC source 7 */
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc), 0,
                    s->memmap[INGENIC_A1_DEV_SFC0]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 7));

    /* SFC1: the flash given as -drive if=mtd,index=1; IRQ -> source 6 */
    qdev_prop_set_uint8(DEVICE(&s->sfc1), "unit", 1);
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc1), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc1), 0,
                    s->memmap[INGENIC_A1_DEV_SFC1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc1), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 6));

    /* GMAC0 - XGMAC (dwxgmac2) at 0x130B0000, IRQ -> INTC source 56 */
    if (!qemu_configure_nic_device(DEVICE(&s->gmac0), true, NULL)) {
        NetClientState *nc = qemu_find_netdev("n0");
        if (nc) {
            qdev_prop_set_netdev(DEVICE(&s->gmac0), "netdev", nc);
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gmac0), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gmac0), 0,
                    s->memmap[INGENIC_A1_DEV_GMAC0]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gmac0), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 56));

    /* GMAC1 - XGMAC (dwxgmac2) at 0x130C0000, IRQ -> INTC source 55 */
    if (!qemu_configure_nic_device(DEVICE(&s->gmac1), true, NULL)) {
        NetClientState *nc = qemu_find_netdev("n1");
        if (nc) {
            qdev_prop_set_netdev(DEVICE(&s->gmac1), "netdev", nc);
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gmac1), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gmac1), 0,
                    s->memmap[INGENIC_A1_DEV_GMAC1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gmac1), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 55));

    /* USB OTG0/1/2 (DWC2). IRQ -> INTC sources 21, 28, 29. */
    {
        static const int otg_dev[3] = {
            INGENIC_A1_DEV_OTG0, INGENIC_A1_DEV_OTG1, INGENIC_A1_DEV_OTG2,
        };
        static const int otg_irq[3] = { 21, 28, 29 };
        for (i = 0; i < 3; i++) {
            ingenic_dwc2_config(OBJECT(&s->dwc2[i]),
                                i == 1 ? INGENIC_DWC2_GHWCFG3_T32 :
                                INGENIC_DWC2_GHWCFG3_A1_0);
            sysbus_realize(SYS_BUS_DEVICE(&s->dwc2[i]), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->dwc2[i]), 0,
                            s->memmap[otg_dev[i]]);
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->dwc2[i]), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), otg_irq[i]));
        }
    }
    /* usb-role sets the OTG0 ID through bit 23 of CPM 0x40 (SOFTAPP) */
    qdev_connect_gpio_out_named(DEVICE(&s->cpm), "otg-id-change", 0,
                                qdev_get_gpio_in_named(DEVICE(&s->dwc2[0]),
                                                       "otg-id-change", 0));

    /* PDMA at 0x13420000, IRQ -> INTC source 10 */
    sysbus_realize(SYS_BUS_DEVICE(&s->pdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pdma), 0,
                    s->memmap[INGENIC_A1_DEV_PDMA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pdma), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 10));

    /*
     * AIC0 (INTC source 1) and the internal codec at 0x10022000, clocked
     * by the CPM's I2S0 dividers. DMA requests as the PDMA numbers them:
     * TX 0x06, RX 0x07, the TFIFO loop 0x05.
     */
    qdev_prop_set_string(DEVICE(&s->aic), "codec", "a1");
    object_property_set_link(OBJECT(&s->aic), "pdma", OBJECT(&s->pdma),
                             &error_abort);
    qdev_connect_clock_in(DEVICE(&s->aic), "i2s-tx",
                          qdev_get_clock_out(DEVICE(&s->cpm), "i2s-tx"));
    qdev_connect_clock_in(DEVICE(&s->aic), "i2s-rx",
                          qdev_get_clock_out(DEVICE(&s->cpm), "i2s-rx"));
    sysbus_realize(SYS_BUS_DEVICE(&s->aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 0,
                    s->memmap[INGENIC_A1_DEV_AIC0]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 1,
                    s->memmap[INGENIC_A1_DEV_CODEC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->aic), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 1));
    {
        static const uint8_t aic_drt[INGENIC_AIC_NR_DREQ] = {
            [INGENIC_AIC_DREQ_TX] = 0x06,
            [INGENIC_AIC_DREQ_RX] = 0x07,
            [INGENIC_AIC_DREQ_LOOP] = 0x05,
        };

        for (i = 0; i < INGENIC_AIC_NR_DREQ; i++) {
            qdev_connect_gpio_out_named(DEVICE(&s->aic), "dreq", i,
                qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq", aic_drt[i]));
        }
    }

    /*
     * AIC1 at 0x10021000, which an A1N reads with AIC0's resets: INTC
     * source 0 and DMA request 0x04, its TX (the PDMA has none for its
     * record side); no codec of its own, and no CPM clock in the model
     */
    qdev_prop_set_string(DEVICE(&s->aic1), "codec", "a1");
    object_property_set_link(OBJECT(&s->aic1), "pdma", OBJECT(&s->pdma),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->aic1), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic1), 0,
                    s->memmap[INGENIC_A1_DEV_AIC1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->aic1), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 0));
    qdev_connect_gpio_out_named(DEVICE(&s->aic1), "dreq", INGENIC_AIC_DREQ_TX,
                                qdev_get_gpio_in_named(DEVICE(&s->pdma),
                                                       "dreq", 0x04));

    /*
     * The DES at 0x10061000, its base.h's DES_IOBASE (the device tree's
     * reg, 0x10043000, is SSI0's), DMA requests 0x2e and 0x2f
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->des), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->des), 0, s->memmap[INGENIC_A1_DEV_DES]);
    /* The bus monitor and the second AXI arbiter (the first reads 0) */
    ingenic_busregs_create("monitor-a1", 0x130a0000, 0);
    ingenic_busregs_create("axiarb2-t40", 0x13014000, 0);
    ingenic_nemc_create(s->memmap[INGENIC_A1_DEV_NEMC]);
    qdev_prop_set_string(DEVICE(&s->usbphy), "model", "a1");
    sysbus_realize(SYS_BUS_DEVICE(&s->usbphy), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->usbphy), 0,
                    s->memmap[INGENIC_A1_DEV_USBPHY]);
    for (i = 0; i < 2; i++) {
        qdev_connect_gpio_out_named(DEVICE(&s->des), "dreq", i,
            qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq", 0x2e + i));
    }

    /* HASH engine, IRQ -> INTC source 22 */
    sysbus_realize(SYS_BUS_DEVICE(&s->hash), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hash), 0,
                    s->memmap[INGENIC_A1_DEV_HASH]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->hash), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 22));

    /* RSA accelerator */
    sysbus_realize(SYS_BUS_DEVICE(&s->rsa), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rsa), 0,
                    s->memmap[INGENIC_A1_DEV_RSA]);

    /*
     * TCU - 8 timer/PWM channels + the global registers + the embedded
     * OST (U-Boot delay loops), all on one page. The watchdog overlaps
     * the low 16 bytes at higher priority. IRQ -> INTC source 27.
     */
    qdev_prop_set_uint32(DEVICE(&s->tcu), "flags", 0x01ff80ff);  /* an A1N's */
    qdev_prop_set_uint32(DEVICE(&s->tcu), "tdfr-reset", 0);
    sysbus_realize(SYS_BUS_DEVICE(&s->tcu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->tcu), 0,
                    s->memmap[INGENIC_A1_DEV_TCU]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->tcu), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 27));

    /* DTRNG - true random number generator, IRQ -> INTC source 34 */
    sysbus_realize(SYS_BUS_DEVICE(&s->dtrng), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dtrng), 0,
                    s->memmap[INGENIC_A1_DEV_DTRNG]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dtrng), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 34));

    /* The BT output (vo_bt), IRQ -> INTC source 30 (IRQ_VO) */
    qdev_connect_clock_in(DEVICE(&s->vo), "clk",
                          qdev_get_clock_out(DEVICE(&s->cpm), "vo-clk"));
    sysbus_realize(SYS_BUS_DEVICE(&s->vo), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->vo), 0, s->memmap[INGENIC_A1_DEV_VO]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->vo), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 30));

    /*
     * The HDMI controller (IRQ 18) with its PHY; the monitor's HPD is
     * PB31, as the SDK's driver reads it
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->hdmi), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hdmi), 0,
                    s->memmap[INGENIC_A1_DEV_HDMI]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hdmi), 1,
                    s->memmap[INGENIC_A1_DEV_HDMI_PHY]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->hdmi), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 18));

    /* PWM controller at 0x13460000, IRQ -> INTC source 31 */
    qdev_connect_clock_in(DEVICE(&s->pwm), "clk",
                          qdev_get_clock_out(DEVICE(&s->cpm), "pwm-clk"));
    sysbus_realize(SYS_BUS_DEVICE(&s->pwm), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pwm), 0,
                    s->memmap[INGENIC_A1_DEV_PWM]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pwm), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 31));

    /* The A1's own RTC at 0x10003000; its kernel driver takes no interrupt */
    sysbus_realize(SYS_BUS_DEVICE(&s->rtc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0,
                    s->memmap[INGENIC_A1_DEV_RTC]);

    /*
     * Global OST at 0x12000000 - 64-bit free-running counter used by
     * both U-Boot delays and the kernel clocksource. The A1 register
     * layout (G_OSTCNTH at 0x0C, G_OSTCNTL at 0x10) differs from the
     * XBurst1 sysost, so we use a dedicated inline model.
     */
    memory_region_init_io(&s->gost, OBJECT(dev), &ingenic_a1_gost_ops,
                          s, "ingenic-a1-gost", 0x20);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_G_OST], &s->gost);
    s->gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->gost_state.enabled = 1;
    qemu_register_reset(ingenic_a1_gost_reset, s);
    qemu_register_reset(a1_cost_reset, s);

    /* CCU - XBurst2 SMP control unit */
    qdev_prop_set_uint32(DEVICE(&s->ccu), "num-cpus", s->num_cpus);
    /*
     * As an A1N reads it: its RISC-V core held; 512 KiB of L2 and bit 4,
     * which Ingenic's A1 SPL also sets, leaving the split
     */
    qdev_prop_set_uint32(DEVICE(&s->ccu), "cfcr-reset", 0x80000078);
    qdev_prop_set_uint32(DEVICE(&s->ccu), "mscr-reset", 0x02000c10);
    s->ccu.mscr_changed = ingenic_a1_mscr_changed;
    s->ccu.mscr_opaque = s;
    sysbus_realize(SYS_BUS_DEVICE(&s->ccu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccu), 0,
                    s->memmap[INGENIC_A1_DEV_CCU]);

    /*
     * The 1 MiB SRAM the L2 shares with the NNA, and the NNDMA, which keeps
     * the T40's descriptor bits; its NN instructions are not modelled here
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->l2sram), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->l2sram), 0, INGENIC_XBURST2_SRAM_BASE);
    object_property_set_link(OBJECT(&s->nna), "sram", OBJECT(&s->l2sram),
                             &error_abort);
    qdev_prop_set_uint64(DEVICE(&s->nna), "desc-mask", 0xfff03fffffffffc7ULL);
    qdev_prop_set_bit(DEVICE(&s->nna), "t40", true);
    sysbus_realize(SYS_BUS_DEVICE(&s->nna), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->nna), 0, 0x12500000);

    /*
     * Core OST at 0x12100000 - per-CPU clockevent timer
     * (CPU0 at +0x000, CPU1 at +0x100). IRQ -> MIPS IP4.
     */
    for (i = 0; i < A1_MAX_CPUS; i++) {
        a1_cost_ctx[i].soc = s;
        a1_cost_ctx[i].cpu_id = i;
        s->cost_timer[i] = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        a1_cost_fire, &a1_cost_ctx[i]);
    }
    memory_region_init_io(&s->cost, OBJECT(dev), &ingenic_a1_cost_ops,
                          s, "ingenic-a1-cost", 0x200);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_N_OST], &s->cost);

    /* GPIO controller: 5 ports with 0x1000 stride */
    qdev_prop_set_uint32(DEVICE(&s->gpio), "port-stride", 0x1000);
    /* GPIO: five ports 0x1000 apart; D has no interrupt, E takes 14 */
    qdev_prop_set_uint32(DEVICE(&s->gpio), "nr-ports", 5);
    qdev_prop_set_string(DEVICE(&s->gpio), "family", "a1");
    sysbus_realize(SYS_BUS_DEVICE(&s->gpio), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0,
                    s->memmap[INGENIC_A1_DEV_GPIO]);
    qdev_connect_gpio_out_named(DEVICE(&s->hdmi), "hpd", 0,
                                qdev_get_gpio_in_named(DEVICE(&s->gpio),
                                                       "pull", 32 + 31));

    /*
     * INTC at 0x12300000: 64 sources, one register window per core; the
     * APB window at 0x10001000
     */
    qdev_prop_set_uint32(DEVICE(&s->intc), "num-cpus", s->num_cpus);
    qdev_prop_set_bit(DEVICE(&s->intc), "apb-window", true);
    sysbus_realize(SYS_BUS_DEVICE(&s->intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0,
                    s->memmap[INGENIC_A1_DEV_INTCN]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 1, 0x10001000);

    /* The RISC-V core: its block, released by the CCU's CFCR, IRQ 2 */
    qdev_prop_set_string(DEVICE(&s->riscv), "model", "a1");
    sysbus_realize(SYS_BUS_DEVICE(&s->riscv), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->riscv), 1, 0x12a00000);
    qdev_connect_gpio_out_named(DEVICE(&s->ccu), "riscv-hold", 0,
        qdev_get_gpio_in_named(DEVICE(&s->riscv), "hold", 0));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->riscv), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 2));
    {
        static const int gpio_irq[] = { 17, 16, 15, -1, 14 };

        for (unsigned n = 0; n < ARRAY_SIZE(gpio_irq); n++) {
            if (gpio_irq[n] >= 0) {
                sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), n,
                                   qdev_get_gpio_in(DEVICE(&s->intc),
                                                    gpio_irq[n]));
            }
        }
    }

    /* I2C0/I2C1: auto-NACK. I2C0 = IRQ 60, I2C1 = IRQ 59 */
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[0]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[0]), 0,
                    s->memmap[INGENIC_A1_DEV_I2C0]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[0]), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 60));
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[1]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[1]), 0,
                    s->memmap[INGENIC_A1_DEV_I2C1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[1]), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 59));

    /* AES = IRQ 23 */
    sysbus_realize(SYS_BUS_DEVICE(&s->aes), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aes), 0, s->memmap[INGENIC_A1_DEV_AES]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->aes), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 23));

    /* SSI0 = IRQ 9, SSI1 = IRQ 8 */
    for (i = 0; i < 2; i++) {
        DeviceState *d = DEVICE(&s->ssi[i]);

        qdev_prop_set_string(d, "bus-name", i ? "ssi1" : "ssi0");
        qdev_prop_set_bit(d, "dreq-status", true);
        qdev_prop_set_bit(d, "spcnt", true);
        sysbus_realize(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0,
                        s->memmap[INGENIC_A1_DEV_SSI0] + i * 0x1000);
        sysbus_connect_irq(SYS_BUS_DEVICE(d), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 9 - i));
    }

    /*
     * MSC0/MSC1: standard SDHCI blocks, which the bootrom drives too;
     * IRQ -> INTC source 37 (MSC0), 36 (MSC1)
     */
    for (unsigned j = 0; j < 2; j++) {
        object_property_set_uint(OBJECT(&s->sdhci[j]),
                                 "sd-spec-version", 3, &error_abort);
        /* The A1 kernel reads write-protect inverted, as the T41's */
        object_property_set_bool(OBJECT(&s->sdhci[j]),
                                 "wp-inverted", true, &error_abort);
        /*
         * No pin carries the controller's card-detect input: both slots
         * read a card from power-on, as an A1N, T41LQ and T40XP do, and
         * only the board's card-detect GPIO follows the card
         */
        object_property_set_bool(OBJECT(&s->sdhci[j]),
                                 "cd-tied", true, &error_abort);
        /*
         * As the parts read them: a DesignWare MSHC of SDHCI 4.20, with
         * SDMA, ADMA2 and the UHS modes. The kernels replace the caps
         */
        object_property_set_uint(OBJECT(&s->sdhci[j]), "capareg",
                                 0x080020772769fa98ULL, &error_abort);
        object_property_set_uint(OBJECT(&s->sdhci[j]), "preset-lo",
                                 0x00040002000401f4ULL, &error_abort);
        object_property_set_uint(OBJECT(&s->sdhci[j]), "preset-hi",
                                 0x0002000000010002ULL, &error_abort);
        object_property_set_uint(OBJECT(&s->sdhci[j]), "host-version",
                                 0x0005, &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->sdhci[j]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[j]), 0,
                        s->memmap[INGENIC_A1_DEV_MSC0 + j]);
        /* Its DesignWare MSHC areas */
        ingenic_busregs_create("mshc-ptr",
                               s->memmap[INGENIC_A1_DEV_MSC0 + j] + 0xe0, 0);
        ingenic_busregs_create("mshc",
                               s->memmap[INGENIC_A1_DEV_MSC0 + j] + 0x100, 0);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdhci[j]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 37 - j));
    }

    /*
     * SATA - AHCI controller at 0x130D0000, IRQ -> INTC source 43.
     * Two ports (DTS ports-implemented = 0x3).
     */
    qdev_prop_set_uint32(DEVICE(&s->sata), "num-ports", 2);
    sysbus_realize(SYS_BUS_DEVICE(&s->sata), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sata), 0,
                    s->memmap[INGENIC_A1_DEV_SATA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sata), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 43));

    /*
     * SATA Innophy PHY blocks - analog tuning registers with no side
     * effects. The ahci_ingenic driver does read-modify-write with no
     * polling, so plain storage is a faithful model.
     */
    memory_region_init_ram(&s->sata_phy0, OBJECT(dev),
                           "ingenic-a1.sata-phy0", 64 * KiB, &error_abort);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_SATA_PHY0],
                                &s->sata_phy0);
    memory_region_init_ram(&s->sata_phy1, OBJECT(dev),
                           "ingenic-a1.sata-phy1", 64 * KiB, &error_abort);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_SATA_PHY1],
                                &s->sata_phy1);

    /* SRAM at 0x12400000 */
    memory_region_init_ram(&s->sram, OBJECT(dev), "ingenic-a1.sram",
                           INGENIC_A1_SRAM_SIZE, &error_abort);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_SRAM], &s->sram);

    /* As an A1N reads it, repeating every 32 KiB over NEMC bank 0 */
    if (!ingenic_bootrom_init(&s->bootrom, OBJECT(dev), "ingenic-a1.bootrom",
                              32 * KiB, true, errp)) {
        return;
    }

    /* UARTs. A1 default console is UART1. IRQ: UART0=51, UART1=50, UART2=49 */
    for (i = 0; i < INGENIC_A1_CPM_UARTS; i++) {
        g_autofree char *name = g_strdup_printf("ingenic-a1-uart%d", i);
        DeviceState *uart = ingenic_uart_create(get_system_memory(),
                            s->memmap[INGENIC_A1_DEV_UART0 + i],
                            qdev_get_gpio_in(DEVICE(&s->intc), 51 - i),
                            serial_hd(i), name, DEVICE(&s->pdma), i);

        qdev_connect_gpio_out_named(DEVICE(&s->cpm), "uart-clock", i,
                                    qdev_get_gpio_in_named(uart, "clock", 0));
    }

    /* HARB0 - SoC ID */
    memory_region_init_io(&s->harb0, OBJECT(dev), &ingenic_a1_harb0_ops,
                          s, "ingenic-a1-harb0", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_HARB0], &s->harb0);

    /* EFUSE - variant identification */
    memory_region_init_io(&s->efuse, OBJECT(dev), &ingenic_a1_efuse_ops,
                          s, "ingenic-a1-efuse", 4 * KiB);
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_A1_DEV_EFUSE], &s->efuse);

    /*
     * DMA requests as the PDMA numbers them: SSI0 TX/RX 0x16/0x17, SSI1
     * 0x18/0x19, I2Cn 0x24 + 2n/0x25 + 2n. The PDMA tells each SSI when a
     * channel serves it.
     */
    for (i = 0; i < ARRAY_SIZE(s->i2c); i++) {
        for (int d = 0; d < INGENIC_I2C_NR_DREQ; d++) {
            qdev_connect_gpio_out_named(DEVICE(&s->i2c[i]), "dreq", d,
                qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq",
                                       0x24 + 2 * i + d));
        }
    }
    for (i = 0; i < 2; i++) {
        for (int d = 0; d < 2; d++) {
            qdev_connect_gpio_out_named(DEVICE(&s->ssi[i]), "dreq", d,
                qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq",
                                       0x16 + 2 * i + d));
            qdev_connect_gpio_out_named(DEVICE(&s->pdma), "dack",
                                        0x16 + 2 * i + d,
                qdev_get_gpio_in_named(DEVICE(&s->ssi[i]), "dma", d));
        }
    }

    /* Unimplemented device stubs */
    for (i = 0; i < ARRAY_SIZE(ingenic_a1_unimp); i++) {
        create_unimplemented_device(ingenic_a1_unimp[i].name,
                                    ingenic_a1_unimp[i].base,
                                    ingenic_a1_unimp[i].size);
    }
}

static const Property ingenic_a1_props[] = {
    DEFINE_PROP_STRING("soc-variant", IngenicA1State, soc_variant),
    DEFINE_PROP_UINT32("efuse-security", IngenicA1State, efuse_security, 0),
    DEFINE_PROP_UINT32("efuse-bootcfg", IngenicA1State, efuse_bootcfg, 0),
    DEFINE_PROP_STRING("efuse-chipid", IngenicA1State, efuse_chipid_hex),
    DEFINE_PROP_STRING("efuse-file", IngenicA1State, efuse_file),
};

static const VMStateDescription vmstate_a1_gost = {
    .name = "ingenic-a1/gost",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base_ns, IngenicA1Gost),
        VMSTATE_UINT64(base_count, IngenicA1Gost),
        VMSTATE_UINT32(high_buf, IngenicA1Gost),
        VMSTATE_UINT32(ccr, IngenicA1Gost),
        VMSTATE_UINT32(enabled, IngenicA1Gost),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_a1_cost = {
    .name = "ingenic-a1/cost",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(base_ns, IngenicA1Cost),
        VMSTATE_UINT32(dfr, IngenicA1Cost),
        VMSTATE_UINT32(enabled, IngenicA1Cost),
        VMSTATE_UINT32(flag, IngenicA1Cost),
        VMSTATE_UINT32(mask, IngenicA1Cost),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_a1 = {
    .name = TYPE_INGENIC_A1,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(gost_state, IngenicA1State, 1, vmstate_a1_gost,
                       IngenicA1Gost),
        VMSTATE_STRUCT_ARRAY(cost_state, IngenicA1State, 2, 1, vmstate_a1_cost,
                             IngenicA1Cost),
        VMSTATE_TIMER_PTR_ARRAY(cost_timer, IngenicA1State, 2),
        VMSTATE_UINT32(efuse_ctrl, IngenicA1State),
        VMSTATE_UINT32(efuse_state, IngenicA1State),
        VMSTATE_UINT32(efuse_data, IngenicA1State),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_a1_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = ingenic_a1_realize;
    dc->vmsd = &vmstate_ingenic_a1;
    dc->user_creatable = false;
    device_class_set_props(dc, ingenic_a1_props);
}

static const TypeInfo ingenic_a1_type_info = {
    .name = TYPE_INGENIC_A1,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(IngenicA1State),
    .instance_init = ingenic_a1_init,
    .class_init = ingenic_a1_class_init,
};

static void ingenic_a1_register_types(void)
{
    type_register_static(&ingenic_a1_type_info);
}

type_init(ingenic_a1_register_types)
