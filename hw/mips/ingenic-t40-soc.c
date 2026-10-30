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
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/char/ingenic-uart.h"
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
#include "system/physmem.h"
#include "hw/mips/ingenic-t40.h"
#include "hw/misc/ingenic-cpm-clk.h"


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
    .valid = { .min_access_size = 1, .max_access_size = 4 },
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
 * U-Boot divides by 4 and times its delays at 6 MHz.
 */
#define G_OST_FREQ  24000000ULL

static struct {
    int64_t base_ns;
    uint64_t base_count;    /* the count at base_ns */
    uint32_t high_buf;
    uint32_t ccr;
    uint32_t enabled;
} t40_gost_state;

/* G_OSTCCR divides EXTCLK by 1, 4 or 16 */
static uint64_t t40_gost_count(void)
{
    int64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                      t40_gost_state.base_ns;

    return t40_gost_state.base_count +
           muldiv64(elapsed, G_OST_FREQ >> (2 * (t40_gost_state.ccr & 3)),
                    NANOSECONDS_PER_SECOND);
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
        t40_gost_state.base_count = t40_gost_count();
        t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        t40_gost_state.ccr = (uint32_t)value;
        break;
    case 0x04:
        if ((value & 1) && !t40_gost_state.enabled) {
            t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            t40_gost_state.base_count = 0;
        }
        t40_gost_state.enabled = (uint32_t)value;
        break;
    case 0x08:
        if (value & 1) {
            t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            t40_gost_state.base_count = 0;
        }
        break;
    }
}

static const MemoryRegionOps t40_gost_ops = {
    .read = t40_gost_read,
    .write = t40_gost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void t40_gost_reset(void *opaque)
{
    t40_gost_state.base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    t40_gost_state.base_count = 0;
    t40_gost_state.high_buf = 0;
    t40_gost_state.ccr = 0;
    t40_gost_state.enabled = 1;
}

/*
 * A store to an address nothing decodes completes on a T40, a load does
 * not. The T40N demo SPL (SDK 1.3.1) needs that: its skew setting hands
 * ddr_writel() absolute PHY addresses, which adds the DDRC base again, and
 * with ERL set the stores land at 0x66501400. Measured on a T40N: the SPL
 * carries on, and from U-Boot such a store neither faults nor shows up in
 * DRAM at 0x06501400.
 */
static MemTxResult t40_unmapped_read(void *opaque, hwaddr addr,
                                     uint64_t *data, unsigned size,
                                     MemTxAttrs attrs)
{
    return MEMTX_DECODE_ERROR;
}

static MemTxResult t40_unmapped_write(void *opaque, hwaddr addr,
                                      uint64_t data, unsigned size,
                                      MemTxAttrs attrs)
{
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-t40: write to unmapped 0x%08"
                  HWADDR_PRIx ", dropped\n", addr);
    return MEMTX_OK;
}

static const MemoryRegionOps t40_unmapped_ops = {
    .read_with_attrs = t40_unmapped_read,
    .write_with_attrs = t40_unmapped_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};


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

static void t40_cost_reset(void *opaque)
{
    IngenicT40State *s = opaque;

    for (int i = 0; i < T40_MAX_CPUS; i++) {
        t40_cost_state[i].base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        t40_cost_state[i].dfr = 0;
        t40_cost_state[i].enabled = 0;
        t40_cost_state[i].flag = 0;
        t40_cost_state[i].mask = 0;
        if (s && s->cost_timer[i]) {
            timer_del(s->cost_timer[i]);
        }
        if (s && s->cost_irq[i]) {
            qemu_irq_lower(s->cost_irq[i]);
        }
    }
}

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
        /*
         * A fire during a masked window only sets the flag. Deliver it
         * on unmask, or the guest's oneshot clockevent chain breaks and
         * that CPU sleeps forever in wait-irqoff.
         */
        if (!t40_cost_state[id].mask && t40_cost_state[id].flag &&
            s->cost_irq[id]) {
            qemu_irq_raise(s->cost_irq[id]);
        }
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
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * eFuse, as a T40N, T40XP, T41NQ and T41LQ read with the read protocol
 * driven from U-Boot: clear the state at 0x008, latch the word index in
 * bits [26:21] of 0x000 and set bit 0, wait for bit 0 of 0x008, read
 * 0x00c. Word n is the one mirrored at 0x200 + 4n.
 */
#define T40_EFUSE_CTRL        0x000
#define T40_EFUSE_STATE       0x008
#define T40_EFUSE_DATA        0x00C
#define T40_EFUSE_SERIAL0     0x200
#define T40_EFUSE_SERIAL1     0x204
#define T40_EFUSE_SERIAL2     0x208
#define T40_EFUSE_SECURITY    0x224
#define T40_EFUSE_BOOTCFG     0x22C
#define T40_EFUSE_SUBSOCTYPE1 0x238
#define T40_EFUSE_CHIPID3     0x23C
#define T40_EFUSE_WORD19      0x24C   /* hash of 0x200-0x208; SPLs halt on 0 */
#define T40_EFUSE_SUBSOCTYPE2 0x250

static uint32_t t40_efuse_word(IngenicT40State *s, hwaddr offset)
{
    switch (offset) {
    case T40_EFUSE_SERIAL0:
    case T40_EFUSE_SERIAL0 + 12:    /* words 3-5 repeat the chip ID */
        return 0x40401234;
    case T40_EFUSE_SERIAL1:
    case T40_EFUSE_SERIAL1 + 12:
        return 0x56780000;
    case T40_EFUSE_SERIAL2:
    case T40_EFUSE_SERIAL2 + 12:
        return 0x12100080;
    case T40_EFUSE_SECURITY:
        return s->efuse_security;
    case T40_EFUSE_BOOTCFG:
        return s->efuse_bootcfg;
    case T40_EFUSE_CHIPID3:
        return 0x00000000;
    case T40_EFUSE_SUBSOCTYPE1:
        return 0x00000000;
    case T40_EFUSE_WORD19:
        /*
         * MD5 rounds over words 0-2 from the IV a108c309 b4f84b07
         * c3073cf8 d2162de9, first word: what Zeratul libimp checks (a
         * T41NQ's word reads so); libimp asks for it as word 0x93.
         */
        return 0xb0d9aeb0;
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

static uint64_t t40_efuse_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicT40State *s = opaque;

    switch (offset) {
    case T40_EFUSE_CTRL:
        return s->efuse_ctrl;
    case T40_EFUSE_STATE:
        /* Bits 15:8 show the top byte of the security word */
        return s->efuse_state | (s->efuse_security >> 16 & 0xff00);
    case T40_EFUSE_DATA:
        return s->efuse_data;
    default:
        return t40_efuse_word(s, offset);
    }
}

static void t40_efuse_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    IngenicT40State *s = opaque;

    switch (offset) {
    case T40_EFUSE_CTRL:
        /* Bit 0 starts a read, done at once; bit 1 would blow fuses */
        if (value & 2) {
            qemu_log_mask(LOG_UNIMP, "%s: fuse programming\n", __func__);
        }
        s->efuse_ctrl = value & 0x07e00000;
        if (value & 1) {
            s->efuse_data = t40_efuse_word(s, 0x200 + 4 *
                                           extract32(value, 21, 6));
            s->efuse_state |= 1;
        }
        break;
    case T40_EFUSE_STATE:
        /* Writing 0 clears done; nothing sets it but a read */
        s->efuse_state &= value;
        break;
    }
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

/*
 * The PLL words and CPCCR the vendor U-Boot programs, which the CPM starts
 * from outside -bios. T41 encodes its PLLs as INGENIC_PLL_PLLREG.
 */
typedef struct {
    const char *name;
    uint32_t cpuid;
    uint32_t type2;
    uint32_t apll;
    uint32_t mpll;
    uint32_t ram_mb;
    bool has_epll;
    uint32_t l2_kib;     /* the CPUs' L2 cache, as Config2 reports it */
    uint32_t cpccr;
    IngenicPllFormat pll;
    bool t41;            /* T41 address map */
} T40Variant;

#define T40_CLK                 0x9a7b5510, INGENIC_PLL_MNOD
#define T41_CLK                 0x9a7b5520, INGENIC_PLL_PLLREG

static const T40Variant t40_variants[] = {
    /*
     * The T40 and the T41 both read cpuid 0x10040003 (family 0x0040); the
     * grade is the upper half of SUBSOCTYPE2, with the codes the vendor
     * libimp names (T40 1.3.1, T41 1.2.6). Whole words as measured on a
     * T40N, T40NN, T40XP, T41NQ and T41LQ. APLL/MPLL 912/1100, 1008/1400.
     */
    { "t40n",  0x10040003, 0x11111111, MNOD(76,1,2,1), MNOD(275,2,3,1), 128, true,  512, T40_CLK },
    { "t40nn", 0x10040003, 0x88881111, MNOD(76,1,2,1), MNOD(275,2,3,1), 128, true,  512, T40_CLK },
    { "t40xp", 0x10040003, 0x77772222, MNOD(84,1,2,1), MNOD(350,2,3,1), 256, true,  512, T40_CLK },
    { "t40a",  0x10040003, 0x44440000, MNOD(84,1,2,1), MNOD(350,2,3,1), 512, true,  512, T40_CLK },
    /* T41 family: no EPLL; APLL 1104, MPLL 1400 (LQ 1200) */
    { "t41nq", 0x10040003, 0xAAAA2222, 0x05b008bd, 0x15d0889d, 128, false, 256, T41_CLK, true },
    { "t41lq", 0x10040003, 0x99991111, 0x05b008bd, 0x063008bd,  64, false, 256, T41_CLK, true },
    { "t41zn", 0x10040003, 0x77772222, 0x05b008bd, 0x15d0889d, 128, false, 256, T41_CLK, true },
    { "t41xq", 0x10040003, 0x22220000, 0x05b008bd, 0x15d0889d, 256, false, 256, T41_CLK, true },
    { "t41zx", 0x10040003, 0x66660000, 0x05b008bd, 0x15d0889d, 256, false, 256, T41_CLK, true },
    /* Default */
    { "qemu",      0x10040003, 0xEE000000, MNOD(76,1,2,1), MNOD(275,2,3,1), 256, true,  512, T40_CLK },
    { "qemu-t41",  0x10040003, 0xEE010000, 0x05b008bd, 0x15d0889d, 256, false, 256, T41_CLK, true },
    { NULL, 0, 0, 0, 0, 0, false, 0 }
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

uint32_t ingenic_t40_l2_cache_size(IngenicT40State *s)
{
    return ingenic_t40_find_variant(s->soc_variant)->l2_kib * KiB;
}

void ingenic_t40_check_ram(IngenicT40State *s, uint64_t ram_size)
{
    const T40Variant *v = ingenic_t40_find_variant(s->soc_variant);

    if (ram_size < (uint64_t)v->ram_mb * MiB) {
        warn_report("the %s has %" PRIu32 " MiB of RAM but -m gives %" PRIu64
                    " MiB: firmware built for it may not start",
                    v->name, v->ram_mb, ram_size / MiB);
    }
}

Clock *ingenic_t40_cpu_clock(IngenicT40State *s)
{
    const T40Variant *v = ingenic_t40_find_variant(s->soc_variant);
    IngenicT40CpmState *cpm = &s->cpm;

    qdev_prop_set_bit(DEVICE(cpm), "has-epll", v->has_epll);
    qdev_prop_set_uint8(DEVICE(cpm), "pll-format", v->pll);
    /* The T40's three sensor clocks, CIM0CDR to CIM2CDR; the T41's one */
    qdev_prop_set_uint8(DEVICE(cpm), "cims", v->t41 ? 1 : 3);
    cpm->regs[0x10 / 4] = v->apll;
    cpm->regs[0x14 / 4] = v->mpll;
    if (v->has_epll) {
        cpm->regs[0x58 / 4] = MNOD(125, 1, 2, 1) | PLL_ON_EN;
    }
    cpm->regs[0xE0 / 4] = MNOD(100, 1, 2, 1) | PLL_ON_EN;
    /* CLKGR, CLKGR1 as the mask ROM leaves them, read in USB boot */
    cpm->regs[0x20 / 4] = v->t41 ? 0x1f9fffc4 : 0x1fdfff80;
    cpm->regs[0x28 / 4] = v->t41 ? 0x000263ff : 0x000033ff;
    if (v->t41) {
        /* SFC0CDR, SFC1CDR: bits 9:8 read 1, which Zeratul SPLs require */
        cpm->regs[0x60 / 4] = 0x00000100;
        cpm->regs[0x7c / 4] = 0x00000100;
    }
    /* With CPCCR still 0, the mask ROM runs from EXTCLK */
    if (!MACHINE(qdev_get_machine())->firmware) {
        cpm->regs[0x00 / 4] = v->cpccr;
    }
    memcpy(cpm->boot_regs, cpm->regs, sizeof(cpm->regs));
    cpm->boot_saved = true;
    return ingenic_t40_cpm_cpu_clock(cpm);
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
    [INGENIC_T40_DEV_I2C2]       = 0x10052000,
    [INGENIC_T40_DEV_I2C3]       = 0x10053000,
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

typedef struct {
    const char *name;
    hwaddr base;
    hwaddr size;
} T40Unimp;

/* Address windows as the vendor kernels' soc/base.h and device trees give them */
static const T40Unimp ingenic_t40_unimp[] = {
    { "ingenic-t40-ssi-slv",   0x10040000, 4 * KiB },
    { "ingenic-t40-usbphy",    0x10060000, 4 * KiB },
    { "ingenic-t40-aic",       0x10020000, 4 * KiB },
    { "ingenic-t40-codec",     0x10021000, 4 * KiB },
    { "ingenic-t40-mipiphy",   0x10022000, 4 * KiB },
    { "ingenic-t40-mipi",      0x10023000, 4 * KiB },
    /* RISCV_BASE: reads 0 on a T40XP; the core's block is at 0x12a00000 */
    { "ingenic-t40-riscv-base", 0x12210000, 64 * KiB },
    { "ingenic-t40-nndma",     0x12500000, 64 * KiB },
    { "ingenic-t40-ddrc-apb",  0x13010000, 4 * KiB },
    { "ingenic-t40-lcdc",      0x13050000, 64 * KiB },
    { "ingenic-t40-ipu",       0x13080000, 64 * KiB },
    { "ingenic-t40-monitor",   0x130a0000, 64 * KiB },
    { "ingenic-t40-i2d",       0x130b0000, 64 * KiB },
    { "ingenic-t40-vo",        0x130c0000, 64 * KiB },
    { "ingenic-t40-drawbox",   0x130d0000, 64 * KiB },
    { "ingenic-t40-isp",       0x13300000, 1024 * KiB },
    { "ingenic-t40-harb2",     0x13400000, 4 * KiB },
    { "ingenic-t40-nemc",      0x13410000, 64 * KiB },
    /* HASH and RSA are real device models */
};

static const T40Unimp ingenic_t40_only_unimp[] = {
    { "ingenic-t40-mipi-dsi",  0x10003000, 4 * KiB },
    { "ingenic-t40-dsi-phy",   0x10004000, 4 * KiB },
    { "ingenic-t40-dmic",      0x10034000, 4 * KiB },
    { "ingenic-t40-mipi-rx4l", 0x10054000, 4 * KiB },
    { "ingenic-t40-lepost",    0x12600000, 64 * KiB },
    { "ingenic-t40-lepccu",    0x12700000, 64 * KiB },
    { "ingenic-t40-axi-arb1",  0x13013000, 4 * KiB },
    { "ingenic-t40-axi-arb2",  0x13014000, 4 * KiB },
    { "ingenic-t40-radix",     0x13100000, 64 * KiB },
    { "ingenic-t40-el150",     0x13200000, 1024 * KiB },
};

static const T40Unimp ingenic_t41_only_unimp[] = {
    { "ingenic-t41-rtc-ftbt",  0x10004000, 4 * KiB },
    { "ingenic-t41-lepccu",    0x12a00000, 64 * KiB },
    { "ingenic-t41-aip",       0x12b00000, 64 * KiB },
    { "ingenic-t41-ldc",       0x13040000, 64 * KiB },
    { "ingenic-t41-el200",     0x13100000, 1024 * KiB },
    { "ingenic-t41-ivdc",      0x13200000, 512 * KiB },
    { "ingenic-t41-jpeg",      0x13280000, 256 * KiB },
    { "ingenic-t41-dmic",      0x134d0000, 64 * KiB },
};

static void ingenic_t40_init(Object *obj)
{
    IngenicT40State *s = INGENIC_T40(obj);

    s->memmap = ingenic_t40_memmap;

    object_initialize_child(obj, "cpm", &s->cpm, TYPE_INGENIC_T40_CPM);
    object_initialize_child(obj, "ddrc", &s->ddrc, TYPE_INGENIC_DDRC);
    object_initialize_child(obj, "sfc", &s->sfc, TYPE_INGENIC_A1_SFC);
    object_initialize_child(obj, "sfc1", &s->sfc1, TYPE_INGENIC_A1_SFC);
    object_initialize_child(obj, "gmac", &s->gmac, TYPE_INGENIC_GMAC);
    object_initialize_child(obj, "tcu", &s->tcu, TYPE_INGENIC_TCU);
    object_initialize_child(obj, "dtrng", &s->dtrng, TYPE_INGENIC_DTRNG);
    object_initialize_child(obj, "pwm", &s->pwm, TYPE_INGENIC_PWM);
    object_initialize_child(obj, "rtc", &s->rtc, TYPE_INGENIC_RTC);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_INGENIC_GPIO);
    object_initialize_child(obj, "sadc", &s->sadc, TYPE_INGENIC_SADC);
    object_initialize_child(obj, "intc", &s->intc, TYPE_INGENIC_INTC);
    object_initialize_child(obj, "ccu", &s->ccu, TYPE_INGENIC_XBURST2_CCU);
    object_initialize_child(obj, "riscv", &s->riscv, TYPE_INGENIC_RISCV);
    object_initialize_child(obj, "i2c0", &s->i2c[0], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c1", &s->i2c[1], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c2", &s->i2c[2], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c3", &s->i2c[3], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "ssi0", &s->ssi[0], TYPE_INGENIC_SSI);
    object_initialize_child(obj, "ssi1", &s->ssi[1], TYPE_INGENIC_SSI);
    object_initialize_child(obj, "aes", &s->aes, TYPE_INGENIC_AES);
    object_initialize_child(obj, "des", &s->des, TYPE_INGENIC_DES);
    object_initialize_child(obj, "sdhci0", &s->sdhci[0], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "sdhci1", &s->sdhci[1], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "dwc2", &s->dwc2, TYPE_DWC2_USB);
    object_initialize_child(obj, "pdma", &s->pdma, TYPE_INGENIC_PDMA);
    object_initialize_child(obj, "hash", &s->hash, TYPE_INGENIC_HASH);
    object_initialize_child(obj, "rsa", &s->rsa, TYPE_INGENIC_RSA);
    object_initialize_child(obj, "lzma0", &s->lzma[0], TYPE_INGENIC_LZMA);
    object_initialize_child(obj, "lzma1", &s->lzma[1], TYPE_INGENIC_LZMA);
    object_initialize_child(obj, "aic", &s->aic, TYPE_INGENIC_AIC);
    object_initialize_child(obj, "dmic", &s->dmic, TYPE_INGENIC_DMIC);
    object_initialize_child(obj, "as-dmic", &s->as_dmic, TYPE_INGENIC_AS_DMIC);
    object_initialize_child(obj, "ipu", &s->ipu, TYPE_INGENIC_IPU);
    object_property_add_const_link(OBJECT(&s->dwc2), "dma-mr",
                                   OBJECT(get_system_memory()));
}

/*
 * The SFCs' DMA sees the low window of memory as the SPL's while the boot
 * SRAM alias is armed: the SPL runs from cache lines that take its DMA
 * writes there, which a T41 Zeratul SPL relies on to read its rootfs
 * header into its own data. Uncached CPU accesses still reach DRAM.
 */
static hwaddr t40_boot_window_addr(IngenicT40State *s, hwaddr addr)
{
    return addr < s->cpu[0]->env.sram_alias_size ?
           s->memmap[INGENIC_T40_DEV_SRAM] + addr : addr;
}

static MemTxResult t40_boot_window_read(void *opaque, hwaddr addr,
                                        uint64_t *data, unsigned size,
                                        MemTxAttrs attrs)
{
    uint8_t buf[8];
    MemTxResult r = address_space_read(&address_space_memory,
                                       t40_boot_window_addr(opaque, addr),
                                       attrs, buf, size);

    *data = ldn_le_p(buf, size);
    return r;
}

static MemTxResult t40_boot_window_write(void *opaque, hwaddr addr,
                                         uint64_t data, unsigned size,
                                         MemTxAttrs attrs)
{
    uint8_t buf[8];

    stn_le_p(buf, size, data);
    return address_space_write(&address_space_memory,
                               t40_boot_window_addr(opaque, addr),
                               attrs, buf, size);
}

static const MemoryRegionOps t40_boot_window_ops = {
    .read_with_attrs = t40_boot_window_read,
    .write_with_attrs = t40_boot_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
    .impl = { .min_access_size = 1, .max_access_size = 8 },
};

static void ingenic_t40_realize(DeviceState *dev, Error **errp)
{
    IngenicT40State *s = INGENIC_T40(dev);
    bool t41 = ingenic_t40_find_variant(s->soc_variant)->t41;
    unsigned i;

    {
        const T40Variant *v = ingenic_t40_find_variant(s->soc_variant);
        s->efuse_subsoctype2 = v->type2;
        s->harb0_cpuid = v->cpuid;
        s->variant = v;
        /*
         * As a T40N, T40XP, T41NQ and T41LQ read them: the bootrom stays
         * off the UART with bit 14 of the boot config set
         */
        if (!s->efuse_security && !v->t41) {
            s->efuse_security = 0x01010000;
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
                    s->memmap[INGENIC_T40_DEV_CPM]);

    /* DDR Controller + PHY */
    sysbus_realize(SYS_BUS_DEVICE(&s->ddrc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 0,
                    s->memmap[INGENIC_T40_DEV_DDRC]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ddrc), 1,
                    s->memmap[INGENIC_T40_DEV_DDR_PHY]);

    memory_region_init(&s->sfc_dma, OBJECT(s), "sfc-dma", UINT64_MAX);
    memory_region_init_alias(&s->sfc_dma_sys, OBJECT(s), "sfc-dma-system",
                             get_system_memory(), 0, UINT64_MAX);
    memory_region_add_subregion(&s->sfc_dma, 0, &s->sfc_dma_sys);
    memory_region_init_io(&s->boot_window, OBJECT(s), &t40_boot_window_ops,
                          s, "sfc-dma-boot-window", INGENIC_T40_SRAM_SIZE);
    memory_region_add_subregion_overlap(&s->sfc_dma, 0, &s->boot_window, 1);
    object_property_set_link(OBJECT(&s->sfc), "dma-memory",
                             OBJECT(&s->sfc_dma), &error_abort);
    object_property_set_link(OBJECT(&s->sfc1), "dma-memory",
                             OBJECT(&s->sfc_dma), &error_abort);

    /* SFC V2 (same IP as A1), IRQ -> INTC source 7 */
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc), 0,
                    s->memmap[INGENIC_T40_DEV_SFC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 7));

    /* T41 SFC1: the flash given as -drive if=mtd,index=1; IRQ -> 19 */
    qdev_prop_set_uint8(DEVICE(&s->sfc1), "unit", 1);
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc1), &error_fatal);
    if (t41) {
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc1), 0, 0x13450000);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc1), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 19));
    }

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
    /* GMAC IRQ -> INTC source 55 (IRQ_GMAC0 = 32 + 23, bank 1 bit 23) */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gmac), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 55));

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

    /* PWM controller, T41 only: IRQ -> INTC source 63 */
    sysbus_realize(SYS_BUS_DEVICE(&s->pwm), &error_fatal);
    if (t41) {
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->pwm), 0, 0x13460000);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->pwm), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 63));
    }

    /* RTC, IRQ -> INTC source 3; the T41 moved it to the APB bus */
    sysbus_realize(SYS_BUS_DEVICE(&s->rtc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc), 0,
                    t41 ? 0x10003000 : s->memmap[INGENIC_T40_DEV_RTC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 3));

    /*
     * GPIO: four ports 0x1000 apart (the driver's PxOFFSET; the device
     * tree's regs-offset of 0x100 goes unused), port N interrupting on
     * INTC 17 - N
     */
    qdev_prop_set_uint32(DEVICE(&s->gpio), "nr-ports", 4);
    sysbus_realize(SYS_BUS_DEVICE(&s->gpio), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0,
                    s->memmap[INGENIC_T40_DEV_GPIO]);

    /* INTC - one register window per core, and the APB one */
    qdev_prop_set_uint32(DEVICE(&s->intc), "num-cpus", s->num_cpus);
    qdev_prop_set_bit(DEVICE(&s->intc), "apb-window", true);
    sysbus_realize(SYS_BUS_DEVICE(&s->intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0,
                    s->memmap[INGENIC_T40_DEV_INTCN]);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 1,
                    s->memmap[INGENIC_T40_DEV_INTC]);
    for (unsigned n = 0; n < 4; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), n,
                           qdev_get_gpio_in(DEVICE(&s->intc), 17 - n));
    }

    sysbus_realize(SYS_BUS_DEVICE(&s->sadc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sadc), 0, 0x10070000);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sadc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 18));

    /* I2C: four channels on the T40, three on the T41; IRQ 60 down */
    for (i = 0; i < 4; i++) {
        sysbus_realize(SYS_BUS_DEVICE(&s->i2c[i]), &error_fatal);
        if (t41 && i == 3) {
            break;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                        s->memmap[INGENIC_T40_DEV_I2C0] + i * 0x1000);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 60 - i));
    }
    /*
     * I2C0's pads are PA12 (SDA) and PA13 (SCL), as on the T31, and
     * vendor factory sensor probes bit-bang them as GPIOs.
     */
    qdev_connect_gpio_out_named(DEVICE(&s->gpio), "pad", 12,
                                qdev_get_gpio_in_named(DEVICE(&s->i2c[0]),
                                                       "pads",
                                                       BITBANG_I2C_SDA));
    qdev_connect_gpio_out_named(DEVICE(&s->gpio), "pad", 13,
                                qdev_get_gpio_in_named(DEVICE(&s->i2c[0]),
                                                       "pads",
                                                       BITBANG_I2C_SCL));
    qdev_connect_gpio_out_named(DEVICE(&s->i2c[0]), "sda", 0,
                                qdev_get_gpio_in_named(DEVICE(&s->gpio),
                                                       "pull", 12));
    /*
     * A T41's I2C1 on PC19 (SDA) and PC20 (SCL): the RISC-V core of a
     * eufy T8160C bit-bangs its OPT3001 there
     */
    if (t41) {
        qdev_connect_gpio_out_named(DEVICE(&s->gpio), "pad", 83,
                                    qdev_get_gpio_in_named(DEVICE(&s->i2c[1]),
                                                           "pads",
                                                           BITBANG_I2C_SDA));
        qdev_connect_gpio_out_named(DEVICE(&s->gpio), "pad", 84,
                                    qdev_get_gpio_in_named(DEVICE(&s->i2c[1]),
                                                           "pads",
                                                           BITBANG_I2C_SCL));
        qdev_connect_gpio_out_named(DEVICE(&s->i2c[1]), "sda", 0,
                                    qdev_get_gpio_in_named(DEVICE(&s->gpio),
                                                           "pull", 83));
    }

    /* AES = IRQ 23; DES has no interrupt */
    sysbus_realize(SYS_BUS_DEVICE(&s->aes), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aes), 0, s->memmap[INGENIC_T40_DEV_AES]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->aes), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 23));
    sysbus_realize(SYS_BUS_DEVICE(&s->des), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->des), 0, 0x10061000);

    /* SSI0 = IRQ 9, SSI1 = IRQ 8 */
    for (i = 0; i < 2; i++) {
        DeviceState *d = DEVICE(&s->ssi[i]);

        qdev_prop_set_string(d, "bus-name", i ? "ssi1" : "ssi0");
        qdev_prop_set_bit(d, "dreq-status", true);
        qdev_prop_set_bit(d, "spcnt", true);
        sysbus_realize(SYS_BUS_DEVICE(d), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(d), 0,
                        s->memmap[INGENIC_T40_DEV_SSI0] + i * 0x1000);
        sysbus_connect_irq(SYS_BUS_DEVICE(d), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 9 - i));
    }

    /*
     * MSC0/MSC1: standard SDHCI blocks, which the mask ROM drives too;
     * IRQ -> INTC source 37 (MSC0), 36 (MSC1)
     */
    for (i = 0; i < 2; i++) {
        g_autofree char *name = g_strdup_printf("msc%d", i);

        object_property_set_uint(OBJECT(&s->sdhci[i]),
                                 "sd-spec-version", 3, &error_abort);
        /*
         * The 4.4 drivers read write-protect inverted and replace these
         * caps with their own, which select ADMA2; QEMU only runs the DMA
         * modes capareg advertises.
         */
        object_property_set_bool(OBJECT(&s->sdhci[i]),
                                 "wp-inverted", true, &error_abort);
        object_property_set_uint(OBJECT(&s->sdhci[i]),
                                 "capareg",
                                 (1ULL << 26) | (1ULL << 25) |
                                 (1ULL << 24) | (1ULL << 22) |
                                 (1ULL << 21) | (1ULL << 19) | (25 << 8),
                                 &error_abort);
        object_property_set_str(OBJECT(&s->sdhci[i]), "bus-name", name,
                                &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->sdhci[i]), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[i]), 0,
                        s->memmap[INGENIC_T40_DEV_MSC0 + i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdhci[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 37 - i));
    }

    /* USB DWC2 OTG (single) -> INTC source 21 */
    sysbus_realize(SYS_BUS_DEVICE(&s->dwc2), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->dwc2), 0,
                    s->memmap[INGENIC_T40_DEV_OTG]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->dwc2), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 21));
    /* usb-role overrides the PHY ID through CPM_USBRDT (IDDIG_REG). */
    qdev_connect_gpio_out_named(DEVICE(&s->cpm), "otg-id-change", 0,
                                qdev_get_gpio_in_named(DEVICE(&s->dwc2),
                                                       "otg-id-change", 0));

    /* PDMA, IRQ -> INTC source 10 */
    sysbus_realize(SYS_BUS_DEVICE(&s->pdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pdma), 0,
                    s->memmap[INGENIC_T40_DEV_PDMA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pdma), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 10));

    /* HASH engine, IRQ -> INTC source 22 */
    sysbus_realize(SYS_BUS_DEVICE(&s->hash), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hash), 0,
                    s->memmap[INGENIC_T40_DEV_HASH]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->hash), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 22));

    /* RSA accelerator */
    sysbus_realize(SYS_BUS_DEVICE(&s->rsa), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rsa), 0,
                    s->memmap[INGENIC_T40_DEV_RSA]);

    /*
     * LZMA decompressors: the T40's is a mode of its BSCALER block, the
     * T41 has two of its own.
     */
    for (i = 0; i < 2; i++) {
        qdev_prop_set_string(DEVICE(&s->lzma[i]), "model", t41 ? "t41" : "t40");
        sysbus_realize(SYS_BUS_DEVICE(&s->lzma[i]), &error_fatal);
    }
    if (t41) {
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->lzma[0]), 0, 0x132c0000);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->lzma[1]), 0, 0x132e0000);
    } else {
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->lzma[0]), 0, 0x13090000);
    }

    /*
     * The AIC (INTC source 1) and the internal codec. DMA requests as the
     * PDMA numbers them: TX 0x06, RX 0x07, the TFIFO loop 0x05.
     */
    qdev_prop_set_string(DEVICE(&s->aic), "codec", t41 ? "t41" : "t40");
    object_property_set_link(OBJECT(&s->aic), "pdma", OBJECT(&s->pdma),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->aic), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 0, 0x10020000);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 1, 0x10021000);
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
     * The T40's DMIC at 0x10034000 (INTC source 12, DMA request 0x0d); the
     * T41 puts UART4 there and has the AudioSystem's (INTC source 33)
     */
    object_property_set_link(OBJECT(&s->dmic), "pdma", OBJECT(&s->pdma),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->dmic), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(&s->as_dmic), &error_fatal);
    if (!t41) {
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->dmic), 0,
                        s->memmap[INGENIC_T40_DEV_DMIC]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->dmic), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 12));
        qdev_connect_gpio_out_named(DEVICE(&s->dmic), "dreq", 0,
            qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq", 0x0d));
    } else {
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->as_dmic), 0, 0x134d0000);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->as_dmic), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 33));
    }

    /*
     * The ISP (INTC source 31) and its VICs (the T40's at 30, 29 and 28, the
     * T41's one at 30): frames from the image sensor out to tx-isp's
     * buffers. The AVPU, the Allegro encoder (62: the T40's EL150, the
     * T41's EL200), and the IPU that libimp draws the OSD with (6).
     */
    object_initialize_child(OBJECT(s), "isp", &s->isp,
                            t41 ? TYPE_INGENIC_T41_ISP : TYPE_INGENIC_T40_ISP);
    sysbus_realize(SYS_BUS_DEVICE(&s->isp), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->isp), 0, s->memmap[INGENIC_T40_DEV_ISP]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->isp), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 31));
    for (i = 0; i < (t41 ? 1 : 3); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->isp), 1 + i,
                           qdev_get_gpio_in(DEVICE(&s->intc), 30 - i));
    }
    /*
     * CLKGR bit 23 gates it, SRBC 22 resets it, and the sensor's MCLK, which
     * tx-isp stops as it closes, stops its frames
     */
    qdev_connect_gpio_out_named(DEVICE(&s->cpm), "isp-clock", 0,
        qdev_get_gpio_in_named(DEVICE(&s->isp), "clock", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->cpm), "isp-reset", 0,
        qdev_get_gpio_in_named(DEVICE(&s->isp), "reset", 0));
    qdev_connect_gpio_out_named(DEVICE(&s->cpm), "cim-clock", 0,
        qdev_get_gpio_in_named(DEVICE(&s->isp), "mclk", 0));
    /* The AVPU's identity, as a T40NN and a T41NQ read it */
    object_initialize_child(OBJECT(s), "avpu", &s->avpu,
                            t41 ? TYPE_INGENIC_T41_AVPU : TYPE_INGENIC_AVPU);
    qdev_prop_set_uint32(DEVICE(&s->avpu), "hw-id",
                         t41 ? 0x71000260 : 0x72000760);
    qdev_prop_set_uint32(DEVICE(&s->avpu), "date",
                         t41 ? 0x20211214 : 0x20200311);
    qdev_prop_set_uint32(DEVICE(&s->avpu), "rev",
                         t41 ? 0x62d42e77 : 0xa8a57397);
    object_property_set_link(OBJECT(&s->avpu), "isp", OBJECT(&s->isp),
                             &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->avpu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->avpu), 0,
                    t41 ? 0x13100000 : s->memmap[INGENIC_T40_DEV_EL150]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->avpu), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 62));
    sysbus_realize(SYS_BUS_DEVICE(&s->ipu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ipu), 0, s->memmap[INGENIC_T40_DEV_IPU]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->ipu), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 6));
    /*
     * The T41's IVDC (13), which in direct mode passes the ISP's channel 0
     * straight to the EL200, over the start of the window the vendor
     * kernels give it
     */
    if (t41) {
        object_initialize_child(OBJECT(s), "ivdc", &s->ivdc,
                                TYPE_INGENIC_IVDC);
        sysbus_realize(SYS_BUS_DEVICE(&s->ivdc), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->ivdc), 0, 0x13200000);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ivdc), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 13));
        for (i = 0; i < INGENIC_IVDC_SENSORS; i++) {
            qdev_connect_gpio_out_named(DEVICE(&s->isp), "ivdc-frame", i,
                qdev_get_gpio_in_named(DEVICE(&s->ivdc), "frame", i));
        }
    }

    /* UARTs (4 channels, IRQs: UART0=51, UART1=50, UART2=49, UART3=48) */
    for (i = 0; i < 4; i++) {
        g_autofree char *name = g_strdup_printf("ingenic-t40-uart%u", i);

        ingenic_uart_create(get_system_memory(),
                            s->memmap[INGENIC_T40_DEV_UART0] + i * 0x1000,
                            qdev_get_gpio_in(DEVICE(&s->intc), 51 - i),
                            serial_hd(i), name);
    }
    /* The T41 adds UART4 and UART5 where the T40 has its DMIC (IRQ 57, 56) */
    for (i = 4; t41 && i < 6; i++) {
        g_autofree char *name = g_strdup_printf("ingenic-t41-uart%u", i);

        ingenic_uart_create(get_system_memory(),
                            s->memmap[INGENIC_T40_DEV_UART0] + i * 0x1000,
                            qdev_get_gpio_in(DEVICE(&s->intc), 61 - i),
                            serial_hd(i), name);
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
    qemu_register_reset(t40_cost_reset, s);

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
    /* A T40XP holds its RISC-V core from reset, a T41NQ runs it */
    qdev_prop_set_uint32(DEVICE(&s->ccu), "cfcr-reset",
                         t41 ? 0x00000038 : 0x80000018);
    sysbus_realize(SYS_BUS_DEVICE(&s->ccu), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ccu), 0,
                    s->memmap[INGENIC_T40_DEV_CCU]);

    /* The RISC-V core: its block, released by the CCU's CFCR, IRQ 2 */
    qdev_prop_set_string(DEVICE(&s->riscv), "model", t41 ? "t41" : "t40");
    sysbus_realize(SYS_BUS_DEVICE(&s->riscv), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->riscv), 1, 0x12a00000);
    qdev_connect_gpio_out_named(DEVICE(&s->ccu), "riscv-hold", 0,
        qdev_get_gpio_in_named(DEVICE(&s->riscv), "hold", 0));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->riscv), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 2));

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

    /*
     * DMA requests as the PDMA numbers them: SSI0 TX/RX 0x16/0x17, SSI1
     * 0x18/0x19, DES 0x2e/0x2f. The PDMA tells each SSI when a channel
     * serves it.
     */
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
    for (int d = 0; d < 2; d++) {
        qdev_connect_gpio_out_named(DEVICE(&s->des), "dreq", d,
            qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq", 0x2e + d));
    }

    memory_region_init_io(&s->unmapped, OBJECT(s), &t40_unmapped_ops, s,
                          "ingenic-t40-unmapped", 4 * GiB);
    memory_region_add_subregion_overlap(get_system_memory(), 0, &s->unmapped,
                                        -2000);

    /* Unimplemented peripherals */
    for (i = 0; i < ARRAY_SIZE(ingenic_t40_unimp); i++) {
        create_unimplemented_device(ingenic_t40_unimp[i].name,
                                    ingenic_t40_unimp[i].base,
                                    ingenic_t40_unimp[i].size);
    }
    if (t41) {
        for (i = 0; i < ARRAY_SIZE(ingenic_t41_only_unimp); i++) {
            create_unimplemented_device(ingenic_t41_only_unimp[i].name,
                                        ingenic_t41_only_unimp[i].base,
                                        ingenic_t41_only_unimp[i].size);
        }
    } else {
        for (i = 0; i < ARRAY_SIZE(ingenic_t40_only_unimp); i++) {
            create_unimplemented_device(ingenic_t40_only_unimp[i].name,
                                        ingenic_t40_only_unimp[i].base,
                                        ingenic_t40_only_unimp[i].size);
        }
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
