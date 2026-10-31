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
#include "hw/char/ingenic-uart.h"
#include "hw/misc/unimp.h"
#include "system/physmem.h"
#include "qemu/log.h"
#include "trace.h"
#include "system/watchdog.h"
#include "system/runstate.h"
#include "system/reset.h"
#include "hw/core/loader.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/sysbus.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "net/net.h"
#include "hw/core/boards.h"
#include "hw/mips/ingenic-t31.h"
#include "hw/misc/ingenic-cpm-clk.h"

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

#define T32PRO_CPUID    0x10032005      /* the CW080's too */

/*
 * The T32Pro's TNPU ID (UMPID, 0x12700038): its tnpu.ko refuses a low
 * half under 0x150, the value given here; silicon's is unmeasured. The
 * driver loads the NPU's RISC-V boot code into an SRAM at 0x150b0000.
 */
#define TNPU_UMPID      0x38

static uint64_t ingenic_t31_tnpu_id_read(void *opaque, hwaddr offset,
                                         unsigned size)
{
    return 0x150;
}

static const MemoryRegionOps ingenic_t31_tnpu_id_ops = {
    .read = ingenic_t31_tnpu_id_read,
    .write = ingenic_t31_harb0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * eFuse: the window at 0x200 mirrors the fuse array word by word, and
 * registers 0x000-0x00c read it (see ingenic_t31_efuse_fetch()).
 */
#define EFUSE_SERIAL0       0x200
#define EFUSE_SERIAL1       0x204
#define EFUSE_SERIAL2       0x208
#define EFUSE_T33_VARIANT   0x21C
#define EFUSE_SUBREMARK     0x231
#define EFUSE_WORD13        0x234   /* Zeratul SPLs power off on a zero */
#define EFUSE_SUBSOCTYPE1   0x238

/*
 * Map SoC variant names to EFUSE SUBSOCTYPE1 values (upper 16 bits).
 * libimp's get_cpu_id() and thingino's /usr/sbin/soc both read this
 * register at 0x13540238 to identify the exact chip variant.
 */
/*
 * T31 variant table: maps sub-model name to EFUSE id, PLL register
 * values, and RAM size. PLL MNOD encoding from U-Boot isvp_common.h:
 *   (nf << 20) | (nr << 14) | (od1 << 11) | (od0 << 8)
 * T10 and T20 add a fraction of nf; T21 and T30 encode their PLLs
 * differently (INGENIC_PLL_PLLREG), so their words are raw. The CPU
 * runs from the APLL. VPLL is always 1200 MHz on T31.
 */
#define PLL_ON_EN  ((1 << 3) | (1 << 0))
#define MNOD(nf, nr, od1, od0)  (((nf) << 20) | ((nr) << 14) | \
                                 ((od1) << 11) | ((od0) << 8) | PLL_ON_EN)

typedef enum {
    T31_MXU2,            /* the MXU2 unit works */
    T31_MXU2_FUSED,      /* Status.CU2 works, MXU2 is reserved (T21N) */
    T31_NO_COP2,         /* Status.CU2 reads 0, MXU2 traps on it (T23N) */
} T31Mxu;

typedef struct {
    const char *name;
    uint32_t cpuid;      /* HARB0+0x2C: bits[27:12]=family, bit28=1 */
    uint32_t type1;      /* EFUSE SUBSOCTYPE1 */
    uint32_t t33var;     /* EFUSE T33 variant word at 0x21C (byte 3 = variant) */
    uint32_t apll;       /* CPAPCR register value */
    uint32_t mpll;       /* CPMPCR register value */
    uint32_t ram_mb;
    uint16_t l1_kib;     /* each of the CPU's L1 caches; 0: CPU model's */
    uint16_t l2_kib;     /* the CPU's L2 cache; 0: CPU model's */
    T31Mxu mxu;
    uint32_t prid;       /* the CPU's PRId; 0: CPU model's */
    uint32_t apll_frac;  /* T10/T20 APLL fraction of nf, in 2^-24 */
} T31Variant;

static const T31Variant t31_variants[] = {
    /*
     * T10 family (cpuid bits[27:12] = 0x0005; 0x10005003 measured on a
     * T10L). T10 has no grade fuse, so the models look alike to software.
     */
    { "t10",   0x10005003, 0x00000000, 0, MNOD( 71,2,1,1), MNOD(100,1,2,1),  64, 32, 128, T31_MXU2, 0, 0x00ae147a },  /* 860.16/600 */
    { "t10n",  0x10005003, 0x00000000, 0, MNOD( 71,2,1,1), MNOD(100,1,2,1),  64, 32, 128, T31_MXU2, 0, 0x00ae147a },  /* 860.16/600 */
    { "t10a",  0x10005003, 0x00000000, 0, MNOD( 71,2,1,1), MNOD(100,1,2,1),  64, 32, 128, T31_MXU2, 0, 0x00ae147a },  /* 860.16/600 */
    { "t10l",  0x10005003, 0x00000000, 0, MNOD( 59,2,1,1), MNOD(100,1,2,1),  64, 32, 128, T31_MXU2, 0, 0x00645a1c },  /* 712.70/600 */
    /* T20 family (cpuid bits[27:12] = 0x2000; 0x12000002 measured on a T20X) */
    { "t20n",  0x12000002, 0x11110000, 0, MNOD( 71,2,1,1), MNOD(125,3,1,1),  64, 32, 128, T31_MXU2, 0x00d00101, 0x00ae147a },  /* 860.16/500 */
    { "t20x",  0x12000002, 0x22220000, 0, MNOD( 71,2,1,1), MNOD(100,1,2,1), 128, 32, 128, T31_MXU2, 0x00d00101, 0x00ae147a },  /* 860.16/600 */
    { "t20l",  0x12000002, 0x33330000, 0, MNOD( 59,2,1,1), MNOD(125,3,1,1),  64, 32, 128, T31_MXU2, 0x00d00101, 0x00645a1c },  /* 712.70/500 */
    /* T21 family (cpuid 0x10021003 on a T21N, InnoSilicon DDR PHY like T31) */
    { "t21n",  0x10021003, 0x11110000, 0, 0x0470484d, 0x04a0484d,  64, 16,  64, T31_MXU2_FUSED },  /* 864/900 */
    { "t21l",  0x10021003, 0x33330000, 0, 0x0470484d, 0x04a0484d,  64, 16,  64, T31_MXU2_FUSED },  /* 864/900 */
    { "t21z",  0x10021003, 0x55550000, 0, 0x0470484d, 0x04a0484d,  64, 16,  64, T31_MXU2_FUSED },  /* 864/900 */
    /*
     * T30 family (cpuid 0x10030005, type1 0x22221111 on a T30X, InnoSilicon
     * DDR PHY, PLL od0 at bit 5 not bit 8 - use raw MNOD values from
     * isvp_common.h)
     */
    { "t30n",  0x10030005, 0x11110000, 0, (( 74<<20)|(1<<14)|(1<<11)|(2<<5))|PLL_ON_EN, 0x07c0882d,  64, 32, 128 },  /*  900/1000 */
    { "t30x",  0x10030005, 0x22221111, 0, (( 74<<20)|(1<<14)|(1<<11)|(2<<5))|PLL_ON_EN, 0x07c0882d, 128, 32, 128 },  /*  900/1000 */
    { "t30l",  0x10030005, 0x33330000, 0, ((124<<20)|(1<<14)|(2<<11)|(2<<5))|PLL_ON_EN, 0x07c0882d,  64, 32, 128 },  /*  750/1000 */
    { "t30a",  0x10030005, 0x44440000, 0, ((149<<20)|(2<<14)|(1<<11)|(1<<5))|PLL_ON_EN, 0x0950882d, 128, 32, 128 },  /* 1200/1200 */
    { "t30z",  0x10030005, 0x55550000, 0, (( 74<<20)|(1<<14)|(1<<11)|(2<<5))|PLL_ON_EN, 0x07c0882d,  64, 32, 128 },  /*  900/1000 */
    /*
     * T23 family (cpuid 0x10023003, type1 0x11111111 on a T23N and
     * 0x33331111 on a T23DL, InnoSilicon DDR PHY like T31)
     */
    { "t23n",  0x10023003, 0x11111111, 0, MNOD(297,3,2,1), MNOD(100,1,2,1),  64, 16,  64, T31_NO_COP2 },  /* 1188/600 */
    { "t23x",  0x10023003, 0x22220000, 0, MNOD(297,3,2,1), MNOD(100,1,2,1), 128, 16,  64, T31_NO_COP2 },  /* 1188/600 */
    { "t23dl", 0x10023003, 0x33331111, 0, MNOD(297,3,2,1), MNOD(100,1,2,1),  32, 16,  64, T31_NO_COP2 },  /* 1188/600 */
    { "t23zn", 0x10023003, 0x77770000, 0, MNOD(125,1,3,1), MNOD(125,1,3,1),  64, 16,  64, T31_NO_COP2 },  /* 1000/500 */
    /* T31 family (cpuid 0x10031003, type1 0x22221111 on a T31X) */
    { "t31n",  0x10031003, 0x11110000, 0, MNOD(117,1,2,1), MNOD(125,1,3,1),  64, 32, 128 },  /* 1404/500 */
    { "t31x",  0x10031003, 0x22221111, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128, 32, 128 },  /* 1392/600 */
    { "t31l",  0x10031003, 0x33330000, 0, MNOD( 84,1,2,1), MNOD(125,1,3,1),  64, 32, 128 },  /* 1008/500 */
    { "t31a",  0x10031003, 0x44440000, 0, MNOD(125,1,2,1), MNOD(125,1,2,1), 128, 32, 128 },  /* 1500/750 */
    { "t31zl", 0x10031003, 0x55550000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1),  64, 32, 128 },  /* 1392/600 */
    { "t31zx", 0x10031003, 0x66660000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128, 32, 128 },  /* 1392/600 */
    { "t31al", 0x10031003, 0xCCCC0000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128, 32, 128 },  /* 1392/600 */
    { "t31zc", 0x10031003, 0xDDDD0000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128, 32, 128 },  /* 1392/600 */
    { "t31lc", 0x10031003, 0xEEEE0000, 0, MNOD( 92,1,2,1), MNOD(125,1,3,1),  64, 32, 128 },  /* 1104/500 */
    /* C100: a die-shrunk T31 that reads the T31A's IDs, 1 Gbit DDR3L in package */
    { "c100",  0x10031003, 0x44440000, 0, MNOD(125,1,2,1), MNOD(125,1,2,1), 128, 32, 128 },  /* 1500/750 */
    /* T32 family (cpuid bits[27:12] = 0x0032, same DDR/GPIO as T31,
     * low nibble = 4 is the revision code expected by vendor SPL;
     * type1 0x99991111 on a T32LQ) */
    { "t32nq", 0x10032004, 0xAAAA0000, 0, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128, 32, 128 },  /* 864/600 */
    { "t32lq", 0x10032004, 0x99991111, 0, MNOD( 72,1,2,1), MNOD(100,1,2,1),  64, 32, 128 },  /* 864/600 */
    { "t32xq", 0x10032004, 0x22220000, 0, MNOD( 72,1,2,1), MNOD(100,1,2,1), 256, 32, 128 },  /* 864/600 */
    { "t32zn", 0x10032004, 0x77770000, 0, MNOD( 72,1,2,1), MNOD(125,1,3,1), 128, 32, 128 },  /* 864/500 */
    /*
     * T32Pro and CW080 (cpuid revision 5, the CW080's type1 top byte below
     * 0x10), with the PLLs the PRJ009 demo's U-Boot sets; their MPLL
     * dividers are stored less one. libimp gives the T32Pro-A no DDR size.
     */
    { "t32prozl",  0x10032005, 0x55550000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0),  64, 32, 128 },  /* 1200/1066 */
    { "t32prozn",  0x10032005, 0x77770000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 128, 32, 128 },  /* 1200/1066 */
    { "t32prozx",  0x10032005, 0x66660000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 256, 32, 128 },  /* 1200/1066 */
    { "t32provl",  0x10032005, 0xCCCC0000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0),  64, 32, 128 },  /* 1200/1066 */
    { "t32provn",  0x10032005, 0xDDDD0000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 128, 32, 128 },  /* 1200/1066 */
    { "t32provx",  0x10032005, 0xFFFF0000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 256, 32, 128 },  /* 1200/1066 */
    { "t32provnp", 0x10032005, 0xBBBB0000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 128, 32, 128 },  /* 1200/1066 */
    { "t32proa",   0x10032005, 0x44440000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 128, 32, 128 },  /* 1200/1066 */
    { "cw080sn",   0x10032005, 0x01110000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 128, 32, 128 },  /* 1200/1066 */
    { "cw080snp",  0x10032005, 0x02220000, 0, MNOD(100,1,2,1), MNOD(533,6,1,0), 128, 32, 128 },  /* 1200/1066 */
    /* T33 family (cpuid bits[27:12] = 0x0033, variant byte at efuse 0x21C>>24) */
    { "t33n",      0x10033004, 0x00000000, 0xAA000000, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128,  0,   0 },
    { "t33zl",     0x10033004, 0x00000000, 0x55000000, MNOD( 72,1,2,1), MNOD(100,1,2,1),  64,  0,   0 },
    { "t33zn",     0x10033004, 0x00000000, 0x77000000, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128,  0,   0 },
    { "qemu-t33",  0x10033004, 0x00000000, 0xEE000000, MNOD( 72,1,2,1), MNOD(100,1,2,1), 128,  0,   0 },
    /* Default */
    { "qemu",  0x10031003, 0xEE000000, 0, MNOD(116,1,2,1), MNOD(100,1,2,1), 128, 32, 128 },  /* 1392/600 */
    { NULL }
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

void ingenic_t31_check_ram(IngenicT31State *s, uint64_t ram_size)
{
    const T31Variant *v = ingenic_t31_find_variant(s->soc_variant);

    if (ram_size < (uint64_t)v->ram_mb * MiB) {
        warn_report("the %s has %" PRIu32 " MiB of RAM but -m gives %" PRIu64
                    " MiB: firmware built for it may not start",
                    v->name, v->ram_mb, ram_size / MiB);
    }
}

static IngenicPllFormat ingenic_t31_pll_format(const T31Variant *v)
{
    switch (extract32(v->cpuid, 12, 16)) {
    case 0x0005:        /* T10 */
    case 0x2000:        /* T20 */
        return INGENIC_PLL_MNOD_FRAC;
    case 0x0021:        /* T21 */
    case 0x0030:        /* T30 */
        return INGENIC_PLL_PLLREG;
    default:
        return INGENIC_PLL_MNOD;
    }
}

/*
 * CLKGR0 and CLKGR1 as the mask ROM leaves them, read in USB boot (it has
 * started the SFC by then); the T33 is taken to match the T32
 */
static void ingenic_t31_clock_gates(const T31Variant *v, uint32_t *gr)
{
    switch (extract32(v->cpuid, 12, 16)) {
    case 0x0005:        /* T10 */
    case 0x2000:        /* T20 */
        gr[0] = 0x1fefff80;
        gr[1] = 0x000023ff;
        break;
    case 0x0023:        /* T23 */
        gr[0] = 0x1befff80;
        gr[1] = 0x000033ff;
        break;
    case 0x0032:        /* T32 */
    case 0x0033:        /* T33 */
        gr[0] = 0xf1b9ffe0;
        gr[1] = 0x0000eb3b;
        break;
    default:            /* T21, T30, T31 */
        gr[0] = 0x1fefff80;
        gr[1] = 0x000033ff;
        break;
    }
}

Clock *ingenic_t31_cpu_clock(IngenicT31State *s)
{
    const T31Variant *v = ingenic_t31_find_variant(s->soc_variant);
    DeviceState *cpm = DEVICE(&s->cpm);
    uint32_t gr[2];

    ingenic_t31_clock_gates(v, gr);
    qdev_prop_set_uint32(cpm, "clkgr0", gr[0]);
    qdev_prop_set_uint32(cpm, "clkgr1", gr[1]);
    qdev_prop_set_uint8(cpm, "pll-format", ingenic_t31_pll_format(v));
    /* The T32Pro and CW080 keep their MPLL's dividers less one */
    qdev_prop_set_uint8(cpm, "mpll-format", v->cpuid == T32PRO_CPUID
                        ? INGENIC_PLL_MNOD_OD1 : ingenic_t31_pll_format(v));
    qdev_prop_set_bit(cpm, "srbc1", extract32(v->cpuid, 12, 16) == 0x0030);
    if (extract32(v->cpuid, 12, 16) == 0x0032 ||
        extract32(v->cpuid, 12, 16) == 0x0033) {
        qdev_prop_set_uint8(cpm, "riscv-gate", 22);
        qdev_prop_set_uint8(cpm, "isp-gate", 19);
    }
    if (MACHINE(qdev_get_machine())->firmware) {
        /* The mask ROM runs before anything starts a PLL: SCLK_A = EXTCLK */
        qdev_prop_set_uint32(cpm, "cpccr", 0x55800000);
    } else {
        /* As the variant's firmware leaves the PLLs, for -kernel boots */
        qdev_prop_set_uint32(cpm, "apll", v->apll);
        qdev_prop_set_uint32(cpm, "apll-frac", v->apll_frac);
        qdev_prop_set_uint32(cpm, "mpll", v->mpll);
    }
    return ingenic_t31_cpm_cpu_clock(&s->cpm);
}

void ingenic_t31_cpu_props(IngenicT31State *s, DeviceState *cpu)
{
    const T31Variant *v = ingenic_t31_find_variant(s->soc_variant);

    qdev_prop_set_uint32(cpu, "prid", v->prid);
    qdev_prop_set_uint32(cpu, "l1i-cache-size", v->l1_kib * KiB);
    qdev_prop_set_uint32(cpu, "l1d-cache-size", v->l1_kib * KiB);
    qdev_prop_set_uint32(cpu, "l2-cache-size", v->l2_kib * KiB);
    qdev_prop_set_bit(cpu, "mxu", v->mxu == T31_MXU2);
    qdev_prop_set_bit(cpu, "cop2", v->mxu != T31_NO_COP2);
}

/*
 * The eFuse controller and fuse contents by family, as a T10L, T20X,
 * T21N, T23N, T23DL, T30X, T31X and T32LQ read when their read protocol
 * is driven from U-Boot. The T33 is taken to match the T32.
 */
typedef struct {
    uint32_t ctrl_mask;     /* the bits of reg 0x000 that read back */
    uint8_t addr_bits;      /* the fuse address at bit 21 */
    uint8_t len_bits;       /* by byte: the length less one at bit 16 */
    hwaddr mirror;          /* the first window word that mirrors its fuse */
    bool blank;             /* no fuse is blown */
    uint32_t word[16];      /* the fuses no property or variant sets */
} IngenicT31Efuse;

/* T10 and T20 address the array by byte: an address and a length */
static const IngenicT31Efuse ingenic_t10_efuse = {
    .ctrl_mask = 0x8fe30000, .addr_bits = 7, .len_bits = 2,
    .mirror = 0x200, .blank = true,
};

/*
 * Word 15 is the MXU licence: the first four MD5 state words XORed, from
 * the IV a5e95a16 b4f84b07 c3073cf8 d2162de9 over four copies of words
 * 0-2 and (~h << 16 | h), h the low half of word 4, zero-padded to 512
 * bytes. The vendor kernels deny unkeyed threads the MXU when it is
 * neither 0 nor that; 0x5f8040b7 is the model's chip ID's. A T20X, T30X
 * and T31X match theirs, a T21N misses by bit 24, T23 and T32 read 0.
 */
#define EFUSE_MXU_LICENCE   0x5f8040b7

/* The T20's window shows its grade and licence but not its chip ID */
static const IngenicT31Efuse ingenic_t20_efuse = {
    .ctrl_mask = 0x0fe70000, .addr_bits = 7, .len_bits = 3,
    .mirror = 0x238, .word = { [15] = EFUSE_MXU_LICENCE },
};

/* From the T21 on, the address is a word index */
static const IngenicT31Efuse ingenic_t21_efuse = {
    .ctrl_mask = 0x07e00000, .addr_bits = 6,
    .mirror = 0x200,
    .word = { [4] = 0x01000000, [15] = EFUSE_MXU_LICENCE ^ BIT(24) },
};

/* Word 5 is 0x40000000 on a T23DL, 0xc0000000 on a T23N */
static const IngenicT31Efuse ingenic_t23_efuse = {
    .ctrl_mask = 0x03e00000, .addr_bits = 5,
    .mirror = 0x200, .word = { [4] = 0x01000000, [5] = 0x40000000 },
};

static const IngenicT31Efuse ingenic_t30_efuse = {
    .ctrl_mask = 0x03e00000, .addr_bits = 5,
    .mirror = 0x200, .word = { [4] = 0x01000000, [15] = EFUSE_MXU_LICENCE },
};

/*
 * The T32 SPL clears 0x008, writes the word index, sets bit 0 and polls
 * 0x008 bit 0. A T32NQ read word 4 as 0x21030000 (secure boot on; with
 * 0x21000000 stock firmware prints "WARNING: scboot disabled") and words
 * 16-23 as the SHA-256 of its RSA public modulus. Device-specific fuses
 * come from efuse-security= and efuse-keyhash=, never from here.
 */
static const IngenicT31Efuse ingenic_t32_efuse = {
    .ctrl_mask = 0x0fe00000, .addr_bits = 7,
    .mirror = 0x200,
    .word = { [4] = 0x21000000, [5] = 0x40000000, [6] = 0x00000006,
              [12] = 0x00008888 },
};

static const IngenicT31Efuse *ingenic_t31_efuse(IngenicT31State *s)
{
    switch (extract32(s->harb0_cpuid, 12, 16)) {
    case 0x0005:        /* T10 */
        return &ingenic_t10_efuse;
    case 0x2000:        /* T20 */
        return &ingenic_t20_efuse;
    case 0x0021:        /* T21 */
        return &ingenic_t21_efuse;
    case 0x0023:        /* T23 */
        return &ingenic_t23_efuse;
    case 0x0030:        /* T30 */
    case 0x0031:        /* T31 */
        return &ingenic_t30_efuse;
    default:
        return &ingenic_t32_efuse;
    }
}

/* The fuse word mirrored at window offset @offset */
static uint32_t ingenic_t31_efuse_fuse(IngenicT31State *s, hwaddr offset)
{
    const IngenicT31Efuse *e = ingenic_t31_efuse(s);

    if (e->blank) {
        return 0;
    }

    switch (offset) {
    case EFUSE_SERIAL0:
        return 0x725C2516;
    case EFUSE_SERIAL1:
        return 0x94FE0281;
    case EFUSE_SERIAL2:
        return 0x12100080;
    case EFUSE_WORD13:
        return s->efuse_word13;
    case 0x210:
        return s->efuse_security ? s->efuse_security : e->word[4];
    case EFUSE_SUBREMARK:
        return 0x00000000;
    case EFUSE_T33_VARIANT: /* 0x21C */
        return s->efuse_t33_variant;
    case 0x21F:
        return 0x00000000;
    case EFUSE_SUBSOCTYPE1: /* 0x238 */
        return s->efuse_subsoctype1 ? s->efuse_subsoctype1 : 0x99991111;
    default:
        if (offset >= 0x240 && offset < 0x260) {
            return s->efuse_keyhash[(offset - 0x240) / 4];
        }
        return offset & 3 ? 0 : e->word[(offset - 0x200) / 4];
    }
}

/* Fuse word @idx; the array is 24 words */
static uint32_t ingenic_t31_efuse_word(IngenicT31State *s, uint32_t idx)
{
    return idx < 24 ? ingenic_t31_efuse_fuse(s, 0x200 + 4 * idx) : 0;
}

/*
 * What a read started from reg 0x000 puts in the data register: the word
 * indexed, or by byte the bytes from the address, little-endian; a read
 * of more than four leaves the first four.
 */
static uint32_t ingenic_t31_efuse_fetch(IngenicT31State *s)
{
    const IngenicT31Efuse *e = ingenic_t31_efuse(s);
    uint32_t addr = extract32(s->efuse_ctrl, 21, e->addr_bits);
    uint32_t len, data = 0;

    if (!e->len_bits) {
        return ingenic_t31_efuse_word(s, addr);
    }
    len = MIN(extract32(s->efuse_ctrl, 16, e->len_bits) + 1, 4);
    for (uint32_t i = 0; i < len; i++) {
        uint32_t a = addr + i;

        data |= extract32(ingenic_t31_efuse_word(s, a / 4), 8 * (a % 4), 8)
                << (8 * i);
    }
    return data;
}

static uint32_t ingenic_t31_efuse_value(IngenicT31State *s, hwaddr offset)
{
    const IngenicT31Efuse *e = ingenic_t31_efuse(s);

    if (offset >= 0x200 && offset < 0x260) {
        return offset >= e->mirror ? ingenic_t31_efuse_fuse(s, offset) : 0;
    }

    switch (offset) {
    case 0x000:
        return s->efuse_ctrl;
    case 0x008:
        /* Bits 15:8 show the top byte of the security word */
        return s->efuse_state |
               (ingenic_t31_efuse_fuse(s, 0x210) >> 16 & 0xff00);
    case 0x00c:
        return s->efuse_data;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: unhandled read at 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }
}

static uint64_t ingenic_t31_efuse_read(void *opaque, hwaddr offset,
                                       unsigned size)
{
    uint32_t value = ingenic_t31_efuse_value(opaque, offset);

    trace_ingenic_t31_efuse_read(offset, value);
    return value;
}

static void ingenic_t31_efuse_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size)
{
    IngenicT31State *s = opaque;

    trace_ingenic_t31_efuse_write(offset, value);
    switch (offset) {
    case 0x000:
        /* Bit 0 starts a read, done at once; bit 1 would blow fuses */
        if (value & 2) {
            qemu_log_mask(LOG_UNIMP, "%s: fuse programming\n", __func__);
        }
        s->efuse_ctrl = value & ingenic_t31_efuse(s)->ctrl_mask;
        if (value & 1) {
            s->efuse_data = ingenic_t31_efuse_fetch(s);
            s->efuse_state |= 1;
        }
        break;
    case 0x008:
        /* Writing 0 clears done; nothing sets it but a read */
        s->efuse_state &= value;
        break;
    }
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
    [INGENIC_T31_DEV_I2C2]      = 0x10052000,
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
    [INGENIC_T31_DEV_ISP]       = 0x13300000,

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
    /* SSI0/SSI1 are real device models, not stubbed */
    /* I2C0-I2C2 are real device models, not stubbed */
    { "ingenic-t31-usbphy", 0x10060000, 4 * KiB },
    /* DES is a real device model, but for the T33 */
    /* SADC is a real device model, not stubbed */
    /* OST is a real device model, not stubbed */
    /* HARB0 uses SoC ID stub */
    /* DDR PHY is a real device model, not stubbed */
    { "ingenic-t31-i2d",    0x13030000, 4 * KiB },
    /* The T23 4.4 kernel's vinit initcall stores a magic word at
     * 0x12221010 (outside every DT node) before registering the VPU;
     * a bus error there panics init. Nothing else lives in this 1 MiB. */
    { "ingenic-t23-vpu-ctl", 0x12200000, 1 * MiB },
    /*
     * T32 NPU register windows. tnpu_core_init touches 0x12700000 and
     * 0x12608000; unmapped, those raise a data bus error that oopses the
     * module's init and takes a userspace process down with it.
     */
    { "ingenic-t32-tnpu0",  0x12600000, 64 * KiB },
    { "ingenic-t32-tnpu1",  0x12700000, 64 * KiB },
    { "ingenic-t31-lcdc",   0x13050000, 4 * KiB },
    { "ingenic-t31-ipu",    0x13080000, 4 * KiB },
    /* DDRC is a real device model, not stubbed */
    { "ingenic-t31-isp",    0x13300000, 1024 * KiB },
    { "ingenic-t31-harb2",  0x13400000, 4 * KiB },
    { "ingenic-t31-nemc",   0x13410000, 4 * KiB },
    /* PDMA is a real device now, not unimplemented */
    /* AES is a real device model, not stubbed */
    /* SFC is a real device model, not stubbed */
    /* MSC0/MSC1 are real device models, not stubbed */
    /* HASH and RSA are real device models */
    /* GMAC is a real device model, not stubbed */
    { "ingenic-t31-otg",    0x13500000, 68 * KiB },
    /* EFUSE uses SoC variant stub */
};

/*
 * The families, by cpuid bits 27:12. The T32Pro and CW080, a T32's family
 * at revision 5, count as T33s: the vendor trees build them with its
 * peripherals.
 */
#define FAM_T10     (1 << 0)
#define FAM_T20     (1 << 1)
#define FAM_T21     (1 << 2)
#define FAM_T23     (1 << 3)
#define FAM_T30     (1 << 4)
#define FAM_T31     (1 << 5)
#define FAM_T32     (1 << 6)
#define FAM_T33     (1 << 7)
#define FAM_ALL     0xff

/* Windows only some families have, from the vendor kernels' soc/base.h */
static const struct {
    const char *name;
    hwaddr base;
    hwaddr size;
    uint32_t fams;
} ingenic_t31_family_unimp[] = {
    /* The video encoder (VPU, HELIX, AVPU or RADIX); TCSM and SRAM are RAM */
    { "ingenic-xb1-vpu",       0x13200000, 1 * MiB,    FAM_ALL },
    { "ingenic-xb1-radix",     0x13600000, 1 * MiB,    FAM_T30 },
    { "ingenic-t32-ivdc",      0x13100000, 960 * KiB,  FAM_T32 | FAM_T33 },
    { "ingenic-t32-ldc",       0x13040000, 64 * KiB,   FAM_T32 | FAM_T33 },
    { "ingenic-t32-monitor",   0x13090000, 64 * KiB,   FAM_T32 | FAM_T33 },
    { "ingenic-t30-ldc",       0x130a0000, 4 * KiB,    FAM_T30 },
    { "ingenic-t32-tnpu",      0x12500000, 64 * KiB,   FAM_T32 | FAM_T33 },
    { "ingenic-t32-fstram",    0x10004000, 4 * KiB,    FAM_T32 | FAM_T33 },
    /* Camera input: MIPI PHY and CSI receiver */
    { "ingenic-xb1-mipi-phy",  0x10022000, 4 * KiB,
      FAM_T23 | FAM_T30 | FAM_T31 | FAM_T32 | FAM_T33 },
    { "ingenic-xb1-mipi-csi",  0x10023000, 4 * KiB,    FAM_T32 | FAM_T33 },
    { "ingenic-xb1-mipi-csi",  0x10040000, 4 * KiB,
      FAM_T10 | FAM_T20 | FAM_T30 },
    /* DMIC */
    { "ingenic-xb1-dmic",      0x10022000, 4 * KiB,    FAM_T10 | FAM_T20 },
    { "ingenic-xb1-dmic",      0x10023000, 4 * KiB,    FAM_T21 | FAM_T30 },
    { "ingenic-t32-dmic",      0x134d0000, 64 * KiB,   FAM_T32 | FAM_T33 },
    /*
     * The T32/T33 SPI slave: not the T31's DesignWare one but a block of
     * Ingenic's own, undocumented (the T40/T41 have it at 0x10040000)
     */
    { "ingenic-t32-ssi-slv",   0x10044000, 4 * KiB,    FAM_T32 | FAM_T33 },
    { "ingenic-t33-des",       0x10061000, 4 * KiB,    FAM_T33 },
};

/*
 * A CW080 has the T32Pro's cpuid but the T33's ISP (its SDK firmware is
 * the T33's); the SDK U-Boot tells it by a type1 top byte under 0x10
 */
static bool ingenic_t31_cw080(const T31Variant *v)
{
    uint8_t ver = v->type1 >> 24;

    return v->cpuid == T32PRO_CPUID && ver && ver < 0x10;
}

static uint32_t ingenic_t31_family_bit(uint32_t cpuid)
{
    switch (extract32(cpuid, 12, 16)) {
    case 0x0005: return FAM_T10;
    case 0x2000: return FAM_T20;
    case 0x0021: return FAM_T21;
    case 0x0023: return FAM_T23;
    case 0x0030: return FAM_T30;
    case 0x0032: return cpuid == T32PRO_CPUID ? FAM_T33 : FAM_T32;
    case 0x0033: return FAM_T33;
    default:     return FAM_T31;
    }
}

static void ingenic_t31_init(Object *obj)
{
    IngenicT31State *s = INGENIC_T31(obj);

    s->memmap = ingenic_t31_memmap;

    object_initialize_child(obj, "cpm", &s->cpm, TYPE_INGENIC_T31_CPM);
    object_initialize_child(obj, "ddrc", &s->ddrc, TYPE_INGENIC_DDRC);
    object_initialize_child(obj, "sfc", &s->sfc, TYPE_INGENIC_T31_SFC);
    object_initialize_child(obj, "sfc1", &s->sfc1, TYPE_INGENIC_T31_SFC);
    object_initialize_child(obj, "gmac", &s->gmac, TYPE_INGENIC_GMAC);
    object_initialize_child(obj, "tcu", &s->tcu, TYPE_INGENIC_TCU);
    object_initialize_child(obj, "sysost", &s->sysost, TYPE_INGENIC_SYSOST);
    object_initialize_child(obj, "gpio", &s->gpio, TYPE_INGENIC_GPIO);
    object_initialize_child(obj, "sadc", &s->sadc, TYPE_INGENIC_SADC);
    object_initialize_child(obj, "intc", &s->intc, TYPE_INGENIC_INTC);
    object_initialize_child(obj, "i2c0", &s->i2c[0], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c1", &s->i2c[1], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "i2c2", &s->i2c[2], TYPE_INGENIC_I2C);
    object_initialize_child(obj, "ssi0", &s->ssi[0], TYPE_INGENIC_SSI);
    object_initialize_child(obj, "ssi1", &s->ssi[1], TYPE_INGENIC_SSI);
    object_initialize_child(obj, "ssi-slv", &s->ssi_slv,
                            TYPE_INGENIC_SSI_SLV);
    object_initialize_child(obj, "aes", &s->aes, TYPE_INGENIC_AES);
    object_initialize_child(obj, "des", &s->des, TYPE_INGENIC_DES);
    object_initialize_child(obj, "msc0", &s->msc[0], TYPE_INGENIC_MSC);
    object_initialize_child(obj, "msc1", &s->msc[1], TYPE_INGENIC_MSC);
    object_initialize_child(obj, "sdhci0", &s->sdhci[0], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "sdhci1", &s->sdhci[1], TYPE_SYSBUS_SDHCI);
    object_initialize_child(obj, "dwc2", &s->dwc2, TYPE_DWC2_USB);
    object_initialize_child(obj, "pdma", &s->pdma, TYPE_INGENIC_PDMA);
    object_initialize_child(obj, "rtc", &s->rtc, TYPE_INGENIC_RTC);
    object_initialize_child(obj, "dtrng", &s->dtrng, TYPE_INGENIC_DTRNG);
    object_initialize_child(obj, "riscv", &s->riscv, TYPE_INGENIC_RISCV);
    object_initialize_child(obj, "pwm", &s->pwm, TYPE_INGENIC_PWM);
    object_initialize_child(obj, "hash", &s->hash, TYPE_INGENIC_HASH);
    object_initialize_child(obj, "rsa", &s->rsa, TYPE_INGENIC_RSA);
    object_initialize_child(obj, "lzma0", &s->lzma[0], TYPE_INGENIC_LZMA);
    object_initialize_child(obj, "lzma1", &s->lzma[1], TYPE_INGENIC_LZMA);
    object_initialize_child(obj, "aic", &s->aic, TYPE_INGENIC_AIC);
    object_initialize_child(obj, "dmic", &s->dmic, TYPE_INGENIC_DMIC);
    object_initialize_child(obj, "as-dmic", &s->as_dmic, TYPE_INGENIC_AS_DMIC);
    object_initialize_child(obj, "avpu", &s->avpu, TYPE_INGENIC_AVPU);
    object_initialize_child(obj, "ipu", &s->ipu, TYPE_INGENIC_IPU);
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
        /* As the bootrom leaves it, after its own reads */
        s->efuse_state = 1;
        /*
         * Word 13 reads 0 on a T31X, and Zeratul SPLs power off on a zero
         * (Tassadar's, the T32's and T33's, halt), so the parts sold for
         * fast boot get one (its value unmeasured): on the T32 those whose
         * type the SPL accepts.
         */
        static const char *const fast_boot_parts[] = {
            "t31zl", "t31zx", "t31zc", "t31a", "t23zn", "t32zn",
            "t32prozl", "t32prozn", "t32prozx", "t32provl", "t32provn",
            "t32provx", "t32provnp", "t33zl", "t33zn",
        };

        for (i = 0; i < ARRAY_SIZE(fast_boot_parts); i++) {
            if (!s->efuse_word13 && !strcmp(v->name, fast_boot_parts[i])) {
                s->efuse_word13 = 1;
            }
        }
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

    /* T32/T33 SFC1: the flash given as -drive if=mtd,index=1; IRQ -> 13 */
    qdev_prop_set_uint8(DEVICE(&s->sfc1), "unit", 1);
    sysbus_realize(SYS_BUS_DEVICE(&s->sfc1), &error_fatal);
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;

        if (cpufam == 0x0032 || cpufam == 0x0033) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->sfc1), 0, 0x13490000);
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->sfc1), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 13));
        }
    }

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

    /* PDMA at 0x13420000, IRQ -> INTC source 10 (IRQ_PDMA) */
    sysbus_realize(SYS_BUS_DEVICE(&s->pdma), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->pdma), 0,
                    s->memmap[INGENIC_T31_DEV_PDMA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->pdma), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 10));

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
     * The RISC-V core: the T31's and T32's SRAM and control pages, the
     * T23's one control page over the VPU stub's. The T32's core counts at
     * its clock and interrupts the host on INTC source 4.
     */
    {
        uint32_t fam_id = extract32(s->harb0_cpuid, 12, 16);
        bool t32 = fam_id == 0x0032 || fam_id == 0x0033;

        if (fam_id == 0x0023) {
            qdev_prop_set_string(DEVICE(&s->riscv), "model", "t23");
        } else if (t32) {
            qdev_prop_set_string(DEVICE(&s->riscv), "model", "t32");
            qdev_connect_clock_in(DEVICE(&s->riscv), "count-clk",
                qdev_get_clock_out(DEVICE(&s->cpm), "riscv-clk"));
        }
        sysbus_realize(SYS_BUS_DEVICE(&s->riscv), &error_fatal);
        if (fam_id == 0x0023) {
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->riscv), 1,
                                    0x12220000, 1);
        } else if (fam_id == 0x0031 || t32) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->riscv), 0, 0x12210000);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->riscv), 1, 0x12220000);
            qdev_connect_gpio_out_named(DEVICE(&s->cpm), "riscv-clock", 0,
                qdev_get_gpio_in_named(DEVICE(&s->riscv), "clock", 0));
        }
        if (t32) {
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->riscv), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 4));
        }
    }

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

    /* HASH engine: IRQ -> INTC source 22; the T33 moves its data ports */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        qdev_prop_set_bit(DEVICE(&s->hash), "t33", fam & FAM_T33);
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->hash), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->hash), 0,
                    s->memmap[INGENIC_T31_DEV_HASH]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->hash), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 22));

    /* RSA accelerator: IRQ -> INTC source 24 */
    sysbus_realize(SYS_BUS_DEVICE(&s->rsa), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rsa), 0,
                    s->memmap[INGENIC_T31_DEV_RSA]);

    /*
     * GPIO: the T10 and T20 have six ports 0x100 apart, the T30 four and
     * the rest three, 0x1000 apart. Port N interrupts on INTC 17 - N.
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;
        if (cpufam == 0x0005 || cpufam == 0x2000) {
            qdev_prop_set_uint32(DEVICE(&s->gpio), "port-stride", 0x100);
            qdev_prop_set_uint32(DEVICE(&s->gpio), "nr-ports", 6);
        } else if (cpufam == 0x0030) {
            qdev_prop_set_uint32(DEVICE(&s->gpio), "nr-ports", 4);
        }
    }
    sysbus_realize(SYS_BUS_DEVICE(&s->gpio), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0,
                    s->memmap[INGENIC_T31_DEV_GPIO]);

    /* INTC: 64 sources -> MIPS IP2. Wired to CPU in board init. */
    sysbus_realize(SYS_BUS_DEVICE(&s->intc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->intc), 0,
                    s->memmap[INGENIC_T31_DEV_INTC]);
    for (unsigned n = 0; n < s->gpio.nr_ports; n++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), n,
                           qdev_get_gpio_in(DEVICE(&s->intc), 17 - n));
    }

    /*
     * I2C0 is INTC source 60 (bank 1 bit 28), I2C1 source 59 (bit 27).
     * I2C0's pads are PA12 (SDA) and PA13 (SCL) on every family, and the
     * T10 kernels bit-bang them as GPIOs.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[0]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[0]), 0,
                    s->memmap[INGENIC_T31_DEV_I2C0]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[0]), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 60));
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
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[1]), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[1]), 0,
                    s->memmap[INGENIC_T31_DEV_I2C1]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[1]), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 59));

    /*
     * I2C2 exists only on the T23/T30 (INTC source 58) and the T32/T33
     * (source 48); the T32 4.4 device tree probes it.
     */
    sysbus_realize(SYS_BUS_DEVICE(&s->i2c[2]), &error_fatal);
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;
        int irq = -1;

        if (cpufam == 0x0023 || cpufam == 0x0030) {
            irq = 58;
        } else if (cpufam == 0x0032 || cpufam == 0x0033) {
            irq = 48;
        }
        if (irq >= 0) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[2]), 0,
                            s->memmap[INGENIC_T31_DEV_I2C2]);
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[2]), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), irq));
        }
    }

    /*
     * SSI0 (INTC source 9) on every family, SSI1 (source 8) on the T21,
     * T30 and T31 only. The T32 shows its DMA requests in SR (the T33 is
     * taken to as well); the T33 driver waits on END via CR0.FIE.
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        for (i = 0; i < 2; i++) {
            DeviceState *d = DEVICE(&s->ssi[i]);

            qdev_prop_set_string(d, "bus-name", i ? "ssi1" : "ssi0");
            qdev_prop_set_bit(d, "dreq-status", fam & (FAM_T32 | FAM_T33));
            qdev_prop_set_bit(d, "fie", fam & FAM_T33);
            sysbus_realize(SYS_BUS_DEVICE(d), &error_fatal);
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->ssi[0]), 0,
                        s->memmap[INGENIC_T31_DEV_SSI0]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ssi[0]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 9));
        if (fam & (FAM_T21 | FAM_T30 | FAM_T31)) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->ssi[1]), 0,
                            s->memmap[INGENIC_T31_DEV_SSI1]);
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->ssi[1]), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 8));
        }

        /* The T31's SPI slave: INTC source 11, DMA requests 0x0e/0x0f */
        sysbus_realize(SYS_BUS_DEVICE(&s->ssi_slv), &error_fatal);
        if (fam & FAM_T31) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->ssi_slv), 0, 0x10040000);
            sysbus_connect_irq(SYS_BUS_DEVICE(&s->ssi_slv), 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 11));
            for (i = 0; i < 2; i++) {
                qdev_connect_gpio_out_named(DEVICE(&s->ssi_slv), "dreq", i,
                    qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq",
                                           0x0e + i));
            }
        }
    }

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
     * UARTs. INTC source numbers (from BSP arch/mips/xburst/soc-t31/
     * include/soc/irq.h): UART0=51, UART1=50, UART2=49. Wired so the tty
     * layer's IRQ-driven TX path completes - otherwise userspace
     * writes to /dev/console block forever in tty_write.
     */
    ingenic_uart_create(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART0],
                        qdev_get_gpio_in(DEVICE(&s->intc), 51), serial_hd(0),
                        "ingenic-t31-uart0", DEVICE(&s->pdma), 0);
    ingenic_uart_create(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART1],
                        qdev_get_gpio_in(DEVICE(&s->intc), 50), serial_hd(1),
                        "ingenic-t31-uart1", DEVICE(&s->pdma), 1);
    ingenic_uart_create(get_system_memory(), s->memmap[INGENIC_T31_DEV_UART2],
                        qdev_get_gpio_in(DEVICE(&s->intc), 49), serial_hd(2),
                        "ingenic-t31-uart2", DEVICE(&s->pdma), 2);

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

    if (s->harb0_cpuid == T32PRO_CPUID) {
        memory_region_init_io(&s->tnpu_id, OBJECT(dev),
                              &ingenic_t31_tnpu_id_ops, s,
                              "ingenic-t32pro-tnpu-id", 4);
        memory_region_add_subregion(get_system_memory(),
                                    0x12700000 + TNPU_UMPID, &s->tnpu_id);
        memory_region_init_ram(&s->tnpu_sram, OBJECT(dev),
                               "ingenic-t32pro-tnpu-sram", 8 * KiB,
                               &error_fatal);
        memory_region_add_subregion(get_system_memory(), 0x150b0000,
                                    &s->tnpu_sram);
    }

    /* MSC0/MSC1 - SD/MMC controllers.
     * T10-T31 use the old Ingenic MSC register interface at 0x13450000.
     * T32/T33 switched to standard SDHCI at 0x13060000/0x13070000, which
     * their mask ROM drives too.
     * Both device types must be realized (QEMU asserts all children are
     * realized), but only the active variant gets memory-mapped. */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t cpufam = v ? (v->cpuid >> 12) & 0xFFFF : 0x0031;
        bool use_sdhci = cpufam == 0x0032 || cpufam == 0x0033;

        /* the controllers in use own the names msc0 and msc1 */
        for (int j = 0; use_sdhci && j < 2; j++) {
            g_autofree char *name = g_strdup_printf("msc%d", j);

            object_property_set_str(OBJECT(&s->msc[j]), "bus-name", "sd-bus",
                                    &error_abort);
            object_property_set_str(OBJECT(&s->sdhci[j]), "bus-name", name,
                                    &error_abort);
        }
        sysbus_realize(SYS_BUS_DEVICE(&s->msc[0]), &error_fatal);
        sysbus_realize(SYS_BUS_DEVICE(&s->msc[1]), &error_fatal);
        /* MSC0 -> INTC source 37, MSC1 -> 36 (XBurst1 T-series) */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->msc[0]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 37));
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->msc[1]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 36));
        for (int j = 0; j < 2; j++) {
            object_property_set_uint(OBJECT(&s->sdhci[j]),
                                     "sd-spec-version", 3, &error_abort);
            /*
             * The Ingenic drivers read the write-protect bit inverted
             * (SDHCI_QUIRK_INVERTED_WRITE_PROTECT) and replace these caps
             * with their own, which select ADMA2; QEMU only runs the DMA
             * modes capareg advertises.
             */
            object_property_set_bool(OBJECT(&s->sdhci[j]),
                                     "wp-inverted", true, &error_abort);
            object_property_set_uint(OBJECT(&s->sdhci[j]),
                                     "capareg",
                                     (1ULL << 26) |  /* 3.3V */
                                     (1ULL << 25) |  /* 3.0V */
                                     (1ULL << 24) |  /* 1.8V */
                                     (1ULL << 22) |  /* SDMA */
                                     (1ULL << 21) |  /* high-speed */
                                     (1ULL << 19) |  /* ADMA2 */
                                     (200 << 8),     /* base clock 200 MHz */
                                     &error_abort);
            sysbus_realize(SYS_BUS_DEVICE(&s->sdhci[j]), &error_fatal);
        }

        /*
         * T32/T33 drive the standard SDHCI blocks, which sit on the same
         * interrupt lines as the legacy MSC controllers: MSC0 -> INTC 37,
         * MSC1 -> INTC 36. These were realized and mapped but never wired,
         * so the controller delivered no interrupt at all and every command
         * ended in "Timeout waiting for hardware interrupt" after 10 s -
         * the card never finished enumerating and mmc_rescan span forever.
         */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdhci[0]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 37));
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdhci[1]), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 36));

        if (use_sdhci) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[0]), 0, 0x13060000);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->sdhci[1]), 0, 0x13070000);
        } else {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[0]),
                            0, s->memmap[INGENIC_T31_DEV_MSC0]);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->msc[1]),
                            0, s->memmap[INGENIC_T31_DEV_MSC1]);
        }
    }

    /*
     * The LZMA decompressors: the T32's and T33's two, whose vendor SPL
     * reads the first's version and unpacks U-Boot with it (Tassadar's
     * unpacks the kernel and the RAM disk on both at once), and the T23's
     * one, over the SRAM's first page, with the T41's registers: Zeratul's
     * SPL unpacks the kernel and its RAM disk with it.
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        for (i = 0; i < 2; i++) {
            qdev_prop_set_string(DEVICE(&s->lzma[i]), "model",
                                 fam & FAM_T23 ? "t41" : "t32");
            sysbus_realize(SYS_BUS_DEVICE(&s->lzma[i]), &error_fatal);
        }
        if (fam & (FAM_T32 | FAM_T33)) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->lzma[0]), 0, 0x131f0000);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->lzma[1]), 0, 0x131f8000);
        } else if (fam & FAM_T23) {
            sysbus_mmio_map_overlap(SYS_BUS_DEVICE(&s->lzma[0]), 0,
                                    0x132f0000, 1);
        }
    }

    /*
     * The AIC (INTC source 1) and the internal codec, at the same addresses
     * on every XBurst1 SoC; the codecs differ. DMA requests as the PDMA
     * numbers them: TX 0x06, RX 0x07 and, where the AIC has its TFIFO loop
     * for echo cancelling, 0x05 (the T10, T20 and T30 put I2S1 there).
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        static const uint8_t aic_drt[INGENIC_AIC_NR_DREQ] = {
            [INGENIC_AIC_DREQ_TX] = 0x06,
            [INGENIC_AIC_DREQ_RX] = 0x07,
            [INGENIC_AIC_DREQ_LOOP] = 0x05,
        };
        const char *codec = fam & (FAM_T10 | FAM_T20 | FAM_T21 | FAM_T30)
                            ? "t10" : fam & FAM_T23 ? "t23"
                            : fam & (FAM_T32 | FAM_T33) ? "t32" : "t31";

        qdev_prop_set_string(DEVICE(&s->aic), "codec", codec);
        object_property_set_link(OBJECT(&s->aic), "pdma", OBJECT(&s->pdma),
                                 &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->aic), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 0,
                        s->memmap[INGENIC_T31_DEV_AIC]);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->aic), 1,
                        s->memmap[INGENIC_T31_DEV_CODEC]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->aic), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 1));
        for (i = 0; i < INGENIC_AIC_NR_DREQ; i++) {
            if (i == INGENIC_AIC_DREQ_LOOP &&
                (fam & (FAM_T10 | FAM_T20 | FAM_T30))) {
                continue;
            }
            qdev_connect_gpio_out_named(DEVICE(&s->aic), "dreq", i,
                qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq",
                                       aic_drt[i]));
        }
    }

    /*
     * The DMIC: the T30's at 0x10023000 (INTC source 40, DMA request 0x21),
     * the T31's at 0x10034000 (12, 0x0d); the T32 and T33 have the
     * AudioSystem's (INTC source 33). No kernel drives the T10's or T20's.
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        SysBusDevice *dmic = SYS_BUS_DEVICE(&s->dmic);
        SysBusDevice *as_dmic = SYS_BUS_DEVICE(&s->as_dmic);
        bool t30 = fam & FAM_T30;

        object_property_set_link(OBJECT(dmic), "pdma", OBJECT(&s->pdma),
                                 &error_abort);
        sysbus_realize(dmic, &error_fatal);
        sysbus_realize(as_dmic, &error_fatal);
        if (fam & (FAM_T30 | FAM_T31)) {
            sysbus_mmio_map(dmic, 0, t30 ? 0x10023000
                                         : s->memmap[INGENIC_T31_DEV_DMIC]);
            sysbus_connect_irq(dmic, 0, qdev_get_gpio_in(DEVICE(&s->intc),
                                                         t30 ? 40 : 12));
            qdev_connect_gpio_out_named(DEVICE(dmic), "dreq", 0,
                qdev_get_gpio_in_named(DEVICE(&s->pdma), "dreq",
                                       t30 ? 0x21 : 0x0d));
        }
        if (fam & (FAM_T32 | FAM_T33)) {
            sysbus_mmio_map(as_dmic, 0, 0x134d0000);
            sysbus_connect_irq(as_dmic, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 33));
        }
    }

    /*
     * The T21's, T23's, T31's and T32's ISP and VIC (INTC sources 29 and
     * 30): frames from the image sensor out to the tx-isp driver's buffers
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        SysBusDevice *isp;

        object_initialize_child(OBJECT(s), "isp", &s->isp,
                                fam & FAM_T21 ? TYPE_INGENIC_T21_ISP
                                : fam & FAM_T23 ? TYPE_INGENIC_T23_ISP
                                : fam & FAM_T32 ? TYPE_INGENIC_T32_ISP
                                : v && v->cpuid == T32PRO_CPUID &&
                                  !ingenic_t31_cw080(v)
                                ? TYPE_INGENIC_T32PRO_ISP
                                : fam & FAM_T33 ? TYPE_INGENIC_T33_ISP
                                : TYPE_INGENIC_T31_ISP);
        isp = SYS_BUS_DEVICE(&s->isp);
        qdev_prop_set_bit(DEVICE(isp), "mipi-switch", s->mipi_switch[0] >= 0);
        sysbus_realize(isp, &error_fatal);
        /*
         * A board's MIPI switch: two or three sensors on the one CSI
         * receiver, the GPIOs choosing whose lanes it sees
         */
        for (int sw = 0; sw < ARRAY_SIZE(s->mipi_switch); sw++) {
            if (s->mipi_switch[sw] < 0) {
                continue;
            }
            if (s->mipi_switch[sw] >=
                object_property_get_uint(OBJECT(&s->gpio), "nr-ports",
                                         &error_abort) * 32) {
                error_setg(errp, "mipi-switch: no GPIO %d",
                           s->mipi_switch[sw]);
                return;
            }
            qdev_connect_gpio_out_named(DEVICE(&s->gpio), "pad",
                                        s->mipi_switch[sw],
                qdev_get_gpio_in_named(DEVICE(isp), "mipi-switch", sw));
        }
        if (fam & (FAM_T32 | FAM_T33)) {
            sysbus_mmio_map(isp, 0, s->memmap[INGENIC_T31_DEV_ISP]);
            /* The RISC-V core sees the ISP's and VIC's lines too */
            for (int line = 0; line < 2; line++) {
                DeviceState *split;

                object_initialize_child(OBJECT(s), "isp-irq[*]",
                                        &s->isp_irq[line], TYPE_SPLIT_IRQ);
                split = DEVICE(&s->isp_irq[line]);
                qdev_prop_set_uint16(split, "num-lines", 2);
                qdev_realize(split, NULL, &error_fatal);
                qdev_connect_gpio_out(split, 0,
                    qdev_get_gpio_in(DEVICE(&s->intc), 29 + line));
                qdev_connect_gpio_out(split, 1,
                    qdev_get_gpio_in_named(DEVICE(&s->riscv), "soc-irq",
                                           29 + line));
                sysbus_connect_irq(isp, line, qdev_get_gpio_in(split, 0));
            }
        } else if (fam & (FAM_T21 | FAM_T23 | FAM_T31)) {
            sysbus_mmio_map(isp, 0, s->memmap[INGENIC_T31_DEV_ISP]);
            sysbus_connect_irq(isp, 0, qdev_get_gpio_in(DEVICE(&s->intc), 29));
            sysbus_connect_irq(isp, 1, qdev_get_gpio_in(DEVICE(&s->intc), 30));
        }
        if (fam & (FAM_T21 | FAM_T23 | FAM_T31 | FAM_T32 | FAM_T33)) {
            /* CLKGR0 bit 23 gates it (19 on the T32/T33), SRBC 22 resets it */
            qdev_connect_gpio_out_named(DEVICE(&s->cpm), "isp-clock", 0,
                qdev_get_gpio_in_named(DEVICE(isp), "clock", 0));
            qdev_connect_gpio_out_named(DEVICE(&s->cpm), "isp-reset", 0,
                qdev_get_gpio_in_named(DEVICE(isp), "reset", 0));
        }
        if (fam & (FAM_T21 | FAM_T23 | FAM_T31 | FAM_T32 | FAM_T33)) {
            /* the sensor's MCLK, which tx-isp stops as it closes */
            qdev_connect_gpio_out_named(DEVICE(&s->cpm), "cim-clock", 0,
                qdev_get_gpio_in_named(DEVICE(isp), "mclk", 0));
        }
        if (fam & (FAM_T23 | FAM_T31 | FAM_T32 | FAM_T33)) {
            object_initialize_child(OBJECT(s), "csi", &s->csi,
                                    TYPE_INGENIC_MIPI_CSI);
            sysbus_realize(SYS_BUS_DEVICE(&s->csi), &error_fatal);
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->csi), 0,
                            fam & (FAM_T32 | FAM_T33) ? 0x10054000
                                                      : 0x10023000);
            qdev_connect_gpio_out_named(DEVICE(&s->csi), "reset", 0,
                qdev_get_gpio_in_named(DEVICE(isp), "csi-reset", 0));
        }
    }

    /*
     * The T10's, T20's and T30's ISP (an Apical core) and VIC, on INTC
     * source 29; the T30's with its noise reducer at 0x13090000, ahead of
     * its multi-scaler at 0x130b0000 (INTC source 19)
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        SysBusDevice *isp, *msc;

        if (fam & (FAM_T10 | FAM_T20 | FAM_T30)) {
            object_initialize_child(OBJECT(s), "t20-isp", &s->t20_isp,
                                    fam & FAM_T10 ? TYPE_INGENIC_T10_ISP
                                    : fam & FAM_T30 ? TYPE_INGENIC_T30_ISP
                                    : TYPE_INGENIC_T20_ISP);
            isp = SYS_BUS_DEVICE(&s->t20_isp);
            sysbus_realize(isp, &error_fatal);
            sysbus_mmio_map(isp, 0, s->memmap[INGENIC_T31_DEV_ISP]);
            sysbus_connect_irq(isp, 0, qdev_get_gpio_in(DEVICE(&s->intc), 29));
        }
        if (fam & FAM_T30) {
            sysbus_mmio_map(isp, 1, 0x13090000);
            object_initialize_child(OBJECT(s), "mscaler", &s->mscaler,
                                    TYPE_INGENIC_T30_MSCALER);
            msc = SYS_BUS_DEVICE(&s->mscaler);
            sysbus_realize(msc, &error_fatal);
            sysbus_mmio_map(msc, 0, 0x130b0000);
            sysbus_connect_irq(msc, 0, qdev_get_gpio_in(DEVICE(&s->intc), 19));
            qdev_connect_gpio_out_named(DEVICE(isp), "frame-out", 0,
                                        qdev_get_gpio_in_named(DEVICE(msc),
                                                               "frame", 0));
        }
    }

    /* The T31's AVPU, the Allegro encoder (INTC source 62, IRQ_HELIX0) */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        SysBusDevice *avpu = SYS_BUS_DEVICE(&s->avpu);

        sysbus_realize(avpu, &error_fatal);
        if (fam & FAM_T31) {
            sysbus_mmio_map(avpu, 0, 0x13200000);
            sysbus_connect_irq(avpu, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 62));
        }
    }

    /*
     * The HELIX encoder: the T23's at 0x13100000 (INTC source 40), the
     * T10's, T20's, T21's and T30's at 0x13200000 (62)
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        SysBusDevice *helix;

        object_initialize_child(OBJECT(s), "helix", &s->helix,
                                fam & FAM_T10 ? TYPE_INGENIC_T10_HELIX
                                : fam & FAM_T21 ? TYPE_INGENIC_T21_HELIX
                                : fam & (FAM_T20 | FAM_T30)
                                ? TYPE_INGENIC_T30_HELIX : TYPE_INGENIC_HELIX);
        helix = SYS_BUS_DEVICE(&s->helix);
        if (fam & FAM_T23) {
            object_property_set_link(OBJECT(helix), "isp", OBJECT(&s->isp),
                                     &error_abort);
        }
        sysbus_realize(helix, &error_fatal);
        qdev_connect_gpio_out_named(DEVICE(&s->cpm), "vpu-reset", 0,
                                    qdev_get_gpio_in_named(DEVICE(helix),
                                                           "reset", 0));
        if (fam & FAM_T23) {
            sysbus_mmio_map(helix, 0, 0x13100000);
            sysbus_connect_irq(helix, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 40));
        } else if (fam & (FAM_T10 | FAM_T20 | FAM_T21 | FAM_T30)) {
            /* under the TCSM and SRAM its window holds */
            sysbus_mmio_map_overlap(helix, 0, 0x13200000, -1);
            sysbus_connect_irq(helix, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 62));
        }
    }

    /* The T30's RADIX, its H.265 encoder (INTC source 45) */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        if (fam & FAM_T30) {
            SysBusDevice *radix;

            object_initialize_child(OBJECT(s), "radix", &s->radix,
                                    TYPE_INGENIC_RADIX);
            radix = SYS_BUS_DEVICE(&s->radix);
            sysbus_realize(radix, &error_fatal);
            sysbus_mmio_map(radix, 0, 0x13600000);
            sysbus_connect_irq(radix, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 45));
            qdev_connect_gpio_out_named(DEVICE(&s->cpm), "radix-reset", 0,
                                        qdev_get_gpio_in_named(DEVICE(radix),
                                                               "reset", 0));
        }
    }

    /*
     * The T32's HERA (INTC source 62) and the JPEG engine at its module
     * 0x1e (63), above the SRAM there
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        if (fam & (FAM_T32 | FAM_T33)) {
            SysBusDevice *hera, *jpeg;

            object_initialize_child(OBJECT(s), "hera", &s->hera,
                                    TYPE_INGENIC_HERA);
            hera = SYS_BUS_DEVICE(&s->hera);
            object_property_set_link(OBJECT(hera), "isp", OBJECT(&s->isp),
                                     &error_abort);
            sysbus_realize(hera, &error_fatal);
            sysbus_mmio_map(hera, 0, 0x13200000);
            sysbus_connect_irq(hera, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 62));
            object_initialize_child(OBJECT(s), "jpeg", &s->jpeg,
                                    TYPE_INGENIC_JPEG);
            jpeg = SYS_BUS_DEVICE(&s->jpeg);
            object_property_set_link(OBJECT(jpeg), "isp", OBJECT(&s->isp),
                                     &error_abort);
            sysbus_realize(jpeg, &error_fatal);
            sysbus_mmio_map_overlap(jpeg, 0, 0x132f0000, 1);
            sysbus_connect_irq(jpeg, 0,
                               qdev_get_gpio_in(DEVICE(&s->intc), 63));
        }
    }

    /*
     * The IVDC, which in direct mode passes the ISP's channel 0 straight to
     * the encoder: the T23's at 0x13200000 (INTC source 38), the T32's and
     * T33's at 0x13100000 (57), its registers over the start of the window
     * the vendor kernels give it
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        if (fam & (FAM_T23 | FAM_T32 | FAM_T33)) {
            bool t23 = fam & FAM_T23;
            SysBusDevice *ivdc;

            object_initialize_child(OBJECT(s), "ivdc", &s->ivdc,
                                    TYPE_INGENIC_IVDC);
            ivdc = SYS_BUS_DEVICE(&s->ivdc);
            sysbus_realize(ivdc, &error_fatal);
            sysbus_mmio_map(ivdc, 0, t23 ? 0x13200000 : 0x13100000);
            sysbus_connect_irq(ivdc, 0, qdev_get_gpio_in(DEVICE(&s->intc),
                                                         t23 ? 38 : 57));
            for (unsigned n = 0; n < INGENIC_IVDC_SENSORS; n++) {
                qdev_connect_gpio_out_named(DEVICE(&s->isp), "ivdc-frame", n,
                                            qdev_get_gpio_in_named(
                                                DEVICE(ivdc), "frame", n));
            }
        }
    }

    /* The IPU, which libimp draws the OSD with (INTC source 6) */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);
        SysBusDevice *ipu = SYS_BUS_DEVICE(&s->ipu);

        sysbus_realize(ipu, &error_fatal);
        if (fam & (FAM_T10 | FAM_T20 | FAM_T21 | FAM_T23 | FAM_T30 |
                   FAM_T31 | FAM_T32 | FAM_T33)) {
            sysbus_mmio_map(ipu, 0, s->memmap[INGENIC_T31_DEV_IPU]);
            sysbus_connect_irq(ipu, 0, qdev_get_gpio_in(DEVICE(&s->intc), 6));
        }
    }

    /*
     * AES (INTC source 23), with EDIN on the T32 and the T33's own register
     * layout and EDOUT; DES everywhere but the T33, whose vendor tree drops
     * it (it has no interrupt the drivers use).
     */
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        qdev_prop_set_bit(DEVICE(&s->aes), "edin", fam & (FAM_T32 | FAM_T33));
        qdev_prop_set_bit(DEVICE(&s->aes), "edout", fam & FAM_T33);
        qdev_prop_set_bit(DEVICE(&s->aes), "t33", fam & FAM_T33);
        sysbus_realize(SYS_BUS_DEVICE(&s->aes), &error_fatal);
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->aes), 0,
                        s->memmap[INGENIC_T31_DEV_AES]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->aes), 0,
                           qdev_get_gpio_in(DEVICE(&s->intc), 23));
        sysbus_realize(SYS_BUS_DEVICE(&s->des), &error_fatal);
        if (!(fam & FAM_T33)) {
            sysbus_mmio_map(SYS_BUS_DEVICE(&s->des), 0,
                            s->memmap[INGENIC_T31_DEV_DES]);
        }
    }

    sysbus_realize(SYS_BUS_DEVICE(&s->sadc), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->sadc), 0,
                    s->memmap[INGENIC_T31_DEV_SADC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sadc), 0,
                       qdev_get_gpio_in(DEVICE(&s->intc), 18));

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

    /* Unimplemented device stubs */
    for (i = 0; i < ARRAY_SIZE(ingenic_t31_unimp); i++) {
        create_unimplemented_device(ingenic_t31_unimp[i].name,
                                    ingenic_t31_unimp[i].base,
                                    ingenic_t31_unimp[i].size);
    }
    {
        const T31Variant *v = (const T31Variant *)s->variant;
        uint32_t fam = ingenic_t31_family_bit(v ? v->cpuid : 0x10031003);

        for (i = 0; i < ARRAY_SIZE(ingenic_t31_family_unimp); i++) {
            if (ingenic_t31_family_unimp[i].fams & fam) {
                create_unimplemented_device(ingenic_t31_family_unimp[i].name,
                                            ingenic_t31_family_unimp[i].base,
                                            ingenic_t31_family_unimp[i].size);
            }
        }
    }
}

static const Property ingenic_t31_props[] = {
    DEFINE_PROP_STRING("soc-variant", IngenicT31State, soc_variant),
    DEFINE_PROP_UINT32("efuse-security", IngenicT31State, efuse_security, 0),
    DEFINE_PROP_UINT32("efuse-word13", IngenicT31State, efuse_word13, 0),
    DEFINE_PROP_STRING("efuse-keyhash", IngenicT31State, efuse_keyhash_hex),
    DEFINE_PROP_INT32("mipi-switch", IngenicT31State, mipi_switch[0], -1),
    DEFINE_PROP_INT32("mipi-switch2", IngenicT31State, mipi_switch[1], -1),
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
