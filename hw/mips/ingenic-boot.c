/*
 * Ingenic boards: a direct Linux boot handed over as thingino's U-Boot does
 *
 * The kernel gets argc 2 and argv { "linux", command line } in a0/a1. The
 * command line is -append's; without one, what the flash's U-Boot
 * environment builds (its bootcmd's "setenv bootargs", variables expanded);
 * without that, thingino's defaults for the RAM size. An -initrd goes at
 * the top of the kernel's memory (the command line's mem=...@0), with
 * rd_start and rd_size appended, which the kernel parses itself: thingino's
 * U-Boot passes none, as it builds with the legacy MIPS command line.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "hw/core/loader.h"
#include "hw/mips/ingenic-boot.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/reset.h"
#include "system/system.h"
#include <zlib.h>

/* thingino's flash layout puts the U-Boot environment here */
#define ENV_OFFSET      0x50000
#define ENV_SIZE        (64 * KiB)

/* The low RAM KSEG0 reaches; argv goes at its top, out of mem= */
#define LOW_RAM_MAX     (256 * MiB)
#define ARGV_AREA       (8 * KiB)

/*
 * What each part's bootrom writes to the GPIO as it reads an SPL from NOR,
 * in its order, as its own run in the model shows: PC0 and PC1, the boot
 * select pins, as inputs with their pulls, then the SFC's pins in their
 * function with theirs (the T23's PA23, 24, 27 and 28 and PB15 and 16, the
 * T32's in function 0, the A1's PC20-25). All through the set and clear
 * ports, so the other pins keep their reset state.
 */
#define PXINT   0x10
#define PXMSK   0x20
#define PXPAT1  0x30
#define PXPAT0  0x40
#define PXPEN   0x60
#define PXPUEN  0x110
#define PXPDEN  0x120
#define SET     4
#define CLR     8

const IngenicRomGpio ingenic_rom_gpio_t10[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPEN + CLR, 3 },
    { 0, PXINT + CLR, 0x1f800000 }, { 0, PXMSK + CLR, 0x1f800000 },
    { 0, PXPAT1 + CLR, 0x1f800000 }, { 0, PXPAT0 + SET, 0x1f800000 },
    { 0, PXPEN + CLR, 0x1f800000 }, { 0xff }
};
const IngenicRomGpio ingenic_rom_gpio_t21[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPUEN + SET, 9 }, { 2, PXPUEN + CLR, 6 },
    { 0, PXINT + CLR, 0x1f800000 }, { 0, PXMSK + CLR, 0x1f800000 },
    { 0, PXPAT1 + CLR, 0x1f800000 }, { 0, PXPAT0 + SET, 0x1f800000 },
    { 0, PXPDEN + SET, 0x01554000 }, { 0, PXPDEN + CLR, 0x02aa8000 }, { 0xff }
};
const IngenicRomGpio ingenic_rom_gpio_t23[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPUEN + SET, 1 }, { 2, PXPDEN + CLR, 1 },
    { 2, PXPUEN + CLR, 2 }, { 2, PXPDEN + SET, 2 },
    { 0, PXINT + CLR, 0x19800000 }, { 0, PXMSK + CLR, 0x19800000 },
    { 0, PXPAT1 + CLR, 0x19800000 }, { 0, PXPAT0 + SET, 0x19800000 },
    { 0, PXPUEN + SET, 0x19800000 }, { 0, PXPDEN + CLR, 0x19800000 },
    { 1, PXINT + CLR, 0x00018000 }, { 1, PXMSK + CLR, 0x00018000 },
    { 1, PXPAT1 + SET, 0x00018000 }, { 1, PXPAT0 + CLR, 0x00018000 },
    { 1, PXPUEN + SET, 0x00018000 }, { 1, PXPDEN + CLR, 0x00018000 }, { 0xff }
};
const IngenicRomGpio ingenic_rom_gpio_t30[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPUEN + SET, 9 },
    { 0, PXINT + CLR, 0x1f800000 }, { 0, PXMSK + CLR, 0x1f800000 },
    { 0, PXPAT1 + CLR, 0x1f800000 }, { 0, PXPAT0 + SET, 0x1f800000 },
    { 0, PXPDEN + CLR, 0x03ffc000 }, { 0xff }
};
const IngenicRomGpio ingenic_rom_gpio_t31[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPUEN + SET, 1 }, { 2, PXPDEN + CLR, 1 },
    { 2, PXPUEN + CLR, 2 }, { 2, PXPDEN + SET, 2 },
    { 0, PXINT + CLR, 0x1f800000 }, { 0, PXMSK + CLR, 0x1f800000 },
    { 0, PXPAT1 + CLR, 0x1f800000 }, { 0, PXPAT0 + SET, 0x1f800000 },
    { 0, PXPUEN + SET, 0x1f800000 }, { 0, PXPDEN + CLR, 0x1f800000 }, { 0xff }
};
const IngenicRomGpio ingenic_rom_gpio_t32[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPUEN + SET, 1 }, { 2, PXPDEN + CLR, 1 },
    { 2, PXPUEN + CLR, 2 }, { 2, PXPDEN + SET, 2 },
    { 0, PXINT + CLR, 0x1f800000 }, { 0, PXMSK + CLR, 0x1f800000 },
    { 0, PXPAT1 + CLR, 0x1f800000 }, { 0, PXPAT0 + CLR, 0x1f800000 },
    { 0, PXPUEN + SET, 0x1f800000 }, { 0, PXPDEN + CLR, 0x1f800000 }, { 0xff }
};
const IngenicRomGpio ingenic_rom_gpio_a1[] = {
    { 2, PXINT + CLR, 3 }, { 2, PXMSK + SET, 3 }, { 2, PXPAT1 + SET, 3 },
    { 2, PXPAT0 + CLR, 3 }, { 2, PXPUEN + SET, 1 }, { 2, PXPDEN + CLR, 1 },
    { 2, PXPUEN + CLR, 2 }, { 2, PXPDEN + SET, 2 },
    { 2, PXINT + CLR, 0x03f00000 }, { 2, PXMSK + CLR, 0x03f00000 },
    { 2, PXPAT1 + CLR, 0x03f00000 }, { 2, PXPAT0 + CLR, 0x03f00000 },
    { 2, PXPUEN + SET, 0x02400000 }, { 2, PXPDEN + CLR, 0x02400000 },
    { 2, PXPUEN + CLR, 0x01b00000 }, { 2, PXPDEN + CLR, 0x01b00000 }, { 0xff }
};

typedef struct {
    Notifier done;
    hwaddr gpio, sfc;
    uint32_t stride, spl_size;
    const IngenicRomGpio *g;
} RomFlashLeft;

/*
 * After each reset the SFC reads as its last read of the SPL left it: one
 * device, 4-byte addresses off, TRAN_LEN the SPL's size, its clock gate
 * open
 */
static void rom_flash_left_reset(void *opaque)
{
    RomFlashLeft *r = opaque;

    for (const IngenicRomGpio *g = r->g; g->port != 0xff; g++) {
        address_space_stl_le(&address_space_memory,
                             r->gpio + g->port * r->stride + g->reg, g->value,
                             MEMTXATTRS_UNSPECIFIED, NULL);
    }
    address_space_stl_le(&address_space_memory, r->sfc + 0x00, 0x00000f8f,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_le(&address_space_memory, r->sfc + 0x04, 0x00000007,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_le(&address_space_memory, r->sfc + 0x14, 0x19010003,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_le(&address_space_memory, r->sfc + 0x2c, r->spl_size,
                         MEMTXATTRS_UNSPECIFIED, NULL);
    address_space_stl_le(&address_space_memory, r->sfc + 0x78, 0,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

/* The devices join the reset only once the machine is done: come after */
static void rom_flash_left_done(Notifier *n, void *data)
{
    qemu_register_reset(rom_flash_left_reset,
                        container_of(n, RomFlashLeft, done));
}

void ingenic_boot_rom_flash_left(hwaddr gpio, uint32_t stride,
                                 const IngenicRomGpio *g, hwaddr sfc,
                                 uint32_t spl_size)
{
    RomFlashLeft *r = g_new0(RomFlashLeft, 1);

    *r = (RomFlashLeft) {
        .done.notify = rom_flash_left_done, .gpio = gpio, .sfc = sfc,
        .stride = stride, .spl_size = spl_size, .g = g,
    };
    qemu_add_machine_init_done_notifier(&r->done);
}

#define BOOTROM_BASE    0x1fc00000
#define BANK0_BASE      0x1c000000
#define BANK0_SIZE      (64 * MiB)

/* Bank 0 reads the bootrom again every ROM size; a write is dropped */
static uint64_t bank0_read(void *opaque, hwaddr addr, unsigned size)
{
    MemoryRegion *rom = opaque;

    return ldn_le_p((uint8_t *)memory_region_get_ram_ptr(rom) +
                    addr % memory_region_size(rom), size);
}

static void bank0_write(void *opaque, hwaddr addr, uint64_t val,
                        unsigned size)
{
}

static const MemoryRegionOps bank0_ops = {
    .read = bank0_read,
    .write = bank0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

bool ingenic_bootrom_init(MemoryRegion *rom, Object *owner, const char *name,
                          uint64_t size, bool bank0, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());

    memory_region_init_rom(rom, owner, name, size, &error_abort);
    memory_region_add_subregion(get_system_memory(), BOOTROM_BASE, rom);
    if (bank0) {
        MemoryRegion *alias = g_new0(MemoryRegion, 1);

        memory_region_init_io(alias, owner, &bank0_ops, rom, "ingenic.bank0",
                              BANK0_SIZE);
        memory_region_add_subregion_overlap(get_system_memory(), BANK0_BASE,
                                            alias, -1);
    }
    if (ms && ms->firmware) {
        g_autofree char *data = NULL;
        g_autoptr(GError) err = NULL;
        gsize len;

        /* A longer dump, read on past the ROM, gives its first copy */
        if (!g_file_get_contents(ms->firmware, &data, &len, &err)) {
            error_setg(errp, "ingenic: failed to load -bios '%s': %s",
                       ms->firmware, err->message);
            return false;
        }
        memcpy(memory_region_get_ram_ptr(rom), data, MIN(len, size));
    } else {
        /*
         * ERET at each exception vector, so that a stray exception (MMIO
         * to an unmodelled device) returns rather than loops
         */
        static const uint32_t eret = 0x42000018;

        for (hwaddr v = 0; v < 0x400; v += 0x80) {
            rom_add_blob_fixed("ingenic.eret", &eret, 4, BOOTROM_BASE + v);
        }
    }
    return true;
}

static GHashTable *env_read(BlockBackend *blk)
{
    g_autofree uint8_t *buf = g_malloc(ENV_SIZE);
    const char *p = (const char *)buf + 4, *end = (const char *)buf + ENV_SIZE;
    GHashTable *env;

    if (!blk || blk_getlength(blk) < ENV_OFFSET + ENV_SIZE ||
        blk_pread(blk, ENV_OFFSET, ENV_SIZE, buf, 0) < 0 ||
        (uint32_t)ldl_le_p(buf) != (uint32_t)crc32(0, buf + 4, ENV_SIZE - 4)) {
        return NULL;
    }
    env = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);
    while (p < end && *p) {
        size_t len = strnlen(p, end - p);
        const char *eq = memchr(p, '=', len);

        if (eq) {
            g_hash_table_insert(env, g_strndup(p, eq - p),
                                g_strndup(eq + 1, p + len - eq - 1));
        }
        p += len + 1;
    }
    return env;
}

/*
 * What bootcmd's "setenv bootargs ..." leaves in bootargs: ${name} expanded,
 * the words joined by single spaces. A plain bootargs variable otherwise.
 */
static char *env_bootargs(GHashTable *env)
{
    const char *cmd = g_hash_table_lookup(env, "bootcmd");
    const char *set = cmd ? strstr(cmd, "setenv bootargs ") : NULL;
    const char *p = set ? set + strlen("setenv bootargs ") :
                    g_hash_table_lookup(env, "bootargs");
    g_autoptr(GString) out = g_string_new(NULL);
    g_auto(GStrv) words = NULL;

    if (!p) {
        return NULL;
    }
    for (; *p && !(set && *p == ';'); p++) {
        const char *close = p[0] == '$' && p[1] == '{' ? strchr(p, '}') : NULL;

        if (close) {
            g_autofree char *name = g_strndup(p + 2, close - p - 2);
            const char *val = g_hash_table_lookup(env, name);

            g_string_append(out, val ?: "");
            p = close;
        } else {
            g_string_append_c(out, *p);
        }
    }
    words = g_strsplit_set(out->str, " \t", -1);
    g_string_truncate(out, 0);
    for (int i = 0; words[i]; i++) {
        if (*words[i]) {
            g_string_append_printf(out, "%s%s", out->len ? " " : "", words[i]);
        }
    }
    return out->len ? g_strdup(out->str) : NULL;
}

/* thingino's Kconfig: rmem by RAM size, osmem below it from 0 */
static char *default_cmdline(uint64_t ram_size, const IngenicBootDefaults *d)
{
    unsigned ram = ram_size / MiB;
    unsigned rmem = !d->camera ? 0 : ram == 32 ? 10 : ram == 64 ? 23 :
                    ram == 128 ? 29 : ram == 256 ? 64 : 23;
    unsigned ispmem = d->ispmem ? 8 : 0;
    unsigned osmem = ram - rmem - ispmem;
    GString *s = g_string_new(NULL);

    g_string_printf(s, "mem=%uM@0x0 rmem=%uM@0x%x", osmem, rmem,
                    (unsigned)((osmem + ispmem) * MiB));
    if (ispmem) {
        g_string_append_printf(s, " ispmem=%uM@0x%x", ispmem,
                               (unsigned)(osmem * MiB));
    }
    g_string_append(s, " console=ttyS1,115200n8 panic=2");
    return g_string_free(s, false);
}

/* The top of the kernel's memory at 0: the command line's mem=size@0 */
static uint64_t cmdline_mem_top(const char *cmdline, uint64_t fallback)
{
    for (const char *p = cmdline; (p = strstr(p, "mem=")); p += 4) {
        const char *end, *after;
        uint64_t size, start = 0;

        if (p != cmdline && p[-1] != ' ') {
            continue;           /* rmem=, ispmem=, nmem= */
        }
        if (qemu_strtosz(p + 4, &end, &size) < 0) {
            continue;
        }
        if (*end == '@' && qemu_strtou64(end + 1, &after, 0, &start) < 0) {
            continue;
        }
        if (start == 0) {
            return MIN(size, fallback);
        }
    }
    return fallback;
}

void ingenic_boot_linux(MachineState *machine, MIPSCPU *cpu,
                        BlockBackend *flash, const IngenicBootDefaults *d,
                        hwaddr kernel_end)
{
    uint64_t low = MIN(machine->ram_size, LOW_RAM_MAX);
    hwaddr argv_phys = low - ARGV_AREA;
    g_autofree uint8_t *area = g_malloc0(ARGV_AREA);
    g_autofree char *cmdline = NULL;
    size_t len;

    if (machine->kernel_cmdline && *machine->kernel_cmdline) {
        cmdline = g_strdup(machine->kernel_cmdline);
    } else {
        g_autoptr(GHashTable) env = env_read(flash);

        cmdline = env ? env_bootargs(env) : NULL;
        if (!cmdline) {
            cmdline = default_cmdline(machine->ram_size, d);
        }
    }

    if (machine->initrd_filename) {
        int64_t size = get_image_size(machine->initrd_filename, NULL);
        uint64_t top = MIN(cmdline_mem_top(cmdline, low), argv_phys);
        hwaddr start = size > 0 && size < top ?
                       QEMU_ALIGN_DOWN(top - size, 64 * KiB) : 0;
        char *full;

        if (start < kernel_end ||
            load_image_targphys(machine->initrd_filename, start, size,
                                NULL) != size) {
            error_report("could not load initrd '%s' between the kernel and "
                         "0x%" PRIx64, machine->initrd_filename, top);
            exit(1);
        }
        full = g_strdup_printf("%s rd_start=0x%08" HWADDR_PRIx
                               " rd_size=0x%" PRIx64, cmdline,
                               start | 0x80000000, size);
        g_free(cmdline);
        cmdline = full;
    }

    /* argv[2], then "linux", then the command line */
    len = MIN(strlen(cmdline), ARGV_AREA - 17);
    if (len < strlen(cmdline)) {
        warn_report("kernel command line cut to %zu bytes", len);
    }
    stl_le_p(area, (argv_phys + 8) | 0x80000000);
    stl_le_p(area + 4, (argv_phys + 16) | 0x80000000);
    memcpy(area + 8, "linux", 6);
    memcpy(area + 16, cmdline, len);
    rom_add_blob_fixed("ingenic.argv", area, 16 + len + 1, argv_phys);

    cpu->env.active_tc.gpr[4] = 2;
    cpu->env.active_tc.gpr[5] = (int32_t)(argv_phys | 0x80000000);
    cpu->env.active_tc.gpr[6] = 0;
    cpu->env.active_tc.gpr[7] = 0;
}

/*
 * Above the RAM, as far as its window, the DDR controller answers: a
 * T23DL's (32 MB), T31X's and T40N's (128 MB) read 0 there up to 0x0ff00000
 * and drop writes, with no bus error and no mirror of the RAM. A vendor SPL
 * probes there to size the RAM (a T23DL's at 0x03f18000).
 */
static uint64_t ingenic_ddr_hole_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    return 0;
}

static void ingenic_ddr_hole_write(void *opaque, hwaddr addr, uint64_t val,
                                   unsigned size)
{
}

static const MemoryRegionOps ingenic_ddr_hole_ops = {
    .read = ingenic_ddr_hole_read,
    .write = ingenic_ddr_hole_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 8 },
};

void ingenic_ddr_hole(Object *owner, hwaddr base, uint64_t ram_size,
                      uint64_t window)
{
    MemoryRegion *mr;

    if (ram_size >= window) {
        return;
    }
    mr = g_new(MemoryRegion, 1);
    memory_region_init_io(mr, owner, &ingenic_ddr_hole_ops, NULL,
                          "ingenic-ddr-hole", window - ram_size);
    /* below any device mapped there */
    memory_region_add_subregion_overlap(get_system_memory(), base + ram_size,
                                        mr, -1);
}
