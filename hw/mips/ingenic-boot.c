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
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "hw/core/loader.h"
#include "hw/mips/ingenic-boot.h"
#include "system/block-backend.h"
#include <zlib.h>

/* thingino's flash layout puts the U-Boot environment here */
#define ENV_OFFSET      0x50000
#define ENV_SIZE        (64 * KiB)

/* The low RAM KSEG0 reaches; argv goes at its top, out of mem= */
#define LOW_RAM_MAX     (256 * MiB)
#define ARGV_AREA       (8 * KiB)

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
