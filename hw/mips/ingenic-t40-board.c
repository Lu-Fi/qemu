/*
 * Ingenic T40/T41 board/machine emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "exec/tb-flush.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/mips/ingenic-t40.h"
#include "hw/net/ingenic-gmac.h"
#include "hw/sd/sd.h"
#include "system/block-backend.h"
#include "hw/core/qdev-properties.h"
#include "target/mips/cpu.h"
#include "system/physmem.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "elf.h"

static void ingenic_t40_uimage_post_reset(void *opaque)
{
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    uint8_t lcr = 0x03;
    physical_memory_write(base + 0x0c, &lcr, 1);
}

typedef struct {
    MIPSCPU *cpu;
    uint32_t pc;
    uint32_t sp;
    bool is_uimage;
    bool is_flash;
    BlockBackend *blk;
    uint32_t spl_total;
    hwaddr spl_phys;
    uint32_t argv[2];
    uint32_t sram_size;
    hwaddr sram_phys;
} T40BootResetCtx;

static void ingenic_t40_cpu_reset(void *opaque)
{
    T40BootResetCtx *b = opaque;
    CPUMIPSState *env = &b->cpu->env;

    /*
     * A fresh boot rewrites RAM wholesale; translation blocks from the
     * previous run otherwise linger on those pages and every guest
     * store pays a tb_page_remove list walk (measured at 95% host CPU
     * after a guest reboot). Boot with an empty translation cache.
     */
    queue_tb_flush(CPU(b->cpu));

    if (b->is_flash && b->blk) {
        uint8_t *spl_data = g_malloc(b->spl_total);
        if (blk_pread(b->blk, 0, b->spl_total, spl_data, 0) >= 0) {
            physical_memory_write(b->spl_phys, spl_data, b->spl_total);
        }
        g_free(spl_data);
    }

    env->active_tc.PC = (int32_t)b->pc;
    env->active_tc.gpr[29] = (int32_t)b->sp;

    if (b->is_uimage) {
        env->active_tc.gpr[4] = 2;
        env->active_tc.gpr[5] = (int32_t)b->argv[0];
        env->active_tc.gpr[6] = 0;
        env->active_tc.gpr[7] = 0;
    }

    /* Re-arm the boot SRAM alias for the SPL (also on guest reboot) */
    env->sram_alias_size = b->sram_size;
    env->sram_alias_phys = b->sram_phys;
}

static void ingenic_t40_board_init(MachineState *machine)
{
    IngenicT40State *s;
    Clock *cpuclk;
    int num_cpus = machine->smp.cpus;

    s = INGENIC_T40(object_new(TYPE_INGENIC_T40));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_unref(OBJECT(s));

    if (!s->soc_variant || !s->soc_variant[0]) {
        const char *mname = MACHINE_GET_CLASS(machine)->name;
        if (strstr(mname, "t41")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t41nq");
        } else {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t40n");
        }
    }

    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 800000000);

    if (num_cpus < 1) num_cpus = 1;
    if (num_cpus > 2) num_cpus = 2;
    s->num_cpus = num_cpus;

    for (int i = 0; i < num_cpus; i++) {
        s->cpu[i] = mips_cpu_create_with_clock(machine->cpu_type,
                                                cpuclk, false);
        cpu_mips_irq_init_cpu(s->cpu[i]);
        cpu_mips_clock_init(s->cpu[i]);
        if (i > 0) {
            CPU(s->cpu[i])->halted = 1;
        }
    }

    qdev_realize(DEVICE(s), NULL, &error_fatal);

    for (int i = 0; i < num_cpus; i++) {
        /* Core OST raises the CCU OST input; the CCU routes it to IP4. */
        s->cost_irq[i] = qdev_get_gpio_in_named(DEVICE(&s->ccu),
                                                "ost-in", i);
        /* INTC core <i> output -> CCU peripheral input <i>. */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), i,
                           qdev_get_gpio_in_named(DEVICE(&s->ccu),
                                                  "intc-in", i));
        /* CCU per-core interrupt outputs -> MIPS IP2/IP3/IP4. */
        qdev_connect_gpio_out_named(DEVICE(&s->ccu), "irq-ip2", i,
                                    s->cpu[i]->env.irq[2]);
        qdev_connect_gpio_out_named(DEVICE(&s->ccu), "irq-ip3", i,
                                    s->cpu[i]->env.irq[3]);
        qdev_connect_gpio_out_named(DEVICE(&s->ccu), "irq-ip4", i,
                                    s->cpu[i]->env.irq[4]);
    }

    /* Attach SD/MMC cards to whichever controller is mapped */
    for (int i = 0; i < 2; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
        BusState *bus;

        if (!blk) {
            continue;
        }
        if (machine->firmware) {
            bus = qdev_get_child_bus(DEVICE(&s->msc[i]), "sd-bus");
        } else {
            bus = qdev_get_child_bus(DEVICE(&s->sdhci[i]), "sd-bus");
        }
        if (!bus) {
            error_report("ingenic-t40: msc%d sd-bus not found", i);
            exit(1);
        }
        DeviceState *card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
        qdev_realize_and_unref(card, bus, &error_fatal);
    }

    /* SDRAM */
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T40_DEV_SDRAM],
                                machine->ram);

    if (machine->firmware) {
        /*
         * Bootrom analysis mode (-bios): run the real mask-ROM image
         * from 0xBFC00000.  The SoC realize already loaded it into the
         * bootrom region.  Attach SD/NOR as needed; the ROM drives
         * boot-device selection from GPIO.
         */
        s->cpu[0]->env.active_tc.PC = (int32_t)0xbfc00000;
    } else if (machine->kernel_filename) {
        uint64_t entry;
        int64_t size;
        bool is_uimage = false;

        size = load_elf(machine->kernel_filename, NULL,
                        cpu_mips_kseg0_to_phys, NULL,
                        &entry, NULL, NULL, NULL,
                        ELFDATA2LSB, EM_MIPS, 1, 0);
        if (size < 0) {
            hwaddr ep = 0, la = LOAD_UIMAGE_LOADADDR_INVALID;
            int is_linux = 0;
            size = load_uimage(machine->kernel_filename, &ep, &la,
                               &is_linux, cpu_mips_kseg0_to_phys, NULL);
            if (size > 0) {
                entry = ep;
                is_uimage = true;
            }
        }
        if (size < 0) {
            size = load_image_targphys(machine->kernel_filename,
                                       s->memmap[INGENIC_T40_DEV_SDRAM],
                                       machine->ram_size, NULL);
            entry = s->memmap[INGENIC_T40_DEV_SDRAM];
        }
        if (size < 0) {
            error_report("could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }

        if (!is_uimage) {
            hwaddr phys = cpu_mips_kseg0_to_phys(NULL, entry);
            uint32_t *p = rom_ptr(phys, 4);
            if (p && le32_to_cpu(*p) == 0x03040506) {
                entry += 0x800;
            }
        }

        s->cpu[0]->env.active_tc.PC = (int32_t)entry;

        if (is_uimage) {
            qemu_register_reset(ingenic_t40_uimage_post_reset,
                                (void *)(uintptr_t)
                                s->memmap[INGENIC_T40_DEV_UART1]);

            const char *cmdline = machine->kernel_cmdline ?
                machine->kernel_cmdline : "";
            const char *prog = "linux";
            hwaddr cmdline_phys = 0x07f00000;
            hwaddr prog_phys = 0x07f00800;
            hwaddr argv_phys = 0x07f01000;
            uint32_t argv_ents[2];
            size_t cmdlen = strlen(cmdline);

            if (cmdlen + 1 < 0x800 &&
                cmdline_phys + cmdlen + 1 < machine->ram_size) {
                physical_memory_write(cmdline_phys, cmdline, cmdlen + 1);
            }
            physical_memory_write(prog_phys, prog, strlen(prog) + 1);

            argv_ents[0] = (uint32_t)prog_phys | 0x80000000;
            argv_ents[1] = (uint32_t)cmdline_phys | 0x80000000;
            physical_memory_write(argv_phys, argv_ents, sizeof(argv_ents));

            s->cpu[0]->env.active_tc.gpr[4] = 2;
            s->cpu[0]->env.active_tc.gpr[5] =
                (int32_t)(argv_phys | 0x80000000);
            s->cpu[0]->env.active_tc.gpr[6] = 0;
            s->cpu[0]->env.active_tc.gpr[7] = 0;
        }
    } else {
        /* Boot from flash image */
        DriveInfo *di = drive_get(IF_MTD, 0, 0);
        if (!di) {
            di = drive_get(IF_PFLASH, 0, 0);
        }
        if (!di) {
            di = drive_get(IF_NONE, 0, 0);
        }
        if (!di) {
            error_report("no -kernel and no flash drive; nothing to boot");
            exit(1);
        }

        BlockBackend *blk = blk_by_legacy_dinfo(di);
        uint8_t header[16];
        if (blk_pread(blk, 0, sizeof(header), header, 0) < 0) {
            error_report("failed to read flash header");
            exit(1);
        }

        uint32_t magic = ldl_le_p(&header[0]);
        if (magic != 0x03040506) {
            error_report("flash missing Ingenic SPL header (got 0x%08x)",
                         magic);
            exit(1);
        }

        uint32_t spl_size = ldl_le_p(&header[12]);
        if (spl_size == 0 || spl_size > 256 * KiB) {
            error_report("SPL size %u out of range", spl_size);
            exit(1);
        }

        uint32_t total = 0x800 + spl_size;
        g_autofree uint8_t *spl_data = g_malloc(total);
        if (blk_pread(blk, 0, total, spl_data, 0) < 0) {
            error_report("failed to read SPL from flash");
            exit(1);
        }

        /*
         * Load the SPL into the boot SRAM. It is linked at 0x80001000
         * and runs from low kseg0; the SRAM alias redirects that to the
         * SRAM, so the SPL's DDR memory test cannot overwrite its own
         * code (on real silicon the SPL runs from on-chip SRAM).
         */
        hwaddr spl_phys = s->memmap[INGENIC_T40_DEV_SRAM] + 0x1000;
        physical_memory_write(spl_phys, spl_data, total);
        s->cpu[0]->env.active_tc.PC = (int32_t)0x80001800;
    }

    /* Stack to top of SRAM */
    s->cpu[0]->env.active_tc.gpr[29] =
        (int32_t)(s->memmap[INGENIC_T40_DEV_SRAM] +
                  INGENIC_T40_SRAM_SIZE + 0x80000000);

    {
        T40BootResetCtx *b = g_new0(T40BootResetCtx, 1);
        b->cpu = s->cpu[0];
        b->pc = (uint32_t)s->cpu[0]->env.active_tc.PC;
        b->sp = (uint32_t)s->cpu[0]->env.active_tc.gpr[29];
        if (machine->kernel_filename) {
            b->is_uimage = (s->cpu[0]->env.active_tc.gpr[4] == 2);
            b->argv[0] = (uint32_t)s->cpu[0]->env.active_tc.gpr[5];
        } else {
            b->sram_size = INGENIC_T40_SRAM_SIZE;
            b->sram_phys = s->memmap[INGENIC_T40_DEV_SRAM];
            b->is_flash = true;
            DriveInfo *di = drive_get(IF_MTD, 0, 0);
            if (!di) di = drive_get(IF_PFLASH, 0, 0);
            if (!di) di = drive_get(IF_NONE, 0, 0);
            if (di) {
                b->blk = blk_by_legacy_dinfo(di);
                uint8_t hdr[16];
                if (blk_pread(b->blk, 0, sizeof(hdr), hdr, 0) >= 0) {
                    b->spl_total = 0x800 + ldl_le_p(&hdr[12]);
                }
                b->spl_phys = s->memmap[INGENIC_T40_DEV_SRAM] + 0x1000;
            }
        }
        qemu_register_reset(ingenic_t40_cpu_reset, b);
    }
}

static void ingenic_t40_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T40 (XBurst2 MIPS32r2, dual-core)";
    mc->init = ingenic_t40_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurst2");
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "ingenic-t40.sdram";
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t40", ingenic_t40_machine_init)

static void ingenic_t41_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T41 (XBurst2 MIPS32r2, dual-core)";
    mc->init = ingenic_t40_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurst2");
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "ingenic-t40.sdram";
    mc->max_cpus = 2;
    mc->default_cpus = 2;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t41", ingenic_t41_machine_init)
