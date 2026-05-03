/*
 * Ingenic T31 board/machine emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/loader.h"
#include "hw/mips/ingenic-t31.h"
#include "hw/net/ingenic-t31-gmac.h"
#include "hw/sd/sd.h"
#include "system/block-backend.h"
#include "hw/core/qdev-properties.h"
#include "target/mips/cpu.h"
#include "exec/cpu-common.h"
#include "system/address-spaces.h"
#include "system/blockdev.h"
#include "elf.h"

static void ingenic_t31_board_init(MachineState *machine)
{
    IngenicT31State *s;
    MIPSCPU *cpu;
    Clock *cpuclk;

    /* Create SoC */
    s = INGENIC_T31(object_new(TYPE_INGENIC_T31));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_unref(OBJECT(s));

    /* CPU clock */
    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    clock_set_hz(cpuclk, 1000000000);

    /* CPU */
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, false);
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);

    /* Connect GMAC NIC to netdev before realize */
    {
        NetClientState *nc = qemu_find_netdev("n0");
        if (nc) {
            qdev_prop_set_netdev(DEVICE(&s->gmac), "netdev", nc);
        }
    }

    qdev_realize(DEVICE(s), NULL, &error_fatal);

    /* Attach SD/MMC cards to MSC0 and MSC1 if drives are provided. */
    for (int i = 0; i < 2; i++) {
        DriveInfo *di = drive_get(IF_SD, 0, i);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
        DeviceState *card;
        BusState *bus;

        if (!blk) {
            continue;
        }
        bus = qdev_get_child_bus(DEVICE(&s->msc[i]), "sd-bus");
        if (!bus) {
            error_report("ingenic-t31: msc%d sd-bus not found", i);
            exit(1);
        }
        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
        qdev_realize_and_unref(card, bus, &error_fatal);
    }

    /* SDRAM */
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_SDRAM],
                                machine->ram);

    /* Give GMAC direct RAM access for DMA descriptor handling */
    s->gmac.ram_ptr = memory_region_get_ram_ptr(machine->ram);
    s->gmac.ram_size = machine->ram_size;

    /* Load firmware via -kernel. Three formats supported, in order:
     *  1. ELF — used for SPL / U-Boot.
     *  2. uImage — used to skip U-Boot and boot Linux directly. Entry
     *     point and load address come from the image header.
     *  3. Raw binary — loaded at SDRAM base.
     */
    if (machine->kernel_filename) {
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
                                       s->memmap[INGENIC_T31_DEV_SDRAM],
                                       machine->ram_size, NULL);
            entry = s->memmap[INGENIC_T31_DEV_SDRAM];
        }
        if (size < 0) {
            error_report("could not load kernel '%s'",
                         machine->kernel_filename);
            exit(1);
        }

        /*
         * Ingenic SPL images have a 2 KiB header starting with magic
         * 0x03040506. The bootrom skips this header and begins
         * execution at entry + 0x800.
         */
        if (!is_uimage) {
            hwaddr phys = cpu_mips_kseg0_to_phys(NULL, entry);
            uint32_t *p = rom_ptr(phys, 4);
            if (p && le32_to_cpu(*p) == 0x03040506) {
                entry += 0x800;
            }
        }

        cpu->env.active_tc.PC = (int32_t)entry;

        /*
         * For uImage (Linux) boot, the Ingenic BSP kernel expects
         * cmdline in $a3 / argv-style at $a0/$a1. Pass the user's
         * -append string in a fixed location and point a1 at it.
         */
        if (is_uimage) {
            const char *cmdline = machine->kernel_cmdline ?
                machine->kernel_cmdline : "";
            hwaddr cmdline_phys = 0x07f00000; /* end-of-RAM scratch */
            hwaddr argv_phys = 0x07f01000;
            uint32_t argv0_kseg0 = (uint32_t)cmdline_phys | 0x80000000;
            size_t cmdlen = strlen(cmdline);

            if (cmdlen + 1 < 0x1000 && cmdline_phys + cmdlen + 1 <
                                       machine->ram_size) {
                cpu_physical_memory_write(cmdline_phys, cmdline, cmdlen + 1);
            }
            cpu_physical_memory_write(argv_phys, &argv0_kseg0, 4);

            cpu->env.active_tc.gpr[4] = 1; /* argc: just the cmdline */
            cpu->env.active_tc.gpr[5] = (int32_t)(argv_phys | 0x80000000);
            cpu->env.active_tc.gpr[6] = 0; /* envp */
            cpu->env.active_tc.gpr[7] = 0;
        }

        /* SPL expects the bootrom to set up the stack in TCSM */
        cpu->env.active_tc.gpr[29] =
            (int32_t)(s->memmap[INGENIC_T31_DEV_TCSM] +
                      INGENIC_T31_TCSM_SIZE + 0x80000000);
    }
}

static void ingenic_t31_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T31 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_T31_GMAC;
}

DEFINE_MACHINE("ingenic-t31", ingenic_t31_machine_init)
