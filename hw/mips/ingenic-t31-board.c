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
#include "target/mips/cpu.h"
#include "exec/cpu-common.h"
#include "system/address-spaces.h"
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

    qdev_realize(DEVICE(s), NULL, &error_fatal);

    /* SDRAM */
    memory_region_add_subregion(get_system_memory(),
                                s->memmap[INGENIC_T31_DEV_SDRAM],
                                machine->ram);

    /* Load firmware via -kernel */
    if (machine->kernel_filename) {
        uint64_t entry;
        int64_t size;

        size = load_elf(machine->kernel_filename, NULL,
                        cpu_mips_kseg0_to_phys, NULL,
                        &entry, NULL, NULL, NULL,
                        ELFDATA2LSB, EM_MIPS, 1, 0);
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
        {
            hwaddr phys = cpu_mips_kseg0_to_phys(NULL, entry);
            uint32_t *p = rom_ptr(phys, 4);
            if (p && le32_to_cpu(*p) == 0x03040506) {
                entry += 0x800;
            }
        }

        cpu->env.active_tc.PC = (int32_t)entry;
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
}

DEFINE_MACHINE("ingenic-t31", ingenic_t31_machine_init)
