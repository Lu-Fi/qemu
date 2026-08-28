/*
 * Ingenic T31 board/machine emulation
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
#include "hw/mips/ingenic-t31.h"
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

static void ingenic_t31_uimage_post_reset(void *opaque)
{
    /*
     * Pre-initialize UART1 LCR so the BSP kernel's check_uart() in
     * serial.c finds a "configured" UART. The check is `if (lcr)`,
     * any non-zero value satisfies it. Real hardware boots through
     * U-Boot which leaves LCR set; direct -kernel uImage boot does
     * not, so check_uart finds LCR=0 everywhere and silently routes
     * earlyprintk to a dummy putchar.
     */
    hwaddr base = (hwaddr)(uintptr_t)opaque;
    uint8_t lcr = 0x03;

    physical_memory_write(base + 0x0c, &lcr, 1);
}

/*
 * Boot state captured at machine init and re-applied on every CPU reset
 * so that 'reset' (U-Boot) and 'reboot' (Linux) actually re-enter the
 * loaded firmware. Without this, after a guest reset the MIPS CPU lands
 * at 0xBFC00000 (bootrom stub) and just executes ERETs into garbage.
 */
typedef struct {
    MIPSCPU *cpu;
    uint32_t pc;
    uint32_t sp;
    bool is_uimage;
    bool is_flash;
    BlockBackend *blk;
    uint32_t spl_total;
    hwaddr spl_phys;
    uint32_t argv[2]; /* argv0=prog, argv1=cmdline (KSEG0 ptrs) */
} BootResetCtx;

static void ingenic_t31_cpu_reset(void *opaque)
{
    BootResetCtx *b = opaque;
    CPUMIPSState *env = &b->cpu->env;

    /*
     * The reset can arrive while this CPU sleeps in 'wait' (a watchdog
     * reset from idle). Devices get reset either way, but a halted vCPU
     * would keep the new PC and never run again: reset the CPU itself,
     * which also clears halted.
     */
    cpu_reset(CPU(b->cpu));

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
}

static void ingenic_t31_board_init(MachineState *machine)
{
    IngenicT31State *s;
    MIPSCPU *cpu;
    Clock *cpuclk;

    /* Create SoC */
    s = INGENIC_T31(object_new(TYPE_INGENIC_T31));
    object_property_add_child(OBJECT(machine), "soc", OBJECT(s));
    object_unref(OBJECT(s));

    /*
     * If the user didn't specify a variant via -global, derive a
     * default from the machine type so -M ingenic-t20 picks t20n
     * and -M ingenic-t10 picks t10 automatically.
     */
    if (!s->soc_variant || !s->soc_variant[0]) {
        const char *mname = MACHINE_GET_CLASS(machine)->name;
        if (strstr(mname, "t10")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t10");
        } else if (strstr(mname, "t33")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t33n");
        } else if (strstr(mname, "t32")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t32nq");
        } else if (strstr(mname, "t30")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t30n");
        } else if (strstr(mname, "t23")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t23n");
        } else if (strstr(mname, "t21")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t21n");
        } else if (strstr(mname, "t20")) {
            qdev_prop_set_string(DEVICE(s), "soc-variant", "t20n");
        }
    }

    /* CPU clock; also drives the CP0 Count/PerfCnt rate (the bootrom's
     * timer_poll timebase). NB: the real CPU runs at the 24 MHz EXTAL before
     * the SPL programs the PLL, so the bootrom's microsecond timeouts expire
     * ~40x faster here than on silicon; harmless because QEMU's devices respond
     * synchronously (the timeout windows still bound truly-stuck poll loops). */
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

    /*
     * The standalone System OST drives the BSP kernel tick via
     * MIPS hardware interrupt IP4 (CAUSEF_IP4 in irq.c).
     * cpu_mips_irq_init_cpu allocates env.irq[0..7] for IP0..IP7.
     */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sysost), 0, cpu->env.irq[4]);

    /* INTC drives MIPS IP2 (peripheral interrupts). */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 0, cpu->env.irq[2]);

    /* Attach SD/MMC cards.
     * T32/T33 use SDHCI, older SoCs use the Ingenic MSC interface. */
    {
        const char *mname = MACHINE_GET_CLASS(machine)->name;
        bool is_sdhci = !machine->firmware &&
                        (strstr(mname, "t32") || strstr(mname, "t33"));

        for (int i = 0; i < 2; i++) {
            DriveInfo *di = drive_get(IF_SD, 0, i);
            BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
            DeviceState *card;
            BusState *bus;

            if (!blk) {
                continue;
            }
            if (is_sdhci) {
                bus = qdev_get_child_bus(DEVICE(&s->sdhci[i]), "sd-bus");
            } else {
                bus = qdev_get_child_bus(DEVICE(&s->msc[i]), "sd-bus");
            }
            if (!bus) {
                error_report("ingenic-t31: msc%d sd-bus not found", i);
                exit(1);
            }
            /*
             * Default to an SD card. Setting INGENIC_EMMC in the environment
             * attaches an eMMC instead, so the bootrom's MMC (vs SD) init path
             * can be exercised for analysis. eMMC and SD-card both derive from
             * sdmmc-common and bind to the same sd-bus, so only the type
             * string differs.
             */
            card = qdev_new(getenv("INGENIC_EMMC") ? TYPE_EMMC : TYPE_SD_CARD);
            qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
            qdev_realize_and_unref(card, bus, &error_fatal);
        }
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
     *
     * If no -kernel is given, boot from flash: parse the Ingenic SPL
     * header at flash offset 0 and load the SPL into SDRAM at its
     * linked address (0x80001000).
     */
    if (machine->firmware) {
        /*
         * Bootrom analysis mode (-bios): the SoC has loaded the real
         * 32 KiB mask-ROM image over the bootrom region at 0x1FC00000.
         * Start the CPU at the reset vector like real silicon and let
         * the ROM drive CPU/clock/boot-device bring-up itself. No flash
         * or -kernel is required. (-bios takes precedence over -kernel.)
         */
        cpu->env.active_tc.PC = (int32_t)0xbfc00000;
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

        if (is_uimage) {
            qemu_register_reset(ingenic_t31_uimage_post_reset,
                                (void *)(uintptr_t)
                                s->memmap[INGENIC_T31_DEV_UART1]);
        }

        if (is_uimage) {
            const char *cmdline = machine->kernel_cmdline ?
                machine->kernel_cmdline : "";
            const char *prog = "linux";
            hwaddr cmdline_phys = 0x07f00000; /* end-of-RAM scratch */
            hwaddr prog_phys = 0x07f00800;
            hwaddr argv_phys = 0x07f01000;
            uint32_t argv_ents[2];
            size_t cmdlen = strlen(cmdline);

            if (cmdlen + 1 < 0x800 && cmdline_phys + cmdlen + 1 <
                                       machine->ram_size) {
                physical_memory_write(cmdline_phys, cmdline, cmdlen + 1);
            }
            physical_memory_write(prog_phys, prog, strlen(prog) + 1);

            argv_ents[0] = (uint32_t)prog_phys | 0x80000000;
            argv_ents[1] = (uint32_t)cmdline_phys | 0x80000000;
            physical_memory_write(argv_phys, argv_ents, sizeof(argv_ents));

            cpu->env.active_tc.gpr[4] = 2; /* argc: prog + cmdline */
            cpu->env.active_tc.gpr[5] = (int32_t)(argv_phys | 0x80000000);
            cpu->env.active_tc.gpr[6] = 0; /* envp */
            cpu->env.active_tc.gpr[7] = 0;
        }
    } else {
        /*
         * No -kernel: boot from flash image.
         *
         * The Ingenic bootrom reads SFC flash at offset 0, finds a 2 KiB
         * header (magic 0x03040506), and loads the SPL binary (starting
         * at flash offset 0x800) into SDRAM. The SPL is linked at
         * 0x80001000 and its J instructions require PC[31:28]=0x8, so it
         * must be placed at physical 0x00001000.
         *
         * QEMU has SDRAM mapped from reset (no DDR init needed), so we
         * load the SPL directly to its linked address and jump there.
         */
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
            error_report("flash missing Ingenic SPL header (got 0x%08x)", magic);
            exit(1);
        }

        uint32_t spl_size = ldl_le_p(&header[12]);
        if (spl_size == 0 || spl_size > 256 * KiB) {
            error_report("SPL size %u out of range", spl_size);
            exit(1);
        }

        /*
         * The SPL ELF is linked with the 2 KiB header occupying
         * 0x80001000..0x800017FF and code starting at 0x80001800.
         * J instructions encode absolute targets assuming this layout.
         * Load the entire header+code block to physical 0x1000 so the
         * addresses match, then start execution past the header.
         */
        uint32_t total = 0x800 + spl_size;
        g_autofree uint8_t *spl_data = g_malloc(total);
        if (blk_pread(blk, 0, total, spl_data, 0) < 0) {
            error_report("failed to read SPL from flash");
            exit(1);
        }

        hwaddr spl_phys = 0x00001000;
        physical_memory_write(spl_phys, spl_data, total);
        cpu->env.active_tc.PC = (int32_t)((spl_phys + 0x800) | 0x80000000);
    }

    /* SPL expects the bootrom to set up the stack in TCSM */
    cpu->env.active_tc.gpr[29] =
        (int32_t)(s->memmap[INGENIC_T31_DEV_TCSM] +
                  INGENIC_T31_TCSM_SIZE + 0x80000000);

    /*
     * Register a reset handler so that 'reset' / 'reboot' inside the
     * guest re-enters the loaded firmware. The MIPS CPU's own reset
     * lands at 0xBFC00000 (our bootrom stub), which doesn't know how
     * to find the loaded SPL/kernel.
     */
    {
        BootResetCtx *b = g_new0(BootResetCtx, 1);
        b->cpu = cpu;
        b->pc = (uint32_t)cpu->env.active_tc.PC;
        b->sp = (uint32_t)cpu->env.active_tc.gpr[29];
        if (machine->kernel_filename) {
            b->is_uimage = (cpu->env.active_tc.gpr[4] == 2);
            b->argv[0] = (uint32_t)cpu->env.active_tc.gpr[5];
        } else {
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
                b->spl_phys = 0x00001000;
            }
        }
        qemu_register_reset(ingenic_t31_cpu_reset, b);
    }
}

static void ingenic_t31_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T31 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

/*
 * All Ingenic T-series XBurst1 SoCs share the same peripheral memory
 * map. The only per-SoC differences are CPUID, EFUSE subtype, PLL
 * defaults, and RAM - all selected by -global ingenic-t31.soc-variant=
 * (e.g. t20x, t10l, t31al). Separate machine names are provided for
 * discoverability (-M ingenic-t20, -M ingenic-t10) but they all run
 * the same board_init and SoC model.
 */
DEFINE_MACHINE("ingenic-t31", ingenic_t31_machine_init)

static void ingenic_t20_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T20 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t20", ingenic_t20_machine_init)

static void ingenic_t21_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T21 (XBurst1 MIPS32r2, ePHY)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t21", ingenic_t21_machine_init)

static void ingenic_t23_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T23 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t23", ingenic_t23_machine_init)

static void ingenic_t30_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T30 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t30", ingenic_t30_machine_init)

static void ingenic_t32_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T32 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t32", ingenic_t32_machine_init)

static void ingenic_t33_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T33 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 256 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t33", ingenic_t33_machine_init)

static void ingenic_t10_machine_init(MachineClass *mc)
{
    mc->desc = "Ingenic T10 (XBurst1 MIPS32r2)";
    mc->init = ingenic_t31_board_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("XBurstR2");
    mc->default_ram_size = 64 * MiB;
    mc->default_ram_id = "ingenic-t31.sdram";
    mc->max_cpus = 1;
    mc->default_nic = TYPE_INGENIC_GMAC;
}

DEFINE_MACHINE("ingenic-t10", ingenic_t10_machine_init)
