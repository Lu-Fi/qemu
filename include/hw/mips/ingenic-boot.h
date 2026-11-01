/*
 * Ingenic boards: a direct Linux boot handed over as thingino's U-Boot does
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MIPS_INGENIC_BOOT_H
#define HW_MIPS_INGENIC_BOOT_H

#include "exec/hwaddr.h"
#include "hw/core/boards.h"
#include "target/mips/cpu.h"

typedef struct {
    bool camera;    /* thingino reserves rmem for media; not on an NVR */
    bool ispmem;    /* T10 and T20 reserve 8 MiB more, below rmem */
} IngenicBootDefaults;

void ingenic_boot_linux(MachineState *machine, MIPSCPU *cpu,
                        BlockBackend *flash, const IngenicBootDefaults *d,
                        hwaddr kernel_end);

/* A GPIO register as a bootrom leaves it: the port, its offset, the value */
typedef struct {
    uint8_t port;
    uint16_t reg;
    uint32_t value;
} IngenicRomGpio;

/*
 * The parts' tables, ending in a port of 0xff: up to the T30 at a port
 * stride of 0x100 (T10, T20) or 0x1000
 */
extern const IngenicRomGpio ingenic_rom_gpio_t10[], ingenic_rom_gpio_t21[],
    ingenic_rom_gpio_t23[], ingenic_rom_gpio_t30[], ingenic_rom_gpio_t31[],
    ingenic_rom_gpio_t32[], ingenic_rom_gpio_a1[];

/* From each reset on, the GPIO and SFC as the bootrom leaves them */
void ingenic_boot_rom_flash_left(hwaddr gpio, uint32_t stride,
                                 const IngenicRomGpio *g, hwaddr sfc,
                                 uint32_t spl_size);

#endif
