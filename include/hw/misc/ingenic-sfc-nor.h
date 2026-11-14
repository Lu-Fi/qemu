/*
 * The SPI NOR part behind an Ingenic SPI Flash Controller (SFC)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_SFC_NOR_H
#define HW_MISC_INGENIC_SFC_NOR_H

#include "qemu/units.h"

/*
 * An image of exactly 8 MiB is an 8 MiB part. Otherwise the smallest
 * power-of-two part between 16 and 64 MiB that holds an image of
 * image_len bytes, so a 32 MiB dump shows up as a 32 MiB chip instead of
 * being cut at 16 MiB.
 */
static inline uint32_t ingenic_sfc_nor_size(int64_t image_len)
{
    uint32_t size = 16 * MiB;

    if (image_len == 8 * MiB) {
        return 8 * MiB;
    }
    while (image_len > size && size < 64 * MiB) {
        size *= 2;
    }
    return size;
}

/*
 * The part ignores the address bits above its capacity, so an access
 * beyond the end lands at the same offset modulo the size. flash_size is
 * a power of two, or 0 when no part is fitted.
 */
static inline uint32_t ingenic_sfc_nor_wrap(uint32_t flash_size,
                                            uint32_t addr)
{
    return flash_size ? addr & (flash_size - 1) : addr;
}

/* The little-endian word at addr, reading on from the end back to 0 */
static inline uint32_t ingenic_sfc_nor_word(const uint8_t *data,
                                            uint32_t flash_size,
                                            uint32_t addr)
{
    uint32_t v = 0;
    int i;

    if (!flash_size) {
        return 0xFFFFFFFF;
    }
    for (i = 3; i >= 0; i--) {
        v = (v << 8) | data[ingenic_sfc_nor_wrap(flash_size, addr + i)];
    }
    return v;
}

/* Program a word the same way: bits only go from 1 to 0 */
static inline void ingenic_sfc_nor_program_word(uint8_t *data,
                                                uint32_t flash_size,
                                                uint32_t addr, uint32_t v)
{
    int i;

    for (i = 0; i < 4; i++) {
        data[ingenic_sfc_nor_wrap(flash_size, addr + i)] &=
            (v >> (8 * i)) & 0xFF;
    }
}

/*
 * The RDID answer as the FIFO word holds it, in wire order (first byte
 * lowest). jedec_id is manufacturer, type and capacity the way datasheets
 * and U-Boot print it (0xEF4018); 0 picks a part of the modeled size so
 * the guest's flash table matches its capacity:
 *    8 MiB -> W25Q64   (EF 40 17)
 *   16 MiB -> W25Q128  (EF 40 18)
 *   32 MiB -> GD25Q256 (C8 40 19)
 *   64 MiB -> W25Q512  (EF 40 20)
 */
static inline uint32_t ingenic_sfc_nor_rdid(uint32_t flash_size,
                                            uint32_t jedec_id)
{
    if (!jedec_id) {
        switch (flash_size) {
        case 8 * MiB:
            jedec_id = 0xEF4017;
            break;
        case 32 * MiB:
            jedec_id = 0xC84019;
            break;
        case 64 * MiB:
            jedec_id = 0xEF4020;
            break;
        default:
            jedec_id = 0xEF4018;
            break;
        }
    }
    return ((jedec_id >> 16) & 0xFF) | (jedec_id & 0xFF00) |
           ((jedec_id & 0xFF) << 16);
}

#endif
