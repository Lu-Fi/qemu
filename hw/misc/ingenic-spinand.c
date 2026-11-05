/*
 * Ingenic SoCs: a SPI NAND behind the SFC
 *
 * The SPI NAND parts thingino's U-Boot (drivers/mtd/nand/spi, the T40
 * and T41 SPL tables) and vendor kernels (the 3.10 and 4.4 SFC NAND
 * drivers' nand_device tables) carry, picked by nand-part or by the ID
 * nand-id gives. Per part: the ID bytes and whether they follow an
 * address or dummy byte (U-Boot's READID method; after a byte for parts
 * only the kernels list, which try both); page, spare, pages a block,
 * blocks, planes and dies (U-Boot's NAND_MEMORG where it has the part,
 * which disagrees with the kernels on some spare sizes, else the kernels'
 * table); the ECC status bits a corrected and an uncorrectable read leave
 * in C0h, as U-Boot's get_status reads them (the kernels' decode for
 * their own parts), with the bitflip count GigaDevice keeps in F0h,
 * Winbond and Toshiba in 30h and Macronix behind 7Ch; and tRD, tPROG and
 * tBERS, the waits the kernels' tables give (the most common of the same
 * maker's, or GigaDevice's, where they give none): no datasheet was at
 * hand. A corrected read reports the count at which drivers scrub.
 *
 * The array is the page data, back to back as thingino builds NAND
 * images, or with nand-layout=raw each page followed by its spare area,
 * as a programmer reads a chip; the other layout keeps the spare areas
 * in RAM, all 0xff at power-on. A two-plane part has a cache per plane,
 * picked by the plane bit above the column; a two-die part has its dies
 * one after the other. PAGE READ, PROGRAM EXECUTE and BLOCK ERASE hold
 * OIP for tRD, tPROG and tBERS of virtual time (nand-busy); what else
 * arrives meanwhile, but GET FEATURE and RESET, is ignored. Tests mark
 * blocks bad (the factory marker, 00h in the first two spare bytes of
 * the block's first page; such a block also fails erases and programs)
 * and make given pages read corrected or uncorrectable, until their
 * block is erased; an uncorrectable read, or any with ECC off, returns
 * the page with bits flipped.
 *
 * Not modelled: continuous read, OTP, read retry, the parameter page,
 * Winbond's BUF bit, per-part feature register layouts beyond the
 * GigaDevice one, partial-page program limits, data in the cache while
 * busy, and any bit error a test did not ask for. Any BP bit locks the
 * whole array.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "system/block-backend.h"
#include "hw/misc/ingenic-spinand.h"
#include "migration/vmstate.h"

#define FEAT_LOCK           0
#define FEAT_CFG            1
#define FEAT_STATUS         2
#define LOCK_BP             0x38
#define CFG_ECC_EN          0x10
#define STS_OIP             0x01
#define STS_WEL             0x02
#define STS_E_FAIL          0x04
#define STS_P_FAIL          0x08

enum {
    FAULT_CORRECTABLE = 1,
    FAULT_UNCORRECTABLE,
};

#define US                  1000
#define MS                  1000000

#define DIE_NONE            INGENIC_SPINAND_DIE_NONE
#define DIE_WINBOND         INGENIC_SPINAND_DIE_WINBOND
#define DIE_MICRON          INGENIC_SPINAND_DIE_MICRON

/*
 * name, ID, its length, whether it follows a byte; page, spare, pages a
 * block, blocks, planes, dies, die select; ECC field in C0h, what it
 * reads corrected and uncorrectable, the count register and its value;
 * tRD, tPROG, tBERS
 */
static const IngenicSpiNandPart parts[] = {
    { "GSS01GSAK1", { 0x52, 0xba }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 25 * US, 500 * US, 3 * MS },
    { "TX25G01", { 0xa1, 0xf1 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x70, 0x10, 0x70, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "AS5F34G04SND", { 0x52, 0x2f }, 2, true,
      2048, 128, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 100 * US, 900 * US, 10 * MS },
    { "AS5F38G04SND", { 0x52, 0x2d }, 2, true,
      4096, 256, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 100 * US, 900 * US, 10 * MS },
    { "ATO25D1GA", { 0x9b, 0x12 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 25 * US, 500 * US, 3 * MS },
    { "DS35Q12C", { 0xe5, 0x75 }, 2, true,
      2048, 64, 64, 512, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q1GA", { 0xe5, 0x71 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q1GD", { 0xe5, 0x51 }, 2, true,
      2048, 128, 64, 1024, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q2GAXXX", { 0xe5, 0x72 }, 2, true,
      2048, 64, 64, 2048, 2, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q2GBS", { 0xe5, 0xb2 }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35X1GAXXX", { 0xe5, 0x71 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 700 * US, 10 * MS },
    { "F50D1G41LB", { 0xc8, 0x11, 0x7f, 0x7f }, 4, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "F50D2G41KA", { 0xc8, 0x51, 0x7f, 0x7f, 0x7f }, 5, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "F50L1G41LB", { 0xc8, 0x01, 0x7f, 0x7f, 0x7f }, 5, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "F50L1G41LC", { 0x8c, 0x2c }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "F50L2G41XA", { 0x2c, 0x24 }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "FM25S01A", { 0xa1, 0xe4 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "FM25S01B", { 0xa1, 0xd4 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "FM25S02A", { 0xa1, 0xe5 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "F35SQA001G", { 0xcd, 0x71, 0x71 }, 3, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "F35SQA002G", { 0xcd, 0x72, 0x72 }, 3, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "F35SQA002G-CD52", { 0xcd, 0x52 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x10, 0x70, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "F35SQA512M", { 0xcd, 0x70 }, 2, true,
      2048, 64, 64, 512, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "FS35ND01G-CDA1", { 0xcd, 0xa1 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x70, 0x40, 0x70, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "FS35ND01G-CDEA", { 0xcd, 0xea }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 450 * US, 1400 * US, 16 * MS },
    { "FS35ND02G", { 0xcd, 0xeb }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 450 * US, 1400 * US, 16 * MS },
    { "GD5F1GM7RExxG", { 0xc8, 0x81 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GM7UE", { 0xc8, 0x91 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GM9RExxG", { 0xc8, 0x81, 0x01 }, 3, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GM9UExxG", { 0xc8, 0x91, 0x01 }, 3, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4RExxG", { 0xc8, 0xc1 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4RExxH", { 0xc8, 0xc9 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4RF", { 0xc8, 0xa1 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4UB", { 0xc8, 0xd1 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4UC", { 0xc8, 0xb1, 0x48 }, 3, false,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x70, 0x60, 0x70, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4xA", { 0xc8, 0xf1 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ5RExxG", { 0xc8, 0x41 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ5RExxH", { 0xc8, 0x21 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ5UE", { 0xc8, 0x51 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0xf0, 0x30, 120 * US, 700 * US, 5 * MS },
    { "GD5F2GM7RExxG", { 0xc8, 0x82 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GM7UE", { 0xc8, 0x92 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4RExxG", { 0xc8, 0xc2 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4UB", { 0xc8, 0xd2 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4UC", { 0xc8, 0xb2 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x60, 0x70, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4xA", { 0xc8, 0xf2 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ5RExxG", { 0xc8, 0x42 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ5UE", { 0xc8, 0x52 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0xf0, 0x30, 120 * US, 700 * US, 5 * MS },
    { "GD5F2GQ5xExxH", { 0xc8, 0x22 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GM8RExxG", { 0xc8, 0x85 }, 2, true,
      2048, 128, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GM8UE", { 0xc8, 0x95 }, 2, true,
      2048, 128, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4RC", { 0xc8, 0xa4, 0x68 }, 3, false,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x60, 0x70, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4UB", { 0xc8, 0xd4 }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 120 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4UC", { 0xc8, 0xb4, 0x68 }, 3, false,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x60, 0x70, 0x00, 0x00, 120 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4xA", { 0xc8, 0xf4 }, 2, true,
      2048, 64, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ6RExxG", { 0xc8, 0x45 }, 2, true,
      2048, 128, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ6UExxG", { 0xc8, 0x55 }, 2, true,
      2048, 128, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "IS37SML01G", { 0xc8, 0x21 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "MX31LF1GE4BC", { 0xc2, 0x1e }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX31LF2GE4BC", { 0xc2, 0x2e }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX31UF1GE4BC", { 0xc2, 0x9e }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35LF1G24AD", { 0xc2, 0x14, 0x03 }, 3, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF1GE4AB", { 0xc2, 0x12 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2G14AC", { 0xc2, 0x20 }, 2, true,
      2048, 64, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x04, 25 * US, 600 * US, 4 * MS },
    { "MX35LF2G24AD", { 0xc2, 0x24, 0x03 }, 3, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2G24AD-Z4I8", { 0xc2, 0x64, 0x03 }, 3, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2GE4AB", { 0xc2, 0x22 }, 2, true,
      2048, 64, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2GE4AD", { 0xc2, 0x26, 0x03 }, 3, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 25 * US, 600 * US, 4 * MS },
    { "MX35LF4G24AD", { 0xc2, 0x35, 0x03 }, 3, true,
      4096, 256, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF4G24AD-Z4I8", { 0xc2, 0x75, 0x03 }, 3, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF4GE4AD", { 0xc2, 0x37, 0x03 }, 3, true,
      4096, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1G14AC", { 0xc2, 0x90 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1G24AD", { 0xc2, 0x94, 0x03 }, 3, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1GE4AC", { 0xc2, 0x92, 0x01 }, 3, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1GE4AD", { 0xc2, 0x96, 0x03 }, 3, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2G14AC", { 0xc2, 0xa0 }, 2, true,
      2048, 64, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2G24AD", { 0xc2, 0xa4, 0x03 }, 3, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2G24AD-Z4I8", { 0xc2, 0xe4, 0x03 }, 3, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2GE4AC", { 0xc2, 0xa2, 0x01 }, 3, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2GE4AD", { 0xc2, 0xa6, 0x03 }, 3, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF4G24AD", { 0xc2, 0xb5, 0x03 }, 3, true,
      4096, 256, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF4G24AD-Z4I8", { 0xc2, 0xf5, 0x03 }, 3, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF4GE4AD", { 0xc2, 0xb7, 0x03 }, 3, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX3UF2GE4BC", { 0xc2, 0xae }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MT29F1G01ABAFD-2C14", { 0x2c, 0x14 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F1G01ABAFD-2C15", { 0x2c, 0x15 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01A", { 0x2c, 0x24 }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01A-B024", { 0xb0, 0x24 }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01AAAED", { 0x2c, 0x9f }, 2, true,
      2048, 64, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01ABBGD", { 0x2c, 0x25 }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F4G01ABAFD", { 0x2c, 0x34 }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F4G01ABBFD", { 0x2c, 0x35 }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F4G01ADAGD", { 0x2c, 0x36 }, 2, true,
      2048, 128, 64, 4096, 2, 2, DIE_MICRON,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F8G01ADAFD", { 0x2c, 0x46 }, 2, true,
      4096, 256, 64, 4096, 1, 2, DIE_MICRON,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F8G01ADBFD", { 0x2c, 0x47 }, 2, true,
      4096, 256, 64, 4096, 1, 2, DIE_MICRON,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "PN26G01AW", { 0xa1, 0xe1 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "PN26G02AW", { 0xa1, 0xe2 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "PN26Q01AW", { 0xa1, 0xc1 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "S35ML01G3", { 0x01, 0x15 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "S35ML01G300", { 0x01, 0x14 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "S35ML02G300", { 0x01, 0x25 }, 2, true,
      2048, 128, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "S35ML04G300", { 0x01, 0x35 }, 2, true,
      2048, 128, 64, 4096, 2, 1, DIE_NONE,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "TC58CVG0S3HRAIG", { 0x98, 0xc2 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG0S3HRAIJ", { 0x98, 0xe2 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG1S3HRAIG", { 0x98, 0xcb }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG1S3HRAIJ", { 0x98, 0xeb }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG2S0HRAIG", { 0x98, 0xcd }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG2S0HRAIJ", { 0x98, 0xed }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG0S3HRAIG", { 0x98, 0xb2 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG0S3HRAIJ", { 0x98, 0xd2 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG1S3HRAIG", { 0x98, 0xbb }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG1S3HRAIJ", { 0x98, 0xdb }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG2S0HRAIG", { 0x98, 0xbd }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG2S0HRAIJ", { 0x98, 0xdd }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58NYG0S3HBAI4", { 0x98, 0xa1 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58CVG3S0HRAIJ", { 0x98, 0xe4 }, 2, true,
      4096, 256, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58CYG3S0HRAIJ", { 0x98, 0xd4 }, 2, true,
      4096, 256, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58NYG2S3HBAI4", { 0x98, 0xac }, 2, true,
      2048, 128, 64, 8192, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58NYG3S0HBAI6", { 0x98, 0xa3 }, 2, true,
      4096, 256, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "UM19A0HISW", { 0xb0, 0x14 }, 2, true,
      2048, 128, 64, 1024, 2, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "UM19A8HISW", { 0xb0, 0xfc }, 2, true,
      2048, 64, 64, 256, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "UM19A9HISW", { 0xb0, 0x0c }, 2, true,
      2048, 64, 64, 512, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "W25M02GV", { 0xef, 0xab, 0x21 }, 3, true,
      2048, 64, 64, 2048, 1, 2, DIE_WINBOND,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01GV", { 0xef, 0xaa, 0x21 }, 3, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01GW", { 0xef, 0xba, 0x21 }, 3, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01JW", { 0xef, 0xbc, 0x21 }, 3, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01KV", { 0xef, 0xae, 0x21 }, 3, true,
      2048, 96, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x40, 60 * US, 700 * US, 10 * MS },
    { "W25N02JW", { 0xef, 0xbf, 0x22 }, 3, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N02KV", { 0xef, 0xaa, 0x22 }, 3, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N02KW", { 0xef, 0xba, 0x22 }, 3, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N04KV", { 0xef, 0xaa, 0x23 }, 3, true,
      2048, 128, 64, 4096, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N04KW", { 0xef, 0xba, 0x23 }, 3, true,
      2048, 128, 64, 4096, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N512GW", { 0xef, 0xba, 0x20 }, 3, true,
      2048, 64, 64, 512, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W35N01JW", { 0xef, 0xdc, 0x21 }, 3, true,
      4096, 128, 64, 512, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W35N02JW", { 0xef, 0xdf, 0x22 }, 3, true,
      4096, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W35N04JW", { 0xef, 0xdf, 0x23 }, 3, true,
      4096, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "WPS3NS01W-S4YO", { 0xa5, 0xa0 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "XT26G01A", { 0x0b, 0xe1 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x3c, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G01C", { 0x0b, 0x11 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0xf0, 0x10, 0xf0, 0x00, 0x00, 260 * US, 1000 * US, 8 * MS },
    { "XT26G01D", { 0x0b, 0x31 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02A", { 0x0b, 0xe2 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x3c, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02B", { 0x0b, 0xf2 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02C", { 0x0b, 0x12 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0xf0, 0x10, 0xf0, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02D", { 0x0b, 0x32 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02E", { 0x2c, 0x24 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x70, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 10 * MS },
    { "XT26G04A", { 0x0b, 0xe3 }, 2, true,
      2048, 64, 128, 2048, 1, 1, DIE_NONE,
      0x3c, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G04D", { 0x0b, 0x33 }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G11C", { 0x0b, 0x15 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G11D", { 0x0b, 0x34 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G12D", { 0x0b, 0x35 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26Q01D", { 0x0b, 0x51 }, 2, true,
      2048, 128, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26Q02D", { 0x0b, 0x52 }, 2, true,
      2048, 128, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26Q04D", { 0x0b, 0x53 }, 2, true,
      4096, 256, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "ZB35Q01B", { 0x5e, 0xa1 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 140 * US, 1000 * US, 5 * MS },
    { "ZB35Q02B", { 0x5e, 0xa2 }, 2, true,
      2048, 64, 64, 2048, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 140 * US, 1000 * US, 5 * MS },
    { "ZD35Q1GA", { 0xba, 0x71 }, 2, true,
      2048, 64, 64, 1024, 1, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 320 * US, 2 * MS },
    { "ZD35Q2GA", { 0xba, 0x72 }, 2, true,
      2048, 64, 64, 2048, 2, 1, DIE_NONE,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 320 * US, 2 * MS },
};

static uint32_t pages(IngenicSpiNand *n)
{
    return n->part.blocks * n->part.ppb;
}

static uint32_t page_size(IngenicSpiNand *n)
{
    return n->part.page;
}

static uint32_t raw_size(IngenicSpiNand *n)
{
    return n->part.page + n->part.oob;
}

/* The array's row for a row address on the selected die */
static uint32_t die_row(IngenicSpiNand *n, uint32_t addr)
{
    uint32_t per_die = pages(n) / n->part.dies;
    uint32_t row = addr & 0xffffff;

    return row < per_die ? row + n->die * per_die : UINT32_MAX;
}

static bool busy(IngenicSpiNand *n)
{
    return n->busy_until &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < n->busy_until;
}

static void hold(IngenicSpiNand *n, uint32_t ns)
{
    if (n->timing && ns) {
        n->busy_until = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns;
    }
}

int64_t ingenic_spinand_busy_until(IngenicSpiNand *n)
{
    return busy(n) ? n->busy_until : 0;
}

static uint8_t status(IngenicSpiNand *n)
{
    return n->feat[FEAT_STATUS] | (busy(n) ? STS_OIP : 0);
}

static int feat_index(uint32_t addr)
{
    switch (addr & 0xff) {
    case 0xa0:
        return 0;
    case 0xb0:
        return 1;
    case 0xc0:
        return 2;
    case 0xd0:
        return 3;
    }
    return -1;
}

static bool block_worn(IngenicSpiNand *n, uint32_t block)
{
    for (guint i = 0; i < n->worn->len; i++) {
        if (g_array_index(n->worn, uint32_t, i) == block) {
            return true;
        }
    }
    return false;
}

static void writeback(IngenicSpiNand *n, uint32_t row, uint32_t count)
{
    uint64_t off, len;
    int ret = 0;

    if (!n->blk) {
        return;
    }
    for (uint32_t r = row; r < row + count && ret >= 0; r++) {
        if (n->raw) {
            off = (uint64_t)r * raw_size(n);
            if (off + raw_size(n) > n->blk_len) {
                break;
            }
            ret = blk_pwrite(n->blk, off, page_size(n),
                             &n->data[(size_t)r * page_size(n)], 0);
            if (ret >= 0) {
                ret = blk_pwrite(n->blk, off + page_size(n), n->part.oob,
                                 &n->oob[(size_t)r * n->part.oob], 0);
            }
        } else {
            /* One write for the run of pages: the data is back to back */
            off = (uint64_t)row * page_size(n);
            len = (uint64_t)count * page_size(n);
            if (off >= n->blk_len) {
                break;
            }
            len = MIN(len, n->blk_len - off);
            ret = blk_pwrite(n->blk, off, len, &n->data[off], 0);
            break;
        }
    }
    if (ret < 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic SPI NAND: write-back of "
                      "page 0x%x failed: %s\n", row, strerror(-ret));
    }
}

/* PROGRAM EXECUTE and BLOCK ERASE need WEL and an unlocked array */
static bool writable(IngenicSpiNand *n, const char *op, uint32_t row)
{
    const char *why = !(n->feat[FEAT_STATUS] & STS_WEL) ? "no WRITE ENABLE" :
                      (n->feat[FEAT_LOCK] & LOCK_BP) ? "feature A0h locks it" :
                      row >= pages(n) ? "past the end" :
                      block_worn(n, row / n->part.ppb) ? "a worn block" : NULL;

    if (why) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic SPI NAND: %s of page 0x%x "
                      "fails: %s\n", op, row, why);
    }
    return !why;
}

static unsigned plane_of_row(IngenicSpiNand *n, uint32_t row)
{
    return n->part.planes > 1 ? (row / n->part.ppb) & 1 : 0;
}

static unsigned plane_of_col(IngenicSpiNand *n, uint32_t addr)
{
    return n->part.planes > 1 ? (addr >> n->col_bits) & 1 : 0;
}

static uint32_t column(IngenicSpiNand *n, uint32_t addr)
{
    return addr & ((1u << n->col_bits) - 1);
}

/* Flip bits as the array holds them: one, or more than ECC can mend */
static void flip(uint8_t *page, bool many)
{
    for (int i = 0; i < (many ? 16 : 1); i++) {
        page[i] ^= 0x01;
    }
}

/* The bitflip count some parts give beside C0h, after a corrected read */
static uint8_t ecc_ext(IngenicSpiNand *n, uint8_t reg)
{
    return n->part.ecc_ext == reg && n->corrected ? n->part.ecc_ext_val : 0;
}

static void page_read(IngenicSpiNand *n, uint32_t row)
{
    uint8_t *cache = n->cache[plane_of_row(n, row)];
    uint8_t *st = &n->feat[FEAT_STATUS];
    bool ecc = n->feat[FEAT_CFG] & CFG_ECC_EN;
    int fault = 0;

    *st &= ~n->part.ecc_mask;
    n->corrected = false;
    if (row >= pages(n)) {
        memset(cache, 0xff, raw_size(n));
        return;
    }
    memcpy(cache, &n->data[(size_t)row * page_size(n)], page_size(n));
    memcpy(&cache[page_size(n)], &n->oob[(size_t)row * n->part.oob],
           n->part.oob);
    fault = GPOINTER_TO_INT(g_hash_table_lookup(n->ecc_faults,
                                                GUINT_TO_POINTER(row)));
    if (fault == FAULT_UNCORRECTABLE) {
        flip(cache, true);
        if (ecc) {
            *st |= n->part.ecc_uncorr;
        }
    } else if (fault == FAULT_CORRECTABLE) {
        if (ecc) {
            *st |= n->part.ecc_corr;
            n->corrected = true;
        } else {
            flip(cache, false);
        }
    }
}

static void forget_faults(IngenicSpiNand *n, uint32_t first)
{
    for (uint32_t r = first; r < first + n->part.ppb; r++) {
        g_hash_table_remove(n->ecc_faults, GUINT_TO_POINTER(r));
    }
}

static void program(IngenicSpiNand *n, uint32_t row)
{
    uint8_t *st = &n->feat[FEAT_STATUS];

    if (writable(n, "PROGRAM EXECUTE", row)) {
        uint8_t *cache = n->cache[plane_of_row(n, row)];
        uint8_t *pg = &n->data[(size_t)row * page_size(n)];
        uint8_t *oob = &n->oob[(size_t)row * n->part.oob];

        for (uint32_t i = 0; i < page_size(n); i++) {
            pg[i] &= cache[i];
        }
        for (uint32_t i = 0; i < n->part.oob; i++) {
            oob[i] &= cache[page_size(n) + i];
        }
        writeback(n, row, 1);
        *st &= ~STS_P_FAIL;
    } else {
        *st |= STS_P_FAIL;
    }
    *st &= ~STS_WEL;
    hold(n, n->part.t_prog_ns);
}

static void erase(IngenicSpiNand *n, uint32_t row)
{
    uint8_t *st = &n->feat[FEAT_STATUS];
    uint32_t first = row - row % n->part.ppb;

    if (writable(n, "BLOCK ERASE", row)) {
        memset(&n->data[(size_t)first * page_size(n)], 0xff,
               (size_t)n->part.ppb * page_size(n));
        memset(&n->oob[(size_t)first * n->part.oob], 0xff,
               (size_t)n->part.ppb * n->part.oob);
        forget_faults(n, first);
        writeback(n, first, n->part.ppb);
        *st &= ~STS_E_FAIL;
    } else {
        *st |= STS_E_FAIL;
    }
    *st &= ~STS_WEL;
    hold(n, n->part.t_erase_ns);
}

/* The factory marker: the first spare bytes of the block's first page */
static void mark_bad(IngenicSpiNand *n, uint32_t block)
{
    uint32_t row = block * n->part.ppb;

    if (!block_worn(n, block)) {
        g_array_append_val(n->worn, block);
    }
    if (n->oob && block < n->part.blocks) {
        memset(&n->oob[(size_t)row * n->part.oob], 0x00, 2);
        forget_faults(n, row);
        writeback(n, row, 1);
    }
}

static uint8_t id_byte(IngenicSpiNand *n, unsigned i)
{
    if (n->part.id_after_byte) {
        if (!i) {
            return 0xff;
        }
        i--;
    }
    return n->part.id[i % n->part.id_len];
}

void ingenic_spinand_command(IngenicSpiNand *n, uint8_t cmd, uint32_t addr,
                             bool dummy, uint8_t *buf, uint32_t len)
{
    uint8_t *st = &n->feat[FEAT_STATUS];

    n->cmd = cmd;
    n->addr = addr;
    memset(buf, 0xff, len);

    if (busy(n) && cmd != 0x0f && cmd != 0xff) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic SPI NAND: command 0x%02x "
                      "while busy, ignored\n", cmd);
        n->cmd = 0;
        return;
    }

    switch (cmd) {
    case 0xff:      /* RESET */
        *st = 0;
        n->busy_until = 0;
        break;
    case 0x9f:      /* READ ID */
        for (uint32_t i = 0; i < MIN(len, 16); i++) {
            buf[i] = id_byte(n, i + (dummy ? 1 : 0));
        }
        break;
    case 0x0f: {    /* GET FEATURE */
        int i = feat_index(addr);

        if (!len) {
            break;
        }
        if (i == FEAT_STATUS) {
            buf[0] = status(n);
        } else if (i >= 0) {
            buf[0] = n->feat[i];
        } else {
            buf[0] = ecc_ext(n, addr & 0xff);
        }
        break;
    }
    case 0x7c:      /* Macronix GET ECC STATUS */
        if (len) {
            buf[0] = ecc_ext(n, 0x7c);
        }
        break;
    case 0xc2:      /* Winbond SOFTWARE DIE SELECT: the die follows */
        break;
    case 0x06:      /* WRITE ENABLE */
        *st |= STS_WEL;
        break;
    case 0x04:      /* WRITE DISABLE */
        *st &= ~STS_WEL;
        break;
    case 0x13:      /* PAGE READ, to the cache */
        page_read(n, die_row(n, addr));
        hold(n, n->part.t_read_ns);
        break;
    case 0x03:      /* READ FROM CACHE x1, x2, x4, dual and quad I/O */
    case 0x0b:
    case 0x3b:
    case 0xbb:
    case 0x6b:
    case 0xeb: {
        uint32_t col = column(n, addr);

        if (col < raw_size(n)) {
            memcpy(buf, &n->cache[plane_of_col(n, addr)][col],
                   MIN(len, raw_size(n) - col));
        }
        break;
    }
    case 0x1f:      /* SET FEATURE and PROGRAM LOAD: data phase follows */
    case 0x02:
    case 0x32:
    case 0x84:
    case 0x34:
    case 0xc4:
        break;
    case 0x10:      /* PROGRAM EXECUTE */
        program(n, die_row(n, addr));
        break;
    case 0xd8:      /* BLOCK ERASE */
        erase(n, die_row(n, addr));
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic SPI NAND: command 0x%02x\n", cmd);
        break;
    }
}

void ingenic_spinand_data(IngenicSpiNand *n, const uint8_t *buf,
                          uint32_t len)
{
    static const uint8_t wmask[4] = { 0xbe, 0x51, 0x00, 0x60 };
    uint8_t *cache = n->cache[plane_of_col(n, n->addr)];
    uint32_t col = column(n, n->addr);
    int i;

    switch (n->cmd) {
    case 0x1f:
        i = feat_index(n->addr);
        if (i >= 0 && len) {
            n->feat[i] = (n->feat[i] & ~wmask[i]) | (buf[0] & wmask[i]);
        }
        if (i == 3 && n->part.die_select == INGENIC_SPINAND_DIE_MICRON) {
            n->die = (n->feat[3] >> 6) & (n->part.dies - 1);
        }
        break;
    case 0xc2:
        if (len && n->part.die_select == INGENIC_SPINAND_DIE_WINBOND) {
            n->die = buf[0] & (n->part.dies - 1);
        }
        break;
    case 0x02:      /* PROGRAM LOAD clears the cache, RANDOM DATA does not */
    case 0x32:
        memset(cache, 0xff, raw_size(n));
        /* fall through */
    case 0x84:
    case 0x34:
    case 0xc4:
        if (col < raw_size(n)) {
            memcpy(&cache[col], buf, MIN(len, raw_size(n) - col));
        }
        break;
    }
}

/* "3,0x40,17": the numbers of a fault list */
static bool parse_list(const char *s, GArray *out, Error **errp)
{
    g_auto(GStrv) items = g_strsplit(s, ",", -1);

    for (int i = 0; items[i]; i++) {
        const char *p = g_strstrip(items[i]);
        uint64_t v;
        uint32_t u;

        if (!*p) {
            continue;
        }
        if (qemu_strtou64(p, NULL, 0, &v) < 0 || v > UINT32_MAX) {
            error_setg(errp, "'%s' is not a page or block number", p);
            return false;
        }
        u = v;
        g_array_append_val(out, u);
    }
    return true;
}

static char *list_faults(IngenicSpiNand *n, int kind)
{
    GString *s = g_string_new("");
    GHashTableIter it;
    gpointer k, v;

    g_hash_table_iter_init(&it, n->ecc_faults);
    while (g_hash_table_iter_next(&it, &k, &v)) {
        if (GPOINTER_TO_INT(v) == kind) {
            g_string_append_printf(s, "%s%u", s->len ? "," : "",
                                   GPOINTER_TO_UINT(k));
        }
    }
    return g_string_free(s, false);
}

static void set_faults(IngenicSpiNand *n, int kind, const char *value,
                       Error **errp)
{
    g_autoptr(GArray) rows = g_array_new(false, false, sizeof(uint32_t));
    GHashTableIter it;
    gpointer k, v;

    if (!parse_list(value, rows, errp)) {
        return;
    }
    g_hash_table_iter_init(&it, n->ecc_faults);
    while (g_hash_table_iter_next(&it, &k, &v)) {
        if (GPOINTER_TO_INT(v) == kind) {
            g_hash_table_iter_remove(&it);
        }
    }
    for (guint i = 0; i < rows->len; i++) {
        g_hash_table_insert(n->ecc_faults,
                            GUINT_TO_POINTER(g_array_index(rows, uint32_t, i)),
                            GINT_TO_POINTER(kind));
    }
}

/* What a test set on the chip: a fault list, or worn blocks */
typedef struct {
    IngenicSpiNand *n;
    int kind;               /* FAULT_*, or 0 for the worn blocks */
} FaultProp;

static void fault_get(Object *obj, Visitor *v, const char *name,
                      void *opaque, Error **errp)
{
    FaultProp *p = opaque;
    g_autofree char *s = NULL;

    if (p->kind) {
        s = list_faults(p->n, p->kind);
    } else {
        GString *g = g_string_new("");

        for (guint i = 0; i < p->n->worn->len; i++) {
            g_string_append_printf(g, "%s%u", g->len ? "," : "",
                                   g_array_index(p->n->worn, uint32_t, i));
        }
        s = g_string_free(g, false);
    }
    visit_type_str(v, name, &s, errp);
}

static void fault_set(Object *obj, Visitor *v, const char *name,
                      void *opaque, Error **errp)
{
    FaultProp *p = opaque;
    g_autofree char *value = NULL;
    g_autoptr(GArray) blocks = NULL;

    if (!visit_type_str(v, name, &value, errp)) {
        return;
    }
    if (p->kind) {
        set_faults(p->n, p->kind, value, errp);
        return;
    }
    blocks = g_array_new(false, false, sizeof(uint32_t));
    if (parse_list(value, blocks, errp)) {
        for (guint i = 0; i < blocks->len; i++) {
            mark_bad(p->n, g_array_index(blocks, uint32_t, i));
        }
    }
}

static void fault_release(Object *obj, const char *name, void *opaque)
{
    g_free(opaque);
}

static void add_fault_prop(Object *owner, IngenicSpiNand *n,
                           const char *name, int kind)
{
    FaultProp *p = g_new(FaultProp, 1);

    p->n = n;
    p->kind = kind;
    object_property_add(owner, name, "str", fault_get, fault_set,
                        fault_release, p);
}

/* The chip's array is its drive's; this is what it holds besides */
const VMStateDescription vmstate_ingenic_spinand = {
    .name = "ingenic-spinand",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_2DARRAY(cache, IngenicSpiNand, 2, INGENIC_SPINAND_RAW),
        VMSTATE_UINT8_ARRAY(feat, IngenicSpiNand, 4),
        VMSTATE_BOOL(corrected, IngenicSpiNand),
        VMSTATE_UINT8(die, IngenicSpiNand),
        VMSTATE_UINT8(cmd, IngenicSpiNand),
        VMSTATE_UINT32(addr, IngenicSpiNand),
        VMSTATE_INT64(busy_until, IngenicSpiNand),
        VMSTATE_END_OF_LIST()
    }
};

void ingenic_spinand_add_properties(Object *owner, IngenicSpiNand *n)
{
    n->ecc_faults = g_hash_table_new(NULL, NULL);
    n->worn = g_array_new(false, false, sizeof(uint32_t));
    /*
     * Blocks to mark bad (adds to those marked; a marked block also fails
     * erases and programs), and the pages whose reads ECC corrects or
     * cannot correct (each replaces its list; an erase clears a block's)
     */
    add_fault_prop(owner, n, "nand-bad-blocks", 0);
    add_fault_prop(owner, n, "nand-ecc-correctable", FAULT_CORRECTABLE);
    add_fault_prop(owner, n, "nand-ecc-uncorrectable", FAULT_UNCORRECTABLE);
}

/* By name; by the whole ID, else by the ID's first bytes */
static const IngenicSpiNandPart *find_part(IngenicSpiNand *n)
{
    for (int whole = 1; whole >= 0; whole--) {
        for (size_t i = 0; i < ARRAY_SIZE(parts); i++) {
            const IngenicSpiNandPart *p = &parts[i];
            uint32_t id = 0;

            if (n->part_name) {
                if (!g_ascii_strcasecmp(n->part_name, p->name)) {
                    return p;
                }
                continue;
            }
            for (int b = 0; b < MIN(p->id_len, 4); b++) {
                id = (id << 8) | p->id[b];
                if (id == n->id && (!whole || b == p->id_len - 1)) {
                    return p;
                }
            }
        }
    }
    return NULL;
}

/* A part nand-id names that the table lacks: the model's old geometry */
static void made_up_part(IngenicSpiNand *n)
{
    IngenicSpiNandPart *p = &n->part;

    *p = (IngenicSpiNandPart) {
        .name = "unknown", .page = 2048, .oob = 128, .ppb = 64,
        .blocks = 1024, .planes = 1, .dies = 1, .ecc_mask = 0x30,
        .ecc_corr = 0x10, .ecc_uncorr = 0x20, .id_after_byte = true,
    };
    if (n->id > 0xffff) {
        p->id[p->id_len++] = n->id >> 16;
    }
    p->id[p->id_len++] = n->id >> 8;
    p->id[p->id_len++] = n->id;
}

static bool load(IngenicSpiNand *n, BlockBackend *blk, Error **errp)
{
    uint64_t chip = (uint64_t)pages(n) * (n->raw ? raw_size(n) : page_size(n));
    int64_t size = blk_getlength(blk);
    g_autofree uint8_t *buf = NULL;
    uint32_t whole;

    if (size < 0) {
        error_setg(errp, "failed to size the SPI NAND image");
        return false;
    }
    n->blk_len = size;
    if (size < chip) {
        warn_report("SPI NAND image holds %" PRId64 " of the chip's %"
                    PRIu64 " bytes; what is written past its end is lost "
                    "when QEMU exits", size, chip);
    }
    size = MIN(size, chip);
    if (!n->raw) {
        if (blk_pread(blk, 0, size, n->data, 0) < 0) {
            error_setg(errp, "failed to read the SPI NAND image");
            return false;
        }
    } else {
        /* A programmer's dump: each page followed by its spare area */
        buf = g_malloc(raw_size(n));
        whole = size / raw_size(n);
        for (uint32_t r = 0; r < whole; r++) {
            if (blk_pread(blk, (uint64_t)r * raw_size(n), raw_size(n), buf,
                          0) < 0) {
                error_setg(errp, "failed to read the SPI NAND image");
                return false;
            }
            memcpy(&n->data[(size_t)r * page_size(n)], buf, page_size(n));
            memcpy(&n->oob[(size_t)r * n->part.oob], &buf[page_size(n)],
                   n->part.oob);
        }
    }
    if (blk_supports_write_perm(blk) &&
        blk_set_perm(blk, BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE,
                     BLK_PERM_ALL, NULL) == 0) {
        n->blk = blk;
    }
    return true;
}

bool ingenic_spinand_realize(IngenicSpiNand *n, void *blk, Error **errp)
{
    const IngenicSpiNandPart *p;

    if (!n->id && !n->part_name) {
        return true;
    }
    p = find_part(n);
    if (p) {
        n->part = *p;
    } else if (n->part_name) {
        error_setg(errp, "nand-part '%s' is not a part the model knows",
                   n->part_name);
        return false;
    } else {
        made_up_part(n);
    }
    n->id = 0;
    for (int b = 0; b < MIN(n->part.id_len, 4); b++) {
        n->id = (n->id << 8) | n->part.id[b];
    }
    if (n->blocks) {
        n->part.blocks = n->blocks;
    }
    if (!n->part.blocks || n->part.blocks > INGENIC_SPINAND_MAX_BLOCKS) {
        error_setg(errp, "nand-blocks must be 1 to %d",
                   INGENIC_SPINAND_MAX_BLOCKS);
        return false;
    }
    if (!n->layout || !strcmp(n->layout, "data")) {
        n->raw = false;
    } else if (!strcmp(n->layout, "raw")) {
        n->raw = true;
    } else {
        error_setg(errp, "nand-layout must be 'data' or 'raw'");
        return false;
    }
    n->col_bits = ctz32(pow2ceil(raw_size(n)));
    n->data = g_malloc((size_t)pages(n) * page_size(n));
    n->oob = g_malloc((size_t)pages(n) * n->part.oob);
    memset(n->data, 0xff, (size_t)pages(n) * page_size(n));
    memset(n->oob, 0xff, (size_t)pages(n) * n->part.oob);
    if (blk && !load(n, blk, errp)) {
        return false;
    }
    /* Blocks marked on the command line, now that there is an array */
    for (guint i = 0; i < n->worn->len; i++) {
        mark_bad(n, g_array_index(n->worn, uint32_t, i));
    }
    n->feat[FEAT_LOCK] = LOCK_BP;
    n->feat[FEAT_CFG] = CFG_ECC_EN;
    n->feat[FEAT_STATUS] = 0;
    n->feat[3] = 0;
    memset(n->cache, 0xff, sizeof(n->cache));
    return true;
}

void ingenic_spinand_finalize(IngenicSpiNand *n)
{
    g_free(n->data);
    g_free(n->oob);
    n->data = n->oob = NULL;
    if (n->ecc_faults) {
        g_hash_table_destroy(n->ecc_faults);
        n->ecc_faults = NULL;
    }
    if (n->worn) {
        g_array_free(n->worn, true);
        n->worn = NULL;
    }
}

int ingenic_spinand_boot_read(IngenicSpiNand *n, void *blk, uint32_t len,
                              void *buf)
{
    if (n && n->data) {
        /* As the bootrom reads it: page data, from the first page on */
        if (len > (uint64_t)pages(n) * page_size(n)) {
            return -EINVAL;
        }
        memcpy(buf, n->data, len);
        return 0;
    }
    return blk_pread(blk, 0, len, buf, 0);
}
