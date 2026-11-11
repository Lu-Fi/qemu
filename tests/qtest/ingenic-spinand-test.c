/*
 * QTest testcase for the SPI NAND behind the Ingenic SFC
 *
 * Every part the model knows, through the XBurst1 SFC (T31) one command
 * at a time as U-Boot drives it: the ID bytes and whether they follow an
 * address byte, the geometry (the last page of the last block, its spare
 * area, from a dump with the spare areas in it), the planes' caches, the
 * ECC status bits a corrected and an uncorrectable read leave, OIP for the
 * part's tRD, tPROG and tBERS, and a block marked bad. The XBurst2 SFC
 * (T41) runs the busy and status-poll cases too.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qobject/qdict.h"
#include "libqtest.h"

#define SFC             0x13440000
#define SFC_GLB         0x00
#define SFC_DEV_STA_EXP 0x08
#define SFC_DEV_STA_RT  0x0c
#define SFC_DEV_STA_MSK 0x10
#define SFC_TRAN_CONF0  0x14
#define SFC_TRAN_LEN    0x2c
#define SFC_DEV_ADDR0   0x30
#define SFC_TRIG        0x64
#define SFC_SR          0x68
#define SFC_SCR         0x6c
#define SFC_CMD_IDX     0x7c
#define SFC_COL_ADDR    0x80
#define SFC_ROW_ADDR    0x84
#define SFC_STA_ADDR0   0x88
#define SFC_CDT         0x800
#define SFC_DR          0x1000

#define GLB_ONE_PHASE   (1 << 3)
#define GLB_TRAN_DIR    (1 << 13)
#define GLB_CDT_EN      (1 << 14)
#define CDT_LINK        (1u << 31)
#define CDT_COL         0
#define CDT_ROW         1
#define CDT_STA0        2
#define CMD_IDX_DATAEN  (1u << 31)
#define TRAN_DATEEN     (1 << 16)
#define TRAN_POLLEN     (1 << 25)
#define TRAN_ADDR(n)    ((n) << 26)
#define TRIG_START      1
#define TRIG_STOP       2
#define TRIG_FLUSH      4
#define SR_END          (1 << 4)

#define STS_OIP         0x01
#define STS_E_FAIL      0x04
#define STS_P_FAIL      0x08

#define US              1000
#define MS              1000000

typedef struct {
    const char *name;
    uint8_t id[6];
    int id_len;
    bool id_after_byte;
    uint32_t page, oob, ppb, blocks, planes, dies;
    uint8_t ecc_mask, ecc_corr, ecc_uncorr;
    uint8_t ecc_ext, ecc_ext_val;       /* the count register, its value */
    uint32_t t_read, t_prog, t_erase;   /* ns */
} Part;

/*
 * As thingino's U-Boot (drivers/mtd/nand/spi) and vendor kernels
 * (nand_device/) describe each part; see hw/misc/ingenic-spinand.c
 */
static const Part parts[] = {
    { "GSS01GSAK1", { 0x52, 0xba }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 25 * US, 500 * US, 3 * MS },
    { "TX25G01", { 0xa1, 0xf1 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x70, 0x10, 0x70, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "AS5F34G04SND", { 0x52, 0x2f }, 2, true,
      2048, 128, 64, 4096, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 100 * US, 900 * US, 10 * MS },
    { "AS5F38G04SND", { 0x52, 0x2d }, 2, true,
      4096, 256, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 100 * US, 900 * US, 10 * MS },
    { "ATO25D1GA", { 0x9b, 0x12 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 25 * US, 500 * US, 3 * MS },
    { "DS35Q12C", { 0xe5, 0x75 }, 2, true,
      2048, 64, 64, 512, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q1GA", { 0xe5, 0x71 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q1GD", { 0xe5, 0x51 }, 2, true,
      2048, 128, 64, 1024, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q2GAXXX", { 0xe5, 0x72 }, 2, true,
      2048, 64, 64, 2048, 2, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35Q2GBS", { 0xe5, 0xb2 }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "DS35X1GAXXX", { 0xe5, 0x71 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 700 * US, 10 * MS },
    { "F50D1G41LB", { 0xc8, 0x11, 0x7f, 0x7f }, 4, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "F50D2G41KA", { 0xc8, 0x51, 0x7f, 0x7f, 0x7f }, 5, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "F50L1G41LB", { 0xc8, 0x01, 0x7f, 0x7f, 0x7f }, 5, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "F50L1G41LC", { 0x8c, 0x2c }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "F50L2G41XA", { 0x2c, 0x24 }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "FM25S01A", { 0xa1, 0xe4 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "FM25S01B", { 0xa1, 0xd4 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "FM25S02A", { 0xa1, 0xe5 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "F35SQA001G", { 0xcd, 0x71, 0x71 }, 3, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "F35SQA002G", { 0xcd, 0x72, 0x72 }, 3, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "F35SQA002G-CD52", { 0xcd, 0x52 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x70, 0x10, 0x70, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "F35SQA512M", { 0xcd, 0x70 }, 2, true,
      2048, 64, 64, 512, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "FS35ND01G-CDA1", { 0xcd, 0xa1 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x70, 0x40, 0x70, 0x00, 0x00, 180 * US, 400 * US, 3 * MS },
    { "FS35ND01G-CDEA", { 0xcd, 0xea }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 450 * US, 1400 * US, 16 * MS },
    { "FS35ND02G", { 0xcd, 0xeb }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 450 * US, 1400 * US, 16 * MS },
    { "GD5F1GM7RExxG", { 0xc8, 0x81 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GM7UE", { 0xc8, 0x91 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GM9RExxG", { 0xc8, 0x81, 0x01 }, 3, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GM9UExxG", { 0xc8, 0x91, 0x01 }, 3, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4RExxG", { 0xc8, 0xc1 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4RExxH", { 0xc8, 0xc9 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4RF", { 0xc8, 0xa1 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4UB", { 0xc8, 0xd1 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4UC", { 0xc8, 0xb1, 0x48 }, 3, false,
      2048, 128, 64, 1024, 1, 1,
      0x70, 0x60, 0x70, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ4xA", { 0xc8, 0xf1 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ5RExxG", { 0xc8, 0x41 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ5RExxH", { 0xc8, 0x21 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F1GQ5UE", { 0xc8, 0x51 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0xf0, 0x30, 120 * US, 700 * US, 5 * MS },
    { "GD5F2GM7RExxG", { 0xc8, 0x82 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GM7UE", { 0xc8, 0x92 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4RExxG", { 0xc8, 0xc2 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4UB", { 0xc8, 0xd2 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4UC", { 0xc8, 0xb2 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x70, 0x60, 0x70, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ4xA", { 0xc8, 0xf2 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ5RExxG", { 0xc8, 0x42 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "GD5F2GQ5UE", { 0xc8, 0x52 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0xf0, 0x30, 120 * US, 700 * US, 5 * MS },
    { "GD5F2GQ5xExxH", { 0xc8, 0x22 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GM8RExxG", { 0xc8, 0x85 }, 2, true,
      2048, 128, 64, 4096, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GM8UE", { 0xc8, 0x95 }, 2, true,
      2048, 128, 64, 4096, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4RC", { 0xc8, 0xa4, 0x68 }, 3, false,
      4096, 256, 64, 2048, 1, 1,
      0x70, 0x60, 0x70, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4UB", { 0xc8, 0xd4 }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 120 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4UC", { 0xc8, 0xb4, 0x68 }, 3, false,
      4096, 256, 64, 2048, 1, 1,
      0x70, 0x60, 0x70, 0x00, 0x00, 120 * US, 700 * US, 5 * MS },
    { "GD5F4GQ4xA", { 0xc8, 0xf4 }, 2, true,
      2048, 64, 64, 4096, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ6RExxG", { 0xc8, 0x45 }, 2, true,
      2048, 128, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "GD5F4GQ6UExxG", { 0xc8, 0x55 }, 2, true,
      2048, 128, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0xf0, 0x30, 80 * US, 700 * US, 5 * MS },
    { "IS37SML01G", { 0xc8, 0x21 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 5 * MS },
    { "MX31LF1GE4BC", { 0xc2, 0x1e }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX31LF2GE4BC", { 0xc2, 0x2e }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX31UF1GE4BC", { 0xc2, 0x9e }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35LF1G24AD", { 0xc2, 0x14, 0x03 }, 3, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF1GE4AB", { 0xc2, 0x12 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2G14AC", { 0xc2, 0x20 }, 2, true,
      2048, 64, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x7c, 0x04, 25 * US, 600 * US, 4 * MS },
    { "MX35LF2G24AD", { 0xc2, 0x24, 0x03 }, 3, true,
      2048, 128, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2G24AD-Z4I8", { 0xc2, 0x64, 0x03 }, 3, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2GE4AB", { 0xc2, 0x22 }, 2, true,
      2048, 64, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF2GE4AD", { 0xc2, 0x26, 0x03 }, 3, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 25 * US, 600 * US, 4 * MS },
    { "MX35LF4G24AD", { 0xc2, 0x35, 0x03 }, 3, true,
      4096, 256, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF4G24AD-Z4I8", { 0xc2, 0x75, 0x03 }, 3, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 600 * US, 4 * MS },
    { "MX35LF4GE4AD", { 0xc2, 0x37, 0x03 }, 3, true,
      4096, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1G14AC", { 0xc2, 0x90 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1G24AD", { 0xc2, 0x94, 0x03 }, 3, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1GE4AC", { 0xc2, 0x92, 0x01 }, 3, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF1GE4AD", { 0xc2, 0x96, 0x03 }, 3, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2G14AC", { 0xc2, 0xa0 }, 2, true,
      2048, 64, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2G24AD", { 0xc2, 0xa4, 0x03 }, 3, true,
      2048, 128, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2G24AD-Z4I8", { 0xc2, 0xe4, 0x03 }, 3, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2GE4AC", { 0xc2, 0xa2, 0x01 }, 3, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x04, 70 * US, 600 * US, 4 * MS },
    { "MX35UF2GE4AD", { 0xc2, 0xa6, 0x03 }, 3, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF4G24AD", { 0xc2, 0xb5, 0x03 }, 3, true,
      4096, 256, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF4G24AD-Z4I8", { 0xc2, 0xf5, 0x03 }, 3, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX35UF4GE4AD", { 0xc2, 0xb7, 0x03 }, 3, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MX3UF2GE4BC", { 0xc2, 0xae }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x7c, 0x08, 70 * US, 600 * US, 4 * MS },
    { "MT29F1G01ABAFD-2C14", { 0x2c, 0x14 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F1G01ABAFD-2C15", { 0x2c, 0x15 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01A", { 0x2c, 0x24 }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01A-B024", { 0xb0, 0x24 }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01AAAED", { 0x2c, 0x9f }, 2, true,
      2048, 64, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F2G01ABBGD", { 0x2c, 0x25 }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F4G01ABAFD", { 0x2c, 0x34 }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F4G01ABBFD", { 0x2c, 0x35 }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F4G01ADAGD", { 0x2c, 0x36 }, 2, true,
      2048, 128, 64, 4096, 2, 2,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F8G01ADAFD", { 0x2c, 0x46 }, 2, true,
      4096, 256, 64, 4096, 1, 2,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "MT29F8G01ADBFD", { 0x2c, 0x47 }, 2, true,
      4096, 256, 64, 4096, 1, 2,
      0x70, 0x50, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "PN26G01AW", { 0xa1, 0xe1 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "PN26G02AW", { 0xa1, 0xe2 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "PN26Q01AW", { 0xa1, 0xc1 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "S35ML01G3", { 0x01, 0x15 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "S35ML01G300", { 0x01, 0x14 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "S35ML02G300", { 0x01, 0x25 }, 2, true,
      2048, 128, 64, 2048, 2, 1,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "S35ML04G300", { 0x01, 0x35 }, 2, true,
      2048, 128, 64, 4096, 2, 1,
      0x30, 0x20, 0x30, 0x00, 0x00, 90 * US, 700 * US, 10 * MS },
    { "TC58CVG0S3HRAIG", { 0x98, 0xc2 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG0S3HRAIJ", { 0x98, 0xe2 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG1S3HRAIG", { 0x98, 0xcb }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG1S3HRAIJ", { 0x98, 0xeb }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG2S0HRAIG", { 0x98, 0xcd }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CVG2S0HRAIJ", { 0x98, 0xed }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG0S3HRAIG", { 0x98, 0xb2 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG0S3HRAIJ", { 0x98, 0xd2 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG1S3HRAIG", { 0x98, 0xbb }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG1S3HRAIJ", { 0x98, 0xdb }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG2S0HRAIG", { 0x98, 0xbd }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58CYG2S0HRAIJ", { 0x98, 0xdd }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TC58NYG0S3HBAI4", { 0x98, 0xa1 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58CVG3S0HRAIJ", { 0x98, 0xe4 }, 2, true,
      4096, 256, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58CYG3S0HRAIJ", { 0x98, 0xd4 }, 2, true,
      4096, 256, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58NYG2S3HBAI4", { 0x98, 0xac }, 2, true,
      2048, 128, 64, 8192, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "TH58NYG3S0HBAI6", { 0x98, 0xa3 }, 2, true,
      4096, 256, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 80 * US, 700 * US, 5 * MS },
    { "UM19A0HISW", { 0xb0, 0x14 }, 2, true,
      2048, 128, 64, 1024, 2, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "UM19A8HISW", { 0xb0, 0xfc }, 2, true,
      2048, 64, 64, 256, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "UM19A9HISW", { 0xb0, 0x0c }, 2, true,
      2048, 64, 64, 512, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 50 * US, 400 * US, 10 * MS },
    { "W25M02GV", { 0xef, 0xab, 0x21 }, 3, true,
      2048, 64, 64, 2048, 1, 2,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01GV", { 0xef, 0xaa, 0x21 }, 3, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01GW", { 0xef, 0xba, 0x21 }, 3, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01JW", { 0xef, 0xbc, 0x21 }, 3, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N01KV", { 0xef, 0xae, 0x21 }, 3, true,
      2048, 96, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x40, 60 * US, 700 * US, 10 * MS },
    { "W25N02JW", { 0xef, 0xbf, 0x22 }, 3, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W25N02KV", { 0xef, 0xaa, 0x22 }, 3, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N02KW", { 0xef, 0xba, 0x22 }, 3, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N04KV", { 0xef, 0xaa, 0x23 }, 3, true,
      2048, 128, 64, 4096, 2, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N04KW", { 0xef, 0xba, 0x23 }, 3, true,
      2048, 128, 64, 4096, 1, 1,
      0x30, 0x10, 0x20, 0x30, 0x80, 60 * US, 700 * US, 10 * MS },
    { "W25N512GW", { 0xef, 0xba, 0x20 }, 3, true,
      2048, 64, 64, 512, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W35N01JW", { 0xef, 0xdc, 0x21 }, 3, true,
      4096, 128, 64, 512, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W35N02JW", { 0xef, 0xdf, 0x22 }, 3, true,
      4096, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "W35N04JW", { 0xef, 0xdf, 0x23 }, 3, true,
      4096, 128, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 60 * US, 700 * US, 10 * MS },
    { "WPS3NS01W-S4YO", { 0xa5, 0xa0 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 240 * US, 1400 * US, 10 * MS },
    { "XT26G01A", { 0x0b, 0xe1 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x3c, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G01C", { 0x0b, 0x11 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0xf0, 0x10, 0xf0, 0x00, 0x00, 260 * US, 1000 * US, 8 * MS },
    { "XT26G01D", { 0x0b, 0x31 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02A", { 0x0b, 0xe2 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x3c, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02B", { 0x0b, 0xf2 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02C", { 0x0b, 0x12 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0xf0, 0x10, 0xf0, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02D", { 0x0b, 0x32 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G02E", { 0x2c, 0x24 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x70, 0x10, 0x20, 0x00, 0x00, 80 * US, 700 * US, 10 * MS },
    { "XT26G04A", { 0x0b, 0xe3 }, 2, true,
      2048, 64, 128, 2048, 1, 1,
      0x3c, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G04D", { 0x0b, 0x33 }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G11C", { 0x0b, 0x15 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G11D", { 0x0b, 0x34 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26G12D", { 0x0b, 0x35 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26Q01D", { 0x0b, 0x51 }, 2, true,
      2048, 128, 64, 1024, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26Q02D", { 0x0b, 0x52 }, 2, true,
      2048, 128, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "XT26Q04D", { 0x0b, 0x53 }, 2, true,
      4096, 256, 64, 2048, 1, 1,
      0x30, 0x30, 0x20, 0x00, 0x00, 260 * US, 350 * US, 3 * MS },
    { "ZB35Q01B", { 0x5e, 0xa1 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 140 * US, 1000 * US, 5 * MS },
    { "ZB35Q02B", { 0x5e, 0xa2 }, 2, true,
      2048, 64, 64, 2048, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 140 * US, 1000 * US, 5 * MS },
    { "ZD35Q1GA", { 0xba, 0x71 }, 2, true,
      2048, 64, 64, 1024, 1, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 320 * US, 2 * MS },
    { "ZD35Q2GA", { 0xba, 0x72 }, 2, true,
      2048, 64, 64, 2048, 2, 1,
      0x30, 0x10, 0x20, 0x00, 0x00, 70 * US, 320 * US, 2 * MS },
};

typedef struct {
    QTestState *qts;
    const Part *p;
    char *image;
} Nand;

/* One command, as U-Boot issues it: one phase, PIO */
static void cmd(Nand *n, uint8_t op, int addr_bytes, uint32_t addr,
                bool out, uint8_t *buf, uint32_t len)
{
    QTestState *qts = n->qts;
    uint32_t conf = op | TRAN_ADDR(addr_bytes) | (len ? TRAN_DATEEN : 0);

    qtest_writel(qts, SFC + SFC_GLB,
                 GLB_ONE_PHASE | (out ? GLB_TRAN_DIR : 0));
    qtest_writel(qts, SFC + SFC_TRAN_CONF0, conf);
    qtest_writel(qts, SFC + SFC_DEV_ADDR0, addr);
    qtest_writel(qts, SFC + SFC_TRAN_LEN, len);
    qtest_writel(qts, SFC + SFC_TRIG, TRIG_FLUSH);
    qtest_writel(qts, SFC + SFC_TRIG, TRIG_START);
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t w = 0;

        if (out) {
            memcpy(&w, &buf[i], MIN(4, len - i));
            qtest_writel(qts, SFC + SFC_DR, w);
        } else {
            w = qtest_readl(qts, SFC + SFC_DR);
            memcpy(&buf[i], &w, MIN(4, len - i));
        }
    }
    g_assert_true(qtest_readl(qts, SFC + SFC_SR) & SR_END);
    qtest_writel(qts, SFC + SFC_SCR, 0x1f);
}

static uint8_t get_feature(Nand *n, uint8_t reg)
{
    uint8_t v[4];

    cmd(n, 0x0f, 1, reg, false, v, 1);
    return v[0];
}

static void set_feature(Nand *n, uint8_t reg, uint8_t val)
{
    uint8_t v[4] = { val };

    cmd(n, 0x1f, 1, reg, true, v, 1);
}

static uint32_t raw(const Part *p)
{
    return p->page + p->oob;
}

/* The column bits; the next one up selects the plane */
static uint32_t col_bits(const Part *p)
{
    uint32_t b = 0;

    while ((1u << b) < raw(p)) {
        b++;
    }
    return b;
}

static uint32_t plane_bit(const Part *p, uint32_t row)
{
    return p->planes > 1 ? ((row / p->ppb) & 1) << col_bits(p) : 0;
}

static void wait_idle(Nand *n, uint32_t ns)
{
    g_assert_cmphex(get_feature(n, 0xc0) & STS_OIP, ==, STS_OIP);
    qtest_clock_step(n->qts, ns - 1);
    g_assert_cmphex(get_feature(n, 0xc0) & STS_OIP, ==, STS_OIP);
    qtest_clock_step(n->qts, 1);
    g_assert_cmphex(get_feature(n, 0xc0) & STS_OIP, ==, 0);
}

static void page_read(Nand *n, uint32_t row)
{
    cmd(n, 0x13, 3, row, false, NULL, 0);
    wait_idle(n, n->p->t_read);
}

static void read_cache(Nand *n, uint32_t row, uint32_t col, uint8_t *buf,
                       uint32_t len)
{
    cmd(n, 0x03, 2, plane_bit(n->p, row) | col, false, buf, len);
}

/* A dump as a programmer reads it, holding the last page of the chip */
static char *make_raw_image(const Part *p, uint32_t row, uint8_t *pg)
{
    char *path = NULL;
    int fd = g_file_open_tmp("spinand-XXXXXX.bin", &path, NULL);
    uint8_t hdr[16] = { 0x06, 0x05, 0x04, 0x03 };

    g_assert_cmpint(fd, >=, 0);
    /* The boards' bootrom stand-in wants an SPL header in page 0 */
    hdr[12] = 0x10;
    g_assert_cmpint(pwrite(fd, hdr, sizeof(hdr), 0), ==, sizeof(hdr));
    g_assert_cmpint(pwrite(fd, pg, raw(p), (off_t)row * raw(p)), ==, raw(p));
    close(fd);
    return path;
}

static Nand *start(const char *machine, const Part *p, const char *extra)
{
    Nand *n = g_new0(Nand, 1);
    uint32_t last = p->blocks * p->ppb - 1;
    g_autofree uint8_t *pg = g_malloc(raw(p));

    for (uint32_t i = 0; i < raw(p); i++) {
        pg[i] = i * 7 + 3;
    }
    n->p = p;
    n->image = make_raw_image(p, last, pg);
    n->qts = qtest_initf("-machine %s -drive if=none,format=raw,file=%s "
                         "-global %s.nand-part=%s "
                         "-global %s.nand-layout=raw %s",
                         machine, n->image,
                         strcmp(machine, "ingenic-t31") ?
                         "ingenic-a1-sfc" : "ingenic-t31-sfc", p->name,
                         strcmp(machine, "ingenic-t31") ?
                         "ingenic-a1-sfc" : "ingenic-t31-sfc", extra);
    return n;
}

static void stop(Nand *n)
{
    qtest_quit(n->qts);
    unlink(n->image);
    g_free(n->image);
    g_free(n);
}

static void qom_set_str(Nand *n, const char *prop, const char *val)
{
    QDict *r = qtest_qmp(n->qts, "{ 'execute': 'qom-set', 'arguments': "
                         "{ 'path': '/machine/soc/sfc', 'property': %s, "
                         "'value': %s } }", prop, val);

    g_assert_false(qdict_haskey(r, "error"));
    qobject_unref(r);
}

static void check_id(Nand *n)
{
    const Part *p = n->p;
    uint8_t got[8];
    uint8_t want[8];

    /* After an address byte, then straight after the opcode */
    for (int a = 1; a >= 0; a--) {
        for (int i = 0; i < 6; i++) {
            int k = i + a - (p->id_after_byte ? 1 : 0);

            want[i] = k < 0 ? 0xff : p->id[k % p->id_len];
        }
        cmd(n, 0x9f, a, 0, false, got, 6);
        g_assert_cmpmem(got, 6, want, 6);
    }
}

/* Two dies: pick the last, by the part's own means */
static uint32_t select_last_die(Nand *n)
{
    const Part *p = n->p;
    uint8_t die = p->dies - 1;

    if (p->dies == 1) {
        return 0;
    }
    if (p->id[0] == 0xef) {
        cmd(n, 0xc2, 0, 0, true, &die, 1);
    } else {
        set_feature(n, 0xd0, die << 6);
    }
    return (p->blocks / p->dies) * p->ppb * die;
}

static void check_geometry(Nand *n, uint32_t last)
{
    const Part *p = n->p;
    g_autofree uint8_t *got = g_malloc(raw(p));

    page_read(n, last);
    read_cache(n, last, 0, got, raw(p));
    for (uint32_t i = 0; i < raw(p); i++) {
        g_assert_cmphex(got[i], ==, (uint8_t)(i * 7 + 3));
    }
    /* The spare area alone, at the column past the page */
    read_cache(n, last, p->page, got, 4);
    g_assert_cmphex(got[0], ==, (uint8_t)(p->page * 7 + 3));
    /* Past the last block the array reads erased */
    page_read(n, last + 1);
    read_cache(n, last + 1, 0, got, 8);
    g_assert_cmphex(got[0], ==, 0xff);
}

/* Two planes keep two caches: the column's plane bit picks one */
static void check_planes(Nand *n, uint32_t last)
{
    uint8_t got[4];

    page_read(n, last);                         /* an odd block: plane 1 */
    cmd(n, 0x03, 2, 1u << col_bits(n->p), false, got, 4);
    g_assert_cmphex(got[0], ==, 3);
    cmd(n, 0x03, 2, 0, false, got, 4);
    g_assert_cmphex(got[0], !=, 3);             /* plane 0's cache */
}

static uint8_t ecc_ext(Nand *n)
{
    uint8_t v[4];

    if (n->p->ecc_ext == 0x7c) {
        cmd(n, 0x7c, 1, 0, false, v, 1);
        return v[0];
    }
    return get_feature(n, n->p->ecc_ext);
}

static void check_ecc(Nand *n, uint32_t last)
{
    const Part *p = n->p;
    g_autofree char *rows = g_strdup_printf("%u", last + select_last_die(n));
    uint8_t got[4];

    qom_set_str(n, "nand-ecc-correctable", rows);
    page_read(n, last);
    g_assert_cmphex(get_feature(n, 0xc0) & p->ecc_mask, ==, p->ecc_corr);
    if (p->ecc_ext) {
        g_assert_cmphex(ecc_ext(n), ==, p->ecc_ext_val);
    }
    read_cache(n, last, 0, got, 4);
    g_assert_cmphex(got[0], ==, 3);             /* mended */

    qom_set_str(n, "nand-ecc-correctable", "");
    qom_set_str(n, "nand-ecc-uncorrectable", rows);
    page_read(n, last);
    g_assert_cmphex(get_feature(n, 0xc0) & p->ecc_mask, ==, p->ecc_uncorr);
    if (p->ecc_ext) {
        g_assert_cmphex(ecc_ext(n), ==, 0);
    }
    read_cache(n, last, 0, got, 4);
    g_assert_cmphex(got[0], ==, 3 ^ 1);         /* as the array holds it */

    /* With the ECC engine off, no status, and the flipped bit shows */
    set_feature(n, 0xb0, get_feature(n, 0xb0) & ~0x10);
    qom_set_str(n, "nand-ecc-uncorrectable", "");
    qom_set_str(n, "nand-ecc-correctable", rows);
    page_read(n, last);
    g_assert_cmphex(get_feature(n, 0xc0) & p->ecc_mask, ==, 0);
    read_cache(n, last, 0, got, 4);
    g_assert_cmphex(got[0], ==, 3 ^ 1);
    set_feature(n, 0xb0, get_feature(n, 0xb0) | 0x10);
    qom_set_str(n, "nand-ecc-correctable", "");
}

static void erase(Nand *n, uint32_t row)
{
    cmd(n, 0x06, 0, 0, false, NULL, 0);
    cmd(n, 0xd8, 3, row, false, NULL, 0);
}

static void check_busy_and_bad(Nand *n)
{
    const Part *p = n->p;
    uint32_t row = 5 * p->ppb;
    g_autofree uint8_t *pg = g_malloc0(raw(p));
    g_autofree char *block = g_strdup_printf("%u",
                                             5 + select_last_die(n) / p->ppb);
    uint8_t got[4];

    set_feature(n, 0xa0, 0);
    erase(n, row);
    wait_idle(n, p->t_erase);
    g_assert_cmphex(get_feature(n, 0xc0) & STS_E_FAIL, ==, 0);
    cmd(n, 0x06, 0, 0, false, NULL, 0);
    cmd(n, 0x02, 2, plane_bit(p, row), true, pg, raw(p));
    cmd(n, 0x10, 3, row, false, NULL, 0);
    wait_idle(n, p->t_prog);
    g_assert_cmphex(get_feature(n, 0xc0) & STS_P_FAIL, ==, 0);

    /* A command while busy is lost: the cache keeps the programmed page */
    erase(n, row);
    cmd(n, 0x13, 3, row + 1, false, NULL, 0);
    qtest_clock_step(n->qts, p->t_erase);
    read_cache(n, row, 0, got, 4);
    g_assert_cmphex(got[0], ==, 0x00);

    /* The factory marker, then a block that will not erase */
    qom_set_str(n, "nand-bad-blocks", block);
    page_read(n, row);
    read_cache(n, row, p->page, got, 2);
    g_assert_cmphex(got[0], ==, 0x00);
    g_assert_cmphex(got[1], ==, 0x00);
    erase(n, row);
    wait_idle(n, p->t_erase);
    g_assert_cmphex(get_feature(n, 0xc0) & STS_E_FAIL, ==, STS_E_FAIL);
}

static void test_part(const void *data)
{
    const Part *p = data;
    Nand *n = start("ingenic-t31", p, "");
    uint32_t last = p->blocks / p->dies * p->ppb - 1;

    check_id(n);
    select_last_die(n);
    check_geometry(n, last);
    if (p->planes > 1) {
        check_planes(n, last);
    }
    check_ecc(n, last);
    check_busy_and_bad(n);
    stop(n);
}

static const Part *part_named(const char *name)
{
    for (size_t i = 0; i < ARRAY_SIZE(parts); i++) {
        if (!strcmp(parts[i].name, name)) {
            return &parts[i];
        }
    }
    g_assert_not_reached();
}

/*
 * The controller's status poll waits the flash out: a PROGRAM EXECUTE
 * then a poll phase for OIP clear ends only after tPROG
 */
static void test_poll(const void *data)
{
    const char *machine = data;
    const Part *p = part_named("GD5F1GQ5UE");
    Nand *n = start(machine, p, "");
    QTestState *qts = n->qts;

    set_feature(n, 0xa0, 0);
    cmd(n, 0x06, 0, 0, false, NULL, 0);
    cmd(n, 0x10, 3, 0x40, false, NULL, 0);
    qtest_writel(qts, SFC + SFC_DEV_STA_EXP, 0);
    qtest_writel(qts, SFC + SFC_DEV_STA_MSK, STS_OIP);
    qtest_writel(qts, SFC + SFC_GLB, GLB_ONE_PHASE);
    qtest_writel(qts, SFC + SFC_TRAN_CONF0,
                 0x0f | TRAN_ADDR(1) | TRAN_POLLEN | TRAN_DATEEN);
    qtest_writel(qts, SFC + SFC_DEV_ADDR0, 0xc0);
    qtest_writel(qts, SFC + SFC_TRAN_LEN, 1);
    qtest_writel(qts, SFC + SFC_TRIG, TRIG_START);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SR) & SR_END, ==, 0);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_DEV_STA_RT) & STS_OIP, ==,
                    STS_OIP);
    qtest_clock_step(qts, p->t_prog);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SR) & SR_END, ==, SR_END);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_DEV_STA_RT) & STS_OIP, ==, 0);
    stop(n);
}

/*
 * A Linux CDT chain, as the 4.4 v2 driver reads a page: PAGE READ, a
 * status poll for OIP clear, READ FROM CACHE. Its data waits for tRD.
 */
static void test_cdt(const void *data)
{
    const char *machine = data;
    const Part *p = part_named("GD5F1GQ5UE");
    Nand *n = start(machine, p, "");
    QTestState *qts = n->qts;
    static const uint32_t chain[3][4] = {
        { CDT_LINK | CDT_ROW, 0x13 | TRAN_ADDR(3), 0, 0 },
        { CDT_LINK | CDT_STA0, 0x0f | TRAN_ADDR(1) | TRAN_POLLEN, 0, STS_OIP },
        { CDT_COL, 0x03 | TRAN_ADDR(2) | TRAN_DATEEN, 0, 0 },
    };

    for (int e = 0; e < 3; e++) {
        for (int w = 0; w < 4; w++) {
            qtest_writel(qts, SFC + SFC_CDT + (e * 4 + w) * 4, chain[e][w]);
        }
    }
    qtest_writel(qts, SFC + SFC_ROW_ADDR, 0);
    qtest_writel(qts, SFC + SFC_COL_ADDR, 0);
    qtest_writel(qts, SFC + SFC_STA_ADDR0, 0xc0);
    qtest_writel(qts, SFC + SFC_TRAN_LEN, 4);
    qtest_writel(qts, SFC + SFC_CMD_IDX, CMD_IDX_DATAEN);
    qtest_writel(qts, SFC + SFC_GLB, GLB_CDT_EN);
    qtest_writel(qts, SFC + SFC_TRIG, TRIG_START);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SR) & (SR_END | 4), ==, 0);
    qtest_clock_step(qts, p->t_read);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_DR), ==, 0x03040506);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SR) & SR_END, ==, SR_END);
    stop(n);
}

/* nand-id finds a part by its whole ID first: C8 51 is not F50D2G41KA */
static void test_by_id(const void *data)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 "
                                 "-global ingenic-t31-sfc.nand-id=0xc851");
    Nand n = { .qts = qts, .p = part_named("GD5F1GQ5UE") };

    check_id(&n);
    qtest_quit(qts);
}

/*
 * The strobes read 0 as a T23N, T30X, T31X and T40XP's do, written or not,
 * and no unimplemented read is logged for them
 */
static void test_strobes(const void *data)
{
    g_autofree char *log = NULL;
    g_autofree char *path = NULL;
    int fd = g_file_open_tmp("ingenic-sfc-XXXXXX", &path, NULL);
    QTestState *qts;

    g_assert_cmpint(fd, >=, 0);
    close(fd);
    qts = qtest_initf("-machine ingenic-t31 -d unimp -D %s", path);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_TRIG), ==, 0);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SCR), ==, 0);
    qtest_writel(qts, SFC + SFC_SCR, 0x1f);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_SCR), ==, 0);
    qtest_writel(qts, SFC + SFC_TRIG, TRIG_FLUSH);
    g_assert_cmphex(qtest_readl(qts, SFC + SFC_TRIG), ==, 0);
    qtest_quit(qts);
    g_assert_true(g_file_get_contents(path, &log, NULL, NULL));
    g_assert_null(strstr(log, "unimplemented read"));
    unlink(path);
}

/* Without nand-busy, operations end at once */
static void test_no_busy(const void *data)
{
    const Part *p = part_named("GD5F1GQ5UE");
    Nand *n = start("ingenic-t31", p, "-global ingenic-t31-sfc.nand-busy=off");

    cmd(n, 0x13, 3, 0, false, NULL, 0);
    g_assert_cmphex(get_feature(n, 0xc0) & STS_OIP, ==, 0);
    stop(n);
}

/*
 * Every part takes a second or more (the larger arrays several), so
 * without -m slow only the first part of each kind runs: its ECC
 * reporting, planes, dies, page size and ID form
 */
static bool first_of_kind(size_t i)
{
    const Part *p = &parts[i];

    for (size_t j = 0; j < i; j++) {
        const Part *q = &parts[j];

        if (q->ecc_mask == p->ecc_mask && q->ecc_corr == p->ecc_corr &&
            q->ecc_uncorr == p->ecc_uncorr && q->ecc_ext == p->ecc_ext &&
            q->ecc_ext_val == p->ecc_ext_val && q->planes == p->planes &&
            q->dies == p->dies && q->page == p->page &&
            q->id_after_byte == p->id_after_byte) {
            return false;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    for (size_t i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-spinand/part/%s",
                                                parts[i].name);

        if (g_test_slow() || first_of_kind(i)) {
            qtest_add_data_func(path, &parts[i], test_part);
        }
    }
    qtest_add_data_func("/ingenic-spinand/t31/poll", "ingenic-t31", test_poll);
    qtest_add_data_func("/ingenic-spinand/t41/poll", "ingenic-t41", test_poll);
    qtest_add_data_func("/ingenic-spinand/t31/cdt", "ingenic-t31", test_cdt);
    qtest_add_data_func("/ingenic-spinand/t41/cdt", "ingenic-t41", test_cdt);
    qtest_add_data_func("/ingenic-spinand/by-id", NULL, test_by_id);
    qtest_add_data_func("/ingenic-spinand/no-busy", NULL, test_no_busy);
    qtest_add_data_func("/ingenic-spinand/strobes", NULL, test_strobes);
    return g_test_run();
}
