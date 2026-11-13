/*
 * Ingenic bus windows that only hold registers: the T32's HARB0, HARB2, the
 * AXI arbiters, the bus monitors, the SSI slave and the windows that read 0
 * and drop writes
 *
 * Each register's value out of reset and the bits a write keeps, as the
 * parts read them at their U-Boot prompts and keep each value written back
 * one by one: the T32LQ's HARB0, 32 words that repeat every 0x80 and all
 * keep a write, the chip ID among them; a T40XP's AXI arbiters (the A1's
 * second reads as the T40's), a version, its configuration and one word
 * that keeps a write; the bus monitors' channels, a T40XP's and an A1N's
 * every 0x100, repeating every 4 KiB, a T41NQ's every KiB with a control
 * block at 0x2000; the SDHCI's DesignWare MSHC areas, the same on every
 * part that has one (a T32LQ's written), its pointers at 0xe0 and from
 * 0x100 its command-queue and vendor registers. Nothing here counts or
 * arbitrates: there is no bus traffic to watch.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-busregs.h"
#include "migration/vmstate.h"

#define HARB0_SOC_ID    0x2c

typedef struct {
    const IngenicBusReg *regs;
    unsigned n;
    uint32_t at;                /* where its registers start */
} IngenicBusRegsBlock;

#define BLOCK(t, a)     { t, ARRAY_SIZE(t), a }

static const IngenicBusReg harb0_t32[] = {
    { 0x004, 0x00000000, 0xffffffff },
    { 0x008, 0x00000000, 0xffffffff },
    { 0x00c, 0x00000000, 0xffffffff },
    { 0x010, 0x00000000, 0xffffffff },
    { 0x014, 0x00000000, 0xffffffff },
    { 0x018, 0x00000000, 0xffffffff },
    { 0x01c, 0x00000000, 0xffffffff },
    { 0x020, 0x00000000, 0xffffffff },
    { 0x024, 0x00000000, 0xffffffff },
    { 0x028, 0x00000000, 0xffffffff },
    { 0x02c, 0x00000000, 0xffffffff },
    { 0x030, 0x000000ee, 0xffffffff },
    { 0x034, 0x00000000, 0xffffffff },
    { 0x038, 0x00000000, 0xffffffff },
    { 0x03c, 0x00000000, 0xffffffff },
    { 0x040, 0x00000000, 0xffffffff },
    { 0x044, 0x00000000, 0xffffffff },
    { 0x048, 0x00000000, 0xffffffff },
    { 0x04c, 0x00000000, 0xffffffff },
    { 0x050, 0x00000000, 0xffffffff },
    { 0x054, 0x00000000, 0xffffffff },
    { 0x058, 0x00000000, 0xffffffff },
    { 0x05c, 0x00000000, 0xffffffff },
    { 0x060, 0x00000000, 0xffffffff },
    { 0x064, 0x00000000, 0xffffffff },
    { 0x068, 0x00000000, 0xffffffff },
    { 0x06c, 0x00000000, 0xffffffff },
    { 0x070, 0x0000ffdd, 0xffffffff },
    { 0x074, 0x00000000, 0xffffffff },
    { 0x078, 0x00000000, 0xffffffff },
    { 0x07c, 0x00000000, 0xffffffff },
};

/*
 * HARB2's registers, 16 words that repeat every 0x40: 0 keeps bits 21:0, 0x2c
 * and up read 0 and drop writes, the rest keep a write but for 0x04 (bits
 * 23:16, 14:12, 10:8, 2:0), 0x08 (a T23's 0x32c, a T32's 0x312, the other
 * parts' all of it, the T10, T20, T21 and T30 assumed from the T31) and
 * 0x18 (bits 31:16, 8, 2:0). The reset values are a part's own, different
 * from one power-up to the next in a few bits (the flops come up in no set
 * state), so one read stands for all. A T33 is assumed to read as a T32.
 * A write of bit 31 to 0 hangs a T31X's bus: not modelled.
 *
 * A T40NN, T41NQ and A1N keep the same bits as the XBurst1 parts (all of
 * 0x08, as the T31X does) and have their own reset values, with more of
 * them changing between power-ups, 0x08 even within one (the T41NQ's moves
 * as U-Boot runs).
 */
static const IngenicBusReg harb2_t10[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00bd2000, 0x00ff7707 },
    { 0x008, 0x8cbc704a, 0xffffffff },
    { 0x00c, 0x44040080, 0xffffffff },
    { 0x010, 0x078e3131, 0xffffffff },
    { 0x014, 0x80384501, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0xb474dc7e, 0xffffffff },
    { 0x020, 0x1c008a23, 0xffffffff },
    { 0x024, 0x2a75a212, 0xffffffff },
    { 0x028, 0x36024129, 0xffffffff },
};

static const IngenicBusReg harb2_t20[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00c04500, 0x00ff7707 },
    { 0x008, 0x252c0a0b, 0xffffffff },
    { 0x00c, 0x00000004, 0xffffffff },
    { 0x010, 0x64a62802, 0xffffffff },
    { 0x014, 0x0c804101, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x528e63d8, 0xffffffff },
    { 0x020, 0x1049685c, 0xffffffff },
    { 0x024, 0x2a895fb8, 0xffffffff },
    { 0x028, 0x535101b3, 0xffffffff },
};

static const IngenicBusReg harb2_t21[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00a22300, 0x00ff7707 },
    { 0x008, 0x28502523, 0xffffffff },
    { 0x00c, 0x59fc5aa2, 0xffffffff },
    { 0x010, 0x4e107856, 0xffffffff },
    { 0x014, 0x03550224, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x00000000, 0xffffffff },
    { 0x020, 0x029c3666, 0xffffffff },
    { 0x024, 0x5a3c1071, 0xffffffff },
    { 0x028, 0x18d82363, 0xffffffff },
};

static const IngenicBusReg harb2_t23[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00477200, 0x00ff7707 },
    { 0x008, 0x050c1203, 0x0000032c },
    { 0x00c, 0x294c704c, 0xffffffff },
    { 0x010, 0x90080891, 0xffffffff },
    { 0x014, 0x24021b61, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x050801c6, 0xffffffff },
    { 0x020, 0x02882000, 0xffffffff },
    { 0x024, 0x80000580, 0xffffffff },
    { 0x028, 0x01030708, 0xffffffff },
};

static const IngenicBusReg harb2_t30[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00f43700, 0x00ff7707 },
    { 0x008, 0x4d8c5836, 0xffffffff },
    { 0x00c, 0x8142e722, 0xffffffff },
    { 0x010, 0xe42f368c, 0xffffffff },
    { 0x014, 0x6ad7f154, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0xb4d9c20b, 0xffffffff },
    { 0x020, 0x5482179c, 0xffffffff },
    { 0x024, 0xd2ab6483, 0xffffffff },
    { 0x028, 0x8f4538cc, 0xffffffff },
};

static const IngenicBusReg harb2_t31[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00ad3100, 0x00ff7707 },
    { 0x008, 0xcc05040b, 0xffffffff },
    { 0x00c, 0x8c005014, 0xffffffff },
    { 0x010, 0x00400040, 0xffffffff },
    { 0x014, 0x64030287, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x30d21c30, 0xffffffff },
    { 0x020, 0x00008304, 0xffffffff },
    { 0x024, 0x8c162065, 0xffffffff },
    { 0x028, 0xc0008042, 0xffffffff },
};

static const IngenicBusReg harb2_t32[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00155600, 0x00ff7707 },
    { 0x008, 0x01441880, 0x00000312 },
    { 0x00c, 0x010a0004, 0xffffffff },
    { 0x010, 0x08900802, 0xffffffff },
    { 0x014, 0x2a005040, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x0003200f, 0xffffffff },
    { 0x020, 0x40901880, 0xffffffff },
    { 0x024, 0x00042082, 0xffffffff },
    { 0x028, 0x30808113, 0xffffffff },
};

static const IngenicBusReg harb2_a1[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00fd7400, 0x00ff7707 },
    { 0x008, 0x5ba36536, 0xffffffff },
    { 0x00c, 0xdcff7d7f, 0xffffffff },
    { 0x010, 0xdf7bddcb, 0xffffffff },
    { 0x014, 0xd0204934, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x00184068, 0xffffffff },
    { 0x020, 0x7fb9ffff, 0xffffffff },
    { 0x024, 0x818e8000, 0xffffffff },
    { 0x028, 0x7bf367ef, 0xffffffff },
};

static const IngenicBusReg harb2_t40[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x00cc4400, 0x00ff7707 },
    { 0x008, 0xffffffc0, 0xffffffff },
    { 0x00c, 0xc091dd90, 0xffffffff },
    { 0x010, 0xbfffdfff, 0xffffffff },
    { 0x014, 0xffffffff, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0x00000000, 0xffffffff },
    { 0x020, 0xffffebff, 0xffffffff },
    { 0x024, 0xe3b96ded, 0xffffffff },
    { 0x028, 0x1270b889, 0xffffffff },
};

static const IngenicBusReg harb2_t41[] = {
    { 0x000, 0x00000000, 0x003fffff },
    { 0x004, 0x003d3200, 0x00ff7707 },
    { 0x008, 0xdcff7fff, 0xffffffff },
    { 0x00c, 0xbefdffbf, 0xffffffff },
    { 0x010, 0xfefe2ffe, 0xffffffff },
    { 0x014, 0x7dff7fff, 0xffffffff },
    { 0x018, 0x00000000, 0xffff0107 },
    { 0x01c, 0xfc33deed, 0xffffffff },
    { 0x020, 0xffbf7fff, 0xffffffff },
    { 0x024, 0xff3fbffb, 0xffffffff },
    { 0x028, 0xffbff3ff, 0xffffffff },
};

/*
 * The XBurst2 parts' SSI slave window, 0x100 bytes repeating: two words that
 * read the same after every power-up and drop writes (a T40NN's and a
 * T41NQ's), the T40XP and T41LQ assumed to read as them
 */
static const IngenicBusReg ssi_slv[] = {
    { 0x000, 0x40945e80, 0x00000000 },
    { 0x01c, 0x00800080, 0x00000000 },
};

static const IngenicBusRegsBlock ssi_slv_blocks[] = {
    BLOCK(ssi_slv, 0),
};

static const IngenicBusReg axiarb1_t40[] = {
    { 0x000, 0x3430302a, 0x00000000 },
    { 0x004, 0x00102001, 0x00000000 },
    { 0x00c, 0x00000000, 0xffffffff },
};

static const IngenicBusReg axiarb2_t40[] = {
    { 0x000, 0x3430302a, 0x00000000 },
    { 0x004, 0x00105001, 0x00000000 },
    { 0x00c, 0x00000000, 0xffffffff },
};

static const IngenicBusReg mon_chan[] = {
    { 0x000, 0x0000ff00, 0xe007ff07 },
    { 0x004, 0xffffffff, 0xffffffff },
    { 0x008, 0x00000000, 0x80ff80ff },
    { 0x00c, 0x00000fff, 0x00000fff },
    { 0x050, 0x00000000, 0xffffffff },
    { 0x054, 0x00000000, 0xffffffff },
    { 0x074, 0x0000007f, 0x0000007f },
    { 0x078, 0x00000000, 0x0000007f },
};

static const IngenicBusReg mon_chan_small[] = {
    { 0x000, 0x00000000, 0xf000f003 },
    { 0x004, 0x00000000, 0xffffffff },
    { 0x008, 0x00000000, 0xffffffff },
    { 0x074, 0x00000003, 0x00000003 },
    { 0x078, 0x00000000, 0x00000003 },
};

static const IngenicBusReg mon41_chan[] = {
    { 0x000, 0x00000000, 0x00011111 },
    { 0x008, 0x00000000, 0x00011111 },
    { 0x010, 0x00000000, 0x00077777 },
    { 0x020, 0x00000200, 0x0000ffff },
    { 0x024, 0x00000200, 0x0000ffff },
    { 0x028, 0x00000200, 0x0000ffff },
    { 0x02c, 0x00000200, 0x0000ffff },
    { 0x030, 0x00000200, 0x0000ffff },
    { 0x100, 0x00000000, 0x00000011 },
    { 0x108, 0x00000000, 0xffff1111 },
    { 0x110, 0x00000000, 0x00000077 },
    { 0x120, 0x00000000, 0xffffffff },
    { 0x124, 0x10000000, 0xffffffff },
    { 0x140, 0x00000000, 0xffffffff },
    { 0x144, 0x10000000, 0xffffffff },
    { 0x200, 0x00000000, 0x01041041 },
    { 0x208, 0x00000000, 0x3fffffff },
    { 0x210, 0x00000000, 0x0f3cf3cf },
    { 0x280, 0x00000000, 0x00011111 },
    { 0x2a0, 0x00000000, 0x00033333 },
    { 0x2c0, 0x00000000, 0xffffffff },
    { 0x2c4, 0x00000000, 0xffffffff },
    { 0x2c8, 0x00000000, 0xffffffff },
    { 0x2cc, 0x00000000, 0xffffffff },
    { 0x2d0, 0x00000000, 0xffffffff },
    { 0x300, 0x00000000, 0x00000011 },
    { 0x308, 0x00000000, 0x00000011 },
    { 0x310, 0x00000000, 0x00000077 },
    { 0x330, 0x00000402, 0x0000ffff },
    { 0x334, 0x00000402, 0x0000ffff },
};

static const IngenicBusReg mon41_chan_small[] = {
    { 0x000, 0x00000000, 0xffffffff },
    { 0x010, 0x00000000, 0xffffffff },
    { 0x100, 0x00000000, 0x00000011 },
    { 0x108, 0x00000000, 0x00ff1111 },
    { 0x110, 0x00000000, 0x00000077 },
    { 0x120, 0x00000000, 0xffffffff },
    { 0x124, 0x10000000, 0xffffffff },
    { 0x140, 0x00000000, 0xffffffff },
    { 0x144, 0x10000000, 0xffffffff },
    { 0x200, 0x00000000, 0x00000011 },
    { 0x208, 0x00000000, 0x00ff0000 },
    { 0x210, 0x00000000, 0x00007711 },
    { 0x280, 0x00000000, 0x00000011 },
    { 0x28c, 0x00000000, 0x00ff1100 },
    { 0x2a0, 0x00000000, 0x00000011 },
    { 0x2c0, 0x00000000, 0xffffffff },
    { 0x2c4, 0x00000000, 0xffffffff },
};

static const IngenicBusReg mon41_ctrl[] = {
    { 0x000, 0x00000000, 0x01011111 },
    { 0x004, 0x00000000, 0x01011111 },
    { 0x008, 0x00000000, 0x01011111 },
    { 0x00c, 0x00000000, 0x01011111 },
    { 0x010, 0x00000000, 0x01011111 },
    { 0x014, 0x00000000, 0x11111111 },
    { 0x018, 0x00000000, 0x11111111 },
    { 0x020, 0x00000000, 0x07077777 },
    { 0x024, 0x00000000, 0x33333333 },
    { 0x028, 0x00000000, 0x07077777 },
    { 0x02c, 0x00000000, 0x33333333 },
    { 0x030, 0x00000000, 0x03033333 },
    { 0x300, 0xffffffff, 0xffffffff },
    { 0x304, 0x00000000, 0x00000011 },
};

static const IngenicBusRegsBlock monitor_t40[] = {
    BLOCK(mon_chan, 0x0000),
    BLOCK(mon_chan, 0x0100),
    BLOCK(mon_chan, 0x0200),
    BLOCK(mon_chan, 0x0300),
    BLOCK(mon_chan, 0x0400),
    BLOCK(mon_chan, 0x0600),
    BLOCK(mon_chan_small, 0x0500),
};

static const IngenicBusRegsBlock monitor_a1[] = {
    BLOCK(mon_chan, 0x0000),
    BLOCK(mon_chan, 0x0100),
    BLOCK(mon_chan, 0x0200),
    BLOCK(mon_chan, 0x0300),
    BLOCK(mon_chan, 0x0400),
    BLOCK(mon_chan, 0x0600),
    BLOCK(mon_chan_small, 0x0500),
    BLOCK(mon_chan_small, 0x0700),
};

static const IngenicBusRegsBlock monitor_t41[] = {
    BLOCK(mon41_chan, 0x0000),
    BLOCK(mon41_chan, 0x0400),
    BLOCK(mon41_chan, 0x0800),
    BLOCK(mon41_chan, 0x0c00),
    BLOCK(mon41_chan, 0x1000),
    BLOCK(mon41_chan, 0x1800),
    BLOCK(mon41_chan_small, 0x1400),
    BLOCK(mon41_chan_small, 0x1c00),
    BLOCK(mon41_ctrl, 0x2000),
};
/*
 * The A1's SATA port 0 vendor words (AHCI PxVS at 0x170) the VGA sample
 * polls: bits 6:5 of the status never clear on an A1N with no drive. The
 * sample only writes bits the control word already has, so that the word
 * keeps other writes is assumed.
 */
static const IngenicBusReg sata_vs[] = {
    { 0x000, 0x10001110, 0xffffffff },
    { 0x004, 0x2020c160, 0x00000000 },
};

/* The MSHC's pointers at 0xe0, over the SDHCI's own registers */
static const IngenicBusReg mshc_ptr[] = {
    { 0x000, 0x0f580f48, 0x00000000 },
    { 0x004, 0x0f6c0f68, 0x00000000 },
    { 0x008, 0x01800500, 0x00000000 },
};

/* The MSHC's areas from 0x100: command queue at 0x180, vendor at 0x500 */
static const IngenicBusReg mshc[] = {
    { 0x080, 0x00000510, 0x00000000 },
    { 0x084, 0x00003018, 0x00000000 },
    { 0x088, 0x00000000, 0x00001101 },
    { 0x08c, 0x00000000, 0x00000001 },
    { 0x094, 0x00000000, 0x0000000f },
    { 0x098, 0x00000000, 0x0000000f },
    { 0x09c, 0x00000000, 0x80000000, 0x00001f7f },
    { 0x0a0, 0x00000000, 0xffffffff },
    { 0x0c0, 0x00011000, 0x000fffff },
    { 0x0c4, 0x00000000, 0x0000ffff },
    { 0x0d0, 0xfdf9a080, 0xffffffff },
    { 0x400, 0x3134302a, 0x00000000 },
    { 0x404, 0x67612a2a, 0x00000000 },
    { 0x408, 0x000f0000, 0xffff0107 },
    { 0x40c, 0x00000120, 0x0000ffff },
    { 0x410, 0x0303000f, 0x0303030f },
    { 0x42c, 0x00000000, 0xf1000703, 0x00010000 },
    { 0xe48, 0x00000000, 0x00000f01 },
    { 0xe4c, 0x00000000, 0x00ff80c0 },
    { 0xe50, 0x00000000, 0x0003ff00 },
    { 0xe54, 0x00000000, 0x000000ff },
    { 0xe58, 0x00040000, 0x00000000 },
    { 0xe60, 0x40000100, 0x00000000 },
    { 0xe64, 0x00000004, 0x00000000 },
    { 0xe6c, 0x00000000, 0x7f770000 },
};

static const IngenicBusRegsBlock mshc_ptr_blocks[] = {
    BLOCK(mshc_ptr, 0),
};

static const IngenicBusRegsBlock mshc_blocks[] = {
    BLOCK(mshc, 0),
};

static const IngenicBusRegsBlock harb0_t32_blocks[] = {
    BLOCK(harb0_t32, 0),
};

static const IngenicBusRegsBlock axiarb1_t40_blocks[] = {
    BLOCK(axiarb1_t40, 0),
};

static const IngenicBusRegsBlock axiarb2_t40_blocks[] = {
    BLOCK(axiarb2_t40, 0),
};

static const IngenicBusRegsBlock harb2_t10_blocks[] = {
    BLOCK(harb2_t10, 0),
};

static const IngenicBusRegsBlock harb2_t20_blocks[] = {
    BLOCK(harb2_t20, 0),
};

static const IngenicBusRegsBlock harb2_t21_blocks[] = {
    BLOCK(harb2_t21, 0),
};

static const IngenicBusRegsBlock harb2_t23_blocks[] = {
    BLOCK(harb2_t23, 0),
};

static const IngenicBusRegsBlock harb2_t30_blocks[] = {
    BLOCK(harb2_t30, 0),
};

static const IngenicBusRegsBlock harb2_t31_blocks[] = {
    BLOCK(harb2_t31, 0),
};

static const IngenicBusRegsBlock harb2_t32_blocks[] = {
    BLOCK(harb2_t32, 0),
};

static const IngenicBusReg gpio_gap[] = {
    { 0x000, 0x00000000, 0x00000000 },
};

static const IngenicBusRegsBlock sata_vs_blocks[] = {
    BLOCK(sata_vs, 0),
};

static const IngenicBusRegsBlock gpio_gap_blocks[] = {
    BLOCK(gpio_gap, 0),
};

static const IngenicBusRegsBlock harb2_a1_blocks[] = {
    BLOCK(harb2_a1, 0),
};

static const IngenicBusRegsBlock harb2_t40_blocks[] = {
    BLOCK(harb2_t40, 0),
};

static const IngenicBusRegsBlock harb2_t41_blocks[] = {
    BLOCK(harb2_t41, 0),
};

typedef struct {
    const char *model;
    uint32_t window;            /* bytes the window spans */
    uint32_t period;            /* it repeats every period bytes */
    const IngenicBusRegsBlock *blocks;
    unsigned nblocks;
} IngenicBusRegsModel;

#define MODEL(name, win, per, b) \
    { name, win, per, b, ARRAY_SIZE(b) }

static const IngenicBusRegsModel models[] = {
    MODEL("harb0-t32", 0x1000, 0x80, harb0_t32_blocks),
    MODEL("harb2-t10", 0x1000, 0x40, harb2_t10_blocks),
    MODEL("harb2-t20", 0x1000, 0x40, harb2_t20_blocks),
    MODEL("harb2-t21", 0x1000, 0x40, harb2_t21_blocks),
    MODEL("harb2-t23", 0x1000, 0x40, harb2_t23_blocks),
    MODEL("harb2-t30", 0x1000, 0x40, harb2_t30_blocks),
    MODEL("harb2-t31", 0x1000, 0x40, harb2_t31_blocks),
    MODEL("harb2-t32", 0x1000, 0x40, harb2_t32_blocks),
    MODEL("gpio-gap", 0x1000, 0x1000, gpio_gap_blocks),
    MODEL("ssi-slv", 0x1000, 0x100, ssi_slv_blocks),
    MODEL("zero4k", 0x1000, 0x1000, gpio_gap_blocks),
    MODEL("zero64k", 0x10000, 0x10000, gpio_gap_blocks),
    MODEL("harb2-a1", 0x1000, 0x40, harb2_a1_blocks),
    MODEL("harb2-t40", 0x1000, 0x40, harb2_t40_blocks),
    MODEL("harb2-t41", 0x1000, 0x40, harb2_t41_blocks),
    MODEL("axiarb1-t40", 0x1000, 0x1000, axiarb1_t40_blocks),
    MODEL("axiarb2-t40", 0x1000, 0x1000, axiarb2_t40_blocks),
    MODEL("monitor-t40", 0x10000, 0x1000, monitor_t40),
    MODEL("monitor-a1", 0x10000, 0x1000, monitor_a1),
    MODEL("monitor-t41", 0x10000, 0x10000, monitor_t41),
    MODEL("sata-vs", 0x8, 0x8, sata_vs_blocks),
    MODEL("mshc-ptr", 0x10, 0x10, mshc_ptr_blocks),
    MODEL("mshc", 0xf00, 0xf00, mshc_blocks),
};

static const IngenicBusRegsModel *ingenic_busregs_model(IngenicBusRegsState *s)
{
    for (int i = 0; i < ARRAY_SIZE(models); i++) {
        if (s->model && !strcmp(s->model, models[i].model)) {
            return &models[i];
        }
    }
    return NULL;
}

static uint64_t ingenic_busregs_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicBusRegsState *s = INGENIC_BUSREGS(opaque);

    offset %= s->period;
    return offset < INGENIC_BUSREGS_MAX * 4 ? s->regs[offset / 4] : 0;
}

static void ingenic_busregs_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicBusRegsState *s = INGENIC_BUSREGS(opaque);
    unsigned i;

    offset %= s->period;
    if (offset >= INGENIC_BUSREGS_MAX * 4) {
        return;
    }
    i = offset / 4;
    s->regs[i] = (s->regs[i] & ~s->wmask[i]) | (value & s->wmask[i]) |
                 (value & s->sticky[i]);
}

static const MemoryRegionOps ingenic_busregs_ops = {
    .read = ingenic_busregs_read,
    .write = ingenic_busregs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_busregs_reset_hold(Object *obj, ResetType type)
{
    IngenicBusRegsState *s = INGENIC_BUSREGS(obj);
    const IngenicBusRegsModel *m = ingenic_busregs_model(s);

    memset(s->regs, 0, sizeof(s->regs));
    for (unsigned b = 0; m && b < m->nblocks; b++) {
        const IngenicBusRegsBlock *k = &m->blocks[b];

        for (unsigned i = 0; i < k->n; i++) {
            s->regs[(k->at + k->regs[i].offset) / 4] = k->regs[i].reset;
        }
    }
    if (m && !strcmp(m->model, "harb0-t32")) {
        s->regs[HARB0_SOC_ID / 4] = s->chip_id;
    }
}

static void ingenic_busregs_realize(DeviceState *dev, Error **errp)
{
    IngenicBusRegsState *s = INGENIC_BUSREGS(dev);
    const IngenicBusRegsModel *m = ingenic_busregs_model(s);

    if (!m) {
        error_setg(errp, "ingenic-busregs: unknown model '%s'",
                   s->model ? s->model : "");
        return;
    }
    s->period = m->period;
    memset(s->wmask, 0, sizeof(s->wmask));
    memset(s->sticky, 0, sizeof(s->sticky));
    for (unsigned b = 0; b < m->nblocks; b++) {
        const IngenicBusRegsBlock *k = &m->blocks[b];

        for (unsigned i = 0; i < k->n; i++) {
            s->wmask[(k->at + k->regs[i].offset) / 4] = k->regs[i].wmask;
            s->sticky[(k->at + k->regs[i].offset) / 4] = k->regs[i].sticky;
        }
    }
    memory_region_init_io(&s->iomem, OBJECT(s), &ingenic_busregs_ops, s,
                          TYPE_INGENIC_BUSREGS, m->window);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static const VMStateDescription vmstate_ingenic_busregs = {
    .name = TYPE_INGENIC_BUSREGS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicBusRegsState, INGENIC_BUSREGS_MAX),
        VMSTATE_END_OF_LIST()
    },
};

static const Property ingenic_busregs_properties[] = {
    DEFINE_PROP_STRING("model", IngenicBusRegsState, model),
    DEFINE_PROP_UINT32("chip-id", IngenicBusRegsState, chip_id, 0),
};

static void ingenic_busregs_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->realize = ingenic_busregs_realize;
    dc->vmsd = &vmstate_ingenic_busregs;
    rc->phases.hold = ingenic_busregs_reset_hold;
    device_class_set_props(dc, ingenic_busregs_properties);
}

static const TypeInfo ingenic_busregs_info = {
    .name = TYPE_INGENIC_BUSREGS,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicBusRegsState),
    .class_init = ingenic_busregs_class_init,
};

void ingenic_busregs_create(const char *model, hwaddr base, uint32_t chip_id)
{
    DeviceState *dev = qdev_new(TYPE_INGENIC_BUSREGS);

    qdev_prop_set_string(dev, "model", model);
    qdev_prop_set_uint32(dev, "chip-id", chip_id);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    /* above a device it lies over, as the MSHC's pointers do the SDHCI */
    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(dev), 0, base, 1);
}

static void ingenic_busregs_register_types(void)
{
    type_register_static(&ingenic_busregs_info);
}

type_init(ingenic_busregs_register_types)
