/*
 * Ingenic TNNA: the T32's and T33's neural network accelerator
 *
 * The engine the TNPU's RISC-V core drives at 0x12500000. To the core,
 * the first 4 KiB are command ports, a store to each a command and a load
 * from each a register or a wait:
 *   stores +0x00 NNRST, +0x04 NNRW (a register write: index << 20 | its
 *     20-bit value), +0x08 NNMAC, +0x0c NNMMT, +0x10 INITCW, +0x14 NNDWF,
 *     +0x18 NNDWW, +0x1c NNDWBT, +0x24 ENDSIG;
 *   loads +0x00 to +0x1c the G registers, +0x40 WAITNNDW, +0x44 WAITNNMAC,
 *     +0x48 WAITRDBT, +0x80 to +0x1fc the M, D and Q registers (0x20 to
 *     0x7f), +0x200 the 16 sums, +0x240 the 8 WFCW entries (again at
 *     +0x260), +0x280 the performance counters.
 * NNRST puts every register back to its reset (the vendor's code writes
 * only those it wants otherwise) and sets each WFCW entry's control field
 * (bits 14:11) to 1. A store of NNRW to WFCW_INP (0x59) pushes an entry
 * at WFCW_WPTR (0x58), which steps on; one to FR_WADDR_ADD (0x51) adds its
 * signed value to FR_WADDR (0x50). Many of the indices that nna.h leaves
 * unnamed read another register and keep nothing written to them.
 *
 * At +0x8000 is the DMA between DDR and the engine's 64 KiB ORAM: Fin
 * (+0x00) and WBTin (+0x80) into the ORAM, Fout (+0x40) out of it. A
 * store of 1 to a channel's RUN_WAIT (+0x00), or to its ONNA (+0x34) or
 * OCVA (+0x30), runs it; a load from RUN_WAIT waits for it. ORAM addresses
 * and sizes are in 16-byte units, DDR addresses in bytes. The DMA reaches
 * DDR through 28 address bits: a diff or stride near 2^28 steps back.
 *   GREG0: diff between feature planes in DDR (16 B) | planes - 1 << 24
 *   GREG1: stride between lines in DDR (16 B) | sets - 1 << 24 (Fin)
 *   GREG2: diff between planes in ORAM | stride between lines << 16
 *   GREGSET0: diff from set 0's DDR to set 1's (16 B) | its planes - 1 << 24
 *   EREGnL: DDR address | 1 for the last entry (Fin and WBTin, 2 at most)
 *   EREGnH: ORAM address | size << 16 | lines - 1 << 28 (not WBTin)
 * Each entry's addresses are live: a Fin run leaves the ORAM address past
 * its last line and on its last plane, moves the DDR address from set to
 * set, and so starts a second set where the first left off; a Fout run
 * moves both on by its planes; a WBTin run moves the ORAM address on by
 * the size.
 *
 * The host's view of the same 1 MiB is different: 384 KiB of RAM from
 * 0x12500000, nothing above.
 *
 * At 0x12b00000 is the CVA, which turns NV12 lines in the ORAM into
 * B, G, R, alpha pixels in the ORAM (R, G, B, alpha with +0x40 bit 24):
 * +0x10 the Y lines, +0x14 the UV lines, +0x18 the output (ORAM byte
 * addresses); +0x1c the output's line size in 16 B << 20 | the input's
 * stride in bytes; +0x20 width | lines << 16; +0x24 to +0x3c the matrix's
 * non-zero terms (B on Y and U, G on Y, U and V, R on Y and V) and +0x44
 * to +0x4c the biases of B, G and R, all Q20 and signed; +0x40 bits 23:16
 * the alpha byte. A store of 1 to +0x00 runs it, 2 resets it; +0x04 reads
 * 1 once it has run.
 *
 * The compute works out of three more RAMs. NNDWW and NNDWBT copy
 * their operand's bits 31:16 bytes from its ORAM unit (bits 15:0) to the
 * W-RAM and the BT-RAM, from WR_WADDR and BTR_WADDR on (16-byte units),
 * each load after the last. NNDWF
 * copies a column of each (plane, row) of the ORAM into the F-RAM, 8192
 * entries of a pixel's 8 channels as 4-bit slices; INITCW starts its
 * line program (WFCW) over. NNMAC adds 16 pixels by 8 output channels of
 * a convolution to the sums: (weight - zero point) x input, unsigned
 * inputs, weights offset binary. NNMMT copies and clears the sums, reads
 * B and T and writes the quantized pixels to the ORAM, and steps the
 * addresses. Everything runs as it is stored, so the waits and ENDSIG
 * have nothing to wait for.
 *
 * Registers, their resets and the bits each keeps, the DMA, the CVA's
 * pixels and the compute are a T32LQ's, driven from its TNPU core: every
 * kind of layer in Ingenic's multiobjectDet, FireworksDet and CryDet
 * samples (CryDet's depthwise ones among them), replayed on random input,
 * comes out of both byte for byte, and so do 1B2T outputs. Not modelled:
 * what the samples do not use (partial sums between input sets, LUT
 * outputs, 12-bit outputs, WRRA_RINC).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "hw/misc/ingenic-tnna.h"
#include "hw/misc/ingenic-tnpu.h"

#define TNNA_WINDOW         (1 * MiB)
#define TNNA_PORTS          0x1000
#define TNNA_DMA            0x8000

#define PORT_NNRST          0x00
#define PORT_NNRW           0x04
#define PORT_ENDSIG         0x24
#define PORT_WAITNNDW       0x40
#define PORT_WAITNNMAC      0x44
#define PORT_WAITRDBT       0x48
#define PORT_REGS           0x80
#define PORT_FSUM           0x200
#define PORT_WFCW           0x240
#define PORT_PC             0x280

#define REG_FIRST           0x20
#define REG_MAC_CFG         0x20
#define MAC_CFG_DW          BIT(0)      /* depthwise */
#define REG_MAC_KERNEL      0x21
#define REG_FP_NUM          0x22
#define REG_MAC_P_MASK      0x23
#define REG_MAC_BIT         0x28
#define REG_MACTL_FP_Y      0x29
#define REG_TLYP_ACC        0x2a
#define REG_MACTL_FP_X      0x2b
#define REG_FP_H            0x2c
#define REG_MAC_FP          0x2d
#define REG_FP_W            0x2e
#define REG_PAD_VALUE       0x2f
#define REG_FR_FP_STRIDE    0x30
#define REG_FR_BBIT_STRIDE  0x31
#define REG_FR_BELT_LEN     0x32
#define REG_FR_RADDR        0x34        /* INC1 to INC3 follow */
#define REG_WR_RADDR        0x38
#define REG_WR_RADDR_RET    0x39
#define REG_WR_RADDR_INC    0x3a
#define REG_OR_RADDR_F      0x40
#define REG_OR_RADDR_INC    0x41
#define REG_LDCF_CFG        0x48
#define REG_LDCF_RBLNUM     0x49
#define REG_LDCF_LP0INC     0x4a
#define REG_LDCF_LP0RBINC   0x4b
#define REG_LDCF_LP1INC     0x4e
#define REG_LDCF_LP1RBINC   0x4f
#define REG_FR_WADDR        0x50
#define REG_FR_WADDR_ADD    0x51
#define REG_WFCW_WPTR       0x58
#define REG_WFCW_INP        0x59
#define REG_WFCW_STTEND     0x5a
#define REG_FR_CFG          0x5b
#define REG_WR_WADDR        0x5c
#define REG_BTR_WADDR       0x5e
#define REG_Q_CFG           0x60
#define REG_OF_OFFSET       0x62
#define Q_CFG_1B2T          BIT(6)
#define REG_OR_WADDR_Q      0x6a
#define REG_OR_WADDR_INC0   0x6c        /* INC1 to INC3 at +1 to +3 */
#define REG_OLINE_SIZE_M    0x74
#define REG_BTR_RADDR       0x78
#define REG_BTR_RADDR_RET   0x79
#define REG_BTR_RADDR_RINC  0x7b
#define REG_PIXEL_OUT       0x7c        /* 4 of them, a lane per nibble */
#define REGDATA_MASK        0xfffff
#define WFCW_MASK           0x7fff
#define WFCW_INC_MASK       0x7ff
#define WFCW_CTRL_1         0x800
#define WFCW_LC             0xf

#define PORT_NNMAC          0x08
#define PORT_NNMMT          0x0c
#define PORT_INITCW         0x10
#define PORT_NNDWF          0x14
#define PORT_NNDWW          0x18
#define PORT_NNDWBT         0x1c

/* NNMMT's operand, from bit 12 */
#define MMT_CPOC            BIT(0)
#define MMT_CLROC           BIT(1)
#define MMT_RDBT            BIT(2)
#define MMT_QSUM            BIT(3)
#define MMT_WRRA_RST        BIT(8)
#define MMT_WRRA_RINC       BIT(9)
#define MMT_WRRA_RET        BIT(10)
#define MMT_WRRA_INC        BIT(11)
#define MMT_BTRRA_RST       BIT(12)
#define MMT_BTRRA_RINC      BIT(13)
#define MMT_BTRRA_RET       BIT(14)
#define MMT_TLYP            1
#define MMT_TLYMXP          2

#define FRAM_LINE           4           /* entries, i.e. pixels */
#define WRAM_ENTRY          32          /* 8 x 8 nibbles */
#define BTRAM_ENTRY         32          /* 8 x int32 */
#define NNDWF_LINES         4096        /* planes x rows one NNDWF reads */

#define DMA_FIN             0
#define DMA_FOUT            1
#define DMA_WBTIN           2
#define DMA_RUN_WAIT        0
#define DMA_GREG0           1
#define DMA_GREG1           2
#define DMA_GREG2           3
#define DMA_GREGSET0        4
#define DMA_GREGSETS        4
#define DMA_EREGL(n)        (8 + 2 * (n))
#define DMA_EREGH(n)        (9 + 2 * (n))
#define DMA_OCVA            12
#define DMA_ONNA            13
#define DMA_LAST            1
#define DMA_DDR_MASK        0x0fffffff
#define ORAM_UNIT           16
#define ORAM_UNITS          (INGENIC_TNNA_ORAM_SIZE / ORAM_UNIT)

#define CVA_SIZE            0x1000
#define CVA_CTL             0x00
#define CVA_IRQ             0x04
#define CVA_YSRC            0x10
#define CVA_UVSRC           0x14
#define CVA_DST             0x18
#define CVA_STRIDE          0x1c
#define CVA_WH              0x20
#define CVA_COEF            0x24        /* 7 of them, Q20 */
#define CVA_CTRL            0x40
#define CVA_BIAS            0x44        /* 3, Q20 */
#define CVA_REGS            INGENIC_TNNA_CVA_REGS
#define CVA_CTL_START       1
#define CVA_CTL_RESET       2

/* 0x20 to 0x7f as from power-on, and after NNRST */
static const uint32_t reg_reset[INGENIC_TNNA_REGS] = {
    0x5a, 0x0, 0x0, 0xffff, 0x0, 0x0, 0x0, 0x0,
    0x0, 0xfff, 0x0, 0xfff, 0x7ff, 0x22, 0x7ff, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x3210, 0x7654, 0xba98, 0xfedc,
};

/* the bits each keeps; WFCW_WPTR is the push pointer, 0 to 7 */
static const uint32_t reg_mask[INGENIC_TNNA_REGS] = {
    0x37f, 0x77, 0xff, 0xffff, 0x0, 0x0, 0x0, 0x0,
    0x73, 0xfff, 0xff, 0xfff, 0x7ff, 0x37, 0x7ff, 0xffff,
    0x7ff, 0x7ff, 0x7ff, 0x7ff, 0x1fff, 0x1fff, 0x1fff, 0x1fff,
    0x1fff, 0x1fff, 0x1fff, 0x0, 0x0, 0x1fff, 0x1fff, 0x0,
    0xfff, 0xfff, 0x0, 0x0, 0x7ff, 0x7ff, 0x7ff, 0x0,
    0x3fff7, 0xf, 0xfff, 0xfff, 0x0, 0x0, 0xfff, 0xfff,
    0x7ff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x7, 0x0, 0x77, 0x3f, 0x7ff, 0x0, 0xff, 0x0,
    0xf7, 0x3, 0xffff, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0xfff, 0x1fff, 0xfff, 0x0, 0x1fff, 0x1fff, 0x1fff,
    0x0, 0x0, 0x0, 0x0, 0xfff, 0x0, 0x0, 0x0,
    0x7f, 0x7f, 0x7f, 0x7f, 0xffff, 0xffff, 0xffff, 0xffff,
};

/* indices that read another register, and keep nothing written to them */
static const uint8_t reg_alias[INGENIC_TNNA_REGS] = {
    [0x24 - REG_FIRST] = 0x2c, [0x25 - REG_FIRST] = 0x2d,
    [0x26 - REG_FIRST] = 0x36, [0x27 - REG_FIRST] = 0x2f,
    [0x3b - REG_FIRST] = 0x3a, [0x3c - REG_FIRST] = 0x3d,
    [0x3f - REG_FIRST] = 0x3e, [0x42 - REG_FIRST] = 0x5a,
    [0x43 - REG_FIRST] = 0x50, [0x47 - REG_FIRST] = 0x5e,
    [0x4c - REG_FIRST] = 0x5c, [0x4d - REG_FIRST] = 0x5b,
    [0x51 - REG_FIRST] = 0x50, [0x52 - REG_FIRST] = 0x5a,
    [0x53 - REG_FIRST] = 0x50,
    [0x54 - REG_FIRST] = 0x50, [0x55 - REG_FIRST] = 0x50,
    [0x56 - REG_FIRST] = 0x5e, [0x57 - REG_FIRST] = 0x5e,
    [0x59 - REG_FIRST] = 0x5b, [0x5d - REG_FIRST] = 0x5b,
    [0x5f - REG_FIRST] = 0x5b, [0x63 - REG_FIRST] = 0x6b,
    [0x64 - REG_FIRST] = 0x74, [0x65 - REG_FIRST] = 0x74,
    [0x66 - REG_FIRST] = 0x74, [0x67 - REG_FIRST] = 0x74,
    [0x68 - REG_FIRST] = 0x78, [0x6c - REG_FIRST] = 0x7c,
    [0x71 - REG_FIRST] = 0x79, [0x72 - REG_FIRST] = 0x7a,
    [0x73 - REG_FIRST] = 0x7b, [0x75 - REG_FIRST] = 0x7d,
    [0x76 - REG_FIRST] = 0x7e, [0x77 - REG_FIRST] = 0x7f,
};

/* the G registers: 0 to 4 read as these, 5 to 7 keep these bits */
static const uint32_t greg_reset[8] = { 0, 0x14, 0, 0x14, 0x7f, 0x7f, 0, 0 };
static const uint32_t greg_mask[8] = { 0, 0, 0, 0, 0, 0xff, 0x7, 0xff };

static void oram_xfer(IngenicTnnaState *s, bool to_oram, uint32_t ddr,
                      uint32_t oram, uint32_t units)
{
    for (uint32_t i = 0; i < units; i++) {
        uint8_t *p = &s->oram[((oram + i) % ORAM_UNITS) * ORAM_UNIT];
        hwaddr a = (ddr + i * ORAM_UNIT) & DMA_DDR_MASK;

        if (to_oram) {
            address_space_read(&address_space_memory, a,
                               MEMTXATTRS_UNSPECIFIED, p, ORAM_UNIT);
        } else {
            address_space_write(&address_space_memory, a,
                                MEMTXATTRS_UNSPECIFIED, p, ORAM_UNIT);
        }
    }
}

static void dma_run_fin(IngenicTnnaState *s)
{
    uint32_t *r = s->dma[DMA_FIN];
    uint32_t sets = extract32(r[DMA_GREG1], 24, 3) + 1;
    uint32_t stride_ddr = extract32(r[DMA_GREG1], 0, 24) * ORAM_UNIT;
    uint32_t diff_ddr = extract32(r[DMA_GREG0], 0, 24) * ORAM_UNIT;
    uint32_t diff_or = extract32(r[DMA_GREG2], 0, 16);
    uint32_t stride_or = extract32(r[DMA_GREG2], 16, 16);
    int entries = (r[DMA_EREGL(0)] & DMA_LAST) ? 1 : 2;

    if (sets > DMA_GREGSETS + 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-tnna: Fin with %u sets\n",
                      sets);
        sets = DMA_GREGSETS + 1;
    }
    for (uint32_t set = 0; set < sets; set++) {
        uint32_t g = set ? r[DMA_GREGSET0 + set - 1] : r[DMA_GREG0];
        uint32_t planes = extract32(g, 24, 8) + 1;

        for (int e = 0; e < entries; e++) {
            uint32_t ddr = r[DMA_EREGL(e)] & ~(ORAM_UNIT - 1);
            uint32_t h = r[DMA_EREGH(e)];
            uint32_t or = extract32(h, 0, 16);
            uint32_t size = extract32(h, 16, 12);
            uint32_t lines = extract32(h, 28, 4) + 1;

            for (uint32_t l = 0; l < lines; l++) {
                for (uint32_t fp = 0; fp < planes; fp++) {
                    oram_xfer(s, true, ddr + fp * diff_ddr + l * stride_ddr,
                              or + fp * diff_or + l * stride_or, size);
                }
            }
            r[DMA_EREGH(e)] = deposit32(h, 0, 16, or + (planes - 1) * diff_or +
                                        lines * stride_or);
            if (set + 1 < sets) {
                r[DMA_EREGL(e)] += extract32(r[DMA_GREGSET0 + set], 0, 24) *
                                   ORAM_UNIT;
            }
        }
    }
}

static void dma_run_fout(IngenicTnnaState *s)
{
    uint32_t *r = s->dma[DMA_FOUT];
    uint32_t planes = extract32(r[DMA_GREG0], 24, 8) + 1;
    uint32_t diff_ddr = extract32(r[DMA_GREG0], 0, 24) * ORAM_UNIT;
    uint32_t stride_ddr = extract32(r[DMA_GREG1], 0, 24) * ORAM_UNIT;
    uint32_t diff_or = extract32(r[DMA_GREG2], 0, 16);
    uint32_t stride_or = extract32(r[DMA_GREG2], 16, 16);
    uint32_t ddr = r[DMA_EREGL(0)] & ~(ORAM_UNIT - 1);
    uint32_t h = r[DMA_EREGH(0)];
    uint32_t or = extract32(h, 0, 16);
    uint32_t size = extract32(h, 16, 12);
    uint32_t lines = extract32(h, 28, 4) + 1;

    for (uint32_t fp = 0; fp < planes; fp++) {
        for (uint32_t l = 0; l < lines; l++) {
            oram_xfer(s, false, ddr + fp * diff_ddr + l * stride_ddr,
                      or + fp * diff_or + l * stride_or, size);
        }
    }
    r[DMA_EREGL(0)] = (ddr + planes * diff_ddr) | DMA_LAST;
    r[DMA_EREGH(0)] = deposit32(h, 0, 16, or + planes * diff_or);
}

static void dma_run_wbtin(IngenicTnnaState *s)
{
    uint32_t *r = s->dma[DMA_WBTIN];
    int entries = (r[DMA_EREGL(0)] & DMA_LAST) ? 1 : 2;

    for (int e = 0; e < entries; e++) {
        uint32_t h = r[DMA_EREGH(e)];
        uint32_t size = extract32(h, 16, 12);

        oram_xfer(s, true, r[DMA_EREGL(e)] & ~(ORAM_UNIT - 1),
                  extract32(h, 0, 16), size);
        r[DMA_EREGH(e)] = deposit32(h, 0, 16, extract32(h, 0, 16) + size);
    }
}

static void dma_run(IngenicTnnaState *s, int ch)
{
    switch (ch) {
    case DMA_FIN:
        dma_run_fin(s);
        break;
    case DMA_FOUT:
        dma_run_fout(s);
        break;
    default:
        dma_run_wbtin(s);
        break;
    }
}

static uint32_t reg(IngenicTnnaState *s, uint32_t idx)
{
    return s->regs[idx - REG_FIRST];
}

static void reg_set(IngenicTnnaState *s, uint32_t idx, uint32_t val)
{
    s->regs[idx - REG_FIRST] = val & reg_mask[idx - REG_FIRST];
}

/*
 * Ingenic's quantify(): offset binary out of a sum, its B and its T,
 * obits / 2 radix-4 digits; the sum shifted first (rounded half up when
 * right).
 */
static uint32_t quantify(int64_t sum, int64_t b, int64_t t, int shift,
                         unsigned obits)
{
    int64_t r;
    uint32_t q = 0;

    sum = shift < 0 ? (sum + (1LL << (-shift - 1))) >> -shift : sum << shift;
    r = sum + b;
    for (unsigned i = 0; i < obits / 2; i++) {
        q <<= 2;
        if (r < 0) {
            if (t < -r) {
                r += t;
            } else {
                q |= 1;
            }
            r += t / 2;
        } else {
            if (r < t) {
                q |= 2;
            } else {
                q |= 3;
                r -= t;
            }
            r -= t / 2;
        }
        r *= 4;
    }
    return q;
}

/*
 * 1B2T (Q_CFG bit 6), as gen_layer.c's quantify(): a sum plus B that is
 * negative goes against a second threshold from TN, with its first r_num
 * digits 3 and its result one up (negated with qds_neg); OF_OFFSET is
 * added and the result clamped to the output's range. The digits start
 * from r - 2T + (T >> (obits - 1)), or r + 2TN - (TN >> (obits - 2 r_num
 * - 1)). TN, bit 31 set, packs (qds_neg 29, r_num 28:26, tneg 25:0), with
 * bit 30 too (tneg 29:4, qds_neg 3, r_num 2:0).
 */
static uint32_t quantify_1b2t(int64_t sum, int64_t b, int64_t t, uint32_t tn,
                              int shift, unsigned obits, uint32_t offset)
{
    int64_t tneg = tn, thd, r;
    unsigned qds_neg = 0, r_num = 0, forced = 0;
    uint32_t q = 0;

    if (tn & BIT(31)) {
        bool low = tn & BIT(30);

        tneg = low ? tn & 0x3ffffff0 : extract32(tn, 0, 26);
        qds_neg = low ? extract32(tn, 3, 1) : extract32(tn, 29, 1);
        r_num = low ? extract32(tn, 0, 3) : extract32(tn, 26, 3);
    }
    sum = shift < 0 ? (sum + (1LL << (-shift - 1))) >> -shift : sum << shift;
    sum += b;
    if (sum < 0) {
        thd = tneg;
        q = ~0u;
        forced = r_num;
        r = sum + 2 * tneg - (tneg >> ((obits - 2 * r_num - 1) & 0x3f));
    } else {
        thd = t;
        r = sum - 2 * t + (t >> (obits - 1));
    }
    for (unsigned i = 0; i < obits / 2; i++) {
        q <<= 2;
        if (forced) {
            q |= 3;
            forced--;
            continue;
        }
        if (r < 0) {
            if (thd < -r) {
                r += thd;
            } else {
                q |= 1;
            }
            r += thd / 2;
        } else {
            if (r < thd) {
                q |= 2;
            } else {
                q |= 3;
                r -= thd;
            }
            r -= thd / 2;
        }
        r *= 4;
    }
    if (sum < 0) {
        q++;
        if (qds_neg) {
            q = -q;
        }
    }
    q += offset;
    return (int32_t)q < 0 ? 0 : MIN(q, (1u << obits) - 1);
}

/*
 * NNDWW and NNDWBT write at a pointer that WR_WADDR and BTR_WADDR set and
 * each load moves on: the weights of a layer that cross the ORAM's end
 * come in two loads, one after the other.
 */
static void ram_load(IngenicTnnaState *s, uint8_t *ram, size_t ram_size,
                     uint32_t *wptr, uint32_t cmd)
{
    uint32_t size = cmd >> 16, src = extract32(cmd, 0, 16) * ORAM_UNIT;

    for (uint32_t i = 0; i < size; i++) {
        ram[(*wptr * ORAM_UNIT + i) % ram_size] =
            s->oram[(src + i) % INGENIC_TNNA_ORAM_SIZE];
    }
    *wptr += size / ORAM_UNIT;
}

/*
 * Run WFCW entries lo to hi - 1, putting the F-RAM lines they write in
 * out (n of them, at most max): an LC entry (control 15) runs those after
 * it up to its end (bits 10:8) its count (bits 7:0) + 1 times; a WFINC
 * (control 1 to 14) writes that many lines, its signed increment apart;
 * an INCO (control 0) steps by its increment.
 */
static void wfcw_run(IngenicTnnaState *s, unsigned lo, unsigned hi,
                     unsigned len, uint32_t *out, unsigned *n, unsigned max,
                     int depth)
{
    unsigned i = lo;

    while (i < hi && i < len) {
        uint32_t ctrl = extract32(s->wfcw[i], 11, 4);
        uint32_t inc = extract32(s->wfcw[i], 0, 11);

        if (ctrl == WFCW_LC) {
            unsigned end = extract32(inc, 8, 3);

            for (unsigned k = 0; depth < 8 && k <= extract32(inc, 0, 8); k++) {
                wfcw_run(s, i + 1, end, len, out, n, max, depth + 1);
            }
            i = MAX(end, i + 1);
            continue;
        }
        if (ctrl == 0) {
            s->wf_line += sextract32(inc, 0, 11);
        }
        for (uint32_t k = 0; k < ctrl; k++) {
            if (*n < max) {
                out[(*n)++] = s->wf_line;
            }
            s->wf_line += sextract32(inc, 0, 11);
        }
        i++;
    }
}

static uint8_t *fram_entry(IngenicTnnaState *s, uint32_t entry)
{
    return s->fram[entry % INGENIC_TNNA_FRAM];
}

/*
 * ORAM to F-RAM: a column of 1 << icol pixels of each (plane, row) that
 * LDCF_CFG walks, each into the F-RAM line the WFCW program gives it,
 * cut in 4-bit slices FR_BBIT_STRIDE lines apart; a column of 4 pixels
 * fills one line of each FR_BELT_LEN-line belt.
 */
static void nndwf(IngenicTnnaState *s)
{
    uint32_t fr_cfg = reg(s, REG_FR_CFG), ldcf = reg(s, REG_LDCF_CFG);
    unsigned npx = 1 << (extract32(fr_cfg, 4, 2) + 2);
    unsigned fbits = 1 << (extract32(fr_cfg, 2, 2) + 2);
    unsigned ic = 8 >> extract32(fr_cfg, 0, 2);
    unsigned bpp = ic * fbits / 8;
    unsigned col_oa = MAX(128, 1 << (extract32(ldcf, 0, 4) + 7)) >> 7;
    unsigned rows = extract32(ldcf, 4, 4) + 1;
    unsigned planes = extract32(ldcf, 8, 8) + 1;
    uint32_t belt = reg(s, REG_FR_BELT_LEN);
    uint32_t bstride = reg(s, REG_FR_BBIT_STRIDE);
    uint32_t rbl = reg(s, REG_LDCF_RBLNUM);
    uint32_t p = s->or_raddr;
    g_autofree uint32_t *lines = g_new0(uint32_t, planes * rows);
    uint8_t col[128 * ORAM_UNIT];
    unsigned n = 0, k = 0;

    wfcw_run(s, 0, INGENIC_TNNA_WFCW, extract32(reg(s, REG_WFCW_STTEND), 0, 3),
             lines, &n, planes * rows, 0);
    memset(col, 0, sizeof(col));
    for (unsigned f = 0; f < planes; f++) {
        bool wrapped = false;

        for (unsigned r = 0; r < rows; r++, k++) {
            for (unsigned i = 0; i < col_oa; i++) {
                memcpy(&col[i * ORAM_UNIT],
                       &s->oram[(p + i) % ORAM_UNITS * ORAM_UNIT], ORAM_UNIT);
            }
            for (unsigned x = 0; x < npx && (x + 1) * bpp <= sizeof(col); x++) {
                const uint8_t *px = &col[x * bpp];

                for (unsigned sl = 0; sl < MAX(fbits / 4, 1); sl++) {
                    uint8_t *e = fram_entry(s, ((x >> 2) * belt + lines[k] +
                                                sl * bstride) * FRAM_LINE +
                                               (x & 3));

                    for (unsigned c = 0; c < INGENIC_TNNA_OC; c++) {
                        if (c >= ic) {
                            e[c] = 0;
                        } else if (fbits == 4) {
                            e[c] = extract32(px[c / 2], 4 * (c & 1), 4);
                        } else {
                            e[c] = extract32(px[c * fbits / 8 + sl / 2],
                                             4 * (sl & 1), 4);
                        }
                    }
                }
            }
            p += col_oa - 1;
            if (r + 1 < rows) {
                wrapped |= r == rbl;
                p += reg(s, r == rbl ? REG_LDCF_LP0RBINC : REG_LDCF_LP0INC);
            }
        }
        p += reg(s, wrapped ? REG_LDCF_LP1RBINC : REG_LDCF_LP1INC);
    }
    s->or_raddr = (s->or_raddr + reg(s, REG_OR_RADDR_INC)) % ORAM_UNITS;
}

/*
 * The pixel dx right and dy lines down of F-RAM entry addr (FRRA_INC in
 * Ingenic's nna_app.h): pixels go 4 to a line, lines FR_BELT_LEN to a belt
 */
static uint32_t fram_addr(uint32_t addr, uint32_t dx, uint32_t dy,
                          uint32_t belt)
{
    uint32_t x = dx + (addr & 3);

    return (addr & ~3) + (x >> 2) * belt * FRAM_LINE + (x & 3) +
           dy * FRAM_LINE;
}

/*
 * 16 lanes (2^LN rows of 2^AN pixels, MAC_FP) by 8 output channels, over
 * every input plane and kernel tap, added to the sums; a tap outside the
 * picture (MACTL_FP_X/Y against FP_W/FP_H) reads PAD_VALUE. The W-RAM
 * holds a tap's 8 x 8 weights as wslices entries of 4-bit slices, the
 * taps of each kernel row in turn, each plane's odd rows right to left.
 * Depthwise (MAC_CFG bit 0), output channel c takes input channel c alone
 * and its weights are a stream of nibbles, a tap's slices low first, the
 * taps in the same order, from WR_RADDR on: eight of them in each W-RAM
 * entry, at the entry's nibbles 8c to 8c + 7.
 */
static void nnmac(IngenicTnnaState *s)
{
    uint32_t kernel = reg(s, REG_MAC_KERNEL), bits = reg(s, REG_MAC_BIT);
    unsigned an = extract32(reg(s, REG_MAC_FP), 0, 4);
    unsigned kh = extract32(kernel, 4, 3) + 1, kw = extract32(kernel, 0, 3) + 1;
    unsigned sy = 1 << extract32(reg(s, REG_MAC_CFG), 8, 2);
    unsigned planes = reg(s, REG_FP_NUM) + 1;
    unsigned fslices = extract32(bits, 0, 2) + 1;
    unsigned wslices = extract32(bits, 4, 3) + 1;
    uint32_t belt = reg(s, REG_FR_BELT_LEN);
    uint32_t fstride = reg(s, REG_FR_FP_STRIDE);
    uint32_t bstride = reg(s, REG_FR_BBIT_STRIDE);
    uint32_t fr = reg(s, REG_FR_RADDR), wr = reg(s, REG_WR_RADDR);
    int fx = sextract32(reg(s, REG_MACTL_FP_X), 0, 12);
    int fy = sextract32(reg(s, REG_MACTL_FP_Y), 0, 12);
    int fw = reg(s, REG_FP_W), fh = reg(s, REG_FP_H);
    int64_t pad = extract32(reg(s, REG_PAD_VALUE), 0, 4 * fslices);
    int64_t wz = 1LL << (4 * wslices - 1);
    bool dw = reg(s, REG_MAC_CFG) & MAC_CFG_DW;

    for (unsigned f = 0; f < planes; f++) {
        for (unsigned ky = 0; ky < kh; ky++) {
            for (unsigned kx = 0; kx < kw; kx++) {
                unsigned kr = f * kh + ky;
                uint32_t went = wr + (kr * kw + (ky & 1 ? kw - 1 - kx : kx)) *
                                wslices;
                int64_t w[INGENIC_TNNA_OC][INGENIC_TNNA_OC];

                for (unsigned i = 0; i < 64; i++) {
                    unsigned oc = i / 8;
                    int64_t v = 0;

                    if (dw && i % 8 != oc) {
                        w[oc][i % 8] = 0;
                        continue;
                    }
                    for (unsigned sl = 0; sl < wslices; sl++) {
                        uint32_t m = went + sl;
                        /* the nibble's index in the W-RAM */
                        uint32_t at = dw ? m / 8 * 64 + oc * 8 + m % 8
                                         : m * 2 * WRAM_ENTRY + i;
                        uint8_t b = s->wram[at / 2 % INGENIC_TNNA_WRAM_SIZE];

                        v |= (int64_t)extract32(b, 4 * (at & 1), 4) << (4 * sl);
                    }
                    w[oc][i % 8] = v - wz;
                }
                for (unsigned p = 0; p < INGENIC_TNNA_LANES; p++) {
                    unsigned x = p & ((1 << an) - 1), row = p >> an;
                    int X = fx + x + kx, Y = fy + row * sy + ky;
                    int64_t in[INGENIC_TNNA_OC];

                    for (unsigned c = 0; c < INGENIC_TNNA_OC; c++) {
                        in[c] = pad;
                    }
                    if (X >= 0 && X <= fw && Y >= 0 && Y <= fh) {
                        uint32_t e = fram_addr(fr, x + kx, f * fstride +
                                               row * sy + ky, belt);

                        for (unsigned c = 0; c < INGENIC_TNNA_OC; c++) {
                            in[c] = 0;
                            for (unsigned sl = 0; sl < fslices; sl++) {
                                in[c] |= fram_entry(s, e + sl * bstride *
                                                    FRAM_LINE)[c] << (4 * sl);
                            }
                        }
                    }
                    for (unsigned oc = 0; oc < INGENIC_TNNA_OC; oc++) {
                        for (unsigned c = 0; c < INGENIC_TNNA_OC; c++) {
                            s->sum[p][oc] += w[oc][c] * in[c];
                        }
                    }
                }
            }
        }
    }
}

/*
 * QSUM: quantize the copied sums with the latched B and T into the
 * ORAM at OR_WADDR_Q (8-byte units): slot i takes lane PIXEL_OUT nibble
 * i, as many slots as MAC_P_MASK has lanes, 2^LN rows of them an output
 * line (OLINE_SIZE_M plus the column's 16-byte units less one) apart,
 * twice over. A pixel is its 8 channels of 4 bits (a byte per 2), 8 or 16
 * (little endian).
 */
static void qsum(IngenicTnnaState *s)
{
    static const int8_t sum_shift[8] = { 0, 5, 0, 0, -4, -8, -12, -16 };
    uint32_t q_cfg = reg(s, REG_Q_CFG);
    unsigned obits = (extract32(q_cfg, 4, 2) + 1) * 4;
    unsigned ln = extract32(reg(s, REG_MAC_FP), 4, 4);
    unsigned slots = ctpop32(reg(s, REG_MAC_P_MASK) & 0xffff);
    unsigned opx = MAX(slots >> ln, 1);
    unsigned col_oa = MAX(opx * obits / ORAM_UNIT, 1);
    uint32_t lsize = reg(s, REG_OLINE_SIZE_M) + col_oa - 1;
    uint32_t base = reg(s, REG_OR_WADDR_Q) * 8;

    if (obits == 12) {
        qemu_log_mask(LOG_UNIMP, "ingenic-tnna: 12-bit output\n");
        return;
    }
    for (unsigned slot = 0; slot < slots; slot++) {
        unsigned lane = extract32(reg(s, REG_PIXEL_OUT + slot / 4),
                                  4 * (slot % 4), 4);
        uint32_t o = base + slot % opx * obits + slot / opx * 2 * lsize * 8;
        uint8_t out[16] = { 0 };

        for (unsigned oc = 0; oc < INGENIC_TNNA_OC; oc++) {
            int shift = sum_shift[extract32(q_cfg, 0, 3)];
            uint32_t q = q_cfg & Q_CFG_1B2T ?
                quantify_1b2t(s->ocb[lane][oc], s->bt[0][oc], s->bt[1][oc],
                              s->bt[2][oc], shift, obits,
                              reg(s, REG_OF_OFFSET)) :
                quantify(s->ocb[lane][oc], s->bt[0][oc], s->bt[1][oc], shift,
                         obits);

            if (obits == 4) {
                out[oc / 2] |= q << (4 * (oc & 1));
            } else {
                for (unsigned b = 0; b < obits / 8; b++) {
                    out[oc * obits / 8 + b] = q >> (8 * b);
                }
            }
        }
        for (unsigned i = 0; i < obits; i++) {
            s->oram[(o + i) % INGENIC_TNNA_ORAM_SIZE] = out[i];
        }
    }
}

/*
 * The sums out (CPOC, CLROC, RDBT, QSUM, in that order), then the
 * address steps it asks for.
 */
static void nnmmt(IngenicTnnaState *s, uint32_t cmd)
{
    uint32_t op = cmd >> 12;
    unsigned an = extract32(reg(s, REG_MAC_FP), 0, 4);
    unsigned ln = extract32(reg(s, REG_MAC_FP), 4, 4);
    unsigned sy = 1 << extract32(reg(s, REG_MAC_CFG), 8, 2);
    unsigned inc;

    if (op & MMT_CPOC) {
        memcpy(s->ocb, s->sum, sizeof(s->ocb));
    }
    if (op & MMT_CLROC) {
        memset(s->sum, 0, sizeof(s->sum));
    }
    if (op & MMT_RDBT) {
        /* B and T (1B2T: and TN) of 8 output channels, then the next 8's */
        uint32_t bt = reg(s, REG_BTR_RADDR);
        unsigned n = reg(s, REG_Q_CFG) & Q_CFG_1B2T ? 3 : 2;

        for (unsigned h = 0; h < n; h++) {
            for (unsigned oc = 0; oc < INGENIC_TNNA_OC; oc++) {
                s->bt[h][oc] = ldl_le_p(&s->btram[((bt + h) * BTRAM_ENTRY +
                                                   4 * oc) %
                                                  INGENIC_TNNA_BTRAM_SIZE]);
            }
        }
        reg_set(s, REG_BTR_RADDR, bt + n);
    }
    if (op & MMT_QSUM) {
        qsum(s);
    }
    inc = extract32(op, 4, 2);
    if (inc) {
        reg_set(s, REG_OR_WADDR_Q, reg(s, REG_OR_WADDR_Q) +
                reg(s, REG_OR_WADDR_INC0 + inc));
    }
    inc = extract32(op, 6, 2);
    if (inc) {
        reg_set(s, REG_FR_RADDR, reg(s, REG_FR_RADDR) +
                reg(s, REG_FR_RADDR + inc));
    }
    if (op & MMT_WRRA_RST) {
        reg_set(s, REG_WR_RADDR, 0);
    }
    if (op & MMT_WRRA_RINC) {
        qemu_log_mask(LOG_UNIMP, "ingenic-tnna: NNMMT WRRA_RINC\n");
    }
    if (op & MMT_WRRA_RET) {
        reg_set(s, REG_WR_RADDR, reg(s, REG_WR_RADDR_RET));
    }
    if (op & MMT_WRRA_INC) {
        reg_set(s, REG_WR_RADDR, reg(s, REG_WR_RADDR) +
                reg(s, REG_WR_RADDR_INC));
    }
    switch (extract32(op, 16, 2)) {
    case 0:
        break;
    case MMT_TLYP:
        reg_set(s, REG_MACTL_FP_Y, reg(s, REG_MACTL_FP_Y) + (sy << ln));
        reg_set(s, REG_TLYP_ACC, reg(s, REG_TLYP_ACC) + 1);
        break;
    default:
        /* TLYM back up the rows TLYP went down; TLYMXP and right a tile */
        reg_set(s, REG_MACTL_FP_Y, reg(s, REG_MACTL_FP_Y) -
                reg(s, REG_TLYP_ACC) * (sy << ln));
        reg_set(s, REG_TLYP_ACC, 0);
        if (extract32(op, 16, 2) == MMT_TLYMXP) {
            reg_set(s, REG_MACTL_FP_X, reg(s, REG_MACTL_FP_X) + (1 << an));
        }
        break;
    }
    if (op & MMT_BTRRA_RST) {
        reg_set(s, REG_BTR_RADDR, 0);
    }
    if (op & MMT_BTRRA_RINC) {
        reg_set(s, REG_BTR_RADDR_RET, reg(s, REG_BTR_RADDR_RET) +
                reg(s, REG_BTR_RADDR_RINC));
    }
    if (op & MMT_BTRRA_RET) {
        reg_set(s, REG_BTR_RADDR, reg(s, REG_BTR_RADDR_RET));
    }
}

static void reg_write(IngenicTnnaState *s, uint32_t cmd)
{
    uint32_t idx = cmd >> 20;
    uint32_t val = cmd & REGDATA_MASK;
    uint32_t *wptr = &s->regs[REG_WFCW_WPTR - REG_FIRST];

    if (idx < ARRAY_SIZE(s->greg)) {
        if (greg_mask[idx]) {
            s->greg[idx] = val & greg_mask[idx];
        }
        return;
    }
    if (idx < REG_FIRST || idx >= REG_FIRST + INGENIC_TNNA_REGS) {
        qemu_log_mask(LOG_UNIMP, "ingenic-tnna: NNRW to register 0x%03x\n",
                      idx);
        return;
    }
    switch (idx) {
    case REG_WFCW_INP:
        s->wfcw[*wptr] = val & WFCW_MASK;
        *wptr = (*wptr + 1) % INGENIC_TNNA_WFCW;
        break;
    case REG_FR_WADDR_ADD:
        s->regs[REG_FR_WADDR - REG_FIRST] =
            (s->regs[REG_FR_WADDR - REG_FIRST] + sextract32(val, 0, 20)) &
            reg_mask[REG_FR_WADDR - REG_FIRST];
        break;
    default:
        reg_set(s, idx, val);
        /* the live pointers the loads move on start from these */
        if (idx == REG_OR_RADDR_F) {
            s->or_raddr = reg(s, idx);
        } else if (idx == REG_FR_WADDR) {
            s->wf_line = reg(s, idx);
        } else if (idx == REG_WR_WADDR) {
            s->wr_wptr = reg(s, idx);
        } else if (idx == REG_BTR_WADDR) {
            s->bt_wptr = reg(s, idx);
        }
        break;
    }
}

static uint32_t port_read(IngenicTnnaState *s, hwaddr offset)
{
    if (offset < 0x20) {
        return s->greg[offset / 4];
    }
    if (offset >= PORT_REGS && offset < PORT_FSUM) {
        uint32_t idx = REG_FIRST + (offset - PORT_REGS) / 4;

        if (reg_alias[idx - REG_FIRST]) {
            idx = reg_alias[idx - REG_FIRST];
        }
        return s->regs[idx - REG_FIRST];
    }
    if (offset >= PORT_WFCW && offset < PORT_PC) {
        return s->wfcw[(offset - PORT_WFCW) / 4 % INGENIC_TNNA_WFCW];
    }
    switch (offset) {
    case PORT_WAITNNMAC:
        return 0x14;
    case PORT_WAITNNDW:
    case PORT_WAITRDBT:
    default:
        /* waits return at once: every command has finished */
        return 0;
    }
}

static void port_write(IngenicTnnaState *s, hwaddr offset, uint32_t val)
{
    switch (offset) {
    case PORT_NNRST:
        /*
         * every register back to its reset, WFCW_WPTR too; each WFCW
         * entry's control field to 1, its other bits kept
         */
        memcpy(s->greg, greg_reset, sizeof(s->greg));
        memcpy(s->regs, reg_reset, sizeof(s->regs));
        for (int i = 0; i < INGENIC_TNNA_WFCW; i++) {
            s->wfcw[i] = (s->wfcw[i] & WFCW_INC_MASK) | WFCW_CTRL_1;
        }
        /* the load pointers too, with the registers they start from */
        s->or_raddr = s->wf_line = s->wr_wptr = s->bt_wptr = 0;
        break;
    case PORT_NNRW:
        reg_write(s, val);
        break;
    case PORT_NNMAC:
        nnmac(s);
        break;
    case PORT_NNMMT:
        nnmmt(s, val);
        break;
    case PORT_INITCW:
        s->wf_line = reg(s, REG_FR_WADDR);
        break;
    case PORT_NNDWF:
        nndwf(s);
        break;
    case PORT_NNDWW:
        ram_load(s, s->wram, sizeof(s->wram), &s->wr_wptr, val);
        break;
    case PORT_NNDWBT:
        ram_load(s, s->btram, sizeof(s->btram), &s->bt_wptr, val);
        break;
    case PORT_ENDSIG:
        /* the work queued before it has finished: all of it runs at once */
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-tnna: store to port +0x%"
                      HWADDR_PRIx "\n", offset);
        break;
    }
}

static MemTxResult tnna_read(void *opaque, hwaddr offset, uint64_t *data,
                             unsigned size, MemTxAttrs attrs)
{
    IngenicTnnaState *s = opaque;

    if (attrs.requester_id != INGENIC_TNPU_REQUESTER) {
        *data = offset < INGENIC_TNNA_RAM_SIZE ? ldl_le_p(&s->ram[offset]) : 0;
    } else if (offset < TNNA_PORTS) {
        *data = port_read(s, offset);
    } else if (offset >= TNNA_DMA && offset < TNNA_DMA + 0xc0) {
        uint32_t ch = (offset - TNNA_DMA) / 0x40, r = offset % 0x40 / 4;

        *data = r == DMA_RUN_WAIT ? 0 : s->dma[ch][r];
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-tnna: core load from +0x%"
                      HWADDR_PRIx "\n", offset);
        *data = 0;
    }
    return MEMTX_OK;
}

static MemTxResult tnna_write(void *opaque, hwaddr offset, uint64_t data,
                              unsigned size, MemTxAttrs attrs)
{
    IngenicTnnaState *s = opaque;

    if (attrs.requester_id != INGENIC_TNPU_REQUESTER) {
        if (offset < INGENIC_TNNA_RAM_SIZE) {
            stl_le_p(&s->ram[offset], data);
        }
    } else if (offset < TNNA_PORTS) {
        port_write(s, offset, data);
    } else if (offset >= TNNA_DMA && offset < TNNA_DMA + 0xc0) {
        uint32_t ch = (offset - TNNA_DMA) / 0x40, r = offset % 0x40 / 4;

        if (r == DMA_RUN_WAIT || r == DMA_OCVA || r == DMA_ONNA) {
            if (data & 1) {
                dma_run(s, ch);
            }
        } else if (ch != DMA_FOUT || r < DMA_EREGL(1)) {
            s->dma[ch][r] = data;
        }
    } else {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-tnna: core store to +0x%"
                      HWADDR_PRIx "\n", offset);
    }
    return MEMTX_OK;
}

static const MemoryRegionOps tnna_ops = {
    .read_with_attrs = tnna_read,
    .write_with_attrs = tnna_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

/*
 * NV12 lines in the ORAM to B, G, R, alpha bytes in the ORAM (R, G, B,
 * alpha with CTRL bit 24): each colour a row of the matrix on Y, U and V
 * plus its bias, all Q20, rounded half up and clamped; the rows for B, G
 * and R take Y and U, Y, U and V, and Y and V.
 */
static void cva_run(IngenicTnnaState *s)
{
    const uint32_t *r = s->cva;
    uint32_t width = extract32(r[CVA_WH / 4], 0, 16);
    uint32_t lines = extract32(r[CVA_WH / 4], 16, 16);
    uint32_t in_stride = extract32(r[CVA_STRIDE / 4], 0, 16);
    uint32_t out_stride = extract32(r[CVA_STRIDE / 4], 20, 12) * ORAM_UNIT;
    uint8_t alpha = extract32(r[CVA_CTRL / 4], 16, 8);
    bool rgba = extract32(r[CVA_CTRL / 4], 24, 1);
    const int32_t *c = (const int32_t *)&r[CVA_COEF / 4];
    const int32_t *bias = (const int32_t *)&r[CVA_BIAS / 4];

    for (uint32_t l = 0; l < lines; l++) {
        for (uint32_t x = 0; x < width; x++) {
            uint32_t uv = r[CVA_UVSRC / 4] + l / 2 * in_stride + (x & ~1);
            int64_t y = s->oram[(r[CVA_YSRC / 4] + l * in_stride + x) %
                                INGENIC_TNNA_ORAM_SIZE];
            int64_t u = s->oram[uv % INGENIC_TNNA_ORAM_SIZE];
            int64_t v = s->oram[(uv + 1) % INGENIC_TNNA_ORAM_SIZE];
            int64_t bgr[3] = {
                c[0] * y + c[1] * u + bias[0],
                c[2] * y + c[3] * u + c[4] * v + bias[1],
                c[5] * y + c[6] * v + bias[2],
            };
            uint32_t out = r[CVA_DST / 4] + l * out_stride + 4 * x;

            for (int i = 0; i < 3; i++) {
                s->oram[(out + (rgba ? 2 - i : i)) % INGENIC_TNNA_ORAM_SIZE] =
                    MIN(MAX((bgr[i] + (1 << 19)) >> 20, 0), 255);
            }
            s->oram[(out + 3) % INGENIC_TNNA_ORAM_SIZE] = alpha;
        }
    }
    s->cva[CVA_IRQ / 4] = 1;
}

static uint64_t cva_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicTnnaState *s = opaque;

    /* CTL bit 0, busy, is clear: a run has finished as it started */
    return offset / 4 < CVA_REGS && offset != CVA_CTL ? s->cva[offset / 4] : 0;
}

static void cva_write(void *opaque, hwaddr offset, uint64_t data,
                      unsigned size)
{
    IngenicTnnaState *s = opaque;

    if (offset == CVA_CTL) {
        if (data & CVA_CTL_START) {
            cva_run(s);
        }
    } else if (offset / 4 < CVA_REGS) {
        s->cva[offset / 4] = data;
    }
}

static const MemoryRegionOps cva_ops = {
    .read = cva_read,
    .write = cva_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void tnna_reset_hold(Object *obj, ResetType type)
{
    IngenicTnnaState *s = INGENIC_TNNA(obj);

    memcpy(s->greg, greg_reset, sizeof(s->greg));
    memcpy(s->regs, reg_reset, sizeof(s->regs));
    for (int i = 0; i < INGENIC_TNNA_WFCW; i++) {
        s->wfcw[i] = WFCW_CTRL_1;
    }
    memset(s->dma, 0, sizeof(s->dma));
    for (int ch = 0; ch < 3; ch++) {
        if (ch != DMA_FOUT) {
            s->dma[ch][DMA_EREGH(0)] = s->dma[ch][DMA_EREGH(1)] = 1 << 16;
        }
    }
    memset(s->oram, 0, sizeof(s->oram));
    memset(s->ram, 0, sizeof(s->ram));
    memset(s->cva, 0, sizeof(s->cva));
    memset(s->fram, 0, sizeof(s->fram));
    memset(s->wram, 0, sizeof(s->wram));
    memset(s->btram, 0, sizeof(s->btram));
    memset(s->sum, 0, sizeof(s->sum));
    memset(s->ocb, 0, sizeof(s->ocb));
    memset(s->bt, 0, sizeof(s->bt));
    s->wf_line = 0;
    s->or_raddr = 0;
    s->wr_wptr = 0;
    s->bt_wptr = 0;
}

static void tnna_init(Object *obj)
{
    IngenicTnnaState *s = INGENIC_TNNA(obj);

    memory_region_init_io(&s->iomem, obj, &tnna_ops, s, "ingenic-tnna",
                          TNNA_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    memory_region_init_io(&s->cva_iomem, obj, &cva_ops, s, "ingenic-tnna.cva",
                          CVA_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->cva_iomem);
}

static const VMStateDescription vmstate_tnna = {
    .name = "ingenic-tnna",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(greg, IngenicTnnaState, 8),
        VMSTATE_UINT32_ARRAY(regs, IngenicTnnaState, INGENIC_TNNA_REGS),
        VMSTATE_UINT32_ARRAY(wfcw, IngenicTnnaState, INGENIC_TNNA_WFCW),
        VMSTATE_UINT32_2DARRAY(dma, IngenicTnnaState, 3,
                               INGENIC_TNNA_DMA_REGS),
        VMSTATE_UINT8_ARRAY(oram, IngenicTnnaState, INGENIC_TNNA_ORAM_SIZE),
        VMSTATE_UINT8_ARRAY(ram, IngenicTnnaState, INGENIC_TNNA_RAM_SIZE),
        VMSTATE_UINT32_ARRAY(cva, IngenicTnnaState, INGENIC_TNNA_CVA_REGS),
        VMSTATE_UINT8_2DARRAY(fram, IngenicTnnaState, INGENIC_TNNA_FRAM,
                              INGENIC_TNNA_OC),
        VMSTATE_UINT8_ARRAY(wram, IngenicTnnaState, INGENIC_TNNA_WRAM_SIZE),
        VMSTATE_UINT8_ARRAY(btram, IngenicTnnaState, INGENIC_TNNA_BTRAM_SIZE),
        VMSTATE_2DARRAY(sum, IngenicTnnaState, INGENIC_TNNA_LANES,
                        INGENIC_TNNA_OC, 0, vmstate_info_int64, int64_t),
        VMSTATE_2DARRAY(ocb, IngenicTnnaState, INGENIC_TNNA_LANES,
                        INGENIC_TNNA_OC, 0, vmstate_info_int64, int64_t),
        VMSTATE_2DARRAY(bt, IngenicTnnaState, 3, INGENIC_TNNA_OC, 0,
                        vmstate_info_int32, int32_t),
        VMSTATE_UINT32(wf_line, IngenicTnnaState),
        VMSTATE_UINT32(or_raddr, IngenicTnnaState),
        VMSTATE_UINT32(wr_wptr, IngenicTnnaState),
        VMSTATE_UINT32(bt_wptr, IngenicTnnaState),
        VMSTATE_END_OF_LIST()
    }
};

static void tnna_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_tnna;
    rc->phases.hold = tnna_reset_hold;
}

static const TypeInfo tnna_type_info = {
    .name = TYPE_INGENIC_TNNA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicTnnaState),
    .instance_init = tnna_init,
    .class_init = tnna_class_init,
};

static void tnna_register_types(void)
{
    type_register_static(&tnna_type_info);
}

type_init(tnna_register_types)
