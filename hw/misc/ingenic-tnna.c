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
 * and sizes are in 16-byte units, DDR addresses in bytes.
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
 * Registers, their resets and the bits each keeps, the DMA and the CVA's
 * pixels are a T32LQ's, driven from its TNPU core. Not modelled yet: the
 * commands that load the F, W and BT RAMs and compute (NNDWF, NNDWW,
 * NNDWBT, INITCW, NNMAC, NNMMT), which are logged once.
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
#define REG_FR_WADDR        0x50
#define REG_FR_WADDR_ADD    0x51
#define REG_WFCW_WPTR       0x58
#define REG_WFCW_INP        0x59
#define REGDATA_MASK        0xfffff
#define WFCW_MASK           0x7fff
#define WFCW_INC_MASK       0x7ff
#define WFCW_CTRL_1         0x800

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

static const char *const port_name[] = {
    [0x08 / 4] = "NNMAC", [0x0c / 4] = "NNMMT", [0x10 / 4] = "INITCW",
    [0x14 / 4] = "NNDWF", [0x18 / 4] = "NNDWW", [0x1c / 4] = "NNDWBT",
    [PORT_ENDSIG / 4] = "ENDSIG",
};

static void oram_xfer(IngenicTnnaState *s, bool to_oram, hwaddr ddr,
                      uint32_t oram, uint32_t units)
{
    for (uint32_t i = 0; i < units; i++) {
        uint8_t *p = &s->oram[((oram + i) % ORAM_UNITS) * ORAM_UNIT];

        if (to_oram) {
            address_space_read(&address_space_memory, ddr + i * ORAM_UNIT,
                               MEMTXATTRS_UNSPECIFIED, p, ORAM_UNIT);
        } else {
            address_space_write(&address_space_memory, ddr + i * ORAM_UNIT,
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
        s->regs[idx - REG_FIRST] = val & reg_mask[idx - REG_FIRST];
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
    static uint32_t logged;

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
        break;
    case PORT_NNRW:
        reg_write(s, val);
        break;
    default:
        if (offset / 4 < ARRAY_SIZE(port_name) && port_name[offset / 4]) {
            if (!(logged & BIT(offset / 4))) {
                logged |= BIT(offset / 4);
                qemu_log_mask(LOG_UNIMP, "ingenic-tnna: %s not modelled\n",
                              port_name[offset / 4]);
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-tnna: store to port "
                          "+0x%" HWADDR_PRIx "\n", offset);
        }
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
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(greg, IngenicTnnaState, 8),
        VMSTATE_UINT32_ARRAY(regs, IngenicTnnaState, INGENIC_TNNA_REGS),
        VMSTATE_UINT32_ARRAY(wfcw, IngenicTnnaState, INGENIC_TNNA_WFCW),
        VMSTATE_UINT32_2DARRAY(dma, IngenicTnnaState, 3,
                               INGENIC_TNNA_DMA_REGS),
        VMSTATE_UINT8_ARRAY(oram, IngenicTnnaState, INGENIC_TNNA_ORAM_SIZE),
        VMSTATE_UINT8_ARRAY(ram, IngenicTnnaState, INGENIC_TNNA_RAM_SIZE),
        VMSTATE_UINT32_ARRAY(cva, IngenicTnnaState, INGENIC_TNNA_CVA_REGS),
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
