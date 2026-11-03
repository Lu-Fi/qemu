/*
 * Ingenic NNA: the T40's, T41's and A1's neural network accelerator
 *
 * The CPU drives it with MXU3's six NN instructions (NNRWR, NNRRD, NNDWR,
 * NNDRD, NNMAC, NNCMD) and feeds it through its own DMA, the NNDMA, which
 * copies between DDR and the ORAM. Its 2048 64-bit descriptors fill the
 * first 16 KiB of the window at 0x12500000:
 *   bits 31:6   DDR address
 *   bits 5:1    (bytes / 64 - 1) bits 16:12
 *   bit 0       another descriptor follows
 *   bits 63:52  (bytes / 64 - 1) bits 11:0
 *   bits 51:32  ORAM address - 0x12600000, in 64-byte units
 * A T40XP keeps only bits 2:1 of the length's high part and bits 45:32 of
 * the ORAM address ("desc-mask"); a write to the T40's ORAM below the L2's
 * share lands in the L2's own SRAM on silicon.
 * Its channels are at +0x8000: +0x0, +0x4 and +0x8 copy DDR to ORAM, +0x10
 * and +0x14 ORAM to DDR (+0xc does nothing), on a T41NQ and a T40XP alike.
 * Writing one sets the index of its first descriptor in bits 10:0, and
 * bit 31 starts it; reading one gives the index after the last descriptor
 * done in bits 10:0 and minus the 64-byte units copied in bits 24:11. The
 * CPU waits for channel n with RDHWR $8 + n (+0x10 and +0x14 are $11 and
 * $12), which stalls until the channel is done and reads 0: the copies
 * finish at once here.
 *
 * The NN instructions drive a convolution engine (registers 0x20-0x5f as
 * NNRWR numbers them, the same map as the T32's TNNA), all of it a
 * T41NQ's, layer for layer, on the instructions Ingenic's
 * personvehiclepetDet sample runs:
 *   NNDWR 0x00/0x02/0x04/0x07 fill the four 64-byte quarters of a 256-byte
 *     W entry at WR_WADDR (0x5c), 0x07 then moves it on. A weight of wbit
 *     bits is wbit/2 slices of 2 bits, slice s in entry s of its group, an
 *     entry [32 oc][32 ic x 2 bits, LSB first]; it weighs 2w - (2^wbit-1).
 *   NNDWR 0x40 fills BT at BTR_WADDR (0x5e): per 32 oc, B[32] then T[32].
 *   NNDWR 0x20/0x21 write F through ports 0 and 1 into 1024 lines of 128
 *     bytes: one slot of a port's WFCW program (T32 words: count, step) per
 *     row of 4 pixels, its line from FR_WADDR (0x50, +0x51 for port 1).
 *   NNMAC: 8 pixels (2 rows of 4 with 0x38 bit 4) x the taps of 0x3e (W
 *     groups in snake order: row 1 reversed) x 0x3c + 1 planes of 32 input
 *     channels; pixel (r, c) tap (ky, kx) reads the pixel
 *     word + (r * Sy + ky) * 0x39 + c * Sx + kx + plane * 0x37 * vec / 8,
 *     4 pixels to a line, vec bytes each (8 << 0x55), or pad (0x2f) when
 *     MACTL_FP_X/Y (0x2b/0x29) put it outside FP_W/H (0x2e/0x2c). Bit 0 of
 *     the immediate moves the W read pointer past what it used; 0x30 sets
 *     that pointer and 0x34 steps it on from its last start.
 *   NNCMD bits 0-3: CPOC, CLROC, RDBT (BT at 0x42, which steps 4), QSUM:
 *     (sum << {1, 5, 10, 1}[0x43]) + B, divided by T's low 26 bits, plus
 *     half the range, clamped to obits (2 * 0x44 + 2); bits 7:5 move the
 *     tile: 5 down a MAC's rows, 7 back, 6 back and right a block.
 *   NNDRD 0x20 reads the QSUM output, 0x00-0x0f the copied sums.
 * Not modelled yet: 2- and 16-bit data, the output pixel order (0x48,
 * 0x49), WFCW's invert-MSB bit, what the registers read back beyond what
 * was written to them, the output a NNDRD reads before any QSUM (the
 * previous contents), and a second window a T41NQ also reads, 120 pixels
 * on, in a 4-bit 3x3 layout with two planes and a stride of 2 (where the
 * sample's layers keep nothing).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-nna.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

#define NNDMA_REGS          0x8000
#define NNDMA_DESCS         2048
#define NNDMA_START         BIT(31)
#define NNDMA_SRAM          0x12600000  /* the L2's SRAM, then the ORAM */

#define NN_LINE             128
#define NN_LINES            (INGENIC_NNA_FRAM / NN_LINE)

/* the register values NNCMD 0 sets; the rest are 0 */
static const struct {
    uint8_t reg;
    uint16_t value;
} nn_reset[] = {
    { 0x20, 1 }, { 0x23, 1 }, { 0x29, 0xfff }, { 0x2b, 0xfff },
    { 0x2c, 0x7ff }, { 0x2e, 0x7ff }, { 0x33, 1 }, { 0x37, 0x50 },
    { 0x38, 0x12 }, { 0x39, 0x14 }, { 0x3e, 0x1ff }, { 0x3f, 0xff },
    { 0x48, 0x3210 }, { 0x49, 0x7654 },
};

/* the bits each register keeps; 0 = all */
static const uint16_t nn_mask[INGENIC_NNA_REGS] = {
    [0x20] = 0x1, [0x21] = 0x3, [0x23] = 0xf, [0x28] = 0x3, [0x29] = 0xfff,
    [0x2a] = 0xff, [0x2b] = 0xfff, [0x2c] = 0x7ff, [0x2d] = 0xffff,
    [0x2e] = 0x7ff, [0x2f] = 0xfff, [0x30] = 0xffff, [0x31] = 0xffff,
    [0x32] = 0x1ff, [0x33] = 0x1, [0x34] = 0x1ff, [0x37] = 0x1fc,
    [0x38] = 0x13, [0x39] = 0x3c, [0x3a] = 0x11, [0x3b] = 0xff, [0x3c] = 0xf,
    [0x3d] = 0xf, [0x3e] = 0x1ff, [0x3f] = 0xff, [0x42] = 0xff, [0x43] = 0x3,
    [0x44] = 0x7, [0x45] = 0xf, [0x46] = 0x3, [0x47] = 0x1f, [0x48] = 0xffff,
    [0x49] = 0xffff, [0x50] = 0x3ff, [0x51] = 0x3ff, [0x52] = 0x3ff,
    [0x53] = 0x3ff, [0x54] = 0x3, [0x55] = 0x3, [0x58] = 0xf, [0x59] = 0x7fff,
    [0x5a] = 0x77, [0x5b] = 0x3, [0x5c] = 0xffff, [0x5e] = 0xff,
};

/* W tap group g is kernel position nn_snake[g] (row, column) */
static const uint8_t nn_snake[9][2] = {
    { 0, 0 }, { 0, 1 }, { 0, 2 }, { 1, 2 }, { 1, 1 }, { 1, 0 },
    { 2, 0 }, { 2, 1 }, { 2, 2 },
};

static const uint8_t nn_qshift[4] = { 1, 5, 10, 1 };

/* ingenic_nna_op's op: MXU3_NN_* in target/mips/cpu.h */
enum { NN_RWR, NN_RRD, NN_DWR, NN_DRD, NN_MAC, NN_CMD };

static int nn_sreg12(IngenicNnaState *s, unsigned r)
{
    return sextract32(s->reg[r], 0, 12);
}

static unsigned nn_wbit(IngenicNnaState *s)
{
    return extract32(s->reg[0x3b], 3, 3) + 2;
}

static unsigned nn_ifbit(IngenicNnaState *s)
{
    return extract32(s->reg[0x3b], 0, 3) * 2 + 2;
}

static unsigned nn_vec(IngenicNnaState *s)
{
    return 8 << s->reg[0x55];
}

static void nn_reset_engine(IngenicNnaState *s)
{
    memset(s->reg, 0, sizeof(s->reg));
    for (int i = 0; i < ARRAY_SIZE(nn_reset); i++) {
        s->reg[nn_reset[i].reg] = nn_reset[i].value;
    }
    s->wfcw_ptr = s->wr_w = s->bt_w = s->wr_r = s->wr_base = 0;
    memset(s->port_line, 0, sizeof(s->port_line));
    memset(s->port_half, 0, sizeof(s->port_half));
    memset(s->port_slot, 0, sizeof(s->port_slot));
    memset(s->sums, 0, sizeof(s->sums));
    memset(s->ocb, 0, sizeof(s->ocb));
    s->bt_valid = false;
    s->out_len = s->out_rd = 0;
    s->slots_dirty = true;
}

/* unroll a port's WFCW words: a write count and step, LC loops, jumps */
static void nn_unroll(IngenicNnaState *s, unsigned p, unsigned lo, unsigned hi,
                      unsigned depth)
{
    const uint16_t *prog = &s->wfcw[8 * p];

    for (unsigned k = lo; k < hi && depth < 4;) {
        unsigned ctrl = extract32(prog[k], 11, 4);
        int inc = sextract32(prog[k], 0, 10);

        if (ctrl == 0xf) {
            unsigned lpend = extract32(prog[k], 8, 3);

            for (unsigned n = 0; n <= extract32(prog[k], 0, 8); n++) {
                nn_unroll(s, p, k + 1, lpend, depth + 1);
            }
            k = MAX(lpend, k + 1);
            continue;
        }
        for (unsigned n = 0; n < MAX(ctrl, 1); n++) {
            if (s->nslots[p] == INGENIC_NNA_SLOTS) {
                return;
            }
            s->slot_inc[p][s->nslots[p]] = inc;
            s->slot_jump[p][s->nslots[p]++] = !ctrl;
        }
        k++;
    }
}

static void nn_slots(IngenicNnaState *s)
{
    if (!s->slots_dirty) {
        return;
    }
    for (unsigned p = 0; p < 2; p++) {
        unsigned n = extract32(s->reg[0x5a], 4 * p, 3);

        s->nslots[p] = 0;
        if (n) {
            nn_unroll(s, p, 0, n, 0);
        } else {
            s->slot_inc[p][0] = 1;
            s->slot_jump[p][0] = false;
            s->nslots[p] = 1;
        }
    }
    s->slots_dirty = false;
}

static void nn_fwrite(IngenicNnaState *s, unsigned p, const uint8_t *data)
{
    unsigned slot = 4 * nn_vec(s);
    uint32_t o = (s->port_line[p] * NN_LINE + s->port_half[p] * 64) %
                 INGENIC_NNA_FRAM;
    unsigned n;

    memcpy(&s->fram[o], data, 64);
    if (++s->port_half[p] * 64 < slot) {
        return;
    }
    s->port_half[p] = 0;
    nn_slots(s);
    n = s->nslots[p];
    if (!n) {
        return;
    }
    do {
        unsigned k = s->port_slot[p]++ % n;

        s->port_line[p] = (s->port_line[p] + s->slot_inc[p][k]) % NN_LINES;
    } while (s->slot_jump[p][s->port_slot[p] % n] &&
             s->port_slot[p] % n);
}

/* the 32 input channels of pixel address px */
static void nn_fpixel(IngenicNnaState *s, int64_t px, int f[32])
{
    unsigned nb = nn_vec(s), bits = nn_ifbit(s);
    int64_t o = (px >> 2) * NN_LINE + (px & 3) * nb;
    uint8_t b[64];

    o %= INGENIC_NNA_FRAM;
    if (o < 0) {
        o += INGENIC_NNA_FRAM;
    }
    for (unsigned i = 0; i < nb; i++) {
        b[i] = s->fram[(o + i) % INGENIC_NNA_FRAM];
    }
    for (unsigned i = 0; i < 32; i++) {
        f[i] = bits == 8 ? b[i] : b[i / 2] >> (4 * (i & 1)) & 0xf;
    }
}

/* Weff[oc][ic] of the tap group starting at W entry e */
static void nn_wgroup(IngenicNnaState *s, uint32_t e, int w[32][32])
{
    unsigned wbit = nn_wbit(s), sl = wbit / 2;
    int zp = (1 << wbit) - 1;

    for (unsigned oc = 0; oc < 32; oc++) {
        for (unsigned ic = 0; ic < 32; ic++) {
            unsigned v = 0;

            for (unsigned k = 0; k < sl; k++) {
                uint32_t o = ((e + k) * 256 + oc * 8 + ic / 4) %
                             INGENIC_NNA_WRAM;

                v |= (s->wram[o] >> (2 * (ic % 4)) & 3) << (2 * k);
            }
            w[oc][ic] = 2 * v - zp;
        }
    }
}

static void nn_mac(IngenicNnaState *s, unsigned imm, int32_t word)
{
    unsigned taps[9], ntaps = 0;
    unsigned sx = s->reg[0x3a] & 1 ? 2 : 1, sy = s->reg[0x3a] & 0x10 ? 2 : 1;
    unsigned planes = s->reg[0x3c] + 1, sl = nn_wbit(s) / 2;
    bool rows2 = s->reg[0x38] & 0x10;
    int fx = nn_sreg12(s, 0x2b), fy = nn_sreg12(s, 0x29);
    int fw = s->reg[0x2e], fh = s->reg[0x2c];
    int pad = s->reg[0x2f] & ((1 << nn_ifbit(s)) - 1);
    int64_t pl_px = s->reg[0x37] * nn_vec(s) / 8;
    int w[32][32], f[32];

    if (nn_ifbit(s) != 8 && nn_ifbit(s) != 4) {
        qemu_log_mask(LOG_UNIMP, "ingenic-nna: %u-bit features\n",
                      nn_ifbit(s));
        return;
    }
    for (unsigned t = 0; t < 9; t++) {
        if (s->reg[0x3e] & BIT(t)) {
            taps[ntaps++] = t;
        }
    }
    for (unsigned i = 0; i < ntaps; i++) {
        unsigned ky = nn_snake[taps[i]][0], kx = nn_snake[taps[i]][1];

        for (unsigned k = 0; k < planes; k++) {
            nn_wgroup(s, s->wr_r + (k * ntaps + i) * sl, w);
            for (unsigned p = 0; p < 8; p++) {
                unsigned r = rows2 ? p >> 2 : 0, c = rows2 ? p & 3 : p;
                int x = fx + c * sx + kx, y = fy + r * sy + ky;
                int64_t *sum = &s->sums[p * 32];

                if (!(s->reg[0x3f] & BIT(p))) {
                    continue;
                }
                if (x < 0 || x > fw || y < 0 || y > fh) {
                    for (unsigned ic = 0; ic < 32; ic++) {
                        f[ic] = pad;
                    }
                } else {
                    nn_fpixel(s, (int64_t)word + (r * sy + ky) * s->reg[0x39] +
                              c * sx + kx + k * pl_px, f);
                }
                for (unsigned oc = 0; oc < 32; oc++) {
                    int64_t acc = 0;

                    for (unsigned ic = 0; ic < 32; ic++) {
                        acc += w[oc][ic] * f[ic];
                    }
                    sum[oc] += acc;
                }
            }
        }
    }
    if (imm & 1) {
        s->wr_r += ntaps * planes * sl;
    }
    s->reg[0x2d] = word & 0xffff;
}

static uint8_t nn_quant(int64_t r, int32_t t, unsigned obits)
{
    int64_t hi = (1 << obits) - 1, q;

    t &= 0x3ffffff;
    if (!t) {
        return r >= 0 ? hi : 0;
    }
    /* floor division */
    q = r / t - (r % t && r < 0);
    return MIN(MAX(q + (1 << (obits - 1)), 0), hi);
}

static void nn_qsum(IngenicNnaState *s)
{
    unsigned obits = s->reg[0x44] * 2 + 2, sh = nn_qshift[s->reg[0x43]];
    unsigned len = obits * 32;          /* 8 pixels x 32 channels */
    uint8_t *o;

    if (obits != 8 && obits != 4) {
        qemu_log_mask(LOG_UNIMP, "ingenic-nna: %u-bit outputs\n", obits);
        return;
    }
    if (!s->bt_valid || s->out_len + len > INGENIC_NNA_OUT) {
        return;
    }
    o = &s->out[s->out_len];
    memset(o, 0, len);
    for (unsigned i = 0; i < 256; i++) {
        uint8_t q = nn_quant(s->ocb[i] * (1 << sh) + s->bt[i % 32],
                             s->bt[32 + i % 32], obits);

        if (obits == 8) {
            o[i] = q;
        } else {
            o[i / 2] |= q << (4 * (i & 1));
        }
    }
    s->out_len += len;
}

static void nn_cmd(IngenicNnaState *s, unsigned imm)
{
    unsigned move = extract32(imm, 5, 3);
    bool rows2 = s->reg[0x38] & 0x10;

    if (!imm) {
        nn_reset_engine(s);
        return;
    }
    if (imm & 1) {
        memcpy(s->ocb, s->sums, sizeof(s->ocb));
    }
    if (imm & 2) {
        memset(s->sums, 0, sizeof(s->sums));
    }
    if (imm & 4) {
        for (unsigned i = 0; i < 64; i++) {
            s->bt[i] = ldl_le_p(&s->btram[(s->reg[0x42] * 64 + i * 4) %
                                          INGENIC_NNA_BTRAM]);
        }
        s->bt_valid = true;
        s->reg[0x42] = (s->reg[0x42] + 4) & 0xff;
    }
    if (imm & 8) {
        nn_qsum(s);
    }
    if (move == 5) {
        unsigned d = (rows2 ? 2 : 1) * (s->reg[0x3a] & 0x10 ? 2 : 1);

        s->reg[0x29] = (s->reg[0x29] + d) & 0xfff;
        s->reg[0x2a] = (s->reg[0x2a] + d) & 0xff;
    } else if (move == 6 || move == 7) {
        s->reg[0x29] = (s->reg[0x29] - s->reg[0x2a]) & 0xfff;
        s->reg[0x2a] = 0;
        if (move == 6) {
            s->reg[0x2b] = (s->reg[0x2b] + (rows2 ? 4 : 8) *
                            (s->reg[0x3a] & 1 ? 2 : 1)) & 0xfff;
        }
    }
}

static void nn_rwr(IngenicNnaState *s, unsigned r, uint32_t v)
{
    if (r >= INGENIC_NNA_REGS) {
        return;
    }
    if (nn_mask[r]) {
        v &= nn_mask[r];
    }
    s->reg[r] = v;
    switch (r) {
    case 0x30:
        s->wr_base = s->wr_r = v;
        s->reg[0x34] = 0;
        break;
    case 0x34:
        s->wr_base += v;
        s->wr_r = s->wr_base;
        break;
    case 0x50:
    case 0x51:
        s->port_line[r & 1] = r == 0x50 ? v : (s->reg[0x50] + v) % NN_LINES;
        s->port_half[r & 1] = s->port_slot[r & 1] = 0;
        break;
    case 0x58:
        s->wfcw_ptr = v;
        break;
    case 0x59:
        s->wfcw[s->wfcw_ptr++ & 15] = v;
        s->slots_dirty = true;
        break;
    case 0x5a:
        s->slots_dirty = true;
        break;
    case 0x5c:
        s->wr_w = v;
        break;
    case 0x5e:
        s->bt_w = v;
        break;
    }
}

static void nn_dwr(IngenicNnaState *s, unsigned imm, const uint8_t *data)
{
    if (imm < 0x20) {
        memcpy(&s->wram[(s->wr_w * 256 + extract32(imm, 1, 2) * 64) %
                        INGENIC_NNA_WRAM], data, 64);
        s->wr_w += imm & 1;
    } else if (imm == 0x40) {
        memcpy(&s->btram[(s->bt_w++ * 64) % INGENIC_NNA_BTRAM], data, 64);
    } else if (imm == 0x20 || imm == 0x21) {
        nn_fwrite(s, imm & 1, data);
    } else {
        qemu_log_mask(LOG_UNIMP, "ingenic-nna: NNDWR 0x%x\n", imm);
    }
}

static void nn_drd(IngenicNnaState *s, unsigned imm, uint8_t *vec)
{
    memset(vec, 0, 64);
    if (imm & 0x20) {
        if (s->out_rd < s->out_len) {
            memcpy(vec, &s->out[s->out_rd], MIN(64, s->out_len - s->out_rd));
            s->out_rd += 64;
        }
        if (s->out_rd >= s->out_len) {
            s->out_len = s->out_rd = 0;
        }
    } else if (imm < 0x10) {
        for (unsigned i = 0; i < 16; i++) {
            stl_le_p(&vec[4 * i], s->ocb[(imm & 0xf) * 16 + i]);
        }
    }
}

/* register groups by imm bits 6:5, two 16-bit registers to a word */
static void nn_rrd(IngenicNnaState *s, unsigned imm, uint8_t *vec)
{
    unsigned group = extract32(imm, 5, 2);

    memset(vec, 0, 64);
    for (unsigned k = 0; k < 16; k++) {
        uint32_t v = 0;

        if (group == 3 || (group == 2 && (imm & 1))) {
            v = k < 8 ? s->wfcw[2 * k] | s->wfcw[2 * k + 1] << 16 : 0;
        } else if (group) {
            unsigned r = 0x20 * group + 2 * k;

            v = (s->reg[r] & 0xffff) | (s->reg[r + 1] & 0xffff) << 16;
        }
        stl_le_p(&vec[4 * k], v);
    }
}

void ingenic_nna_op(void *opaque, unsigned op, uint32_t imm, uint8_t *vec)
{
    IngenicNnaState *s = opaque;

    trace_ingenic_nna_op(op, imm, ldl_le_p(vec));
    switch (op) {
    case NN_RWR:
        nn_rwr(s, imm, ldl_le_p(vec));
        break;
    case NN_RRD:
        nn_rrd(s, imm, vec);
        break;
    case NN_DWR:
        nn_dwr(s, imm, vec);
        break;
    case NN_DRD:
        nn_drd(s, imm, vec);
        break;
    case NN_MAC:
        nn_mac(s, imm, ldl_le_p(vec));
        break;
    case NN_CMD:
        nn_cmd(s, imm);
        break;
    }
    if (s->log) {
        uint32_t head[2] = { cpu_to_le32(op), cpu_to_le32(imm) };

        fwrite(head, sizeof(head), 1, s->log);
        fwrite(vec, 64, 1, s->log);
    }
}

/* the channels at +0x8000, by register: 1 to the ORAM, 0 to DDR, -1 none */
static const int8_t nndma_to_oram[INGENIC_NNA_NNDMA_REGS] = {
    1, 1, 1, -1, 0, 0, -1, -1,
};

static void nndma_copy(hwaddr dst, hwaddr src, uint32_t bytes)
{
    uint8_t buf[4 * KiB];

    while (bytes) {
        uint32_t n = MIN(bytes, sizeof(buf));

        address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                           buf, n);
        address_space_write(&address_space_memory, dst, MEMTXATTRS_UNSPECIFIED,
                            buf, n);
        src += n;
        dst += n;
        bytes -= n;
    }
}

static void nndma_run(IngenicNnaState *s, unsigned reg)
{
    unsigned idx = s->nndma[reg] & (NNDMA_DESCS - 1);
    uint32_t units = 0;
    uint64_t d;
    int i;

    for (i = 0; i < NNDMA_DESCS; i++) {
        uint32_t lo, hi, n;
        hwaddr ddr, oram;

        d = ldq_le_p(&s->window[idx * 8]);
        lo = d;
        hi = d >> 32;
        n = extract32(lo, 1, 5) << 12 | extract32(hi, 20, 12);
        ddr = lo & ~0x3fu;
        oram = NNDMA_SRAM + (hwaddr)extract32(hi, 0, 20) * 64;
        if (nndma_to_oram[reg]) {
            nndma_copy(oram, ddr, (n + 1) * 64);
        } else {
            nndma_copy(ddr, oram, (n + 1) * 64);
        }
        units += n + 1;
        idx = (idx + 1) & (NNDMA_DESCS - 1);
        if (!(lo & 1)) {
            break;
        }
    }
    if (i == NNDMA_DESCS) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-nna: NNDMA chain at +0x%x "
                      "never ends\n", reg * 4);
    }
    s->nndma[reg] = (-units & 0x3fff) << 11 | idx;
}

static bool nndma_reg(hwaddr offset, unsigned *reg)
{
    *reg = (offset - NNDMA_REGS) / 4;
    return offset >= NNDMA_REGS && *reg < INGENIC_NNA_NNDMA_REGS;
}

static uint64_t nna_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicNnaState *s = opaque;
    unsigned reg;
    uint64_t v;

    if (nndma_reg(offset, &reg)) {
        v = s->nndma[reg];
        trace_ingenic_nna_read(offset, size, v);
        return v;
    }
    v = ldn_le_p(&s->window[offset], size);

    trace_ingenic_nna_read(offset, size, v);
    return v;
}

static void nna_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    IngenicNnaState *s = opaque;
    unsigned reg;

    trace_ingenic_nna_write(offset, size, value);
    if (nndma_reg(offset, &reg)) {
        if (nndma_to_oram[reg] < 0) {
            qemu_log_mask(LOG_UNIMP, "ingenic-nna: NNDMA +0x%x = 0x%" PRIx64
                          "\n", reg * 4, value);
            return;
        }
        s->nndma[reg] = value & (NNDMA_DESCS - 1);
        if (value & NNDMA_START) {
            nndma_run(s, reg);
        }
        return;
    }
    stn_le_p(&s->window[offset], size, value);
    if (offset < NNDMA_DESCS * 8) {
        hwaddr w = offset & ~3;
        uint32_t keep = extract64(s->desc_mask, (w & 4) * 8, 32);

        stl_le_p(&s->window[w], ldl_le_p(&s->window[w]) & keep);
    }
}

static const MemoryRegionOps nna_ops = {
    .read = nna_read,
    .write = nna_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void nna_reset_hold(Object *obj, ResetType type)
{
    IngenicNnaState *s = INGENIC_NNA(obj);

    memset(s->nndma, 0, sizeof(s->nndma));
    nn_reset_engine(s);
}

static int nna_post_load(void *opaque, int version_id)
{
    IngenicNnaState *s = opaque;

    s->slots_dirty = true;
    return 0;
}

static const VMStateDescription vmstate_nna = {
    .name = TYPE_INGENIC_NNA,
    .version_id = 2,
    .minimum_version_id = 2,
    .post_load = nna_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(window, IngenicNnaState, INGENIC_NNA_WINDOW),
        VMSTATE_UINT32_ARRAY(nndma, IngenicNnaState, INGENIC_NNA_NNDMA_REGS),
        VMSTATE_UINT32_ARRAY(reg, IngenicNnaState, INGENIC_NNA_REGS),
        VMSTATE_UINT16_ARRAY(wfcw, IngenicNnaState, 16),
        VMSTATE_UINT32(wfcw_ptr, IngenicNnaState),
        VMSTATE_UINT32(wr_w, IngenicNnaState),
        VMSTATE_UINT32(bt_w, IngenicNnaState),
        VMSTATE_UINT32(wr_r, IngenicNnaState),
        VMSTATE_UINT32(wr_base, IngenicNnaState),
        VMSTATE_UINT32_ARRAY(port_line, IngenicNnaState, 2),
        VMSTATE_UINT32_ARRAY(port_half, IngenicNnaState, 2),
        VMSTATE_UINT32_ARRAY(port_slot, IngenicNnaState, 2),
        VMSTATE_INT64_ARRAY(sums, IngenicNnaState, 8 * 32),
        VMSTATE_INT64_ARRAY(ocb, IngenicNnaState, 8 * 32),
        VMSTATE_INT32_ARRAY(bt, IngenicNnaState, 64),
        VMSTATE_BOOL(bt_valid, IngenicNnaState),
        VMSTATE_UINT8_ARRAY(out, IngenicNnaState, INGENIC_NNA_OUT),
        VMSTATE_UINT32(out_len, IngenicNnaState),
        VMSTATE_UINT32(out_rd, IngenicNnaState),
        VMSTATE_UINT8_ARRAY(wram, IngenicNnaState, INGENIC_NNA_WRAM),
        VMSTATE_UINT8_ARRAY(btram, IngenicNnaState, INGENIC_NNA_BTRAM),
        VMSTATE_UINT8_ARRAY(fram, IngenicNnaState, INGENIC_NNA_FRAM),
        VMSTATE_END_OF_LIST()
    }
};

static void nna_realize(DeviceState *dev, Error **errp)
{
    IngenicNnaState *s = INGENIC_NNA(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &nna_ops, s, "ingenic-nna",
                          INGENIC_NNA_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    if (!memory_region_init_ram(&s->oram, OBJECT(s), "ingenic-nna.oram",
                                s->oram_size, errp)) {
        return;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->oram);
    if (s->log_path) {
        s->log = fopen(s->log_path, "wb");
        if (!s->log) {
            error_setg_errno(errp, errno, "ingenic-nna: cannot open %s",
                             s->log_path);
        }
    }
}

static const Property nna_properties[] = {
    DEFINE_PROP_UINT32("oram-size", IngenicNnaState, oram_size, 384 * KiB),
    /* the descriptor bits the RAM keeps, high word over low */
    DEFINE_PROP_UINT64("desc-mask", IngenicNnaState, desc_mask, UINT64_MAX),
    DEFINE_PROP_STRING("log", IngenicNnaState, log_path),
};

static void nna_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = nna_realize;
    dc->vmsd = &vmstate_nna;
    rc->phases.hold = nna_reset_hold;
    device_class_set_props(dc, nna_properties);
}

static const TypeInfo nna_type_info = {
    .name = TYPE_INGENIC_NNA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicNnaState),
    .class_init = nna_class_init,
};

static void nna_register_types(void)
{
    type_register_static(&nna_type_info);
}

type_init(nna_register_types)
