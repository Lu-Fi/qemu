/*
 *  Ingenic XBurst MXU2 (128-bit VPR SIMD) translation routines.
 *
 *  MXU2 extends the XBurst1 ISA with 16 x 128-bit Vector Processing
 *  Registers (VPR0-VPR15) and ~368 SIMD operations. Instructions are
 *  encoded in two opcode spaces:
 *
 *    - SPECIAL2 (opcode 0x1C): LU1Q/SU1Q load/store, immediate shifts,
 *      replicate, saturate, boolean select, shuffle.
 *    - COP2 CO (opcode 0x12, bit25=1): arithmetic, compare, logic,
 *      multiply, min/max, float, unary operations on VPR registers.
 *
 *  Reference: Ingenic XBurst ISA MXU2 Programming Manual and
 *  mxu2_shim.h encoding tables verified on T20/T31/T32 hardware.
 *
 *  Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "qemu/osdep.h"
#include "translate.h"
#include "tcg/tcg-op-gvec.h"

/*
 * VPR register access helpers.
 *
 * Each VPR is stored as two uint64_t in TCState:
 *   mxu2_vpr[reg][0] = bits  63:0   (low)
 *   mxu2_vpr[reg][1] = bits 127:64  (high)
 *
 * This matches the convention used by QEMU's MSA and LoongArch
 * vector register layouts on little-endian targets.
 */

static TCGv_i64 mxu2_vpr_d[64]; /* [reg*2] = lo, [reg*2+1] = hi */

static inline int vpr_offset(int reg, int half)
{
    return offsetof(CPUMIPSState, active_tc.mxu2_vpr[reg][half]);
}

static inline int vpr_full_offset(int reg)
{
    return vpr_offset(reg, 0);
}

void mxu2_translate_init(void)
{
    for (int i = 0; i < 32; i++) {
        char lo_name[8], hi_name[8];
        snprintf(lo_name, sizeof(lo_name), "vr%d_lo", i);
        snprintf(hi_name, sizeof(hi_name), "vr%d_hi", i);

        mxu2_vpr_d[i * 2] = tcg_global_mem_new_i64(
            tcg_env, vpr_offset(i, 0), lo_name);
        mxu2_vpr_d[i * 2 + 1] = tcg_global_mem_new_i64(
            tcg_env, vpr_offset(i, 1), hi_name);
    }
}

/*
 * SPECIAL2 MXU2 instructions.
 *
 * These share the SPECIAL2 opcode (0x1C) with MXU1. When ASE_MXU2 is
 * set, the MXU2 decoder runs first and claims the funct codes it owns.
 * Unrecognized funct codes return false, allowing MXU1 to try next.
 *
 * LU1Q encoding: (0x1C<<26)|(base<<21)|(offset_idx<<11)|(vpr<<6)|0x14
 *   address = GPR[base] + sign_extend(offset_idx)
 *   VPR[vpr] = MEM[address], 128-bit aligned load
 *
 * SU1Q encoding: (0x1C<<26)|(base<<21)|(offset_idx<<11)|(vpr<<6)|0x1C
 *   address = GPR[base] + sign_extend(offset_idx)
 *   MEM[address] = VPR[vpr], 128-bit aligned store
 */
static void gen_mxu2_lu1q(DisasContext *ctx)
{
    int base = (ctx->opcode >> 21) & 0x1f;
    int offset_raw = (ctx->opcode >> 11) & 0x3ff;
    int vpr = (ctx->opcode >> 6) & 0x1f;
    int offset = (offset_raw >= 512) ? offset_raw - 1024 : offset_raw;
    TCGv addr;
    TCGv_i64 lo, hi;

    (void)vpr; /* 5-bit field, always 0-31, all VPRs valid */

    addr = tcg_temp_new();
    if (base == 0) {
        tcg_gen_movi_tl(addr, offset);
    } else {
        gen_load_gpr(addr, base);
        if (offset != 0) {
            tcg_gen_addi_tl(addr, addr, offset);
        }
    }

    lo = tcg_temp_new_i64();
    hi = tcg_temp_new_i64();
    tcg_gen_qemu_ld_i64(lo, addr, ctx->mem_idx, MO_TEUQ);
    tcg_gen_addi_tl(addr, addr, 8);
    tcg_gen_qemu_ld_i64(hi, addr, ctx->mem_idx, MO_TEUQ);

    tcg_gen_mov_i64(mxu2_vpr_d[vpr * 2], lo);
    tcg_gen_mov_i64(mxu2_vpr_d[vpr * 2 + 1], hi);
}

static void gen_mxu2_su1q(DisasContext *ctx)
{
    int base = (ctx->opcode >> 21) & 0x1f;
    int offset_raw = (ctx->opcode >> 11) & 0x3ff;
    int vpr = (ctx->opcode >> 6) & 0x1f;
    int offset = (offset_raw >= 512) ? offset_raw - 1024 : offset_raw;
    TCGv addr;
    TCGv_i64 lo, hi;

    (void)vpr; /* 5-bit field, always 0-31, all VPRs valid */

    addr = tcg_temp_new();
    if (base == 0) {
        tcg_gen_movi_tl(addr, offset);
    } else {
        gen_load_gpr(addr, base);
        if (offset != 0) {
            tcg_gen_addi_tl(addr, addr, offset);
        }
    }

    lo = tcg_temp_new_i64();
    hi = tcg_temp_new_i64();
    tcg_gen_mov_i64(lo, mxu2_vpr_d[vpr * 2]);
    tcg_gen_mov_i64(hi, mxu2_vpr_d[vpr * 2 + 1]);

    tcg_gen_qemu_st_i64(lo, addr, ctx->mem_idx, MO_TEUQ);
    tcg_gen_addi_tl(addr, addr, 8);
    tcg_gen_qemu_st_i64(hi, addr, ctx->mem_idx, MO_TEUQ);
}

/*
 * SPECIAL2 immediate shift/compare/replicate operations.
 *
 * funct=0x38: sats/satu/slli  (rs encodes size*8 + variant)
 *   rs= 4: slli.b,  12: slli.h,  20: slli.w,  28: slli.d
 *   rs= 0: sats.b,   8: sats.h,  16: sats.w,  24: sats.d
 *   rs= 2: satu.b,  10: satu.h,  18: satu.w,  26: satu.d
 *
 * funct=0x39: srai/srari/srli/srlri
 *   rs= 0: srai.b,   8: srai.h,  16: srai.w,  24: srai.d
 *   rs= 2: srari.b, 10: srari.h, 18: srari.w, 26: srari.d
 *   rs= 4: srli.b,  12: srli.h,  20: srli.w,  28: srli.d
 *   rs= 6: srlri.b, 14: srlri.h, 22: srlri.w, 30: srlri.d
 *
 * funct=0x35: repi (replicate immediate to all lanes)
 *   rs= 0: repi.b,  8: repi.h, 16: repi.w, 24: repi.d
 *
 * Common encoding: (0x1C<<26)|(rs<<21)|(imm<<16)|(vs<<11)|(vd<<6)|funct
 *   imm: 5-bit immediate (bits 20:16)
 *   vs: source VPR (bits 15:11)
 *   vd: destination VPR (bits 10:6)
 */
static void gen_mxu2_slli(DisasContext *ctx, int vece)
{
    int imm = (ctx->opcode >> 16) & 0x1f;
    int vs = (ctx->opcode >> 11) & 0x1f;
    int vd = (ctx->opcode >> 6) & 0x1f;

    (void)vs; (void)vd; /* 5-bit fields, all VPRs valid */

    tcg_gen_gvec_shli(vece, vpr_full_offset(vd), vpr_full_offset(vs),
                       imm, 16, 16);
}

static void gen_mxu2_srai(DisasContext *ctx, int vece)
{
    int imm = (ctx->opcode >> 16) & 0x1f;
    int vs = (ctx->opcode >> 11) & 0x1f;
    int vd = (ctx->opcode >> 6) & 0x1f;

    (void)vs; (void)vd; /* 5-bit fields, all VPRs valid */

    tcg_gen_gvec_sari(vece, vpr_full_offset(vd), vpr_full_offset(vs),
                       imm, 16, 16);
}

static void gen_mxu2_srli(DisasContext *ctx, int vece)
{
    int imm = (ctx->opcode >> 16) & 0x1f;
    int vs = (ctx->opcode >> 11) & 0x1f;
    int vd = (ctx->opcode >> 6) & 0x1f;

    (void)vs; (void)vd; /* 5-bit fields, all VPRs valid */

    tcg_gen_gvec_shri(vece, vpr_full_offset(vd), vpr_full_offset(vs),
                       imm, 16, 16);
}

static void gen_mxu2_repi(DisasContext *ctx, int vece)
{
    int idx = (ctx->opcode >> 16) & 0x1f;
    int vd = (ctx->opcode >> 6) & 0x1f;
    int64_t val;

    (void)vd; /* 5-bit field, all VPRs valid */

    val = (int64_t)(int8_t)(idx | (idx & 0x10 ? 0xe0 : 0));
    tcg_gen_gvec_dup_imm(vece, vpr_full_offset(vd), 16, 16, val);
}

static bool decode_mxu2_special2_imm(DisasContext *ctx, int funct)
{
    int rs = (ctx->opcode >> 21) & 0x1f;
    int variant = rs & 0x7;
    int size_sel = (rs >> 3) & 0x3;
    static const int vece_map[] = { MO_8, MO_16, MO_32, MO_64 };

    if (size_sel > 3) {
        return false;
    }

    switch (funct) {
    case 0x38: /* sats/satu/slli */
        switch (variant) {
        case 4: /* slli */
            gen_mxu2_slli(ctx, vece_map[size_sel]);
            return true;
        case 0: /* sats - saturate signed (approximate as NOP for now) */
        case 2: /* satu - saturate unsigned */
            /* TODO: implement saturate with helper */
            return true;
        default:
            return false;
        }

    case 0x39: /* srai/srari/srli/srlri */
        switch (variant) {
        case 0: /* srai */
            gen_mxu2_srai(ctx, vece_map[size_sel]);
            return true;
        case 4: /* srli */
            gen_mxu2_srli(ctx, vece_map[size_sel]);
            return true;
        case 2: /* srari - shift right arithmetic with rounding */
        case 6: /* srlri - shift right logical with rounding */
            /* TODO: implement rounding variants with helper */
            gen_mxu2_srai(ctx, vece_map[size_sel]);
            return true;
        default:
            return false;
        }

    case 0x35: /* repi */
        gen_mxu2_repi(ctx, vece_map[size_sel]);
        return true;

    default:
        return false;
    }
}

/*
 * LA1Q / SA1Q - aligned 128-bit VPR load/store.
 *
 * Same field layout as LU1Q/SU1Q but offset is in units of 16 bytes
 * (binutils operand type '=L' vs '=l' for LU1Q/SU1Q):
 *   (0x1C<<26)|(base<<21)|(offset_idx<<11)|(vpr<<6)|funct
 *   address = GPR[base] + sign_extend(offset_idx) * 16
 *
 * la1q funct=0x2C, sa1q funct=0x3C
 */
static void gen_mxu2_la1q(DisasContext *ctx)
{
    int base = (ctx->opcode >> 21) & 0x1f;
    int offset_raw = (ctx->opcode >> 11) & 0x3ff;
    int vpr = (ctx->opcode >> 6) & 0x1f;
    int offset = ((offset_raw >= 512) ? offset_raw - 1024 : offset_raw) * 16;
    TCGv addr;
    TCGv_i64 lo, hi;

    (void)vpr; /* 5-bit field, always 0-31, all VPRs valid */

    addr = tcg_temp_new();
    if (base == 0) {
        tcg_gen_movi_tl(addr, offset);
    } else {
        gen_load_gpr(addr, base);
        if (offset != 0) {
            tcg_gen_addi_tl(addr, addr, offset);
        }
    }

    lo = tcg_temp_new_i64();
    hi = tcg_temp_new_i64();
    tcg_gen_qemu_ld_i64(lo, addr, ctx->mem_idx, MO_TEUQ);
    tcg_gen_addi_tl(addr, addr, 8);
    tcg_gen_qemu_ld_i64(hi, addr, ctx->mem_idx, MO_TEUQ);

    tcg_gen_mov_i64(mxu2_vpr_d[vpr * 2], lo);
    tcg_gen_mov_i64(mxu2_vpr_d[vpr * 2 + 1], hi);
}

static void gen_mxu2_sa1q(DisasContext *ctx)
{
    int base = (ctx->opcode >> 21) & 0x1f;
    int offset_raw = (ctx->opcode >> 11) & 0x3ff;
    int vpr = (ctx->opcode >> 6) & 0x1f;
    int offset = ((offset_raw >= 512) ? offset_raw - 1024 : offset_raw) * 16;
    TCGv addr;
    TCGv_i64 lo, hi;

    (void)vpr; /* 5-bit field, always 0-31, all VPRs valid */

    addr = tcg_temp_new();
    if (base == 0) {
        tcg_gen_movi_tl(addr, offset);
    } else {
        gen_load_gpr(addr, base);
        if (offset != 0) {
            tcg_gen_addi_tl(addr, addr, offset);
        }
    }

    lo = tcg_temp_new_i64();
    hi = tcg_temp_new_i64();
    tcg_gen_mov_i64(lo, mxu2_vpr_d[vpr * 2]);
    tcg_gen_mov_i64(hi, mxu2_vpr_d[vpr * 2 + 1]);

    tcg_gen_qemu_st_i64(lo, addr, ctx->mem_idx, MO_TEUQ);
    tcg_gen_addi_tl(addr, addr, 8);
    tcg_gen_qemu_st_i64(hi, addr, ctx->mem_idx, MO_TEUQ);
}

/*
 * BEQZ / BNEZ - VPR conditional branch.
 *
 * beqz encoding: (0x1C<<26)|(sz<<21)|(vpr<<16)|(offset<<6)|0x28
 *   sz: 0=16b, 1=8h, 2=4w, 4=1q  (bits 25:21)
 *   vpr: source VPR register (bits 20:16)
 *   offset: signed branch offset in instruction units (bits 15:6)
 *
 * bnez: same but with bit 23 set (sz | 4)
 *
 * beqz: branch if ALL lanes of VPR are zero
 * bnez: branch if ANY lane of VPR is non-zero
 *
 * For strlen, the key use is: check if any byte in the loaded 128-bit
 * vector is zero (null terminator search).
 */
static void gen_mxu2_vpr_branch(DisasContext *ctx)
{
    int rs_field = (ctx->opcode >> 21) & 0x1f;
    int vpr = (ctx->opcode >> 16) & 0x1f;
    int offset_raw = (ctx->opcode >> 6) & 0x3ff;
    int offset = (offset_raw >= 512) ? offset_raw - 1024 : offset_raw;
    bool is_bnez = !!(rs_field & 4);
    int size_sel = rs_field & 3;
    TCGv_i64 lo, hi, has_zero;
    TCGv_i64 cond64;

    (void)vpr; /* 5-bit field, always 0-31, all VPRs valid */

    lo = tcg_temp_new_i64();
    hi = tcg_temp_new_i64();
    has_zero = tcg_temp_new_i64();

    tcg_gen_mov_i64(lo, mxu2_vpr_d[vpr * 2]);
    tcg_gen_mov_i64(hi, mxu2_vpr_d[vpr * 2 + 1]);

    /*
     * Per-element zero detection.
     *
     *   beqz16b: branch if ANY byte == 0
     *   bnez16b: branch if NO byte == 0
     *
     * Required for libc functions (strerror, etc.) that work on
     * tightly packed string tables where adjacent strings have no
     * 16-byte all-zero padding between them.
     *
     * SWAR tricks per element size:
     *   byte:  ((v - 0x01010101...) & ~v & 0x80808080...) != 0
     *   half:  ((v - 0x00010001...) & ~v & 0x80008000...) != 0
     *   word:  ((v - 0x00000001...) & ~v & 0x80000000...) != 0
     *   dword: just OR both halves (whole-vector zero test)
     */
    {
        uint64_t mask01, mask80;
        bool use_swar = true;

        switch (size_sel) {
        case 0:  /* 16b - byte lanes */
            mask01 = 0x0101010101010101ULL;
            mask80 = 0x8080808080808080ULL;
            break;
        case 1:  /* 8h - halfword lanes */
            mask01 = 0x0001000100010001ULL;
            mask80 = 0x8000800080008000ULL;
            break;
        case 2:  /* 4w - word lanes */
            mask01 = 0x0000000100000001ULL;
            mask80 = 0x8000000080000000ULL;
            break;
        default:
            /* 2d (3) or 1q (4 - separate funct 0x29): whole-vector test */
            use_swar = false;
            mask01 = 0;
            mask80 = 0;
            break;
        }

        if (!use_swar) {
            tcg_gen_or_i64(has_zero, lo, hi);
        } else {
            TCGv_i64 t1 = tcg_temp_new_i64();
            TCGv_i64 t2 = tcg_temp_new_i64();
            TCGv_i64 t3 = tcg_temp_new_i64();
            TCGv_i64 zlo = tcg_temp_new_i64();
            TCGv_i64 zhi = tcg_temp_new_i64();

            /* Compute zlo = ((lo - mask01) & ~lo) & mask80 */
            tcg_gen_subi_i64(t1, lo, mask01);
            tcg_gen_not_i64(t2, lo);
            tcg_gen_and_i64(t3, t1, t2);
            tcg_gen_andi_i64(zlo, t3, mask80);

            /* Compute zhi = ((hi - mask01) & ~hi) & mask80 */
            tcg_gen_subi_i64(t1, hi, mask01);
            tcg_gen_not_i64(t2, hi);
            tcg_gen_and_i64(t3, t1, t2);
            tcg_gen_andi_i64(zhi, t3, mask80);

            tcg_gen_or_i64(has_zero, zlo, zhi);
        }
    }

    /*
     * has_zero != 0 means "found a zero lane".
     *   beqz: branch if any zero lane found  -> bcond = (has_zero != 0)
     *   bnez: branch if no zero lane found   -> bcond = (has_zero == 0)
     */
    cond64 = tcg_temp_new_i64();
    if (is_bnez) {
        tcg_gen_setcondi_i64(TCG_COND_EQ, cond64, has_zero, 0);
    } else {
        tcg_gen_setcondi_i64(TCG_COND_NE, cond64, has_zero, 0);
    }
    tcg_gen_extrl_i64_i32(bcond, cond64);

    ctx->btarget = ctx->base.pc_next + (offset << 2);
    ctx->hflags |= MIPS_HFLAG_BC | MIPS_HFLAG_BDS32;
}

/*
 * MTCPUSB / MTCPUUB - Move VPR element to CPU GPR.
 *
 * mtcpusb: (0x1C<<26)|(sz<<21)|(vpr<<16)|(elem<<11)|(rd<<6)|0x33
 *   Extracts element from VPR, sign-extends to 32 bits, writes to GPR[rd].
 *
 * mtcpuub: same with funct=0x34, zero-extends instead.
 */
static void gen_mxu2_mtcpu(DisasContext *ctx, bool sign_extend)
{
    int rs = (ctx->opcode >> 21) & 0x1f;
    int vpr = (ctx->opcode >> 16) & 0x1f;
    int elem = (ctx->opcode >> 11) & 0x1f;
    int rd = (ctx->opcode >> 6) & 0x1f;
    int size_sel = (rs >> 3) & 0x3;
    TCGv_i64 src;
    TCGv val;
    int half, bit_offset;

    if (rd == 0) {
        return;
    }

    val = tcg_temp_new();
    src = tcg_temp_new_i64();

    switch (size_sel) {
    case 0: /* byte */
        half = (elem >= 8) ? 1 : 0;
        bit_offset = (elem & 7) * 8;
        tcg_gen_mov_i64(src, mxu2_vpr_d[vpr * 2 + half]);
        if (bit_offset) {
            tcg_gen_shri_i64(src, src, bit_offset);
        }
        tcg_gen_extrl_i64_i32(val, src);
        if (sign_extend) {
            tcg_gen_ext8s_tl(val, val);
        } else {
            tcg_gen_andi_tl(val, val, 0xff);
        }
        break;
    case 1: /* halfword */
        half = (elem >= 4) ? 1 : 0;
        bit_offset = (elem & 3) * 16;
        tcg_gen_mov_i64(src, mxu2_vpr_d[vpr * 2 + half]);
        if (bit_offset) {
            tcg_gen_shri_i64(src, src, bit_offset);
        }
        tcg_gen_extrl_i64_i32(val, src);
        if (sign_extend) {
            tcg_gen_ext16s_tl(val, val);
        } else {
            tcg_gen_andi_tl(val, val, 0xffff);
        }
        break;
    case 2: /* word */
        half = (elem >= 2) ? 1 : 0;
        bit_offset = (elem & 1) * 32;
        tcg_gen_mov_i64(src, mxu2_vpr_d[vpr * 2 + half]);
        if (bit_offset) {
            tcg_gen_shri_i64(src, src, bit_offset);
        }
        tcg_gen_extrl_i64_i32(val, src);
        break;
    default:
        tcg_gen_movi_tl(val, 0);
        break;
    }

    gen_store_gpr(val, rd);
}

/*
 * INSFCPU - Insert GPR value into VPR element.
 *
 * insfcpub: (0x1C<<26)|(sz<<21)|(vpr<<16)|(elem<<11)|(rs<<6)|0x31
 */
static void gen_mxu2_insfcpu(DisasContext *ctx)
{
    int rs_field = (ctx->opcode >> 21) & 0x1f;
    int vpr = (ctx->opcode >> 16) & 0x1f;
    int elem = (ctx->opcode >> 11) & 0x1f;
    int rs = (ctx->opcode >> 6) & 0x1f;
    int size_sel = (rs_field >> 3) & 0x3;
    TCGv val;
    TCGv_i64 src, mask, ins;
    int half, bit_offset;

    (void)vpr; /* 5-bit field, always 0-31, all VPRs valid */

    val = tcg_temp_new();
    gen_load_gpr(val, rs);

    src = tcg_temp_new_i64();
    mask = tcg_temp_new_i64();
    ins = tcg_temp_new_i64();

    tcg_gen_extu_tl_i64(ins, val);

    switch (size_sel) {
    case 0: /* byte */
        half = (elem >= 8) ? 1 : 0;
        bit_offset = (elem & 7) * 8;
        tcg_gen_andi_i64(ins, ins, 0xff);
        tcg_gen_shli_i64(ins, ins, bit_offset);
        tcg_gen_movi_i64(mask, ~((uint64_t)0xff << bit_offset));
        tcg_gen_and_i64(src, mxu2_vpr_d[vpr * 2 + half], mask);
        tcg_gen_or_i64(mxu2_vpr_d[vpr * 2 + half], src, ins);
        break;
    case 1: /* halfword */
        half = (elem >= 4) ? 1 : 0;
        bit_offset = (elem & 3) * 16;
        tcg_gen_andi_i64(ins, ins, 0xffff);
        tcg_gen_shli_i64(ins, ins, bit_offset);
        tcg_gen_movi_i64(mask, ~((uint64_t)0xffff << bit_offset));
        tcg_gen_and_i64(src, mxu2_vpr_d[vpr * 2 + half], mask);
        tcg_gen_or_i64(mxu2_vpr_d[vpr * 2 + half], src, ins);
        break;
    case 2: /* word */
        half = (elem >= 2) ? 1 : 0;
        bit_offset = (elem & 1) * 32;
        tcg_gen_andi_i64(ins, ins, 0xffffffffULL);
        tcg_gen_shli_i64(ins, ins, bit_offset);
        tcg_gen_movi_i64(mask, ~((uint64_t)0xffffffff << bit_offset));
        tcg_gen_and_i64(src, mxu2_vpr_d[vpr * 2 + half], mask);
        tcg_gen_or_i64(mxu2_vpr_d[vpr * 2 + half], src, ins);
        break;
    default:
        break;
    }
}

/*
 * COP2 CO (VPR arithmetic) operations.
 *
 * Encoding: (18<<26)|(1<<25)|(major<<21)|(vt<<16)|(vs<<11)|(vd<<6)|minor
 *
 * Major groups:
 *   0: Compare/MinMax/Shift-by-register
 *   1: Add/Sub/Average/SLL-by-register
 *   2: Mul/Div/Mod/Madd/Msub/Dot
 *   6: 128-bit logic (andv, norv, orv, xorv)
 *   8: Float arith/compare/Q-format
 *  14: Unary int/float ops
 */

static void gen_mxu2_cop2_compare(DisasContext *ctx, int vd, int vs, int vt,
                                   int minor)
{
    /*
     * Compare operations: minor encodes condition and element size.
     *
     * minor = base + size_offset
     *   base: 40=ceq, 44=cne, 48=clt_s, 52=clt_u, 56=cle_s, 60=cle_u
     *   size_offset: 0=.b, 1=.h, 2=.w, 3=.d
     *
     * Result: all-ones lane if condition true, all-zeros if false.
     */
    int base = minor & ~3;
    int vece = minor & 3;
    TCGCond cond;

    switch (base) {
    case 40: cond = TCG_COND_EQ;  break;
    case 44: cond = TCG_COND_NE;  break;
    case 48: cond = TCG_COND_LT;  break;
    case 52: cond = TCG_COND_LTU; break;
    case 56: cond = TCG_COND_LE;  break;
    case 60: cond = TCG_COND_LEU; break;
    default:
        gen_reserved_instruction(ctx);
        return;
    }

    tcg_gen_gvec_cmp(cond, vece, vpr_full_offset(vd),
                      vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
}

static void gen_mxu2_cop2_minmax(DisasContext *ctx, int vd, int vs, int vt,
                                  int minor)
{
    /*
     * Min/Max operations: minor encodes operation and element size.
     *   0-3: max_s  (.b/.h/.w/.d)
     *   4-7: max_u
     *   8-11: maxa_s (max of absolute values, signed)
     *  12-15: min_s
     *  16-19: min_u
     *  20-23: mina_s (min of absolute values, signed)
     */
    int base = minor & ~3;
    int vece = minor & 3;

    switch (base) {
    case 0: /* max_s */
        tcg_gen_gvec_smax(vece, vpr_full_offset(vd),
                           vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 4: /* max_u */
        tcg_gen_gvec_umax(vece, vpr_full_offset(vd),
                           vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 12: /* min_s */
        tcg_gen_gvec_smin(vece, vpr_full_offset(vd),
                           vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 16: /* min_u */
        tcg_gen_gvec_umin(vece, vpr_full_offset(vd),
                           vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    default:
        gen_reserved_instruction(ctx);
        break;
    }
}

static void gen_mxu2_cop2_addsub(DisasContext *ctx, int vd, int vs, int vt,
                                  int minor)
{
    /*
     * Add/Sub operations: minor encodes operation and element size.
     *   0-3:   add_s  (.b/.h/.w/.d)  - wrapping signed add
     *   4-7:   sub_s  (.b/.h/.w/.d)  - wrapping signed sub
     *   8-11:  addss  - saturating signed add
     *  12-15:  subss  - saturating signed sub
     *  16-19:  adduu  - saturating unsigned add
     *  20-23:  subuu  - saturating unsigned sub
     *  40-43:  sll    - shift left logical (by register)
     *  48-51:  aves   - average signed
     *  52-55:  aveu   - average unsigned
     */
    int base = minor & ~3;
    int vece = minor & 3;

    switch (base) {
    case 0: /* add */
        tcg_gen_gvec_add(vece, vpr_full_offset(vd),
                          vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 4: /* sub */
        tcg_gen_gvec_sub(vece, vpr_full_offset(vd),
                          vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 8: /* addss - saturating signed add */
        tcg_gen_gvec_ssadd(vece, vpr_full_offset(vd),
                            vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 12: /* subss - saturating signed sub */
        tcg_gen_gvec_sssub(vece, vpr_full_offset(vd),
                            vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 16: /* adduu - saturating unsigned add */
        tcg_gen_gvec_usadd(vece, vpr_full_offset(vd),
                            vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 20: /* subuu - saturating unsigned sub */
        tcg_gen_gvec_ussub(vece, vpr_full_offset(vd),
                            vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    default:
        gen_reserved_instruction(ctx);
        break;
    }
}

static void gen_mxu2_cop2_logic(DisasContext *ctx, int vd, int vs, int vt,
                                 int minor)
{
    /*
     * 128-bit logic operations.
     *   56: andv    58: orv    59: xorv    57: norv
     */
    switch (minor) {
    case 56: /* andv */
        tcg_gen_gvec_and(MO_64, vpr_full_offset(vd),
                          vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 57: /* norv */
        tcg_gen_gvec_nor(MO_64, vpr_full_offset(vd),
                          vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 58: /* orv */
        tcg_gen_gvec_or(MO_64, vpr_full_offset(vd),
                         vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    case 59: /* xorv */
        tcg_gen_gvec_xor(MO_64, vpr_full_offset(vd),
                          vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    default:
        gen_reserved_instruction(ctx);
        break;
    }
}

static void gen_mxu2_cop2_mul(DisasContext *ctx, int vd, int vs, int vt,
                               int minor)
{
    /*
     * Multiply operations: minor encodes operation and element size.
     *   0-3: mul_s (.b/.h/.w/.d) - low half of signed multiply
     *   4-7: mul_u - low half of unsigned multiply
     */
    int base = minor & ~3;
    int vece = minor & 3;

    switch (base) {
    case 0: /* mul_s */
    case 4: /* mul_u */
        tcg_gen_gvec_mul(vece, vpr_full_offset(vd),
                          vpr_full_offset(vs), vpr_full_offset(vt), 16, 16);
        break;
    default:
        gen_reserved_instruction(ctx);
        break;
    }
}

static bool decode_mxu2_cop2_co(DisasContext *ctx)
{
    int major = (ctx->opcode >> 21) & 0xf;
    int vt = (ctx->opcode >> 16) & 0x1f;
    int vs = (ctx->opcode >> 11) & 0x1f;
    int vd = (ctx->opcode >> 6) & 0x1f;
    int minor = ctx->opcode & 0x3f;

    (void)0; /* 5-bit VPR fields, all 32 registers valid */

    switch (major) {
    case 0: /* Compare / MinMax / Shift-by-register */
        if (minor >= 40) {
            gen_mxu2_cop2_compare(ctx, vd, vs, vt, minor);
        } else {
            gen_mxu2_cop2_minmax(ctx, vd, vs, vt, minor);
        }
        return true;

    case 1: /* Add/Sub/Average/SLL */
        gen_mxu2_cop2_addsub(ctx, vd, vs, vt, minor);
        return true;

    case 2: /* Mul/Div/Mod/Madd/Msub/Dot */
        gen_mxu2_cop2_mul(ctx, vd, vs, vt, minor);
        return true;

    case 6: /* 128-bit logic */
        gen_mxu2_cop2_logic(ctx, vd, vs, vt, minor);
        return true;

    default:
        return false;
    }
}

/*
 * COP2 CF/CT (move from/to MXU2 control registers).
 *
 * CFCMXU: (18<<26)|(30<<21)|(1<<16)|(rd<<11)|(mcsrs<<6)|(30<<1)|1
 *   Reads MIR (mcsrs=0) or MCSR (mcsrs=31) into GPR[rd].
 *
 * CTCMXU: similar encoding for writes (not commonly used).
 */
static bool decode_mxu2_cop2_cf(DisasContext *ctx)
{
    int rd = (ctx->opcode >> 11) & 0x1f;
    int mcsrs = (ctx->opcode >> 6) & 0x1f;
    TCGv val = tcg_temp_new();

    switch (mcsrs) {
    case 0: /* MIR - Implementation Register (read-only) */
        tcg_gen_ld_tl(val, tcg_env,
                       offsetof(CPUMIPSState, active_tc.mxu2_mir));
        break;
    case 31: /* MCSR - Control/Status Register */
        tcg_gen_ld_tl(val, tcg_env,
                       offsetof(CPUMIPSState, active_tc.mxu2_mcsr));
        break;
    default:
        tcg_gen_movi_tl(val, 0);
        break;
    }

    gen_store_gpr(val, rd);
    return true;
}

/*
 * Top-level COP2 MXU2 decoder.
 *
 * Called from translate.c when opcode=0x12 and ASE_MXU2 is set.
 * Returns true if the instruction was handled, false otherwise.
 */
bool decode_ase_mxu2_cop2(DisasContext *ctx, uint32_t insn)
{
    int co = (insn >> 25) & 1;
    int fmt = (insn >> 21) & 0x1f;

    if (co) {
        return decode_mxu2_cop2_co(ctx);
    }

    /* CF format: fmt=30 (0x1e) for CFCMXU */
    if (fmt == 30) {
        return decode_mxu2_cop2_cf(ctx);
    }

    return false;
}

/*
 * Top-level SPECIAL2 MXU2 decoder.
 *
 * Called from translate.c before the MXU1 decoder when ASE_MXU2 is
 * set. Returns true if the instruction was handled, false to let
 * MXU1 (or legacy SPECIAL2) try.
 */
bool decode_ase_mxu2_special2(DisasContext *ctx, uint32_t insn)
{
    int funct = insn & 0x3f;

    switch (funct) {
    case 0x14: /* LU1Q - aligned 128-bit load */
        gen_mxu2_lu1q(ctx);
        return true;

    case 0x1c: /* SU1Q - aligned 128-bit store */
        gen_mxu2_su1q(ctx);
        return true;

    case 0x2c: /* LA1Q - aligned 128-bit load (alt encoding) */
        gen_mxu2_la1q(ctx);
        return true;

    case 0x3c: /* SA1Q - aligned 128-bit store (alt encoding) */
        gen_mxu2_sa1q(ctx);
        return true;

    case 0x28: /* BEQZ/BNEZ - VPR conditional branch (16b/8h/4w) */
    case 0x29: /* BEQZ1Q/BNEZ1Q - VPR conditional branch (1q) */
        gen_mxu2_vpr_branch(ctx);
        return true;

    case 0x31: /* INSFCPU - insert GPR into VPR element */
        gen_mxu2_insfcpu(ctx);
        return true;

    case 0x33: /* MTCPUSB - extract VPR element to GPR (sign-extend) */
        gen_mxu2_mtcpu(ctx, true);
        return true;

    case 0x34: /* MTCPUUB - extract VPR element to GPR (zero-extend) */
        gen_mxu2_mtcpu(ctx, false);
        return true;

    case 0x35: /* REPI - replicate immediate */
    case 0x38: /* SATS/SATU/SLLI */
    case 0x39: /* SRAI/SRARI/SRLI/SRLRI */
        return decode_mxu2_special2_imm(ctx, funct);

    default:
        return false;
    }
}
