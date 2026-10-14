/*
 * Ingenic XBurst2 MXU3 (SIMD512) translation routines
 *
 * MXU3 adds 32 512-bit vector registers (VPRs), four 512-bit sum
 * registers (VSRs) and the MIR/MCSR control registers to XBurst2. It is
 * coprocessor 2, encoded in the COP2 and SPECIAL2 major opcodes.
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * Reference: XBurst Instruction Set Architecture MXU3 Programming Manual,
 * revision 1.0
 */

#include "qemu/osdep.h"
#include "translate.h"
#include "tcg/tcg-op-gvec.h"

static inline int times_4(DisasContext *ctx, int x)
{
    return x * 4;
}

static inline int times_8(DisasContext *ctx, int x)
{
    return x * 8;
}

static inline int times_16(DisasContext *ctx, int x)
{
    return x * 16;
}

static inline int times_32(DisasContext *ctx, int x)
{
    return x * 32;
}

/* Include the auto-generated decoder. */
#include "decode-mxu3.c.inc"

static inline int vpr_offset(int r)
{
    return offsetof(CPUMIPSState, active_tc.mxu3_vpr[r]);
}

static inline int vsr_offset(int r)
{
    return offsetof(CPUMIPSState, active_tc.mxu3_vsr[r]);
}

/*
 * Registers are arrays of host-endian uint64_t, least significant first,
 * so elements narrower than 64 bits are reversed within each unit on a
 * big-endian host.
 */
static int vpr_element_offset(int r, int i, MemOp esz)
{
    int ofs = i << esz;

#if HOST_BIG_ENDIAN
    if (esz < MO_64) {
        ofs ^= 8 - (1 << esz);
    }
#endif
    return vpr_offset(r) + ofs;
}

#define TRANS_MXU3(NAME, FUNC, ...)                                     \
    static bool trans_##NAME(DisasContext *ctx, arg_##NAME *a)          \
    { return !check_mxu_cu2(ctx) || FUNC(ctx, a, __VA_ARGS__); }

/*
 * Element-wise operations on B, H and W elements. Format 3 is reserved,
 * or the single-precision form of the operation.
 */

static bool gen_vec_3(DisasContext *ctx, arg_r3 *a, GVecGen3Fn *fn)
{
    if (a->df > MO_32) {
        return false;
    }
    fn(a->df, vpr_offset(a->vd), vpr_offset(a->vs), vpr_offset(a->vp),
       MXU3_VLEN, MXU3_VLEN);
    return true;
}

static bool gen_vec_2(DisasContext *ctx, arg_r2 *a, GVecGen2Fn *fn)
{
    if (a->df > MO_32) {
        return false;
    }
    fn(a->df, vpr_offset(a->vd), vpr_offset(a->vs), MXU3_VLEN, MXU3_VLEN);
    return true;
}

static bool gen_ool_3(DisasContext *ctx, arg_r3 *a,
                      gen_helper_gvec_3 * const fns[3])
{
    if (a->df > MO_32) {
        return false;
    }
    tcg_gen_gvec_3_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                       vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN, 0,
                       fns[a->df]);
    return true;
}

#define MXU_FNS_BHW(NAME)                                               \
    static gen_helper_gvec_3 * const NAME##_fns[3] = {                  \
        gen_helper_mxu_##NAME##_b, gen_helper_mxu_##NAME##_h,           \
        gen_helper_mxu_##NAME##_w,                                      \
    }

/* Branch */

/*
 * BEQZ branches if any element is zero and BNEZ if none is, using the
 * SWAR zero-element test on each 64-bit unit; format 3 tests the whole
 * register instead. The offset counts instructions from the delay slot.
 */
static bool gen_branch(DisasContext *ctx, arg_b *a, bool nez)
{
    uint64_t ones = dup_const(a->df, 1);
    uint64_t msbs = dup_const(a->df, 1ull << ((8 << a->df) - 1));
    TCGv_i64 acc, unit, t;

    if (ctx->hflags & MIPS_HFLAG_BMASK) {
        gen_reserved_instruction(ctx);
        return true;
    }

    acc = tcg_temp_new_i64();
    unit = tcg_temp_new_i64();
    t = tcg_temp_new_i64();
    tcg_gen_movi_i64(acc, 0);
    for (int i = 0; i < MXU3_VLEN / 8; i++) {
        tcg_gen_ld_i64(unit, tcg_env, vpr_offset(a->vs) + i * 8);
        if (a->df != MO_64) {
            tcg_gen_subi_i64(t, unit, ones);
            tcg_gen_andc_i64(unit, t, unit);
        }
        tcg_gen_or_i64(acc, acc, unit);
    }
    if (a->df != MO_64) {
        tcg_gen_andi_i64(acc, acc, msbs);
    }
    tcg_gen_setcondi_i64((a->df == MO_64) == nez ? TCG_COND_NE : TCG_COND_EQ,
                         acc, acc, 0);
    tcg_gen_trunc_i64_tl(bcond, acc);

    ctx->btarget = ctx->base.pc_next + 4 + (a->offset << 2);
    ctx->hflags |= MIPS_HFLAG_BC | MIPS_HFLAG_BDS32;
    return true;
}

TRANS_MXU3(BEQZ, gen_branch, false)
TRANS_MXU3(BNEZ, gen_branch, true)

/* Compare */

static bool gen_cmp(DisasContext *ctx, arg_r3 *a, TCGCond cond)
{
    if (a->df > MO_32) {
        return false;
    }
    tcg_gen_gvec_cmp(cond, a->df, vpr_offset(a->vd), vpr_offset(a->vs),
                     vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN);
    return true;
}

static bool gen_cmpz(DisasContext *ctx, arg_r2 *a, TCGCond cond)
{
    if (a->df > MO_32) {
        return false;
    }
    tcg_gen_gvec_cmpi(cond, a->df, vpr_offset(a->vd), vpr_offset(a->vs), 0,
                      MXU3_VLEN, MXU3_VLEN);
    return true;
}

TRANS_MXU3(CEQ, gen_cmp, TCG_COND_EQ)
TRANS_MXU3(CLES, gen_cmp, TCG_COND_LE)
TRANS_MXU3(CLEU, gen_cmp, TCG_COND_LEU)
TRANS_MXU3(CLTS, gen_cmp, TCG_COND_LT)
TRANS_MXU3(CLTU, gen_cmp, TCG_COND_LTU)
TRANS_MXU3(CEQZ, gen_cmpz, TCG_COND_EQ)
TRANS_MXU3(CLEZ, gen_cmpz, TCG_COND_LE)
TRANS_MXU3(CLTZ, gen_cmpz, TCG_COND_LT)

/* Integer arithmetic */

MXU_FNS_BHW(maxa);
MXU_FNS_BHW(mina);

/* Formats 4 and 5 of the unsigned forms take 2-bit and 4-bit elements */
static bool gen_minmax_u(DisasContext *ctx, arg_r3 *a, GVecGen3Fn *fn,
                         gen_helper_gvec_3 *fn_2bi, gen_helper_gvec_3 *fn_4bi)
{
    if (a->df == 4 || a->df == 5) {
        tcg_gen_gvec_3_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                           vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN, 0,
                           a->df == 4 ? fn_2bi : fn_4bi);
        return true;
    }
    return gen_vec_3(ctx, a, fn);
}

TRANS_MXU3(ADD, gen_vec_3, tcg_gen_gvec_add)
TRANS_MXU3(SUB, gen_vec_3, tcg_gen_gvec_sub)
TRANS_MXU3(ABS, gen_vec_2, tcg_gen_gvec_abs)
TRANS_MXU3(MUL, gen_vec_3, tcg_gen_gvec_mul)
TRANS_MXU3(MAXA, gen_ool_3, maxa_fns)
TRANS_MXU3(MAXS, gen_vec_3, tcg_gen_gvec_smax)
TRANS_MXU3(MAXU, gen_minmax_u, tcg_gen_gvec_umax,
           gen_helper_mxu3_maxu_2bi, gen_helper_mxu3_maxu_4bi)
TRANS_MXU3(MINA, gen_ool_3, mina_fns)
TRANS_MXU3(MINS, gen_vec_3, tcg_gen_gvec_smin)
TRANS_MXU3(MINU, gen_minmax_u, tcg_gen_gvec_umin,
           gen_helper_mxu3_minu_2bi, gen_helper_mxu3_minu_4bi)

/* Bitwise */

static bool gen_logic(DisasContext *ctx, arg_v3 *a, GVecGen3Fn *fn)
{
    fn(MO_64, vpr_offset(a->vd), vpr_offset(a->vs), vpr_offset(a->vp),
       MXU3_VLEN, MXU3_VLEN);
    return true;
}

/* The N forms invert vs: vd = ~vs op vp */
static bool gen_logic_n(DisasContext *ctx, arg_v3 *a, GVecGen3Fn *fn)
{
    fn(MO_64, vpr_offset(a->vd), vpr_offset(a->vp), vpr_offset(a->vs),
       MXU3_VLEN, MXU3_VLEN);
    return true;
}

static bool gen_logic_imm(DisasContext *ctx, arg_vi *a, GVecGen2iFn *fn)
{
    fn(MO_8, vpr_offset(a->vd), vpr_offset(a->vs), a->imm,
       MXU3_VLEN, MXU3_VLEN);
    return true;
}

TRANS_MXU3(ANDV, gen_logic, tcg_gen_gvec_and)
TRANS_MXU3(ANDNV, gen_logic_n, tcg_gen_gvec_andc)
TRANS_MXU3(ORV, gen_logic, tcg_gen_gvec_or)
TRANS_MXU3(ORNV, gen_logic_n, tcg_gen_gvec_orc)
TRANS_MXU3(XORV, gen_logic, tcg_gen_gvec_xor)
TRANS_MXU3(XORNV, gen_logic_n, tcg_gen_gvec_eqv)
TRANS_MXU3(ANDIB, gen_logic_imm, tcg_gen_gvec_andi)
TRANS_MXU3(ORIB, gen_logic_imm, tcg_gen_gvec_ori)
TRANS_MXU3(XORIB, gen_logic_imm, tcg_gen_gvec_xori)

/* vd = vd ? vp : vs, bitwise */
static bool trans_BSELV(DisasContext *ctx, arg_BSELV *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_bitsel(MO_64, vpr_offset(a->vd), vpr_offset(a->vd),
                            vpr_offset(a->vp), vpr_offset(a->vs),
                            MXU3_VLEN, MXU3_VLEN);
    }
    return true;
}

/* Shift: the count is the low log2(esize) bits of vp or the immediate */

MXU_FNS_BHW(srar);
MXU_FNS_BHW(srlr);

static bool gen_shift_imm(DisasContext *ctx, arg_ri *a, GVecGen2iFn *fn)
{
    if (a->df > MO_32) {
        return false;
    }
    fn(a->df, vpr_offset(a->vd), vpr_offset(a->vs),
       a->imm & ((8 << a->df) - 1), MXU3_VLEN, MXU3_VLEN);
    return true;
}

static bool gen_shift_round_imm(DisasContext *ctx, arg_ri *a, bool arith)
{
    static gen_helper_gvec_2 * const srari_fns[3] = {
        gen_helper_mxu_srari_b, gen_helper_mxu_srari_h,
        gen_helper_mxu_srari_w,
    };
    static gen_helper_gvec_2 * const srlri_fns[3] = {
        gen_helper_mxu_srlri_b, gen_helper_mxu_srlri_h,
        gen_helper_mxu_srlri_w,
    };

    if (a->df > MO_32) {
        return false;
    }
    tcg_gen_gvec_2_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                       MXU3_VLEN, MXU3_VLEN, a->imm,
                       (arith ? srari_fns : srlri_fns)[a->df]);
    return true;
}

TRANS_MXU3(SLL, gen_vec_3, tcg_gen_gvec_shlv)
TRANS_MXU3(SRA, gen_vec_3, tcg_gen_gvec_sarv)
TRANS_MXU3(SRAR, gen_ool_3, srar_fns)
TRANS_MXU3(SRL, gen_vec_3, tcg_gen_gvec_shrv)
TRANS_MXU3(SRLR, gen_ool_3, srlr_fns)
TRANS_MXU3(SLLI, gen_shift_imm, tcg_gen_gvec_shli)
TRANS_MXU3(SRAI, gen_shift_imm, tcg_gen_gvec_sari)
TRANS_MXU3(SRARI, gen_shift_round_imm, true)
TRANS_MXU3(SRLI, gen_shift_imm, tcg_gen_gvec_shri)
TRANS_MXU3(SRLRI, gen_shift_round_imm, false)

/* Load/store */

/*
 * Element n of size esz of vr is transferred at an address with no
 * alignment restriction. A 256-bit element is two 128-bit accesses; a
 * fault on the second one restarts the whole instruction, which is safe
 * because it leaves the register (or redoes the store) unchanged.
 */
static void gen_ldst_element(DisasContext *ctx, int vr, int n, MemOp esz,
                             TCGv addr, bool store)
{
    MemOp mop = MO_TE | MO_UNALN;
    int ofs = vpr_element_offset(vr, n, esz);

    if (esz <= MO_64) {
        TCGv_i64 val = tcg_temp_new_i64();

        if (store) {
            if (esz == MO_32) {
                tcg_gen_ld32u_i64(val, tcg_env, ofs);
            } else {
                tcg_gen_ld_i64(val, tcg_env, ofs);
            }
            tcg_gen_qemu_st_i64(val, addr, ctx->mem_idx, mop | esz);
        } else {
            tcg_gen_qemu_ld_i64(val, addr, ctx->mem_idx, mop | esz);
            if (esz == MO_32) {
                tcg_gen_st32_i64(val, tcg_env, ofs);
            } else {
                tcg_gen_st_i64(val, tcg_env, ofs);
            }
        }
    } else {
        int parts = 1 << (esz - MO_128);
        TCGv_i128 val[2];
        TCGv_i64 lo = tcg_temp_new_i64();
        TCGv_i64 hi = tcg_temp_new_i64();
        TCGv a = tcg_temp_new();

        mop |= MO_128 | MO_ATOM_IFALIGN_PAIR;
        for (int i = 0; i < parts; i++) {
            val[i] = tcg_temp_new_i128();
            tcg_gen_addi_tl(a, addr, i * 16);
            if (store) {
                tcg_gen_ld_i64(lo, tcg_env, ofs + i * 16);
                tcg_gen_ld_i64(hi, tcg_env, ofs + i * 16 + 8);
                tcg_gen_concat_i64_i128(val[i], lo, hi);
                tcg_gen_qemu_st_i128(val[i], a, ctx->mem_idx, mop);
            } else {
                tcg_gen_qemu_ld_i128(val[i], a, ctx->mem_idx, mop);
            }
        }
        for (int i = 0; !store && i < parts; i++) {
            tcg_gen_extr_i128_i64(lo, hi, val[i]);
            tcg_gen_st_i64(lo, tcg_env, ofs + i * 16);
            tcg_gen_st_i64(hi, tcg_env, ofs + i * 16 + 8);
        }
    }
}

/* The element index wraps at the number of elements in a register */
static bool gen_ldst(DisasContext *ctx, arg_ldst *a, MemOp esz, bool store)
{
    int n = a->n & ((MXU3_VLEN >> esz) - 1);
    TCGv addr = tcg_temp_new();

    gen_base_offset_addr(ctx, addr, a->base, a->offset);
    gen_ldst_element(ctx, a->vr, n, esz, addr, store);
    return true;
}

/* The update forms address base, then advance it by the element size */
static bool gen_ldst_update(DisasContext *ctx, arg_ldst *a, MemOp esz,
                            bool store)
{
    TCGv base = tcg_temp_new();

    gen_ldst(ctx, a, esz, store);
    gen_load_gpr(base, a->base);
    tcg_gen_addi_tl(base, base, 1 << esz);
    gen_store_gpr(base, a->base);
    return true;
}

TRANS_MXU3(LAW, gen_ldst, MO_32, false)
TRANS_MXU3(LAD, gen_ldst, MO_64, false)
TRANS_MXU3(LAQ, gen_ldst, MO_128, false)
TRANS_MXU3(LAO, gen_ldst, MO_256, false)
TRANS_MXU3(SAW, gen_ldst, MO_32, true)
TRANS_MXU3(SAD, gen_ldst, MO_64, true)
TRANS_MXU3(SAQ, gen_ldst, MO_128, true)
TRANS_MXU3(SAO, gen_ldst, MO_256, true)
TRANS_MXU3(LUW, gen_ldst_update, MO_32, false)
TRANS_MXU3(LUD, gen_ldst_update, MO_64, false)
TRANS_MXU3(LUQ, gen_ldst_update, MO_128, false)
TRANS_MXU3(LUO, gen_ldst_update, MO_256, false)
TRANS_MXU3(SUW, gen_ldst_update, MO_32, true)
TRANS_MXU3(SUD, gen_ldst_update, MO_64, true)
TRANS_MXU3(SUQ, gen_ldst_update, MO_128, true)
TRANS_MXU3(SUO, gen_ldst_update, MO_256, true)

/* Sum registers */

static bool trans_SUMZ(DisasContext *ctx, arg_SUMZ *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_dup_imm(MO_64, vsr_offset(a->sr), MXU3_VLEN, MXU3_VLEN,
                             0);
    }
    return true;
}

static bool trans_MFSUM(DisasContext *ctx, arg_MFSUM *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_mov(MO_64, vpr_offset(a->vr), vsr_offset(a->sr),
                         MXU3_VLEN, MXU3_VLEN);
    }
    return true;
}

static bool trans_MFSUMZ(DisasContext *ctx, arg_MFSUMZ *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_mov(MO_64, vpr_offset(a->vr), vsr_offset(a->sr),
                         MXU3_VLEN, MXU3_VLEN);
        tcg_gen_gvec_dup_imm(MO_64, vsr_offset(a->sr), MXU3_VLEN, MXU3_VLEN,
                             0);
    }
    return true;
}

static bool trans_MTSUM(DisasContext *ctx, arg_MTSUM *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_mov(MO_64, vsr_offset(a->sr), vpr_offset(a->vr),
                         MXU3_VLEN, MXU3_VLEN);
    }
    return true;
}

/* vd = sr, sr = vs; vd may be vs */
static bool trans_MXSUM(DisasContext *ctx, arg_MXSUM *a)
{
    TCGv_i64 old[MXU3_VLEN / 8];

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    for (int i = 0; i < ARRAY_SIZE(old); i++) {
        old[i] = tcg_temp_new_i64();
        tcg_gen_ld_i64(old[i], tcg_env, vsr_offset(a->sr) + i * 8);
    }
    tcg_gen_gvec_mov(MO_64, vsr_offset(a->sr), vpr_offset(a->vs),
                     MXU3_VLEN, MXU3_VLEN);
    for (int i = 0; i < ARRAY_SIZE(old); i++) {
        tcg_gen_st_i64(old[i], tcg_env, vpr_offset(a->vd) + i * 8);
    }
    return true;
}

/* Control registers: MIR is 0 and read-only, MCSR is 31 */

static bool trans_CFCMXU(DisasContext *ctx, arg_CFCMXU *a)
{
    TCGv val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    val = tcg_temp_new();
    switch (a->cr) {
    case 0:
        tcg_gen_ld32u_tl(val, tcg_env,
                         offsetof(CPUMIPSState, active_tc.mxu_mir));
        break;
    case 31:
        tcg_gen_ld32u_tl(val, tcg_env,
                         offsetof(CPUMIPSState, active_tc.mxu_mcsr));
        break;
    default:
        tcg_gen_movi_tl(val, 0);
        break;
    }
    gen_store_gpr(val, a->rt);
    return true;
}

static bool trans_CTCMXU(DisasContext *ctx, arg_CTCMXU *a)
{
    TCGv val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    if (a->cr == 31) {
        val = tcg_temp_new();
        gen_load_gpr(val, a->rt);
        gen_helper_mxu_ctcmxu(tcg_env, val);
    }
    return true;
}

/*
 * MXU3 owns COP2: an encoding it does not define still checks Status.CU2
 * before it is reserved. In SPECIAL2 it shares the space with the base
 * ISA, which decodes whatever it does not claim.
 */
bool decode_ase_mxu3_cop2(DisasContext *ctx, uint32_t insn)
{
    if (!decode_mxu3(ctx, insn) && check_mxu_cu2(ctx)) {
        gen_reserved_instruction(ctx);
    }
    return true;
}

bool decode_ase_mxu3_special2(DisasContext *ctx, uint32_t insn)
{
    return decode_mxu3(ctx, insn);
}
