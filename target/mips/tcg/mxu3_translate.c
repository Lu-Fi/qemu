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
#include "mxu_internal.h"

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

/* Double-width results from B or H elements, data selects which half */
static bool gen_widen(DisasContext *ctx, arg_w3 *a,
                      gen_helper_gvec_3 * const fns[2])
{
    tcg_gen_gvec_3_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                       vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN, a->p,
                       fns[a->df]);
    return true;
}

#define MXU3_FNS_BH(NAME)                                               \
    static gen_helper_gvec_3 * const NAME##_fns[2] = {                  \
        gen_helper_mxu3_##NAME##_b, gen_helper_mxu3_##NAME##_h,         \
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

/*
 * The W forms widen the low or high half of vp and combine it with the
 * double-width elements of vs; UMUL and SMUL multiply the even or odd
 * elements of both.
 */
MXU3_FNS_BH(waddu);
MXU3_FNS_BH(wadds);
MXU3_FNS_BH(wsubu);
MXU3_FNS_BH(wsubs);
MXU3_FNS_BH(wumul);
MXU3_FNS_BH(wsmul);
MXU3_FNS_BH(umul);
MXU3_FNS_BH(smul);

TRANS_MXU3(WADDU, gen_widen, waddu_fns)
TRANS_MXU3(WADDS, gen_widen, wadds_fns)
TRANS_MXU3(WSUBU, gen_widen, wsubu_fns)
TRANS_MXU3(WSUBS, gen_widen, wsubs_fns)
TRANS_MXU3(WUMUL, gen_widen, wumul_fns)
TRANS_MXU3(WSMUL, gen_widen, wsmul_fns)
TRANS_MXU3(UMUL, gen_widen, umul_fns)
TRANS_MXU3(SMUL, gen_widen, smul_fns)

/* Multiply-accumulate into the words of a sum register */

static void gen_mla_i32(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_mul_i32(a, a, b);
    tcg_gen_add_i32(d, d, a);
}

static void gen_mls_i32(TCGv_i32 d, TCGv_i32 a, TCGv_i32 b)
{
    tcg_gen_mul_i32(a, a, b);
    tcg_gen_sub_i32(d, d, a);
}

static void gen_mla_vec(unsigned vece, TCGv_vec d, TCGv_vec a, TCGv_vec b)
{
    tcg_gen_mul_vec(vece, a, a, b);
    tcg_gen_add_vec(vece, d, d, a);
}

static void gen_mls_vec(unsigned vece, TCGv_vec d, TCGv_vec a, TCGv_vec b)
{
    tcg_gen_mul_vec(vece, a, a, b);
    tcg_gen_sub_vec(vece, d, d, a);
}

static bool gen_mlaw(DisasContext *ctx, arg_mac *a, bool sub)
{
    static const TCGOpcode mla_list[] = {
        INDEX_op_mul_vec, INDEX_op_add_vec, 0
    };
    static const TCGOpcode mls_list[] = {
        INDEX_op_mul_vec, INDEX_op_sub_vec, 0
    };
    static const GVecGen3 ops[2] = {
        { .fni4 = gen_mla_i32, .fniv = gen_mla_vec, .opt_opc = mla_list,
          .load_dest = true, .vece = MO_32 },
        { .fni4 = gen_mls_i32, .fniv = gen_mls_vec, .opt_opc = mls_list,
          .load_dest = true, .vece = MO_32 },
    };

    tcg_gen_gvec_3(vsr_offset(a->sr), vpr_offset(a->vs), vpr_offset(a->vp),
                   MXU3_VLEN, MXU3_VLEN, &ops[sub]);
    return true;
}

static bool gen_mac(DisasContext *ctx, arg_mac *a, gen_helper_gvec_3 *fn)
{
    tcg_gen_gvec_3_ool(vsr_offset(a->sr), vpr_offset(a->vs),
                       vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN, a->p, fn);
    return true;
}

TRANS_MXU3(MLAW, gen_mlaw, false)
TRANS_MXU3(MLSW, gen_mlaw, true)
TRANS_MXU3(SMLAH, gen_mac, gen_helper_mxu3_smlah)
TRANS_MXU3(SMLSH, gen_mac, gen_helper_mxu3_smlsh)
TRANS_MXU3(WSMLAH, gen_mac, gen_helper_mxu3_wsmlah)
TRANS_MXU3(WSMLSH, gen_mac, gen_helper_mxu3_wsmlsh)

/*
 * Segmented sums: each 2^le-word segment of vs, or of its products with
 * vp, is summed into word m of the segment in the sum register. vp is
 * either segment n replicated or, for n < 0, the matching segment; vp < 0
 * means a plain sum. A SIMD512 register has up to 16-word segments.
 */
static bool gen_seg_sum(DisasContext *ctx, int sr, int vs, int vp, int esz,
                        bool ssign, bool psign, int le, int m, int n)
{
    uint32_t data = 0;

    if (le > 4 || m >= 1 << le) {
        return false;
    }
    if (!check_mxu_cu2(ctx)) {
        return true;
    }

    data = FIELD_DP32(data, MXU3_SUM, ESZ, esz);
    data = FIELD_DP32(data, MXU3_SUM, SSIGN, ssign);
    data = FIELD_DP32(data, MXU3_SUM, PSIGN, psign);
    data = FIELD_DP32(data, MXU3_SUM, LE, le);
    data = FIELD_DP32(data, MXU3_SUM, M, m);
    if (vp < 0) {
        tcg_gen_gvec_2_ool(vsr_offset(sr), vpr_offset(vs),
                           MXU3_VLEN, MXU3_VLEN, data, gen_helper_mxu3_srsum);
        return true;
    }
    if (n >= 0) {
        /* n wraps at the number of segments */
        data = FIELD_DP32(data, MXU3_SUM, REPL, 1);
        data = FIELD_DP32(data, MXU3_SUM, N,
                          n & ((MXU3_VLEN / 4 >> le) - 1));
    }
    tcg_gen_gvec_3_ool(vsr_offset(sr), vpr_offset(vs), vpr_offset(vp),
                       MXU3_VLEN, MXU3_VLEN, data, gen_helper_mxu3_mac);
    return true;
}

static bool trans_SRSUM(DisasContext *ctx, arg_SRSUM *a)
{
    /* 2BI, 4BI, UB, UH, SB, SH, W */
    static const int8_t esz[8] = { 1, 2, 3, 4, 3, 4, 5, -1 };

    if (esz[a->t] < 0) {
        return false;
    }
    return gen_seg_sum(ctx, a->sr, a->vs, -1, esz[a->t], a->t >= 4, false,
                       a->le, a->m, -1);
}

/* m is above n in mn, sized by the segment length */
static bool trans_SRMAC(DisasContext *ctx, arg_SRMAC *a)
{
    /* 2BI, 4BI, UUB, -, SSB, SSH, SUB; 7 is SMAC */
    static const int8_t esz[8] = { 1, 2, 3, -1, 3, 4, 3, -1 };

    if (esz[a->t] < 0 || a->le > 4) {
        return false;
    }
    return gen_seg_sum(ctx, a->sr, a->vs, a->vp, esz[a->t], a->t >= 4,
                       a->t == 4 || a->t == 5, a->le, a->mn >> (5 - a->le),
                       extract32(a->mn, 0, 5 - a->le));
}

/* mle is m above a 1 at bit 5 - le; t is UUB, SUB, SSB or SSH */
static bool trans_SMAC(DisasContext *ctx, arg_SMAC *a)
{
    int le = 5 - ctz32(a->mle);

    if (le < 0) {
        return false;
    }
    return gen_seg_sum(ctx, a->sr, a->vs, a->vp, a->t == 3 ? 4 : 3, a->t > 0,
                       a->t > 1, le, a->mle >> (6 - le), -1);
}

/* Word 0 of the sum register counts the leading elements with bit 0 set */
static bool trans_TOC(DisasContext *ctx, arg_TOC *a)
{
    if (a->df > MO_32) {
        return false;
    }
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_2_ool(vsr_offset(a->sr), vpr_offset(a->vs),
                           MXU3_VLEN, MXU3_VLEN, a->df + 3,
                           gen_helper_mxu3_toc);
    }
    return true;
}

/* Saturation to 2BI, 4BI, B or H ranges, keeping the element width */

static bool gen_sat(DisasContext *ctx, int vd, int vs, MemOp esz, int bits,
                    bool ssign, bool dsign)
{
    static gen_helper_gvec_2 * const fns[3] = {
        gen_helper_mxu3_sat_b, gen_helper_mxu3_sat_h, gen_helper_mxu3_sat_w,
    };
    uint32_t data = 0;

    if (esz > MO_32 || bits >= 8 << esz) {
        return false;
    }
    if (!check_mxu_cu2(ctx)) {
        return true;
    }

    data = FIELD_DP32(data, MXU3_SAT, BITS, bits);
    data = FIELD_DP32(data, MXU3_SAT, DSIGN, dsign);
    data = FIELD_DP32(data, MXU3_SAT, SSIGN, ssign);
    tcg_gen_gvec_2_ool(vpr_offset(vd), vpr_offset(vs), MXU3_VLEN, MXU3_VLEN,
                       data, fns[esz]);
    return true;
}

static bool trans_SATSS(DisasContext *ctx, arg_SATSS *a)
{
    return gen_sat(ctx, a->vd, a->vs, a->df, 8 << a->tgt, true, true);
}

static bool trans_SATSU(DisasContext *ctx, arg_SATSU *a)
{
    return gen_sat(ctx, a->vd, a->vs, a->df, 2 << a->tgt, true, false);
}

static bool trans_SATUU(DisasContext *ctx, arg_SATUU *a)
{
    /* B to 2BI and 4BI, H to 2BI, 4BI and B, W to 4BI, B and H */
    static const uint8_t esz[8] = {
        MO_8, MO_8, MO_16, MO_16, MO_16, MO_32, MO_32, MO_32,
    };
    static const uint8_t bits[8] = { 2, 4, 2, 4, 8, 4, 8, 16 };

    return gen_sat(ctx, a->vd, a->vs, esz[a->f], bits[a->f], false, false);
}

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

/* Shuffle and permute */

static bool gen_shf(DisasContext *ctx, int vd, int vp, uint32_t data,
                    gen_helper_gvec_2 *fn)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_2_ool(vpr_offset(vd), vpr_offset(vp), MXU3_VLEN,
                           MXU3_VLEN, data, fn);
    }
    return true;
}

static bool gen_shf3(DisasContext *ctx, int vd, int vs, int vp,
                     uint32_t data, gen_helper_gvec_3 *fn)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_3_ool(vpr_offset(vd), vpr_offset(vs), vpr_offset(vp),
                           MXU3_VLEN, MXU3_VLEN, data, fn);
    }
    return true;
}

/* log2 of the element size in bits for B, H, 1BI, 2BI and 4BI */
static int shf_esz(int fmt)
{
    static const int8_t esz[8] = { 3, 4, -1, -1, 0, 1, 2, -1 };

    return esz[fmt];
}

static bool trans_GT(DisasContext *ctx, arg_GT *a)
{
    int esz = shf_esz(a->fmt);

    if (esz < 0) {
        return false;
    }
    return gen_shf(ctx, a->vd, a->vp, FIELD_DP32(0, MXU3_SHF, ESZ, esz),
                   gen_helper_mxu3_gt);
}

/*
 * fp is W, D, Q or O as 0, 10, 110 or 1110, then pos; nm has m below a 1
 * at bit log2(n), except GT2W, which is 0000m. Like the T40 and T41, a
 * group must be at most half the register and pos must be in it.
 */
static bool trans_GTN(DisasContext *ctx, arg_GTN *a)
{
    int fmt = clo32(a->fp << 27);
    int esz = 5 + fmt;
    int lg = 31 - clz32(a->nm);
    int pos;
    uint32_t data = 0;

    if (fmt == 0 && a->nm < 4) {
        lg = a->nm < 2 ? 1 : -1;
    }
    if (fmt > 3 || lg < 1 || esz + lg > 8) {
        return false;
    }
    pos = extract32(a->fp, 0, 4 - fmt);
    if (pos >= 1 << lg) {
        return false;
    }
    data = FIELD_DP32(data, MXU3_SHF, ESZ, esz);
    data = FIELD_DP32(data, MXU3_SHF, LG, lg);
    data = FIELD_DP32(data, MXU3_SHF, POS, pos);
    data = FIELD_DP32(data, MXU3_SHF, M, extract32(a->nm, 0, lg));
    return gen_shf(ctx, a->vd, a->vp, data, gen_helper_mxu3_gtn);
}

/* W, D, Q and O; X needs SIMD1024 */
static bool trans_EXTUW(DisasContext *ctx, arg_EXTUW *a)
{
    uint32_t data = 0;

    if (a->fmt > 3) {
        return false;
    }
    data = FIELD_DP32(data, MXU3_SHF, ESZ, 5 + a->fmt);
    data = FIELD_DP32(data, MXU3_SHF, ZERO, a->w);
    data = FIELD_DP32(data, MXU3_SHF, HI, a->hi);
    data = FIELD_DP32(data, MXU3_SHF, LOW, a->zp);
    return gen_shf(ctx, a->vd, a->vp, data, gen_helper_mxu3_extuw);
}

static bool trans_EXT(DisasContext *ctx, arg_EXT *a)
{
    int esz = shf_esz(a->fmt);
    uint32_t data = 0;

    if (esz < 0) {
        return false;
    }
    data = FIELD_DP32(data, MXU3_SHF, ESZ, esz);
    data = FIELD_DP32(data, MXU3_SHF, SIGN, a->s);
    data = FIELD_DP32(data, MXU3_SHF, HI, a->hi);
    return gen_shf(ctx, a->vd, a->vp, data, gen_helper_mxu3_ext);
}

static bool trans_EXTU3BW(DisasContext *ctx, arg_EXTU3BW *a)
{
    return gen_shf(ctx, a->vd, a->vp, 0, gen_helper_mxu3_extu3bw);
}

/*
 * fn is B, H, W, D, Q or O as 0 to 5 ones and a zero, then n, which wraps
 * at the number of elements; X needs SIMD1024.
 */
static bool trans_REPI(DisasContext *ctx, arg_REPI *a)
{
    int esz = clo32(a->fn << 24);
    int n, src;

    if (esz > 5) {
        return false;
    }
    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    n = a->fn & ((MXU3_VLEN >> esz) - 1);
    src = esz <= MO_64 ? vpr_element_offset(a->vp, n, esz)
                       : vpr_offset(a->vp) + (n << esz);
    tcg_gen_gvec_dup_mem(esz, vpr_offset(a->vd), src, MXU3_VLEN, MXU3_VLEN);
    return true;
}

/* The element indices wrap at the number of elements */
static bool trans_MOVW(DisasContext *ctx, arg_MOVW *a)
{
    int mask = MXU3_VLEN / 4 - 1;
    TCGv_i32 t;

    if (check_mxu_cu2(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_ld_i32(t, tcg_env,
                       vpr_element_offset(a->vp, a->n & mask, MO_32));
        tcg_gen_st_i32(t, tcg_env,
                       vpr_element_offset(a->vd, a->m & mask, MO_32));
    }
    return true;
}

/* D, Q or O, whole 64-bit units; X needs SIMD1024 */
static bool trans_MOV(DisasContext *ctx, arg_MOV *a)
{
    int esz = MO_64 + a->fmt;
    int mask = (MXU3_VLEN >> esz) - 1;
    TCGv_i64 t[4];

    if (a->fmt > 2) {
        return false;
    }
    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    for (int i = 0; i < 1 << a->fmt; i++) {
        t[i] = tcg_temp_new_i64();
        tcg_gen_ld_i64(t[i], tcg_env,
                       vpr_offset(a->vp) + ((a->n & mask) << esz) + i * 8);
    }
    for (int i = 0; i < 1 << a->fmt; i++) {
        tcg_gen_st_i64(t[i], tcg_env,
                       vpr_offset(a->vd) + ((a->m & mask) << esz) + i * 8);
    }
    return true;
}

/* A group is at most half the register */
static bool trans_SHUF(DisasContext *ctx, arg_SHUF *a)
{
    int esz = 5 + a->fmt, lg = 1 + a->grp;
    uint32_t data = 0;

    if (esz + lg > 8) {
        return false;
    }
    data = FIELD_DP32(data, MXU3_SHF, ESZ, esz);
    data = FIELD_DP32(data, MXU3_SHF, LG, lg);
    return gen_shf(ctx, a->vd, a->vp, data, gen_helper_mxu3_shuf);
}

/* 2BI to O; X needs SIMD1024 */
static bool trans_ILV(DisasContext *ctx, arg_ILV *a)
{
    uint32_t data = 0;

    if (a->fmt > 7) {
        return false;
    }
    data = FIELD_DP32(data, MXU3_SHF, ESZ, 1 + a->fmt);
    data = FIELD_DP32(data, MXU3_SHF, ODD, a->odd);
    return gen_shf3(ctx, a->vd, a->vs, a->vp, data, gen_helper_mxu3_ilv);
}

/* SIMD512 takes the low 6 bits of the byte count */
static bool trans_BSHI(DisasContext *ctx, arg_BSHI *a)
{
    return gen_shf(ctx, a->vd, a->vp, (a->imm & 63) | a->right << 6,
                   gen_helper_mxu3_bshi);
}

static bool trans_BSH(DisasContext *ctx, arg_BSH *a)
{
    return gen_shf3(ctx, a->vd, a->vs, a->vp, a->right, gen_helper_mxu3_bsh);
}

/*
 * Generic shuffles. MXU3.1 replaces GSHUFWB, GSHUFB, GSHUFWH and GSHUFH
 * with GSHUFWB1, GSHUFWB2 and GSHUFVB, and decodes the zero and keep word
 * selectors of GSHUFW and GSHUFWB1 from bits 7:6 only (data bit 2).
 */
static bool is_mxu31(DisasContext *ctx)
{
    return ctx->insn_flags & ASE_MXU3_1;
}

static bool trans_GSHUFW(DisasContext *ctx, arg_GSHUFW *a)
{
    return gen_shf3(ctx, a->vd, a->vs, a->vp, a->imm | is_mxu31(ctx) << 2,
                    gen_helper_mxu3_gshufw);
}

static bool trans_GSHUFWB(DisasContext *ctx, arg_GSHUFWB *a)
{
    return !is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, 0, gen_helper_mxu3_gshufwb);
}

static bool trans_GSHUFWB1(DisasContext *ctx, arg_GSHUFWB1 *a)
{
    return is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, a->p | 4,
                    gen_helper_mxu3_gshufwb);
}

static bool trans_GSHUFWB2(DisasContext *ctx, arg_GSHUFWB2 *a)
{
    return is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, a->p, gen_helper_mxu3_gshufwb2);
}

static bool trans_GSHUFB(DisasContext *ctx, arg_GSHUFB *a)
{
    return !is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, 0, gen_helper_mxu3_gshufb);
}

static bool trans_GSHUFWH(DisasContext *ctx, arg_GSHUFWH *a)
{
    return !is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, 0, gen_helper_mxu3_gshufwh);
}

static bool trans_GSHUFH(DisasContext *ctx, arg_GSHUFH *a)
{
    return !is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, 0, gen_helper_mxu3_gshufh);
}

static bool trans_GSHUFVB(DisasContext *ctx, arg_GSHUFVB *a)
{
    return is_mxu31(ctx) &&
           gen_shf3(ctx, a->vd, a->vs, a->vp, 0, gen_helper_mxu3_gshufvb);
}

/* Immediate loads and the VWR: LIWH sets the odd halfwords only */

static bool gen_li(DisasContext *ctx, arg_li *a, MemOp esz)
{
    tcg_gen_gvec_dup_imm(esz, vpr_offset(a->vd), MXU3_VLEN, MXU3_VLEN,
                         a->imm);
    return true;
}

TRANS_MXU3(LIH, gen_li, MO_16)
TRANS_MXU3(LIW, gen_li, MO_32)

static bool trans_LIWH(DisasContext *ctx, arg_LIWH *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_andi(MO_32, vpr_offset(a->vd), vpr_offset(a->vd),
                          0xffff, MXU3_VLEN, MXU3_VLEN);
        tcg_gen_gvec_ori(MO_32, vpr_offset(a->vd), vpr_offset(a->vd),
                         a->imm << 16, MXU3_VLEN, MXU3_VLEN);
    }
    return true;
}

/* VWR i is word i of vr31; the index wraps at the number of words */
static int vwr_offset(int i)
{
    return vpr_element_offset(31, i & (MXU3_VLEN / 4 - 1), MO_32);
}

static bool trans_LIWR(DisasContext *ctx, arg_LIWR *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_st_i32(tcg_constant_i32(a->imm), tcg_env,
                       vwr_offset(a->vwd));
    }
    return true;
}

/* vwd = vwp + imm, or vwp + vwd for ADDRW */
static bool gen_addw(DisasContext *ctx, int vwd, int vwp, int imm, bool reg)
{
    TCGv_i32 t = tcg_temp_new_i32();

    tcg_gen_ld_i32(t, tcg_env, vwr_offset(vwp));
    if (reg) {
        TCGv_i32 u = tcg_temp_new_i32();

        tcg_gen_ld_i32(u, tcg_env, vwr_offset(vwd));
        tcg_gen_add_i32(t, t, u);
    } else {
        tcg_gen_addi_i32(t, t, imm);
    }
    tcg_gen_st_i32(t, tcg_env, vwr_offset(vwd));
    return true;
}

static bool trans_ADDIW(DisasContext *ctx, arg_ADDIW *a)
{
    return !check_mxu_cu2(ctx) || gen_addw(ctx, a->vwd, a->vwp, a->imm, false);
}

static bool trans_ADDRW(DisasContext *ctx, arg_ADDRW *a)
{
    return !check_mxu_cu2(ctx) || gen_addw(ctx, a->vwd, a->vwp, 0, true);
}

static bool trans_MTCPUW(DisasContext *ctx, arg_MTCPUW *a)
{
    TCGv t;

    if (check_mxu_cu2(ctx)) {
        t = tcg_temp_new();
        tcg_gen_ld32s_tl(t, tcg_env, vwr_offset(a->vwp));
        gen_store_gpr(t, a->rd);
    }
    return true;
}

static bool trans_MFCPUW(DisasContext *ctx, arg_MFCPUW *a)
{
    TCGv t;

    if (check_mxu_cu2(ctx)) {
        t = tcg_temp_new();
        gen_load_gpr(t, a->rs);
        tcg_gen_st32_tl(t, tcg_env, vwr_offset(a->vwd));
    }
    return true;
}

/*
 * The FPR moves take the low word of the 64-bit register, as with
 * Status.FR set. Like a T41 they check only Status.CU2, so a task that
 * does not own the FPU reaches registers it then loses.
 */
static bool trans_MTFPUW(DisasContext *ctx, arg_MTFPUW *a)
{
    TCGv_i32 t;

    if (check_mxu_cu2(ctx)) {
        t = tcg_temp_new_i32();
        tcg_gen_ld_i32(t, tcg_env, vwr_offset(a->vwp));
        gen_store_fpr32(ctx, t, a->fd);
    }
    return true;
}

static bool trans_MFFPUW(DisasContext *ctx, arg_MFFPUW *a)
{
    TCGv_i32 t;

    if (check_mxu_cu2(ctx)) {
        t = tcg_temp_new_i32();
        gen_load_fpr32(ctx, t, a->fs);
        tcg_gen_st_i32(t, tcg_env, vwr_offset(a->vwd));
    }
    return true;
}

/* Conditional word move and piecewise mapping */

static bool trans_CMVW(DisasContext *ctx, arg_CMVW *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_3_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                           vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN, a->imm,
                           gen_helper_mxu3_cmvw);
    }
    return true;
}

static bool trans_PMAP(DisasContext *ctx, arg_PMAP *a)
{
    static gen_helper_gvec_3 * const fns[4] = {
        [MO_16] = gen_helper_mxu3_pmap_h,
        [MO_32] = gen_helper_mxu3_pmap_w,
    };

    if (!fns[a->df]) {
        return false;
    }
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_3_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                           vpr_offset(a->vp), MXU3_VLEN, MXU3_VLEN, 0,
                           fns[a->df]);
    }
    return true;
}

/* Load/store */

/*
 * The 2^len bytes at addr, which need no alignment, as 64-bit values
 * from the least significant. 32 bytes are two 128-bit accesses; a fault
 * on the second restarts the instruction, which is safe because the
 * registers are only written after both (or the store is redone).
 */
static void gen_load_chunk(DisasContext *ctx, TCGv_i64 *t, MemOp len,
                           TCGv addr)
{
    MemOp mop = MO_TE | MO_UNALN;
    TCGv a = tcg_temp_new();

    if (len <= MO_64) {
        tcg_gen_qemu_ld_i64(t[0], addr, ctx->mem_idx, mop | len);
        return;
    }
    for (int i = 0; i < 1 << (len - MO_128); i++) {
        TCGv_i128 v = tcg_temp_new_i128();

        tcg_gen_addi_tl(a, addr, i * 16);
        tcg_gen_qemu_ld_i128(v, a, ctx->mem_idx,
                             mop | MO_128 | MO_ATOM_IFALIGN_PAIR);
        tcg_gen_extr_i128_i64(t[2 * i], t[2 * i + 1], v);
    }
}

static void gen_store_chunk(DisasContext *ctx, TCGv_i64 *t, MemOp len,
                            TCGv addr)
{
    MemOp mop = MO_TE | MO_UNALN;
    TCGv a = tcg_temp_new();

    if (len <= MO_64) {
        tcg_gen_qemu_st_i64(t[0], addr, ctx->mem_idx, mop | len);
        return;
    }
    for (int i = 0; i < 1 << (len - MO_128); i++) {
        TCGv_i128 v = tcg_temp_new_i128();

        tcg_gen_addi_tl(a, addr, i * 16);
        tcg_gen_concat_i64_i128(v, t[2 * i], t[2 * i + 1]);
        tcg_gen_qemu_st_i128(v, a, ctx->mem_idx,
                             mop | MO_128 | MO_ATOM_IFALIGN_PAIR);
    }
}

/* Element n of vr, of 2^esz bytes, and the memory at addr */
static void gen_ldst_element(DisasContext *ctx, int vr, int n, MemOp esz,
                             TCGv addr, bool store)
{
    int units = esz <= MO_64 ? 1 : 1 << (esz - MO_64);
    int ofs = vpr_element_offset(vr, n, esz);
    TCGv_i64 t[4];

    for (int i = 0; i < units; i++) {
        t[i] = tcg_temp_new_i64();
    }
    if (store) {
        for (int i = 0; i < units; i++) {
            if (esz == MO_32) {
                tcg_gen_ld32u_i64(t[i], tcg_env, ofs);
            } else {
                tcg_gen_ld_i64(t[i], tcg_env, ofs + i * 8);
            }
        }
        gen_store_chunk(ctx, t, esz, addr);
    } else {
        gen_load_chunk(ctx, t, esz, addr);
        for (int i = 0; i < units; i++) {
            if (esz == MO_32) {
                tcg_gen_st32_i64(t[i], tcg_env, ofs);
            } else {
                tcg_gen_st_i64(t[i], tcg_env, ofs + i * 8);
            }
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
