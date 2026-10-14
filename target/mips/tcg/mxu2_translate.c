/*
 * Ingenic XBurst MXU2 (128-bit SIMD) translation routines
 *
 * MXU2 adds 32 128-bit vector registers (VPRs) and an MCSR control
 * register to XBurst1. It is coprocessor 2, encoded in the COP2 and
 * SPECIAL2 major opcodes.
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 *
 * Reference: XBurst Instruction Set Architecture MXU2 Programming Manual,
 * revision 2.16
 */

#include "qemu/osdep.h"
#include "translate.h"
#include "tcg/tcg-op-gvec.h"

static inline int times_16(DisasContext *ctx, int x)
{
    return x * 16;
}

static inline int plus_1(DisasContext *ctx, int x)
{
    return x + 1;
}

static inline int plus_2(DisasContext *ctx, int x)
{
    return x + 2;
}

/* Include the auto-generated decoder. */
#include "decode-mxu2.c.inc"

#define MXU2_VLEN 16

void mxu2_translate_init(void)
{
    /* VPRs are accessed through tcg_env, so there are no globals */
}

static inline int vpr_offset(int r)
{
    return offsetof(CPUMIPSState, active_tc.mxu2_vpr[r]);
}

/*
 * A VPR is two host-endian uint64_t, the first holding bits 63:0, so on a
 * big-endian host narrower elements are reversed within each half.
 */
static int vpr_element_offset(int r, int i, MemOp esz)
{
    int ofs = i << esz;

#if HOST_BIG_ENDIAN
    ofs ^= 8 - (1 << esz);
#endif
    return vpr_offset(r) + ofs;
}

/* Number of elements of size @esz in a VPR, minus one */
static inline int vpr_index_mask(MemOp esz)
{
    return (MXU2_VLEN >> esz) - 1;
}

#define TRANS_MXU2(NAME, FUNC, ...)                                     \
    static bool trans_##NAME(DisasContext *ctx, arg_##NAME *a)          \
    { return !check_mxu_cu2(ctx) || FUNC(ctx, a, __VA_ARGS__); }

typedef void GVecGen3Fn(unsigned, uint32_t, uint32_t, uint32_t,
                        uint32_t, uint32_t);
typedef void GVecGen2iFn(unsigned, uint32_t, uint32_t, int64_t,
                         uint32_t, uint32_t);

/* Sized element accesses, zero-extending loads */

static void gen_ld_element(TCGv_i64 val, TCGv_ptr base, int ofs, MemOp esz)
{
    switch (esz) {
    case MO_8:
        tcg_gen_ld8u_i64(val, base, ofs);
        break;
    case MO_16:
        tcg_gen_ld16u_i64(val, base, ofs);
        break;
    case MO_32:
        tcg_gen_ld32u_i64(val, base, ofs);
        break;
    case MO_64:
        tcg_gen_ld_i64(val, base, ofs);
        break;
    default:
        g_assert_not_reached();
    }
}

static void gen_st_element(TCGv_i64 val, int ofs, MemOp esz)
{
    switch (esz) {
    case MO_8:
        tcg_gen_st8_i64(val, tcg_env, ofs);
        break;
    case MO_16:
        tcg_gen_st16_i64(val, tcg_env, ofs);
        break;
    case MO_32:
        tcg_gen_st32_i64(val, tcg_env, ofs);
        break;
    case MO_64:
        tcg_gen_st_i64(val, tcg_env, ofs);
        break;
    default:
        g_assert_not_reached();
    }
}

/* Inline gvec operations */

static bool gen_gvec_3(DisasContext *ctx, arg_r3 *a, GVecGen3Fn *fn)
{
    fn(a->df, vpr_offset(a->vd), vpr_offset(a->vs), vpr_offset(a->vt),
       MXU2_VLEN, MXU2_VLEN);
    return true;
}

static bool gen_gvec_cmp(DisasContext *ctx, arg_r3 *a, TCGCond cond)
{
    tcg_gen_gvec_cmp(cond, a->df, vpr_offset(a->vd), vpr_offset(a->vs),
                     vpr_offset(a->vt), MXU2_VLEN, MXU2_VLEN);
    return true;
}

static bool gen_gvec_cmpz(DisasContext *ctx, arg_r2 *a, TCGCond cond)
{
    tcg_gen_gvec_cmpi(cond, a->df, vpr_offset(a->vd), vpr_offset(a->vs), 0,
                      MXU2_VLEN, MXU2_VLEN);
    return true;
}

static bool gen_gvec_shift_imm(DisasContext *ctx, arg_i *a, GVecGen2iFn *fn)
{
    int shift = a->imm & ((8 << a->df) - 1);

    fn(a->df, vpr_offset(a->vd), vpr_offset(a->vs), shift,
       MXU2_VLEN, MXU2_VLEN);
    return true;
}

static bool gen_gvec_logic_imm(DisasContext *ctx, arg_i *a, GVecGen2iFn *fn,
                               bool invert)
{
    fn(MO_8, vpr_offset(a->vd), vpr_offset(a->vs), a->imm,
       MXU2_VLEN, MXU2_VLEN);
    if (invert) {
        tcg_gen_gvec_not(MO_8, vpr_offset(a->vd), vpr_offset(a->vd),
                         MXU2_VLEN, MXU2_VLEN);
    }
    return true;
}

/* Out-of-line helpers, one per element size; NULL means reserved */

static bool gen_ool_3(DisasContext *ctx, arg_r3 *a,
                      gen_helper_gvec_3 * const fns[4])
{
    if (!fns[a->df]) {
        gen_reserved_instruction(ctx);
        return true;
    }
    tcg_gen_gvec_3_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                       vpr_offset(a->vt), MXU2_VLEN, MXU2_VLEN, 0,
                       fns[a->df]);
    return true;
}

static bool gen_ool_2(DisasContext *ctx, arg_r2 *a,
                      gen_helper_gvec_2 * const fns[4])
{
    tcg_gen_gvec_2_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                       MXU2_VLEN, MXU2_VLEN, 0, fns[a->df]);
    return true;
}

static bool gen_ool_imm(DisasContext *ctx, arg_i *a,
                        gen_helper_gvec_2 * const fns[4])
{
    tcg_gen_gvec_2_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                       MXU2_VLEN, MXU2_VLEN, a->imm, fns[a->df]);
    return true;
}

/* Floating point helpers take env for MCSR and the float status */

static bool gen_fp_3(DisasContext *ctx, arg_r3 *a,
                     gen_helper_gvec_3_ptr * const fns[4])
{
    tcg_gen_gvec_3_ptr(vpr_offset(a->vd), vpr_offset(a->vs),
                       vpr_offset(a->vt), tcg_env, MXU2_VLEN, MXU2_VLEN, 0,
                       fns[a->df]);
    return true;
}

static bool gen_fp_2(DisasContext *ctx, arg_r2 *a,
                     gen_helper_gvec_2_ptr * const fns[4])
{
    tcg_gen_gvec_2_ptr(vpr_offset(a->vd), vpr_offset(a->vs), tcg_env,
                       MXU2_VLEN, MXU2_VLEN, 0, fns[a->df]);
    return true;
}

#define MXU2_FNS_BHWD(NAME)                                             \
    static gen_helper_gvec_3 * const NAME##_fns[4] = {                  \
        gen_helper_mxu2_##NAME##_b, gen_helper_mxu2_##NAME##_h,         \
        gen_helper_mxu2_##NAME##_w, gen_helper_mxu2_##NAME##_d,         \
    }

#define MXU2_FNS_HWD(NAME)                                              \
    static gen_helper_gvec_3 * const NAME##_fns[4] = {                  \
        NULL, gen_helper_mxu2_##NAME##_h,                               \
        gen_helper_mxu2_##NAME##_w, gen_helper_mxu2_##NAME##_d,         \
    }

#define MXU2_FNS_HW(NAME)                                               \
    static gen_helper_gvec_3 * const NAME##_fns[4] = {                  \
        [MO_16] = gen_helper_mxu2_##NAME##_h,                           \
        [MO_32] = gen_helper_mxu2_##NAME##_w,                           \
    }

#define MXU2_FNS2_BHWD(NAME)                                            \
    static gen_helper_gvec_2 * const NAME##_fns[4] = {                  \
        gen_helper_mxu2_##NAME##_b, gen_helper_mxu2_##NAME##_h,         \
        gen_helper_mxu2_##NAME##_w, gen_helper_mxu2_##NAME##_d,         \
    }

#define MXU2_FP_FNS(NAME)                                               \
    static gen_helper_gvec_3_ptr * const NAME##_fns[4] = {              \
        [MO_32] = gen_helper_mxu2_##NAME##_w,                           \
        [MO_64] = gen_helper_mxu2_##NAME##_d,                           \
    }

#define MXU2_FP2_FNS(NAME)                                              \
    static gen_helper_gvec_2_ptr * const NAME##_fns[4] = {              \
        [MO_32] = gen_helper_mxu2_##NAME##_w,                           \
        [MO_64] = gen_helper_mxu2_##NAME##_d,                           \
    }

/* Compare */

TRANS_MXU2(CEQ, gen_gvec_cmp, TCG_COND_EQ)
TRANS_MXU2(CNE, gen_gvec_cmp, TCG_COND_NE)
TRANS_MXU2(CLTS, gen_gvec_cmp, TCG_COND_LT)
TRANS_MXU2(CLTU, gen_gvec_cmp, TCG_COND_LTU)
TRANS_MXU2(CLES, gen_gvec_cmp, TCG_COND_LE)
TRANS_MXU2(CLEU, gen_gvec_cmp, TCG_COND_LEU)
TRANS_MXU2(CEQZ, gen_gvec_cmpz, TCG_COND_EQ)
TRANS_MXU2(CNEZ, gen_gvec_cmpz, TCG_COND_NE)
TRANS_MXU2(CLTZ, gen_gvec_cmpz, TCG_COND_LT)
TRANS_MXU2(CLEZ, gen_gvec_cmpz, TCG_COND_LE)

/* Integer arithmetic */

MXU2_FNS_BHWD(maxa);
MXU2_FNS_BHWD(mina);
MXU2_FNS_BHWD(adda);
MXU2_FNS_BHWD(addas);
MXU2_FNS_BHWD(subsa);
MXU2_FNS_BHWD(subua);
MXU2_FNS_BHWD(subus);
MXU2_FNS_BHWD(aves);
MXU2_FNS_BHWD(avers);
MXU2_FNS_BHWD(aveu);
MXU2_FNS_BHWD(averu);
MXU2_FNS_BHWD(divs);
MXU2_FNS_BHWD(divu);
MXU2_FNS_BHWD(mods);
MXU2_FNS_BHWD(modu);
MXU2_FNS_BHWD(madd);
MXU2_FNS_BHWD(msub);
MXU2_FNS_BHWD(srar);
MXU2_FNS_BHWD(srlr);

TRANS_MXU2(MAXA, gen_ool_3, maxa_fns)
TRANS_MXU2(MINA, gen_ool_3, mina_fns)
TRANS_MXU2(MAXS, gen_gvec_3, tcg_gen_gvec_smax)
TRANS_MXU2(MINS, gen_gvec_3, tcg_gen_gvec_smin)
TRANS_MXU2(MAXU, gen_gvec_3, tcg_gen_gvec_umax)
TRANS_MXU2(MINU, gen_gvec_3, tcg_gen_gvec_umin)
TRANS_MXU2(ADDA, gen_ool_3, adda_fns)
TRANS_MXU2(ADDAS, gen_ool_3, addas_fns)
TRANS_MXU2(SUBSA, gen_ool_3, subsa_fns)
TRANS_MXU2(SUBUA, gen_ool_3, subua_fns)
TRANS_MXU2(SUBUS, gen_ool_3, subus_fns)
TRANS_MXU2(ADDSS, gen_gvec_3, tcg_gen_gvec_ssadd)
TRANS_MXU2(SUBSS, gen_gvec_3, tcg_gen_gvec_sssub)
TRANS_MXU2(ADDUU, gen_gvec_3, tcg_gen_gvec_usadd)
TRANS_MXU2(SUBUU, gen_gvec_3, tcg_gen_gvec_ussub)
TRANS_MXU2(ADD, gen_gvec_3, tcg_gen_gvec_add)
TRANS_MXU2(SUB, gen_gvec_3, tcg_gen_gvec_sub)
TRANS_MXU2(AVES, gen_ool_3, aves_fns)
TRANS_MXU2(AVERS, gen_ool_3, avers_fns)
TRANS_MXU2(AVEU, gen_ool_3, aveu_fns)
TRANS_MXU2(AVERU, gen_ool_3, averu_fns)
TRANS_MXU2(MUL, gen_gvec_3, tcg_gen_gvec_mul)
TRANS_MXU2(DIVS, gen_ool_3, divs_fns)
TRANS_MXU2(DIVU, gen_ool_3, divu_fns)
TRANS_MXU2(MODS, gen_ool_3, mods_fns)
TRANS_MXU2(MODU, gen_ool_3, modu_fns)
TRANS_MXU2(MADD, gen_ool_3, madd_fns)
TRANS_MXU2(MSUB, gen_ool_3, msub_fns)

static bool gen_sat(DisasContext *ctx, arg_i *a, bool is_signed)
{
    static gen_helper_gvec_2 * const sats_fns[4] = {
        gen_helper_mxu2_sats_b, gen_helper_mxu2_sats_h,
        gen_helper_mxu2_sats_w, gen_helper_mxu2_sats_d,
    };
    static gen_helper_gvec_2 * const satu_fns[4] = {
        gen_helper_mxu2_satu_b, gen_helper_mxu2_satu_h,
        gen_helper_mxu2_satu_w, gen_helper_mxu2_satu_d,
    };

    return gen_ool_imm(ctx, a, is_signed ? sats_fns : satu_fns);
}

TRANS_MXU2(SATS, gen_sat, true)
TRANS_MXU2(SATU, gen_sat, false)

/* Dot product */

MXU2_FNS_HWD(dotps);
MXU2_FNS_HWD(dotpu);
MXU2_FNS_HWD(dadds);
MXU2_FNS_HWD(daddu);
MXU2_FNS_HWD(dsubs);
MXU2_FNS_HWD(dsubu);

TRANS_MXU2(DOTPS, gen_ool_3, dotps_fns)
TRANS_MXU2(DOTPU, gen_ool_3, dotpu_fns)
TRANS_MXU2(DADDS, gen_ool_3, dadds_fns)
TRANS_MXU2(DADDU, gen_ool_3, daddu_fns)
TRANS_MXU2(DSUBS, gen_ool_3, dsubs_fns)
TRANS_MXU2(DSUBU, gen_ool_3, dsubu_fns)

/* Bitwise */

MXU2_FNS2_BHWD(loc);
MXU2_FNS2_BHWD(lzc);
MXU2_FNS2_BHWD(bcnt);

TRANS_MXU2(LOC, gen_ool_2, loc_fns)
TRANS_MXU2(LZC, gen_ool_2, lzc_fns)
TRANS_MXU2(BCNT, gen_ool_2, bcnt_fns)
TRANS_MXU2(ANDV, gen_gvec_3, tcg_gen_gvec_and)
TRANS_MXU2(NORV, gen_gvec_3, tcg_gen_gvec_nor)
TRANS_MXU2(ORV, gen_gvec_3, tcg_gen_gvec_or)
TRANS_MXU2(XORV, gen_gvec_3, tcg_gen_gvec_xor)
TRANS_MXU2(ANDIB, gen_gvec_logic_imm, tcg_gen_gvec_andi, false)
TRANS_MXU2(NORIB, gen_gvec_logic_imm, tcg_gen_gvec_ori, true)
TRANS_MXU2(ORIB, gen_gvec_logic_imm, tcg_gen_gvec_ori, false)
TRANS_MXU2(XORIB, gen_gvec_logic_imm, tcg_gen_gvec_xori, false)

/* vd = vr ? vt : vs, bitwise */
static bool trans_BSELV(DisasContext *ctx, arg_BSELV *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_bitsel(MO_64, vpr_offset(a->vd), vpr_offset(a->vr),
                            vpr_offset(a->vt), vpr_offset(a->vs),
                            MXU2_VLEN, MXU2_VLEN);
    }
    return true;
}

/* Shift */

TRANS_MXU2(SLL, gen_gvec_3, tcg_gen_gvec_shlv)
TRANS_MXU2(SRA, gen_gvec_3, tcg_gen_gvec_sarv)
TRANS_MXU2(SRL, gen_gvec_3, tcg_gen_gvec_shrv)
TRANS_MXU2(SRAR, gen_ool_3, srar_fns)
TRANS_MXU2(SRLR, gen_ool_3, srlr_fns)
TRANS_MXU2(SLLI, gen_gvec_shift_imm, tcg_gen_gvec_shli)
TRANS_MXU2(SRAI, gen_gvec_shift_imm, tcg_gen_gvec_sari)
TRANS_MXU2(SRLI, gen_gvec_shift_imm, tcg_gen_gvec_shri)

static bool gen_shift_round_imm(DisasContext *ctx, arg_i *a, bool arith)
{
    static gen_helper_gvec_2 * const srari_fns[4] = {
        gen_helper_mxu2_srari_b, gen_helper_mxu2_srari_h,
        gen_helper_mxu2_srari_w, gen_helper_mxu2_srari_d,
    };
    static gen_helper_gvec_2 * const srlri_fns[4] = {
        gen_helper_mxu2_srlri_b, gen_helper_mxu2_srlri_h,
        gen_helper_mxu2_srlri_w, gen_helper_mxu2_srlri_d,
    };

    return gen_ool_imm(ctx, a, arith ? srari_fns : srlri_fns);
}

TRANS_MXU2(SRARI, gen_shift_round_imm, true)
TRANS_MXU2(SRLRI, gen_shift_round_imm, false)

/* Fixed-point multiply */

MXU2_FNS_HW(mulq);
MXU2_FNS_HW(mulqr);
MXU2_FNS_HW(maddq);
MXU2_FNS_HW(maddqr);
MXU2_FNS_HW(msubq);
MXU2_FNS_HW(msubqr);

TRANS_MXU2(MULQ, gen_ool_3, mulq_fns)
TRANS_MXU2(MULQR, gen_ool_3, mulqr_fns)
TRANS_MXU2(MADDQ, gen_ool_3, maddq_fns)
TRANS_MXU2(MADDQR, gen_ool_3, maddqr_fns)
TRANS_MXU2(MSUBQ, gen_ool_3, msubq_fns)
TRANS_MXU2(MSUBQR, gen_ool_3, msubqr_fns)

/* Floating point */

MXU2_FP_FNS(fadd);
MXU2_FP_FNS(fsub);
MXU2_FP_FNS(fmul);
MXU2_FP_FNS(fdiv);
MXU2_FP_FNS(fmadd);
MXU2_FP_FNS(fmsub);
MXU2_FP_FNS(fmax);
MXU2_FP_FNS(fmaxa);
MXU2_FP_FNS(fmin);
MXU2_FP_FNS(fmina);
MXU2_FP_FNS(fcor);
MXU2_FP_FNS(fceq);
MXU2_FP_FNS(fclt);
MXU2_FP_FNS(fcle);

TRANS_MXU2(FADD, gen_fp_3, fadd_fns)
TRANS_MXU2(FSUB, gen_fp_3, fsub_fns)
TRANS_MXU2(FMUL, gen_fp_3, fmul_fns)
TRANS_MXU2(FDIV, gen_fp_3, fdiv_fns)
TRANS_MXU2(FMADD, gen_fp_3, fmadd_fns)
TRANS_MXU2(FMSUB, gen_fp_3, fmsub_fns)
TRANS_MXU2(FMAX, gen_fp_3, fmax_fns)
TRANS_MXU2(FMAXA, gen_fp_3, fmaxa_fns)
TRANS_MXU2(FMIN, gen_fp_3, fmin_fns)
TRANS_MXU2(FMINA, gen_fp_3, fmina_fns)
TRANS_MXU2(FCOR, gen_fp_3, fcor_fns)
TRANS_MXU2(FCEQ, gen_fp_3, fceq_fns)
TRANS_MXU2(FCLT, gen_fp_3, fclt_fns)
TRANS_MXU2(FCLE, gen_fp_3, fcle_fns)

MXU2_FP2_FNS(fsqrt);
MXU2_FP2_FNS(fclass);
MXU2_FP2_FNS(vcvts);
MXU2_FP2_FNS(vcvtu);
MXU2_FP2_FNS(vcvtfs);
MXU2_FP2_FNS(vcvtfu);
MXU2_FP2_FNS(vtruncs);
MXU2_FP2_FNS(vtruncu);
MXU2_FP2_FNS(vcvtr);
MXU2_FP2_FNS(vcvte);
MXU2_FP2_FNS(vcvto);
MXU2_FP2_FNS(vcvtqe);
MXU2_FP2_FNS(vcvtqo);

TRANS_MXU2(FSQRT, gen_fp_2, fsqrt_fns)
TRANS_MXU2(FCLASS, gen_fp_2, fclass_fns)
TRANS_MXU2(VCVTS, gen_fp_2, vcvts_fns)
TRANS_MXU2(VCVTU, gen_fp_2, vcvtu_fns)
TRANS_MXU2(VCVTFS, gen_fp_2, vcvtfs_fns)
TRANS_MXU2(VCVTFU, gen_fp_2, vcvtfu_fns)
TRANS_MXU2(VTRUNCS, gen_fp_2, vtruncs_fns)
TRANS_MXU2(VTRUNCU, gen_fp_2, vtruncu_fns)
TRANS_MXU2(VCVTR, gen_fp_2, vcvtr_fns)
TRANS_MXU2(VCVTE, gen_fp_2, vcvte_fns)
TRANS_MXU2(VCVTO, gen_fp_2, vcvto_fns)
TRANS_MXU2(VCVTQE, gen_fp_2, vcvtqe_fns)
TRANS_MXU2(VCVTQO, gen_fp_2, vcvtqo_fns)

static bool gen_fp_narrow(DisasContext *ctx, arg_r3 *a,
                          gen_helper_gvec_3_ptr *fn)
{
    tcg_gen_gvec_3_ptr(vpr_offset(a->vd), vpr_offset(a->vs),
                       vpr_offset(a->vt), tcg_env, MXU2_VLEN, MXU2_VLEN, 0,
                       fn);
    return true;
}

TRANS_MXU2(VCVTHS, gen_fp_narrow, gen_helper_mxu2_vcvths)
TRANS_MXU2(VCVTSD, gen_fp_narrow, gen_helper_mxu2_vcvtsd)
TRANS_MXU2(VCVTQHS, gen_fp_narrow, gen_helper_mxu2_vcvtqhs)
TRANS_MXU2(VCVTQWD, gen_fp_narrow, gen_helper_mxu2_vcvtqwd)

/* Element permute */

static bool trans_SHUFV(DisasContext *ctx, arg_SHUFV *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_4_ool(vpr_offset(a->vd), vpr_offset(a->vs),
                           vpr_offset(a->vt), vpr_offset(a->vr),
                           MXU2_VLEN, MXU2_VLEN, 0, gen_helper_mxu2_shufv);
    }
    return true;
}

static bool trans_REPI(DisasContext *ctx, arg_REPI *a)
{
    int i = a->imm & vpr_index_mask(a->df);

    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_dup_mem(a->df, vpr_offset(a->vd),
                             vpr_element_offset(a->vs, i, a->df),
                             MXU2_VLEN, MXU2_VLEN);
    }
    return true;
}

/* Replicate element GPR[rt] of vs; the index wraps to the element count */
static bool trans_REPX(DisasContext *ctx, arg_REPX *a)
{
    TCGv_i32 idx;
    TCGv_ptr ptr;
    TCGv_i64 val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }

    idx = tcg_temp_new_i32();
    ptr = tcg_temp_new_ptr();
    val = tcg_temp_new_i64();

    gen_load_gpr(idx, a->rt);
    tcg_gen_andi_i32(idx, idx, vpr_index_mask(a->df));
    tcg_gen_shli_i32(idx, idx, a->df);
#if HOST_BIG_ENDIAN
    tcg_gen_xori_i32(idx, idx, 8 - (1 << a->df));
#endif
    tcg_gen_ext_i32_ptr(ptr, idx);
    tcg_gen_add_ptr(ptr, ptr, tcg_env);
    gen_ld_element(val, ptr, vpr_offset(a->vs), a->df);
    tcg_gen_gvec_dup_i64(a->df, vpr_offset(a->vd), MXU2_VLEN, MXU2_VLEN, val);
    return true;
}

/* Insert element 0 of vs into element imm of vd */
static bool trans_INSFMXU(DisasContext *ctx, arg_INSFMXU *a)
{
    int i = a->imm & vpr_index_mask(a->df);
    TCGv_i64 val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    val = tcg_temp_new_i64();
    gen_ld_element(val, tcg_env, vpr_element_offset(a->vs, 0, a->df), a->df);
    gen_st_element(val, vpr_element_offset(a->vd, i, a->df), a->df);
    return true;
}

static bool trans_INSFCPU(DisasContext *ctx, arg_INSFCPU *a)
{
    int i = a->imm & vpr_index_mask(a->df);
    TCGv val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    if (a->df == MO_64) {
        gen_reserved_instruction(ctx);
        return true;
    }
    val = tcg_temp_new();
    gen_load_gpr(val, a->rt);
    switch (a->df) {
    case MO_8:
        tcg_gen_st8_tl(val, tcg_env, vpr_element_offset(a->vr, i, MO_8));
        break;
    case MO_16:
        tcg_gen_st16_tl(val, tcg_env, vpr_element_offset(a->vr, i, MO_16));
        break;
    case MO_32:
        tcg_gen_st32_tl(val, tcg_env, vpr_element_offset(a->vr, i, MO_32));
        break;
    default:
        g_assert_not_reached();
    }
    return true;
}

/* Move element imm of vr to GPR rt, sign- or zero-extended */
static bool gen_mtcpu(DisasContext *ctx, arg_gpr *a, bool is_signed)
{
    int ofs = vpr_element_offset(a->vr, a->imm & vpr_index_mask(a->df),
                                 a->df);
    TCGv val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    if (a->df == MO_64) {
        gen_reserved_instruction(ctx);
        return true;
    }
    val = tcg_temp_new();
    switch (a->df) {
    case MO_8:
        if (is_signed) {
            tcg_gen_ld8s_tl(val, tcg_env, ofs);
        } else {
            tcg_gen_ld8u_tl(val, tcg_env, ofs);
        }
        break;
    case MO_16:
        if (is_signed) {
            tcg_gen_ld16s_tl(val, tcg_env, ofs);
        } else {
            tcg_gen_ld16u_tl(val, tcg_env, ofs);
        }
        break;
    case MO_32:
        tcg_gen_ld32s_tl(val, tcg_env, ofs);
        break;
    default:
        g_assert_not_reached();
    }
    gen_store_gpr(val, a->rt);
    return true;
}

static bool trans_MTCPUS(DisasContext *ctx, arg_MTCPUS *a)
{
    return gen_mtcpu(ctx, a, true);
}

static bool trans_MTCPUU(DisasContext *ctx, arg_MTCPUU *a)
{
    return gen_mtcpu(ctx, a, false);
}

/* Replicate GPR rt into every element of vd */
static bool trans_MFCPU(DisasContext *ctx, arg_MFCPU *a)
{
    TCGv val;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    if (a->df == MO_64) {
        gen_reserved_instruction(ctx);
        return true;
    }
    val = tcg_temp_new();
    gen_load_gpr(val, a->rt);
    tcg_gen_gvec_dup_tl(a->df, vpr_offset(a->vd), MXU2_VLEN, MXU2_VLEN, val);
    return true;
}

/*
 * The FPR moves touch coprocessor 1 state, so they also need the FPU
 * enabled for the kernel to hand the task its own FP registers.
 */

static bool trans_MFFPU(DisasContext *ctx, arg_MFFPU *a)
{
    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    check_cp1_enabled(ctx);
    if (a->df == 0) {
        TCGv_i32 val = tcg_temp_new_i32();

        gen_load_fpr32(ctx, val, a->ft);
        tcg_gen_gvec_dup_i32(MO_32, vpr_offset(a->vd), MXU2_VLEN, MXU2_VLEN,
                             val);
    } else {
        TCGv_i64 val = tcg_temp_new_i64();

        gen_load_fpr64(ctx, val, a->ft);
        tcg_gen_gvec_dup_i64(MO_64, vpr_offset(a->vd), MXU2_VLEN, MXU2_VLEN,
                             val);
    }
    return true;
}

static bool trans_INSFFPU(DisasContext *ctx, arg_INSFFPU *a)
{
    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    check_cp1_enabled(ctx);
    if (a->df == 0) {
        TCGv_i32 val = tcg_temp_new_i32();

        gen_load_fpr32(ctx, val, a->ft);
        tcg_gen_st_i32(val, tcg_env,
                       vpr_element_offset(a->vr, a->imm & 3, MO_32));
    } else {
        TCGv_i64 val = tcg_temp_new_i64();

        gen_load_fpr64(ctx, val, a->ft);
        tcg_gen_st_i64(val, tcg_env,
                       vpr_element_offset(a->vr, a->imm & 1, MO_64));
    }
    return true;
}

static bool trans_MTFPU(DisasContext *ctx, arg_MTFPU *a)
{
    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    check_cp1_enabled(ctx);
    if (a->df == 0) {
        TCGv_i32 val = tcg_temp_new_i32();

        tcg_gen_ld_i32(val, tcg_env,
                       vpr_element_offset(a->vr, a->imm & 3, MO_32));
        gen_store_fpr32(ctx, val, a->ft);
    } else {
        TCGv_i64 val = tcg_temp_new_i64();

        tcg_gen_ld_i64(val, tcg_env,
                       vpr_element_offset(a->vr, a->imm & 1, MO_64));
        gen_store_fpr64(ctx, val, a->ft);
    }
    return true;
}

/* Load immediate: bytes take imm[7:0], wider elements sign-extend imm */
static bool trans_LI(DisasContext *ctx, arg_LI *a)
{
    if (check_mxu_cu2(ctx)) {
        tcg_gen_gvec_dup_imm(a->df, vpr_offset(a->vd), MXU2_VLEN, MXU2_VLEN,
                             a->df == MO_8 ? (uint8_t)a->imm : a->imm);
    }
    return true;
}

/* Branch */

/*
 * BEQZ branches if any element is zero, BNEZ if none is, using the SWAR
 * zero-element test on each 64-bit half. The 1Q forms test the whole
 * register. The offset counts instructions from the delay slot.
 */
static bool gen_branch(DisasContext *ctx, arg_b *a, bool nez)
{
    TCGv_i64 lo, hi;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    if (ctx->hflags & MIPS_HFLAG_BMASK) {
        gen_reserved_instruction(ctx);
        return true;
    }

    lo = tcg_temp_new_i64();
    hi = tcg_temp_new_i64();
    tcg_gen_ld_i64(lo, tcg_env, vpr_offset(a->vs));
    tcg_gen_ld_i64(hi, tcg_env, vpr_offset(a->vs) + 8);

    if (a->df == 4) {
        tcg_gen_or_i64(lo, lo, hi);
        tcg_gen_setcondi_i64(nez ? TCG_COND_NE : TCG_COND_EQ, lo, lo, 0);
    } else {
        uint64_t ones = dup_const(a->df, 1);
        uint64_t msbs = dup_const(a->df, 1ull << ((8 << a->df) - 1));
        TCGv_i64 t = tcg_temp_new_i64();

        tcg_gen_subi_i64(t, lo, ones);
        tcg_gen_andc_i64(lo, t, lo);
        tcg_gen_subi_i64(t, hi, ones);
        tcg_gen_andc_i64(hi, t, hi);
        tcg_gen_or_i64(lo, lo, hi);
        tcg_gen_andi_i64(lo, lo, msbs);
        tcg_gen_setcondi_i64(nez ? TCG_COND_EQ : TCG_COND_NE, lo, lo, 0);
    }
    tcg_gen_trunc_i64_tl(bcond, lo);

    ctx->btarget = ctx->base.pc_next + 4 + (a->offset << 2);
    ctx->hflags |= MIPS_HFLAG_BC | MIPS_HFLAG_BDS32;
    return true;
}

static bool trans_BEQZ(DisasContext *ctx, arg_BEQZ *a)
{
    return gen_branch(ctx, a, false);
}

static bool trans_BNEZ(DisasContext *ctx, arg_BNEZ *a)
{
    return gen_branch(ctx, a, true);
}

static bool trans_BEQZ_V(DisasContext *ctx, arg_BEQZ_V *a)
{
    return gen_branch(ctx, a, false);
}

static bool trans_BNEZ_V(DisasContext *ctx, arg_BNEZ_V *a)
{
    return gen_branch(ctx, a, true);
}

/* Load/store */

/*
 * The aligned forms raise an address error on a misaligned address, the
 * unaligned ones have no restriction. Neither is a single-copy atomic
 * 128-bit access.
 */
static void gen_ldst(DisasContext *ctx, int vd, TCGv addr, bool store,
                     bool aligned)
{
    MemOp mop = MO_TE | MO_128 | MO_ATOM_IFALIGN_PAIR |
                (aligned ? MO_ALIGN_16 : MO_UNALN);
    TCGv_i128 val = tcg_temp_new_i128();
    TCGv_i64 lo = tcg_temp_new_i64();
    TCGv_i64 hi = tcg_temp_new_i64();

    if (store) {
        tcg_gen_ld_i64(lo, tcg_env, vpr_offset(vd));
        tcg_gen_ld_i64(hi, tcg_env, vpr_offset(vd) + 8);
        tcg_gen_concat_i64_i128(val, lo, hi);
        tcg_gen_qemu_st_i128(val, addr, ctx->mem_idx, mop);
    } else {
        tcg_gen_qemu_ld_i128(val, addr, ctx->mem_idx, mop);
        tcg_gen_extr_i128_i64(lo, hi, val);
        tcg_gen_st_i64(lo, tcg_env, vpr_offset(vd));
        tcg_gen_st_i64(hi, tcg_env, vpr_offset(vd) + 8);
    }
}

static bool gen_ldst_imm(DisasContext *ctx, arg_ldst *a, bool store,
                         bool aligned)
{
    TCGv addr;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    addr = tcg_temp_new();
    gen_base_offset_addr(ctx, addr, a->base, a->offset);
    gen_ldst(ctx, a->vd, addr, store, aligned);
    return true;
}

static bool gen_ldst_idx(DisasContext *ctx, arg_ldst_x *a, bool store,
                         bool aligned)
{
    TCGv addr;

    if (!check_mxu_cu2(ctx)) {
        return true;
    }
    addr = tcg_temp_new();
    gen_base_index_addr(ctx, addr, a->base, a->index);
    gen_ldst(ctx, a->vd, addr, store, aligned);
    return true;
}

TRANS(LU1Q, gen_ldst_imm, false, false)
TRANS(SU1Q, gen_ldst_imm, true, false)
TRANS(LA1Q, gen_ldst_imm, false, true)
TRANS(SA1Q, gen_ldst_imm, true, true)
TRANS(LU1QX, gen_ldst_idx, false, false)
TRANS(SU1QX, gen_ldst_idx, true, false)
TRANS(LA1QX, gen_ldst_idx, false, true)
TRANS(SA1QX, gen_ldst_idx, true, true)

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
 * MXU2 owns COP2: an encoding it does not define still checks Status.CU2
 * before it is reserved. In SPECIAL2 it shares the space with the base
 * ISA and MXU1, which decode whatever it does not claim.
 */
bool decode_ase_mxu2_cop2(DisasContext *ctx, uint32_t insn)
{
    if (!decode_mxu2(ctx, insn) && check_mxu_cu2(ctx)) {
        gen_reserved_instruction(ctx);
    }
    return true;
}

bool decode_ase_mxu2_special2(DisasContext *ctx, uint32_t insn)
{
    return decode_mxu2(ctx, insn);
}
