/*
 * RISC-V translation routines for the Ingenic AIE vector extension
 * (XIngenicAIE); the work is in xingenicaie_helper.c
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

static bool gen_aie(DisasContext *ctx, int op, int esz, int vd, int vs1,
                    int vs2, int pg, int imm, TCGv x)
{
    if (!ctx->cfg_ptr->ext_xingenicaie) {
        return false;
    }
    gen_helper_aie_vop(tcg_env,
                       tcg_constant_i32(AIE_DESC(op, esz, vd, vs1, vs2, pg)),
                       tcg_constant_i32(imm), x ? x : tcg_constant_tl(0));
    return true;
}

/* A load or store at rs1 + off; inc, if set, is then added to rs1 */
static bool gen_aie_mem(DisasContext *ctx, bool st, int op, int esz, int vd,
                        int pg, int idx, int rs1, int off, int inc)
{
    TCGv addr;
    TCGv_i32 desc;

    if (!ctx->cfg_ptr->ext_xingenicaie) {
        return false;
    }
    addr = get_address(ctx, rs1, off);
    desc = tcg_constant_i32(AIE_DESC(op, esz, vd, 0, 0, pg));
    decode_save_opc(ctx, 0);
    if (st) {
        gen_helper_aie_st(tcg_env, desc, tcg_constant_i32(idx), addr);
    } else {
        gen_helper_aie_ld(tcg_env, desc, tcg_constant_i32(idx), addr);
    }
    if (inc) {
        TCGv t = tcg_temp_new();

        tcg_gen_addi_tl(t, get_gpr(ctx, rs1, EXT_NONE), inc);
        gen_set_gpr(ctx, rs1, t);
    }
    return true;
}

static bool trans_aie_law(DisasContext *ctx, arg_aie_elem *a)
{
    return gen_aie_mem(ctx, false, AIE_MEME, MO_32, a->vd, a->pg, a->idx,
                       a->rs1, a->imm << MO_32, 0);
}

static bool trans_aie_satuuhb(DisasContext *ctx, arg_aie_v *a)
{
    /* the saturations clamp in their source's elements */
    return gen_aie(ctx, AIE_SATUU, MO_16, a->vd, a->vs1, 0, a->pg, 8, NULL);
}

static bool trans_aie_satuuwb(DisasContext *ctx, arg_aie_v *a)
{
    return gen_aie(ctx, AIE_SATUU, MO_32, a->vd, a->vs1, 0, a->pg, 8, NULL);
}

static bool trans_aie_satuuwh(DisasContext *ctx, arg_aie_v *a)
{
    return gen_aie(ctx, AIE_SATUU, MO_32, a->vd, a->vs1, 0, a->pg, 16, NULL);
}

static bool trans_aie_pdec(DisasContext *ctx, arg_aie_mem *a)
{
    return gen_aie(ctx, AIE_PDEC, MO_8, a->vd, 0, 0, 0, 0,
                   get_gpr(ctx, a->rs1, EXT_NONE));
}

static bool trans_aie_psubs(DisasContext *ctx, arg_r *a)
{
    TCGv dest, t;

    /* rd = rs1 - rs2, or 0 if that is below 0 */
    if (!ctx->cfg_ptr->ext_xingenicaie) {
        return false;
    }
    dest = dest_gpr(ctx, a->rd);
    t = tcg_temp_new();
    tcg_gen_sub_tl(t, get_gpr(ctx, a->rs1, EXT_SIGN),
                   get_gpr(ctx, a->rs2, EXT_SIGN));
    tcg_gen_smax_tl(dest, t, tcg_constant_tl(0));
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_aie_pld(DisasContext *ctx, arg_aie_mem *a)
{
    /* a predicate is a bit a byte of a vector: two bytes */
    return gen_aie_mem(ctx, false, AIE_MEMP, MO_16, a->vd, 0, 0, a->rs1,
                       a->imm * (AIE_VLENB / 8), 0);
}

static bool trans_aie_psd(DisasContext *ctx, arg_aie_mem *a)
{
    return gen_aie_mem(ctx, true, AIE_MEMP, MO_16, a->vd, 0, 0, a->rs1,
                       a->imm * (AIE_VLENB / 8), 0);
}

#define TRANS_AIE(NAME, OP, ESZ)                                           \
static bool trans_aie_##NAME(DisasContext *ctx, arg_aie_v *a)              \
{                                                                          \
    return gen_aie(ctx, OP, ESZ, a->vd, a->vs1, a->vs2, a->pg, a->imm,     \
                   NULL);                                                  \
}

TRANS_AIE(addb, AIE_ADD, MO_8)
TRANS_AIE(addh, AIE_ADD, MO_16)
TRANS_AIE(addw, AIE_ADD, MO_32)
TRANS_AIE(subb, AIE_SUB, MO_8)
TRANS_AIE(subh, AIE_SUB, MO_16)
TRANS_AIE(subw, AIE_SUB, MO_32)
TRANS_AIE(mulh, AIE_MUL, MO_16)
TRANS_AIE(mulw, AIE_MUL, MO_32)
TRANS_AIE(andb, AIE_AND, MO_8)
TRANS_AIE(orb, AIE_OR, MO_8)
TRANS_AIE(xorb, AIE_XOR, MO_8)
TRANS_AIE(maxub, AIE_MAXU, MO_8)
TRANS_AIE(maxuh, AIE_MAXU, MO_16)
TRANS_AIE(maxuw, AIE_MAXU, MO_32)
TRANS_AIE(maxsw, AIE_MAXS, MO_32)
TRANS_AIE(minuw, AIE_MINU, MO_32)
TRANS_AIE(minsw, AIE_MINS, MO_32)
TRANS_AIE(maxu4bi, AIE_MAXU4, MO_8)
TRANS_AIE(umulhe, AIE_UMULE, MO_32)
TRANS_AIE(umulho, AIE_UMULO, MO_32)
TRANS_AIE(faddw, AIE_FADD, MO_32)
TRANS_AIE(fsubw, AIE_FSUB, MO_32)
TRANS_AIE(fmulw, AIE_FMUL, MO_32)
TRANS_AIE(fmaxw, AIE_FMAX, MO_32)
TRANS_AIE(fminw, AIE_FMIN, MO_32)
TRANS_AIE(fclew, AIE_FCLE, MO_32)
TRANS_AIE(fcltw, AIE_FCLT, MO_32)
TRANS_AIE(ilveb, AIE_ILVE, MO_8)
TRANS_AIE(ilveh, AIE_ILVE, MO_16)
TRANS_AIE(ilvew, AIE_ILVE, MO_32)
TRANS_AIE(ilved, AIE_ILVE, MO_64)
TRANS_AIE(ilvob, AIE_ILVO, MO_8)
TRANS_AIE(ilvoh, AIE_ILVO, MO_16)
TRANS_AIE(ilvow, AIE_ILVO, MO_32)
TRANS_AIE(ilvod, AIE_ILVO, MO_64)
TRANS_AIE(gshufvb, AIE_GSHUF, MO_8)
TRANS_AIE(bselv, AIE_BSEL, MO_8)
TRANS_AIE(orib, AIE_ORI, MO_8)
TRANS_AIE(xorib, AIE_XORI, MO_8)
TRANS_AIE(sllib, AIE_SLLI, MO_8)
TRANS_AIE(sllih, AIE_SLLI, MO_16)
TRANS_AIE(slliw, AIE_SLLI, MO_32)
TRANS_AIE(srlib, AIE_SRLI, MO_8)
TRANS_AIE(srlih, AIE_SRLI, MO_16)
TRANS_AIE(srliw, AIE_SRLI, MO_32)
TRANS_AIE(sraiw, AIE_SRAI, MO_32)
TRANS_AIE(ffsiw, AIE_FFSI, MO_32)
TRANS_AIE(ffuiw, AIE_FFUI, MO_32)
TRANS_AIE(ftsiw, AIE_FTSI, MO_32)
TRANS_AIE(ftuiw, AIE_FTUI, MO_32)
TRANS_AIE(extubl, AIE_EXTUL, MO_16)
TRANS_AIE(extubh, AIE_EXTUH, MO_16)
TRANS_AIE(extuhl, AIE_EXTUL, MO_32)
TRANS_AIE(extuhh, AIE_EXTUH, MO_32)
TRANS_AIE(extshl, AIE_EXTSL, MO_32)
TRANS_AIE(extshh, AIE_EXTSH, MO_32)
TRANS_AIE(extu4bil, AIE_EXTU4L, MO_8)
TRANS_AIE(gtb, AIE_GT, MO_8)
TRANS_AIE(gth, AIE_GT, MO_16)
TRANS_AIE(gt2w, AIE_GT2W, MO_32)
TRANS_AIE(pidw, AIE_PID, MO_32)
TRANS_AIE(repib, AIE_REPI, MO_8)
TRANS_AIE(repih, AIE_REPI, MO_16)
TRANS_AIE(repiw, AIE_REPI, MO_32)
TRANS_AIE(repid, AIE_REPI, MO_64)
TRANS_AIE(movw, AIE_MOVE, MO_32)
TRANS_AIE(movd, AIE_MOVE, MO_64)
TRANS_AIE(lih, AIE_LI, MO_16)
TRANS_AIE(liw, AIE_LI, MO_32)
TRANS_AIE(pcltub, AIE_PCLTU, MO_8)
TRANS_AIE(pcleub, AIE_PCLEU, MO_8)
TRANS_AIE(pand, AIE_PAND, MO_8)
TRANS_AIE(porn, AIE_PORN, MO_8)

#define TRANS_AIE_MFCPU(NAME, ESZ)                                         \
static bool trans_aie_##NAME(DisasContext *ctx, arg_aie_mem *a)            \
{                                                                          \
    return gen_aie(ctx, AIE_MFCPU, ESZ, a->vd, 0, 0, a->pg, 0,             \
                   get_gpr(ctx, a->rs1, EXT_NONE));                        \
}

TRANS_AIE_MFCPU(mfcpub, MO_8)
TRANS_AIE_MFCPU(mfcpuh, MO_16)
TRANS_AIE_MFCPU(mfcpuw, MO_32)

#define TRANS_AIE_LDST(NAME, ST, ESZ)                                      \
static bool trans_aie_##NAME(DisasContext *ctx, arg_aie_mem *a)            \
{                                                                          \
    return gen_aie_mem(ctx, ST, AIE_MEMV, ESZ, a->vd, a->pg, 0, a->rs1,    \
                       a->imm * AIE_VLENB, 0);                             \
}

TRANS_AIE_LDST(lavb, false, MO_8)
TRANS_AIE_LDST(lavh, false, MO_16)
TRANS_AIE_LDST(lavw, false, MO_32)
TRANS_AIE_LDST(savb, true, MO_8)
TRANS_AIE_LDST(savh, true, MO_16)
TRANS_AIE_LDST(savw, true, MO_32)

#define TRANS_AIE_LDSTU(NAME, ST, ESZ)                                     \
static bool trans_aie_##NAME(DisasContext *ctx, arg_aie_mem *a)            \
{                                                                          \
    return gen_aie_mem(ctx, ST, AIE_MEMV, ESZ, a->vd, a->pg, 0, a->rs1,    \
                       0, AIE_VLENB);                                      \
}

TRANS_AIE_LDSTU(luvb, false, MO_8)
TRANS_AIE_LDSTU(luvh, false, MO_16)
TRANS_AIE_LDSTU(luvw, false, MO_32)
TRANS_AIE_LDSTU(suvb, true, MO_8)
TRANS_AIE_LDSTU(suvh, true, MO_16)
TRANS_AIE_LDSTU(suvw, true, MO_32)

#define TRANS_AIE_LDSTE(NAME, ST, ESZ)                                     \
static bool trans_aie_##NAME(DisasContext *ctx, arg_aie_elem *a)           \
{                                                                          \
    return gen_aie_mem(ctx, ST, AIE_MEME, ESZ, a->vd, a->pg, a->idx,       \
                       a->rs1, a->imm << ESZ, 0);                          \
}

TRANS_AIE_LDSTE(lad, false, MO_64)
TRANS_AIE_LDSTE(saw, true, MO_32)
TRANS_AIE_LDSTE(sad, true, MO_64)
