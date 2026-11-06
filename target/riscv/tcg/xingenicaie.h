/*
 * Ingenic AIE vector extension (XIngenicAIE): what the translator hands
 * its helpers
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef RISCV_XINGENICAIE_H
#define RISCV_XINGENICAIE_H

typedef enum AIEOp {
    AIE_ADD, AIE_SUB, AIE_MUL, AIE_AND, AIE_OR, AIE_XOR,
    AIE_MAXU, AIE_MAXS, AIE_MINU, AIE_MINS, AIE_MAXU4,
    AIE_UMULE, AIE_UMULO,
    AIE_FADD, AIE_FSUB, AIE_FMUL, AIE_FMAX, AIE_FMIN, AIE_FCLE, AIE_FCLT,
    AIE_ILVE, AIE_ILVO, AIE_GSHUF, AIE_BSEL,
    AIE_ORI, AIE_XORI, AIE_SLLI, AIE_SRLI, AIE_SRAI,
    AIE_SATUU, AIE_FFSI, AIE_FFUI, AIE_FTSI, AIE_FTUI,
    AIE_EXTUL, AIE_EXTUH, AIE_EXTSL, AIE_EXTSH, AIE_EXTU4L,
    AIE_GT, AIE_GT2W, AIE_PID, AIE_REPI, AIE_MOVE, AIE_LI,
    AIE_MFCPU, AIE_PDEC, AIE_PCLTU, AIE_PCLEU, AIE_PAND, AIE_PORN,
    /* memory: aie_ld and aie_st */
    AIE_MEMV, AIE_MEME, AIE_MEMP,
} AIEOp;

/* esz is log2 of the element's bytes */
#define AIE_DESC(op, esz, vd, vs1, vs2, pg) \
    ((op) | (esz) << 7 | (vd) << 9 | (vs1) << 14 | (vs2) << 19 | (pg) << 24)

#define AIE_DESC_OP(d)      ((d) & 0x7f)
#define AIE_DESC_ESZ(d)     (((d) >> 7) & 3)
#define AIE_DESC_VD(d)      (((d) >> 9) & 0x1f)
#define AIE_DESC_VS1(d)     (((d) >> 14) & 0x1f)
#define AIE_DESC_VS2(d)     (((d) >> 19) & 0x1f)
#define AIE_DESC_PG(d)      (((d) >> 24) & 3)

#endif
