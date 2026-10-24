/*
 * Ingenic camera SoC RISC-V co-processor CSRs, as the cores read them
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "target/riscv/tcg/csr.h"

static RISCVException any(CPURISCVState *env, int csrno)
{
    return RISCV_EXCP_NONE;
}

static RISCVException none(CPURISCVState *env, int csrno)
{
    return RISCV_EXCP_ILLEGAL_INST;
}

static RISCVException read_zero(CPURISCVState *env, int csrno,
                                target_ulong *val)
{
    *val = 0;
    return RISCV_EXCP_NONE;
}

static RISCVException write_ignore(CPURISCVState *env, int csrno,
                                   target_ulong val, uintptr_t ra)
{
    return RISCV_EXCP_NONE;
}

/* The T31's: no counters, and every CSR it lacks reads 0 */
static RISCVException read_t31_mtvec(CPURISCVState *env, int csrno,
                                     target_ulong *val)
{
    *val = env->mtvec & ~3;     /* where it boots */
    return RISCV_EXCP_NONE;
}

#define T31_ZERO(csr, name) \
    { .csrno = csr, .csr_ops = { name, any, read_zero, write_ignore } }

const RISCVCSR ingenic_t31_csr_list[] = {
    T31_ZERO(CSR_MISA, "misa"),
    T31_ZERO(CSR_MARCHID, "marchid"),
    T31_ZERO(CSR_MIE, "mie"),
    {
        .csrno = CSR_MTVEC,
        .csr_ops = { "mtvec", any, read_t31_mtvec, write_ignore }
    },
    T31_ZERO(CSR_MCYCLE, "mcycle"),
    T31_ZERO(CSR_MINSTRET, "minstret"),
    T31_ZERO(CSR_MCYCLEH, "mcycleh"),
    T31_ZERO(CSR_MINSTRETH, "minstreth"),
    T31_ZERO(CSR_CYCLE, "cycle"),
    T31_ZERO(CSR_TIME, "time"),
    T31_ZERO(CSR_INSTRET, "instret"),
    T31_ZERO(CSR_CYCLEH, "cycleh"),
    T31_ZERO(CSR_TIMEH, "timeh"),
    T31_ZERO(CSR_INSTRETH, "instreth"),
    T31_ZERO(CSR_MCOUNTINHIBIT, "mcountinhibit"),
    T31_ZERO(CSR_DCSR, "dcsr"),
    T31_ZERO(CSR_MCONFIGPTR, "mconfigptr"),
    { }
};

/* Victory0 (the T32's, the T23's): vectored traps only, 256-byte aligned */
static RISCVException write_victory0_mtvec(CPURISCVState *env, int csrno,
                                           target_ulong val, uintptr_t ra)
{
    env->mtvec = (val & ~0xff) | 1;
    return RISCV_EXCP_NONE;
}

static RISCVException read_mtvec(CPURISCVState *env, int csrno,
                                 target_ulong *val)
{
    *val = env->mtvec;
    return RISCV_EXCP_NONE;
}

const RISCVCSR ingenic_victory0_csr_list[] = {
    {
        .csrno = CSR_MTVEC,
        .csr_ops = { "mtvec", any, read_mtvec, write_victory0_mtvec }
    },
    { .csrno = CSR_MSTATUSH, .csr_ops = { "mstatush", none } },
    { .csrno = CSR_CYCLE, .csr_ops = { "cycle", none } },
    { .csrno = CSR_TIME, .csr_ops = { "time", none } },
    { .csrno = CSR_INSTRET, .csr_ops = { "instret", none } },
    { .csrno = CSR_CYCLEH, .csr_ops = { "cycleh", none } },
    { .csrno = CSR_TIMEH, .csr_ops = { "timeh", none } },
    { .csrno = CSR_INSTRETH, .csr_ops = { "instreth", none } },
    { }
};

/* The T23's, an earlier Victory0 whose IDs read 0 */
const RISCVCSR ingenic_t23_csr_list[] = {
    {
        .csrno = CSR_MARCHID,
        .csr_ops = { "marchid", any, read_zero }
    },
    {
        .csrno = CSR_MTVEC,
        .csr_ops = { "mtvec", any, read_mtvec, write_victory0_mtvec }
    },
    { .csrno = CSR_MSTATUSH, .csr_ops = { "mstatush", none } },
    { .csrno = CSR_CYCLE, .csr_ops = { "cycle", none } },
    { .csrno = CSR_TIME, .csr_ops = { "time", none } },
    { .csrno = CSR_INSTRET, .csr_ops = { "instret", none } },
    { .csrno = CSR_CYCLEH, .csr_ops = { "cycleh", none } },
    { .csrno = CSR_TIMEH, .csr_ops = { "timeh", none } },
    { .csrno = CSR_INSTRETH, .csr_ops = { "instreth", none } },
    { }
};

/* The T41's and A1's: misa claims I alone, though M is there */
static RISCVException read_little_misa(CPURISCVState *env, int csrno,
                                       target_ulong *val)
{
    *val = (target_ulong)MXL_RV32 << 30 | RVI;
    return RISCV_EXCP_NONE;
}

const RISCVCSR ingenic_little_csr_list[] = {
    {
        .csrno = CSR_MISA,
        .csr_ops = { "misa", any, read_little_misa, write_ignore }
    },
    { .csrno = CSR_MSTATUSH, .csr_ops = { "mstatush", none } },
    { .csrno = CSR_TSELECT, .csr_ops = { "tselect", none } },
    { .csrno = CSR_TDATA1, .csr_ops = { "tdata1", none } },
    { .csrno = CSR_TDATA2, .csr_ops = { "tdata2", none } },
    { .csrno = CSR_TDATA3, .csr_ops = { "tdata3", none } },
    { .csrno = CSR_CYCLE, .csr_ops = { "cycle", none } },
    { .csrno = CSR_TIME, .csr_ops = { "time", none } },
    { .csrno = CSR_INSTRET, .csr_ops = { "instret", none } },
    { .csrno = CSR_CYCLEH, .csr_ops = { "cycleh", none } },
    { .csrno = CSR_TIMEH, .csr_ops = { "timeh", none } },
    { .csrno = CSR_INSTRETH, .csr_ops = { "instreth", none } },
    { }
};
