/*
 * Helpers for emulation of FPU-related MIPS instructions.
 *
 *  Copyright (C) 2004-2005  Jocelyn Mayer
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "qemu/osdep.h"
#include "fpu/softfloat-helpers.h"
#include "fpu_helper.h"

/* convert MIPS rounding mode in FCR31 to IEEE library */
const FloatRoundMode ieee_rm[4] = {
    float_round_nearest_even,
    float_round_to_zero,
    float_round_up,
    float_round_down
};

void cpu_mips_restore_fp_status(CPUMIPSState *env)
{
    restore_fp_status(env);
}

/* Call after Status.FR changes; FCSR.NAN2008/ABS2008 read back as FR */
void cpu_mips_update_fpu_nan_mode(CPUMIPSState *env)
{
    const uint32_t mask = (1 << FCR31_ABS2008) | (1 << FCR31_NAN2008);
    uint32_t bits = env->CP0_Status & (1 << CP0St_FR) ? mask : 0;

    if (!(env->fpu_flags & MIPS_FPU_NAN_FR) ||
        (env->active_fpu.fcr31 & mask) == bits) {
        return;
    }
    env->active_fpu.fcr31 = (env->active_fpu.fcr31 & ~mask) | bits;
    restore_snan_bit_mode(env);
}

const char fregnames[32][4] = {
    "f0",  "f1",  "f2",  "f3",  "f4",  "f5",  "f6",  "f7",
    "f8",  "f9",  "f10", "f11", "f12", "f13", "f14", "f15",
    "f16", "f17", "f18", "f19", "f20", "f21", "f22", "f23",
    "f24", "f25", "f26", "f27", "f28", "f29", "f30", "f31",
};
