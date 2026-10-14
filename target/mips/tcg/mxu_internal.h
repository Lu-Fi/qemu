/*
 * Ingenic XBurst MXU2/MXU3 SIMD helpers, shared parts
 *
 * Copyright (c) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef TARGET_MIPS_MXU_INTERNAL_H
#define TARGET_MIPS_MXU_INTERNAL_H

#include "fpu/softfloat-types.h"

/*
 * Vector registers are arrays of host-endian uint64_t, the first holding
 * the least significant bits, so on a big-endian host narrower elements
 * are reversed within each 64-bit unit.
 */
#if HOST_BIG_ENDIAN
#define H1(x)   ((x) ^ 7)
#define H2(x)   ((x) ^ 3)
#define H4(x)   ((x) ^ 1)
#else
#define H1(x)   (x)
#define H2(x)   (x)
#define H4(x)   (x)
#endif
#define H8(x)   (x)

/*
 * Floating point: mxu_fpst() clears the accumulated exception flags and
 * returns the status to compute with; mxu_fp_end() then sets MCSR.Cause,
 * raises the MXU floating-point exception for an enabled cause and
 * otherwise accumulates MCSR.Flags.
 */
float_status *mxu_fpst(CPUMIPSState *env);
void mxu_fp_end(CPUMIPSState *env, uintptr_t ra);

#endif
