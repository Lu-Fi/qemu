/*
 * Ingenic XBurst2 MXU3 (SIMD512) helpers
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

DEF_HELPER_FLAGS_4(mxu3_maxu_2bi, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu3_maxu_4bi, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu3_minu_2bi, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu3_minu_4bi, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
