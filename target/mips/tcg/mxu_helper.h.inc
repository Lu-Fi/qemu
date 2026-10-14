/*
 * Ingenic XBurst MXU2/MXU3 SIMD helpers, shared parts
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

DEF_HELPER_2(mxu_ctcmxu, void, env, i32)

DEF_HELPER_FLAGS_4(mxu_maxa_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_maxa_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_maxa_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_maxa_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_mina_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_mina_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_mina_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_mina_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srar_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srar_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srar_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srar_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srlr_b, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srlr_h, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srlr_w, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_4(mxu_srlr_d, TCG_CALL_NO_RWG, void, ptr, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srari_b, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srari_h, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srari_w, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srari_d, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srlri_b, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srlri_h, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srlri_w, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
DEF_HELPER_FLAGS_3(mxu_srlri_d, TCG_CALL_NO_RWG, void, ptr, ptr, i32)
