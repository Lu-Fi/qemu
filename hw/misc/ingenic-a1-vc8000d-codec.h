/*
 * Ingenic A1 VC8000D: the picture decoding behind the register model
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_VC8000D_CODEC_H
#define HW_MISC_INGENIC_A1_VC8000D_CODEC_H

#include "hw/misc/ingenic-a1-vc8000d.h"

Vc8000dCodec *vc8000d_codec_new(IngenicA1Vc8000dState *s);

/* Drop every stream's decoder state (a reset) */
void vc8000d_codec_reset(Vc8000dCodec *c);

/* Decode the picture the core's registers describe, report in them */
void vc8000d_codec_run(IngenicA1Vc8000dState *s);

#endif
