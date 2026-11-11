/*
 * Ingenic ISP: the horizontal lens distortion correction (HLDC)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_ISP_HLDC_H
#define HW_MISC_INGENIC_ISP_HLDC_H

#include "exec/hwaddr.h"
#include "migration/vmstate.h"

#define INGENIC_HLDC_SIZE   0x48
#define INGENIC_HLDC_FIELDS 5       /* k, zoom, norm, centre, 0x24 */

typedef struct IngenicIspHldc {
    uint32_t ctrl;                                  /* 0x00 */
    uint32_t pend[2][INGENIC_HLDC_FIELDS];          /* luma, chroma */
    uint32_t act[2][INGENIC_HLDC_FIELDS];
    uint32_t latch;                                 /* 0x44 */
    bool stuck;                                     /* until a reset */
    bool ran;                                       /* a frame has started */
    bool ctl_latch;             /* T21: 0x00 bits 0 and 4 latch, no 0x44 */

    /* what ingenic_hldc_stalls() last found, recomputed on demand */
    bool cache_valid, cache_stalls;
    uint32_t cache_w, cache_h;
    uint32_t cache_act[2][INGENIC_HLDC_FIELDS];
} IngenicIspHldc;

extern const VMStateDescription vmstate_ingenic_isp_hldc;

void ingenic_hldc_reset(IngenicIspHldc *h, bool ctl_latch);

/* @reg within the block; @w x @h is the frame the ISP takes */
uint32_t ingenic_hldc_read(IngenicIspHldc *h, hwaddr reg, uint32_t w,
                           uint32_t ht);
void ingenic_hldc_write(IngenicIspHldc *h, hwaddr reg, uint32_t v);

/* At a frame start the settings asked for (0x44, the T21's 0x00) apply */
void ingenic_hldc_frame_start(IngenicIspHldc *h);

/* Whether, enabled, the settings stop the ISP for a @w x @h frame */
bool ingenic_hldc_stalls(IngenicIspHldc *h, uint32_t w, uint32_t ht);

/* Remap an NV12 frame @w x @h, its planes @w apart, in place */
void ingenic_hldc_remap(const IngenicIspHldc *h, uint8_t *luma, uint8_t *uv,
                        uint32_t w, uint32_t ht);

#endif
