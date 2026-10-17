/*
 * Ingenic T31 ISP (Tiziano) and VIC: the frame path
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T31_ISP_H
#define HW_MISC_INGENIC_T31_ISP_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_ISP "ingenic-t31-isp"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31IspState, INGENIC_T31_ISP)

#define INGENIC_T31_ISP_IOSIZE  0x100000        /* the VIC at +0xe0000 */
#define INGENIC_T31_ISP_CHANS   3
#define INGENIC_T31_ISP_FIFO    16

typedef struct IngenicT31IspChan {
    /* Buffers queued for the next frames, and the ones filled */
    uint32_t y[INGENIC_T31_ISP_FIFO], uv[INGENIC_T31_ISP_FIFO];
    uint32_t y_rd, y_n, uv_rd, uv_n;
    uint32_t done_y[INGENIC_T31_ISP_FIFO], done_uv[INGENIC_T31_ISP_FIFO];
    uint32_t done_frame[INGENIC_T31_ISP_FIFO];
    uint32_t done_rd, done_n;
    uint32_t popped_uv, popped_frame;   /* of the entry +0x74 popped */
} IngenicT31IspChan;

struct IngenicT31IspState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq isp_irq;
    qemu_irq vic_irq;
    QEMUTimer *frame_timer;

    uint32_t regs[INGENIC_T31_ISP_IOSIZE / 4];
    IngenicT31IspChan chan[INGENIC_T31_ISP_CHANS];
    uint32_t frame;
    bool vic_running;
};

#endif /* HW_MISC_INGENIC_T31_ISP_H */
