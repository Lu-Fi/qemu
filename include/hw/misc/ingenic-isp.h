/*
 * Ingenic ISP (Tiziano) and VIC: the frame path
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_ISP_H
#define HW_MISC_INGENIC_ISP_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_ISP "ingenic-isp"
OBJECT_DECLARE_TYPE(IngenicIspState, IngenicIspClass, INGENIC_ISP)

#define TYPE_INGENIC_T21_ISP "ingenic-t21-isp"
#define TYPE_INGENIC_T23_ISP "ingenic-t23-isp"
#define TYPE_INGENIC_T30_MSCALER "ingenic-t30-mscaler"
#define TYPE_INGENIC_T31_ISP "ingenic-t31-isp"
#define TYPE_INGENIC_T32_ISP "ingenic-t32-isp"
#define TYPE_INGENIC_T33_ISP "ingenic-t33-isp"
#define TYPE_INGENIC_T32PRO_ISP "ingenic-t32pro-isp"
#define TYPE_INGENIC_T40_ISP "ingenic-t40-isp"
#define TYPE_INGENIC_T41_ISP "ingenic-t41-isp"

#define INGENIC_ISP_IOSIZE  0x100000
#define INGENIC_ISP_PIPES   2
#define INGENIC_ISP_CHANS   3
#define INGENIC_ISP_VICS    3
#define INGENIC_ISP_FIFO    16
#define INGENIC_ISP_SENSORS 4

typedef struct IngenicIspChan {
    /* Buffers queued for the next frames, and the ones filled */
    uint32_t y[INGENIC_ISP_FIFO], uv[INGENIC_ISP_FIFO];
    uint32_t y_rd, y_n, uv_rd, uv_n;
    uint32_t done_y[INGENIC_ISP_FIFO], done_uv[INGENIC_ISP_FIFO];
    uint32_t done_frame[INGENIC_ISP_FIFO];
    uint32_t done_rd, done_n;
    uint32_t popped_uv, popped_frame;   /* of the entry the Y read popped */
} IngenicIspChan;

typedef struct IngenicIspLayout IngenicIspLayout;

struct IngenicIspClass {
    SysBusDeviceClass parent_class;
    const IngenicIspLayout *layout;
};

struct IngenicIspState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq isp_irq;
    qemu_irq vic_irq[INGENIC_ISP_VICS];
    qemu_irq ivdc_frame[INGENIC_ISP_SENSORS];  /* frames sent over IVDC */
    QEMUTimer *frame_timer;

    uint32_t regs[INGENIC_ISP_IOSIZE / 4];
    IngenicIspChan chan[INGENIC_ISP_PIPES][INGENIC_ISP_CHANS];
    uint32_t frame;
    uint32_t vic_running;               /* a bit per VIC */
    uint32_t shadow_value;
    bool shadow_pending;
    bool rcfg_running;                  /* in a register list */
    bool gated;                         /* its clock is off */
    bool csi_reset;                     /* its CSI receiver is held */
    bool osd_stalled;                   /* logged */
};

/*
 * The picture channel 0 sends over IVDC, as the ISP draws it, top-left in
 * an NV12 buffer w x h (both even); past the channel's own size its last
 * row and column repeat
 */
void ingenic_isp_render(IngenicIspState *s, uint32_t w, uint32_t h,
                        uint8_t *nv12);

#endif /* HW_MISC_INGENIC_ISP_H */
