/*
 * Ingenic T10/T20 ISP (an Apical core) and VIC: the frame path
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T20_ISP_H
#define HW_MISC_INGENIC_T20_ISP_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_T20_ISP "ingenic-t20-isp"
OBJECT_DECLARE_TYPE(IngenicT20IspState, IngenicT20IspClass, INGENIC_T20_ISP)

#define TYPE_INGENIC_T10_ISP "ingenic-t10-isp"

#define INGENIC_T20_ISP_IOSIZE      0x100000
#define INGENIC_T20_ISP_WRITERS     6       /* FR, DS1, DS2; Y and UV each */

/* An Apical video DMA writer's bank pointers */
typedef struct IngenicT20IspWriter {
    uint32_t curr, last;
    uint32_t base_curr, base_last;
    uint32_t frames;
} IngenicT20IspWriter;

struct IngenicT20IspClass {
    SysBusDeviceClass parent_class;
    hwaddr vic_ctrl;
};

struct IngenicT20IspState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *frame_timer;

    uint32_t regs[INGENIC_T20_ISP_IOSIZE / 4];
    IngenicT20IspWriter writer[INGENIC_T20_ISP_WRITERS];
    uint32_t status;                /* the Apical interrupt slots' */
    uint32_t vic_status;            /* the glue's VIC bits, 22:16 */
    uint32_t frame;
    bool running;
    bool mid_frame;                 /* frame started, not yet written */
};

#endif /* HW_MISC_INGENIC_T20_ISP_H */
