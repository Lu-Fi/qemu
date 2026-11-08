/*
 * Ingenic T10/T20/T30 ISP (an Apical core) and VIC: the frame path
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T20_ISP_H
#define HW_MISC_INGENIC_T20_ISP_H

#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-frame-tap.h"
#include "hw/sensor/image-sensor.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_INGENIC_T20_ISP "ingenic-t20-isp"
OBJECT_DECLARE_TYPE(IngenicT20IspState, IngenicT20IspClass, INGENIC_T20_ISP)

#define TYPE_INGENIC_T10_ISP "ingenic-t10-isp"
#define TYPE_INGENIC_T30_ISP "ingenic-t30-isp"

#define INGENIC_T20_ISP_IOSIZE      0x100000
#define INGENIC_T20_ISP_WRITERS     6       /* FR, DS1, DS2; Y and UV each */

/*
 * What the core keeps up with: a frame is put out only while the load, the
 * active width times the sensor's line rate over the ISP clock (in
 * thousandths), is within a row's; @keep_pm of a thousand frames then pass
 */
typedef struct IngenicT20IspKeep {
    uint32_t load_pm, keep_pm;
} IngenicT20IspKeep;

/* The frames, in thousandths, that pass at @load */
uint32_t ingenic_t20_isp_keep_pm(const IngenicT20IspKeep *rows, unsigned n,
                                 uint32_t load);

/* An Apical video DMA writer's bank pointers */
typedef struct IngenicT20IspWriter {
    uint32_t curr, last;
    uint32_t base_curr, base_last;
    uint32_t frames;
} IngenicT20IspWriter;

struct IngenicT20IspClass {
    SysBusDeviceClass parent_class;
    hwaddr vic_ctrl;
    /* T30: the writers feed the noise reducer and multi-scaler, not memory */
    bool line_buffer;
    /* T30: the writers (bit n for writer n) whose raw output is zeros */
    uint8_t raw_blank;
    /* Temper's change that restarts its count at 6, at a noise level of 0 */
    double temper_u0;
    /* The CPM divider of its clock; the frames it keeps up with, per load */
    hwaddr ispcdr;
    const IngenicT20IspKeep *keep;
    unsigned keeps;
};

struct IngenicT20IspState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    IngenicFrameTap tap;
    qemu_irq irq;
    qemu_irq frame_start;           /* T30: a frame into the line buffer */
    qemu_irq frame_out;             /* T30: a frame for the noise reducer */
    QEMUTimer *frame_timer;
    Object *cpm;                    /* the clock the core runs at */
    uint32_t keep_acc;              /* frames let out so far, in thousandths */

    uint32_t regs[INGENIC_T20_ISP_IOSIZE / 4];
    IngenicT20IspWriter writer[INGENIC_T20_ISP_WRITERS];
    uint32_t status;                /* the Apical interrupt slots' */
    uint32_t vic_status;            /* the glue's VIC bits, 22:16 */
    uint32_t frame;
    bool running;
    bool mid_frame;                 /* frame started, not yet written */
    bool irq_level;                 /* as last traced */
    /* Temper's change to this frame's pixels (filtered - frame), if any */
    int16_t *temper_delta;
    uint32_t temper_w, temper_h;
    /* Temper's buffers, kept from frame to frame (pixels, t20_isp_temper) */
    uint32_t *temper_words;
    uint16_t *temper_chg;
    int16_t *temper_buf;
    size_t temper_cap, temper_chg_cap;
};

/*
 * The white balance's gains (R 0x80300, Gr 0x80304, Gb 0x80308, B
 * 0x8030c, 1.0 = 0x100) as factors on the linear light of the scene's R, G
 * and B, with the raw response of a grey in @sensor's light; false until the
 * firmware sets them
 */
bool ingenic_t20_isp_wb_factors(IngenicT20IspState *s,
                                ImageSensorState *sensor, double f[3]);

#endif /* HW_MISC_INGENIC_T20_ISP_H */
