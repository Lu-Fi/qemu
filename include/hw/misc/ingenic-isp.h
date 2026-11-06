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
#include "hw/misc/ingenic-isp-hldc.h"
#include "hw/misc/ingenic-t30-ncu.h"

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
#define INGENIC_ISP_LATCHED 128     /* words that wait for a latch */
#define INGENIC_ISP_SHD_RAM 512     /* the T41's shadow port's pairs */
#define INGENIC_ISP_SHADOW  512     /* pairs a T23's or T40's port holds */

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

/* A VIC's frames, from the sensor on its input and at that sensor's rate */
typedef struct IngenicIspVic {
    IngenicIspState *isp;
    QEMUTimer *timer;
    uint32_t frame;
    unsigned n;
    const char *waiting;                /* why it last waited, traced */
    int64_t served;                     /* when the guest served its last */
                                        /* frame behind a MIPI switch */
    uint32_t sub;                       /* T32: which of its two inputs */
} IngenicIspVic;

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
    IngenicIspVic vic[INGENIC_ISP_VICS];

    uint32_t regs[INGENIC_ISP_IOSIZE / 4];
    IngenicIspChan chan[INGENIC_ISP_PIPES][INGENIC_ISP_CHANS];
    uint32_t vic_running;               /* a bit per VIC */
    uint32_t shadow_value;
    bool shadow_pending;
    /* T41: what the latched words take next, and the port's RAM */
    uint32_t latch[INGENIC_ISP_LATCHED];
    uint32_t shd_val[INGENIC_ISP_SHD_RAM], shd_off[INGENIC_ISP_SHD_RAM];
    uint32_t shd_n;                     /* the next entry written */
    uint32_t shd_run;                   /* entries to write out, if armed */
    bool shd_open;
    bool core_idle;                     /* T41: idle requested */
    bool core_wait;                     /* the core waits for a latch */
    int64_t core_resumed;               /* restarted at */
    bool shadow_open;                   /* a run being queued */
    bool shadow_done;                   /* a run written since it opened */
    uint32_t shadow_n;                  /* pairs queued */
    uint32_t shadow_due;                /* pairs waiting for the frame end */
    uint32_t shadow_first;              /* the entry they start at */
    uint32_t shadow_val[INGENIC_ISP_SHADOW], shadow_off[INGENIC_ISP_SHADOW];
    int64_t shadow_idle_until;          /* the MSCA between frames until */
    int64_t msca_busy_at[INGENIC_ISP_PIPES];    /* its busy bits set at */
    bool rcfg_running;                  /* in a register list */
    bool gated;                         /* its clock is off */
    int64_t gated_at;                   /* since */
    int64_t vic_due[INGENIC_ISP_VICS];  /* a VIC's next frame as it was */
    uint32_t mdma_slot[INGENIC_ISP_VICS];   /* its raw capture's next */
    uint32_t bus_last;                  /* the last word read */
    uint32_t msca_chans[INGENIC_ISP_PIPES];     /* the frame's channels, */
    bool msca_latched[INGENIC_ISP_PIPES];       /* once it reached the MSCA */
                                                /* (after it, T31: those */
                                                /* that dropped it) */
    bool csi_reset;                     /* its CSI receiver is held */
    bool mclk_stopped;                  /* the sensor has no clock */
    bool switched;                      /* VIC 0 behind a MIPI switch */
    uint32_t mipi_switch;               /* its position: the input seen */
    uint32_t vic_switch;                /* the position VIC 0 drives it to */
    uint32_t chans_set;                 /* channels the driver set up for */
                                        /* the next frame behind it */
    uint32_t ivdc_input;                /* whose frame IVDC last carried */
    uint32_t ivdc_sent[INGENIC_ISP_SENSORS];     /* each ring sensor's last */
    uint32_t ivdc_frame_no[INGENIC_ISP_SENSORS]; /* input and frame number */
    uint32_t ivdc_taken[2];             /* the ring sensor each VPU took */
    /* T32 with several sensors: the core takes their frames in turn */
    QEMUTimer *core_timer;
    bool core_kicked;                   /* the driver lets it take the next */
    bool core_busy;                     /* took one whose end is unserved */
    uint32_t core_turn;                 /* the input whose turn it is */
    uint32_t ipc_ready;                 /* inputs with a frame waiting */
    uint32_t in_frame[INGENIC_ISP_SENSORS];     /* each input's frames */
    uint32_t vic_closed;                /* T32 Pro: inputs kept off the core */
    bool ivdc_coding;                   /* an encoder codes an IVDC frame */
    bool ivdc_waiting;                  /* the IVDC's waits for one */
    int64_t ivdc_held;                  /* VIC 0 waits on it since */
    bool osd_stalled;                   /* logged */
    bool isp_level;                     /* the interrupts as last traced */
    uint32_t vic_level;
    IngenicIspHldc hldc;                /* T41 */
    bool hldc_stalled;                  /* logged */
    IngenicT30NcuState *ncu;            /* T30: the noise reducer before it */
};

/*
 * The picture channel 0 sends over IVDC, as the ISP draws it, top-left in
 * an NV12 buffer w x h (both even); past the channel's own size its last
 * row and column repeat. With two sensors on one ring it is the scene of
 * whichever sent the last frame, as a late job on silicon reads it
 */
void ingenic_isp_render(IngenicIspState *s, uint32_t w, uint32_t h,
                        uint8_t *nv12);

/*
 * The same, of the frame the IVDC's encoder @vpu (0: video, 1: JPEG) took
 * last, as its "taken" output names it; before any, ingenic_isp_render()
 */
void ingenic_isp_render_vpu(IngenicIspState *s, unsigned vpu, uint32_t w,
                            uint32_t h, uint8_t *nv12);

/* An encoder starts (@on) or ends coding the frame channel 0 sent over IVDC */
void ingenic_isp_ivdc_coding(IngenicIspState *s, bool on);

#endif /* HW_MISC_INGENIC_ISP_H */
