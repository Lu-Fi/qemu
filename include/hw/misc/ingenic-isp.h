/*
 * Ingenic ISP (Tiziano) and VIC: the frame path
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_ISP_H
#define HW_MISC_INGENIC_ISP_H

#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "qom/object.h"
#include "hw/misc/ingenic-isp-hldc.h"
#include "hw/misc/ingenic-isp-ldc.h"
#include "hw/misc/ingenic-t20-isp.h"
#include "hw/misc/ingenic-t30-ncu.h"
#include "hw/sensor/image-sensor.h"

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
#define INGENIC_ISP_MASKS   4               /* privacy masks a channel */
#define INGENIC_ISP_VICS    3
#define INGENIC_ISP_FIFO    16
#define INGENIC_ISP_SENSORS 4
#define INGENIC_ISP_LATCHED 128     /* words that wait for a latch */
#define INGENIC_ISP_SHD_RAM 512     /* the T41's shadow port's pairs */
#define INGENIC_ISP_SHADOW  512     /* pairs a T23's or T40's port holds */
#define INGENIC_ISP_LCE_BLOCKS 45   /* T41: LCE's 5 x 9 blocks */
#define INGENIC_ISP_LCE_BINS   32
#define INGENIC_ISP_RAM_PORTS  8    /* T41: statistics RAM ports */
#define INGENIC_ISP_RAW_FIFO   4    /* T41: raw WDMA and RDMA FIFOs */
#define INGENIC_ISP_MDMA_FIFO 4     /* T32 Pro: raw queue depth, chosen */

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
typedef struct IspPool IspPool;

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

/* A picture of a frame the ISP's passes share: see isp_pic_keep() */
typedef struct IngenicIspPic {
    const void *sensor;
    uint32_t frame, w, h;
    uint8_t *buf;                       /* the picture: own, or one lent */
    uint8_t *own;
    size_t cap;
    bool valid;
} IngenicIspPic;

/* A call that came while a pass ran, to be made when the frame ends */
typedef struct IngenicIspHeld {
    void (*line)(void *opaque, int n, int level);   /* NULL: a write */
    hwaddr off;                         /* of the write; the line's n */
    uint32_t val;                       /* what it writes; the line's level */
} IngenicIspHeld;

struct IngenicIspState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    IngenicFrameTap tap;
    qemu_irq isp_irq;
    qemu_irq ldc_irq;                   /* T33, T32 Pro: IRQ_LDC */
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
    uint32_t mdma_kick[INGENIC_ISP_VICS];   /* buffers queued, count 0 */
    /*
     * T32 Pro: each VIC's raw capture takes its buffers from a Y and a UV
     * queue, and puts the filled ones in a done FIFO of each
     */
    uint32_t mdma_q[INGENIC_ISP_VICS * 2][INGENIC_ISP_MDMA_FIFO];
    uint32_t mdma_done[INGENIC_ISP_VICS * 2][INGENIC_ISP_MDMA_FIFO];
    uint8_t mdma_qn[INGENIC_ISP_VICS][2], mdma_dn[INGENIC_ISP_VICS][2];
    uint32_t bus_last;                  /* the last word read */
    /*
     * T41: the raw WDMA's queue of buffers for frames, its done FIFO, the
     * RDMA's, whether the WDMA takes the raw frame (else the IR channel's)
     * and the RDMA has been started, and the frame the RDMA feeds the core
     */
    uint32_t wdma_q[INGENIC_ISP_RAW_FIFO], wdma_done[INGENIC_ISP_RAW_FIFO];
    uint32_t rdma_done[INGENIC_ISP_RAW_FIFO];
    uint32_t wdma_head;
    uint8_t wdma_n, wdma_done_n, rdma_done_n;
    bool wdma_raw, rdma_start;
    uint32_t raw_in_addr, raw_in_stride, raw_in_bpp;    /* bpp 0: none */
    uint8_t *raw_in_v8;         /* the RDMA frame's samples, top 8 bits */
    uint32_t msca_chans[INGENIC_ISP_PIPES];     /* the frame's channels, */
    bool msca_latched[INGENIC_ISP_PIPES];       /* once it reached the MSCA */
                                                /* (after it, T31: those */
                                                /* that dropped it) */
    /*
     * Each channel's frame in its frame-drop window, and of the frame that
     * reached the MSCA, the channels whose drop it decided and those it
     * dropped
     */
    uint8_t drop_pos[INGENIC_ISP_PIPES][INGENIC_ISP_CHANS];
    uint32_t drop_decided[INGENIC_ISP_PIPES];
    uint32_t drop_skip[INGENIC_ISP_PIPES];
    /*
     * T31, T40: channel 0's frame in flight, put out from ch0_from over
     * ch0_span (0: none), and what its line counter reads without one
     */
    int64_t ch0_from, ch0_span;
    uint32_t ch0_rest;
    /* T31: channel 0 had a buffer as the frame reached the MSCA */
    bool ch0_latched;
    /* T31: each mask's position and size written, for the next load */
    uint32_t mask_next[INGENIC_ISP_CHANS][INGENIC_ISP_MASKS * 2];
    bool csi_reset;                     /* its CSI receiver is held */
    bool mclk_stopped;                  /* the sensor has no clock */
    bool switched;                      /* VIC 0 behind a MIPI switch */
    bool mdns_avx2;                     /* x-mdns-avx2: false, only the */
                                        /* default build of the filters */
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
    IngenicIspHldc hldc[2];             /* its lens correction; T40's two */
    /* T41: the GSM's and LCE's statistics RAMs, and where each port reads */
    uint32_t gsm[2][256];
    uint32_t lce[2][INGENIC_ISP_LCE_BLOCKS * (INGENIC_ISP_LCE_BINS + 1)];
    uint32_t ram_at[INGENIC_ISP_RAM_PORTS];
    uint32_t lce_bank;                  /* the bank the next frame fills */
    bool gsm_armed;                     /* a write of 1 to 0x1b000 */
    bool hldc_stalled;                  /* logged */
    IngenicIspLdc ldc;                  /* T33, T32 Pro */
    IngenicT30NcuState *ncu;            /* T30: the noise reducer before it */
    Object *cpm;                        /* the CPM: the clock the ISP runs at */
    uint32_t keep_acc;                  /* frames let through, in 1/1000ths */
    IngenicT20IspState *core;           /* T30: the Apical core before that */
    /* T23, T31: the frame its 3D noise reducer put out last */
    uint8_t *mdns_pic;
    size_t mdns_pic_len;
    const uint8_t *mdns_out;            /* what the channels take: that, or */
                                        /* a coded reference's frame */
    uint32_t mdns_w, mdns_h, mdns_frame;
    void *mdns_work[18];                /* its scratch, kept; 7 and 8 */
    size_t mdns_work_len[18];           /* the luma of the frames before */
    uint64_t mdns_post_key;             /* its last filtered frame's */
    bool mdns_post_ok;                  /* settings; the rest: work 15-17 */
    IspPool *pool;                      /* the threads that band a pass */
    /*
     * A frame whose passes run without the BQL: see isp_par(). The calls
     * that reach the ISP meanwhile (register writes, lines) wait in held;
     * txn counts the frames' entries nested
     */
    QemuCond idle;                      /* a frame has ended */
    unsigned txn;
    bool unlocked;                      /* a pass runs, the BQL released */
    bool flushing;
    GArray *held;                       /* IngenicIspHeld, in arrival order */
    QEMUBH *bh;                         /* runs the frames the timers came */
    uint32_t due;                       /* for: a bit each VIC, the core's */
    bool frame_bh;                      /* x-frame-bh: in a qtest too */
    bool sensors_held;
    ImageSensorState *held_sensor[IMAGE_SENSOR_INPUTS];
    uint32_t nbands;                    /* x-bands */
    uint32_t khz[2], khz_cdr[2];        /* the clocks as the frame began */
    bool khz_ok;
    char *poke;                         /* x-poke: writes during a pass */
    uint32_t poke_passes, poke_misses;
    IngenicIspPic pic[2];               /* what a frame's passes share */
    unsigned pic_next;
    bool pic_scope;
    uint32_t mdns_hist;                 /* how many of those it holds, */
    int mdns_hist_input;                /* of this sensor */
    int mdns_input;                     /* -1: none */
    uint32_t mdns_count;                /* T21: the frames it took */
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

/*
 * White-balance gains on an NV12 picture, the planes w x h and w apart:
 * @f the factors on the linear light of its R, G and B, 1.0 for none
 */
void ingenic_isp_wb_picture(uint8_t *luma, uint8_t *uv, uint32_t w,
                            uint32_t h, const double f[3]);

#endif /* HW_MISC_INGENIC_ISP_H */
