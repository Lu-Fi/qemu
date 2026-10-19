/*
 * Ingenic AVPU (Allegro video encoder: the T31's and T40's EL150, the T41's
 * EL200)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_AVPU_H
#define HW_MISC_INGENIC_AVPU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_AVPU "ingenic-avpu"
#define TYPE_INGENIC_T41_AVPU "ingenic-t41-avpu"
OBJECT_DECLARE_TYPE(IngenicAvpuState, IngenicAvpuClass, INGENIC_AVPU)

/* The VPU's 1 MiB also holds the TCSM and SRAM, at 0xc0000 and 0xf0000 */
#define INGENIC_AVPU_IOSIZE     0x10000
#define INGENIC_AVPU_REGS_SIZE  0x3000  /* from the layout's base */
#define INGENIC_AVPU_CORES      2       /* video, JPEG */

typedef struct IngenicAvpuJob IngenicAvpuJob;
typedef struct IngenicAvpuLayout IngenicAvpuLayout;

struct IngenicAvpuClass {
    SysBusDeviceClass parent_class;
    const IngenicAvpuLayout *layout;
};

struct IngenicAvpuState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[INGENIC_AVPU_REGS_SIZE / 4];
    bool clock[INGENIC_AVPU_CORES];
    IngenicAvpuJob *job[INGENIC_AVPU_CORES];   /* in flight */
    GHashTable *chans;          /* encoder state, by a channel's buffer */
    uint32_t hw_id, date, rev;
};

/* An encode's settings, from a command list entry */
typedef struct IngenicAvpuPic {
    uint32_t width, height;
    int qp;
    bool hevc;
    bool intra;
    bool cabac;
    bool transform_8x8;
    bool scaling_list;          /* the default matrices, else flat */
    int cabac_init_idc;         /* H.265: cabac_init_flag */
    int alpha_c0_div2, beta_div2;
    int chroma_qp_offset;       /* H.264's, or H.265's Cb */
    int cr_qp_offset;
    uint32_t hdr_tail;          /* slice header bits the AVPU writes first */
    int hdr_tail_bits;
    bool auto_qp;
    /*
     * The QP of each 16x16 (H.264) or 32x32 (H.265) block, raster order;
     * NULL for the slice QP throughout
     */
    const uint8_t *block_qp;
} IngenicAvpuPic;

/* What the hardware counts while it encodes */
typedef struct IngenicAvpuStats {
    uint32_t intra_mbs, inter_mbs, skip_mbs;    /* H.264 macroblocks */
    uint32_t cus[3];            /* H.265 coding units of 8, 16 and 32 */
    uint32_t skip_8x8;          /* H.265 8x8 blocks of skipped units */
    uint32_t coded_4x4;         /* 4x4 coefficient groups not all zero */
    uint32_t bins;
} IngenicAvpuStats;

typedef struct IngenicAvpuH264 IngenicAvpuH264;
typedef struct IngenicAvpuHevc IngenicAvpuHevc;

/* Append @p to @out with emulation prevention bytes, as in a NAL unit */
void ingenic_avpu_escape(GByteArray *out, const uint8_t *p, size_t n);

/*
 * The H.265 slice data of one frame (NV12, @stride for both planes), to
 * follow libimp's slice segment header, emulation-prevented. A P picture
 * refers to the last picture @h encoded; @h then keeps this one. Returns
 * false if the frame could not be encoded.
 */
IngenicAvpuHevc *ingenic_avpu_hevc_new(void);
bool ingenic_avpu_hevc_encode(IngenicAvpuHevc *h, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st);
void ingenic_avpu_hevc_free(IngenicAvpuHevc *h);

/*
 * A baseline 4:2:0 JPEG of the NV12 frame, laid out and computed as the
 * JPEG core does it: @qt (luma, chroma, zigzag order) goes into the DQT
 * segments, @recip (natural order) quantizes, and APP0 carries @units,
 * @xdensity and @ydensity. Returns how many Huffman codes the scan holds.
 */
uint32_t ingenic_avpu_jpeg_encode(const uint8_t *y, uint32_t ystride,
                                  const uint8_t *uv, uint32_t cstride,
                                  uint32_t width, uint32_t height,
                                  const uint8_t qt[2][64],
                                  const uint16_t recip[2][64], uint8_t units,
                                  uint16_t xdensity, uint16_t ydensity,
                                  GByteArray *out);

#ifdef CONFIG_X264
IngenicAvpuH264 *ingenic_avpu_h264_new(const IngenicAvpuPic *pic);
bool ingenic_avpu_h264_matches(IngenicAvpuH264 *e, const IngenicAvpuPic *pic);
/*
 * Encode one frame (NV12, @stride for both planes); @out gets the slice
 * data that follows the slice header, emulation-prevented, as the AVPU
 * writes it. Returns false if the frame could not be encoded.
 */
bool ingenic_avpu_h264_encode(IngenicAvpuH264 *e, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st);
void ingenic_avpu_h264_free(IngenicAvpuH264 *e);
#else
static inline IngenicAvpuH264 *ingenic_avpu_h264_new(const IngenicAvpuPic *p)
{
    return NULL;
}

static inline bool ingenic_avpu_h264_matches(IngenicAvpuH264 *e,
                                             const IngenicAvpuPic *pic)
{
    return false;
}

static inline bool ingenic_avpu_h264_encode(IngenicAvpuH264 *e,
                                            const IngenicAvpuPic *pic,
                                            const uint8_t *y,
                                            const uint8_t *uv,
                                            uint32_t stride, GByteArray *out,
                                            IngenicAvpuStats *st)
{
    return false;
}

static inline void ingenic_avpu_h264_free(IngenicAvpuH264 *e)
{
}
#endif

#endif /* HW_MISC_INGENIC_AVPU_H */
