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
#include "hw/misc/ingenic-isp.h"

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
    IngenicIspState *isp;       /* whose channel 0 comes over IVDC */
};

/*
 * What an H.265 stream's parameter sets and slice header fix for its slice
 * data, beyond what the core never codes (tiles, wavefronts). libimp's AVPU
 * streams: CTBs of 32, transform trees one level deep, transform skip,
 * cu_qp_delta per CTB, five merge candidates, no SAO, no sign hiding.
 */
typedef struct IngenicAvpuHevcSyntax {
    int log2_ctb;               /* CtbLog2SizeY, 5 or 6 */
    int tu_depth;               /* max_transform_hierarchy_depth_intra */
    bool transform_skip;        /* transform_skip_enabled_flag */
    int qp_delta_depth;         /* diff_cu_qp_delta_depth, -1 for none */
    bool sao_luma, sao_chroma;  /* slice_sao_luma_flag, slice_sao_chroma_flag */
    bool sign_hiding;           /* sign_data_hiding_enabled_flag */
    int merge_cands;            /* MaxNumMergeCand */
} IngenicAvpuHevcSyntax;

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
    const IngenicAvpuHevcSyntax *syntax;    /* H.265; NULL for the AVPU's */
    /* H.264 Intra and Inter Y 8x8 lists, zigzag; NULL for scaling_list's */
    const uint8_t *scale8[2];
    bool raw;                   /* the RBSP, no emulation prevention */
    bool no_deblock;            /* H.264: the reference left unfiltered */
    int alpha_c0_div2, beta_div2;
    int chroma_qp_offset;       /* H.264's, or H.265's Cb */
    int cr_qp_offset;
    uint32_t hdr_tail;          /* slice header bits the AVPU writes first */
    int hdr_tail_bits;
    bool auto_qp;
    /*
     * The QP of each 16x16 (H.264) block or CTB (H.265), raster order; NULL
     * for the slice QP throughout
     */
    const uint8_t *block_qp;
    uint8_t lambda[52];         /* H.264 mode decisions' lambda, by QP */
} IngenicAvpuPic;

/* What the hardware counts while it encodes */
typedef struct IngenicAvpuStats {
    uint32_t intra_mbs, inter_mbs, skip_mbs;    /* H.264 macroblocks */
    uint32_t i16_mbs, i8_mbs;   /* of intra_mbs, 16x16 and 8x8 */
    /*
     * H.265 coding units of 8, 16 and 32; H.264 the 8x8 blocks of
     * macroblocks split smaller, then the macroblocks left whole
     */
    uint32_t cus[3];
    uint32_t skip_8x8;          /* H.265 8x8 blocks of skipped units */
    uint32_t coded_4x4;         /* H.265 4x4 coefficient groups not all 0 */
    uint32_t bins;              /* CABAC bins; with CAVLC, bits */
} IngenicAvpuStats;

typedef struct IngenicAvpuH264 IngenicAvpuH264;
typedef struct IngenicAvpuHevc IngenicAvpuHevc;

/* Append @p to @out with emulation prevention bytes, as in a NAL unit */
void ingenic_avpu_escape(GByteArray *out, const uint8_t *p, size_t n);

/*
 * The H.265 slice data of one frame (NV12, @stride for both planes), to
 * follow libimp's slice segment header, emulation-prevented unless @pic is
 * raw. A P picture refers to the last picture @h encoded, or the one put in
 * ingenic_avpu_hevc_ref(); @h then keeps this one. Returns false if the
 * frame could not be encoded.
 */
IngenicAvpuHevc *ingenic_avpu_hevc_new(void);
bool ingenic_avpu_hevc_encode(IngenicAvpuHevc *h, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st);
/*
 * The picture a P picture of @width x @height refers to: Y, then Cb and Cr
 * at half size. A caller may fill it in for the next encode; an encode
 * leaves its own picture there, deblocked.
 */
uint8_t *ingenic_avpu_hevc_ref(IngenicAvpuHevc *h, uint32_t width,
                               uint32_t height);
void ingenic_avpu_hevc_free(IngenicAvpuHevc *h);

/*
 * A baseline 4:2:0 JPEG of the NV12 frame, laid out and computed as the
 * JPEG core does it: @qt (luma, chroma, zigzag order) goes into the DQT
 * segments, @recip (natural order) quantizes, APP0 carries @units,
 * @xdensity and @ydensity, and a @restart interval of MCUs, if not 0, DRI
 * and the RSTn markers. Returns how many Huffman codes the scan holds.
 */
uint32_t ingenic_avpu_jpeg_encode(const uint8_t *y, uint32_t ystride,
                                  const uint8_t *uv, uint32_t cstride,
                                  uint32_t width, uint32_t height,
                                  const uint8_t qt[2][64],
                                  const uint16_t recip[2][64], uint8_t units,
                                  uint16_t xdensity, uint16_t ydensity,
                                  uint16_t restart, GByteArray *out);

/*
 * The same picture in the file a T32's JPEG engine writes: SOI, the luma
 * DQT, the chroma one twice (both as table 1), DRI for a @restart
 * interval, SOF0, the four tables, SOS, the scan and EOI, with no APP0
 * and no padding. Its own DCT's coefficient c quantizes to (c * mul +
 * 32768) >> 16, @mul in natural order.
 */
uint32_t ingenic_avpu_jpeg_encode_bare(const uint8_t *y, uint32_t ystride,
                                       const uint8_t *uv, uint32_t cstride,
                                       uint32_t width, uint32_t height,
                                       const uint8_t qt[2][64],
                                       const uint32_t mul[2][64],
                                       uint16_t restart, GByteArray *out);

/* A Huffman code by symbol: the code, and its length (0 for none) */
typedef struct IngenicAvpuJpegHuff {
    uint16_t code[256];
    uint8_t size[256];
} IngenicAvpuJpegHuff;

/* The Annex K codes: luma DC, luma AC, chroma DC, chroma AC */
void ingenic_avpu_jpeg_annex_k(IngenicAvpuJpegHuff huff[4]);

/*
 * The scan alone of that JPEG, stuffed and filled with ones, without
 * markers but a @restart interval's RSTn: a
 * coefficient c (8 times the DCT's) quantizes to (c * mul + half) >>
 * shift, natural order, luma then chroma, and codes with @huff. Returns how
 * many Huffman codes it holds.
 */
uint32_t ingenic_avpu_jpeg_scan(const uint8_t *y, uint32_t ystride,
                                const uint8_t *uv, uint32_t cstride,
                                uint32_t width, uint32_t height,
                                const uint32_t mul[2][64],
                                const uint8_t shift[2][64],
                                const IngenicAvpuJpegHuff huff[4],
                                uint16_t restart, GByteArray *out);

/*
 * The H.264 slice data of one frame (NV12, @stride for both planes), to
 * follow libimp's slice header, emulation-prevented unless @pic is raw. A
 * P picture refers to the last picture @h encoded; @h then keeps this one.
 */
IngenicAvpuH264 *ingenic_avpu_h264_new(const IngenicAvpuPic *pic);
bool ingenic_avpu_h264_matches(IngenicAvpuH264 *h, const IngenicAvpuPic *pic);
bool ingenic_avpu_h264_encode(IngenicAvpuH264 *h, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st);
/*
 * The picture a P picture refers to: Y, then Cb and Cr at half size, whole
 * macroblocks. A caller may fill them in for the next encode; an encode
 * leaves its own picture there, deblocked, in new planes.
 */
uint8_t *const *ingenic_avpu_h264_ref(IngenicAvpuH264 *h);
void ingenic_avpu_h264_free(IngenicAvpuH264 *h);

#endif /* HW_MISC_INGENIC_AVPU_H */
