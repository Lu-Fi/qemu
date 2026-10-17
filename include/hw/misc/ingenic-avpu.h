/*
 * Ingenic AVPU (Allegro video encoder, T31)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_AVPU_H
#define HW_MISC_INGENIC_AVPU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_AVPU "ingenic-avpu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAvpuState, INGENIC_AVPU)

/* The VPU's 1 MiB also holds the TCSM and SRAM, at 0xc0000 and 0xf0000 */
#define INGENIC_AVPU_IOSIZE     0x10000
#define INGENIC_AVPU_REGS       0x8000  /* the registers: 0x8000-0x8fff */
#define INGENIC_AVPU_CORES      2       /* video, JPEG */

typedef struct IngenicAvpuJob IngenicAvpuJob;

struct IngenicAvpuState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[0x1000 / 4];
    bool clock[INGENIC_AVPU_CORES];
    IngenicAvpuJob *job[INGENIC_AVPU_CORES];   /* in flight */
    GHashTable *chans;          /* encoder state, by a channel's buffer */
};

/* An encode's settings, from a command list entry */
typedef struct IngenicAvpuPic {
    uint32_t width, height;
    int qp;
    bool intra;
    bool cabac;
    bool transform_8x8;
    int cabac_init_idc;
    int alpha_c0_div2, beta_div2;
    int chroma_qp_offset;
} IngenicAvpuPic;

/* What the hardware counts while it encodes, per 16x16 macroblock */
typedef struct IngenicAvpuStats {
    uint32_t intra_mbs, inter_mbs, skip_mbs;
} IngenicAvpuStats;

typedef struct IngenicAvpuH264 IngenicAvpuH264;

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
