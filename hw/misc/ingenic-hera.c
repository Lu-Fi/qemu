/*
 * Ingenic HERA: the video encoder of the T32
 *
 * libimp (an x264 and an x265) sets up each picture as a command list in
 * memory, pairs of words (value, then VLD bit 31 | TERM bit 30 | a checksum
 * in 29:20 | register offset), and the radix driver hands its address to
 * EMC 0x90068, then writes ACM_CTRL with bit 0. The model replays the list
 * into its registers, modules 0x8000 apart; the list ends with the front
 * end's run (0x20004 = 1) and a TERM pair, and the picture is then encoded
 * by the AVPU model's H.264 or H.265 core, 0x20180 bit 16 saying which.
 *
 * H.264, as libimp's headers announce it: CABAC (cabac_init_idc 0), one
 * reference, a picture of 0x68008 bits 23:16 by 31:24 macroblocks, the QP
 * in 0x68018 bits 13:8, I or P in bits 3:0 (1, 2), and High's 8x8
 * transform with the default 8x8 lists where bit 5 says, flat lists
 * otherwise; deblocking off where 0x50004 bit 17 says, its offsets plus
 * 12 in bits 4:0 and 12:8; the chroma QP offset in 0x2001c bits 4:0.
 *
 * H.265: CTBs of 64, no transform skip, flat lists, one reference; the
 * picture's size in 0x68004 (width in bits 31:16), the QP in 0x60004
 * bits 6:1, and in 0x68000 I or P in bits 3:2 (2, 1), cu_qp_delta (4) and
 * its depth (19:18), sign hiding (14), SAO (15) with its slice flags (16
 * luma, 17 chroma), MaxNumMergeCand (24:22) and the transform tree's depth
 * (31); the chroma QP offset in 0x48004 bits 27:20, NV21 where 0x20000
 * bit 0 says.
 *
 * Both: the source at 0x2000c and 0x20010, strides 0x20014 bits 15:0 and
 * 31:16; the slice data an RBSP ended by its stop bit, libimp writing the
 * header in front and the emulation prevention bytes (0x68018 bit 7 on
 * H.264, 0x68000 bit 28 on H.265, would have the core add them). The
 * reference is read from 0x28014 and 0x28018, the reconstruction written
 * to 0x10008 and 0x1000c; with buffer share (bit 31 of 0x28004 and
 * 0x10004) each plane is a ring, base and end at 0x28020-0x2802c and
 * 0x10030-0x1003c, and each P picture's reconstruction starts before the
 * last, over the reference: the model reads the whole reference first.
 * Silicon compresses both; the model keeps NV12 rows, the width rounded up
 * to 64 apart (16 without a ring), which fit and which nothing but the
 * core reads. The T33's lists give the reconstruction address of an intra
 * picture only, and the reference address of the picture after it only,
 * their compressed ring alone saying where otherwise: the model then
 * keeps each ring's last picture itself, by its base (0x10030, 0x28020).
 * A source address with bit 31 set comes over IVDC, straight from the ISP,
 * which draws it (the "isp" link).
 *
 * The stream goes to EMC 0x90008. With multi-section on (0x9005c) it goes
 * in halves of 0x90064 KiB, alternately there and at 0x9001c: once a half
 * is full the core holds BSFULL (0x2c bit 1) with the bytes sent so far in
 * 0x44 until libimp has copied it and writes 1 to 0x90060. At the end it
 * holds ENDF (0x2c bit 0), the whole length in 0x44; libimp takes a length
 * of 0 for a failure. Its interrupt is up while a bit of 0x2c is enabled in
 * 0x28; 0x30 clears them. Writing 0x0 with bit 31 resets it, which the 2.2
 * driver polls 0x0 back to 0 for.
 *
 * Not modelled: motion search (inter blocks do not move), the QP
 * generator (one QP per picture), the CABAC start states the list loads,
 * and the statistics libimp's rate control reads back, which stay 0: a
 * still scene to it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "block/thread-pool.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-hera.h"
#include "hw/misc/ingenic-avpu.h"
#include "hw/core/qdev-properties.h"

/* CFGC, the scheduler */
#define CFGC_GLB_CTRL       0x00000
#define  GLB_RESET          BIT(31)
#define CFGC_ACM_CTRL       0x00008
#define  ACM_RUN            BIT(0)
#define CFGC_INTR_EN        0x00028
#define CFGC_INTR_STAT      0x0002c
#define CFGC_INTR_CLR       0x00030
#define  INTR_ENDF          BIT(0)
#define  INTR_BSFULL        BIT(1)
#define  INTR_AMCERR        BIT(2)
#define CFGC_BSLEN          0x00044

#define CL_VLD              BIT(31)
#define CL_TERM             BIT(30)
#define CL_REG              0xffffcu
#define CL_MAX              0x4000      /* pairs; libimp's hold ~1200 */

#define ODMA_CTRL           0x10004
#define  BUF_SHARE          BIT(31)
#define ODMA_RECON_Y        0x10008
#define ODMA_RECON_C        0x1000c
#define ODMA_RING           0x10030     /* Y base, C base, Y end, C end */
#define EFE_CTRL            0x20000
#define  EFE_NV21           BIT(0)      /* H.265 */
#define EFE_STAT            0x20004
#define  EFE_RUN            BIT(0)
#define EFE_RAWY            0x2000c
#define EFE_RAWC            0x20010
#define EFE_RAW_STRD        0x20014
#define EFE_FRM_QP          0x2001c
#define EFE_EXTRA_CFG0      0x20180
#define  EFE_H264           BIT(16)
#define JRFD_CTRL           0x28004
#define JRFD_REF_Y          0x28014
#define JRFD_REF_C          0x28018
#define JRFD_RING           0x28020     /* as ODMA_RING */
#define DT_CFG              0x48004
#define DBLK_CFG            0x50004
#define  DBLK_OFF           BIT(17)
#define BC_CFG1             0x60004
#define SDE_CFG0            0x68000     /* H.265 */
#define SDE_SIZE            0x68004     /* H.265 */
#define SDE_SL_GEOM         0x68008     /* H.264 */
#define SDE_CFG1            0x68018     /* H.264 */
#define EMC_BS0             0x90008
#define EMC_BS1             0x9001c
#define EMC_BSFULL_EN       0x9005c
#define EMC_BSFULL_STAT     0x90060
#define  BSFULL_GO          1
#define EMC_BSFULL_SIZE     0x90064
#define EMC_ADDR_CHN        0x90068

#define IVDC_ADDR           BIT(31)     /* a source address over IVDC */

#define FRAME_MAX           4096        /* samples a side */

#define R(off)              (s->regs[(off) / 4])

/* Rows of a plane from @addr, wrapping from @end back to @start if a ring */
typedef struct HeraPlane {
    uint32_t addr, start, end, stride;
} HeraPlane;

struct IngenicHeraJob {
    IngenicHeraState *s;        /* NULL once the device has reset */
    bool h264;
    IngenicAvpuPic pic;
    IngenicAvpuHevcSyntax syn;
    IngenicAvpuH264 *avc;
    IngenicAvpuHevc *hevc;
    uint32_t w, h;
    uint8_t *y, *uv;            /* the source, @w apart */
    uint8_t *ref_y, *ref_uv;    /* P: the reference, as the ring holds it */
    HeraPlane recon_y, recon_c;
    GByteArray *data;
    IngenicAvpuStats st;
    bool ok;
    uint32_t sent, half;        /* the stream out so far, the next half */
    bool kept;                  /* no reconstruction address: refs */
    uint32_t key;               /* refs' key, the channel's ring */
};

/* A channel's last picture, NV12 rows @w apart */
typedef struct HeraRef {
    uint32_t w, h;
    uint8_t *y, *uv;
} HeraRef;

static void hera_ref_free(gpointer p)
{
    HeraRef *r = p;

    g_free(r->y);
    g_free(r->uv);
    g_free(r);
}

/* The JVT's Default_8x8_Intra and _Inter, zigzag */
static const uint8_t hera_scale8[2][64] = {
    {
         6, 10, 10, 13, 11, 13, 16, 16, 16, 16, 18, 18, 18, 18, 18, 23,
        23, 23, 23, 23, 23, 25, 25, 25, 25, 25, 25, 25, 27, 27, 27, 27,
        27, 27, 27, 27, 29, 29, 29, 29, 29, 29, 29, 31, 31, 31, 31, 31,
        31, 33, 33, 33, 33, 33, 36, 36, 36, 36, 38, 38, 38, 40, 40, 42,
    }, {
         9, 13, 13, 15, 13, 15, 17, 17, 17, 17, 19, 19, 19, 19, 19, 21,
        21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 24, 24, 24, 24,
        24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 27, 27, 27, 27, 27,
        27, 28, 28, 28, 28, 28, 30, 30, 30, 30, 32, 32, 32, 33, 33, 35,
    },
};

/* The mode decisions' lambda by QP, the AVPU model's default */
static const uint8_t hera_lambda[52] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 10, 11, 12, 14, 16, 17,
    20, 22, 25, 28, 32, 35, 40,
};

static void hera_update(IngenicHeraState *s)
{
    qemu_set_irq(s->irq, !!(R(CFGC_INTR_STAT) & R(CFGC_INTR_EN)));
}

/* Row @r's address, and the bytes it holds before the ring wraps */
static uint32_t hera_row(const HeraPlane *p, uint32_t r, uint32_t *n)
{
    uint32_t off = r * p->stride;

    if (p->end > p->start && p->addr >= p->start && p->addr < p->end) {
        uint32_t a = p->start + (p->addr - p->start + off) %
                                (p->end - p->start);

        *n = MIN(*n, p->end - a);
        return a;
    }
    return p->addr + off;
}

static void hera_plane_io(const HeraPlane *p, uint8_t *buf, uint32_t w,
                          uint32_t rows, bool write)
{
    for (uint32_t r = 0; r < rows; r++) {
        uint32_t n = w, a = hera_row(p, r, &n);

        address_space_rw(&address_space_memory, a, MEMTXATTRS_UNSPECIFIED,
                         buf + r * w, n, write);
        if (n < w) {
            address_space_rw(&address_space_memory, p->start,
                             MEMTXATTRS_UNSPECIFIED, buf + r * w + n, w - n,
                             write);
        }
    }
}

/* A plane's rows of @stride from @addr, in the ring @ring sets if shared */
static HeraPlane hera_plane(IngenicHeraState *s, uint32_t addr, hwaddr ring,
                            int c, bool share, uint32_t stride)
{
    return (HeraPlane) {
        addr, share ? R(ring + 4 * c) : 0, share ? R(ring + 8 + 4 * c) : 0,
        stride,
    };
}

static void hera_job_free(IngenicHeraJob *j)
{
    ingenic_avpu_h264_free(j->avc);
    ingenic_avpu_hevc_free(j->hevc);
    g_free(j->y);
    g_free(j->uv);
    g_free(j->ref_y);
    g_free(j->ref_uv);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

static int hera_work(void *opaque)
{
    IngenicHeraJob *j = opaque;
    size_t cn = (size_t)j->w * j->h / 4;

    if (j->h264) {
        uint8_t *const *ref = ingenic_avpu_h264_ref(j->avc);

        if (j->ref_y) {
            memcpy(ref[0], j->ref_y, (size_t)j->w * j->h);
            for (size_t i = 0; i < cn; i++) {
                ref[1][i] = j->ref_uv[2 * i];
                ref[2][i] = j->ref_uv[2 * i + 1];
            }
        }
        j->ok = ingenic_avpu_h264_encode(j->avc, &j->pic, j->y, j->uv, j->w,
                                         j->data, &j->st);
        if (j->ok) {
            /* the reconstruction, back to NV12 */
            ref = ingenic_avpu_h264_ref(j->avc);
            memcpy(j->y, ref[0], (size_t)j->w * j->h);
            for (size_t i = 0; i < cn; i++) {
                j->uv[2 * i] = ref[1][i];
                j->uv[2 * i + 1] = ref[2][i];
            }
        }
        return 0;
    }

    if (j->ref_y) {
        uint8_t *ref = ingenic_avpu_hevc_ref(j->hevc, j->w, j->h);

        memcpy(ref, j->ref_y, (size_t)j->w * j->h);
        for (size_t i = 0; i < cn; i++) {
            ref[j->w * j->h + i] = j->ref_uv[2 * i];
            ref[j->w * j->h + cn + i] = j->ref_uv[2 * i + 1];
        }
    }
    j->ok = ingenic_avpu_hevc_encode(j->hevc, &j->pic, j->y, j->uv, j->w,
                                     j->data, &j->st);
    if (j->ok) {
        const uint8_t *ref = ingenic_avpu_hevc_ref(j->hevc, j->w, j->h);

        memcpy(j->y, ref, (size_t)j->w * j->h);
        for (size_t i = 0; i < cn; i++) {
            j->uv[2 * i] = ref[j->w * j->h + i];
            j->uv[2 * i + 1] = ref[j->w * j->h + cn + i];
        }
    }
    return 0;
}

/*
 * The stream out from where it stopped: the rest, or with multi-section on
 * the next half, held at BSFULL if more follows. The job ends at ENDF.
 */
static void hera_send(IngenicHeraState *s)
{
    IngenicHeraJob *j = s->job;
    uint32_t size = R(EMC_BSFULL_SIZE) * KiB, n = j->data->len - j->sent;
    bool sections = R(EMC_BSFULL_EN) && size;
    hwaddr bs = R(EMC_BS0);

    if (sections) {
        n = MIN(n, size);
        bs = j->half & 1 ? R(EMC_BS1) : R(EMC_BS0);
    }
    address_space_write(&address_space_memory, bs, MEMTXATTRS_UNSPECIFIED,
                        j->data->data + j->sent, n);
    j->sent += n;
    j->half++;
    R(CFGC_BSLEN) = j->sent;
    if (sections && j->sent < j->data->len) {
        R(CFGC_INTR_STAT) |= INTR_BSFULL;
        hera_update(s);
        return;
    }
    R(EFE_STAT) &= ~EFE_RUN;
    R(CFGC_INTR_STAT) |= INTR_ENDF;
    s->job = NULL;
    hera_update(s);
    hera_job_free(j);
}

/*
 * Back to the device, for the next picture of its size: the coder keeps
 * buffers and nothing a picture takes from the last
 */
static void hera_keep_coder(IngenicHeraState *s, IngenicHeraJob *j)
{
    int i = 0;

    while (i < INGENIC_HERA_CODERS - 1 && s->coders[i]) {
        i++;
    }
    ingenic_avpu_h264_free(s->coders[i]);
    s->coders[i] = j->avc;
    j->avc = NULL;
}

static void hera_done(void *opaque, int ret)
{
    IngenicHeraJob *j = opaque;
    IngenicHeraState *s = j->s;

    if (!s) {
        hera_job_free(j);
        return;
    }
    if (j->avc) {
        hera_keep_coder(s, j);
    }
    if (!j->ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hera: a %ux%u picture could "
                      "not be encoded\n", j->w, j->h);
        g_byte_array_set_size(j->data, 0);
    } else if (j->kept) {
        HeraRef *r = g_new(HeraRef, 1);

        r->w = j->w;
        r->h = j->h;
        r->y = g_steal_pointer(&j->y);
        r->uv = g_steal_pointer(&j->uv);
        g_hash_table_replace(s->refs, GUINT_TO_POINTER(j->key), r);
    } else {
        hera_plane_io(&j->recon_y, j->y, j->w, j->h, true);
        hera_plane_io(&j->recon_c, j->uv, j->w, j->h / 2, true);
    }
    hera_send(s);
}

/* The H.264 settings of the list, or false for a picture the core cannot */
static bool hera_h264(IngenicHeraState *s, IngenicHeraJob *j)
{
    uint32_t geom = R(SDE_SL_GEOM), cfg = R(SDE_CFG1), dblk = R(DBLK_CFG);
    int type = extract32(cfg, 0, 4);

    j->w = extract32(geom, 16, 8) * 16;
    j->h = extract32(geom, 24, 8) * 16;
    j->pic.qp = MIN(extract32(cfg, 8, 8), 51);
    j->pic.intra = type == 1;
    j->pic.cabac = true;
    if (extract32(cfg, 5, 1)) {
        j->pic.transform_8x8 = true;
        j->pic.scale8[0] = hera_scale8[0];
        j->pic.scale8[1] = hera_scale8[1];
    }
    j->pic.raw = !extract32(cfg, 7, 1);
    j->pic.no_deblock = dblk & DBLK_OFF;
    j->pic.alpha_c0_div2 = ((int)extract32(dblk, 0, 5) - 12) / 2;
    j->pic.beta_div2 = ((int)extract32(dblk, 8, 5) - 12) / 2;
    j->pic.chroma_qp_offset = sextract32(R(EFE_FRM_QP), 0, 5);
    j->pic.cr_qp_offset = j->pic.chroma_qp_offset;
    memcpy(j->pic.lambda, hera_lambda, sizeof(hera_lambda));
    return type == 1 || type == 2;
}

static bool hera_h265(IngenicHeraState *s, IngenicHeraJob *j)
{
    uint32_t cfg = R(SDE_CFG0), size = R(SDE_SIZE);
    bool sao = extract32(cfg, 15, 1);
    int type = extract32(cfg, 2, 2);

    j->w = size >> 16;
    j->h = size & 0xffff;
    j->syn = (IngenicAvpuHevcSyntax) {
        .log2_ctb = 6,
        .tu_depth = extract32(cfg, 31, 1),
        .qp_delta_depth = extract32(cfg, 4, 1) ? extract32(cfg, 18, 2) : -1,
        .sao_luma = sao && extract32(cfg, 16, 1),
        .sao_chroma = sao && extract32(cfg, 17, 1),
        .sign_hiding = extract32(cfg, 14, 1),
        .merge_cands = MIN(MAX(extract32(cfg, 22, 3), 1), 5),
    };
    j->pic.qp = MIN(extract32(R(BC_CFG1), 1, 6), 51);
    j->pic.intra = type == 2;
    j->pic.cabac = true;
    j->pic.syntax = &j->syn;
    j->pic.raw = !extract32(cfg, 28, 1);
    j->pic.chroma_qp_offset = sextract32(R(DT_CFG), 20, 8);
    j->pic.cr_qp_offset = j->pic.chroma_qp_offset;
    return type == 1 || type == 2;
}

static void hera_encode(IngenicHeraState *s)
{
    IngenicHeraJob *j = g_new0(IngenicHeraJob, 1);
    uint32_t strd = R(EFE_RAW_STRD);
    bool ok, ring, ref_kept;
    uint32_t stride;

    j->s = s;
    j->h264 = R(EFE_EXTRA_CFG0) & EFE_H264;
    j->data = g_byte_array_new();
    j->pic.hevc = !j->h264;
    ok = j->h264 ? hera_h264(s, j) : hera_h265(s, j);
    j->pic.width = j->w;
    j->pic.height = j->h;
    s->job = j;
    if (!ok || !j->w || !j->h || j->w % 8 || j->h % 8 ||
        j->w > FRAME_MAX || j->h > FRAME_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hera: a %ux%u %s picture "
                      "the core cannot code\n", j->w, j->h,
                      j->h264 ? "H.264" : "H.265");
        hera_send(s);
        return;
    }
    if (j->h264) {
        for (int i = 0; i < INGENIC_HERA_CODERS && !j->avc; i++) {
            if (s->coders[i] &&
                ingenic_avpu_h264_matches(s->coders[i], &j->pic)) {
                j->avc = s->coders[i];
                s->coders[i] = NULL;
            }
        }
        if (!j->avc) {
            j->avc = ingenic_avpu_h264_new(&j->pic);
        }
    } else {
        j->hevc = ingenic_avpu_hevc_new();
    }

    /*
     * The source; rows past the picture's buffers read 0. Over IVDC the
     * ISP draws it.
     */
    j->y = g_malloc0((size_t)j->w * j->h);
    j->uv = g_malloc0((size_t)j->w * j->h / 2);
    if (R(EFE_RAWY) & IVDC_ADDR) {
        g_autofree uint8_t *nv12 = g_malloc((size_t)j->w * j->h * 3 / 2);

        if (s->isp) {
            ingenic_isp_render(s->isp, j->w, j->h, nv12);
            memcpy(j->y, nv12, (size_t)j->w * j->h);
            memcpy(j->uv, nv12 + j->w * j->h, (size_t)j->w * j->h / 2);
        }
    } else {
        hera_plane_io(&(HeraPlane) { R(EFE_RAWY), 0, 0,
                                     extract32(strd, 0, 16) ?: j->w },
                      j->y, j->w, j->h, false);
        hera_plane_io(&(HeraPlane) { R(EFE_RAWC), 0, 0,
                                     extract32(strd, 16, 16) ?: j->w },
                      j->uv, j->w, j->h / 2, false);
    }
    if (!j->h264 && R(EFE_CTRL) & EFE_NV21) {
        for (size_t i = 0; i < (size_t)j->w * j->h / 2; i += 2) {
            uint8_t v = j->uv[i];

            j->uv[i] = j->uv[i + 1];
            j->uv[i + 1] = v;
        }
    }

    /*
     * A list without a reconstruction or reference address (the T33's but
     * around an intra picture: its compressed ring alone says where) leaves
     * that picture to the model
     */
    j->kept = !R(ODMA_RECON_Y) && !R(ODMA_RECON_C);
    j->key = R(ODMA_RING);
    ref_kept = !R(JRFD_REF_Y) && !R(JRFD_REF_C);
    if (!j->pic.intra && ref_kept) {
        HeraRef *r = g_hash_table_lookup(s->refs,
                                         GUINT_TO_POINTER(R(JRFD_RING)));

        j->ref_y = g_malloc0((size_t)j->w * j->h);
        j->ref_uv = g_malloc0((size_t)j->w * j->h / 2);
        if (r && r->w == j->w && r->h == j->h) {
            memcpy(j->ref_y, r->y, (size_t)j->w * j->h);
            memcpy(j->ref_uv, r->uv, (size_t)j->w * j->h / 2);
        }
    }

    /* all of the reference before any reconstruction overwrites it */
    if (!j->pic.intra && !ref_kept) {
        HeraPlane ry, rc;

        ring = R(JRFD_CTRL) & BUF_SHARE;
        stride = ROUND_UP(j->w, ring ? 64 : 16);
        ry = hera_plane(s, R(JRFD_REF_Y), JRFD_RING, 0, ring, stride);
        rc = hera_plane(s, R(JRFD_REF_C), JRFD_RING, 1, ring, stride);
        j->ref_y = g_malloc((size_t)j->w * j->h);
        j->ref_uv = g_malloc((size_t)j->w * j->h / 2);
        hera_plane_io(&ry, j->ref_y, j->w, j->h, false);
        hera_plane_io(&rc, j->ref_uv, j->w, j->h / 2, false);
    }
    ring = R(ODMA_CTRL) & BUF_SHARE;
    stride = ROUND_UP(j->w, ring ? 64 : 16);
    j->recon_y = hera_plane(s, R(ODMA_RECON_Y), ODMA_RING, 0, ring, stride);
    j->recon_c = hera_plane(s, R(ODMA_RECON_C), ODMA_RING, 1, ring, stride);
    thread_pool_submit_aio(hera_work, j, hera_done, j);
}

/* Replay a command list into the registers; the last pair ends it */
static void hera_run(IngenicHeraState *s, uint32_t list)
{
    uint32_t pair[2], cmd = 0, addr = list;
    int n;

    if (s->job) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hera: started while busy\n");
        return;
    }
    for (n = 0; n < CL_MAX; n++, addr += 8) {
        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, pair, sizeof(pair)) !=
            MEMTX_OK) {
            break;
        }
        cmd = le32_to_cpu(pair[1]);
        if (cmd & CL_VLD && (cmd & CL_REG) < INGENIC_HERA_IOSIZE) {
            R(cmd & CL_REG) = le32_to_cpu(pair[0]);
        }
        if (cmd & CL_TERM) {
            break;
        }
    }
    if (n == CL_MAX || !(cmd & CL_TERM)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-hera: command list at "
                      "0x%08x has no end\n", list);
        R(CFGC_INTR_STAT) |= INTR_AMCERR;
        hera_update(s);
        return;
    }
    if (R(EFE_STAT) & EFE_RUN) {
        hera_encode(s);
    }
}

static void hera_reset(IngenicHeraState *s)
{
    if (s->job) {
        s->job->s = NULL;           /* its completion just frees it */
        if (s->job->sent) {         /* already done: it waits for a half */
            hera_job_free(s->job);
        }
        s->job = NULL;
    }
    memset(s->regs, 0, INGENIC_HERA_IOSIZE);
}

static uint64_t ingenic_hera_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicHeraState *s = INGENIC_HERA(opaque);

    return R(addr);
}

static void ingenic_hera_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    IngenicHeraState *s = INGENIC_HERA(opaque);
    uint32_t v = value;

    switch (addr) {
    case CFGC_GLB_CTRL:
        if (v & GLB_RESET) {
            hera_reset(s);
            hera_update(s);
            return;
        }
        break;
    case CFGC_ACM_CTRL:
        R(addr) = v;
        if (v & ACM_RUN) {
            hera_run(s, R(EMC_ADDR_CHN) & ~0xffu);
        }
        return;
    case CFGC_INTR_EN:
        R(addr) = v;
        hera_update(s);
        return;
    case CFGC_INTR_STAT:
        return;
    case CFGC_INTR_CLR:
        R(CFGC_INTR_STAT) &= ~v;
        hera_update(s);
        return;
    case EMC_BSFULL_STAT:
        R(addr) = v;
        if (v == BSFULL_GO && s->job && s->job->sent) {
            hera_send(s);
        }
        return;
    }
    R(addr) = v;
}

static const MemoryRegionOps ingenic_hera_ops = {
    .read = ingenic_hera_read,
    .write = ingenic_hera_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_hera_reset_hold(Object *obj, ResetType type)
{
    IngenicHeraState *s = INGENIC_HERA(obj);

    hera_reset(s);
    g_hash_table_remove_all(s->refs);
}

static void ingenic_hera_reset_exit(Object *obj, ResetType type)
{
    hera_update(INGENIC_HERA(obj));
}

static void ingenic_hera_init(Object *obj)
{
    IngenicHeraState *s = INGENIC_HERA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->regs = g_malloc0(INGENIC_HERA_IOSIZE);
    s->refs = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL,
                                    hera_ref_free);
    memory_region_init_io(&s->iomem, obj, &ingenic_hera_ops, s,
                          TYPE_INGENIC_HERA, INGENIC_HERA_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_hera_finalize(Object *obj)
{
    IngenicHeraState *s = INGENIC_HERA(obj);

    for (int i = 0; i < INGENIC_HERA_CODERS; i++) {
        ingenic_avpu_h264_free(s->coders[i]);
    }
    g_free(s->regs);
    g_hash_table_destroy(s->refs);
}

/* An encode in flight is host state */
static const VMStateDescription vmstate_ingenic_hera = {
    .name = TYPE_INGENIC_HERA,
    .unmigratable = 1,
};

static const Property ingenic_hera_props[] = {
    DEFINE_PROP_LINK("isp", IngenicHeraState, isp, TYPE_INGENIC_ISP,
                     IngenicIspState *),
};

static void ingenic_hera_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic HERA video encoder";
    device_class_set_props(dc, ingenic_hera_props);
    dc->vmsd = &vmstate_ingenic_hera;
    rc->phases.hold = ingenic_hera_reset_hold;
    rc->phases.exit = ingenic_hera_reset_exit;
}

static const TypeInfo ingenic_hera_types[] = {
    {
        .name = TYPE_INGENIC_HERA,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicHeraState),
        .instance_init = ingenic_hera_init,
        .instance_finalize = ingenic_hera_finalize,
        .class_init = ingenic_hera_class_init,
    },
};

DEFINE_TYPES(ingenic_hera_types)
