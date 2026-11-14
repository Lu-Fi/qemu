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
#include "migration/qemu-file-types.h"
#include "qemu/aio-wait.h"
#include "qemu/error-report.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-hera.h"
#include "hw/misc/ingenic-avpu.h"
#include "hw/misc/ingenic-enc-tap.h"
#include "hw/core/qdev-properties.h"
#ifdef CONFIG_AVX2_OPT
#include <immintrin.h>
#include "host/cpuinfo.h"
#elif defined(__ARM_NEON)
#include <arm_neon.h>
#endif

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

/* What libimp's rate control reads back of an H.265 picture: hera_h265_stats */
typedef struct HeraStats {
    uint32_t units;             /* of 16x16 */
    uint32_t n_g32, n_g512;     /* units whose gradient is over 32, 512 */
    uint32_t n_r2;              /* units whose range of luma is over 2 */
    uint32_t n_flat;            /* units whose luma is one value */
    uint64_t grad;              /* the sum of the gradients */
    uint64_t sad_src;           /* the source's change from the reference */
    uint32_t n_sad128;          /* P: units that differ from the reference */
    uint64_t sad;               /* P: the sum of the differences */
} HeraStats;

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
    HeraStats hs;
    bool ok;
    uint32_t sent, half;        /* the stream out so far, the next half */
    bool kept;                  /* no reconstruction address: refs */
    uint32_t key;               /* refs' key, the channel's ring */
    uint8_t *cap;               /* the source, captured: see "capture" */
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
    g_free(j->cap);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

#ifdef CONFIG_AVX2_OPT
__attribute__((target("avx2")))
static size_t hera_deinterleave_avx2(uint8_t *u, uint8_t *v, const uint8_t *uv,
                                     size_t n)
{
    const __m256i even = _mm256_set1_epi16(0xff);
    size_t i = 0;

    for (; i + 32 <= n; i += 32) {
        __m256i a = _mm256_loadu_si256((const __m256i *)(uv + 2 * i));
        __m256i b = _mm256_loadu_si256((const __m256i *)(uv + 2 * i + 32));
        __m256i lo = _mm256_packus_epi16(_mm256_and_si256(a, even),
                                         _mm256_and_si256(b, even));
        __m256i hi = _mm256_packus_epi16(_mm256_srli_epi16(a, 8),
                                         _mm256_srli_epi16(b, 8));

        _mm256_storeu_si256((__m256i *)(u + i),
                            _mm256_permute4x64_epi64(lo, 0xd8));
        _mm256_storeu_si256((__m256i *)(v + i),
                            _mm256_permute4x64_epi64(hi, 0xd8));
    }
    return i;
}

__attribute__((target("avx2")))
static size_t hera_interleave_avx2(uint8_t *uv, const uint8_t *u,
                                   const uint8_t *v, size_t n)
{
    size_t i = 0;

    for (; i + 32 <= n; i += 32) {
        __m256i a = _mm256_loadu_si256((const __m256i *)(u + i));
        __m256i b = _mm256_loadu_si256((const __m256i *)(v + i));
        __m256i lo = _mm256_unpacklo_epi8(a, b);
        __m256i hi = _mm256_unpackhi_epi8(a, b);

        _mm256_storeu_si256((__m256i *)(uv + 2 * i),
                            _mm256_permute2x128_si256(lo, hi, 0x20));
        _mm256_storeu_si256((__m256i *)(uv + 2 * i + 32),
                            _mm256_permute2x128_si256(lo, hi, 0x31));
    }
    return i;
}
#endif

/*
 * NV12's interleaved chroma to two planes and back: the coders keep planar
 * ones, and at 4K each of these is four million pairs a picture
 */
static void hera_deinterleave(uint8_t *u, uint8_t *v, const uint8_t *uv,
                              size_t n)
{
    size_t i = 0;

#ifdef CONFIG_AVX2_OPT
    if (cpuinfo & CPUINFO_AVX2) {
        i = hera_deinterleave_avx2(u, v, uv, n);
    }
#elif defined(__ARM_NEON)
    for (; i + 16 <= n; i += 16) {
        uint8x16x2_t p = vld2q_u8(uv + 2 * i);

        vst1q_u8(u + i, p.val[0]);
        vst1q_u8(v + i, p.val[1]);
    }
#endif
    for (; i < n; i++) {
        u[i] = uv[2 * i];
        v[i] = uv[2 * i + 1];
    }
}

static void hera_interleave(uint8_t *uv, const uint8_t *u, const uint8_t *v,
                            size_t n)
{
    size_t i = 0;

#ifdef CONFIG_AVX2_OPT
    if (cpuinfo & CPUINFO_AVX2) {
        i = hera_interleave_avx2(uv, u, v, n);
    }
#elif defined(__ARM_NEON)
    for (; i + 16 <= n; i += 16) {
        uint8x16x2_t p = { { vld1q_u8(u + i), vld1q_u8(v + i) } };

        vst2q_u8(uv + 2 * i, p);
    }
#endif
    for (; i < n; i++) {
        uv[2 * i] = u[i];
        uv[2 * i + 1] = v[i];
    }
}

typedef uint8_t hera_v16b __attribute__((vector_size(16)));
typedef uint16_t hera_v16h __attribute__((vector_size(32)));

static inline hera_v16b hera_load(const uint8_t *p)
{
    hera_v16b v;

    memcpy(&v, p, sizeof(v));
    return v;
}

static inline hera_v16b hera_absdiff(hera_v16b a, hera_v16b b)
{
    hera_v16b m = (hera_v16b)(a > b);

    return ((a - b) & m) | ((b - a) & ~m);
}

static inline uint32_t hera_hsum(const hera_v16h *v)
{
    uint32_t sum = 0;

    for (int i = 0; i < 16; i++) {
        sum += (*v)[i];
    }
    return sum;
}

/*
 * The statistics libimp's H.265 rate control reads back after a picture,
 * 38 registers it copies into its frame record next to the stream's length.
 * They follow the picture, as hera_stats_regs has them. Here the picture is
 * counted in units of 16x16 of the luma of the reconstruction (the decoded
 * picture, which is what was measured): those whose gradient (the sum of the
 * differences of neighbours inside the unit) is over 32 or 512, whose range
 * is over 2, which are one value, and in a P picture those that differ from
 * the reference by over 128 in all; with the sums of the gradients, of the
 * units' differences from the reference, and of the source's own.
 */
static void hera_h265_stats(IngenicHeraJob *j, const uint8_t *rec)
{
    uint32_t w = j->w, h = j->h, uw = w / 16, uh = h / 16;
    HeraStats *hs = &j->hs;
    const hera_v16b keep = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                             -1, -1, -1, 0 };

    memset(hs, 0, sizeof(*hs));
    hs->units = uw * uh;
    for (uint32_t uy = 0; uy < uh; uy++) {
        for (uint32_t ux = 0; ux < uw; ux++) {
            hera_v16h gacc = { 0 }, sacc = { 0 }, ssacc = { 0 };
            hera_v16b hi = { 0 }, lo = { -1, -1, -1, -1, -1, -1, -1, -1, -1,
                                         -1, -1, -1, -1, -1, -1, -1 };
            uint32_t g, sad = 0, range = 0;

            for (uint32_t y = 0; y < 16; y++) {
                size_t off = (size_t)(uy * 16 + y) * w + ux * 16;
                hera_v16b a = hera_load(rec + off), nx = a;
                hera_v16b mh = (hera_v16b)(a > hi), ml = (hera_v16b)(a < lo);

                hi = (a & mh) | (hi & ~mh);
                lo = (a & ml) | (lo & ~ml);
                /* the 16th sample has no right neighbour in its unit */
                if (off + 17 <= (size_t)w * h) {
                    nx = hera_load(rec + off + 1);
                } else {
                    memcpy(&nx, rec + off + 1, 15);
                }
                gacc += __builtin_convertvector(hera_absdiff(a, nx) & keep,
                                                hera_v16h);
                if (y < 15) {
                    gacc += __builtin_convertvector(
                        hera_absdiff(a, hera_load(rec + off + w)),
                        hera_v16h);
                }
                if (j->ref_y) {
                    sacc += __builtin_convertvector(
                        hera_absdiff(a, hera_load(j->ref_y + off)),
                        hera_v16h);
                    ssacc += __builtin_convertvector(
                        hera_absdiff(hera_load(j->y + off),
                                     hera_load(j->ref_y + off)), hera_v16h);
                }
            }
            g = hera_hsum(&gacc);
            for (int i = 0; i < 16; i++) {
                range = MAX(range, (uint32_t)hi[i] - (uint32_t)lo[i]);
            }
            hs->n_g32 += g > 32;
            hs->n_g512 += g > 512;
            hs->n_r2 += range > 2;
            hs->n_flat += range == 0;
            hs->grad += g;
            if (j->ref_y) {
                sad = hera_hsum(&sacc);
                hs->sad_src += hera_hsum(&ssacc);
                hs->n_sad128 += sad > 128;
                hs->sad += sad;
            }
        }
    }
}

static int hera_work(void *opaque)
{
    IngenicHeraJob *j = opaque;
    size_t cn = (size_t)j->w * j->h / 4;

    if (j->h264) {
        uint8_t *const *ref = ingenic_avpu_h264_ref(j->avc);

        if (j->ref_y) {
            memcpy(ref[0], j->ref_y, (size_t)j->w * j->h);
            hera_deinterleave(ref[1], ref[2], j->ref_uv, cn);
        }
        j->ok = ingenic_avpu_h264_encode(j->avc, &j->pic, j->y, j->uv, j->w,
                                         j->data, &j->st);
        if (j->ok) {
            /* the reconstruction, back to NV12 */
            ref = ingenic_avpu_h264_ref(j->avc);
            memcpy(j->y, ref[0], (size_t)j->w * j->h);
            hera_interleave(j->uv, ref[1], ref[2], cn);
        }
        return 0;
    }

    if (j->ref_y) {
        uint8_t *ref = ingenic_avpu_hevc_ref(j->hevc, j->w, j->h);

        memcpy(ref, j->ref_y, (size_t)j->w * j->h);
        hera_deinterleave(ref + (size_t)j->w * j->h,
                          ref + (size_t)j->w * j->h + cn, j->ref_uv, cn);
    }
    j->ok = ingenic_avpu_hevc_encode(j->hevc, &j->pic, j->y, j->uv, j->w,
                                     j->data, &j->st);
    if (j->ok) {
        const uint8_t *ref = ingenic_avpu_hevc_ref(j->hevc, j->w, j->h);

        hera_h265_stats(j, ref);
        memcpy(j->y, ref, (size_t)j->w * j->h);
        hera_interleave(j->uv, ref + (size_t)j->w * j->h,
                        ref + (size_t)j->w * j->h + cn, cn);
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

/*
 * The registers libimp reads, from the counts of hera_h265_stats: a half's
 * value is its scale times a count (A: a constant, scaled by the area, in
 * units of 1920x1080's 8160 units of 16x16; C: a constant).
 *
 * Read on a T32LQ (libimp 2.2.0, 1920x1080 H.265, default rate control) in
 * five scenes, the room lit, dark, lit by the colour strip or the white one,
 * and the sensor's own colour-bar pattern, the stream's pictures decoded:
 * each register's mean over a scene was fitted to the counts, the best of
 * them for each, scale by a mean of logarithms. 0x20110 high, 0x20114,
 * 0x20118, 0x2011c low, 0x20120, 0x20130 low, 0x40358 low and 0x400b4 low
 * are those fits (logarithmic errors of 0.1 to 0.7 over the five scenes).
 * 0x20130 high, read only in the dark and in the pattern, the scenes of
 * large flat areas, is assumed 0.2 of the flat units over 500. The rest
 * read the same in every scene and are held at the lit scene's value:
 * 0x20110 low, 0x400a0 to 0x400b0, 0x40350, 0x40354 and the other halves,
 * and the motion counts 0x20140 to 0x2014c, which read about the same in
 * each scene: the model shows no motion, there being no scene read with any.
 *
 * 0x20124 is not a squared error: the lit scene read 4.4 million at QP 28
 * and 6.1 million at QP 15, the pattern 0.54 million at QP 15. The model
 * takes the source's difference from the reference in a P picture (the lit
 * scene's 2.1 a sample of a noisy still scene, the pattern's 0.26 of a
 * clean one) and half the gradients in an I picture. 0x203a4 and 0x203a8,
 * which silicon reads beside it (4.4 and 0.38 million for the lit scene),
 * stay 0: the rate control is held in check by them, and what they are is
 * not known. Held at the lit scene's values for every picture, fitted to the
 * counts, or formed from the model's squared error, they took the demo of a
 * still colour-bar pattern to QP 45 at 1% of its bitrate (15 to 45 with the
 * squared error); at 0 its QP is 15 to 29, 15 on most pictures, as the
 * silicon's pattern was on every picture.
 */
enum { SRC_C, SRC_A, SRC_G32, SRC_G512, SRC_R2, SRC_SAD128, SRC_SAD, SRC_FLAT };

static void hera_stats_regs(IngenicHeraState *s, const IngenicHeraJob *j)
{
    static const struct {
        uint32_t reg;
        uint8_t hi_src, lo_src;
        double hi, lo;
    } t[] = {
        { 0x20110, SRC_G32, SRC_A, 0.0378, 1005 },
        { 0x20114, SRC_SAD128, SRC_G512, 0.1266, 0.0678 },
        { 0x20118, SRC_SAD, SRC_SAD, 0.0001245, 0.0001293 },
        { 0x2011c, SRC_C, SRC_R2, 3, 0.005096 },
        { 0x20120, SRC_C, SRC_G32, 0, 5.407 },
        { 0x20130, SRC_FLAT, SRC_G32, 0.2, 0.2415 },
        { 0x20140, SRC_A, SRC_A, 150, 1500 },
        { 0x20144, SRC_C, SRC_A, 0, 20 },
        { 0x400a0, SRC_A, SRC_A, 1287, 2992 },
        { 0x400a4, SRC_A, SRC_A, 104, 635 },
        { 0x400a8, SRC_A, SRC_A, 1136, 2282 },
        { 0x400ac, SRC_A, SRC_A, 972, 1681 },
        { 0x400b0, SRC_C, SRC_A, 0, 8902 },
        { 0x400b4, SRC_C, SRC_G32, 28, 6.069 },
        { 0x40354, SRC_C, SRC_C, 0x15e, 6 },
        { 0x40358, SRC_C, SRC_G32, 0, 6.781 },
    };
    const HeraStats *hs = &j->hs;
    double c;
    double src[] = { 1, hs->units / 8160.0, hs->n_g32, hs->n_g512, hs->n_r2,
                     hs->n_sad128, hs->sad,
                     hs->n_flat > 500 ? hs->n_flat - 500 : 0 };

    for (int i = 0; i < ARRAY_SIZE(t); i++) {
        double hi = t[i].hi * src[t[i].hi_src];
        double lo = t[i].lo * src[t[i].lo_src];

        R(t[i].reg) = (uint32_t)MIN(hi, 0xffff) << 16 |
                      (uint32_t)MIN(lo, 0xffff);
    }
    R(0x40350) = 0x19c00a1a;
    c = j->ref_y ? hs->sad_src : hs->grad * 0.5;
    R(0x20124) = c * 0.93;
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
    if (j->ok && !j->h264) {
        hera_stats_regs(s, j);
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
    if (ingenic_enc_tap_on(&s->tap) && j->ok) {
        /*
         * libimp copies the halves into a stream of its own; where the core
         * escapes them (the T33's lists) libimp adds nothing
         */
        IngenicEncTapAu au = {
            .name = "hera",
            .stream = j->key,
            .codec = j->pic.hevc ? INGENIC_TAP_H265 : INGENIC_TAP_H264,
            .width = j->w,
            .height = j->h,
            .data = j->data->data,
            .len = j->data->len,
            .rbsp = j->pic.raw,
            .at = INGENIC_TAP_FIND,
        };

        ingenic_enc_tap_au(&s->tap, &au);
    }
    if (ingenic_enc_capture_on(&s->capture)) {
        g_autoptr(GString) info = g_string_new(NULL);

        ingenic_enc_capture_pic(info, &j->pic);
        ingenic_enc_capture_regs(info, "reg", s->regs, INGENIC_HERA_IOSIZE / 4);
        g_autofree uint8_t *rec = NULL;

        if (j->ok) {
            rec = j->avc ? ingenic_avpu_h264_recon_nv12(j->avc, j->w, j->h) :
                  j->hevc ? ingenic_avpu_hevc_recon_nv12(j->hevc, j->w, j->h) :
                  NULL;
        }
        ingenic_enc_capture_rec(&s->capture, "hera", rec, j->w, j->h);
        ingenic_enc_capture(&s->capture, "hera", "bin", j->data->data,
                            j->data->len, j->cap, j->w, j->h, info);
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
    j->pic.threads = s->h265_threads;
    j->pic.h264_search = !g_strcmp0(s->h264_motion, "search");
    j->pic.h265_search = !g_strcmp0(s->h265_motion, "search");
    if (j->h264 && j->pic.h264_search) {
        j->pic.threads = s->h264_threads;
    }
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
            ingenic_isp_render_vpu(s->isp, 0, j->w, j->h, nv12);
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
    /* the coder leaves its reconstruction in the source's buffers */
    j->cap = ingenic_enc_capture_frame(&s->capture, j->y, j->w, j->uv, j->w,
                                       j->w, j->h);

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

static bool hera_encoding(IngenicHeraState *s)
{
    return s->job && !s->job->sent;
}

/* A stop lets the encode in flight finish, into RAM not yet saved */
static void hera_vm_change(void *opaque, bool running, RunState state)
{
    IngenicHeraState *s = opaque;

    if (!running) {
        AIO_WAIT_WHILE(NULL, hera_encoding(s));
    }
}

static void ingenic_hera_realize(DeviceState *dev, Error **errp)
{
    IngenicHeraState *s = INGENIC_HERA(dev);

    if (!ingenic_avpu_motion_parse("h264-motion", s->h264_motion, errp) ||
        !ingenic_avpu_motion_parse("h265-motion", s->h265_motion, errp)) {
        return;
    }
    qdev_add_vm_change_state_handler(dev, hera_vm_change, NULL, dev);
}

static int ingenic_hera_pre_save(void *opaque)
{
    if (hera_encoding(opaque)) {
        error_report("ingenic-hera: an encode is still in flight");
        return -EBUSY;
    }
    return 0;
}

/* The stream of a job waiting for the guest to take its next half */
static int hera_job_put(QEMUFile *f, void *pv, size_t size,
                        const VMStateField *field, JSONWriter *vmdesc)
{
    IngenicHeraJob *j = *(IngenicHeraJob **)pv;

    qemu_put_byte(f, !!j);
    if (j) {
        qemu_put_be32(f, j->data->len);
        qemu_put_buffer(f, j->data->data, j->data->len);
        qemu_put_be32(f, j->sent);
        qemu_put_be32(f, j->half);
    }
    return 0;
}

static int hera_job_get(QEMUFile *f, void *pv, size_t size,
                        const VMStateField *field)
{
    IngenicHeraState *s = container_of(pv, IngenicHeraState, job);
    IngenicHeraJob *j;
    uint32_t len;

    if (!qemu_get_byte(f)) {
        return 0;
    }
    j = g_new0(IngenicHeraJob, 1);
    s->job = j;
    j->s = s;
    len = qemu_get_be32(f);
    if (len > 64 * MiB) {
        return -EINVAL;
    }
    j->data = g_byte_array_sized_new(len);
    g_byte_array_set_size(j->data, len);
    qemu_get_buffer(f, j->data->data, len);
    j->sent = qemu_get_be32(f);
    j->half = qemu_get_be32(f);
    return j->sent > len ? -EINVAL : qemu_file_get_error(f);
}

static const VMStateInfo hera_job_info = {
    .name = "ingenic-hera-job",
    .get = hera_job_get,
    .put = hera_job_put,
};

/* The last pictures of the channels whose lists leave them to the model */
static int hera_refs_put(QEMUFile *f, void *pv, size_t size,
                         const VMStateField *field, JSONWriter *vmdesc)
{
    GHashTable *refs = *(GHashTable **)pv;
    GHashTableIter it;
    gpointer key, value;

    qemu_put_be32(f, g_hash_table_size(refs));
    g_hash_table_iter_init(&it, refs);
    while (g_hash_table_iter_next(&it, &key, &value)) {
        HeraRef *r = value;

        qemu_put_be32(f, GPOINTER_TO_UINT(key));
        qemu_put_be32(f, r->w);
        qemu_put_be32(f, r->h);
        qemu_put_buffer(f, r->y, (size_t)r->w * r->h);
        qemu_put_buffer(f, r->uv, (size_t)r->w * r->h / 2);
    }
    return 0;
}

static int hera_refs_get(QEMUFile *f, void *pv, size_t size,
                         const VMStateField *field)
{
    GHashTable *refs = *(GHashTable **)pv;
    uint32_t n = qemu_get_be32(f);

    g_hash_table_remove_all(refs);
    for (uint32_t i = 0; i < n; i++) {
        uint32_t key = qemu_get_be32(f);
        HeraRef *r = g_new0(HeraRef, 1);

        r->w = qemu_get_be32(f);
        r->h = qemu_get_be32(f);
        if (r->w > FRAME_MAX || r->h > FRAME_MAX) {
            g_free(r);
            return -EINVAL;
        }
        r->y = g_malloc((size_t)r->w * r->h);
        r->uv = g_malloc((size_t)r->w * r->h / 2);
        qemu_get_buffer(f, r->y, (size_t)r->w * r->h);
        qemu_get_buffer(f, r->uv, (size_t)r->w * r->h / 2);
        g_hash_table_replace(refs, GUINT_TO_POINTER(key), r);
    }
    return qemu_file_get_error(f);
}

static const VMStateInfo hera_refs_info = {
    .name = "ingenic-hera-refs",
    .get = hera_refs_get,
    .put = hera_refs_put,
};

/* The coders kept are buffers only, nothing a picture takes from the last */
static const VMStateDescription vmstate_ingenic_hera = {
    .name = TYPE_INGENIC_HERA,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = ingenic_hera_pre_save,
    .fields = (const VMStateField[]) {
        VMSTATE_BUFFER_POINTER_UNSAFE(regs, IngenicHeraState, 0,
                                      INGENIC_HERA_IOSIZE),
        VMSTATE_SINGLE(job, IngenicHeraState, 0, hera_job_info,
                       IngenicHeraJob *),
        VMSTATE_SINGLE(refs, IngenicHeraState, 0, hera_refs_info,
                       GHashTable *),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_hera_props[] = {
    DEFINE_INGENIC_ENC_CAPTURE_PROPERTY(IngenicHeraState, capture),
    DEFINE_INGENIC_ENC_TAP_PROPERTY(IngenicHeraState, tap),
    DEFINE_PROP_UINT32("h265-threads", IngenicHeraState, h265_threads, 0),
    DEFINE_PROP_STRING("h264-motion", IngenicHeraState, h264_motion),
    DEFINE_PROP_STRING("h265-motion", IngenicHeraState, h265_motion),
    DEFINE_PROP_UINT32("h264-threads", IngenicHeraState, h264_threads, 1),
    DEFINE_PROP_LINK("isp", IngenicHeraState, isp, TYPE_INGENIC_ISP,
                     IngenicIspState *),
};

static void ingenic_hera_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic HERA video encoder";
    device_class_set_props(dc, ingenic_hera_props);
    dc->realize = ingenic_hera_realize;
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
