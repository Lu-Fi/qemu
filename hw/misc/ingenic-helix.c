/*
 * Ingenic HELIX: the T23's video encoder
 *
 * libimp sets up each picture as a command list in memory, pairs of words
 * (value, then VLD bit 31 | TERM bit 30 | register offset), and the
 * soc_vpu driver hands its address to the scheduler (0x84, bit 0 to run).
 * The model replays the list into its registers; the last pair (TERM)
 * writes the front end's control (0x40000) with RUN, and the picture is
 * then encoded by the H.264 core the AVPU model uses, in the syntax
 * libimp's headers announce: High profile, CABAC (cabac_init_idc 0), one
 * reference, deblocking on, flat 4x4 scaling lists and libimp's 8x8 ones.
 *
 * What the list says, by register: the picture's last macroblock column
 * and row (0x40004, bits 7:0 and 15:8), the NV12 source (0x40010 luma,
 * 0x40014 chroma, strides 0x40038 bits 31:16 and 15:0), the QP (0x40000
 * bits 13:8) and P (bit 4); the bitstream window (0x30004, 0x30040 in
 * KiB), where the slice data goes as an RBSP (libimp puts the slice header
 * in front and adds the emulation prevention bytes); the reconstruction
 * (0x60008 luma, 0x6000c chroma, both rows 0x60010 bits 31:16 apart) and
 * the reference (0xb0014, 0xb0018). libimp keeps both in one ring per
 * channel (0x60004 bit 30, luma 0x60014 to 0x6001c, chroma 0x60018 to
 * 0x60020): the reconstruction of each picture starts 256 rows before its
 * reference, rows wrapping at the ring's end. The model keeps the
 * reconstruction there as NV12 rows; nothing but the core reads it.
 *
 * At the end the core holds the bytes written in 0x90038, the status
 * 0x301 in 0x34 (end, and two done flags helix.h does not name), and the
 * statistics libimp's rate control reads back, as a real T23 counts them:
 * 0x800e0 the intra 16x16 macroblocks (bits 14:0) and the 4x4 or 8x8 ones
 * (30:16), 0x80168 the others of these (14:0) and part of the inter ones
 * (30:16), 0x800e4 the rest of the inter ones (14:0) and the skipped ones
 * (30:16), 0x800f8 all of them; 0x80080 a sum of absolute differences
 * against the prediction, 0x80088 (64 bits) the luma squared error, and
 * 0x40094-0x400a0 the macroblocks by activity class (16 bits each),
 * against the thresholds at 0x4007c-0x40088. It raises its interrupt while
 * 0x34 bit 0 and the end interrupt's enable (0x0 bit 16) are both set.
 * A JPEG job (0x40000 bits 15:14 = 1) codes a scan with the JPEG core's
 * tables, see helix_jpeg().
 *
 * Not modelled: the direct line from the ISP (IVDC), the bitstream-full
 * protocol, motion search (inter macroblocks do not move, so 0x800e4 bits
 * 14:0 and the motion sums 0x800e8-0x800f4 stay 0), macroblock rate
 * control (one QP per picture), and 0x80120-0x8014c.
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
#include "hw/misc/ingenic-helix.h"
#include "hw/misc/ingenic-avpu.h"

#define SCH_GLBC            0x00000
#define  GLBC_INTE_END      BIT(16)
#define SCH_STAT            0x00034
#define  STAT_END           BIT(0)
#define  STAT_JPEG          BIT(4)
#define  STAT_DONE          (BIT(8) | BIT(9))
#define VDMA_TASKRG_T21     0x00084
#define VDMA_TASKRG         0x10008
#define  TASK_RUN           BIT(0)
#define  TASK_ADDR          0xffffff80u
#define VDMA_TASKST         0x1000c
#define  TASKST_BUSY        BIT(0)
#define  TASKST_END         BIT(2)
#define CL_VLD              BIT(31)
#define CL_TERM             BIT(30)
#define CL_REG              0xffffcu
#define CL_MAX              0x4000      /* pairs; libimp's lists hold ~1030 */

#define EMC_BS_ADDR         0x30004
#define EMC_BS_SIZE         0x30040
#define EFE_CTRL            0x40000
#define  CTRL_RUN           BIT(0)
#define  CTRL_P             BIT(4)
#define EFE_GEOM            0x40004
#define EFE_RAWY            0x40010
#define EFE_RAWC            0x40014
#define EFE_RAW_STRD        0x40038
#define EFE_ACT_THD         0x4007c     /* 4 words of u16 thresholds */
#define EFE_ACT_HIST        0x40094     /* 4 words of u16 counts */
#define ODMA_CTRL           0x60004
#define  ODMA_RING          BIT(30)
#define ODMA_RECON_Y        0x60008
#define ODMA_RECON_C        0x6000c
#define ODMA_STRIDE         0x60010
#define ODMA_RING_Y         0x60014
#define ODMA_RING_C         0x60018
#define ODMA_RING_Y_END     0x6001c
#define ODMA_RING_C_END     0x60020
#define DBLK_GSTA           0x70070
#define  DBLK_DOEND         BIT(0)
#define VMAU_SAD            0x80080
#define VMAU_SSE            0x80088     /* and 0x8008c, the high word */
#define VMAU_INTRA          0x800e0
#define VMAU_INTER          0x800e4
#define VMAU_MBS            0x800f8
#define VMAU_INTRA8         0x80168
#define SDE_STAT            0x90000
#define  SDE_BSEND          BIT(1)
#define SDE_BS_LEN          0x90038     /* helix.h: REG_SDE_CFG9 */
#define REF_Y               0xb0014
#define REF_C               0xb0018
#define JPGC_STAT           0xe0008
#define  JPGC_ENDF          BIT(31)
#define JPGC_BSA            0xe000c
#define JPGC_MAX_BS         0xe0068
#define  MAX_BS_EN          BIT(31)
#define JPGC_ACT_BS         0xe006c
#define JPGC_QMEM           0xe1400     /* luma, then chroma, zigzag */
#define JPGC_HUFE           0xe1800

#define ACT_CLASSES         8
#define JPEG_MAX            4096        /* samples a side */

#define R(off)              (s->regs[(off) / 4])

/* Rows of a plane from @addr, wrapping from @end back to @start if a ring */
typedef struct HelixPlane {
    uint32_t addr, start, end, stride;
} HelixPlane;

struct IngenicHelixJob {
    IngenicHelixState *s;       /* NULL once the device has reset */
    bool jpeg;
    uint32_t mul[2][64];        /* JPEG: the quantizers, natural order */
    uint8_t shift[2][64];
    IngenicAvpuJpegHuff huff[4];
    IngenicAvpuPic pic;
    IngenicAvpuH264 *h264;
    uint32_t w, h;              /* whole macroblocks */
    uint8_t *y, *uv;            /* the source, @w apart */
    uint8_t *ref_y, *ref_uv;    /* P: the reference, as the ring holds it */
    HelixPlane recon_y, recon_c;
    uint32_t bs, room;
    uint16_t thd[ACT_CLASSES - 1];
    GByteArray *data;
    IngenicAvpuStats st;
    uint64_t sad, sse;
    uint16_t hist[ACT_CLASSES];
    bool ok;
};

/* libimp's 8x8 lists, the default ones plus 10, zigzag */
static const uint8_t helix_scale8[2][64] = {
    {
        16, 20, 20, 23, 21, 23, 26, 26, 26, 26, 28, 28, 28, 28, 28, 33,
        33, 33, 33, 33, 33, 35, 35, 35, 35, 35, 35, 35, 37, 37, 37, 37,
        37, 37, 37, 37, 39, 39, 39, 39, 39, 39, 39, 41, 41, 41, 41, 41,
        41, 43, 43, 43, 43, 43, 46, 46, 46, 46, 48, 48, 48, 50, 50, 52,
    }, {
        19, 23, 23, 25, 23, 25, 27, 27, 27, 27, 29, 29, 29, 29, 29, 31,
        31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32, 34, 34, 34, 34,
        34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 37, 37, 37, 37, 37,
        37, 38, 38, 38, 38, 38, 40, 40, 40, 40, 42, 42, 42, 43, 43, 45,
    },
};

/* The mode decisions' lambda by QP, the AVPU model's default */
static const uint8_t helix_lambda[52] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 2, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 10, 11, 12, 14, 16, 17,
    20, 22, 25, 28, 32, 35, 40,
};

static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

static void helix_update(IngenicHelixState *s)
{
    qemu_set_irq(s->irq, (R(SCH_STAT) & STAT_END) &&
                         (R(SCH_GLBC) & GLBC_INTE_END));
}

static uint32_t helix_row(const HelixPlane *p, uint32_t r)
{
    uint32_t off = r * p->stride;

    if (p->end > p->start && p->addr >= p->start && p->addr < p->end) {
        return p->start + (p->addr - p->start + off) % (p->end - p->start);
    }
    return p->addr + off;
}

static void helix_plane_io(const HelixPlane *p, uint8_t *buf, uint32_t w,
                           uint32_t rows, bool write)
{
    for (uint32_t r = 0; r < rows; r++) {
        address_space_rw(&address_space_memory, helix_row(p, r),
                         MEMTXATTRS_UNSPECIFIED, buf + r * w, w, write);
    }
}

static void helix_job_free(IngenicHelixJob *j)
{
    ingenic_avpu_h264_free(j->h264);
    g_free(j->y);
    g_free(j->uv);
    g_free(j->ref_y);
    g_free(j->ref_uv);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

static uint32_t sad16(const uint8_t *a, int as, const uint8_t *b, int bs)
{
    uint32_t sum = 0;

    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
            sum += abs(a[y * as + x] - b[y * bs + x]);
        }
    }
    return sum;
}

/*
 * The least SAD of a macroblock against intra 16x16 DC, vertical and
 * horizontal prediction from the source around it
 */
static uint32_t intra_sad(const IngenicHelixJob *j, int mbx, int mby)
{
    int w = j->w;
    const uint8_t *p = j->y + mby * 16 * w + mbx * 16;
    uint8_t pred[256];
    uint32_t best, dc = 0, n = 0;

    for (int i = 0; i < 16; i++) {
        if (mby) {
            dc += p[i - w];
            n++;
        }
        if (mbx) {
            dc += p[i * w - 1];
            n++;
        }
    }
    memset(pred, n ? (dc + n / 2) / n : 128, sizeof(pred));
    best = sad16(p, w, pred, 16);
    if (mby) {
        best = MIN(best, sad16(p, w, p - w, 0));
    }
    if (mbx) {
        for (int y = 0; y < 16; y++) {
            memset(pred + 16 * y, p[y * w - 1], 16);
        }
        best = MIN(best, sad16(p, w, pred, 16));
    }
    return best;
}

/*
 * How busy a macroblock's luma is: the Hadamard transforms of its 4x4
 * blocks without their DC, over 6.28, which puts a T23's sub-stream
 * macroblocks into its classes within 3 %
 */
static uint32_t activity(const IngenicHelixJob *j, int mbx, int mby)
{
    const uint8_t *p = j->y + mby * 16 * j->w + mbx * 16;
    uint32_t sum = 0;

    for (int b = 0; b < 16; b++) {
        const uint8_t *q = p + (b / 4) * 4 * j->w + (b % 4) * 4;
        int t[16];

        for (int i = 0; i < 4; i++) {
            const uint8_t *r = q + i * j->w;
            int s0 = r[0] + r[1], s1 = r[2] + r[3];
            int d0 = r[0] - r[1], d1 = r[2] - r[3];

            t[i * 4] = s0 + s1;
            t[i * 4 + 1] = s0 - s1;
            t[i * 4 + 2] = d0 + d1;
            t[i * 4 + 3] = d0 - d1;
        }
        for (int i = 0; i < 4; i++) {
            int s0 = t[i] + t[4 + i], s1 = t[8 + i] + t[12 + i];
            int d0 = t[i] - t[4 + i], d1 = t[8 + i] - t[12 + i];

            sum += (i ? abs(s0 + s1) : 0) + abs(s0 - s1) + abs(d0 + d1) +
                   abs(d0 - d1);
        }
    }
    return sum * 163 >> 10;
}

static void helix_source_stats(IngenicHelixJob *j)
{
    for (int mby = 0; mby < j->h / 16; mby++) {
        for (int mbx = 0; mbx < j->w / 16; mbx++) {
            uint32_t a = activity(j, mbx, mby), sad = intra_sad(j, mbx, mby);
            int c = 0;

            while (c < ACT_CLASSES - 1 && j->thd[c] && a >= j->thd[c]) {
                c++;
            }
            j->hist[c]++;
            if (j->ref_y) {
                int off = mby * 16 * j->w + mbx * 16;

                sad = MIN(sad, sad16(j->y + off, j->w, j->ref_y + off, j->w));
            }
            j->sad += sad;
        }
    }
}

static int helix_work(void *opaque)
{
    IngenicHelixJob *j = opaque;
    uint32_t cw = j->w / 2;
    uint8_t *const *ref;

    if (j->jpeg) {
        ingenic_avpu_jpeg_scan(j->y, j->w, j->uv, j->w, j->w, j->h, j->mul,
                               j->shift, j->huff, j->data);
        j->ok = true;
        return 0;
    }

    if (j->ref_y) {
        ref = ingenic_avpu_h264_ref(j->h264);
        memcpy(ref[0], j->ref_y, (size_t)j->w * j->h);
        for (size_t i = 0; i < (size_t)cw * j->h / 2; i++) {
            ref[1][i] = j->ref_uv[2 * i];
            ref[2][i] = j->ref_uv[2 * i + 1];
        }
    }
    helix_source_stats(j);
    j->ok = ingenic_avpu_h264_encode(j->h264, &j->pic, j->y, j->uv, j->w,
                                     j->data, &j->st);
    if (!j->ok) {
        return 0;
    }
    /* the reconstruction, back to NV12 in the reference buffers */
    ref = ingenic_avpu_h264_ref(j->h264);
    if (!j->ref_y) {
        j->ref_y = g_malloc((size_t)j->w * j->h);
        j->ref_uv = g_malloc((size_t)j->w * j->h / 2);
    }
    for (size_t i = 0; i < (size_t)j->w * j->h; i++) {
        int d = j->y[i] - ref[0][i];

        j->sse += d * d;
    }
    memcpy(j->ref_y, ref[0], (size_t)j->w * j->h);
    for (size_t i = 0; i < (size_t)cw * j->h / 2; i++) {
        j->ref_uv[2 * i] = ref[1][i];
        j->ref_uv[2 * i + 1] = ref[2][i];
    }
    return 0;
}

static void helix_done(void *opaque, int ret)
{
    IngenicHelixJob *j = opaque;
    IngenicHelixState *s = j->s;
    uint32_t len, i4;

    if (!s) {
        helix_job_free(j);
        return;
    }
    if (j->jpeg) {
        len = MIN(j->data->len, j->room);
        address_space_write(&address_space_memory, j->bs,
                            MEMTXATTRS_UNSPECIFIED, j->data->data, len);
        R(JPGC_STAT) = JPGC_ENDF | (len & 0xffffff);
        R(JPGC_ACT_BS) = len;
        R(VDMA_TASKST) = TASKST_END;
        R(SCH_STAT) = STAT_END | STAT_JPEG;
        s->job = NULL;
        helix_update(s);
        helix_job_free(j);
        return;
    }
    if (!j->ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-helix: encode failed\n");
        g_byte_array_set_size(j->data, 0);
    }
    len = MIN(j->data->len, j->room);
    if (len < j->data->len) {
        qemu_log_mask(LOG_UNIMP, "ingenic-helix: %u bytes of slice data for "
                      "a %u byte window\n", j->data->len, j->room);
    }
    address_space_write(&address_space_memory, j->bs,
                        MEMTXATTRS_UNSPECIFIED, j->data->data, len);
    if (j->ok) {
        helix_plane_io(&j->recon_y, j->ref_y, j->w, j->h, true);
        helix_plane_io(&j->recon_c, j->ref_uv, j->w, j->h / 2, true);
    }

    i4 = j->st.intra_mbs - j->st.i16_mbs - j->st.i8_mbs;
    R(SDE_BS_LEN) = len;
    R(VMAU_INTRA) = j->st.i16_mbs | i4 << 16;
    R(VMAU_INTRA8) = j->st.i8_mbs | j->st.inter_mbs << 16;
    R(VMAU_INTER) = j->st.skip_mbs << 16;
    R(VMAU_MBS) = (j->w / 16) * (j->h / 16);
    R(VMAU_SAD) = MIN(j->sad, UINT32_MAX);
    R(VMAU_SSE) = j->sse;
    R(VMAU_SSE + 4) = j->sse >> 32;
    for (int c = 0; c < ACT_CLASSES; c += 2) {
        R(EFE_ACT_HIST + 2 * c) = j->hist[c] | j->hist[c + 1] << 16;
    }
    R(SDE_STAT) |= SDE_BSEND;
    R(DBLK_GSTA) |= DBLK_DOEND;
    R(VDMA_TASKST) = TASKST_END;
    R(SCH_STAT) = STAT_END | STAT_DONE;
    s->job = NULL;
    helix_update(s);
    helix_job_free(j);
}

static void helix_encode(IngenicHelixState *s)
{
    IngenicHelixJob *j = g_new0(IngenicHelixJob, 1);
    uint32_t ctrl = R(EFE_CTRL), geom = R(EFE_GEOM);
    uint32_t ystride = R(EFE_RAW_STRD) >> 16, cstride = R(EFE_RAW_STRD) & 0xffff;
    uint32_t stride = R(ODMA_STRIDE) >> 16;
    bool ring = R(ODMA_CTRL) & ODMA_RING;

    j->s = s;
    j->w = (extract32(geom, 0, 8) + 1) * 16;
    j->h = (extract32(geom, 8, 8) + 1) * 16;
    j->data = g_byte_array_new();
    j->bs = R(EMC_BS_ADDR);
    j->room = R(EMC_BS_SIZE) * KiB;
    for (int c = 0; c < ACT_CLASSES - 1; c++) {
        j->thd[c] = R(EFE_ACT_THD + 4 * (c / 2)) >> (16 * (c & 1));
    }

    j->pic.width = j->w;
    j->pic.height = j->h;
    j->pic.qp = MIN(extract32(ctrl, 8, 6), 51);
    j->pic.intra = !(ctrl & CTRL_P);
    j->pic.cabac = true;
    j->pic.transform_8x8 = true;
    j->pic.scale8[0] = helix_scale8[0];
    j->pic.scale8[1] = helix_scale8[1];
    j->pic.raw = true;
    memcpy(j->pic.lambda, helix_lambda, sizeof(helix_lambda));
    j->h264 = ingenic_avpu_h264_new(&j->pic);

    if (!stride) {
        stride = j->w;
    }
    j->recon_y = (HelixPlane) {
        R(ODMA_RECON_Y), ring ? R(ODMA_RING_Y) : 0,
        ring ? R(ODMA_RING_Y_END) : 0, stride,
    };
    j->recon_c = (HelixPlane) {
        R(ODMA_RECON_C), ring ? R(ODMA_RING_C) : 0,
        ring ? R(ODMA_RING_C_END) : 0, stride,
    };

    j->y = g_malloc((size_t)j->w * j->h);
    j->uv = g_malloc((size_t)j->w * j->h / 2);
    helix_plane_io(&(HelixPlane) { R(EFE_RAWY), 0, 0, ystride ?: j->w },
                   j->y, j->w, j->h, false);
    helix_plane_io(&(HelixPlane) { R(EFE_RAWC), 0, 0, cstride ?: j->w },
                   j->uv, j->w, j->h / 2, false);
    if (!j->pic.intra) {
        HelixPlane ry = j->recon_y, rc = j->recon_c;

        ry.addr = R(REF_Y);
        rc.addr = R(REF_C);
        j->ref_y = g_malloc((size_t)j->w * j->h);
        j->ref_uv = g_malloc((size_t)j->w * j->h / 2);
        helix_plane_io(&ry, j->ref_y, j->w, j->h, false);
        helix_plane_io(&rc, j->ref_uv, j->w, j->h / 2, false);
    }
    s->job = j;
    R(VDMA_TASKST) = TASKST_BUSY;
    thread_pool_submit_aio(helix_work, j, helix_done, j);
}

/* One Huffman code from JPGC_HUFE: its length less one, its low byte */
static void helix_huff_code(IngenicAvpuJpegHuff *h, int sym, uint32_t e)
{
    int len = extract32(e, 8, 4) + 1;

    h->size[sym] = len;
    h->code[sym] = (len > 8 ? MAKE_64BIT_MASK(8, len - 8) : 0) | (e & 0xff);
}

/*
 * A baseline 4:2:0 JPEG scan of the NV12 picture at the EFE's source,
 * whole macroblocks (0x40004: rows less one in bits 31:16, columns in
 * 15:0), to JPGC_BSA. The Huffman codes come from JPGC_HUFE: AC luma at
 * run * 10 + size - 1, then its EOB and ZRL (160, 161), AC chroma the same
 * from 176, DC luma by category from 352 and DC chroma from 368; codes
 * longer than 8 bits start with ones. The quantizers come from JPGC_QMEM,
 * k << 11 | round(2^(11 + k) / q), and 1 for q = 1. The core reports the
 * scan's length in JPGC_STAT and JPGC_ACT_BS.
 */
static void helix_jpeg(IngenicHelixState *s)
{
    IngenicHelixJob *j = g_new0(IngenicHelixJob, 1);
    uint32_t geom = R(EFE_GEOM), max = R(JPGC_MAX_BS);
    uint32_t ystride = R(EFE_RAW_STRD) >> 16, cstride = R(EFE_RAW_STRD) & 0xffff;

    j->s = s;
    j->jpeg = true;
    j->w = (extract32(geom, 0, 16) + 1) * 16;
    j->h = (extract32(geom, 16, 16) + 1) * 16;
    j->data = g_byte_array_new();
    if (j->w > JPEG_MAX || j->h > JPEG_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-helix: a %ux%u JPEG\n",
                      j->w, j->h);
        s->job = j;
        helix_done(j, 0);
        return;
    }
    j->bs = R(JPGC_BSA);
    /* libimp enables the limit with its pool size, which may be 0 */
    j->room = max & ~MAX_BS_EN && max & MAX_BS_EN ? max & ~MAX_BS_EN
                                                  : UINT32_MAX;
    for (int t = 0; t < 2; t++) {
        for (int k = 0; k < 64; k++) {
            uint32_t e = R(JPGC_QMEM + 4 * (64 * t + k));

            j->mul[t][zigzag[k]] = e > 1 ? e & 0x7ff : e;
            j->shift[t][zigzag[k]] = e > 1 ? 14 + extract32(e, 11, 3) : 3;
        }
    }
    for (int c = 0; c < 2; c++) {
        IngenicAvpuJpegHuff *ac = &j->huff[2 * c + 1];
        IngenicAvpuJpegHuff *dc = &j->huff[2 * c];
        hwaddr base = JPGC_HUFE + c * 176 * 4;

        for (int run = 0; run < 16; run++) {
            for (int size = 1; size <= 10; size++) {
                helix_huff_code(ac, run << 4 | size,
                                R(base + 4 * (run * 10 + size - 1)));
            }
        }
        helix_huff_code(ac, 0x00, R(base + 160 * 4));
        helix_huff_code(ac, 0xf0, R(base + 161 * 4));
        for (int cat = 0; cat < 12; cat++) {
            helix_huff_code(dc, cat, R(JPGC_HUFE + (352 + 16 * c + cat) * 4));
        }
    }

    j->y = g_malloc((size_t)j->w * j->h);
    j->uv = g_malloc((size_t)j->w * j->h / 2);
    helix_plane_io(&(HelixPlane) { R(EFE_RAWY), 0, 0, ystride ?: j->w },
                   j->y, j->w, j->h, false);
    helix_plane_io(&(HelixPlane) { R(EFE_RAWC), 0, 0, cstride ?: j->w },
                   j->uv, j->w, j->h / 2, false);
    s->job = j;
    R(VDMA_TASKST) = TASKST_BUSY;
    thread_pool_submit_aio(helix_work, j, helix_done, j);
}

/* Replay a command list into the registers; its last write may start */
static void helix_run(IngenicHelixState *s, uint32_t list)
{
    uint32_t pair[2], ctrl = 0, addr = list;
    int n;

    if (s->job) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-helix: started while busy\n");
        return;
    }
    R(SCH_STAT) = 0;
    helix_update(s);
    for (n = 0; n < CL_MAX; n++, addr += 8) {
        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, pair, sizeof(pair)) !=
            MEMTX_OK) {
            break;
        }
        ctrl = le32_to_cpu(pair[1]);
        if (ctrl & CL_VLD) {
            R(ctrl & CL_REG) = le32_to_cpu(pair[0]);
        }
        if (ctrl & CL_TERM) {
            break;
        }
    }
    if (n == CL_MAX || !(ctrl & CL_TERM)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-helix: command list at 0x%08x "
                      "has no end\n", list);
        return;
    }
    if (!(R(EFE_CTRL) & CTRL_RUN)) {
        R(VDMA_TASKST) = TASKST_END;
        return;
    }
    switch (extract32(R(EFE_CTRL), 14, 2)) {
    case 0:
        helix_encode(s);
        break;
    case 1:
        helix_jpeg(s);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-helix: only H.264 and JPEG are "
                      "modelled (0x40000 = 0x%08x)\n", R(EFE_CTRL));
        R(VDMA_TASKST) = TASKST_END;
        R(SCH_STAT) = STAT_END;
        helix_update(s);
    }
}

static uint64_t ingenic_helix_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicHelixState *s = INGENIC_HELIX(opaque);

    return R(addr);
}

static void ingenic_helix_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    IngenicHelixState *s = INGENIC_HELIX(opaque);
    uint32_t v = value;

    switch (addr) {
    case SCH_GLBC:
        R(addr) = v;
        helix_update(s);
        return;
    case VDMA_TASKRG_T21:
    case VDMA_TASKRG:
        R(addr) = v;
        if (v & TASK_RUN) {
            helix_run(s, v & TASK_ADDR);
        }
        return;
    case VDMA_TASKST:
        return;
    }
    R(addr) = v;
}

static const MemoryRegionOps ingenic_helix_ops = {
    .read = ingenic_helix_read,
    .write = ingenic_helix_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_helix_reset_hold(Object *obj, ResetType type)
{
    IngenicHelixState *s = INGENIC_HELIX(obj);

    if (s->job) {
        s->job->s = NULL;           /* its completion just frees it */
        s->job = NULL;
    }
    memset(s->regs, 0, INGENIC_HELIX_IOSIZE);
}

static void ingenic_helix_reset_exit(Object *obj, ResetType type)
{
    helix_update(INGENIC_HELIX(obj));
}

static void ingenic_helix_init(Object *obj)
{
    IngenicHelixState *s = INGENIC_HELIX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->regs = g_malloc0(INGENIC_HELIX_IOSIZE);
    memory_region_init_io(&s->iomem, obj, &ingenic_helix_ops, s,
                          TYPE_INGENIC_HELIX, INGENIC_HELIX_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_helix_finalize(Object *obj)
{
    g_free(INGENIC_HELIX(obj)->regs);
}

/* An encode in flight is host state */
static const VMStateDescription vmstate_ingenic_helix = {
    .name = TYPE_INGENIC_HELIX,
    .unmigratable = 1,
};

static void ingenic_helix_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic HELIX video encoder";
    dc->vmsd = &vmstate_ingenic_helix;
    rc->phases.hold = ingenic_helix_reset_hold;
    rc->phases.exit = ingenic_helix_reset_exit;
}

static const TypeInfo ingenic_helix_types[] = {
    {
        .name = TYPE_INGENIC_HELIX,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicHelixState),
        .instance_init = ingenic_helix_init,
        .instance_finalize = ingenic_helix_finalize,
        .class_init = ingenic_helix_class_init,
    },
};

DEFINE_TYPES(ingenic_helix_types)
