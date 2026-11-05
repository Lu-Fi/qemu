/*
 * Ingenic HELIX: the video encoder (and JPEG decoder) of the T10, T20, T21,
 * T23 and T30
 *
 * libimp sets up each picture as a command list in memory, pairs of words
 * (value, then VLD bit 31 | TERM bit 30 | register offset), and the
 * soc_vpu driver hands its address to the scheduler (0x84 on the T21 and
 * T23, 0x10008 elsewhere, bit 0 to run).
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
 * That is the T23's list, and the T21's but for the ring. The T30's and
 * T20's lists have neither the window nor 0x6xxxx and 0xbxxxx: the slice
 * data goes to 0x90024 (up to a raw picture's size), the reconstruction to
 * 0x70084 and 0x70088, and a P picture refers to 0x50800 and 0x50804;
 * their syntax has no 8x8 transform and flat scaling lists. The T10's
 * refers to 0x50304 and 0x50b04, its pictures having a macroblock of
 * border all round: the reconstruction starts (width / 16 + 3) * 256 luma
 * and half as many chroma bytes into the planes 0x70084 and 0x70088 name.
 * Their kernels take the frame's SAD (cmpx) from 0x40108 (0x80080 on the
 * T20), which a T23 leaves 0.
 *
 * At the end the core holds the bytes written in 0x90038, the status
 * 0x301 in 0x34 (end, and two done flags helix.h does not name; the end
 * and bit 9 last until the driver clears the units' flags, which on a T10,
 * T20 or T30 clears bit 8 too: a T23 reads 0x100 between pictures, they 0,
 * and while they code 0x20000 (T10) or 0x40000), and the
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
 * tables, see helix_jpeg(). A source address with bit 31 set (0x40010)
 * comes over IVDC, straight from the ISP, which draws it (the "isp"
 * link).
 *
 * The T41 keeps the JPEG path alone as an engine of its own (0x13280000,
 * 256 KiB, INTC source 0, reset by the CPM's SRBC bit 28), which libimp
 * uses for a JPEG channel whose encVputype asks for the IVPU: the same
 * units, the EFE renumbered 2 and the JPGC 3, its lists indexing them
 * 0x80000 above, the scheduler at 0x84. A T41NQ's scans of libimp's lists
 * are the model's byte for byte.
 *
 * The JPEG codec (JPGC) also decodes, for libimp's IMP_Decoder on the T23
 * and T30: a list that ends by triggering it (0xe0000 bits 1 and 2) with
 * 0xe0004 (GLBI) bits 0 and 3 set, and no front end. The scan's entropy
 * coded data is at JPGC_BSA, the tables in decoding form at 0xe1200
 * (HUFB), 0xe1300 (HUFM), 0xe1400 (QMEM) and 0xe1800 (symbols), the MCUs
 * less one in 0xe0028, the restart interval less one in 0xe002c (GLBI bit
 * 2 on), the components' tables in 0xe0030-0xe003c; the MCUs go out as
 * tiles from JPGC_P0A (luma) and P1A (chroma), see helix_jpgc_decode().
 * At the end 0xe0064 holds the tiles decoded, P0A and P1A point past them
 * and BSA past the burst after the one holding the scan's end, 0x34 reads
 * 0x11; a bitstream error ends with 0x34 bit 7 alone (0xe0008 bit 30
 * instead of 31), and data that end with the tile after NMCU's with both,
 * 0x91. GLBI reads its fields 5:4 and 3:2 swapped. A T30X's and a T23N's
 * tiles and registers, for every scan probed, are the model's, but for
 * what the list below leaves out.
 *
 * 0x30 (TLBA), 0x50 (TLBC), 0x54 (TLBV) and GLBC bits 26:21 are a TLB's
 * (helix.h, JZM_V2_TLB: the units' enables), and the T10's and T20's
 * driver turns them on, but on a T20X, T23N or T30X setting them, or GLBC
 * bit 30 (the TLB's enable in Ingenic's other VPUs, which reads 0 on the
 * T20 and T30), translates no address: the units read and write memory
 * physically, which is what the model does. The registers keep what those
 * parts keep (TLBA bits 31:12, TLBC 31:4, TLBV 0xcfff0fff; GLBC all on the
 * T23, 0xaffffff7 on the T20 and T30), TLBA and TLBC through the soft
 * reset.
 *
 * Not modelled: the bitstream-full protocol, motion search (inter
 * macroblocks do not move, so 0x800e4 bits 14:0 and the motion sums
 * 0x800e8-0x800f4 stay 0), macroblock rate control (one QP per picture),
 * and 0x80120-0x8014c; the JPGC's UNIV mode and its encoding from tiles;
 * the blocks a T23N decodes past the data of a scan of two or three
 * bytes; a run past three multiples of 64 (a bitstream error here); where
 * the fetch stands at a bitstream error, which on a T23N varies with the
 * scan (the model's formula is its, for scans of one byte a block); P2A
 * and P3A's fixed upper bits.
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
#include "hw/core/qdev-properties.h"
#include "qemu/timer.h"
#include "trace.h"

#define SCH_GLBC            0x00000
#define  GLBC_INTE_END      BIT(16)
#define  GLBC_INTE_BSERR    BIT(17)
#define SCH_TLBA            0x00030
#define SCH_STAT            0x00034
#define SCH_TLBC            0x00050     /* bits 3:0 are triggers, read 0 */
#define SCH_TLBV            0x00054
#define  STAT_END           BIT(0)
#define  STAT_JPEG          BIT(4)
#define  STAT_BSERR         BIT(7)
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
#define  IVDC_ADDR          BIT(31)     /* a source address over IVDC */
#define EFE_RAWC            0x40014
#define EFE_RAW_STRD        0x40038
#define EFE_ACT_THD         0x4007c     /* 4 words of u16 thresholds */
#define EFE_ACT_HIST        0x40094     /* 4 words of u16 counts */
#define EFE_SSAD            0x40108
#define EFE_CQP_OFST        0x40120
#define MCE_REF1_Y          0x50304     /* T10 */
#define MCE_REF1_C          0x50b04
#define MCE_REF_Y           0x50800     /* T20, T30 */
#define MCE_REF_C           0x50804
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
#define DBLK_CTRL           0x70064
#define  DBLK_EN            BIT(0)
#define DBLK_RECON_Y        0x70084
#define DBLK_RECON_C        0x70088
#define VMAU_SAD            0x80080
#define VMAU_SSE            0x80088     /* and 0x8008c, the high word */
#define VMAU_INTRA          0x800e0
#define VMAU_INTER          0x800e4
#define VMAU_MBS            0x800f8
#define VMAU_INTRA8         0x80168
#define SDE_STAT            0x90000
#define  SDE_BSEND          BIT(1)
#define SDE_BS_ADDR         0x90024     /* helix.h: REG_SDE_CFG4 */
#define SDE_BS_LEN          0x90038     /* helix.h: REG_SDE_CFG9 */
#define REF_Y               0xb0014
#define REF_C               0xb0018
#define JPGC_TRIG           0xe0000     /* reads 0 */
#define  TRIG_BS            BIT(1)
#define  TRIG_PP            BIT(2)
#define JPGC_GLBI           0xe0004
#define  GLBI_EN            BIT(0)
#define  GLBI_UNIV          BIT(1)
#define  GLBI_RSM           BIT(2)
#define  GLBI_DEC           BIT(3)
#define  GLBI_CLR           BIT(6)      /* with EN: clears 5:2 */
#define  GLBI_MASK          0xfffff1bfu
#define JPGC_STAT           0xe0008
#define  JPGC_ENDF          BIT(31)
#define  JPGC_ERRF          BIT(30)     /* with a bitstream error */
#define  JPGC_STAT_MASK     0xc0ffffffu
#define JPGC_BSA            0xe000c
#define JPGC_P0A            0xe0010     /* luma tiles */
#define JPGC_P1A            0xe0014     /* chroma tiles */
#define JPGC_P2A            0xe0018
#define JPGC_P3A            0xe001c
#define JPGC_E20            0xe0020     /* a constant, by part */
#define JPGC_GLBI_RD        0xe0024     /* GLBI's bits 5:2 as read */
#define JPGC_NMCU           0xe0028
#define JPGC_NRSM           0xe002c
#define JPGC_CCI            0xe0030     /* 4 components */
#define JPGC_E60            0xe0060     /* a constant, by part */
#define JPGC_MCUS           0xe0064
#define JPGC_MAX_BS         0xe0068
#define  MAX_BS_EN          BIT(31)
#define JPGC_ACT_BS         0xe006c
#define JPGC_HUFB           0xe1200     /* decoding: 16 bases a table */
#define JPGC_HUFM           0xe1300     /* decoding: 4 words of minima */
#define JPGC_QMEM           0xe1400     /* luma, then chroma, zigzag */
#define JPGC_HUFE           0xe1800     /* decoding: the symbols */

#define ACT_CLASSES         8
#define JPEG_MAX            4096        /* samples a side */
#define JPGC_SPILL          3           /* blocks a run can skip into */

#define R(off)              (s->regs[(off) / 4])

/* Where a SoC's lists put things: 0 for a register its lists lack */
struct IngenicHelixLayout {
    bool window;            /* slice data to the EMC window, else SDE_BS_ADDR */
    hwaddr recon_y, recon_c, ref_y, ref_c;
    bool border;                /* the T10's references */
    bool high;                  /* 8x8 transform, libimp's 8x8 lists */
    bool cmpx;                  /* the SAD in EFE_SSAD too */
    bool jpeg_limit;            /* JPGC_MAX_BS caps the scan */
    uint32_t busy;              /* 0x34 while it codes; 0 between pictures */
    const uint8_t *units;       /* the part's unit numbers, as the T23's */
    uint32_t list_base;         /* a list's register index of offset 0 */
    uint32_t iosize;            /* the window, if not INGENIC_HELIX_IOSIZE */
    bool t32_dct;               /* JPEG with the T32 engine's DCT */
    uint32_t jpgc_e20, jpgc_e60; /* what they read, 0 if not measured */
    uint32_t glbc_mask;         /* GLBC's bits, if not all of them */
};

/* Rows of a plane from @addr, wrapping from @end back to @start if a ring */
typedef struct HelixPlane {
    uint32_t addr, start, end, stride;
} HelixPlane;

struct IngenicHelixJob {
    IngenicHelixState *s;       /* NULL once the device has reset */
    int tasks;                  /* in the pool: the coding, the statistics */
    int64_t start;              /* host ns at the start */
    bool jpeg;
    bool t32_dct;               /* the layout's, read off the thread */
    uint32_t mul[2][64];        /* JPEG: the quantizers, natural order */
    uint8_t shift[2][64];
    IngenicAvpuJpegHuff huff[4];
    IngenicAvpuPic pic;
    IngenicAvpuH264 *h264;
    uint32_t w, h;              /* whole macroblocks */
    uint8_t *y, *uv;            /* the source, @w apart; uv within y's */
    uint8_t *ref_y, *ref_uv;    /* P: the reference, as the ring holds it */
    uint8_t *rec_uv;            /* the reconstruction's chroma, NV12 */
    HelixPlane recon_y, recon_c;
    uint32_t bs, room;
    uint16_t thd[ACT_CLASSES - 1];
    GByteArray *data;
    IngenicAvpuStats st;
    uint64_t sad, sse;
    uint16_t hist[ACT_CLASSES];
    bool ok;
    /* JPEG decoding: the JPGC's registers, the scan from JPGC_BSA */
    bool dec;
    uint32_t glbi, nmcu, nrsm, cci[4];
    uint32_t hufm[16], hufb[64], hufs[512], qmem[256];
    uint8_t *scan;
    uint32_t scan_len;
    uint8_t *tiles_y, *tiles_c;
    uint32_t mcus, ytiles, ctiles;
    uint32_t end;               /* the fetch's end, from JPGC_BSA */
    bool bserr, full;           /* full: tile NMCU + 1 decoded */
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

static const IngenicHelixLayout ingenic_helix_t23 = {
    .window = true,
    .recon_y = ODMA_RECON_Y, .recon_c = ODMA_RECON_C,
    .ref_y = REF_Y, .ref_c = REF_C,
    .high = true,
    .jpeg_limit = true,
    .jpgc_e20 = 0xf0, .jpgc_e60 = 0x00250600,
};

static const IngenicHelixLayout ingenic_helix_t21 = {
    .window = true,
    .recon_y = ODMA_RECON_Y, .recon_c = ODMA_RECON_C,
    .ref_y = REF_Y, .ref_c = REF_C,
    .high = true,
    .cmpx = true,
};

static const IngenicHelixLayout ingenic_helix_t30 = {
    .recon_y = DBLK_RECON_Y, .recon_c = DBLK_RECON_C,
    .ref_y = MCE_REF_Y, .ref_c = MCE_REF_C,
    .cmpx = true,
    .busy = BIT(18),
    .jpgc_e20 = 0xf0, .jpgc_e60 = 0x001a0600,
    .glbc_mask = 0xaffffff7,
};

static const IngenicHelixLayout ingenic_helix_t10 = {
    .recon_y = DBLK_RECON_Y, .recon_c = DBLK_RECON_C,
    .ref_y = MCE_REF1_Y, .ref_c = MCE_REF1_C,
    .border = true,
    .cmpx = true,
    .busy = BIT(17),
};

/*
 * The T41's: the JPEG path alone, its EFE as unit 2 and JPGC as 3 (the AUX,
 * TCSM, SDE, DBLK and EMC all 7, none of which a JPEG job uses), its list
 * indexes the registers' addresses' low 20 bits, so 0x80000 above them
 */
static const uint8_t helix_t41_units[16] = {
    0x0, 0x1, 0x4, 0xe, 0xf, 0xf, 0xf, 0xf,
    0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
};

static const IngenicHelixLayout ingenic_helix_t41 = {
    .units = helix_t41_units,
    .list_base = 0x80000,
    .iosize = 256 * KiB,
    .t32_dct = true,
};

static const IngenicHelixLayout *helix_layout(IngenicHelixState *s)
{
    return INGENIC_HELIX_GET_CLASS(s)->layout;
}

/* a part's register offset, as the T23's */
static hwaddr helix_reg(IngenicHelixState *s, hwaddr off)
{
    const uint8_t *u = helix_layout(s)->units;

    return u ? (hwaddr)u[extract32(off, 16, 4)] << 16 | (off & 0xffff) : off;
}

static void helix_update(IngenicHelixState *s)
{
    qemu_set_irq(s->irq, ((R(SCH_STAT) & STAT_END) &&
                          (R(SCH_GLBC) & GLBC_INTE_END)) ||
                         ((R(SCH_STAT) & STAT_BSERR) &&
                          (R(SCH_GLBC) & GLBC_INTE_BSERR)));
}

/*
 * 0x34 reports the units' end flags: once the driver clears the SDE's
 * BSEND and the deblocker's DOEND (the codec's ENDF after a JPEG), the end,
 * the JPEG end (bit 4) and bit 9 go, and a T23 reads 0x100 between
 * pictures, a T10, T20 or T30 0. The T10's and T20's dispatcher can run
 * the handler a second time for one picture (it reads both pending banks
 * before servicing the first, whose softirqs take the encoder's
 * interrupt), and their handler completes on anything but 0, so that call
 * must find 0.
 */
static void helix_unit_end(IngenicHelixState *s)
{
    bool end = R(SCH_STAT) & STAT_JPEG ? R(JPGC_STAT) & JPGC_ENDF
               : (R(SDE_STAT) & SDE_BSEND) && (R(DBLK_GSTA) & DBLK_DOEND);

    if (!(R(DBLK_GSTA) & DBLK_DOEND)) {
        R(SCH_STAT) &= ~BIT(9);
    }
    if (!end) {
        R(SCH_STAT) &= ~(STAT_END | STAT_JPEG |
                         (helix_layout(s)->busy ? BIT(8) : 0));
    }
    helix_update(s);
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
    g_free(j->ref_y);
    g_free(j->ref_uv);
    g_free(j->rec_uv);
    g_free(j->scan);
    g_free(j->tiles_y);
    g_free(j->tiles_c);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

static uint32_t sad16(const uint8_t *a, int as, const uint8_t *b, int bs)
{
    uint32_t sum = 0;

    for (int y = 0; y < 16; y++, a += as, b += bs) {
        for (int x = 0; x < 16; x++) {
            sum += MAX(a[x], b[x]) - MIN(a[x], b[x]);
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

/* The statistics, beside the coding: they read only the source and reference */
static int helix_stats(void *opaque)
{
    helix_source_stats(opaque);
    return 0;
}

/*
 * The JPGC's view of a scan: FF 00 is a data FF, FF FF a fill byte and a
 * restart marker is dropped wherever it stands; any other marker ends the
 * data. Past their end it reads FF, then the data's last three bytes
 * again from the first, 0 for any before the data (a T23N's, as far as a
 * symbol and its value reach: FF and two bytes; the model repeats the
 * four). Data of fewer than four bytes run on through the first four of
 * those: the decoder takes them as data too.
 */
typedef struct JpgcReader {
    const uint8_t *bs;
    uint32_t len, pos;          /* pos: the next byte; at a marker, its FF */
    uint32_t acc;
    int n;                      /* bits in acc */
    bool marker;
    uint32_t loaded, used;      /* data bits taken from the scan, decoded */
    uint32_t raw[8];            /* where each of the last data bytes ends */
    uint8_t tail[3];            /* the last three data bytes, oldest first */
    uint32_t past;              /* bytes read past the data */
} JpgcReader;

static void jpgc_fill(JpgcReader *r)
{
    while (r->n <= 24) {
        bool data = false;
        uint8_t b;

        while (!r->marker && r->pos < r->len && !data) {
            uint8_t next = r->pos + 1 < r->len ? r->bs[r->pos + 1] : 0;

            if (r->bs[r->pos] != 0xff || !next) {
                b = r->bs[r->pos];
                r->pos += r->bs[r->pos] == 0xff ? 2 : 1;
                r->raw[r->loaded / 8 % 8] = r->pos;
                r->loaded += 8;
                r->tail[0] = r->tail[1];
                r->tail[1] = r->tail[2];
                r->tail[2] = b;
                data = true;
            } else if (next != 0xff && (next & 0xf8) != 0xd0) {
                r->marker = true;
                if (r->loaded < 32) {
                    r->loaded += 32;
                }
            } else {
                r->pos += next == 0xff ? 1 : 2;
            }
        }
        if (!data) {
            b = r->past % 4 ? r->tail[r->past % 4 - 1] : 0xff;
            r->past++;
        }
        r->acc = r->acc << 8 | b;
        r->n += 8;
    }
}

static uint32_t jpgc_peek(JpgcReader *r, int n)
{
    jpgc_fill(r);
    return (r->acc >> (r->n - n)) & MAKE_64BIT_MASK(0, n);
}

static void jpgc_skip(JpgcReader *r, int n)
{
    r->used += n;
    r->n -= n;
}

/* to the next byte of data, for a restart */
static void jpgc_align(JpgcReader *r)
{
    jpgc_skip(r, r->n % 8);
}

static int jpgc_extend(JpgcReader *r, int size)
{
    int v;

    if (!size) {
        return 0;
    }
    v = jpgc_peek(r, size);
    jpgc_skip(r, size);
    return v < (1 << (size - 1)) ? v - (1 << size) + 1 : v;
}

/*
 * A symbol from table @t (AC0, DC0, AC1, DC1): the code is the longest
 * whose length's minimum the next bits reach. HUFM holds the minima of
 * lengths 1-8 packed at their widths (word 3's low nibble, then word 2),
 * the low bytes of 9-12 in word 1 and of 13-16 in word 0, whose upper bits
 * are ones; HUFB the symbol index less the code, by length (9 bits);
 * HUFE the AC symbols, and DC0's in the low nibble, DC1's in the high one.
 * HUFE has 336 entries: an index past them reads the one 256 below.
 */
static int jpgc_symbol(const IngenicHelixJob *j, JpgcReader *r, int t)
{
    const uint32_t *m = j->hufm + 4 * t;
    uint64_t lo = (uint64_t)(m[3] & 0xf) << 32 | m[2];
    uint32_t code = jpgc_peek(r, 16), idx;
    int len = 0, sh = 36;

    for (int l = 1; l <= 16; l++) {
        uint32_t min;

        if (l <= 8) {
            sh -= l;
            min = extract64(lo, sh, l);
        } else {
            min = MAKE_64BIT_MASK(8, l - 8) |
                  extract32(l <= 12 ? m[1] : m[0], 8 * (3 - (l - 9) % 4), 8);
        }
        if (code >> (16 - l) >= min) {
            len = l;
        }
    }
    if (!len) {
        return 0;
    }
    jpgc_skip(r, len);
    idx = ((code >> (16 - len)) + j->hufb[16 * t + len - 1]) & 0x1ff;
    if (idx >= 336) {
        idx -= 256;
    }
    if (t & 1) {
        return extract32(j->hufs[idx], t == 3 ? 4 : 0, 4);
    }
    return j->hufs[idx] & 0xff;
}

typedef enum {
    JPGC_BLOCK,                 /* decoded */
    JPGC_DRY,                   /* the data end before it is */
    JPGC_BAD,                   /* a coefficient's place a multiple of 64 */
} JpgcBlock;

#define JPGC_END_PLACE      11  /* a block ends with the data from here */

/*
 * One block, with the tables and quantizer CCI @ci names, into coef[]
 * (natural order, dequantized and clamped to +-1023). Places count on past
 * 63 when a run skips 64: those coefficients go to the next block
 * (coef[1]...), which comes out without a DC as the next one in the MCU's
 * order, its component's prediction taking the value that crossed; the
 * block goes on to its EOB, or to a multiple of 64. The DC prediction is
 * 11 bits, and a sum that would read -1024 holds the prediction's sign at
 * +-1023 instead.
 *
 * At the end of the data a block whose DC is whole comes out if the
 * coefficient it stands at, the one whose symbol or value runs past the
 * data or else the last decoded, has a place of 11 or more; the rest of
 * its places then hold that coefficient's value times the quantizer of
 * the place after it. A symbol past the data decodes from what the
 * reader gives there, see JpgcReader.
 */
static JpgcBlock jpgc_block(const IngenicHelixJob *j, JpgcReader *r,
                            uint32_t ci, int *pred,
                            int32_t coef[JPGC_SPILL + 1][64], int *cross,
                            int *spill)
{
    const uint32_t *q = j->qmem + 64 * extract32(ci, 2, 2);
    int s = jpgc_symbol(j, r, 2 * (ci & 1) + 1), k = 1, p, last = 0;
    int from = 64;
    int64_t fill = 0;
    JpgcBlock st = JPGC_BLOCK;

    *spill = 0;
    memset(coef, 0, sizeof(int32_t) * 64 * (JPGC_SPILL + 1));
    p = ((*pred + jpgc_extend(r, s) + 1024) & 2047) - 1024;
    if (r->used > r->loaded) {
        return JPGC_DRY;
    }
    *pred = p == -1024 ? (*pred < 0 ? -1023 : 1023) : p;
    coef[0][0] = *pred;
    for (;;) {
        int rs, pos, v = 0;

        jpgc_fill(r);
        if (r->used == r->loaded) {
            if (k - 1 < JPGC_END_PLACE) {
                return JPGC_DRY;
            }
            from = k % 64;
            fill = (int64_t)last * q[k % 64];
            break;
        }
        rs = jpgc_symbol(j, r, 2 * extract32(ci, 1, 1));
        if (!(rs & 0xf) && rs >> 4 != 0xf) {
            break;
        }
        pos = k + (rs >> 4);
        if (rs & 0xf) {
            v = jpgc_extend(r, rs & 0xf);
        }
        if (r->used > r->loaded) {
            if (pos < JPGC_END_PLACE) {
                return JPGC_DRY;
            }
            if (pos % 64 == 0 || pos / 64 > JPGC_SPILL) {
                st = JPGC_BAD;
            } else if (pos % 64 >= JPGC_END_PLACE) {
                if (pos / 64 > *spill) {
                    cross[++*spill - 1] = v;
                }
                coef[*spill][zigzag[pos % 64]] = v;
                from = pos % 64 + 1;
                fill = (int64_t)v * q[MIN(pos % 64 + 1, 63)];
            }
            break;
        }
        last = v;
        if (pos % 64 == 0 || pos / 64 > JPGC_SPILL) {
            st = JPGC_BAD;
            break;
        }
        if (pos / 64 > *spill) {
            cross[++*spill - 1] = v;
        }
        coef[pos / 64][zigzag[pos % 64]] = v;
        k = pos + 1;
        if (k % 64 == 0) {
            break;
        }
    }
    for (int b = 0; b <= *spill; b++) {
        for (k = 0; k < 64; k++) {
            int64_t d = (int64_t)coef[b][zigzag[k]] * q[k];

            coef[b][zigzag[k]] = MIN(MAX(d, -1023), 1023);
        }
    }
    for (k = from; k < 64; k++) {
        coef[*spill][zigzag[k]] = MIN(MAX(fill, -1023), 1023);
    }
    return st;
}

/* Block @slot of the output: 4 of luma (16x16), then U and V (8 rows each) */
static void jpgc_put(IngenicHelixJob *j, uint32_t slot, int32_t *coef)
{
    uint32_t t = slot / 6, b = slot % 6;

    if (b < 4) {
        ingenic_avpu_jpeg_idct_t32(coef, j->tiles_y + 256 * t +
                                   (b / 2) * 128 + (b % 2) * 8, 16);
    } else {
        ingenic_avpu_jpeg_idct_t32(coef, j->tiles_c + 128 * t +
                                   (b - 4) * 8, 16);
    }
}

/*
 * A scan in the JPGC's SPEC mode. An MCU is NBLK + 1 blocks (CCI bits
 * 7:4) of each component up to NCOL, which use their CCI's tables and
 * keep a prediction each; the blocks go out in order as 4:2:0 tiles
 * whatever the sampling, six to a tile: a 256-byte luma tile (16x16) and
 * a 128-byte chroma one (8 rows of 8 U, 8 V), each written once its
 * blocks are all decoded. A restart (GLBI bit 2, every NRSM MCUs) drops
 * the bits to the next byte and clears the predictions; restart markers
 * play no part. The data stop at a marker: a block whose DC needs bits
 * past it ends the scan, and so does one that stands before place 11
 * there; the rest are finished, see jpgc_block(). Up to NMCU + 1 tiles
 * are decoded: a block past them, or a coefficient whose place is a
 * multiple of 64, is a bitstream error, after which every block to the
 * end of tile NMCU + 1 is zero (reads 128). A scan whose data end with
 * tile NMCU + 1 ends with both.
 */
static void helix_jpgc_decode(IngenicHelixJob *j)
{
    JpgcReader r = { .bs = j->scan, .len = j->scan_len };
    uint32_t slots = 6 * (j->nmcu + 1), slot = 0, e;
    int ncol = extract32(j->glbi, 4, 2), pred[4] = { 0 }, n = 0;
    uint8_t seq[64];
    JpgcBlock st = JPGC_BLOCK;

    for (int c = 0; c <= ncol; c++) {
        for (int b = 0; b <= extract32(j->cci[c], 4, 4); b++) {
            seq[n++] = c;
        }
    }
    for (uint32_t g = 0; st == JPGC_BLOCK && slot < slots; g++) {
        int32_t coef[JPGC_SPILL + 1][64];
        int cross[JPGC_SPILL], spill, c = seq[g % n];

        if ((j->glbi & GLBI_RSM) && g % n == 0 && g &&
            (g / n) % j->nrsm == 0) {
            jpgc_align(&r);
            memset(pred, 0, sizeof(pred));
        }
        st = jpgc_block(j, &r, j->cci[c], &pred[c], coef, cross, &spill);
        if (st == JPGC_DRY) {
            break;
        }
        for (int b = 0; b <= spill && slot < slots; b++) {
            if (b) {
                g++;
                pred[seq[g % n]] += cross[b - 1];
            }
            jpgc_put(j, slot++, coef[b]);
        }
        if (st == JPGC_BLOCK && slot == slots) {
            /* one more block decoded is one too many */
            st = jpgc_block(j, &r, j->cci[seq[(g + 1) % n]],
                            &pred[seq[(g + 1) % n]], coef, cross, &spill);
            j->full = st == JPGC_DRY;
            st = j->full ? st : JPGC_BAD;
        }
    }
    j->bserr = st == JPGC_BAD;
    while (j->bserr && slot < slots) {
        int32_t zero[64] = { 0 };

        jpgc_put(j, slot++, zero);
    }
    j->mcus = slot / 6;
    j->ytiles = j->mcus + (slot % 6 >= 4);
    j->ctiles = j->mcus;
    /* the marker that ends the scan, or where the fetch stood at an error */
    for (e = r.pos; e + 1 < r.len; e++) {
        if (r.bs[e] == 0xff && r.bs[e + 1] && r.bs[e + 1] != 0xff &&
            (r.bs[e + 1] & 0xf8) != 0xd0) {
            break;
        }
    }
    j->end = e + 4;
    if (j->bserr) {
        uint32_t used = DIV_ROUND_UP(MIN(r.used, r.loaded), 8);

        j->end = (used ? r.raw[(used - 1) % 8] : 0) + 45;
    }
}

static int helix_work(void *opaque)
{
    IngenicHelixJob *j = opaque;
    uint32_t cw = j->w / 2;
    uint8_t *const *ref;

    if (j->dec) {
        helix_jpgc_decode(j);
        return 0;
    }
    if (j->jpeg) {
        (j->t32_dct ? ingenic_avpu_jpeg_scan_t32
         : ingenic_avpu_jpeg_scan)(j->y, j->w, j->uv, j->w, j->w, j->h,
                                   j->mul, j->shift, j->huff, 0, j->data);
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
    j->ok = ingenic_avpu_h264_encode(j->h264, &j->pic, j->y, j->uv, j->w,
                                     j->data, &j->st);
    if (!j->ok) {
        return 0;
    }
    /* the reconstruction: luma stays in the core, chroma back to NV12 */
    ref = ingenic_avpu_h264_ref(j->h264);
    for (size_t r = 0; r < j->h; r++) {
        const uint8_t *a = j->y + r * j->w, *b = ref[0] + r * j->w;
        uint32_t row = 0;

        for (uint32_t x = 0; x < j->w; x += 16) {
            for (int i = 0; i < 16; i++) {
                int d = a[x + i] - b[x + i];

                row += d * d;
            }
        }
        j->sse += row;
    }
    j->rec_uv = g_malloc((size_t)j->w * j->h / 2);
    for (size_t i = 0; i < (size_t)cw * j->h / 2; i += 8) {
        for (int k = 0; k < 8; k++) {
            j->rec_uv[2 * (i + k)] = ref[1][i + k];
            j->rec_uv[2 * (i + k) + 1] = ref[2][i + k];
        }
    }
    return 0;
}

/*
 * Back to the device, for the next picture of its size: the coder keeps
 * buffers and nothing a picture takes from the last
 */
static void helix_keep_coder(IngenicHelixState *s, IngenicHelixJob *j)
{
    int i = 0;

    while (i < INGENIC_HELIX_CODERS - 1 && s->coders[i]) {
        i++;
    }
    ingenic_avpu_h264_free(s->coders[i]);
    s->coders[i] = j->h264;
    j->h264 = NULL;
}

static void helix_done(void *opaque, int ret)
{
    IngenicHelixJob *j = opaque;
    IngenicHelixState *s = j->s;
    uint32_t len, i4;

    if (--j->tasks) {
        return;
    }
    if (!s) {
        helix_job_free(j);
        return;
    }
    if (j->dec) {
        trace_ingenic_helix_jpeg_decode(j->mcus, j->nmcu, j->end,
                                        (get_clock() - j->start) / 1000,
                                        j->bserr);
        address_space_write(&address_space_memory, R(JPGC_P0A),
                            MEMTXATTRS_UNSPECIFIED, j->tiles_y,
                            256 * j->ytiles);
        address_space_write(&address_space_memory, R(JPGC_P1A),
                            MEMTXATTRS_UNSPECIFIED, j->tiles_c,
                            128 * j->ctiles);
        /* the fetch runs a burst past the one holding its end */
        R(JPGC_BSA) += ROUND_UP(j->end, 128) + 128;
        R(JPGC_P0A) += 256 * j->ytiles;
        R(JPGC_P1A) += 128 * j->ctiles;
        R(JPGC_MCUS) = j->mcus;
        R(JPGC_STAT) = (j->bserr ? 0 : JPGC_ENDF) |
                       (j->bserr || j->full ? JPGC_ERRF : 0);
        R(VDMA_TASKST) = TASKST_END;
        R(SCH_STAT) = (j->bserr || j->full ? STAT_BSERR : 0) |
                      (j->bserr ? 0 : STAT_END | STAT_JPEG);
        s->job = NULL;
        helix_update(s);
        helix_job_free(j);
        return;
    }
    trace_ingenic_helix_end(j->jpeg, j->w, j->h,
                            (get_clock() - j->start) / 1000,
                            j->st.intra_mbs, j->st.skip_mbs, j->data->len);
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
        helix_plane_io(&j->recon_y, ingenic_avpu_h264_ref(j->h264)[0], j->w,
                       j->h, true);
        helix_plane_io(&j->recon_c, j->rec_uv, j->w, j->h / 2, true);
    }

    i4 = j->st.intra_mbs - j->st.i16_mbs - j->st.i8_mbs;
    R(SDE_BS_LEN) = len;
    R(VMAU_INTRA) = j->st.i16_mbs | i4 << 16;
    R(VMAU_INTRA8) = j->st.i8_mbs | j->st.inter_mbs << 16;
    R(VMAU_INTER) = j->st.skip_mbs << 16;
    R(VMAU_MBS) = (j->w / 16) * (j->h / 16);
    R(VMAU_SAD) = MIN(j->sad, UINT32_MAX);
    if (helix_layout(s)->cmpx) {
        R(EFE_SSAD) = R(VMAU_SAD);
    }
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
    helix_keep_coder(s, j);
    helix_job_free(j);
}

/* A job under way, in the pool */
static void helix_start(IngenicHelixState *s, IngenicHelixJob *j)
{
    s->job = j;
    j->start = get_clock();
    j->tasks = j->jpeg || j->dec ? 1 : 2;
    R(VDMA_TASKST) = TASKST_BUSY;
    R(SCH_STAT) |= helix_layout(s)->busy;
    if (!j->jpeg && !j->dec) {
        thread_pool_submit_aio(helix_stats, j, helix_done, j);
    }
    thread_pool_submit_aio(helix_work, j, helix_done, j);
}

/* The picture to code, from memory or over IVDC */
static void helix_source(IngenicHelixState *s, IngenicHelixJob *j,
                         uint32_t ystride, uint32_t cstride)
{
    if (R(EFE_RAWY) & IVDC_ADDR) {
        /* ingenic_isp_render() writes every byte */
        j->y = s->isp ? g_malloc((size_t)j->w * j->h * 3 / 2)
                      : g_malloc0((size_t)j->w * j->h * 3 / 2);
        j->uv = j->y + (size_t)j->w * j->h;
        if (s->isp) {
            ingenic_isp_render(s->isp, j->w, j->h, j->y);
        }
        return;
    }
    j->y = g_malloc0((size_t)j->w * j->h * 3 / 2);
    j->uv = j->y + (size_t)j->w * j->h;
    helix_plane_io(&(HelixPlane) { R(EFE_RAWY), 0, 0, ystride ?: j->w },
                   j->y, j->w, j->h, false);
    helix_plane_io(&(HelixPlane) { R(EFE_RAWC), 0, 0, cstride ?: j->w },
                   j->uv, j->w, j->h / 2, false);
}

static void helix_encode(IngenicHelixState *s)
{
    const IngenicHelixLayout *l = helix_layout(s);
    IngenicHelixJob *j = g_new0(IngenicHelixJob, 1);
    uint32_t ctrl = R(EFE_CTRL), geom = R(EFE_GEOM);
    uint32_t ystride = R(EFE_RAW_STRD) >> 16;
    uint32_t cstride = R(EFE_RAW_STRD) & 0xffff;
    bool odma = l->recon_y == ODMA_RECON_Y;
    bool ring = odma && R(ODMA_CTRL) & ODMA_RING;
    uint32_t stride = odma ? R(ODMA_STRIDE) >> 16 : 0, border = 0;

    j->s = s;
    j->w = (extract32(geom, 0, 8) + 1) * 16;
    j->h = (extract32(geom, 8, 8) + 1) * 16;
    j->data = g_byte_array_new();
    if (l->window) {
        j->bs = R(EMC_BS_ADDR);
        j->room = R(EMC_BS_SIZE) * KiB;
    } else {
        j->bs = R(SDE_BS_ADDR);
        j->room = j->w * j->h * 3 / 2;
    }
    for (int c = 0; c < ACT_CLASSES - 1; c++) {
        j->thd[c] = R(EFE_ACT_THD + 4 * (c / 2)) >> (16 * (c & 1));
    }

    j->pic.width = j->w;
    j->pic.height = j->h;
    j->pic.qp = MIN(extract32(ctrl, 8, 6), 51);
    j->pic.intra = !(ctrl & CTRL_P);
    j->pic.cabac = true;
    if (l->high) {
        j->pic.transform_8x8 = true;
        j->pic.scale8[0] = helix_scale8[0];
        j->pic.scale8[1] = helix_scale8[1];
    }
    /* the T20's libimp turns deblocking off in the slice header and here */
    j->pic.no_deblock = !l->window && !(R(DBLK_CTRL) & DBLK_EN);
    j->pic.chroma_qp_offset = sextract32(R(EFE_CQP_OFST), 0, 5);
    j->pic.cr_qp_offset = j->pic.chroma_qp_offset;
    j->pic.raw = true;
    memcpy(j->pic.lambda, helix_lambda, sizeof(helix_lambda));
    for (int i = 0; i < INGENIC_HELIX_CODERS && !j->h264; i++) {
        if (s->coders[i] && ingenic_avpu_h264_matches(s->coders[i], &j->pic)) {
            j->h264 = s->coders[i];
            s->coders[i] = NULL;
        }
    }
    if (!j->h264) {
        j->h264 = ingenic_avpu_h264_new(&j->pic);
    }

    if (!stride) {
        stride = j->w;
    }
    if (l->border) {
        border = (j->w / 16 + 3) * 256;
    }
    j->recon_y = (HelixPlane) {
        R(l->recon_y) + border, ring ? R(ODMA_RING_Y) : 0,
        ring ? R(ODMA_RING_Y_END) : 0, stride,
    };
    j->recon_c = (HelixPlane) {
        R(l->recon_c) + border / 2, ring ? R(ODMA_RING_C) : 0,
        ring ? R(ODMA_RING_C_END) : 0, stride,
    };

    trace_ingenic_helix_start(j->w, j->h, !j->pic.intra, R(EFE_RAWY),
                              R(l->ref_y), j->recon_y.addr);
    helix_source(s, j, ystride, cstride);
    if (!j->pic.intra) {
        HelixPlane ry = j->recon_y, rc = j->recon_c;

        ry.addr = R(l->ref_y);
        rc.addr = R(l->ref_c);
        j->ref_y = g_malloc((size_t)j->w * j->h);
        j->ref_uv = g_malloc((size_t)j->w * j->h / 2);
        helix_plane_io(&ry, j->ref_y, j->w, j->h, false);
        helix_plane_io(&rc, j->ref_uv, j->w, j->h / 2, false);
    }
    helix_start(s, j);
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
    uint32_t ystride = R(EFE_RAW_STRD) >> 16;
    uint32_t cstride = R(EFE_RAW_STRD) & 0xffff;

    j->s = s;
    j->jpeg = true;
    j->t32_dct = helix_layout(s)->t32_dct;
    j->w = (extract32(geom, 0, 16) + 1) * 16;
    j->h = (extract32(geom, 16, 16) + 1) * 16;
    j->data = g_byte_array_new();
    if (j->w > JPEG_MAX || j->h > JPEG_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-helix: a %ux%u JPEG\n",
                      j->w, j->h);
        s->job = j;
        j->tasks = 1;
        helix_done(j, 0);
        return;
    }
    j->bs = R(JPGC_BSA);
    /* T23 libimp enables the limit with its pool size, which may be 0 */
    j->room = helix_layout(s)->jpeg_limit && max & MAX_BS_EN &&
              max & ~MAX_BS_EN ? max & ~MAX_BS_EN : UINT32_MAX;
    for (int t = 0; t < 2; t++) {
        for (int k = 0; k < 64; k++) {
            uint32_t e = R(JPGC_QMEM + 4 * (64 * t + k));
            bool t32 = helix_layout(s)->t32_dct;

            /* the T32's DCT gives the coefficients, the EL200's 8 times */
            j->mul[t][zigzag[k]] = e > 1 ? e & 0x7ff : t32 ? e << 1 : e;
            j->shift[t][zigzag[k]] = e > 1 ? (t32 ? 11 : 14) +
                                     extract32(e, 11, 3) : t32 ? 1 : 3;
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

    helix_source(s, j, ystride, cstride);
    helix_start(s, j);
}

/*
 * A decoding job: the JPGC's tables as they stand, and the scan from
 * JPGC_BSA up to the marker that ends it (a restart's does not)
 */
static void helix_jpgc_start(IngenicHelixState *s)
{
    IngenicHelixJob *j = g_new0(IngenicHelixJob, 1);
    uint32_t cap, len = 0;
    bool end = false;

    j->s = s;
    j->dec = true;
    j->glbi = R(JPGC_GLBI);
    j->nmcu = R(JPGC_NMCU) + 1;
    j->nrsm = R(JPGC_NRSM) + 1;
    if (j->nmcu > (JPEG_MAX / 16) * (JPEG_MAX / 16)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-helix: decoding %u MCUs, only "
                      "the first %u\n", j->nmcu,
                      (JPEG_MAX / 16) * (JPEG_MAX / 16));
        j->nmcu = (JPEG_MAX / 16) * (JPEG_MAX / 16);
    }
    for (int c = 0; c < 4; c++) {
        j->cci[c] = R(JPGC_CCI + 4 * c);
    }
    memcpy(j->hufm, &R(JPGC_HUFM), sizeof(j->hufm));
    memcpy(j->hufb, &R(JPGC_HUFB), sizeof(j->hufb));
    memcpy(j->hufs, &R(JPGC_HUFE), sizeof(j->hufs));
    memcpy(j->qmem, &R(JPGC_QMEM), sizeof(j->qmem));
    /* six blocks a tile, each at most 27 + 63 * 26 bits, all stuffed */
    cap = MIN((j->nmcu + 2) * 2560 + 4096, 64 * MiB);
    while (len < cap && !end) {
        uint32_t n = MIN(64 * KiB, cap - len);

        j->scan = g_realloc(j->scan, len + n);
        memset(j->scan + len, 0, n);
        address_space_read(&address_space_memory, R(JPGC_BSA) + len,
                           MEMTXATTRS_UNSPECIFIED, j->scan + len, n);
        for (uint32_t i = len ? len - 1 : 0; i + 1 < len + n && !end; i++) {
            end = j->scan[i] == 0xff && j->scan[i + 1] &&
                  j->scan[i + 1] != 0xff && (j->scan[i + 1] & 0xf8) != 0xd0;
        }
        len += n;
    }
    j->scan_len = len;
    j->tiles_y = g_malloc(256 * (j->nmcu + 1));
    j->tiles_c = g_malloc(128 * (j->nmcu + 1));
    helix_start(s, j);
}

/* A register as written: the JPGC's keep only some bits */
static void helix_store(IngenicHelixState *s, hwaddr off, uint32_t v)
{
    switch (off) {
    case JPGC_TRIG:
    case JPGC_E20:
    case JPGC_GLBI_RD:
    case JPGC_E60:
    case JPGC_MCUS:
        return;
    case SCH_GLBC:
        v &= helix_layout(s)->glbc_mask ?: UINT32_MAX;
        break;
    case SCH_TLBA:
        v &= ~0xfffu;
        break;
    case SCH_TLBC:
        v &= ~0xfu;
        break;
    case SCH_TLBV:
        v &= 0xcfff0fffu;
        break;
    case JPGC_GLBI:
        if ((v & (GLBI_CLR | GLBI_EN)) == (GLBI_CLR | GLBI_EN)) {
            v &= ~MAKE_64BIT_MASK(2, 4);
        }
        v &= GLBI_MASK;
        break;
    case JPGC_STAT:
        v &= JPGC_STAT_MASK;
        break;
    case JPGC_P0A:
    case JPGC_P1A:
    case JPGC_P2A:
    case JPGC_P3A:
        v &= ~7u;
        break;
    case JPGC_NMCU:
        v &= MAKE_64BIT_MASK(0, 26);
        break;
    case JPGC_NRSM:
        v &= 0xffff;
        break;
    case JPGC_CCI ... JPGC_CCI + 12:
        v &= 0xff;
        break;
    }
    R(off) = v;
}

/*
 * GLBI reads its fields 5:4 and 3:2 swapped: NCOL in 3:2, DEC and RSM in
 * 5:4; 0xe0024 reads those four bits
 */
static uint32_t helix_glbi_read(IngenicHelixState *s)
{
    uint32_t v = R(JPGC_GLBI);

    return (v & ~MAKE_64BIT_MASK(2, 4)) | extract32(v, 2, 2) << 4 |
           extract32(v, 4, 2) << 2;
}

/* Replay a command list into the registers; its last write may start */
static void helix_run(IngenicHelixState *s, uint32_t list)
{
    uint32_t pair[2], ctrl = 0, addr = list;
    bool efe = false, jpgc = false;
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
        if ((ctrl & CL_VLD) && (ctrl & CL_REG) >= helix_layout(s)->list_base) {
            hwaddr off = helix_reg(s, (ctrl & CL_REG) -
                                      helix_layout(s)->list_base);
            uint32_t v = le32_to_cpu(pair[0]);

            efe |= off == EFE_CTRL && v & CTRL_RUN;
            jpgc |= off == JPGC_TRIG && v & (TRIG_BS | TRIG_PP);
            helix_store(s, off, v);
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
    if (!efe && jpgc) {
        /* the JPGC alone: decoding, in SPEC mode (tiled 4:2:0) */
        if ((R(JPGC_GLBI) & (GLBI_EN | GLBI_UNIV | GLBI_DEC)) ==
            (GLBI_EN | GLBI_DEC)) {
            helix_jpgc_start(s);
            return;
        }
        qemu_log_mask(LOG_UNIMP, "ingenic-helix: the JPGC alone only "
                      "decodes (0xe0004 = 0x%08x)\n", R(JPGC_GLBI));
        R(JPGC_STAT) = JPGC_ENDF;
        R(VDMA_TASKST) = TASKST_END;
        R(SCH_STAT) = STAT_END | STAT_JPEG;
        helix_update(s);
        return;
    }
    if (!efe) {
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

    addr = helix_reg(s, addr);
    switch (addr) {
    case JPGC_GLBI:
        return helix_glbi_read(s);
    case JPGC_GLBI_RD:
        return extract32(helix_glbi_read(s), 2, 4);
    case JPGC_E20:
        return helix_layout(s)->jpgc_e20;
    case JPGC_E60:
        return helix_layout(s)->jpgc_e60;
    }
    return R(addr);
}

static void ingenic_helix_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    IngenicHelixState *s = INGENIC_HELIX(opaque);
    uint32_t v = value;

    addr = helix_reg(s, addr);
    switch (addr) {
    case SCH_GLBC:
        helix_store(s, addr, v);
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
    case SDE_STAT:
    case DBLK_GSTA:
    case JPGC_STAT:
        helix_store(s, addr, v);
        helix_unit_end(s);
        return;
    }
    helix_store(s, addr, v);
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

/*
 * The CPM's soft reset (SRBC), which the kernel pulses before each job: it
 * clears SCH_STAT, and with it the last job's end, which would otherwise
 * complete the next job as soon as its interrupt is enabled. The TLB's
 * base and control registers keep their values through it.
 */
static void helix_soft_reset(void *opaque, int n, int level)
{
    IngenicHelixState *s = INGENIC_HELIX(opaque);

    if (level) {
        uint32_t tlba = R(SCH_TLBA), tlbc = R(SCH_TLBC);

        ingenic_helix_reset_hold(OBJECT(opaque), RESET_TYPE_COLD);
        R(SCH_TLBA) = tlba;
        R(SCH_TLBC) = tlbc;
        helix_update(s);
    }
}

static void ingenic_helix_init(Object *obj)
{
    IngenicHelixState *s = INGENIC_HELIX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->regs = g_malloc0(INGENIC_HELIX_IOSIZE);
    memory_region_init_io(&s->iomem, obj, &ingenic_helix_ops, s,
                          TYPE_INGENIC_HELIX,
                          helix_layout(s)->iosize ?: INGENIC_HELIX_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), helix_soft_reset, "reset", 1);
}

static void ingenic_helix_finalize(Object *obj)
{
    IngenicHelixState *s = INGENIC_HELIX(obj);

    for (int i = 0; i < INGENIC_HELIX_CODERS; i++) {
        ingenic_avpu_h264_free(s->coders[i]);
    }
    g_free(s->regs);
}

/* An encode in flight is host state */
static const VMStateDescription vmstate_ingenic_helix = {
    .name = TYPE_INGENIC_HELIX,
    .unmigratable = 1,
};

static const Property ingenic_helix_props[] = {
    DEFINE_PROP_LINK("isp", IngenicHelixState, isp, TYPE_INGENIC_ISP,
                     IngenicIspState *),
};

static void ingenic_helix_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic HELIX video encoder and JPEG decoder";
    device_class_set_props(dc, ingenic_helix_props);
    dc->vmsd = &vmstate_ingenic_helix;
    rc->phases.hold = ingenic_helix_reset_hold;
    rc->phases.exit = ingenic_helix_reset_exit;
    INGENIC_HELIX_CLASS(oc)->layout = &ingenic_helix_t23;
}

static void ingenic_t10_helix_class_init(ObjectClass *oc, const void *data)
{
    INGENIC_HELIX_CLASS(oc)->layout = &ingenic_helix_t10;
}

static void ingenic_t21_helix_class_init(ObjectClass *oc, const void *data)
{
    INGENIC_HELIX_CLASS(oc)->layout = &ingenic_helix_t21;
}

static void ingenic_t30_helix_class_init(ObjectClass *oc, const void *data)
{
    INGENIC_HELIX_CLASS(oc)->layout = &ingenic_helix_t30;
}

static void ingenic_t41_helix_class_init(ObjectClass *oc, const void *data)
{
    INGENIC_HELIX_CLASS(oc)->layout = &ingenic_helix_t41;
}

static const TypeInfo ingenic_helix_types[] = {
    {
        .name = TYPE_INGENIC_HELIX,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicHelixState),
        .instance_init = ingenic_helix_init,
        .instance_finalize = ingenic_helix_finalize,
        .class_size = sizeof(IngenicHelixClass),
        .class_init = ingenic_helix_class_init,
    }, {
        .name = TYPE_INGENIC_T10_HELIX,
        .parent = TYPE_INGENIC_HELIX,
        .class_init = ingenic_t10_helix_class_init,
    }, {
        .name = TYPE_INGENIC_T21_HELIX,
        .parent = TYPE_INGENIC_HELIX,
        .class_init = ingenic_t21_helix_class_init,
    }, {
        .name = TYPE_INGENIC_T30_HELIX,
        .parent = TYPE_INGENIC_HELIX,
        .class_init = ingenic_t30_helix_class_init,
    }, {
        .name = TYPE_INGENIC_T41_HELIX,
        .parent = TYPE_INGENIC_HELIX,
        .class_init = ingenic_t41_helix_class_init,
    },
};

DEFINE_TYPES(ingenic_helix_types)
