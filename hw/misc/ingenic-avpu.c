/*
 * Ingenic AVPU: the Allegro video encoder of the T31, T40 and T41
 *
 * libimp runs Allegro's encoder control software in user space and reaches
 * the AVPU through the avpu driver's register ioctls. The T31's and T40's
 * EL150 is described here; the T41's EL200 has the same cores with its
 * registers elsewhere (ingenic_avpu_el200). The registers sit at 0x8000:
 * the IP's identity, the interrupt mask (0x8014) and the pending
 * interrupts (0x8018, write 1 to clear), and per core (video 0, JPEG 1)
 * at 0x8200 + core * 0x200 a register zone, the command list address
 * (+0x1e0), the start (+0x1e4: 2 runs the command list, 8 its entropy pass,
 * 1 the JPEG zone), a reset (+0x1f0), a clock gate (+0x1f4) and the busy
 * flags (+0x1f8).
 *
 * A command list is a run of 512-byte entries, one per slice, the last
 * flagged in bit 31 of its first word. The T31's libimp asks for the whole
 * encode in the first pass: the entry's first 0x100 bytes say what to
 * encode, and the AVPU writes the slice data into the stream buffer at the
 * entry's offset, then fills the entry's status words (0x40-0x5f) and
 * raises interrupt core * 4. For H.264 libimp has put the slice header in
 * front already, and as a CAVLC one need not end on a byte, it puts only
 * the header's first bytes there and hands the AVPU the bits after them,
 * which it writes first: how many in bits 28:24 of entry word 0x1e, the
 * bits themselves in word 0x1f. For H.265 libimp writes its headers in
 * front afterwards. The layout is libimp's (its CmdRegsEnc1ToSliceParam
 * and EncodingStatusRegsToSliceStatus) checked against a real T31.
 *
 * The AVPU picks the QP of each macroblock (H.264) or CTB (H.265) itself.
 * It scores a block by the absolute differences between each luma sample
 * and its right and lower neighbours, over all but the block's last row
 * and column, the picture extended by repeating its last row and column.
 * With auto QP on (entry word 9 bit 26), a score below the u16 thresholds
 * of the EP2 header (at entry word 0x23) at 4, 2 or 0, tested in that
 * order, lowers the slice QP by header byte 14, 13 or 12, and one above
 * those at 10, 8 or 6 raises it by byte 17, 16 or 15; the thresholds are
 * for 16x16 and scale with the block's area. The QP then stays within
 * bytes 18 and 19. Status word 0x4c sums the scores, 0x4d the QPs of the
 * 8x8 blocks, 0x4e holds the largest and smallest. The AVPU's own rate
 * control (entry words 0x14-0x18, word 9 bit 16) never moved a block's QP
 * on a real T31, and is not modelled.
 *
 * The JPEG core takes its job from its zone: 0x00 the mode (bits 1:0, 1 for
 * 4:2:0, 13:12 APP0's density units), 0x04 the width and height less one
 * (bits 31:16, 15:0), 0x08 APP0's x and y density (same halves), 0x0c the
 * stride, 0x10 and 0x14 the NV12 planes, 0x18 the table buffer (luma and
 * chroma quantization first, zigzag order), 0x1c the output buffer, 0x24
 * the offset to write at and 0x28 the room there. It reports at 0x30 how
 * many Huffman codes the scan holds (a real T31's count is of the same
 * order; libimp only traces it), at 0x34 the bytes written, and in bit 1
 * of 0x38 an overflow.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "block/thread-pool.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-avpu.h"

#define AVPU_UID            0x00    /* from the layout's base */
#define AVPU_HW_ID          0x04
#define AVPU_DATE           0x08
#define AVPU_REV            0x0c
#define AVPU_AXI            0x10
#define AVPU_INT_MASK       0x14
#define AVPU_INT            0x18
#define AVPU_TOP_CTRL       0x54

#define START_ENC1          2
#define START_ENC2          8
#define START_JPEG          1

#define JPEG_OVERFLOW       BIT(1)

#define CL_LAST             BIT(31)
#define CL_ENTRY_MAX        0x1000
#define CL_STATUS           0x40    /* the EL150's status words, in an entry */
#define CL_STATUS_WORDS     0x20

#define EP2_HEADER          20      /* auto QP's thresholds, steps, range */

/* A JPEG core's registers, in its zone; codes and status 0 for none */
typedef struct AvpuJpegRegs {
    hwaddr mode, size, ystride, cstride, y, uv, tables;
    hwaddr out, out_off, room;      /* where the file goes */
    hwaddr density;                 /* APP0's, x << 16 | y */
    hwaddr codes, bytes, status;
} AvpuJpegRegs;

/*
 * Where a model has its registers, from its base: the IP's, then a zone per
 * core with the command list address, the start and a busy flag, core 1
 * the JPEG core. The T31's EL150 also has a clock gate that reads back with
 * bit 16; the T41's EL200 takes the address the status words go to.
 */
struct IngenicAvpuLayout {
    hwaddr base;
    hwaddr core, core_stride;
    hwaddr cl_addr, start, reset, clock, busy, status;
    uint32_t busy_idle, busy_running;
    unsigned cores;
    unsigned irq_stride;            /* interrupt bits per core */
    size_t entry;                   /* bytes of a command list entry */
    void (*decode)(IngenicAvpuJob *j);
    const AvpuJpegRegs *jpeg;
};

#define R(off)              (s->regs[(off) / 4])
#define CORE(l, c, reg)     ((l)->core + (c) * (l)->core_stride + (l)->reg)

/* A video core's encoder state, by the channel buffer in entry word 0x23 */
typedef struct AvpuChan {
    IngenicAvpuH264 *h264;
} AvpuChan;

struct IngenicAvpuJob {
    IngenicAvpuState *s;        /* NULL once the device has reset */
    int core;
    bool jpeg;
    uint8_t units;              /* APP0's density */
    uint16_t xdensity, ydensity;
    uint8_t qt[2][64];
    uint16_t recip[2][64];
    uint32_t codes;
    uint32_t cl;
    uint32_t w[CL_ENTRY_MAX / 4];
    IngenicAvpuPic pic;
    uint32_t chan;
    uint32_t src_y, src_uv, ep2_addr;
    uint32_t stream, offset, room;  /* where the slice data goes */
    IngenicAvpuH264 *h264;      /* the channel's, while this encodes */
    uint8_t *y, *uv;
    uint32_t stride, cstride;
    uint8_t ep2[EP2_HEADER];
    uint8_t *qps;               /* by block, see IngenicAvpuPic.block_qp */
    uint32_t score, qp_sum;
    int qp_min, qp_max;
    GByteArray *data;
    IngenicAvpuStats st;
    bool ok;
};

static const IngenicAvpuLayout *avpu_layout(IngenicAvpuState *s)
{
    return INGENIC_AVPU_GET_CLASS(s)->layout;
}

static void avpu_update(IngenicAvpuState *s)
{
    qemu_set_irq(s->irq, !!(R(AVPU_INT) & R(AVPU_INT_MASK)));
}

static void avpu_chan_free(gpointer p)
{
    AvpuChan *ch = p;

    ingenic_avpu_h264_free(ch->h264);
    g_free(ch);
}

static void avpu_job_free(IngenicAvpuJob *j)
{
    ingenic_avpu_h264_free(j->h264);
    g_free(j->y);
    g_free(j->uv);
    g_free(j->qps);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

void ingenic_avpu_escape(GByteArray *out, const uint8_t *p, size_t n)
{
    static const uint8_t epb = 3;
    int zeros = 0;

    for (size_t i = 0; i < n; i++) {
        if (zeros >= 2 && p[i] <= 3) {
            g_byte_array_append(out, &epb, 1);
            zeros = 0;
        }
        g_byte_array_append(out, &p[i], 1);
        zeros = p[i] ? 0 : zeros + 1;
    }
}

static int32_t sext(uint32_t v, int bits)
{
    return (int32_t)(v << (32 - bits)) >> (32 - bits);
}

/* The settings of an encode, from an EL150 entry's command words */
static void avpu_decode_el150(IngenicAvpuJob *j)
{
    const uint32_t *w = j->w;
    IngenicAvpuPic *pic = &j->pic;

    pic->width = (extract32(w[1], 0, 11) + 1) * 8;
    pic->height = (extract32(w[1], 12, 11) + 1) * 8;
    pic->hevc = extract32(w[0], 28, 2) == 1;
    pic->transform_8x8 = extract32(w[0], 10, 1);
    pic->scaling_list = extract32(w[9], 30, 2) == 3;
    pic->cabac = extract32(w[2], 10, 1);
    pic->cabac_init_idc = extract32(w[2], 8, 2);
    pic->intra = extract32(w[3], 28, 2) == 2;
    pic->qp = extract32(w[3], 16, 8);
    pic->chroma_qp_offset = sext(extract32(w[3], 0, 5), 5);
    pic->cr_qp_offset = sext(extract32(w[3], 8, 5), 5);
    pic->hdr_tail_bits = extract32(w[0x1e], 24, 5);
    pic->hdr_tail = w[0x1f] & MAKE_64BIT_MASK(0, pic->hdr_tail_bits);
    pic->beta_div2 = sext(extract32(w[4], 0, 5), 5);
    pic->alpha_c0_div2 = sext(extract32(w[4], 8, 6), 6);
    pic->auto_qp = extract32(w[9], 26, 1);
    j->src_y = w[0x20];
    j->src_uv = w[0x21];
    j->stride = w[0x22];
    j->ep2_addr = w[0x23];
    j->stream = w[0x30];
    j->offset = w[0x32];
    j->room = w[0x33];
}

/*
 * The EL200 packs the same fields elsewhere (libimp T41's
 * SliceParamToCmdRegsEnc1, against a real T41NQ): codec in word 0 bits 2:0,
 * the size with the height at bit 16, the tools word 152 holding word 9's
 * bits 31:26 (scaling lists 7:6, auto QP 1), and the entropy pass's words at
 * entry byte 0x800, where the slice header bits left for the AVPU share a
 * word with their count (bits 28:24).
 */
static void avpu_decode_el200(IngenicAvpuJob *j)
{
    const uint32_t *w = j->w;
    const uint32_t *e = j->w + 0x800 / 4;
    IngenicAvpuPic *pic = &j->pic;

    pic->width = (extract32(w[1], 0, 11) + 1) * 8;
    pic->height = (extract32(w[1], 16, 11) + 1) * 8;
    pic->hevc = extract32(w[0], 0, 3) == 1;
    pic->transform_8x8 = extract32(w[0], 10, 2) == 1;
    pic->scaling_list = extract32(w[152], 6, 2) == 3;
    pic->cabac = extract32(w[3], 10, 1);
    pic->cabac_init_idc = extract32(w[3], 8, 2);
    pic->intra = extract32(w[24], 28, 2) == 2;
    pic->qp = extract32(w[24], 16, 8);
    pic->chroma_qp_offset = sext(extract32(w[24], 0, 5), 5);
    pic->cr_qp_offset = sext(extract32(w[24], 8, 5), 5);
    pic->hdr_tail_bits = extract32(e[5], 24, 5);
    pic->hdr_tail = e[5] & MAKE_64BIT_MASK(0, pic->hdr_tail_bits);
    pic->beta_div2 = sext(extract32(w[25], 0, 5), 5);
    pic->alpha_c0_div2 = sext(extract32(w[25], 8, 6), 6);
    pic->auto_qp = extract32(w[152], 1, 1);
    j->src_y = w[40];
    j->src_uv = w[42];
    j->stride = w[52];
    j->ep2_addr = w[154];
    j->stream = w[136];
    j->offset = w[140];
    j->room = w[141];
}

static uint32_t avpu_score(const uint8_t *y, uint32_t stride, uint32_t w,
                           uint32_t h, uint32_t x0, uint32_t y0, int n)
{
    uint32_t score = 0;

    for (int i = 0; i < n - 1; i++) {
        const uint8_t *row = y + MIN(y0 + i, h - 1) * stride;
        const uint8_t *below = y + MIN(y0 + i + 1, h - 1) * stride;

        for (int k = 0; k < n - 1; k++) {
            uint32_t x = MIN(x0 + k, w - 1), right = MIN(x0 + k + 1, w - 1);

            score += abs(row[right] - row[x]) + abs(below[x] - row[x]);
        }
    }
    return score;
}

static int avpu_qp_step(const uint8_t *ep2, uint32_t score, uint32_t area)
{
    for (int i = 2; i >= 0; i--) {
        if (score < lduw_le_p(ep2 + 2 * i) * area) {
            return -ep2[12 + i];
        }
    }
    for (int i = 5; i >= 3; i--) {
        if (score > lduw_le_p(ep2 + 2 * i) * area) {
            return ep2[12 + i];
        }
    }
    return 0;
}

/* Each block's QP, and the sums of status words 0x4c-0x4e */
static void avpu_block_qps(IngenicAvpuJob *j)
{
    const IngenicAvpuPic *pic = &j->pic;
    uint32_t n = pic->hevc ? 32 : 16;
    uint32_t bw = DIV_ROUND_UP(pic->width, n);
    uint32_t bh = DIV_ROUND_UP(pic->height, n);

    j->qps = g_malloc(bw * bh);
    j->qp_min = INT_MAX;
    j->qp_max = INT_MIN;
    for (uint32_t by = 0; by < bh; by++) {
        for (uint32_t bx = 0; bx < bw; bx++) {
            uint32_t score = 0;
            int qp = pic->qp;
            /* H.264 counts a macroblock's 8x8 blocks, H.265 the picture's */
            uint32_t blocks = pic->hevc ?
                MIN(n, pic->width - bx * n) / 8 *
                (MIN(n, pic->height - by * n) / 8) : 4;

            /* without auto QP the AVPU scores nothing, and reports 0 */
            if (pic->auto_qp) {
                score = avpu_score(j->y, j->stride, pic->width, pic->height,
                                   bx * n, by * n, n);
                qp += avpu_qp_step(j->ep2, score, n * n / 256);
            }
            qp = MIN(MAX(qp, j->ep2[18]), j->ep2[19]);
            j->qps[by * bw + bx] = qp;
            j->score += score;
            j->qp_sum += qp * blocks;
            j->qp_min = MIN(j->qp_min, qp);
            j->qp_max = MAX(j->qp_max, qp);
        }
    }
    j->pic.block_qp = j->qps;
}

static int avpu_work(void *opaque)
{
    IngenicAvpuJob *j = opaque;

    if (j->jpeg) {
        j->codes = ingenic_avpu_jpeg_encode(j->y, j->stride, j->uv,
                                            j->cstride, j->pic.width,
                                            j->pic.height, j->qt, j->recip,
                                            j->units, j->xdensity,
                                            j->ydensity, j->data);
        j->ok = true;
        return 0;
    }
    avpu_block_qps(j);
    if (j->pic.hevc) {
        j->ok = ingenic_avpu_hevc_encode(&j->pic, j->y, j->uv, j->stride,
                                         j->data, &j->st);
        return 0;
    }
    j->ok = ingenic_avpu_h264_encode(j->h264, &j->pic, j->y, j->uv,
                                     j->stride, j->data, &j->st);
    return 0;
}

/* The status words of an entry, as the T31's AVPU leaves them */
static void avpu_status(IngenicAvpuJob *j, uint32_t *st)
{
    uint32_t mbs = DIV_ROUND_UP(j->pic.width, 16) *
                   DIV_ROUND_UP(j->pic.height, 16);
    uint32_t len = j->data->len;
    uint32_t avail = j->room;

    memset(st, 0, CL_STATUS_WORDS * 4);
    st[0x41 - CL_STATUS] = MIN(len, avail) | (len > avail ? BIT(31) : 0);
    st[0x4c - CL_STATUS] = j->score;
    st[0x4d - CL_STATUS] = j->qp_sum;
    st[0x4e - CL_STATUS] = j->qp_max << 8 | j->qp_min;
    if (j->pic.hevc) {
        st[0x42 - CL_STATUS] = j->st.bins;
        st[0x44 - CL_STATUS] = j->st.coded_4x4;
        st[0x45 - CL_STATUS] = (j->pic.width / 8) * (j->pic.height / 8);
        st[0x48 - CL_STATUS] = j->st.cus[0];        /* 8x8 coding units */
        st[0x49 - CL_STATUS] = j->st.cus[1];        /* 16x16 */
        st[0x4a - CL_STATUS] = j->st.cus[2];        /* 32x32 */
        return;
    }
    if (j->pic.intra) {
        j->st.intra_mbs = mbs;
        j->st.inter_mbs = j->st.skip_mbs = 0;
    }
    st[0x42 - CL_STATUS] = len * 8;                 /* CABAC bins */
    st[0x45 - CL_STATUS] = j->st.intra_mbs * 4;     /* intra 8x8 blocks */
    st[0x47 - CL_STATUS] = j->st.skip_mbs * 4;      /* skipped 8x8 blocks */
    st[0x49 - CL_STATUS] = mbs;                     /* 16x16 blocks */
}

static void avpu_jpeg_done(IngenicAvpuState *s, IngenicAvpuJob *j)
{
    const IngenicAvpuLayout *l = avpu_layout(s);
    const AvpuJpegRegs *r = l->jpeg;
    hwaddr zone = l->core + j->core * l->core_stride;
    uint32_t room = R(zone + r->room);

    address_space_write(&address_space_memory,
                        R(zone + r->out) + R(zone + r->out_off),
                        MEMTXATTRS_UNSPECIFIED, j->data->data,
                        MIN(j->data->len, room));
    if (r->codes) {
        R(zone + r->codes) = j->codes;
    }
    R(zone + r->bytes) = MIN(j->data->len, room);
    if (r->status) {
        R(zone + r->status) = j->data->len > room ? JPEG_OVERFLOW : 0;
    }
    s->job[j->core] = NULL;
    R(AVPU_INT) |= BIT(j->core * l->irq_stride);
    avpu_update(s);
    avpu_job_free(j);
}

static void avpu_done(void *opaque, int ret)
{
    IngenicAvpuJob *j = opaque;
    IngenicAvpuState *s = j->s;
    uint32_t st[CL_STATUS_WORDS];
    AvpuChan *ch;

    if (!s) {
        avpu_job_free(j);
        return;
    }
    if (j->jpeg) {
        avpu_jpeg_done(s, j);
        return;
    }
    ch = g_hash_table_lookup(s->chans, GUINT_TO_POINTER(j->chan));
    if (ch && !ch->h264) {
        ch->h264 = g_steal_pointer(&j->h264);
    }
    if (!j->ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: encode failed\n");
        g_byte_array_set_size(j->data, 0);
    }
    address_space_write(&address_space_memory, j->stream + j->offset,
                        MEMTXATTRS_UNSPECIFIED, j->data->data,
                        MIN(j->data->len, j->room));
    avpu_status(j, st);
    if (avpu_layout(s)->status) {
        /* the EL200's status: the same words from 0x41, without 0x46 */
        uint32_t out[CL_STATUS_WORDS - 2];

        for (int i = 0; i < ARRAY_SIZE(out); i++) {
            out[i] = cpu_to_le32(st[i < 5 ? i + 1 : i + 2]);
        }
        address_space_write(&address_space_memory,
                            R(CORE(avpu_layout(s), j->core, status)),
                            MEMTXATTRS_UNSPECIFIED, out, sizeof(out));
    } else {
        for (int i = 0; i < CL_STATUS_WORDS; i++) {
            st[i] = cpu_to_le32(st[i]);
        }
        address_space_write(&address_space_memory, j->cl + CL_STATUS * 4,
                            MEMTXATTRS_UNSPECIFIED, st, sizeof(st));
    }
    s->job[j->core] = NULL;
    R(AVPU_INT) |= BIT(j->core * avpu_layout(s)->irq_stride);
    avpu_update(s);
    avpu_job_free(j);
}

static void avpu_enc1(IngenicAvpuState *s, int core)
{
    const IngenicAvpuLayout *l = avpu_layout(s);
    IngenicAvpuJob *j = g_new0(IngenicAvpuJob, 1);
    AvpuChan *ch;
    size_t ylen, uvlen;

    j->s = s;
    j->core = core;
    j->cl = R(CORE(l, core, cl_addr));
    j->data = g_byte_array_new();
    address_space_read(&address_space_memory, j->cl, MEMTXATTRS_UNSPECIFIED,
                       j->w, l->entry);
    for (int i = 0; i < l->entry / 4; i++) {
        j->w[i] = le32_to_cpu(j->w[i]);
    }
    l->decode(j);
    if (j->ep2_addr) {
        address_space_read(&address_space_memory, j->ep2_addr,
                           MEMTXATTRS_UNSPECIFIED, j->ep2, sizeof(j->ep2));
    } else {
        j->ep2[19] = 51;                /* no steps, and any QP */
    }
    s->job[core] = j;
    if (!(j->w[0] & CL_LAST) ||
        (l->decode == avpu_decode_el150 ? extract32(j->w[0], 28, 2)
                                        : extract32(j->w[0], 0, 3)) > 1) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: only single-slice H.264 "
                      "and H.265 command lists are modelled "
                      "(word 0 0x%08x)\n", j->w[0]);
        avpu_done(j, 0);
        return;
    }
    if (j->pic.hevc) {
        goto source;
    }

    j->chan = j->ep2_addr;
    ch = g_hash_table_lookup(s->chans, GUINT_TO_POINTER(j->chan));
    if (!ch) {
        ch = g_new0(AvpuChan, 1);
        g_hash_table_insert(s->chans, GUINT_TO_POINTER(j->chan), ch);
    }
    j->h264 = g_steal_pointer(&ch->h264);
    if (j->h264 && !ingenic_avpu_h264_matches(j->h264, &j->pic)) {
        ingenic_avpu_h264_free(j->h264);
        j->h264 = NULL;
    }
    if (!j->h264) {
        j->h264 = ingenic_avpu_h264_new(&j->pic);
    }
    if (!j->h264) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: no H.264 encoder (x264)\n");
        avpu_done(j, 0);
        return;
    }

source:
    ylen = (size_t)j->stride * j->pic.height;
    uvlen = (size_t)j->stride * j->pic.height / 2;
    j->y = g_malloc(ylen);
    j->uv = g_malloc(uvlen);
    address_space_read(&address_space_memory, j->src_y,
                       MEMTXATTRS_UNSPECIFIED, j->y, ylen);
    address_space_read(&address_space_memory, j->src_uv,
                       MEMTXATTRS_UNSPECIFIED, j->uv, uvlen);
    thread_pool_submit_aio(avpu_work, j, avpu_done, j);
}

static void avpu_jpeg(IngenicAvpuState *s, int core)
{
    const IngenicAvpuLayout *l = avpu_layout(s);
    const AvpuJpegRegs *r = l->jpeg;
    IngenicAvpuJob *j = g_new0(IngenicAvpuJob, 1);
    hwaddr zone = l->core + core * l->core_stride;
    uint32_t size = R(zone + r->size);
    uint32_t mode = R(zone + r->mode);
    size_t ylen, uvlen;

    j->s = s;
    j->core = core;
    j->jpeg = true;
    j->units = extract32(mode, 12, 2);
    j->xdensity = R(zone + r->density) >> 16;
    j->ydensity = R(zone + r->density);
    j->data = g_byte_array_new();
    j->pic.width = (size >> 16) + 1;
    j->pic.height = (size & 0xffff) + 1;
    j->stride = R(zone + r->ystride) & 0xffff;
    j->cstride = R(zone + r->cstride) & 0xffff;
    s->job[core] = j;
    if ((mode & 3) != 1 || mode >> 16 || j->stride < j->pic.width ||
        j->cstride < j->pic.width) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: only 4:2:0 JPEG without "
                      "restart markers is modelled (mode 0x%08x)\n", mode);
        avpu_jpeg_done(s, j);
        return;
    }
    /* the quantizers, then their reciprocals */
    address_space_read(&address_space_memory, R(zone + r->tables),
                       MEMTXATTRS_UNSPECIFIED, j->qt, sizeof(j->qt));
    address_space_read(&address_space_memory,
                       R(zone + r->tables) + sizeof(j->qt),
                       MEMTXATTRS_UNSPECIFIED, j->recip, sizeof(j->recip));
    for (int i = 0; i < 2; i++) {
        for (int k = 0; k < 64; k++) {
            j->recip[i][k] = le16_to_cpu(j->recip[i][k]);
        }
    }
    ylen = (size_t)j->stride * j->pic.height;
    uvlen = (size_t)j->cstride * ((j->pic.height + 1) / 2);
    j->y = g_malloc(ylen);
    j->uv = g_malloc(uvlen);
    address_space_read(&address_space_memory, R(zone + r->y),
                       MEMTXATTRS_UNSPECIFIED, j->y, ylen);
    address_space_read(&address_space_memory, R(zone + r->uv),
                       MEMTXATTRS_UNSPECIFIED, j->uv, uvlen);
    thread_pool_submit_aio(avpu_work, j, avpu_done, j);
}

static void avpu_start(IngenicAvpuState *s, int core, uint32_t v)
{
    if (s->job[core]) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-avpu: core %d started while "
                      "busy\n", core);
        return;
    }
    if (core == 0 && v == START_ENC1) {
        avpu_enc1(s, core);
        return;
    }
    if (core == 1 && v == START_JPEG && avpu_layout(s)->cores > 1) {
        avpu_jpeg(s, core);
        return;
    }
    qemu_log_mask(LOG_UNIMP, "ingenic-avpu: core %d start 0x%x not modelled\n",
                  core, v);
}

/* The core a register belongs to and its offset in the core's, or -1 */
static int avpu_core_reg(const IngenicAvpuLayout *l, hwaddr off, hwaddr *reg)
{
    int core;

    if (off < l->core) {
        return -1;
    }
    core = (off - l->core) / l->core_stride;
    *reg = (off - l->core) % l->core_stride;
    return core < l->cores ? core : -1;
}

static uint64_t ingenic_avpu_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAvpuState *s = INGENIC_AVPU(opaque);
    const IngenicAvpuLayout *l = avpu_layout(s);
    hwaddr off = addr - l->base, reg;
    int core;

    if (addr < l->base || off >= INGENIC_AVPU_REGS_SIZE) {
        qemu_log_mask(LOG_UNIMP, "%s: 0x%05" HWADDR_PRIx "\n", __func__, addr);
        return 0;
    }
    switch (off) {
    case AVPU_UID:
        return 0x30ab6e51;
    case AVPU_HW_ID:
        return s->hw_id;
    case AVPU_DATE:
        return s->date;
    case AVPU_REV:
        return s->rev;
    case AVPU_AXI:
    case AVPU_TOP_CTRL:
        return 0;
    }
    core = avpu_core_reg(l, off, &reg);
    if (core >= 0) {
        if (reg == l->start || (l->reset && reg == l->reset)) {
            return 0;
        }
        if (l->clock && reg == l->clock) {
            return s->clock[core] ? 0x00010001 : 0;
        }
        if (reg == l->busy) {
            return s->job[core] ? l->busy_running : l->busy_idle;
        }
    }
    return R(off);
}

static void ingenic_avpu_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    IngenicAvpuState *s = INGENIC_AVPU(opaque);
    const IngenicAvpuLayout *l = avpu_layout(s);
    hwaddr off = addr - l->base, reg;
    uint32_t v = value;
    int core;

    if (addr < l->base || off >= INGENIC_AVPU_REGS_SIZE) {
        qemu_log_mask(LOG_UNIMP, "%s: 0x%05" HWADDR_PRIx " = 0x%08x\n",
                      __func__, addr, v);
        return;
    }
    switch (off) {
    case AVPU_UID:
    case AVPU_HW_ID:
    case AVPU_DATE:
    case AVPU_REV:
        return;
    case AVPU_INT:
        R(AVPU_INT) &= ~v;
        avpu_update(s);
        return;
    case AVPU_INT_MASK:
        R(AVPU_INT_MASK) = v;
        avpu_update(s);
        return;
    }
    core = avpu_core_reg(l, off, &reg);
    if (core >= 0) {
        if (reg == l->start) {
            avpu_start(s, core, v);
            return;
        }
        if (reg == l->busy || (l->reset && reg == l->reset)) {
            return;
        }
        if (l->clock && reg == l->clock) {
            s->clock[core] = v & 1;
            return;
        }
    }
    R(off) = v;
}

static const MemoryRegionOps ingenic_avpu_ops = {
    .read = ingenic_avpu_read,
    .write = ingenic_avpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_avpu_reset_hold(Object *obj, ResetType type)
{
    IngenicAvpuState *s = INGENIC_AVPU(obj);

    for (int c = 0; c < INGENIC_AVPU_CORES; c++) {
        if (s->job[c]) {
            s->job[c]->s = NULL;    /* its completion just frees it */
            s->job[c] = NULL;
        }
        s->clock[c] = false;
    }
    memset(s->regs, 0, sizeof(s->regs));
    g_hash_table_remove_all(s->chans);
}

static void ingenic_avpu_reset_exit(Object *obj, ResetType type)
{
    avpu_update(INGENIC_AVPU(obj));
}

static void ingenic_avpu_init(Object *obj)
{
    IngenicAvpuState *s = INGENIC_AVPU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_avpu_ops, s,
                          TYPE_INGENIC_AVPU, INGENIC_AVPU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    s->chans = g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL,
                                     avpu_chan_free);
}

static void ingenic_avpu_finalize(Object *obj)
{
    g_hash_table_destroy(INGENIC_AVPU(obj)->chans);
}

/* An encode in flight and the encoders' reference frames are host state */
static const VMStateDescription vmstate_ingenic_avpu = {
    .name = TYPE_INGENIC_AVPU,
    .unmigratable = 1,
};

static const AvpuJpegRegs avpu_jpeg_el150 = {
    .mode = 0x00,
    .size = 0x04,
    .density = 0x08,
    .ystride = 0x0c,
    .cstride = 0x0c,
    .y = 0x10,
    .uv = 0x14,
    .tables = 0x18,
    .out = 0x1c,
    .out_off = 0x24,
    .room = 0x28,
    .codes = 0x30,
    .bytes = 0x34,
    .status = 0x38,
};

/*
 * The EL200's JPEG core takes the EL150's settings as libimp T41's
 * JpegParamToCtrlRegs packs them, written straight into its zone (words 0-2
 * at 0x40, 6-27 at 0x58), with a stride per plane. A real T41NQ reports
 * only the bytes, and leaves 0x600 0.
 */
static const AvpuJpegRegs avpu_jpeg_el200 = {
    .mode = 0x40,
    .size = 0x44,
    .density = 0x48,
    .tables = 0x58,
    .out = 0x60,
    .out_off = 0x6c,
    .room = 0x70,
    .ystride = 0x74,
    .cstride = 0x78,
    .y = 0x80,
    .uv = 0x88,
    .bytes = 0x604,
};

static const IngenicAvpuLayout ingenic_avpu_el150 = {
    .base = 0x8000,
    .core = 0x200,
    .core_stride = 0x200,
    .cl_addr = 0x1e0,
    .start = 0x1e4,
    .reset = 0x1f0,
    .clock = 0x1f4,
    .busy = 0x1f8,
    .busy_running = 3,
    .cores = 2,
    .irq_stride = 4,
    .entry = 0x200,
    .decode = avpu_decode_el150,
    .jpeg = &avpu_jpeg_el150,
};

static const IngenicAvpuLayout ingenic_avpu_el200 = {
    .base = 0,
    .core = 0x1000,
    .core_stride = 0x1000,
    .cl_addr = 0x0,
    .status = 0x8,
    .start = 0x18,
    .busy = 0x1c,
    .busy_idle = BIT(31),
    .cores = 2,
    .irq_stride = 2,
    .entry = CL_ENTRY_MAX,
    .decode = avpu_decode_el200,
    .jpeg = &avpu_jpeg_el200,
};

/* The identity a T31 reads; libimp checks the hardware ID */
static const Property ingenic_avpu_properties[] = {
    DEFINE_PROP_UINT32("hw-id", IngenicAvpuState, hw_id, 0x72000460),
    DEFINE_PROP_UINT32("date", IngenicAvpuState, date, 0x20190326),
    DEFINE_PROP_UINT32("rev", IngenicAvpuState, rev, 0x5de5a975),
};

static void ingenic_avpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic AVPU video encoder";
    device_class_set_props(dc, ingenic_avpu_properties);
    dc->vmsd = &vmstate_ingenic_avpu;
    rc->phases.hold = ingenic_avpu_reset_hold;
    rc->phases.exit = ingenic_avpu_reset_exit;
    INGENIC_AVPU_CLASS(oc)->layout = &ingenic_avpu_el150;
}

static void ingenic_t41_avpu_class_init(ObjectClass *oc, const void *data)
{
    INGENIC_AVPU_CLASS(oc)->layout = &ingenic_avpu_el200;
}

static const TypeInfo ingenic_avpu_types[] = {
    {
        .name = TYPE_INGENIC_AVPU,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicAvpuState),
        .instance_init = ingenic_avpu_init,
        .instance_finalize = ingenic_avpu_finalize,
        .class_size = sizeof(IngenicAvpuClass),
        .class_init = ingenic_avpu_class_init,
    }, {
        .name = TYPE_INGENIC_T41_AVPU,
        .parent = TYPE_INGENIC_AVPU,
        .class_init = ingenic_t41_avpu_class_init,
    },
};

DEFINE_TYPES(ingenic_avpu_types)
