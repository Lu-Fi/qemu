/*
 * Ingenic AVPU: H.264 slice data from libx264
 *
 * libimp writes the SPS, the PPS and each slice header itself, and the
 * AVPU writes only what follows the header, with its emulation prevention
 * bytes. A CAVLC slice header need not end on a byte, so libimp writes
 * its first bytes and hands the AVPU the bits after them, which it writes
 * before the slice data; a CABAC one ends aligned and hands none. So x264
 * runs with the settings that header and libimp's parameter sets imply
 * (CABAC or CAVLC and the CABAC init table, the 8x8 transform, the
 * deblocking offsets, one reference, no weighted prediction, the chroma
 * QP offset, and the default scaling matrices where entry word 9 asks for
 * them, as libimp's High profile SPS signals), encodes each macroblock at
 * the QP the AVPU picks for it, and of the slice it puts out the header is
 * dropped: x264's own SPS and PPS say where it ends. x264 takes per
 * macroblock QPs only as adaptive quantization's offsets, which constant
 * QP turns off: so it runs in CRF mode with the frame's QP forced and its
 * own adaptive quantization too weak to move a QP. It cannot start a slice
 * with a QP change, so the slice's first macroblock stays at the slice QP
 * where the AVPU would code its own.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/ingenic-avpu.h"
#include <x264.h>

typedef struct H264Sps {
    int log2_max_frame_num;
    int poc_type;
    int log2_max_poc_lsb;
    bool delta_pic_order_always_zero;
    bool frame_mbs_only;
} H264Sps;

typedef struct H264Pps {
    bool cabac;
    bool bottom_field_pic_order;
    bool weighted_pred;
    int weighted_bipred;
    bool deblocking_filter_control;
    bool redundant_pic_cnt;
} H264Pps;

struct IngenicAvpuH264 {
    x264_t *x264;
    IngenicAvpuPic cfg;
    H264Sps sps;
    H264Pps pps;
    int64_t pts;
    IngenicAvpuStats last;      /* from x264's per-frame debug line */
};

typedef struct Bits {
    const uint8_t *p;
    size_t n;
    size_t pos;
    bool err;
} Bits;

static uint32_t bits_u(Bits *b, int n)
{
    uint32_t v = 0;

    while (n--) {
        if (b->pos >= b->n * 8) {
            b->err = true;
            return 0;
        }
        v = v << 1 | (b->p[b->pos >> 3] >> (7 - (b->pos & 7)) & 1);
        b->pos++;
    }
    return v;
}

static uint32_t bits_ue(Bits *b)
{
    int zeros = 0;

    while (!bits_u(b, 1)) {
        if (b->err || ++zeros > 31) {
            b->err = true;
            return 0;
        }
    }
    return zeros ? (1u << zeros) - 1 + bits_u(b, zeros) : 0;
}

static int32_t bits_se(Bits *b)
{
    uint32_t k = bits_ue(b);

    return k & 1 ? (k + 1) / 2 : -(int32_t)(k / 2);
}

/* The RBSP of a NAL unit's payload (after its header byte) */
static GByteArray *h264_unescape(const uint8_t *p, size_t n)
{
    GByteArray *rbsp = g_byte_array_sized_new(n);
    int zeros = 0;

    for (size_t i = 0; i < n; i++) {
        if (zeros >= 2 && p[i] == 3) {
            zeros = 0;
            continue;
        }
        g_byte_array_append(rbsp, &p[i], 1);
        zeros = p[i] ? 0 : zeros + 1;
    }
    return rbsp;
}

/* A NAL's header byte and payload, past an Annex B start code */
static const uint8_t *h264_nal(const uint8_t *p, size_t n, size_t *len)
{
    size_t i = 0;

    while (i + 1 < n && p[i] == 0) {
        i++;
    }
    if (i + 1 >= n || p[i] != 1 || i < 2) {
        return NULL;
    }
    *len = n - i - 1;
    return p + i + 1;
}

static void h264_skip_scaling_list(Bits *b, int size)
{
    int last = 8, next = 8;

    for (int i = 0; i < size && !b->err; i++) {
        if (next) {
            next = (last + bits_se(b) + 256) % 256;
        }
        last = next ? next : last;
    }
}

static bool h264_parse_sps(const uint8_t *nal, size_t n, H264Sps *sps)
{
    g_autoptr(GByteArray) rbsp = h264_unescape(nal + 1, n - 1);
    Bits b = { rbsp->data, rbsp->len };
    int profile = bits_u(&b, 8);

    bits_u(&b, 16);                         /* constraints, level */
    bits_ue(&b);                            /* seq_parameter_set_id */
    if (profile == 100 || profile == 110 || profile == 122 ||
        profile == 244 || profile == 44 || profile == 83 || profile == 86 ||
        profile == 118 || profile == 128) {
        int lists = 8;

        if (bits_ue(&b) == 3) {             /* chroma_format_idc */
            bits_u(&b, 1);
            lists = 12;
        }
        bits_ue(&b);                        /* bit depths */
        bits_ue(&b);
        bits_u(&b, 1);                      /* qpprime_y_zero_transform... */
        if (bits_u(&b, 1)) {                /* seq_scaling_matrix_present */
            for (int i = 0; i < lists; i++) {
                if (bits_u(&b, 1)) {
                    h264_skip_scaling_list(&b, i < 6 ? 16 : 64);
                }
            }
        }
    }
    sps->log2_max_frame_num = bits_ue(&b) + 4;
    sps->poc_type = bits_ue(&b);
    if (sps->poc_type == 0) {
        sps->log2_max_poc_lsb = bits_ue(&b) + 4;
    } else if (sps->poc_type == 1) {
        int cycle;

        sps->delta_pic_order_always_zero = bits_u(&b, 1);
        bits_se(&b);
        bits_se(&b);
        cycle = bits_ue(&b);
        while (cycle-- > 0 && !b.err) {
            bits_se(&b);
        }
    }
    bits_ue(&b);                            /* max_num_ref_frames */
    bits_u(&b, 1);
    bits_ue(&b);                            /* picture size in MBs */
    bits_ue(&b);
    sps->frame_mbs_only = bits_u(&b, 1);
    return !b.err;
}

static bool h264_parse_pps(const uint8_t *nal, size_t n, H264Pps *pps)
{
    g_autoptr(GByteArray) rbsp = h264_unescape(nal + 1, n - 1);
    Bits b = { rbsp->data, rbsp->len };

    bits_ue(&b);                            /* pic_parameter_set_id */
    bits_ue(&b);                            /* seq_parameter_set_id */
    pps->cabac = bits_u(&b, 1);
    pps->bottom_field_pic_order = bits_u(&b, 1);
    if (bits_ue(&b)) {                      /* num_slice_groups_minus1 */
        return false;
    }
    bits_ue(&b);                            /* num_ref_idx defaults */
    bits_ue(&b);
    pps->weighted_pred = bits_u(&b, 1);
    pps->weighted_bipred = bits_u(&b, 2);
    bits_se(&b);                            /* pic_init_qp, pic_init_qs */
    bits_se(&b);
    bits_se(&b);                            /* chroma_qp_index_offset */
    pps->deblocking_filter_control = bits_u(&b, 1);
    bits_u(&b, 1);                          /* constrained_intra_pred */
    pps->redundant_pic_cnt = bits_u(&b, 1);
    return !b.err;
}

/* Where the slice data starts in a slice's RBSP, in bits; -1 if unknown */
static int h264_slice_data(IngenicAvpuH264 *e, const uint8_t *nal, size_t n,
                           GByteArray **rbsp_out)
{
    int ref_idc = nal[0] >> 5 & 3, type = nal[0] & 31;
    GByteArray *rbsp = h264_unescape(nal + 1, n - 1);
    Bits b = { rbsp->data, rbsp->len };
    int slice_type;

    *rbsp_out = rbsp;
    bits_ue(&b);                            /* first_mb_in_slice */
    slice_type = bits_ue(&b) % 5;
    bits_ue(&b);                            /* pic_parameter_set_id */
    bits_u(&b, e->sps.log2_max_frame_num);
    if (!e->sps.frame_mbs_only && bits_u(&b, 1)) {
        bits_u(&b, 1);
    }
    if (type == 5) {
        bits_ue(&b);                        /* idr_pic_id */
    }
    if (e->sps.poc_type == 0) {
        bits_u(&b, e->sps.log2_max_poc_lsb);
        if (e->pps.bottom_field_pic_order) {
            bits_se(&b);
        }
    } else if (e->sps.poc_type == 1 && !e->sps.delta_pic_order_always_zero) {
        bits_se(&b);
        if (e->pps.bottom_field_pic_order) {
            bits_se(&b);
        }
    }
    if (e->pps.redundant_pic_cnt) {
        bits_ue(&b);
    }
    if (slice_type == 1) {                  /* B */
        bits_u(&b, 1);
    }
    if (slice_type == 0 || slice_type == 1) {
        if (bits_u(&b, 1)) {                /* num_ref_idx_active_override */
            bits_ue(&b);
            if (slice_type == 1) {
                bits_ue(&b);
            }
        }
        for (int l = 0; l < (slice_type == 1 ? 2 : 1); l++) {
            if (bits_u(&b, 1)) {            /* ref_pic_list_modification */
                uint32_t idc;

                do {
                    idc = bits_ue(&b);
                    if (idc != 3) {
                        bits_ue(&b);
                    }
                } while (idc != 3 && !b.err);
            }
        }
    }
    if ((e->pps.weighted_pred && slice_type == 0) ||
        (e->pps.weighted_bipred == 1 && slice_type == 1)) {
        return -1;
    }
    if (ref_idc) {
        if (type == 5) {
            bits_u(&b, 2);
        } else if (bits_u(&b, 1)) {         /* adaptive_ref_pic_marking */
            uint32_t op;

            do {
                op = bits_ue(&b);
                if (op == 1 || op == 3) {
                    bits_ue(&b);
                }
                if (op == 2 || op == 3 || op == 4 || op == 6) {
                    bits_ue(&b);
                }
            } while (op && !b.err);
        }
    }
    if (e->pps.cabac && slice_type != 2) {
        bits_ue(&b);                        /* cabac_init_idc */
    }
    bits_se(&b);                            /* slice_qp_delta */
    if (e->pps.deblocking_filter_control && bits_ue(&b) != 1) {
        bits_se(&b);
        bits_se(&b);
    }
    if (b.err) {
        return -1;
    }
    if (e->pps.cabac) {
        return ROUND_UP(b.pos, 8);          /* past cabac_alignment_one_bit */
    }
    return b.pos;
}

static void h264_put_bits(GByteArray *out, uint32_t *acc, int *n,
                          uint32_t v, int bits)
{
    while (bits--) {
        *acc = *acc << 1 | (v >> bits & 1);
        if (++*n == 8) {
            uint8_t byte = *acc;

            g_byte_array_append(out, &byte, 1);
            *acc = *n = 0;
        }
    }
}

/*
 * The AVPU's output RBSP: the header bits libimp handed over, the slice
 * data from bit @start of @rbsp to its stop bit, and the trailing bits
 */
static GByteArray *h264_slice_rbsp(const IngenicAvpuPic *pic,
                                   const GByteArray *rbsp, size_t start)
{
    GByteArray *out = g_byte_array_new();
    uint32_t acc = 0;
    int n = 0;
    size_t stop = rbsp->len * 8;

    while (stop > start &&
           !(rbsp->data[(stop - 1) / 8] >> (7 - (stop - 1) % 8) & 1)) {
        stop--;
    }
    h264_put_bits(out, &acc, &n, pic->hdr_tail, pic->hdr_tail_bits);
    for (size_t i = start; i + 1 < stop; i++) {
        h264_put_bits(out, &acc, &n, rbsp->data[i / 8] >> (7 - i % 8), 1);
    }
    h264_put_bits(out, &acc, &n, 1, 1);
    if (n) {
        h264_put_bits(out, &acc, &n, 0, 8 - n);
    }
    return out;
}

static void G_GNUC_PRINTF(3, 0)
h264_log(void *opaque, int level, const char *fmt, va_list ap)
{
    IngenicAvpuH264 *e = opaque;
    g_autofree char *line = g_strdup_vprintf(fmt, ap);
    const char *s = strstr(line, " I:");
    unsigned i, p, skip;

    if (level == X264_LOG_DEBUG && strncmp(line, "frame=", 6) == 0 && s &&
        sscanf(s, " I:%u P:%u SKIP:%u", &i, &p, &skip) == 3) {
        e->last.intra_mbs = i;
        e->last.inter_mbs = p;
        e->last.skip_mbs = skip;
    } else if (level <= X264_LOG_WARNING) {
        qemu_log_mask(LOG_UNIMP, "ingenic-avpu: x264: %s", line);
    }
}

IngenicAvpuH264 *ingenic_avpu_h264_new(const IngenicAvpuPic *pic)
{
    IngenicAvpuH264 *e = g_new0(IngenicAvpuH264, 1);
    x264_param_t p;
    x264_nal_t *nal;
    int n;

    if (x264_param_default_preset(&p, "superfast", "zerolatency") < 0) {
        g_free(e);
        return NULL;
    }
    p.i_threads = 1;
    p.b_sliced_threads = 0;
    p.i_width = pic->width;
    p.i_height = pic->height;
    p.i_csp = X264_CSP_NV12;
    p.i_bframe = 0;
    p.i_frame_reference = 1;
    p.i_keyint_max = X264_KEYINT_MAX_INFINITE;
    p.i_scenecut_threshold = 0;
    p.b_cabac = pic->cabac;
    p.i_cabac_init_idc = pic->cabac_init_idc;
    p.b_deblocking_filter = 1;
    p.i_deblocking_filter_alphac0 = pic->alpha_c0_div2;
    p.i_deblocking_filter_beta = pic->beta_div2;
    p.analyse.b_transform_8x8 = pic->transform_8x8;
    p.analyse.i_weighted_pred = X264_WEIGHTP_NONE;
    p.analyse.b_psy = 0;
    p.analyse.i_chroma_qp_offset = pic->chroma_qp_offset;
    p.i_cqm_preset = pic->scaling_list ? X264_CQM_JVT : X264_CQM_FLAT;
    p.rc.i_rc_method = X264_RC_CRF;
    p.rc.i_qp_min = 0;
    p.rc.i_qp_max = 51;
    p.rc.i_aq_mode = X264_AQ_VARIANCE;
    p.rc.f_aq_strength = 0.0001f;
    p.rc.b_mb_tree = 0;
    p.b_annexb = 1;
    p.b_repeat_headers = 0;
    p.i_log_level = X264_LOG_DEBUG;
    p.pf_log = (void (*)(void *, int, const char *, va_list))h264_log;
    p.p_log_private = e;

    e->cfg = *pic;
    e->x264 = x264_encoder_open(&p);
    if (!e->x264 || x264_encoder_headers(e->x264, &nal, &n) < 0) {
        ingenic_avpu_h264_free(e);
        return NULL;
    }
    for (int i = 0; i < n; i++) {
        size_t len;
        const uint8_t *u = h264_nal(nal[i].p_payload, nal[i].i_payload, &len);

        if (u && nal[i].i_type == NAL_SPS && !h264_parse_sps(u, len, &e->sps)) {
            u = NULL;
        }
        if (u && nal[i].i_type == NAL_PPS && !h264_parse_pps(u, len, &e->pps)) {
            u = NULL;
        }
        if (!u) {
            ingenic_avpu_h264_free(e);
            return NULL;
        }
    }
    return e;
}

bool ingenic_avpu_h264_matches(IngenicAvpuH264 *e, const IngenicAvpuPic *pic)
{
    const IngenicAvpuPic *c = &e->cfg;

    return c->width == pic->width && c->height == pic->height &&
           c->cabac == pic->cabac && c->transform_8x8 == pic->transform_8x8 &&
           c->scaling_list == pic->scaling_list &&
           c->cabac_init_idc == pic->cabac_init_idc &&
           c->alpha_c0_div2 == pic->alpha_c0_div2 &&
           c->beta_div2 == pic->beta_div2 &&
           c->chroma_qp_offset == pic->chroma_qp_offset;
}

bool ingenic_avpu_h264_encode(IngenicAvpuH264 *e, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st)
{
    x264_picture_t in, res;
    x264_nal_t *nal;
    int n, mbs = DIV_ROUND_UP(pic->width, 16) * DIV_ROUND_UP(pic->height, 16);
    g_autofree float *offsets = g_new0(float, mbs);

    /*
     * x264 sets a slice's QP to that of its first macroblock, where libimp's
     * header has the commanded QP: the first macroblock has to keep it
     */
    for (int i = 1; pic->block_qp && i < mbs; i++) {
        offsets[i] = pic->block_qp[i] - pic->qp;
    }
    x264_picture_init(&in);
    in.prop.quant_offsets = offsets;
    in.img.i_csp = X264_CSP_NV12;
    in.img.i_plane = 2;
    in.img.plane[0] = (uint8_t *)y;
    in.img.i_stride[0] = stride;
    in.img.plane[1] = (uint8_t *)uv;
    in.img.i_stride[1] = stride;
    in.i_type = pic->intra ? X264_TYPE_IDR : X264_TYPE_P;
    in.i_qpplus1 = pic->qp + 1;
    in.i_pts = e->pts++;
    memset(&e->last, 0, sizeof(e->last));
    if (x264_encoder_encode(e->x264, &nal, &n, &in, &res) < 0) {
        return false;
    }
    for (int i = 0; i < n; i++) {
        g_autoptr(GByteArray) rbsp = NULL;
        g_autoptr(GByteArray) data = NULL;
        size_t len;
        const uint8_t *u;
        int off;

        if (nal[i].i_type != NAL_SLICE && nal[i].i_type != NAL_SLICE_IDR) {
            continue;
        }
        u = h264_nal(nal[i].p_payload, nal[i].i_payload, &len);
        if (!u) {
            return false;
        }
        off = h264_slice_data(e, u, len, &rbsp);
        if (off < 0) {
            return false;
        }
        data = h264_slice_rbsp(pic, rbsp, off);
        ingenic_avpu_escape(out, data->data, data->len);
        *st = e->last;
        return true;
    }
    return false;
}

void ingenic_avpu_h264_free(IngenicAvpuH264 *e)
{
    if (e && e->x264) {
        x264_encoder_close(e->x264);
    }
    g_free(e);
}
