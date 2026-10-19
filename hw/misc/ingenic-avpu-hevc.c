/*
 * Ingenic AVPU: H.265 slice data
 *
 * For H.265 libimp writes the parameter sets and the slice segment header
 * itself, after the encode, and the AVPU writes only the slice data: CABAC,
 * from the byte after the header, ended by the stop bit and emulation
 * prevented. libimp's streams are Main profile with CTBs of 32, coding
 * blocks down to 8, transform blocks of 4 to 32 one level deep, the default
 * scaling lists (flat ones if entry word 9 says), no sign hiding, and
 * cu_qp_delta per CTB; its P slices set cabac_init_flag, so take the
 * contexts of initType 2.
 *
 * This codes every coding block at the QP the AVPU picks for its CTB
 * (ingenic-avpu.c): 32x32, or 16x16 and 8x8 where the picture's edge cuts a
 * CTB, each one transform block. An intra block is predicted vertically,
 * horizontally or DC, the chroma as the luma; 8x8 blocks are DC only, which
 * keeps every scan the diagonal one. In a P picture a block is either that
 * or skipped, copying the channel's last picture: libimp's P slices refer
 * to the picture before, alone. That picture is kept deblocked, as the
 * decoder has it; libimp turns SAO off.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "hw/misc/ingenic-avpu.h"

#define LOG2_CTB        5
#define LOG2_MIN_CB     3
#define CTB             (1 << LOG2_CTB)

enum { MODE_PLANAR = 0, MODE_DC = 1, MODE_HOR = 10, MODE_VER = 26 };

/* Context variables, in the order of ctx_init's rows */
enum {
    CTX_SPLIT_CU = 0,       /* 3 */
    CTX_SKIP = 3,           /* 3 */
    CTX_PRED_MODE = 6,
    CTX_PART_MODE = 7,      /* bin 0 */
    CTX_PREV_INTRA = 8,
    CTX_CHROMA_MODE = 9,
    CTX_SPLIT_TU = 10,      /* 3 */
    CTX_CBF_LUMA = 13,      /* 2 */
    CTX_CBF_CHROMA = 15,    /* 4, cb and cr */
    CTX_QP_DELTA = 19,      /* 2 */
    CTX_LAST_X = 21,        /* 18 */
    CTX_LAST_Y = 39,        /* 18 */
    CTX_CSBF = 57,          /* 4 */
    CTX_SIG = 61,           /* 42 */
    CTX_GT1 = 103,          /* 24 */
    CTX_GT2 = 127,          /* 6 */
    CTX_TRANSFORM_SKIP = 133,   /* luma, chroma */
    CTX_MERGE_IDX = 135,
    CTX_COUNT = 136,
};

/* initValue by initType: I slices, P slices, P with cabac_init_flag */
static const uint8_t ctx_init[3][CTX_COUNT] = {
    {
        139, 141, 157,
        154, 154, 154,
        154,
        184,
        184,
        63,
        153, 138, 138,
        111, 141,
        94, 138, 182, 154,
        154, 154,
        110, 110, 124, 125, 140, 153, 125, 127, 140,
        109, 111, 143, 127, 111, 79, 108, 123, 63,
        110, 110, 124, 125, 140, 153, 125, 127, 140,
        109, 111, 143, 127, 111, 79, 108, 123, 63,
        91, 171, 134, 141,
        111, 111, 125, 110, 110, 94, 124, 108, 124, 107, 125, 141, 179, 153,
        125, 107, 125, 141, 179, 153, 125, 107, 125, 141, 179, 153, 125, 140,
        139, 182, 182, 152, 136, 152, 136, 153, 136, 139, 111, 136, 139, 111,
        140, 92, 137, 138, 140, 152, 138, 139, 153, 74, 149, 92,
        139, 107, 122, 152, 140, 179, 166, 182, 140, 227, 122, 197,
        138, 153, 136, 167, 152, 152,
        139, 139,
        154,
    }, {
        107, 139, 126,
        197, 185, 201,
        149,
        154,
        154,
        152,
        124, 138, 94,
        153, 111,
        149, 107, 167, 154,
        154, 154,
        125, 110, 94, 110, 95, 79, 125, 111, 110,
        78, 110, 111, 111, 95, 94, 108, 123, 108,
        125, 110, 94, 110, 95, 79, 125, 111, 110,
        78, 110, 111, 111, 95, 94, 108, 123, 108,
        121, 140, 61, 154,
        155, 154, 139, 153, 139, 123, 123, 63, 153, 166, 183, 140, 136, 153,
        154, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 170,
        153, 123, 123, 107, 121, 107, 121, 167, 151, 183, 140, 151, 183, 140,
        154, 196, 196, 167, 154, 152, 167, 182, 182, 134, 149, 136,
        153, 121, 136, 137, 169, 194, 166, 167, 154, 167, 137, 182,
        107, 167, 91, 122, 107, 167,
        139, 139,
        122,
    }, {
        107, 139, 126,
        197, 185, 201,
        134,
        154,
        183,
        152,
        224, 167, 122,
        153, 111,
        149, 92, 167, 154,
        154, 154,
        125, 110, 124, 110, 95, 94, 125, 111, 111,
        79, 125, 126, 111, 111, 79, 108, 123, 93,
        125, 110, 124, 110, 95, 94, 125, 111, 111,
        79, 125, 126, 111, 111, 79, 108, 123, 93,
        121, 140, 61, 154,
        170, 154, 139, 153, 139, 123, 123, 63, 124, 166, 183, 140, 136, 153,
        154, 166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 170,
        153, 138, 138, 122, 121, 122, 121, 167, 151, 183, 140, 151, 183, 140,
        154, 196, 167, 167, 154, 152, 167, 182, 182, 134, 149, 136,
        153, 121, 136, 122, 169, 208, 166, 167, 154, 152, 167, 182,
        107, 167, 91, 107, 107, 167,
        139, 139,
        137,
    },
};

static const uint8_t lps_table[64][4] = {
    { 128, 176, 208, 240 }, { 128, 167, 197, 227 }, { 128, 158, 187, 216 },
    { 123, 150, 178, 205 }, { 116, 142, 169, 195 }, { 111, 135, 160, 185 },
    { 105, 128, 152, 175 }, { 100, 122, 144, 166 }, { 95, 116, 137, 158 },
    { 90, 110, 130, 150 }, { 85, 104, 123, 142 }, { 81, 99, 117, 135 },
    { 77, 94, 111, 128 }, { 73, 89, 105, 122 }, { 69, 85, 100, 116 },
    { 66, 80, 95, 110 }, { 62, 76, 90, 104 }, { 59, 72, 86, 99 },
    { 56, 69, 81, 94 }, { 53, 65, 77, 89 }, { 51, 62, 73, 85 },
    { 48, 59, 69, 80 }, { 46, 56, 66, 76 }, { 43, 53, 63, 72 },
    { 41, 50, 59, 69 }, { 39, 48, 56, 65 }, { 37, 45, 54, 62 },
    { 35, 43, 51, 59 }, { 33, 41, 48, 56 }, { 32, 39, 46, 53 },
    { 30, 37, 43, 50 }, { 29, 35, 41, 48 }, { 27, 33, 39, 45 },
    { 26, 31, 37, 43 }, { 24, 30, 35, 41 }, { 23, 28, 33, 39 },
    { 22, 27, 32, 37 }, { 21, 26, 30, 35 }, { 20, 24, 29, 33 },
    { 19, 23, 27, 31 }, { 18, 22, 26, 30 }, { 17, 21, 25, 28 },
    { 16, 20, 23, 27 }, { 15, 19, 22, 25 }, { 14, 18, 21, 24 },
    { 14, 17, 20, 23 }, { 13, 16, 19, 22 }, { 12, 15, 18, 21 },
    { 12, 14, 17, 20 }, { 11, 14, 16, 19 }, { 11, 13, 15, 18 },
    { 10, 12, 15, 17 }, { 10, 12, 14, 16 }, { 9, 11, 13, 15 },
    { 9, 11, 12, 14 }, { 8, 10, 12, 14 }, { 8, 9, 11, 13 },
    { 7, 9, 11, 12 }, { 7, 9, 10, 12 }, { 7, 8, 10, 11 },
    { 6, 8, 9, 11 }, { 6, 7, 9, 10 }, { 6, 7, 8, 9 },
    { 2, 2, 2, 2 },
};

static const uint8_t lps_next[64] = {
    0, 0, 1, 2, 2, 4, 4, 5, 6, 7, 8, 9, 9, 11, 11, 12,
    13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
    24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33,
    33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63,
};

/* The default intra scaling list of 8x8 and larger blocks, in diagonal order */
static const uint8_t scaling_intra[64] = {
    16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 16, 17, 16, 17, 18,
    17, 18, 18, 17, 18, 21, 19, 20, 21, 20, 19, 21, 24, 22, 22, 24,
    24, 22, 22, 24, 25, 25, 27, 30, 27, 25, 25, 29, 31, 35, 35, 31,
    29, 36, 41, 44, 41, 36, 47, 54, 54, 47, 65, 70, 65, 88, 88, 115,
};

/* Of a 32-point DCT row, cos(m * pi / 64) scaled; the smaller DCTs subsample */
static const uint8_t dct_cos[33] = {
    64, 90, 90, 90, 89, 88, 87, 85, 83, 82, 80, 78, 75, 73, 70, 67,
    64, 61, 57, 54, 50, 46, 43, 38, 36, 31, 25, 22, 18, 13, 9, 4, 0,
};

static const uint8_t last_group[32] = {
    0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9,
};

static const uint8_t last_group_min[10] = { 0, 1, 2, 3, 4, 6, 8, 12, 16, 24 };

static const uint8_t sig_ctx_4x4[16] = {
    0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8,
};

static const uint8_t level_scale[6] = { 40, 45, 51, 57, 64, 72 };
static const uint16_t quant_scale[6] = {
    26214, 23302, 20560, 18396, 16384, 14564,
};

typedef struct HevcTables {
    int8_t dct[32][32];             /* row k: basis k of the 32-point DCT */
    uint16_t bitcost[64][2];        /* -log2 p of the MPS, the LPS, in 1/256 */
    uint8_t diag[4][64][2];         /* diagonal scans of 1x1 to 8x8: x, y */
    uint8_t scale[2][4][32 * 32];   /* ScalingFactor, default and flat */
    uint16_t quant[2][6][4][32 * 32];   /* the encoder's, by QP % 6 */
} HevcTables;

static HevcTables tables;

typedef struct Cabac {
    uint32_t low, range;
    int bits_left;
    int buffered;               /* bytes held back for a carry */
    uint32_t buffered_byte;
    uint32_t bits, nbits;       /* the bit writer's partial byte */
    uint8_t state[CTX_COUNT];   /* pStateIdx << 1 | valMps */
    uint32_t bins;
    GByteArray *rbsp;
} Cabac;

/* A channel's last picture, deblocked: what its next P picture refers to */
struct IngenicAvpuHevc {
    uint32_t w, h;
    uint8_t *ref;               /* Y, then Cb and Cr */
};

typedef struct HevcEnc {
    const IngenicAvpuPic *pic;
    uint32_t w, h, w4, h4;
    int qp, qp_c[2];
    const uint8_t *src[3];      /* Y, Cb, Cr */
    uint32_t src_stride[3];
    uint8_t *rec[3];
    const uint8_t *ref[3];      /* NULL in an I picture */
    uint8_t *done;              /* reconstructed, by 4x4 luma block */
    uint8_t *mode;              /* luma intra mode, by 4x4 luma block */
    uint8_t *depth;             /* coding quadtree depth, by 4x4 luma block */
    uint8_t *skip;              /* cu_skip_flag, by 4x4 luma block */
    uint8_t *qpy;               /* the decoder's QpY, by 4x4 luma block */
    uint8_t *vedge;             /* a coding block's left edge, by 4 rows x 8 */
    uint8_t *hedge;             /* its top edge, by 8 rows x 4 columns */
    int64_t lambda;             /* the CTB's, skip against intra, 1/256 */
    int qp_pred;                /* the decoder's, for this CTB */
    bool qp_delta_coded;
    Cabac c;
    IngenicAvpuStats *st;
} HevcEnc;

static void hevc_tables_init(void)
{
    static gsize done;

    if (!g_once_init_enter(&done)) {
        return;
    }
    for (int k = 0; k < 32; k++) {
        for (int n = 0; n < 32; n++) {
            int m = ((2 * n + 1) * k) % 128, sign = 1;

            if (m > 64) {
                m -= 64;
                sign = -sign;
            }
            if (m > 32) {
                m = 64 - m;
                sign = -sign;
            }
            tables.dct[k][n] = sign * dct_cos[m];
        }
    }
    for (int log2 = 0; log2 < 4; log2++) {
        int size = 1 << log2, i = 0, x = 0, y = 0;

        while (i < size * size) {
            while (y >= 0) {
                if (x < size && y < size) {
                    tables.diag[log2][i][0] = x;
                    tables.diag[log2][i][1] = y;
                    i++;
                }
                y--;
                x++;
            }
            y = x;
            x = 0;
        }
    }
    for (int st = 0; st < 64; st++) {
        double lps = 0.5 * pow(0.01875 / 0.5, st / 63.0);

        tables.bitcost[st][0] = lround(-log2(1 - lps) * 256);
        tables.bitcost[st][1] = lround(-log2(lps) * 256);
    }
    memset(tables.scale, 16, sizeof(tables.scale));
    for (int log2 = 3; log2 <= 5; log2++) {
        int n = 1 << log2, rep = n / 8;

        for (int i = 0; i < 64; i++) {
            int x = tables.diag[3][i][0], y = tables.diag[3][i][1];

            for (int j = 0; j < rep; j++) {
                for (int k = 0; k < rep; k++) {
                    tables.scale[0][log2 - 2][(y * rep + j) * n + x * rep + k] =
                        scaling_intra[i];
                }
            }
        }
        tables.scale[0][log2 - 2][0] = 16;  /* scaling_list_dc_coef */
    }
    for (int f = 0; f < 2; f++) {
        for (int q = 0; q < 6; q++) {
            for (int s = 0; s < 4; s++) {
                for (int i = 0; i < (16 << 2 * s); i++) {
                    tables.quant[f][q][s][i] = (quant_scale[q] << 4) /
                                               tables.scale[f][s][i];
                }
            }
        }
    }
    g_once_init_leave(&done, 1);
}

/* CABAC, as the HM reference encoder writes it */

static void cabac_put_byte(Cabac *c, uint8_t b)
{
    g_byte_array_append(c->rbsp, &b, 1);
}

static void cabac_put_bits(Cabac *c, uint32_t v, int n)
{
    while (n--) {
        c->bits = c->bits << 1 | (v >> n & 1);
        if (++c->nbits == 8) {
            cabac_put_byte(c, c->bits);
            c->bits = c->nbits = 0;
        }
    }
}

static void cabac_out(Cabac *c)
{
    uint32_t lead = c->low >> (24 - c->bits_left);

    c->bits_left += 8;
    c->low &= 0xffffffffu >> c->bits_left;
    if (lead == 0xff) {
        c->buffered++;
    } else if (c->buffered) {
        uint32_t carry = lead >> 8;

        cabac_put_byte(c, c->buffered_byte + carry);
        c->buffered_byte = lead & 0xff;
        while (c->buffered > 1) {
            cabac_put_byte(c, 0xff + carry);
            c->buffered--;
        }
    } else {
        c->buffered = 1;
        c->buffered_byte = lead;
    }
}

static void cabac_init(Cabac *c, int init_type, int qp, GByteArray *rbsp)
{
    memset(c, 0, sizeof(*c));
    c->range = 510;
    c->bits_left = 23;
    c->buffered_byte = 0xff;
    c->rbsp = rbsp;
    for (int i = 0; i < CTX_COUNT; i++) {
        int v = ctx_init[init_type][i];
        int m = (v >> 4) * 5 - 45, n = ((v & 15) << 3) - 16;
        int pre = MIN(MAX(((m * qp) >> 4) + n, 1), 126);

        c->state[i] = pre <= 63 ? (63 - pre) << 1 : (pre - 64) << 1 | 1;
    }
}

static void cabac_bin(Cabac *c, int ctx, int bin)
{
    uint8_t *s = &c->state[ctx];
    uint32_t lps = lps_table[*s >> 1][(c->range >> 6) & 3];

    c->bins++;
    c->range -= lps;
    if (bin != (*s & 1)) {
        int n = 0;

        while ((lps << n) < 256) {
            n++;
        }
        c->low = (c->low + c->range) << n;
        c->range = lps << n;
        c->bits_left -= n;
        *s = (*s >> 1) ? lps_next[*s >> 1] << 1 | (*s & 1)
                       : (*s ^ 1);
    } else {
        if ((*s >> 1) < 62) {
            *s += 2;
        }
        if (c->range >= 256) {
            return;
        }
        c->low <<= 1;
        c->range <<= 1;
        c->bits_left--;
    }
    if (c->bits_left < 12) {
        cabac_out(c);
    }
}

static void cabac_bypass(Cabac *c, uint32_t bins, int n)
{
    c->bins += n;
    while (n--) {
        c->low <<= 1;
        if (bins >> n & 1) {
            c->low += c->range;
        }
        if (--c->bits_left < 12) {
            cabac_out(c);
        }
    }
}

static void cabac_terminate(Cabac *c, int bin)
{
    c->bins++;
    c->range -= 2;
    if (bin) {
        c->low += c->range;
        c->low <<= 7;
        c->range = 2 << 7;
        c->bits_left -= 7;
    } else if (c->range >= 256) {
        return;
    } else {
        c->low <<= 1;
        c->range <<= 1;
        c->bits_left--;
    }
    if (c->bits_left < 12) {
        cabac_out(c);
    }
}

/* Bits written so far, as HM counts them: held-back bytes and the register */
static uint64_t cabac_tell(const Cabac *c)
{
    return 8 * ((uint64_t)c->rbsp->len + c->buffered) + 23 - c->bits_left;
}

/* What coding @bin with context @ctx would cost now, in 1/256 bits */
static int cabac_cost(const Cabac *c, int ctx, int bin)
{
    uint8_t s = c->state[ctx];

    return tables.bitcost[s >> 1][bin != (s & 1)];
}

/* Flush, then rbsp_slice_segment_trailing_bits */
static void cabac_finish(Cabac *c)
{
    if (c->low >> (32 - c->bits_left)) {
        cabac_put_byte(c, c->buffered_byte + 1);
        while (c->buffered > 1) {
            cabac_put_byte(c, 0);
            c->buffered--;
        }
        c->low -= 1 << (32 - c->bits_left);
    } else {
        if (c->buffered) {
            cabac_put_byte(c, c->buffered_byte);
        }
        while (c->buffered > 1) {
            cabac_put_byte(c, 0xff);
            c->buffered--;
        }
    }
    cabac_put_bits(c, c->low >> 8, 24 - c->bits_left);
    cabac_put_bits(c, 1, 1);
    if (c->nbits) {
        cabac_put_bits(c, 0, 8 - c->nbits);
    }
}

/* Prediction */

static bool hevc_avail(HevcEnc *e, int x, int y)
{
    return x >= 0 && y >= 0 && x < e->w && y < e->h &&
           e->done[(y >> 2) * e->w4 + (x >> 2)];
}

/*
 * The reference samples of an n x n block at (x0, y0) of component cidx,
 * substituted as 8.4.4.2.2 has it: ref[2n] is the corner, ref[2n - 1 - k]
 * the left column's k-th, ref[2n + 1 + k] the top row's k-th.
 */
static void hevc_ref(HevcEnc *e, int cidx, int x0, int y0, int n,
                     uint8_t *ref)
{
    int sub = cidx ? 2 : 1, stride = e->w / sub, count = 4 * n + 1;
    const uint8_t *rec = e->rec[cidx];
    bool avail[4 * 32 + 1], any = false;
    int first = -1;

    for (int i = 0; i < count; i++) {
        int x, y;

        if (i < 2 * n) {
            x = x0 - 1;
            y = y0 + 2 * n - 1 - i;
        } else {
            x = x0 + i - 2 * n - 1;
            y = y0 - 1;
        }
        avail[i] = hevc_avail(e, x * sub, y * sub);
        if (avail[i]) {
            ref[i] = rec[y * stride + x];
            any = true;
            if (first < 0) {
                first = i;
            }
        }
    }
    if (!any) {
        memset(ref, 128, count);
        return;
    }
    if (!avail[0]) {
        ref[0] = ref[first];
    }
    for (int i = 1; i < count; i++) {
        if (!avail[i]) {
            ref[i] = ref[i - 1];
        }
    }
}

static void hevc_predict(int mode, int cidx, int n, const uint8_t *ref,
                         uint8_t *pred)
{
    const uint8_t *corner = ref + 2 * n;
    bool edge = !cidx && n < 32;

#define LEFT(k)     (corner[-1 - (k)])
#define TOP(k)      (corner[1 + (k)])
    switch (mode) {
    case MODE_VER:
        for (int y = 0; y < n; y++) {
            memcpy(pred + y * n, &TOP(0), n);
            if (edge) {
                pred[y * n] = MIN(MAX(TOP(0) + ((LEFT(y) - *corner) >> 1),
                                      0), 255);
            }
        }
        break;
    case MODE_HOR:
        for (int y = 0; y < n; y++) {
            memset(pred + y * n, LEFT(y), n);
        }
        if (edge) {
            for (int x = 0; x < n; x++) {
                pred[x] = MIN(MAX(LEFT(0) + ((TOP(x) - *corner) >> 1), 0),
                              255);
            }
        }
        break;
    default: {
        int sum = n, dc;

        for (int k = 0; k < n; k++) {
            sum += LEFT(k) + TOP(k);
        }
        dc = sum >> (ctz32(n) + 1);
        memset(pred, dc, n * n);
        if (edge) {
            pred[0] = (LEFT(0) + 2 * dc + TOP(0) + 2) >> 2;
            for (int k = 1; k < n; k++) {
                pred[k] = (TOP(k) + 3 * dc + 2) >> 2;
                pred[k * n] = (LEFT(k) + 3 * dc + 2) >> 2;
            }
        }
        break;
    }
    }
#undef LEFT
#undef TOP
}

/* Transform and quantization */

/*
 * The HM encoder's forward DCT. Every basis but the first sums to zero, so
 * a constant input transforms to its first coefficient alone; that and
 * rows repeating the one above (the residual of a picture's columns, after
 * vertical prediction) are taken short, to the same result.
 */
static void hevc_fdct(const int16_t *res, int log2, int32_t *coef)
{
    int n = 1 << log2, step = 32 >> log2;
    int shift1 = log2 - 1, shift2 = log2 + 6;
    int32_t tmp[32 * 32];

    for (int y = 0; y < n; y++) {
        const int16_t *row = res + y * n;
        bool flat = true;

        if (y && !memcmp(row, row - n, n * sizeof(*row))) {
            for (int k = 0; k < n; k++) {
                tmp[k * n + y] = tmp[k * n + y - 1];
            }
            continue;
        }
        for (int i = 1; i < n && flat; i++) {
            flat = row[i] == row[0];
        }
        for (int k = 0; k < n; k++) {
            int32_t sum = 0;

            if (flat) {
                sum = k ? 0 : 64 * n * row[0];
            } else {
                for (int i = 0; i < n; i++) {
                    sum += tables.dct[k * step][i] * row[i];
                }
            }
            tmp[k * n + y] = (sum + (1 << (shift1 - 1))) >> shift1;
        }
    }
    for (int x = 0; x < n; x++) {
        const int32_t *col = tmp + x * n;
        bool flat = true;

        for (int i = 1; i < n && flat; i++) {
            flat = col[i] == col[0];
        }
        for (int k = 0; k < n; k++) {
            int32_t sum = 0;

            if (flat) {
                sum = k ? 0 : 64 * n * col[0];
            } else {
                for (int i = 0; i < n; i++) {
                    sum += tables.dct[k * step][i] * col[i];
                }
            }
            coef[k * n + x] = (sum + (1 << (shift2 - 1))) >> shift2;
        }
    }
}

/*
 * 8.6.4.2, the decoder's: residuals from the scaled coefficients. The sums
 * stop at the last nonzero input; with 16-bit inputs they fit 32 bits.
 */
static void hevc_idct(const int32_t *d, int log2, int16_t *res)
{
    int n = 1 << log2, step = 32 >> log2;
    int32_t tmp[32 * 32];

    for (int x = 0; x < n; x++) {
        int last = n - 1;

        while (last >= 0 && !d[last * n + x]) {
            last--;
        }
        for (int y = 0; y < n; y++) {
            int32_t sum = 0;

            for (int k = 0; k <= last; k++) {
                sum += tables.dct[k * step][y] * d[k * n + x];
            }
            tmp[y * n + x] = MIN(MAX((sum + 64) >> 7, -32768), 32767);
        }
    }
    for (int y = 0; y < n; y++) {
        const int32_t *row = tmp + y * n;
        int last = n - 1;

        while (last >= 0 && !row[last]) {
            last--;
        }
        for (int x = 0; x < n; x++) {
            int32_t sum = 0;

            for (int k = 0; k <= last; k++) {
                sum += tables.dct[k * step][x] * row[k];
            }
            res[y * n + x] = (sum + 2048) >> 12;
        }
    }
}

/*
 * Quantize the residual of an n x n block; returns whether any level is
 * left, and then the decoder's residual from them in res.
 */
static bool hevc_quant(int16_t *res, int log2, int qp, bool flat,
                       int32_t *level)
{
    int n = 1 << log2, size = n * n;
    const uint8_t *m = tables.scale[flat][log2 - 2];
    const uint16_t *qc = tables.quant[flat][qp % 6][log2 - 2];
    int qbits = 21 + qp / 6 - log2, bd_shift = 3 + log2;
    int64_t offset = (int64_t)171 << (qbits - 9);
    int32_t coef[32 * 32], d[32 * 32];
    bool any = false;

    for (int i = 0; i < size && !any; i++) {
        any = res[i] != 0;
    }
    if (!any) {
        memset(level, 0, size * sizeof(*level));
        return false;
    }
    hevc_fdct(res, log2, coef);
    any = false;
    for (int i = 0; i < size; i++) {
        int64_t q = (int64_t)abs(coef[i]) * qc[i];
        int32_t l = coef[i] ? MIN((q + offset) >> qbits, 32767) : 0;

        level[i] = coef[i] < 0 ? -l : l;
        any |= l != 0;
    }
    if (!any) {
        return false;
    }
    for (int i = 0; i < size; i++) {
        int64_t v = ((int64_t)level[i] * m[i] * level_scale[qp % 6]
                     << (qp / 6)) + (1 << (bd_shift - 1));

        d[i] = MIN(MAX(v >> bd_shift, -32768), 32767);
    }
    hevc_idct(d, log2, res);
    return true;
}

/* Predict, quantize and reconstruct one block; returns its cbf */
static bool hevc_block(HevcEnc *e, int cidx, int x0, int y0, int log2,
                       int mode, int32_t *level)
{
    int n = 1 << log2, sub = cidx ? 2 : 1, stride = e->w / sub;
    uint8_t ref[4 * 32 + 1], pred[32 * 32];
    int16_t res[32 * 32];
    const uint8_t *src = e->src[cidx];
    uint8_t *rec = e->rec[cidx];
    bool cbf;

    hevc_ref(e, cidx, x0, y0, n, ref);
    hevc_predict(mode, cidx, n, ref, pred);
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            res[y * n + x] = src[(y0 + y) * e->src_stride[cidx] + x0 + x] -
                             pred[y * n + x];
        }
    }
    cbf = hevc_quant(res, log2, cidx ? e->qp_c[cidx - 1] : e->qp,
                     !e->pic->scaling_list, level);
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            int v = pred[y * n + x] + (cbf ? res[y * n + x] : 0);

            rec[(y0 + y) * stride + x0 + x] = MIN(MAX(v, 0), 255);
        }
    }
    return cbf;
}

/* The intra mode that predicts a luma block best, by SAD */
static int hevc_pick_mode(HevcEnc *e, int x0, int y0, int log2)
{
    static const uint8_t modes[] = { MODE_VER, MODE_HOR, MODE_DC };
    int n = 1 << log2, best = MODE_DC;
    uint32_t best_sad = UINT32_MAX;
    uint8_t ref[4 * 32 + 1], pred[32 * 32];

    if (log2 == LOG2_MIN_CB) {
        return MODE_DC;
    }
    hevc_ref(e, 0, x0, y0, n, ref);
    for (int i = 0; i < ARRAY_SIZE(modes); i++) {
        uint32_t sad = 0;

        hevc_predict(modes[i], 0, n, ref, pred);
        for (int y = 0; y < n; y++) {
            for (int x = 0; x < n; x++) {
                sad += abs(e->src[0][(y0 + y) * e->src_stride[0] + x0 + x] -
                           pred[y * n + x]);
            }
        }
        if (sad < best_sad) {
            best_sad = sad;
            best = modes[i];
        }
        if (!sad) {
            break;
        }
    }
    return best;
}

/* Syntax */

static void hevc_last(Cabac *c, int x, int y, int log2, int cidx)
{
    int offset = cidx ? 15 : 3 * (log2 - 2) + ((log2 - 1) >> 2);
    int shift = cidx ? log2 - 2 : (log2 + 1) >> 2;
    int max = (log2 << 1) - 1;
    int gx = last_group[x], gy = last_group[y];

    for (int i = 0; i < gx; i++) {
        cabac_bin(c, CTX_LAST_X + offset + (i >> shift), 1);
    }
    if (gx < max) {
        cabac_bin(c, CTX_LAST_X + offset + (gx >> shift), 0);
    }
    for (int i = 0; i < gy; i++) {
        cabac_bin(c, CTX_LAST_Y + offset + (i >> shift), 1);
    }
    if (gy < max) {
        cabac_bin(c, CTX_LAST_Y + offset + (gy >> shift), 0);
    }
    if (gx > 3) {
        cabac_bypass(c, x - last_group_min[gx], (gx >> 1) - 1);
    }
    if (gy > 3) {
        cabac_bypass(c, y - last_group_min[gy], (gy >> 1) - 1);
    }
}

static void hevc_remaining(Cabac *c, uint32_t v, int rice)
{
    if (v < (4u << rice)) {
        int q = v >> rice;

        cabac_bypass(c, (1 << (q + 1)) - 2, q + 1);
        cabac_bypass(c, v & ((1 << rice) - 1), rice);
    } else {
        int k = rice + 1, ones = 4;

        v -= 4 << rice;
        while (v >= (1u << k)) {
            v -= 1 << k;
            k++;
            ones++;
        }
        cabac_bypass(c, (1u << (ones + 1)) - 2, ones + 1);
        cabac_bypass(c, v, k);
    }
}

/* residual_coding(), the diagonal scan */
static void hevc_residual(HevcEnc *e, const int32_t *level, int log2,
                          int cidx)
{
    Cabac *c = &e->c;
    int n = 1 << log2, log2sb = log2 - 2, nsb = 1 << log2sb;
    const uint8_t (*sb_scan)[2] = tables.diag[log2sb];
    const uint8_t (*scan)[2] = tables.diag[2];
    uint8_t csbf[8][8] = { { 0 } };
    int last_sb = -1, last_pos = 0, c1 = 1;

    for (int i = nsb * nsb - 1; i >= 0 && last_sb < 0; i--) {
        for (int p = 15; p >= 0; p--) {
            int x = sb_scan[i][0] * 4 + scan[p][0];
            int y = sb_scan[i][1] * 4 + scan[p][1];

            if (level[y * n + x]) {
                last_sb = i;
                last_pos = p;
                break;
            }
        }
    }
    if (log2 == 2) {
        cabac_bin(c, CTX_TRANSFORM_SKIP + !!cidx, 0);
    }
    hevc_last(c, sb_scan[last_sb][0] * 4 + scan[last_pos][0],
              sb_scan[last_sb][1] * 4 + scan[last_pos][1], log2, cidx);

    for (int i = last_sb; i >= 0; i--) {
        int xs = sb_scan[i][0], ys = sb_scan[i][1];
        int abs_level[16], nsig = 0, first_g1 = -1, ctx_set;
        uint32_t signs = 0;
        bool infer_dc = false, escape = false;

        if (i < last_sb && i > 0) {
            int right = xs + 1 < nsb ? csbf[xs + 1][ys] : 0;
            int below = ys + 1 < nsb ? csbf[xs][ys + 1] : 0;
            bool any = false;

            for (int p = 0; p < 16 && !any; p++) {
                any = level[(ys * 4 + scan[p][1]) * n +
                            xs * 4 + scan[p][0]] != 0;
            }
            cabac_bin(c, CTX_CSBF + MIN(right + below, 1) + (cidx ? 2 : 0),
                      any);
            csbf[xs][ys] = any;
            if (!any) {
                continue;
            }
            infer_dc = true;
        } else {
            csbf[xs][ys] = 1;
        }

        for (int p = i == last_sb ? last_pos : 15; p >= 0; p--) {
            int x = xs * 4 + scan[p][0], y = ys * 4 + scan[p][1];
            int32_t v = level[y * n + x];

            if (!(i == last_sb && p == last_pos) && !(p == 0 && infer_dc)) {
                int sig;

                if (log2 == 2) {
                    sig = sig_ctx_4x4[(y << 2) + x];
                } else if (x + y == 0) {
                    sig = 0;
                } else {
                    int prev = (xs + 1 < nsb ? csbf[xs + 1][ys] : 0) |
                               (ys + 1 < nsb ? csbf[xs][ys + 1] : 0) << 1;
                    int xp = x & 3, yp = y & 3;

                    switch (prev) {
                    case 0:
                        sig = xp + yp == 0 ? 2 : xp + yp < 3 ? 1 : 0;
                        break;
                    case 1:
                        sig = yp == 0 ? 2 : yp == 1 ? 1 : 0;
                        break;
                    case 2:
                        sig = xp == 0 ? 2 : xp == 1 ? 1 : 0;
                        break;
                    default:
                        sig = 2;
                        break;
                    }
                    if (!cidx) {
                        sig += (xs || ys) ? 3 : 0;
                        sig += log2 == 3 ? 9 : 21;
                    } else {
                        sig += log2 == 3 ? 9 : 12;
                    }
                }
                cabac_bin(c, CTX_SIG + (cidx ? 27 : 0) + sig, v != 0);
                if (v) {
                    infer_dc = false;
                }
            }
            if (v) {
                abs_level[nsig] = abs(v);
                signs = signs << 1 | (v < 0);
                nsig++;
            }
        }
        if (!nsig) {
            continue;
        }

        e->st->coded_4x4++;
        ctx_set = (i > 0 && !cidx) ? 2 : 0;
        if (c1 == 0) {
            ctx_set++;
        }
        c1 = 1;
        for (int k = 0; k < MIN(nsig, 8); k++) {
            int g1 = abs_level[k] > 1;

            cabac_bin(c, CTX_GT1 + (cidx ? 16 : 0) + ctx_set * 4 + c1, g1);
            if (g1) {
                c1 = 0;
                if (first_g1 < 0) {
                    first_g1 = k;
                } else {
                    escape = true;
                }
            } else if (c1 > 0 && c1 < 3) {
                c1++;
            }
        }
        if (first_g1 >= 0) {
            int g2 = abs_level[first_g1] > 2;

            cabac_bin(c, CTX_GT2 + (cidx ? 4 : 0) + ctx_set, g2);
            escape |= g2;
        }
        cabac_bypass(c, signs, nsig);
        if (escape || nsig > 8) {
            int rice = 0, first2 = 1;

            for (int k = 0; k < nsig; k++) {
                int base = k < 8 ? 2 + first2 : 1;

                if (abs_level[k] >= base) {
                    hevc_remaining(c, abs_level[k] - base, rice);
                    if (abs_level[k] > 3 * (1 << rice)) {
                        rice = MIN(rice + 1, 4);
                    }
                }
                if (abs_level[k] >= 2) {
                    first2 = 0;
                }
            }
        }
    }
}

static void hevc_set_map(HevcEnc *e, uint8_t *map, int x0, int y0, int n,
                         int v)
{
    for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
        memset(map + y * e->w4 + (x0 >> 2), v, n >> 2);
    }
}

static void hevc_mpm(HevcEnc *e, int x0, int y0, int *cand)
{
    int a = MODE_DC, b = MODE_DC;

    if (x0 > 0) {
        a = e->mode[(y0 >> 2) * e->w4 + ((x0 - 1) >> 2)];
    }
    if (y0 > 0 && (y0 & (CTB - 1))) {
        b = e->mode[((y0 - 1) >> 2) * e->w4 + (x0 >> 2)];
    }
    if (a == b) {
        if (a < 2) {
            cand[0] = MODE_PLANAR;
            cand[1] = MODE_DC;
            cand[2] = MODE_VER;
        } else {
            cand[0] = a;
            cand[1] = 2 + ((a + 29) % 32);
            cand[2] = 2 + ((a - 2 + 1) % 32);
        }
    } else {
        cand[0] = a;
        cand[1] = b;
        cand[2] = a != MODE_PLANAR && b != MODE_PLANAR ? MODE_PLANAR :
                  a != MODE_DC && b != MODE_DC ? MODE_DC : MODE_VER;
    }
}

/*
 * cu_qp_delta_abs, a truncated unary prefix of up to 5 then EG0, and sign;
 * the decoder takes the QP modulo 52, so a step of over 26 wraps
 */
static void hevc_qp_delta(Cabac *c, int delta)
{
    int v, k = 0;

    delta += delta < -26 ? 52 : delta > 25 ? -52 : 0;
    v = abs(delta);

    for (int i = 0; i < MIN(v, 5); i++) {
        cabac_bin(c, CTX_QP_DELTA + !!i, 1);
    }
    if (v < 5) {
        cabac_bin(c, CTX_QP_DELTA + !!v, 0);
    } else {
        for (v -= 5; v >= 1 << k; k++) {
            cabac_bypass(c, 1, 1);
            v -= 1 << k;
        }
        cabac_bypass(c, 0, 1);
        cabac_bypass(c, v, k);
    }
    if (delta) {
        cabac_bypass(c, delta < 0, 1);
    }
}

static void hevc_intra_cu(HevcEnc *e, int x0, int y0, int log2)
{
    Cabac *c = &e->c;
    int n = 1 << log2, cand[3], idx = -1, mode;
    int32_t level[3][32 * 32];
    bool cbf[3];

    if (log2 == LOG2_MIN_CB) {
        cabac_bin(c, CTX_PART_MODE, 1);     /* PART_2Nx2N */
    }

    mode = hevc_pick_mode(e, x0, y0, log2);
    hevc_mpm(e, x0, y0, cand);
    for (int i = 0; i < 3; i++) {
        if (cand[i] == mode) {
            idx = i;
        }
    }
    cabac_bin(c, CTX_PREV_INTRA, idx >= 0);
    if (idx >= 0) {
        cabac_bypass(c, idx ? 2 | (idx - 1) : 0, idx ? 2 : 1);
    } else {
        int rem = mode;

        for (int i = 0; i < 3; i++) {
            rem -= cand[i] < mode;
        }
        cabac_bypass(c, rem, 5);
    }
    cabac_bin(c, CTX_CHROMA_MODE, 0);       /* 4: as the luma */
    hevc_set_map(e, e->mode, x0, y0, n, mode);

    cbf[0] = hevc_block(e, 0, x0, y0, log2, mode, level[0]);
    hevc_set_map(e, e->done, x0, y0, n, 1);
    for (int i = 1; i < 3; i++) {
        cbf[i] = hevc_block(e, i, x0 / 2, y0 / 2, log2 - 1, mode, level[i]);
    }

    cabac_bin(c, CTX_SPLIT_TU + 5 - log2, 0);
    cabac_bin(c, CTX_CBF_CHROMA, cbf[1]);
    cabac_bin(c, CTX_CBF_CHROMA, cbf[2]);
    cabac_bin(c, CTX_CBF_LUMA + 1, cbf[0]);
    if ((cbf[0] || cbf[1] || cbf[2]) && !e->qp_delta_coded) {
        hevc_qp_delta(c, e->qp - e->qp_pred);
        e->qp_delta_coded = true;
    }
    for (int i = 0; i < 3; i++) {
        if (cbf[i]) {
            hevc_residual(e, level[i], i ? log2 - 1 : log2, i);
        }
    }
    e->st->cus[log2 - LOG2_MIN_CB]++;
}

/* The squared error of a coding block's samples in @p, all components */
static uint64_t hevc_ssd(HevcEnc *e, int x0, int y0, int n,
                         const uint8_t *const p[3])
{
    uint64_t ssd = 0;

    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = n / sub, stride = e->w / sub;
        int xs = x0 / sub, ys = y0 / sub;

        for (int y = 0; y < m; y++) {
            const uint8_t *src = e->src[i] + (ys + y) * e->src_stride[i] + xs;
            const uint8_t *row = p[i] + (ys + y) * stride + xs;

            for (int x = 0; x < m; x++) {
                int d = src[x] - row[x];

                ssd += d * d;
            }
        }
    }
    return ssd;
}

/* The QpY the decoder gives a coding block coded now (8.6.1) */
static int hevc_cu_qpy(HevcEnc *e)
{
    return e->qp_delta_coded ? e->qp : e->qp_pred;
}

/*
 * A coding block of a P picture: skipped, with merge candidate 0, or intra.
 * Every motion vector is zero, so every merge candidate is the zero vector
 * on the one reference, the temporal one too; skip copies that picture.
 * The choice is by SSD + lambda * bits: the intra block is coded, and taken
 * back if skipping costs less.
 */
static void hevc_inter_cu(HevcEnc *e, int x0, int y0, int log2)
{
    Cabac *c = &e->c, saved;
    int n = 1 << log2, ctx = 0;
    IngenicAvpuStats st;
    uint64_t start;
    int64_t j_intra, j_skip;
    guint len;
    bool coded;

    if (x0 > 0) {
        ctx += e->skip[(y0 >> 2) * e->w4 + ((x0 - 1) >> 2)];
    }
    if (y0 > 0) {
        ctx += e->skip[((y0 - 1) >> 2) * e->w4 + (x0 >> 2)];
    }
    saved = *c;
    len = c->rbsp->len;
    st = *e->st;
    coded = e->qp_delta_coded;
    start = cabac_tell(c);
    cabac_bin(c, CTX_SKIP + ctx, 0);
    cabac_bin(c, CTX_PRED_MODE, 1);         /* MODE_INTRA */
    hevc_intra_cu(e, x0, y0, log2);
    if (!e->ref[0]) {
        return;
    }
    j_intra = (int64_t)hevc_ssd(e, x0, y0, n,
                                (const uint8_t *const *)e->rec) * 256 +
              e->lambda * (int64_t)(cabac_tell(c) - start);
    j_skip = (int64_t)hevc_ssd(e, x0, y0, n, e->ref) * 256 +
             e->lambda * (cabac_cost(&saved, CTX_SKIP + ctx, 1) +
                          cabac_cost(&saved, CTX_MERGE_IDX, 0)) / 256;
    if (j_skip >= j_intra) {
        return;
    }
    *c = saved;
    g_byte_array_set_size(c->rbsp, len);
    *e->st = st;
    e->qp_delta_coded = coded;
    cabac_bin(c, CTX_SKIP + ctx, 1);
    cabac_bin(c, CTX_MERGE_IDX, 0);         /* merge_idx 0 */
    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = n / sub, stride = e->w / sub;

        for (int y = 0; y < m; y++) {
            size_t off = (y0 / sub + y) * stride + x0 / sub;

            memcpy(e->rec[i] + off, e->ref[i] + off, m);
        }
    }
    hevc_set_map(e, e->mode, x0, y0, n, MODE_DC);
    hevc_set_map(e, e->done, x0, y0, n, 1);
    hevc_set_map(e, e->skip, x0, y0, n, 1);
    e->st->cus[log2 - LOG2_MIN_CB]++;
    e->st->skip_8x8 += (n / 8) * (n / 8);
}

static void hevc_coding_unit(HevcEnc *e, int x0, int y0, int log2)
{
    int n = 1 << log2;

    if (x0 > 0) {
        for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
            e->vedge[y * (e->w / 8) + (x0 >> 3)] = 1;
        }
    }
    if (y0 > 0) {
        memset(e->hedge + (y0 >> 3) * e->w4 + (x0 >> 2), 1, n >> 2);
    }
    if (e->pic->intra) {
        hevc_intra_cu(e, x0, y0, log2);
    } else {
        hevc_inter_cu(e, x0, y0, log2);
    }
    hevc_set_map(e, e->qpy, x0, y0, n, hevc_cu_qpy(e));
}

static int hevc_qp_c(int qpi)
{
    static const uint8_t table[14] = {
        29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37,
    };

    qpi = MIN(MAX(qpi, 0), 57);
    return qpi < 30 ? qpi : qpi > 43 ? qpi - 6 : table[qpi - 30];
}

/* Deblocking (8.7.2), for the picture the next one refers to */

static const uint8_t beta_table[52] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24,
    26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56,
    58, 60, 62, 64,
};

static const uint8_t tc_table[54] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4,
    4, 4, 5, 5, 6, 6, 7, 8, 9, 10, 11, 13, 14, 16, 18, 20, 22, 24,
};

/* p_i and q_i of line k of an edge segment whose q0 is at @q */
#define DB_P(i, k)  q[(k) * ys - ((i) + 1) * xs]
#define DB_Q(i, k)  q[(k) * ys + (i) * xs]

static bool deblock_strong(const uint8_t *q, int xs, int ys, int k, int dpq,
                           int beta, int tc)
{
    return dpq < (beta >> 2) &&
           abs(DB_P(3, k) - DB_P(0, k)) + abs(DB_Q(0, k) - DB_Q(3, k)) <
               (beta >> 3) &&
           abs(DB_P(0, k) - DB_Q(0, k)) < ((5 * tc + 1) >> 1);
}

/*
 * One four-line segment of a luma edge (8.7.2.5.3, 8.7.2.5.7): @xs steps
 * across the edge, @ys along it
 */
static void deblock_luma(uint8_t *q, int xs, int ys, int bs, int qp,
                         int beta_off, int tc_off)
{
    int beta = beta_table[MIN(MAX(qp + 2 * beta_off, 0), 51)];
    int tc = tc_table[MIN(MAX(qp + 2 * (bs - 1) + 2 * tc_off, 0), 53)];
    int dp0 = abs(DB_P(2, 0) - 2 * DB_P(1, 0) + DB_P(0, 0));
    int dp3 = abs(DB_P(2, 3) - 2 * DB_P(1, 3) + DB_P(0, 3));
    int dq0 = abs(DB_Q(2, 0) - 2 * DB_Q(1, 0) + DB_Q(0, 0));
    int dq3 = abs(DB_Q(2, 3) - 2 * DB_Q(1, 3) + DB_Q(0, 3));
    bool strong, dep, deq;

    if (dp0 + dq0 + dp3 + dq3 >= beta) {
        return;
    }
    strong = deblock_strong(q, xs, ys, 0, 2 * (dp0 + dq0), beta, tc) &&
             deblock_strong(q, xs, ys, 3, 2 * (dp3 + dq3), beta, tc);
    dep = dp0 + dp3 < ((beta + (beta >> 1)) >> 3);
    deq = dq0 + dq3 < ((beta + (beta >> 1)) >> 3);
    for (int k = 0; k < 4; k++) {
        int p0 = DB_P(0, k), p1 = DB_P(1, k), p2 = DB_P(2, k), p3 = DB_P(3, k);
        int q0 = DB_Q(0, k), q1 = DB_Q(1, k), q2 = DB_Q(2, k), q3 = DB_Q(3, k);

        if (strong) {
            int t = 2 * tc;

            DB_P(0, k) = MIN(MAX((p2 + 2 * p1 + 2 * p0 + 2 * q0 + q1 + 4) >> 3,
                                 p0 - t), p0 + t);
            DB_P(1, k) = MIN(MAX((p2 + p1 + p0 + q0 + 2) >> 2, p1 - t), p1 + t);
            DB_P(2, k) = MIN(MAX((2 * p3 + 3 * p2 + p1 + p0 + q0 + 4) >> 3,
                                 p2 - t), p2 + t);
            DB_Q(0, k) = MIN(MAX((p1 + 2 * p0 + 2 * q0 + 2 * q1 + q2 + 4) >> 3,
                                 q0 - t), q0 + t);
            DB_Q(1, k) = MIN(MAX((p0 + q0 + q1 + q2 + 2) >> 2, q1 - t), q1 + t);
            DB_Q(2, k) = MIN(MAX((p0 + q0 + q1 + 3 * q2 + 2 * q3 + 4) >> 3,
                                 q2 - t), q2 + t);
        } else {
            int delta = (9 * (q0 - p0) - 3 * (q1 - p1) + 8) >> 4;

            if (abs(delta) >= tc * 10) {
                continue;
            }
            delta = MIN(MAX(delta, -tc), tc);
            DB_P(0, k) = MIN(MAX(p0 + delta, 0), 255);
            DB_Q(0, k) = MIN(MAX(q0 - delta, 0), 255);
            if (dep) {
                int dp = MIN(MAX((((p2 + p0 + 1) >> 1) - p1 + delta) >> 1,
                                 -(tc >> 1)), tc >> 1);

                DB_P(1, k) = MIN(MAX(p1 + dp, 0), 255);
            }
            if (deq) {
                int dq = MIN(MAX((((q2 + q0 + 1) >> 1) - q1 - delta) >> 1,
                                 -(tc >> 1)), tc >> 1);

                DB_Q(1, k) = MIN(MAX(q1 + dq, 0), 255);
            }
        }
    }
}

/* One four-line segment of a chroma edge of bS 2 (8.7.2.5.5, 8.7.2.5.8) */
static void deblock_chroma(uint8_t *q, int xs, int ys, int qpc, int tc_off)
{
    int tc = tc_table[MIN(MAX(qpc + 2 + 2 * tc_off, 0), 53)];

    for (int k = 0; k < 4; k++) {
        int p0 = DB_P(0, k), p1 = DB_P(1, k), q0 = DB_Q(0, k), q1 = DB_Q(1, k);
        int delta = MIN(MAX((((q0 - p0) * 4) + p1 - q1 + 4) >> 3, -tc), tc);

        DB_P(0, k) = MIN(MAX(p0 + delta, 0), 255);
        DB_Q(0, k) = MIN(MAX(q0 - delta, 0), 255);
    }
}

#undef DB_P
#undef DB_Q

/*
 * The vertical edges of the whole picture, then its horizontal ones. Every
 * coding block is one transform block, so the edges are the coding blocks';
 * bS is 2 where a side is intra, else 0, both sides being skipped with the
 * zero vector.
 */
static void hevc_deblock(HevcEnc *e)
{
    int beta_off = e->pic->beta_div2, tc_off = e->pic->alpha_c0_div2;
    uint32_t w8 = e->w / 8, cw = e->w / 2;

    for (int dir = 0; dir < 2; dir++) {
        for (uint32_t y = dir ? 8 : 0; y < e->h; y += dir ? 8 : 4) {
            for (uint32_t x = dir ? 0 : 8; x < e->w; x += dir ? 4 : 8) {
                uint32_t pb, qb = (y >> 2) * e->w4 + (x >> 2);
                int bs, qp;

                if (!(dir ? e->hedge[(y >> 3) * e->w4 + (x >> 2)]
                          : e->vedge[(y >> 2) * w8 + (x >> 3)])) {
                    continue;
                }
                pb = dir ? qb - e->w4 : qb - 1;
                bs = e->skip[pb] && e->skip[qb] ? 0 : 2;
                if (!bs) {
                    continue;
                }
                qp = (e->qpy[pb] + e->qpy[qb] + 1) >> 1;
                deblock_luma(e->rec[0] + y * e->w + x, dir ? e->w : 1,
                             dir ? 1 : e->w, bs, qp, beta_off, tc_off);
                /*
                 * chroma edges lie on its own 8x8 grid, a segment of four
                 * lines takes the bS of the first luma one it spans
                 */
                if ((dir ? y : x) % 16 || (dir ? x : y) % 8) {
                    continue;
                }
                for (int i = 1; i < 3; i++) {
                    int off = i == 1 ? e->pic->chroma_qp_offset
                                     : e->pic->cr_qp_offset;

                    deblock_chroma(e->rec[i] + (y / 2) * cw + x / 2,
                                   dir ? cw : 1, dir ? 1 : cw,
                                   hevc_qp_c(qp + off), tc_off);
                }
            }
        }
    }
}

static void hevc_quadtree(HevcEnc *e, int x0, int y0, int log2, int depth)
{
    int n = 1 << log2;
    bool split;

    if (x0 + n <= e->w && y0 + n <= e->h && log2 > LOG2_MIN_CB) {
        int ctx = 0;

        if (x0 > 0) {
            ctx += e->depth[(y0 >> 2) * e->w4 + ((x0 - 1) >> 2)] > depth;
        }
        if (y0 > 0) {
            ctx += e->depth[((y0 - 1) >> 2) * e->w4 + (x0 >> 2)] > depth;
        }
        split = false;
        cabac_bin(&e->c, CTX_SPLIT_CU + ctx, split);
    } else {
        split = log2 > LOG2_MIN_CB;
    }
    if (split) {
        int h = n / 2;

        for (int i = 0; i < 4; i++) {
            int x = x0 + (i & 1) * h, y = y0 + (i >> 1) * h;

            if (x < e->w && y < e->h) {
                hevc_quadtree(e, x, y, log2 - 1, depth + 1);
            }
        }
        return;
    }
    hevc_set_map(e, e->depth, x0, y0, n, depth);
    hevc_coding_unit(e, x0, y0, log2);
}

IngenicAvpuHevc *ingenic_avpu_hevc_new(void)
{
    return g_new0(IngenicAvpuHevc, 1);
}

void ingenic_avpu_hevc_free(IngenicAvpuHevc *h)
{
    if (h) {
        g_free(h->ref);
        g_free(h);
    }
}

bool ingenic_avpu_hevc_encode(IngenicAvpuHevc *h, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st)
{
    g_autoptr(GByteArray) rbsp = g_byte_array_new();
    g_autofree uint8_t *planes = NULL, *maps = NULL;
    uint32_t cw, ch, ctbs_w, ctbs_h, w8, size;
    HevcEnc e = { .pic = pic, .st = st };

    if (!pic->width || !pic->height || (pic->width | pic->height) & 7 ||
        pic->width > 8192 || pic->height > 8192 || pic->qp > 51) {
        return false;
    }
    hevc_tables_init();
    e.w = pic->width;
    e.h = pic->height;
    e.w4 = e.w / 4;
    e.h4 = e.h / 4;
    w8 = e.w / 8;
    cw = e.w / 2;
    ch = e.h / 2;
    size = e.w * e.h + 2 * cw * ch;

    /* Deinterleave the chroma; reconstruct into planes of our own */
    planes = g_malloc(e.w * e.h + 4 * cw * ch);
    e.rec[0] = planes;
    e.rec[1] = e.rec[0] + e.w * e.h;
    e.rec[2] = e.rec[1] + cw * ch;
    e.src[0] = y;
    e.src_stride[0] = stride;
    e.src[1] = e.rec[2] + cw * ch;
    e.src[2] = e.src[1] + cw * ch;
    e.src_stride[1] = e.src_stride[2] = cw;
    for (uint32_t j = 0; j < ch; j++) {
        for (uint32_t i = 0; i < cw; i++) {
            ((uint8_t *)e.src[1])[j * cw + i] = uv[j * stride + 2 * i];
            ((uint8_t *)e.src[2])[j * cw + i] = uv[j * stride + 2 * i + 1];
        }
    }
    maps = g_malloc0(5 * e.w4 * e.h4 + w8 * e.h4 + e.w4 * (e.h / 8));
    e.done = maps;
    e.mode = maps + e.w4 * e.h4;
    e.depth = maps + 2 * e.w4 * e.h4;
    e.skip = maps + 3 * e.w4 * e.h4;
    e.qpy = maps + 4 * e.w4 * e.h4;
    e.vedge = maps + 5 * e.w4 * e.h4;
    e.hedge = e.vedge + w8 * e.h4;

    /* the channel's last picture, if this P picture can refer to it */
    if (!pic->intra && h && h->ref && h->w == e.w && h->h == e.h) {
        e.ref[0] = h->ref;
        e.ref[1] = e.ref[0] + e.w * e.h;
        e.ref[2] = e.ref[1] + cw * ch;
    }

    memset(st, 0, sizeof(*st));
    cabac_init(&e.c, pic->intra ? 0 : pic->cabac_init_idc ? 2 : 1, pic->qp,
               rbsp);
    ctbs_w = DIV_ROUND_UP(e.w, CTB);
    ctbs_h = DIV_ROUND_UP(e.h, CTB);
    e.qp_pred = pic->qp;
    for (uint32_t j = 0; j < ctbs_h; j++) {
        for (uint32_t i = 0; i < ctbs_w; i++) {
            /*
             * A CTB is a quantization group: its QP goes with its first
             * coded residual, and without one the decoder keeps the last
             */
            e.qp = pic->block_qp ? pic->block_qp[j * ctbs_w + i] : pic->qp;
            e.qp_c[0] = hevc_qp_c(e.qp + pic->chroma_qp_offset);
            e.qp_c[1] = hevc_qp_c(e.qp + pic->cr_qp_offset);
            e.lambda = lround(256 * 0.57 * pow(2, (e.qp - 12) / 3.0));
            e.qp_delta_coded = false;
            hevc_quadtree(&e, i * CTB, j * CTB, LOG2_CTB, 0);
            if (e.qp_delta_coded) {
                e.qp_pred = e.qp;
            }
            cabac_terminate(&e.c, i == ctbs_w - 1 && j == ctbs_h - 1);
        }
    }
    cabac_finish(&e.c);
    st->bins = e.c.bins;
    ingenic_avpu_escape(out, rbsp->data, rbsp->len);

    if (h) {
        hevc_deblock(&e);
        if (h->w != e.w || h->h != e.h) {
            g_free(h->ref);
            h->ref = g_malloc(size);
            h->w = e.w;
            h->h = e.h;
        }
        memcpy(h->ref, planes, size);
    }
    return true;
}
