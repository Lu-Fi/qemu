/*
 * Ingenic AVPU and RADIX: H.265 slice data
 *
 * For H.265 libimp writes the parameter sets and the slice segment header
 * itself, after the encode, and the encoder writes only the slice data:
 * CABAC, from the byte after the header, ended by the stop bit, emulation
 * prevented by the AVPU and raw from the T30's RADIX. libimp's streams are
 * Main profile with coding blocks down to 8 and transform blocks of 4 to
 * 32; what else they fix, IngenicAvpuHevcSyntax, differs by encoder. The
 * AVPU's have CTBs of 32, the default scaling lists (flat ones if entry word
 * 9 says), transform skip where the entry says (libimp's eEncTools bit 7:
 * on for the T31's and T40's channels, off for their YUV encoder and for
 * the T41's) and cabac_init_flag in P slices, which take the contexts of
 * initType 2; the RADIX's have CTBs of 64, flat lists, quantization groups
 * of 16x16 and SAO for luma.
 *
 * This codes every coding block at the QP the caller picks for its CTB
 * (ingenic-avpu.c): 32x32, or 16x16 and 8x8 where the picture's edge cuts a
 * CTB, each one transform block; a CTB of 64 splits once. An intra block is
 * predicted vertically, horizontally or DC, the chroma as the luma; 8x8
 * blocks are DC only, which keeps every scan the diagonal one. In a P
 * picture a block is either that, skipped (a copy of the channel's last
 * picture) or merged with it, no vector, with the residual transformed and
 * quantized: libimp's P slices refer to the picture before, alone. That
 * picture is kept deblocked, as the decoder has it; where SAO is on, every
 * CTB leaves it off.
 *
 * Mode decisions and reconstruction read only the picture, so rows of CTBs
 * make them on helper threads, a row behind the row above. What alone knows
 * the contexts the skip decisions are made with is the entropy coder: it
 * takes each block's candidate in order on the calling thread, and makes it
 * again where a thread's guess of the neighbours was wrong. The stream is
 * what one thread writes, byte for byte, at any thread count. A P picture of
 * the AVPU or HERA that searches for motion (h265_search) is coded on the
 * calling thread alone, as the search below describes.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/atomic.h"
#include "qemu/thread.h"
#include "qemu/processor.h"
#include "hw/misc/ingenic-avpu.h"
#include "migration/qemu-file-types.h"
#ifdef CONFIG_AVX2_OPT
#include <immintrin.h>
#elif defined(__ARM_NEON)
#include <arm_neon.h>
#endif

#define LOG2_MIN_CB     3
#define LOG2_MAX_CB     5       /* the largest transform block */

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
    CTX_SAO_MERGE = 136,    /* left and up */
    CTX_SAO_TYPE = 137,     /* luma and chroma */
    CTX_MERGE_FLAG = 138,   /* the motion search's: P pictures only */
    CTX_MVP = 139,
    CTX_RQT_ROOT = 140,
    CTX_MVD_GT0 = 141,
    CTX_MVD_GT1 = 142,
    CTX_COUNT = 143,
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
        153,
        200,
        154, 168, 79, 140, 198,
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
        153,
        185,
        110, 168, 79, 140, 198,
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
        153,
        160,
        154, 168, 79, 169, 198,
    },
};

static const IngenicAvpuHevcSyntax avpu_syntax = {
    .log2_ctb = 5,
    .tu_depth = 1,
    .qp_delta_depth = 0,
    .merge_cands = 5,
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

/* The default inter scaling list of 8x8 and larger blocks, by row */
static const uint8_t scaling_inter[64] = {
    16, 16, 16, 16, 17, 18, 20, 24,
    16, 16, 16, 17, 18, 20, 24, 25,
    16, 16, 17, 18, 20, 24, 25, 28,
    16, 17, 18, 20, 24, 25, 28, 33,
    17, 18, 20, 24, 25, 28, 33, 41,
    18, 20, 24, 25, 28, 33, 41, 54,
    20, 24, 25, 28, 33, 41, 54, 71,
    24, 25, 28, 33, 41, 54, 71, 91,
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
    uint8_t scale_inter[4][32 * 32];    /* the default inter lists */
    uint16_t quant_inter[6][4][32 * 32];
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
    /* count: only adapt the contexts and add up what the bins would cost */
    bool count;
    uint64_t cost;              /* 1/256 bit */
} Cabac;

/* A motion vector in quarter samples */
typedef struct HevcMv {
    int16_t x, y;
} HevcMv;

/* What a picture left of its motion at 16x16 (8.5.3.2.8 compresses to it) */
typedef struct HevcMvf {
    HevcMv mv;
    bool inter;
} HevcMvf;

/* A channel's last picture, deblocked: what its next P picture refers to */
struct IngenicAvpuHevc {
    uint32_t w, h;
    uint8_t *ref;               /* Y, then Cb and Cr */
    uint8_t *spare;             /* the next picture's, to swap with ref */
    HevcMvf *mvf;               /* ref's motion, by 16x16: not migrated */
    uint32_t *mvrec;            /* the last search's records for the guest */
    size_t mvrec_n;
};

/*
 * What the mode decisions and the reconstruction make of a coding block
 * before any bit is written: they read only the picture, never the coder's
 * state, so rows of CTBs can make them on several threads. Each carries
 * the reference samples it was made from, which the entropy coder checks
 * against the final neighbours where a thread had to guess them.
 */
typedef struct HevcCand {
    uint16_t x, y;
    uint8_t log2;
    uint8_t mode;
    uint8_t cbf;                /* bit i: luma, Cb, Cr */
    bool ready;                 /* the rest is made: else only ssd_skip is */
    bool in_rec;                /* its reconstruction is in the picture's */
    int16_t *level[3];          /* the levels of each block with a cbf */
    uint8_t *refs;              /* reference samples: luma, Cb, Cr */
    uint64_t ssd_intra, ssd_skip;   /* P pictures: against the source */
    /*
     * P pictures: the last picture's samples with a residual (merge, zero
     * vector), made from the source and the reference alone; icbf is 0 where
     * there is no residual, which makes the block a skipped one
     */
    bool iready;
    uint8_t icbf;
    int16_t *ilevel[3];
    uint64_t ssd_inter;
} HevcCand;

typedef struct HevcInter HevcInter;
struct HevcSnap;

/*
 * Buffers of the search's inner functions: the build zeroes a local array at
 * each call, and these run thousands of times a picture
 */
typedef struct HevcScratch {
    uint8_t blk[(32 + 7) * (32 + 7)];
    int16_t tmp[(32 + 7) * 32];
    uint8_t pred[32 * 32];
    int16_t res[32 * 32];
    int32_t level[32 * 32];
} HevcScratch;

typedef struct HevcEnc {
    const IngenicAvpuPic *pic;
    const IngenicAvpuHevcSyntax *syn;
    int log2_qg;                /* Log2MinCuQpDeltaSize */
    uint32_t w, h, w4, h4;
    uint32_t ctbs_w, ctbs_h;
    int qp, qp_c[2];
    const uint8_t *src[3];      /* Y, Cb, Cr */
    uint32_t src_stride[3];
    uint8_t *rec[3];            /* the picture as the decoder has it */
    uint8_t *cand[3];           /* reconstruction of the intra candidates */
    const uint8_t *ref[3];      /* NULL in an I picture */
    uint32_t *order;            /* coding order of the block, by 4x4 luma */
    HevcCand *cands;            /* every coding block, in coding order */
    int16_t *level_arena;       /* what cands' levels and refs point into */
    uint8_t *refs_arena;
    size_t level_used, refs_used;
    uint32_t *ctb_first;        /* the first of each CTB's, then the end */
    uint32_t cu_next;           /* the entropy coder's cursor in cands */
    HevcCand scratch;           /* one thread: the block being coded */
    int16_t scratch_level[32 * 32 * 3];
    uint8_t scratch_refs[8 * 32 + 3];
    bool exact;                 /* threads' reconstruction is the final one */
    bool shadow;                /* a thread's count of the entropy coder */
    uint8_t *spec_mode, *spec_skip;     /* what a thread guesses of those */
    uint8_t *spec_inter;        /* and of the blocks coded with a residual */
    uint8_t *inter;             /* a thread's: coded inter with a residual */
    uint8_t *bsmap;             /* deblocking: 1 intra, 2 luma levels, by 4x4 */
    uint8_t *irec[3];           /* the inter candidates' reconstruction */
    uint8_t *mode;              /* luma intra mode, by 4x4 luma block */
    uint8_t *depth;             /* coding quadtree depth, by 4x4 luma block */
    uint8_t *skip;              /* cu_skip_flag, by 4x4 luma block */
    uint8_t *qpy;               /* the decoder's QpY, by 4x4 luma block */
    uint8_t *vedge;             /* a coding block's left edge, by 4 rows x 8 */
    uint8_t *hedge;             /* its top edge, by 8 rows x 4 columns */
    int64_t lambda;             /* the CTB's, skip against intra, 1/256 */
    int qp_pred;                /* the decoder's, for this quantization group */
    bool qp_delta_coded;
    /* P pictures that search for motion */
    bool search;
    const uint8_t *pad;         /* the reference's luma with a border */
    uint32_t pad_stride;
    uint8_t *dsrc, *dpad;       /* the source and reference at half size */
    uint32_t dsrc_stride, dpad_stride;
    uint32_t seq;               /* coding blocks coded so far */
    HevcMv *mvmap;              /* by 4x4 luma block */
    uint8_t *imap;              /* inter: coded, not intra */
    uint8_t *cmap;              /* the block's transform block has levels */
    uint8_t *done;              /* coded already */
    const HevcMvf *col;         /* the reference's motion, or NULL */
    uint32_t w16;
    int lam_me;                 /* a vector's bit against a sample of SAD */
    uint8_t *maps;              /* imap, cmap and done */
    /* the CTB's best whole samples: 32x32, then the four 16x16 */
    HevcMv ime[5];
    bool ime_ok;
    int ime_x, ime_y;
    HevcInter *ic;              /* a coding block's inter options */
    HevcScratch *sc;
    struct HevcSnap *snap;
    Cabac c;
    IngenicAvpuStats *st;
} HevcEnc;

/* One thread's view for a coding block's candidate */
typedef struct HevcAn {
    const HevcEnc *e;
    const uint8_t *nb[3];       /* neighbouring samples are read from here */
    const uint8_t *skipmap;     /* where it marks a block, from refp instead */
    const uint8_t *refp[3];
    const uint8_t *intermap;    /* likewise, from irecp */
    const uint8_t *irecp[3];
    uint8_t *out[3];            /* the candidate's reconstruction */
    uint32_t cur;               /* the block's place in coding order */
    int qp, qp_c[2];
    int64_t lambda;             /* the CTB's, for a guess at skipping */
} HevcAn;

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
    memset(tables.scale_inter, 16, sizeof(tables.scale_inter));
    for (int log2 = 3; log2 <= 5; log2++) {
        int n = 1 << log2, rep = n / 8;

        for (int y = 0; y < 8; y++) {
            for (int x = 0; x < 8; x++) {
                for (int j = 0; j < rep; j++) {
                    for (int k = 0; k < rep; k++) {
                        tables.scale_inter[log2 - 2][(y * rep + j) * n +
                                                     x * rep + k] =
                            scaling_inter[y * 8 + x];
                    }
                }
            }
        }
        tables.scale_inter[log2 - 2][0] = 16;
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
    for (int q = 0; q < 6; q++) {
        for (int s = 0; s < 4; s++) {
            for (int i = 0; i < (16 << 2 * s); i++) {
                tables.quant_inter[q][s][i] = (quant_scale[q] << 4) /
                                              tables.scale_inter[s][i];
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
    uint32_t lps;

    if (c->count) {
        c->cost += tables.bitcost[*s >> 1][bin != (*s & 1)];
        if (bin != (*s & 1)) {
            *s = (*s >> 1) ? lps_next[*s >> 1] << 1 | (*s & 1) : (*s ^ 1);
        } else if ((*s >> 1) < 62) {
            *s += 2;
        }
        return;
    }
    lps = lps_table[*s >> 1][(c->range >> 6) & 3];
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
    if (c->count) {
        c->cost += 256 * n;
        return;
    }
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
    if (c->count) {
        return c->cost >> 8;
    }
    return 8 * ((uint64_t)c->rbsp->len + c->buffered) + 23 - c->bits_left;
}

/* What coding @bin with context @ctx would cost now, in 1/256 bits */
static int cabac_cost(const Cabac *c, int ctx, int bin)
{
    uint8_t s = c->state[ctx];

    return tables.bitcost[s >> 1][bin != (s & 1)];
}

/* How many bits the register shifts by coding @bin with context @ctx now */
static int cabac_shifts(const Cabac *c, int ctx, int bin)
{
    uint8_t s = c->state[ctx];
    uint32_t lps = lps_table[s >> 1][(c->range >> 6) & 3];
    int n = 0;

    if (bin != (s & 1)) {
        while ((lps << n) < 256) {
            n++;
        }
        return n;
    }
    return c->range - lps >= 256 ? 0 : 1;
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


/*
 * The reference samples of an n x n block at (x0, y0) of component cidx,
 * substituted as 8.4.4.2.2 has it: ref[2n] is the corner, ref[2n - 1 - k]
 * the left column's k-th, ref[2n + 1 + k] the top row's k-th.
 */
static void hevc_ref(const HevcAn *a, int cidx, int x0, int y0, int n,
                     uint8_t *ref)
{
    const HevcEnc *e = a->e;
    int sub = cidx ? 2 : 1, stride = e->w / sub, count = 4 * n + 1;
    bool avail[4 * 32 + 1], any = false, last_avail = false;
    const uint8_t *last_rec = NULL;
    uint32_t last_blk = UINT32_MAX;
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
        /* available: coded before the block, which is by coding order */
        avail[i] = false;
        if (x >= 0 && y >= 0 && x * sub < e->w && y * sub < e->h) {
            uint32_t blk = ((y * sub) >> 2) * e->w4 + ((x * sub) >> 2);

            if (blk != last_blk) {
                last_blk = blk;
                last_avail = e->order[blk] < a->cur;
                last_rec = a->skipmap && a->skipmap[blk] ? a->refp[cidx] :
                           a->intermap && a->intermap[blk] ? a->irecp[cidx] :
                                                             a->nb[cidx];
            }
            avail[i] = last_avail;
            if (avail[i]) {
                ref[i] = last_rec[y * stride + x];
            }
        }
        if (avail[i]) {
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
 * The transforms run every column of a block at once: a matrix of n rows
 * of n samples, each row a few four-lane vectors. Row k of the n-point
 * matrix is basis k * step of the 32-point one, whose rows are symmetric
 * (k even) or antisymmetric (k odd), so a forward transform's odd outputs
 * take the differences of the input's halves and its even ones the n / 2-
 * point transform of their sums, a quarter of the multiplications and the
 * same integer sums. Where a host can, the same code is built for AVX2 and
 * picked at run time.
 */
#if defined(CONFIG_AVX2_OPT) && defined(CONFIG_LINUX)
#define HEVC_DCT_CLONES __attribute__((target_clones("avx2", "default")))
#else
#define HEVC_DCT_CLONES
#endif

typedef int32_t hevc_v4s __attribute__((vector_size(16), aligned(4),
                                        may_alias));

#define V4(p, i) (((hevc_v4s *)(p))[i])

/* @out[k] of n rows of @n lanes: sum over i of basis k at i times @in[i] */
HEVC_DCT_CLONES
static void dct_fwd(int n, int step, int lanes, int32_t *in, int32_t *out)
{
    int h = n / 2, nv = lanes / 4;
    QEMU_UNINITIALIZED int32_t e[16 * 32], o[16 * 32], eo[16 * 32];

    if (n == 1) {
        for (int v = 0; v < nv; v++) {
            V4(out, v) = V4(in, v) * tables.dct[0][0];
        }
        return;
    }
    for (int i = 0; i < h; i++) {
        for (int v = 0; v < nv; v++) {
            hevc_v4s a = V4(in + i * lanes, v);
            hevc_v4s b = V4(in + (n - 1 - i) * lanes, v);

            V4(e + i * lanes, v) = a + b;
            V4(o + i * lanes, v) = a - b;
        }
    }
    dct_fwd(h, step * 2, lanes, e, eo);
    for (int k = 0; k < h; k++) {
        const int8_t *row = tables.dct[(2 * k + 1) * step];

        for (int v = 0; v < nv; v++) {
            hevc_v4s sum = V4(o, v) * row[0];

            for (int i = 1; i < h; i++) {
                sum += V4(o + i * lanes, v) * row[i];
            }
            V4(out + (2 * k + 1) * lanes, v) = sum;
            V4(out + 2 * k * lanes, v) = V4(eo + k * lanes, v);
        }
    }
}

/* The inverse: out[i] is the sum over k of basis k at i times @d[k] */
HEVC_DCT_CLONES
static void dct_inv(int n, int step, int lanes, int32_t *d, int32_t *out)
{
    int h = n / 2, nv = lanes / 4;
    QEMU_UNINITIALIZED int32_t ev[16 * 32], eo[16 * 32];
    bool nz[32];

    if (n == 1) {
        for (int v = 0; v < nv; v++) {
            V4(out, v) = V4(d, v) * tables.dct[0][0];
        }
        return;
    }
    for (int k = 0; k < n; k++) {
        hevc_v4s any = { 0 };

        for (int v = 0; v < nv; v++) {
            any |= V4(d + k * lanes, v);
        }
        nz[k] = any[0] | any[1] | any[2] | any[3];
    }
    for (int j = 0; j < h; j++) {
        memcpy(ev + j * lanes, d + 2 * j * lanes, lanes * sizeof(*d));
    }
    dct_inv(h, step * 2, lanes, ev, eo);
    for (int i = 0; i < h; i++) {
        for (int v = 0; v < nv; v++) {
            hevc_v4s odd = { 0 }, even = V4(eo + i * lanes, v);

            for (int k = 1; k < n; k += 2) {
                if (nz[k]) {
                    odd += V4(d + k * lanes, v) * tables.dct[k * step][i];
                }
            }
            V4(out + i * lanes, v) = even + odd;
            V4(out + (n - 1 - i) * lanes, v) = even - odd;
        }
    }
}

/*
 * The HM encoder's forward DCT: first along the rows of the residual
 * (tmp[k][y]), then down the columns of that
 */
static void hevc_fdct(const int16_t *res, int log2, int32_t *coef)
{
    int n = 1 << log2, step = 32 >> log2;
    int shift1 = log2 - 1, shift2 = log2 + 6;
    QEMU_UNINITIALIZED int32_t in[32 * 32], tmp[32 * 32];

    for (int y = 0; y < n; y++) {
        for (int i = 0; i < n; i++) {
            in[i * n + y] = res[y * n + i];
        }
    }
    dct_fwd(n, step, n, in, tmp);
    for (int i = 0; i < n; i++) {
        for (int x = 0; x < n; x++) {
            in[x * n + i] = (tmp[i * n + x] + (1 << (shift1 - 1))) >> shift1;
        }
    }
    dct_fwd(n, step, n, in, coef);
    for (int i = 0; i < n * n; i++) {
        coef[i] = (coef[i] + (1 << (shift2 - 1))) >> shift2;
    }
}

/*
 * 8.6.4.2, the decoder's: residuals from the scaled coefficients. With
 * 16-bit inputs the sums fit 32 bits.
 */
static void hevc_idct(int32_t *d, int log2, int16_t *res)
{
    int n = 1 << log2, step = 32 >> log2;
    QEMU_UNINITIALIZED int32_t tmp[32 * 32], out[32 * 32];

    dct_inv(n, step, n, d, tmp);
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            out[x * n + y] = MIN(MAX((tmp[y * n + x] + 64) >> 7, -32768),
                                 32767);
        }
    }
    dct_inv(n, step, n, out, tmp);
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            res[y * n + x] = (tmp[x * n + y] + 2048) >> 12;
        }
    }
}

/*
 * Sign data hiding: where a 4x4 group's significant levels span more than
 * four scan positions, the decoder takes the sign of the first from the
 * parity of their sum. Where that parity is wrong, step by one the level,
 * from the first to the last, that moves least from its unrounded value
 * (@q, << qbits, signed).
 */
static void hevc_hide_signs(int32_t *level, const int64_t *q, int log2,
                            int qbits)
{
    int n = 1 << log2, nsb = n / 4;
    const uint8_t (*scan)[2] = tables.diag[2];

    for (int ys = 0; ys < nsb; ys++) {
        for (int xs = 0; xs < nsb; xs++) {
            int pos[16], first = -1, last = -1, best = -1, step = 0;
            int64_t best_cost = INT64_MAX;
            uint32_t sum = 0;

            for (int p = 0; p < 16; p++) {
                pos[p] = (ys * 4 + scan[p][1]) * n + xs * 4 + scan[p][0];
                if (level[pos[p]]) {
                    first = first < 0 ? p : first;
                    last = p;
                    sum += abs(level[pos[p]]);
                }
            }
            if (last - first <= 3 || (sum & 1) == (level[pos[first]] < 0)) {
                continue;
            }
            for (int p = first; p <= last; p++) {
                int64_t u = llabs(q[pos[p]]), l = abs(level[pos[p]]);
                int64_t cur = llabs((l << qbits) - u);

                for (int s = -1; s <= 1; s += 2) {
                    int64_t cost = llabs(((l + s) << qbits) - u) - cur;

                    /* the first and last stay significant */
                    if (l + s < (p == first || p == last) || l + s > 32767) {
                        continue;
                    }
                    if (cost < best_cost) {
                        best_cost = cost;
                        best = pos[p];
                        step = s;
                    }
                }
            }
            if (level[best] ? level[best] < 0 : q[best] < 0) {
                step = -step;
            }
            level[best] += step;
        }
    }
}

typedef uint32_t hevc_v4u __attribute__((vector_size(16)));
typedef int64_t hevc_v4q __attribute__((vector_size(32)));
typedef int64_t hevc_v4qm __attribute__((vector_size(32), aligned(8),
                                         may_alias));
typedef uint64_t hevc_v4uq __attribute__((vector_size(32)));

/*
 * The levels of @size coefficients, (|c| * @qc + @offset) >> @qbits at most
 * 32767, with the coefficient's sign; @q, if given, takes |c| * @qc with it.
 * Whether any level is not 0.
 */
HEVC_DCT_CLONES
static bool hevc_levels(int32_t *coef, const uint16_t *qc, int size,
                        int64_t offset, int qbits, int32_t *level,
                        int64_t *q)
{
    hevc_v4uq any = { 0 };
    const hevc_v4uq off = { offset, offset, offset, offset };
    const hevc_v4uq cap = { 32767, 32767, 32767, 32767 };

    for (int i = 0; i < size; i += 4) {
        hevc_v4s c = V4(coef, i / 4), neg = c < 0;
        hevc_v4u a = (hevc_v4u)((c ^ neg) - neg);
        hevc_v4uq u, l, big;
        uint16_t w[4] = { qc[i], qc[i + 1], qc[i + 2], qc[i + 3] };
        hevc_v4u f = { w[0], w[1], w[2], w[3] };

        u = __builtin_convertvector(a, hevc_v4uq) *
            __builtin_convertvector(f, hevc_v4uq);
        l = (u + off) >> qbits;
        big = (hevc_v4uq)(l > cap);
        l = (l & ~big) | (cap & big);
        any |= l;
        V4(level, i / 4) = (__builtin_convertvector(l, hevc_v4s) ^ neg) - neg;
        if (q) {
            hevc_v4q nq = __builtin_convertvector(neg, hevc_v4q);

            *(hevc_v4qm *)(q + i) = ((hevc_v4q)u ^ nq) - nq;
        }
    }
    return any[0] | any[1] | any[2] | any[3];
}

/*
 * Quantize the residual of an n x n block; returns whether any level is
 * left, and then the decoder's residual from them in res.
 */
static bool hevc_quant(int16_t *res, int log2, int qp, bool flat, bool sdh,
                       bool inter, int round, int32_t *level)
{
    int n = 1 << log2, size = n * n;
    const uint8_t *m = inter && !flat ? tables.scale_inter[log2 - 2] :
                                        tables.scale[flat][log2 - 2];
    const uint16_t *qc = inter && !flat ?
                         tables.quant_inter[qp % 6][log2 - 2] :
                         tables.quant[flat][qp % 6][log2 - 2];
    int qbits = 21 + qp / 6 - log2, bd_shift = 3 + log2;
    int64_t offset = (int64_t)round << (qbits - 9);
    QEMU_UNINITIALIZED int64_t q[32 * 32];
    QEMU_UNINITIALIZED int32_t coef[32 * 32], d[32 * 32];
    bool any = false;

    for (int i = 0; i < size && !any; i++) {
        any = res[i] != 0;
    }
    if (!any) {
        memset(level, 0, size * sizeof(*level));
        return false;
    }
    hevc_fdct(res, log2, coef);
    any = hevc_levels(coef, qc, size, offset, qbits, level, sdh ? q : NULL);
    if (!any) {
        return false;
    }
    if (sdh) {
        hevc_hide_signs(level, q, log2, qbits);
    }
    for (int i = 0; i < size; i++) {
        int64_t v = ((int64_t)level[i] * m[i] * level_scale[qp % 6]
                     << (qp / 6)) + (1 << (bd_shift - 1));

        d[i] = level[i] ? MIN(MAX(v >> bd_shift, -32768), 32767) : 0;
    }
    hevc_idct(d, log2, res);
    return true;
}

/*
 * Predict from the reference samples @ref, quantize and reconstruct one
 * block; returns its cbf
 */
static bool hevc_block(const HevcAn *a, int cidx, int x0, int y0, int log2,
                       int mode, const uint8_t *ref, int32_t *level)
{
    const HevcEnc *e = a->e;
    int n = 1 << log2, sub = cidx ? 2 : 1, stride = e->w / sub;
    QEMU_UNINITIALIZED uint8_t pred[32 * 32];
    QEMU_UNINITIALIZED int16_t res[32 * 32];
    const uint8_t *src = e->src[cidx];
    uint8_t *rec = a->out[cidx];
    bool cbf;

    hevc_predict(mode, cidx, n, ref, pred);
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            res[y * n + x] = src[(y0 + y) * e->src_stride[cidx] + x0 + x] -
                             pred[y * n + x];
        }
    }
    cbf = hevc_quant(res, log2, cidx ? a->qp_c[cidx - 1] : a->qp,
                     !e->pic->scaling_list, e->syn->sign_hiding, false, 171,
                     level);
    for (int y = 0; y < n; y++) {
        for (int x = 0; x < n; x++) {
            int v = pred[y * n + x] + (cbf ? res[y * n + x] : 0);

            rec[(y0 + y) * stride + x0 + x] = MIN(MAX(v, 0), 255);
        }
    }
    return cbf;
}

/* The intra mode that predicts a luma block best, by SAD */
static int hevc_pick_mode(const HevcAn *a, int x0, int y0, int log2,
                          const uint8_t *ref)
{
    static const uint8_t modes[] = { MODE_VER, MODE_HOR, MODE_DC };
    const HevcEnc *e = a->e;
    int n = 1 << log2, best = MODE_DC;
    uint32_t best_sad = UINT32_MAX;
    QEMU_UNINITIALIZED uint8_t pred[32 * 32];

    if (log2 == LOG2_MIN_CB) {
        return MODE_DC;
    }
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
static void hevc_residual(HevcEnc *e, const int16_t *level, int log2,
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
    if (log2 == 2 && e->syn->transform_skip) {
        cabac_bin(c, CTX_TRANSFORM_SKIP + !!cidx, 0);
    }
    hevc_last(c, sb_scan[last_sb][0] * 4 + scan[last_pos][0],
              sb_scan[last_sb][1] * 4 + scan[last_pos][1], log2, cidx);

    for (int i = last_sb; i >= 0; i--) {
        int xs = sb_scan[i][0], ys = sb_scan[i][1];
        int abs_level[16], nsig = 0, first_g1 = -1, ctx_set;
        int first_p = -1, last_p = -1;
        uint32_t signs = 0;
        bool infer_dc = false, escape = false, hidden;

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
                last_p = last_p < 0 ? p : last_p;
                first_p = p;
            }
        }
        if (!nsig) {
            continue;
        }
        /* the first one's sign in the parity, hevc_hide_signs() */
        hidden = e->syn->sign_hiding && last_p - first_p > 3;

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
        cabac_bypass(c, signs >> hidden, nsig - hidden);
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
    if (y0 > 0 && (y0 & ((1 << e->syn->log2_ctb) - 1))) {
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

static int hevc_qp_c(int qpi)
{
    static const uint8_t table[14] = {
        29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37,
    };

    qpi = MIN(MAX(qpi, 0), 57);
    return qpi < 30 ? qpi : qpi > 43 ? qpi - 6 : table[qpi - 30];
}

/*
 * The squared error of @n rows of @m samples; it fits 32 bits up to 32x32.
 * A row a constant length is one the compiler takes in vectors.
 */
#define HEVC_SSD_ROWS(M)                                                    \
    for (int y = 0; y < n; y++, a += a_stride, b += b_stride) {             \
        for (int x = 0; x < (M); x++) {                                     \
            int d = a[x] - b[x];                                            \
                                                                            \
            ssd += d * d;                                                   \
        }                                                                   \
    }

static uint32_t hevc_ssd_rows(const uint8_t *a, int a_stride, const uint8_t *b,
                              int b_stride, int m, int n)
{
    uint32_t ssd = 0;

    switch (m) {
    case 32:
        HEVC_SSD_ROWS(32)
        break;
    case 16:
        HEVC_SSD_ROWS(16)
        break;
    case 8:
        HEVC_SSD_ROWS(8)
        break;
    default:
        HEVC_SSD_ROWS(4)
        break;
    }
    return ssd;
}

/* The squared error of a coding block's samples in @p, all components */
static uint64_t hevc_ssd(const HevcEnc *e, int x0, int y0, int n,
                         const uint8_t *const p[3])
{
    uint64_t ssd = 0;

    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = n / sub, stride = e->w / sub;
        int xs = x0 / sub, ys = y0 / sub;

        ssd += hevc_ssd_rows(e->src[i] + ys * e->src_stride[i] + xs,
                             e->src_stride[i], p[i] + ys * stride + xs,
                             stride, m, m);
    }
    return ssd;
}

static int64_t hevc_lambda(int qp)
{
    return lround(256 * 0.57 * pow(2, (qp - 12) / 3.0));
}

/*
 * Rounding of the levels of an inter block's residual, /512: the search's
 * blocks and the intra's take 171, which the T31's records agree with; this
 * and the lambda below are what the T32's pictures of a still scene with
 * sensor noise fit (the curve of size by QP)
 */
#define HEVC_ROUND_ZERO 85

/*
 * What a P picture's blocks are chosen by: the lambda above scaled to the
 * point where the size of a picture of a still scene with sensor noise
 * falls with the QP as a T32's does (MEASURED: its pictures at QP 27 to 29,
 * against this coder's with Gaussian noise added to a decoded camera scene)
 */
static int64_t hevc_p_lambda(int qp)
{
    return hevc_lambda(qp) * 19 / 10;
}

static int hevc_ctb_qp(const HevcEnc *e, uint32_t i, uint32_t j)
{
    return e->pic->block_qp ? e->pic->block_qp[j * e->ctbs_w + i] : e->pic->qp;
}

static void hevc_an_qp(HevcAn *a, int qp)
{
    a->qp = qp;
    a->qp_c[0] = hevc_qp_c(qp + a->e->pic->chroma_qp_offset);
    a->qp_c[1] = hevc_qp_c(qp + a->e->pic->cr_qp_offset);
    a->lambda = hevc_p_lambda(qp);
}

static void hevc_an_init(HevcAn *a, const HevcEnc *e, const uint8_t *nb[3],
                         uint8_t *out[3], int qp)
{
    a->e = e;
    a->skipmap = a->intermap = NULL;
    for (int i = 0; i < 3; i++) {
        a->nb[i] = nb[i];
        a->out[i] = out[i];
        a->refp[i] = a->irecp[i] = NULL;
    }
    hevc_an_qp(a, qp);
}

/*
 * Mode, levels and reconstruction of an intra coding block, from the
 * samples around it in a->nb
 */
static void hevc_analyze(HevcAn *a, HevcCand *cd)
{
    const HevcEnc *e = a->e;
    int x0 = cd->x, y0 = cd->y, log2 = cd->log2, n = 1 << log2;
    uint8_t *ref[3] = { cd->refs, cd->refs + 4 * n + 1, cd->refs + 6 * n + 2 };
    QEMU_UNINITIALIZED int32_t level[32 * 32];

    a->cur = e->order[(y0 >> 2) * e->w4 + (x0 >> 2)];
    hevc_ref(a, 0, x0, y0, n, ref[0]);
    cd->mode = hevc_pick_mode(a, x0, y0, log2, ref[0]);
    cd->cbf = 0;
    for (int i = 0; i < 3; i++) {
        int sz = i ? log2 - 1 : log2, m = 1 << sz;
        int bx = i ? x0 / 2 : x0, by = i ? y0 / 2 : y0;

        if (i) {
            hevc_ref(a, i, bx, by, m, ref[i]);
        }
        if (hevc_block(a, i, bx, by, sz, cd->mode, ref[i], level)) {
            cd->cbf |= 1 << i;
            for (int k = 0; k < m * m; k++) {
                cd->level[i][k] = level[k];
            }
        }
    }
    if (e->ref[0]) {
        cd->ssd_intra = hevc_ssd(e, x0, y0, n, (const uint8_t *const *)a->out);
    }
    cd->ready = true;
    cd->in_rec = a->out[0] == e->rec[0];
}

static void hevc_intra_emit(HevcEnc *e, const HevcCand *cd)
{
    Cabac *c = &e->c;
    int x0 = cd->x, y0 = cd->y, log2 = cd->log2, n = 1 << log2;
    int cand[3], idx = -1, mode = cd->mode;
    bool cbf[3] = { cd->cbf & 1, cd->cbf >> 1 & 1, cd->cbf >> 2 };

    if (log2 == LOG2_MIN_CB) {
        cabac_bin(c, CTX_PART_MODE, 1);     /* PART_2Nx2N */
    }

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

    if (e->syn->tu_depth) {
        cabac_bin(c, CTX_SPLIT_TU + 5 - log2, 0);
    }
    cabac_bin(c, CTX_CBF_CHROMA, cbf[1]);
    cabac_bin(c, CTX_CBF_CHROMA, cbf[2]);
    cabac_bin(c, CTX_CBF_LUMA + 1, cbf[0]);
    if ((cbf[0] || cbf[1] || cbf[2]) && e->syn->qp_delta_depth >= 0 &&
        !e->qp_delta_coded) {
        hevc_qp_delta(c, e->qp - e->qp_pred);
        e->qp_delta_coded = true;
    }
    for (int i = 0; i < 3; i++) {
        if (cbf[i]) {
            hevc_residual(e, cd->level[i], i ? log2 - 1 : log2, i);
        }
    }
    e->st->cus[log2 - LOG2_MIN_CB]++;
}

/*
 * What a coding block leaves for deblocking and for a thread's guess of the
 * samples: 0 skipped, 1 intra, 2 inter with a residual but no luma levels, 3
 * inter with luma levels
 */
static void hevc_kind(HevcEnc *e, int x0, int y0, int n, int kind)
{
    if (e->bsmap) {
        hevc_set_map(e, e->bsmap, x0, y0, n, kind == 1 ? 1 : kind == 3 ? 2 : 0);
    }
    if (e->inter) {
        hevc_set_map(e, e->inter, x0, y0, n, kind >= 2);
    }
}

/*
 * The transform tree of an inter block with a residual: one transform block,
 * inter with the intra's depth as nothing else is known of it
 */
static void hevc_inter_tu(HevcEnc *e, int log2, int cbf_bits,
                          const int16_t *const level[3])
{
    Cabac *c = &e->c;
    bool cbf[3] = { cbf_bits & 1, cbf_bits >> 1 & 1, cbf_bits >> 2 };

    if (e->syn->tu_depth) {
        cabac_bin(c, CTX_SPLIT_TU + 5 - log2, 0);
    }
    cabac_bin(c, CTX_CBF_CHROMA, cbf[1]);
    cabac_bin(c, CTX_CBF_CHROMA, cbf[2]);
    if (cbf[1] || cbf[2]) {
        cabac_bin(c, CTX_CBF_LUMA + 1, cbf[0]);
    }
    if (e->syn->qp_delta_depth >= 0 && !e->qp_delta_coded) {
        hevc_qp_delta(c, e->qp - e->qp_pred);
        e->qp_delta_coded = true;
    }
    for (int i = 0; i < 3; i++) {
        if (cbf[i]) {
            hevc_residual(e, level[i], i ? log2 - 1 : log2, i);
        }
    }
}

/* The QpY the decoder gives a coding block coded now (8.6.1) */
static int hevc_cu_qpy(HevcEnc *e)
{
    return e->qp_delta_coded ? e->qp : e->qp_pred;
}

/* The coding block's samples in @dst: @src's, in all three */
static void hevc_copy_cu(const HevcEnc *e, uint8_t *const dst[3],
                         const uint8_t *const src[3], int x0, int y0, int n)
{
    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = n / sub, stride = e->w / sub;

        if (src[i] == dst[i]) {
            continue;
        }
        for (int y = 0; y < m; y++) {
            size_t off = (y0 / sub + y) * stride + x0 / sub;

            memcpy(dst[i] + off, src[i] + off, m);
        }
    }
}

static int hevc_skip_ctx(const HevcEnc *e, int x0, int y0)
{
    int ctx = 0;

    if (x0 > 0) {
        ctx += e->skip[(y0 >> 2) * e->w4 + ((x0 - 1) >> 2)];
    }
    if (y0 > 0) {
        ctx += e->skip[((y0 - 1) >> 2) * e->w4 + (x0 >> 2)];
    }
    return ctx;
}

/*
 * Code the coding block as skipped, @ctx being its cu_skip_flag's; @restore
 * where its candidate was made over the reference's samples in the picture
 */
static void hevc_skip_cu(HevcEnc *e, const HevcCand *cd, int ctx,
                         bool restore)
{
    int x0 = cd->x, y0 = cd->y, n = 1 << cd->log2;

    cabac_bin(&e->c, CTX_SKIP + ctx, 1);
    if (e->syn->merge_cands > 1) {
        cabac_bin(&e->c, CTX_MERGE_IDX, 0);     /* merge_idx 0 */
    }
    if (restore) {
        hevc_copy_cu(e, e->rec, e->ref, x0, y0, n);
    }
    hevc_set_map(e, e->mode, x0, y0, n, MODE_DC);
    hevc_set_map(e, e->skip, x0, y0, n, 1);
    hevc_kind(e, x0, y0, n, 0);
    e->st->cus[cd->log2 - LOG2_MIN_CB]++;
    e->st->skip_8x8 += (n / 8) * (n / 8);
}

static void hevc_ensure_cand(HevcEnc *e, HevcCand *cd);

/*
 * The block predicted from the last picture's samples at its own place (the
 * zero vector) with the residual of the source against them: levels,
 * reconstruction (in the inter plane) and error. Only the source, the
 * reference and the QP go in, so a thread's candidate is the final one.
 */
static void hevc_analyze_inter(const HevcAn *a, HevcCand *cd)
{
    const HevcEnc *e = a->e;
    int x0 = cd->x, y0 = cd->y, log2 = cd->log2;

    cd->icbf = 0;
    cd->ssd_inter = 0;
    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, sz = log2 - !!i, m = 1 << sz, stride = e->w / sub;
        int bx = x0 / sub, by = y0 / sub;
        const uint8_t *src = e->src[i] + by * e->src_stride[i] + bx;
        const uint8_t *rf = e->ref[i] + by * stride + bx;
        uint8_t *rec = e->irec[i] + by * stride + bx;
        QEMU_UNINITIALIZED int16_t res[32 * 32];
        QEMU_UNINITIALIZED int32_t level[32 * 32];
        bool cbf;

        for (int y = 0; y < m; y++) {
            for (int x = 0; x < m; x++) {
                res[y * m + x] = src[y * e->src_stride[i] + x] -
                                 rf[y * stride + x];
            }
        }
        cbf = hevc_quant(res, sz, i ? a->qp_c[i - 1] : a->qp,
                         !e->pic->scaling_list, e->syn->sign_hiding, true,
                         HEVC_ROUND_ZERO, level);
        if (cbf) {
            cd->icbf |= 1 << i;
            for (int k = 0; k < m * m; k++) {
                cd->ilevel[i][k] = level[k];
            }
        }
        for (int y = 0; y < m; y++) {
            for (int x = 0; x < m; x++) {
                int v = rf[y * stride + x] + (cbf ? res[y * m + x] : 0);
                int d;

                v = MIN(MAX(v, 0), 255);
                rec[y * stride + x] = v;
                d = src[y * e->src_stride[i] + x] - v;
                cd->ssd_inter += d * d;
            }
        }
    }
    cd->iready = true;
}

/* A block of a P picture merged with candidate 0, with a residual */
static void hevc_zinter_emit(HevcEnc *e, const HevcCand *cd, int ctx)
{
    Cabac *c = &e->c;
    int n = 1 << cd->log2;
    const int16_t *const level[3] = { cd->ilevel[0], cd->ilevel[1],
                                      cd->ilevel[2] };

    cabac_bin(c, CTX_SKIP + ctx, 0);
    cabac_bin(c, CTX_PRED_MODE, 0);         /* MODE_INTER */
    cabac_bin(c, CTX_PART_MODE, 1);         /* PART_2Nx2N */
    cabac_bin(c, CTX_MERGE_FLAG, 1);
    if (e->syn->merge_cands > 1) {
        cabac_bin(c, CTX_MERGE_IDX, 0);
    }
    hevc_inter_tu(e, cd->log2, cd->icbf, level);
    hevc_set_map(e, e->mode, cd->x, cd->y, n, MODE_DC);
    hevc_kind(e, cd->x, cd->y, n, cd->icbf & 1 ? 3 : 2);
    e->st->cus[cd->log2 - LOG2_MIN_CB]++;
}

/*
 * A coding block of a P picture: skipped, with merge candidate 0; that
 * candidate with a residual; or intra. Every motion vector is zero, so every
 * merge candidate is the zero vector on the one reference, the temporal one
 * too; skip copies that picture. The choice is by SSD + lambda * bits, the
 * bits of each option counted without writing them. An option that is not
 * skipping takes at least its skip flag and one bypass bin, and that is
 * enough to see skipping win without making any.
 */
static bool hevc_inter_cu(HevcEnc *e, HevcCand *cd)
{
    Cabac *c = &e->c, saved;
    int x0 = cd->x, y0 = cd->y, n = 1 << cd->log2;
    int ctx = hevc_skip_ctx(e, x0, y0);
    IngenicAvpuStats st;
    int64_t j_best, j;
    uint64_t start;
    bool coded, real = !e->shadow;
    int best = 0;           /* 0 skip, 1 intra, 2 inter with a residual */

    if (!e->ref[0]) {
        hevc_ensure_cand(e, cd);
        cabac_bin(c, CTX_SKIP + ctx, 0);
        cabac_bin(c, CTX_PRED_MODE, 1);     /* MODE_INTRA */
        hevc_intra_emit(e, cd);
        hevc_kind(e, x0, y0, n, 1);
        return false;
    }
    j_best = (int64_t)cd->ssd_skip * 256 +
             e->lambda * (cabac_cost(c, CTX_SKIP + ctx, 1) +
                          (e->syn->merge_cands > 1 ?
                           cabac_cost(c, CTX_MERGE_IDX, 0) : 0)) / 256;
    if (real &&
        j_best < e->lambda * (cabac_shifts(c, CTX_SKIP + ctx, 0) + 1)) {
        hevc_skip_cu(e, cd, ctx, false);
        return true;
    }
    if (e->shadow) {
        if (!cd->ready) {
            hevc_skip_cu(e, cd, ctx, false);
            return true;
        }
    } else {
        hevc_ensure_cand(e, cd);
        if (!cd->iready) {
            hevc_analyze_inter(&(HevcAn){ .e = e, .qp = e->qp,
                                          .qp_c = { e->qp_c[0], e->qp_c[1] } },
                               cd);
        }
    }
    saved = *c;
    st = *e->st;
    coded = e->qp_delta_coded;
    c->count = true;

    start = c->cost;
    cabac_bin(c, CTX_SKIP + ctx, 0);
    cabac_bin(c, CTX_PRED_MODE, 1);         /* MODE_INTRA */
    hevc_intra_emit(e, cd);
    j = (int64_t)cd->ssd_intra * 256 +
        e->lambda * (int64_t)(c->cost - start) / 256;
    if (j <= j_best) {
        j_best = j;
        best = 1;
    }
    if (cd->icbf) {
        *c = saved;
        *e->st = st;
        e->qp_delta_coded = coded;
        c->count = true;
        start = c->cost;
        hevc_zinter_emit(e, cd, ctx);
        j = (int64_t)cd->ssd_inter * 256 +
            e->lambda * (int64_t)(c->cost - start) / 256;
        if (j < j_best) {
            best = 2;
        }
    }
    *c = saved;
    *e->st = st;
    e->qp_delta_coded = coded;

    switch (best) {
    case 0:
        hevc_skip_cu(e, cd, ctx, real && cd->in_rec);
        return true;
    case 1:
        cabac_bin(c, CTX_SKIP + ctx, 0);
        cabac_bin(c, CTX_PRED_MODE, 1);     /* MODE_INTRA */
        hevc_intra_emit(e, cd);
        hevc_kind(e, x0, y0, n, 1);
        if (real && !cd->in_rec) {
            hevc_copy_cu(e, e->rec, (const uint8_t *const *)e->cand, x0, y0,
                         n);
        }
        return false;
    default:
        hevc_zinter_emit(e, cd, ctx);
        if (real) {
            hevc_copy_cu(e, e->rec, (const uint8_t *const *)e->irec, x0, y0,
                         n);
        }
        return false;
    }
}

/* Whether a thread's candidate was made from the samples that are final */
static bool hevc_cand_valid(const HevcEnc *e, const HevcCand *cd)
{
    HevcAn a = { .e = e, .nb = { e->rec[0], e->rec[1], e->rec[2] } };
    int x0 = cd->x, y0 = cd->y, n = 1 << cd->log2;
    uint8_t ref[8 * 32 + 3];

    a.cur = e->order[(y0 >> 2) * e->w4 + (x0 >> 2)];
    hevc_ref(&a, 0, x0, y0, n, ref);
    hevc_ref(&a, 1, x0 / 2, y0 / 2, n / 2, ref + 4 * n + 1);
    hevc_ref(&a, 2, x0 / 2, y0 / 2, n / 2, ref + 6 * n + 2);
    return !memcmp(ref, cd->refs, 8 * n + 3);
}

/* Make a candidate from the final neighbours */
static void hevc_ensure_cand(HevcEnc *e, HevcCand *cd)
{
    const uint8_t *nb[3] = { e->rec[0], e->rec[1], e->rec[2] };
    HevcAn a;

    if (cd->ready && (e->exact || hevc_cand_valid(e, cd))) {
        return;
    }
    hevc_an_init(&a, e, nb, e->rec, e->qp);
    hevc_analyze(&a, cd);
}

/* The next coding block: a thread's, or this one's to make */
static HevcCand *hevc_cand_slot(HevcEnc *e, int x0, int y0, int log2)
{
    HevcCand *cd;

    if (e->cands) {
        return &e->cands[e->cu_next++];
    }
    cd = &e->scratch;
    cd->x = x0;
    cd->y = y0;
    cd->log2 = log2;
    cd->ready = false;
    cd->iready = false;
    if (e->ref[0]) {
        cd->ssd_skip = hevc_ssd(e, x0, y0, 1 << log2, e->ref);
    }
    return cd;
}

static void hevc_coding_unit(HevcEnc *e, int x0, int y0, int log2)
{
    int n = 1 << log2;
    HevcCand *cd = hevc_cand_slot(e, x0, y0, log2);

    if (x0 > 0) {
        for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
            e->vedge[y * (e->w / 8) + (x0 >> 3)] = 1;
        }
    }
    if (y0 > 0) {
        memset(e->hedge + (y0 >> 3) * e->w4 + (x0 >> 2), 1, n >> 2);
    }
    if (e->pic->intra) {
        hevc_ensure_cand(e, cd);
        hevc_intra_emit(e, cd);
    } else {
        hevc_inter_cu(e, cd);
    }
    hevc_set_map(e, e->qpy, x0, y0, n, hevc_cu_qpy(e));
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
static int hevc_bs(const HevcEnc *e, uint32_t pb, uint32_t qb);

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
                if (e->search) {
                    bs = hevc_bs(e, pb, qb);
                } else if (e->pic->intra) {
                    bs = 2;
                } else {
                    uint8_t m = e->bsmap[pb] | e->bsmap[qb];

                    bs = m & 1 ? 2 : m >> 1;
                }
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
                if ((dir ? y : x) % 16 || (dir ? x : y) % 8 || bs != 2) {
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

/*
 * A quantization group's QP goes with its first coded residual; without one
 * the decoder keeps the group before's
 */
static void hevc_quadtree(HevcEnc *e, int x0, int y0, int log2, int depth)
{
    int n = 1 << log2;
    bool split;

    if (log2 >= e->log2_qg) {
        e->qp_delta_coded = false;
    }
    if (x0 + n <= e->w && y0 + n <= e->h && log2 > LOG2_MIN_CB) {
        int ctx = 0;

        if (x0 > 0) {
            ctx += e->depth[(y0 >> 2) * e->w4 + ((x0 - 1) >> 2)] > depth;
        }
        if (y0 > 0) {
            ctx += e->depth[((y0 - 1) >> 2) * e->w4 + (x0 >> 2)] > depth;
        }
        split = log2 > LOG2_MAX_CB;
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
    } else {
        hevc_set_map(e, e->depth, x0, y0, n, depth);
        hevc_coding_unit(e, x0, y0, log2);
    }
    if (log2 >= e->log2_qg && e->qp_delta_coded) {
        e->qp_pred = e->qp;
    }
}

/* sao() of a CTB, the parameters of the CTB before, or none at the first */
static void hevc_sao(HevcEnc *e, uint32_t rx, uint32_t ry)
{
    if (rx || ry) {
        cabac_bin(&e->c, CTX_SAO_MERGE, 1);     /* sao_merge_left/up_flag */
        return;
    }
    if (e->syn->sao_luma) {
        cabac_bin(&e->c, CTX_SAO_TYPE, 0);      /* sao_type_idx_luma */
    }
    if (e->syn->sao_chroma) {
        cabac_bin(&e->c, CTX_SAO_TYPE, 0);      /* sao_type_idx_chroma */
    }
}

static int hevc_threads(const IngenicAvpuPic *pic)
{
    if (pic->threads > 0) {
        return MIN(pic->threads, 64);
    }
    return MIN(g_get_num_processors(), 8);
}

/*
 * The coding order of the blocks, and the coding blocks themselves where
 * threads are to make their candidates. The quadtree's shape is the
 * picture's alone: only an edge cuts a block, the CTB of 64 splits once.
 */
static void hevc_layout(HevcEnc *e, int x0, int y0, int log2, uint32_t *seq)
{
    int n = 1 << log2;
    bool split;

    if (x0 + n <= e->w && y0 + n <= e->h && log2 > LOG2_MIN_CB) {
        split = log2 > LOG2_MAX_CB;
    } else {
        split = log2 > LOG2_MIN_CB;
    }
    if (split) {
        for (int i = 0; i < 4; i++) {
            int x = x0 + (i & 1) * n / 2, y = y0 + (i >> 1) * n / 2;

            if (x < e->w && y < e->h) {
                hevc_layout(e, x, y, log2 - 1, seq);
            }
        }
        return;
    }
    for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
        for (int x = x0 >> 2; x < (x0 + n) >> 2; x++) {
            e->order[y * e->w4 + x] = *seq;
        }
    }
    if (e->cands) {
        HevcCand *cd = &e->cands[*seq];

        cd->x = x0;
        cd->y = y0;
        cd->log2 = log2;
        cd->level[0] = e->level_arena + e->level_used;
        cd->level[1] = cd->level[0] + n * n;
        cd->level[2] = cd->level[1] + n * n / 4;
        cd->ilevel[0] = cd->level[2] + n * n / 4;
        cd->ilevel[1] = cd->ilevel[0] + n * n;
        cd->ilevel[2] = cd->ilevel[1] + n * n / 4;
        cd->refs = e->refs_arena + e->refs_used;
    }
    e->level_used += n * n * 3;
    e->refs_used += 8 * n + 3;
    (*seq)++;
}

/* The entropy coder's turn at a CTB */
static void hevc_code_ctb(HevcEnc *e, uint32_t i, uint32_t j)
{
    uint32_t ctb = 1 << e->syn->log2_ctb;

    e->qp = hevc_ctb_qp(e, i, j);
    e->qp_c[0] = hevc_qp_c(e->qp + e->pic->chroma_qp_offset);
    e->qp_c[1] = hevc_qp_c(e->qp + e->pic->cr_qp_offset);
    e->lambda = hevc_p_lambda(e->qp);
    if (e->syn->sao_luma || e->syn->sao_chroma) {
        hevc_sao(e, i, j);
    }
    hevc_quadtree(e, i * ctb, j * ctb, e->syn->log2_ctb, 0);
    cabac_terminate(&e->c, i == e->ctbs_w - 1 && j == e->ctbs_h - 1);
}

/*
 * Rows of CTBs wavefront: a thread makes the candidates of a row's CTB once
 * the row above has made those up to the CTB right of it, which holds
 * every sample a block can predict from. The entropy coder, on the thread
 * that called, takes CTBs in order as their candidates are ready.
 */
/* a row's count, a cache line to itself: its thread writes it as others read */
typedef struct HevcProgress {
    int made;
    char pad[60];
} HevcProgress;

/* CTBs a row goes on for between looks at the row above */
#define HEVC_WAVE_BATCH 8

typedef struct HevcWave {
    HevcEnc *e;
    HevcProgress *progress;     /* CTBs of each row made */
    uint8_t (*state)[CTX_COUNT];    /* each row's contexts after its 2nd CTB */
    int next_row;               /* the next row for a thread to take */
    int waiting;                /* threads in hevc_wave_wait() */
    QemuMutex lock;
    QemuCond cond;
    int max_helpers, helpers;   /* helpers in, under hevc_pool.lock */
    uint32_t next_ctb;          /* the entropy coder's, by its thread alone */
} HevcWave;

static struct {
    QemuMutex lock;
    QemuCond cond;
    GList *waves;
    int threads;
} hevc_pool;

static void hevc_wave_wait(HevcWave *w, uint32_t row, int need)
{
    /* a CTB takes a microsecond or two: most waits are over before a sleep */
    for (int spin = 0; spin < 2000; spin++) {
        if (qatomic_load_acquire(&w->progress[row].made) >= need) {
            return;
        }
        cpu_relax();
    }
    qatomic_inc(&w->waiting);
    qemu_mutex_lock(&w->lock);
    while (qatomic_load_acquire(&w->progress[row].made) < need) {
        qemu_cond_wait(&w->cond, &w->lock);
    }
    qemu_mutex_unlock(&w->lock);
    qatomic_dec(&w->waiting);
}

static void hevc_wave_publish(HevcWave *w, uint32_t row, int made)
{
    qatomic_store_release(&w->progress[row].made, made);
    /* pairs with the increment in hevc_wave_wait(): no wakeup is lost */
    smp_mb();
    if (qatomic_read(&w->waiting)) {
        qemu_mutex_lock(&w->lock);
        qemu_cond_broadcast(&w->cond);
        qemu_mutex_unlock(&w->lock);
    }
}

/*
 * Whether the entropy coder will see skipping win without the block made,
 * going by the shadow's contexts, the bits of a skip flag of 0 being what
 * an intra block must code first and then at least one bypass bin more
 */
static bool hevc_skip_likely(const HevcEnc *sh, const HevcCand *cd)
{
    int ctx = hevc_skip_ctx(sh, cd->x, cd->y);
    int64_t j_skip = (int64_t)cd->ssd_skip * 256 +
                     sh->lambda * (cabac_cost(&sh->c, CTX_SKIP + ctx, 1) +
                                   (sh->syn->merge_cands > 1 ?
                                    cabac_cost(&sh->c, CTX_MERGE_IDX, 0) : 0))
                     / 256;
    int bits = MAX((cabac_cost(&sh->c, CTX_SKIP + ctx, 0) >> 8) - 1, 0);

    return j_skip < sh->lambda * (bits + 1);
}

/*
 * The candidates of one CTB. Where the neighbours are a guess, so is each
 * block's final form: @sh codes the blocks as the entropy coder will, but
 * its contexts only have the CTBs of this row behind them, and counted
 * rather than written, so a block's reconstruction may still go the
 * other way and be made again. A block that skipping all but reproduces
 * is not made: the entropy coder makes it if it cannot show skipping wins.
 */
static void hevc_analyze_ctb(HevcAn *a, HevcEnc *sh, const HevcEnc *e,
                             uint32_t i, uint32_t j)
{
    uint32_t ctb = j * e->ctbs_w + i;
    int qp = hevc_ctb_qp(e, i, j);

    hevc_an_qp(a, qp);
    sh->qp = qp;
    sh->qp_c[0] = a->qp_c[0];
    sh->qp_c[1] = a->qp_c[1];
    sh->lambda = a->lambda;
    for (uint32_t k = e->ctb_first[ctb]; k < e->ctb_first[ctb + 1]; k++) {
        HevcCand *cd = &e->cands[k];

        cd->ready = false;
        cd->iready = false;
        if (e->exact) {
            hevc_analyze(a, cd);
            continue;
        }
        cd->ssd_skip = hevc_ssd(e, cd->x, cd->y, 1 << cd->log2, e->ref);
        if (!hevc_skip_likely(sh, cd)) {
            hevc_analyze(a, cd);
            hevc_analyze_inter(a, cd);
        }
        sh->qp_pred = qp;
        sh->qp_delta_coded = false;
        hevc_inter_cu(sh, cd);
    }
}

static void hevc_wave_code(HevcWave *w)
{
    HevcEnc *e = w->e;

    for (;;) {
        uint32_t row = w->next_ctb / e->ctbs_w, col = w->next_ctb % e->ctbs_w;

        if (w->next_ctb == e->ctbs_w * e->ctbs_h ||
            qatomic_load_acquire(&w->progress[row].made) <= col) {
            return;
        }
        hevc_code_ctb(e, col, row);
        w->next_ctb++;
    }
}

static void hevc_wave_row(HevcWave *w, uint32_t row, bool lead)
{
    HevcEnc *e = w->e;
    const uint8_t *nb[3] = { e->cand[0], e->cand[1], e->cand[2] };
    g_autoptr(GByteArray) none = g_byte_array_new();
    IngenicAvpuStats st;
    HevcEnc sh = {
        .pic = e->pic, .syn = e->syn, .log2_qg = e->log2_qg, .w = e->w,
        .h = e->h, .w4 = e->w4, .h4 = e->h4, .ctbs_w = e->ctbs_w,
        .ctbs_h = e->ctbs_h, .src = { e->src[0], e->src[1], e->src[2] },
        .src_stride = { e->src_stride[0], e->src_stride[1], e->src_stride[2] },
        .cand = { e->cand[0], e->cand[1], e->cand[2] },
        .ref = { e->ref[0], e->ref[1], e->ref[2] }, .order = e->order,
        .mode = e->spec_mode, .skip = e->spec_skip, .inter = e->spec_inter,
        .st = &st,
        .shadow = true,
    };
    HevcAn a;

    hevc_an_init(&a, e, nb, e->cand, 0);
    if (!e->exact) {
        a.skipmap = e->spec_skip;
        a.intermap = e->spec_inter;
        for (int i = 0; i < 3; i++) {
            a.refp[i] = e->ref[i];
            a.irecp[i] = e->irec[i];
        }
    }
    cabac_init(&sh.c, e->pic->cabac_init_idc ? 2 : 1, e->pic->qp, none);
    sh.c.count = true;
    for (uint32_t i = 0; i < e->ctbs_w; i++) {
        if (row && !(i % HEVC_WAVE_BATCH)) {
            hevc_wave_wait(w, row - 1, MIN(i + HEVC_WAVE_BATCH + 1, e->ctbs_w));
            if (!i) {
                memcpy(sh.c.state, w->state[row - 1], CTX_COUNT);
            }
        }
        hevc_analyze_ctb(&a, &sh, e, i, row);
        if (i == MIN(1, e->ctbs_w - 1)) {
            memcpy(w->state[row], sh.c.state, CTX_COUNT);
        }
        hevc_wave_publish(w, row, i + 1);
        if (lead) {
            hevc_wave_code(w);
        }
    }
}

static void *hevc_helper(void *opaque)
{
    for (;;) {
        HevcWave *w = NULL;
        uint32_t row;

        qemu_mutex_lock(&hevc_pool.lock);
        while (!w) {
            for (GList *l = hevc_pool.waves; l && !w; l = l->next) {
                HevcWave *c = l->data;

                if (c->helpers < c->max_helpers &&
                    qatomic_read(&c->next_row) < c->e->ctbs_h) {
                    w = c;
                }
            }
            if (!w) {
                qemu_cond_wait(&hevc_pool.cond, &hevc_pool.lock);
            }
        }
        w->helpers++;
        qemu_mutex_unlock(&hevc_pool.lock);

        while ((row = qatomic_fetch_inc(&w->next_row)) < w->e->ctbs_h) {
            hevc_wave_row(w, row, false);
        }

        qemu_mutex_lock(&hevc_pool.lock);
        w->helpers--;
        qemu_cond_broadcast(&hevc_pool.cond);
        qemu_mutex_unlock(&hevc_pool.lock);
    }
    return NULL;
}

/* Code the picture's CTBs with candidates made by up to @threads threads */
static void hevc_run_wave(HevcEnc *e, int threads)
{
    static gsize once;
    HevcWave w = { .e = e, .max_helpers = threads - 1 };
    uint32_t total = e->ctbs_w * e->ctbs_h;

    if (g_once_init_enter(&once)) {
        qemu_mutex_init(&hevc_pool.lock);
        qemu_cond_init(&hevc_pool.cond);
        g_once_init_leave(&once, 1);
    }
    w.progress = g_new0(HevcProgress, e->ctbs_h);
    w.state = g_malloc(e->ctbs_h * CTX_COUNT);
    qemu_mutex_init(&w.lock);
    qemu_cond_init(&w.cond);

    qemu_mutex_lock(&hevc_pool.lock);
    while (hevc_pool.threads < threads - 1) {
        QemuThread th;

        qemu_thread_create(&th, "hevc", hevc_helper, NULL,
                           QEMU_THREAD_DETACHED);
        hevc_pool.threads++;
    }
    hevc_pool.waves = g_list_prepend(hevc_pool.waves, &w);
    qemu_cond_broadcast(&hevc_pool.cond);
    qemu_mutex_unlock(&hevc_pool.lock);

    while (w.next_ctb < total) {
        uint32_t row;

        hevc_wave_code(&w);
        if (w.next_ctb == total) {
            break;
        }
        row = qatomic_fetch_inc(&w.next_row);
        if (row < e->ctbs_h) {
            hevc_wave_row(&w, row, true);
        } else {
            hevc_wave_wait(&w, w.next_ctb / e->ctbs_w,
                           w.next_ctb % e->ctbs_w + 1);
        }
    }

    qemu_mutex_lock(&hevc_pool.lock);
    hevc_pool.waves = g_list_remove(hevc_pool.waves, &w);
    while (w.helpers) {
        qemu_cond_wait(&hevc_pool.cond, &hevc_pool.lock);
    }
    qemu_mutex_unlock(&hevc_pool.lock);
    qemu_cond_destroy(&w.cond);
    qemu_mutex_destroy(&w.lock);
    g_free(w.progress);
    g_free(w.state);
}

/*
 * P pictures that search for motion (h265-motion=search)
 *
 * Measured on a T31's EL150 with synthetic pictures (qemu-lab/hevcsearch):
 * the search covers the whole samples from 32 below its centre to 31 above,
 * on both axes, and finds a white-noise block at all of them but for the odd
 * shifts of the farther half; the centre is the vector the previous picture
 * had at the same 16x16, rounded to a grid of 8 samples. A shift of a
 * quarter sample is found up to 3 quarters from a whole sample, as a ring of
 * halves and one of quarters around the whole sample's best. Vectors of the
 * neighbours (and the previous picture's) are candidates too, which carries
 * a vector past the window along a row of blocks. The record of each 16x16
 * unit holds the vector and 0x100 where the block is inter and not skipped.
 *
 * Coding blocks are 32x32, or the four 16x16 where the vectors differ, as
 * one prediction unit each, the vector merged with a neighbour's or coded
 * as a difference from the AMVP predictor. The motion vector candidates are
 * derived as 8.6.2 has them for one reference picture, which the slice
 * header's slice_temporal_mvp_enabled_flag makes include the previous
 * picture's co-located vector.
 */

#define ME_PAD 96               /* the reference's border, in samples */

static const int8_t luma_taps[4][8] = {
    { 0, 0, 0, 64, 0, 0, 0, 0 }, { -1, 4, -10, 58, 17, -5, 1, 0 },
    { -1, 4, -11, 40, 40, -11, 4, -1 }, { 0, 1, -5, 17, 58, -10, 4, -1 },
};

static const int8_t chroma_taps[8][4] = {
    { 0, 64, 0, 0 }, { -2, 58, 10, -2 }, { -4, 54, 16, -2 },
    { -6, 46, 28, -4 }, { -4, 36, 36, -4 }, { -4, 28, 46, -6 },
    { -2, 16, 54, -4 }, { -2, 10, 58, -2 },
};

typedef int16_t hevc_v8h __attribute__((vector_size(16)));
typedef int32_t hevc_v8s __attribute__((vector_size(32)));
typedef uint8_t hevc_v8b __attribute__((vector_size(8)));

static inline hevc_v8h hevc_load8(const uint8_t *p)
{
    hevc_v8b b;

    memcpy(&b, p, 8);
    return __builtin_convertvector(b, hevc_v8h);
}

/*
 * The samples of a bw x bh block of a plane at (x0, y0), displaced by (mx, my)
 * in 1/4 (luma, @shift 2) or 1/8 samples (chroma, 3), as 8.5.3.3.3 predicts
 * them with the default weights; the plane's edge samples repeat outwards.
 * The filters run eight columns at a time on a window of the plane.
 */
static void hevc_mc_win(const uint8_t *src, int ss, int bw, int bh, int taps,
                        const int8_t *tx, const int8_t *ty, bool fx, bool fy,
                        uint8_t *dst, int ds, int16_t *tmp)
{
    int before = taps / 2 - 1;

    src += before * ss + before;
    if (!fx && !fy) {
        for (int r = 0; r < bh; r++) {
            memcpy(dst + r * ds, src + r * ss, bw);
        }
        return;
    }
    if (bw < 8) {
        for (int r = 0; r < bh; r++) {
            for (int c = 0; c < bw; c++) {
                int sum = 0;

                if (!fy) {
                    for (int k = 0; k < taps; k++) {
                        sum += tx[k] * src[r * ss + c + k - before];
                    }
                    sum = (sum + 32) >> 6;
                } else if (!fx) {
                    for (int k = 0; k < taps; k++) {
                        sum += ty[k] * src[(r + k - before) * ss + c];
                    }
                    sum = (sum + 32) >> 6;
                } else {
                    for (int k = 0; k < taps; k++) {
                        int row = 0;

                        for (int j = 0; j < taps; j++) {
                            row += tx[j] * src[(r + k - before) * ss + c + j -
                                               before];
                        }
                        sum += ty[k] * row;
                    }
                    sum = ((sum >> 6) + 32) >> 6;
                }
                dst[r * ds + c] = MIN(MAX(sum, 0), 255);
            }
        }
        return;
    }
    for (int c = 0; c < bw; c += 8) {
        if (!fy) {
            for (int r = 0; r < bh; r++) {
                hevc_v8h acc = { 0 };
                hevc_v8h v;
                hevc_v8b o;

                for (int k = 0; k < taps; k++) {
                    acc += tx[k] * hevc_load8(src + r * ss + c + k - before);
                }
                v = (acc + 32) >> 6;
                v = v & ~(v >> 15);
                v = 255 + ((v - 255) & ((v - 255) >> 15));
                o = __builtin_convertvector(v, hevc_v8b);
                memcpy(dst + r * ds + c, &o, 8);
            }
        } else if (!fx) {
            for (int r = 0; r < bh; r++) {
                hevc_v8h acc = { 0 };
                hevc_v8h v;
                hevc_v8b o;

                for (int k = 0; k < taps; k++) {
                    acc += ty[k] * hevc_load8(src + (r + k - before) * ss + c);
                }
                v = (acc + 32) >> 6;
                v = v & ~(v >> 15);
                v = 255 + ((v - 255) & ((v - 255) >> 15));
                o = __builtin_convertvector(v, hevc_v8b);
                memcpy(dst + r * ds + c, &o, 8);
            }
        } else {
            for (int r = 0; r < bh + taps - 1; r++) {
                hevc_v8h acc = { 0 };

                for (int k = 0; k < taps; k++) {
                    acc += tx[k] * hevc_load8(src + (r - before) * ss + c + k -
                                              before);
                }
                memcpy(tmp + r * 8, &acc, 16);
            }
            for (int r = 0; r < bh; r++) {
#ifdef CONFIG_AVX2_OPT
                __m128i lo = _mm_setzero_si128(), hi = lo;

                for (int k = 0; k < taps; k += 2) {
                    __m128i a, b, cf = _mm_set1_epi32((uint16_t)ty[k] |
                                                     ty[k + 1] * 65536);

                    a = _mm_loadu_si128((const __m128i *)(tmp + (r + k) * 8));
                    b = _mm_loadu_si128((const __m128i *)
                                        (tmp + (r + k + 1) * 8));
                    lo = _mm_add_epi32(lo, _mm_madd_epi16(
                                           _mm_unpacklo_epi16(a, b), cf));
                    hi = _mm_add_epi32(hi, _mm_madd_epi16(
                                           _mm_unpackhi_epi16(a, b), cf));
                }
                lo = _mm_srai_epi32(_mm_add_epi32(_mm_srai_epi32(lo, 6),
                                                  _mm_set1_epi32(32)), 6);
                hi = _mm_srai_epi32(_mm_add_epi32(_mm_srai_epi32(hi, 6),
                                                  _mm_set1_epi32(32)), 6);
                _mm_storel_epi64((__m128i *)(dst + r * ds + c),
                                 _mm_packus_epi16(_mm_packs_epi32(lo, hi),
                                                  _mm_setzero_si128()));
#else
                hevc_v8s acc = { 0 };
                hevc_v8s v;
                hevc_v8h h;
                hevc_v8b o;

                for (int k = 0; k < taps; k++) {
                    hevc_v8h t;

                    memcpy(&t, tmp + (r + k) * 8, 16);
                    acc += ty[k] * __builtin_convertvector(t, hevc_v8s);
                }
                v = (((acc >> 6) + 32) >> 6);
                v = v & ~(v >> 31);
                v = 255 + ((v - 255) & ((v - 255) >> 31));
                h = __builtin_convertvector(v, hevc_v8h);
                o = __builtin_convertvector(h, hevc_v8b);
                memcpy(dst + r * ds + c, &o, 8);
#endif
            }
        }
    }
}

static void hevc_mc(HevcScratch *sc, const uint8_t *ref, int w, int h, int x0,
                    int y0, int bw, int bh, int mx, int my, int shift,
                    uint8_t *dst, int ds)
{
    int fx = mx & ((1 << shift) - 1), fy = my & ((1 << shift) - 1);
    int ix = x0 + (mx >> shift), iy = y0 + (my >> shift);
    int taps = shift == 2 ? 8 : 4, before = taps / 2 - 1;
    const int8_t *tx = shift == 2 ? luma_taps[fx] : chroma_taps[fx];
    const int8_t *ty = shift == 2 ? luma_taps[fy] : chroma_taps[fy];
    int rows = bh + taps - 1, cols = bw + taps - 1;
    uint8_t *blk = sc->blk;
    const uint8_t *src;
    int ss;

    if (ix - before >= 0 && iy - before >= 0 && ix - before + cols <= w &&
        iy - before + rows <= h) {
        src = ref + (size_t)(iy - before) * w + ix - before;
        ss = w;
    } else {
        for (int r = 0; r < rows; r++) {
            const uint8_t *row = ref + (size_t)MIN(MAX(iy - before + r, 0),
                                                   h - 1) * w;

            for (int c = 0; c < cols; c++) {
                blk[r * cols + c] = row[MIN(MAX(ix - before + c, 0), w - 1)];
            }
        }
        src = blk;
        ss = cols;
    }
    hevc_mc_win(src, ss, bw, bh, taps, tx, ty, fx, fy, dst, ds, sc->tmp);
}

static inline uint32_t hevc_sad(const uint8_t *a, int as, const uint8_t *b,
                                int bs, int n, int rows)
{
#ifdef CONFIG_AVX2_OPT
    __m128i acc = _mm_setzero_si128();

    for (int y = 0; y < rows; y++) {
        if (n >= 16) {
            for (int x = 0; x < n; x += 16) {
                acc = _mm_add_epi64(acc, _mm_sad_epu8(
                          _mm_loadu_si128((const __m128i *)(a + x)),
                          _mm_loadu_si128((const __m128i *)(b + x))));
            }
        } else {
            acc = _mm_add_epi64(acc, _mm_sad_epu8(
                      _mm_loadl_epi64((const __m128i *)a),
                      _mm_loadl_epi64((const __m128i *)b)));
        }
        a += as;
        b += bs;
    }
    return _mm_cvtsi128_si32(acc) +
           _mm_cvtsi128_si32(_mm_srli_si128(acc, 8));
#elif defined(__ARM_NEON)
    uint32_t sad = 0;

    for (int y = 0; y < rows; y++) {
        if (n >= 16) {
            for (int x = 0; x < n; x += 16) {
                sad += vaddlvq_u8(vabdq_u8(vld1q_u8(a + x), vld1q_u8(b + x)));
            }
        } else {
            sad += vaddlv_u8(vabd_u8(vld1_u8(a), vld1_u8(b)));
        }
        a += as;
        b += bs;
    }
    return sad;
#else
    uint32_t sad = 0;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < n; x++) {
            sad += abs(a[x] - b[x]);
        }
        a += as;
        b += bs;
    }
    return sad;
#endif
}

/* The bits of a vector difference's component, to weigh against SAD */
static inline int hevc_mvd_bits(int d)
{
    d = abs(d);
    return d ? 2 * (32 - __builtin_clz(d)) + 1 : 1;
}

/*
 * The SAD of two n x n blocks, given up at a row (every 4th for 16 and 32)
 * once it has reached @limit, with the sum so far
 */
static uint32_t hevc_sad_lim(const uint8_t *a, int as, const uint8_t *b, int bs,
                             int n, uint32_t limit)
{
    uint32_t sum = 0;
    int step = n >= 16 ? 4 : n;

    for (int y = 0; y < n; y += step) {
        sum += hevc_sad(a + y * as, as, b + y * bs, bs, n, step);
        if (sum >= limit) {
            break;
        }
    }
    return sum;
}

#define ME_SPAN 64

#define ME_DPAD (ME_PAD / 2)

/*
 * The coarse stage: the block at half size (2x2 means of the source and of
 * the reference) in a window of 32 samples round the centre (cx, cy), by
 * its SAD, four times over, plus the cost of the vector's bits. Returns the
 * best position in half samples
 */
static HevcMv hevc_cme(const HevcEnc *e, int x0, int y0, int n, int cx, int cy)
{
    const uint8_t *s = e->dsrc + (y0 / 2) * e->dsrc_stride + x0 / 2;
    int h = n / 2, hs = ME_SPAN / 2;
    int64_t best = INT64_MAX, bx[ME_SPAN / 2], by[ME_SPAN / 2];
    int ix = hs / 2, iy = hs / 2;

    for (int i = 0; i < hs; i++) {
        bx[i] = (int64_t)e->lam_me *
                hevc_mvd_bits(4 * (cx + 2 * (i - hs / 2)));
        by[i] = (int64_t)e->lam_me *
                hevc_mvd_bits(4 * (cy + 2 * (i - hs / 2)));
    }
    for (int dy = 0; dy < hs; dy++) {
        const uint8_t *r = e->dpad + (size_t)(y0 / 2 + cy / 2 + dy - hs / 2 +
                                              ME_DPAD) * e->dpad_stride +
                           x0 / 2 + cx / 2 - hs / 2 + ME_DPAD;

        for (int dx = 0; dx < hs; dx++) {
            int64_t lim = (best - bx[dx] - by[dy] + 3) / 4, c;
            uint32_t sad;

            if (lim <= 0) {
                continue;
            }
            sad = hevc_sad_lim(s, e->dsrc_stride, r + dx, e->dpad_stride, h,
                               MIN(lim, UINT32_MAX));
            c = 4 * (int64_t)sad + bx[dx] + by[dy];
            if (c < best) {
                best = c;
                ix = dx;
                iy = dy;
            }
        }
    }
    return (HevcMv){ cx / 2 + ix - hs / 2, cy / 2 + iy - hs / 2 };
}

/*
 * The best whole-sample vector of the n x n luma block at (x0, y0) in the
 * window round (cx, cy), by SAD plus the cost of the vector's bits against
 * zero, and its cost. The sums give up where they cannot win: the answer
 * is the exhaustive one.
 */
static int64_t hevc_ime_full(const HevcEnc *e, int x0, int y0, int n, int cx,
                             int cy, int span, HevcMv *out)
{
    const uint8_t *s = e->src[0] + y0 * e->src_stride[0] + x0;
    int64_t best = INT64_MAX, bx[ME_SPAN], by[ME_SPAN];
    int lo = ME_SPAN / 2 - span / 2;
    int ix = 0, iy = 0;

    for (int i = 0; i < ME_SPAN; i++) {
        bx[i] = (int64_t)e->lam_me * hevc_mvd_bits(4 * (cx + i - ME_SPAN / 2));
        by[i] = (int64_t)e->lam_me * hevc_mvd_bits(4 * (cy + i - ME_SPAN / 2));
    }
    /* the window's own centre and zero first, so the rest give up early */
    for (int k = 0; k < 2; k++) {
        int dx = k ? ME_SPAN / 2 : -cx + ME_SPAN / 2;
        int dy = k ? ME_SPAN / 2 : -cy + ME_SPAN / 2;
        const uint8_t *r;
        int64_t c;

        if (dx < 0 || dx >= ME_SPAN || dy < 0 || dy >= ME_SPAN) {
            continue;
        }
        r = e->pad + (size_t)(y0 + cy + dy - ME_SPAN / 2 + ME_PAD) *
            e->pad_stride + x0 + cx + dx - ME_SPAN / 2 + ME_PAD;
        c = hevc_sad_lim(s, e->src_stride[0], r, e->pad_stride, n, UINT32_MAX) +
            bx[dx] + by[dy];
        if (c < best) {
            best = c;
            ix = dx;
            iy = dy;
        }
    }
    for (int dy = lo; dy < lo + span; dy++) {
        const uint8_t *r = e->pad + (size_t)(y0 + cy + dy - ME_SPAN / 2 +
                                             ME_PAD) * e->pad_stride +
                           x0 + cx - ME_SPAN / 2 + ME_PAD;

        for (int dx = lo; dx < lo + span; dx++) {
            int64_t lim = best - bx[dx] - by[dy], c;
            uint32_t sad;

            if (lim <= 0) {
                continue;
            }
            sad = hevc_sad_lim(s, e->src_stride[0], r + dx, e->pad_stride, n,
                               MIN(lim, UINT32_MAX));
            c = (int64_t)sad + bx[dx] + by[dy];
            if (c < best) {
                best = c;
                ix = dx;
                iy = dy;
            }
        }
    }
    *out = (HevcMv){ 4 * (cx + ix - ME_SPAN / 2), 4 * (cy + iy - ME_SPAN / 2) };
    return best;
}

/*
 * The best whole-sample vector of a block of 16 or 32: the coarse stage's
 * best position and the whole samples round it (and zero and the centre):
 * a block no coarse match lands near is not found
 */
static int64_t hevc_ime(const HevcEnc *e, int x0, int y0, int n, int cx, int cy,
                        HevcMv *out)
{
    const uint8_t *s = e->src[0] + y0 * e->src_stride[0] + x0;
    HevcMv c, cand[11];
    int64_t best;
    int nc = 0;

    if (n < 16) {
        return hevc_ime_full(e, x0, y0, n, cx, cy, ME_SPAN, out);
    }
    best = hevc_ime_full(e, x0, y0, n, cx, cy, ME_SPAN / 2, out);
    c = hevc_cme(e, x0, y0, n, cx, cy);
    cand[nc++] = (HevcMv){ 0, 0 };
    cand[nc++] = (HevcMv){ cx, cy };
    for (int k = 0; k < 9; k++) {
        cand[nc++] = (HevcMv){ 2 * c.x + k % 3 - 1, 2 * c.y + k / 3 - 1 };
    }
    for (int k = 0; k < nc; k++) {
        int vx = cand[k].x, vy = cand[k].y;
        int64_t cost;

        if (vx < cx - ME_SPAN / 2 || vx >= cx + ME_SPAN / 2 ||
            vy < cy - ME_SPAN / 2 || vy >= cy + ME_SPAN / 2) {
            continue;
        }
        cost = hevc_sad_lim(s, e->src_stride[0],
                            e->pad + (size_t)(y0 + vy + ME_PAD) * e->pad_stride +
                            x0 + vx + ME_PAD, e->pad_stride, n, UINT32_MAX) +
               (int64_t)e->lam_me * (hevc_mvd_bits(4 * vx) +
                                     hevc_mvd_bits(4 * vy));
        if (cost < best) {
            best = cost;
            *out = (HevcMv){ 4 * vx, 4 * vy };
        }
    }
    return best;
}

static inline bool hevc_mv_eq(HevcMv a, HevcMv b)
{
    return a.x == b.x && a.y == b.y;
}

/* The decoded motion of the 4x4 block at (xn, yn), if it is inter */
static bool hevc_nb(const HevcEnc *e, int xn, int yn, HevcMv *mv)
{
    uint32_t blk;

    if (xn < 0 || yn < 0 || xn >= e->w || yn >= e->h) {
        return false;
    }
    blk = (yn >> 2) * e->w4 + (xn >> 2);
    if (!e->done[blk] || !e->imap[blk]) {
        return false;
    }
    *mv = e->mvmap[blk];
    return true;
}

/*
 * The temporal candidate (8.6.2.2.2), the reference's motion at the block's
 * bottom right, in the CTB row, else at its centre. Every block of a channel
 * refers to the picture before, so the picture distances are alike and the
 * vector is not scaled.
 */
static bool hevc_tmvp(const HevcEnc *e, int x0, int y0, int n, HevcMv *mv)
{
    int xb = x0 + n, yb = y0 + n, lc = e->syn->log2_ctb;

    if (!e->col) {
        return false;
    }
    if ((y0 >> lc) == (yb >> lc) && yb < e->h && xb < e->w) {
        const HevcMvf *c = &e->col[(yb >> 4) * e->w16 + (xb >> 4)];

        if (c->inter) {
            *mv = c->mv;
            return true;
        }
    }
    {
        const HevcMvf *c = &e->col[((y0 + n / 2) >> 4) * e->w16 +
                                   ((x0 + n / 2) >> 4)];

        if (c->inter) {
            *mv = c->mv;
            return true;
        }
    }
    return false;
}

/* The merge candidates of a 2Nx2N block (8.6.2.2): count, in @list */
static int hevc_merge_list(const HevcEnc *e, int x0, int y0, int n,
                           HevcMv *list)
{
    int max = e->syn->merge_cands, cnt = 0;
    HevcMv a1, b1, b0, a0, b2, t;
    bool fa1 = hevc_nb(e, x0 - 1, y0 + n - 1, &a1);
    bool fb1 = hevc_nb(e, x0 + n - 1, y0 - 1, &b1);
    bool fb0 = hevc_nb(e, x0 + n, y0 - 1, &b0);
    bool fa0 = hevc_nb(e, x0 - 1, y0 + n, &a0);
    bool fb2 = hevc_nb(e, x0 - 1, y0 - 1, &b2);
    bool ua1 = fa1, ub1 = fb1 && !(fa1 && hevc_mv_eq(a1, b1));
    bool ub0 = fb0 && !(fb1 && hevc_mv_eq(b1, b0));
    bool ua0 = fa0 && !(fa1 && hevc_mv_eq(a1, a0));
    bool ub2 = fb2 && !(fa1 && hevc_mv_eq(a1, b2)) &&
               !(fb1 && hevc_mv_eq(b1, b2)) &&
               ua1 + ub1 + ub0 + ua0 != 4;

    if (ua1) {
        list[cnt++] = a1;
    }
    if (ub1) {
        list[cnt++] = b1;
    }
    if (ub0) {
        list[cnt++] = b0;
    }
    if (ua0) {
        list[cnt++] = a0;
    }
    if (ub2) {
        list[cnt++] = b2;
    }
    if (cnt < max && hevc_tmvp(e, x0, y0, n, &t)) {
        list[cnt++] = t;
    }
    while (cnt < max) {
        list[cnt++] = (HevcMv){ 0, 0 };
    }
    return max;
}

/* The two AMVP candidates of a block (8.6.2.6) */
static void hevc_amvp(const HevcEnc *e, int x0, int y0, int n, HevcMv *cand)
{
    HevcMv a0, a1, b0, b1, b2, a = { 0, 0 }, b = { 0, 0 }, t;
    bool fa0 = hevc_nb(e, x0 - 1, y0 + n, &a0);
    bool fa1 = hevc_nb(e, x0 - 1, y0 + n - 1, &a1);
    bool fb0 = hevc_nb(e, x0 + n, y0 - 1, &b0);
    bool fb1 = hevc_nb(e, x0 + n - 1, y0 - 1, &b1);
    bool fb2 = hevc_nb(e, x0 - 1, y0 - 1, &b2);
    bool have_a = fa0 || fa1, have_b = fb0 || fb1 || fb2;
    int cnt = 0;

    if (have_a) {
        a = fa0 ? a0 : a1;
    }
    if (have_b) {
        b = fb0 ? b0 : fb1 ? b1 : b2;
    }
    if (!(fa0 || fa1) && have_b) {
        /* without a left neighbour the above one stands in for it too */
        a = b;
        have_a = true;
    }
    if (have_a) {
        cand[cnt++] = a;
    }
    if (have_b && !(have_a && hevc_mv_eq(a, b))) {
        cand[cnt++] = b;
    }
    if (cnt < 2 && hevc_tmvp(e, x0, y0, n, &t)) {
        cand[cnt++] = t;
    }
    while (cnt < 2) {
        cand[cnt++] = (HevcMv){ 0, 0 };
    }
}

/* The search window's centre: the reference's vector here, on a grid of 8 */
static void hevc_centre(const HevcEnc *e, int x0, int y0, int *cx, int *cy)
{
    *cx = *cy = 0;
    if (e->col) {
        const HevcMvf *c = &e->col[(y0 >> 4) * e->w16 + (x0 >> 4)];

        if (c->inter) {
            *cx = (((c->mv.x + 2) >> 2) + 4) & ~7;
            *cy = (((c->mv.y + 2) >> 2) + 4) & ~7;
            *cx = MIN(MAX(*cx, -48), 48);
            *cy = MIN(MAX(*cy, -48), 48);
        }
    }
}

static int64_t hevc_mv_cost(const HevcEnc *e, uint32_t sad, HevcMv mv,
                            HevcMv pred)
{
    return sad + (int64_t)e->lam_me * (hevc_mvd_bits(mv.x - pred.x) +
                                       hevc_mvd_bits(mv.y - pred.y));
}

static uint32_t hevc_mv_sad(const HevcEnc *e, int x0, int y0, int n,
                            HevcMv mv)
{
    uint8_t *blk = e->sc->pred;

    hevc_mc(e->sc, e->ref[0], e->w, e->h, x0, y0, n, n, mv.x, mv.y, 2, blk, n);
    return hevc_sad(e->src[0] + y0 * e->src_stride[0] + x0,
                    e->src_stride[0], blk, n, n, n);
}

/*
 * The vector of the block at (x0, y0), from its best whole sample @m: the
 * neighbours' vectors are tried, then a ring of halves and one of quarters
 * round the best. @pred is what the vector's difference is taken from.
 */
static HevcMv hevc_search(const HevcEnc *e, int x0, int y0, int n, HevcMv m,
                          HevcMv pred)
{
    HevcMv seeds[8];
    int64_t best = hevc_mv_cost(e, hevc_mv_sad(e, x0, y0, n, m), m, pred);
    int ns = 0;

    seeds[ns++] = (HevcMv){ 0, 0 };
    {
        HevcMv sd;
        const HevcMvf *c = e->col ? &e->col[(y0 >> 4) * e->w16 + (x0 >> 4)]
                                  : NULL;

        if (hevc_nb(e, x0 - 1, y0 + n - 1, &sd)) {
            seeds[ns++] = sd;
        }
        if (hevc_nb(e, x0 + n - 1, y0 - 1, &sd)) {
            seeds[ns++] = sd;
        }
        if (hevc_nb(e, x0 + n, y0 - 1, &sd)) {
            seeds[ns++] = sd;
        }
        if (c && c->inter) {
            seeds[ns++] = c->mv;
        }
        if (hevc_tmvp(e, x0, y0, n, &sd)) {
            seeds[ns++] = sd;
        }
    }
    for (int i = 0; i < ns; i++) {
        int64_t c;

        if (hevc_mv_eq(seeds[i], m)) {
            continue;
        }
        c = hevc_mv_cost(e, hevc_mv_sad(e, x0, y0, n, seeds[i]), seeds[i],
                         pred);
        if (c < best) {
            best = c;
            m = seeds[i];
        }
    }
    for (int step = 2; step >= 1; step--) {
        HevcMv c0 = m;

        for (int k = 0; k < 9; k++) {
            HevcMv t = { c0.x + (k % 3 - 1) * step, c0.y + (k / 3 - 1) * step };
            int64_t c;

            if (k == 4) {
                continue;
            }
            c = hevc_mv_cost(e, hevc_mv_sad(e, x0, y0, n, t), t, pred);
            if (c < best) {
                best = c;
                m = t;
            }
        }
    }
    return m;
}

/* An inter coding of a block: what the stream says and what it makes */
struct HevcInter {
    HevcMv mv, mvd;
    int merge;                  /* merge_idx, or -1 for AMVP */
    int mvp;                    /* mvp_l0_flag */
    uint8_t cbf;                /* luma, Cb, Cr */
    uint64_t ssd;
    int16_t level[3][32 * 32];
    uint8_t rec[3][32 * 32];    /* the planes' blocks, each n / sub wide */
};

/* Predict the block at @mv, add the residual when @res, and measure */
static void hevc_inter_make(const HevcEnc *e, int x0, int y0, int log2,
                            HevcMv mv, bool res, HevcInter *ic)
{
    int n = 1 << log2;

    ic->cbf = 0;
    ic->ssd = 0;
    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = n / sub, sz = log2 - !!i;
        int bx = x0 / sub, by = y0 / sub, stride = e->w / sub;
        const uint8_t *src = e->src[i] + by * e->src_stride[i] + bx;
        int16_t *r = e->sc->res;
        int32_t *level = e->sc->level;
        uint8_t *rec = ic->rec[i];
        bool cbf = false;

        hevc_mc(e->sc, e->ref[i], stride, e->h / sub, bx, by, m, m, mv.x, mv.y,
                i ? 3 : 2, rec, m);
        if (res) {
            for (int y = 0; y < m; y++) {
                for (int x = 0; x < m; x++) {
                    r[y * m + x] = src[y * e->src_stride[i] + x] -
                                   rec[y * m + x];
                }
            }
            cbf = hevc_quant(r, sz, i ? e->qp_c[i - 1] : e->qp,
                             !e->pic->scaling_list, e->syn->sign_hiding,
                             true, 171, level);
        }
        if (cbf) {
            ic->cbf |= 1 << i;
            for (int k = 0; k < m * m; k++) {
                ic->level[i][k] = level[k];
                rec[k] = MIN(MAX(rec[k] + r[k], 0), 255);
            }
        }
        for (int y = 0; y < m; y++) {
            for (int x = 0; x < m; x++) {
                int d = src[y * e->src_stride[i] + x] - rec[y * m + x];

                ic->ssd += d * d;
            }
        }
    }
}

/* merge_idx: a truncated unary code, its first bin in a context */
static void hevc_merge_idx(HevcEnc *e, int idx)
{
    int cmax = e->syn->merge_cands - 1;

    for (int i = 0; i < cmax; i++) {
        if (i) {
            cabac_bypass(&e->c, idx > i, 1);
        } else {
            cabac_bin(&e->c, CTX_MERGE_IDX, idx > 0);
        }
        if (idx <= i) {
            break;
        }
    }
}

/* mvd_coding: greater-than flags, then the remainders of EG1 and the signs */
static void hevc_mvd_code(Cabac *c, int dx, int dy)
{
    int d[2] = { dx, dy };

    cabac_bin(c, CTX_MVD_GT0, dx != 0);
    cabac_bin(c, CTX_MVD_GT0, dy != 0);
    for (int i = 0; i < 2; i++) {
        if (d[i]) {
            cabac_bin(c, CTX_MVD_GT1, abs(d[i]) > 1);
        }
    }
    for (int i = 0; i < 2; i++) {
        int v = abs(d[i]), k = 1;

        if (!d[i]) {
            continue;
        }
        if (v > 1) {
            for (v -= 2; v >= 1 << k; k++) {
                cabac_bypass(c, 1, 1);
                v -= 1 << k;
            }
            cabac_bypass(c, 0, 1);
            cabac_bypass(c, v, k);
        }
        cabac_bypass(c, d[i] < 0, 1);
    }
}

/* What a coding block leaves for the blocks after it and the deblocking */
static void hevc_mark(HevcEnc *e, int x0, int y0, int n, bool inter, HevcMv mv,
                      bool luma_levels)
{
    for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
        for (int x = x0 >> 2; x < (x0 + n) >> 2; x++) {
            uint32_t i = y * e->w4 + x;

            e->imap[i] = inter;
            e->mvmap[i] = mv;
            e->cmap[i] = luma_levels;
            e->done[i] = 1;
        }
    }
}

static void hevc_put(HevcEnc *e, int x0, int y0, int n,
                     const uint8_t rec[3][32 * 32])
{
    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = n / sub, stride = e->w / sub;

        for (int y = 0; y < m; y++) {
            memcpy(e->rec[i] + (y0 / sub + y) * stride + x0 / sub,
                   rec[i] + y * m, m);
        }
    }
}

/* The skipped block, as the merge candidate @idx has it */
static void hevc_pskip_emit(HevcEnc *e, int x0, int y0, int log2, int ctx,
                            int idx, HevcMv mv)
{
    int n = 1 << log2;

    cabac_bin(&e->c, CTX_SKIP + ctx, 1);
    if (e->syn->merge_cands > 1) {
        hevc_merge_idx(e, idx);
    }
    hevc_set_map(e, e->mode, x0, y0, n, MODE_DC);
    hevc_set_map(e, e->skip, x0, y0, n, 1);
    hevc_mark(e, x0, y0, n, true, mv, false);
    e->st->cus[log2 - LOG2_MIN_CB]++;
    e->st->skip_8x8 += (n / 8) * (n / 8);
}

/* An inter block that is not skipped */
static void hevc_pinter_emit(HevcEnc *e, int x0, int y0, int log2, int ctx,
                             const HevcInter *ic)
{
    Cabac *c = &e->c;
    int n = 1 << log2;
    bool cbf[3] = { ic->cbf & 1, ic->cbf >> 1 & 1, ic->cbf >> 2 };

    cabac_bin(c, CTX_SKIP + ctx, 0);
    cabac_bin(c, CTX_PRED_MODE, 0);         /* MODE_INTER */
    cabac_bin(c, CTX_PART_MODE, 1);         /* PART_2Nx2N */
    cabac_bin(c, CTX_MERGE_FLAG, ic->merge >= 0);
    if (ic->merge >= 0) {
        if (e->syn->merge_cands > 1) {
            hevc_merge_idx(e, ic->merge);
        }
    } else {
        hevc_mvd_code(c, ic->mvd.x, ic->mvd.y);
        cabac_bin(c, CTX_MVP, ic->mvp);
        cabac_bin(c, CTX_RQT_ROOT, ic->cbf != 0);
    }
    if (ic->cbf) {
        const int16_t *const level[3] = { ic->level[0], ic->level[1],
                                          ic->level[2] };

        hevc_inter_tu(e, log2, ic->cbf, level);
    }
    hevc_set_map(e, e->mode, x0, y0, n, MODE_DC);
    hevc_mark(e, x0, y0, n, true, ic->mv, cbf[0]);
    e->st->cus[log2 - LOG2_MIN_CB]++;
}

typedef struct HevcSave {
    Cabac c;
    guint len;
    IngenicAvpuStats st;
    bool coded;
} HevcSave;

static void hevc_save(const HevcEnc *e, HevcSave *s)
{
    s->c = e->c;
    s->len = e->c.rbsp->len;
    s->st = *e->st;
    s->coded = e->qp_delta_coded;
}

static void hevc_restore(HevcEnc *e, const HevcSave *s)
{
    e->c = s->c;
    g_byte_array_set_size(e->c.rbsp, s->len);
    *e->st = s->st;
    e->qp_delta_coded = s->coded;
}
static void hevc_set_order(HevcEnc *e, int x0, int y0, int n, uint32_t v)
{
    for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
        for (int x = x0 >> 2; x < (x0 + n) >> 2; x++) {
            e->order[y * e->w4 + x] = v;
        }
    }
}


/* The best whole-sample vector of a block: the CTB's, or a search of its own */
static HevcMv hevc_block_ime(HevcEnc *e, int x0, int y0, int n)
{
    HevcMv m;
    int cx, cy;

    if (e->ime_ok && n >= 16 && x0 >= e->ime_x && y0 >= e->ime_y &&
        x0 < e->ime_x + 32 && y0 < e->ime_y + 32) {
        return e->ime[n == 32 ? 0 : 1 + (((y0 - e->ime_y) >> 4) * 2 +
                                         ((x0 - e->ime_x) >> 4))];
    }
    hevc_centre(e, x0, y0, &cx, &cy);
    hevc_ime(e, x0, y0, n, cx, cy, &m);
    return m;
}

/* Whether skipping with the first merge candidate is certainly cheapest */
static bool hevc_skip_sure(const HevcEnc *e, int x0, int y0, int log2, int ctx,
                           const HevcMv *ml, HevcInter *ic)
{
    int64_t j;

    hevc_inter_make(e, x0, y0, log2, ml[0], false, ic);
    j = (int64_t)ic->ssd * 256 +
        e->lambda * (cabac_cost(&e->c, CTX_SKIP + ctx, 1) +
                     (e->syn->merge_cands > 1 ?
                      cabac_cost(&e->c, CTX_MERGE_IDX, 0) : 0)) / 256;
    return j < e->lambda * (cabac_shifts(&e->c, CTX_SKIP + ctx, 0) + 1);
}

/*
 * How a CTB is coded: 0 as one 32x32 block, 1 as four of 16x16 where their
 * vectors differ, 2 as whichever costs less where one of the four holds
 * little of what the block's vector leaves to code (a flat quarter beside a
 * textured one): a transform of the whole spreads the textured quarter's
 * quantization error over the flat ones, which a search then finds the
 * vectors of
 */
static int hevc_p_split(HevcEnc *e, int x0, int y0)
{
    HevcMv ml[5];
    HevcInter *ic = &e->ic[0];
    int cx, cy;
    int64_t c32, c16 = 0;
    uint32_t lo = UINT32_MAX, hi = 0;

    e->ime_ok = false;
    hevc_merge_list(e, x0, y0, 32, ml);
    if (hevc_skip_sure(e, x0, y0, 5, hevc_skip_ctx(e, x0, y0), ml, ic)) {
        return 0;
    }
    hevc_centre(e, x0, y0, &cx, &cy);
    c32 = hevc_ime(e, x0, y0, 32, cx, cy, &e->ime[0]);
    for (int q = 0; q < 4; q++) {
        int qx = x0 + 16 * (q & 1), qy = y0 + 16 * (q >> 1);
        uint32_t sad;

        c16 += hevc_ime(e, qx, qy, 16, cx, cy, &e->ime[1 + q]);
        sad = hevc_mv_sad(e, qx, qy, 16, e->ime[0]);
        lo = MIN(lo, sad);
        hi = MAX(hi, sad);
    }
    e->ime_ok = true;
    e->ime_x = x0;
    e->ime_y = y0;
    if (c16 + 4 * e->lam_me < c32) {
        return 1;
    }
    return hi >= 1024 && (uint64_t)lo * 8 < hi ? 2 : 0;
}

/* A coding block of a P picture that searches for motion */
static void hevc_pcu(HevcEnc *e, int x0, int y0, int log2)
{
    int n = 1 << log2, ctx = hevc_skip_ctx(e, x0, y0), nm, skip_idx = 0;
    uint32_t seq = e->seq++;
    HevcMv ml[5], amvp[2], mv;
    HevcInter *ic_skip = &e->ic[0], *ic_res = &e->ic[1], *ic_nr = &e->ic[2],
              *tmp = &e->ic[3];
    HevcSave sv;
    int64_t best_j = INT64_MAX;
    uint64_t best_ssd = 0;
    int kind = 0;
    uint64_t t0;

    hevc_set_order(e, x0, y0, n, seq);
    if (x0 > 0) {
        for (int y = y0 >> 2; y < (y0 + n) >> 2; y++) {
            e->vedge[y * (e->w / 8) + (x0 >> 3)] = 1;
        }
    }
    if (y0 > 0) {
        memset(e->hedge + (y0 >> 3) * e->w4 + (x0 >> 2), 1, n >> 2);
    }
    nm = hevc_merge_list(e, x0, y0, n, ml);
    if (hevc_skip_sure(e, x0, y0, log2, ctx, ml, ic_skip)) {
        hevc_pskip_emit(e, x0, y0, log2, ctx, 0, ml[0]);
        hevc_put(e, x0, y0, n, ic_skip->rec);
        goto done;
    }
    hevc_save(e, &sv);

    hevc_amvp(e, x0, y0, n, amvp);
    mv = hevc_search(e, x0, y0, n, hevc_block_ime(e, x0, y0, n),
                     (HevcMv){ 0, 0 });
    for (int k = 0; k < nm; k++) {
        int64_t j;
        bool dup = false;

        for (int i = 0; i < k; i++) {
            dup |= hevc_mv_eq(ml[i], ml[k]);
        }
        if (dup || (k && !hevc_mv_eq(ml[k], mv))) {
            continue;
        }
        if (k) {
            hevc_inter_make(e, x0, y0, log2, ml[k], false, tmp);
        }
        t0 = cabac_tell(&e->c);
        hevc_pskip_emit(e, x0, y0, log2, ctx, k, ml[k]);
        j = (int64_t)(k ? tmp : ic_skip)->ssd * 256 +
            e->lambda * (int64_t)(cabac_tell(&e->c) - t0);
        hevc_restore(e, &sv);
        if (j < best_j) {
            best_j = j;
            best_ssd = (k ? tmp : ic_skip)->ssd;
            skip_idx = k;
            if (k) {
                memcpy(ic_skip->rec, tmp->rec, sizeof(ic_skip->rec));
                ic_skip->ssd = tmp->ssd;
            }
        }
    }

    {
        int64_t j;
        int merge = -1;

        for (int k = 0; k < nm && merge < 0; k++) {
            if (hevc_mv_eq(ml[k], mv)) {
                merge = k;
            }
        }
        ic_res->mv = mv;
        ic_res->merge = merge;
        if (merge < 0) {
            int b0 = hevc_mvd_bits(mv.x - amvp[0].x) +
                     hevc_mvd_bits(mv.y - amvp[0].y);
            int b1 = hevc_mvd_bits(mv.x - amvp[1].x) +
                     hevc_mvd_bits(mv.y - amvp[1].y);

            ic_res->mvp = b1 < b0;
            ic_res->mvd = (HevcMv){ mv.x - amvp[ic_res->mvp].x,
                                    mv.y - amvp[ic_res->mvp].y };
        }
        hevc_inter_make(e, x0, y0, log2, mv, true, ic_res);
        if (merge < 0 || ic_res->cbf) {
            t0 = cabac_tell(&e->c);
            hevc_pinter_emit(e, x0, y0, log2, ctx, ic_res);
            j = (int64_t)ic_res->ssd * 256 +
                e->lambda * (int64_t)(cabac_tell(&e->c) - t0);
            hevc_restore(e, &sv);
            if (j < best_j) {
                best_j = j;
                best_ssd = ic_res->ssd;
                kind = 1;
            }
        }
        if (merge < 0 && ic_res->cbf) {
            ic_nr->mv = mv;
            ic_nr->merge = -1;
            ic_nr->mvp = ic_res->mvp;
            ic_nr->mvd = ic_res->mvd;
            hevc_inter_make(e, x0, y0, log2, mv, false, ic_nr);
            t0 = cabac_tell(&e->c);
            hevc_pinter_emit(e, x0, y0, log2, ctx, ic_nr);
            j = (int64_t)ic_nr->ssd * 256 +
                e->lambda * (int64_t)(cabac_tell(&e->c) - t0);
            hevc_restore(e, &sv);
            if (j < best_j) {
                best_j = j;
                best_ssd = ic_nr->ssd;
                kind = 2;
            }
        }
    }

    if (best_ssd > (uint64_t)40 * n * n) {
        HevcCand *cd = &e->scratch;
        uint64_t j_intra;

        cd->x = x0;
        cd->y = y0;
        cd->log2 = log2;
        cd->ready = false;
        hevc_ensure_cand(e, cd);
        t0 = cabac_tell(&e->c);
        cabac_bin(&e->c, CTX_SKIP + ctx, 0);
        cabac_bin(&e->c, CTX_PRED_MODE, 1);     /* MODE_INTRA */
        hevc_intra_emit(e, cd);
        j_intra = cd->ssd_intra * 256 +
                  e->lambda * (int64_t)(cabac_tell(&e->c) - t0);
        if (j_intra < best_j) {
            hevc_set_map(e, e->skip, x0, y0, n, 0);
            hevc_mark(e, x0, y0, n, false, (HevcMv){ 0, 0 }, false);
            goto done;
        }
        hevc_restore(e, &sv);
    }
    hevc_set_map(e, e->skip, x0, y0, n, 0);
    if (kind == 0) {
        hevc_pskip_emit(e, x0, y0, log2, ctx, skip_idx, ml[skip_idx]);
        hevc_put(e, x0, y0, n, ic_skip->rec);
    } else {
        HevcInter *ic = kind == 1 ? ic_res : ic_nr;

        hevc_pinter_emit(e, x0, y0, log2, ctx, ic);
        hevc_put(e, x0, y0, n, ic->rec);
    }
done:
    hevc_set_map(e, e->qpy, x0, y0, n, hevc_cu_qpy(e));
}

/* A CTB's state before it is coded, to code it again another way */
typedef struct HevcSnap {
    HevcSave sv;
    uint32_t seq;
    int qp_pred;
    uint8_t m4[7][64];          /* mode, depth, skip, qpy, imap, cmap, done */
    uint32_t order[64];
    HevcMv mv[64];
    uint8_t vedge[8 * 4], hedge[4 * 8];
    uint8_t rec[3][32 * 32];
} HevcSnap;

static void hevc_snap(HevcEnc *e, int x0, int y0, bool put, HevcSnap *sn)
{
    uint8_t *const maps[7] = { e->mode, e->depth, e->skip, e->qpy, e->imap,
                               e->cmap, e->done };
    int x4 = x0 >> 2, y4 = y0 >> 2;

    if (!put) {
        hevc_save(e, &sn->sv);
        sn->seq = e->seq;
        sn->qp_pred = e->qp_pred;
    } else {
        hevc_restore(e, &sn->sv);
        e->seq = sn->seq;
        e->qp_pred = sn->qp_pred;
    }
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            uint32_t i = (y4 + y) * e->w4 + x4 + x;

            for (int m = 0; m < 7; m++) {
                if (put) {
                    maps[m][i] = sn->m4[m][y * 8 + x];
                } else {
                    sn->m4[m][y * 8 + x] = maps[m][i];
                }
            }
            if (put) {
                e->order[i] = sn->order[y * 8 + x];
                e->mvmap[i] = sn->mv[y * 8 + x];
            } else {
                sn->order[y * 8 + x] = e->order[i];
                sn->mv[y * 8 + x] = e->mvmap[i];
            }
        }
        for (int x = 0; x < 4; x++) {
            uint8_t *v = e->vedge + (y4 + y) * (e->w / 8) + (x0 >> 3) + x;

            if (put) {
                *v = sn->vedge[y * 4 + x];
            } else {
                sn->vedge[y * 4 + x] = *v;
            }
        }
    }
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 8; x++) {
            uint8_t *h = e->hedge + ((y0 >> 3) + y) * e->w4 + x4 + x;

            if (put) {
                *h = sn->hedge[y * 8 + x];
            } else {
                sn->hedge[y * 8 + x] = *h;
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        int sub = i ? 2 : 1, m = 32 / sub, stride = e->w / sub;

        for (int y = 0; y < m; y++) {
            uint8_t *r = e->rec[i] + (y0 / sub + y) * stride + x0 / sub;

            if (put) {
                memcpy(r, sn->rec[i] + y * m, m);
            } else {
                memcpy(sn->rec[i] + y * m, r, m);
            }
        }
    }
}

/* @split: -1 to decide, 0 for one block, 1 for four, where 32x32 fits */
static void hevc_pquadtree(HevcEnc *e, int x0, int y0, int log2, int depth,
                           int split_as)
{
    int n = 1 << log2;
    bool split;

    if (log2 >= e->log2_qg) {
        e->qp_delta_coded = false;
    }
    if (x0 + n <= e->w && y0 + n <= e->h && log2 > LOG2_MIN_CB) {
        int ctx = 0;

        if (x0 > 0) {
            ctx += e->depth[(y0 >> 2) * e->w4 + ((x0 - 1) >> 2)] > depth;
        }
        if (y0 > 0) {
            ctx += e->depth[((y0 - 1) >> 2) * e->w4 + (x0 >> 2)] > depth;
        }
        if (split_as < 0) {
            /* a CTB of 64 is coded as four of 32, the search's own size */
            split_as = log2 == 5 ? hevc_p_split(e, x0, y0) : log2 > 5;
        }
        if (split_as == 2) {
            HevcSnap *sn = e->snap;
            const uint8_t *rec[3] = { e->rec[0], e->rec[1], e->rec[2] };
            int64_t j[2];

            hevc_snap(e, x0, y0, false, sn);
            for (int k = 0; k < 2; k++) {
                uint64_t t0 = cabac_tell(&e->c);

                hevc_pquadtree(e, x0, y0, log2, depth, k);
                j[k] = (int64_t)hevc_ssd(e, x0, y0, n, rec) * 256 +
                       e->lambda * (int64_t)(cabac_tell(&e->c) - t0);
                if (!k) {
                    hevc_snap(e, x0, y0, true, sn);
                }
            }
            if (j[1] < j[0]) {
                return;
            }
            hevc_snap(e, x0, y0, true, sn);
            hevc_pquadtree(e, x0, y0, log2, depth, 0);
            return;
        }
        split = split_as == 1;
        cabac_bin(&e->c, CTX_SPLIT_CU + ctx, split);
    } else {
        split = log2 > LOG2_MIN_CB;
    }
    if (split) {
        int h = n / 2;

        for (int i = 0; i < 4; i++) {
            int x = x0 + (i & 1) * h, y = y0 + (i >> 1) * h;

            if (x < e->w && y < e->h) {
                hevc_pquadtree(e, x, y, log2 - 1, depth + 1, -1);
            }
        }
    } else {
        hevc_set_map(e, e->depth, x0, y0, n, depth);
        hevc_pcu(e, x0, y0, log2);
    }
    if (log2 >= e->log2_qg && e->qp_delta_coded) {
        e->qp_pred = e->qp;
    }
}

/* Deblocking's boundary strength of the edge between 4x4 blocks pb and qb */
static int hevc_bs(const HevcEnc *e, uint32_t pb, uint32_t qb)
{
    HevcMv p = e->mvmap[pb], q = e->mvmap[qb];

    if (!e->imap[pb] || !e->imap[qb]) {
        return 2;
    }
    if (e->cmap[pb] || e->cmap[qb]) {
        return 1;
    }
    return abs(p.x - q.x) >= 4 || abs(p.y - q.y) >= 4;
}

/* Set up the search of a P picture: the reference with a border, the maps */
static void hevc_search_begin(HevcEnc *e, const HevcMvf *col)
{
    uint32_t w = e->w, h = e->h;
    size_t pw = w + 2 * ME_PAD, ph = h + 2 * ME_PAD;
    uint8_t *pad = g_malloc(pw * ph);

    for (size_t y = 0; y < ph; y++) {
        const uint8_t *src = e->ref[0] + MIN(MAX((int)y - ME_PAD, 0),
                                             (int)h - 1) * (size_t)w;
        uint8_t *dst = pad + y * pw;

        memset(dst, src[0], ME_PAD);
        memcpy(dst + ME_PAD, src, w);
        memset(dst + ME_PAD + w, src[w - 1], ME_PAD);
    }
    {
        uint32_t dw = w / 2, dh = h / 2, dpw = dw + 2 * ME_DPAD;
        uint8_t *dpad = g_malloc((size_t)dpw * (dh + 2 * ME_DPAD));

        e->dsrc = g_malloc((size_t)dw * dh);
        e->dsrc_stride = dw;
        for (uint32_t y = 0; y < dh; y++) {
            const uint8_t *a = e->src[0] + 2 * y * e->src_stride[0];
            const uint8_t *r = e->ref[0] + 2 * (size_t)y * w;
            uint8_t *row = dpad + (size_t)(y + ME_DPAD) * dpw + ME_DPAD;

            for (uint32_t x = 0; x < dw; x++) {
                e->dsrc[y * dw + x] = (a[2 * x] + a[2 * x + 1] +
                                       a[e->src_stride[0] + 2 * x] +
                                       a[e->src_stride[0] + 2 * x + 1] + 2) >> 2;
                row[x] = (r[2 * x] + r[2 * x + 1] + r[w + 2 * x] +
                          r[w + 2 * x + 1] + 2) >> 2;
            }
            memset(row - ME_DPAD, row[0], ME_DPAD);
            memset(row + dw, row[dw - 1], ME_DPAD);
        }
        for (uint32_t y = 0; y < ME_DPAD; y++) {
            memcpy(dpad + (size_t)y * dpw, dpad + (size_t)ME_DPAD * dpw, dpw);
            memcpy(dpad + (size_t)(dh + ME_DPAD + y) * dpw,
                   dpad + (size_t)(dh + ME_DPAD - 1) * dpw, dpw);
        }
        e->dpad = dpad;
        e->dpad_stride = dpw;
    }
    e->search = true;
    e->pad = pad;
    e->pad_stride = pw;
    e->maps = g_malloc0(3 * e->w4 * e->h4);
    e->imap = e->maps;
    e->cmap = e->maps + e->w4 * e->h4;
    e->done = e->maps + 2 * e->w4 * e->h4;
    e->mvmap = g_new0(HevcMv, e->w4 * e->h4);
    e->ic = g_new(HevcInter, 4);
    e->sc = g_new(HevcScratch, 1);
    e->snap = g_new(HevcSnap, 1);
    e->col = col;
    e->w16 = DIV_ROUND_UP(w, 16);
    memset(e->order, 0xff, sizeof(uint32_t) * e->w4 * e->h4);
}

static void hevc_search_run(HevcEnc *e)
{
    uint32_t ctb = 1 << e->syn->log2_ctb;

    for (uint32_t j = 0; j < e->ctbs_h; j++) {
        for (uint32_t i = 0; i < e->ctbs_w; i++) {
            int qp = e->qp = hevc_ctb_qp(e, i, j);

            e->qp_c[0] = hevc_qp_c(qp + e->pic->chroma_qp_offset);
            e->qp_c[1] = hevc_qp_c(qp + e->pic->cr_qp_offset);
            e->lambda = hevc_p_lambda(qp);
            e->lam_me = e->pic->lambda_p[qp][1] ?:
                        MAX(lround(sqrt(0.57 * pow(2, (qp - 12) / 3.0))), 1);
            e->ime_ok = false;
            if (e->syn->sao_luma || e->syn->sao_chroma) {
                hevc_sao(e, i, j);
            }
            hevc_pquadtree(e, i * ctb, j * ctb, e->syn->log2_ctb, 0, -1);
            cabac_terminate(&e->c, i == e->ctbs_w - 1 && j == e->ctbs_h - 1);
        }
    }
}

/* What the picture leaves for the guest and for the next one */
static void hevc_search_end(HevcEnc *e, IngenicAvpuHevc *h)
{
    size_t units = (size_t)e->w16 * DIV_ROUND_UP(e->h, 16);
    int per = 1 << (2 * (e->syn->log2_ctb - 4));    /* 16x16 units a CTB */
    int side = 1 << (e->syn->log2_ctb - 4);
    size_t recs = (size_t)e->ctbs_w * e->ctbs_h * per;
    HevcMvf *mvf = g_new0(HevcMvf, units);

    g_free(h->mvrec);
    h->mvrec = g_new0(uint32_t, recs * 4);
    h->mvrec_n = recs * 4;
    for (uint32_t cj = 0; cj < e->ctbs_h; cj++) {
        for (uint32_t ci = 0; ci < e->ctbs_w; ci++) {
            for (int q = 0; q < per; q++) {
                /* the units in Z order */
                uint32_t ux = ci * side + ((q & 1) | (q >> 2 & 1) << 1);
                uint32_t uy = cj * side + ((q >> 1 & 1) | (q >> 3 & 1) << 1);
                uint32_t blk, *r;

                if (ux * 16 >= e->w || uy * 16 >= e->h) {
                    continue;
                }
                blk = uy * 4 * e->w4 + ux * 4;
                mvf[uy * e->w16 + ux] = (HevcMvf){
                    .mv = e->mvmap[blk], .inter = e->imap[blk],
                };
                r = h->mvrec + ((cj * e->ctbs_w + ci) * per + q) * 4;
                if (e->imap[blk]) {
                    r[0] = (uint16_t)e->mvmap[blk].x |
                           (uint32_t)(uint16_t)e->mvmap[blk].y << 16;
                    r[2] = 0x100;
                }
            }
        }
    }
    g_free(h->mvf);
    h->mvf = mvf;
}

static void hevc_search_free(HevcEnc *e)
{
    g_free((void *)e->pad);
    g_free(e->dsrc);
    g_free(e->dpad);
    g_free(e->maps);
    g_free(e->mvmap);
    g_free(e->ic);
    g_free(e->sc);
    g_free(e->snap);
}

const uint32_t *ingenic_avpu_hevc_mvrec(IngenicAvpuHevc *h, size_t *n)
{
    *n = h->mvrec_n;
    return h->mvrec;
}

IngenicAvpuHevc *ingenic_avpu_hevc_new(void)
{
    return g_new0(IngenicAvpuHevc, 1);
}

uint8_t *ingenic_avpu_hevc_ref(IngenicAvpuHevc *h, uint32_t width,
                               uint32_t height)
{
    if (h->w != width || h->h != height) {
        g_free(h->ref);
        g_free(h->spare);
        g_free(h->mvf);
        h->spare = NULL;
        h->mvf = NULL;
        h->ref = g_malloc0(width * height * 3 / 2);
        h->w = width;
        h->h = height;
    }
    return h->ref;
}

uint8_t *ingenic_avpu_hevc_recon_nv12(IngenicAvpuHevc *h, uint32_t width,
                                      uint32_t height)
{
    uint8_t *out;
    size_t ysize = (size_t)width * height;
    const uint8_t *cb, *cr;

    if (!h->ref || h->w != width || h->h != height) {
        return NULL;
    }
    out = g_malloc(ysize * 3 / 2);
    memcpy(out, h->ref, ysize);
    cb = h->ref + ysize;
    cr = cb + ysize / 4;
    for (size_t i = 0; i < ysize / 4; i++) {
        out[ysize + 2 * i] = cb[i];
        out[ysize + 2 * i + 1] = cr[i];
    }
    return out;
}

void ingenic_avpu_hevc_put(QEMUFile *f, IngenicAvpuHevc *h)
{
    qemu_put_be32(f, h->ref ? h->w : 0);
    qemu_put_be32(f, h->ref ? h->h : 0);
    if (h->ref) {
        qemu_put_buffer(f, h->ref, h->w * h->h * 3 / 2);
    }
}

IngenicAvpuHevc *ingenic_avpu_hevc_get(QEMUFile *f)
{
    IngenicAvpuHevc *h = g_new0(IngenicAvpuHevc, 1);
    uint32_t w = qemu_get_be32(f), ht = qemu_get_be32(f);

    if (w > 8192 || ht > 8192) {
        g_free(h);
        return NULL;
    }
    if (w && ht) {
        qemu_get_buffer(f, ingenic_avpu_hevc_ref(h, w, ht), w * ht * 3 / 2);
    }
    return h;
}

void ingenic_avpu_hevc_free(IngenicAvpuHevc *h)
{
    if (h) {
        g_free(h->ref);
        g_free(h->spare);
        g_free(h->mvf);
        g_free(h->mvrec);
        g_free(h);
    }
}

/* Whether this picture is coded by the motion search */
static bool hevc_searches(const HevcEnc *e)
{
    return e->pic->h265_search && !e->pic->intra && e->ref[0] &&
           e->syn->log2_ctb <= 6;
}

bool ingenic_avpu_hevc_encode(IngenicAvpuHevc *h, const IngenicAvpuPic *pic,
                              const uint8_t *y, const uint8_t *uv,
                              uint32_t stride, GByteArray *out,
                              IngenicAvpuStats *st)
{
    g_autoptr(GByteArray) rbsp = g_byte_array_new();
    g_autofree uint8_t *src_c = NULL, *maps = NULL, *irec = NULL;
    uint8_t *planes;
    uint32_t cw, ch, ctbs_w, ctbs_h, w8, size, ctb;
    IngenicAvpuHevcSyntax avpu = avpu_syntax;
    HevcEnc e = { .pic = pic, .syn = pic->syntax ?: &avpu, .st = st };
    g_autofree uint32_t *order = NULL;
    uint32_t seq = 0;
    int threads;

    if (!pic->width || !pic->height || (pic->width | pic->height) & 7 ||
        pic->width > 8192 || pic->height > 8192 || pic->qp > 51) {
        return false;
    }
    hevc_tables_init();
    avpu.transform_skip = pic->transform_skip;
    e.w = pic->width;
    e.h = pic->height;
    e.w4 = e.w / 4;
    e.h4 = e.h / 4;
    w8 = e.w / 8;
    cw = e.w / 2;
    ch = e.h / 2;
    size = e.w * e.h + 2 * cw * ch;

    /* Deinterleave the chroma; reconstruct into the picture the channel had */
    if (h && h->spare && h->w == e.w && h->h == e.h) {
        planes = g_steal_pointer(&h->spare);
    } else {
        planes = g_malloc(size);
    }
    src_c = g_malloc(2 * cw * ch);
    e.rec[0] = planes;
    e.rec[1] = e.rec[0] + e.w * e.h;
    e.rec[2] = e.rec[1] + cw * ch;
    e.src[0] = y;
    e.src_stride[0] = stride;
    e.src[1] = src_c;
    e.src[2] = src_c + cw * ch;
    e.src_stride[1] = e.src_stride[2] = cw;
    for (uint32_t j = 0; j < ch; j++) {
        for (uint32_t i = 0; i < cw; i++) {
            ((uint8_t *)e.src[1])[j * cw + i] = uv[j * stride + 2 * i];
            ((uint8_t *)e.src[2])[j * cw + i] = uv[j * stride + 2 * i + 1];
        }
    }
    maps = g_malloc0(5 * e.w4 * e.h4 + w8 * e.h4 + e.w4 * (e.h / 8));
    e.mode = maps;
    e.depth = maps + e.w4 * e.h4;
    e.skip = maps + 2 * e.w4 * e.h4;
    e.qpy = maps + 3 * e.w4 * e.h4;
    e.bsmap = maps + 4 * e.w4 * e.h4;
    e.vedge = maps + 5 * e.w4 * e.h4;
    e.hedge = e.vedge + w8 * e.h4;

    /* the channel's last picture, if this P picture can refer to it */
    if (!pic->intra && h && h->ref && h->w == e.w && h->h == e.h) {
        e.ref[0] = h->ref;
        e.ref[1] = e.ref[0] + e.w * e.h;
        e.ref[2] = e.ref[1] + cw * ch;
        /* a skipped block leaves its samples as they are */
        memcpy(planes, h->ref, size);
        if (!hevc_searches(&e)) {
            irec = g_malloc(size);
            e.irec[0] = irec;
            e.irec[1] = irec + e.w * e.h;
            e.irec[2] = e.irec[1] + cw * ch;
        }
    }

    memset(st, 0, sizeof(*st));
    cabac_init(&e.c, pic->intra ? 0 : pic->cabac_init_idc ? 2 : 1, pic->qp,
               rbsp);
    ctb = 1 << e.syn->log2_ctb;
    ctbs_w = DIV_ROUND_UP(e.w, ctb);
    ctbs_h = DIV_ROUND_UP(e.h, ctb);
    e.ctbs_w = ctbs_w;
    e.ctbs_h = ctbs_h;
    e.exact = !e.ref[0];
    for (int i = 0; i < 3; i++) {
        e.cand[i] = e.rec[i];
    }
    order = g_new(uint32_t, e.w4 * e.h4);
    e.order = order;
    for (uint32_t j = 0; j < ctbs_h; j++) {
        for (uint32_t i = 0; i < ctbs_w; i++) {
            hevc_layout(&e, i * ctb, j * ctb, e.syn->log2_ctb, &seq);
        }
    }
    e.scratch.level[0] = e.scratch_level;
    e.scratch.level[1] = e.scratch_level + 32 * 32;
    e.scratch.level[2] = e.scratch.level[1] + 16 * 16;
    e.scratch.ilevel[0] = e.scratch.level[2] + 16 * 16;
    e.scratch.ilevel[1] = e.scratch.ilevel[0] + 32 * 32;
    e.scratch.ilevel[2] = e.scratch.ilevel[1] + 16 * 16;
    e.scratch.refs = e.scratch_refs;
    e.log2_qg = e.syn->log2_ctb - MAX(e.syn->qp_delta_depth, 0);
    e.qp_pred = pic->qp;

    threads = hevc_threads(pic);
    if (hevc_searches(&e)) {
        hevc_search_begin(&e, h && h->w == e.w && h->h == e.h ? h->mvf : NULL);
        hevc_search_run(&e);
    } else if (threads > 1 && ctbs_h > 1 && (uint64_t)e.w * e.h >= 256 * 128) {
        /* the candidates, made apart and in order of the CTBs */
        g_autofree HevcCand *cands = g_new0(HevcCand, seq);
        g_autofree uint32_t *ctb_first = g_new(uint32_t, ctbs_w * ctbs_h + 1);
        g_autofree int16_t *level_arena = g_new(int16_t, 2 * size);
        g_autofree uint8_t *refs_arena = g_malloc(e.refs_used);
        g_autofree uint8_t *extra = NULL, *spec_maps = NULL;

        e.cands = cands;
        e.ctb_first = ctb_first;
        e.level_arena = level_arena;
        e.refs_arena = refs_arena;
        if (!e.exact) {
            extra = g_malloc(size);
            spec_maps = g_malloc0(3 * e.w4 * e.h4);
            e.spec_mode = spec_maps;
            e.spec_skip = spec_maps + e.w4 * e.h4;
            e.spec_inter = spec_maps + 2 * e.w4 * e.h4;
            for (int i = 0; i < 3; i++) {
                e.cand[i] = extra + (e.rec[i] - e.rec[0]);
            }
        }
        seq = 0;
        e.level_used = e.refs_used = 0;
        for (uint32_t j = 0; j < ctbs_h; j++) {
            for (uint32_t i = 0; i < ctbs_w; i++) {
                ctb_first[j * ctbs_w + i] = seq;
                hevc_layout(&e, i * ctb, j * ctb, e.syn->log2_ctb, &seq);
            }
        }
        ctb_first[ctbs_w * ctbs_h] = seq;
        hevc_run_wave(&e, threads);
    } else {
        for (uint32_t j = 0; j < ctbs_h; j++) {
            for (uint32_t i = 0; i < ctbs_w; i++) {
                hevc_code_ctb(&e, i, j);
            }
        }
    }
    cabac_finish(&e.c);
    st->bins = e.c.bins;
    if (pic->raw) {
        g_byte_array_append(out, rbsp->data, rbsp->len);
    } else {
        ingenic_avpu_escape(out, rbsp->data, rbsp->len);
    }

    if (h) {
        if (e.search) {
            hevc_search_end(&e, h);
        } else if (pic->h265_search && h->w == e.w && h->h == e.h) {
            /* an intra picture: the next one's motion has nothing to take */
            g_free(h->mvf);
            h->mvf = g_new0(HevcMvf, (size_t)DIV_ROUND_UP(e.w, 16) *
                                      DIV_ROUND_UP(e.h, 16));
        } else {
            g_free(h->mvf);
            h->mvf = NULL;
        }
        hevc_deblock(&e);
        if (h->w != e.w || h->h != e.h) {
            g_free(h->ref);
            g_free(h->spare);
            g_free(h->mvf);
            h->mvf = NULL;
            h->ref = h->spare = NULL;
            h->w = e.w;
            h->h = e.h;
        }
        g_free(h->spare);
        h->spare = h->ref;
        h->ref = planes;
    } else {
        g_free(planes);
    }
    if (e.search) {
        hevc_search_free(&e);
    }
    return true;
}
