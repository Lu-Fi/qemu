/*
 * Ingenic JPEG engine: the T32's JPEG encoder, beside its HERA
 *
 * The helix driver resets the engine, writes a job's registers from
 * libimp's register image and starts it: an NV12 picture (0x14 luma, 0x18
 * chroma, both rows 0x10 apart) of 0xc bits 15:0 by 31:16, both multiples
 * of 16, coded as baseline 4:2:0 to 0x1c with the base tables at 0x300
 * (luma, then chroma, zigzag, four to a word, 7 bits of each byte) scaled
 * by the index in 0x4 bits 24:18: IJG's percentage for a quality of 99
 * less the index (two as one), times 5 / 512 for 1 / 100, rounded half
 * down. The engine writes the whole file, bare
 * (ingenic_avpu_jpeg_encode_bare(); the T33's taken for the same), with a
 * DCT of its own and a coefficient quantized as (c * r + 32768) >> 16: r
 * is 65536 / q rounded down, but up where q's odd part is one of those
 * below, as a T32NQ rounds an exact half of those divisors away from 0
 * and of every other one, bar powers of two, toward it (no q it can make
 * has an odd part of 97). A source address with bit 31 set comes over
 * IVDC: the ISP draws it (the "isp" link).
 *
 * Writing 0x0 bit 1 resets the engine, which then reads bit 31 set; bit 0
 * starts the job, which a T32 runs only after such a reset. At the end it
 * holds the file's length in 0x104, frame end in 0x100 bit 31 and 0x110
 * bit 0, or, past the limit at 0x2c that 0x4 bit 16 enables (without
 * sections, below), only 0x110 bit 1 and nothing written. Interrupt k
 * (0x110 bit k) is enabled by 0x4 bit 25 + k; 0x114 clears.
 *
 * The T33's engine can send the file in halves of 0x4c KiB, alternately
 * to 0x1c and 0x44, while 0x48 is set and interrupt 4 enabled: once a half
 * is full it holds 0x110 bit 4 with the bytes sent so far in 0x154 until
 * libimp has copied it and writes 1 to 0x50.
 *
 * 0x28 bit 0 asks for restart markers, (0x28 >> 16) + 1 MCUs apart, as
 * the AVPU's JPEG core writes them (a T32's decode the same as without).
 *
 * TRIG bit 9, with the reset and again with the start, makes the job a
 * decode (a T32Pro's engine in libimp's IMP_Decoder): the scan, as it is in
 * the file with its stuffing and RSTn, from 0x1c, up to the EOI libimp
 * appends, in 4:2:2 baseline with the Annex K tables, NMCU (0x8) + 1 MCUs
 * of 16 x 8 pixels, FRM_INFO's width a multiple of 16 across. The
 * quantizers are the two tables at 0x300 (luma, chroma, bytes in zigzag
 * order, four to a word); 0x28 bit 0 and (0x28 >> 16) MCUs between restarts.
 * The pixels go out packed, Y0 U Y1 V, from 0x14 as lines of a picture
 * row's half, 0x10 apart (libimp gives the width aligned to 128): a
 * picture row of 2 * width bytes is two lines. At the end 0x16c holds the
 * bytes written, 256 an MCU, and frame end is raised as for an encode.
 * 0x54 (the Bayer order libimp's IMP_Decoder takes) is 0 for the sample's
 * pixel format and taken as that for the others.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "hw/misc/ingenic-enc-tap.h"
#include "block/thread-pool.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "migration/qemu-file-types.h"
#include "qemu/aio-wait.h"
#include "qemu/error-report.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-jpeg.h"
#include "hw/misc/ingenic-avpu.h"
#include "hw/core/qdev-properties.h"

#define JPEG_TRIG           0x000
#define  TRIG_START         BIT(0)
#define  TRIG_RESET         BIT(1)
#define  TRIG_DECODE        BIT(9)
#define  TRIG_RESET_END     BIT(31)
#define JPEG_CTRL           0x004
#define  CTRL_LIMIT         BIT(16)
#define JPEG_NMCU           0x008
#define JPEG_FRM_INFO       0x00c
#define JPEG_LINE_STRD      0x010
#define JPEG_RAWY           0x014
#define JPEG_RAWC           0x018
#define JPEG_BS_ADDR        0x01c
#define JPEG_MARKERS        0x028
#define JPEG_LIMIT          0x02c
#define JPEG_BS1_ADDR       0x044
#define JPEG_BSFULL_EN      0x048
#define JPEG_BSFULL_SIZE    0x04c       /* KiB */
#define JPEG_BSFULL_STATE   0x050
#define  BSFULL_GO          1
#define JPEG_DECODE_MODE    0x054
#define JPEG_GLB_STAT       0x100
#define  STAT_FRM_END       BIT(31)
#define JPEG_GLB_BS_LEN     0x104
#define JPEG_INTR_STA       0x110
#define JPEG_INTR_CLR       0x114
#define  INTR_FRAME_END     BIT(0)
#define  INTR_BSLESS        BIT(1)
#define  INTR_BSFULL        BIT(4)
#define JPEG_BSBI_BS_LEN    0x154
#define JPEG_DEC_LEN        0x16c
#define JPEG_QT_BASE        0x300

#define IVDC_ADDR           BIT(31)     /* a source address over IVDC */

#define FRAME_MAX           16384       /* samples a side */
#define DEC_MCU_BYTES       256         /* 16 x 8 pixels of 4:2:2, packed */
#define DEC_SCAN_BYTES      700         /* the most a MCU's scan takes */

#define R(off)              (s->regs[(off) / 4])

static const uint8_t zigzag[64] = {
     0,  1,  8, 16,  9,  2,  3, 10, 17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
};

struct IngenicJpegJob {
    IngenicJpegState *s;        /* NULL once the device has reset */
    uint32_t w, h, bs, limit;
    uint16_t restart;           /* MCUs between RSTn, 0 for none */
    uint8_t *y, *uv;            /* the source, @w apart */
    uint8_t qt[2][64];          /* zigzag */
    uint32_t mul[2][64];        /* natural order */
    GByteArray *data;
    uint32_t sent, half;        /* the file out so far, the next half */
    bool decode;
    hwaddr dst;                 /* decoding: the lines' start */
    uint32_t stride, mcus, cols;
    uint8_t *scan, *pix;        /* decoding: the scan; the rows' bytes */
    size_t scan_len;
};

static void jpeg_update(IngenicJpegState *s)
{
    uint32_t en = extract32(R(JPEG_CTRL), 25, 5);

    qemu_set_irq(s->irq, !!(R(JPEG_INTR_STA) & en));
}

static void jpeg_job_free(IngenicJpegJob *j)
{
    g_free(j->y);
    g_free(j->uv);
    g_free(j->scan);
    g_free(j->pix);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}


/* Annex K's codes by their next 16 bits: the length << 8 | the symbol */
static uint16_t jpeg_lut[4][65536];

static void jpeg_lut_init(void)
{
    static gsize once;
    IngenicAvpuJpegHuff huff[4];

    if (g_once_init_enter(&once)) {
        ingenic_avpu_jpeg_annex_k(huff);
        for (int t = 0; t < 4; t++) {
            for (int sym = 0; sym < 256; sym++) {
                int len = huff[t].size[sym];

                if (len) {
                    uint32_t base = huff[t].code[sym] << (16 - len);

                    for (uint32_t i = 0; i < 1u << (16 - len); i++) {
                        jpeg_lut[t][base + i] = len << 8 | sym;
                    }
                }
            }
        }
        g_once_init_leave(&once, 1);
    }
}

typedef struct JpegBits {
    const uint8_t *p;
    size_t len, pos;
    uint32_t acc;
    int n;
    bool marker;                /* at a marker: zeros from here */
    int pad;                    /* the zero bytes given from there */
} JpegBits;

static void bits_fill(JpegBits *b)
{
    while (b->n <= 24) {
        uint8_t v = 0;
        bool data = false;

        if (!b->marker && b->pos < b->len) {
            v = b->p[b->pos];
            if (v == 0xff) {
                uint8_t next = b->pos + 1 < b->len ? b->p[b->pos + 1] : 0xd9;

                if (!next) {
                    b->pos += 2;
                    data = true;
                } else if (next == 0xff) {
                    b->pos++;
                    continue;
                } else {
                    b->marker = true;
                    v = 0;
                }
            } else {
                b->pos++;
                data = true;
            }
        }
        b->pad += !data;
        b->acc = b->acc << 8 | v;
        b->n += 8;
    }
}

static uint32_t bits_get(JpegBits *b, int n)
{
    uint32_t v;

    bits_fill(b);
    v = (b->acc >> (b->n - n)) & ((1u << n) - 1);
    b->n -= n;
    return v;
}

/* The scan goes on after an RSTn: the byte it stood in is done with */
static void bits_restart(JpegBits *b)
{
    b->acc = 0;
    b->n = 0;
    b->pad = 0;
    if (b->marker && b->pos + 1 < b->len && (b->p[b->pos + 1] & 0xf8) == 0xd0) {
        b->pos += 2;
        b->marker = false;
    }
}

/* The next symbol of table @table, or -1 where no code matches */
static int jpeg_symbol(JpegBits *b, int table)
{
    uint16_t e;

    bits_fill(b);
    e = jpeg_lut[table][(b->acc >> (b->n - 16)) & 0xffff];
    if (!e) {
        return -1;
    }
    b->n -= e >> 8;
    return e & 0xff;
}

static int jpeg_extend(JpegBits *b, int size)
{
    int v = size ? bits_get(b, size) : 0;

    return size && v < 1 << (size - 1) ? v - (1 << size) + 1 : v;
}

/*
 * A block's 64 coefficients as the stream gives them (zigzag order)
 * dequantized into @c (natural), or false if no code matches or the data
 * ran out before it was done
 */
static bool jpeg_block(JpegBits *b, int comp, const uint8_t *q, int *pred,
                       int32_t c[64])
{
    int size = jpeg_symbol(b, comp ? 2 : 0), k = 1;

    if (size < 0 || size > 11) {
        return false;
    }
    memset(c, 0, 64 * sizeof(*c));
    *pred += jpeg_extend(b, size);
    c[0] = *pred * q[0];
    while (k < 64) {
        int rs = jpeg_symbol(b, comp ? 3 : 1);

        if (rs < 0) {
            return false;
        }
        if (!(rs & 15)) {
            if (rs != 0xf0) {
                break;
            }
            k += 16;
            continue;
        }
        k += rs >> 4;
        if (k > 63) {
            return false;
        }
        c[zigzag[k]] = jpeg_extend(b, rs & 15) * q[k];
        k++;
    }
    return b->n >= 8 * b->pad;
}

#define FIX_0_298631336     2446
#define FIX_0_390180644     3196
#define FIX_0_541196100     4433
#define FIX_0_765366865     6270
#define FIX_0_899976223     7373
#define FIX_1_175875602     9633
#define FIX_1_501321110     12299
#define FIX_1_847759065     15137
#define FIX_1_961570560     16069
#define FIX_2_053119869     16819
#define FIX_2_562915447     20995
#define FIX_3_072711026     25172

/* One 1-D pass of the integer inverse DCT, @v apart, in place */
static void idct_pass(int32_t *d, int v, int shift)
{
    int32_t z2 = d[2 * v], z3 = d[6 * v];
    int32_t z1 = (z2 + z3) * FIX_0_541196100;
    int32_t t2 = z1 - z3 * FIX_1_847759065;
    int32_t t3 = z1 + z2 * FIX_0_765366865;
    int32_t t0 = (d[0] + d[4 * v]) * 8192, t1 = (d[0] - d[4 * v]) * 8192;
    int32_t t10 = t0 + t3, t13 = t0 - t3, t11 = t1 + t2, t12 = t1 - t2;
    int32_t a0 = d[7 * v], a1 = d[5 * v], a2 = d[3 * v], a3 = d[v];
    int32_t y1 = a0 + a3, y2 = a1 + a2, y3 = a0 + a2, y4 = a1 + a3;
    int32_t y5 = (y3 + y4) * FIX_1_175875602;
    int32_t r = 1 << (shift - 1);

    a0 *= FIX_0_298631336;
    a1 *= FIX_2_053119869;
    a2 *= FIX_3_072711026;
    a3 *= FIX_1_501321110;
    y1 *= -FIX_0_899976223;
    y2 *= -FIX_2_562915447;
    y3 = y3 * -FIX_1_961570560 + y5;
    y4 = y4 * -FIX_0_390180644 + y5;
    a0 += y1 + y3;
    a1 += y2 + y4;
    a2 += y2 + y3;
    a3 += y1 + y4;
    d[0] = (t10 + a3 + r) >> shift;
    d[7 * v] = (t10 - a3 + r) >> shift;
    d[v] = (t11 + a2 + r) >> shift;
    d[6 * v] = (t11 - a2 + r) >> shift;
    d[2 * v] = (t12 + a1 + r) >> shift;
    d[5 * v] = (t12 - a1 + r) >> shift;
    d[3 * v] = (t13 + a0 + r) >> shift;
    d[4 * v] = (t13 - a0 + r) >> shift;
}

/* The 8 x 8 block's samples, the IJG slow integer inverse DCT's */
static void jpeg_idct(int32_t c[64], uint8_t *out, int stride)
{
    for (int i = 0; i < 8; i++) {
        idct_pass(c + i, 8, 11);
    }
    for (int i = 0; i < 8; i++) {
        idct_pass(c + 8 * i, 1, 18);
        for (int x = 0; x < 8; x++) {
            out[i * stride + x] = MIN(MAX(c[8 * i + x] + 128, 0), 255);
        }
    }
}

/*
 * The scan's MCUs as packed rows, 2 * cols * 16 bytes a pixel row. A scan
 * that ends or breaks leaves the MCUs after it as they were (zero here).
 */
static void jpeg_decode(IngenicJpegJob *j)
{
    JpegBits b = { .p = j->scan, .len = j->scan_len };
    int pred[3] = { 0 }, ri = j->restart;
    size_t row = (size_t)j->cols * 32;

    for (uint32_t m = 0; m < j->mcus; m++) {
        uint32_t mx = m % j->cols, my = m / j->cols;
        uint8_t y[8][16], cb[8][8], cr[8][8];
        int32_t c[64];

        if (ri && m && m % ri == 0) {
            bits_restart(&b);
            memset(pred, 0, sizeof(pred));
        }
        if (!jpeg_block(&b, 0, j->qt[0], &pred[0], c)) {
            return;
        }
        jpeg_idct(c, &y[0][0], 16);
        if (!jpeg_block(&b, 0, j->qt[0], &pred[0], c)) {
            return;
        }
        jpeg_idct(c, &y[0][8], 16);
        if (!jpeg_block(&b, 1, j->qt[1], &pred[1], c)) {
            return;
        }
        jpeg_idct(c, &cb[0][0], 8);
        if (!jpeg_block(&b, 2, j->qt[1], &pred[2], c)) {
            return;
        }
        jpeg_idct(c, &cr[0][0], 8);
        for (int r = 0; r < 8; r++) {
            uint8_t *d = j->pix + (my * 8 + r) * row + mx * 32;

            for (int x = 0; x < 16; x++) {
                d[2 * x] = y[r][x];
                d[2 * x + 1] = x & 1 ? cr[r][x / 2] : cb[r][x / 2];
            }
        }
    }
}

static int jpeg_work(void *opaque)
{
    IngenicJpegJob *j = opaque;

    if (j->decode) {
        jpeg_decode(j);
        return 0;
    }
    ingenic_avpu_jpeg_encode_bare(j->y, j->w, j->uv, j->w, j->w, j->h,
                                  j->qt, j->mul, j->restart, j->data);
    return 0;
}

static bool jpeg_sections(IngenicJpegState *s)
{
    return R(JPEG_BSFULL_EN) && R(JPEG_BSFULL_SIZE) &&
           extract32(R(JPEG_CTRL), 25 + 4, 1);
}

/* The file out from where it stopped, in halves where sections are on */
static void jpeg_send(IngenicJpegState *s)
{
    IngenicJpegJob *j = s->job;
    uint32_t size = R(JPEG_BSFULL_SIZE) * KiB, n = j->data->len - j->sent;
    bool sections = jpeg_sections(s);
    hwaddr bs = j->bs;

    if (sections) {
        n = MIN(n, size);
        bs = j->half & 1 ? R(JPEG_BS1_ADDR) : j->bs;
    }
    address_space_write(&address_space_memory, bs, MEMTXATTRS_UNSPECIFIED,
                        j->data->data + j->sent, n);
    j->sent += n;
    j->half++;
    if (sections && j->sent < j->data->len) {
        R(JPEG_BSBI_BS_LEN) = j->sent;
        R(JPEG_INTR_STA) |= INTR_BSFULL;
        jpeg_update(s);
        return;
    }
    R(JPEG_GLB_BS_LEN) = j->sent;
    R(JPEG_GLB_STAT) |= STAT_FRM_END;
    R(JPEG_INTR_STA) |= INTR_FRAME_END;
    s->job = NULL;
    jpeg_update(s);
    jpeg_job_free(j);
}

/* The lines out, then 0x16c and the frame end */
static void jpeg_decode_end(IngenicJpegState *s, IngenicJpegJob *j)
{
    uint32_t lw = j->cols * 16, lines = DIV_ROUND_UP(j->mcus, j->cols) * 16;

    for (uint32_t l = 0; l < lines; l++) {
        address_space_write(&address_space_memory,
                            j->dst + (hwaddr)l * j->stride,
                            MEMTXATTRS_UNSPECIFIED, j->pix + (size_t)l * lw,
                            lw);
    }
    R(JPEG_DEC_LEN) = j->mcus * DEC_MCU_BYTES;
    R(JPEG_GLB_STAT) |= STAT_FRM_END;
    R(JPEG_INTR_STA) |= INTR_FRAME_END;
    s->job = NULL;
    jpeg_update(s);
    jpeg_job_free(j);
}

static void jpeg_done(void *opaque, int ret)
{
    IngenicJpegJob *j = opaque;
    IngenicJpegState *s = j->s;

    if (!s) {
        jpeg_job_free(j);
        return;
    }
    if (j->decode) {
        jpeg_decode_end(s, j);
        return;
    }
    if (j->limit && j->data->len > j->limit && !jpeg_sections(s)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-jpeg: %u bytes over the "
                      "%u byte limit\n", j->data->len, j->limit);
        R(JPEG_GLB_BS_LEN) = 0;
        R(JPEG_INTR_STA) |= INTR_BSLESS;
        s->job = NULL;
        jpeg_update(s);
        jpeg_job_free(j);
        return;
    }
    if (ingenic_enc_tap_on(&s->tap)) {
        IngenicEncTapAu au = {
            .name = "jpeg",
            .stream = j->bs,
            .codec = INGENIC_TAP_JPEG,
            .width = j->w,
            .height = j->h,
            .data = j->data->data,
            .len = j->data->len,
        };

        ingenic_enc_tap_au(&s->tap, &au);
    }
    if (ingenic_enc_capture_on(&s->capture)) {
        g_autoptr(GString) info = g_string_new("codec jpeg\n");
        g_autofree uint8_t *frame =
            ingenic_enc_capture_frame(&s->capture, j->y, j->w, j->uv, j->w,
                                      j->w, j->h);

        g_string_append_printf(info, "restart %u\n", j->restart);
        ingenic_enc_capture_regs(info, "reg", s->regs,
                                 INGENIC_JPEG_IOSIZE / 4);
        ingenic_enc_capture(&s->capture, "jpeg", "jpg", j->data->data,
                            j->data->len, frame, j->w, j->h, info);
    }
    jpeg_send(s);
}

/* The odd parts of the divisors whose reciprocal rounds up */
static const uint8_t jpeg_recip_up[] = {
    5, 17, 25, 27, 29, 37, 41, 49, 55, 57, 69, 71, 73, 75, 79, 83, 87, 95,
    99, 101, 113, 115, 119, 121, 123,
};

static uint32_t jpeg_recip(int q)
{
    int odd = q >> ctz32(q);

    for (int i = 0; i < ARRAY_SIZE(jpeg_recip_up); i++) {
        if (odd == jpeg_recip_up[i]) {
            return DIV_ROUND_UP(65536, q);
        }
    }
    return 65536 / q;
}

/* The base tables, by the index in CTRL (a T32NQ's, all 99) */
static void jpeg_tables(IngenicJpegState *s, IngenicJpegJob *j)
{
    int quality = 99 - (int)extract32(R(JPEG_CTRL), 18, 7);
    int scale = quality <= 2 ? 5000 :
                quality < 50 ? 5000 / quality : 200 - 2 * quality;

    for (int t = 0; t < 2; t++) {
        for (int i = 0; i < 64; i++) {
            uint32_t word = R(JPEG_QT_BASE + 64 * t + (i & ~3));
            int q = ((int)extract32(word, 8 * (i & 3), 7) * scale * 5 + 255)
                    >> 9;

            q = MIN(MAX(q, 1), 255);
            j->qt[t][i] = q;
            j->mul[t][zigzag[i]] = jpeg_recip(q);
        }
    }
}

/* The scan from @addr to its EOI, or @max bytes */
static uint8_t *jpeg_read_scan(hwaddr addr, size_t max, size_t *len)
{
    GByteArray *a = g_byte_array_new();
    size_t scanned = 0;

    while (a->len < max) {
        uint8_t chunk[4096];
        size_t n = MIN(sizeof(chunk), max - a->len);

        if (address_space_read(&address_space_memory, addr + a->len,
                               MEMTXATTRS_UNSPECIFIED, chunk, n)) {
            break;
        }
        g_byte_array_append(a, chunk, n);
        for (; scanned + 1 < a->len; scanned++) {
            if (a->data[scanned] == 0xff && a->data[scanned + 1] == 0xd9) {
                *len = scanned + 2;
                return g_byte_array_free(a, false);
            }
        }
    }
    *len = a->len;
    return g_byte_array_free(a, false);
}

static void jpeg_start_decode(IngenicJpegState *s)
{
    IngenicJpegJob *j = g_new0(IngenicJpegJob, 1);
    uint32_t width = R(JPEG_FRM_INFO) & 0xffff, mcus = R(JPEG_NMCU) + 1;
    uint32_t lw, lines;

    j->s = s;
    j->decode = true;
    j->cols = DIV_ROUND_UP(width, 16);
    lw = j->cols * 16;
    j->mcus = mcus;
    j->stride = R(JPEG_LINE_STRD);
    j->dst = R(JPEG_RAWY);
    if (!width || width > FRAME_MAX || !mcus || mcus > FRAME_MAX * 128 ||
        j->stride < lw) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-jpeg: decoding %u MCUs of a "
                      "%u wide picture into lines %u apart\n", mcus, width,
                      j->stride);
        R(JPEG_INTR_STA) |= INTR_BSLESS;
        jpeg_update(s);
        jpeg_job_free(j);
        return;
    }
    if (R(JPEG_DECODE_MODE)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-jpeg: decode mode %u as 0\n",
                      R(JPEG_DECODE_MODE));
    }
    if (R(JPEG_MARKERS) & 1) {
        j->restart = R(JPEG_MARKERS) >> 16;
    }
    for (int t = 0; t < 2; t++) {
        for (int i = 0; i < 64; i++) {
            j->qt[t][i] = extract32(R(JPEG_QT_BASE + 64 * t + (i & ~3)),
                                    8 * (i & 3), 8);
        }
    }
    jpeg_lut_init();
    j->scan = jpeg_read_scan(R(JPEG_BS_ADDR),
                             (size_t)mcus * DEC_SCAN_BYTES + 4096,
                             &j->scan_len);
    /* what the decode does not reach stays as it is in memory */
    lines = DIV_ROUND_UP(mcus, j->cols) * 16;
    j->pix = g_malloc((size_t)lines * lw);
    for (uint32_t l = 0; l < lines; l++) {
        address_space_read(&address_space_memory,
                           j->dst + (hwaddr)l * j->stride,
                           MEMTXATTRS_UNSPECIFIED, j->pix + (size_t)l * lw,
                           lw);
    }
    s->job = j;
    thread_pool_submit_aio(jpeg_work, j, jpeg_done, j);
}

static void jpeg_start(IngenicJpegState *s, bool decode)
{
    IngenicJpegJob *j;
    uint32_t info = R(JPEG_FRM_INFO), stride = R(JPEG_LINE_STRD);

    if (s->job) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-jpeg: started while busy\n");
        return;
    }
    if (decode) {
        jpeg_start_decode(s);
        return;
    }
    j = g_new0(IngenicJpegJob, 1);
    j->s = s;
    j->w = info & 0xffff;
    j->h = info >> 16;
    j->bs = R(JPEG_BS_ADDR);
    j->limit = R(JPEG_CTRL) & CTRL_LIMIT ? R(JPEG_LIMIT) : 0;
    if (R(JPEG_MARKERS) & 1) {
        j->restart = (R(JPEG_MARKERS) >> 16) + 1;
    }
    j->data = g_byte_array_new();
    if (!j->w || !j->h || j->w > FRAME_MAX || j->h > FRAME_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-jpeg: a %ux%u picture\n",
                      j->w, j->h);
        R(JPEG_INTR_STA) |= INTR_BSLESS;
        jpeg_update(s);
        jpeg_job_free(j);
        return;
    }
    s->job = j;
    jpeg_tables(s, j);
    stride = stride ?: j->w;
    j->y = g_malloc0((size_t)j->w * j->h);
    j->uv = g_malloc0((size_t)j->w * ((j->h + 1) / 2));
    if (R(JPEG_RAWY) & IVDC_ADDR) {
        uint32_t h2 = ROUND_UP(j->h, 2);
        g_autofree uint8_t *nv12 = g_malloc((size_t)j->w * h2 * 3 / 2);

        if (s->isp) {
            ingenic_isp_render_vpu(s->isp, 1, j->w, h2, nv12);
            memcpy(j->y, nv12, (size_t)j->w * j->h);
            memcpy(j->uv, nv12 + j->w * h2, (size_t)j->w * (h2 / 2));
        }
    } else {
        for (uint32_t r = 0; r < j->h; r++) {
            address_space_read(&address_space_memory,
                               R(JPEG_RAWY) + r * stride,
                               MEMTXATTRS_UNSPECIFIED, j->y + r * j->w, j->w);
        }
        for (uint32_t r = 0; r < (j->h + 1) / 2; r++) {
            address_space_read(&address_space_memory,
                               R(JPEG_RAWC) + r * stride,
                               MEMTXATTRS_UNSPECIFIED, j->uv + r * j->w,
                               j->w);
        }
    }
    thread_pool_submit_aio(jpeg_work, j, jpeg_done, j);
}

static void jpeg_reset(IngenicJpegState *s)
{
    if (s->job) {
        s->job->s = NULL;           /* its completion just frees it */
        if (s->job->sent) {         /* already done: it waits for a half */
            jpeg_job_free(s->job);
        }
        s->job = NULL;
    }
    R(JPEG_INTR_STA) = 0;
    R(JPEG_GLB_STAT) = 0;
    R(JPEG_GLB_BS_LEN) = 0;
    R(JPEG_DEC_LEN) = 0;
}

static uint64_t ingenic_jpeg_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicJpegState *s = INGENIC_JPEG(opaque);

    return R(addr);
}

static void ingenic_jpeg_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    IngenicJpegState *s = INGENIC_JPEG(opaque);
    uint32_t v = value;

    switch (addr) {
    case JPEG_TRIG:
        if (v & TRIG_RESET) {
            jpeg_reset(s);
            R(addr) = TRIG_RESET_END;
            jpeg_update(s);
            return;
        }
        R(addr) = v & ~TRIG_START;
        if (v & TRIG_START) {
            jpeg_start(s, v & TRIG_DECODE);
        }
        return;
    case JPEG_CTRL:
        R(addr) = v;
        jpeg_update(s);
        return;
    case JPEG_INTR_STA:
        return;
    case JPEG_INTR_CLR:
        R(JPEG_INTR_STA) &= ~v;
        jpeg_update(s);
        return;
    case JPEG_BSFULL_STATE:
        R(addr) = v;
        if (v == BSFULL_GO && s->job && s->job->sent) {
            jpeg_send(s);
        }
        return;
    }
    R(addr) = v;
}

static const MemoryRegionOps ingenic_jpeg_ops = {
    .read = ingenic_jpeg_read,
    .write = ingenic_jpeg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_jpeg_reset_hold(Object *obj, ResetType type)
{
    IngenicJpegState *s = INGENIC_JPEG(obj);

    jpeg_reset(s);
    memset(s->regs, 0, sizeof(s->regs));
}

static void ingenic_jpeg_reset_exit(Object *obj, ResetType type)
{
    jpeg_update(INGENIC_JPEG(obj));
}

static void ingenic_jpeg_init(Object *obj)
{
    IngenicJpegState *s = INGENIC_JPEG(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_jpeg_ops, s,
                          TYPE_INGENIC_JPEG, INGENIC_JPEG_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static bool jpeg_encoding(IngenicJpegState *s)
{
    return s->job && !s->job->sent;
}

/* A stop lets the encode in flight finish, into RAM not yet saved */
static void jpeg_vm_change(void *opaque, bool running, RunState state)
{
    IngenicJpegState *s = opaque;

    if (!running) {
        AIO_WAIT_WHILE(NULL, jpeg_encoding(s));
    }
}

static void ingenic_jpeg_realize(DeviceState *dev, Error **errp)
{
    qdev_add_vm_change_state_handler(dev, jpeg_vm_change, NULL, dev);
}

static int ingenic_jpeg_pre_save(void *opaque)
{
    if (jpeg_encoding(opaque)) {
        error_report("ingenic-jpeg: an encode is still in flight");
        return -EBUSY;
    }
    return 0;
}

/* The file of a job waiting for the guest to take its next half */
static int jpeg_job_put(QEMUFile *f, void *pv, size_t size,
                        const VMStateField *field, JSONWriter *vmdesc)
{
    IngenicJpegJob *j = *(IngenicJpegJob **)pv;

    qemu_put_byte(f, !!j);
    if (j) {
        qemu_put_be32(f, j->data->len);
        qemu_put_buffer(f, j->data->data, j->data->len);
        qemu_put_be32(f, j->sent);
        qemu_put_be32(f, j->half);
        qemu_put_be32(f, j->bs);
    }
    return 0;
}

static int jpeg_job_get(QEMUFile *f, void *pv, size_t size,
                        const VMStateField *field)
{
    IngenicJpegState *s = container_of(pv, IngenicJpegState, job);
    IngenicJpegJob *j;
    uint32_t len;

    if (!qemu_get_byte(f)) {
        return 0;
    }
    j = g_new0(IngenicJpegJob, 1);
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
    j->bs = qemu_get_be32(f);
    return j->sent > len ? -EINVAL : qemu_file_get_error(f);
}

static const VMStateInfo jpeg_job_info = {
    .name = "ingenic-jpeg-job",
    .get = jpeg_job_get,
    .put = jpeg_job_put,
};

static const VMStateDescription vmstate_ingenic_jpeg = {
    .name = TYPE_INGENIC_JPEG,
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = ingenic_jpeg_pre_save,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicJpegState, INGENIC_JPEG_IOSIZE / 4),
        VMSTATE_SINGLE(job, IngenicJpegState, 0, jpeg_job_info,
                       IngenicJpegJob *),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_jpeg_props[] = {
    DEFINE_INGENIC_ENC_CAPTURE_PROPERTY(IngenicJpegState, capture),
    DEFINE_INGENIC_ENC_TAP_PROPERTY(IngenicJpegState, tap),
    DEFINE_PROP_LINK("isp", IngenicJpegState, isp, TYPE_INGENIC_ISP,
                     IngenicIspState *),
};

static void ingenic_jpeg_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic JPEG engine";
    device_class_set_props(dc, ingenic_jpeg_props);
    dc->realize = ingenic_jpeg_realize;
    dc->vmsd = &vmstate_ingenic_jpeg;
    rc->phases.hold = ingenic_jpeg_reset_hold;
    rc->phases.exit = ingenic_jpeg_reset_exit;
}

static const TypeInfo ingenic_jpeg_types[] = {
    {
        .name = TYPE_INGENIC_JPEG,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicJpegState),
        .instance_init = ingenic_jpeg_init,
        .class_init = ingenic_jpeg_class_init,
    },
};

DEFINE_TYPES(ingenic_jpeg_types)
