/*
 * QTest testcase for the Ingenic AVPU's H.265 transform skip and H.264
 * motion
 *
 * One intra command list entry, H.265, a 32x8 picture of alternating rows:
 * 8x8 coding blocks whose 4x4 chroma blocks have levels, through the T31's
 * EL150 and the T41's EL200. transform_skip_enabled_flag comes with the
 * entry (EL150 word 2 bit 31, EL200 word 3 bit 31); without it the slice
 * data must not carry transform_skip_flag. The two cores round the levels
 * of an I picture their own way, so each layout's slice data is pinned. The
 * 64 KiB of registers answers at every 64 KiB of the window a part's vendor
 * kernels give it: a T41NQ's 1 MiB and a T40NN's 2 MiB, which holds its
 * RADIX and RTC addresses.
 *
 * The EL150's H.264 P pictures search for motion: a picture of texture and
 * the same shifted by whole samples, which the motion vector buffer (entry
 * word 0x2e) gives back, in the layout measured on a T31.
 *
 * The EL200's I pictures at a fixed QP (auto QP off): pictures made here from
 * integers, the bytes of the model within 8% of those a T41NQ writes for the
 * same pictures.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define W               32
#define H               8
#define QP              27

#define SRC_Y           0x00800000
#define SRC_C           0x00810000
#define LIST            0x00820000
#define STATUS          0x00830000
#define BS              0x00840000
#define ROOM            0x10000

#define MW              128
#define MH              64
#define MV              0x00850000
#define EP2             0x00860000

#define CL_LAST         (1u << 31)
#define TRANSFORM_SKIP  (1u << 31)
#define CABAC           (1u << 10)

typedef struct {
    const char *machine;
    uint32_t base;          /* the AVPU's registers */
    uint32_t cl_addr, start, irq, status;
    bool el200;
    uint32_t crc_on, crc_off;   /* the slice data, 23 and 22 bytes */
} Avpu;

static const Avpu el150 = {
    .machine = "ingenic-t31",
    .base = 0x13200000,
    .cl_addr = 0x83e0,
    .start = 0x83e4,
    .irq = 0x8018,
    .crc_on = 0x4e0670ba,
    .crc_off = 0x87da9da8,
};

static const Avpu el150_t40 = {
    .machine = "ingenic-t40",
    .base = 0x13200000,
    .cl_addr = 0x83e0,
    .start = 0x83e4,
    .irq = 0x8018,
};

static const Avpu el200 = {
    .machine = "ingenic-t41",
    .base = 0x13100000,
    .cl_addr = 0x1000,
    .start = 0x1018,
    .irq = 0x18,
    .status = 0x1008,
    .el200 = true,
    .crc_on = 0x9430ff20,
    .crc_off = 0xd1edd7c3,
};

static uint32_t crc32_le(const uint8_t *p, size_t n)
{
    uint32_t crc = ~0u;

    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = crc >> 1 ^ (crc & 1 ? 0xedb88320 : 0);
        }
    }
    return ~crc;
}

/* Encode the picture with or without transform skip; the slice data's length */
static uint32_t encode(const Avpu *a, bool skip, uint8_t *out)
{
    QTestState *qts = qtest_initf("-machine %s", a->machine);
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    uint8_t y[W * H], c[W * H / 2];
    uint32_t len;

    /* rows of 180 and 84, chroma 128 and 184: levels in every block */
    for (int i = 0; i < W * H; i++) {
        y[i] = (i / W) & 1 ? 84 : 180;
    }
    for (int i = 0; i < W * H / 2; i++) {
        c[i] = (i / W) & 1 ? 184 : 128;
    }
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));

    if (a->el200) {
        w[0] = CL_LAST | 1;                         /* H.265 */
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 16;
        w[3] = CABAC | (skip ? TRANSFORM_SKIP : 0);
        w[24] = 2u << 28 | QP << 16;                /* intra */
        w[152] = 2;                                 /* auto QP */
        w[40] = SRC_Y;
        w[42] = SRC_C;
        w[52] = W;
        w[136] = BS;
        w[141] = ROOM;
    } else {
        w[0] = CL_LAST | 1u << 28;                  /* H.265 */
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
        w[2] = CABAC | (skip ? TRANSFORM_SKIP : 0);
        w[3] = 2u << 28 | QP << 16;                 /* intra */
        w[0x20] = SRC_Y;
        w[0x21] = SRC_C;
        w[0x22] = W;
        w[0x30] = BS;
        w[0x33] = ROOM;
    }
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    if (a->status) {
        qtest_writel(qts, a->base + a->status, STATUS);
    }
    qtest_writel(qts, a->base + a->cl_addr, LIST);
    qtest_writel(qts, a->base + a->start, 2);
    for (int i = 0; i < 20000 && !(qtest_readl(qts, a->base + a->irq) & 1);
         i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, a->base + a->irq) & 1, ==, 1);
    /* status word 0x41, the bytes written; the EL200's first word */
    len = qtest_readl(qts, a->status ? STATUS : LIST + 0x41 * 4);
    g_assert_cmpuint(len, >, 0);
    g_assert_cmpuint(len, <, ROOM);
    qtest_memread(qts, BS, out, len);
    qtest_quit(qts);
    return len;
}

static void test_transform_skip(const void *data)
{
    const Avpu *a = data;
    g_autofree uint8_t *on = g_malloc(ROOM), *off = g_malloc(ROOM);
    uint32_t n_on = encode(a, true, on), n_off = encode(a, false, off);

    /* the flag's bins go, and the rest stays as coded */
    g_assert_cmpuint(n_on, ==, 23);
    g_assert_cmphex(crc32_le(on, n_on), ==, a->crc_on);
    g_assert_cmpuint(n_off, ==, 22);
    g_assert_cmphex(crc32_le(off, n_off), ==, a->crc_off);
}


/* Smoothed noise, larger than a picture by a margin to shift in */
#define MARGIN          16
#define TW              (MW + 2 * MARGIN)
#define TH              (MH + 2 * MARGIN)

static void make_texture(uint8_t *t)
{
    static int a[TW * TH], b[TW * TH];
    uint32_t s = 0x2545f491;

    for (int i = 0; i < TW * TH; i++) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        a[i] = s & 255;
    }
    for (int pass = 0; pass < 2; pass++) {
        for (int y = 0; y < TH; y++) {
            for (int x = 0; x < TW; x++) {
                int sum = 0;

                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int yy = MIN(MAX(y + dy, 0), TH - 1);
                        int xx = MIN(MAX(x + dx, 0), TW - 1);

                        sum += a[yy * TW + xx];
                    }
                }
                b[y * TW + x] = sum / 9;
            }
        }
        memcpy(a, b, sizeof(a));
    }
    for (int i = 0; i < TW * TH; i++) {
        t[i] = MIN(MAX(128 + (a[i] - 128) * 6, 16), 235);
    }
}

/*
 * One H.264 picture of the texture shifted by (dx, dy): the picture of a
 * channel (EP2 keeps its reference), intra or P, CABAC, QP 27 with the
 * default lambdas. Returns status word 0x47 (skipped 8x8 blocks) and fills
 * @mv with the motion vector buffer.
 */
static uint32_t encode_h264(QTestState *qts, const uint8_t *tex, int dx,
                            int dy, bool intra, uint32_t *mv)
{
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    uint8_t y[MW * MH], c[MW * MH / 2];
    uint32_t skip;

    for (int r = 0; r < MH; r++) {
        for (int x = 0; x < MW; x++) {
            y[r * MW + x] = tex[(r + MARGIN + dy) * TW + x + MARGIN + dx];
        }
    }
    memset(c, 128, sizeof(c));
    qtest_memwrite(qts, SRC_Y, y, sizeof(y));
    qtest_memwrite(qts, SRC_C, c, sizeof(c));
    w[0] = CL_LAST;
    w[1] = (MW / 8 - 1) | (MH / 8 - 1) << 12;
    w[2] = CABAC;
    w[3] = (intra ? 2u : 1u) << 28 | QP << 16;
    w[0x20] = SRC_Y;
    w[0x21] = SRC_C;
    w[0x22] = MW;
    w[0x23] = EP2;
    w[0x2e] = MV;
    w[0x30] = BS;
    w[0x33] = ROOM;
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    qtest_writel(qts, el150.base + el150.cl_addr, LIST);
    qtest_writel(qts, el150.base + el150.start, 2);
    for (int i = 0; i < 20000 &&
         !(qtest_readl(qts, el150.base + el150.irq) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, el150.base + el150.irq) & 1, ==, 1);
    qtest_writel(qts, el150.base + el150.irq, 1);
    skip = qtest_readl(qts, LIST + 0x47 * 4);
    qtest_memread(qts, MV, mv, MW / 16 * MH / 16 * 32);
    for (int i = 0; i < MW / 16 * MH / 16 * 8; i++) {
        mv[i] = le32_to_cpu(mv[i]);
    }
    return skip;
}

/* A vector as the buffer has it: x in 14 bits, y in 12 above, in quarters */
static uint32_t mv_word(int x, int y)
{
    return (x & 0x3fff) | (y & 0xfff) << 14;
}

static void test_motion(const void *data)
{
    bool search = data != NULL;
    QTestState *qts = qtest_initf("-machine %s %s", el150.machine,
                                  search ? "-global "
                                  "ingenic-avpu.h264-motion=search" : "");
    static uint8_t tex[TW * TH];
    uint32_t mv[MW / 16 * MH / 16 * 8];
    int n = MW / 16 * MH / 16;

    make_texture(tex);

    /* an intra picture has no vectors, and its buffer says so */
    encode_h264(qts, tex, 0, 0, true, mv);
    for (int i = 0; i < n * 8; i++) {
        g_assert_cmphex(mv[i], ==, 0xfc000000);
    }

    /*
     * the texture, moved 3 samples one way and 1 the other: with the
     * search the vector points back to where each block was; without it
     * none has a vector but 0, or is intra
     */
    encode_h264(qts, tex, 3, -1, false, mv);
    for (int mb = 0; !search && mb < n; mb++) {
        for (int q = 0; q < 4; q++) {
            g_assert_true(mv[mb * 8 + q * 2] == 0 ||
                          mv[mb * 8 + q * 2] == 0xfc000000);
        }
    }
    if (!search) {
        qtest_quit(qts);
        return;
    }
    for (int mb = 0; mb < n; mb++) {
        for (int q = 0; q < 4; q++) {
            g_assert_cmphex(mv[mb * 8 + q * 2], ==, mv_word(12, -4));
            g_assert_cmphex(mv[mb * 8 + q * 2 + 1], ==, 0x7c000000);
        }
    }

    /* back to where it began: the same vector the other way */
    encode_h264(qts, tex, 0, 0, false, mv);
    for (int mb = 0; mb < n; mb++) {
        g_assert_cmphex(mv[mb * 8], ==, mv_word(-12, 4));
    }

    /* and still: every macroblock skipped, at no motion */
    g_assert_cmpuint(encode_h264(qts, tex, 0, 0, false, mv), ==, n * 4);
    for (int mb = 0; mb < n; mb++) {
        g_assert_cmphex(mv[mb * 8], ==, mv_word(0, 0));
    }
    qtest_quit(qts);
}


/*
 * While a core codes, its zone shows the command list's first entry, as a
 * T40XP's and a T41LQ's AVPU do under a running encoder (their words 0 to
 * 0x1dc of the zone, and 0x40 up on the EL200, from the entry's first); idle
 * it reads 0. The job lasts a few hundred microseconds, so a run of jobs
 * has to catch one in flight.
 */
static void test_zone(const void *data)
{
    const Avpu *a = data;
    QTestState *qts = qtest_initf("-machine %s", a->machine);
    uint32_t w[0x1000 / 4] = {};
    uint32_t zone = a->base + (a->el200 ? 0x1000 + 0x40 : 0x8200);
    bool caught = false;

    qtest_memset(qts, SRC_Y, 0x80, W * H);
    qtest_memset(qts, SRC_C, 0x80, W * H / 2);
    if (a->el200) {
        w[0] = CL_LAST | 1;
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 16;
        w[3] = CABAC;
        w[24] = 2u << 28 | QP << 16;
        w[152] = 2;
        w[40] = SRC_Y;
        w[42] = SRC_C;
        w[52] = W;
        w[136] = BS;
        w[141] = ROOM;
    } else {
        w[0] = CL_LAST | 1u << 28;
        w[1] = (W / 8 - 1) | (H / 8 - 1) << 12;
        w[2] = CABAC;
        w[3] = 2u << 28 | QP << 16;
        w[0x20] = SRC_Y;
        w[0x21] = SRC_C;
        w[0x22] = W;
        w[0x30] = BS;
        w[0x33] = ROOM;
    }
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    if (a->status) {
        qtest_writel(qts, a->base + a->status, STATUS);
    }
    qtest_writel(qts, a->base + a->cl_addr, LIST);
    g_assert_cmphex(qtest_readl(qts, zone), ==, 0);
    for (int i = 0; i < 2000 && !caught; i++) {
        qtest_writel(qts, a->base + a->start, 2);
        caught = qtest_readl(qts, zone) == le32_to_cpu(w[0]) &&
                 qtest_readl(qts, zone + 4) == le32_to_cpu(w[1]);
        while (!(qtest_readl(qts, a->base + a->irq) & 1)) {
            g_usleep(100);
        }
        qtest_writel(qts, a->base + a->irq, 1);
    }
    g_assert_true(caught);
    g_assert_cmphex(qtest_readl(qts, zone), ==, 0);
    qtest_quit(qts);
}

typedef struct {
    const char *machine;
    uint32_t first, last;   /* the windows' first and last 64 KiB */
    uint32_t id, hw_id;     /* where the ID words sit, and the second */
    uint32_t int_mask;
    uint32_t axi;           /* "AXI0", then its version */
    uint32_t axi_ver;
} Mirror;

static const Mirror mirror_t40 = {
    "ingenic-t40", 0x13100000, 0x132f0000, 0x8000, 0x72000760, 0x8014,
    0x8100, 1
};

static const Mirror mirror_t41 = {
    "ingenic-t41", 0x13100000, 0x131f0000, 0x0, 0x71000260, 0x14,
    0x800, 3
};

static void test_mirror(const void *data)
{
    const Mirror *m = data;
    QTestState *qts = qtest_initf("-machine %s", m->machine);

    for (uint32_t at = m->first; at <= m->last; at += 0x10000) {
        g_test_message("0x%08x", at);
        g_assert_cmphex(qtest_readl(qts, at + m->id), ==, 0x30ab6e51);
        g_assert_cmphex(qtest_readl(qts, at + m->id + 4), ==, m->hw_id);
        g_assert_cmphex(qtest_readl(qts, at + m->axi), ==, 0x41584930);
        g_assert_cmphex(qtest_readl(qts, at + m->axi + 0xc), ==, m->axi_ver);
    }
    /* A write through one lands in all */
    qtest_writel(qts, m->first + 0x30000 + m->int_mask, 0x5a5a5a5a);
    for (uint32_t at = m->first; at <= m->last; at += 0x10000) {
        g_assert_cmphex(qtest_readl(qts, at + m->int_mask), ==, 0x5a5a5a5a);
    }
    qtest_quit(qts);
}

/*
 * The EL200's I pictures at a fixed QP against the camera's: pictures made
 * here from integers (a texture of smoothed noise, squares with bars and a
 * disc), 640x352, injected into a T41NQ with auto QP off. The camera's bytes
 * are the size of the slice data of the IDR. The model rounds the levels as
 * the camera does, a third of a step at every QP and transform size
 * (MEASURED against HM's decode of the camera's blocks).
 */
#define PW              640
#define PH              352
#define PSRC_Y          0x00900000
#define PSRC_C          0x00940000
#define PBS             0x00a00000
#define PROOM           0x40000

typedef enum { PIC_TEX, PIC_SQUARES } PicKind;

static uint32_t xorshift(uint32_t *s)
{
    *s ^= *s << 13;
    *s ^= *s >> 17;
    *s ^= *s << 5;
    return *s;
}

static void make_picture(PicKind kind, uint8_t *y, uint8_t *c)
{
    if (kind == PIC_TEX) {
        static int a[PW * PH], b[PW * PH];
        uint32_t s = 0x2545f491;

        for (int i = 0; i < PW * PH; i++) {
            a[i] = xorshift(&s) & 255;
        }
        for (int pass = 0; pass < 2; pass++) {
            for (int r = 0; r < PH; r++) {
                for (int x = 0; x < PW; x++) {
                    int sum = 0;

                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            sum += a[MIN(MAX(r + dy, 0), PH - 1) * PW +
                                     MIN(MAX(x + dx, 0), PW - 1)];
                        }
                    }
                    b[r * PW + x] = sum / 9;
                }
            }
            memcpy(a, b, sizeof(a));
        }
        for (int i = 0; i < PW * PH; i++) {
            y[i] = MIN(MAX(128 + (a[i] - 128) * 6, 16), 235);
        }
    } else {
        for (int r = 0; r < PH; r++) {
            for (int x = 0; x < PW; x++) {
                int v = 90 + (x + r) / 10 + 40 * ((x / 48 + r / 48) & 1) +
                        50 * ((x - 420) * (x - 420) +
                              (r - 220) * (r - 220) < 3600);

                if (r > 100 && r < 160 && (x / 5 & 1)) {
                    v += 30;
                }
                y[r * PW + x] = MIN(MAX(v, 16), 235);
            }
        }
    }
    for (int r = 0; r < PH / 2; r++) {
        for (int x = 0; x < PW / 2; x++) {
            c[r * PW + 2 * x] = MIN(MAX(100 + x / 8, 16), 240);
            c[r * PW + 2 * x + 1] = MIN(MAX(150 - r / 6, 16), 240);
        }
    }
}

static uint32_t encode_picture(PicKind kind, int qp)
{
    QTestState *qts = qtest_initf("-machine %s", el200.machine);
    g_autofree uint32_t *w = g_new0(uint32_t, 0x1000 / 4);
    g_autofree uint8_t *y = g_malloc(PW * PH), *c = g_malloc(PW * PH / 2);
    uint32_t len;

    make_picture(kind, y, c);
    qtest_memwrite(qts, PSRC_Y, y, PW * PH);
    qtest_memwrite(qts, PSRC_C, c, PW * PH / 2);
    w[0] = CL_LAST | 1;
    w[1] = (PW / 8 - 1) | (PH / 8 - 1) << 16;
    w[3] = CABAC;
    w[24] = 2u << 28 | qp << 16;
    w[40] = PSRC_Y;
    w[42] = PSRC_C;
    w[52] = PW;
    w[136] = PBS;
    w[141] = PROOM;
    w[152] = 0xf4;                  /* scaling lists on, auto QP off */
    for (int i = 0; i < 0x1000 / 4; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    qtest_memwrite(qts, LIST, w, 0x1000);
    qtest_writel(qts, el200.base + el200.status, STATUS);
    qtest_writel(qts, el200.base + el200.cl_addr, LIST);
    qtest_writel(qts, el200.base + el200.start, 2);
    for (int i = 0; i < 40000 &&
         !(qtest_readl(qts, el200.base + el200.irq) & 1); i++) {
        g_usleep(500);
    }
    g_assert_cmphex(qtest_readl(qts, el200.base + el200.irq) & 1, ==, 1);
    len = qtest_readl(qts, STATUS);
    qtest_quit(qts);
    return len;
}

typedef struct {
    const char *name;
    PicKind kind;
    int qp;
    uint32_t camera;            /* bytes of the IDR's slice data */
} PictureCase;

static const PictureCase picture_cases[] = {
    { "squares-24", PIC_SQUARES, 24, 3536 },
    { "squares-28", PIC_SQUARES, 28, 2807 },
    { "squares-32", PIC_SQUARES, 32, 2243 },
    { "squares-36", PIC_SQUARES, 36, 1630 },
    { "squares-40", PIC_SQUARES, 40, 1206 },
    { "texture-24", PIC_TEX, 24, 113788 },
    { "texture-28", PIC_TEX, 28, 92713 },
    { "texture-32", PIC_TEX, 32, 72473 },
};

static void test_picture_sizes(const void *data)
{
    const PictureCase *p = data;
    uint32_t len = encode_picture(p->kind, p->qp);

    g_test_message("%s: model %u camera %u", p->name, len, p->camera);
    g_assert_cmpuint(len * 100, >=, p->camera * (100 - 8));
    g_assert_cmpuint(len * 100, <=, p->camera * (100 + 8));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_data_func("ingenic/avpu/el150/transform-skip", &el150,
                        test_transform_skip);
    qtest_add_data_func("ingenic/avpu/el200/transform-skip", &el200,
                        test_transform_skip);
    qtest_add_data_func("ingenic/avpu/el150/h264-stand-in", NULL, test_motion);
    qtest_add_data_func("ingenic/avpu/el150/h264-motion", &el150,
                        test_motion);
    for (int i = 0; i < ARRAY_SIZE(picture_cases); i++) {
        g_autofree char *path = g_strdup_printf(
            "ingenic/avpu/el200/i-size/%s", picture_cases[i].name);

        qtest_add_data_func(path, &picture_cases[i], test_picture_sizes);
    }
    qtest_add_data_func("ingenic/avpu/zone/t40", &el150_t40, test_zone);
    qtest_add_data_func("ingenic/avpu/zone/t41", &el200, test_zone);
    qtest_add_data_func("ingenic/avpu/mirror/t40", &mirror_t40, test_mirror);
    qtest_add_data_func("ingenic/avpu/mirror/t41", &mirror_t41, test_mirror);
    return g_test_run();
}
