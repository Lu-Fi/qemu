/*
 * Ingenic IPU (ipu_v13, on the T10 to T32 and the T40): OSD blending
 *
 * libimp draws its OSD regions onto each frame before encoding with the
 * IPU, one job per region: the frame is the NV12 background (IPU_Y_ADDR,
 * IPU_U_ADDR, IPU_IN_FM_GS), usually also the output (IPU_OUT_ADDR,
 * IPU_NV_OUT_ADDR), and up to four OSD channels, each a picture at a
 * position (IPU_OSD_CHn_POS) blended in by its IPU_OSD_CHn_PARA: bit 0
 * enables it, bit 1 takes the global alpha (bits 10:3) instead of the
 * pixels', bit 2 multiplies the two ((a * g) >> 8), bits 13:11 give the
 * picture type, bits 17:14 the order of a pixel's bytes, bit 18 says the
 * colours are premultiplied, and bit 25 takes them as RGB, to YUV through
 * the channel's coefficients (C0..C4, two to a register in bits 11:0 and
 * 31:20, the offsets Y in bits 15:0 and UV in bits 31:16 of
 * IPU_CHn_CSC_OFSET_PARA, out of 1024); without it they are Y, U and V.
 * The types are ARGB (4 bytes a pixel), NV12 and NV21 (no alpha: seen
 * only through the global alpha) and 16-bit (type 4: three 5-bit fields,
 * low first, in the order's colour bytes, alpha bit 15); RGB (type 1) and
 * types 5 to 7 draw nothing, nor does a PARA with bit 20 or 23 set.
 * A pixel blends as (f * a + b * (255 - a) + 128) >> 8, alpha 0 and 255
 * exact, premultiplied as f + that of black; chroma blends each pixel's U
 * and V, then averages the 2x2 block as (sum + 2) >> 2.
 *
 * A job with no OSD channel enabled is the CSC command instead: the frame
 * converted to the output IPU_D_FMT names, NV21, 32-bit RGB in one of four
 * orders, or HSV, through channel 0's coefficients. An OSD job blends onto
 * NV12 whatever IPU_D_FMT holds (older libimp leaves it 0).
 *
 * Writing IPU_RUN to IPU_TRIGGER runs the job; IPU_STATUS bit 0 (OUT_END)
 * then reads set, and the interrupt is up while it is and IPU_GLB_CTRL
 * enables it. The driver clears OUT_END itself. All as a T31X does at
 * U-Boot, byte for byte; what PARA bits 21, 22 and 24 do is not modelled.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-ipu.h"

#define IPU_FM_CTRL         0x000
#define IPU_STATUS          0x004
#define IPU_D_FMT           0x008
#define IPU_Y_ADDR          0x00c
#define IPU_U_ADDR          0x010
#define IPU_IN_FM_GS        0x018
#define IPU_Y_STRIDE        0x01c
#define IPU_UV_STRIDE       0x020
#define IPU_OUT_ADDR        0x024
#define IPU_OUT_STRIDE      0x02c
#define IPU_OUT_V_ADDR      0x030
#define IPU_OUT_V_STRIDE    0x034
#define IPU_TRIGGER         0x074
#define IPU_GLB_CTRL        0x07c
#define IPU_NV_OUT_ADDR     0x0b4
#define IPU_NV_OUT_STRIDE   0x0b8
#define IPU_OSD_IN_Y_ADDR(n)    (0x0bc + (n) * 0x10)
#define IPU_OSD_IN_Y_STRIDE(n)  (0x0c0 + (n) * 0x10)
#define IPU_OSD_IN_UV_ADDR(n)   (0x0c4 + (n) * 0x10)
#define IPU_OSD_IN_UV_STRIDE(n) (0x0c8 + (n) * 0x10)
#define IPU_OSD_GS(n)       (0x0fc + (n) * 4)
#define IPU_OSD_POS(n)      (0x10c + (n) * 4)
#define IPU_OSD_PARA(n)     (0x11c + (n) * 4)
#define IPU_CSC(n, i)       (0x148 + (n) * 0x18 + (i) * 4)  /* i = 5: offsets */

#define STATUS_OUT_END      BIT(0)
#define STATUS_IDLE         0x80010000  /* what a T31 reads besides OUT_END */
#define TRIGGER_RUN         BIT(0)
#define TRIGGER_STOP        BIT(1)
#define TRIGGER_RESET       BIT(3)
#define GLB_IRQ_EN          BIT(0)

#define PARA_EN             BIT(0)
#define PARA_GLOBAL_ALPHA   BIT(1)
#define PARA_ALPHA_X_GLOBAL BIT(2)
#define PARA_PREMULTIPLIED  BIT(18)
#define PARA_HIDDEN         (BIT(20) | BIT(23))
#define PARA_CSC            BIT(25)

/* the output formats of D_FMT bits 6:0; NV12 is the OSD blend */
#define DFMT_ARGB           0x00
#define DFMT_NV21           0x03
#define DFMT_HSV            0x04
#define DFMT_ABGR           0x28
#define DFMT_RGBA           0x40
#define DFMT_BGRA           0x68

enum { PIC_ARGB, PIC_RGB, PIC_NV12, PIC_NV21, PIC_1555 };

#define R(off)              (s->regs[(off) / 4])

static void ipu_update(IngenicIpuState *s)
{
    qemu_set_irq(s->irq, (R(IPU_STATUS) & STATUS_OUT_END) &&
                         (R(IPU_GLB_CTRL) & GLB_IRQ_EN));
}

static void dma_read(hwaddr addr, void *buf, size_t len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
}

static void dma_write(hwaddr addr, const void *buf, size_t len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

/* Where R, G, B and A sit in a pixel, for the byte orders the PARA names */
static bool ipu_argb_order(unsigned type, int pos[4])
{
    static const char *const orders[16] = {
        [0x0] = "ARGB", [0x1] = "ARBG", [0x2] = "AGRB", [0x3] = "AGBR",
        [0x4] = "ABRG", [0x5] = "ABGR", [0x8] = "RGBA", [0x9] = "RBGA",
        [0xa] = "GRBA", [0xb] = "GBRA", [0xc] = "BRGA", [0xd] = "BGRA",
    };
    const char *o = orders[type & 15];

    if (!o) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        pos[strchr("RGBA", o[i]) - "RGBA"] = i;
    }
    return true;
}

static int clip8(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* @fg over @bg at alpha @a: the ends exact, the rest a shift */
static int blend(int bg, int fg, int a)
{
    if (a == 0) {
        return bg;
    }
    if (a == 255) {
        return fg;
    }
    return (fg * a + bg * (255 - a) + 128) >> 8;
}

static int mix(int bg, int fg, int a, bool premultiplied)
{
    return premultiplied ? clip8(fg + blend(bg, 0, a)) : blend(bg, fg, a);
}

/* A channel's coefficients C0..C4, two to a register, then the offsets */
typedef struct {
    int m[9];
    int yoff, uvoff;
} IpuCsc;

static void ipu_csc(IngenicIpuState *s, int n, IpuCsc *k)
{
    const uint32_t *c = &R(IPU_CSC(n, 0));

    for (int i = 0; i < 9; i++) {
        k->m[i] = extract32(c[i / 2], i & 1 ? 20 : 0, 12);
    }
    k->yoff = extract32(c[5], 0, 16);
    k->uvoff = extract32(c[5], 16, 16);
}

/* Blend OSD channel @n into the output frame (@w x @h), in place */
static void ipu_osd(IngenicIpuState *s, int n, uint32_t w, uint32_t h)
{
    uint32_t para = R(IPU_OSD_PARA(n));
    unsigned type = extract32(para, 11, 3);
    int galpha = extract32(para, 3, 8);
    uint32_t gs = R(IPU_OSD_GS(n)), pos = R(IPU_OSD_POS(n));
    uint32_t ow = gs >> 16, oh = gs & 0xffff;
    uint32_t ox = pos >> 16, oy = pos & 0xffff;
    uint32_t stride = R(IPU_OSD_IN_Y_STRIDE(n));
    uint32_t outs = R(IPU_OUT_STRIDE) ? R(IPU_OUT_STRIDE) : w;
    uint32_t nvs = R(IPU_NV_OUT_STRIDE) ? R(IPU_NV_OUT_STRIDE) : w;
    bool prem = para & PARA_PREMULTIPLIED;
    g_autofree uint8_t *pic = NULL, *picuv = NULL;
    g_autofree uint8_t *py = NULL, *pu = NULL, *pv = NULL, *pa = NULL;
    g_autofree uint8_t *line = NULL;
    uint32_t x0, x1;
    int bpp = 1, at[4];
    IpuCsc k;

    if (type == PIC_RGB || type > PIC_1555 || (para & PARA_HIDDEN)) {
        return;
    }
    if (type == PIC_ARGB || type == PIC_1555) {
        bpp = type == PIC_ARGB ? 4 : 2;
        ow /= bpp;                  /* the size is in bytes across */
        if (!ipu_argb_order(extract32(para, 14, 4), at)) {
            qemu_log_mask(LOG_UNIMP, "ingenic-ipu: OSD byte order %u\n",
                          extract32(para, 14, 4));
            return;
        }
    }
    ow = MIN(ow, ox < w ? w - ox : 0);
    oh = MIN(oh, oy < h ? h - oy : 0);
    if (!ow || !oh) {
        return;
    }
    ipu_csc(s, n, &k);

    /* The picture as YUV and alpha, a sample of each per pixel */
    py = g_malloc(ow * oh);
    pu = g_malloc(ow * oh);
    pv = g_malloc(ow * oh);
    pa = g_malloc(ow * oh);
    /* row by row, each row as wide as the picture whatever the stride */
    pic = g_malloc((size_t)ow * bpp * oh);
    for (uint32_t j = 0; j < oh; j++) {
        dma_read(R(IPU_OSD_IN_Y_ADDR(n)) + (hwaddr)j * stride,
                 pic + (size_t)j * ow * bpp, (size_t)ow * bpp);
    }
    stride = ow * bpp;
    if (type == PIC_NV12 || type == PIC_NV21) {
        uint32_t uvstride = R(IPU_OSD_IN_UV_STRIDE(n));
        uint32_t uvw = (ow + 1) & ~1u;

        picuv = g_malloc((size_t)uvw * ((oh + 1) / 2));
        for (uint32_t j = 0; j < (oh + 1) / 2; j++) {
            dma_read(R(IPU_OSD_IN_UV_ADDR(n)) + (hwaddr)j * uvstride,
                     picuv + (size_t)j * uvw, uvw);
        }
        for (uint32_t j = 0; j < oh; j++) {
            for (uint32_t i = 0; i < ow; i++) {
                const uint8_t *c2 = picuv + (j / 2) * uvw + (i & ~1u);
                int idx = j * ow + i;

                py[idx] = pic[j * stride + i];
                pu[idx] = c2[type == PIC_NV21];
                pv[idx] = c2[type != PIC_NV21];
                pa[idx] = 0;        /* shown only through the global alpha */
            }
        }
    } else {
        for (uint32_t j = 0; j < oh; j++) {
            for (uint32_t i = 0; i < ow; i++) {
                const uint8_t *q = pic + j * stride + i * bpp;
                uint8_t px[4];
                int r, g, b, idx = j * ow + i;

                if (type == PIC_1555) {
                    /* the 5-bit fields, low first, fill the colour bytes */
                    uint16_t v = lduw_le_p(q);
                    int f = 0;

                    for (int c = 0; c < 4; c++) {
                        int e = extract32(v, 5 * f, 5);

                        if (c == at[3]) {
                            px[c] = v & 0x8000 ? 255 : 0;
                        } else {
                            px[c] = e << 3 | e >> 2;
                            f++;
                        }
                    }
                    q = px;
                }
                r = q[at[0]];
                g = q[at[1]];
                b = q[at[2]];
                pa[idx] = q[at[3]];
                if (para & PARA_CSC) {
                    py[idx] = clip8(((k.m[0] * r + k.m[1] * g + k.m[2] * b) >>
                                     10) + k.yoff);
                    pu[idx] = clip8(((-k.m[3] * r - k.m[4] * g + k.m[5] * b) >>
                                     10) + k.uvoff);
                    pv[idx] = clip8(((k.m[6] * r - k.m[7] * g - k.m[8] * b) >>
                                     10) + k.uvoff);
                } else {
                    py[idx] = r;
                    pu[idx] = g;
                    pv[idx] = b;
                }
            }
        }
    }
    for (int idx = 0; idx < ow * oh; idx++) {
        if (para & PARA_GLOBAL_ALPHA) {
            pa[idx] = galpha;
        } else if (para & PARA_ALPHA_X_GLOBAL) {
            pa[idx] = pa[idx] * galpha >> 8;
        }
    }

    line = g_malloc(ow + 2);
    for (uint32_t j = 0; j < oh; j++) {
        hwaddr a = R(IPU_OUT_ADDR) + (hwaddr)(oy + j) * outs + ox;

        dma_read(a, line, ow);
        for (uint32_t i = 0; i < ow; i++) {
            line[i] = mix(line[i], py[j * ow + i], pa[j * ow + i], prem);
        }
        dma_write(a, line, ow);
    }

    /*
     * Chroma by 2x2 blocks: each pixel's own U and V blended, the four then
     * averaged; a pixel of the block outside the picture keeps the frame's
     */
    x0 = ox & ~1u;
    x1 = (ox + ow + 1) & ~1u;
    for (uint32_t cy = oy / 2; cy < (oy + oh + 1) / 2; cy++) {
        hwaddr a = R(IPU_NV_OUT_ADDR) + (hwaddr)cy * nvs + x0;

        dma_read(a, line, x1 - x0);
        for (uint32_t x = x0; x < x1; x += 2) {
            for (int c = 0; c < 2; c++) {
                const uint8_t *pc = c ? pv : pu;
                int bg = line[x - x0 + c], sum = 0;

                for (uint32_t yy = cy * 2; yy < cy * 2 + 2; yy++) {
                    for (uint32_t xx = x; xx < x + 2; xx++) {
                        if (yy >= oy && yy < oy + oh &&
                            xx >= ox && xx < ox + ow) {
                            int idx = (yy - oy) * ow + (xx - ox);

                            sum += mix(bg, pc[idx], pa[idx], prem);
                        } else {
                            sum += bg;
                        }
                    }
                }
                line[x - x0 + c] = (sum + 2) >> 2;
            }
        }
        dma_write(a, line, x1 - x0);
    }
}

/* The HSV of a pixel: H in degrees, S and V out of 255 */
static void ipu_hsv(int r, int g, int b, int *hue, int *sat, int *val)
{
    int mx = MAX(r, MAX(g, b)), mn = MIN(r, MIN(g, b)), d = mx - mn;
    int num, base;

    *val = mx;
    *sat = mx ? MIN(255, (d * ((16384 + mx / 2) / mx)) >> 6) : 0;
    *hue = 0;
    if (!d) {
        return;
    }
    if (mx == r) {
        num = g - b;
        base = 0;
    } else if (mx == g) {
        num = b - r;
        base = 120;
    } else {
        num = r - g;
        base = 240;
    }
    *hue = base + ((num * (((60 << 14) + d / 2) / d) + 8192) >> 14);
    if (mx == r && num < 0) {
        /* below red: from 360 down, a hue that rounds to 360 kept at 359 */
        *hue = MIN(359, *hue + 360);
    }
}

/*
 * The CSC command: the NV12 frame to NV21, 32-bit RGB in one of four orders
 * or HSV (H 16-bit at NV_OUT, S at OUT, V at OUT_V), through channel 0's
 * coefficients. It writes h + 1 rows, the last read from past the frame,
 * each plane's last row over the start of what follows it.
 */
static void ipu_convert(IngenicIpuState *s, uint32_t w, uint32_t h)
{
    unsigned fmt = R(IPU_D_FMT) & 0x7f;
    uint32_t ys = R(IPU_Y_STRIDE) ? R(IPU_Y_STRIDE) : w;
    uint32_t uvs = R(IPU_UV_STRIDE) >> 16 ? R(IPU_UV_STRIDE) >> 16 : w;
    uint32_t outs = R(IPU_OUT_STRIDE), nvs = R(IPU_NV_OUT_STRIDE);
    uint32_t ovs = R(IPU_OUT_V_STRIDE);
    static const char *const orders[] = {
        [DFMT_ARGB] = "ARGB", [DFMT_ABGR] = "ABGR",
        [DFMT_RGBA] = "RGBA", [DFMT_BGRA] = "BGRA",
    };
    g_autofree uint8_t *yl = g_malloc(w), *uvl = g_malloc(w + 1);
    g_autofree uint8_t *o1 = g_malloc(4 * w), *o2 = g_malloc(w);
    g_autofree uint8_t *o3 = g_malloc(w);
    IpuCsc k;

    if (fmt != DFMT_NV21 && fmt != DFMT_HSV &&
        (fmt >= ARRAY_SIZE(orders) || !orders[fmt])) {
        qemu_log_mask(LOG_UNIMP, "ingenic-ipu: output format 0x%x\n", fmt);
        return;
    }
    ipu_csc(s, 0, &k);
    for (uint32_t r = 0; r <= h; r++) {
        dma_read(R(IPU_Y_ADDR) + (hwaddr)r * ys, yl, w);
        dma_read(R(IPU_U_ADDR) + (hwaddr)(r / 2) * uvs, uvl, w);
        if (fmt == DFMT_NV21) {
            dma_write(R(IPU_OUT_ADDR) + (hwaddr)r * outs, yl, w);
            if (!(r & 1)) {
                for (uint32_t x = 0; x + 1 < w; x += 2) {
                    o2[x] = uvl[x + 1];
                    o2[x + 1] = uvl[x];
                }
                dma_write(R(IPU_NV_OUT_ADDR) + (hwaddr)(r / 2) * nvs, o2,
                          w & ~1u);
            }
            continue;
        }
        for (uint32_t x = 0; x < w; x++) {
            int yv = yl[x] - k.yoff;
            int u = uvl[x & ~1u] - k.uvoff, v = uvl[(x & ~1u) + 1] - k.uvoff;
            int rgb[3] = {
                clip8((k.m[0] * yv + k.m[1] * u + k.m[2] * v) >> 10),
                clip8((k.m[3] * yv - k.m[4] * u - k.m[5] * v) >> 10),
                clip8((k.m[6] * yv + k.m[7] * u + k.m[8] * v) >> 10),
            };

            if (fmt == DFMT_HSV) {
                int hue, sat, val;

                ipu_hsv(rgb[0], rgb[1], rgb[2], &hue, &sat, &val);
                stw_le_p(o1 + 2 * x, hue);
                o2[x] = sat;
                o3[x] = val;
            } else {
                for (int c = 0; c < 4; c++) {
                    const char *p = strchr("RGBA", orders[fmt][c]);

                    o1[4 * x + c] = p[0] == 'A' ? 0 : rgb[p - "RGBA"];
                }
            }
        }
        if (fmt == DFMT_HSV) {
            dma_write(R(IPU_NV_OUT_ADDR) + (hwaddr)r * nvs, o1, 2 * w);
            dma_write(R(IPU_OUT_ADDR) + (hwaddr)r * outs, o2, w);
            dma_write(R(IPU_OUT_V_ADDR) + (hwaddr)r * ovs, o3, w);
        } else {
            dma_write(R(IPU_OUT_ADDR) + (hwaddr)r * outs, o1, 4 * w);
        }
    }
}

static void ipu_run(IngenicIpuState *s)
{
    uint32_t gs = R(IPU_IN_FM_GS);
    uint32_t w = gs >> 16, h = gs & 0xffff;
    bool osd = false;

    for (int n = 0; n < INGENIC_IPU_OSD_CHANNELS; n++) {
        osd |= R(IPU_OSD_PARA(n)) & PARA_EN;
    }
    if (!w || !h) {
        qemu_log_mask(LOG_UNIMP, "ingenic-ipu: a job with no frame size\n");
    } else if (!osd) {
        ipu_convert(s, w, h);
    } else {
        if (R(IPU_OUT_ADDR) != R(IPU_Y_ADDR) ||
            R(IPU_NV_OUT_ADDR) != R(IPU_U_ADDR)) {
            uint32_t ys = R(IPU_Y_STRIDE) ? R(IPU_Y_STRIDE) : w;
            uint32_t uvs = R(IPU_UV_STRIDE) >> 16;
            uint32_t outs = R(IPU_OUT_STRIDE) ? R(IPU_OUT_STRIDE) : w;
            uint32_t nvs = R(IPU_NV_OUT_STRIDE) ? R(IPU_NV_OUT_STRIDE) : w;
            g_autofree uint8_t *line = g_malloc(w);

            uvs = uvs ? uvs : w;
            for (uint32_t j = 0; j < h; j++) {
                dma_read(R(IPU_Y_ADDR) + (hwaddr)j * ys, line, w);
                dma_write(R(IPU_OUT_ADDR) + (hwaddr)j * outs, line, w);
            }
            for (uint32_t j = 0; j < (h + 1) / 2; j++) {
                dma_read(R(IPU_U_ADDR) + (hwaddr)j * uvs, line, w);
                dma_write(R(IPU_NV_OUT_ADDR) + (hwaddr)j * nvs, line, w);
            }
        }
        for (int n = 0; n < INGENIC_IPU_OSD_CHANNELS; n++) {
            if (R(IPU_OSD_PARA(n)) & PARA_EN) {
                ipu_osd(s, n, w, h);
            }
        }
    }
    R(IPU_STATUS) |= STATUS_OUT_END;
}

static uint64_t ingenic_ipu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIpuState *s = INGENIC_IPU(opaque);

    switch (off) {
    case IPU_STATUS:
        return STATUS_IDLE | R(IPU_STATUS);
    case IPU_TRIGGER:
        return 0;
    default:
        return off < INGENIC_IPU_IOSIZE ? R(off) : 0;
    }
}

static void ingenic_ipu_write(void *opaque, hwaddr off, uint64_t value,
                              unsigned size)
{
    IngenicIpuState *s = INGENIC_IPU(opaque);
    uint32_t v = value;

    switch (off) {
    case IPU_STATUS:
        R(IPU_STATUS) = v & STATUS_OUT_END;
        break;
    case IPU_TRIGGER:
        if (v & (TRIGGER_STOP | TRIGGER_RESET)) {
            R(IPU_STATUS) = 0;
        }
        if (v & TRIGGER_RUN) {
            ipu_run(s);
        }
        break;
    default:
        R(off) = v;
        break;
    }
    ipu_update(s);
}

static const MemoryRegionOps ingenic_ipu_ops = {
    .read = ingenic_ipu_read,
    .write = ingenic_ipu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_ipu_reset_hold(Object *obj, ResetType type)
{
    IngenicIpuState *s = INGENIC_IPU(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void ingenic_ipu_reset_exit(Object *obj, ResetType type)
{
    ipu_update(INGENIC_IPU(obj));
}

static void ingenic_ipu_init(Object *obj)
{
    IngenicIpuState *s = INGENIC_IPU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ipu_ops, s,
                          TYPE_INGENIC_IPU, INGENIC_IPU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_ingenic_ipu = {
    .name = TYPE_INGENIC_IPU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIpuState, INGENIC_IPU_IOSIZE / 4),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ipu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic IPU (OSD blending)";
    dc->vmsd = &vmstate_ingenic_ipu;
    rc->phases.hold = ingenic_ipu_reset_hold;
    rc->phases.exit = ingenic_ipu_reset_exit;
}

static const TypeInfo ingenic_ipu_type_info = {
    .name = TYPE_INGENIC_IPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicIpuState),
    .instance_init = ingenic_ipu_init,
    .class_init = ingenic_ipu_class_init,
};

static void ingenic_ipu_register_types(void)
{
    type_register_static(&ingenic_ipu_type_info);
}

type_init(ingenic_ipu_register_types)
