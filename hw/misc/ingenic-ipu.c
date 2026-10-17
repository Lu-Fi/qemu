/*
 * Ingenic IPU (the T31's and T40's, ipu_v13): OSD blending
 *
 * libimp draws its OSD regions onto each frame before encoding with the
 * IPU, one job per region: the frame is the NV12 background (IPU_Y_ADDR,
 * IPU_U_ADDR, IPU_IN_FM_GS), usually also the output (IPU_OUT_ADDR,
 * IPU_NV_OUT_ADDR), and up to four OSD channels, each a picture at a
 * position (IPU_OSD_CHn_POS) blended in by its IPU_OSD_CHn_PARA: bit 0
 * enables it, bit 1 takes the global alpha (bits 10:3) instead of the
 * pixels', bit 2 multiplies the two, bits 13:11 give the picture type
 * (ARGB, RGB, NV12, NV21), bits 17:14 the order of a pixel's bytes, bit 18
 * says the colours are premultiplied. An RGB picture becomes YUV through
 * the channel's coefficients: C0..C4 hold R, G, B to Y and to U and V,
 * two to a register (bits 11:0 and 31:20), the offsets Y in bits 15:0 and
 * UV in bits 31:16 of IPU_CHn_CSC_OFSET_PARA, out of 1024.
 *
 * Writing IPU_RUN to IPU_TRIGGER runs the job; IPU_STATUS bit 0 (OUT_END)
 * then reads set, and the interrupt is up while it is and IPU_GLB_CTRL
 * enables it. The driver clears OUT_END itself.
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
#define IPU_Y_ADDR          0x00c
#define IPU_U_ADDR          0x010
#define IPU_IN_FM_GS        0x018
#define IPU_Y_STRIDE        0x01c
#define IPU_UV_STRIDE       0x020
#define IPU_OUT_ADDR        0x024
#define IPU_OUT_STRIDE      0x02c
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

enum { PIC_ARGB, PIC_RGB, PIC_NV12, PIC_NV21 };

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

static uint8_t blend(int bg, int fg, int a, bool premultiplied)
{
    if (premultiplied) {
        return clip8(fg + bg * (255 - a) / 255);
    }
    return clip8((fg * a + bg * (255 - a) + 127) / 255);
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
    const uint32_t *c = &R(IPU_CSC(n, 0));
    int ry = extract32(c[0], 0, 12), gy = extract32(c[0], 20, 12);
    int by = extract32(c[1], 0, 12), ru = extract32(c[1], 20, 12);
    int gu = extract32(c[2], 0, 12), bu = extract32(c[2], 20, 12);
    int rv = extract32(c[3], 0, 12), gv = extract32(c[3], 20, 12);
    int bv = extract32(c[4], 0, 12);
    int yoff = extract32(c[5], 0, 16), uvoff = extract32(c[5], 16, 16);
    g_autofree uint8_t *pic = NULL, *picuv = NULL;
    g_autofree uint8_t *py = NULL, *pu = NULL, *pv = NULL, *pa = NULL;
    g_autofree uint8_t *line = NULL;
    uint32_t x0, x1;
    int bpp = 1, at[4];

    if (type == PIC_ARGB || type == PIC_RGB) {
        bpp = type == PIC_ARGB ? 4 : 3;
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

    /* The picture as YUV and alpha, a sample of each per pixel */
    py = g_malloc(ow * oh);
    pu = g_malloc(ow * oh);
    pv = g_malloc(ow * oh);
    pa = g_malloc(ow * oh);
    pic = g_malloc((size_t)stride * oh);
    dma_read(R(IPU_OSD_IN_Y_ADDR(n)), pic, (size_t)stride * oh);
    if (type == PIC_NV12 || type == PIC_NV21) {
        uint32_t uvstride = R(IPU_OSD_IN_UV_STRIDE(n));

        picuv = g_malloc((size_t)uvstride * ((oh + 1) / 2));
        dma_read(R(IPU_OSD_IN_UV_ADDR(n)), picuv,
                 (size_t)uvstride * ((oh + 1) / 2));
        for (uint32_t j = 0; j < oh; j++) {
            for (uint32_t i = 0; i < ow; i++) {
                const uint8_t *c2 = picuv + (j / 2) * uvstride + (i & ~1u);
                int k = j * ow + i;

                py[k] = pic[j * stride + i];
                pu[k] = c2[type == PIC_NV21];
                pv[k] = c2[type != PIC_NV21];
                pa[k] = 255;
            }
        }
    } else {
        for (uint32_t j = 0; j < oh; j++) {
            for (uint32_t i = 0; i < ow; i++) {
                const uint8_t *q = pic + j * stride + i * bpp;
                int r = q[at[0]], g = q[at[1]], b = q[at[2]];
                int k = j * ow + i;

                pa[k] = bpp == 4 ? q[at[3]] : 255;
                py[k] = clip8(((ry * r + gy * g + by * b) >> 10) + yoff);
                pu[k] = clip8(((-ru * r - gu * g + bu * b) >> 10) + uvoff);
                pv[k] = clip8(((rv * r - gv * g - bv * b) >> 10) + uvoff);
            }
        }
    }
    for (int k = 0; k < ow * oh; k++) {
        if (para & PARA_GLOBAL_ALPHA) {
            pa[k] = galpha;
        } else if (para & PARA_ALPHA_X_GLOBAL) {
            pa[k] = pa[k] * galpha / 255;
        }
    }

    line = g_malloc(ow + 2);
    for (uint32_t j = 0; j < oh; j++) {
        hwaddr a = R(IPU_OUT_ADDR) + (hwaddr)(oy + j) * outs + ox;

        dma_read(a, line, ow);
        for (uint32_t i = 0; i < ow; i++) {
            line[i] = blend(line[i], py[j * ow + i], pa[j * ow + i], prem);
        }
        dma_write(a, line, ow);
    }

    /* Chroma by 2x2 blocks: their mean colour at their mean alpha */
    x0 = ox & ~1u;
    x1 = (ox + ow + 1) & ~1u;
    for (uint32_t cy = oy / 2; cy < (oy + oh + 1) / 2; cy++) {
        hwaddr a = R(IPU_NV_OUT_ADDR) + (hwaddr)cy * nvs + x0;

        dma_read(a, line, x1 - x0);
        for (uint32_t x = x0; x < x1; x += 2) {
            int u = 0, v = 0, al = 0, cnt = 0;

            for (uint32_t yy = cy * 2; yy < cy * 2 + 2; yy++) {
                for (uint32_t xx = x; xx < x + 2; xx++) {
                    if (yy >= oy && yy < oy + oh && xx >= ox && xx < ox + ow) {
                        int k = (yy - oy) * ow + (xx - ox);

                        u += pu[k];
                        v += pv[k];
                        al += pa[k];
                        cnt++;
                    }
                }
            }
            if (cnt) {
                /* a block the picture only partly covers is covered less */
                line[x - x0] = blend(line[x - x0], u / cnt, al / 4, prem);
                line[x - x0 + 1] = blend(line[x - x0 + 1], v / cnt, al / 4,
                                         prem);
            }
        }
        dma_write(a, line, x1 - x0);
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
    if (!w || !h || !osd) {
        qemu_log_mask(LOG_UNIMP, "ingenic-ipu: only OSD blending onto NV12 "
                      "is modelled\n");
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
