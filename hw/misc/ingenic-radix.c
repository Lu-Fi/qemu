/*
 * Ingenic RADIX: the H.265 encoder of the T30
 *
 * libimp (i265e, an x265) sets up each picture as a command list in memory,
 * pairs of words (value, then VLD bit 31 | TERM bit 30 | register offset),
 * and the radix driver hands its address to the scheduler (ACM_CTRL, bit 0
 * to run). The model replays the list into its registers, sixteen modules
 * 0x8000 apart; the list ends with the front end's run (0x20018 = 1) and a
 * TERM pair, and the picture is then encoded by the H.265 core the AVPU
 * model uses, in the syntax libimp's headers announce: CTBs of 64, flat
 * scaling lists, no transform skip, one reference.
 *
 * What the list says, by register: the picture's size (0x20004, height in
 * bits 31:16) and NV12 source (0x2000c luma, 0x20010 chroma, strides
 * 0x20014 bits 15:0 and 31:16); the entropy coder's switches (0x68000):
 * slice type (3:2, 2 for I and 1 for P), slice QP (10:5), cu_qp_delta (4)
 * and its depth (19:18), MaxNumMergeCand (13:11), sign hiding (14), and SAO
 * (15) with its slice flags (16 luma, 17 chroma); the chroma QP offsets
 * (0x50010 bits 23:16 Cb, 31:24 Cr); where the slice data goes (0x68008),
 * as an RBSP ended by its stop bit, libimp putting the slice header in front
 * and adding the emulation prevention bytes; the reconstruction (0x10014,
 * 0x10018) and a P picture's reference (0x30800, 0x30804). Silicon's
 * reconstruction is tiled, by its registers' names; the model keeps NV12
 * rows, the picture's width apart, which fit the buffers and which nothing
 * but the core reads.
 *
 * At the end the core holds the slice data's length in bytes in 0x68024
 * and 0x44, and the status 0x104 (radix.h: RADIX_END, GLB_END) in 0x40. It
 * raises its interrupt while 0x40 bit 8 and the end interrupt's enable (0x0
 * bit 24) are both set. libimp takes a length of 0 for a failure.
 *
 * libimp's slice header announces no deblocking offsets, though it gives the
 * hardware -1 for both (0x50010 bits 15:0): the model deblocks as the
 * decoder will, with 0.
 *
 * Not modelled: motion search (inter blocks do not move), the QP generator
 * (one QP per picture: ROI, the per-CTB QP table at 0x20168), and the
 * statistics libimp's rate control reads back (0x40028-0x400b8,
 * 0x200a0-0x200bc, 0x20128-0x20130), which stay 0: a still scene to it.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "block/thread-pool.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-radix.h"
#include "hw/misc/ingenic-avpu.h"

/* CFGC, the scheduler */
#define CFGC_GLB_CTRL       0x00000
#define  GLB_INTE_ENDF      BIT(24)
#define CFGC_ACM_CTRL       0x00008
#define  ACM_RUN            BIT(0)
#define  ACM_ADDR           0xffffff00u
#define CFGC_STAT           0x00040
#define  STAT_ACM_ERR       BIT(1)
#define  STAT_GLB_END       BIT(2)
#define  STAT_END           BIT(8)
#define CFGC_BSLEN          0x00044

#define CL_VLD              BIT(31)
#define CL_TERM             BIT(30)
#define CL_REG              0x7fffcu
#define CL_MAX              0x4000      /* pairs; libimp's lists hold ~520 */

#define ODMA_RECON_Y        0x10014
#define ODMA_RECON_C        0x10018
#define EFE_GLB_CTRL        0x20000
#define  EFE_NV21           BIT(1)
#define EFE_FRM_SIZE        0x20004
#define EFE_RAWY            0x2000c
#define EFE_RAWC            0x20010
#define EFE_RAW_STR         0x20014
#define EFE_STAT            0x20018
#define  EFE_RUN            BIT(0)
#define MCE_FRM_STRD        0x30064
#define MCE_REF_Y           0x30800
#define MCE_REF_C           0x30804
#define DBLK_FLT_PARA       0x50010
#define SDE_CFG0            0x68000
#define SDE_BS_ADDR         0x68008
#define SDE_SLBL            0x68024

#define FRAME_MAX           4096        /* samples a side */

#define R(off)              (s->regs[(off) / 4])

struct IngenicRadixJob {
    IngenicRadixState *s;       /* NULL once the device has reset */
    IngenicAvpuPic pic;
    IngenicAvpuHevcSyntax syn;
    IngenicAvpuHevc *hevc;
    uint32_t w, h;
    uint8_t *y, *uv;            /* the source, @w apart */
    uint32_t bs, recon_y, recon_c;
    GByteArray *data;
    IngenicAvpuStats st;
    bool ok;
};

static void radix_update(IngenicRadixState *s)
{
    qemu_set_irq(s->irq, (R(CFGC_STAT) & STAT_END) &&
                         (R(CFGC_GLB_CTRL) & GLB_INTE_ENDF));
}

static void radix_plane_io(uint32_t addr, uint32_t stride, uint8_t *buf,
                           uint32_t w, uint32_t rows, bool write)
{
    for (uint32_t r = 0; r < rows; r++) {
        address_space_rw(&address_space_memory, addr + r * stride,
                         MEMTXATTRS_UNSPECIFIED, buf + r * w, w, write);
    }
}

static void radix_job_free(IngenicRadixJob *j)
{
    ingenic_avpu_hevc_free(j->hevc);
    g_free(j->y);
    g_free(j->uv);
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

static int radix_work(void *opaque)
{
    IngenicRadixJob *j = opaque;

    j->ok = ingenic_avpu_hevc_encode(j->hevc, &j->pic, j->y, j->uv, j->w,
                                     j->data, &j->st);
    return 0;
}

static void radix_done(void *opaque, int ret)
{
    IngenicRadixJob *j = opaque;
    IngenicRadixState *s = j->s;
    uint32_t len, room = j->w * j->h * 3 / 2;

    if (!s) {
        radix_job_free(j);
        return;
    }
    if (!j->ok) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-radix: a %ux%u picture could "
                      "not be encoded\n", j->w, j->h);
        g_byte_array_set_size(j->data, 0);
    }
    /* there is no size register: a raw picture's size is the cap */
    len = MIN(j->data->len, room);
    if (len < j->data->len) {
        qemu_log_mask(LOG_UNIMP, "ingenic-radix: %u bytes of slice data cut "
                      "to %u\n", j->data->len, room);
    }
    address_space_write(&address_space_memory, j->bs, MEMTXATTRS_UNSPECIFIED,
                        j->data->data, len);
    if (j->ok) {
        uint8_t *ref = ingenic_avpu_hevc_ref(j->hevc, j->w, j->h);
        const uint8_t *cb = ref + j->w * j->h, *cr = cb + j->w * j->h / 4;

        for (size_t i = 0; i < (size_t)j->w * j->h / 4; i++) {
            j->uv[2 * i] = cb[i];
            j->uv[2 * i + 1] = cr[i];
        }
        radix_plane_io(j->recon_y, j->w, ref, j->w, j->h, true);
        radix_plane_io(j->recon_c, j->w, j->uv, j->w, j->h / 2, true);
    }
    R(SDE_SLBL) = len;
    R(CFGC_BSLEN) = len;
    R(EFE_STAT) &= ~EFE_RUN;
    R(CFGC_STAT) |= STAT_END | STAT_GLB_END;
    s->job = NULL;
    radix_update(s);
    radix_job_free(j);
}

/* NV12 chroma rows, @w apart, into the core's Cb and Cr planes */
static void radix_split_chroma(const uint8_t *uv, uint8_t *cb, uint8_t *cr,
                               uint32_t w, uint32_t h)
{
    for (size_t i = 0; i < (size_t)w * h / 4; i++) {
        cb[i] = uv[2 * i];
        cr[i] = uv[2 * i + 1];
    }
}

static void radix_encode(IngenicRadixState *s)
{
    IngenicRadixJob *j = g_new0(IngenicRadixJob, 1);
    uint32_t cfg = R(SDE_CFG0), size = R(EFE_FRM_SIZE), flt = R(DBLK_FLT_PARA);
    uint32_t ystride = R(EFE_RAW_STR) & 0xffff, cstride = R(EFE_RAW_STR) >> 16;
    bool sao = extract32(cfg, 15, 1);
    int type = extract32(cfg, 2, 2);

    j->s = s;
    j->w = extract32(size, 0, 16);
    j->h = extract32(size, 16, 16);
    j->data = g_byte_array_new();
    j->hevc = ingenic_avpu_hevc_new();
    s->job = j;
    if (!j->w || !j->h || j->w % 16 || j->h % 8 || j->w > FRAME_MAX ||
        j->h > FRAME_MAX || (type != 1 && type != 2)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-radix: a %ux%u picture of "
                      "slice type %d\n", j->w, j->h, type);
        radix_done(j, 0);
        return;
    }

    j->syn = (IngenicAvpuHevcSyntax) {
        .log2_ctb = 6,
        .qp_delta_depth = extract32(cfg, 4, 1) ? extract32(cfg, 18, 2) : -1,
        .sao_luma = sao && extract32(cfg, 16, 1),
        .sao_chroma = sao && extract32(cfg, 17, 1),
        .sign_hiding = extract32(cfg, 14, 1),
        .merge_cands = MIN(MAX(extract32(cfg, 11, 3), 1), 5),
    };
    j->pic.width = j->w;
    j->pic.height = j->h;
    j->pic.qp = MIN(extract32(cfg, 5, 6), 51);
    j->pic.hevc = true;
    j->pic.intra = type == 2;
    j->pic.cabac = true;
    j->pic.syntax = &j->syn;
    j->pic.raw = true;
    j->pic.chroma_qp_offset = sextract32(flt, 16, 8);
    j->pic.cr_qp_offset = sextract32(flt, 24, 8);
    j->bs = R(SDE_BS_ADDR);
    j->recon_y = R(ODMA_RECON_Y);
    j->recon_c = R(ODMA_RECON_C);

    j->y = g_malloc((size_t)j->w * j->h);
    j->uv = g_malloc((size_t)j->w * j->h / 2);
    radix_plane_io(R(EFE_RAWY), ystride ?: j->w, j->y, j->w, j->h, false);
    radix_plane_io(R(EFE_RAWC), cstride ?: j->w, j->uv, j->w, j->h / 2,
                   false);
    if (R(EFE_GLB_CTRL) & EFE_NV21) {
        for (size_t i = 0; i < (size_t)j->w * j->h / 2; i += 2) {
            uint8_t v = j->uv[i];

            j->uv[i] = j->uv[i + 1];
            j->uv[i + 1] = v;
        }
    }
    if (!j->pic.intra) {
        uint8_t *ref = ingenic_avpu_hevc_ref(j->hevc, j->w, j->h);
        g_autofree uint8_t *uv = g_malloc((size_t)j->w * j->h / 2);
        uint32_t strides = R(MCE_FRM_STRD);

        radix_plane_io(R(MCE_REF_Y), (strides & 0xffff) ?: j->w, ref, j->w,
                       j->h, false);
        radix_plane_io(R(MCE_REF_C), (strides >> 16) ?: j->w, uv, j->w,
                       j->h / 2, false);
        radix_split_chroma(uv, ref + j->w * j->h,
                           ref + j->w * j->h * 5 / 4, j->w, j->h);
    }
    thread_pool_submit_aio(radix_work, j, radix_done, j);
}

/* Replay a command list into the registers; the last pair ends it */
static void radix_run(IngenicRadixState *s, uint32_t list)
{
    uint32_t pair[2], cmd = 0, addr = list;
    int n;

    if (s->job) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-radix: started while busy\n");
        return;
    }
    for (n = 0; n < CL_MAX; n++, addr += 8) {
        if (address_space_read(&address_space_memory, addr,
                               MEMTXATTRS_UNSPECIFIED, pair, sizeof(pair)) !=
            MEMTX_OK) {
            break;
        }
        cmd = le32_to_cpu(pair[1]);
        if (cmd & CL_VLD) {
            R(cmd & CL_REG) = le32_to_cpu(pair[0]);
        }
        if (cmd & CL_TERM) {
            break;
        }
    }
    if (n == CL_MAX || !(cmd & CL_TERM)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-radix: command list at "
                      "0x%08x has no end\n", list);
        R(CFGC_STAT) |= STAT_ACM_ERR;
        return;
    }
    if (R(EFE_STAT) & EFE_RUN) {
        radix_encode(s);
    }
}

static uint64_t ingenic_radix_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicRadixState *s = INGENIC_RADIX(opaque);

    return R(addr);
}

static void ingenic_radix_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned size)
{
    IngenicRadixState *s = INGENIC_RADIX(opaque);
    uint32_t v = value;

    switch (addr) {
    case CFGC_GLB_CTRL:
    case CFGC_STAT:
        R(addr) = v;
        radix_update(s);
        return;
    case CFGC_ACM_CTRL:
        R(addr) = v;
        if (v & ACM_RUN) {
            radix_run(s, v & ACM_ADDR);
        }
        return;
    }
    R(addr) = v;
}

static const MemoryRegionOps ingenic_radix_ops = {
    .read = ingenic_radix_read,
    .write = ingenic_radix_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_radix_reset_hold(Object *obj, ResetType type)
{
    IngenicRadixState *s = INGENIC_RADIX(obj);

    if (s->job) {
        s->job->s = NULL;           /* its completion just frees it */
        s->job = NULL;
    }
    memset(s->regs, 0, INGENIC_RADIX_IOSIZE);
}

static void ingenic_radix_reset_exit(Object *obj, ResetType type)
{
    radix_update(INGENIC_RADIX(obj));
}

/* The CPM's soft reset (SRBC bit 28), which the kernel pulses per job */
static void radix_soft_reset(void *opaque, int n, int level)
{
    if (level) {
        ingenic_radix_reset_hold(OBJECT(opaque), RESET_TYPE_COLD);
        radix_update(INGENIC_RADIX(opaque));
    }
}

static void ingenic_radix_init(Object *obj)
{
    IngenicRadixState *s = INGENIC_RADIX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->regs = g_malloc0(INGENIC_RADIX_IOSIZE);
    memory_region_init_io(&s->iomem, obj, &ingenic_radix_ops, s,
                          TYPE_INGENIC_RADIX, INGENIC_RADIX_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), radix_soft_reset, "reset", 1);
}

static void ingenic_radix_finalize(Object *obj)
{
    g_free(INGENIC_RADIX(obj)->regs);
}

/* An encode in flight is host state */
static const VMStateDescription vmstate_ingenic_radix = {
    .name = TYPE_INGENIC_RADIX,
    .unmigratable = 1,
};

static void ingenic_radix_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic RADIX H.265 encoder";
    dc->vmsd = &vmstate_ingenic_radix;
    rc->phases.hold = ingenic_radix_reset_hold;
    rc->phases.exit = ingenic_radix_reset_exit;
}

static const TypeInfo ingenic_radix_types[] = {
    {
        .name = TYPE_INGENIC_RADIX,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicRadixState),
        .instance_init = ingenic_radix_init,
        .instance_finalize = ingenic_radix_finalize,
        .class_init = ingenic_radix_class_init,
    },
};

DEFINE_TYPES(ingenic_radix_types)
