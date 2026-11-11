/*
 * Ingenic A1 IPU: the descriptor-driven 2D engine at 0x13080000
 *
 * Mapped from the vendor ipu.ko and the libimp that drives it (TDE).
 * Registers:
 *   +0x10 GO (write 1 to run the descriptor at +0x14), +0x14 CHAIN
 *   +0x20 STATUS (bit 0: the chain is done), cleared by writing the bits
 *   to +0x24; +0x2c masks it
 *   +0x40 and +0x1024 reset (write 1; both read 0 at once)
 *   +0x1010 starts the job the registers describe
 *   +0x1014 IIP raw status (bits 1, 2 and 3 are errors), cleared by
 *   writing the bits to +0x1018; +0x101c masks it
 *   +0x1040 BK, +0x1060 FG, +0x1080 MK and +0x10a0 OT DMA: SIZE (width <<
 *   16 | height), ADDR, ADDR_UV, STRIDE (signed 16 bits of Y, UV above),
 *   INFO (bits 19:16 the pixel type, 23:20 its channel order, bit 30 a
 *   chroma plane, bit 31 enabled), INFO2, INFO3
 *   +0x10c0 CLIP, +0x10d0 CSC, +0x10f0 colour key, +0x1108 OSD, +0x1110
 *   SCALER, +0x1140 CP_DMA; all plain registers a job reads.
 * A descriptor is four words, the first (low 4 bits the TDE operation,
 * bits 4 and 5 the scaler, bits 31:16 the length of the rest) followed by
 * the next descriptor's address (0xffffffff for none) and the vertical and
 * horizontal scaler tables, then a list of register writes: a word of
 * count << 16 | register offset and that many values. The write of 1 to
 * 0x1010 that ends the list runs the job. The chain's end raises the IRQ
 * with STATUS bit 0.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bitops.h"
#include "qemu/units.h"
#include "hw/core/irq.h"
#include "hw/misc/ingenic-a1-ipu.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

#define REG_GO          0x10
#define REG_CHAIN       0x14
#define REG_STATUS      0x20
#define REG_STATUS_CLR  0x24
#define REG_STATUS_MASK 0x2c
#define REG_RESET       0x40
#define REG_KICK        0x1010
#define REG_IIP_RAW     0x1014
#define REG_IIP_CLR     0x1018
#define REG_IIP_MASK    0x101c
#define REG_RESET2      0x1024
#define REG_BK          0x1040
#define REG_FG          0x1060
#define REG_MK          0x1080
#define REG_OT          0x10a0
#define REG_CLIP        0x10c0

#define STATUS_DONE     0x1
#define IIP_BADJOB      0x4

#define DESC_NONE       0xffffffffu
#define DESC_HDR        4
#define DESC_MAXWORDS   0x400

#define DMA_SIZE        0
#define DMA_ADDR        1
#define DMA_ADDR_UV     2
#define DMA_STRIDE      3
#define DMA_INFO        4

#define INFO_TYPE(i)    extract32(i, 16, 4)
#define INFO_CHROMA     (1u << 30)
#define INFO_EN         (1u << 31)
#define INFO_CONST      (1u << 25)      /* FG: a constant colour in INFO3 */
#define INFO_MIRROR     (1u << 28)      /* OT: each row turned round */

#define NS_PER_PIXEL    2

typedef struct Surface {
    uint32_t w, h;
    uint32_t addr, addr_uv;
    int32_t stride, stride_uv;
    uint32_t info;
    unsigned type;
} Surface;

static uint32_t rd32(hwaddr addr)
{
    uint32_t v = 0;

    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       &v, 4);
    return le32_to_cpu(v);
}

static void mem_read(hwaddr addr, void *buf, hwaddr len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
}

static void mem_write(hwaddr addr, const void *buf, hwaddr len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                        buf, len);
}

static void ipu_update_irq(IngenicA1IpuState *s)
{
    bool on = (s->status & ~s->status_mask) || (s->iip_raw & ~s->iip_mask);

    qemu_set_irq(s->irq, on);
}

/* Bytes of one pixel of a type; 0 for the planar ones and unknown types */
static unsigned type_bpp(unsigned type)
{
    static const uint8_t bpp[8] = { 3, 2, 2, 2, 4, 3, 2, 2 };

    return type < ARRAY_SIZE(bpp) ? bpp[type] : 0;
}

static bool type_planar(unsigned type)
{
    return type >= 8 && type <= 11;
}

static void surface_load(IngenicA1IpuState *s, unsigned base, Surface *sf)
{
    const uint32_t *r = &s->regs[base / 4];

    sf->w = r[DMA_SIZE] >> 16;
    sf->h = r[DMA_SIZE] & 0xffff;
    sf->addr = r[DMA_ADDR];
    sf->addr_uv = r[DMA_ADDR_UV];
    sf->stride = (int16_t)r[DMA_STRIDE];
    sf->stride_uv = (int16_t)(r[DMA_STRIDE] >> 16);
    sf->info = r[DMA_INFO];
    sf->type = INFO_TYPE(sf->info);
}

/* The bytes a row of the surface holds, and how many rows each plane has */
static bool plane_geometry(const Surface *sf, unsigned plane, uint32_t *bytes,
                           uint32_t *rows)
{
    if (!type_planar(sf->type)) {
        if (plane || !type_bpp(sf->type)) {
            return false;
        }
        *bytes = sf->w * type_bpp(sf->type);
        *rows = sf->h;
        return true;
    }
    *bytes = sf->w;
    /* NV12 and NV21 halve the chroma rows, NV16 and NV61 do not */
    *rows = plane && sf->type < 10 ? sf->h / 2 : sf->h;
    return true;
}

typedef struct Job {
    uint32_t header;
    uint32_t tab_h, tab_v;      /* the scaler tables, at the descriptor */
} Job;

/* The unit a mirror turns round in a plane: a pixel, or the UV pair */
static unsigned plane_unit(const Surface *sf, unsigned plane)
{
    if (!type_planar(sf->type)) {
        return type_bpp(sf->type);
    }
    return plane ? 2 : 1;
}

/*
 * Rows from FG to OT without changing the pixels: the surfaces share a
 * type and a size. A negative FG stride reads the rows bottom up (the
 * start address stays the first row's), OT bit 28 turns each row round, the
 * clip box keeps what it holds or all but that, and a constant FG (bit 25)
 * fills with INFO3 in OT's format.
 */
static void job_raw(IngenicA1IpuState *s, const Surface *src,
                    const Surface *dst, bool fill, uint32_t color)
{
    bool mirror = dst->info & INFO_MIRROR;
    uint32_t clip_en = s->regs[REG_CLIP / 4 + 2] >> 31;
    bool inside = s->regs[REG_CLIP / 4 + 2] & 1;
    uint32_t cx = s->regs[REG_CLIP / 4] >> 16;
    uint32_t cy = s->regs[REG_CLIP / 4] & 0xffff;
    uint32_t cw = s->regs[REG_CLIP / 4 + 1] >> 16;
    uint32_t ch = s->regs[REG_CLIP / 4 + 1] & 0xffff;
    g_autofree uint8_t *row = NULL;
    g_autofree uint8_t *tmp = NULL;

    for (unsigned plane = 0; plane < (type_planar(dst->type) ? 2 : 1);
         plane++) {
        uint32_t bytes, rows;
        unsigned unit = plane_unit(dst, plane);
        unsigned sub = plane && dst->type < 10 ? 2 : 1;
        hwaddr sa = plane ? src->addr_uv : src->addr;
        hwaddr da = plane ? dst->addr_uv : dst->addr;
        int32_t ss = plane ? src->stride_uv : src->stride;
        int32_t ds = plane ? dst->stride_uv : dst->stride;

        if (!plane_geometry(dst, plane, &bytes, &rows)) {
            continue;
        }
        row = g_realloc(row, bytes);
        tmp = g_realloc(tmp, bytes);
        for (uint32_t y = 0; y < rows; y++) {
            uint32_t sy = ss < 0 ? rows - 1 - y : y;
            hwaddr base = sa + (int64_t)sy * abs(ss);

            if (fill) {
                for (uint32_t i = 0; i < bytes; i++) {
                    row[i] = color >> (8 * (i % unit));
                }
            } else {
                mem_read(base, row, bytes);
            }
            if (mirror) {
                for (uint32_t i = 0; i < bytes; i += unit) {
                    memcpy(tmp + i, row + bytes - unit - i, unit);
                }
                memcpy(row, tmp, bytes);
            }
            if (clip_en) {
                bool row_in = y * sub >= cy && y * sub < cy + ch;
                uint32_t x0 = MIN(cx / sub * unit, bytes);
                uint32_t x1 = MIN((cx + cw + sub - 1) / sub * unit, bytes);

                if (inside) {
                    if (!row_in) {
                        continue;
                    }
                    mem_write(da + (int64_t)y * ds + x0, row + x0, x1 - x0);
                } else if (!row_in) {
                    mem_write(da + (int64_t)y * ds, row, bytes);
                } else {
                    mem_write(da + (int64_t)y * ds, row, x0);
                    mem_write(da + (int64_t)y * ds + x1, row + x1, bytes - x1);
                }
                continue;
            }
            mem_write(da + (int64_t)y * ds, row, bytes);
        }
    }
}

static bool ipu_run_job(IngenicA1IpuState *s, const Job *job)
{
    Surface src, dst;
    bool fill;

    surface_load(s, REG_FG, &src);
    surface_load(s, REG_OT, &dst);
    fill = src.info & INFO_CONST;
    if (!(dst.info & INFO_EN) || !(fill || (src.info & INFO_EN))) {
        return false;
    }
    if (fill) {
        src.type = dst.type;
        src.w = dst.w;
        src.h = dst.h;
    }
    if (src.type != dst.type || src.w != dst.w || src.h != dst.h) {
        qemu_log_mask(LOG_UNIMP, "ingenic-a1-ipu: job (operation %u) %ux%u "
                      "type %u to %ux%u type %u\n", job->header & 0xf, src.w,
                      src.h, src.type, dst.w, dst.h, dst.type);
        return false;
    }
    job_raw(s, &src, &dst, fill, s->regs[REG_FG / 4 + 6]);
    s->busy_ns += (int64_t)dst.w * dst.h * NS_PER_PIXEL;
    return true;
}

/* Run one descriptor; returns the next one's address */
static uint32_t ipu_run_desc(IngenicA1IpuState *s, uint32_t addr)
{
    Job job = { .header = rd32(addr), .tab_h = rd32(addr + 8),
                .tab_v = rd32(addr + 12) };
    uint32_t hdr = job.header;
    uint32_t next = rd32(addr + 4);
    uint32_t nwords = hdr >> 16;
    uint32_t i = 0;

    trace_ingenic_a1_ipu_desc(addr, hdr, next, nwords);
    if (nwords > DESC_MAXWORDS) {
        s->iip_raw |= IIP_BADJOB;
        return DESC_NONE;
    }
    while (i < nwords) {
        uint32_t cmd = rd32(addr + (DESC_HDR + i++) * 4);
        uint32_t reg = cmd & 0xffff;
        uint32_t count = cmd >> 16;

        for (uint32_t k = 0; k < count && i < nwords; k++, i++) {
            uint32_t v = rd32(addr + (DESC_HDR + i) * 4);
            uint32_t r = reg + k * 4;

            trace_ingenic_a1_ipu_write(r, v);
            if (r >= REG_KICK && r < REG_KICK + 4) {
                if (v & 1) {
                    if (!ipu_run_job(s, &job)) {
                        s->iip_raw |= IIP_BADJOB;
                    }
                }
            } else if (r / 4 < INGENIC_A1_IPU_NREGS) {
                s->regs[r / 4] = v;
            }
        }
    }
    return next;
}

static void ipu_timer_cb(void *opaque)
{
    IngenicA1IpuState *s = opaque;

    s->running = false;
    s->status |= STATUS_DONE;
    ipu_update_irq(s);
}

static void ipu_go(IngenicA1IpuState *s)
{
    uint32_t addr = s->chain;
    unsigned guard = 0;

    s->busy_ns = 1000;
    while (addr != DESC_NONE && addr && guard++ < 64) {
        addr = ipu_run_desc(s, addr);
    }
    s->running = true;
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->busy_ns);
}

static void ipu_reset_regs(IngenicA1IpuState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->status = 0;
    s->iip_raw = 0;
    s->running = false;
    timer_del(s->timer);
    ipu_update_irq(s);
}

static uint64_t ipu_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicA1IpuState *s = opaque;

    switch (addr) {
    case REG_GO:
        return s->running;
    case REG_CHAIN:
        return s->chain;
    case REG_STATUS:
        return s->status;
    case REG_STATUS_MASK:
        return s->status_mask;
    case REG_RESET:
    case REG_RESET2:
        return 0;
    case REG_IIP_RAW:
        return s->iip_raw;
    case REG_IIP_MASK:
        return s->iip_mask;
    }
    if (addr < INGENIC_A1_IPU_NREGS * 4 && addr >= REG_KICK) {
        return s->regs[addr / 4];
    }
    return 0;
}

static void ipu_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    IngenicA1IpuState *s = opaque;

    switch (addr) {
    case REG_GO:
        if ((val & 1) && !s->running) {
            ipu_go(s);
        }
        return;
    case REG_CHAIN:
        s->chain = val;
        return;
    case REG_STATUS_CLR:
        s->status &= ~val;
        break;
    case REG_STATUS_MASK:
        s->status_mask = val;
        break;
    case REG_RESET:
    case REG_RESET2:
        if (val & 1) {
            ipu_reset_regs(s);
        }
        return;
    case REG_IIP_CLR:
        s->iip_raw &= ~val;
        break;
    case REG_IIP_MASK:
        s->iip_mask = val;
        break;
    case REG_KICK:
        if (val & 1) {
            ipu_run_job(s, &(Job){ 0 });
        }
        return;
    default:
        if (addr >= REG_KICK && addr < INGENIC_A1_IPU_NREGS * 4) {
            s->regs[addr / 4] = val;
        }
        return;
    }
    ipu_update_irq(s);
}

static const MemoryRegionOps ipu_ops = {
    .read = ipu_read,
    .write = ipu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ipu_reset_hold(Object *obj, ResetType type)
{
    IngenicA1IpuState *s = INGENIC_A1_IPU(obj);

    ipu_reset_regs(s);
    s->chain = 0;
    s->status_mask = 0;
    s->iip_mask = 0;
}

static void ipu_realize(DeviceState *dev, Error **errp)
{
    IngenicA1IpuState *s = INGENIC_A1_IPU(dev);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ipu_timer_cb, s);
}

static void ipu_init(Object *obj)
{
    IngenicA1IpuState *s = INGENIC_A1_IPU(obj);

    memory_region_init_io(&s->iomem, obj, &ipu_ops, s, "ingenic-a1-ipu",
                          INGENIC_A1_IPU_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const VMStateDescription vmstate_ipu = {
    .name = "ingenic-a1-ipu",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicA1IpuState, INGENIC_A1_IPU_NREGS),
        VMSTATE_UINT32(status, IngenicA1IpuState),
        VMSTATE_UINT32(status_mask, IngenicA1IpuState),
        VMSTATE_UINT32(iip_raw, IngenicA1IpuState),
        VMSTATE_UINT32(iip_mask, IngenicA1IpuState),
        VMSTATE_UINT32(chain, IngenicA1IpuState),
        VMSTATE_BOOL(running, IngenicA1IpuState),
        VMSTATE_TIMER_PTR(timer, IngenicA1IpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ipu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ipu_realize;
    dc->vmsd = &vmstate_ipu;
    rc->phases.hold = ipu_reset_hold;
}

static const TypeInfo ipu_type_info = {
    .name = TYPE_INGENIC_A1_IPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1IpuState),
    .instance_init = ipu_init,
    .class_init = ipu_class_init,
};

static void ipu_register_types(void)
{
    type_register_static(&ipu_type_info);
}

type_init(ipu_register_types)
