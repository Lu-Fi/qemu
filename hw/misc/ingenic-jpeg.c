/*
 * Ingenic JPEG engine: the T32's JPEG encoder, beside its HERA
 *
 * The helix driver writes a job's registers from libimp's register image
 * and starts it: an NV12 picture (0x14 luma, 0x18 chroma, both rows 0x10
 * apart) of 0xc bits 15:0 by 31:16, coded as baseline 4:2:0 to 0x1c with
 * the base tables at 0x300 (luma, then chroma, zigzag, four to a word)
 * scaled by the index in 0x4 bits 24:18, which libimp sets to 98 less the
 * quality: the model scales as the IJG library does for that quality. The
 * engine writes the whole file, which the model takes from the AVPU's JPEG
 * core: JFIF, the scaled tables, the Annex K Huffman tables.
 *
 * Writing 0x0 bit 1 resets the engine, which then reads bit 31 set; bit 0
 * starts the job. At the end it holds the file's length in 0x104, frame
 * end in 0x100 bit 31 and 0x110 bit 0, or, past the limit at 0x2c that
 * 0x4 bit 16 enables, only 0x110 bit 1 and nothing written. Interrupt k
 * (0x110 bit k) is enabled by 0x4 bit 25 + k; 0x114 clears.
 *
 * Not modelled: restart markers (0x28), IVDC sources, the PRJ008 and later
 * engines' multi-section output.
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
#include "hw/misc/ingenic-jpeg.h"
#include "hw/misc/ingenic-avpu.h"

#define JPEG_TRIG           0x000
#define  TRIG_START         BIT(0)
#define  TRIG_RESET         BIT(1)
#define  TRIG_RESET_END     BIT(31)
#define JPEG_CTRL           0x004
#define  CTRL_LIMIT         BIT(16)
#define JPEG_FRM_INFO       0x00c
#define JPEG_LINE_STRD      0x010
#define JPEG_RAWY           0x014
#define JPEG_RAWC           0x018
#define JPEG_BS_ADDR        0x01c
#define JPEG_LIMIT          0x02c
#define JPEG_GLB_STAT       0x100
#define  STAT_FRM_END       BIT(31)
#define JPEG_GLB_BS_LEN     0x104
#define JPEG_INTR_STA       0x110
#define JPEG_INTR_CLR       0x114
#define  INTR_FRAME_END     BIT(0)
#define  INTR_BSLESS        BIT(1)
#define JPEG_QT_BASE        0x300

#define FRAME_MAX           16384       /* samples a side */

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
    uint8_t *y, *uv;            /* the source, @w apart */
    uint8_t qt[2][64];          /* zigzag */
    uint16_t recip[2][64];      /* natural order */
    GByteArray *data;
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
    if (j->data) {
        g_byte_array_unref(j->data);
    }
    g_free(j);
}

static int jpeg_work(void *opaque)
{
    IngenicJpegJob *j = opaque;

    ingenic_avpu_jpeg_encode(j->y, j->w, j->uv, j->w, j->w, j->h, j->qt,
                             j->recip, 0, 1, 1, j->data);
    return 0;
}

static void jpeg_done(void *opaque, int ret)
{
    IngenicJpegJob *j = opaque;
    IngenicJpegState *s = j->s;

    if (!s) {
        jpeg_job_free(j);
        return;
    }
    if (j->limit && j->data->len > j->limit) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-jpeg: %u bytes over the "
                      "%u byte limit\n", j->data->len, j->limit);
        R(JPEG_GLB_BS_LEN) = 0;
        R(JPEG_INTR_STA) |= INTR_BSLESS;
    } else {
        address_space_write(&address_space_memory, j->bs,
                            MEMTXATTRS_UNSPECIFIED, j->data->data,
                            j->data->len);
        R(JPEG_GLB_BS_LEN) = j->data->len;
        R(JPEG_GLB_STAT) |= STAT_FRM_END;
        R(JPEG_INTR_STA) |= INTR_FRAME_END;
    }
    s->job = NULL;
    jpeg_update(s);
    jpeg_job_free(j);
}

/* The base tables, by the index in CTRL: the IJG scaling for 98 - index */
static void jpeg_tables(IngenicJpegState *s, IngenicJpegJob *j)
{
    int quality = MIN(MAX(98 - (int)extract32(R(JPEG_CTRL), 18, 7), 1), 98);
    int scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;

    for (int t = 0; t < 2; t++) {
        for (int i = 0; i < 64; i++) {
            uint32_t word = R(JPEG_QT_BASE + 64 * t + (i & ~3));
            int q = ((int)extract32(word, 8 * (i & 3), 8) * scale + 50) / 100;

            q = MIN(MAX(q, 1), 255);
            j->qt[t][i] = q;
            j->recip[t][zigzag[i]] = (65536 + q / 2) / q - 1;
        }
    }
}

static void jpeg_start(IngenicJpegState *s)
{
    IngenicJpegJob *j;
    uint32_t info = R(JPEG_FRM_INFO), stride = R(JPEG_LINE_STRD);

    if (s->job) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-jpeg: started while busy\n");
        return;
    }
    j = g_new0(IngenicJpegJob, 1);
    j->s = s;
    j->w = info & 0xffff;
    j->h = info >> 16;
    j->bs = R(JPEG_BS_ADDR);
    j->limit = R(JPEG_CTRL) & CTRL_LIMIT ? R(JPEG_LIMIT) : 0;
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
    for (uint32_t r = 0; r < j->h; r++) {
        address_space_read(&address_space_memory, R(JPEG_RAWY) + r * stride,
                           MEMTXATTRS_UNSPECIFIED, j->y + r * j->w, j->w);
    }
    for (uint32_t r = 0; r < (j->h + 1) / 2; r++) {
        address_space_read(&address_space_memory, R(JPEG_RAWC) + r * stride,
                           MEMTXATTRS_UNSPECIFIED, j->uv + r * j->w, j->w);
    }
    thread_pool_submit_aio(jpeg_work, j, jpeg_done, j);
}

static void jpeg_reset(IngenicJpegState *s)
{
    if (s->job) {
        s->job->s = NULL;           /* its completion just frees it */
        s->job = NULL;
    }
    R(JPEG_INTR_STA) = 0;
    R(JPEG_GLB_STAT) = 0;
    R(JPEG_GLB_BS_LEN) = 0;
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
            jpeg_start(s);
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

/* An encode in flight is host state */
static const VMStateDescription vmstate_ingenic_jpeg = {
    .name = TYPE_INGENIC_JPEG,
    .unmigratable = 1,
};

static void ingenic_jpeg_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic JPEG engine";
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
