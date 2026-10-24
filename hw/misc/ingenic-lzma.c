/*
 * Ingenic hardware LZMA decompressor (JZLZMA)
 *
 * The T32 and T33 carry two, at 0x131f0000 and 0x131f8000, the T40 one
 * inside its BSCALER block at 0x13090000, the T41 two at 0x132c0000 and
 * 0x132e0000; the A1 has none. Each unpacks a [4B LE dict size][4B LE
 * uncompressed size][stream] blob from memory to memory, as made by
 * Ingenic's lzma tool (thingino-linux scripts/ingenic_hardlzma).
 *
 * The stream uses LZMA's literal/match/rep grammar and distance model,
 * but it is plain bit-packed rather than range-coded, which is what
 * makes it decodable in hardware (and why no stock LZMA decoder will
 * touch it). Bits are consumed LSB-first; most fields are then bit
 * reversed (jz_readnum), except the posSlot direct bits and the final
 * align bits, which are read straight (jz_readraw).
 *
 * Each chip has its own register map ("model"):
 *   t32  CTRL (0xc000 from reset) keeps bits 31 and 7 once written 1,
 *        bit 0 starts a run, which clears bit 14 and sets bit 3, and bit
 *        1 resets the engine: CTRL to 0xc000 but bits 31 and 7, the
 *        timeout (0x10) to all ones and the out length (0x14, read-only)
 *        to 0. SRC 0x04, LEN 0x08, DST 0x0c, version 0x58. As both of a
 *        T32LQ's engines do, the second at 0x131f8000.
 *   t40  CTRL bit 31 picks LZMA over BSCALER and flips when written 1
 *        (writing 0 leaves it), bit 2 reset and bit 1 RAM init clear
 *        themselves, bit 3 masks the interrupt, bit 0 starts. BS_BASE
 *        0x04, BS_SIZE 0x08, DST_BASE 0x0c, ICRC 0x10, OCRC 0x14,
 *        TIMEOUT 0x18, out length 0x1c.
 *   t41  CTRL bit 31 as the T40, bit 1 reset, bit 0 start; a finished
 *        run reads 0x80004008. BS_BASE, BS_SIZE, DST_BASE, TIMEOUT
 *        0x10, out length 0x14.
 * A run finishes within the write that starts it. The interrupt is not
 * modelled (the vendor drivers poll bit 0), nor is the BSCALER side.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "migration/vmstate.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-lzma.h"
#include "system/physmem.h"

enum { LZMA_T32, LZMA_T40, LZMA_T41 };

#define CTRL                0x00
#define BS_BASE             0x04
#define BS_SIZE             0x08
#define DST_BASE            0x0c
#define T32_OUT_LEN         0x14
#define T32_VERSION         0x58
#define T40_OUT_LEN         0x1c
#define T41_OUT_LEN         0x14

#define CTRL_START          (1u << 0)
#define T32_CTRL_RESET      (1u << 1)
#define T32_CTRL_DONE       (1u << 3)
#define T32_CTRL_KEEP       ((1u << 31) | (1u << 7))
#define T32_CTRL_IDLE       0xc000
#define T32_CTRL_READY      (1u << 14)
#define T32_TIMEOUT         0x10
#define CTRL_LZMA_MODE      (1u << 31)
#define T32_VERSION_VALUE   0x20231219
#define T41_CTRL_DONE       0x4008

#define JZ_START_POS_MODEL_INDEX 4
#define JZ_END_POS_MODEL_INDEX   14
#define JZ_NUM_ALIGN_BITS        4

typedef struct {
    const uint8_t *data;
    size_t len, pos;
    uint64_t buf;
    unsigned nbits;
    bool eof;
} JzBits;

static uint32_t jz_rev(uint32_t v, unsigned k)
{
    uint32_t r = 0;
    while (k--) {
        r = (r << 1) | (v & 1);
        v >>= 1;
    }
    return r;
}

static void jz_fill(JzBits *b, unsigned k)
{
    while (b->nbits < k) {
        if (b->pos >= b->len) {
            b->eof = true;
            return;
        }
        b->buf |= (uint64_t)b->data[b->pos++] << b->nbits;
        b->nbits += 8;
    }
}

static uint32_t jz_readraw(JzBits *b, unsigned k)
{
    uint32_t v;

    if (k == 0) {
        return 0;
    }
    jz_fill(b, k);
    if (b->eof) {
        return 0;
    }
    v = b->buf & ((1u << k) - 1);
    b->buf >>= k;
    b->nbits -= k;
    return v;
}

static uint32_t jz_readnum(JzBits *b, unsigned k)
{
    return k ? jz_rev(jz_readraw(b, k), k) : 0;
}

static uint32_t jz_getbit(JzBits *b)
{
    return jz_readraw(b, 1);
}

static uint32_t jz_decode_length(JzBits *b)
{
    if (jz_getbit(b) == 0) {
        return jz_readnum(b, 3) + 2;
    }
    if (jz_getbit(b) == 0) {
        return jz_readnum(b, 3) + 10;
    }
    return jz_readnum(b, 8) + 18;
}

static uint32_t jz_decode_dist(JzBits *b)
{
    uint32_t pos_slot = jz_readnum(b, 6);
    unsigned direct;
    uint32_t pos;

    if (pos_slot < JZ_START_POS_MODEL_INDEX) {
        return pos_slot;
    }
    direct = (pos_slot >> 1) - 1;
    pos = (2 | (pos_slot & 1)) << direct;
    if (pos_slot < JZ_END_POS_MODEL_INDEX) {
        pos += jz_readraw(b, direct);
    } else {
        pos += jz_readnum(b, direct - JZ_NUM_ALIGN_BITS) << JZ_NUM_ALIGN_BITS;
        pos += jz_readraw(b, JZ_NUM_ALIGN_BITS);
    }
    return pos;
}

static size_t jz_lzma_decompress(const uint8_t *in, size_t in_len,
                                 uint8_t *out, size_t out_max)
{
    JzBits b = { .data = in, .len = in_len };
    uint32_t reps[4] = { 0, 0, 0, 0 };
    size_t n = 0;

    while (n < out_max && !b.eof) {
        uint32_t size = 0;
        size_t start;

        if (jz_getbit(&b) == 0) {                 /* literal */
            out[n++] = jz_rev(jz_readraw(&b, 8), 8);
            continue;
        }

        if (jz_getbit(&b) == 0) {                 /* new distance */
            size = jz_decode_length(&b);
            reps[3] = reps[2]; reps[2] = reps[1]; reps[1] = reps[0];
            reps[0] = jz_decode_dist(&b);
        } else if (jz_getbit(&b) == 0) {
            if (jz_getbit(&b) == 0) {
                size = 1;                         /* short rep0 */
            }
        } else {
            uint32_t t;
            if (jz_getbit(&b) == 0) {             /* rep1 */
                t = reps[1];
                reps[1] = reps[0];
            } else if (jz_getbit(&b) == 0) {      /* rep2 */
                t = reps[2];
                reps[2] = reps[1]; reps[1] = reps[0];
            } else {                              /* rep3 */
                t = reps[3];
                reps[3] = reps[2]; reps[2] = reps[1]; reps[1] = reps[0];
            }
            reps[0] = t;
        }

        if (size == 0) {
            size = jz_decode_length(&b);
        }
        if (b.eof) {
            break;
        }

        if (reps[0] + 1 > n) {
            break;                                /* bad back-reference */
        }
        start = n - reps[0] - 1;
        while (size-- && n < out_max) {
            out[n] = out[start++];
            n++;
        }
    }
    return n;
}

static uint32_t lzma_run(IngenicLzmaState *s)
{
    uint32_t src = s->regs[BS_BASE >> 2];
    uint32_t len = s->regs[BS_SIZE >> 2];
    uint32_t dst = s->regs[DST_BASE >> 2];
    uint8_t hdr[8];
    uint32_t dict, uncomp;
    uint8_t *in, *out;
    size_t produced;

    if (len <= sizeof(hdr)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-lzma: %u bytes is no stream\n",
                      len);
        return 0;
    }
    physical_memory_read(src, hdr, sizeof(hdr));
    dict = ldl_le_p(hdr);
    uncomp = ldl_le_p(hdr + 4);
    if (dict < 0x1000 || dict > 0x4000000 || !uncomp || uncomp > 64 * MiB) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-lzma: bad header at 0x%x: "
                      "dict 0x%x size %u\n", src, dict, uncomp);
        return 0;
    }
    in = g_malloc(len - sizeof(hdr));
    out = g_malloc(uncomp);
    physical_memory_read(src + sizeof(hdr), in, len - sizeof(hdr));
    produced = jz_lzma_decompress(in, len - sizeof(hdr), out, uncomp);
    physical_memory_write(dst, out, produced);
    g_free(in);
    g_free(out);
    return produced;
}

static uint64_t lzma_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicLzmaState *s = opaque;

    if (s->kind == LZMA_T32 && offset == T32_VERSION) {
        return T32_VERSION_VALUE;
    }
    return offset < sizeof(s->regs) ? s->regs[offset >> 2] : 0;
}

static void lzma_ctrl_write(IngenicLzmaState *s, uint32_t v)
{
    uint32_t *ctrl = &s->regs[CTRL >> 2];

    switch (s->kind) {
    case LZMA_T32:
        if (v & T32_CTRL_RESET) {
            *ctrl = (*ctrl & T32_CTRL_KEEP) | T32_CTRL_IDLE;
            s->regs[T32_TIMEOUT >> 2] = UINT32_MAX;
            s->regs[T32_OUT_LEN >> 2] = 0;
        }
        *ctrl |= v & T32_CTRL_KEEP;
        if (v & CTRL_START) {
            s->regs[T32_OUT_LEN >> 2] = lzma_run(s);
            *ctrl = (*ctrl & ~T32_CTRL_READY) | T32_CTRL_DONE;
        }
        return;
    case LZMA_T40:
        *ctrl = ((*ctrl ^ (v & CTRL_LZMA_MODE)) & CTRL_LZMA_MODE) |
                (v & (1u << 3));
        if (v & (1u << 2)) {
            s->regs[T40_OUT_LEN >> 2] = 0;
        }
        if (v & CTRL_START) {
            if (*ctrl & CTRL_LZMA_MODE) {
                s->regs[T40_OUT_LEN >> 2] = lzma_run(s);
            } else {
                qemu_log_mask(LOG_UNIMP, "ingenic-lzma: BSCALER mode\n");
            }
        }
        return;
    case LZMA_T41:
        *ctrl = ((*ctrl ^ (v & CTRL_LZMA_MODE)) & CTRL_LZMA_MODE) |
                (*ctrl & T41_CTRL_DONE);
        if (v & (1u << 1)) {
            *ctrl &= ~T41_CTRL_DONE;
            s->regs[T41_OUT_LEN >> 2] = 0;
        }
        if (v & CTRL_START) {
            if (*ctrl & CTRL_LZMA_MODE) {
                s->regs[T41_OUT_LEN >> 2] = lzma_run(s);
                *ctrl |= T41_CTRL_DONE;
            } else {
                qemu_log_mask(LOG_UNIMP, "ingenic-lzma: BSCALER mode\n");
            }
        }
        return;
    }
}

static void lzma_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    IngenicLzmaState *s = opaque;
    hwaddr out_len = s->kind == LZMA_T40 ? T40_OUT_LEN : T41_OUT_LEN;

    if (offset == CTRL) {
        lzma_ctrl_write(s, value);
    } else if (offset < sizeof(s->regs) && offset != out_len) {
        s->regs[offset >> 2] = value;
    }
}

static const MemoryRegionOps lzma_ops = {
    .read = lzma_read,
    .write = lzma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

static void lzma_realize(DeviceState *dev, Error **errp)
{
    IngenicLzmaState *s = INGENIC_LZMA(dev);
    static const char *const models[] = { "t32", "t40", "t41" };

    for (s->kind = 0; s->kind < ARRAY_SIZE(models); s->kind++) {
        if (s->model && !strcmp(s->model, models[s->kind])) {
            break;
        }
    }
    if (s->kind == ARRAY_SIZE(models)) {
        error_setg(errp, "ingenic-lzma: model must be t32, t40 or t41");
        return;
    }
    /* The T32's two engines sit 32 KiB apart */
    memory_region_init_io(&s->iomem, OBJECT(s), &lzma_ops, s,
                          TYPE_INGENIC_LZMA, s->kind == LZMA_T32 ?
                          INGENIC_LZMA_IOSIZE / 2 : INGENIC_LZMA_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
}

static void lzma_reset_hold(Object *obj, ResetType type)
{
    IngenicLzmaState *s = INGENIC_LZMA(obj);

    memset(s->regs, 0, sizeof(s->regs));
    if (s->kind == LZMA_T32) {
        s->regs[CTRL >> 2] = T32_CTRL_IDLE;
        s->regs[T32_TIMEOUT >> 2] = UINT32_MAX;
    }
}

static const VMStateDescription vmstate_lzma = {
    .name = TYPE_INGENIC_LZMA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicLzmaState, 64),
        VMSTATE_END_OF_LIST()
    },
};

static const Property lzma_props[] = {
    DEFINE_PROP_STRING("model", IngenicLzmaState, model),
};

static void lzma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_props(dc, lzma_props);
    dc->realize = lzma_realize;
    dc->vmsd = &vmstate_lzma;
    rc->phases.hold = lzma_reset_hold;
}

static const TypeInfo lzma_type_info = {
    .name = TYPE_INGENIC_LZMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicLzmaState),
    .class_init = lzma_class_init,
};

static void lzma_register_types(void)
{
    type_register_static(&lzma_type_info);
}

type_init(lzma_register_types)
