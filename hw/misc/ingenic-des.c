/*
 * Ingenic DES engine
 *
 * DES and triple DES (EDE), ECB and CBC, fed through 16-word DIN and DOUT
 * FIFOs. Nothing documents the block beyond its feature list; the
 * registers are those of the vendor jz-des test driver, and the rest is as
 * measured on the T31 and T41:
 *  - CR2.BE set takes key, IV and data words big-endian, the first byte
 *    in bits 31:24; clear, each word is byte-swapped. Keys and IV are
 *    converted as they are written; keys read 0, the IV reads back in the
 *    engine's (big-endian) order.
 *  - Setting EN empties the FIFOs and loads the CBC chain from the IV;
 *    clearing EN zeroes the keys (the IV stays). DIN writes with EN clear
 *    are dropped, and so are IV writes with EN set.
 *  - A block is done as soon as its second word is in; a finished block
 *    waits for room in DOUT, holding one more block of DIN back.
 *  - SR: bit 0 DOUT empty, 1 DOUT at least half full, 2 DOUT without room
 *    for a block, 3 no whole block in DIN, 4 DIN at most half full.
 *  - DOUT reads 0 when empty.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "hw/core/irq.h"
#include "hw/misc/ingenic-des.h"
#include "migration/vmstate.h"

#define DES_CR1         0x00
#define DES_CR2         0x04
#define DES_SR          0x08
#define DES_KEY         0x10    /* K1L, K1R, K2L, K2R, K3L, K3R */
#define DES_IVL         0x28
#define DES_IVR         0x2c
#define DES_DIN         0x30
#define DES_DOUT        0x34
#define DES_IOSIZE      0x1000

#define CR1_EN          BIT(0)
#define CR2_DMAE        BIT(0)
#define CR2_TDES        BIT(1)
#define CR2_CBC         BIT(2)
#define CR2_DEC         BIT(3)
#define CR2_BE          BIT(4)
#define CR2_MASK        0x1f

#define SR_OUT_EMPTY    BIT(0)
#define SR_OUT_HALF     BIT(1)
#define SR_OUT_FULL     BIT(2)
#define SR_IN_NOBLOCK   BIT(3)
#define SR_IN_HALF      BIT(4)

/* FIPS 46-3; table entries count bits from 1 at the most significant */
static const uint8_t des_ip[64] = {
    58, 50, 42, 34, 26, 18, 10, 2, 60, 52, 44, 36, 28, 20, 12, 4,
    62, 54, 46, 38, 30, 22, 14, 6, 64, 56, 48, 40, 32, 24, 16, 8,
    57, 49, 41, 33, 25, 17, 9, 1, 59, 51, 43, 35, 27, 19, 11, 3,
    61, 53, 45, 37, 29, 21, 13, 5, 63, 55, 47, 39, 31, 23, 15, 7,
};

static const uint8_t des_fp[64] = {
    40, 8, 48, 16, 56, 24, 64, 32, 39, 7, 47, 15, 55, 23, 63, 31,
    38, 6, 46, 14, 54, 22, 62, 30, 37, 5, 45, 13, 53, 21, 61, 29,
    36, 4, 44, 12, 52, 20, 60, 28, 35, 3, 43, 11, 51, 19, 59, 27,
    34, 2, 42, 10, 50, 18, 58, 26, 33, 1, 41, 9, 49, 17, 57, 25,
};

static const uint8_t des_e[48] = {
    32, 1, 2, 3, 4, 5, 4, 5, 6, 7, 8, 9,
    8, 9, 10, 11, 12, 13, 12, 13, 14, 15, 16, 17,
    16, 17, 18, 19, 20, 21, 20, 21, 22, 23, 24, 25,
    24, 25, 26, 27, 28, 29, 28, 29, 30, 31, 32, 1,
};

static const uint8_t des_p[32] = {
    16, 7, 20, 21, 29, 12, 28, 17, 1, 15, 23, 26, 5, 18, 31, 10,
    2, 8, 24, 14, 32, 27, 3, 9, 19, 13, 30, 6, 22, 11, 4, 25,
};

static const uint8_t des_pc1[56] = {
    57, 49, 41, 33, 25, 17, 9, 1, 58, 50, 42, 34, 26, 18,
    10, 2, 59, 51, 43, 35, 27, 19, 11, 3, 60, 52, 44, 36,
    63, 55, 47, 39, 31, 23, 15, 7, 62, 54, 46, 38, 30, 22,
    14, 6, 61, 53, 45, 37, 29, 21, 13, 5, 28, 20, 12, 4,
};

static const uint8_t des_pc2[48] = {
    14, 17, 11, 24, 1, 5, 3, 28, 15, 6, 21, 10,
    23, 19, 12, 4, 26, 8, 16, 7, 27, 20, 13, 2,
    41, 52, 31, 37, 47, 55, 30, 40, 51, 45, 33, 48,
    44, 49, 39, 56, 34, 53, 46, 42, 50, 36, 29, 32,
};

static const uint8_t des_shifts[16] = {
    1, 1, 2, 2, 2, 2, 2, 2, 1, 2, 2, 2, 2, 2, 2, 1,
};

static const uint8_t des_sbox[8][64] = {
    {
        14, 4, 13, 1, 2, 15, 11, 8, 3, 10, 6, 12, 5, 9, 0, 7,
        0, 15, 7, 4, 14, 2, 13, 1, 10, 6, 12, 11, 9, 5, 3, 8,
        4, 1, 14, 8, 13, 6, 2, 11, 15, 12, 9, 7, 3, 10, 5, 0,
        15, 12, 8, 2, 4, 9, 1, 7, 5, 11, 3, 14, 10, 0, 6, 13,
    },
    {
        15, 1, 8, 14, 6, 11, 3, 4, 9, 7, 2, 13, 12, 0, 5, 10,
        3, 13, 4, 7, 15, 2, 8, 14, 12, 0, 1, 10, 6, 9, 11, 5,
        0, 14, 7, 11, 10, 4, 13, 1, 5, 8, 12, 6, 9, 3, 2, 15,
        13, 8, 10, 1, 3, 15, 4, 2, 11, 6, 7, 12, 0, 5, 14, 9,
    },
    {
        10, 0, 9, 14, 6, 3, 15, 5, 1, 13, 12, 7, 11, 4, 2, 8,
        13, 7, 0, 9, 3, 4, 6, 10, 2, 8, 5, 14, 12, 11, 15, 1,
        13, 6, 4, 9, 8, 15, 3, 0, 11, 1, 2, 12, 5, 10, 14, 7,
        1, 10, 13, 0, 6, 9, 8, 7, 4, 15, 14, 3, 11, 5, 2, 12,
    },
    {
        7, 13, 14, 3, 0, 6, 9, 10, 1, 2, 8, 5, 11, 12, 4, 15,
        13, 8, 11, 5, 6, 15, 0, 3, 4, 7, 2, 12, 1, 10, 14, 9,
        10, 6, 9, 0, 12, 11, 7, 13, 15, 1, 3, 14, 5, 2, 8, 4,
        3, 15, 0, 6, 10, 1, 13, 8, 9, 4, 5, 11, 12, 7, 2, 14,
    },
    {
        2, 12, 4, 1, 7, 10, 11, 6, 8, 5, 3, 15, 13, 0, 14, 9,
        14, 11, 2, 12, 4, 7, 13, 1, 5, 0, 15, 10, 3, 9, 8, 6,
        4, 2, 1, 11, 10, 13, 7, 8, 15, 9, 12, 5, 6, 3, 0, 14,
        11, 8, 12, 7, 1, 14, 2, 13, 6, 15, 0, 9, 10, 4, 5, 3,
    },
    {
        12, 1, 10, 15, 9, 2, 6, 8, 0, 13, 3, 4, 14, 7, 5, 11,
        10, 15, 4, 2, 7, 12, 9, 5, 6, 1, 13, 14, 0, 11, 3, 8,
        9, 14, 15, 5, 2, 8, 12, 3, 7, 0, 4, 10, 1, 13, 11, 6,
        4, 3, 2, 12, 9, 5, 15, 10, 11, 14, 1, 7, 6, 0, 8, 13,
    },
    {
        4, 11, 2, 14, 15, 0, 8, 13, 3, 12, 9, 7, 5, 10, 6, 1,
        13, 0, 11, 7, 4, 9, 1, 10, 14, 3, 5, 12, 2, 15, 8, 6,
        1, 4, 11, 13, 12, 3, 7, 14, 10, 15, 6, 8, 0, 5, 9, 2,
        6, 11, 13, 8, 1, 4, 10, 7, 9, 5, 0, 15, 14, 2, 3, 12,
    },
    {
        13, 2, 8, 4, 6, 15, 11, 1, 10, 9, 3, 14, 5, 0, 12, 7,
        1, 15, 13, 8, 10, 3, 7, 4, 12, 5, 6, 11, 0, 14, 9, 2,
        7, 11, 4, 1, 9, 12, 14, 2, 0, 6, 10, 13, 15, 3, 5, 8,
        2, 1, 14, 7, 4, 10, 8, 13, 15, 12, 9, 0, 3, 5, 6, 11,
    },
};

static uint64_t des_perm(uint64_t v, const uint8_t *t, int n, int nin)
{
    uint64_t o = 0;

    for (int i = 0; i < n; i++) {
        o = o << 1 | ((v >> (nin - t[i])) & 1);
    }
    return o;
}

static uint32_t des_f(uint32_t r, uint64_t k)
{
    uint64_t e = des_perm(r, des_e, 48, 32) ^ k;
    uint32_t o = 0;

    for (int i = 0; i < 8; i++) {
        unsigned six = (e >> (42 - 6 * i)) & 0x3f;
        unsigned row = ((six >> 4) & 2) | (six & 1);

        o = o << 4 | des_sbox[i][row * 16 + ((six >> 1) & 0xf)];
    }
    return des_perm(o, des_p, 32, 32);
}

static uint64_t des_one(uint64_t v, uint64_t key, bool dec)
{
    uint64_t cd = des_perm(key, des_pc1, 56, 64), sk[16], ip;
    uint32_t c = cd >> 28, d = cd & 0xfffffff, l, r;

    for (int i = 0; i < 16; i++) {
        int n = des_shifts[i];

        c = ((c << n) | (c >> (28 - n))) & 0xfffffff;
        d = ((d << n) | (d >> (28 - n))) & 0xfffffff;
        sk[i] = des_perm((uint64_t)c << 28 | d, des_pc2, 48, 56);
    }
    ip = des_perm(v, des_ip, 64, 64);
    l = ip >> 32;
    r = ip;
    for (int i = 0; i < 16; i++) {
        uint32_t t = r;

        r = l ^ des_f(r, sk[dec ? 15 - i : i]);
        l = t;
    }
    return des_perm((uint64_t)r << 32 | l, des_fp, 64, 64);
}

static uint64_t des_key(IngenicDesState *s, int n)
{
    return (uint64_t)s->key[2 * n] << 32 | s->key[2 * n + 1];
}

static uint64_t des_cipher(IngenicDesState *s, uint64_t v)
{
    bool dec = s->cr2 & CR2_DEC;

    if (!(s->cr2 & CR2_TDES)) {
        return des_one(v, des_key(s, 0), dec);
    }
    if (dec) {
        v = des_one(v, des_key(s, 2), true);
        v = des_one(v, des_key(s, 1), false);
        return des_one(v, des_key(s, 0), true);
    }
    v = des_one(v, des_key(s, 0), false);
    v = des_one(v, des_key(s, 1), true);
    return des_one(v, des_key(s, 2), false);
}

/* A register word as the engine takes it, by CR2.BE */
static uint32_t des_word(IngenicDesState *s, uint32_t v)
{
    return s->cr2 & CR2_BE ? v : bswap32(v);
}

static void des_update(IngenicDesState *s)
{
    bool on = (s->cr1 & CR1_EN) && (s->cr2 & CR2_DMAE);

    qemu_set_irq(s->dreq[0], on && s->in_n <= INGENIC_DES_FIFO / 2);
    qemu_set_irq(s->dreq[1], on && s->out_n);
}

/* Move finished blocks out and DIN blocks through, as room allows */
static void des_run(IngenicDesState *s)
{
    for (;;) {
        uint64_t v, o;

        if (s->held) {
            if (INGENIC_DES_FIFO - s->out_n < 2) {
                return;
            }
            for (int i = 0; i < 2; i++) {
                s->out[(s->out_rd + s->out_n++) % INGENIC_DES_FIFO] =
                    s->hold[i];
            }
            s->held = false;
        }
        if (s->in_n < 2) {
            return;
        }
        v = (uint64_t)s->in[s->in_rd] << 32 |
            s->in[(s->in_rd + 1) % INGENIC_DES_FIFO];
        s->in_rd = (s->in_rd + 2) % INGENIC_DES_FIFO;
        s->in_n -= 2;
        if (s->cr2 & CR2_CBC) {
            if (s->cr2 & CR2_DEC) {
                o = des_cipher(s, v) ^ s->chain;
                s->chain = v;
            } else {
                o = des_cipher(s, v ^ s->chain);
                s->chain = o;
            }
        } else {
            o = des_cipher(s, v);
        }
        s->hold[0] = des_word(s, o >> 32);
        s->hold[1] = des_word(s, o);
        s->held = true;
    }
}

static uint32_t des_sr(IngenicDesState *s)
{
    uint32_t sr = 0;

    sr |= s->out_n ? 0 : SR_OUT_EMPTY;
    sr |= s->out_n >= INGENIC_DES_FIFO / 2 ? SR_OUT_HALF : 0;
    sr |= INGENIC_DES_FIFO - s->out_n < 2 ? SR_OUT_FULL : 0;
    sr |= s->in_n < 2 ? SR_IN_NOBLOCK : 0;
    sr |= s->in_n <= INGENIC_DES_FIFO / 2 ? SR_IN_HALF : 0;
    return sr;
}

static uint64_t ingenic_des_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicDesState *s = opaque;
    uint32_t v = 0;

    switch (offset) {
    case DES_CR1:
        return s->cr1;
    case DES_CR2:
        return s->cr2;
    case DES_SR:
        return des_sr(s);
    case DES_IVL:
        return s->iv[0];
    case DES_IVR:
        return s->iv[1];
    case DES_DOUT:
        if (s->out_n) {
            v = s->out[s->out_rd];
            s->out_rd = (s->out_rd + 1) % INGENIC_DES_FIFO;
            s->out_n--;
            des_run(s);
            des_update(s);
        }
        return v;
    case 0x0c:
    case DES_KEY ... DES_KEY + 0x14:
    case DES_DIN:
    case 0x38:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-des: read of 0x%"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void ingenic_des_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IngenicDesState *s = opaque;
    uint32_t v = value;

    switch (offset) {
    case DES_CR1:
        if ((v & CR1_EN) && !(s->cr1 & CR1_EN)) {
            s->in_rd = s->in_n = s->out_rd = s->out_n = 0;
            s->held = false;
            s->chain = (uint64_t)s->iv[0] << 32 | s->iv[1];
        } else if (!(v & CR1_EN) && (s->cr1 & CR1_EN)) {
            memset(s->key, 0, sizeof(s->key));
        }
        s->cr1 = v & CR1_EN;
        break;
    case DES_CR2:
        s->cr2 = v & CR2_MASK;
        break;
    case DES_SR:
    case DES_DOUT:
        break;
    case DES_KEY ... DES_KEY + 0x14:
        if (!(offset & 3)) {
            s->key[(offset - DES_KEY) / 4] = des_word(s, v);
        }
        break;
    case DES_IVL:
    case DES_IVR:
        if (!(s->cr1 & CR1_EN)) {
            s->iv[offset == DES_IVR] = des_word(s, v);
        }
        break;
    case DES_DIN:
        if ((s->cr1 & CR1_EN) && s->in_n < INGENIC_DES_FIFO) {
            s->in[(s->in_rd + s->in_n++) % INGENIC_DES_FIFO] = des_word(s, v);
            des_run(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-des: write of 0x%x to 0x%"
                      HWADDR_PRIx "\n", v, offset);
        return;
    }
    des_update(s);
}

static const MemoryRegionOps ingenic_des_ops = {
    .read = ingenic_des_read,
    .write = ingenic_des_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_des_reset_hold(Object *obj, ResetType type)
{
    IngenicDesState *s = INGENIC_DES(obj);

    s->cr1 = s->cr2 = 0;
    memset(s->key, 0, sizeof(s->key));
    s->in_rd = s->in_n = s->out_rd = s->out_n = 0;
    s->held = false;
    s->chain = 0;
}

static void ingenic_des_reset_exit(Object *obj, ResetType type)
{
    des_update(INGENIC_DES(obj));
}

static void ingenic_des_init(Object *obj)
{
    IngenicDesState *s = INGENIC_DES(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_des_ops, s,
                          TYPE_INGENIC_DES, DES_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), s->dreq, "dreq", 2);
}

static const VMStateDescription vmstate_ingenic_des = {
    .name = TYPE_INGENIC_DES,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr1, IngenicDesState),
        VMSTATE_UINT32(cr2, IngenicDesState),
        VMSTATE_UINT32_ARRAY(key, IngenicDesState, 6),
        VMSTATE_UINT32_ARRAY(iv, IngenicDesState, 2),
        VMSTATE_UINT32_ARRAY(in, IngenicDesState, INGENIC_DES_FIFO),
        VMSTATE_UINT32_ARRAY(out, IngenicDesState, INGENIC_DES_FIFO),
        VMSTATE_UINT32(in_rd, IngenicDesState),
        VMSTATE_UINT32(in_n, IngenicDesState),
        VMSTATE_UINT32(out_rd, IngenicDesState),
        VMSTATE_UINT32(out_n, IngenicDesState),
        VMSTATE_UINT32_ARRAY(hold, IngenicDesState, 2),
        VMSTATE_BOOL(held, IngenicDesState),
        VMSTATE_UINT64(chain, IngenicDesState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_des_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_des;
    rc->phases.hold = ingenic_des_reset_hold;
    rc->phases.exit = ingenic_des_reset_exit;
}

static const TypeInfo ingenic_des_info = {
    .name          = TYPE_INGENIC_DES,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDesState),
    .instance_init = ingenic_des_init,
    .class_init    = ingenic_des_class_init,
};

static void ingenic_des_register_types(void)
{
    type_register_static(&ingenic_des_info);
}

type_init(ingenic_des_register_types)
