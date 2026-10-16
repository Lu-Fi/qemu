/*
 * Ingenic AES engine
 *
 * ECB and CBC with 128, 192 and 256-bit keys, a block at a time through
 * the ASDI/ASDO FIFOs or ASTC blocks by the engine's own DMA. Register
 * words hold the cipher's bytes most significant first. DMA moves memory
 * as little-endian words read the same way, so each four bytes in memory
 * are reversed against the cipher's byte stream. ASCR.EDIN takes memory in
 * byte order instead: on the T32 for both directions (measured), on the
 * T33 for input only, with EDOUT for the output.
 *
 * As measured on the T31 and T41:
 *  - ASCR resets to 0x30000000. PS, DMAE, KEYL, MODE, DECE and EN hold
 *    their values; the action bits (INIT_IV, KEYS, AESS, DMAS, CLR) read 0.
 *  - Writing ASSR toggles the bits written. Drivers only write back bits
 *    they saw set, which clears them.
 *  - One key expansion serves both directions, and CLR leaves it in
 *    place; CLR empties the FIFOs and the IV.
 *  - 192 and 256-bit keys work, despite the SDK's "128 only".
 *  - CBC chains across blocks and across DMA starts, from INIT_IV on.
 *  - ASTC counts 16-byte blocks (the 4.4 driver's length / 8 asks for
 *    twice the data); ASSA, ASDA and ASTC stay as written.
 *  - A DMA completion sets DMAD and AESD.
 *  - Reading ASDO past the fourth word returns the fourth again.
 * The T33 layout (the "t33" property) follows its vendor driver and U-Boot
 * and is not checked against silicon: the data registers move up and the
 * DMA count becomes ASBNUM blocks, ASSR/ASINTM lose KEYD (AESD is bit 0,
 * DMAD bit 1).
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qom/object.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-aes.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"

#define AES_IOSIZE      0x10000

#define ASCR_EN         BIT(0)
#define ASCR_INIT_IV    BIT(1)
#define ASCR_KEYS       BIT(2)
#define ASCR_AESS       BIT(3)
#define ASCR_DECE       BIT(4)
#define ASCR_MODE       BIT(5)
#define ASCR_KEYL(v)    extract32(v, 6, 2)
#define ASCR_DMAE       BIT(8)
#define ASCR_DMAS       BIT(9)
#define ASCR_CLR        BIT(10)
#define ASCR_EDIN       BIT(12)
#define ASCR_EDOUT      BIT(13)
#define ASCR_KEY_MODE   (0xfu << 16)    /* T33: eFuse keys */
#define ASCR_MASK       0xf00001f1
#define ASCR_RESET      0x30000000

/* Larger DMA requests are cut here: 64 MiB */
#define AES_DMA_MAX     (1u << 22)

enum {
    R_NONE, R_ASCR, R_ASSR, R_ASINTM, R_ASSA, R_ASDA, R_ASTC, R_ASSTC,
    R_ASDTC, R_ASDI, R_ASDO, R_ASKY, R_ASIV,
};

static const uint8_t aes_regs[2][12] = {
    { R_ASCR, R_ASSR, R_ASINTM, R_ASSA, R_ASDA, R_ASTC, R_ASDI, R_ASDO,
      R_ASKY, R_ASIV, R_NONE, R_NONE },
    { R_ASCR, R_ASSR, R_ASINTM, R_ASSA, R_ASSTC, R_ASDA, R_ASDTC, R_ASTC,
      R_ASDI, R_ASDO, R_ASKY, R_ASIV },
};

static int aes_reg(IngenicAesState *s, hwaddr offset)
{
    return offset < 0x30 ? aes_regs[s->t33][offset / 4] : R_NONE;
}

/* ASSR bits: KEYD, AESD, DMAD; the T33 has no KEYD */
static uint32_t aes_keyd(IngenicAesState *s)
{
    return s->t33 ? 0 : BIT(0);
}

static uint32_t aes_aesd(IngenicAesState *s)
{
    return s->t33 ? BIT(0) : BIT(1);
}

static uint32_t aes_dmad(IngenicAesState *s)
{
    return s->t33 ? BIT(1) : BIT(2);
}

static uint32_t aes_sr_mask(IngenicAesState *s)
{
    return aes_keyd(s) | aes_aesd(s) | aes_dmad(s);
}

static void aes_update(IngenicAesState *s)
{
    qemu_set_irq(s->irq, !!(s->assr & s->asintm & aes_sr_mask(s)));
}

static void aes_word_bytes(uint32_t w, uint8_t *b)
{
    stl_be_p(b, w);
}

static void aes_expand(IngenicAesState *s)
{
    static const int bits[4] = { 128, 192, 256, 256 };
    int nk;

    s->ukey_bits = bits[ASCR_KEYL(s->ascr)];
    nk = s->ukey_bits / 32;
    for (int i = 0; i < nk; i++) {
        aes_word_bytes(s->key[i], s->ukey + 4 * i);
    }
    AES_set_encrypt_key(s->ukey, s->ukey_bits, &s->ekey);
    AES_set_decrypt_key(s->ukey, s->ukey_bits, &s->dkey);
    s->key_n = 0;
    s->assr |= aes_keyd(s);
}

static void aes_block(IngenicAesState *s, const uint8_t *in, uint8_t *out)
{
    uint8_t t[16];

    if (s->ascr & ASCR_DECE) {
        AES_decrypt(in, t, &s->dkey);
        if (s->ascr & ASCR_MODE) {
            for (int i = 0; i < 16; i++) {
                t[i] ^= s->chain[i];
            }
            memcpy(s->chain, in, 16);
        }
    } else {
        memcpy(t, in, 16);
        if (s->ascr & ASCR_MODE) {
            for (int i = 0; i < 16; i++) {
                t[i] ^= s->chain[i];
            }
        }
        AES_encrypt(t, t, &s->ekey);
        if (s->ascr & ASCR_MODE) {
            memcpy(s->chain, t, 16);
        }
    }
    memcpy(out, t, 16);
}

/* One block from ASDI to ASDO */
static void aes_pio(IngenicAesState *s)
{
    uint8_t in[16], out[16];

    for (int i = 0; i < 4; i++) {
        aes_word_bytes(s->di[i], in + 4 * i);
    }
    aes_block(s, in, out);
    for (int i = 0; i < 4; i++) {
        s->dout[i] = ldl_be_p(out + 4 * i);
    }
    s->di_n = 0;
    s->do_rd = 0;
    s->do_n = 4;
    s->assr |= aes_aesd(s);
}

/* Memory to cipher bytes and back: each 4 bytes reversed, unless EDIN/EDOUT */
static void aes_swap_words(uint8_t *b, bool swap)
{
    if (swap) {
        for (int i = 0; i < 16; i += 4) {
            uint32_t w = ldl_le_p(b + i);

            stl_be_p(b + i, w);
        }
    }
}

static void aes_dma(IngenicAesState *s)
{
    uint32_t n = s->tc;

    if (n > AES_DMA_MAX) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aes: %u blocks cut to %u\n",
                      n, AES_DMA_MAX);
        n = AES_DMA_MAX;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint8_t b[16];

        address_space_read(&address_space_memory, s->sa + 16 * (hwaddr)i,
                           MEMTXATTRS_UNSPECIFIED, b, 16);
        aes_swap_words(b, !(s->ascr & ASCR_EDIN));
        aes_block(s, b, b);
        aes_swap_words(b, !(s->ascr & (s->edout ? ASCR_EDOUT : ASCR_EDIN)));
        address_space_write(&address_space_memory, s->da + 16 * (hwaddr)i,
                            MEMTXATTRS_UNSPECIFIED, b, 16);
    }
    s->assr |= aes_dmad(s) | aes_aesd(s);
}

static void aes_clear(IngenicAesState *s)
{
    s->key_n = s->iv_n = s->di_n = 0;
    s->do_rd = s->do_n = s->do_last = 0;
    memset(s->iv, 0, sizeof(s->iv));
    memset(s->chain, 0, sizeof(s->chain));
}

static void aes_write_ascr(IngenicAesState *s, uint32_t v)
{
    uint32_t mask = ASCR_MASK;

    if (s->edin) {
        mask |= ASCR_EDIN;
    }
    if (s->edout) {
        mask |= ASCR_EDOUT;
    }
    if (s->t33) {
        mask |= ASCR_KEY_MODE;
        if (v & ASCR_KEY_MODE) {
            qemu_log_mask(LOG_UNIMP, "ingenic-aes: eFuse keys are not "
                          "modelled, using the key written\n");
        }
    }
    s->ascr = v & mask;
    if (v & ASCR_CLR) {
        aes_clear(s);
    }
    if (!(s->ascr & ASCR_EN)) {
        return;
    }
    if (v & ASCR_INIT_IV) {
        for (int i = 0; i < 4; i++) {
            aes_word_bytes(s->iv[i], s->chain + 4 * i);
        }
        s->iv_n = 0;
    }
    if (v & ASCR_KEYS) {
        aes_expand(s);
    }
    if ((v & ASCR_AESS) && !(s->ascr & ASCR_DMAE)) {
        aes_pio(s);
    }
    if ((v & ASCR_DMAS) && (s->ascr & ASCR_DMAE)) {
        aes_dma(s);
    }
}

static uint64_t ingenic_aes_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicAesState *s = opaque;

    switch (aes_reg(s, offset)) {
    case R_ASCR:
        return s->ascr;
    case R_ASSR:
        return s->assr;
    case R_ASINTM:
        return s->asintm;
    case R_ASSA:
        return s->sa;
    case R_ASDA:
        return s->da;
    case R_ASTC:
        return s->tc;
    case R_ASSTC:
        return s->stc;
    case R_ASDTC:
        return s->dtc;
    case R_ASDO:
        if (s->do_n) {
            s->do_last = s->dout[s->do_rd++];
            s->do_n--;
        }
        return s->do_last;
    case R_ASDI:
    case R_ASKY:
    case R_ASIV:
        return 0;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aes: read of 0x%"
                      HWADDR_PRIx "\n", offset);
        return 0;
    }
}

static void ingenic_aes_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IngenicAesState *s = opaque;
    uint32_t v = value;

    switch (aes_reg(s, offset)) {
    case R_ASCR:
        aes_write_ascr(s, v);
        break;
    case R_ASSR:
        s->assr ^= v & aes_sr_mask(s);
        break;
    case R_ASINTM:
        s->asintm = v & aes_sr_mask(s);
        break;
    case R_ASSA:
        s->sa = v;
        break;
    case R_ASDA:
        s->da = v;
        break;
    case R_ASTC:
        s->tc = v;
        break;
    case R_ASSTC:
        s->stc = v;
        break;
    case R_ASDTC:
        s->dtc = v;
        break;
    case R_ASDI:
        s->di[s->di_n++ % 4] = v;
        break;
    case R_ASKY:
        s->key[s->key_n++ % 8] = v;
        break;
    case R_ASIV:
        s->iv[s->iv_n++ % 4] = v;
        break;
    case R_ASDO:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-aes: write of 0x%x to 0x%"
                      HWADDR_PRIx "\n", v, offset);
        return;
    }
    aes_update(s);
}

static const MemoryRegionOps ingenic_aes_ops = {
    .read = ingenic_aes_read,
    .write = ingenic_aes_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_aes_reset_hold(Object *obj, ResetType type)
{
    IngenicAesState *s = INGENIC_AES(obj);

    s->ascr = ASCR_RESET;
    s->assr = s->asintm = 0;
    s->sa = s->da = s->tc = s->stc = s->dtc = 0;
    memset(s->key, 0, sizeof(s->key));
    memset(s->di, 0, sizeof(s->di));
    memset(s->dout, 0, sizeof(s->dout));
    aes_clear(s);
    memset(s->ukey, 0, sizeof(s->ukey));
    s->ukey_bits = 128;
    AES_set_encrypt_key(s->ukey, s->ukey_bits, &s->ekey);
    AES_set_decrypt_key(s->ukey, s->ukey_bits, &s->dkey);
}

static void ingenic_aes_reset_exit(Object *obj, ResetType type)
{
    aes_update(INGENIC_AES(obj));
}

static void ingenic_aes_init(Object *obj)
{
    IngenicAesState *s = INGENIC_AES(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_aes_ops, s,
                          TYPE_INGENIC_AES, AES_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static int ingenic_aes_post_load(void *opaque, int version_id)
{
    IngenicAesState *s = opaque;

    if (s->ukey_bits != 128 && s->ukey_bits != 192 && s->ukey_bits != 256) {
        return -EINVAL;
    }
    AES_set_encrypt_key(s->ukey, s->ukey_bits, &s->ekey);
    AES_set_decrypt_key(s->ukey, s->ukey_bits, &s->dkey);
    return 0;
}

static const VMStateDescription vmstate_ingenic_aes = {
    .name = TYPE_INGENIC_AES,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = ingenic_aes_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ascr, IngenicAesState),
        VMSTATE_UINT32(assr, IngenicAesState),
        VMSTATE_UINT32(asintm, IngenicAesState),
        VMSTATE_UINT32(sa, IngenicAesState),
        VMSTATE_UINT32(da, IngenicAesState),
        VMSTATE_UINT32(tc, IngenicAesState),
        VMSTATE_UINT32(stc, IngenicAesState),
        VMSTATE_UINT32(dtc, IngenicAesState),
        VMSTATE_UINT32_ARRAY(key, IngenicAesState, 8),
        VMSTATE_UINT32_ARRAY(iv, IngenicAesState, 4),
        VMSTATE_UINT32_ARRAY(di, IngenicAesState, 4),
        VMSTATE_UINT32_ARRAY(dout, IngenicAesState, 4),
        VMSTATE_UINT32(key_n, IngenicAesState),
        VMSTATE_UINT32(iv_n, IngenicAesState),
        VMSTATE_UINT32(di_n, IngenicAesState),
        VMSTATE_UINT32(do_rd, IngenicAesState),
        VMSTATE_UINT32(do_n, IngenicAesState),
        VMSTATE_UINT32(do_last, IngenicAesState),
        VMSTATE_UINT8_ARRAY(chain, IngenicAesState, 16),
        VMSTATE_UINT8_ARRAY(ukey, IngenicAesState, 32),
        VMSTATE_UINT32(ukey_bits, IngenicAesState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_aes_props[] = {
    DEFINE_PROP_BOOL("edin", IngenicAesState, edin, false),
    DEFINE_PROP_BOOL("edout", IngenicAesState, edout, false),
    DEFINE_PROP_BOOL("t33", IngenicAesState, t33, false),
};

static void ingenic_aes_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_aes;
    device_class_set_props(dc, ingenic_aes_props);
    rc->phases.hold = ingenic_aes_reset_hold;
    rc->phases.exit = ingenic_aes_reset_exit;
}

static const TypeInfo ingenic_aes_info = {
    .name          = TYPE_INGENIC_AES,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAesState),
    .instance_init = ingenic_aes_init,
    .class_init    = ingenic_aes_class_init,
};

static void ingenic_aes_register_types(void)
{
    type_register_static(&ingenic_aes_info);
}

type_init(ingenic_aes_register_types)
