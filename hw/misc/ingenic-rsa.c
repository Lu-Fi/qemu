/*
 * Ingenic RSA hardware accelerator emulation
 *
 * The T31 bootrom uses this for RSA-2048 signature verification.
 * Register protocol (from ROM analysis):
 *   0x00  control/status (bit 7 = 2048-bit mode, bit 2 = modulus ready,
 *         bit 5 = message ready, bit 9 = result busy). It resets to
 *         0x00030200, busy, as every part reads it; busy clears once an
 *         operation starts (the exponent written, or bit 4 set) and sets
 *         again when bit 0 clears the FIFOs.
 *   0x04  exponent, 64 words most significant first, each write shifting
 *         the rest up (U-Boot writes 63 zeros then e, a private exponent
 *         in full; the bootrom one word after reset); reads the last
 *         word written
 *   0x08  modulus data FIFO (write 64 words for RSA-2048)
 *   0x0C  message data FIFO (write 64 words)
 *   0x10  result data FIFO (read 64 words, reverse order)
 *
 * The T33's (from its U-Boot's jz_rsa_PRJ008; no T33 to check):
 * RSAC bits 10:4 the modulus in words (32 or 64) and bit 0 starting the
 * operation; the message (0x0C) and exponent 64 words each, most
 * significant first; the modulus (0x08) its words, after which RSAINFO
 * (0x18) bit 4 is set; the result at 0x10, least significant word first,
 * with RSAINFO bit 0; RSACLR (0x14) clears either bit.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/bitops.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-rsa.h"
#include "migration/vmstate.h"
#include "trace.h"
#include <gmp.h>

#define RSA_CTRL     0x00
#define RSA_EXP      0x04
#define RSA_MODIN    0x08
#define RSA_MSGIN    0x0C
#define RSA_DOUT     0x10

#define RSA_CTRL_ENABLE (1 << 0)
#define RSA_CTRL_2048   (1 << 7)
#define RSA_CTRL_MODOK  (1 << 2)
#define RSA_CTRL_MSGOK  (1 << 5)
#define RSA_CTRL_MAIN   (1 << 4)    /* start the main process */
#define RSA_CTRL_BUSY   (1 << 9)

#define RSA_CLR         0x14        /* the T33's */
#define RSA_INFO        0x18
#define RSA_INFO_DONE   (1 << 0)
#define RSA_INFO_N_DONE (1 << 4)

/* Shift @value into a 64-word operand, most significant word first */
static void ingenic_rsa_shift(uint32_t *words, uint32_t value)
{
    memmove(words, words + 1, (INGENIC_RSA_MAXWORDS - 1) * sizeof(*words));
    words[INGENIC_RSA_MAXWORDS - 1] = value;
}

static void ingenic_rsa_t33_compute(IngenicRsaState *s)
{
    mpz_t mod, msg, exp, res;
    size_t count = 0;

    mpz_inits(mod, msg, exp, res, NULL);
    mpz_import(mod, s->nwords, 1, 4, 0, 0, s->modulus);
    mpz_import(msg, INGENIC_RSA_MAXWORDS, 1, 4, 0, 0, s->message);
    mpz_import(exp, INGENIC_RSA_MAXWORDS, 1, 4, 0, 0, s->exponent);
    if (mpz_sgn(mod) > 0) {
        mpz_powm(res, msg, exp, mod);
    }
    memset(s->result, 0, sizeof(s->result));
    mpz_export(s->result, &count, -1, 4, 0, 0, res);
    mpz_clears(mod, msg, exp, res, NULL);
    s->res_pos = 0;
    s->info |= RSA_INFO_DONE;
}

static uint64_t ingenic_rsa_t33_read(IngenicRsaState *s, hwaddr offset)
{
    switch (offset) {
    case RSA_CTRL:
        return s->reg_ctrl;
    case RSA_EXP:
        return s->reg_exp;
    case RSA_DOUT:
        return s->res_pos < s->nwords ? s->result[s->res_pos++] : 0;
    case RSA_INFO:
        return s->info;
    default:
        return 0;
    }
}

static void ingenic_rsa_t33_write(IngenicRsaState *s, hwaddr offset,
                                     uint32_t value)
{
    switch (offset) {
    case RSA_CTRL: {
        bool start = (value & RSA_CTRL_ENABLE) &&
                     !(s->reg_ctrl & RSA_CTRL_ENABLE);
        uint32_t nw = extract32(value, 4, 7);

        /* An operation ends with the enable off: the modulus starts over */
        if (!(value & RSA_CTRL_ENABLE) && (s->reg_ctrl & RSA_CTRL_ENABLE)) {
            s->mod_pos = 0;
        }
        s->reg_ctrl = value;
        s->nwords = nw && nw <= INGENIC_RSA_MAXWORDS ? nw
                                                     : INGENIC_RSA_MAXWORDS;
        if (start) {
            ingenic_rsa_t33_compute(s);
        }
        break;
    }
    case RSA_EXP:
        ingenic_rsa_shift(s->exponent, value);
        s->reg_exp = value;
        break;
    case RSA_MODIN:
        if (s->mod_pos >= s->nwords) {
            s->mod_pos = 0;
        }
        s->modulus[s->mod_pos++] = value;
        if (s->mod_pos >= s->nwords) {
            s->info |= RSA_INFO_N_DONE;
        }
        break;
    case RSA_MSGIN:
        ingenic_rsa_shift(s->message, value);
        break;
    case RSA_CLR:
        s->info &= ~(value & (RSA_INFO_DONE | RSA_INFO_N_DONE));
        if (value & RSA_INFO_N_DONE) {
            s->mod_pos = 0;
        }
        break;
    default:
        break;
    }
}

static void ingenic_rsa_compute(IngenicRsaState *s)
{
    mpz_t mod, msg, exp, res;
    uint32_t nw = s->nwords ? s->nwords : INGENIC_RSA_MAXWORDS;

    mpz_init(mod);
    mpz_init(msg);
    mpz_init(exp);
    mpz_init(res);

    mpz_import(mod, nw, 1, 4, 0, 0, s->modulus);
    mpz_import(msg, nw, 1, 4, 0, 0, s->message);
    mpz_import(exp, INGENIC_RSA_MAXWORDS, 1, 4, 0, 0, s->exponent);

    if (mpz_sgn(mod) > 0) {
        mpz_powm(res, msg, exp, mod);
    }

    memset(s->result, 0, sizeof(s->result));
    size_t count = 0;
    /*
     * The Ingenic RSA hardware outputs result words LSW-first via DOUT.
     * The T31 ROM reads [0..63] and stores reversed, so output[0] = MSW.
     * Use LSW-first export so DOUT matches real hardware behavior.
     */
    mpz_export(s->result, &count, -1, 4, 0, 0, res);

    mpz_clear(mod);
    mpz_clear(msg);
    mpz_clear(exp);
    mpz_clear(res);

    s->computed = true;
    s->res_pos = 0;
    s->reg_ctrl &= ~RSA_CTRL_BUSY;
    trace_ingenic_rsa_modexp(nw, s->reg_exp, s->message[0], s->modulus[0],
                             s->result[0]);
}

static uint64_t ingenic_rsa_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicRsaState *s = INGENIC_RSA(opaque);

    if (s->t33) {
        return ingenic_rsa_t33_read(s, offset);
    }
    switch (offset) {
    case RSA_CTRL:
        return s->reg_ctrl;
    case RSA_EXP:
        return s->reg_exp;
    case RSA_DOUT:
        /*
         * The ROM may program the exponent before or after filling the
         * modulus/message FIFOs. Compute on demand, when the result is
         * first read and both operands are known to be loaded, rather
         * than on the exponent write - otherwise an exponent written
         * first runs modexp against a zero modulus and yields zeros.
         */
        if (!s->computed) {
            ingenic_rsa_compute(s);
        }
        if (s->computed && s->res_pos < s->nwords) {
            return s->result[s->res_pos++];
        }
        return 0;
    default:
        return 0;
    }
}

static void ingenic_rsa_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    IngenicRsaState *s = INGENIC_RSA(opaque);

    if (s->t33) {
        ingenic_rsa_t33_write(s, offset, value);
        return;
    }
    switch (offset) {
    case RSA_CTRL: {
        uint32_t old_ctrl = s->reg_ctrl;

        s->reg_ctrl = ((uint32_t)value & ~RSA_CTRL_BUSY) |
                      (old_ctrl & RSA_CTRL_BUSY);
        if ((value & RSA_CTRL_ENABLE) && !(old_ctrl & RSA_CTRL_ENABLE)) {
            s->reg_ctrl |= RSA_CTRL_BUSY;
        }
        if (value & RSA_CTRL_MAIN) {
            s->reg_ctrl &= ~RSA_CTRL_BUSY;
        }
        if (value & RSA_CTRL_2048) {
            s->nwords = 64;
        } else {
            s->nwords = 32;
        }
        /*
         * Re-enabling RSA starts a new operation and resets the FIFOs.
         * U-Boot toggles enable with read-modify-write, preserving the done
         * bits from SPL. Testing only those bits leaves both input FIFOs
         * full, so subsequent key/message writes are silently discarded.
         */
        if (((value & RSA_CTRL_ENABLE) && !(old_ctrl & RSA_CTRL_ENABLE)) ||
            !(value & (RSA_CTRL_MODOK | RSA_CTRL_MSGOK))) {
            s->mod_pos = 0;
            s->msg_pos = 0;
            s->res_pos = 0;
            s->computed = false;
            s->reg_ctrl |= RSA_CTRL_MODOK | RSA_CTRL_MSGOK;
        }
        break;
    }
    case RSA_EXP:
        ingenic_rsa_shift(s->exponent, value);
        s->reg_exp = (uint32_t)value;
        s->reg_ctrl &= ~RSA_CTRL_BUSY;
        s->computed = false;
        s->res_pos = 0;
        break;
    case RSA_MODIN:
        /*
         * A word arriving into an already-full FIFO starts a NEW operation.
         * The guest refills the modulus/message FIFOs for each modexp without
         * necessarily re-clearing CTRL first, and the old code only reset the
         * write pointers on that one CTRL pattern. Once mod_pos/msg_pos
         * saturated at MAXWORDS every later write was silently dropped, so a
         * second operation re-served the first one's result. That is why the
         * stock T32 SPL check passed but U-Boot's FIT signature check always
         * failed with the same em.
         */
        if (s->mod_pos >= s->nwords) {
            s->mod_pos = 0;
            s->computed = false;
            s->res_pos = 0;
        }
        if (s->mod_pos < INGENIC_RSA_MAXWORDS) {
            s->modulus[s->mod_pos++] = (uint32_t)value;
        }
        if (s->mod_pos >= s->nwords) {
            s->reg_ctrl |= RSA_CTRL_MODOK;
        }
        break;
    case RSA_MSGIN:
        if (s->msg_pos >= s->nwords) {
            s->msg_pos = 0;
            s->computed = false;
            s->res_pos = 0;
        }
        if (s->msg_pos < INGENIC_RSA_MAXWORDS) {
            s->message[s->msg_pos++] = (uint32_t)value;
        }
        if (s->msg_pos >= s->nwords) {
            s->reg_ctrl |= RSA_CTRL_MSGOK;
            s->computed = false;
            s->res_pos = 0;
        }
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_rsa_ops = {
    .read = ingenic_rsa_read,
    .write = ingenic_rsa_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_rsa_reset_hold(Object *obj, ResetType type)
{
    IngenicRsaState *s = INGENIC_RSA(obj);
    s->reg_ctrl = 0x00030200;
    s->info = 0;
    s->reg_exp = 0;
    memset(s->exponent, 0, sizeof(s->exponent));
    s->mod_pos = 0;
    s->msg_pos = 0;
    s->res_pos = 0;
    s->nwords = 64;
    s->computed = false;
    memset(s->modulus, 0, sizeof(s->modulus));
    memset(s->message, 0, sizeof(s->message));
    memset(s->result, 0, sizeof(s->result));
}

static void ingenic_rsa_realize(DeviceState *dev, Error **errp)
{
}

static void ingenic_rsa_init(Object *obj)
{
    IngenicRsaState *s = INGENIC_RSA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_rsa_ops, s,
                          TYPE_INGENIC_RSA, INGENIC_RSA_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

/* Indices a state file could set past the operands */
static int ingenic_rsa_post_load(void *opaque, int version_id)
{
    IngenicRsaState *s = opaque;

    return MAX(MAX(s->mod_pos, s->msg_pos), MAX(s->res_pos, s->nwords)) >
           INGENIC_RSA_MAXWORDS ? -EINVAL : 0;
}

static const VMStateDescription vmstate_ingenic_rsa = {
    .name = TYPE_INGENIC_RSA,
    .version_id = 3,
    .minimum_version_id = 1,
    .post_load = ingenic_rsa_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reg_ctrl, IngenicRsaState),
        VMSTATE_UINT32_V(reg_exp, IngenicRsaState, 2),
        VMSTATE_UINT32_ARRAY_V(exponent, IngenicRsaState,
                               INGENIC_RSA_MAXWORDS, 3),
        VMSTATE_UINT32_V(info, IngenicRsaState, 3),
        VMSTATE_UINT32_ARRAY_V(modulus, IngenicRsaState,
                               INGENIC_RSA_MAXWORDS, 2),
        VMSTATE_UINT32_ARRAY_V(message, IngenicRsaState,
                               INGENIC_RSA_MAXWORDS, 2),
        VMSTATE_UINT32_ARRAY_V(result, IngenicRsaState,
                               INGENIC_RSA_MAXWORDS, 2),
        VMSTATE_UINT32_V(mod_pos, IngenicRsaState, 2),
        VMSTATE_UINT32_V(msg_pos, IngenicRsaState, 2),
        VMSTATE_UINT32_V(res_pos, IngenicRsaState, 2),
        VMSTATE_UINT32_V(nwords, IngenicRsaState, 2),
        VMSTATE_BOOL_V(computed, IngenicRsaState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_rsa_properties[] = {
    DEFINE_PROP_BOOL("t33", IngenicRsaState, t33, false),
};

static void ingenic_rsa_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_rsa_realize;
    dc->vmsd = &vmstate_ingenic_rsa;
    device_class_set_props(dc, ingenic_rsa_properties);
    rc->phases.hold = ingenic_rsa_reset_hold;
}

static const TypeInfo ingenic_rsa_type_info = {
    .name = TYPE_INGENIC_RSA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicRsaState),
    .instance_init = ingenic_rsa_init,
    .class_init = ingenic_rsa_class_init,
};

static void ingenic_rsa_register_types(void)
{
    type_register_static(&ingenic_rsa_type_info);
}

type_init(ingenic_rsa_register_types)
