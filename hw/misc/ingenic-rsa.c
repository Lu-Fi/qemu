/*
 * Ingenic RSA hardware accelerator emulation
 *
 * The T31 bootrom uses this for RSA-2048 signature verification.
 * Register protocol (from ROM analysis):
 *   0x00  control/status (bit 7 = 2048-bit mode, bit 2 = modulus ready,
 *         bit 5 = message ready, bit 9 = result busy)
 *   0x04  exponent (32-bit, typically 0x10001 or 3)
 *   0x08  modulus data FIFO (write 64 words for RSA-2048)
 *   0x0C  message data FIFO (write 64 words)
 *   0x10  result data FIFO (read 64 words, reverse order)
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/misc/ingenic-rsa.h"
#include "migration/vmstate.h"
#include <gmp.h>

#define RSA_CTRL     0x00
#define RSA_EXP      0x04
#define RSA_MODIN    0x08
#define RSA_MSGIN    0x0C
#define RSA_DOUT     0x10

#define RSA_CTRL_2048   (1 << 7)
#define RSA_CTRL_MODOK  (1 << 2)
#define RSA_CTRL_MSGOK  (1 << 5)
#define RSA_CTRL_BUSY   (1 << 9)

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
    mpz_set_ui(exp, s->reg_exp);

    if (mpz_sgn(mod) > 0) {
        mpz_powm(res, msg, exp, mod);
    }

    memset(s->result, 0, sizeof(s->result));
    size_t count = 0;
    /* The Ingenic RSA hardware outputs result words LSW-first via DOUT.
     * The T31 ROM reads [0..63] and stores reversed, so output[0] = MSW.
     * Use LSW-first export so DOUT matches real hardware behavior. */
    mpz_export(s->result, &count, -1, 4, 0, 0, res);

    mpz_clear(mod);
    mpz_clear(msg);
    mpz_clear(exp);
    mpz_clear(res);

    s->computed = true;
    s->res_pos = 0;
    s->reg_ctrl &= ~RSA_CTRL_BUSY;
    fprintf(stderr, "RSA: computed %u-word modexp, exp=0x%x, result[0]=0x%08x\n",
            nw, s->reg_exp, s->result[0]);
}

static uint64_t ingenic_rsa_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicRsaState *s = INGENIC_RSA(opaque);

    switch (offset) {
    case RSA_CTRL:
        return s->reg_ctrl;
    case RSA_EXP:
        return s->reg_exp;
    case RSA_DOUT:
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

    switch (offset) {
    case RSA_CTRL:
        s->reg_ctrl = (uint32_t)value;
        if (value & RSA_CTRL_2048) {
            s->nwords = 64;
        } else {
            s->nwords = 32;
        }
        if (!(value & RSA_CTRL_MODOK) && !(value & RSA_CTRL_MSGOK)) {
            s->mod_pos = 0;
            s->msg_pos = 0;
            s->computed = false;
            s->reg_ctrl |= RSA_CTRL_MODOK | RSA_CTRL_MSGOK;
        }
        break;
    case RSA_EXP:
        s->reg_exp = (uint32_t)value;
        ingenic_rsa_compute(s);
        break;
    case RSA_MODIN:
        if (s->mod_pos < INGENIC_RSA_MAXWORDS) {
            s->modulus[s->mod_pos++] = (uint32_t)value;
        }
        if (s->mod_pos >= s->nwords) {
            s->reg_ctrl |= RSA_CTRL_MODOK;
        }
        break;
    case RSA_MSGIN:
        if (s->msg_pos < INGENIC_RSA_MAXWORDS) {
            s->message[s->msg_pos++] = (uint32_t)value;
        }
        if (s->msg_pos >= s->nwords) {
            s->reg_ctrl |= RSA_CTRL_MSGOK;
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
    s->reg_ctrl = 0;
    s->reg_exp = 0;
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

static const VMStateDescription vmstate_ingenic_rsa = {
    .name = TYPE_INGENIC_RSA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reg_ctrl, IngenicRsaState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_rsa_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_rsa_realize;
    dc->vmsd = &vmstate_ingenic_rsa;
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
