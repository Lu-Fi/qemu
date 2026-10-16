/*
 * Ingenic AES engine
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_AES_H
#define HW_MISC_INGENIC_AES_H

#include "hw/core/sysbus.h"
#include "qom/object.h"
#include "crypto/aes.h"

#define TYPE_INGENIC_AES "ingenic-aes"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicAesState, INGENIC_AES)

struct IngenicAesState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t ascr, assr, asintm;
    uint32_t sa, da, tc;            /* T33: tc is ASBNUM */
    uint32_t stc, dtc;              /* T33 only */
    uint32_t key[8], iv[4], di[4], dout[4];
    uint32_t key_n, iv_n, di_n, do_rd, do_n, do_last;
    uint8_t chain[16];
    uint8_t ukey[32];               /* the key last expanded */
    uint32_t ukey_bits;
    AES_KEY ekey, dkey;

    bool edin;                      /* T32, T33: ASCR.EDIN */
    bool edout;                     /* T33: ASCR.EDOUT, else EDIN sets both */
    bool t33;                       /* T33 register layout */
};

#endif /* HW_MISC_INGENIC_AES_H */
