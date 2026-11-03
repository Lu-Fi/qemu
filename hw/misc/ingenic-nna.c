/*
 * Ingenic NNA: the T40's, T41's and A1's neural network accelerator
 *
 * The CPU drives it with MXU3's six NN instructions (NNRWR, NNRRD, NNDWR,
 * NNDRD, NNMAC, NNCMD) and feeds it through its own DMA, the NNDMA, which
 * copies between DDR and the ORAM. Its 2048 64-bit descriptors fill the
 * first 16 KiB of the window at 0x12500000:
 *   bits 31:6   DDR address
 *   bits 5:1    (bytes / 64 - 1) bits 16:12
 *   bit 0       another descriptor follows
 *   bits 63:52  (bytes / 64 - 1) bits 11:0
 *   bits 51:32  ORAM address - 0x12600000, in 64-byte units
 * Its channels are at +0x8000: +0x0, +0x4 and +0x8 copy DDR to ORAM, +0x10
 * and +0x14 ORAM to DDR. Writing one sets the index of its first
 * descriptor in bits 10:0, and bit 31 starts it; reading one gives the
 * index after the last descriptor done in bits 10:0 and minus the 64-byte
 * units copied in bits 24:11. The CPU waits for channel n with RDHWR
 * $8 + n (+0x10 and +0x14 are $11 and $12), which stalls until the
 * channel is done and reads 0: the copies finish at once here.
 *
 * Not modelled yet: everything it computes. The rest of the window keeps
 * what is written to it, the ORAM is RAM, and the NN instructions read
 * zeros; with "log" set, each instruction and its 64 bytes go to that file.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-nna.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "trace.h"

#define NNDMA_REGS          0x8000
#define NNDMA_DESCS         2048
#define NNDMA_START         BIT(31)
#define NNDMA_SRAM          0x12600000  /* the L2's SRAM, then the ORAM */

void ingenic_nna_op(void *opaque, unsigned op, uint32_t imm, uint8_t *vec)
{
    IngenicNnaState *s = opaque;

    trace_ingenic_nna_op(op, imm, ldl_le_p(vec));
    if (s->log) {
        uint32_t head[2] = { cpu_to_le32(op), cpu_to_le32(imm) };

        fwrite(head, sizeof(head), 1, s->log);
        fwrite(vec, 64, 1, s->log);
    }
}

/* the channels at +0x8000, by register: 1 to the ORAM, 0 to DDR, -1 none */
static const int8_t nndma_to_oram[INGENIC_NNA_NNDMA_REGS] = {
    1, 1, 1, -1, 0, 0, -1, -1,
};

static void nndma_copy(hwaddr dst, hwaddr src, uint32_t bytes)
{
    uint8_t buf[4 * KiB];

    while (bytes) {
        uint32_t n = MIN(bytes, sizeof(buf));

        address_space_read(&address_space_memory, src, MEMTXATTRS_UNSPECIFIED,
                           buf, n);
        address_space_write(&address_space_memory, dst, MEMTXATTRS_UNSPECIFIED,
                            buf, n);
        src += n;
        dst += n;
        bytes -= n;
    }
}

static void nndma_run(IngenicNnaState *s, unsigned reg)
{
    unsigned idx = s->nndma[reg] & (NNDMA_DESCS - 1);
    uint32_t units = 0;
    uint64_t d;
    int i;

    for (i = 0; i < NNDMA_DESCS; i++) {
        uint32_t lo, hi, n;
        hwaddr ddr, oram;

        d = ldq_le_p(&s->window[idx * 8]);
        lo = d;
        hi = d >> 32;
        n = extract32(lo, 1, 5) << 12 | extract32(hi, 20, 12);
        ddr = lo & ~0x3fu;
        oram = NNDMA_SRAM + (hwaddr)extract32(hi, 0, 20) * 64;
        if (nndma_to_oram[reg]) {
            nndma_copy(oram, ddr, (n + 1) * 64);
        } else {
            nndma_copy(ddr, oram, (n + 1) * 64);
        }
        units += n + 1;
        idx = (idx + 1) & (NNDMA_DESCS - 1);
        if (!(lo & 1)) {
            break;
        }
    }
    if (i == NNDMA_DESCS) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-nna: NNDMA chain at +0x%x "
                      "never ends\n", reg * 4);
    }
    s->nndma[reg] = (-units & 0x3fff) << 11 | idx;
}

static bool nndma_reg(hwaddr offset, unsigned *reg)
{
    *reg = (offset - NNDMA_REGS) / 4;
    return offset >= NNDMA_REGS && *reg < INGENIC_NNA_NNDMA_REGS;
}

static uint64_t nna_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicNnaState *s = opaque;
    unsigned reg;
    uint64_t v;

    if (nndma_reg(offset, &reg)) {
        v = s->nndma[reg];
        trace_ingenic_nna_read(offset, size, v);
        return v;
    }
    v = ldn_le_p(&s->window[offset], size);

    trace_ingenic_nna_read(offset, size, v);
    return v;
}

static void nna_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    IngenicNnaState *s = opaque;
    unsigned reg;

    trace_ingenic_nna_write(offset, size, value);
    if (nndma_reg(offset, &reg)) {
        if (nndma_to_oram[reg] < 0) {
            qemu_log_mask(LOG_UNIMP, "ingenic-nna: NNDMA +0x%x = 0x%" PRIx64
                          "\n", reg * 4, value);
            return;
        }
        s->nndma[reg] = value & (NNDMA_DESCS - 1);
        if (value & NNDMA_START) {
            nndma_run(s, reg);
        }
        return;
    }
    stn_le_p(&s->window[offset], size, value);
}

static const MemoryRegionOps nna_ops = {
    .read = nna_read,
    .write = nna_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void nna_reset_hold(Object *obj, ResetType type)
{
    IngenicNnaState *s = INGENIC_NNA(obj);

    memset(s->nndma, 0, sizeof(s->nndma));
}

static const VMStateDescription vmstate_nna = {
    .name = TYPE_INGENIC_NNA,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_ARRAY(window, IngenicNnaState, INGENIC_NNA_WINDOW),
        VMSTATE_UINT32_ARRAY(nndma, IngenicNnaState, INGENIC_NNA_NNDMA_REGS),
        VMSTATE_END_OF_LIST()
    }
};

static void nna_realize(DeviceState *dev, Error **errp)
{
    IngenicNnaState *s = INGENIC_NNA(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &nna_ops, s, "ingenic-nna",
                          INGENIC_NNA_WINDOW);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    if (!memory_region_init_ram(&s->oram, OBJECT(s), "ingenic-nna.oram",
                                s->oram_size, errp)) {
        return;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->oram);
    if (s->log_path) {
        s->log = fopen(s->log_path, "wb");
        if (!s->log) {
            error_setg_errno(errp, errno, "ingenic-nna: cannot open %s",
                             s->log_path);
        }
    }
}

static const Property nna_properties[] = {
    DEFINE_PROP_UINT32("oram-size", IngenicNnaState, oram_size, 384 * KiB),
    DEFINE_PROP_STRING("log", IngenicNnaState, log_path),
};

static void nna_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = nna_realize;
    dc->vmsd = &vmstate_nna;
    rc->phases.hold = nna_reset_hold;
    device_class_set_props(dc, nna_properties);
}

static const TypeInfo nna_type_info = {
    .name = TYPE_INGENIC_NNA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicNnaState),
    .class_init = nna_class_init,
};

static void nna_register_types(void)
{
    type_register_static(&nna_type_info);
}

type_init(nna_register_types)
