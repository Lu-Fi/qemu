/*
 * Ingenic T31 PDMA (Programmable DMA) controller emulation
 *
 * 32 DMA channels with descriptor-chain and single-transfer modes.
 * The kernel driver (drivers/dma/jzdma/jz_dma.c) uses this for all
 * peripheral DMA: audio (AIC/I2S), SPI, UART, I2C, MSC, etc.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "exec/cpu-common.h"
#include "migration/vmstate.h"
#include "hw/dma/ingenic-pdma.h"

/* Per-channel register offsets within a 0x20-byte slot */
#define CH_DSA      0x00
#define CH_DTA      0x04
#define CH_DTC      0x08
#define CH_DRT      0x0C
#define CH_DCS      0x10
#define CH_DCM      0x14
#define CH_DDA      0x18
#define CH_DSD      0x1C

/* Global registers at base + 0x1000 */
#define DMAC        0x1000
#define DIRQP       0x1004
#define DDR         0x1008
#define DDRS        0x100C
#define DMACP       0x101C
#define DSIRQP      0x1020
#define DSIRQM      0x1024
#define DCIRQP      0x1028
#define DCIRQM      0x102C
#define DMCS        0x1030
#define DMNMB       0x1034
#define DMSMB       0x1038
#define DMINT       0x103C

/* DCS bits */
#define DCS_NDES    BIT(31)
#define DCS_AR      BIT(4)
#define DCS_TT      BIT(3)
#define DCS_HLT     BIT(2)
#define DCS_CTE     BIT(0)

/* DCM bits */
#define DCM_SAI     BIT(23)
#define DCM_DAI     BIT(22)
#define DCM_TIE     BIT(1)
#define DCM_LINK    BIT(0)

/* DTC: low 24 bits = count, upper bits used by kernel for desc chaining */
#define DTC_COUNT_MASK  0x00FFFFFF

/* HW descriptor layout (32 bytes) */
typedef struct {
    uint32_t dcm;
    uint32_t dsa;
    uint32_t dta;
    uint32_t dtc;
    uint32_t sd;
    uint32_t drt;
    uint32_t reserved[2];
} IngenicDmaDesc;

#define DESC_SIZE   sizeof(IngenicDmaDesc)

/* Transfer size encoding in DCM bits [10:8] */
static unsigned pdma_tsz_bytes(uint32_t dcm)
{
    unsigned tsz = (dcm >> 8) & 7;
    switch (tsz) {
    case 0: return 4;     /* 32-bit */
    case 1: return 1;     /* 8-bit */
    case 2: return 2;     /* 16-bit */
    case 3: return 2;     /* 16-bit (alt) */
    case 4: return 4;     /* 32-bit (alt) */
    case 5: return 16;    /* 128-bit / 16-byte burst */
    case 6: return 32;    /* 256-bit / 32-byte burst */
    case 7: return 64;    /* 512-bit / 64-byte burst */
    default: return 4;
    }
}

static void pdma_update_irq(IngenicPdmaState *s)
{
    int level = !!(s->dirqp);
    qemu_set_irq(s->irq, level);
}

static void pdma_do_transfer(IngenicPdmaState *s, int ch,
                             uint32_t dsa, uint32_t dta,
                             uint32_t count, uint32_t dcm)
{
    unsigned tsz = pdma_tsz_bytes(dcm);
    uint32_t nbytes = count * tsz;
    uint8_t buf[4096];

    while (nbytes > 0) {
        uint32_t chunk = (nbytes > sizeof(buf)) ? sizeof(buf) : nbytes;

        cpu_physical_memory_read(dsa, buf, chunk);
        cpu_physical_memory_write(dta, buf, chunk);

        if (dcm & DCM_SAI) {
            dsa += chunk;
        }
        if (dcm & DCM_DAI) {
            dta += chunk;
        }
        nbytes -= chunk;
    }

    s->ch[ch].dsa = dsa;
    s->ch[ch].dta = dta;
    s->ch[ch].dtc = 0;
}

static void pdma_run_channel(IngenicPdmaState *s, int ch)
{
    struct IngenicT31PdmaChannel *c = &s->ch[ch];
    uint32_t dcs = c->dcs;

    if (!(dcs & DCS_CTE)) {
        return;
    }

    if (dcs & DCS_NDES) {
        /* Single-transfer (no descriptor) mode */
        uint32_t count = c->dtc & DTC_COUNT_MASK;
        pdma_do_transfer(s, ch, c->dsa, c->dta, count, c->dcm);

        c->dcs &= ~DCS_CTE;
        c->dcs |= DCS_TT;

        if (c->dcm & DCM_TIE) {
            s->dirqp |= BIT(ch);
            pdma_update_irq(s);
        }
    } else {
        /* Descriptor mode: walk the chain from DDA */
        uint32_t dda = c->dda;
        int max_descs = 256;

        while (max_descs-- > 0) {
            IngenicDmaDesc dd;

            cpu_physical_memory_read(dda, &dd, DESC_SIZE);
            dd.dcm = le32_to_cpu(dd.dcm);
            dd.dsa = le32_to_cpu(dd.dsa);
            dd.dta = le32_to_cpu(dd.dta);
            dd.dtc = le32_to_cpu(dd.dtc);
            dd.drt = le32_to_cpu(dd.drt);

            uint32_t count = dd.dtc & DTC_COUNT_MASK;

            /* Update channel regs to reflect current descriptor */
            c->dsa = dd.dsa;
            c->dta = dd.dta;
            c->dcm = dd.dcm;
            c->drt = dd.drt;
            c->dtc = dd.dtc;

            pdma_do_transfer(s, ch, dd.dsa, dd.dta, count, dd.dcm);

            if (dd.dcm & DCM_LINK) {
                dda += DESC_SIZE;
                c->dda = dda;
            } else {
                /* End of chain */
                c->dcs &= ~DCS_CTE;
                c->dcs |= DCS_TT;
                if (dd.dcm & DCM_TIE) {
                    s->dirqp |= BIT(ch);
                    pdma_update_irq(s);
                }
                break;
            }
        }
    }
}

/* --- MMIO read/write ------------------------------------------------- */

static uint64_t pdma_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicPdmaState *s = INGENIC_PDMA(opaque);

    if (offset < PDMA_NR_CHANNELS * 0x20) {
        int ch = offset / 0x20;
        int reg = offset & 0x1f;
        struct IngenicT31PdmaChannel *c = &s->ch[ch];

        switch (reg) {
        case CH_DSA: return c->dsa;
        case CH_DTA: return c->dta;
        case CH_DTC: return c->dtc;
        case CH_DRT: return c->drt;
        case CH_DCS: return c->dcs;
        case CH_DCM: return c->dcm;
        case CH_DDA: return c->dda;
        case CH_DSD: return c->dsd;
        default:     return 0;
        }
    }

    switch (offset) {
    case DMAC:   return s->dmac;
    case DIRQP:  return s->dirqp;
    case DDR:    return s->ddr;
    case DDRS:   return s->ddrs;
    case DMACP:  return s->dmacp;
    case DSIRQP: return s->dsirqp;
    case DSIRQM: return s->dsirqm;
    case DCIRQP: return s->dcirqp;
    case DCIRQM: return s->dcirqm;
    case DMCS:   return s->dmcs;
    case DMNMB:  return s->dmnmb;
    case DMSMB:  return s->dmsmb;
    case DMINT:  return s->dmint;
    default:
        return 0;
    }
}

static void pdma_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    IngenicPdmaState *s = INGENIC_PDMA(opaque);

    if (offset < PDMA_NR_CHANNELS * 0x20) {
        int ch = offset / 0x20;
        int reg = offset & 0x1f;
        struct IngenicT31PdmaChannel *c = &s->ch[ch];

        switch (reg) {
        case CH_DSA: c->dsa = value; break;
        case CH_DTA: c->dta = value; break;
        case CH_DTC: c->dtc = value; break;
        case CH_DRT: c->drt = value; break;
        case CH_DCS:
            c->dcs = value;
            if (value & DCS_CTE) {
                pdma_run_channel(s, ch);
            }
            break;
        case CH_DCM: c->dcm = value; break;
        case CH_DDA: c->dda = value; break;
        case CH_DSD: c->dsd = value; break;
        }
        return;
    }

    switch (offset) {
    case DMAC:
        s->dmac = value;
        break;
    case DIRQP:
        /* W1C: writing 1 clears pending bits */
        s->dirqp &= ~value;
        pdma_update_irq(s);
        break;
    case DDR:
        s->ddr = value;
        break;
    case DDRS:
        /* Set doorbell bits; the kernel writes BIT(ch) here to
         * trigger descriptor fetch. The actual fetch happens when
         * CTE is set, so just record the doorbell. */
        s->ddrs |= value;
        break;
    case DMACP:
        s->dmacp = value;
        break;
    case DSIRQP:
        s->dsirqp &= ~value;
        break;
    case DSIRQM:
        s->dsirqm = value;
        break;
    case DCIRQP:
        s->dcirqp &= ~value;
        break;
    case DCIRQM:
        s->dcirqm = value;
        break;
    case DMCS:
        s->dmcs = value;
        break;
    case DMNMB:
        s->dmnmb = value;
        break;
    case DMSMB:
        s->dmsmb = value;
        break;
    case DMINT:
        s->dmint = value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps pdma_ops = {
    .read = pdma_read,
    .write = pdma_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

/* --- Lifecycle ------------------------------------------------------- */

static void pdma_reset_hold(Object *obj, ResetType type)
{
    IngenicPdmaState *s = INGENIC_PDMA(obj);
    int i;

    for (i = 0; i < PDMA_NR_CHANNELS; i++) {
        memset(&s->ch[i], 0, sizeof(s->ch[i]));
    }

    s->dmac   = 0;
    s->dirqp  = 0;
    s->ddr    = 0;
    s->ddrs   = 0;
    s->dmacp  = 0;
    s->dsirqp = 0;
    s->dsirqm = 0;
    s->dcirqp = 0;
    s->dcirqm = 0;
    s->dmcs   = 0;
    s->dmnmb  = 0;
    s->dmsmb  = 0;
    s->dmint  = 0;
}

static void pdma_init(Object *obj)
{
    IngenicPdmaState *s = INGENIC_PDMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pdma_ops, s,
                          TYPE_INGENIC_PDMA, 64 * 1024);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_pdma_channel = {
    .name = "ingenic-pdma-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dsa, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dta, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dtc, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(drt, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dcs, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dcm, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dda, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dsd, struct IngenicT31PdmaChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pdma = {
    .name = "ingenic-pdma",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, IngenicPdmaState, PDMA_NR_CHANNELS, 1,
                             vmstate_pdma_channel, struct IngenicT31PdmaChannel),
        VMSTATE_UINT32(dmac,   IngenicPdmaState),
        VMSTATE_UINT32(dirqp,  IngenicPdmaState),
        VMSTATE_UINT32(ddr,    IngenicPdmaState),
        VMSTATE_UINT32(ddrs,   IngenicPdmaState),
        VMSTATE_UINT32(dmacp,  IngenicPdmaState),
        VMSTATE_UINT32(dsirqp, IngenicPdmaState),
        VMSTATE_UINT32(dsirqm, IngenicPdmaState),
        VMSTATE_UINT32(dcirqp, IngenicPdmaState),
        VMSTATE_UINT32(dcirqm, IngenicPdmaState),
        VMSTATE_UINT32(dmcs,   IngenicPdmaState),
        VMSTATE_UINT32(dmnmb,  IngenicPdmaState),
        VMSTATE_UINT32(dmsmb,  IngenicPdmaState),
        VMSTATE_UINT32(dmint,  IngenicPdmaState),
        VMSTATE_END_OF_LIST()
    }
};

static void pdma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_pdma;
    rc->phases.hold = pdma_reset_hold;
}

static const TypeInfo pdma_type_info = {
    .name = TYPE_INGENIC_PDMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicPdmaState),
    .instance_init = pdma_init,
    .class_init = pdma_class_init,
};

static void pdma_register_types(void)
{
    type_register_static(&pdma_type_info);
}

type_init(pdma_register_types)
