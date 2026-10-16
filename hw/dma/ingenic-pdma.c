/*
 * Ingenic PDMA (Programmable DMA) controller emulation
 *
 * 32 channels, each copying memory to memory or between memory and a
 * peripheral FIFO, set up in its registers (no-descriptor mode) or by a
 * chain of 4- or 8-word descriptors. A channel with request type 8 (auto)
 * runs as soon as it is enabled. Any other waits on that peripheral's
 * request line (the "dreq" inputs, numbered by request type) and moves one
 * data unit per request (TSZ, or RDIL bytes in autonomy mode) while the
 * line stays up, reading or writing the fixed peripheral address in
 * port-width (SP/DP) accesses. The "dack" outputs tell a peripheral that a
 * channel is serving it.
 *
 * As measured on the T31 and T41:
 *  - DTC counts data units, or bytes in autonomy mode (TSZ 7); DSA and DTA
 *    advance by the bytes moved. A finished channel sets TT and keeps CTE.
 *    A channel started with DTC 0 never finishes.
 *  - DIRQP bit n is channel n's TT with TIE: writing DIRQP does nothing,
 *    clearing TT clears it.
 *  - A descriptor run loads DCM, DSA, DTA, DTC and (8-word) DSD and DRT
 *    from memory, links through DOA within DDA's 4 KiB page, leaves DDA as
 *    written and keeps the last finished descriptor's offset in DCS.CDOA.
 *    DDS starts the fetch.
 *  - Clearing CTE on an unfinished channel sets DCS.HLT and DMAC.HLT.
 * A linked descriptor finishing raises nothing, TIE or not: a descriptor
 * linked to itself (vendor audio) runs forever without an interrupt.
 * The PDMA's own MCU is not modelled: its memory is RAM and its registers
 * hold what is written.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/main-loop.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "system/address-spaces.h"
#include "system/memory.h"
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

/* Global registers */
#define DMAC        0x1000
#define DIRQP       0x1004
#define DDB         0x1008
#define DDS         0x100C
#define DIP         0x1010
#define DIC         0x1014
#define DMACP       0x101C
#define DSIRQP      0x1020
#define DSIRQM      0x1024
#define DCIRQP      0x1028
#define DCIRQM      0x102C
#define DMCS        0x1030
#define DMNMB       0x1034
#define DMSMB       0x1038
#define DMINT       0x103C

#define PDMA_TCSM       0x2000
#define PDMA_TCSM_SIZE  0x2000
#define PDMA_IOSIZE     0x10000

#define DMAC_DMAE   BIT(0)
#define DMAC_AR     BIT(2)
#define DMAC_HLT    BIT(3)

#define DCS_NDES    BIT(31)
#define DCS_DES8    BIT(30)
#define DCS_AR      BIT(4)
#define DCS_TT      BIT(3)
#define DCS_HLT     BIT(2)
#define DCS_CTE     BIT(0)
#define DCS_MASK    0xc000ff1d

#define DCM_SAI     BIT(23)
#define DCM_DAI     BIT(22)
#define DCM_RDIL(v) extract32(v, 16, 4)
#define DCM_SP(v)   extract32(v, 14, 2)
#define DCM_DP(v)   extract32(v, 12, 2)
#define DCM_TSZ(v)  extract32(v, 8, 3)
#define DCM_STDE    BIT(2)
#define DCM_TIE     BIT(1)
#define DCM_LINK    BIT(0)

#define DTC_MASK    0x00ffffff
#define DRT_AUTO    8
#define TSZ_AUTONOMY 7

/* Bytes in a data unit by TSZ, in a request by RDIL, in an access by SP/DP */
static const uint8_t pdma_tsz[7] = { 4, 1, 2, 16, 32, 64, 128 };
static const uint8_t pdma_rdil[16] = {
    1, 1, 2, 3, 4, 8, 16, 32, 64, 128, 128, 128, 128, 128, 128, 128,
};
static const uint8_t pdma_port[4] = { 4, 1, 2, 4 };

typedef struct IngenicT31PdmaChannel PdmaChannel;

static void pdma_run(IngenicPdmaState *s);

static bool pdma_runnable(IngenicPdmaState *s, PdmaChannel *c)
{
    return (s->dmac & DMAC_DMAE) && (c->dcs & DCS_CTE) &&
           !(c->dcs & (DCS_TT | DCS_HLT | DCS_AR));
}

static bool pdma_has_work(PdmaChannel *c)
{
    return ((c->dcs & DCS_NDES) || c->loaded) && (c->dtc & DTC_MASK);
}

static void pdma_update(IngenicPdmaState *s)
{
    uint64_t dack = 0;

    for (int n = 0; n < PDMA_NR_CHANNELS; n++) {
        PdmaChannel *c = &s->ch[n];

        if (pdma_runnable(s, c) && pdma_has_work(c) && c->drt != DRT_AUTO &&
            c->drt < PDMA_NR_REQUESTS) {
            dack |= 1ULL << c->drt;
        }
    }
    for (int r = 0; r < PDMA_NR_REQUESTS; r++) {
        if (((dack ^ s->dack_level) >> r) & 1) {
            qemu_set_irq(s->dack[r], (dack >> r) & 1);
        }
    }
    s->dack_level = dack;
    qemu_set_irq(s->irq, !!s->dirqp);
}

static void pdma_error(IngenicPdmaState *s, int n)
{
    s->ch[n].dcs |= DCS_AR;
    s->dmac |= DMAC_AR;
    s->dirqp |= BIT(n);
    qemu_log_mask(LOG_GUEST_ERROR, "ingenic-pdma: channel %d address error\n",
                  n);
}

static bool pdma_fetch(IngenicPdmaState *s, int n, uint32_t addr)
{
    PdmaChannel *c = &s->ch[n];
    int words = c->dcs & DCS_DES8 ? 8 : 4;
    uint32_t d[8];

    if (address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                           d, 4 * words) != MEMTX_OK) {
        pdma_error(s, n);
        return false;
    }
    c->dcm = le32_to_cpu(d[0]);
    c->dsa = le32_to_cpu(d[1]);
    c->dta = le32_to_cpu(d[2]);
    c->dtc = le32_to_cpu(d[3]) & DTC_MASK;
    c->doa = le32_to_cpu(d[3]) >> 24;
    if (words == 8) {
        c->dsd = le32_to_cpu(d[4]);
        c->drt = le32_to_cpu(d[5]) & 0x3f;
    }
    c->desc = addr;
    c->block = c->dtc & 0xffff;
    c->loaded = true;
    return true;
}

/*
 * One side of a move: memory runs straight through, a fixed address
 * (SAI/DAI clear, a peripheral FIFO) takes it in port-width accesses.
 */
static bool pdma_access(uint32_t *addr, bool inc, unsigned port, uint8_t *buf,
                        uint32_t len, bool write)
{
    MemTxResult r = MEMTX_OK;

    if (inc) {
        r = address_space_rw(&address_space_memory, *addr,
                             MEMTXATTRS_UNSPECIFIED, buf, len, write);
        *addr += len;
        return r == MEMTX_OK;
    }
    for (uint32_t i = 0; i < len && r == MEMTX_OK; i += port) {
        r = address_space_rw(&address_space_memory, *addr,
                             MEMTXATTRS_UNSPECIFIED, buf + i,
                             MIN(port, len - i), write);
    }
    return r == MEMTX_OK;
}

static bool pdma_move(IngenicPdmaState *s, int n, uint32_t len)
{
    PdmaChannel *c = &s->ch[n];
    uint8_t buf[4096];

    assert(len <= sizeof(buf));
    if (!pdma_access(&c->dsa, c->dcm & DCM_SAI, pdma_port[DCM_SP(c->dcm)],
                     buf, len, false) ||
        !pdma_access(&c->dta, c->dcm & DCM_DAI, pdma_port[DCM_DP(c->dcm)],
                     buf, len, true)) {
        pdma_error(s, n);
        return false;
    }
    return true;
}

static void pdma_done(IngenicPdmaState *s, int n)
{
    PdmaChannel *c = &s->ch[n];

    if (!(c->dcs & DCS_NDES)) {
        c->dcs = deposit32(c->dcs, 8, 8, (c->desc >> 4) & 0xff);
        if (c->dcm & DCM_LINK) {
            pdma_fetch(s, n, (c->dda & ~0xfff) | (c->doa << 4));
            return;
        }
        c->loaded = false;
    }
    c->dcs |= DCS_TT;
    if (c->dcm & DCM_TIE) {
        s->dirqp |= BIT(n);
    }
}

/*
 * The bytes the next step moves: a request's worth, or for auto as much
 * as a step takes. Stride mode stops at the end of each sub-block.
 */
static uint32_t pdma_step_units(PdmaChannel *c, bool all)
{
    uint32_t left = c->dtc & (c->dcm & DCM_STDE ? 0xffff : DTC_MASK);
    bool bytes = DCM_TSZ(c->dcm) == TSZ_AUTONOMY;
    uint32_t unit = bytes ? 1 : pdma_tsz[DCM_TSZ(c->dcm)];

    if (!all) {
        return bytes ? MIN(left, pdma_rdil[DCM_RDIL(c->dcm)]) : 1;
    }
    return MIN(left, 4096 / unit);
}

static void pdma_channel(IngenicPdmaState *s, int n)
{
    PdmaChannel *c = &s->ch[n];
    int budget = 1024;

    while (pdma_runnable(s, c)) {
        if (!budget--) {
            /* An endless auto chain: let the guest run, carry on later */
            qemu_bh_schedule(s->bh);
            return;
        }
        bool autoreq = c->drt == DRT_AUTO;
        uint32_t units, bytes;

        if (!(c->dcs & DCS_NDES) && !c->loaded) {
            if (!(s->ddr & BIT(n))) {
                return;
            }
            s->ddr &= ~BIT(n);
            if (!pdma_fetch(s, n, c->dda & ~0xf)) {
                return;
            }
            continue;
        }
        if (!(c->dtc & DTC_MASK)) {
            return;
        }
        if (!autoreq && (c->drt >= PDMA_NR_REQUESTS ||
                         !((s->dreq >> c->drt) & 1))) {
            return;
        }
        units = pdma_step_units(c, autoreq);
        bytes = units;
        if (DCM_TSZ(c->dcm) != TSZ_AUTONOMY) {
            bytes *= pdma_tsz[DCM_TSZ(c->dcm)];
        }
        if (!pdma_move(s, n, bytes)) {
            return;
        }
        if (c->dcm & DCM_STDE) {
            uint32_t lo = (c->dtc & 0xffff) - units, hi = (c->dtc >> 16) & 0xff;

            if (!lo && hi > 1) {
                /* Next sub-block: the stride differences are signed */
                if (c->dcm & DCM_SAI) {
                    c->dsa += (int16_t)extract32(c->dsd, 0, 16);
                }
                if (c->dcm & DCM_DAI) {
                    c->dta += (int16_t)extract32(c->dsd, 16, 16);
                }
                lo = c->block;
                hi--;
            } else if (!lo) {
                hi = 0;
            }
            c->dtc = hi << 16 | lo;
        } else {
            c->dtc -= units;
        }
        if (!(c->dtc & DTC_MASK)) {
            pdma_done(s, n);
        }
    }
}

static void pdma_run(IngenicPdmaState *s)
{
    if (s->busy) {
        s->again = true;
        return;
    }
    s->busy = true;
    do {
        s->again = false;
        for (int n = 0; n < PDMA_NR_CHANNELS; n++) {
            pdma_channel(s, n);
        }
    } while (s->again);
    s->busy = false;
    pdma_update(s);
}

static void pdma_bh(void *opaque)
{
    pdma_run(opaque);
}

/* A peripheral's request line; serviced from a BH, outside its MMIO */
static void pdma_set_dreq(void *opaque, int n, int level)
{
    IngenicPdmaState *s = opaque;
    bool rise = level && !((s->dreq >> n) & 1);

    s->dreq = deposit64(s->dreq, n, 1, level != 0);
    if (rise) {
        if (s->busy) {
            s->again = true;
        } else {
            qemu_bh_schedule(s->bh);
        }
    }
}

/* --- MMIO read/write ------------------------------------------------- */

static uint64_t pdma_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicPdmaState *s = INGENIC_PDMA(opaque);

    if (offset < PDMA_NR_CHANNELS * 0x20) {
        PdmaChannel *c = &s->ch[offset / 0x20];

        switch (offset & 0x1f) {
        case CH_DSA: return c->dsa;
        case CH_DTA: return c->dta;
        case CH_DTC: return c->dtc;
        case CH_DRT: return c->drt;
        case CH_DCS: return c->dcs;
        case CH_DCM: return c->dcm;
        case CH_DDA: return c->dda;
        case CH_DSD: return c->dsd;
        }
    }

    switch (offset) {
    case DMAC:   return s->dmac;
    case DIRQP:  return s->dirqp;
    case DDB:    return s->ddr;
    case DIP:    return s->dip;
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

static void pdma_write_dcs(IngenicPdmaState *s, int n, uint32_t v)
{
    PdmaChannel *c = &s->ch[n];
    uint32_t old = c->dcs;

    v &= DCS_MASK;
    /* AR and HLT only clear */
    v = (v & ~(DCS_AR | DCS_HLT)) | (old & v & (DCS_AR | DCS_HLT));
    if ((old & DCS_CTE) && !(v & DCS_CTE) &&
        !(old & (DCS_TT | DCS_HLT | DCS_AR))) {
        v |= DCS_HLT;
        s->dmac |= DMAC_HLT;
    }
    if (!(v & DCS_CTE)) {
        c->loaded = false;
    }
    if (!(v & DCS_TT)) {
        s->dirqp &= ~BIT(n);
    }
    c->dcs = v;
}

static void pdma_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    IngenicPdmaState *s = INGENIC_PDMA(opaque);

    if (offset < PDMA_NR_CHANNELS * 0x20) {
        int n = offset / 0x20;
        PdmaChannel *c = &s->ch[n];

        switch (offset & 0x1f) {
        case CH_DSA:
            c->dsa = value;
            break;
        case CH_DTA:
            c->dta = value;
            break;
        case CH_DTC:
            c->dtc = value & DTC_MASK;
            break;
        case CH_DRT:
            c->drt = value & 0x3f;
            break;
        case CH_DCS:
            pdma_write_dcs(s, n, value);
            break;
        case CH_DCM:
            c->dcm = value;
            break;
        case CH_DDA:
            c->dda = value & ~0xf;
            break;
        case CH_DSD:
            c->dsd = value;
            break;
        }
        pdma_run(s);
        return;
    }

    switch (offset) {
    case DMAC:
        /* AR and HLT only clear */
        s->dmac = (value & ~(DMAC_AR | DMAC_HLT)) |
                  (s->dmac & value & (DMAC_AR | DMAC_HLT));
        break;
    case DIRQP:
        /* Read only: the silicon ignores writes here, TT clears a bit */
        break;
    case DDB:
    case DDS:
        for (int n = 0; n < PDMA_NR_CHANNELS; n++) {
            if (extract64(value, n, 1)) {
                s->ch[n].loaded = false;
            }
        }
        s->ddr |= value;
        break;
    case DIC:
        s->dip &= ~value;
        break;
    case DMACP:
        s->dmacp = value;
        break;
    case DSIRQP:
        s->dsirqp = value;
        break;
    case DSIRQM:
        s->dsirqm = value;
        break;
    case DCIRQP:
        s->dcirqp = value;
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
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-pdma: write of 0x%" PRIx64
                      " to 0x%" HWADDR_PRIx "\n", value, offset);
        return;
    }
    pdma_run(s);
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

    memset(s->ch, 0, sizeof(s->ch));
    s->dmac   = 0;
    s->dirqp  = 0;
    s->ddr    = 0;
    s->dip    = 0;
    s->dmacp  = 0;
    s->dsirqp = 0;
    s->dsirqm = 0;
    s->dcirqp = 0;
    s->dcirqm = 0;
    s->dmcs   = 0;
    s->dmnmb  = 0;
    s->dmsmb  = 0;
    s->dmint  = 0;
    s->busy = s->again = false;
}

static void pdma_reset_exit(Object *obj, ResetType type)
{
    pdma_update(INGENIC_PDMA(obj));
}

static void pdma_init(Object *obj)
{
    IngenicPdmaState *s = INGENIC_PDMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &pdma_ops, s,
                          TYPE_INGENIC_PDMA, PDMA_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), pdma_set_dreq, "dreq",
                            PDMA_NR_REQUESTS);
    qdev_init_gpio_out_named(DEVICE(obj), s->dack, "dack", PDMA_NR_REQUESTS);
}

static void pdma_realize(DeviceState *dev, Error **errp)
{
    IngenicPdmaState *s = INGENIC_PDMA(dev);

    if (!memory_region_init_ram(&s->tcsm, OBJECT(dev), "ingenic-pdma.tcsm",
                                PDMA_TCSM_SIZE, errp)) {
        return;
    }
    memory_region_add_subregion_overlap(&s->iomem, PDMA_TCSM, &s->tcsm, 1);
    s->bh = qemu_bh_new_guarded(pdma_bh, s, &dev->mem_reentrancy_guard);
}

static const VMStateDescription vmstate_pdma_channel = {
    .name = "ingenic-pdma-channel",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dsa, PdmaChannel),
        VMSTATE_UINT32(dta, PdmaChannel),
        VMSTATE_UINT32(dtc, PdmaChannel),
        VMSTATE_UINT32(drt, PdmaChannel),
        VMSTATE_UINT32(dcs, PdmaChannel),
        VMSTATE_UINT32(dcm, PdmaChannel),
        VMSTATE_UINT32(dda, PdmaChannel),
        VMSTATE_UINT32(dsd, PdmaChannel),
        VMSTATE_BOOL(loaded, PdmaChannel),
        VMSTATE_UINT32(desc, PdmaChannel),
        VMSTATE_UINT32(doa, PdmaChannel),
        VMSTATE_UINT32(block, PdmaChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pdma = {
    .name = "ingenic-pdma",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, IngenicPdmaState, PDMA_NR_CHANNELS, 2,
                             vmstate_pdma_channel, PdmaChannel),
        VMSTATE_UINT32(dmac,   IngenicPdmaState),
        VMSTATE_UINT32(dirqp,  IngenicPdmaState),
        VMSTATE_UINT32(ddr,    IngenicPdmaState),
        VMSTATE_UINT32(dip,    IngenicPdmaState),
        VMSTATE_UINT32(dmacp,  IngenicPdmaState),
        VMSTATE_UINT32(dsirqp, IngenicPdmaState),
        VMSTATE_UINT32(dsirqm, IngenicPdmaState),
        VMSTATE_UINT32(dcirqp, IngenicPdmaState),
        VMSTATE_UINT32(dcirqm, IngenicPdmaState),
        VMSTATE_UINT32(dmcs,   IngenicPdmaState),
        VMSTATE_UINT32(dmnmb,  IngenicPdmaState),
        VMSTATE_UINT32(dmsmb,  IngenicPdmaState),
        VMSTATE_UINT32(dmint,  IngenicPdmaState),
        VMSTATE_UINT64(dreq,   IngenicPdmaState),
        VMSTATE_UINT64(dack_level, IngenicPdmaState),
        VMSTATE_END_OF_LIST()
    }
};

static void pdma_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = pdma_realize;
    dc->vmsd = &vmstate_pdma;
    rc->phases.hold = pdma_reset_hold;
    rc->phases.exit = pdma_reset_exit;
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
