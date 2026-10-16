/*
 * Ingenic SSI_SLV, the SPI slave of the T31
 *
 * A DesignWare APB SSI (version 4.00*) in its slave-only configuration,
 * as the X1600 PM describes it: 64-word TX and RX FIFOs, frames of 4 to
 * 32 bits, Motorola SPI only (FRF is not writable). An
 * "ingenic-ssi-slv-port" on a master's SSI bus stands for its pins:
 * -device ingenic-ssi-slv-port,bus=ssi1,cs=0 wires it to SSI1's CE0, as
 * the vendor's loop test does. A transfer brings 8 bits of a frame when
 * the frame is whole bytes and the whole frame otherwise, which is how the
 * Ingenic master sends them. The "dreq" outputs are the TX and RX DMA
 * requests.
 *
 * As measured on the T31:
 *  - The registers repeat every 0x100 bytes; the master-only ones read 0
 *    and every DR address (0x60-0xec) is the FIFOs.
 *  - Unconnected, the chip select input is active: once enabled the slave
 *    is BUSY, and enabling it with the TX FIFO empty sets TXE.
 *  - TXE is an event: reading SR clears it, and it is not set again while
 *    the FIFO stays empty.
 *  - Writes of 64 or more to a threshold or DMA level are ignored, as are
 *    data frame sizes below 4 bits and CTRLR writes while enabled.
 *  - A DR write while disabled is lost. Reading the empty RX FIFO returns
 *    its RAM at the read pointer, and sets RXU only while enabled.
 *  - TXOICR, RXOICR, RXUICR and ICR clear the raw flags and return the
 *    masked ones.
 *  - Disabling empties the FIFOs.
 *  - The TX DMA request is checked before each data unit, so a channel
 *    moving 16 or 32 bytes per unit can overshoot the level by one unit.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qom/object.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/ssi/ingenic-ssi-slv.h"
#include "migration/vmstate.h"

#define SLV_CTRLR       0x00
#define SLV_SSIENR      0x08
#define SLV_MWCR        0x0c
#define SLV_TXFTLR      0x18
#define SLV_RXFTLR      0x1c
#define SLV_TXFLR       0x20
#define SLV_RXFLR       0x24
#define SLV_SR          0x28
#define SLV_IMR         0x2c
#define SLV_ISR         0x30
#define SLV_RISR        0x34
#define SLV_TXOICR      0x38
#define SLV_RXOICR      0x3c
#define SLV_RXUICR      0x40
#define SLV_ICR         0x48
#define SLV_DMACR       0x4c
#define SLV_DMATDLR     0x50
#define SLV_DMARDLR     0x54
#define SLV_IDR         0x58
#define SLV_VERSION     0x5c
#define SLV_DR          0x60
#define SLV_DR_END      0xf0
#define SLV_IOSIZE      0x1000

#define CTRLR_RESET     0x00070000
#define CTRLR_MASK      0x001fffc0
#define CTRLR_DFS(v)    extract32(v, 16, 5)
#define CTRLR_SRL       BIT(11)
#define CTRLR_SLV_OE    BIT(10)
#define CTRLR_TMOD(v)   extract32(v, 8, 2)

#define TMOD_TX         1
#define TMOD_RX         2

#define SR_TXE          BIT(5)
#define SR_RFF          BIT(4)
#define SR_RFNE         BIT(3)
#define SR_TFE          BIT(2)
#define SR_TFNF         BIT(1)
#define SR_BUSY         BIT(0)

#define INT_RXF         BIT(4)
#define INT_RXO         BIT(3)
#define INT_RXU         BIT(2)
#define INT_TXO         BIT(1)
#define INT_TXE         BIT(0)
#define INT_MASK        0x1f

#define DMACR_TDMAE     BIT(1)
#define DMACR_RDMAE     BIT(0)

#define SLV_IDR_VALUE   0xffffffff
#define SLV_VERSION_ID  0x3430302a      /* "4.00*" */

static bool slv_enabled(IngenicSsiSlvState *s)
{
    return s->ssienr & 1;
}

static int slv_frame_bits(IngenicSsiSlvState *s)
{
    return CTRLR_DFS(s->ctrlr) + 1;
}

static uint32_t slv_risr(IngenicSsiSlvState *s)
{
    uint32_t r = 0;

    if (slv_enabled(s)) {
        r |= s->tx_n <= s->txftlr ? INT_TXE : 0;
        r |= s->rx_n > s->rxftlr ? INT_RXF : 0;
    }
    r |= s->txo ? INT_TXO : 0;
    r |= s->rxu ? INT_RXU : 0;
    r |= s->rxo ? INT_RXO : 0;
    return r;
}

static void slv_update(IngenicSsiSlvState *s)
{
    bool en = slv_enabled(s);

    qemu_set_irq(s->irq, !!(slv_risr(s) & s->imr));
    qemu_set_irq(s->dreq[0], en && (s->dmacr & DMACR_TDMAE) &&
                 s->tx_n <= s->dmatdlr);
    qemu_set_irq(s->dreq[1], en && (s->dmacr & DMACR_RDMAE) &&
                 s->rx_n > s->dmardlr);
}

/* A transfer begins: the slave is enabled with its chip select active */
static void slv_begin(IngenicSsiSlvState *s)
{
    s->bit = 0;
    if (!s->tx_n && CTRLR_TMOD(s->ctrlr) != TMOD_RX) {
        s->txe = true;
    }
}

/* The next frame into the TX shift register; the last one again if none */
static void slv_load(IngenicSsiSlvState *s)
{
    if (CTRLR_TMOD(s->ctrlr) == TMOD_RX) {
        s->tx_shift = 0;
    } else if (s->tx_n) {
        s->tx_last = s->tx[s->tx_rd];
        s->tx_rd = (s->tx_rd + 1) % INGENIC_SSI_SLV_FIFO;
        s->tx_n--;
        s->tx_shift = s->tx_last;
    } else {
        s->txe = true;
        s->tx_shift = s->tx_last;
    }
    s->rx_shift = 0;
}

static void slv_store(IngenicSsiSlvState *s, int n)
{
    if (CTRLR_TMOD(s->ctrlr) == TMOD_TX) {
        return;
    }
    if (s->rx_n == INGENIC_SSI_SLV_FIFO) {
        s->rxo = true;
        return;
    }
    s->rx[(s->rx_rd + s->rx_n) % INGENIC_SSI_SLV_FIFO] =
        s->rx_shift & MAKE_64BIT_MASK(0, n);
    s->rx_n++;
}

/* Bits from the master, MSB first; the slave's own go back the same way */
static uint32_t slv_transfer(IngenicSsiSlvState *s, uint32_t val)
{
    int n = slv_frame_bits(s);
    int k = n % 8 ? n : 8;
    uint32_t out = 0;

    if (!slv_enabled(s)) {
        return 0;
    }
    for (int i = k - 1; i >= 0; i--) {
        uint32_t bit;

        if (!s->bit) {
            slv_load(s);
        }
        bit = extract32(s->tx_shift, n - 1 - s->bit, 1);
        out = out << 1 | bit;
        s->rx_shift = s->rx_shift << 1 |
                      (s->ctrlr & CTRLR_SRL ? bit : extract32(val, i, 1));
        if (++s->bit == n) {
            slv_store(s, n);
            s->bit = 0;
        }
    }
    slv_update(s);
    return s->ctrlr & CTRLR_SLV_OE ? 0 : out;
}

static void slv_select(IngenicSsiSlvState *s, bool selected)
{
    bool was = slv_enabled(s) && s->selected;

    s->selected = selected;
    if (!selected) {
        /* A frame cut short is lost */
        s->bit = 0;
    } else if (!was && slv_enabled(s)) {
        slv_begin(s);
    }
    slv_update(s);
}

static uint32_t slv_sr(IngenicSsiSlvState *s)
{
    uint32_t sr = 0;

    sr |= s->txe ? SR_TXE : 0;
    sr |= s->rx_n == INGENIC_SSI_SLV_FIFO ? SR_RFF : 0;
    sr |= s->rx_n ? SR_RFNE : 0;
    sr |= s->tx_n ? 0 : SR_TFE;
    sr |= s->tx_n < INGENIC_SSI_SLV_FIFO ? SR_TFNF : 0;
    sr |= slv_enabled(s) && s->selected ? SR_BUSY : 0;
    return sr;
}

/* Read a clear register: the masked flags, then the raw ones go */
static uint32_t slv_clear(IngenicSsiSlvState *s, uint32_t bits)
{
    uint32_t v = !!(slv_risr(s) & s->imr & bits);

    s->txo = s->txo && !(bits & INT_TXO);
    s->rxu = s->rxu && !(bits & INT_RXU);
    s->rxo = s->rxo && !(bits & INT_RXO);
    return v;
}

static uint64_t ingenic_ssi_slv_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicSsiSlvState *s = opaque;
    hwaddr reg = offset & 0xfc;
    uint32_t v = 0;

    if (reg >= SLV_DR && reg < SLV_DR_END) {
        v = s->rx[s->rx_rd];
        if (slv_enabled(s)) {
            if (s->rx_n) {
                s->rx_rd = (s->rx_rd + 1) % INGENIC_SSI_SLV_FIFO;
                s->rx_n--;
            } else {
                s->rxu = true;
            }
        }
        slv_update(s);
        return v >> (offset & 3) * 8;
    }
    switch (reg) {
    case SLV_CTRLR:
        v = s->ctrlr;
        break;
    case SLV_SSIENR:
        v = s->ssienr;
        break;
    case SLV_MWCR:
        v = s->mwcr;
        break;
    case SLV_TXFTLR:
        v = s->txftlr;
        break;
    case SLV_RXFTLR:
        v = s->rxftlr;
        break;
    case SLV_TXFLR:
        v = s->tx_n;
        break;
    case SLV_RXFLR:
        v = s->rx_n;
        break;
    case SLV_SR:
        v = slv_sr(s);
        s->txe = false;
        break;
    case SLV_IMR:
        v = s->imr;
        break;
    case SLV_ISR:
        v = slv_risr(s) & s->imr;
        break;
    case SLV_RISR:
        v = slv_risr(s);
        break;
    case SLV_TXOICR:
        v = slv_clear(s, INT_TXO);
        break;
    case SLV_RXOICR:
        v = slv_clear(s, INT_RXO);
        break;
    case SLV_RXUICR:
        v = slv_clear(s, INT_RXU);
        break;
    case SLV_ICR:
        v = slv_clear(s, INT_TXO | INT_RXU | INT_RXO);
        break;
    case SLV_DMACR:
        v = s->dmacr;
        break;
    case SLV_DMATDLR:
        v = s->dmatdlr;
        break;
    case SLV_DMARDLR:
        v = s->dmardlr;
        break;
    case SLV_IDR:
        v = SLV_IDR_VALUE;
        break;
    case SLV_VERSION:
        v = SLV_VERSION_ID;
        break;
    }
    slv_update(s);
    return v >> (offset & 3) * 8;
}

/* FIFO levels: a value of 64 or more leaves the register as it was */
static void slv_set_level(uint32_t *reg, uint32_t v)
{
    if ((v & 0xff) < INGENIC_SSI_SLV_FIFO) {
        *reg = v & (INGENIC_SSI_SLV_FIFO - 1);
    }
}

/* A writable register's value, for narrow writes to merge into */
static uint32_t slv_reg(IngenicSsiSlvState *s, hwaddr reg)
{
    switch (reg) {
    case SLV_CTRLR:
        return s->ctrlr;
    case SLV_SSIENR:
        return s->ssienr;
    case SLV_MWCR:
        return s->mwcr;
    case SLV_TXFTLR:
        return s->txftlr;
    case SLV_RXFTLR:
        return s->rxftlr;
    case SLV_IMR:
        return s->imr;
    case SLV_DMACR:
        return s->dmacr;
    case SLV_DMATDLR:
        return s->dmatdlr;
    case SLV_DMARDLR:
        return s->dmardlr;
    }
    return 0;
}

static void ingenic_ssi_slv_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicSsiSlvState *s = opaque;
    hwaddr reg = offset & 0xfc;
    uint32_t v = value << (offset & 3) * 8;

    if (size < 4 && !(reg >= SLV_DR && reg < SLV_DR_END)) {
        v = deposit32(slv_reg(s, reg), (offset & 3) * 8, size * 8, value);
    }
    if (reg >= SLV_DR && reg < SLV_DR_END) {
        if (!slv_enabled(s)) {
            return;
        }
        if (s->tx_n == INGENIC_SSI_SLV_FIFO) {
            s->txo = true;
        } else {
            s->tx[(s->tx_rd + s->tx_n) % INGENIC_SSI_SLV_FIFO] = v;
            s->tx_n++;
        }
        slv_update(s);
        return;
    }
    switch (reg) {
    case SLV_CTRLR:
        if (slv_enabled(s)) {
            break;
        }
        v &= CTRLR_MASK;
        if (CTRLR_DFS(v) < 3) {
            v = deposit32(v, 16, 5, CTRLR_DFS(s->ctrlr));
        }
        s->ctrlr = v;
        break;
    case SLV_SSIENR:
        if ((v & 1) && !slv_enabled(s)) {
            s->ssienr = 1;
            if (s->selected) {
                slv_begin(s);
            }
        } else if (!(v & 1) && slv_enabled(s)) {
            s->ssienr = 0;
            s->tx_n = s->rx_n = 0;
            s->txo = s->rxu = s->rxo = false;
            s->bit = 0;
        }
        break;
    case SLV_MWCR:
        s->mwcr = v & 7;
        break;
    case SLV_TXFTLR:
        slv_set_level(&s->txftlr, v);
        break;
    case SLV_RXFTLR:
        slv_set_level(&s->rxftlr, v);
        break;
    case SLV_IMR:
        s->imr = v & INT_MASK;
        break;
    case SLV_DMACR:
        s->dmacr = v & (DMACR_TDMAE | DMACR_RDMAE);
        break;
    case SLV_DMATDLR:
        slv_set_level(&s->dmatdlr, v);
        break;
    case SLV_DMARDLR:
        slv_set_level(&s->dmardlr, v);
        break;
    }
    slv_update(s);
}

static const MemoryRegionOps ingenic_ssi_slv_ops = {
    .read = ingenic_ssi_slv_read,
    .write = ingenic_ssi_slv_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void ingenic_ssi_slv_reset_hold(Object *obj, ResetType type)
{
    IngenicSsiSlvState *s = INGENIC_SSI_SLV(obj);

    s->ctrlr = CTRLR_RESET;
    s->ssienr = s->mwcr = s->txftlr = s->rxftlr = 0;
    s->dmacr = s->dmatdlr = s->dmardlr = 0;
    s->imr = INT_MASK;
    s->txo = s->rxu = s->rxo = s->txe = false;
    s->tx_rd = s->tx_n = s->rx_rd = s->rx_n = 0;
    s->tx_shift = s->rx_shift = s->tx_last = s->bit = 0;
    if (!s->ported) {
        /* No pins: the chip select input reads active */
        s->selected = true;
    }
}

static void ingenic_ssi_slv_reset_exit(Object *obj, ResetType type)
{
    slv_update(INGENIC_SSI_SLV(obj));
}

static void ingenic_ssi_slv_init(Object *obj)
{
    IngenicSsiSlvState *s = INGENIC_SSI_SLV(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ssi_slv_ops, s,
                          TYPE_INGENIC_SSI_SLV, SLV_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), s->dreq, "dreq", 2);
}

static const VMStateDescription vmstate_ingenic_ssi_slv = {
    .name = TYPE_INGENIC_SSI_SLV,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrlr, IngenicSsiSlvState),
        VMSTATE_UINT32(ssienr, IngenicSsiSlvState),
        VMSTATE_UINT32(mwcr, IngenicSsiSlvState),
        VMSTATE_UINT32(txftlr, IngenicSsiSlvState),
        VMSTATE_UINT32(rxftlr, IngenicSsiSlvState),
        VMSTATE_UINT32(imr, IngenicSsiSlvState),
        VMSTATE_UINT32(dmacr, IngenicSsiSlvState),
        VMSTATE_UINT32(dmatdlr, IngenicSsiSlvState),
        VMSTATE_UINT32(dmardlr, IngenicSsiSlvState),
        VMSTATE_BOOL(txo, IngenicSsiSlvState),
        VMSTATE_BOOL(rxu, IngenicSsiSlvState),
        VMSTATE_BOOL(rxo, IngenicSsiSlvState),
        VMSTATE_BOOL(txe, IngenicSsiSlvState),
        VMSTATE_UINT32_ARRAY(tx, IngenicSsiSlvState, INGENIC_SSI_SLV_FIFO),
        VMSTATE_UINT32_ARRAY(rx, IngenicSsiSlvState, INGENIC_SSI_SLV_FIFO),
        VMSTATE_UINT32(tx_rd, IngenicSsiSlvState),
        VMSTATE_UINT32(tx_n, IngenicSsiSlvState),
        VMSTATE_UINT32(rx_rd, IngenicSsiSlvState),
        VMSTATE_UINT32(rx_n, IngenicSsiSlvState),
        VMSTATE_UINT32(tx_shift, IngenicSsiSlvState),
        VMSTATE_UINT32(rx_shift, IngenicSsiSlvState),
        VMSTATE_UINT32(tx_last, IngenicSsiSlvState),
        VMSTATE_UINT32(bit, IngenicSsiSlvState),
        VMSTATE_BOOL(selected, IngenicSsiSlvState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ssi_slv_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    dc->desc = "Ingenic SSI_SLV SPI slave";
    dc->vmsd = &vmstate_ingenic_ssi_slv;
    rc->phases.hold = ingenic_ssi_slv_reset_hold;
    rc->phases.exit = ingenic_ssi_slv_reset_exit;
}

/* The pins, on a master's SSI bus: CE active low */

static uint32_t ingenic_ssi_slv_port_transfer(SSIPeripheral *dev,
                                              uint32_t val)
{
    return slv_transfer(INGENIC_SSI_SLV_PORT(dev)->slv, val);
}

static int ingenic_ssi_slv_port_set_cs(SSIPeripheral *dev, bool level)
{
    slv_select(INGENIC_SSI_SLV_PORT(dev)->slv, !level);
    return 0;
}

static void ingenic_ssi_slv_port_realize(SSIPeripheral *dev, Error **errp)
{
    IngenicSsiSlvPort *p = INGENIC_SSI_SLV_PORT(dev);

    if (!p->slv) {
        bool ambiguous = false;
        Object *o = object_resolve_path_type("", TYPE_INGENIC_SSI_SLV,
                                             &ambiguous);

        if (!o) {
            error_setg(errp, "%s: %s; name one with slv=",
                       TYPE_INGENIC_SSI_SLV_PORT,
                       ambiguous ? "more than one SSI_SLV" : "no SSI_SLV");
            return;
        }
        p->slv = INGENIC_SSI_SLV(o);
    }
    if (p->slv->ported) {
        error_setg(errp, "%s: the SSI_SLV already has a port",
                   TYPE_INGENIC_SSI_SLV_PORT);
        return;
    }
    p->slv->ported = true;
}

static const Property ingenic_ssi_slv_port_properties[] = {
    DEFINE_PROP_LINK("slv", IngenicSsiSlvPort, slv, TYPE_INGENIC_SSI_SLV,
                     IngenicSsiSlvState *),
};

static const VMStateDescription vmstate_ingenic_ssi_slv_port = {
    .name = TYPE_INGENIC_SSI_SLV_PORT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_SSI_PERIPHERAL(parent_obj, IngenicSsiSlvPort),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ssi_slv_port_class_init(ObjectClass *klass,
                                            const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = ingenic_ssi_slv_port_realize;
    k->transfer = ingenic_ssi_slv_port_transfer;
    k->set_cs = ingenic_ssi_slv_port_set_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->desc = "Ingenic SSI_SLV pins on an SSI bus";
    dc->vmsd = &vmstate_ingenic_ssi_slv_port;
    device_class_set_props(dc, ingenic_ssi_slv_port_properties);
}

static const TypeInfo ingenic_ssi_slv_types[] = {
    {
        .name = TYPE_INGENIC_SSI_SLV,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicSsiSlvState),
        .instance_init = ingenic_ssi_slv_init,
        .class_init = ingenic_ssi_slv_class_init,
    }, {
        .name = TYPE_INGENIC_SSI_SLV_PORT,
        .parent = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(IngenicSsiSlvPort),
        .class_init = ingenic_ssi_slv_port_class_init,
    },
};

DEFINE_TYPES(ingenic_ssi_slv_types)
