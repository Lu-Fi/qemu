/*
 * Ingenic I2C controller (a Synopsys DesignWare core)
 *
 * A master on a QEMU I2C bus named after the controller (i2c0, i2c1, ...),
 * for the vendor kernels' i2c-v12-jz (3.10) and i2c-ingenic (4.4) drivers.
 * Those keep the controller enabled from one transfer to the next and
 * write I2C_TAR while it is: so a command after a STOP starts a new
 * transfer, a read after writes turns the bus round with a repeated START
 * (I2C_CTRL.REST), and I2C_DC's STOP bit (9) ends one. Commands run as they
 * are written, so the TX FIFO always reads empty; reads queue in a 64-entry
 * RX FIFO.
 *
 * An address or data NACK aborts the transfer as the core does: TX_ABRT
 * with the source in I2C_TXABRT, the bus released, and further commands
 * dropped until I2C_CTXABRT (or I2C_CINTR) is read.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/i2c/ingenic-i2c.h"

/* Registers, as the DesignWare core lays them out */
#define I2C_CTRL        0x00
#define I2C_TAR         0x04
#define I2C_SAR         0x08
#define I2C_DC          0x10
#define I2C_SHCNT       0x14
#define I2C_FLCNT       0x20
#define I2C_INTST       0x2c
#define I2C_INTM        0x30
#define I2C_RINTST      0x34
#define I2C_RXTL        0x38
#define I2C_TXTL        0x3c
#define I2C_CINTR       0x40
#define I2C_CRXUF       0x44
#define I2C_CRXOF       0x48
#define I2C_CTXOF       0x4c
#define I2C_CRXREQ      0x50
#define I2C_CTXABRT     0x54
#define I2C_CRXDONE     0x58
#define I2C_CACT        0x5c
#define I2C_CSTP        0x60
#define I2C_CSTT        0x64
#define I2C_CGC         0x68
#define I2C_ENB         0x6c
#define I2C_STA         0x70
#define I2C_TXFLR       0x74
#define I2C_RXFLR       0x78
#define I2C_SDAHD       0x7c
#define I2C_TXABRT      0x80
#define I2C_DMACR       0x88
#define I2C_DMATDLR     0x8c
#define I2C_DMARDLR     0x90
#define I2C_SDASU       0x94
#define I2C_ACKGC       0x98
#define I2C_ENSTA       0x9c
#define I2C_FLT         0xa0

#define DC_READ         BIT(8)
#define DC_STP          BIT(9)
#define DC_REST         BIT(10)

#define STA_ACT         BIT(0)
#define STA_TFNF        BIT(1)
#define STA_TFE         BIT(2)
#define STA_RFNE        BIT(3)
#define STA_RFF         BIT(4)
#define STA_MSTACT      BIT(5)

/* Interrupts: INTST is RINTST under INTM */
#define INT_RXUF        BIT(0)
#define INT_RXOF        BIT(1)
#define INT_RXFL        BIT(2)          /* level: RX holds more than RXTL */
#define INT_TXOF        BIT(3)
#define INT_TXEMP       BIT(4)          /* level: TX at or under TXTL */
#define INT_RDREQ       BIT(5)
#define INT_TXABT       BIT(6)
#define INT_RXDN        BIT(7)
#define INT_ACT         BIT(8)
#define INT_ISTP        BIT(9)
#define INT_ISTT        BIT(10)
#define INT_IGC         BIT(11)
#define INT_LATCHED     (INT_RXUF | INT_RXOF | INT_TXOF | INT_RDREQ | \
                         INT_TXABT | INT_RXDN | INT_ACT | INT_ISTP | \
                         INT_ISTT | INT_IGC)

#define ABRT_7B_ADDR_NOACK  BIT(0)
#define ABRT_TXDATA_NOACK   BIT(3)

enum { I2C_IDLE, I2C_WRITING, I2C_READING };

static void i2c_update(IngenicI2cState *s)
{
    uint32_t raw = s->raw & INT_LATCHED;

    if (s->enb & 1) {
        raw |= INT_TXEMP;
        raw |= s->rx_n > s->rxtl ? INT_RXFL : 0;
    }
    s->raw = raw;
    qemu_set_irq(s->irq, !!(s->raw & s->intm));
}

static void i2c_release(IngenicI2cState *s)
{
    if (s->state != I2C_IDLE) {
        i2c_end_transfer(s->bus);
        s->state = I2C_IDLE;
        s->raw |= INT_ISTP;
    }
}

static void i2c_abort(IngenicI2cState *s, uint32_t source)
{
    s->txabrt |= source;
    s->raw |= INT_TXABT;
    i2c_release(s);
}

static void i2c_command(IngenicI2cState *s, uint32_t v)
{
    bool rd = v & DC_READ;

    /* Disabled, or flushed by an abort not yet cleared */
    if (!(s->enb & 1) || (s->raw & INT_TXABT)) {
        return;
    }
    s->raw |= INT_ACT;
    if (s->state == I2C_IDLE || (v & DC_REST) ||
        rd != (s->state == I2C_READING)) {
        s->raw |= INT_ISTT;
        if (i2c_start_transfer(s->bus, extract32(s->tar, 0, 7), rd)) {
            /* No device: the address NACKs, nothing is on the bus */
            s->state = I2C_IDLE;
            i2c_abort(s, ABRT_7B_ADDR_NOACK);
            return;
        }
        s->state = rd ? I2C_READING : I2C_WRITING;
    }
    if (rd) {
        uint8_t b = i2c_recv(s->bus);

        if (s->rx_n < INGENIC_I2C_FIFO) {
            s->rx[(s->rx_rd + s->rx_n++) % INGENIC_I2C_FIFO] = b;
        } else {
            s->raw |= INT_RXOF;
        }
        if (v & DC_STP) {
            i2c_nack(s->bus);
        }
    } else if (i2c_send(s->bus, extract32(v, 0, 8))) {
        i2c_abort(s, ABRT_TXDATA_NOACK);
        return;
    }
    if (v & DC_STP) {
        i2c_release(s);
    }
}

static uint64_t ingenic_i2c_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicI2cState *s = INGENIC_I2C(opaque);
    uint32_t v = 0;

    switch (offset) {
    case I2C_CTRL:
        return s->ctrl;
    case I2C_TAR:
        return s->tar;
    case I2C_SAR:
        return s->sar;
    case I2C_DC:
        if (!s->rx_n) {
            s->raw |= INT_RXUF;
        } else {
            v = s->rx[s->rx_rd];
            s->rx_rd = (s->rx_rd + 1) % INGENIC_I2C_FIFO;
            s->rx_n--;
        }
        break;
    case I2C_SHCNT ... I2C_FLCNT:
        return s->timing[(offset - I2C_SHCNT) / 4];
    case I2C_INTST:
        return s->raw & s->intm;
    case I2C_INTM:
        return s->intm;
    case I2C_RINTST:
        return s->raw;
    case I2C_RXTL:
        return s->rxtl;
    case I2C_TXTL:
        return s->txtl;
    case I2C_CINTR:
        v = !!(s->raw & s->intm & INT_LATCHED);
        s->raw &= ~INT_LATCHED;
        s->txabrt = 0;
        break;
    case I2C_CRXUF ... I2C_CGC: {
        /* Each clears one interrupt, in INT bit order from RXUF */
        static const uint32_t bit[] = {
            INT_RXUF, INT_RXOF, INT_TXOF, INT_RDREQ, INT_TXABT, INT_RXDN,
            INT_ACT, INT_ISTP, INT_ISTT, INT_IGC,
        };
        uint32_t b = bit[(offset - I2C_CRXUF) / 4];

        v = !!(s->raw & b);
        if (b == INT_ACT && s->state != I2C_IDLE) {
            break;      /* ACTIVITY stays while the bus is busy */
        }
        s->raw &= ~b;
        if (b == INT_TXABT) {
            s->txabrt = 0;
        }
        break;
    }
    case I2C_ENB:
        return s->enb;
    case I2C_STA:
        v = STA_TFNF | STA_TFE;
        v |= s->state != I2C_IDLE ? STA_ACT | STA_MSTACT : 0;
        v |= s->rx_n ? STA_RFNE : 0;
        v |= s->rx_n == INGENIC_I2C_FIFO ? STA_RFF : 0;
        return v;
    case I2C_TXFLR:
        return 0;
    case I2C_RXFLR:
        return s->rx_n;
    case I2C_SDAHD:
        return s->sdahd;
    case I2C_TXABRT:
        return s->txabrt;
    case I2C_DMACR:
        return s->dmacr;
    case I2C_DMATDLR:
        return s->dmatdlr;
    case I2C_DMARDLR:
        return s->dmardlr;
    case I2C_SDASU:
        return s->sdasu;
    case I2C_ACKGC:
        return s->ackgc;
    case I2C_ENSTA:
        return s->enb & 1;
    case I2C_FLT:
        return s->flt;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read of 0x%03" HWADDR_PRIx "\n",
                      __func__, offset);
        return 0;
    }
    i2c_update(s);
    return v;
}

static void ingenic_i2c_write(void *opaque, hwaddr offset, uint64_t value,
                              unsigned size)
{
    IngenicI2cState *s = INGENIC_I2C(opaque);
    uint32_t v = value;

    switch (offset) {
    case I2C_CTRL:
        s->ctrl = v & 0x7ff;
        break;
    case I2C_TAR:
        s->tar = v & 0x1fff;
        break;
    case I2C_SAR:
        s->sar = v & 0x3ff;
        break;
    case I2C_DC:
        i2c_command(s, v);
        break;
    case I2C_SHCNT ... I2C_FLCNT:
        s->timing[(offset - I2C_SHCNT) / 4] = v & 0xffff;
        break;
    case I2C_INTM:
        s->intm = v & 0xfff;
        break;
    case I2C_RXTL:
        s->rxtl = MIN(v & 0xff, INGENIC_I2C_FIFO - 1);
        break;
    case I2C_TXTL:
        s->txtl = MIN(v & 0xff, INGENIC_I2C_FIFO - 1);
        break;
    case I2C_ENB:
        s->enb = v & 1;
        if (!(v & 1)) {
            i2c_release(s);
            s->rx_rd = s->rx_n = 0;
            s->raw &= ~INT_ISTP;
        }
        break;
    case I2C_SDAHD:
        s->sdahd = v;
        break;
    case I2C_DMACR:
        s->dmacr = v & 3;
        break;
    case I2C_DMATDLR:
        s->dmatdlr = v & 0xff;
        break;
    case I2C_DMARDLR:
        s->dmardlr = v & 0xff;
        break;
    case I2C_SDASU:
        s->sdasu = v & 0xff;
        break;
    case I2C_ACKGC:
        s->ackgc = v & 1;
        break;
    case I2C_FLT:
        s->flt = v & 0xff;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write of 0x%08x to 0x%03"
                      HWADDR_PRIx "\n", __func__, v, offset);
        return;
    }
    i2c_update(s);
}

static const MemoryRegionOps ingenic_i2c_ops = {
    .read = ingenic_i2c_read,
    .write = ingenic_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_i2c_reset_hold(Object *obj, ResetType type)
{
    IngenicI2cState *s = INGENIC_I2C(obj);

    if (s->state != I2C_IDLE) {
        i2c_end_transfer(s->bus);
        s->state = I2C_IDLE;
    }
    s->ctrl = 0x7d;
    s->tar = 0x55;
    s->sar = 0x55;
    s->intm = 0x8ff;
    s->raw = 0;
    s->rxtl = s->txtl = 0;
    s->enb = 0;
    s->txabrt = 0;
    memset(s->timing, 0, sizeof(s->timing));
    s->sdahd = s->sdasu = s->dmacr = s->dmatdlr = s->dmardlr = 0;
    s->ackgc = s->flt = 0;
    s->rx_rd = s->rx_n = 0;
}

static void ingenic_i2c_reset_exit(Object *obj, ResetType type)
{
    i2c_update(INGENIC_I2C(obj));
}

static void ingenic_i2c_realize(DeviceState *dev, Error **errp)
{
    IngenicI2cState *s = INGENIC_I2C(dev);

    /* The SoC's child name: i2c0, i2c1, ... for -device ...,bus=i2c0 */
    s->bus = i2c_init_bus(dev,
                          object_get_canonical_path_component(OBJECT(dev)));
}

static void ingenic_i2c_init(Object *obj)
{
    IngenicI2cState *s = INGENIC_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_i2c_ops, s,
                          TYPE_INGENIC_I2C, INGENIC_I2C_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_ingenic_i2c = {
    .name = TYPE_INGENIC_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl, IngenicI2cState),
        VMSTATE_UINT32(tar, IngenicI2cState),
        VMSTATE_UINT32(sar, IngenicI2cState),
        VMSTATE_UINT32(intm, IngenicI2cState),
        VMSTATE_UINT32(raw, IngenicI2cState),
        VMSTATE_UINT32(rxtl, IngenicI2cState),
        VMSTATE_UINT32(txtl, IngenicI2cState),
        VMSTATE_UINT32(enb, IngenicI2cState),
        VMSTATE_UINT32(txabrt, IngenicI2cState),
        VMSTATE_UINT32_ARRAY(timing, IngenicI2cState, 4),
        VMSTATE_UINT32(sdahd, IngenicI2cState),
        VMSTATE_UINT32(sdasu, IngenicI2cState),
        VMSTATE_UINT32(dmacr, IngenicI2cState),
        VMSTATE_UINT32(dmatdlr, IngenicI2cState),
        VMSTATE_UINT32(dmardlr, IngenicI2cState),
        VMSTATE_UINT32(ackgc, IngenicI2cState),
        VMSTATE_UINT32(flt, IngenicI2cState),
        VMSTATE_UINT8_ARRAY(rx, IngenicI2cState, INGENIC_I2C_FIFO),
        VMSTATE_UINT32(rx_rd, IngenicI2cState),
        VMSTATE_UINT32(rx_n, IngenicI2cState),
        VMSTATE_UINT8(state, IngenicI2cState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_i2c_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_i2c_realize;
    dc->vmsd = &vmstate_ingenic_i2c;
    rc->phases.hold = ingenic_i2c_reset_hold;
    rc->phases.exit = ingenic_i2c_reset_exit;
}

static const TypeInfo ingenic_i2c_type_info = {
    .name = TYPE_INGENIC_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicI2cState),
    .instance_init = ingenic_i2c_init,
    .class_init = ingenic_i2c_class_init,
};

static void ingenic_i2c_register_types(void)
{
    type_register_static(&ingenic_i2c_type_info);
}

type_init(ingenic_i2c_register_types)
