/*
 * Ingenic T31 I2C controller (Synopsys DesignWare derivative)
 *
 * No actual I2C bus is wired; every addressed transfer NAKs at the
 * address phase. The kernel's i2c-v12-jz driver queues a transfer,
 * unmasks completion interrupts via I2C_INTM, then waits on a
 * completion. We respond by raising I2C_INTST_TXABT (with TXABRT
 * source = address NACK) and asserting the IRQ to INTC source 60
 * (I2C0) or 59 (I2C1). The driver returns -ENXIO and the bus scan
 * proceeds.
 *
 * Just enough state is tracked to make the kernel's polling paths
 * (read I2C_STA before the IRQ fires, read I2C_TXABRT to learn the
 * abort source, read clear-on-read registers) behave coherently.
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
#include "hw/i2c/ingenic-t31-i2c.h"

/* Register offsets (Synopsys DesignWare I2C derivative) */
#define I2C_CTRL        0x00
#define I2C_TAR         0x04
#define I2C_SAR         0x08
#define I2C_DC          0x10
#define I2C_SHCNT       0x14
#define I2C_SLCNT       0x18
#define I2C_FHCNT       0x1c
#define I2C_FLCNT       0x20
#define I2C_INTST       0x2c
#define I2C_INTM        0x30
#define I2C_RXTL        0x38
#define I2C_TXTL        0x3c
#define I2C_CINTR       0x40    /* clear all interrupts (read-to-clear) */
#define I2C_CRXUF       0x44
#define I2C_CRXOF       0x48
#define I2C_CTXOF       0x4c
#define I2C_CRXREQ      0x50
#define I2C_CTXABRT     0x54    /* clear TX abort */
#define I2C_CRXDONE     0x58
#define I2C_CACT        0x5c
#define I2C_CSTP        0x60    /* clear stop bit */
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

/* I2C_STA bits */
#define STA_TFNF        (1u << 1)   /* TX FIFO not full */
#define STA_TFE         (1u << 2)   /* TX FIFO empty */
#define STA_RFNE        (1u << 3)   /* RX FIFO not empty */
#define STA_MSTACT      (1u << 5)   /* Master active */

/* I2C_INTST bits */
#define INTST_TXABT     (1u << 6)
#define INTST_RXDN      (1u << 7)
#define INTST_TXEMP     (1u << 4)
#define INTST_RXFL      (1u << 2)
#define INTST_ISTP      (1u << 9)

/* I2C_INTM bit positions (mirror INTST) */
#define INTM_MIGC       (1u << 11)
#define INTM_MISTT      (1u << 10)
#define INTM_MISTP      (1u << 9)
#define INTM_MIACT      (1u << 8)
#define INTM_MRXDN      (1u << 7)
#define INTM_MTXABT     (1u << 6)
#define INTM_MRDREQ     (1u << 5)
#define INTM_MTXEMP     (1u << 4)
#define INTM_MTXOF      (1u << 3)
#define INTM_MRXFL      (1u << 2)
#define INTM_MRXOF      (1u << 1)
#define INTM_MRXUF      (1u << 0)

/*
 * I2C_TXABRT abort source bits. Bits 1..15 indicate device-NACK
 * conditions. The kernel maps TXABRT in (0x1, 0x10) -> -ENXIO.
 * Bit 0 = "abrt 7b address NOACK".
 */
#define TXABRT_7B_ADDR_NOACK    (1u << 0)

static void i2c_update_irq(IngenicT31I2cState *s)
{
    qemu_set_irq(s->irq, (s->intst & s->intm) != 0);
}

/*
 * Called when the driver writes I2C_INTM with a non-zero set of
 * completion bits. Synthesize the abort and raise the requested
 * completion bits so wait_for_completion_timeout returns.
 */
static void i2c_simulate_xfer_end(IngenicT31I2cState *s)
{
    /* Always treat the addressed device as absent. */
    s->txabrt = TXABRT_7B_ADDR_NOACK;
    s->intst |= INTST_TXABT;
    /* Raise whatever completion the driver is waiting on. */
    if (s->intm & INTM_MTXEMP) {
        s->intst |= INTST_TXEMP;
    }
    if (s->intm & INTM_MRXFL) {
        s->intst |= INTST_RXFL;
    }
    if (s->intm & INTM_MISTP) {
        s->intst |= INTST_ISTP;
    }
    i2c_update_irq(s);
}

static uint64_t ingenic_t31_i2c_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT31I2cState *s = INGENIC_T31_I2C(opaque);
    uint32_t v;

    switch (offset) {
    case I2C_CTRL:
        return s->ctrl;
    case I2C_TAR:
        return s->tar;
    case I2C_INTM:
        return s->intm;
    case I2C_INTST:
        return s->intst;
    case I2C_ENB:
    case I2C_ENSTA:
        return s->enb & 1;
    case I2C_TXABRT:
        return s->txabrt;

    case I2C_STA:
        /* Always idle: TX FIFO empty + not full, RX empty. */
        return STA_TFE | STA_TFNF;

    case I2C_TXFLR:
    case I2C_RXFLR:
        return 0;

    /* Read-to-clear individual interrupt flags */
    case I2C_CINTR:
        v = s->intst;
        s->intst = 0;
        i2c_update_irq(s);
        return v;
    case I2C_CTXABRT:
        v = s->txabrt;
        s->txabrt = 0;
        s->intst &= ~INTST_TXABT;
        i2c_update_irq(s);
        return v;
    case I2C_CSTP:
        v = !!(s->intst & INTST_ISTP);
        s->intst &= ~INTST_ISTP;
        i2c_update_irq(s);
        return v;
    case I2C_CTXOF:
        s->intst &= ~(1u << 3);
        i2c_update_irq(s);
        return 0;
    case I2C_CRXOF:
    case I2C_CRXUF:
    case I2C_CRXREQ:
    case I2C_CRXDONE:
    case I2C_CACT:
    case I2C_CSTT:
    case I2C_CGC:
        return 0;

    case I2C_DC:
        /* No real device -> RX FIFO is empty. */
        return 0;

    default:
        return 0;
    }
}

static void ingenic_t31_i2c_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT31I2cState *s = INGENIC_T31_I2C(opaque);
    uint32_t v = (uint32_t)value;

    switch (offset) {
    case I2C_CTRL:
        s->ctrl = v;
        break;
    case I2C_TAR:
        s->tar = v;
        s->xfer_active = false;
        s->txabrt = 0;
        s->intst = 0;
        i2c_update_irq(s);
        break;
    case I2C_DC:
        /* The driver writes data bytes here, then enables INTM
         * to wait for completion. We mark a transfer in progress
         * and let i2c_simulate_xfer_end synthesize the result. */
        s->xfer_active = true;
        break;
    case I2C_INTM:
        s->intm = v;
        if (v && s->xfer_active) {
            i2c_simulate_xfer_end(s);
            s->xfer_active = false;
        } else {
            i2c_update_irq(s);
        }
        break;
    case I2C_ENB:
        s->enb = v;
        break;

    /* Speed/timing/threshold registers - accept and ignore. */
    case I2C_SHCNT: case I2C_SLCNT: case I2C_FHCNT: case I2C_FLCNT:
    case I2C_RXTL: case I2C_TXTL:
    case I2C_SDAHD: case I2C_SDASU: case I2C_FLT:
    case I2C_DMACR: case I2C_DMATDLR: case I2C_DMARDLR:
    case I2C_SAR:
        break;

    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write 0x%03" HWADDR_PRIx
                      " val 0x%08x\n", __func__, offset, v);
        break;
    }
}

static const MemoryRegionOps ingenic_t31_i2c_ops = {
    .read = ingenic_t31_i2c_read,
    .write = ingenic_t31_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_i2c_reset_hold(Object *obj, ResetType type)
{
    IngenicT31I2cState *s = INGENIC_T31_I2C(obj);

    s->ctrl = 0;
    s->tar = 0;
    s->intm = 0;
    s->intst = 0;
    s->enb = 0;
    s->txabrt = 0;
    s->xfer_active = false;
    qemu_set_irq(s->irq, 0);
}

static void ingenic_t31_i2c_init(Object *obj)
{
    IngenicT31I2cState *s = INGENIC_T31_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_i2c_ops, s,
                          TYPE_INGENIC_T31_I2C,
                          INGENIC_T31_I2C_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_t31_i2c_class_init(ObjectClass *oc, const void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    rc->phases.hold = ingenic_t31_i2c_reset_hold;
}

static const TypeInfo ingenic_t31_i2c_type_info = {
    .name = TYPE_INGENIC_T31_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31I2cState),
    .instance_init = ingenic_t31_i2c_init,
    .class_init = ingenic_t31_i2c_class_init,
};

static void ingenic_t31_i2c_register_types(void)
{
    type_register_static(&ingenic_t31_i2c_type_info);
}

type_init(ingenic_t31_i2c_register_types)
