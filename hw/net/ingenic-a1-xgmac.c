/*
 * Ingenic A1 XGMAC Ethernet controller
 *
 * The A1 GMAC is a Synopsys DesignWare dwxgmac2 MAC. This models enough
 * of it for the Linux stmmac (ingenic-dwmac) driver and U-Boot: the
 * version/feature registers, the MDIO bus with an attached PHY, and DMA
 * channel 0 with descriptor-based TX/RX and interrupts.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/net/ingenic-a1-xgmac.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "system/dma.h"
#include "system/address-spaces.h"

/* Control / status registers */
#define XGMAC_VERSION           0x0110
#define XGMAC_HW_FEATURE0       0x011c
#define XGMAC_HW_FEATURE1       0x0120
#define XGMAC_HW_FEATURE2       0x0124
#define XGMAC_HW_FEATURE3       0x0128
#define XGMAC_MDIO_ADDR         0x0200
#define XGMAC_MDIO_DATA         0x0204
#define XGMAC_MDIO_BUSY         (1U << 22)
#define XGMAC_MDIO_CMD_SHIFT    16
#define XGMAC_MDIO_CMD_WRITE    1
#define XGMAC_MDIO_CMD_READ     3

/* DMA block - channel 0 only (HW_FEATURE2 reports a single channel) */
#define XGMAC_DMA_MODE          0x3000
#define XGMAC_DMA_MODE_SWR      (1U << 0)
#define XGMAC_CH0_TXDESC_LADDR  0x3114
#define XGMAC_CH0_RXDESC_LADDR  0x311c
#define XGMAC_CH0_TXDESC_TAIL   0x3124
#define XGMAC_CH0_RXDESC_TAIL   0x312c
#define XGMAC_CH0_TXDESC_RING   0x3130
#define XGMAC_CH0_RXDESC_RING   0x3134
#define XGMAC_CH0_INT_EN        0x3138
#define XGMAC_CH0_STATUS        0x3160

/* DMA channel status / interrupt-enable bits (positions align) */
#define XGMAC_INT_TI            (1U << 0)
#define XGMAC_INT_RI            (1U << 6)
#define XGMAC_INT_NIS           (1U << 15)

/* Descriptor bits */
#define XGMAC_TDES3_OWN         (1U << 31)
#define XGMAC_RDES3_OWN         (1U << 31)
#define XGMAC_RDES3_LD          (1U << 28)
#define XGMAC_DESC_LEN_MASK     0x3fff

/* MII (PHY) register numbers */
#define MII_BMCR                0
#define MII_BMSR                1
#define MII_BMCR_RESET          (1U << 15)
#define MII_BMCR_ANRESTART      (1U << 9)

#define R(off)                  ((off) / 4)

static void xgmac_update_irq(IngenicA1XgmacState *s)
{
    uint32_t pending = s->regs[R(XGMAC_CH0_STATUS)] &
                       s->regs[R(XGMAC_CH0_INT_EN)];
    qemu_set_irq(s->irq, !!pending);
}

static void xgmac_phy_reset(IngenicA1XgmacState *s)
{
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    s->phy_regs[MII_BMCR]  = 0x1000;        /* autoneg enable */
    s->phy_regs[MII_BMSR]  = 0x796d;        /* link up, autoneg complete */
    s->phy_regs[2]         = 0x2000;        /* PHY ID1 */
    s->phy_regs[3]         = 0xa140;        /* PHY ID2 */
    s->phy_regs[4]         = 0x05e1;        /* ANAR */
    s->phy_regs[5]         = 0x45e1;        /* ANLPAR */
}

/* Process the TX descriptor ring for channel 0. */
static void xgmac_process_tx(IngenicA1XgmacState *s)
{
    uint32_t base = s->regs[R(XGMAC_CH0_TXDESC_LADDR)];
    uint32_t tail = s->regs[R(XGMAC_CH0_TXDESC_TAIL)];
    uint32_t ring_len = (s->regs[R(XGMAC_CH0_TXDESC_RING)] & 0x3ff) + 1;
    bool sent = false;

    if (!base || !ring_len) {
        return;
    }
    if (!s->tx_cur) {
        s->tx_cur = base;
    }

    for (uint32_t i = 0; i < ring_len; i++) {
        uint32_t phys = s->tx_cur & 0x1fffffff;
        uint32_t des[4];

        dma_memory_read(&address_space_memory, phys, des, sizeof(des),
                        MEMTXATTRS_UNSPECIFIED);
        if (!(des[3] & XGMAC_TDES3_OWN)) {
            break;
        }

        uint32_t buf = des[0] & 0x1fffffff;
        uint32_t len = des[2] & XGMAC_DESC_LEN_MASK;
        if (buf && len > 0 && len <= 2048) {
            uint8_t pkt[2048];
            dma_memory_read(&address_space_memory, buf, pkt, len,
                            MEMTXATTRS_UNSPECIFIED);
            qemu_send_packet(qemu_get_queue(s->nic), pkt, len);
        }

        des[3] &= ~XGMAC_TDES3_OWN;
        dma_memory_write(&address_space_memory, phys, des, sizeof(des),
                         MEMTXATTRS_UNSPECIFIED);
        sent = true;

        s->tx_cur += 16;
        if (s->tx_cur >= base + ring_len * 16) {
            s->tx_cur = base;
        }
        if (s->tx_cur == tail) {
            break;
        }
    }

    if (sent) {
        s->regs[R(XGMAC_CH0_STATUS)] |= XGMAC_INT_TI | XGMAC_INT_NIS;
        xgmac_update_irq(s);
    }
}

static bool xgmac_can_receive(NetClientState *nc)
{
    return true;
}

static ssize_t xgmac_receive(NetClientState *nc, const uint8_t *buf,
                             size_t size)
{
    IngenicA1XgmacState *s = qemu_get_nic_opaque(nc);
    uint32_t ring = s->regs[R(XGMAC_CH0_RXDESC_LADDR)];
    uint32_t ring_len = (s->regs[R(XGMAC_CH0_RXDESC_RING)] & 0x3ff) + 1;

    if (!ring || size > 2048) {
        return -1;
    }

    /* Find a descriptor the DMA owns and write the frame into it. */
    for (uint32_t i = 0; i < ring_len; i++) {
        uint32_t phys = (ring + i * 16) & 0x1fffffff;
        uint32_t des[4];

        dma_memory_read(&address_space_memory, phys, des, sizeof(des),
                        MEMTXATTRS_UNSPECIFIED);
        if (!(des[3] & XGMAC_RDES3_OWN)) {
            continue;
        }

        uint32_t bufaddr = des[0] & 0x1fffffff;
        if (!bufaddr) {
            continue;
        }
        dma_memory_write(&address_space_memory, bufaddr, buf, size,
                         MEMTXATTRS_UNSPECIFIED);

        /*
         * Hand the descriptor back: clear OWN, mark last descriptor,
         * store length. The stmmac driver strips a 4-byte FCS from the
         * reported length, so account for it (QEMU frames carry no FCS).
         */
        des[2] = 0;
        des[3] = XGMAC_RDES3_LD | (((uint32_t)size + 4) & XGMAC_DESC_LEN_MASK);
        dma_memory_write(&address_space_memory, phys, des, sizeof(des),
                         MEMTXATTRS_UNSPECIFIED);

        s->regs[R(XGMAC_CH0_STATUS)] |= XGMAC_INT_RI | XGMAC_INT_NIS;
        xgmac_update_irq(s);
        return size;
    }
    return 0;
}

static void xgmac_mdio_xfer(IngenicA1XgmacState *s, uint32_t data)
{
    uint32_t addr = s->regs[R(XGMAC_MDIO_ADDR)];
    uint32_t phy = (addr >> 16) & 0x1f;
    uint32_t reg = addr & 0x1f;
    uint32_t cmd = (data >> XGMAC_MDIO_CMD_SHIFT) & 0x3;

    /* Only the PHY at MDIO address 0 is present. */
    if (phy != 0) {
        s->regs[R(XGMAC_MDIO_DATA)] = 0xffff;
        return;
    }

    if (cmd == XGMAC_MDIO_CMD_READ) {
        s->regs[R(XGMAC_MDIO_DATA)] = s->phy_regs[reg];
    } else if (cmd == XGMAC_MDIO_CMD_WRITE) {
        uint16_t val = data & 0xffff;
        switch (reg) {
        case MII_BMSR:
        case 2:
        case 3:
            break;  /* read-only */
        case MII_BMCR:
            /* reset and autoneg-restart complete immediately */
            s->phy_regs[reg] = val & ~(MII_BMCR_RESET | MII_BMCR_ANRESTART);
            break;
        default:
            s->phy_regs[reg] = val;
            break;
        }
        s->regs[R(XGMAC_MDIO_DATA)] = data & ~XGMAC_MDIO_BUSY;
    } else {
        s->regs[R(XGMAC_MDIO_DATA)] = data & ~XGMAC_MDIO_BUSY;
    }
}

static uint64_t xgmac_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicA1XgmacState *s = opaque;

    if (offset >= INGENIC_A1_XGMAC_IOSIZE) {
        return 0;
    }

    switch (offset) {
    case XGMAC_VERSION:
        /* User ID 0x21, Synopsys ID 0x21 (>= DWXGMAC_CORE_2_10) */
        return 0x00002121;
    case XGMAC_HW_FEATURE0:
        return 0x000203e7;
    case XGMAC_HW_FEATURE1:
        return 0x03110000;
    case XGMAC_HW_FEATURE2:
    case XGMAC_HW_FEATURE3:
        return 0x00000000;
    case XGMAC_MDIO_DATA:
        /* BUSY always clear - transactions complete synchronously */
        return s->regs[R(offset)] & ~XGMAC_MDIO_BUSY;
    case XGMAC_DMA_MODE:
        /* software reset bit self-clears */
        return s->regs[R(offset)] & ~XGMAC_DMA_MODE_SWR;
    default:
        return s->regs[R(offset)];
    }
}

static void xgmac_write(void *opaque, hwaddr offset, uint64_t value,
                        unsigned size)
{
    IngenicA1XgmacState *s = opaque;
    uint32_t val = value;

    if (offset >= INGENIC_A1_XGMAC_IOSIZE) {
        return;
    }

    switch (offset) {
    case XGMAC_MDIO_DATA:
        /* Writing the data register triggers the MDIO transaction. */
        xgmac_mdio_xfer(s, val);
        break;
    case XGMAC_CH0_STATUS:
        /* write-1-to-clear */
        s->regs[R(offset)] &= ~val;
        xgmac_update_irq(s);
        break;
    case XGMAC_CH0_INT_EN:
        s->regs[R(offset)] = val;
        xgmac_update_irq(s);
        break;
    case XGMAC_CH0_TXDESC_TAIL:
        s->regs[R(offset)] = val;
        xgmac_process_tx(s);
        break;
    default:
        s->regs[R(offset)] = val;
        break;
    }
}

static const MemoryRegionOps xgmac_ops = {
    .read = xgmac_read,
    .write = xgmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void xgmac_reset_hold(Object *obj, ResetType type)
{
    IngenicA1XgmacState *s = INGENIC_A1_XGMAC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    s->tx_cur = 0;
    xgmac_phy_reset(s);
    qemu_set_irq(s->irq, 0);
}

static void xgmac_realize(DeviceState *dev, Error **errp)
{
    IngenicA1XgmacState *s = INGENIC_A1_XGMAC(dev);

    static NetClientInfo net_info = {
        .type = NET_CLIENT_DRIVER_NIC,
        .size = sizeof(NICState),
        .can_receive = xgmac_can_receive,
        .receive = xgmac_receive,
    };

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&net_info, &s->conf, TYPE_INGENIC_A1_XGMAC,
                          dev->id, &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void xgmac_init(Object *obj)
{
    IngenicA1XgmacState *s = INGENIC_A1_XGMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &xgmac_ops, s,
                          TYPE_INGENIC_A1_XGMAC, INGENIC_A1_XGMAC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_xgmac = {
    .name = TYPE_INGENIC_A1_XGMAC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicA1XgmacState,
                             INGENIC_A1_XGMAC_IOSIZE / 4),
        VMSTATE_UINT16_ARRAY(phy_regs, IngenicA1XgmacState, 32),
        VMSTATE_UINT32(tx_cur, IngenicA1XgmacState),
        VMSTATE_END_OF_LIST()
    },
};

static const Property xgmac_props[] = {
    DEFINE_NIC_PROPERTIES(IngenicA1XgmacState, conf),
};

static void xgmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = xgmac_realize;
    dc->vmsd = &vmstate_xgmac;
    device_class_set_props(dc, xgmac_props);
    rc->phases.hold = xgmac_reset_hold;
}

static const TypeInfo xgmac_type_info = {
    .name = TYPE_INGENIC_A1_XGMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1XgmacState),
    .instance_init = xgmac_init,
    .class_init = xgmac_class_init,
};

static void xgmac_register_types(void)
{
    type_register_static(&xgmac_type_info);
}

type_init(xgmac_register_types)
