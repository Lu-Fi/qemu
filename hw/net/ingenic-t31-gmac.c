/*
 * Ingenic T31 GMAC (Synopsys DesignWare MAC) emulation
 *
 * Implements the Synopsys DesignWare GMAC register interface with MDIO
 * PHY emulation and DMA descriptor-based TX/RX for U-Boot networking.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "migration/vmstate.h"
#include "hw/net/ingenic-t31-gmac.h"
#include "hw/net/mii.h"
#include "net/eth.h"
#include "exec/cpu-common.h"

/* MAC register offsets */
#define MAC_CONFIG          0x0000
#define MAC_FRAME_FILTER    0x0004
#define MAC_HASH_HI         0x0008
#define MAC_HASH_LO         0x000C
#define MAC_MII_ADDR        0x0010
#define MAC_MII_DATA        0x0014
#define MAC_FLOW_CTRL       0x0018
#define MAC_ADDR0_HI        0x0040
#define MAC_ADDR0_LO        0x0044

/* MAC_MII_ADDR bits */
#define MII_ADDR_BUSY       (1 << 0)
#define MII_ADDR_WRITE      (1 << 1)
#define MII_ADDR_REG_SHIFT  6
#define MII_ADDR_REG_MASK   (0x1F << 6)
#define MII_ADDR_PHY_SHIFT  11
#define MII_ADDR_PHY_MASK   (0x1F << 11)

/* DMA register offsets (from DMA base at 0x1000) */
#define DMA_BUS_MODE        0x0000
#define DMA_TX_POLL         0x0004
#define DMA_RX_POLL         0x0008
#define DMA_RX_BASE_ADDR    0x000C
#define DMA_TX_BASE_ADDR    0x0010
#define DMA_STATUS          0x0014
#define DMA_CONTROL         0x0018
#define DMA_INTR_ENA        0x001C
#define DMA_CUR_TX_DESC     0x0048
#define DMA_CUR_RX_DESC     0x004C
#define DMA_CUR_TX_BUF      0x0050
#define DMA_CUR_RX_BUF      0x0054

/* DMA_BUS_MODE bits */
#define DMA_BUS_MODE_SWR    (1 << 0)

/* DMA_CONTROL bits */
#define DMA_CONTROL_ST      (1 << 13)
#define DMA_CONTROL_SR      (1 << 1)

/* DMA_STATUS bits */
#define DMA_STATUS_TI       (1 << 0)
#define DMA_STATUS_RI       (1 << 6)
#define DMA_STATUS_NIS      (1 << 16)

/* DMA descriptor bits */
#define TDES0_OWN           (1u << 31)
#define TDES0_FS            (1 << 28)
#define TDES0_LS            (1 << 29)
#define TDES1_SIZE1_MASK    0x7FF
#define RDES0_OWN           (1u << 31)
#define RDES0_FS            (1 << 9)
#define RDES0_LS            (1 << 8)
#define RDES0_FL_SHIFT      16
#define RDES1_SIZE1_MASK    0x7FF
#define RDES1_RER           (1 << 25)

#define DMA_BASE_OFFSET     0x1000
#define MAC_IDX(off)        ((off) / 4)
#define DMA_IDX(off)        ((off) / 4)

static void ingenic_t31_gmac_mdio_read(IngenicT31GmacState *s)
{
    uint32_t addr = s->mac_regs[MAC_IDX(MAC_MII_ADDR)];
    uint32_t phy = (addr & MII_ADDR_PHY_MASK) >> MII_ADDR_PHY_SHIFT;
    uint32_t reg = (addr & MII_ADDR_REG_MASK) >> MII_ADDR_REG_SHIFT;

    if (phy == 0 && reg < INGENIC_T31_GMAC_PHY_REGS) {
        s->mac_regs[MAC_IDX(MAC_MII_DATA)] = s->phy_regs[reg];
    } else {
        s->mac_regs[MAC_IDX(MAC_MII_DATA)] = 0xFFFF;
    }

    s->mac_regs[MAC_IDX(MAC_MII_ADDR)] &= ~MII_ADDR_BUSY;
}

static void ingenic_t31_gmac_mdio_write(IngenicT31GmacState *s)
{
    uint32_t addr = s->mac_regs[MAC_IDX(MAC_MII_ADDR)];
    uint32_t phy = (addr & MII_ADDR_PHY_MASK) >> MII_ADDR_PHY_SHIFT;
    uint32_t reg = (addr & MII_ADDR_REG_MASK) >> MII_ADDR_REG_SHIFT;

    if (phy == 0 && reg < INGENIC_T31_GMAC_PHY_REGS) {
        s->phy_regs[reg] = s->mac_regs[MAC_IDX(MAC_MII_DATA)] & 0xFFFF;
    }

    s->mac_regs[MAC_IDX(MAC_MII_ADDR)] &= ~MII_ADDR_BUSY;
}

#define TDES0_TER           (1 << 25)
#define RDES0_RER_ENH       (1 << 25)
#define NUM_TX_DESCS        16
#define DESC_SIZE           16

static void ingenic_t31_gmac_do_tx(IngenicT31GmacState *s)
{
    uint32_t base = s->dma_regs[DMA_IDX(DMA_TX_BASE_ADDR)];
    uint8_t buf[2048];
    int count;

    if (!base) {
        return;
    }

    for (count = 0; count < NUM_TX_DESCS; count++) {
        uint32_t desc_addr = s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)];
        uint32_t des[4];
        hwaddr phys;

        if (!desc_addr) {
            desc_addr = base;
        }
        phys = desc_addr & 0x1FFFFFFF;

        cpu_physical_memory_read(phys, des, DESC_SIZE);

        if (!(des[0] & TDES0_OWN)) {
            break;
        }

        uint32_t len = des[1] & TDES1_SIZE1_MASK;
        hwaddr buf_phys = des[2] & 0x1FFFFFFF;

        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        if (len > 0 && buf_phys) {
            cpu_physical_memory_read(buf_phys, buf, len);

            if ((des[0] & TDES0_FS) && (des[0] & TDES0_LS)) {
                qemu_send_packet(qemu_get_queue(s->nic), buf, len);
            }
        }

        des[0] &= ~TDES0_OWN;
        cpu_physical_memory_write(phys, des, 4);

        s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_TI | DMA_STATUS_NIS;

        if (des[0] & TDES0_TER) {
            s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = base;
        } else {
            s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = desc_addr + DESC_SIZE;
        }
    }
}

static bool ingenic_t31_gmac_can_receive(NetClientState *nc)
{
    return true;
}

static ssize_t ingenic_t31_gmac_receive(NetClientState *nc,
                                        const uint8_t *buf, size_t len)
{
    IngenicT31GmacState *s = qemu_get_nic_opaque(nc);
    uint32_t base = s->dma_regs[DMA_IDX(DMA_RX_BASE_ADDR)];
    uint32_t desc_addr;
    uint32_t des[4];
    hwaddr phys;

    if (!(s->dma_regs[DMA_IDX(DMA_CONTROL)] & DMA_CONTROL_SR)) {
        return -1;
    }

    if (!base) {
        return -1;
    }

    desc_addr = s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)];
    if (!desc_addr) {
        desc_addr = base;
    }
    phys = desc_addr & 0x1FFFFFFF;

    cpu_physical_memory_read(phys, des, DESC_SIZE);

    if (!(des[0] & RDES0_OWN)) {
        return 0;
    }

    uint32_t buf_size = des[1] & RDES1_SIZE1_MASK;
    hwaddr buf_phys = des[2] & 0x1FFFFFFF;
    uint32_t write_len = len < buf_size ? len : buf_size;

    if (buf_phys) {
        cpu_physical_memory_write(buf_phys, buf, write_len);
    }

    des[0] = RDES0_FS | RDES0_LS | ((write_len + 4) << RDES0_FL_SHIFT);
    cpu_physical_memory_write(phys, des, 4);

    s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_RI | DMA_STATUS_NIS;

    if (des[0] & RDES0_RER_ENH) {
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = base;
    } else {
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = desc_addr + DESC_SIZE;
    }

    return len;
}

static uint64_t ingenic_t31_gmac_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IngenicT31GmacState *s = INGENIC_T31_GMAC(opaque);

    if (offset < DMA_BASE_OFFSET) {
        return s->mac_regs[MAC_IDX(offset)];
    }

    uint32_t dma_off = offset - DMA_BASE_OFFSET;
    if (dma_off < 0x1000) {
        return s->dma_regs[DMA_IDX(dma_off)];
    }

    return 0;
}

static void ingenic_t31_gmac_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IngenicT31GmacState *s = INGENIC_T31_GMAC(opaque);

    if (offset < DMA_BASE_OFFSET) {
        uint32_t idx = MAC_IDX(offset);
        s->mac_regs[idx] = (uint32_t)value;

        switch (offset) {
        case MAC_MII_ADDR:
            if (value & MII_ADDR_BUSY) {
                if (value & MII_ADDR_WRITE) {
                    ingenic_t31_gmac_mdio_write(s);
                } else {
                    ingenic_t31_gmac_mdio_read(s);
                }
            }
            break;
        }
        return;
    }

    uint32_t dma_off = offset - DMA_BASE_OFFSET;
    if (dma_off >= 0x1000) {
        return;
    }

    uint32_t idx = DMA_IDX(dma_off);

    switch (dma_off) {
    case DMA_BUS_MODE:
        s->dma_regs[idx] = (uint32_t)value & ~DMA_BUS_MODE_SWR;
        break;
    case DMA_TX_POLL:
        if (s->dma_regs[DMA_IDX(DMA_CONTROL)] & DMA_CONTROL_ST) {
            ingenic_t31_gmac_do_tx(s);
        }
        break;
    case DMA_STATUS:
        s->dma_regs[idx] &= ~(uint32_t)value;
        break;
    default:
        s->dma_regs[idx] = (uint32_t)value;
        break;
    }
}

static const MemoryRegionOps ingenic_t31_gmac_ops = {
    .read = ingenic_t31_gmac_read,
    .write = ingenic_t31_gmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_gmac_phy_reset(IngenicT31GmacState *s)
{
    memset(s->phy_regs, 0, sizeof(s->phy_regs));

    s->phy_regs[MII_BMCR] = MII_BMCR_AUTOEN;
    s->phy_regs[MII_BMSR] = MII_BMSR_100TX_FD | MII_BMSR_100TX_HD |
                             MII_BMSR_10T_FD | MII_BMSR_10T_HD |
                             MII_BMSR_AN_COMP | MII_BMSR_AUTONEG |
                             MII_BMSR_LINK_ST | MII_BMSR_MFPS;
    s->phy_regs[MII_PHYID1] = 0x0000;
    s->phy_regs[MII_PHYID2] = 0x0128;
    s->phy_regs[MII_ANAR] = MII_ANAR_TXFD | MII_ANAR_TX |
                             MII_ANAR_10FD | MII_ANAR_10 |
                             MII_ANAR_CSMACD;
    s->phy_regs[MII_ANLPAR] = MII_ANLPAR_ACK | MII_ANLPAR_TXFD |
                               MII_ANLPAR_TX | MII_ANLPAR_10FD |
                               MII_ANLPAR_10 | MII_ANLPAR_CSMACD;
}

static void ingenic_t31_gmac_reset_hold(Object *obj, ResetType type)
{
    IngenicT31GmacState *s = INGENIC_T31_GMAC(obj);

    memset(s->mac_regs, 0, sizeof(s->mac_regs));
    memset(s->dma_regs, 0, sizeof(s->dma_regs));
    ingenic_t31_gmac_phy_reset(s);
}

static void ingenic_t31_gmac_realize(DeviceState *dev, Error **errp)
{
    IngenicT31GmacState *s = INGENIC_T31_GMAC(dev);

    static NetClientInfo net_info = {
        .type = NET_CLIENT_DRIVER_NIC,
        .size = sizeof(NICState),
        .can_receive = ingenic_t31_gmac_can_receive,
        .receive = ingenic_t31_gmac_receive,
    };

    s->nic = qemu_new_nic(&net_info, &s->conf, TYPE_INGENIC_T31_GMAC,
                           dev->id, &dev->mem_reentrancy_guard, s);
}

static void ingenic_t31_gmac_init(Object *obj)
{
    IngenicT31GmacState *s = INGENIC_T31_GMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_gmac_ops, s,
                          TYPE_INGENIC_T31_GMAC, INGENIC_T31_GMAC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const Property ingenic_t31_gmac_props[] = {
    DEFINE_NIC_PROPERTIES(IngenicT31GmacState, conf),
};

static void ingenic_t31_gmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_t31_gmac_realize;
    device_class_set_props(dc, ingenic_t31_gmac_props);
    rc->phases.hold = ingenic_t31_gmac_reset_hold;
}

static const TypeInfo ingenic_t31_gmac_type_info = {
    .name = TYPE_INGENIC_T31_GMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31GmacState),
    .instance_init = ingenic_t31_gmac_init,
    .class_init = ingenic_t31_gmac_class_init,
};

static void ingenic_t31_gmac_register_types(void)
{
    type_register_static(&ingenic_t31_gmac_type_info);
}

type_init(ingenic_t31_gmac_register_types)
