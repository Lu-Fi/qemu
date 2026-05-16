/*
 * Ingenic T31 GMAC (Synopsys DesignWare MAC) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/net/ingenic-t31-gmac.h"
#include "hw/net/mii.h"
#include "net/eth.h"
#include "system/dma.h"
#include "exec/cpu-common.h"
#include "qemu/main-loop.h"

#define MAC_MII_ADDR        0x0010
#define MAC_MII_DATA        0x0014

#define MII_ADDR_BUSY       (1 << 0)
#define MII_ADDR_WRITE      (1 << 1)
#define MII_ADDR_REG_SHIFT  6
#define MII_ADDR_REG_MASK   (0x1F << 6)
#define MII_ADDR_PHY_SHIFT  11
#define MII_ADDR_PHY_MASK   (0x1F << 11)

#define DMA_BUS_MODE        0x0000
#define DMA_TX_POLL         0x0004
#define DMA_RX_POLL         0x0008
#define DMA_RX_BASE_ADDR    0x000C
#define DMA_TX_BASE_ADDR    0x0010
#define DMA_STATUS          0x0014
#define DMA_CONTROL         0x0018
#define DMA_INTERRUPT       0x001C
#define DMA_CUR_TX_DESC     0x0048
#define DMA_CUR_RX_DESC     0x004C

#define DMA_BUS_MODE_SWR    (1 << 0)
#define DMA_CONTROL_ST      (1 << 13)
#define DMA_CONTROL_SR      (1 << 1)
#define DMA_STATUS_TI       (1 << 0)
#define DMA_STATUS_RI       (1 << 6)
#define DMA_STATUS_NIS      (1 << 16)
#define DMA_STATUS_AIS      (1 << 15)

#define TDES0_OWN           (1u << 31)
/*
 * ENH_DESC vs ENH_DESC_8W use different TDES0 layouts. The Ingenic Linux
 * driver and U-Boot driver in this tree compile with both ENH_DESC and
 * ENH_DESC_8W defined, but the active enum is the ENH_DESC one (see
 * SynopGMAC_Dev.h - the ENH_DESC_8W enum is in the #else branch).
 *
 * For ENH_DESC mode (first segment / last segment / TER all in TDES0):
 *   FS = bit 28, LS = bit 29, IC = bit 30, OWN = bit 31, TER = bit 21.
 * For ENH_DESC_8W mode they would be at bits 29/30/31/25 respectively,
 * but that mode is not what the driver compiles.
 *
 * Using the wrong TER bit causes the DMA cursor to walk past the end of
 * the ring into adjacent memory (e.g. gmacdev->MacBase) and corrupt it.
 */
#define TDES0_LS            (1 << 29)
#define TDES0_FS            (1 << 28)
#define TDES0_TER           (1 << 21)
/*
 * TCH (TDES0 bit 20) selects chained descriptors: TDES3 is the next
 * descriptor address instead of buffer2, and ring stride/TER do not
 * apply. The vendor SynopGMAC driver uses ring mode; mainline U-Boot
 * designware.c uses chain mode. Real DesignWare hardware supports
 * both, so model both here.
 */
#define TDES0_TCH           (1 << 20)
#define TDES1_SIZE1_MASK    0x1FFF
#define RDES0_OWN           (1u << 31)
#define RDES0_FS            (1 << 9)
#define RDES0_LS            (1 << 8)
#define RDES0_FL_SHIFT      16
/* RX TER lives in RDES1 (length field) at bit 15 in ENH_DESC mode. */
#define RDES1_RER           (1 << 15)
#define RDES1_RCH           (1 << 14)	/* RX chain: RDES3 = next desc */
#define RDES1_SIZE1_MASK    0x1FFF

#define DMA_BASE_OFFSET     0x1000
#define MAC_IDX(off)        ((off) / 4)
#define DMA_IDX(off)        ((off) / 4)
#define NUM_TX_DESCS        16

/* DMA_BUS_MODE bits */
#define DMA_BUS_MODE_ATDS   (1 << 7)        /* alternate (8-word) descriptor */
#define DMA_BUS_MODE_DSL_S  2               /* DSL field shift */
#define DMA_BUS_MODE_DSL_M  0x1F            /* DSL field mask (5 bits) */

static void gmac_ram_read(IngenicT31GmacState *s, hwaddr phys,
                          void *buf, int len)
{
    cpu_physical_memory_read(phys, buf, len);
}

static void gmac_ram_write(IngenicT31GmacState *s, hwaddr phys,
                           const void *buf, int len)
{
    cpu_physical_memory_write(phys, buf, len);
}

static void gmac_update_irq(IngenicT31GmacState *s)
{
    uint32_t pending = s->dma_regs[DMA_IDX(DMA_STATUS)] &
                       s->dma_regs[DMA_IDX(DMA_INTERRUPT)];
    qemu_set_irq(s->irq, pending != 0);
}

static uint32_t gmac_desc_stride(IngenicT31GmacState *s)
{
    uint32_t bm = s->dma_regs[DMA_IDX(DMA_BUS_MODE)];
    uint32_t hw_size = (bm & DMA_BUS_MODE_ATDS) ? 32 : 16;
    uint32_t dsl = (bm >> DMA_BUS_MODE_DSL_S) & DMA_BUS_MODE_DSL_M;
    return hw_size + dsl * 4;
}

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
        uint16_t val = s->mac_regs[MAC_IDX(MAC_MII_DATA)] & 0xFFFF;
        /*
         * BMCR_RESET (bit 15) and BMCR_ANRESTART (bit 9) are self-clearing
         * on real PHYs - they tell the PHY to perform an action and clear
         * once the action is done. Linux's jz_mac_open() polls BMCR_RESET
         * waiting for the reset to complete; if we leave the bit set,
         * eth0 up hangs forever in that loop. Clear them immediately to
         * simulate instant completion.
         */
        if (reg == MII_BMCR) {
            val &= ~(MII_BMCR_RESET | MII_BMCR_ANRESTART);
        }
        s->phy_regs[reg] = val;
    }
    s->mac_regs[MAC_IDX(MAC_MII_ADDR)] &= ~MII_ADDR_BUSY;
}

static void ingenic_t31_gmac_do_tx(IngenicT31GmacState *s)
{
    uint32_t base = s->dma_regs[DMA_IDX(DMA_TX_BASE_ADDR)];
    uint8_t buf[2048];
    uint32_t stride;
    int count;

    if (!base) {
        return;
    }

    stride = gmac_desc_stride(s);

    for (count = 0; count < NUM_TX_DESCS; count++) {
        uint32_t desc_addr = s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)];
        if (!desc_addr) {
            desc_addr = base;
        }
        uint32_t des[4];
        hwaddr phys = desc_addr & 0x1FFFFFFF;
        bool end_of_ring;

        gmac_ram_read(s, phys, des, 16);

        if (!(des[0] & TDES0_OWN)) {
            break;
        }

        end_of_ring = des[0] & TDES0_TER;
        uint32_t len = des[1] & TDES1_SIZE1_MASK;
        hwaddr buf_phys = des[2] & 0x1FFFFFFF;

        if (len > sizeof(buf)) {
            len = sizeof(buf);
        }
        if (len > 0 && buf_phys) {
            gmac_ram_read(s, buf_phys, buf, len);
            if ((des[0] & TDES0_FS) && (des[0] & TDES0_LS) && s->nic) {
                qemu_send_packet(qemu_get_queue(s->nic), buf, len);
            }
        }

        des[0] &= ~TDES0_OWN;
        gmac_ram_write(s, phys, &des[0], 4);

        s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_TI | DMA_STATUS_NIS;
        gmac_update_irq(s);

        if (des[0] & TDES0_TCH) {
            /* Chained: TDES3 is the next descriptor address. */
            s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = des[3];
        } else if (end_of_ring) {
            s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = base;
        } else {
            s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = desc_addr + stride;
        }
    }
}

static bool ingenic_t31_gmac_can_receive(NetClientState *nc)
{
    IngenicT31GmacState *s = qemu_get_nic_opaque(nc);
    return (s->dma_regs[DMA_IDX(DMA_CONTROL)] & DMA_CONTROL_SR) != 0;
}

static ssize_t ingenic_t31_gmac_receive(NetClientState *nc,
                                        const uint8_t *buf, size_t len)
{
    IngenicT31GmacState *s = qemu_get_nic_opaque(nc);
    uint32_t base = s->dma_regs[DMA_IDX(DMA_RX_BASE_ADDR)];
    uint32_t desc_addr, des[4];
    hwaddr phys;

    if (!base || !(s->dma_regs[DMA_IDX(DMA_CONTROL)] & DMA_CONTROL_SR)) {
        return -1;
    }

    desc_addr = s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)];
    if (!desc_addr) {
        desc_addr = base;
    }
    phys = desc_addr & 0x1FFFFFFF;

    gmac_ram_read(s, phys, des, 16);

    if (!(des[0] & RDES0_OWN)) {
        return 0;
    }

    uint32_t buf_size = des[1] & RDES1_SIZE1_MASK;
    hwaddr buf_phys = des[2] & 0x1FFFFFFF;
    uint32_t write_len = len < buf_size ? len : buf_size;

    if (buf_phys) {
        gmac_ram_write(s, buf_phys, buf, write_len);
    }

    des[0] = RDES0_FS | RDES0_LS | ((write_len + 4) << RDES0_FL_SHIFT);
    gmac_ram_write(s, phys, &des[0], 4);

    s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_RI | DMA_STATUS_NIS;
    gmac_update_irq(s);

    if (des[1] & RDES1_RCH) {
        /* Chained: RDES3 is the next descriptor address. */
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = des[3];
    } else if (des[1] & RDES1_RER) {
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = base;
    } else {
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = desc_addr + gmac_desc_stride(s);
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
        if (offset == MAC_MII_ADDR && (value & MII_ADDR_BUSY)) {
            if (value & MII_ADDR_WRITE) {
                ingenic_t31_gmac_mdio_write(s);
            } else {
                ingenic_t31_gmac_mdio_read(s);
            }
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
        if (value & DMA_BUS_MODE_SWR) {
            /* Software reset: clear DMA state */
            s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = 0;
            s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = 0;
            s->dma_regs[DMA_IDX(DMA_STATUS)] = 0;
            s->dma_regs[DMA_IDX(DMA_CONTROL)] = 0;
        }
        s->dma_regs[idx] = (uint32_t)value & ~DMA_BUS_MODE_SWR;
        break;
    case DMA_TX_BASE_ADDR:
        /* Setting TX base address resets the DMA's TX desc pointer to base */
        s->dma_regs[idx] = (uint32_t)value;
        s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = (uint32_t)value;
        break;
    case DMA_RX_BASE_ADDR:
        s->dma_regs[idx] = (uint32_t)value;
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = (uint32_t)value;
        break;
    case DMA_TX_POLL:
        ingenic_t31_gmac_do_tx(s);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case DMA_RX_POLL:
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case DMA_STATUS:
        /* W1C: writing 1 to a status bit clears it. */
        s->dma_regs[idx] &= ~(uint32_t)value;
        gmac_update_irq(s);
        break;
    case DMA_INTERRUPT:
        s->dma_regs[idx] = (uint32_t)value;
        gmac_update_irq(s);
        break;
    case DMA_CONTROL:
        s->dma_regs[idx] = (uint32_t)value;
        if (value & DMA_CONTROL_ST) {
            ingenic_t31_gmac_do_tx(s);
            if (s->nic) {
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
            }
        } else if ((value & DMA_CONTROL_SR) && s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
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
                             MII_ANAR_10FD | MII_ANAR_10 | MII_ANAR_CSMACD;
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
    sysbus_init_irq(sbd, &s->irq);
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
