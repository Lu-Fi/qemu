/*
 * Ingenic T31 GMAC (Synopsys DesignWare MAC) emulation
 *
 * The frame path follows a DWC Ether MAC 3.70a with enhanced (alternate)
 * descriptors as the T21 and T40 bench boards run it: perfect, hash and
 * inverse address filtering, pad and FCS on transmit, ACS on receive,
 * frames spanning descriptors, TX and RX checksum offload, and the MTL's
 * cut-through rules for errored and undersized frames.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <zlib.h>
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/net/ingenic-gmac.h"
#include "hw/net/mii.h"
#include "net/eth.h"
#include "net/checksum.h"
#include "system/dma.h"
#include "system/physmem.h"
#include "qemu/main-loop.h"

#define MAC_CONFIG          0x0000
#define MAC_CONFIG_RE       (1 << 2)
#define MAC_CONFIG_TE       (1 << 3)
#define MAC_CONFIG_ACS      (1 << 7)
#define MAC_CONFIG_IPC      (1 << 10)
#define MAC_CONFIG_LM       (1 << 12)
#define MAC_CONFIG_FES      (1 << 14)   /* 100M, not 10M, in MII mode */
#define MAC_CONFIG_JE       (1 << 20)
#define MAC_CONFIG_JD       (1 << 22)
#define MAC_CONFIG_WD       (1 << 23)
#define MAC_FRAME_FILTER    0x0004
#define MAC_HASH_HI         0x0008
#define MAC_HASH_LO         0x000C
#define MAC_MII_ADDR        0x0010
#define MAC_MII_DATA        0x0014
#define MAC_VERSION         0x0020
#define MAC_ADDR_HI(n)     (0x0040 + 8 * (n))
#define MAC_ADDR_LO(n)     (0x0044 + 8 * (n))
#define MAC_ADDR_COUNT      16

#define FF_PR               (1 << 0)
#define FF_HUC              (1 << 1)
#define FF_HMC              (1 << 2)
#define FF_DAIF             (1 << 3)
#define FF_PM               (1 << 4)
#define FF_DBF              (1 << 5)
#define FF_PCF_SHIFT        6
#define FF_SAIF             (1 << 8)
#define FF_SAF              (1 << 9)
#define FF_HPF              (1 << 10)
#define FF_RA               (1u << 31)

#define ADDR_HI_AE          (1u << 31)
#define ADDR_HI_SA          (1u << 30)
#define ADDR_HI_MBC_SHIFT   24

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
#define DMA_MISSED          0x0020
#define DMA_RIWT            0x0024
#define DMA_CUR_TX_DESC     0x0048
#define DMA_CUR_RX_DESC     0x004C
#define DMA_CUR_TX_BUF      0x0050
#define DMA_CUR_RX_BUF      0x0054
#define DMA_HW_FEATURE      0x0058

#define DMA_BUS_MODE_SWR    (1 << 0)
#define DMA_BUS_MODE_RESET  0x00020100
#define DMA_BUS_MODE_ATDS   (1 << 7)        /* alternate (8-word) descriptor */
#define DMA_BUS_MODE_DSL_S  2               /* DSL field shift */
#define DMA_BUS_MODE_DSL_M  0x1F            /* DSL field mask (5 bits) */

#define DMA_CONTROL_SR      (1 << 1)
#define DMA_CONTROL_RTC_S   3
#define DMA_CONTROL_FUF     (1 << 6)
#define DMA_CONTROL_FEF     (1 << 7)
#define DMA_CONTROL_ST      (1 << 13)
#define DMA_CONTROL_RSF     (1 << 25)
#define DMA_CONTROL_DT      (1 << 26)

#define DMA_STATUS_TI       (1 << 0)
#define DMA_STATUS_TPS      (1 << 1)
#define DMA_STATUS_TU       (1 << 2)
#define DMA_STATUS_TJT      (1 << 3)
#define DMA_STATUS_RI       (1 << 6)
#define DMA_STATUS_RU       (1 << 7)
#define DMA_STATUS_RPS      (1 << 8)
#define DMA_STATUS_RWT      (1 << 9)
#define DMA_STATUS_ETI      (1 << 10)
#define DMA_STATUS_ERI      (1 << 14)
#define DMA_STATUS_AIS      (1 << 15)
#define DMA_STATUS_NIS      (1 << 16)
#define DMA_STATUS_RS_S     17
#define DMA_STATUS_TS_S     20
/* the interrupt sources the summary bits collect, and the W1C ones */
#define DMA_STATUS_NORMAL   (DMA_STATUS_TI | DMA_STATUS_TU | DMA_STATUS_RI | \
                             DMA_STATUS_ERI)
#define DMA_STATUS_ABNORMAL 0x000027BA
#define DMA_STATUS_W1C      (DMA_STATUS_NORMAL | DMA_STATUS_ABNORMAL | \
                             DMA_STATUS_AIS | DMA_STATUS_NIS)
#define DMA_INT_NIE         (1 << 16)
#define DMA_INT_AIE         (1 << 15)

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
#define TDES0_IC            (1 << 30)
#define TDES0_LS            (1 << 29)
#define TDES0_FS            (1 << 28)
#define TDES0_DC            (1 << 27)
#define TDES0_DP            (1 << 26)
#define TDES0_CIC_SHIFT     22
#define TDES0_TER           (1 << 21)
/*
 * TCH (TDES0 bit 20) selects chained descriptors: TDES3 is the next
 * descriptor address instead of buffer2, and ring stride/TER do not
 * apply. The vendor SynopGMAC driver uses ring mode; mainline U-Boot
 * designware.c uses chain mode. Real DesignWare hardware supports
 * both, so model both here.
 */
#define TDES0_TCH           (1 << 20)
#define TDES0_ES            (1 << 15)
#define TDES0_JT            (1 << 14)
#define TDES0_PCE           (1 << 12)
#define TDES0_VF            (1 << 7)
#define TDES0_IHE           (1 << 16)
#define TDES1_SIZE_MASK     0x1FFF
#define TDES1_SIZE2_SHIFT   16

#define RDES0_OWN           (1u << 31)
#define RDES0_AFM           (1 << 30)
#define RDES0_FL_SHIFT      16
#define RDES0_ES            (1 << 15)
#define RDES0_DE            (1 << 14)
#define RDES0_SAF           (1 << 13)
#define RDES0_LE            (1 << 12)
#define RDES0_VLAN          (1 << 10)
#define RDES0_FS            (1 << 9)
#define RDES0_LS            (1 << 8)
#define RDES0_GF            (1 << 7)
#define RDES0_FT            (1 << 5)
#define RDES0_RWT           (1 << 4)
#define RDES0_CE            (1 << 1)
#define RDES0_ESA           (1 << 0)
#define RDES1_DIC           (1u << 31)
/* RX TER lives in RDES1 (length field) at bit 15 in ENH_DESC mode. */
#define RDES1_RER           (1 << 15)
#define RDES1_RCH           (1 << 14)  /* RX chain: RDES3 = next desc */
#define RDES1_SIZE1_MASK    0x1FFF
#define RDES1_SIZE2_SHIFT   16
#define RDES4_IPHDR_ERR     (1 << 3)
#define RDES4_IPPAY_ERR     (1 << 4)
#define RDES4_IPV4          (1 << 6)

#define DMA_BASE_OFFSET     0x1000
#define MAC_IDX(off)        ((off) / 4)
#define DMA_IDX(off)        ((off) / 4)
#define NUM_TX_DESCS        1024
#define IP_PROTO_ICMP      1
#define ETH_P_PAUSE        0x8808

/* what the MAC puts on the wire, FCS included */
#define GMAC_FRAME_MAX      16400
#define ETH_MIN_NOFCS       60
#define ETH_MIN_FRAME       64
#define ETH_FCS_LEN         4

static void gmac_ram_read(IngenicGmacState *s, hwaddr phys,
                          void *buf, int len)
{
    physical_memory_read(phys, buf, len);
}

static void gmac_ram_write(IngenicGmacState *s, hwaddr phys,
                           const void *buf, int len)
{
    physical_memory_write(phys, buf, len);
}

static uint32_t gmac_status(IngenicGmacState *s)
{
    uint32_t ie = s->dma_regs[DMA_IDX(DMA_INTERRUPT)];
    uint32_t ctl = s->dma_regs[DMA_IDX(DMA_CONTROL)];
    uint32_t ev = s->dma_regs[DMA_IDX(DMA_STATUS)];
    uint32_t st = ev & ~(DMA_STATUS_NIS | DMA_STATUS_AIS);
    uint32_t rs = 0;

    if (ctl & DMA_CONTROL_SR) {
        rs = s->rx_suspended ? 4 : 3;
    }
    if (ev & ie & DMA_STATUS_NORMAL) {
        st |= DMA_STATUS_NIS;
    }
    if (ev & ie & DMA_STATUS_ABNORMAL) {
        st |= DMA_STATUS_AIS;
    }
    st |= rs << DMA_STATUS_RS_S;
    st |= ((ctl & DMA_CONTROL_ST) && !s->tx_stopped ? 6u : 0u) <<
          DMA_STATUS_TS_S;
    return st;
}

static void gmac_update_irq(IngenicGmacState *s)
{
    uint32_t ie = s->dma_regs[DMA_IDX(DMA_INTERRUPT)];
    uint32_t st = gmac_status(s);

    qemu_set_irq(s->irq, ((st & DMA_STATUS_NIS) && (ie & DMA_INT_NIE)) ||
                         ((st & DMA_STATUS_AIS) && (ie & DMA_INT_AIE)));
}

static void gmac_raise(IngenicGmacState *s, uint32_t bits)
{
    s->dma_regs[DMA_IDX(DMA_STATUS)] |= bits;
    gmac_update_irq(s);
}

static uint32_t gmac_desc_stride(IngenicGmacState *s)
{
    uint32_t bm = s->dma_regs[DMA_IDX(DMA_BUS_MODE)];
    uint32_t hw_size = (bm & DMA_BUS_MODE_ATDS) ? 32 : 16;
    uint32_t dsl = (bm >> DMA_BUS_MODE_DSL_S) & DMA_BUS_MODE_DSL_M;
    return hw_size + dsl * 4;
}

static void ingenic_gmac_mdio_read(IngenicGmacState *s)
{
    uint32_t addr = s->mac_regs[MAC_IDX(MAC_MII_ADDR)];
    uint32_t phy = (addr & MII_ADDR_PHY_MASK) >> MII_ADDR_PHY_SHIFT;
    uint32_t reg = (addr & MII_ADDR_REG_MASK) >> MII_ADDR_REG_SHIFT;

    if (s->has_phy && phy == 0 && reg < INGENIC_GMAC_PHY_REGS) {
        s->mac_regs[MAC_IDX(MAC_MII_DATA)] = s->phy_regs[reg];
    } else if (s->mdio_hold) {
        /* The last bit the MAC drove before turnaround, REGAD's lowest */
        s->mac_regs[MAC_IDX(MAC_MII_DATA)] = reg & 1 ? 0xFFFF : 0;
    } else {
        s->mac_regs[MAC_IDX(MAC_MII_DATA)] = 0xFFFF;
    }
    s->mac_regs[MAC_IDX(MAC_MII_ADDR)] &= ~MII_ADDR_BUSY;
}

static void ingenic_gmac_mdio_write(IngenicGmacState *s)
{
    uint32_t addr = s->mac_regs[MAC_IDX(MAC_MII_ADDR)];
    uint32_t phy = (addr & MII_ADDR_PHY_MASK) >> MII_ADDR_PHY_SHIFT;
    uint32_t reg = (addr & MII_ADDR_REG_MASK) >> MII_ADDR_REG_SHIFT;

    if (s->has_phy && phy == 0 && reg < INGENIC_GMAC_PHY_REGS) {
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

/*
 * Transmit checksum insertion (TDES0 CIC), as the silicon does it on frames
 * it can parse: IPv4 over Ethernet, optionally one VLAN tag, a TCP, UDP or
 * ICMP payload. CIC 1 fixes the IPv4 header checksum; CIC 2 also sums the
 * payload, taking the checksum field the driver left there as the seed
 * (the pseudo-header sum); CIC 3 zeroes it and adds the pseudo header
 * itself. ICMP has no pseudo header, so 3 sums it as 2 does. A frame that is
 * not IPv4 passes untouched, except that an IP header the MAC cannot parse
 * flags IHE, and a payload cut short of its IP length flags PCE.
 */
static uint32_t gmac_tx_csum(uint8_t *f, size_t len, int cic)
{
    size_t off = 14, l4len, avail, csum_off;
    uint16_t et;
    uint8_t *ip, *l4;
    uint32_t sum;
    unsigned hl;
    uint32_t err = 0;

    if (len < 14) {
        return 0;
    }
    et = lduw_be_p(f + 12);
    if (et == ETH_P_VLAN) {
        if (len < 18) {
            return 0;
        }
        et = lduw_be_p(f + 16);
        off = 18;
    }
    if (et != ETH_P_IP && et != ETH_P_IPV6) {
        return 0;
    }
    ip = f + off;
    hl = (ip[0] & 15) * 4;
    if (len < off + 20 || hl < 20 || len < off + hl) {
        return TDES0_IHE | TDES0_ES;
    }
    if ((ip[0] >> 4) != 4) {
        err = TDES0_IHE | TDES0_ES;
    }
    stw_be_p(ip + 10, 0);
    stw_be_p(ip + 10, net_raw_checksum(ip, hl));
    if (cic == 1) {
        return err;
    }

    switch (ip[9]) {
    case IP_PROTO_TCP:
        csum_off = 16;
        break;
    case IP_PROTO_UDP:
        csum_off = 6;
        break;
    case IP_PROTO_ICMP:
        csum_off = 2;
        break;
    default:
        return err;
    }
    avail = len - off - hl;
    l4len = lduw_be_p(ip + 2) > hl ? lduw_be_p(ip + 2) - hl : 0;
    if (l4len > avail) {
        l4len = avail;
        err |= TDES0_PCE | TDES0_ES;
    }
    if (l4len < csum_off + 2) {
        return err;
    }
    l4 = ip + hl;
    if (cic == 3 && ip[9] != IP_PROTO_ICMP) {
        stw_be_p(l4 + csum_off, 0);
        sum = net_checksum_add(8, ip + 12) + ip[9] + l4len +
              net_checksum_add(l4len, l4);
    } else {
        sum = net_checksum_add(l4len, l4);
    }
    sum = net_checksum_finish(sum);
    if (!sum && ip[9] == IP_PROTO_UDP) {
        sum = 0xffff;
    }
    stw_be_p(l4 + csum_off, sum);
    return err;
}

/*
 * Receive-side IP checksum check (MAC_CONFIG.IPC), reported in RDES4 and
 * RDES0 bit 0 for an unfragmented IPv4 TCP, UDP or ICMP frame: the payload
 * type, IPv4, and header and payload error bits. Returns false when the
 * frame is not one the MAC parses.
 */
static bool gmac_rx_ipc(const uint8_t *f, size_t len, uint32_t *ext, bool *err)
{
    size_t off = 14, l4len, avail;
    const uint8_t *ip;
    unsigned hl;
    uint32_t sum, type;
    uint16_t et;

    if (len < 18) {
        return false;
    }
    et = lduw_be_p(f + 12);
    if (et == ETH_P_VLAN) {
        et = lduw_be_p(f + 16);
        off = 18;
    }
    if (et != ETH_P_IP || len < off + 20) {
        return false;
    }
    ip = f + off;
    if ((ip[0] >> 4) != 4) {
        return false;
    }
    hl = (ip[0] & 15) * 4;
    if (hl < 20 || len < off + hl || (lduw_be_p(ip + 6) & 0x3fff)) {
        return false;
    }
    *ext = RDES4_IPV4;
    if (net_raw_checksum((uint8_t *)ip, hl)) {
        *ext |= RDES4_IPHDR_ERR;
        *err = true;
    }
    switch (ip[9]) {
    case IP_PROTO_UDP:
        type = 1;
        break;
    case IP_PROTO_TCP:
        type = 2;
        break;
    case IP_PROTO_ICMP:
        type = 3;
        break;
    default:
        return true;
    }
    *ext |= type;
    avail = len - off - hl;
    l4len = lduw_be_p(ip + 2) > hl ? lduw_be_p(ip + 2) - hl : 0;
    if (l4len > avail) {
        l4len = avail;
    }
    sum = net_checksum_add(l4len, (uint8_t *)ip + hl);
    if (type != 3) {
        sum += net_checksum_add(8, (uint8_t *)ip + 12) + ip[9] + l4len;
    }
    if (net_checksum_finish(sum) &&
        !(type == 1 && lduw_be_p(ip + hl + 6) == 0)) {
        *ext |= RDES4_IPPAY_ERR;
        *err = true;
    }
    return true;
}

static void gmac_desc_read(IngenicGmacState *s, uint32_t addr,
                              uint32_t *des)
{
    uint8_t raw[16];
    int i;

    gmac_ram_read(s, addr & 0x1FFFFFFF, raw, sizeof(raw));
    for (i = 0; i < 4; i++) {
        des[i] = ldl_le_p(raw + 4 * i);
    }
}

static void gmac_desc_write_word(IngenicGmacState *s, uint32_t addr,
                                 int word, uint32_t val)
{
    uint8_t raw[4];

    stl_le_p(raw, val);
    gmac_ram_write(s, (addr & 0x1FFFFFFF) + 4 * word, raw, 4);
}

static bool gmac_rx_frame(IngenicGmacState *s, const uint8_t *buf,
                          size_t len, bool crc_ok);

/* the next descriptor after the one at desc_addr, by mode */
static uint32_t gmac_next_desc(IngenicGmacState *s, uint32_t base,
                               uint32_t desc_addr, bool chain, bool end,
                               uint32_t des3)
{
    if (chain) {
        return des3;
    }
    if (end) {
        return base;
    }
    return desc_addr + gmac_desc_stride(s);
}

/*
 * One frame leaves the MAC. Short frames are padded to 60 bytes unless DP;
 * the FCS is appended unless DC says the driver supplied it, which the MAC
 * honours only above the 60 bytes it would pad to.
 */
static void gmac_tx_frame(IngenicGmacState *s, uint32_t flags, uint32_t *tdes0)
{
    uint32_t cfg = s->mac_regs[MAC_IDX(MAC_CONFIG)];
    int cic = (flags >> TDES0_CIC_SHIFT) & 3;
    size_t len = s->tx_len, limit;
    uint8_t *f = s->tx_buf;
    uint32_t status = 0;

    if (!(cfg & MAC_CONFIG_JD)) {
        limit = (cfg & MAC_CONFIG_JE) ? 10240 : 2048;
        if (len + (flags & TDES0_DC ? 0 : ETH_FCS_LEN) > limit) {
            len = limit;
            status |= TDES0_JT | TDES0_ES;
        }
    }
    if (len >= 14 && lduw_be_p(f + 12) == ETH_P_VLAN) {
        status |= TDES0_VF;
    }
    if (cic && !(status & TDES0_JT)) {
        status |= gmac_tx_csum(f, len, cic);
    }
    if (!(flags & TDES0_DP) && len < ETH_MIN_NOFCS) {
        memset(f + len, 0, ETH_MIN_NOFCS - len);
        len = ETH_MIN_NOFCS;
    }
    if (!(status & TDES0_JT) &&
        (!(flags & TDES0_DC) ||
         (!(flags & TDES0_DP) && len <= ETH_MIN_NOFCS))) {
        stl_le_p(f + len, crc32(0, f, len));
        len += ETH_FCS_LEN;
    }
    s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_ETI;

    if (s->mac_regs[MAC_IDX(MAC_CONFIG)] & MAC_CONFIG_LM) {
        bool crc_ok = len >= ETH_FCS_LEN &&
                      (uint32_t)ldl_le_p(f + len - ETH_FCS_LEN) ==
                      (uint32_t)crc32(0, f, len - ETH_FCS_LEN);

        gmac_rx_frame(s, f, len, crc_ok);
    } else if (s->nic && len > ETH_FCS_LEN) {
        qemu_send_packet(qemu_get_queue(s->nic), f, len - ETH_FCS_LEN);
    }
    *tdes0 = status;
}

static void ingenic_gmac_do_tx(IngenicGmacState *s)
{
    uint32_t base = s->dma_regs[DMA_IDX(DMA_TX_BASE_ADDR)];
    int count;

    if (!base || s->tx_stopped ||
        !(s->dma_regs[DMA_IDX(DMA_CONTROL)] & DMA_CONTROL_ST)) {
        return;
    }

    for (count = 0; count < NUM_TX_DESCS; count++) {
        uint32_t desc_addr = s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)];
        uint32_t des[4];
        uint32_t size1, size2, status = 0;
        bool chain, end;

        if (!desc_addr) {
            desc_addr = base;
        }
        gmac_desc_read(s, desc_addr, des);

        if (!(des[0] & TDES0_OWN)) {
            /* idle, the current buffer address reads 0 (a T21N, a T40XP) */
            s->dma_regs[DMA_IDX(DMA_CUR_TX_BUF)] = 0;
            gmac_raise(s, DMA_STATUS_TU);
            break;
        }

        chain = des[0] & TDES0_TCH;
        end = des[0] & TDES0_TER;
        size1 = des[1] & TDES1_SIZE_MASK;
        size2 = chain ? 0 : (des[1] >> TDES1_SIZE2_SHIFT) & TDES1_SIZE_MASK;

        if (des[0] & TDES0_FS) {
            s->tx_len = 0;
            s->tx_flags = des[0];
        }
        if (size1 && size1 + s->tx_len <= sizeof(s->tx_buf) - 8) {
            gmac_ram_read(s, des[2] & 0x1FFFFFFF, s->tx_buf + s->tx_len,
                          size1);
            s->tx_len += size1;
        }
        if (size2 && size2 + s->tx_len <= sizeof(s->tx_buf) - 8) {
            gmac_ram_read(s, des[3] & 0x1FFFFFFF, s->tx_buf + s->tx_len,
                          size2);
            s->tx_len += size2;
        }
        s->dma_regs[DMA_IDX(DMA_CUR_TX_BUF)] = des[2];

        if (des[0] & TDES0_LS) {
            gmac_tx_frame(s, s->tx_flags, &status);
            s->tx_len = 0;
            if (status & TDES0_JT) {
                s->tx_stopped = true;
            }
            if (des[0] & TDES0_IC) {
                gmac_raise(s, DMA_STATUS_TI);
            }
        }

        gmac_desc_write_word(s, desc_addr, 0,
                             (des[0] & ~(TDES0_OWN | 0x1ffff)) | status);

        s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] =
            gmac_next_desc(s, base, desc_addr, chain, end, des[3]);
        if (s->tx_stopped) {
            gmac_raise(s, DMA_STATUS_TPS | DMA_STATUS_TJT);
            break;
        }
    }
    gmac_update_irq(s);
}

static bool ingenic_gmac_can_receive(NetClientState *nc)
{
    IngenicGmacState *s = qemu_get_nic_opaque(nc);
    return (s->dma_regs[DMA_IDX(DMA_CONTROL)] & DMA_CONTROL_SR) != 0;
}

static void ingenic_gmac_set_link_status(NetClientState *nc)
{
    IngenicGmacState *s = qemu_get_nic_opaque(nc);

    /*
     * Mirror the backend link state into the PHY status register.
     * The guest drivers use phylib polling (no PHY interrupt line),
     * so flipping BMSR is all carrier propagation takes.
     */
    if (nc->link_down) {
        s->phy_regs[MII_BMSR] &= ~(MII_BMSR_LINK_ST | MII_BMSR_AN_COMP);
    } else {
        s->phy_regs[MII_BMSR] |= MII_BMSR_LINK_ST | MII_BMSR_AN_COMP;
    }
}

static bool gmac_addr_match(uint32_t hi, uint32_t lo, const uint8_t *a)
{
    uint8_t m[ETH_ALEN] = { lo, lo >> 8, lo >> 16, lo >> 24, hi, hi >> 8 };
    unsigned mbc = (hi >> ADDR_HI_MBC_SHIFT) & 0x3f;
    int i;

    for (i = 0; i < ETH_ALEN; i++) {
        if (!(mbc & (1 << i)) && m[i] != a[i]) {
            return false;
        }
    }
    return true;
}

static bool gmac_hash_hit(IngenicGmacState *s, const uint8_t *da)
{
    uint64_t table = ((uint64_t)s->mac_regs[MAC_IDX(MAC_HASH_HI)] << 32) |
                     s->mac_regs[MAC_IDX(MAC_HASH_LO)];

    return table >> (revbit32(~net_crc32_le(da, ETH_ALEN)) >> 26) & 1;
}

/*
 * The MAC's address filter. Returns whether the frame goes on to the DMA;
 * afm and saf are the descriptor's DA and SA filter-fail flags, which the
 * MAC raises for a frame it forwards although a filter failed (RA, or a
 * control frame), and for an SA mismatch even where SAF does not drop it.
 */
static bool gmac_rx_filter(IngenicGmacState *s, const uint8_t *f,
                           bool *afm, bool *saf)
{
    uint32_t ff = s->mac_regs[MAC_IDX(MAC_FRAME_FILTER)];
    const uint8_t *da = f, *sa = f + ETH_ALEN;
    bool bcast = is_broadcast_ether_addr(da);
    bool mcast = da[0] & 1;
    bool match = false, da_pass, sa_match = false, have_sa = false;
    bool sa_fail, control;
    int i;

    if (ff & FF_PR) {
        return true;
    }

    for (i = 0; i < MAC_ADDR_COUNT; i++) {
        uint32_t hi = s->mac_regs[MAC_IDX(MAC_ADDR_HI(i))];
        uint32_t lo = s->mac_regs[MAC_IDX(MAC_ADDR_LO(i))];

        if (i && !(hi & ADDR_HI_AE)) {
            continue;
        }
        if (i && (hi & ADDR_HI_SA)) {
            have_sa = true;
            sa_match |= gmac_addr_match(hi, lo, sa);
        } else if (!mcast || i) {
            match |= gmac_addr_match(hi, lo, da);
        }
    }

    if (bcast) {
        da_pass = !(ff & FF_DBF);
    } else if (mcast) {
        match |= (ff & FF_HMC) && gmac_hash_hit(s, da);
        da_pass = (ff & FF_PM) || ((ff & FF_DAIF) ? !match : match);
    } else {
        if (ff & FF_HUC) {
            bool hit = gmac_hash_hit(s, da);

            match = (ff & FF_HPF) ? (match || hit) : hit;
        }
        da_pass = (ff & FF_DAIF) ? !match : match;
    }

    sa_fail = (ff & FF_SAIF) ? sa_match :
              (have_sa || (ff & FF_SAF)) && !sa_match;
    *saf = sa_fail;
    *afm = !da_pass;

    control = lduw_be_p(f + 12) == ETH_P_PAUSE;
    if (control) {
        switch ((ff >> FF_PCF_SHIFT) & 3) {
        case 0:
            return false;
        case 3:
            return da_pass;
        default:
            return true;
        }
    }
    if (!da_pass && !(ff & FF_RA)) {
        return false;
    }
    return !(sa_fail && (ff & FF_SAF) && !(ff & FF_RA));
}

/*
 * Put a received frame into the ring. RDES0 carries FS on the first
 * descriptor and the byte count so far on each; the status flags and LS
 * arrive on the last. A frame that does not fit the descriptors the host
 * has left ends in a descriptor error, LS and ES.
 */
static void gmac_rx_store(IngenicGmacState *s, const uint8_t *data,
                          size_t len, uint32_t flags, uint32_t ext,
                          bool has_ext)
{
    uint32_t base = s->dma_regs[DMA_IDX(DMA_RX_BASE_ADDR)];
    uint32_t addr = s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] ?: base;
    size_t off = 0;
    int nd = 0;

    if (!base) {
        return;
    }
    for (;;) {
        uint32_t des[4], next, nd_des[4];
        uint32_t size1, size2, st = 0;
        size_t n;
        bool chain, end, done, trunc = false;

        gmac_desc_read(s, addr, des);
        if (!(des[0] & RDES0_OWN)) {
            s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = addr;
            s->rx_suspended = true;
            gmac_raise(s, DMA_STATUS_RU);
            return;
        }
        size1 = des[1] & RDES1_SIZE1_MASK & ~3;
        chain = des[1] & RDES1_RCH;
        end = des[1] & RDES1_RER;
        size2 = chain ? 0 :
                (des[1] >> RDES1_SIZE2_SHIFT) & RDES1_SIZE1_MASK & ~3;

        n = MIN(size1, len - off);
        if (n) {
            gmac_ram_write(s, des[2] & 0x1FFFFFFF, data + off, n);
        }
        off += n;
        if (size2 && off < len) {
            n = MIN(size2, len - off);
            gmac_ram_write(s, des[3] & 0x1FFFFFFF, data + off, n);
            off += n;
        }
        s->dma_regs[DMA_IDX(DMA_CUR_RX_BUF)] = des[2];
        done = off >= len;
        next = gmac_next_desc(s, base, addr, chain, end, des[3]);

        st = (nd ? 0 : RDES0_FS) | (MIN(off, 0x3fff) << RDES0_FL_SHIFT);
        gmac_desc_write_word(s, addr, 0, st);
        if (!done) {
            gmac_desc_read(s, next, nd_des);
            trunc = !(nd_des[0] & RDES0_OWN);
        }
        nd++;
        if (done || trunc) {
            if (trunc) {
                st = (MIN(off, 0x3fff) << RDES0_FL_SHIFT) |
                     RDES0_LS | RDES0_DE | RDES0_ES;
                s->dma_regs[DMA_IDX(DMA_MISSED)]++;
            } else {
                st |= RDES0_LS | flags;
                if (has_ext && (s->dma_regs[DMA_IDX(DMA_BUS_MODE)] &
                                DMA_BUS_MODE_ATDS)) {
                    gmac_desc_write_word(s, addr, 4, ext);
                }
            }
            gmac_desc_write_word(s, addr, 0, st);
            if (!(des[1] & RDES1_DIC)) {
                gmac_raise(s, DMA_STATUS_RI);
            }
            if (trunc || nd > 1) {
                gmac_desc_read(s, next, nd_des);
                if (!(nd_des[0] & RDES0_OWN)) {
                    s->rx_suspended = true;
                    gmac_raise(s, DMA_STATUS_RU);
                }
            }
            s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = next;
            return;
        }
        addr = next;
    }
}

/*
 * A frame arrives from the wire, FCS included. crc_ok says whether that FCS
 * checks out. The MTL forwards an errored or undersized frame by its
 * cut-through rule: the frame is already on its way to the DMA once the
 * threshold (RTC) plus the FCS has been received; a frame that ended earlier
 * is judged whole, and kept only if FEF (errored) or FUF (undersized) says
 * so. Store-and-forward (RSF) always judges whole.
 */
static bool gmac_rx_frame(IngenicGmacState *s, const uint8_t *buf,
                          size_t len, bool crc_ok)
{
    uint32_t cfg = s->mac_regs[MAC_IDX(MAC_CONFIG)];
    uint32_t ctl = s->dma_regs[DMA_IDX(DMA_CONTROL)];
    uint32_t flags = 0, ext = 0, limit, store = len;
    bool afm = false, saf = false, bad, ipc = false, ipc_err = false;
    uint16_t et;
    bool vlan;

    if (!(cfg & MAC_CONFIG_RE) || len < 14 + ETH_FCS_LEN) {
        return false;
    }
    if (!gmac_rx_filter(s, buf, &afm, &saf)) {
        return false;
    }
    if (afm) {
        flags |= RDES0_AFM;
    }
    if (saf) {
        flags |= RDES0_SAF;
    }

    et = lduw_be_p(buf + 12);
    vlan = et == ETH_P_VLAN;
    if (et >= 0x600) {
        flags |= RDES0_FT;
    }
    if (vlan) {
        flags |= RDES0_VLAN;
    }
    limit = (cfg & MAC_CONFIG_JE) ? 9018 : 1518;
    if (len > limit + (vlan && !(cfg & MAC_CONFIG_JE) ? 4 : 0)) {
        flags |= RDES0_GF | RDES0_ES;
    }
    if (!(cfg & MAC_CONFIG_WD)) {
        limit = (cfg & MAC_CONFIG_JE) ? 10240 : 2048;
        if (len > limit) {
            store = limit;
            crc_ok = false;
            flags |= RDES0_RWT | RDES0_ES;
            s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_RWT;
        }
    }
    if (!crc_ok) {
        flags |= RDES0_CE | RDES0_ES;
    }
    if (et < 0x600) {
        size_t payload = len - ETH_FCS_LEN - 14;

        if (et != payload && !(et < 46 && payload == 46)) {
            flags |= RDES0_LE;
        }
        if ((cfg & MAC_CONFIG_ACS) && 14u + et < store) {
            store = 14 + et;
        }
    }
    if ((cfg & MAC_CONFIG_IPC) && store == len &&
        gmac_rx_ipc(buf, len - ETH_FCS_LEN, &ext, &ipc_err)) {
        ipc = true;
        flags |= RDES0_ESA;
        if (ipc_err) {
            flags |= RDES0_ES;
        }
    }

    bad = !crc_ok || (ipc_err && !(ctl & DMA_CONTROL_DT)) ||
          (flags & RDES0_GF);
    if (bad || len < ETH_MIN_FRAME) {
        static const unsigned rtc[4] = { 64, 32, 96, 128 };
        unsigned thr = rtc[(ctl >> DMA_CONTROL_RTC_S) & 3];
        bool keep = bad ? (ctl & DMA_CONTROL_FEF) : (ctl & DMA_CONTROL_FUF);

        if (!keep && (ctl & DMA_CONTROL_RSF || len <= thr + ETH_FCS_LEN)) {
            return false;
        }
    }

    if (ipc && (ext & RDES4_IPPAY_ERR)) {
        s->dma_regs[DMA_IDX(DMA_STATUS)] |= DMA_STATUS_RWT;
    }
    gmac_rx_store(s, buf, store, flags, ext, ipc);
    return true;
}

static ssize_t ingenic_gmac_receive(NetClientState *nc,
                                    const uint8_t *buf, size_t len)
{
    IngenicGmacState *s = qemu_get_nic_opaque(nc);
    size_t plen = MAX(len, ETH_MIN_NOFCS);
    uint8_t *f;

    /*
     * The wire carries the padding and the FCS the backend left out. A
     * frame the ring has no room for is dropped, flagged as RU, as the MAC
     * does; returning 0 would stop delivery until a flush nothing
     * guarantees.
     */
    f = g_malloc(plen + ETH_FCS_LEN);
    memcpy(f, buf, len);
    memset(f + len, 0, plen - len);
    stl_le_p(f + plen, crc32(0, f, plen));
    gmac_rx_frame(s, f, plen + ETH_FCS_LEN, true);
    g_free(f);
    return len;
}

static uint32_t mac_write_mask(hwaddr off)
{
    switch (off) {
    case MAC_CONFIG:
        return 0x7adffeff;
    case MAC_FRAME_FILTER:
        return 0x800107ff;
    case 0x0018:                /* flow control */
        return 0xffff00bf;
    case 0x001c:                /* VLAN tag */
        return 0x000fffff;
    case MAC_VERSION ... 0x003c:
        return 0;
    case MAC_ADDR_HI(0):
        return 0x0000ffff;
    case MAC_ADDR_LO(0):
        return ~0u;
    case MAC_ADDR_HI(1) ... MAC_ADDR_LO(MAC_ADDR_COUNT - 1):
        return (off & 4) ? ~0u : 0xff00ffff;
    case 0x0100:                /* MMC control */
        return 0x2e;
    case 0x0104 ... 0x01fc:     /* MMC counters are not counted */
        return 0;
    default:
        return ~0u;
    }
}

static uint32_t dma_write_mask(hwaddr off)
{
    switch (off) {
    case DMA_RX_BASE_ADDR:
    case DMA_TX_BASE_ADDR:
        return 0xfffffffc;
    case DMA_CONTROL:
        return 0x0721ffde;
    case DMA_INTERRUPT:
        return 0x0001e7ff;
    case DMA_RIWT:
        return 0xff;
    case DMA_CUR_TX_DESC ... DMA_HW_FEATURE:
        return 0;
    default:
        return ~0u;
    }
}

static uint64_t ingenic_gmac_read(void *opaque, hwaddr offset,
                                  unsigned size)
{
    IngenicGmacState *s = INGENIC_GMAC(opaque);

    if (offset < DMA_BASE_OFFSET) {
        return s->mac_regs[MAC_IDX(offset)];
    }
    uint32_t dma_off = offset - DMA_BASE_OFFSET;
    if (dma_off < 0x1000) {
        uint32_t v;

        switch (dma_off) {
        case DMA_TX_POLL:
        case DMA_RX_POLL:
            return 0;
        case DMA_STATUS:
            return gmac_status(s);
        case DMA_MISSED:
            v = s->dma_regs[DMA_IDX(DMA_MISSED)];
            s->dma_regs[DMA_IDX(DMA_MISSED)] = 0;
            return v;
        case DMA_HW_FEATURE:
            return s->hw_feature;
        default:
            return s->dma_regs[DMA_IDX(dma_off)];
        }
    }
    return 0;
}

/* A poll demand resumes a suspended DMA only if a descriptor was freed */
static void gmac_rx_resume(IngenicGmacState *s)
{
    uint32_t addr = s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)];
    uint32_t des[4];

    if (!addr) {
        s->rx_suspended = false;
        return;
    }
    gmac_desc_read(s, addr, des);
    if (des[0] & RDES0_OWN) {
        s->rx_suspended = false;
    }
}

static void gmac_dma_reset(IngenicGmacState *s)
{
    memset(s->dma_regs, 0, sizeof(s->dma_regs));
    s->dma_regs[DMA_IDX(DMA_BUS_MODE)] = DMA_BUS_MODE_RESET |
                                         (s->has_phy ? 0 : DMA_BUS_MODE_SWR);
    s->rx_suspended = false;
    s->tx_stopped = false;
    s->tx_len = 0;
    gmac_update_irq(s);
}

static void gmac_mac_reset(IngenicGmacState *s)
{
    int i;

    memset(s->mac_regs, 0, sizeof(s->mac_regs));
    s->mac_regs[MAC_IDX(MAC_CONFIG)] = 0x8000;
    s->mac_regs[MAC_IDX(MAC_VERSION)] = 0x2037;
    s->mac_regs[MAC_IDX(MAC_ADDR_HI(0))] = ADDR_HI_AE | 0xffff;
    s->mac_regs[MAC_IDX(MAC_ADDR_LO(0))] = ~0u;
    for (i = 1; i < MAC_ADDR_COUNT; i++) {
        s->mac_regs[MAC_IDX(MAC_ADDR_HI(i))] = 0xffff;
        s->mac_regs[MAC_IDX(MAC_ADDR_LO(i))] = ~0u;
    }
}

static void ingenic_gmac_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned size)
{
    IngenicGmacState *s = INGENIC_GMAC(opaque);

    if (offset < DMA_BASE_OFFSET) {
        uint32_t idx = MAC_IDX(offset);

        if (offset == MAC_VERSION) {
            return;
        }
        s->mac_regs[idx] = ((uint32_t)value & mac_write_mask(offset)) |
                           (s->mac_regs[idx] & ~mac_write_mask(offset));
        if (offset == MAC_ADDR_HI(0)) {
            s->mac_regs[idx] |= ADDR_HI_AE;
        }
        if (offset == MAC_CONFIG) {
            /* The CPM's MAC PHY control reads this speed, as silicon does */
            qemu_set_irq(s->speed_100, !!(value & MAC_CONFIG_FES));
        }
        if (offset == MAC_MII_ADDR && (value & MII_ADDR_BUSY)) {
            if (value & MII_ADDR_WRITE) {
                ingenic_gmac_mdio_write(s);
            } else {
                ingenic_gmac_mdio_read(s);
            }
        }
        return;
    }

    uint32_t dma_off = offset - DMA_BASE_OFFSET;
    if (dma_off >= 0x1000) {
        return;
    }

    uint32_t idx = DMA_IDX(dma_off);
    uint32_t v = (uint32_t)value;

    switch (dma_off) {
    case DMA_BUS_MODE:
        if (v & DMA_BUS_MODE_SWR) {
            /*
             * measured: the software reset takes the MAC's registers too.
             * With no PHY on the bus there is no receive clock for it to
             * finish on, and SWR stays set, from power-on too: 0x00020101
             * with the clock just opened on a T10L, T20X, T23N, T30X, and
             * under Linux on a T23DL
             */
            gmac_mac_reset(s);
            gmac_dma_reset(s);
            v &= ~DMA_BUS_MODE_SWR;
            if (!v) {
                v = DMA_BUS_MODE_RESET;
            }
            if (!s->has_phy) {
                v |= DMA_BUS_MODE_SWR;
            }
        } else if (!s->has_phy) {
            v |= s->dma_regs[idx] & DMA_BUS_MODE_SWR;
        }
        s->dma_regs[idx] = v;
        break;
    case DMA_TX_BASE_ADDR:
        /* Setting TX base address resets the DMA's TX desc pointer to base */
        s->dma_regs[idx] = v & dma_write_mask(dma_off);
        s->dma_regs[DMA_IDX(DMA_CUR_TX_DESC)] = s->dma_regs[idx];
        break;
    case DMA_RX_BASE_ADDR:
        s->dma_regs[idx] = v & dma_write_mask(dma_off);
        s->dma_regs[DMA_IDX(DMA_CUR_RX_DESC)] = s->dma_regs[idx];
        break;
    case DMA_TX_POLL:
        ingenic_gmac_do_tx(s);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case DMA_RX_POLL:
        gmac_rx_resume(s);
        if (s->nic) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
        break;
    case DMA_STATUS:
        /* W1C: writing 1 to a status bit clears it. */
        s->dma_regs[idx] &= ~(v & DMA_STATUS_W1C);
        gmac_update_irq(s);
        break;
    case DMA_INTERRUPT:
        s->dma_regs[idx] = v & dma_write_mask(dma_off);
        gmac_update_irq(s);
        break;
    case DMA_CONTROL: {
        uint32_t old = s->dma_regs[idx];

        s->dma_regs[idx] = v & dma_write_mask(dma_off);
        if (!(v & DMA_CONTROL_ST)) {
            s->tx_stopped = false;
        } else if (!(old & DMA_CONTROL_ST)) {
            ingenic_gmac_do_tx(s);
        }
        if ((v & DMA_CONTROL_SR) && !(old & DMA_CONTROL_SR)) {
            gmac_rx_resume(s);
            if (s->nic) {
                qemu_flush_queued_packets(qemu_get_queue(s->nic));
            }
        }
        gmac_update_irq(s);
        break;
    }
    default:
        s->dma_regs[idx] = v & dma_write_mask(dma_off);
        break;
    }
}

static const MemoryRegionOps ingenic_gmac_ops = {
    .read = ingenic_gmac_read,
    .write = ingenic_gmac_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_gmac_phy_reset(IngenicGmacState *s)
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

static void ingenic_gmac_reset_hold(Object *obj, ResetType type)
{
    IngenicGmacState *s = INGENIC_GMAC(obj);

    gmac_mac_reset(s);
    gmac_dma_reset(s);
    ingenic_gmac_phy_reset(s);
}

static void ingenic_gmac_realize(DeviceState *dev, Error **errp)
{
    IngenicGmacState *s = INGENIC_GMAC(dev);

    static NetClientInfo net_info = {
        .type = NET_CLIENT_DRIVER_NIC,
        .size = sizeof(NICState),
        .can_receive = ingenic_gmac_can_receive,
        .receive = ingenic_gmac_receive,
        .link_status_changed = ingenic_gmac_set_link_status,
    };

    s->nic = qemu_new_nic(&net_info, &s->conf, TYPE_INGENIC_GMAC,
                           dev->id, &dev->mem_reentrancy_guard, s);
}

static void ingenic_gmac_init(Object *obj)
{
    IngenicGmacState *s = INGENIC_GMAC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_gmac_ops, s,
                          TYPE_INGENIC_GMAC, INGENIC_GMAC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->speed_100, "speed-100", 1);
}

static int ingenic_gmac_post_load(void *opaque, int version_id)
{
    IngenicGmacState *s = opaque;

    if (s->nic) {
        ingenic_gmac_set_link_status(qemu_get_queue(s->nic));
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_gmac = {
    .name = TYPE_INGENIC_GMAC,
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = ingenic_gmac_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mac_regs, IngenicGmacState,
                             INGENIC_GMAC_MAC_REGS),
        VMSTATE_UINT32_ARRAY(dma_regs, IngenicGmacState,
                             INGENIC_GMAC_DMA_REGS),
        VMSTATE_UINT16_ARRAY(phy_regs, IngenicGmacState,
                             INGENIC_GMAC_PHY_REGS),
        VMSTATE_BOOL_V(rx_suspended, IngenicGmacState, 2),
        VMSTATE_BOOL_V(tx_stopped, IngenicGmacState, 2),
        VMSTATE_END_OF_LIST()
    }
};

/*
 * "phy" off for a board with no PHY on the MDIO bus, which reads all ones,
 * or with "mdio-hold" the bus as a T23DL's, with no pull-up: a read nothing
 * answers returns the register number's low bit sixteen times, so that a
 * U-Boot takes ID 0x0000ffff at address 0 for a PHY
 */
static const Property ingenic_gmac_props[] = {
    DEFINE_NIC_PROPERTIES(IngenicGmacState, conf),
    DEFINE_PROP_BOOL("phy", IngenicGmacState, has_phy, true),
    DEFINE_PROP_BOOL("mdio-hold", IngenicGmacState, mdio_hold, false),
    DEFINE_PROP_UINT32("hw-feature", IngenicGmacState, hw_feature,
                       0x090d0935),
};

static void ingenic_gmac_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_gmac_realize;
    dc->vmsd = &vmstate_ingenic_gmac;
    device_class_set_props(dc, ingenic_gmac_props);
    rc->phases.hold = ingenic_gmac_reset_hold;
}

static const TypeInfo ingenic_gmac_type_info = {
    .name = TYPE_INGENIC_GMAC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicGmacState),
    .instance_init = ingenic_gmac_init,
    .class_init = ingenic_gmac_class_init,
};

static void ingenic_gmac_register_types(void)
{
    type_register_static(&ingenic_gmac_type_info);
}

type_init(ingenic_gmac_register_types)
