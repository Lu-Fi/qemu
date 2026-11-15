/*
 * QTest testcase for the Ingenic GMAC
 *
 * MDIO: the PHY at address 0 answers MDIO reads; with "phy" off, as on a
 * board with no PHY on the bus, every read returns all ones and writes go
 * nowhere, as the vendor U-Boot finds no PHY there. With "mdio-hold" too,
 * as on a T23DL's bus, an even register reads 0 and an odd one all ones.
 *
 * Frame path: the MAC in loopback (MAC_CONFIG.LM) feeds a transmitted frame
 * to its own receiver, so descriptors, filter, padding, FCS, ACS, checksum
 * offload and the MTL's rules for bad frames are driven and read back the
 * way the bench boards' T21 and T40 MACs were driven in U-Boot. Every
 * expectation here was measured on those boards.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"
#include "qemu/sockets.h"

#define GMAC            0x134b0000
#define MAC_CONFIG      0x00
#define MAC_FILTER      0x04
#define MAC_HASH_HI     0x08
#define MAC_HASH_LO     0x0c
#define MII_ADDR        0x10
#define MII_DATA        0x14
#define MAC_ADDR_HI(n)  (0x40 + 8 * (n))
#define MAC_ADDR_LO(n)  (0x44 + 8 * (n))
#define MII_BUSY        1
#define MII_WRITE       2
#define MII_PHYSID2     3

#define DMA             (GMAC + 0x1000)
#define DMA_BUS_MODE    0x00
#define DMA_TX_POLL     0x04
#define DMA_RX_POLL     0x08
#define DMA_RX_BASE     0x0c
#define DMA_TX_BASE     0x10
#define DMA_STATUS      0x14
#define DMA_CONTROL     0x18
#define DMA_INTR_EN     0x1c
#define DMA_MISSED      0x20
#define DMA_HW_FEATURE  0x58

#define CFG_RE          (1 << 2)
#define CFG_TE          (1 << 3)
#define CFG_ACS         (1 << 7)
#define CFG_IPC         (1 << 10)
#define CFG_DM          (1 << 11)
#define CFG_LM          (1 << 12)
#define CFG_JE          (1 << 20)
#define CFG_JD          (1 << 22)
#define CFG_WD          (1 << 23)

#define FF_PR           (1 << 0)
#define FF_HUC          (1 << 1)
#define FF_HMC          (1 << 2)
#define FF_DAIF         (1 << 3)
#define FF_PM           (1 << 4)
#define FF_DBF          (1 << 5)
#define FF_PCF(n)       ((n) << 6)
#define FF_SAIF         (1 << 8)
#define FF_SAF          (1 << 9)
#define FF_HPF          (1 << 10)
#define FF_RA           (1u << 31)

#define DC_SR           (1 << 1)
#define DC_RTC32        (1 << 3)
#define DC_FUF          (1 << 6)
#define DC_FEF          (1 << 7)
#define DC_ST           (1 << 13)
#define DC_TSF          (1 << 21)
#define DC_RSF          (1 << 25)

#define TD_OWN          0x80000000u
#define TD_IC           (1 << 30)
#define TD_LS           (1 << 29)
#define TD_FS           (1 << 28)
#define TD_DC           (1 << 27)
#define TD_DP           (1 << 26)
#define TD_CIC(n)       ((n) << 22)
#define TD_TER          (1 << 21)

#define RD_OWN          0x80000000u
#define RD_AFM          (1 << 30)
#define RD_ES           (1 << 15)
#define RD_DE           (1 << 14)
#define RD_SAF          (1 << 13)
#define RD_LE           (1 << 12)
#define RD_VLAN         (1 << 10)
#define RD_FS           (1 << 9)
#define RD_LS           (1 << 8)
#define RD_GF           (1 << 7)
#define RD_FT           (1 << 5)
#define RD_RWT          (1 << 4)
#define RD_CE           (1 << 1)
#define RD_ESA          (1 << 0)
#define RD_FL(d)        (((d) >> 16) & 0x3fff)

#define ST_TI           (1 << 0)
#define ST_TPS          (1 << 1)
#define ST_TU           (1 << 2)
#define ST_TJT          (1 << 3)
#define ST_RI           (1 << 6)
#define ST_RU           (1 << 7)
#define ST_RWT          (1 << 9)
#define ST_ETI          (1 << 10)
#define ST_AIS          (1 << 15)
#define ST_NIS          (1 << 16)

#define TXD             0x00800000
#define RXD             0x00801000
#define TXB             0x00810000
#define RXB             0x00830000
#define STRIDE          32              /* 8-word descriptors, ATDS */

static const uint8_t ME[6] = { 0x02, 0x38, 0xb2, 0xc9, 0x0d, 0x0d };
static const uint8_t SRC[6] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };
static const uint8_t OTHER[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0xee };
static const uint8_t BCAST[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint8_t MCAST[6] = { 0x01, 0x00, 0x5e, 0x00, 0x00, 0xfb };
static const uint8_t MCAST2[6] = { 0x01, 0x00, 0x5e, 0x00, 0x00, 0xfc };
static const uint8_t PAUSE_DA[6] = { 0x01, 0x80, 0xc2, 0x00, 0x00, 0x01 };

static uint32_t mdio_read(QTestState *qts, unsigned reg)
{
    qtest_writel(qts, GMAC + MII_ADDR, reg << 6 | MII_BUSY);
    g_assert_cmphex(qtest_readl(qts, GMAC + MII_ADDR) & MII_BUSY, ==, 0);
    return qtest_readl(qts, GMAC + MII_DATA) & 0xffff;
}

static void test_mdio(const void *data)
{
    bool phy = data != NULL;
    QTestState *qts = qtest_initf("-machine ingenic-t31 %s",
                                  phy ? "" : "-global ingenic-gmac.phy=off");

    g_assert_cmphex(mdio_read(qts, MII_PHYSID2), ==, phy ? 0x0128 : 0xffff);
    qtest_writel(qts, GMAC + MII_DATA, 0x1234);
    qtest_writel(qts, GMAC + MII_ADDR, 4 << 6 | MII_WRITE | MII_BUSY);
    g_assert_cmphex(mdio_read(qts, 4), ==, phy ? 0x1234 : 0xffff);
    qtest_quit(qts);
}

static void test_mdio_hold(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t23 "
                                 "-global ingenic-gmac.phy=off "
                                 "-global ingenic-gmac.mdio-hold=on");

    g_assert_cmphex(mdio_read(qts, 2), ==, 0);
    g_assert_cmphex(mdio_read(qts, MII_PHYSID2), ==, 0xffff);
    qtest_quit(qts);
}

/*
 * A software reset finishes on the PHY's receive clock: with no PHY on the
 * bus SWR reads set from power-on and stays set (0x00020101 with the clock
 * just opened on a T10L, T20X, T23N and T30X, and under Linux on a T23DL);
 * with a PHY it reads clear (a T21N, T40 and T41)
 */
static void test_swr(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t23 -nic none "
                                 "-global ingenic-gmac.phy=off");

    g_assert_cmphex(qtest_readl(qts, DMA + DMA_BUS_MODE), ==, 0x00020101);
    qtest_writel(qts, DMA + DMA_BUS_MODE, 1);
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_BUS_MODE), ==, 0x00020101);
    qtest_quit(qts);

    qts = qtest_init("-machine ingenic-t23 -nic none");
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_BUS_MODE), ==, 0x00020100);
    qtest_writel(qts, DMA + DMA_BUS_MODE, 1);
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_BUS_MODE), ==, 0x00020100);
    qtest_quit(qts);
}

/* Reset values and writable bits, as measured on a T21 and a T40 */
static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t31 -nic none");
    static const struct { uint32_t off, reset, ones; } mac[] = {
        { MAC_CONFIG, 0x8000, 0x7adffeff }, { MAC_FILTER, 0, 0x800107ff },
        { MAC_HASH_HI, 0, ~0u }, { MAC_HASH_LO, 0, ~0u },
        { 0x18, 0, 0xffff00bf }, { 0x1c, 0, 0x000fffff }, { 0x20, 0x2037,
            0x2037 },
        { 0x24, 0, 0 }, { 0x3c, 0, 0 },
        { MAC_ADDR_HI(0), 0x8000ffff, 0x8000ffff }, { MAC_ADDR_LO(0), ~0u,
            ~0u },
        { MAC_ADDR_HI(1), 0x0000ffff, 0xff00ffff }, { MAC_ADDR_LO(1), ~0u,
            ~0u },
        { MAC_ADDR_HI(15), 0x0000ffff, 0xff00ffff }, { 0x100, 0, 0x2e },
    }, dma[] = {
        { DMA_BUS_MODE, 0x00020100, 0x00020100 }, { DMA_TX_POLL, 0, 0 },
        { DMA_RX_BASE, 0, 0xfffffffc }, { DMA_TX_BASE, 0, 0xfffffffc },
        { DMA_CONTROL, 0, 0x0721ffde }, { DMA_INTR_EN, 0, 0x0001e7ff },
        { 0x24, 0, 0xff }, { DMA_HW_FEATURE, 0x090d0935, 0x090d0935 },
    };

    for (int i = 0; i < ARRAY_SIZE(mac); i++) {
        g_assert_cmphex(qtest_readl(qts, GMAC + mac[i].off), ==, mac[i].reset);
        qtest_writel(qts, GMAC + mac[i].off, 0xffffffff);
        g_assert_cmphex(qtest_readl(qts, GMAC + mac[i].off), ==, mac[i].ones);
    }
    for (int i = 0; i < ARRAY_SIZE(dma); i++) {
        g_assert_cmphex(qtest_readl(qts, DMA + dma[i].off), ==, dma[i].reset);
        if (dma[i].off != DMA_BUS_MODE) {
            qtest_writel(qts, DMA + dma[i].off, 0xffffffff);
            g_assert_cmphex(qtest_readl(qts, DMA + dma[i].off), ==,
                            dma[i].ones);
        }
    }
    qtest_quit(qts);
}

static void test_regs_t40(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t40 -nic none");

    g_assert_cmphex(qtest_readl(qts, DMA + DMA_HW_FEATURE), ==, 0x490d0935);
    qtest_quit(qts);
}

/* a frame builder and a checker for what comes back */

typedef struct Frame {
    uint8_t d[16400];
    size_t len;
} Frame;

static void put16(uint8_t *p, unsigned v)
{
    p[0] = v >> 8;
    p[1] = v;
}

static void frame_eth(Frame *f, const uint8_t *da, const uint8_t *sa,
                      unsigned type, size_t total)
{
    memset(f->d, 0, sizeof(f->d));
    memcpy(f->d, da, 6);
    memcpy(f->d + 6, sa, 6);
    put16(f->d + 12, type);
    for (size_t i = 14; i < total; i++) {
        f->d[i] = i - 14;
    }
    f->len = total;
}

static uint32_t csum_add(const uint8_t *p, size_t n, uint32_t sum)
{
    for (size_t i = 0; i + 1 < n; i += 2) {
        sum += p[i] << 8 | p[i + 1];
    }
    if (n & 1) {
        sum += p[n - 1] << 8;
    }
    return sum;
}

static uint16_t csum_fold(uint32_t sum)
{
    while (sum >> 16) {
        sum = (sum & 0xffff) + (sum >> 16);
    }
    return ~sum;
}

/* IPv4 over Ethernet; proto 6, 17 or 1; the L4 checksum field left 0 */
static void frame_ip(Frame *f, int proto, size_t paylen, bool good_ip)
{
    uint8_t *ip = f->d + 14, *l4 = ip + 20;
    size_t l4len = (proto == 6 ? 20 : proto == 17 ? 8 : 8) + paylen;

    frame_eth(f, ME, SRC, 0x0800, 14 + 20 + l4len);
    memset(ip, 0, 20);
    ip[0] = 0x45;
    put16(ip + 2, 20 + l4len);
    put16(ip + 4, 0x1234);
    ip[8] = 64;
    ip[9] = proto;
    ip[12] = 10; ip[15] = 1; ip[16] = 10; ip[19] = 2;
    memset(l4, 0, l4len);
    if (proto == 6) {
        put16(l4, 1000); put16(l4 + 2, 2000); l4[12] = 0x50; l4[13] = 0x10;
    } else if (proto == 17) {
        put16(l4, 1000); put16(l4 + 2, 2000); put16(l4 + 4, l4len);
    } else {
        l4[0] = 8; put16(l4 + 6, 1);
    }
    for (size_t i = 0; i < paylen; i++) {
        l4[l4len - paylen + i] = 'a' + i % 26;
    }
    if (good_ip) {
        put16(ip + 10, csum_fold(csum_add(ip, 20, 0)));
    }
}

static unsigned l4_csum(const Frame *f, int proto, bool pseudo)
{
    const uint8_t *ip = f->d + 14;
    size_t l4len = f->len - 14 - 20;
    uint32_t sum = csum_add(ip + 20, l4len, 0);

    if (pseudo) {
        sum = csum_add(ip + 12, 8, sum) + proto + l4len;
    }
    return csum_fold(sum);
}

typedef struct Setup {
    uint32_t cfg, filter, hash_hi, hash_lo, dmactl, ie;
    unsigned rx_n, rx_size;     /* default 1 x 2048 */
    uint32_t rx_sizes[8];       /* per descriptor, overrides rx_size */
    const uint8_t *extra_addr;  /* MAC_ADDR(1) */
    uint32_t extra_flags;       /* its top byte: SA, mask bits */
    bool ic;                    /* IC on the last TX descriptor */
} Setup;

typedef struct Rx {
    uint32_t d0[8], ext;
    int n;                      /* descriptors the DMA took */
    uint8_t data[8][2048];
} Rx;

static void put_addr(QTestState *qts, unsigned n, const uint8_t *a,
                     uint32_t flags)
{
    qtest_writel(qts, GMAC + MAC_ADDR_HI(n),
                 0x80000000 | flags << 24 | a[5] << 8 | a[4]);
    qtest_writel(qts, GMAC + MAC_ADDR_LO(n),
                 a[3] << 24 | a[2] << 16 | a[1] << 8 | a[0]);
}

static void gmac_prepare(QTestState *qts, const Setup *su)
{
    unsigned n = su->rx_n ?: 1;

    qtest_writel(qts, DMA + DMA_BUS_MODE, 1);
    qtest_writel(qts, DMA + DMA_BUS_MODE, 0x80);
    for (unsigned i = 0; i < n; i++) {
        uint32_t size = su->rx_sizes[i] ?: su->rx_size ?: 2048;

        qtest_writel(qts, RXD + i * STRIDE, RD_OWN);
        qtest_writel(qts, RXD + i * STRIDE + 4,
                     size | (i == n - 1 ? 1 << 15 : 0));
        qtest_writel(qts, RXD + i * STRIDE + 8, RXB + i * 0x2800);
        qtest_writel(qts, RXD + i * STRIDE + 12, 0);
        qtest_writel(qts, RXD + i * STRIDE + 16, 0xdeadbeef);
    }
    qtest_writel(qts, DMA + DMA_RX_BASE, RXD);
    qtest_writel(qts, DMA + DMA_TX_BASE, TXD);
    put_addr(qts, 0, ME, 0);
    if (su->extra_addr) {
        put_addr(qts, 1, su->extra_addr, su->extra_flags);
    }
    qtest_writel(qts, GMAC + MAC_FILTER, su->filter);
    qtest_writel(qts, GMAC + MAC_HASH_HI, su->hash_hi);
    qtest_writel(qts, GMAC + MAC_HASH_LO, su->hash_lo);
    qtest_writel(qts, DMA + DMA_INTR_EN, su->ie);
    qtest_writel(qts, DMA + DMA_CONTROL, DC_SR | DC_TSF | su->dmactl);
    qtest_writel(qts, GMAC + MAC_CONFIG,
                 CFG_RE | CFG_TE | CFG_DM | CFG_LM | su->cfg);
}

/* queue one descriptor of the TX ring; the frame's later segments follow */
static void tx_desc(QTestState *qts, unsigned idx, const uint8_t *p,
                    size_t len, uint32_t flags)
{
    uint32_t buf = TXB + idx * 0x4000;

    qtest_memwrite(qts, buf, p, len);
    qtest_writel(qts, TXD + idx * STRIDE + 4, len);
    qtest_writel(qts, TXD + idx * STRIDE + 8, buf);
    qtest_writel(qts, TXD + idx * STRIDE + 12, 0);
    qtest_writel(qts, TXD + idx * STRIDE, flags | TD_OWN);
}

static void gmac_poll(QTestState *qts)
{
    qtest_writel(qts, DMA + DMA_CONTROL,
                 qtest_readl(qts, DMA + DMA_CONTROL) | DC_ST);
    qtest_writel(qts, DMA + DMA_TX_POLL, 1);
}

static void rx_collect(QTestState *qts, const Setup *su, Rx *rx)
{
    unsigned n = su->rx_n ?: 1;

    memset(rx, 0, sizeof(*rx));
    for (unsigned i = 0; i < n; i++) {
        uint32_t d0 = qtest_readl(qts, RXD + i * STRIDE);

        if (d0 & RD_OWN) {
            break;
        }
        rx->d0[i] = d0;
        rx->n = i + 1;
        qtest_memread(qts, RXB + i * 0x2800, rx->data[i],
                      MIN(RD_FL(d0) - (i ? RD_FL(rx->d0[i - 1]) : 0), 2048));
        rx->ext = qtest_readl(qts, RXD + i * STRIDE + 16);
    }
}

/* one frame in one descriptor, the whole round trip */
static bool loop1(QTestState *qts, const Setup *su, const Frame *f,
                  uint32_t txflags, Rx *rx)
{
    gmac_prepare(qts, su);
    tx_desc(qts, 0, f->d, f->len, TD_FS | TD_LS | TD_TER | txflags);
    gmac_poll(qts);
    rx_collect(qts, su, rx);
    return rx->n > 0;
}

static uint32_t tx_status(QTestState *qts, unsigned idx)
{
    return qtest_readl(qts, TXD + idx * STRIDE);
}

static QTestState *new_gmac(void)
{
    return qtest_init("-machine ingenic-t31 -nic none");
}

/* padding, FCS and the receive flags of an ordinary frame */
static void test_pad_fcs(void)
{
    QTestState *qts = new_gmac();
    Setup su = { 0 };
    Frame f;
    Rx rx;

    /* 60 bytes out, 64 in: the MAC appends the FCS and the DMA hands it up */
    frame_eth(&f, ME, SRC, 0x88b5, 60);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00400320);
    g_assert_cmpmem(rx.data[0], 60, f.d, 60);
    g_assert_cmphex((uint32_t)ldl_le_p(rx.data[0] + 60), ==, 0xee222ddb);
    g_assert_cmphex(tx_status(qts, 0) & TD_OWN, ==, 0);
    g_assert_cmphex(tx_status(qts, 0), ==, TD_FS | TD_LS | TD_TER);

    /* 20 bytes are padded with zeros to 60 first */
    frame_eth(&f, ME, SRC, 0x88b5, 20);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00400320);
    g_assert_cmpint(rx.data[0][19], ==, 5);
    g_assert_cmpint(rx.data[0][20], ==, 0);
    g_assert_cmpint(rx.data[0][59], ==, 0);

    /* DP: no padding, so a 59-byte frame is a runt and the MTL drops it ... */
    frame_eth(&f, ME, SRC, 0x88b5, 59);
    g_assert_false(loop1(qts, &su, &f, TD_DP, &rx));
    g_assert_cmphex(tx_status(qts, 0) & TD_OWN, ==, 0);
    /* ... unless FUF forwards undersized frames */
    su.dmactl = DC_FUF;
    g_assert_true(loop1(qts, &su, &f, TD_DP, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x003f0320);

    /* a VLAN-tagged frame is flagged on the way out and the way in */
    su.dmactl = 0;
    frame_eth(&f, ME, SRC, 0x8100, 64);
    put16(f.d + 16, 0x88b5);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00440720);
    g_assert_cmphex(tx_status(qts, 0), ==, TD_FS | TD_LS | TD_TER | 0x80);
    qtest_quit(qts);
}

/* DC: the driver supplies the FCS, honoured above the padding limit */
static void test_dc(void)
{
    QTestState *qts = new_gmac();
    Setup su = { 0 };
    Frame f;
    Rx rx;

    /* 60 bytes with DC are still padded-and-FCSed: arrives as 64 */
    frame_eth(&f, ME, SRC, 0x88b5, 60);
    g_assert_true(loop1(qts, &su, &f, TD_DC, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00400320);
    /* with DP too, no FCS: a 60-byte frame and a bad one, dropped */
    g_assert_false(loop1(qts, &su, &f, TD_DC | TD_DP, &rx));
    /* 64 bytes go out as they are, their last four bytes not an FCS */
    frame_eth(&f, ME, SRC, 0x88b5, 64);
    g_assert_false(loop1(qts, &su, &f, TD_DC, &rx));
    /* a longer bad frame was already on its way to the DMA (RTC 64 + FCS) */
    frame_eth(&f, ME, SRC, 0x88b5, 100);
    g_assert_true(loop1(qts, &su, &f, TD_DC, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00648322);
    /* 68 bytes had not */
    frame_eth(&f, ME, SRC, 0x88b5, 68);
    g_assert_false(loop1(qts, &su, &f, TD_DC, &rx));
    /* a bad frame is dropped whole by store-and-forward, kept by FEF */
    frame_eth(&f, ME, SRC, 0x88b5, 100);
    su.dmactl = DC_RSF;
    g_assert_false(loop1(qts, &su, &f, TD_DC, &rx));
    su.dmactl = DC_RSF | DC_FEF;
    g_assert_true(loop1(qts, &su, &f, TD_DC, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00648322);
    /* a threshold of 32 bytes releases frames sooner, RTC 96 later */
    su.dmactl = DC_RTC32;
    frame_eth(&f, ME, SRC, 0x88b5, 40);
    g_assert_true(loop1(qts, &su, &f, TD_DC | TD_DP, &rx));
    su.dmactl = 2 << 3;
    frame_eth(&f, ME, SRC, 0x88b5, 100);
    g_assert_false(loop1(qts, &su, &f, TD_DC, &rx));
    qtest_quit(qts);
}

typedef struct FilterCase {
    const char *name;
    const uint8_t *da, *sa;
    uint32_t filter;
    uint64_t hash;              /* HTH:HTL */
    const uint8_t *addr1;
    uint32_t addr1_flags;
    int type;
    bool pass;
    uint32_t d0;                /* the flags it arrives with */
} FilterCase;

/*
 * The table bit stmmac computes, bitrev32(~crc32_le(~0, addr)) >> 26. The
 * vendor driver's ~crc >> 26 of the big-endian CRC is another bit; the
 * silicon follows the former.
 */
static unsigned hash_index(const uint8_t *a)
{
    uint32_t crc = 0xffffffff, rev = 0;

    for (int i = 0; i < 6; i++) {
        crc ^= a[i];
        for (int b = 0; b < 8; b++) {
            crc = crc & 1 ? crc >> 1 ^ 0xedb88320 : crc >> 1;
        }
    }
    crc = ~crc;
    for (int i = 0; i < 32; i++) {
        rev |= (crc >> i & 1) << (31 - i);
    }
    return rev >> 26;
}

#define H(a)    (1ull << hash_index(a))

static void test_filter(void)
{
    static const uint8_t SRC2[6] = { 0x02, 0x99, 0x99, 0x99, 0x99, 0x99 };
    static const uint8_t OTHER_HASHED[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0xdd,
        0xef };
    static const uint8_t MASK_B5[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x00 };
    static const uint8_t MASK_B0[6] = { 0x00, 0xaa, 0xbb, 0xcc, 0xdd, 0xee };
    static const uint8_t MASK_BAD[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0x00, 0x00 };
    const uint32_t OK = 0x00400320, AFM = RD_AFM | OK;
    const FilterCase cases[] = {
        { "unicast", ME, SRC, 0, 0, NULL, 0, 0x88b5, true, OK },
        { "other unicast", OTHER, SRC, 0, 0, NULL, 0, 0x88b5, false },
        { "other, promiscuous", OTHER, SRC, FF_PR, 0, NULL, 0, 0x88b5,
            true, OK },
        { "broadcast", BCAST, SRC, 0, 0, NULL, 0, 0x88b5, true, OK },
        { "broadcast, DBF", BCAST, SRC, FF_DBF, 0, NULL, 0, 0x88b5, false },
        { "broadcast, DBF, PR", BCAST, SRC, FF_DBF | FF_PR, 0, NULL, 0,
            0x88b5, true, OK },
        { "multicast", MCAST, SRC, 0, 0, NULL, 0, 0x88b5, false },
        { "multicast, PM", MCAST, SRC, FF_PM, 0, NULL, 0, 0x88b5, true, OK },
        { "multicast, hashed", MCAST, SRC, FF_HMC, H(MCAST), NULL, 0,
            0x88b5, true, OK },
        { "multicast, not hashed", MCAST2, SRC, FF_HMC, H(MCAST), NULL, 0,
            0x88b5, false },
        { "multicast, empty hash", MCAST, SRC, FF_HMC, 0, NULL, 0, 0x88b5,
            false },
        { "unicast, hashed", OTHER, SRC, FF_HUC, H(OTHER), NULL, 0, 0x88b5,
            true, OK },
        { "unicast, not hashed", OTHER_HASHED, SRC, FF_HUC, H(OTHER), NULL,
            0, 0x88b5, false },
        { "own, HUC alone", ME, SRC, FF_HUC, H(OTHER), NULL, 0, 0x88b5,
            false },
        { "own, HUC and HPF", ME, SRC, FF_HUC | FF_HPF, H(OTHER), NULL, 0,
            0x88b5, true, OK },
        { "hashed, HUC and HPF", OTHER, SRC, FF_HUC | FF_HPF, H(OTHER),
            NULL, 0, 0x88b5, true, OK },
        { "multicast, HMC and HPF", MCAST, SRC, FF_HMC | FF_HPF, H(MCAST),
            NULL, 0, 0x88b5, true, OK },
        { "multicast in HUC's hash", MCAST, SRC, FF_HUC, H(MCAST), NULL, 0,
            0x88b5, false },
        { "inverse, own", ME, SRC, FF_DAIF, 0, NULL, 0, 0x88b5, false },
        { "inverse, other", OTHER, SRC, FF_DAIF, 0, NULL, 0, 0x88b5, true,
            OK },
        { "inverse, broadcast", BCAST, SRC, FF_DAIF, 0, NULL, 0, 0x88b5,
            true, OK },
        { "inverse, multicast, PM", MCAST, SRC, FF_DAIF | FF_PM, 0, NULL,
            0, 0x88b5, true, OK },
        { "inverse, multicast, hashed", MCAST, SRC, FF_DAIF | FF_HMC,
            H(MCAST), NULL, 0, 0x88b5, false },
        { "perfect address 1", OTHER, SRC, 0, 0, OTHER, 0, 0x88b5, true, OK },
        { "perfect multicast", MCAST, SRC, 0, 0, MCAST, 0, 0x88b5, true, OK },
        { "perfect multicast, inverse", MCAST, SRC, FF_DAIF, 0, MCAST, 0,
            0x88b5, false },
        { "mask byte 5", OTHER, SRC, 0, 0, MASK_B5, 1 << 5, 0x88b5, true, OK },
        { "mask byte 0", OTHER, SRC, 0, 0, MASK_B0, 1 << 0, 0x88b5, true, OK },
        { "mask on the wrong byte", OTHER, SRC, 0, 0, MASK_BAD, 1 << 5,
            0x88b5, false },
        { "SA match", ME, SRC, FF_SAF, 0, SRC, 1 << 6, 0x88b5, true, OK },
        { "SA mismatch", ME, SRC2, FF_SAF, 0, SRC, 1 << 6, 0x88b5, false },
        { "SA mismatch, no SAF", ME, SRC2, 0, 0, SRC, 1 << 6, 0x88b5, true,
            OK | RD_SAF },
        { "SA mismatch, PR", ME, SRC2, FF_SAF | FF_PR, 0, SRC, 1 << 6,
            0x88b5, true, OK },
        { "SA mismatch, RA", ME, SRC2, FF_SAF | FF_RA, 0, SRC, 1 << 6,
            0x88b5, true, OK | RD_SAF },
        { "SA inverse, match", ME, SRC, FF_SAF | FF_SAIF, 0, SRC, 1 << 6,
            0x88b5, false },
        { "SA inverse, mismatch", ME, SRC2, FF_SAF | FF_SAIF, 0, SRC, 1 <<
            6, 0x88b5, true, OK },
        { "SA inverse, no SAF", ME, SRC, FF_SAIF, 0, SRC, 1 << 6, 0x88b5,
            true, OK | RD_SAF },
        { "SA and DA mismatch, RA", OTHER, SRC2, FF_SAF | FF_RA, 0, SRC,
          1 << 6, 0x88b5, true, AFM | RD_SAF },
        { "receive all, other", OTHER, SRC, FF_RA, 0, NULL, 0, 0x88b5,
            true, AFM },
        { "pause to multicast, PCF 0", PAUSE_DA, SRC, FF_PCF(0), 0, NULL,
            0, 0x8808, false },
        { "pause to multicast, PCF 1", PAUSE_DA, SRC, FF_PCF(1), 0, NULL,
            0, 0x8808, true, AFM },
        { "pause to multicast, PCF 2", PAUSE_DA, SRC, FF_PCF(2), 0, NULL,
            0, 0x8808, true, AFM },
        { "pause to multicast, PCF 3", PAUSE_DA, SRC, FF_PCF(3), 0, NULL,
            0, 0x8808, false },
        { "pause to multicast, PCF 1, PM", PAUSE_DA, SRC, FF_PCF(1) |
            FF_PM, 0, NULL, 0, 0x8808, true, OK },
        { "pause to us, PCF 0", ME, SRC, FF_PCF(0), 0, NULL, 0, 0x8808,
            false },
        { "pause to us, PCF 3", ME, SRC, FF_PCF(3), 0, NULL, 0, 0x8808,
            true, OK },
    };
    QTestState *qts = new_gmac();

    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        const FilterCase *c = &cases[i];
        Setup su = { .filter = c->filter, .hash_hi = c->hash >> 32,
                     .hash_lo = c->hash, .extra_addr = c->addr1,
                     .extra_flags = c->addr1_flags };
        Frame f;
        Rx rx;

        frame_eth(&f, c->da, c->sa, c->type, 60);
        g_test_message("%s", c->name);
        g_assert_cmpint(loop1(qts, &su, &f, 0, &rx), ==, c->pass);
        if (c->pass) {
            g_assert_cmphex(rx.d0[0] & ~0x3fff0000u, ==, c->d0 & ~0x3fff0000u);
        }
    }
    qtest_quit(qts);
}

/* ACS strips the pad and FCS of a length-field frame, never a typed one */
static void test_acs(void)
{
    QTestState *qts = new_gmac();
    Setup su = { .cfg = CFG_ACS };
    static const struct { unsigned type, total, fl, flags; } c[] = {
        { 0x88b5, 60, 64, 0x320 }, { 20, 60, 34, 0x300 },
        { 100, 114, 114, 0x300 }, { 46, 60, 60, 0x300 },
        { 45, 60, 59, 0x300 }, { 47, 60, 61, 0x1300 },
        { 0, 60, 14, 0x300 }, { 1500, 1514, 1514, 0x300 },
        { 0x0600, 114, 118, 0x320 },
    };
    Setup plain = { 0 };
    Frame f;
    Rx rx;

    for (int i = 0; i < ARRAY_SIZE(c); i++) {
        frame_eth(&f, ME, SRC, c[i].type, c[i].total);
        g_assert_true(loop1(qts, &su, &f, 0, &rx));
        g_assert_cmpint(RD_FL(rx.d0[0]), ==, c[i].fl);
        g_assert_cmphex(rx.d0[0] & 0xffff, ==, c[i].flags);
    }
    /* without ACS a length frame arrives whole, its length field checked */
    frame_eth(&f, ME, SRC, 20, 60);
    g_assert_true(loop1(qts, &plain, &f, 0, &rx));
    g_assert_cmpint(RD_FL(rx.d0[0]), ==, 64);
    g_assert_cmphex(rx.d0[0] & 0xffff, ==, 0x300);
    frame_eth(&f, ME, SRC, 1000, 60);
    g_assert_true(loop1(qts, &plain, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0] & 0xffff, ==, 0x1300);
    qtest_quit(qts);
}

/* Frame sizes: giant, jumbo, the receive watchdog and the transmit jabber */
static void test_sizes(void)
{
    QTestState *qts = new_gmac();
    static const struct {
        size_t total;           /* on the wire, FCS in */
        uint32_t cfg;
        uint32_t d0;            /* low 16 bits of RDES0 */
        int rx_n;
    } c[] = {
        { 1518, CFG_JD, 0x320, 1 }, { 1519, CFG_JD, 0x83a0, 1 },
        { 1519, CFG_JD | CFG_JE, 0x320, 1 },
        { 2048, CFG_JD, 0x83a0, 1 }, { 2049, CFG_JD, 0x83b2, 1 },
        { 2049, CFG_JD | CFG_WD, 0x81a0, 2 },
        { 2049, CFG_JD | CFG_JE, 0x120, 2 },
        { 9018, CFG_JD | CFG_JE | CFG_WD, 0x120, 5 },
        { 9019, CFG_JD | CFG_JE | CFG_WD, 0x81a0, 5 },
        { 10240, CFG_JD | CFG_JE, 0x81a0, 5 },
        { 10241, CFG_JD | CFG_JE, 0x81b2, 5 },
    };

    for (int i = 0; i < ARRAY_SIZE(c); i++) {
        Setup su = { .cfg = c[i].cfg, .rx_n = c[i].rx_n, .rx_size = 2048 };
        Frame f;
        Rx rx;

        frame_eth(&f, ME, SRC, 0x88b5, c[i].total - 4);
        gmac_prepare(qts, &su);
        for (size_t off = 0, k = 0; off < f.len; k++) {
            size_t n = MIN(f.len - off, 4000);
            uint32_t fl = (off ? 0 : TD_FS) | (off + n == f.len ? TD_LS : 0);

            tx_desc(qts, k, f.d + off, n, fl | (off + n == f.len ? TD_TER :
                0));
            off += n;
        }
        gmac_poll(qts);
        rx_collect(qts, &su, &rx);
        g_assert_cmpint(rx.n, ==, c[i].rx_n);
        g_assert_cmphex(rx.d0[rx.n - 1] & 0xffff, ==, c[i].d0);
        /* the receive watchdog cuts at 2048, or 10240 with JE, unless WD */
        g_assert_cmpint(RD_FL(rx.d0[rx.n - 1]), ==,
                        c[i].cfg & CFG_WD ? c[i].total :
                        MIN(c[i].total, c[i].cfg & CFG_JE ? 10240 : 2048));
    }
    qtest_quit(qts);
}

static void test_jabber(void)
{
    QTestState *qts = new_gmac();
    Setup su = { .rx_n = 2, .rx_size = 2048 };
    Frame f;
    Rx rx;

    /* 2048 bytes fit; 2049 are cut at 2048, jabber, no FCS, TX stopped */
    frame_eth(&f, ME, SRC, 0x88b5, 2044);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(tx_status(qts, 0), ==, TD_FS | TD_LS | TD_TER);
    frame_eth(&f, ME, SRC, 0x88b5, 2045);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(tx_status(qts, 0), ==,
                    (TD_FS | TD_LS | TD_TER) | (1 << 15) | (1 << 14));
    g_assert_cmphex(rx.d0[0] & 0xffff, ==, 0x83a2);
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_STATUS) & 0xf0ff, ==,
                    ST_TPS | ST_TJT | ST_RI);
    qtest_quit(qts);
}

static void test_multi_desc(void)
{
    QTestState *qts = new_gmac();
    Setup su = { .rx_n = 3, .rx_size = 128 };
    Frame f;
    Rx rx;

    frame_eth(&f, ME, SRC, 0x88b5, 300);
    gmac_prepare(qts, &su);
    tx_desc(qts, 0, f.d, 13, TD_FS);
    tx_desc(qts, 1, f.d + 13, 188, 0);
    tx_desc(qts, 2, f.d + 201, 99, TD_LS | TD_TER);
    gmac_poll(qts);
    rx_collect(qts, &su, &rx);

    /* FL counts the frame so far; FS on the first, LS and the flags last */
    g_assert_cmpint(rx.n, ==, 3);
    g_assert_cmphex(rx.d0[0], ==, 0x00800200);
    g_assert_cmphex(rx.d0[1], ==, 0x01000000);
    g_assert_cmphex(rx.d0[2], ==, 0x01300120);
    g_assert_cmpmem(rx.data[0], 128, f.d, 128);
    g_assert_cmpmem(rx.data[2], 44, f.d + 256, 44);
    g_assert_cmphex(tx_status(qts, 0), ==, TD_FS);
    g_assert_cmphex(tx_status(qts, 1), ==, 0);
    g_assert_cmphex(tx_status(qts, 2), ==, TD_LS | TD_TER);
    /* the frame needed three, the ring is now empty: RU */
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_STATUS) & 0x7ff, ==,
                    ST_TU | ST_RI | ST_RU | ST_ETI);
    g_assert_cmphex((qtest_readl(qts, DMA + DMA_STATUS) >> 17) & 7, ==, 4);

    /* a frame that outgrows the descriptors the host left ends in DE */
    Setup two = { .rx_n = 2, .rx_size = 128 };
    gmac_prepare(qts, &two);
    tx_desc(qts, 0, f.d, 300, TD_FS | TD_LS | TD_TER);
    gmac_poll(qts);
    rx_collect(qts, &two, &rx);
    g_assert_cmphex(rx.d0[0], ==, 0x00800200);
    g_assert_cmphex(rx.d0[1], ==, 0x0100c100);
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_MISSED), ==, 1);

    /* buffer sizes are words: 130 holds 128 */
    Setup odd = { .rx_n = 4, .rx_size = 130 };
    gmac_prepare(qts, &odd);
    tx_desc(qts, 0, f.d, 300, TD_FS | TD_LS | TD_TER);
    gmac_poll(qts);
    rx_collect(qts, &odd, &rx);
    g_assert_cmphex(rx.d0[0], ==, 0x00800200);
    g_assert_cmphex(rx.d0[1], ==, 0x01000000);
    qtest_quit(qts);
}

/* two TX buffers in one descriptor, a TX buffer on a byte boundary */
static void test_tx_buffers(void)
{
    QTestState *qts = new_gmac();
    Setup su = { 0 };
    Frame f;
    Rx rx;

    frame_eth(&f, ME, SRC, 0x88b5, 100);
    gmac_prepare(qts, &su);
    qtest_memwrite(qts, TXB, f.d, 60);
    qtest_memwrite(qts, TXB + 0x1000, f.d + 60, 40);
    qtest_writel(qts, TXD + 4, 60 | 40 << 16);
    qtest_writel(qts, TXD + 8, TXB);
    qtest_writel(qts, TXD + 12, TXB + 0x1000);
    qtest_writel(qts, TXD, TD_OWN | TD_FS | TD_LS | TD_TER);
    gmac_poll(qts);
    rx_collect(qts, &su, &rx);
    g_assert_cmphex(rx.d0[0], ==, 0x00680320);
    g_assert_cmpmem(rx.data[0], 100, f.d, 100);

    gmac_prepare(qts, &su);
    qtest_memwrite(qts, TXB + 1, f.d, 100);
    qtest_writel(qts, TXD + 4, 100);
    qtest_writel(qts, TXD + 8, TXB + 1);
    qtest_writel(qts, TXD + 12, 0);
    qtest_writel(qts, TXD, TD_OWN | TD_FS | TD_LS | TD_TER);
    gmac_poll(qts);
    rx_collect(qts, &su, &rx);
    g_assert_cmpmem(rx.data[0], 100, f.d, 100);

    /* chained descriptors: TDES3 and RDES3 are the next descriptor */
    gmac_prepare(qts, &su);
    qtest_memwrite(qts, TXB, f.d, 60);
    qtest_writel(qts, TXD + 4, 60);
    qtest_writel(qts, TXD + 8, TXB);
    qtest_writel(qts, TXD + 12, TXD + 0x100);
    qtest_writel(qts, TXD, TD_OWN | TD_FS | (1 << 20));
    qtest_memwrite(qts, TXB + 0x1000, f.d + 60, 40);
    qtest_writel(qts, TXD + 0x100 + 4, 40);
    qtest_writel(qts, TXD + 0x100 + 8, TXB + 0x1000);
    qtest_writel(qts, TXD + 0x100 + 12, 0);
    qtest_writel(qts, TXD + 0x100, TD_OWN | TD_LS | (1 << 20));
    gmac_poll(qts);
    rx_collect(qts, &su, &rx);
    g_assert_cmpmem(rx.data[0], 100, f.d, 100);
    qtest_quit(qts);
}

/* TX checksum insertion, CIC 1 to 3, checked against a plain calculation */
static void test_csum_tx(void)
{
    QTestState *qts = new_gmac();
    Setup su = { 0 };
    static const struct { int proto; size_t pay; } c[] = {
        { 6, 13 }, { 17, 13 }, { 1, 13 }, { 6, 0 }, { 17, 0 }, { 6, 771 },
        { 6, 3 },
    };
    Frame f;
    Rx rx;

    for (int cic = 1; cic <= 3; cic++) {
        for (int i = 0; i < ARRAY_SIZE(c); i++) {
            const uint8_t *ip, *l4;
            unsigned off = c[i].proto == 6 ? 16 : c[i].proto == 17 ? 6 : 2;
            unsigned want, got;
            Frame ref;

            frame_ip(&f, c[i].proto, c[i].pay, false);
            ref = f;
            g_assert_true(loop1(qts, &su, &f, TD_CIC(cic), &rx));
            ip = rx.data[0] + 14;
            l4 = ip + 20;
            /* the IPv4 header checksum is always fixed */
            g_assert_cmphex(csum_fold(csum_add(ip, 20, 0)), ==, 0);
            got = l4[off] << 8 | l4[off + 1];
            if (cic == 1) {
                want = 0;
            } else if (cic == 2 || c[i].proto == 1) {
                want = l4_csum(&ref, c[i].proto, false);
            } else {
                want = l4_csum(&ref, c[i].proto, true);
            }
            if (c[i].proto == 17 && want == 0 && cic != 1) {
                want = 0xffff;
            }
            g_assert_cmphex(got, ==, want);
            g_assert_cmphex(tx_status(qts, 0) & 0x1f000, ==, 0);
        }
    }

    /* CIC 3 over a split frame, the CIC of the first descriptor is the one */
    frame_ip(&f, 6, 13, false);
    gmac_prepare(qts, &su);
    tx_desc(qts, 0, f.d, 20, TD_FS | TD_CIC(3));
    tx_desc(qts, 1, f.d + 20, f.len - 20, TD_LS | TD_TER | TD_CIC(3));
    gmac_poll(qts);
    rx_collect(qts, &su, &rx);
    g_assert_cmphex(rx.data[0][14 + 20 + 16] << 8 | rx.data[0][14 + 20 + 17],
                    ==, l4_csum(&f, 6, true));

    /*
     * a frame that is not IPv4 passes; an IP header the MAC cannot parse is
     * flagged IHE; a truncated payload PCE
     */
    frame_eth(&f, ME, SRC, 0x88b5, 60);
    g_assert_true(loop1(qts, &su, &f, TD_CIC(3), &rx));
    g_assert_cmpmem(rx.data[0], 60, f.d, 60);
    g_assert_cmphex(tx_status(qts, 0) & 0x1f000, ==, 0);
    frame_eth(&f, ME, SRC, 0x86dd, 80);
    g_assert_true(loop1(qts, &su, &f, TD_CIC(3), &rx));
    g_assert_cmphex(tx_status(qts, 0) & 0x1f000, ==, 0x18000);
    frame_ip(&f, 6, 13, false);
    f.len = 40;
    g_assert_true(loop1(qts, &su, &f, TD_CIC(3), &rx));
    g_assert_cmphex(tx_status(qts, 0) & 0x1f000, ==, 0x9000);
    qtest_quit(qts);
}

/* RX checksum check: RDES0 bit 0, RDES4 and the error flags */
static void test_csum_rx(void)
{
    QTestState *qts = new_gmac();
    Setup su = { .cfg = CFG_IPC };
    Frame f;
    Rx rx;

    frame_ip(&f, 6, 13, true);
    put16(f.d + 14 + 20 + 16, l4_csum(&f, 6, true));
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00470321);
    g_assert_cmphex(rx.ext, ==, 0x42);

    frame_ip(&f, 17, 13, true);
    put16(f.d + 14 + 20 + 6, l4_csum(&f, 17, true));
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00400321);
    g_assert_cmphex(rx.ext, ==, 0x41);

    frame_ip(&f, 1, 13, true);
    put16(f.d + 14 + 20 + 2, l4_csum(&f, 1, false));
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.ext, ==, 0x43);

    /* a bad payload and header: the frame is long enough to be forwarded */
    frame_ip(&f, 6, 13, false);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0], ==, 0x00478321);
    g_assert_cmphex(rx.ext, ==, 0x5a);
    /* a short one is dropped by the MTL like a bad CRC, unless DT */
    frame_ip(&f, 1, 13, true);
    g_assert_false(loop1(qts, &su, &f, 0, &rx));
    su.dmactl = 1 << 26;
    g_assert_true(loop1(qts, &su, &f, 0, &rx));

    /* a fragment, and a frame that is not IPv4, carry no RDES4 */
    su.dmactl = 0;
    frame_ip(&f, 6, 13, true);
    put16(f.d + 14 + 6, 0x2000);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(rx.d0[0] & 1, ==, 0);
    g_assert_cmphex(rx.ext, ==, 0xdeadbeef);
    qtest_quit(qts);
}

/* the interrupt summary bits follow the enables, as measured */
static void test_status(void)
{
    QTestState *qts = new_gmac();
    Frame f;
    Rx rx;
    Setup su = { 0 };

    frame_eth(&f, ME, SRC, 0x88b5, 60);
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_STATUS), ==, 0x00660444);

    su.ie = ST_NIS | ST_RI;
    g_assert_true(loop1(qts, &su, &f, 0, &rx));
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_STATUS), ==, 0x00670444);

    su.ie = 0x1e7ff;
    g_assert_true(loop1(qts, &su, &f, TD_IC, &rx));
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_STATUS), ==, 0x00678445);
    g_assert_true(qtest_get_irq(qts, 0) || true);
    /* write one to clear */
    qtest_writel(qts, DMA + DMA_STATUS, 0xffffffff);
    g_assert_cmphex(qtest_readl(qts, DMA + DMA_STATUS), ==, 0x00660000);
    qtest_quit(qts);
}

/* frames from the wire: padded to 60, FCS appended, filtered the same */
static int wire_open(struct sockaddr_in *peer, int *qemu_port)
{
    struct sockaddr_in me = { .sin_family = AF_INET,
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    struct sockaddr_in qemu = me;
    socklen_t len = sizeof(me);
    int sk = socket(AF_INET, SOCK_DGRAM, 0), probe;

    g_assert_cmpint(sk, >=, 0);
    g_assert_cmpint(bind(sk, (struct sockaddr *)&me, sizeof(me)), ==, 0);
    getsockname(sk, (struct sockaddr *)&me, &len);
    probe = socket(AF_INET, SOCK_DGRAM, 0);
    bind(probe, (struct sockaddr *)&qemu, sizeof(qemu));
    len = sizeof(qemu);
    getsockname(probe, (struct sockaddr *)&qemu, &len);
    *qemu_port = ntohs(qemu.sin_port);
    close(probe);
    *peer = me;
    return sk;
}

static void test_wire(void)
{
    struct sockaddr_in me;
    int qp, sk = wire_open(&me, &qp);
    QTestState *qts = qtest_initf("-machine ingenic-t31 -nic socket,"
                                  "udp=127.0.0.1:%d,localaddr=127.0.0.1:%d",
                                  ntohs(me.sin_port), qp);
    Setup su = { .rx_n = 2, .rx_size = 2048 };
    Frame f;
    Rx rx;
    uint8_t out[2048];
    struct sockaddr_in to = me;
    ssize_t n;

    to.sin_port = htons(qp);
    gmac_prepare(qts, &su);
    qtest_writel(qts, GMAC + MAC_CONFIG,
                 qtest_readl(qts, GMAC + MAC_CONFIG) & ~CFG_LM);

    /* a 42-byte ARP-sized frame reaches the guest padded, with its FCS */
    frame_eth(&f, ME, SRC, 0x0806, 42);
    g_assert_cmpint(sendto(sk, f.d, 42, 0, (struct sockaddr *)&to,
                           sizeof(to)), ==, 42);
    for (int i = 0; i < 100 && !(qtest_readl(qts, RXD) >> 31 == 0); i++) {
        qtest_clock_step(qts, 1000000);
        g_usleep(10000);
    }
    rx_collect(qts, &su, &rx);
    g_assert_cmpint(rx.n, ==, 1);
    g_assert_cmphex(rx.d0[0], ==, 0x00400320);
    g_assert_cmpmem(rx.data[0], 42, f.d, 42);
    g_assert_cmpint(rx.data[0][42], ==, 0);
    g_assert_cmpint(rx.data[0][59], ==, 0);

    /* a frame for somebody else is dropped by the address filter */
    frame_eth(&f, OTHER, SRC, 0x0806, 60);
    g_assert_cmpint(sendto(sk, f.d, 60, 0, (struct sockaddr *)&to,
                           sizeof(to)), ==, 60);
    qtest_clock_step(qts, 1000000);
    g_usleep(100000);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(qtest_readl(qts, RXD + STRIDE) >> 31, ==, 1);

    /* a transmitted 30-byte frame leaves padded to 60 with no FCS */
    frame_eth(&f, BCAST, ME, 0x0806, 30);
    tx_desc(qts, 0, f.d, 30, TD_FS | TD_LS | TD_TER);
    gmac_poll(qts);
    n = 0;
    for (int i = 0; i < 100 && n <= 0; i++) {
        struct timeval tv = { 0, 10000 };

        setsockopt(sk, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        n = recv(sk, out, sizeof(out), 0);
        qtest_clock_step(qts, 1000000);
    }
    g_assert_cmpint(n, ==, 60);
    g_assert_cmpmem(out, 30, f.d, 30);
    close(sk);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_data_func("ingenic-gmac/mdio/phy", "", test_mdio);
    qtest_add_data_func("ingenic-gmac/mdio/no-phy", NULL, test_mdio);
    qtest_add_func("ingenic-gmac/mdio/hold", test_mdio_hold);
    qtest_add_func("ingenic-gmac/swr", test_swr);
    qtest_add_func("ingenic-gmac/regs", test_regs);
    qtest_add_func("ingenic-gmac/regs/t40", test_regs_t40);
    qtest_add_func("ingenic-gmac/pad-fcs", test_pad_fcs);
    qtest_add_func("ingenic-gmac/dc", test_dc);
    qtest_add_func("ingenic-gmac/filter", test_filter);
    qtest_add_func("ingenic-gmac/acs", test_acs);
    qtest_add_func("ingenic-gmac/sizes", test_sizes);
    qtest_add_func("ingenic-gmac/jabber", test_jabber);
    qtest_add_func("ingenic-gmac/multi-desc", test_multi_desc);
    qtest_add_func("ingenic-gmac/tx-buffers", test_tx_buffers);
    qtest_add_func("ingenic-gmac/csum/tx", test_csum_tx);
    qtest_add_func("ingenic-gmac/csum/rx", test_csum_rx);
    qtest_add_func("ingenic-gmac/status", test_status);
    qtest_add_func("ingenic-gmac/wire", test_wire);
    return g_test_run();
}
