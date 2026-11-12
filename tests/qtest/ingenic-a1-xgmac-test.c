/*
 * QTest testcases for the Ingenic A1 XGMAC's DMA
 *
 * Receive: a frame that arrives while the driver has the receive DMA
 * stopped, as stmmac does while it resets the adapter and the ring may be
 * freed, must not be written to the ring; it is delivered once the DMA is
 * started.
 *
 * Transmit: stmmac stops the DMA and frees the ring while it resets the
 * adapter, and the ring it allocates next lives elsewhere. Neither a
 * kick of the tail pointer with the DMA stopped nor one after a new
 * ring was programmed may touch the old ring, which holds other data by
 * then.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/sockets.h"

#define GMAC            0x130b0000
#define RX_CONFIG       (GMAC + 0x0004)
#define RX_CONTROL      (GMAC + 0x3108)
#define RXDESC_LADDR    (GMAC + 0x311c)
#define RXDESC_RING     (GMAC + 0x3134)
#define TX_CONTROL      (GMAC + 0x3104)
#define TXDESC_LADDR    (GMAC + 0x3114)
#define TXDESC_TAIL     (GMAC + 0x3124)
#define TXDESC_RING     (GMAC + 0x3130)
#define CH_STATUS       (GMAC + 0x3160)
#define RAM             0x01000000
#define RING            (RAM + 0x1000)
#define BUF             (RAM + 0x2000)

static void test_rx_gated(void)
{
    struct sockaddr_in me = { .sin_family = AF_INET,
                              .sin_addr.s_addr = htonl(INADDR_LOOPBACK) };
    struct sockaddr_in qemu = me;
    socklen_t len = sizeof(me);
    uint8_t frame[64] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x52, 0x54,
                          0x00, 0x12, 0x34, 0x56, 0x08, 0x00, 0xaa, 0x55 };
    QTestState *qts;
    int sk = socket(AF_INET, SOCK_DGRAM, 0);
    int qp;
    uint32_t des3;

    g_assert_cmpint(sk, >=, 0);
    g_assert_cmpint(bind(sk, (struct sockaddr *)&me, sizeof(me)), ==, 0);
    getsockname(sk, (struct sockaddr *)&me, &len);
    {
        int probe = socket(AF_INET, SOCK_DGRAM, 0);

        bind(probe, (struct sockaddr *)&qemu, sizeof(qemu));
        len = sizeof(qemu);
        getsockname(probe, (struct sockaddr *)&qemu, &len);
        qp = ntohs(qemu.sin_port);
        close(probe);
    }
    qts = qtest_initf("-machine ingenic-a1 -nic socket,udp=127.0.0.1:%d,"
                      "localaddr=127.0.0.1:%d", ntohs(me.sin_port), qp);

    /* one owned descriptor: buffer address, buffer 2 address, length, OWN */
    qtest_writel(qts, RING, BUF);
    qtest_writel(qts, RING + 4, 0);
    qtest_writel(qts, RING + 8, 0);
    qtest_writel(qts, RING + 12, 0x80000000);
    qtest_writel(qts, BUF, 0);
    qtest_writel(qts, RXDESC_LADDR, RING);
    qtest_writel(qts, RXDESC_RING, 0);

    me.sin_port = htons(qp);
    g_assert_cmpint(sendto(sk, frame, sizeof(frame), 0,
                           (struct sockaddr *)&me, sizeof(me)), ==,
                    sizeof(frame));
    qtest_clock_step(qts, 1000000);
    qtest_readl(qts, CH_STATUS);

    g_assert_cmphex(qtest_readl(qts, BUF), ==, 0);
    g_assert_cmphex(qtest_readl(qts, RING + 12), ==, 0x80000000);

    qtest_writel(qts, RX_CONTROL, 1);
    qtest_writel(qts, RX_CONFIG, 1);
    for (int i = 0; i < 50; i++) {
        qtest_clock_step(qts, 1000000);
        des3 = qtest_readl(qts, RING + 12);
        if (!(des3 & 0x80000000)) {
            break;
        }
        g_usleep(10000);
    }
    g_assert_cmphex(des3 & 0x80000000, ==, 0);
    g_assert_cmphex(qtest_readl(qts, BUF + 12) >> 16, ==, 0x55aa);

    close(sk);
    qtest_quit(qts);
}

/* A kernel pointer where a descriptor's OWN bit would be: bit 31 is set */
#define PTR             0x8aff2000

static void tx_ring(QTestState *qts, uint32_t ring, uint32_t last)
{
    /* two descriptors, the last word of each OWN (or a pointer) */
    qtest_writel(qts, ring, BUF);
    qtest_writel(qts, ring + 4, 0);
    qtest_writel(qts, ring + 8, 0);
    qtest_writel(qts, ring + 12, 0x80000000 | (1u << 29) | (1u << 28));
    qtest_writel(qts, ring + 16, BUF);
    qtest_writel(qts, ring + 20, 0);
    qtest_writel(qts, ring + 24, 0);
    qtest_writel(qts, ring + 28, last);
    qtest_writel(qts, TXDESC_LADDR, ring);
    qtest_writel(qts, TXDESC_RING, 1);
}

static void test_tx_stopped(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1 -nic none");

    tx_ring(qts, RING, PTR);
    qtest_writel(qts, TXDESC_TAIL, RING + 32);
    g_assert_cmphex(qtest_readl(qts, RING + 12), ==,
                    0x80000000 | (1u << 29) | (1u << 28));

    qtest_writel(qts, TX_CONTROL, 1);
    qtest_writel(qts, TXDESC_TAIL, RING + 16);
    g_assert_cmphex(qtest_readl(qts, RING + 12) & 0x80000000, ==, 0);
    g_assert_cmphex(qtest_readl(qts, RING + 28), ==, PTR);
    qtest_quit(qts);
}

static void test_tx_new_ring(void)
{
    QTestState *qts = qtest_init("-machine ingenic-a1 -nic none");
    uint32_t ring2 = RING + 0x1000;

    tx_ring(qts, RING, 0);
    qtest_writel(qts, TX_CONTROL, 1);
    qtest_writel(qts, TXDESC_TAIL, RING + 16);
    g_assert_cmphex(qtest_readl(qts, RING + 12) & 0x80000000, ==, 0);

    /* the old ring is freed and reused: its next slot holds a pointer */
    qtest_writel(qts, TX_CONTROL, 0);
    qtest_writel(qts, RING + 28, PTR);
    tx_ring(qts, ring2, 0);
    qtest_writel(qts, TX_CONTROL, 1);
    qtest_writel(qts, TXDESC_TAIL, ring2 + 16);

    g_assert_cmphex(qtest_readl(qts, RING + 28), ==, PTR);
    g_assert_cmphex(qtest_readl(qts, ring2 + 12) & 0x80000000, ==, 0);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-a1-xgmac/rx-gated", test_rx_gated);
    qtest_add_func("ingenic-a1-xgmac/tx-stopped", test_tx_stopped);
    qtest_add_func("ingenic-a1-xgmac/tx-new-ring", test_tx_new_ring);
    return g_test_run();
}
