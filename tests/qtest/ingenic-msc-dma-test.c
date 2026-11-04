/*
 * QTest testcase for the Ingenic MSC's data FIFO and common-DMA requests
 *
 * As a T31X with an SD card reads them: a read fills the FIFO to 127
 * words (RTCNT, STAT.DATA_FIFO_AFULL); with DMAC.DMAEN and DMAC.DMASEL
 * set, MSC0 requests a PDMA channel on 0x1b for a read and 0x1a for a
 * write, either bit alone requests nothing, and IFLG.RXFIFO_RD_REQ reads
 * 0. A channel moves the card's blocks to RAM and back.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bswap.h"

#define MSC0            0x13450000
#define CTRL            0x00
#define STAT            0x04
#define CMDAT           0x0c
#define BLKLEN          0x18
#define NOB             0x1c
#define IFLG            0x28
#define CMD             0x2c
#define ARG             0x30
#define RES             0x34
#define RXFIFO          0x38
#define TXFIFO          0x3c
#define DMAC            0x44
#define RTCNT           0x5c

#define STAT_AFULL      (1u << 10)
#define IFLG_DMA_DONE   (1u << 31)
#define IFLG_DMAEND     (1u << 16)
#define IFLG_FIFO_EMP   (1u << 13)
#define IFLG_RD_REQ     (1u << 5)
#define IFLG_TRAN_DONE  (1u << 0)

/* CMDAT: response type, data, write, 4-bit bus */
#define R1              0x401
#define R2              0x402
#define R3              0x403
#define R1_READ         0x409
#define R1_WRITE        0x419
#define RTRG(n)         ((n) << 14)

#define PDMA            0x13420000
#define CH(r)           (PDMA + 31 * 0x20 + (r))
#define DSA             0x00
#define DTA             0x04
#define DTC             0x08
#define DRT             0x0c
#define DCS             0x10
#define DCM             0x14
#define PDMA_DMAC       (PDMA + 0x1000)
#define DCS_GO          0x80000001      /* no descriptor, enabled */
#define DCM_SAI_W       0x00800000      /* source increments, words */
#define DCM_DAI_W       0x00400000      /* target increments, words */

#define REQ_TX          0x1a
#define REQ_RX          0x1b

#define BUF             0x00200000
#define BLOCKS          8

static char *img_path;

static uint32_t pattern(uint32_t i)
{
    return 0x5a000000 | (i * 0x01010101u >> 8);
}

static uint32_t cmd(QTestState *qts, uint32_t index, uint32_t arg,
                    uint32_t cmdat)
{
    uint32_t a, b, c;

    qtest_writel(qts, MSC0 + IFLG, ~0u);
    qtest_writel(qts, MSC0 + CMD, index);
    qtest_writel(qts, MSC0 + ARG, arg);
    qtest_writel(qts, MSC0 + CMDAT, cmdat);
    qtest_writel(qts, MSC0 + CTRL, 4);
    a = qtest_readl(qts, MSC0 + RES) & 0xffff;
    b = qtest_readl(qts, MSC0 + RES) & 0xffff;
    c = qtest_readl(qts, MSC0 + RES) & 0xffff;
    return (a << 24) | (b << 8) | (c & 0xff);
}

/* A card in transfer state, 512-byte blocks */
static QTestState *card_init(void)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 "
                                  "-global ingenic-t31.soc-variant=t31x "
                                  "-drive file=%s,if=sd,format=raw",
                                  img_path);
    uint32_t rca;

    qtest_writel(qts, PDMA_DMAC, 1);
    cmd(qts, 0, 0, 0x400);
    cmd(qts, 8, 0x1aa, R1);
    for (int i = 0; i < 100; i++) {
        cmd(qts, 55, 0, R1);
        if (cmd(qts, 41, 0x40ff8000, R3) & 0x80000000) {
            break;
        }
        qtest_clock_step(qts, 10 * 1000 * 1000);
    }
    cmd(qts, 2, 0, R2);
    rca = cmd(qts, 3, 0, R1) & 0xffff0000;
    cmd(qts, 7, rca, R1);
    cmd(qts, 16, 512, R1);
    qtest_writel(qts, MSC0 + BLKLEN, 512);
    return qts;
}

static void ch_start(QTestState *qts, uint32_t dsa, uint32_t dta,
                     uint32_t dtc, uint32_t drt, uint32_t dcm)
{
    qtest_writel(qts, CH(DCS), 0);
    qtest_writel(qts, CH(DSA), dsa);
    qtest_writel(qts, CH(DTA), dta);
    qtest_writel(qts, CH(DTC), dtc);
    qtest_writel(qts, CH(DRT), drt);
    qtest_writel(qts, CH(DCM), dcm);
    qtest_writel(qts, CH(DCS), DCS_GO);
}

static void read_blocks(QTestState *qts, uint32_t nob, uint32_t dmac,
                        uint32_t cmdat)
{
    qtest_writel(qts, MSC0 + NOB, nob);
    qtest_writel(qts, MSC0 + DMAC, dmac);
    cmd(qts, nob > 1 ? 18 : 17, 0, cmdat);
}

/* Without the common DMA, a read waits in the FIFO for the CPU */
static void test_fifo(void)
{
    QTestState *qts = card_init();

    read_blocks(qts, 2, 0, R1_READ);
    g_assert_cmpuint(qtest_readl(qts, MSC0 + RTCNT), ==, 127);
    g_assert_cmphex(qtest_readl(qts, MSC0 + STAT) & STAT_AFULL, ==,
                    STAT_AFULL);
    g_assert_cmphex(qtest_readl(qts, MSC0 + IFLG) &
                    (IFLG_RD_REQ | IFLG_FIFO_EMP), ==, IFLG_RD_REQ);
    for (int i = 0; i < 256; i++) {
        g_assert_cmphex(qtest_readl(qts, MSC0 + RXFIFO), ==, pattern(i));
        g_assert_cmpuint(qtest_readl(qts, MSC0 + RTCNT), ==,
                         MIN(127, 255 - i));
    }
    g_assert_cmphex(qtest_readl(qts, MSC0 + STAT) & STAT_AFULL, ==, 0);
    g_assert_cmphex(qtest_readl(qts, MSC0 + IFLG) &
                    (IFLG_RD_REQ | IFLG_FIFO_EMP | IFLG_TRAN_DONE), ==,
                    IFLG_FIFO_EMP | IFLG_TRAN_DONE);
    qtest_quit(qts);
}

/* Either DMAC bit alone raises no request */
static void test_no_request(void)
{
    static const uint32_t dmac[] = { 0, 2 };
    QTestState *qts = card_init();

    for (int i = 0; i < ARRAY_SIZE(dmac); i++) {
        ch_start(qts, MSC0 + RXFIFO, BUF, 64, REQ_RX, DCM_DAI_W);
        read_blocks(qts, 1, dmac[i], R1_READ);
        g_assert_cmphex(qtest_readl(qts, CH(DTC)), ==, 64);
        g_assert_cmpuint(qtest_readl(qts, MSC0 + RTCNT), ==, 127);
        for (int j = 0; j < 128; j++) {
            (void)qtest_readl(qts, MSC0 + RXFIFO);
        }
    }
    qtest_quit(qts);
}

/* A channel on 0x1b reads the blocks; one shorter than the read stops */
static void test_rx(void)
{
    QTestState *qts = card_init();
    uint32_t iflg;

    ch_start(qts, MSC0 + RXFIFO, BUF, BLOCKS * 128, REQ_RX, DCM_DAI_W);
    read_blocks(qts, BLOCKS, 3, R1_READ | RTRG(1));
    g_assert_cmphex(qtest_readl(qts, CH(DTC)), ==, 0);
    for (int i = 0; i < BLOCKS * 128; i++) {
        g_assert_cmphex(qtest_readl(qts, BUF + 4 * i), ==, pattern(i));
    }
    iflg = qtest_readl(qts, MSC0 + IFLG);
    g_assert_cmphex(iflg & IFLG_TRAN_DONE, ==, IFLG_TRAN_DONE);
    g_assert_cmphex(iflg & (IFLG_DMA_DONE | IFLG_DMAEND), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, MSC0 + RTCNT), ==, 0);
    cmd(qts, 12, 0, R1);

    ch_start(qts, MSC0 + RXFIFO, BUF, 100, REQ_RX, DCM_DAI_W);
    read_blocks(qts, 1, 3, R1_READ | RTRG(3));
    g_assert_cmphex(qtest_readl(qts, CH(DTC)), ==, 0);
    g_assert_cmpuint(qtest_readl(qts, MSC0 + RTCNT), ==, 28);
    g_assert_cmphex(qtest_readl(qts, MSC0 + IFLG) & IFLG_RD_REQ, ==, 0);
    g_assert_cmphex(qtest_readl(qts, MSC0 + RXFIFO), ==, pattern(100));
    qtest_quit(qts);
}

/* A channel on 0x1a writes a block from RAM */
static void test_tx(void)
{
    QTestState *qts = card_init();

    for (int i = 0; i < 128; i++) {
        qtest_writel(qts, BUF + 4 * i, 0xc0de0000 + i);
    }
    ch_start(qts, BUF, MSC0 + TXFIFO, 128, REQ_TX, DCM_SAI_W);
    qtest_writel(qts, MSC0 + NOB, 1);
    qtest_writel(qts, MSC0 + DMAC, 3);
    cmd(qts, 24, 3 * 512, R1_WRITE);
    g_assert_cmphex(qtest_readl(qts, CH(DTC)), ==, 0);
    g_assert_cmphex(qtest_readl(qts, MSC0 + IFLG) & IFLG_TRAN_DONE, ==,
                    IFLG_TRAN_DONE);

    qtest_writel(qts, MSC0 + DMAC, 0);
    cmd(qts, 17, 3 * 512, R1_READ);
    for (int i = 0; i < 128; i++) {
        g_assert_cmphex(qtest_readl(qts, MSC0 + RXFIFO), ==, 0xc0de0000 + i);
    }
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    int fd, ret;
    g_autofree uint32_t *img = g_new(uint32_t, 1024 * 1024);

    g_test_init(&argc, &argv, NULL);

    for (int i = 0; i < 1024 * 1024; i++) {
        img[i] = cpu_to_le32(pattern(i));
    }
    fd = g_file_open_tmp("ingenic-msc-XXXXXX", &img_path, NULL);
    g_assert(fd >= 0);
    g_assert(write(fd, img, 4 * 1024 * 1024) == 4 * 1024 * 1024);
    close(fd);

    qtest_add_func("ingenic-msc-dma/fifo", test_fifo);
    qtest_add_func("ingenic-msc-dma/no-request", test_no_request);
    qtest_add_func("ingenic-msc-dma/rx", test_rx);
    qtest_add_func("ingenic-msc-dma/tx", test_tx);
    ret = g_test_run();
    unlink(img_path);
    g_free(img_path);
    return ret;
}
