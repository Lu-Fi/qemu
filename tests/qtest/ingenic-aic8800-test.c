/*
 * QTest testcase for the AIC8800D80 SDIO Wi-Fi
 *
 * The card sits on a T41's SDHCI, which the test drives as a driver does:
 * the SDIO enumeration, then function 1's byte registers and its two
 * FIFOs, with the boot ROM's debug messages through them. The two parts
 * differ in the identity word the BSP reads first: the D80 as a
 * thingino driver wants it (an H variant), the D40L as a 2024 vendor BSP
 * wants it, the plain U02 whose revision byte picks the _8800d80_u02
 * firmware tables.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

#define SDHCI           0x13060000

#define SD_BLKSIZE      0x04
#define SD_BLKCNT       0x06
#define SD_ARG          0x08
#define SD_XFER         0x0c
#define SD_CMD          0x0e
#define SD_RESP         0x10
#define SD_DATA         0x20
#define SD_POWER        0x29
#define SD_CLOCK        0x2c
#define SD_INT          0x30
#define SD_INT_EN       0x34
#define SD_INT_CMD      BIT(0)
#define SD_INT_XFER     BIT(1)
#define SD_INT_RDY_RD   BIT(5)
#define SD_INT_RDY_WR   BIT(4)

#define XFER_READ       0x10
#define XFER_BLKCNT     0x02
#define XFER_MULTI      0x20

/* function 1: the bytes the BSP polls, and the FIFOs */
#define REG_INT_PENDING 0x01
#define REG_BLOCKS      0x04
#define REG_RD_FIFO     0x0f
#define REG_WR_FIFO     0x10
#define INT_AWAKE       BIT(4)
#define REG_TO_DEVICE   0x02

#define BLOCK           512
#define PKT_HDR         4
#define TYPE_CMD        0x11

#define CHIP_ID         0x40500000
#define DBG_MEM_READ_REQ        0x0400
#define DBG_MEM_WRITE_REQ       0x0402
#define DBG_START_APP_REQ       0x040d
#define MM_VERSION_REQ          0x0004
#define FW_LMAC_VERSION         0x0b040000

static uint32_t sd_cmd(QTestState *qts, unsigned cmd, uint32_t arg,
                       unsigned flags)
{
    qtest_writel(qts, SDHCI + SD_INT, 0xffffffff);
    qtest_writel(qts, SDHCI + SD_ARG, arg);
    qtest_writew(qts, SDHCI + SD_CMD, cmd << 8 | flags);
    g_assert_cmphex(qtest_readl(qts, SDHCI + SD_INT) & SD_INT_CMD, ==,
                    SD_INT_CMD);
    return qtest_readl(qts, SDHCI + SD_RESP);
}

static uint8_t sd_io_direct(QTestState *qts, bool write, unsigned fn,
                            unsigned addr, uint8_t val)
{
    return sd_cmd(qts, 52, (uint32_t)write << 31 | fn << 28 | addr << 9 | val,
                  0x02) & 0xff;
}

/* CMD53 on function 1 at a fixed address (a FIFO), in 512-byte blocks */
static void sd_fifo(QTestState *qts, bool write, unsigned addr, uint8_t *buf,
                    size_t len)
{
    uint32_t rdy = write ? SD_INT_RDY_WR : SD_INT_RDY_RD;
    uint32_t arg = (uint32_t)write << 31 | 1 << 28 | BIT(27) | addr << 9 |
                   len / BLOCK;

    g_assert_cmpuint(len % BLOCK, ==, 0);
    qtest_writew(qts, SDHCI + SD_BLKSIZE, BLOCK);
    qtest_writew(qts, SDHCI + SD_BLKCNT, len / BLOCK);
    qtest_writew(qts, SDHCI + SD_XFER, (write ? 0 : XFER_READ) | XFER_BLKCNT |
                 XFER_MULTI);
    qtest_writel(qts, SDHCI + SD_INT, 0xffffffff);
    qtest_writel(qts, SDHCI + SD_ARG, arg);
    qtest_writew(qts, SDHCI + SD_CMD, 53 << 8 | 0x22);
    for (size_t i = 0; i < len; i += 4) {
        if (!(i % BLOCK)) {
            g_assert_cmphex(qtest_readl(qts, SDHCI + SD_INT) & rdy, ==, rdy);
        }
        if (write) {
            qtest_writel(qts, SDHCI + SD_DATA, ldl_le_p(buf + i));
        } else {
            stl_le_p(buf + i, qtest_readl(qts, SDHCI + SD_DATA));
        }
    }
    g_assert_cmphex(qtest_readl(qts, SDHCI + SD_INT) & SD_INT_XFER, ==,
                    SD_INT_XFER);
}

/* Power the bus and take the card up as the BSP does; two functions */
static void sdio_enumerate(QTestState *qts)
{
    uint32_t r;

    qtest_writeb(qts, SDHCI + SD_POWER, 0x0f);
    qtest_writel(qts, SDHCI + SD_INT_EN, 0xffffffff);
    qtest_writew(qts, SDHCI + SD_CLOCK, 0x0001);
    g_assert_cmphex(qtest_readw(qts, SDHCI + SD_CLOCK) & 0x2, ==, 0x2);
    qtest_writew(qts, SDHCI + SD_CLOCK, 0x0007);
    sd_cmd(qts, 0, 0, 0);
    sd_cmd(qts, 5, 0, 0x02);
    r = sd_cmd(qts, 5, 0x200000, 0x02);
    g_assert_cmphex(r & BIT(31), ==, BIT(31));
    g_assert_cmpuint(r >> 28 & 7, ==, 2);
    r = sd_cmd(qts, 3, 0, 0x02);
    sd_cmd(qts, 7, r & 0xffff0000, 0x03);
    sd_io_direct(qts, true, 0, 0x02, 0x02);
    sd_io_direct(qts, true, 0, 0x110, 0x00);
    sd_io_direct(qts, true, 0, 0x111, 0x02);
}

/* A request: the word of length and type, a spare one, then the message */
static void msg_push(QTestState *qts, uint16_t id, const uint32_t *p,
                     size_t words)
{
    uint8_t buf[BLOCK] = {};

    stw_le_p(buf, 12 + words * 4);
    buf[2] = TYPE_CMD;
    stw_le_p(buf + 8, id);
    stw_le_p(buf + 10, id >> 10);
    stw_le_p(buf + 14, words * 4);
    for (size_t i = 0; i < words; i++) {
        stl_le_p(buf + 16 + i * 4, p[i]);
    }
    sd_fifo(qts, true, REG_WR_FIFO, buf, sizeof(buf));
}

/* The next message from the chip: its id, the parameters in @p */
static bool msg_pop(QTestState *qts, uint16_t *id, uint8_t *p, size_t len)
{
    uint8_t buf[BLOCK];

    if (!sd_io_direct(qts, false, 1, REG_BLOCKS, 0)) {
        return false;
    }
    sd_fifo(qts, false, REG_RD_FIFO, buf, sizeof(buf));
    g_assert_cmphex(buf[2], ==, TYPE_CMD);
    *id = lduw_le_p(buf + 4);
    memcpy(p, buf + 16, len);
    return true;
}

static uint32_t chip_read(QTestState *qts, uint32_t addr)
{
    uint8_t p[8];
    uint16_t id;

    msg_push(qts, DBG_MEM_READ_REQ, &addr, 1);
    g_assert_true(msg_pop(qts, &id, p, sizeof(p)));
    g_assert_cmphex(id, ==, DBG_MEM_READ_REQ + 1);
    g_assert_cmphex(ldl_le_p(p), ==, addr);
    return ldl_le_p(p + 4);
}

static QTestState *card(const char *dev)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 "
                                  "-device %s,bus=msc0", dev);

    sdio_enumerate(qts);
    /* the chip wakes when told to, and says so */
    sd_io_direct(qts, true, 1, REG_TO_DEVICE, 1);
    g_assert_cmphex(sd_io_direct(qts, false, 1, REG_INT_PENDING, 0) & INT_AWAKE,
                    ==, INT_AWAKE);
    return qts;
}

/*
 * The BSP takes the revision from bits 16 to 23 and, in a 2024 build, only
 * 3 or 7 as a U02 (the _8800d80_u02 tables); a thingino build also reads
 * the H variant's bits 6 and 7 from it
 */
static void test_ids(const void *data)
{
    QTestState *qts = card("aic8800d80-sdio");
    uint32_t id = chip_read(qts, CHIP_ID);

    g_assert_cmphex(id, ==, 0x02c30000);
    g_assert_cmphex((id >> 16 & 0x3f), ==, 3);
    g_assert_cmphex((id >> 16 & 0xc0), ==, 0xc0);
    qtest_quit(qts);
}

static void test_ids_d40l(const void *data)
{
    QTestState *qts = card("aic8800d40l-sdio");
    uint32_t id = chip_read(qts, CHIP_ID);

    g_assert_cmphex(id, ==, 0x02030000);
    g_assert_cmphex((id >> 16 & 0xff) & 0xfb, ==, 3);
    qtest_quit(qts);
}

/* The handshake: the boot ROM takes memory writes, the firmware starts */
static void test_firmware(const void *data)
{
    QTestState *qts = card(data);
    uint32_t w[2] = { 0x00120180, 0xdeadbeef };
    uint32_t start[2] = { 0x00120000, 1 };
    uint8_t p[32];
    uint16_t id;

    msg_push(qts, DBG_MEM_WRITE_REQ, w, 2);
    g_assert_true(msg_pop(qts, &id, p, 4));
    g_assert_cmphex(id, ==, DBG_MEM_WRITE_REQ + 1);
    g_assert_cmphex(chip_read(qts, w[0]), ==, 0xdeadbeef);

    msg_push(qts, DBG_START_APP_REQ, start, 2);
    g_assert_true(msg_pop(qts, &id, p, 4));
    g_assert_cmphex(id, ==, DBG_START_APP_REQ + 1);

    msg_push(qts, MM_VERSION_REQ, NULL, 0);
    g_assert_true(msg_pop(qts, &id, p, sizeof(p)));
    g_assert_cmphex(id, ==, MM_VERSION_REQ + 1);
    g_assert_cmphex(ldl_le_p(p), ==, FW_LMAC_VERSION);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    if (!qtest_has_machine("ingenic-t41") ||
        !qtest_has_device("aic8800d80-sdio") ||
        !qtest_has_device("aic8800d40l-sdio")) {
        return g_test_run();
    }
    qtest_add_data_func("aic8800/d80/ids", NULL, test_ids);
    qtest_add_data_func("aic8800/d40l/ids", NULL, test_ids_d40l);
    qtest_add_data_func("aic8800/d80/firmware", "aic8800d80-sdio",
                        test_firmware);
    qtest_add_data_func("aic8800/d40l/firmware", "aic8800d40l-sdio",
                        test_firmware);
    return g_test_run();
}
