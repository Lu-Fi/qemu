/*
 * QTest testcase for saving and loading a machine with a Wi-Fi card on
 * its SDIO bus
 *
 * Each SDIO radio is enumerated on a T41's SDHCI and the machine saved to
 * a file; a second QEMU loads it. The card must still be selected, with
 * its function enabled and its block size set, and answer CMD52 as before.
 * The families' own state is checked by their tests (ingenic-atbm-test,
 * ingenic-aic8800-test) and by the guest suite.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qobject/qdict.h"

#define SDHCI           0x13060000

/* the standard host controller's registers */
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

/* Power the bus and take the card up as a driver does */
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
    r = sd_cmd(qts, 3, 0, 0x02);
    sd_cmd(qts, 7, r & 0xffff0000, 0x03);
    /* vendor 0x007a, from the CIS the driver reads; enable function 1 */
    sd_io_direct(qts, true, 0, 0x02, 0x02);
    g_assert_cmphex(sd_io_direct(qts, false, 0, 0x03, 0), ==, 0x02);
    sd_io_direct(qts, true, 0, 0x04, 0x02);
    /* function 1 moves blocks of 512 bytes: FBR 0x110 */
    sd_io_direct(qts, true, 0, 0x110, 0x00);
    sd_io_direct(qts, true, 0, 0x111, 0x02);
}

static void wait_status(QTestState *qts, const char *key, const char *busy)
{
    for (int i = 0; i < 6000; i++) {
        QDict *rsp = qtest_qmp(qts, "{ 'execute': %s }", key);
        QDict *ret = qdict_get_qdict(rsp, "return");
        const char *st = qdict_get_try_str(ret, "status");
        bool done = st && strcmp(st, busy) && strcmp(st, "setup") &&
                    strcmp(st, "device");

        if (done) {
            g_assert_cmpstr(st, !=, "failed");
            qobject_unref(rsp);
            return;
        }
        qobject_unref(rsp);
        g_usleep(10000);
    }
    g_assert_not_reached();
}


static void test_migrate(const void *data)
{
    g_autofree char *cmd = g_strdup_printf(
        "-machine ingenic-t41 -device %s,bus=msc0,ssid=lab,psk=passphrase1",
        (const char *)data);
    g_autofree char *path = g_strdup_printf("%s/wifi-migrate-%d.state",
                                            g_get_tmp_dir(), getpid());
    g_autofree char *uri = g_strdup_printf("file:%s", path);
    QTestState *src, *dst;

    src = qtest_init(cmd);
    sdio_enumerate(src);
    g_assert_cmphex(sd_io_direct(src, false, 0, 0x03, 0), ==, 0x02);

    qtest_qmp_assert_success(src, "{ 'execute': 'stop' }");
    qtest_qmp_assert_success(src, "{ 'execute': 'migrate', 'arguments': "
                             "{ 'uri': %s } }", uri);
    wait_status(src, "query-migrate", "active");

    dst = qtest_initf("%s -incoming %s", cmd, uri);
    wait_status(dst, "query-status", "inmigrate");
    g_assert_cmphex(sd_io_direct(dst, false, 0, 0x03, 0), ==, 0x02);
    g_assert_cmphex(sd_io_direct(dst, false, 0, 0x110, 0), ==, 0x00);
    g_assert_cmphex(sd_io_direct(dst, false, 0, 0x111, 0), ==, 0x02);

    qtest_quit(dst);
    qtest_quit(src);
    unlink(path);
}

int main(int argc, char **argv)
{
    static const char *const cards[] = {
        "atbm6031", "atbm6062s", "atbm6162s", "aic8800d80-sdio",
        "aic8800d40l-sdio", "ssv6158", "rtl8189fs", "rtl8189es",
        "rtl8192fs", "bcm43438", "hi3881",
    };

    g_test_init(&argc, &argv, NULL);
    if (!qtest_has_machine("ingenic-t41")) {
        return g_test_run();
    }
    for (int i = 0; i < ARRAY_SIZE(cards); i++) {
        g_autofree char *name = g_strdup_printf("wifi/migrate/%s", cards[i]);

        if (qtest_has_device(cards[i])) {
            qtest_add_data_func(name, cards[i], test_migrate);
        }
    }
    return g_test_run();
}
