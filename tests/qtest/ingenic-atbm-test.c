/*
 * QTest testcase for the AltoBeam ATBM6041 and ATBM6441 SDIO Wi-Fi
 *
 * The card sits on a T41's SDHCI, which the test drives as a driver does:
 * the SDIO enumeration, then function 1's host interface registers, with
 * the chip's memory behind them and a queue of WSM messages each way. The
 * Hera parts' own (IoT) firmware starts at power-on, behind four messages
 * their drivers take for debris, and answers in its own dialect: a join
 * with an SSID and a key, a scan, the keep-alive check. The 6441 adds its
 * MCU: the UART frames, the PIR line, and the line that wakes the SoC for
 * the reasons the host asked to hear of. The 6461 is the 6441's successor on
 * a T23: its driver sends the Wi-Fi 6 header and talks to the RTOS beside
 * the Wi-Fi firmware through a general command.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <glib/gstdio.h>
#include <sys/un.h>
#include "libqtest.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

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

/* function 1's registers, at four bytes each */
#define HIF_CONFIG      0x00
#define HIF_CONTROL     0x04
#define HIF_QUEUE       0x08
#define HIF_AHB_DPORT   0x0c
#define HIF_SRAM_BASE   0x10
#define HIF_SRAM_DPORT  0x14
#define CONFIG_CPU_RESET BIT(14)
#define CONFIG_IRQ      (BIT(16) | BIT(17))

#define CHIP_VERSION    0x0acc017c
#define XMITED          0x09000100

#define WSM_STARTUP     0x0801
#define WSM_EVENT       0x0805

#define TX_ALIGN        256

/* the header of a message from the host: 8 bytes, 32 for the 6461's driver */
static size_t tx_hdr = 8;

#define RTOS_MAGIC      0xacaccacaU
#define WSM_CUSTOMER    0x0038
#define WSM_CUSTOMER_SZ 532

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

/*
 * CMD53 through the buffer data port, a word at a time: in byte mode up to
 * 512 bytes, in 512-byte blocks for more
 */
static void sd_io_extended(QTestState *qts, bool write, unsigned addr,
                           uint8_t *buf, size_t len)
{
    bool blocks = len > 512;
    unsigned count = blocks ? len / 512 : len & 0x1ff;
    uint32_t rdy = write ? SD_INT_RDY_WR : SD_INT_RDY_RD;
    uint32_t arg = (uint32_t)write << 31 | 1 << 28 | (uint32_t)blocks << 27 |
                   BIT(26) | addr << 9 | count;

    g_assert_true(!blocks || len % 512 == 0);
    qtest_writew(qts, SDHCI + SD_BLKSIZE, blocks ? 512 : len);
    qtest_writew(qts, SDHCI + SD_BLKCNT, blocks ? count : 1);
    qtest_writew(qts, SDHCI + SD_XFER, (write ? 0 : XFER_READ) |
                 (blocks ? XFER_BLKCNT | XFER_MULTI : 0));
    qtest_writel(qts, SDHCI + SD_INT, 0xffffffff);
    qtest_writel(qts, SDHCI + SD_ARG, arg);
    qtest_writew(qts, SDHCI + SD_CMD, 53 << 8 | 0x22);
    for (size_t i = 0; i < len; i += 4) {
        if (!(i % 512)) {
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

static uint32_t hif_read(QTestState *qts, unsigned reg)
{
    uint8_t b[4];

    sd_io_extended(qts, false, reg, b, 4);
    return ldl_le_p(b);
}

static void hif_write(QTestState *qts, unsigned reg, uint32_t val)
{
    uint8_t b[4];

    stl_le_p(b, val);
    sd_io_extended(qts, true, reg, b, 4);
}

static uint32_t chip_read(QTestState *qts, uint32_t addr)
{
    hif_write(qts, HIF_SRAM_BASE, addr);
    return hif_read(qts, HIF_AHB_DPORT);
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

/* Hold the CPU and let go of it as the 60xx drivers do, or not at all */
static void irq_enable(QTestState *qts)
{
    hif_write(qts, HIF_CONFIG, CONFIG_IRQ);
}

typedef struct Msg {
    uint16_t id;
    uint8_t body[2048];
    size_t len;
} Msg;

/* The next message from the chip, its length from CONTROL */
static bool msg_pop(QTestState *qts, Msg *m)
{
    uint32_t ctl = hif_read(qts, HIF_CONTROL);
    unsigned words = (ctl & 0xfff) | (ctl >> 14 & 3) << 12;
    uint8_t buf[2048];
    size_t len;

    if (!words) {
        return false;
    }
    len = ROUND_UP(words * 2 + 2, TX_ALIGN);
    sd_io_extended(qts, false, HIF_QUEUE, buf, len > 512 ? ROUND_UP(len, 512)
                                                         : len);
    m->len = lduw_le_p(buf) - 4;
    m->id = lduw_le_p(buf + 2) & 0x1fff;
    memcpy(m->body, buf + 4, MIN(m->len, sizeof(m->body)));
    return true;
}

/* A message for the chip: a header ending in the length and id, the body */
static void msg_push(QTestState *qts, uint16_t id, const void *body,
                     size_t len)
{
    uint8_t buf[2048] = {};
    size_t total = ROUND_UP(tx_hdr + len, TX_ALIGN);

    g_assert_cmpuint(total, <=, sizeof(buf));
    stw_le_p(buf + tx_hdr - 4, tx_hdr + len);
    stw_le_p(buf + tx_hdr - 2, id);
    memcpy(buf + tx_hdr, body, len);
    sd_io_extended(qts, true, HIF_QUEUE, buf, total > 512 ? ROUND_UP(total, 512)
                                                          : total);
}

/* Take the debris and the startup indication the IoT firmware begins with */
static void iot_startup(QTestState *qts, Msg *m)
{
    qtest_clock_step(qts, 500 * 1000000LL);
    for (int i = 0; i < 4; i++) {
        g_assert_true(msg_pop(qts, m));
        g_assert_cmphex(m->id, ==, 0);
    }
    g_assert_true(msg_pop(qts, m));
    g_assert_cmphex(m->id, ==, WSM_STARTUP);
    /* the firmware's version tells the driver which sleep it has */
    g_assert_cmpuint(lduw_le_p(m->body + 18), ==, 14000);
    g_assert_false(msg_pop(qts, m));
}

static void request(QTestState *qts, Msg *m, uint16_t id, const void *body,
                    size_t len)
{
    msg_push(qts, id, body, len);
    g_assert_true(msg_pop(qts, m));
    g_assert_cmphex(m->id, ==, id | 0x400);
}

static void test_ids(const void *data)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 "
                                  "-device %s,bus=msc0", (const char *)data);

    sdio_enumerate(qts);
    g_assert_cmphex(chip_read(qts, CHIP_VERSION) & 0xff, ==, 0x64);
    qtest_quit(qts);
}

/* The 6461: the RHEA family's chip version, assumed from the open driver */
static void test_ids_6461(const void *data)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-device atbm6461,bus=msc0");

    sdio_enumerate(qts);
    g_assert_cmphex(chip_read(qts, CHIP_VERSION) & 0xff, ==, 0xb1);
    qtest_quit(qts);
}

/*
 * The 6041 with its 60xx driver holds the CPU, downloads firmware and lets
 * go: no debris, the other firmware's version, its dialect
 */
/* The 6461 has the IoT firmware of the Hera parts behind the wider header */
static void test_6461_startup(const void *data)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-device atbm6461,bus=msc0");
    uint8_t cfg[2];
    Msg m;

    tx_hdr = 32;
    sdio_enumerate(qts);
    irq_enable(qts);
    iot_startup(qts, &m);
    stw_le_p(cfg, 167);
    request(qts, &m, 0x0002, cfg, 2);
    g_assert_cmpuint(m.len, ==, 167);
    tx_hdr = 8;
    qtest_quit(qts);
}

/* A command to the RTOS: its id and payload in the 532 bytes of a request */
static void rtos(QTestState *qts, Msg *m, uint32_t id, const void *payload,
                 size_t len)
{
    uint8_t req[WSM_CUSTOMER_SZ] = {};
    uint32_t crc = ~0;

    stl_le_p(req, RTOS_MAGIC);
    stl_le_p(req + 4, id);
    memcpy(req + 16, payload, len);
    /* the zlib CRC, without its last inversion */
    for (size_t i = 0; i < len; i++) {
        crc ^= req[16 + i];
        for (int b = 0; b < 8; b++) {
            crc = crc & 1 ? crc >> 1 ^ 0xedb88320 : crc >> 1;
        }
    }
    stl_le_p(req + 8, crc);
    stl_le_p(req + 12, len);
    request(qts, m, WSM_CUSTOMER, req, sizeof(req));
    g_assert_cmpuint(m->len, ==, WSM_CUSTOMER_SZ);
    g_assert_cmphex((uint32_t)ldl_le_p(m->body), ==, RTOS_MAGIC);
    g_assert_cmpuint(ldl_le_p(m->body + 4), ==, id);
    /* the return code */
    g_assert_cmpuint(ldl_le_p(m->body + 12), ==, 0);
}

static void test_6461_rtos(const void *data)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-device atbm6461,bus=msc0,battery=87,"
                                 "battery-mv=3950,charging=true");
    uint8_t v[4];
    Msg m;

    tx_hdr = 32;
    sdio_enumerate(qts);
    irq_enable(qts);
    iot_startup(qts, &m);

    /* the count the driver reads starts at the buffers, then follows it */
    g_assert_cmpuint(chip_read(qts, XMITED), ==, 16);
    rtos(qts, &m, 0x24, NULL, 0);
    g_assert_cmpuint(chip_read(qts, XMITED), ==, 17);
    g_assert_cmpuint(ldl_le_p(m.body + 16), ==, 5);
    g_assert_cmpstr((char *)m.body + 20, ==, "qemu");
    rtos(qts, &m, 0x1a, NULL, 0);
    g_assert_cmpuint(ldl_le_p(m.body + 16), ==, 12);
    g_assert_cmpuint(ldl_le_p(m.body + 20), ==, 87);
    g_assert_cmpuint(ldl_le_p(m.body + 24), ==, 3950);
    g_assert_cmpuint(ldl_le_p(m.body + 28), ==, 1);
    /* a setting is read back */
    stl_le_p(v, 600);
    rtos(qts, &m, 0x45, v, 4);
    g_assert_cmpuint(ldl_le_p(m.body + 16), ==, 0);
    rtos(qts, &m, 0x44, NULL, 0);
    g_assert_cmpuint(ldl_le_p(m.body + 16), ==, 4);
    g_assert_cmpuint(ldl_le_p(m.body + 20), ==, 600);
    tx_hdr = 8;
    qtest_quit(qts);
}

/* The RTOS joins the access point itself, for a key it was given */
static void test_6461_connect(const void *data)
{
    QTestState *qts = qtest_init("-machine ingenic-t41 "
                                 "-device atbm6461,bus=msc0,ssid=lab,"
                                 "psk=passphrase1");
    uint8_t c[108] = {};
    Msg m;

    tx_hdr = 32;
    sdio_enumerate(qts);
    irq_enable(qts);
    iot_startup(qts, &m);

    stl_le_p(c, 1);
    strcpy((char *)c + 4, "lab");
    strcpy((char *)c + 40, "passphrase1");
    rtos(qts, &m, 0x02, c, sizeof(c));
    qtest_clock_step(qts, 200 * 1000000LL);
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmphex(m.id, ==, WSM_EVENT);
    g_assert_cmpuint(ldl_le_p(m.body), ==, 1);
    g_assert_cmpstr((char *)m.body + 4, ==, "lab");

    /* a key that is not the access point's is refused */
    strcpy((char *)c + 40, "wrong-key-1");
    rtos(qts, &m, 0x02, c, sizeof(c));
    qtest_clock_step(qts, 200 * 1000000LL);
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmpuint(ldl_le_p(m.body), ==, 14);
    tx_hdr = 8;
    qtest_quit(qts);
}

static void test_download(const void *data)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 -device atbm6041,"
                                  "bus=msc0");
    uint8_t fw[4] = { 1, 2, 3, 4 }, cfg[2] = {};
    Msg m;

    sdio_enumerate(qts);
    hif_write(qts, HIF_CONFIG, CONFIG_CPU_RESET);
    hif_write(qts, HIF_SRAM_BASE, 0x00010000);
    sd_io_extended(qts, true, HIF_SRAM_DPORT, fw, sizeof(fw));
    hif_write(qts, HIF_CONFIG, CONFIG_IRQ);
    qtest_clock_step(qts, 500 * 1000000LL);
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmphex(m.id, ==, WSM_STARTUP);
    g_assert_cmpuint(lduw_le_p(m.body + 18), ==, 15000);
    g_assert_false(msg_pop(qts, &m));
    /* 0x0003 is a generic request here, not a join: the answer echoes it */
    request(qts, &m, 0x0003, cfg, 2);
    g_assert_cmpuint(m.len, ==, 256);
    qtest_quit(qts);
}

static void test_join(const void *data)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 "
                                  "-device %s,bus=msc0,ssid=lab,"
                                  "psk=passphrase1", (const char *)data);
    uint8_t join[108] = {};
    uint8_t cfg[2];
    Msg m;

    sdio_enumerate(qts);
    irq_enable(qts);
    iot_startup(qts, &m);

    /* the config the host reads: not connected */
    stw_le_p(cfg, 128);
    request(qts, &m, 0x0002, cfg, 2);
    g_assert_cmpuint(m.len, ==, 128);
    g_assert_cmpuint(m.body[10], ==, 0);

    /* the right key joins, in an event the driver reads the link from */
    join[7] = 3;
    memcpy(join + 8, "lab", 3);
    join[40] = 4;
    join[41] = 11;
    memcpy(join + 44, "passphrase1", 11);
    request(qts, &m, 0x0003, join, sizeof(join));
    qtest_clock_step(qts, 200 * 1000000LL);
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmphex(m.id, ==, WSM_EVENT);
    g_assert_cmpuint(ldl_le_p(m.body), ==, 1);
    g_assert_cmpstr((char *)m.body + 4, ==, "lab");
    g_assert_cmpuint(m.body[4 + 32], ==, 3);
    request(qts, &m, 0x0002, cfg, 2);
    g_assert_cmpuint(m.body[10], ==, 1);

    /* the wrong one does not */
    join[44] = 'P';
    request(qts, &m, 0x0003, join, sizeof(join));
    qtest_clock_step(qts, 200 * 1000000LL);
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmpuint(ldl_le_p(m.body), ==, 14);
    g_assert_cmpuint(m.body[4], ==, 1);
    qtest_quit(qts);
}

static void test_scan(const void *data)
{
    QTestState *qts = qtest_initf("-machine ingenic-t41 "
                                  "-device %s,bus=msc0,ssid=lab,"
                                  "psk=passphrase1,channel=11",
                                  (const char *)data);
    uint8_t state[2], scan[4] = {};
    uint8_t info[8 + 47 * 30] = {};
    Msg m;

    sdio_enumerate(qts);
    irq_enable(qts);
    iot_startup(qts, &m);

    request(qts, &m, 0x000a, scan, sizeof(scan));
    stw_le_p(state, 8);
    request(qts, &m, 0x0029, state, 2);
    g_assert_cmpuint(m.body[4], ==, 1);
    qtest_clock_step(qts, 300 * 1000000LL);
    request(qts, &m, 0x0029, state, 2);
    g_assert_cmpuint(m.body[4], ==, 0);

    request(qts, &m, 0x000b, info, sizeof(info));
    g_assert_cmpuint(lduw_le_p(m.body + 4), ==, 1);
    g_assert_cmpstr((char *)m.body + 8, ==, "lab");
    g_assert_cmpuint(m.body[8 + 40], ==, 11);
    /* RSN and encrypted */
    g_assert_cmphex(m.body[8 + 41], ==, 0x44);
    qtest_quit(qts);
}

/* A host that sleeps and asked to be woken by the PIR is told why */
static void test_wake(const void *data)
{
    g_autofree char *dir = g_dir_make_tmp("atbm-XXXXXX", NULL);
    g_autofree char *sock = g_build_filename(dir, "uart", NULL);
    QTestState *qts = qtest_initf("-machine ingenic-t41 "
                                  "-chardev socket,id=u0,path=%s,server=on,"
                                  "wait=off "
                                  "-device %s,id=w,bus=msc0,chardev=u0,"
                                  "battery=64,battery-mv=3900",
                                  sock, (const char *)data);
    uint8_t ps[12] = {}, ev[8] = {}, alive[16] = {};
    uint8_t rx[16];
    uint8_t req[4] = { 0x7b, 0x04, 0x45, 0xc5 };
    struct sockaddr_un addr = { .sun_family = AF_UNIX };
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    Msg m;
    ssize_t n;

    g_assert(fd >= 0);
    g_strlcpy(addr.sun_path, sock, sizeof(addr.sun_path));
    g_assert(connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    qtest_irq_intercept_out_named(qts, "/machine/peripheral/w", "wake");

    sdio_enumerate(qts);
    irq_enable(qts);
    iot_startup(qts, &m);

    /* the app's power info request, answered over the UART */
    g_assert_cmpint(write(fd, req, sizeof(req)), ==, sizeof(req));
    qtest_clock_step(qts, 10 * 1000000LL);
    n = read(fd, rx, sizeof(rx));
    g_assert_cmpint(n, ==, 16);
    g_assert_cmphex(rx[0], ==, 0x7b);
    g_assert_cmpuint(rx[1], ==, 16);
    g_assert_cmphex(rx[2], ==, 0x45);
    g_assert_cmpuint(rx[4], ==, 64);
    g_assert_cmpuint(lduw_le_p(rx + 7), ==, 3900);
    g_assert_cmphex(rx[15], ==, (uint8_t)~(0x7b ^ 16 ^ 0x45 ^ 64 ^
                                           (3900 & 0xff) ^ (3900 >> 8)));

    /* awake, motion is reported but wakes no one */
    qtest_set_irq_in(qts, "/machine/peripheral/w", "pir", 0, 1);
    qtest_clock_step(qts, 10 * 1000000LL);
    n = read(fd, rx, 4);
    g_assert_cmpint(n, ==, 4);
    g_assert_cmphex(rx[2], ==, 0x03);
    g_assert_cmphex(rx[3], ==, 0x83);
    qtest_set_irq_in(qts, "/machine/peripheral/w", "pir", 0, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* light sleep, with all the reasons to wake allowed */
    stl_le_p(ev + 4, 0x0f);
    request(qts, &m, 0x0037, ev, sizeof(ev));
    stl_le_p(ps + 8, 2);
    request(qts, &m, 0x0008, ps, sizeof(ps));
    qtest_set_irq_in(qts, "/machine/peripheral/w", "pir", 0, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmphex(m.id, ==, WSM_EVENT);
    g_assert_cmpuint(ldl_le_p(m.body), ==, 12);
    g_assert_cmpuint(m.body[4], ==, 0);
    qtest_set_irq_in(qts, "/machine/peripheral/w", "pir", 0, 0);

    /* the host's first word lowers the line; sleep again, with a keep-alive */
    request(qts, &m, 0x0002, ps, 2);
    g_assert_false(qtest_get_irq(qts, 0));
    stl_le_p(alive + 8, 1);
    stl_le_p(alive + 12, 2);
    request(qts, &m, 0x003a, alive, sizeof(alive));
    g_assert_cmpuint(ldl_le_p(m.body + 4), ==, 2121);
    stl_le_p(ps + 8, 2);
    request(qts, &m, 0x0008, ps, sizeof(ps));
    /* the link was never joined: two periods unanswered wake the host */
    qtest_clock_step(qts, 2500 * 1000000LL);
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_true(msg_pop(qts, &m));
    g_assert_cmpuint(ldl_le_p(m.body), ==, 12);
    g_assert_cmpuint(m.body[4], ==, 3);

    close(fd);
    qtest_quit(qts);
    g_remove(sock);
    g_rmdir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    if (!qtest_has_machine("ingenic-t41") ||
        !qtest_has_device("atbm6041") || !qtest_has_device("atbm6441") ||
        !qtest_has_device("atbm6461")) {
        return g_test_run();
    }
    qtest_add_data_func("atbm/6041/ids", "atbm6041", test_ids);
    qtest_add_data_func("atbm/6041/join", "atbm6041", test_join);
    qtest_add_data_func("atbm/6041/scan", "atbm6041", test_scan);
    qtest_add_data_func("atbm/6041/download", "atbm6041", test_download);
    qtest_add_data_func("atbm/6441/ids", "atbm6441", test_ids);
    qtest_add_data_func("atbm/6441/join", "atbm6441", test_join);
    qtest_add_data_func("atbm/6441/scan", "atbm6441", test_scan);
    qtest_add_data_func("atbm/6441/wake", "atbm6441", test_wake);
    qtest_add_data_func("atbm/6461/ids", "atbm6461", test_ids_6461);
    qtest_add_data_func("atbm/6461/startup", "atbm6461", test_6461_startup);
    qtest_add_data_func("atbm/6461/rtos", "atbm6461", test_6461_rtos);
    qtest_add_data_func("atbm/6461/connect", "atbm6461", test_6461_connect);
    return g_test_run();
}
