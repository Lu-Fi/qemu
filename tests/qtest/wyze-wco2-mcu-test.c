/*
 * QTest testcase for the Wyze Cam Outdoor v2's MCU
 *
 * Driven over the T31's I2C0 controller: the part answers only after a
 * rising edge on its wake input, a command is a framed 8 bytes with the
 * status and data back as 8 bytes and a CRC, a set takes a payload, and
 * its RTC, alarms, outputs, event flags, watchdog and inputs work as the
 * firmware's do.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define MCU             "/machine/peripheral/mcu"
#define I2C0            0x10050000
#define TAR             (I2C0 + 0x04)
#define DC              (I2C0 + 0x10)
#define CTXABRT         (I2C0 + 0x54)
#define ENB             (I2C0 + 0x6c)
#define TXABRT          (I2C0 + 0x80)

#define MS              1000000LL

typedef struct {
    uint8_t b[9];
} Reply;

static void i2c_init(QTestState *qts);

static QTestState *mcu_init(const char *extra)
{
    QTestState *qts = qtest_initf("-machine ingenic-t31 -rtc clock=vm "
                                  "-global ingenic-t31.soc-variant=t31x "
                                  "-device wyze-wco2-mcu,bus=i2c0,"
                                  "id=mcu%s", extra);

    i2c_init(qts);
    return qts;
}

static uint8_t crc8(const uint8_t *p, int n)
{
    uint8_t crc = 0;

    while (n--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = crc & 1 ? crc >> 1 ^ 0x8c : crc >> 1;
        }
    }
    return crc;
}

static void wake(QTestState *qts)
{
    qtest_set_irq_in(qts, MCU, "wake", 0, 0);
    qtest_set_irq_in(qts, MCU, "wake", 0, 1);
}

static bool nacked(QTestState *qts)
{
    bool n = qtest_readl(qts, TXABRT) & 1;

    qtest_readl(qts, CTXABRT);
    return n;
}

static void i2c_write(QTestState *qts, const uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, DC, b[i] | (i == n - 1 ? 0x200 : 0));
    }
}

static void i2c_read(QTestState *qts, uint8_t *b, int n)
{
    for (int i = 0; i < n; i++) {
        qtest_writel(qts, DC, 0x100 | (i == n - 1 ? 0x200 : 0));
    }
    for (int i = 0; i < n; i++) {
        b[i] = qtest_readl(qts, DC);
    }
}

static void frame(uint8_t *f, uint16_t idx)
{
    f[0] = 0xc3;
    f[1] = 0x09;
    f[2] = idx >> 8;
    f[3] = idx;
    f[4] = ~idx >> 8;
    f[5] = ~idx;
    f[6] = 0x55;
    f[7] = 0xaa;
}

/* a 64-bit value as the host sends it: each halfword byte-swapped */
static void wire64(uint8_t *w, uint64_t v)
{
    for (int i = 0; i < 8; i++) {
        w[i ^ 1] = v >> (8 * i);
    }
}

static void get_reply(QTestState *qts, Reply *r)
{
    i2c_read(qts, r->b, 9);
    g_assert_cmphex(r->b[8], ==, crc8(r->b, 8));
}

static uint32_t status_of(const Reply *r)
{
    return r->b[0] | r->b[1] << 8 | r->b[2] << 16 | (uint32_t)r->b[3] << 24;
}

/* wake, send a frame, read the status; the caller goes on from there */
static uint32_t cmd_status(QTestState *qts, uint16_t idx)
{
    uint8_t f[8];
    Reply r;

    frame(f, idx);
    wake(qts);
    i2c_write(qts, f, 8);
    g_assert_false(nacked(qts));
    get_reply(qts, &r);
    g_assert_false(nacked(qts));
    return status_of(&r);
}

/* a get: the status, then the 8 data bytes as a little-endian 64 bits */
static uint64_t cmd_get(QTestState *qts, uint16_t idx)
{
    Reply r;
    uint64_t v = 0;

    g_assert_cmphex(cmd_status(qts, idx), ==, 0x5aa5a55a);
    get_reply(qts, &r);
    for (int i = 7; i >= 0; i--) {
        v = v << 8 | r.b[i];
    }
    return v;
}

/* a set: the status, then the wire payload */
static void cmd_set(QTestState *qts, uint16_t idx, const uint8_t *payload)
{
    g_assert_cmphex(cmd_status(qts, idx), ==, 0x5aa5a55a);
    i2c_write(qts, payload, 8);
    g_assert_false(nacked(qts));
}

static void cmd_set64(QTestState *qts, uint16_t idx, uint64_t v)
{
    uint8_t w[8];

    wire64(w, v);
    cmd_set(qts, idx, w);
}

static void i2c_init(QTestState *qts)
{
    qtest_writel(qts, ENB, 0);
    qtest_writel(qts, TAR, 0x11);
    qtest_writel(qts, ENB, 1);
}

static void cmd_only(QTestState *qts, uint16_t idx)
{
    g_assert_cmphex(cmd_status(qts, idx), ==, 0x5aa5a55a);
}

static bool out(QTestState *qts, const char *name)
{
    return qtest_qom_get_bool(qts, MCU, name);
}

static void set_in(QTestState *qts, const char *name, bool v)
{
    qtest_qom_set_bool(qts, MCU, name, v);
}

static void test_asleep(void)
{
    QTestState *qts = mcu_init("");
    uint8_t f[8];
    Reply r;

    frame(f, 0xc001);
    i2c_write(qts, f, 8);
    g_assert_true(nacked(qts));

    /* an edge opens one command, for a second */
    wake(qts);
    qtest_clock_step(qts, 1100 * MS);
    i2c_write(qts, f, 8);
    g_assert_true(nacked(qts));

    /* and the command's end puts it back to sleep */
    g_assert_cmphex(cmd_status(qts, 0xc001), ==, 0x5aa5a55a);
    get_reply(qts, &r);
    i2c_write(qts, f, 8);
    g_assert_true(nacked(qts));
    qtest_quit(qts);
}

static void test_status_and_crc(void)
{
    QTestState *qts = mcu_init("");
    uint8_t f[8];
    Reply r;
    static const uint8_t ok[] = { 0x5a, 0xa5, 0xa5, 0x5a, 0x23, 0x2c, 0, 8 };

    frame(f, 0xc001);
    wake(qts);
    i2c_write(qts, f, 8);
    get_reply(qts, &r);
    g_assert_cmpmem(r.b, 8, ok, 8);
    get_reply(qts, &r);
    g_assert_cmpmem(r.b, 8, "02.01.09", 8);
    qtest_quit(qts);
}

static void test_idle_bus(void)
{
    QTestState *qts = mcu_init("");
    uint8_t f[8], b[12];

    frame(f, 0x4001);
    wake(qts);
    i2c_write(qts, f, 8);
    i2c_read(qts, b, 12);
    g_assert_cmphex(b[8], ==, crc8(b, 8));
    g_assert_cmphex(b[9], ==, 0xff);
    g_assert_cmphex(b[11], ==, 0xff);
    qtest_quit(qts);
}

static void test_errors(void)
{
    QTestState *qts = mcu_init("");
    uint8_t f[8];
    Reply r;

    /* the firmware dispatches a bad frame anyway until 10000 of them */
    frame(f, 0xc001);
    f[0] = 0;
    wake(qts);
    i2c_write(qts, f, 8);
    get_reply(qts, &r);
    g_assert_cmphex(status_of(&r), ==, 0x5aa5a55a);
    get_reply(qts, &r);

    /* an index the part does not know: the unknown status, no data */
    g_assert_cmphex(cmd_status(qts, 0xc00b), ==, 0x80030002);
    g_assert_cmphex(cmd_status(qts, 0xc005), ==, 0x80030002);
    g_assert_cmphex(cmd_status(qts, 0x4000), ==, 0x80030002);
    g_assert_cmphex(cmd_status(qts, 0xc012), ==, 0x80030002);

    /* a get without its read ends the command */
    frame(f, 0xc001);
    wake(qts);
    i2c_write(qts, f, 8);
    i2c_read(qts, r.b, 4);
    g_assert_false(nacked(qts));
    i2c_write(qts, f, 8);
    g_assert_true(nacked(qts));
    qtest_quit(qts);
}

static void test_no_reply(void)
{
    QTestState *qts = mcu_init("");
    uint8_t f[8];

    /* no status and no data bit: nothing to read, the part sleeps */
    frame(f, 0x0001);
    wake(qts);
    i2c_write(qts, f, 8);
    g_assert_false(nacked(qts));
    g_assert_true(out(qts, "out-wifi"));
    qtest_quit(qts);
}

static void test_outputs(void)
{
    QTestState *qts = mcu_init("");

    g_assert_true(out(qts, "out-wifi"));
    cmd_only(qts, 0x4004);
    g_assert_false(out(qts, "out-wifi"));
    cmd_only(qts, 0x4003);
    g_assert_true(out(qts, "out-wifi"));
    cmd_only(qts, 0x4005);
    g_assert_false(out(qts, "out-wifi"));
    qtest_clock_step(qts, 150 * MS);
    g_assert_true(out(qts, "out-wifi"));

    cmd_only(qts, 0x400d);
    g_assert_true(out(qts, "out-red-led"));
    cmd_only(qts, 0x400e);
    g_assert_false(out(qts, "out-red-led"));
    cmd_only(qts, 0x4006);
    g_assert_true(out(qts, "out-pir"));
    g_assert_cmphex(cmd_get(qts, 0xc011), ==, 0x1d);
    cmd_only(qts, 0x4007);
    g_assert_false(out(qts, "out-pir"));
    g_assert_cmphex(cmd_get(qts, 0xc011), ==, 0x0);

    /* the pins as the firmware reads them: PB0, PB1 and PB6 high */
    g_assert_cmphex(cmd_get(qts, 0xc00a) & 0x7f, ==, 0x43);
    qtest_quit(qts);
}

static void test_settings(void)
{
    QTestState *qts = mcu_init("");

    /* power-on defaults, then the sets read back */
    g_assert_cmphex(cmd_get(qts, 0xc00e), ==, 2000);
    g_assert_cmphex(cmd_get(qts, 0xc00f), ==, 3000);
    g_assert_cmphex(cmd_get(qts, 0xc010), ==, 1000);
    cmd_set64(qts, 0x4017, 1234);
    cmd_set64(qts, 0x4018, 2345);
    cmd_set64(qts, 0x4019, 345);
    g_assert_cmphex(cmd_get(qts, 0xc00e), ==, 1234);
    g_assert_cmphex(cmd_get(qts, 0xc00f), ==, 2345);
    g_assert_cmphex(cmd_get(qts, 0xc010), ==, 345);
    cmd_set64(qts, 0x4016, 77);
    g_assert_cmphex(cmd_get(qts, 0xc00d), ==, 77);
    g_assert_cmphex(cmd_get(qts, 0xc00c), ==, 0);
    cmd_set64(qts, 0x4015, 5);
    g_assert_cmphex(cmd_get(qts, 0xc00c), ==, 5);
    g_assert_cmphex(cmd_get(qts, 0xc011) & 2, ==, 2);
    cmd_set64(qts, 0x400f, 0x870);
    qtest_quit(qts);
}

static uint64_t bcd_time(int y, int mo, int d, int h, int mi, int s)
{
    return s | (uint64_t)mi << 8 | (uint64_t)h << 16 | (uint64_t)d << 24 |
           (uint64_t)mo << 32 | (uint64_t)(y - 2000) << 40;
}

static void test_rtc(void)
{
    QTestState *qts = mcu_init("");
    uint64_t t;

    cmd_set64(qts, 0x4008, bcd_time(2026, 11, 9, 23, 59, 50));
    t = cmd_get(qts, 0xc006);
    g_assert_cmphex(t, ==, bcd_time(2026, 11, 9, 23, 59, 50));
    qtest_clock_step(qts, 12 * 1000 * MS);
    t = cmd_get(qts, 0xc006);
    g_assert_cmphex(t, ==, bcd_time(2026, 11, 10, 0, 0, 2));
    qtest_quit(qts);
}

static void test_alarm(void)
{
    QTestState *qts = mcu_init("");
    uint64_t a = bcd_time(2000, 0, 0, 10, 20, 33) & 0xffffff;
    uint8_t w[8];

    cmd_set64(qts, 0x4008, bcd_time(2026, 6, 1, 10, 20, 30));

    /* alarm A at 10:20:33 */
    wire64(w, 33 | 20 << 8 | 10 << 16);
    cmd_set(qts, 0x4009, w);
    g_assert_cmphex(cmd_get(qts, 0xc007), ==, a);
    g_assert_false(out(qts, "out-int"));
    qtest_clock_step(qts, 2900 * MS);
    g_assert_false(out(qts, "out-int"));
    qtest_clock_step(qts, 100 * MS);
    g_assert_true(out(qts, "out-int"));
    qtest_clock_step(qts, 11 * MS);
    g_assert_false(out(qts, "out-int"));
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x40);

    /* clear the flag, and the sentinel turns the alarm off */
    cmd_set64(qts, 0x401a, 0x40);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0);
    memcpy(w, "\x2d\x23\x00\x19\0\0\0\0", 8);
    cmd_set(qts, 0x4009, w);
    qtest_clock_step(qts, 86400 * 1000 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0);

    /* B reports only while PA0 is high */
    wire64(w, 5 | 21 << 8 | 10 << 16);
    cmd_set(qts, 0x4010, w);
    g_assert_cmphex(cmd_get(qts, 0xc008), ==, 5 | 21 << 8 | 10 << 16);
    qtest_clock_step(qts, 86400 * 1000 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0);
    set_in(qts, "pir", true);
    qtest_clock_step(qts, 86400 * 1000 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002) & 0x40, ==, 0x40);
    qtest_quit(qts);
}

static void test_inputs(void)
{
    QTestState *qts = mcu_init("");

    /* a fuel gauge alert sets BAT after 100 ms, with an INT pulse */
    set_in(qts, "battery-alert", true);
    g_assert_false(out(qts, "out-int"));
    qtest_clock_step(qts, 101 * MS);
    g_assert_true(out(qts, "out-int"));
    g_assert_cmphex(cmd_get(qts, 0xc003), ==, 0x8);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x8);
    cmd_only(qts, 0x4002);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0);
    set_in(qts, "battery-alert", false);

    /* the Wi-Fi's wake line */
    set_in(qts, "wifi-wake", true);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x2);
    cmd_set64(qts, 0x401a, 0x2);
    set_in(qts, "wifi-wake", false);

    /* one press, then two */
    set_in(qts, "button", true);
    qtest_clock_step(qts, 10 * MS);
    set_in(qts, "button", false);
    qtest_clock_step(qts, 600 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x1);
    cmd_set64(qts, 0x401a, 0x1);
    for (int i = 0; i < 2; i++) {
        set_in(qts, "button", true);
        qtest_clock_step(qts, 10 * MS);
        set_in(qts, "button", false);
        qtest_clock_step(qts, 100 * MS);
    }
    qtest_clock_step(qts, 600 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x1000);
    cmd_set64(qts, 0x401a, 0x1000);

    /* USB power, either edge */
    set_in(qts, "usb-power", true);
    qtest_clock_step(qts, 40 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x20);
    cmd_set64(qts, 0x401a, 0x20);
    qtest_quit(qts);
}

static void test_pir(void)
{
    QTestState *qts = mcu_init("");

    cmd_set64(qts, 0x4015, 0);
    cmd_only(qts, 0x4006);
    cmd_set64(qts, 0x4017, 100);
    g_assert_cmphex(cmd_get(qts, 0xc009), ==, 2048);

    /* an edge, with the sample inside the window: no event */
    set_in(qts, "pir", true);
    qtest_clock_step(qts, 300 * MS);
    set_in(qts, "pir", false);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0);

    /* outside it: PIR, the ADC value in the data */
    qtest_qmp_assert_success(qts, "{ 'execute': 'qom-set', 'arguments': "
                             "{ 'path': %s, 'property': 'pir-adc', "
                             "'value': 3500 } }", MCU);
    set_in(qts, "pir", true);
    qtest_clock_step(qts, 5 * MS);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x4);
    g_assert_cmphex(cmd_get(qts, 0xc004), ==, 3500ULL | 3500ULL << 32);
    qtest_quit(qts);
}

static void test_watchdog(void)
{
    QTestState *qts = mcu_init("");

    cmd_set64(qts, 0x4013, 2);
    cmd_only(qts, 0x4011);

    /* feeding keeps it off */
    for (int i = 0; i < 4; i++) {
        qtest_clock_step(qts, 1000 * MS);
        cmd_only(qts, 0x4014);
    }
    g_assert_true(out(qts, "out-master"));

    /*
     * not fed: the SoC loses its power at the period, and gets it back
     * (and a reset) three slow ticks on, with the flag
     */
    qtest_clock_step(qts, 2200 * MS);
    g_assert_false(out(qts, "out-master"));
    qtest_clock_step(qts, 3500 * MS);
    g_assert_true(out(qts, "out-master"));
    qtest_qmp_eventwait(qts, "RESET");
    i2c_init(qts);
    g_assert_cmphex(cmd_get(qts, 0xc002), ==, 0x200);
    qtest_quit(qts);
}

static void test_watchdog_scale(void)
{
    QTestState *qts = mcu_init(",wdt-scale=3");

    cmd_set64(qts, 0x4013, 2);
    cmd_only(qts, 0x4011);

    /* a period of 2 slow ticks runs for 6 */
    qtest_clock_step(qts, 5000 * MS);
    g_assert_true(out(qts, "out-master"));
    qtest_clock_step(qts, 2000 * MS);
    g_assert_false(out(qts, "out-master"));

    /* and the power cycle that follows is as long as on any */
    qtest_clock_step(qts, 3500 * MS);
    g_assert_true(out(qts, "out-master"));
    qtest_quit(qts);
}

static void test_power_commands(void)
{
    QTestState *qts = mcu_init("");
    uint8_t f[8];

    /* off: the SoC's power drops, and stays off with no event pending */
    frame(f, 0x0001);
    wake(qts);
    i2c_write(qts, f, 8);
    g_assert_false(out(qts, "out-master"));
    qtest_clock_step(qts, 2000 * MS);
    g_assert_false(out(qts, "out-master"));

    /* a button press brings it back */
    set_in(qts, "button", true);
    qtest_clock_step(qts, 10 * MS);
    set_in(qts, "button", false);
    qtest_clock_step(qts, 600 * MS);
    g_assert_true(out(qts, "out-master"));
    qtest_qmp_eventwait(qts, "RESET");
    i2c_init(qts);

    /* recovery: off, then on again by itself */
    frame(f, 0x0002);
    wake(qts);
    i2c_write(qts, f, 8);
    g_assert_false(out(qts, "out-master"));
    qtest_clock_step(qts, 3100 * MS);
    g_assert_true(out(qts, "out-master"));
    qtest_qmp_eventwait(qts, "RESET");
    i2c_init(qts);

    /* shutoff: the wake line no longer starts a command */
    frame(f, 0x0003);
    wake(qts);
    i2c_write(qts, f, 8);
    g_assert_false(out(qts, "out-master"));
    wake(qts);
    i2c_write(qts, f, 8);
    g_assert_true(nacked(qts));
    qtest_quit(qts);
}

static void test_version(void)
{
    QTestState *qts = mcu_init(",version=03.00.01");
    uint8_t f[8];
    Reply r;

    frame(f, 0xc001);
    wake(qts);
    i2c_write(qts, f, 8);
    get_reply(qts, &r);
    get_reply(qts, &r);
    g_assert_cmpmem(r.b, 8, "03.00.01", 8);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("wyze-wco2-mcu/asleep", test_asleep);
    qtest_add_func("wyze-wco2-mcu/status-crc", test_status_and_crc);
    qtest_add_func("wyze-wco2-mcu/idle-bus", test_idle_bus);
    qtest_add_func("wyze-wco2-mcu/errors", test_errors);
    qtest_add_func("wyze-wco2-mcu/no-reply", test_no_reply);
    qtest_add_func("wyze-wco2-mcu/outputs", test_outputs);
    qtest_add_func("wyze-wco2-mcu/settings", test_settings);
    qtest_add_func("wyze-wco2-mcu/rtc", test_rtc);
    qtest_add_func("wyze-wco2-mcu/alarm", test_alarm);
    qtest_add_func("wyze-wco2-mcu/inputs", test_inputs);
    qtest_add_func("wyze-wco2-mcu/pir", test_pir);
    qtest_add_func("wyze-wco2-mcu/watchdog", test_watchdog);
    qtest_add_func("wyze-wco2-mcu/watchdog-scale", test_watchdog_scale);
    qtest_add_func("wyze-wco2-mcu/power", test_power_commands);
    qtest_add_func("wyze-wco2-mcu/version", test_version);
    return g_test_run();
}
