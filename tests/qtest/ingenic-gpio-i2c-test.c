/*
 * QTest testcase for the Ingenic GPIO I2C bus and the CW2015 on it
 *
 * The test is the guest's i2c-gpio: it bit-bangs START, bytes, ACKs and
 * STOP through a T31's PB registers on two buses at once, SDA and SCL on
 * pads that are no I2C controller's, and reads back what a CW2015 answers.
 * The gauge's registers, samples, sleep, quick start, POR and alert follow
 * the CW2015-DS V2.1 datasheet.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"
#include "qobject/qdict.h"

#define GPIO            0x10010000
#define PORT(pin)       (GPIO + ((pin) / 32) * 0x1000)
#define BIT_OF(pin)     (1u << ((pin) % 32))

/* PXMSK 0x20, PXPAT1 0x30, PXPAT0 0x40, with set at +4 and clear at +8 */
#define MSK             0x20
#define PAT1            0x30
#define PAT0            0x40

typedef struct Bus {
    QTestState *qts;
    int sda, scl;
} Bus;

static void pin_out(Bus *b, int pin, int level)
{
    uint32_t base = PORT(pin);

    qtest_writel(b->qts, base + MSK + 4, BIT_OF(pin));
    qtest_writel(b->qts, base + PAT1 + 8, BIT_OF(pin));
    qtest_writel(b->qts, base + PAT0 + (level ? 4 : 8), BIT_OF(pin));
}

/* An open-drain release: the pad an input, high unless a device holds it */
static void sda_release(Bus *b)
{
    uint32_t base = PORT(b->sda);

    qtest_writel(b->qts, base + MSK + 4, BIT_OF(b->sda));
    qtest_writel(b->qts, base + PAT1 + 4, BIT_OF(b->sda));
}

static void sda_set(Bus *b, int level)
{
    if (level) {
        sda_release(b);
    } else {
        pin_out(b, b->sda, 0);
    }
}

static int sda_get(Bus *b)
{
    return !!(qtest_readl(b->qts, PORT(b->sda)) & BIT_OF(b->sda));
}

static void scl_set(Bus *b, int level)
{
    pin_out(b, b->scl, level);
}

static void i2c_start(Bus *b)
{
    sda_set(b, 1);
    scl_set(b, 1);
    sda_set(b, 0);
    scl_set(b, 0);
}

static void i2c_stop(Bus *b)
{
    sda_set(b, 0);
    scl_set(b, 1);
    sda_set(b, 1);
}

/* Returns true when the slave ACKs: it pulls SDA low */
static bool i2c_put(Bus *b, uint8_t v)
{
    bool ack;

    for (int i = 7; i >= 0; i--) {
        sda_set(b, v >> i & 1);
        scl_set(b, 1);
        scl_set(b, 0);
    }
    sda_release(b);
    scl_set(b, 1);
    ack = !sda_get(b);
    scl_set(b, 0);
    return ack;
}

static uint8_t i2c_get(Bus *b, bool ack)
{
    uint8_t v = 0;

    sda_release(b);
    for (int i = 0; i < 8; i++) {
        scl_set(b, 1);
        v = v << 1 | sda_get(b);
        scl_set(b, 0);
    }
    sda_set(b, !ack);
    scl_set(b, 1);
    scl_set(b, 0);
    sda_release(b);
    return v;
}

static bool reg_write(Bus *b, uint8_t addr, uint8_t reg, const uint8_t *v,
                      size_t n)
{
    bool ok;

    i2c_start(b);
    ok = i2c_put(b, addr << 1) && i2c_put(b, reg);
    for (size_t i = 0; ok && i < n; i++) {
        ok = i2c_put(b, v[i]);
    }
    i2c_stop(b);
    return ok;
}

static void reg_write1(Bus *b, uint8_t addr, uint8_t reg, uint8_t v)
{
    g_assert_true(reg_write(b, addr, reg, &v, 1));
}

static void reg_read(Bus *b, uint8_t addr, uint8_t reg, uint8_t *v, size_t n)
{
    i2c_start(b);
    g_assert_true(i2c_put(b, addr << 1));
    g_assert_true(i2c_put(b, reg));
    /* repeated START */
    sda_release(b);
    scl_set(b, 1);
    sda_set(b, 0);
    scl_set(b, 0);
    g_assert_true(i2c_put(b, addr << 1 | 1));
    for (size_t i = 0; i < n; i++) {
        v[i] = i2c_get(b, i + 1 < n);
    }
    i2c_stop(b);
}

static uint8_t reg_read1(Bus *b, uint8_t addr, uint8_t reg)
{
    uint8_t v;

    reg_read(b, addr, reg, &v, 1);
    return v;
}

static uint16_t reg_read2(Bus *b, uint8_t addr, uint8_t reg)
{
    uint8_t v[2];

    reg_read(b, addr, reg, v, 2);
    return v[0] << 8 | v[1];
}

#define MS              1000000ULL
#define ADDR            0x62

static QTestState *init(void)
{
    return qtest_init("-machine ingenic-t31 "
                      "-global ingenic-t31.soc-variant=t31x "
                      "-device ingenic-gpio-i2c,id=i2c-gpio1,sda=57,scl=58 "
                      "-device cw2015,id=gauge,bus=i2c-gpio1 "
                      "-device ingenic-gpio-i2c,id=i2c-gpio2,sda=54,scl=51 "
                      "-device cw2015,id=other,bus=i2c-gpio2,address=0x11,"
                      "voltage-mv=3700");
}

static void set_prop(QTestState *qts, const char *dev, const char *prop,
                     int64_t v)
{
    g_autofree char *path = g_strdup_printf("/machine/peripheral/%s", dev);

    qtest_qmp_assert_success(qts, "{'execute': 'qom-set', 'arguments': "
                             "{'path': %s, 'property': %s, 'value': %" PRId64
                             "}}", path, prop, v);
}

static void test_bus(void)
{
    QTestState *qts = init();
    Bus a = { qts, 57, 58 }, b = { qts, 54, 51 };
    uint8_t cfg = 0x18;

    /* idle: both lines high, nothing holds SDA */
    sda_release(&a);
    g_assert_cmpint(sda_get(&a), ==, 1);

    /* an absent address is NACKed, and each bus has its own devices */
    g_assert_false(reg_write(&a, 0x11, 0, &cfg, 1));
    g_assert_false(reg_write(&b, ADDR, 0, &cfg, 1));
    g_assert_cmphex(reg_read1(&a, ADDR, 0), ==, 0x6f);
    g_assert_cmphex(reg_read1(&b, 0x11, 0), ==, 0x6f);

    /* the two gauges are separate parts */
    reg_write1(&a, ADDR, 0x08, 0x5a);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x08), ==, 0x5a);
    g_assert_cmphex(reg_read1(&b, 0x11, 0x08), ==, 0x18);
    qtest_quit(qts);
}

static void test_registers(void)
{
    QTestState *qts = init();
    Bus a = { qts, 57, 58 };
    uint8_t p[64], q[64];

    g_assert_cmphex(reg_read1(&a, ADDR, 0x00), ==, 0x6f);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x08), ==, 0x18);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x0a), ==, 0xc0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 0);

    /* asleep: nothing is sampled */
    qtest_clock_step(qts, 1000 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 0);

    /* the profile steps through its 64 bytes on one transaction */
    for (int i = 0; i < 64; i++) {
        p[i] = 0xa0 + i;
    }
    g_assert_true(reg_write(&a, ADDR, 0x10, p, 64));
    reg_read(&a, ADDR, 0x10, q, 64);
    g_assert_cmpmem(p, 64, q, 64);

    /* wake: VCELL and SOC arrive with the first sample, 10 ms on */
    reg_write1(&a, ADDR, 0x0a, 0);
    qtest_clock_step(qts, 5 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 0);
    qtest_clock_step(qts, 6 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 4000 * 1000 / 305);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 80 * 256);

    /* it follows the cell every 250 ms */
    set_prop(qts, "gauge", "voltage-mv", 3800);
    qtest_clock_step(qts, 100 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 4000 * 1000 / 305);
    qtest_clock_step(qts, 200 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 3800 * 1000 / 305);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 50 * 256);

    /* a set "soc" is what it reports; between curve points it interpolates */
    set_prop(qts, "gauge", "voltage-mv", 3850);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 59 * 256);
    set_prop(qts, "gauge", "soc", 42);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 42 * 256);

    /* RRT is read-only and the version is too */
    set_prop(qts, "gauge", "rrt", 0x123);
    reg_write1(&a, ADDR, 0x07, 0xff);
    reg_write1(&a, ADDR, 0x00, 0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x06), ==, 0x123);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x00), ==, 0x6f);

    /* sleep stops the samples and keeps what they gave */
    reg_write1(&a, ADDR, 0x0a, 0xc0);
    set_prop(qts, "gauge", "voltage-mv", 3300);
    qtest_clock_step(qts, 1000 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 3850 * 1000 / 305);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x0a), ==, 0xc0);
    qtest_quit(qts);
}

static void test_mode(void)
{
    QTestState *qts = init();
    Bus a = { qts, 57, 58 };
    uint8_t p[64] = { 1, 2, 3 }, q[64];

    reg_write1(&a, ADDR, 0x0a, 0);
    qtest_clock_step(qts, 20 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 80 * 256);

    /* quick start: the estimate restarts and the bits do not stick */
    reg_write1(&a, ADDR, 0x0a, 0x30);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x0a), ==, 0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 0);
    qtest_clock_step(qts, 11 * MS);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 80 * 256);

    /* POR: back asleep with the registers zero; the profile stays */
    g_assert_true(reg_write(&a, ADDR, 0x10, p, 64));
    reg_write1(&a, ADDR, 0x08, 0x2a);
    reg_write1(&a, ADDR, 0x0a, 0x0f);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x0a), ==, 0xc0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x02), ==, 0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 0);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x08), ==, 0x2a);
    reg_read(&a, ADDR, 0x10, q, 64);
    g_assert_cmpmem(p, 64, q, 64);

    /* a cell under 2.5 V puts it to sleep by itself */
    set_prop(qts, "gauge", "voltage-mv", 2400);
    reg_write1(&a, ADDR, 0x0a, 0);
    qtest_clock_step(qts, 11 * MS);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x0a), ==, 0xc0);
    g_assert_cmphex(reg_read2(&a, ADDR, 0x04), ==, 0);
    qtest_quit(qts);
}

static void test_alert(void)
{
    QTestState *qts = init();
    Bus a = { qts, 57, 58 };

    reg_write1(&a, ADDR, 0x0a, 0);
    reg_write1(&a, ADDR, 0x08, 5 << 3);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x06) & 0x80, ==, 0);

    set_prop(qts, "gauge", "soc", 5);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x06) & 0x80, ==, 0x80);

    /* only the host clears it, and it does not come back while SOC stays low */
    reg_write1(&a, ADDR, 0x06, 0x80);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x06) & 0x80, ==, 0x80);
    reg_write1(&a, ADDR, 0x06, 0);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x06) & 0x80, ==, 0);

    /* above the threshold and down again it alerts anew */
    set_prop(qts, "gauge", "soc", 50);
    qtest_clock_step(qts, 300 * MS);
    set_prop(qts, "gauge", "soc", 4);
    qtest_clock_step(qts, 300 * MS);
    g_assert_cmphex(reg_read1(&a, ADDR, 0x06) & 0x80, ==, 0x80);
    qtest_quit(qts);
}

static void test_bad_pins(void)
{
    if (g_test_subprocess()) {
        qtest_init("-machine ingenic-t31 "
                   "-device ingenic-gpio-i2c,sda=3,scl=3");
        return;
    }
    g_test_trap_subprocess(NULL, 0, 0);
    g_test_trap_assert_failed();
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("ingenic-gpio-i2c/bus", test_bus);
    qtest_add_func("ingenic-gpio-i2c/bad-pins", test_bad_pins);
    qtest_add_func("cw2015/registers", test_registers);
    qtest_add_func("cw2015/mode", test_mode);
    qtest_add_func("cw2015/alert", test_alert);
    return g_test_run();
}
