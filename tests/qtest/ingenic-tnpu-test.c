/*
 * QTest testcase for the Ingenic TNPU's control unit (CCU) and window
 *
 * The T32's: resets and the bits each register keeps, LOCK and UNLOCK,
 * TOHOST as the SoC's interrupt, a release with no core linked, and TIME
 * at the TNPU clock, as a T32LQ reads them. The T32Pro's two-core layout,
 * as its driver has it, and its window.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include <sys/socket.h>
#include <sys/un.h>
#include "qemu/bswap.h"
#include "libqtest.h"
#include "hw/misc/ingenic-riscv-link.h"

#define CPM             0x10000000
#define CPM_CLKGR1      0x28
#define CPM_CPCCR       0x00
#define CPM_CPAPCR      0x10
#define CPM_TNPUCDR     0x98
#define CCU             0x12700000
#define DRAM            0x12608000

static uint32_t rd(QTestState *qts, uint32_t off)
{
    return qtest_readl(qts, CCU + off);
}

static void wr(QTestState *qts, uint32_t off, uint32_t v)
{
    qtest_writel(qts, CCU + off, v);
}

/* Written all ones then zero: what each reads, the reset put back after */
static void check(QTestState *qts, uint32_t off, uint32_t reset,
                  uint32_t ones, uint32_t zero)
{
    g_test_message("CCU 0x%03x", off);
    g_assert_cmphex(rd(qts, off), ==, reset);
    wr(qts, off, 0xffffffff);
    g_assert_cmphex(rd(qts, off), ==, ones);
    wr(qts, off, 0);
    g_assert_cmphex(rd(qts, off), ==, zero);
    wr(qts, off, reset);
}

static void test_regs(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    g_assert_cmphex(rd(qts, 0xf0), ==, 0x80000000);
    check(qts, 0x00, 0x1e01, 0x1e21, 0x1e00);
    check(qts, 0x04, 0x12608000, 0xffffffff, 0);
    check(qts, 0x18, 0xffffffff, 0xffffffff, 0);
    check(qts, 0x30, 0x202, 0xffffffff, 0);
    check(qts, 0x38, 0x100, 0x100, 0x100);
    check(qts, 0x58, 0x05ffffff, 0xffffffff, 0);
    check(qts, 0x60, 7, 0x3f, 0);
    check(qts, 0x78, 0x1f, 0x3f, 0);
    check(qts, 0x80, 0x002001fd, 0x81ffffff, 0);
    check(qts, 0x88, 0, 1, 0);
    check(qts, 0xfc, 1, 1, 0);
    check(qts, 0x100, 6, 0xf, 6);
    check(qts, 0x104, 0, 0, 0);
    check(qts, 0x1fc, 0, 0, 0);

    /* LOCK takes a write only while unlocked */
    wr(qts, 0xf4, 1);
    g_assert_cmphex(rd(qts, 0xf4), ==, 0x80000001);
    wr(qts, 0xf4, 0);
    g_assert_cmphex(rd(qts, 0xf4), ==, 0x80000001);
    wr(qts, 0xf8, 1);
    g_assert_cmphex(rd(qts, 0xf4), ==, 1);
    g_assert_cmphex(rd(qts, 0xf8), ==, 0);
    wr(qts, 0xf4, 0);
    g_assert_cmphex(rd(qts, 0xf4), ==, 0x80000000);
    qtest_quit(qts);
}

static void test_irq_and_release(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_irq_intercept_out_named(qts, "/machine/soc/tnpu", "sysbus-irq");
    g_assert_false(qtest_get_irq(qts, 0));
    wr(qts, 0x0c, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(qts, 0x0c, 0);
    g_assert_false(qtest_get_irq(qts, 0));

    /* FROMHOST keeps its value held; no core linked, it never sleeps */
    wr(qts, 0x08, 1);
    g_assert_cmphex(rd(qts, 0x08), ==, 1);
    wr(qts, 0xf0, 0);
    g_assert_cmphex(rd(qts, 0x00), ==, 0x1c11);
    wr(qts, 0xf0, 0x80000000);
    g_assert_cmphex(rd(qts, 0x00), ==, 0x1e01);
    qtest_quit(qts);
}

/* TIME at SCLK_A (the APLL's 900 MHz, as U-Boot sets them) over TNPUCDR's 2 */
static void test_time(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");
    uint32_t t;

    qtest_writel(qts, CPM + CPM_CPAPCR, 0x04b0510d);
    qtest_writel(qts, CPM + CPM_CPCCR, 0x9a7b5510);
    qtest_writel(qts, CPM + CPM_TNPUCDR, 1u << 29 | 1);
    qtest_writel(qts, CPM + CPM_CLKGR1,
                 qtest_readl(qts, CPM + CPM_CLKGR1) & ~(1u << 11));
    wr(qts, 0x10, 0);
    qtest_clock_step(qts, 1000000);
    g_assert_cmphex(rd(qts, 0x10), ==, 0);
    wr(qts, 0x00, rd(qts, 0x00) | 1 << 5);
    qtest_clock_step(qts, 1000000);
    t = rd(qts, 0x10);
    g_assert_cmpuint(t, >=, 449999);
    g_assert_cmpuint(t, <=, 450001);
    qtest_quit(qts);
}

/*
 * The T32Pro's, as its driver lays the CCU out (no board read): two
 * cores' blocks, each core held by its own bit, either core's TOHOST the
 * SoC's interrupt, plain PMA and LOCK words
 */
static void test_pro(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32 "
                                 "-global ingenic-t31.soc-variant=t32prozl");

    qtest_irq_intercept_out_named(qts, "/machine/soc/tnpu", "sysbus-irq");
    g_assert_cmphex(rd(qts, 0x38), ==, 0x150);
    g_assert_cmphex(rd(qts, 0xf0), ==, 0xc0000000);
    check(qts, 0x1004, 0x150b0000, 0xffffffff, 0);
    check(qts, 0x2004, 0x150b0000, 0xffffffff, 0);
    check(qts, 0x54, 0x23ffffff, 0xffffffff, 0);
    check(qts, 0x74, 0x3f, 0x3f, 0);
    check(qts, 0x24, 0, 0xffffffff, 0);
    check(qts, 0x20c, 0, 0xffffffff, 0);
    check(qts, 0x100, 0x16, 0x1f, 0x16);
    g_assert_cmphex(rd(qts, 0x04), ==, 0);
    g_assert_cmphex(rd(qts, 0x3004), ==, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    wr(qts, 0x200c, 1);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(qts, 0x200c, 0);
    wr(qts, 0x100c, 2);
    g_assert_true(qtest_get_irq(qts, 0));
    wr(qts, 0x100c, 0);
    g_assert_false(qtest_get_irq(qts, 0));
    /* released with no core linked, neither sleeps */
    wr(qts, 0xf0, 0x40000000);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0);
    wr(qts, 0xf0, 0);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0);
    qtest_writel(qts, 0x150b1ffc, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, 0x150b1ffc), ==, 0x12345678);
    qtest_quit(qts);
}

/* A T32Pro core of the test's own, speaking the link to the SoC */
typedef struct {
    int fd;
    uint32_t wakes;             /* IRQ and RUN messages had */
    uint32_t run_addr, run, irq3;
} FakeCore;

static void fake_send(FakeCore *f, uint32_t type, uint32_t addr,
                      uint32_t size, uint32_t data)
{
    IngenicRiscvMsg m = {
        .type = cpu_to_le32(type), .addr = cpu_to_le32(addr),
        .size = cpu_to_le32(size), .data = cpu_to_le32(data),
    };

    g_assert_cmpint(send(f->fd, &m, sizeof(m), 0), ==, sizeof(m));
}

/* The next message, any memory it passes closed; its type */
static uint32_t fake_recv(FakeCore *f, IngenicRiscvMsg *m)
{
    char ctl[CMSG_SPACE(sizeof(int))];
    struct iovec iov = { .iov_base = m, .iov_len = sizeof(*m) };
    struct msghdr mh = {
        .msg_iov = &iov, .msg_iovlen = 1,
        .msg_control = ctl, .msg_controllen = sizeof(ctl),
    };
    struct cmsghdr *c;

    g_assert_cmpint(recvmsg(f->fd, &mh, MSG_WAITALL), ==, sizeof(*m));
    for (c = CMSG_FIRSTHDR(&mh); c; c = CMSG_NXTHDR(&mh, c)) {
        if (c->cmsg_type == SCM_RIGHTS) {
            close(*(int *)CMSG_DATA(c));
        }
    }
    switch (le32_to_cpu(m->type)) {
    case IRL_RUN:
        f->wakes++;
        f->run_addr = le32_to_cpu(m->addr);
        f->run = le32_to_cpu(m->data);
        break;
    case IRL_IRQ:
        f->wakes++;
        if (le32_to_cpu(m->addr) == 3) {
            f->irq3 = le32_to_cpu(m->data);
        }
        break;
    }
    return le32_to_cpu(m->type);
}

/* A read across the link: everything the SoC sent before is in */
static uint32_t fake_read(FakeCore *f, uint32_t addr)
{
    IngenicRiscvMsg m;

    fake_send(f, IRL_READ, addr, 4, 0);
    while (fake_recv(f, &m) != IRL_DATA) {
    }
    return le32_to_cpu(m.data);
}

static void fake_connect(FakeCore *f, const char *path)
{
    struct sockaddr_un sa = { .sun_family = AF_UNIX };

    memset(f, 0, sizeof(*f));
    f->fd = socket(AF_UNIX, SOCK_STREAM, 0);
    g_assert_cmpint(f->fd, >=, 0);
    g_strlcpy(sa.sun_path, path, sizeof(sa.sun_path));
    g_assert_cmpint(connect(f->fd, (struct sockaddr *)&sa, sizeof(sa)), ==, 0);
    fake_send(f, IRL_HELLO, 0, 0, INGENIC_RISCV_LINK_VERSION);
}

/*
 * The T32Pro's cores, each on its link: released by its own SOFT_RESET
 * bit at its CRER (core 1 at the driver's boot + 0x80), asleep from its
 * WFI (one sent before a wake it had not seen is stale) to its FROMHOST,
 * CCSR bit 3 while every released core sleeps, a core's TOHOST written
 * across its link the SoC's interrupt
 */
static void test_pro_link(void)
{
    g_autofree char *dir = g_dir_make_tmp("ingenic-tnpu-XXXXXX", NULL);
    g_autofree char *s0 = g_build_filename(dir, "c0.sock", NULL);
    g_autofree char *s1 = g_build_filename(dir, "c1.sock", NULL);
    FakeCore c[2];
    QTestState *qts;

    qts = qtest_initf("-machine ingenic-t32,memory-backend=ddr -m 64M "
                      "-global ingenic-t31.soc-variant=t32prozl "
                      "-object memory-backend-shm,id=ddr,size=64M "
                      "-chardev socket,id=c0,path=%s,server=on,wait=off "
                      "-chardev socket,id=c1,path=%s,server=on,wait=off "
                      "-global ingenic-tnpu.chardev=c0 "
                      "-global ingenic-tnpu.chardev1=c1", s0, s1);
    qtest_irq_intercept_out_named(qts, "/machine/soc/tnpu", "sysbus-irq");
    fake_connect(&c[0], s0);
    fake_connect(&c[1], s1);
    for (int n = 0; n < 2; n++) {
        fake_read(&c[n], CCU + 0x38);
        g_assert_cmpuint(c[n].run, ==, 0);
    }

    wr(qts, 0x2004, 0x150b0180);
    wr(qts, 0xf0, 0x80000000);
    g_assert_cmphex(fake_read(&c[1], CCU + 0x38), ==, 0x150);
    g_assert_cmpuint(c[1].run, ==, 1);
    g_assert_cmphex(c[1].run_addr, ==, 0x150b0180);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0);
    fake_send(&c[1], IRL_WFI, 0, 0, c[1].wakes);
    fake_read(&c[1], CCU + 0x38);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0x88);

    /* FROMHOST wakes it; a WFI from before that wake is stale */
    wr(qts, 0x2008, 1);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0);
    fake_send(&c[1], IRL_WFI, 0, 0, c[1].wakes);
    g_assert_cmphex(fake_read(&c[1], CCU + 0x2008), ==, 1);
    g_assert_cmpuint(c[1].irq3, ==, 1);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0);
    qtest_writel(qts, CCU + 0x2008, 0);
    fake_read(&c[1], CCU + 0x38);
    fake_send(&c[1], IRL_WFI, 0, 0, c[1].wakes);
    fake_read(&c[1], CCU + 0x38);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0x88);

    /* v0 released too: bit 3 waits for it */
    wr(qts, 0xf0, 0);
    fake_read(&c[0], CCU + 0x38);
    g_assert_cmpuint(c[0].run, ==, 1);
    g_assert_cmphex(c[0].run_addr, ==, 0x150b0000);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0x80);
    fake_send(&c[0], IRL_WFI, 0, 0, c[0].wakes);
    fake_read(&c[0], CCU + 0x38);
    g_assert_cmphex(rd(qts, 0x00) & 0xc8, ==, 0xc8);

    /* a task done: core 1 posts its TOHOST */
    g_assert_false(qtest_get_irq(qts, 0));
    fake_send(&c[1], IRL_WRITE, CCU + 0x200c, 4, 0x00000001);
    while (fake_recv(&c[1], &(IngenicRiscvMsg){ 0 }) != IRL_DATA) {
    }
    g_assert_true(qtest_get_irq(qts, 0));
    g_assert_cmphex(rd(qts, 0x200c), ==, 1);

    close(c[0].fd);
    close(c[1].fd);
    qtest_quit(qts);
    unlink(s0);
    unlink(s1);
    rmdir(dir);
}

/* The T32's window is RAM the driver loads the boot code into */
static void test_window(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_writel(qts, DRAM + 0xffc, 0xfedcba98);
    g_assert_cmphex(qtest_readl(qts, DRAM + 0xffc), ==, 0xfedcba98);
    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-tnpu/regs", test_regs);
    qtest_add_func("ingenic-tnpu/irq-and-release", test_irq_and_release);
    qtest_add_func("ingenic-tnpu/time", test_time);
    qtest_add_func("ingenic-tnpu/pro", test_pro);
    qtest_add_func("ingenic-tnpu/pro-link", test_pro_link);
    qtest_add_func("ingenic-tnpu/window", test_window);
    return g_test_run();
}
