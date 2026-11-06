/*
 * QTest testcase for the Ingenic TNNA (the T32's neural network accelerator)
 *
 * The host's view of its window, and, through a linked TNPU core (a
 * qemu-system-riscv32 -M ingenic-tnpu whose accesses outside DDR cross the
 * link as the core's), its registers, its DDR<->ORAM DMA and its colour
 * converter (CVA), as a T32LQ's TNPU core reads and drives them.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define TNNA            0x12500000
#define NNRST           0x00
#define NNRW            0x04
#define REGS            0x80
#define WFCW            0x240
#define DMA             0x12508000
#define FIN             0x00
#define FOUT            0x40
#define WBTIN           0x80
#define RUN             0x00
#define GREG0           0x04
#define GREG1           0x08
#define GREG2           0x0c
#define GREGSET0        0x10
#define EREG0L          0x20
#define EREG0H          0x24
#define EREG1L          0x28
#define EREG1H          0x2c

#define NNMAC           0x08
#define NNMMT           0x0c
#define INITCW          0x10
#define NNDWF           0x14
#define NNDWW           0x18
#define NNDWBT          0x1c
#define ENDSIG          0x24
#define ONNA            0x34

#define CVA             0x12b00000

#define PRO_NNA         0x15070000
#define PRO_DMA         0x15050000
#define PRO_ORAM        0x15100000

#define SRC             0x01810000
#define DST             0x01820000
#define ZEROS           0x01830000
#define WBT             0x03800000
#define PIC             0x03801000
#define OUT             0x03802000
#define PRE             0x03804000

typedef struct {
    QTestState *soc, *core;
    char *dir;
} Linked;

/* the T32 and its TNPU's core, linked; NULL without a riscv32 build */
static bool linked_init(Linked *l)
{
    g_autofree char *bin_dir = g_path_get_dirname(getenv("QTEST_QEMU_BINARY"));
    g_autofree char *rv = g_build_filename(bin_dir, "qemu-system-riscv32",
                                           NULL);
    g_autofree char *sock = NULL;
    g_autofree char *args = NULL;

    if (!g_file_test(rv, G_FILE_TEST_IS_EXECUTABLE)) {
        g_test_skip("no qemu-system-riscv32 to link");
        return false;
    }
    l->dir = g_dir_make_tmp("ingenic-tnna-XXXXXX", NULL);
    sock = g_build_filename(l->dir, "tnpu.sock", NULL);
    l->soc = qtest_initf("-machine ingenic-t32,memory-backend=ddr -m 128M "
                         "-object memory-backend-shm,id=ddr,size=128M "
                         "-chardev socket,id=tnpu,path=%s,server=on,wait=off "
                         "-global ingenic-tnpu.chardev=tnpu", sock);
    g_setenv("QTEST_QEMU_BINARY_TNPU", rv, true);
    args = g_strdup_printf("-machine ingenic-tnpu,socket=%s", sock);
    l->core = qtest_init_ext("QTEST_QEMU_BINARY_TNPU", args, NULL, true);
    return true;
}

static void linked_quit(Linked *l)
{
    g_autofree char *sock = g_build_filename(l->dir, "tnpu.sock", NULL);

    qtest_quit(l->core);
    qtest_quit(l->soc);
    unlink(sock);
    rmdir(l->dir);
    g_free(l->dir);
}

static void nnrw(Linked *l, uint32_t reg, uint32_t val)
{
    qtest_writel(l->core, TNNA + NNRW, reg << 20 | (val & 0xfffff));
}

static uint32_t reg(Linked *l, uint32_t reg)
{
    return qtest_readl(l->core, TNNA + REGS + (reg - 0x20) * 4);
}

static void dma(Linked *l, uint32_t ch, uint32_t off, uint32_t val)
{
    qtest_writel(l->core, DMA + ch + off, val);
}

static uint32_t dma_rd(Linked *l, uint32_t ch, uint32_t off)
{
    return qtest_readl(l->core, DMA + ch + off);
}

static void run(Linked *l, uint32_t ch)
{
    dma(l, ch, RUN, 1);
    qtest_readl(l->core, DMA + ch + RUN);
}

/* DDR words that hold their own address */
static void fill_addr(Linked *l, uint32_t base, int words)
{
    for (int i = 0; i < words; i++) {
        qtest_writel(l->soc, base + 4 * i, base + 4 * i);
    }
}

static void fill_zero(Linked *l, uint32_t base, int words)
{
    for (int i = 0; i < words; i++) {
        qtest_writel(l->soc, base + 4 * i, 0);
    }
}

/* DST + off holds 16 B from SRC + from, as words of their own address */
static void check_unit(Linked *l, uint32_t off, uint32_t from)
{
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(l->soc, DST + off + 4 * i), ==,
                        SRC + from + 4 * i);
    }
}

static void check_zero(Linked *l, uint32_t off)
{
    for (int i = 0; i < 4; i++) {
        g_assert_cmphex(qtest_readl(l->soc, DST + off + 4 * i), ==, 0);
    }
}

/* The host sees 384 KiB of RAM at the window, the ports not at all */
static void test_host_view(void)
{
    QTestState *qts = qtest_init("-machine ingenic-t32");

    qtest_writel(qts, TNNA, 0x12345678);
    qtest_writel(qts, TNNA + 0x8000, 0x9abcdef0);
    qtest_writel(qts, TNNA + 0x5fffc, 0x0badf00d);
    g_assert_cmphex(qtest_readl(qts, TNNA), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, TNNA + 0x8000), ==, 0x9abcdef0);
    g_assert_cmphex(qtest_readl(qts, TNNA + 0x5fffc), ==, 0x0badf00d);
    qtest_quit(qts);
}

static void test_regs(void)
{
    Linked l;

    if (!linked_init(&l)) {
        return;
    }
    /* resets, kept through NNRST */
    qtest_writel(l.core, TNNA + NNRST, 0);
    g_assert_cmphex(reg(&l, 0x20), ==, 0x5a);
    g_assert_cmphex(reg(&l, 0x23), ==, 0xffff);
    g_assert_cmphex(reg(&l, 0x29), ==, 0xfff);
    g_assert_cmphex(reg(&l, 0x2d), ==, 0x22);
    g_assert_cmphex(reg(&l, 0x49), ==, 0xf);
    g_assert_cmphex(reg(&l, 0x7f), ==, 0xfedc);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x04), ==, 0x14);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x44), ==, 0x14);

    /* the bits kept; an alias reads its register and keeps nothing */
    nnrw(&l, 0x20, 0xfffff);
    nnrw(&l, 0x21, 0xfffff);
    nnrw(&l, 0x2c, 0x123);
    nnrw(&l, 0x24, 0x456);
    g_assert_cmphex(reg(&l, 0x20), ==, 0x37f);
    g_assert_cmphex(reg(&l, 0x21), ==, 0x77);
    g_assert_cmphex(reg(&l, 0x2c), ==, 0x123);
    g_assert_cmphex(reg(&l, 0x24), ==, 0x123);
    nnrw(&l, 0x6a, 0xfffff);
    g_assert_cmphex(reg(&l, 0x6a), ==, 0x1fff);
    nnrw(&l, 0x40, 0xfffff);
    g_assert_cmphex(reg(&l, 0x40), ==, 0xfff);
    nnrw(&l, 0x52, 0x11);
    nnrw(&l, 0x5a, 0x22);
    g_assert_cmphex(reg(&l, 0x42), ==, 0x22);
    g_assert_cmphex(reg(&l, 0x52), ==, 0x22);

    /* FR_WADDR_ADD adds its signed value, in FR_WADDR's 11 bits */
    nnrw(&l, 0x50, 0x10);
    nnrw(&l, 0x51, 0xfffff);
    g_assert_cmphex(reg(&l, 0x50), ==, 0xf);
    nnrw(&l, 0x51, 0x800);
    g_assert_cmphex(reg(&l, 0x50), ==, 0xf);
    nnrw(&l, 0x51, 0x801);
    g_assert_cmphex(reg(&l, 0x53), ==, 0x10);

    /* WFCW_INP pushes at WFCW_WPTR, which steps on */
    nnrw(&l, 0x58, 7);
    nnrw(&l, 0x59, 0xfffff);
    nnrw(&l, 0x59, 0x1234);
    g_assert_cmphex(reg(&l, 0x58), ==, 1);
    g_assert_cmphex(qtest_readl(l.core, TNNA + WFCW + 7 * 4), ==, 0x7fff);
    g_assert_cmphex(qtest_readl(l.core, TNNA + WFCW + 0x20), ==, 0x1234);

    /* G 5 to 7 keep a write, 0 to 4 do not */
    nnrw(&l, 0x04, 0xfffff);
    nnrw(&l, 0x05, 0xfffff);
    nnrw(&l, 0x06, 0xfffff);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x10), ==, 0x7f);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x14), ==, 0xff);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x18), ==, 0x7);

    /* NNRST puts the registers back, WFCW_WPTR too; WFCW control fields 1 */
    qtest_writel(l.core, TNNA + NNRST, 0);
    g_assert_cmphex(reg(&l, 0x20), ==, 0x5a);
    g_assert_cmphex(reg(&l, 0x2c), ==, 0x7ff);
    g_assert_cmphex(reg(&l, 0x50), ==, 0);
    g_assert_cmphex(reg(&l, 0x58), ==, 0);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x14), ==, 0x7f);
    g_assert_cmphex(qtest_readl(l.core, TNNA + 0x18), ==, 0);
    g_assert_cmphex(qtest_readl(l.core, TNNA + WFCW), ==, 0xa34);
    g_assert_cmphex(qtest_readl(l.core, TNNA + WFCW + 7 * 4), ==, 0xfff);
    linked_quit(&l);
}

/* Fin: 2 planes 768 B apart, 3 lines 144 B apart, 32 B a line */
static void test_dma_fin(void)
{
    static const struct { uint32_t off, from; } units[] = {
        { 0x000, 0x000 }, { 0x010, 0x010 }, { 0x050, 0x090 }, { 0x060, 0x0a0 },
        { 0x0a0, 0x120 }, { 0x0b0, 0x130 }, { 0x140, 0x300 }, { 0x150, 0x310 },
        { 0x190, 0x390 }, { 0x1a0, 0x3a0 }, { 0x1e0, 0x420 }, { 0x1f0, 0x430 },
    };
    Linked l;
    int u = 0;

    if (!linked_init(&l)) {
        return;
    }
    fill_addr(&l, SRC, 512);
    fill_zero(&l, ZEROS, 256);
    fill_zero(&l, DST, 256);
    dma(&l, FIN, EREG0L, ZEROS | 1);
    dma(&l, FIN, EREG0H, 0x00400100);
    run(&l, FIN);
    dma(&l, FIN, GREG0, 0x01000030);
    dma(&l, FIN, GREG1, 0x00000009);
    dma(&l, FIN, GREG2, 0x00050014);
    dma(&l, FIN, EREG0L, SRC | 1);
    dma(&l, FIN, EREG0H, 0x20020100);
    run(&l, FIN);
    g_assert_cmphex(dma_rd(&l, FIN, EREG0L), ==, SRC | 1);
    g_assert_cmphex(dma_rd(&l, FIN, EREG0H), ==, 0x20020123);
    dma(&l, FOUT, EREG0L, DST);
    dma(&l, FOUT, EREG0H, 0x00400100);
    run(&l, FOUT);
    g_assert_cmphex(dma_rd(&l, FOUT, EREG0L), ==, DST | 1);
    for (uint32_t off = 0; off < 0x400; off += 16) {
        if (u < ARRAY_SIZE(units) && units[u].off == off) {
            check_unit(&l, off, units[u++].from);
        } else {
            check_zero(&l, off);
        }
    }
    linked_quit(&l);
}

/* Fout: 2 planes 512 B and 3 lines 96 B apart in DDR, 16 B a line */
static void test_dma_fout(void)
{
    Linked l;

    if (!linked_init(&l)) {
        return;
    }
    fill_addr(&l, SRC, 256);
    fill_zero(&l, DST, 512);
    dma(&l, FIN, EREG0L, SRC | 1);
    dma(&l, FIN, EREG0H, 0x00400100);
    run(&l, FIN);
    dma(&l, FOUT, GREG0, 0x01000020);
    dma(&l, FOUT, GREG1, 0x00000006);
    dma(&l, FOUT, GREG2, 0x00020008);
    dma(&l, FOUT, EREG0L, DST);
    dma(&l, FOUT, EREG0H, 0x20010100);
    run(&l, FOUT);
    g_assert_cmphex(dma_rd(&l, FOUT, EREG0L), ==, (DST + 0x400) | 1);
    g_assert_cmphex(dma_rd(&l, FOUT, EREG0H), ==, 0x20010110);
    check_unit(&l, 0x000, 0x000);
    check_unit(&l, 0x060, 0x020);
    check_unit(&l, 0x0c0, 0x040);
    check_unit(&l, 0x200, 0x080);
    check_unit(&l, 0x260, 0x0a0);
    check_unit(&l, 0x2c0, 0x0c0);
    check_zero(&l, 0x010);
    check_zero(&l, 0x300);
    linked_quit(&l);
}

/* Fin with 2 sets and 2 entries; WBTin with 2 entries */
static void test_dma_sets(void)
{
    Linked l;

    if (!linked_init(&l)) {
        return;
    }
    fill_addr(&l, SRC, 1024);
    fill_zero(&l, ZEROS, 1024);
    fill_zero(&l, DST, 1024);
    dma(&l, FIN, EREG0L, ZEROS | 1);
    dma(&l, FIN, EREG0H, 0x01000100);
    run(&l, FIN);
    dma(&l, FIN, GREG0, 0x00000010);
    dma(&l, FIN, GREG1, 0x01000000);
    dma(&l, FIN, GREG2, 0x00000020);
    dma(&l, FIN, GREGSET0, 0x01000040);
    dma(&l, FIN, EREG0L, SRC);
    dma(&l, FIN, EREG0H, 0x00010100);
    dma(&l, FIN, EREG1L, SRC + 0x81);
    dma(&l, FIN, EREG1H, 0x00010108);
    run(&l, FIN);
    g_assert_cmphex(dma_rd(&l, FIN, EREG0L), ==, SRC + 0x400);
    g_assert_cmphex(dma_rd(&l, FIN, EREG0H), ==, 0x10120);
    g_assert_cmphex(dma_rd(&l, FIN, EREG1L), ==, SRC + 0x481);
    g_assert_cmphex(dma_rd(&l, FIN, EREG1H), ==, 0x10128);
    dma(&l, WBTIN, EREG0L, SRC + 0x400);
    dma(&l, WBTIN, EREG0H, 0x00030180);
    dma(&l, WBTIN, EREG1L, SRC + 0x801);
    dma(&l, WBTIN, EREG1H, 0x000101c0);
    run(&l, WBTIN);
    g_assert_cmphex(dma_rd(&l, WBTIN, EREG0H), ==, 0x30183);
    g_assert_cmphex(dma_rd(&l, WBTIN, EREG1H), ==, 0x101c1);
    dma(&l, FOUT, EREG0L, DST);
    dma(&l, FOUT, EREG0H, 0x01000100);
    run(&l, FOUT);
    check_unit(&l, 0x000, 0x400);
    check_unit(&l, 0x080, 0x480);
    check_unit(&l, 0x200, 0x500);
    check_unit(&l, 0x280, 0x580);
    check_unit(&l, 0x800, 0x400);
    check_unit(&l, 0x810, 0x410);
    check_unit(&l, 0x820, 0x420);
    check_unit(&l, 0xc00, 0x800);
    check_zero(&l, 0x010);
    check_zero(&l, 0x830);
    linked_quit(&l);
}

/*
 * Diffs and strides near 2^24 units step back through DDR: Fin of 2
 * planes 768 B and 3 lines 144 B down from SRC + 0x800, then Fout of 2
 * planes 512 B and 3 lines 96 B down from DST + 0x1800, whose address
 * moves on past bit 28 (the DMA's DDR has 28 address bits)
 */
static void test_dma_back(void)
{
    static const struct { uint32_t off, from; } units[] = {
        { 0x000, 0x800 }, { 0x010, 0x810 }, { 0x050, 0x770 }, { 0x060, 0x780 },
        { 0x0a0, 0x6e0 }, { 0x0b0, 0x6f0 }, { 0x140, 0x500 }, { 0x150, 0x510 },
        { 0x190, 0x470 }, { 0x1a0, 0x480 }, { 0x1e0, 0x3e0 }, { 0x1f0, 0x3f0 },
        { 0x1540, 0x3e0 }, { 0x15a0, 0x470 }, { 0x1600, 0x500 },
        { 0x1740, 0x6e0 }, { 0x17a0, 0x770 }, { 0x1800, 0x800 },
    };
    Linked l;
    int u = 0;

    if (!linked_init(&l)) {
        return;
    }
    fill_addr(&l, SRC, 768);
    fill_zero(&l, ZEROS, 256);
    fill_zero(&l, DST, 1664);
    dma(&l, FIN, EREG0L, ZEROS | 1);
    dma(&l, FIN, EREG0H, 0x00400100);
    run(&l, FIN);
    dma(&l, FIN, GREG0, 0x01ffffd0);
    dma(&l, FIN, GREG1, 0x00fffff7);
    dma(&l, FIN, GREG2, 0x00050014);
    dma(&l, FIN, EREG0L, (SRC + 0x800) | 1);
    dma(&l, FIN, EREG0H, 0x20020100);
    run(&l, FIN);
    g_assert_cmphex(dma_rd(&l, FIN, EREG0L), ==, (SRC + 0x800) | 1);
    g_assert_cmphex(dma_rd(&l, FIN, EREG0H), ==, 0x20020123);
    dma(&l, FOUT, EREG0L, DST);
    dma(&l, FOUT, EREG0H, 0x00400100);
    run(&l, FOUT);
    dma(&l, FOUT, GREG0, 0x01ffffe0);
    dma(&l, FOUT, GREG1, 0x00fffffa);
    dma(&l, FOUT, GREG2, 0x00050014);
    dma(&l, FOUT, EREG0L, DST + 0x1800);
    dma(&l, FOUT, EREG0H, 0x20010100);
    run(&l, FOUT);
    g_assert_cmphex(dma_rd(&l, FOUT, EREG0L), ==, (DST + 0x20001400) | 1);
    g_assert_cmphex(dma_rd(&l, FOUT, EREG0H), ==, 0x20010128);
    for (uint32_t off = 0; off < 0x1840; off += 16) {
        if (u < ARRAY_SIZE(units) && units[u].off == off) {
            check_unit(&l, off, units[u++].from);
        } else if (off < 0x400 || off >= 0x1400) {
            check_zero(&l, off);
        }
    }
    linked_quit(&l);
}

/* The pixels a T32LQ's CVA makes of 32x2 NV12, R first, as in cva_run() */
static const uint32_t cva_bt601[64] = {
    0x000000b8, 0x000000c1, 0x000000b1, 0x000000ba, 0x000000aa, 0x000000b4,
    0x000004a3, 0x00000ead, 0x00001e9c, 0x000027a6, 0x00003796, 0x0000409f,
    0x001d508f, 0x00265a98, 0x004f6a88, 0x00597391, 0x00828381, 0x008c8c8a,
    0x00b59d7a, 0x00bfa683, 0x00e8b673, 0x00f2bf7c, 0x00ffcf6c, 0x00ffd975,
    0x00ffe965, 0x00fff26e, 0x00ffff5e, 0x00ffff68, 0x00ffff57, 0x00ffff61,
    0x00ffff50, 0x00ffff5a, 0x0014e1ff, 0x000bd8ff, 0x0022d5ff, 0x0018ccff,
    0x002fc9ff, 0x0026c0ff, 0x003dbdff, 0x0034b4ff, 0x004bb2ff, 0x0041a8ff,
    0x0058a6ff, 0x004f9cfb, 0x00669ad8, 0x005d90cf, 0x00748eac, 0x006a85a2,
    0x00818280, 0x00787976, 0x008f7653, 0x00866d4a, 0x009d6a27, 0x0093611e,
    0x00aa5e00, 0x00a15500, 0x00b85300, 0x00af4900, 0x00c64700, 0x00bc3d00,
    0x00d33b00, 0x00ca3100, 0x00e12f00, 0x00d82600,
};

/* the same with halves and quarters, biases 0.5 - 2^-20, 0.5 and -3, V = U */
static const uint32_t cva_round[64] = {
    0x00000100, 0x00010504, 0x00090f0c, 0x000d1310, 0x00151d18, 0x0019211c,
    0x00212b24, 0x00252f28, 0x002d3930, 0x00313d34, 0x0039473c, 0x003d4b40,
    0x00455548, 0x0049594c, 0x00516354, 0x00556758, 0x005d7160, 0x00617564,
    0x00697f6c, 0x006d8370, 0x00758d78, 0x0079917c, 0x00819b84, 0x00859f88,
    0x008da990, 0x0091ad94, 0x0099b79c, 0x009dbba0, 0x00a5c5a8, 0x00a9c9ac,
    0x00b1d3b4, 0x00b5d7b8, 0x007d8080, 0x00797c7c, 0x00797e7c, 0x00757a78,
    0x00757c78, 0x00717874, 0x00717a74, 0x006d7670, 0x006d7870, 0x0069746c,
    0x0069766c, 0x00657268, 0x00657468, 0x00617064, 0x00617264, 0x005d6e60,
    0x005d7060, 0x00596c5c, 0x00596e5c, 0x00556a58, 0x00556c58, 0x00516854,
    0x00516a54, 0x004d6650, 0x004d6850, 0x0049644c, 0x0049664c, 0x00456248,
    0x00456448, 0x00416044, 0x00416244, 0x003d5e40,
};

/*
 * 32x2 NV12 (Y a ramp up then down, U a ramp, V 255 - U or U) into the
 * ORAM at 0x1000, through the CVA to 0x2000, and out to DST
 */
static void cva_case(const uint32_t *coef, const uint32_t *bias, uint32_t ctrl,
                     bool v_is_u, const uint32_t *want)
{
    Linked l;
    uint8_t in[96];

    if (!linked_init(&l)) {
        return;
    }
    for (int i = 0; i < 32; i++) {
        in[i] = i * 8;
        in[32 + i] = 255 - i * 8;
    }
    for (int i = 0; i < 16; i++) {
        in[64 + 2 * i] = i * 16;
        in[65 + 2 * i] = v_is_u ? i * 16 : 255 - i * 16;
    }
    qtest_memwrite(l.soc, SRC, in, sizeof(in));
    fill_zero(&l, DST, 64);
    dma(&l, FIN, EREG0L, SRC | 1);
    dma(&l, FIN, EREG0H, 0x00080100);
    run(&l, FIN);
    qtest_writel(l.core, CVA + 0x00, 2);
    qtest_writel(l.core, CVA + 0x1c, 0x00800020);
    for (int i = 0; i < 7; i++) {
        qtest_writel(l.core, CVA + 0x24 + 4 * i, coef[i]);
    }
    qtest_writel(l.core, CVA + 0x40, ctrl);
    for (int i = 0; i < 3; i++) {
        qtest_writel(l.core, CVA + 0x44 + 4 * i, bias[i]);
    }
    qtest_writel(l.core, CVA + 0x10, 0x1000);
    qtest_writel(l.core, CVA + 0x14, 0x1040);
    qtest_writel(l.core, CVA + 0x18, 0x2000);
    qtest_writel(l.core, CVA + 0x20, 0x00020020);
    qtest_writel(l.core, CVA + 0x00, 1);
    g_assert_cmphex(qtest_readl(l.core, CVA + 0x00), ==, 0);
    g_assert_cmphex(qtest_readl(l.core, CVA + 0x04), ==, 1);
    dma(&l, FOUT, EREG0L, DST);
    dma(&l, FOUT, EREG0H, 0x00100200);
    run(&l, FOUT);
    for (int i = 0; i < 64; i++) {
        g_assert_cmphex(qtest_readl(l.soc, DST + 4 * i), ==, want[i]);
    }
    linked_quit(&l);
}

/* the vendor's BT.601, R first */
static void test_cva_bt601(void)
{
    static const uint32_t coef[7] = {
        0x00129fbe, 0x002049ba, 0x00129fbe, 0xfff9be77, 0xfff2fdf4,
        0x00129fbe, 0x00198937,
    };
    static const uint32_t bias[3] = { 0xeeb12720, 0x0877cea0, 0xf21168a0 };

    cva_case(coef, bias, 0x01000010, false, cva_bt601);
}

/* rounding half up, and B first with bit 24 clear */
static void test_cva_round(void)
{
    static const uint32_t coef[7] = {
        0x80000, 0x40000, 0x80000, 0x40000, 0x20000, 0x80000, 0x40000,
    };
    static const uint32_t bias[3] = { 0x7ffff, 0x80000, 0xffd00000 };

    cva_case(coef, bias, 0, true, cva_round);
}

typedef struct {
    uint32_t reg, val;
} RegVal;

/*
 * A one-tile convolution: weights (an LCG's bytes) then B and T into the
 * ORAM at 0, the picture (the LCG's too, its planes one after the other)
 * at unit 0x100, one NNDWF, one NNMAC from a pixel left of the picture,
 * one NNMMT that quantizes into unit 0x200 (and another RDBT and QSUM to
 * OR_WADDR_Q or2), and out to DDR. The bytes a T32LQ's TNNA wrote,
 * running the same.
 */
typedef struct {
    uint32_t w_size, w_seed, pic_seed;
    uint32_t planes, pic_rows, pic_units, out_rows, out_units;
    int32_t b[8], t[8];
    const uint32_t *bt;                 /* in place of b and t */
    uint32_t bt_size;
    uint32_t or2;
    const RegVal *regs;
    int nregs;
    const uint32_t *wfcw;
    int nwfcw;
    uint32_t fr_raddr;
    const uint8_t *want;
    int want_len;
    const uint32_t *pre;                /* words Fin'd into ORAM unit 0x300 */
    uint32_t pre_words;
    const RegVal *mmt_regs;             /* written just before the NNMMT */
    int nmmt_regs;
    uint32_t out_unit;                  /* Fout'd from, 0x200 if 0 */
    const RegVal *after;                /* registers as they read after */
    int nafter;
} ConvCase;

static void lcg_fill(uint8_t *buf, size_t n, uint32_t x)
{
    for (size_t i = 0; i < n; i++) {
        x = x * 1103515245 + 12345;
        buf[i] = x >> 16;
    }
}

static void port(Linked *l, uint32_t off, uint32_t val)
{
    qtest_writel(l->core, TNNA + off, val);
}

static void conv_case(const ConvCase *c)
{
    Linked l;
    uint32_t planes = MAX(c->planes, 1), bt_size = c->bt ? c->bt_size : 64;
    uint32_t plane = planes > 1 ? c->pic_rows * c->pic_units : 0;
    uint8_t buf[2048], got[512];

    if (!linked_init(&l)) {
        return;
    }
    lcg_fill(buf, c->w_size, c->w_seed);
    for (int i = 0; i < bt_size / 4; i++) {
        stl_le_p(&buf[c->w_size + 4 * i], c->bt ? c->bt[i] :
                 i < 8 ? c->b[i] : c->t[i - 8]);
    }
    qtest_memwrite(l.soc, WBT, buf, c->w_size + bt_size);
    lcg_fill(buf, planes * c->pic_rows * c->pic_units * 16, c->pic_seed);
    qtest_memwrite(l.soc, PIC, buf, planes * c->pic_rows * c->pic_units * 16);
    fill_zero(&l, OUT, sizeof(got) / 4);

    qtest_writel(l.core, TNNA + NNRST, 0);
    dma(&l, WBTIN, EREG0L, WBT | 1);
    dma(&l, WBTIN, EREG0H, (c->w_size + bt_size) / 16 << 16);
    run(&l, WBTIN);
    dma(&l, FIN, GREG0, (planes - 1) << 24 | plane);
    dma(&l, FIN, GREG1, c->pic_units);
    dma(&l, FIN, GREG2, c->pic_units << 16 | plane);
    dma(&l, FIN, EREG0L, PIC | 1);
    dma(&l, FIN, EREG0H, (c->pic_rows - 1) << 28 | c->pic_units << 16 | 0x100);
    run(&l, FIN);
    if (c->pre) {
        for (int i = 0; i < c->pre_words; i++) {
            stl_le_p(&buf[4 * i], c->pre[i]);
        }
        qtest_memwrite(l.soc, PRE, buf, 4 * c->pre_words);
        dma(&l, FIN, GREG0, 0);
        dma(&l, FIN, GREG1, c->pre_words / 4);
        dma(&l, FIN, GREG2, c->pre_words / 4 << 16);
        dma(&l, FIN, EREG0L, PRE | 1);
        dma(&l, FIN, EREG0H, c->pre_words / 4 << 16 | 0x300);
        run(&l, FIN);
    }
    for (int i = 0; i < c->nregs; i++) {
        nnrw(&l, c->regs[i].reg, c->regs[i].val);
    }
    for (int i = 0; i < c->nwfcw; i++) {
        nnrw(&l, 0x59, c->wfcw[i]);
    }
    nnrw(&l, 0x5a, c->nwfcw);
    port(&l, INITCW, 0);
    nnrw(&l, 0x6a, 0x400);
    nnrw(&l, 0x5c, 0);
    port(&l, NNDWW, c->w_size << 16);
    nnrw(&l, 0x5e, 0);
    port(&l, NNDWBT, bt_size << 16 | c->w_size / 16);
    nnrw(&l, 0x40, 0x100);
    nnrw(&l, 0x50, 0);
    port(&l, NNDWF, 0);
    nnrw(&l, 0x34, c->fr_raddr);
    nnrw(&l, 0x29, 0);
    nnrw(&l, 0x2b, 0xfffff);
    port(&l, NNMAC, 0);
    for (int i = 0; i < c->nmmt_regs; i++) {
        nnrw(&l, c->mmt_regs[i].reg, c->mmt_regs[i].val);
    }
    port(&l, NNMMT, 0xf000);
    if (c->or2) {
        nnrw(&l, 0x6a, c->or2);
        port(&l, NNMMT, 0xc000);
    }
    dma(&l, FOUT, GREG0, 0);
    dma(&l, FOUT, GREG1, c->out_units);
    dma(&l, FOUT, GREG2, c->out_units << 16);
    dma(&l, FOUT, EREG0L, OUT);
    dma(&l, FOUT, EREG0H,
        (c->out_rows - 1) << 28 | c->out_units << 16 |
        (c->out_unit ? c->out_unit : 0x200));
    port(&l, ENDSIG, 0);
    dma(&l, FOUT, ONNA, 1);
    qtest_readl(l.core, DMA + FOUT + RUN);
    qtest_memread(l.soc, OUT, got, sizeof(got));
    for (int i = 0; i < sizeof(got); i++) {
        g_assert_cmphex(got[i], ==, i < c->want_len ? c->want[i] : 0);
    }
    for (int i = 0; i < c->nafter; i++) {
        g_assert_cmphex(qtest_readl(l.core, TNNA + REGS +
                                    4 * (c->after[i].reg - 0x20)),
                        ==, c->after[i].val);
    }
    linked_quit(&l);
}

/* 3x3, stride 1, 8 to 8 channels, all 8-bit, an 8x4 picture to 8x2 */
static void test_conv_3x3(void)
{
    static const RegVal regs[] = {
        { 0x20, 0x5e }, { 0x21, 0x22 }, { 0x2c, 3 }, { 0x2e, 7 },
        { 0x2d, 0x13 }, { 0x28, 0x11 }, { 0x30, 4 }, { 0x31, 4 },
        { 0x32, 10 }, { 0x33, 0x3b }, { 0x60, 0x10 }, { 0x74, 1 },
        { 0x5b, 0x14 }, { 0x41, 4 }, { 0x48, 0x32 }, { 0x4a, 1 },
        { 0x4e, 0 },
    };
    static const uint32_t wfcw[] = { 0x7a00, 0x2001, 0x10 };
    static const uint8_t want[128] = {
        0x8e, 0x91, 0x73, 0x36, 0x2c, 0x78, 0x41, 0x43,
        0x00, 0x75, 0x35, 0x8f, 0x41, 0xb9, 0x74, 0x56,
        0x72, 0x5a, 0x82, 0x73, 0x28, 0x83, 0x47, 0x27,
        0x30, 0x3e, 0x32, 0x82, 0x48, 0x64, 0x7c, 0x25,
        0x7b, 0x97, 0x31, 0x50, 0x4c, 0x7d, 0x37, 0x12,
        0x00, 0x0e, 0x6b, 0x9d, 0x6e, 0xab, 0x48, 0x26,
        0x5d, 0x38, 0x42, 0x7b, 0x41, 0xa4, 0x2b, 0x24,
        0x59, 0x68, 0x59, 0x98, 0x75, 0x95, 0x53, 0x2a,
        0x74, 0x39, 0x5d, 0x76, 0x43, 0x56, 0x66, 0x32,
        0x25, 0x5f, 0x6e, 0x73, 0x6b, 0x89, 0x4b, 0x42,
        0x48, 0x60, 0x47, 0x8a, 0x62, 0xbe, 0x44, 0x30,
        0x4d, 0x87, 0x58, 0x84, 0x61, 0x67, 0x4e, 0x28,
        0x00, 0x4f, 0x82, 0x90, 0x55, 0x6c, 0x49, 0x16,
        0x17, 0x56, 0x67, 0x63, 0x68, 0x7e, 0x14, 0x17,
        0x25, 0x76, 0x6b, 0xb6, 0x2d, 0xa2, 0x52, 0x0e,
        0x09, 0x62, 0x8c, 0x66, 0x3b, 0xa9, 0x4f, 0x37,
    };
    static const ConvCase c = {
        .w_size = 576, .w_seed = 1, .pic_seed = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 2, .out_units = 4,
        .b = { 34470, -13262, -60482, 22348, -25384, 57444, 10224, -37506 },
        .t = { 80000, 85000, 90000, 95000, 100000, 105000, 110000, 115000 },
        .regs = regs, .nregs = ARRAY_SIZE(regs),
        .wfcw = wfcw, .nwfcw = ARRAY_SIZE(wfcw),
        .fr_raddr = 0x1fdb, .want = want, .want_len = sizeof(want),
    };

    conv_case(&c);
}

/*
 * 3x3, stride 2 (MAC_P_MASK keeps the even lanes, PIXEL_OUT packs them),
 * all 4-bit, a 16x3 picture to 8x1
 */
static void test_conv_3x3_s2_4bit(void)
{
    static const RegVal regs[] = {
        { 0x20, 0x15e }, { 0x21, 0x22 }, { 0x23, 0x5555 }, { 0x2c, 2 },
        { 0x2e, 15 }, { 0x2d, 0x04 }, { 0x28, 0 }, { 0x30, 3 },
        { 0x31, 3 }, { 0x32, 5 }, { 0x33, 0x13 }, { 0x60, 0 },
        { 0x74, 1 }, { 0x7c, 0x6420 }, { 0x7d, 0xeca8 }, { 0x5b, 0x20 },
        { 0x41, 4 }, { 0x48, 0x22 }, { 0x4a, 1 }, { 0x4e, 0 },
    };
    static const uint32_t wfcw[] = { 0x1801, 0x11 };
    static const uint8_t want[32] = {
        0xd0, 0xb2, 0xbe, 0x77, 0x50, 0x80, 0x66, 0x87,
        0x60, 0x80, 0x27, 0x66, 0x20, 0xd3, 0xc6, 0x75,
        0x70, 0xb0, 0xa6, 0x46, 0x50, 0x70, 0x77, 0x55,
        0x50, 0x80, 0xa7, 0x49, 0x30, 0x60, 0xa6, 0x67,
    };
    static const ConvCase c = {
        .w_size = 288, .w_seed = 4, .pic_seed = 5,
        .pic_rows = 3, .pic_units = 4, .out_rows = 1, .out_units = 2,
        .b = { 306, 248, 190, 388, 331, 273, 215, 158 },
        .t = { 180, 200, 220, 240, 260, 280, 300, 320 },
        .regs = regs, .nregs = ARRAY_SIZE(regs),
        .wfcw = wfcw, .nwfcw = ARRAY_SIZE(wfcw),
        .fr_raddr = 0x1fef, .want = want, .want_len = sizeof(want),
    };

    conv_case(&c);
}

/*
 * 3x3, stride 1, 16 to 8 channels (2 planes), all 8-bit: the second
 * plane's taps follow the first's, odd rows of each right to left
 */
static void test_conv_3x3_2planes(void)
{
    static const RegVal regs[] = {
        { 0x20, 0x5e }, { 0x21, 0x22 }, { 0x22, 1 }, { 0x2c, 3 },
        { 0x2e, 7 }, { 0x2d, 0x13 }, { 0x28, 0x11 }, { 0x30, 4 },
        { 0x31, 8 }, { 0x32, 18 }, { 0x33, 107 }, { 0x46, 35 },
        { 0x60, 0x10 }, { 0x74, 1 }, { 0x5b, 0x14 }, { 0x41, 4 },
        { 0x48, 0x132 }, { 0x4a, 1 }, { 0x4e, 1 },
    };
    static const uint32_t wfcw[] = { 0x7a01, 0x2001, 28 };
    static const uint8_t want[128] = {
        0x5b, 0xac, 0x41, 0x5d, 0x6e, 0x86, 0x80, 0xba,
        0x9f, 0xa8, 0x63, 0x60, 0x5b, 0x86, 0x90, 0xa9,
        0x71, 0xd5, 0x9f, 0x8d, 0x61, 0x95, 0x86, 0xbc,
        0x7b, 0x91, 0x5f, 0x78, 0x62, 0x83, 0xac, 0xb7,
        0x9e, 0x9b, 0x7e, 0x71, 0x80, 0x93, 0x91, 0xd4,
        0x90, 0xa8, 0x7c, 0x79, 0x52, 0x80, 0x7b, 0xb2,
        0x69, 0x84, 0x6b, 0x65, 0x78, 0x83, 0xa7, 0xe0,
        0x74, 0x6b, 0x78, 0x77, 0x8e, 0x92, 0x80, 0x95,
        0xa0, 0xb9, 0x40, 0x71, 0x73, 0x93, 0x91, 0x93,
        0x6d, 0xc5, 0x66, 0x6e, 0x70, 0xad, 0x98, 0xf3,
        0x62, 0x9d, 0x6b, 0x69, 0x8a, 0xa1, 0x8a, 0xcb,
        0x8c, 0x97, 0x89, 0x6a, 0x7a, 0x8c, 0x93, 0xdb,
        0x9b, 0x97, 0x73, 0x4a, 0x8a, 0x8d, 0x7b, 0xdc,
        0x92, 0xb4, 0x84, 0x63, 0x7d, 0x85, 0x8b, 0xd4,
        0x87, 0xa6, 0x7a, 0x7c, 0x79, 0x94, 0x74, 0xd6,
        0x88, 0x81, 0x9e, 0x91, 0x77, 0xb2, 0x73, 0xb0,
    };
    static const ConvCase c = {
        .w_size = 1152, .w_seed = 7, .pic_seed = 8, .planes = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 2, .out_units = 4,
        .b = { 10224, -37506, 45322, -2410, -50652, 33200, -14532, -62774 },
        .t = { 160000, 170000, 180000, 190000, 200000, 210000, 220000, 230000 },
        .regs = regs, .nregs = ARRAY_SIZE(regs),
        .wfcw = wfcw, .nwfcw = ARRAY_SIZE(wfcw),
        .fr_raddr = 0x1fbb, .want = want, .want_len = sizeof(want),
    };

    conv_case(&c);
}

/*
 * conv-3x3 depthwise (MAC_CFG bit 0): each output channel its own input
 * channel's 3x3, the weights a stream of nibbles per channel (96 bytes)
 */
static void test_conv_3x3_dw(void)
{
    static const RegVal regs[] = {
        { 0x20, 0x5f }, { 0x21, 0x22 }, { 0x2c, 3 }, { 0x2e, 7 },
        { 0x2d, 0x13 }, { 0x28, 0x11 }, { 0x30, 4 }, { 0x31, 4 },
        { 0x32, 10 }, { 0x33, 0x3b }, { 0x60, 0x10 }, { 0x74, 1 },
        { 0x5b, 0x14 }, { 0x41, 4 }, { 0x48, 0x32 }, { 0x4a, 1 },
        { 0x4e, 0 },
    };
    static const uint32_t wfcw[] = { 0x7a00, 0x2001, 0x10 };
    static const uint8_t want[128] = {
        0x49, 0xaa, 0x42, 0x00, 0xc3, 0x77, 0x6e, 0x88,
        0x00, 0xa2, 0x00, 0x57, 0x99, 0x00, 0xe2, 0x2f,
        0x86, 0xff, 0x0a, 0x00, 0xa2, 0x52, 0xe7, 0x19,
        0x59, 0xff, 0x00, 0x00, 0x84, 0x00, 0xff, 0x78,
        0x07, 0xff, 0x21, 0x00, 0xaa, 0x00, 0xb0, 0x7c,
        0x0f, 0xc8, 0x2d, 0x00, 0x3e, 0x00, 0xbf, 0x75,
        0x00, 0xc4, 0x26, 0x00, 0x5d, 0x00, 0xff, 0x2e,
        0x00, 0x6b, 0x2e, 0x00, 0x21, 0x00, 0xc4, 0xfc,
        0x86, 0xe3, 0x5b, 0x30, 0xb9, 0x7f, 0x58, 0x42,
        0x00, 0xf6, 0x00, 0x00, 0x70, 0x00, 0xc0, 0x7f,
        0x00, 0xf9, 0x00, 0x00, 0x5b, 0x24, 0xff, 0x43,
        0x57, 0xdc, 0x12, 0x00, 0xa1, 0x00, 0xc3, 0x3f,
        0x54, 0xef, 0x00, 0x00, 0x81, 0x00, 0xff, 0xcf,
        0x1e, 0xff, 0x00, 0x00, 0x8e, 0x00, 0xbf, 0x4d,
        0x0e, 0xf5, 0x00, 0x00, 0x2b, 0x00, 0xce, 0x7d,
        0x11, 0xf5, 0x0c, 0x00, 0x6d, 0x2b, 0xf6, 0xba,
    };
    static const ConvCase c = {
        .w_size = 96, .w_seed = 11, .pic_seed = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 2, .out_units = 4,
        .t = { 20000, 20000, 20000, 20000, 20000, 20000, 20000, 20000 },
        .regs = regs, .nregs = ARRAY_SIZE(regs),
        .wfcw = wfcw, .nwfcw = ARRAY_SIZE(wfcw),
        .fr_raddr = 0x1fdb, .want = want, .want_len = sizeof(want),
    };

    conv_case(&c);
}

/*
 * conv-3x3 quantized 1B2T (Q_CFG bit 6, OF_OFFSET 128) twice: RDBT takes
 * B, T and TN and steps 3 entries on, the TNs in each packing
 */
static const uint8_t conv_1b2t_want[256] = {
    0x8f, 0x91, 0x90, 0x67, 0x76, 0x81, 0x7f, 0xac,
    0x00, 0x76, 0xe0, 0x90, 0x78, 0xba, 0x80, 0x9e,
    0x6e, 0x5d, 0x83, 0x7c, 0x75, 0x83, 0x7f, 0xbf,
    0x15, 0x43, 0xe3, 0x83, 0x79, 0x82, 0x80, 0xbf,
    0x7b, 0x98, 0xe5, 0x70, 0x7a, 0x80, 0x7e, 0xbf,
    0x00, 0x15, 0x9b, 0x9d, 0x7e, 0xac, 0x7f, 0xbf,
    0x52, 0x3c, 0xcf, 0x7e, 0x78, 0xa4, 0x7e, 0xbf,
    0x4c, 0x6a, 0xb1, 0x98, 0x7f, 0x96, 0x7f, 0xbd,
    0x71, 0x3d, 0xac, 0x7d, 0x78, 0x83, 0x7f, 0xb8,
    0x07, 0x61, 0x97, 0x7c, 0x7d, 0x8a, 0x7f, 0xac,
    0x36, 0x62, 0xc9, 0x8a, 0x7c, 0xbe, 0x7f, 0xb9,
    0x3d, 0x87, 0xb3, 0x84, 0x7c, 0x82, 0x7f, 0xbf,
    0x00, 0x52, 0x83, 0x90, 0x7b, 0x82, 0x7f, 0xbf,
    0x00, 0x59, 0xa0, 0x76, 0x7d, 0x80, 0x7e, 0xbf,
    0x07, 0x77, 0x9a, 0xb6, 0x76, 0xa2, 0x7f, 0xbf,
    0x00, 0x64, 0x8c, 0x77, 0x77, 0xa9, 0x7f, 0xb4,
    0x74, 0x89, 0x8a, 0x7e, 0x7f, 0x31, 0x88, 0x22,
    0x00, 0x45, 0xff, 0x80, 0x80, 0x9f, 0x83, 0x40,
    0x5d, 0x00, 0x8c, 0x7f, 0x7f, 0x49, 0x88, 0x00,
    0x28, 0x00, 0xff, 0x80, 0x80, 0x07, 0x82, 0x00,
    0x65, 0x92, 0xff, 0x7f, 0x80, 0x3d, 0x89, 0x00,
    0x00, 0x00, 0x9b, 0x89, 0x80, 0x8f, 0x87, 0x00,
    0x4d, 0x00, 0xec, 0x80, 0x80, 0x87, 0x8b, 0x00,
    0x49, 0x20, 0xbe, 0x84, 0x80, 0x6f, 0x86, 0x00,
    0x5f, 0x00, 0xb6, 0x7f, 0x80, 0x00, 0x84, 0x08,
    0x1f, 0x06, 0x95, 0x7f, 0x80, 0x57, 0x87, 0x20,
    0x3c, 0x0a, 0xe3, 0x80, 0x80, 0xa4, 0x88, 0x05,
    0x40, 0x79, 0xc1, 0x80, 0x80, 0x0e, 0x87, 0x00,
    0x00, 0x00, 0x8c, 0x80, 0x80, 0x19, 0x87, 0x00,
    0x15, 0x00, 0xa3, 0x7f, 0x80, 0x3f, 0x8d, 0x00,
    0x1f, 0x49, 0x9a, 0xa7, 0x7f, 0x85, 0x86, 0x00,
    0x09, 0x10, 0x98, 0x7f, 0x80, 0x8d, 0x87, 0x0f,
};

static void conv_1b2t(const uint32_t *bt, size_t bt_size)
{
    static const RegVal regs[] = {
        { 0x20, 0x5e }, { 0x21, 0x22 }, { 0x2c, 3 }, { 0x2e, 7 },
        { 0x2d, 0x13 }, { 0x28, 0x11 }, { 0x30, 4 }, { 0x31, 4 },
        { 0x32, 10 }, { 0x33, 0x3b }, { 0x60, 0x50 }, { 0x62, 0x80 },
        { 0x74, 1 }, { 0x5b, 0x14 }, { 0x41, 4 }, { 0x48, 0x32 },
        { 0x4a, 1 }, { 0x4e, 0 },
    };
    static const uint32_t wfcw[] = { 0x7a00, 0x2001, 0x10 };
    const ConvCase c = {
        .w_size = 576, .w_seed = 1, .pic_seed = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 4, .out_units = 4,
        .bt = bt, .bt_size = bt_size, .or2 = 0x410,
        .regs = regs, .nregs = ARRAY_SIZE(regs),
        .wfcw = wfcw, .nwfcw = ARRAY_SIZE(wfcw),
        .fr_raddr = 0x1fdb, .want = conv_1b2t_want,
        .want_len = sizeof(conv_1b2t_want),
    };

    conv_case(&c);
}

static void test_conv_1b2t(void)
{
    static const uint32_t bt[48] = {
        0x000086a6, 0xffffcc32, 0xffff13be, 0x0000574c,
        0xffff9cd8, 0x0000e064, 0x000027f0, 0xffff6d7e,
        0x00013880, 0x00014c08, 0x00015f90, 0x00017318,
        0x000186a0, 0x00019a28, 0x0001adb0, 0x0001c138,
        0x0000ea60, 0x00015f90, 0xa0011170, 0x84011170,
        0x8800c350, 0xc0138808, 0xc0138801, 0xa4009c40,
        0xfffff533, 0xffff97f9, 0xffff3bbf, 0xffffdd86,
        0xffff804c, 0x00002212, 0xffffc5d8, 0xffff689f,
        0x0000ea60, 0x000105b8, 0x00012110, 0x00013c68,
        0x000157c0, 0x00017318, 0x00018e70, 0x0001a9c8,
        0x000186a0, 0x00007530, 0xa000afc8, 0x8c015f90,
        0xc00ea602, 0x0000c350, 0xa800fde8, 0x000124f8,
    };

    conv_1b2t(bt, sizeof(bt));
}

/*
 * The same with bits 31:30 set in every T and bit 30 in every plain TN:
 * a T32LQ reads 30 bits of each, so the outputs do not change
 */
static void test_conv_1b2t_wide(void)
{
    static const uint32_t bt[48] = {
        0x000086a6, 0xffffcc32, 0xffff13be, 0x0000574c,
        0xffff9cd8, 0x0000e064, 0x000027f0, 0xffff6d7e,
        0x40013880, 0x80014c08, 0xc0015f90, 0x40017318,
        0x800186a0, 0xc0019a28, 0x4001adb0, 0x8001c138,
        0x4000ea60, 0x40015f90, 0xa0011170, 0x84011170,
        0x8800c350, 0xc0138808, 0xc0138801, 0xa4009c40,
        0xfffff533, 0xffff97f9, 0xffff3bbf, 0xffffdd86,
        0xffff804c, 0x00002212, 0xffffc5d8, 0xffff689f,
        0x8000ea60, 0xc00105b8, 0x40012110, 0x80013c68,
        0xc00157c0, 0x40017318, 0x80018e70, 0xc001a9c8,
        0x400186a0, 0x40007530, 0xa000afc8, 0x8c015f90,
        0xc00ea602, 0x4000c350, 0xa800fde8, 0x400124f8,
    };

    conv_1b2t(bt, sizeof(bt));
}

/*
 * NNMMT's read-address steps, in their order: RST sets the return point
 * to ORI, RINC moves it on by RINC, RET sets the address to it and INC
 * moves the address on, for the W-RAM's (WRRA, from bit 8) and the
 * BT-RAM's (BTRRA, from bit 12). The registers a T32LQ read after.
 */
static void test_mmt_steps(void)
{
    static const struct {
        uint32_t ops[4];
        uint32_t wr, wr_ret, bt, bt_ret;
    } cases[] = {
        { { 0x200 }, 0x10, 0x08, 0x11, 0x06 },
        { { 0x200, 0x200 }, 0x10, 0x0b, 0x11, 0x06 },
        { { 0x600 }, 0x08, 0x08, 0x11, 0x06 },
        { { 0x200, 0x400 }, 0x08, 0x08, 0x11, 0x06 },
        { { 0xa00 }, 0x17, 0x08, 0x11, 0x06 },
        { { 0x100 }, 0x10, 0x20, 0x11, 0x06 },
        { { 0x400, 0x800, 0x200, 0x400 }, 0x08, 0x08, 0x11, 0x06 },
        { { 0x500 }, 0x20, 0x20, 0x11, 0x06 },
        { { 0x300 }, 0x10, 0x23, 0x11, 0x06 },
        { { 0xc00 }, 0x0c, 0x05, 0x11, 0x06 },
        { { 0x2000 }, 0x10, 0x05, 0x11, 0x0a },
        { { 0x2000, 0x2000 }, 0x10, 0x05, 0x11, 0x0e },
        { { 0x1000 }, 0x10, 0x05, 0x11, 0x21 },
        { { 0x2000, 0x4000 }, 0x10, 0x05, 0x0a, 0x0a },
        { { 0x5000 }, 0x10, 0x05, 0x21, 0x21 },
        { { 0x3000 }, 0x10, 0x05, 0x11, 0x25 },
    };
    static const RegVal set[] = {
        { 0x38, 0x10 }, { 0x39, 0x5 }, { 0x3a, 0x7 }, { 0x3d, 0x20 },
        { 0x3e, 0x3 }, { 0x78, 0x11 }, { 0x79, 0x6 }, { 0x7a, 0x21 },
        { 0x7b, 0x4 },
    };
    Linked l;

    if (!linked_init(&l)) {
        return;
    }
    for (int i = 0; i < ARRAY_SIZE(cases); i++) {
        qtest_writel(l.core, TNNA + NNRST, 0);
        for (int j = 0; j < ARRAY_SIZE(set); j++) {
            nnrw(&l, set[j].reg, set[j].val);
        }
        for (int j = 0; j < 4 && cases[i].ops[j]; j++) {
            port(&l, NNMMT, cases[i].ops[j] << 12);
        }
        g_assert_cmphex(qtest_readl(l.core, TNNA + REGS + 4 * 0x18), ==,
                        cases[i].wr);
        g_assert_cmphex(qtest_readl(l.core, TNNA + REGS + 4 * 0x19), ==,
                        cases[i].wr_ret);
        g_assert_cmphex(qtest_readl(l.core, TNNA + REGS + 4 * 0x58), ==,
                        cases[i].bt);
        g_assert_cmphex(qtest_readl(l.core, TNNA + REGS + 4 * 0x59), ==,
                        cases[i].bt_ret);
    }
    linked_quit(&l);
}

/* conv-3x3's set-up, for the partial sums' cases */
static const RegVal conv_3x3_regs[] = {
    { 0x20, 0x5e }, { 0x21, 0x22 }, { 0x2c, 3 }, { 0x2e, 7 },
    { 0x2d, 0x13 }, { 0x28, 0x11 }, { 0x30, 4 }, { 0x31, 4 },
    { 0x32, 10 }, { 0x33, 0x3b }, { 0x60, 0x10 }, { 0x74, 1 },
    { 0x5b, 0x14 }, { 0x41, 4 }, { 0x48, 0x32 }, { 0x4a, 1 },
    { 0x4e, 0 },
};
static const uint32_t conv_3x3_wfcw[] = { 0x7a00, 0x2001, 0x10 };

static void words_le(uint8_t *out, const uint32_t *w, int n)
{
    for (int i = 0; i < n; i++) {
        stl_le_p(&out[4 * i], w[i]);
    }
}

/*
 * conv-3x3's sums written out (SUMP_CFG 1, SUMOUT) at OR_WADDR_SUM 0x300
 * as 32-bit words, 8 slots of MAC_P_MASK's even lanes (slot i lane 2i):
 * a slot's 8 sums at 32 i bytes; OR_WADDR_SUM steps 2 units a slot
 */
static void test_conv_sump_out(void)
{
    static const RegVal mmt[] = {
        { 0x23, 0x5555 }, { 0x7c, 0x6420 }, { 0x7d, 0xeca8 }, { 0x61, 1 },
        { 0x6b, 0x300 },
    };
    static const RegVal after[] = { { 0x6a, 0x400 }, { 0x6b, 0x310 } };
    static const uint32_t want32[64] = {
        0xffffc107, 0x00008d99, 0x0000aa00, 0xfffdfdd9,
        0xfffe675f, 0xfffeee49, 0xfffe323f, 0xfffee723,
        0xffff380d, 0xffff7278, 0x0000fc2d, 0xffff5e58,
        0xfffe4a55, 0xffff34f3, 0xfffe5a08, 0xfffe23fb,
        0xffff65c7, 0x0000afa8, 0xffff3b68, 0xfffe97ff,
        0xffff2635, 0xffff110e, 0xfffdf153, 0xfffd8fbb,
        0xfffed261, 0xfffec07d, 0xffff9af9, 0xffff8e17,
        0xfffee3aa, 0x000006c6, 0xfffd9dbe, 0xfffe12fb,
        0xffff4216, 0xfffec5be, 0x00002f6c, 0xffff7072,
        0xfffeeee1, 0xfffe17a4, 0xffff2f8c, 0xfffe72c1,
        0xfffe6904, 0xffff903b, 0xffffb333, 0xffffe55a,
        0xffffb1fd, 0x0000ad69, 0xfffe4b1c, 0xfffe65b7,
        0xfffcfbce, 0xffff379f, 0x0000fa6f, 0x0000082d,
        0xffff5e2e, 0xfffea3a3, 0xfffe67c2, 0xfffdad31,
        0xfffdbdb7, 0x0000026b, 0x00007c4e, 0x0000e208,
        0xfffe6e31, 0xfffffbfd, 0xfffea839, 0xfffd7543,
    };
    uint8_t want[256];

    words_le(want, want32, 64);
    const ConvCase c = {
        .w_size = 576, .w_seed = 1, .pic_seed = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 1, .out_units = 16,
        .b = { 34470, -13262, -60482, 22348, -25384, 57444, 10224, -37506 },
        .t = { 80000, 85000, 90000, 95000, 100000, 105000, 110000, 115000 },
        .regs = conv_3x3_regs, .nregs = ARRAY_SIZE(conv_3x3_regs),
        .wfcw = conv_3x3_wfcw, .nwfcw = ARRAY_SIZE(conv_3x3_wfcw),
        .fr_raddr = 0x1fdb, .want = want, .want_len = sizeof(want),
        .mmt_regs = mmt, .nmmt_regs = ARRAY_SIZE(mmt), .out_unit = 0x300,
        .after = after, .nafter = ARRAY_SIZE(after),
    };

    conv_case(&c);
}

/*
 * SUMIO (2): 128 words in the ORAM at 0x300 of every top nibble read,
 * conv-3x3's sums added (wrapping at 32 bits) and written back in place;
 * both pointers step
 */
static void test_conv_sump_io(void)
{
    static const int32_t mants[8] = {
        1, 3, -1, -3, 100, -100, 0x7ffffff, -0x8000000,
    };
    static const RegVal mmt[] = {
        { 0x61, 2 }, { 0x69, 0x300 }, { 0x6b, 0x300 },
    };
    static const RegVal after[] = { { 0x69, 0x320 }, { 0x6b, 0x320 } };
    static const uint32_t want32[128] = {
        0xffffc108, 0x00008d9c, 0x1000a9ff, 0x0ffdfdd6,
        0xfffe67c3, 0x0ffeede5, 0x07fe323e, 0x07fee723,
        0x0ffcedde, 0x0ffffc6f, 0x1fff505d, 0x2000056e,
        0x0ffee5e7, 0x20009250, 0x17ff87a8, 0x17ff7048,
        0x1fff380e, 0x1fff727b, 0x3000fc2c, 0x2fff5e55,
        0x1ffe4ab9, 0x2fff348f, 0x27fe5a07, 0x27fe23fb,
        0x2ffdf335, 0x2ffee293, 0x3fff4495, 0x3fffb745,
        0x2fff120c, 0x3ffe6d5d, 0x37ffbdba, 0x37fe13d0,
        0x3fff65c8, 0x4000afab, 0x4fff3b67, 0x4ffe97fc,
        0x3fff2699, 0x4fff10aa, 0x47fdf152, 0x47fd8fbb,
        0x4ffc9b0e, 0x4ffde7f8, 0x60007a72, 0x6000511e,
        0x4ffff867, 0x600036a2, 0x57fe63bf, 0x57fe1b56,
        0x5ffed262, 0x5ffec080, 0x6fff9af8, 0x6fff8e14,
        0x5ffee40e, 0x70000662, 0x67fd9dbd, 0x67fe12fb,
        0x6ffebaf8, 0x6fffb882, 0x80001b09, 0x8000342b,
        0x700020b2, 0x7fffaaee, 0x77feaa42, 0x77fe3ba8,
        0x7fff4217, 0x7ffec5c1, 0x90002f6b, 0x8fff706f,
        0x7ffeef45, 0x8ffe1740, 0x87ff2f8b, 0x87fe72c1,
        0x8ffdbdaa, 0x8fff8959, 0xa0008b2f, 0x9fff6316,
        0x8fffe40d, 0x9fff5f27, 0x97fe7a27, 0x97fee1de,
        0x9ffe6905, 0x9fff903e, 0xafffb332, 0xafffe557,
        0x9fffb261, 0xb000ad05, 0xa7fe4b1b, 0xa7fe65b7,
        0xaffe8440, 0xb0005a83, 0xc00011a8, 0xbfffc22b,
        0xafffa72e, 0xbffe809c, 0xb7fe8df8, 0xb7fe2f86,
        0xbffcfbcf, 0xbfff37a2, 0xd000fa6e, 0xd000082a,
        0xbfff5e92, 0xcffea33f, 0xc7fe67c1, 0xc7fdad31,
        0xcffd7ca0, 0xcfff5c59, 0xe0006521, 0xdfff0372,
        0xcfffd35f, 0xdfff15df, 0xd7fd02fe, 0xd7fdb4f8,
        0xdffdbdb8, 0xe000026e, 0xf0007c4d, 0xf000e205,
        0xdffe6e95, 0xeffffb99, 0xe7fea838, 0xe7fd7543,
        0xeffd360a, 0xefff9b74, 0x00012ecb, 0xffff12d1,
        0xeffebf2d, 0x0000265a, 0xf7fe9161, 0xf7fe924d,
    };
    uint32_t pre[128];
    uint8_t want[512];

    for (int k = 0; k < 128; k++) {
        pre[k] = (k / 8) << 28 | (mants[k % 8] & 0xfffffff);
    }
    words_le(want, want32, 128);
    const ConvCase c = {
        .w_size = 576, .w_seed = 1, .pic_seed = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 1, .out_units = 32,
        .b = { 34470, -13262, -60482, 22348, -25384, 57444, 10224, -37506 },
        .t = { 80000, 85000, 90000, 95000, 100000, 105000, 110000, 115000 },
        .regs = conv_3x3_regs, .nregs = ARRAY_SIZE(conv_3x3_regs),
        .wfcw = conv_3x3_wfcw, .nwfcw = ARRAY_SIZE(conv_3x3_wfcw),
        .fr_raddr = 0x1fdb, .want = want, .want_len = sizeof(want),
        .pre = pre, .pre_words = 128,
        .mmt_regs = mmt, .nmmt_regs = ARRAY_SIZE(mmt), .out_unit = 0x300,
        .after = after, .nafter = ARRAY_SIZE(after),
    };

    conv_case(&c);
}

/*
 * SUMINQ (3): the words at 0x300 added to conv-3x3's sums and quantized,
 * 16-bit (B 0, T 2^17); OR_RADDR_SUM steps, OR_WADDR_SUM does not
 */
static void test_conv_sump_inq(void)
{
    static const RegVal mmt[] = {
        { 0x60, 0x30 }, { 0x61, 3 }, { 0x69, 0x300 }, { 0x6b, 0x300 },
    };
    static const RegVal after[] = { { 0x69, 0x320 }, { 0x6b, 0x300 } };
    static const uint16_t want16[128] = {
        0x58e0, 0x72f1, 0x76fd, 0x21f7, 0x2fa6, 0x4102, 0x29ff, 0x411a,
        0x0270, 0x64c1, 0x4fbe, 0x66df, 0x435f, 0x7984, 0x58a1, 0x5634,
        0x4fab, 0x5777, 0x892c, 0x55f0, 0x33ef, 0x51c1, 0x36e2, 0x309f,
        0x2b05, 0x496f, 0x562f, 0x6503, 0x50ce, 0x3cd0, 0x674e, 0x328f,
        0x5d4c, 0x8707, 0x58fe, 0x450f, 0x5755, 0x552e, 0x31b6, 0x2601,
        0x07ea, 0x3206, 0x84d4, 0x8029, 0x7583, 0x7de2, 0x43f8, 0x3b6a,
        0x52ca, 0x510c, 0x6cda, 0x6bbc, 0x56ed, 0x7bcf, 0x2f2d, 0x3e53,
        0x53d1, 0x7401, 0x80d1, 0x8474, 0x8277, 0x7456, 0x54b3, 0x475e,
        0x68aa, 0x599e, 0x8752, 0x6ff2, 0x603e, 0x45d5, 0x6951, 0x5236,
        0x3c12, 0x7606, 0x96c0, 0x723c, 0x82cc, 0x72c7, 0x5699, 0x640f,
        0x5572, 0x7ad8, 0x7fb5, 0x8679, 0x808c, 0xa078, 0x54ad, 0x587f,
        0x5cce, 0x9815, 0x8f79, 0x8608, 0x831a, 0x5ee0, 0x60fd, 0x55ae,
        0x2fb5, 0x77ae, 0xb087, 0x92bd, 0x7dfc, 0x6729, 0x602c, 0x4958,
        0x43c4, 0x803a, 0xa1d2, 0x761b, 0x908a, 0x7972, 0x3788, 0x4e46,
        0x4fdc, 0x98f2, 0xa8ad, 0xb5e2, 0x67e6, 0x9a1e, 0x7024, 0x4a44,
        0x42dc, 0x9007, 0xc2f1, 0x7ff1, 0x75ee, 0xa36b, 0x713f, 0x71db,
    };
    uint32_t pre[128];
    uint8_t want[256];

    for (int k = 0; k < 128; k++) {
        pre[k] = k * 1013 - 64000;
        stw_le_p(&want[2 * k], want16[k]);
    }
    const ConvCase c = {
        .w_size = 576, .w_seed = 1, .pic_seed = 2,
        .pic_rows = 4, .pic_units = 4, .out_rows = 1, .out_units = 16,
        .t = { 1 << 17, 1 << 17, 1 << 17, 1 << 17,
               1 << 17, 1 << 17, 1 << 17, 1 << 17 },
        .regs = conv_3x3_regs, .nregs = ARRAY_SIZE(conv_3x3_regs),
        .wfcw = conv_3x3_wfcw, .nwfcw = ARRAY_SIZE(conv_3x3_wfcw),
        .fr_raddr = 0x1fdb, .want = want, .want_len = sizeof(want),
        .pre = pre, .pre_words = 128,
        .mmt_regs = mmt, .nmmt_regs = ARRAY_SIZE(mmt),
        .after = after, .nafter = ARRAY_SIZE(after),
    };

    conv_case(&c);
}

/*
 * The T32Pro's windows, from its core 0: the ports at 0x15070000, the DMA
 * at 0x15050000, and the 128 KiB ORAM at 0x15100000, which the core reads
 * and writes as memory and the DMA fills and empties
 */
static void test_pro(void)
{
    g_autofree char *bin_dir = g_path_get_dirname(getenv("QTEST_QEMU_BINARY"));
    g_autofree char *rv = g_build_filename(bin_dir, "qemu-system-riscv32",
                                           NULL);
    g_autofree char *dir = NULL;
    g_autofree char *sock = NULL;
    g_autofree char *args = NULL;
    QTestState *soc, *core;

    if (!g_file_test(rv, G_FILE_TEST_IS_EXECUTABLE)) {
        g_test_skip("no qemu-system-riscv32 to link");
        return;
    }
    dir = g_dir_make_tmp("ingenic-tnna-XXXXXX", NULL);
    sock = g_build_filename(dir, "tnpu.sock", NULL);
    soc = qtest_initf("-machine ingenic-t32,memory-backend=ddr -m 64M "
                      "-global ingenic-t31.soc-variant=t32prozl "
                      "-object memory-backend-shm,id=ddr,size=64M "
                      "-chardev socket,id=tnpu,path=%s,server=on,wait=off "
                      "-global ingenic-tnpu.chardev=tnpu", sock);
    g_setenv("QTEST_QEMU_BINARY_TNPU", rv, true);
    args = g_strdup_printf("-machine ingenic-t32pro-tnpu,core=0,socket=%s",
                           sock);
    core = qtest_init_ext("QTEST_QEMU_BINARY_TNPU", args, NULL, true);

    qtest_writel(core, PRO_NNA + NNRST, 0);
    g_assert_cmphex(qtest_readl(core, PRO_NNA + REGS), ==, 0x5a);
    qtest_writel(core, PRO_NNA + NNRW, 0x2c << 20 | 0x123);
    g_assert_cmphex(qtest_readl(core, PRO_NNA + REGS + 0xc * 4), ==, 0x123);

    /* its RAMs' address registers keep a bit more than the T32's */
    qtest_writel(core, PRO_NNA + NNRW, 0x6a << 20 | 0xfffff);
    g_assert_cmphex(qtest_readl(core, PRO_NNA + REGS + 0x4a * 4), ==, 0x3fff);
    qtest_writel(core, PRO_NNA + NNRW, 0x40 << 20 | 0xfffff);
    g_assert_cmphex(qtest_readl(core, PRO_NNA + REGS + 0x20 * 4), ==, 0x1fff);
    qtest_writel(core, PRO_NNA + NNRW, 0x5c << 20 | 0xfffff);
    g_assert_cmphex(qtest_readl(core, PRO_NNA + REGS + 0x3c * 4), ==, 0xfff);
    qtest_writel(core, PRO_NNA + NNRW, 0x78 << 20 | 0xfffff);
    g_assert_cmphex(qtest_readl(core, PRO_NNA + REGS + 0x58 * 4), ==, 0xff);

    /* the core's own words, above the T32's 64 KiB too */
    qtest_writel(core, PRO_ORAM + 0x10000, 0x11223344);
    qtest_writeb(core, PRO_ORAM + 0x10001, 0xaa);
    g_assert_cmphex(qtest_readl(core, PRO_ORAM + 0x10000), ==, 0x1122aa44);

    /* 4 units from DDR to ORAM unit 0x1004, and back out from there */
    for (int i = 0; i < 16; i++) {
        qtest_writel(soc, SRC + 4 * i, 0x5a000000 | i);
        qtest_writel(soc, DST + 4 * i, 0);
    }
    qtest_writel(core, PRO_DMA + FIN + EREG0L, SRC | 1);
    qtest_writel(core, PRO_DMA + FIN + EREG0H, 0x00041004);
    qtest_writel(core, PRO_DMA + FIN + RUN, 1);
    g_assert_cmphex(qtest_readl(core, PRO_ORAM + 0x10040 + 4 * 15), ==,
                    0x5a00000f);
    qtest_writel(core, PRO_ORAM + 0x10040, 0x77);
    qtest_writel(core, PRO_DMA + FOUT + EREG0L, DST);
    qtest_writel(core, PRO_DMA + FOUT + EREG0H, 0x00041004);
    qtest_writel(core, PRO_DMA + FOUT + RUN, 1);
    g_assert_cmphex(qtest_readl(soc, DST), ==, 0x77);
    for (int i = 1; i < 16; i++) {
        g_assert_cmphex(qtest_readl(soc, DST + 4 * i), ==, 0x5a000000 | i);
    }

    qtest_quit(core);
    qtest_quit(soc);
    unlink(sock);
    rmdir(dir);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("ingenic-tnna/host-view", test_host_view);
    qtest_add_func("ingenic-tnna/regs", test_regs);
    qtest_add_func("ingenic-tnna/dma-fin", test_dma_fin);
    qtest_add_func("ingenic-tnna/dma-fout", test_dma_fout);
    qtest_add_func("ingenic-tnna/dma-sets", test_dma_sets);
    qtest_add_func("ingenic-tnna/dma-back", test_dma_back);
    qtest_add_func("ingenic-tnna/cva-bt601", test_cva_bt601);
    qtest_add_func("ingenic-tnna/cva-round", test_cva_round);
    qtest_add_func("ingenic-tnna/conv-3x3", test_conv_3x3);
    qtest_add_func("ingenic-tnna/conv-3x3-s2-4bit", test_conv_3x3_s2_4bit);
    qtest_add_func("ingenic-tnna/conv-3x3-2planes", test_conv_3x3_2planes);
    qtest_add_func("ingenic-tnna/conv-1b2t", test_conv_1b2t);
    qtest_add_func("ingenic-tnna/conv-1b2t-wide", test_conv_1b2t_wide);
    qtest_add_func("ingenic-tnna/conv-3x3-dw", test_conv_3x3_dw);
    qtest_add_func("ingenic-tnna/mmt-steps", test_mmt_steps);
    qtest_add_func("ingenic-tnna/conv-sump-out", test_conv_sump_out);
    qtest_add_func("ingenic-tnna/conv-sump-io", test_conv_sump_io);
    qtest_add_func("ingenic-tnna/conv-sump-inq", test_conv_sump_inq);
    qtest_add_func("ingenic-tnna/pro", test_pro);
    return g_test_run();
}
