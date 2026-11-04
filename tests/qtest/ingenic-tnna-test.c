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

#define SRC             0x01810000
#define DST             0x01820000
#define ZEROS           0x01830000
#define WBT             0x03800000
#define PIC             0x03801000
#define OUT             0x03802000

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
 * ORAM at 0, the picture (the LCG's too) at unit 0x100, one NNDWF, one
 * NNMAC from a pixel left of the picture, one NNMMT that quantizes into
 * unit 0x200, and out to DDR. The bytes a T32LQ's TNNA wrote, running
 * the same.
 */
typedef struct {
    uint32_t w_size, w_seed, pic_seed;
    uint32_t pic_rows, pic_units, out_rows, out_units;
    int32_t b[8], t[8];
    const RegVal *regs;
    int nregs;
    const uint32_t *wfcw;
    int nwfcw;
    uint32_t fr_raddr;
    const uint8_t *want;
    int want_len;
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
    uint8_t buf[640], got[128];

    if (!linked_init(&l)) {
        return;
    }
    lcg_fill(buf, c->w_size, c->w_seed);
    for (int i = 0; i < 8; i++) {
        stl_le_p(&buf[c->w_size + 4 * i], c->b[i]);
        stl_le_p(&buf[c->w_size + 32 + 4 * i], c->t[i]);
    }
    qtest_memwrite(l.soc, WBT, buf, c->w_size + 64);
    lcg_fill(buf, c->pic_rows * c->pic_units * 16, c->pic_seed);
    qtest_memwrite(l.soc, PIC, buf, c->pic_rows * c->pic_units * 16);
    fill_zero(&l, OUT, sizeof(got) / 4);

    qtest_writel(l.core, TNNA + NNRST, 0);
    dma(&l, WBTIN, EREG0L, WBT | 1);
    dma(&l, WBTIN, EREG0H, (c->w_size + 64) / 16 << 16);
    run(&l, WBTIN);
    dma(&l, FIN, GREG0, 0);
    dma(&l, FIN, GREG1, c->pic_units);
    dma(&l, FIN, GREG2, c->pic_units << 16);
    dma(&l, FIN, EREG0L, PIC | 1);
    dma(&l, FIN, EREG0H, (c->pic_rows - 1) << 28 | c->pic_units << 16 | 0x100);
    run(&l, FIN);
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
    port(&l, NNDWBT, 64 << 16 | c->w_size / 16);
    nnrw(&l, 0x40, 0x100);
    nnrw(&l, 0x50, 0);
    port(&l, NNDWF, 0);
    nnrw(&l, 0x34, c->fr_raddr);
    nnrw(&l, 0x29, 0);
    nnrw(&l, 0x2b, 0xfffff);
    port(&l, NNMAC, 0);
    port(&l, NNMMT, 0xf000);
    dma(&l, FOUT, GREG0, 0);
    dma(&l, FOUT, GREG1, c->out_units);
    dma(&l, FOUT, GREG2, c->out_units << 16);
    dma(&l, FOUT, EREG0L, OUT);
    dma(&l, FOUT, EREG0H,
        (c->out_rows - 1) << 28 | c->out_units << 16 | 0x200);
    port(&l, ENDSIG, 0);
    dma(&l, FOUT, ONNA, 1);
    qtest_readl(l.core, DMA + FOUT + RUN);
    qtest_memread(l.soc, OUT, got, sizeof(got));
    for (int i = 0; i < sizeof(got); i++) {
        g_assert_cmphex(got[i], ==, i < c->want_len ? c->want[i] : 0);
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
    return g_test_run();
}
