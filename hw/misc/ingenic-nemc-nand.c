/*
 * Parallel NAND on an Ingenic NEMC bank
 *
 * This follows the vendor and mainline drivers, not silicon: no bench board
 * carries a parallel NAND, so only the contract those drivers use is here.
 * The chip sits behind one NEMC bank (CS1 to CS6) with its data port at the
 * start of the bank's window, its command latch at +0x400000 and its address
 * latch at +0x800000, each one byte per access; the chip is selected while
 * the bank's NFE and NFCE bits in NFCSR are both set (the CS2 pin of a T31X
 * measured low only then), and the ready/busy pin is the "rb" output (high
 * when ready), pulsed low for as long as a command takes, which here is no
 * time at all. The NEMC does the rest the drivers leave to the chip.
 *
 * The chip is an 8-bit, 2 KiB or larger page ONFI-style part with a raw
 * image behind it, each page followed by its spare area, the way a
 * programmer dumps one. It takes reset (0xff), read ID (0x90, address 0x00
 * for the five ID bytes, 0x20 for "ONFI"), read status (0x70), page read
 * (0x00 .. 0x30) with random data out (0x05 .. 0xe0), page program (0x80
 * .. 0x10) with random data in (0x85), and block erase (0x60 .. 0xd0). A
 * program only clears bits, as the cells do. Not modelled: the parameter
 * page, cache and multi-plane commands, 16-bit buses, bad blocks beyond
 * what the image holds, bit errors and timing.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/irq.h"
#include "hw/misc/ingenic-nemc.h"
#include "migration/vmstate.h"
#include "system/block-backend.h"

#define NAND_RESET      0xff
#define NAND_READID     0x90
#define NAND_STATUS     0x70
#define NAND_READ0      0x00
#define NAND_READ_END   0x30
#define NAND_RNDOUT     0x05
#define NAND_RNDOUT_END 0xe0
#define NAND_SEQIN      0x80
#define NAND_RNDIN      0x85
#define NAND_PAGEPROG   0x10
#define NAND_ERASE      0x60
#define NAND_ERASE_END  0xd0

#define STATUS_FAIL     0x01
#define STATUS_READY    0x60    /* ready, and ready for another command */
#define STATUS_NOWP     0x80

static const char onfi_sig[4] = "ONFI";

static uint32_t nand_stride(IngenicNemcNandState *n)
{
    return n->buf_size;
}

static void nand_busy(IngenicNemcNandState *n, bool busy)
{
    qemu_set_irq(n->rb, !busy);
}

static uint32_t nand_row(IngenicNemcNandState *n, unsigned from)
{
    uint32_t row = 0;

    for (unsigned i = 0; i < n->row_cycles; i++) {
        row |= (uint32_t)n->addr[from + i] << (8 * i);
    }
    return row;
}

static uint32_t nand_col(IngenicNemcNandState *n)
{
    return n->addr[0] | n->addr[1] << 8;
}

static bool nand_page_io(IngenicNemcNandState *n, uint32_t row, bool write)
{
    uint64_t off = (uint64_t)row * nand_stride(n);
    int ret;

    if (row >= n->pages) {
        return false;
    }
    if (write) {
        ret = blk_pwrite(n->blk, off, nand_stride(n), n->buf, 0);
    } else {
        ret = blk_pread(n->blk, off, nand_stride(n), n->buf, 0);
    }
    return ret >= 0;
}

static void nand_program(IngenicNemcNandState *n)
{
    uint32_t row = nand_row(n, 2);
    g_autofree uint8_t *old = g_malloc(nand_stride(n));

    /* Cells only go from 1 to 0: the page as it was, ANDed with the data */
    if (row >= n->pages ||
        blk_pread(n->blk, (uint64_t)row * nand_stride(n), nand_stride(n),
                  old, 0) < 0) {
        n->status |= STATUS_FAIL;
    } else {
        for (uint32_t i = 0; i < nand_stride(n); i++) {
            n->buf[i] &= old[i];
        }
        if (!nand_page_io(n, row, true)) {
            n->status |= STATUS_FAIL;
        }
    }
}

static void nand_erase(IngenicNemcNandState *n)
{
    uint32_t row = nand_row(n, 0) & ~(n->pages_per_block - 1);
    g_autofree uint8_t *ff = g_malloc(nand_stride(n));

    memset(ff, 0xff, nand_stride(n));
    for (uint32_t i = 0; i < n->pages_per_block; i++) {
        if (row + i >= n->pages ||
            blk_pwrite(n->blk, (uint64_t)(row + i) * nand_stride(n),
                       nand_stride(n), ff, 0) < 0) {
            n->status |= STATUS_FAIL;
            return;
        }
    }
}

void ingenic_nemc_nand_cmd(IngenicNemcNandState *n, uint8_t cmd)
{
    switch (cmd) {
    case NAND_RESET:
        nand_busy(n, true);
        n->mode = NAND_IDLE;
        n->status = STATUS_READY | STATUS_NOWP;
        nand_busy(n, false);
        break;
    case NAND_READID:
        n->mode = NAND_ID;
        n->naddr = 0;
        n->ptr = 0;
        break;
    case NAND_STATUS:
        n->mode = NAND_STAT;
        break;
    case NAND_READ0:
        n->mode = NAND_READ_ADDR;
        n->naddr = 0;
        break;
    case NAND_READ_END:
        if (n->mode == NAND_READ_ADDR && n->naddr == 2 + n->row_cycles) {
            nand_busy(n, true);
            n->status &= ~STATUS_FAIL;
            if (!nand_page_io(n, nand_row(n, 2), false)) {
                memset(n->buf, 0xff, nand_stride(n));
                n->status |= STATUS_FAIL;
            }
            n->ptr = nand_col(n);
            n->mode = NAND_DATA_OUT;
            nand_busy(n, false);
        }
        break;
    case NAND_RNDOUT:
        if (n->mode == NAND_DATA_OUT) {
            n->mode = NAND_RND_ADDR;
            n->naddr = 0;
        }
        break;
    case NAND_RNDOUT_END:
        if (n->mode == NAND_RND_ADDR && n->naddr == 2) {
            n->ptr = nand_col(n);
            n->mode = NAND_DATA_OUT;
        }
        break;
    case NAND_SEQIN:
        n->mode = NAND_PROG_ADDR;
        n->naddr = 0;
        memset(n->buf, 0xff, nand_stride(n));
        break;
    case NAND_RNDIN:
        if (n->mode == NAND_PROG_DATA) {
            n->mode = NAND_RND_IN_ADDR;
            n->naddr = 0;
        }
        break;
    case NAND_PAGEPROG:
        if (n->mode == NAND_PROG_DATA) {
            nand_busy(n, true);
            n->status &= ~STATUS_FAIL;
            nand_program(n);
            nand_busy(n, false);
        }
        n->mode = NAND_STAT;
        break;
    case NAND_ERASE:
        n->mode = NAND_ERASE_ADDR;
        n->naddr = 0;
        break;
    case NAND_ERASE_END:
        if (n->mode == NAND_ERASE_ADDR && n->naddr == n->row_cycles) {
            nand_busy(n, true);
            n->status &= ~STATUS_FAIL;
            nand_erase(n);
            nand_busy(n, false);
        }
        n->mode = NAND_STAT;
        break;
    default:
        break;
    }
}

void ingenic_nemc_nand_addr(IngenicNemcNandState *n, uint8_t val)
{
    switch (n->mode) {
    case NAND_ID:
        n->ptr = 0;
        n->id_addr = val;
        break;
    case NAND_READ_ADDR:
    case NAND_RND_ADDR:
    case NAND_PROG_ADDR:
    case NAND_RND_IN_ADDR:
    case NAND_ERASE_ADDR:
        if (n->naddr < sizeof(n->addr)) {
            n->addr[n->naddr++] = val;
        }
        if (n->mode == NAND_PROG_ADDR && n->naddr == 2 + n->row_cycles) {
            n->ptr = nand_col(n);
            n->mode = NAND_PROG_DATA;
        }
        if (n->mode == NAND_RND_IN_ADDR && n->naddr == 2) {
            n->ptr = nand_col(n);
            n->mode = NAND_PROG_DATA;
        }
        break;
    default:
        break;
    }
}

uint8_t ingenic_nemc_nand_read(IngenicNemcNandState *n)
{
    switch (n->mode) {
    case NAND_STAT:
        return n->status;
    case NAND_ID:
        if (n->id_addr == 0x20) {
            return n->ptr < sizeof(onfi_sig) ? onfi_sig[n->ptr++] : 0;
        }
        return n->ptr < sizeof(n->id) ? n->id[n->ptr++] : 0;
    case NAND_DATA_OUT:
        return n->ptr < nand_stride(n) ? n->buf[n->ptr++] : 0xff;
    default:
        return 0xff;
    }
}

void ingenic_nemc_nand_write(IngenicNemcNandState *n, uint8_t val)
{
    if (n->mode == NAND_PROG_DATA && n->ptr < nand_stride(n)) {
        n->buf[n->ptr++] = val;
    }
}

static void nand_realize(DeviceState *dev, Error **errp)
{
    IngenicNemcNandState *n = INGENIC_NEMC_NAND(dev);
    int64_t size;

    if (!n->blk) {
        error_setg(errp, "ingenic-nemc-nand needs a drive");
        return;
    }
    if (n->page_size < 2048 || !is_power_of_2(n->page_size) ||
        n->oob_size < 16 || n->oob_size > n->page_size / 8 ||
        n->pages_per_block < 2 || !is_power_of_2(n->pages_per_block)) {
        error_setg(errp, "ingenic-nemc-nand: bad page-size, oob-size or "
                   "pages-per-block");
        return;
    }
    n->buf_size = n->page_size + n->oob_size;
    size = blk_getlength(n->blk);
    if (size <= 0 || size % ((int64_t)n->buf_size * n->pages_per_block)) {
        error_setg(errp, "ingenic-nemc-nand: the image must be a whole "
                   "number of blocks of %u pages of %u + %u bytes",
                   n->pages_per_block, n->page_size, n->oob_size);
        return;
    }
    if (blk_set_perm(n->blk, BLK_PERM_CONSISTENT_READ |
                     (blk_supports_write_perm(n->blk) ? BLK_PERM_WRITE : 0),
                     BLK_PERM_ALL, errp) < 0) {
        return;
    }
    n->pages = size / n->buf_size;
    n->row_cycles = n->pages > 65536 ? 3 : 2;
    n->buf = g_malloc(nand_stride(n));
    for (int i = 0; i < 5; i++) {
        n->id[i] = n->id_bytes >> (8 * i);
    }
    n->status = STATUS_READY | STATUS_NOWP;
    memset(n->buf, 0xff, nand_stride(n));
    if (!ingenic_nemc_attach_nand(n, n->bank, errp)) {
        return;
    }
    nand_busy(n, false);
}

static void nand_unrealize(DeviceState *dev)
{
    g_free(INGENIC_NEMC_NAND(dev)->buf);
}

static void nand_reset(DeviceState *dev)
{
    IngenicNemcNandState *n = INGENIC_NEMC_NAND(dev);

    n->mode = NAND_IDLE;
    n->status = STATUS_READY | STATUS_NOWP;
    nand_busy(n, false);
}

static void nand_init(Object *obj)
{
    qdev_init_gpio_out_named(DEVICE(obj), &INGENIC_NEMC_NAND(obj)->rb, "rb",
                             1);
}

static const Property nand_props[] = {
    DEFINE_PROP_DRIVE("drive", IngenicNemcNandState, blk),
    DEFINE_PROP_UINT32("bank", IngenicNemcNandState, bank, 1),
    DEFINE_PROP_UINT32("page-size", IngenicNemcNandState, page_size, 2048),
    DEFINE_PROP_UINT32("oob-size", IngenicNemcNandState, oob_size, 64),
    DEFINE_PROP_UINT32("pages-per-block", IngenicNemcNandState,
                       pages_per_block, 64),
    /* ID bytes 0 to 4, byte 0 lowest: Samsung K9F1G08U0A's ec f1 00 95 40 */
    DEFINE_PROP_UINT64("id", IngenicNemcNandState, id_bytes, 0x409500f1ec),
};

static int nand_post_load(void *opaque, int version_id)
{
    IngenicNemcNandState *n = opaque;

    return n->ptr <= nand_stride(n) ? 0 : -EINVAL;
}

static const VMStateDescription vmstate_nand = {
    .name = TYPE_INGENIC_NEMC_NAND,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = nand_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mode, IngenicNemcNandState),
        VMSTATE_UINT32(naddr, IngenicNemcNandState),
        VMSTATE_UINT8_ARRAY(addr, IngenicNemcNandState, 8),
        VMSTATE_UINT32(ptr, IngenicNemcNandState),
        VMSTATE_UINT8(id_addr, IngenicNemcNandState),
        VMSTATE_UINT8(status, IngenicNemcNandState),
        VMSTATE_VBUFFER_UINT32(buf, IngenicNemcNandState, 1, NULL,
                               buf_size),
        VMSTATE_END_OF_LIST()
    },
};

static void nand_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nand_realize;
    dc->unrealize = nand_unrealize;
    dc->vmsd = &vmstate_nand;
    device_class_set_props(dc, nand_props);
    device_class_set_legacy_reset(dc, nand_reset);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo nand_info = {
    .name = TYPE_INGENIC_NEMC_NAND,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(IngenicNemcNandState),
    .instance_init = nand_init,
    .class_init = nand_class_init,
};

static void nand_register_types(void)
{
    type_register_static(&nand_info);
}

type_init(nand_register_types)
