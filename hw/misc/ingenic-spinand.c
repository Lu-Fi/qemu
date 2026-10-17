/*
 * Ingenic SoCs: a SPI NAND behind the SFC
 *
 * 2 KiB pages with a 128-byte spare area, 64 pages a block, the
 * GigaDevice feature registers, the ID the controller was given. The
 * page data lives in the controller's flash array, back to back, as
 * thingino builds NAND images; the spare areas live in RAM, all 0xff at
 * power-on, so every block reads good. Operations finish at once and
 * ECC never reports a bitflip. Any BP bit locks the whole array.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/misc/ingenic-spinand.h"

#define PAGE                INGENIC_SPINAND_PAGE
#define OOB                 INGENIC_SPINAND_OOB
#define PPB                 INGENIC_SPINAND_PPB

#define FEAT_LOCK           0
#define FEAT_CFG            1
#define FEAT_STATUS         2
#define LOCK_BP             0x38
#define CFG_ECC_EN          0x10
#define STS_WEL             0x02
#define STS_E_FAIL          0x04
#define STS_P_FAIL          0x08
#define STS_ECC             0x70

static int feat_index(uint32_t addr)
{
    switch (addr & 0xff) {
    case 0xa0:
        return 0;
    case 0xb0:
        return 1;
    case 0xc0:
        return 2;
    case 0xd0:
        return 3;
    }
    return -1;
}

static uint32_t pages(IngenicSpiNand *n)
{
    return n->blocks * PPB;
}

/* PROGRAM EXECUTE and BLOCK ERASE need WEL and an unlocked array */
static bool writable(IngenicSpiNand *n, const char *op, uint32_t row)
{
    const char *why = !(n->feat[FEAT_STATUS] & STS_WEL) ? "no WRITE ENABLE" :
                      (n->feat[FEAT_LOCK] & LOCK_BP) ? "feature A0h locks it" :
                      row >= pages(n) ? "past the end" : NULL;

    if (why) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic SPI NAND: %s of page 0x%x "
                      "fails: %s\n", op, row, why);
    }
    return !why;
}

static void page_read(IngenicSpiNand *n, uint32_t row)
{
    if (row < pages(n)) {
        memcpy(n->cache, &n->data[(size_t)row * PAGE], PAGE);
        memcpy(&n->cache[PAGE], &n->oob[(size_t)row * OOB], OOB);
    } else {
        memset(n->cache, 0xff, sizeof(n->cache));
    }
    n->feat[FEAT_STATUS] &= ~STS_ECC;
}

static void program(IngenicSpiNand *n, uint32_t row)
{
    uint8_t *st = &n->feat[FEAT_STATUS];

    if (writable(n, "PROGRAM EXECUTE", row)) {
        uint8_t *pg = &n->data[(size_t)row * PAGE];
        uint8_t *oob = &n->oob[(size_t)row * OOB];

        for (int i = 0; i < PAGE; i++) {
            pg[i] &= n->cache[i];
        }
        for (int i = 0; i < OOB; i++) {
            oob[i] &= n->cache[PAGE + i];
        }
        n->writeback(n->opaque, row * PAGE, PAGE);
        *st &= ~STS_P_FAIL;
    } else {
        *st |= STS_P_FAIL;
    }
    *st &= ~STS_WEL;
}

static void erase(IngenicSpiNand *n, uint32_t row)
{
    uint8_t *st = &n->feat[FEAT_STATUS];
    uint32_t first = row - row % PPB;

    if (writable(n, "BLOCK ERASE", row)) {
        memset(&n->data[(size_t)first * PAGE], 0xff, PPB * PAGE);
        memset(&n->oob[(size_t)first * OOB], 0xff, PPB * OOB);
        n->writeback(n->opaque, first * PAGE, PPB * PAGE);
        *st &= ~STS_E_FAIL;
    } else {
        *st |= STS_E_FAIL;
    }
    *st &= ~STS_WEL;
}

void ingenic_spinand_init(IngenicSpiNand *n, uint8_t *data,
                          IngenicSpiNandWriteback *writeback, void *opaque)
{
    size_t oob = (size_t)pages(n) * OOB;

    n->data = data;
    n->writeback = writeback;
    n->opaque = opaque;
    n->oob = g_malloc(oob);
    memset(n->oob, 0xff, oob);
    n->feat[FEAT_LOCK] = LOCK_BP;
    n->feat[FEAT_CFG] = CFG_ECC_EN;
    n->feat[FEAT_STATUS] = 0;
    n->feat[3] = 0;
    memset(n->cache, 0xff, sizeof(n->cache));
}

void ingenic_spinand_finalize(IngenicSpiNand *n)
{
    g_free(n->oob);
    n->oob = NULL;
}

void ingenic_spinand_command(IngenicSpiNand *n, uint8_t cmd, uint32_t addr,
                             bool dummy, uint8_t *buf, uint32_t len)
{
    uint8_t *st = &n->feat[FEAT_STATUS];
    uint32_t col = addr & 0xfff;

    n->cmd = cmd;
    n->addr = addr;
    memset(buf, 0xff, len);

    switch (cmd) {
    case 0xff:      /* RESET */
        *st = 0;
        break;
    case 0x9f: {    /* READ ID: MID, DID, repeating, after a dummy byte */
        uint8_t id[9] = { 0xff };

        for (int i = 1; i < 9; i++) {
            id[i] = (i & 1) ? n->id >> 8 : n->id;
        }
        memcpy(buf, dummy ? &id[1] : id, MIN(len, 8));
        break;
    }
    case 0x0f: {    /* GET FEATURE */
        int i = feat_index(addr);

        if (len) {
            buf[0] = i < 0 ? 0 : n->feat[i];
        }
        break;
    }
    case 0x06:      /* WRITE ENABLE */
        *st |= STS_WEL;
        break;
    case 0x04:      /* WRITE DISABLE */
        *st &= ~STS_WEL;
        break;
    case 0x13:      /* PAGE READ, to the cache */
        page_read(n, addr & 0xffffff);
        break;
    case 0x03:      /* READ FROM CACHE x1, x2, x4, dual and quad I/O */
    case 0x0b:
    case 0x3b:
    case 0xbb:
    case 0x6b:
    case 0xeb:
        if (col < sizeof(n->cache)) {
            memcpy(buf, &n->cache[col], MIN(len, sizeof(n->cache) - col));
        }
        break;
    case 0x1f:      /* SET FEATURE and PROGRAM LOAD: data phase follows */
    case 0x02:
    case 0x32:
    case 0x84:
    case 0x34:
    case 0xc4:
        break;
    case 0x10:      /* PROGRAM EXECUTE */
        program(n, addr & 0xffffff);
        break;
    case 0xd8:      /* BLOCK ERASE */
        erase(n, addr & 0xffffff);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic SPI NAND: command 0x%02x\n", cmd);
        break;
    }
}

void ingenic_spinand_data(IngenicSpiNand *n, const uint8_t *buf,
                          uint32_t len)
{
    static const uint8_t wmask[4] = { 0xbe, 0x51, 0x00, 0x60 };
    uint32_t col = n->addr & 0xfff;
    int i;

    switch (n->cmd) {
    case 0x1f:
        i = feat_index(n->addr);
        if (i >= 0 && len) {
            n->feat[i] = (n->feat[i] & ~wmask[i]) | (buf[0] & wmask[i]);
        }
        break;
    case 0x02:      /* PROGRAM LOAD clears the cache, RANDOM DATA does not */
    case 0x32:
        memset(n->cache, 0xff, sizeof(n->cache));
        /* fall through */
    case 0x84:
    case 0x34:
    case 0xc4:
        if (col < sizeof(n->cache)) {
            memcpy(&n->cache[col], buf, MIN(len, sizeof(n->cache) - col));
        }
        break;
    }
}
