/*
 * Ingenic A1 SPI Flash Controller V2 (SFC) emulation
 *
 * SFC V2 flash model for the A1/T40/T41 XBurst2 SoCs. Supports PIO
 * reads, DMA descriptor chain reads, NOR erase and page-program with
 * writeback to the backing image, and a SPI NAND (nand-id).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/bswap.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/units.h"
#include "migration/vmstate.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-a1-sfc.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/physmem.h"

static void ingenic_a1_sfc_writeback(IngenicA1SfcState *s,
                                     uint32_t offset, uint32_t len)
{
    BlockBackend *blk = s->blk;
    int ret;

    if (!blk) {
        return;
    }
    /* Past the end of a short image the flash lives in RAM only */
    if (offset >= MIN(s->flash_size, s->blk_len)) {
        return;
    }
    if (offset + len > MIN(s->flash_size, s->blk_len)) {
        len = MIN(s->flash_size, s->blk_len) - offset;
    }
    ret = blk_pwrite(blk, offset, len, &s->flash_data[offset], 0);
    if (ret < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "ingenic-a1-sfc: write-back failed at "
                      "offset 0x%" PRIx32 " len %" PRIu32 ": %s\n",
                      offset, len, strerror(-ret));
    }
}

/* Register offsets */
#define SFC_GLB             0x0000
#define SFC_DEV_CONF        0x0004
#define SFC_DEV_STA_EXP     0x0008
#define SFC_DEV_STA_RT      0x000C
#define SFC_DEV_STA_MSK     0x0010
#define SFC_TRAN_CONF0      0x0014
#define SFC_TRAN_LEN        0x002C
#define SFC_DEV_ADDR0       0x0030
#define SFC_DEV_ADDR_PLUS0  0x0048
#define SFC_MEM_ADDR        0x0060
#define SFC_TRIG            0x0064
#define SFC_SR              0x0068
#define SFC_SCR             0x006C
#define SFC_INTC            0x0070
#define SFC_CGE             0x0078
#define SFC_CMD_IDX         0x007C
#define SFC_COL_ADDR        0x0080
#define SFC_ROW_ADDR        0x0084
#define SFC_STA_ADDR0       0x0088
#define SFC_STA_ADDR1       0x008C
#define SFC_DES_ADDR        0x0090
#define SFC_GLB1            0x0094
#define SFC_DEV1_STA_RT     0x0098
#define SFC_TRAN_CONF1_BASE 0x009C
#define SFC_TRAN_CONF1_END  0x00B4
#define SFC_CDT_BASE        0x0800
#define SFC_CDT_END         0x0C00
#define SFC_DR              0x1000

/* SFC_TRIG bits */
#define TRIG_START      (1 << 0)
#define TRIG_STOP       (1 << 1)
#define TRIG_FLUSH      (1 << 2)

/* SFC_SR bits */
#define SR_RECE_REQ     (1 << 2)
#define SR_TRAN_REQ     (1 << 3)
#define SR_END          (1 << 4)

/* SFC_TRAN_CONF bits, also word 1 of a CDT entry */
#define TRAN_CMD_MSK    0xFFFF
#define TRAN_DATEEN     (1 << 16)
#define TRAN_POLL_EN    (1 << 25)

/* SFC_GLB bits */
#define GLB_OP_MODE     (1 << 6)
#define GLB_TRAN_DIR    (1 << 13)
#define GLB_CDT_EN      (1 << 14)
#define GLB_DES_EN      (1 << 15)

/* SFC_CMD_IDX bits, and the LINK bit in word 0 of a CDT entry */
#define CMD_IDX_WRITE   (1u << 30)
#define CMD_IDX_DATAEN  (1u << 31)
#define CDT_LINK        (1u << 31)

/* SPI NOR commands */
#define SPI_CMD_READ            0x03
#define SPI_CMD_FAST_READ       0x0B
#define SPI_CMD_READ_ID         0x9F
#define SPI_CMD_READ_STATUS     0x05
#define SPI_CMD_WRITE_ENABLE    0x06
#define SPI_CMD_WRITE_DISABLE   0x04
#define SPI_CMD_PAGE_PROGRAM    0x02
#define SPI_CMD_ERASE_4K        0x20
#define SPI_CMD_ERASE_32K       0x52
#define SPI_CMD_ERASE_64K       0xD8
#define SPI_CMD_ERASE_CHIP      0x60
#define SPI_CMD_READ_SR2        0x35
#define SPI_CMD_READ_SR3        0x15
#define SPI_CMD_WRITE_SR        0x01
#define SPI_CMD_WRITE_SR2       0x31
#define SPI_CMD_WRITE_SR3       0x11
#define SPI_CMD_READ_SFDP       0x5A

static uint32_t ingenic_a1_sfc_sr(IngenicA1SfcState *s)
{
    uint32_t available = s->fifo_len - s->fifo_pos;
    if (available > 0x7f) {
        available = 0x7f;
    }
    return (s->sr & 0xffff) | (available << 16);
}

static void ingenic_a1_sfc_update_irq(IngenicA1SfcState *s)
{
    /*
     * SFC_INTC is a mask register: bit set = source MASKED. The
     * kernel writes 0x1f to mask all and 0 to unmask all.
     */
    qemu_set_irq(s->irq, (s->sr & ~s->intc) != 0);
}

static uint32_t sfc_get_threshold(IngenicA1SfcState *s)
{
    uint32_t t = (s->glb >> 7) & 0x3F;
    return t ? t : 31;
}

static void ingenic_a1_sfc_fill_fifo(IngenicA1SfcState *s)
{
    uint32_t threshold = sfc_get_threshold(s);
    uint32_t addr = s->dev_addr[0] + s->flash_pos * 4;
    uint32_t remaining = s->words_total - s->flash_pos;
    uint32_t chunk = remaining > threshold ? threshold : remaining;
    uint32_t i;

    for (i = 0; i < chunk; i++) {
        uint32_t faddr = addr + i * 4;
        if (s->fill_const) {
            s->fifo[i] = s->fill_word;
        } else if (s->nand_id) {
            uint32_t off = (s->flash_pos + i) * 4;

            s->fifo[i] = off + 4 <= sizeof(s->nand_buf) ?
                         ldl_le_p(&s->nand_buf[off]) : 0xFFFFFFFF;
        } else if (faddr < s->flash_size && faddr + 4 <= s->flash_size) {
            memcpy(&s->fifo[i], &s->flash_data[faddr], 4);
        } else {
            s->fifo[i] = 0xFFFFFFFF;
        }
    }
    s->fifo_pos = 0;
    s->fifo_len = chunk;
}

/* A register read: the flash answers one byte */
static void ingenic_a1_sfc_reply(IngenicA1SfcState *s, uint32_t v)
{
    s->fifo[0] = v;
    s->fifo_len = 1;
    s->fifo_pos = 0;
    s->words_total = 1;
    s->sr = SR_RECE_REQ;
}

/* The data of a WRSR/WRSR2/WRSR3 burst arrived */
static void ingenic_a1_sfc_status_data(IngenicA1SfcState *s, uint32_t v)
{
    switch (s->status_cmd) {
    case SPI_CMD_WRITE_SR:
        /* WRSR with two data bytes also carries SR2 (QE) */
        if (s->tran_len >= 2) {
            s->sr2 = (v >> 8) & 0xFF;
        }
        break;
    case SPI_CMD_WRITE_SR2:
        s->sr2 = v & 0xFF;
        break;
    case SPI_CMD_WRITE_SR3:
        s->sr3 = v & 0xFF;
        break;
    }
    s->status_cmd = 0;
    s->write_enabled = false;
    s->sr &= ~SR_TRAN_REQ;
    s->sr |= SR_END;
}

/*
 * SPI NAND, chosen by a non-zero nand-id ((MID << 8) | DID): 2 KiB pages
 * with a 128-byte spare area, 64 pages a block, nand-blocks blocks, the
 * GigaDevice feature registers. The image holds the page data back to
 * back, as thingino builds it; the spare areas live in RAM, all 0xff at
 * power-on, so every block reads good. Operations finish at once and
 * ECC never reports a bitflip. Any BP bit locks the whole array.
 *
 * U-Boot issues one command per TRAN_CONF0 and moves the data by PIO.
 * Linux sets CDT_EN and starts a chain of CDT entries (CMD_IDX names the
 * first, each LINK bit the next): a command, a status poll, the data
 * phase, by PIO or by DMA through the descriptor chain at DES_ADDR. A
 * PIO data phase holds the chain until the guest has moved the data.
 */
#define NAND_PAGE           INGENIC_A1_SFC_NAND_PAGE
#define NAND_OOB            INGENIC_A1_SFC_NAND_OOB
#define NAND_PPB            64

#define NAND_FEAT_LOCK      0
#define NAND_FEAT_CFG       1
#define NAND_FEAT_STATUS    2
#define NAND_LOCK_BP        0x38
#define NAND_CFG_ECC_EN     0x10
#define NAND_STS_WEL        0x02
#define NAND_STS_E_FAIL     0x04
#define NAND_STS_P_FAIL     0x08
#define NAND_STS_ECC        0x70

static int nand_feat_index(uint32_t addr)
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

static void nand_power_on(IngenicA1SfcState *s)
{
    s->nand_feat[NAND_FEAT_LOCK] = NAND_LOCK_BP;
    s->nand_feat[NAND_FEAT_CFG] = NAND_CFG_ECC_EN;
    s->nand_feat[NAND_FEAT_STATUS] = 0;
    s->nand_feat[3] = 0;
    memset(s->nand_cache, 0xff, sizeof(s->nand_cache));
}

static uint32_t nand_pages(IngenicA1SfcState *s)
{
    return s->nand_blocks * NAND_PPB;
}

/* PROGRAM EXECUTE and BLOCK ERASE need WEL and an unlocked array */
static bool nand_writable(IngenicA1SfcState *s, uint32_t row)
{
    return (s->nand_feat[NAND_FEAT_STATUS] & NAND_STS_WEL) &&
           !(s->nand_feat[NAND_FEAT_LOCK] & NAND_LOCK_BP) &&
           row < nand_pages(s);
}

static void nand_page_read(IngenicA1SfcState *s, uint32_t row)
{
    if (row < nand_pages(s)) {
        memcpy(s->nand_cache, &s->flash_data[(size_t)row * NAND_PAGE],
               NAND_PAGE);
        memcpy(&s->nand_cache[NAND_PAGE],
               &s->nand_oob[(size_t)row * NAND_OOB], NAND_OOB);
    } else {
        memset(s->nand_cache, 0xff, sizeof(s->nand_cache));
    }
    s->nand_feat[NAND_FEAT_STATUS] &= ~NAND_STS_ECC;
}

static void nand_program(IngenicA1SfcState *s, uint32_t row)
{
    uint8_t *st = &s->nand_feat[NAND_FEAT_STATUS];

    if (nand_writable(s, row)) {
        uint8_t *pg = &s->flash_data[(size_t)row * NAND_PAGE];
        uint8_t *oob = &s->nand_oob[(size_t)row * NAND_OOB];

        for (int i = 0; i < NAND_PAGE; i++) {
            pg[i] &= s->nand_cache[i];
        }
        for (int i = 0; i < NAND_OOB; i++) {
            oob[i] &= s->nand_cache[NAND_PAGE + i];
        }
        ingenic_a1_sfc_writeback(s, row * NAND_PAGE, NAND_PAGE);
        *st &= ~NAND_STS_P_FAIL;
    } else {
        *st |= NAND_STS_P_FAIL;
    }
    *st &= ~NAND_STS_WEL;
}

static void nand_erase(IngenicA1SfcState *s, uint32_t row)
{
    uint8_t *st = &s->nand_feat[NAND_FEAT_STATUS];
    uint32_t first = row - row % NAND_PPB;

    if (nand_writable(s, row)) {
        memset(&s->flash_data[(size_t)first * NAND_PAGE], 0xff,
               NAND_PPB * NAND_PAGE);
        memset(&s->nand_oob[(size_t)first * NAND_OOB], 0xff,
               NAND_PPB * NAND_OOB);
        ingenic_a1_sfc_writeback(s, first * NAND_PAGE, NAND_PPB * NAND_PAGE);
        *st &= ~NAND_STS_E_FAIL;
    } else {
        *st |= NAND_STS_E_FAIL;
    }
    *st &= ~NAND_STS_WEL;
}

/* One command on the bus; what it answers lands in nand_buf */
static void nand_command(IngenicA1SfcState *s, uint32_t xfer, uint32_t addr)
{
    uint8_t cmd = xfer & 0xff;
    uint8_t *st = &s->nand_feat[NAND_FEAT_STATUS];
    uint32_t col = addr & 0xfff;

    s->nand_cmd = cmd;
    s->nand_addr = addr;
    memset(s->nand_buf, 0xff, sizeof(s->nand_buf));

    switch (cmd) {
    case 0xff:      /* RESET */
        *st = 0;
        break;
    case 0x9f: {    /* READ ID: MID, DID, repeating, after a dummy byte */
        bool dummy = ((xfer >> 26) & 7) || ((xfer >> 17) & 0x3f);
        uint8_t id[9] = { 0xff };

        for (int i = 1; i < 9; i++) {
            id[i] = (i & 1) ? s->nand_id >> 8 : s->nand_id;
        }
        memcpy(s->nand_buf, dummy ? &id[1] : id, 8);
        break;
    }
    case 0x0f: {    /* GET FEATURE */
        int i = nand_feat_index(addr);

        s->nand_buf[0] = i < 0 ? 0 : s->nand_feat[i];
        break;
    }
    case 0x06:      /* WRITE ENABLE */
        *st |= NAND_STS_WEL;
        break;
    case 0x04:      /* WRITE DISABLE */
        *st &= ~NAND_STS_WEL;
        break;
    case 0x13:      /* PAGE READ, to the cache */
        nand_page_read(s, addr & 0xffffff);
        break;
    case 0x03:      /* READ FROM CACHE x1, x2, x4, dual and quad I/O */
    case 0x0b:
    case 0x3b:
    case 0xbb:
    case 0x6b:
    case 0xeb:
        if (col < sizeof(s->nand_cache)) {
            memcpy(s->nand_buf, &s->nand_cache[col],
                   sizeof(s->nand_cache) - col);
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
        nand_program(s, addr & 0xffffff);
        break;
    case 0xd8:      /* BLOCK ERASE */
        nand_erase(s, addr & 0xffffff);
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "ingenic-a1-sfc: SPI NAND command 0x%02x\n",
                      cmd);
        break;
    }
}

/* The data a SET FEATURE or PROGRAM LOAD sent sits in nand_buf */
static void nand_write_done(IngenicA1SfcState *s, uint32_t len)
{
    static const uint8_t wmask[4] = { 0xbe, 0x51, 0x00, 0x60 };
    uint32_t col = s->nand_addr & 0xfff;
    int i;

    switch (s->nand_cmd) {
    case 0x1f:
        i = nand_feat_index(s->nand_addr);
        if (i >= 0) {
            s->nand_feat[i] = (s->nand_feat[i] & ~wmask[i]) |
                              (s->nand_buf[0] & wmask[i]);
        }
        break;
    case 0x02:      /* PROGRAM LOAD clears the cache, RANDOM DATA does not */
    case 0x32:
        memset(s->nand_cache, 0xff, sizeof(s->nand_cache));
        /* fall through */
    case 0x84:
    case 0x34:
    case 0xc4:
        if (col < sizeof(s->nand_cache)) {
            memcpy(&s->nand_cache[col], s->nand_buf,
                   MIN(len, sizeof(s->nand_cache) - col));
        }
        break;
    }
}

/* Move len bytes between nand_buf and RAM: descriptor chain or MEM_ADDR */
static void nand_dma(IngenicA1SfcState *s, uint32_t len, bool to_ram)
{
    uint32_t des = s->v2_regs[0] & 0x1FFFFFFF;
    uint32_t done = 0;

    for (int n = 0; done < len && n < 256; n++) {
        uint32_t mem = s->mem_addr & 0x1FFFFFFF;
        uint32_t chunk = len - done;
        bool more = false;

        if (s->glb & GLB_DES_EN) {
            uint8_t d[16];      /* next, mem_addr, tran_len, link */

            if (!des) {
                break;
            }
            physical_memory_read(des, d, sizeof(d));
            mem = ldl_le_p(&d[4]) & 0x1FFFFFFF;
            chunk = MIN(ldl_le_p(&d[8]), chunk);
            more = ldl_le_p(&d[12]);
            des = ldl_le_p(&d[0]) & 0x1FFFFFFF;
        }
        if (to_ram) {
            physical_memory_write(mem, &s->nand_buf[done], chunk);
        } else {
            physical_memory_read(mem, &s->nand_buf[done], chunk);
        }
        done += chunk;
        if (!more) {
            break;
        }
    }
}

/* The address register a CDT entry names: COL, ROW, STA0 or STA1 */
static uint32_t nand_cdt_addr(IngenicA1SfcState *s, uint32_t link)
{
    switch (link & 3) {
    case 0:
        return s->col_addr;
    case 1:
        return s->row_addr;
    default:
        return s->sta_addr[(link & 3) - 2];
    }
}

/*
 * Run the transfer from CDT entry idx, or the one in TRAN_CONF0 when
 * CDT_EN is clear, to the end of the chain or to a PIO data phase.
 */
static void nand_run(IngenicA1SfcState *s, unsigned idx)
{
    bool cdt = s->glb & GLB_CDT_EN;
    bool out = cdt ? s->cmd_idx & CMD_IDX_WRITE : s->glb & GLB_TRAN_DIR;
    uint32_t len = MIN(s->tran_len, (uint32_t)sizeof(s->nand_buf));

    for (unsigned n = 0; n < INGENIC_A1_SFC_CDT_ENTRIES; n++) {
        uint32_t link = cdt ? s->cdt[idx * 4] : 0;
        uint32_t xfer = cdt ? s->cdt[idx * 4 + 1] : s->tran_conf[0];
        uint32_t addr = cdt ? nand_cdt_addr(s, link) : s->dev_addr[0];
        uint32_t width = (xfer >> 26) & 7;
        bool data = (xfer & TRAN_DATEEN) && len &&
                    (!cdt || (s->cmd_idx & CMD_IDX_DATAEN));

        if (width < 4) {
            addr &= (1ULL << (8 * width)) - 1;
        }
        idx = (idx + 1) % INGENIC_A1_SFC_CDT_ENTRIES;
        nand_command(s, xfer, addr);
        if (xfer & TRAN_POLL_EN) {
            /* The flash is never busy: the first status read matches */
            s->dev_sta_rt = s->nand_buf[0];
        } else if (data && !(s->glb & GLB_OP_MODE)) {
            s->nand_resume = cdt && (link & CDT_LINK) ? idx : -1;
            s->flash_pos = 0;
            s->words_total = (s->tran_len + 3) / 4;
            if (out) {
                s->nand_pio_out = true;
                s->sr |= SR_TRAN_REQ;
            } else {
                ingenic_a1_sfc_fill_fifo(s);
                s->sr |= SR_RECE_REQ;
            }
            return;
        } else if (data) {
            nand_dma(s, len, !out);
            if (out) {
                nand_write_done(s, len);
            }
        }
        if (!(link & CDT_LINK)) {
            break;
        }
    }
    s->sr |= SR_END;
}

/* The guest moved a PIO data phase: apply it, run the rest of the chain */
static void nand_pio_done(IngenicA1SfcState *s)
{
    int next = s->nand_resume;

    if (s->nand_pio_out) {
        s->nand_pio_out = false;
        s->sr &= ~SR_TRAN_REQ;
        nand_write_done(s, MIN(s->tran_len, (uint32_t)sizeof(s->nand_buf)));
    }
    s->nand_resume = -1;
    if (next >= 0) {
        nand_run(s, next);
    } else {
        s->sr |= SR_END;
    }
}

static void nand_dr_write(IngenicA1SfcState *s, uint32_t v)
{
    uint32_t off = s->flash_pos * 4;

    if (!s->nand_pio_out) {
        return;
    }
    if (off + 4 <= sizeof(s->nand_buf)) {
        stl_le_p(&s->nand_buf[off], v);
    }
    if (++s->flash_pos >= s->words_total) {
        nand_pio_done(s);
    } else if ((s->flash_pos % sfc_get_threshold(s)) == 0) {
        s->sr |= SR_TRAN_REQ;
    }
}

static void ingenic_a1_sfc_do_transfer(IngenicA1SfcState *s)
{
    bool to_flash = (s->glb & GLB_TRAN_DIR) || (s->cmd_idx & (1u << 30));
    bool from_cdt = true;
    uint32_t cmd;
    uint32_t cdt_index = s->cmd_idx & 0x3F;
    uint32_t cdt_xfer = s->cdt[cdt_index * 4 + 1];

    if (s->nand_id) {
        s->fifo_pos = 0;
        s->fifo_len = 0;
        s->flash_pos = 0;
        s->words_total = 0;
        s->fill_const = false;
        s->nand_pio_out = false;
        s->nand_resume = -1;
        s->sr = 0;
        nand_run(s, cdt_index);
        return;
    }

    /*
     * DMA descriptor chain mode: the kernel SFC driver sets GLB.DES_EN
     * and writes a descriptor chain address to SFC_DES_ADDR (0x90).
     * Each descriptor has: next_des_addr, mem_addr, tran_len, link.
     * Walk the chain, transfer data between flash and guest RAM.
     */
    if ((s->glb & GLB_DES_EN) && s->tran_len > 0) {
        uint32_t des_phys = s->v2_regs[0] & 0x1FFFFFFF;
        uint32_t flash_addr = s->row_addr ? s->row_addr : s->dev_addr[0];
        bool is_write = !!(s->cmd_idx & (1u << 30));

        if (des_phys) {
            uint32_t desc[4];
            physical_memory_read(des_phys, desc, 16);
            if (desc[1] || desc[2]) {
                uint32_t max_iter = 256;
                while (des_phys && max_iter--) {
                    physical_memory_read(des_phys, desc, 16);
                    uint32_t mem_phys = desc[1] & 0x1FFFFFFF;
                    uint32_t tran_len = desc[2];
                    if (mem_phys && tran_len > 0 &&
                        tran_len <= 16 * 1024 * 1024 &&
                        flash_addr < s->flash_size) {
                        uint32_t avail = s->flash_size - flash_addr;
                        uint32_t len = tran_len < avail ? tran_len : avail;
                        if (is_write) {
                            g_autofree uint8_t *buf = g_malloc(len);
                            physical_memory_read(mem_phys, buf, len);
                            for (uint32_t j = 0; j < len; j++) {
                                s->flash_data[flash_addr + j] &= buf[j];
                            }
                            ingenic_a1_sfc_writeback(s, flash_addr, len);
                        } else {
                            physical_memory_write(mem_phys,
                                                  &s->flash_data[flash_addr],
                                                  len);
                        }
                        flash_addr += len;
                    }
                    if (desc[3] == 0 || desc[0] == 0) {
                        break;
                    }
                    des_phys = desc[0] & 0x1FFFFFFF;
                }
                if (is_write) {
                    s->write_enabled = false;
                }
                s->sr = SR_END;
                ingenic_a1_sfc_update_irq(s);
                return;
            }
        }
        /* No valid descriptor: fall through to PIO path */
    }

    if (cdt_xfer != 0) {
        cmd = cdt_xfer & 0xFF;
        s->dev_addr[0] = s->row_addr;
    } else {
        uint32_t cdt_word0 = s->cdt[cdt_index * 4];
        if (cdt_word0 != 0) {
            cmd = cdt_word0 & 0xFF;
            s->dev_addr[0] = s->row_addr;
        } else {
            from_cdt = false;
            cmd = s->tran_conf[0] & TRAN_CMD_MSK;
            if (s->row_addr != 0) {
                s->dev_addr[0] = s->row_addr;
            }
        }
    }

    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->sr = 0;
    s->fill_const = false;
    s->discard = false;

    if (s->absent) {
        /* Nothing answers: reads see the data lines high */
        if (!to_flash && s->tran_len) {
            s->words_total = (s->tran_len + 3) / 4;
            s->fill_const = true;
            s->fill_word = 0xFFFFFFFF;
            ingenic_a1_sfc_fill_fifo(s);
            s->sr = SR_RECE_REQ;
        } else {
            s->sr = SR_END;
        }
        return;
    }

    /*
     * CDT multi-phase: if the command is WRITE_ENABLE and the CDT LINK
     * bit (bit 31 of word0) is set, the real operation is in the next
     * CDT entry. Follow the link to find the actual erase/program cmd.
     */
    if (cmd == SPI_CMD_WRITE_ENABLE) {
        uint32_t w0 = s->cdt[cdt_index * 4];
        if (w0 & (1u << 31)) {
            uint32_t next_idx = (cdt_index + 1) & 0x3F;
            uint32_t nw0 = s->cdt[next_idx * 4];
            uint32_t nw1 = s->cdt[next_idx * 4 + 1];
            uint32_t next_cmd = nw1 ? (nw1 & 0xFF) : (nw0 & 0xFF);
            if (next_cmd) {
                s->write_enabled = true;
                cmd = next_cmd;
            }
        }
    }

    switch (cmd) {
    case SPI_CMD_READ:
    case SPI_CMD_FAST_READ:
        if (s->row_addr) {
            s->dev_addr[0] = s->row_addr;
        }
        s->words_total = (s->tran_len + 3) / 4;
        ingenic_a1_sfc_fill_fifo(s);
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_ID:
        ingenic_a1_sfc_reply(s, 0x001840EF);   /* W25Q128 */
        break;

    case SPI_CMD_READ_STATUS:
        ingenic_a1_sfc_reply(s, s->write_enabled ? 0x02 : 0x00);
        break;

    case SPI_CMD_READ_SR2:
        ingenic_a1_sfc_reply(s, s->sr2);
        break;

    case SPI_CMD_READ_SR3:
        ingenic_a1_sfc_reply(s, s->sr3);
        break;

    case SPI_CMD_READ_SFDP:
        /* No SFDP table: all ones, like a part without one */
        s->fill_const = true;
        s->fill_word = 0xFFFFFFFF;
        s->words_total = (s->tran_len + 3) / 4;
        ingenic_a1_sfc_fill_fifo(s);
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_WRITE_SR:
    case SPI_CMD_WRITE_SR2:
    case SPI_CMD_WRITE_SR3:
        /* The data arrives through SFC_DR like a page program */
        if (to_flash && s->tran_len) {
            s->words_total = (s->tran_len + 3) / 4;
            s->status_cmd = cmd;
            s->sr = SR_TRAN_REQ;
        } else {
            s->write_enabled = false;
            s->sr = SR_END;
        }
        break;

    case SPI_CMD_WRITE_ENABLE:
        s->write_enabled = true;
        s->sr = SR_END;
        break;

    case SPI_CMD_WRITE_DISABLE:
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_PAGE_PROGRAM:
        if ((s->glb & GLB_TRAN_DIR) || (s->cmd_idx & (1u << 30))) {
            s->words_total = (s->tran_len + 3) / 4;
            s->writing = true;
            s->sr = SR_TRAN_REQ;
        } else {
            s->sr = SR_END;
        }
        break;

    case SPI_CMD_ERASE_4K:
    case SPI_CMD_ERASE_32K:
    case SPI_CMD_ERASE_64K:
    {
        uint32_t erase_sz = (cmd == SPI_CMD_ERASE_4K) ? 4096 :
                             (cmd == SPI_CMD_ERASE_32K) ? 32768 : 65536;
        uint32_t addr = s->dev_addr[0];

        if (s->write_enabled && addr < s->flash_size &&
            (s->flash_size - addr) >= erase_sz) {
            memset(&s->flash_data[addr], 0xFF, erase_sz);
            ingenic_a1_sfc_writeback(s, addr, erase_sz);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;
    }

    case SPI_CMD_ERASE_CHIP:
        if (s->write_enabled) {
            memset(s->flash_data, 0xFF, s->flash_size);
            ingenic_a1_sfc_writeback(s, 0, s->flash_size);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    default:
        /*
         * The controller runs the data phase it is given whatever the
         * opcode: reads see zeros, written data goes nowhere.
         */
        if (!from_cdt && (s->tran_conf[0] & TRAN_DATEEN) && s->tran_len) {
            s->words_total = (s->tran_len + 3) / 4;
            if (to_flash) {
                s->discard = true;
                s->sr = SR_TRAN_REQ;
            } else {
                s->fill_const = true;
                s->fill_word = 0;
                ingenic_a1_sfc_fill_fifo(s);
                s->sr = SR_RECE_REQ;
            }
            break;
        }
        s->sr = SR_END;
        break;
    }
}

static uint64_t ingenic_a1_sfc_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(opaque);

    switch (offset) {
    case SFC_GLB:
        return s->glb;
    case SFC_DEV_CONF:
        return s->dev_conf;
    case SFC_DEV_STA_RT:
        return s->dev_sta_rt;
    case SFC_DEV_STA_EXP:
    case SFC_DEV_STA_MSK:
        return 0;
    case SFC_TRAN_CONF0:
        return s->tran_conf[0];
    case SFC_TRAN_LEN:
        return s->tran_len;
    case SFC_DEV_ADDR0:
        return s->dev_addr[0];
    case SFC_TRIG:
        return 0;
    case SFC_SR:
        return ingenic_a1_sfc_sr(s);
    case SFC_INTC:
        return s->intc;
    case SFC_CGE:
        return s->cge;
    case SFC_CMD_IDX:
        return s->cmd_idx;
    case SFC_COL_ADDR:
        return s->col_addr;
    case SFC_ROW_ADDR:
        return s->row_addr;
    case SFC_STA_ADDR0:
    case SFC_STA_ADDR1:
        return s->sta_addr[(offset - SFC_STA_ADDR0) / 4];

    case SFC_DR:
        if (s->fifo_pos < s->fifo_len) {
            uint32_t val = s->fifo[s->fifo_pos++];
            s->flash_pos++;
            if (s->fifo_pos >= s->fifo_len) {
                s->sr &= ~SR_RECE_REQ;
                if (s->flash_pos >= s->words_total && s->nand_id) {
                    nand_pio_done(s);
                } else if (s->flash_pos >= s->words_total) {
                    s->sr |= SR_END;
                } else {
                    ingenic_a1_sfc_fill_fifo(s);
                    s->sr |= SR_RECE_REQ;
                }
                ingenic_a1_sfc_update_irq(s);
            }
            return val;
        }
        return 0xFFFFFFFF;

    default:
        /* V2 extended registers: DES_ADDR through TRAN_CONF1[5] */
        if (offset >= SFC_DES_ADDR && offset < SFC_TRAN_CONF1_END) {
            return s->v2_regs[(offset - SFC_DES_ADDR) / 4];
        }
        /* TRAN_CONF[1..5] at offsets 0x18..0x28 */
        if (offset >= 0x18 && offset <= 0x28) {
            return 0;
        }
        /* CDT read-back */
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_A1_SFC_CDT_ENTRIES * 4) {
                return s->cdt[idx];
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read (offset 0x%04" HWADDR_PRIx ")\n",
                      __func__, offset);
        return 0;
    }
}

static void ingenic_a1_sfc_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(opaque);

    switch (offset) {
    case SFC_DEV_STA_EXP:
    case SFC_DEV_STA_MSK:
        break;
    case SFC_GLB:
        s->glb = (uint32_t)value;
        break;
    case SFC_DEV_CONF:
        s->dev_conf = (uint32_t)value;
        break;
    case SFC_TRAN_CONF0:
        s->tran_conf[0] = (uint32_t)value;
        break;
    case SFC_TRAN_LEN:
        s->tran_len = (uint32_t)value;
        break;
    case SFC_DEV_ADDR0:
        s->dev_addr[0] = (uint32_t)value;
        break;
    case SFC_DEV_ADDR_PLUS0:
        s->dev_addr_plus[0] = (uint32_t)value;
        break;
    case SFC_MEM_ADDR:
        s->mem_addr = (uint32_t)value;
        break;
    case SFC_INTC:
        s->intc = (uint32_t)value;
        ingenic_a1_sfc_update_irq(s);
        break;
    case SFC_CGE:
        s->cge = (uint32_t)value;
        break;
    case SFC_CMD_IDX:
        s->cmd_idx = (uint32_t)value;
        break;
    case SFC_COL_ADDR:
        s->col_addr = (uint32_t)value;
        break;
    case SFC_ROW_ADDR:
        s->row_addr = (uint32_t)value;
        break;
    case SFC_STA_ADDR0:
    case SFC_STA_ADDR1:
        s->sta_addr[(offset - SFC_STA_ADDR0) / 4] = (uint32_t)value;
        break;

    case SFC_TRIG:
        if (value & TRIG_FLUSH) {
            s->fifo_pos = 0;
            s->fifo_len = 0;
        }
        if (value & TRIG_STOP) {
            s->sr = 0;
            s->nand_pio_out = false;
            s->nand_resume = -1;
        }
        if (value & TRIG_START) {
            ingenic_a1_sfc_do_transfer(s);
        }
        ingenic_a1_sfc_update_irq(s);
        break;

    case SFC_DR:
        if (s->nand_id) {
            nand_dr_write(s, (uint32_t)value);
            ingenic_a1_sfc_update_irq(s);
            break;
        }
        if (s->status_cmd) {
            ingenic_a1_sfc_status_data(s, (uint32_t)value);
            ingenic_a1_sfc_update_irq(s);
            break;
        }
        if (s->discard) {
            if (++s->flash_pos >= s->words_total) {
                s->discard = false;
                s->sr &= ~SR_TRAN_REQ;
                s->sr |= SR_END;
            }
            ingenic_a1_sfc_update_irq(s);
            break;
        }
        if (s->writing) {
            uint32_t faddr = s->dev_addr[0] + s->flash_pos * 4;
            if (s->write_enabled && faddr < s->flash_size &&
                faddr + 4 <= s->flash_size) {
                uint32_t v = (uint32_t)value;
                s->flash_data[faddr]     &= v & 0xFF;
                s->flash_data[faddr + 1] &= (v >> 8) & 0xFF;
                s->flash_data[faddr + 2] &= (v >> 16) & 0xFF;
                s->flash_data[faddr + 3] &= (v >> 24) & 0xFF;
            }
            s->flash_pos++;
            if (s->flash_pos >= s->words_total) {
                if (s->write_enabled || s->writing) {
                    uint32_t bytes = s->flash_pos * 4;
                    ingenic_a1_sfc_writeback(s, s->dev_addr[0], bytes);
                }
                s->writing = false;
                s->write_enabled = false;
                s->sr &= ~SR_TRAN_REQ;
                s->sr |= SR_END;
            } else if ((s->flash_pos % sfc_get_threshold(s)) == 0) {
                s->sr |= SR_TRAN_REQ;
            }
            ingenic_a1_sfc_update_irq(s);
        }
        break;

    case SFC_SCR:
        s->sr &= ~(uint32_t)value;
        ingenic_a1_sfc_update_irq(s);
        break;

    default:
        /* V2 extended registers */
        if (offset >= SFC_DES_ADDR && offset < SFC_TRAN_CONF1_END) {
            s->v2_regs[(offset - SFC_DES_ADDR) / 4] = (uint32_t)value;
            break;
        }
        /* TRAN_CONF[1..5] at offsets 0x18..0x28 */
        if (offset >= 0x18 && offset <= 0x28) {
            break;
        }
        /* CDT writes */
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_A1_SFC_CDT_ENTRIES * 4) {
                s->cdt[idx] = (uint32_t)value;
            }
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented write (offset 0x%04" HWADDR_PRIx
                      ", value 0x%08" PRIx64 ")\n",
                      __func__, offset, value);
        break;
    }
}

static const MemoryRegionOps ingenic_a1_sfc_ops = {
    .read = ingenic_a1_sfc_read,
    .write = ingenic_a1_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_a1_sfc_reset_hold(Object *obj, ResetType type)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(obj);

    s->glb = 0;
    s->dev_conf = 0;
    memset(s->tran_conf, 0, sizeof(s->tran_conf));
    s->tran_len = 0;
    memset(s->dev_addr, 0, sizeof(s->dev_addr));
    memset(s->dev_addr_plus, 0, sizeof(s->dev_addr_plus));
    s->mem_addr = 0;
    s->sr = 0;
    s->intc = 0x1f;
    s->cge = 0;
    s->cmd_idx = 0;
    s->col_addr = 0;
    s->row_addr = 0;
    memset(s->sta_addr, 0, sizeof(s->sta_addr));
    s->dev_sta_rt = 0;
    s->des_addr = 0;
    s->glb1 = 0;
    s->dev1_sta_rt = 0;
    memset(s->tran_conf1, 0, sizeof(s->tran_conf1));
    memset(s->v2_regs, 0, sizeof(s->v2_regs));
    memset(s->cdt, 0, sizeof(s->cdt));
    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->write_enabled = false;
    s->writing = false;
    s->discard = false;
    s->fill_const = false;
    s->fill_word = 0;
    s->sr2 = 0;
    s->sr3 = 0;
    s->status_cmd = 0;
    /* A SoC reset leaves the flash chip powered: its state stays */
    s->nand_pio_out = false;
    s->nand_resume = -1;
}

static void ingenic_a1_sfc_realize(DeviceState *dev, Error **errp)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(dev);
    DriveInfo *di;

    if (s->unit) {
        /* A second controller only has the flash given to it */
        di = drive_get(IF_MTD, 0, s->unit);
        s->absent = !di;
        if (di && blk_attach_dev(blk_by_legacy_dinfo(di), dev) < 0) {
            error_setg(errp, "SPI flash drive unit %u is already in use",
                       s->unit);
            return;
        }
    } else {
        di = drive_get(IF_MTD, 0, 0);
        if (!di) {
            di = drive_get(IF_PFLASH, 0, 0);
        }
        if (!di) {
            di = drive_get(IF_NONE, 0, 0);
        }
    }
    if (s->absent) {
        s->nand_id = 0;
    }
    if (s->nand_id && (!s->nand_blocks || s->nand_blocks > 16384)) {
        error_setg(errp, "nand-blocks must be 1 to 16384");
        return;
    }
    if (s->nand_id) {
        size_t oob = (size_t)nand_pages(s) * NAND_OOB;

        s->flash_size = nand_pages(s) * NAND_PAGE;
        s->nand_oob = g_malloc(oob);
        memset(s->nand_oob, 0xFF, oob);
        nand_power_on(s);
    } else {
        s->flash_size = s->absent ? 0 : INGENIC_A1_SFC_FLASH_SIZE;
    }
    s->flash_data = g_malloc0(s->flash_size);
    memset(s->flash_data, 0xFF, s->flash_size);
    if (di) {
        BlockBackend *blk = blk_by_legacy_dinfo(di);
        int64_t size = blk_getlength(blk);

        s->blk_len = MAX(size, 0);
        if (s->nand_id && size < s->flash_size) {
            warn_report("SPI NAND image holds %" PRId64 " of the chip's %"
                        PRIu32 " bytes; what is written past its end is "
                        "lost when QEMU exits", size, s->flash_size);
        }
        if (size > s->flash_size) {
            size = s->flash_size;
        }
        if (blk_pread(blk, 0, size, s->flash_data, 0) < 0) {
            error_setg(errp, "failed to read SPI flash image");
            return;
        }
        if (blk_supports_write_perm(blk)) {
            uint64_t perm = BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE;
            if (blk_set_perm(blk, perm, BLK_PERM_ALL, NULL) == 0) {
                s->blk = blk;
            }
        }
    }
}

static void ingenic_a1_sfc_init(Object *obj)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_a1_sfc_ops, s,
                          TYPE_INGENIC_A1_SFC, INGENIC_A1_SFC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_a1_sfc_finalize(Object *obj)
{
    IngenicA1SfcState *s = INGENIC_A1_SFC(obj);

    g_free(s->flash_data);
    g_free(s->nand_oob);
}

static const Property ingenic_a1_sfc_props[] = {
    DEFINE_PROP_UINT8("unit", IngenicA1SfcState, unit, 0),
    DEFINE_PROP_UINT32("nand-id", IngenicA1SfcState, nand_id, 0),
    DEFINE_PROP_UINT32("nand-blocks", IngenicA1SfcState, nand_blocks, 1024),
};

static void ingenic_a1_sfc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_props(dc, ingenic_a1_sfc_props);
    dc->realize = ingenic_a1_sfc_realize;
    rc->phases.hold = ingenic_a1_sfc_reset_hold;
}

static const TypeInfo ingenic_a1_sfc_type_info = {
    .name = TYPE_INGENIC_A1_SFC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1SfcState),
    .instance_init = ingenic_a1_sfc_init,
    .instance_finalize = ingenic_a1_sfc_finalize,
    .class_init = ingenic_a1_sfc_class_init,
};

static void ingenic_a1_sfc_register_types(void)
{
    type_register_static(&ingenic_a1_sfc_type_info);
}

type_init(ingenic_a1_sfc_register_types)
