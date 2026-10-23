/*
 * Ingenic T31 SPI Flash Controller (SFC) emulation
 *
 * Implements PIO (CPU mode) read from a backing flash image. The flash
 * image is loaded from the first -drive with if=mtd on the command line.
 * A non-zero nand-id makes the flash a SPI NAND (hw/misc/ingenic-spinand.c).
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
#include "hw/misc/ingenic-t31-sfc.h"
#include "hw/misc/ingenic-sfc-nor.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/physmem.h"

/*
 * Persist a range of flash_data back to the underlying disk image so
 * that env writes ('saveenv' from U-Boot, etc.) survive across QEMU
 * restarts. blk is NULL when no -drive was attached, in which case we
 * silently keep the changes only in RAM.
 *
 * Errors are logged but not fatal: a write-back failure shouldn't
 * crash the guest, since the in-memory copy is still consistent.
 */
static void ingenic_t31_sfc_writeback(IngenicT31SfcState *s,
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
                      "ingenic-t31-sfc: write-back failed at "
                      "offset 0x%" PRIx32 " len %" PRIu32 ": %s\n",
                      offset, len, strerror(-ret));
    }
}

static uint32_t ingenic_t31_sfc_sr(IngenicT31SfcState *s)
{
    /* SR bits 16-22 are the FIFO entry count read by the driver's
     * sfc_fifo_num() to decide how many words to drain. */
    uint32_t available = s->fifo_len - s->fifo_pos;
    if (available > 0x7f) {
        available = 0x7f;
    }
    return (s->sr & 0xffff) | (available << 16);
}

static void ingenic_t31_sfc_update_irq(IngenicT31SfcState *s)
{
    /*
     * SFC_INTC is a mask register: bit set = source MASKED. The
     * kernel writes 0x1f to mask all and 0 to unmask all.
     */
    qemu_set_irq(s->irq, (s->sr & ~s->intc) != 0);
}

/* Register offsets */
#define SFC_GLB             0x0000
#define SFC_DEV_CONF        0x0004
#define SFC_DEV_STA_EXP     0x0008
#define SFC_DEV_STA_RT      0x000C
#define SFC_DEV_STA_MSK     0x0010
#define SFC_TRAN_CONF0_BASE 0x0014
#define SFC_TRAN_CONF0_END  0x002C
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
#define SR_END          (1 << 4)

/* SFC_TRAN_CONF bits */
#define TRAN_CMD_MSK    0xFFFF
#define TRAN_DATEEN     (1 << 16)

/* SFC_GLB bits */
#define GLB_CDT_EN          (1 << 14)
#define GLB_TRAN_DIR        (1 << 13)
#define GLB_OP_MODE         (1 << 6)    /* 1 = DMA to/from SFC_MEM_ADDR */
#define GLB_PHASE_NUM(g)    (((g) >> 3) & 0x7)

/* SFC_TRAN_CONF bits (in addition to TRAN_CMD_MSK / TRAN_DATEEN) */
#define TRAN_POLLEN         (1 << 25)   /* phase polls flash status */

/* CDT mode: data direction in SFC_CMD_IDX, chaining in the entry's link word */
#define CDT_DIR             (1U << 30)  /* 1 = host to flash */
#define CDT_LINK            (1U << 31)

/* SFC_SR bits */
#define SR_TRAN_REQ     (1 << 3)

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
/*
 * Variants the Ingenic 4.4 sfc-nor driver issues on chips above 16 MiB
 * (GD25Q256 etc.): 4-byte-address opcodes, dual/quad reads and programs,
 * EN4B/EX4B, and the status-register accesses it uses to verify quad and
 * 4-byte mode. The controller registers already carry the full 32-bit
 * device address, so the address width never matters to the model.
 */
#define SPI_CMD_READ_4B         0x13
#define SPI_CMD_FAST_READ_4B    0x0C
#define SPI_CMD_DOR             0x3B
#define SPI_CMD_DOR_4B          0x3C
#define SPI_CMD_QOR             0x6B
#define SPI_CMD_QOR_4B          0x6C
#define SPI_CMD_DIOR            0xBB
#define SPI_CMD_DIOR_4B         0xBC
#define SPI_CMD_QIOR            0xEB
#define SPI_CMD_QIOR_4B         0xEC
#define SPI_CMD_PAGE_PROGRAM_4B 0x12
#define SPI_CMD_QPP             0x32
#define SPI_CMD_QPP_4B          0x34
#define SPI_CMD_QPP_ALT         0x38
#define SPI_CMD_ERASE_4K_4B     0x21
#define SPI_CMD_ERASE_32K_4B    0x5C
#define SPI_CMD_ERASE_64K_4B    0xDC
#define SPI_CMD_ERASE_CHIP_ALT  0xC7
#define SPI_CMD_EN4B            0xB7
#define SPI_CMD_EX4B            0xE9
#define SPI_CMD_READ_SR2        0x35
#define SPI_CMD_READ_SR3        0x15
#define SPI_CMD_WRITE_SR        0x01
#define SPI_CMD_WRITE_SR2       0x31
#define SPI_CMD_WRITE_SR3       0x11
#define SPI_CMD_READ_SFDP       0x5A
#define SPI_CMD_RESET_EN        0x66
#define SPI_CMD_RESET           0x99

/* SR3 bit 0: ADS (4-byte address mode), SR2 bit 1: QE (quad enable) */
#define SR3_ADS                 (1 << 0)
#define SR2_QE                  (1 << 1)

#define SFC_IOSIZE          0x2000

static uint32_t sfc_get_threshold(IngenicT31SfcState *s)
{
    uint32_t t = (s->glb >> 7) & 0x3F;
    return t ? t : 31;
}

static void ingenic_t31_sfc_fill_fifo(IngenicT31SfcState *s)
{
    uint32_t threshold = sfc_get_threshold(s);
    uint32_t addr = s->xfer_addr + s->flash_pos * 4;
    uint32_t remaining = s->words_total - s->flash_pos;
    uint32_t chunk = remaining > threshold ? threshold : remaining;
    uint32_t i;

    for (i = 0; i < chunk; i++) {
        uint32_t faddr = addr + i * 4;
        if (s->nand.id) {
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

/* Data byte(s) of a WRSR/WRSR2/WRSR3 burst arrived (PIO via SFC_DR or DMA). */
static void ingenic_t31_sfc_status_data(IngenicT31SfcState *s, uint32_t v)
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
        s->sr3 = (v & 0xFF & ~SR3_ADS) | (s->sr3 & SR3_ADS);
        break;
    }
    s->status_cmd = 0;
    s->write_enabled = false;
    s->sr &= ~SR_TRAN_REQ;
    s->sr |= SR_END;
}

/*
 * Where the data phase of a DMA transfer lands.
 *
 * The 4.4 sfc-nor driver hands the controller one flat buffer in
 * SFC_MEM_ADDR. The 3.10 ingenic_sfc_v2 driver (T32/PRJ007) instead
 * writes 0 to SFC_MEM_ADDR and points SFC_DES_ADDR at a chain of
 *
 *     struct sfc_desc { next_des_addr; mem_addr; tran_len; link; }
 *
 * with one descriptor per physically contiguous run of the target
 * buffer. Walk the chain by following next_des_addr: `link` cannot be
 * the terminator because create_sfc_desc() leaves it 0 on every
 * descriptor under CONFIG_SOC_PRJ007, linked ones included.
 *
 * A chain is only used when the guest actually asked for one, i.e. it
 * left SFC_MEM_ADDR at 0, so a driver that programs a real address
 * keeps the flat path even if a stale descriptor address is still in
 * the register.
 */
#define SFC_DESC_MAX_CHAIN  256

static void ingenic_t31_sfc_dma_copy(IngenicT31SfcState *s, uint8_t *buf,
                                     uint32_t len, bool to_guest)
{
    uint32_t des_phys = s->des_addr & 0x1FFFFFFF;
    uint32_t done = 0;
    uint32_t i;

    if (!len) {
        return;
    }

    if (!des_phys || s->mem_addr) {
        if (to_guest) {
            physical_memory_write(s->mem_addr, buf, len);
        } else {
            physical_memory_read(s->mem_addr, buf, len);
        }
        return;
    }

    for (i = 0; des_phys && done < len && i < SFC_DESC_MAX_CHAIN; i++) {
        uint32_t desc[4];
        uint32_t mem_phys, tran_len, n;

        physical_memory_read(des_phys, desc, sizeof(desc));
        mem_phys = le32_to_cpu(desc[1]) & 0x1FFFFFFF;
        tran_len = le32_to_cpu(desc[2]);

        n = len - done;
        if (tran_len < n) {
            n = tran_len;
        }
        if (mem_phys && n) {
            if (to_guest) {
                physical_memory_write(mem_phys, buf + done, n);
            } else {
                physical_memory_read(mem_phys, buf + done, n);
            }
            done += n;
        }
        des_phys = le32_to_cpu(desc[0]) & 0x1FFFFFFF;
    }

    if (done < len) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: descriptor chain covered %u of %u bytes\n",
                      __func__, done, len);
    }
}

/*
 * DMA mode (GLB_OP_MODE): the controller moves the data phase itself
 * between the flash and SFC_MEM_ADDR (a physical address) and only
 * signals END, no RREQ/TREQ. `words` supplies register-read results
 * (RDID, RDSR...); NULL streams from the flash array at xfer_addr.
 */
static void ingenic_t31_sfc_dma_out(IngenicT31SfcState *s, const uint32_t *words,
                                    uint32_t len)
{
    g_autofree uint8_t *buf = g_malloc(len ? len : 1);

    if (words) {
        uint32_t i;
        for (i = 0; i < len; i++) {
            buf[i] = (words[i / 4] >> ((i % 4) * 8)) & 0xFF;
        }
    } else {
        memset(buf, 0xFF, len);
        if (s->xfer_addr < s->flash_size) {
            uint32_t n = s->flash_size - s->xfer_addr;
            memcpy(buf, &s->flash_data[s->xfer_addr], n < len ? n : len);
        }
    }
    ingenic_t31_sfc_dma_copy(s, buf, len, true);
    s->sr = SR_END;
}

static void ingenic_t31_sfc_dma_program(IngenicT31SfcState *s, uint32_t len)
{
    if (s->write_enabled && len && s->xfer_addr < s->flash_size) {
        g_autofree uint8_t *buf = g_malloc(len);
        uint32_t n = s->flash_size - s->xfer_addr;
        uint32_t i;

        ingenic_t31_sfc_dma_copy(s, buf, len, false);
        if (n > len) {
            n = len;
        }
        for (i = 0; i < n; i++) {
            s->flash_data[s->xfer_addr + i] &= buf[i];
        }
        ingenic_t31_sfc_writeback(s, s->xfer_addr, n);
    }
    s->write_enabled = false;
    s->sr = SR_END;
}

/*
 * SPI NAND (nand-id). The 3.10 and 4.4 jz_sfc_nand drivers program up
 * to six phases (GLB phase count): a PAGE READ alone, then a status
 * poll, then READ FROM CACHE with its data; WREN, PROGRAM LOAD with the
 * data, PROGRAM EXECUTE. A command table chain (CDT_EN) runs the same
 * way, entry by entry. Every phase runs, in order; a PIO data phase
 * holds the rest until the guest has moved the data through SFC_DR. A
 * phase flagged POLLEN reads the status the driver then finds in
 * DEV_STA_RT (the P_FAIL and E_FAIL bits); the flash is never busy.
 */
#define CDT_DATAEN          (1U << 31)  /* SFC_CMD_IDX: the chain moves data */

static void nand_writeback(void *opaque, uint32_t offset, uint32_t len)
{
    ingenic_t31_sfc_writeback(opaque, offset, len);
}

/* The address register a CDT entry names: COL, ROW, STA0 or STA1 */
static uint32_t nand_cdt_addr(IngenicT31SfcState *s, uint32_t link)
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

static void nand_run(IngenicT31SfcState *s, unsigned idx)
{
    bool cdt = s->glb & GLB_CDT_EN;
    bool out = cdt ? s->cmd_idx & CDT_DIR : s->glb & GLB_TRAN_DIR;
    unsigned nphase = MIN(MAX(GLB_PHASE_NUM(s->glb), 1), 6);
    uint32_t len = MIN(s->tran_len, (uint32_t)sizeof(s->nand_buf));

    for (unsigned n = 0; n < INGENIC_T31_SFC_CDT_ENTRIES; n++) {
        uint32_t link = cdt ? s->cdt[idx * 4] : 0;
        uint32_t xfer = cdt ? s->cdt[idx * 4 + 1] : s->tran_conf[idx];
        uint32_t addr = cdt ? nand_cdt_addr(s, link) : s->dev_addr[idx];
        uint32_t width = (xfer >> 26) & 7;
        bool dummy = width || ((xfer >> 17) & 0x3f);
        bool data = (xfer & TRAN_DATEEN) && len &&
                    (!cdt || (s->cmd_idx & CDT_DATAEN));
        bool more;

        if (width < 4) {
            addr &= (1ULL << (8 * width)) - 1;
        }
        idx = cdt ? (idx + 1) % INGENIC_T31_SFC_CDT_ENTRIES : idx + 1;
        more = cdt ? link & CDT_LINK : idx < nphase;
        ingenic_spinand_command(&s->nand, xfer & 0xff, addr, dummy,
                                s->nand_buf, sizeof(s->nand_buf));
        if (xfer & TRAN_POLLEN) {
            s->dev_sta_rt = s->nand_buf[0];
        } else if (data && !(s->glb & GLB_OP_MODE)) {
            s->nand_resume = more ? idx : -1;
            s->flash_pos = 0;
            s->words_total = (s->tran_len + 3) / 4;
            if (out) {
                s->nand_pio_out = true;
                s->sr |= SR_TRAN_REQ;
            } else {
                ingenic_t31_sfc_fill_fifo(s);
                s->sr |= SR_RECE_REQ;
            }
            return;
        } else if (data) {
            ingenic_t31_sfc_dma_copy(s, s->nand_buf, len, !out);
            if (out) {
                ingenic_spinand_data(&s->nand, s->nand_buf, len);
            }
        }
        if (!more) {
            break;
        }
    }
    s->sr |= SR_END;
}

/* The guest moved a PIO data phase: apply it, run what follows */
static void nand_pio_done(IngenicT31SfcState *s)
{
    int next = s->nand_resume;

    if (s->nand_pio_out) {
        s->nand_pio_out = false;
        s->sr &= ~SR_TRAN_REQ;
        ingenic_spinand_data(&s->nand, s->nand_buf,
                             MIN(s->tran_len, (uint32_t)sizeof(s->nand_buf)));
    }
    s->nand_resume = -1;
    if (next >= 0) {
        nand_run(s, next);
    } else {
        s->sr |= SR_END;
    }
}

static void nand_dr_write(IngenicT31SfcState *s, uint32_t v)
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

static void ingenic_t31_sfc_exec_cmd(IngenicT31SfcState *s, uint32_t cmd);

/*
 * CDT mode (the T32/T33 ingenic_sfc_v2 driver): SFC_CMD_IDX names the
 * first of a chain of {link, xfer, sta_exp, sta_msk} entries, such as
 * WREN, PAGE PROGRAM, then an RDSR poll. Every entry addresses ROW_ADDR.
 */
static void ingenic_t31_sfc_run_cdt(IngenicT31SfcState *s, uint32_t first)
{
    uint32_t i = first;

    s->xfer_addr = s->row_addr;
    do {
        uint32_t link = s->cdt[i * 4];
        uint32_t xfer = s->cdt[i * 4 + 1];

        if (!(xfer & TRAN_POLLEN)) {
            ingenic_t31_sfc_exec_cmd(s, xfer & TRAN_CMD_MSK);
        }
        /*
         * A PIO data phase completes through SFC_DR, which raises END.
         * What the drivers chain after one (status polls, an OTP exit)
         * has nothing for the model to do.
         */
        if ((s->sr & (SR_TRAN_REQ | SR_RECE_REQ)) || !(link & CDT_LINK)) {
            break;
        }
        i = (i + 1) % INGENIC_T31_SFC_CDT_ENTRIES;
    } while (i != first);

    if (!(s->sr & (SR_TRAN_REQ | SR_RECE_REQ))) {
        s->sr |= SR_END;
    }
}

/*
 * Run one TRIG_START. U-Boot programs a single phase; the 4.4 kernel
 * driver chains up to six (GLB phase count): e.g. WREN, then PAGE
 * PROGRAM with the data phase, or WREN then SECTOR ERASE. Every phase
 * carries its own command and device address; a phase flagged POLLEN
 * polls the flash status register until it matches DEV_STA_EXP, which
 * this model satisfies immediately (the flash is never busy).
 */
static void ingenic_t31_sfc_do_transfer(IngenicT31SfcState *s)
{
    uint32_t cdt_index = s->cmd_idx & 0x3F;
    uint32_t cdt_xfer = s->cdt[cdt_index * 4 + 1];
    uint32_t nphase = GLB_PHASE_NUM(s->glb);
    uint32_t i;

    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->sr = 0;

    if (s->nand.id) {
        s->nand_pio_out = false;
        s->nand_resume = -1;
        nand_run(s, (s->glb & GLB_CDT_EN) ? cdt_index : 0);
        return;
    }
    if ((s->glb & GLB_CDT_EN) && cdt_xfer != 0) {
        ingenic_t31_sfc_run_cdt(s, cdt_index);
        return;
    }
    if (cdt_xfer != 0) {
        s->xfer_addr = s->row_addr;
        ingenic_t31_sfc_exec_cmd(s, cdt_xfer & TRAN_CMD_MSK);
        return;
    }

    if (nphase == 0) {
        nphase = 1;
    }
    for (i = 0; i < nphase; i++) {
        uint32_t conf = s->tran_conf[i];

        if (conf & TRAN_POLLEN) {
            continue;
        }
        s->xfer_addr = s->dev_addr[i];
        ingenic_t31_sfc_exec_cmd(s, conf & TRAN_CMD_MSK);
        if (conf & TRAN_DATEEN) {
            /* the data phase ends the transfer; later phases only poll */
            break;
        }
    }
    if (s->sr == 0) {
        s->sr = SR_END;
    }
}

static void ingenic_t31_sfc_exec_cmd(IngenicT31SfcState *s, uint32_t cmd)
{
    bool dma = (s->glb & GLB_OP_MODE) != 0;
    /* CDT mode leaves GLB_TRAN_DIR stale; the direction is in CMD_IDX */
    bool is_write = (s->glb & GLB_CDT_EN) ? (s->cmd_idx & CDT_DIR) != 0
                                          : (s->glb & GLB_TRAN_DIR) != 0;

    if (s->absent) {
        /* Nothing answers: reads see the data lines high */
        cmd = is_write || !s->tran_len ? SPI_CMD_WRITE_DISABLE : SPI_CMD_READ;
    }

    switch (cmd) {
    case SPI_CMD_READ:
    case SPI_CMD_FAST_READ:
    case SPI_CMD_READ_4B:
    case SPI_CMD_FAST_READ_4B:
    case SPI_CMD_DOR:
    case SPI_CMD_DOR_4B:
    case SPI_CMD_QOR:
    case SPI_CMD_QOR_4B:
    case SPI_CMD_DIOR:
    case SPI_CMD_DIOR_4B:
    case SPI_CMD_QIOR:
    case SPI_CMD_QIOR_4B:
        if (dma) {
            ingenic_t31_sfc_dma_out(s, NULL, s->tran_len);
            break;
        }
        s->words_total = (s->tran_len + 3) / 4;
        ingenic_t31_sfc_fill_fifo(s);
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_ID:
        s->fifo[0] = ingenic_sfc_nor_rdid(s->flash_size, s->jedec_id);
        s->fifo_len = 1;
        s->fifo_pos = 0;
        s->words_total = 1;
        if (dma) {
            ingenic_t31_sfc_dma_out(s, s->fifo, s->tran_len ? s->tran_len : 4);
            break;
        }
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_STATUS:
        s->fifo[0] = s->write_enabled ? 0x02 : 0x00;
        s->fifo_len = 1;
        s->fifo_pos = 0;
        s->words_total = 1;
        if (dma) {
            ingenic_t31_sfc_dma_out(s, s->fifo, s->tran_len ? s->tran_len : 4);
            break;
        }
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_SR2:
        s->fifo[0] = s->sr2;
        s->fifo_len = 1;
        s->fifo_pos = 0;
        s->words_total = 1;
        if (dma) {
            ingenic_t31_sfc_dma_out(s, s->fifo, s->tran_len ? s->tran_len : 4);
            break;
        }
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_SR3:
        s->fifo[0] = s->sr3;
        s->fifo_len = 1;
        s->fifo_pos = 0;
        s->words_total = 1;
        if (dma) {
            ingenic_t31_sfc_dma_out(s, s->fifo, s->tran_len ? s->tran_len : 4);
            break;
        }
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_READ_SFDP:
        /* No SFDP table: answer all-ones like an unsupported part */
        s->words_total = (s->tran_len + 3) / 4;
        for (uint32_t i = 0; i < s->words_total &&
             i < INGENIC_T31_SFC_FIFO_DEPTH; i++) {
            s->fifo[i] = 0xFFFFFFFF;
        }
        s->fifo_len = s->words_total < INGENIC_T31_SFC_FIFO_DEPTH ?
                      s->words_total : INGENIC_T31_SFC_FIFO_DEPTH;
        s->fifo_pos = 0;
        if (dma) {
            ingenic_t31_sfc_dma_out(s, s->fifo, s->tran_len);
            break;
        }
        s->sr = SR_RECE_REQ;
        break;

    case SPI_CMD_EN4B:
        s->sr3 |= SR3_ADS;
        s->sr = SR_END;
        break;

    case SPI_CMD_EX4B:
        s->sr3 &= ~SR3_ADS;
        s->sr = SR_END;
        break;

    case SPI_CMD_RESET_EN:
    case SPI_CMD_RESET:
        s->sr = SR_END;
        break;

    case SPI_CMD_WRITE_SR:
    case SPI_CMD_WRITE_SR2:
    case SPI_CMD_WRITE_SR3:
        /*
         * Data arrives through SFC_DR like a page program; remember
         * which register the burst targets. A burst with no data
         * (DATEEN clear) completes immediately.
         */
        if (is_write && s->tran_len && dma) {
            uint32_t vlen = s->tran_len > 4 ? 4 : s->tran_len;
            uint8_t vbuf[4] = { 0 };
            uint32_t v;

            ingenic_t31_sfc_dma_copy(s, vbuf, vlen, false);
            v = vbuf[0] | (vbuf[1] << 8) | (vbuf[2] << 16) | (vbuf[3] << 24);
            s->status_cmd = cmd;
            ingenic_t31_sfc_status_data(s, v);
        } else if (is_write && s->tran_len) {
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
    case SPI_CMD_PAGE_PROGRAM_4B:
    case SPI_CMD_QPP:
    case SPI_CMD_QPP_4B:
    case SPI_CMD_QPP_ALT:
        if (is_write && dma) {
            ingenic_t31_sfc_dma_program(s, s->tran_len);
        } else if (is_write) {
            s->words_total = (s->tran_len + 3) / 4;
            s->writing = true;
            s->sr = SR_TRAN_REQ;
        } else {
            s->sr = SR_END;
        }
        break;

    case SPI_CMD_ERASE_4K:
    case SPI_CMD_ERASE_4K_4B:
        if (s->write_enabled && s->xfer_addr < s->flash_size &&
            s->xfer_addr + 4096 <= s->flash_size) {
            memset(&s->flash_data[s->xfer_addr], 0xFF, 4096);
            ingenic_t31_sfc_writeback(s, s->xfer_addr, 4096);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_ERASE_32K:
    case SPI_CMD_ERASE_32K_4B:
        if (s->write_enabled && s->xfer_addr < s->flash_size &&
            s->xfer_addr + 32768 <= s->flash_size) {
            memset(&s->flash_data[s->xfer_addr], 0xFF, 32768);
            ingenic_t31_sfc_writeback(s, s->xfer_addr, 32768);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_ERASE_64K:
    case SPI_CMD_ERASE_64K_4B:
        if (s->write_enabled && s->xfer_addr < s->flash_size &&
            s->xfer_addr + 65536 <= s->flash_size) {
            memset(&s->flash_data[s->xfer_addr], 0xFF, 65536);
            ingenic_t31_sfc_writeback(s, s->xfer_addr, 65536);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    case SPI_CMD_ERASE_CHIP:
    case SPI_CMD_ERASE_CHIP_ALT:
        if (s->write_enabled) {
            memset(s->flash_data, 0xFF, s->flash_size);
            ingenic_t31_sfc_writeback(s, 0, s->flash_size);
        }
        s->write_enabled = false;
        s->sr = SR_END;
        break;

    default:
        s->sr = SR_END;
        break;
    }
}

static uint64_t ingenic_t31_sfc_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(opaque);

    switch (offset) {
    case SFC_GLB:
        return s->glb;
    case SFC_DEV_CONF:
        return s->dev_conf;
    case 0x0014 ... 0x0028: {
        uint32_t idx = (offset - SFC_TRAN_CONF0_BASE) / 4;
        return s->tran_conf[idx];
    }
    case SFC_TRAN_LEN:
        return s->tran_len;
    case SFC_DEV_ADDR0 ... SFC_DEV_ADDR0 + 0x14:
        return s->dev_addr[(offset - SFC_DEV_ADDR0) / 4];
    case SFC_DEV_ADDR_PLUS0 ... SFC_DEV_ADDR_PLUS0 + 0x14:
        return s->dev_addr_plus[(offset - SFC_DEV_ADDR_PLUS0) / 4];
    case SFC_SR:
        return ingenic_t31_sfc_sr(s);
    case SFC_INTC:
        return s->intc;
    case SFC_CGE:
        return s->cge;
    case SFC_DEV_STA_EXP:
        return s->dev_sta_exp;
    case SFC_DEV_STA_RT:
        return s->dev_sta_rt;
    case SFC_DEV_STA_MSK:
        return s->dev_sta_msk;
    case SFC_CMD_IDX:
        return s->cmd_idx;
    case SFC_COL_ADDR:
        return s->col_addr;
    case SFC_ROW_ADDR:
        return s->row_addr;
    case SFC_STA_ADDR0:
        return s->sta_addr[0];
    case SFC_STA_ADDR1:
        return s->sta_addr[1];
    case SFC_DES_ADDR:
        return s->des_addr;
    case SFC_GLB1:
        return s->glb1;
    case SFC_DEV1_STA_RT:
        return s->dev1_sta_rt;

    case SFC_DR:
        if (s->fifo_pos < s->fifo_len) {
            uint32_t val = s->fifo[s->fifo_pos++];
            s->flash_pos++;
            if (s->fifo_pos >= s->fifo_len) {
                s->sr &= ~SR_RECE_REQ;
                if (s->flash_pos >= s->words_total && s->nand.id) {
                    nand_pio_done(s);
                } else if (s->flash_pos >= s->words_total) {
                    s->sr |= SR_END;
                } else {
                    ingenic_t31_sfc_fill_fifo(s);
                    s->sr |= SR_RECE_REQ;
                }
                ingenic_t31_sfc_update_irq(s);
            }
            return val;
        }
        return 0xFFFFFFFF;

    default:
        if (offset >= SFC_TRAN_CONF1_BASE && offset < SFC_TRAN_CONF1_END) {
            uint32_t idx = (offset - SFC_TRAN_CONF1_BASE) / 4;
            if (idx < 6) {
                return s->tran_conf1[idx];
            }
        }
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_T31_SFC_CDT_ENTRIES * 4) {
                return s->cdt[idx];
            }
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: unimplemented read (offset 0x%04" HWADDR_PRIx ")\n",
                      __func__, offset);
        return 0;
    }
}

static void ingenic_t31_sfc_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(opaque);

    switch (offset) {
    case SFC_GLB:
        s->glb = (uint32_t)value;
        break;
    case SFC_DEV_CONF:
        s->dev_conf = (uint32_t)value;
        break;
    case 0x0014 ... 0x0028: {
        uint32_t idx = (offset - SFC_TRAN_CONF0_BASE) / 4;
        s->tran_conf[idx] = (uint32_t)value;
        break;
    }
    case SFC_TRAN_LEN:
        s->tran_len = (uint32_t)value;
        break;
    /* one device address (+ plus) register per transfer phase, 6 each */
    case SFC_DEV_ADDR0 ... SFC_DEV_ADDR0 + 0x14:
        s->dev_addr[(offset - SFC_DEV_ADDR0) / 4] = (uint32_t)value;
        break;
    case SFC_DEV_ADDR_PLUS0 ... SFC_DEV_ADDR_PLUS0 + 0x14:
        s->dev_addr_plus[(offset - SFC_DEV_ADDR_PLUS0) / 4] = (uint32_t)value;
        break;
    case SFC_MEM_ADDR:
        s->mem_addr = (uint32_t)value;
        break;
    case SFC_INTC:
        s->intc = (uint32_t)value;
        ingenic_t31_sfc_update_irq(s);
        break;
    case SFC_CGE:
        s->cge = (uint32_t)value;
        break;
    case SFC_DEV_STA_EXP:
        s->dev_sta_exp = (uint32_t)value;
        break;
    case SFC_DEV_STA_RT:
        s->dev_sta_rt = (uint32_t)value;
        break;
    case SFC_DEV_STA_MSK:
        s->dev_sta_msk = (uint32_t)value;
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
        s->sta_addr[0] = (uint32_t)value;
        break;
    case SFC_STA_ADDR1:
        s->sta_addr[1] = (uint32_t)value;
        break;
    case SFC_DES_ADDR:
        s->des_addr = (uint32_t)value;
        break;
    case SFC_GLB1:
        s->glb1 = (uint32_t)value;
        break;
    case SFC_DEV1_STA_RT:
        s->dev1_sta_rt = (uint32_t)value;
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
            ingenic_t31_sfc_do_transfer(s);
        }
        ingenic_t31_sfc_update_irq(s);
        break;

    case SFC_DR:
        if (s->nand.id) {
            nand_dr_write(s, (uint32_t)value);
            ingenic_t31_sfc_update_irq(s);
            break;
        }
        if (s->status_cmd) {
            ingenic_t31_sfc_status_data(s, (uint32_t)value);
            ingenic_t31_sfc_update_irq(s);
            break;
        }
        if (s->writing) {
            uint32_t faddr = s->xfer_addr + s->flash_pos * 4;
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
                /*
                 * End of a PAGE PROGRAM burst: flush the whole written
                 * range back to disk. Doing it once per burst avoids
                 * a blk_pwrite per 4-byte word.
                 */
                if (s->write_enabled || s->writing) {
                    uint32_t bytes = s->flash_pos * 4;
                    ingenic_t31_sfc_writeback(s, s->xfer_addr, bytes);
                }
                s->writing = false;
                s->write_enabled = false;
                s->sr &= ~SR_TRAN_REQ;
                s->sr |= SR_END;
            } else if ((s->flash_pos % sfc_get_threshold(s)) == 0) {
                s->sr |= SR_TRAN_REQ;
            }
            ingenic_t31_sfc_update_irq(s);
        }
        break;

    case SFC_SCR:
        s->sr &= ~(uint32_t)value;
        ingenic_t31_sfc_update_irq(s);
        break;

    default:
        if (offset >= SFC_TRAN_CONF1_BASE && offset < SFC_TRAN_CONF1_END) {
            uint32_t idx = (offset - SFC_TRAN_CONF1_BASE) / 4;
            if (idx < 6) {
                s->tran_conf1[idx] = (uint32_t)value;
            }
            break;
        }
        if (offset >= SFC_CDT_BASE && offset < SFC_CDT_END) {
            uint32_t idx = (offset - SFC_CDT_BASE) / 4;
            if (idx < INGENIC_T31_SFC_CDT_ENTRIES * 4) {
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

static const MemoryRegionOps ingenic_t31_sfc_ops = {
    .read = ingenic_t31_sfc_read,
    .write = ingenic_t31_sfc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t31_sfc_reset_hold(Object *obj, ResetType type)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(obj);

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
    memset(s->cdt, 0, sizeof(s->cdt));
    s->fifo_pos = 0;
    s->fifo_len = 0;
    s->flash_pos = 0;
    s->words_total = 0;
    s->write_enabled = false;
    s->writing = false;
    s->status_cmd = 0;
    s->sr2 = 0;
    s->sr3 = 0;
    /* A SoC reset leaves the flash chip powered: its state stays */
    s->nand_pio_out = false;
    s->nand_resume = -1;
}

static void ingenic_t31_sfc_realize(DeviceState *dev, Error **errp)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(dev);
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
        s->nand.id = 0;
    }
    if (s->nand.id && (!s->nand.blocks ||
                       s->nand.blocks > INGENIC_SPINAND_MAX_BLOCKS)) {
        error_setg(errp, "nand-blocks must be 1 to %d",
                   INGENIC_SPINAND_MAX_BLOCKS);
        return;
    }
    s->flash_size = s->absent ? 0 : INGENIC_T31_SFC_FLASH_SIZE;
    if (s->nand.id) {
        s->flash_size = ingenic_spinand_size(&s->nand);
    } else if (di) {
        s->flash_size = ingenic_sfc_nor_size(
            blk_getlength(blk_by_legacy_dinfo(di)));
    }
    s->flash_data = g_malloc0(s->flash_size);
    memset(s->flash_data, 0xFF, s->flash_size);
    if (s->nand.id) {
        ingenic_spinand_init(&s->nand, s->flash_data, nand_writeback, s);
    }

    if (di) {
        BlockBackend *blk = blk_by_legacy_dinfo(di);
        int64_t size = blk_getlength(blk);

        s->blk_len = MAX(size, 0);
        if (s->nand.id && size < s->flash_size) {
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
        /*
         * Request write permission so page-program and erase commands
         * can persist back to the backing image. If the image is
         * opened read-only (snapshot=on, --readonly, etc.) we silently
         * fall back to in-memory-only modifications.
         */
        if (blk_supports_write_perm(blk)) {
            uint64_t perm = BLK_PERM_CONSISTENT_READ | BLK_PERM_WRITE;
            if (blk_set_perm(blk, perm, BLK_PERM_ALL, NULL) == 0) {
                s->blk = blk;
            }
        }
    }
}

static void ingenic_t31_sfc_init(Object *obj)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_sfc_ops, s,
                          TYPE_INGENIC_T31_SFC, SFC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void ingenic_t31_sfc_finalize(Object *obj)
{
    IngenicT31SfcState *s = INGENIC_T31_SFC(obj);

    g_free(s->flash_data);
    ingenic_spinand_finalize(&s->nand);
}

static const Property ingenic_t31_sfc_props[] = {
    DEFINE_PROP_UINT8("unit", IngenicT31SfcState, unit, 0),
    DEFINE_PROP_UINT32("nand-id", IngenicT31SfcState, nand.id, 0),
    DEFINE_PROP_UINT32("nand-blocks", IngenicT31SfcState, nand.blocks, 1024),
    DEFINE_PROP_UINT32("jedec-id", IngenicT31SfcState, jedec_id, 0),
};

static void ingenic_t31_sfc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    device_class_set_props(dc, ingenic_t31_sfc_props);
    dc->realize = ingenic_t31_sfc_realize;
    rc->phases.hold = ingenic_t31_sfc_reset_hold;
}

static const TypeInfo ingenic_t31_sfc_type_info = {
    .name = TYPE_INGENIC_T31_SFC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31SfcState),
    .instance_init = ingenic_t31_sfc_init,
    .instance_finalize = ingenic_t31_sfc_finalize,
    .class_init = ingenic_t31_sfc_class_init,
};

static void ingenic_t31_sfc_register_types(void)
{
    type_register_static(&ingenic_t31_sfc_type_info);
}

type_init(ingenic_t31_sfc_register_types)
