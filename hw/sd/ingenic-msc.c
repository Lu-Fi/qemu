/*
 * Ingenic T31 MSC (MMC/SD) Controller emulation
 *
 * The T31 MSC is a custom Synopsys-derived MMC/SD host controller. The
 * register layout matches the Ingenic JZ family. Driver-visible quirks:
 *
 *  - Response is read 16 bits at a time from MSC_RES. The driver uses a
 *    pattern that consumes 3 reads for short (R1/R3/R6/R7) responses and
 *    9 reads for long (R2). Format described in the read handler below.
 *  - Data is moved through MSC_RXFIFO / MSC_TXFIFO 32 bits at a time.
 *  - Completion signaled via MSC_IFLG (END_CMD_RES, DATA_TRAN_DONE,
 *    PRG_DONE) and MSC_STAT (DATA_FIFO_EMPTY, DATA_TRAN_DONE, PRG_DONE).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/qdev-properties.h"
#include "hw/sd/ingenic-msc.h"
#include "migration/vmstate.h"
#include "hw/core/irq.h"
#include "system/physmem.h"

/* Registers */
#define MSC_CTRL        0x000
#define MSC_STAT        0x004
#define MSC_CLKRT       0x008
#define MSC_CMDAT       0x00C
#define MSC_RESTO       0x010
#define MSC_RDTO        0x014
#define MSC_BLKLEN      0x018
#define MSC_NOB         0x01C
#define MSC_SNOB        0x020
#define MSC_IMASK       0x024
#define MSC_IFLG        0x028
#define MSC_CMD         0x02C
#define MSC_ARG         0x030
#define MSC_RES         0x034
#define MSC_RXFIFO      0x038
#define MSC_TXFIFO      0x03C
#define MSC_LPM         0x040
#define MSC_DMAC        0x044
#define MSC_DMANDA      0x048
#define MSC_DMADA       0x04C
#define MSC_DMALEN      0x050
#define MSC_DMACMD      0x054
#define MSC_CTRL2       0x058
#define MSC_RTCNT       0x05C

/* DMAC / descriptor bits (jzmmc v12 descriptor DMA) */
#define DMAC_DMAEN              (1 << 0)
#define DMACMD_LINK             (1 << 0)
#define DMACMD_ENDI             (1 << 1)

/* IFLG bits the interrupt-driven kernel driver completes on */
#define IFLG_DMA_DATA_DONE      (1u << 31)
#define IFLG_WR_ALL_DONE        (1 << 23)
#define IFLG_DMAEND             (1 << 16)
#define IFLG_AUTO_CMD12_DONE    (1 << 15)
#define STAT_AUTO_CMD12_DONE    (1u << 31)
#define CMDAT_AUTO_CMD12        (1 << 16)

/* CTRL bits */
#define CTRL_RESET              (1 << 3)
#define CTRL_START_OP           (1 << 2)

/* STAT bits */
#define STAT_PRG_DONE           (1 << 13)
#define STAT_DATA_TRAN_DONE     (1 << 12)
#define STAT_END_CMD_RES        (1 << 11)
#define STAT_DATA_FIFO_FULL     (1 << 7)
#define STAT_DATA_FIFO_EMPTY    (1 << 6)
#define STAT_TIME_OUT_RES       (1 << 1)
#define STAT_TIME_OUT_READ      (1 << 0)

/* CMDAT bits */
#define CMDAT_BUSY              (1 << 6)
#define CMDAT_WRITE             (1 << 4)
#define CMDAT_DATA_EN           (1 << 3)
#define CMDAT_RESPONSE_MASK     0x7
#define CMDAT_RESPONSE_NONE     0x0
#define CMDAT_RESPONSE_R2       0x2
#define CMDAT_RESPONSE_R3       0x3

/* IFLG bits */
#define IFLG_TIME_OUT_RES       (1 << 9)
#define IFLG_SDIO               (1 << 7)
#define IFLG_TIME_OUT_READ      (1 << 8)
#define IFLG_TXFIFO_WR_REQ      (1 << 6)
#define IFLG_RXFIFO_RD_REQ      (1 << 5)
#define IFLG_END_CMD_RES        (1 << 2)
#define IFLG_PRG_DONE           (1 << 1)
#define IFLG_DATA_TRAN_DONE     (1 << 0)

/*
 * Standard command path: U-Boot/Linux write CMD, ARG, CMDAT as separate
 * 32-bit registers and trigger via CTRL START_OP.
 */
/*
 * Interrupt line: IFLG bits not masked in IMASK (1 = masked). The
 * Ingenic kernel driver is fully interrupt driven; the bootrom polls.
 */
/* IFLG.SDIO follows the card's DAT1, so a write of 1 cannot clear it */
static uint32_t ingenic_msc_iflg(IngenicMscState *s)
{
    return s->reg_iflg | (s->sdio_irq ? IFLG_SDIO : 0);
}

static void ingenic_msc_update_irq(IngenicMscState *s)
{
    qemu_set_irq(s->irq, (ingenic_msc_iflg(s) & ~s->reg_imask) != 0);
}

/*
 * Descriptor DMA (DMAC/DMANDA, jzmmc v12): the kernel driver builds a
 * chain of {next, addr, len, cmd} descriptors in RAM, points DMANDA at
 * the first and sets DMAC.DMAEN. Data commands prefetch reads into
 * data_buf at command time; beyond that buffer the bytes are streamed
 * from the card as the descriptors consume them. Completion raises the
 * flags the driver waits for (DMA_DATA_DONE for reads, WR_ALL_DONE for
 * writes) and, when CMDAT.AUTO_CMD12 is set, stops the multi-block
 * transfer and reports AUTO_CMD12_DONE.
 */
static void ingenic_msc_dma_run(IngenicMscState *s)
{
    uint32_t nda = s->reg_dmanda;
    uint32_t remaining = s->reg_nob * s->reg_blklen;
    int guard = 4096;

    if (s->data_pos < remaining) {
        remaining -= s->data_pos;
    } else {
        remaining = 0;
    }

    while (remaining && guard--) {
        uint32_t desc[4];
        uint32_t da, len, dcmd, n, i;

        physical_memory_read(nda, desc, sizeof(desc));
        da = le32_to_cpu(desc[1]);
        len = le32_to_cpu(desc[2]);
        dcmd = le32_to_cpu(desc[3]);
        n = len < remaining ? len : remaining;

        if (n) {
            g_autofree uint8_t *buf = g_malloc(n);
            if (s->data_is_write) {
                physical_memory_read(da, buf, n);
                for (i = 0; i < n; i++) {
                    sdbus_write_byte(&s->sdbus, buf[i]);
                    if (s->data_pos + i < s->data_buffered) {
                        s->data_buf[s->data_pos + i] = buf[i];
                    }
                }
            } else {
                for (i = 0; i < n; i++) {
                    if (s->data_pos + i < s->data_buffered) {
                        buf[i] = s->data_buf[s->data_pos + i];
                    } else {
                        buf[i] = sdbus_read_byte(&s->sdbus);
                    }
                }
                physical_memory_write(da, buf, n);
            }
        }
        s->data_pos += n;
        remaining -= n;
        s->reg_dmada = da + n;
        s->reg_dmalen = len - n;
        s->reg_dmacmd = dcmd;
        if ((dcmd & DMACMD_ENDI) || !(dcmd & DMACMD_LINK)) {
            break;
        }
        nda = le32_to_cpu(desc[0]);
        s->reg_dmanda = nda;
    }

    s->data_pending = false;
    /*
     * The chain is done and DMAEN drops: the 3.10 jzmmc driver moves
     * anything under 64 bytes by PIO without touching DMAC, which must not
     * rerun the last descriptors.
     */
    s->reg_dmac &= ~DMAC_DMAEN;
    s->reg_iflg |= IFLG_DATA_TRAN_DONE | IFLG_DMAEND;
    if (s->data_is_write) {
        s->reg_iflg |= IFLG_PRG_DONE | IFLG_WR_ALL_DONE;
    } else {
        s->reg_iflg |= IFLG_DMA_DATA_DONE;
    }
    if (s->reg_cmdat & CMDAT_AUTO_CMD12) {
        SDRequest stop = { .cmd = 12, .arg = 0 };
        uint8_t resp[16];
        sdbus_do_command(&s->sdbus, &stop, resp, sizeof(resp));
        s->reg_iflg |= IFLG_AUTO_CMD12_DONE;
        s->stat_extra |= STAT_AUTO_CMD12_DONE;
    }
    ingenic_msc_update_irq(s);
}

static void ingenic_msc_run_command(IngenicMscState *s)
{
    SDRequest req = {
        .cmd = s->reg_cmd & 0x3f,
        .arg = s->reg_arg,
    };
    uint8_t resp[16];
    size_t rsplen;
    uint32_t cmdat = s->reg_cmdat;
    uint32_t resp_type = cmdat & CMDAT_RESPONSE_MASK;

    s->resp_idx = 0;
    s->resp_size = 0;
    s->stat_extra = 0;

    rsplen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));

    if (resp_type != CMDAT_RESPONSE_NONE && rsplen == 0) {
        s->reg_iflg |= IFLG_TIME_OUT_RES | IFLG_END_CMD_RES;
        s->stat_extra |= STAT_TIME_OUT_RES;
        ingenic_msc_update_irq(s);
        return;
    }

    if (resp_type == CMDAT_RESPONSE_R2) {
        s->resp_size = 16;
        if (rsplen >= 16) {
            memcpy(s->resp_buf, resp, 16);
        } else {
            memset(s->resp_buf, 0, 16);
        }
    } else if (resp_type != CMDAT_RESPONSE_NONE) {
        s->resp_size = 4;
        if (rsplen >= 4) {
            memcpy(s->resp_buf, resp, 4);
        } else {
            memset(s->resp_buf, 0, 4);
        }
    }

    s->resp_idxbyte = (resp_type == CMDAT_RESPONSE_R2 ||
                       resp_type == CMDAT_RESPONSE_R3)
                      ? 0x3f : (uint8_t)(req.cmd & 0x3f);

    s->reg_iflg |= IFLG_END_CMD_RES;

    if (cmdat & CMDAT_DATA_EN) {
        uint32_t total = s->reg_nob * s->reg_blklen;

        s->data_total = total;
        s->data_buffered = MIN(total, sizeof(s->data_buf));
        s->data_pos = 0;
        s->data_is_write = !!(cmdat & CMDAT_WRITE);
        s->reg_iflg &= ~(IFLG_DATA_TRAN_DONE | IFLG_PRG_DONE);

        if (!s->data_is_write) {
            for (uint32_t i = 0; i < s->data_buffered; i++) {
                s->data_buf[i] = sdbus_read_byte(&s->sdbus);
            }
        }
        s->data_pending = true;
        if (s->reg_dmac & DMAC_DMAEN) {
            ingenic_msc_dma_run(s);
        }
    } else {
        s->data_total = 0;
        s->data_buffered = 0;
        s->data_pos = 0;
        s->data_pending = false;
    }
    ingenic_msc_update_irq(s);
}

static void ingenic_msc_reset_state(IngenicMscState *s)
{
    s->reg_ctrl = 0;
    s->reg_clkrt = 0;
    s->reg_cmdat = 0;
    s->reg_resto = 0xff;
    s->reg_rdto = 0xffff;
    s->reg_blklen = 0;
    s->reg_nob = 0;
    s->reg_imask = 0;
    s->reg_iflg = 0;
    s->reg_cmd = 0;
    s->reg_arg = 0;
    s->reg_lpm = 0;
    s->resp_size = 0;
    s->resp_idx = 0;
    s->data_total = 0;
    s->data_buffered = 0;
    s->data_pos = 0;
    s->data_is_write = false;
    s->reg_dmac = 0;
    s->reg_dmanda = 0;
    s->reg_dmada = 0;
    s->reg_dmalen = 0;
    s->reg_dmacmd = 0;
    s->reg_ctrl2 = 0;
    s->stat_extra = 0;
    s->data_pending = false;
    memset(s->resp_buf, 0, sizeof(s->resp_buf));
    if (s->irq) {
        ingenic_msc_update_irq(s);
    }
}

static uint32_t ingenic_msc_status(IngenicMscState *s)
{
    uint32_t stat = 0;

    if (s->data_total == 0 || s->data_pos >= s->data_total) {
        stat |= STAT_DATA_FIFO_EMPTY;
    }
    /*
     * DATA_TRAN_DONE / PRG_DONE are sticky completion flags: once a transfer
     * finishes they stay asserted (mirroring the IFLG latch) until the driver
     * acks via MSC_IFLG - including across the CMD12 STOP that follows a
     * multi-block read. Deriving them live from data_total would drop them at
     * CMD12 (which resets data_total), hanging the bootrom's done-wait poll.
     */
    if (s->reg_iflg & IFLG_DATA_TRAN_DONE) {
        stat |= STAT_DATA_TRAN_DONE;
    }
    if (s->reg_iflg & IFLG_PRG_DONE) {
        stat |= STAT_PRG_DONE;
    }
    if (s->resp_size) {
        stat |= STAT_END_CMD_RES;
    }
    stat |= s->stat_extra;
    return stat;
}

static uint16_t ingenic_msc_resp_word(IngenicMscState *s, uint8_t idx)
{
    /* Map the 16-bit read window onto the response bytes such that the
     * U-Boot driver's reconstruction yields the expected big-endian
     * response. The pattern differs slightly between short and long
     * responses (driver implementation quirk):
     *
     *   short (R1/R3/...):  resp = (read[0]<<24) | (read[1]<<8) | (read[2]&0xFF)
     *     read[0] = resp_byte[0]
     *     read[1] = (resp_byte[1] << 8) | resp_byte[2]
     *     read[2] = resp_byte[3]
     *
     *   long (R2):  resp[i] = (a<<24) | (b<<8) | (c>>8); a := c (9 reads)
     *     read[0] = resp_byte[0]
     *     read[k] for k=1..7 = (resp_byte[2k-1] << 8) | resp_byte[2k]
     *     read[8] = resp_byte[15] << 8
     */
    if (s->resp_size == 4) {
        switch (idx) {
        /* high byte of the first word is the response index field (resp[5]) */
        case 0: return ((uint16_t)s->resp_idxbyte << 8) | s->resp_buf[0];
        case 1: return ((uint16_t)s->resp_buf[1] << 8) | s->resp_buf[2];
        case 2: return s->resp_buf[3];
        default: return 0;
        }
    } else if (s->resp_size == 16) {
        if (idx == 0) {
            return s->resp_buf[0];
        }
        if (idx >= 1 && idx <= 7) {
            return ((uint16_t)s->resp_buf[2 * idx - 1] << 8) |
                                s->resp_buf[2 * idx];
        }
        if (idx == 8) {
            return (uint16_t)s->resp_buf[15] << 8;
        }
    }
    return 0;
}

static uint64_t ingenic_msc_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicMscState *s = INGENIC_MSC(opaque);
    uint32_t val = 0;
    hwaddr orig_offset = offset;
    unsigned orig_size = size;

    /* Align sub-register reads to the parent 32-bit register. */
    if (size < 4 && (offset & 3)) {
        offset = offset & ~3;
        size = 4;
    }

    switch (offset) {
    case MSC_CTRL:
        val = s->reg_ctrl;
        break;
    case MSC_STAT:
        val = ingenic_msc_status(s);
        break;
    case MSC_CLKRT:
        val = s->reg_clkrt;
        break;
    case MSC_CMDAT:
        val = s->reg_cmdat;
        break;
    case MSC_RESTO:
        val = s->reg_resto;
        break;
    case MSC_RDTO:
        val = s->reg_rdto;
        break;
    case MSC_BLKLEN:
        val = s->reg_blklen;
        break;
    case MSC_NOB:
        val = s->reg_nob;
        break;
    case MSC_SNOB:
        val = s->reg_nob;
        break;
    case MSC_IMASK:
        val = s->reg_imask;
        break;
    case MSC_IFLG:
        val = ingenic_msc_iflg(s);
        /* Driver polls these flags during data transfer; report them
         * dynamically based on current buffer state. */
        if (s->data_total && s->data_pos < s->data_total) {
            if (s->data_is_write) {
                val |= IFLG_TXFIFO_WR_REQ;
            } else {
                val |= IFLG_RXFIFO_RD_REQ;
            }
        }
        break;
    case MSC_CMD:
        val = s->reg_cmd;
        break;
    case MSC_ARG:
        val = s->reg_arg;
        break;
    case MSC_RES:
        val = ingenic_msc_resp_word(s, s->resp_idx);
        if (s->resp_idx < 0xff) {
            s->resp_idx++;
        }
        break;
    case MSC_RXFIFO:
        if (s->data_pos < s->data_total) {
            /* an SDIO byte transfer can end part way into a word */
            unsigned n = MIN(4, s->data_total - s->data_pos);

            for (unsigned i = 0; i < n; i++) {
                uint32_t at = s->data_pos + i;
                uint8_t b = at < s->data_buffered ? s->data_buf[at]
                                                  : sdbus_read_byte(&s->sdbus);

                val |= (uint32_t)b << (8 * i);
            }
            s->data_pos += n;
            if (s->data_pos >= s->data_total) {
                s->reg_iflg |= IFLG_DATA_TRAN_DONE | IFLG_PRG_DONE;
            }
        }
        break;
    case MSC_LPM:
        val = s->reg_lpm;
        break;
    case MSC_DMAC:
        val = s->reg_dmac;
        break;
    case MSC_DMANDA:
        val = s->reg_dmanda;
        break;
    case MSC_DMADA:
        val = s->reg_dmada;
        break;
    case MSC_DMALEN:
        val = s->reg_dmalen;
        break;
    case MSC_DMACMD:
        val = s->reg_dmacmd;
        break;
    case MSC_CTRL2:
        val = s->reg_ctrl2;
        break;
    case MSC_RTCNT:
        val = 0;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "ingenic-msc: read unimpl offset 0x%03"HWADDR_PRIx"\n",
                      offset);
        break;
    }

    /* Extract the requested sub-register bytes */
    if (orig_size < 4) {
        unsigned shift = (orig_offset & 3) * 8;
        val = (val >> shift) & ((1u << (orig_size * 8)) - 1);
    }
    return val;
}

static void ingenic_msc_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicMscState *s = INGENIC_MSC(opaque);
    uint32_t val = (uint32_t)value;

    /*
     * A byte or halfword write updates its part of the 32-bit register:
     * merge it into the register's current value.
     */
    if (size < 4) {
        hwaddr reg_off = offset & ~3;
        unsigned shift = (offset & 3) * 8;
        uint32_t mask = ((1u << (size * 8)) - 1) << shift;
        uint32_t cur = 0;

        switch (reg_off) {
        case MSC_CTRL:   cur = s->reg_ctrl;   break;
        case MSC_STAT:   cur = (s->reg_nob << 16) | s->reg_blklen; break;
        case MSC_CLKRT:  cur = s->reg_clkrt;  break;
        case MSC_CMDAT:  cur = s->reg_cmdat;  break;
        case MSC_RESTO:  cur = s->reg_resto;  break;
        case MSC_RDTO:   cur = s->reg_rdto;   break;
        case MSC_BLKLEN: cur = s->reg_blklen; break;
        case MSC_NOB:    cur = s->reg_nob;    break;
        case MSC_SNOB:   cur = s->reg_nob;    break;
        case MSC_IMASK:  cur = s->reg_imask;  break;
        case MSC_IFLG:   cur = s->reg_iflg;   break;
        case MSC_CMD:    cur = s->reg_cmd;    break;
        case MSC_ARG:    cur = s->reg_arg;    break;
        default: break;
        }
        val = (cur & ~mask) | ((val << shift) & mask);
        offset = reg_off;
    }

    switch (offset) {
    case MSC_CTRL:
        s->reg_ctrl = val;
        if (val & CTRL_RESET) {
            ingenic_msc_reset_state(s);
            return;
        }
        if (val & CTRL_START_OP) {
            ingenic_msc_run_command(s);
        }
        break;
    case MSC_STAT:
        /* Read-only */
        break;
    case MSC_CLKRT:
        s->reg_clkrt = val;
        break;
    case MSC_CMDAT:
        s->reg_cmdat = val;
        break;
    case MSC_RESTO:
        s->reg_resto = val;
        break;
    case MSC_RDTO:
        s->reg_rdto = val;
        break;
    case MSC_BLKLEN:
        s->reg_blklen = val;
        break;
    case MSC_NOB:
        s->reg_nob = val;
        break;
    case MSC_IMASK:
        s->reg_imask = val;
        ingenic_msc_update_irq(s);
        break;
    case MSC_IFLG:
        /* Write 1 to clear */
        s->reg_iflg &= ~val;
        ingenic_msc_update_irq(s);
        break;
    case MSC_CMD:
        s->reg_cmd = val;
        break;
    case MSC_ARG:
        s->reg_arg = val;
        break;
    case MSC_TXFIFO:
        if (s->data_is_write && s->data_pos < s->data_total) {
            unsigned n = MIN(4, s->data_total - s->data_pos);

            for (unsigned i = 0; i < n; i++) {
                uint8_t b = val >> (8 * i);

                sdbus_write_byte(&s->sdbus, b);
                if (s->data_pos + i < s->data_buffered) {
                    s->data_buf[s->data_pos + i] = b;
                }
            }
            s->data_pos += n;
            if (s->data_pos >= s->data_total) {
                s->reg_iflg |= IFLG_PRG_DONE | IFLG_DATA_TRAN_DONE;
            }
        }
        break;
    case MSC_LPM:
        s->reg_lpm = val;
        break;
    case MSC_DMAC:
        s->reg_dmac = val;
        if ((val & DMAC_DMAEN) && s->data_pending) {
            ingenic_msc_dma_run(s);
        }
        break;
    case MSC_DMANDA:
        s->reg_dmanda = val;
        break;
    case MSC_DMADA:
        s->reg_dmada = val;
        break;
    case MSC_DMALEN:
        s->reg_dmalen = val;
        break;
    case MSC_DMACMD:
        s->reg_dmacmd = val;
        break;
    case MSC_CTRL2:
        s->reg_ctrl2 = val;
        break;
    case MSC_RTCNT:
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "ingenic-msc: write unimpl 0x%03"HWADDR_PRIx
                      " = 0x%08x\n", offset, val);
        break;
    }
}

static const MemoryRegionOps ingenic_msc_ops = {
    .read = ingenic_msc_read,
    .write = ingenic_msc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 1, .max_access_size = 4 },
};

static void ingenic_msc_reset_hold(Object *obj, ResetType type)
{
    ingenic_msc_reset_state(INGENIC_MSC(obj));
}

static void ingenic_msc_set_sdio_irq(DeviceState *dev, bool level)
{
    IngenicMscState *s = INGENIC_MSC(dev);

    s->sdio_irq = level;
    ingenic_msc_update_irq(s);
}

/*
 * The bus takes the controller's name, msc0 or msc1, so that a card can be
 * put on either from the command line: -device atbm6031,bus=msc1
 */
static void ingenic_msc_realize(DeviceState *dev, Error **errp)
{
    IngenicMscState *s = INGENIC_MSC(dev);
    const char *name = object_get_canonical_path_component(OBJECT(dev));

    qbus_init(&s->sdbus, sizeof(s->sdbus), TYPE_INGENIC_MSC_BUS, dev,
              s->bus_name ?: name ?: "sd-bus");
}

static const Property ingenic_msc_props[] = {
    DEFINE_PROP_STRING("bus-name", IngenicMscState, bus_name),
};

static void ingenic_msc_init(Object *obj)
{
    IngenicMscState *s = INGENIC_MSC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_msc_ops, s,
                          TYPE_INGENIC_MSC, INGENIC_MSC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static const VMStateDescription vmstate_ingenic_msc = {
    .name = TYPE_INGENIC_MSC,
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(reg_ctrl, IngenicMscState),
        VMSTATE_UINT32(reg_clkrt, IngenicMscState),
        VMSTATE_UINT32(reg_cmdat, IngenicMscState),
        VMSTATE_UINT32(reg_resto, IngenicMscState),
        VMSTATE_UINT32(reg_rdto, IngenicMscState),
        VMSTATE_UINT32(reg_blklen, IngenicMscState),
        VMSTATE_UINT32(reg_nob, IngenicMscState),
        VMSTATE_UINT32(reg_imask, IngenicMscState),
        VMSTATE_UINT32(reg_iflg, IngenicMscState),
        VMSTATE_UINT32(reg_cmd, IngenicMscState),
        VMSTATE_UINT32(reg_arg, IngenicMscState),
        VMSTATE_UINT32(reg_lpm, IngenicMscState),
        VMSTATE_UINT8_ARRAY(resp_buf, IngenicMscState, 16),
        VMSTATE_UINT8(resp_size, IngenicMscState),
        VMSTATE_UINT8(resp_idx, IngenicMscState),
        VMSTATE_UINT32(data_total, IngenicMscState),
        VMSTATE_UINT32(data_buffered, IngenicMscState),
        VMSTATE_UINT32(data_pos, IngenicMscState),
        VMSTATE_BOOL(data_is_write, IngenicMscState),
        VMSTATE_BOOL_V(sdio_irq, IngenicMscState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_msc_bus_class_init(ObjectClass *oc, const void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(oc);

    sbc->set_sdio_irq = ingenic_msc_set_sdio_irq;
}

static void ingenic_msc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_msc_realize;
    dc->vmsd = &vmstate_ingenic_msc;
    device_class_set_props(dc, ingenic_msc_props);
    rc->phases.hold = ingenic_msc_reset_hold;
}

static const TypeInfo ingenic_msc_type_info = {
    .name = TYPE_INGENIC_MSC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicMscState),
    .instance_init = ingenic_msc_init,
    .class_init = ingenic_msc_class_init,
};

static const TypeInfo ingenic_msc_bus_info = {
    .name = TYPE_INGENIC_MSC_BUS,
    .parent = TYPE_SD_BUS,
    .instance_size = sizeof(SDBus),
    .class_init = ingenic_msc_bus_class_init,
};

static void ingenic_msc_register_types(void)
{
    type_register_static(&ingenic_msc_type_info);
    type_register_static(&ingenic_msc_bus_info);
}

type_init(ingenic_msc_register_types)
