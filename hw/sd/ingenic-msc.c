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
#include "hw/sd/ingenic-msc.h"
#include "migration/vmstate.h"

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

    rsplen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));

    fprintf(stderr, "MSC: CMD%d arg=0x%08x cmdat=0x%04x rsplen=%zu\n",
            req.cmd, req.arg, cmdat, rsplen);

    /* Suppress CARD_POWER_UP on the first ACMD41 (same fix as the ROM
     * command path -- QEMU's SD model powers up instantly, but the
     * bootrom sends a mandatory second CMD55+ACMD41). */
    if (req.cmd == 41 && rsplen >= 4 && (resp[0] & 0x80)) {
        s->acmd41_count++;
        if (s->acmd41_count == 1) {
            resp[0] &= ~0x80;
            SDRequest rst = { .cmd = 0, .arg = 0 };
            sdbus_do_command(&s->sdbus, &rst, NULL, 0);
        }
    }

    if (resp_type != CMDAT_RESPONSE_NONE && rsplen == 0) {
        fprintf(stderr, "MSC: CMD%d TIMEOUT\n", req.cmd);
        s->reg_iflg |= IFLG_TIME_OUT_RES | IFLG_END_CMD_RES;
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

        if (total > sizeof(s->data_buf)) {
            total = sizeof(s->data_buf);
        }
        s->data_total = total;
        s->data_pos = 0;
        s->data_is_write = !!(cmdat & CMDAT_WRITE);
        s->reg_iflg &= ~(IFLG_DATA_TRAN_DONE | IFLG_PRG_DONE);

        if (!s->data_is_write) {
            for (uint32_t i = 0; i < total; i++) {
                s->data_buf[i] = sdbus_read_byte(&s->sdbus);
            }
            fprintf(stderr, "MSC: data read %u bytes (blklen=%u nob=%u) first4=%02x%02x%02x%02x\n",
                    total, s->reg_blklen, s->reg_nob,
                    total > 0 ? s->data_buf[0] : 0,
                    total > 1 ? s->data_buf[1] : 0,
                    total > 2 ? s->data_buf[2] : 0,
                    total > 3 ? s->data_buf[3] : 0);
        }
    } else {
        s->data_total = 0;
        s->data_pos = 0;
    }
}

/*
 * QEMU's sd.c powers up instantly on non-enquiry ACMD41, which races the
 * T41 ROM: the ROM always sends a second CMD55+ACMD41 after the initial
 * probe, but by then the card has moved to sd_ready_state and rejects
 * CMD55.  Suppress CARD_POWER_UP on the first ACMD41 so the polling
 * loop works.
 */
#define ACMD41_POWER_UP_BIT  (1u << 31)

/*
 * Bootrom command path.  The T41 ROM packs everything into a halfword
 * written to the upper half of MSC_CMDAT (offset 0x00E):
 *   bits [13:8] = command index
 *   bits  [7:6] = flags
 *   bit     [5] = keep_clock
 *   bits  [1:0] = response (0=none, 1=R3, 2=R1/R7, 3=R2)
 *
 * The argument was previously written to MSC_CLKRT (offset 0x008).
 * Completion/error status goes into MSC_ARG (0x030):
 *   low halfword bit 0 = clear when done
 *   high halfword low nibble = 0 on success
 * Response data goes to MSC_RESTO (0x010).
 */
static void ingenic_msc_rom_command(IngenicMscState *s, uint16_t packed)
{
    uint8_t cmd_idx = (packed >> 8) & 0x3f;
    uint8_t rom_resp = packed & 0x3;
    uint32_t cmdat_resp;

    switch (rom_resp) {
    case 0: cmdat_resp = CMDAT_RESPONSE_NONE; break;
    case 1: cmdat_resp = CMDAT_RESPONSE_R3;   break;
    case 3: cmdat_resp = CMDAT_RESPONSE_R2;   break;
    default: cmdat_resp = 1;                   break;
    }

    SDRequest req = {
        .cmd = cmd_idx,
        .arg = s->reg_clkrt,
    };
    uint8_t resp[16];
    size_t rsplen;

    s->resp_idx = 0;
    s->resp_size = 0;

    rsplen = sdbus_do_command(&s->sdbus, &req, resp, sizeof(resp));

    fprintf(stderr, "MSC: ROM CMD%d arg=0x%08x resp_type=%d rsplen=%zu\n",
            cmd_idx, req.arg, rom_resp, rsplen);

    if (cmdat_resp != CMDAT_RESPONSE_NONE && rsplen == 0) {
        fprintf(stderr, "MSC: ROM CMD%d TIMEOUT\n", cmd_idx);
        s->reg_arg = 0xffff0001;
        s->reg_iflg |= IFLG_TIME_OUT_RES | IFLG_END_CMD_RES;
        return;
    }

    /* ARG low bit 0 = 1 (cmd done), bit 1 = 1 (transfer done).
     * ARG high low nibble = 0 (no error), upper bits preserved. */
    s->reg_arg = 0xfff00003;

    /* Suppress CARD_POWER_UP on the first ACMD41 so the ROM enters its
     * polling loop before the SD card transitions to ready state.
     * Also reset the card back to idle so subsequent CMD55 is accepted. */
    if (cmd_idx == 41 && rsplen >= 4 && (resp[0] & 0x80)) {
        s->acmd41_count++;
        if (s->acmd41_count == 1) {
            resp[0] &= ~0x80;
            SDRequest rst = { .cmd = 0, .arg = 0 };
            sdbus_do_command(&s->sdbus, &rst, NULL, 0);
        }
    }

    if (cmdat_resp == CMDAT_RESPONSE_R2) {
        s->resp_size = 16;
        if (rsplen >= 16) {
            memcpy(s->resp_buf, resp, 16);
        }
        s->reg_resto = (uint32_t)resp[0] << 24 | (uint32_t)resp[1] << 16 |
                       (uint32_t)resp[2] << 8 | resp[3];
    } else if (cmdat_resp != CMDAT_RESPONSE_NONE) {
        s->resp_size = 4;
        if (rsplen >= 4) {
            memcpy(s->resp_buf, resp, 4);
        }
        s->reg_resto = (uint32_t)resp[0] << 24 | (uint32_t)resp[1] << 16 |
                       (uint32_t)resp[2] << 8 | resp[3];
    }

    s->resp_idxbyte = (cmdat_resp == CMDAT_RESPONSE_R2 ||
                       cmdat_resp == CMDAT_RESPONSE_R3)
                      ? 0x3f : cmd_idx;

    s->reg_iflg |= IFLG_END_CMD_RES;

    /* CMD6 (SWITCH_FUNC) and CMD51 (SEND_SCR) produce data that the SD
     * card model expects the host to read.  The ROM doesn't read it
     * through the MSC data path, so drain it here to keep the card
     * state machine moving. */
    if (cmd_idx == 6 || cmd_idx == 51 || cmd_idx == 13) {
        int drain = (cmd_idx == 6) ? 64 : 8;
        for (int i = 0; i < drain; i++) {
            sdbus_read_byte(&s->sdbus);
        }
    }

    /* CMD17 (READ_SINGLE_BLOCK) and CMD18 (READ_MULTIPLE_BLOCK):
     * pre-read block data from the SD bus so the ROM can drain it
     * from MSC+0x020 (the ROM's RXFIFO address).  The ROM writes
     * block length to MSC+0x004 and block count to MSC+0x006 before
     * the command; use reg_blklen/reg_nob as fallback. */
    if (cmd_idx == 17 || cmd_idx == 18) {
        uint32_t blklen = s->reg_blklen ? s->reg_blklen : 512;
        uint32_t nob = (cmd_idx == 17) ? 1 : (s->reg_nob ? s->reg_nob : 1);
        uint32_t total = blklen * nob;
        if (total > sizeof(s->data_buf)) {
            total = sizeof(s->data_buf);
        }
        s->data_total = total;
        s->data_pos = 0;
        s->data_is_write = false;
        for (uint32_t i = 0; i < total; i++) {
            s->data_buf[i] = sdbus_read_byte(&s->sdbus);
        }
        /* CMD18 is multi-block: the card stays in sendingdata until
         * CMD12.  The ROM relies on auto-CMD12 from the MSC hardware. */
        if (cmd_idx == 18) {
            SDRequest stop = { .cmd = 12, .arg = 0 };
            uint8_t stopresp[4] = {};
            sdbus_do_command(&s->sdbus, &stop, stopresp, sizeof(stopresp));
        }
        s->reg_arg |= 0x0020;
        fprintf(stderr, "MSC: ROM data read %u bytes (blklen=%u nob=%u)\n",
                total, blklen, nob);
    }
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
    s->data_pos = 0;
    s->data_is_write = false;
    s->acmd41_count = 0;
    memset(s->resp_buf, 0, sizeof(s->resp_buf));
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
        /* The T41 ROM reads MSC+0x020 as RXFIFO (not standard SNOB).
         * Return data bytes when a transfer is active. */
        if (s->data_total && !s->data_is_write &&
            s->data_pos + 4 <= s->data_total) {
            val = (uint32_t)s->data_buf[s->data_pos]
                | ((uint32_t)s->data_buf[s->data_pos + 1] << 8)
                | ((uint32_t)s->data_buf[s->data_pos + 2] << 16)
                | ((uint32_t)s->data_buf[s->data_pos + 3] << 24);
            s->data_pos += 4;
            if (s->data_pos >= s->data_total) {
                s->reg_arg = (s->reg_arg & ~0x0020) | 0x0002;
            } else if ((s->data_pos % 512) == 0) {
                s->reg_arg |= 0x0020;
            }
        } else {
            val = s->reg_nob;
        }
        break;
    case MSC_IMASK:
        val = s->reg_imask;
        break;
    case MSC_IFLG:
        val = s->reg_iflg;
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
        /* T41 ROM protocol: re-assert data-ready (bit 5) on reads
         * when transfer data is available in the FIFO. */
        if (s->data_total && !s->data_is_write &&
            s->data_pos < s->data_total) {
            val |= 0x0020;
        }
        break;
    case MSC_RES:
        val = ingenic_msc_resp_word(s, s->resp_idx);
        if (s->resp_idx < 0xff) {
            s->resp_idx++;
        }
        break;
    case MSC_RXFIFO:
        if (s->data_pos + 4 <= s->data_total) {
            val = (uint32_t)s->data_buf[s->data_pos]
                | ((uint32_t)s->data_buf[s->data_pos + 1] << 8)
                | ((uint32_t)s->data_buf[s->data_pos + 2] << 16)
                | ((uint32_t)s->data_buf[s->data_pos + 3] << 24);
            s->data_pos += 4;
            if (s->data_pos >= s->data_total) {
                s->reg_iflg |= IFLG_DATA_TRAN_DONE | IFLG_PRG_DONE;
            }
        }
        break;
    case MSC_LPM:
        val = s->reg_lpm;
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
    if (orig_offset != MSC_RXFIFO) {
        fprintf(stderr, "MSC: R[%u] @0x%03x = 0x%0*x\n",
                orig_size, (unsigned)orig_offset,
                (int)(orig_size * 2), (unsigned)val);
    }
    return val;
}

static void ingenic_msc_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicMscState *s = INGENIC_MSC(opaque);
    fprintf(stderr, "MSC: W[%u] @0x%03x = 0x%0*x\n",
            size, (unsigned)offset, size*2, (unsigned)value);
    uint32_t val = (uint32_t)value;
    hwaddr orig_offset = offset;

    /* The T41 ROM writes individual bytes/halfwords within 32-bit registers.
     * Align sub-register accesses to the parent register by merging the
     * written bytes into the existing register value. */
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
        fprintf(stderr, "MSC: CTRL write 0x%08x\n", val);
        s->reg_ctrl = val;
        if (val & CTRL_RESET) {
            fprintf(stderr, "MSC: RESET\n");
            ingenic_msc_reset_state(s);
            return;
        }
        if (val & CTRL_START_OP) {
            ingenic_msc_run_command(s);
        }
        break;
    case MSC_STAT:
        /* The T41 ROM writes block length (halfword at 0x004) and block
         * count (halfword at 0x006) to the STAT register address as part
         * of data transfer setup.  Route these to blklen/nob. */
        s->reg_blklen = val & 0xffff;
        s->reg_nob = (val >> 16) & 0xffff;
        break;
    case MSC_CLKRT:
        s->reg_clkrt = val;
        break;
    case MSC_CMDAT:
        s->reg_cmdat = val;
        /* The T41 ROM writes a packed command halfword to offset 0x00E
         * (the upper half of CMDAT).  This is the command trigger. */
        if (orig_offset == 0x00E) {
            uint16_t hi = (val >> 16) & 0xffff;
            ingenic_msc_rom_command(s, hi);
            s->reg_cmdat &= 0x0000ffff;
        }
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
        break;
    case MSC_IFLG:
        /* Write 1 to clear */
        s->reg_iflg &= ~val;
        break;
    case MSC_CMD:
        s->reg_cmd = val;
        /* T41 ROM uses MSC_CMD byte 3 (offset 0x2F) as a self-clearing
         * STRPCL-like control register: bit 0 = stop clock, bit 1 = start
         * clock, bit 2 = START_OP, bit 3 = RESET.  Hardware processes the
         * request and clears the byte.  Auto-clear it here so the ROM's
         * poll loop terminates. */
        {
            uint32_t ctrl_byte3 = (val >> 24) & 0xff;
            if (ctrl_byte3) {
                s->reg_cmd &= 0x00ffffff;
                if (ctrl_byte3 & CTRL_RESET) {
                    ingenic_msc_reset_state(s);
                    return;
                }
            }
        }
        break;
    case MSC_ARG:
        s->reg_arg = val;
        break;
    case MSC_TXFIFO:
        if (s->data_is_write && s->data_pos + 4 <= s->data_total) {
            uint8_t b[4] = {
                val & 0xff, (val >> 8) & 0xff,
                (val >> 16) & 0xff, (val >> 24) & 0xff,
            };
            for (int i = 0; i < 4; i++) {
                sdbus_write_byte(&s->sdbus, b[i]);
                s->data_buf[s->data_pos + i] = b[i];
            }
            s->data_pos += 4;
            if (s->data_pos >= s->data_total) {
                s->reg_iflg |= IFLG_PRG_DONE | IFLG_DATA_TRAN_DONE;
            }
        }
        break;
    case MSC_LPM:
        s->reg_lpm = val;
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

static void ingenic_msc_realize(DeviceState *dev, Error **errp)
{
}

static void ingenic_msc_init(Object *obj)
{
    IngenicMscState *s = INGENIC_MSC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_msc_ops, s,
                          TYPE_INGENIC_MSC, INGENIC_MSC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    qbus_init(&s->sdbus, sizeof(s->sdbus),
              TYPE_SD_BUS, DEVICE(obj), "sd-bus");
}

static const VMStateDescription vmstate_ingenic_msc = {
    .name = TYPE_INGENIC_MSC,
    .version_id = 1,
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
        VMSTATE_UINT32(data_pos, IngenicMscState),
        VMSTATE_BOOL(data_is_write, IngenicMscState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_msc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_msc_realize;
    dc->vmsd = &vmstate_ingenic_msc;
    rc->phases.hold = ingenic_msc_reset_hold;
}

static const TypeInfo ingenic_msc_type_info = {
    .name = TYPE_INGENIC_MSC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicMscState),
    .instance_init = ingenic_msc_init,
    .class_init = ingenic_msc_class_init,
};

static void ingenic_msc_register_types(void)
{
    type_register_static(&ingenic_msc_type_info);
}

type_init(ingenic_msc_register_types)
