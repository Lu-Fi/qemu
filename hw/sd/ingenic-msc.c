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

/* IFLG bits */
#define IFLG_TIME_OUT_RES       (1 << 9)
#define IFLG_TIME_OUT_READ      (1 << 8)
#define IFLG_TXFIFO_WR_REQ      (1 << 6)
#define IFLG_RXFIFO_RD_REQ      (1 << 5)
#define IFLG_END_CMD_RES        (1 << 2)
#define IFLG_PRG_DONE           (1 << 1)
#define IFLG_DATA_TRAN_DONE     (1 << 0)

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

    if (resp_type != CMDAT_RESPONSE_NONE && rsplen == 0) {
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

    s->reg_iflg |= IFLG_END_CMD_RES;

    /* Set up data transfer if requested */
    if (cmdat & CMDAT_DATA_EN) {
        uint32_t total = s->reg_nob * s->reg_blklen;

        if (total > sizeof(s->data_buf)) {
            total = sizeof(s->data_buf);
        }
        s->data_total = total;
        s->data_pos = 0;
        s->data_is_write = !!(cmdat & CMDAT_WRITE);

        if (!s->data_is_write) {
            /* Read direction: pull all bytes from the card now so the
             * driver can drain MSC_RXFIFO synchronously. */
            for (uint32_t i = 0; i < total; i++) {
                s->data_buf[i] = sdbus_read_byte(&s->sdbus);
            }
        }
    } else {
        s->data_total = 0;
        s->data_pos = 0;
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
    s->reg_imask = 0xffff;
    s->reg_iflg = 0;
    s->reg_cmd = 0;
    s->reg_arg = 0;
    s->reg_lpm = 0;
    s->resp_size = 0;
    s->resp_idx = 0;
    s->data_total = 0;
    s->data_pos = 0;
    s->data_is_write = false;
    memset(s->resp_buf, 0, sizeof(s->resp_buf));
}

static uint32_t ingenic_msc_status(IngenicMscState *s)
{
    uint32_t stat = 0;

    if (s->data_total == 0 || s->data_pos >= s->data_total) {
        stat |= STAT_DATA_FIFO_EMPTY;
    }
    if (s->data_total && s->data_pos >= s->data_total) {
        stat |= STAT_DATA_TRAN_DONE;
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
        case 0: return s->resp_buf[0];
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
                s->reg_iflg |= IFLG_DATA_TRAN_DONE;
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
    return val;
}

static void ingenic_msc_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicMscState *s = INGENIC_MSC(opaque);
    uint32_t val = (uint32_t)value;

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
        break;
    case MSC_IFLG:
        /* Write 1 to clear */
        s->reg_iflg &= ~val;
        break;
    case MSC_CMD:
        s->reg_cmd = val;
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
