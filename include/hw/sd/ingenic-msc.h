/*
 * Ingenic T31 MSC (MMC/SD) Controller emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_SD_INGENIC_MSC_H
#define HW_SD_INGENIC_MSC_H

#include "hw/core/sysbus.h"
#include "hw/sd/sd.h"
#include "qom/object.h"

#define TYPE_INGENIC_MSC "ingenic-msc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicMscState, INGENIC_MSC)

#define INGENIC_MSC_IOSIZE  0x1000
#define INGENIC_MSC_BUFSIZE (64 * 1024)

struct IngenicMscState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    SDBus sdbus;

    uint32_t reg_ctrl;
    uint32_t reg_clkrt;
    uint32_t reg_cmdat;
    uint32_t reg_resto;
    uint32_t reg_rdto;
    uint32_t reg_blklen;
    uint32_t reg_nob;
    uint32_t reg_imask;
    uint32_t reg_iflg;
    uint32_t reg_cmd;
    uint32_t reg_arg;
    uint32_t reg_lpm;

    /* Response state. The Ingenic MSC presents the response 16 bits at a
     * time on MSC_RES. resp_buf holds the bytes returned by sdbus_do_command
     * (big-endian); resp_idx tracks the next 16-bit window the driver will
     * read. resp_size is 0 (no response yet), 4 (short), or 16 (long). */
    uint8_t resp_buf[16];
    uint8_t resp_size;
    uint8_t resp_idx;

    /* Data transfer state */
    uint8_t data_buf[INGENIC_MSC_BUFSIZE];
    uint32_t data_total;
    uint32_t data_pos;
    bool data_is_write;
};

#endif
