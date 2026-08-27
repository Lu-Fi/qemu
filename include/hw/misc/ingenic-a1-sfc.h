/*
 * Ingenic A1 SPI Flash Controller V2 (SFC) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_A1_SFC_H
#define HW_MISC_INGENIC_A1_SFC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_SFC "ingenic-a1-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1SfcState, INGENIC_A1_SFC)

#define INGENIC_A1_SFC_IOSIZE       0x2000
#define INGENIC_A1_SFC_FLASH_SIZE   (16 * 1024 * 1024)
#define INGENIC_A1_SFC_FIFO_DEPTH   64
#define INGENIC_A1_SFC_CDT_ENTRIES  64

struct IngenicA1SfcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    /* Core registers */
    uint32_t glb;
    uint32_t dev_conf;
    uint32_t tran_conf[6];
    uint32_t tran_len;
    uint32_t dev_addr[6];
    uint32_t dev_addr_plus[6];
    uint32_t mem_addr;
    uint32_t sr;
    uint32_t intc;
    uint32_t cge;
    uint32_t cmd_idx;
    uint32_t col_addr;
    uint32_t row_addr;

    /* V2 extended registers: DES_ADDR, GLB1, DEV1_STA_RT, TRAN_CONF1[6] */
    uint32_t des_addr;
    uint32_t glb1;
    uint32_t dev1_sta_rt;
    uint32_t tran_conf1[6];
    uint32_t v2_regs[16];

    /* Command descriptor table */
    uint32_t cdt[INGENIC_A1_SFC_CDT_ENTRIES * 4];

    /* Data FIFO */
    uint32_t fifo[INGENIC_A1_SFC_FIFO_DEPTH];
    uint32_t fifo_pos;
    uint32_t fifo_len;

    /* Transfer state */
    uint32_t flash_pos;
    uint32_t words_total;
    bool write_enabled;
    bool writing;

    /* Backing flash */
    uint8_t *flash_data;
    uint32_t flash_size;
    void *blk;
};

#endif /* HW_MISC_INGENIC_A1_SFC_H */
