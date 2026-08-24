/*
 * Ingenic T31 SPI Flash Controller (SFC) emulation
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T31_SFC_H
#define HW_MISC_INGENIC_T31_SFC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T31_SFC "ingenic-t31-sfc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT31SfcState, INGENIC_T31_SFC)

#define INGENIC_T31_SFC_FLASH_SIZE  (16 * 1024 * 1024)
#define INGENIC_T31_SFC_FIFO_DEPTH  64
#define INGENIC_T31_SFC_CDT_ENTRIES 64

struct IngenicT31SfcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t glb;
    uint32_t dev_conf;
    uint32_t dev_sta_exp;
    uint32_t dev_sta_rt;
    uint32_t dev_sta_msk;
    uint32_t tran_conf[6];
    uint32_t tran_conf1[6];
    uint32_t tran_len;
    uint32_t dev_addr[6];
    uint32_t dev_addr_plus[6];
    uint32_t mem_addr;
    uint32_t sr;
    uint32_t scr;
    uint32_t intc;
    uint32_t cge;
    uint32_t cmd_idx;
    uint32_t col_addr;
    uint32_t row_addr;
    uint32_t sta_addr[2];
    uint32_t des_addr;
    uint32_t glb1;
    uint32_t dev1_sta_rt;

    uint32_t cdt[INGENIC_T31_SFC_CDT_ENTRIES * 4];

    uint32_t fifo[INGENIC_T31_SFC_FIFO_DEPTH];
    uint32_t fifo_pos;
    uint32_t fifo_len;
    uint32_t flash_pos;
    uint32_t words_total;
    bool write_enabled;
    bool writing;

    uint8_t *flash_data;
    uint32_t flash_size;

    /*
     * Block backend the flash image was loaded from. Held so we can
     * write modifications (erase, program) back to the underlying disk
     * image and have them survive across QEMU restarts. NULL when no
     * -drive was attached.
     */
    void *blk;
};

#endif /* HW_MISC_INGENIC_T31_SFC_H */
