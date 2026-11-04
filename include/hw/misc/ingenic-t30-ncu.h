/*
 * Ingenic T30 noise reducer (NCU) and its post-processor
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_T30_NCU_H
#define HW_MISC_INGENIC_T30_NCU_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_T30_NCU "ingenic-t30-ncu"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicT30NcuState, INGENIC_T30_NCU)

#define INGENIC_T30_NCU_IOSIZE  0x1000
#define INGENIC_T30_NCU_REGS    (0x580 / 4)

/* The control words the NCU and the post-processor each have */
typedef struct IngenicT30NcuCtl {
    uint8_t resets;             /* soft resets, RESET bits 15:8 */
    uint8_t stops;              /* stop requests, STOP bits 15:8 */
    bool stop;                  /* STOP bit 0 */
    bool stop_done;             /* STOP bit 1 */
    bool int_en;                /* INT_CNTRL bit 0 */
    bool done;                  /* INT_CNTRL bit 4 */
    uint32_t sreg;              /* SREG_CNTRL */
} IngenicT30NcuCtl;

struct IngenicT30NcuState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq pp_irq;
    qemu_irq frame_out;         /* a frame for the multi-scaler */
    QEMUBH *bh;

    uint32_t regs[INGENIC_T30_NCU_REGS];
    IngenicT30NcuCtl ncu, pp;
    uint32_t frames;            /* started, DEBUG_VAL select 0 */
    uint32_t clk;               /* 0x20 in its clock count view */
    uint16_t first;             /* START bits 15:0 */
    bool busy;
    bool hung;                  /* started as it cannot run */
    bool loaded;                /* SREG has loaded since the reset */
    bool ref1;                  /* the last frame went to REF1 */
    bool clk_view;
    bool pp_first;              /* PP_FST_FRM */
    uint32_t unimp_fun;         /* FUN_CNTRL last logged as unmodelled */
};

#endif /* HW_MISC_INGENIC_T30_NCU_H */
