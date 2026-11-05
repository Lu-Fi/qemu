/*
 * Ingenic IVDC, the ISP to encoder direct connection (T23, T32, T33, T41)
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_MISC_INGENIC_IVDC_H
#define HW_MISC_INGENIC_IVDC_H

#include "hw/core/sysbus.h"
#include "qom/object.h"

#define TYPE_INGENIC_IVDC "ingenic-ivdc"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicIvdcState, INGENIC_IVDC)

#define INGENIC_IVDC_IOSIZE     0x10000
#define INGENIC_IVDC_SENSORS    4

struct IngenicIvdcState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq waiting;           /* a frame's lines wait for an encoder */
    QEMUTimer *timer;

    uint32_t regs[INGENIC_IVDC_IOSIZE / 4];
    /* the frame the ISP is sending */
    int64_t start, line_ns;     /* its line clock, from the first line */
    int64_t began;              /* when it started, for the period */
    uint32_t lines, sensor;
    uint32_t state;
    uint32_t pending;           /* its events still to raise */
    int64_t taken[2];           /* when VPU0 and VPU1 took it, or -1 */
    uint32_t queued[2];         /* starts waiting for the next frame */
    uint32_t ring;              /* the DDR ring's line, modulo twice its size */
    uint32_t frames;            /* taken since the last overflow */
    uint32_t pair_next;         /* of 0x208 and 0x3a8, the one latched */
    uint32_t pair_value;
    bool stopped;               /* its clock: the CPM's divider stopped */
    bool hold_lines;            /* a frame's lines wait for an encoder */
    bool held;                  /* this one's do */
    int64_t held_until;         /* and are dropped then, untaken */
};

#endif /* HW_MISC_INGENIC_IVDC_H */
