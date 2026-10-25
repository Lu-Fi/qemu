/*
 * Ingenic MIPI CSI-2 receiver
 *
 * A DesignWare CSI-2 host: the lanes at 0x04, then its active-low resets.
 * Out of DPHY_RSTZ (0x0c) and CSI2_RESETN (0x10) it passes the sensor's
 * frames to the VIC. tx-isp puts both back as it closes and lets them go
 * as it opens again, after the ISP's own reset. PHY_SHUTDOWNZ (0x08) it
 * clears and never sets, so it holds nothing back here.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/misc/ingenic-mipi-csi.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "trace.h"

#define CSI_DPHY_RSTZ       0x0c
#define CSI_CSI2_RESETN     0x10

static void csi_update(IngenicMipiCsiState *s)
{
    bool rx = s->regs[CSI_DPHY_RSTZ / 4] & s->regs[CSI_CSI2_RESETN / 4] & 1;

    trace_ingenic_mipi_csi_rx(rx);
    qemu_set_irq(s->reset, !rx);
}

static uint64_t csi_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicMipiCsiState *s = opaque;

    return s->regs[offset / 4];
}

static void csi_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicMipiCsiState *s = opaque;

    s->regs[offset / 4] = val;
    if (offset == CSI_DPHY_RSTZ || offset == CSI_CSI2_RESETN) {
        csi_update(s);
    }
}

static const MemoryRegionOps csi_ops = {
    .read = csi_read,
    .write = csi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void csi_reset_hold(Object *obj, ResetType type)
{
    IngenicMipiCsiState *s = INGENIC_MIPI_CSI(obj);

    memset(s->regs, 0, sizeof(s->regs));
}

static void csi_reset_exit(Object *obj, ResetType type)
{
    csi_update(INGENIC_MIPI_CSI(obj));
}

static void csi_init(Object *obj)
{
    IngenicMipiCsiState *s = INGENIC_MIPI_CSI(obj);

    memory_region_init_io(&s->iomem, obj, &csi_ops, s,
                          TYPE_INGENIC_MIPI_CSI, INGENIC_MIPI_CSI_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    qdev_init_gpio_out_named(DEVICE(obj), &s->reset, "reset", 1);
}

static const VMStateDescription vmstate_csi = {
    .name = TYPE_INGENIC_MIPI_CSI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicMipiCsiState,
                             INGENIC_MIPI_CSI_IOSIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void csi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic MIPI CSI-2 receiver";
    dc->vmsd = &vmstate_csi;
    rc->phases.hold = csi_reset_hold;
    rc->phases.exit = csi_reset_exit;
}

static const TypeInfo csi_type_info = {
    .name = TYPE_INGENIC_MIPI_CSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicMipiCsiState),
    .instance_init = csi_init,
    .class_init = csi_class_init,
};

static void csi_register_types(void)
{
    type_register_static(&csi_type_info);
}

type_init(csi_register_types)
