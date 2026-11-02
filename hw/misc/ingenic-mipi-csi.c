/*
 * Ingenic MIPI CSI-2 receiver
 *
 * A DesignWare CSI-2 host, version 1.30: the lanes at 0x04, then its
 * active-low resets. Out of DPHY_RSTZ (0x0c) and CSI2_RESETN (0x10) it
 * passes the sensor's frames to the VIC. tx-isp puts both back as it closes
 * and lets them go as it opens again, after the ISP's own reset. DPHY_RSTZ
 * also holds the D-PHY in reset. PHY_SHUTDOWNZ (0x08) the T31's tx-isp
 * clears and never sets, so it holds nothing back here.
 *
 * Its values out of reset and the bits a write keeps are as the parts read
 * them, one KiB of registers repeated across the window: the T20's, T30's
 * and T31's plain host ("dw"); the T23's, T41's and the T32's second
 * window's, with lane maps at 0x110 to 0x11c ("x2", the T40's second
 * window "x2-t40" with two more set); the T32's and T40's four-lane hosts
 * ("x4"). From 0x120 on, those parts read words that differ from part to
 * part and keep no write, which this model reads as 0. PHY_STATE reads 0x200
 * at rest; receiving, its clock lane reads in high speed and the lanes past
 * N_LANES stopped, as a T31X, T40XP and T41NQ read it while streaming. Live
 * status this model leaves alone: the received lanes' stop bits in the
 * blanking between lines, the counters at 0x040 (and 0x050 on the
 * four-lane hosts), and an idle host's view of a D-PHY another host let go.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/misc/ingenic-mipi-csi.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "trace.h"

#define CSI_VERSION         0x00
#define CSI_N_LANES         0x04
#define CSI_DPHY_RSTZ       0x0c
#define CSI_CSI2_RESETN     0x10
#define CSI_PHY_STATE       0x14

#define PHY_STATE_STOP(n)   (1 << (4 + (n)))    /* a data lane stopped */
#define PHY_STATE_CLK_HS    (1 << 8)
#define PHY_STATE_ULPS_NOT  (1 << 9)
#define PHY_STATE_BYPASS    (1 << 11)           /* the one bit a write keeps */

static const uint32_t wmask_dw[INGENIC_MIPI_CSI_REGS] = {
    [0x04 / 4] = 0x1, [0x08 / 4 ... 0x10 / 4] = 0x1,
    [0x14 / 4] = PHY_STATE_BYPASS,
    [0x18 / 4 ... 0x1c / 4] = 0xffffffff, [0x28 / 4] = 0x1ffffff3,
    [0x2c / 4] = 0x00ffff33, [0x30 / 4] = 0x3, [0x34 / 4] = 0x000100ff,
};

static const uint32_t wmask_x2[INGENIC_MIPI_CSI_REGS] = {
    [0x04 / 4] = 0x1, [0x08 / 4 ... 0x10 / 4] = 0x1,
    [0x14 / 4] = PHY_STATE_BYPASS,
    [0x18 / 4 ... 0x1c / 4] = 0xffffffff, [0x28 / 4] = 0x1ffffff3,
    [0x2c / 4] = 0x00ffff33, [0x30 / 4] = 0x3, [0x34 / 4] = 0x000100ff,
    [0x100 / 4] = 0x3, [0x110 / 4 ... 0x11c / 4] = 0xffffffff,
};

static const uint32_t wmask_x4[INGENIC_MIPI_CSI_REGS] = {
    [0x04 / 4] = 0x3, [0x08 / 4 ... 0x10 / 4] = 0x1,
    [0x14 / 4] = PHY_STATE_BYPASS,
    [0x18 / 4 ... 0x1c / 4] = 0xffffffff, [0x28 / 4] = 0x1fffffff,
    [0x2c / 4] = 0x00ffffff, [0x30 / 4] = 0x3, [0x34 / 4] = 0x000100ff,
    [0x80 / 4] = 0x1, [0x100 / 4] = 0xf,
    [0x110 / 4 ... 0x11c / 4] = 0xffffffff,
};

static const uint32_t reset_dw[INGENIC_MIPI_CSI_REGS] = {
    [0x00 / 4] = 0x3130322a, [0x04 / 4] = 0x1,
};

static const uint32_t reset_x2[INGENIC_MIPI_CSI_REGS] = {
    [0x00 / 4] = 0x3130322a, [0x04 / 4] = 0x1,
    [0x118 / 4] = 0xeeaa6622, [0x11c / 4] = 0x3377bbff,
};

static const uint32_t reset_x2_t40[INGENIC_MIPI_CSI_REGS] = {
    [0x00 / 4] = 0x3130322a, [0x04 / 4] = 0x1,
    [0x110 / 4] = 0xcc884400, [0x114 / 4] = 0x115599dd,
    [0x118 / 4] = 0xeeaa6622, [0x11c / 4] = 0x3377bbff,
};

static const uint32_t reset_x4[INGENIC_MIPI_CSI_REGS] = {
    [0x00 / 4] = 0x3130322a, [0x04 / 4] = 0x3,
};

static const struct {
    const char *model;
    unsigned lanes;
    const uint32_t *wmask;
    const uint32_t *reset;
} models[] = {
    { "dw", 2, wmask_dw, reset_dw },
    { "x2", 2, wmask_x2, reset_x2 },
    { "x2-t40", 2, wmask_x2, reset_x2_t40 },
    { "x4", 4, wmask_x4, reset_x4 },
};

static bool csi_rx(IngenicMipiCsiState *s)
{
    return s->regs[CSI_DPHY_RSTZ / 4] & s->regs[CSI_CSI2_RESETN / 4] & 1;
}

static void csi_update(IngenicMipiCsiState *s)
{
    bool rx = csi_rx(s);

    trace_ingenic_mipi_csi_rx(rx);
    qemu_set_irq(s->reset, !rx);
    qemu_set_irq(s->phy_reset, !(s->regs[CSI_DPHY_RSTZ / 4] & 1));
}

static uint32_t csi_phy_state(IngenicMipiCsiState *s)
{
    uint32_t st = PHY_STATE_ULPS_NOT |
                  (s->regs[CSI_PHY_STATE / 4] & PHY_STATE_BYPASS);

    if (csi_rx(s)) {
        st |= PHY_STATE_CLK_HS;
        for (unsigned n = (s->regs[CSI_N_LANES / 4] & 3) + 1; n < s->lanes;
             n++) {
            st |= PHY_STATE_STOP(n);
        }
    }
    return st;
}

static uint64_t csi_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicMipiCsiState *s = opaque;

    offset %= KiB;
    if (offset == CSI_PHY_STATE) {
        return csi_phy_state(s);
    }
    return s->regs[offset / 4];
}

static void csi_write(void *opaque, hwaddr offset, uint64_t val,
                      unsigned size)
{
    IngenicMipiCsiState *s = opaque;
    unsigned i = (offset % KiB) / 4;

    s->regs[i] = (s->regs[i] & ~s->wmask[i]) | (val & s->wmask[i]);
    if (i == CSI_DPHY_RSTZ / 4 || i == CSI_CSI2_RESETN / 4) {
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

    memcpy(s->regs, s->reset_values, sizeof(s->regs));
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
    qdev_init_gpio_out_named(DEVICE(obj), &s->phy_reset, "phy-reset", 1);
}

static void csi_realize(DeviceState *dev, Error **errp)
{
    IngenicMipiCsiState *s = INGENIC_MIPI_CSI(dev);

    for (int i = 0; i < ARRAY_SIZE(models); i++) {
        if (s->model && !strcmp(s->model, models[i].model)) {
            s->lanes = models[i].lanes;
            s->wmask = models[i].wmask;
            s->reset_values = models[i].reset;
            return;
        }
    }
    error_setg(errp, "ingenic-mipi-csi: unknown model '%s'",
               s->model ? s->model : "");
}

static const VMStateDescription vmstate_csi = {
    .name = TYPE_INGENIC_MIPI_CSI,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicMipiCsiState,
                             INGENIC_MIPI_CSI_REGS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property csi_properties[] = {
    DEFINE_PROP_STRING("model", IngenicMipiCsiState, model),
};

static void csi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic MIPI CSI-2 receiver";
    dc->realize = csi_realize;
    dc->vmsd = &vmstate_csi;
    rc->phases.hold = csi_reset_hold;
    rc->phases.exit = csi_reset_exit;
    device_class_set_props(dc, csi_properties);
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
