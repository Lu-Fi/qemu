/*
 * Ingenic A1 HDMI: the transmitter's controller and PHY, with a monitor
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_DISPLAY_INGENIC_A1_HDMI_H
#define HW_DISPLAY_INGENIC_A1_HDMI_H

#include "hw/core/sysbus.h"
#include "hw/misc/ingenic-a1-cpm.h"
#include "qom/object.h"

#define TYPE_INGENIC_A1_HDMI "ingenic-a1-hdmi"
OBJECT_DECLARE_SIMPLE_TYPE(IngenicA1HdmiState, INGENIC_A1_HDMI)

#define INGENIC_A1_HDMI_CTL_SIZE    0x20000
#define INGENIC_A1_HDMI_PHY_SIZE    0x1000
#define INGENIC_A1_HDMI_EDID_SIZE   256

struct IngenicA1HdmiState {
    SysBusDevice parent_obj;

    MemoryRegion ctl, phy;
    qemu_irq irq;
    qemu_irq hpd;               /* the HPD pin: high with a monitor */

    uint32_t regs[INGENIC_A1_HDMI_CTL_SIZE / 4];
    uint32_t phy_regs[INGENIC_A1_HDMI_PHY_SIZE / 4];
    uint8_t edid[INGENIC_A1_HDMI_EDID_SIZE];
    uint32_t edid_len;

    /* The audio sink: what AIC1 sent while the transmitter carried audio */
    bool audio_on;
    int audio_sign;             /* of the last left sample that was not zero */
    int64_t audio_last_ns;
    uint32_t audio_rate, audio_peak, audio_zrun;
    uint64_t audio_frames, audio_dropped, audio_nonzero, audio_active;
    uint64_t audio_crossings;

    /* properties */
    IngenicA1CpmState *cpm;     /* its CLKGR0 gates the block */
    bool connected;
    char *edid_file;
    uint32_t xres, yres;        /* the monitor's preferred mode */
};

/*
 * AIC1's frame sink (ingenic_aic_set_sink): one S16 stereo frame at rate Hz,
 * taken while the transmitter's audio path is enabled and dropped otherwise
 */
void ingenic_a1_hdmi_audio_frame(void *opaque, const int16_t frame[2],
                                 uint32_t rate);

#endif /* HW_DISPLAY_INGENIC_A1_HDMI_H */
