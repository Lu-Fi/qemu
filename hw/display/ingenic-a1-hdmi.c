/*
 * Ingenic A1 HDMI: the transmitter's controller and PHY, with a monitor
 *
 * What the A1's VDE driver and U-Boot's VO driver touch, in the 128 KiB
 * window at 0x13380000 (the driver reaches it at VDE + 0x80000) and the
 * PHY page at 0x10075000. Plain registers keep what they are given, but:
 *
 *  - 0x410 (status) reads 0x3c (receiver sense) with a monitor, plus bit 1
 *    (PLL lock) once the PHY's 0x2c0 bit 2 powers it. 0x400 and 0x404 read
 *    what the driver's debug status looks for.
 *  - 0x414 is the interrupt status (write one to clear) and 0x614 its mask
 *    (set masks); the interrupt is 0x414 & ~0x614.
 *  - The DDC master at 0x1f800: 0x1f800 the slave, 0x1f804 the offset,
 *    0x1f808 the data to write, 0x1f80c the data read. A write of 1 (read),
 *    2 (read with the segment in 0x1f828) or 0x10 (write) to 0x1f810 runs
 *    at once and sets 0x414 bit 1, or bit 0 when the slave does not answer.
 *    The monitor answers at 0x50 with its EDID (256 bytes: a base block and
 *    a CEA-861 one) and takes writes without effect.
 *  - The PHY's 0x334 reads 0x55 and 0x2a4 reads 1 while locked.
 *
 * With no monitor the status words (0x400 to 0x414) and the PHY read 0, as
 * an A1N reads them with the output not started. The mask words from 0x600
 * and the DDC's from 0x1f814 reset to what the A1N reads.
 *
 * The audio registers are a DesignWare HDMI TX's (byte registers at a
 * 4-byte stride; the address is offset / 4): the ID block at 0, FC_INVIDCONF
 * (bit 3, HDMI mode, 0x4000), FC_AUDSSTAT at 0x4190, MC_CLKDIS (bit 3 stops
 * the audio clock, 0x10004), the audio sampler AUD_CONF0 to AUD_CONF2 at
 * 0xc400 and the clock regenerator AUD_N1..AUD_CTS3 at 0xc800. They hold
 * what is written but for the ID block (read only), AUD_CONF0's FIFO reset
 * bit 7 (self clearing) and FC_AUDSSTAT (bit 0: a sample is present). The
 * vendor's firmware sets 0xc400 to 0xa1, 0xc404 to the sample width, 0xc40c
 * to 4, N and CTS (CTS_MANUAL set) and bit 3 of 0x438c (audio infoframe on).
 *
 * AIC1's frames reach the sink through ingenic_a1_hdmi_audio_frame(). They
 * go nowhere unless the output is in HDMI mode and locked, the audio clock
 * runs and AUD_CONF0 enables an I2S input; the sink counts them (QOM
 * properties audio-frames, -dropped, -nonzero, -active (the frames of
 * non-silence: a tone's own zero samples count, a run of 16 does not),
 * -peak, -crossings of the left channel, -rate) so a test can tell a tone
 * from silence and measure its pitch.
 *
 * CPM CLKGR0 bits 27 (gate_hdmi) and 28 (gate_vde) shut the window and the
 * PHY: every read is 0 and writes are dropped, what was written before
 * comes back when the gate opens, as on an A1N (libimp shuts both when it
 * stops the output). An A1N with only bit 27 shut hangs on a read; the
 * model reads 0 and logs it. Both bits are clear after the kernel boots.
 *
 * The HPD pin follows "connected" (a property, so a test or monitor can
 * unplug it). Assumed, not measured: the status bit meanings above beyond
 * what the driver tests, the DDC's timing (instant), FC_AUDSSTAT's meaning
 * (only read as 1 while playing) and that audio needs the locked PHY.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/display/edid.h"
#include "hw/display/ingenic-a1-hdmi.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "trace.h"

#define H_RXSENSE       0x410
#define H_IRQ_STATUS    0x414
#define H_IRQ_MASK      0x614
#define H_DDC_SLAVE     0x1f800
#define H_DDC_OFFSET    0x1f804
#define H_DDC_WDATA     0x1f808
#define H_DDC_RDATA     0x1f80c
#define H_DDC_CMD       0x1f810
#define H_DDC_SEGMENT   0x1f828

#define H_ID_END        0x20        /* the ID block, read only */
#define H_INVIDCONF     0x4000
#define H_AUDSSTAT      0x4190
#define H_AUDINFO       0x438c
#define H_MC_CLKDIS     0x10004
#define H_AUD_CONF0     0xc400
#define H_AUD_N1        0xc800
#define H_AUD_N2        0xc804
#define H_AUD_N3        0xc808
#define H_AUD_CTS1      0xc80c
#define H_AUD_CTS2      0xc810
#define H_AUD_CTS3      0xc814

#define INVIDCONF_HDMI  BIT(3)
#define MC_CLKDIS_AUD   BIT(3)
#define AUD_CONF0_RST   BIT(7)
#define AUD_CONF0_I2S   0x0f
#define AUDIO_SILENCE   16
#define AUDSSTAT_WINDOW (20 * SCALE_MS)

#define DDC_READ        1
#define DDC_READ_SEG    2
#define DDC_WRITE       0x10
#define IRQ_NACK        1
#define IRQ_DONE        2

#define GATE_REG        0x30            /* CPM CLKGR0 */
#define GATE_HDMI       BIT(27)
#define GATE_VDE        BIT(28)

#define H_PHYRSTZ       0x10014     /* bit 0 releases the PHY's reset */
#define PHY_BLOCK       0x400

#define PHY_POWER       0x2c0
#define PHY_STATE       0x2a4
#define PHY_SENSE       0x334
#define PHY_POWER_ON    4

/*
 * The PHY's registers as an A1N reads them with the reset released (offset,
 * value, the bits a write keeps): a 0x400-byte block that repeats through
 * the 4 KiB. A word not listed reads 0 and keeps nothing.
 */
static const struct {
    uint16_t off;
    uint32_t reset, wmask;
} phy_table[] = {
    { 0x000, 0x000000c0, 0x000000ff },
    { 0x004, 0x00000000, 0x000000ff },
    { 0x008, 0x00000081, 0x000000ff },
    { 0x00c, 0x00000000, 0x000000ff },
    { 0x014, 0x00000000, 0x000000ff },
    { 0x01c, 0x00000000, 0x000000ff },
    { 0x024, 0x00000000, 0x000000ff },
    { 0x028, 0x00000010, 0x000000ff },
    { 0x02c, 0x00000002, 0x000000ff },
    { 0x030, 0x00000001, 0x000000ff },
    { 0x034, 0x00000000, 0x000000ff },
    { 0x038, 0x00000000, 0x000000ff },
    { 0x03c, 0x00000000, 0x000000ff },
    { 0x040, 0x00000055, 0x000000ff },
    { 0x044, 0x00000055, 0x000000ff },
    { 0x048, 0x00000055, 0x000000ff },
    { 0x04c, 0x00000001, 0x000000ff },
    { 0x050, 0x00000000, 0x000000ff },
    { 0x054, 0x00000000, 0x000000ff },
    { 0x058, 0x00000000, 0x000000ff },
    { 0x05c, 0x00000000, 0x000000ff },
    { 0x060, 0x00000000, 0x000000ff },
    { 0x064, 0x00000055, 0x000000ff },
    { 0x068, 0x00000055, 0x000000ff },
    { 0x06c, 0x00000055, 0x000000ff },
    { 0x070, 0x00000000, 0x000000ff },
    { 0x074, 0x00000000, 0x000000ff },
    { 0x078, 0x00000000, 0x000000ff },
    { 0x07c, 0x00000000, 0x000000ff },
    { 0x080, 0x00000055, 0x000000ff },
    { 0x084, 0x00000055, 0x000000ff },
    { 0x088, 0x00000055, 0x000000ff },
    { 0x08c, 0x00000010, 0x000000ff },
    { 0x090, 0x00000000, 0x000000ff },
    { 0x094, 0x00000000, 0x000000ff },
    { 0x098, 0x00000000, 0x000000ff },
    { 0x09c, 0x00000000, 0x000000ff },
    { 0x0a0, 0x00000000, 0x000000ff },
    { 0x0a4, 0x00000010, 0x000000ff },
    { 0x0a8, 0x00000000, 0x000000ff },
    { 0x0ac, 0x00000000, 0x000000ff },
    { 0x0b0, 0x00000000, 0x000000ff },
    { 0x0b4, 0x00000000, 0x000000ff },
    { 0x0b8, 0x00000011, 0x000000ff },
    { 0x0bc, 0x00000001, 0x000000ff },
    { 0x0c0, 0x00000055, 0x000000ff },
    { 0x0c4, 0x00000055, 0x000000ff },
    { 0x0c8, 0x00000055, 0x000000ff },
    { 0x0cc, 0x00000000, 0x000000ff },
    { 0x0d0, 0x00000000, 0x000000ff },
    { 0x0d4, 0x00000000, 0x000000ff },
    { 0x0d8, 0x00000000, 0x000000ff },
    { 0x0dc, 0x00000001, 0x000000ff },
    { 0x0e0, 0x00000000, 0x000000ff },
    { 0x0e4, 0x00000000, 0x000000ff },
    { 0x0e8, 0x00000000, 0x000000ff },
    { 0x0ec, 0x00000000, 0x000000ff },
    { 0x0f0, 0x00000011, 0x000000ff },
    { 0x0f4, 0x00000010, 0x000000ff },
    { 0x0f8, 0x00000000, 0x000000ff },
    { 0x0fc, 0x00000001, 0x000000ff },
    { 0x100, 0x00000000, 0x000000ff },
    { 0x104, 0x00000000, 0x000000ff },
    { 0x108, 0x00000000, 0x000000ff },
    { 0x10c, 0x00000000, 0x000000ff },
    { 0x110, 0x00000000, 0x000000ff },
    { 0x114, 0x00000000, 0x000000ff },
    { 0x118, 0x00000000, 0x000000ff },
    { 0x11c, 0x00000000, 0x000000ff },
    { 0x120, 0x00000000, 0x000000ff },
    { 0x124, 0x00000000, 0x000000ff },
    { 0x128, 0x00000000, 0x000000ff },
    { 0x12c, 0x00000000, 0x000000ff },
    { 0x130, 0x00000000, 0x000000ff },
    { 0x134, 0x00000000, 0x000000ff },
    { 0x138, 0x00000000, 0x000000ff },
    { 0x280, 0x00000001, 0x000000ff },
    { 0x284, 0x00000001, 0x000000ff },
    { 0x288, 0x000000f0, 0x000000ff },
    { 0x28c, 0x00000063, 0x000000ff },
    { 0x290, 0x00000015, 0x000000ff },
    { 0x294, 0x00000041, 0x000000ff },
    { 0x298, 0x00000042, 0x000000ff },
    { 0x29c, 0x0000005a, 0x000000ff },
    { 0x2a0, 0x00000080, 0x000000ff },
    { 0x2a4, 0x00000000, 0x000000fe },
    { 0x2a8, 0x0000000f, 0x000000ff },
    { 0x2ac, 0x00000001, 0x000000ff },
    { 0x2b0, 0x00000014, 0x000000ff },
    { 0x2b4, 0x00000009, 0x000000ff },
    { 0x2b8, 0x0000005a, 0x000000ff },
    { 0x2bc, 0x00000000, 0x000000fe },
    { 0x2c0, 0x0000000a, 0x000000ff },
    { 0x2c4, 0x00000000, 0x000000ff },
    { 0x2c8, 0x00000080, 0x000000ff },
    { 0x2cc, 0x00000000, 0x000000ff },
    { 0x2d0, 0x00000000, 0x000000ff },
    { 0x2d4, 0x0000001c, 0x000000ff },
    { 0x2d8, 0x0000001c, 0x000000ff },
    { 0x2dc, 0x0000001c, 0x000000ff },
    { 0x2e0, 0x0000001c, 0x000000ff },
    { 0x2e4, 0x00000000, 0x000000ff },
    { 0x2e8, 0x00000055, 0x000000ff },
    { 0x2ec, 0x00000000, 0x000000ff },
    { 0x2f0, 0x00000000, 0x000000ff },
    { 0x2f4, 0x00000000, 0x000000ff },
    { 0x2f8, 0x00000000, 0x000000ff },
    { 0x2fc, 0x00000000, 0x000000ff },
    { 0x300, 0x00000000, 0x000000ff },
    { 0x304, 0x00000000, 0x000000ff },
    { 0x308, 0x00000000, 0x000000ff },
    { 0x30c, 0x0000000f, 0x000000ff },
    { 0x310, 0x0000003f, 0x000000c0 },
    { 0x314, 0x00000004, 0x000000ff },
    { 0x318, 0x00000000, 0x000000ff },
    { 0x31c, 0x00000000, 0x000000f6 },
    { 0x320, 0x00000000, 0x000000c0 },
    { 0x324, 0x00000000, 0x000000c0 },
    { 0x328, 0x00000000, 0x000000c0 },
    { 0x32c, 0x00000000, 0x000000c0 },
    { 0x330, 0x00000000, 0x000000ff },
    { 0x334, 0x000000aa, 0x00000000 },
    { 0x338, 0x000000bb, 0x00000000 },
    { 0x33c, 0x0000003b, 0x00000000 },
    { 0x340, 0x00000046, 0x000000ff },
    { 0x344, 0x00000000, 0x000000ff },
    { 0x348, 0x00000000, 0x000000ff },
    { 0x34c, 0x00000000, 0x000000ff },
    { 0x350, 0x0000007f, 0x000000ff },
    { 0x354, 0x00000000, 0x000000ff },
    { 0x358, 0x00000080, 0x000000ff },
    { 0x35c, 0x00000002, 0x000000ff },
    { 0x360, 0x00000001, 0x000000ff },
    { 0x364, 0x00000040, 0x000000ff },
    { 0x368, 0x00000001, 0x000000ff },
    { 0x36c, 0x000000ef, 0x000000ff },
    { 0x370, 0x00000082, 0x000000ff },
    { 0x374, 0x0000000f, 0x000000f0 },
    { 0x378, 0x00000000, 0x000000ff },
    { 0x37c, 0x00000000, 0x000000ff },
};

static const uint8_t cea_vics[] = {
    16, 4, 31, 19, 32, 33, 34, 1, 2, 3, 17, 18,
};

static void hdmi_build_edid(IngenicA1HdmiState *s)
{
    qemu_edid_info info = {
        .vendor = "ING",
        .name = "A1 HDMI",
        .serial = "0",
        .width_mm = 530,
        .height_mm = 300,
        .prefx = s->xres,
        .prefy = s->yres,
        .refresh_rate = 60000,
    };
    uint8_t *cea = s->edid + 128;
    unsigned n = 4, sum = 0;

    memset(s->edid, 0, sizeof(s->edid));
    qemu_edid_generate(s->edid, 128, &info);
    s->edid[126] = 1;
    for (int i = 0; i < 127; i++) {
        sum += s->edid[i];
    }
    s->edid[127] = -sum;

    cea[0] = 0x02;
    cea[1] = 0x03;
    cea[3] = 0x00;
    cea[n++] = 2 << 5 | sizeof(cea_vics);
    memcpy(cea + n, cea_vics, sizeof(cea_vics));
    n += sizeof(cea_vics);
    /* HDMI vendor block: OUI 00-0C-03, source address 1.0.0.0 */
    cea[n++] = 3 << 5 | 5;
    cea[n++] = 0x03;
    cea[n++] = 0x0c;
    cea[n++] = 0x00;
    cea[n++] = 0x10;
    cea[n++] = 0x00;
    cea[2] = n;
    sum = 0;
    for (int i = 0; i < 127; i++) {
        sum += cea[i];
    }
    cea[127] = -sum;
    s->edid_len = 256;
}

static bool hdmi_gated(IngenicA1HdmiState *s, bool log)
{
    uint32_t g = s->cpm ? s->cpm->regs[GATE_REG / 4] : 0;

    if (log && (g & GATE_HDMI) && !(g & GATE_VDE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-a1-hdmi: access with only "
                      "gate_hdmi shut hangs the bus of an A1N\n");
    }
    return g & (GATE_HDMI | GATE_VDE);
}

/* The PHY is in reset (every read 0, writes dropped) until PHYRSTZ is set */
static bool hdmi_phy_up(IngenicA1HdmiState *s)
{
    return s->regs[H_PHYRSTZ / 4] & 1;
}

static void hdmi_phy_reset(IngenicA1HdmiState *s)
{
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    for (int i = 0; i < ARRAY_SIZE(phy_table); i++) {
        s->phy_regs[phy_table[i].off / 4] = phy_table[i].reset;
    }
}

static uint32_t hdmi_phy_wmask(hwaddr off)
{
    for (int i = 0; i < ARRAY_SIZE(phy_table); i++) {
        if (phy_table[i].off == off) {
            return phy_table[i].wmask;
        }
    }
    return 0;
}

static bool hdmi_locked(IngenicA1HdmiState *s)
{
    return s->connected && hdmi_phy_up(s) &&
           (s->phy_regs[PHY_POWER / 4] & PHY_POWER_ON);
}

static bool hdmi_audio_enabled(IngenicA1HdmiState *s)
{
    return !hdmi_gated(s, false) && hdmi_locked(s) &&
           (s->regs[H_INVIDCONF / 4] & INVIDCONF_HDMI) &&
           !(s->regs[H_MC_CLKDIS / 4] & MC_CLKDIS_AUD) &&
           (s->regs[H_AUD_CONF0 / 4] & AUD_CONF0_I2S);
}

/* N or CTS: three byte registers, the top one four bits wide */
static uint32_t hdmi_aud_word(IngenicA1HdmiState *s, uint32_t lo)
{
    return (s->regs[lo / 4] & 0xff) | (s->regs[lo / 4 + 1] & 0xff) << 8 |
           (s->regs[lo / 4 + 2] & 0xf) << 16;
}

void ingenic_a1_hdmi_audio_frame(void *opaque, const int16_t frame[2],
                                 uint32_t rate)
{
    IngenicA1HdmiState *s = opaque;
    uint32_t l = abs(frame[0]), r = abs(frame[1]);

    if (!hdmi_audio_enabled(s)) {
        if (s->audio_on) {
            s->audio_on = false;
            trace_ingenic_a1_hdmi_audio_stop(s->audio_frames);
        }
        s->audio_dropped++;
        return;
    }
    if (!s->audio_on) {
        s->audio_on = true;
        trace_ingenic_a1_hdmi_audio_start(rate,
            hdmi_aud_word(s, H_AUD_N1), hdmi_aud_word(s, H_AUD_CTS1));
    }
    s->audio_frames++;
    s->audio_rate = rate;
    s->audio_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->audio_peak = MAX(s->audio_peak, MAX(l, r));
    if (l || r) {
        /* A tone has zero samples in it: only a long run is silence */
        s->audio_nonzero++;
        s->audio_active += (s->audio_zrun < AUDIO_SILENCE ? s->audio_zrun : 0)
                           + 1;
        s->audio_zrun = 0;
    } else {
        s->audio_zrun++;
    }
    if (frame[0]) {
        int sign = frame[0] > 0 ? 1 : -1;

        if (s->audio_sign && sign != s->audio_sign) {
            s->audio_crossings++;
        }
        s->audio_sign = sign;
    }
}

static void hdmi_update_irq(IngenicA1HdmiState *s)
{
    qemu_set_irq(s->irq, !!(s->regs[H_IRQ_STATUS / 4] &
                            ~s->regs[H_IRQ_MASK / 4]));
}

static void hdmi_ddc(IngenicA1HdmiState *s, uint32_t cmd)
{
    uint32_t slave = s->regs[H_DDC_SLAVE / 4] & 0xff;
    uint32_t off = s->regs[H_DDC_OFFSET / 4] & 0xff;

    if (!s->connected || slave != 0x50) {
        s->regs[H_IRQ_STATUS / 4] |= IRQ_NACK;
    } else {
        if (cmd == DDC_READ || cmd == DDC_READ_SEG) {
            uint32_t seg = cmd == DDC_READ_SEG ?
                           s->regs[H_DDC_SEGMENT / 4] & 0xff : 0;
            uint32_t at = seg * 256 + off;

            s->regs[H_DDC_RDATA / 4] = at < s->edid_len ? s->edid[at] : 0;
        }
        s->regs[H_IRQ_STATUS / 4] |= IRQ_DONE;
    }
    hdmi_update_irq(s);
}

static uint64_t hdmi_ctl_read4(void *opaque, hwaddr off)
{
    IngenicA1HdmiState *s = opaque;

    if (hdmi_gated(s, true)) {
        return 0;
    }
    switch (off) {
    case 0x400:
        return s->connected ? 7 : 0;
    case 0x404:
        return s->connected ? 2 : 0;
    case H_AUDSSTAT:
        return s->audio_on &&
               qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->audio_last_ns <
               AUDSSTAT_WINDOW;
    case H_RXSENSE:
        return (s->connected ? 0x3c : 0) | (hdmi_locked(s) ? 2 : 0);
    default:
        return s->regs[off / 4];
    }
}

static void hdmi_ctl_write4(void *opaque, hwaddr off, uint64_t v)
{
    IngenicA1HdmiState *s = opaque;

    if (off < H_ID_END || hdmi_gated(s, true)) {
        return;
    }
    switch (off) {
    case 0x400:
    case 0x404:
    case H_RXSENSE:
    case H_AUDSSTAT:
        return;
    case H_AUD_CONF0:
        /* The FIFO reset bit clears itself */
        s->regs[off / 4] = v & ~AUD_CONF0_RST;
        return;
    case H_IRQ_STATUS:
        s->regs[off / 4] &= ~v;
        hdmi_update_irq(s);
        return;
    case H_DDC_CMD:
        s->regs[off / 4] = v;
        hdmi_ddc(s, v);
        return;
    case H_PHYRSTZ:
        if ((v & 1) && !hdmi_phy_up(s)) {
            hdmi_phy_reset(s);
        }
        s->regs[off / 4] = v;
        return;
    default:
        s->regs[off / 4] = v;
        if (off == H_IRQ_MASK) {
            hdmi_update_irq(s);
        }
    }
}

static uint64_t hdmi_phy_read4(void *opaque, hwaddr off)
{
    IngenicA1HdmiState *s = opaque;

    if (hdmi_gated(s, true) || !hdmi_phy_up(s)) {
        return 0;
    }
    off %= PHY_BLOCK;
    if (off == PHY_SENSE && hdmi_locked(s)) {
        return 0x55;
    }
    if (off == PHY_STATE && hdmi_locked(s)) {
        return 1;
    }
    return s->phy_regs[off / 4];
}

static void hdmi_phy_write4(void *opaque, hwaddr off, uint64_t v)
{
    IngenicA1HdmiState *s = opaque;

    uint32_t m;

    if (hdmi_gated(s, true) || !hdmi_phy_up(s)) {
        return;
    }
    off %= PHY_BLOCK;
    m = hdmi_phy_wmask(off);
    s->phy_regs[off / 4] = (s->phy_regs[off / 4] & ~m) | (v & m);
}

/* Sub-word accesses, as the CPU splits a word stored at an odd address */
#define HDMI_SPLIT(name)                                                  \
static uint64_t name##_read(void *opaque, hwaddr off, unsigned size)      \
{                                                                         \
    return extract32(name##_read4(opaque, off & ~3), (off & 3) * 8,       \
                     size * 8);                                           \
}                                                                         \
                                                                          \
static void name##_write(void *opaque, hwaddr off, uint64_t v,            \
                         unsigned size)                                   \
{                                                                         \
    if (size < 4) {                                                       \
        v = deposit32(name##_read4(opaque, off & ~3), (off & 3) * 8,      \
                      size * 8, v);                                       \
    }                                                                     \
    name##_write4(opaque, off & ~3, v);                                   \
}

HDMI_SPLIT(hdmi_ctl)
HDMI_SPLIT(hdmi_phy)

static const MemoryRegionOps hdmi_ctl_ops = {
    .read = hdmi_ctl_read,
    .write = hdmi_ctl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static const MemoryRegionOps hdmi_phy_ops = {
    .read = hdmi_phy_read,
    .write = hdmi_phy_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void hdmi_set_connected(Object *obj, bool v, Error **errp)
{
    IngenicA1HdmiState *s = INGENIC_A1_HDMI(obj);

    s->connected = v;
    qemu_set_irq(s->hpd, v);
}

static bool hdmi_get_connected(Object *obj, Error **errp)
{
    return INGENIC_A1_HDMI(obj)->connected;
}

/* What an A1N reads with its gate open and nothing written */
static const struct {
    uint32_t reg, val;
} hdmi_reset[] = {
    { 0x600, 0x18 }, { 0x604, 0x4 }, { 0x608, 0x10 }, { 0x60c, 0x8 },
    { 0x614, 0x4 },
    { 0x00, 0x21 }, { 0x04, 0x1a }, { 0x08, 0xa0 }, { 0x0c, 0x01 },
    { 0x10, 0x9a }, { 0x14, 0x22 }, { 0x18, 0xfe },
    { H_AUD_CONF0, 0x2f }, { H_AUD_CONF0 + 0x10, 0x10 },
    { H_AUD_CTS3 + 8, 0x11 },
    { 0x1f814, 0x40 }, { 0x1f81c, 0xb }, { 0x1f824, 1 }, { 0x1f830, 0x6c },
    { 0x1f838, 0x7f }, { 0x1f840, 0x11 },
};

static void hdmi_reset_hold(Object *obj, ResetType type)
{
    IngenicA1HdmiState *s = INGENIC_A1_HDMI(obj);

    s->audio_on = false;
    s->audio_sign = 0;
    s->audio_last_ns = 0;
    s->audio_rate = s->audio_peak = s->audio_zrun = 0;
    s->audio_frames = s->audio_dropped = s->audio_nonzero = 0;
    s->audio_active = 0;
    s->audio_crossings = 0;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->phy_regs, 0, sizeof(s->phy_regs));
    for (int i = 0; i < ARRAY_SIZE(hdmi_reset); i++) {
        s->regs[hdmi_reset[i].reg / 4] = hdmi_reset[i].val;
    }
    hdmi_update_irq(s);
    qemu_set_irq(s->hpd, s->connected);
}

static void hdmi_realize(DeviceState *dev, Error **errp)
{
    IngenicA1HdmiState *s = INGENIC_A1_HDMI(dev);

    if (s->edid_file) {
        g_autofree char *data = NULL;
        gsize len;

        if (!g_file_get_contents(s->edid_file, &data, &len, NULL) ||
            len < 128 || len > sizeof(s->edid)) {
            error_setg(errp, "edid-file must hold 128 to 256 bytes");
            return;
        }
        memset(s->edid, 0, sizeof(s->edid));
        memcpy(s->edid, data, len);
        s->edid_len = len;
    } else {
        hdmi_build_edid(s);
    }
}

static void hdmi_init(Object *obj)
{
    IngenicA1HdmiState *s = INGENIC_A1_HDMI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    s->connected = true;
    memory_region_init_io(&s->ctl, obj, &hdmi_ctl_ops, s, "ingenic-a1-hdmi",
                          INGENIC_A1_HDMI_CTL_SIZE);
    memory_region_init_io(&s->phy, obj, &hdmi_phy_ops, s,
                          "ingenic-a1-hdmi-phy", INGENIC_A1_HDMI_PHY_SIZE);
    sysbus_init_mmio(sbd, &s->ctl);
    sysbus_init_mmio(sbd, &s->phy);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->hpd, "hpd", 1);
    object_property_add_bool(obj, "connected", hdmi_get_connected,
                             hdmi_set_connected);
    object_property_add_uint64_ptr(obj, "audio-frames", &s->audio_frames,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "audio-dropped", &s->audio_dropped,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "audio-nonzero", &s->audio_nonzero,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "audio-active", &s->audio_active,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint64_ptr(obj, "audio-crossings",
                                   &s->audio_crossings, OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "audio-peak", &s->audio_peak,
                                   OBJ_PROP_FLAG_READ);
    object_property_add_uint32_ptr(obj, "audio-rate", &s->audio_rate,
                                   OBJ_PROP_FLAG_READ);
}

static const VMStateDescription vmstate_ingenic_a1_hdmi = {
    .name = "ingenic-a1-hdmi",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicA1HdmiState,
                             INGENIC_A1_HDMI_CTL_SIZE / 4),
        VMSTATE_UINT32_ARRAY(phy_regs, IngenicA1HdmiState,
                             INGENIC_A1_HDMI_PHY_SIZE / 4),
        VMSTATE_BOOL(connected, IngenicA1HdmiState),
        VMSTATE_BOOL_V(audio_on, IngenicA1HdmiState, 2),
        VMSTATE_INT32_V(audio_sign, IngenicA1HdmiState, 2),
        VMSTATE_INT64_V(audio_last_ns, IngenicA1HdmiState, 2),
        VMSTATE_UINT32_V(audio_rate, IngenicA1HdmiState, 2),
        VMSTATE_UINT32_V(audio_peak, IngenicA1HdmiState, 2),
        VMSTATE_UINT64_V(audio_frames, IngenicA1HdmiState, 2),
        VMSTATE_UINT64_V(audio_dropped, IngenicA1HdmiState, 2),
        VMSTATE_UINT64_V(audio_nonzero, IngenicA1HdmiState, 2),
        VMSTATE_UINT64_V(audio_crossings, IngenicA1HdmiState, 2),
        VMSTATE_UINT32_V(audio_zrun, IngenicA1HdmiState, 2),
        VMSTATE_UINT64_V(audio_active, IngenicA1HdmiState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static const Property hdmi_props[] = {
    DEFINE_PROP_LINK("cpm", IngenicA1HdmiState, cpm, TYPE_INGENIC_A1_CPM,
                     IngenicA1CpmState *),
    DEFINE_PROP_STRING("edid-file", IngenicA1HdmiState, edid_file),
    DEFINE_PROP_UINT32("xres", IngenicA1HdmiState, xres, 1920),
    DEFINE_PROP_UINT32("yres", IngenicA1HdmiState, yres, 1080),
};

static void hdmi_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = hdmi_realize;
    dc->vmsd = &vmstate_ingenic_a1_hdmi;
    device_class_set_props(dc, hdmi_props);
    rc->phases.hold = hdmi_reset_hold;
}

static const TypeInfo hdmi_info = {
    .name = TYPE_INGENIC_A1_HDMI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1HdmiState),
    .instance_init = hdmi_init,
    .class_init = hdmi_class_init,
};

static void hdmi_register_types(void)
{
    type_register_static(&hdmi_info);
}

type_init(hdmi_register_types)
