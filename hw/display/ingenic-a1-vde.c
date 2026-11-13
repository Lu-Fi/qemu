/*
 * Ingenic A1 VDE: the video display engine (mixer and output timing)
 *
 * The 512 KiB at 0x13300000: TOP (0x0), DMA (0x10000), MIXER (0x20000),
 * DSC (0x30000, the output timing) and INTP (0x60000, interrupts). The
 * HDMI controller at VDE + 0x80000 is ingenic-a1-hdmi. Registers keep what
 * they are given except:
 *
 *  - TOP_SHADOW_SET0/1 (0x10, 0x14): a bit the driver sets asks for a latch
 *    and stays set (a write only adds bits) until a frame's end applies it
 *    and clears it, or TOP_MODULE_EN's bit 0 goes from 0 to 1.
 *  - TOP_RST (0x30) bit 0 clears DMA_STOP_ON (0x10000) and the output
 *    enable (0x30000), and reads 0; DMA_STOP_STATE (0x1000c) reads 1, as an
 *    A1N's does.
 *  - INTP 0x60020 + 0x10n (n = 0..4) enable, +4 clear (write one to clear
 *    the status), +8 the status; the interrupt is any status & enable. A
 *    status bit sets only where its enable is set at the event.
 *
 * Frames run while TOP_MODULE_EN (0x40) bit 0 is set, a DSC output is
 * enabled (0x30000 bits 0, 4, 8) and DSC_SIZE (0x30010) is set, at the
 * rate of the DSC_TOTAL (0x30020) timing's standard mode, or the
 * refresh-hz property. Each frame's end sets INTP0 bits 0 (mixer done),
 * 1 to 3 and 24 to 27 and 29 to 31 (an A1N raises them all while frames run,
 * their phase in the frame is not known), and bit 28 half a frame in.
 *
 * What the console shows, composed when the display asks: the background
 * (0x20008, YUV), the V0 layer (TOP_LAYER_EN 0x44 bit 0; 36 windows from
 * 0xf0, 0x20 apart), V1 (bit 4; 2 windows from 0x590), G (bit 8, the
 * framebuffer: format 0x624, address 0x63c, stride 0x640) and M (bit 12,
 * the cursor: 0x5e0 format, 0x5e4 address, 0x5e8 stride, 0x5ec position,
 * 0x5f0 size), in that order. A window is NV12, NV21, NV16 or NV61 (0x80 and
 * 0x580 bits 2:0, 0, 4, 1, 5; V0 keeps no more of the low byte, so SDK's
 * 0x20 and 0x24 read back as 0 and 4) at the screen position and size
 * its registers give; YUV goes to RGB by the mixer's matrix (0x202a0 on)
 * and offsets (0x202b4). G and M are one of 24 ARGB orders, over what is
 * below by the global alpha (0x202c0 bits 15:8 for G, 0x202c8 7:0 for M; 0
 * is opaque), the pixel's alpha (one alpha bit picks alpha0 or alpha1,
 * 0x202bc) or both, as bits 17:16 of 0x202d4 (0x202e0) say (0, 1, 2), and
 * hidden where the colour key (0x202c0 to 0x202d8) matches.
 *
 * With CPM CLKGR0 bit 28 (gate_vde) set the whole window, DSC and the mixer
 * included, reads 0 and takes no write, as an A1N's does. Open, it reads
 * the version 0x20210731 (+0x0) and 0x11111111 (+0x4), and every register
 * resets to the value, and keeps the bits, the vde_regs table gives (all
 * measured on an A1N). The words of the top block from 0x60 to 0x700, the
 * DMA block's to 0x10300, the mixer's to 0x20400, the DSC's to 0x30140 and
 * INTP's to 0x60080 that the table does not list read 0 and keep nothing; a
 * window's words +0x18 and +0x1c, and the stride words +0xc and +0x14 beyond
 * 16 bits, likewise. TOP_LAYER_EN keeps 0x11111, and reads 0 and keeps
 * nothing until TOP_MODULE_EN's bit 0 is set. +0x800 to +0x84c mirror the
 * active configuration and are read-only.
 *
 * Not modelled: scaling and crop (0x20000 zoom-in and scaler), the BCSH
 * (brightness, contrast, saturation, hue) and sharpening stages,
 * the colour key's other modes, DMA descriptors, the VGA and TFT pins.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "hw/display/ingenic-a1-vde.h"
#include "hw/misc/ingenic-a1-cpm.h"
#include "trace.h"

#define V_VERSION       0x000
#define V_SHADOW0       0x010
#define V_SHADOW1       0x014
#define V_RST           0x030
#define V_MODULE_EN     0x040
#define V_LAYER_EN      0x044
#define V_GATE_REG      0x30    /* CPM CLKGR0 */
#define V_GATE_VDE      (1u << 28)
#define V_V0_FMT        0x080
#define V_MIRROR        0x800
#define V_MIRROR_END    0x850
#define V_V1_FMT        0x580
#define V_G_FMT         0x624
#define V_G_ADDR        0x63c
#define V_G_STRIDE      0x640
#define V_M_FMT         0x5e0
#define V_STOP_ON       0x10000
#define V_STOP_STATE    0x1000c
#define V_BG0           0x20008
#define V_CSC           0x202a0
#define V_CSC_OFFSET    0x202b4
#define V_G_PARA0       0x202c0
#define V_G_PARA3       0x202d4
#define V_DSC_ENABLE    0x30000
#define V_DSC_SIZE      0x30010
#define V_DSC_TOTAL     0x30020
#define V_INTP_CTRL     0x60000
#define V_INTP_EN(n)    (0x60020 + 0x10 * (n))

#define LAYER_V0        (1 << 0)
#define LAYER_V1        (1 << 4)
#define LAYER_G         (1 << 8)
#define LAYER_M         (1 << 12)

#define INT0_MIXER_DONE (1 << 0)
#define INT0_DSC_DONE   (1 << 1)
#define INT0_HDMI_START (1 << 3)
#define INT0_HALF       (1 << 28)
/* The rest an A1N's frames raise: bit 2 and bits 24 to 31 but 28 */
#define INT0_OTHERS     (INT0_MIXER_DONE << 2 | 0xef000000)

/* The standard modes' totals: htotal, vtotal, Hz */
static const struct {
    uint16_t h, v, hz;
} vde_modes[] = {
    { 2200, 1125, 60 }, { 2750, 1125, 24 }, { 2640, 1125, 50 },
    { 1650, 750, 60 }, { 1980, 750, 50 }, { 858, 525, 60 },
    { 864, 625, 50 }, { 800, 525, 60 }, { 1344, 806, 60 },
    { 1056, 628, 60 }, { 1688, 1066, 60 }, { 1560, 805, 60 },
    { 1904, 932, 60 }, { 1792, 795, 60 }, { 1440, 933, 60 },
};

static const char vde_argb8888[12][5] = {
    "ARGB", "ARBG", "AGBR", "AGRB", "ABGR", "ABRG",
    "RGBA", "RBGA", "GBRA", "GRBA", "BGRA", "BRGA",
};

/*
 * The registers that reset to something and the bits they keep; a mask of 0
 * is a register no write changes. Not listed: reset 0, all 32 bits kept.
 */
static const struct {
    uint32_t off, val, keep;
} vde_regs[] = {
    { 0x00000, 0x20210731, 0x0 }, { 0x00004, 0x11111111, 0x0 },
    { 0x00040, 0x0, 0x1 }, { 0x00044, 0x0, 0x00011111 },
    { 0x00050, 0x04380780, ~0u }, { 0x00054, 0x04380780, ~0u },
    { 0x00058, 0x04380780, ~0u }, { 0x0005c, 0x00200020, ~0u },
    { 0x00080, 0x00200001, 0xffff0007 }, { 0x00084, 0x07800000, ~0u },
    { 0x00088, 0x04380000, ~0u }, { 0x00800, 0x07800000, 0x0 },
    { 0x00804, 0x04380000, 0x0 }, { 0x0080c, 0x20, 0x0 },
    { 0x00818, 0x08000004, 0x0 }, { 0x00820, 0x07800000, 0x0 },
    { 0x00824, 0x04380000, 0x0 }, { 0x0082c, 0x20, 0x0 },
    { 0x00838, 0x08000004, 0x0 }, { 0x00844, 0x02001001, 0x0 },
    { 0x00848, 0x100, 0x0 }, { 0x0084c, 0x01000000, 0x0 },
    { 0x10000, 0x0, ~0u }, { 0x10004, 0x2, 0x3 }, { 0x10008, 0x1, ~0u },
    { 0x1000c, 0x1, 0x0 },
    { 0x10098, 0xf1, 0xf1 }, { 0x10100, 0x00ff0300, 0x00ff0311 },
    { 0x10104, 0x0, 0x33333333 }, { 0x10108, 0x0, 0x800000ff },
    { 0x10120, 0x03000000, 0xf3ff0307 }, { 0x10124, 0x0, 0x33333333 },
    { 0x10140, 0x0, 0xffff3ff1 }, { 0x10200, 0x1, 0x1 },
    { 0x10210, 0x01900100, ~0u }, { 0x10214, 0x320, 0xffff },
    { 0x10218, 0x01900100, ~0u }, { 0x1021c, 0x320, 0xffff },
    { 0x10220, 0x01900100, ~0u }, { 0x10224, 0x320, 0xffff },
    { 0x10228, 0x01000080, ~0u }, { 0x1022c, 0x1f4, 0xffff },
    { 0x10240, 0x3333, 0x7777 }, { 0x10280, 0x3ff, 0x3ff },
    { 0x20000, 0x0, 0x1 }, { 0x20004, 0x14015000, 0x17055557 },
    { 0x20008, 0x00808000, ~0u }, { 0x2000c, 0x01e00280, ~0u },
    { 0x20010, 0x0, ~0u }, { 0x20014, 0x01e00280, ~0u },
    { 0x20018, 0x40004000, ~0u }, { 0x2001c, 0x0, 0x01001111 },
    { 0x20020, 0x0, 0x03ff03ff }, { 0x20024, 0x8080, 0xf1ffffff },
    { 0x20028, 0x0, 0x1 }, { 0x20040, 0x80fffff9, 0x0 },
    { 0x20044, 0x83fefdd9, 0x0 }, { 0x20048, 0x835fdf4b, 0x0 },
    { 0x2004c, 0x8378fe5a, 0x0 }, { 0x20050, 0x0, 0x31 },
    { 0x20090, 0x0, ~0u }, { 0x20094, 0x0, ~0u }, { 0x20098, 0x0, ~0u },
    { 0x2009c, 0x0, ~0u }, { 0x200a0, 0x0, ~0u }, { 0x200a4, 0x0, ~0u },
    { 0x200a8, 0x0, ~0u }, { 0x200ac, 0x0, ~0u }, { 0x200b0, 0x0, ~0u },
    { 0x200b4, 0x0, ~0u }, { 0x200b8, 0x0, ~0u }, { 0x200bc, 0x0, ~0u },
    { 0x200c0, 0x0, ~0u }, { 0x200c4, 0x0, ~0u }, { 0x200c8, 0x0, ~0u },
    { 0x200cc, 0x0, ~0u }, { 0x200d0, 0x0, ~0u }, { 0x200d4, 0x0, ~0u },
    { 0x200d8, 0x0408175e, 0x1f7df7df }, { 0x200dc, 0x765a3300, ~0u },
    { 0x200e0, 0xbaaf9f8c, 0xeffffd7f }, { 0x200e4, 0xe1d8cec4, ~0u },
    { 0x200e8, 0xfbf6f0e9, ~0u }, { 0x200ec, 0xf8ffffff, ~0u },
    { 0x200f0, 0x6ea0cee8, ~0u }, { 0x200f4, 0x1220364e, ~0u },
    { 0x200f8, 0x207, ~0u }, { 0x200fc, 0x010000ff, 0x01ffffff },
    { 0x20100, 0x01000000, ~0u }, { 0x20104, 0x170c0602, ~0u },
    { 0x20108, 0x51413224, ~0u }, { 0x2010c, 0x92837362, ~0u },
    { 0x20110, 0xd7cbba9e, ~0u }, { 0x20114, 0xf2ede7e0, ~0u },
    { 0x20118, 0xfefcf8f6, ~0u }, { 0x20120, 0xf9fdffff, ~0u },
    { 0x20124, 0xaec6dcee, ~0u }, { 0x20128, 0x415c7595, ~0u },
    { 0x2012c, 0x00061528, ~0u }, { 0x20130, 0xff, 0xffff },
    { 0x20144, 0x0, 0x1 }, { 0x20148, 0x0, ~0u }, { 0x2014c, 0x0, 0x1 },
    { 0x20150, 0xffff0000, 0x0 }, { 0x20154, 0xf3ec0000, 0x0 },
    { 0x20160, 0x0, 0x1 }, { 0x20164, 0x0, 0x1 }, { 0x20168, 0x0, 0x1ff },
    { 0x2016c, 0x0, 0x1 }, { 0x20170, 0x009c8500, 0x0 },
    { 0x20174, 0x000416c8, 0x0 }, { 0x20178, 0x0, ~0u },
    { 0x2017c, 0x0, ~0u }, { 0x20180, 0x0, ~0u }, { 0x20184, 0x0, ~0u },
    { 0x20188, 0x0, ~0u }, { 0x2018c, 0x0, ~0u }, { 0x20190, 0x0, ~0u },
    { 0x20194, 0x0, ~0u }, { 0x20198, 0x0, ~0u }, { 0x2019c, 0x0, ~0u },
    { 0x201a0, 0x0, 0xffffffef }, { 0x201a4, 0x0, ~0u },
    { 0x201a8, 0x0, ~0u }, { 0x201ac, 0x0, ~0u }, { 0x201b0, 0x0, ~0u },
    { 0x201b4, 0x0, ~0u }, { 0x201b8, 0x0, ~0u }, { 0x201bc, 0x0, ~0u },
    { 0x20200, 0x3ff, 0x03ff03ff }, { 0x20204, 0x3ff, 0x03ff03ff },
    { 0x20208, 0x3ff, 0x03ff03ff }, { 0x2020c, 0x3ff, 0x03ff03ff },
    { 0x20210, 0x3ff, 0x03ff03ff }, { 0x20214, 0x3ff, 0x03ff03ff },
    { 0x20218, 0x04000400, 0x07ff07ff },
    { 0x2021c, 0x04000400, 0x07ff07ff },
    { 0x20220, 0x04000400, 0x07ff07ff }, { 0x20224, 0x400, 0x3fff3fff },
    { 0x20228, 0x0, 0x3fff3fff }, { 0x2022c, 0x400, 0x3fff3fff },
    { 0x20230, 0x0, 0x3fff3fff }, { 0x20234, 0x400, 0x3fff },
    { 0x20254, 0x00010000, 0x7fff0001 },
    { 0x20258, 0x00010400, 0x7fff7fff },
    { 0x2025c, 0x03ff0000, 0x03ff03ff },
    { 0x20260, 0x03ff0000, 0x03ff03ff }, { 0x20264, 0x0, 0x1fff0001 },
    { 0x20268, 0x00210001, 0x03ff03ff },
    { 0x2026c, 0x04000400, 0x1fff1fff }, { 0x20270, 0x00808000, ~0u },
    { 0x202a0, 0x400, 0x3fff3fff }, { 0x202a4, 0x0400064d, 0x3fff3fff },
    { 0x202a8, 0x3e213f40, 0x3fff3fff },
    { 0x202ac, 0x076c0400, 0x3fff3fff }, { 0x202b0, 0x0, 0x3fff },
    { 0x202b4, 0x8000, ~0u }, { 0x202b8, 0xff00ff00, ~0u },
    { 0x202bc, 0xff00ff00, ~0u }, { 0x202c0, 0x0, ~0u },
    { 0x202c4, 0x0, ~0u }, { 0x202c8, 0x0, 0xffff00ff },
    { 0x202cc, 0x0, ~0u }, { 0x202d0, 0x00ff00ff, 0x11ff11ff },
    { 0x202d4, 0xff, 0x000311ff }, { 0x202d8, 0xff, ~0u },
    { 0x202dc, 0x00ff00ff, ~0u }, { 0x202e0, 0xff, ~0u },
    { 0x202e4, 0xff, ~0u }, { 0x20300, 0x01e00280, ~0u },
    { 0x20304, 0x40004000, ~0u }, { 0x20308, 0x0, 0x01001111 },
    { 0x2030c, 0x0, 0x03ff03ff }, { 0x20310, 0x8080, ~0u },
    { 0x20314, 0x0, 0x1 }, { 0x20330, 0x8123d9b5, 0x0 },
    { 0x20334, 0x819dfa2b, 0x0 }, { 0x20338, 0x83de0a56, 0x0 },
    { 0x2033c, 0x812fec4f, 0x0 }, { 0x20380, 0x0, 0x33 },
    { 0x30000, 0x0, 0x00011111 }, { 0x30010, 0x04380780, ~0u },
    { 0x30020, 0x04650898, ~0u }, { 0x30024, 0x002d0001, ~0u },
    { 0x30028, 0x00060001, ~0u }, { 0x3002c, 0x002d0001, ~0u },
    { 0x30030, 0x00060001, ~0u }, { 0x30034, 0x084100c1, ~0u },
    { 0x30038, 0x0462002a, ~0u }, { 0x3003c, 0x0, 0x00111111 },
    { 0x30040, 0x76c, 0xfff }, { 0x30044, 0x3025, 0x71ff },
    { 0x30050, 0x02dc00da, 0x3fff3fff },
    { 0x30054, 0x3f8b004a, 0x3fff3fff },
    { 0x30058, 0x02003e75, 0x3fff3fff },
    { 0x3005c, 0x3e2f0200, 0x3fff3fff }, { 0x30060, 0x3fd1, 0x3fff },
    { 0x30064, 0x80000000, ~0u }, { 0x30068, 0xff00ff00, ~0u },
    { 0x30080, 0x0, 0xff }, { 0x30084, 0x0, 0x3 }, { 0x30090, 0x1, 0x1 },
    { 0x300a0, 0x0, 0x111 }, { 0x300a4, 0x210, 0x333 },
    { 0x300ac, 0x210, 0x333 }, { 0x300b0, 0x0, 0x00ffffff },
    { 0x300b4, 0x0, 0x7 }, { 0x30100, 0x05dc0500, 0x0fff0fff },
    { 0x30104, 0x708, 0xfff }, { 0x30110, 0x03c00010, 0x0fff0011 },
    { 0x30120, 0x0, ~0u }, { 0x30124, 0x0, ~0u }, { 0x30128, 0x0, ~0u },
    { 0x3012c, 0x0, ~0u }, { 0x60000, 0x0, 0x1f }, { 0x60020, 0x0, ~0u },
    { 0x6002c, 0x0, ~0u }, { 0x60030, 0x0, ~0u }, { 0x6003c, 0x0, ~0u },
    { 0x60040, 0x0, ~0u }, { 0x6004c, 0x0, ~0u }, { 0x60050, 0x0, ~0u },
    { 0x6005c, 0x0, ~0u }, { 0x60060, 0x0, ~0u }, { 0x6006c, 0x0, ~0u },
};

/* The bits of a window's words (0x20 apart) that keep a write */
static uint32_t vde_win_keep(uint32_t off)
{
    switch (off / 4 % 8) {
    case 3:
    case 5:
        return 0xffff;
    case 6:
    case 7:
        return 0;
    default:
        return ~0u;
    }
}

/* What the top block's words from 0x60 on (windows, V1, M and G) keep */
static uint32_t vde_top_keep(uint32_t off)
{
    if (off < 0x80 || (off >= 0x8c && off < 0xf0)) {
        return 0;
    } else if (off >= 0xf0 && off < 0x570) {
        return vde_win_keep(off - 0xf0);
    } else if (off == 0x580) {
        return 7;
    } else if (off >= 0x590 && off < 0x5d0) {
        return vde_win_keep(off - 0x590);
    } else if (off == 0x5e0) {
        return 0x111f;
    } else if (off >= 0x5e4 && off < 0x5f4) {
        return off == 0x5e8 ? 0xffff : ~0u;
    } else if (off == 0x624) {
        return 0x1f;
    } else if (off >= 0x62c && off < 0x644) {
        return off == 0x638 ? 0xfdbcffff : ~0u;
    } else if (off >= 0x570 && off < 0x700) {
        return 0;
    }
    return ~0u;
}

/*
 * The bits a write to off keeps. Where an A1N was swept, a word no table
 * entry lists reads 0 and keeps nothing; elsewhere it keeps all 32 bits
 */
static uint32_t vde_keep(uint32_t off)
{
    for (int i = 0; i < ARRAY_SIZE(vde_regs); i++) {
        if (vde_regs[i].off == off) {
            return vde_regs[i].keep;
        }
    }
    if (off >= 0x60 && off < 0x700) {
        return vde_top_keep(off);
    }
    if ((off >= 0x10000 && off < 0x10300) ||
        (off >= 0x20000 && off < 0x20400) ||
        (off >= 0x30000 && off < 0x30140) ||
        (off >= 0x60000 && off < 0x60080)) {
        return 0;
    }
    return ~0u;
}

static bool vde_gated(IngenicA1VdeState *s)
{
    return s->cpm && (s->cpm->regs[V_GATE_REG / 4] & V_GATE_VDE);
}

static uint32_t vde_reg(IngenicA1VdeState *s, uint32_t off)
{
    return s->regs[off / 4];
}

static void vde_update_irq(IngenicA1VdeState *s)
{
    bool on = false;

    for (int n = 0; n < INGENIC_A1_VDE_INTP; n++) {
        on |= !!(s->intp[n] & vde_reg(s, V_INTP_EN(n)));
    }
    qemu_set_irq(s->irq, on);
}

static bool vde_running(IngenicA1VdeState *s)
{
    return (vde_reg(s, V_MODULE_EN) & 1) &&
           (vde_reg(s, V_DSC_ENABLE) & 0x111) && vde_reg(s, V_DSC_SIZE);
}

static int64_t vde_frame_ns(IngenicA1VdeState *s)
{
    uint32_t total = vde_reg(s, V_DSC_TOTAL);
    uint32_t hz = s->refresh_hz ? s->refresh_hz : 60;

    if (!s->refresh_hz) {
        for (int i = 0; i < ARRAY_SIZE(vde_modes); i++) {
            if (vde_modes[i].h == (total & 0xffff) &&
                vde_modes[i].v == total >> 16) {
                hz = vde_modes[i].hz;
                break;
            }
        }
    }
    return NANOSECONDS_PER_SECOND / hz;
}

static void vde_arm(IngenicA1VdeState *s, int64_t now)
{
    s->frame_ns = vde_frame_ns(s);
    timer_mod(s->frame, now + s->frame_ns);
    timer_mod(s->half, now + s->frame_ns / 2);
}

/* A status bit sets only where its enable is set when the event comes */
static void vde_raise(IngenicA1VdeState *s, uint32_t bits)
{
    s->intp[0] |= bits & vde_reg(s, V_INTP_EN(0));
    vde_update_irq(s);
}

static void vde_frame_end(void *opaque)
{
    IngenicA1VdeState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    s->regs[V_SHADOW0 / 4] = 0;
    s->regs[V_SHADOW1 / 4] = 0;
    vde_raise(s, INT0_MIXER_DONE | INT0_DSC_DONE | INT0_HDMI_START |
              INT0_OTHERS);
    if (vde_running(s)) {
        vde_arm(s, now);
    }
}

static void vde_half(void *opaque)
{
    IngenicA1VdeState *s = opaque;

    vde_raise(s, INT0_HALF);
}

/* Start or stop the frames after a register changed */
static void vde_sched(IngenicA1VdeState *s)
{
    if (vde_running(s)) {
        if (!timer_pending(s->frame)) {
            vde_arm(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        }
    } else {
        timer_del(s->frame);
        timer_del(s->half);
    }
}

static uint64_t vde_read4(void *opaque, hwaddr off)
{
    IngenicA1VdeState *s = opaque;

    if (vde_gated(s) || off == V_RST) {
        return 0;
    }
    if (off == V_STOP_STATE) {
        return 1;
    }
    if (off >= 0x60028 && off <= 0x60068 && (off & 0xf) == 8) {
        return s->intp[(off - 0x60028) / 0x10];
    }
    return s->regs[off / 4];
}

static void vde_write4(void *opaque, hwaddr off, uint64_t v)
{
    IngenicA1VdeState *s = opaque;

    trace_ingenic_a1_vde_write(off, v);
    if (vde_gated(s)) {
        return;
    }
    if (off == V_RST) {
        if (v & 1) {
            s->regs[V_STOP_ON / 4] = 0;
            s->regs[V_DSC_ENABLE / 4] = 0;
            vde_sched(s);
        }
        return;
    }
    if (off >= 0x60024 && off <= 0x60064 && (off & 0xf) == 4) {
        s->intp[(off - 0x60024) / 0x10] &= ~v;
        vde_update_irq(s);
        return;
    }
    if (off == V_VERSION || off == V_STOP_STATE ||
        (off >= 0x60028 && (off & 0xf) == 8 && off <= 0x60068) ||
        (off >= V_MIRROR && off < V_MIRROR_END)) {
        return;
    }
    if (off == V_LAYER_EN && !(vde_reg(s, V_MODULE_EN) & 1)) {
        return;
    }
    v &= vde_keep(off);
    if (!vde_keep(off)) {
        return;
    }
    if (off == V_SHADOW0 || off == V_SHADOW1) {
        /* Requests only add; a frame's end or the module's start clears */
        s->regs[off / 4] |= v;
        return;
    }
    if (off == V_MODULE_EN && (v & 1) && !(vde_reg(s, V_MODULE_EN) & 1)) {
        s->regs[V_SHADOW0 / 4] = 0;
        s->regs[V_SHADOW1 / 4] = 0;
    }
    s->regs[off / 4] = v;
    if (off == V_MODULE_EN || off == V_DSC_ENABLE || off == V_DSC_SIZE ||
        off == V_DSC_TOTAL) {
        vde_sched(s);
    } else if (off >= 0x60020 && off < 0x60070) {
        vde_update_irq(s);
    }
}

/* The driver stores words at odd addresses, which the CPU splits */
static uint64_t vde_read(void *opaque, hwaddr off, unsigned size)
{
    return extract32(vde_read4(opaque, off & ~3), (off & 3) * 8, size * 8);
}

static void vde_write(void *opaque, hwaddr off, uint64_t v, unsigned size)
{
    if (size < 4) {
        v = deposit32(vde_read4(opaque, off & ~3), (off & 3) * 8, size * 8, v);
    }
    vde_write4(opaque, off & ~3, v);
}

static const MemoryRegionOps vde_ops = {
    .read = vde_read,
    .write = vde_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* ---- composing ---- */

typedef struct VdeCsc {
    int32_t m[9];
    int yoff, coff;
} VdeCsc;

static void vde_load_csc(IngenicA1VdeState *s, VdeCsc *c)
{
    for (int i = 0; i < 9; i++) {
        uint32_t r = vde_reg(s, V_CSC + (i / 2) * 4);

        c->m[i] = sextract32(i & 1 ? r >> 16 : r, 0, 14);
    }
    c->yoff = vde_reg(s, V_CSC_OFFSET) & 0xff;
    c->coff = vde_reg(s, V_CSC_OFFSET) >> 8 & 0xff;
}

static uint32_t vde_yuv(const VdeCsc *c, int y, int u, int v)
{
    int32_t yy = y - c->yoff, uu = u - c->coff, vv = v - c->coff;
    int32_t rgb[3];

    for (int k = 0; k < 3; k++) {
        rgb[k] = (c->m[k * 3] * yy + c->m[k * 3 + 1] * uu +
                  c->m[k * 3 + 2] * vv) >> 10;
        rgb[k] = MIN(MAX(rgb[k], 0), 255);
    }
    return rgb[0] << 16 | rgb[1] << 8 | rgb[2];
}

static void vde_video_window(IngenicA1VdeState *s, const VdeCsc *c,
                             uint32_t fmt, uint32_t start, uint32_t size,
                             uint32_t ya, uint32_t ys, uint32_t uva,
                             uint32_t uvs, uint32_t *dst, int w, int h,
                             int stride)
{
    int x0 = start & 0xffff, y0 = start >> 16;
    int ww = size & 0xffff, wh = size >> 16;
    /* NV12 0, NV16 1, NV21 4, NV61 5: bit 2 swaps U and V, bit 0 is 4:2:2 */
    bool swap = fmt & 4;
    int vshift = fmt & 1 ? 0 : 1;
    g_autofree uint8_t *yl = NULL, *uvl = NULL;

    if (!ww || !wh || !ya || !uva || x0 >= w || y0 >= h) {
        return;
    }
    ww = MIN(ww, w - x0);
    wh = MIN(wh, h - y0);
    yl = g_malloc(ww + 2);
    uvl = g_malloc(ww + 2);
    for (int j = 0; j < wh; j++) {
        uint32_t *d = dst + (y0 + j) * stride + x0;

        dma_memory_read(&address_space_memory, (ya & 0x1fffffff) +
                        (hwaddr)j * ys, yl, ww, MEMTXATTRS_UNSPECIFIED);
        dma_memory_read(&address_space_memory, (uva & 0x1fffffff) +
                        (hwaddr)(j >> vshift) * uvs, uvl, ww,
                        MEMTXATTRS_UNSPECIFIED);
        for (int i = 0; i < ww; i++) {
            int u = uvl[i & ~1], v = uvl[i | 1];

            d[i] = vde_yuv(c, yl[i], swap ? v : u, swap ? u : v);
        }
    }
}

/* One pixel of a G or M format: RGB, and its alpha (255 with none) */
static uint32_t vde_pixel(uint32_t fmt, const uint8_t *p, unsigned *alpha)
{
    static const uint8_t shifts[3][4] = {
        { 24, 16, 8, 0 }, { 15, 10, 5, 0 }, { 11, 6, 1, 0 },
    };
    const char *names = vde_argb8888[fmt % 12];
    unsigned kind = fmt < 12 ? 0 : fmt < 18 ? 1 : 2;
    unsigned bits = kind ? 5 : 8, abits = kind ? 1 : 8;
    uint32_t val = kind ? lduw_le_p(p) : ldl_le_p(p), rgb = 0;

    *alpha = 255;
    for (int k = 0; k < 4; k++) {
        unsigned nb = names[k] == 'A' ? abits : bits;
        uint32_t f = extract32(val, shifts[kind][k], nb);

        if (nb == 5) {
            f = f << 3 | f >> 2;
        } else if (nb == 1) {
            f = f ? 255 : 0;
        }
        switch (names[k]) {
        case 'A':
            *alpha = f;
            break;
        case 'R':
            rgb |= f << 16;
            break;
        case 'G':
            rgb |= f << 8;
            break;
        default:
            rgb |= f;
        }
    }
    return rgb;
}

/* The alpha and colour key registers of G (0) and M (1) */
typedef struct VdeKey {
    uint32_t key_rb, key_g, mask_rg, mask_b, mode;
    uint8_t galpha, a0, a1;
} VdeKey;

static void vde_load_key(IngenicA1VdeState *s, int layer, VdeKey *k)
{
    uint32_t base = layer ? 0x202c8 : V_G_PARA0;
    uint32_t alpha = vde_reg(s, 0x202bc);

    k->key_rb = vde_reg(s, base);
    k->key_g = vde_reg(s, base + 4);
    k->mask_rg = vde_reg(s, layer ? 0x202dc : 0x202d0);
    k->mask_b = vde_reg(s, layer ? 0x202e0 : V_G_PARA3);
    k->mode = k->mask_b >> 16 & 3;
    k->galpha = layer ? k->key_rb & 0xff : k->key_rb >> 8 & 0xff;
    k->a0 = alpha >> (layer ? 16 : 0);
    k->a1 = alpha >> (layer ? 24 : 8);
}

/*
 * A pixel the colour key hides: each channel, masked, inside its range. An
 * A1N resets the key's ranges to 0, which this takes for a key not set:
 * where its enable is, if not in the masks' spare bits, is not known
 */
static bool vde_keyed(const VdeKey *k, uint32_t rgb)
{
    uint32_t r = rgb >> 16 & 0xff, g = rgb >> 8 & 0xff, b = rgb & 0xff;
    uint32_t mr = k->mask_rg >> 16 & 0xff, mg = k->mask_rg & 0xff;
    uint32_t mb = k->mask_b & 0xff;
    uint32_t rlo = k->key_rb >> 16 & 0xff, rhi = k->key_rb >> 24;
    uint32_t glo = k->key_g & 0xff, ghi = k->key_g >> 8 & 0xff;
    uint32_t blo = k->key_g >> 16 & 0xff, bhi = k->key_g >> 24;

    /* Both ranges 0, as they reset: no key was set, black stays */
    if (!k->key_rb && !k->key_g) {
        return false;
    }
    return (r & mr) >= MIN(rlo, rhi) && (r & mr) <= MAX(rlo, rhi) &&
           (g & mg) >= MIN(glo, ghi) && (g & mg) <= MAX(glo, ghi) &&
           (b & mb) >= MIN(blo, bhi) && (b & mb) <= MAX(blo, bhi);
}

static void vde_blend(IngenicA1VdeState *s, int layer, uint32_t fmt,
                      uint32_t addr, uint32_t stride, int x0, int y0,
                      int ww, int wh, uint32_t *dst, int w, int h,
                      int dstride)
{
    unsigned pix = fmt < 12 ? 4 : 2;
    g_autofree uint8_t *line = NULL;
    VdeKey k;

    if (!addr || fmt >= 24 || x0 >= w || y0 >= h) {
        return;
    }
    vde_load_key(s, layer, &k);
    ww = MIN(ww, w - x0);
    wh = MIN(wh, h - y0);
    line = g_malloc((size_t)ww * pix);
    for (int j = 0; j < wh; j++) {
        uint32_t *d = dst + (y0 + j) * dstride + x0;

        dma_memory_read(&address_space_memory, (addr & 0x1fffffff) +
                        (hwaddr)j * stride, line, (size_t)ww * pix,
                        MEMTXATTRS_UNSPECIFIED);
        for (int i = 0; i < ww; i++) {
            unsigned a, pa;
            uint32_t p = vde_pixel(fmt, line + i * pix, &pa);
            unsigned g = k.galpha ? k.galpha : 255;

            if (vde_keyed(&k, p)) {
                continue;
            }
            /* One alpha bit picks alpha0 or alpha1 */
            if (fmt >= 12) {
                pa = pa ? k.a1 : k.a0;
            }
            a = k.mode == 0 ? g : k.mode == 1 ? pa : pa * g / 255;
            if (a == 255) {
                d[i] = p;
            } else if (a) {
                uint32_t o = d[i], r = 0;

                for (int sh = 0; sh < 24; sh += 8) {
                    r |= (((p >> sh & 255) * a +
                           (o >> sh & 255) * (255 - a)) / 255) << sh;
                }
                d[i] = r;
            }
        }
    }
}

static bool vde_gfx_update(void *opaque)
{
    IngenicA1VdeState *s = opaque;
    uint32_t size = vde_reg(s, V_DSC_SIZE);
    int w = size & 0xffff, h = size >> 16;
    uint32_t layers = vde_reg(s, V_LAYER_EN);
    DisplaySurface *ds;
    uint32_t *out;
    VdeCsc csc;
    uint32_t bg;

    if (!w || !h || !(vde_reg(s, V_MODULE_EN) & 1) ||
        !(vde_reg(s, V_DSC_ENABLE) & 0x111)) {
        return true;
    }
    ds = qemu_console_surface(s->con);
    if (!ds || surface_width(ds) != w || surface_height(ds) != h) {
        qemu_console_resize(s->con, w, h);
        ds = qemu_console_surface(s->con);
    }
    if (surface_bits_per_pixel(ds) != 32) {
        return true;
    }
    out = surface_data(ds);
    vde_load_csc(s, &csc);
    bg = vde_reg(s, V_BG0);
    bg = vde_yuv(&csc, bg & 0xff, bg >> 8 & 0xff, bg >> 16 & 0xff);
    for (int i = 0; i < w * h; i++) {
        out[i] = bg;
    }

    if (layers & LAYER_V0) {
        for (int k = 0; k < 36; k++) {
            uint32_t b = 0xf0 + k * 0x20;

            vde_video_window(s, &csc, vde_reg(s, V_V0_FMT), vde_reg(s, b),
                             vde_reg(s, b + 4), vde_reg(s, b + 8),
                             vde_reg(s, b + 0xc), vde_reg(s, b + 0x10),
                             vde_reg(s, b + 0x14), out, w, h,
                             surface_stride(ds) / 4);
        }
    }
    if (layers & LAYER_V1) {
        for (int k = 0; k < 2; k++) {
            uint32_t b = 0x590 + k * 0x20;

            vde_video_window(s, &csc, vde_reg(s, V_V1_FMT), vde_reg(s, b + 4),
                             vde_reg(s, b), vde_reg(s, b + 8),
                             vde_reg(s, b + 0xc), vde_reg(s, b + 0x10),
                             vde_reg(s, b + 0x14), out, w, h,
                             surface_stride(ds) / 4);
        }
    }
    if (layers & LAYER_G) {
        vde_blend(s, 0, vde_reg(s, V_G_FMT), vde_reg(s, V_G_ADDR),
                  vde_reg(s, V_G_STRIDE), 0, 0, w, h, out, w, h,
                  surface_stride(ds) / 4);
    }
    if (layers & LAYER_M) {
        uint32_t pos = vde_reg(s, 0x5ec), msize = vde_reg(s, 0x5f0);

        vde_blend(s, 1, vde_reg(s, V_M_FMT), vde_reg(s, 0x5e4),
                  vde_reg(s, 0x5e8), pos & 0xffff, pos >> 16,
                  msize & 0xffff, msize >> 16, out, w, h,
                  surface_stride(ds) / 4);
    }
    qemu_console_update(s->con, 0, 0, w, h);
    return true;
}

static const GraphicHwOps vde_gfx_ops = {
    .gfx_update = vde_gfx_update,
};

static void vde_reset_hold(Object *obj, ResetType type)
{
    IngenicA1VdeState *s = INGENIC_A1_VDE(obj);

    timer_del(s->frame);
    timer_del(s->half);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->intp, 0, sizeof(s->intp));
    for (int i = 0; i < ARRAY_SIZE(vde_regs); i++) {
        s->regs[vde_regs[i].off / 4] = vde_regs[i].val;
    }
    vde_update_irq(s);
}

static void vde_realize(DeviceState *dev, Error **errp)
{
    IngenicA1VdeState *s = INGENIC_A1_VDE(dev);

    s->con = qemu_graphic_console_create(dev, 0, &vde_gfx_ops, s);
    qemu_console_resize(s->con, 1280, 720);
}

static void vde_init(Object *obj)
{
    IngenicA1VdeState *s = INGENIC_A1_VDE(obj);

    memory_region_init_io(&s->iomem, obj, &vde_ops, s, "ingenic-a1-vde",
                          INGENIC_A1_VDE_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, vde_frame_end, s);
    s->half = timer_new_ns(QEMU_CLOCK_VIRTUAL, vde_half, s);
}

static const VMStateDescription vmstate_ingenic_a1_vde = {
    .name = "ingenic-a1-vde",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicA1VdeState,
                             INGENIC_A1_VDE_SIZE / 4),
        VMSTATE_UINT32_ARRAY(intp, IngenicA1VdeState, INGENIC_A1_VDE_INTP),
        VMSTATE_INT64(frame_ns, IngenicA1VdeState),
        VMSTATE_TIMER_PTR(frame, IngenicA1VdeState),
        VMSTATE_TIMER_PTR(half, IngenicA1VdeState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property vde_props[] = {
    DEFINE_PROP_UINT32("refresh-hz", IngenicA1VdeState, refresh_hz, 0),
    DEFINE_PROP_LINK("cpm", IngenicA1VdeState, cpm, TYPE_INGENIC_A1_CPM,
                     IngenicA1CpmState *),
};

static void vde_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = vde_realize;
    dc->vmsd = &vmstate_ingenic_a1_vde;
    device_class_set_props(dc, vde_props);
    rc->phases.hold = vde_reset_hold;
}

static const TypeInfo vde_info = {
    .name = TYPE_INGENIC_A1_VDE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1VdeState),
    .instance_init = vde_init,
    .class_init = vde_class_init,
};

static void vde_register_types(void)
{
    type_register_static(&vde_info);
}

type_init(vde_register_types)
