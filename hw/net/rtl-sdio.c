/*
 * Realtek RTL8189FS and RTL8189ES Wi-Fi on SDIO
 *
 * Function 1 is split by the top address bits into what the drivers call
 * devices: the SDIO local registers, the TX FIFOs of each queue, the RX
 * FIFO, and with bit 16 set the chip's WLAN register space.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "hw/sd/sdio.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "trace.h"

#define TYPE_RTL_SDIO "rtl-sdio"
OBJECT_DECLARE_TYPE(RtlSdioState, RtlSdioClass, RTL_SDIO)

#define DEV_SDIO_LOCAL          0
#define DEV_TX_EXQ              3
#define DEV_TX_HIQ              4
#define DEV_TX_MIQ              5
#define DEV_TX_LOQ              6
#define DEV_RX0FF               7
#define DEV_WLAN_IOREG          8

#define SDIO_LOCAL_SIZE         0x1000
#define WLAN_IOREG_SIZE         0x8000

/* HCI suspend: bit 0 asks, bit 1 says the resumed power is ready */
#define SDIO_REG_HSUS_CTRL      0x0086
#define HCI_SUS_REQ             BIT(0)
#define HCI_RESUME_PWR_RDY      BIT(1)

/* REG_APS_FSMCO: the MAC power on and off requests clear once done */
#define REG_APS_FSMCO_1         0x0005
#define APFM_ONMAC              BIT(0)
#define APFM_OFF                BIT(1)
#define REG_APS_FSMCO_2         0x0006
#define POWER_READY             BIT(1)

/* the byte offset, then in the top byte the flag: read ready, write busy */
#define REG_EFUSE_CTRL          0x0030
#define EFUSE_FLAG              BIT(7)
#define EFUSE_SIZE              512

/*
 * Firmware download: the checksum reports in once the download window
 * closes, and the firmware is up as soon as the driver hands over to it
 */
#define REG_MCUFWDL             0x0080
#define MCUFWDL_EN              BIT(0)
#define MCUFWDL_RDY             BIT(1)
#define FWDL_CHKSUM_RPT         BIT(2)
#define WINTINI_RDY             BIT(6)
#define RAM_DL_SEL              BIT(7)

/*
 * The firmware's report mailbox: an id, a sequence byte, then the report.
 * A driver that leaves C2H_DEFEATURE_RSVD in it asks the firmware to post
 * the MAC hidden report there once it boots: what the part is fused for.
 */
#define REG_C2HEVT_MSG_NORMAL   0x01a0
#define C2H_MAC_HIDDEN_RPT      0x19
#define C2H_DEFEATURE_RSVD      0xfd

/* the LLT builds itself on request, clearing the request once built */
#define REG_AUTO_LLT_2          0x0226
#define AUTO_INIT_LLT           BIT(0)
/* or, on the 8188E, entry by entry: the operation in [31:30] runs at once */
#define REG_LLT_INIT_3          0x01e3
#define LLT_OP_MASK             0xc0

/*
 * The 8188E's IO offload: a command bit in [3:0] runs, then clears; an
 * error would show as the same bit in [7:4]
 */
#define REG_HMEBOX_E0           0x0088
#define IOL_CMD_MASK            0x0f

typedef struct RtlSdioChip {
    const char *name;
    const char *desc;
    uint16_t device;
} RtlSdioChip;

struct RtlSdioClass {
    SDIOCardClass parent_class;

    const RtlSdioChip *chip;
};

static const RtlSdioChip rtl_sdio_chips[] = {
    { "rtl8189fs", "Realtek RTL8189FS SDIO Wi-Fi", 0xf179 },
    { "rtl8189es", "Realtek RTL8189ES SDIO Wi-Fi", 0x8179 },
};

/*
 * SDIO, every function and bandwidth (the driver keeps what the chip has),
 * one antenna, 802.11b/g/n; then a cut version of 0xf, which changes none
 */
static const uint8_t rtl_sdio_hidden_rpt[8 + 5] = {
    0, 0, 0, 0, 0x01, 0x0f, 0x02 | 1 << 5, 2 << 2,
    0x0f, 0, 0, 0, 0,
};

struct RtlSdioState {
    SDIOCard parent_obj;

    uint8_t local[SDIO_LOCAL_SIZE];
    uint8_t ioreg[WLAN_IOREG_SIZE];
    uint8_t efuse[EFUSE_SIZE];
};

/* Bit 16 picks the WLAN registers, else bits 15-13 the device */
static unsigned rtl_sdio_device(uint32_t addr)
{
    return addr & BIT(16) ? DEV_WLAN_IOREG : (addr >> 13) & 7;
}

/*
 * Clearing the flag starts a read of the byte at the offset; setting it
 * programs one, which the blank efuse here takes without changing
 */
static void rtl_sdio_efuse(RtlSdioState *s)
{
    uint8_t *r = s->ioreg + REG_EFUSE_CTRL;
    unsigned addr = (r[2] & 3) << 8 | r[1];

    if (r[3] & EFUSE_FLAG) {
        r[3] &= ~EFUSE_FLAG;
    } else {
        r[0] = addr < EFUSE_SIZE ? s->efuse[addr] : 0xff;
        r[3] |= EFUSE_FLAG;
    }
}

/* What the firmware does as it comes up */
static void rtl_sdio_fw_boot(RtlSdioState *s)
{
    uint8_t *c2h = &s->ioreg[REG_C2HEVT_MSG_NORMAL];

    if (c2h[0] == C2H_DEFEATURE_RSVD) {
        c2h[0] = C2H_MAC_HIDDEN_RPT;
        memcpy(c2h + 2, rtl_sdio_hidden_rpt, sizeof(rtl_sdio_hidden_rpt));
    }
}

/* What the MAC holds out of reset, where drivers check it */
static const struct {
    uint16_t reg;
    uint8_t size;
    uint32_t val;
} rtl_sdio_reset_regs[] = {
    /* the SIFS timings */
    { 0x0428, 2, 0x100a },
    { 0x0514, 4, 0x0e0a0e0a },
    { 0x063a, 2, 0x100a },
    { 0x063c, 4, 0x0e0e0a0a },
};

static bool rtl_sdio_covers(uint32_t off, size_t len, uint32_t reg)
{
    return off <= reg && off + len > reg;
}

/* What the chip does about a write to its WLAN registers */
static void rtl_sdio_ioreg_written(RtlSdioState *s, uint32_t off, size_t len)
{
    if (rtl_sdio_covers(off, len, REG_APS_FSMCO_1)) {
        s->ioreg[REG_APS_FSMCO_1] &= ~(APFM_ONMAC | APFM_OFF);
    }
    if (rtl_sdio_covers(off, len, REG_APS_FSMCO_2)) {
        s->ioreg[REG_APS_FSMCO_2] |= POWER_READY;
    }
    if (rtl_sdio_covers(off, len, REG_EFUSE_CTRL + 3)) {
        rtl_sdio_efuse(s);
    }
    if (rtl_sdio_covers(off, len, REG_AUTO_LLT_2)) {
        s->ioreg[REG_AUTO_LLT_2] &= ~AUTO_INIT_LLT;
    }
    if (rtl_sdio_covers(off, len, REG_LLT_INIT_3)) {
        s->ioreg[REG_LLT_INIT_3] &= ~LLT_OP_MASK;
    }
    if (rtl_sdio_covers(off, len, REG_HMEBOX_E0)) {
        s->ioreg[REG_HMEBOX_E0] &= ~IOL_CMD_MASK;
    }
    if (rtl_sdio_covers(off, len, REG_MCUFWDL)) {
        uint8_t *fwdl = &s->ioreg[REG_MCUFWDL];

        if (!(*fwdl & MCUFWDL_EN)) {
            *fwdl |= FWDL_CHKSUM_RPT;
        }
        if (*fwdl & MCUFWDL_RDY) {
            *fwdl |= WINTINI_RDY | RAM_DL_SEL;
            rtl_sdio_fw_boot(s);
        }
    }
}

/* ... and to its SDIO local registers */
static void rtl_sdio_local_written(RtlSdioState *s, uint32_t off, size_t len)
{
    uint8_t *hsus = &s->local[SDIO_REG_HSUS_CTRL];

    if (rtl_sdio_covers(off, len, SDIO_REG_HSUS_CTRL)) {
        *hsus = *hsus & HCI_SUS_REQ ? *hsus & ~HCI_RESUME_PWR_RDY
                                    : *hsus | HCI_RESUME_PWR_RDY;
    }
}

static void rtl_sdio_read(SDIOCard *card, unsigned fn, uint32_t addr,
                          bool incr, uint8_t *buf, size_t len)
{
    RtlSdioState *s = RTL_SDIO(card);
    unsigned dev = rtl_sdio_device(addr);
    uint32_t off;

    memset(buf, 0, len);
    switch (dev) {
    case DEV_SDIO_LOCAL:
        off = addr & (SDIO_LOCAL_SIZE - 1);
        memcpy(buf, s->local + off, MIN(len, SDIO_LOCAL_SIZE - off));
        break;
    case DEV_WLAN_IOREG:
        off = addr & (WLAN_IOREG_SIZE - 1);
        memcpy(buf, s->ioreg + off, MIN(len, WLAN_IOREG_SIZE - off));
        break;
    default:
        off = addr & 0x1fff;
        break;
    }
    trace_rtl_sdio_read(dev, off, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void rtl_sdio_write(SDIOCard *card, unsigned fn, uint32_t addr,
                           bool incr, const uint8_t *buf, size_t len)
{
    RtlSdioState *s = RTL_SDIO(card);
    unsigned dev = rtl_sdio_device(addr);
    uint32_t off;

    switch (dev) {
    case DEV_SDIO_LOCAL:
        off = addr & (SDIO_LOCAL_SIZE - 1);
        memcpy(s->local + off, buf, MIN(len, SDIO_LOCAL_SIZE - off));
        rtl_sdio_local_written(s, off, MIN(len, SDIO_LOCAL_SIZE - off));
        break;
    case DEV_WLAN_IOREG:
        off = addr & (WLAN_IOREG_SIZE - 1);
        memcpy(s->ioreg + off, buf, MIN(len, WLAN_IOREG_SIZE - off));
        rtl_sdio_ioreg_written(s, off, MIN(len, WLAN_IOREG_SIZE - off));
        break;
    default:
        off = addr & 0x1fff;
        break;
    }
    trace_rtl_sdio_write(dev, off, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void rtl_sdio_io_reset(SDIOCard *card)
{
    RtlSdioState *s = RTL_SDIO(card);

    memset(s->local, 0, sizeof(s->local));
    memset(s->ioreg, 0, sizeof(s->ioreg));
    s->local[SDIO_REG_HSUS_CTRL] = HCI_RESUME_PWR_RDY;
    s->ioreg[REG_APS_FSMCO_2] = POWER_READY;
    for (size_t i = 0; i < ARRAY_SIZE(rtl_sdio_reset_regs); i++) {
        stn_le_p(s->ioreg + rtl_sdio_reset_regs[i].reg,
                 rtl_sdio_reset_regs[i].size, rtl_sdio_reset_regs[i].val);
    }
}

static void rtl_sdio_init(Object *obj)
{
    RtlSdioState *s = RTL_SDIO(obj);

    memset(s->efuse, 0xff, sizeof(s->efuse));
    rtl_sdio_io_reset(SDIO_CARD(obj));
}

static const VMStateDescription rtl_sdio_vmstate = {
    .name = TYPE_RTL_SDIO,
    .unmigratable = 1,
};

static void rtl_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->vmsd = &rtl_sdio_vmstate;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x024c;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = 512;
    sc->read = rtl_sdio_read;
    sc->write = rtl_sdio_write;
    sc->io_reset = rtl_sdio_io_reset;
}

static void rtl_sdio_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);
    const RtlSdioChip *chip = data;

    RTL_SDIO_CLASS(oc)->chip = chip;
    dc->desc = chip->desc;
    sc->device = chip->device;
}

static void rtl_sdio_register_types(void)
{
    static const TypeInfo rtl_sdio_info = {
        .name           = TYPE_RTL_SDIO,
        .parent         = TYPE_SDIO_CARD,
        .instance_size  = sizeof(RtlSdioState),
        .instance_init  = rtl_sdio_init,
        .class_size     = sizeof(RtlSdioClass),
        .class_init     = rtl_sdio_class_init,
        .abstract       = true,
    };

    type_register_static(&rtl_sdio_info);
    for (size_t i = 0; i < ARRAY_SIZE(rtl_sdio_chips); i++) {
        const TypeInfo ti = {
            .name       = rtl_sdio_chips[i].name,
            .parent     = TYPE_RTL_SDIO,
            .class_init = rtl_sdio_chip_class_init,
            .class_data = &rtl_sdio_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(rtl_sdio_register_types)
