/*
 * AltoBeam ATBM60xx Wi-Fi on SDIO
 *
 * Function 1's host interface registers: configuration and control, the
 * message queue, and ports onto the chip memory through an address
 * register. The firmware behind them is atbm-wifi.c.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/sd/sdio.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "atbm-wifi.h"
#include "trace.h"

#define TYPE_ATBM_SDIO "atbm-sdio"
OBJECT_DECLARE_TYPE(AtbmSdioState, AtbmSdioClass, ATBM_SDIO)

/*
 * Host interface registers, at function 1 address (register * 4); bits
 * 6-12 carry a buffer id the driver rotates through on queue transfers
 */
enum {
    HIF_CONFIG,
    HIF_CONTROL,
    HIF_QUEUE,
    HIF_AHB_DPORT,
    HIF_SRAM_BASE,
    HIF_SRAM_DPORT,
    HIF_TSET_GEN,
    HIF_FRAME_OUT,
    HIF_NREGS,
};

#define CONFIG_AHB_PFETCH       BIT(11)
#define CONFIG_APB_PFETCH       BIT(13)
#define CONFIG_CPU_RESET        BIT(14)
#define CONFIG_IRQ_ENABLE       (BIT(16) | BIT(17))

#define CONTROL_WUP             BIT(12)
#define CONTROL_RDY             BIT(13)

/* the driver packs several messages into a transfer, 256-byte aligned */
#define TX_ALIGN                256

typedef struct AtbmSdioChip {
    const char *name;
    const char *desc;
    uint16_t device;
    uint8_t chip_id;
    bool wifi6;
} AtbmSdioChip;

struct AtbmSdioClass {
    SDIOCardClass parent_class;

    const AtbmSdioChip *chip;
};

/* AresB, then the Wi-Fi 6 dies Cronus and Oceanus */
static const AtbmSdioChip atbm_sdio_chips[] = {
    { "atbm6031",  "AltoBeam ATBM6031 SDIO Wi-Fi",  0x6011, 0x49, false },
    { "atbm6062s", "AltoBeam ATBM6062S SDIO Wi-Fi", 0x6051, 0x5a, true },
    { "atbm6162s", "AltoBeam ATBM6162S SDIO Wi-Fi", 0x6055, 0x30, true },
};

struct AtbmSdioState {
    SDIOCard parent_obj;

    AtbmCore core;
    uint32_t reg[HIF_NREGS];
};

static void atbm_sdio_update_irq(AtbmCore *c)
{
    AtbmSdioState *s = container_of(c, AtbmSdioState, core);

    sdio_card_set_irq(SDIO_CARD(s), 1, atbm_core_peek(c) &&
                      (s->reg[HIF_CONFIG] & CONFIG_IRQ_ENABLE));
}

/* CONTROL: the next message's length, in 16-bit words, split 14:13 / 11:0 */
static uint32_t atbm_sdio_control(AtbmSdioState *s)
{
    GByteArray *m = atbm_core_peek(&s->core);
    uint32_t words = m ? (m->len + 1) / 2 : 0;

    return s->reg[HIF_CONTROL] | (words & 0x0fff) | ((words >> 12) & 3) << 14;
}

/*
 * The host reads the next message padded to its transfer, whose last
 * 16 bits carry CONTROL again: the length of the one after
 */
static void atbm_sdio_pop(AtbmSdioState *s, uint8_t *buf, size_t len)
{
    GByteArray *m = atbm_core_pop(&s->core);

    memset(buf, 0, len);
    if (m) {
        memcpy(buf, m->data, MIN(m->len, len));
        g_byte_array_unref(m);
    }
    if (len >= 2) {
        stw_le_p(buf + len - 2, atbm_sdio_control(s));
    }
    atbm_sdio_update_irq(&s->core);
}

/*
 * Messages from the host, each on a TX_ALIGN boundary: a flag word, then
 * the length and id. The Wi-Fi 6 drivers put 24 more bytes before those.
 */
static void atbm_sdio_take(AtbmSdioState *s, const uint8_t *buf, size_t len)
{
    size_t hdr = s->core.wifi6 ? 32 : 8;
    size_t off, mlen;

    for (off = 0; off + hdr <= len; off += ROUND_UP(mlen, TX_ALIGN)) {
        mlen = lduw_le_p(buf + off + hdr - 4);
        if (mlen < hdr || off + mlen > len) {
            break;
        }
        atbm_core_wsm(&s->core, lduw_le_p(buf + off + hdr - 2),
                      buf + off + hdr, mlen - hdr);
    }
}

/* The data ports: bytes from the address in SRAM_BASE onwards */
static void atbm_sdio_mem_read(AtbmSdioState *s, uint8_t *buf, size_t len)
{
    uint32_t base = s->reg[HIF_SRAM_BASE];

    for (size_t i = 0; i < len; i++) {
        uint32_t a = base + i;

        buf[i] = atbm_core_read32(&s->core, a & ~3) >> (8 * (a & 3));
    }
    trace_atbm_mem_read(base, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void atbm_sdio_mem_write(AtbmSdioState *s, const uint8_t *buf,
                                size_t len)
{
    uint32_t base = s->reg[HIF_SRAM_BASE];

    trace_atbm_mem_write(base, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
    if (len == 4 && !(base & 3)) {
        atbm_core_write32(&s->core, base, ldl_le_p(buf));
        return;
    }
    for (size_t i = 0; i < len; i++) {
        uint32_t a = base + i;
        uint32_t w = atbm_core_read32(&s->core, a & ~3);
        unsigned sh = 8 * (a & 3);

        atbm_core_write32(&s->core, a & ~3,
                          (w & ~(0xffu << sh)) | buf[i] << sh);
    }
}

static void atbm_sdio_reg_write(AtbmSdioState *s, unsigned reg, uint32_t val)
{
    uint32_t old = s->reg[reg];

    trace_atbm_reg_write(reg, val);
    switch (reg) {
    case HIF_CONFIG:
        /* a prefetch is done as soon as it is asked for */
        s->reg[reg] = val & ~(CONFIG_AHB_PFETCH | CONFIG_APB_PFETCH);
        if (!(old & CONFIG_CPU_RESET) && (val & CONFIG_CPU_RESET)) {
            atbm_core_hold(&s->core);
        } else if ((old & CONFIG_CPU_RESET) && !(val & CONFIG_CPU_RESET)) {
            atbm_core_run(&s->core);
        }
        atbm_sdio_update_irq(&s->core);
        break;
    case HIF_CONTROL:
        /* asked to wake, the device is ready at once */
        s->reg[reg] = val & CONTROL_WUP ? CONTROL_WUP | CONTROL_RDY
                                        : old & CONTROL_RDY;
        break;
    default:
        s->reg[reg] = val;
        break;
    }
}

static uint32_t atbm_sdio_reg_read(AtbmSdioState *s, unsigned reg)
{
    uint32_t val = reg == HIF_CONTROL ? atbm_sdio_control(s) : s->reg[reg];

    trace_atbm_reg_read(reg, val);
    return val;
}

static void atbm_sdio_read(SDIOCard *card, unsigned fn, uint32_t addr,
                           bool incr, uint8_t *buf, size_t len)
{
    AtbmSdioState *s = ATBM_SDIO(card);
    unsigned reg = (addr & 0x1f) >> 2;

    switch (reg) {
    case HIF_QUEUE:
        atbm_sdio_pop(s, buf, len);
        return;
    case HIF_AHB_DPORT:
    case HIF_SRAM_DPORT:
        atbm_sdio_mem_read(s, buf, len);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        uint32_t a = (addr & 0x1f) + (incr ? i : 0);

        buf[i] = atbm_sdio_reg_read(s, (a >> 2) % HIF_NREGS) >> (8 * (a & 3));
    }
}

static void atbm_sdio_write(SDIOCard *card, unsigned fn, uint32_t addr,
                            bool incr, const uint8_t *buf, size_t len)
{
    AtbmSdioState *s = ATBM_SDIO(card);
    unsigned reg = (addr & 0x1f) >> 2;
    uint32_t val;

    switch (reg) {
    case HIF_QUEUE:
        atbm_sdio_take(s, buf, len);
        return;
    case HIF_AHB_DPORT:
    case HIF_SRAM_DPORT:
        atbm_sdio_mem_write(s, buf, len);
        return;
    }
    if (len >= 4 && !(addr & 3)) {
        atbm_sdio_reg_write(s, reg, ldl_le_p(buf));
        return;
    }
    /* a CMD52 writes one byte of a register */
    for (size_t i = 0; i < len; i++) {
        uint32_t a = (addr & 0x1f) + (incr ? i : 0);
        unsigned r = (a >> 2) % HIF_NREGS, sh = 8 * (a & 3);

        val = (s->reg[r] & ~(0xffu << sh)) | buf[i] << sh;
        atbm_sdio_reg_write(s, r, val);
    }
}

static void atbm_sdio_io_reset(SDIOCard *card)
{
    AtbmSdioState *s = ATBM_SDIO(card);

    memset(s->reg, 0, sizeof(s->reg));
    atbm_core_hold(&s->core);
}

static void atbm_sdio_realize(DeviceState *dev, Error **errp)
{
    AtbmSdioState *s = ATBM_SDIO(dev);

    atbm_core_realize(&s->core, errp);
}

static void atbm_sdio_unrealize(DeviceState *dev)
{
    AtbmSdioState *s = ATBM_SDIO(dev);

    atbm_core_unrealize(&s->core);
}

static void atbm_sdio_init(Object *obj)
{
    AtbmSdioState *s = ATBM_SDIO(obj);
    const AtbmSdioChip *chip = ATBM_SDIO_GET_CLASS(obj)->chip;

    s->core.dev = DEVICE(obj);
    s->core.notify = atbm_sdio_update_irq;
    s->core.chip_id = chip->chip_id;
    s->core.wifi6 = chip->wifi6;
    atbm_core_init(&s->core);
}

static void atbm_sdio_finalize(Object *obj)
{
    AtbmSdioState *s = ATBM_SDIO(obj);

    atbm_core_finalize(&s->core);
}

static const Property atbm_sdio_props[] = {
    DEFINE_ATBM_PROPERTIES(AtbmSdioState, core),
};

/* the queued messages and the chip memory are not migrated */
static const VMStateDescription atbm_sdio_vmstate = {
    .name = TYPE_ATBM_SDIO,
    .unmigratable = 1,
};

static void atbm_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = atbm_sdio_realize;
    dc->unrealize = atbm_sdio_unrealize;
    dc->vmsd = &atbm_sdio_vmstate;
    device_class_set_props(dc, atbm_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x007a;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = 512;
    sc->read = atbm_sdio_read;
    sc->write = atbm_sdio_write;
    sc->io_reset = atbm_sdio_io_reset;
}

static void atbm_sdio_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);
    const AtbmSdioChip *chip = data;

    ATBM_SDIO_CLASS(oc)->chip = chip;
    dc->desc = chip->desc;
    sc->device = chip->device;
}

static void atbm_sdio_register_types(void)
{
    static const TypeInfo atbm_sdio_info = {
        .name           = TYPE_ATBM_SDIO,
        .parent         = TYPE_SDIO_CARD,
        .instance_size  = sizeof(AtbmSdioState),
        .instance_init  = atbm_sdio_init,
        .instance_finalize = atbm_sdio_finalize,
        .class_size     = sizeof(AtbmSdioClass),
        .class_init     = atbm_sdio_class_init,
        .abstract       = true,
    };

    type_register_static(&atbm_sdio_info);
    for (size_t i = 0; i < ARRAY_SIZE(atbm_sdio_chips); i++) {
        const TypeInfo ti = {
            .name       = atbm_sdio_chips[i].name,
            .parent     = TYPE_ATBM_SDIO,
            .class_init = atbm_sdio_chip_class_init,
            .class_data = &atbm_sdio_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(atbm_sdio_register_types)
