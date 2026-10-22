/*
 * AltoBeam ATBM603x Wi-Fi on SDIO
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

#define TYPE_ATBM6031 "atbm6031"
OBJECT_DECLARE_SIMPLE_TYPE(AtbmSdioState, ATBM6031)

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

/* ARES_B, the die of the 603x */
#define CHIP_ARES_B             0x49

/* the driver packs several messages into a transfer, 256-byte aligned */
#define TX_ALIGN                256

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

/* Messages from the host, each on a TX_ALIGN boundary */
static void atbm_sdio_take(AtbmSdioState *s, const uint8_t *buf, size_t len)
{
    size_t off, mlen;

    for (off = 0; off + 8 <= len; off += ROUND_UP(mlen, TX_ALIGN)) {
        /* each starts with a flag word, then its length and id */
        mlen = lduw_le_p(buf + off + 4);
        if (mlen < 8 || off + mlen > len) {
            break;
        }
        atbm_core_wsm(&s->core, lduw_le_p(buf + off + 6), buf + off + 8,
                      mlen - 8);
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
    AtbmSdioState *s = ATBM6031(card);
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
    AtbmSdioState *s = ATBM6031(card);
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
    AtbmSdioState *s = ATBM6031(card);

    memset(s->reg, 0, sizeof(s->reg));
    atbm_core_hold(&s->core);
}

static void atbm_sdio_realize(DeviceState *dev, Error **errp)
{
    AtbmSdioState *s = ATBM6031(dev);

    atbm_core_realize(&s->core, errp);
}

static void atbm_sdio_unrealize(DeviceState *dev)
{
    AtbmSdioState *s = ATBM6031(dev);

    atbm_core_unrealize(&s->core);
}

static void atbm_sdio_init(Object *obj)
{
    AtbmSdioState *s = ATBM6031(obj);

    s->core.dev = DEVICE(obj);
    s->core.notify = atbm_sdio_update_irq;
    s->core.chip_id = CHIP_ARES_B;
    atbm_core_init(&s->core);
}

static void atbm_sdio_finalize(Object *obj)
{
    AtbmSdioState *s = ATBM6031(obj);

    atbm_core_finalize(&s->core);
}

static const Property atbm_sdio_props[] = {
    DEFINE_ATBM_PROPERTIES(AtbmSdioState, core),
};

/* the queued messages and the chip memory are not migrated */
static const VMStateDescription atbm_sdio_vmstate = {
    .name = TYPE_ATBM6031,
    .unmigratable = 1,
};

static void atbm_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = atbm_sdio_realize;
    dc->unrealize = atbm_sdio_unrealize;
    dc->desc = "AltoBeam ATBM6031 SDIO Wi-Fi";
    dc->vmsd = &atbm_sdio_vmstate;
    device_class_set_props(dc, atbm_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x007a;
    sc->device = 0x6011;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = 512;
    sc->read = atbm_sdio_read;
    sc->write = atbm_sdio_write;
    sc->io_reset = atbm_sdio_io_reset;
}

static const TypeInfo atbm_sdio_types[] = {
    {
        .name           = TYPE_ATBM6031,
        .parent         = TYPE_SDIO_CARD,
        .instance_size  = sizeof(AtbmSdioState),
        .instance_init  = atbm_sdio_init,
        .instance_finalize = atbm_sdio_finalize,
        .class_init     = atbm_sdio_class_init,
    },
};

DEFINE_TYPES(atbm_sdio_types)
