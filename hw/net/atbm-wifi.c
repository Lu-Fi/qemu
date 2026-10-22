/*
 * AltoBeam ATBM603x Wi-Fi on SDIO
 *
 * The chip as its Linux driver (atbm-wifi hal_apollo, derived from
 * ST-Ericsson's cw1200) sees it: function 1's host interface registers,
 * the chip memory reached through their address and data ports, and the
 * WSM messages its firmware trades with the host. The firmware the driver
 * uploads is taken and dropped; the model answers in its place.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/sd/sdio.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "trace.h"

#define TYPE_ATBM6031 "atbm6031"
OBJECT_DECLARE_SIMPLE_TYPE(AtbmState, ATBM6031)

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
#define CHIP_VERSION_ADDR       0x0acc017c
#define CHIP_VERSION            0x49
/* the firmware's count of host messages taken, which paces the driver */
#define XMITED_ADDR             0x09000100
/* ITCM and DTCM, where the firmware goes and nothing is read back */
#define FIRMWARE_END            0x01000000

/* the driver packs several messages into a transfer, 256-byte aligned */
#define TX_ALIGN                256

#define WSM_CNF                 0x0400
#define WSM_GENERIC_REQ         0x0003
#define WSM_READ_MIB_REQ        0x0005
#define WSM_CONFIGURATION_REQ   0x0009
#define WSM_STARTUP_IND         0x0801

#define MIB_MAC_FROM_EFUSE      0x103a
#define MIB_EFUSE_DATA          0x103c

#define CAP_NO_CONFIRM          BIT(3)
#define CAP_VIFADDR_LOCAL_BIT   BIT(18)

#define NUM_INPUT_BUFS          16
#define INPUT_BUF_SIZE          1632

struct AtbmState {
    SDIOCard parent_obj;

    NICState *nic;
    NICConf conf;
    QEMUTimer *boot;

    uint32_t reg[HIF_NREGS];
    GHashTable *mem;
    uint32_t xmited;
    uint8_t seq;
    GQueue out;
};

static void atbm_update_irq(AtbmState *s)
{
    sdio_card_set_irq(SDIO_CARD(s), 1, !g_queue_is_empty(&s->out) &&
                      (s->reg[HIF_CONFIG] & CONFIG_IRQ_ENABLE));
}

/* A message for the host: its length and id, then the body */
static void atbm_send(AtbmState *s, uint16_t id, const void *body,
                      size_t len)
{
    GByteArray *m = g_byte_array_sized_new(4 + len);
    uint8_t hdr[4];

    id |= (s->seq & 7) << 13;
    s->seq++;
    stw_le_p(&hdr[0], 4 + len);
    stw_le_p(&hdr[2], id);
    g_byte_array_append(m, hdr, sizeof(hdr));
    g_byte_array_append(m, body, len);
    trace_atbm_wsm_send(id, 4 + len);
    g_queue_push_tail(&s->out, m);
    atbm_update_irq(s);
}

/* CONTROL: the next message's length, in 16-bit words, split 14:13 / 11:0 */
static uint32_t atbm_control(AtbmState *s)
{
    GByteArray *m = g_queue_peek_head(&s->out);
    uint32_t words = m ? (m->len + 1) / 2 : 0;

    return s->reg[HIF_CONTROL] | (words & 0x0fff) | ((words >> 12) & 3) << 14;
}

/*
 * The host reads the next message padded to its transfer, whose last
 * 16 bits carry CONTROL again: the length of the one after
 */
static void atbm_pop(AtbmState *s, uint8_t *buf, size_t len)
{
    GByteArray *m = g_queue_pop_head(&s->out);

    memset(buf, 0, len);
    if (m) {
        memcpy(buf, m->data, MIN(m->len, len));
        g_byte_array_unref(m);
    }
    if (len >= 2) {
        stw_le_p(buf + len - 2, atbm_control(s));
    }
    atbm_update_irq(s);
}

static uint32_t atbm_mem_read32(AtbmState *s, uint32_t addr)
{
    switch (addr) {
    case CHIP_VERSION_ADDR:
        return CHIP_VERSION;
    case XMITED_ADDR:
        return s->xmited;
    }
    return GPOINTER_TO_UINT(g_hash_table_lookup(s->mem,
                                                GUINT_TO_POINTER(addr)));
}

static void atbm_mem_write32(AtbmState *s, uint32_t addr, uint32_t val)
{
    if (addr < FIRMWARE_END) {
        return;
    }
    g_hash_table_insert(s->mem, GUINT_TO_POINTER(addr),
                        GUINT_TO_POINTER(val));
}

/* The data ports: bytes from the address in SRAM_BASE onwards */
static void atbm_mem_read(AtbmState *s, uint8_t *buf, size_t len)
{
    uint32_t base = s->reg[HIF_SRAM_BASE];

    for (size_t i = 0; i < len; i++) {
        uint32_t a = base + i;

        buf[i] = atbm_mem_read32(s, a & ~3) >> (8 * (a & 3));
    }
    trace_atbm_mem_read(base, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void atbm_mem_write(AtbmState *s, const uint8_t *buf, size_t len)
{
    uint32_t base = s->reg[HIF_SRAM_BASE];

    if (base < FIRMWARE_END) {
        return;
    }
    trace_atbm_mem_write(base, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
    for (size_t i = 0; i < len; i++) {
        uint32_t a = base + i;
        uint32_t w = atbm_mem_read32(s, a & ~3);
        unsigned sh = 8 * (a & 3);

        atbm_mem_write32(s, a & ~3, (w & ~(0xffu << sh)) | buf[i] << sh);
    }
}

static void atbm_startup(void *opaque)
{
    AtbmState *s = opaque;
    uint8_t ind[20 + 128 + 16 + 8] = {};
    uint32_t cap = CAP_NO_CONFIRM | CAP_VIFADDR_LOCAL_BIT;
    uint8_t *p = ind;

    stw_le_p(p, NUM_INPUT_BUFS);
    stw_le_p(p + 2, INPUT_BUF_SIZE);
    /* hardware id and sub id, then status 0 */
    stw_le_p(p + 10, cap);
    /* firmware type WFM, API version, build, version */
    stw_le_p(p + 12, 1);
    stw_le_p(p + 14, 0x0100);
    stw_le_p(p + 16, 1);
    stw_le_p(p + 18, 15000);
    p += 20;
    pstrcpy((char *)p, 128, "QEMU model of the ATBM603x firmware");
    p += 128;
    /* 8 stations, 8 aggregates each way, 2 interfaces */
    stl_le_p(p, 8 | 8 << 8 | 2 << 16 | 8u << 24);
    stl_le_p(p + 12, XMITED_ADDR);
    p += 16;
    stw_le_p(p, cap >> 16);
    atbm_send(s, WSM_STARTUP_IND, ind, sizeof(ind));
}

static void atbm_confirm(AtbmState *s, uint16_t cnf, const void *data,
                         size_t len)
{
    g_autofree uint8_t *body = g_malloc0(4 + len);

    /* status: success */
    memcpy(body + 4, data, len);
    atbm_send(s, cnf, body, 4 + len);
}

static void atbm_read_mib(AtbmState *s, uint16_t cnf, uint16_t mib)
{
    uint8_t mibcnf[4 + 64] = {};
    const uint8_t *mac = s->conf.macaddr.a;

    stw_le_p(&mibcnf[0], mib);
    stw_le_p(&mibcnf[2], 64);
    switch (mib) {
    case MIB_MAC_FROM_EFUSE:
        memcpy(&mibcnf[4], mac, 6);
        break;
    case MIB_EFUSE_DATA:
        /* struct efuse_headr: nine bytes of trims, then the MAC */
        memcpy(&mibcnf[4 + 9], mac, 6);
        break;
    }
    atbm_confirm(s, cnf, mibcnf, sizeof(mibcnf));
}

/* One message from the host */
static void atbm_wsm(AtbmState *s, uint16_t id, const uint8_t *body,
                     size_t len)
{
    uint16_t cnf = (id & 0x03ff) | WSM_CNF;
    uint8_t zeros[256] = {};
    uint8_t c[40] = {};

    trace_atbm_wsm_take(id, len);
    switch (id & 0x3f) {
    case WSM_READ_MIB_REQ:
        atbm_read_mib(s, cnf, len >= 2 ? lduw_le_p(body) : 0);
        return;
    case WSM_CONFIGURATION_REQ:
        /* the MAC, 2.4 GHz only, the 802.11b/g rates, 0-20 dBm */
        memcpy(&c[0], s->conf.macaddr.a, 6);
        c[6] = 1;
        stl_le_p(&c[8], 0x3fff);
        stl_le_p(&c[16], 200);
        stl_le_p(&c[20], 10);
        stl_le_p(&c[28], 200);
        stl_le_p(&c[32], 10);
        atbm_confirm(s, cnf, c, 36);
        return;
    case WSM_GENERIC_REQ:
        /* echo the request id; status and payload follow */
        memcpy(zeros, body, MIN(len, 4));
        atbm_send(s, cnf, zeros, sizeof(zeros));
        return;
    }
    atbm_confirm(s, cnf, zeros, 64);
}

/* Messages from the host, each on a TX_ALIGN boundary */
static void atbm_take(AtbmState *s, const uint8_t *buf, size_t len)
{
    size_t off, mlen;

    for (off = 0; off + 8 <= len; off += ROUND_UP(mlen, TX_ALIGN)) {
        /* each starts with a flag word, then its length and id */
        mlen = lduw_le_p(buf + off + 4);
        if (mlen < 8 || off + mlen > len) {
            break;
        }
        atbm_wsm(s, lduw_le_p(buf + off + 6), buf + off + 8, mlen - 8);
        s->xmited++;
    }
}

static void atbm_reg_write(AtbmState *s, unsigned reg, uint32_t val)
{
    uint32_t old = s->reg[reg];

    trace_atbm_reg_write(reg, val);
    switch (reg) {
    case HIF_CONFIG:
        /* a prefetch is done as soon as it is asked for */
        s->reg[reg] = val & ~(CONFIG_AHB_PFETCH | CONFIG_APB_PFETCH);
        if ((old & CONFIG_CPU_RESET) && !(val & CONFIG_CPU_RESET)) {
            /* out of reset, the firmware reports in once it is up */
            timer_mod(s->boot, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 20);
        }
        atbm_update_irq(s);
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

static uint32_t atbm_reg_read(AtbmState *s, unsigned reg)
{
    uint32_t val = reg == HIF_CONTROL ? atbm_control(s) : s->reg[reg];

    trace_atbm_reg_read(reg, val);
    return val;
}

static void atbm_read(SDIOCard *card, unsigned fn, uint32_t addr, bool incr,
                      uint8_t *buf, size_t len)
{
    AtbmState *s = ATBM6031(card);
    unsigned reg = (addr & 0x1f) >> 2;

    switch (reg) {
    case HIF_QUEUE:
        atbm_pop(s, buf, len);
        return;
    case HIF_AHB_DPORT:
    case HIF_SRAM_DPORT:
        atbm_mem_read(s, buf, len);
        return;
    }
    for (size_t i = 0; i < len; i++) {
        uint32_t a = (addr & 0x1f) + (incr ? i : 0);

        buf[i] = atbm_reg_read(s, (a >> 2) % HIF_NREGS) >> (8 * (a & 3));
    }
}

static void atbm_write(SDIOCard *card, unsigned fn, uint32_t addr, bool incr,
                       const uint8_t *buf, size_t len)
{
    AtbmState *s = ATBM6031(card);
    unsigned reg = (addr & 0x1f) >> 2;
    uint32_t val;

    switch (reg) {
    case HIF_QUEUE:
        atbm_take(s, buf, len);
        return;
    case HIF_AHB_DPORT:
    case HIF_SRAM_DPORT:
        atbm_mem_write(s, buf, len);
        return;
    }
    if (len >= 4 && !(addr & 3)) {
        atbm_reg_write(s, reg, ldl_le_p(buf));
        return;
    }
    /* a CMD52 writes one byte of a register */
    for (size_t i = 0; i < len; i++) {
        uint32_t a = (addr & 0x1f) + (incr ? i : 0);
        unsigned r = (a >> 2) % HIF_NREGS, sh = 8 * (a & 3);

        val = (s->reg[r] & ~(0xffu << sh)) | buf[i] << sh;
        atbm_reg_write(s, r, val);
    }
}

static void atbm_io_reset(SDIOCard *card)
{
    AtbmState *s = ATBM6031(card);
    GByteArray *m;

    timer_del(s->boot);
    memset(s->reg, 0, sizeof(s->reg));
    while ((m = g_queue_pop_head(&s->out))) {
        g_byte_array_unref(m);
    }
    g_hash_table_remove_all(s->mem);
    s->xmited = 0;
    s->seq = 0;
}

static bool atbm_can_receive(NetClientState *nc)
{
    return true;
}

static ssize_t atbm_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    return size;
}

static NetClientInfo atbm_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = atbm_can_receive,
    .receive = atbm_receive,
};

static void atbm_realize(DeviceState *dev, Error **errp)
{
    AtbmState *s = ATBM6031(dev);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&atbm_net_info, &s->conf, TYPE_ATBM6031, dev->id,
                          &dev->mem_reentrancy_guard, s);
}

static void atbm_init(Object *obj)
{
    AtbmState *s = ATBM6031(obj);

    s->boot = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_startup, s);
    s->mem = g_hash_table_new(g_direct_hash, g_direct_equal);
    g_queue_init(&s->out);
}

static void atbm_finalize(Object *obj)
{
    AtbmState *s = ATBM6031(obj);

    atbm_io_reset(SDIO_CARD(s));
    timer_free(s->boot);
    g_hash_table_destroy(s->mem);
}

static const Property atbm_props[] = {
    DEFINE_NIC_PROPERTIES(AtbmState, conf),
};

/* the queued messages and the chip memory are not migrated */
static const VMStateDescription atbm_vmstate = {
    .name = TYPE_ATBM6031,
    .unmigratable = 1,
};

static void atbm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = atbm_realize;
    dc->desc = "AltoBeam ATBM6031 SDIO Wi-Fi";
    dc->vmsd = &atbm_vmstate;
    device_class_set_props(dc, atbm_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

    sc->vendor = 0x007a;
    sc->device = 0x6011;
    sc->num_funcs = 1;
    sc->func_class = 0x07;
    sc->max_blksz = 512;
    sc->read = atbm_read;
    sc->write = atbm_write;
    sc->io_reset = atbm_io_reset;
}

static const TypeInfo atbm_types[] = {
    {
        .name           = TYPE_ATBM6031,
        .parent         = TYPE_SDIO_CARD,
        .instance_size  = sizeof(AtbmState),
        .instance_init  = atbm_init,
        .instance_finalize = atbm_finalize,
        .class_init     = atbm_class_init,
    },
};

DEFINE_TYPES(atbm_types)
