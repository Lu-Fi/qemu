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
#include "hw/net/wifi-ap.h"
#include "migration/vmstate.h"
#include "net/eth.h"
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
#define WSM_TX_REQ              0x0004
#define WSM_READ_MIB_REQ        0x0005
#define WSM_START_SCAN_REQ      0x0007
#define WSM_STOP_SCAN_REQ       0x0008
#define WSM_CONFIGURATION_REQ   0x0009
#define WSM_JOIN_REQ            0x000b
#define WSM_ADD_KEY_REQ         0x000c
#define WSM_REMOVE_KEY_REQ      0x000d
#define WSM_SET_PM_REQ          0x0010
#define WSM_STARTUP_IND         0x0801
#define WSM_RECEIVE_IND         0x0804
#define WSM_SCAN_COMPLETE_IND   0x0806
#define WSM_SET_PM_COMPLETE_IND 0x0809

#define WSM_STATUS_FAILURE      1
#define WSM_STATUS_RETRY_EXCEEDED 6
#define WSM_STATUS_NO_KEY_FOUND 9
#define WSM_STATUS_NOEFFECT     14

/* struct wsm_tx, after the message header: the frame follows */
#define TX_FLAGS                7
#define TX_HT_PARAMS            16
#define TX_FRAME                20
#define TX_2BYTES_SHIFT         BIT(7)
#define TX_NEED_CONFIRM         BIT(8)

/* struct wsm_rx, before the frame */
#define RX_DESC                 16
#define RX_STATUS_AES           3
#define RX_STATUS_BEACON        BIT(7)
#define RX_STATUS_GROUP_KEY     BIT(19)

#define RATE_1                  0
#define RATE_54                 13
#define RSSI                    (-40)

#define KEY_TYPE_AES_GROUP      4
#define KEY_TYPE_AES_PAIRWISE   5

#define FC_PROTECTED            BIT(14)
#define CCMP_HDR                8
#define CCMP_MIC                8

/* indications the driver has not read yet before the network waits */
#define OUT_QUEUE_MAX           64

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

    WifiAP ap;
    QEMUTimer *scan;
    unsigned scan_channels;
    bool scan_hit;
    uint8_t scan_if;
    uint8_t join_if;
    bool ptk_set;
    bool gtk_set;
    uint8_t gtk_id;
    uint64_t ptk_pn;
    uint64_t gtk_pn;
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
        if (g_queue_get_length(&s->out) == OUT_QUEUE_MAX - 1) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
        }
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

static void atbm_confirm(AtbmState *s, uint16_t cnf, uint32_t status,
                         const void *data, size_t len)
{
    g_autofree uint8_t *body = g_malloc0(4 + len);

    stl_le_p(body, status);
    if (len) {
        memcpy(body + 4, data, len);
    }
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
    atbm_confirm(s, cnf, 0, mibcnf, sizeof(mibcnf));
}

/*
 * A frame off the air, in the shape the hardware leaves it: one that came
 * encrypted keeps its CCMP header and MIC, now over the plaintext
 */
static void atbm_rx(AtbmState *s, uint8_t if_id, const uint8_t *frame,
                    size_t len, bool protected)
{
    g_autofree uint8_t *ind = g_malloc0(RX_DESC + len + CCMP_HDR + CCMP_MIC);
    uint16_t fc = lduw_le_p(frame);
    uint8_t *f = ind + RX_DESC;
    uint32_t status = 0, flags = 0;

    if (protected) {
        bool group = is_multicast_ether_addr(frame + 4);
        uint64_t pn = group ? ++s->gtk_pn : ++s->ptk_pn;

        if (!(group ? s->gtk_set : s->ptk_set)) {
            status = WSM_STATUS_NO_KEY_FOUND;
        }
        memcpy(f, frame, 24);
        stw_le_p(f, fc | FC_PROTECTED);
        f[24] = pn;
        f[25] = pn >> 8;
        /* ExtIV, then the key id */
        f[27] = 0x20 | (group ? s->gtk_id : 0) << 6;
        stl_le_p(f + 28, pn >> 16);
        memcpy(f + 24 + CCMP_HDR, frame + 24, len - 24);
        len += CCMP_HDR + CCMP_MIC;
        flags = RX_STATUS_AES | (group ? RX_STATUS_GROUP_KEY : 0);
    } else {
        memcpy(f, frame, len);
        if (fc == 0x0080) {
            flags = RX_STATUS_BEACON;
        }
    }
    stl_le_p(ind, status);
    stw_le_p(ind + 4, s->ap.channel);
    ind[6] = (fc & 0x0c) == 0x08 ? RATE_54 : RATE_1;
    ind[7] = RSSI;
    stl_le_p(ind + 8, flags);
    atbm_send(s, WSM_RECEIVE_IND | if_id << 6, ind, RX_DESC + len);
}

static void atbm_deliver(void *opaque, const uint8_t *frame, size_t len,
                         bool protected)
{
    AtbmState *s = opaque;

    atbm_rx(s, s->join_if, frame, len, protected);
}

/*
 * A frame for the air. The driver wants to hear back about those it marks,
 * and whatever it wants encrypted comes with room for the CCMP header and
 * MIC, which go before the frame reaches the access point.
 */
static void atbm_tx(AtbmState *s, uint16_t id, const uint8_t *body,
                    size_t len)
{
    size_t off = TX_FRAME + (body[TX_FLAGS] & TX_2BYTES_SHIFT ? 2 : 0);
    const uint8_t *frame = body + off;
    g_autofree uint8_t *plain = NULL;
    size_t flen = len > off ? len - off : 0;
    uint32_t status = 0;
    uint16_t fc;
    size_t hdr;

    if (flen < 24) {
        return;
    }
    fc = lduw_le_p(frame);
    trace_atbm_tx(ldl_le_p(body), fc, flen);
    /* no one else is out there to acknowledge it */
    if (!is_multicast_ether_addr(frame + 4) &&
        memcmp(frame + 4, s->ap.bssid.a, 6)) {
        status = WSM_STATUS_RETRY_EXCEEDED;
    }
    if (ldl_le_p(body + TX_HT_PARAMS) & TX_NEED_CONFIRM) {
        uint8_t c[20] = {};

        stl_le_p(c, ldl_le_p(body));
        stl_le_p(c + 4, status);
        c[8] = body[4];
        atbm_send(s, (id & 0x03ff) | WSM_CNF, c, sizeof(c));
    }
    if (status) {
        return;
    }
    if (fc & FC_PROTECTED) {
        hdr = 24 + ((fc & 0x8c) == 0x88 ? 2 : 0);
        if (flen < hdr + CCMP_HDR + CCMP_MIC) {
            return;
        }
        flen -= CCMP_HDR + CCMP_MIC;
        plain = g_malloc(flen);
        memcpy(plain, frame, hdr);
        stw_le_p(plain, fc & ~FC_PROTECTED);
        memcpy(plain + hdr, frame + hdr + CCMP_HDR, flen - hdr);
        frame = plain;
    }
    wifi_ap_receive(&s->ap, frame, flen);
}

static void atbm_scan_done(void *opaque)
{
    AtbmState *s = opaque;
    /* status, power save mode, channels done, then busy ratios */
    uint8_t c[4 + 2 + 14] = {};

    if (s->scan_hit) {
        uint8_t b[WIFI_AP_BEACON_MAX];

        atbm_rx(s, s->scan_if, b, wifi_ap_beacon(&s->ap, b), false);
    }
    c[5] = s->scan_channels;
    atbm_send(s, WSM_SCAN_COMPLETE_IND, c, sizeof(c));
}

/* The channel list follows 12 bytes of parameters, 16 bytes a channel */
static void atbm_scan(AtbmState *s, uint16_t id, const uint8_t *b,
                      size_t len)
{
    unsigned n = len >= 12 ? b[9] : 0;

    s->scan_if = (id >> 6) & 0xf;
    s->scan_channels = n;
    s->scan_hit = false;
    for (unsigned i = 0; i < n && 12 + 16 * (i + 1) <= len; i++) {
        if (lduw_le_p(b + 12 + 16 * i) == s->ap.channel) {
            s->scan_hit = true;
        }
    }
    timer_mod(s->scan, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                       10 * MAX(n, 1));
}

/* struct wsm_add_key: type, index, two reserved, then by type */
static void atbm_key(AtbmState *s, bool add, const uint8_t *b, size_t len)
{
    if (len < 4) {
        return;
    }
    switch (b[0]) {
    case KEY_TYPE_AES_PAIRWISE:
        s->ptk_set = add;
        s->ptk_pn = 0;
        break;
    case KEY_TYPE_AES_GROUP:
        s->gtk_set = add;
        s->gtk_id = len > 20 ? b[20] & 3 : 0;
        s->gtk_pn = 0;
        break;
    }
}

/* One message from the host */
static void atbm_wsm(AtbmState *s, uint16_t id, const uint8_t *body,
                     size_t len)
{
    uint16_t cnf = (id & 0x03ff) | WSM_CNF;
    uint8_t zeros[256] = {};
    uint8_t c[40] = {};
    bool ok;

    trace_atbm_wsm_take(id, len);
    switch (id & 0x3f) {
    case WSM_TX_REQ:
        atbm_tx(s, id, body, len);
        return;
    case WSM_START_SCAN_REQ:
        atbm_confirm(s, cnf, 0, NULL, 0);
        atbm_scan(s, id, body, len);
        return;
    case WSM_STOP_SCAN_REQ:
        if (!timer_pending(s->scan)) {
            atbm_confirm(s, cnf, WSM_STATUS_NOEFFECT, NULL, 0);
            return;
        }
        atbm_confirm(s, cnf, 0, NULL, 0);
        timer_del(s->scan);
        s->scan_hit = false;
        atbm_scan_done(s);
        return;
    case WSM_JOIN_REQ:
        /* channel, BSSID; the power range in 0.1 dBm */
        ok = len >= 10 && lduw_le_p(body + 2) == s->ap.channel &&
             !memcmp(body + 4, s->ap.bssid.a, 6);
        s->join_if = (id >> 6) & 0xf;
        stl_le_p(&c[4], 200);
        atbm_confirm(s, cnf, ok ? 0 : WSM_STATUS_FAILURE, c, 8);
        return;
    case WSM_SET_PM_REQ:
        /* the mode is in once the confirm is out */
        atbm_confirm(s, cnf, 0, NULL, 0);
        c[0] = len ? body[0] : 0;
        atbm_send(s, WSM_SET_PM_COMPLETE_IND, c, 4);
        return;
    case WSM_ADD_KEY_REQ:
    case WSM_REMOVE_KEY_REQ:
        atbm_key(s, (id & 0x3f) == WSM_ADD_KEY_REQ, body, len);
        break;
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
        atbm_confirm(s, cnf, 0, c, 36);
        return;
    case WSM_GENERIC_REQ:
        /* echo the request id; status and payload follow */
        memcpy(zeros, body, MIN(len, 4));
        atbm_send(s, cnf, zeros, sizeof(zeros));
        return;
    }
    atbm_confirm(s, cnf, 0, zeros, 64);
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
    timer_del(s->scan);
    memset(s->reg, 0, sizeof(s->reg));
    while ((m = g_queue_pop_head(&s->out))) {
        g_byte_array_unref(m);
    }
    g_hash_table_remove_all(s->mem);
    s->xmited = 0;
    s->seq = 0;
    s->ptk_set = false;
    s->gtk_set = false;
    if (s->ap.eapol_timer) {
        wifi_ap_reset(&s->ap);
    }
}

static bool atbm_can_receive(NetClientState *nc)
{
    AtbmState *s = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&s->out) < OUT_QUEUE_MAX;
}

static ssize_t atbm_receive(NetClientState *nc, const uint8_t *buf,
                            size_t size)
{
    AtbmState *s = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&s->ap, buf, size);
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
    if (!wifi_ap_realize(&s->ap, s->nic, atbm_deliver, s, errp)) {
        qemu_del_nic(s->nic);
        s->nic = NULL;
    }
}

static void atbm_unrealize(DeviceState *dev)
{
    AtbmState *s = ATBM6031(dev);

    wifi_ap_unrealize(&s->ap);
    qemu_del_nic(s->nic);
}

static void atbm_init(Object *obj)
{
    AtbmState *s = ATBM6031(obj);

    s->boot = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_startup, s);
    s->scan = timer_new_ms(QEMU_CLOCK_VIRTUAL, atbm_scan_done, s);
    s->mem = g_hash_table_new(g_direct_hash, g_direct_equal);
    g_queue_init(&s->out);
}

static void atbm_finalize(Object *obj)
{
    AtbmState *s = ATBM6031(obj);

    atbm_io_reset(SDIO_CARD(s));
    timer_free(s->boot);
    timer_free(s->scan);
    g_hash_table_destroy(s->mem);
}

/* the access point the chip finds on the air */
static const Property atbm_props[] = {
    DEFINE_NIC_PROPERTIES(AtbmState, conf),
    DEFINE_PROP_STRING("ssid", AtbmState, ap.ssid),
    DEFINE_PROP_STRING("psk", AtbmState, ap.psk),
    DEFINE_PROP_UINT8("channel", AtbmState, ap.channel, 6),
    DEFINE_PROP_MACADDR("bssid", AtbmState, ap.bssid),
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
    dc->unrealize = atbm_unrealize;
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
