/*
 * iComm-semi SSV6006C, SSV6020 and SSV6030 Wi-Fi: the chip and firmware
 * the bus front ends share
 *
 * The ssv6x5x driver is SoftMAC: mac80211 builds every frame, and the chip's
 * N10 firmware only does what the host commands ask, with the registers
 * the driver checks along the way. The firmware the driver downloads is
 * summed for the boot handshake and otherwise dropped; this model answers
 * in its place, and puts an access point (wifi-ap.c) on the air, and a
 * client (wifi-client.c) for an access point the host runs.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/qdev.h"
#include "net/eth.h"
#include "ssv-wifi.h"
#include "trace.h"
#include "wifi-frame.h"

/* "SSV6006C0" or "SSV6020C0", space padded, the first character on top */
#define REG_CHIP_ID_0           0xc0000008
#define REG_CHIP_ID_3           0xc0000014
/* the driver attaches the chip's own PHY only to a part of type 0xa0 */
#define REG_CHIP_TYPE_VER       0xc00000c8
#define CHIP_TYPE_ASIC          0xa0000000

/* a MAC reset, done at once */
#define REG_BRG_SW_RST          0xc0000000
/*
 * The N10 CPU: leaving reset with the vector base at 4 runs the boot ROM,
 * at 0 the downloaded firmware. The 6006C starts it with its clock enable,
 * the 6020 with a reboot command to the system controller, the vector
 * base in the top half of N10CFG_SETTING.
 */
#define REG_PLATFORM_CLOCK_EN   0xc000001c
#define RESET_N_CPUN10          BIT(24)
#define REG_IVB                 0xc00000e8
#define REG_N10CFG_SETTING      0xc000014c
#define IVB_ROM                 4
#define REG_SYSCTRL_COMMAND     0xc0000054
#define SYSCTRL_REBOOT          0x4008
#define SYSCTRL_REBOOT_STANDBY  0x400b
/* the 6020's DLM block, which its firmware leaves out of the sum */
#define FW_DLM_6020             0x00100000
/* the CPU always shows standby, the power controller always on */
#define REG_N10_DBG2            0xc000002c
#define N10_STANDBY             BIT(30)
/* RTC, oscillator and, on the 6020, DPLL ready; the PMU on */
#define REG_SYSCTRL_STATUS      0xc000005c
#define SYSCTRL_ON_6006C        0x303
#define SYSCTRL_ON_6020         0x307
/* the 6020's SRAM banks come up as soon as they are asked to */
#define REG_SRAM_PG_STATUS0     0xc0000080
#define REG_SRAM_PG_STATUS1     0xc0000084
#define REG_SRAM_PG_CTRL0       0xc0000150
#define REG_SRAM_PG_CTRL1       0xc0000154
#define REG_USB20_HOST_SEL      0xc00000f0
#define REG_CLOCK_SELECTION     0xc0000018
#define CLOCK_SEL_PLL           8
#define REG_PMU_STATE           0xccb0b044
#define PMU_ON                  0x13

/*
 * TX_SEG: once the firmware runs, its checksum in [23:16]; the driver
 * acknowledges it with the complement, and the firmware posts its version
 */
#define REG_TX_SEG              0xc0000a10
#define FW_SUM_SEED             0x12345678
#define FW_VERSION_6006C        10097
#define FW_VERSION_6020         8089

/* eFuse: read done, progress done, the part's identity, then data words */
#define REG_EFUSE_RD_FLAG       0xc2000108
#define REG_EFUSE_DONE          0xc200010c
#define REG_EFUSE_ID            0xc2000120
#define REG_EFUSE_DATA          0xc2000124
#define EFUSE_ITEM_MAC          0xb
/* the 6020's: a window with the SKU on top and the MAC big-endian below */
#define REG_OTP_MAC             0xd9003fb8
#define REG_OTP_MAC_LO          0xd9003fbc
#define REG_OTP_SKU             0xd9003ffc

/* the TX pages and ids in use: none, as every frame leaves at once */
#define REG_TX_ID_ALL_INFO2     0xcd01006c

/*
 * The 6020 keeps its beacon in a packet buffer: the host asks for one with
 * a write to WR_ALC and reads its address back, fills it through the
 * register port, and names it to the MAC by id in MTX_BCN_PKT_SET0
 */
#define REG_WR_ALC              0xcd010000
#define REG_MTX_BCN_PKT_SET0    0xc6002088
#define PBUF_ID_MASK            0x7f
#define PBUF_ADDR(id)           (0xe0000000 | (id) << 16)

/*
 * Words 0 of what crosses the bus: the length, the content type, and for
 * a host command or event its id in the top byte
 */
#define MSG_LEN(w0)             ((w0) & 0xffff)
#define MSG_CTYPE(w0)           (((w0) >> 16) & 7)
#define MSG_ID(w0)              ((w0) >> 24)
#define CTYPE_TX_FRAME          2
#define CTYPE_RX_FRAME          3
#define CTYPE_HOST_CMD          5
#define CTYPE_EVENT             6

/* a host command: w0, sub-command, sequence, blocking tag, payload */
#define CMD_HDR_LEN             16
#define EVT_HDR_LEN             12

#define SSV_HOST_CMD_SOFT_BEACON 10
#define SSV_HOST_CMD_RFPHY_OPS  13
#define SSV_HOST_CMD_SECURITY   14
#define SSV_HOST_CMD_AMPDU_OPS  15
#define SSV_HOST_CMD_RC_OPS     16
#define SSV_HOST_CMD_TX_OPS     17
#define SSV_HOST_CMD_VIF_OPS    18
#define SSV_HOST_CMD_ADAPTIVE   21
#define SSV_HOST_CMD_ADAPTIVE_6020 24

#define RFPHY_CHAN              1
#define RFPHY_RF_ENABLE         2
#define RFPHY_RF_DISABLE        3

#define SOFT_BEACON_START       0
#define SOFT_BEACON_STOP        1
#define SOFT_BEACON_FILL        3

#define SOC_EVT_FW_NOTIFY       28

/*
 * The 80-byte descriptor ahead of a frame each way: w0 as above, the
 * header offset in w2 [7:0]; on RX the channel in w3 [31:24], the rate
 * in w4 [23:16], the signal in w5 [23:16], and the length again in w9
 */
#define DESC_LEN                80
#define RXD_W3_WSID_NONE        (0xf << 19)
#define RX_TRAILER_LEN          4
#define RX_RATE_1M              0x00
#define RX_RATE_54M             0x87
#define RX_RSSI                 45
#define RX_SNR                  30

#define BEACON_MS               102
#define HEARTBEAT_MS            500

static uint32_t ssv_reg(SsvCore *c, uint32_t addr)
{
    return GPOINTER_TO_UINT(g_hash_table_lookup(c->regs,
                            GUINT_TO_POINTER(addr)));
}

static void ssv_queue(SsvCore *c, GByteArray *m)
{
    g_queue_push_tail(&c->out, m);
    c->notify(c);
}

GByteArray *ssv_core_peek(SsvCore *c)
{
    return g_queue_peek_head(&c->out);
}

GByteArray *ssv_core_pop(SsvCore *c)
{
    GByteArray *m;

    if (c->chip == SSV_CHIP_6030) {
        return ssv_fmac_pop(c);
    }
    m = g_queue_pop_head(&c->out);

    if (m && g_queue_get_length(&c->out) == SSV_OUT_QUEUE_MAX - 1) {
        qemu_flush_queued_packets(qemu_get_queue(c->nic));
        if (c->client_nic) {
            qemu_flush_queued_packets(qemu_get_queue(c->client_nic));
        }
    }
    return m;
}

static void ssv_flush(SsvCore *c)
{
    GByteArray *m;

    while ((m = g_queue_pop_head(&c->out))) {
        g_byte_array_unref(m);
    }
}

/* An event: the header, then its data */
static void ssv_event(SsvCore *c, uint8_t id, uint32_t blocking,
                      const uint8_t *data, size_t len)
{
    GByteArray *e = g_byte_array_sized_new(EVT_HDR_LEN + len);
    uint8_t h[EVT_HDR_LEN] = {};

    stl_le_p(h, (EVT_HDR_LEN + len) | CTYPE_EVENT << 16 | id << 24);
    stl_le_p(h + 8, blocking);
    g_byte_array_append(e, h, sizeof(h));
    g_byte_array_append(e, data, len);
    ssv_queue(c, e);
}

/*
 * The firmware's heartbeat: a status count that moves. Without one for
 * two seconds the driver takes the firmware for hung and loads it again.
 */
static void ssv_heartbeat(void *opaque)
{
    SsvCore *c = opaque;
    /* rate update, a rate per station, status count update, the count */
    uint8_t n[15] = {};
    int stations = c->chip == SSV_CHIP_6020 ? 9 : 8;

    n[1 + stations] = 1;
    stl_le_p(n + 2 + stations, ++c->heartbeats);
    ssv_event(c, SOC_EVT_FW_NOTIFY, 0, n, 6 + stations);
    timer_mod(c->heartbeat, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                            HEARTBEAT_MS);
}

/*
 * A frame off the air: the descriptor, the MPDU and a trailer. One the
 * air carried encrypted comes decrypted, its CCMP header and MIC gone and
 * Protected still set, as the driver expects of the chip.
 */
static void ssv_rx_frame(SsvCore *c, const uint8_t *frame, size_t len,
                         bool protected)
{
    size_t total = DESC_LEN + len + RX_TRAILER_LEN;
    uint8_t d[DESC_LEN] = {};
    uint8_t pad[RX_TRAILER_LEN + 4] = {};
    uint16_t fc;
    GByteArray *u;

    if (len < WIFI_HDR_LEN || !c->rf_on ||
        g_queue_get_length(&c->out) >= SSV_OUT_QUEUE_MAX) {
        return;
    }
    fc = lduw_le_p(frame);
    stl_le_p(d, total | CTYPE_RX_FRAME << 16);
    stl_le_p(d + 8, DESC_LEN);
    stl_le_p(d + 12, RXD_W3_WSID_NONE | c->channel << 24);
    stl_le_p(d + 16, len | (WIFI_FC_TYPE(fc) == WIFI_TYPE_DATA ?
                            RX_RATE_54M : RX_RATE_1M) << 16);
    stl_le_p(d + 20, RX_RSSI << 16 | RX_SNR << 24);
    stl_le_p(d + 36, total);
    u = g_byte_array_sized_new(total + sizeof(pad));
    g_byte_array_append(u, d, sizeof(d));
    g_byte_array_append(u, frame, len);
    if (protected) {
        stw_le_p(u->data + DESC_LEN, fc | WIFI_FC_PROTECTED);
    }
    /* the trailer, out to a word; a transfer of whole packets gets a byte */
    g_byte_array_append(u, pad, RX_TRAILER_LEN + (-total & 3));
    if (u->len % 512 == 0) {
        g_byte_array_append(u, pad, 1);
    }
    trace_ssv_rx(fc, len);
    ssv_queue(c, u);
}

/* From the access point, heard on its channel */
static void ssv_deliver(void *opaque, const uint8_t *frame, size_t len,
                        bool protected)
{
    SsvCore *c = opaque;

    if (c->channel == c->ap.channel) {
        ssv_rx_frame(c, frame, len, protected);
    }
}

/* From the client, on the channel of the host's own BSS */
static void ssv_client_deliver(void *opaque, const uint8_t *frame,
                               size_t len, bool protected)
{
    SsvCore *c = opaque;

    if (c->bss_up) {
        ssv_rx_frame(c, frame, len, protected);
    }
}

static void ssv_beacon_tick(void *opaque)
{
    SsvCore *c = opaque;
    uint8_t b[WIFI_AP_BEACON_MAX];

    ssv_deliver(c, b, wifi_ap_beacon(&c->ap, b), false);
    timer_mod(c->beacon, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
}

/* Tell the client whether there is an access point to join */
static void ssv_bss_update(SsvCore *c)
{
    if (!c->client_nic) {
        return;
    }
    if (c->bss_up) {
        wifi_client_bss(&c->client, c->bssid.a, c->bss_ssid,
                        c->bss_ssid_len, NULL);
    } else {
        wifi_client_bss(&c->client, NULL, NULL, 0, NULL);
    }
}

void ssv_core_set_bss(SsvCore *c, bool up)
{
    if (c->bss_up != up) {
        trace_ssv_bss(up);
        c->bss_up = up;
        ssv_bss_update(c);
    }
}

/* The host's beacon for its access point: the BSSID and the SSID */
void ssv_core_beacon(SsvCore *c, const uint8_t *f, size_t flen)
{
    const size_t ie = WIFI_HDR_LEN + 12;

    if (flen < ie + 2 || lduw_le_p(f) != WIFI_FC_MGMT(WIFI_STYPE_BEACON) ||
        f[ie] != WIFI_EID_SSID || ie + 2 + f[ie + 1] > flen) {
        return;
    }
    memcpy(c->bssid.a, f + 16, 6);
    c->bss_ssid_len = MIN(f[ie + 1], sizeof(c->bss_ssid));
    memcpy(c->bss_ssid, f + ie + 2, c->bss_ssid_len);
    c->bcn_valid = true;
    if (c->bss_up) {
        ssv_bss_update(c);
    }
}

/*
 * The 6020's beacon, from the packet buffer the host filled: a descriptor
 * with the frame's length in its first word, then the frame
 */
static void ssv_pbuf_beacon(SsvCore *c, uint32_t addr)
{
    uint8_t f[512];
    size_t flen = MIN(ssv_reg(c, addr) & 0xffff, sizeof(f));

    for (size_t i = 0; i < flen; i += 4) {
        stl_le_p(f + i, ssv_reg(c, addr + DESC_LEN + i));
    }
    ssv_core_beacon(c, f, flen);
}

/*
 * SOFT_BEACON: the firmware sends the host's beacon for its access point.
 * The 6006C's FILL_CONTEXT brings the beacon: its length, descriptor
 * included, in the top half of the third word, the descriptor 12 bytes in.
 */
static void ssv_soft_beacon(SsvCore *c, uint32_t sub, const uint8_t *p,
                            size_t len)
{
    size_t flen;

    switch (sub) {
    case SOFT_BEACON_STOP:
        ssv_core_set_bss(c, false);
        break;
    case SOFT_BEACON_START:
        ssv_core_set_bss(c, c->bcn_valid);
        break;
    case SOFT_BEACON_FILL:
        if (len < 12 + DESC_LEN) {
            return;
        }
        flen = MIN(ldl_le_p(p + 8) >> 16, len - 12);
        ssv_core_beacon(c, p + 12 + DESC_LEN,
                        flen > DESC_LEN ? flen - DESC_LEN : 0);
        break;
    }
}

static void ssv_rfphy(SsvCore *c, uint32_t sub, const uint8_t *p,
                      size_t len)
{
    switch (sub) {
    case RFPHY_CHAN:
        if (len >= 2) {
            uint8_t ch = lduw_le_p(p);

            if (ch != c->channel) {
                trace_ssv_channel(c->channel, ch);
                c->channel = ch;
                /* a dwell on the channel catches a beacon */
                if (ch == c->ap.channel) {
                    timer_mod(c->beacon,
                              qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 1);
                }
            }
        }
        break;
    case RFPHY_RF_ENABLE:
        c->rf_on = true;
        break;
    case RFPHY_RF_DISABLE:
        c->rf_on = false;
        break;
    }
}

/*
 * A blocking command waits for the event of its family; the 6020's
 * firmware numbers the adaptive test command and event further on
 */
static uint8_t ssv_reply_id(SsvCore *c, uint8_t cmd)
{
    static const uint8_t replies[][2] = {
        { SSV_HOST_CMD_RFPHY_OPS, 20 },
        { SSV_HOST_CMD_SECURITY, 23 },
        { SSV_HOST_CMD_AMPDU_OPS, 24 },
        { SSV_HOST_CMD_RC_OPS, 25 },
        { SSV_HOST_CMD_TX_OPS, 26 },
        { SSV_HOST_CMD_VIF_OPS, 27 },
    };

    for (size_t i = 0; i < ARRAY_SIZE(replies); i++) {
        if (replies[i][0] == cmd) {
            return replies[i][1];
        }
    }
    if (c->chip == SSV_CHIP_6020) {
        return cmd == SSV_HOST_CMD_ADAPTIVE_6020 ? 38 : 0;
    }
    return cmd == SSV_HOST_CMD_ADAPTIVE ? 32 : 0;
}

static void ssv_host_cmd(SsvCore *c, const uint8_t *buf, size_t len)
{
    uint8_t cmd = MSG_ID(ldl_le_p(buf)), reply;
    uint32_t sub = ldl_le_p(buf + 4), blocking = ldl_le_p(buf + 12);
    const uint8_t *p = buf + CMD_HDR_LEN;
    size_t plen = len - CMD_HDR_LEN;

    trace_ssv_cmd(cmd, sub, blocking);
    switch (cmd) {
    case SSV_HOST_CMD_RFPHY_OPS:
        ssv_rfphy(c, sub, p, plen);
        break;
    case SSV_HOST_CMD_SOFT_BEACON:
        ssv_soft_beacon(c, sub, p, plen);
        break;
    }
    reply = ssv_reply_id(c, cmd);
    if (blocking && reply) {
        ssv_event(c, reply, blocking, NULL, 0);
    }
}

/*
 * A frame for the air, its body in the clear whatever Protected says. The
 * client hears what the host's own BSS sends, the access point the rest
 * on its channel.
 */
static void ssv_tx_frame(SsvCore *c, const uint8_t *buf, size_t len)
{
    size_t off = buf[8];
    g_autofree uint8_t *f = NULL;
    uint16_t fc;

    if (off < DESC_LEN || len < off + WIFI_HDR_LEN || !c->rf_on) {
        return;
    }
    len -= off;
    f = g_memdup2(buf + off, len);
    fc = lduw_le_p(f);
    stw_le_p(f, fc & ~WIFI_FC_PROTECTED);
    trace_ssv_tx(fc, len);
    if (c->bss_up && !memcmp(f + 10, c->bssid.a, 6)) {
        if (c->client_nic) {
            wifi_client_receive(&c->client, f, len);
        }
    } else if (c->channel == c->ap.channel) {
        wifi_ap_receive(&c->ap, f, len);
    }
}

void ssv_core_host(SsvCore *c, const uint8_t *buf, size_t len)
{
    uint32_t w0;

    if (c->chip == SSV_CHIP_6030) {
        ssv_fmac_host(c, buf, len);
        return;
    }
    if (len < 4) {
        return;
    }
    w0 = ldl_le_p(buf);
    len = MIN(len, MSG_LEN(w0));
    switch (MSG_CTYPE(w0)) {
    case CTYPE_HOST_CMD:
        if (len >= CMD_HDR_LEN && c->fw_up) {
            ssv_host_cmd(c, buf, len);
        }
        break;
    case CTYPE_TX_FRAME:
        if (len >= DESC_LEN && c->fw_up) {
            ssv_tx_frame(c, buf, len);
        }
        break;
    }
}

/* The firmware starts: it sums what it was given and posts the sum */
static void ssv_fw_boot(SsvCore *c)
{
    uint32_t s = c->fw_sum;
    uint8_t sum = (s >> 24) + (s >> 16) + (s >> 8) + s;

    c->fw_status = sum << 16;
    c->fw_booting = true;
    trace_ssv_fw_boot(sum);
}

/* The CPU leaves reset into the boot ROM or the firmware */
static void ssv_cpu_start(SsvCore *c, uint16_t ivb)
{
    if (ivb == IVB_ROM) {
        c->fw_sum = FW_SUM_SEED;
        c->fw_booting = false;
        c->fw_up = false;
        c->rf_on = false;
        timer_del(c->heartbeat);
        ssv_core_set_bss(c, false);
        c->bcn_valid = false;
    } else {
        ssv_fw_boot(c);
    }
}

void ssv_core_fw_write(SsvCore *c, uint32_t addr, const uint8_t *buf,
                       size_t len)
{
    /* the SSV6030 takes its firmware on trust */
    if (c->chip == SSV_CHIP_6030) {
        return;
    }
    if (!addr) {
        c->fw_sum = FW_SUM_SEED;
    }
    if (c->chip == SSV_CHIP_6020 && (addr & 0xfff00000) == FW_DLM_6020) {
        return;
    }
    for (size_t i = 0; i + 4 <= len; i += 4) {
        c->fw_sum += ldl_le_p(buf + i);
    }
}

/*
 * The 6006C's eFuse: the part's identity, then a stream of nibbles, low
 * nibble first, each item its id and then its value; the MAC is item 11
 */
static uint32_t ssv_efuse(SsvCore *c, uint32_t addr)
{
    uint64_t stream = EFUSE_ITEM_MAC;

    if (addr == REG_EFUSE_ID) {
        return c->efuse_id;
    }
    for (int i = 0; i < 6; i++) {
        stream |= (uint64_t)c->conf.macaddr.a[i] << (4 + 8 * i);
    }
    if (addr == REG_EFUSE_DATA) {
        return stream;
    }
    return addr == REG_EFUSE_DATA + 4 ? stream >> 32 : 0;
}

/* The 6020's: the SKU, then the MAC in the top six bytes of two words */
static uint32_t ssv_otp(SsvCore *c, uint32_t addr)
{
    const uint8_t *m = c->conf.macaddr.a;

    switch (addr) {
    case REG_OTP_SKU:
        return c->efuse_id;
    case REG_OTP_MAC:
        return ldl_be_p(m);
    case REG_OTP_MAC_LO:
        return lduw_be_p(m + 4) << 16;
    }
    return ssv_reg(c, addr);
}

uint32_t ssv_core_reg_read(SsvCore *c, uint32_t addr)
{
    /* "SSV6006C0" and "SSV6020C0", spaces after, a register from the top */
    static const uint32_t chip_id[][4] = {
        [SSV_CHIP_6006C] = { 0x20202020, 0x30202020, 0x30303643,
                             0x53535636 },
        [SSV_CHIP_6020] = { 0x20202020, 0x30202020, 0x30323043,
                            0x53535636 },
    };
    bool is_6020 = c->chip == SSV_CHIP_6020;
    uint32_t v = ssv_reg(c, addr);

    if (c->chip == SSV_CHIP_6030) {
        return ssv_fmac_reg_read(c, addr);
    }
    if (addr >= REG_CHIP_ID_0 && addr <= REG_CHIP_ID_3 && !(addr & 3)) {
        return chip_id[c->chip][(addr - REG_CHIP_ID_0) / 4];
    }
    if (is_6020 && addr >= REG_OTP_MAC && addr <= REG_OTP_SKU) {
        return ssv_otp(c, addr);
    }
    if (!is_6020 && addr >= REG_EFUSE_ID && addr < REG_EFUSE_DATA + 7 * 4) {
        return ssv_efuse(c, addr);
    }
    switch (addr) {
    case REG_CHIP_TYPE_VER:
        return CHIP_TYPE_ASIC;
    case REG_BRG_SW_RST:
    case REG_TX_ID_ALL_INFO2:
        return 0;
    case REG_N10_DBG2:
        return v | N10_STANDBY;
    case REG_SYSCTRL_STATUS:
        return is_6020 ? SYSCTRL_ON_6020 : SYSCTRL_ON_6006C;
    case REG_SRAM_PG_STATUS0:
        return is_6020 ? ssv_reg(c, REG_SRAM_PG_CTRL0) : v;
    case REG_SRAM_PG_STATUS1:
        return is_6020 ? ssv_reg(c, REG_SRAM_PG_CTRL1) : v;
    case REG_USB20_HOST_SEL:
    case REG_EFUSE_RD_FLAG:
    case REG_EFUSE_DONE:
        return v | 1;
    case REG_PMU_STATE:
        return PMU_ON;
    case REG_TX_SEG:
        return c->fw_status;
    }
    return v;
}

void ssv_core_reg_write(SsvCore *c, uint32_t addr, uint32_t val)
{
    uint32_t old;

    if (c->chip == SSV_CHIP_6030) {
        ssv_fmac_reg_write(c, addr, val);
        return;
    }
    old = ssv_core_reg_read(c, addr);
    g_hash_table_insert(c->regs, GUINT_TO_POINTER(addr),
                        GUINT_TO_POINTER(val));
    switch (addr) {
    case REG_PLATFORM_CLOCK_EN:
        if (c->chip == SSV_CHIP_6006C && !(old & RESET_N_CPUN10) &&
            (val & RESET_N_CPUN10)) {
            ssv_cpu_start(c, ssv_reg(c, REG_IVB));
        }
        break;
    case REG_SYSCTRL_COMMAND:
        if (c->chip == SSV_CHIP_6020 &&
            (val == SYSCTRL_REBOOT || val == SYSCTRL_REBOOT_STANDBY)) {
            ssv_cpu_start(c, ssv_reg(c, REG_N10CFG_SETTING) >> 16);
        }
        break;
    case REG_WR_ALC:
        /* any size fits; the next id, its address read back */
        if (c->chip == SSV_CHIP_6020) {
            c->pbuf_id = (c->pbuf_id + 1) & PBUF_ID_MASK;
            g_hash_table_insert(c->regs, GUINT_TO_POINTER(addr),
                                GUINT_TO_POINTER(PBUF_ADDR(c->pbuf_id)));
        }
        break;
    case REG_MTX_BCN_PKT_SET0:
        if (c->chip == SSV_CHIP_6020) {
            ssv_pbuf_beacon(c, PBUF_ADDR(val & PBUF_ID_MASK));
        }
        break;
    case REG_TX_SEG:
        /* the sum acknowledged: the firmware is up, and says which */
        if (c->fw_booting && val == (~c->fw_status & 0x00ff0000)) {
            c->fw_status = c->chip == SSV_CHIP_6020 ? FW_VERSION_6020
                                                    : FW_VERSION_6006C;
            c->fw_booting = false;
            c->fw_up = true;
            c->channel = c->ap.channel;
            timer_mod(c->heartbeat, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                                    HEARTBEAT_MS);
        } else {
            c->fw_status = val;
        }
        break;
    }
}

void ssv_core_reset(SsvCore *c)
{
    g_hash_table_remove_all(c->regs);
    g_hash_table_insert(c->regs, GUINT_TO_POINTER(REG_CLOCK_SELECTION),
                        GUINT_TO_POINTER(CLOCK_SEL_PLL));
    g_hash_table_insert(c->regs, GUINT_TO_POINTER(REG_IVB),
                        GUINT_TO_POINTER(IVB_ROM));
    g_hash_table_insert(c->regs, GUINT_TO_POINTER(REG_PLATFORM_CLOCK_EN),
                        GUINT_TO_POINTER(RESET_N_CPUN10));
    timer_del(c->heartbeat);
    ssv_flush(c);
    c->fw_sum = FW_SUM_SEED;
    c->fw_booting = false;
    c->fw_up = false;
    c->fw_status = 0;
    c->rf_on = false;
    c->channel = c->ap.channel;
    c->bcn_valid = false;
    c->bss_up = false;
    c->pbuf_id = 0;
    ssv_bss_update(c);
    if (c->chip == SSV_CHIP_6030) {
        ssv_fmac_reset(c);
    }
}

static bool ssv_can_receive(NetClientState *nc)
{
    SsvCore *c = qemu_get_nic_opaque(nc);

    return g_queue_get_length(&c->out) < SSV_OUT_QUEUE_MAX;
}

static ssize_t ssv_receive(NetClientState *nc, const uint8_t *buf,
                           size_t size)
{
    SsvCore *c = qemu_get_nic_opaque(nc);

    wifi_ap_net_receive(&c->ap, buf, size);
    return size;
}

static NetClientInfo ssv_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ssv_can_receive,
    .receive = ssv_receive,
};

static bool ssv_client_can_receive(NetClientState *nc)
{
    SsvCore *c = qemu_get_nic_opaque(nc);

    return wifi_client_can_receive(&c->client) &&
           g_queue_get_length(&c->out) < SSV_OUT_QUEUE_MAX;
}

static ssize_t ssv_client_receive(NetClientState *nc, const uint8_t *buf,
                                  size_t size)
{
    SsvCore *c = qemu_get_nic_opaque(nc);

    wifi_client_net_receive(&c->client, buf, size);
    return size;
}

static NetClientInfo ssv_client_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ssv_client_can_receive,
    .receive = ssv_client_receive,
};

void ssv_core_init(SsvCore *c)
{
    c->regs = g_hash_table_new(g_direct_hash, g_direct_equal);
    c->beacon = timer_new_ms(QEMU_CLOCK_VIRTUAL, ssv_beacon_tick, c);
    c->heartbeat = timer_new_ms(QEMU_CLOCK_VIRTUAL, ssv_heartbeat, c);
    g_queue_init(&c->out);
    if (c->chip == SSV_CHIP_6030) {
        ssv_fmac_init(c);
    }
}

bool ssv_core_realize(SsvCore *c, Error **errp)
{
    DeviceState *dev = c->dev;
    bool fmac = c->chip == SSV_CHIP_6030;

    qemu_macaddr_default_if_unset(&c->conf.macaddr);
    c->nic = qemu_new_nic(&ssv_net_info, &c->conf,
                          object_get_typename(OBJECT(dev)), dev->id,
                          &dev->mem_reentrancy_guard, c);
    if (!wifi_ap_realize(&c->ap, c->nic, fmac ? ssv_fmac_deliver : ssv_deliver,
                         c, errp)) {
        qemu_del_nic(c->nic);
        c->nic = NULL;
        return false;
    }
    if (c->client_conf.peers.ncs[0]) {
        qemu_macaddr_default_if_unset(&c->client_conf.macaddr);
        c->client_nic = qemu_new_nic(&ssv_client_net_info, &c->client_conf,
                                     object_get_typename(OBJECT(dev)), NULL,
                                     &dev->mem_reentrancy_guard, c);
        wifi_client_init(&c->client, c->client_nic, &c->client_conf.macaddr,
                         fmac ? ssv_fmac_client_deliver : ssv_client_deliver,
                         c);
    }
    ssv_core_reset(c);
    /* a FullMAC scans by itself; the others hear the beacons */
    if (!fmac) {
        timer_mod(c->beacon,
                  qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + BEACON_MS);
    }
    return true;
}

void ssv_core_unrealize(SsvCore *c)
{
    timer_del(c->beacon);
    timer_del(c->heartbeat);
    if (c->chip == SSV_CHIP_6030) {
        timer_del(c->fmac.scan);
        timer_del(c->fmac.join);
    }
    if (c->client_nic) {
        wifi_client_cleanup(&c->client);
        qemu_del_nic(c->client_nic);
        c->client_nic = NULL;
    }
    wifi_ap_unrealize(&c->ap);
    qemu_del_nic(c->nic);
}

void ssv_core_finalize(SsvCore *c)
{
    if (c->chip == SSV_CHIP_6030) {
        ssv_fmac_finalize(c);
    }
    timer_free(c->beacon);
    timer_free(c->heartbeat);
    ssv_flush(c);
    g_hash_table_destroy(c->regs);
}
