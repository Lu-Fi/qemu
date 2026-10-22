/*
 * Broadcom BCM43438 and BCM43436 Wi-Fi on SDIO
 *
 * Function 1 reaches the chip's backplane through a 32 KiB window, and
 * with address bit 16 a few registers of the SDIO core's own; function 2
 * is the packet FIFO, which carries SDPCM frames each way. The dongle
 * firmware behind them is bcm-wifi.c. The chip asks for service on a pin
 * of its own, the host-wake GPIO, and WL_REG_ON, a GPIO the other way,
 * powers it.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/gpio/ingenic-gpio.h"
#include "hw/sd/sdio.h"
#include "migration/vmstate.h"
#include "qom/object.h"
#include "bcm-wifi.h"
#include "trace.h"

#define TYPE_BCM_SDIO "bcm-sdio"
OBJECT_DECLARE_TYPE(BcmSdioState, BcmSdioClass, BCM_SDIO)

/*
 * Function 1 registers, at bit 16: the backplane window (address bits 15,
 * 23-16 and 31-24), the clock control and status, the sleep control
 */
#define F1_REGS                 0x10000
#define F1_REGS_SIZE            0x20
#define F1_FRAMECTRL            0x0d
#define F1_CHIPCLKCSR           0x0e
#define F1_SBADDRLOW            0x0a
#define F1_SBADDRMID            0x0b
#define F1_SBADDRHIGH           0x0c
#define F1_SLEEPCSR             0x1f

#define FRAMECTRL_RF_TERM       BIT(0)
#define FRAMECTRL_WF_TERM       BIT(1)
#define CLK_FORCE_ALP           BIT(0)
#define CLK_FORCE_HT            BIT(1)
#define CLK_ALP_REQ             BIT(3)
#define CLK_HT_REQ              BIT(4)
#define CLK_ALP_AVAIL           BIT(6)
#define CLK_HT_AVAIL            BIT(7)
#define SLEEPCSR_KSO            BIT(0)
#define SLEEPCSR_DEVON          BIT(1)

/* the window's 32 KiB, and the flag that makes an access 32 bits wide */
#define WINDOW_MASK             0x7fff
#define WINDOW_DEFAULT          0x18000000

/* CCCR 0xf2: the interrupt goes to its own pin, driven, active high */
#define CCCR_SEPINT             0xf2
#define SEPINT_MASK             BIT(0)
#define SEPINT_OE               BIT(1)
#define SEPINT_ACT_HI           BIT(2)

/*
 * The backplane: 512 KiB of RAM at 0, the cores from 0x18000000 a page
 * apiece, each with its wrapper 1 MiB above, and the enumeration ROM
 */
#define RAM_SIZE                0x80000
#define CORE_BASE               0x18000000
#define WRAP_BASE               0x18100000
#define NCORES                  5
#define CORE_CC                 0
#define CORE_SDIOD              2
#define CORE_ARM                3
#define CORE_SOCRAM             4
#define CORE_REGS               (0x1000 / 4)
#define EROM_BASE               0x18109000

#define CC_CHIPID               0x00
#define CC_EROMPTR              0xfc

#define SDIOD_INTSTATUS         0x20
#define SDIOD_HOSTINTMASK       0x24
#define SDIOD_TOSBMAILBOX       0x40
#define SDIOD_TOHOSTMAILBOXDATA 0x4c
#define I_HMB_FRAME_IND         BIT(6)
#define I_HMB_HOST_INT          BIT(7)
#define SMB_NAK                 BIT(0)
/* to the host: the NAK handled, the device and firmware ready, version 4 */
#define HMB_DATA_NAKHANDLED     BIT(0)
#define HMB_DATA_DEVREADY       BIT(1)
#define HMB_DATA_FWREADY        BIT(3)
#define SDPCM_PROT_VERSION      4

/* SOCRAM: 8 banks of 64 KiB */
#define SOCRAM_COREINFO         0x00
#define SOCRAM_BANKINFO         0x40
#define SOCRAM_NBANKS           8
#define SOCRAM_BANK_64K         7

#define WRAP_IOCTRL             0x408
#define WRAP_RESETCTRL          0x800
#define IOCTRL_CLK_EN           BIT(0)
#define RESETCTRL_RESET         BIT(0)

/*
 * The last word of RAM: the NVRAM's length token on the way in, the
 * address of the firmware's sdpcm_shared_t once it is up
 */
#define RAM_LAST_WORD           (RAM_SIZE - 4)
#define SDPCM_SHARED_LEN        32
#define SDPCM_SHARED_VERSION    1

/*
 * SDPCM: a hardware tag (length, its complement), then sequence, channel,
 * next length, data offset, flow control, the window of sequences the
 * host may use, and the version
 */
#define SDPCM_HDR_LEN           12
#define SDPCM_CREDITS           32
/* refresh the host's window this short of running out */
#define SDPCM_CREDIT_LOW        16
#define F2_MAX                  (16 * 1024)

typedef struct BcmSdioChip {
    const char *name;
    const char *desc;
    uint32_t chipid;
} BcmSdioChip;

struct BcmSdioClass {
    SDIOCardClass parent_class;

    ResettablePhases parent_phases;
    const BcmSdioChip *chip;
};

struct BcmSdioState {
    SDIOCard parent_obj;

    BcmCore core;
    DeviceState *gpio;
    int32_t host_wake_pin;
    int32_t reg_on_pin;
    qemu_irq host_wake;

    uint8_t f1[F1_REGS_SIZE];
    uint8_t vendor_cccr[16];
    uint32_t regs[NCORES][CORE_REGS];
    uint32_t wrap[NCORES][CORE_REGS];
    uint8_t *ram;
    bool fw_up;

    /* F2: the frame the host is writing, the one it is reading */
    GByteArray *tx;
    GByteArray *rx;
    size_t rx_off;
    GByteArray *last;
    uint8_t rx_seq;
    uint8_t host_next;
    uint8_t window;
    bool credit_due;
};

/*
 * The enumeration ROM: ChipCommon, the 802.11 core, the SDIO device core,
 * the Cortex-M3 and SOCRAM, each with its slave port and wrapper, SOCRAM
 * with a second port for its 512 KiB at 0
 */
static const uint32_t bcm_erom[] = {
    0x4bf800d1, 0x2b080201, 0x18000005, 0x18100085,
    0x4bf812d1, 0x28004211, 0x00000103, 0x18001005, 0x181010c5,
    0x4bf829d1, 0x15004211, 0x00000103, 0x18002005, 0x181020c5,
    0x4bf82ad1, 0x08004211, 0x00000103, 0x18003005, 0x181030c5,
    0x4bf80ed1, 0x11080401, 0x18004005, 0x00000135, 0x00080000, 0x18104185,
    0x0000000f,
};

/* A1 and B0: chip 43430, 5 cores, package 4, revision 1 or 2 */
static const BcmSdioChip bcm_sdio_chips[] = {
    { "bcm43438", "Broadcom BCM43438 SDIO Wi-Fi", 0x1541a9a6 },
    { "bcm43436", "Broadcom BCM43436 SDIO Wi-Fi", 0x1542a9a6 },
};

static uint32_t *bcm_sdiod(BcmSdioState *s, uint32_t reg)
{
    return &s->regs[CORE_SDIOD][reg / 4];
}

/* The host-wake pin: driven once enabled, at the level CCCR 0xf2 asks */
static void bcm_sdio_wake(BcmSdioState *s)
{
    uint8_t sepint = s->vendor_cccr[CCCR_SEPINT & 0xf];
    bool pending = *bcm_sdiod(s, SDIOD_INTSTATUS) &
                   *bcm_sdiod(s, SDIOD_HOSTINTMASK);
    bool level = false;

    if ((sepint & SEPINT_MASK) && (sepint & SEPINT_OE)) {
        level = (sepint & SEPINT_ACT_HI) ? pending : !pending;
    }
    trace_bcm_sdio_wake(level);
    qemu_set_irq(s->host_wake, level);
}

static void bcm_sdio_raise(BcmSdioState *s, uint32_t bits)
{
    *bcm_sdiod(s, SDIOD_INTSTATUS) |= bits;
    bcm_sdio_wake(s);
}

static void bcm_sdio_notify(BcmCore *c)
{
    BcmSdioState *s = container_of(c, BcmSdioState, core);

    bcm_sdio_raise(s, I_HMB_FRAME_IND);
}

/* The firmware is gone: nothing queued, nothing half read */
static void bcm_sdio_halt(BcmSdioState *s)
{
    s->fw_up = false;
    bcm_core_reset(&s->core);
    g_byte_array_set_size(s->tx, 0);
    g_clear_pointer(&s->rx, g_byte_array_unref);
    g_clear_pointer(&s->last, g_byte_array_unref);
    s->rx_seq = 0;
    s->host_next = 0;
    s->window = SDPCM_CREDITS;
    s->credit_due = false;
}

/*
 * The Cortex-M3 leaves reset with a firmware in RAM. It leaves its shared
 * structure below the NVRAM, points the last word of RAM at it, and tells
 * the host it is ready.
 */
static void bcm_sdio_boot(BcmSdioState *s)
{
    uint32_t token = ldl_le_p(s->ram + RAM_LAST_WORD);
    uint32_t nvram = RAM_LAST_WORD;
    uint32_t shared;

    if (!ldl_le_p(s->ram + 4)) {
        return;
    }
    if (((~token >> 16) & 0xffff) == (token & 0xffff)) {
        nvram -= MIN((token & 0xffff) * 4, RAM_LAST_WORD / 2);
    }
    shared = (nvram - SDPCM_SHARED_LEN) & ~3;
    memset(s->ram + shared, 0, SDPCM_SHARED_LEN);
    stl_le_p(s->ram + shared, SDPCM_SHARED_VERSION);
    stl_le_p(s->ram + RAM_LAST_WORD, shared);
    trace_bcm_sdio_boot(shared);
    s->fw_up = true;
    *bcm_sdiod(s, SDIOD_TOHOSTMAILBOXDATA) = HMB_DATA_DEVREADY |
                                             HMB_DATA_FWREADY |
                                             SDPCM_PROT_VERSION << 16;
    bcm_sdio_raise(s, I_HMB_HOST_INT);
}

/* Power on, or WL_REG_ON pulled: the chip as it comes out of reset */
static void bcm_sdio_chip_reset(BcmSdioState *s)
{
    memset(s->f1, 0, sizeof(s->f1));
    s->f1[F1_SBADDRHIGH] = WINDOW_DEFAULT >> 24;
    memset(s->vendor_cccr, 0, sizeof(s->vendor_cccr));
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->wrap, 0, sizeof(s->wrap));
    for (int i = 0; i < NCORES; i++) {
        s->wrap[i][WRAP_IOCTRL / 4] = IOCTRL_CLK_EN;
    }
    s->wrap[CORE_ARM][WRAP_RESETCTRL / 4] = RESETCTRL_RESET;
    memset(s->ram, 0, RAM_SIZE);
    bcm_sdio_halt(s);
    bcm_sdio_wake(s);
}

static void bcm_sdio_reg_on(void *opaque, int line, int level)
{
    BcmSdioState *s = opaque;

    trace_bcm_sdio_power(level);
    if (!level) {
        bcm_sdio_chip_reset(s);
    }
}

static uint32_t bcm_sdio_bp_read32(BcmSdioState *s, uint32_t addr)
{
    BcmSdioClass *bc = BCM_SDIO_GET_CLASS(s);
    uint32_t off = addr & 0xfff;
    unsigned core;

    if (addr >= EROM_BASE && addr < EROM_BASE + sizeof(bcm_erom)) {
        return bcm_erom[(addr - EROM_BASE) / 4];
    }
    if (addr >= WRAP_BASE && addr < WRAP_BASE + NCORES * 0x1000) {
        return s->wrap[(addr - WRAP_BASE) >> 12][off / 4];
    }
    if (addr < CORE_BASE || addr >= CORE_BASE + NCORES * 0x1000) {
        return 0;
    }
    core = (addr - CORE_BASE) >> 12;
    switch (core) {
    case CORE_CC:
        if (off == CC_CHIPID) {
            return bc->chip->chipid;
        }
        if (off == CC_EROMPTR) {
            return EROM_BASE;
        }
        break;
    case CORE_SOCRAM:
        if (off == SOCRAM_COREINFO) {
            return SOCRAM_NBANKS << 4;
        }
        if (off == SOCRAM_BANKINFO) {
            return SOCRAM_BANK_64K;
        }
        break;
    }
    return s->regs[core][off / 4];
}

/* A write of the bytes @mask covers */
static void bcm_sdio_bp_write32(BcmSdioState *s, uint32_t addr, uint32_t val,
                                uint32_t mask)
{
    uint32_t off = addr & 0xfff, old, *reg;
    unsigned core;

    if (addr >= WRAP_BASE && addr < WRAP_BASE + NCORES * 0x1000) {
        core = (addr - WRAP_BASE) >> 12;
        reg = &s->wrap[core][off / 4];
        old = *reg;
        *reg = (old & ~mask) | (val & mask);
        if (core == CORE_ARM && off == WRAP_RESETCTRL) {
            if ((old & RESETCTRL_RESET) && !(*reg & RESETCTRL_RESET)) {
                bcm_sdio_boot(s);
            } else if (!(old & RESETCTRL_RESET) &&
                       (*reg & RESETCTRL_RESET) && s->fw_up) {
                bcm_sdio_halt(s);
            }
        }
        return;
    }
    if (addr < CORE_BASE || addr >= CORE_BASE + NCORES * 0x1000) {
        return;
    }
    core = (addr - CORE_BASE) >> 12;
    reg = &s->regs[core][off / 4];
    if (core != CORE_SDIOD) {
        *reg = (*reg & ~mask) | (val & mask);
        return;
    }
    switch (off) {
    case SDIOD_INTSTATUS:
        *reg &= ~(val & mask);
        bcm_sdio_wake(s);
        return;
    case SDIOD_HOSTINTMASK:
        *reg = (*reg & ~mask) | (val & mask);
        bcm_sdio_wake(s);
        return;
    case SDIOD_TOSBMAILBOX:
        /* a frame went bad on the way: it comes again, as it was */
        if ((val & mask & SMB_NAK) && s->last) {
            g_clear_pointer(&s->rx, g_byte_array_unref);
            s->rx = g_steal_pointer(&s->last);
            s->rx_off = 0;
            *bcm_sdiod(s, SDIOD_TOHOSTMAILBOXDATA) = HMB_DATA_NAKHANDLED;
            bcm_sdio_raise(s, I_HMB_HOST_INT);
        }
        return;
    }
    *reg = (*reg & ~mask) | (val & mask);
}

static void bcm_sdio_bp_read(BcmSdioState *s, uint32_t addr, uint8_t *buf,
                             size_t len)
{
    size_t i = 0;

    if (addr < RAM_SIZE) {
        i = MIN(len, RAM_SIZE - addr);
        memcpy(buf, s->ram + addr, i);
    }
    for (; i < len; i++) {
        uint32_t a = addr + i;

        buf[i] = bcm_sdio_bp_read32(s, a & ~3) >> (8 * (a & 3));
    }
}

static void bcm_sdio_bp_write(BcmSdioState *s, uint32_t addr,
                              const uint8_t *buf, size_t len)
{
    size_t i = 0;

    if (addr < RAM_SIZE) {
        i = MIN(len, RAM_SIZE - addr);
        memcpy(s->ram + addr, buf, i);
    }
    while (i < len) {
        uint32_t a = addr + i, val = 0, mask = 0;

        do {
            val |= buf[i] << (8 * (a & 3));
            mask |= 0xffu << (8 * (a & 3));
            i++;
            a++;
        } while (i < len && (a & 3));
        bcm_sdio_bp_write32(s, (a - 1) & ~3, val, mask);
    }
}

static uint32_t bcm_sdio_window(BcmSdioState *s)
{
    return s->f1[F1_SBADDRHIGH] << 24 | s->f1[F1_SBADDRMID] << 16 |
           (s->f1[F1_SBADDRLOW] & 0x80) << 8;
}

static uint8_t bcm_sdio_f1_reg_read(BcmSdioState *s, uint32_t reg)
{
    uint8_t v = s->f1[reg];
    bool ht, alp;

    switch (reg) {
    case F1_CHIPCLKCSR:
        /* the clocks come at once, HT only with the firmware up */
        v &= ~(CLK_ALP_AVAIL | CLK_HT_AVAIL);
        ht = s->fw_up && (v & (CLK_FORCE_HT | CLK_HT_REQ));
        alp = ht || (v & (CLK_FORCE_ALP | CLK_ALP_REQ));
        return v | (alp ? CLK_ALP_AVAIL : 0) | (ht ? CLK_HT_AVAIL : 0);
    case F1_SLEEPCSR:
        return (v & SLEEPCSR_KSO) | (v & SLEEPCSR_KSO ? SLEEPCSR_DEVON : 0);
    case F1_FRAMECTRL:
        return 0;
    }
    /* the frame byte counts read 0: nothing is ever half moved */
    return reg >= 0x19 && reg <= 0x1c ? 0 : v;
}

static void bcm_sdio_f1_reg_write(BcmSdioState *s, uint32_t reg,
                                  uint8_t val)
{
    s->f1[reg] = val;
    if (reg != F1_FRAMECTRL) {
        return;
    }
    if (val & FRAMECTRL_RF_TERM) {
        /* the frame being read stops; a NAK may ask for it again */
        if (s->rx) {
            g_clear_pointer(&s->last, g_byte_array_unref);
            s->last = g_steal_pointer(&s->rx);
        }
    }
    if (val & FRAMECTRL_WF_TERM) {
        g_byte_array_set_size(s->tx, 0);
    }
}

/* An SDPCM frame for the host, stamped with the window as of now */
static GByteArray *bcm_sdio_frame(BcmSdioState *s, unsigned chan,
                                  const uint8_t *payload, size_t len)
{
    size_t total = SDPCM_HDR_LEN + len;
    GByteArray *f = g_byte_array_sized_new(total);
    uint8_t h[SDPCM_HDR_LEN] = {};

    s->window = s->host_next + SDPCM_CREDITS;
    stw_le_p(h, total);
    stw_le_p(h + 2, ~total);
    h[4] = s->rx_seq++;
    h[5] = chan;
    h[7] = SDPCM_HDR_LEN;
    h[9] = s->window;
    h[10] = SDPCM_PROT_VERSION;
    g_byte_array_append(f, h, sizeof(h));
    if (len) {
        g_byte_array_append(f, payload, len);
    }
    trace_bcm_sdio_frame_out(h[4], chan, total);
    return f;
}

/* The next frame for the host, if any: the dongle's, else a credit */
static void bcm_sdio_next(BcmSdioState *s)
{
    BcmMsg *m = bcm_core_pop(&s->core);

    if (m) {
        s->rx = bcm_sdio_frame(s, m->chan, m->data->data, m->data->len);
        bcm_msg_free(m);
    } else if (s->credit_due) {
        s->rx = bcm_sdio_frame(s, BCM_CHAN_DATA, NULL, 0);
    } else {
        return;
    }
    s->credit_due = false;
    s->rx_off = 0;
}

/*
 * The host reads a frame's first bytes, then the rest, rounded up; past
 * the end it reads zeros, and a zero header tells it the FIFO is empty
 */
static void bcm_sdio_f2_read(BcmSdioState *s, uint8_t *buf, size_t len)
{
    size_t n = 0;

    if (!s->rx && s->fw_up) {
        bcm_sdio_next(s);
    }
    if (s->rx) {
        n = MIN(len, s->rx->len - s->rx_off);
        memcpy(buf, s->rx->data + s->rx_off, n);
        s->rx_off += n;
        if (s->rx_off == s->rx->len) {
            g_clear_pointer(&s->last, g_byte_array_unref);
            s->last = g_steal_pointer(&s->rx);
        }
    }
    memset(buf + n, 0, len - n);
}

static void bcm_sdio_host_frame(BcmSdioState *s, const uint8_t *f,
                                size_t len)
{
    unsigned doff = f[7];

    trace_bcm_sdio_frame_in(f[4], f[5] & 0xf, len);
    s->host_next = f[4] + 1;
    if (doff >= SDPCM_HDR_LEN && doff <= len) {
        bcm_core_host(&s->core, f[5] & 0xf, f + doff, len - doff);
    }
    /* a host only sending has to hear of more room now and then */
    if ((uint8_t)(s->window - s->host_next) < SDPCM_CREDIT_LOW &&
        !s->rx && !bcm_core_peek(&s->core)) {
        s->credit_due = true;
        bcm_sdio_raise(s, I_HMB_FRAME_IND);
    }
}

/* A frame from the host: its length leads, padding fills out the write */
static void bcm_sdio_f2_write(BcmSdioState *s, const uint8_t *buf,
                              size_t len)
{
    size_t flen;

    if (s->tx->len + len > F2_MAX) {
        g_byte_array_set_size(s->tx, 0);
        return;
    }
    g_byte_array_append(s->tx, buf, len);
    if (s->tx->len < 4) {
        return;
    }
    flen = lduw_le_p(s->tx->data);
    if ((flen ^ lduw_le_p(s->tx->data + 2)) != 0xffff ||
        flen < SDPCM_HDR_LEN) {
        g_byte_array_set_size(s->tx, 0);
        return;
    }
    if (s->tx->len < flen) {
        return;
    }
    if (s->fw_up) {
        bcm_sdio_host_frame(s, s->tx->data, flen);
    }
    g_byte_array_set_size(s->tx, 0);
}

static void bcm_sdio_read(SDIOCard *card, unsigned fn, uint32_t addr,
                          bool incr, uint8_t *buf, size_t len)
{
    BcmSdioState *s = BCM_SDIO(card);

    if (fn == 2) {
        bcm_sdio_f2_read(s, buf, len);
        return;
    }
    if (addr >= F1_REGS) {
        for (size_t i = 0; i < len; i++) {
            uint32_t reg = addr - F1_REGS + (incr ? i : 0);

            buf[i] = reg < F1_REGS_SIZE ? bcm_sdio_f1_reg_read(s, reg) : 0;
        }
    } else {
        bcm_sdio_bp_read(s, bcm_sdio_window(s) | (addr & WINDOW_MASK), buf,
                         len);
    }
    trace_bcm_sdio_f1_read(addr, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
}

static void bcm_sdio_write(SDIOCard *card, unsigned fn, uint32_t addr,
                           bool incr, const uint8_t *buf, size_t len)
{
    BcmSdioState *s = BCM_SDIO(card);

    if (fn == 2) {
        bcm_sdio_f2_write(s, buf, len);
        return;
    }
    trace_bcm_sdio_f1_write(addr, len, len >= 4 ? ldl_le_p(buf) : buf[0]);
    if (addr >= F1_REGS) {
        for (size_t i = 0; i < len; i++) {
            uint32_t reg = addr - F1_REGS + (incr ? i : 0);

            if (reg < F1_REGS_SIZE) {
                bcm_sdio_f1_reg_write(s, reg, buf[i]);
            }
        }
    } else {
        bcm_sdio_bp_write(s, bcm_sdio_window(s) | (addr & WINDOW_MASK),
                          buf, len);
    }
}

static uint8_t bcm_sdio_vendor_read(SDIOCard *card, uint32_t addr)
{
    return BCM_SDIO(card)->vendor_cccr[addr & 0xf];
}

static void bcm_sdio_vendor_write(SDIOCard *card, uint32_t addr, uint8_t val)
{
    BcmSdioState *s = BCM_SDIO(card);

    s->vendor_cccr[addr & 0xf] = val;
    if (addr == CCCR_SEPINT) {
        bcm_sdio_wake(s);
    }
}

static void bcm_sdio_reset_hold(Object *obj, ResetType type)
{
    BcmSdioClass *bc = BCM_SDIO_GET_CLASS(obj);

    if (bc->parent_phases.hold) {
        bc->parent_phases.hold(obj, type);
    }
    bcm_sdio_chip_reset(BCM_SDIO(obj));
}

/*
 * The pins go to the SoC's GPIO controller: the one linked as "gpio", or
 * the only one there is
 */
static void bcm_sdio_wire(BcmSdioState *s)
{
    bool ambiguous;

    if (!s->gpio) {
        Object *o = object_resolve_path_type("", TYPE_INGENIC_GPIO,
                                             &ambiguous);

        s->gpio = o && !ambiguous ? DEVICE(o) : NULL;
    }
    if (!s->gpio) {
        return;
    }
    if (s->host_wake_pin >= 0) {
        qdev_connect_gpio_out_named(DEVICE(s), "host-wake", 0,
                                    qdev_get_gpio_in_named(s->gpio, "pull",
                                                           s->host_wake_pin));
    }
    if (s->reg_on_pin >= 0) {
        qdev_connect_gpio_out_named(s->gpio, "pad", s->reg_on_pin,
                                    qdev_get_gpio_in_named(DEVICE(s),
                                                           "reg-on", 0));
    }
}

static void bcm_sdio_realize(DeviceState *dev, Error **errp)
{
    BcmSdioState *s = BCM_SDIO(dev);

    if (!bcm_core_realize(&s->core, errp)) {
        return;
    }
    bcm_sdio_wire(s);
    bcm_sdio_chip_reset(s);
}

static void bcm_sdio_unrealize(DeviceState *dev)
{
    bcm_core_unrealize(&BCM_SDIO(dev)->core);
}

static void bcm_sdio_init(Object *obj)
{
    BcmSdioState *s = BCM_SDIO(obj);

    s->core.dev = DEVICE(obj);
    s->core.notify = bcm_sdio_notify;
    s->ram = g_malloc0(RAM_SIZE);
    s->tx = g_byte_array_new();
    qdev_init_gpio_out_named(DEVICE(obj), &s->host_wake, "host-wake", 1);
    qdev_init_gpio_in_named(DEVICE(obj), bcm_sdio_reg_on, "reg-on", 1);
    bcm_core_init(&s->core);
}

static void bcm_sdio_finalize(Object *obj)
{
    BcmSdioState *s = BCM_SDIO(obj);

    g_free(s->ram);
    g_byte_array_unref(s->tx);
    g_clear_pointer(&s->rx, g_byte_array_unref);
    g_clear_pointer(&s->last, g_byte_array_unref);
    bcm_core_finalize(&s->core);
}

/*
 * The pins default to what thingino's driver package hard-codes for every
 * SYN4343 camera: host wake on 39 (PB7), WL_REG_ON on 48 (PB16)
 */
static const Property bcm_sdio_props[] = {
    DEFINE_BCM_PROPERTIES(BcmSdioState, core),
    DEFINE_PROP_LINK("gpio", BcmSdioState, gpio, TYPE_DEVICE,
                     DeviceState *),
    DEFINE_PROP_INT32("host-wake-pin", BcmSdioState, host_wake_pin, 39),
    DEFINE_PROP_INT32("reg-on-pin", BcmSdioState, reg_on_pin, 48),
};

/* the RAM, the queued frames and the dongle's state are not migrated */
static const VMStateDescription bcm_sdio_vmstate = {
    .name = TYPE_BCM_SDIO,
    .unmigratable = 1,
};

static void bcm_sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);
    SDIOCardClass *sc = SDIO_CARD_CLASS(oc);

    dc->realize = bcm_sdio_realize;
    dc->unrealize = bcm_sdio_unrealize;
    resettable_class_set_parent_phases(rc, NULL, bcm_sdio_reset_hold, NULL,
                                       &BCM_SDIO_CLASS(oc)->parent_phases);
    dc->vmsd = &bcm_sdio_vmstate;
    device_class_set_props(dc, bcm_sdio_props);
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    sc->vendor = 0x02d0;
    sc->device = 0xa9a6;
    sc->num_funcs = 2;
    sc->max_blksz = 512;
    sc->read = bcm_sdio_read;
    sc->write = bcm_sdio_write;
    sc->vendor_read = bcm_sdio_vendor_read;
    sc->vendor_write = bcm_sdio_vendor_write;
}

static void bcm_sdio_chip_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    BCM_SDIO_CLASS(oc)->chip = data;
    dc->desc = ((const BcmSdioChip *)data)->desc;
}

static void bcm_sdio_register_types(void)
{
    static const TypeInfo bcm_sdio_info = {
        .name           = TYPE_BCM_SDIO,
        .parent         = TYPE_SDIO_CARD,
        .instance_size  = sizeof(BcmSdioState),
        .instance_init  = bcm_sdio_init,
        .instance_finalize = bcm_sdio_finalize,
        .class_size     = sizeof(BcmSdioClass),
        .class_init     = bcm_sdio_class_init,
        .abstract       = true,
    };

    type_register_static(&bcm_sdio_info);
    for (size_t i = 0; i < ARRAY_SIZE(bcm_sdio_chips); i++) {
        const TypeInfo ti = {
            .name       = bcm_sdio_chips[i].name,
            .parent     = TYPE_BCM_SDIO,
            .class_init = bcm_sdio_chip_class_init,
            .class_data = &bcm_sdio_chips[i],
        };

        type_register_static(&ti);
    }
}

type_init(bcm_sdio_register_types)
