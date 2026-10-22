/*
 * SDIO card: the I/O card's side of the bus (CMD5, CMD3, CMD7, CMD52,
 * CMD53) and function 0, the CCCR, the FBRs and the CIS. A subclass
 * supplies functions 1-7: their registers, data and interrupts.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/sd/sdio.h"
#include "migration/vmstate.h"
#include "trace.h"

enum {
    SDIO_STATE_IDLE,
    SDIO_STATE_STANDBY,
    SDIO_STATE_TRANSFER,
};

/* R5 IO_CURRENT_STATE */
#define R5_STATE_CMD        (1 << 12)
#define R5_STATE_TRN        (2 << 12)
#define R5_FUNCTION_NUMBER  (1 << 9)
#define R5_OUT_OF_RANGE     (1 << 8)

/* CCCR */
#define CCCR_REV            0x00
#define CCCR_SD             0x01
#define CCCR_IOE            0x02
#define CCCR_IOR            0x03
#define CCCR_IEN            0x04
#define CCCR_INT            0x05
#define CCCR_ABORT          0x06
#define CCCR_IF             0x07
#define CCCR_CAPS           0x08
#define CCCR_CIS            0x09
#define CCCR_BLKSIZE        0x10
#define CCCR_SPEED          0x13
#define CCCR_VENDOR         0xf0

#define ABORT_RES           (1 << 3)
#define CAPS_SDC            (1 << 0)
#define CAPS_SMB            (1 << 1)
#define CAPS_S4MI           (1 << 4)
#define SPEED_SHS           (1 << 0)
#define SPEED_EHS           (1 << 1)

#define FBR_IF              0x00
#define FBR_CIS             0x09
#define FBR_BLKSIZE         0x10

/* where function 0's address space keeps the CIS */
#define CIS_COMMON          0x1000
#define CIS_FUNC(fn)        (0x2000 + ((fn) - 1) * 0x100)
#define CIS_END             (CIS_FUNC(SDIO_MAX_FUNCS) + 0x100)

/* SDIO 2.00, CCCR 1.20 */
#define SDIO_REV            0x32
/* 2.7-3.6 V */
#define SDIO_OCR            0x00ff8000

static const char *sdio_name(SDIOCard *s)
{
    return object_get_canonical_path_component(OBJECT(s)) ?: "sdio";
}

/*
 * The CIS: the common one (CISTPL_FUNCID, function 0's CISTPL_FUNCE with
 * its block size and 25 MHz, CISTPL_MANFID) and one per function, whose
 * CISTPL_FUNCE has the 42-byte SDIO 1.1+ body Linux insists on.
 */
static uint8_t sdio_cis_byte(SDIOCard *s, uint32_t addr)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);
    uint8_t common[] = {
        0x21, 0x02, 0x0c, 0x00,
        0x22, 0x04, 0x00, sc->max_blksz & 0xff, sc->max_blksz >> 8, 0x32,
        0x20, 0x04, sc->vendor & 0xff, sc->vendor >> 8,
                    sc->device & 0xff, sc->device >> 8,
        0xff,
    };
    uint8_t func[4 + 2 + 42 + 1] = {
        0x21, 0x02, 0x0c, 0x00,
        0x22, 42, 0x01,
    };
    uint8_t *funce = &func[6];

    if (addr >= CIS_COMMON && addr < CIS_COMMON + sizeof(common)) {
        return common[addr - CIS_COMMON];
    }
    if (addr < CIS_FUNC(1) || addr >= CIS_FUNC(sc->num_funcs + 1)) {
        return 0;
    }
    addr = (addr - CIS_FUNC(1)) & 0xff;
    funce[12] = sc->max_blksz & 0xff;
    funce[13] = sc->max_blksz >> 8;
    stl_le_p(&funce[14], SDIO_OCR);
    /* enable timeout, in 10 ms */
    funce[28] = 100;
    func[sizeof(func) - 1] = 0xff;
    return addr < sizeof(func) ? func[addr] : 0;
}

static void sdio_update_irq(SDIOCard *s)
{
    bool level = (s->int_enable & 1) && (s->int_pending & s->int_enable & 0xfe);
    BusState *bus = qdev_get_parent_bus(DEVICE(s));

    if (level != s->irq_level) {
        s->irq_level = level;
        trace_sdio_irq(sdio_name(s), level);
        if (bus) {
            sdbus_set_sdio_irq(SD_BUS(bus), level);
        }
    }
}

void sdio_card_set_irq(SDIOCard *s, unsigned fn, bool level)
{
    if (level) {
        s->int_pending |= 1 << fn;
    } else {
        s->int_pending &= ~(1 << fn);
    }
    sdio_update_irq(s);
}

static uint8_t sdio_f0_read(SDIOCard *s, uint32_t addr)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);
    unsigned fn;

    if (addr >= CIS_COMMON && addr < CIS_END) {
        return sdio_cis_byte(s, addr);
    }
    if (addr >= CCCR_VENDOR && addr < 0x100) {
        return sc->vendor_read ? sc->vendor_read(s, addr) : 0;
    }
    if (addr >= 0x100 && addr < 0x100 * (sc->num_funcs + 1)) {
        fn = addr >> 8;
        switch (addr & 0xff) {
        case FBR_IF:
            return sc->func_class & 0x0f;
        case FBR_CIS:
            return CIS_FUNC(fn) & 0xff;
        case FBR_CIS + 1:
            return (CIS_FUNC(fn) >> 8) & 0xff;
        case FBR_CIS + 2:
            return CIS_FUNC(fn) >> 16;
        case FBR_BLKSIZE:
            return s->blksz[fn] & 0xff;
        case FBR_BLKSIZE + 1:
            return s->blksz[fn] >> 8;
        }
        return 0;
    }
    switch (addr) {
    case CCCR_REV:
        return SDIO_REV;
    case CCCR_SD:
        return 0x02;
    case CCCR_IOE:
        return s->io_enable;
    case CCCR_IOR:
        /* functions are ready as soon as they are enabled */
        return s->io_enable;
    case CCCR_IEN:
        return s->int_enable;
    case CCCR_INT:
        return s->int_pending & 0xfe;
    case CCCR_IF:
        return s->bus_if;
    case CCCR_CAPS:
        return CAPS_SDC | CAPS_SMB | CAPS_S4MI;
    case CCCR_CIS:
        return CIS_COMMON & 0xff;
    case CCCR_CIS + 1:
        return (CIS_COMMON >> 8) & 0xff;
    case CCCR_CIS + 2:
        return CIS_COMMON >> 16;
    case CCCR_BLKSIZE:
        return s->blksz[0] & 0xff;
    case CCCR_BLKSIZE + 1:
        return s->blksz[0] >> 8;
    case CCCR_SPEED:
        return SPEED_SHS | (s->speed & SPEED_EHS);
    }
    return 0;
}

static void sdio_io_reset(SDIOCard *s)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);

    s->io_enable = 0;
    s->int_enable = 0;
    s->bus_if = 0;
    s->speed = 0;
    memset(s->blksz, 0, sizeof(s->blksz));
    if (sc->io_reset) {
        sc->io_reset(s);
    }
    sdio_update_irq(s);
}

static void sdio_f0_write(SDIOCard *s, uint32_t addr, uint8_t val)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);
    uint8_t funcs = ((1 << (sc->num_funcs + 1)) - 1) & 0xfe;
    unsigned fn;

    if (addr >= CCCR_VENDOR && addr < 0x100) {
        if (sc->vendor_write) {
            sc->vendor_write(s, addr, val);
        }
        return;
    }
    if (addr >= 0x100 && addr < 0x100 * (sc->num_funcs + 1)) {
        fn = addr >> 8;
        switch (addr & 0xff) {
        case FBR_BLKSIZE:
            s->blksz[fn] = (s->blksz[fn] & 0xff00) | val;
            break;
        case FBR_BLKSIZE + 1:
            s->blksz[fn] = (s->blksz[fn] & 0x00ff) | (val << 8);
            break;
        }
        return;
    }
    switch (addr) {
    case CCCR_IOE:
        s->io_enable = val & funcs;
        break;
    case CCCR_IEN:
        s->int_enable = val & (funcs | 1);
        sdio_update_irq(s);
        break;
    case CCCR_ABORT:
        if (val & ABORT_RES) {
            sdio_io_reset(s);
        }
        break;
    case CCCR_IF:
        s->bus_if = val;
        break;
    case CCCR_BLKSIZE:
        s->blksz[0] = (s->blksz[0] & 0xff00) | val;
        break;
    case CCCR_BLKSIZE + 1:
        s->blksz[0] = (s->blksz[0] & 0x00ff) | (val << 8);
        break;
    case CCCR_SPEED:
        s->speed = val & SPEED_EHS;
        break;
    }
}

static void sdio_func_read(SDIOCard *s, unsigned fn, uint32_t addr, bool incr,
                           uint8_t *buf, size_t len)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);

    if (fn == 0) {
        for (size_t i = 0; i < len; i++) {
            buf[i] = sdio_f0_read(s, addr + (incr ? i : 0));
        }
    } else if (sc->read) {
        sc->read(s, fn, addr, incr, buf, len);
    } else {
        memset(buf, 0, len);
    }
}

static void sdio_func_write(SDIOCard *s, unsigned fn, uint32_t addr,
                            bool incr, const uint8_t *buf, size_t len)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);

    if (fn == 0) {
        for (size_t i = 0; i < len; i++) {
            sdio_f0_write(s, addr + (incr ? i : 0), buf[i]);
        }
    } else if (sc->write) {
        sc->write(s, fn, addr, incr, buf, len);
    }
}

static size_t sdio_r4(uint32_t val, uint8_t *resp)
{
    stl_be_p(resp, val);
    return 4;
}

/* CMD52 IO_RW_DIRECT */
static size_t sdio_rw_direct(SDIOCard *s, uint32_t arg, uint8_t *resp)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);
    bool write = arg >> 31;
    unsigned fn = (arg >> 28) & 7;
    bool raw = (arg >> 27) & 1;
    uint32_t addr = (arg >> 9) & 0x1ffff;
    uint8_t val = arg & 0xff;
    uint32_t r5 = s->state == SDIO_STATE_TRANSFER ? R5_STATE_CMD : 0;

    if (fn > sc->num_funcs) {
        return sdio_r4(r5 | R5_FUNCTION_NUMBER, resp);
    }
    if (write) {
        sdio_func_write(s, fn, addr, false, &val, 1);
        if (raw) {
            sdio_func_read(s, fn, addr, false, &val, 1);
        }
    } else {
        sdio_func_read(s, fn, addr, false, &val, 1);
    }
    trace_sdio_rw_direct(sdio_name(s), write, fn, addr, val);
    return sdio_r4(r5 | val, resp);
}

/* CMD53 IO_RW_EXTENDED: the data phase follows on the data lines */
static size_t sdio_rw_extended(SDIOCard *s, uint32_t arg, uint8_t *resp)
{
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);
    bool write = arg >> 31;
    unsigned fn = (arg >> 28) & 7;
    bool block = (arg >> 27) & 1;
    bool incr = (arg >> 26) & 1;
    uint32_t addr = (arg >> 9) & 0x1ffff;
    uint32_t count = arg & 0x1ff;
    uint32_t len;

    if (fn > sc->num_funcs) {
        return sdio_r4(R5_STATE_CMD | R5_FUNCTION_NUMBER, resp);
    }
    if (block) {
        if (!count || !s->blksz[fn]) {
            /* no infinite block transfers; no block size set */
            qemu_log_mask(LOG_UNIMP, "%s: CMD53 %u blocks of %u bytes\n",
                          sdio_name(s), count, s->blksz[fn]);
            return sdio_r4(R5_STATE_CMD | R5_OUT_OF_RANGE, resp);
        }
        len = count * s->blksz[fn];
    } else {
        len = count ? count : 512;
    }
    trace_sdio_rw_extended(sdio_name(s), write, fn, addr, incr, len);

    s->xfer_write = write;
    s->xfer_incr = incr;
    s->xfer_fn = fn;
    s->xfer_addr = addr;
    s->xfer_len = len;
    s->xfer_pos = 0;
    s->xfer_buf = g_realloc(s->xfer_buf, len);
    if (!write) {
        sdio_func_read(s, fn, addr, incr, s->xfer_buf, len);
    }
    return sdio_r4(R5_STATE_TRN, resp);
}

static size_t sdio_do_command(SDState *sd, SDRequest *req,
                              uint8_t *resp, size_t respsz)
{
    SDIOCard *s = SDIO_CARD(sd);
    SDIOCardClass *sc = SDIO_CARD_GET_CLASS(s);
    uint32_t r;

    trace_sdio_cmd(sdio_name(s), req->cmd, req->arg);
    assert(respsz >= 4);

    switch (req->cmd) {
    case 3:     /* SEND_RELATIVE_ADDR, R6 */
        s->rca = 0x0001;
        s->state = SDIO_STATE_STANDBY;
        return sdio_r4((uint32_t)s->rca << 16, resp);
    case 5:     /* IO_SEND_OP_COND, R4: ready, the functions, no memory */
        r = SDIO_OCR | (uint32_t)sc->num_funcs << 28;
        if (req->arg & 0xffffff) {
            r |= 1u << 31;
        }
        return sdio_r4(r, resp);
    case 7:     /* SELECT/DESELECT_CARD, R1b */
        s->state = (req->arg >> 16) == s->rca ? SDIO_STATE_TRANSFER
                                              : SDIO_STATE_STANDBY;
        return sdio_r4(0, resp);
    case 52:
        return sdio_rw_direct(s, req->arg, resp);
    case 53:
        return sdio_rw_extended(s, req->arg, resp);
    case 0:     /* GO_IDLE_STATE leaves the I/O side alone */
    case 8:     /* SEND_IF_COND, SD_APP_OP_COND, APP_CMD, SEND_OP_COND: */
    case 41:    /* an I/O-only card answers none of them */
    case 55:
    case 1:
        return 0;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "%s: CMD%u not supported\n",
                  sdio_name(s), req->cmd);
    return 0;
}

static size_t sdio_read_data(SDState *sd, void *buf, size_t len)
{
    SDIOCard *s = SDIO_CARD(sd);
    size_t n = 0;

    if (!s->xfer_write && s->xfer_pos < s->xfer_len) {
        n = MIN(len, s->xfer_len - s->xfer_pos);
        memcpy(buf, s->xfer_buf + s->xfer_pos, n);
        s->xfer_pos += n;
    }
    if (n < len) {
        memset((uint8_t *)buf + n, 0, len - n);
    }
    return len;
}

static size_t sdio_write_data(SDState *sd, const void *buf, size_t len)
{
    SDIOCard *s = SDIO_CARD(sd);
    size_t n;

    if (!s->xfer_write || s->xfer_pos >= s->xfer_len) {
        return len;
    }
    n = MIN(len, s->xfer_len - s->xfer_pos);
    memcpy(s->xfer_buf + s->xfer_pos, buf, n);
    s->xfer_pos += n;
    if (s->xfer_pos == s->xfer_len) {
        sdio_func_write(s, s->xfer_fn, s->xfer_addr, s->xfer_incr,
                        s->xfer_buf, s->xfer_len);
    }
    return len;
}

static bool sdio_data_ready(SDState *sd)
{
    SDIOCard *s = SDIO_CARD(sd);

    return !s->xfer_write && s->xfer_pos < s->xfer_len;
}

static bool sdio_receive_ready(SDState *sd)
{
    SDIOCard *s = SDIO_CARD(sd);

    return s->xfer_write && s->xfer_pos < s->xfer_len;
}

static void sdio_set_voltage(SDState *sd, uint16_t millivolts)
{
}

static uint8_t sdio_get_dat_lines(SDState *sd)
{
    SDIOCard *s = SDIO_CARD(sd);

    /* an interrupt holds DAT1 low */
    return s->irq_level ? 0x0d : 0x0f;
}

static bool sdio_get_cmd_line(SDState *sd)
{
    return true;
}

static bool sdio_get_inserted(SDState *sd)
{
    return true;
}

static bool sdio_get_readonly(SDState *sd)
{
    return false;
}

static void sdio_reset(DeviceState *dev)
{
    SDIOCard *s = SDIO_CARD(dev);

    s->rca = 0;
    s->state = SDIO_STATE_IDLE;
    s->int_pending = 0;
    s->xfer_len = 0;
    s->xfer_pos = 0;
    sdio_io_reset(s);
}

static void sdio_finalize(Object *obj)
{
    SDIOCard *s = SDIO_CARD(obj);

    g_free(s->xfer_buf);
}

static const VMStateDescription vmstate_sdio = {
    .name = "sdio-card",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT16(rca, SDIOCard),
        VMSTATE_UINT8(state, SDIOCard),
        VMSTATE_UINT8(io_enable, SDIOCard),
        VMSTATE_UINT8(int_enable, SDIOCard),
        VMSTATE_UINT8(int_pending, SDIOCard),
        VMSTATE_UINT8(bus_if, SDIOCard),
        VMSTATE_UINT8(speed, SDIOCard),
        VMSTATE_UINT16_ARRAY(blksz, SDIOCard, SDIO_MAX_FUNCS + 1),
        VMSTATE_BOOL(irq_level, SDIOCard),
        VMSTATE_BOOL(xfer_write, SDIOCard),
        VMSTATE_BOOL(xfer_incr, SDIOCard),
        VMSTATE_UINT8(xfer_fn, SDIOCard),
        VMSTATE_UINT32(xfer_addr, SDIOCard),
        VMSTATE_UINT32(xfer_len, SDIOCard),
        VMSTATE_UINT32(xfer_pos, SDIOCard),
        VMSTATE_VBUFFER_ALLOC_UINT32(xfer_buf, SDIOCard, 0, NULL, xfer_len),
        VMSTATE_END_OF_LIST()
    },
};

static void sdio_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    SDCardClass *sc = (SDCardClass *)oc;

    dc->bus_type = TYPE_SD_BUS;
    dc->vmsd = &vmstate_sdio;
    device_class_set_legacy_reset(dc, sdio_reset);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);

    sc->do_command = sdio_do_command;
    sc->read_data = sdio_read_data;
    sc->write_data = sdio_write_data;
    sc->data_ready = sdio_data_ready;
    sc->receive_ready = sdio_receive_ready;
    sc->set_voltage = sdio_set_voltage;
    sc->get_dat_lines = sdio_get_dat_lines;
    sc->get_cmd_line = sdio_get_cmd_line;
    sc->get_inserted = sdio_get_inserted;
    sc->get_readonly = sdio_get_readonly;
}

static const TypeInfo sdio_types[] = {
    {
        .name           = TYPE_SDIO_CARD,
        .parent         = TYPE_DEVICE,
        .abstract       = true,
        .instance_size  = sizeof(SDIOCard),
        .instance_finalize = sdio_finalize,
        .class_size     = sizeof(SDIOCardClass),
        .class_init     = sdio_class_init,
    },
};

DEFINE_TYPES(sdio_types)
