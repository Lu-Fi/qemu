/*
 * Ingenic T31 OS Timer (OST) emulation
 *
 * The OST is a 64-bit free-running counter clocked from EXTAL/4 (6 MHz).
 * U-Boot uses it as the system timer via get_timer().
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "hw/misc/ingenic-t31-ost.h"

/* TCU global register offsets */
#define TCU_TER       0x10
#define TCU_TESR      0x14
#define TCU_TECR      0x18
#define TCU_TSR       0x1C
#define TCU_TFR       0x20
#define TCU_TFSR      0x24
#define TCU_TFCR      0x28
#define TCU_TSSR      0x2C
#define TCU_TMR       0x30
#define TCU_TMSR      0x34
#define TCU_TMCR      0x38
#define TCU_TSCR      0x3C
#define TCU_TSTR      0xF0
#define TCU_TSTSR     0xF4
#define TCU_TSTCR     0xF8

/* Per-channel register offsets (8 channels, 0x10 each, at 0x40) */
#define TCU_CH_BASE   0x40
#define TCU_CH_SIZE   0x10
#define CH_TDFR       0x00
#define CH_TDHR       0x04
#define CH_TCNT       0x08
#define CH_TCSR       0x0C

/* OST register offsets (at TCU base + 0xE0) */
#define OST_DR        0xE0
#define OST_CNTL      0xE4
#define OST_CNTH      0xE8
#define OST_CSR       0xEC
#define OST_CNTHBUF   0xFC

#define OST_IOSIZE    0x100

/* 6 MHz - EXTAL(24MHz) / prescale(4) */
#define OST_FREQ      6000000ULL

static uint64_t ingenic_t31_ost_get_count(IngenicT31OstState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed = now - s->base_time;

    return (uint64_t)(elapsed * OST_FREQ / NANOSECONDS_PER_SECOND);
}

static uint64_t ingenic_t31_ost_read(void *opaque, hwaddr offset,
                                     unsigned size)
{
    IngenicT31OstState *s = INGENIC_T31_OST(opaque);
    uint64_t count;

    /* TCU per-channel registers (offsets 0x40-0xBF) */
    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + TCU_NR_CHANNELS * TCU_CH_SIZE) {
        int ch = (offset - TCU_CH_BASE) / TCU_CH_SIZE;
        int reg = (offset - TCU_CH_BASE) % TCU_CH_SIZE;
        switch (reg) {
        case CH_TDFR: return s->ch[ch].tdfr;
        case CH_TDHR: return s->ch[ch].tdhr;
        case CH_TCNT: return s->ch[ch].tcnt;
        case CH_TCSR: return s->ch[ch].tcsr;
        }
        return 0;
    }

    switch (offset) {
    /* TCU global */
    case TCU_TER:   return s->ter;
    case TCU_TSR:   return s->tsr;
    case TCU_TFR:   return s->tfr;
    case TCU_TMR:   return s->tmr;
    case TCU_TSTR:  return s->tstr;
    /* OST */
    case OST_CNTL:
        count = ingenic_t31_ost_get_count(s);
        s->cnth_buf = (uint32_t)(count >> 32);
        return (uint32_t)count;
    case OST_CNTH:
        return s->cnth_buf;
    case OST_CNTHBUF:
        return s->cnth_buf;
    case OST_DR:
        return s->data_reg;
    case OST_CSR:
        return s->csr;
    default:
        return 0;
    }
}

static void ingenic_t31_ost_write(void *opaque, hwaddr offset,
                                  uint64_t value, unsigned size)
{
    IngenicT31OstState *s = INGENIC_T31_OST(opaque);

    /* TCU per-channel registers (offsets 0x40-0xBF) */
    if (offset >= TCU_CH_BASE &&
        offset < TCU_CH_BASE + TCU_NR_CHANNELS * TCU_CH_SIZE) {
        int ch = (offset - TCU_CH_BASE) / TCU_CH_SIZE;
        int reg = (offset - TCU_CH_BASE) % TCU_CH_SIZE;
        switch (reg) {
        case CH_TDFR: s->ch[ch].tdfr = value; break;
        case CH_TDHR: s->ch[ch].tdhr = value; break;
        case CH_TCNT: s->ch[ch].tcnt = value; break;
        case CH_TCSR: s->ch[ch].tcsr = value; break;
        }
        return;
    }

    switch (offset) {
    /* TCU global set/clear registers */
    case TCU_TER:   s->ter = value; break;
    case TCU_TESR:  s->ter |= value; break;
    case TCU_TECR:  s->ter &= ~value; break;
    case TCU_TSR:   s->tsr = value; break;
    case TCU_TSSR:  s->tsr |= value; break;
    case TCU_TSCR:  s->tsr &= ~value; break;
    case TCU_TFR:   s->tfr = value; break;
    case TCU_TFSR:  s->tfr |= value; break;
    case TCU_TFCR:  s->tfr &= ~value; break;
    case TCU_TMR:   s->tmr = value; break;
    case TCU_TMSR:  s->tmr |= value; break;
    case TCU_TMCR:  s->tmr &= ~value; break;
    case TCU_TSTR:  s->tstr = value; break;
    case TCU_TSTSR: s->tstr |= value; break;
    case TCU_TSTCR: s->tstr &= ~value; break;
    /* OST */
    case OST_DR:
        s->data_reg = (uint32_t)value;
        break;
    case OST_CNTL:
        s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    case OST_CSR:
        s->csr = (uint32_t)value;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_t31_ost_ops = {
    .read = ingenic_t31_ost_read,
    .write = ingenic_t31_ost_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 4 },
    .impl  = { .min_access_size = 2, .max_access_size = 4 },
};

static void ingenic_t31_ost_reset_hold(Object *obj, ResetType type)
{
    IngenicT31OstState *s = INGENIC_T31_OST(obj);

    s->base_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->cnth_buf = 0;
    s->data_reg = 0;
    s->csr = 0;

    s->ter = 0;
    s->tsr = 0;
    s->tfr = 0;
    s->tmr = 0;
    s->tstr = 0;
    memset(s->ch, 0, sizeof(s->ch));
}

static void ingenic_t31_ost_init(Object *obj)
{
    IngenicT31OstState *s = INGENIC_T31_OST(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t31_ost_ops, s,
                          TYPE_INGENIC_T31_OST, OST_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void ingenic_t31_ost_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_t31_ost;
    rc->phases.hold = ingenic_t31_ost_reset_hold;
}

static const TypeInfo ingenic_t31_ost_type_info = {
    .name = TYPE_INGENIC_T31_OST,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicT31OstState),
    .instance_init = ingenic_t31_ost_init,
    .class_init = ingenic_t31_ost_class_init,
};

static void ingenic_t31_ost_register_types(void)
{
    type_register_static(&ingenic_t31_ost_type_info);
}

type_init(ingenic_t31_ost_register_types)
