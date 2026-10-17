/*
 * Ingenic AudioSystem DMIC capture, the T32, T33 and T41's digital
 * microphone path to memory
 *
 * Five blocks in one 64 KiB window: a descriptor DMA (0x0000) with a FIFO
 * per channel (0x1000), a format converter per channel (0x2000), the audio
 * bus that routes timeslots between devices (0x4000) and the DMIC itself
 * (0xa000). The vendor driver routes the DMIC (bus source 0) to DMA
 * channel 5 and runs that channel over a ring of linked descriptors, one
 * per 20 ms fragment, polling the channel's DBA for its position.
 *
 * As read on a T41NQ under the vendor driver: reset values (all zero but
 * the DMIC's CR0, 0x40, and the bus's BFSR); DBA advancing a transfer unit
 * at a time; DTC counting the units left in the descriptor; DSR showing
 * bits 11:4 of the descriptor's address and TT once one has completed;
 * DGER left clear; the bus keeping only the BFCR1/BFCR2 bits of the devices
 * it has. The block reads 0xdeaddead while its clock is gated.
 *
 * Every microphone hears the "mic" property's source (see ingenic-mic.c);
 * the gain and the filters are registers only.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "migration/vmstate.h"
#include "hw/audio/ingenic-as-dmic.h"

/* DMA, per channel at 0x18 * ch */
#define DMA_CH_STRIDE   0x18
#define DBA             0x00
#define DTC             0x04
#define DCR             0x08
#define DSR             0x0c
#define DCM             0x10
#define DDA             0x14
#define DGRR            0x100
#define DGER            0x104
#define AEER            0x108
#define AESR            0x10c
#define AIPR            0x110

#define DCR_CTE         BIT(0)
#define DCR_RESET       BIT(1)

#define DSR_LTT         BIT(2)
#define DSR_TT          BIT(3)
#define DSR_LTT_INT     BIT(4)
#define DSR_TT_INT      BIT(5)
#define DSR_W1C         (DSR_LTT | DSR_TT | DSR_LTT_INT | DSR_TT_INT)

#define DCM_NDES        BIT(31)

/* A descriptor's word 0, laid out as DCM's low bits */
#define DESC_LTIE       BIT(0)
#define DESC_TIE        BIT(1)
#define DESC_LINK       BIT(2)
#define DESC_TSZ(v)     extract32(v, 4, 3)
#define DESC_BAI        BIT(8)

/* FIFO, per channel at 0x10 * ch */
#define FIFO_CH_STRIDE  0x10
#define FAS             0x0
#define FCR             0x4
#define FFR             0x8
#define FSR             0xc

#define FCR_FIFO_EN     BIT(0)
#define FCR_FLUSH       BIT(2)
#define FSR_NOT_EMPTY   BIT(0)
#define FSR_TURROR      BIT(2)
#define FSR_W1C         (FSR_TURROR | BIT(4) | BIT(5))

/* Format converter */
#define DFCR_EN         BIT(0)
#define DFCR_PACKEN     BIT(2)
#define DFCR_SS(v)      extract32(v, 8, 3)

/* Audio bus */
#define BTSET           0x00
#define BTCLR           0x04
#define BTSR            0x08
#define BFSR            0x0c
#define BFCR0           0x10
#define BFCR1           0x14
#define BFCR2           0x18
#define BST0            0x1c
#define BTT0            0x28
#define BUS_DMIC        0           /* the DMIC's source device */
#define BUS_LO_PORTS    12

/* DMIC */
#define CR0             0x00
#define GCR             0x04
#define IER             0x08
#define ICR             0x0c
#define TCR             0x10

#define CR0_RESET       BIT(31)
#define CR0_EN          BIT(0)
#define CR0_MASK        0x007f39c1
#define CR0_CHNUM(v)    extract32(v, 16, 4)
#define CR0_SR(v)       extract32(v, 6, 3)

static const uint8_t ss_bits[8] = { 8, 12, 13, 16, 18, 20, 24, 32 };

static void as_schedule(IngenicAsDmicState *s);

static uint32_t as_rate(IngenicAsDmicState *s)
{
    static const uint32_t rates[4] = { 8000, 16000, 48000, 96000 };
    unsigned sr = CR0_SR(s->cr0);

    return sr < 4 ? rates[sr] : 48000;
}

/* Is the DMIC's timeslot on bus port d, which feeds DMA channel d? */
static bool as_routed(IngenicAsDmicState *s, unsigned d)
{
    static const uint8_t shift[6] = { 0, 5, 10, 16, 21, 26 };
    unsigned slot = extract32(s->bst[0], 0, 5);

    return slot && (s->btsr & BIT(slot)) &&
           extract32(s->btt[d / 6], shift[d % 6], 5) == slot;
}

static bool as_capturing(IngenicAsDmicState *s, unsigned c)
{
    return as_routed(s, c) && (s->ch[c].dfcr & DFCR_EN) &&
           (s->ch[c].fcr & FCR_FIFO_EN);
}

static unsigned as_fifo_size(IngenicAsChannel *c)
{
    return MIN(extract32(c->fas, 0, 13) * 4, INGENIC_AS_FIFO_BYTES);
}

static unsigned as_sample_bytes(IngenicAsChannel *c)
{
    unsigned bits = ss_bits[DFCR_SS(c->dfcr)];

    return bits <= 8 ? 1 : bits <= 16 ? 2 : c->dfcr & DFCR_PACKEN ? 3 : 4;
}

/* The bytes of a DMA transfer unit: TSZ 0 is a word, n is 2^(n+1) words */
static unsigned as_unit(IngenicAsChannel *c)
{
    unsigned tsz = DESC_TSZ(c->flags);

    return 4 << (tsz ? tsz + 1 : 0);
}

static void as_update_irq(IngenicAsDmicState *s)
{
    bool irq = false;

    for (unsigned i = 0; i < INGENIC_AS_CHANNELS; i++) {
        irq |= !!(s->ch[i].dsr & (DSR_TT_INT | DSR_LTT_INT));
    }
    qemu_set_irq(s->irq, irq);
}

/* --- DMA ---------------------------------------------------------------- */

static void as_load_desc(IngenicAsChannel *c, hwaddr addr)
{
    uint32_t d[4];

    if (address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                           d, sizeof(d)) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-as-dmic: descriptor at 0x%"
                      HWADDR_PRIx " unreadable\n", addr);
        c->busy = false;
        return;
    }
    c->flags = le32_to_cpu(d[0]);
    c->dba = le32_to_cpu(d[1]);
    c->next = le32_to_cpu(d[2]);
    c->dtc = extract32(le32_to_cpu(d[3]), 0, 24);
    c->dsr = deposit32(c->dsr, 8, 8, extract32(addr, 4, 8));
}

static void as_start(IngenicAsChannel *c)
{
    c->busy = true;
    if (c->dcm & DCM_NDES) {
        c->flags = c->dcm & 0x1ff & ~DESC_LINK;
    } else {
        as_load_desc(c, c->dda);
    }
}

/*
 * Move whole transfer units from the FIFO while the descriptor wants them.
 * Linked descriptors sit in the page of the first: their next field gives
 * bits 11:4, which is what DSR shows.
 */
static void as_dma_run(IngenicAsChannel *c)
{
    unsigned loads = 0;

    while (c->busy) {
        unsigned unit = as_unit(c);
        uint8_t buf[1024];

        if (c->dtc) {
            if (c->n < unit) {
                return;
            }
            for (unsigned i = 0; i < unit; i++) {
                buf[i] = c->fifo[(c->rd + i) % INGENIC_AS_FIFO_BYTES];
            }
            c->rd = (c->rd + unit) % INGENIC_AS_FIFO_BYTES;
            c->n -= unit;
            address_space_write(&address_space_memory, c->dba,
                                MEMTXATTRS_UNSPECIFIED, buf, unit);
            if (c->flags & DESC_BAI) {
                c->dba += unit;
            }
            if (--c->dtc) {
                continue;
            }
        }
        c->dsr |= DSR_TT | (c->flags & DESC_TIE ? DSR_TT_INT : 0);
        if (!(c->flags & DESC_LINK)) {
            c->dsr |= DSR_LTT | (c->flags & DESC_LTIE ? DSR_LTT_INT : 0);
            c->busy = false;
        } else if (++loads > 64) {
            return;     /* a ring of empty descriptors: wait for a tick */
        } else {
            as_load_desc(c, (c->dda & ~0xfffu) | (c->next & 0xff0));
        }
    }
}

/* --- Capture ------------------------------------------------------------ */

static void as_push(IngenicAsChannel *c, int16_t x)
{
    unsigned bits = ss_bits[DFCR_SS(c->dfcr)];
    unsigned len = as_sample_bytes(c);
    int32_t v = bits >= 16 ? x * (1 << (bits - 16)) : x >> (16 - bits);

    if (c->n + len > as_fifo_size(c)) {
        c->fsr |= FSR_TURROR;
        return;
    }
    for (unsigned i = 0; i < len; i++) {
        c->fifo[(c->rd + c->n++) % INGENIC_AS_FIFO_BYTES] = v >> (8 * i);
    }
}

static void as_frame(IngenicAsDmicState *s)
{
    unsigned mics = CR0_CHNUM(s->cr0) + 1;
    int16_t f[2];

    ingenic_mic_frame(&s->mic_src, s->rate, f);
    for (unsigned i = 0; i < INGENIC_AS_CHANNELS; i++) {
        if (as_capturing(s, i)) {
            for (unsigned m = 0; m < mics; m++) {
                as_push(&s->ch[i], f[m & 1]);
            }
            as_dma_run(&s->ch[i]);
        }
    }
}

static void as_clock_update(IngenicAsDmicState *s)
{
    bool run = s->cr0 & CR0_EN;
    uint32_t rate = as_rate(s);

    if (run && (!s->running || rate != s->rate)) {
        s->t0 = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->frames = 0;
        s->rate = rate;
        ingenic_mic_open(&s->mic_src, s->audio_be, rate);
    }
    s->running = run;
    ingenic_mic_set_active(&s->mic_src, run);
    if (run) {
        as_schedule(s);
    } else {
        timer_del(s->timer);
    }
}

static void as_tick(void *opaque)
{
    IngenicAsDmicState *s = opaque;
    uint64_t due;

    if (!s->running) {
        return;
    }
    due = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->t0, s->rate,
                   NANOSECONDS_PER_SECOND);
    for (; s->frames < due; s->frames++) {
        as_frame(s);
    }
    as_update_irq(s);
    as_schedule(s);
}

/* Wake as a transfer unit completes, at least every 10 ms */
static void as_schedule(IngenicAsDmicState *s)
{
    unsigned mics = CR0_CHNUM(s->cr0) + 1;
    uint64_t k = MAX(s->rate / 100, 1);

    for (unsigned i = 0; i < INGENIC_AS_CHANNELS; i++) {
        IngenicAsChannel *c = &s->ch[i];

        if (as_capturing(s, i) && c->busy && c->n < as_unit(c)) {
            k = MIN(k, DIV_ROUND_UP(as_unit(c) - c->n,
                                    mics * as_sample_bytes(c)));
        }
    }
    k = MAX(k, 1);
    timer_mod(s->timer, s->t0 + muldiv64(s->frames + k, NANOSECONDS_PER_SECOND,
                                         s->rate) + 1);
}

/* --- Registers ---------------------------------------------------------- */

static void as_reset_channel(IngenicAsChannel *c)
{
    c->busy = false;
    c->dcr = c->dsr = 0;
}

static uint64_t as_dma_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAsDmicState *s = opaque;
    unsigned i = addr / DMA_CH_STRIDE;
    uint32_t v = 0;

    if (i < INGENIC_AS_CHANNELS) {
        IngenicAsChannel *c = &s->ch[i];

        switch (addr % DMA_CH_STRIDE) {
        case DBA:
            return c->dba;
        case DTC:
            return c->dtc;
        case DCR:
            return c->dcr;
        case DSR:
            return c->dsr;
        case DCM:
            return c->dcm;
        case DDA:
            return c->dda;
        }
    }
    switch (addr) {
    case DGER:
        return s->dger;
    case AEER:
        return s->aeer;
    case AESR:
        return s->aesr;
    case AIPR:
        for (i = 0; i < INGENIC_AS_CHANNELS; i++) {
            v |= s->ch[i].dsr & (DSR_TT_INT | DSR_LTT_INT) ? BIT(i) : 0;
        }
        return v;
    }
    return 0;
}

static void as_dma_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    IngenicAsDmicState *s = opaque;
    unsigned i = addr / DMA_CH_STRIDE;

    if (i < INGENIC_AS_CHANNELS) {
        IngenicAsChannel *c = &s->ch[i];

        switch (addr % DMA_CH_STRIDE) {
        case DBA:
            c->dba = value;
            break;
        case DTC:
            c->dtc = extract32(value, 0, 24);
            break;
        case DCR:
            if (value & DCR_RESET) {
                as_reset_channel(c);
            }
            if ((value & DCR_CTE) && !(c->dcr & DCR_CTE)) {
                c->dcr = DCR_CTE;
                as_start(c);
                as_dma_run(c);
            } else if (!(value & DCR_CTE)) {
                c->dcr = 0;
                c->busy = false;
            }
            break;
        case DSR:
            c->dsr &= ~(value & DSR_W1C);
            break;
        case DCM:
            c->dcm = value;
            break;
        case DDA:
            c->dda = value;
            break;
        }
    } else {
        switch (addr) {
        case DGRR:
            if (value & 1) {
                for (i = 0; i < INGENIC_AS_CHANNELS; i++) {
                    as_reset_channel(&s->ch[i]);
                }
            }
            break;
        case DGER:
            s->dger = value & 1;
            break;
        case AEER:
            s->aeer = value & 1;
            break;
        case AESR:
            s->aesr &= ~(value & 1);
            break;
        }
    }
    as_update_irq(s);
    if (s->running) {
        as_schedule(s);
    }
}

static uint64_t as_fifo_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAsDmicState *s = opaque;
    unsigned i = addr / FIFO_CH_STRIDE;
    IngenicAsChannel *c;

    if (i >= INGENIC_AS_CHANNELS) {
        return 0;
    }
    c = &s->ch[i];
    switch (addr % FIFO_CH_STRIDE) {
    case FAS:
        return c->fas;
    case FCR:
        return c->fcr;
    case FFR:
        return c->ffr;
    default:
        return deposit32(c->fsr, 16, 13, c->n / 4) |
               (c->n ? FSR_NOT_EMPTY : 0);
    }
}

static void as_fifo_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    IngenicAsDmicState *s = opaque;
    unsigned i = addr / FIFO_CH_STRIDE;
    IngenicAsChannel *c;

    if (i >= INGENIC_AS_CHANNELS) {
        return;
    }
    c = &s->ch[i];
    switch (addr % FIFO_CH_STRIDE) {
    case FAS:
        c->fas = extract32(value, 0, 13);
        break;
    case FCR:
        if (value & FCR_FLUSH) {
            c->rd = c->n = 0;
        }
        c->fcr = value & 3;
        break;
    case FFR:
        c->ffr = value & 0x1fff0380;
        break;
    default:
        c->fsr &= ~(value & FSR_W1C);
        break;
    }
    if (s->running) {
        as_schedule(s);
    }
}

static uint64_t as_fmt_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAsDmicState *s = opaque;

    return addr / 4 < INGENIC_AS_CHANNELS ? s->ch[addr / 4].dfcr : 0;
}

static void as_fmt_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    IngenicAsDmicState *s = opaque;

    if (addr / 4 < INGENIC_AS_CHANNELS) {
        s->ch[addr / 4].dfcr = value & 0xf705;
    }
    if (s->running) {
        as_schedule(s);
    }
}

static uint64_t as_dbus_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAsDmicState *s = opaque;

    switch (addr) {
    case BTSR:
        return s->btsr;
    case BFSR:
        return s->bfsr;
    case BFCR1:
        return s->bfcr1;
    case BFCR2:
        return s->bfcr2;
    case BST0 ... BST0 + 8:
        return s->bst[(addr - BST0) / 4];
    case BTT0 ... BTT0 + 4:
        return s->btt[(addr - BTT0) / 4];
    }
    return 0;
}

static void as_dbus_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    IngenicAsDmicState *s = opaque;

    switch (addr) {
    case BTSET:
        s->btsr |= value & ~1u;
        break;
    case BTCLR:
        s->btsr &= ~value;
        break;
    case BFSR:
        s->bfsr &= ~value;
        break;
    case BFCR1:
        s->bfcr1 = value & 0x20;
        break;
    case BFCR2:
        s->bfcr2 = value & 0x1;
        break;
    case BST0 ... BST0 + 8:
        s->bst[(addr - BST0) / 4] = value & 0x7fff7fff;
        break;
    case BTT0 ... BTT0 + 4:
        s->btt[(addr - BTT0) / 4] = value & 0x7fff7fff;
        break;
    }
    if (s->running) {
        as_schedule(s);
    }
}

static uint64_t as_dmic_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicAsDmicState *s = opaque;

    switch (addr) {
    case CR0:
        return s->cr0;
    case GCR:
        return s->gcr;
    case IER:
        return s->ier;
    case ICR:
        return s->icr;
    case TCR:
        return s->tcr;
    }
    return 0;
}

static void as_dmic_write(void *opaque, hwaddr addr, uint64_t value,
                          unsigned size)
{
    IngenicAsDmicState *s = opaque;

    switch (addr) {
    case CR0:
        /* The driver waits for RESET to clear: it does at once */
        s->cr0 = value & CR0_MASK;
        as_clock_update(s);
        break;
    case GCR:
        s->gcr = value & 0x1f;
        break;
    case IER:
        s->ier = value;
        break;
    case ICR:
        s->icr = value;
        break;
    case TCR:
        s->tcr = value;
        break;
    }
}

#define AS_OPS(name)                                                \
    static const MemoryRegionOps as_##name##_ops = {                \
        .read = as_##name##_read,                                   \
        .write = as_##name##_write,                                 \
        .endianness = DEVICE_LITTLE_ENDIAN,                         \
        .valid = { .min_access_size = 4, .max_access_size = 4 },    \
    }

AS_OPS(dma);
AS_OPS(fifo);
AS_OPS(fmt);
AS_OPS(dbus);
AS_OPS(dmic);

/* --- Lifecycle ---------------------------------------------------------- */

static void as_realize(DeviceState *dev, Error **errp)
{
    IngenicAsDmicState *s = INGENIC_AS_DMIC(dev);
    Error *err = NULL;

    if (!ingenic_mic_init(&s->mic_src, TYPE_INGENIC_AS_DMIC, s->mic, errp)) {
        return;
    }
    /* As for the AIC, an audiodev that cannot start costs the host side */
    if (!audio_be_check(&s->audio_be, &err)) {
        warn_report_err(err);
    }
}

static void as_unrealize(DeviceState *dev)
{
    ingenic_mic_cleanup(&INGENIC_AS_DMIC(dev)->mic_src);
}

static void as_reset_hold(Object *obj, ResetType type)
{
    IngenicAsDmicState *s = INGENIC_AS_DMIC(obj);

    timer_del(s->timer);
    s->running = false;
    memset(s->ch, 0, sizeof(s->ch));
    s->dger = s->aeer = s->aesr = 0;
    s->btsr = 0;
    s->bfsr = 0x00200000;
    s->bfcr1 = s->bfcr2 = 0;
    memset(s->bst, 0, sizeof(s->bst));
    memset(s->btt, 0, sizeof(s->btt));
    s->cr0 = 0x40;
    s->gcr = s->ier = s->icr = s->tcr = 0;
}

static void as_reset_exit(Object *obj, ResetType type)
{
    as_update_irq(INGENIC_AS_DMIC(obj));
}

static void as_init(Object *obj)
{
    IngenicAsDmicState *s = INGENIC_AS_DMIC(obj);
    static const struct {
        const char *name;
        const MemoryRegionOps *ops;
        hwaddr offset;
        ptrdiff_t mr;
    } blocks[] = {
        { "dma", &as_dma_ops, 0x0000,
          offsetof(IngenicAsDmicState, dma_iomem) },
        { "fifo", &as_fifo_ops, 0x1000,
          offsetof(IngenicAsDmicState, fifo_iomem) },
        { "fmtcov", &as_fmt_ops, 0x2000,
          offsetof(IngenicAsDmicState, fmt_iomem) },
        { "dbus", &as_dbus_ops, 0x4000,
          offsetof(IngenicAsDmicState, dbus_iomem) },
        { "dmic", &as_dmic_ops, 0xa000,
          offsetof(IngenicAsDmicState, dmic_iomem) },
    };

    memory_region_init(&s->iomem, obj, TYPE_INGENIC_AS_DMIC, 0x10000);
    for (int i = 0; i < ARRAY_SIZE(blocks); i++) {
        MemoryRegion *mr = (MemoryRegion *)((char *)s + blocks[i].mr);
        g_autofree char *name = g_strdup_printf("%s.%s", TYPE_INGENIC_AS_DMIC,
                                                blocks[i].name);

        memory_region_init_io(mr, obj, blocks[i].ops, s, name, 0x1000);
        memory_region_add_subregion(&s->iomem, blocks[i].offset, mr);
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, as_tick, s);
}

static int as_post_load(void *opaque, int version_id)
{
    IngenicAsDmicState *s = opaque;

    if (s->running) {
        ingenic_mic_open(&s->mic_src, s->audio_be, s->rate);
        ingenic_mic_set_active(&s->mic_src, true);
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_as_channel = {
    .name = TYPE_INGENIC_AS_DMIC "/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dba, IngenicAsChannel),
        VMSTATE_UINT32(dtc, IngenicAsChannel),
        VMSTATE_UINT32(dcr, IngenicAsChannel),
        VMSTATE_UINT32(dsr, IngenicAsChannel),
        VMSTATE_UINT32(dcm, IngenicAsChannel),
        VMSTATE_UINT32(dda, IngenicAsChannel),
        VMSTATE_UINT32(flags, IngenicAsChannel),
        VMSTATE_UINT32(next, IngenicAsChannel),
        VMSTATE_BOOL(busy, IngenicAsChannel),
        VMSTATE_UINT32(fas, IngenicAsChannel),
        VMSTATE_UINT32(fcr, IngenicAsChannel),
        VMSTATE_UINT32(ffr, IngenicAsChannel),
        VMSTATE_UINT32(fsr, IngenicAsChannel),
        VMSTATE_UINT8_ARRAY(fifo, IngenicAsChannel, INGENIC_AS_FIFO_BYTES),
        VMSTATE_UINT32(rd, IngenicAsChannel),
        VMSTATE_UINT32(n, IngenicAsChannel),
        VMSTATE_UINT32(dfcr, IngenicAsChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_as_dmic = {
    .name = TYPE_INGENIC_AS_DMIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = as_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, IngenicAsDmicState, INGENIC_AS_CHANNELS, 1,
                             vmstate_ingenic_as_channel, IngenicAsChannel),
        VMSTATE_UINT32(dger, IngenicAsDmicState),
        VMSTATE_UINT32(aeer, IngenicAsDmicState),
        VMSTATE_UINT32(aesr, IngenicAsDmicState),
        VMSTATE_UINT32(btsr, IngenicAsDmicState),
        VMSTATE_UINT32(bfsr, IngenicAsDmicState),
        VMSTATE_UINT32(bfcr1, IngenicAsDmicState),
        VMSTATE_UINT32(bfcr2, IngenicAsDmicState),
        VMSTATE_UINT32_ARRAY(bst, IngenicAsDmicState, 3),
        VMSTATE_UINT32_ARRAY(btt, IngenicAsDmicState, 2),
        VMSTATE_UINT32(cr0, IngenicAsDmicState),
        VMSTATE_UINT32(gcr, IngenicAsDmicState),
        VMSTATE_UINT32(ier, IngenicAsDmicState),
        VMSTATE_UINT32(icr, IngenicAsDmicState),
        VMSTATE_UINT32(tcr, IngenicAsDmicState),
        VMSTATE_BOOL(running, IngenicAsDmicState),
        VMSTATE_UINT32(rate, IngenicAsDmicState),
        VMSTATE_INT64(t0, IngenicAsDmicState),
        VMSTATE_UINT64(frames, IngenicAsDmicState),
        VMSTATE_TIMER_PTR(timer, IngenicAsDmicState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property as_props[] = {
    DEFINE_AUDIO_PROPERTIES(IngenicAsDmicState, audio_be),
    DEFINE_PROP_STRING("mic", IngenicAsDmicState, mic),
};

static void as_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = as_realize;
    dc->unrealize = as_unrealize;
    dc->vmsd = &vmstate_ingenic_as_dmic;
    rc->phases.hold = as_reset_hold;
    rc->phases.exit = as_reset_exit;
    device_class_set_props(dc, as_props);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo as_type_info = {
    .name = TYPE_INGENIC_AS_DMIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicAsDmicState),
    .instance_init = as_init,
    .class_init = as_class_init,
};

static void as_register_types(void)
{
    type_register_static(&as_type_info);
}

type_init(as_register_types)
