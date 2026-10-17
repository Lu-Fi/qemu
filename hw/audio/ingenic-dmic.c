/*
 * Ingenic DMIC, the digital microphone controller of the T30, T31 and T40
 *
 * The controller turns up to four PDM microphones into 16-bit samples at
 * 8, 16 or 48 kHz and queues them, two to a word with PACK_EN, in a 64-word
 * FIFO that the PDMA drains from DMICDR while it holds DMICFTHR words or
 * more. The vendor drivers mask every interrupt and poll the DMA, as they
 * do for the AIC.
 *
 * As read on a T40NN: the reset values, CR0.RESET emptying the FIFO but
 * keeping the settings (the 4.4 driver configures, then resets and starts),
 * and an empty FIFO reading the word last read.
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
#include "migration/vmstate.h"
#include "hw/audio/ingenic-dmic.h"

#define DMICCR0     0x00
#define DMICGCR     0x04
#define DMICIMR     0x08
#define DMICINTCR   0x0c
#define DMICTRICR   0x10
#define DMICTHRH    0x14
#define DMICTHRL    0x18
#define DMICTRIMMAX 0x1c
#define DMICTRINMAX 0x20
#define DMICDR      0x30
#define DMICFTHR    0x34
#define DMICFSR     0x38
#define DMICCGDIS   0x50

#define CR0_RESET       BIT(31)
#define CR0_SW_LR       BIT(11)
#define CR0_PACK_EN     BIT(8)
#define CR0_EN          BIT(0)
#define CR0_MASK        0x00073dcf
#define CR0_CHNUM(v)    extract32(v, 16, 3)
#define CR0_SR(v)       extract32(v, 6, 2)

#define INT_FULL        BIT(2)
#define INT_EMPTY       BIT(3)
#define INT_FIFO        BIT(5)      /* the FIFO reached its threshold */
#define INT_MASK        0x3f

#define FTHR_RDMS       BIT(31)
#define FTHR_MASK       0x8000003f
#define FTHR_THR(v)     extract32(v, 0, 6)

#define FSR_FULL        BIT(19)
#define FSR_EMPTY       BIT(16)

/* SR 3 is the reset value; no driver sets it */
static const uint32_t dmic_rates[4] = { 8000, 16000, 48000, 96000 };

static void dmic_schedule(IngenicDmicState *s);

static unsigned dmic_thr(IngenicDmicState *s)
{
    return MAX(FTHR_THR(s->fthr), 1);
}

static bool dmic_dma_wanted(IngenicDmicState *s)
{
    return (s->cr0 & CR0_EN) && (s->fthr & FTHR_RDMS) &&
           s->fifo_n >= dmic_thr(s);
}

static void dmic_update(IngenicDmicState *s)
{
    qemu_set_irq(s->dreq, dmic_dma_wanted(s));
    qemu_set_irq(s->irq, !!(s->intcr & ~s->imr & INT_MASK));
}

static void dmic_fifo_reset(IngenicDmicState *s)
{
    s->fifo_rd = s->fifo_n = 0;
    s->half_full = false;
    s->intcr |= INT_EMPTY;
}

static void dmic_push(IngenicDmicState *s, uint32_t v)
{
    if (s->fifo_n == INGENIC_DMIC_FIFO) {
        s->intcr |= INT_FULL;
        return;
    }
    s->fifo[(s->fifo_rd + s->fifo_n) % INGENIC_DMIC_FIFO] = v;
    s->fifo_n++;
    if (s->fifo_n >= dmic_thr(s)) {
        s->intcr |= INT_FIFO;
    }
}

/* A sample from each microphone; with PACK_EN two to a word, first low */
static void dmic_frame(IngenicDmicState *s)
{
    unsigned chans = CR0_CHNUM(s->cr0) + 1;
    int16_t f[2];

    ingenic_mic_frame(&s->mic_src, s->rate, f);
    for (unsigned c = 0; c < chans; c++) {
        uint32_t x = (uint16_t)f[c & 1];

        if (!(s->cr0 & CR0_PACK_EN)) {
            dmic_push(s, (int32_t)f[c & 1]);
        } else if (!s->half_full) {
            s->half = x;
            s->half_full = true;
        } else {
            dmic_push(s, s->cr0 & CR0_SW_LR ? s->half << 16 | x
                                            : x << 16 | s->half);
            s->half_full = false;
        }
    }
}

/* --- The frame clock ---------------------------------------------------- */

static void dmic_clock_update(IngenicDmicState *s)
{
    bool run = s->cr0 & CR0_EN;
    uint32_t rate = dmic_rates[CR0_SR(s->cr0)];

    if (run && (!s->running || rate != s->rate)) {
        s->t0 = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->frames = 0;
        s->rate = rate;
        ingenic_mic_open(&s->mic_src, s->audio_be, rate);
    }
    s->running = run;
    ingenic_mic_set_active(&s->mic_src, run);
    if (run) {
        dmic_schedule(s);
    } else {
        timer_del(s->timer);
    }
}

static void dmic_tick(void *opaque)
{
    IngenicDmicState *s = opaque;
    uint64_t due, n;

    if (!s->running) {
        return;
    }
    due = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->t0, s->rate,
                   NANOSECONDS_PER_SECOND);
    n = due > s->frames ? due - s->frames : 0;
    while (n--) {
        dmic_frame(s);
        s->frames++;
        /* As for the AIC: in a late batch the PDMA drains between frames */
        if (n && s->pdma && dmic_dma_wanted(s)) {
            dmic_update(s);
            ingenic_pdma_kick(s->pdma);
        }
    }
    dmic_update(s);
    dmic_schedule(s);
}

/*
 * Wake when the FIFO reaches its threshold or, above it, fills; at least
 * every 10 ms to keep the host microphone flowing
 */
static void dmic_schedule(IngenicDmicState *s)
{
    unsigned chans = CR0_CHNUM(s->cr0) + 1;
    unsigned per_word = s->cr0 & CR0_PACK_EN ? 2 : 1;
    unsigned goal = s->fifo_n < dmic_thr(s) ? dmic_thr(s) : INGENIC_DMIC_FIFO;
    uint64_t k = MAX(s->rate / 100, 1);

    if (s->fifo_n < goal) {
        unsigned samples = (goal - s->fifo_n) * per_word - s->half_full;

        k = MIN(k, DIV_ROUND_UP(samples, chans));
    }
    k = MAX(k, 1);
    timer_mod(s->timer, s->t0 + muldiv64(s->frames + k, NANOSECONDS_PER_SECOND,
                                         s->rate) + 1);
}

/* --- Registers ---------------------------------------------------------- */

static uint64_t dmic_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicDmicState *s = opaque;
    uint32_t v;

    switch (addr) {
    case DMICCR0:
        return s->cr0;
    case DMICGCR:
        return s->gcr;
    case DMICIMR:
        return s->imr;
    case DMICINTCR:
        return s->intcr;
    case DMICTRICR:
        return s->tricr;
    case DMICTHRH:
        return s->thrh;
    case DMICTHRL:
        return s->thrl;
    case DMICTRIMMAX:
        return s->trimmax;
    case DMICTRINMAX:
        return s->trinmax;
    case DMICDR:
        if (s->fifo_n) {
            s->dr = s->fifo[s->fifo_rd];
            s->fifo_rd = (s->fifo_rd + 1) % INGENIC_DMIC_FIFO;
            s->fifo_n--;
            if (!s->fifo_n) {
                s->intcr |= INT_EMPTY;
            }
            dmic_update(s);
            if (s->fifo_n + 1 == dmic_thr(s) && s->running) {
                dmic_schedule(s);
            }
        }
        return s->dr;
    case DMICFTHR:
        return s->fthr;
    case DMICFSR:
        v = s->fifo_n & 0x3f;
        v |= s->fifo_n ? 0 : FSR_EMPTY;
        v |= s->fifo_n == INGENIC_DMIC_FIFO ? FSR_FULL : 0;
        return v;
    case DMICCGDIS:
        return s->cgdis;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-dmic: read of 0x%"
                      HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void dmic_write(void *opaque, hwaddr addr, uint64_t value,
                       unsigned size)
{
    IngenicDmicState *s = opaque;

    switch (addr) {
    case DMICCR0:
        if (value & CR0_RESET) {
            dmic_fifo_reset(s);
        }
        s->cr0 = value & CR0_MASK;
        dmic_clock_update(s);
        break;
    case DMICGCR:
        s->gcr = value & 0x1f;
        break;
    case DMICIMR:
        s->imr = value & INT_MASK;
        break;
    case DMICINTCR:
        /* Write 1 to clear, as elsewhere: no driver clears these */
        s->intcr &= ~(value & INT_MASK);
        break;
    case DMICTRICR:
        s->tricr = value & 0x000f001e;  /* TRI_CLR, bit 0, clears itself */
        break;
    case DMICTHRH:
        s->thrh = value & 0xfffff;
        break;
    case DMICTHRL:
        s->thrl = value & 0xfffff;
        break;
    case DMICTRIMMAX:
        s->trimmax = value & 0xffffff;
        break;
    case DMICTRINMAX:
        s->trinmax = value & 0xffff;
        break;
    case DMICFTHR:
        s->fthr = value & FTHR_MASK;
        break;
    case DMICCGDIS:
        s->cgdis = value;
        break;
    case DMICDR:
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-dmic: write of 0x%" PRIx64
                      " to 0x%" HWADDR_PRIx "\n", value, addr);
        return;
    }
    dmic_update(s);
    if (s->running) {
        dmic_schedule(s);
    }
}

static const MemoryRegionOps dmic_ops = {
    .read = dmic_read,
    .write = dmic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* --- Lifecycle ---------------------------------------------------------- */

static void dmic_realize(DeviceState *dev, Error **errp)
{
    IngenicDmicState *s = INGENIC_DMIC(dev);
    Error *err = NULL;

    if (!ingenic_mic_init(&s->mic_src, TYPE_INGENIC_DMIC, s->mic, errp)) {
        return;
    }
    /* As for the AIC, an audiodev that cannot start costs the host side */
    if (!audio_be_check(&s->audio_be, &err)) {
        warn_report_err(err);
    }
}

static void dmic_unrealize(DeviceState *dev)
{
    ingenic_mic_cleanup(&INGENIC_DMIC(dev)->mic_src);
}

static void dmic_reset_hold(Object *obj, ResetType type)
{
    IngenicDmicState *s = INGENIC_DMIC(obj);

    timer_del(s->timer);
    s->running = false;
    s->cr0 = 0xc8;
    s->gcr = 0;
    s->imr = INT_MASK;
    s->tricr = 0x00030000;
    s->thrh = 0x00020000;
    s->thrl = 0x147a;
    s->trimmax = 0x3e8;
    s->trinmax = 3;
    s->fthr = 0x20;
    s->cgdis = 0;
    s->dr = 0;
    dmic_fifo_reset(s);
    s->intcr = 0;
}

static void dmic_reset_exit(Object *obj, ResetType type)
{
    dmic_update(INGENIC_DMIC(obj));
}

static void dmic_init(Object *obj)
{
    IngenicDmicState *s = INGENIC_DMIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &dmic_ops, s, TYPE_INGENIC_DMIC,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(DEVICE(obj), &s->dreq, "dreq", 1);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, dmic_tick, s);
}

static int dmic_post_load(void *opaque, int version_id)
{
    IngenicDmicState *s = opaque;

    if (s->running) {
        ingenic_mic_open(&s->mic_src, s->audio_be, s->rate);
        ingenic_mic_set_active(&s->mic_src, true);
    }
    return 0;
}

static const VMStateDescription vmstate_ingenic_dmic = {
    .name = TYPE_INGENIC_DMIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = dmic_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr0, IngenicDmicState),
        VMSTATE_UINT32(gcr, IngenicDmicState),
        VMSTATE_UINT32(imr, IngenicDmicState),
        VMSTATE_UINT32(intcr, IngenicDmicState),
        VMSTATE_UINT32(tricr, IngenicDmicState),
        VMSTATE_UINT32(thrh, IngenicDmicState),
        VMSTATE_UINT32(thrl, IngenicDmicState),
        VMSTATE_UINT32(trimmax, IngenicDmicState),
        VMSTATE_UINT32(trinmax, IngenicDmicState),
        VMSTATE_UINT32(fthr, IngenicDmicState),
        VMSTATE_UINT32(cgdis, IngenicDmicState),
        VMSTATE_UINT32_ARRAY(fifo, IngenicDmicState, INGENIC_DMIC_FIFO),
        VMSTATE_UINT32(fifo_rd, IngenicDmicState),
        VMSTATE_UINT32(fifo_n, IngenicDmicState),
        VMSTATE_UINT32(dr, IngenicDmicState),
        VMSTATE_UINT32(half, IngenicDmicState),
        VMSTATE_BOOL(half_full, IngenicDmicState),
        VMSTATE_BOOL(running, IngenicDmicState),
        VMSTATE_UINT32(rate, IngenicDmicState),
        VMSTATE_INT64(t0, IngenicDmicState),
        VMSTATE_UINT64(frames, IngenicDmicState),
        VMSTATE_TIMER_PTR(timer, IngenicDmicState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property dmic_props[] = {
    DEFINE_AUDIO_PROPERTIES(IngenicDmicState, audio_be),
    DEFINE_PROP_STRING("mic", IngenicDmicState, mic),
    DEFINE_PROP_LINK("pdma", IngenicDmicState, pdma, TYPE_INGENIC_PDMA,
                     IngenicPdmaState *),
};

static void dmic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = dmic_realize;
    dc->unrealize = dmic_unrealize;
    dc->vmsd = &vmstate_ingenic_dmic;
    rc->phases.hold = dmic_reset_hold;
    rc->phases.exit = dmic_reset_exit;
    device_class_set_props(dc, dmic_props);
    set_bit(DEVICE_CATEGORY_SOUND, dc->categories);
}

static const TypeInfo dmic_type_info = {
    .name = TYPE_INGENIC_DMIC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDmicState),
    .instance_init = dmic_init,
    .class_init = dmic_class_init,
};

static void dmic_register_types(void)
{
    type_register_static(&dmic_type_info);
}

type_init(dmic_register_types)
