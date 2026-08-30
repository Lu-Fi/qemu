/*
 * Ingenic XBurst2 CCU (Core Control Unit) emulation
 *
 * SMP control block of the XBurst2 SoCs: per-core software reset, per-core
 * sleep status, the inter-processor mailbox and per-core interrupt masks.
 * See the XBurst2 CPU Core Programming Manual, chapter 8.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/cpu.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/intc/ingenic-xburst2-ccu.h"

/* Register offsets (XBurst2 Core PM, Table 8-1). */
#define CCU_CSCR    0x000
#define CCU_CSSR    0x020
#define CCU_CSRR    0x040
#define CCU_MSCR    0x060
#define CCU_MSIR    0x064
#define CCU_CCR     0x070
#define CCU_PIPR    0x100
#define CCU_PIMR    0x120
#define CCU_MIPR    0x140
#define CCU_MIMR    0x160
#define CCU_OIPR    0x180
#define CCU_OIMR    0x1a0
#define CCU_DIPR    0x1c0
#define CCU_DIMR    0x1e0
#define CCU_RER     0xf00
#define CCU_CSLR    0xfa0
#define CCU_CSAR    0xfa4
#define CCU_GIMR    0xfc0
#define CCU_CFCR    0xfe0
#define CCU_MBR0    0x1000
#define CCU_BCER    0x1f00

/* MSIR: ProcessorID 0x20 in bits [15:8], revision 0. */
#define CCU_MSIR_VALUE      0x00002000

/* CSLR.Lock - set while the spinlock is held. */
#define CCU_CSLR_LOCK       0x80000000

/* CFCR.EnSRE0Wr - makes CSRR.SRE0 writable. */
#define CCU_CFCR_ENSRE0WR   (1u << 10)

static uint32_t ccu_core_mask(IngenicXBurst2CcuState *s)
{
    if (s->num_cpus >= 32) {
        return 0xffffffff;
    }
    return (1u << s->num_cpus) - 1;
}

/* Pack per-core input levels into a pending bitmask, one bit per core. */
static uint32_t ccu_level_mask(IngenicXBurst2CcuState *s, const bool *level)
{
    uint32_t v = 0;

    for (unsigned i = 0; i < s->num_cpus; i++) {
        if (level[i]) {
            v |= 1u << i;
        }
    }
    return v;
}

/*
 * Recompute the per-core interrupt outputs. An interrupt class reaches
 * core N when its pending bit, its per-core mask and the global mask
 * GIMR.IM<N> are all set.
 */
static void ccu_update_irqs(IngenicXBurst2CcuState *s)
{
    for (unsigned i = 0; i < s->num_cpus; i++) {
        bool g = (s->gimr >> i) & 1;

        qemu_set_irq(s->periph_irq[i],
                     s->intc_level[i] && ((s->pimr >> i) & 1) && g);
        qemu_set_irq(s->mailbox_irq[i],
                     ((s->mipr >> i) & 1) && ((s->mimr >> i) & 1) && g);
        qemu_set_irq(s->ost_irq[i],
                     s->ost_level[i] && ((s->oimr >> i) & 1) && g);
    }
}

/*
 * CSSR is not a stored register: SS<N> means core N has executed WAIT and
 * is sleeping. A core held in software reset (CSRR.SRE<N>=1) is halted
 * too, but that is reset, not sleep, so it is excluded.
 */
static uint32_t ccu_cssr(IngenicXBurst2CcuState *s)
{
    uint32_t val = 0;

    for (unsigned i = 0; i < s->num_cpus; i++) {
        if (!((s->csrr >> i) & 1) && s->cpu[i] && s->cpu[i]->halted) {
            val |= 1u << i;
        }
    }
    return val;
}

static void ccu_release_core(IngenicXBurst2CcuState *s, unsigned n)
{
    CPUState *cs = s->cpu[n];

    if (!cs) {
        return;
    }
    cpu_reset(cs);
    cpu_set_pc(cs, s->rer);
    cs->halted = 0;
    qemu_cpu_kick(cs);
}

static void ccu_hold_core(IngenicXBurst2CcuState *s, unsigned n)
{
    CPUState *cs = s->cpu[n];

    if (!cs) {
        return;
    }
    cs->halted = 1;
    qemu_cpu_kick(cs);
}

/*
 * CSRR.SRE<N>: 1 = core N held in reset, 0 = running. A 1->0 transition
 * releases the core at the address in RER; 0->1 holds it in reset.
 */
static void ccu_csrr_write(IngenicXBurst2CcuState *s, uint32_t val)
{
    uint32_t prev = s->csrr;

    /* SRE0 is read-only unless CFCR.EnSRE0Wr is set. */
    if (!(s->cfcr & CCU_CFCR_ENSRE0WR)) {
        val = (val & ~1u) | (prev & 1u);
    }
    s->csrr = val;

    for (unsigned i = 0; i < s->num_cpus; i++) {
        bool was = (prev >> i) & 1;
        bool now = (val >> i) & 1;

        if (was && !now) {
            ccu_release_core(s, i);
        } else if (!was && now) {
            ccu_hold_core(s, i);
        }
    }
}

/*
 * Writing a nonzero message to MBR<N> raises core N's mailbox IRQ (it
 * sets MIPR.IP<N>); writing zero clears it. The IRQ is level-triggered.
 */
static void ccu_mbr_write(IngenicXBurst2CcuState *s, unsigned n, uint32_t val)
{
    s->mbr[n] = val;
    if (val) {
        s->mipr |= 1u << n;
    } else {
        s->mipr &= ~(1u << n);
    }
    ccu_update_irqs(s);
}

/* INTC per-core output - routed to core <n>'s IP2 via PIPR/PIMR/GIMR. */
static void ccu_intc_in(void *opaque, int n, int level)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(opaque);

    if (n < INGENIC_XBURST2_CCU_MAX_CORES) {
        s->intc_level[n] = level;
        ccu_update_irqs(s);
    }
}

/* Core OST output for core <n> - routed to its IP4 via OIPR/OIMR/GIMR. */
static void ccu_ost_in(void *opaque, int n, int level)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(opaque);

    if (n < INGENIC_XBURST2_CCU_MAX_CORES) {
        s->ost_level[n] = level;
        ccu_update_irqs(s);
    }
}

static uint64_t ccu_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(opaque);

    if (offset >= CCU_MBR0 && offset < CCU_MBR0 + s->num_cpus * 4 &&
        !(offset & 3)) {
        return s->mbr[(offset - CCU_MBR0) / 4];
    }

    switch (offset) {
    case CCU_CSCR:  return s->cscr;
    case CCU_CSSR:  return ccu_cssr(s);
    case CCU_CSRR:  return s->csrr;
    case CCU_MSCR:  return s->mscr;
    case CCU_MSIR:  return CCU_MSIR_VALUE;
    case CCU_CCR:   return s->num_cpus ? s->num_cpus - 1 : 0;
    case CCU_PIPR:  return ccu_level_mask(s, s->intc_level);
    case CCU_PIMR:  return s->pimr;
    case CCU_MIPR:  return s->mipr;
    case CCU_MIMR:  return s->mimr;
    case CCU_OIPR:  return ccu_level_mask(s, s->ost_level);
    case CCU_OIMR:  return s->oimr;
    case CCU_DIPR:  return 0;
    case CCU_DIMR:  return s->dimr;
    case CCU_RER:   return s->rer;
    case CCU_CSLR:  return s->cslr;
    case CCU_CSAR:  return s->csar;
    case CCU_GIMR:  return s->gimr;
    case CCU_CFCR:  return s->cfcr;
    case CCU_BCER:  return s->bcer;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: read from unknown offset 0x%03"
                      HWADDR_PRIx "\n", __func__, offset);
        return 0;
    }
}

static void ccu_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(opaque);
    uint32_t val = value;

    if (offset >= CCU_MBR0 && offset < CCU_MBR0 + s->num_cpus * 4 &&
        !(offset & 3)) {
        ccu_mbr_write(s, (offset - CCU_MBR0) / 4, val);
        return;
    }

    switch (offset) {
    case CCU_CSCR:  s->cscr = val; break;
    case CCU_CSRR:  ccu_csrr_write(s, val); break;
    case CCU_MSCR:  s->mscr = val; break;
    case CCU_PIMR:  s->pimr = val; ccu_update_irqs(s); break;
    case CCU_MIMR:  s->mimr = val; ccu_update_irqs(s); break;
    case CCU_OIMR:  s->oimr = val; ccu_update_irqs(s); break;
    case CCU_DIMR:  s->dimr = val; break;
    case CCU_RER:   s->rer = val; break;
    case CCU_GIMR:  s->gimr = val; ccu_update_irqs(s); break;
    case CCU_CFCR:  s->cfcr = val; break;
    case CCU_BCER:  s->bcer = val; break;
    case CCU_CSLR:
        /* RW0: writing Lock=0 releases the spinlock; Value is read-only. */
        if (!(val & CCU_CSLR_LOCK)) {
            s->cslr = 0;
        }
        break;
    case CCU_CSAR:
        /* If the lock is free, atomically take it with the written value. */
        s->csar = val & ~CCU_CSLR_LOCK;
        if (!(s->cslr & CCU_CSLR_LOCK)) {
            s->cslr = CCU_CSLR_LOCK | (val & ~CCU_CSLR_LOCK);
        }
        break;
    case CCU_CSSR:
    case CCU_MSIR:
    case CCU_CCR:
    case CCU_PIPR:
    case CCU_MIPR:
    case CCU_OIPR:
    case CCU_DIPR:
        /* Read-only registers. */
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "%s: write 0x%08x to unknown offset 0x%03"
                      HWADDR_PRIx "\n", __func__, val, offset);
        break;
    }
}

static const MemoryRegionOps ccu_ops = {
    .read = ccu_read,
    .write = ccu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ccu_reset_hold(Object *obj, ResetType type)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(obj);
    uint32_t mask = ccu_core_mask(s);

    s->cscr = mask;             /* all SM<N>=1 */
    s->csrr = mask & ~1u;       /* core 0 running, secondaries in reset */
    s->mscr = 0;
    s->pimr = 1;                /* IM0=1 */
    s->mipr = 0;
    s->mimr = 0;
    s->oimr = 1;                /* IM0=1 */
    s->dimr = 1;                /* IM0=1 */
    s->rer = 0xbfc00000;
    s->cslr = 0;
    s->csar = 0;
    s->gimr = mask;             /* all IM<N>=1 */
    s->cfcr = 0;
    s->bcer = 0x000fffff;
    memset(s->mbr, 0, sizeof(s->mbr));

    memset(s->intc_level, 0, sizeof(s->intc_level));
    memset(s->ost_level, 0, sizeof(s->ost_level));
    ccu_update_irqs(s);

    /*
     * Re-park the secondaries. CSRR above is only register state; a
     * warm reset must also halt the vCPUs, or a core that was running
     * free-runs through the next boot's memory rewrite until the
     * kernel's release finds it in an undefined state.
     */
    for (unsigned i = 1; i < s->num_cpus; i++) {
        ccu_hold_core(s, i);
    }
}

static void ccu_realize(DeviceState *dev, Error **errp)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(dev);

    if (s->num_cpus < 1 || s->num_cpus > INGENIC_XBURST2_CCU_MAX_CORES) {
        error_setg(errp, "num-cpus must be between 1 and %d",
                   INGENIC_XBURST2_CCU_MAX_CORES);
        return;
    }
    for (unsigned i = 0; i < s->num_cpus; i++) {
        s->cpu[i] = qemu_get_cpu(i);
    }
}

static void ccu_init(Object *obj)
{
    IngenicXBurst2CcuState *s = INGENIC_XBURST2_CCU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ccu_ops, s,
                          TYPE_INGENIC_XBURST2_CCU,
                          INGENIC_XBURST2_CCU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_out_named(dev, s->periph_irq, "irq-ip2",
                             INGENIC_XBURST2_CCU_MAX_CORES);
    qdev_init_gpio_out_named(dev, s->mailbox_irq, "irq-ip3",
                             INGENIC_XBURST2_CCU_MAX_CORES);
    qdev_init_gpio_out_named(dev, s->ost_irq, "irq-ip4",
                             INGENIC_XBURST2_CCU_MAX_CORES);
    qdev_init_gpio_in_named(dev, ccu_intc_in, "intc-in",
                            INGENIC_XBURST2_CCU_MAX_CORES);
    qdev_init_gpio_in_named(dev, ccu_ost_in, "ost-in",
                            INGENIC_XBURST2_CCU_MAX_CORES);
}

static const Property ccu_properties[] = {
    DEFINE_PROP_UINT32("num-cpus", IngenicXBurst2CcuState, num_cpus, 1),
};

static int ccu_post_load(void *opaque, int version_id)
{
    ccu_update_irqs(opaque);
    return 0;
}

static const VMStateDescription vmstate_ingenic_xburst2_ccu = {
    .name = TYPE_INGENIC_XBURST2_CCU,
    .version_id = 3,
    .minimum_version_id = 3,
    .post_load = ccu_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cscr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(csrr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(mscr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(pimr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(mipr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(mimr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(oimr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(dimr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(rer, IngenicXBurst2CcuState),
        VMSTATE_UINT32(cslr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(csar, IngenicXBurst2CcuState),
        VMSTATE_UINT32(gimr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(cfcr, IngenicXBurst2CcuState),
        VMSTATE_UINT32(bcer, IngenicXBurst2CcuState),
        VMSTATE_UINT32_ARRAY(mbr, IngenicXBurst2CcuState,
                             INGENIC_XBURST2_CCU_MAX_CORES),
        VMSTATE_BOOL_ARRAY(intc_level, IngenicXBurst2CcuState,
                           INGENIC_XBURST2_CCU_MAX_CORES),
        VMSTATE_BOOL_ARRAY(ost_level, IngenicXBurst2CcuState,
                           INGENIC_XBURST2_CCU_MAX_CORES),
        VMSTATE_END_OF_LIST()
    },
};

static void ccu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ccu_realize;
    dc->vmsd = &vmstate_ingenic_xburst2_ccu;
    dc->user_creatable = false;
    device_class_set_props(dc, ccu_properties);
    rc->phases.hold = ccu_reset_hold;
}

static const TypeInfo ccu_type_info = {
    .name = TYPE_INGENIC_XBURST2_CCU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicXBurst2CcuState),
    .instance_init = ccu_init,
    .class_init = ccu_class_init,
};

static void ccu_register_types(void)
{
    type_register_static(&ccu_type_info);
}

type_init(ccu_register_types)
