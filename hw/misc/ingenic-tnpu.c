/*
 * Ingenic TNPU: the T32's, T33's and T32Pro's neural engine's RISC-V cores
 *
 * A small RV32IMCU core (machine mode, IDs reading 0) that the vendor's
 * tnpu.ko boots and hands tasks: its control unit (CCU) at 0x12700000 and
 * a window of RAM for its boot code, stack and the tasks it shares with
 * the host, at 0x12608000 (4 KiB) on the T32 and T33 and 0x150b0000
 * (8 KiB) on the T32Pro. The core sees the SoC's map as the MIPS side
 * does, DDR included.
 *
 * SOFT_RESET bit 31 holds the core (as from power-on); cleared, the core
 * starts at CRER (the window's base) + 0x80. FROMHOST, written by the
 * host, is the core's machine software interrupt until the core clears
 * it; TOHOST, written by the core, is the SoC's interrupt until the host
 * clears it. TIME counts at the TNPU clock (the CPM's TNPUCDR) while CCSR
 * bit 5 is set, and is the core's own cycle count; TIME_CMP reached is its
 * timer interrupt. CCSR bit 4 says the core is released, bits 3 and 8 that
 * it is asleep, bit 9 that it is held or asleep. The core is taken to be
 * asleep from its read of CCSR, which the vendor's boot code makes just
 * before its wfi, to the next FROMHOST. LOCK takes a write only while its
 * bit 31 is clear, which the write sets and an UNLOCK write clears.
 *
 * The T32Pro's TNPU has two cores, v0 and v1, and a 16 KiB CCU laid out
 * as its tnpu.ko has it: each core's CRER, FROMHOST and TOHOST at +4, +8
 * and +0xc of its block (0x1000 for v0, 0x2000 for v1), with words at
 * +0x10, +0x20 and +0x24 its boot code's interrupt handler uses; SOFT_RESET
 * bit 31 holds v0 and bit 30 v1 (v1 starts at its CRER, v0 at +0x80); CCSR
 * bits 6 and 7 say v0 and v1 are asleep, bit 3 that every released core
 * is; INTC_MASK_H (0x24), INTC_PENDING (0x2c), its memory protection words
 * (0x40 to 0x7c) and LOCK(n) (0x200 + 4n) plain words; BECR bits 1, 2 and 4
 * v0, the ORAM and v1 idle. Its boot code reads nothing of the CCU before
 * its wfi: a core is asleep from its wfi, which its QEMU tells, to the
 * next FROMHOST. Either core's TOHOST is the SoC's interrupt.
 *
 * With "chardev" set the core runs in a qemu-system-riscv32 (machine
 * ingenic-tnpu; the T32Pro's ingenic-t32pro-tnpu, its v1 through
 * "chardev1") linked over that socket, as for the fast-boot core (see
 * include/hw/misc/ingenic-riscv-link.h): DDR and the window are shared,
 * the CCU and everything else are reached across the link. Unlinked, no
 * core runs: released, it never says it is asleep, and tnpu.ko gives up
 * on it as it would on a part whose core does not start.
 *
 * Registers, their resets and the bits they keep are a T32LQ's, read with
 * the core held, booted with the vendor's code and running a task of ours.
 * The T32Pro's, with no board to read, are its driver's and boot code's
 * (its ID 0x150, its PMA words' resets as the driver documents them), the
 * rest as the T32's. Not modelled: the T32Pro's other windows (its CVA,
 * NNA, DMAs, LUT and ORAM), what raises its INTC_PENDING, and the tensor
 * units' interrupts (12, 13).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/blocker.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "system/ramblock.h"
#include "hw/misc/ingenic-tnpu.h"
#include "trace.h"

#define CCU_CCSR            0x00
#define CCU_CRER            0x04
#define CCU_FROMHOST        0x08
#define CCU_TOHOST          0x0c
#define CCU_TIME_L          0x10
#define CCU_TIME_H          0x14
#define CCU_TIME_CMP_L      0x18
#define CCU_TIME_CMP_H      0x1c
#define CCU_INTC_MASK_H     0x24        /* the T32Pro's */
#define CCU_INTC_PENDING    0x2c        /* the T32Pro's */
#define CCU_ERROR_EPC       0x30
#define CCU_INDEX           0x34
#define CCU_ID              0x38
#define CCU_MISC            0x40
#define CCU_SOFT_RESET      0xf0
#define CCU_LOCK            0xf4
#define CCU_UNLOCK          0xf8
#define CCU_CSCR            0xfc
#define CCU_BECR            0x100
#define CCU_LOCKS           0x200       /* the T32Pro's */
#define CCU_CORE            0x1000      /* the T32Pro's cores' blocks */
#define CORE_INTC           0x10        /* +0x10, +0x20, +0x24 */

#define CCSR_RW             0x21
#define CCSR_TIMER          (1 << 5)
#define CCSR_RELEASED       (1 << 4)
#define CCSR_ASLEEP         ((1 << 3) | (1 << 8))
#define CCSR_STOPPED        (1 << 9)
#define CCSR_FIXED          0x1c00
#define CCSR_PRO_RW         0x27
#define CCSR_PRO_ASLEEP     (1 << 3)
#define CCSR_PRO_CORE(n)    (1 << (6 + (n)))
#define SOFT_RESET_HOLD     (1u << 31)
#define SOFT_RESET_PRO(n)   (1u << (31 - (n)))
#define LOCK_HELD           (1u << 31)
#define BECR_PRO_IDLE       0x16

#define CORE_IRQ_SOFT       3
#define CORE_IRQ_TIMER      7

#define T32_DRAM            0x12608000
#define T32PRO_DRAM         0x150b0000

/* 0x40 to 0x88: resets and the bits each keeps */
static const uint32_t misc_reset[INGENIC_TNPU_MISC] = {
    0, 0, 0, 0, 0, 0, 0x05ffffff, 0x0fffffff,
    7, 7, 7, 7, 7, 7, 0x1f, 0x3f, 0x002001fd, 0, 0,
};
static const uint32_t misc_mask[INGENIC_TNPU_MISC] = {
    ~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u,
    0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x3f, 0x81ffffff, 1, 1,
};

/* The T32Pro's: PMA_ADDRESS5 and PMA_CONFIG5 as its driver documents them */
static const uint32_t misc_reset_pro[INGENIC_TNPU_MISC] = {
    0, 0, 0, 0, 0, 0x23ffffff, 0x05ffffff, 0x0fffffff,
    7, 7, 7, 7, 7, 0x3f, 0x1f, 0x3f, 0x002001fd, 0, 0,
};

static unsigned tnpu_cores(IngenicTnpuState *s)
{
    return s->pro ? INGENIC_TNPU_CORES : 1;
}

static bool tnpu_linked(IngenicTnpuState *s)
{
    return qemu_chr_fe_backend_connected(&s->core[0].link);
}

static bool tnpu_released(IngenicTnpuState *s, unsigned n)
{
    return !(s->soft_reset & (s->pro ? SOFT_RESET_PRO(n) : SOFT_RESET_HOLD));
}

static void link_send(IngenicTnpuCore *c, uint32_t type, uint32_t addr,
                      uint32_t size, uint32_t data, int fd)
{
    IngenicRiscvMsg m = {
        .type = cpu_to_le32(type),
        .addr = cpu_to_le32(addr),
        .size = cpu_to_le32(size),
        .data = cpu_to_le32(data),
    };

    if (!c->hello) {
        return;
    }
    if (fd >= 0 && qemu_chr_fe_set_msgfds(&c->link, &fd, 1) < 0) {
        error_report("ingenic-tnpu: the core's link cannot pass memory; "
                     "it must be a UNIX socket");
        return;
    }
    if (type == IRL_IRQ || type == IRL_RUN) {
        /* a WFI the core sent before this one arrives is stale */
        c->wakes++;
        if (data) {
            c->asleep = false;
        }
    }
    qemu_chr_fe_write_all(&c->link, (const uint8_t *)&m, sizeof(m));
}

/* TIME, at the TNPU clock while CCSR bit 5 is set */
static bool tnpu_counting(IngenicTnpuState *s)
{
    return (s->ccsr & CCSR_TIMER) && clock_get_hz(s->clk);
}

static uint64_t tnpu_time(IngenicTnpuState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!tnpu_counting(s)) {
        return s->time_base;
    }
    return s->time_base + muldiv64(now - s->time_ns, clock_get_hz(s->clk),
                                   NANOSECONDS_PER_SECOND);
}

static void tnpu_rebase(IngenicTnpuState *s, uint64_t count)
{
    s->time_base = count;
    s->time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void tnpu_update(IngenicTnpuState *s)
{
    uint64_t now = tnpu_time(s);
    bool timer = now >= s->time_cmp;
    bool tohost = false;

    if (timer) {
        timer_del(s->cmp_timer);
    } else if (tnpu_counting(s) &&
               s->time_cmp - now < clock_get_hz(s->clk) * (uint64_t)INT32_MAX) {
        /* a compare more than 68 years off is never met */
        timer_mod(s->cmp_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  muldiv64(s->time_cmp - now, NANOSECONDS_PER_SECOND,
                           clock_get_hz(s->clk)) + 1);
    } else {
        timer_del(s->cmp_timer);
    }
    for (unsigned n = 0; n < tnpu_cores(s); n++) {
        IngenicTnpuCore *c = &s->core[n];
        uint32_t lines = (c->fromhost ? BIT(CORE_IRQ_SOFT) : 0) |
                         (timer ? BIT(CORE_IRQ_TIMER) : 0);

        for (int i = 0; i < 32; i++) {
            if ((lines ^ c->irqs) & BIT(i)) {
                link_send(c, IRL_IRQ, i, 0, extract32(lines, i, 1), -1);
            }
        }
        c->irqs = lines;
        tohost |= c->tohost != 0;
    }
    qemu_set_irq(s->irq, tohost);
}

static void tnpu_cmp_fire(void *opaque)
{
    tnpu_update(opaque);
}

static void tnpu_run(IngenicTnpuState *s, unsigned n, bool run)
{
    IngenicTnpuCore *c = &s->core[n];

    c->asleep = false;
    link_send(c, IRL_RUN, c->crer, 0, run, -1);
}

static uint32_t tnpu_ccsr(IngenicTnpuState *s)
{
    IngenicTnpuCore *c = &s->core[0];
    bool released = tnpu_released(s, 0);
    bool running = released && !c->asleep;

    return s->ccsr | CCSR_FIXED | (released ? CCSR_RELEASED : 0) |
           (c->asleep ? CCSR_ASLEEP : 0) | (running ? 0 : CCSR_STOPPED);
}

static uint32_t tnpu_pro_ccsr(IngenicTnpuState *s)
{
    uint32_t v = s->ccsr;
    bool any = false, all = true;

    for (unsigned n = 0; n < INGENIC_TNPU_CORES; n++) {
        if (s->core[n].asleep) {
            v |= CCSR_PRO_CORE(n);
        }
        if (tnpu_released(s, n)) {
            any = true;
            all &= s->core[n].asleep;
        }
    }
    return v | (any && all ? CCSR_PRO_ASLEEP : 0);
}

/* The T32Pro's words, but for those it shares with the T32 */
static bool tnpu_pro_read(IngenicTnpuState *s, hwaddr offset, uint64_t *data)
{
    if (offset >= CCU_CORE) {
        unsigned n = offset / CCU_CORE - 1;
        IngenicTnpuCore *c = &s->core[MIN(n, INGENIC_TNPU_CORES - 1)];

        switch (n < INGENIC_TNPU_CORES ? offset % CCU_CORE : 0) {
        case CCU_CRER:
            *data = c->crer;
            break;
        case CCU_FROMHOST:
            *data = c->fromhost;
            break;
        case CCU_TOHOST:
            *data = c->tohost;
            break;
        case CORE_INTC:
            *data = c->intc[0];
            break;
        case CORE_INTC + 0x10:
            *data = c->intc[1];
            break;
        case CORE_INTC + 0x14:
            *data = c->intc[2];
            break;
        default:
            *data = 0;
            break;
        }
        return true;
    }
    switch (offset) {
    case CCU_CCSR:
        *data = tnpu_pro_ccsr(s);
        return true;
    case CCU_CRER:
    case CCU_FROMHOST:
    case CCU_TOHOST:
    case CCU_LOCK:
    case CCU_UNLOCK:
        *data = 0;
        return true;
    case CCU_INTC_MASK_H:
        *data = s->intc_mask;
        return true;
    case CCU_INTC_PENDING:
        *data = 0;
        return true;
    case CCU_LOCKS ... CCU_LOCKS + 4 * (INGENIC_TNPU_LOCKS - 1):
        *data = s->locks[(offset - CCU_LOCKS) / 4];
        return true;
    default:
        return false;
    }
}

static MemTxResult tnpu_ccu_read(void *opaque, hwaddr offset, uint64_t *data,
                                 unsigned size, MemTxAttrs attrs)
{
    IngenicTnpuState *s = opaque;
    IngenicTnpuCore *c = &s->core[0];
    bool core = attrs.requester_id == INGENIC_TNPU_REQUESTER;

    if (s->pro && tnpu_pro_read(s, offset, data)) {
        return MEMTX_OK;
    }
    switch (offset) {
    case CCU_CCSR:
        *data = tnpu_ccsr(s);
        if (core && tnpu_released(s, 0)) {
            c->asleep = true;
        }
        break;
    case CCU_CRER:
        *data = c->crer;
        break;
    case CCU_FROMHOST:
        *data = c->fromhost;
        break;
    case CCU_TOHOST:
        *data = c->tohost;
        break;
    case CCU_TIME_L:
        *data = (uint32_t)tnpu_time(s);
        break;
    case CCU_TIME_H:
        *data = tnpu_time(s) >> 32;
        break;
    case CCU_TIME_CMP_L:
        *data = (uint32_t)s->time_cmp;
        break;
    case CCU_TIME_CMP_H:
        *data = s->time_cmp >> 32;
        break;
    case CCU_ERROR_EPC:
        *data = s->epc;
        break;
    case CCU_INDEX:
        *data = s->index;
        break;
    case CCU_ID:
        *data = s->pro ? 0x150 : 0x100;
        break;
    case CCU_MISC ... CCU_MISC + 4 * (INGENIC_TNPU_MISC - 1):
        *data = s->misc[(offset - CCU_MISC) / 4];
        break;
    case CCU_SOFT_RESET:
        *data = s->soft_reset;
        break;
    case CCU_LOCK:
        *data = s->lock;
        break;
    case CCU_CSCR:
        *data = s->cscr;
        break;
    case CCU_BECR:
        *data = s->becr;
        break;
    default:
        *data = 0;
        break;
    }
    return MEMTX_OK;
}

static bool tnpu_pro_write(IngenicTnpuState *s, hwaddr offset, uint32_t v,
                           bool core)
{
    if (offset >= CCU_CORE) {
        unsigned n = offset / CCU_CORE - 1;
        IngenicTnpuCore *c = &s->core[MIN(n, INGENIC_TNPU_CORES - 1)];

        switch (n < INGENIC_TNPU_CORES ? offset % CCU_CORE : 0) {
        case CCU_CRER:
            c->crer = v;
            break;
        case CCU_FROMHOST:
            c->fromhost = v;
            if (v && !core) {
                c->asleep = false;
            }
            break;
        case CCU_TOHOST:
            c->tohost = v;
            break;
        case CORE_INTC:
            c->intc[0] = v;
            break;
        case CORE_INTC + 0x10:
            c->intc[1] = v;
            break;
        case CORE_INTC + 0x14:
            c->intc[2] = v;
            break;
        default:
            break;
        }
        return true;
    }
    switch (offset) {
    case CCU_CCSR: {
        uint64_t now = tnpu_time(s);

        s->ccsr = v & CCSR_PRO_RW;
        tnpu_rebase(s, now);
        return true;
    }
    case CCU_CRER:
    case CCU_FROMHOST:
    case CCU_TOHOST:
    case CCU_LOCK:
    case CCU_UNLOCK:
    case CCU_INTC_PENDING:
        return true;
    case CCU_INTC_MASK_H:
        s->intc_mask = v;
        return true;
    case CCU_SOFT_RESET: {
        uint32_t was = s->soft_reset;

        s->soft_reset = v & (SOFT_RESET_PRO(0) | SOFT_RESET_PRO(1));
        for (unsigned n = 0; n < INGENIC_TNPU_CORES; n++) {
            if ((was ^ s->soft_reset) & SOFT_RESET_PRO(n)) {
                tnpu_run(s, n, tnpu_released(s, n));
            }
        }
        return true;
    }
    case CCU_BECR:
        s->becr = (v & 0x9) | BECR_PRO_IDLE;
        return true;
    case CCU_LOCKS ... CCU_LOCKS + 4 * (INGENIC_TNPU_LOCKS - 1):
        s->locks[(offset - CCU_LOCKS) / 4] = v;
        return true;
    default:
        return false;
    }
}

static MemTxResult tnpu_ccu_write(void *opaque, hwaddr offset, uint64_t value,
                                  unsigned size, MemTxAttrs attrs)
{
    IngenicTnpuState *s = opaque;
    IngenicTnpuCore *c = &s->core[0];
    bool core = attrs.requester_id == INGENIC_TNPU_REQUESTER;
    uint32_t v = value;
    uint64_t now;

    if (s->pro && tnpu_pro_write(s, offset, v, core)) {
        tnpu_update(s);
        return MEMTX_OK;
    }
    switch (offset) {
    case CCU_CCSR:
        now = tnpu_time(s);
        s->ccsr = v & CCSR_RW;
        tnpu_rebase(s, now);
        break;
    case CCU_CRER:
        c->crer = v;
        break;
    case CCU_FROMHOST:
        c->fromhost = v;
        if (v && !core) {
            c->asleep = false;
        }
        break;
    case CCU_TOHOST:
        c->tohost = v;
        break;
    case CCU_TIME_L:
        tnpu_rebase(s, deposit64(tnpu_time(s), 0, 32, v));
        break;
    case CCU_TIME_H:
        tnpu_rebase(s, deposit64(tnpu_time(s), 32, 32, v));
        break;
    case CCU_TIME_CMP_L:
        s->time_cmp = deposit64(s->time_cmp, 0, 32, v);
        break;
    case CCU_TIME_CMP_H:
        s->time_cmp = deposit64(s->time_cmp, 32, 32, v);
        break;
    case CCU_ERROR_EPC:
        s->epc = v;
        break;
    case CCU_INDEX:
        s->index = v;
        break;
    case CCU_MISC ... CCU_MISC + 4 * (INGENIC_TNPU_MISC - 1):
        s->misc[(offset - CCU_MISC) / 4] = v &
            misc_mask[(offset - CCU_MISC) / 4];
        break;
    case CCU_SOFT_RESET:
        if ((v ^ s->soft_reset) & SOFT_RESET_HOLD) {
            s->soft_reset = v & SOFT_RESET_HOLD;
            tnpu_run(s, 0, tnpu_released(s, 0));
        }
        break;
    case CCU_LOCK:
        if (!(s->lock & LOCK_HELD)) {
            s->lock = v | LOCK_HELD;
        }
        break;
    case CCU_UNLOCK:
        s->lock &= ~LOCK_HELD;
        break;
    case CCU_CSCR:
        s->cscr = v & 1;
        break;
    case CCU_BECR:
        s->becr = (v & 0x9) | 0x6;
        break;
    default:
        break;
    }
    tnpu_update(s);
    return MEMTX_OK;
}

static const MemoryRegionOps tnpu_ccu_ops = {
    .read_with_attrs = tnpu_ccu_read,
    .write_with_attrs = tnpu_ccu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static bool link_send_mem(IngenicTnpuCore *c, MemoryRegion *mr, hwaddr addr)
{
    int fd = memory_region_get_fd(mr);

    if (fd < 0 || qemu_ram_get_fd_offset(mr->ram_block)) {
        return false;
    }
    link_send(c, IRL_MEM, addr, memory_region_size(mr), 0, fd);
    return true;
}

static void link_hello(IngenicTnpuCore *c, uint32_t version)
{
    IngenicTnpuState *s = c->tnpu;
    unsigned n = c - s->core;
    MemoryRegionSection ddr;
    bool shared;

    if (version != INGENIC_RISCV_LINK_VERSION) {
        error_report("ingenic-tnpu: the core speaks link version %u, not %u",
                     version, INGENIC_RISCV_LINK_VERSION);
        qemu_chr_fe_disconnect(&c->link);
        return;
    }
    c->hello = true;
    ddr = memory_region_find(get_system_memory(), 0, 1);
    shared = ddr.mr && link_send_mem(c, ddr.mr, 0);
    if (ddr.mr) {
        memory_region_unref(ddr.mr);
    }
    if (!shared || !link_send_mem(c, &s->dram, s->dram.addr)) {
        error_report("ingenic-tnpu: the core needs DDR it can share: "
                     "-object memory-backend-memfd,id=ddr,size=<RAM>,"
                     "share=on -machine memory-backend=ddr");
        c->hello = false;
        qemu_chr_fe_disconnect(&c->link);
        return;
    }
    for (int i = 0; i < 32; i++) {
        if (c->irqs & BIT(i)) {
            link_send(c, IRL_IRQ, i, 0, 1, -1);
        }
    }
    if (s->t33) {
        link_send(c, IRL_VARIANT, 0, 0, 1, -1);
    }
    link_send(c, IRL_CLOCK, 0, 0, clock_get_hz(s->clk) != 0, -1);
    link_send(c, IRL_RUN, c->crer, 0, tnpu_released(s, n), -1);
}

/* An access by the core, made on the SoC's bus */
static void link_access(IngenicTnpuCore *c, const IngenicRiscvMsg *m)
{
    MemTxAttrs attrs = { .requester_id = INGENIC_TNPU_REQUESTER };
    uint32_t addr = le32_to_cpu(m->addr);
    uint32_t size = le32_to_cpu(m->size);
    uint32_t data = 0;
    uint8_t buf[4];

    if (size != 1 && size != 2 && size != 4) {
        error_report("ingenic-tnpu: the core sent a %u-byte access", size);
    } else if (le32_to_cpu(m->type) == IRL_READ) {
        address_space_read(&address_space_memory, addr, attrs, buf, size);
        data = ldm_p(buf, size_memop(size) | MO_LE);
        trace_ingenic_tnpu_core_access("read", addr, size, data);
    } else {
        stm_p(buf, size_memop(size) | MO_LE, le32_to_cpu(m->data));
        address_space_write(&address_space_memory, addr, attrs, buf, size);
        trace_ingenic_tnpu_core_access("write", addr, size,
                                       le32_to_cpu(m->data));
    }
    link_send(c, IRL_DATA, 0, 0, data, -1);
}

static int link_can_read(void *opaque)
{
    IngenicTnpuCore *c = opaque;

    return sizeof(c->rx) - c->rx_len;
}

static void link_read(void *opaque, const uint8_t *buf, int size)
{
    IngenicTnpuCore *c = opaque;
    IngenicTnpuState *s = c->tnpu;

    memcpy((uint8_t *)&c->rx + c->rx_len, buf, size);
    c->rx_len += size;
    if (c->rx_len < sizeof(c->rx)) {
        return;
    }
    c->rx_len = 0;
    switch (le32_to_cpu(c->rx.type)) {
    case IRL_HELLO:
        link_hello(c, le32_to_cpu(c->rx.data));
        break;
    case IRL_READ:
    case IRL_WRITE:
        if (c->hello) {
            link_access(c, &c->rx);
        }
        break;
    case IRL_WFI:
        if (s->pro && le32_to_cpu(c->rx.data) == c->wakes &&
            tnpu_released(s, c - s->core)) {
            c->asleep = true;
        }
        break;
    default:
        error_report("ingenic-tnpu: unexpected message %u from the core",
                     le32_to_cpu(c->rx.type));
        break;
    }
}

static void link_event(void *opaque, QEMUChrEvent event)
{
    IngenicTnpuCore *c = opaque;

    if (event == CHR_EVENT_CLOSED) {
        c->hello = false;
        c->rx_len = 0;
    }
}

static void tnpu_clk_update(void *opaque, ClockEvent event)
{
    IngenicTnpuState *s = opaque;

    if (event == ClockPreUpdate) {
        tnpu_rebase(s, tnpu_time(s));
        return;
    }
    s->time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    for (unsigned n = 0; n < tnpu_cores(s); n++) {
        link_send(&s->core[n], IRL_CLOCK, 0, 0, clock_get_hz(s->clk) != 0,
                  -1);
    }
    tnpu_update(s);
}

static void tnpu_reset_hold(Object *obj, ResetType type)
{
    IngenicTnpuState *s = INGENIC_TNPU(obj);

    s->ccsr = 1;
    for (unsigned n = 0; n < INGENIC_TNPU_CORES; n++) {
        IngenicTnpuCore *c = &s->core[n];

        c->asleep = false;
        c->crer = s->pro ? T32PRO_DRAM : T32_DRAM;
        c->fromhost = c->tohost = 0;
        memset(c->intc, 0, sizeof(c->intc));
    }
    tnpu_rebase(s, 0);
    s->time_cmp = UINT64_MAX;
    s->epc = 0x202;
    s->index = 0;
    s->soft_reset = s->pro ? SOFT_RESET_PRO(0) | SOFT_RESET_PRO(1)
                           : SOFT_RESET_HOLD;
    s->lock = 0;
    s->cscr = 1;
    s->becr = s->pro ? BECR_PRO_IDLE : 6;
    memcpy(s->misc, s->pro ? misc_reset_pro : misc_reset, sizeof(s->misc));
    s->intc_mask = 0;
    memset(s->locks, 0, sizeof(s->locks));
    for (unsigned n = 0; n < tnpu_cores(s); n++) {
        link_send(&s->core[n], IRL_RUN, s->core[n].crer, 0, 0, -1);
    }
    tnpu_update(s);
}

static void tnpu_realize(DeviceState *dev, Error **errp)
{
    IngenicTnpuState *s = INGENIC_TNPU(dev);

    if (!s->pro && qemu_chr_fe_backend_connected(&s->core[1].link)) {
        error_setg(errp, "ingenic-tnpu: only the T32Pro's has a core 1");
        return;
    }
    if (tnpu_linked(s)) {
        error_setg(&s->migration_blocker, "ingenic-tnpu: its cores run in "
                   "other QEMUs, whose state is not saved with this one");
        if (migrate_add_blocker(&s->migration_blocker, errp) < 0) {
            return;
        }
    }
    memory_region_init_io(&s->ccu, OBJECT(dev), &tnpu_ccu_ops, s,
                          "ingenic-tnpu.ccu", s->pro ? 0x4000 : 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ccu);
    /* Linked, the window is shared with the cores' QEMUs */
    if (!memory_region_init_ram_flags_nomigrate(&s->dram, OBJECT(dev),
            "ingenic-tnpu.dram", s->pro ? 0x2000 : 0x1000,
            tnpu_linked(s) ? RAM_SHARED : 0, errp)) {
        return;
    }
    vmstate_register_ram(&s->dram, dev);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->dram);
    s->cmp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tnpu_cmp_fire, s);
    for (unsigned n = 0; n < INGENIC_TNPU_CORES; n++) {
        IngenicTnpuCore *c = &s->core[n];

        c->tnpu = s;
        if (qemu_chr_fe_backend_connected(&c->link)) {
            qemu_chr_fe_set_handlers(&c->link, link_can_read, link_read,
                                     link_event, NULL, c, NULL, true);
        }
    }
}

static void tnpu_init(Object *obj)
{
    IngenicTnpuState *s = INGENIC_TNPU(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", tnpu_clk_update, s,
                                ClockPreUpdate | ClockUpdate);
}

static const Property tnpu_props[] = {
    DEFINE_PROP_BOOL("pro", IngenicTnpuState, pro, false),
    DEFINE_PROP_BOOL("t33", IngenicTnpuState, t33, false),
    DEFINE_PROP_CHR("chardev", IngenicTnpuState, core[0].link),
    DEFINE_PROP_CHR("chardev1", IngenicTnpuState, core[1].link),
};

static const VMStateDescription vmstate_tnpu_core = {
    .name = "ingenic-tnpu/core",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(irqs, IngenicTnpuCore),
        VMSTATE_UINT32(wakes, IngenicTnpuCore),
        VMSTATE_BOOL(asleep, IngenicTnpuCore),
        VMSTATE_UINT32(crer, IngenicTnpuCore),
        VMSTATE_UINT32(fromhost, IngenicTnpuCore),
        VMSTATE_UINT32(tohost, IngenicTnpuCore),
        VMSTATE_UINT32_ARRAY(intc, IngenicTnpuCore, INGENIC_TNPU_INTC),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_tnpu = {
    .name = "ingenic-tnpu",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(core, IngenicTnpuState, INGENIC_TNPU_CORES, 1,
                             vmstate_tnpu_core, IngenicTnpuCore),
        VMSTATE_UINT32(ccsr, IngenicTnpuState),
        VMSTATE_UINT64(time_base, IngenicTnpuState),
        VMSTATE_INT64(time_ns, IngenicTnpuState),
        VMSTATE_UINT64(time_cmp, IngenicTnpuState),
        VMSTATE_UINT32(epc, IngenicTnpuState),
        VMSTATE_UINT32(index, IngenicTnpuState),
        VMSTATE_UINT32(soft_reset, IngenicTnpuState),
        VMSTATE_UINT32(lock, IngenicTnpuState),
        VMSTATE_UINT32(cscr, IngenicTnpuState),
        VMSTATE_UINT32(becr, IngenicTnpuState),
        VMSTATE_UINT32_ARRAY(misc, IngenicTnpuState, INGENIC_TNPU_MISC),
        VMSTATE_UINT32(intc_mask, IngenicTnpuState),
        VMSTATE_UINT32_ARRAY(locks, IngenicTnpuState, INGENIC_TNPU_LOCKS),
        VMSTATE_TIMER_PTR(cmp_timer, IngenicTnpuState),
        VMSTATE_CLOCK(clk, IngenicTnpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void tnpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = tnpu_realize;
    dc->vmsd = &vmstate_tnpu;
    device_class_set_props(dc, tnpu_props);
    rc->phases.hold = tnpu_reset_hold;
}

static const TypeInfo tnpu_type_info = {
    .name = TYPE_INGENIC_TNPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicTnpuState),
    .instance_init = tnpu_init,
    .class_init = tnpu_class_init,
};

static void tnpu_register_types(void)
{
    type_register_static(&tnpu_type_info);
}

type_init(tnpu_register_types)
