/*
 * Ingenic camera SoC RISC-V co-processor (Zeratul fast boot), as the
 * MIPS side sees it
 *
 * On the T31 Ingenic's fast-boot SDK (Zeratul) runs auto-exposure and
 * white balance on a small RISC-V core while U-Boot loads Linux: the SPL
 * ungates the core's clock (CPM CLKGR0 bit 26), holds the core in reset
 * (0x1008 = 1), enables it (0x1004 = 1), copies its code into the SRAM
 * and releases it (0x1008 = 0). The core drives the sensor and the ISP
 * and keeps a result block in DDR that the ISP driver reads when it
 * takes over: it waits for six frames, sets bit 0 of the mailbox (0x1010)
 * to stop the core, waits for the core to clear it, and adopts the
 * results.
 *
 * With "chardev" set the core runs, in a qemu-system-riscv32 linked over
 * that socket (hw/riscv/ingenic-riscv-core.c, the protocol in
 * hw/misc/ingenic-riscv-link.h). This device serves it the SoC's bus and
 * keeps the two blocks both sides reach, as the core's firmware uses them:
 *
 * - 0x0000: a timer counting at 696 MHz (the firmware's delays count
 *   696000 a millisecond). A write to 0x08 restarts the count (0x20). In
 *   mode (0x34) 1 the count wraps each period (0x14), setting 0x0c,
 *   which the core clears by writing 0.
 *
 * - 0x1000: the enable (0x04: the core starts only if bit 0 is set when
 *   reset is released), reset (0x08, held from power-on), where the core
 *   boots (0x0c: its SRAM from power-on, though DDR serves as well), the
 *   mailbox (0x10), whose write by the MIPS side makes an interrupt
 *   pending (0x20 bit 1) until either side writes 1 to 0x14, and the
 *   interrupt enables (0x24). Enable bit 1 lets the mailbox raise the
 *   core's interrupt id 2, bit 31 the timer its id 31.
 *
 * Gating the clock stops the core, and both blocks then read 0. The
 * firmware parks the core for good in wfi with its interrupts off, which
 * Linux's later mailbox writes do not wake it from.
 *
 * The T23's core ("model" t23) has one block, at 0x12220000, and runs
 * from DDR: the SPL sets bit 31 of 0xf0 to hold it, writes where it boots
 * to 0x04 (its vectors, reset at +0x80), copies it there and clears the
 * bit. Linux sets bit 0 of the mailbox (0x08), which is the core's software
 * interrupt until the core clears it, and waits for the core's 1 in 0x0c.
 * 0x10 and 0x14 count at 700 MHz, 64 bits (the firmware's delays count
 * 700000 a millisecond). Its clock is not gated.
 *
 * The T32's and T33's core ("model" t32), Victory0 as the T23's is, has
 * its block at 0x12220000 and 64 KiB of SRAM below it, and runs from DDR:
 * the SPL sets bit 31 of 0xf0 (0xf0 to 0xfc read alike) to hold it, sets
 * up the block, writes where its vectors are to 0x04 (reset at +0x80),
 * copies it and clears the bit. A nonzero 0x08 (the host's word) is the
 * core's interrupt 16 and bit 0 of 0x30 its software interrupt, each
 * until the core clears it; a nonzero 0x0c (the core's) is the host's
 * interrupt (INTC source 4). 0x10 and 0x14 count at the core's clock (the
 * CPM's RISCVHCDR), 64 bits, and reaching 0x18 and 0x1c raises its timer
 * interrupt. 0x28 and 0x2c read the SoC's interrupt lines as its INTC
 * numbers them (the ISP's 29, the VIC's 30, the block's own 4), which
 * 0x20 and 0x24 let through to its external interrupt. The CPM's CLKGR0
 * bit 22 gates the core and the block, which then reads 0.
 *
 * The T41's core ("model" t41, the T40's "t40" and the A1's "a1" the same)
 * has its block at 0x12a00000 and runs from DDR at the reset address in
 * 0x04 (the code 0x80 below). It runs while bit 1 of 0x00 (the bus mask) is
 * set and the CCU's CFCR bit 31 is clear, a T41's from reset, a T40's and
 * an A1's once their SPL or driver clears the bit CFCR resets with: they
 * set it, set up the block, copy the core and clear it. 0x00 keeps bits 0,
 * 1 and 5 (0x23 from reset) and reads bit 2 (bus idle) always, bit 4 while
 * the core is stopped and bit 5 while it runs, as on a T40XP, a T41NQ and
 * an A1N. The block repeats every 0x100, its PMA words (0x40 and 0x60)
 * again 0x10 further on, and 0x80 to 0xff read 0x1e01. The host's mailbox
 * (0x08) is the core's software interrupt until the core clears it; the
 * core's (0x0c) is the host's interrupt until the host does. 0x10 and 0x14
 * count at 500 MHz, 64 bits, and the count reaching 0x18 and 0x1c raises
 * its timer interrupt.
 *
 * Without it the core's code is not run. Released, the model counts
 * frames at the sensor's rate into a result block that reports an
 * exposure the ISP core itself starts from, and answers the stop at once.
 * The block's layout is the T31 ISP core's (libt31-firmware,
 * ae_para_init).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-clock.h"
#include "hw/misc/ingenic-riscv.h"
#include "hw/misc/ingenic-riscv-link.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/ramblock.h"
#include "exec/cpu-common.h"
#include "exec/memop.h"

#define RV_TIMER_CTRL   0x0008
#define RV_TIMER_FLAG   0x000c
#define RV_TIMER_PERIOD 0x0014
#define RV_TIMER_COUNT  0x0020
#define RV_TIMER_MODE   0x0034
#define RV_ENABLE       0x1004
#define RV_RESET        0x1008  /* 1 holds the core, 0 runs it */
#define RV_BOOT         0x100c
#define RV_MBOX         0x1010  /* bit 0: Linux asks the core to stop */
#define RV_MBOX_CLR     0x1014
#define RV_PENDING      0x1020
#define RV_IRQ_EN       0x1024

#define RV_TIMER_HZ     INGENIC_RISCV_T31_COUNT_HZ

#define RV23_BOOT       0x0004
#define RV23_MBOX       0x0008
#define RV23_COUNT_LO   0x0010
#define RV23_COUNT_HI   0x0014
#define RV23_HOLD       0x00f0
#define RV23_COUNT_HZ   700000000
#define RV23_IRQ_MBOX   3       /* machine software interrupt */

#define RV41_CTRL       0x0000  /* bit 1 lets the core run */
#define RV41_CTRL_RW    (BIT(0) | BIT(1) | BIT(5))
#define RV41_CTRL_IDLE  BIT(2)
#define RV41_CTRL_STOP  BIT(4)
#define RV41_CTRL_RUN   BIT(5)
#define RV41_PMA_ADR    0x0040
#define RV41_PMA_CFG    0x0060
#define RV41_PMA_REST   0x1e01  /* what 0x80 to 0xff read */
#define RV41_TO_HOST    0x000c
#define RV41_CMP_LO     0x0018
#define RV41_CMP_HI     0x001c
#define RV41_COUNT_HZ   500000000
#define RV41_IRQ_TIMER  7       /* machine timer interrupt */

#define RV32_CCSR       0x0000
#define RV32_CCSR_IDLE  BIT(2)
#define RV32_CCSR_RUN   BIT(4)
#define RV32_MASK_LO    0x0020
#define RV32_MASK_HI    0x0024
#define RV32_PEND_LO    0x0028
#define RV32_PEND_HI    0x002c
#define RV32_SIP        0x0030
#define RV32_IRQ_SOFT   3
#define RV32_IRQ_TIMER  7
#define RV32_IRQ_EXT    11
#define RV32_IRQ_HOST   16      /* the host's word in 0x08 */
#define RV32_HOST_LINE  4       /* the block's to the host, at the INTC */

#define RV_IRQ_MBOX     2
#define RV_IRQ_TIMER    31

#define RV_MAGIC        0x55aaaa55

/* Result block, in 32-bit words */
#define RES_MAGIC       (0x00 / 4)
#define RES_AE_STAT     (0x04 / 4)
#define RES_AE_RESULT   (0x18 / 4)
#define RES_AE_WEIGHTS  (0x30 / 4)
#define RES_LUMA        (0x70 / 4)
#define RES_EV          (0x74 / 4)
#define RES_FRAMES      (0x78 / 4)
#define RES_AWB         (0x7c / 4)
#define RES_CT          (0x94 / 4)
#define RES_CT_LAST     (0x98 / 4)
#define RES_NIGHT       (0x9c / 4)
#define RES_STABLE      (0xa8 / 4)
#define RES_WORDS       (0xac / 4)

#define REG(s, offset)  ((s)->regs[(offset) / 4])

static bool riscv_linked(IngenicRiscvState *s)
{
    return qemu_chr_fe_backend_connected(&s->link);
}

static void riscv_publish(IngenicRiscvState *s)
{
    /*
     * The values the ISP core initialises its own AE and AWB to: an
     * integration time of 983 lines at 1x again and 1x ISP gain, flat
     * zone weights, unity white-balance gains at 5000 K.
     */
    static const uint32_t ae_stat[] = { 37, 0, 37, 0, 0 };
    static const uint32_t ae_result[] = { 983, 1024, 1024, 500, 1024, 1024 };
    static const uint32_t awb[] = { 0, 0, 256, 256, 256, 256 };
    uint32_t res[RES_WORDS] = { 0 };

    res[RES_MAGIC] = RV_MAGIC;
    memcpy(&res[RES_AE_STAT], ae_stat, sizeof(ae_stat));
    memcpy(&res[RES_AE_RESULT], ae_result, sizeof(ae_result));
    for (int i = 0; i < 15; i++) {
        res[RES_AE_WEIGHTS + i] = 37;
    }
    res[RES_LUMA] = 128;
    res[RES_EV] = 983;
    res[RES_FRAMES] = s->frames;
    memcpy(&res[RES_AWB], awb, sizeof(awb));
    res[RES_CT] = 5000;
    res[RES_CT_LAST] = 5000;
    res[RES_NIGHT] = s->night;
    res[RES_STABLE] = 1;
    for (int i = 0; i < RES_WORDS; i++) {
        res[i] = cpu_to_le32(res[i]);
    }
    address_space_write(&address_space_memory, s->result_addr,
                        MEMTXATTRS_UNSPECIFIED, res, sizeof(res));
}

static void riscv_frame(void *opaque)
{
    IngenicRiscvState *s = opaque;

    s->frames++;
    riscv_publish(s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / s->frame_rate);
}

static void link_send(IngenicRiscvState *s, uint32_t type, uint32_t addr,
                      uint32_t size, uint32_t data, int fd)
{
    IngenicRiscvMsg m = {
        .type = cpu_to_le32(type),
        .addr = cpu_to_le32(addr),
        .size = cpu_to_le32(size),
        .data = cpu_to_le32(data),
    };

    if (!s->hello) {
        return;
    }
    if (fd >= 0 && qemu_chr_fe_set_msgfds(&s->link, &fd, 1) < 0) {
        error_report("ingenic-riscv: the core's link cannot pass memory; "
                     "it must be a UNIX socket");
        return;
    }
    qemu_chr_fe_write_all(&s->link, (const uint8_t *)&m, sizeof(m));
}

static uint64_t riscv_count_hz(IngenicRiscvState *s)
{
    if (s->t32) {
        return clock_get_hz(s->count_clk) ?: 1;
    }
    return s->t41 ? RV41_COUNT_HZ : RV23_COUNT_HZ;
}

/* The T23's, T32's and T41's 64-bit count */
static uint64_t riscv_count64(IngenicRiscvState *s)
{
    /* A T40XP's and an A1N's count stays 0 while CFCR holds the core */
    if (s->t41 && s->hold) {
        return s->count_held;
    }
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->count_start,
                    riscv_count_hz(s), NANOSECONDS_PER_SECOND);
}

/* Restart the count from where it stands, at the clock's new rate */
static void riscv_rebase_count(IngenicRiscvState *s, uint64_t count)
{
    s->count_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                     muldiv64(count, NANOSECONDS_PER_SECOND,
                              riscv_count_hz(s));
}

static uint64_t riscv_cmp(IngenicRiscvState *s)
{
    return (uint64_t)REG(s, RV41_CMP_HI) << 32 | REG(s, RV41_CMP_LO);
}

static void riscv_update_irqs(IngenicRiscvState *s)
{
    uint32_t en = REG(s, RV_IRQ_EN);
    uint32_t level = 0;
    uint32_t changed;

    if (s->t41) {
        qemu_set_irq(s->host_irq, REG(s, RV41_TO_HOST) != 0);
        level = REG(s, RV23_MBOX) ? BIT(RV23_IRQ_MBOX) : 0;
        if (riscv_count64(s) >= riscv_cmp(s)) {
            level |= BIT(RV41_IRQ_TIMER);
        }
    } else if (s->t32) {
        uint64_t mask = (uint64_t)REG(s, RV32_MASK_HI) << 32 |
                        REG(s, RV32_MASK_LO);
        bool to_host = REG(s, RV41_TO_HOST) != 0;

        qemu_set_irq(s->host_irq, to_host);
        s->soc_irqs = deposit64(s->soc_irqs, RV32_HOST_LINE, 1, to_host);
        level = REG(s, RV23_MBOX) ? BIT(RV32_IRQ_HOST) : 0;
        if (REG(s, RV32_SIP) & 1) {
            level |= BIT(RV32_IRQ_SOFT);
        }
        if (riscv_count64(s) >= riscv_cmp(s)) {
            level |= BIT(RV32_IRQ_TIMER);
        }
        if (s->soc_irqs & mask) {
            level |= BIT(RV32_IRQ_EXT);
        }
    } else if (s->t23) {
        level = REG(s, RV23_MBOX) & 1 ? BIT(RV23_IRQ_MBOX) : 0;
    } else {
        if (s->mbox_pending && (en & BIT(1))) {
            level |= BIT(RV_IRQ_MBOX);
        }
        if ((REG(s, RV_TIMER_FLAG) & 1) && (en & BIT(31))) {
            level |= BIT(RV_IRQ_TIMER);
        }
    }
    changed = level ^ s->irq_level;
    s->irq_level = level;
    for (int i = 0; changed; i++, changed >>= 1) {
        if (changed & 1) {
            link_send(s, IRL_IRQ, i, 0, extract32(level, i, 1), -1);
        }
    }
}

static void riscv_clock(void *opaque, int n, int level)
{
    IngenicRiscvState *s = opaque;

    if (s->clock != !!level) {
        s->clock = level;
        link_send(s, IRL_CLOCK, 0, 0, s->clock, -1);
    }
}

/* Where the core boots: its vectors, and reset at +0x80 */
static uint32_t riscv_boot(IngenicRiscvState *s)
{
    if (s->t41) {
        return REG(s, RV23_BOOT) - 0x80;
    }
    return s->t23 || s->t32 ? REG(s, RV23_BOOT) : REG(s, RV_BOOT);
}

static void riscv_run(IngenicRiscvState *s, bool run)
{
    if (run == s->running) {
        return;
    }
    s->running = run;
    if (riscv_linked(s)) {
        link_send(s, IRL_RUN, riscv_boot(s), 0, run, -1);
    } else if (s->t23 || s->t32 || s->t41) {
        return;
    } else if (run) {
        s->frames = 0;
        riscv_frame(s);
    } else {
        timer_del(s->frame_timer);
    }
}

static void riscv41_run(IngenicRiscvState *s)
{
    riscv_run(s, (REG(s, RV41_CTRL) & BIT(1)) && !s->hold);
}

/* The T41's block repeats every 0x100, and its PMA words 0x10 further on */
static hwaddr riscv41_reg(hwaddr offset)
{
    offset &= 0xff;
    if ((offset >= 0x50 && offset < 0x60) ||
        (offset >= 0x70 && offset < 0x80)) {
        offset -= 0x10;
    }
    return offset;
}

/* A write to half of the count moves the count there */
static void riscv41_set_count(IngenicRiscvState *s, hwaddr offset,
                              uint32_t value)
{
    uint64_t count = riscv_count64(s);

    count = offset == RV23_COUNT_LO ? deposit64(count, 0, 32, value) :
                                      deposit64(count, 32, 32, value);
    if (s->t41 && s->hold) {
        s->count_held = count;
    } else {
        riscv_rebase_count(s, count);
    }
}

static void riscv41_cmp_arm(IngenicRiscvState *s)
{
    uint64_t cmp = riscv_cmp(s);

    if ((s->t41 && s->hold) || cmp == UINT64_MAX || cmp <= riscv_count64(s)) {
        timer_del(s->cmp_timer);
        return;
    }
    timer_mod(s->cmp_timer, s->count_start +
              muldiv64(cmp, NANOSECONDS_PER_SECOND, riscv_count_hz(s)) + 1);
}

static void riscv_hold(void *opaque, int n, int level)
{
    IngenicRiscvState *s = opaque;

    if (s->t41 && level && !s->hold) {
        s->count_held = riscv_count64(s);
    } else if (s->t41 && !level && s->hold) {
        riscv_rebase_count(s, s->count_held);
    }
    s->hold = level;
    if (s->t41) {
        riscv41_cmp_arm(s);
        riscv41_run(s);
    }
}

static void riscv41_cmp_fire(void *opaque)
{
    riscv_update_irqs(opaque);
}

static uint64_t riscv_count(IngenicRiscvState *s)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->count_start;
    uint64_t count = muldiv64(ns, RV_TIMER_HZ, NANOSECONDS_PER_SECOND);
    uint32_t period = REG(s, RV_TIMER_PERIOD);

    return REG(s, RV_TIMER_MODE) == 1 && period ? count % period : count;
}

/* The count as it now stands, for a linked core to count on from */
static void riscv_send_count(IngenicRiscvState *s)
{
    uint32_t period = REG(s, RV_TIMER_MODE) == 1 ? REG(s, RV_TIMER_PERIOD) : 0;

    link_send(s, IRL_COUNT, period, 0, riscv_count(s), -1);
}

static void riscv_tick_arm(IngenicRiscvState *s)
{
    uint32_t period = REG(s, RV_TIMER_PERIOD);
    int64_t ns, periods;

    if (REG(s, RV_TIMER_MODE) != 1 || !period) {
        timer_del(s->tick);
        return;
    }
    ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->count_start;
    periods = muldiv64(ns, RV_TIMER_HZ, NANOSECONDS_PER_SECOND) / period + 1;
    timer_mod(s->tick, s->count_start +
              muldiv64(periods * period, NANOSECONDS_PER_SECOND,
                       RV_TIMER_HZ) + 1);
}

static void riscv_tick(void *opaque)
{
    IngenicRiscvState *s = opaque;

    REG(s, RV_TIMER_FLAG) |= 1;
    riscv_update_irqs(s);
    riscv_tick_arm(s);
}

static MemTxResult riscv_ctrl_read(void *opaque, hwaddr offset,
                                   uint64_t *data, unsigned size,
                                   MemTxAttrs attrs)
{
    IngenicRiscvState *s = opaque;

    uint64_t count;

    if (s->t41) {
        offset = riscv41_reg(offset);
    }
    if (!s->t23 && !s->t41 && !s->clock) {
        *data = 0;
    } else if (s->t41 && offset >= 0x80) {
        *data = RV41_PMA_REST;
    } else if ((s->t23 || s->t32 || s->t41) &&
               (offset == RV23_COUNT_LO || offset == RV23_COUNT_HI)) {
        count = riscv_count64(s);
        *data = offset == RV23_COUNT_LO ? (uint32_t)count : count >> 32;
    } else if (s->t41 && offset == RV41_CTRL) {
        *data = REG(s, RV41_CTRL) | RV41_CTRL_IDLE |
                (s->running ? RV41_CTRL_RUN : RV41_CTRL_STOP);
    } else if (s->t32 && offset == RV32_CCSR) {
        *data = REG(s, RV32_CCSR) |
                (s->running ? RV32_CCSR_RUN : RV32_CCSR_IDLE);
    } else if (s->t32 && (offset == RV32_PEND_LO || offset == RV32_PEND_HI)) {
        *data = offset == RV32_PEND_LO ? (uint32_t)s->soc_irqs
                                       : s->soc_irqs >> 32;
    } else if (s->t32 && offset >= RV23_HOLD && offset < 0x100) {
        *data = REG(s, RV23_HOLD);
    } else if (!s->t23 && !s->t32 && !s->t41) {
        if (offset == RV_TIMER_COUNT) {
            *data = (uint32_t)riscv_count(s);
        } else if (offset == RV_PENDING) {
            *data = s->mbox_pending ? BIT(1) : 0;
        } else {
            *data = s->regs[offset / 4];
        }
    } else {
        *data = s->regs[offset / 4];
    }
    return MEMTX_OK;
}

static MemTxResult riscv_ctrl_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size,
                                    MemTxAttrs attrs)
{
    IngenicRiscvState *s = opaque;
    bool core = !attrs.unspecified &&
                attrs.requester_id == INGENIC_RISCV_REQUESTER;

    if (s->t41) {
        offset = riscv41_reg(offset);
        if (offset >= 0x80) {
            return MEMTX_OK;
        }
    }
    if (!s->t23 && !s->t41 && !s->clock) {
        return MEMTX_OK;
    }
    s->regs[offset / 4] = value;
    if (s->t41) {
        if (offset == RV41_CTRL) {
            REG(s, RV41_CTRL) &= RV41_CTRL_RW;
            riscv41_run(s);
        } else if (offset == RV23_COUNT_LO || offset == RV23_COUNT_HI) {
            riscv41_set_count(s, offset, value);
        }
        if (offset != RV41_CTRL) {
            riscv41_cmp_arm(s);
        }
        riscv_update_irqs(s);
        return MEMTX_OK;
    }
    if (s->t32) {
        if (offset >= RV23_HOLD && offset < 0x100) {
            REG(s, RV23_HOLD) = value;
            riscv_run(s, !(value & BIT(31)));
        } else if (offset == RV32_CCSR) {
            REG(s, RV32_CCSR) &= ~(RV32_CCSR_IDLE | BIT(3) | RV32_CCSR_RUN);
        } else if (offset == RV23_COUNT_LO || offset == RV23_COUNT_HI) {
            riscv41_set_count(s, offset, value);
        }
        riscv41_cmp_arm(s);
        riscv_update_irqs(s);
        return MEMTX_OK;
    }
    if (s->t23) {
        if (offset == RV23_HOLD) {
            riscv_run(s, !(value & BIT(31)));
        }
        riscv_update_irqs(s);
        return MEMTX_OK;
    }
    switch (offset) {
    case RV_TIMER_CTRL:
        s->count_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        riscv_tick_arm(s);
        riscv_send_count(s);
        break;
    case RV_TIMER_PERIOD:
    case RV_TIMER_MODE:
        riscv_tick_arm(s);
        riscv_send_count(s);
        break;
    case RV_RESET:
        if (value || REG(s, RV_ENABLE) & 1) {
            riscv_run(s, !value);
        }
        break;
    case RV_MBOX:
        if (riscv_linked(s)) {
            if (!core && value) {
                s->mbox_pending = true;
            }
        } else if ((value & 1) && s->running) {
            /* The core finishes its frame and acknowledges at once */
            riscv_run(s, false);
            riscv_publish(s);
            REG(s, RV_MBOX) &= ~1u;
        }
        break;
    case RV_MBOX_CLR:
        if (value & 1) {
            s->mbox_pending = false;
        }
        break;
    }
    riscv_update_irqs(s);
    return MEMTX_OK;
}

static const MemoryRegionOps riscv_ctrl_ops = {
    .read_with_attrs = riscv_ctrl_read,
    .write_with_attrs = riscv_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static bool link_send_mem(IngenicRiscvState *s, MemoryRegion *mr,
                          hwaddr addr)
{
    int fd = memory_region_get_fd(mr);

    if (fd < 0 || qemu_ram_get_fd_offset(mr->ram_block)) {
        return false;
    }
    link_send(s, IRL_MEM, addr, memory_region_size(mr), 0, fd);
    return true;
}

static void link_hello(IngenicRiscvState *s, uint32_t version)
{
    MemoryRegionSection ddr;
    bool shared;

    if (version != INGENIC_RISCV_LINK_VERSION) {
        error_report("ingenic-riscv: the core speaks link version %u, "
                     "not %u", version, INGENIC_RISCV_LINK_VERSION);
        qemu_chr_fe_disconnect(&s->link);
        return;
    }
    s->hello = true;

    ddr = memory_region_find(get_system_memory(), 0, 1);
    shared = ddr.mr && link_send_mem(s, ddr.mr, 0);
    if (ddr.mr) {
        memory_region_unref(ddr.mr);
    }
    if (!shared || (memory_region_is_mapped(&s->sram) &&
                    !link_send_mem(s, &s->sram, s->sram.addr))) {
        error_report("ingenic-riscv: the core needs DDR it can share: "
                     "-object memory-backend-memfd,id=ddr,size=<RAM>,"
                     "share=on -machine memory-backend=ddr");
        s->hello = false;
        qemu_chr_fe_disconnect(&s->link);
        return;
    }
    for (int i = 0; i < 32; i++) {
        if (s->irq_level & BIT(i)) {
            link_send(s, IRL_IRQ, i, 0, 1, -1);
        }
    }
    link_send(s, IRL_CLOCK, 0, 0, s->clock, -1);
    if (!s->t23 && !s->t41) {
        riscv_send_count(s);
    }
    link_send(s, IRL_RUN, riscv_boot(s), 0, s->running, -1);
}

/* An access by the core, made on the SoC's bus */
static void link_access(IngenicRiscvState *s, const IngenicRiscvMsg *m)
{
    MemTxAttrs attrs = { .requester_id = INGENIC_RISCV_REQUESTER };
    uint32_t addr = le32_to_cpu(m->addr);
    uint32_t size = le32_to_cpu(m->size);
    uint32_t data = 0;
    uint8_t buf[4];

    if (size != 1 && size != 2 && size != 4) {
        error_report("ingenic-riscv: the core sent a %u-byte access", size);
    } else if (le32_to_cpu(m->type) == IRL_READ) {
        address_space_read(&address_space_memory, addr, attrs, buf, size);
        data = ldm_p(buf, size_memop(size) | MO_LE);
    } else {
        stm_p(buf, size_memop(size) | MO_LE, le32_to_cpu(m->data));
        address_space_write(&address_space_memory, addr, attrs, buf, size);
    }
    link_send(s, IRL_DATA, 0, 0, data, -1);
}

static int link_can_read(void *opaque)
{
    IngenicRiscvState *s = opaque;

    return sizeof(s->rx) - s->rx_len;
}

static void link_read(void *opaque, const uint8_t *buf, int size)
{
    IngenicRiscvState *s = opaque;

    memcpy((uint8_t *)&s->rx + s->rx_len, buf, size);
    s->rx_len += size;
    if (s->rx_len < sizeof(s->rx)) {
        return;
    }
    s->rx_len = 0;
    switch (le32_to_cpu(s->rx.type)) {
    case IRL_HELLO:
        link_hello(s, le32_to_cpu(s->rx.data));
        break;
    case IRL_READ:
    case IRL_WRITE:
        if (s->hello) {
            link_access(s, &s->rx);
        }
        break;
    default:
        error_report("ingenic-riscv: unexpected message %u from the core",
                     le32_to_cpu(s->rx.type));
        break;
    }
}

static void link_event(void *opaque, QEMUChrEvent event)
{
    IngenicRiscvState *s = opaque;

    if (event == CHR_EVENT_CLOSED) {
        s->hello = false;
        s->rx_len = 0;
    }
}

static void riscv_reset_hold(Object *obj, ResetType type)
{
    IngenicRiscvState *s = INGENIC_RISCV(obj);

    timer_del(s->frame_timer);
    timer_del(s->tick);
    memset(s->regs, 0, sizeof(s->regs));
    s->frames = 0;
    s->mbox_pending = false;
    s->count_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->count_held = 0;
    if (s->t23) {
        REG(s, RV23_HOLD) = BIT(31);
    } else if (s->t32) {
        /* As read on a T32LQ; 0x3c is not known */
        REG(s, RV32_CCSR) = 0x23;
        REG(s, RV23_HOLD) = BIT(31);
        REG(s, RV41_CMP_LO) = UINT32_MAX;
        REG(s, RV41_CMP_HI) = UINT32_MAX;
        REG(s, 0x003c) = 1;
        REG(s, 0x0058) = 0x05ffffff;        /* PMA_ADR6 and 7 */
        REG(s, 0x005c) = 0x0fffffff;
        for (int i = 0x60; i < 0x78; i += 4) {
            REG(s, i) = 7;                  /* PMA_CFG0 to 5 */
        }
        REG(s, 0x0078) = 0x1f;
        REG(s, 0x007c) = 0x3f;
    } else if (s->t41) {
        static const uint32_t adr[] = { 0x04000000, 0x05ffffff, 0x1fffffff };
        static const uint32_t cfg[] = { 0x1f01, 0x2e03, 0x1f01, 0x1e01 };

        /* As a T40XP, a T41NQ and an A1N read it */
        REG(s, RV41_CTRL) = RV41_CTRL_RW;
        REG(s, RV41_CMP_LO) = UINT32_MAX;
        REG(s, RV41_CMP_HI) = UINT32_MAX;
        for (unsigned i = 0; i < ARRAY_SIZE(cfg); i++) {
            REG(s, RV41_PMA_ADR + i * 4) = i < ARRAY_SIZE(adr) ? adr[i] : 0;
            REG(s, RV41_PMA_CFG + i * 4) = cfg[i];
        }
    } else {
        /* As read on a T31; 0x0010 and 0x10f8 are not known */
        REG(s, RV_RESET) = 1;
        REG(s, RV_BOOT) = INGENIC_RISCV_SRAM_BASE;
        REG(s, RV_TIMER_PERIOD) = UINT32_MAX;
        REG(s, 0x0010) = 1;
        REG(s, 0x10f8) = 1;
        riscv_send_count(s);
    }
    timer_del(s->cmp_timer);
    riscv_clock(s, 0, s->t23 || s->t32 || s->t41);
    riscv_run(s, false);
    riscv_update_irqs(s);
}

/* The T41's core runs from reset unless the CCU's CFCR holds it */
static void riscv_reset_exit(Object *obj, ResetType type)
{
    IngenicRiscvState *s = INGENIC_RISCV(obj);

    if (s->t41) {
        riscv41_run(s);
    }
}

static void riscv_realize(DeviceState *dev, Error **errp)
{
    IngenicRiscvState *s = INGENIC_RISCV(dev);

    if (!s->frame_rate) {
        error_setg(errp, "frame-rate must be at least 1");
        return;
    }
    if (s->model && strcmp(s->model, "t31") && strcmp(s->model, "t23") &&
        strcmp(s->model, "t32") && strcmp(s->model, "t40") &&
        strcmp(s->model, "t41") && strcmp(s->model, "a1")) {
        error_setg(errp, "model must be t31, t23, t32, t40, t41 or a1");
        return;
    }
    s->t23 = s->model && !strcmp(s->model, "t23");
    s->t32 = s->model && !strcmp(s->model, "t32");
    s->t41 = s->model && (!strcmp(s->model, "t40") ||
                          !strcmp(s->model, "t41") ||
                          !strcmp(s->model, "a1"));
    /* Linked, the SRAM is shared with the core's QEMU */
    if (!memory_region_init_ram_flags_nomigrate(&s->sram, OBJECT(dev),
            "ingenic-riscv.sram", INGENIC_RISCV_SRAM_SIZE,
            riscv_linked(s) ? RAM_SHARED : 0, errp)) {
        return;
    }
    vmstate_register_ram(&s->sram, dev);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->sram);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ctrl);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, riscv_frame, s);
    s->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, riscv_tick, s);
    s->cmp_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, riscv41_cmp_fire, s);
    if (riscv_linked(s)) {
        qemu_chr_fe_set_handlers(&s->link, link_can_read, link_read,
                                 link_event, NULL, s, NULL, true);
    }
}

/* The T32's count keeps its place across a change of the core's clock */
static void riscv_count_clk_update(void *opaque, ClockEvent event)
{
    IngenicRiscvState *s = opaque;

    if (event == ClockPreUpdate) {
        s->count_at_hz = riscv_count64(s);
    } else {
        riscv_rebase_count(s, s->count_at_hz);
        riscv41_cmp_arm(s);
    }
}

static void riscv_soc_irq(void *opaque, int n, int level)
{
    IngenicRiscvState *s = opaque;

    s->soc_irqs = deposit64(s->soc_irqs, n, 1, level != 0);
    riscv_update_irqs(s);
}

static void riscv_init(Object *obj)
{
    IngenicRiscvState *s = INGENIC_RISCV(obj);

    memory_region_init_io(&s->ctrl, obj, &riscv_ctrl_ops, s,
                          "ingenic-riscv.ctrl", INGENIC_RISCV_CTRL_SIZE);
    qdev_init_gpio_in_named(DEVICE(obj), riscv_clock, "clock", 1);
    qdev_init_gpio_in_named(DEVICE(obj), riscv_hold, "hold", 1);
    qdev_init_gpio_in_named(DEVICE(obj), riscv_soc_irq, "soc-irq", 64);
    s->count_clk = qdev_init_clock_in(DEVICE(obj), "count-clk",
                                      riscv_count_clk_update, s,
                                      ClockPreUpdate | ClockUpdate);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->host_irq);
}

static const Property riscv_props[] = {
    DEFINE_PROP_UINT32("result-addr", IngenicRiscvState, result_addr,
                       0x03700000),
    DEFINE_PROP_UINT32("frame-rate", IngenicRiscvState, frame_rate, 60),
    DEFINE_PROP_BOOL("night", IngenicRiscvState, night, false),
    DEFINE_PROP_CHR("chardev", IngenicRiscvState, link),
    DEFINE_PROP_STRING("model", IngenicRiscvState, model),
};

static const VMStateDescription vmstate_riscv = {
    .name = TYPE_INGENIC_RISCV,
    .version_id = 5,
    .minimum_version_id = 5,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicRiscvState,
                             INGENIC_RISCV_CTRL_SIZE / 4),
        VMSTATE_BOOL(running, IngenicRiscvState),
        VMSTATE_UINT32(frames, IngenicRiscvState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicRiscvState),
        VMSTATE_TIMER_PTR(tick, IngenicRiscvState),
        VMSTATE_INT64(count_start, IngenicRiscvState),
        VMSTATE_UINT64(count_held, IngenicRiscvState),
        VMSTATE_BOOL(mbox_pending, IngenicRiscvState),
        VMSTATE_UINT32(irq_level, IngenicRiscvState),
        VMSTATE_BOOL(clock, IngenicRiscvState),
        VMSTATE_BOOL(hold, IngenicRiscvState),
        VMSTATE_TIMER_PTR(cmp_timer, IngenicRiscvState),
        VMSTATE_UINT64(soc_irqs, IngenicRiscvState),
        VMSTATE_CLOCK(count_clk, IngenicRiscvState),
        VMSTATE_END_OF_LIST()
    },
};

static void riscv_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = riscv_realize;
    dc->vmsd = &vmstate_riscv;
    device_class_set_props(dc, riscv_props);
    rc->phases.hold = riscv_reset_hold;
    rc->phases.exit = riscv_reset_exit;
}

static const TypeInfo riscv_type_info = {
    .name = TYPE_INGENIC_RISCV,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicRiscvState),
    .instance_init = riscv_init,
    .class_init = riscv_class_init,
};

static void riscv_register_types(void)
{
    type_register_static(&riscv_type_info);
}

type_init(riscv_register_types)
