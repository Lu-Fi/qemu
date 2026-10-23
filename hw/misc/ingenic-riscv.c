/*
 * Ingenic camera SoC RISC-V co-processor (Zeratul fast boot), as the
 * MIPS side sees it
 *
 * On the T31 Ingenic's fast-boot SDK (Zeratul) runs auto-exposure and
 * white balance on a small RISC-V core while U-Boot loads Linux: the SPL
 * holds the core in reset (0x1008 = 1), copies its code into the SRAM,
 * and releases it (0x1008 = 0). The core drives the sensor and the ISP
 * and keeps a result block in DDR that the ISP driver reads when it
 * takes over: it waits for six frames, sets bit 0 of the mailbox (0x1010)
 * to stop the core, waits for the core to clear it, and adopts the
 * results.
 *
 * With "chardev" set the core runs, in a qemu-system-riscv32 linked over
 * that socket (hw/riscv/ingenic-t31-riscv.c, the protocol in
 * hw/misc/ingenic-riscv-link.h). This device serves it the SoC's bus and
 * keeps the two blocks both sides reach, as the core's firmware uses them:
 *
 * - 0x0000: a timer counting at 696 MHz (the firmware's delays count
 *   696000 a millisecond). A write to 0x08 restarts the count (0x20). In
 *   mode (0x34) 1 the count wraps each period (0x14), setting 0x0c,
 *   which the core clears by writing 0.
 *
 * - 0x1000: reset (0x08), the mailbox (0x10), whose write by the MIPS
 *   side raises an interrupt the core clears by writing 1 to 0x14, and
 *   the interrupt enables (0x24). Enable bit 1 lets the mailbox raise the
 *   core's interrupt id 2, bit 31 the timer its id 31.
 *
 * The core's clock is taken to be the CPM's OPCR bit 31 ("clock"), which
 * Zeratul's SPL sets before it releases the core and the firmware clears
 * as the last thing it does before it parks in wfi for good: Linux writes
 * the mailbox again after that, and a parked core would wake to spin.
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
#define RV_RESET        0x1008  /* 1 holds the core, 0 runs it */
#define RV_MBOX         0x1010  /* bit 0: Linux asks the core to stop */
#define RV_MBOX_CLR     0x1014
#define RV_IRQ_EN       0x1024

#define RV_TIMER_HZ     696000000

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

static void riscv_update_irqs(IngenicRiscvState *s)
{
    uint32_t en = REG(s, RV_IRQ_EN);
    uint32_t level = 0;
    uint32_t changed;

    if (s->mbox_pending && (en & BIT(1))) {
        level |= BIT(RV_IRQ_MBOX);
    }
    if ((REG(s, RV_TIMER_FLAG) & 1) && (en & BIT(31))) {
        level |= BIT(RV_IRQ_TIMER);
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

static void riscv_run(IngenicRiscvState *s, bool run)
{
    if (run == s->running) {
        return;
    }
    s->running = run;
    if (riscv_linked(s)) {
        link_send(s, IRL_RUN, 0, 0, run, -1);
    } else if (run) {
        s->frames = 0;
        riscv_frame(s);
    } else {
        timer_del(s->frame_timer);
    }
}

static uint64_t riscv_count(IngenicRiscvState *s)
{
    int64_t ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->count_start;
    uint64_t count = muldiv64(ns, RV_TIMER_HZ, NANOSECONDS_PER_SECOND);
    uint32_t period = REG(s, RV_TIMER_PERIOD);

    return REG(s, RV_TIMER_MODE) == 1 && period ? count % period : count;
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

    *data = offset == RV_TIMER_COUNT ? (uint32_t)riscv_count(s)
                                     : s->regs[offset / 4];
    return MEMTX_OK;
}

static MemTxResult riscv_ctrl_write(void *opaque, hwaddr offset,
                                    uint64_t value, unsigned size,
                                    MemTxAttrs attrs)
{
    IngenicRiscvState *s = opaque;
    bool core = !attrs.unspecified &&
                attrs.requester_id == INGENIC_RISCV_REQUESTER;

    s->regs[offset / 4] = value;
    switch (offset) {
    case RV_TIMER_CTRL:
        s->count_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        riscv_tick_arm(s);
        break;
    case RV_TIMER_PERIOD:
    case RV_TIMER_MODE:
        riscv_tick_arm(s);
        break;
    case RV_RESET:
        riscv_run(s, !value);
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
    if (!shared || !link_send_mem(s, &s->sram, s->sram.addr)) {
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
    link_send(s, IRL_RUN, 0, 0, s->running, -1);
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
    riscv_clock(s, 0, 0);
    riscv_run(s, false);
    riscv_update_irqs(s);
}

static void riscv_realize(DeviceState *dev, Error **errp)
{
    IngenicRiscvState *s = INGENIC_RISCV(dev);

    if (!s->frame_rate) {
        error_setg(errp, "frame-rate must be at least 1");
        return;
    }
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
    if (riscv_linked(s)) {
        qemu_chr_fe_set_handlers(&s->link, link_can_read, link_read,
                                 link_event, NULL, s, NULL, true);
    }
}

static void riscv_init(Object *obj)
{
    IngenicRiscvState *s = INGENIC_RISCV(obj);

    memory_region_init_io(&s->ctrl, obj, &riscv_ctrl_ops, s,
                          "ingenic-riscv.ctrl", INGENIC_RISCV_CTRL_SIZE);
    qdev_init_gpio_in_named(DEVICE(obj), riscv_clock, "clock", 1);
}

static const Property riscv_props[] = {
    DEFINE_PROP_UINT32("result-addr", IngenicRiscvState, result_addr,
                       0x03700000),
    DEFINE_PROP_UINT32("frame-rate", IngenicRiscvState, frame_rate, 60),
    DEFINE_PROP_BOOL("night", IngenicRiscvState, night, false),
    DEFINE_PROP_CHR("chardev", IngenicRiscvState, link),
};

static const VMStateDescription vmstate_riscv = {
    .name = TYPE_INGENIC_RISCV,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicRiscvState,
                             INGENIC_RISCV_CTRL_SIZE / 4),
        VMSTATE_BOOL(running, IngenicRiscvState),
        VMSTATE_UINT32(frames, IngenicRiscvState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicRiscvState),
        VMSTATE_TIMER_PTR(tick, IngenicRiscvState),
        VMSTATE_INT64(count_start, IngenicRiscvState),
        VMSTATE_BOOL(mbox_pending, IngenicRiscvState),
        VMSTATE_UINT32(irq_level, IngenicRiscvState),
        VMSTATE_BOOL(clock, IngenicRiscvState),
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
