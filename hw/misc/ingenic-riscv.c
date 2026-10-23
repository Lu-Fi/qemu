/*
 * Ingenic camera SoC RISC-V co-processor (Zeratul fast boot), as the
 * MIPS side sees it
 *
 * On the T31 Ingenic's fast-boot SDK (Zeratul) runs auto-exposure and
 * white balance on a small RISC-V core while U-Boot loads Linux: the SPL
 * holds the core in reset (0x1008 = 1), copies its code into the SRAM,
 * and releases it (0x1008 = 0). The core drives the sensor and the ISP
 * and keeps a result block in DDR that the ISP driver reads when it
 * takes over: it waits for six frames, sets bit 0 of 0x1010 to stop the
 * core, waits for the core to clear it, and adopts the results.
 *
 * The core's code is not run. Released, the model counts frames at the
 * sensor's rate into a result block that reports an exposure the ISP
 * core itself starts from, and answers the stop at once. The block's
 * layout is the T31 ISP core's (libt31-firmware, ae_para_init).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/misc/ingenic-riscv.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "exec/cpu-common.h"

#define RV_RESET        0x1008  /* 1 holds the core, 0 runs it */
#define RV_STOP         0x1010  /* bit 0: Linux asks the core to stop */

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

static void riscv_run(IngenicRiscvState *s, bool run)
{
    if (run == s->running) {
        return;
    }
    s->running = run;
    if (run) {
        s->frames = 0;
        riscv_frame(s);
    } else {
        timer_del(s->frame_timer);
    }
}

static uint64_t riscv_ctrl_read(void *opaque, hwaddr offset, unsigned size)
{
    IngenicRiscvState *s = opaque;

    return s->regs[offset / 4];
}

static void riscv_ctrl_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    IngenicRiscvState *s = opaque;

    s->regs[offset / 4] = value;
    switch (offset) {
    case RV_RESET:
        riscv_run(s, !value);
        break;
    case RV_STOP:
        if ((value & 1) && s->running) {
            /* The core finishes its frame and acknowledges at once */
            riscv_run(s, false);
            riscv_publish(s);
            s->regs[offset / 4] &= ~1u;
        }
        break;
    }
}

static const MemoryRegionOps riscv_ctrl_ops = {
    .read = riscv_ctrl_read,
    .write = riscv_ctrl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void riscv_reset_hold(Object *obj, ResetType type)
{
    IngenicRiscvState *s = INGENIC_RISCV(obj);

    timer_del(s->frame_timer);
    memset(s->regs, 0, sizeof(s->regs));
    s->running = false;
    s->frames = 0;
}

static void riscv_realize(DeviceState *dev, Error **errp)
{
    IngenicRiscvState *s = INGENIC_RISCV(dev);

    if (!s->frame_rate) {
        error_setg(errp, "frame-rate must be at least 1");
        return;
    }
    if (!memory_region_init_ram(&s->sram, OBJECT(dev), "ingenic-riscv.sram",
                                INGENIC_RISCV_SRAM_SIZE, errp)) {
        return;
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->sram);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->ctrl);
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, riscv_frame, s);
}

static void riscv_init(Object *obj)
{
    IngenicRiscvState *s = INGENIC_RISCV(obj);

    memory_region_init_io(&s->ctrl, obj, &riscv_ctrl_ops, s,
                          "ingenic-riscv.ctrl", INGENIC_RISCV_CTRL_SIZE);
}

static const Property riscv_props[] = {
    DEFINE_PROP_UINT32("result-addr", IngenicRiscvState, result_addr,
                       0x03700000),
    DEFINE_PROP_UINT32("frame-rate", IngenicRiscvState, frame_rate, 60),
    DEFINE_PROP_BOOL("night", IngenicRiscvState, night, false),
};

static const VMStateDescription vmstate_riscv = {
    .name = TYPE_INGENIC_RISCV,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicRiscvState,
                             INGENIC_RISCV_CTRL_SIZE / 4),
        VMSTATE_BOOL(running, IngenicRiscvState),
        VMSTATE_UINT32(frames, IngenicRiscvState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicRiscvState),
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
