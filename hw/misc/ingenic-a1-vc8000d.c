/*
 * Ingenic A1 VC8000D video decoder
 *
 * A VeriSilicon VC8000D decoder core behind a VCMD command-buffer
 * sequencer, as vdec.ko (VeriSilicon's hantro_dec/hantro_vcmd) drives it:
 * the guest programs a core only through command buffers in DRAM. The
 * kernel starts the sequencer with its CMDBUF address, length and an
 * interrupt mask, libimp's DWL fills the buffers with register writes, a
 * stall on the core's interrupt and reads of its status back to DRAM.
 *
 * The model runs a chain when the kernel starts the sequencer (a write of
 * the control register's start bit), decodes a picture when a command
 * buffer sets the core's enable bit, and completes the way the hardware
 * does: the sequencer stops at the last JMP with status bit 1 and the
 * core's interrupt flags are in the status the buffer read back. A stall
 * on the core's interrupt takes time, and the commands after it are read
 * when it ends: the kernel links the next jobs' buffers onto a running
 * chain and fills them while the sequencer works on the earlier ones.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/misc/ingenic-a1-vc8000d.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/memory.h"
#include "ingenic-a1-vc8000d-codec.h"
#include "trace.h"

/* VCMD registers, in bytes */
#define VCMD_ID         0x00
#define VCMD_CTRL       0x40
#define VCMD_STATUS     0x44
#define VCMD_IRQ_EN     0x48
#define VCMD_EXE_ADDR   0x50
#define VCMD_EXE_LEN    0x58
#define VCMD_EXE_ID     0x68

#define VCMD_CTRL_START BIT(0)
#define VCMD_ST_END     BIT(1)      /* the chain ended at a JMP not ready */

/* command opcodes, word 0 [31:27] */
#define OP_END          0x00
#define OP_WREG         0x01
#define OP_NOP          0x03
#define OP_STALL        0x09
#define OP_RREG         0x16
#define OP_INT          0x18
#define OP_JMP          0x19
#define OP_CLRINT       0x1a

#define JMP_READY       BIT(26)
#define JMP_IRQ         BIT(25)

#define MAX_COMMANDS    (1 << 20)

/*
 * A bench A1N waits 544 us on average (5 us to 3.5 ms) between a job's
 * stall buffer and the next buffer, decoding 640x360 H.264 and H.265.
 */
#define STALL_NS        (544 * 1000)

/* a status address still in the first page is waited for, 100 ms at most */
#define PATCH_MIN       0x1000
#define PATCH_WAIT_NS   (50 * 1000)
#define PATCH_WAITS     2000

/* decoder core registers */
#define DEC_ID          0
#define DEC_CTRL        1
#define DEC_CFG_H264    50      /* 0xc8 */
#define DEC_CFG_PP      54      /* 0xd8 */
#define DEC_CFG_HEVC    56      /* 0xe0 */
#define DEC_FUSE        57      /* 0xe4 */
#define DEC_CFG_CAP     260     /* 0x410 */
#define DEC_BUILD_ID    0x135   /* 0x4d4 */

#define DEC_CTRL_E      BIT(0)
#define DEC_CTRL_IRQ    BIT(8)
#define DEC_CTRL_RDY    BIT(12)

/*
 * VCMD +0x08 to +0x3c, read on a bench A1N after vdec.ko had initialised.
 * Before it they read 0 (+0x3c 0x00400000, +0x40 0x20, +0x4c 0x80008000,
 * +0x5c 0x00100000, +0x04 as here), and vdec.ko neither reads nor writes
 * them: the model latches them at the first start of the sequencer, which
 * is an assumption (the bench cannot say what sets them)
 */
static const uint32_t vcmd_caps[] = {
    0, 1, 0xca000000, 0, 5, 5, 1,
    1, 0xc9, 0xc9, 0xf, 0xf, 0xf, 0x00400073,
};

static void vc8000d_update_irq(IngenicA1Vc8000dState *s)
{
    uint32_t st = s->vcmd[VCMD_STATUS / 4];
    uint32_t en = s->vcmd[VCMD_IRQ_EN / 4];

    qemu_set_irq(s->irq, (st & en) != 0);
}

static uint32_t vc8000d_dec_read(IngenicA1Vc8000dState *s, unsigned idx)
{
    return idx < VC8000D_DEC_WORDS ? s->dec[idx] : 0;
}

static void vc8000d_dec_write(IngenicA1Vc8000dState *s, unsigned idx,
                              uint32_t val)
{
    if (idx >= VC8000D_DEC_WORDS) {
        return;
    }
    switch (idx) {
    case DEC_ID:
    case DEC_CFG_H264:
    case DEC_CFG_PP:
    case DEC_CFG_HEVC:
    case DEC_FUSE:
    case DEC_CFG_CAP:
    case DEC_BUILD_ID:
        return;                         /* read-only */
    case DEC_CTRL:
        s->dec[DEC_CTRL] = val;
        if (val & DEC_CTRL_E) {
            vc8000d_codec_run(s);
            s->dec[DEC_CTRL] &= ~DEC_CTRL_E;
            s->dec[DEC_CTRL] |= DEC_CTRL_IRQ;
        }
        return;
    default:
        s->dec[idx] = val;
    }
}

static uint32_t vc8000d_reg_read(IngenicA1Vc8000dState *s, uint32_t off)
{
    if (off < VC8000D_VCMD_SIZE) {
        return s->vcmd[off / 4];
    }
    if (off >= VC8000D_DEC_BASE &&
        off < VC8000D_DEC_BASE + VC8000D_DEC_WORDS * 4) {
        return vc8000d_dec_read(s, (off - VC8000D_DEC_BASE) / 4);
    }
    if (off >= VC8000D_CACHE_BASE &&
        off < VC8000D_CACHE_BASE + VC8000D_CACHE_WORDS * 4) {
        return s->cache[(off - VC8000D_CACHE_BASE) / 4];
    }
    return 0;
}

static void vc8000d_reg_write(IngenicA1Vc8000dState *s, uint32_t off,
                              uint32_t val)
{
    if (off >= VC8000D_DEC_BASE &&
        off < VC8000D_DEC_BASE + VC8000D_DEC_WORDS * 4) {
        vc8000d_dec_write(s, (off - VC8000D_DEC_BASE) / 4, val);
    } else if (off >= VC8000D_CACHE_BASE &&
               off < VC8000D_CACHE_BASE + VC8000D_CACHE_WORDS * 4) {
        s->cache[(off - VC8000D_CACHE_BASE) / 4] = val;
    } else if (off >= VC8000D_VCMD_SIZE) {
        trace_ingenic_a1_vc8000d_unimp("write", off, val);
    }
}

static bool vc8000d_fetch(IngenicA1Vc8000dState *s, uint64_t addr,
                          uint32_t *w, unsigned n)
{
    return address_space_read(s->as, addr, MEMTXATTRS_UNSPECIFIED, w,
                              n * 4) == MEMTX_OK;
}

/*
 * Run the chain at addr to its end, or to the next stall: true when the
 * chain ended, false when a timer resumes it
 */
static bool vc8000d_run(IngenicA1Vc8000dState *s, uint64_t addr)
{
    unsigned n;

    for (n = 0; n < MAX_COMMANDS; n++) {
        uint32_t w[4] = { 0 };
        uint32_t op, cnt, reg, i;

        if (!vc8000d_fetch(s, addr, w, 1)) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "ingenic-a1-vc8000d: cannot fetch 0x%" PRIx64 "\n",
                          addr);
            return true;
        }
        trace_ingenic_a1_vc8000d_cmd(addr, w[0]);
        op = w[0] >> 27;
        switch (op) {
        case OP_WREG:
            cnt = extract32(w[0], 16, 11);
            reg = w[0] & 0xffff;
            for (i = 0; i < cnt; i++) {
                uint32_t v;

                if (!vc8000d_fetch(s, addr + 4 + i * 4, &v, 1)) {
                    return true;
                }
                vc8000d_reg_write(s, reg + i * 4, v);
            }
            addr += (1 + cnt + ((1 + cnt) & 1)) * 4;
            break;
        case OP_NOP:
            addr += 8;
            break;
        case OP_STALL:
            s->resume = addr + 8;
            s->stalled = true;
            timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      STALL_NS);
            return false;
        case OP_RREG:
            if (!vc8000d_fetch(s, addr, w, 4)) {
                return true;
            }
            cnt = extract32(w[0], 16, 11);
            reg = w[0] & 0xffff;
            trace_ingenic_a1_vc8000d_rreg(addr, reg, cnt,
                                          ((uint64_t)w[2] << 32) + w[1]);
            if (w[2] == 0 && w[1] < PATCH_MIN &&
                s->patch_waits < PATCH_WAITS) {
                /*
                 * A status address in the first page is one the driver has
                 * not filled in yet. On a bench A1N its store lands before
                 * the sequencer fetches the command; a guest on a loaded
                 * host can be milliseconds behind, and the dump would land
                 * on the exception vectors at physical 0.
                 */
                s->patch_waits++;
                s->resume = addr;
                s->stalled = true;
                timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          PATCH_WAIT_NS);
                return false;
            }
            s->patch_waits = 0;
            for (i = 0; i < cnt; i++) {
                uint32_t v = vc8000d_reg_read(s, reg + i * 4);

                address_space_write(s->as, ((uint64_t)w[2] << 32) + w[1] +
                                    i * 4, MEMTXATTRS_UNSPECIFIED, &v, 4);
            }
            addr += 16;
            break;
        case OP_INT:
            s->vcmd[VCMD_STATUS / 4] =
                deposit32(s->vcmd[VCMD_STATUS / 4], 16, 16, w[0] & 0xffff);
            addr += 8;
            break;
        case OP_CLRINT:
            if (!vc8000d_fetch(s, addr, w, 2)) {
                return true;
            }
            vc8000d_reg_write(s, w[0] & 0xffff, w[1]);
            addr += 8;
            break;
        case OP_JMP:
            if (!vc8000d_fetch(s, addr, w, 4)) {
                return true;
            }
            s->vcmd[VCMD_EXE_ADDR / 4] = addr;
            if (w[0] & JMP_IRQ) {
                s->vcmd[VCMD_STATUS / 4] |= VCMD_ST_END;
            }
            if (!(w[0] & JMP_READY)) {
                s->vcmd[VCMD_STATUS / 4] |= VCMD_ST_END;
                return true;
            }
            /* the next buffer's id, which the interrupt handler reads back */
            s->vcmd[VCMD_EXE_ID / 4] = w[3] & 0xffff;
            addr = ((uint64_t)w[2] << 32) | w[1];
            break;
        case OP_END:
        default:
            s->vcmd[VCMD_EXE_ADDR / 4] = addr;
            s->vcmd[VCMD_STATUS / 4] |= VCMD_ST_END;
            return true;
        }
    }
    return true;
}

static void vc8000d_start(IngenicA1Vc8000dState *s)
{
    uint64_t addr = s->vcmd[VCMD_EXE_ADDR / 4];
    unsigned len = extract32(s->vcmd[VCMD_EXE_LEN / 4], 0, 16);

    if (!s->vcmd_latched) {
        memcpy(&s->vcmd[2], vcmd_caps, sizeof(vcmd_caps));
        s->vcmd_latched = true;
    }
    trace_ingenic_a1_vc8000d_run(addr, len);
    timer_del(s->timer);
    s->stalled = false;
    s->patch_waits = 0;
    if (vc8000d_run(s, addr)) {
        vc8000d_update_irq(s);
    }
}

static void vc8000d_stall_done(void *opaque)
{
    IngenicA1Vc8000dState *s = opaque;

    s->stalled = false;
    if (vc8000d_run(s, s->resume)) {
        vc8000d_update_irq(s);
    }
}

static uint64_t vc8000d_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicA1Vc8000dState *s = opaque;
    uint32_t val = vc8000d_reg_read(s, off);

    if (off < VC8000D_VCMD_SIZE) {
        trace_ingenic_a1_vc8000d_vcmd_read(off, val);
    }
    return val;
}

static void vc8000d_write(void *opaque, hwaddr off, uint64_t val,
                          unsigned size)
{
    IngenicA1Vc8000dState *s = opaque;

    if (off >= VC8000D_VCMD_SIZE) {
        vc8000d_reg_write(s, off, val);
        return;
    }
    trace_ingenic_a1_vc8000d_vcmd_write(off, val);
    switch (off) {
    case VCMD_ID ... 0x3c:
        return;
    case VCMD_STATUS:
        s->vcmd[VCMD_STATUS / 4] &= ~val;
        vc8000d_update_irq(s);
        return;
    case VCMD_IRQ_EN:
        s->vcmd[VCMD_IRQ_EN / 4] = val;
        vc8000d_update_irq(s);
        return;
    case VCMD_CTRL:
        s->vcmd[off / 4] = val & ~VCMD_CTRL_START;
        if (val & VCMD_CTRL_START) {
            vc8000d_start(s);
        }
        return;
    default:
        s->vcmd[off / 4] = val;
    }
}

static const MemoryRegionOps vc8000d_ops = {
    .read = vc8000d_read,
    .write = vc8000d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void vc8000d_reset(DeviceState *dev)
{
    IngenicA1Vc8000dState *s = INGENIC_A1_VC8000D(dev);

    memset(s->vcmd, 0, sizeof(s->vcmd));
    memset(s->dec, 0, sizeof(s->dec));
    memset(s->cache, 0, sizeof(s->cache));
    s->vcmd[VCMD_ID / 4] = s->vcmd_version;
    s->vcmd[1] = 0x20200618;
    s->vcmd[0x3c / 4] = 0x00400000;
    s->vcmd[VCMD_CTRL / 4] = 0x20;
    s->vcmd[0x4c / 4] = 0x80008000;
    s->vcmd[0x5c / 4] = 0x00100000;
    s->vcmd_latched = false;
    timer_del(s->timer);
    s->stalled = false;
    s->dec[DEC_ID] = s->dec_id;
    s->dec[DEC_BUILD_ID] = s->build_id;
    s->dec[2] = 0x454;
    s->dec[DEC_CFG_H264] = 0x00101000;
    s->dec[DEC_CFG_PP] = 0x00020400;
    s->dec[DEC_CFG_HEVC] = 0x04801000;
    s->dec[DEC_FUSE] = 0xffffffff;
    s->dec[DEC_CFG_CAP] = 0xc8000000;
    s->cache[0] = 0x5000;
    if (s->codec) {
        vc8000d_codec_reset(s->codec);
    }
    vc8000d_update_irq(s);
}

static void vc8000d_realize(DeviceState *dev, Error **errp)
{
    IngenicA1Vc8000dState *s = INGENIC_A1_VC8000D(dev);

    s->as = &address_space_memory;
    memory_region_init_io(&s->iomem, OBJECT(s), &vc8000d_ops, s,
                          TYPE_INGENIC_A1_VC8000D,
                          INGENIC_A1_VC8000D_IOSIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->irq);
    s->codec = vc8000d_codec_new(s);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, vc8000d_stall_done, s);
}

static int vc8000d_post_load(void *opaque, int version_id)
{
    IngenicA1Vc8000dState *s = opaque;

    if (s->stalled) {
        timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + STALL_NS);
    }
    return 0;
}

static const VMStateDescription vc8000d_vmstate = {
    .name = TYPE_INGENIC_A1_VC8000D,
    .version_id = 3,
    .minimum_version_id = 2,
    .post_load = vc8000d_post_load,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(vcmd_latched, IngenicA1Vc8000dState),
        VMSTATE_BOOL_V(stalled, IngenicA1Vc8000dState, 3),
        VMSTATE_UINT64_V(resume, IngenicA1Vc8000dState, 3),
        VMSTATE_UINT32_ARRAY(vcmd, IngenicA1Vc8000dState,
                             VC8000D_VCMD_SIZE / 4),
        VMSTATE_UINT32_ARRAY(dec, IngenicA1Vc8000dState, VC8000D_DEC_WORDS),
        VMSTATE_UINT32_ARRAY(cache, IngenicA1Vc8000dState,
                             VC8000D_CACHE_WORDS),
        VMSTATE_END_OF_LIST()
    },
};

static const Property vc8000d_props[] = {
    DEFINE_PROP_UINT32("vcmd-version", IngenicA1Vc8000dState, vcmd_version,
                       0x43421200),
    DEFINE_PROP_UINT32("id", IngenicA1Vc8000dState, dec_id, 0x80018000),
    DEFINE_PROP_UINT32("build-id", IngenicA1Vc8000dState, build_id, 0x1f87),
};

static void vc8000d_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = vc8000d_realize;
    device_class_set_legacy_reset(dc, vc8000d_reset);
    dc->vmsd = &vc8000d_vmstate;
    device_class_set_props(dc, vc8000d_props);
}

static const TypeInfo vc8000d_info = {
    .name = TYPE_INGENIC_A1_VC8000D,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicA1Vc8000dState),
    .class_init = vc8000d_class_init,
};

static void vc8000d_register_types(void)
{
    type_register_static(&vc8000d_info);
}

type_init(vc8000d_register_types)
