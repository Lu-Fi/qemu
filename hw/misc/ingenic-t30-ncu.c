/*
 * Ingenic T30 noise reducer (NCU) and its post-processor
 *
 * A 4 KiB block at 0x13090000 between the ISP's line buffer and the
 * multi-scaler, on INTC source 33 (the post-processor on 35), reset by the
 * CPM's SRBC1 bit 2. tx-isp-t30 streams the ISP's frames through it, and
 * libimp programs its 3D noise reduction through /dev/mem when
 * /etc/sensor/ncu_<sensor>.bin is there (the SDK ships one table under 27
 * sensors' names). Its control words, as a T30X reads them:
 *
 * - RESET (0x0) reads 0x11, bits 15:8 counting the writes with bit 0
 *   clear: each a soft reset, which clears STOP's frame done and points the
 *   reference ping-pong back at REF0.
 *
 * - START (0x4): bit 16 reads idle. Writing bit 1 marks the next frame the
 *   first, bit 0 of bits 15:0, which shift up a nibble as each frame ends.
 *   Writing bit 0 starts a frame from memory (FUN_CNTRL bit 4); on the line
 *   buffer the NCU takes each frame the ISP puts out by itself.
 *
 * - STOP (0x8): bit 0 stops it after the frame in hand, bits 15:8 count the
 *   stops asked, bit 1 is set as a frame ends (and at a stop while idle)
 *   until the interrupt is cleared or a soft reset.
 *
 * - SREG_CNTRL (0xc), bits 4 and 2:0: bit 0 loads the function registers
 *   as the next frame starts, at once with bit 4, then reads 0. A frame
 *   from memory started before any load since the reset, or while stopped,
 *   never ends; only the CPM's reset ends it.
 *
 * - INT_CNTRL (0x10): bit 0 enables the interrupt, writing bit 1 clears bit
 *   4 (a frame's end) and STOP bit 1; bit 8 is the interrupt line, low
 *   while raised.
 *
 * - 0x14 reads 0x0a170510. DEBUG_SEL (0x18) picks what DEBUG_VAL (0x1c)
 *   shows: select 0 counts the frames started, the others the state of
 *   units the model has not, here as an idle T30X shows them.
 *
 * - 0x20 bits 31:30 show the buffer the last frame went to, REF0 (bit 30)
 *   or REF1. Writing bit 0 switches it to the clock count of the last
 *   frame (12529 for a 128x64 frame from memory on a T30X; the model counts
 *   its bytes), bit 1 back.
 *
 * The first frame goes to REF0 (0x38 and 0x48), the later ones alternate
 * REF1 (0x50 and 0x58) and REF0. With FUN_CNTRL bit 5 a frame from memory
 * (CUR, 0x30 and 0x40) is written there, NV12 with each plane's stride; on
 * the line buffer the kernel leaves the addresses 0 and the frame goes on
 * to the multi-scaler.
 *
 * The functions at their reset values (FUN_CNTRL 0x0c000000) pass a frame
 * through unchanged, as thingino runs them, having no such file. The 3D
 * noise reduction libimp's tables ask for, its statistics (STA, PSN) and
 * their ring of frame slots are not modelled: frames pass through. The
 * post-processor (0x500) has the same control words but no start, and a
 * first frame flag (0x508) only its reset clears; nothing on a T30 uses it.
 * The other registers keep the bits a T30X keeps.
 *
 * Not modelled: on a T30X a stop while a line buffer frame is in hand hangs
 * the NCU until its reset; the model ends the frame.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-t30-ncu.h"
#include "trace.h"

#define NCU_REGS            INGENIC_T30_NCU_REGS

#define NCU_RESET           0x000
#define NCU_START           0x004
#define  START_RUN          BIT(0)
#define  START_FIRST        BIT(1)
#define  START_IDLE         BIT(16)
#define NCU_STOP            0x008
#define NCU_SREG_CNTRL      0x00c
#define  SREG_LOAD          BIT(0)
#define  SREG_NOW           BIT(4)
#define  SREG_MASK          0x17
#define NCU_INT_CNTRL       0x010
#define  INT_EN             BIT(0)
#define  INT_CLEAR          BIT(1)
#define NCU_STATUS          0x014
#define NCU_DEBUG_SEL       0x018
#define NCU_DEBUG_VAL       0x01c
#define NCU_CLK_CNT         0x020
#define NCU_FUN_CNTRL       0x024
#define  FUN_IN_DDR         BIT(4)
#define  FUN_OUT_DDR        BIT(5)
#define  FUN_PASS           0x0c000000  /* the functions at reset */
#define NCU_FRAME_SIZE      0x02c
#define Y_CUR_ADDR          0x030
#define Y_CUR_STRIDE        0x034
#define Y_REF0_ADDR         0x038
#define Y_REF0_STRIDE       0x03c
#define UV_CUR_ADDR         0x040
#define UV_CUR_STRIDE       0x044
#define UV_REF0_ADDR        0x048
#define UV_REF0_STRIDE      0x04c
#define Y_REF1_ADDR         0x050
#define Y_REF1_STRIDE       0x054
#define UV_REF1_ADDR        0x058
#define UV_REF1_STRIDE      0x05c

#define PP_RESET            0x500
#define PP_STOP             0x504
#define PP_FST_FRM          0x508
#define PP_SREG_CNTRL       0x50c
#define PP_INT_CNTRL        0x510
#define PP_STATUS           0x514
#define PP_DBG_SEL          0x518
#define PP_DBG_VAL          0x51c
#define PP_CLK_CNT          0x520

#define NCU_VERSION         0x0a170510

#define R(off)              (s->regs[(off) / 4])

/* The bits a T30X keeps; the control words above are handled apart */
static const uint32_t ncu_wmask[NCU_REGS] = {
    [0x018 / 4] = 0x0000001f, [0x024 / 4 ... 0x028 / 4] = 0xffffffff,
    [0x02c / 4] = 0x0fff0fff, [0x030 / 4] = 0xffffffff,
    [0x034 / 4] = 0x0000ffff, [0x038 / 4] = 0xffffffff,
    [0x03c / 4] = 0x0000ffff, [0x040 / 4] = 0xffffffff,
    [0x044 / 4] = 0x0000ffff, [0x048 / 4] = 0xffffffff,
    [0x04c / 4] = 0x0000ffff, [0x050 / 4] = 0xffffffff,
    [0x054 / 4] = 0x0000ffff, [0x058 / 4] = 0xffffffff,
    [0x05c / 4] = 0x0000ffff, [0x060 / 4] = 0x0000003f,
    [0x064 / 4] = 0xffffffff, [0x068 / 4] = 0x0000ffff,
    [0x06c / 4 ... 0x070 / 4] = 0xffffffff, [0x074 / 4] = 0x0000ffff,
    [0x078 / 4 ... 0x07c / 4] = 0xffffffff, [0x080 / 4] = 0x0000ffff,
    [0x084 / 4] = 0xffffffff, [0x088 / 4] = 0x0000003f,
    [0x08c / 4] = 0xffffffff, [0x090 / 4] = 0x0000ffff,
    [0x094 / 4 ... 0x098 / 4] = 0xffffffff, [0x09c / 4] = 0x0000ffff,
    [0x0a0 / 4 ... 0x0a4 / 4] = 0xffffffff, [0x0a8 / 4] = 0x0000ffff,
    [0x0ac / 4 ... 0x0b4 / 4] = 0xffffffff, [0x0b8 / 4] = 0x0fff0fff,
    [0x0bc / 4] = 0x00000111, [0x0c0 / 4 ... 0x0cc / 4] = 0x00ff00ff,
    [0x0e4 / 4 ... 0x0f0 / 4] = 0x00ff00ff, [0x180 / 4] = 0x00000003,
    [0x184 / 4] = 0xffffffff, [0x188 / 4] = 0x001f1f1f,
    [0x18c / 4] = 0x7f7f7f00, [0x190 / 4] = 0xffffffff,
    [0x194 / 4] = 0x000f0f0f, [0x198 / 4] = 0x1f1f1f00,
    [0x19c / 4] = 0xffffffff, [0x1a0 / 4] = 0x00000033,
    [0x1a4 / 4] = 0x03ff03ff, [0x1a8 / 4 ... 0x1ac / 4] = 0x0003ffff,
    [0x1b0 / 4] = 0x0001ffff, [0x1d0 / 4] = 0x000707f3,
    [0x1d4 / 4] = 0x1ff7fdff, [0x1d8 / 4] = 0x0007fdff,
    [0x1dc / 4 ... 0x1e0 / 4] = 0x01ff01ff,
    [0x1e4 / 4 ... 0x1e8 / 4] = 0x03ff03ff,
    [0x1ec / 4 ... 0x1f4 / 4] = 0x3f3f3f3f, [0x1f8 / 4] = 0x003f3f3f,
    [0x1fc / 4] = 0x0000ffff, [0x200 / 4] = 0x0000ff7f,
    [0x204 / 4] = 0x000001ff, [0x208 / 4] = 0x3f3f03ff,
    [0x220 / 4] = 0x7ff7fdff, [0x224 / 4 ... 0x228 / 4] = 0x00ff00ff,
    [0x22c / 4] = 0x01ff01ff, [0x230 / 4] = 0x000001ff,
    [0x234 / 4] = 0x03ff03ff, [0x238 / 4] = 0x000003ff,
    [0x23c / 4 ... 0x240 / 4] = 0x00ff00ff, [0x244 / 4] = 0x01ff01ff,
    [0x248 / 4] = 0x000001ff, [0x24c / 4] = 0x03ff03ff,
    [0x250 / 4] = 0x000003ff, [0x254 / 4] = 0x07ff07ff,
    [0x258 / 4 ... 0x25c / 4] = 0x00ff00ff, [0x260 / 4] = 0x01ff01ff,
    [0x264 / 4] = 0x000001ff, [0x268 / 4] = 0x03ff03ff,
    [0x26c / 4] = 0x000003ff, [0x270 / 4 ... 0x274 / 4] = 0x00ff00ff,
    [0x278 / 4] = 0x01ff01ff, [0x27c / 4] = 0x000001ff,
    [0x280 / 4] = 0x03ff03ff, [0x284 / 4] = 0x000003ff,
    [0x288 / 4] = 0x07ff07ff, [0x2a0 / 4] = 0x1ffff31f,
    [0x2a4 / 4] = 0x01ff01ff, [0x2a8 / 4] = 0x000000f3,
    [0x2ac / 4] = 0x00ff00ff, [0x2b0 / 4] = 0x00ff01ff,
    [0x2b4 / 4] = 0x01ff01ff, [0x2b8 / 4] = 0x001fffff,
    [0x2bc / 4] = 0x7fff7fff, [0x2c0 / 4] = 0x00000f0f,
    [0x2c4 / 4] = 0x00ff03ff, [0x2d0 / 4] = 0xffffffff,
    [0x2d4 / 4] = 0x1f1f1f1f, [0x2d8 / 4] = 0x3f3f3f3f,
    [0x2dc / 4] = 0x00033333, [0x300 / 4] = 0x0001ffff,
    [0x304 / 4] = 0x00ffffff, [0x308 / 4] = 0x001f001f,
    [0x30c / 4] = 0x003f003f, [0x310 / 4] = 0x00ffffff,
    [0x314 / 4 ... 0x318 / 4] = 0x000f000f, [0x31c / 4] = 0x0003ffff,
    [0x320 / 4] = 0x0001ffff, [0x324 / 4] = 0x03ff03ff,
    [0x328 / 4 ... 0x32c / 4] = 0x0003ffff, [0x340 / 4] = 0x71ff00f3,
    [0x344 / 4] = 0x00ff00ff, [0x348 / 4 ... 0x350 / 4] = 0x01ff01ff,
    [0x354 / 4] = 0x000001ff, [0x358 / 4] = 0x01ff01ff,
    [0x35c / 4] = 0x000001ff, [0x360 / 4 ... 0x368 / 4] = 0x3f3f3f3f,
    [0x36c / 4] = 0x003f3f3f, [0x370 / 4] = 0x3f3f03ff,
    [0x390 / 4] = 0x000001ff, [0x394 / 4] = 0x00ffffff,
    [0x398 / 4 ... 0x39c / 4] = 0x01ff01ff, [0x3a0 / 4] = 0x00ffffff,
    [0x3a4 / 4 ... 0x3a8 / 4] = 0x01ff01ff, [0x3ac / 4] = 0x000001ff,
    [0x3b0 / 4] = 0x07ff07ff, [0x3b4 / 4] = 0x00ffffff,
    [0x3b8 / 4 ... 0x3bc / 4] = 0x01ff01ff, [0x3c0 / 4] = 0x00ffffff,
    [0x3c4 / 4 ... 0x3c8 / 4] = 0x01ff01ff, [0x3cc / 4] = 0x000001ff,
    [0x3d0 / 4] = 0x07ff07ff, [0x3e0 / 4] = 0x1ffff1ff,
    [0x3e4 / 4] = 0x01ff01ff, [0x3e8 / 4] = 0x00ff00ff,
    [0x3ec / 4] = 0x00ff01ff, [0x3f0 / 4] = 0xf1ff01ff,
    [0x3f4 / 4] = 0x001fffff, [0x3f8 / 4] = 0x0013ffff,
    [0x3fc / 4] = 0x00000f0f, [0x400 / 4] = 0x00ff03ff,
    [0x518 / 4] = 0x0000000f, [0x524 / 4] = 0x0fff0fff,
    [0x528 / 4] = 0xffffffff, [0x52c / 4] = 0x0000ffff,
    [0x530 / 4] = 0xffffffff, [0x534 / 4] = 0x0000ffff,
    [0x538 / 4] = 0xffffffff, [0x53c / 4] = 0x0000ffff,
    [0x540 / 4] = 0xffffffff, [0x544 / 4] = 0x0000ffff,
    [0x548 / 4] = 0xffffffff, [0x54c / 4] = 0x0000ffff,
    [0x550 / 4] = 0xffffffff, [0x554 / 4] = 0x0000ffff,
    [0x558 / 4] = 0x00000001, [0x55c / 4] = 0xffffffff,
    [0x560 / 4 ... 0x56c / 4] = 0x001fffff, [0x570 / 4] = 0x0000000f,
    [0x574 / 4] = 0x000001ff, [0x578 / 4] = 0x0000001f,
    [0x57c / 4] = 0x00000111,
};

static const uint32_t ncu_reset[NCU_REGS] = {
    [0x024 / 4] = 0x0c000000, [0x028 / 4] = 0xff00ff00,
    [0x02c / 4] = 0x00100010, [0x0b0 / 4 ... 0x0b4 / 4] = 0x02060101,
    [0x0b8 / 4] = 0x00200030, [0x2ac / 4] = 0x00010001,
    [0x2c4 / 4] = 0x00020002, [0x3e8 / 4] = 0x00010001,
    [0x400 / 4] = 0x00020002, [0x524 / 4] = 0x00100010,
    [0x55c / 4] = 0x02010201,
};

/* DEBUG_VAL and PP_DBG_VAL by select, as an idle T30X reads them */
static const uint32_t ncu_debug[32] = {
    [0x01] = 0x00000005, [0x02] = 0x00004101, [0x05] = 0x00000001,
    [0x06] = 0x05414444, [0x08] = 0x00100011, [0x09] = 0x2c2c4444,
    [0x0a] = 0x01010000, [0x0c] = 0x01010000, [0x0e] = 0x00008888,
    [0x0f] = 0x00005000, [0x10] = 0x00060010, [0x11] = 0x00060008,
    [0x1e] = 0x00011212, [0x1f] = 0x20170614,
};

static const uint32_t pp_debug[16] = {
    [0x0] = 0x00000001, [0x1] = 0x00000004, [0x2] = 0x00008880,
    [0x3] = 0x00004101, [0x5] = 0x00000444,
};

static void ncu_update(IngenicT30NcuState *s)
{
    qemu_set_irq(s->irq, s->ncu.int_en && s->ncu.done);
    qemu_set_irq(s->pp_irq, s->pp.int_en && s->pp.done);
}

static uint32_t ctl_reset_read(IngenicT30NcuCtl *c)
{
    return 0x11 | c->resets << 8;
}

static uint32_t ctl_stop_read(IngenicT30NcuCtl *c)
{
    return c->stop | c->stop_done << 1 | c->stops << 8;
}

static uint32_t ctl_int_read(IngenicT30NcuCtl *c)
{
    return c->int_en | c->done << 4 | !(c->int_en && c->done) << 8;
}

static void ctl_soft_reset(IngenicT30NcuCtl *c, uint32_t v)
{
    if (!(v & 1)) {
        c->resets++;
        c->stop_done = false;
    }
}

/* A stop asked while idle is done at once */
static void ctl_stop(IngenicT30NcuCtl *c, uint32_t v, bool busy)
{
    if ((v & 1) && !c->stop) {
        c->stops++;
        if (!busy) {
            c->stop_done = c->done = true;
        }
    }
    c->stop = v & 1;
}

static void ctl_int(IngenicT30NcuCtl *c, uint32_t v)
{
    c->int_en = v & INT_EN;
    if (v & INT_CLEAR) {
        c->done = c->stop_done = false;
    }
}

static uint32_t ctl_sreg(uint32_t v)
{
    v &= SREG_MASK;
    return (v & (SREG_LOAD | SREG_NOW)) == (SREG_LOAD | SREG_NOW) ?
           v & ~SREG_LOAD : v;
}

static uint32_t ncu_frame_bytes(IngenicT30NcuState *s)
{
    uint32_t size = R(NCU_FRAME_SIZE);

    return extract32(size, 0, 12) * extract32(size, 16, 12) * 3 / 2;
}

/*
 * A frame starts: it is counted, takes the load asked for and its buffer.
 * False if it cannot run.
 */
static bool ncu_begin(IngenicT30NcuState *s)
{
    uint32_t fun = R(NCU_FUN_CNTRL);

    s->frames++;
    if (s->ncu.sreg & SREG_LOAD) {
        s->ncu.sreg &= ~SREG_LOAD;
        s->loaded = true;
    }
    s->ref1 = (s->first & 1) ? false : !s->ref1;
    s->busy = true;
    trace_ingenic_t30_ncu_frame(s->frames, s->ref1, fun);
    if (!s->loaded || s->ncu.stop) {
        s->hung = true;
        return false;
    }
    if ((fun & ~0xff) != FUN_PASS && fun != s->unimp_fun) {
        qemu_log_mask(LOG_UNIMP, "ingenic-t30-ncu: noise reduction "
                      "(FUN_CNTRL 0x%08x) not modelled: frames pass "
                      "through\n", fun);
        s->unimp_fun = fun;
    }
    return true;
}

static void ncu_end(IngenicT30NcuState *s)
{
    s->busy = false;
    s->first <<= 4;
    s->ncu.stop_done = s->ncu.done = true;
    if (s->clk_view) {
        s->clk = ncu_frame_bytes(s);
    }
    ncu_update(s);
}

static void ncu_copy_plane(uint32_t src, uint32_t sstride, uint32_t dst,
                           uint32_t dstride, uint32_t w, uint32_t h)
{
    g_autofree uint8_t *row = g_malloc(w);

    for (uint32_t y = 0; y < h; y++) {
        address_space_read(&address_space_memory, src + y * sstride,
                           MEMTXATTRS_UNSPECIFIED, row, w);
        address_space_write(&address_space_memory, dst + y * dstride,
                            MEMTXATTRS_UNSPECIFIED, row, w);
    }
}

/* A frame from memory: to REF0 or REF1, or on to the multi-scaler */
static void ncu_run(void *opaque)
{
    IngenicT30NcuState *s = opaque;
    uint32_t w = extract32(R(NCU_FRAME_SIZE), 0, 12);
    uint32_t h = extract32(R(NCU_FRAME_SIZE), 16, 12);

    if (!s->busy || s->hung) {
        return;
    }
    if (R(NCU_FUN_CNTRL) & FUN_OUT_DDR) {
        hwaddr y = s->ref1 ? Y_REF1_ADDR : Y_REF0_ADDR;
        hwaddr uv = s->ref1 ? UV_REF1_ADDR : UV_REF0_ADDR;

        ncu_copy_plane(R(Y_CUR_ADDR), R(Y_CUR_STRIDE), R(y), R(y + 4), w, h);
        ncu_copy_plane(R(UV_CUR_ADDR), R(UV_CUR_STRIDE), R(uv), R(uv + 4),
                       w, h / 2);
    }
    ncu_end(s);
    if (!(R(NCU_FUN_CNTRL) & FUN_OUT_DDR)) {
        qemu_irq_pulse(s->frame_out);
    }
}

/* The ISP starts a frame into the line buffer */
static void ncu_frame_start(void *opaque, int n, int level)
{
    IngenicT30NcuState *s = INGENIC_T30_NCU(opaque);

    if (!level || s->busy || s->ncu.stop ||
        (R(NCU_FUN_CNTRL) & FUN_IN_DDR) ||
        !(s->loaded || (s->ncu.sreg & SREG_LOAD))) {
        return;
    }
    ncu_begin(s);
}

/* The ISP's frame is through: on to the multi-scaler, if the NCU took it */
static void ncu_frame_end(void *opaque, int n, int level)
{
    IngenicT30NcuState *s = INGENIC_T30_NCU(opaque);

    if (!level || !s->busy || s->hung ||
        (R(NCU_FUN_CNTRL) & FUN_IN_DDR)) {
        return;
    }
    ncu_end(s);
    qemu_irq_pulse(s->frame_out);
}

static uint64_t ingenic_t30_ncu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicT30NcuState *s = INGENIC_T30_NCU(opaque);
    uint32_t v;

    switch (off) {
    case NCU_RESET:
        v = ctl_reset_read(&s->ncu);
        break;
    case NCU_START:
        v = (s->busy ? 0 : START_IDLE) | s->first;
        break;
    case NCU_STOP:
        v = ctl_stop_read(&s->ncu);
        break;
    case NCU_SREG_CNTRL:
        v = s->ncu.sreg;
        break;
    case NCU_INT_CNTRL:
        v = ctl_int_read(&s->ncu);
        break;
    case NCU_STATUS:
    case PP_STATUS:
        v = NCU_VERSION;
        break;
    case NCU_DEBUG_VAL:
        v = R(NCU_DEBUG_SEL) ? ncu_debug[R(NCU_DEBUG_SEL)] : s->frames;
        break;
    case NCU_CLK_CNT:
        v = s->clk_view ? s->clk : s->ref1 ? BIT(31) : BIT(30);
        break;
    case PP_RESET:
        v = ctl_reset_read(&s->pp);
        break;
    case PP_STOP:
        v = ctl_stop_read(&s->pp);
        break;
    case PP_FST_FRM:
        v = s->pp_first;
        break;
    case PP_SREG_CNTRL:
        v = s->pp.sreg;
        break;
    case PP_INT_CNTRL:
        v = ctl_int_read(&s->pp);
        break;
    case PP_DBG_VAL:
        v = pp_debug[R(PP_DBG_SEL)];
        break;
    default:
        v = off < sizeof(s->regs) ? R(off) : 0;
        break;
    }
    trace_ingenic_t30_ncu_read(off, v);
    return v;
}

static void ingenic_t30_ncu_write(void *opaque, hwaddr off, uint64_t value,
                                  unsigned size)
{
    IngenicT30NcuState *s = INGENIC_T30_NCU(opaque);
    uint32_t v = value;

    trace_ingenic_t30_ncu_write(off, v);
    switch (off) {
    case NCU_RESET:
        ctl_soft_reset(&s->ncu, v);
        if (!(v & 1)) {
            s->ref1 = false;
        }
        break;
    case NCU_START:
        if (v & START_FIRST) {
            s->first |= 1;
        }
        if ((v & START_RUN) && !s->busy &&
            (R(NCU_FUN_CNTRL) & FUN_IN_DDR) && ncu_begin(s)) {
            qemu_bh_schedule(s->bh);
        }
        break;
    case NCU_STOP:
        ctl_stop(&s->ncu, v, s->busy);
        break;
    case NCU_SREG_CNTRL:
        s->ncu.sreg = ctl_sreg(v);
        if ((v & (SREG_LOAD | SREG_NOW)) == (SREG_LOAD | SREG_NOW)) {
            s->loaded = true;
        }
        break;
    case NCU_INT_CNTRL:
        ctl_int(&s->ncu, v);
        break;
    case NCU_CLK_CNT:
        if (v & 1) {
            s->clk_view = true;
            s->clk = 0;
        } else if (v & 2) {
            s->clk_view = false;
        }
        break;
    case PP_RESET:
        ctl_soft_reset(&s->pp, v);
        break;
    case PP_STOP:
        ctl_stop(&s->pp, v, false);
        break;
    case PP_FST_FRM:
        s->pp_first |= v & 1;
        break;
    case PP_SREG_CNTRL:
        s->pp.sreg = ctl_sreg(v);
        break;
    case PP_INT_CNTRL:
        ctl_int(&s->pp, v);
        break;
    default:
        if (off < sizeof(s->regs)) {
            R(off) = v & ncu_wmask[off / 4];
        }
        break;
    }
    ncu_update(s);
}

static const MemoryRegionOps ingenic_t30_ncu_ops = {
    .read = ingenic_t30_ncu_read,
    .write = ingenic_t30_ncu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_t30_ncu_reset_hold(Object *obj, ResetType type)
{
    IngenicT30NcuState *s = INGENIC_T30_NCU(obj);

    qemu_bh_cancel(s->bh);
    memcpy(s->regs, ncu_reset, sizeof(s->regs));
    memset(&s->ncu, 0, sizeof(s->ncu));
    memset(&s->pp, 0, sizeof(s->pp));
    s->ncu.sreg = s->pp.sreg = 0x4;
    s->frames = 0;
    s->clk = 0;
    s->first = 0;
    s->busy = false;
    s->hung = false;
    s->loaded = false;
    s->ref1 = false;
    s->clk_view = false;
    s->pp_first = false;
    s->unimp_fun = 0;
}

static void ingenic_t30_ncu_reset_exit(Object *obj, ResetType type)
{
    ncu_update(INGENIC_T30_NCU(obj));
}

/* The CPM's SRBC1 bit 2 */
static void ncu_cpm_reset(void *opaque, int n, int level)
{
    if (level) {
        ingenic_t30_ncu_reset_hold(OBJECT(opaque), RESET_TYPE_COLD);
        ncu_update(INGENIC_T30_NCU(opaque));
    }
}

static void ingenic_t30_ncu_init(Object *obj)
{
    IngenicT30NcuState *s = INGENIC_T30_NCU(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_t30_ncu_ops, s,
                          TYPE_INGENIC_T30_NCU, INGENIC_T30_NCU_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->pp_irq);
    qdev_init_gpio_in_named(DEVICE(obj), ncu_cpm_reset, "reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), ncu_frame_start, "frame-start", 1);
    qdev_init_gpio_in_named(DEVICE(obj), ncu_frame_end, "frame", 1);
    qdev_init_gpio_out_named(DEVICE(obj), &s->frame_out, "frame-out", 1);
    s->bh = qemu_bh_new(ncu_run, s);
}

static const VMStateDescription vmstate_ingenic_t30_ncu_ctl = {
    .name = TYPE_INGENIC_T30_NCU "/ctl",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8(resets, IngenicT30NcuCtl),
        VMSTATE_UINT8(stops, IngenicT30NcuCtl),
        VMSTATE_BOOL(stop, IngenicT30NcuCtl),
        VMSTATE_BOOL(stop_done, IngenicT30NcuCtl),
        VMSTATE_BOOL(int_en, IngenicT30NcuCtl),
        VMSTATE_BOOL(done, IngenicT30NcuCtl),
        VMSTATE_UINT32(sreg, IngenicT30NcuCtl),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_t30_ncu = {
    .name = TYPE_INGENIC_T30_NCU,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicT30NcuState, NCU_REGS),
        VMSTATE_STRUCT(ncu, IngenicT30NcuState, 1,
                       vmstate_ingenic_t30_ncu_ctl, IngenicT30NcuCtl),
        VMSTATE_STRUCT(pp, IngenicT30NcuState, 1,
                       vmstate_ingenic_t30_ncu_ctl, IngenicT30NcuCtl),
        VMSTATE_UINT32(frames, IngenicT30NcuState),
        VMSTATE_UINT32(clk, IngenicT30NcuState),
        VMSTATE_UINT16(first, IngenicT30NcuState),
        VMSTATE_BOOL(busy, IngenicT30NcuState),
        VMSTATE_BOOL(hung, IngenicT30NcuState),
        VMSTATE_BOOL(loaded, IngenicT30NcuState),
        VMSTATE_BOOL(ref1, IngenicT30NcuState),
        VMSTATE_BOOL(clk_view, IngenicT30NcuState),
        VMSTATE_BOOL(pp_first, IngenicT30NcuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_t30_ncu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic T30 noise reducer";
    dc->vmsd = &vmstate_ingenic_t30_ncu;
    rc->phases.hold = ingenic_t30_ncu_reset_hold;
    rc->phases.exit = ingenic_t30_ncu_reset_exit;
}

static const TypeInfo ingenic_t30_ncu_types[] = {
    {
        .name = TYPE_INGENIC_T30_NCU,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicT30NcuState),
        .instance_init = ingenic_t30_ncu_init,
        .class_init = ingenic_t30_ncu_class_init,
    },
};

DEFINE_TYPES(ingenic_t30_ncu_types)
