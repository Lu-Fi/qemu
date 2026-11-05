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
 *   or REF1, bits 15:8 the statistics slot the last frame wrote and 7:0
 *   the one it read. Writing bit 0 switches it to the clock count of the
 *   last frame (12529 for a 128x64 frame from memory on a T30X; the model
 *   counts its bytes), bit 1 back.
 *
 * The first frame goes to REF0 (0x38 and 0x48), the later ones alternate
 * REF1 (0x50 and 0x58) and REF0. With FUN_CNTRL bit 5 a frame from memory
 * (CUR, 0x30 and 0x40) is written there, NV12 with each plane's stride; on
 * the line buffer the kernel leaves the addresses 0 and the frame goes on
 * to the multi-scaler.
 *
 * The functions at their reset values (FUN_CNTRL 0x0c000000) pass a frame
 * through unchanged, as thingino runs them, having no such file. The 3D
 * noise reduction follows a T30X with libimp's settings, the frame after
 * the first-frame mark counting as the first. On the line buffer it works
 * on the frame the ISP puts out, at the NCU's frame size, writes it to the
 * reference as a T30X does (libimp's REF0 and REF1 are one buffer), and
 * the multi-scaler scales its channels from it:
 *
 * - Luma (FUN_CNTRL bits 8 and 9): the first frame is stage 2 alone (each
 *   pixel the mean of the pixels of its 5 x 5 within 0x2d0 of it). Later
 *   ones blend each pixel with its box mean (0x220 bits 30:29) and the
 *   reference, rounded once, the reference weighing y / 256, then stage 2.
 *   On the second frame y is the curve's (0x1dc). From the third on, a
 *   pixel is static where its measure, the difference of the box means
 *   (0x1d0) of cur and ref, each doubled and rounded on its own, is under
 *   0x1d4, less 0x2b0 in a block of age 0 or a byte of the table at 0x1ec
 *   by age; a static pixel's y grows with its block's age. In the picture's
 *   corners (0x208) the measure is taken down.
 *
 * - Statistics (bit 19): each frame from the second writes a slot of the
 *   ring at 0x8c (slots 0x94 apart, 0x88 + 1 of them), a header row with
 *   the frame count, then a byte per block of 0x2c0 (28 x 28 for libimp)
 *   its age: 0 where 0x2b8 or more pixels' measure (box 0x2a8) is above
 *   0x2b4, less the age's cut, else one more than in the slot read, up to
 *   0x2a0 bits 4:0.
 *
 * - Chroma (bits 10, 11, 14): the same blend with a 5 x 5 box and its own
 *   weights, U and V each by its own 5 x 5 measure: on the second frame
 *   0x350's low or high half, from the third the luma blocks' ages with the
 *   chroma's registers (0x348 to 0x3ec); then false-colour coring (0x3f8)
 *   of U and V near grey.
 *
 * Not modelled:
 * - On a T30X a stop while a line buffer frame is in hand hangs the NCU
 *   until its reset; the model ends the frame.
 * - The chroma's own statistics (libimp leaves their addresses 0), the
 *   post-processor, PSN and the stages libimp leaves at 0 (0x180 to 0x1b0).
 * The post-processor (0x500) has the same control words but no start, and
 * a first frame flag (0x508) only its reset clears; nothing on a T30 uses
 * it. The other registers keep the bits a T30X keeps.
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
#include "hw/sensor/image-sensor.h"
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
#define  FUN_Y_TDNS         BIT(8)
#define  FUN_Y_SDNS         BIT(9)
#define  FUN_UV_TDNS        BIT(10)
#define  FUN_UV_SDNS        BIT(11)
#define  FUN_UV_FC          BIT(14)
#define  FUN_Y_STA          BIT(19)
#define  FUN_NR             (FUN_Y_TDNS | FUN_Y_SDNS | FUN_UV_TDNS | \
                             FUN_UV_SDNS | FUN_UV_FC)
#define NCU_CUT_VALUE       0x028
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

#define NCU_STA_SLOTS       0x088       /* the statistics ring's last slot */
#define Y_STA_ADDR          0x08c
#define Y_STA_STRIDE        0x090
#define Y_STA_FRAME         0x094       /* a slot's bytes */
#define Y_SE_CON            0x1d0       /* [1:0] the measure's window */
#define Y_SE_SGM_X          0x1d4       /* [8:0] static below */
#define Y_SE_SGM_Y0         0x1dc       /* [8:0] the reference weight */
#define Y_SE_STA_ADJ        0x1ec       /* bytes, off Y_SE_SGM_X by age */
#define Y_SE_STA_EN0        0x1fc
#define Y_SE_STA_EN1        0x200
#define Y_SE_CORNER         0x208       /* [9:0] size, [21:16], [29:24] */
#define Y_STA_WEI           0x2a0
#define Y_STA_FULL_WEI      0x2a4       /* [8:0] at the oldest ages */
#define Y_STA_WIN           0x2a8       /* [1:0] the count's window */
#define Y_STA_NEW_ADJ       0x2b0       /* [8:0] off Y_SE_SGM_X at age 0 */
#define Y_STA_DIFF_THRES    0x2b4
#define Y_STA_MV_NUM        0x2b8
#define Y_STA_BLK_SIZE      0x2c0       /* [11:8] and [3:0], less 16 */
#define Y_STA_BLKS          0x2c4       /* [23:16] across, [9:0] down */
#define Y_FUSION_AVE        0x220
#define Y_ST2_SGM_P_X       0x2d0
#define Y_ST2_SGM_P_Y       0x2d4
#define Y_ST2_SGM_P_S       0x2d8
#define Y_ST2_GAUS          0x2dc
#define UV_SE_CON           0x340
#define UV_SE_SGM_X         0x348       /* [8:0] U, [24:16] V */
#define UV_SE_SGM_Y0        0x350       /* [8:0] static, [24:16] moving */
#define UV_SE_STA_ADJ       0x360
#define UV_SE_CORNER        0x370
#define UV_FUSION_MV_JUDGE  0x390
#define UV_FUSION_S_BLUR    0x3ac
#define UV_FUSION_M_BLUR    0x3cc
#define UV_STA_WEI          0x3e0
#define UV_STA_FULL_WEI     0x3e4
#define UV_STA_NEW_ADJ      0x3ec
#define UV_FALSE_COLOR      0x3f8

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
/* The function registers as SREG_CNTRL last loaded them */
#define F(off)              (s->fregs[(off) / 4])

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
        memcpy(s->fregs, s->regs, sizeof(s->fregs));
    }
    s->ref1 = (s->first & 1) ? false : !s->ref1;
    s->cur_first = s->first & 1;
    if (s->first & 1) {
        s->seq = 0;
        s->sta_wr = s->sta_rd = 0;
    } else if (++s->seq > 1) {
        s->seq = 2;
        s->sta_rd = s->sta_wr;
        s->sta_wr = s->sta_wr >= (F(NCU_STA_SLOTS) & 0x3f) ? 0 : s->sta_wr + 1;
    }
    s->busy = true;
    trace_ingenic_t30_ncu_frame(s->frames, s->ref1, fun);
    if (!s->loaded || s->ncu.stop) {
        s->hung = true;
        return false;
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

static void ncu_read_plane(uint32_t addr, uint32_t stride, uint8_t *dst,
                           uint32_t w, uint32_t h, uint32_t step)
{
    g_autofree uint8_t *row = g_malloc(w * step);

    for (uint32_t y = 0; y < h; y++) {
        address_space_read(&address_space_memory, addr + y * stride,
                           MEMTXATTRS_UNSPECIFIED, row, w * step);
        for (uint32_t x = 0; x < w; x++) {
            dst[y * w + x] = row[x * step];
        }
    }
}

static void ncu_write_plane(uint32_t addr, uint32_t stride,
                            const uint8_t *src, uint32_t len, uint32_t h)
{
    for (uint32_t y = 0; y < h; y++) {
        address_space_write(&address_space_memory, addr + y * stride,
                            MEMTXATTRS_UNSPECIFIED, src + y * len, len);
    }
}

/*
 * The fusion's box mean of k x k, the picture's edges repeated: the sum
 * times BOX_RECIP of the window, >> 14
 */
static const struct {
    int k;
    uint32_t recip;
} ncu_box_win[4] = { { 5, 656 }, { 7, 336 }, { 11, 136 }, { 15, 73 } };

static void ncu_box_sum(const uint16_t *src, uint32_t *dst, int w, int h,
                        int win)
{
    int rad = ncu_box_win[win].k / 2;
    g_autofree uint32_t *col = g_new0(uint32_t, w);

    /* column sums of rows -rad..rad, slid down a row at a time */
    for (int dy = -rad; dy <= rad; dy++) {
        const uint16_t *row = src + MIN(MAX(dy, 0), h - 1) * w;

        for (int x = 0; x < w; x++) {
            col[x] += row[x];
        }
    }
    for (int y = 0; y < h; y++) {
        uint32_t sum = 0;

        if (y) {
            const uint16_t *in = src + MIN(y + rad, h - 1) * w;
            const uint16_t *out = src + MAX(y - rad - 1, 0) * w;

            for (int x = 0; x < w; x++) {
                col[x] += in[x] - out[x];
            }
        }
        for (int dx = -rad; dx <= rad; dx++) {
            sum += col[MIN(MAX(dx, 0), w - 1)];
        }
        for (int x = 0; x < w; x++) {
            dst[y * w + x] = sum;
            sum += col[MIN(x + rad + 1, w - 1)] - col[MAX(x - rad, 0)];
        }
    }
}

static void ncu_box(const uint8_t *src, uint8_t *dst, int w, int h, int win)
{
    g_autofree uint16_t *v = g_new(uint16_t, w * h);
    g_autofree uint32_t *sum = g_new(uint32_t, w * h);

    for (int i = 0; i < w * h; i++) {
        v[i] = src[i];
    }
    ncu_box_sum(v, sum, w, h, win);
    for (int i = 0; i < w * h; i++) {
        dst[i] = (sum[i] * ncu_box_win[win].recip + 8192) >> 14;
    }
}

/*
 * A motion measure: the difference of twice the box means of cur and of
 * ref, each rounded on its own (Q14 reciprocal), over the window's size.
 * Not a function of the sum of cur - ref alone: a pixel on its threshold
 * goes either way with the level the two sit at.
 */
static void ncu_measure(const uint8_t *cur, const uint8_t *ref, uint16_t *m,
                        int w, int h, int win)
{
    g_autofree uint16_t *v = g_new(uint16_t, w * h);
    g_autofree uint32_t *sc = g_new(uint32_t, w * h);
    g_autofree uint32_t *sr = g_new(uint32_t, w * h);
    uint32_t recip = 2 * ncu_box_win[win].recip;

    for (int i = 0; i < w * h; i++) {
        v[i] = cur[i];
    }
    ncu_box_sum(v, sc, w, h, win);
    for (int i = 0; i < w * h; i++) {
        v[i] = ref[i];
    }
    ncu_box_sum(v, sr, w, h, win);
    for (int i = 0; i < w * h; i++) {
        m[i] = abs((int32_t)((sc[i] * recip + 8192) >> 14) -
                   (int32_t)((sr[i] * recip + 8192) >> 14));
    }
}

/* Within the corners' squares of len a side, as 0x208 and 0x370 set them */
static bool ncu_corner(int x, int y, int w, int h, int len)
{
    return (x < len || x >= w - len) && (y < len || y >= h - len);
}

/* Each column's block across, the last block's beyond, and its corner flag */
static void ncu_columns(int w, int bw, int nbx, int len, uint16_t *bx,
                        uint8_t *edge)
{
    for (int x = 0; x < w; x++) {
        bx[x] = MIN(x / bw, nbx - 1);
        edge[x] = x < len || x >= w - len;
    }
}

/* A table of the thresholds' cuts by age, ages 1 to 15, a byte each */
static void ncu_age_adj(IngenicT30NcuState *s, hwaddr off, uint8_t *adj)
{
    for (int i = 0; i < 15; i++) {
        adj[i] = extract32(F(off + i / 4 * 4), i % 4 * 8, 6);
    }
}

/*
 * Stage 2: each pixel the mean of the pixels of its 5 x 5 (edges repeated)
 * within x of it, itself included, times round(2048 / n) >> 11
 */
static void ncu_st2(const uint8_t *src, uint8_t *dst, int w, int h, int x0)
{
    /* the 2048 / n of each count, rounded */
    uint32_t recip[26];
    g_autofree uint8_t *pad = g_malloc(5 * (w + 4));
    g_autofree uint16_t *sum = g_new(uint16_t, w);
    g_autofree uint8_t *n = g_malloc(w);

    for (int i = 1; i <= 25; i++) {
        recip[i] = (4096 + i) / (2 * i);
    }
    for (int y = 0; y < h; y++) {
        const uint8_t *c = src + y * w;

        /* the 5 rows around, each with its ends repeated twice */
        for (int r = 0; r < 5; r++) {
            const uint8_t *row = src + MIN(MAX(y + r - 2, 0), h - 1) * w;
            uint8_t *p = pad + r * (w + 4);

            memcpy(p + 2, row, w);
            p[0] = p[1] = row[0];
            p[w + 2] = p[w + 3] = row[w - 1];
        }
        memset(sum, 0, w * sizeof(*sum));
        memset(n, 0, w);
        /*
         * A pass per tap over the row, in runs of 16 that -O2 vectorizes
         * (it leaves loops of unknown length alone)
         */
        for (int t = 0; t < 25; t++) {
            const uint8_t *p = pad + t / 5 * (w + 4) + t % 5;
            int x = 0;

            for (; x + 16 <= w; x += 16) {
                for (int k = 0; k < 16; k++) {
                    int d = p[x + k] - c[x + k];
                    uint16_t in = d < x0 && d > -x0;

                    sum[x + k] += in * p[x + k];
                    n[x + k] += in;
                }
            }
            for (; x < w; x++) {
                int d = p[x] - c[x];
                uint16_t in = d < x0 && d > -x0;

                sum[x] += in * p[x];
                n[x] += in;
            }
        }
        for (int x = 0; x < w; x++) {
            dst[y * w + x] = MIN(255, (sum[x] * recip[n[x]] + 1024) >> 11);
        }
    }
}

/*
 * The temporal blend with the fusion's box: the reference weighs y / 256
 * (each pixel's from ys if given), the box (w * (256 - y) + 128) >> 8 of
 * 256, rounded once, w being wlo where y is under judge, else whi
 */
static void ncu_blend(uint8_t *cur, const uint8_t *ref, const uint8_t *box,
                      int n, const uint16_t *ys, int y, int judge, int wlo,
                      int whi)
{
    for (int i = 0; i < n; i++) {
        int yy = ys ? ys[i] : y;
        int wq = box ? ((yy < judge ? wlo : whi) * (256 - yy) + 128) >> 8 : 0;
        int c = cur[i];
        int v = c + ((((box ? box[i] : c) - c) * wq + (ref[i] - c) * yy + 128)
                     >> 8);

        cur[i] = MIN(MAX(v, 0), 255);
    }
}

/* False colour: U and V both within t of 128 shrink to s / 4 of it */
static void ncu_false_colour(uint8_t *u, uint8_t *v, int n, int t, int sl)
{
    for (int i = 0; i < n; i++) {
        int du = u[i] - 128, dv = v[i] - 128;

        if (MAX(abs(du), abs(dv)) < t) {
            u[i] = 128 + (du < 0 ? -1 : 1) *
                   MIN(abs(du), (abs(du) * sl + 3) / 4);
            v[i] = 128 + (dv < 0 ? -1 : 1) *
                   MIN(abs(dv), (abs(dv) * sl + 3) / 4);
        }
    }
}

static void ncu_clamp(uint8_t *p, int n, int lo, int hi)
{
    for (int i = 0; i < n; i++) {
        p[i] = MIN(MAX(p[i], lo), hi);
    }
}

/* The statistics blocks: how many across and down, and their size */
static void ncu_sta_blocks(IngenicT30NcuState *s, int *nbx, int *nby, int *bw,
                           int *bh)
{
    *nbx = extract32(F(Y_STA_BLKS), 16, 8);
    *nby = extract32(F(Y_STA_BLKS), 0, 10);
    *bw = extract32(F(Y_STA_BLK_SIZE), 8, 4) + 16;
    *bh = extract32(F(Y_STA_BLK_SIZE), 0, 4) + 16;
}

/* Row r of a slot of the luma statistics ring, 0 its header */
static hwaddr ncu_sta_row(IngenicT30NcuState *s, unsigned slot, unsigned r)
{
    return F(Y_STA_ADDR) + slot * F(Y_STA_FRAME) +
           r * (F(Y_STA_STRIDE) & 0xffff);
}

/* The blocks' ages in the slot read: bits 4:0 of a byte each */
static void ncu_sta_read(IngenicT30NcuState *s, uint8_t *age, int nbx,
                         int nby)
{
    for (int by = 0; by < nby; by++) {
        address_space_read(&address_space_memory,
                           ncu_sta_row(s, s->sta_rd, by + 1),
                           MEMTXATTRS_UNSPECIFIED, age + by * nbx, nbx);
    }
    for (int i = 0; i < nbx * nby; i++) {
        age[i] &= 0x1f;
    }
}

/*
 * The ages a frame writes to its slot: 0 for a block where at least
 * Y_STA_MV_NUM pixels' measure (over the window of Y_STA_WIN, less
 * Y_SE_CORNER bits 29:24 in the corners) is above Y_STA_DIFF_THRES, less
 * the cut Y_SE_STA_ADJ gives the pixel's age read, else one more than the
 * age read (0 on the second frame), up to Y_STA_WEI bits 4:0. Unlike the
 * reader's, the last block across and down ends at the picture's edge.
 * Rows are written in 8-byte groups, the header first with the frame count
 * in its first byte.
 */
static void ncu_sta_write(IngenicT30NcuState *s, const uint16_t *m,
                          const uint8_t *age, int w, int h)
{
    int nbx, nby, bw, bh;
    int len, thr = extract32(F(Y_STA_DIFF_THRES), 0, 9);
    int oldest = extract32(F(Y_STA_WEI), 0, 5);
    int clen = extract32(F(Y_SE_CORNER), 0, 10);
    int cadj = extract32(F(Y_SE_CORNER), 24, 6);
    uint8_t adj[15];
    g_autofree uint8_t *in = g_malloc(w * h);
    g_autofree uint8_t *row = NULL;
    g_autofree uint16_t *bxs = g_new(uint16_t, w);
    g_autofree uint8_t *edge = g_malloc(w);

    ncu_sta_blocks(s, &nbx, &nby, &bw, &bh);
    ncu_age_adj(s, Y_SE_STA_ADJ, adj);
    len = ROUND_UP(nbx, 8);
    row = g_malloc0(len);
    ncu_columns(w, bw, nbx, clen, bxs, edge);
    for (int y = 0; y < h; y++) {
        const uint8_t *arow = age ? age + MIN(y / bh, nby - 1) * nbx : NULL;
        bool yedge = y < clen || y >= h - clen;

        for (int x = 0; x < w; x++) {
            int a = arow ? arow[bxs[x]] : 0;
            int v = m[y * w + x] - (yedge && edge[x] ? cadj : 0);

            in[y * w + x] = v > thr - (a && a < 16 ? adj[a - 1] : 0);
        }
    }
    row[0] = s->frames;
    address_space_write(&address_space_memory, ncu_sta_row(s, s->sta_wr, 0),
                        MEMTXATTRS_UNSPECIFIED, row, len);
    for (int by = 0; by < nby; by++) {
        int y0 = MAX(MIN(by * bh, h - bh), 0);

        memset(row, 0, len);
        for (int bx = 0; bx < nbx; bx++) {
            int x0 = MAX(MIN(bx * bw, w - bw), 0);
            uint32_t n = 0;

            for (int y = y0; y < MIN(y0 + bh, h); y++) {
                for (int x = x0; x < MIN(x0 + bw, w); x++) {
                    n += in[y * w + x];
                }
            }
            row[bx] = n >= F(Y_STA_MV_NUM) ? 0 :
                      MIN((age ? age[by * nbx + bx] : 0) + 1, oldest);
        }
        address_space_write(&address_space_memory,
                            ncu_sta_row(s, s->sta_wr, by + 1),
                            MEMTXATTRS_UNSPECIFIED, row, len);
    }
}

/*
 * Each pixel's reference weight from the third frame on. A pixel is static
 * where its SE measure (over the window of Y_SE_CON, less Y_SE_CORNER bits
 * 21:16 in the corners) is under Y_SE_SGM_X, less Y_STA_NEW_ADJ in a block
 * of age 0 and, for ages 1 to 15, the 6-bit byte from Y_SE_STA_ADJ on, and
 * Y_SE_STA_EN0 and EN1 allow it. A static
 * pixel's weight grows with its block's age a: by Y_STA_WEI bits 19:12 an
 * age from the curve's, or with bit 8 as round(256 (a + 1) / (a + 2)), at
 * most bits 28:20; from the oldest age but one it is Y_STA_FULL_WEI.
 * Moving pixels keep the curve's.
 */
static void ncu_y_weights(IngenicT30NcuState *s, const uint16_t *m,
                          const uint8_t *age, uint16_t *yw, int w, int h)
{
    uint32_t con = F(Y_STA_WEI), en = F(Y_SE_STA_EN0);
    int yc = extract32(F(Y_SE_SGM_Y0), 0, 9);
    int oldest = extract32(con, 0, 5);
    bool on = (en & 0xff) && (en & 0xff00) && (F(Y_SE_STA_EN1) & 0x7f);
    int clen = extract32(F(Y_SE_CORNER), 0, 10);
    int cadj = extract32(F(Y_SE_CORNER), 16, 6);
    int nbx, nby, bw, bh;
    uint8_t adj[15];
    g_autofree uint16_t *bxs = g_new(uint16_t, w);
    g_autofree uint8_t *edge = g_malloc(w);

    ncu_age_adj(s, Y_SE_STA_ADJ, adj);
    ncu_sta_blocks(s, &nbx, &nby, &bw, &bh);
    ncu_columns(w, bw, nbx, clen, bxs, edge);
    for (int y = 0; y < h; y++) {
        const uint8_t *arow = age + MIN(y / bh, nby - 1) * nbx;
        bool yedge = y < clen || y >= h - clen;

        for (int x = 0; x < w; x++) {
            int a = arow[bxs[x]];
            int thr = extract32(F(Y_SE_SGM_X), 0, 9) -
                      (a ? (a < 16 ? adj[a - 1] : 0) :
                       extract32(F(Y_STA_NEW_ADJ), 0, 9));
            int v = yc;

            if (yedge && edge[x]) {
                thr += cadj;
            }
            if (on && m[y * w + x] < thr) {
                if (a + 1 >= oldest) {
                    v = extract32(F(Y_STA_FULL_WEI), 0, 9);
                } else {
                    v = con & BIT(8) ?
                        (256 * (a + 1) + (a + 2) / 2) / (a + 2) :
                        yc + a * extract32(con, 12, 8);
                    v = MIN(v, extract32(con, 20, 9));
                }
            }
            yw[y * w + x] = MIN(v, 256);
        }
    }
}

/*
 * Each chroma sample's reference weight, U and V apart. The measure is over
 * 5 x 5 of the plane, less UV_SE_CORNER bits 21:16 in the corners. On the
 * second frame a sample takes UV_SE_SGM_Y0's low half under its plane's
 * UV_SE_SGM_X, else the high half. From the third on, a static sample (as
 * the luma's, with UV_STA_NEW_ADJ and the table at UV_SE_STA_ADJ) takes a
 * weight by the age of the luma's block it lies in: with UV_STA_WEI bit 0
 * round(256 (a + 1) / (a + 2)), else the low half plus bits 19:12 an age, at
 * most bits 28:20; from bits 8:4's age but one UV_STA_FULL_WEI's.
 */
static void ncu_uv_weights(IngenicT30NcuState *s, const uint8_t *cur,
                           const uint8_t *ref, const uint8_t *age,
                           uint16_t *yw, int w, int h, int pl)
{
    uint32_t con = F(UV_STA_WEI);
    int y0 = extract32(F(UV_SE_SGM_Y0), 0, 9);
    int y1 = extract32(F(UV_SE_SGM_Y0), 16, 9);
    int x0 = extract32(F(UV_SE_SGM_X), pl * 16, 9);
    int oldest = extract32(con, 4, 5);
    int clen = extract32(F(UV_SE_CORNER), 0, 10);
    int cadj = extract32(F(UV_SE_CORNER), 16, 6);
    int nbx, nby, bw, bh;
    uint8_t adj[15];
    g_autofree uint16_t *m = g_new(uint16_t, w * h);

    ncu_age_adj(s, UV_SE_STA_ADJ, adj);
    ncu_sta_blocks(s, &nbx, &nby, &bw, &bh);
    ncu_measure(cur, ref, m, w, h, 0);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int mm = m[y * w + x] - (ncu_corner(x, y, w, h, clen) ? cadj : 0);
            int v;

            if (!age) {
                v = mm < x0 ? y0 : y1;
            } else {
                int a = age[MIN(2 * y / bh, nby - 1) * nbx +
                            MIN(2 * x / bw, nbx - 1)];
                int thr = x0 - (a ? (a < 16 ? adj[a - 1] : 0) :
                                extract32(F(UV_STA_NEW_ADJ), 0, 9));

                v = y1;
                if (mm < thr) {
                    if (a + 1 >= oldest) {
                        v = extract32(F(UV_STA_FULL_WEI), 0, 9);
                    } else {
                        v = con & BIT(0) ?
                            (256 * (a + 1) + (a + 2) / 2) / (a + 2) :
                            y0 + a * extract32(con, 12, 8);
                        v = MIN(v, extract32(con, 20, 9));
                    }
                }
            }
            yw[y * w + x] = MIN(v, 256);
        }
    }
}

/* The settings libimp never writes, which the model does not follow */
static bool ncu_nr_unmodelled(IngenicT30NcuState *s)
{
    uint32_t st2_y = F(Y_ST2_SGM_P_Y) & 0x1f;

    for (hwaddr off = 0x180; off <= 0x1b0; off += 4) {
        if (F(off)) {
            return true;
        }
    }
    return extract32(F(Y_SE_SGM_Y0), 0, 9) !=
           extract32(F(Y_SE_SGM_Y0), 16, 9) ||
           (st2_y && (st2_y < 4 || st2_y > 16)) ||
           F(Y_ST2_SGM_P_S) || F(Y_ST2_GAUS) ||
           extract32(F(UV_FALSE_COLOR), 8, 7) ||
           extract32(F(Y_FUSION_AVE), 0, 9) > 256 ||
           extract32(F(Y_FUSION_AVE), 10, 9) > 256 ||
           F(UV_FUSION_S_BLUR) > 256 || F(UV_FUSION_M_BLUR) > 256 ||
           extract32(F(Y_SE_CON), 8, 2) < 2 ||
           (F(Y_SE_STA_EN0) != 0xffff && (F(Y_SE_STA_EN0) & 0xff) &&
            (F(Y_SE_STA_EN0) & 0xff00)) ||
           extract32(F(Y_STA_WEI), 20, 9) > 256 ||
           extract32(F(UV_STA_WEI), 20, 9) > 256 ||
           extract32(F(UV_SE_CON), 1, 1);
}

/* The 3D noise reduction of a frame from memory, NV12 */
static void ncu_nr(IngenicT30NcuState *s, uint8_t *yp, uint8_t *up,
                   uint8_t *vp, int w, int h, bool first)
{
    uint32_t fun = F(NCU_FUN_CNTRL), cut = F(NCU_CUT_VALUE);
    hwaddr ry = s->ref1 ? Y_REF0_ADDR : Y_REF1_ADDR;
    hwaddr ruv = s->ref1 ? UV_REF0_ADDR : UV_REF1_ADDR;
    int cw = w / 2, ch = h / 2;
    g_autofree uint8_t *ref = g_malloc(w * h);
    g_autofree uint8_t *box = g_malloc(w * h);
    g_autofree uint8_t *tmp = g_malloc(w * h);
    g_autofree uint8_t *age = NULL;
    int nbx, nby, bw, bh;

    ncu_sta_blocks(s, &nbx, &nby, &bw, &bh);
    if (!first && s->seq > 1 && nbx && nby) {
        age = g_malloc(nbx * nby);
        ncu_sta_read(s, age, nbx, nby);
    }
    if (fun & FUN_Y_TDNS && !first) {
        uint32_t ave = F(Y_FUSION_AVE);
        bool fuse = fun & FUN_Y_SDNS;
        g_autofree uint16_t *yw = NULL;
        g_autofree uint16_t *mse = NULL;
        g_autofree uint16_t *msta = NULL;

        ncu_read_plane(F(ry), F(ry + 4), ref, w, h, 1);
        if (age || (fun & FUN_Y_STA && nbx && nby)) {
            mse = g_new(uint16_t, w * h);
            ncu_measure(yp, ref, mse, w, h, F(Y_SE_CON) & 3);
        }
        if (age) {
            yw = g_new(uint16_t, w * h);
            ncu_y_weights(s, mse, age, yw, w, h);
        }
        if (fun & FUN_Y_STA && nbx && nby) {
            if ((F(Y_STA_WIN) & 3) != (F(Y_SE_CON) & 3)) {
                msta = g_new(uint16_t, w * h);
                ncu_measure(yp, ref, msta, w, h, F(Y_STA_WIN) & 3);
            }
            ncu_sta_write(s, msta ? msta : mse, age, w, h);
        }
        if (fuse) {
            ncu_box(yp, box, w, h, extract32(ave, 29, 2));
        }
        ncu_blend(yp, ref, fuse ? box : NULL, w * h, yw,
                  extract32(F(Y_SE_SGM_Y0), 0, 9), extract32(ave, 20, 9),
                  extract32(ave, 10, 9), extract32(ave, 0, 9));
    }
    if (fun & FUN_Y_SDNS && F(Y_ST2_SGM_P_Y) & 0x1f) {
        ncu_st2(yp, tmp, w, h, F(Y_ST2_SGM_P_X) & 0xff);
        memcpy(yp, tmp, w * h);
    }
    if (fun & FUN_UV_TDNS && !first) {
        bool fuse = F(UV_SE_CON) & 1;
        g_autofree uint16_t *yw = g_new(uint16_t, cw * ch);
        g_autofree uint8_t *both = g_malloc(cw * ch * 2);

        ncu_read_plane(F(ruv), F(ruv + 4), both, cw * 2, ch, 1);
        for (int pl = 0; pl < 2; pl++) {
            uint8_t *p = pl ? vp : up;

            for (int i = 0; i < cw * ch; i++) {
                ref[i] = both[i * 2 + pl];
            }
            ncu_uv_weights(s, p, ref, age, yw, cw, ch, pl);
            if (fuse) {
                ncu_box(p, box, cw, ch, 0);
            }
            ncu_blend(p, ref, fuse ? box : NULL, cw * ch, yw, 0,
                      F(UV_FUSION_MV_JUDGE) & 0x1ff,
                      F(UV_FUSION_M_BLUR) & 0x1ff, F(UV_FUSION_S_BLUR) & 0x1ff);
        }
    }
    if ((fun & (FUN_UV_SDNS | FUN_UV_FC)) == (FUN_UV_SDNS | FUN_UV_FC) &&
        F(UV_FALSE_COLOR) & BIT(15)) {
        ncu_false_colour(up, vp, cw * ch, F(UV_FALSE_COLOR) & 0xff,
                         extract32(F(UV_FALSE_COLOR), 16, 2));
    }
    ncu_clamp(yp, w * h, extract32(cut, 0, 8), extract32(cut, 8, 8));
    ncu_clamp(up, cw * ch, extract32(cut, 16, 8), extract32(cut, 24, 8));
    ncu_clamp(vp, cw * ch, extract32(cut, 16, 8), extract32(cut, 24, 8));
}

/* The noise reduction of an NV12 frame w x h, packed, in place */
static void ncu_process(IngenicT30NcuState *s, uint8_t *pic, int w, int h)
{
    int cw = w / 2, ch = h / 2;
    uint8_t *uv = pic + w * h;
    g_autofree uint8_t *up = g_malloc(cw * ch);
    g_autofree uint8_t *vp = g_malloc(cw * ch);

    if (ncu_nr_unmodelled(s) && F(NCU_FUN_CNTRL) != s->unimp_fun) {
        qemu_log_mask(LOG_UNIMP, "ingenic-t30-ncu: noise reduction settings "
                      "libimp never writes are taken as libimp's\n");
        s->unimp_fun = F(NCU_FUN_CNTRL);
    }
    for (int i = 0; i < cw * ch; i++) {
        up[i] = uv[i * 2];
        vp[i] = uv[i * 2 + 1];
    }
    ncu_nr(s, pic, up, vp, w, h, s->cur_first);
    for (int i = 0; i < cw * ch; i++) {
        uv[i * 2] = up[i];
        uv[i * 2 + 1] = vp[i];
    }
}

/* The frame in hand to the reference it goes to, REF0 or REF1 */
static void ncu_write_ref(IngenicT30NcuState *s, const uint8_t *pic, int w,
                          int h)
{
    hwaddr oy = s->ref1 ? Y_REF1_ADDR : Y_REF0_ADDR;
    hwaddr ouv = s->ref1 ? UV_REF1_ADDR : UV_REF0_ADDR;

    ncu_write_plane(F(oy), F(oy + 4), pic, w, h);
    ncu_write_plane(F(ouv), F(ouv + 4), pic + w * h, w, h / 2);
}

/* A frame from memory: to REF0 or REF1, or on to the multi-scaler */
static void ncu_run(void *opaque)
{
    IngenicT30NcuState *s = opaque;
    int w = extract32(F(NCU_FRAME_SIZE), 0, 12) & ~1;
    int h = extract32(F(NCU_FRAME_SIZE), 16, 12) & ~1;

    if (!s->busy || s->hung) {
        return;
    }
    if (F(NCU_FUN_CNTRL) & FUN_OUT_DDR && w && h) {
        g_autofree uint8_t *pic = g_malloc(w * h * 3 / 2);

        ncu_read_plane(F(Y_CUR_ADDR), F(Y_CUR_STRIDE), pic, w, h, 1);
        ncu_read_plane(F(UV_CUR_ADDR), F(UV_CUR_STRIDE), pic + w * h, w,
                       h / 2, 1);
        if (F(NCU_FUN_CNTRL) & FUN_NR) {
            ncu_process(s, pic, w, h);
        }
        ncu_write_ref(s, pic, w, h);
    }
    ncu_end(s);
    if (!(F(NCU_FUN_CNTRL) & FUN_OUT_DDR)) {
        qemu_irq_pulse(s->frame_out);
    }
}

const uint8_t *ingenic_t30_ncu_lfb_frame(IngenicT30NcuState *s,
                                         ImageSensorState *sensor,
                                         uint32_t frame, uint32_t *pw,
                                         uint32_t *ph)
{
    uint32_t fun = F(NCU_FUN_CNTRL);
    int w = extract32(F(NCU_FRAME_SIZE), 0, 12) & ~1;
    int h = extract32(F(NCU_FRAME_SIZE), 16, 12) & ~1;

    if (!(fun & FUN_NR) || fun & FUN_IN_DDR || !w || !h || s->hung) {
        return NULL;
    }
    if (!s->lfb || s->lfb_frame != frame || s->lfb_w != w || s->lfb_h != h) {
        g_free(s->lfb);
        s->lfb = g_malloc(w * h * 3 / 2);
        image_sensor_render(sensor, frame, w, h, s->lfb);
        ncu_process(s, s->lfb, w, h);
        ncu_write_ref(s, s->lfb, w, h);
        s->lfb_frame = frame;
        s->lfb_w = w;
        s->lfb_h = h;
    }
    *pw = w;
    *ph = h;
    return s->lfb;
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
        v = s->clk_view ? s->clk :
            (s->ref1 ? BIT(31) : BIT(30)) | s->sta_wr << 8 | s->sta_rd;
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
            memcpy(s->fregs, s->regs, sizeof(s->fregs));
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
    memcpy(s->fregs, ncu_reset, sizeof(s->fregs));
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
    s->seq = 0;
    s->sta_wr = s->sta_rd = 0;
    g_clear_pointer(&s->lfb, g_free);
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
    .version_id = 2,
    .minimum_version_id = 2,
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
        VMSTATE_UINT32_ARRAY(fregs, IngenicT30NcuState, NCU_REGS),
        VMSTATE_UINT8(seq, IngenicT30NcuState),
        VMSTATE_BOOL(cur_first, IngenicT30NcuState),
        VMSTATE_UINT8(sta_wr, IngenicT30NcuState),
        VMSTATE_UINT8(sta_rd, IngenicT30NcuState),
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
