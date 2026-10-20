/*
 * Ingenic ISP (Tiziano) and VIC: the frame path
 *
 * The tx-isp drivers program some thousands of ISP registers, and those
 * read back as written. What they wait on is modelled:
 *
 * - VIC: a command written to VIC_CTRL completes at once and the register
 *   reads 0; 1 starts the video input and 2 resets it. Once started it
 *   takes frames from the image sensor at the sensor's rate. The T31 has
 *   one VIC, at +0xe0000; the T40 three, at +0x80000 + N * 0x10000, each
 *   with its own interrupt; the T41 the first of those.
 *
 * - The multi-scaler (MSCA) puts each frame out on the channels enabled
 *   in its channel enable register, as NV12 scaled to +0x0 and cropped to
 *   +0x28 (x << 16 | y) and +0x2c (w << 16 | h). The T31 has one MSCA,
 *   its channels at 0x9900 + N * 0x100; the T40 one for each of its two
 *   pipelines, at 0x16100 and 0x17100; the T41 one, at 0xf0100. The
 *   driver queues a buffer by writing its Y and UV addresses to +0x6c and
 *   +0x84; a frame fills the oldest and moves it to the done FIFO, which
 *   the driver drains while +0x7c bit 0 (empty) is clear (bits 11:8 count
 *   the entries): +0x74 gives the Y address and pops the entry, +0x8c and
 *   +0x90 its UV address and frame number. With no buffer queued the
 *   channel drops the frame, and it does while the last is undrained: the
 *   T40 driver reports only the last entry it drains, and a guest can fall
 *   a frame behind here where silicon would not.
 *
 * - The T23's MSCA, at 0xd100 + N * 0x100 (channel enable 0xd040), packs
 *   the same registers: scaled size +0x4, crop +0x8 and +0xc, Y push +0x30,
 *   done +0x38, empty +0x3c and stride +0x40, UV push +0x50, done +0x58
 *   and stride +0x60, FIFO clears +0x44, +0x48, +0x64 and +0x68. It has no
 *   frame number, and its driver reads the UV address before the Y read
 *   pops the entry.
 *
 * - The T32's MSCA is the T23's at 0xf0000: channel enable 0xf0040, busy
 *   0xf0054, channels at 0xf0100 + N * 0x100. Its driver writes 0 to the
 *   push FIFO clears after each push, so only bit 0 clears there. The
 *   channels' done bits have a word of their own, 0x30038 (enable 0x30030,
 *   clear 0x30034), beside the frame done at 0x30028 bit 0. Its VIC, at
 *   +0x80000, runs while +0x10 bit 0 is set (4 resets it), and the driver
 *   waits for 0x24 bits 3:0 (a reset) and 0x22014 bits 17:16 (its register
 *   DMA) to clear, which they do at once.
 *
 * - The T21's MSCA has its channels at 0x2400 + N * 0x100 (enable 0x2304,
 *   busy 0x2308) with the T31's registers but for the frame number, at
 *   +0x78. Its ISP interrupt registers are at 0x80 (enable), 0x84 (status)
 *   and 0x88 (clear), with no frame start or end bit, and its VIC at
 *   +0x10000 has one interrupt status, mask and clear, at 0x100, 0x104 and
 *   0x108, where the others have two, at 0x1e0 to 0x1f4.
 *
 * - The T30's MSCA is a block of its own, 4 KiB at 0x130b0000 on INTC source
 *   19, behind the Apical ISP and the noise reducer: channel enable 0x4,
 *   busy 0x8, interrupt status 0xc, mask 0x10 (1 masks) and clear 0x14,
 *   channels at 0x100 + N * 0x100 as the T21's, with the Y push FIFO's
 *   state at +0x70 (bit 0 empty, bit 1 full, 4:2 the count). It takes a
 *   frame when its "frame" input pulses.
 *
 * - Interrupts: a pipeline's status bit N is channel N's frame done, and
 *   the frame start (and on the T31 the frame end) has a bit too; the
 *   enable register gates the ISP interrupt and the clear register clears.
 *   A VIC's status (bit 0: frame done, bit 4 on the T40: frame start) is
 *   masked by VIC_INT_MSK and cleared through VIC_INT_CLR.
 *
 * - The T32, T40 and T41 drivers request the ISP idle through 0x200 bit 0
 *   before they reconfigure, and spin until 0x204 bit 0 says so; frames
 *   take no time here, so the ISP is always idle. The MSCA registers the
 *   T40 driver writes through the shadow port (0x50324: a value, then the
 *   register's offset) take effect at once.
 *
 * The one image sensor feeds the first pipeline.
 *
 * The layouts are the T21, T23, T31, T40 and T41 tx-isp drivers' (their
 * decompilations) and the T32's (its module's code), the T41's checked
 * against a streaming T41NQ.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-isp.h"
#include "hw/sensor/image-sensor.h"

/* A channel's registers, offsets within its 0x100 */
typedef struct IngenicIspChanRegs {
    uint32_t size;                  /* width << 16 | height */
    uint32_t crop_pos, crop_size;
    uint32_t y_push, done_y, done_sta, y_stride;
    uint32_t uv_push, done_uv, done_frame, uv_stride;
    uint32_t clr_y, clr_uv, clr_done_y, clr_done_uv;   /* write 1 */
    uint32_t push_sta;              /* the Y push FIFO's empty, full, count */
} IngenicIspChanRegs;

#define CH_NONE             0x100   /* a register the channel lacks */

static const IngenicIspChanRegs ingenic_isp_chan_regs = {
    .size = 0x00, .crop_pos = 0x28, .crop_size = 0x2c,
    .y_push = 0x6c, .done_y = 0x74, .done_sta = 0x7c, .y_stride = 0x80,
    .uv_push = 0x84, .done_uv = 0x8c, .done_frame = 0x90, .uv_stride = 0x98,
    .clr_y = 0x9c, .clr_uv = 0xa0, .clr_done_y = 0xa4, .clr_done_uv = 0xa8,
    .push_sta = CH_NONE,
};

static const IngenicIspChanRegs ingenic_t21_isp_chan_regs = {
    .size = 0x00, .crop_pos = 0x28, .crop_size = 0x2c,
    .y_push = 0x6c, .done_y = 0x74, .done_sta = 0x7c, .y_stride = 0x80,
    .uv_push = 0x84, .done_uv = 0x8c, .done_frame = 0x78, .uv_stride = 0x98,
    .clr_y = 0x9c, .clr_uv = 0xa0, .clr_done_y = 0xa4, .clr_done_uv = 0xa8,
    .push_sta = CH_NONE,
};

static const IngenicIspChanRegs ingenic_t30_mscaler_chan_regs = {
    .size = 0x00, .crop_pos = 0x28, .crop_size = 0x2c,
    .y_push = 0x6c, .done_y = 0x74, .done_sta = 0x7c, .y_stride = 0x80,
    .uv_push = 0x84, .done_uv = 0x8c, .done_frame = 0x78, .uv_stride = 0x98,
    .clr_y = 0x9c, .clr_uv = 0xa0, .clr_done_y = 0xa4, .clr_done_uv = 0xa8,
    .push_sta = 0x70,
};

static const IngenicIspChanRegs ingenic_t23_isp_chan_regs = {
    .size = 0x04, .crop_pos = 0x08, .crop_size = 0x0c,
    .y_push = 0x30, .done_y = 0x38, .done_sta = 0x3c, .y_stride = 0x40,
    .uv_push = 0x50, .done_uv = 0x58, .done_frame = CH_NONE, .uv_stride = 0x60,
    .clr_y = 0x44, .clr_uv = 0x64, .clr_done_y = 0x48, .clr_done_uv = 0x68,
    .push_sta = CH_NONE,
};

#define VIC_START           BIT(0)
#define VIC_RESET           BIT(1)
#define VIC_INT_FRAME_DONE  BIT(0)

/*
 * A VIC's interrupt status, mask and clear: on the T21 one set at 0x100,
 * elsewhere two at 0x1e0, the second 4 above the first
 */
typedef struct IngenicIspVicInt {
    hwaddr sta, msk, clr;
    bool pair;
} IngenicIspVicInt;

static const IngenicIspVicInt ingenic_isp_vic_int = {
    .sta = 0x1e0, .msk = 0x1e8, .clr = 0x1f0, .pair = true,
};

typedef struct IngenicIspPipe {
    hwaddr int_en, int_sta, int_clr;
    hwaddr done_en, done_sta, done_clr; /* channel done, if not in int_sta */
    hwaddr ch_en, ch;
} IngenicIspPipe;

/* A register whose busy bits read back clear once written */
typedef struct IngenicIspBusy {
    hwaddr off;
    uint32_t bits;
} IngenicIspBusy;

/* Offsets of 0 are registers a layout lacks */
struct IngenicIspLayout {
    const char *desc;
    hwaddr iosize;                  /* 0 for INGENIC_ISP_IOSIZE */
    unsigned pipes;
    IngenicIspPipe pipe[INGENIC_ISP_PIPES];
    bool int_mask;                  /* int_en masks: 1 off */
    uint32_t int_frame_start, int_frame_end;
    hwaddr ch_busy;                 /* reads 0 */
    unsigned vics;
    hwaddr vic, vic_stride;
    hwaddr vic_ctrl;                /* VIC_CTRL, or this */
    bool vic_ctrl_level;            /* the VIC runs while its bit 0 is set */
    const IngenicIspVicInt *vic_int;
    uint32_t vic_frame_start;
    hwaddr idle;                    /* bit 0 reads 1 */
    IngenicIspBusy busy[2];
    hwaddr shadow;
    const IngenicIspChanRegs *chan;
    bool done_uv_head;              /* the UV done shows the head entry */
    bool clr_bit0;                  /* a FIFO clear needs bit 0 */
};

static const IngenicIspLayout ingenic_t31_isp_layout = {
    .desc = "Ingenic T31 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x0b0, .int_sta = 0x0b4, .int_clr = 0x0b8,
          .ch_en = 0x9804, .ch = 0x9900 },
    },
    .int_frame_start = BIT(12),
    .int_frame_end = BIT(13),
    .ch_busy = 0x9808,
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .chan = &ingenic_isp_chan_regs,
};

static const IngenicIspLayout ingenic_t21_isp_layout = {
    .desc = "Ingenic T21 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x080, .int_sta = 0x084, .int_clr = 0x088,
          .ch_en = 0x2304, .ch = 0x2400 },
    },
    .ch_busy = 0x2308,
    .vics = 1,
    .vic = 0x10000,
    .vic_stride = 0x10000,
    .vic_int = &(const IngenicIspVicInt) {
        .sta = 0x100, .msk = 0x104, .clr = 0x108,
    },
    .chan = &ingenic_t21_isp_chan_regs,
};

/* The T30's multi-scaler, apart from its ISP; frames come from its "frame" */
static const IngenicIspLayout ingenic_t30_mscaler_layout = {
    .desc = "Ingenic T30 multi-scaler",
    .iosize = 0x1000,
    .pipes = 1,
    .pipe = {
        { .int_en = 0x010, .int_sta = 0x00c, .int_clr = 0x014,
          .ch_en = 0x004, .ch = 0x100 },
    },
    .int_mask = true,
    .ch_busy = 0x008,
    .chan = &ingenic_t30_mscaler_chan_regs,
};

static const IngenicIspLayout ingenic_t23_isp_layout = {
    .desc = "Ingenic T23 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x0b0, .int_sta = 0x0b4, .int_clr = 0x0b8,
          .ch_en = 0xd040, .ch = 0xd100 },
    },
    .int_frame_start = BIT(12),
    .int_frame_end = BIT(13),
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .chan = &ingenic_t23_isp_chan_regs,
    .done_uv_head = true,
};

static const IngenicIspLayout ingenic_t32_isp_layout = {
    .desc = "Ingenic T32 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x30020, .int_sta = 0x30028, .int_clr = 0x30024,
          .done_en = 0x30030, .done_sta = 0x30038, .done_clr = 0x30034,
          .ch_en = 0xf0040, .ch = 0xf0100 },
    },
    .int_frame_end = BIT(0),
    .ch_busy = 0xf0054,
    .vics = 1,                      /* the second sensor's, at 0x81000, idles */
    .vic = 0x80000,
    .vic_stride = 0x1000,
    .vic_ctrl = 0x010,
    .vic_ctrl_level = true,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { 0x22014, 0x30000 } },
    .chan = &ingenic_t23_isp_chan_regs,
    .clr_bit0 = true,
};

static const IngenicIspLayout ingenic_t40_isp_layout = {
    .desc = "Ingenic T40 ISP and VICs",
    .pipes = 2,
    .pipe = {
        { .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
          .ch_en = 0x16008, .ch = 0x16100 },
        { .int_en = 0x40030, .int_sta = 0x40038, .int_clr = 0x40034,
          .ch_en = 0x17008, .ch = 0x17100 },
    },
    .int_frame_start = BIT(13),
    .vics = 3,
    .vic = 0x80000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .shadow = 0x50324,
    .chan = &ingenic_isp_chan_regs,
};

static const IngenicIspLayout ingenic_t41_isp_layout = {
    .desc = "Ingenic T41 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
          .ch_en = 0xf0008, .ch = 0xf0100 },
    },
    .int_frame_start = BIT(13),
    .vics = 1,
    .vic = 0x80000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .chan = &ingenic_isp_chan_regs,
};

#define R(off)              (s->regs[(off) / 4])

static const IngenicIspLayout *isp_layout(IngenicIspState *s)
{
    return INGENIC_ISP_GET_CLASS(s)->layout;
}

static IngenicIspChan *isp_chan(IngenicIspState *s, hwaddr off)
{
    const IngenicIspLayout *l = isp_layout(s);

    for (unsigned p = 0; p < l->pipes; p++) {
        hwaddr ch = l->pipe[p].ch;

        if (off >= ch && off < ch + INGENIC_ISP_CHANS * 0x100) {
            return &s->chan[p][(off - ch) / 0x100];
        }
    }
    return NULL;
}

/* The VIC an offset falls in, or -1; *reg is the offset within it */
static int isp_vic(IngenicIspState *s, hwaddr off, hwaddr *reg)
{
    const IngenicIspLayout *l = isp_layout(s);

    if (off < l->vic || off >= l->vic + l->vics * l->vic_stride) {
        return -1;
    }
    *reg = (off - l->vic) % l->vic_stride;
    return (off - l->vic) / l->vic_stride;
}

#define VIC(n, reg)         R(l->vic + (n) * l->vic_stride + (reg))

static void isp_push(uint32_t *fifo, uint32_t rd, uint32_t *n, uint32_t v)
{
    if (*n < INGENIC_ISP_FIFO) {
        fifo[(rd + (*n)++) % INGENIC_ISP_FIFO] = v;
    }
}

static uint32_t isp_pop(uint32_t *fifo, uint32_t *rd, uint32_t *n)
{
    uint32_t v;

    if (!*n) {
        return 0;
    }
    v = fifo[*rd];
    *rd = (*rd + 1) % INGENIC_ISP_FIFO;
    (*n)--;
    return v;
}

static void isp_update(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);
    bool isp = false;

    for (unsigned p = 0; p < l->pipes; p++) {
        const IngenicIspPipe *pipe = &l->pipe[p];
        uint32_t en = R(pipe->int_en);

        isp |= R(pipe->int_sta) & (l->int_mask ? ~en : en);
        if (pipe->done_sta) {
            en = R(pipe->done_en);
            isp |= R(pipe->done_sta) & (l->int_mask ? ~en : en);
        }
    }
    qemu_set_irq(s->isp_irq, isp);
    for (unsigned n = 0; n < l->vics; n++) {
        const IngenicIspVicInt *vi = l->vic_int;
        uint32_t pending = VIC(n, vi->sta) & ~VIC(n, vi->msk);

        if (vi->pair) {
            pending |= VIC(n, vi->sta + 4) & ~VIC(n, vi->msk + 4);
        }
        qemu_set_irq(s->vic_irq[n], !!pending);
    }
}

/* The w x h window at (x, y) of a frame src_w wide, out to a plane */
static void isp_dma_plane(hwaddr addr, const uint8_t *src, uint32_t src_w,
                          uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                          uint32_t stride)
{
    src += y * src_w + x;
    if (stride <= w && src_w == w) {
        address_space_write(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, src, w * h);
        return;
    }
    stride = MAX(stride, w);
    for (uint32_t i = 0; i < h; i++) {
        address_space_write(&address_space_memory, addr + i * stride,
                            MEMTXATTRS_UNSPECIFIED, src + i * src_w, w);
    }
}

static void isp_chan_frame(IngenicIspState *s, ImageSensorState *sensor,
                           unsigned p, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspChanRegs *r = l->chan;
    hwaddr base = l->pipe[p].ch + n * 0x100;
    IngenicIspChan *ch = &s->chan[p][n];
    uint32_t size = R(base + r->size), crop = R(base + r->crop_size);
    uint32_t pos = R(base + r->crop_pos);
    uint32_t w = size >> 16, h = size & 0xffff;
    uint32_t cx = pos >> 16, cy = pos & 0xffff;
    uint32_t cw = crop >> 16, ch_h = crop & 0xffff;
    g_autofree uint8_t *nv12 = NULL;
    uint32_t y, uv, i;

    /* NV12 needs even sizes and positions */
    w &= ~1;
    h &= ~1;
    if (!crop || cx + cw > w || cy + ch_h > h) {
        cx = cy = 0;
        cw = w;
        ch_h = h;
    }
    cx &= ~1;
    cy &= ~1;
    cw &= ~1;
    ch_h &= ~1;
    if (!ch->y_n || !ch->uv_n || !cw || !ch_h || ch->done_n) {
        return;
    }
    y = isp_pop(ch->y, &ch->y_rd, &ch->y_n);
    uv = isp_pop(ch->uv, &ch->uv_rd, &ch->uv_n);
    nv12 = g_malloc(w * h * 3 / 2);
    image_sensor_render(sensor, s->frame, w, h, nv12);
    isp_dma_plane(y, nv12, w, cx, cy, cw, ch_h, R(base + r->y_stride));
    isp_dma_plane(uv, nv12 + w * h, w, cx, cy / 2, cw, ch_h / 2,
                  R(base + r->uv_stride));
    i = (ch->done_rd + ch->done_n++) % INGENIC_ISP_FIFO;
    ch->done_y[i] = y;
    ch->done_uv[i] = uv;
    ch->done_frame[i] = s->frame;
    R(l->pipe[p].done_sta ?: l->pipe[p].int_sta) |= BIT(n);
}

static void isp_frame(void *opaque)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspPipe *pipe = &l->pipe[0];
    ImageSensorState *sensor = image_sensor_find();

    if (!s->vic_running || !sensor) {
        return;
    }
    s->frame++;
    R(pipe->int_sta) |= l->int_frame_start;
    for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
        if (R(pipe->ch_en) & BIT(n)) {
            isp_chan_frame(s, sensor, 0, n);
        }
    }
    R(pipe->int_sta) |= l->int_frame_end;
    for (unsigned n = 0; n < l->vics; n++) {
        if (s->vic_running & BIT(n)) {
            VIC(n, l->vic_int->sta) |= l->vic_frame_start |
                                       VIC_INT_FRAME_DONE;
        }
    }
    isp_update(s);
    timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              NANOSECONDS_PER_SECOND / sensor->fps);
}

static void isp_vic_stop(IngenicIspState *s, int vic)
{
    s->vic_running &= ~BIT(vic);
    if (!s->vic_running) {
        timer_del(s->frame_timer);
    }
}

static void isp_vic_start(IngenicIspState *s, int vic)
{
    bool idle = !s->vic_running;

    if (s->vic_running & BIT(vic)) {
        return;
    }
    s->vic_running |= BIT(vic);
    if (idle) {
        isp_frame(s);
    }
}

/* A layout without a VIC: a frame from elsewhere, out to the channels */
static void isp_frame_in(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    ImageSensorState *sensor = image_sensor_find();

    if (!level || !sensor) {
        return;
    }
    s->frame++;
    for (unsigned c = 0; c < INGENIC_ISP_CHANS; c++) {
        if (R(l->pipe[0].ch_en) & BIT(c)) {
            isp_chan_frame(s, sensor, 0, c);
        }
    }
    isp_update(s);
}

static uint64_t ingenic_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspState *s = INGENIC_ISP(opaque);
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspChan *ch = isp_chan(s, off);
    hwaddr reg;

    if (ch) {
        const IngenicIspChanRegs *r = l->chan;
        uint32_t i = ch->done_rd, cr = off & 0xff;

        if (cr == r->done_y) {
            if (!ch->done_n) {
                return 0;
            }
            ch->done_rd = (i + 1) % INGENIC_ISP_FIFO;
            ch->done_n--;
            ch->popped_uv = ch->done_uv[i];
            ch->popped_frame = ch->done_frame[i];
            return ch->done_y[i];
        }
        if (cr == r->done_sta) {
            return ch->done_n << 8 | !ch->done_n;
        }
        if (cr == r->done_uv) {
            if (l->done_uv_head) {
                return ch->done_n ? ch->done_uv[i] : 0;
            }
            return ch->popped_uv;
        }
        if (cr == r->done_frame) {
            return ch->popped_frame;
        }
        if (cr == r->push_sta) {
            return (ch->y_n ? 0 : 1) | (ch->y_n >= INGENIC_ISP_FIFO) << 1 |
                   MIN(ch->y_n, 7) << 2;
        }
    }
    if (isp_vic(s, off, &reg) >= 0 && reg == l->vic_ctrl &&
        !l->vic_ctrl_level) {
        return 0;
    }
    if (l->ch_busy && off == l->ch_busy) {
        return 0;
    }
    if (l->idle && off == l->idle) {
        return R(off) | 1;
    }
    return R(off);
}

static void ingenic_isp_write(void *opaque, hwaddr off, uint64_t value,
                              unsigned size)
{
    IngenicIspState *s = INGENIC_ISP(opaque);
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspChan *ch = isp_chan(s, off);
    uint32_t v = value;
    hwaddr reg;
    int vic;

    if (ch) {
        const IngenicIspChanRegs *r = l->chan;
        uint32_t cr = off & 0xff;

        if (cr == r->y_push) {
            isp_push(ch->y, ch->y_rd, &ch->y_n, v);
            return;
        }
        if (cr == r->uv_push) {
            isp_push(ch->uv, ch->uv_rd, &ch->uv_n, v);
            return;
        }
        if (cr == r->clr_y || cr == r->clr_uv || cr == r->clr_done_y ||
            cr == r->clr_done_uv) {
            if (l->clr_bit0 && !(v & 1)) {
                return;
            }
            if (cr == r->clr_y) {
                ch->y_n = 0;
            } else if (cr == r->clr_uv) {
                ch->uv_n = 0;
            } else {
                ch->done_n = 0;
            }
            return;
        }
        if (cr == r->done_y || cr == r->done_sta || cr == r->done_uv ||
            cr == r->done_frame) {
            return;
        }
    }

    vic = isp_vic(s, off, &reg);
    if (vic >= 0) {
        const IngenicIspVicInt *vi = l->vic_int;

        for (unsigned i = 0; i <= vi->pair; i++) {
            if (reg == vi->sta + 4 * i) {
                return;
            }
            if (reg == vi->clr + 4 * i) {
                VIC(vic, vi->sta + 4 * i) &= ~v;
                isp_update(s);
                return;
            }
        }
    }
    if (vic >= 0 && reg == l->vic_ctrl) {
        if (l->vic_ctrl_level) {
            R(off) = v;
        }
        if (v & VIC_RESET || (l->vic_ctrl_level && !(v & VIC_START))) {
            isp_vic_stop(s, vic);
        }
        if (v & VIC_START) {
            isp_vic_start(s, vic);
        }
        return;
    }

    if (l->shadow && off == l->shadow) {
        if (!s->shadow_pending) {
            s->shadow_value = v;
        } else if (v != l->shadow && v < INGENIC_ISP_IOSIZE) {
            s->shadow_pending = false;
            ingenic_isp_write(s, v & ~3, s->shadow_value, 4);
            return;
        }
        s->shadow_pending = !s->shadow_pending;
        return;
    }
    for (unsigned p = 0; p < l->pipes; p++) {
        if (off == l->pipe[p].int_sta) {
            return;
        }
        if (off == l->pipe[p].int_clr) {
            R(l->pipe[p].int_sta) &= ~v;
            isp_update(s);
            return;
        }
        if (l->pipe[p].done_sta && off == l->pipe[p].done_sta) {
            return;
        }
        if (l->pipe[p].done_clr && off == l->pipe[p].done_clr) {
            R(l->pipe[p].done_sta) &= ~v;
            isp_update(s);
            return;
        }
    }
    R(off) = v;
    for (unsigned i = 0; i < ARRAY_SIZE(l->busy); i++) {
        if (l->busy[i].bits && off == l->busy[i].off) {
            R(off) &= ~l->busy[i].bits;
        }
    }
    isp_update(s);
}

static const MemoryRegionOps ingenic_isp_ops = {
    .read = ingenic_isp_read,
    .write = ingenic_isp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl  = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_isp_reset_hold(Object *obj, ResetType type)
{
    IngenicIspState *s = INGENIC_ISP(obj);

    timer_del(s->frame_timer);
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->chan, 0, sizeof(s->chan));
    s->frame = 0;
    s->vic_running = 0;
    s->shadow_value = 0;
    s->shadow_pending = false;
}

static void ingenic_isp_reset_exit(Object *obj, ResetType type)
{
    isp_update(INGENIC_ISP(obj));
}

static void ingenic_isp_init(Object *obj)
{
    IngenicIspState *s = INGENIC_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    const IngenicIspLayout *l = isp_layout(s);

    memory_region_init_io(&s->iomem, obj, &ingenic_isp_ops, s,
                          object_get_typename(obj),
                          l->iosize ?: INGENIC_ISP_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->isp_irq);
    for (unsigned n = 0; n < l->vics; n++) {
        sysbus_init_irq(sbd, &s->vic_irq[n]);
    }
    s->frame_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, isp_frame, s);
    if (!l->vics) {
        qdev_init_gpio_in_named(DEVICE(obj), isp_frame_in, "frame", 1);
    }
}

static const VMStateDescription vmstate_ingenic_isp_chan = {
    .name = TYPE_INGENIC_ISP "/chan",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(y, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(uv, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32(y_rd, IngenicIspChan),
        VMSTATE_UINT32(y_n, IngenicIspChan),
        VMSTATE_UINT32(uv_rd, IngenicIspChan),
        VMSTATE_UINT32(uv_n, IngenicIspChan),
        VMSTATE_UINT32_ARRAY(done_y, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(done_uv, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32_ARRAY(done_frame, IngenicIspChan, INGENIC_ISP_FIFO),
        VMSTATE_UINT32(done_rd, IngenicIspChan),
        VMSTATE_UINT32(done_n, IngenicIspChan),
        VMSTATE_UINT32(popped_uv, IngenicIspChan),
        VMSTATE_UINT32(popped_frame, IngenicIspChan),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_isp = {
    .name = TYPE_INGENIC_ISP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIspState, INGENIC_ISP_IOSIZE / 4),
        VMSTATE_STRUCT_2DARRAY(chan, IngenicIspState, INGENIC_ISP_PIPES,
                               INGENIC_ISP_CHANS, 1, vmstate_ingenic_isp_chan,
                               IngenicIspChan),
        VMSTATE_UINT32(frame, IngenicIspState),
        VMSTATE_UINT32(vic_running, IngenicIspState),
        VMSTATE_UINT32(shadow_value, IngenicIspState),
        VMSTATE_BOOL(shadow_pending, IngenicIspState),
        VMSTATE_TIMER_PTR(frame_timer, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_isp;
    rc->phases.hold = ingenic_isp_reset_hold;
    rc->phases.exit = ingenic_isp_reset_exit;
}

static void ingenic_isp_soc_class_init(ObjectClass *oc, const void *data)
{
    const IngenicIspLayout *l = data;

    INGENIC_ISP_CLASS(oc)->layout = l;
    DEVICE_CLASS(oc)->desc = l->desc;
}

static const TypeInfo ingenic_isp_types[] = {
    {
        .name = TYPE_INGENIC_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicIspState),
        .instance_init = ingenic_isp_init,
        .class_size = sizeof(IngenicIspClass),
        .class_init = ingenic_isp_class_init,
        .abstract = true,
    }, {
        .name = TYPE_INGENIC_T21_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t21_isp_layout,
    }, {
        .name = TYPE_INGENIC_T30_MSCALER,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t30_mscaler_layout,
    }, {
        .name = TYPE_INGENIC_T23_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t23_isp_layout,
    }, {
        .name = TYPE_INGENIC_T31_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t31_isp_layout,
    }, {
        .name = TYPE_INGENIC_T32_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t32_isp_layout,
    }, {
        .name = TYPE_INGENIC_T40_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t40_isp_layout,
    }, {
        .name = TYPE_INGENIC_T41_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t41_isp_layout,
    },
};

DEFINE_TYPES(ingenic_isp_types)
