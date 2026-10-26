/*
 * Ingenic IVDC, the ISP to encoder direct connection (T23, T32, T33, T41)
 *
 * In direct mode (tx-isp's direct_mode) the multi-scaler writes a sensor's
 * channel 0 to no memory: its buffer addresses, Y 0x80000000 and UV
 * 0x81000000 with 4096 bytes a line, are the IVDC's, which passes the
 * lines to the encoder as the ISP sends them: HERA or the JPEG engine on
 * the T32 and T33, the Helix on the T23, the EL200 on the T41. Those
 * encoders' lists take the same addresses as their source. Frame channel
 * 0's frames carry no picture then, and on the T32 and T33 their one
 * "done" is this block's interrupt: status 0x44 bit 17, a frame starting,
 * with the sensor in 0xf0 bits 2:0. The T23's and T41's drivers complete
 * them on the ISP's own frame start and only count this block's
 * interrupts.
 *
 * The model takes a pulse on its input "frame" n, which the ISP model
 * sends in place of such a frame, as the start of sensor n's frame. The
 * ISP then sends its lines (0x10 bits 27:16, or the bank's) over three
 * quarters of the time to the next frame, as a T41's gc4023 sends 1440
 * lines in 38 of 50 ms. They pass through a RAM, 0x28 lines of Y in bits
 * 15:0 and of UV in 31:16 (none on the T23 and T33), and once that is
 * full through a ring in DDR, 0x30 lines split the same way. An encoder
 * takes the frame by writing bit 0 of 0x70 (VPU0, video) or 0x88 (VPU1,
 * JPEG); one that starts after the frame is gone waits for the next.
 * Status 0x44, as a T41 raises it: bit 17 at a frame's start, 13 once the
 * ISP has sent 0x4c lines, 14 (15 for VPU1) once the encoder, reading
 * 32-line groups 15 lines behind the ISP, has read more than 0x50, and 16
 * with 18 (19) at the end of a frame an encoder took. A frame that nobody
 * takes overflows the ring once the UV half would hold all but one of its
 * lines, the UV lines running 7 lines ahead of the Y lines: bits 1 (DDR UV
 * overflow) and 24 (VPU ack timeout), and the block drops the frame and
 * clears its line counts. A frame that ends before that overflows when
 * the next one starts.
 *
 * 0x208 holds the DDR ring's Y write line in bits 30:16, bit 31 toggling
 * at each wrap, the frames taken since the last overflow in bits 15:12
 * and the lines VPU0 has read of this one in 11:0; 0x21c the ISP's lines
 * of it in the RAM in bits 31:16 and the ring's read line in 14:0, bit 15
 * the wrap (0x3a8 on the T23). The T23's Helix driver reads 0x208 and
 * 0x3a8 in turn and cancels the job when the write line leads by more
 * than 500: whichever of the two it reads second holds its value from the
 * first read. The model's encoder reads each line as it comes where
 * silicon's trails by a group, so a line passing between the reads would
 * put the read line ahead, which the driver takes for 4G lines behind.
 * 0xec keeps the most lines the ring held, Y
 * in bits 14:0 and UV in 30:16, until written; at an overflow, with a
 * T41's 76-line RAM, 351 and 178 of a 360-line ring. The ring itself
 * holds no pixels: the encoders draw the scene for an IVDC source.
 *
 * Writing 0x0 with bit 0 or 1 set resets the block, done at once. 0x40
 * enables the status bits, writing ones to 0x54 clears them. The
 * encoders' drivers write sensor << 8 | 1 to 0x70 or 0x88 before a job
 * and after it want that sensor back in bits 6:4 of 0x7c or 0x94, with
 * bits 31:30, the errors, clear.
 *
 * Not modelled: the SID FIFO at 0x158 that several sensors need, the run
 * FIFO's pointers (0x78 to 0x80), the FIFO overflows (0x44 bits 2 and 3),
 * modes 2 to 4, and what follows the encoders' pace, which the model's
 * do not keep:
 * - The DDR half full events, bits 11 and 12, which the drivers only
 *   count: never on a T41. On a T32, whose HERA runs up to 300 to 500
 *   lines behind, 12 comes about once a frame whatever 0x48 holds and 11
 *   as often as 0x48 bits 15:0 (the driver's ivdc_mem_line / 2, 0 when
 *   not given) allow: once a frame at 0, about 7 at 80, none at 360.
 * - The lines in 0xa4 and 0xa8, the driver's ivdc_threshold_line (half
 *   the height when not given) and the height less it. A T32 takes them
 *   when its stream starts (writing them later changes nothing): at 100
 *   HERA loses every other frame, at 200 one in 14, each as a "VPU
 *   abandon" (bit 5) with no frame done; at 720 and 1300 none. 0xa8 has
 *   shown no effect, nor either line on a T41.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "migration/vmstate.h"
#include "hw/misc/ingenic-ivdc.h"
#include "trace.h"

#define IVDC_CTRL           0x000
#define  CTRL_RESET         (BIT(0) | BIT(1))
#define IVDC_ISP_SIZE       0x010
#define IVDC_RAM_LINES      0x028
#define IVDC_DDR_LINES      0x030
#define IVDC_INT_EN         0x040
#define IVDC_INT_STA        0x044
#define  INT_DDR_UV_OVF     BIT(1)
#define  INT_ISP_HALF       BIT(13)
#define  INT_VPU_HALF(v)    BIT(14 + (v))
#define  INT_FRM_DONE       BIT(16)
#define  INT_FRM_START      BIT(17)
#define  INT_VPU_END(v)     BIT(18 + (v))
#define  INT_VPU_ACK_TO     BIT(24)
#define IVDC_ISP_HALF_THR   0x04c
#define IVDC_VPU_HALF_THR   0x050
#define IVDC_INT_CLR        0x054
#define IVDC_VPU0_START     0x070
#define IVDC_VPU0_STAT      0x07c
#define IVDC_VPU0_FIFO      0x080
#define IVDC_VPU1_START     0x088
#define IVDC_VPU1_STAT      0x094
#define IVDC_MAX_LINE       0x0ec
#define IVDC_SENSOR         0x0f0
#define IVDC_SID_FIFO       0x158
#define IVDC_DDR_WR         0x208
#define IVDC_LINES          0x21c
#define IVDC_DDR_RD         0x3a8
#define IVDC_BANK(n)        (0x600 + ((n) - 1) * 0x40)

#define IVDC_UV_LEAD        7
#define IVDC_VPU_GROUP      32
#define IVDC_VPU_LAG        15
#define IVDC_PERIOD_NS      (NANOSECONDS_PER_SECOND / 25)

enum { FRAME_NONE, FRAME_LIVE, FRAME_DROPPED, FRAME_DONE };

#define R(off)              (s->regs[(off) / 4])

/* Sensor n's copy of a sensor 0 register: its offset in n's bank */
static uint32_t ivdc_sensor_reg(IngenicIvdcState *s, hwaddr off,
                                hwaddr bank_off)
{
    return s->sensor ? R(IVDC_BANK(s->sensor) + bank_off) : R(off);
}

static uint32_t ivdc_ram(IngenicIvdcState *s, bool uv)
{
    return extract32(R(IVDC_RAM_LINES), uv ? 16 : 0, 16);
}

static uint32_t ivdc_ring(IngenicIvdcState *s, bool uv)
{
    return extract32(R(IVDC_DDR_LINES), uv ? 16 : 0, 16);
}

static int64_t ivdc_line_time(IngenicIvdcState *s, uint32_t line)
{
    return s->start + (int64_t)line * s->line_ns;
}

/* The Y lines the ISP has sent of the frame by @now */
static uint32_t ivdc_isp_lines(IngenicIvdcState *s, int64_t now)
{
    if (now <= s->start) {
        return 0;
    }
    return MIN(s->lines, (now - s->start) / s->line_ns);
}

/* The ISP line at which a frame nobody takes overflows the UV ring */
static uint32_t ivdc_overflow_line(IngenicIvdcState *s)
{
    uint32_t uv = ivdc_ring(s, true);

    if (uv < 2) {
        return UINT32_MAX;
    }
    return MAX(2 * (uv - 1 + ivdc_ram(s, true)), IVDC_UV_LEAD + 1) -
           IVDC_UV_LEAD;
}

/* The lines the ring holds, Y and UV, once the ISP has sent @n */
static void ivdc_ring_used(IngenicIvdcState *s, uint32_t n, uint32_t *y,
                           uint32_t *uv)
{
    uint32_t ram_y = ivdc_ram(s, false), ram_uv = ivdc_ram(s, true);
    uint32_t uv_n = (n + IVDC_UV_LEAD) / 2, cap = ivdc_ring(s, true);

    *y = n > ram_y ? MIN(n - ram_y, ivdc_ring(s, false)) : 0;
    *uv = uv_n > ram_uv && cap >= 2 ? MIN(uv_n - ram_uv, cap - 2) : 0;
}

static void ivdc_note_used(IngenicIvdcState *s, uint32_t y, uint32_t uv)
{
    uint32_t v = R(IVDC_MAX_LINE);

    R(IVDC_MAX_LINE) = MAX(extract32(v, 16, 15), uv) << 16 |
                       MAX(extract32(v, 0, 15), y);
}

static bool ivdc_taken(IngenicIvdcState *s)
{
    return s->taken[0] >= 0 || s->taken[1] >= 0;
}

static int64_t ivdc_first_take(IngenicIvdcState *s)
{
    if (s->taken[0] < 0 || s->taken[1] < 0) {
        return MAX(s->taken[0], s->taken[1]);
    }
    return MIN(s->taken[0], s->taken[1]);
}

/*
 * The ring's Y write line by @now, counted modulo twice its lines: what
 * the ISP sent past the RAM before an encoder took the frame, or with no
 * RAM every line, which the encoder reads back as soon as it runs.
 */
static uint32_t ivdc_ring_write(IngenicIvdcState *s, int64_t now)
{
    uint32_t n, cap = ivdc_ring(s, false), ram_y = ivdc_ram(s, false);

    if (s->state != FRAME_LIVE || !cap) {
        return s->ring;
    }
    n = ivdc_isp_lines(s, now);
    if (ivdc_taken(s) && ram_y) {
        n = ivdc_isp_lines(s, ivdc_first_take(s));
    }
    n = n > ram_y ? n - ram_y : 0;
    return (s->ring + n) % (2 * cap);
}

static uint32_t ivdc_ring_read(IngenicIvdcState *s, int64_t now)
{
    if (s->state == FRAME_LIVE && !ivdc_taken(s)) {
        return s->ring;
    }
    return ivdc_ring_write(s, now);
}

/* A ring line as the counters show it: the wrap in the bit above */
static uint32_t ivdc_ring_reg(IngenicIvdcState *s, uint32_t line)
{
    uint32_t cap = ivdc_ring(s, false);

    return cap ? (line / cap) << 15 | line % cap : 0;
}

/* The lines VPU0 has read of the frame by @now */
static uint32_t ivdc_vpu_lines(IngenicIvdcState *s, int64_t now)
{
    uint32_t n = ivdc_isp_lines(s, now);

    if (s->taken[0] < 0 || s->state == FRAME_DROPPED ||
        now < s->taken[0] || n < IVDC_VPU_LAG) {
        return 0;
    }
    return QEMU_ALIGN_DOWN(n - IVDC_VPU_LAG, IVDC_VPU_GROUP);
}

/* When VPU v has read past its half done line */
static int64_t ivdc_vpu_half_time(IngenicIvdcState *s, unsigned v)
{
    uint32_t thr = extract32(ivdc_sensor_reg(s, IVDC_VPU_HALF_THR, 0x18),
                             0, 16);
    uint32_t line = QEMU_ALIGN_DOWN(thr, IVDC_VPU_GROUP) + IVDC_VPU_GROUP +
                    IVDC_VPU_LAG;

    return MAX(s->taken[v], ivdc_line_time(s, MIN(line, s->lines)));
}

static int64_t ivdc_event_time(IngenicIvdcState *s, uint32_t ev)
{
    uint32_t line;

    switch (ev) {
    case INT_ISP_HALF:
        line = extract32(ivdc_sensor_reg(s, IVDC_ISP_HALF_THR, 0x14), 0, 16);
        return line && line <= s->lines ? ivdc_line_time(s, line) : -1;
    case INT_DDR_UV_OVF:
        line = ivdc_overflow_line(s);
        return !ivdc_taken(s) && line < s->lines ? ivdc_line_time(s, line)
                                                 : -1;
    case INT_VPU_HALF(0):
    case INT_VPU_HALF(1):
        return s->taken[ev == INT_VPU_HALF(1)] < 0 ? -1 :
               ivdc_vpu_half_time(s, ev == INT_VPU_HALF(1));
    case INT_VPU_END(0):
    case INT_VPU_END(1):
        if (s->taken[ev == INT_VPU_END(1)] < 0) {
            return -1;
        }
        return MAX(s->taken[ev == INT_VPU_END(1)],
                   ivdc_line_time(s, s->lines));
    case INT_FRM_DONE:
        return ivdc_taken(s) ? MAX(ivdc_first_take(s),
                                   ivdc_line_time(s, s->lines)) : -1;
    }
    g_assert_not_reached();
}

static void ivdc_update(IngenicIvdcState *s)
{
    qemu_set_irq(s->irq, !!(R(IVDC_INT_STA) & R(IVDC_INT_EN)));
}

static void ivdc_drop(IngenicIvdcState *s)
{
    uint32_t y, uv;

    ivdc_ring_used(s, MIN(ivdc_overflow_line(s), s->lines), &y, &uv);
    ivdc_note_used(s, y, uv);
    R(IVDC_INT_STA) |= INT_DDR_UV_OVF | INT_VPU_ACK_TO;
    s->state = FRAME_DROPPED;
    s->pending = 0;
    s->ring = 0;
    s->frames = 0;
}

/* Raise the frame's events due by @now, in order, and time the next */
static void ivdc_run(IngenicIvdcState *s, int64_t now)
{
    int64_t next;

    for (;;) {
        uint32_t ev = 0;

        next = INT64_MAX;
        for (uint32_t todo = s->pending; todo; todo &= todo - 1) {
            int64_t t = ivdc_event_time(s, 1u << ctz32(todo));

            if (t >= 0 && t < next) {
                next = t;
                ev = 1u << ctz32(todo);
            }
        }
        if (!ev || next > now) {
            break;
        }
        if (ev == INT_DDR_UV_OVF) {
            ivdc_drop(s);
            next = INT64_MAX;
            break;
        }
        s->pending &= ~ev;
        R(IVDC_INT_STA) |= ev;
    }
    if (s->state == FRAME_LIVE && ivdc_taken(s) &&
        !(s->pending & (INT_FRM_DONE | INT_VPU_END(0) | INT_VPU_END(1)))) {
        s->ring = ivdc_ring_write(s, now);
        s->state = FRAME_DONE;
    }
    if (next == INT64_MAX) {
        timer_del(s->timer);
    } else {
        timer_mod(s->timer, next);
    }
    ivdc_update(s);
}

static void ivdc_timer(void *opaque)
{
    IngenicIvdcState *s = opaque;

    ivdc_run(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

static void ivdc_take(IngenicIvdcState *s, unsigned v, int64_t now)
{
    uint32_t y, uv;

    trace_ingenic_ivdc_take(v, ivdc_isp_lines(s, now));
    if (!ivdc_taken(s)) {
        ivdc_ring_used(s, ivdc_isp_lines(s, now), &y, &uv);
        ivdc_note_used(s, y, uv);
    }
    s->taken[v] = now;
    s->pending &= ~INT_DDR_UV_OVF;
    s->pending |= INT_VPU_HALF(v) | INT_VPU_END(v) | INT_FRM_DONE;
}

static void ivdc_vpu_start(IngenicIvdcState *s, unsigned v)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    ivdc_run(s, now);
    if (s->state == FRAME_LIVE && s->taken[v] < 0) {
        ivdc_take(s, v, now);
        ivdc_run(s, now);
    } else if (s->queued[v] < INGENIC_IVDC_SENSORS) {
        s->queued[v]++;
    }
}

static void ivdc_frame_in(void *opaque, int n, int level)
{
    IngenicIvdcState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t period = now - s->start;

    if (!level) {
        return;
    }
    ivdc_run(s, now);
    if (s->state == FRAME_LIVE) {
        if (ivdc_taken(s)) {
            ivdc_run(s, INT64_MAX);
        } else {
            ivdc_drop(s);
        }
    }
    if (s->state == FRAME_DONE) {
        s->frames++;
    }
    if (s->state == FRAME_NONE || period < SCALE_MS ||
        period > NANOSECONDS_PER_SECOND) {
        period = IVDC_PERIOD_NS;
    }
    s->sensor = n;
    s->start = now;
    s->lines = MAX(extract32(ivdc_sensor_reg(s, IVDC_ISP_SIZE, 0x00), 16, 12),
                   1);
    trace_ingenic_ivdc_frame(n, s->lines, period / 1000);
    s->line_ns = MAX(period * 3 / 4 / s->lines, 1);
    s->state = FRAME_LIVE;
    s->taken[0] = s->taken[1] = -1;
    s->pending = INT_ISP_HALF | INT_DDR_UV_OVF;
    R(IVDC_SENSOR) = n;
    R(IVDC_INT_STA) |= INT_FRM_START;
    for (unsigned v = 0; v < 2; v++) {
        if (s->queued[v]) {
            s->queued[v]--;
            ivdc_take(s, v, now);
        }
    }
    ivdc_run(s, now);
}

static uint64_t ingenic_ivdc_read(void *opaque, hwaddr addr, unsigned size)
{
    IngenicIvdcState *s = INGENIC_IVDC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t y, uv, n, v;

    /* a timer late on the virtual clock must not let a read see the past */
    ivdc_run(s, now);
    switch (addr) {
    case IVDC_INT_CLR:
        return 0;
    case IVDC_DDR_WR:
    case IVDC_DDR_RD:
        if (s->pair_next == addr) {
            s->pair_next = 0;
            v = s->pair_value;
        } else {
            uint32_t wr = ivdc_ring_reg(s, ivdc_ring_write(s, now)) << 16 |
                          (s->frames & 0xf) << 12 | ivdc_vpu_lines(s, now);
            uint32_t rd = ivdc_ring_reg(s, ivdc_ring_read(s, now));

            s->pair_next = addr == IVDC_DDR_WR ? IVDC_DDR_RD : IVDC_DDR_WR;
            s->pair_value = addr == IVDC_DDR_WR ? rd : wr;
            v = addr == IVDC_DDR_WR ? wr : rd;
            trace_ingenic_ivdc_ring(wr, rd);
        }
        return v;
    case IVDC_LINES:
        n = 0;
        if (s->state == FRAME_LIVE) {
            n = ivdc_isp_lines(s, now);
            if (!ivdc_taken(s)) {
                n = MIN(n, ivdc_ram(s, false));
            }
        }
        return n << 16 | ivdc_ring_reg(s, ivdc_ring_read(s, now));
    case IVDC_MAX_LINE:
        if (s->state == FRAME_LIVE && !ivdc_taken(s)) {
            ivdc_ring_used(s, ivdc_isp_lines(s, now), &y, &uv);
            return MAX(extract32(R(addr), 16, 15), uv) << 16 |
                   MAX(extract32(R(addr), 0, 15), y);
        }
        break;
    }
    return R(addr);
}

static void ivdc_reset(IngenicIvdcState *s)
{
    timer_del(s->timer);
    s->pair_next = 0;
    s->state = FRAME_NONE;
    s->pending = 0;
    s->queued[0] = s->queued[1] = 0;
    s->ring = 0;
    s->frames = 0;
}

static void ingenic_ivdc_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    IngenicIvdcState *s = INGENIC_IVDC(opaque);
    uint32_t v = value;

    switch (addr) {
    case IVDC_CTRL:
        if (v & CTRL_RESET) {
            R(IVDC_INT_STA) = 0;
            R(IVDC_VPU0_STAT) = R(IVDC_VPU1_STAT) = 0;
            R(IVDC_VPU0_FIFO) = R(IVDC_SID_FIFO) = R(IVDC_MAX_LINE) = 0;
            ivdc_reset(s);
            ivdc_update(s);
        }
        R(addr) = v & ~CTRL_RESET;
        return;
    case IVDC_INT_EN:
        R(addr) = v;
        ivdc_update(s);
        return;
    case IVDC_INT_STA:
    case IVDC_DDR_WR:
    case IVDC_LINES:
    case IVDC_DDR_RD:
        return;
    case IVDC_INT_CLR:
        R(IVDC_INT_STA) &= ~v;
        ivdc_update(s);
        return;
    case IVDC_VPU0_START:
    case IVDC_VPU1_START:
        R(addr) = v;
        if (v & 1) {
            R(addr == IVDC_VPU0_START ? IVDC_VPU0_STAT : IVDC_VPU1_STAT) =
                extract32(v, 8, 3) << 4;
            ivdc_vpu_start(s, addr == IVDC_VPU1_START);
        }
        return;
    }
    R(addr) = v;
}

static const MemoryRegionOps ingenic_ivdc_ops = {
    .read = ingenic_ivdc_read,
    .write = ingenic_ivdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_ivdc_reset_hold(Object *obj, ResetType type)
{
    IngenicIvdcState *s = INGENIC_IVDC(obj);

    memset(s->regs, 0, sizeof(s->regs));
    ivdc_reset(s);
    s->start = 0;
}

static void ingenic_ivdc_reset_exit(Object *obj, ResetType type)
{
    ivdc_update(INGENIC_IVDC(obj));
}

static void ingenic_ivdc_init(Object *obj)
{
    IngenicIvdcState *s = INGENIC_IVDC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_ivdc_ops, s,
                          TYPE_INGENIC_IVDC, INGENIC_IVDC_IOSIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in_named(DEVICE(obj), ivdc_frame_in, "frame",
                            INGENIC_IVDC_SENSORS);
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, ivdc_timer, s);
}

static const VMStateDescription vmstate_ingenic_ivdc = {
    .name = TYPE_INGENIC_IVDC,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIvdcState, INGENIC_IVDC_IOSIZE / 4),
        VMSTATE_TIMER_PTR(timer, IngenicIvdcState),
        VMSTATE_INT64(start, IngenicIvdcState),
        VMSTATE_INT64(line_ns, IngenicIvdcState),
        VMSTATE_UINT32(lines, IngenicIvdcState),
        VMSTATE_UINT32(sensor, IngenicIvdcState),
        VMSTATE_UINT32(state, IngenicIvdcState),
        VMSTATE_UINT32(pending, IngenicIvdcState),
        VMSTATE_INT64_ARRAY(taken, IngenicIvdcState, 2),
        VMSTATE_UINT32_ARRAY(queued, IngenicIvdcState, 2),
        VMSTATE_UINT32(ring, IngenicIvdcState),
        VMSTATE_UINT32(frames, IngenicIvdcState),
        VMSTATE_UINT32(pair_next, IngenicIvdcState),
        VMSTATE_UINT32(pair_value, IngenicIvdcState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_ivdc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->desc = "Ingenic IVDC";
    dc->vmsd = &vmstate_ingenic_ivdc;
    rc->phases.hold = ingenic_ivdc_reset_hold;
    rc->phases.exit = ingenic_ivdc_reset_exit;
}

static const TypeInfo ingenic_ivdc_types[] = {
    {
        .name = TYPE_INGENIC_IVDC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicIvdcState),
        .instance_init = ingenic_ivdc_init,
        .class_init = ingenic_ivdc_class_init,
    },
};

DEFINE_TYPES(ingenic_ivdc_types)
