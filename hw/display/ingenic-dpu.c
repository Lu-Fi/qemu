/*
 * Ingenic DPU: the T40's display controller, and the T41's LCDC tiny
 *
 * A composer reads a chain of frame descriptors (next address, size,
 * control, writeback buffer and stride, four layer descriptor addresses,
 * the layers' enables, scale enables and Z orders, interrupt masks) and
 * blends up to four layers into a frame, which it sends to the display
 * (DirectEn), writes back to memory (WriteBack), or both. A simple read
 * channel instead sends a frame buffer from its own chain. The frame goes
 * out as TFT (HSYNC to VDE timings) or smart LCD (SLCD), at the pixel
 * clock (the CPM's LPCDR), and is shown on a QEMU console.
 *
 * As a T40N composes with writeback at U-Boot: the background is
 * transparent black; layers stack by their Z order, and a disabled layer
 * given an enabled one's Z hides it. A layer's alpha is its pixels' (the
 * ARGB formats) or 255; with GAlpha_en it is the global alpha, or the
 * pixel's times global alpha plus one over 256. With layer config bit 14
 * the colour is (s * a + d * (255 - a) + 128) >> 8, d at alpha 0 and s at
 * 255; without it the source counts as premultiplied, s + (d * (255 - a)
 * + 128 >> 8) saturating. Alpha adds up as a + (d_a * (255 - a) + 127) /
 * 255. A writeback-only frame raises no CMP_START interrupt. RGB565 and
 * RGB555 widen by shifting; YUV goes through the layer's CSC registers,
 * the products summed and shifted right by 10. A TFT frame lasts HPE *
 * VPE pixel clocks, an SLCD frame one transfer per pixel per bus width of
 * DSTIME + DDTIME + 2 clocks plus two (not measured on the T40N; the
 * T41's is one). ST holds
 * each event; the interrupt is ST & INTC, DISP_END, WDMA_END, CMP_START
 * and CMP_END also gated by the descriptor's masks.
 *
 * Reads of 0x8000 to 0x8fff and 0xb000 up return DISP_COM. With CLKGR
 * bit 24 set the block reads 0 and drops writes; with the pixel clock
 * stopped a read of the display registers (0x8000 up) hangs the bus,
 * which is only logged here.
 *
 * The T41 has the T21's smaller controller instead, SLCD only: one chain
 * of descriptors (next address, RGB or Y address and stride, CHAIN_END,
 * FRM_END's mask, UV address and stride), one source format in GLB_CFG.
 * As a T41LQ and a T41NQ run it at U-Boot: START takes a descriptor and the
 * DMA fills its FIFO, setting DMA_END and DMA_SITE at once if the frame fits
 * (1920 bytes of 4-byte pixels, 1024 of 2-byte ones, within a few pixels
 * and shape) and WORKING if not; SLCD_START sends the frame (the panel's TE
 * taken as there at once), setting FRM_END at its end, with DMA_END too for
 * a frame that did not fit. CHAIN_END stops the chain; otherwise the next
 * descriptor is taken and, with SLCD_CFG bit 31, sent. DMA_SITE ends at the
 * last 64-byte burst of the frame's last line; DES_READ gives the
 * descriptor's words with INTC in place of the interrupt word. A frame is
 * pixels * (transfers * (DSTIME + DDTIME + 2) + 1) pixel clocks, its size
 * fields 10 bits a side (a height of 0 is 1024 lines, a width of 0 or 1024
 * never ends the frame), a quick or general stop's acknowledge comes up
 * again with the next START, and 0xa01c holds one bit. The T41's has no
 * TFT: 0x9000 up reads 0, and with the pixel clock stopped a read of the
 * display registers (0x8000 up, SLCD_CFG too) hangs the bus.
 *
 * The T21, T23 and T31 have this block too (a T23N's and a T31X's, with
 * the T21N's registers alike): the same resets and masks and the same
 * behaviour but a FIFO of 1984 and 1088 bytes, no hang with the pixel
 * clock stopped (SLCD_CFG reads 0) and LPCDR's change enable in bit 28 (the
 * T41's is bit 29). The pixel clock waits for a LPCDR write with CE. A T31X
 * also loses the first SLCD register write after a read of the display
 * registers.
 *
 * Write-back, as a T40N does it: ARGB8888, RGB565, ARGB1555, YUV422 and
 * one byte a pixel, RGB888 as 32 bits with 0xff on top; a 4x4 ordered dither
 * on the channels before they truncate (before YUV422's and the byte's
 * conversion too); a layer scaler, a Catmull-Rom cubic
 * of the four source pixels about x * Layer_Resize_Coef / 512, that rounds as
 * the silicon does for 2x and 1.5x of a ramp and a step but is not known to
 * the bit for other ratios. The layer rotation field does nothing there, as
 * the manual says.
 *
 * A layer can be read through the TLB (GLBC's TLBE for its channel), as a
 * T40NN does it: see dpu_tlb_xlate.
 *
 * Not modelled: the T21N's frame engine (its DMA ran only once, and not
 * reproducibly), the T30's smaller DPU, the TLB's fetch modes and its
 * cache size, the scaler's sharp levels, the TFT and SLCD wire formats,
 * MIPI DSI, the composer's prefetch ahead of the display (CMP_START comes
 * some lines before the frame on silicon).
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bswap.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/core/clock.h"
#include "hw/core/qdev-clock.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "system/dma.h"
#include "ui/console.h"
#include "hw/display/ingenic-dpu.h"

#define DC_FRM_CFG_ADDR     0x0000
#define DC_FRM_CFG_CTRL     0x0004
#define DC_SRD_CHAIN_ADDR   0x1000
#define DC_SRD_CHAIN_CTRL   0x1004
#define DC_CTRL             0x2000
#define DC_ST               0x2004
#define DC_CLR_ST           0x2008
#define DC_INTC             0x200c
#define DC_INT_FLAG         0x2010
#define DC_COM_CFG          0x2014
#define DC_PCFG             0x2018
#define DC_FRM_DES          0x2100
#define DC_LAY0_DES         0x2104
#define DC_RDMA_DES         0x2114
#define DC_FRM_SITE         0x2200
#define DC_RDMA_CHAIN_SITE  0x2204
#define DC_WDMA_SITE        0x221c
#define DC_TLB_GLBC         0x3000
#define DC_TLB_TLBA         0x3010
#define DC_TLB_TLBC         0x3020
#define DC_TLB_VPN          0x3030
#define DC_TLB_TLBV         0x3040
#define DC_TLB_STAT         0x3050
#define DC_LAY0_SITE        0x3100
#define DC_RDMA_SITE        0x3110
#define DC_CSC              0x3200
#define DC_DISP_COM         0x8000
#define DC_TFT              0x9000
#define DC_TFT_ST           0x9014
#define DC_SLCD             0xa000
#define DC_SLCD_ST          0xa018

#define CTRL_QCK_STP_CMP    (1 << 0)
#define CTRL_QCK_STP_SRD    (1 << 1)
#define CTRL_DES_CNT_RST    (1 << 2)
#define CTRL_GEN_STP_CMP    (1 << 3)
#define CTRL_GEN_STP_SRD    (1 << 4)

#define ST_WORKING          (1 << 0)
#define ST_SRD_END          (1 << 1)
#define ST_SRD_START        (1 << 2)
#define ST_SRD_WORKING      (1 << 3)
#define ST_DIRECT_WORKING   (1 << 4)
#define ST_WRBK_WORKING     (1 << 5)
#define ST_STOP_CMP_ACK     (1 << 6)
#define ST_STOP_SRD_ACK     (1 << 7)
#define ST_CMP_END          (1 << 9)
#define ST_LAY0_END         (1 << 10)
#define ST_CMP_START        (1 << 14)
#define ST_CMP_W_SLOW       (1u << 31)
#define ST_WDMA_END         (1 << 15)
#define ST_DISP_END         (1 << 17)
#define ST_EVENTS           0x8003ffc6
/* The events a descriptor's masks also gate */
#define CMP_DESC_MASKS      (ST_DISP_END | ST_WDMA_END | ST_CMP_START | \
                             ST_CMP_END)
#define SRD_DESC_MASKS      (ST_DISP_END | ST_SRD_START | ST_SRD_END)

#define FRM_STOP            (1 << 0)
#define FRM_WB              (1 << 1)
#define FRM_DIRECT          (1 << 2)
#define FRM_CHANGE_2_SRD    (1 << 3)
#define SRD_CHAIN_END       (1 << 0)
#define SRD_CHANGE_2_CMP    (1 << 1)

#define DISP_IF_TFT         1

/* Source formats, as the layer descriptors code them */
enum {
    FMT_RGB555 = 0,
    FMT_ARGB1555 = 1,
    FMT_RGB565 = 2,
    FMT_RGB888 = 4,
    FMT_ARGB8888 = 5,
    FMT_MONO8 = 6,
    FMT_NV12 = 8,
    FMT_NV21 = 9,
    FMT_YUV422 = 10,
};

/* Which of a pixel's three colour fields, high to low, hold R, G and B */
static const uint8_t dpu_order[8][3] = {
    { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 2, 0, 1 },
    { 1, 2, 0 }, { 2, 1, 0 }, { 0, 1, 2 }, { 0, 1, 2 },
};

static uint8_t sat8(int v)
{
    return MIN(MAX(v, 0), 255);
}

static uint32_t argb(unsigned a, unsigned r, unsigned g, unsigned b)
{
    return a << 24 | r << 16 | g << 8 | b;
}

/* YUV to RGB through a CSC block: CSC_MULT_YRV, GUGV, BU, SUB_YUV */
static uint32_t dpu_csc(const uint32_t *csc, int y, int u, int v)
{
    int my = extract32(csc[0], 0, 11), mrv = extract32(csc[0], 16, 11);
    int mgu = extract32(csc[1], 0, 9), mgv = extract32(csc[1], 16, 10);
    int mbu = extract32(csc[2], 0, 12);
    int yy = (y - (int)extract32(csc[3], 0, 5)) * my;
    int sub = extract32(csc[3], 16, 8);

    u -= sub;
    v -= sub;
    return argb(255, sat8((yy + mrv * v) >> 10),
                sat8((yy - mgu * u - mgv * v) >> 10),
                sat8((yy + mbu * u) >> 10));
}

static unsigned dpu_bytes_pp(unsigned fmt)
{
    switch (fmt) {
    case FMT_RGB888:
    case FMT_ARGB8888:
        return 4;
    case FMT_MONO8:
    case FMT_NV12:
    case FMT_NV21:
        return 1;
    default:
        return 2;
    }
}

/*
 * The TLB of a layer's channel, as a T40NN does it with GLBC's TLBE set for
 * the layer: the address's bits 31:22 index a table of 4-byte entries at
 * TLBA, whose entry's bits 31:12 are a second table's page, indexed by bits
 * 21:12; that entry's bits 31:12 are the 4 KiB page. An entry is usable
 * with bit 0 set, and the second one also with (entry ^ GCN) & CNM clear
 * in its low 12 bits; TLBV's RCI skips both checks. Translations are
 * cached until TLBC invalidates them, whatever the table says meanwhile;
 * the cache's size is not known.
 *
 * An unusable entry records the page in the channel's VPN register and the
 * channel in STAT (bit 3 - n for channel n, as every channel with a miss
 * recorded), and stalls the composer until TLBC's RETRY. A page that
 * missed does not record again until retried.
 */
static bool dpu_tlb_usable(const IngenicDpuState *s, uint32_t e, bool second)
{
    uint32_t cnm = extract32(s->tlbv, 16, 12), gcn = extract32(s->tlbv, 0, 12);

    return s->tlbv & (1u << 31) ||
           (e & 1 && (!second || !((e ^ gcn) & cnm)));
}

static bool dpu_tlb_xlate(IngenicDpuState *s, unsigned ch, uint32_t va,
                          hwaddr *pa)
{
    uint32_t page = va & ~0xfffu, e;
    uint32_t *ent = s->tlb_ent[ch];

    for (unsigned i = 0; i < INGENIC_DPU_TLB_ENTRIES; i++) {
        if (ent[2 * i] == (page | 1)) {
            *pa = ent[2 * i + 1];
            return true;
        }
    }
    if (s->tlb_neg[ch] == (page | 1)) {
        return false;
    }
    dma_memory_read(&address_space_memory, s->tlba + 4 * (va >> 22), &e, 4,
                    MEMTXATTRS_UNSPECIFIED);
    e = le32_to_cpu(e);
    if (dpu_tlb_usable(s, e, false)) {
        dma_memory_read(&address_space_memory,
                        (e & ~0xfffu) + 4 * extract32(va, 12, 10), &e, 4,
                        MEMTXATTRS_UNSPECIFIED);
        e = le32_to_cpu(e);
        if (dpu_tlb_usable(s, e, true)) {
            unsigned i = s->tlb_rr[ch]++ % INGENIC_DPU_TLB_ENTRIES;

            ent[2 * i] = page | 1;
            ent[2 * i + 1] = e & ~0xfffu;
            *pa = ent[2 * i + 1];
            return true;
        }
    }
    s->tlb_neg[ch] = page | 1;
    s->tlb_vpn[ch] = page;
    s->tlb_pend |= 1 << ch;
    for (unsigned c = 0; c < INGENIC_DPU_LAYERS; c++) {
        s->tlb_stat |= (s->tlb_pend >> c & 1) << (3 - c);
    }
    return false;
}

/* A read through layer @ch's TLB if it has one on: false on a miss */
static bool dpu_read(IngenicDpuState *s, unsigned ch, hwaddr addr, void *buf,
                     size_t len)
{
    if (!s || !(s->tlb_glbc >> ch & 1)) {
        dma_memory_read(&address_space_memory, addr, buf, len,
                        MEMTXATTRS_UNSPECIFIED);
        return true;
    }
    while (len) {
        size_t n = MIN(len, 0x1000 - (addr & 0xfff));
        hwaddr pa;

        if (!dpu_tlb_xlate(s, ch, addr, &pa)) {
            return false;
        }
        dma_memory_read(&address_space_memory, pa | (addr & 0xfff), buf, n,
                        MEMTXATTRS_UNSPECIFIED);
        addr += n;
        buf = (uint8_t *)buf + n;
        len -= n;
    }
    return true;
}

/*
 * One line of @w pixels at @addr in format @fmt to ARGB; @uv is the line's
 * chroma for NV12 and NV21, @csc its CSC block; @s and @ch name the layer
 * whose TLB it goes through, if any. False if the TLB missed.
 */
static bool dpu_fetch_line(uint32_t *out, unsigned fmt, unsigned order,
                           hwaddr addr, hwaddr uv, unsigned w,
                           const uint32_t *csc, IngenicDpuState *s,
                           unsigned ch)
{
    g_autofree uint8_t *buf = g_malloc(w * 4 + 4);
    g_autofree uint8_t *cb = NULL;
    const uint8_t *o = dpu_order[order & 7];

    if (!dpu_read(s, ch, addr, buf, w * dpu_bytes_pp(fmt))) {
        return false;
    }
    if (fmt == FMT_NV12 || fmt == FMT_NV21) {
        cb = g_malloc(w + 2);
        if (!dpu_read(s, ch, uv, cb, (w + 1) & ~1u)) {
            return false;
        }
    }
    for (unsigned i = 0; i < w; i++) {
        uint8_t c[3];
        unsigned a = 255;
        uint32_t p;

        switch (fmt) {
        case FMT_RGB555:
        case FMT_ARGB1555:
            p = lduw_le_p(buf + 2 * i);
            c[0] = extract32(p, 10, 5) << 3;
            c[1] = extract32(p, 5, 5) << 3;
            c[2] = extract32(p, 0, 5) << 3;
            if (fmt == FMT_ARGB1555) {
                a = p & 0x8000 ? 255 : 0;
            }
            break;
        case FMT_RGB565:
            p = lduw_le_p(buf + 2 * i);
            c[0] = extract32(p, 11, 5) << 3;
            c[1] = extract32(p, 5, 6) << 2;
            c[2] = extract32(p, 0, 5) << 3;
            break;
        case FMT_RGB888:
        case FMT_ARGB8888:
            p = ldl_le_p(buf + 4 * i);
            c[0] = p >> 16;
            c[1] = p >> 8;
            c[2] = p;
            if (fmt == FMT_ARGB8888) {
                a = p >> 24;
            }
            break;
        case FMT_MONO8:
            c[0] = c[1] = c[2] = buf[i];
            break;
        case FMT_NV12:
        case FMT_NV21: {
            int u = cb[i & ~1u], v = cb[i | 1];

            out[i] = fmt == FMT_NV12 ? dpu_csc(csc, buf[i], u, v)
                                     : dpu_csc(csc, buf[i], v, u);
            continue;
        }
        case FMT_YUV422:
            /* V Y1 U Y0 in each 32 bits, two pixels */
            p = ldl_le_p(buf + 4 * (i / 2));
            out[i] = dpu_csc(csc, i & 1 ? extract32(p, 16, 8) : p & 0xff,
                             extract32(p, 8, 8), p >> 24);
            continue;
        default:
            out[i] = 0;
            continue;
        }
        out[i] = argb(a, c[o[0]], c[o[1]], c[o[2]]);
    }
    return true;
}

/* A source pixel @s of alpha @a over @d, as a T40N blends */
static uint32_t dpu_blend(uint32_t s, unsigned a, uint32_t d, bool mult)
{
    unsigned ad = d >> 24, ao;
    uint32_t r = 0;

    if (a == 0 && mult) {
        return d;
    }
    ao = a + (ad * (255 - a) + 127) / 255;
    for (int sh = 0; sh < 24; sh += 8) {
        unsigned sc = (s >> sh) & 0xff, dc = (d >> sh) & 0xff, c;

        if (!mult) {
            c = MIN(255, sc + ((dc * (255 - a) + 128) >> 8));
        } else if (a == 255) {
            c = sc;
        } else {
            c = (sc * a + dc * (255 - a) + 128) >> 8;
        }
        r |= c << sh;
    }
    return r | ao << 24;
}

/*
 * Smart LCD transfers: a pixel takes one transfer of DSTIME + DDTIME + 2
 * clocks on a bus of 16 bits or more, and more on a narrower one (the
 * format's bits over the bus's: 9 bits for 18-bit pixels, 8 for 16, 18 or
 * 24), plus @extra clocks: two on a T41LQ, one on a T23N and a T31X
 */
static uint64_t slcd_frame_clocks(const IngenicDpuDisp *d, uint64_t pixels,
                                  unsigned extra)
{
    static const unsigned width[8] = { 8, 9, 16, 18, 24, 24, 24, 24 };
    static const unsigned bits[4] = { 16, 18, 24, 24 };
    uint32_t cfg = d->slcd[0], duty = d->slcd[1];
    unsigned bus = width[extract32(cfg, 3, 3)];
    unsigned bpp = bus == 9 ? 18 : bits[extract32(cfg, 21, 2)];
    uint64_t per = extract32(duty, 24, 8) + extract32(duty, 16, 8) + 2;
    unsigned transfers = bus >= 16 ? 1 : DIV_ROUND_UP(bpp, bus);

    return pixels * (transfers * per + extra);
}

static int64_t clocks_ns(Clock *clk, uint64_t clocks)
{
    uint64_t hz = clock_get_hz(clk);

    if (!hz) {
        return 0;
    }
    return MAX(muldiv64(clocks, NANOSECONDS_PER_SECOND, hz), 1);
}

static void dpu_show(QemuConsole *con, const uint32_t *pix, unsigned w,
                     unsigned h)
{
    DisplaySurface *ds = qemu_console_surface(con);

    if (!w || !h) {
        return;
    }
    if (!ds || surface_width(ds) != w || surface_height(ds) != h) {
        qemu_console_resize(con, w, h);
        ds = qemu_console_surface(con);
    }
    if (surface_bits_per_pixel(ds) != 32) {
        return;
    }
    for (unsigned j = 0; j < h; j++) {
        uint32_t *d = (uint32_t *)((uint8_t *)surface_data(ds) +
                                   j * surface_stride(ds));

        for (unsigned i = 0; i < w; i++) {
            d[i] = pix[j * w + i] & 0xffffff;
        }
    }
    qemu_console_update(con, 0, 0, w, h);
}

static void dpu_read_desc(hwaddr addr, uint32_t *w, unsigned n)
{
    dma_memory_read(&address_space_memory, addr, w, n * 4,
                    MEMTXATTRS_UNSPECIFIED);
    for (unsigned i = 0; i < n; i++) {
        w[i] = le32_to_cpu(w[i]);
    }
}

/* ---- the T40's DPU ---- */

static uint32_t dpu_int_flag(IngenicDpuState *s)
{
    uint32_t f = s->st & s->intc;

    f &= ~(CMP_DESC_MASKS | SRD_DESC_MASKS) | s->desc_mask;
    return f & (~ST_DISP_END | s->eod_mask);
}

static void dpu_update_irq(IngenicDpuState *s)
{
    qemu_set_irq(s->irq, !!dpu_int_flag(s));
}

static unsigned dpu_disp_if(IngenicDpuState *s)
{
    return extract32(s->disp.disp_com, 0, 2);
}

/* A frame's length in pixel clocks on the selected interface */
static int64_t dpu_frame_ns(IngenicDpuState *s, unsigned w, unsigned h)
{
    uint64_t clocks;

    if (dpu_disp_if(s) == DISP_IF_TFT) {
        clocks = (uint64_t)extract32(s->disp.tft[0], 0, 12) *
                 extract32(s->disp.tft[1], 0, 12);
    } else if (dpu_disp_if(s)) {
        clocks = slcd_frame_clocks(&s->disp, (uint64_t)w * h, 2);
    } else {
        clocks = (uint64_t)w * h;
    }
    return clocks_ns(s->clk, MAX(clocks, 1));
}

static void dpu_arm(IngenicDpuState *s, unsigned w, unsigned h)
{
    int64_t ns = dpu_frame_ns(s, w, h);

    if (ns) {
        timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    } else {
        timer_del(s->frame);
    }
}

/*
 * A layer's pixel (@x, @y) after the scaler: Catmull-Rom cubic over the four
 * source pixels around the position x * @cx / 512 (@cy for y), the edge
 * pixels repeating past the layer, rounded and clamped per channel. A zero
 * coefficient leaves that axis alone. The silicon's weights are not
 * known to the bit, so a value can differ from it by one.
 */
static void dpu_cubic(double t, double w[4])
{
    w[0] = (-t * t * t + 2 * t * t - t) / 2;
    w[1] = (3 * t * t * t - 5 * t * t + 2) / 2;
    w[2] = (-3 * t * t * t + 4 * t * t + t) / 2;
    w[3] = (t * t * t - t * t) / 2;
}

static uint32_t dpu_resize(const uint32_t *src, unsigned sw, unsigned sh,
                           unsigned x, unsigned y, uint32_t cx, uint32_t cy)
{
    uint64_t px = (uint64_t)x * (cx ? cx : 512);
    uint64_t py = (uint64_t)y * (cy ? cy : 512);
    unsigned ix = px >> 9, iy = py >> 9;
    double wx[4], wy[4], acc[4] = { 0 };
    uint32_t r = 0;

    dpu_cubic((px & 511) / 512.0, wx);
    dpu_cubic((py & 511) / 512.0, wy);
    for (int j = 0; j < 4; j++) {
        unsigned yy = MIN(MAX((int)iy - 1 + j, 0), (int)sh - 1);

        for (int i = 0; i < 4; i++) {
            unsigned xx = MIN(MAX((int)ix - 1 + i, 0), (int)sw - 1);
            uint32_t p = src[yy * sw + xx];

            for (int c = 0; c < 4; c++) {
                acc[c] += wx[i] * wy[j] * ((p >> (8 * c)) & 0xff);
            }
        }
    }
    for (int c = 0; c < 4; c++) {
        r |= (uint32_t)sat8((int)(acc[c] + 0.5)) << (8 * c);
    }
    return r;
}

/*
 * Layer @l of the current frame blended into @out (@fw x @fh); false if its
 * TLB missed
 */
static bool dpu_compose_layer(IngenicDpuState *s, unsigned l, uint32_t *out,
                              unsigned fw, unsigned fh)
{
    const uint32_t *d = s->lay[l];
    unsigned sw = extract32(d[0], 0, 12), sh = extract32(d[0], 16, 12);
    unsigned fmt = extract32(d[1], 16, 4), order = extract32(d[1], 10, 3);
    bool galpha = d[1] & (1 << 13), mult = d[1] & (1 << 14);
    unsigned g = d[1] & 0xff;
    bool scale = s->frm[9] & (1 << l);
    unsigned tw = scale ? extract32(d[3], 0, 12) : 0;
    unsigned th = scale ? extract32(d[3], 16, 12) : 0;
    unsigned px = extract32(d[6], 0, 12), py = extract32(d[6], 16, 12);
    unsigned dw = tw ? tw : sw, dh = th ? th : sh;
    bool yuv = fmt == FMT_NV12 || fmt == FMT_NV21 || fmt == FMT_YUV422;
    hwaddr stride = (hwaddr)extract32(d[9], 0, 12) * dpu_bytes_pp(fmt);
    hwaddr uvs = extract32(d[11], 0, 12);
    g_autofree uint32_t *src = NULL;

    if (!sw || !sh) {
        return true;
    }
    if (yuv && l > 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-dpu: layer %u is YUV, which "
                      "only layers 0 and 1 convert\n", l);
        return true;
    }
    if (fmt == 7 || fmt == 3 || fmt > FMT_YUV422) {
        qemu_log_mask(LOG_UNIMP, "ingenic-dpu: layer format %u\n", fmt);
        return true;
    }
    src = g_new(uint32_t, (size_t)sw * sh);
    for (unsigned y = 0; y < sh; y++) {
        if (!dpu_fetch_line(src + (size_t)y * sw, fmt, order,
                            d[2] + y * stride, d[10] + (y / 2) * uvs, sw,
                            s->csc[l & 1], s, l)) {
            return false;
        }
    }
    for (unsigned y = 0; y < dh && py + y < fh; y++) {
        uint32_t *o = out + (py + y) * fw;

        for (unsigned x = 0; x < dw && px + x < fw; x++) {
            uint32_t p = tw || th ? dpu_resize(src, sw, sh, x, y, tw ? d[7] : 0,
                                               th ? d[8] : 0)
                                  : src[y * sw + x];
            unsigned a = p >> 24;
            bool has_alpha = fmt == FMT_ARGB8888 || fmt == FMT_ARGB1555;

            if (galpha) {
                a = has_alpha ? (a * (g + 1)) >> 8 : g;
            }
            o[px + x] = dpu_blend(p, a, o[px + x], mult);
        }
    }
    s->lay_site[l] = d[2] + (sh - 1) * stride;
    return true;
}

/* The write-back's 4x4 ordered dither: its thresholds, at 4 bits */
static const uint8_t dpu_dither[4][4] = {
    { 15, 1, 12, 2 }, { 7, 11, 4, 8 }, { 13, 3, 14, 0 }, { 5, 9, 6, 10 },
};

/*
 * A colour channel's value after the dither: the threshold, shifted down to
 * the @n bits the channel drops, is added with saturation and the low @n
 * bits cleared
 */
static unsigned dpu_dither_chan(unsigned v, unsigned x, unsigned y,
                                unsigned n)
{
    if (!n) {
        return v;
    }
    v = MIN(v + (dpu_dither[y & 3][x & 3] >> (4 - n)), 255);
    return v >> n << n;
}

/*
 * Bits each channel drops: the frame control's WB_Dither_DW codes (0, 2, 3,
 * 4 bits) for blue, green and red with the dither enabled, or, with only the
 * auto bit set, what the write-back format truncates (3, 2, 3 bits for
 * RGB565, 3 for RGB555, none for the 8-bit ones)
 */
static void dpu_dither_bits(uint32_t ctrl, unsigned fmt, unsigned n[3])
{
    static const uint8_t drop[4] = { 0, 2, 3, 4 };

    if (ctrl & (1 << 4)) {
        n[0] = drop[extract32(ctrl, 24, 2)];
        n[1] = drop[extract32(ctrl, 22, 2)];
        n[2] = drop[extract32(ctrl, 20, 2)];
    } else if (ctrl & (1 << 5)) {
        n[0] = n[2] = fmt == 1 || fmt == 2 ? 3 : 0;
        n[1] = fmt == 1 ? 2 : fmt == 2 ? 3 : 0;
    } else {
        n[0] = n[1] = n[2] = 0;
    }
}

/*
 * Write-back formats, from the frame control's bits 18:16: 0 ARGB8888, 1
 * RGB565, 2 ARGB1555 (the alpha bit always set), 3 YUV422 (the composed R
 * as Y, a pixel pair's mean G and B as the chroma: R0, G, R1, B in the
 * bytes of 32 bits, with no conversion), 4 one byte a pixel (the B), 6
 * RGB888 stored as 32 bits with 0xff in the top byte. Measured on a T40N.
 */
static unsigned dpu_wb_bytes_pp(unsigned fmt)
{
    switch (fmt) {
    case 1:
    case 2:
    case 3:
        return 2;
    case 4:
        return 1;
    default:
        return 4;
    }
}

static void dpu_writeback(IngenicDpuState *s, const uint32_t *pix,
                          unsigned w, unsigned h)
{
    unsigned fmt = extract32(s->frm[2], 16, 3);
    unsigned bpp = dpu_wb_bytes_pp(fmt);
    hwaddr stride = (hwaddr)s->frm[4] * bpp;
    g_autofree uint8_t *buf = g_malloc(w * 4);
    g_autofree uint32_t *row = g_new(uint32_t, w);
    unsigned n[3];

    if (fmt != 0 && fmt != 1 && fmt != 2 && fmt != 3 && fmt != 4 &&
        fmt != 6) {
        qemu_log_mask(LOG_UNIMP, "ingenic-dpu: writeback format %u\n", fmt);
        return;
    }
    dpu_dither_bits(s->frm[2], fmt, n);
    for (unsigned j = 0; j < h; j++) {
        for (unsigned i = 0; i < w; i++) {
            uint32_t p = pix[j * w + i];
            unsigned r = dpu_dither_chan(extract32(p, 16, 8), i, j, n[0]);
            unsigned g = dpu_dither_chan(extract32(p, 8, 8), i, j, n[1]);
            unsigned b = dpu_dither_chan(p & 0xff, i, j, n[2]);

            row[i] = (p & 0xff000000) | r << 16 | g << 8 | b;
        }
        for (unsigned i = 0; i < w; i++) {
            uint32_t p = row[i];
            unsigned r = extract32(p, 16, 8), g = extract32(p, 8, 8);
            unsigned b = p & 0xff;

            switch (fmt) {
            case 1:
                stw_le_p(buf + 2 * i, (r >> 3) << 11 | (g >> 2) << 5 | b >> 3);
                break;
            case 2:
                stw_le_p(buf + 2 * i,
                         0x8000 | (r >> 3) << 10 | (g >> 3) << 5 | b >> 3);
                break;
            case 3: {
                unsigned i0 = i & ~1u, i1 = MIN(i0 + 1, w - 1);
                unsigned ga = extract32(row[i0], 8, 8) +
                              extract32(row[i1], 8, 8);
                unsigned ba = (row[i0] & 0xff) + (row[i1] & 0xff);

                buf[2 * i] = r;
                buf[2 * i + 1] = (i & 1 ? ba : ga) / 2;
                break;
            }
            case 4:
                buf[i] = b;
                break;
            case 6:
                stl_le_p(buf + 4 * i, 0xff000000 | (p & 0xffffff));
                break;
            default:
                stl_le_p(buf + 4 * i, p);
                break;
            }
        }
        dma_memory_write(&address_space_memory, s->frm[3] + j * stride, buf,
                         w * bpp, MEMTXATTRS_UNSPECIFIED);
    }
    s->wdma_site = s->frm[3] + (h - 1) * stride;
}

/*
 * Compose the frame in s->frm. A layer's TLB miss leaves the composer
 * working, as a T40NN does, until TLBC's RETRY runs this again, and the
 * frame ends with CMP_W_SLOW then.
 */
static void dpu_cmp_run(IngenicDpuState *s, bool retry)
{
    unsigned w = extract32(s->frm[1], 0, 12), h = extract32(s->frm[1], 16, 12);
    uint8_t slot[INGENIC_DPU_LAYERS] = { 0, 1, 2, 3 };
    g_autofree uint32_t *pix = NULL;
    uint32_t ends = 0;

    /* A later layer with the same Z takes the slot, enabled or not */
    for (unsigned l = 0; l < INGENIC_DPU_LAYERS; l++) {
        slot[extract32(s->frm[9], 8 + 2 * l, 2)] = l;
    }
    pix = g_new0(uint32_t, (size_t)MAX(w, 1) * MAX(h, 1));
    s->st |= ST_CMP_START;
    for (unsigned z = 0; z < INGENIC_DPU_LAYERS; z++) {
        unsigned l = slot[z];

        if (s->frm[9] & (1 << (4 + l))) {
            dpu_read_desc(s->frm[5 + l], s->lay[l], INGENIC_DPU_LAY_WORDS);
            if (!dpu_compose_layer(s, l, pix, w, h)) {
                s->stalled = true;
                s->st |= s->wb ? ST_WRBK_WORKING : 0;
                return;
            }
            ends |= ST_LAY0_END << l;
        }
    }
    s->stalled = false;
    s->st |= ends | ST_CMP_END | (retry ? ST_CMP_W_SLOW : 0);
    if (s->direct) {
        s->st |= ST_DIRECT_WORKING;
        dpu_show(s->con, pix, w, h);
    }
    if (s->wb && w && h) {
        s->st |= ST_WRBK_WORKING;
        dpu_writeback(s, pix, w, h);
    }
    dpu_update_irq(s);
    dpu_arm(s, w, h);
}

/* The composer takes the descriptor at FRM_CFG_ADDR and composes it */
static void dpu_cmp_frame(IngenicDpuState *s)
{
    s->frm_site = s->frm_addr;
    dpu_read_desc(s->frm_addr, s->frm, INGENIC_DPU_FRM_WORDS);
    s->frm_addr = s->frm[0];
    s->direct = s->frm[2] & FRM_DIRECT;
    s->wb = s->frm[2] & FRM_WB;
    s->eod_mask = s->frm[10] & ST_DISP_END;
    s->desc_mask = s->frm[10] & CMP_DESC_MASKS;
    /* A writeback-only frame raises no CMP_START, as a T40N has it */
    if (!s->direct) {
        s->desc_mask &= ~ST_CMP_START;
    }
    if (s->frm[2] & FRM_CHANGE_2_SRD && s->wb) {
        s->to_srd = true;
    }
    for (unsigned l = 0; l < INGENIC_DPU_LAYERS; l++) {
        s->lay_cnt[l] = 0;
    }
    s->frm_cnt = 0;
    dpu_cmp_run(s, false);
}

/* The display's size for the simple read channel, which has none */
static void dpu_disp_size(IngenicDpuState *s, unsigned *w, unsigned *h)
{
    if (dpu_disp_if(s) == DISP_IF_TFT) {
        *w = extract32(s->disp.tft[2], 0, 12) -
             MIN(extract32(s->disp.tft[2], 16, 12),
                 extract32(s->disp.tft[2], 0, 12));
        *h = extract32(s->disp.tft[3], 0, 12) -
             MIN(extract32(s->disp.tft[3], 16, 12),
                 extract32(s->disp.tft[3], 0, 12));
    } else {
        *w = extract32(s->disp.slcd[3], 0, 16);
        *h = extract32(s->disp.slcd[3], 16, 16);
    }
    *w = MIN(*w, 4096);
    *h = MIN(*h, 4096);
}

static void dpu_srd_frame(IngenicDpuState *s)
{
    unsigned w, h, fmt;
    hwaddr stride;
    g_autofree uint32_t *pix = NULL;

    s->srd_site = s->srd_addr;
    dpu_read_desc(s->srd_addr, s->srd, INGENIC_DPU_SRD_WORDS);
    s->srd_addr = s->srd[0];
    s->srd_cnt = 0;
    s->eod_mask = s->srd[4] & ST_DISP_END;
    s->desc_mask = s->srd[4] & SRD_DESC_MASKS;
    if (s->srd[3] & SRD_CHANGE_2_CMP) {
        s->to_cmp = true;
    }
    fmt = extract32(s->srd[3], 19, 4);
    stride = (hwaddr)extract32(s->srd[2], 0, 13) * dpu_bytes_pp(fmt);
    dpu_disp_size(s, &w, &h);
    s->st |= ST_SRD_START | ST_SRD_WORKING;
    if (w && h) {
        pix = g_new(uint32_t, (size_t)w * h);
        for (unsigned j = 0; j < h; j++) {
            dpu_fetch_line(pix + j * w, fmt, extract32(s->srd[3], 16, 3),
                           s->srd[1] + j * stride, 0, w, s->csc[0], NULL, 0);
        }
        dpu_show(s->con, pix, w, h);
        s->rdma_site = s->srd[1] + (h - 1) * stride;
    }
    s->st |= ST_SRD_END;
    dpu_update_irq(s);
    dpu_arm(s, w, h);
}

static void dpu_tft_off(IngenicDpuState *s)
{
    s->st &= ~(ST_DIRECT_WORKING | ST_WRBK_WORKING | ST_SRD_WORKING);
}

static void dpu_stop(IngenicDpuState *s)
{
    timer_del(s->frame);
    s->cmp_on = s->srd_on = false;
    s->cmp_gen_stop = s->srd_gen_stop = false;
    s->to_srd = s->to_cmp = false;
    s->stalled = s->stop_wait = false;
    dpu_tft_off(s);
    s->st &= ~ST_WORKING;
}

static void dpu_frame_end(void *opaque)
{
    IngenicDpuState *s = opaque;

    if (s->cmp_on) {
        if (s->direct) {
            s->st |= ST_DISP_END;
        }
        if (s->wb) {
            s->st = (s->st & ~ST_WRBK_WORKING) | ST_WDMA_END;
        }
        if (s->cmp_gen_stop) {
            dpu_stop(s);
            s->st |= ST_STOP_CMP_ACK;
        } else if (s->frm[2] & FRM_STOP) {
            dpu_stop(s);
        } else if (s->to_srd) {
            s->cmp_on = false;
            s->to_srd = false;
            s->srd_on = true;
            s->st &= ~(ST_DIRECT_WORKING | ST_WRBK_WORKING);
            dpu_srd_frame(s);
        } else {
            dpu_cmp_frame(s);
        }
    } else if (s->srd_on) {
        s->st |= ST_DISP_END;
        if (s->srd_gen_stop) {
            dpu_stop(s);
            s->st |= ST_STOP_SRD_ACK;
        } else if (s->srd[3] & SRD_CHAIN_END) {
            dpu_stop(s);
        } else if (s->to_cmp) {
            s->srd_on = false;
            s->to_cmp = false;
            s->cmp_on = true;
            s->st &= ~ST_SRD_WORKING;
            dpu_cmp_frame(s);
        } else {
            dpu_srd_frame(s);
        }
    }
    dpu_update_irq(s);
}

/* TLBC: bits 3:0 invalidate a channel's translations, 7:4 retry its miss */
static void dpu_tlb_trigger(IngenicDpuState *s, uint32_t v)
{
    for (unsigned ch = 0; ch < INGENIC_DPU_LAYERS; ch++) {
        if (v >> ch & 1) {
            memset(s->tlb_ent[ch], 0, sizeof(s->tlb_ent[ch]));
            s->tlb_neg[ch] = 0;
            s->tlb_pend &= ~(1 << ch);
        }
        if (v >> (4 + ch) & 1) {
            s->tlb_neg[ch] = 0;
        }
    }
    if (v & 0xf0 && s->stop_wait) {
        dpu_stop(s);
    } else if (v & 0xf0 && s->stalled) {
        dpu_cmp_run(s, true);
        if (!s->stalled) {
            s->tlb_pend &= ~(v >> 4);
        }
    }
}

static void dpu_ctrl(IngenicDpuState *s, uint32_t v)
{
    if (v & CTRL_DES_CNT_RST) {
        s->frm_cnt = s->srd_cnt = 0;
        memset(s->lay_cnt, 0, sizeof(s->lay_cnt));
    }
    if (v & CTRL_QCK_STP_CMP && s->stalled) {
        /*
         * A composer waiting on the TLB takes the stop, with WDMA_END if
         * it writes back, but stays working until a retry or the
         * channel's TLB turned off
         */
        s->stop_wait = true;
        if (s->wb) {
            s->st = (s->st & ~ST_WRBK_WORKING) | ST_WDMA_END;
        }
    } else if ((v & CTRL_QCK_STP_CMP && s->cmp_on) ||
               (v & CTRL_QCK_STP_SRD && s->srd_on)) {
        dpu_stop(s);
    }
    if (v & CTRL_GEN_STP_CMP && s->cmp_on) {
        s->cmp_gen_stop = true;
    }
    if (v & CTRL_GEN_STP_SRD && s->srd_on) {
        s->srd_gen_stop = true;
    }
    dpu_update_irq(s);
}

/* On silicon this read hangs the bus; here it is logged and answered */
static void dpu_disp_check(Clock *clk, hwaddr off)
{
    if (off >= DC_DISP_COM && !clock_get_hz(clk)) {
        qemu_log_mask(LOG_GUEST_ERROR, "ingenic-dpu: read of 0x%04" HWADDR_PRIx
                      " with the pixel clock stopped\n", off);
    }
}

/* The display interface registers, from 0x8000: @t40 has the TFT */
static uint32_t disp_read(const IngenicDpuDisp *d, hwaddr off, bool tft_on,
                          bool t40)
{
    if (off >= DC_TFT && off < DC_TFT + 0x14) {
        return t40 ? d->tft[(off - DC_TFT) / 4] : 0;
    }
    if (off == DC_TFT_ST) {
        return t40 && tft_on ? 2 : 0;
    }
    if (off >= DC_SLCD && off < DC_SLCD + 0x18) {
        return d->slcd[(off - DC_SLCD) / 4];
    }
    if (off == DC_SLCD_ST) {
        return 0;
    }
    if ((off >= DC_DISP_COM && off < DC_DISP_COM + 0x1000) ||
        off >= 0xb000) {
        return d->disp_com;
    }
    return 0;
}

static void disp_write(IngenicDpuDisp *d, hwaddr off, uint32_t v, bool t40)
{
    static const uint32_t tft_mask[5] = {
        0x0fff0fff, 0x0fff0fff, 0x0fff0fff, 0x0fff0fff, 0x003f0707,
    };
    static const uint32_t slcd_mask[6] = {
        0x0fff0fff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffff, 0xe0ffffff,
    };

    if (off == DC_DISP_COM) {
        d->disp_com = v & (t40 ? 0x3f00f3 : 0x3f0007);
    } else if (t40 && off >= DC_TFT && off < DC_TFT + 0x14) {
        d->tft[(off - DC_TFT) / 4] = v & tft_mask[(off - DC_TFT) / 4];
    } else if (off >= DC_SLCD && off < DC_SLCD + 0x18) {
        unsigned i = (off - DC_SLCD) / 4;

        /* The T41's SLCD_CFG keeps bit 31 (continuous frames) instead */
        d->slcd[i] = v & (i == 0 && !t40 ? 0x8fff0f7f : slcd_mask[i]);
    }
}

static void disp_reset(IngenicDpuDisp *d, bool t40)
{
    memset(d, 0, sizeof(*d));
    d->disp_com = t40 ? 0xe0 : 1;
    d->slcd[0] = 0x01540d20;
    d->slcd[5] = 0x80000000;
}

static uint32_t dpu_des_word(const uint32_t *w, uint32_t *cnt, unsigned n)
{
    uint32_t v = w[*cnt % n];

    *cnt = (*cnt + 1) % n;
    return v;
}

static uint64_t ingenic_dpu_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicDpuState *s = opaque;

    if (!s->gate_open) {
        return 0;
    }
    dpu_disp_check(s->clk, off);
    if (off >= DC_DISP_COM) {
        return disp_read(&s->disp, off, dpu_disp_if(s) == DISP_IF_TFT &&
                         s->st & (ST_DIRECT_WORKING | ST_SRD_WORKING), true);
    }
    if (off >= DC_PCFG && off < DC_PCFG + 0x20) {
        return s->pcfg[(off - DC_PCFG) / 4];
    }
    if (off >= DC_CSC && off < DC_CSC + 0x20) {
        return s->csc[(off - DC_CSC) / 16][(off & 0xf) / 4];
    }
    if (off >= DC_LAY0_DES && off < DC_RDMA_DES) {
        unsigned l = (off - DC_LAY0_DES) / 4;

        return dpu_des_word(s->lay[l], &s->lay_cnt[l], INGENIC_DPU_LAY_WORDS);
    }
    if (off >= DC_LAY0_SITE && off < DC_RDMA_SITE) {
        return s->lay_site[(off - DC_LAY0_SITE) / 4];
    }
    switch (off) {
    case DC_FRM_CFG_ADDR:
        return s->frm_addr;
    case DC_FRM_CFG_CTRL:
        return s->to_srd ? 2 : 0;
    case DC_SRD_CHAIN_ADDR:
        return s->srd_addr;
    case DC_ST:
        return s->st;
    case DC_INTC:
        return s->intc;
    case DC_INT_FLAG:
        return dpu_int_flag(s);
    case DC_COM_CFG:
        return s->com_cfg;
    case DC_FRM_DES:
        return dpu_des_word(s->frm, &s->frm_cnt, INGENIC_DPU_FRM_WORDS);
    case DC_RDMA_DES:
        return dpu_des_word(s->srd, &s->srd_cnt, INGENIC_DPU_SRD_WORDS);
    case DC_FRM_SITE:
        return s->frm_site;
    case DC_RDMA_CHAIN_SITE:
        return s->srd_site;
    case DC_WDMA_SITE:
        return s->wdma_site;
    case DC_RDMA_SITE:
        return s->rdma_site;
    case DC_TLB_GLBC:
        return s->tlb_glbc;
    case DC_TLB_TLBA:
        return s->tlba;
    case DC_TLB_VPN ... DC_TLB_VPN + 0xc:
        return s->tlb_vpn[(off - DC_TLB_VPN) / 4];
    case DC_TLB_TLBV:
        return s->tlbv;
    case DC_TLB_STAT:
        return s->tlb_stat;
    default:
        return 0;
    }
}

static void ingenic_dpu_write(void *opaque, hwaddr off, uint64_t value,
                              unsigned size)
{
    IngenicDpuState *s = opaque;
    uint32_t v = value;
    static const uint32_t pcfg_mask[8] = {
        0x7, 0x7, 0x07ffffff, 0x07ffffff, 0x7ff, 0x01ffffff, 0x01ffffff,
        0x01ffffff,
    };
    static const uint32_t csc_mask[4] = {
        0x07ff07ff, 0x03ff01ff, 0x00000fff, 0x00ff001f,
    };

    if (!s->gate_open) {
        return;
    }
    if (off >= DC_DISP_COM) {
        disp_write(&s->disp, off, v, true);
        return;
    }
    if (off >= DC_PCFG && off < DC_PCFG + 0x20) {
        s->pcfg[(off - DC_PCFG) / 4] = v & pcfg_mask[(off - DC_PCFG) / 4];
        return;
    }
    if (off >= DC_CSC && off < DC_CSC + 0x20) {
        s->csc[(off - DC_CSC) / 16][(off & 0xf) / 4] = v & csc_mask[(off & 0xf)
                                                                   / 4];
        return;
    }
    switch (off) {
    case DC_FRM_CFG_ADDR:
        s->frm_addr = v;
        break;
    case DC_FRM_CFG_CTRL:
        if (v & 2 && s->cmp_on) {
            s->to_srd = true;
        }
        /* A start while the composer runs only lets it go on */
        if (v & 1 && !s->cmp_on && !s->srd_on) {
            s->cmp_on = true;
            s->st |= ST_WORKING;
            dpu_cmp_frame(s);
        }
        break;
    case DC_SRD_CHAIN_ADDR:
        s->srd_addr = v;
        break;
    case DC_SRD_CHAIN_CTRL:
        if (v & 2 && s->srd_on) {
            s->to_cmp = true;
        }
        if (v & 1 && !s->cmp_on && !s->srd_on) {
            s->srd_on = true;
            s->st |= ST_WORKING;
            dpu_srd_frame(s);
        }
        break;
    case DC_CTRL:
        dpu_ctrl(s, v);
        break;
    case DC_CLR_ST:
        s->st &= ~(v & ST_EVENTS);
        dpu_update_irq(s);
        break;
    case DC_INTC:
        s->intc = v & ST_EVENTS;
        dpu_update_irq(s);
        break;
    case DC_COM_CFG:
        s->com_cfg = v & 0x00ff00fe;
        break;
    case DC_TLB_GLBC:
        if (s->stop_wait && s->tlb_glbc & ~v & 0xf) {
            dpu_stop(s);
        }
        s->tlb_glbc = v & 0xf0f;
        break;
    case DC_TLB_TLBC:
        dpu_tlb_trigger(s, v);
        break;
    case DC_TLB_TLBA:
        s->tlba = v & 0xfffff000;
        break;
    case DC_TLB_VPN ... DC_TLB_VPN + 0xc:
        s->tlb_vpn[(off - DC_TLB_VPN) / 4] = v & 0xfffff000;
        break;
    case DC_TLB_TLBV:
        s->tlbv = v & 0x8fff0fff;
        break;
    case DC_TLB_STAT:
        s->tlb_stat = v & 0xf;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_dpu_ops = {
    .read = ingenic_dpu_read,
    .write = ingenic_dpu_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_dpu_gate(void *opaque, int n, int level)
{
    IngenicDpuState *s = opaque;

    s->gate_open = level;
}

static const GraphicHwOps ingenic_dpu_gfx_ops = {
};

/* As a T40N reads them at U-Boot, its gate opened */
static void ingenic_dpu_reset_hold(Object *obj, ResetType type)
{
    IngenicDpuState *s = INGENIC_DPU(obj);
    static const uint32_t pcfg[8] = {
        6, 6, 0x03190842, 0x00c04010, 0xb, 0x00600400, 0x01000400,
        0x01e00400,
    };

    dpu_stop(s);
    s->frm_addr = s->srd_addr = 0;
    s->st = s->intc = 0;
    s->com_cfg = 0x00cf000c;
    memcpy(s->pcfg, pcfg, sizeof(pcfg));
    s->tlb_glbc = s->tlba = s->tlbv = s->tlb_stat = 0;
    memset(s->tlb_vpn, 0, sizeof(s->tlb_vpn));
    memset(s->tlb_ent, 0, sizeof(s->tlb_ent));
    memset(s->tlb_neg, 0, sizeof(s->tlb_neg));
    s->tlb_pend = 0;
    for (int i = 0; i < 2; i++) {
        s->csc[i][0] = 0x066204a8;
        s->csc[i][1] = 0x03410190;
        s->csc[i][2] = 0x00000812;
        s->csc[i][3] = 0x00800000;
    }
    disp_reset(&s->disp, true);
    s->eod_mask = s->desc_mask = 0;
    memset(s->frm, 0, sizeof(s->frm));
    memset(s->lay, 0, sizeof(s->lay));
    memset(s->srd, 0, sizeof(s->srd));
    s->frm_cnt = s->srd_cnt = 0;
    memset(s->lay_cnt, 0, sizeof(s->lay_cnt));
    s->frm_site = s->srd_site = s->wdma_site = s->rdma_site = 0;
    memset(s->lay_site, 0, sizeof(s->lay_site));
    qemu_set_irq(s->irq, 0);
}

static void ingenic_dpu_realize(DeviceState *dev, Error **errp)
{
    IngenicDpuState *s = INGENIC_DPU(dev);

    s->con = qemu_graphic_console_create(dev, 0, &ingenic_dpu_gfx_ops, s);
    qemu_console_resize(s->con, 640, 480);
}

static void ingenic_dpu_init(Object *obj)
{
    IngenicDpuState *s = INGENIC_DPU(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_dpu_ops, s, "ingenic-dpu",
                          0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", NULL, NULL, 0);
    qdev_init_gpio_in_named(DEVICE(obj), ingenic_dpu_gate, "gate", 1);
    s->gate_open = true;
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, dpu_frame_end, s);
}

static const VMStateDescription vmstate_ingenic_dpu_disp = {
    .name = "ingenic-dpu-disp",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(disp_com, IngenicDpuDisp),
        VMSTATE_UINT32_ARRAY(tft, IngenicDpuDisp, 5),
        VMSTATE_UINT32_ARRAY(slcd, IngenicDpuDisp, 6),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ingenic_dpu = {
    .name = "ingenic-dpu",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(gate_open, IngenicDpuState),
        VMSTATE_UINT32(frm_addr, IngenicDpuState),
        VMSTATE_UINT32(srd_addr, IngenicDpuState),
        VMSTATE_UINT32(st, IngenicDpuState),
        VMSTATE_UINT32(intc, IngenicDpuState),
        VMSTATE_UINT32(com_cfg, IngenicDpuState),
        VMSTATE_UINT32_ARRAY(pcfg, IngenicDpuState, 8),
        VMSTATE_UINT32(tlb_glbc, IngenicDpuState),
        VMSTATE_UINT32(tlba, IngenicDpuState),
        VMSTATE_UINT32(tlbv, IngenicDpuState),
        VMSTATE_UINT32(tlb_stat, IngenicDpuState),
        VMSTATE_UINT32_ARRAY_V(tlb_vpn, IngenicDpuState, INGENIC_DPU_LAYERS, 2),
        VMSTATE_UINT32_2DARRAY_V(tlb_ent, IngenicDpuState, INGENIC_DPU_LAYERS,
                                 INGENIC_DPU_TLB_ENTRIES * 2, 2),
        VMSTATE_UINT8_ARRAY_V(tlb_rr, IngenicDpuState, INGENIC_DPU_LAYERS, 2),
        VMSTATE_UINT32_ARRAY_V(tlb_neg, IngenicDpuState, INGENIC_DPU_LAYERS,
                               2),
        VMSTATE_UINT8_V(tlb_pend, IngenicDpuState, 2),
        VMSTATE_BOOL_V(stalled, IngenicDpuState, 2),
        VMSTATE_BOOL_V(stop_wait, IngenicDpuState, 2),
        VMSTATE_UINT32_2DARRAY(csc, IngenicDpuState, 2, 4),
        VMSTATE_STRUCT(disp, IngenicDpuState, 1, vmstate_ingenic_dpu_disp,
                       IngenicDpuDisp),
        VMSTATE_BOOL(cmp_on, IngenicDpuState),
        VMSTATE_BOOL(srd_on, IngenicDpuState),
        VMSTATE_BOOL(cmp_gen_stop, IngenicDpuState),
        VMSTATE_BOOL(srd_gen_stop, IngenicDpuState),
        VMSTATE_BOOL(to_srd, IngenicDpuState),
        VMSTATE_BOOL(to_cmp, IngenicDpuState),
        VMSTATE_BOOL(direct, IngenicDpuState),
        VMSTATE_BOOL(wb, IngenicDpuState),
        VMSTATE_UINT32(eod_mask, IngenicDpuState),
        VMSTATE_UINT32(desc_mask, IngenicDpuState),
        VMSTATE_UINT32_ARRAY(frm, IngenicDpuState, INGENIC_DPU_FRM_WORDS),
        VMSTATE_UINT32_2DARRAY(lay, IngenicDpuState, INGENIC_DPU_LAYERS,
                               INGENIC_DPU_LAY_WORDS),
        VMSTATE_UINT32_ARRAY(srd, IngenicDpuState, INGENIC_DPU_SRD_WORDS),
        VMSTATE_UINT32(frm_cnt, IngenicDpuState),
        VMSTATE_UINT32_ARRAY(lay_cnt, IngenicDpuState, INGENIC_DPU_LAYERS),
        VMSTATE_UINT32(srd_cnt, IngenicDpuState),
        VMSTATE_UINT32(frm_site, IngenicDpuState),
        VMSTATE_UINT32(srd_site, IngenicDpuState),
        VMSTATE_UINT32(wdma_site, IngenicDpuState),
        VMSTATE_UINT32(rdma_site, IngenicDpuState),
        VMSTATE_UINT32_ARRAY(lay_site, IngenicDpuState, INGENIC_DPU_LAYERS),
        VMSTATE_TIMER_PTR(frame, IngenicDpuState),
        VMSTATE_CLOCK(clk, IngenicDpuState),
        VMSTATE_END_OF_LIST()
    }
};

static void ingenic_dpu_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_dpu_realize;
    dc->vmsd = &vmstate_ingenic_dpu;
    rc->phases.hold = ingenic_dpu_reset_hold;
}

static const TypeInfo ingenic_dpu_type_info = {
    .name = TYPE_INGENIC_DPU,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicDpuState),
    .instance_init = ingenic_dpu_init,
    .class_init = ingenic_dpu_class_init,
};

/* ---- the T41's LCDC tiny ---- */

#define LCDC_CHAIN_ADDR     0x0000
#define LCDC_GLB_CFG        0x0004
#define LCDC_CSC            0x0100
#define LCDC_CTRL           0x1000
#define LCDC_ST             0x1004
#define LCDC_CSR            0x1008
#define LCDC_INTC           0x100c
#define LCDC_PCFG           0x1010
#define LCDC_INT_FLAG       0x101c
#define LCDC_CHAIN_SITE     0x2000
#define LCDC_DMA_SITE       0x2004
#define LCDC_DES_READ       0x3000

#define LCTRL_START         (1 << 0)
#define LCTRL_QCK_STOP      (1 << 1)
#define LCTRL_DES_CNT_RST   (1 << 2)
#define LCTRL_GEN_STOP      (1 << 3)
#define LCTRL_SLCD_START    (1 << 4)

#define LST_WORKING         (1 << 0)
#define LST_FRM_END         (1 << 1)
#define LST_DMA_END         (1 << 2)
#define LST_GEN_ACK         (1 << 5)
#define LST_QCK_ACK         (1 << 6)
#define LST_EVENTS          0x66

#define SLCD_CONTINUOUS     (1u << 31)


/* GLB_CFG's colour orders, in the composer's numbering */
static const uint8_t lcdc_order[8] = { 0, 1, 2, 4, 3, 5, 0, 0 };

static unsigned lcdc_fmt(IngenicLcdcState *s)
{
    static const uint8_t fmt[8] = {
        FMT_RGB555, FMT_RGB565, FMT_RGB888, FMT_RGB888, FMT_NV12, FMT_NV21,
        FMT_RGB888, FMT_RGB888,
    };

    return fmt[extract32(s->glb_cfg, 16, 3)];
}

/*
 * The frame size: the T23N, T31X, T41LQ and T41NQ keep 10 bits of each,
 * count a height of 0 as 1024 lines and never end a frame of width 0 or
 * 1024 (w comes back 0)
 */
static void lcdc_size(IngenicLcdcState *s, unsigned *w, unsigned *h)
{
    *w = extract32(s->disp.slcd[3], 0, 10);
    *h = extract32(s->disp.slcd[3], 16, 10);
    *h = *h ? *h : 1024;
}

/* The last 64-byte burst of the frame's last line, where DMA_SITE ends */
static hwaddr lcdc_last_burst(IngenicLcdcState *s, unsigned w, unsigned h)
{
    hwaddr stride = (hwaddr)s->desc[2] * dpu_bytes_pp(lcdc_fmt(s));
    hwaddr line = (hwaddr)w * dpu_bytes_pp(lcdc_fmt(s));

    return s->desc[1] + (h - 1) * stride + ((line - 1) & ~63ull);
}

static uint32_t lcdc_int_flag(IngenicLcdcState *s)
{
    uint32_t f = s->st & s->intc;

    if (!(s->desc[4] & LST_FRM_END)) {
        f &= ~LST_FRM_END;
    }
    return f;
}

static void lcdc_update_irq(IngenicLcdcState *s)
{
    qemu_set_irq(s->irq, !!lcdc_int_flag(s));
}

/*
 * What the DMA reads ahead of the panel. A T31X took 1984 bytes of 4-byte
 * pixels and 1088 of 2-byte ones, as a single line; a T41LQ and a T41NQ
 * 1920 and 1024, within a few pixels. Taller frames of the same size did
 * not all fit, narrow ones least.
 */
static uint64_t lcdc_fifo(IngenicLcdcState *s)
{
    bool wide = dpu_bytes_pp(lcdc_fmt(s)) == 4;

    if (s->xburst1) {
        return wide ? 1984 : 1088;
    }
    return wide ? 1920 : 1024;
}

/*
 * Where a frame that does not fit stalls the DMA, as a T41LQ's DMA_SITE
 * shows: the last 64-byte burst of the FIFO's worth, counted line by line
 * in whole bursts
 */
static hwaddr lcdc_stall_site(IngenicLcdcState *s, unsigned w)
{
    unsigned bpp = dpu_bytes_pp(lcdc_fmt(s));
    unsigned per = DIV_ROUND_UP(w * bpp, 64);
    unsigned n = lcdc_fifo(s) / 64 - 1;

    return s->desc[1] + (hwaddr)(n / per) * s->desc[2] * bpp +
           (n % per) * 64;
}

/*
 * START, or the chain going on: the descriptor at CHAIN_ADDR, and the DMA
 * filling its FIFO
 */
static void lcdc_fetch(IngenicLcdcState *s, bool start)
{
    unsigned w, h;

    s->chain_site = s->chain_addr;
    dpu_read_desc(s->chain_addr, s->desc, INGENIC_LCDC_DESC_WORDS);
    s->chain_addr = s->desc[0];
    s->des_cnt = 0;
    s->fetched = true;
    s->st |= s->ack_pending;
    s->ack_pending = 0;
    lcdc_size(s, &w, &h);
    s->fit = (uint64_t)w * h * dpu_bytes_pp(lcdc_fmt(s)) <= lcdc_fifo(s);
    if (s->fit) {
        s->st |= LST_DMA_END;
        if (w) {
            s->dma_site = lcdc_last_burst(s, w, h);
        }
    } else {
        s->st |= LST_WORKING;
        if (start && !s->xburst1 && w) {
            s->dma_site = lcdc_stall_site(s, w);
        }
    }
}

static void lcdc_arm(IngenicLcdcState *s)
{
    unsigned w, h;
    int64_t ns;

    lcdc_size(s, &w, &h);
    if (!w) {
        return;
    }
    ns = clocks_ns(s->clk, MAX(slcd_frame_clocks(&s->disp, (uint64_t)w * h,
                                                 1), 1));
    if (ns) {
        timer_mod(s->frame, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
    }
}

static void lcdc_send(IngenicLcdcState *s)
{
    s->sending = true;
    s->st |= LST_WORKING;
    lcdc_arm(s);
}

/* A frame sent with the pixel clock stopped goes on when it runs */
static void lcdc_clk_update(void *opaque, ClockEvent event)
{
    IngenicLcdcState *s = opaque;

    if (s->sending && !timer_pending(s->frame)) {
        lcdc_arm(s);
    }
}

static void lcdc_stop(IngenicLcdcState *s)
{
    timer_del(s->frame);
    s->fetched = s->sending = s->gen_stop = false;
    s->st &= ~LST_WORKING;
    s->dma_site = 0;
}

static void lcdc_frame_end(void *opaque)
{
    IngenicLcdcState *s = opaque;
    unsigned w, h, fmt = lcdc_fmt(s);
    hwaddr stride;
    g_autofree uint32_t *pix = NULL;

    lcdc_size(s, &w, &h);
    stride = (hwaddr)s->desc[2] * dpu_bytes_pp(fmt);
    if (w && h) {
        pix = g_new(uint32_t, (size_t)w * h);
        for (unsigned j = 0; j < h; j++) {
            dpu_fetch_line(pix + j * w, fmt,
                           lcdc_order[extract32(s->glb_cfg, 19, 3)],
                           s->desc[1] + j * stride,
                           s->desc[5] + (j / 2) * (hwaddr)s->desc[6], w,
                           s->csc, NULL, 0);
        }
        dpu_show(s->con, pix, w, h);
        s->dma_site = lcdc_last_burst(s, w, h);
    }
    /* A frame that fitted the FIFO had its DMA_END at START */
    s->st |= LST_FRM_END | (s->fit ? 0 : LST_DMA_END);
    s->sending = false;
    if (s->gen_stop) {
        lcdc_stop(s);
        s->st |= LST_GEN_ACK;
    } else if (s->desc[3] & 1) {
        s->fetched = false;
        s->st &= ~LST_WORKING;
    } else {
        lcdc_fetch(s, false);
        if (s->disp.slcd[0] & SLCD_CONTINUOUS) {
            lcdc_send(s);
        }
    }
    lcdc_update_irq(s);
}

static void lcdc_ctrl(IngenicLcdcState *s, uint32_t v)
{
    if (v & LCTRL_DES_CNT_RST) {
        s->des_cnt = 0;
    }
    if (v & LCTRL_QCK_STOP) {
        lcdc_stop(s);
        s->st |= LST_QCK_ACK;
        s->ack_pending |= LST_QCK_ACK;
    }
    if (v & LCTRL_GEN_STOP) {
        if (s->sending) {
            s->gen_stop = true;
        } else {
            lcdc_stop(s);
            s->st |= LST_GEN_ACK;
            s->ack_pending |= LST_GEN_ACK;
        }
    }
    if (v & LCTRL_START && !s->fetched) {
        lcdc_fetch(s, true);
    }
    if (v & LCTRL_SLCD_START && s->fetched && !s->sending) {
        lcdc_send(s);
    }
    lcdc_update_irq(s);
}

static uint64_t ingenic_lcdc_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicLcdcState *s = opaque;

    if (!s->gate_open) {
        return 0;
    }
    if (!s->xburst1) {
        dpu_disp_check(s->clk, off);
    }
    if (s->quirk) {
        if (off >= DC_SLCD && off < DC_SLCD + 0x1000) {
            s->arm = false;
        } else if (off >= DC_DISP_COM) {
            s->arm = true;
        }
    }
    if (off == DC_SLCD + 0x1c) {
        return s->slcd_1c;
    }
    /* With the pixel clock stopped SLCD_CFG reads 0, and nothing hangs */
    if (s->xburst1 && off == DC_SLCD && !clock_get_hz(s->clk)) {
        return 0;
    }
    if (off >= DC_DISP_COM) {
        return disp_read(&s->disp, off, false, false);
    }
    if (off >= LCDC_CSC && off < LCDC_CSC + 0x10) {
        return s->csc[(off - LCDC_CSC) / 4];
    }
    switch (off) {
    case LCDC_CHAIN_ADDR:
        return s->chain_addr;
    case LCDC_GLB_CFG:
        return s->glb_cfg;
    case LCDC_ST:
        return s->st;
    case LCDC_INTC:
        return s->intc;
    case LCDC_PCFG:
        return s->pcfg;
    case LCDC_INT_FLAG:
        return lcdc_int_flag(s);
    case LCDC_CHAIN_SITE:
        return s->chain_site;
    case LCDC_DMA_SITE:
        return s->dma_site;
    case LCDC_DES_READ: {
        /* The interrupt word reads as INTC, not the descriptor's mask */
        unsigned i = s->des_cnt;

        s->des_cnt = (i + 1) % INGENIC_LCDC_DESC_WORDS;
        return i == 4 ? s->intc : s->desc[i];
    }
    default:
        return 0;
    }
}

static void ingenic_lcdc_write(void *opaque, hwaddr off, uint64_t value,
                               unsigned size)
{
    IngenicLcdcState *s = opaque;
    uint32_t v = value;
    static const uint32_t csc_mask[4] = {
        0x07ff07ff, 0x03ff01ff, 0x00000fff, 0x00ff001f,
    };

    if (!s->gate_open) {
        return;
    }
    if (off == DC_SLCD + 0x1c) {
        s->arm = false;
        s->slcd_1c = v & 1;
        return;
    }
    if (s->quirk && off >= DC_SLCD && off < DC_SLCD + 0x1000) {
        /*
         * A T31X loses the first write to the SLCD registers after a read
         * of the display registers (0x8000 to 0x9fff, 0xb000 up), or after
         * a reset: all of WR_DUTY, TIMING, FRM_SIZE, SLOW_TIME and REG_IF,
         * and FMT_EN of SLCD_CFG, whose other bits sometimes go too. A read
         * of an SLCD register, or any write there, ends it.
         */
        bool lost = s->arm;

        s->arm = false;
        if (lost && off == DC_SLCD) {
            v &= ~(1u << 26);
        } else if (lost && off > DC_SLCD && off <= DC_SLCD + 0x14) {
            return;
        }
    }
    if (off >= DC_DISP_COM) {
        disp_write(&s->disp, off, v, false);
        return;
    }
    if (off >= LCDC_CSC && off < LCDC_CSC + 0x10) {
        s->csc[(off - LCDC_CSC) / 4] = v & csc_mask[(off - LCDC_CSC) / 4];
        return;
    }
    switch (off) {
    case LCDC_CHAIN_ADDR:
        s->chain_addr = v;
        break;
    case LCDC_GLB_CFG:
        s->glb_cfg = v & 0x3f003f;
        break;
    case LCDC_CTRL:
        lcdc_ctrl(s, v);
        break;
    case LCDC_CSR:
        s->st &= ~(v & LST_EVENTS);
        lcdc_update_irq(s);
        break;
    case LCDC_INTC:
        s->intc = v & 0x7e;
        lcdc_update_irq(s);
        break;
    case LCDC_PCFG:
        s->pcfg = v & 0x07ffffff;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps ingenic_lcdc_ops = {
    .read = ingenic_lcdc_read,
    .write = ingenic_lcdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void ingenic_lcdc_gate(void *opaque, int n, int level)
{
    IngenicLcdcState *s = opaque;

    s->gate_open = level;
}

/* As a T41LQ reads them at U-Boot, its gate opened */
static void ingenic_lcdc_reset_hold(Object *obj, ResetType type)
{
    IngenicLcdcState *s = INGENIC_LCDC(obj);

    lcdc_stop(s);
    s->chain_addr = 0;
    s->glb_cfg = 0x12;
    s->csc[0] = 0x066204a8;
    s->csc[1] = 0x03410190;
    s->csc[2] = 0x00000812;
    s->csc[3] = 0x00800000;
    s->st = s->intc = 0;
    s->pcfg = 0x03190842;
    disp_reset(&s->disp, false);
    memset(s->desc, 0, sizeof(s->desc));
    s->des_cnt = 0;
    s->chain_site = 0;
    s->slcd_1c = 0;
    s->arm = s->quirk;
    s->ack_pending = 0;
    s->fit = false;
    qemu_set_irq(s->irq, 0);
}

static void ingenic_lcdc_realize(DeviceState *dev, Error **errp)
{
    IngenicLcdcState *s = INGENIC_LCDC(dev);

    s->con = qemu_graphic_console_create(dev, 0, &ingenic_dpu_gfx_ops, s);
    qemu_console_resize(s->con, 240, 320);
}

static void ingenic_lcdc_init(Object *obj)
{
    IngenicLcdcState *s = INGENIC_LCDC(obj);

    memory_region_init_io(&s->iomem, obj, &ingenic_lcdc_ops, s,
                          "ingenic-lcdc", 0x10000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->clk = qdev_init_clock_in(DEVICE(obj), "clk", lcdc_clk_update, s,
                                ClockUpdate);
    qdev_init_gpio_in_named(DEVICE(obj), ingenic_lcdc_gate, "gate", 1);
    s->gate_open = true;
    s->frame = timer_new_ns(QEMU_CLOCK_VIRTUAL, lcdc_frame_end, s);
}

static const VMStateDescription vmstate_ingenic_lcdc = {
    .name = "ingenic-lcdc",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(gate_open, IngenicLcdcState),
        VMSTATE_UINT32(chain_addr, IngenicLcdcState),
        VMSTATE_UINT32(glb_cfg, IngenicLcdcState),
        VMSTATE_UINT32_ARRAY(csc, IngenicLcdcState, 4),
        VMSTATE_UINT32(st, IngenicLcdcState),
        VMSTATE_UINT32(intc, IngenicLcdcState),
        VMSTATE_UINT32(pcfg, IngenicLcdcState),
        VMSTATE_STRUCT(disp, IngenicLcdcState, 1, vmstate_ingenic_dpu_disp,
                       IngenicDpuDisp),
        VMSTATE_BOOL(fetched, IngenicLcdcState),
        VMSTATE_BOOL(sending, IngenicLcdcState),
        VMSTATE_BOOL(gen_stop, IngenicLcdcState),
        VMSTATE_UINT32_ARRAY(desc, IngenicLcdcState, INGENIC_LCDC_DESC_WORDS),
        VMSTATE_UINT32(des_cnt, IngenicLcdcState),
        VMSTATE_UINT32(chain_site, IngenicLcdcState),
        VMSTATE_UINT32(dma_site, IngenicLcdcState),
        VMSTATE_UINT32_V(slcd_1c, IngenicLcdcState, 2),
        VMSTATE_BOOL_V(arm, IngenicLcdcState, 2),
        VMSTATE_BOOL_V(fit, IngenicLcdcState, 2),
        VMSTATE_UINT32_V(ack_pending, IngenicLcdcState, 2),
        VMSTATE_TIMER_PTR(frame, IngenicLcdcState),
        VMSTATE_CLOCK(clk, IngenicLcdcState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property ingenic_lcdc_properties[] = {
    DEFINE_PROP_BOOL("xburst1", IngenicLcdcState, xburst1, false),
    DEFINE_PROP_BOOL("slcd-quirk", IngenicLcdcState, quirk, false),
};

static void ingenic_lcdc_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->realize = ingenic_lcdc_realize;
    dc->vmsd = &vmstate_ingenic_lcdc;
    device_class_set_props(dc, ingenic_lcdc_properties);
    rc->phases.hold = ingenic_lcdc_reset_hold;
}

static const TypeInfo ingenic_lcdc_type_info = {
    .name = TYPE_INGENIC_LCDC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IngenicLcdcState),
    .instance_init = ingenic_lcdc_init,
    .class_init = ingenic_lcdc_class_init,
};

static void ingenic_dpu_register_types(void)
{
    type_register_static(&ingenic_dpu_type_info);
    type_register_static(&ingenic_lcdc_type_info);
}

type_init(ingenic_dpu_register_types)
