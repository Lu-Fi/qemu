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
 * - The T33's is the T32's, but its driver writes most registers through
 *   lists the core loads itself: writing bit N of 0x22010 (0x22014 for N +
 *   32) runs the list word N of the table at 0x22020 points to, its length
 *   in words and then commands, each bit 31 | (count - 1) << 18 | the word
 *   offset of the first of count registers, their values after it. The
 *   model runs a list at once. After a reset this driver waits for 0x0 to
 *   read other than 0.
 *
 * - In direct mode (T23, T32, T33, T41) the driver gives channel 0 the
 *   IVDC's addresses, Y 0x80000000 and UV 0x81000000, the sensor in bits
 *   26:25: the model writes no frame there but pulses "ivdc-frame" for
 *   that sensor, and the encoders take the scene from the sensor
 *   themselves. The T23's and T41's drivers complete frame channel 0 on
 *   the core's frame start (the T41's at 0x40028 bit 20), and the T41's
 *   never drains channel 0's done FIFO then: its IVDC frames leave none.
 *
 * - The T23's, T32's, T33's, T40's and T41's MSCA draws libimp's ISP OSD
 *   into what a channel puts out, after scaling and crop: four opaque YUV
 *   masks a channel (position, size, 0 for off, colour), then the T23's,
 *   T40's and T41's draw blocks (isp_osd_draw()), then ARGB pictures
 *   (position, size, address, stride) on channel 0 (the T40's on 0 and 1
 *   too), read from memory every frame. Each SoC packs the pictures'
 *   type, byte order and opaque bit its own way, but the order codes mean
 *   the same: an ARGB8888 pixel's bytes from the lowest address, an
 *   ARGB1555 one's fields from bit 0 (1-bit alpha, colours shifted up 3).
 *   The RGB goes through a matrix in registers of its own (limited-range
 *   BT.601 out of reset), a pixel blends over what is below as (o * a +
 *   b * (256 - a)) >> 8, and each 2x2's chroma takes its top-left pixel's:
 *   as a T23, a T32, a T40 and a T41 draw (the T33 taken to draw as the
 *   T32). An alpha of 255, the opaque bit's too, covers all on a T40 and a
 *   T41, but a T23 and a T32 blend it as 255 and let 1/256 of what is
 *   below through. The T23's, T40's and T41's type register's bit c is
 *   channel c's ARGB1555 (a T41's "2-bit" type 2 draws 8888) and channel
 *   c's order is in bits 8c + 3:8c (the T40's driver puts channel 1's at
 *   7:4, which does not stick). The T32's type 2 is 2 bits a pixel, from
 *   bit 0 of each byte, into a palette of four a group: R in bits 7:0, G
 *   15:8, B 23:16 and a 1-bit alpha in 24, the bits above reading back 0.
 *   A shown picture whose rows are not whole 8-byte units (on a T23 or a
 *   T32, one of an odd width) stops their ISP until it is gone.
 *   The drivers load these registers once a frame through a port of
 *   (value, offset) pairs, the T23's offsets from its MSCA at 0xd000 (the
 *   T33 through a register list): the T23's from the thread its AE
 *   histogram interrupt (0xb4 bit 27) wakes, the T32's and T33's on the
 *   MSCA's frame done (0x300a8 bit 1), the T40's and T41's on channel 0's.
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
 * - The T21's, T23's, T31's, T32's and T33's CPM gates the ISP's clock,
 *   which stands it and its VIC still where they are, and resets it
 *   (SRBC's TIZIANO_SR): tx-isp gates it as it closes and resets it as it
 *   opens, before it gives it new buffers. Run on, the model would write
 *   frames and statistics into the buffers a closed session has given
 *   back.
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
#include <math.h>
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
#define VIC_MODE_MIPI       2

#define ISP_IVDC_ADDR       BIT(31)     /* a channel buffer over IVDC */

#define T33_RCFG_TRIG       0x22010     /* lists 0-31; 32-63 at +4 */
#define T33_RCFG_TABLE      0x22020
#define T32PRO_RCFG_TRIG    0x1c010
#define T32PRO_RCFG_TABLE   0x1c020
#define RCFG_MAX            0x4000      /* words in a list */

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

/*
 * Tiziano's AE and AWB statistics, as the T23's and T31's ISP cores and
 * Zeratul's RISC-V firmware unpack them (tisp_ae0_get_statistics,
 * tisp_ae0_get_hist, JZ_Isp_Get_Awb_Statistics). Both blocks work on the
 * frame's 2x2 quads (size: width << 16 | height) in a grid of zones (+0x4:
 * columns in bits 31:28, rows in 15:12; their sizes in quads a byte each
 * from +0x8 and +0x18) and, enabled (bit 0), each frame write to the next
 * of their buffers and raise a status bit.
 *
 * - AE: a luma histogram of 256 21-bit bins and one of IR, every quad in
 *   bin 0 (2 KiB, as two T31s write them), and per zone the luma sums of
 *   its dark, middle and bright quads and how many are dark and bright
 *   (below +0x28 bits 7:0, above bits 15:8), packed 21, 21, 21, 13 and 13
 *   bits into 16 bytes. Zone buffers from +0x2c, histogram buffers from
 *   +0x3c, their counts less one in +0x4c bits 5:4 and 1:0; +0x50 says
 *   which were written, in the same bits. The luma is in linear light: a
 *   T31's and a T41NQ's histograms are those of their pictures' luma with
 *   its gamma undone, and the model undoes the scene's (2.2). The T40's and
 *   T41's block (tisp_ae_get_statistics) packs 22, 22, 22, 14 and 14 bits,
 *   and the model writes their luma histogram twice.
 *
 * - AWB: per zone the R, G, B and IR sums of the quads it counts and their
 *   number, packed 21, 21, 21, 21 and 13 bits into 16 bytes, to the buffers
 *   from +0x3c (+0x4c their count less one, +0x50 which). It counts those
 *   in the white region of +0x28 to +0x34 (isp_awb_white()) whose luma is
 *   above +0x38 bits 7:0 and below bits 15:8, as a T31X's and a T31AL's
 *   held to bounds of our own; the quads have no IR. Its sums are of the
 *   raw picture, in linear light: the model undoes the scene's gamma and
 *   white balance for them (ISP_RAW_R, ISP_RAW_B), without which no grey of
 *   a balanced scene is in the drivers' white region.
 *
 * - The T40's and T41's AWB block, at 0x18000, keeps those registers (a
 *   write of 1 to +0x0 loads them, which reads 0 on a T41NQ; the model
 *   keeps it and starts on it) but sorts each zone's quads into 4 bands by
 *   luma, split at the bytes of +0x38 (10, 128 and 230 from the drivers),
 *   and gives each band a pair of records (128 bytes a zone), or only band
 *   +0x4c bits 26:25 its pair (32 bytes) with bit 28 set. A record is the
 *   R, G, B and IR sums and number packed 22, 22, 22, 22 and 14 bits into
 *   16 bytes, the first of all the band's quads, the second of those in the
 *   white region of +0x28 to +0x34 (isp_awb_white(), as a T40XP's and a
 *   T41NQ's held to bounds of our own; a T41NQ also leaves some quads out
 *   of every record, a few in a lit room and two thirds in a dark one,
 *   which the model does not). Bit 3 is raised. So the drivers read them
 *   (tisp_awb_get_statistics), and so a T41NQ writes them, raw and in
 *   linear light as the T31's.
 *
 * - The T32's AE and AWB blocks keep those registers 0x3c further on (AE at
 *   0x17000, AWB at 0x16000) but their enables at +0x10, write 8 bytes into
 *   each buffer before their data and raise 0x30028 bits 4 and 5 (AE) and 3
 *   (AWB), as its core firmware and tx-isp take them. AE zones are packed
 *   22, 22, 22, 14, 14 and 22 bits (the last nothing reads, left 0), the
 *   histogram written once, two 24-bit bins a 64-bit word. AWB zones are
 *   all the size in 0x160f4 (height << 16 | width, in quads), each the R,
 *   G, B and IR sums and number of all its quads (22, 22, 22, 22 and 14
 *   bits) and then of those near the Planckian locus, taken here as those
 *   in the white region (the last number's 2 low bits in the zone's 6
 *   words, its 12 high bits in 2 words after each 4 zones, half a word a
 *   zone). A row is whole groups of 4 zones (26 words) and the rest's (6 a
 *   zone, 2). Its tx-isp sets that region as the T41's do
 *   (tisp_awb_set_regional_threshold); its 4 bytes of luma at +0x38 the
 *   model does not decode.
 *
 * A buffer at 0 is taken as none yet: the T23's Linux driver enables AWB
 * before it gives it buffers, while frames from Zeratul's RISC-V core may
 * still be coming in.
 */
typedef struct IngenicIspStats {
    hwaddr size, ae, awb;           /* awb 0: none */
    hwaddr ae_en, awb_en;           /* the enables, if not at ae and awb */
    hwaddr awb_zone;                /* T32: every AWB zone's size */
    uint32_t int_ae_zones, int_ae_hist, int_awb;
    unsigned sum_bits, count_bits;  /* an AE zone's fields */
    unsigned head;                  /* bytes before a buffer's data */
    bool hist_pairs;                /* two 24-bit bins a 64-bit word */
    bool hist_ir;                   /* T23, T31: the second, of IR */
    bool awb_pairs;                 /* T40, T41: pairs of AWB records */
} IngenicIspStats;

static const IngenicIspStats ingenic_isp_stats = {
    .size = 0x004, .ae = 0xa000, .awb = 0xb000,
    .int_ae_zones = BIT(26), .int_ae_hist = BIT(27), .int_awb = BIT(30),
    .sum_bits = 21, .count_bits = 13, .hist_ir = true,
};

static const IngenicIspStats ingenic_t40_isp_stats = {
    .size = 0x080, .ae = 0x19000, .awb = 0x18000,
    .int_ae_zones = BIT(4), .int_ae_hist = BIT(5), .int_awb = BIT(3),
    .sum_bits = 22, .count_bits = 14,
    .awb_pairs = true,
};

static const IngenicIspStats ingenic_t32_isp_stats = {
    .size = 0x080, .ae = 0x1703c, .awb = 0x1603c,
    .ae_en = 0x17010, .awb_en = 0x16010, .awb_zone = 0x160f4,
    .int_ae_zones = BIT(4), .int_ae_hist = BIT(5), .int_awb = BIT(3),
    .sum_bits = 22, .count_bits = 14,
    .head = 8, .hist_pairs = true,
};

/* An interrupt word of its own, enable 1, raised each frame */
typedef struct IngenicIspTick {
    hwaddr en, sta, clr;
    uint32_t bits;
} IngenicIspTick;

/* How a layout packs its OSD pictures' type, opaque bit and byte order */
typedef enum IngenicIspOsdFmt {
    OSD_FMT_T40,        /* fmt: 1555 bit c, opaque bit 8 + c; order 8c + 3:8c */
    OSD_FMT_T32,        /* fmt: pictures 4g to 4g + 3 at bit 16g, type 3:0, */
                        /* opaque bit 4, order 11:8 */
} IngenicIspOsdFmt;

/* The MSCA's overlay, offsets of 0 for what a layout lacks */
typedef struct IngenicIspOsd {
    hwaddr mask;                    /* channel c's block b at */
    hwaddr mask_chan, mask_step;    /* + c * mask_chan + b * mask_step */
    hwaddr mask_en;                 /* bit c draws channel c's */
    hwaddr pic;                     /* channel c's picture n at */
    hwaddr pic_chan;                /* + c * pic_chan + n * 0x10 */
    unsigned pic_chans, pics;
    hwaddr pic_en;                  /* bit c * 8 + n shows it */
    IngenicIspOsdFmt style;
    hwaddr fmt, order;
    hwaddr palette;                 /* 2-bit pictures' colours, 0x10 a group */
    hwaddr csc;                     /* RGB to YUV: R, G, B, offsets */
    hwaddr draw;                    /* channel c's draw block n at */
    hwaddr draw_chan;               /* + c * draw_chan + n * 0x10 */
    unsigned draw_chans, draws;
    bool line_before;               /* a line's width before its run */
    bool blend_255;                 /* alpha 255 lets 1/256 through */
    bool stall_odd;                 /* an odd picture width stops it */
} IngenicIspOsd;

#define OSD_MASKS           4
#define OSD_DRAW_LINES      8       /* blocks that can draw a sloped line */

#define DRAW_EN             BIT(0)
#define DRAW_RANGE          BIT(4)
#define DRAW_LINE           BIT(5)

/*
 * A pixel's bytes, lowest address first, by the order code: the T23's and
 * T40's IMPISPARGBType names (the T41's SDK names the same codes from the
 * other end, so raptor's BGRA there is 13, bytes A, R, G, B: a T41 shows
 * its BGRA bitmaps so)
 */
static const char *const isp_osd_orders[16] = {
    "BGRA", "GBRA", "BRGA", "RBGA", "GRBA", "RGBA", NULL, NULL,
    "ABGR", "AGBR", "ABRG", "AGRB", "ARBG", "ARGB",
};

/* The matrix the T23, T40 and T41 read back: BT.601, limited range, Q10 */
static const uint32_t isp_osd_csc[4] = {
    0x381da506, 0xd13b5e02, 0xf6e70064, 0x00008010,
};

static const IngenicIspOsd ingenic_t23_isp_osd = {
    .mask = 0xd400, .mask_chan = 0x40, .mask_step = 0x10, .mask_en = 0xd044,
    .pic = 0xd760, .pic_chans = 1, .pics = 8, .pic_en = 0xd048,
    .style = OSD_FMT_T40, .fmt = 0xd750, .order = 0xd754, .csc = 0xd740,
    .draw = 0xd4c0, .draw_chans = 1, .draws = 6,
    .blend_255 = true, .stall_odd = true,
};

static const IngenicIspOsd ingenic_t32_isp_osd = {
    .mask = 0xf0400, .mask_chan = 0x40, .mask_step = 0x10, .mask_en = 0xf0044,
    .pic = 0xf0780, .pic_chans = 1, .pics = 8, .pic_en = 0xf0048,
    .style = OSD_FMT_T32, .fmt = 0xf0754, .palette = 0xf0760, .csc = 0xf0740,
    .blend_255 = true, .stall_odd = true,
};

static const IngenicIspOsd ingenic_t33_isp_osd = {
    .mask = 0xf0400, .mask_chan = 0x40, .mask_step = 0x10, .mask_en = 0xf0044,
    .pic = 0xf0500, .pic_chans = 1, .pics = 4, .pic_en = 0xf0048,
    .style = OSD_FMT_T32, .fmt = 0xf04e4, .palette = 0xf04f0, .csc = 0xf04d0,
    .blend_255 = true, .stall_odd = true,
};

/* The T33's, with the T32's 8 pictures after a 2-group palette */
static const IngenicIspOsd ingenic_t32pro_isp_osd = {
    .mask = 0xf0400, .mask_chan = 0x40, .mask_step = 0x10, .mask_en = 0xf0044,
    .pic = 0xf0510, .pic_chans = 1, .pics = 8, .pic_en = 0xf0048,
    .style = OSD_FMT_T32, .fmt = 0xf04e4, .palette = 0xf04f0, .csc = 0xf04d0,
    .blend_255 = true, .stall_odd = true,
};

static const IngenicIspOsd ingenic_t40_isp_osd = {
    .mask = 0x16138, .mask_chan = 0x100, .mask_step = 0xc, .mask_en = 0x16014,
    .pic = 0x16600, .pic_chan = 0x80, .pic_chans = 2, .pics = 8,
    .pic_en = 0x16018,
    .style = OSD_FMT_T40, .fmt = 0x16060, .order = 0x16064, .csc = 0x16050,
    .draw = 0x16400, .draw_chan = 0x100, .draw_chans = 2, .draws = 16,
    .line_before = true,
};

static const IngenicIspOsd ingenic_t41_isp_osd = {
    .mask = 0xf0138, .mask_chan = 0x100, .mask_step = 0xc, .mask_en = 0xf0014,
    .pic = 0xf0600, .pic_chans = 1, .pics = 8, .pic_en = 0xf0018,
    .style = OSD_FMT_T40, .fmt = 0xf0060, .order = 0xf0064, .csc = 0xf0050,
    .draw = 0xf0400, .draw_chans = 1, .draws = 20,
};

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
    hwaddr vic_mode;                /* its input: VIC_MODE_MIPI via the CSI */
    bool vic_ctrl_level;            /* the VIC runs while its bit 0 is set */
    const IngenicIspVicInt *vic_int;
    uint32_t vic_frame_start;
    hwaddr idle;                    /* bit 0 reads 1 */
    IngenicIspBusy busy[3];
    IngenicIspBusy set;             /* a register whose bits read set */
    hwaddr rcfg;                    /* the register lists' table, if any */
    hwaddr rcfg_trig;               /* lists 0-31 start; 32-63 at +4 */
    hwaddr shadow;                  /* the port of (value, offset) pairs */
    hwaddr shadow_open;             /* a write starts a run of pairs */
    hwaddr shadow_base;             /* where the pairs' offsets count from */
    IngenicIspTick tick;
    const IngenicIspStats *stats;
    const IngenicIspOsd *osd;
    const IngenicIspChanRegs *chan;
    bool done_uv_head;              /* the UV done shows the head entry */
    bool ivdc_no_done;              /* an IVDC frame leaves no done entry */
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
    .stats = &ingenic_isp_stats,
    .ch_busy = 0x9808,
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
    .vic_mode = 0x00c,
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
    .int_frame_end = BIT(13) | BIT(27),    /* 27: AE histogram done */
    .stats = &ingenic_isp_stats,
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
    .vic_mode = 0x00c,
    .vic_int = &ingenic_isp_vic_int,
    .shadow = 0xe004,
    .shadow_open = 0xe000,
    .shadow_base = 0xd000,
    .osd = &ingenic_t23_isp_osd,
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
    .vic_mode = 0x01c,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { 0x22014, 0x30000 } },
    .shadow = 0xf8004,
    .shadow_open = 0xf8000,
    .tick = { 0x300a0, 0x300a8, 0x300a4, BIT(1) },  /* the MSCA's frame done */
    .stats = &ingenic_t32_isp_stats,
    .osd = &ingenic_t32_isp_osd,
    .chan = &ingenic_t23_isp_chan_regs,
    .clr_bit0 = true,
};

/* The T32's, with register lists the core loads itself */
static const IngenicIspLayout ingenic_t33_isp_layout = {
    .desc = "Ingenic T33 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x30020, .int_sta = 0x30028, .int_clr = 0x30024,
          .done_en = 0x30030, .done_sta = 0x30038, .done_clr = 0x30034,
          .ch_en = 0xf0040, .ch = 0xf0100 },
    },
    .int_frame_end = BIT(0),
    .ch_busy = 0xf0054,
    .vics = 1,
    .vic = 0x80000,
    .vic_stride = 0x1000,
    .vic_ctrl = 0x010,
    .vic_ctrl_level = true,
    .vic_mode = 0x01c,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { T33_RCFG_TRIG, ~0u },
              { T33_RCFG_TRIG + 4, ~0u } },
    .set = { 0x00000, 1 },
    .rcfg = T33_RCFG_TABLE,
    .rcfg_trig = T33_RCFG_TRIG,
    .tick = { 0x300a0, 0x300a8, 0x300a4, BIT(1) },
    .osd = &ingenic_t33_isp_osd,
    .chan = &ingenic_t23_isp_chan_regs,
    .clr_bit0 = true,
};

/* The T33's, its register lists' block and OSD pictures moved */
static const IngenicIspLayout ingenic_t32pro_isp_layout = {
    .desc = "Ingenic T32Pro ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x30020, .int_sta = 0x30028, .int_clr = 0x30024,
          .done_en = 0x30030, .done_sta = 0x30038, .done_clr = 0x30034,
          .ch_en = 0xf0040, .ch = 0xf0100 },
    },
    .int_frame_end = BIT(0),
    .ch_busy = 0xf0054,
    .vics = 1,
    .vic = 0x80000,
    .vic_stride = 0x1000,
    .vic_ctrl = 0x010,
    .vic_ctrl_level = true,
    .vic_mode = 0x01c,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { T32PRO_RCFG_TRIG, ~0u },
              { T32PRO_RCFG_TRIG + 4, ~0u } },
    .set = { 0x00000, 1 },
    .rcfg = T32PRO_RCFG_TABLE,
    .rcfg_trig = T32PRO_RCFG_TRIG,
    .tick = { 0x300a0, 0x300a8, 0x300a4, BIT(1) },
    .osd = &ingenic_t32pro_isp_osd,
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
    .shadow_open = 0x50320,
    .stats = &ingenic_t40_isp_stats,
    .osd = &ingenic_t40_isp_osd,
    .chan = &ingenic_isp_chan_regs,
};

static const IngenicIspLayout ingenic_t41_isp_layout = {
    .desc = "Ingenic T41 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
          .ch_en = 0xf0008, .ch = 0xf0100 },
    },
    .int_frame_start = BIT(13) | BIT(20),
    .vics = 1,
    .vic = 0x80000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .idle = 0x204,
    .shadow = 0xf8004,
    .shadow_open = 0xf8000,
    .stats = &ingenic_t40_isp_stats,
    .osd = &ingenic_t41_isp_osd,
    .chan = &ingenic_isp_chan_regs,
    .ivdc_no_done = true,
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
    if (l->tick.sta) {
        isp |= R(l->tick.sta) & R(l->tick.en);
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

/* A channel's scaled size and the window it puts out; false if none */
static bool isp_chan_geometry(IngenicIspState *s, unsigned p, unsigned n,
                              uint32_t *w, uint32_t *h, uint32_t *cx,
                              uint32_t *cy, uint32_t *cw, uint32_t *ch_h)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspChanRegs *r = l->chan;
    hwaddr base = l->pipe[p].ch + n * 0x100;
    uint32_t size = R(base + r->size), crop = R(base + r->crop_size);
    uint32_t pos = R(base + r->crop_pos);

    /* NV12 needs even sizes and positions */
    *w = (size >> 16) & ~1;
    *h = (size & 0xffff) & ~1;
    *cx = pos >> 16;
    *cy = pos & 0xffff;
    *cw = crop >> 16;
    *ch_h = crop & 0xffff;
    if (!crop || *cx + *cw > *w || *cy + *ch_h > *h) {
        *cx = *cy = 0;
        *cw = *w;
        *ch_h = *h;
    }
    *cx &= ~1;
    *cy &= ~1;
    *cw &= ~1;
    *ch_h &= ~1;
    return *cw && *ch_h;
}

static void isp_osd_mask(uint8_t *buf, uint32_t bw, uint32_t bh,
                         uint32_t ox, uint32_t oy, uint32_t ow, uint32_t oh,
                         uint32_t pos, uint32_t size, uint32_t yuv)
{
    uint32_t x0 = pos >> 16, y0 = pos & 0xffff;
    uint32_t w = size >> 16, h = size & 0xffff;
    uint8_t *uv = buf + bw * bh;

    if (!w || !h || x0 >= ow || y0 >= oh) {
        return;
    }
    w = MIN(w, ow - x0);
    h = MIN(h, oh - y0);
    x0 += ox;
    y0 += oy;
    for (uint32_t y = y0; y < y0 + h; y++) {
        memset(buf + y * bw + x0, yuv >> 16, w);
    }
    for (uint32_t y = y0 & ~1; y < y0 + h; y += 2) {
        for (uint32_t x = x0 & ~1; x < x0 + w; x += 2) {
            uv[y / 2 * bw + x] = yuv >> 8;
            uv[y / 2 * bw + x + 1] = yuv;
        }
    }
}

static void isp_osd_format(IngenicIspState *s, const IngenicIspOsd *o,
                           unsigned c, unsigned n, unsigned *type,
                           bool *opaque, unsigned *order)
{
    uint32_t f = R(o->fmt);

    switch (o->style) {
    case OSD_FMT_T40:
        *type = extract32(f, c, 1);
        *opaque = extract32(f, 8 + c, 1);
        *order = extract32(R(o->order), 8 * c, 4);
        break;
    case OSD_FMT_T32:
        f >>= 16 * (n / 4);
        *type = f & 0xf;
        *opaque = f & BIT(4);
        *order = extract32(f, 8, 4);
        break;
    default:
        g_assert_not_reached();
    }
}

static uint8_t isp_clip(int v)
{
    return v < 0 ? 0 : v > 255 ? 255 : v;
}

/* What a picture's pixel o is over a pixel b, at an alpha of 0 to 256 */
static uint8_t isp_osd_blend(int o, int b, int a)
{
    return (o * a + b * (256 - a)) >> 8;
}

/*
 * Picture n of channel c: each pixel's RGB through the matrix at o->csc
 * (rounding down), blended over the luma, and the top-left pixel of each
 * 2x2 over the chroma
 */
static void isp_osd_picture(IngenicIspState *s, const IngenicIspOsd *o,
                            unsigned c, unsigned n, uint8_t *buf,
                            uint32_t bw, uint32_t bh, uint32_t ox,
                            uint32_t oy, uint32_t ow, uint32_t oh)
{
    hwaddr reg = o->pic + c * o->pic_chan + n * 0x10;
    uint32_t pos = R(reg), size = R(reg + 4), addr = R(reg + 8);
    uint32_t stride = R(reg + 12) & 0xffff;
    uint32_t px = pos >> 16, py = pos & 0xffff;
    uint32_t pw = size >> 16, ph = size & 0xffff;
    int m[3][3], off[3];
    unsigned type, code;
    bool opaque;
    const char *order;
    g_autofree uint8_t *row = NULL;
    uint8_t *uvb = buf + bw * bh;
    int at[4], x0, y0, w, h;

    if (!pw || !ph || !addr || px >= ow || py >= oh) {
        return;
    }
    isp_osd_format(s, o, c, n, &type, &opaque, &code);
    order = isp_osd_orders[code];
    if (type > 2 || !order || (type == 2 && !o->palette)) {
        qemu_log_mask(LOG_UNIMP, "ingenic-isp: OSD picture %u: type %u, byte "
                      "order %u\n", n, type, code);
        return;
    }
    for (int i = 0; i < 4; i++) {
        at[strchr("RGBA", order[i]) - "RGBA"] = i;
    }
    /* per input R, G, B: Y in bits 9:0, U 20:10 and V 31:21 signed, Q10 */
    for (int k = 0; k < 3; k++) {
        uint32_t v = R(o->csc + 4 * k);

        m[0][k] = extract32(v, 0, 10);
        m[1][k] = sextract32(v, 10, 11);
        m[2][k] = sextract32(v, 21, 11);
    }
    off[0] = extract32(R(o->csc + 12), 0, 8);
    off[1] = off[2] = extract32(R(o->csc + 12), 8, 8);
    w = MIN(pw, ow - px);
    h = MIN(ph, oh - py);
    x0 = ox + px;
    y0 = oy + py;
    row = g_malloc(type == 0 ? pw * 4 : type == 1 ? pw * 2 : (pw + 3) / 4);

    for (int r = 0; r < h; r++) {
        uint8_t *d = buf + (size_t)(y0 + r) * bw + x0;
        uint8_t *uv = uvb + (size_t)((y0 + r) / 2) * bw + x0;

        address_space_read(&address_space_memory, addr + r * stride,
                           MEMTXATTRS_UNSPECIFIED, row,
                           type == 0 ? w * 4 : type == 1 ? w * 2 :
                           (w + 3) / 4);
        for (int x = 0; x < w; x++) {
            int rgb[3], a = 0, yuv[3];

            if (type == 0) {
                for (int k = 0; k < 3; k++) {
                    rgb[k] = row[4 * x + at[k]];
                }
                a = row[4 * x + at[3]];
            } else if (type == 1) {
                uint16_t v = row[2 * x] | row[2 * x + 1] << 8;

                /* the order's letters name the fields from bit 0 up */
                for (int i = 0, b = 0; i < 4; i++) {
                    if (order[i] == 'A') {
                        a = extract32(v, b++, 1) ? 255 : 0;
                    } else {
                        rgb[strchr("RGB", order[i]) - "RGB"] =
                            extract32(v, b, 5) << 3;
                        b += 5;
                    }
                }
            } else {
                uint32_t e = R(o->palette + 0x10 * (n / 4) +
                               4 * extract32(row[x / 4], 2 * (x % 4), 2));

                for (int k = 0; k < 3; k++) {
                    rgb[k] = extract32(e, 8 * k, 8);
                }
                a = extract32(e, 24, 1) ? 255 : 0;
            }
            if (opaque) {
                a = 255;
            }
            if (a == 255 && !o->blend_255) {
                a = 256;
            }
            for (int k = 0; k < 3; k++) {
                yuv[k] = isp_clip(((m[k][0] * rgb[0] + m[k][1] * rgb[1] +
                                    m[k][2] * rgb[2]) >> 10) + off[k]);
            }
            d[x] = isp_osd_blend(yuv[0], d[x], a);
            if (!((y0 + r) & 1) && !((x0 + x) & 1)) {
                uv[x] = isp_osd_blend(yuv[1], uv[x], a);
                uv[x + 1] = isp_osd_blend(yuv[2], uv[x + 1], a);
            }
        }
    }
}

/*
 * Whether draw block n, a box bw x bh, covers its (x, y): a window's
 * border lw wide, of a range only what lies in the ext x ext squares at
 * the corners, a line from the box's top left (direction 0, 2) or bottom
 * left (1, 3), its run along the long side by the Q8 slope, rounded down,
 * and lw rows thick below the run if shallow (0, 1), lw columns right of
 * it if steep (2, 3), or above and left with @before (a T40's); a block
 * past the line engines fills the box instead
 */
static bool isp_draw_covers(unsigned n, int bw, int bh, int lw, uint32_t ctl,
                            bool before, int x, int y)
{
    int ext = ctl >> 16, slope = extract32(ctl, 16, 9), d;
    unsigned dir = extract32(ctl, 25, 2);
    bool border = x < lw || x >= bw - lw || y < lw || y >= bh - lw;

    if (ctl & DRAW_LINE) {
        if (n >= OSD_DRAW_LINES) {
            return true;
        }
        if (dir & 1) {
            y = bh - 1 - y;
        }
        d = dir & 2 ? x - (y * slope >> 8) : y - (x * slope >> 8);
        if (before) {
            d = -d;
        }
        return d >= 0 && d < lw;
    }
    if (ctl & DRAW_RANGE) {
        return border && (x < ext || x >= bw - ext) &&
               (y < ext || y >= bh - ext);
    }
    return border;
}

/* A draw block's colour c over b at its alpha: 0 to 4 quarters, 5 to 7 0 */
static uint8_t isp_draw_blend(int c, int b, unsigned a)
{
    return a > 4 ? 0 : (c * a + b * (4 - a)) >> 2;
}

/*
 * Channel c's draw blocks: position, size, colour 23:0 with the line
 * width in 31:24, then enable, range (bit 4, its arms in 31:16), line
 * (bit 5, the slope in 24:16 and the direction in 26:25) and alpha (10:8).
 * The lowest numbered block over a pixel draws it, at its alpha, alone;
 * chroma takes each 2x2's top-left pixel's.
 */
static void isp_osd_draw(IngenicIspState *s, const IngenicIspOsd *o,
                         unsigned c, uint8_t *buf, uint32_t bw, uint32_t bh,
                         uint32_t ox, uint32_t oy, uint32_t ow, uint32_t oh)
{
    g_autofree uint8_t *taken = NULL;
    uint8_t *uvb = buf + bw * bh;

    for (unsigned n = 0; n < o->draws; n++) {
        hwaddr reg = o->draw + c * o->draw_chan + n * 0x10;
        uint32_t pos = R(reg), size = R(reg + 4), col = R(reg + 8);
        uint32_t ctl = R(reg + 12);
        uint32_t x0 = pos >> 16, y0 = pos & 0xffff;
        uint32_t w = size >> 16, h = size & 0xffff;
        unsigned a = extract32(ctl, 8, 3);

        if (!(ctl & DRAW_EN) || !w || !h || x0 >= ow || y0 >= oh) {
            continue;
        }
        if (!taken) {
            taken = g_malloc0((size_t)ow * oh);
        }
        for (uint32_t y = 0; y < MIN(h, oh - y0); y++) {
            uint32_t by = oy + y0 + y;
            uint8_t *d = buf + (size_t)by * bw + ox + x0;
            uint8_t *uv = uvb + (size_t)(by / 2) * bw + ox + x0;
            uint8_t *t = taken + (size_t)(y0 + y) * ow + x0;

            for (uint32_t x = 0; x < MIN(w, ow - x0); x++) {
                if (t[x] || !isp_draw_covers(n, w, h, col >> 24, ctl,
                                             o->line_before, x, y)) {
                    continue;
                }
                t[x] = 1;
                d[x] = isp_draw_blend(col >> 16 & 0xff, d[x], a);
                if (!(by & 1) && !((ox + x0 + x) & 1)) {
                    uv[x] = isp_draw_blend(col >> 8 & 0xff, uv[x], a);
                    uv[x + 1] = isp_draw_blend(col & 0xff, uv[x + 1], a);
                }
            }
        }
    }
}

/*
 * Whether a shown picture's rows are not whole 8-byte units (on a T23 or
 * a T32, it has an odd width), which stops the ISP, every channel, until
 * they are
 */
static bool isp_osd_stalled(IngenicIspState *s)
{
    const IngenicIspOsd *o = isp_layout(s)->osd;

    if (!o) {
        return false;
    }
    for (unsigned c = 0; c < o->pic_chans; c++) {
        for (unsigned n = 0; n < o->pics; n++) {
            hwaddr reg = o->pic + c * o->pic_chan + n * 0x10;
            unsigned type, code;
            bool opaque;

            if (!(R(o->pic_en) & BIT(c * 8 + n)) || !R(reg + 4) ||
                !R(reg + 8)) {
                continue;
            }
            isp_osd_format(s, o, c, n, &type, &opaque, &code);
            if (o->stall_odd ? R(reg + 4) & BIT(16) :
                (R(reg + 4) >> 16) * (type ? 2 : 4) % 8) {
                return true;
            }
        }
    }
    return false;
}

/*
 * Channel c's overlay, into an NV12 picture bw x bh whose window ow x oh
 * at (ox, oy) the channel puts out: its masks, its draw blocks, then its
 * pictures
 */
static void isp_overlay(IngenicIspState *s, unsigned c, uint8_t *buf,
                        uint32_t bw, uint32_t bh, uint32_t ox, uint32_t oy,
                        uint32_t ow, uint32_t oh)
{
    const IngenicIspOsd *o = isp_layout(s)->osd;

    if (!o) {
        return;
    }
    if (R(o->mask_en) & BIT(c)) {
        for (unsigned b = 0; b < OSD_MASKS; b++) {
            hwaddr m = o->mask + c * o->mask_chan + b * o->mask_step;

            isp_osd_mask(buf, bw, bh, ox, oy, ow, oh, R(m), R(m + 4),
                         R(m + 8));
        }
    }
    if (c < o->draw_chans) {
        isp_osd_draw(s, o, c, buf, bw, bh, ox, oy, ow, oh);
    }
    for (unsigned n = 0; c < o->pic_chans && n < o->pics; n++) {
        if (R(o->pic_en) & BIT(c * 8 + n)) {
            isp_osd_picture(s, o, c, n, buf, bw, bh, ox, oy, ow, oh);
        }
    }
}

void ingenic_isp_render(IngenicIspState *s, uint32_t w, uint32_t h,
                        uint8_t *nv12)
{
    ImageSensorState *sensor = image_sensor_find();
    uint32_t sw, sh, cx, cy, cw, ch_h;
    g_autofree uint8_t *pic = NULL;

    memset(nv12, 0, (size_t)w * h * 3 / 2);
    if (!sensor) {
        return;
    }
    if (!isp_chan_geometry(s, 0, 0, &sw, &sh, &cx, &cy, &cw, &ch_h)) {
        sw = cw = w;
        sh = ch_h = h;
        cx = cy = 0;
    }
    pic = g_malloc((size_t)sw * sh * 3 / 2);
    image_sensor_render(sensor, s->frame, sw, sh, pic);
    isp_overlay(s, 0, pic, sw, sh, cx, cy, cw, ch_h);
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *src = pic + (size_t)(cy + MIN(y, ch_h - 1)) * sw + cx;
        uint8_t *dst = nv12 + (size_t)y * w;

        memcpy(dst, src, MIN(w, cw));
        if (w > cw) {
            memset(dst + cw, src[cw - 1], w - cw);
        }
    }
    for (uint32_t y = 0; y < h / 2; y++) {
        const uint8_t *src = pic + (size_t)sw * sh +
                             (size_t)(cy / 2 + MIN(y, ch_h / 2 - 1)) * sw + cx;
        uint8_t *dst = nv12 + (size_t)w * h + (size_t)y * w;

        memcpy(dst, src, MIN(w, cw));
        for (uint32_t x = cw; x + 1 < w; x += 2) {
            dst[x] = src[cw - 2];
            dst[x + 1] = src[cw - 1];
        }
    }
}

static void isp_chan_frame(IngenicIspState *s, ImageSensorState *sensor,
                           unsigned p, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspChanRegs *r = l->chan;
    hwaddr base = l->pipe[p].ch + n * 0x100;
    IngenicIspChan *ch = &s->chan[p][n];
    g_autofree uint8_t *nv12 = NULL;
    uint32_t w, h, cx, cy, cw, ch_h;
    uint32_t y, uv, i;

    if (!isp_chan_geometry(s, p, n, &w, &h, &cx, &cy, &cw, &ch_h) ||
        !ch->y_n || !ch->uv_n || ch->done_n) {
        return;
    }
    y = isp_pop(ch->y, &ch->y_rd, &ch->y_n);
    uv = isp_pop(ch->uv, &ch->uv_rd, &ch->uv_n);
    if (y & ISP_IVDC_ADDR) {
        qemu_irq_pulse(s->ivdc_frame[extract32(y, 25, 2)]);
        if (l->ivdc_no_done) {
            R(l->pipe[p].done_sta ?: l->pipe[p].int_sta) |= BIT(n);
            return;
        }
    } else {
        nv12 = g_malloc(w * h * 3 / 2);
        image_sensor_render(sensor, s->frame, w, h, nv12);
        if (p == 0) {
            isp_overlay(s, n, nv12, w, h, cx, cy, cw, ch_h);
        }
        isp_dma_plane(y, nv12, w, cx, cy, cw, ch_h, R(base + r->y_stride));
        isp_dma_plane(uv, nv12 + w * h, w, cx, cy / 2, cw, ch_h / 2,
                      R(base + r->uv_stride));
    }
    i = (ch->done_rd + ch->done_n++) % INGENIC_ISP_FIFO;
    ch->done_y[i] = y;
    ch->done_uv[i] = uv;
    ch->done_frame[i] = s->frame;
    R(l->pipe[p].done_sta ?: l->pipe[p].int_sta) |= BIT(n);
}

#define STATS_GRID_MAX  15
#define STATS_ZONES     (STATS_GRID_MAX * STATS_GRID_MAX)

/* A block's grid: for each quad column and row its zone's, or -1 past */
typedef struct IspStatsGrid {
    unsigned cols, rows;
    int8_t *col, *row;
} IspStatsGrid;

static void isp_stats_map(IngenicIspState *s, hwaddr sizes, unsigned n,
                          int8_t *map, uint32_t len)
{
    uint32_t at = 0;

    memset(map, -1, len);
    for (unsigned i = 0; i < n && at < len; i++) {
        uint32_t size = extract32(R(sizes + i / 4 * 4), i % 4 * 8, 8);

        memset(map + at, i, MIN(size, len - at));
        at += size;
    }
}

static void isp_stats_grid(IngenicIspState *s, hwaddr b, IspStatsGrid *g,
                           uint32_t qw, uint32_t qh)
{
    g->cols = MIN(R(b + 0x4) >> 28, STATS_GRID_MAX);
    g->rows = MIN(extract32(R(b + 0x4), 12, 4), STATS_GRID_MAX);
    g->col = g_malloc(qw);
    g->row = g_malloc(qh);
    isp_stats_map(s, b + 0x8, g->cols, g->col, qw);
    isp_stats_map(s, b + 0x18, g->rows, g->row, qh);
}

/* The T32's AWB grid: zones all one size, as many as fit */
static void isp_stats_grid_even(IngenicIspState *s, hwaddr zone,
                                IspStatsGrid *g, uint32_t qw, uint32_t qh)
{
    uint32_t zw = extract32(R(zone), 0, 16), zh = extract32(R(zone), 16, 16);

    g->cols = zw ? MIN(qw / zw, STATS_GRID_MAX) : 0;
    g->rows = zh ? MIN(qh / zh, STATS_GRID_MAX) : 0;
    g->col = g_malloc(qw);
    g->row = g_malloc(qh);
    for (uint32_t x = 0; x < qw; x++) {
        g->col[x] = zw && x / zw < g->cols ? x / zw : -1;
    }
    for (uint32_t y = 0; y < qh; y++) {
        g->row[y] = zh && y / zh < g->rows ? y / zh : -1;
    }
}

static void isp_stats_grid_free(IspStatsGrid *g)
{
    g_free(g->col);
    g_free(g->row);
}

typedef struct IspStatsSums {
    uint32_t ae[STATS_ZONES][5];    /* dark, middle, bright sums; counts */
    uint32_t awb[STATS_ZONES][4];   /* R, G, B sums; count */
    uint32_t awb_all[STATS_ZONES][4];   /* the same of all quads */
    uint32_t awb_band[STATS_ZONES][4][4];   /* T40, T41: by luma band */
    uint32_t awb_white[STATS_ZONES][4][4];  /* and their white quads */
    uint32_t hist[256];
} IspStatsSums;

/* An 8-bit gamma-2.2 value in linear light, 8-bit */
static uint8_t isp_linear[256];

/*
 * A grey's raw R and B in 1/256ths of its G, as an OS04D10 sees daylight
 * (the 5000 K a T41's driver assumes with no white quads): the scene is
 * white-balanced
 */
#define ISP_RAW_R 144
#define ISP_RAW_B 105

static void isp_stats_dma(hwaddr addr, const void *buf, hwaddr len)
{
    if (addr) {
        address_space_write(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, buf, len);
    }
}

/* v, saturated to bits, at bit at of the little-endian words w */
static void isp_stats_put(uint32_t *w, unsigned at, unsigned bits,
                          uint32_t v)
{
    uint64_t x = (uint64_t)MIN(v, MAKE_64BIT_MASK(0, bits)) << (at % 32);

    w[at / 32] |= x;
    w[at / 32 + 1] |= x >> 32;
}

static void isp_ae_write(IngenicIspState *s, const IspStatsSums *sum,
                         const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->ae;
    uint32_t idx = R(b + 0x50), counts = R(b + 0x4c);
    uint32_t zbuf = (extract32(idx, 4, 2) + 1) % (extract32(counts, 4, 2) + 1);
    uint32_t hbuf = (extract32(idx, 0, 2) + 1) % (extract32(counts, 0, 2) + 1);
    uint32_t zone[STATS_ZONES * 4 + 1] = { 0 };
    uint32_t hist[512] = { 0 };
    hwaddr hist_len = sizeof(hist);

    for (unsigned z = 0; z < g->rows * g->cols; z++) {
        unsigned at = z * 128;

        for (unsigned f = 0; f < 5; f++) {
            unsigned bits = f < 3 ? st->sum_bits : st->count_bits;

            isp_stats_put(zone, at, bits, sum->ae[z][f]);
            at += bits;
        }
    }
    for (unsigned i = 0; i < ARRAY_SIZE(zone); i++) {
        zone[i] = cpu_to_le32(zone[i]);
    }
    if (st->hist_pairs) {
        for (unsigned i = 0; i < 256; i++) {
            isp_stats_put(hist, i * 24 + i / 2 * 16, 24, sum->hist[i]);
        }
        for (unsigned i = 0; i < 256; i++) {
            hist[i] = cpu_to_le32(hist[i]);
        }
        hist_len = 256 * 4;
    } else {
        uint32_t quads = 0;

        for (unsigned i = 0; i < 256; i++) {
            hist[i] = cpu_to_le32(MIN(sum->hist[i], 0x1fffff));
            hist[256 + i] = st->hist_ir ? 0 : hist[i];
            quads += sum->hist[i];
        }
        if (st->hist_ir) {
            hist[256] = cpu_to_le32(MIN(quads, 0x1fffff));
        }
    }
    if (R(b + 0x2c + zbuf * 4)) {
        isp_stats_dma(R(b + 0x2c + zbuf * 4) + st->head, zone,
                      STATS_ZONES * 16);
    }
    if (R(b + 0x3c + hbuf * 4)) {
        isp_stats_dma(R(b + 0x3c + hbuf * 4) + st->head, hist, hist_len);
    }
    R(b + 0x50) = zbuf << 4 | hbuf;
    R(isp_layout(s)->pipe[0].int_sta) |= st->int_ae_zones | st->int_ae_hist;
}

static void isp_awb_write(IngenicIspState *s, const IspStatsSums *sum,
                          const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb;
    uint32_t buf = (R(b + 0x50) + 1) % ((R(b + 0x4c) & 3) + 1);
    uint32_t zone[STATS_ZONES * 4] = { 0 };

    for (unsigned z = 0; z < g->rows * g->cols; z++) {
        uint32_t r = MIN(sum->awb[z][0], 0x1fffff);
        uint32_t gr = MIN(sum->awb[z][1], 0x1fffff);
        uint32_t bl = MIN(sum->awb[z][2], 0x1fffff);
        uint32_t n = MIN(sum->awb[z][3], 0x1fff);

        zone[z * 4] = cpu_to_le32(r | gr << 21);
        zone[z * 4 + 1] = cpu_to_le32(gr >> 11 | bl << 10);
        zone[z * 4 + 2] = cpu_to_le32(n << 20);
        zone[z * 4 + 3] = cpu_to_le32(n >> 12);
    }
    isp_stats_dma(R(b + 0x3c + buf * 4), zone, sizeof(zone));
    R(b + 0x50) = buf;
    R(isp_layout(s)->pipe[0].int_sta) |= st->int_awb;
}

/* The T32's: both sets a zone, in groups of 4 zones */
static void isp_awb_write_t32(IngenicIspState *s, const IspStatsSums *sum,
                              const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb;
    uint32_t buf = (R(b + 0x50) + 1) % ((R(b + 0x4c) & 3) + 1);
    unsigned rem = g->cols % 4;
    unsigned row_words = g->cols / 4 * 26 + (rem ? rem * 6 + 2 : 0);
    g_autofree uint32_t *w = g_new0(uint32_t, row_words * g->rows + 1);

    for (unsigned r = 0; r < g->rows; r++) {
        for (unsigned c = 0; c < g->cols; c++) {
            unsigned z = r * g->cols + c, k = c % 4;
            unsigned grp = r * row_words + c / 4 * 26;
            uint32_t *zw = w + grp + k * 6;
            uint32_t *extra = w + grp + (c / 4 < g->cols / 4 ? 4 : rem) * 6;
            uint32_t n = MIN(sum->awb[z][3], 0x3fff);

            for (unsigned f = 0; f < 3; f++) {
                isp_stats_put(zw, f * 22, 22, sum->awb_all[z][f]);
                isp_stats_put(zw, 102 + f * 22, 22, sum->awb[z][f]);
            }
            isp_stats_put(zw, 88, 14, sum->awb_all[z][3]);
            zw[5] |= n << 30;
            extra[k / 2] |= (n >> 2) << (k % 2 * 16);
        }
    }
    for (unsigned i = 0; i < row_words * g->rows; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    if (R(b + 0x3c + buf * 4)) {
        isp_stats_dma(R(b + 0x3c + buf * 4) + st->head, w,
                      row_words * g->rows * 4);
    }
    R(b + 0x50) = buf;
    R(isp_layout(s)->pipe[0].int_sta) |= st->int_awb;
}

/*
 * The white region, in 1/256ths: R/G and B/G rounded down strictly between
 * the halves of +0x28 and +0x2c, so that a quad without R or B is never
 * white, and (c * R + 256 * B) / G, c the low half of +0x30 and +0x34, at
 * least the high half of +0x30 and at most that of +0x34
 */
static bool isp_awb_white(IngenicIspState *s, const uint32_t *lin)
{
    hwaddr b = isp_layout(s)->stats->awb;
    uint32_t rg = R(b + 0x28), bg = R(b + 0x2c);
    uint32_t lo = R(b + 0x30), hi = R(b + 0x34);
    uint64_t g = lin[1], bl = lin[2] * 256ull;
    uint32_t qr, qb;

    if (!g) {
        return false;
    }
    qr = lin[0] * 256 / g;
    qb = lin[2] * 256 / g;
    return qr > extract32(rg, 0, 16) && qr < extract32(rg, 16, 16) &&
           qb > extract32(bg, 0, 16) && qb < extract32(bg, 16, 16) &&
           extract32(lo, 0, 16) * (uint64_t)lin[0] + bl >=
               extract32(lo, 16, 16) * g &&
           extract32(hi, 0, 16) * (uint64_t)lin[0] + bl <=
               extract32(hi, 16, 16) * g;
}

/* The T40's and T41's: a pair of records for each luma band */
static void isp_awb_write_pairs(IngenicIspState *s, const IspStatsSums *sum,
                                const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb;
    uint32_t buf = (R(b + 0x50) + 1) % ((R(b + 0x4c) & 3) + 1);
    bool one = R(b + 0x4c) & BIT(28);
    unsigned pairs = one ? 1 : 4;
    unsigned zones = g->rows * g->cols;
    g_autofree uint32_t *w = g_new0(uint32_t, zones * pairs * 8 + 1);

    for (unsigned z = 0; z < zones; z++) {
        for (unsigned p = 0; p < pairs; p++) {
            unsigned band = one ? extract32(R(b + 0x4c), 25, 2) : p;

            for (unsigned r = 0; r < 2; r++) {
                const uint32_t *v = r ? sum->awb_white[z][band]
                                      : sum->awb_band[z][band];
                uint32_t *rec = w + (z * pairs + p) * 8 + r * 4;

                for (unsigned f = 0; f < 3; f++) {
                    isp_stats_put(rec, f * 22, 22, v[f]);
                }
                isp_stats_put(rec, 88, 14, v[3]);
            }
        }
    }
    for (unsigned i = 0; i < zones * pairs * 8; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    isp_stats_dma(R(b + 0x3c + buf * 4), w, zones * pairs * 32);
    R(b + 0x50) = buf;
    R(isp_layout(s)->pipe[0].int_sta) |= st->int_awb;
}

/* One pass over the frame's quads for both blocks */
static void isp_stats(IngenicIspState *s, ImageSensorState *sensor)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    uint32_t qw = (R(st->size) >> 16) / 2 & ~1;
    uint32_t qh = (R(st->size) & 0xffff) / 2 & ~1;
    bool ae = R(st->ae_en ?: st->ae) & 1;
    bool awb = st->awb && R(st->awb_en ?: st->awb) & 1;
    uint32_t dark = extract32(R(st->ae + 0x28), 0, 8);
    uint32_t bright = extract32(R(st->ae + 0x28), 8, 8);
    uint32_t least = extract32(R(st->awb + 0x38), 0, 8);
    uint32_t most = extract32(R(st->awb + 0x38), 8, 8);
    g_autofree IspStatsSums *sum = NULL;
    g_autofree uint8_t *pic = NULL;
    IspStatsGrid ga, gw;

    if ((!ae && !awb) || !qw || !qh) {
        return;
    }
    pic = g_malloc(qw * qh * 3 / 2);
    image_sensor_render(sensor, s->frame, qw, qh, pic);
    sum = g_new0(IspStatsSums, 1);
    isp_stats_grid(s, st->ae, &ga, qw, qh);
    if (st->awb_zone) {
        isp_stats_grid_even(s, st->awb_zone, &gw, qw, qh);
    } else {
        isp_stats_grid(s, st->awb, &gw, qw, qh);
    }
    for (uint32_t y = 0; y < qh; y++) {
        const uint8_t *row = pic + y * qw;
        const uint8_t *uv = pic + qw * qh + y / 2 * qw;
        int ar = ae ? ga.row[y] : -1;
        int wr = awb ? gw.row[y] : -1;

        for (uint32_t x = 0; x < qw; x++) {
            int luma = row[x];
            uint32_t al = isp_linear[luma];

            if (ae) {
                sum->hist[al]++;
                if (ar >= 0 && ga.col[x] >= 0) {
                    uint32_t *z = sum->ae[ar * ga.cols + ga.col[x]];
                    unsigned k = al < dark ? 0 : al > bright ? 2 : 1;

                    z[k] += al;
                    if (k != 1) {
                        z[3 + k / 2]++;
                    }
                }
            }
            if (wr >= 0 && gw.col[x] >= 0) {
                unsigned zi = wr * gw.cols + gw.col[x];
                int u = uv[x & ~1] - 128, v = uv[x | 1] - 128;
                int c = (luma - 16) * 298 + 128;
                /* Limited-range BT.601, as the scene is rendered */
                uint32_t rgb[3] = {
                    MIN(MAX((c + 409 * v) >> 8, 0), 255),
                    MIN(MAX((c - 100 * u - 208 * v) >> 8, 0), 255),
                    MIN(MAX((c + 516 * u) >> 8, 0), 255),
                };
                uint32_t lin[3] = {
                    isp_linear[rgb[0]] * ISP_RAW_R >> 8,
                    isp_linear[rgb[1]],
                    isp_linear[rgb[2]] * ISP_RAW_B >> 8,
                };
                uint32_t ly = (lin[0] * 77 + lin[1] * 150 + lin[2] * 29) >> 8;
                bool white = isp_awb_white(s, lin);

                for (unsigned i = 0; i < 3; i++) {
                    sum->awb_all[zi][i] += lin[i];
                }
                sum->awb_all[zi][3]++;
                if (st->awb_pairs) {
                    uint32_t th = R(st->awb + 0x38);
                    unsigned band = ly < extract32(th, 0, 8) ? 0 :
                                    ly < extract32(th, 8, 8) ? 1 :
                                    ly < extract32(th, 16, 8) ? 2 : 3;
                    uint32_t *bz = sum->awb_band[zi][band];

                    for (unsigned i = 0; i < 3; i++) {
                        bz[i] += lin[i];
                    }
                    bz[3]++;
                    if (white) {
                        bz = sum->awb_white[zi][band];
                        for (unsigned i = 0; i < 3; i++) {
                            bz[i] += lin[i];
                        }
                        bz[3]++;
                    }
                } else if (white &&
                           (st->awb_zone || (ly > least && ly < most))) {
                    for (unsigned i = 0; i < 3; i++) {
                        sum->awb[zi][i] += lin[i];
                    }
                    sum->awb[zi][3]++;
                }
            }
        }
    }
    if (ae) {
        isp_ae_write(s, sum, &ga);
    }
    if (awb && st->awb_zone) {
        isp_awb_write_t32(s, sum, &gw);
    } else if (awb && st->awb_pairs) {
        isp_awb_write_pairs(s, sum, &gw);
    } else if (awb) {
        isp_awb_write(s, sum, &gw);
    }
    isp_stats_grid_free(&ga);
    isp_stats_grid_free(&gw);
}

/* A VIC on MIPI takes its frames from a CSI receiver out of reset */
static bool isp_csi_held(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);

    return s->csi_reset && l->vic_mode &&
           VIC(0, l->vic_mode) == VIC_MODE_MIPI;
}

static void isp_frame(void *opaque)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspPipe *pipe = &l->pipe[0];
    ImageSensorState *sensor = image_sensor_find();

    if (!s->vic_running || !sensor || s->gated) {
        return;
    }
    /*
     * A sensor in standby or without its clock sends nothing, nor does a
     * CSI receiver held in reset: the VIC idles until frames come
     */
    if (!image_sensor_streaming(sensor) || s->mclk_stopped ||
        isp_csi_held(s)) {
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / sensor->fps);
        return;
    }
    if (isp_osd_stalled(s)) {
        if (!s->osd_stalled) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-isp: a shown OSD "
                          "picture's width stops the ISP\n");
        }
        s->osd_stalled = true;
        timer_mod(s->frame_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  NANOSECONDS_PER_SECOND / sensor->fps);
        return;
    }
    s->osd_stalled = false;
    s->frame++;
    R(pipe->int_sta) |= l->int_frame_start;
    for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
        if (R(pipe->ch_en) & BIT(n)) {
            isp_chan_frame(s, sensor, 0, n);
        }
    }
    if (l->stats) {
        isp_stats(s, sensor);
    }
    R(pipe->int_sta) |= l->int_frame_end;
    if (l->tick.sta) {
        R(l->tick.sta) |= l->tick.bits;
    }
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

static void isp_clock(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;

    if (s->gated == !level) {
        return;
    }
    s->gated = !level;
    if (s->gated) {
        timer_del(s->frame_timer);
    } else {
        isp_frame(s);
    }
}

/* A layout without a VIC: a frame from elsewhere, out to the channels */
static void isp_frame_in(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    ImageSensorState *sensor = image_sensor_find();

    if (!level || !sensor || s->gated) {
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
    if (l->set.bits && off == l->set.off) {
        return R(off) | l->set.bits;
    }
    if (l->idle && off == l->idle) {
        return R(off) | 1;
    }
    return R(off);
}

static void ingenic_isp_write(void *opaque, hwaddr off, uint64_t value,
                              unsigned size);

static uint32_t isp_ldl(hwaddr addr)
{
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

/* The register lists set in @mask, from list @base, written in */
static void isp_rcfg(IngenicIspState *s, unsigned base, uint32_t mask)
{
    const IngenicIspLayout *l = isp_layout(s);

    s->rcfg_running = true;
    for (unsigned n = 0; n < 32; n++) {
        uint32_t list, len;

        if (!(mask & BIT(n))) {
            continue;
        }
        list = isp_ldl(R(l->rcfg) + 4 * (base + n));
        len = MIN(isp_ldl(list), RCFG_MAX);
        for (uint32_t i = 1; i <= len;) {
            uint32_t cmd = isp_ldl(list + 4 * i);
            uint32_t cnt = extract32(cmd, 18, 12) + 1;
            hwaddr reg = extract32(cmd, 0, 18) * 4;
            /* bit 31 clear: all the words to one register, a table's port */
            hwaddr step = cmd & BIT(31) ? 4 : 0;

            for (uint32_t k = 0; k < cnt && i + 1 + k <= len; k++) {
                if (reg + step * k < INGENIC_ISP_IOSIZE) {
                    ingenic_isp_write(s, reg + step * k,
                                      isp_ldl(list + 4 * (i + 1 + k)), 4);
                }
            }
            i += cnt + 1;
        }
    }
    s->rcfg_running = false;
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

        /* the T33's lists push 0s that queue no buffer */
        if (cr == r->y_push) {
            if (v || !l->rcfg) {
                isp_push(ch->y, ch->y_rd, &ch->y_n, v);
            }
            return;
        }
        if (cr == r->uv_push) {
            if (v || !l->rcfg) {
                isp_push(ch->uv, ch->uv_rd, &ch->uv_n, v);
            }
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
        hwaddr to = l->shadow_base + (v & ~3);

        if (!s->shadow_pending) {
            s->shadow_value = v;
        } else if (to != l->shadow && to < INGENIC_ISP_IOSIZE) {
            s->shadow_pending = false;
            ingenic_isp_write(s, to, s->shadow_value, 4);
            return;
        }
        s->shadow_pending = !s->shadow_pending;
        return;
    }
    if (l->shadow_open && off == l->shadow_open) {
        s->shadow_pending = false;
    }
    if (l->tick.sta && off == l->tick.sta) {
        return;
    }
    if (l->tick.clr && off == l->tick.clr) {
        R(l->tick.sta) &= ~v;
        isp_update(s);
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
    if (l->rcfg && (off == l->rcfg_trig || off == l->rcfg_trig + 4) &&
        !s->rcfg_running) {
        isp_rcfg(s, off == l->rcfg_trig ? 0 : 32, v);
    }
    /* a 2-bit picture's palette entry keeps one bit of alpha */
    if (l->osd && l->osd->palette && off >= l->osd->palette &&
        off < l->osd->palette + 0x10 * DIV_ROUND_UP(l->osd->pics, 4)) {
        v &= 0x01ffffff;
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
    if (isp_layout(s)->osd) {
        memcpy(&R(isp_layout(s)->osd->csc), isp_osd_csc, sizeof(isp_osd_csc));
    }
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

static void isp_csi_reset(void *opaque, int n, int level)
{
    INGENIC_ISP(opaque)->csi_reset = level;
}

static void isp_mclk(void *opaque, int n, int level)
{
    INGENIC_ISP(opaque)->mclk_stopped = !level;
}

static void isp_soft_reset(void *opaque, int n, int level)
{
    if (level) {
        ingenic_isp_reset_hold(opaque, RESET_TYPE_COLD);
        ingenic_isp_reset_exit(opaque, RESET_TYPE_COLD);
    }
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
    qdev_init_gpio_out_named(DEVICE(obj), s->ivdc_frame, "ivdc-frame",
                             INGENIC_ISP_SENSORS);
    if (!l->vics) {
        qdev_init_gpio_in_named(DEVICE(obj), isp_frame_in, "frame", 1);
    }
    qdev_init_gpio_in_named(DEVICE(obj), isp_clock, "clock", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_soft_reset, "reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_csi_reset, "csi-reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_mclk, "mclk", 1);
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
    .version_id = 4,
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
        VMSTATE_BOOL_V(gated, IngenicIspState, 2),
        VMSTATE_BOOL_V(csi_reset, IngenicIspState, 3),
        VMSTATE_BOOL_V(mclk_stopped, IngenicIspState, 4),
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
    for (unsigned i = 0; i < 256; i++) {
        isp_linear[i] = lrint(255.0 * pow(i / 255.0, 2.2));
    }
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
        .name = TYPE_INGENIC_T33_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t33_isp_layout,
    }, {
        .name = TYPE_INGENIC_T32PRO_ISP,
        .parent = TYPE_INGENIC_ISP,
        .class_init = ingenic_isp_soc_class_init,
        .class_data = &ingenic_t32pro_isp_layout,
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
