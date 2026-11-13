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
 *   its channels at 0x9900 + N * 0x100, and a front crop window of the
 *   frame in 0x9860 (x << 16 | y) and 0x9864 (w << 16 | h) that every
 *   channel's scaled size then shows whole; the T40 one for each of its two
 *   pipelines, at 0x16100 and 0x17100; the T41 one, at 0xf0100. The
 *   driver queues a buffer by writing its Y and UV addresses to +0x6c and
 *   +0x84; a frame fills the oldest and moves it to the done FIFO, which
 *   the driver drains while +0x7c bit 0 (empty) is clear (bits 11:8 count
 *   the entries): +0x74 gives the Y address and pops the entry, +0x8c and
 *   +0x90 its UV address and frame number. With no buffer queued the
 *   channel drops the frame, and it does while the last is undrained: the
 *   T40 driver reports only the last entry it drains, and a guest can fall
 *   a frame behind here where silicon would not. On a T23DL, T31X and
 *   T40XP a frame goes out on the channels enabled as it reached the MSCA
 *   (when the T23's and T40's busy bits set): one disabled mid-frame still
 *   puts it out, one enabled mid-frame waits for the next frame. A T31X
 *   takes the buffers queued as the frame reaches the MSCA: channel 0's
 *   counter (0x9888) stays held when one is pushed after that, runs from 0
 *   at the next frame and the whole frame lands. A push reads back
 *   address | 1 until its frame lands. Its +0x7c read 1, then 0x11001
 *   with a frame done, with the consumer frozen; the stock driver stops
 *   draining with that (it reads the count in bits 11:8), so the model
 *   keeps the empty bit and the count there. The other parts are not
 *   measured.
 *
 * - The T23's MSCA, at 0xd100 + N * 0x100 (channel enable 0xd040), packs
 *   the same registers: scaled size +0x4, crop +0x8 and +0xc, Y push +0x30,
 *   done +0x38, empty +0x3c and stride +0x40, UV push +0x50, done +0x58
 *   and stride +0x60, FIFO clears +0x44, +0x48, +0x64 and +0x68 (bit 0:
 *   the dual-sensor driver writes 0 there after a push). It has no frame
 *   number, and its driver reads the UV address before the Y read pops
 *   the entry, the dual-sensor driver after it.
 *
 * - The T23's dual-sensor driver (tx-isp's "-mult" firmware) shares the
 *   one VIC, ISP and MSCA between two or three sensors behind a MIPI
 *   switch and, on the core's "all done" (0xc8 bit 15, enable 0xc0, clear
 *   0xc4), gives the channels the next sensor's buffers. Given the
 *   switch's GPIOs (IMP_ISP_MultiCamera_SetSwitchgpio) it flips them as
 *   each frame ends. Without, it hands the pin its mipi_switch_gpio
 *   parameter names (PA7, the default, PB22 or PB30) to the VIC and sets
 *   VIC +0x200 to 0x80010001: the VIC flips the switch between two sensors
 *   itself; it starts the VIC before it sets that up. Its reset writes
 *   0x24 and waits for 0x28 bit 0. The model's VIC 0 takes each frame from
 *   the input the switch selects (see isp_vic_sensor()), the first a frame
 *   time after the start, as a sensor's comes, and each later one once the
 *   driver has dealt with the last (see isp_frame_held()).
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
 *   that sensor, and the encoders take the scene from the sensor that
 *   sent the last frame over it (the T23's direct mode 2 sends both
 *   sensors' mains, channels 0 and 3, over the one ring; the encoder's
 *   source address does not say whose). On the T23 VIC 0 holds its next
 *   frame while the Helix codes one sent over IVDC or the IVDC waits for
 *   it to take one (see isp_frame_held()). The T23's and T41's drivers
 *   complete frame channel 0 on the core's frame start (the T41's at
 *   0x40028 bit 20), and the T41's never drains channel 0's done FIFO
 *   then: its IVDC frames leave none.
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
 * - The T23's and T40's ports hold a run in a RAM of pairs, 512 on a T23DL
 *   and 128 on a T40XP: 0x101 to the open register (0xe000, 0x50320) puts
 *   the next pair at entry 0, pairs then go in, going round past the last
 *   entry (a pair outside an open run is dropped), the close (bits 1:0 =
 *   2) ends it, and kicking the run (0xe100 bit 0, the pairs less one in
 *   bits 15:4 and the first entry in 24:16; 0x50420 bit 0, in bits 30:24
 *   and 22:16) writes those entries, going round too, once the MSCA is
 *   between frames: at once from a little after a frame's end (0.16 ms on
 *   the T23, 0.25 ms on the T40) to its next frame's line 17 or 21, else
 *   at the end of the frame going through. The entries stay and a kick
 *   writes them again. The status (0xe00c, 0x5032c) reads 0x10 while a run
 *   is open, bit 0 once closed on a T23 and once written on a T40; 0x50420
 *   reads back the length and first entry, with bit 8 while the run waits,
 *   and 0x50328 the last value a run wrote. The
 *   model's frame ends as it is sent, and its MSCA stays between frames
 *   for 1/16 of a frame time (the T23's 1.9 ms in 29.6, the T40's 2.0 in
 *   33.3), its second pipeline's frames aside. The T41's port writes each
 *   pair at once.
 *
 * - The T23's, T32's and T40's MSCA has a read-only word of busy bits, bit
 *   n for channel n (0xd054; 0xf0054; 0x160e0, and 0x170e0 for the second
 *   pipeline), as a T23DL, a T32LQ and a T40XP read it: the bits of the
 *   channels enabled then set as a frame reaches the MSCA (VIC line 17, 32,
 *   21), and a channel's clears as it puts that frame out into a buffer,
 *   0.16 ms (0.81 ms, 0.25 ms) after the frame's last line; one with no
 *   buffer stays busy, until the next frame
 *   sets the bits again (then without it, if it was disabled since). The
 *   model's MSCA takes its next frame as it stops being between frames.
 *   The T31's (0x9808, as a T31X reads it) sets the same way, at line 19,
 *   but a channel that puts the frame out stays busy unless it was
 *   disabled by then; between frames a channel's bit clears as it is
 *   disabled, but for one left busy by a frame it dropped, and comes back
 *   only with the next frame. tx-isp's channel stop disables the channel
 *   and waits up to 3 s for the bit to clear. The T31's channel enable
 *   bits 19:16 (the driver sets them as it starts a channel) clear as the
 *   next frame reaches the MSCA.
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
 * - An ISP with a lens correction (the T21's, T23's, T31's, T40's and T41's)
 *   corrects distortion along each row before its MSCA
 *   (ingenic-isp-hldc.c) once its bypass bit (layout hldc_bypass) is clear,
 *   on the frame size in the statistics' size register; a channel of
 *   another size comes down from that through the model's own
 *   nearest-pixel scaler.
 *
 * - Interrupts: a pipeline's status bit N is channel N's frame done, and
 *   the frame start (and on the T31 the frame end) has a bit too; the
 *   enable register gates the ISP interrupt and the clear register clears.
 *   A VIC's status (bit 0: frame done, bit 4 on the T40: frame start) is
 *   masked by VIC_INT_MSK and cleared through VIC_INT_CLR.
 *
 * - The T32, T40 and T41 drivers request the ISP idle through 0x200 bit 0
 *   before they reconfigure, and spin until 0x204 bit 0 says so; frames
 *   take no time here, so the ISP is always idle (a T41NQ's 0x204 reads 1
 *   while it streams too).
 *
 * - On a T41NQ the idle request (0x200 bit 0, which reads back 0) stops
 *   the ISP from the frame in flight on: the VIC goes on taking frames and
 *   interrupting, but the core puts none out and raises nothing. Only a
 *   write of 0x24 bit 2 and a latch (0x1000 bit 0) resume it, in either
 *   order (the driver's DMSC and WDR changes do both after the request):
 *   bit 2 on its own stops it the same way, until a latch, and takes one
 *   that is pending (0x1000 reading 1) at once, as a write of 0x1000 bit 0
 *   while it waits does, 0x1000 then reading 0. The frame in flight as it
 *   resumes is lost; the next goes out. A latch alone does not resume an
 *   idle request. 0x24 bits 1 and 3 to 23 changed nothing seen: frames
 *   go out, a pending latch stays.
 *
 * - The CPM gates the ISP's clock (CLKGR0 bit 23, 19 on the T32 and
 *   T33), which stands it and its VICs still where they are, and resets
 *   it (SRBC's TIZIANO_SR): tx-isp gates it as it closes and resets it as
 *   it opens, before it gives it new buffers. Run on, the model would
 *   write frames and statistics into the buffers a closed session has
 *   given back. A gated T21N's, T23DL's, T31X's and T40XP's words read the
 *   last value read and take no writes (isp_clock()).
 *
 * - The T41's registers come out of reset, at power-on and through SRBC's
 *   bit 22 alike, as a T41NQ reads them (ingenic-t41-isp-reset.c.inc),
 *   the others' at 0. A T41 answers only with CLKGR bit 23 open and the
 *   divider behind that gate (CPM 0x80) at a rate it takes: at its reset
 *   value (SCLKA undivided) every register reads 0x20220304, and with its
 *   stop bit (27) set, as tx-isp leaves it as it closes, a read hangs the
 *   bus (its two other dividers, 0xa8 and 0xac, may stay at reset, and
 *   0xa8 stopped). The model answers regardless.
 *
 * - A T41's write keeps only the bits a T41NQ keeps (its core, the tables
 *   from 0x6000 to 0x1f000, the VIC, the MSCA and its OSD), the others
 *   reading as they came out of reset (ingenic-t41-isp-masks.c.inc). Left
 *   as written are words a T41NQ changes itself while it streams, the DMA
 *   addresses, enables and FIFOs, and the HLDC (its own).
 *
 * - A T41's 0x24 keeps bits 31:24; bits 23:0 read 0 at once. Bit 0 resets
 *   the ISP but for its VIC, which goes on sending frames into an ISP that
 *   puts none out until the driver sets it up again. Bit 2 (the driver's,
 *   around a DMSC change) stopped a streaming T41NQ's frames until tx-isp
 *   set it up again; the model leaves it and bits 23:1.
 *
 * - Some of a T41's words take a write only when the ISP latches them, and
 *   read what it latched last (ingenic_t41_isp_latched): 0x40 at the end
 *   of each frame (its last line in), 0x44 at each start, and the DMSC's,
 *   YSP's, SDNS's and MSCA's at the first frame start after bit 0 of their
 *   trigger is written (0xa19c, 0x13080, 0x14054, 0xf0010), which reads 1
 *   until then (the MDNS's 0xf004 and YDNS's 0x10004 too). With no frames
 *   nothing latches, but 0x1000 bit 0 latches 0x40 and 0x44 at once; while
 *   frames run it stays set, and writes to them take effect at once, until
 *   0 is written. The CDNS's trigger (0x1c004) reads 1 on a T41NQ, which
 *   never latches it; the model leaves it.
 *
 * - The T41's shadow port is a RAM of 512 (value, offset) pairs: writing
 *   0x101 to 0xf8000 opens it at entry 0 (0xf800c reads 0x10), the pairs
 *   go to 0xf8004 (dropped while closed; past entry 511 they go round to
 *   0) and 0x1010102 closes it (0xf800c reads 1). Writing n - 1 << 4 | 1
 *   to 0xf8100 has the ISP write n entries from 0 at the next frame's end,
 *   once, as the CPU would, going round the RAM past 511 (0 there
 *   cancels); entries stay until written over, through a reset by 0x24
 *   too. 0xf8008 reads the last value a run wrote.
 *
 * - The T32's port at 0xf8000 is the T41's with a RAM of 256 pairs, as a
 *   T32LQ runs it: a run goes at its frame's end, or at once while the
 *   MSCA is between frames (from a little after a frame's end to the next
 *   frame's line 30 to 35, 2.1 ms on in 33.3), and a pair into the RAM
 *   clears 0xf8008.
 *
 * - The T40's VICs come out of reset and keep writes as a T40XP's do. Its
 *   VIC 2 has no pipeline behind it: the driver takes a third sensor there
 *   (DVP, +0xc = 3) only raw, through the VIC's own capture (MDMA, +0x300),
 *   which any VIC has (the T23's, T31's, T32's and T41's too, their
 *   drivers' raw snapshot): control +0x300 (bit 31 on, bits 2:0 the format: 0
 *   raw, 16 bits a sample, the sensor's bits at the top as a T40XP's 12
 *   are; 7 NV12, 6 NV21), size +0x304, strides +0x310 and +0x314, five
 *   slots' Y and UV addresses at +0x318 and +0x32c. Each frame goes into
 *   the next slot, round the five, while bits 19:16 allow (bit 19 every
 *   frame, else the count, down one a frame, as a T40XP's goes from 1 to 0
 *   after one frame), sets the VIC's second status bit 0 and leaves its
 *   address in +0x380; 1 to +0x308 starts again from slot 0 and clears
 *   +0x380. +0x3a4 counts the VIC's frames. The model's sensors send RGGB
 *   and no second exposure, which a WDR sensor's would take into slots at
 *   +0x340, done at +0x388, second status bit 1.
 *   On a T40XP streaming with its 3D noise reduction on (0x40 bit 13
 *   clear) a capture used up its count with no frame written; tx-isp's
 *   snapraw bypasses that block first. The model captures regardless.
 *
 * - The T40's joint mode (two sensors' pictures in one) is the driver's:
 *   it gives the second pipeline's channel the first's buffer, offset by
 *   the first picture's width or height, both with the joint stride.
 *
 * VIC n takes the frames of the sensor on input n, into pipeline n where a
 * layout has it; a board's only sensor feeds whichever VIC runs.
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
#include "qemu/thread.h"
#include "qemu/main-loop.h"
#include "qemu/cutils.h"
#include "system/qtest.h"
#include "qapi/error.h"
#include "hw/core/cpu.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "migration/qemu-file.h"
#include "io/channel-buffer.h"
#include "hw/core/resettable.h"
#include "system/address-spaces.h"
#include "hw/misc/ingenic-isp.h"
#include "hw/misc/ingenic-cgu.h"
#include "hw/sensor/image-sensor.h"
#ifdef CONFIG_AVX2_OPT
#include "host/cpuinfo.h"
#endif
#include "trace.h"

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
#define VIC_MODE_DVP        3
#define VIC_INPUT           0x00c       /* T40: its mode, VIC_MODE_* */
#define VIC_DVP_FMT         0x010       /* bits 19:17: 8, 10 or 12 bits */
#define VIC_MIPI_FMT        0x014       /* RAW8, 10, 12; 7: 16 */
#define VIC_INT_MDMA        BIT(0)      /* in the second status word */

/* A VIC's raw capture, from its @mdma */
#define MDMA_CTRL           0x00
#define MDMA_ON             BIT(31)
#define MDMA_ALL            BIT(19)     /* in the count: every frame */
#define MDMA_SIZE           0x04        /* width << 16 | height */
#define MDMA_RESET          0x08        /* bit 0: back to slot 0 */
#define MDMA_Y_STRIDE       0x10
#define MDMA_UV_STRIDE      0x14
#define MDMA_Y              0x18        /* the 5 slots' addresses */
#define MDMA_UV             0x2c
#define MDMA_SLOTS          5
#define MDMA_KICK           0x70        /* write 1: one more buffer queued */
#define MDMA_DONE           0x80        /* the last frame's address */
/*
 * The T32 Pro's queues: a buffer pushed at +0x20 (Y) or +0x30 (UV), the
 * done ones read at +0x28 and +0x38, the queues cleared at +0x48 and +0x58
 */
#define MDMA_PUSH_Y         0x20
#define MDMA_PUSH_UV        0x30
#define MDMA_DONE_Y         0x28
#define MDMA_DONE_UV        0x38
#define MDMA_CLEAR_Y        0x48
#define MDMA_CLEAR_UV       0x58
#define VIC_INT_MDMA_UV     BIT(1)
#define MDMA_RAW16          0
#define MDMA_NV21           6
#define MDMA_NV12           7

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
    hwaddr busy;                        /* its MSCA's busy bits, if any */
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
 *   in the white region of +0x28 to +0x34 (awb_span()) whose luma is
 *   above +0x38 bits 7:0 and below bits 15:8, as a T31X's and a T31AL's
 *   held to bounds of our own; the quads have no IR. Its sums are of the
 *   raw picture, in linear light: the model undoes the scene's gamma and
 *   white balance for them (ISP_RAW_R, ISP_RAW_B; a sensor model with a
 *   response of its own for the part has that, see isp_raw_factors()),
 *   without which no grey of a balanced scene is in the drivers' white
 *   region. Where the part has a response, the quads come from a picture
 *   without the light's tint, which the scene's 8-bit chroma rounds into
 *   its ratios; the sums keep 1/256th of a level (isp_awb_sums_round()).
 *
 * - The T40's and T41's AWB block, at 0x18000, keeps those registers (a
 *   write of 1 to +0x0 loads them, which reads 0 on a T41NQ; the model
 *   keeps it and starts on it) but sorts each zone's quads into 4 bands by
 *   luma, split at the bytes of +0x38 (10, 128 and 230 from the drivers),
 *   and gives each band a pair of records (128 bytes a zone), or only band
 *   +0x4c bits 26:25 its pair (32 bytes) with bit 28 set. A record is the
 *   R, G, B and IR sums and number packed 22, 22, 22, 22 and 14 bits into
 *   16 bytes, the first of all the band's quads, the second of those in the
 *   white region of +0x28 to +0x34 (awb_span(), as a T40XP's and a
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
 * - The T33's and T32 Pro's blocks have the T32's registers, which their
 *   tx-isp loads from register lists (the T33's AWB is list 0x1d and its AE
 *   0x1e, the T32 Pro's 0x1e and 0x1f), and the zone sizes of 0x160f4 and
 *   0x17044 (height << 16 | width, in quads), but write whole fields, as
 *   those modules read them (tisp_t33_awb_long_alogrithm,
 *   tisp_ae_weight_mean: from the firmware alone, no board of either part
 *   is on the bench). A buffer starts with a 16-bit tag in 8 bytes, of
 *   which nothing else is read, and carries the tag again after its data;
 *   a driver drops a buffer whose two differ, so they are the frame's
 *   number here. AWB: 0x7400 bytes a buffer; a zone is 28 bytes, the R, G
 *   and B sums of all its quads and then of its white ones (32 bits each)
 *   and their numbers (16 bits each), a row padded to a multiple of 8
 *   bytes. A T33 steps through four buffers by the index at +0x50, a
 *   T32 Pro's driver moves the one address itself. AE: a histogram of 256
 *   32-bit bins from byte 8 of each buffer, and from 0x410 on the zone
 *   buffer (the addresses at +0x2c): 16 bytes a zone, three luma sums (32
 *   bits) and how many quads are below and above the levels of +0x28 (16
 *   bits), 15 zones a row whatever the columns used. The lists carry the
 *   index registers' words too, which the blocks do not take from them.
 *
 * - The T21's blocks are the T31's at 0x700 (AE) and 0x800 (AWB), raising
 *   0x84 bits 16, 17 (AE zones, histogram) and 18 (AWB), in an older
 *   layout, as its tx-isp firmware reads them and a T21N writes them: the
 *   histogram is the 256 luma bins alone (1 KiB, no IR), 20 bits a word,
 *   each odd word carrying in bits 27:20 the frame tag the AE block counts
 *   in +0x50 bits 15:8; an AWB zone is the R, G and B sums and the number
 *   (21, 21, 21 and 13 bits, no IR) and that tag less one in bits 83:76.
 *
 * - The T21's blocks run whatever their first words hold: a T21N's +0x700
 *   and +0x800 read 0 while its AE and AWB statistics come, a 1 written to
 *   either reads back for a few frames and falls to 0 again, and its tx-isp
 *   writes +0x700 only when it updates the AE parameters, +0x800 never.
 *
 * - The T31's AE block counts its frames in +0x50 bits 15:8, the tag its
 *   other blocks write into their buffers.
 *
 * A buffer at 0 is taken as none yet: the T23's Linux driver enables AWB
 * before it gives it buffers, while frames from Zeratul's RISC-V core may
 * still be coming in.
 */
typedef struct IngenicIspStats {
    hwaddr size, ae, awb;           /* awb 0: none */
    hwaddr ae_en, awb_en;           /* the enables, if not at ae and awb */
    hwaddr awb_zone;                /* T32: every AWB zone's size */
    hwaddr ae_zone;                 /* T33, T32 Pro: every AE zone's */
    uint32_t int_ae_zones, int_ae_hist, int_awb;
    unsigned sum_bits, count_bits;  /* an AE zone's fields */
    unsigned head;                  /* bytes before a buffer's data */
    bool hist_pairs;                /* two 24-bit bins a 64-bit word */
    bool flat;                      /* T33, T32 Pro: whole-field records */
    bool hist_ir;                   /* T23, T31: the second, of IR */
    bool awb_pairs;                 /* T40, T41: pairs of AWB records */
    hwaddr sec;                     /* pipeline 1's blocks, this far up */
    bool older;                     /* T21: its histogram, AWB zones, tags */
    bool tagged;                    /* T31: AE counts the frames' tags */
    bool t31;                       /* T31: its ADR, defog and AF blocks */
    bool t41;                       /* T41: its AF, GSM, LCE, ADR, defog */
                                    /* and TMO blocks */
} IngenicIspStats;

static const IngenicIspStats ingenic_isp_stats = {
    .size = 0x004, .ae = 0xa000, .awb = 0xb000,
    .int_ae_zones = BIT(26), .int_ae_hist = BIT(27), .int_awb = BIT(30),
    .sum_bits = 21, .count_bits = 13, .hist_ir = true,
};

static const IngenicIspStats ingenic_t31_isp_stats = {
    .size = 0x004, .ae = 0xa000, .awb = 0xb000,
    .int_ae_zones = BIT(26), .int_ae_hist = BIT(27), .int_awb = BIT(30),
    .sum_bits = 21, .count_bits = 13, .hist_ir = true,
    .tagged = true, .t31 = true,
};

static const IngenicIspStats ingenic_t21_isp_stats = {
    .size = 0x004, .ae = 0x700, .awb = 0x800,
    .int_ae_zones = BIT(16), .int_ae_hist = BIT(17), .int_awb = BIT(18),
    .sum_bits = 21, .count_bits = 13, .older = true,
};

static const IngenicIspStats ingenic_t40_isp_stats = {
    .size = 0x080, .ae = 0x19000, .awb = 0x18000,
    .int_ae_zones = BIT(4), .int_ae_hist = BIT(5), .int_awb = BIT(3),
    .sum_bits = 22, .count_bits = 14,
    .awb_pairs = true,
    .sec = 0x200,
};

static const IngenicIspStats ingenic_t41_isp_stats = {
    .size = 0x080, .ae = 0x19000, .awb = 0x18000,
    .int_ae_zones = BIT(4), .int_ae_hist = BIT(5), .int_awb = BIT(3),
    .sum_bits = 22, .count_bits = 14,
    .awb_pairs = true,
    .t41 = true,
};

static const IngenicIspStats ingenic_t32_isp_stats = {
    .size = 0x080, .ae = 0x1703c, .awb = 0x1603c,
    .ae_en = 0x17010, .awb_en = 0x16010, .awb_zone = 0x160f4,
    .int_ae_zones = BIT(4), .int_ae_hist = BIT(5), .int_awb = BIT(3),
    .sum_bits = 22, .count_bits = 14,
    .head = 8, .hist_pairs = true,
};

static const IngenicIspStats ingenic_t33_isp_stats = {
    .size = 0x080, .ae = 0x1703c, .awb = 0x1603c,
    .ae_en = 0x17010, .awb_en = 0x16010,
    .ae_zone = 0x17044, .awb_zone = 0x160f4,
    .int_ae_zones = BIT(4), .int_ae_hist = BIT(5), .int_awb = BIT(3),
    .flat = true,
};

/*
 * An interrupt word of its own, enable 1, raised each frame: @start as the
 * frame starts, @bits as it ends; @gated: a bit whose enable is 0 does not
 * latch, so enabling it later raises nothing until the next frame
 */
typedef struct IngenicIspTick {
    hwaddr en, sta, clr;
    uint32_t bits;
    uint32_t start;
    bool gated;
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
    unsigned mask_en_shift;         /* or bit c + this */
    unsigned mask_load;             /* bit c + this of mask_en loads */
                                    /* channel c's positions and sizes */
                                    /* as a frame reaches the MSCA; 0: */
                                    /* they take writes at once */
    bool mask_vu;                   /* the colour's Cr in 15:8, Cb 7:0 */
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

#define OSD_MASKS           INGENIC_ISP_MASKS
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

/*
 * The T31's: masks alone, enabled by the channel enable's bits 5:3 and
 * loaded by its bits 19:17, as a T31X shows them
 */
static const IngenicIspOsd ingenic_t31_isp_osd = {
    .mask = 0x9938, .mask_chan = 0x100, .mask_step = 0xc, .mask_en = 0x9804,
    .mask_en_shift = 3, .mask_load = 17, .mask_vu = true,
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

/* A register's value out of reset */
typedef struct IngenicIspResetVal {
    uint32_t off, val;
} IngenicIspResetVal;

#include "ingenic-t41-isp-reset.c.inc"

/* A run of @n words from @off whose writes keep only the bits in @mask */
typedef struct IngenicIspMask {
    uint32_t off, n, mask;
} IngenicIspMask;

#include "ingenic-t41-isp-masks.c.inc"

/*
 * Each of the T40's VICs, as a T40XP's read out of reset (in U-Boot, the
 * ISP's clock opened) and kept writes (its VIC 2, idle): words from the
 * VIC's base. The others read 0 and keep every bit.
 */
static const IngenicIspResetVal ingenic_t40_vic_reset[] = {
    { 0x004, 0x07800438 }, { 0x00c, 0x00000002 }, { 0x014, 0x00000002 },
    { 0x018, 0x00000f00 }, { 0x060, 0x00800800 }, { 0x064, 0x009d09d0 },
    { 0x070, 0x00006002 }, { 0x074, 0x00007003 }, { 0x0c0, 0x00eb8080 },
    { 0x0c4, 0x00108080 }, { 0x0c8, 0x0029f06e }, { 0x0cc, 0x00913622 },
    { 0x0d0, 0x00515af0 }, { 0x0d4, 0x00aaa610 }, { 0x0d8, 0x00d21092 },
    { 0x0dc, 0x006acade }, { 0x0e0, 0x00eb8080 }, { 0x0e4, 0x00108080 },
    { 0x0e8, 0x0029f06e }, { 0x0ec, 0x00913622 }, { 0x0f0, 0x00515af0 },
    { 0x0f4, 0x00aaa610 }, { 0x0f8, 0x00d21092 }, { 0x0fc, 0x006acade },
    { 0x100, 0x000002d0 }, { 0x10c, 0x0002c000 }, { 0x110, 0x07800000 },
    { 0x1a4, 0x00100010 }, { 0x1a8, 0x00004404 }, { 0x1ac, 0x00004444 },
    { 0x1b0, 0x00000010 }, { 0x304, 0x07800438 }, { 0x310, 0x00000f00 },
    { 0x314, 0x00000f00 }, { 0x3e0, 0x00000081 }, { 0x3e4, 0x00000081 },
};

/*
 * The T40's two MDNS at reset, as a T40XP's idle second one reads (a T41NQ's
 * first reads the same): the ID, enables and tuning; the driver writes the
 * rest
 */
static const IngenicIspResetVal ingenic_t40_isp_reset[] = {
    { 0x0f000, 0x20191209 }, { 0x0f00c, 0x00000040 },
    { 0x0f020, 0x00000011 }, { 0x0f100, 0x00000010 },
    { 0x0f104, 0x00000808 }, { 0x0f108, 0x01000100 },
    { 0x0f10c, 0x00100010 }, { 0x0f114, 0x00890000 },
    { 0x0f118, 0x00890000 }, { 0x0f11c, 0x00890000 },
    { 0x0f120, 0x00020010 }, { 0x0f150, 0x0000ff00 },
    { 0x0f1b0, 0x00e60000 }, { 0x0f234, 0x0000ff00 },
    { 0x0f238, 0x00e60000 },
    { 0x0f800, 0x20191209 }, { 0x0f80c, 0x00000040 },
    { 0x0f820, 0x00000011 }, { 0x0f900, 0x00000010 },
    { 0x0f904, 0x00000808 }, { 0x0f908, 0x01000100 },
    { 0x0f90c, 0x00100010 }, { 0x0f914, 0x00890000 },
    { 0x0f918, 0x00890000 }, { 0x0f91c, 0x00890000 },
    { 0x0f920, 0x00020010 }, { 0x0f950, 0x0000ff00 },
    { 0x0f9b0, 0x00e60000 }, { 0x0fa34, 0x0000ff00 },
    { 0x0fa38, 0x00e60000 },
    { 0x0fb00, 0x00000100 }, { 0x0fb04, 0x00001000 },
};

/* The T32's MDNS: its two ID words and its idle strides, as a T32NQ reads */
static const IngenicIspResetVal ingenic_t32_isp_reset[] = {
    { 0x0d000, 0x24041703 }, { 0x0d004, 0x24041928 },
    { 0x0d044, 0x00000780 }, { 0x0d04c, 0x00000780 },
};

static const IngenicIspMask ingenic_t40_vic_masks[] = {
    { 0x008,  1, 0x00000003 }, { 0x00c,  1, 0x00000007 },
    { 0x014,  1, 0x0000003f }, { 0x040,  8, 0x00000000 },
    { 0x060,  2, 0x00ffffff }, { 0x068,  2, 0x00000000 },
    { 0x070,  2, 0x00ffffff }, { 0x078, 18, 0x00000000 },
    { 0x0c0, 16, 0x00ffffff }, { 0x100,  1, 0x0000ffff },
    { 0x10c,  1, 0x03f7ff37 }, { 0x114,  3, 0x0000ffff },
    { 0x120,  1, 0x0000000f }, { 0x124, 31, 0x00000000 },
    { 0x1a0,  1, 0x0000033f }, { 0x1a8,  1, 0x00007777 },
    { 0x1ac,  1, 0x00017777 }, { 0x1b0,  2, 0x0000ffff },
    { 0x1b8,  2, 0x00000000 }, { 0x1c0,  1, 0x00000013 },
    { 0x1c4,  3, 0x00000000 }, { 0x1d0,  2, 0x0000001f },
    { 0x1d8,  4, 0x00000000 }, { 0x1ec,  1, 0x0000000f },
    { 0x1f0, 68, 0x00000000 }, { 0x308,  1, 0x00000000 },
    { 0x30c,  1, 0x0fff0003 }, { 0x310,  2, 0x0000ffff },
    { 0x368, 10, 0x00000000 }, { 0x390,  1, 0x00000311 },
    { 0x39c, 25, 0x00000000 },
};

/*
 * A run of @n words a write reaches only when the ISP latches them: at the
 * first frame start after bit 0 of @trig is written, or at every frame's
 * start or end. A run of none is a trigger with no word measured behind it.
 */
typedef struct IngenicIspLatch {
    uint32_t off, n, trig;
} IngenicIspLatch;

#define LATCH_START         1
#define LATCH_END           2

/* As a streaming T41NQ latched them, in order; its channels' at 0x100 */
static const IngenicIspLatch ingenic_t41_isp_latched[] = {
    { 0x00040,  1, LATCH_END },     /* bypasses */
    { 0x00044,  1, LATCH_START },   /* enables */
    /* DMSC */
    { 0x0a000,  1, 0x0a19c }, { 0x0a008,  1, 0x0a19c },
    { 0x0a048,  1, 0x0a19c }, { 0x0a05c,  2, 0x0a19c },
    { 0x0a0b0,  2, 0x0a19c }, { 0x0a184,  1, 0x0a19c },
    { 0x0a1c0,  1, 0x0a19c }, { 0x0a1cc,  1, 0x0a19c },
    { 0x0a1d8,  1, 0x0a19c }, { 0x0a1e8,  1, 0x0a19c },
    { 0x0a1fc,  1, 0x0a19c }, { 0x0a204,  2, 0x0a19c },
    { 0x0a214,  1, 0x0a19c }, { 0x0a2d4,  8, 0x0a19c },
    { 0x0f004,  0, 0x0f004 },       /* MDNS */
    { 0x10004,  0, 0x10004 },       /* YDNS */
    /* YSP */
    { 0x1301c,  1, 0x13080 }, { 0x13050,  3, 0x13080 },
    { 0x1312c, 16, 0x13080 }, { 0x1318c,  1, 0x13080 },
    /* SDNS */
    { 0x14004,  1, 0x14054 }, { 0x14010,  1, 0x14054 },
    { 0x14018,  4, 0x14054 }, { 0x1402c,  3, 0x14054 },
    { 0x14088,  4, 0x14054 }, { 0x14140,  4, 0x14054 },
    /* MSCA: crops, scaled sizes and crops of its channels, steps, CSC */
    { 0xf0028,  8, 0xf0010 }, { 0xf0080,  2, 0xf0010 },
    { 0xf00a0,  1, 0xf0010 }, { 0xf00a8,  1, 0xf0010 },
    { 0xf00b0,  1, 0xf0010 }, { 0xf0100,  1, 0xf0010 },
    { 0xf0128,  2, 0xf0010 }, { 0xf01d0,  1, 0xf0010 },
    { 0xf0200,  1, 0xf0010 }, { 0xf0228,  2, 0xf0010 },
    { 0xf02d0,  1, 0xf0010 }, { 0xf0300,  1, 0xf0010 },
    { 0xf0328,  2, 0xf0010 }, { 0xf03d0,  1, 0xf0010 },
    { 0xf0704, 12, 0xf0010 }, { 0xf08e0,  4, 0xf0010 },
};

/*
 * A shadow port that queues its pairs: a write to @sta's open register
 * (bits 1:0 = 1) empties the queue and 2 closes it, and bit 0 of @kick
 * writes the first (@kick >> @len_shift & @len_mask) + 1 pairs once the
 * MSCA is between frames
 */
typedef struct IngenicIspShadowRun {
    hwaddr kick;
    hwaddr sta;                     /* bit 4: open */
    unsigned len_shift;
    uint32_t len_mask;
    uint32_t busy;                  /* @kick's bit while a run waits, which */
                                    /* reads back the length too */
    bool done_on_close;             /* @sta bit 0: closed, or a run written */
    unsigned ram;                   /* the pairs it holds */
    unsigned start_shift;           /* @kick's first entry */
    uint32_t start_mask;
    hwaddr last;                    /* reads the last value a run wrote */
} IngenicIspShadowRun;

/* Offsets of 0 are registers a layout lacks */
/* A 3D noise reducer (MDNS), see isp_mdns_frame(); offsets from its base */
typedef struct IngenicIspMdns {
    hwaddr base;
    uint32_t id;                    /* +0x0 reads it, 0: as the layout's */
    uint32_t refresh_reads;         /* +0x18 after a write */
    hwaddr y, uv;                   /* the reference's address, its stride */
                                    /* at +4 */
    hwaddr sta[4];                  /* the statistics buffers', likewise */
    hwaddr grid;                    /* the first grid's size, 0: none */
    hwaddr blocks;                  /* or the blocks across (15:0) and */
                                    /* down (31:16) */
    hwaddr bsize;                   /* the blocks' size, then the grid's */
                                    /* origin: x in 7:0, y in 15:8 */
    hwaddr clear[2];                /* words whose bit 0 reads 0 once */
                                    /* written, 0: none */
    bool sta5;                      /* ages in 5-bit fields, 12 to 8 bytes */
    unsigned age_cap;               /* their ceiling, 0: 7 */
    bool coded;                     /* the reference is a coded stream: */
                                    /* the model keeps its own, see */
                                    /* ref_delay */
    unsigned ref_delay;             /* frames back it takes the frame to */
                                    /* differ from */
    hwaddr fun;                     /* bit 0 runs it; with wt, bits 12 */
                                    /* and 28 filter Y and UV */
    hwaddr age_max;                 /* bits 7:4 (T21: 3:0), 0: 7 */
    hwaddr blk_thr;                 /* bits 23:16, 0: 5 */
    hwaddr wt;                      /* the weights by age, 0: no filter */
    hwaddr wt_y, wt_uv;             /* Y's and UV's still blocks' limits */
                                    /* on them: most in 23:16, least in */
                                    /* 31:24 */
    hwaddr gate_y, gate_uv;         /* bits 11:3 */
    hwaddr spa;                     /* Y's spatial filter's weights for the */
                                    /* frame (a byte each), 0: none */
    hwaddr fiir;                    /* and for the reference, 16 bytes in */
                                    /* four words */
    hwaddr sharp;                   /* the sharpening's registers, from */
                                    /* the ISP's start, 0: none */
    bool rings;                     /* T21: the statistics in rings */
} IngenicIspMdns;

static const IngenicIspMdns ingenic_t23_isp_mdns = {
    .base = 0x7800, .id = 0x23021515, .refresh_reads = 0x20,
    .y = 0x20, .uv = 0x28, .sta = { 0x30, 0x40, 0x48, 0x50 },
    .grid = 0x74, .fun = 0x10, .age_max = 0x78, .blk_thr = 0x7c,
    .wt = 0x148, .wt_y = 0x19c, .wt_uv = 0x26c, .gate_y = 0x144,
    .gate_uv = 0x264,
};

static const IngenicIspMdns ingenic_t31_isp_mdns = {
    .base = 0x7800, .id = 0x20151120,
    .y = 0x20, .uv = 0x28, .sta = { 0x30, 0x40, 0x48, 0x50 },
    .grid = 0x74, .fun = 0x10, .age_max = 0x78, .blk_thr = 0x7c,
    .wt = 0x148, .wt_y = 0x19c, .wt_uv = 0x26c, .gate_y = 0x144,
    .gate_uv = 0x264, .spa = 0x1b8, .fiir = 0x1f0, .sharp = 0x7000,
};

static const IngenicIspMdns ingenic_t40_isp_mdns = {
    .base = 0xf000, .y = 0x40, .uv = 0x48, .sta = { 0x50 },
    .blocks = 0x10c, .bsize = 0x100, .clear = { 0x004, 0x008 },
    .coded = true, .ref_delay = 2,
};

static const IngenicIspMdns ingenic_t32_isp_mdns = {
    .base = 0xd000, .y = 0x40, .uv = 0x48, .sta = { 0x50 },
    .blocks = 0x10c, .bsize = 0x100, .clear = { 0x014, 0x018 },
    .sta5 = true, .age_cap = 15, .coded = true, .ref_delay = 1,
};

static const IngenicIspMdns ingenic_t21_isp_mdns = {
    .base = 0x1b00, .y = 0x10, .uv = 0x20, .sta = { 0x30, 0x40 },
    .age_max = 0x80, .rings = true,
};

static const IngenicIspMdns ingenic_t41_isp_mdns = {
    .base = 0xf000, .y = 0x40, .uv = 0x48, .sta = { 0x50 },
    .blocks = 0x10c, .bsize = 0x100,
};

/*
 * What an MSCA keeps up with: the scaled pixels a line of the frame the ISP
 * takes (a channel's scaled size over the front crop window's height, or the
 * frame's) that fit what the ISP clock does in a line time. Past that every
 * channel stops. Each row is one ISP clock the MPLL divides to: the highest
 * scaled size that kept streaming and the lowest that stalled, measured
 * with channel scaling (@scaler) and, where the part has a front crop, with
 * a window enlarged (@window). The limits are the midpoints of each pair,
 * fitted: the pairs bracket them by 2 to 5 percent. They are not linear in
 * the clock, so the limit is straight between the rows and scaled off the
 * first and last. The frame rate moved none of them, so the limit is a
 * pixel rate: these pixels a line at the sensor line rate @line_rate the
 * rows were measured at, which another sensor's line rate scales to its own.
 */
typedef struct IngenicIspLoad {
    uint32_t khz, scaler, window;
} IngenicIspLoad;

typedef struct IngenicIspLoads {
    const IngenicIspLoad *rows;
    unsigned n;
    uint32_t line_rate;
    uint32_t default_khz;           /* chosen: for a CPM that gives none */
                                    /* (a guest's tx-isp always sets one) */
    /* The core's input: the frames that pass at a width x line rate / clock */
    const IngenicT20IspKeep *keep;
    unsigned keeps;
} IngenicIspLoads;

/*
 * A T31X V3 (GC2053, 1080p, 1418 lines at 30 fps) with channel 1 at
 * 1280x720 streaming. AutoZoom is the scaler, a front crop the window. Channel
 * 1 off let AutoZoom one step further (1.65x at 200 MHz), well short of what
 * its pixels would free if the channels shared the limit, so each is held to
 * it alone.
 */
static const IngenicIspLoad ingenic_t31_isp_load_rows[] = {
    { 100000, 1960, 2290 },
    { 120000, 2941, 3127 },
    { 150000, 3968, 4054 },
    { 200000, 5148, 5763 },
    { 240000, 6308, 6650 },
    { 300000, 7300, 8132 },
};

static const IngenicIspLoads ingenic_t31_isp_loads = {
    .rows = ingenic_t31_isp_load_rows,
    .n = ARRAY_SIZE(ingenic_t31_isp_load_rows),
    .line_rate = 1418 * 30,
    .default_khz = 200000,          /* a V3's */
};

/*
 * A T21N (Wansview W6, ov2735b at 1080p, 1595 lines at 25 fps) with the
 * vendor tx-isp, channel 0 scaled up and cropped back to 1080p by the frame
 * source, the SDK having no AutoZoom or front crop. The MPLL is 900 MHz, the
 * divider 4 bits. No window: the SDK has no front crop. 75 MHz stalls an
 * unscaled 1080p, which the first row's slope also gives. The step from 100
 * to 112.5 MHz repeated in two runs. The clock tx-isp leaves when the
 * guest never set one is chosen: the open driver's isp_clk default.
 */
static const IngenicIspLoad ingenic_t21_isp_load_rows[] = {
    { 81818, 1988, 0 },
    { 90000, 2152, 0 },
    { 100000, 2297, 0 },
    { 112500, 2809, 0 },
    { 128571, 3214, 0 },
    { 150000, 3721, 0 },
    { 180000, 4467, 0 },
    { 225000, 5612, 0 },
};

static const IngenicIspLoads ingenic_t21_isp_loads = {
    .rows = ingenic_t21_isp_load_rows,
    .n = ARRAY_SIZE(ingenic_t21_isp_load_rows),
    .line_rate = 1595 * 25,
    .default_khz = 100000,
};

/*
 * A T23N (sc2336 at 1080p, 1440 lines at 25 fps) with thingino's open
 * tx-isp. The scaler column is channel 0's scaled size past the sensor's
 * (the frame source's scaler up with its crop back to 1080p; AutoZoom stalls
 * at its first enlargement at any clock, so it gives no limit), the window
 * column a front crop of channel 0 at 1080p, the window in 0xd090 and
 * 0xd094. The MPLL is 1200 MHz, the divider 4 bits; the AXI clock
 * (cgu_vpu, 200 to 600 MHz) moved neither.
 */
static const IngenicIspLoad ingenic_t23_isp_load_rows[] = {
    { 100000, 2503, 2352 },
    { 120000, 2961, 2452 },
    { 133333, 3287, 2680 },
    { 150000, 3808, 2992 },
    { 171429, 4368, 3116 },
    { 200000, 4967, 3494 },
    { 240000, 6280, 4272 },
    { 300000, 7489, 5122 },
};

static const IngenicIspLoads ingenic_t23_isp_loads = {
    .rows = ingenic_t23_isp_load_rows,
    .n = ARRAY_SIZE(ingenic_t23_isp_load_rows),
    .line_rate = 1440 * 25,
    .default_khz = 200000,          /* the T31's; the driver's is 153 MHz */
};

/*
 * A T40XP (Wyze V3 Pro, gc4653 at 2560x1440, 1920 lines at 25 fps) with
 * thingino's tx-isp: the scaler column is channel 0 scaled up by the frame
 * source and cropped back to 2560x1440, the 150 MHz row only known to
 * stream at 3098 (the largest tried), so it is the 200 MHz row's rate over
 * clock; the 171 and 240 MHz rows were not measured and run straight
 * between their neighbours. The window column is AutoZoom's (2945, 3458,
 * 4096, 4876 and 6165 px a line streamed at 150, 171.4, 200, 240 and 300
 * MHz, 3077, 3586, 4358, 5135 and 6423 stalled), each row the middle of the
 * two. The clock divides SCLK_A or MPLL (1200 MHz) by 8 bits.
 *
 * The core's input drops every other frame from a load of 0.92 (133 and 126
 * MHz at 2560 px) and all of them from 1.02 (120 MHz); it streamed at 0.853
 * (144 MHz). A T30X streamed at 0.885 and stalled at 0.983, so 0.90 and
 * 0.98 are chosen to fit both. The frame rate moved neither.
 */
static const IngenicIspLoad ingenic_t40_isp_load_rows[] = {
    { 150000, 3122, 3011 },
    { 171429, 3567, 3522 },
    { 200000, 4163, 4227 },
    { 240000, 4961, 5005 },
    { 300000, 6157, 6294 },
};

static const IngenicT20IspKeep ingenic_t40_isp_keep[] = {
    { 900, 1000 }, { 901, 500 }, { 980, 500 }, { 981, 0 },
};

static const IngenicIspLoads ingenic_t40_isp_loads = {
    .rows = ingenic_t40_isp_load_rows,
    .n = ARRAY_SIZE(ingenic_t40_isp_load_rows),
    .line_rate = 1920 * 25,
    .default_khz = 200000,          /* thingino's isp_clk */
    .keep = ingenic_t40_isp_keep,
    .keeps = ARRAY_SIZE(ingenic_t40_isp_keep),
};

/*
 * A T41NQ (gc4023 at 2560x1440, 1875 lines, which the board streams at 20
 * frames a second: 37500 lines a second) with thingino's tx-isp. Three
 * clocks: the core on ISPM (CPM 0x80), the MSCA on ISPS (0xa8, SCLK_A over a
 * divider of 8 bits), the AXI on ISPA (0xac, not varied). The core at 270 MHz
 * left the MSCA's limit alone, and ISPM at 120 with ISPS at 276 gave the same
 * window limit as 270.
 *
 * The core keeps every frame up to 100.0 MHz (a load of 0.960) and none at
 * 98.2 (0.978); no clock in between exists, so the edge is chosen at 0.969.
 * A T41LQ (gc5603 at 2880x1620, 1750 lines at 30 fps) kept all at 171.4 MHz
 * (0.882) and none at 150 (1.008), which the edge fits; there is no half
 * band like the T40's at either clock. A deeper overload (92 and 93 MHz,
 * 1.03) raised status bit 4 and a VIC reset instead of bit 5; that is not
 * modelled.
 *
 * The MSCA's rows, in scaled pixels a line, the middle of the last size that
 * streamed and the first that stalled. The scaler column (the frame source's
 * scaler up, cropped back to 2560x1440, sizes even) gave 2660 and 2765 at
 * 100.4 MHz, 3268 and 3386 at 122.7, 4193 and 4326 at 157.7: 27.0 a MHz at
 * each. 276 and 368 MHz streamed to 6554 and 7398, the most the board's
 * memory takes, so the column is that 27.0 straight through all the rows:
 * CHOSEN beyond 157.7 MHz. The window column (AutoZoom, whose window is
 * smaller than the frame) is lower and bends: 2560 and 2691 at 100.4, 2818
 * and 2945 at 122.7, 3077 and 3333 at 138, 3333 and 3458 at 157.7, 3586 and
 * 3848 at 184, 4228 and 4358 at 220.8, 4876 and 5135 at 276, 6144 and 6291
 * at 368. Odd sizes stall earlier than these, a width or height not even; the
 * model does not. The frame rate (15 and 25 fps) moved none of it.
 */
static const IngenicIspLoad ingenic_t41_isp_load_rows[] = {
    { 100364, 2710, 2625 },
    { 122667, 3312, 2881 },
    { 138000, 3726, 3205 },
    { 157714, 4258, 3395 },
    { 184000, 4968, 3717 },
    { 220800, 5962, 4293 },
    { 276000, 7452, 5005 },
    { 368000, 9936, 6217 },
};

static const IngenicT20IspKeep ingenic_t41_isp_keep[] = {
    { 969, 1000 }, { 970, 0 },
};

static const IngenicIspLoads ingenic_t41_isp_loads = {
    .rows = ingenic_t41_isp_load_rows,
    .n = ARRAY_SIZE(ingenic_t41_isp_load_rows),
    .line_rate = 1875 * 20,
    .default_khz = 270000,          /* ISPM as a T41NQ's tx-isp leaves it */
    .keep = ingenic_t41_isp_keep,
    .keeps = ARRAY_SIZE(ingenic_t41_isp_keep),
};

/*
 * A T32LQ (sc4336p at 2560x1440, 1500 lines at 30 fps) with thingino's
 * tx-isp. The core sits on ISPM (CPM 0x80, MPLL or VPLL or SCLK_A over a
 * divider), the MSCA on ISPS (0x90). The core kept every frame at 120 MHz
 * (a load of 0.96) and 118.8 (0.97), with a lone "irq-status unusual 0x20" a
 * few times a minute, and lost all at 112.5 MHz (1.02) and below, the VIC
 * then reporting status bit 9 ("intp error 0x200") every frame. The clocks
 * the dividers give left a gap from 0.97 to 1.02, so the edge is CHOSEN at
 * 0.997; there is no half band between. The frame rate was not varied.
 *
 * The MSCA's columns are not measured: the board holds 42 MB, a probe
 * fails to allocate the ISP's 512 KB block once raptor has run, and the
 * raptor daemon restarts that would scale channel 0 reboot the board. The
 * T33 and the T32 Pro follow this part, unmeasured.
 */
static const IngenicT20IspKeep ingenic_t32_isp_keep[] = {
    { 996, 1000 }, { 997, 0 },
};

static const IngenicIspLoads ingenic_t32_isp_loads = {
    .line_rate = 1500 * 30,
    .default_khz = 200000,          /* ISPM as a T32LQ's tx-isp leaves it */
    .keep = ingenic_t32_isp_keep,
    .keeps = ARRAY_SIZE(ingenic_t32_isp_keep),
};

struct IngenicIspLayout {
    const char *desc;
    hwaddr iosize;                  /* 0 for INGENIC_ISP_IOSIZE */
    unsigned pipes;
    IngenicIspPipe pipe[INGENIC_ISP_PIPES];
    bool int_mask;                  /* int_en masks: 1 off */
    uint32_t int_frame_start, int_frame_end;
    uint32_t int_ch0;               /* raised with channel 0's frame */
    hwaddr ch_busy;                 /* reads 0 */
    bool busy_held;                 /* a channel that put its frame out */
                                    /* stays busy while enabled */
    uint32_t ch_en_pulse;           /* channel enable bits that clear as */
                                    /* a frame reaches the MSCA */
    bool chans_at_start;            /* the MSCA's channels: those enabled */
                                    /* as a frame reaches it */
    bool gate_holds;                /* gated: see isp_clock() */
    uint32_t vic_ungate;            /* VIC status bits the clock's return */
                                    /* sets */
    unsigned vics;
    hwaddr vic, vic_stride;
    hwaddr vic_ctrl;                /* VIC_CTRL, or this */
    hwaddr vic_mode;                /* its input: VIC_MODE_MIPI via the CSI */
    hwaddr vic_mipi_fmt;            /* its MIPI data type, if not +0x14 */
    hwaddr vic_switch;              /* bit 0: the VIC flips the MIPI switch */
    uint32_t vic_switch_on;         /* or these bits, on a VIC of two inputs */
    hwaddr vic_sub;                 /* which of the two a frame came from */
    bool vic_turns;                 /* several sensors: see isp_core_turn() */
    hwaddr core_kick;               /* a write lets the core take a frame */
    uint32_t core_kick_bits;        /* with these bits; 0: bit 0 */
    hwaddr core_next;               /* bits 11:8: the input it takes next */
    bool vic_ctrl_level;            /* the VIC runs while its bit 0 is set */
    const IngenicIspVicInt *vic_int;
    uint32_t vic_frame_start;
    hwaddr vic_frames;              /* counts the frames it takes */
    hwaddr mdma;                    /* its raw capture, if any */
    bool mdma_fifo;                 /* it takes its buffers from queues */
    hwaddr vic_hold;                /* bit 6 holds its frames from the ISP */
    hwaddr raw_dma;                 /* the raw WDMA, the RDMA 0x1000 on */
    const IngenicIspResetVal *vic_reset;    /* each VIC's, from its base */
    unsigned vic_resets;
    const IngenicIspMask *vic_masks;
    unsigned vic_nmasks;
    hwaddr idle;                    /* bit 0 reads 1 */
    IngenicIspBusy busy[3];
    IngenicIspBusy set;             /* a register whose bits read set */
    hwaddr rcfg;                    /* the register lists' table, if any */
    hwaddr rcfg_trig;               /* lists 0-31 start; 32-63 at +4 */
    hwaddr shadow;                  /* the port of (value, offset) pairs */
    hwaddr shadow_open;             /* a write starts a run of pairs */
    hwaddr shadow_base;             /* where the pairs' offsets count from */
    IngenicIspShadowRun run;        /* the run's queue and its start, if any */
    IngenicIspTick tick;            /* its modules' frame done */
    IngenicIspTick dma;             /* its DMAs' frame done */
    IngenicIspTick lists[2];        /* lists 0-31's and 32-63's done */
    const IngenicIspStats *stats;
    const IngenicIspOsd *osd;
    const IngenicIspChanRegs *chan;
    hwaddr fcrop;                   /* the front crop window, x << 16 | y, */
                                    /* then w << 16 | h at +4 */
    hwaddr ispcdr;                  /* the ISP's clock divider; its MSCA */
                                    /* stalls beyond what it keeps up */
                                    /* with (isp_overload()) */
    hwaddr scdr;                    /* the MSCA's own clock divider, if */
                                    /* not the ISP's */
    hwaddr chwin;                   /* each channel's input window (the */
                                    /* AutoZoom), x << 16 | y, then */
                                    /* w << 16 | h at +4, 8 apart */
    hwaddr chratio;                 /* where the size is not given: the */
                                    /* horizontal ratios, the vertical */
                                    /* ones 12 below, 4 apart */
    const IngenicIspLoads *loads;
    uint32_t stall_sta;             /* the status bits a stall raises */
    hwaddr err_sta, err_clr;        /* an error bank instead, which keeps */
                                    /* the line up while any bit is set */
    uint32_t err_overflow;          /* its bit for an ISP overflow */
    uint32_t stall_vic;             /* the VIC status bit a dropped frame */
                                    /* raises, if the VIC reports it */
    uint32_t unscaled_bit;          /* channel enable bit: channel 0 */
                                    /* unscaled */
    hwaddr drop;                    /* a channel's frame drop: bit i of */
                                    /* the mask at +4 lets frame i of a */
                                    /* window (length - 1 here) out */
    uint32_t drop_mark;             /* the mask out of reset */
    bool drop_rephase;              /* a mask write restarts the window */
    hwaddr ch0_out[2];              /* channel 0's line << 16 | pixel, */
                                    /* as it puts a frame out */
    bool ch0_out_holds;             /* after it, its height; else 0 */
    bool push_latch;                /* a buffer pushed after a frame */
                                    /* reached the MSCA waits for the next; */
                                    /* a push reads back addr | 1 until */
                                    /* its frame lands, done counts */
    hwaddr wb_r, wb_b;              /* the guest's white-balance gains, */
    hwaddr wb_pipe;                 /* a pipeline's apart */
    ImageSensorRawPart raw_part;    /* whose tuning files give the sensor's */
                                    /* raw response; none: the constants */
    const IngenicIspMdns *mdns;     /* its 3D noise reducer, if modelled */
    bool done_uv_head;              /* the UV done shows the head entry, */
                                    /* or what the Y read just popped */
    bool ivdc_no_done;              /* an IVDC frame leaves no done entry */
    bool done_unread;               /* the oldest done entry gives way */
    bool clr_bit0;                  /* a FIFO clear needs bit 0 */
    bool clr_edge;                  /* and acts as bit 0 rises */
    bool drop_done;                 /* a frame dropped still ends: done */
    bool list_push_latest;          /* a list's push replaces the queued */
    hwaddr vic_out;                 /* each VIC input's output gate */
    hwaddr hldc;                    /* its lens correction, if any */
    hwaddr hldc_sec;                /* T40: the second pipeline's */
    uint32_t hldc_size;             /* the registers it has, 0: 0x48 */
    bool hldc_ctl_latch;            /* T21: 0x00 bits 0 and 4 latch */
    bool hldc_wide;                 /* T23, T31: 0x44 reads both bits */
    hwaddr ldc;                     /* T33, T32 Pro: the table-driven LDC */
    IngenicIspBusy hldc_bypass;     /* the bit that bypasses it */
    IngenicIspBusy hldc_sec_bypass; /* and the second's */
    const IngenicIspResetVal *reset;    /* what reads other than 0 */
    unsigned resets;
    const IngenicIspMask *masks;    /* words not all of whose bits stick, */
    unsigned nmasks;                /* in order */
    const IngenicIspLatch *latched; /* words that wait for a latch, */
    unsigned nlatched;              /* in order */
    hwaddr latch_now;               /* bit 0 latches the per-frame ones */
    int64_t vic_first;              /* the first frame, if later than a */
                                    /* frame time after the VIC starts */
    hwaddr shadow_kick;             /* writes the port's RAM of pairs out */
    hwaddr shadow_state;            /* reads whether the port is open */
    hwaddr shadow_last;             /* reads the last value it wrote out */
    unsigned shd_ram;               /* its RAM's pairs, if not the T41's */
    bool shd_kick_idle;             /* a kick between frames writes at once */
    bool shd_pair_clears_last;      /* a pair into the RAM clears its last */
    hwaddr core_reset;              /* bit 0 resets all but the VICs, */
                                    /* bit 2 restarts the core at a latch */
    hwaddr idle_req;                /* bit 0 stops the core */
    bool par_unlocked;              /* the passes run without the BQL */
};

static const IngenicIspLayout ingenic_t31_isp_layout = {
    .desc = "Ingenic T31 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x0b0, .int_sta = 0x0b4, .int_clr = 0x0b8,
          .ch_en = 0x9804, .ch = 0x9900, .busy = 0x9808 },
    },
    .int_frame_start = BIT(12),
    .int_frame_end = BIT(13),
    .stats = &ingenic_t31_isp_stats,
    .busy_held = true,
    .ch_en_pulse = 0xf0000,
    .chans_at_start = true,
    .gate_holds = true,
    .vic_ungate = BIT(4) | BIT(9),
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
    .vic_mode = 0x00c,
    .vic_int = &ingenic_isp_vic_int,
    .mdma = 0x300,
    .osd = &ingenic_t31_isp_osd,
    .chan = &ingenic_isp_chan_regs,
    .fcrop = 0x9860,
    .ispcdr = 0x80,
    .loads = &ingenic_t31_isp_loads,
    .stall_sta = BIT(10) | BIT(9),
    .unscaled_bit = BIT(8),
    .drop = 0x30,
    .drop_mark = 1,
    .drop_rephase = true,
    .ch0_out = { 0x9888 },
    .ch0_out_holds = true,
    .push_latch = true,
    .wb_r = 0x1804,
    .wb_b = 0x1808,
    .raw_part = IMAGE_SENSOR_PART_T31,
    .mdns = &ingenic_t31_isp_mdns,
    .hldc = 0x9000,
    .hldc_wide = true,
    .hldc_bypass = { 0x0c, BIT(18) },
    .par_unlocked = true,
};

static const IngenicIspLayout ingenic_t21_isp_layout = {
    .desc = "Ingenic T21 ISP and VIC",
    .mdns = &ingenic_t21_isp_mdns,
    .pipes = 1,
    .pipe = {
        { .int_en = 0x080, .int_sta = 0x084, .int_clr = 0x088,
          .ch_en = 0x2304, .ch = 0x2400 },
    },
    .ch_busy = 0x2308,
    .gate_holds = true,
    .vic_ungate = BIT(3),
    .stats = &ingenic_t21_isp_stats,
    .vics = 1,
    .vic = 0x10000,
    .vic_stride = 0x10000,
    .vic_int = &(const IngenicIspVicInt) {
        .sta = 0x100, .msk = 0x104, .clr = 0x108,
    },
    .chan = &ingenic_t21_isp_chan_regs,
    .wb_r = 0x604,
    .wb_b = 0x608,
    .raw_part = IMAGE_SENSOR_PART_T21,
    .ispcdr = 0x80,
    .loads = &ingenic_t21_isp_loads,
    .stall_sta = BIT(9),            /* tx-isp: "irq-status 0x200" */
    .hldc = 0x2200,
    .hldc_size = 0x2c,
    .hldc_ctl_latch = true,
    .hldc_bypass = { 0x0c, BIT(13) },
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
    .drop = 0x30,
    .drop_mark = 1,
};

static const IngenicIspLayout ingenic_t23_isp_layout = {
    .desc = "Ingenic T23 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x0b0, .int_sta = 0x0b4, .int_clr = 0x0b8,
          .ch_en = 0xd040, .ch = 0xd100, .busy = 0xd054 },
    },
    .int_frame_start = BIT(12),
    .int_frame_end = BIT(13) | BIT(27),    /* 27: AE histogram done */
    /* "all done": the dual-sensor firmware moves on to the next sensor */
    .tick = { 0x0c0, 0x0c8, 0x0c4, BIT(15) },
    .idle = 0x028,                  /* its reset waits for it after 0x24 */
    .stats = &ingenic_isp_stats,
    .chans_at_start = true,
    .gate_holds = true,
    .vic_ungate = BIT(4) | BIT(9),
    .vics = 1,
    .vic = 0xe0000,
    .vic_stride = 0x10000,
    .vic_mode = 0x00c,
    .vic_switch = 0x200,
    .vic_int = &ingenic_isp_vic_int,
    .mdma = 0x300,
    .wb_r = 0x1804,
    .wb_b = 0x1808,
    .raw_part = IMAGE_SENSOR_PART_T23,
    .mdns = &ingenic_t23_isp_mdns,
    .hldc = 0x9000,
    .hldc_wide = true,
    .hldc_bypass = { 0x0c, BIT(18) },
    .shadow = 0xe004,
    .shadow_open = 0xe000,
    .shadow_base = 0xd000,
    .run = { .kick = 0xe100, .sta = 0xe00c, .len_shift = 4, .len_mask = 0xfff,
             .done_on_close = true, .ram = 512,
             .start_shift = 16, .start_mask = 0x1ff },
    .osd = &ingenic_t23_isp_osd,
    .chan = &ingenic_t23_isp_chan_regs,
    .fcrop = 0xd090,
    .ispcdr = 0x80,
    .loads = &ingenic_t23_isp_loads,
    /* chosen as the T31's: the ISR clears them before a read sees them */
    .stall_sta = BIT(10) | BIT(9),
    .drop = 0x20,
    .drop_mark = 1,
    .done_uv_head = true,
    .clr_bit0 = true,               /* the dual firmware writes 0 to release */
};

/*
 * The T32's, T33's and T32 Pro's two VICs, each of which can take two
 * sensors behind a switch of its own; their driver's
 */
#define T32_VICS \
    .vics = 2, \
    .vic = 0x80000, \
    .vic_stride = 0x1000, \
    .vic_ctrl = 0x010, \
    .vic_ctrl_level = true, \
    .vic_mode = 0x01c, \
    .vic_mipi_fmt = 0x024, \
    .mdma = 0x300, \
    .vic_switch = 0x200, \
    .vic_switch_on = BIT(31) | BIT(16), \
    .vic_sub = 0x20c, \
    .vic_turns = true, \
    .vic_int = &ingenic_isp_vic_int, \
    .vic_frame_start = BIT(4)

/*
 * Each module's frame done (bit 1: the MSCA's), latched only where enabled:
 * a T32LQ raises 0xfffff9b7 each frame. Bit 0 as the frame starts, as a
 * T41's does; the T33's and T32 Pro's drivers also take 6 (DPC), 10 (TMO).
 */
#define T32_MODULES_DONE(more) \
    { 0x300a0, 0x300a8, 0x300a4, 0xfffff9b6 | (more), BIT(0), true }

/*
 * The T32 family's white-balance gains, R then B (0x04000000 | gain, a T32
 * Pro's and T33's B without that bit). A T32's AWB stores them, with a copy
 * at 0x2a040 that does not scale the picture; a T33's and T32 Pro's tx-isp
 * puts them in register list 7, which the core loads
 */
#define T32_WB_R 0x6040
#define T32_WB_B 0x6044

static const IngenicIspLayout ingenic_t32_isp_layout = {
    .desc = "Ingenic T32 ISP and VIC",
    .pipes = 1,
    .pipe = {
        { .int_en = 0x30020, .int_sta = 0x30028, .int_clr = 0x30024,
          .done_en = 0x30030, .done_sta = 0x30038, .done_clr = 0x30034,
          .ch_en = 0xf0040, .ch = 0xf0100, .busy = 0xf0054 },
    },
    .int_frame_end = BIT(0),
    T32_VICS,
    .core_kick = 0x0002c,
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { 0x22014, 0x30000 } },
    .rcfg = T33_RCFG_TABLE,
    .lists = { {}, { 0x300d0, 0x300d8, 0x300d4 } },
    .rcfg_trig = T33_RCFG_TRIG,
    .shadow = 0xf8004,
    .shadow_open = 0xf8000,
    .shadow_kick = 0xf8100,
    .shadow_state = 0xf800c,
    .shadow_last = 0xf8008,
    .shd_ram = 256,
    .shd_kick_idle = true,
    .shd_pair_clears_last = true,
    .tick = T32_MODULES_DONE(0),
    .stats = &ingenic_t32_isp_stats,
    .osd = &ingenic_t32_isp_osd,
    .mdns = &ingenic_t32_isp_mdns,
    .reset = ingenic_t32_isp_reset,
    .resets = ARRAY_SIZE(ingenic_t32_isp_reset),
    .chan = &ingenic_t23_isp_chan_regs,
    .drop = 0x20,
    .drop_mark = UINT32_MAX,
    .clr_bit0 = true,
    .wb_r = T32_WB_R,
    .wb_b = T32_WB_B,
    .raw_part = IMAGE_SENSOR_PART_T32,
    .ispcdr = 0x80,
    .scdr = 0x90,
    .loads = &ingenic_t32_isp_loads,
    .stall_vic = BIT(9),
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
    T32_VICS,
    /* the T33's driver runs its top list (33) to start each frame */
    .core_kick = T33_RCFG_TRIG + 4,
    .core_kick_bits = BIT(1),
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { T33_RCFG_TRIG, ~0u },
              { T33_RCFG_TRIG + 4, ~0u } },
    .set = { 0x00000, 1 },
    .rcfg = T33_RCFG_TABLE,
    .lists = { { 0x300c0, 0x300c8, 0x300c4 },
               { 0x300d0, 0x300d8, 0x300d4 } },
    .rcfg_trig = T33_RCFG_TRIG,
    .tick = T32_MODULES_DONE(BIT(6) | BIT(10)),
    .stats = &ingenic_t33_isp_stats,
    .osd = &ingenic_t33_isp_osd,
    .chan = &ingenic_t23_isp_chan_regs,
    .drop = 0x20,
    .drop_mark = UINT32_MAX,
    .clr_bit0 = true,
    .ldc = 0x27000,
    .wb_r = T32_WB_R,
    .wb_b = T32_WB_B,
    .raw_part = IMAGE_SENSOR_PART_T33,
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
    /* 8: LCE statistics, which its multi-sensor ISR waits for too */
    .int_frame_end = BIT(0) | BIT(8),
    .ch_busy = 0xf0054,
    T32_VICS,
    /* its driver reloads every list each frame, the top one (34) last */
    .core_kick = T32PRO_RCFG_TRIG + 4,
    .core_kick_bits = BIT(2),
    .core_next = 0x01304,
    .idle = 0x204,
    .busy = { { 0x00024, 0xf }, { T32PRO_RCFG_TRIG, ~0u },
              { T32PRO_RCFG_TRIG + 4, ~0u } },
    .set = { 0x00000, 1 },
    .rcfg = T32PRO_RCFG_TABLE,
    .lists = { { 0x300c0, 0x300c8, 0x300c4 },
               { 0x300d0, 0x300d8, 0x300d4 } },
    .rcfg_trig = T32PRO_RCFG_TRIG,
    .tick = T32_MODULES_DONE(BIT(6) | BIT(10)),
    .stats = &ingenic_t33_isp_stats,
    .osd = &ingenic_t32pro_isp_osd,
    .chan = &ingenic_t23_isp_chan_regs,
    .drop = 0x20,
    .drop_mark = UINT32_MAX,
    .clr_bit0 = true,
    .done_unread = true,
    .ldc = 0x1f000,
    .clr_edge = true,
    .drop_done = true,
    .list_push_latest = true,
    .vic_out = 0x130,
    .mdma_fifo = true,
    .wb_r = T32_WB_R,
    .wb_b = T32_WB_B,
    .raw_part = IMAGE_SENSOR_PART_T32PRO,
};

static const IngenicIspLayout ingenic_t40_isp_layout = {
    .desc = "Ingenic T40 ISP and VICs",
    .pipes = 2,
    .pipe = {
        { .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
          .ch_en = 0x16008, .ch = 0x16100, .busy = 0x160e0 },
        { .int_en = 0x40030, .int_sta = 0x40038, .int_clr = 0x40034,
          .ch_en = 0x17008, .ch = 0x17100, .busy = 0x170e0 },
    },
    /*
     * A T40N raises bit 13 only while channel 0 takes the frame; its driver
     * counts each pipeline's frames on it
     */
    .int_ch0 = BIT(13),
    .chans_at_start = true,
    .gate_holds = true,
    .vic_ungate = BIT(4) | BIT(9),
    .vics = 3,
    .vic = 0x80000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .vic_frames = 0x3a4,
    .mdma = 0x300,
    .vic_reset = ingenic_t40_vic_reset,
    .vic_resets = ARRAY_SIZE(ingenic_t40_vic_reset),
    .vic_masks = ingenic_t40_vic_masks,
    .vic_nmasks = ARRAY_SIZE(ingenic_t40_vic_masks),
    .idle = 0x204,
    .shadow = 0x50324,
    .shadow_open = 0x50320,
    .run = { .kick = 0x50420, .sta = 0x5032c, .len_shift = 24, .len_mask = 0x7f,
             .busy = BIT(8), .ram = 128,
             .start_shift = 16, .start_mask = 0x7f, .last = 0x50328 },
    /*
     * Its driver enables 25, 27 and 29, which a T40N raises as each frame
     * ends, with channel 0 off too; on 25 with channel 0 off it does the
     * per-frame work. It reads this word for pipeline 1's frames as well.
     */
    .tick = { 0x400a0, 0x400a8, 0x400a4, BIT(25) | BIT(27) | BIT(29), 0,
              true },
    .stats = &ingenic_t40_isp_stats,
    .osd = &ingenic_t40_isp_osd,
    .mdns = &ingenic_t40_isp_mdns,
    .reset = ingenic_t40_isp_reset,
    .resets = ARRAY_SIZE(ingenic_t40_isp_reset),
    .chan = &ingenic_isp_chan_regs,
    .drop = 0x30,
    .drop_mark = UINT32_MAX,
    .ch0_out = { 0x1675c, 0x167f4 },
    .wb_r = 0x4004,
    .wb_b = 0x4008,
    .wb_pipe = 0x1000,
    .raw_part = IMAGE_SENSOR_PART_T40,
    .hldc = 0x15000,
    .hldc_bypass = { 0x40, BIT(20) },
    .hldc_sec = 0x15080,
    .hldc_sec_bypass = { 0x6c, BIT(20) },
    /*
     * The ISR reads the error bank at 0x40068 and acknowledges at 0x40064;
     * bit 5 is "isp overflow". Its enable is not known: the line is held
     * while the bank is non-zero.
     */
    .ispcdr = 0x80,
    .chwin = 0x160a0,
    .loads = &ingenic_t40_isp_loads,
    .err_sta = 0x40068,
    .err_clr = 0x40064,
    .err_overflow = BIT(5),
};

static const IngenicIspLayout ingenic_t41_isp_layout = {
    .desc = "Ingenic T41 ISP and VIC",
    .mdns = &ingenic_t41_isp_mdns,
    .pipes = 1,
    .pipe = {
        { .int_en = 0x40020, .int_sta = 0x40028, .int_clr = 0x40024,
          .ch_en = 0xf0008, .ch = 0xf0100 },
    },
    .int_frame_start = BIT(20),     /* a T41NQ never raises 13 */
    .vics = 1,
    .vic = 0x80000,
    .vic_stride = 0x10000,
    .vic_int = &ingenic_isp_vic_int,
    .vic_frame_start = BIT(4),
    .mdma = 0x300,
    .vic_hold = 0x1a8,
    .raw_dma = 0x20000,
    .idle = 0x204,
    .shadow = 0xf8004,
    .shadow_open = 0xf8000,
    .stats = &ingenic_t41_isp_stats,
    .osd = &ingenic_t41_isp_osd,
    .chan = &ingenic_isp_chan_regs,
    .drop = 0x30,
    .drop_mark = UINT32_MAX,
    .wb_r = 0x4004,
    .wb_b = 0x4008,
    .raw_part = IMAGE_SENSOR_PART_T41,
    .ivdc_no_done = true,
    .hldc = 0x15000,
    .hldc_bypass = { 0x00040, BIT(20) },
    .reset = ingenic_t41_isp_reset,
    .resets = ARRAY_SIZE(ingenic_t41_isp_reset),
    .masks = ingenic_t41_isp_masks,
    .nmasks = ARRAY_SIZE(ingenic_t41_isp_masks),
    .latched = ingenic_t41_isp_latched,
    .nlatched = ARRAY_SIZE(ingenic_t41_isp_latched),
    .latch_now = 0x01000,
    /*
     * tx-isp arms the MSCA's trigger as it resets the ISP and writes the
     * channel's size 43 ms after it starts the VIC, with no trigger of its
     * own: the first frame has to latch it. A T41NQ with a gc4023 latches
     * 135.0 ms after the start in seven runs, asked for 10 fps or not.
     */
    .vic_first = 135 * SCALE_MS,
    /*
     * A T41NQ's each frame: module done bit 0 as it starts, 0x7efffffe as
     * it ends; DMA done 0xd, bits 4 and 5 (raw WDMA, RDMA) without a raw
     * capture never
     */
    .tick = { 0x400a0, 0x400a8, 0x400a4, 0x7efffffe, BIT(0), true },
    .dma = { 0x400b0, 0x400b8, 0x400b4, 0xd, 0, true },
    .shadow_kick = 0xf8100,
    .shadow_state = 0xf800c,
    .shadow_last = 0xf8008,
    .core_reset = 0x00024,
    .idle_req = 0x00200,
    .ispcdr = 0x80,
    .scdr = 0xa8,
    .chwin = 0xf00a0,
    .chratio = 0xf0728,
    .loads = &ingenic_t41_isp_loads,
    .err_sta = 0x40068,
    .err_clr = 0x40064,
    .err_overflow = BIT(5),
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

#define ISP_NAME_LEN        24

/* The guest PC behind an access, as of the start of its translation block */
static uint32_t isp_pc(void)
{
    return current_cpu ? CPU_GET_CLASS(current_cpu)->get_pc(current_cpu) : 0;
}

/* In the @n blocks @step apart from @base: "what" block number + offset */
static bool isp_name_in(char *buf, hwaddr off, hwaddr base, hwaddr step,
                        unsigned n, const char *what)
{
    if (!step || off < base || off >= base + n * step) {
        return false;
    }
    if (n > 1) {
        snprintf(buf, ISP_NAME_LEN, "%s%u+0x%x", what,
                 (unsigned)((off - base) / step),
                 (unsigned)((off - base) % step));
    } else {
        snprintf(buf, ISP_NAME_LEN, "%s+0x%x", what, (unsigned)(off - base));
    }
    return true;
}

/* The OSD's: "osd.pic0.3+0x4" is channel 0's picture 3, its second word */
static bool isp_osd_name(IngenicIspState *s, hwaddr off, char *buf)
{
    const IngenicIspOsd *o = isp_layout(s)->osd;
    const struct {
        hwaddr off;
        const char *name;
    } regs[] = {
        { o->mask_en, "osd.mask_en" }, { o->pic_en, "osd.pic_en" },
        { o->fmt, "osd.fmt" }, { o->order, "osd.order" },
    };
    char what[16];

    for (unsigned i = 0; i < ARRAY_SIZE(regs); i++) {
        if (regs[i].off && off == regs[i].off) {
            snprintf(buf, ISP_NAME_LEN, "%s", regs[i].name);
            return true;
        }
    }
    if ((o->csc && isp_name_in(buf, off, o->csc, 0x10, 1, "osd.csc")) ||
        (o->palette && isp_name_in(buf, off, o->palette, 0x10,
                                   DIV_ROUND_UP(o->pics, 4), "osd.palette"))) {
        return true;
    }
    for (unsigned c = 0; c < INGENIC_ISP_CHANS; c++) {
        snprintf(what, sizeof(what), "osd.mask%u.", c);
        if (o->mask && isp_name_in(buf, off, o->mask + c * o->mask_chan,
                                   o->mask_step, OSD_MASKS, what)) {
            return true;
        }
        snprintf(what, sizeof(what), "osd.pic%u.", c);
        if (c < o->pic_chans &&
            isp_name_in(buf, off, o->pic + c * o->pic_chan, 0x10, o->pics,
                        what)) {
            return true;
        }
        snprintf(what, sizeof(what), "osd.draw%u.", c);
        if (c < o->draw_chans && o->draw &&
            isp_name_in(buf, off, o->draw + c * o->draw_chan, 0x10, o->draws,
                        what)) {
            return true;
        }
    }
    return false;
}

/*
 * A register's name as its layout knows it, for the trace: "ch1.y_push",
 * "p1.int_sta" (pipeline 1's), "vic0.ctrl", "ae+0x2c"; "" for the many it
 * does not name
 */
static const char *isp_reg_name(IngenicIspState *s, hwaddr off, char *buf)
{
    static const struct {
        size_t at;
        const char *name;
    } pipe_regs[] = {
        { offsetof(IngenicIspPipe, int_en), "int_en" },
        { offsetof(IngenicIspPipe, int_sta), "int_sta" },
        { offsetof(IngenicIspPipe, int_clr), "int_clr" },
        { offsetof(IngenicIspPipe, done_en), "done_en" },
        { offsetof(IngenicIspPipe, done_sta), "done_sta" },
        { offsetof(IngenicIspPipe, done_clr), "done_clr" },
        { offsetof(IngenicIspPipe, ch_en), "ch_en" },
        { offsetof(IngenicIspPipe, busy), "msca_busy" },
    }, chan_regs[] = {
        { offsetof(IngenicIspChanRegs, size), "size" },
        { offsetof(IngenicIspChanRegs, crop_pos), "crop_pos" },
        { offsetof(IngenicIspChanRegs, crop_size), "crop_size" },
        { offsetof(IngenicIspChanRegs, y_push), "y_push" },
        { offsetof(IngenicIspChanRegs, done_y), "done_y" },
        { offsetof(IngenicIspChanRegs, done_sta), "done_sta" },
        { offsetof(IngenicIspChanRegs, y_stride), "y_stride" },
        { offsetof(IngenicIspChanRegs, uv_push), "uv_push" },
        { offsetof(IngenicIspChanRegs, done_uv), "done_uv" },
        { offsetof(IngenicIspChanRegs, done_frame), "done_frame" },
        { offsetof(IngenicIspChanRegs, uv_stride), "uv_stride" },
        { offsetof(IngenicIspChanRegs, clr_y), "clr_y" },
        { offsetof(IngenicIspChanRegs, clr_uv), "clr_uv" },
        { offsetof(IngenicIspChanRegs, clr_done_y), "clr_done_y" },
        { offsetof(IngenicIspChanRegs, clr_done_uv), "clr_done_uv" },
        { offsetof(IngenicIspChanRegs, push_sta), "push_sta" },
    };
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspStats *st = l->stats;
    const struct {
        hwaddr off;
        const char *name;
    } regs[] = {
        { l->ch_busy, "ch_busy" }, { l->idle, "idle" },
        { l->shadow, "shadow" }, { l->shadow_open, "shadow_open" },
        { l->run.kick, "shadow_kick" }, { l->run.sta, "shadow_sta" },
        { l->rcfg, "rcfg_table" }, { l->rcfg_trig, "rcfg_trig" },
        { l->rcfg_trig ? l->rcfg_trig + 4 : 0, "rcfg_trig_hi" },
        { l->tick.en, "tick_en" }, { l->tick.sta, "tick_sta" },
        { l->tick.clr, "tick_clr" }, { l->dma.en, "dma_en" },
        { l->dma.sta, "dma_sta" }, { l->dma.clr, "dma_clr" },
        { st ? st->size : 0, "size" },
    };
    const char *pre;
    int chan = -1, vic;
    hwaddr reg;

    for (unsigned i = 0; i < ARRAY_SIZE(regs); i++) {
        if (regs[i].off && off == regs[i].off) {
            return regs[i].name;
        }
    }
    if (l->set.bits && off == l->set.off) {
        return "ready";
    }
    for (unsigned i = 0; i < ARRAY_SIZE(l->busy); i++) {
        if (l->busy[i].bits && off == l->busy[i].off) {
            return "busy";
        }
    }
    for (unsigned p = 0; p < l->pipes; p++) {
        const IngenicIspPipe *pipe = &l->pipe[p];

        pre = l->pipes > 1 ? (p ? "p1." : "p0.") : "";
        for (unsigned i = 0; i < ARRAY_SIZE(pipe_regs); i++) {
            hwaddr at = *(const hwaddr *)((const char *)pipe +
                                          pipe_regs[i].at);

            if (at && off == at) {
                snprintf(buf, ISP_NAME_LEN, "%s%s", pre, pipe_regs[i].name);
                return buf;
            }
        }
        if (off < pipe->ch || off >= pipe->ch + INGENIC_ISP_CHANS * 0x100) {
            continue;
        }
        for (unsigned i = 0; i < ARRAY_SIZE(chan_regs); i++) {
            uint32_t at = *(const uint32_t *)((const char *)l->chan +
                                              chan_regs[i].at);

            if (at != CH_NONE && (off - pipe->ch) % 0x100 == at) {
                snprintf(buf, ISP_NAME_LEN, "%sch%u.%s", pre,
                         (unsigned)(off - pipe->ch) / 0x100,
                         chan_regs[i].name);
                return buf;
            }
        }
        chan = p;
    }
    vic = isp_vic(s, off, &reg);
    if (vic >= 0) {
        const IngenicIspVicInt *vi = l->vic_int;
        const char *what = NULL;

        if (reg == l->vic_ctrl) {
            what = "ctrl";
        } else if (l->vic_mode && reg == l->vic_mode) {
            what = "mode";
        } else if (l->vic_switch && reg == l->vic_switch) {
            what = "switch";
        }
        for (unsigned i = 0; !what && i <= vi->pair; i++) {
            what = reg == vi->sta + 4 * i ? (i ? "int_sta1" : "int_sta") :
                   reg == vi->msk + 4 * i ? (i ? "int_msk1" : "int_msk") :
                   reg == vi->clr + 4 * i ? (i ? "int_clr1" : "int_clr") :
                   NULL;
        }
        if (what) {
            snprintf(buf, ISP_NAME_LEN, "vic%d.%s", vic, what);
        } else {
            snprintf(buf, ISP_NAME_LEN, "vic%d+0x%x", vic, (unsigned)reg);
        }
        return buf;
    }
    for (unsigned p = 0; st && p < (st->sec ? l->pipes : 1); p++) {
        hwaddr up = p * st->sec;

        pre = p ? "p1.ae" : "ae";
        if (st->ae_en && off == st->ae_en + up) {
            snprintf(buf, ISP_NAME_LEN, "%s_en", pre);
            return buf;
        }
        if (isp_name_in(buf, off, st->ae + up, 0x60, 1, pre)) {
            return buf;
        }
        pre = p ? "p1.awb" : "awb";
        if (st->awb_en && off == st->awb_en + up) {
            snprintf(buf, ISP_NAME_LEN, "%s_en", pre);
            return buf;
        }
        if (st->awb_zone && off == st->awb_zone + up) {
            snprintf(buf, ISP_NAME_LEN, "%s_zone", pre);
            return buf;
        }
        if (st->awb && isp_name_in(buf, off, st->awb + up, 0x60, 1, pre)) {
            return buf;
        }
    }
    if (l->osd && isp_osd_name(s, off, buf)) {
        return buf;
    }
    if (chan >= 0) {
        hwaddr ch = l->pipe[chan].ch;

        snprintf(buf, ISP_NAME_LEN, "%sch%u+0x%02x",
                 l->pipes > 1 ? (chan ? "p1." : "p0.") : "",
                 (unsigned)(off - ch) / 0x100, (unsigned)(off - ch) % 0x100);
        return buf;
    }
    return "";
}

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
    if (l->err_sta) {
        isp |= R(l->err_sta);
    }
    if (l->dma.sta) {
        isp |= R(l->dma.sta) & R(l->dma.en);
    }
    for (unsigned k = 0; k < ARRAY_SIZE(l->lists); k++) {
        if (l->lists[k].sta) {
            isp |= R(l->lists[k].sta) & R(l->lists[k].en);
        }
    }
    if (isp != s->isp_level) {
        s->isp_level = isp;
        trace_ingenic_isp_irq(isp);
    }
    qemu_set_irq(s->isp_irq, isp);
    for (unsigned n = 0; n < l->vics; n++) {
        const IngenicIspVicInt *vi = l->vic_int;
        uint32_t pending = VIC(n, vi->sta) & ~VIC(n, vi->msk);

        if (vi->pair) {
            pending |= VIC(n, vi->sta + 4) & ~VIC(n, vi->msk + 4);
        }
        if (!pending != !(s->vic_level & BIT(n))) {
            s->vic_level ^= BIT(n);
            trace_ingenic_isp_vic_irq(n, !!pending);
        }
        qemu_set_irq(s->vic_irq[n], !!pending);
    }
}

/* Pipeline and channel numbers of @ch */
static void isp_chan_index(IngenicIspState *s, IngenicIspChan *ch,
                           unsigned *p, unsigned *n)
{
    unsigned i = ch - &s->chan[0][0];

    *p = i / INGENIC_ISP_CHANS;
    *n = i % INGENIC_ISP_CHANS;
}

/* Status bits the hardware raises in register @off */
static void isp_raise(IngenicIspState *s, hwaddr off, uint32_t bits)
{
    char name[ISP_NAME_LEN];

    if (!bits) {
        return;
    }
    R(off) |= bits;
    if (trace_event_get_state_backends(TRACE_INGENIC_ISP_RAISE)) {
        trace_ingenic_isp_raise(off, bits, isp_reg_name(s, off, name));
    }
}

static void isp_tick(IngenicIspState *s, const IngenicIspTick *t,
                     uint32_t bits)
{
    if (t->sta) {
        isp_raise(s, t->sta, t->gated ? bits & R(t->en) : bits);
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
    if (R(o->mask_en) & BIT(c + o->mask_en_shift)) {
        for (unsigned b = 0; b < OSD_MASKS; b++) {
            hwaddr m = o->mask + c * o->mask_chan + b * o->mask_step;
            uint32_t yuv = R(m + 8);

            if (o->mask_vu) {
                yuv = (yuv & 0xff0000) | (yuv & 0xff) << 8 |
                      extract32(yuv, 8, 8);
            }
            isp_osd_mask(buf, bw, bh, ox, oy, ow, oh, R(m), R(m + 4), yuv);
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

/* The frame the ISP takes, width << 16 | height in its statistics' size */
static void isp_frame_size(IngenicIspState *s, uint32_t *w, uint32_t *h)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    uint32_t size = st ? R(st->size) : 0;

    *w = (size >> 16) & ~1;
    *h = (size & 0xffff) & ~1;
}

static uint32_t isp_hldc_size(const IngenicIspLayout *l)
{
    return l->hldc_size ?: INGENIC_HLDC_SIZE;
}

static void isp_hldc_reset(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);

    for (unsigned p = 0; p < 2; p++) {
        ingenic_hldc_reset(&s->hldc[p], l->hldc_ctl_latch, l->hldc_wide);
    }
}

/* The lens correction that works on pipeline @pipe's frames, if any */
static IngenicIspHldc *isp_hldc(IngenicIspState *s, unsigned pipe)
{
    const IngenicIspLayout *l = isp_layout(s);

    return (pipe ? l->hldc_sec : l->hldc) ? &s->hldc[pipe] : NULL;
}

/*
 * The frame the correction takes: the ISP's; the second pipeline's is taken
 * to be its channel 0's size, no register of its own being known
 */
static void isp_hldc_frame(IngenicIspState *s, unsigned pipe, uint32_t *w,
                           uint32_t *h)
{
    const IngenicIspLayout *l = isp_layout(s);

    if (!pipe) {
        isp_frame_size(s, w, h);
        return;
    }
    *w = (R(l->pipe[pipe].ch + l->chan->size) >> 16) & ~1;
    *h = (R(l->pipe[pipe].ch + l->chan->size) & 0xffff) & ~1;
}

static bool isp_hldc_on(IngenicIspState *s, unsigned pipe)
{
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspBusy by = pipe ? l->hldc_sec_bypass : l->hldc_bypass;

    return isp_hldc(s, pipe) && !(R(by.off) & by.bits);
}

/*
 * The passes over a frame's rows that are worked in bands run on threads
 * that live as long as the device does, started the first time a frame is
 * large enough to want them: a pass would otherwise start and join as many.
 * The caller runs the first band itself.
 *
 * On a layout with par_unlocked the BQL is released while the bands run, so
 * that the guest's CPUs and the other devices go on with it, and the frame
 * stays whole to the guest by the register file standing still for all of
 * it. The passes read nothing of the device: each takes its inputs from the
 * structure the frame's code built for it from the registers, and works on
 * buffers it was given (the guest's memory is mapped before and unmapped
 * after, with the BQL held). What the frame's code reads between passes is
 * the registers, and nothing changes them but that code while a frame is
 * under way (isp_txn_begin() to isp_txn_end()), because every other way in
 * is held until it ends:
 *
 * - a register write goes on a list and is made, in order, as the frame
 *   ends, but for those that only drive the interrupt lines, which no pass
 *   reads and which a guest's handler needs to take effect (a level that
 *   an acknowledgement does not lower runs the handler again at once); a
 *   read of a register that is held gives the value held;
 * - a line from another device goes on the same list;
 * - a timer's frame is run from a bottom half, which another thread's frame
 *   in a pass leaves to its end (isp_frame_due());
 * - the sensor's registers, which the drawing of the picture reads, are held
 *   by image_sensor_hold() the same way;
 * - saving, loading and resetting wait in isp_quiesce() for the frame to
 *   end, a condition variable on the BQL: nothing that reaches the ISP
 *   through memory accesses waits.
 *
 * s->unlocked is set while the BQL is out, and is seen only by those that
 * took it meanwhile: the frame's own code holds the BQL whenever it runs.
 * Outside a frame (txn 0) a pass keeps the BQL, as on a layout without the
 * flag.
 */
#define ISP_BANDS           4
#define ISP_LATE_FRAMES     16      /* the frames whose lateness is kept */
#define ISP_LATE_MAX        4       /* late in them that has the BQL held */
#define ISP_HOLD_FRAMES     256     /* frames held, before one is tried */
#define MDNS_BAND_PIXELS    (1 << 17)       /* chosen: fewer go in one band */

static void isp_write_now(IngenicIspState *s, hwaddr off, uint32_t v);
static void isp_poke(IngenicIspState *s);
static void isp_act_start(IngenicIspState *s);
static uint32_t isp_clock_khz(IngenicIspState *s, hwaddr cdr);

static void isp_sensors_hold(IngenicIspState *s)
{
    if (s->sensors_held) {
        return;
    }
    s->sensors_held = true;
    for (unsigned i = 0; i < IMAGE_SENSOR_INPUTS; i++) {
        s->held_sensor[i] = image_sensor_find(i);
        if (s->held_sensor[i]) {
            image_sensor_hold(s->held_sensor[i]);
        }
    }
}

static void isp_sensors_release(IngenicIspState *s)
{
    if (!s->sensors_held) {
        return;
    }
    s->sensors_held = false;
    for (unsigned i = 0; i < IMAGE_SENSOR_INPUTS; i++) {
        if (s->held_sensor[i]) {
            image_sensor_release(s->held_sensor[i]);
        }
    }
}

/* A line or a write that comes while a pass runs is held, not made */
static void isp_hold(IngenicIspState *s,
                     void (*line)(void *opaque, int n, int level),
                     hwaddr off, uint32_t val)
{
    IngenicIspHeld h = { line, off, val };

    g_array_append_val(s->held, h);
}

static bool isp_line_held(IngenicIspState *s,
                          void (*line)(void *opaque, int n, int level),
                          int n, int level)
{
    if (!s->unlocked) {
        return false;
    }
    isp_hold(s, line, n, level);
    return true;
}

/*
 * A timer's frame is run from a bottom half, not by the timer: a thread that
 * stops the clock (qemu_clock_enable(), in vm_stop) waits with the BQL for
 * the timers' callbacks to return, and one that released the BQL for a pass
 * would wait for it. A qtest run, where a clock step must find its frames
 * done, runs it in the callback unless x-frame-bh is on.
 */
static void isp_run_due(IngenicIspState *s);

static void isp_frame_due(IngenicIspState *s, unsigned bit)
{
    s->due |= BIT(bit);
    if (s->unlocked) {
        /* another thread's frame is in a pass: its end runs this one */
        return;
    }
    if (s->frame_bh || !qtest_enabled()) {
        qemu_bh_schedule(s->bh);
    } else {
        isp_run_due(s);
    }
}

/*
 * What a frame reads of other devices is read as it begins, with the BQL
 * held: the ISP's clock from the CGU, here; the IR cut and LEDs' pins by the
 * sensor, which image_sensor_hold() freezes as the first pass starts
 */
/*
 * A frame that ran past the sensor's period (isp_frame_run() puts the next
 * back by the work it took, which loses frames) is noted: with the BQL
 * released, the guest's CPU takes the bands' cores on a small host, and a
 * heavy stream (a 4 MP sensor) loses a frame in each of a few. When
 * ISP_LATE_MAX of the last ISP_LATE_FRAMES were late the next ISP_HOLD_FRAMES
 * hold the BQL, in all the bands, and one run of ISP_LATE_FRAMES released
 * is tried after them. A frame late now and then, as a boot's are, costs
 * the stream none.
 */
static void isp_note_late(IngenicIspState *s, bool late)
{
    if (!isp_layout(s)->par_unlocked || s->hold_pass) {
        return;
    }
    s->late_hist = (s->late_hist << 1 | late) &
                   MAKE_64BIT_MASK(0, ISP_LATE_FRAMES);
    if (ctpop16(s->late_hist) >= ISP_LATE_MAX) {
        s->hold_left = ISP_HOLD_FRAMES;
        s->late_hist = 0;
    }
}

static void isp_txn_begin(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);

    if (!s->txn++ && l->par_unlocked) {
        s->hold_pass = s->hold_left > 0;
        if (s->hold_pass && !--s->hold_left) {
            s->late_hist = 0;
        }
        s->khz_cdr[0] = l->scdr ?: l->ispcdr;
        s->khz_cdr[1] = l->ispcdr;
        for (unsigned i = 0; i < 2; i++) {
            s->khz[i] = isp_clock_khz(s, s->khz_cdr[i]);
        }
        s->khz_ok = true;
    }
}

/* The frame's end: what came meanwhile is made, in the order it came */
static void isp_txn_end(IngenicIspState *s)
{
    if (--s->txn) {
        return;
    }
    s->khz_ok = false;
    isp_sensors_release(s);
    if (!s->flushing) {
        s->flushing = true;
        for (unsigned i = 0; i < s->held->len; i++) {
            IngenicIspHeld h = g_array_index(s->held, IngenicIspHeld, i);

            if (h.line) {
                h.line(s, h.off, h.val);
            } else {
                isp_write_now(s, h.off, h.val);
            }
        }
        g_array_set_size(s->held, 0);
        s->flushing = false;
    }
    if (s->due) {
        qemu_bh_schedule(s->bh);
    }
    qemu_cond_broadcast(&s->idle);
}

/*
 * For saving, loading and resetting, and for a picture asked for: wait for
 * the frame under way, if another thread's is. A frame's own code, which
 * holds the BQL, finds it unlocked not set and goes on.
 */
static void isp_quiesce(IngenicIspState *s)
{
    if (!bql_locked()) {
        return;
    }
    if (s->txn && s->unlocked) {
        s->quiesce_waits++;
    }
    while (s->txn && s->unlocked) {
        qemu_cond_wait_bql(&s->idle);
    }
}

static bool isp_may_unlock(IngenicIspState *s)
{
    return s->txn && !s->unlocked && isp_layout(s)->par_unlocked &&
           !s->hold_pass && bql_locked();
}

static void isp_unlock(IngenicIspState *s)
{
    isp_sensors_hold(s);
    s->unlocked = true;
    bql_unlock();
}

static void isp_relock(IngenicIspState *s)
{
    bql_lock();
    s->unlocked = false;
}

typedef void (*IspParFn)(void *arg, unsigned band, unsigned bands);

/*
 * The bands a pass is worked in at most: x-bands, else the pool's, or where
 * the BQL is released two fewer than the host has CPUs, as the guest's and
 * the main loop's keep theirs: more make the frames late
 */
static unsigned isp_nbands(IngenicIspState *s)
{
    if (s->nbands) {
        return CLAMP(s->nbands, 1, ISP_BANDS);
    }
    if (!isp_layout(s)->par_unlocked || s->hold_pass) {
        return ISP_BANDS;
    }
    if (!s->auto_bands) {
        s->auto_bands = CLAMP(g_get_num_processors(), 3, ISP_BANDS + 2) - 2;
    }
    return s->auto_bands;
}

typedef struct IspPoolWorker {
    IspPool *pool;
    unsigned i;
} IspPoolWorker;

struct IspPool {
    QemuThread th[ISP_BANDS - 1];
    QemuSemaphore go[ISP_BANDS - 1];
    QemuSemaphore done;
    IspPoolWorker worker[ISP_BANDS - 1];
    IspParFn fn;
    void *arg;
    unsigned bands;
    bool quit;
};

static void *isp_pool_main(void *opaque)
{
    IspPoolWorker *wk = opaque;
    IspPool *pool = wk->pool;

    for (;;) {
        qemu_sem_wait(&pool->go[wk->i]);
        if (pool->quit) {
            return NULL;
        }
        pool->fn(pool->arg, wk->i + 1, pool->bands);
        qemu_sem_post(&pool->done);
    }
}

static void isp_pool_free(IngenicIspState *s)
{
    IspPool *pool = s->pool;

    if (!pool) {
        return;
    }
    pool->quit = true;
    for (unsigned i = 0; i < ISP_BANDS - 1; i++) {
        qemu_sem_post(&pool->go[i]);
    }
    for (unsigned i = 0; i < ISP_BANDS - 1; i++) {
        qemu_thread_join(&pool->th[i]);
        qemu_sem_destroy(&pool->go[i]);
    }
    qemu_sem_destroy(&pool->done);
    g_free(pool);
    s->pool = NULL;
}

/* @fn over @bands bands of a pass, the BQL released where the layout lets */
static void isp_par(IngenicIspState *s, unsigned bands, IspParFn fn,
                    void *arg)
{
    IspPool *pool = s->pool;
    bool out;

    if (bands <= 1) {
        fn(arg, 0, 1);
        return;
    }
    if (!pool) {
        pool = s->pool = g_new0(IspPool, 1);
        qemu_sem_init(&pool->done, 0);
        for (unsigned i = 0; i < ISP_BANDS - 1; i++) {
            qemu_sem_init(&pool->go[i], 0);
            pool->worker[i] = (IspPoolWorker){ pool, i };
            qemu_thread_create(&pool->th[i], "isp-bands", isp_pool_main,
                               &pool->worker[i], QEMU_THREAD_JOINABLE);
        }
    }
    bands = MIN(bands, ISP_BANDS);
    pool->fn = fn;
    pool->arg = arg;
    pool->bands = bands;
    out = isp_may_unlock(s);
    if (out) {
        isp_unlock(s);
    }
    for (unsigned i = 0; i < bands - 1; i++) {
        qemu_sem_post(&pool->go[i]);
    }
    if (out && s->poke) {
        isp_poke(s);
    }
    if (out && s->act && !s->act_started) {
        s->act_started = true;
        isp_act_start(s);
    }
    if (out && s->pass_delay_ms) {
        g_usleep(s->pass_delay_ms * 1000);
    }
    fn(arg, 0, bands);
    for (unsigned i = 0; i < bands - 1; i++) {
        qemu_sem_wait(&pool->done);
    }
    if (out) {
        isp_relock(s);
    }
}

/*
 * The w x h window at (x, y) of a frame src_w wide, out to a plane: through
 * the guest's memory where it is one run of it, its rows copied in bands
 * without the BQL, else a row at a time
 */
typedef struct IspDma {
    uint8_t *dst;
    const uint8_t *src;
    uint32_t src_w, w, h, stride;
} IspDma;

static void isp_dma_band(void *arg, unsigned i, unsigned n)
{
    const IspDma *d = arg;

    for (uint32_t y = d->h * i / n; y < d->h * (i + 1) / n; y++) {
        memcpy(d->dst + (size_t)y * d->stride, d->src + (size_t)y * d->src_w,
               d->w);
    }
}

static void isp_dma_plane(IngenicIspState *s, hwaddr addr, const uint8_t *src,
                          uint32_t src_w, uint32_t x, uint32_t y, uint32_t w,
                          uint32_t h, uint32_t stride)
{
    hwaddr len, got;
    void *map;

    src += y * src_w + x;
    stride = stride <= w && src_w == w ? w : MAX(stride, w);
    len = got = h ? (hwaddr)(h - 1) * stride + w : 0;
    if (len >= MDNS_BAND_PIXELS) {
        map = address_space_map(&address_space_memory, addr, &got, true,
                                MEMTXATTRS_UNSPECIFIED);
        if (map && got == len) {
            IspDma d = { map, src, src_w, w, h, stride };

            isp_par(s, isp_nbands(s), isp_dma_band, &d);
            address_space_unmap(&address_space_memory, map, got, true, len);
            return;
        }
        if (map) {
            address_space_unmap(&address_space_memory, map, got, true, 0);
        }
    }
    if (stride == w && src_w == w) {
        address_space_write(&address_space_memory, addr,
                            MEMTXATTRS_UNSPECIFIED, src, w * h);
        return;
    }
    for (uint32_t i = 0; i < h; i++) {
        address_space_write(&address_space_memory, addr + i * stride,
                            MEMTXATTRS_UNSPECIFIED, src + i * src_w, w);
    }
}

/* @n bytes copied in bands, as a plane is */
typedef struct IspCopy {
    uint8_t *dst;
    const uint8_t *src;
    size_t len;
} IspCopy;

static void isp_copy_band(void *arg, unsigned i, unsigned n)
{
    const IspCopy *c = arg;
    size_t a = c->len * i / n, b = c->len * (i + 1) / n;

    memcpy(c->dst + a, c->src + a, b - a);
}

static void isp_copy(IngenicIspState *s, uint8_t *dst, const uint8_t *src,
                     size_t len)
{
    IspCopy c = { dst, src, len };

    isp_par(s, len < MDNS_BAND_PIXELS ? 1 : isp_nbands(s), isp_copy_band, &c);
}

/* An NV12 frame fw x fh, packed, down to planes w x h by nearest pixels */
typedef struct IspScale {
    const uint8_t *pic;
    uint32_t fw, fh, w, h;
    uint8_t *luma, *uv;
    uint32_t *cx, *cu;          /* the source column of each, in luma and */
                                /* chroma pairs */
} IspScale;

static void isp_scale_band(void *arg, unsigned i, unsigned n)
{
    const IspScale *c = arg;

    for (uint32_t y = c->h * i / n; y < c->h * (i + 1) / n; y++) {
        const uint8_t *src = c->pic + (size_t)(y * c->fh / c->h) * c->fw;

        for (uint32_t x = 0; x < c->w; x++) {
            c->luma[(size_t)y * c->w + x] = src[c->cx[x]];
        }
    }
    for (uint32_t y = c->h / 2 * i / n; y < c->h / 2 * (i + 1) / n; y++) {
        const uint8_t *src = c->pic + (size_t)c->fw * c->fh +
                             (size_t)(y * (c->fh / 2) / (c->h / 2)) * c->fw;

        for (uint32_t x = 0; x < c->w / 2; x++) {
            c->uv[(size_t)y * c->w + x * 2] = src[c->cu[x]];
            c->uv[(size_t)y * c->w + x * 2 + 1] = src[c->cu[x] + 1];
        }
    }
}

static void isp_scale(IngenicIspState *s, const uint8_t *pic, uint32_t fw,
                      uint32_t fh, uint32_t w, uint32_t h, uint8_t *luma,
                      uint8_t *uv)
{
    g_autofree uint32_t *cx = g_new(uint32_t, w);
    g_autofree uint32_t *cu = g_new(uint32_t, w / 2);
    IspScale c = { pic, fw, fh, w, h, luma, uv, cx, cu };

    for (uint32_t x = 0; x < w; x++) {
        cx[x] = x * fw / w;
    }
    for (uint32_t x = 0; x < w / 2; x++) {
        cu[x] = x * (fw / 2) / (w / 2) * 2;
    }
    isp_par(s, (size_t)w * h < MDNS_BAND_PIXELS ? 1 : isp_nbands(s),
            isp_scale_band, &c);
}

/*
 * Frame @frame of the scene into planes w x h, w apart, through the T41's
 * lens correction when it is on, or the T30's noise reducer when it runs
 * on the line buffer: both work on the frame the ISP takes, and the
 * model's own nearest-pixel scaler brings a channel of another size down
 * from it
 */
/*
 * A grey's raw R and B in 1/256ths of its G, as an OS04D10 sees daylight
 * (the 5000 K a T41's driver assumes with no white quads): the scene is
 * white-balanced
 */
#define ISP_RAW_R IMAGE_SENSOR_RAW_R
#define ISP_RAW_B IMAGE_SENSOR_RAW_B

static uint8_t isp_clamp8(int v)
{
    return MIN(MAX(v, 0), 255);
}

/*
 * The factors that take the scene's linear light, per colour, to the raw
 * signal: the constants above, or on a part with tuning files the sensor's
 * own response in the scene's light where its model has one
 */
static void isp_raw_factors(IngenicIspState *s, ImageSensorState *sensor,
                            double f[3])
{
    f[0] = ISP_RAW_R / 256.0;
    f[1] = 1;
    f[2] = ISP_RAW_B / 256.0;
    if (isp_layout(s)->raw_part) {
        image_sensor_raw_factors(sensor, isp_layout(s)->raw_part, f);
    }
}

/*
 * A colour of the picture in 8.8 fixed point, 0 to 255, and down to 4 levels
 * below 0 (the model's choice of how far): the scene's 8-bit YUV puts the
 * other two colours of a primary a level under 0, and clamping them there
 * adds that level back to its chroma
 */
static int isp_clamp88(int64_t v)
{
    return MIN(MAX(v, -4 * 256), 255 * 256);
}

/* One 2 x 2 block's worth of ingenic_isp_wb_picture(), in 64 bits */
static void isp_wb_block(uint8_t *l0, uint8_t *l1, uint8_t *c, const int k[3])
{
    int u = c[0] - 128, v = c[1] - 128;
    int64_t su = 0, sv = 0;

    for (unsigned i = 0; i < 4; i++) {
        uint8_t *p = (i < 2 ? l0 : l1) + i % 2;
        int ly = (*p - 16) * 298;
        int64_t pr = ly + 409 * v, pg = ly - 100 * u - 208 * v;
        int64_t pb = ly + 516 * u;
        int r = isp_clamp88(pr * k[0] >> 12);
        int g = isp_clamp88(pg * k[1] >> 12);
        int b = isp_clamp88(pb * k[2] >> 12);

        *p = isp_clamp8(16 + ((66 * r + 129 * g + 25 * b + BIT(15)) >> 16));
        su += -38 * r - 74 * g + 112 * b;
        sv += 112 * r - 94 * g - 18 * b;
    }
    c[0] = isp_clamp8(128 + ((su + BIT(17)) >> 18));
    c[1] = isp_clamp8(128 + ((sv + BIT(17)) >> 18));
}

/*
 * The kernels of the per-pixel work, which -O2 does not vectorise for a
 * length known only at run time: built for it, and where the host has AVX2
 * also for that, picked as a kernel runs (the integers come out the same
 * either way). clang vectorises at -O2 and has no optimize attribute.
 */
#ifdef __clang__
#define ISPK_OPT
#else
#define ISPK_OPT            __attribute__((optimize("O3")))
#endif

#define ISP_AWB_CHUNK       64

/* Two samples of a row as a word, read and written at any alignment */
typedef uint16_t __attribute__((may_alias, aligned(1))) IspPair;

#define ISPK(n)             isp_k_##n##_dflt
#define ISPK_ATTR           ISPK_OPT
#include "ingenic-isp-kern.c.inc"
#undef ISPK
#undef ISPK_ATTR

#ifdef CONFIG_AVX2_OPT
#define ISPK(n)             isp_k_##n##_avx2
#define ISPK_ATTR           ISPK_OPT __attribute__((target("avx2")))
#include "ingenic-isp-kern.c.inc"
#undef ISPK
#undef ISPK_ATTR
#define ISP_KERNEL(n, ...)                                          \
    ((cpuinfo & CPUINFO_AVX2) ? isp_k_##n##_avx2(__VA_ARGS__) :     \
                                isp_k_##n##_dflt(__VA_ARGS__))
#else
#define ISP_KERNEL(n, ...)  isp_k_##n##_dflt(__VA_ARGS__)
#endif

/*
 * The white-balance gains of a picture, as factors on each colour's linear
 * light (@f: R, G, B; 1.0 leaves a grey grey), put on the NV12 planes w x h,
 * w apart. The picture is the scene gamma-encoded, where a factor m on
 * linear light is m^(1/2.2) on each component. A 2x2 block's four pixels
 * share their chroma. The colours keep 8 fractional bits through the gains:
 * rounded to 8 bits in between, the scene's bars come back a level or two
 * off with gains a hair from 1.0.
 */
void ingenic_isp_wb_picture(uint8_t *luma, uint8_t *uv, uint32_t w,
                            uint32_t h, const double f[3])
{
    g_autofree uint8_t *alike = NULL;
    int k[3];
    bool fast;

    for (unsigned i = 0; i < 3; i++) {
        k[i] = lround(4096 * pow(f[i], 1 / 2.2));
    }
    if (k[0] == 4096 && k[1] == 4096 && k[2] == 4096) {
        return;
    }
    /* in 32 bits while no product with the largest colour (137k) passes 2^31 */
    fast = true;
    for (unsigned i = 0; i < 3; i++) {
        fast &= k[i] >= 0 && k[i] <= 15000;
    }
    /* a pair of rows like the last, as they came in, comes out as it did */
    alike = g_malloc0(h / 2);
    for (uint32_t y = 2; y + 1 < h; y += 2) {
        alike[y / 2] = !memcmp(luma + (size_t)y * w,
                               luma + (size_t)(y - 2) * w, (size_t)w * 2) &&
                       !memcmp(uv + (size_t)y / 2 * w,
                               uv + ((size_t)y / 2 - 1) * w, w);
    }
    for (uint32_t y = 0; y + 1 < h; y += 2) {
        uint8_t *l0 = luma + (size_t)y * w, *c0 = uv + (size_t)y / 2 * w;

        if (alike[y / 2]) {
            memcpy(l0, l0 - 2 * w, (size_t)w * 2);
            memcpy(c0, c0 - w, w);
            continue;
        }
        if (fast) {
            ISP_KERNEL(wb_blocks, l0, l0 + w, c0, w / 2, k);
            continue;
        }
        for (uint32_t x = 0; x + 1 < w; x += 2) {
            isp_wb_block(l0 + x, l0 + w + x, c0 + x, k);
        }
    }
}

/*
 * The guest's white-balance gains as factors on the picture's colours (@f: R,
 * G, B): the R and B words of the layout (1.0 at 0x400, 0 until written;
 * pipeline @pipe's a wb_pipe on), or the Apical core's ahead of a T30's
 * multi-scaler. They act on the sensor's raw signal, which the AWB
 * statistics take as the scene in linear light times isp_raw_factors() (the
 * core's, a T30's, its own). False when the gains are not set.
 */
static bool isp_wb_factors(IngenicIspState *s, ImageSensorState *sensor,
                           unsigned pipe, double f[3])
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr at = pipe * l->wb_pipe;
    double raw[3];
    uint32_t gr, gb;

    f[0] = f[1] = f[2] = 1;
    if (s->core) {
        return ingenic_t20_isp_wb_factors(s->core, sensor, f);
    }
    if (!l->wb_r) {
        return false;
    }
    gr = extract32(R(l->wb_r + at), 0, 14);
    gb = extract32(R(l->wb_b + at), 0, 14);
    if (!gr || !gb) {
        return false;
    }
    f[0] = gr / 1024.0;
    f[2] = gb / 1024.0;
    /* an RDMA frame's samples are the raw signal already */
    if (!s->raw_in_bpp) {
        isp_raw_factors(s, sensor, raw);
        f[0] *= raw[0];
        f[2] *= raw[2];
    }
    return true;
}

/* The guest's white-balance gains on the picture put out */
static void isp_wb_gains(IngenicIspState *s, ImageSensorState *sensor,
                         unsigned pipe, uint32_t w, uint32_t h, uint8_t *luma,
                         uint8_t *uv)
{
    double f[3];

    if (isp_wb_factors(s, sensor, pipe, f)) {
        ingenic_isp_wb_picture(luma, uv, w, h, f);
    }
}

/*
 * The T41's raw WDMA and RDMA (0x20000 and 0x21000), which a T41NQ runs as
 * libimp's IMP_ISP_RAW_RwControl() does:
 *
 * - The WDMA takes the frame the VIC sends, before the ISP, into the
 *   buffers queued at +0x18 (four at most; +0x40 reads (4 - n) << 12 |
 *   n << 8 | n == 0, +0x18 the buffer it writes or writes next), each of
 *   the VIC's frames while one is queued, lines +0x1c apart, 16 bits a
 *   sample with the sensor's bits at the top (+0x14 bit 16 set: RAW16) or
 *   8 (the top 8 of them), RGGB. The frame's end raises 0x400b8 bit 4 and
 *   0x40028 bit 18 (and, of a raw frame, 0x400a8 bits 3:1 and 27, which
 *   the frame the RDMA gives the core then does not end with), and puts
 *   the buffer on the done FIFO, which a read of +0x20 pops. +0x04 (write
 *   1: take the raw frame) reads 0.
 *
 * - With no raw frame asked for, the same WDMA is the IR channel's (tx-isp
 *   writes +0x14 = 0xa2 and the line stride in tisp_init(), and queues the
 *   IR channel's buffers when it is bound): one byte a sample, the IR
 *   plane, which a sensor with no IR pixels, a gc4023 as much as the
 *   model's, leaves at 0 in the whole frame.
 *
 * - The RDMA gives the core its frames in place of the VIC's once the
 *   driver has written +0x04 = 1 and a mode to +0x24 (0x2a; 0 at the end):
 *   a buffer written to +0x0c is read (+0x08 bit 0: 16 bits a sample, +0x14
 *   the line stride) and runs through the core as a frame of its own, 0.5
 *   ms after the write on silicon, 14 ms long; at its end 0x400b8 bit 5
 *   rises with the frame's bits and the buffer waits at +0x1c for a read
 *   (the driver gives it back to the WDMA at +0x18). The VIC's frames do
 *   not start the core meanwhile: a core with nothing to read has no
 *   frames.
 *
 * - The VIC's +0x1a8 bit 6 holds the VIC's frames short of the ISP (the
 *   driver sets it for the second it changes the DMAs over), the WDMA's too.
 *
 * - The statistics blocks measure the RDMA's frame, not the sensor's. On a
 *   T41NQ (gc4023, 2560 x 1440) AE's zones and histogram, AWB, AF, ADR,
 *   defog, TMO and LCE all follow the buffer: a flat one puts every quad in
 *   one bin (the lens shading's spread aside), a gradient makes the zones
 *   rise across, a colour cast shows in AWB as the samples' R:G:B, and a
 *   buffer left alone gives the VIC's frame's statistics. Here they take the
 *   samples' picture (isp_raw_scene()) through the sensor path's code, AWB
 *   without ISP_RAW_R and ISP_RAW_B: the samples are the raw signal already,
 *   so a flat buffer has no white quads (as on silicon: the driver's 5000 K
 *   fallback, and isp_wb_gains() above). Silicon also takes 16 of 255 off the
 *   samples and applies its lens shading and a digital gain first, and its
 *   samples are linear where the model's are the picture's: the levels
 *   differ, which statistics see which content does not. The WDR block needs
 *   two exposures and does not run.
 *
 * - Timing, not modelled: the RDMA's frame starts 0.4 to 0.6 ms after the
 *   write to +0x0c and ends 14.25 ms after it (14.0 to 14.4 ms in 24 frames,
 *   RAW8 and RAW16 alike, the write 0, 40 or 50 ms into the WDMA's 54 ms
 *   frame), the rows counted in +0x54. It does not hold up the VIC's next
 *   frame, nor does that frame hold it: the WDMA's frame ends stayed 54.0 ms
 *   apart with an RDMA running across them. A frame that starts with no
 *   buffer queued is dropped, with no done bits (0x400a8 still ticks), and
 *   the buffer a guest returns while a frame is in flight is the next
 *   frame's; here a frame takes its buffer at its end, a frame early.
 *   Nothing the driver does depends on the delay: tisp_raw_set_rw_handler()
 *   ends with the write, and the done handler gives the buffer back and
 *   completes a counting completion, which the disable path waits on, so a
 *   done raised inside the write loses no wakeup.
 */
#define RAW_WDMA            0x0000
#define RAW_RDMA            0x1000
#define RAW_START           0x04    /* write 1 */
#define RAW_WDMA_CFG        0x14
#define RAW_WDMA_ADDR       0x18
#define RAW_WDMA_STRIDE     0x1c
#define RAW_WDMA_DONE       0x20
#define RAW_WDMA_FIFO       0x40
#define RAW_RDMA_FMT        0x08
#define RAW_RDMA_ADDR       0x0c
#define RAW_RDMA_STRIDE     0x14
#define RAW_RDMA_DONE       0x1c
#define RAW_RDMA_MODE       0x24
#define RAW_VIC_SIZE        0x004   /* the VIC's input size */
#define RAW_VIC_HOLD        BIT(6)
#define RAW_DMA_WDMA        BIT(4)  /* in 0x400b8 */
#define RAW_DMA_RDMA        BIT(5)
#define RAW_INT_WDMA        BIT(18) /* in 0x40028 */
#define RAW_TICK_WDMA       (BIT(27) | 0xe) /* in 0x400a8 */

/* The frame the VIC sends, from its input size register */
static bool isp_raw_size(IngenicIspState *s, uint32_t *w, uint32_t *h)
{
    const IngenicIspLayout *l = isp_layout(s);
    uint32_t size = VIC(0, RAW_VIC_SIZE);

    *w = (size >> 16) & ~1;
    *h = (size & 0xffff) & ~1;
    return *w && *h;
}

/* The sample at (x, y) of a Bayer frame, mirrored at its edges */
static inline int isp_raw_at(const uint8_t *v, uint32_t fw, uint32_t fh,
                             int x, int y)
{
    x = x < 0 ? 1 : x >= (int)fw ? fw - 2 : x;
    y = y < 0 ? 1 : y >= (int)fh ? fh - 2 : y;
    return v[(size_t)y * fw + x];
}

/* RGGB's colours at (x, y): its own sample, the neighbours' means else */
static inline void isp_raw_rgb(const uint8_t *v, uint32_t fw, uint32_t fh,
                               int x, int y, int *r, int *g, int *b)
{
    int p = isp_raw_at(v, fw, fh, x, y);
    int hor = (isp_raw_at(v, fw, fh, x - 1, y) +
               isp_raw_at(v, fw, fh, x + 1, y)) / 2;
    int ver = (isp_raw_at(v, fw, fh, x, y - 1) +
               isp_raw_at(v, fw, fh, x, y + 1)) / 2;
    int diag = (isp_raw_at(v, fw, fh, x - 1, y - 1) +
                isp_raw_at(v, fw, fh, x + 1, y - 1) +
                isp_raw_at(v, fw, fh, x - 1, y + 1) +
                isp_raw_at(v, fw, fh, x + 1, y + 1)) / 4;

    switch ((y & 1) << 1 | (x & 1)) {
    case 0:
        *r = p;
        *g = (hor + ver) / 2;
        *b = diag;
        break;
    case 1:
        *r = hor;
        *g = p;
        *b = ver;
        break;
    case 2:
        *r = ver;
        *g = p;
        *b = hor;
        break;
    default:
        *r = diag;
        *g = (hor + ver) / 2;
        *b = p;
        break;
    }
}

/*
 * The picture a raw frame in memory makes, planes w x h: each pixel's
 * colours from its RGGB neighbours, in BT.601 as image_sensor_render_bayer()
 * takes the sensor's picture apart, the nearest source pixel for a size other
 * than the frame's
 */
static void isp_raw_scene(IngenicIspState *s, uint32_t w, uint32_t h,
                          uint8_t *luma, uint8_t *uv)
{
    uint32_t fw, fh, bpp = s->raw_in_bpp, stride = s->raw_in_stride;
    g_autofree uint8_t *row = NULL;
    uint8_t *v8 = s->raw_in_v8;

    if (!isp_raw_size(s, &fw, &fh)) {
        memset(luma, 16, (size_t)w * h);
        memset(uv, 128, (size_t)w * h / 2);
        return;
    }
    if (!v8) {
        /* the frame's top 8 bits, read once for all of its passes */
        v8 = s->raw_in_v8 = g_malloc((size_t)fw * fh);
        row = g_malloc((size_t)fw * bpp);
        for (uint32_t y = 0; y < fh; y++) {
            address_space_read(&address_space_memory,
                               s->raw_in_addr + (hwaddr)y * stride,
                               MEMTXATTRS_UNSPECIFIED, row, fw * bpp);
            for (uint32_t x = 0; x < fw; x++) {
                v8[(size_t)y * fw + x] = bpp == 2 ? row[2 * x + 1] : row[x];
            }
        }
    }
    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            int r, g, b;

            isp_raw_rgb(v8, fw, fh, x * fw / w, y * fh / h, &r, &g, &b);
            luma[(size_t)y * w + x] =
                isp_clamp8(16 + ((66 * r + 129 * g + 25 * b + 128) >> 8));
            if (!(y & 1) && !(x & 1)) {
                uv[(size_t)y / 2 * w + x] =
                    isp_clamp8(128 + ((-38 * r - 74 * g + 112 * b + 128) >> 8));
                uv[(size_t)y / 2 * w + x + 1] =
                    isp_clamp8(128 + ((112 * r - 94 * g - 18 * b + 128) >> 8));
            }
        }
    }
}

static void isp_scene_draw(IngenicIspState *s, ImageSensorState *sensor,
                           unsigned pipe, uint32_t frame, uint32_t w,
                           uint32_t h, uint8_t *luma, uint8_t *uv)
{
    g_autofree uint8_t *pic = NULL;
    const uint8_t *nr = NULL;
    uint32_t fw, fh;

    if (s->raw_in_bpp) {
        isp_raw_scene(s, w, h, luma, uv);
        return;
    }
    if (s->ncu) {
        nr = ingenic_t30_ncu_lfb_frame(s->ncu, sensor, frame, &fw, &fh);
    }
    if (nr) {
        isp_scale(s, nr, fw, fh, w, h, luma, uv);
        return;
    }
    if (!isp_hldc_on(s, pipe)) {
        image_sensor_render_planes(sensor, frame, w, h, luma, uv);
        return;
    }
    isp_hldc_frame(s, pipe, &fw, &fh);
    if (!fw || !fh || (fw == w && fh == h)) {
        image_sensor_render_planes(sensor, frame, w, h, luma, uv);
        ingenic_hldc_remap(&s->hldc[pipe], luma, uv, w, h);
        return;
    }
    pic = g_malloc((size_t)fw * fh * 3 / 2);
    image_sensor_render(sensor, frame, fw, fh, pic);
    ingenic_hldc_remap(&s->hldc[pipe], pic, pic + (size_t)fw * fh, fw, fh);
    isp_scale(s, pic, fw, fh, w, h, luma, uv);
}

/*
 * The passes of one frame (the MDNS, the statistics, a channel) draw the
 * sensor's picture of it through pipe 0's gains at the same size more than
 * once. Inside isp_core_frame() they share it: a pass that finds the picture
 * kept copies it or, if it only reads it, reads it there; one that makes it
 * keeps it. Only the plain draw is shared:
 * the lens correction, the T30's NR frame and an RDMA frame's samples make
 * the picture some other way.
 */
static bool isp_pic_plain(IngenicIspState *s)
{
    return s->pic_scope && !s->raw_in_bpp && !s->ncu && !isp_hldc_on(s, 0);
}

static IngenicIspPic *isp_pic_find(IngenicIspState *s,
                                   ImageSensorState *sensor, uint32_t frame,
                                   uint32_t w, uint32_t h)
{
    if (!isp_pic_plain(s)) {
        return NULL;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(s->pic); i++) {
        if (s->pic[i].valid && s->pic[i].sensor == sensor &&
            s->pic[i].frame == frame && s->pic[i].w == w &&
            s->pic[i].h == h) {
            return &s->pic[i];
        }
    }
    return NULL;
}

static bool isp_pic_get(IngenicIspState *s, ImageSensorState *sensor,
                        uint32_t frame, uint32_t w, uint32_t h, uint8_t *dst)
{
    IngenicIspPic *e = isp_pic_find(s, sensor, frame, w, h);

    if (e) {
        memcpy(dst, e->buf, (size_t)w * h * 3 / 2);
    }
    return e;
}

/* The entry the next picture of a frame goes in, not yet valid */
static IngenicIspPic *isp_pic_entry(IngenicIspState *s,
                                    ImageSensorState *sensor, uint32_t frame,
                                    uint32_t w, uint32_t h)
{
    IngenicIspPic *e = &s->pic[s->pic_next++ % ARRAY_SIZE(s->pic)];

    e->sensor = sensor;
    e->frame = frame;
    e->w = w;
    e->h = h;
    e->valid = false;
    return e;
}

/* The same with a buffer of its own for @w x @h */
static IngenicIspPic *isp_pic_slot(IngenicIspState *s,
                                   ImageSensorState *sensor, uint32_t frame,
                                   uint32_t w, uint32_t h)
{
    size_t len = (size_t)w * h * 3 / 2;
    IngenicIspPic *e = isp_pic_entry(s, sensor, frame, w, h);

    if (e->cap < len) {
        g_free(e->own);
        e->own = g_malloc(len);
        e->cap = len;
    }
    e->buf = e->own;
    return e;
}

static void isp_pic_keep(IngenicIspState *s, ImageSensorState *sensor,
                         uint32_t frame, uint32_t w, uint32_t h,
                         const uint8_t *src)
{
    IngenicIspPic *e;

    if (!isp_pic_plain(s)) {
        return;
    }
    e = isp_pic_slot(s, sensor, frame, w, h);
    memcpy(e->buf, src, (size_t)w * h * 3 / 2);
    e->valid = true;
}

/*
 * The picture as the passes take it, in the frame's cache for a pass that
 * reads it where it lies: NULL out of a frame's passes or when it is not
 * the plain draw. What it points to holds until the pass makes another
 * picture of the frame, which may take its entry.
 */
static const uint8_t *isp_pic_ref(IngenicIspState *s, ImageSensorState *sensor,
                                  uint32_t frame, uint32_t w, uint32_t h)
{
    IngenicIspPic *e = isp_pic_find(s, sensor, frame, w, h);

    if (!e && isp_pic_plain(s)) {
        e = isp_pic_slot(s, sensor, frame, w, h);
        isp_scene_draw(s, sensor, 0, frame, w, h, e->buf,
                       e->buf + (size_t)w * h);
        isp_wb_gains(s, sensor, 0, w, h, e->buf, e->buf + (size_t)w * h);
        e->valid = true;
    }
    return e ? e->buf : NULL;
}

/*
 * The picture as the passes take it: drawn and put through the gains. A
 * caller whose buffer @lend holds until the frame's end lets the cache read
 * it there instead of keeping a copy.
 */
static void isp_pic_draw(IngenicIspState *s, ImageSensorState *sensor,
                         uint32_t frame, uint32_t w, uint32_t h, uint8_t *nv12,
                         bool lend)
{
    if (isp_pic_get(s, sensor, frame, w, h, nv12)) {
        return;
    }
    isp_scene_draw(s, sensor, 0, frame, w, h, nv12, nv12 + (size_t)w * h);
    isp_wb_gains(s, sensor, 0, w, h, nv12, nv12 + (size_t)w * h);
    if (lend && isp_pic_plain(s)) {
        /* @nv12 outlives the frame's passes and they do not change it */
        IngenicIspPic *e = isp_pic_entry(s, sensor, frame, w, h);

        e->buf = nv12;
        e->valid = true;
        return;
    }
    isp_pic_keep(s, sensor, frame, w, h, nv12);
}

/*
 * The NV12 picture a statistics pass measures, @w x @h with the chroma plane
 * after the luma: from the buffer when the core is reading one (false
 * otherwise, the caller renders the sensor's)
 */
static bool isp_stats_raw_pic(IngenicIspState *s, uint32_t w, uint32_t h,
                              uint8_t *pic)
{
    if (!s->raw_in_bpp) {
        return false;
    }
    isp_raw_scene(s, w, h, pic, pic + (size_t)w * h);
    return true;
}

/*
 * The T23's and T31's 3D noise reducer (MDNS, 0x7800), between the colour
 * conversion and the sharpening, as a T23N and a T31AL run it with
 * libimp's settings. tx-isp gives it its buffers in DDR (tx_isp_set_buf):
 * a reference frame, NV12, Y at +0x20 (stride +0x24) and UV at +0x28
 * (+0x2c), four statistics buffers at +0x30, +0x40, +0x48 and +0x50
 * (strides at +4), and three of a background model (+0x58 to +0x6c) that
 * libimp leaves unused. Each frame it blends the frame the ISP takes with
 * the reference and writes the result back there, which is what a T23's
 * channels scale from (a T31's take it smoothed and sharpened, see
 * isp_mdns_spa()):
 *
 * - Each 16 x 16 block has an age, 0 where the block moved (the mean of
 *   its pixels' differences from the reference, signed, above +0x7c bits
 *   23:16), else one more than last frame's, up to +0x78 bits 7:4. The
 *   reference weighs the byte of +0x148 to +0x154 its block's age selects,
 *   in 1/256ths, at a pixel whose 3 x 3's mean difference from the
 *   reference, unsigned and doubled, is within +0x144 bits 11:3 (+0x264
 *   for U and V), and nothing where it is not. U and V take their luma
 *   block's age. In a block that has not moved the weight is held between
 *   bits 31:24 and 23:16 of +0x19c for Y and +0x26c for U and V (libimp's
 *   mdns_y_ and mdns_c_ref_wei_b_min and b_max): a T31X at 0xd1 for UV,
 *   whose age 15 weighs 0xfa, closes about a fifth of the gap a frame after
 *   a step in its white-balance gains, and follows a slow walk of them, as
 *   209/256 does; held to 0x80 its noise between frames rises (Y from 1.3
 *   levels to 2.6, at 0xf3 by default; UV from 0.76 to 1.14), and with no
 *   limit below the age's 0xfa its UV holds still (0.04). Rounding to 8 bits
 *   holds a still picture nearly frozen, as it holds a T23N's: its noise
 *   between frames, 3.5 levels with the reduction off, falls to 0.3 (and its
 *   chroma's from 1.0 to 0.01). A T31X's reference in DDR is this blend
 *   with the still weight capped by b_max (its change between frames
 *   follows (1 - w) / sqrt(1 + w) of the raw noise's from b_max 0 to 0xfa,
 *   0.4 levels at 0xf3, and b_min = b_max = 0xff freezes it), but what its
 *   channels put out is that reference smoothed in space (+0x1a4 to +0x1fc)
 *   and then sharpened after the block, so its picture keeps 1.1 levels of
 *   noise a second apart where the reference keeps 2.1; a T23N's likewise
 *   sharpened. The model does both for a T31's Y (see isp_mdns_spa()); a
 *   T23's channels take the reference itself. The rest of +0x198, +0x1a0
 *   and +0x268 to +0x270 (the moving blocks' limits, the increase) is not
 *   modelled.
 *
 * - The statistics buffers each take a byte per 32 x 16, the ages of its
 *   left (bits 3:0) and right (7:4) block, for the columns and rows of the
 *   grids at +0x74, +0x8c, +0xa4 and +0xbc (bits 31:24, 23:16); the first
 *   from its second row, the others from their first, the rest 0. The
 *   grids sit 8 pixels apart on a T31AL, here all four take the first's
 *   blocks. With isp_memopt the four share one buffer, the first's last.
 *
 * - +0x0 reads the block's ID (a T23's 0x23021515), and +0xc, +0x1c and
 *   +0x304 up 0x20151120; +0x4 bit 0 (the driver's update) and +0x18 bit
 *   0 clear as written (a T23's +0x18 then reads 0x20), and +0x300 reads
 *   0x4040, as both read between frames.
 *
 * The T41's (0xf000, tx_isp_set_mdns_buf) writes its reference to +0x40
 * and +0x48 (strides at +4) and its blocks' ages, two a byte from the
 * buffer's first row, to +0x50 (+0x54), for the blocks of +0x10c (across
 * in bits 15:0, down in 31:16), up to 7, as a T41LQ does; the driver's
 * room after each plane (a 32nd of it) stays as it is. A block is +0x100
 * pixels square (a tuning value that follows the gain, 36 on a T40XP at
 * one gain, 27 on a T32NQ) from the origin at +0x104 (x in 7:0, y in
 * 15:8); each row of ages is written in whole 8-byte groups, and what
 * lies past it, left by an earlier size, stays. Its filter is not
 * modelled: the reference is the frame (the T41LQ looked at a scene too
 * bright to measure it), and its blocks' ages follow the T31's rule with
 * a threshold of 5.
 *
 * The T40's (0xf000; its second pipeline's, at 0xf800, is left alone) is
 * the T41's block with the same buffers, but what a T40XP puts in the
 * reference is a coded stream (a flat white scene reads as an 11-byte
 * pattern repeated, a dim one as noise, and each word is rewritten every
 * other frame): the model does not decode it and leaves the planes as the
 * driver set them. The ages are the T41's (blocks of +0x100, rows from 0,
 * up to 7) and a T40XP's count one more each frame; a block that moves
 * (a step of 5 in the output's luma does, 4 does not) stays 0 for two
 * frames, so they are measured against the model's own luma from two
 * frames back. +0x4 and +0x8, the driver's update and start, read 0 once
 * written.
 *
 * The T32's (0xd000, IDs 0x24041703 and 0x24041928) has the same blocks
 * (+0x100, +0x104, +0x10c) and a coded reference as well, in one buffer at
 * +0x40 (its size in 256-byte units at +0x80, strides it does not write
 * reading 0x780) when the compression ratio its driver is given, in
 * thousandths, is over 1000, as a T32NQ's is; a driver given less lays out
 * planes there, which the model does not write either. Its ages, in the
 * buffer at +0x50 (+0x54), are 5-bit fields, twelve to a little-endian
 * 8-byte group, a row of blocks to a row of the stride, up to 15; a block
 * that moves is 0 for one frame. The model measures them against its own
 * luma of the frame before, with the T31's threshold of 5 (the T32NQ's
 * scene did not change to measure it); what lies past the grid is ages of
 * an earlier block size, left as it was. +0x14 and +0x18 read 0 once
 * written.
 *
 * The T21's (0x1b00) has its reference at +0x10 and +0x20 (strides at
 * +4) and puts its statistics in two rings of slots (+0x30 and +0x40,
 * row stride at +4, slot size at +8) that libimp reads (IMP_ISP_Tuning_
 * GetNCUInfo, through 0x1b34, 0x1b38 and 0x1b78, which read back as
 * written): of the 20 slots it is given a T21N fills 16 in turn, see
 * isp_mdns_rings(). Its filter is not modelled either, the blocks' ages
 * following the T31's rule up to +0x80 bits 3:0; a T21N's first ring
 * holds in bits 15:6 a measure not decoded (with flags in 5:0), which the
 * model leaves as the driver gave it: with a frame count in its header and
 * a measure of its own there, a vendor T21N demo segfaults in libimp.
 *
 * Not modelled: a T23's spatial filters and sharpening, the rest of a T31's,
 * the motion vectors, the corner, edge and luma adjustments, and the
 * background model. Writing +0x10 = 0x101
 * (tisp_mdns_bypass()'s value) with +0x14 as libimp leaves it stopped a
 * T23N's and a T31AL's ISP: no frames until a power cycle.
 */
#define MDNS_ID_ANY         0x20151120
#define MDNS(o)             R(m->base + (o))

static bool isp_mdns_read(IngenicIspState *s, hwaddr off, uint32_t *v)
{
    const IngenicIspMdns *m = isp_layout(s)->mdns;

    if (!m || !m->id || off < m->base || off >= m->base + 0x800) {
        return false;
    }
    switch (off - m->base) {
    case 0x000:
        *v = m->id;
        return true;
    case 0x00c:
    case 0x01c:
        *v = MDNS_ID_ANY;
        return true;
    case 0x300:
        *v = 0x4040;
        return true;
    }
    if (off - m->base > 0x300) {
        *v = MDNS_ID_ANY;
        return true;
    }
    return false;
}

static bool isp_mdns_write(IngenicIspState *s, hwaddr off, uint32_t v)
{
    const IngenicIspMdns *m = isp_layout(s)->mdns;

    if (!m || off < m->base || off >= m->base + 0x800) {
        return false;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(m->clear); i++) {
        if (m->clear[i] && off == m->base + m->clear[i]) {
            MDNS(m->clear[i]) = v & ~1u;
            return true;
        }
    }
    if (!m->id) {
        return false;
    }
    switch (off - m->base) {
    case 0x004:
        MDNS(0x004) = v & ~1u;
        return true;
    case 0x018:
        MDNS(0x018) = (v & ~1u) | m->refresh_reads;
        return true;
    }
    return false;
}

/*
 * The reference's plane at @addr, @w x @h: in place where it is one run of
 * RAM (*@map set, to unmap), else read into @dst
 */
static const uint8_t *isp_mdns_ref(hwaddr addr, uint32_t stride, uint8_t *dst,
                                   uint32_t w, uint32_t h, void **map,
                                   hwaddr *len)
{
    *map = NULL;
    *len = (hwaddr)w * h;
    if (stride == w) {
        *map = address_space_map(&address_space_memory, addr, len, false,
                                 MEMTXATTRS_UNSPECIFIED);
        if (*map && *len == (hwaddr)w * h) {
            return *map;
        }
        if (*map) {
            address_space_unmap(&address_space_memory, *map, *len, false, 0);
            *map = NULL;
        }
    }
    for (uint32_t y = 0; y < h; y++) {
        address_space_read(&address_space_memory, addr + (hwaddr)y * stride,
                           MEMTXATTRS_UNSPECIFIED, dst + (size_t)y * w, w);
    }
    return dst;
}

#define MDNS_SEG            64      /* samples checked for change at once */

/* Scratch the MDNS keeps between frames, so as not to fault it in anew */
static void *isp_mdns_work(IngenicIspState *s, unsigned k, size_t len)
{
    if (s->mdns_work_len[k] < len) {
        g_free(s->mdns_work[k]);
        s->mdns_work[k] = g_malloc(len);
        s->mdns_work_len[k] = len;
    }
    return s->mdns_work[k];
}

/* @o[x0..x1) = |cur - ref| summed over x - step, x and x + step */
static void isp_mdns_sad3(const uint8_t *c, const uint8_t *r, uint16_t *o,
                          uint32_t x0, uint32_t x1, uint32_t w, unsigned step)
{
    uint32_t a = MAX(x0, step), b = MIN(x1, w - MIN(w, step));

    for (uint32_t x = x0; x < x1; x++) {
        uint32_t xl, xr;

        if (x == a && a < b) {
            ISP_KERNEL(mdns_sad3, c + a - step, r + a - step, c + a, r + a,
                       c + a + step, r + a + step, o + a, b - a);
            x = b - 1;
            continue;
        }
        xl = x >= step ? x - step : x;
        xr = x + step < w ? x + step : x;
        o[x] = abs(c[xl] - r[xl]) + abs(c[x] - r[x]) + abs(c[xr] - r[xr]);
    }
}

/*
 * Blend plane @cur with @ref (@w samples a row, @step apart, rows @h) into
 * @out: the reference weighs wt[age of the sample's block] where the 3 x 3
 * of like samples' mean |cur - ref|, doubled, is within @thr; @wmap takes
 * the weight each sample got. It is worked in passes over bands of rows,
 * each of which needs the rows next to its own from the one before.
 */
typedef struct IspMdnsBlend {
    const uint8_t *cur, *ref;
    uint8_t *out, *wmap, *wrow;
    uint16_t *hs;
    bool *diff, *nearby;
    uint32_t w, h, segs, bw;
    unsigned step, bsh_x, bsh_y, pass;
    const uint8_t *age, *wt;
    uint16_t lim;
} IspMdnsBlend;

static void isp_mdns_blend_band(void *arg, unsigned i, unsigned n)
{
    const IspMdnsBlend *b = arg;
    uint32_t w = b->w, h = b->h, segs = b->segs;
    uint32_t y0 = h * i / n, y1 = h * (i + 1) / n;
    uint8_t *wrow = b->wrow + (size_t)i * w;

    switch (b->pass) {
    case 0:
        /* where a segment of cur and ref match */
        for (uint32_t y = y0; y < y1; y++) {
            for (uint32_t sg = 0; sg < segs; sg++) {
                size_t k = (size_t)y * w + sg * MDNS_SEG;
                uint32_t len = MIN(MDNS_SEG, w - sg * MDNS_SEG);

                b->diff[y * segs + sg] =
                    memcmp(b->cur + k, b->ref + k, len) != 0;
            }
        }
        break;
    case 1:
        /* and which have one next to them */
        for (uint32_t y = y0; y < y1; y++) {
            for (uint32_t sg = 0; sg < segs; sg++) {
                bool any = false;

                for (uint32_t yy = y ? y - 1 : 0; yy <= MIN(y + 1, h - 1);
                     yy++) {
                    for (uint32_t k = sg ? sg - 1 : 0;
                         k <= MIN(sg + 1, segs - 1); k++) {
                        any |= b->diff[yy * segs + k];
                    }
                }
                b->nearby[y * segs + sg] = any;
            }
        }
        break;
    case 2:
        for (uint32_t y = y0; y < y1; y++) {
            const uint8_t *c = b->cur + (size_t)y * w;
            const uint8_t *r = b->ref + (size_t)y * w;
            uint16_t *o = b->hs + (size_t)y * w;

            for (uint32_t sg = 0; sg < segs; sg++) {
                uint32_t x0 = sg * MDNS_SEG, x1 = MIN(x0 + MDNS_SEG, w);

                /* the rows a sample worked through takes its 3 x 3 from */
                if (!b->nearby[y * segs + sg] &&
                    !(y && b->nearby[(y - 1) * segs + sg]) &&
                    !(y + 1 < h && b->nearby[(y + 1) * segs + sg])) {
                    continue;
                }
                isp_mdns_sad3(c, r, o, x0, x1, w, b->step);
            }
        }
        break;
    default:
        for (uint32_t y = y0; y < y1; y++) {
            uint32_t ya = y ? y - 1 : 0, yb = y + 1 < h ? y + 1 : y;
            const uint16_t *up = b->hs + (size_t)ya * w;
            const uint16_t *mid = b->hs + (size_t)y * w;
            const uint16_t *dn = b->hs + (size_t)yb * w;
            const uint8_t *c = b->cur + (size_t)y * w;
            uint8_t *o = b->out + (size_t)y * w;

            if (y == y0 || (y >> b->bsh_y) != ((y - 1) >> b->bsh_y)) {
                const uint8_t *ab = b->age + (y >> b->bsh_y) * b->bw;
                unsigned sh = b->bsh_x + (b->step == 2);

                for (uint32_t x = 0; x < w; x++) {
                    wrow[x] = b->wt[ab[x >> sh]];
                }
            }
            for (uint32_t sg = 0; sg < segs; sg++) {
                uint32_t x0 = sg * MDNS_SEG, x1 = MIN(x0 + MDNS_SEG, w);

                if (!b->nearby[y * segs + sg]) {
                    memcpy(o + x0, c + x0, x1 - x0);
                    memcpy(b->wmap + (size_t)y * w + x0, wrow + x0, x1 - x0);
                    continue;
                }
                ISP_KERNEL(mdns_mix, c + x0, b->ref + (size_t)y * w + x0,
                           up + x0, mid + x0, dn + x0, wrow + x0, o + x0,
                           b->wmap + (size_t)y * w + x0, x1 - x0, b->lim);
            }
        }
        break;
    }
}

static void isp_mdns_blend(IngenicIspState *s, const uint8_t *cur,
                           const uint8_t *ref, uint8_t *out, uint32_t w,
                           uint32_t h, unsigned step, unsigned bsh_x,
                           unsigned bsh_y, const uint8_t *age, uint32_t bw,
                           const uint8_t *wt, uint32_t thr, uint8_t *wmap)
{
    uint32_t segs = DIV_ROUND_UP(w, MDNS_SEG);
    unsigned bands = (size_t)w * h < MDNS_BAND_PIXELS ? 1 : isp_nbands(s);
    IspMdnsBlend b = {
        .cur = cur, .ref = ref, .out = out, .wmap = wmap,
        .hs = isp_mdns_work(s, 0, (size_t)w * h * 2),
        .diff = isp_mdns_work(s, 1, (size_t)segs * h),
        .nearby = isp_mdns_work(s, 2, (size_t)segs * h),
        .wrow = isp_mdns_work(s, 3, (size_t)w * bands),
        .w = w, .h = h, .segs = segs, .bw = bw, .step = step,
        .bsh_x = bsh_x, .bsh_y = bsh_y, .age = age, .wt = wt,
        .lim = MIN(9 * thr, 0xffff),
    };

    /*
     * Where cur and ref match over a sample's 3 x 3 it comes out as it went
     * in: only segments with a difference next to them are worked through
     */
    for (b.pass = 0; b.pass < 4; b.pass++) {
        isp_par(s, bands, isp_mdns_blend_band, &b);
    }
}

/*
 * What a T31X's channels take is not the reference: its Y goes on through
 * a spatial filter and then the sharpening. Both were measured on a T31X
 * with the reference held still (b_min = b_max = 0xff, so that the frame
 * the channels take is a function of one fixed reference), the sharpening
 * off or on by its weights, and the constants below are fits to that.
 *
 * The filter takes each sample towards a 5 x 5 mean of its neighbours, by
 * a fraction k that is 0.97 where the mean is within 2 levels of the
 * sample and falls to 0.5 by 6 and 0.1 by 12 (an edge keeps its sample):
 * isp_mdns_spa_k, measured at a T31X's defaults. The mean's weights
 * ({36, 58, 68, 58, 36} / 256 a side) fit the least-squares kernel of a
 * flat part, whose support is exactly 5 x 5. The registers that move it:
 *
 * - The fraction rises with the reference's weights in +0x1fc, measured
 *   from 0 to 255 (the sweep of its four bytes' mean, the other twelve
 *   weights of +0x1f0 to +0x1f8 not moving the picture): k at a mean
 *   within 2 levels goes from 0.94 to 1.07 and the distance at which it
 *   is 0.5 from 4.9 to 9. Fitted linearly.
 * - A sample's own weight, the byte at +0x1b8, sets how much of its
 *   frame is smoothed: with the reference not in the picture (b_max 0)
 *   a T31X's noise a frame to the next is 0.87 of the reference's at
 *   weight 0, 0.38 at 0x80 and 0.21 at 0xff, a fraction of 0.14, 0.70
 *   and 1 of the mean; fitted as 0.14 + 0.86 (1 - (1 - w) ^ 1.5). The two
 *   are mixed by the weight the blend gave the reference at the sample.
 * - The weights at +0x1c8 to +0x1d4 and +0x1a4 to +0x1b0 did not move a
 *   held reference's picture; +0x1d8 to +0x1ec (its edge limits) move
 *   its tail only. Not modelled. Where the guest has written no weights
 *   the block is taken as unprogrammed and passes the reference on.
 *
 * The sharpening (at 0x7000, the ISP's) adds to a sample the high-pass of
 * its 5 x 5, a zero-sum kernel measured as the least-squares fit of the
 * difference the sharpening made on a held picture (a centre of 0.2225,
 * its 4 neighbours 0.1045, its diagonals 0.016, the 16 beyond -0.044), by
 * a gain per side: 0.0334 a unit of the weight (6 bits) at +0x7048 to
 * +0x7054 for a rise and 0.0839 a unit of +0x7058 to +0x7064 for a dip.
 * The weight's byte is the age of the sample's block (on a still picture
 * only the 15th moved the sharpening; the others serve moving blocks). Where
 * the 5 x 5's standard deviation is under 2.5 levels it adds nothing,
 * and over 4 it adds all, a ramp in the variance between (chosen: the
 * measurement shows only the ends, a flat part's noise being passed
 * unsharpened). The ends of its curve (the 0x7068 to 0x7074 slopes and
 * the limits at 0x702c to 0x7034) only moved the large edges' tails and
 * are not modelled; nor are the thresholds at 0x7000 to 0x7028.
 * The picture on a T31X's channel is also 1.047 times this, flat in
 * level, where no register tried moved it; the model leaves that out.
 */
static const struct {
    float d, k;
} isp_mdns_spa_k[] = {
    { 0.5, .97 }, { 1.5, .93 }, { 2.5, .91 }, { 3.5, .85 }, { 4.5, .76 },
    { 5.5, .62 }, { 6.5, .47 }, { 7.5, .34 }, { 9, .23 }, { 11, .13 },
    { 13.5, .07 }, { 17.5, .04 }, { 25, .03 }, { 40, .03 }, { 152, .02 },
};

static double isp_mdns_spa_curve(double d)
{
    unsigned n = ARRAY_SIZE(isp_mdns_spa_k);

    if (d <= isp_mdns_spa_k[0].d) {
        return isp_mdns_spa_k[0].k;
    }
    for (unsigned i = 1; i < n; i++) {
        if (d <= isp_mdns_spa_k[i].d) {
            double t = (d - isp_mdns_spa_k[i - 1].d) /
                       (isp_mdns_spa_k[i].d - isp_mdns_spa_k[i - 1].d);

            return isp_mdns_spa_k[i - 1].k +
                   t * (isp_mdns_spa_k[i].k - isp_mdns_spa_k[i - 1].k);
        }
    }
    return isp_mdns_spa_k[n - 1].k;
}

/*
 * The two filters run as one sweep over the rows of a band of the picture:
 * a sharpened row takes the smoothed rows two either side of it, which a
 * ring of rows holds as the smoothing produces them. A band starts by
 * smoothing the two rows above its first, which the band above also does,
 * so that the bands need not wait for each other; each writes its own rows
 * of the output.
 */
#define MDNS_RING           8               /* smoothed rows kept: 6 are used */

typedef struct IspMdnsJob {
    void (*band)(const struct IspMdnsJob *p);   /* the build to run */
    uint32_t w, h;
    const uint8_t *src;
    uint8_t *dst;
    const uint8_t *wmap;            /* spatial filter: the blend's weights */
    uint32_t kk[256];               /* the smoothing's fractions, see */
                                    /* isp_mdns_fractions() */
    bool sharp;
    const uint8_t *age;             /* sharpening: the blocks' ages */
    const uint16_t *xb;             /* the block each column falls in */
    uint32_t bw, bh, bsz, gy;
    int32_t gr[16], gd[16];         /* the gains by age, in 1/256ths */
    uint32_t y0, y1;                /* the rows of this band */
} IspMdnsJob;

/* The smoothing of rows one after the other, from @j0 */
typedef struct IspMdnsSpa {
    const IspMdnsJob *p;
    uint32_t j0;
    uint16_t *ring[5];              /* the rows' taps, 5 at a time */
    int32_t *dv;
} IspMdnsSpa;

/* A gain per unit of the weight's 6 bits, and the standard deviation of the */
/* 5 x 5 the sharpening adds nothing under and all over, as variance * 625 */
#define MDNS_SHARP_RISE     0.0334
#define MDNS_SHARP_DIP      0.0839
#define MDNS_SHARP_FLAT     3906            /* 2.5 levels */
#define MDNS_SHARP_FULL     10000           /* 4 */

/*
 * The loops of the filters are built to be vectorised, which -O2 does not
 * do for a length known only at run time, and where the host has AVX2 also
 * built for it, picked when a frame is filtered (the integers come out the
 * same either way). clang vectorises at -O2 and has no optimize attribute.
 */
#ifdef __clang__
#define MDNS_OPT
#else
#define MDNS_OPT            __attribute__((optimize("O3")))
#endif

#define MDNS_K(n)           isp_mdns_##n##_dflt
#define MDNS_ATTR           MDNS_OPT
#include "ingenic-isp-mdns.c.inc"
#undef MDNS_K
#undef MDNS_ATTR

#ifdef CONFIG_AVX2_OPT
#define MDNS_K(n)           isp_mdns_##n##_avx2
#define MDNS_ATTR           MDNS_OPT __attribute__((target("avx2")))
#include "ingenic-isp-mdns.c.inc"
#undef MDNS_K
#undef MDNS_ATTR
#endif

static void isp_mdns_run(void *arg, unsigned i, unsigned n)
{
    const IspMdnsJob *p = arg;
    IspMdnsJob band = *p;

    band.y0 = p->h * i / n;
    band.y1 = p->h * (i + 1) / n;
    band.band(&band);
}

/* @p over all the rows, in bands as the frame's size allows */
static void isp_mdns_bands(IngenicIspState *s, const IspMdnsJob *p)
{
    isp_par(s, (size_t)p->w * p->h < MDNS_BAND_PIXELS ? 1 : isp_nbands(s),
            isp_mdns_run, (void *)p);
}

/*
 * The fraction of the way to the mean a sample goes, by the distance to it,
 * in 1/256ths: p->kk[d] holds the frame's in its low half and the
 * reference's in its high one, which the blend's weight at the sample, of
 * 256, mixes
 */
static void isp_mdns_fractions(IspMdnsJob *p, double kc, double amp,
                               double reach)
{
    for (unsigned d = 0; d < 256; d++) {
        int kr = lround(256 * MIN(1.0, isp_mdns_spa_curve(d / reach) * amp));
        int kf = lround(256 * kc * isp_mdns_spa_curve(d) / 0.97);

        p->kk[d] = kf | kr << 16;
    }
}

/*
 * Y's pass-on picture, @pic (w x h), smoothed and sharpened into @out;
 * @wmap: the blend's weights. False: the picture goes on as it is
 */
static bool isp_mdns_filter(IngenicIspState *s, const IngenicIspMdns *m,
                            const uint8_t *pic, uint8_t *out,
                            const uint8_t *wmap, uint32_t w, uint32_t h,
                            const uint8_t *age, uint32_t bw, uint32_t bh,
                            uint32_t bsz, uint32_t gx, uint32_t gy)
{
    uint32_t fw = MDNS(m->fiir + 12);
    double x = (extract32(fw, 0, 8) + extract32(fw, 8, 8) +
                extract32(fw, 16, 8) + extract32(fw, 24, 8)) / 4.0;
    double wc = extract32(MDNS(m->spa), 0, 8) / 255.0;
    double kc = 0.14 + 0.86 * (1 - pow(1 - wc, 1.5));
    double amp = (0.94 + 0.00055 * x) / 0.97;
    double reach = (0.875 + 0.00294 * x) / 1.022;
    IspMdnsJob p = { .w = w, .h = h, .src = pic, .dst = out, .wmap = wmap,
                     .age = age, .bw = bw, .bh = bh, .bsz = bsz, .gy = gy };
    uint16_t *xb = isp_mdns_work(s, 13, (size_t)w * 2);

    if ((!MDNS(m->spa) && !fw) || w < 5 || h < 5) {
        return false;
    }
    isp_mdns_fractions(&p, kc, amp, reach);
    p.band = isp_mdns_band_dflt;
#ifdef CONFIG_AVX2_OPT
    if (s->mdns_avx2 && (cpuinfo & CPUINFO_AVX2)) {
        p.band = isp_mdns_band_avx2;
    }
#endif

    for (unsigned a = 0; m->sharp && a < 16; a++) {
        unsigned up = extract32(R(m->sharp + 0x48 + a / 4 * 4), a % 4 * 8, 6);
        unsigned dn = extract32(R(m->sharp + 0x58 + a / 4 * 4), a % 4 * 8, 6);

        p.gr[a] = lround(256 * MDNS_SHARP_RISE * up);
        p.gd[a] = lround(256 * MDNS_SHARP_DIP * dn);
        p.sharp |= up || dn;
    }
    if (p.sharp && bw && bh) {
        for (uint32_t c = 0; c < w; c++) {
            xb[c] = c < gx ? 0 : MIN((c - gx) / bsz, bw - 1);
        }
        p.xb = xb;
    } else {
        p.sharp = false;
    }
    isp_mdns_bands(s, &p);
    return true;
}

/*
 * The spatial filter and the sharpening on the new reference in
 * s->mdns_pic. A still scene gives the same reference, weights and ages
 * frame after frame, which costs the filters a good deal: it takes the
 * last result again then. What the filters took in is kept by the buffers
 * changing places with those it came in.
 */
static void isp_mdns_post(IngenicIspState *s, const IngenicIspMdns *m,
                          uint32_t w, uint32_t h, const uint8_t *age,
                          uint32_t bw, uint32_t bh, uint32_t bsz, uint32_t gx,
                          uint32_t gy)
{
    size_t n = (size_t)w * h;
    uint8_t *wmap = s->mdns_work[11];
    uint8_t *in = isp_mdns_work(s, 15, n * 3 / 2);
    uint8_t *wm = isp_mdns_work(s, 16, n);
    uint8_t *out = isp_mdns_work(s, 17, n);
    uint8_t *pic;
    size_t len;
    uint64_t key = 0xcbf29ce484222325ull;
    uint32_t v[] = { MDNS(m->spa), MDNS(m->fiir + 12), w, h, bsz, gx, gy, bw,
                     bh };

    for (unsigned i = 0; i < ARRAY_SIZE(v); i++) {
        key = (key ^ v[i]) * 0x100000001b3ull;
    }
    for (unsigned i = 0; m->sharp && i < 8; i++) {
        key = (key ^ R(m->sharp + 0x48 + i * 4)) * 0x100000001b3ull;
    }
    for (size_t i = 0; i < (size_t)bw * bh; i++) {
        key = (key ^ age[i]) * 0x100000001b3ull;
    }
    if (s->mdns_post_ok && key == s->mdns_post_key &&
        !memcmp(s->mdns_pic, in, n) && !memcmp(wmap, wm, n)) {
        memcpy(s->mdns_pic, out, n);
        return;
    }
    if (!isp_mdns_filter(s, m, s->mdns_pic, out, wmap, w, h, age, bw, bh,
                         bsz, gx, gy)) {
        return;
    }
    /* the weights and the picture the filters took in stay where they lie */
    wm = s->mdns_work[16];
    s->mdns_work[16] = s->mdns_work[11];
    s->mdns_work[11] = wm;
    len = s->mdns_work_len[16];
    s->mdns_work_len[16] = s->mdns_work_len[11];
    s->mdns_work_len[11] = len;
    pic = s->mdns_pic;
    s->mdns_pic = s->mdns_work[15];
    s->mdns_work[15] = pic;
    len = s->mdns_pic_len;
    s->mdns_pic_len = s->mdns_work_len[15];
    s->mdns_work_len[15] = len;
    memcpy(s->mdns_pic, out, n);
    memcpy(s->mdns_pic + n, pic + n, n / 2);
    s->mdns_post_key = key;
    s->mdns_post_ok = true;
}

/*
 * T21: each frame takes the next of 16 slots (+0x8 bytes each) of the
 * second ring (+0x40), a header row (the frame's count in its first word, 0
 * in the second) then a 16-bit word per block, its age in bits 3:0
 */
static hwaddr isp_mdns_slot(IngenicIspState *s, const IngenicIspMdns *m,
                            unsigned k, uint32_t count)
{
    return MDNS(m->sta[k]) + (hwaddr)((count - 1) % 16) * MDNS(m->sta[k] + 8);
}

static void isp_mdns_ring_ages(IngenicIspState *s, const IngenicIspMdns *m,
                               uint8_t *age, uint32_t bw, uint32_t bh)
{
    uint32_t stride = MDNS(m->sta[1] + 4);
    g_autofree uint16_t *row = NULL;

    if (!s->mdns_count || !MDNS(m->sta[1]) || stride < bw * 2) {
        return;
    }
    row = g_new(uint16_t, bw);
    for (uint32_t by = 0; by < bh; by++) {
        address_space_read(&address_space_memory,
                           isp_mdns_slot(s, m, 1, s->mdns_count) +
                           (hwaddr)(by + 1) * stride,
                           MEMTXATTRS_UNSPECIFIED, row, bw * 2);
        for (uint32_t bx = 0; bx < bw; bx++) {
            age[by * bw + bx] = le16_to_cpu(row[bx]) & 15;
        }
    }
}

static void isp_mdns_rings(IngenicIspState *s, const IngenicIspMdns *m,
                           const uint8_t *age, uint32_t bw, uint32_t bh)
{
    uint32_t count = ++s->mdns_count;
    uint32_t stride = MDNS(m->sta[1] + 4);
    g_autofree uint8_t *buf = NULL;

    if (!MDNS(m->sta[1]) || stride < MAX(bw * 2, 8)) {
        return;
    }
    buf = g_malloc0((size_t)stride * (bh + 1));
    stl_le_p(buf, count);
    stl_le_p(buf + 4, 0);
    for (uint32_t by = 0; by < bh; by++) {
        for (uint32_t bx = 0; bx < bw; bx++) {
            stw_le_p(buf + (by + 1) * stride + bx * 2, age[by * bw + bx]);
        }
    }
    /* the header's row past its two words stays as it was */
    address_space_write(&address_space_memory, isp_mdns_slot(s, m, 1, count),
                        MEMTXATTRS_UNSPECIFIED, buf, 8);
    address_space_write(&address_space_memory,
                        isp_mdns_slot(s, m, 1, count) + stride,
                        MEMTXATTRS_UNSPECIFIED, buf + stride,
                        (size_t)stride * bh);
}

/*
 * A row of ages: two to a byte (the left block in bits 3:0), or 5-bit
 * fields, twelve to a little-endian 8-byte group
 */
static uint32_t isp_mdns_row_bytes(const IngenicIspMdns *m, uint32_t n)
{
    return m->sta5 ? DIV_ROUND_UP(n, 12) * 8 : DIV_ROUND_UP(n, 2);
}

static unsigned isp_mdns_age_get(const IngenicIspMdns *m, const uint8_t *row,
                                 uint32_t bx)
{
    if (m->sta5) {
        return extract64(ldq_le_p(row + bx / 12 * 8), bx % 12 * 5, 5);
    }
    return row[bx / 2] >> (bx & 1) * 4 & 15;
}

static void isp_mdns_age_put(const IngenicIspMdns *m, uint8_t *row,
                             uint32_t bx, unsigned age)
{
    if (m->sta5) {
        stq_le_p(row + bx / 12 * 8,
                 deposit64(ldq_le_p(row + bx / 12 * 8), bx % 12 * 5, 5, age));
    } else {
        row[bx / 2] |= age << (bx & 1) * 4;
    }
}

/*
 * A coded reference is not the model's to read: its luma @m->ref_delay
 * frames back stands in for it, the frame itself while there are not as
 * many
 */
static const uint8_t *isp_mdns_back(IngenicIspState *s,
                                    const IngenicIspMdns *m, uint32_t w,
                                    uint32_t h, int input,
                                    const uint8_t *cur)
{
    if (s->mdns_w != w || s->mdns_h != h || s->mdns_hist_input != input) {
        s->mdns_hist = 0;
    }
    return s->mdns_hist >= m->ref_delay ? s->mdns_work[6 + m->ref_delay]
                                        : cur;
}

/* This frame becomes the one before; the oldest's buffer is the next's */
static void isp_mdns_keep(IngenicIspState *s, uint32_t w, uint32_t h,
                          int input)
{
    void *oldest;
    size_t len;

    isp_mdns_work(s, 8, (size_t)w * h * 3 / 2);
    oldest = s->mdns_work[8];
    len = s->mdns_work_len[8];
    s->mdns_work[8] = s->mdns_work[7];
    s->mdns_work_len[8] = s->mdns_work_len[7];
    s->mdns_work[7] = s->mdns_work[4];
    s->mdns_work_len[7] = s->mdns_work_len[4];
    s->mdns_work[4] = oldest;
    s->mdns_work_len[4] = len;
    s->mdns_hist = MIN(s->mdns_hist + 1, 2);
    s->mdns_hist_input = input;
}

/* A frame's sums over its blocks: each band takes whole rows of blocks */
typedef struct IspMdnsBlocks {
    const uint8_t *cur, *ref;
    uint32_t w, h, gx, gy, bsz, bw, bh;
    int32_t *bsum, *babs;
} IspMdnsBlocks;

static void isp_mdns_blocks_band(void *arg, unsigned i, unsigned n)
{
    const IspMdnsBlocks *b = arg;
    uint32_t y1 = MIN(b->gy + b->bh * (i + 1) / n * b->bsz, b->h);

    for (uint32_t y = b->gy + b->bh * i / n * b->bsz; y < y1; y++) {
        const uint8_t *c = b->cur + (size_t)y * b->w;
        const uint8_t *r = b->ref + (size_t)y * b->w;
        int32_t *bs = b->bsum + (y - b->gy) / b->bsz * b->bw;
        int32_t *ba = b->babs + (y - b->gy) / b->bsz * b->bw;

        ISP_KERNEL(mdns_blocks, c, r, b->gx, b->w, b->bsz, b->bw, bs, ba);
    }
}

/* This frame through the MDNS, kept for the channels to scale from */
static void isp_mdns_frame(IngenicIspState *s, ImageSensorState *sensor,
                           uint32_t frame)
{
    const IngenicIspMdns *m = isp_layout(s)->mdns;
    g_autofree uint8_t *age = NULL, *st = NULL;
    const uint8_t *ref, *ref_uv, *cur;
    void *map_y = NULL, *map_uv = NULL;
    hwaddr len_y, len_uv;
    g_autofree int32_t *bsum = NULL, *babs = NULL;
    uint32_t w, h, bw, bh, fun, ys, us, amax, thr, stride;
    uint32_t bsz = 16, gx = 0, gy = 0;
    uint8_t wt[16], wt_uv[16];
    size_t n;
    bool through;

    s->mdns_input = -1;
    if (!m) {
        return;
    }
    isp_frame_size(s, &w, &h);
    fun = m->fun ? MDNS(m->fun) : 1;
    ys = MDNS(m->y + 4);
    us = MDNS(m->uv + 4);
    if (!w || !h || !(fun & BIT(0)) ||
        (m->coded ? !MDNS(m->sta[0])
                  : !MDNS(m->y) || !MDNS(m->uv) || ys < w || us < w)) {
        return;
    }
    n = (size_t)w * h;
    /* a coded reference's history keeps the frame's buffer: it has its own */
    cur = m->coded ? NULL : isp_pic_ref(s, sensor, frame, w, h);
    if (!cur) {
        uint8_t *buf = isp_mdns_work(s, 4, n * 3 / 2);

        if (isp_pic_plain(s)) {
            isp_pic_draw(s, sensor, frame, w, h, buf, true);
        } else {
            isp_scene_draw(s, sensor, 0, frame, w, h, buf, buf + n);
            isp_wb_gains(s, sensor, 0, w, h, buf, buf + n);
        }
        cur = buf;
    }
    if (m->coded) {
        ref = isp_mdns_back(s, m, w, h, sensor->input, cur);
        ref_uv = NULL;
    } else {
        ref = isp_mdns_ref(MDNS(m->y), ys, isp_mdns_work(s, 5, n), w, h,
                           &map_y, &len_y);
        ref_uv = isp_mdns_ref(MDNS(m->uv), us, isp_mdns_work(s, 6, n / 2), w,
                              h / 2, &map_uv, &len_uv);
    }

    /* the blocks' ages, last frame's from the first statistics buffer */
    if (m->bsize && extract32(MDNS(m->bsize), 0, 8) >= 4) {
        bsz = extract32(MDNS(m->bsize), 0, 8);
        gx = extract32(MDNS(m->bsize + 4), 0, 8);
        gy = extract32(MDNS(m->bsize + 4), 8, 8);
    }
    bw = DIV_ROUND_UP(w - MIN(gx, w), bsz);
    bh = DIV_ROUND_UP(h - MIN(gy, h), bsz);
    if (m->blocks && MDNS(m->blocks)) {
        bw = MIN(extract32(MDNS(m->blocks), 0, 16), bw);
        bh = MIN(extract32(MDNS(m->blocks), 16, 16), bh);
    }
    age = g_malloc0(bw * bh);
    stride = MDNS(m->sta[0] + 4);
    if (m->rings) {
        isp_mdns_ring_ages(s, m, age, bw, bh);
    } else if (MDNS(m->sta[0]) && stride >= isp_mdns_row_bytes(m, bw)) {
        g_autofree uint8_t *row = g_malloc(stride);
        uint32_t top = !!m->grid;

        for (uint32_t by = 0; by < bh; by++) {
            address_space_read(&address_space_memory,
                               MDNS(m->sta[0]) + (hwaddr)(by + top) * stride,
                               MEMTXATTRS_UNSPECIFIED, row, stride);
            for (uint32_t bx = 0; bx < bw; bx++) {
                age[by * bw + bx] = isp_mdns_age_get(m, row, bx);
            }
        }
    }
    amax = m->age_max ? extract32(MDNS(m->age_max), m->rings ? 0 : 4, 4)
                      : m->age_cap ?: 7;
    thr = m->blk_thr ? extract32(MDNS(m->blk_thr), 16, 8) : 5;
    bsum = g_new0(int32_t, bw * bh);
    babs = g_new0(int32_t, bw * bh);
    isp_par(s, n < MDNS_BAND_PIXELS ? 1 : isp_nbands(s), isp_mdns_blocks_band,
            &(IspMdnsBlocks){ cur, ref, w, h, gx, gy, bsz, bw, bh, bsum,
                              babs });
    for (uint32_t by = 0; by < bh; by++) {
        for (uint32_t bx = 0; bx < bw; bx++) {
            uint32_t px = (MIN(gx + bx * bsz + bsz, w) -
                           MIN(gx + bx * bsz, w)) *
                          (MIN(gy + by * bsz + bsz, h) -
                           MIN(gy + by * bsz, h));
            uint8_t *ag = &age[by * bw + bx];

            *ag = abs(bsum[by * bw + bx]) > (int)(thr * px) ? 0
                                                          : MIN(*ag + 1, amax);
        }
    }

    /* a coded reference has no weights: the frame goes out as it came in */
    through = m->coded && !m->wt;
    if (!through && s->mdns_pic_len < n * 3 / 2) {
        g_free(s->mdns_pic);
        s->mdns_pic = g_malloc(n * 3 / 2);
        s->mdns_pic_len = n * 3 / 2;
    }
    if (!through && (!m->wt || !(fun & BIT(12)))) {
        isp_copy(s, s->mdns_pic, cur, n);
    }
    if (!through && (!m->wt || !(fun & BIT(28)))) {
        isp_copy(s, s->mdns_pic + n, cur + n, n / 2);
    }
    if (m->wt) {
        uint8_t *wmap = isp_mdns_work(s, 11, n);
        uint8_t *uvmap = isp_mdns_work(s, 13, n / 2);
        uint32_t lim_y = m->wt_y ? MDNS(m->wt_y) : 0xff0000;
        uint32_t lim = m->wt_uv ? MDNS(m->wt_uv) : 0xff0000;

        for (unsigned i = 0; i < 16; i++) {
            uint8_t a = extract32(MDNS(m->wt + i / 4 * 4), i % 4 * 8, 8);

            wt[i] = i ? MIN(MAX(a, extract32(lim_y, 24, 8)),
                            extract32(lim_y, 16, 8)) : a;
            wt_uv[i] = i ? MIN(MAX(a, extract32(lim, 24, 8)),
                               extract32(lim, 16, 8)) : a;
        }
        if (fun & BIT(12)) {
            isp_mdns_blend(s, cur, ref, s->mdns_pic, w, h, 1, 4, 4, age, bw,
                           wt, extract32(MDNS(m->gate_y), 3, 9), wmap);
        }
        if (fun & BIT(28)) {
            isp_mdns_blend(s, cur + n, ref_uv, s->mdns_pic + n, w, h / 2, 2,
                           3, 3, age, bw, wt_uv,
                           extract32(MDNS(m->gate_uv), 3, 9), uvmap);
        }
    }
    if (map_y) {
        address_space_unmap(&address_space_memory, map_y, len_y, false, 0);
    }
    if (map_uv) {
        address_space_unmap(&address_space_memory, map_uv, len_uv, false, 0);
    }
    if (!m->coded) {
        isp_dma_plane(s, MDNS(m->y), s->mdns_pic, w, 0, 0, w, h, ys);
        isp_dma_plane(s, MDNS(m->uv), s->mdns_pic + n, w, 0, 0, w, h / 2, us);
    }
    if (m->spa && m->wt && (fun & BIT(12))) {
        isp_mdns_post(s, m, w, h, age, bw, bh, bsz, gx, gy);
    }

    if (m->rings) {
        isp_mdns_rings(s, m, age, bw, bh);
    }
    /* the statistics, the first buffer last: it wins a shared one */
    for (int k = m->rings ? -1 : (int)ARRAY_SIZE(m->sta) - 1; k >= 0; k--) {
        uint32_t grid = m->grid ? MDNS(m->grid + k * 0x18) : 0;
        uint32_t cols = MIN(extract32(grid, 24, 8) ?: bw, bw);
        uint32_t rows = MIN(extract32(grid, 16, 8) ?: bh, bh);
        uint32_t top = m->grid && !k, len;

        stride = MDNS(m->sta[k] + 4);
        if (!m->sta[k] || !MDNS(m->sta[k]) ||
            stride < isp_mdns_row_bytes(m, cols)) {
            continue;
        }
        if (!m->grid) {
            /* a row in whole 8-byte groups: what lies past stays as it was */
            len = MIN(stride, ROUND_UP(isp_mdns_row_bytes(m, cols), 8));
            st = g_realloc(st, len);
            for (uint32_t by = 0; by < rows; by++) {
                memset(st, 0, len);
                for (uint32_t bx = 0; bx < cols; bx++) {
                    isp_mdns_age_put(m, st, bx, age[by * bw + bx]);
                }
                address_space_write(&address_space_memory,
                                    MDNS(m->sta[k]) + (hwaddr)by * stride,
                                    MEMTXATTRS_UNSPECIFIED, st, len);
            }
            continue;
        }
        len = stride * (bh + 1);
        st = g_realloc(st, len);
        memset(st, 0, len);
        for (uint32_t by = 0; by < rows; by++) {
            for (uint32_t bx = 0; bx < cols; bx++) {
                st[(by + top) * stride + bx / 2] |=
                    age[by * bw + bx] << (bx & 1) * 4;
            }
        }
        address_space_write(&address_space_memory, MDNS(m->sta[k]),
                            MEMTXATTRS_UNSPECIFIED, st, len);
    }
    if (m->coded) {
        isp_mdns_keep(s, w, h, sensor->input);
    }
    s->mdns_out = through ? cur : s->mdns_pic;
    s->mdns_w = w;
    s->mdns_h = h;
    s->mdns_frame = frame;
    s->mdns_input = sensor->input;
}

static void isp_scene(IngenicIspState *s, unsigned pipe,
                      ImageSensorState *sensor, uint32_t frame, uint32_t w,
                      uint32_t h, uint8_t *luma, uint8_t *uv)
{
    if (s->mdns_out && s->mdns_input == sensor->input &&
        s->mdns_frame == frame) {
        size_t n = (size_t)w * h;

        if (w == s->mdns_w && h == s->mdns_h) {
            isp_copy(s, luma, s->mdns_out, n);
            isp_copy(s, uv, s->mdns_out + n, n / 2);
        } else {
            isp_scale(s, s->mdns_out, s->mdns_w, s->mdns_h, w, h, luma, uv);
        }
        return;
    }
    if (!pipe && uv == luma + (size_t)w * h && isp_pic_plain(s)) {
        isp_pic_draw(s, sensor, frame, w, h, luma, false);
        return;
    }
    isp_scene_draw(s, sensor, pipe, frame, w, h, luma, uv);
    isp_wb_gains(s, sensor, pipe, w, h, luma, uv);
}

void ingenic_isp_ivdc_coding(IngenicIspState *s, bool on)
{
    s->ivdc_coding = on;
}

static void isp_render(IngenicIspState *s, unsigned input, uint32_t seq,
                       uint32_t w, uint32_t h, uint8_t *nv12)
{
    ImageSensorState *sensor = image_sensor_find(input);
    uint32_t sw, sh, cx, cy, cw, ch_h;
    g_autofree uint8_t *pic = NULL;

    if (!sensor) {
        memset(nv12, 0, (size_t)w * h * 3 / 2);
        return;
    }
    if (!isp_chan_geometry(s, 0, 0, &sw, &sh, &cx, &cy, &cw, &ch_h)) {
        sw = cw = w;
        sh = ch_h = h;
        cx = cy = 0;
    }
    /* the whole frame, uncropped: drawn in place, its last row repeated */
    if (!cx && !cy && sw == w && cw == w && ch_h == sh && sh <= h) {
        uint8_t *uv = nv12 + (size_t)w * h;

        isp_scene(s, 0, sensor, seq, w, sh, nv12, uv);
        isp_overlay(s, 0, nv12, w, h, 0, 0, w, sh);
        for (uint32_t y = sh; y < h; y++) {
            memcpy(nv12 + (size_t)y * w, nv12 + (size_t)(sh - 1) * w, w);
        }
        for (uint32_t y = sh / 2; y < h / 2; y++) {
            memcpy(uv + (size_t)y * w, uv + (size_t)(sh / 2 - 1) * w, w);
        }
        return;
    }
    pic = g_malloc((size_t)sw * sh * 3 / 2);
    isp_scene(s, 0, sensor, seq, sw, sh, pic, pic + (size_t)sw * sh);
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
        if (w > cw && (w - cw) & 1) {
            dst[w - 1] = 0;
        }
    }
}

void ingenic_isp_render(IngenicIspState *s, uint32_t w, uint32_t h,
                        uint8_t *nv12)
{
    /* waits for a frame in a pass: only IVDC calls this, which no T31 has */
    isp_quiesce(s);
    isp_render(s, s->ivdc_input, s->vic[0].frame, w, h, nv12);
}

void ingenic_isp_render_vpu(IngenicIspState *s, unsigned vpu, uint32_t w,
                            uint32_t h, uint8_t *nv12)
{
    uint32_t n;

    isp_quiesce(s);
    n = s->ivdc_taken[vpu];
    if (n >= INGENIC_ISP_SENSORS) {
        ingenic_isp_render(s, w, h, nv12);
        return;
    }
    isp_render(s, s->ivdc_sent[n], s->ivdc_frame_no[n], w, h, nv12);
}

/*
 * A frame crop window narrower than the frame the ISP takes: false if there
 * is none, or it does not lie in the frame
 */
static bool isp_fcrop(IngenicIspState *s, uint32_t *x, uint32_t *y,
                      uint32_t *w, uint32_t *h)
{
    hwaddr reg = isp_layout(s)->fcrop;
    uint32_t fw, fh;

    if (!reg) {
        return false;
    }
    isp_frame_size(s, &fw, &fh);
    *x = (R(reg) >> 16) & ~1;
    *y = (R(reg) & 0xffff) & ~1;
    *w = (R(reg + 4) >> 16) & ~1;
    *h = (R(reg + 4) & 0xffff) & ~1;
    return *w && *h && *x + *w <= fw && *y + *h <= fh &&
           (*w < fw || *h < fh);
}

/*
 * Channel @n's input window: its register's, where the part has one for
 * each channel (an AutoZoom), else the frame crop's. False if it is none, or
 * does not lie in the frame, or is not smaller than it.
 */
static bool isp_chan_win(IngenicIspState *s, unsigned n, uint32_t *x,
                         uint32_t *y, uint32_t *w, uint32_t *h)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr reg = l->chwin + n * 8;
    uint32_t fw, fh;

    if (!l->chwin) {
        return isp_fcrop(s, x, y, w, h);
    }
    isp_frame_size(s, &fw, &fh);
    if (l->chratio) {
        /* the ratios are the window over the scaled size, in 1/16384 */
        uint32_t size = R(l->pipe[0].ch + n * 0x100 + l->chan->size);

        *x = ((R(reg) >> 16) & 0xfff) & ~1;
        *y = (R(reg) & 0x1fff) & ~1;
        *w = (((uint64_t)R(l->chratio + n * 4) * (size >> 16) + 8192) >> 14)
             & ~1;
        *h = (((uint64_t)R(l->chratio - 12 + n * 4) * (size & 0xffff) + 8192)
              >> 14) & ~1;
    } else {
        *x = (R(reg) >> 16) & ~1;
        *y = (R(reg) & 0xffff) & ~1;
        *w = (R(reg + 4) >> 16) & ~1;
        *h = (R(reg + 4) & 0xffff) & ~1;
    }
    return *w && *h && *x + *w <= fw && *y + *h <= fh &&
           (*w < fw || *h < fh);
}

/*
 * The scene as a channel scaled to @w x @h puts it out: the frame the ISP
 * takes, or with a front crop its window of that, enlarged by nearest pixels
 */
static void isp_chan_scene(IngenicIspState *s, unsigned p, unsigned n,
                           ImageSensorState *sensor, uint32_t frame,
                           uint32_t w, uint32_t h, uint8_t *luma, uint8_t *uv)
{
    g_autofree uint8_t *pic = NULL;
    uint32_t x0, y0, cw, ch_h, fw, fh;

    if (p || !isp_chan_win(s, n, &x0, &y0, &cw, &ch_h)) {
        isp_scene(s, p, sensor, frame, w, h, luma, uv);
        return;
    }
    isp_frame_size(s, &fw, &fh);
    pic = g_malloc((size_t)fw * fh * 3 / 2);
    isp_scene(s, p, sensor, frame, fw, fh, pic, pic + (size_t)fw * fh);
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *src = pic + (size_t)(y0 + y * ch_h / h) * fw + x0;

        for (uint32_t x = 0; x < w; x++) {
            luma[(size_t)y * w + x] = src[x * cw / w];
        }
    }
    for (uint32_t y = 0; y < h / 2; y++) {
        const uint8_t *src = pic + (size_t)fw * fh +
                             (size_t)(y0 / 2 + y * (ch_h / 2) / (h / 2)) * fw +
                             x0;

        for (uint32_t x = 0; x < w / 2; x++) {
            uint32_t sx = x * (cw / 2) / (w / 2) * 2;

            uv[(size_t)y * w + x * 2] = src[sx];
            uv[(size_t)y * w + x * 2 + 1] = src[sx + 1];
        }
    }
}

/* A clock in kHz: the guest's divider at @cdr, else the part's default */
static uint32_t isp_clock_khz(IngenicIspState *s, hwaddr cdr)
{
    const IngenicIspLayout *l = isp_layout(s);
    uint64_t hz;

    for (unsigned i = 0; s->khz_ok && i < 2; i++) {
        if (s->khz_cdr[i] == cdr) {
            return s->khz[i];
        }
    }
    hz = s->cpm ? ingenic_cgu_hz(s->cpm, cdr) : 0;

    return hz ? hz / 1000 : l->loads->default_khz;
}

/* The limit at @khz: straight between the rows, scaled off the ends */
static uint32_t isp_load_limit(const IngenicIspLoads *ld, uint32_t khz,
                               bool window)
{
    if (!ld->n) {
        return 0;
    }
    for (unsigned i = 0; i < ld->n; i++) {
        const IngenicIspLoad *r = &ld->rows[i];
        uint32_t lim = window ? r->window : r->scaler;

        if (khz <= r->khz) {
            const IngenicIspLoad *q = i ? &ld->rows[i - 1] : NULL;
            uint32_t lo = q ? (window ? q->window : q->scaler) : 0;
            uint32_t lo_khz = q ? q->khz : 0;

            return lo + (uint64_t)(lim - lo) * (khz - lo_khz) /
                        (r->khz - lo_khz);
        }
    }
    return (uint64_t)(window ? ld->rows[ld->n - 1].window :
                      ld->rows[ld->n - 1].scaler) * khz /
           ld->rows[ld->n - 1].khz;
}

/*
 * Whether the MSCA stalls on the frame: false if it keeps up, else why. A
 * T31's front crop window smaller than channel 0's output stalls it when
 * its scaler is off, which the channel enable's bit 8 says (a T31X reads
 * 0x903 with channel 0 unscaled and channel 1 scaled, 0x3 with both scaled;
 * bit 11 comes with it; only channel 0's was seen unscaled), as the sample
 * that enlarges a window under an unscaled 1080p channel finds; and on any
 * part a channel whose line load is past the limit, for the window the
 * channel takes (its own on the T40 and T41, the front crop's elsewhere) at
 * the MSCA's clock. A limit of 0 is one not measured: no stall.
 */
static const char *isp_overload(IngenicIspState *s, ImageSensorState *sensor,
                                unsigned p)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspLoads *ld = l->loads;
    uint32_t x, y, fw, fh, chans = R(l->pipe[p].ch_en);
    uint32_t rate = image_sensor_line_rate(sensor, l->raw_part), khz;

    if (!ld || p) {
        return NULL;
    }
    isp_frame_size(s, &fw, &fh);
    if (!fh) {
        return NULL;
    }
    khz = isp_clock_khz(s, l->scdr ?: l->ispcdr);
    for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
        hwaddr base = l->pipe[p].ch + n * 0x100;
        uint32_t size = R(base + l->chan->size);
        uint32_t lim, wx, wy;
        bool fc;
        uint64_t load;

        if (!(chans & BIT(n)) || !size) {
            continue;
        }
        fc = isp_chan_win(s, n, &x, &y, &wx, &wy);
        lim = isp_load_limit(ld, khz, fc);
        if (l->unscaled_bit && fc && !n && (chans & l->unscaled_bit) &&
            (wx < (size >> 16) || wy < (size & 0xffff))) {
            return "front crop under an unscaled channel";
        }
        load = (uint64_t)(size >> 16) * (size & 0xffff) / (fc ? wy : fh);
        if (lim && load * (rate ? rate : ld->line_rate) >
            (uint64_t)lim * ld->line_rate) {
            return fc ? "front crop enlarges too far" : "scaler too large";
        }
    }
    return NULL;
}

/*
 * Whether the core's input keeps up with the frame: false for one it
 * drops. The load is the width of the frame the core takes times the sensor's
 * line rate over the clock (the frame rate does not enter); the frames that
 * pass at it come from the part's table.
 */
static bool isp_core_keeps(IngenicIspState *s, ImageSensorState *sensor)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspLoads *ld = l->loads;
    uint32_t rate = image_sensor_line_rate(sensor, l->raw_part), fw, fh;
    uint64_t khz;

    if (!ld || !ld->keeps || !rate) {
        return true;
    }
    isp_frame_size(s, &fw, &fh);
    khz = isp_clock_khz(s, l->ispcdr);
    s->keep_acc += ingenic_t20_isp_keep_pm(ld->keep, ld->keeps,
                                           (uint64_t)fw * rate / khz);
    if (s->keep_acc < 1000) {
        return false;
    }
    s->keep_acc -= 1000;
    return true;
}

/* Whether the frame tap takes channel @n of pipeline @p's @w x @h picture */
static bool isp_tap_want(IngenicIspState *s, unsigned p, unsigned n,
                         uint32_t w, uint32_t h)
{
    g_autofree char *name = NULL;

    if (!ingenic_frame_tap_on(&s->tap)) {
        return false;
    }
    name = g_strdup_printf("isp-%u-%u", p, n);
    return ingenic_frame_tap_ready(&s->tap, name, w, h, NULL);
}

/* The picture channel @n of pipeline @p puts out, to the frame tap */
static void isp_tap_chan(IngenicIspState *s, unsigned p, unsigned n,
                         ImageSensorState *sensor, const uint8_t *nv12,
                         uint32_t w, uint32_t h, uint32_t cx, uint32_t cy,
                         uint32_t cw, uint32_t ch_h)
{
    g_autofree char *name = NULL;

    if (!ingenic_frame_tap_on(&s->tap)) {
        return;
    }
    name = g_strdup_printf("isp-%u-%u", p, n);
    ingenic_frame_tap_nv12(&s->tap, name, sensor->fps, nv12, w, h, cx, cy,
                           cw, ch_h);
}

/* Channel @n of pipeline @p takes a frame: false if it drops it */
static bool isp_chan_frame(IngenicIspState *s, ImageSensorState *sensor,
                           uint32_t frame, unsigned p, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspChanRegs *r = l->chan;
    hwaddr base = l->pipe[p].ch + n * 0x100;
    IngenicIspChan *ch = &s->chan[p][n];
    g_autofree uint8_t *nv12 = NULL;
    uint32_t w, h, cx, cy, cw, ch_h;
    uint32_t y, uv, i;

    if (!isp_chan_geometry(s, p, n, &w, &h, &cx, &cy, &cw, &ch_h)) {
        trace_ingenic_isp_chan_drop(p, n, frame, "no size");
        return false;
    }
    /*
     * The T32 Pro's driver, with several sensors, sets its channels up
     * through register lists and never reads their done entries
     */
    if (l->done_unread && ch->done_n) {
        ch->done_rd = (ch->done_rd + 1) % INGENIC_ISP_FIFO;
        ch->done_n--;
    }
    /*
     * A T31 takes the buffers queued as the frame reached the MSCA (a T31X's
     * counter, held with a buffer pushed mid-frame, ran from line 0 at the
     * next frame): one pushed since waits for the next
     */
    if (!ch->y_n || !ch->uv_n || ch->done_n ||
        (l->push_latch && !p && !n && s->msca_latched[0] &&
         !s->ch0_latched)) {
        trace_ingenic_isp_chan_drop(p, n, frame, ch->done_n ?
                                    "the last not read" :
                                    ch->y_n && ch->uv_n ? "buffer too late" :
                                    "no buffer");
        if (!p && !n) {
            s->ch0_from = 0;
        }
        /*
         * The T32 Pro's driver, with several sensors, waits for the done of
         * every channel it enabled, a buffer or not, before the next frame
         */
        if (l->drop_done) {
            isp_raise(s, l->pipe[p].done_sta ?: l->pipe[p].int_sta, BIT(n));
        }
        return false;
    }
    y = isp_pop(ch->y, &ch->y_rd, &ch->y_n);
    uv = isp_pop(ch->uv, &ch->uv_rd, &ch->uv_n);
    if (l->push_latch) {
        /* the push registers' bit 0 is a buffer pending */
        if (!ch->y_n) {
            R(base + r->y_push) &= ~1;
        }
        if (!ch->uv_n) {
            R(base + r->uv_push) &= ~1;
        }
    }
    if (y & ISP_IVDC_ADDR) {
        trace_ingenic_isp_ivdc(p, n, frame, extract32(y, 25, 2),
                               sensor->input);
        if (isp_tap_want(s, p, n, cw, ch_h)) {
            /* no frame is written, but the encoder takes this one */
            nv12 = g_malloc(w * h * 3 / 2);
            isp_chan_scene(s, p, n, sensor, frame, w, h, nv12, nv12 + w * h);
            if (p == 0) {
                isp_overlay(s, n, nv12, w, h, cx, cy, cw, ch_h);
            }
            isp_tap_chan(s, p, n, sensor, nv12, w, h, cx, cy, cw, ch_h);
            ingenic_frame_tap_done(&s->tap);
        }
        s->ivdc_input = sensor->input;
        s->ivdc_sent[extract32(y, 25, 2)] = sensor->input;
        s->ivdc_frame_no[extract32(y, 25, 2)] = frame;
        qemu_irq_pulse(s->ivdc_frame[extract32(y, 25, 2)]);
        if (l->ivdc_no_done) {
            isp_raise(s, l->pipe[p].done_sta ?: l->pipe[p].int_sta, BIT(n));
            return true;
        }
    } else {
        nv12 = g_malloc(w * h * 3 / 2);
        isp_chan_scene(s, p, n, sensor, frame, w, h, nv12, nv12 + w * h);
        if (p == 0) {
            isp_overlay(s, n, nv12, w, h, cx, cy, cw, ch_h);
        }
        isp_tap_chan(s, p, n, sensor, nv12, w, h, cx, cy, cw, ch_h);
        isp_dma_plane(s, y, nv12, w, cx, cy, cw, ch_h, R(base + r->y_stride));
        isp_dma_plane(s, uv, nv12 + w * h, w, cx, cy / 2, cw, ch_h / 2,
                      R(base + r->uv_stride));
        if (!p && !n) {
            s->ch0_from = 0;
            s->ch0_rest = l->ch0_out_holds ? ch_h << 16 : 0;
        }
    }
    trace_ingenic_isp_chan_done(p, n, frame, y, uv);
    i = (ch->done_rd + ch->done_n++) % INGENIC_ISP_FIFO;
    ch->done_y[i] = y;
    ch->done_uv[i] = uv;
    ch->done_frame[i] = frame;
    isp_raise(s, l->pipe[p].done_sta ?: l->pipe[p].int_sta, BIT(n));
    return true;
}

/* The masks' positions and sizes the enable's load bits ask for */
static void isp_mask_load(IngenicIspState *s, unsigned p)
{
    const IngenicIspOsd *o = isp_layout(s)->osd;

    for (unsigned c = 0; !p && o && o->mask_load && c < INGENIC_ISP_CHANS;
         c++) {
        if (!(R(o->mask_en) & BIT(o->mask_load + c))) {
            continue;
        }
        for (unsigned b = 0; b < OSD_MASKS; b++) {
            hwaddr m = o->mask + c * o->mask_chan + b * o->mask_step;

            R(m) = s->mask_next[c][b * 2];
            R(m + 4) = s->mask_next[c][b * 2 + 1];
        }
    }
}

/*
 * Whether channel @n of pipeline @p lets out the frame reaching the MSCA,
 * by its frame drop; the window moves on either way, and a window cut
 * short (length - 1 written below the frame it is at) ends at that frame
 */
static bool isp_drop_take(IngenicIspState *s, unsigned p, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr reg = l->pipe[p].ch + n * 0x100 + l->drop;
    uint8_t *pos = &s->drop_pos[p][n];
    bool out;

    if (!l->drop) {
        return true;
    }
    out = R(reg + 4) & BIT(*pos);
    *pos = *pos >= R(reg) ? 0 : *pos + 1;
    return out;
}

/*
 * The pipeline's next frame reaching its MSCA: it takes the channels
 * enabled then, sets their busy bits, and decides which drop it
 */
static void isp_msca_sync(IngenicIspState *s, unsigned p)
{
    const IngenicIspPipe *pipe = &isp_layout(s)->pipe[p];

    if (s->msca_busy_at[p] &&
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) >= s->msca_busy_at[p]) {
        s->msca_chans[p] = R(pipe->ch_en) &
                           MAKE_64BIT_MASK(0, INGENIC_ISP_CHANS);
        s->msca_latched[p] = true;
        s->drop_decided[p] = s->msca_chans[p];
        s->drop_skip[p] = 0;
        for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
            if ((s->msca_chans[p] & BIT(n)) && !isp_drop_take(s, p, n)) {
                s->drop_skip[p] |= BIT(n);
            }
        }
        if (pipe->busy) {
            R(pipe->busy) = s->msca_chans[p];
        }
        isp_mask_load(s, p);
        /* the last frame's done read by the frame's end is enough */
        s->ch0_latched = !p && (s->msca_chans[0] & ~s->drop_skip[0] & 1) &&
                         s->chan[0][0].y_n && s->chan[0][0].uv_n;
        if (s->ch0_latched && !s->chan[0][0].done_n) {
            s->ch0_from = s->msca_busy_at[p];
        }
        R(pipe->ch_en) &= ~isp_layout(s)->ch_en_pulse;
        s->msca_busy_at[p] = 0;
    }
}

#define STATS_GRID_MAX  15
#define STATS_ZONES     (STATS_GRID_MAX * STATS_GRID_MAX)
/*
 * The T33's and T32 Pro's AWB zones: their tx-isp lays them out 32 to a
 * row of its working arrays and gives each buffer 0x7400 bytes, 32 rows of
 * 28 bytes by 32 with room to spare. The model takes 32 x 32 as the limit;
 * no board has run a grid past 15 x 15, so that is unverified.
 */
#define FLAT_AWB_MAX    32
#define AWB_ZONES       (FLAT_AWB_MAX * FLAT_AWB_MAX)

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
                                IspStatsGrid *g, uint32_t qw, uint32_t qh,
                                unsigned max)
{
    uint32_t zw = extract32(R(zone), 0, 16), zh = extract32(R(zone), 16, 16);

    g->cols = zw ? MIN(qw / zw, max) : 0;
    g->rows = zh ? MIN(qh / zh, max) : 0;
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
    /* the AWB sums in 1/256ths until rounded, which a coarse grid's */
    /* zone can take past 32 bits */
    uint64_t awb[AWB_ZONES][4];     /* R, G, B sums; count */
    uint64_t awb_all[AWB_ZONES][4]; /* the same of all quads */
    uint64_t awb_band[AWB_ZONES][4][4];     /* T40, T41: by luma band */
    uint64_t awb_white[AWB_ZONES][4][4];    /* and their white quads */
    uint32_t hist[256];
} IspStatsSums;

/* An 8-bit gamma-2.2 value in linear light, 8-bit */
static uint8_t isp_linear[256];
/* The same in 1/256ths, which the AWB sums keep (see isp_awb_sums_round()) */
static uint32_t isp_linear_fine[256];

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

/* The AE block at @b's buffers, and its bits raised */
static void isp_ae_write(IngenicIspState *s, unsigned pipe, hwaddr b,
                         uint32_t raise, const IspStatsSums *sum,
                         const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    uint32_t idx = R(b + 0x50), counts = R(b + 0x4c);
    uint32_t zbuf = (extract32(idx, 4, 2) + 1) % (extract32(counts, 4, 2) + 1);
    uint32_t hbuf = (extract32(idx, 0, 2) + 1) % (extract32(counts, 0, 2) + 1);
    uint32_t tag = (extract32(idx, 8, 8) + 1) & 0xff;
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
    } else if (st->older) {
        for (unsigned i = 0; i < 256; i++) {
            hist[i] = cpu_to_le32(MIN(sum->hist[i], 0xfffff) |
                                  (i & 1 ? tag << 20 : 0));
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
    R(b + 0x50) = (st->older || st->tagged ? tag << 8 : 0) | zbuf << 4 | hbuf;
    isp_raise(s, isp_layout(s)->pipe[pipe].int_sta, raise);
}

/*
 * The T33's and T32 Pro's flat buffers: a 16-bit tag in 8 bytes at the
 * start, the data, the tag again; the drivers read nothing else of the
 * 8 bytes, which are left 0 (what the hardware puts there is unverified).
 * The sums are the T32's, 8-bit linear light: the AWB reader multiplies a
 * white zone's sum by a gain in 32 bits (tisp_t33_awb_sat_weight), which
 * fits a zone of 64 x 36 quads of 8-bit levels at the gains it runs with
 * and would not at 10 bits; also unverified.
 */
#define FLAT_HEAD       8
#define FLAT_AE_GRID    15      /* zones a row of the AE buffer holds */
#define FLAT_AE_ZONE    16
#define FLAT_AE_HIST    (FLAT_HEAD + 256 * 4)
#define FLAT_AE_ZONES   (FLAT_HEAD + FLAT_AE_GRID * FLAT_AE_GRID * FLAT_AE_ZONE)
#define FLAT_AWB_ZONE   28

/* The T33's and T32 Pro's: the histogram and the zones, each its buffer */
static void isp_ae_write_flat(IngenicIspState *s, unsigned pipe, hwaddr b,
                              uint32_t frame, const IspStatsSums *sum,
                              const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    uint32_t idx = R(b + 0x50), counts = R(b + 0x4c);
    uint32_t zbuf = (extract32(idx, 4, 2) + 1) % (extract32(counts, 4, 2) + 1);
    uint32_t hbuf = (extract32(idx, 0, 2) + 1) % (extract32(counts, 0, 2) + 1);
    uint8_t hist[FLAT_AE_HIST + 2] = { 0 };
    uint8_t zone[FLAT_AE_ZONES + 2] = { 0 };

    stw_le_p(hist, frame);
    for (unsigned i = 0; i < 256; i++) {
        stl_le_p(hist + FLAT_HEAD + i * 4, sum->hist[i]);
    }
    stw_le_p(hist + FLAT_AE_HIST, frame);
    stw_le_p(zone, frame);
    for (unsigned r = 0; r < g->rows; r++) {
        for (unsigned c = 0; c < g->cols; c++) {
            const uint32_t *v = sum->ae[r * g->cols + c];
            uint8_t *z = zone + FLAT_HEAD +
                         (r * FLAT_AE_GRID + c) * FLAT_AE_ZONE;

            for (unsigned f = 0; f < 3; f++) {
                stl_le_p(z + f * 4, v[f]);
            }
            stw_le_p(z + 12, MIN(v[3], 0xffff));
            stw_le_p(z + 14, MIN(v[4], 0xffff));
        }
    }
    stw_le_p(zone + FLAT_AE_ZONES, frame);
    isp_stats_dma(R(b + 0x2c + zbuf * 4), zone, sizeof(zone));
    isp_stats_dma(R(b + 0x3c + hbuf * 4), hist, sizeof(hist));
    R(b + 0x50) = zbuf << 4 | hbuf;
    isp_raise(s, isp_layout(s)->pipe[pipe].int_sta,
              st->int_ae_zones | st->int_ae_hist);
}

static void isp_awb_write(IngenicIspState *s, unsigned pipe,
                          const IspStatsSums *sum, const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb + pipe * st->sec;
    uint32_t buf = (R(b + 0x50) + 1) % ((R(b + 0x4c) & 3) + 1);
    uint32_t zone[STATS_ZONES * 4] = { 0 };

    for (unsigned z = 0; z < g->rows * g->cols; z++) {
        uint32_t r = MIN(sum->awb[z][0], 0x1fffff);
        uint32_t gr = MIN(sum->awb[z][1], 0x1fffff);
        uint32_t bl = MIN(sum->awb[z][2], 0x1fffff);
        uint32_t n = MIN(sum->awb[z][3], 0x1fff);

        if (st->older) {
            uint32_t tag = extract32(R(st->ae + 0x50) - BIT(8), 8, 8);

            isp_stats_put(zone + z * 4, 0, 21, r);
            isp_stats_put(zone + z * 4, 21, 21, gr);
            isp_stats_put(zone + z * 4, 42, 21, bl);
            isp_stats_put(zone + z * 4, 63, 13, n);
            isp_stats_put(zone + z * 4, 76, 8, tag);
            continue;
        }
        zone[z * 4] = r | gr << 21;
        zone[z * 4 + 1] = gr >> 11 | bl << 10;
        zone[z * 4 + 2] = n << 20;
        zone[z * 4 + 3] = n >> 12;
    }
    for (unsigned i = 0; i < ARRAY_SIZE(zone); i++) {
        zone[i] = cpu_to_le32(zone[i]);
    }
    isp_stats_dma(R(b + 0x3c + buf * 4), zone, sizeof(zone));
    R(b + 0x50) = buf;
    isp_raise(s, isp_layout(s)->pipe[pipe].int_sta, st->int_awb);
}

/* The T32's: both sets a zone, in groups of 4 zones */
static void isp_awb_write_t32(IngenicIspState *s, unsigned pipe,
                              const IspStatsSums *sum, const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb + pipe * st->sec;
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
    isp_raise(s, isp_layout(s)->pipe[pipe].int_sta, st->int_awb);
}

/*
 * The T33's and T32 Pro's: a zone is 28 bytes, the R, G and B sums of all
 * its quads and of its white ones (32 bits each) and their numbers (16), a
 * row of zones padded to 8 bytes, the tag after the last
 */
static void isp_awb_write_flat(IngenicIspState *s, unsigned pipe,
                               uint32_t frame, const IspStatsSums *sum,
                               const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb + pipe * st->sec;
    uint32_t buf = (R(b + 0x50) + 1) % ((R(b + 0x4c) & 3) + 1);
    unsigned stride = ROUND_UP(g->cols * FLAT_AWB_ZONE, 8);
    size_t len = FLAT_HEAD + stride * g->rows + 2;
    g_autofree uint8_t *w = g_malloc0(len);

    if (!g->cols || !g->rows) {
        return;
    }
    stw_le_p(w, frame);
    for (unsigned r = 0; r < g->rows; r++) {
        for (unsigned c = 0; c < g->cols; c++) {
            unsigned zi = r * g->cols + c;
            uint8_t *z = w + FLAT_HEAD + r * stride + c * FLAT_AWB_ZONE;

            for (unsigned f = 0; f < 3; f++) {
                stl_le_p(z + f * 4, sum->awb_all[zi][f]);
                stl_le_p(z + 12 + f * 4, sum->awb[zi][f]);
            }
            stw_le_p(z + 24, MIN(sum->awb_all[zi][3], 0xffff));
            stw_le_p(z + 26, MIN(sum->awb[zi][3], 0xffff));
        }
    }
    stw_le_p(w + len - 2, frame);
    isp_stats_dma(R(b + 0x3c + buf * 4), w, len);
    R(b + 0x50) = buf;
    isp_raise(s, isp_layout(s)->pipe[pipe].int_sta, st->int_awb);
}

/*
 * The white region, in 1/256ths: R/G and B/G rounded down strictly between
 * the halves of +0x28 and +0x2c, so that a quad without R or B is never
 * white, and (c * R + 256 * B) / G, c the low half of +0x30 and +0x34, at
 * least the high half of +0x30 and at most that of +0x34
 */
/* The white region, +0x28 to +0x34 of the AWB block */
typedef struct IspAwbWhite {
    uint32_t rg, bg, lo, hi;
} IspAwbWhite;

/* The T40's and T41's: a pair of records for each luma band */
static void isp_awb_write_pairs(IngenicIspState *s, unsigned pipe,
                                const IspStatsSums *sum,
                                const IspStatsGrid *g)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr b = st->awb + pipe * st->sec;
    uint32_t buf = (R(b + 0x50) + 1) % ((R(b + 0x4c) & 3) + 1);
    bool one = R(b + 0x4c) & BIT(28);
    unsigned pairs = one ? 1 : 4;
    unsigned zones = g->rows * g->cols;
    g_autofree uint32_t *w = g_new0(uint32_t, zones * pairs * 8 + 1);

    for (unsigned z = 0; z < zones; z++) {
        for (unsigned p = 0; p < pairs; p++) {
            unsigned band = one ? extract32(R(b + 0x4c), 25, 2) : p;

            for (unsigned r = 0; r < 2; r++) {
                const uint64_t *v = r ? sum->awb_white[z][band]
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
    isp_raise(s, isp_layout(s)->pipe[pipe].int_sta, st->int_awb);
}

/*
 * The T31's ADR, defog and AF statistics, as its tx-isp firmware reads them
 * (tiziano_adr_get_data, tiziano_defog_get_data, tisp_af_get_statistics) and a
 * T31X streaming 2560x1440 writes them. Each block writes, every frame, to the
 * next of up to 4 buffers (their count less one in bits 1:0 of a word of its
 * own) and raises a status bit. Each buffer carries the frame's tag, AE's
 * +0x50 bits 15:8.
 *
 * - ADR (bit 18): buffers from 0x4494 (count 0x4490), the address of the last
 *   written at 0x44b0. The tag, a word of 0, then 6 x 8 zones of 16 bytes: how
 *   many quads have their luma below the 4 thresholds of 0x4484 and 0x4488 (a
 *   half each, rising) and how many above the last (16 bits each, the fifth in
 *   the zone's fourth word), and the sum of their luma (28 bits, the third
 *   word). The luma is 12 bits of linear light, AE's times 16 (a T31X's ADR
 *   and AE histograms agree). The zones' edges are the frame's, its margins in
 *   0x4454 (rows) and 0x4458 (columns), low half first, and between them the
 *   inner edges of the tone map's 4 x 6 blocks (0x4010 to 0x4024). After the
 *   zones, a histogram of every quad's luma in 512 bins (21 bits, three to a
 *   64-bit word), then the tag again.
 *
 * - Defog (bit 20): buffers from 0x5b84 (count 0x5b80), the address of the
 *   last written at 0x5ba4. The tag, a word of 0, then 10 x 18 zones of 16
 *   bytes, their edges in pixels from 0x5800 (rows) and 0x5820 (columns), 12
 *   bits a half: the sums of the picture's B, G and R (24 bits each, R split
 *   12 and 12 between the second word and the fourth) and how many of the
 *   zone's pixels have a luma below 0x5b10 bits 15:0, below bits 31:16, and
 *   not (16 bits each), then the tag again. The picture is the one put out,
 *   gamma and all, as a T31X's sums and that frame's agree.
 *
 * - AF (bit 31): buffers from 0xb8a8 (count 0xb8b8), the index of the last
 *   written at 0xb8bc. The zones of 0xb804 (columns in bits 31:28 and rows in
 *   15:12, the first's left and top edges in bits 27:16 and 11:0), sized in
 *   pixels a byte each from 0xb808 (columns) and 0xb818 (rows), 16 bytes each:
 *   two vertical (FIR) and two horizontal (IIR) filters' sums (22 bits each),
 *   the sum of the output luma (23 bits) and how many pixels are above 0xb828
 *   bits 15:8 (15 bits); bits 127:126 of the first 4 zones carry the tag, 2
 *   bits each from the low. The model does not decode the filters' taps,
 *   corings and gains (0xb830 to 0xb8a4): it sums the size of the luma's
 *   second difference over 1 and 2 pixels, scaled to what a T31X sums at the
 *   edges in a room. A flat picture gives 0, where a sensor's noise gives a
 *   T31X about 1% of a zone's luma sum.
 *
 * AE1 (0xa800) and the WDR block's (0x2000, 0x2680) write nothing on a T31X
 * whose sensor runs without WDR, nor do they here.
 */
#define T31_ADR_ROWS        6
#define T31_ADR_COLS        8
#define T31_ADR_HIST        194             /* word of the histogram */
#define T31_ADR_WORDS       0x219
#define T31_DEFOG_ROWS      10
#define T31_DEFOG_COLS      18
#define T31_DEFOG_WORDS     (2 + T31_DEFOG_ROWS * T31_DEFOG_COLS * 4 + 1)

/* Luma in 12 bits of linear light */
static uint16_t isp_linear12[256];

/* The next of a block's buffers, noted as its address or its index */
static hwaddr isp_t31_next(IngenicIspState *s, hwaddr count, hwaddr bufs,
                           hwaddr last, bool index)
{
    unsigned n = (R(count) & 3) + 1, i = 0;

    if (index) {
        i = (R(last) + 1) % n;
    } else {
        for (unsigned k = 0; k < n; k++) {
            if (R(bufs + k * 4) == R(last)) {
                i = (k + 1) % n;
                break;
            }
        }
    }
    R(last) = index ? i : R(bufs + i * 4);
    return R(bufs + i * 4);
}

/* For each of len positions the span of edges[] it lies in, or -1 */
static void isp_t31_map(int8_t *map, uint32_t len, const uint32_t *edges,
                        unsigned spans)
{
    memset(map, -1, len);
    for (unsigned i = 0; i < spans; i++) {
        for (uint32_t x = edges[i]; x < MIN(edges[i + 1], len); x++) {
            map[x] = i;
        }
    }
}

static void isp_t31_dma(hwaddr addr, uint32_t *w, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        w[i] = cpu_to_le32(w[i]);
    }
    isp_stats_dma(addr, w, n * 4);
}

/* The ADR's counts and sums over a quarter-size frame's rows, in bands */
typedef struct IspT31AdrPart {
    uint32_t cnt[T31_ADR_ROWS * T31_ADR_COLS][5];
    uint64_t sum[T31_ADR_ROWS * T31_ADR_COLS];
    uint32_t hist[512];
} IspT31AdrPart;

typedef struct IspT31Adr {
    const uint8_t *pic;
    uint32_t qw, qh, thr[4];
    const int8_t *row, *col;
    IspT31AdrPart *part;
} IspT31Adr;

static void isp_t31_adr_band(void *arg, unsigned bi, unsigned n)
{
    const IspT31Adr *a = arg;
    IspT31AdrPart *part = &a->part[bi];
    uint32_t (*cnt)[5] = part->cnt;
    uint64_t *sum = part->sum;
    uint32_t *hist = part->hist;
    const uint8_t *pic = a->pic;
    const int8_t *row = a->row, *col = a->col;
    const uint32_t *thr = a->thr;
    uint32_t qw = a->qw, qh = a->qh, y1 = qh * (bi + 1) / n;

    for (uint32_t y = qh * bi / n, m; y < y1; y += m) {
        const uint8_t *p = pic + y * qw;
        int r = row[y * 2];

        for (m = 1; y + m < y1 && row[(y + m) * 2] == r &&
                    !memcmp(p + m * qw, p, qw); m++) {
        }
        for (uint32_t x = 0; x < qw; x++) {
            uint32_t l = isp_linear12[p[x]];
            unsigned b = 0;

            hist[l >> 3] += m;
            if (r < 0 || col[x * 2] < 0) {
                continue;
            }
            while (b < 4 && l >= thr[b]) {
                b++;
            }
            cnt[r * T31_ADR_COLS + col[x * 2]][b] += m;
            sum[r * T31_ADR_COLS + col[x * 2]] += (uint64_t)l * m;
        }
    }
}

static void isp_t31_adr(IngenicIspState *s, const uint8_t *pic, uint32_t qw,
                        uint32_t qh, uint32_t tag)
{
    const uint32_t rows[T31_ADR_ROWS + 1] = {
        0, extract32(R(0x4454), 0, 16), R(0x4010) >> 16,
        extract32(R(0x4014), 0, 16), R(0x4014) >> 16, R(0x4454) >> 16, qh * 2,
    };
    const uint32_t cols[T31_ADR_COLS + 1] = {
        0, extract32(R(0x4458), 0, 16), R(0x401c) >> 16,
        extract32(R(0x4020), 0, 16), R(0x4020) >> 16,
        extract32(R(0x4024), 0, 16), R(0x4024) >> 16, R(0x4458) >> 16,
        qw * 2,
    };
    const uint32_t thr[4] = {
        extract32(R(0x4484), 0, 16), R(0x4484) >> 16,
        extract32(R(0x4488), 0, 16), R(0x4488) >> 16,
    };
    g_autofree IspT31AdrPart *part = NULL;
    unsigned bands = (size_t)qw * qh < MDNS_BAND_PIXELS ? 1 : isp_nbands(s);
    uint32_t (*cnt)[5];
    uint64_t *sum;
    uint32_t *hist;
    uint32_t w[T31_ADR_WORDS] = { tag };
    g_autofree int8_t *row = g_malloc(qh * 2);
    g_autofree int8_t *col = g_malloc(qw * 2);
    IspT31Adr adr = { .pic = pic, .qw = qw, .qh = qh, .row = row, .col = col };
    hwaddr addr;

    isp_t31_map(row, qh * 2, rows, T31_ADR_ROWS);
    isp_t31_map(col, qw * 2, cols, T31_ADR_COLS);
    memcpy(adr.thr, thr, sizeof(thr));
    part = adr.part = g_new0(IspT31AdrPart, bands);
    isp_par(s, bands, isp_t31_adr_band, &adr);
    cnt = part[0].cnt;
    sum = part[0].sum;
    hist = part[0].hist;
    for (unsigned i = 1; i < bands; i++) {
        for (unsigned z = 0; z < T31_ADR_ROWS * T31_ADR_COLS; z++) {
            for (unsigned k = 0; k < 5; k++) {
                cnt[z][k] += part[i].cnt[z][k];
            }
            sum[z] += part[i].sum[z];
        }
        for (unsigned k = 0; k < 512; k++) {
            hist[k] += part[i].hist[k];
        }
    }
    for (unsigned z = 0; z < T31_ADR_ROWS * T31_ADR_COLS; z++) {
        uint32_t *zw = w + 2 + z * 4;

        for (unsigned b = 0; b < 4; b++) {
            isp_stats_put(zw, b * 16, 16, cnt[z][b]);
        }
        isp_stats_put(zw, 64, 28, MIN(sum[z], UINT32_MAX));
        isp_stats_put(zw, 96, 16, cnt[z][4]);
    }
    for (unsigned i = 0; i < 512; i++) {
        isp_stats_put(w + T31_ADR_HIST, i / 3 * 64 + i % 3 * 21, 21, hist[i]);
    }
    w[T31_ADR_WORDS - 1] = tag;
    addr = isp_t31_next(s, 0x4490, 0x4494, 0x44b0, false);
    if (addr) {
        isp_t31_dma(addr, w, T31_ADR_WORDS);
    }
}

/* The sums of a row's pixels in each zone column: AF's, then defog's */
typedef struct IspT31Row {
    uint64_t af[STATS_GRID_MAX][4];         /* luma, bright, IIR0, IIR1 */
    uint64_t defog[T31_DEFOG_COLS][6];      /* R, G, B; luma's 3 bins */
} IspT31Row;

/* AF's: luma, bright, IIR0, IIR1, FIR0, FIR1; defog's: R, G, B, bins */
typedef uint64_t IspT31Zone[6];

/* The columns a zone takes, from a column map (-1: none) */
typedef struct IspT31Span {
    uint32_t x0, x1;
    unsigned zone;
} IspT31Span;

static unsigned isp_t31_spans(const int8_t *map, uint32_t len,
                              IspT31Span *sp)
{
    unsigned n = 0;

    for (uint32_t x = 0; x < len; x++) {
        if (map[x] < 0) {
            continue;
        }
        if (n && sp[n - 1].x1 == x && sp[n - 1].zone == map[x]) {
            sp[n - 1].x1++;
        } else {
            sp[n++] = (IspT31Span){ x, x + 1, map[x] };
        }
    }
    return n;
}

/* The sums of a frame's rows for the defog and AF zones, in bands of rows */
typedef struct IspT31Part {
    IspT31Row r;
    IspT31Zone af[STATS_ZONES];
    IspT31Zone df[T31_DEFOG_ROWS * T31_DEFOG_COLS];
} IspT31Part;

typedef struct IspT31Sums {
    const uint8_t *pic;
    uint32_t fw, fh, bright, t0, t1;    /* t0, t1: the defog's thresholds */
    const int8_t *afc, *afr, *dfc, *dfr;
    const IspT31Span *afs, *dfs;
    unsigned naf, ndf, af_cols;
    IspT31Part *part;
} IspT31Sums;

static void isp_t31_row(const IspT31Sums *sm, const uint8_t *luma,
                        const uint8_t *uv, IspT31Row *r)
{
    uint32_t fw = sm->fw, bright = sm->bright, t0 = sm->t0, t1 = sm->t1;
    const IspT31Span *afs = sm->afs, *dfs = sm->dfs;
    unsigned naf = sm->naf, ndf = sm->ndf;

    memset(r, 0, sizeof(*r));
    for (unsigned i = 0; i < naf; i++) {
        ISP_KERNEL(af_span, luma, afs[i].x0, afs[i].x1, fw, bright,
                   r->af[afs[i].zone]);
    }
    for (unsigned i = 0; i < ndf; i++) {
        ISP_KERNEL(defog_span, luma, uv, dfs[i].x0, dfs[i].x1, t0, t1,
                   r->defog[dfs[i].zone]);
    }
}

static void isp_t31_band(void *arg, unsigned bi, unsigned n)
{
    const IspT31Sums *sm = arg;
    IspT31Part *part = &sm->part[bi];
    const uint8_t *pic = sm->pic, *uvp = sm->pic + (size_t)sm->fw * sm->fh;
    uint32_t fw = sm->fw, fh = sm->fh;
    const int8_t *afr = sm->afr, *dfr = sm->dfr;
    const IspT31Span *afs = sm->afs;
    unsigned naf = sm->naf, af_cols = sm->af_cols;
    IspT31Row *r = &part->r;
    IspT31Zone *af = part->af, *df = part->df;
    uint32_t y0 = fh * bi / n, y1 = fh * (bi + 1) / n;

    for (uint32_t y = fh * bi / n; y < y1; y++) {
        const uint8_t *l = pic + (size_t)y * fw;
        const uint8_t *uv = uvp + (size_t)(y / 2) * fw;
        const uint8_t *nb[4] = {
            pic + (size_t)(y > 1 ? y - 2 : 0) * fw,
            pic + (size_t)(y ? y - 1 : 0) * fw,
            pic + (size_t)MIN(y + 1, fh - 1) * fw,
            pic + (size_t)MIN(y + 2, fh - 1) * fw,
        };
        int ar = afr[y], dr = dfr[y];

        /* rows alike sum alike: most of a scene's are */
        if (y == y0 || memcmp(l, l - fw, fw) ||
            memcmp(uv, uvp + (size_t)((y - 1) / 2) * fw, fw)) {
            isp_t31_row(sm, l, uv, r);
        }
        if (ar >= 0) {
            for (unsigned c = 0; c < af_cols; c++) {
                uint64_t *z = af[ar * af_cols + c];

                z[0] += r->af[c][0];
                z[1] += r->af[c][1];
                z[2] += r->af[c][2];
                z[3] += r->af[c][3];
            }
            /* the vertical filters, where the rows around differ */
            if (memcmp(nb[0], l, fw) || memcmp(nb[1], l, fw) ||
                memcmp(nb[2], l, fw) || memcmp(nb[3], l, fw)) {
                for (unsigned i = 0; i < naf; i++) {
                    ISP_KERNEL(af_vert, l, nb[0], nb[1], nb[2], nb[3],
                               afs[i].x0, afs[i].x1,
                               af[ar * af_cols + afs[i].zone] + 4);
                }
            }
        }
        if (dr >= 0) {
            for (unsigned c = 0; c < T31_DEFOG_COLS; c++) {
                uint64_t *z = df[dr * T31_DEFOG_COLS + c];

                for (unsigned i = 0; i < 6; i++) {
                    z[i] += r->defog[c][i];
                }
            }
        }
    }

}

/* @c's sums, into @af and @df */
static void isp_t31_sums(IngenicIspState *s, IspT31Sums *c, IspT31Zone *af,
                         IspT31Zone *df)
{
    unsigned bands = (size_t)c->fw * c->fh < MDNS_BAND_PIXELS ? 1 :
                     isp_nbands(s);
    g_autofree IspT31Part *part = g_new0(IspT31Part, bands);

    c->part = part;
    isp_par(s, bands, isp_t31_band, c);
    for (unsigned i = 0; i < bands; i++) {
        for (unsigned z = 0; z < STATS_ZONES; z++) {
            for (unsigned k = 0; k < 6; k++) {
                af[z][k] += part[i].af[z][k];
            }
        }
        for (unsigned z = 0; z < T31_DEFOG_ROWS * T31_DEFOG_COLS; z++) {
            for (unsigned k = 0; k < 6; k++) {
                df[z][k] += part[i].df[z][k];
            }
        }
    }
}

static void isp_t31_defog_af(IngenicIspState *s, const uint8_t *pic,
                             uint32_t fw, uint32_t fh, uint32_t tag)
{
    uint32_t af_grid = R(0xb804);
    unsigned af_cols = MIN(af_grid >> 28, STATS_GRID_MAX);
    unsigned af_rows = MIN(extract32(af_grid, 12, 4), STATS_GRID_MAX);
    uint32_t af_x[STATS_GRID_MAX + 1] = { extract32(af_grid, 16, 12) };
    uint32_t af_y[STATS_GRID_MAX + 1] = { extract32(af_grid, 0, 12) };
    uint32_t df_x[T31_DEFOG_COLS + 1], df_y[T31_DEFOG_ROWS + 1];
    g_autofree int8_t *afc = g_malloc(fw), *afr = g_malloc(fh);
    g_autofree int8_t *dfc = g_malloc(fw), *dfr = g_malloc(fh);
    g_autofree IspT31Span *afs = g_new(IspT31Span, fw);
    g_autofree IspT31Span *dfs = g_new(IspT31Span, fw);
    unsigned naf, ndf;
    g_autofree IspT31Row *r = g_new0(IspT31Row, 1);
    g_autofree IspT31Zone *af = g_new0(IspT31Zone, STATS_ZONES);
    g_autofree IspT31Zone *df =
        g_new0(IspT31Zone, T31_DEFOG_ROWS * T31_DEFOG_COLS);
    uint32_t w[T31_DEFOG_WORDS] = { tag };
    uint32_t aw[STATS_ZONES * 4] = { 0 };
    hwaddr addr;

    for (unsigned i = 0; i < af_cols; i++) {
        af_x[i + 1] = af_x[i] + extract32(R(0xb808 + i / 4 * 4), i % 4 * 8, 8);
    }
    for (unsigned i = 0; i < af_rows; i++) {
        af_y[i + 1] = af_y[i] + extract32(R(0xb818 + i / 4 * 4), i % 4 * 8, 8);
    }
    for (unsigned i = 0; i <= T31_DEFOG_COLS; i++) {
        df_x[i] = extract32(R(0x5820 + i / 2 * 4), i % 2 * 16, 12);
    }
    for (unsigned i = 0; i <= T31_DEFOG_ROWS; i++) {
        df_y[i] = extract32(R(0x5800 + i / 2 * 4), i % 2 * 16, 12);
    }
    isp_t31_map(afc, fw, af_x, af_cols);
    isp_t31_map(afr, fh, af_y, af_rows);
    isp_t31_map(dfc, fw, df_x, T31_DEFOG_COLS);
    isp_t31_map(dfr, fh, df_y, T31_DEFOG_ROWS);
    naf = isp_t31_spans(afc, fw, afs);
    ndf = isp_t31_spans(dfc, fw, dfs);

    isp_t31_sums(s, &(IspT31Sums){
        .pic = pic, .fw = fw, .fh = fh, .afc = afc, .afr = afr, .dfc = dfc,
        .dfr = dfr, .afs = afs, .dfs = dfs, .naf = naf, .ndf = ndf,
        .af_cols = af_cols, .bright = extract32(R(0xb828), 8, 8),
        .t0 = extract32(R(0x5b10), 0, 16), .t1 = R(0x5b10) >> 16,
    }, af, df);

    for (unsigned i = 0; i < T31_DEFOG_ROWS * T31_DEFOG_COLS; i++) {
        uint32_t *zw = w + 2 + i * 4;
        uint32_t red = MIN(df[i][0], 0xffffff);

        isp_stats_put(zw, 0, 24, MIN(df[i][2], 0xffffff));
        isp_stats_put(zw, 24, 24, MIN(df[i][1], 0xffffff));
        isp_stats_put(zw, 48, 12, red & 0xfff);
        isp_stats_put(zw, 64, 16, df[i][3]);
        isp_stats_put(zw, 80, 16, df[i][4]);
        isp_stats_put(zw, 96, 16, df[i][5]);
        isp_stats_put(zw, 112, 12, red >> 12);
    }
    w[T31_DEFOG_WORDS - 1] = tag;
    addr = isp_t31_next(s, 0x5b80, 0x5b84, 0x5ba4, false);
    if (addr) {
        isp_t31_dma(addr, w, T31_DEFOG_WORDS);
    }

    for (unsigned z = 0; z < af_rows * af_cols; z++) {
        uint32_t *zw = aw + z * 4;

        /* scaled to a T31X's sums */
        isp_stats_put(zw, 0, 22, MIN(af[z][4] * 23 >> 8, UINT32_MAX));
        isp_stats_put(zw, 22, 22, MIN(af[z][5] * 5 >> 8, UINT32_MAX));
        isp_stats_put(zw, 44, 22, MIN(af[z][2] * 20 >> 8, UINT32_MAX));
        isp_stats_put(zw, 66, 22, MIN(af[z][3] * 9 >> 8, UINT32_MAX));
        isp_stats_put(zw, 88, 23, MIN(af[z][0], UINT32_MAX));
        isp_stats_put(zw, 111, 15, af[z][1]);
        if (z < 4) {
            isp_stats_put(zw, 126, 2, extract32(tag, z * 2, 2));
        }
    }
    addr = isp_t31_next(s, 0xb8b8, 0xb8a8, 0xb8bc, true);
    if (addr) {
        isp_t31_dma(addr, aw, af_rows * af_cols * 4);
    }
}

/*
 * The T31 in WDR mode (0x10 reads 0x33f, 0x133 linear; the model takes
 * bit 9), as a T31X with a GC4653 in its DOL mode runs it: AE0 meters the
 * long exposure, AE1 (0xa800, laid out as AE0, bits 28 and 29) the short
 * one, and the WDR block (bit 11) writes to the next of its buffers from
 * 0x2010 (count 0x2024), the address of the last written at 0x2680, 8 rows
 * of 129 words 0x2020 bytes apart (tiziano_wdr_interrupt_static packs
 * them): per bin of 256 the number of quads whose R, G and B (8 bits of
 * linear light) fall in it, of the long exposure (21 bits each, packed in 2
 * words) and of the short one (2 more), then the numbers of quads whose
 * 12-bit luma of the long exposure is below 0x2030 bits 27:16, below 4000
 * and above, the same of the short one against 0x2034 bits 11:0, the long
 * one's luma sum, a word of 0, the short one's luma sum and a word of 0.
 * The luma is (R + 2 G + B) / 4 of the histograms' channels. A T31X's three
 * numbers sum to the quads of AE's zones (1275 of 1280 columns) less those
 * with a channel at 250 or more of 255, to within 1% on 4 exposures, the
 * model's rule. The thresholds themselves are not known (4000 is the
 * model's): 0x2028 to 0x203c change none of the numbers beyond their noise
 * of some 100.
 */
#define T31_WDR_ON          BIT(9)
#define T31_WDR_ROW_WORDS   129
#define T31_WDR_WORDS       (T31_WDR_ROW_WORDS * 8)

static bool isp_t31_wdr(IngenicIspState *s, ImageSensorState *sensor)
{
    const IngenicIspStats *st = isp_layout(s)->stats;

    return st->t31 && (R(0x10) & T31_WDR_ON) && image_sensor_wdr(sensor);
}

/*
 * An exposure's quads: AE1's sums, the WDR block's histograms and counts.
 * The block's channels read 1.235 times AE's, which meters about 0.81 of
 * the block's luma, (R + 2 G + B) / 4 to within 2%, over nine exposures on
 * a T31X; both clip at full scale
 */
#define T31_WDR_GAIN        1235    /* per mille */

static void isp_t31_wdr_pass(const uint8_t *pic, uint32_t qw, uint32_t qh,
                             IspStatsSums *ae, const IspStatsGrid *g,
                             uint32_t dark, uint32_t bright, uint32_t *hist,
                             uint32_t thr, uint32_t *num, uint64_t *ysum)
{
    for (uint32_t y = 0; y < qh; y++) {
        const uint8_t *row = pic + y * qw;
        const uint8_t *uv = pic + qw * qh + y / 2 * qw;

        for (uint32_t x = 0; x < qw; x++) {
            int luma = row[x];
            uint32_t al = isp_linear[luma], l12;
            int u = uv[x & ~1] - 128, v = uv[x | 1] - 128;
            int c = (luma - 16) * 298 + 128;
            /* Limited-range BT.601, as the scene is rendered */
            int rgb[3] = {
                MIN(MAX((c + 409 * v) >> 8, 0), 255),
                MIN(MAX((c - 100 * u - 208 * v) >> 8, 0), 255),
                MIN(MAX((c + 516 * u) >> 8, 0), 255),
            };
            uint32_t sum12 = 0;
            bool clipped = false;

            if (ae) {
                ae->hist[al]++;
                if (g->row[y] >= 0 && g->col[x] >= 0) {
                    uint32_t *z = ae->ae[g->row[y] * g->cols + g->col[x]];
                    unsigned k = al < dark ? 0 : al > bright ? 2 : 1;

                    z[k] += al;
                    if (k != 1) {
                        z[3 + k / 2]++;
                    }
                }
            }
            for (unsigned i = 0; i < 3; i++) {
                unsigned bin = MIN(isp_linear[rgb[i]] * T31_WDR_GAIN / 1000,
                                   255);

                hist[bin * 3 + i]++;
                clipped |= bin >= 250;
                sum12 += MIN(isp_linear12[rgb[i]] * T31_WDR_GAIN / 1000,
                             4095) * (i == 1 ? 2 : 1);
            }
            l12 = sum12 / 4;
            if (!clipped && g->row[y] >= 0 && g->col[x] >= 0) {
                num[l12 < thr ? 0 : l12 < 4000 ? 1 : 2]++;
            }
            *ysum += l12;
        }
    }
}

static void isp_t31_wdr_stats(IngenicIspState *s, ImageSensorState *sensor,
                              uint32_t frame)
{
    uint32_t fw, fh, qw, qh;
    bool ae1 = R(0xa800) & 1;
    g_autofree uint8_t *pic = NULL;
    g_autofree uint32_t *hist = g_new0(uint32_t, 2 * 256 * 3);
    g_autofree uint32_t *w = g_new0(uint32_t, T31_WDR_WORDS);
    g_autofree IspStatsSums *sum = g_new0(IspStatsSums, 1);
    uint32_t num[2][3] = { { 0 } };
    uint64_t ysum[2] = { 0 };
    IspStatsGrid g;
    hwaddr addr;

    isp_frame_size(s, &fw, &fh);
    qw = fw / 2 & ~1;
    qh = fh / 2 & ~1;
    if (!qw || !qh) {
        return;
    }
    pic = g_malloc(qw * qh * 3 / 2);
    isp_stats_grid(s, 0xa800, &g, qw, qh);
    for (unsigned e = 0; e < 2; e++) {
        image_sensor_render_exposure(sensor, frame, qw, qh, pic,
                                     e ? IMAGE_SENSOR_SHORT :
                                     IMAGE_SENSOR_LONG);
        isp_t31_wdr_pass(pic, qw, qh, e && ae1 ? sum : NULL, &g,
                         extract32(R(0xa828), 0, 8),
                         extract32(R(0xa828), 8, 8), hist + e * 768,
                         e ? extract32(R(0x2034), 0, 12) :
                             extract32(R(0x2030), 16, 12),
                         num[e], &ysum[e]);
    }
    if (ae1) {
        isp_ae_write(s, 0, 0xa800, BIT(28) | BIT(29), sum, &g);
    }
    isp_stats_grid_free(&g);

    if (!R(0x2010)) {
        return;
    }
    for (unsigned i = 0; i < 256; i++) {
        for (unsigned e = 0; e < 2; e++) {
            for (unsigned c = 0; c < 3; c++) {
                isp_stats_put(w + i * 4 + e * 2, c * 21, 21,
                              hist[e * 768 + i * 3 + c]);
            }
        }
    }
    for (unsigned e = 0; e < 2; e++) {
        for (unsigned c = 0; c < 3; c++) {
            isp_stats_put(w + 1024 + e * 2, c * 21, 21, num[e][c]);
        }
        w[1028 + e * 2] = MIN(ysum[e], UINT32_MAX);
    }
    addr = isp_t31_next(s, 0x2024, 0x2010, 0x2680, false);
    if (addr) {
        uint32_t stride = R(0x2020) ?: T31_WDR_ROW_WORDS * 4;

        for (unsigned r = 0; r < 8; r++) {
            isp_t31_dma(addr + r * stride, w + r * T31_WDR_ROW_WORDS,
                        T31_WDR_ROW_WORDS);
        }
    }
    isp_raise(s, isp_layout(s)->pipe[0].int_sta, BIT(11));
}

static void isp_stats_t31(IngenicIspState *s, ImageSensorState *sensor,
                          uint32_t frame)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    uint32_t tag = extract32(R(st->ae + 0x50), 8, 8);
    uint32_t fw, fh, bits = 0;
    g_autofree uint8_t *pic = NULL;

    isp_frame_size(s, &fw, &fh);
    if (!fw || !fh) {
        return;
    }
    if (R(0x4494)) {
        uint32_t qw = fw / 2 & ~1, qh = fh / 2 & ~1;
        const uint8_t *ref = isp_pic_ref(s, sensor, frame, qw, qh);

        if (!ref) {
            pic = g_malloc((size_t)qw * qh * 3 / 2);
            image_sensor_render(sensor, frame, qw, qh, pic);
            isp_wb_gains(s, sensor, 0, qw, qh, pic, pic + (size_t)qw * qh);
            ref = pic;
        }
        isp_t31_adr(s, ref, qw, qh, tag);
        g_free(g_steal_pointer(&pic));
        bits |= BIT(18);
    }
    if (R(0x5b84) || R(0xb8a8)) {
        const uint8_t *ref = isp_pic_ref(s, sensor, frame, fw, fh);

        if (!ref) {
            pic = g_malloc((size_t)fw * fh * 3 / 2);
            image_sensor_render(sensor, frame, fw, fh, pic);
            isp_wb_gains(s, sensor, 0, fw, fh, pic, pic + (size_t)fw * fh);
            ref = pic;
        }
        isp_t31_defog_af(s, ref, fw, fh, tag);
        bits |= (R(0x5b84) ? BIT(20) : 0) | (R(0xb8a8) ? BIT(31) : 0);
    }
    isp_raise(s, isp_layout(s)->pipe[0].int_sta, bits);
    if (isp_t31_wdr(s, sensor)) {
        isp_t31_wdr_stats(s, sensor, frame);
    }
}

/*
 * The T41's statistics beyond AE and AWB, as its tx-isp firmware reads them
 * (tisp_af_get_statistics, tisp_gsm_read_ram, tisp_lce_get_data,
 * tiziano_adr_read_data, tisp_defog_get_data, tisp_tmo_get_data) and a T41NQ
 * streaming 2560x1440 writes them, each block raising its bit in 0x40028
 * once it wrote a frame's. The tag is the frame's number, AF's low byte.
 *
 * - AF (bit 6): buffers from 0x1a0cc (count 0x1a0dc bits 1:0), the index of
 *   the last written at 0x1a0e0 and the frame's number (16 bits) at 0x1a120.
 *   Its zones are the T31's (0x1a004 as the T31's 0xb804; sizes in pixels,
 *   16 bits each from 0x1a008 for columns and a byte each from 0x1a028 for
 *   rows; bright above 0x1a038 bits 15:8), 17 bytes each, a row of them
 *   padded to 8 bytes with the tag in the last: the four filters' sums (23
 *   bits each, as the T31's), the output luma's sum (24 bits) and how many
 *   pixels are bright (16 bits).
 *
 * - GSM (bit 7): two histograms of 256 bins (22 bits), read once 0x1b004 and
 *   0x1b044 read 1 through the RAM ports at 0x501a0 and 0x501b0 (a write
 *   with bit 0 set starts a read at bin 0, each read of +8 the next bin); a
 *   write of 1 to 0x1b000 or 0x1b040 clears the flags. A T41NQ's wrote none,
 *   armed so or not; the model writes the even and odd columns' linear luma
 *   once armed, which its driver never does first.
 *
 * - LCE (bit 8): the 5 x 9 blocks of 0xe000 (height << 16 | width, pixels;
 *   the first 0xe004 bits 3:0 columns 2 wider, bits 10:8 rows 1 taller),
 *   each a histogram of every other pixel's luma in 32 bins and the sum of
 *   that luma, into one of two banks in turn, but not into one still unread:
 *   0xe014 bit 0 says bank 0 was filled, bits 8 and 16 bank 1 (bit 16 stays
 *   until bank 0 is next), a write of 0xe010 bit 0 or 16 clears bit 0 or 8.
 *   Bank 0's ports are 0x501c0 (the histograms) and 0x50200 (the sums), bank
 *   1's 0x501d0 and 0x50210; reading a histogram clears it. A T41NQ's luma
 *   is that before the LCE's curve; the model's, the output's.
 *
 * - ADR (bit 9): buffers from 0x9514 (count 0x9510), the address of the last
 *   written at 0x9540, the tag at 0x9544. The tag, a word of 0, then 4 x 6
 *   zones of 14 words, their edges in 0x9030 to 0x9038 (rows) and 0x903c to
 *   0x9048 (columns), 16 bits each: a histogram of the zone's quads' 12-bit
 *   linear luma in 20 bins of 18 bits (isp_t41_adr_bin: the quads above the
 *   last are left out), the most and the least of that luma (12 bits each)
 *   and its sum (29 bits, twice; a T41NQ's second is 5 to 40% above the
 *   first, more in coloured zones, which the model does not take). After
 *   the zones the T31's histogram of every quad's luma in 512 bins, then
 *   the tag again.
 *
 * - Defog (bit 10): buffers from 0xc290 (count 0xc28c), the address of the
 *   last written at 0xc2b4, the tag at 0xc2b0. The tag, a word of 0, then
 *   10 x 18 zones of 8 bytes, 0xc008 their height (bits 23:16) and width
 *   (bits 7:0) in pixels from the top left, each the sums of the output
 *   picture's B, G and R (21 bits each; the order as a T41NQ's in daylight)
 *   right-shifted by 0xc284, then the tag again.
 *
 * - TMO (bit 17, enabled by 0x1e020 bit 0): two buffers of 16 KiB at 0x1e010
 *   and 0x1e014, the index of the last written at 0x1e028 and its address at
 *   0x1e02c. The 15 x 25 zones of 0x1e080 (width in bits 15:8, height in 7:0,
 *   pixels; the last column and row to the frame's edges) row by row, each a
 *   histogram of its pixels' luma, a 13-bit log2 of linear light, in 10 bins
 *   of 819: per bin a pair of 16-bit halves, the sum of the luma above the
 *   bin's floor right-shifted by 9 (as the driver's tisp_tmo_fpga weighs
 *   them), and the number of pixels; the rest of the buffer it leaves
 *   alone. Where the log scale starts is the model's: neither the driver
 *   nor a T41NQ's tone-mapped picture says.
 */
#define T41_AF_BYTES        17
#define T41_ADR_ROWS        4
#define T41_ADR_COLS        6
#define T41_ADR_ZONE_WORDS  14
#define T41_ADR_BINS        20
#define T41_ADR_ZONES       (T41_ADR_ROWS * T41_ADR_COLS)
#define T41_ADR_HIST        (2 + T41_ADR_ZONES * T41_ADR_ZONE_WORDS)
#define T41_ADR_WORDS       (T41_ADR_HIST + 171 * 2 + 1)
#define T41_DEFOG_ROWS      10
#define T41_DEFOG_COLS      18
#define T41_DEFOG_WORDS     (2 + T41_DEFOG_ROWS * T41_DEFOG_COLS * 2 + 1)
#define T41_LCE_ROWS        5
#define T41_LCE_COLS        9
#define T41_TMO_ROWS        15
#define T41_TMO_COLS        25
#define T41_TMO_BINS        10
#define T41_TMO_BUF         0x4000
#define T41_TMO_BIN         819         /* of its 13-bit log luma */
#define T41_RAM_PORT        0x501a0     /* the first; one each 0x10 */
#define T41_RAM_PORT_END    0x50220

/* Luma in a 13-bit log2 of 12-bit linear light */
static uint16_t isp_log13[256];

/*
 * An 8-bit luma's ADR zone histogram bin: 0.35 stops each of linear light
 * below 3072 of 4095, none above (fitted to a T41NQ's)
 */
static uint8_t isp_t41_adr_bin[256];

/* The statistics RAM port at @off (its control word), or -1 */
static int isp_t41_ram_port(hwaddr off)
{
    if (off < T41_RAM_PORT || off >= T41_RAM_PORT_END) {
        return -1;
    }
    /* 0x501e0 and 0x501f0 are LCE's and others' tables, written */
    switch ((off - T41_RAM_PORT) >> 4) {
    case 4:
    case 5:
        return -1;
    default:
        return (off - T41_RAM_PORT) >> 4;
    }
}

/* The next word port @port reads */
static uint32_t isp_t41_ram_read(IngenicIspState *s, unsigned port)
{
    unsigned bank = port & 1, at = s->ram_at[port]++;
    const uint32_t *lce = s->lce[bank];
    uint32_t v;

    switch (port) {
    case 0:
    case 1:
        return at < 256 ? s->gsm[port][at] : 0;
    case 2:
    case 3:
        if (at >= INGENIC_ISP_LCE_BLOCKS * INGENIC_ISP_LCE_BINS) {
            return 0;
        }
        v = lce[at];
        s->lce[bank][at] = 0;
        return v;
    default:
        return at < INGENIC_ISP_LCE_BLOCKS ?
               lce[INGENIC_ISP_LCE_BLOCKS * INGENIC_ISP_LCE_BINS + at] : 0;
    }
}

typedef struct IspT41Sums {
    uint64_t af[STATS_ZONES][6];            /* as the T31's */
    uint64_t defog[T41_DEFOG_ROWS * T41_DEFOG_COLS][3];
    uint64_t lce[INGENIC_ISP_LCE_BLOCKS][INGENIC_ISP_LCE_BINS + 1];
    uint64_t tmo[T41_TMO_ROWS * T41_TMO_COLS][T41_TMO_BINS][2];
    uint32_t gsm[2][256];
} IspT41Sums;

/* Edges every @size from 0, the last at @len */
static void isp_t41_even(uint32_t *edges, unsigned n, uint32_t size,
                         uint32_t len)
{
    for (unsigned i = 0; i <= n; i++) {
        edges[i] = MIN(i * size, len);
    }
}

/* LCE's: @size each, the first @rem blocks @more larger */
static void isp_t41_lce_edges(uint32_t *edges, unsigned n, uint32_t size,
                              unsigned rem, unsigned more, uint32_t len)
{
    edges[0] = 0;
    for (unsigned i = 0; i < n; i++) {
        edges[i + 1] = MIN(edges[i] + size + (i < rem ? more : 0), len);
    }
}

/* The values of @l's samples from @x0 to @x1, @step apart, counted @m each */
static void isp_t41_hist(uint32_t *restrict h, const uint8_t *restrict l,
                         uint32_t x0, uint32_t x1, unsigned step, uint32_t m)
{
    for (uint32_t x = x0; x < x1; x += step) {
        h[l[x]] += m;
    }
}

static void isp_t41_af_write(IngenicIspState *s, const IspT41Sums *sum,
                             unsigned cols, unsigned rows, uint32_t frame)
{
    bool padded = cols * T41_AF_BYTES % 8;
    /* a row of zones, 17 bytes each, padded to 8 bytes */
    unsigned stride = DIV_ROUND_UP(cols * T41_AF_BYTES, 8) * 2;
    uint32_t w[1024] = { 0 };
    hwaddr addr;

    for (unsigned y = 0; y < rows; y++) {
        for (unsigned x = 0; x < cols; x++) {
            const uint64_t *z = sum->af[y * cols + x];
            uint32_t *zw = w + y * stride;
            unsigned at = x * T41_AF_BYTES * 8;

            /* scaled to a T31X's sums */
            isp_stats_put(zw, at, 23, MIN(z[4] * 23 >> 8, UINT32_MAX));
            isp_stats_put(zw, at + 23, 23, MIN(z[5] * 5 >> 8, UINT32_MAX));
            isp_stats_put(zw, at + 46, 23, MIN(z[2] * 20 >> 8, UINT32_MAX));
            isp_stats_put(zw, at + 69, 23, MIN(z[3] * 9 >> 8, UINT32_MAX));
            isp_stats_put(zw, at + 92, 24, MIN(z[0], UINT32_MAX));
            isp_stats_put(zw, at + 116, 16, z[1]);
        }
        if (padded) {
            isp_stats_put(w + y * stride, stride * 32 - 8, 8, frame & 0xff);
        }
    }
    addr = isp_t31_next(s, 0x1a0dc, 0x1a0cc, 0x1a0e0, true);
    R(0x1a120) = frame & 0xffff;
    if (addr) {
        isp_t31_dma(addr, w, MIN(rows * stride, ARRAY_SIZE(w)));
    }
}

static void isp_t41_adr(IngenicIspState *s, const uint8_t *pic, uint32_t qw,
                        uint32_t qh, uint32_t tag)
{
    const uint32_t rows[T41_ADR_ROWS + 1] = {
        extract32(R(0x9030), 0, 16), R(0x9030) >> 16,
        extract32(R(0x9034), 0, 16), R(0x9034) >> 16,
        extract32(R(0x9038), 0, 16),
    };
    const uint32_t cols[T41_ADR_COLS + 1] = {
        extract32(R(0x903c), 0, 16), R(0x903c) >> 16,
        extract32(R(0x9040), 0, 16), R(0x9040) >> 16,
        extract32(R(0x9044), 0, 16), R(0x9044) >> 16,
        extract32(R(0x9048), 0, 16),
    };
    uint32_t cnt[T41_ADR_ROWS * T41_ADR_COLS][T41_ADR_BINS] = { 0 };
    uint32_t lo[T41_ADR_ROWS * T41_ADR_COLS], hi[T41_ADR_ROWS * T41_ADR_COLS];
    uint64_t sum[T41_ADR_ROWS * T41_ADR_COLS] = { 0 };
    uint32_t hist[512] = { 0 };
    uint32_t w[T41_ADR_WORDS] = { tag };
    g_autofree int8_t *row = g_malloc(qh * 2);
    g_autofree int8_t *col = g_malloc(qw * 2);
    hwaddr addr;

    for (unsigned z = 0; z < T41_ADR_ROWS * T41_ADR_COLS; z++) {
        lo[z] = 0xfff;
        hi[z] = 0;
    }
    isp_t31_map(row, qh * 2, rows, T41_ADR_ROWS);
    isp_t31_map(col, qw * 2, cols, T41_ADR_COLS);
    for (uint32_t y = 0, m; y < qh; y += m) {
        const uint8_t *p = pic + y * qw;
        int r = row[y * 2];

        for (m = 1; y + m < qh && row[(y + m) * 2] == r &&
                    !memcmp(p + m * qw, p, qw); m++) {
        }
        for (uint32_t x = 0; x < qw; x++) {
            uint32_t l = isp_linear12[p[x]];
            unsigned z, b;

            hist[l >> 3] += m;
            if (r < 0 || col[x * 2] < 0) {
                continue;
            }
            z = r * T41_ADR_COLS + col[x * 2];
            b = isp_t41_adr_bin[p[x]];
            if (b < T41_ADR_BINS) {
                cnt[z][b] += m;
            }
            sum[z] += (uint64_t)l * m;
            lo[z] = MIN(lo[z], l);
            hi[z] = MAX(hi[z], l);
        }
    }
    for (unsigned z = 0; z < T41_ADR_ROWS * T41_ADR_COLS; z++) {
        uint32_t *zw = w + 2 + z * T41_ADR_ZONE_WORDS;

        for (unsigned b = 0; b < T41_ADR_BINS; b++) {
            isp_stats_put(zw, b * 18, 18, cnt[z][b]);
        }
        isp_stats_put(zw, 360, 12, hi[z]);
        isp_stats_put(zw, 372, 12, hi[z] >= lo[z] ? lo[z] : 0);
        isp_stats_put(zw, 384, 29, MIN(sum[z], UINT32_MAX));
        isp_stats_put(zw, 416, 29, MIN(sum[z], UINT32_MAX));
    }
    for (unsigned i = 0; i < 512; i++) {
        isp_stats_put(w + T41_ADR_HIST, i / 3 * 64 + i % 3 * 21, 21, hist[i]);
    }
    w[T41_ADR_WORDS - 1] = tag;
    addr = isp_t31_next(s, 0x9510, 0x9514, 0x9540, false);
    R(0x9544) = tag;
    if (addr) {
        isp_t31_dma(addr, w, T41_ADR_WORDS);
    }
}

static void isp_t41_lce_tmo_write(IngenicIspState *s, const IspT41Sums *sum,
                                  bool lce, bool tmo)
{
    /* a bank still unread is not written again */
    if (lce && !(R(0xe014) & (s->lce_bank ? BIT(8) : BIT(0)))) {
        uint32_t *bank = s->lce[s->lce_bank];

        for (unsigned b = 0; b < INGENIC_ISP_LCE_BLOCKS; b++) {
            for (unsigned i = 0; i < INGENIC_ISP_LCE_BINS; i++) {
                bank[b * INGENIC_ISP_LCE_BINS + i] = sum->lce[b][i];
            }
            bank[INGENIC_ISP_LCE_BLOCKS * INGENIC_ISP_LCE_BINS + b] =
                MIN(sum->lce[b][INGENIC_ISP_LCE_BINS], UINT32_MAX);
        }
        if (s->lce_bank) {
            R(0xe014) |= BIT(16) | BIT(8);
        } else {
            R(0xe014) = (R(0xe014) & ~BIT(16)) | BIT(0);
        }
        s->lce_bank ^= 1;
    }
    if (tmo) {
        g_autofree uint32_t *w = g_new0(uint32_t, T41_TMO_BUF / 4);
        unsigned i = (R(0x1e028) + 1) & 1;

        for (unsigned z = 0; z < T41_TMO_ROWS * T41_TMO_COLS; z++) {
            for (unsigned b = 0; b < T41_TMO_BINS; b++) {
                uint64_t y = sum->tmo[z][b][0] >> 9;

                w[z * T41_TMO_BINS + b] = MIN(y, 0xffff) |
                                          MIN(sum->tmo[z][b][1], 0xffff) << 16;
            }
        }
        R(0x1e028) = i;
        R(0x1e02c) = R(0x1e010 + i * 4);
        if (R(0x1e02c)) {
            isp_t31_dma(R(0x1e02c), w, T41_TMO_ROWS * T41_TMO_COLS *
                                       T41_TMO_BINS);
        }
    }
}

static void isp_t41_defog_write(IngenicIspState *s, const IspT41Sums *sum,
                                uint32_t tag)
{
    uint32_t w[T41_DEFOG_WORDS] = { tag };
    unsigned shift = R(0xc284) & 0xf;
    hwaddr addr;

    for (unsigned z = 0; z < T41_DEFOG_ROWS * T41_DEFOG_COLS; z++) {
        for (unsigned i = 0; i < 3; i++) {
            isp_stats_put(w + 2 + z * 2, i * 21, 21,
                          MIN(sum->defog[z][i] >> shift, UINT32_MAX));
        }
    }
    w[T41_DEFOG_WORDS - 1] = tag;
    addr = isp_t31_next(s, 0xc28c, 0xc290, 0xc2b4, false);
    R(0xc2b0) = tag;
    if (addr) {
        isp_t31_dma(addr, w, T41_DEFOG_WORDS);
    }
}

/*
 * The T41's WDR block (bit 11), as its tx-isp firmware reads it
 * (tisp_wdr_interrupt_static, tisp_wdr_get_data, tisp_wdr_process) and a
 * T41LQ with a GC5603 in its DOL mode writes it (sensor default_boot 1,
 * IMP_ISP_WDR_ENABLE first). Each frame it writes the next of 4 buffers of
 * 6 KiB from 0x6010, 0 first, puts that buffer's index at 0x6038 and its
 * address at 0x603c; 0x6030, which the driver reads the index from, stays
 * as the driver left it (3), so the driver reads the same buffer each
 * time. What the buffer holds:
 *
 * - 15 x 15 blocks (ceil(height / 15) by ceil(width / 15) pixels, the last
 *   row and column the rest), each row's 15 blocks of the long exposure
 *   then its 15 of the short one, 24 bits each, packed: the sums of their
 *   pixels' 8-bit linear luma (the driver divides them by a block's pixels,
 *   times 16, into 12 bits; 255 is 4080);
 * - at word 0x152 four 12-bit halves, the short exposure's largest values
 *   (the driver takes their mean as its white); at 0x154 two 23-bit words,
 *   how many pixels of the long exposure lie below 0x6048 bits 11:0 and at
 *   or above bits 27:16 (the driver's rate_long; a pixel counts once);
 * - from word 0x156 160 bins of 24 bits, the short exposure's 12-bit
 *   linear luma, two counts a quad, in bins 64 wide to 1792, then
 *   narrowing to 16 (measured: the knots below), the last holding the top;
 * - with 0x600c bit 4 (the driver's stat_en), from word 0x1ce the T31's 256
 *   bins of R, G and B of the long exposure and from 0x3ce of the short
 *   one; without it the hardware leaves them as they were.
 */
#define T41_WDR_BUF         0x1800
#define T41_WDR_GRID        15
#define T41_WDR_HIST_BINS   160

/* The 12-bit values and bins where the histogram's width changes */
static const uint16_t isp_t41_wdr_knot[][2] = {
    { 0, 0 }, { 1792, 28 }, { 2157, 39 }, { 3004, 91 }, { 3994, 158 },
    { 4095, 160 },
};

static unsigned isp_t41_wdr_bin(unsigned v)
{
    for (unsigned k = 1; k < ARRAY_SIZE(isp_t41_wdr_knot); k++) {
        const uint16_t *lo = isp_t41_wdr_knot[k - 1], *hi = isp_t41_wdr_knot[k];

        if (v < hi[0]) {
            return MIN(lo[1] + (v - lo[0]) * (hi[1] - lo[1]) /
                       (hi[0] - lo[0]), T41_WDR_HIST_BINS - 1);
        }
    }
    return T41_WDR_HIST_BINS - 1;
}

static void isp_t41_wdr(IngenicIspState *s, ImageSensorState *sensor,
                        uint32_t frame, uint32_t fw, uint32_t fh)
{
    uint32_t qw = fw / 2 & ~1, qh = fh / 2 & ~1;
    uint32_t bw = DIV_ROUND_UP(qw * 2, T41_WDR_GRID);
    uint32_t bh = DIV_ROUND_UP(qh * 2, T41_WDR_GRID);
    bool rgb = R(0x600c) & BIT(4);
    uint32_t below = extract32(R(0x6048), 0, 12);
    uint32_t above = extract32(R(0x6048), 16, 12);
    g_autofree uint8_t *pic = g_malloc(qw * qh * 3 / 2);
    g_autofree uint32_t *w = g_new0(uint32_t, T41_WDR_BUF / 4);
    uint64_t blk[2][T41_WDR_GRID * T41_WDR_GRID] = { { 0 } };
    uint32_t hist[T41_WDR_HIST_BINS] = { 0 };
    uint32_t rate[2] = { 0 }, most[4] = { 0 };
    unsigned i = R(0x603c) ? (R(0x6038) + 1) & 3 : 0;

    for (unsigned e = 0; e < 2; e++) {
        g_autofree uint32_t *chan = g_new0(uint32_t, 256 * 3);

        image_sensor_render_exposure(sensor, frame, qw, qh, pic,
                                     e ? IMAGE_SENSOR_SHORT :
                                     IMAGE_SENSOR_LONG);
        for (uint32_t y = 0; y < qh; y++) {
            const uint8_t *row = pic + y * qw;
            const uint8_t *uv = pic + qw * qh + y / 2 * qw;
            unsigned by = MIN(y * 2 / bh, T41_WDR_GRID - 1);

            for (uint32_t x = 0; x < qw; x++) {
                int luma = row[x];
                int u = uv[x & ~1] - 128, v = uv[x | 1] - 128;
                int c = (luma - 16) * 298 + 128;
                /* the grey at full range: the scene's white is full scale */
                int grey = MIN(MAX(c >> 8, 0), 255);
                /* Limited-range BT.601, as the scene is rendered */
                int px[3] = {
                    MIN(MAX((c + 409 * v) >> 8, 0), 255),
                    MIN(MAX((c - 100 * u - 208 * v) >> 8, 0), 255),
                    MIN(MAX((c + 516 * u) >> 8, 0), 255),
                };
                unsigned bx = MIN(x * 2 / bw, T41_WDR_GRID - 1);

                blk[e][by * T41_WDR_GRID + bx] += isp_linear[grey] * 4;
                if (!e) {
                    rate[0] += isp_linear12[grey] < below ? 4 : 0;
                    rate[1] += isp_linear12[grey] >= above ? 4 : 0;
                }
                for (unsigned k = 0; k < 3; k++) {
                    chan[isp_linear[px[k]] * 3 + k]++;
                    if (e) {
                        most[k] = MAX(most[k], isp_linear12[px[k]]);
                    }
                }
                if (e) {
                    most[3] = MAX(most[3], isp_linear12[grey]);
                    hist[isp_t41_wdr_bin(isp_linear12[grey])] += 2;
                }
            }
        }
        for (unsigned b = 0; rgb && b < 256; b++) {
            for (unsigned k = 0; k < 3; k++) {
                isp_stats_put(w + 0x1ce + e * 0x200 + b * 2, k * 21, 21,
                              chan[b * 3 + k]);
            }
        }
    }
    for (unsigned r = 0; r < T41_WDR_GRID; r++) {
        for (unsigned e = 0; e < 2; e++) {
            for (unsigned c = 0; c < T41_WDR_GRID; c++) {
                unsigned at = (r * 2 + e) * T41_WDR_GRID + c;

                isp_stats_put(w, at * 24, 24,
                              MIN(blk[e][r * T41_WDR_GRID + c], UINT32_MAX));
            }
        }
    }
    w[0x152] = most[0] | most[1] << 16;
    w[0x153] = most[2] | most[3] << 16;
    w[0x154] = MIN(rate[0], 0x7fffff);
    w[0x155] = MIN(rate[1], 0x7fffff);
    for (unsigned b = 0; b < T41_WDR_HIST_BINS; b++) {
        isp_stats_put(w + 0x156, b * 24, 24, hist[b]);
    }
    R(0x6038) = i;
    R(0x603c) = R(0x6010 + i * 4);
    if (R(0x6010 + i * 4)) {
        isp_t31_dma(R(0x6010 + i * 4), w, (rgb ? T41_WDR_BUF : 0x1ce * 4) / 4);
    }
    isp_raise(s, isp_layout(s)->pipe[0].int_sta, BIT(11));
}

/*
 * A frame's T41 sums of AF, defog, LCE, TMO and, if @gsm, GSM, in the zones
 * the maps give (-1: none), added to @sum
 */
static void isp_t41_sum(const uint8_t *ref, uint32_t fw, uint32_t fh,
                        const int8_t *afc, const int8_t *afr,
                        const int8_t *dfc, const int8_t *dfr,
                        const int8_t *lcc, const int8_t *lcr,
                        const int8_t *tmc, const int8_t *tmr,
                        unsigned af_cols, uint32_t af_bright, bool gsm,
                        bool lce, bool tmo, IspT41Sums *sum)
{
    g_autofree IspT31Span *afs = g_new(IspT31Span, fw);
    g_autofree IspT31Span *dfs = g_new(IspT31Span, fw);
    g_autofree IspT31Span *lcs = g_new(IspT31Span, fw);
    g_autofree IspT31Span *tms = g_new(IspT31Span, fw);
    g_autofree uint32_t *gsmh = g_new0(uint32_t, gsm ? 2 * 256 : 0);
    g_autofree uint32_t *lceh =
        g_new0(uint32_t, lce ? T41_LCE_ROWS * T41_LCE_COLS * 256 : 0);
    g_autofree uint32_t *tmoh =
        g_new0(uint32_t, tmo ? T41_TMO_ROWS * T41_TMO_COLS * 256 : 0);
    const uint8_t *uvp = ref + (size_t)fw * fh;
    unsigned naf, ndf, nlc, ntm;

    naf = isp_t31_spans(afc, fw, afs);
    ndf = isp_t31_spans(dfc, fw, dfs);
    nlc = isp_t31_spans(lcc, fw, lcs);
    ntm = isp_t31_spans(tmc, fw, tms);
    /*
     * GSM, LCE and TMO count by luma value, and take their bins and sums
     * of it once the frame is through
     */
    for (uint32_t y = 0, m; y < fh; y += m) {
        const uint8_t *l = ref + (size_t)y * fw;
        const uint8_t *uv = uvp + (size_t)(y / 2) * fw;
        int ar = afr[y], dr = dfr[y], lr = lcr[y], tr = tmr[y];

        /* rows alike in alike zones sum alike: m of them at once */
        for (m = 1; y + m < fh; m++) {
            if (afr[y + m] != ar || dfr[y + m] != dr || lcr[y + m] != lr ||
                tmr[y + m] != tr || memcmp(ref + (size_t)(y + m) * fw, l, fw) ||
                memcmp(uvp + (size_t)((y + m) / 2) * fw, uv, fw)) {
                break;
            }
        }
        for (unsigned i = 0; gsm && i < 2; i++) {
            isp_t41_hist(gsmh + 256 * i, l, i, fw, 2, m);
        }
        for (unsigned i = 0; ar >= 0 && i < naf; i++) {
            uint64_t o[4] = { 0 }, *z = sum->af[ar * af_cols + afs[i].zone];

            ISP_KERNEL(af_span, l, afs[i].x0, afs[i].x1, fw, af_bright, o);
            for (unsigned k = 0; k < 4; k++) {
                z[k] += o[k] * m;
            }
        }
        for (unsigned i = 0; dr >= 0 && i < ndf; i++) {
            uint64_t o[6] = { 0 }, *z = sum->defog[dr * T41_DEFOG_COLS +
                                                    dfs[i].zone];

            ISP_KERNEL(defog_span, l, uv, dfs[i].x0, dfs[i].x1, 0, 0, o);
            z[0] += o[2] * m;
            z[1] += o[1] * m;
            z[2] += o[0] * m;
        }
        for (unsigned i = 0; lr >= 0 && i < nlc; i++) {
            isp_t41_hist(lceh + (size_t)(lr * T41_LCE_COLS + lcs[i].zone) * 256,
                         l, (lcs[i].x0 + 1) & ~1u, lcs[i].x1, 2, m);
        }
        for (unsigned i = 0; tr >= 0 && i < ntm; i++) {
            isp_t41_hist(tmoh + (size_t)(tr * T41_TMO_COLS + tms[i].zone) * 256,
                         l, tms[i].x0, tms[i].x1, 1, m);
        }
        /* the vertical filters, where the rows around differ */
        for (uint32_t k = y; ar >= 0 && k < y + m; k++) {
            const uint8_t *lk = ref + (size_t)k * fw;
            const uint8_t *nb[4] = {
                ref + (size_t)(k > 1 ? k - 2 : 0) * fw,
                ref + (size_t)(k ? k - 1 : 0) * fw,
                ref + (size_t)MIN(k + 1, fh - 1) * fw,
                ref + (size_t)MIN(k + 2, fh - 1) * fw,
            };

            if (memcmp(nb[0], lk, fw) || memcmp(nb[1], lk, fw) ||
                memcmp(nb[2], lk, fw) || memcmp(nb[3], lk, fw)) {
                for (unsigned i = 0; i < naf; i++) {
                    ISP_KERNEL(af_vert, lk, nb[0], nb[1], nb[2], nb[3],
                               afs[i].x0, afs[i].x1,
                               sum->af[ar * af_cols + afs[i].zone] + 4);
                }
            }
        }
    }
    for (unsigned h = 0; gsm && h < 2; h++) {
        for (unsigned v = 0; v < 256; v++) {
            sum->gsm[h][isp_linear[v]] += gsmh[h * 256 + v];
        }
    }
    for (unsigned z = 0; lce && z < T41_LCE_ROWS * T41_LCE_COLS; z++) {
        for (unsigned v = 0; v < 256; v++) {
            uint32_t n = lceh[z * 256 + v];

            sum->lce[z][v >> 3] += n;
            sum->lce[z][INGENIC_ISP_LCE_BINS] += (uint64_t)n * v;
        }
    }
    for (unsigned z = 0; tmo && z < T41_TMO_ROWS * T41_TMO_COLS; z++) {
        for (unsigned v = 0; v < 256; v++) {
            uint32_t n = tmoh[z * 256 + v], ll = isp_log13[v];
            uint64_t *t = sum->tmo[z][ll / T41_TMO_BIN];

            t[0] += (uint64_t)n * (ll % T41_TMO_BIN);
            t[1] += n;
        }
    }
}

static void isp_stats_t41(IngenicIspState *s, ImageSensorState *sensor,
                          uint32_t frame)
{
    uint32_t af_grid = R(0x1a004);
    unsigned af_cols = MIN(af_grid >> 28, STATS_GRID_MAX);
    unsigned af_rows = MIN(extract32(af_grid, 12, 4), STATS_GRID_MAX);
    uint32_t af_x[STATS_GRID_MAX + 1] = { extract32(af_grid, 16, 12) };
    uint32_t af_y[STATS_GRID_MAX + 1] = { extract32(af_grid, 0, 12) };
    uint32_t df_x[T41_DEFOG_COLS + 1], df_y[T41_DEFOG_ROWS + 1];
    uint32_t lc_x[T41_LCE_COLS + 1], lc_y[T41_LCE_ROWS + 1];
    uint32_t tm_x[T41_TMO_COLS + 1], tm_y[T41_TMO_ROWS + 1];
    bool af = R(0x1a0cc), defog = R(0xc290), adr = R(0x9514);
    bool gsm = s->gsm_armed, lce = R(0xe000);
    bool tmo = (R(0x1e020) & 1) && R(0x1e010);
    uint32_t tag = frame & 0xff;
    uint32_t fw, fh, bits = 0;
    g_autofree uint8_t *pic = NULL;
    g_autofree int8_t *afc = NULL, *afr = NULL, *dfc = NULL, *dfr = NULL;
    g_autofree int8_t *lcc = NULL, *lcr = NULL, *tmc = NULL, *tmr = NULL;
    g_autofree IspT41Sums *sum = NULL;
    const uint8_t *ref;

    isp_frame_size(s, &fw, &fh);
    if (!fw || !fh) {
        return;
    }
    if (R(0x6010) && image_sensor_wdr(sensor) && !s->raw_in_bpp) {
        isp_t41_wdr(s, sensor, frame, fw, fh);
    }
    if (adr) {
        uint32_t qw = fw / 2 & ~1, qh = fh / 2 & ~1;
        const uint8_t *q = isp_pic_ref(s, sensor, frame, qw, qh);

        if (!q) {
            pic = g_malloc((size_t)qw * qh * 3 / 2);
            if (!isp_stats_raw_pic(s, qw, qh, pic)) {
                image_sensor_render(sensor, frame, qw, qh, pic);
            }
            isp_wb_gains(s, sensor, 0, qw, qh, pic, pic + (size_t)qw * qh);
            q = pic;
        }
        isp_t41_adr(s, q, qw, qh, tag);
        g_free(g_steal_pointer(&pic));
        bits |= BIT(9);
    }
    if (!af && !defog && !gsm && !lce && !tmo) {
        isp_raise(s, isp_layout(s)->pipe[0].int_sta, bits);
        return;
    }

    for (unsigned i = 0; i < af_cols; i++) {
        af_x[i + 1] = af_x[i] + extract32(R(0x1a008 + i / 2 * 4),
                                          i % 2 * 16, 16);
    }
    for (unsigned i = 0; i < af_rows; i++) {
        af_y[i + 1] = af_y[i] + extract32(R(0x1a028 + i / 4 * 4),
                                          i % 4 * 8, 8);
    }
    isp_t41_even(df_x, T41_DEFOG_COLS, extract32(R(0xc008), 0, 8), fw);
    isp_t41_even(df_y, T41_DEFOG_ROWS, extract32(R(0xc008), 16, 8), fh);
    isp_t41_lce_edges(lc_x, T41_LCE_COLS, extract32(R(0xe000), 0, 16),
                      extract32(R(0xe004), 0, 4), 2, fw);
    isp_t41_lce_edges(lc_y, T41_LCE_ROWS, extract32(R(0xe000), 16, 16),
                      extract32(R(0xe004), 8, 3), 1, fh);
    isp_t41_even(tm_x, T41_TMO_COLS, extract32(R(0x1e080), 8, 8), fw);
    isp_t41_even(tm_y, T41_TMO_ROWS, extract32(R(0x1e080), 0, 8), fh);
    /* the last column and row take the rest */
    tm_x[T41_TMO_COLS] = fw;
    tm_y[T41_TMO_ROWS] = fh;
    afc = g_malloc(fw);
    afr = g_malloc(fh);
    dfc = g_malloc(fw);
    dfr = g_malloc(fh);
    lcc = g_malloc(fw);
    lcr = g_malloc(fh);
    tmc = g_malloc(fw);
    tmr = g_malloc(fh);
    isp_t31_map(afc, fw, af_x, af ? af_cols : 0);
    isp_t31_map(afr, fh, af_y, af ? af_rows : 0);
    isp_t31_map(dfc, fw, df_x, defog ? T41_DEFOG_COLS : 0);
    isp_t31_map(dfr, fh, df_y, defog ? T41_DEFOG_ROWS : 0);
    isp_t31_map(lcc, fw, lc_x, lce ? T41_LCE_COLS : 0);
    isp_t31_map(lcr, fh, lc_y, lce ? T41_LCE_ROWS : 0);
    isp_t31_map(tmc, fw, tm_x, tmo ? T41_TMO_COLS : 0);
    isp_t31_map(tmr, fh, tm_y, tmo ? T41_TMO_ROWS : 0);

    ref = isp_pic_ref(s, sensor, frame, fw, fh);
    if (!ref) {
        pic = g_malloc((size_t)fw * fh * 3 / 2);
        if (!isp_stats_raw_pic(s, fw, fh, pic)) {
            image_sensor_render(sensor, frame, fw, fh, pic);
        }
        isp_wb_gains(s, sensor, 0, fw, fh, pic, pic + (size_t)fw * fh);
        ref = pic;
    }
    sum = g_new0(IspT41Sums, 1);
    isp_t41_sum(ref, fw, fh, afc, afr, dfc, dfr, lcc, lcr, tmc, tmr, af_cols,
                extract32(R(0x1a038), 8, 8), gsm, lce, tmo, sum);
    if (af) {
        isp_t41_af_write(s, sum, af_cols, af_rows, frame);
        bits |= BIT(6);
    }
    if (gsm) {
        memcpy(s->gsm, sum->gsm, sizeof(s->gsm));
        s->gsm_armed = false;
        R(0x1b004) = 1;
        R(0x1b044) = 1;
        bits |= BIT(7);
    }
    if (lce || tmo) {
        isp_t41_lce_tmo_write(s, sum, lce, tmo);
        bits |= (lce ? BIT(8) : 0) | (tmo ? BIT(17) : 0);
    }
    if (defog) {
        isp_t41_defog_write(s, sum, tag);
        bits |= BIT(10);
    }
    isp_raise(s, isp_layout(s)->pipe[0].int_sta, bits);
}

/*
 * The AWB sums are added in 1/256ths of a level. The registers' widths fit
 * 8-bit samples (a count's 13 or 14 bits times 255 fill a sum's 21 or 22),
 * and a dim scene is a few levels of them: rounding each quad to a whole
 * level, which a sensor's noise would dither away over a zone, biased a
 * colour by up to a few percent. The sum is rounded instead.
 */
/*
 * The AE and AWB sums over a quarter-size frame's rows, worked in bands of
 * them, each with sums of its own that are added up after
 */
typedef struct IspAeAwbPart {
    IspStatsSums sum;
    uint32_t cnt4[4][256];
} IspAeAwbPart;

typedef struct IspAeAwb {
    const IngenicIspStats *st;
    const uint8_t *pic, *epic, *apic;
    uint32_t qw, qh;
    bool ae, awb;
    const IspStatsGrid *ga, *gw;
    const IspT31Span *aes, *aws;
    unsigned nae, nawb;
    uint32_t dark, bright, least, most, bands;
    const uint32_t *lin;
    IspAwbWhite wreg;
    IspAeAwbPart *part;
} IspAeAwb;

static void isp_ae_awb_band(void *arg, unsigned bi, unsigned n)
{
    const IspAeAwb *a = arg;
    const IngenicIspStats *st = a->st;
    IspStatsSums *sum = &a->part[bi].sum;
    uint32_t (*cnt4)[256] = a->part[bi].cnt4;
    uint32_t qw = a->qw, qh = a->qh;
    const uint8_t *pic = a->pic, *epic = a->epic, *apic = a->apic;
    const IspStatsGrid ga = *a->ga, gw = *a->gw;
    const IspT31Span *aes = a->aes, *aws = a->aws;
    unsigned nae = a->nae, nawb = a->nawb;
    uint32_t dark = a->dark, bright = a->bright, least = a->least;
    uint32_t most = a->most, bands = a->bands;
    const uint32_t *lin = a->lin;
    const IspAwbWhite wreg = a->wreg;
    bool ae = a->ae, awb = a->awb;
    g_autofree uint8_t *aev = g_malloc(qw);
    g_autofree int32_t *cu = g_new(int32_t, 2 * qw);
    uint32_t tmp[5 * ISP_AWB_CHUNK];
    uint32_t y1 = qh * (bi + 1) / n;

    for (uint32_t y = qh * bi / n, m; y < y1; y += m) {
        const uint8_t *row = pic + y * qw;
        const uint8_t *uv = pic + qw * qh + y / 2 * qw;
        const uint8_t *erow = epic + y * qw;
        const uint8_t *arow = apic ? apic + y * qw : row;
        const uint8_t *auv = apic ? apic + qw * qh + y / 2 * qw : uv;
        int ar = ae ? ga.row[y] : -1;
        int wr = awb ? gw.row[y] : -1;

        /* the same rows in the same zones add up alike: m of them at once */
        for (m = 1; y + m < y1; m++) {
            if ((ae ? ga.row[y + m] : -1) != ar ||
                (awb ? gw.row[y + m] : -1) != wr ||
                memcmp(pic + (y + m) * qw, row, qw) ||
                memcmp(pic + qw * qh + (y + m) / 2 * qw, uv, qw) ||
                (epic != pic && memcmp(epic + (y + m) * qw, erow, qw)) ||
                (apic && (memcmp(apic + (y + m) * qw, arow, qw) ||
                          memcmp(apic + qw * qh + (y + m) / 2 * qw, auv,
                                 qw)))) {
                break;
            }
        }
        if (ae) {
            /*
             * the histogram of the values in four, so that a run of one
             * does not wait on its own count, and their exposure values
             */
            for (uint32_t x = 0; x < qw; x++) {
                cnt4[x & 3][erow[x]] += m;
            }
            image_sensor_lookup(aev, erow, qw, isp_linear);
            for (unsigned i = 0; ar >= 0 && i < nae; i++) {
                uint32_t *z = sum->ae[ar * ga.cols + aes[i].zone];
                uint32_t o[5] = { 0 };

                ISP_KERNEL(ae_span, aev, aes[i].x0, aes[i].x1, dark, bright,
                           o);
                z[0] += o[0] * m;
                z[1] += (o[2] - o[0] - o[1]) * m;
                z[2] += o[1] * m;
                z[3] += o[3] * m;
                z[4] += o[4] * m;
            }
        }
        if (nawb && wr >= 0) {
            /* a white quad needs a luma in range only where AWB has no pairs */
            int64_t lo_ly = st->awb_pairs || st->awb_zone ? -1 : least;
            int64_t hi_ly = st->awb_pairs || st->awb_zone ? INT64_MAX : most;

            ISP_KERNEL(uv_expand, auv, qw, cu, cu + qw);
            for (unsigned i = 0; i < nawb; i++) {
                unsigned zi = wr * gw.cols + aws[i].zone;
                uint64_t o[36] = { 0 };

                ISP_KERNEL(awb_span, arow, cu, cu + qw, aws[i].x0, aws[i].x1,
                           lin, (const uint32_t *)&wreg,
                           st->awb_pairs ? bands : 0, st->awb_pairs ? 4 : 1,
                           lo_ly, hi_ly, o, tmp);
                for (unsigned c = 0; c < 4; c++) {
                    sum->awb_all[zi][c] += o[c] * m;
                    for (unsigned b = 0; b < (st->awb_pairs ? 4u : 1u); b++) {
                        if (st->awb_pairs) {
                            sum->awb_band[zi][b][c] += o[4 + 4 * b + c] * m;
                            sum->awb_white[zi][b][c] += o[20 + 4 * b + c] * m;
                        } else {
                            sum->awb[zi][c] += o[20 + 4 * b + c] * m;
                        }
                    }
                }
            }
        }
    }
}

static void isp_stats_add(IspStatsSums *d, const IspStatsSums *a)
{
    uint32_t *dw = (uint32_t *)d->ae, *aw = (uint32_t *)a->ae;
    uint64_t *dq, *aq;

    for (unsigned k = 0; k < sizeof(d->ae) / 4; k++) {
        dw[k] += aw[k];
    }
    dq = (uint64_t *)d->awb;
    aq = (uint64_t *)a->awb;
    for (unsigned k = 0; k < sizeof(d->awb) / 8; k++) {
        dq[k] += aq[k];
    }
    dq = (uint64_t *)d->awb_all;
    aq = (uint64_t *)a->awb_all;
    for (unsigned k = 0; k < sizeof(d->awb_all) / 8; k++) {
        dq[k] += aq[k];
    }
    dq = (uint64_t *)d->awb_band;
    aq = (uint64_t *)a->awb_band;
    for (unsigned k = 0; k < sizeof(d->awb_band) / 8; k++) {
        dq[k] += aq[k];
    }
    dq = (uint64_t *)d->awb_white;
    aq = (uint64_t *)a->awb_white;
    for (unsigned k = 0; k < sizeof(d->awb_white) / 8; k++) {
        dq[k] += aq[k];
    }
    for (unsigned k = 0; k < 256; k++) {
        d->hist[k] += a->hist[k];
    }
}

/* @a over its rows, in bands as the frame's size allows, into @sum */
static void isp_ae_awb_sums(IngenicIspState *s, IspAeAwb *a, IspStatsSums *sum)
{
    unsigned bands = (size_t)a->qw * a->qh < MDNS_BAND_PIXELS ? 1 :
                     isp_nbands(s);
    g_autofree IspAeAwbPart *part = g_new0(IspAeAwbPart, bands);

    a->part = part;
    isp_par(s, bands, isp_ae_awb_band, a);
    for (unsigned i = 0; i < bands; i++) {
        for (unsigned v = 0; v < 256; v++) {
            part[i].sum.hist[isp_linear[v]] += part[i].cnt4[0][v] +
                part[i].cnt4[1][v] + part[i].cnt4[2][v] + part[i].cnt4[3][v];
        }
        isp_stats_add(sum, &part[i].sum);
    }
}

static void isp_awb_sums_round(IspStatsSums *sum, unsigned zones)
{
    for (unsigned z = 0; z < zones; z++) {
        for (unsigned i = 0; i < 3; i++) {
            sum->awb[z][i] = (sum->awb[z][i] + 128) >> 8;
            sum->awb_all[z][i] = (sum->awb_all[z][i] + 128) >> 8;
            for (unsigned b = 0; b < 4; b++) {
                sum->awb_band[z][b][i] = (sum->awb_band[z][b][i] + 128) >> 8;
                sum->awb_white[z][b][i] =
                    (sum->awb_white[z][b][i] + 128) >> 8;
            }
        }
    }
}

/* One pass over the frame's quads for both blocks */
static void isp_stats_ae_awb(IngenicIspState *s, ImageSensorState *sensor,
                             uint32_t frame, unsigned pipe)
{
    const IngenicIspStats *st = isp_layout(s)->stats;
    hwaddr ab = st->ae + pipe * st->sec, wb = st->awb + pipe * st->sec;
    uint32_t qw = (R(st->size) >> 16) / 2 & ~1;
    uint32_t qh = (R(st->size) & 0xffff) / 2 & ~1;
    bool ae = st->older || R(st->ae_en ?: ab) & 1;
    bool awb = st->awb && (st->older || R(st->awb_en ?: wb) & 1);
    uint32_t dark = extract32(R(ab + 0x28), 0, 8);
    uint32_t bright = extract32(R(ab + 0x28), 8, 8);
    uint32_t least = extract32(R(wb + 0x38), 0, 8);
    uint32_t most = extract32(R(wb + 0x38), 8, 8);
    g_autofree IspStatsSums *sum = NULL;
    g_autofree uint8_t *pic = NULL;
    IspStatsGrid ga, gw;
    const IspAwbWhite wreg = {
        R(wb + 0x28), R(wb + 0x2c), R(wb + 0x30), R(wb + 0x34),
    };
    uint32_t bands = R(wb + 0x38);
    g_autofree uint8_t *apic = NULL, *gpic = NULL;
    g_autofree IspT31Span *aws = NULL;
    uint32_t lin[3 * 256];
    g_autofree IspT31Span *aes = NULL;
    unsigned nae;
    unsigned nawb;
    const uint8_t *epic;
    double wf[3];
    /* the sensor's response on the factors' 16 fractional bits */
    uint64_t raw_r = 65536, raw_g = 65536, raw_b = 65536;
    ImageSensorExposureSel sel = IMAGE_SENSOR_FUSED;
    bool stats_raw;
    double rf[3];

    if ((!ae && !awb) || !qw || !qh) {
        return;
    }
    pic = g_malloc(qw * qh * 3 / 2);
    stats_raw = isp_stats_raw_pic(s, qw, qh, pic);
    if (!stats_raw) {
        sel = isp_t31_wdr(s, sensor) ? IMAGE_SENSOR_LONG : IMAGE_SENSOR_FUSED;

        image_sensor_render_exposure(sensor, frame, qw, qh, pic, sel);
        /*
         * AWB's picture leaves the light's tint out and puts the response
         * on the grey: the tint is rounded into the 8-bit chroma, which a
         * dim picture has too little of for the ratios of its colours
         */
        if (awb && isp_layout(s)->raw_part && image_sensor_tinted(sensor) &&
            image_sensor_raw_response(sensor, isp_layout(s)->raw_part, rf)) {
            apic = g_malloc(qw * qh * 3 / 2);
            image_sensor_render_untinted(sensor, frame, qw, qh, apic, sel);
            raw_r = llround(rf[0] * 65536);
            raw_g = llround(rf[1] * 65536);
            raw_b = llround(rf[2] * 65536);
        }
    }
    /* an RDMA frame's samples are the raw signal already: no sensor response */
    if (!s->raw_in_bpp && !apic) {
        isp_raw_factors(s, sensor, rf);
        raw_r = llround(rf[0] * 65536);
        raw_b = llround(rf[2] * 65536);
    }
    /*
     * AE sees the picture after the guest's gains (the AWB sums, from the
     * ungained signal, do not)
     */
    epic = pic;
    if (ae && isp_wb_factors(s, sensor, pipe, wf)) {
        /* what the other passes keep, when pic is the plain draw of it */
        bool share = !pipe && !stats_raw && isp_pic_plain(s) &&
                     sel == IMAGE_SENSOR_FUSED;

        IngenicIspPic *e = share ? isp_pic_find(s, sensor, frame, qw, qh) :
                           NULL;

        if (!e && share) {
            e = isp_pic_slot(s, sensor, frame, qw, qh);
            memcpy(e->buf, pic, (size_t)qw * qh * 3 / 2);
            ingenic_isp_wb_picture(e->buf, e->buf + (size_t)qw * qh, qw, qh,
                                   wf);
            e->valid = true;
        } else if (!e) {
            gpic = g_malloc((size_t)qw * qh * 3 / 2);
            memcpy(gpic, pic, (size_t)qw * qh * 3 / 2);
            ingenic_isp_wb_picture(gpic, gpic + (size_t)qw * qh, qw, qh, wf);
        }
        epic = e ? e->buf : gpic;
    }
    sum = g_new0(IspStatsSums, 1);
    if (st->ae_zone) {
        isp_stats_grid_even(s, st->ae_zone, &ga, qw, qh, STATS_GRID_MAX);
    } else {
        isp_stats_grid(s, ab, &ga, qw, qh);
    }
    if (st->awb_zone) {
        isp_stats_grid_even(s, st->awb_zone, &gw, qw, qh,
                            st->flat ? FLAT_AWB_MAX : STATS_GRID_MAX);
    } else {
        isp_stats_grid(s, wb, &gw, qw, qh);
    }
    /*
     * AWB's colours in linear light by value, and where its zones' columns
     * run: its sums go by the span, as the defog's do
     */
    for (unsigned v = 0; v < 256; v++) {
        lin[v] = MIN((isp_linear_fine[v] * raw_r + 32768) >> 16, 255 * 256);
        lin[256 + v] = MIN((isp_linear_fine[v] * raw_g + 32768) >> 16,
                           255 * 256);
        lin[512 + v] = MIN((isp_linear_fine[v] * raw_b + 32768) >> 16,
                           255 * 256);
    }
    aws = g_new(IspT31Span, qw);
    nawb = awb ? isp_t31_spans(gw.col, qw, aws) : 0;
    aes = g_new(IspT31Span, qw);
    nae = ae ? isp_t31_spans(ga.col, qw, aes) : 0;
    isp_ae_awb_sums(s, &(IspAeAwb){
        .st = st, .pic = pic, .epic = epic, .apic = apic, .qw = qw, .qh = qh,
        .ae = ae, .awb = awb, .ga = &ga, .gw = &gw, .aes = aes, .aws = aws,
        .nae = nae, .nawb = nawb, .dark = dark, .bright = bright,
        .least = least, .most = most, .bands = bands, .lin = lin,
        .wreg = wreg,
    }, sum);
    isp_awb_sums_round(sum, AWB_ZONES);
    if (ae && st->flat) {
        isp_ae_write_flat(s, pipe, ab, frame, sum, &ga);
    } else if (ae) {
        isp_ae_write(s, pipe, ab, st->int_ae_zones | st->int_ae_hist, sum,
                     &ga);
    }
    if (awb && st->flat) {
        isp_awb_write_flat(s, pipe, frame, sum, &gw);
    } else if (awb && st->awb_zone) {
        isp_awb_write_t32(s, pipe, sum, &gw);
    } else if (awb && st->awb_pairs) {
        isp_awb_write_pairs(s, pipe, sum, &gw);
    } else if (awb) {
        isp_awb_write(s, pipe, sum, &gw);
    }
    isp_stats_grid_free(&ga);
    isp_stats_grid_free(&gw);
}

static void isp_stats(IngenicIspState *s, ImageSensorState *sensor,
                      uint32_t frame, unsigned pipe)
{
    isp_stats_ae_awb(s, sensor, frame, pipe);
    if (isp_layout(s)->stats->t31) {
        isp_stats_t31(s, sensor, frame);
    }
    if (isp_layout(s)->stats->t41) {
        isp_stats_t41(s, sensor, frame);
    }
}

/* A VIC on MIPI takes its frames from a CSI receiver out of reset */
static bool isp_csi_held(IngenicIspState *s, unsigned vic)
{
    const IngenicIspLayout *l = isp_layout(s);

    return s->csi_reset && l->vic_mode &&
           VIC(vic, l->vic_mode) == VIC_MODE_MIPI;
}

/* VIC 0 drives the board's MIPI switch, rather than the switch's GPIOs */
static bool isp_vic_switching(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);

    return s->switched && l->vic_switch && (VIC(0, l->vic_switch) & 1);
}

/* A T32 VIC's switch: on, it takes inputs n and n + 2 in turn */
static bool isp_vic_two(IngenicIspState *s, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);

    return l->vic_switch_on &&
           (VIC(n, l->vic_switch) & l->vic_switch_on) == l->vic_switch_on;
}

/* The inputs whose sensors send frames into running VICs */
static uint32_t isp_turn_inputs(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);
    uint32_t in = 0;

    for (unsigned n = 0; n < l->vics; n++) {
        if (!(s->vic_running & BIT(n))) {
            continue;
        }
        for (unsigned k = 0; k <= isp_vic_two(s, n); k++) {
            if (image_sensor_find(n + k * l->vics)) {
                in |= BIT(n + k * l->vics);
            }
        }
    }
    return in;
}

/* The sensors on the inputs */
static unsigned isp_sensors(IngenicIspState *s)
{
    unsigned n = 0;

    for (unsigned in = 0; in < INGENIC_ISP_SENSORS; in++) {
        n += !!image_sensor_find(in);
    }
    return n;
}

/* Several sensors on a T32: its core takes their frames in turn */
static bool isp_turns(IngenicIspState *s)
{
    return isp_layout(s)->vic_turns && ctpop32(isp_turn_inputs(s)) > 1;
}

/* The input whose frame the core takes next */
static unsigned isp_next_turn(IngenicIspState *s)
{
    uint32_t inputs = isp_turn_inputs(s);
    unsigned in = s->core_turn % INGENIC_ISP_SENSORS;

    while (inputs && !(inputs & BIT(in))) {
        in = (in + 1) % INGENIC_ISP_SENSORS;
    }
    return inputs ? in : 0;
}

/*
 * VIC n's sensor: the one on input n, or a board's only one on input 0.
 * A T32's VIC n sees input n, or n + 2 when its switch has turned there.
 * Behind a MIPI switch VIC 0 sees the input the switch selects, as a T23
 * with two or three sensors flips it after every frame
 */
static ImageSensorState *isp_vic_sensor(IngenicIspState *s, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);

    if (l->vic_turns) {
        return image_sensor_find(n + s->vic[n].sub * l->vics);
    }
    if (s->switched && !n) {
        return image_sensor_find(isp_vic_switching(s) ? s->vic_switch
                                                       : s->mipi_switch);
    }
    if (image_sensor_find(n) || !n) {
        return image_sensor_find(n);
    }
    for (unsigned i = 1; i < IMAGE_SENSOR_INPUTS; i++) {
        if (image_sensor_find(i)) {
            return NULL;
        }
    }
    return image_sensor_find(0);
}

/*
 * Behind a MIPI switch the driver counts frames twice, in the VIC's
 * interrupt (where it flips the switch's GPIOs) and in the core's "all
 * done" (where it picks the next sensor's buffers): a frame one count
 * misses puts every later frame in the other sensor's buffers, and leaves
 * a done entry no buffer list expects. Silicon serves both and queues the
 * next sensor's buffers long before its frame; a TCG guest can stall
 * longer than a frame, or clear the interrupts and only then queue the
 * buffers. So VIC 0 holds the next frame until the last one's interrupts
 * are served, the driver has set each channel up for the next sensor (it
 * enables or disables each in turn, writing its Y push FIFO clear, after
 * it queues the channel's buffer) and each enabled channel has its
 * buffers, or ten frame times after the serving (a channel the driver
 * skips stays empty; a loaded host has stalled a guest for most of a
 * second there, and a frame let go early jams the driver's lists).
 *
 * Over IVDC the encoder has to start on a frame within some 500 lines of
 * it, the next sensor's frame following on the ring, which silicon's
 * encoder, done with the last frame long before, always does; the model's
 * codes in host time, behind which a TCG guest's next job starts late and
 * its driver cancels it. So VIC 0 also holds its next frame while an
 * encoder codes the last one sent over IVDC, for up to a frame time, or
 * the IVDC holds it for one to take ("hold-lines").
 */
#define SWITCH_HOLD         10

static const char *isp_frame_held(IngenicIspState *s, IngenicIspVic *v,
                                  int64_t period)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspVicInt *vi = l->vic_int;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (!v->n && !isp_turns(s)) {
        bool busy = s->ivdc_coding || s->ivdc_waiting;

        if (busy && !s->ivdc_held) {
            s->ivdc_held = now;
        }
        /* the IVDC waits 50 ms at most, then the guest clears its overflow */
        if (busy && now < s->ivdc_held + (s->ivdc_coding ? period
                                                         : 4 * period)) {
            return s->ivdc_coding ? "the encoder coding the last IVDC frame"
                                  : "the last IVDC frame not taken";
        }
        s->ivdc_held = 0;
    }
    if (!s->switched || v->n) {
        return NULL;
    }
    if ((VIC(0, vi->sta) & ~VIC(0, vi->msk) & VIC_INT_FRAME_DONE) ||
        (R(l->tick.sta) & R(l->tick.en) & (l->tick.bits | l->tick.start))) {
        v->served = 0;
        return "the last frame's interrupts unserved";
    }
    if (!v->served) {
        v->served = now;
    }
    if (s->chans_set != MAKE_64BIT_MASK(0, INGENIC_ISP_CHANS) &&
        now < v->served + SWITCH_HOLD * period) {
        return "the next channels not set up";
    }
    for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
        IngenicIspChan *ch = &s->chan[0][n];

        if ((R(l->pipe[0].ch_en) & BIT(n)) &&
            (!ch->y_n || !ch->uv_n || ch->done_n) &&
            now < v->served + SWITCH_HOLD * period) {
            return "the next buffers unqueued";
        }
    }
    v->served = 0;
    return NULL;
}

/* Why VIC @v takes no frame, NULL when it does: traced as it changes */
static void isp_vic_wait(IngenicIspVic *v, const char *why)
{
    if (why != v->waiting) {
        v->waiting = why;
        if (why) {
            trace_ingenic_isp_vic_wait(v->n, why);
        }
    }
}

static void isp_latch(IngenicIspState *s, uint32_t when);
static void isp_shadow_ram_run(IngenicIspState *s);
static void isp_shadow_run(IngenicIspState *s, uint32_t n);

/*
 * The T41's core, stopped by an idle request or 0x24 bit 2 until a latch,
 * or restarted during the frame now ending (one of @period, which reached
 * it 1/16 in)
 */
static bool isp_core_stopped(IngenicIspState *s, int64_t period)
{
    return s->core_idle || s->core_wait ||
           (s->core_resumed &&
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - period + period / 16 <
            s->core_resumed);
}

static void isp_core_resume(IngenicIspState *s)
{
    R(isp_layout(s)->latch_now) = 0;
    s->core_wait = false;
    s->core_resumed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

/* The bits a sample of the sensor on VIC @n carries */
static unsigned isp_vic_bits(IngenicIspState *s, unsigned n)
{
    const IngenicIspLayout *l = isp_layout(s);
    static const uint8_t mipi[8] = { 8, 10, 12, 8, 8, 8, 8, 16 };

    if ((VIC(n, l->vic_mode ?: VIC_INPUT) & 7) == VIC_MODE_DVP) {
        switch (extract32(VIC(n, VIC_DVP_FMT), 17, 3)) {
        case 1:
            return 10;
        case 2:
            return 12;
        default:
            return 8;
        }
    }
    return mipi[VIC(n, l->vic_mipi_fmt ?: VIC_MIPI_FMT) & 7];
}

/* The head of a VIC's queue for plane @pl is filled: on to its done FIFO */
static void isp_mdma_fifo_done(IngenicIspState *s, unsigned n, unsigned pl)
{
    uint32_t *q = s->mdma_q[n * 2 + pl], addr = q[0];

    memmove(q, q + 1, (s->mdma_qn[n][pl] - 1) * sizeof(addr));
    s->mdma_qn[n][pl]--;
    if (s->mdma_dn[n][pl] < INGENIC_ISP_MDMA_FIFO) {
        s->mdma_done[n * 2 + pl][s->mdma_dn[n][pl]++] = addr;
    }
}

/*
 * VIC @v's raw capture: the frame as the sensor sent it, into the next of
 * the five slots, while the control's count allows (bit 19: every frame,
 * else the count's low bits, down one a frame). The T41's tx-isp leaves the
 * count 0 and writes 1 to MDMA_KICK for each buffer it queues (qbuf,
 * streamon, the frame done interrupt): one frame each.
 */
static void isp_mdma_frame(IngenicIspState *s, IngenicIspVic *v,
                           ImageSensorState *sensor)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr m = l->mdma;
    uint32_t ctrl = VIC(v->n, m + MDMA_CTRL), count = extract32(ctrl, 16, 4);
    uint32_t w = VIC(v->n, m + MDMA_SIZE) >> 16;
    uint32_t h = VIC(v->n, m + MDMA_SIZE) & 0xffff;
    uint32_t ys = VIC(v->n, m + MDMA_Y_STRIDE) & 0xffff;
    uint32_t uvs = VIC(v->n, m + MDMA_UV_STRIDE) & 0xffff;
    unsigned fmt = ctrl & 7, slot = s->mdma_slot[v->n];
    bool planes = fmt == MDMA_NV12 || fmt == MDMA_NV21;
    uint32_t y = VIC(v->n, m + MDMA_Y + 4 * slot);
    uint32_t uv = VIC(v->n, m + MDMA_UV + 4 * slot);

    if (l->mdma_fifo) {
        y = s->mdma_qn[v->n][0] ? s->mdma_q[v->n * 2][0] : 0;
        uv = s->mdma_qn[v->n][1] ? s->mdma_q[v->n * 2 + 1][0] : 0;
        if (planes && !uv) {
            return;
        }
    }
    if (!(ctrl & MDMA_ON) || (!count && !s->mdma_kick[v->n]) || !w || !h) {
        return;
    }
    /*
     * Not measured: the driver zeroes the slots and fills each as a buffer
     * is queued, in the order the capture takes them; the model waits on
     * an empty slot rather than write a frame to address 0
     */
    if (!y) {
        return;
    }
    if (fmt == MDMA_RAW16) {
        g_autofree uint16_t *raw = g_malloc((size_t)w * h * 2);
        uint16_t keep = ~((1u << (16 - isp_vic_bits(s, v->n))) - 1);

        image_sensor_render_bayer(sensor, v->frame, w, h, raw);
        for (size_t i = 0; i < (size_t)w * h; i++) {
            raw[i] = cpu_to_le16(raw[i] & keep);
        }
        isp_dma_plane(s, y, (uint8_t *)raw, w * 2, 0, 0, w * 2, h, ys);
    } else if (fmt == MDMA_NV12 || fmt == MDMA_NV21) {
        g_autofree uint8_t *pic = g_malloc((size_t)w * h * 3 / 2);
        uint8_t *c = pic + (size_t)w * h;

        image_sensor_render_planes(sensor, v->frame, w, h, pic, c);
        for (size_t i = 0; fmt == MDMA_NV21 && i + 1 < (size_t)w * (h / 2);
             i += 2) {
            uint8_t t = c[i];

            c[i] = c[i + 1];
            c[i + 1] = t;
        }
        isp_dma_plane(s, y, pic, w, 0, 0, w, h, ys);
        isp_dma_plane(s, uv, c, w, 0, 0, w, h / 2, uvs);
    } else {
        qemu_log_mask(LOG_UNIMP, "ingenic-isp: VIC %u raw capture format "
                      "%u\n", v->n, fmt);
        return;
    }
    if (!count) {
        s->mdma_kick[v->n]--;
    } else if (!(ctrl & MDMA_ALL)) {
        VIC(v->n, m + MDMA_CTRL) = deposit32(ctrl, 16, 4, count - 1);
    }
    trace_ingenic_isp_mdma(v->n, v->frame, slot, y, fmt,
                           !count ? s->mdma_kick[v->n] :
                           ctrl & MDMA_ALL ? 8 : count - 1);
    if (l->mdma_fifo) {
        isp_mdma_fifo_done(s, v->n, 0);
        if (planes) {
            isp_mdma_fifo_done(s, v->n, 1);
        }
    } else {
        VIC(v->n, m + MDMA_DONE) = y;
        s->mdma_slot[v->n] = (slot + 1) % MDMA_SLOTS;
    }
    isp_raise(s, l->vic + v->n * l->vic_stride + l->vic_int->sta + 4,
              VIC_INT_MDMA | (l->mdma_fifo && planes ? VIC_INT_MDMA_UV : 0));
}

/*
 * VIC @v's frame as the sensor sent it, to the frame tap: Bayer, RGGB, the
 * sensor's bits at the top of 16 as the raw capture writes them
 */
static void isp_tap_raw(IngenicIspState *s, IngenicIspVic *v,
                        ImageSensorState *sensor)
{
    const IngenicIspLayout *l = isp_layout(s);
    uint32_t size = VIC(v->n, 0x004);
    uint32_t w = (size >> 16) & ~1, h = (size & 0xffff) & ~1;
    uint16_t keep = ~((1u << (16 - isp_vic_bits(s, v->n))) - 1);
    g_autofree uint16_t *raw = NULL;
    g_autofree char *key = g_strdup_printf("raw-%u", v->n);

    if (!w || !h || !ingenic_frame_tap_ready(&s->tap, key, w, h,
                                              "bayer_rggb16le")) {
        return;
    }
    raw = g_malloc((size_t)w * h * 2);
    image_sensor_render_bayer(sensor, v->frame, w, h, raw);
    for (size_t i = 0; i < (size_t)w * h; i++) {
        raw[i] &= keep;
    }
    ingenic_frame_tap_raw(&s->tap, v->n, w, h, "bayer_rggb16le", raw);
    ingenic_frame_tap_done(&s->tap);
}

/* The raw WDMA: a frame of @sensor into the buffer at the head of its queue */
static void isp_raw_wdma_frame(IngenicIspState *s, ImageSensorState *sensor,
                               uint32_t frame)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr base = l->raw_dma + RAW_WDMA;
    uint32_t w, h, addr = s->wdma_q[0], stride = R(base + RAW_WDMA_STRIDE);
    uint32_t cfg = R(base + RAW_WDMA_CFG);
    uint32_t bpp = s->wdma_raw ? 1 + extract32(cfg, 16, 1) : 1;
    g_autofree uint8_t *pic = NULL;

    if (!s->wdma_n || !isp_raw_size(s, &w, &h)) {
        return;
    }
    stride = MAX(stride, w * bpp);
    if (s->wdma_raw) {
        g_autofree uint16_t *raw = g_malloc((size_t)w * h * 2);
        uint16_t keep = ~((1u << (16 - isp_vic_bits(s, 0))) - 1);

        image_sensor_render_bayer(sensor, frame, w, h, raw);
        pic = g_malloc((size_t)w * h * bpp);
        for (size_t i = 0; i < (size_t)w * h; i++) {
            uint16_t v = raw[i] & keep;

            if (bpp == 2) {
                stw_le_p(pic + 2 * i, v);
            } else {
                pic[i] = v >> 8;
            }
        }
    } else {
        pic = g_malloc0((size_t)w * h);
    }
    isp_dma_plane(s, addr, pic, w * bpp, 0, 0, w * bpp, h, stride);
    trace_ingenic_isp_raw_wdma(frame, addr, bpp, s->wdma_raw, s->wdma_n - 1);
    s->wdma_head = addr;
    memmove(s->wdma_q, s->wdma_q + 1, (s->wdma_n - 1) * sizeof(addr));
    s->wdma_n--;
    if (s->wdma_done_n < INGENIC_ISP_RAW_FIFO) {
        s->wdma_done[s->wdma_done_n++] = addr;
    }
    isp_raise(s, l->pipe[0].int_sta, RAW_INT_WDMA);
    if (s->wdma_raw) {
        isp_tick(s, &l->tick, RAW_TICK_WDMA);
    }
    isp_tick(s, &l->dma, RAW_DMA_WDMA);
}

/*
 * A VIC frame in the T41's raw loop: held, or into the WDMA; true when it
 * goes no further, with the RDMA (once started) the core's input
 */
static bool isp_raw_vic_frame(IngenicIspState *s, IngenicIspVic *v,
                              ImageSensorState *sensor)
{
    const IngenicIspLayout *l = isp_layout(s);

    if (!l->raw_dma || v->n) {
        return false;
    }
    if (VIC(v->n, l->vic_hold) & RAW_VIC_HOLD) {
        return true;
    }
    isp_raw_wdma_frame(s, sensor, v->frame);
    return s->rdma_start && R(l->raw_dma + RAW_RDMA + RAW_RDMA_MODE);
}

/*
 * A frame of @sensor (its @frame) through the core into pipeline @p, from
 * VIC @vn; @waited: the VIC held it, and it takes the channels enabled now
 */
static void isp_core_frame(IngenicIspState *s, ImageSensorState *sensor,
                           uint32_t frame, unsigned vn, unsigned p,
                           bool waited)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspPipe *pipe = &l->pipe[p];
    uint32_t chans = R(pipe->ch_en), done = 0;
    const char *stalled;

    /* the MSCA runs on what latches as the frame starts, stalled or not */
    isp_latch(s, LATCH_START);
    stalled = !p && !isp_core_keeps(s, sensor) ? "the core's input overflows" :
              isp_overload(s, sensor, p);
    if (stalled) {
        trace_ingenic_isp_stall(p, frame, stalled);
        if (l->err_sta) {
            isp_raise(s, l->err_sta, l->err_overflow);
            isp_update(s);
        } else {
            isp_raise(s, pipe->int_sta, l->stall_sta);
        }
        if (l->stall_vic) {
            isp_raise(s, l->vic + vn * l->vic_stride + l->vic_int->sta,
                      l->stall_vic);
        }
        return;
    }
    s->pic_scope = true;
    if (!p) {
        isp_mdns_frame(s, sensor, frame);
    }
    isp_raise(s, pipe->int_sta, l->int_frame_start);
    isp_tick(s, &l->tick, l->tick.start);
    /*
     * The frame goes out on the channels enabled as it reached the MSCA; a
     * frame that waited (as a VIC's first does) takes those enabled now.
     * So does one behind a MIPI switch, which goes at the guest's pace
     * (isp_frame_held()): the driver enables the next sensor's channels as
     * it serves the last frame, which a TCG guest does after the next has
     * reached the MSCA.
     */
    if (pipe->busy || l->chans_at_start) {
        isp_msca_sync(s, p);
        if (l->chans_at_start && s->msca_latched[p] && !waited &&
            !(s->switched && !vn)) {
            chans = s->msca_chans[p];
        }
    }
    if (chans & BIT(0)) {
        isp_raise(s, pipe->int_sta, l->int_ch0);
    }
    for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
        if (!(chans & BIT(n))) {
            continue;
        }
        /* a frame it drops goes nowhere: no buffer, no done, still busy */
        if ((s->drop_decided[p] & BIT(n)) ? s->drop_skip[p] & BIT(n) :
            !isp_drop_take(s, p, n)) {
            trace_ingenic_isp_chan_drop(p, n, frame, "frame drop");
            continue;
        }
        if (isp_chan_frame(s, sensor, frame, p, n)) {
            done |= BIT(n);
        }
    }
    s->drop_decided[p] = 0;
    /* a channel that put the frame out goes idle, one that dropped it not */
    if (pipe->busy || l->chans_at_start) {
        if (pipe->busy && l->busy_held) {
            /* msca_chans keeps those that dropped it, busy until the next */
            chans &= MAKE_64BIT_MASK(0, INGENIC_ISP_CHANS);
            s->msca_chans[p] = chans & ~done;
            R(pipe->busy) = chans & (R(pipe->ch_en) | ~done);
        } else if (pipe->busy) {
            R(pipe->busy) &= ~done;
        }
        s->msca_latched[p] = false;
        s->msca_busy_at[p] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                             NANOSECONDS_PER_SECOND / sensor->fps / 16;
        s->ch0_span = NANOSECONDS_PER_SECOND / sensor->fps * 15 / 16;
    }
    if (l->stats && (!p || l->stats->sec)) {
        isp_stats(s, sensor, frame, p);
    }
    isp_latch(s, LATCH_END);
    isp_shadow_ram_run(s);
    if (!vn) {
        /* the next frame reaches the MSCA ~1/16 of a frame time on */
        s->shadow_idle_until = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                               NANOSECONDS_PER_SECOND / sensor->fps / 16;
        if (s->shadow_due) {
            isp_shadow_run(s, s->shadow_due);
        }
    }
    isp_raise(s, pipe->int_sta, l->int_frame_end);
    /* the RDMA's frame ends without the bits the WDMA raised for it */
    isp_tick(s, &l->tick, l->tick.bits & ~(s->raw_in_bpp ? RAW_TICK_WDMA : 0));
    isp_tick(s, &l->dma, l->dma.bits);
    s->pic_scope = false;
    for (unsigned i = 0; i < ARRAY_SIZE(s->pic); i++) {
        s->pic[i].valid = false;
    }
}

/*
 * The raw RDMA takes the frame at @addr in as the core's: through it as a
 * frame of the VIC's sensor, its picture the buffer's raw samples
 */
static void isp_raw_rdma_frame(IngenicIspState *s, uint32_t addr)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr base = l->raw_dma + RAW_RDMA;
    ImageSensorState *sensor = isp_vic_sensor(s, 0);

    if (!sensor || s->gated) {
        return;
    }
    s->raw_in_addr = addr;
    s->raw_in_stride = R(base + RAW_RDMA_STRIDE);
    s->raw_in_bpp = 1 + extract32(R(base + RAW_RDMA_FMT), 0, 1);
    trace_ingenic_isp_raw_rdma(addr, s->vic[0].frame, s->raw_in_bpp);
    isp_core_frame(s, sensor, s->vic[0].frame, 0, 0, false);
    s->raw_in_bpp = 0;
    g_free(s->raw_in_v8);
    s->raw_in_v8 = NULL;
    if (s->rdma_done_n < INGENIC_ISP_RAW_FIFO) {
        s->rdma_done[s->rdma_done_n++] = addr;
    }
    isp_tick(s, &l->dma, RAW_DMA_RDMA);
    isp_update(s);
}

/*
 * Several sensors on a T32 (tx-isp's direct_mode 2 to 4): each VIC passes
 * its sensors' frames into the core's input cache, and the core takes one
 * sensor's frame at a time, the sensors in turn as the driver counts them
 * (its mult_id table: 0, 1, 2, 3, 0, ...), each once the driver has given
 * the channels that sensor's buffers, loaded its register list and written
 * the kick (0x2c bit 0). A frame that comes before its sensor's turn waits
 * there, a later one of the same sensor in its place. The VIC's interrupt
 * says which of its inputs sent the frame (0x20c, bits 2:0 at its start and
 * 10:8 at its end, which the driver only tests for 0). The cache's own
 * registers (0x1080 to 0x10c8) are not modelled.
 */
static void isp_vic_turn(IngenicIspState *s, IngenicIspVic *v,
                         ImageSensorState *sensor)
{
    const IngenicIspLayout *l = isp_layout(s);
    unsigned in = sensor->input;

    VIC(v->n, l->vic_sub) = v->sub | v->sub << 8;
    s->in_frame[in]++;
    s->ipc_ready |= BIT(in);
    trace_ingenic_isp_frame(v->n, s->in_frame[in], 0, in);
    isp_raise(s, l->vic + v->n * l->vic_stride + l->vic_int->sta,
              l->vic_frame_start | VIC_INT_FRAME_DONE);
    /* the switch turns to the other input once each frame, if it has one */
    if (isp_vic_two(s, v->n) && image_sensor_find(v->n + !v->sub * l->vics)) {
        v->sub = !v->sub;
    } else if (!isp_vic_two(s, v->n)) {
        v->sub = 0;
    }
    isp_update(s);
}

static void isp_core_turn(IngenicIspState *s)
{
    uint32_t inputs = isp_turn_inputs(s);
    unsigned in = isp_next_turn(s);

    if (!s->core_kicked || !inputs || s->gated) {
        return;
    }
    if (!(s->ipc_ready & BIT(in))) {
        return;
    }
    /*
     * The ring holds one frame: the next waits for an encoder to take the
     * last, as VIC 0's frames do with one sensor (isp_frame_held())
     */
    if (s->ivdc_waiting || s->ivdc_coding) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t period = NANOSECONDS_PER_SECOND / image_sensor_find(in)->fps;

        if (!s->ivdc_held) {
            s->ivdc_held = now;
        }
        if (now < s->ivdc_held + 4 * period) {
            timer_mod(s->core_timer, now + SCALE_MS);
            return;
        }
    }
    s->ivdc_held = 0;
    s->ipc_ready &= ~BIT(in);
    s->core_kicked = false;
    s->core_busy = true;
    s->core_turn = in + 1;
    trace_ingenic_isp_turn(in, s->in_frame[in]);
    isp_core_frame(s, image_sensor_find(in), s->in_frame[in], 0, 0, true);
    isp_update(s);
}

static void isp_core_kick(void *opaque)
{
    IngenicIspState *s = opaque;

    isp_txn_begin(s);
    isp_core_turn(s);
    isp_txn_end(s);
}

static void isp_core_timer(void *opaque)
{
    isp_frame_due(opaque, INGENIC_ISP_VICS);
}

/*
 * A frame through VIC n and, on a layout with that many, pipeline n: the
 * T40 runs a second sensor through VIC 1 and pipeline 1 with interrupts,
 * channels and statistics of their own
 */
static void isp_frame_run(IngenicIspVic *v)
{
    IngenicIspState *s = v->isp;
    const IngenicIspLayout *l = isp_layout(s);
    bool piped = v->n < l->pipes;
    unsigned p = piped ? v->n : 0;
    ImageSensorState *sensor = isp_vic_sensor(s, v->n);
    const char *held;
    bool waited = v->waiting;
    int64_t next, began, now;

    if (!(s->vic_running & BIT(v->n)) || s->gated) {
        return;
    }
    if (!sensor) {
        isp_vic_wait(v, "no sensor on its input");
        return;
    }
    began = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    next = began + NANOSECONDS_PER_SECOND / sensor->fps;
    /*
     * A sensor in standby or without its clock sends nothing, nor does a
     * CSI receiver held in reset: the VIC idles until frames come
     */
    if (!image_sensor_streaming(sensor) || s->mclk_stopped ||
        isp_csi_held(s, v->n)) {
        isp_vic_wait(v, !image_sensor_streaming(sensor) ? "sensor in standby"
                        : s->mclk_stopped ? "sensor clock stopped"
                        : "CSI receiver held in reset");
        timer_mod(v->timer, next);
        return;
    }
    if (isp_osd_stalled(s)) {
        if (!s->osd_stalled) {
            qemu_log_mask(LOG_GUEST_ERROR, "ingenic-isp: a shown OSD "
                          "picture's width stops the ISP\n");
        }
        s->osd_stalled = true;
        isp_vic_wait(v, "an OSD picture's width stops the ISP");
        timer_mod(v->timer, next);
        return;
    }
    s->osd_stalled = false;
    if (isp_hldc(s, p)) {
        IngenicIspHldc *hl = &s->hldc[p];
        uint32_t fw, fh;

        ingenic_hldc_frame_start(hl);
        isp_hldc_frame(s, p, &fw, &fh);
        if (hl->stuck || (isp_hldc_on(s, p) && fw && fh &&
                          ingenic_hldc_stalls(hl, fw, fh))) {
            if (!s->hldc_stalled) {
                qemu_log_mask(LOG_GUEST_ERROR, "ingenic-isp: the lens "
                              "correction's settings stop the ISP\n");
            }
            s->hldc_stalled = true;
            isp_vic_wait(v, "the lens correction stops the ISP");
            timer_mod(v->timer, next);
            return;
        }
    }
    held = isp_frame_held(s, v, NANOSECONDS_PER_SECOND / sensor->fps);
    if (held) {
        isp_vic_wait(v, held);
        timer_mod(v->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + SCALE_MS);
        return;
    }
    isp_vic_wait(v, NULL);
    v->frame++;
    if (l->vic_frames) {
        VIC(v->n, l->vic_frames)++;
    }
    if (l->mdma) {
        isp_mdma_frame(s, v, sensor);
    }
    if (ingenic_frame_tap_on(&s->tap) && l->vics) {
        isp_tap_raw(s, v, sensor);
    }
    if (s->vic_closed & BIT(sensor->input)) {
        isp_raise(s, l->pipe[0].int_sta, BIT(28 + v->n));
        isp_raise(s, l->vic + v->n * l->vic_stride + l->vic_int->sta,
                  l->vic_frame_start | VIC_INT_FRAME_DONE);
        isp_update(s);
        timer_mod(v->timer, next);
        return;
    }
    if (isp_raw_vic_frame(s, v, sensor) ||
        (!piped && !isp_turns(s)) ||
        isp_core_stopped(s, NANOSECONDS_PER_SECOND / sensor->fps)) {
        isp_raise(s, l->vic + v->n * l->vic_stride + l->vic_int->sta,
                  l->vic_frame_start | VIC_INT_FRAME_DONE);
        isp_update(s);
        timer_mod(v->timer, next);
        return;
    }
    if (isp_turns(s)) {
        isp_vic_turn(s, v, sensor);
        timer_mod(v->timer, next);
        isp_core_turn(s);
        return;
    }
    trace_ingenic_isp_frame(v->n, v->frame, p, sensor->input);
    isp_core_frame(s, sensor, v->frame, v->n, p, waited);
    isp_raise(s, l->vic + v->n * l->vic_stride + l->vic_int->sta,
              l->vic_frame_start | VIC_INT_FRAME_DONE);
    if (s->switched && !v->n) {
        s->chans_set = 0;
    }
    if (!v->n && isp_vic_switching(s)) {
        s->vic_switch ^= 1;
        trace_ingenic_isp_switch(s->vic_switch);
    }
    isp_update(s);
    /*
     * A frame that takes the host longer than its period (two 4K sensors
     * on a busy host) would otherwise rearm the timer already due and
     * keep the main loop, and the BQL with it, from the guest's CPUs:
     * after such a frame the next comes no sooner than the work took
     */
    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    isp_note_late(s, now >= next);
    if (now >= next) {
        next = now + (now - began);
    }
    timer_mod(v->timer, next);
}

static void isp_frame(void *opaque)
{
    IngenicIspVic *v = opaque;
    IngenicIspState *s = v->isp;

    isp_txn_begin(s);
    isp_frame_run(v);
    isp_txn_end(s);
}

static void isp_frame_timer(void *opaque)
{
    IngenicIspVic *v = opaque;

    isp_frame_due(v->isp, v->n);
}

static void isp_run_due(IngenicIspState *s)
{
    while (s->due && !s->unlocked) {
        unsigned n = ctz32(s->due);

        s->due &= ~BIT(n);
        if (n < INGENIC_ISP_VICS) {
            isp_frame(&s->vic[n]);
        } else {
            isp_core_kick(s);
        }
    }
}

static void isp_due_bh(void *opaque)
{
    isp_run_due(opaque);
}

static void isp_vic_stop(IngenicIspState *s, int vic)
{
    if (s->vic_running & BIT(vic)) {
        trace_ingenic_isp_vic(vic, "stop");
    }
    s->vic_running &= ~BIT(vic);
    s->vic[vic].waiting = NULL;
    s->vic[vic].sub = 0;
    timer_del(s->vic[vic].timer);
    if (vic < isp_layout(s)->pipes) {
        s->msca_busy_at[vic] = 0;
        s->msca_latched[vic] = false;
    }
}

static void isp_vic_start(IngenicIspState *s, int vic)
{
    if (s->vic_running & BIT(vic)) {
        return;
    }
    trace_ingenic_isp_vic(vic, "start");
    s->vic_running |= BIT(vic);
    s->vic[vic].served = 0;
    if (isp_layout(s)->vic_turns && !vic) {
        /* the driver counts its sensors' frames from 0 as it starts */
        s->core_turn = 0;
        s->ipc_ready = 0;
    }
    if (s->switched && !vic) {
        s->vic_switch = 0;          /* the driver's first frame is input 0's */
    }
    /*
     * Where words latch at frames (the T41), the first frame comes a frame
     * time on, as from a sensor, or vic_first, and the frame-end words take
     * what was written while the VIC idled. A T41NQ has them latched by
     * then through 0x1000 (tisp_init writes it just after the start);
     * taking them at the start (not measured) keeps a guest slower than the
     * chip from running its first frame with the bypasses of reset, its
     * HLDC on.
     */
    if (isp_layout(s)->latched) {
        isp_latch(s, LATCH_END);
    }
    /*
     * So does a T32's with several sensors, which keeps a VIC its driver
     * starts just after another from taking the core alone, as the
     * sensors' first frames would not
     */
    if ((s->switched && !vic) || isp_layout(s)->latched ||
        (isp_layout(s)->vic_turns && isp_sensors(s) > 1)) {
        ImageSensorState *sensor = isp_vic_sensor(s, vic);

        if (sensor) {
            int64_t wait = NANOSECONDS_PER_SECOND / sensor->fps;

            timer_mod(s->vic[vic].timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      MAX(wait, isp_layout(s)->vic_first));
            return;
        }
    }
    isp_frame(&s->vic[vic]);
}

/*
 * The ISP's clock. Gated, a T21N's, T23DL's, T31X's and T40XP's ISP and
 * VICs stand still: every word reads the last value read from it before
 * (the bus's, not 0), writes are lost, and the interrupts hold their
 * level. Back, each VIC's frame goes on for the time it had left, and
 * goes out as any other; a running VIC's status takes vic_ungate. The
 * other parts' (unmeasured) take their next frame at once.
 */
static void isp_clock_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    int64_t now;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->gated == !level) {
        return;
    }
    trace_ingenic_isp_clock(level);
    s->gated = !level;
    if (l->gate_holds && s->gated) {
        s->gated_at = now;
        for (unsigned v = 0; v < INGENIC_ISP_VICS; v++) {
            s->vic_due[v] = timer_expire_time_ns(s->vic[v].timer);
            timer_del(s->vic[v].timer);
        }
        return;
    }
    if (l->gate_holds) {
        int64_t d = now - s->gated_at;

        for (unsigned p = 0; p < INGENIC_ISP_PIPES; p++) {
            if (s->msca_busy_at[p]) {
                s->msca_busy_at[p] += d;
            }
        }
        if (s->shadow_idle_until) {
            s->shadow_idle_until += d;
        }
        for (unsigned v = 0; v < l->vics; v++) {
            if (!(s->vic_running & BIT(v))) {
                continue;
            }
            isp_raise(s, l->vic + v * l->vic_stride + l->vic_int->sta,
                      l->vic_ungate);
            if (s->vic_due[v] >= 0) {
                timer_mod(s->vic[v].timer, s->vic_due[v] + d);
            } else {
                isp_frame(&s->vic[v]);
            }
        }
        isp_update(s);
        return;
    }
    for (unsigned v = 0; v < INGENIC_ISP_VICS; v++) {
        if (s->gated) {
            timer_del(s->vic[v].timer);
        } else {
            isp_frame(&s->vic[v]);
        }
    }
}

/* A layout without a VIC: a frame from elsewhere, out to the channels */
static void isp_frame_in_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;
    const IngenicIspLayout *l = isp_layout(s);
    ImageSensorState *sensor = image_sensor_find(0);

    if (!level || !sensor || s->gated) {
        return;
    }
    s->vic[0].frame++;
    trace_ingenic_isp_frame(0, s->vic[0].frame, 0, sensor->input);
    for (unsigned c = 0; c < INGENIC_ISP_CHANS; c++) {
        if (!(R(l->pipe[0].ch_en) & BIT(c))) {
            continue;
        }
        if (isp_drop_take(s, 0, c)) {
            isp_chan_frame(s, sensor, s->vic[0].frame, 0, c);
        } else {
            trace_ingenic_isp_chan_drop(0, c, s->vic[0].frame, "frame drop");
        }
    }
    isp_update(s);
}

/* Where channel 0 is in the frame it puts out */
static uint32_t isp_ch0_out(IngenicIspState *s)
{
    uint32_t w, h, cx, cy, cw, ch_h;
    uint64_t px;

    isp_msca_sync(s, 0);
    if (!s->ch0_from || !s->ch0_span ||
        !isp_chan_geometry(s, 0, 0, &w, &h, &cx, &cy, &cw, &ch_h)) {
        return s->ch0_rest;
    }
    px = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->ch0_from,
                  (uint64_t)cw * ch_h, s->ch0_span);
    px = MIN(px, (uint64_t)cw * ch_h - 1);
    return (px / cw) << 16 | px % cw;
}

/* The raw WDMA's and RDMA's registers that are not RAM; false for the rest */
static bool isp_raw_read(IngenicIspState *s, hwaddr o, uint32_t *v)
{
    unsigned n = s->wdma_n;

    switch (o) {
    case RAW_WDMA + RAW_START:
    case RAW_RDMA + RAW_START:
        *v = 0;
        return true;
    case RAW_WDMA + RAW_WDMA_ADDR:
        *v = n ? s->wdma_q[0] : s->wdma_head;
        return true;
    case RAW_WDMA + RAW_WDMA_DONE:
        *v = 0;
        if (s->wdma_done_n) {
            *v = s->wdma_done[0];
            memmove(s->wdma_done, s->wdma_done + 1,
                    --s->wdma_done_n * sizeof(*v));
        }
        return true;
    case RAW_WDMA + RAW_WDMA_FIFO:
        *v = (INGENIC_ISP_RAW_FIFO - n) << 12 | n << 8 | !n;
        return true;
    case RAW_RDMA + RAW_RDMA_DONE:
        *v = 0;
        if (s->rdma_done_n) {
            *v = s->rdma_done[0];
            memmove(s->rdma_done, s->rdma_done + 1,
                    --s->rdma_done_n * sizeof(*v));
        }
        return true;
    }
    return false;
}

/*
 * The T32 Pro VIC's raw capture queues (its driver's tisp_vic_mdma_fifo_*):
 * buffers pushed for the Y and UV planes, the filled ones read back, a read
 * taking the oldest. The driver clears all four registers it names together;
 * which clears which queue is the model's reading of them.
 */
static bool isp_mdma_fifo_read(IngenicIspState *s, unsigned n, hwaddr reg,
                               uint32_t *v)
{
    unsigned pl = reg == MDMA_DONE_UV;

    if (reg != MDMA_DONE_Y && reg != MDMA_DONE_UV) {
        return false;
    }
    uint32_t *d = s->mdma_done[n * 2 + pl];

    *v = 0;
    if (s->mdma_dn[n][pl]) {
        *v = d[0];
        memmove(d, d + 1, --s->mdma_dn[n][pl] * sizeof(*v));
    }
    return true;
}

static bool isp_mdma_fifo_write(IngenicIspState *s, unsigned n, hwaddr reg,
                                uint32_t v)
{
    unsigned pl = reg == MDMA_PUSH_UV || reg == MDMA_DONE_UV ||
                  reg == MDMA_CLEAR_UV;

    switch (reg) {
    case MDMA_PUSH_Y:
    case MDMA_PUSH_UV:
        if (s->mdma_qn[n][pl] < INGENIC_ISP_MDMA_FIFO) {
            s->mdma_q[n * 2 + pl][s->mdma_qn[n][pl]++] = v;
        }
        return true;
    case MDMA_DONE_Y:
    case MDMA_DONE_UV:
        if (v & 1) {
            s->mdma_dn[n][pl] = 0;
        }
        return true;
    case MDMA_CLEAR_Y:
    case MDMA_CLEAR_UV:
        if (v & 1) {
            s->mdma_qn[n][pl] = 0;
        }
        return true;
    }
    return false;
}

static uint32_t isp_reg_read(IngenicIspState *s, hwaddr off)
{
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspChan *ch = isp_chan(s, off);
    hwaddr reg;
    int vic;
    uint32_t v;

    if (l->core_next && off == l->core_next) {
        return deposit32(R(off), 8, 4, isp_next_turn(s));
    }
    if (l->raw_dma && off >= l->raw_dma && off < l->raw_dma + 0x2000 &&
        isp_raw_read(s, off - l->raw_dma, &v)) {
        return v;
    }
    if (l->ch0_out[0] &&
        (off == l->ch0_out[0] || (l->ch0_out[1] && off == l->ch0_out[1]))) {
        return isp_ch0_out(s);
    }
    if (isp_mdns_read(s, off, &v)) {
        return v;
    }
    if (ch) {
        const IngenicIspChanRegs *r = l->chan;
        uint32_t i = ch->done_rd, cr = off & 0xff;
        unsigned p, n;

        if (cr == r->done_y) {
            if (!ch->done_n) {
                return 0;
            }
            ch->done_rd = (i + 1) % INGENIC_ISP_FIFO;
            ch->done_n--;
            ch->popped_uv = ch->done_uv[i];
            ch->popped_frame = ch->done_frame[i];
            isp_chan_index(s, ch, &p, &n);
            trace_ingenic_isp_chan_pop(p, n, ch->done_y[i], ch->done_n);
            return ch->done_y[i];
        }
        if (cr == r->done_sta) {
            return ch->done_n << 8 | !ch->done_n;
        }
        if (cr == r->done_uv) {
            if (l->done_uv_head && ch->done_n) {
                return ch->done_uv[i];
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
    if (l->stats && l->stats->t41 && (off & 0xf) == 8 &&
        isp_t41_ram_port(off & ~0xf) >= 0) {
        return isp_t41_ram_read(s, isp_t41_ram_port(off & ~0xf));
    }
    if (l->ldc && off >= l->ldc && off < l->ldc + INGENIC_ISP_LDC_SIZE) {
        return ingenic_isp_ldc_read(&s->ldc, off - l->ldc);
    }
    for (unsigned p = 0; p < 2; p++) {
        hwaddr at = p ? l->hldc_sec : l->hldc;
        uint32_t fw, fh;

        if (at && off >= at && off < at + isp_hldc_size(l)) {
            isp_hldc_frame(s, p, &fw, &fh);
            return ingenic_hldc_read(&s->hldc[p], off - at, fw, fh);
        }
    }
    vic = isp_vic(s, off, &reg);
    if (l->mdma_fifo && vic >= 0 && reg >= l->mdma &&
        isp_mdma_fifo_read(s, vic, reg - l->mdma, &v)) {
        return v;
    }
    if (vic >= 0 && reg == l->vic_ctrl && !l->vic_ctrl_level) {
        return 0;
    }
    if (l->ch_busy && off == l->ch_busy) {
        return 0;
    }
    for (unsigned p = 0; p < l->pipes; p++) {
        if ((l->pipe[p].busy && off == l->pipe[p].busy) ||
            (l->ch_en_pulse && off == l->pipe[p].ch_en)) {
            isp_msca_sync(s, p);
            return R(off);
        }
    }
    if (l->run.kick && (off == l->shadow || off == l->shadow_open)) {
        return 0;
    }
    if (l->run.kick && off == l->run.kick) {
        return l->run.busy ? R(off) | (s->shadow_due ? l->run.busy : 0) : 0;
    }
    if (l->run.sta && off == l->run.sta) {
        return (s->shadow_open ? BIT(4) : 0) |
               (l->run.done_on_close ? !s->shadow_open : s->shadow_done);
    }
    if (l->set.bits && off == l->set.off) {
        return R(off) | l->set.bits;
    }
    if (l->idle && off == l->idle) {
        return R(off) | 1;
    }
    return R(off);
}

/*
 * A register whose write is just a store, as far as a read can tell: the
 * value a held write left is what it reads back as
 */
static bool isp_reg_stored(IngenicIspState *s, hwaddr off)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspMdns *m = l->mdns;
    hwaddr reg;

    if ((off & 3) || isp_chan(s, off) || isp_vic(s, off, &reg) >= 0 ||
        (m && off >= m->base && off < m->base + 0x800) ||
        (l->raw_dma && off >= l->raw_dma && off < l->raw_dma + 0x2000) ||
        (l->ldc && off >= l->ldc && off < l->ldc + INGENIC_ISP_LDC_SIZE)) {
        return false;
    }
    for (unsigned p = 0; p < 2; p++) {
        hwaddr at = p ? l->hldc_sec : l->hldc;

        if (at && off >= at && off < at + isp_hldc_size(l)) {
            return false;
        }
    }
    for (unsigned p = 0; p < l->pipes; p++) {
        const IngenicIspPipe *pipe = &l->pipe[p];

        if (off == pipe->int_en || off == pipe->int_sta ||
            off == pipe->int_clr || off == pipe->done_en ||
            off == pipe->done_sta || off == pipe->done_clr ||
            off == pipe->ch_en || (pipe->busy && off == pipe->busy)) {
            return false;
        }
    }
    return !(l->ch_busy && off == l->ch_busy) &&
           !(l->ch0_out[0] && (off == l->ch0_out[0] || off == l->ch0_out[1])) &&
           !(l->shadow && off == l->shadow) &&
           !(l->shadow_open && off == l->shadow_open) &&
           !(l->latch_now && off == l->latch_now) &&
           !(l->idle && off == l->idle) &&
           !(l->set.bits && off == l->set.off) &&
           !(l->run.kick && (off == l->run.kick || off == l->run.sta)) &&
           !(l->tick.sta && (off == l->tick.sta || off == l->tick.clr)) &&
           !(l->dma.sta && (off == l->dma.sta || off == l->dma.clr)) &&
           !(l->err_sta && (off == l->err_sta || off == l->err_clr));
}

/* The latest write held for @off, if the register reads back what it got */
static bool isp_held_value(IngenicIspState *s, hwaddr off, uint32_t *v)
{
    if (!isp_reg_stored(s, off)) {
        return false;
    }
    for (unsigned i = s->held->len; i--;) {
        IngenicIspHeld *h = &g_array_index(s->held, IngenicIspHeld, i);

        if (!h->line && h->off == off) {
            *v = h->val;
            return true;
        }
    }
    return false;
}

static uint64_t ingenic_isp_read(void *opaque, hwaddr off, unsigned size)
{
    IngenicIspState *s = INGENIC_ISP(opaque);
    char name[ISP_NAME_LEN];
    uint32_t v;

    if (s->gated && isp_layout(s)->gate_holds) {
        v = s->bus_last;
    } else if (s->unlocked && isp_held_value(s, off, &v)) {
        s->bus_last = v;
    } else {
        v = s->bus_last = isp_reg_read(s, off);
    }
    if (trace_event_get_state_backends(TRACE_INGENIC_ISP_READ)) {
        trace_ingenic_isp_read(off, v, isp_pc(), isp_reg_name(s, off, name));
    }
    return v;
}

static void isp_reg_write(IngenicIspState *s, hwaddr off, uint32_t v);

static uint32_t isp_ldl(hwaddr addr)
{
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

/*
 * The register of a T33's or T32 Pro's block that says which buffer it
 * wrote last: the lists carry its words, which the hardware does not take
 * from them (a list's copy is the 0 its driver left, taken here as ignored)
 */
static bool isp_stats_index(IngenicIspState *s, hwaddr off)
{
    const IngenicIspStats *st = isp_layout(s)->stats;

    return st && st->flat && (off == st->ae + 0x50 || off == st->awb + 0x50);
}

/* The register lists set in @mask, from list @base, written in */
static void isp_rcfg(IngenicIspState *s, unsigned base, uint32_t mask)
{
    const IngenicIspLayout *l = isp_layout(s);
    char name[ISP_NAME_LEN];

    s->rcfg_running = true;
    for (unsigned n = 0; n < 32; n++) {
        uint32_t list, len;

        if (!(mask & BIT(n))) {
            continue;
        }
        list = isp_ldl(R(l->rcfg) + 4 * (base + n));
        len = MIN(isp_ldl(list), RCFG_MAX);
        trace_ingenic_isp_rcfg(base + n, list, len);
        for (uint32_t i = 1; i <= len;) {
            uint32_t cmd = isp_ldl(list + 4 * i);
            uint32_t cnt = extract32(cmd, 18, 12) + 1;
            hwaddr reg = extract32(cmd, 0, 18) * 4;
            /* bit 31 clear: all the words to one register, a table's port */
            hwaddr step = cmd & BIT(31) ? 4 : 0;

            for (uint32_t k = 0; k < cnt && i + 1 + k <= len; k++) {
                hwaddr to = reg + step * k;
                uint32_t v = isp_ldl(list + 4 * (i + 1 + k));

                if (to >= INGENIC_ISP_IOSIZE || isp_stats_index(s, to)) {
                    continue;
                }
                if (trace_event_get_state_backends(
                        TRACE_INGENIC_ISP_RCFG_WRITE)) {
                    trace_ingenic_isp_rcfg_write(to, v, isp_pc(),
                                                 isp_reg_name(s, to, name));
                }
                isp_reg_write(s, to, v);
            }
            i += cnt + 1;
        }
    }
    s->rcfg_running = false;
    /* a list's done bit rises as it ends; lists run at once here */
    if (l->lists[base / 32].sta) {
        R(l->lists[base / 32].sta) |= mask;
        isp_update(s);
    }
}

static uint32_t isp_mask_find(const IngenicIspMask *masks, unsigned n,
                              hwaddr off)
{
    unsigned lo = 0, hi = n;

    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        const IngenicIspMask *m = &masks[mid];

        if (off < m->off) {
            hi = mid;
        } else if (off >= m->off + 4 * m->n) {
            lo = mid + 1;
        } else {
            return m->mask;
        }
    }
    return ~0u;
}

/* The bits of the word at @off a write keeps */
static uint32_t isp_write_mask(const IngenicIspLayout *l, hwaddr off)
{
    if (l->vic_masks && off >= l->vic &&
        off < l->vic + l->vics * l->vic_stride) {
        return isp_mask_find(l->vic_masks, l->vic_nmasks,
                             (off - l->vic) % l->vic_stride);
    }
    return isp_mask_find(l->masks, l->nmasks, off);
}

/* Where in s->latch the latched word at @off waits, or -1; *trig its latch */
static int isp_latched(const IngenicIspLayout *l, hwaddr off, uint32_t *trig)
{
    unsigned lo = 0, hi = l->nlatched;

    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        const IngenicIspLatch *r = &l->latched[mid];

        if (off < r->off) {
            hi = mid;
        } else if (off >= r->off + 4 * r->n) {
            lo = mid + 1;
        } else {
            int k = (off - r->off) / 4;

            for (unsigned i = 0; i < mid; i++) {
                k += l->latched[i].n;
            }
            *trig = r->trig;
            return k;
        }
    }
    return -1;
}

/*
 * The latched words take what was written to them: at a frame's start or
 * end (@when), or (0) the per-frame ones at once
 */
static void isp_latch(IngenicIspState *s, uint32_t when)
{
    const IngenicIspLayout *l = isp_layout(s);
    unsigned k = 0;

    for (unsigned i = 0; i < l->nlatched; i++) {
        const IngenicIspLatch *r = &l->latched[i];
        bool now = r->trig > LATCH_END ?
                   when == LATCH_START && (R(r->trig) & 1) :
                   !when || r->trig == when;

        for (unsigned j = 0; now && j < r->n; j++) {
            R(r->off + 4 * j) = s->latch[k + j];
        }
        k += r->n;
    }
    for (unsigned i = 0; when == LATCH_START && i < l->nlatched; i++) {
        if (l->latched[i].trig > LATCH_END) {
            R(l->latched[i].trig) &= ~1;
        }
    }
}

/*
 * The T41's shadow port writes its RAM's pairs out, once: a run longer
 * than the RAM goes round it again
 */
static unsigned isp_shd_ram(const IngenicIspLayout *l)
{
    return l->shd_ram ?: INGENIC_ISP_SHD_RAM;
}

static void isp_shadow_ram_run(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);
    uint32_t n = s->shd_run;
    char name[ISP_NAME_LEN];

    s->shd_run = 0;
    for (uint32_t i = 0; i < n; i++) {
        unsigned e = i % isp_shd_ram(l);
        hwaddr to = s->shd_off[e];

        R(l->shadow_last) = s->shd_val[e];
        if (to >= INGENIC_ISP_IOSIZE) {
            continue;
        }
        if (trace_event_get_state_backends(TRACE_INGENIC_ISP_SHADOW)) {
            trace_ingenic_isp_shadow(to, s->shd_val[e], isp_pc(),
                                     isp_reg_name(s, to, name));
        }
        isp_reg_write(s, to, s->shd_val[e]);
    }
}

static void isp_raw_reset(IngenicIspState *s)
{
    s->wdma_n = s->wdma_done_n = s->rdma_done_n = 0;
    s->wdma_head = 0;
    s->wdma_raw = s->rdma_start = false;
    s->raw_in_bpp = 0;
}

/*
 * The T41's reset through its own register: every register but the VICs'
 * back to its reset value, the channels' queues emptied and the shadow
 * port closed, its RAM and its last value kept. The VIC goes on sending
 * frames.
 */
static void isp_core_reset(IngenicIspState *s)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr vic_end = l->vic + l->vics * l->vic_stride;

    trace_ingenic_isp_reset();
    for (hwaddr off = 0; off < INGENIC_ISP_IOSIZE; off += 4) {
        if ((off < l->vic || off >= vic_end) && off != l->shadow_last) {
            R(off) = 0;
        }
    }
    if (l->osd && l->osd->csc) {
        memcpy(&R(l->osd->csc), isp_osd_csc, sizeof(isp_osd_csc));
    }
    for (unsigned i = 0; i < l->resets; i++) {
        if (l->reset[i].off < l->vic || l->reset[i].off >= vic_end) {
            R(l->reset[i].off) = l->reset[i].val;
        }
    }
    for (unsigned i = 0, k = 0; i < l->nlatched; i++) {
        for (unsigned j = 0; j < l->latched[i].n; j++) {
            s->latch[k++] = R(l->latched[i].off + 4 * j);
        }
    }
    s->shd_n = 0;
    s->shd_run = 0;
    s->shd_open = false;
    s->shadow_pending = false;
    s->core_idle = false;
    s->core_wait = false;
    s->core_resumed = 0;
    memset(s->chan, 0, sizeof(s->chan));
    isp_raw_reset(s);
    isp_hldc_reset(s);
    s->hldc_stalled = false;
    isp_update(s);
}

/* The first @n pairs of the shadow run, written now */
static void isp_shadow_run(IngenicIspState *s, uint32_t n)
{
    const IngenicIspLayout *l = isp_layout(s);
    char name[ISP_NAME_LEN];

    trace_ingenic_isp_shadow_run(n);
    s->shadow_due = 0;
    s->shadow_done = true;
    for (uint32_t i = 0; i < n; i++) {
        unsigned e = (s->shadow_first + i) % l->run.ram;
        hwaddr to = s->shadow_off[e];

        if (l->run.last) {
            R(l->run.last) = s->shadow_val[e];
        }
        if (to == l->shadow || to >= INGENIC_ISP_IOSIZE) {
            continue;
        }
        if (trace_event_get_state_backends(TRACE_INGENIC_ISP_SHADOW)) {
            trace_ingenic_isp_shadow(to, s->shadow_val[e], isp_pc(),
                                     isp_reg_name(s, to, name));
        }
        isp_reg_write(s, to, s->shadow_val[e]);
    }
}

/*
 * The MSCA is between frames: from a little after the last frame's end to
 * a little after the next one's start, while VIC 0 takes no frames, and
 * with its clock stopped
 */
static bool isp_msca_idle(IngenicIspState *s)
{
    return !(s->vic_running & 1) || s->vic[0].waiting || s->gated ||
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) < s->shadow_idle_until;
}

/* A write to a queueing shadow port's registers, false for others */
static bool isp_shadow_run_write(IngenicIspState *s, hwaddr off, uint32_t v)
{
    const IngenicIspLayout *l = isp_layout(s);
    const IngenicIspShadowRun *r = &l->run;

    if (off == l->shadow_open) {
        if ((v & 3) == 1) {
            s->shadow_open = true;
            s->shadow_done = false;
            s->shadow_n = 0;
            s->shadow_pending = false;
        } else if ((v & 3) == 2) {
            s->shadow_open = false;
        }
        return true;
    }
    if (off == l->shadow) {
        /* pairs outside a run are dropped */
        if (!s->shadow_open) {
            return true;
        }
        if (!s->shadow_pending) {
            s->shadow_value = v;
        } else {
            hwaddr to = l->shadow_base + (v & ~3);

            trace_ingenic_isp_shadow_queue(s->shadow_n, to, s->shadow_value);
            s->shadow_val[s->shadow_n] = s->shadow_value;
            s->shadow_off[s->shadow_n] = to;
            s->shadow_n = (s->shadow_n + 1) % r->ram;
        }
        s->shadow_pending = !s->shadow_pending;
        return true;
    }
    if (off == r->kick) {
        uint32_t n = extract32(v, r->len_shift, ctpop32(r->len_mask)) + 1;

        R(off) = v & (r->len_mask << r->len_shift |
                      r->start_mask << r->start_shift);
        if (!(v & 1)) {
            return true;
        }
        s->shadow_first = (v >> r->start_shift) & r->start_mask;
        if (isp_msca_idle(s)) {
            isp_shadow_run(s, n);
        } else {
            trace_ingenic_isp_shadow_wait(n);
            s->shadow_due = n;
        }
        return true;
    }
    return off == r->sta || (r->last && off == r->last);
}

static bool isp_raw_write(IngenicIspState *s, hwaddr o, uint32_t v)
{
    const IngenicIspLayout *l = isp_layout(s);

    switch (o) {
    case RAW_WDMA + RAW_START:
        s->wdma_raw = v & 1;
        return true;
    case RAW_RDMA + RAW_START:
        s->rdma_start = v & 1;
        return true;
    case RAW_WDMA + RAW_WDMA_ADDR:
        if (s->wdma_n < INGENIC_ISP_RAW_FIFO) {
            s->wdma_q[s->wdma_n++] = v;
        }
        return true;
    case RAW_WDMA + RAW_WDMA_FIFO:
        return true;
    case RAW_RDMA + RAW_RDMA_ADDR:
        R(l->raw_dma + o) = v;
        if (s->rdma_start && R(l->raw_dma + RAW_RDMA + RAW_RDMA_MODE)) {
            isp_raw_rdma_frame(s, v);
        }
        return true;
    }
    return false;
}

static void isp_reg_write(IngenicIspState *s, hwaddr off, uint32_t v)
{
    const IngenicIspLayout *l = isp_layout(s);
    IngenicIspChan *ch = isp_chan(s, off);
    char name[ISP_NAME_LEN];
    hwaddr reg;
    int vic;

    for (unsigned p = 0; p < 2; p++) {
        hwaddr at = p ? l->hldc_sec : l->hldc;

        if (at && off >= at && off < at + isp_hldc_size(l)) {
            ingenic_hldc_write(&s->hldc[p], off - at, v);
            return;
        }
    }
    if (l->raw_dma && off >= l->raw_dma && off < l->raw_dma + 0x2000 &&
        isp_raw_write(s, off - l->raw_dma, v)) {
        return;
    }
    if (l->stats && l->stats->t41) {
        int port = isp_t41_ram_port(off);

        if (port >= 0 && (v & 1)) {
            s->ram_at[port] = 0;
        }
        switch (off) {
        case 0x1b000:
        case 0x1b040:
            if (v & 1) {
                R(off + 4) = 0;
                s->gsm_armed = true;
            }
            return;
        case 0xe010:
            R(0xe014) &= ~((v & 1) | (v & BIT(16) ? BIT(8) : 0));
            return;
        }
    }
    if (l->ldc && off >= l->ldc && off < l->ldc + INGENIC_ISP_LDC_SIZE) {
        if (ingenic_isp_ldc_write(&s->ldc, off - l->ldc, v)) {
            qemu_set_irq(s->ldc_irq, ingenic_isp_ldc_irq(&s->ldc));
        }
        return;
    }
    if (isp_mdns_write(s, off, v)) {
        return;
    }
    if (ch) {
        const IngenicIspChanRegs *r = l->chan;
        uint32_t cr = off & 0xff;
        unsigned p, n;

        isp_chan_index(s, ch, &p, &n);
        if (!p && l->osd && l->osd->mask_load) {
            hwaddr m = off - (l->osd->mask + n * l->osd->mask_chan);

            /* the positions and sizes read the ones loaded */
            if (m < OSD_MASKS * l->osd->mask_step &&
                m % l->osd->mask_step < 8) {
                s->mask_next[n][m / l->osd->mask_step * 2 +
                                m % l->osd->mask_step / 4] = v;
                return;
            }
        }
        if (l->drop && (cr == l->drop || cr == l->drop + 4)) {
            /* the frame that reached the MSCA decided with the old */
            isp_msca_sync(s, p);
        }
        if (l->drop && cr == l->drop) {
            R(off) = v & 0x1f;
            return;
        }
        if (l->drop && cr == l->drop + 4) {
            R(off) = v;
            if (l->drop_rephase) {
                s->drop_pos[p][n] = 0;
            }
            return;
        }
        if (l->push_latch && (cr == r->y_push || cr == r->uv_push ||
                              cr == r->clr_y || cr == r->clr_uv)) {
            /* the frame at the MSCA took the buffers there before this */
            isp_msca_sync(s, p);
        }
        /* the T33's lists push 0s that queue no buffer */
        if (cr == r->y_push) {
            /*
             * The T32 Pro's driver, with several sensors, gives a channel the
             * next frame's buffer in each sensor's register list
             */
            if (l->list_push_latest && s->rcfg_running && v) {
                ch->y_n = 0;
            }
            if (v || !l->rcfg) {
                isp_push(ch->y, ch->y_rd, &ch->y_n, v);
                if (l->push_latch) {
                    R(off) = v | 1;
                }
                trace_ingenic_isp_chan_push(p, n, "Y", v, ch->y_n);
            }
            return;
        }
        if (cr == r->uv_push) {
            if (l->list_push_latest && s->rcfg_running && v) {
                ch->uv_n = 0;
            }
            if (v || !l->rcfg) {
                isp_push(ch->uv, ch->uv_rd, &ch->uv_n, v);
                if (l->push_latch) {
                    R(off) = v | 1;
                }
                trace_ingenic_isp_chan_push(p, n, "UV", v, ch->uv_n);
            }
            return;
        }
        if (cr == r->clr_y && s->switched && !p) {
            s->chans_set |= BIT(n);
        }
        if (cr == r->clr_y || cr == r->clr_uv || cr == r->clr_done_y ||
            cr == r->clr_done_uv) {
            uint32_t was = R(off);

            R(off) = v;
            if (l->clr_bit0 && !(v & 1)) {
                return;
            }
            /*
             * The T32 Pro's driver, with several sensors, leaves 1 in a
             * channel's clears in its register lists once the channel ran
             * short of buffers, and so writes it again after each push
             */
            if (l->clr_edge && (was & 1)) {
                return;
            }
            if (cr == r->clr_y) {
                ch->y_n = 0;
                R(off - cr + r->y_push) &= ~1;
                trace_ingenic_isp_chan_clear(p, n, "Y push");
            } else if (cr == r->clr_uv) {
                ch->uv_n = 0;
                R(off - cr + r->uv_push) &= ~1;
                trace_ingenic_isp_chan_clear(p, n, "UV push");
            } else {
                ch->done_n = 0;
                trace_ingenic_isp_chan_clear(p, n, "done");
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
    /*
     * The T32 Pro's VIC n, at +0x130 and +0x134, gates inputs n and n + 2
     * into the core, bits 7:4 0 closing it (its driver writes 0xf0 to open
     * and nothing at its start: the gates start open). Once a gate closes
     * the core holds none of the VIC's frames, here at once, and
     * 0x30028 bit 28 + n says so, again for each frame the VIC then keeps
     * from the core. A single frame through (bits 7:4 1) is not modelled.
     */
    if (vic >= 0 && l->vic_out && (reg == l->vic_out ||
                                   reg == l->vic_out + 4)) {
        uint32_t in = BIT(vic + (reg - l->vic_out) / 4 * l->vics);

        R(off) = v;
        if (v & 0xf0) {
            s->vic_closed &= ~in;
        } else {
            s->vic_closed |= in;
            isp_raise(s, l->pipe[0].int_sta, BIT(28 + vic));
            isp_update(s);
        }
        return;
    }
    if (l->mdma_fifo && vic >= 0 && reg >= l->mdma &&
        isp_mdma_fifo_write(s, vic, reg - l->mdma, v)) {
        return;
    }
    if (vic >= 0 && l->mdma && reg == l->mdma + MDMA_RESET) {
        if (v & 1) {
            s->mdma_slot[vic] = 0;
            s->mdma_kick[vic] = 0;
            VIC(vic, l->mdma + MDMA_DONE) = 0;
        }
        return;
    }
    if (vic >= 0 && l->mdma && reg == l->mdma + MDMA_KICK) {
        if (v & 1 && s->mdma_kick[vic] < MDMA_SLOTS) {
            s->mdma_kick[vic]++;
        }
        return;
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

    if (l->shadow_kick && off == l->shadow_open) {
        if ((v & 3) == 1) {
            s->shd_open = true;
            s->shd_n = 0;
            s->shadow_pending = false;
            R(l->shadow_state) = 0x10;
        } else if ((v & 3) == 2 && s->shd_open) {
            s->shd_open = false;
            R(l->shadow_state) = 1;
        }
        return;
    }
    if (l->shadow_last && off == l->shadow_last) {
        return;
    }
    if (l->core_reset && off == l->core_reset && (v & 1)) {
        isp_core_reset(s);
        return;
    }
    if (l->core_reset && off == l->core_reset && (v & BIT(2))) {
        s->core_idle = false;
        s->core_wait = true;
        if (R(l->latch_now) & 1) {
            isp_core_resume(s);
        }
    }
    /*
     * The T33's driver starts each frame with its top list written twice
     * over: the core, given its frame, takes no other until the guest has
     * served that frame's end
     */
    if (l->core_kick && off == l->core_kick &&
        (v & (l->core_kick_bits ?: 1)) && !s->core_busy) {
        s->core_kicked = true;
        timer_mod(s->core_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
    if (l->idle_req && off == l->idle_req && (v & 1)) {
        s->core_idle = true;
    }
    if (l->shadow_kick && off == l->shadow) {
        if (!s->shd_open) {
            return;
        }
        if (!s->shadow_pending) {
            s->shadow_value = v;
        } else {
            s->shd_val[s->shd_n] = s->shadow_value;
            s->shd_off[s->shd_n] = v & ~3;
            s->shd_n = (s->shd_n + 1) % isp_shd_ram(l);
            if (l->shd_pair_clears_last) {
                R(l->shadow_last) = 0;
            }
        }
        s->shadow_pending = !s->shadow_pending;
        return;
    }
    if (l->shadow_kick && off == l->shadow_kick) {
        s->shd_run = v & 1 ? extract32(v, 4, 12) + 1 : 0;
        if (l->shd_kick_idle && isp_msca_idle(s)) {
            isp_shadow_ram_run(s);
        }
        return;
    }
    if (l->run.kick && isp_shadow_run_write(s, off, v)) {
        return;
    }
    if (l->shadow && off == l->shadow) {
        hwaddr to = l->shadow_base + (v & ~3);

        if (!s->shadow_pending) {
            s->shadow_value = v;
        } else if (to != l->shadow && to < INGENIC_ISP_IOSIZE) {
            s->shadow_pending = false;
            if (trace_event_get_state_backends(TRACE_INGENIC_ISP_SHADOW)) {
                trace_ingenic_isp_shadow(to, s->shadow_value, isp_pc(),
                                         isp_reg_name(s, to, name));
            }
            isp_reg_write(s, to, s->shadow_value);
            return;
        }
        s->shadow_pending = !s->shadow_pending;
        return;
    }
    if (l->shadow_open && off == l->shadow_open) {
        s->shadow_pending = false;
    }
    if ((l->tick.sta && off == l->tick.sta) ||
        (l->dma.sta && off == l->dma.sta)) {
        return;
    }
    if ((l->tick.clr && off == l->tick.clr) ||
        (l->dma.clr && off == l->dma.clr)) {
        R(off == l->tick.clr ? l->tick.sta : l->dma.sta) &= ~v;
        isp_update(s);
        return;
    }
    for (unsigned k = 0; k < ARRAY_SIZE(l->lists); k++) {
        if (l->lists[k].sta && off == l->lists[k].sta) {
            return;
        }
        if (l->lists[k].clr && off == l->lists[k].clr) {
            R(l->lists[k].sta) &= ~v;
            isp_update(s);
            return;
        }
    }
    if (l->err_sta && off == l->err_sta) {
        return;
    }
    if (l->err_clr && off == l->err_clr) {
        R(l->err_sta) &= ~v;
        isp_update(s);
        return;
    }
    for (unsigned p = 0; p < l->pipes; p++) {
        if (off == l->pipe[p].int_sta ||
            (l->pipe[p].busy && off == l->pipe[p].busy)) {
            return;
        }
        if ((l->pipe[p].busy || l->chans_at_start) &&
            off == l->pipe[p].ch_en) {
            isp_msca_sync(s, p);
        }
        if (off == l->pipe[p].int_clr) {
            R(l->pipe[p].int_sta) &= ~v;
            if (v & l->int_frame_end) {
                s->core_busy = false;
            }
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
    if (l->latch_now && off == l->latch_now) {
        if (v & 1) {
            isp_latch(s, 0);
        }
        /* with frames running it stays set, and the words take writes */
        R(off) = (v & 1) && s->vic_running;
        if ((v & 1) && s->core_wait && !s->core_idle) {
            isp_core_resume(s);
        }
        return;
    }
    if (l->latched) {
        uint32_t trig;
        int k = isp_latched(l, off, &trig);

        if (k >= 0) {
            uint32_t m = isp_write_mask(l, off);

            s->latch[k] = (s->latch[k] & ~m) | (v & m);
            if (trig <= LATCH_END && (R(l->latch_now) & 1)) {
                R(off) = s->latch[k];
            }
            return;
        }
    }
    if (l->masks || l->vic_masks) {
        uint32_t m = isp_write_mask(l, off);

        v = (R(off) & ~m) | (v & m);
    }
    R(off) = v;
    for (unsigned i = 0; i < ARRAY_SIZE(l->busy); i++) {
        if (l->busy[i].bits && off == l->busy[i].off) {
            R(off) &= ~l->busy[i].bits;
        }
    }
    for (unsigned p = 0; l->busy_held && p < l->pipes; p++) {
        if (off == l->pipe[p].ch_en && !s->msca_latched[p]) {
            R(l->pipe[p].busy) &= v | s->msca_chans[p];
        }
    }
    isp_update(s);
}

/*
 * The registers that only drive the interrupt lines: written at once while a
 * pass runs, as no pass reads them, since a handler that acknowledges or
 * masks a level would be run again for as long as it were held
 */
static bool isp_irq_reg(IngenicIspState *s, hwaddr off)
{
    const IngenicIspLayout *l = isp_layout(s);
    hwaddr reg;
    int vic = isp_vic(s, off, &reg);

    for (unsigned p = 0; p < l->pipes; p++) {
        if (off == l->pipe[p].int_en || off == l->pipe[p].int_clr) {
            return true;
        }
    }
    for (unsigned i = 0; vic >= 0 && l->vic_int && i <= l->vic_int->pair;
         i++) {
        if (reg == l->vic_int->msk + 4 * i || reg == l->vic_int->clr + 4 * i) {
            return true;
        }
    }
    return false;
}

static void isp_write_now(IngenicIspState *s, hwaddr off, uint32_t v)
{
    if (s->gated && isp_layout(s)->gate_holds) {
        return;
    }
    isp_reg_write(s, off, v);
}

static void ingenic_isp_write(void *opaque, hwaddr off, uint64_t value,
                              unsigned size)
{
    IngenicIspState *s = INGENIC_ISP(opaque);
    char name[ISP_NAME_LEN];

    if (trace_event_get_state_backends(TRACE_INGENIC_ISP_WRITE)) {
        trace_ingenic_isp_write(off, value, isp_pc(),
                                isp_reg_name(s, off, name));
    }
    if (s->unlocked && !(s->gated && isp_layout(s)->gate_holds) &&
        !isp_irq_reg(s, off)) {
        isp_hold(s, NULL, off, value);
        return;
    }
    isp_write_now(s, off, value);
}

/* In a test, writes as a guest's would land while a pass runs */
static void isp_poke(IngenicIspState *s)
{
    g_auto(GStrv) w = g_strsplit(s->poke, ",", -1);

    s->poke_passes++;
    bql_lock();
    for (unsigned i = 0; w[i]; i++) {
        uint64_t off, val;
        const char *eq = strchr(w[i], '=');

        if (!eq || qemu_strtou64(w[i], &eq, 0, &off) || *eq != '=' ||
            qemu_strtou64(eq + 1, NULL, 0, &val)) {
            continue;
        }
        ingenic_isp_write(s, off, val, 4);
        if (isp_reg_stored(s, off) && ingenic_isp_read(s, off, 4) != val) {
            s->poke_misses++;
        }
    }
    bql_unlock();
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
    const IngenicIspLayout *l = isp_layout(s);

    isp_quiesce(s);
    s->due = 0;
    for (unsigned v = 0; v < INGENIC_ISP_VICS; v++) {
        timer_del(s->vic[v].timer);
        s->vic[v].frame = 0;
        s->vic[v].served = 0;
        s->vic[v].sub = 0;
    }
    memset(s->regs, 0, sizeof(s->regs));
    g_clear_pointer(&s->mdns_pic, g_free);
    s->mdns_pic_len = 0;
    s->mdns_out = NULL;
    for (unsigned k = 0; k < ARRAY_SIZE(s->pic); k++) {
        g_clear_pointer(&s->pic[k].own, g_free);
        s->pic[k].buf = NULL;
        s->pic[k].cap = 0;
        s->pic[k].valid = false;
    }
    s->pic_scope = false;
    for (unsigned k = 0; k < ARRAY_SIZE(s->mdns_work); k++) {
        g_clear_pointer(&s->mdns_work[k], g_free);
        s->mdns_work_len[k] = 0;
    }
    s->mdns_input = -1;
    s->mdns_hist = 0;
    s->mdns_count = 0;
    if (l->osd && l->osd->csc) {
        memcpy(&R(l->osd->csc), isp_osd_csc, sizeof(isp_osd_csc));
    }
    memset(s->mask_next, 0, sizeof(s->mask_next));
    s->ch0_from = 0;
    s->ch0_span = 0;
    s->ch0_rest = 0;
    s->ch0_latched = false;
    for (unsigned p = 0; l->drop && p < l->pipes; p++) {
        for (unsigned n = 0; n < INGENIC_ISP_CHANS; n++) {
            R(l->pipe[p].ch + n * 0x100 + l->drop + 4) = l->drop_mark;
        }
    }
    for (unsigned i = 0; i < l->resets; i++) {
        R(l->reset[i].off) = l->reset[i].val;
    }
    for (unsigned v = 0; v < l->vics; v++) {
        for (unsigned i = 0; i < l->vic_resets; i++) {
            VIC(v, l->vic_reset[i].off) = l->vic_reset[i].val;
        }
        s->mdma_slot[v] = 0;
        s->mdma_kick[v] = 0;
        memset(s->mdma_qn[v], 0, sizeof(s->mdma_qn[v]));
        memset(s->mdma_dn[v], 0, sizeof(s->mdma_dn[v]));
    }
    for (unsigned i = 0, k = 0; i < l->nlatched; i++) {
        for (unsigned j = 0; j < l->latched[i].n; j++) {
            s->latch[k++] = R(l->latched[i].off + 4 * j);
        }
    }
    memset(s->shd_val, 0, sizeof(s->shd_val));
    memset(s->shd_off, 0, sizeof(s->shd_off));
    memset(s->gsm, 0, sizeof(s->gsm));
    memset(s->lce, 0, sizeof(s->lce));
    memset(s->ram_at, 0, sizeof(s->ram_at));
    s->lce_bank = 0;
    s->gsm_armed = false;
    s->shd_n = 0;
    s->shd_run = 0;
    s->shd_open = false;
    s->core_idle = false;
    s->core_wait = false;
    s->core_resumed = 0;
    memset(s->chan, 0, sizeof(s->chan));
    s->vic_running = 0;
    s->vic_switch = 0;
    s->ivdc_input = 0;
    memset(s->ivdc_sent, 0, sizeof(s->ivdc_sent));
    memset(s->ivdc_frame_no, 0, sizeof(s->ivdc_frame_no));
    s->ivdc_taken[0] = s->ivdc_taken[1] = INGENIC_ISP_SENSORS;
    timer_del(s->core_timer);
    s->core_kicked = false;
    s->core_busy = false;
    s->core_turn = 0;
    s->ipc_ready = 0;
    s->vic_closed = 0;
    memset(s->in_frame, 0, sizeof(s->in_frame));
    s->ivdc_held = 0;
    s->chans_set = 0;
    s->shadow_value = 0;
    s->shadow_pending = false;
    s->shadow_open = false;
    s->shadow_done = false;
    s->shadow_n = 0;
    s->shadow_due = 0;
    s->shadow_first = 0;
    s->shadow_idle_until = 0;
    memset(s->msca_busy_at, 0, sizeof(s->msca_busy_at));
    memset(s->msca_latched, 0, sizeof(s->msca_latched));
    memset(s->drop_pos, 0, sizeof(s->drop_pos));
    memset(s->drop_decided, 0, sizeof(s->drop_decided));
    memset(s->drop_skip, 0, sizeof(s->drop_skip));
    s->bus_last = 0;
    isp_raw_reset(s);
    isp_hldc_reset(s);
    s->hldc_stalled = false;
    s->keep_acc = 0;
    ingenic_isp_ldc_reset(&s->ldc);
}

static void ingenic_isp_reset_exit(Object *obj, ResetType type)
{
    IngenicIspState *s = INGENIC_ISP(obj);

    isp_update(s);
    qemu_set_irq(s->ldc_irq, ingenic_isp_ldc_irq(&s->ldc));
}

static void isp_csi_reset_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;

    if (s->csi_reset != !!level) {
        trace_ingenic_isp_csi(level);
    }
    s->csi_reset = level;
}

static void isp_ivdc_waiting_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;

    s->ivdc_waiting = level;
}

/* The IVDC's encoder @n / 4 took sensor @n % 4's frame */
static void isp_ivdc_taken_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;

    if (level) {
        s->ivdc_taken[n / INGENIC_ISP_SENSORS] = n % INGENIC_ISP_SENSORS;
    }
}

static void isp_mipi_switch_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;
    uint32_t pos = deposit32(s->mipi_switch, n, 1, level);

    if (pos != s->mipi_switch) {
        trace_ingenic_isp_switch(pos);
    }
    s->mipi_switch = pos;
}

static void isp_mclk_now(void *opaque, int n, int level)
{
    IngenicIspState *s = opaque;

    if (s->mclk_stopped != !level) {
        trace_ingenic_isp_mclk(level);
    }
    s->mclk_stopped = !level;
}

static void isp_soft_reset_now(void *opaque, int n, int level)
{
    if (level) {
        trace_ingenic_isp_reset();
        ingenic_isp_reset_hold(opaque, RESET_TYPE_COLD);
        ingenic_isp_reset_exit(opaque, RESET_TYPE_COLD);
    }
}

/* The lines from other devices wait for a frame whose pass is running */
#define ISP_LINE(name)                                                  \
    static void name(void *opaque, int n, int level)                    \
    {                                                                   \
        if (!isp_line_held(opaque, name##_now, n, level)) {             \
            name##_now(opaque, n, level);                               \
        }                                                               \
    }

ISP_LINE(isp_frame_in)
ISP_LINE(isp_clock)
ISP_LINE(isp_soft_reset)
ISP_LINE(isp_csi_reset)
ISP_LINE(isp_ivdc_waiting)
ISP_LINE(isp_ivdc_taken)
ISP_LINE(isp_mipi_switch)
ISP_LINE(isp_mclk)

static void ingenic_isp_finalize(Object *obj)
{
    isp_pool_free(INGENIC_ISP(obj));
    qemu_bh_delete(INGENIC_ISP(obj)->bh);
    qemu_cond_destroy(&INGENIC_ISP(obj)->idle);
    g_array_free(INGENIC_ISP(obj)->held, true);
    ingenic_hldc_free(&INGENIC_ISP(obj)->hldc[0]);
    ingenic_hldc_free(&INGENIC_ISP(obj)->hldc[1]);
    ingenic_frame_tap_free(&INGENIC_ISP(obj)->tap);
}

static void ingenic_isp_init(Object *obj)
{
    IngenicIspState *s = INGENIC_ISP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    const IngenicIspLayout *l = isp_layout(s);

    memory_region_init_io(&s->iomem, obj, &ingenic_isp_ops, s,
                          object_get_typename(obj),
                          l->iosize ?: INGENIC_ISP_IOSIZE);
    qemu_cond_init(&s->idle);
    s->held = g_array_new(false, false, sizeof(IngenicIspHeld));
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->isp_irq);
    for (unsigned n = 0; n < l->vics; n++) {
        sysbus_init_irq(sbd, &s->vic_irq[n]);
    }
    for (unsigned v = 0; v < INGENIC_ISP_VICS; v++) {
        s->vic[v].isp = s;
        s->vic[v].n = v;
        s->vic[v].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, isp_frame_timer,
                                       &s->vic[v]);
    }
    s->core_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, isp_core_timer, s);
    s->bh = qemu_bh_new(isp_due_bh, s);
    qdev_init_gpio_out_named(DEVICE(obj), s->ivdc_frame, "ivdc-frame",
                             INGENIC_ISP_SENSORS);
    qdev_init_gpio_out_named(DEVICE(obj), &s->ldc_irq, "ldc-irq", 1);
    if (!l->vics) {
        qdev_init_gpio_in_named(DEVICE(obj), isp_frame_in, "frame", 1);
    }
    qdev_init_gpio_in_named(DEVICE(obj), isp_clock, "clock", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_soft_reset, "reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_csi_reset, "csi-reset", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_mclk, "mclk", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_mipi_switch, "mipi-switch", 2);
    qdev_init_gpio_in_named(DEVICE(obj), isp_ivdc_waiting, "ivdc-waiting", 1);
    qdev_init_gpio_in_named(DEVICE(obj), isp_ivdc_taken, "ivdc-taken",
                            2 * INGENIC_ISP_SENSORS);
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

static const VMStateDescription vmstate_ingenic_isp_vic = {
    .name = TYPE_INGENIC_ISP "/vic",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, IngenicIspVic),
        VMSTATE_UINT32(frame, IngenicIspVic),
        VMSTATE_END_OF_LIST()
    }
};

static bool ingenic_isp_hldc_needed(void *opaque)
{
    return isp_layout(opaque)->hldc;
}

static const VMStateDescription vmstate_ingenic_isp_hldc_sub = {
    .name = TYPE_INGENIC_ISP "/hldc",
    .version_id = 2,
    .minimum_version_id = 2,
    .needed = ingenic_isp_hldc_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(hldc, IngenicIspState, 2, 1,
                             vmstate_ingenic_isp_hldc, IngenicIspHldc),
        VMSTATE_END_OF_LIST()
    }
};

static bool ingenic_isp_ldc_needed(void *opaque)
{
    return isp_layout(opaque)->ldc;
}

static const VMStateDescription vmstate_ingenic_isp_ldc_sub = {
    .name = TYPE_INGENIC_ISP "/ldc",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = ingenic_isp_ldc_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(ldc, IngenicIspState, 1, vmstate_ingenic_isp_ldc,
                       IngenicIspLdc),
        VMSTATE_END_OF_LIST()
    }
};

static bool ingenic_isp_latch_needed(void *opaque)
{
    return isp_layout(opaque)->nlatched || isp_layout(opaque)->shadow_kick;
}

static const VMStateDescription vmstate_ingenic_isp_latch_sub = {
    .name = TYPE_INGENIC_ISP "/latch",
    .version_id = 2,
    .minimum_version_id = 1,
    .needed = ingenic_isp_latch_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(latch, IngenicIspState, INGENIC_ISP_LATCHED),
        VMSTATE_UINT32_ARRAY(shd_val, IngenicIspState, INGENIC_ISP_SHD_RAM),
        VMSTATE_UINT32_ARRAY(shd_off, IngenicIspState, INGENIC_ISP_SHD_RAM),
        VMSTATE_UINT32(shd_n, IngenicIspState),
        VMSTATE_UINT32(shd_run, IngenicIspState),
        VMSTATE_BOOL(shd_open, IngenicIspState),
        VMSTATE_BOOL_V(core_idle, IngenicIspState, 2),
        VMSTATE_BOOL_V(core_wait, IngenicIspState, 2),
        VMSTATE_INT64_V(core_resumed, IngenicIspState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_gate_needed(void *opaque)
{
    const IngenicIspLayout *l = isp_layout(opaque);

    return l->gate_holds || l->chans_at_start;
}

static const VMStateDescription vmstate_ingenic_isp_gate_sub = {
    .name = TYPE_INGENIC_ISP "/gate",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_gate_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(bus_last, IngenicIspState),
        VMSTATE_INT64(gated_at, IngenicIspState),
        VMSTATE_INT64_ARRAY(vic_due, IngenicIspState, INGENIC_ISP_VICS),
        VMSTATE_UINT32_ARRAY(msca_chans, IngenicIspState, INGENIC_ISP_PIPES),
        VMSTATE_BOOL_ARRAY(msca_latched, IngenicIspState, INGENIC_ISP_PIPES),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_drop_needed(void *opaque)
{
    return isp_layout(opaque)->drop;
}

static const VMStateDescription vmstate_ingenic_isp_drop_sub = {
    .name = TYPE_INGENIC_ISP "/drop",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_drop_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT8_2DARRAY(drop_pos, IngenicIspState, INGENIC_ISP_PIPES,
                              INGENIC_ISP_CHANS),
        VMSTATE_UINT32_ARRAY(drop_decided, IngenicIspState,
                             INGENIC_ISP_PIPES),
        VMSTATE_UINT32_ARRAY(drop_skip, IngenicIspState, INGENIC_ISP_PIPES),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_mask_load_needed(void *opaque)
{
    const IngenicIspOsd *o = isp_layout(opaque)->osd;

    return o && o->mask_load;
}

static const VMStateDescription vmstate_ingenic_isp_mask_load_sub = {
    .name = TYPE_INGENIC_ISP "/mask-load",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_mask_load_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(mask_next, IngenicIspState, INGENIC_ISP_CHANS,
                               INGENIC_ISP_MASKS * 2),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_ch0_out_needed(void *opaque)
{
    return isp_layout(opaque)->ch0_out[0];
}

static const VMStateDescription vmstate_ingenic_isp_ch0_out_sub = {
    .name = TYPE_INGENIC_ISP "/ch0-out",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_ch0_out_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_INT64(ch0_from, IngenicIspState),
        VMSTATE_INT64(ch0_span, IngenicIspState),
        VMSTATE_UINT32(ch0_rest, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_push_latch_needed(void *opaque)
{
    return isp_layout(opaque)->push_latch;
}

static const VMStateDescription vmstate_ingenic_isp_push_latch_sub = {
    .name = TYPE_INGENIC_ISP "/push-latch",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_push_latch_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(ch0_latched, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_t41_stats_needed(void *opaque)
{
    IngenicIspState *s = opaque;
    const IngenicIspStats *st = isp_layout(s)->stats;

    return st && st->t41;
}

static const VMStateDescription vmstate_ingenic_isp_t41_stats_sub = {
    .name = TYPE_INGENIC_ISP "/t41-stats",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_t41_stats_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(gsm, IngenicIspState, 2, 256),
        VMSTATE_UINT32_2DARRAY(lce, IngenicIspState, 2,
                               INGENIC_ISP_LCE_BLOCKS *
                               (INGENIC_ISP_LCE_BINS + 1)),
        VMSTATE_UINT32_ARRAY(ram_at, IngenicIspState, INGENIC_ISP_RAM_PORTS),
        VMSTATE_UINT32(lce_bank, IngenicIspState),
        VMSTATE_BOOL(gsm_armed, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_mdns_needed(void *opaque)
{
    const IngenicIspMdns *m = isp_layout(opaque)->mdns;

    return m && m->rings;
}

static const VMStateDescription vmstate_ingenic_isp_mdns_sub = {
    .name = TYPE_INGENIC_ISP "/mdns",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_mdns_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(mdns_count, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_mdma_needed(void *opaque)
{
    return isp_layout(opaque)->mdma;
}

static const VMStateDescription vmstate_ingenic_isp_mdma_sub = {
    .name = TYPE_INGENIC_ISP "/mdma",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_mdma_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mdma_slot, IngenicIspState, INGENIC_ISP_VICS),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_mdma_kick_needed(void *opaque)
{
    IngenicIspState *s = INGENIC_ISP(opaque);

    for (unsigned v = 0; v < INGENIC_ISP_VICS; v++) {
        if (s->mdma_kick[v]) {
            return true;
        }
    }
    return false;
}

static const VMStateDescription vmstate_ingenic_isp_mdma_kick_sub = {
    .name = TYPE_INGENIC_ISP "/mdma-kick",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_mdma_kick_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(mdma_kick, IngenicIspState, INGENIC_ISP_VICS),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_mdma_fifo_needed(void *opaque)
{
    return isp_layout(opaque)->mdma_fifo;
}

static const VMStateDescription vmstate_ingenic_isp_mdma_fifo_sub = {
    .name = TYPE_INGENIC_ISP "/mdma-fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_mdma_fifo_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_2DARRAY(mdma_q, IngenicIspState,
                               INGENIC_ISP_VICS * 2, INGENIC_ISP_MDMA_FIFO),
        VMSTATE_UINT32_2DARRAY(mdma_done, IngenicIspState,
                               INGENIC_ISP_VICS * 2, INGENIC_ISP_MDMA_FIFO),
        VMSTATE_UINT8_2DARRAY(mdma_qn, IngenicIspState, INGENIC_ISP_VICS, 2),
        VMSTATE_UINT8_2DARRAY(mdma_dn, IngenicIspState, INGENIC_ISP_VICS, 2),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_raw_dma_needed(void *opaque)
{
    IngenicIspState *s = INGENIC_ISP(opaque);

    return s->wdma_raw || s->rdma_start || s->wdma_n || s->wdma_done_n ||
           s->rdma_done_n || s->wdma_head;
}

static const VMStateDescription vmstate_ingenic_isp_raw_dma_sub = {
    .name = TYPE_INGENIC_ISP "/raw-dma",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_raw_dma_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(wdma_q, IngenicIspState, INGENIC_ISP_RAW_FIFO),
        VMSTATE_UINT32_ARRAY(wdma_done, IngenicIspState,
                             INGENIC_ISP_RAW_FIFO),
        VMSTATE_UINT32_ARRAY(rdma_done, IngenicIspState,
                             INGENIC_ISP_RAW_FIFO),
        VMSTATE_UINT32(wdma_head, IngenicIspState),
        VMSTATE_UINT8(wdma_n, IngenicIspState),
        VMSTATE_UINT8(wdma_done_n, IngenicIspState),
        VMSTATE_UINT8(rdma_done_n, IngenicIspState),
        VMSTATE_BOOL(wdma_raw, IngenicIspState),
        VMSTATE_BOOL(rdma_start, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_switch_needed(void *opaque)
{
    return INGENIC_ISP(opaque)->switched;
}

static const VMStateDescription vmstate_ingenic_isp_switch_sub = {
    .name = TYPE_INGENIC_ISP "/switch",
    .version_id = 2,
    .minimum_version_id = 1,
    .needed = isp_switch_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(chans_set, IngenicIspState),
        VMSTATE_INT64_V(vic[0].served, IngenicIspState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_ivdc_needed(void *opaque)
{
    IngenicIspState *s = opaque;

    return s->ivdc_coding || s->ivdc_waiting || s->ivdc_held;
}

static const VMStateDescription vmstate_ingenic_isp_ivdc_sub = {
    .name = TYPE_INGENIC_ISP "/ivdc",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_ivdc_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_BOOL(ivdc_coding, IngenicIspState),
        VMSTATE_BOOL(ivdc_waiting, IngenicIspState),
        VMSTATE_INT64(ivdc_held, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_ivdc_taken_needed(void *opaque)
{
    IngenicIspState *s = opaque;

    return s->ivdc_taken[0] < INGENIC_ISP_SENSORS ||
           s->ivdc_taken[1] < INGENIC_ISP_SENSORS;
}

static const VMStateDescription vmstate_ingenic_isp_ivdc_taken_sub = {
    .name = TYPE_INGENIC_ISP "/ivdc-taken",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_ivdc_taken_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(ivdc_sent, IngenicIspState, INGENIC_ISP_SENSORS),
        VMSTATE_UINT32_ARRAY(ivdc_frame_no, IngenicIspState,
                             INGENIC_ISP_SENSORS),
        VMSTATE_UINT32_ARRAY(ivdc_taken, IngenicIspState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_turns_needed(void *opaque)
{
    return isp_layout(INGENIC_ISP(opaque))->vic_turns;
}

static const VMStateDescription vmstate_ingenic_isp_turns_sub = {
    .name = TYPE_INGENIC_ISP "/turns",
    .version_id = 2,
    .minimum_version_id = 1,
    .needed = isp_turns_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(core_timer, IngenicIspState),
        VMSTATE_BOOL(core_kicked, IngenicIspState),
        VMSTATE_BOOL_V(core_busy, IngenicIspState, 2),
        VMSTATE_UINT32(core_turn, IngenicIspState),
        VMSTATE_UINT32(ipc_ready, IngenicIspState),
        VMSTATE_UINT32_ARRAY(in_frame, IngenicIspState, INGENIC_ISP_SENSORS),
        VMSTATE_UINT32(vic[0].sub, IngenicIspState),
        VMSTATE_UINT32(vic[1].sub, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static bool isp_vic_out_needed(void *opaque)
{
    IngenicIspState *s = opaque;

    return s->vic_closed;
}

static const VMStateDescription vmstate_ingenic_isp_vic_out_sub = {
    .name = TYPE_INGENIC_ISP "/vic_out",
    .version_id = 1,
    .minimum_version_id = 1,
    .needed = isp_vic_out_needed,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(vic_closed, IngenicIspState),
        VMSTATE_END_OF_LIST()
    }
};

static int ingenic_isp_pre_load(void *opaque)
{
    IngenicIspState *s = opaque;

    isp_quiesce(s);
    s->due = 0;
    return 0;
}

/* A frame due but not yet run is a timer that fires as the state is loaded */
static int ingenic_isp_pre_save(void *opaque)
{
    IngenicIspState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    isp_quiesce(s);
    for (unsigned n = 0; n <= INGENIC_ISP_VICS; n++) {
        if (s->due & BIT(n)) {
            timer_mod(n < INGENIC_ISP_VICS ? s->vic[n].timer : s->core_timer,
                      now);
        }
    }
    s->due = 0;
    return 0;
}

static const VMStateDescription vmstate_ingenic_isp = {
    .name = TYPE_INGENIC_ISP,
    .version_id = 9,
    .minimum_version_id = 5,
    .pre_load = ingenic_isp_pre_load,
    .pre_save = ingenic_isp_pre_save,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, IngenicIspState, INGENIC_ISP_IOSIZE / 4),
        VMSTATE_STRUCT_2DARRAY(chan, IngenicIspState, INGENIC_ISP_PIPES,
                               INGENIC_ISP_CHANS, 1, vmstate_ingenic_isp_chan,
                               IngenicIspChan),
        VMSTATE_STRUCT_ARRAY(vic, IngenicIspState, INGENIC_ISP_VICS, 1,
                             vmstate_ingenic_isp_vic, IngenicIspVic),
        VMSTATE_UINT32(vic_running, IngenicIspState),
        VMSTATE_UINT32(shadow_value, IngenicIspState),
        VMSTATE_BOOL(shadow_pending, IngenicIspState),
        VMSTATE_BOOL_V(gated, IngenicIspState, 2),
        VMSTATE_BOOL_V(csi_reset, IngenicIspState, 3),
        VMSTATE_BOOL_V(mclk_stopped, IngenicIspState, 4),
        VMSTATE_UINT32_V(mipi_switch, IngenicIspState, 6),
        VMSTATE_UINT32_V(ivdc_input, IngenicIspState, 6),
        VMSTATE_UINT32_V(vic_switch, IngenicIspState, 7),
        VMSTATE_BOOL_V(shadow_open, IngenicIspState, 8),
        VMSTATE_BOOL_V(shadow_done, IngenicIspState, 8),
        VMSTATE_UINT32_V(shadow_n, IngenicIspState, 8),
        VMSTATE_UINT32_V(shadow_due, IngenicIspState, 8),
        VMSTATE_SUB_ARRAY(shadow_val, IngenicIspState, 0, 256, 8,
                          vmstate_info_uint32, uint32_t),
        VMSTATE_SUB_ARRAY(shadow_off, IngenicIspState, 0, 256, 8,
                          vmstate_info_uint32, uint32_t),
        VMSTATE_INT64_V(shadow_idle_until, IngenicIspState, 8),
        VMSTATE_INT64_ARRAY_V(msca_busy_at, IngenicIspState,
                              INGENIC_ISP_PIPES, 8),
        VMSTATE_SUB_ARRAY(shadow_val, IngenicIspState, 256,
                          INGENIC_ISP_SHADOW - 256, 9,
                          vmstate_info_uint32, uint32_t),
        VMSTATE_SUB_ARRAY(shadow_off, IngenicIspState, 256,
                          INGENIC_ISP_SHADOW - 256, 9,
                          vmstate_info_uint32, uint32_t),
        VMSTATE_UINT32_V(shadow_first, IngenicIspState, 9),
        VMSTATE_END_OF_LIST()
    },
    .subsections = (const VMStateDescription * const []) {
        &vmstate_ingenic_isp_hldc_sub,
        &vmstate_ingenic_isp_ldc_sub,
        &vmstate_ingenic_isp_latch_sub,
        &vmstate_ingenic_isp_gate_sub,
        &vmstate_ingenic_isp_drop_sub,
        &vmstate_ingenic_isp_mask_load_sub,
        &vmstate_ingenic_isp_ch0_out_sub,
        &vmstate_ingenic_isp_push_latch_sub,
        &vmstate_ingenic_isp_switch_sub,
        &vmstate_ingenic_isp_ivdc_sub,
        &vmstate_ingenic_isp_mdma_sub,
        &vmstate_ingenic_isp_mdma_kick_sub,
        &vmstate_ingenic_isp_mdma_fifo_sub,
        &vmstate_ingenic_isp_raw_dma_sub,
        &vmstate_ingenic_isp_ivdc_taken_sub,
        &vmstate_ingenic_isp_turns_sub,
        &vmstate_ingenic_isp_vic_out_sub,
        &vmstate_ingenic_isp_t41_stats_sub,
        &vmstate_ingenic_isp_mdns_sub,
        NULL
    }
};

/*
 * In a test, another thread saves or resets the device as a pass begins, as a
 * migration thread or the main loop would: it must find no frame half done
 */
static void *isp_act_main(void *opaque)
{
    IngenicIspState *s = opaque;

    bql_lock();
    if (!strcmp(s->act, "reset")) {
        resettable_reset(OBJECT(s), RESET_TYPE_COLD);
    } else {
        QIOChannelBuffer *b = qio_channel_buffer_new(0);
        QEMUFile *f = qemu_file_new_output(QIO_CHANNEL(b));

        vmstate_save_state(f, &vmstate_ingenic_isp, s, NULL, &error_abort);
        qemu_fclose(f);
        object_unref(OBJECT(b));
    }
    if (s->txn || s->unlocked || s->held->len) {
        s->act_bad++;
    }
    s->act_done++;
    bql_unlock();
    return NULL;
}

static void isp_act_start(IngenicIspState *s)
{
    QemuThread th;

    qemu_thread_create(&th, "isp-act", isp_act_main, s, QEMU_THREAD_DETACHED);
}

static const Property ingenic_isp_props[] = {
    DEFINE_PROP_BOOL("mipi-switch", IngenicIspState, switched, false),
    DEFINE_PROP_BOOL("x-mdns-avx2", IngenicIspState, mdns_avx2, true),
    DEFINE_PROP_BOOL("x-frame-bh", IngenicIspState, frame_bh, false),
    DEFINE_PROP_UINT32("x-bands", IngenicIspState, nbands, 0),
    DEFINE_PROP_UINT32("x-pass-delay-ms", IngenicIspState, pass_delay_ms, 0),
    DEFINE_PROP_STRING("x-act", IngenicIspState, act),
    DEFINE_PROP_UINT32("x-act-done", IngenicIspState, act_done, 0),
    DEFINE_PROP_UINT32("x-act-bad", IngenicIspState, act_bad, 0),
    DEFINE_PROP_UINT32("x-quiesce-waits", IngenicIspState, quiesce_waits, 0),
    DEFINE_PROP_UINT32("x-due", IngenicIspState, due, 0),
    DEFINE_PROP_UINT32("x-txn", IngenicIspState, txn, 0),
    DEFINE_PROP_STRING("x-poke", IngenicIspState, poke),
    DEFINE_PROP_UINT32("x-poke-passes", IngenicIspState, poke_passes, 0),
    DEFINE_PROP_UINT32("x-poke-misses", IngenicIspState, poke_misses, 0),
    DEFINE_INGENIC_FRAME_TAP_PROPERTY(IngenicIspState, tap),
    DEFINE_PROP_LINK("ncu", IngenicIspState, ncu, TYPE_INGENIC_T30_NCU,
                     IngenicT30NcuState *),
    DEFINE_PROP_LINK("core", IngenicIspState, core, TYPE_INGENIC_T20_ISP,
                     IngenicT20IspState *),
    DEFINE_PROP_LINK("cpm", IngenicIspState, cpm, TYPE_INGENIC_CGU,
                     Object *),
};

static void ingenic_isp_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    ResettableClass *rc = RESETTABLE_CLASS(oc);

    dc->vmsd = &vmstate_ingenic_isp;
    device_class_set_props(dc, ingenic_isp_props);
    rc->phases.hold = ingenic_isp_reset_hold;
    rc->phases.exit = ingenic_isp_reset_exit;
    for (unsigned i = 0; i < 256; i++) {
        isp_linear[i] = lrint(255.0 * pow(i / 255.0, 2.2));
        isp_linear_fine[i] = lrint(255.0 * 256 * pow(i / 255.0, 2.2));
        isp_linear12[i] = lrint(4095.0 * pow(i / 255.0, 2.2));
        isp_log13[i] = MIN(lrint(log2(isp_linear12[i] + 1.0) / 12 * 8192),
                           8189);
        isp_t41_adr_bin[i] = isp_linear12[i] ?
            MIN(MAX(20 + (int)floor(log2(isp_linear12[i] / 3072.0) / 0.35),
                    0), 20) : 0;
    }
}

static void ingenic_isp_soc_class_init(ObjectClass *oc, const void *data)
{
    const IngenicIspLayout *l = data;
    unsigned latched = 0;

    for (unsigned i = 0; i < l->nlatched; i++) {
        latched += l->latched[i].n;
    }
    assert(latched <= INGENIC_ISP_LATCHED);
    INGENIC_ISP_CLASS(oc)->layout = l;
    DEVICE_CLASS(oc)->desc = l->desc;
}

static const TypeInfo ingenic_isp_types[] = {
    {
        .name = TYPE_INGENIC_ISP,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(IngenicIspState),
        .instance_init = ingenic_isp_init,
        .instance_finalize = ingenic_isp_finalize,
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
