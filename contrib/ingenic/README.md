# Ingenic XBurst1 helper scripts

Companion scripts for booting and driving the Ingenic T-series machine
types (`ingenic-t10` through `ingenic-t33`) provided by this QEMU fork.
Distributed alongside the `qemu-system-mipsel` binary in release tarballs;
also usable in-tree from the build directory.

## Release binaries

The Linux binaries need glib, pixman and libslirp
(`libglib2.0-0 libpixman-1-0 libslirp0` on Debian and Ubuntu). The macOS
binary uses Homebrew's libraries:
`brew install glib pixman libslirp gmp gnutls jpeg-turbo libpng zstd libusb`.
The Windows build carries the DLLs it needs next to the `.exe`. The Linux
and macOS ones also carry `qemu-system-riscv32`, which runs the RISC-V
cores of the T31, T23, T32, T33, T40, T41 and A1 (see
[Zeratul fast boot](#zeratul-fast-boot)); Windows cannot share its memory
with them.

## Scripts

| Script           | Purpose |
|------------------|---------|
| `qemu-bsp.sh`    | Main launcher. Auto-detects machine type from `-V <variant>`, sets up SSH forwarding, console socket/TCP, USB storage. |
| `qemu-uboot.sh`  | Start QEMU, wait for U-Boot prompt, send commands, capture output. Useful for scripting flash provisioning. |
| `qemu_drive.py`  | Connect to a UART chardev (UNIX socket or TCP), feed commands from stdin, append output to a log file. Pairs with `qemu-bsp.sh -c` / `-T`. |
| `qemu-boot.exp`  | Expect-based wrapper: wait for U-Boot, interrupt autoboot. |
| `qemu-cmd.exp`   | Expect-based wrapper: send a single U-Boot command, capture response. |
| `isp-trace.py`   | Read the ISP and sensor trace events: each frame's changes, snapshots, diffs between two runs, waveforms, a live page. See [ISP trace](#isp-trace). |

## Quick start

Boot a thingino firmware image on T31:

```bash
./qemu-bsp.sh -i thingino-cam.bin
ssh -p 2230 root@localhost   # password: root
```

Boot a different SoC family by selecting a variant — the machine type is
auto-detected from the `-V` prefix:

```bash
./qemu-bsp.sh -i thingino-t20.bin -V t20x      # ingenic-t20
./qemu-bsp.sh -i thingino-t21.bin -V t21n      # ingenic-t21
./qemu-bsp.sh -i thingino-t23.bin -V t23n      # ingenic-t23
./qemu-bsp.sh -i thingino-t30.bin -V t30x      # ingenic-t30
./qemu-bsp.sh -i thingino-t31.bin -V t31x      # ingenic-t31 (default)
./qemu-bsp.sh -i thingino-t10.bin -V t10l      # ingenic-t10
```

Run `./qemu-bsp.sh -h` for the full option list.

## Supported variants

| Family | Variants |
|--------|----------|
| T10    | t10, t10l, t10n, t10a |
| T20    | t20n, t20x, t20l |
| T21    | t21n, t21l, t21z |
| T23    | t23n, t23x, t23dl, t23zn |
| T30    | t30n, t30x, t30l, t30a, t30z |
| T31    | t31n, t31x, t31l, t31a, t31zl, t31zx, t31al, t31zc, t31lc |
| C100   | c100 (a T31 machine) |
| T32    | t32nq, t32lq, t32xq, t32zn |
| T32Pro | t32prozl, t32prozn, t32prozx, t32provl, t32provn, t32provx, t32provnp, t32proa (a T32 machine) |
| CW080  | cw080sn, cw080snp (a T32 machine) |
| T33    | t33n, t33zl, t33zn |
| T40    | t40n, t40nn, t40xp, t40a |
| T41    | t41nq, t41lq, t41zn, t41xq, t41zx |
| A1     | a1n, a1l, a1x, a1a, a1nt |

The variant selects EFUSE/PRid bits and PLL config, read by the
firmware's `soc` tool and libimp, and how much DDR the part has: QEMU
warns when `-m` gives less, as firmware built for the part stops early
without it. Without `-m` each machine has its default RAM, and
`ingenic-t20` and `ingenic-t30` are a t20x and a t30x. The first 256 MiB
of RAM sit below the peripherals, all an XBurst1 SoC takes; on the T40,
T41 and A1 all of it shows again from 0x20000000, where Linux on a
512 MiB T40A or A1X finds the rest (at most 512 MiB on a T40 or T41,
1 GiB on an A1).

Every machine reports the model's own chip ID in eFuse words 0 to 2
unless `efuse-chipid` gives one, as 24 hex digits (words 0 to 2):
`-global ingenic-t31.efuse-chipid=a4063f37060780000c0a1914` on the T10 to
T33, `ingenic-t40.` on the T40 and T41, `ingenic-a1.` on the A1, so that
several machines in one test are told apart. The words Ingenic computes
from the chip ID follow it: the MXU licence in word 15 of a T20, T21, T30
or T31, and the words the fast-boot SDKs check (see
[Zeratul fast boot](#zeratul-fast-boot)). `efuse-file=<path>` loads a
chip's whole array instead, from a dump of its words (little-endian, from
word 0), in place of every word the model gives.

## Locating the QEMU binary

`qemu-bsp.sh` looks for `qemu-system-mipsel` in this order:

1. `$QEMU` environment variable
2. Same directory as the script (release tarball layout)
3. `../../build/qemu-system-mipsel` (in-tree dev layout)
4. `$PATH`

Set `QEMU=/some/other/path` to override.

## Useful environment overrides

```bash
QEMU=...      # path to qemu-system-mipsel
KERNEL=...    # path to uImage (required in linux mode)
UBOOT=...     # path to u-boot-spl ELF (required in uboot mode)
FLASH=...     # path to flash image (required in linux/uboot mode)
CMDLINE=...   # kernel command line (linux mode only; sensible default)
```

## Console scripting

```bash
# Expose UART1 as a UNIX socket and drive it from Python:
./qemu-bsp.sh -i fw.bin -c /tmp/qemu.sock &
echo "uname -a" | ./qemu_drive.py /tmp/qemu.sock

# Or as TCP for the /uart Claude skill:
./qemu-bsp.sh -i fw.bin -T 3331
```

## SPI NOR

The NOR part follows the image: the smallest of 16, 32 and 64 MiB that
holds it, answering RDID as a W25Q128, GD25Q256 or W25Q512. Firmware that
only knows the chip its board shipped with gets that one with `jedec-id`,
as datasheets print the ID: `-global ingenic-a1-sfc.jedec-id=0xc22019`
(a Macronix MX25L25645G) on the T40, T41 and A1,
`-global ingenic-t31-sfc.jedec-id=...` on the XBurst1 machines.

## SPI NAND

`nand-id=0xc851` puts a GigaDevice GD5F1GQ5UE (128 MiB) on the flash
controller in place of the NOR: `-global ingenic-t31-sfc.nand-id=0xc851`
on the XBurst1 machines (T10 to T33), `-global ingenic-a1-sfc.nand-id=0xc851`
on the T40, T41 and A1. `nand-part` names the part instead (any of the
SPI NAND parts thingino's U-Boot and vendor kernels carry, such as
`W25N01GV` or `MX35LF1GE4AB`), with its ID, geometry, planes, ECC status
bits and timing; `nand-blocks` overrides its size in blocks. The image
holds the page data back to back, as thingino's and Ingenic's NAND images
do; `nand-layout=raw` takes a programmer's dump instead, each page
followed by its spare area. Pad the image to the chip size with 0xff, or
what the guest writes past its end is lost when QEMU exits.

Reads, programs and erases keep the chip busy for the part's tRD, tPROG
and tBERS (`nand-busy=off` ends them at once). `nand-bad-blocks=3,40`
marks blocks bad (the factory marker; they also fail erases and
programs), and `nand-ecc-correctable` and `nand-ecc-uncorrectable` take
lists of pages whose reads report corrected or uncorrectable bit errors
until their block is erased. All three can also be set while the guest
runs, with `qom-set /machine/soc/sfc nand-bad-blocks 7` in the monitor.

```bash
head -c $(( (128 << 20) - $(stat -c %s fw.bin) )) /dev/zero | tr '\0' '\377' >> fw.bin
qemu-system-mipsel -M ingenic-t41 -m 128M -display none \
    -global ingenic-a1-sfc.nand-id=0xc851 \
    -drive file=fw.bin,format=raw,if=none,id=flash0 \
    -serial null -serial stdio
```

## Boot ROM and secure boot

Without `-bios` the machine loads the SPL from the flash image itself.
With `-bios` it runs a chip's real 32 KiB boot ROM instead (mapped at
0x1fc00000, the CPU starting at 0xbfc00000, its clock 24 MHz until the
SPL sets the PLLs), which boots as the chip does: from the source its
boot-select pads pick, then through its secure boot when the eFuse turns
it on. The pads are PC's (PE's on the T20), set with `-global
ingenic-gpio.in-pc=<mask>` (see [Test inputs](#test-inputs)): `0xfffffffd`
(PC1 low) boots a T31 from SPI NOR, `0xfffffbfd` a T32 from its SD card.
`ingenic-gpio.pinval=<mask>` sets every port's pads at once: with
`0xfffffffd` PA1, PB1 and the rest read low too, whatever the board wires
to them.

Secure boot is off unless the eFuse says otherwise:

| SoC | Security fuse | Key hash |
|---|---|---|
| T10 to T33 (`ingenic-t31.`) | `efuse-security=` is word 4; bit 16 turns secure boot on (a T32NQ with it on reads 0x21030000, a T31X 0x01000000) | `efuse-keyhash=`, words 16 to 23 |
| T40, T41 (`ingenic-t40.`) | `efuse-security=` (a T40N and T40XP read 0x01010000, the default; a Wyze Cam 4's T41NQ 0x00000003) | `efuse-keyhash=`, at 0x280 |

The key hash is the SHA-256 of the 256-byte RSA modulus the vendor's SPL
carries at 0x300, 64 hex digits. With it fused the ROM hashes the SPL
on the SHA engine and checks its RSA-PSS signature on the RSA engine,
both modelled, and boots it only if they match. A ROM that refuses an
SPL stops: the T31's blinks PB23 and PB24, and the T32's prints a code on
UART1 for each attempt (2349 accepted, 234b key hash mismatch, 234f bad
signature). On the T32, `efuse-fdekey=` sets words 40 to 47, the key its
vendor's dm-crypt reads.

A T31LC leaves the factory with secure boot on, so only Ingenic's signed
SPL boots it. That SPL also starts the T31's RISC-V core (`-R` runs it,
see [Zeratul fast boot](#zeratul-fast-boot)) with a firmware that loops
for as long as the chip runs, watching the ISP's three output sizes and
the encoder: above 2304x1296 it gates the ISP's clock (CLKGR bit 23),
and when bit 28 of the encoder's 0x13208200 is set, the encoder's (CLKGR1
bit 0). A T31LC image, its own key fused:

```bash
key=$(python3 -c "import hashlib,sys; print(hashlib.sha256(open(sys.argv[1],'rb').read()[0x300:0x400]).hexdigest())" t31lc.bin)
qemu-system-mipsel -M ingenic-t31 -m 64M -display none \
    -global ingenic-t31.soc-variant=t31lc \
    -bios t31_bootrom.bin -global ingenic-gpio.in-pc=0xfffffffd \
    -global ingenic-t31.efuse-security=0x01010000 \
    -global ingenic-t31.efuse-keyhash=$key \
    -drive file=t31lc.bin,format=raw,if=none,id=flash0 \
    -serial null -serial stdio
```

## Video

The camera pipeline of every T-series SoC and of the T40 and T41 is
emulated, from an image sensor on I2C through the ISP to the hardware
encoders, so the vendor drivers and `libimp` stream H.264, H.265 and
JPEG unmodified. Give the board its sensor with `-device image-sensor`:

```bash
./qemu-bsp.sh -i thingino-t31.bin -V t31x -E "-device image-sensor,model=gc2053,bus=i2c0"
```

The sensor answers at its usual I2C address with the chip ID its vendor
driver probes for, and sends frames while that driver has it streaming.
On the T21, T23, T31, T32 and T33 it also needs its MCLK, which tx-isp
stops while the ISP is closed, and on all of them but the T21 a MIPI
sensor's frames need the CSI receiver out of reset, which tx-isp leaves
it only while the ISP is open. It sees eight colour bars over a luma
ramp, crossed by a white stripe every two seconds. Its options:

- `model`: cv2003, cv2005, cv8001, gc1084, gc2023, gc2053, gc2063, gc2083,
  gc3003a, gc4023, gc4653, gc4663, gc5603, gc5613, imx307, imx327,
  jxf22, jxf23, jxf28, jxf37, jxh42, jxh62, jxh63p, jxk04, jxq03,
  jxq03p, mis2008, mis20c1, os02b10, os02g10, os02n10, os03b10, os04d10,
  os05a10, os08a10, ov2710, ov2735b, ov9712, ov9732, ov9750, ps5250,
  ps5260, sc1235, sc1a4t, sc2135, sc2232, sc2232h, sc2235, sc223a,
  sc2300, sc2310, sc231hai, sc2331, sc2332, sc2336, sc2336p, sc301iot,
  sc3235, sc3332, sc3335, sc3336, sc3338, sc401ai, sc4236, sc4336,
  sc4336p, sc500ai, sc5235, sc5336, sc8238, sc830ai or sc835hai
- `bus`: the I2C bus the board's sensor sits on: `i2c0` on most
  cameras, `i2c1` on some T40 boards
- `address`: another I2C address, for a board that straps the sensor
  differently (the model's usual one by default): 0x36 for the imx327
  of thingino's T23, T30 and T31 drivers
- `fps`: the frame rate, 30 by default
- `input`: the video input it feeds, 0 by default: on a board with more
  than one sensor, the VIC each sensor's frames go through. Input 1 sees
  the scene mirrored, 2 upside down and 3 both, so that a stream shows
  which sensor sent it
- `noise=on`: luma noise that changes every frame, for pictures an
  encoder cannot code small
- `light`: the scene's brightness in percent, 100 by default: a well-lit
  room. The gc2053, gc2063, os02g10, os08c10 and sc4236 follow their
  exposure registers, so the picture brightens and darkens with what AE
  sets, in linear light, and AE answers `light`; it can change while the
  guest runs (given `id=cam`, `qom-set /machine/peripheral/cam light 5`
  for dusk)

A board's only sensor feeds whichever VIC its driver starts. The T40
also takes a second sensor, through VIC 1 into its second pipeline with
channels and statistics of its own, as an eufy T8416 runs its sc830ai and
sc3338 in its dual (allcached) mode:

```bash
-device image-sensor,model=sc830ai,bus=i2c1 \
-device image-sensor,model=sc3338,bus=i2c3,address=0x32,input=1
```

A third sensor (`input=2`, the T40's DVP port) feeds VIC 2, which has no
pipeline: its frames reach memory only raw, through the VIC's own capture,
as libimp's ISP bypass and tx-isp's `snapraw` take them (every VIC has
one: `echo snapraw 0 1 > /proc/jz/isp/isp-w02` saves a raw frame of
sensor 0). Raw frames are 16 bits a pixel, the sensor's bits at the top,
in an RGGB pattern. Joint mode, two sensors' pictures in one buffer,
needs nothing more: the driver sets both pipelines' channels into it.

A T23 runs two or three sensors through its one VIC behind a MIPI switch,
the sensors taking turns frame by frame. Its dual-sensor tx-isp has the
VIC flip the switch itself, on the pin its `mipi_switch_gpio` parameter
names (GPIO 7 by default); given the switch's GPIOs instead (libimp's
`IMP_ISP_MultiCamera_SetSwitchgpio`, raptor's `[mipi_switch]`, which a
third sensor needs), it flips those after every frame. Give the SoC the
switch's GPIO with `-global ingenic-t31.mipi-switch=N` (`mipi-switch2`
for a third sensor) and each sensor, as its input, the switch position
that selects it; each streams at half the rate given (a third with
three). A jooan W8U, its main sensor at 0 on GPIO 7:

```bash
-device image-sensor,model=sc2336p,bus=i2c0,fps=30 \
-device image-sensor,model=sc2336p,bus=i2c0,address=0x32,input=1,fps=30 \
-global ingenic-t31.mipi-switch=7
```

Three sensors, as the T23 SDK 1.3.0's `sample-Encoder-Treble-Ivdc` runs
them on the dual-sensor tx-isp (switch GPIOs 6 and 7 in states 1, 2 and
3, all three mains over IVDC with `direct_mode=3`; the sample's second
and third sensors moved from I2C1 to I2C0):

```bash
-device image-sensor,model=jxh63p,bus=i2c0,address=0x40,input=1,fps=45 \
-device image-sensor,model=jxh63p,bus=i2c0,address=0x42,input=2,fps=45 \
-device image-sensor,model=jxh63p,bus=i2c0,address=0x46,input=3,fps=45 \
-global ingenic-t31.mipi-switch=6 -global ingenic-t31.mipi-switch2=7
```

The driver counts each frame in two interrupts and puts a frame it was
not ready for into the other sensor's buffers, so here VIC 0 holds each
frame until the guest has served the last one, set each channel up for
the next sensor and queued its buffers: the frame rate follows the guest.
The W8U's thingino profile streams all four channels as it is (direct
mode 0). In direct mode 2 the two 1080p mains share the IVDC ring, and
the frames wait for the Helix too (below): mains and subs come out at the
same rate, the guest's.

The T32, T33 and T32 Pro run up to four sensors through their two VICs,
each with two inputs behind a switch of its own: inputs 0 and 1 on I2C0
and I2C1 at the sensor's own address, inputs 2 and 3 on the same buses
at its second one (0x3f for a gc2063, 0x10 for a gc5613), as the SDK's
three- and four-sensor demos have them. The core
takes their frames in turn, one per start from the driver (0x2c on a
T32, its top register list on a T33 and a T32 Pro). The T33's three gc2063:

```bash
-device image-sensor,model=gc2063,bus=i2c0 \
-device image-sensor,model=gc2063,bus=i2c1,input=1 \
-device image-sensor,model=gc2063,bus=i2c0,address=0x3f,input=2
```

The encoders behind them:

| SoC                       | Encoders                 | Output             |
|---------------------------|--------------------------|--------------------|
| T10, T20, T21, T23        | HELIX                    | H.264, JPEG        |
| T30                       | HELIX, RADIX             | H.264, H.265, JPEG |
| T31, T40                  | Allegro EL150            | H.264, H.265, JPEG |
| T41                       | Allegro EL200            | H.264, H.265, JPEG |
| T32, T33, T32Pro, CW080   | HERA, JPEG engine        | H.264, H.265, JPEG |

The IPU blends `libimp`'s OSD, and the ISP draws its own on the T23,
T32, T33, T40 and T41, as each of them draws it. In the ISP's direct
mode (`direct_mode=1`) the T23, T32, T33 and T41 send the main channel to
the encoder through the IVDC line ring rather than through memory. The
encoders run on the host CPU, so a 4K stream or a noisy one may come out
below the sensor's frame rate.

Over IVDC a late picture is dropped, not delayed: the T23's driver
cancels a job that starts more than 500 lines into its frame (dmesg
`valid_data_line = ...`), 50 ms after it (`time = ...`) or after an
overflow (`overflow_cnt = ...`), and `libimp` then predicts the pictures
after it from the wrong rows of its reference ring until its next IDR.
So on the T23 a frame's lines wait for the Helix to take it, and the ISP
holds its next frame while the Helix codes the last (up to a frame time)
or the IVDC waits (up to 50 ms): the rate follows the encoder and the
guest, and a cancel comes from the guest falling further behind. The
OSD has to be the ISP's there: the frames reach `libimp` with no buffer,
at physical 0, and its IPU OSD writes over the kernel there.
`-trace 'ingenic_ivdc_*'` shows each frame, how many lines in an encoder
took it and the line counts the T23's driver checks;
`-trace 'ingenic_helix_*'` each Helix job's size, its source, reference
and reconstruction addresses, host time and macroblock counts.

How the stream leaves the camera is up to its firmware. A thingino
image with a wired interface serves RTSP on port 554: forward it when
QEMU starts (the Ethernet MAC takes the netdev `n0`) and watch:

```bash
qemu-system-mipsel -M ingenic-t31 -m 128M -display none \
    -drive file=fw.bin,format=raw,if=none,id=flash0 \
    -serial null -serial stdio \
    -netdev user,id=n0,hostfwd=tcp::8554-:554 \
    -device image-sensor,model=gc2053,bus=i2c0
ffplay -rtsp_transport tcp rtsp://thingino:thingino@127.0.0.1:8554/ch0
```

### Stream tap

Whatever the firmware does with its streams, the encoders can write them
to the host, with nothing in the guest changed: a stock image whose
streams go only to its vendor's cloud is watched the same way. Set
`x-stream-tap` to a directory on the encoder (the property is
experimental, and off by default; off, an encoder does nothing for it):

```bash
-global ingenic-avpu.x-stream-tap=/tmp/tap     # T31, T40, T41
-global ingenic-helix.x-stream-tap=/tmp/tap    # T10 to T30, the T41's JPEG engine
-global ingenic-hera.x-stream-tap=/tmp/tap     # T32 (and ingenic-jpeg, its JPEG)
-global ingenic-radix.x-stream-tap=/tmp/tap    # T30's H.265
```

Each stream the guest codes becomes `<encoder>-<n>.h264`, `.h265` or
`.mjpg` there (JPEG files back to back; a snapshot is one), `<n>` counting
the encoder's streams from 0 in the order their first pictures are
written. The file grows with every picture; follow it with

```bash
ffplay -f h264 -follow 1 -i file:/tmp/tap/avpu-0.h264
```

or make a FIFO at the stream's path first (`mkfifo /tmp/tap/avpu-0.h264`)
and play that. A FIFO gets pictures once a player has it open, from an IDR
picture on, and drops them, to resume at the next IDR picture, while the
player is behind: the guest never waits for it. A video file starts at the
first IDR picture that comes with its parameter sets.

The output is what the guest's application receives. A core writes only an
H.264 or H.265 picture's slice data; the parameter sets and the slice
header are libimp's, so the tap joins both:

* AVPU (T31, T40, T41): libimp writes the headers into the stream buffer
  right before the slice data's offset (the command list's), H.264 before
  the job (`0x220` bytes are left for them), H.265 after it. The stream is
  the command list's channel buffer.
* HELIX (T10 to T23, T30 JPEG), HERA (T32), RADIX (T30): libimp keeps no
  header in front of the core's window. It copies the slice data (the
  window is shared by all channels) and builds the NALs, with the 03
  bytes, in its own stream buffer. The tap finds the escaped slice data in
  the guest memory written since its last look, and takes the NALs' start
  code and header from before it. Where libimp's ring is in user-space
  pages (the T30's SDK demo), the picture lies in pages that are not next
  to each other in RAM, and the tap follows it from each page to the one
  that starts with the next bytes. The stream is the
  channel's reconstruction ring (HELIX where it has one). Where the core
  adds the 03 bytes itself (the T33's HERA lists ask for it), libimp adds
  none and the tap looks for the slice data as the core wrote it.
* JPEG: the AVPU and the T32's engine write the whole file. HELIX writes
  the scan; libimp's JFIF header and EOI are found round it.

A picture is written once the bytes before its data are a slice NAL's start
code and header, with the parameter sets that go with an IDR picture
(`ingenic_enc_tap_*` trace events show each picture and each drop); one
that does not show up in memory within 8 seconds of guest time is dropped
with a warning (a busy host starves the guest, and libimp hands a picture
over more than a second late then; the trace shows how long each waited). Streams that share an encoder and a size, on an encoder
that cannot tell them apart (a HELIX without a ring), are one stream. On,
the tap costs a copy of each picture, a look at the pages written in the
last 2 ms of guest time, and the guest's first write to each page after a
look going the slow way.

### Frame tap

The ISP's pictures can be written to the host the same way, ahead of any
encoder: each ISP channel's picture and the raw frame of each VIC. Set
`x-frame-tap` to a directory on the ISP (experimental, off by default; off,
the ISP does nothing for it and renders nothing extra):

```bash
-global ingenic-isp.x-frame-tap=/tmp/tap       # T21 to T41, and the T30's multi-scaler
-global ingenic-t20-isp.x-frame-tap=/tmp/tap   # T10, T20, and the T30's Apical core
```

A T30 has both devices: the channels come from the multi-scaler
(`ingenic-isp`), the raw frame from the Apical core (`ingenic-t20-isp`).

* `isp-<pipe>-<chan>.y4m`: each channel's picture as the ISP puts it out
  (its crop window of the scaled frame, with the overlays), the NV12 the
  guest's buffer gets written as a YUV4MPEG2 stream, 4:2:0 planar, at
  the sensor's frame rate. A channel an IVDC takes straight to the encoder
  (direct mode: T23, T32, T33, T41) writes no buffer, but its picture is
  drawn for the tap all the same. A T10 or T20's channels are its FR, DS1
  and DS2 writers, 0 to 2.
* `raw-<n>-<W>x<H>-<pixfmt>.raw`: the frame VIC `<n>` takes from the
  sensor, as 16-bit little-endian samples, frames back to back, the
  sensor's bits (the VIC's 8, 10 or 12) left justified in 16 with the low
  bits 0, the way the VIC's raw capture writes them. `<pixfmt>` is ffmpeg's
  name for the Bayer order: `bayer_rggb16le` on the T21 to T41, the
  order the core is set to on a T10, T20 or T30 (10-bit samples, before
  any black level or gain).

A file starts with the channel's or VIC's first frame and grows with each
one. A new size (the guest reconfigures a channel) goes on in a new file:
`isp-0-0-1.y4m`, the next size in a `.raw`'s name. Play them while the
guest runs:

```bash
ffplay -follow 1 -i file:/tmp/tap/isp-0-0.y4m
ffplay -follow 1 -framerate 25 -f rawvideo -video_size 1920x1080 \
    -pixel_format bayer_rggb16le -i file:/tmp/tap/raw-0-1920x1080-bayer_rggb16le.raw
```

The raw file's size and pixel format are in its name; ffmpeg and ffplay
debayer it themselves (`-pixel_format bayer_rggb16le`). The y4m header
carries the sensor's frame rate, so a channel that drops frames plays
faster than the guest sees it. At 1080p a file grows by 3 MB a frame (raw:
4 MB) as fast as the guest runs, so for more than a few seconds use a FIFO
(below) and a player or `ffmpeg -frames:v N`.

A FIFO made beforehand at `isp-<pipe>-<chan>.y4m` or `raw-<n>.raw` is used
instead of a file: it gets the stream (a `.y4m` with its header first) once
a reader has it open, drops a frame that finds the last not yet read, and
never makes the guest wait. `mkfifo /tmp/tap/isp-0-0.y4m` and then
`ffplay /tmp/tap/isp-0-0.y4m`, or
`ffmpeg -i /tmp/tap/isp-0-0.y4m -frames:v 5 f%02d.png` for five frames.

Cost: off, nothing. On, each channel's picture is copied (and converted to
planar); the raw frame, and the picture of a channel an IVDC takes, are
rendered a second time, which at 1080p and up can take longer than the
guest's frame time. A render over 10 ms of host time is followed by a pause
as long, in which those frames are skipped, so the tap takes half the host
at most; on a slow host the raw file has fewer frames than the guest's VIC
saw. A FIFO or file nobody reads is not written to.

## Test inputs

Buttons, card-detect switches and light sensors are inputs a test drives
from outside, through QOM properties of the SoC. `in-pa` to `in-pf` are
the levels outside drives onto each port's pads, one bit per pin, all
high (pulled up) until set; a pad set up as a GPIO output reads what it
drives. A switch that is closed from power-on, such as the card-detect
switch of a card in its slot, is set with `-global`, so U-Boot sees it
too; a reset keeps these levels, as it does not reach outside the chip:

```bash
qemu-system-mipsel -M ingenic-t41 ... \
    -drive file=card.img,format=raw,if=sd \
    -global ingenic-gpio.in-pb=0xfbffffff         # PB26 low: a Wyze Cam v4's card is in
```

While the guest runs, set them from the QEMU monitor; with the console on
stdio, put the monitor on a socket
(`-monitor unix:qemu.mon,server=on,wait=off`, or `-E` that to
`qemu-bsp.sh`) and reach it with `socat - unix:qemu.mon`:

```bash
(qemu) qom-set /machine/soc/gpio in-pb 0xefffffff   # PB28 low: press a button
(qemu) qom-set /machine/soc/sadc aux0 300           # darken channel 0's photoresistor
```

`aux0` to `aux7` are the 12-bit samples of the SADC's channels, 2025 by
default: a bright room to a photoresistor.

## Pin trace

`-global ingenic-gpio.trace=<chardev>` logs every pin whose function or
level changes, one JSON line each:

```bash
qemu-system-mipsel -M ingenic-t31 ... \
    -chardev file,id=gpiotrace,path=gpio.jsonl -global ingenic-gpio.trace=gpiotrace
```

```json
{"t":0,"reset":true,"ports":3}
{"t":488804570,"pin":"PA23","gpio":23,"fn":"func","func":1,"pc":"0x87fac260"}
{"t":11904180350,"pin":"PA18","gpio":18,"fn":"out","lvl":1,"pc":"0x80017874"}
{"t":80215212363,"pin":"PB28","gpio":60,"fn":"irq","trig":"low","mask":0,"lvl":0,"src":"ext"}
```

`t` is QEMU's virtual time in ns. `fn` is what the pin now is: `func`, a
device function (`func` 0 to 3); `out` or `in` with its level; or `irq`
with its trigger (`low`, `high`, `falling` or `rising`), `mask` and
level. `pc` is the guest PC that made the change, as of the start of its
translation block; `"src":"ext"` marks a level driven from outside, by a
test input or a device on the wire. A reset writes the `reset` line, and
a pin keeps its reset state until it is logged.

## PWM trace

The timer unit's channels and the PWM block's (T32, T33, T41, A1) compute
the level each would drive at its pad, as a T31X, T40XP, T41NQ and T32LQ
drive theirs at U-Boot. Each channel's output reads as QOM properties,
`pwmN-level`, `pwmN-period-ns` and `pwmN-high-ns` (0 while the level
holds), under `/machine/soc/tcu` and `/machine/soc/pwm`, and with `trace`
set to a chardev each change is a JSON line:

```bash
qemu-system-mipsel -M ingenic-t41 ... \
    -chardev file,id=tcutrace,path=tcu.jsonl -global ingenic-tcu.trace=tcutrace \
    -chardev file,id=pwmtrace,path=pwm.jsonl -global ingenic-pwm.trace=pwmtrace
```

```json
{"t":0,"reset":true,"channels":16}
{"t":48224695078,"pwm":0,"lvl":0,"period":3799188,"high":1424695,"edge":2372209,"pc":"0x55ecd7c0"}
{"t":49269497394,"pwm":0,"lvl":0,"period":3799188,"high":2374492,"edge":1396908,"src":"hw"}
{"t":50291474965,"pwm":0,"lvl":1,"src":"hw"}
```

`lvl` is the level now; while the output toggles, `period` and `high` are
its period and high time in ns and `edge` the ns until its level next
changes, so a line describes every edge until the next line for that
channel (`"tcu":N` for the timer unit's). `pc` is the guest PC behind the
change, `"src":"hw"` a change the block makes itself, as a stop at the end
of a period or a `PWMUPDATE` taking effect.

`pin-trace.py` reads the GPIO, timer unit and PWM traces together:

```bash
contrib/ingenic/pin-trace.py log gpio.jsonl pwm.jsonl
contrib/ingenic/pin-trace.py vcd gpio.jsonl pwm.jsonl -o pins.vcd
contrib/ingenic/pin-trace.py serve gpio.jsonl pwm.jsonl
```

`log` prints each change in time order, `vcd` the pins and every PWM edge
as waveforms (a pin in a device function as `z`), beside each channel's
share of time high as a real signal, and `serve` a live page of the PWM
outputs' frequency and duty and the pins' state, following the files as
they grow.

## ISP trace

The ISP models' trace events show what a driver programs and what the
hardware does with it, for the stock tx-isp and any other driver alike:

- `ingenic_isp_*`, the T21's, T23's, T31's, T32's, T33's, T32 Pro's,
  T40's and T41's ISP and the T30's multi-scaler: `_read` and `_write`,
  each register access with the guest PC behind it (as of the start of
  its translation block) and the register's name where the model knows
  it (`ch1.y_push`, `p1.int_sta` for pipeline 1, `vic0.ctrl`, `ae+0x2c`,
  `osd.pic0.3+0x4`); `_shadow`, each (value, offset) pair through the
  shadow port as the register it lands in; `_rcfg` and `_rcfg_write`, the
  register lists a T33's or T32 Pro's core loads and what they write;
  `_frame`, `_vic` and `_vic_wait`, frames as they start, VICs starting
  and stopping and why one takes no frame (its sensor in standby or
  without its clock, the CSI receiver held in reset, no sensor);
  `_raise`, `_irq` and `_vic_irq`, the status bits the hardware sets and
  the interrupt lines; `_chan_push`, `_chan_done`, `_chan_drop`,
  `_chan_pop`, `_chan_clear` and `_ivdc`, channel buffers queued, filled
  or dropped (and why), read back and cleared, and frames over IVDC;
  `_clock`, `_reset`, `_csi`, `_mclk` and `_switch`, the ISP's clock gate
  and soft reset, the CSI receiver's reset, the sensor's clock and a MIPI
  switch
- `ingenic_t20_isp_*`, the T10's, T20's and T30's Apical ISP: `_read`
  and `_write` (the VIC, the glue's `top.*`, the core's interrupt slots
  and ports and its writers by name), `_vic`, `_frame` (a frame's start
  and, half a frame later, its output), `_event` (an Apical event and the
  interrupt slots it latches in), `_writer` (a video DMA writer moving to
  its next bank) and `_irq`; `ingenic_t30_ncu_write` the T30's noise
  reducer
- `image_sensor_*`: `_write` and `_read`, each register by its address (a
  banked part's as page << 8 | register), `_stream`, frames on or off,
  and `_exposure`, the integration time and gain as they change, on the
  parts whose exposure registers are known

The model applies a shadow write at once: when silicon latches one, at
frame start or a sync, is not modelled. Reads are most of a log (the
Apical firmware polls): `-trace '-ingenic_isp_read'` after the pattern
leaves them out.

`isp-trace.py` reads such a log:

```bash
qemu-system-mipsel -M ingenic-t31 ... -device image-sensor,model=gc2053,bus=i2c0 \
    -msg timestamp=on -trace 'ingenic_isp_*' -trace 'image_sensor_*' -D isp.log
contrib/ingenic/isp-trace.py log isp.log
contrib/ingenic/isp-trace.py snapshot isp.log --at frame:30
contrib/ingenic/isp-trace.py diff stock.log other.log --at stream
contrib/ingenic/isp-trace.py vcd isp.log -o isp.vcd
contrib/ingenic/isp-trace.py serve isp.log
```

A register's state there is what was last written to it, by the driver,
through the shadow port or by a register list. `log` prints each frame's
events as they come and then every register the frame left changed, from
its value before the frame to its last (`--writes` every write, `--pc`
the PC behind it, `--irq`, `--channels` and `--reads` more). `snapshot`
gives each register's value at a point, `stream` (as a VIC starts),
`frame:N` (as VIC 0's frame N starts) or `end`, and how and in which
frame it got there. `diff` compares two logs' snapshots at the same
point, as a stock driver against another for the same camera would be:
the values that differ say what one sets and the other does not, rather
than in what order. Both leave out the registers that hold no state
(status, clears, FIFO pushes, ports) unless `--all`; `diff
--no-addresses` leaves out buffer addresses, which differ between any two
runs, and `--ignore` whatever else matches. `vcd` writes the registers that change as waveforms, beside the
frame counts and the sensor's exposure and gain. `serve` shows the
registers on a live page as the log grows: those changed this frame and
recently highlighted, the events and the exposure. With `/proc/modules`
and a module's symbols, a PC names the driver function behind a write.

## Wi-Fi

AICSemi's AIC8800, AIC8800D80, AIC8800DC and AIC8800DW, AltoBeam's
ATBM60xx, Broadcom's BCM43438 and BCM43436, HiSilicon's Hi3881 and
WS73V100, Huge-IC's TXW901U, iComm-semi's SSV6155P, SSV6158 and SSV6355,
MediaTek's MT7601U, Realtek's RTL8189FS, RTL8189ES, RTL8192FS,
RTL8188FU, RTL8188EUS and RTL8733BU, Seekwave's SWT6621S, and WuQi's
WQ9001 chips are emulated as their Linux drivers see them, with a WPA2
access point on the air for the camera to join.
The firmware a driver uploads is accepted and ignored; the model
answers in its place. SDIO parts go on
the controller the camera wires them to, usually `msc1`: the MSC on the
T10 to T31, the SDHCI on the T32, T33, T40 and T41. USB parts go on the
OTG port, with `port=1` so no hub comes between:

| Device | Bus | Driver (thingino package) |
|---|---|---|
| `aic8800` | USB | aic8800 with the AIC8800 USB model (cameras named aic8800d) |
| `aic8800d80` | USB | aic8800 with the AIC8800D80 USB model (cameras named aic8800u) |
| `aic8800d80-sdio` | SDIO | aic8800 with the AIC8800D80 SDIO model (cameras named aic8800d40) |
| `aic8800dc`, `aic8800dw` | USB | aic8800 with the AIC8800DC USB model (cameras named aic8800dc; a DW is a DC without Bluetooth) |
| `atbm6031` | SDIO | atbm6031, atbm6031x |
| `atbm6062s`, `atbm6162s` | SDIO | atbm6062s, atbm6162s (Wi-Fi 6 driver) |
| `atbm6012b`, `atbm6012bx`, `atbm6032`, `atbm6132u` | USB | same names |
| `atbm6062u`, `atbm6062cu`, `atbm6132cu`, `atbm6132cu-6162` | USB | same names (Wi-Fi 6 driver) |
| `mt7601u` | USB | mt7601u, mt7601sta (MediaTek's STA driver; cameras named mt7601, mt7601u, mt7601sta) |
| `rtl8189fs`, `rtl8189es`, `rtl8192fs` | SDIO | rtl8189fs, rtl8189es, rtl8192fs (cameras named rtl8189ftv, rtl8189etv) |
| `rtl8188fu` | USB | rtl8188fu (cameras named rtl8188ftv) |
| `rtl8188eu` | USB | rtl8188eus (cameras named rtl8188eu, rtl8188eus) |
| `rtl8733bu` | USB | rtl8733bu (cameras named rtl8131bu, rtl8731, rtl8731bu) |
| `bcm43438`, `bcm43436` | SDIO | syn4343 (the A1 and B0 parts; the driver picks the firmware by revision) |
| `hi3881` | SDIO | hi3881 (its access point is ap0) |
| `ssv6155p` | USB | ssv6x5x with SV6155P or SV6256P (cameras named ssv6155, ssv6155p, ssv6256p) |
| `ssv6158` | SDIO | ssv6158 |
| `ssv6355` | USB | ssv6x5x with SV6355 |
| `swt6621s` | USB | swt6621s (its boot ROM leaves the bus after EXEC, back with four interfaces) |
| `txw901u` | USB | txw901u (its boot ROM runs the firmware with a reset, back on the bus) |
| `wq9001` | USB | wq9001 (its own access point is wlan1) |
| `ws73v100` | USB | ws73v100 (its boot ROM leaves the bus on QUIT, back with five endpoints) |

`atbm6132cu-6162` is the ATBM6132CU part paired with Bluetooth, which
takes USB id 0x6162 in place of 0x6055 (the atbm6132cu package with its
6162 option). The ATBM6461 has a binary driver and no model. The
RTL8192FS's efuse holds the device's `mac`; its driver turns down a
locally administered address, QEMU's default 52:54:00 one among them,
and makes up one of its own. `rtl8733bu` is the family's Wi-Fi-only
part at 0BDA:F72B, as the RTL8731BU in thingino's cameras enumerates:
dual-band, no Bluetooth.

The Broadcom parts are FullMAC: the model plays their firmware, joining
the access point itself and leaving the 4-way handshake to the camera's
supplicant. They signal on a GPIO of their own and are powered from
another, `host-wake-pin` and `reg-on-pin` on the SoC's GPIO controller,
39 and 48 by default as thingino's syn4343 package has them.

The SSV6355 is FullMAC too, its firmware played the same way; for an
access point of its own the camera answers the management frames.

So is the AIC8800D80. It comes up as its boot ROM, A69C:8D80, which
takes the firmware; started, the chip leaves the bus and comes back as
A69C:8D81, with two Bluetooth interfaces nothing drives. The
AIC8800D40, which the driver takes for one, runs on this model too.

```bash
# a T31 camera with an SDIO ATBM6031
qemu-system-mipsel -M ingenic-t31 ... \
    -device atbm6031,bus=msc1,netdev=w0,ssid=thingino,psk=thingino123 \
    -netdev user,id=w0,hostfwd=tcp:127.0.0.1:2222-:22
# a T23 camera with a USB ATBM6012B-X
qemu-system-mipsel -M ingenic-t23 ... \
    -device atbm6012bx,bus=usb-bus.0,port=1,netdev=w0,ssid=thingino,psk=thingino123 \
    -netdev user,id=w0
# a T32 camera with an SDIO ATBM6162S on the SDHCI
qemu-system-mipsel -M ingenic-t32 ... \
    -device atbm6162s,bus=msc1,netdev=w0,ssid=thingino,psk=thingino123 \
    -netdev user,id=w0
```

The access point is `ssid` (default `qemu`) on 2.4 GHz `channel` 6 with
BSSID `bssid` (default 52:54:00:77:69:01): open without `psk`, WPA2-PSK
with CCMP when given a passphrase of 8 to 63 characters or a 64-digit
hex PMK. The camera scans and finds it, authenticates, associates and
completes the 4-way handshake; a wrong passphrase fails the handshake
after four tries and a deauthentication, as on a real access point. Data
then reaches the netdev as Ethernet frames, so `-netdev user` gives the
camera DHCP, DNS and host forwards. The access point takes one station
and speaks 802.11b/g only.

When the camera runs an access point of its own, as thingino's setup
portal does, a client can join it the way a phone would: give the chip
`client-netdev`, and `client-mac` set to the address that netdev sends
from. Once the chip has its BSS up, the client authenticates,
associates and carries the netdev's traffic. A user netdev on the
portal's subnet makes the client a host the portal can be reached
through, its MAC 52:55 followed by the `host` address:

```bash
qemu-system-mipsel -M ingenic-t31 ... \
    -device atbm6031,bus=msc1,netdev=w0,ssid=thingino,psk=thingino123,client-netdev=c0,client-mac=52:55:ac:10:00:fa \
    -netdev user,id=w0 \
    -netdev user,id=c0,net=172.16.0.0/24,host=172.16.0.250,ipv6=off,hostfwd=tcp:127.0.0.1:8080-172.16.0.1:80
```

The portal then answers on port 8080, and what it is told takes the
camera onto the `ssid` network when it reboots. The client joins open
networks, and WPA2-PSK ones with CCMP on the chips that give it a
passphrase as `client-psk`: the MT7601U, whose driver runs no open
access point, and the Hi3881.

`-trace 'wifi_ap_*'` follows the station through the handshake and
`-trace 'wifi_client_*'` the client through joining. Each chip has its
own: `aic_*` the AIC8800's firmware messages, frames, station state and
returns to the bus; `atbm_*` the ATBM driver's messages and register
traffic; `rtl_*` the Realtek chip's frames, channel changes and SDIO or
USB register traffic; `bcm_*` the Broadcom dongle's ioctls, events and
SDPCM frames; `ssv_*` the iComm-semi chip's host commands, or the
SSV6355's firmware messages and station state, its frames and USB or
SDIO register traffic; `mt7601u_*` the MT7601U's registers, MCU
commands, frames and channels; `wq9001_*` the WQ9001's firmware, IPC
messages, events, registers and frames; `hi3881_*` the Hi3881's
download, HCC packets, scans and frames; `txw901u_*` the TXW901U's boot
ROM, commands and frames; `ws73_*` the WS73V100's boot, HCC traffic,
messages and frames; and `swt_*` the SWT6621S's boot ROM, commands,
events and frames.

## Board I2C devices

Some boards put an I2C bus on plain GPIOs, not on a controller's pads, as
the kernel's `i2c-gpio` does. `ingenic-gpio-i2c` is that bus, on any
Ingenic machine; the guest bit-bangs it through the GPIO registers.

| Property | Meaning |
|---|---|
| `sda`, `scl` | the two GPIO numbers (port * 32 + pad: PB25 is 57) |
| `gpio` | the GPIO controller, when there is more than one (rare) |
| `id` | the bus's name for `bus=`; `gpio-i2c<sda>` without one |

Any number can coexist, each on its own two pads. A Wyze Cam Outdoor v2
(T31ZX) has a CellWise CW2015 battery gauge on PB25 and PB26 (the kernel's
`/dev/i2c-1`, `1-0062`) and its board MCU on PB22 and PB19 (`/dev/i2c-2`):

```bash
-device ingenic-gpio-i2c,id=i2c-gpio1,sda=57,scl=58 \
-device cw2015,id=gauge,bus=i2c-gpio1,voltage-mv=3900
```

`cw2015` is the gauge, at 0x62 unless given an `address`. It follows the
CW2015-DS V2.1 datasheet: it powers up asleep (MODE 0xc0), a write of 0 to
MODE wakes it, VCELL and SOC arrive 10 ms later and every 250 ms after,
and a quick start or a POR in MODE restarts them (a POR also puts it back
to sleep). The 64-byte profile at 0x10 and UFG in CONFIG are kept through
a POR, as the part keeps them in flash. ALRT in RRT_ALRT sets when SOC
falls to ATHD or below and only the host clears it; the `alrt` output is
its pin, low while it is set. The battery is set with `qom-set` while the
guest runs:

| Property | Meaning |
|---|---|
| `voltage-mv` | the cell voltage (default 4000); under 2500 the part sleeps |
| `soc` | the charge in percent; -1 (default), the part estimates it from the voltage |
| `rrt` | the remaining run time in minutes (default 0) |

```bash
qom-set /machine/peripheral/gauge voltage-mv 3600
```

## Audio

Every T-series SoC's audio controller (AIC) and internal codec are
emulated, as are the A1's AIC0 and codec, and the digital microphone (DMIC)
of the T30, T31, T32, T33, T40 and T41, so the SDK's `audio.ko` and
`libimp` record and play at the rate the driver sets: the codec's rate on
the T series, 1/256 of the CPM's I2S0 clocks on the A1, whose AIC is the
I2S master. The A1's AIC1, which feeds HDMI, is a stub. The speaker goes
to an audiodev, for instance a WAV file.
A microphone hears the audiodev's input unless its device's `mic` sets a
test tone (`tone[:<Hz>[:<dBFS>]]`, 1 kHz at -20 dBFS by default) or a
16-bit PCM WAV file to loop: `ingenic-aic` for the codec, `ingenic-dmic`
for the T30, T31 and T40 DMIC, `ingenic-as-dmic` for the T32, T33 and T41
one.

```bash
qemu-system-mipsel -M ingenic-t31 ... \
    -audiodev wav,id=snd0,path=speaker.wav -machine audiodev=snd0 \
    -global ingenic-aic.mic=tone:1000 -global ingenic-dmic.mic=tone:700
```

Gains, mutes and filters are registers only; vendor `libimp` adds its own
software gain, 10 dB at raptor's default volume. The 4.4 kernels'
`audio.ko` drives the DMIC only when loaded with `dmic_enable=1`; the 3.10
ones need `CONFIG_JZ_TS_DMIC` (thingino's `BR2_THINGINO_AUDIO_DMIC`).

## Zeratul fast boot

Ingenic's Zeratul SDK (Tassadar for the T32 and T33) boots battery
cameras fast: on a T31, T23, T32, T33 or T41 the SPL loads AE/AWB firmware
into a small RISC-V core, which runs the sensor and the ISP while Linux
loads, and the ISP driver then stops it and takes over its exposure and
white balance.

The core runs in `qemu-system-riscv32` (machine `ingenic-t31-riscv`,
`ingenic-t23-riscv`, `ingenic-t32-riscv` for a T32 or T33,
`ingenic-t40-riscv`, `ingenic-t41-riscv` or `ingenic-a1-riscv`), linked to
the SoC's QEMU over a UNIX socket.
`qemu-bsp.sh -R` starts the two together:

```bash
qemu-bsp.sh -i zeratul.bin -V t31zl -m 64M -R \
    -E "-device image-sensor,model=gc2053,bus=i2c0"
```

or by hand, the SoC's QEMU first:

```bash
qemu-system-mipsel -M ingenic-t31,memory-backend=ddr -m 64M ... \
    -global ingenic-t31.soc-variant=t31zl \
    -object memory-backend-shm,id=ddr,size=64M \
    -chardev socket,id=rv,path=rv.sock,server=on,wait=on \
    -global ingenic-riscv.chardev=rv
qemu-system-riscv32 -M ingenic-t31-riscv,socket=rv.sock -display none
```

The two share DDR, which must be a shared memory backend, and the T31
core's SRAM; everything else the core reaches (the sensor over I2C, GPIO,
the ISP, its timer and mailbox) is the SoC's model, across the socket. The
core's QEMU takes `-gdb` and `-d` as any does (`--riscv-args` passes them
through `qemu-bsp.sh`), and quits when the SoC's does. Stock Zeratul
firmware runs as the SDK's does: a 360 doorbell's (T31, sc4236), a
Wansview A1's and a Cinnado B6's (T23ZN, os02g10), and a eufyCam 3C's
(T41ZN, os08c10). So does a T32ZN image built with the Tassadar 2.0.0
SDK (gc2063) on its 3.10.14 kernel, whose prebuilt sensor objects are
built for it (on 4.4.94 the ISP driver oopses registering the sensor):

```bash
qemu-bsp.sh -i ZRT_PRJ007ZN_GC2063.bin -V t32zn -m 128M -R \
    -E "-device image-sensor,model=gc2063,bus=i2c0"
```

The eufyCam's core first reads the light from a TI OPT3001 it bit-bangs
on I2C1's pads (PC19, PC20), and sets its first exposure from it. Give it
one with `-device opt3001,bus=i2c1,lux=500` (at 0x44; `qom-set` changes
`lux` while it runs); without it the core sees 0 lux.

The cores run in machine mode and start at +0x80 from where the SoC boots
them. They are three designs, told apart by the CSRs read on cameras:

| Core | SoCs | ISA | mvendorid, marchid, mimpid |
|---|---|---|---|
| zero-riscy-like | T31 | RV32IMC | 0, 0, 0 (misa reads 0) |
| Victory0 | T32, T33, T23 | RV32IMC | 0x129, 0x9203, 1 (T32); 0, 0, 0 (T23) |
| T41/A1 little core | T40, T41, A1 | RV32IM | 0x4ee, 0x8000000c, 0x010a0202 |

Each is a CPU model of its own (`ingenic-t31`, `ingenic-victory0` and
`ingenic-t23`, `ingenic-t41-little`) whose CSRs read as they do on a
camera: the T31's misa, IDs and counters read 0, its mtvec reads where it
boots, and no CSR traps; Victory0 has U-mode and takes vectored traps
only, at a 256-byte aligned mtvec; the little core's misa reads 0x40000100
(I alone).

- The T31's boots from where 0x1222100c says: its SRAM (0x12210000) from
  power-on, though DDR serves as well. It starts only if bit 0 of
  0x12221004 is set when the SPL releases it (0x12221008 = 0). Its vectors
  are laid out where it boots as PULP's zero-riscy lays them out:
  interrupt id N at +4N. An exception goes to the base, not to
  zero-riscy's +0x84 and +0x88. Its clock is the CPM's CLKGR0 bit 26;
  gated, the core stops and its registers read 0. As on zero-riscy, wfi
  does not wake while interrupts are off (mstatus.MIE clear).
- The T23's boots in DDR, where its SPL copies it (0x12220004 says where),
  and sets its own vectors. Linux's stop is its software interrupt.
- The T32's (and T33's) boots in DDR too, where its SPL copies it
  (0x12220004 says where), while bit 31 of 0x122200f0 holds it. The
  host's word in 0x12220008 is its local interrupt 16, bit 0 of 0x12220030
  its software interrupt; its own word in 0x1222000c is the host's
  interrupt 4. It counts (0x12220010) at its clock, which the CPM's
  RISCVHCDR (0x9c) divides from APLL or MPLL, and reaching 0x12220018
  raises its timer interrupt. 0x12220028 mirrors the SoC's interrupt
  lines (the ISP's 29, the VIC's 30), which 0x12220020 lets through to its
  external interrupt. Its clock is the CPM's CLKGR0 bit 22.
- The T41's runs from DDR at the reset address in 0x12a00004 (0 from
  reset) unless bit 31 of the CCU's 0x12200fe0 holds it: its SPL sets the
  bit, loads it, sets the address and clears the bit. It counts at
  500 MHz and sets its own vectors. A T41 Zeratul image may keep its
  rootfs on a second NOR flash: `-drive file=...,format=raw,if=mtd,index=1`.
- The T40's is the T41's, held from reset (0x12200fe0 reads 0x80000018 on
  a T40XP), its count still until it is released. No T40 firmware runs it
  that we know of, its Zeratul included, but a program loaded and released
  as the T41's SPL does runs as on a T40XP.
- The A1's is the T41's. Its SDK (1.7.0) has no fast boot but a driver and
  a sample (`drivers/misc/sample_riscv_rotate`) that load a program into
  DDR and hand it pictures to rotate through the mailbox; the core's answer
  in 0x12a0000c is the A1's interrupt 2, and the sample's pictures come out
  as the driver expects.

The T32's, T33's and T32Pro's neural engine has a core of the T23's design
(misa 0x40101104, IDs 0), which the vendor's tnpu.ko (Magik's uranus) boots
and hands tasks. `qemu-bsp.sh -N` runs it in a second
`qemu-system-riscv32` (machine `ingenic-tnpu`), beside `-R`'s core or
alone. Its control unit is at 0x12700000: bit 31 of 0xf0 holds it (from
reset), and released it starts at 0x04's address (its window, 0x12608000;
the T32Pro's, 0x150b0000) + 0x80. The host's word in 0x08 is its software
interrupt, its own in 0x0c the SoC's interrupt 0. It counts (0x10) at the
CPM's TNPUCDR (0x98) while bit 5 of 0x00 is set, and bit 3 of 0x00 says
it sleeps. The vendor's boot code and a task of ours run as on a T32LQ;
the tensor units its tasks drive (the TNNA, the CVA) are not modelled.

The firmware converges on the AE and AWB statistics the ISP model
computes from the scene the sensor model renders; Linux's ISP core gets
them too. Both are in linear light, as a T31's and a T41's are: AE's
luma with the scene's gamma undone, AWB's sums raw as well, the white
balance taken back out so that a grey is as an OS04D10 sees daylight, of
only the quads in the drivers' white region, as a T31X, a T40XP and a
T41NQ count them. On a sensor model that follows its exposure (gc2053,
gc2063, os02g10, os08c10, sc4236) the AE converges as on a camera, over
a few frames, and the drivers' AWB settles near that daylight (4300 to
4600 K on the Wyze Cam 3 and 4, T40 and Tassadar images). The T32's
Tassadar core processes 4 frames before it stops and hands Linux its EV
and AWB gains. The T40's and T41's AWB blocks sort a zone's quads into
luma bands as well. A core sees DDR again at 0x20000000, where its
firmware reaches the SPL's uncached pointers.

Once Linux has stopped it, a core does whatever its firmware does next,
as it would on the camera. The SDK's T31 firmware parks the core in wfi
with interrupts off, which costs nothing. A 360 doorbell's spins in place
and a eufy lock's acks the mailbox every 2 ms for good, so their core's
QEMU keeps a host thread busy for as long as the guest runs. Tassadar's
T32 kernel restarts its core into a second firmware at the top of rmem,
which polls the VIC every 100000 counts of a 900 MHz clock for a stop
command; each poll reads across the socket, so that one loads both QEMUs.
A T31 core reads its timer's count from its own clock rather than across
the socket, so its delays do not load the SoC's QEMU.

The T21's manual and kernel name a RISC-V clock gate and interrupt, and a
T21N has the T31's timer page at 0x12220000 and its control page at
0x12211000, but no SRAM, and its core never ran there. The T10, T20 and
T30 have none of it.

Without `ingenic-riscv.chardev` the T31's core is not run: once the SPL
releases it, `ingenic-riscv` counts frames into the result block in DDR
with the ISP core's own starting values and answers the driver's stop, so
a Zeratul T31 image still boots to its login (a T23 one boots without the
core's results):

| Property | Default | |
|---|---|---|
| `ingenic-riscv.frame-rate` | 60 | frames a second it counts |
| `ingenic-riscv.night` | off | night mode, as U-Boot's light sensor would set it |
| `ingenic-riscv.result-addr` | 0x03700000 | the result block |

The fast-boot SDKs check words Ingenic computes from the chip ID, and the
model computes them the same way for any `efuse-chipid`:

| Parts | Word | Value | Checked by |
|---|---|---|---|
| T30Z, T31ZL/ZX/ZC, T31A, T23ZN/ZX, T32 and T32 Pro Z and V | 13 | the chip ID's signature | the SPL (non-zero), libimp ("Invalid chip type" otherwise) |
| T40, T41 | 19 | the signature | libimp, the SPL |
| T23DL/DN, T32DL | 12, bits 23:16 | a crc8 of word 0 | libimp (and its clock limiter) |
| T33 Z, V and DL | byte 0x213 | 0x80 \| a six-bit key of the ID | libimp, the SPL |

A T30Z, a T31ZX (a Wyze Cam Pan v2) and a T23DL read so, word for word;
word 13 reads 0 on parts sold for other firmware (a T31X, a T23DL).
`-global ingenic-t31.efuse-word13=<n>` sets word 13 on any of these SoCs.

The T40's Zeratul (SDK 2023-04-10) has no core in its fast boot: the SPL
reads the tag, sets the start EV from the light sensor (SADC, `aux0` and
`aux1`: `-global ingenic-sadc.aux0=<mV>`) and the tag's AE table into the
same result block, and boots the kernel without U-Boot; the kernel's
built-in ISP driver starts the sensor at that EV. Its libimp runs on a
T40N, XP or A (`-global ingenic-t40.soc-variant=t40n`), not a T40NN, and
checks eFuse word 0x13 against a hash of the chip ID, which the model
answers. An image built from the SDK for a gc2063, with the sample
encoder its fast-boot guide adds, records from its first frames:

```bash
qemu-system-mipsel -M ingenic-t40 -m 128M -display none \
    -global ingenic-t40.soc-variant=t40n \
    -drive file=zrt-t40n-gc2063.bin,format=raw,if=none \
    -device image-sensor,model=gc2063,bus=i2c1 \
    -serial null -serial stdio
```

## Wyze Cam Outdoor v2 MCU

A Zeratul battery camera has a part beside the SoC that powers it, of the
board vendor's choosing (an STM32, an ATBM6441 or ATBM6431, an XR871...).
The Wyze Cam Outdoor v2's is an STM32L0 that runs the SoC's own power,
the Wi-Fi's, the PIR's and a red LED, an RTC with two daily alarms, a
watchdog, and the button, charger and PIR inputs, all read and set over
I2C at 0x11 by Ingenic's `zrt_cam_daemon` and the board's `od_cam`.
`-device wyze-wco2-mcu` models Wyze's firmware 02.01.09 and answers as it does: only after a rising edge on its
wake pin (SoC GPIO 53) and for the one command that edge starts, with a
CRC-8 after each 8-byte status and data.

On the Wyze Cam Outdoor v2 the MCU is on a bit-banged bus, SDA 54 and SCL
51, next to the fuel gauge's (SDA 57, SCL 58):

```bash
qemu-bsp.sh -i wco2.bin -V t31zx -m 128M -R \
    -E "-device image-sensor,model=gc2063,bus=i2c0 \
        -device bcm43438,bus=msc1,netdev=w0,reg-on-pin=-1 \
        -netdev user,id=w0 \
        -device ingenic-gpio-i2c,id=i2c-gpio1,sda=57,scl=58 \
        -device cw2015,bus=i2c-gpio1,voltage-mv=3900 \
        -device ingenic-gpio-i2c,id=i2c-gpio2,sda=54,scl=51 \
        -device wyze-wco2-mcu,bus=i2c-gpio2,id=mcu"
```

| Property | Default | |
|---|---|---|
| `wake-pin` | 53 | SoC GPIO the wake edge comes from |
| `int-pin` | 10 | SoC GPIO the 10 ms interrupt pulse goes to |
| `wifi-pin` | -1 | SoC GPIO for the Wi-Fi power, when no `bcm43438` is there |
| `version` | `02.01.09` | what `0xc001` returns, up to 8 characters |
| `quit-on-power-off` | off | quit when the MCU cuts the SoC's power |

The Wi-Fi power is wired to the `bcm43438`'s `reg-on` when there is one.
The outputs read back with `qom-get` as `out-wifi`, `out-pir`,
`out-red-led`, `out-int`, `out-master` and `out-charge-led`.

The inputs change at run time with `qom-set`, and set the event flags the
firmware does: `button` (two presses are a second event), `pir`,
`pir-b`, `pir-adc` (the sample, 12 bits, 2048 at start), `wifi-wake`,
`battery-alert`, `usb-power`, `charge-stat` and `power-switch` (on at
start). The RTC starts at the host's time and runs on QEMU's RTC clock,
so `-rtc clock=vm` puts it on virtual time.

The SoC's power is the `out-master` output. When the MCU drops it, as a
camera going to sleep asks it to, the SoC's CPU stops while the board and
the MCU run on, so a PIR or button event or an alarm wakes it. When the MCU raises it again (that wake, its watchdog when the
SoC stops feeding it, a recovery command), the machine is reset, as from
a cold boot; with `quit-on-power-off` the drop quits instead. The stock image's daemons enable the watchdog, so a
guest that stops feeding it reboots.

## Timing

Guest time follows the host clock and the CPU clock comes from the CPM,
so U-Boot and Linux delays last as long as on the boards. Code runs at
QEMU's own speed, measured against T21, T31 and T41 silicon: integer code
1.5-3x faster than the chip, FPU code 2-5x slower, code that misses the
guest TLB 7-25x slower (each refill is an exception).

The `ips` plugin caps the instruction rate (it only slows QEMU down):

```bash
qemu-system-mipsel ... -plugin plugins/libips.so,ips=1030m
```

Instructions per CPU clock measured on the silicon:

| Code                     | XBurst1 (T21, T31) | XBurst2 (T41) |
|--------------------------|--------------------|---------------|
| ALU loop                 | 0.90               | 1.31          |
| qsort                    | 0.74               | 0.71          |
| unaligned copy           | 0.52-0.57          | 0.68          |
| dependent floating point | 0.27               | 0.66          |

For typical integer code, `ips` = 0.74 x the CPU clock: T31X (1392 MHz)
1030m, T41NQ (1400 MHz) 1000m, T20X or T21N (about 860 MHz) 640m.

The two cores of the T40, T41 and A1 see each other's memory accesses in
program order, as on the silicon; fencing the host for that costs those
SoCs about 10 % of their speed.

## Cache coherency checker

The XBurst L1 caches are write-back and do not snoop DMA, so a missing
cache flush or invalidate works in QEMU and fails on a board. The
`coherency` plugin follows every L1 line and reports where the guest
relies on coherence the silicon lacks:

```bash
qemu-system-mipsel ... -plugin plugins/libcoherency.so,ram=128M,l1=32K \
    -d plugin -D coherency.log
```

`ram` is the guest RAM size, `l1` the size of each L1: 16K on the T21 and
T23, 32K on the others. A report marked `latent` concerns a line that an
LRU model of the cache had already evicted, so the board most likely got
away with it. The report kinds are described at the top of
`contrib/plugins/coherency.c`. Boots take about 2.5x longer.

## Sample firmwares

The Thingino project publishes firmware images for each supported camera
model: <https://github.com/themactep/thingino-firmware>
