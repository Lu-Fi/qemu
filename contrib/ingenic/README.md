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

Per-machine memory maps and the state of every block (modelled,
register-accurate, register-less window, stub), generated from the model
by `devdoc.py`, are in [devices/](devices/index.md), with the boot paths
and how the models were checked in [devices/README.md](devices/README.md).
`devdoc.py --check` fails when the pages are stale; CI runs it.

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
1 GiB on an A1). Above the RAM, up to those 256 MiB, the DDR controller
reads 0 and drops writes, with no bus error and no mirror of the RAM, as
a T23DL (32 MB), T31X and T40N do: a vendor SPL that probes past the RAM
to size it carries on. The XBurst1 parts show their RAM again at
0x20000000 too, with the same hole above it (measured on a T20X, T21N,
T23N, T30X, T31X and T32LQ): a 3.10 kernel's `dma_common_mmap()` hands
user space that alias of a KSEG1 buffer. A T32 shows it at 0x30000000,
0x40000000 and 0x80000000 as well.

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

The T40, T41 and A1 NOR also answers the GigaDevice block protection that
the 4.4 `sfc-nor` driver exercises at probe: with SR3.WPS set every 4 KiB
sector (64 KiB blocks, 4 KiB in the first and last 64 KiB) powers up locked
and a program or erase of a locked one is ignored, until `0x39` unlocks it
(`0x36` locks, `0x3d` reads the lock, `0x7e` and `0x98` lock and unlock
all). The extended address register (`0xc5`, `0xc8`) is stored and read
back but does not change addressing, which the 4-byte mode makes moot.

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

A T31LC camera's sensor is the firmware's own choice: its sinfo module
probes its list in order (a mis2008 at 0x30 first, an os02g10 at 0x3c
among the last), so add the one the board carries with
`-device image-sensor,model=<sensor>,bus=i2c0` or the ISP gets no frames.
A Jooan's firmware takes either of those two and streams two H.264
channels (1920x1080 and 640x360) with each.

A T33L camera's own flash boots the same way, through the T33's bootrom.
Give it its chip's ID (eFuse words 0 to 2, 24 hex digits), its board's
inputs and its sensor, and run its TNPU core (`-N`, see below). PC1 low
boots from NOR; its factory-reset key, PA22, is active high and held low
by the board, so with every pad high the app takes it as held and resets
itself. The app feeds the watchdog only while its person detector, on the
TNPU, answers.

```bash
./qemu-bsp.sh -i t33l.bin -M ingenic-t33 -V t33l -m 64M -N -E "-bios t33_bootrom.bin \
    -global ingenic-t31.efuse-chipid=<24 hex digits> \
    -global ingenic-gpio.in-pc=0xfffffffd -global ingenic-gpio.in-pa=0xffbfffff \
    -device image-sensor,model=cv2005,bus=i2c0"
```

The hash engine (0x13480000) sits behind CLKGR0 bit 18 (SC_HASH) on a T31:
the bootrom leaves it set, and while it is set every offset reads
0x00108001 and takes no write, as on a T31X. Open, a block written with no
INIT starts from the SHA-256 IV, +0x14 and +0x1c read 0xdeadbeef, and a
store to +0x1c does nothing. The other parts have the registers but no gate
(measured on a T31X only).

On a T40 the 256 bytes at 0x1000f000, below GPIO port A, read 0 and drop
stores with no fault and no log, as on a T40XP; a T41 is not measured and
still faults the load.

Above the RAM and its aliases (from 0x30000000 on the T20 to T33, from
0x20000000 on the T40 and T41) up to 0xffffffff nothing answers on silicon
and a load never takes a bus error; it returns stale bus data. The model
reads 0 there and drops stores, under the RAM, its aliases and a T32LQ's
mirrors. Below 0x20000000 the unmapped windows are unchanged.

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
- `scene`: a YUV4MPEG2 file or FIFO to see instead of the bars (below)

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
An H.265 picture is coded on several host threads, `h265-threads` on
`ingenic-avpu`, `ingenic-hera` and `ingenic-radix` (0, the default, is
the host's cores up to 8; 1 codes on the thread that was called alone):
rows of CTBs make their mode decisions and reconstruction in a wavefront,
a CTB once the row above has made the one to its upper right, while the
calling thread writes the bits in order. The stream is the same bytes at
any count. In a P picture a thread cannot know whether a neighbour ends
skipped or intra, so it goes on from what the entropy coder would most
likely decide, counting bits as it would write them; the coder checks each
block against its final neighbours and makes it again where they differ.
A block that skipping all but reproduces is left for the coder to make,
which it does only when it cannot show that skipping wins, so a still
scene costs little. On a 4-core host a thingino T32 with its 4K main
stream over IVDC (`direct_mode=1`) at 20 frames a second has them coded at
their rate with 4 threads and 19 with 2, where one thread of the former
coder took about 6 of every 20: the rest overflow the IVDC.
The ISP's frames run in QEMU's main loop: a frame the host takes longer
over than its period (two sensors of 8 and 3 megapixels, as an eufy T8416
boots with) is followed by an idle gap as long as the work, so the guest's
CPUs get at least half the host's time and the camera still boots. Under
`-icount` the frames keep the sensor's period.

An H.264 P picture is by default what the model began as: each
macroblock skipped, coded with no motion, or intra. `h264-motion=search`
on `ingenic-avpu` and `ingenic-hera` (`off` is the default) has it search
for motion as a T31's AVPU does (vectors round the last picture's, by SAD
and the vector's bits; 8x8 transforms; the vector buffer at entry word
0x2e then holds them, and holds zeros without it), which matches the T31's
choice of skip, coded and intra on about 84% of macroblocks of moving
scenes and its streams' size within 20%, where the stand-in's were 3 times
too big. A search costs: a 1080p P picture takes a few hundred
milliseconds, and a guest streaming one comes out at a tenth of the
encoded frames. `h264-threads` (default 1, own property so the H.265
wavefront's count stays) spreads the search over that many threads for
about half the time, at the host's cores' expense: it can cost the guest's
ISP frames a third on a 4-core host. Use it to check a stream's size or
how a decoder takes a moving scene, not for frame rate.
`capture=DIR` on `ingenic-avpu` leaves each job's slice data, its NV12
source and its settings; an H.264 job's settings carry `mb_kind`, a hex
digit for each macroblock (0 I4x4, 1 I8x8, 2 I16x16, 3 P16x16, 4 skipped,
plus 8 for an 8x8 transform), which the byte-exact checks against silicon
read. `x-h264-types=<digits>` is for those checks only: a digit (0, 1, 2)
a macroblock in raster order forces the intra kinds an I picture codes,
silicon's own, so the slice data can be compared byte for byte; it is
not for guests.
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
-global ingenic-hera.x-stream-tap=/tmp/tap     # T32, T33 (and ingenic-jpeg, its JPEG)
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
  libimp writes an H.264 IDR picture's PPS extension byte (High profile)
  some time after the job, so a High profile IDR picture waits until its
  PPS is whole (seen on a Wyze Cam 4's second stream: its PPS lacked the
  byte in the tap's first looks and had it in what the guest's streamer
  handed out).
* HELIX (T10 to T23, T30 JPEG), HERA (T32, T33), RADIX (T30): libimp keeps no
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
code and header (for H.265 one a decoder takes as the first slice of a
picture: libimp may copy the data a moment before it writes the header, and
what the buffer held then can look like a start code), with the parameter sets that go with an IDR picture
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

### Scene from a video

With `scene=` the sensor sees a video instead of the bars, in every
picture the guest gets: each ISP channel, the raw Bayer frames, the AE
and AWB statistics, and so what the encoders code. The source is
YUV4MPEG2, 4:2:0, the format the frame tap writes; ffmpeg decodes
outside and QEMU takes no decoder. The bars stay the default, and show
too until a FIFO's first frame arrives.

A file loops, the guest's frame n showing the file's frame n modulo its
frames, so a run repeats exactly:

```bash
ffmpeg -i clip.mp4 -vf fps=25 -pix_fmt yuv420p -f yuv4mpegpipe clip.y4m
-device image-sensor,model=gc2053,bus=i2c0,fps=25,scene=clip.y4m
```

A FIFO is live: a thread of the model reads it as ffmpeg writes, keeps the
latest frame and a partial frame until it completes, and takes up a writer that goes
away and comes back (the picture stays as it was meanwhile; a new header
may change the size). The guest never waits on it. Start the writer
before or after the guest; `-re` keeps ffmpeg at the video's own pace,
as a camera is:

```bash
mkfifo /tmp/scene.y4m
-device image-sensor,model=gc2053,bus=i2c0,fps=25,scene=/tmp/scene.y4m
ffmpeg -re -stream_loop -1 -i clip.mp4 -vf fps=25 -pix_fmt yuv420p \
    -f yuv4mpegpipe -y /tmp/scene.y4m                  # a video file
ffmpeg -re -loop 1 -i face.jpg -vf fps=25 -pix_fmt yuv420p \
    -f yuv4mpegpipe -y /tmp/scene.y4m                  # a still picture
ffmpeg -f v4l2 -i /dev/video0 -vf fps=25 -pix_fmt yuv420p \
    -f yuv4mpegpipe -y /tmp/scene.y4m                  # a USB camera
ffmpeg -rtsp_transport tcp -i rtsp://camera/stream -vf fps=25 \
    -pix_fmt yuv420p -f yuv4mpegpipe -y /tmp/scene.y4m # a camera's stream
```

Each render takes the frame at the size it asks for (main, sub and raw
from one source), averaging the source pixels each lands on, so a source
of the main channel's size and aspect shows undistorted. The light and
its ramp, `kelvin`, IR and the IR-cut filter, `noise`, `sensor-noise`,
each input's mirror and flip and `stamp` apply on top, as on the bars;
`object` is the bars' and `grey` replaces the scene. A file or stream that
is not 4:2:0 YUV4MPEG2 is refused at start (`-pix_fmt yuv420p`).

### Lens correction

The T21's, T23's, T31's, T40's and T41's ISPs correct lens distortion along
each row (the HLDC, `hw/misc/ingenic-isp-hldc.c`) once its bypass bit is
clear: bit 20 of 0x40 on the T40 and T41 (0x6c for the T40's second
pipeline), bit 18 of 0x0c on the T23 and T31, bit 13 of 0x0c on the T21. The
block is at 0x15000 (T40, T41; 0x15080 for the T40's second pipeline),
0x9000 (T23, T31) or 0x2200 (T21), ten settings at 0x04 to 0x28 (k1 and k2, zoom, norm,
centre, the luma's then the chroma's) that take effect at a frame start
once asked for: through 0x44 bit 0 (luma) and 1 (chroma), or on the T21,
which has no 0x44, bits 0 and 4 of 0x00.

What tx-isp does with it, read from the vendor drivers and the sensor IQ
files: every IQ file of the T21 (36 checked) and T23 (33) and all but one of
the T31's (sc4236) and T40's (sc223a) bypass it, and the T41's tx-isp leaves
it bypassed; the T23's and T40's libimp can program it
(`IMP_ISP_Tuning_SetHLDCAttr`), the T31's and T21's only the tuning file's
block. A T41NQ, a T31X and a T40N have been measured: the settings read the
active copy and a write shows once 0x44 asks and a frame starts (which
clears 0x44; the T31 reads both its bits back, the T40 and T41 bit 0), 0x00
keeps 0x00010001, the T40's second block resets as the T41's, and the T31's
state words (0x2c to 0x40) differ from the T41's. The T41NQ's masks, remap
and stall rule, the T21's and T23's registers, the T21's latch and the T40's
second pipeline taking its frame size from its channel 0 are assumed. `ingenic-isp-hldc-test` runs each part's block and frames
through the correction against a reference remap.

### A1 video decoder

The A1's VC8000D decoder (0x13100000, IRQ 42) is modelled the way the
vendor `vdec.ko` and `libimp` drive it: the kernel starts a command-buffer
sequencer (VCMD), `libimp` fills the buffers with register writes to the
decoder core, and a core's enable bit decodes one picture of the stream
ring it was given: every slice of the picture, written to the picture
address as NV12 in the coded size, the stream position left where the
next picture starts. H.264 and HEVC (8-bit 4:2:0) are decoded; the
other codecs, the MMU and the reference-frame compression are not
modelled (a job in another mode only completes).

libimp reads a channel's pictures from a post-processor unit, not from the
decoder's pictures. Two units (register words 0x140 and 0x156; libimp uses
the one a channel's number differs from) take a window of the decoded
picture, scale it to the channel's output size and write it as NV12 or
NV21 to their own buffers. Measured against a bench A1N running the same
libimp calls: at the source size the output is byte-identical to FFmpeg's
NV12; a downscale is a separable polyphase filter, 8 bits between the
passes, with the coefficient table libimp builds in RAM (the registers give
its base, the tap counts and the step), and 98 to 99.5 percent of its
bytes equal the hardware's on 100x56 to 500x280 outputs of 640x360 and
1080p streams, the rest off by one. An upscale is Catmull-Rom
horizontally and a smoothstep blend of two rows vertically, 99.7 to 100
percent of bytes equal to the hardware's from 1.09x to 3x. Both are fitted
to the bench's output, not read from documentation. A downscale whose
unit has no tap count or table (libimp's second channel, unit 0, as
sample-pip's 640x360 window of a 1080p stream programs it: ratio words in
the step register, nothing at 0x1b5-0x1bc) is modelled as the mean of the
source samples each output sample spans: assumed, not measured; it agrees
with FFmpeg's area scaler to one level in a 3:1 case. The units' tiled, planar and RGB
outputs are logged as unimplemented. HEVC pictures are written at the job
that decodes them (the SPS libavcodec sees asks for no reordering).

The bitstream decoding is libavcodec's, linked only when QEMU is
configured with `--enable-ingenic-vdec-avcodec` (off by default; it needs
the libavcodec 61 and libavutil development files). Without it the
device answers the same protocol and completes every picture as mid-grey,
and logs once that decoding is not built in. With it, pictures are
bit-exact with FFmpeg's decoder on the same stream (checked on the
SDK's 640x360 Main-profile H.264 and Main HEVC streams, whose decode
order is display order: a stream with reordered pictures would come out
in decode order, as the hardware writes it). One decoder is kept per
stream ring, and QEMU refuses to migrate while one is open.

The release builds have it: they link a static libavcodec and libavutil
from FFmpeg 7.1.5, unmodified, with only the H.264 and HEVC decoders
(`.github/scripts/build-ffmpeg.sh`), so they need no FFmpeg installed.
FFmpeg is LGPL 2.1 or later; its source is
<https://ffmpeg.org/releases/ffmpeg-7.1.5.tar.xz>. The same script builds
it for a local static build: `build-ffmpeg.sh $HOME/ffmpeg`, then
`PKG_CONFIG_PATH=$HOME/ffmpeg/lib/pkgconfig ../configure
--enable-ingenic-vdec-avcodec ...`.

The identity is what a bench A1N reads after `vdec.ko` has initialised:
VCMD +0x00 is 0x43421200, so the driver takes the new command protocol,
in which the interrupt handler finds the finished buffer by the id the
sequencer leaves in +0x68 (a JMP sets it from its fourth word); +0x04 is
0x20200618 and +0x08 to +0x3c are the read-only capability words, the
core's ID is 0x80018000 with build id 0x1f87, and the cache channel's
version is 5. Before the driver's init an A1N reads 0 in +0x08 to +0x38,
+0x3c 0x00400000, +0x40 0x20, +0x4c 0x80008000 and +0x5c 0x00100000;
`vdec.ko` neither reads nor writes the capability words or +0x3c, so the
model shows them from the first start of the sequencer (the last write of
the init), which is an assumption: the bench can say when they appear.

A libimp of the 1.5 series (a stock A1 NVR image's) runs a job in decoding
mode 0 for every channel it creates. Mode 0 is not modelled (the job only
completes); the stock libimp is not known to take it in a bare sample, where
it picks mode 0xf like the SDK's. The windows of such an image were wrong
before the post-processor existed because their buffers were never written.

Trace the jobs with `-trace 'ingenic_a1_vc8000d_*'`: the command buffers,
each NAL unit the job walks, the job's stream and output addresses, and
a CRC of every picture written, luma and chroma, to compare with a host
decode (`ffmpeg -apply_cropping 0 -f rawvideo`).

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

## Silicon corpora

What the tests pin to silicon, so a model change that departs from a part
is a failing test on every build. None of it is a camera picture or vendor
code: inputs are generated in the test (xorshift noise or gradients), and
only what the part answered is kept, as a CRC32 or a register value.

| Where | Part | What | Origin |
|---|---|---|---|
| `tests/qtest/ingenic-ldc-test.c` | T32LQ (T41's engine takes the same jobs) | CRC32 of the output of 640x360 LDC jobs: noise, and gradients of 16 and 64 levels a pixel, with and without a 128 offset, for zoom alone, k1 alone and the vendor's table | the part's LDC run from user space over `/dev/mem`, Nov 2026 |
| `tests/qtest/ingenic-ldc-test.c` | T30X | CRC32 of two 1280x720 jobs | same method |
| `tests/qtest/ingenic-census-test.c`, `ingenic-census-regs.inc` | T10L, T20X, T21N, T23N, T23DL, T30X, T31X, T32LQ, T40N, T40XP, T41NQ, T41LQ, A1N | about 3000 peripheral registers (address, value): AES, AIC, DES, DMIC, DRAWBOX, DTRNG, GPIO, HASH, I2C, I2D, INTC, IVDC, MSC, PDMA, SADC, SFC, SSI, TCU, UART and the A1's, T32's and T41's ASDMIC | each part's registers read at the U-Boot prompt (reads only, no side effects), Oct to Nov 2026, kept where the model at reset read the same, twice |

The census table holds the registers on which the silicon and the model
agreed when the census ran. A register the firmware had programmed before
the read, and that the model leaves at its reset value, is not there; the
CPM has its own test, the RTC counts, and the eFuse differs per chip, so
those blocks are left out. A block the model stubs as zeroes appears where
the silicon read zero too, and the test cannot tell the two apart. The
other tests of this directory that check CRC32s, ramps and register values
against a part (JPEG, IPU, I2D, HLDC, CPM, UART and the others) name their
origin in their own headers.

Left out: the T32's k2-alone jobs, whose top rows (and a few others) the
part fills from its line buffer's earlier contents, and the out-of-range-zoom
job but for its first columns, where a few rows of the next column differ;
the lens-correction (HLDC) captures and the ISP statistics
captures, which are sensor pictures of a scene; the stock firmware and
vendor library dumps.

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
| `aic8800d40l-sdio` | SDIO | the same chip answering as a plain U02 (revision byte 3), for a 2024 vendor BSP that knows no H variant: the eufy T8416's `aic8800_bsp` reads the H variant's 0xc3 as an unsupported revision and asks for `fw_patch_table.bin` instead of the `_8800d80_u02` tables (the reboot loop of its app) |
| `aic8800dc`, `aic8800dw` | USB | aic8800 with the AIC8800DC USB model (cameras named aic8800dc; a DW is a DC without Bluetooth) |
| `atbm6031` | SDIO | atbm6031, atbm6031x |
| `atbm6041` | SDIO | wifi-atbm6041 (its 60xx driver downloads firmware); the eufy T86P2's atbm6041_wifi_sdio (IoT firmware, nothing downloaded) |
| `atbm6441` | SDIO | wifi-atbm6441 (the Jooan S7-U's IoT driver), with the MCU below |
| `atbm6062s`, `atbm6162s` | SDIO | atbm6062s, atbm6162s (Wi-Fi 6 driver); the 6162S answers as a Pan v4 unit does: chip version 0x30, extended capabilities 2, efuse trims 5, 0x38, 5, 6, 6, 0x25, 0, 0 and then `mac=` (the MAC is the property, 52:54:00 by default) |
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
6162 option).

The ATBM6041 and ATBM6441 are AltoBeam's Hera parts, chip version 0x64.
Their own firmware runs from flash, which is what their IoT drivers
(eufy's atbm6041_wifi_sdio, thingino's atbm6441_wifi_sdio) take for
granted: nothing is downloaded, four messages the driver drops come
first, and the host hands the firmware an SSID and a key in a join
request and hears the outcome in an event, scans and checks its
keep-alive the same way. A 6041 whose driver does download firmware (the
60xx one) gets the 60xx model's dialect instead; the chip tells by the
download. The 6441 adds the MCU of a battery camera: `chardev=` is the
UART it reports on (0x7b frames: PIR motion, the charger, a power info
request answered with `battery=` and `battery-mv=`), and the GPIO lines
`pir` and `charger` in and `wake` out. While the host sleeps (a power
save mode of light sleep or deep) the chip wakes it, over `wake` and with
an event that says why, for the reasons the host allowed: the PIR, a
frame carrying the heartbeat's wake key, a keep-alive left unanswered.

The ATBM6461 is the 6441's successor in the Wansview A1, a T23 battery
camera with the chip on `msc1`. Its driver is the IoT one, which
downloads nothing, but it sends the Wi-Fi 6 drivers' message header and
reaches the low-power core in the chip (the RTOS) through a general
command rather than over a UART. The command carries what the camera's
librtos library packs: a magic, the command id, a CRC and the payload.
What the model answers: the version (`qemu`), `battery=`, `battery-mv=`
and `charging=` as the battery status, the settings the camera writes
(PIR, detect range, watchdog, RTC mode, upgrade interval, firmware
date) read back where the firmware reads them, and success to the rest.
A connect command (`wifi_connect` in the camera's `mcu_test`) joins the
`ssid` network the way the IoT firmware's join does. The driver reads no
chip version, and the RHEA family's 0xb1 is assumed for it; the SDIO ID
is the 6441's, which its driver accepts along with every other.

The RTL8192FS's efuse holds the device's `mac`; its driver turns down a
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

## USB UVC camera

`usb-uvc-camera` is a UVC 1.1 webcam for the guest's USB host: the
guest's uvcvideo driver binds to it, `/dev/video0` appears and a V4L2
program gets frames. It is built for the A1's three DWC2 hosts (the SDK's
`sample-usbcamera` wants one) and goes on any DWC2 bus:

```bash
qemu-system-mipsel -M ingenic-a1 ... \
    -device usb-uvc-camera,bus=usb-bus.1,port=1
```

The three A1 hosts' buses are `usb-bus.0`, `usb-bus.1` and `usb-bus.2`,
for OTG0 (0x13600000), OTG1 and OTG2. The guest's kernel needs `CONFIG_USB_VIDEO_CLASS` and the V4L2 core: the
SDK's A1 kernel has no media support, so its demo image cannot bind the
camera as shipped.

The descriptors are an interface association with a video control
interface (a camera terminal, a processing unit with brightness and
contrast, a USB output terminal) and a video streaming interface with
three formats: MJPEG at 1920x1080, 1280x720, 640x480, 640x360 and
320x240; YUYV and NV12 at 640x480, 640x360 and 320x240. Every size
runs at 30, 15 or 1 frames a second. The stream is bulk, with 12-byte
payload headers (FID, EOF, PTS and SCR) and 16 KiB payloads
(`payload=`); a payload that ends on a packet boundary is followed by a
zero-length packet. Isochronous streaming is not modelled, and neither
is H.264: the sample's `1` type (H.264) finds no such format.

The pictures are a colour-bar pattern that scrolls, with a square that
bounces; the frame number is stamped in the top left corner, as 16
binary cells (the most significant first, white for a 1) over six
decimal digits, so a test sees whether the picture moves. `stamp=off`
leaves it out. `sensor=ID` takes the scene of an `image-sensor` device
instead, its light, its video file and its noise included. The
brightness and contrast controls act on the luma. MJPEG is encoded by
the model itself (baseline, `quality=` 1 to 100, 85 by default), one
picture when the guest asks for it, so a 1080p stream costs the host
about 40 ms a picture. It is 4:2:2, as webcams send it, because that is
what the A1's JPEG block decodes into YUYV (a 4:2:0 file, which
`mjpeg-422=off` gives, comes out of it with its chroma scrambled).

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
I2S master. The A1's AIC1 feeds HDMI (`hdmi_audio.ko` enables it with no
codec on its pins) at 1/256 of the CPM's I2S1 clock: its frames go to the
HDMI transmitter's audio sink, which counts them while the transmitter's
audio path is enabled (HDMI mode, locked PHY, audio clock on, an I2S input
enabled in AUD_CONF0) and plays nothing. The counts are QOM properties of
`/machine/soc/hdmi` (`audio-frames`, `-dropped`, `-nonzero`, `-active`,
`-peak`, `-crossings`, `-rate`; `qom-get` from the monitor), so a test can
tell that a tone arrived and at what pitch. The speaker goes to an audiodev,
for instance a WAV file. The
TX FIFO level in AICSR follows the frames played, not the last threshold
crossing.
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

## A1 display

The A1's HDMI transmitter (`ingenic-a1-hdmi`: the controller at 0x13380000
and its PHY at 0x10075000) has a monitor on it, as the SDK's `vo.ko` and
U-Boot's VO driver use it: the receiver sense and PLL lock they poll, the
DDC master that reads the EDID, the interrupt on a finished transfer, and
the HPD pin on PB31. The monitor's EDID is 256 bytes (a base block with the
preferred mode, and a CEA-861 block with an HDMI vendor block and the
SDK's modes), from `xres` and `yres` (1920 by 1080) or a file:
`-global ingenic-a1-hdmi.edid-file=monitor.bin`. Unplug and plug it with
`qom-set /machine/soc/hdmi connected false` (or `true`): the HPD pin
follows and the driver's hot-plug thread sees it.

The A1's video display engine (`ingenic-a1-vde`, 0x13300000, IRQ 20) runs
frames at the rate of the timing the driver programs (the standard modes'
totals; `-global ingenic-a1-vde.refresh-hz=N` sets it), with the frame
and half-frame interrupts and the latch bits of the SDK's `vo.ko`. What its
mixer shows is a QEMU graphic console of its own, composed when the display
asks: the background, the V0 and V1 NV12 windows, the G layer (`fb.ko`'s
framebuffer, any of the 24 ARGB orders, with global or pixel alpha and the
colour key) and the M layer (the mouse cursor). `-display gtk` or `vnc`
shows it as the second console (the first is the BT output's), and the
monitor takes it by name:

```
(qemu) screendump /tmp/a1.ppm vde
```

Scaling and crop, the BCSH and sharpening stages and the test pattern are
not modelled.

The VDE, DSC, mixer and CSC sit behind CPM CLKGR0 bit 28 (gate_vde), shut
at power-on: while it is set the window reads 0 and takes no write, as on
an A1N. Open, +0x0 reads 0x20210731 and +0x4 0x11111111, and every register
resets to what an A1N reads (1080p sizes in TOP and V0, the read-only status
mirror at +0x800 to +0x84c, the mixer's background 0x00808000, the CSC's
BT.709 matrix, DSC's 2200 by 1125 timing) and keeps the bits it keeps
(V0 +0x80 0xffff0007, DMA +0x04 3, mixer +0x20004 0x17055557 and +0x20024
0xf1ffffff, the matrix 0x3fff3fff). TOP_LAYER_EN keeps nothing until
TOP_MODULE_EN is on. The HDMI status words (+0x400 to +0x414) and the PHY
read 0 with no monitor, and its mask and DDC words reset to the A1N's.
These were measured with the output not started; the colour key's enable
bit is not known, so a key whose ranges are both 0 hides nothing.

The VGA output's DAC (`ingenic-a1-vdac`, 0x10076000) reads 0 and keeps no
write: that is what an A1N shows with the VO running (0x10076200 to
0x100762ff), though `vo.ko` writes there. The rest of the page is assumed
the same.

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

Two more I2C parts the stock eufy T40 firmwares look for (both attach with
`-device NAME,bus=i2cN`; `qom-set` moves their readings while they run):
the T86P2's TI TMAG5273 Hall sensor on i2c3 at 0x35 (`tmag5273`: `x`, `y`,
`z` in mT, `temperature` in deg C, `version=2` for the 133/266 mT part; its
tmag5273 driver turns the X-Y angle into degrees) and the T8160's and
T8161's ambient light sensor on i2c2 at 0x53 (`ltr308als`: `lux` or raw
`counts`, which the firmware compares with 300000 and 500000; the firmware
never starts the ALS, so give `active=on` or it reads 0).

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
it sleeps. The vendor's boot code and a task of ours run as on a T32LQ.
The tensor engine its tasks drive (the TNNA at 0x12500000, the CVA at
0x12b00000) runs in the core's own QEMU on the T32 and T33, so that a
layer's millions of stores do not each cross the link; the T32Pro's two
cores share the SoC's. The T33's engine is the T32's at half the pixel
width (Ingenic's pixel number bit 3, the T32's 4): 8 pixels a MAC, two
PIXEL_OUT registers, 16 KiB of F-RAM with narrower WFCW entries, as its
vendor library and models program it; the SoC tells a linked core which
it drives. Without `-N` no task completes: a stock app whose
person detection runs on the TNPU, such as a T33L's, waits on it for good,
and one that feeds its watchdog only while its detector answers (the
T33L's `ovfs_core`) has the camera reset.

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
| T23DL/DN, T32DL | 12, bits 23:16 | a crc8 of word 0 (on the T23DL and T32DL 0x5a when that is 0) | libimp (and its clock limiter), the T23DL's SPL |
| T33 Z, V and DL | byte 0x213 | 0x80 \| a six-bit key of the ID | libimp, the SPL |

A T30Z, a T31ZX (a Wyze Cam Pan v2) and a T23DL read so, word for word;
word 13 reads 0 on parts sold for other firmware (a T31X, a T23DL).
`-global ingenic-t31.efuse-word13=<n>` sets word 13 on any of these SoCs.

The T41's fast-boot SPLs also compare SUBSOCTYPE2's lower half with their
own part's code and print "error ddr_check" for one they do not know: it
reads 0x1111 on a T41ZL and 0x4444 on a T41ZM. A T41ZG's SPL takes 0x3333
or 0x4444; which one a T41ZG carries is not known, and the model leaves 0.
`-global ingenic-t40.efuse-subsoctype2=<word>` sets the whole word. A T41ZM
image (sc401ai) boots to its login with the default and streams. A T41ZG
image prints "error ddr_check" with the default 0 and boots on regardless;
with 0xcccc3333 and with 0xcccc4444 it prints nothing and boots the same, so
neither value is told from the other here.

Ingenic's T23DL and T41ZM/ZG demo images (ZRT_SOC_*) start no application
and carry no streaming server: rcS has the SDK sample commented out, and
what the T23DL images ship in its place is a client of an imp_sync_server
that is in no image. The SDK's own `sample-Encoder-video` (built from the
release's userland, on its toolchain, for the tag's size and nrvbs) added to
`/system/bin` writes the stream to /tmp; with no network device, `dd` it
to the recovery partition (mtd4) and read it from the flash file. The
T23DL's SPL probes past its 32 MB of DRAM to size it, where the DDR
window reads 0 as on the part (`-m 32M`). Its 2024 images are
built for a gc2063 (named gc2053), its 2025 ones for a gc2053; the first
nrvbs frames of the main stream are the kernel phase's cache and are drawn
at the start exposure.

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
| `wdt-scale` | 1 | stretches the MCU's watchdog by this factor |

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

The watchdog is re-armed each time the SoC gets its power, and the stock
image disables it only once its init scripts run. A host that boots the
SoC slower than the 32 s default period is bitten into a reboot loop, so
give such a host `wdt-scale=10` or more.

### Sleep and wake

The stock app sleeps by itself when it has no task (`go_sleep_immediately`
on the console): it feeds the watchdog, sets the next alarm (`0x4009`),
turns the Wi-Fi off (`0x4004`) and sends `poweroff_master` (`0x0001`).
The MCU drops `out-master`, which stops the SoC's CPUs. Before that the app
powers the PIR (`0x4006`), but only on a paired camera: one with a
`[WIFI];SSID=...` token in the boot environment (`senv;...eenv;`) the SPL
hands the kernel. Raise an input to wake it:

```bash
qom-set /machine/peripheral/mcu button true     # then false
qom-set /machine/peripheral/mcu pir-adc 3500    # outside the PIR window
qom-set /machine/peripheral/mcu pir true        # then false
```

The charger does not wake it: the app disables charge wakeup (`0x401c`)
for its sleep. The machine resets as from a cold boot, and the app prints
the reason it read from the MCU (`wakeupFlag : 0X1` for the button, `0X20`
for the PIR). A PIR wake starts the app's alarm recording; the sensor and
ISP models do not yet deliver it frames, so the app gives up and sleeps
again. `wco2-sleep-e2e.py` runs the cycle on a stock image, paired in a
copy: boot, sleep, wake by the PIR, sleep, wake by the button, checking
the SoC stopped and cold-booted each time:

```bash
contrib/ingenic/wco2-sleep-e2e.py wco2.bin --wdt-scale 10
```

`-trace 'wyze_wco2_mcu*'` logs each command, output change and NAK.

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

## Watchdog

Each part's TCU watchdog resets the machine as its silicon does, as a
T20X, T21N, T23N, T30X, T31X, T32LQ, T40N, T41NQ and a T33L were measured
to. It counts what TCSR selects: RTC_EN at EXCLK/512 (OPCR's ERCS left 0),
PCLK or EXTAL (the T32 and T33 have no EXTAL input), or with none selected
it stands; TSR's WDTS stops it. A T20, T21, T23, T30, T31, T40 or T41
counts on with TCER clear, which then only holds the reset back; a T32's
and T33's count stands. A T21, T23, T30, T31 or T40 resets only as its
count reaches TDR, so a TDR written below the count waits for the 16-bit
counter to wrap round to it. A T32, T33 or T41 resets at its next count
with TDR at or below it, which on a T33 a clear (TCSR bit 10) written
straight after prevents and on a T32 or T41 does not: their clear lands
at the next count. A T41's WDTS holds the count only until TDR is written:
the count then runs on from where it stood, a clear written while it was
held is dropped, and TCNT reads the held count until TSCR; without a TDR
write it stays held, and such a clear lands as TSCR lets the clock run.
The vendor kernel's restart depends on that: it stops the watchdog, then
writes a clear, TDR 0x100 and TCER without letting WDTS go, and a T41NQ is
reset 0x100 counts after the last feed. The T32's and T33's TCNT takes no
write: their vendor kernels feed them through TCSR bit 10.

The TCU's channels count RTC_EN at the same EXCLK/512. RSR (CPM 0x08)
reads 0x1 after power-on; a watchdog reset adds 0x2 and keeps the rest,
and both stay until software writes them clear, as a T20X, T31X and T40N
read it.

A reset is QEMU's watchdog action: `-action watchdog=none`, `debug` or
`pause` keeps the machine running instead and logs the bite, for a guest
whose feeding falls behind where QEMU runs its code slower than the chip.

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

## L2 cache and the AI engine's SRAM (T40, T41, A1)

The T40, T41 and A1 share one on-chip SRAM between the L2 cache and the
AI engine: CCU MSCR bits 12:10 give the L2 its share (1 = 128 KiB, 2 =
256 KiB, 3 = 512 KiB, 4 = 1 MiB) and the NNA's ORAM is the rest, from
0x12600000 plus that share. Each MSCR write moves the ORAM window and
every core's Config2 (the L2 geometry Linux and U-Boot read), as on a
T40XP, T41NQ and A1N, whose bootroms leave MSCR at 0x00000c00, 0x02000800
and 0x02000c10. The SRAM is 1 MiB on the T40 and A1, and 512 KiB on the
T41, which shows it twice in its window. The NNDMA addresses it by offset,
the L2's share included: AI code built for a 128 KiB L2 that writes there
on a T40 left at 512 KiB corrupts the cache on silicon, and the model logs
a guest error.

## A1 media blocks

The A1's AIP at 0x13090000 is the T41's (`ingenic-aip`): the same T, F
and P units at +0x000, +0x200 and +0x300 with the registers of the SDK's
`aip-driver`, interrupts 40, 39 and 38 of the INTC (IRQ_AIP0 to
IRQ_AIP2), and the SDK's `sample_aip` runs on it. With the `a1` property
it reads as an A1N does: each unit's TIMEOUT (+0x0c) resets to 0xffffffff,
T's +0x10 to 0x00020002, T's CFG does not keep bit 10, a T step with no job
loaded leaves CTRL 0x9 and IQR 0x8, and F's TIMEOUT counts down 0xd4 cycles
over a job (only the 2x2 NV12 case is measured). Its IPU at 0x13080000
is not the T-series' `ingenic-ipu`; see below.

Its JPEG engine at 0x13200000 is a HELIX, the T23's (`ingenic-a1-helix`; JPGC
+0x20 reads 0, +0x60 0x00110800, GLBC keeps all 32 bits, the EFE's +0x0c and
+0x18 to +0x30 read 0x13200000; the SCH words and JPGC +0x04 to +0x1c hold
junk on an A1N at power-on and read 0 here): the
vendor kernel's `soc_vpu` helix driver (scheduler at 0x84, units SCH,
VDMA, EFE, SDE, JPGC at their T-series offsets), interrupt 33 of the INTC,
reset through CPM SRBC0 bit 22 (the stop request, bit 21, is acknowledged
at once in bit 20). libimp's `IMP_VENC` JPEG channels run on it.

## The A1's IPU

The A1's 2D engine at 0x13080000 (INTC source 46) is the one the vendor
`ipu.ko` and libimp's TDE drive, and it is not the T-series IPU. The
driver hands it a chain of descriptors in RAM at +0x14 and starts it with
+0x10; a descriptor is a header (the TDE operation, the length, the next
descriptor or 0xffffffff, two scaler tables) and register writes in
(count << 16 | register) groups that fill the BK, FG, MK and OT surface
registers from 0x1040, and a write of 1 to 0x1010 runs the job. The chain's
end sets STATUS bit 0 and raises the interrupt; +0x24 and +0x1018 clear
the status bits, and a job the model does not run sets bit 2 of +0x1014.
A job takes 2 ns a pixel of virtual time. Surfaces are in bytes of stride;
a negative FG stride reads the rows bottom up (libimp's overturn), OT bit 28
turns each row round (mirror), a constant FG (bit 25, the colour in INFO3)
fills, and the CLIP box keeps what it holds or all but that. libimp's copy,
fill, overturn, mirror and clip jobs on ARGB1555 pictures come out byte for
byte.

Scaling and conversion work on 8-bit A, R, G, B pixels (the chroma of the
NV12 types repeated over each block): the packed types 0 to 7 (RGB888,
RGB565, RGB555, RGB444, ARGB8888, ARGB8565, ARGB1555, ARGB4444) in the
channel orders 0 (ARGB), 5 (ABGR), 8 (RGBA) and 13 (BGRA), and NV12, NV21,
NV16 and NV61. The scaler is two passes of the 513-phase, eight-tap
tables libimp builds with a Lanczos curve (14-bit signed taps summing to
2048), horizontal from the descriptor's first table and vertical from its
second, the source position of output n being step * n + step / 2 - half a
pixel, the step (source << 14) / destination; the rounding, the origin and
the phase's last bit are the model's, not measured. The colour conversion is
the 3 x 3 matrix at +0x10d4 (14-bit, 10 fraction bits) on the three
channels, libimp's default BT.709 full range included.

The blend (OSD at +0x1108: the command and the two factor modes of the
TDE's blend options) takes FG over BK per channel with the factors of
the command (Porter-Duff on straight alpha) or the two modes, the
alpha a surface's pixel alpha or, with INFO2 bits 9:8 of 2, the global alpha
in INFO2 bits 7:0. Writing 1 to +0x1020, which every descriptor does first,
puts the job registers back to their reset values: a clip box or a
conversion does not outlive its job.

The registers read what an A1N reads: the version 0x20210708 at +0x0 and
+0x1000, +0x80 0x00140120, +0x2c (the status mask) 3 at reset, and 640 by
480 (0x028001e0) in each surface's SIZE, with the kept bits measured on it
(+0x1050 0xc2ffffff, +0x10c8 0x80000001, +0x10d0 0xffff0003, +0x10d4
0x3fff3fff, +0x1108 0x80000fff, +0x1110 0x01011111, +0x1114 0x001fffff).
+0x40 puts the register file back to its reset values too and clears the
interrupt mask at +0x101c, but keeps +0x14 and +0x2c (measured on an A1N);
+0x1024 clears nothing. While CPM CLKGR0 bit 29 (gate_ipu, set at power-on)
is set every register reads 0 and takes no write, as an A1N's does under
U-Boot (measured). The status mask resets to 3, so a driver unmasks the interrupt
(vendor `ipu.ko` writes 0). IPUMCDR (CPM 0xbc) reads 0 at reset and stores
what the kernel's clock driver writes.

## The A1 as a network and SMP guest

The A1's XGMAC (0x130b0000, dwxgmac2) takes a received frame only while
the driver has the receive DMA running (RXST at 0x3108 and the MAC's RE at
0x0004, as `dwxgmac2_dma_start_rx()` sets them). stmmac stops it, frees the
ring and allocates a new one whenever it resets the adapter; a frame
written into the old ring lands in memory the guest has reused.

The transmit DMA is gated the same way (TXST at 0x3104) and restarts at the
head of the ring whenever the driver writes the ring's address (0x3114).
The model used to keep its position across that. U-Boot leaves the position
in its own ring, so Linux's first frames were read from there and never
sent: the queue timed out ("NETDEV WATCHDOG", "Reset adapter") every 12 s,
and each reset freed the ring and allocated another. The next kick of the
tail pointer then read the freed ring as descriptors: a kernel pointer in
its last word has bit 31 set, which counted as an owned descriptor, and
clearing OWN wrote the pointer back without bit 31. That is the oops seen
in about one stock NVR boot in eight, always right after "Reset adapter" (a
task's stack pointer 0x8aff2000 read as 0x0aff2000, a page-table page's
address, an rb-tree node), with `-accel tcg,thread=single` as well as
multi-threaded.

With the fix, 48 stock-image and 48 stress-image boots (a fork/thread/COW
loop) of 900 s each, half of them single-threaded and half multi-threaded,
on two aarch64 hosts with user networking, ended with no oops and no "Reset
adapter". The control with the old model (12 single-threaded stress boots
on one host, no networking) ended in 2 oopses and 10 watchdog reboots, with
1 to 60 "Reset adapter" lines each.

With `-accel tcg,thread=single` the stress image's app dies of its own
watchdog ("hand ... purpose_id=0x9000 Is Dead", `dog_reboot`) about 12
minutes after power-on, in 24 of 24 boots, and the stock image in 1 of 24.
The multi-threaded default had none in 24 stress and 24 stock boots. The
cause is not found; the VC8000D is the same in both modes.

In the same runs a `usb-mouse` on the first USB bus was not enumerated by
the guest after U-Boot had used the port, so no pointer reaches the GUI;
the cause is not found.

## Sample firmwares

The Thingino project publishes firmware images for each supported camera
model: <https://github.com/themactep/thingino-firmware>
