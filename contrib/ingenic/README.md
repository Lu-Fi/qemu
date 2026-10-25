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
on the T40, T41 and A1. `nand-blocks` sets the size in 128 KiB blocks
(default 1024). The image holds the page data back to back, as thingino's
and Ingenic's NAND images do. Pad it to the chip size with 0xff, or what
the guest writes past its end is lost when QEMU exits:

```bash
head -c $(( (128 << 20) - $(stat -c %s fw.bin) )) /dev/zero | tr '\0' '\377' >> fw.bin
qemu-system-mipsel -M ingenic-t41 -m 128M -display none \
    -global ingenic-a1-sfc.nand-id=0xc851 \
    -drive file=fw.bin,format=raw,if=none,id=flash0 \
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
driver probes for, and sends frames while that driver has it streaming;
on the T23 and T31 a MIPI sensor's frames also need the CSI receiver out
of reset, as tx-isp leaves it only while the ISP is open. It sees eight
colour bars over a luma ramp, crossed by a white stripe every two
seconds. Its options:

- `model`: cv2003, cv8001, gc1084, gc2023, gc2053, gc2063, gc2083,
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
- `noise=on`: luma noise that changes every frame, for pictures an
  encoder cannot code small
- `light`: the scene's brightness in percent, 100 by default: a well-lit
  room. The gc2053, gc2063, os02g10, os08c10 and sc4236 follow their
  exposure registers, so the picture brightens and darkens with what AE
  sets, in linear light, and AE answers `light`; it can change while the
  guest runs (given `id=cam`, `qom-set /machine/peripheral/cam light 5`
  for dusk)

One sensor per machine. The encoders behind it:

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

## Wi-Fi

AICSemi's AIC8800, AIC8800D80, AIC8800DC and AIC8800DW, AltoBeam's
ATBM60xx, Broadcom's BCM43438 and BCM43436, HiSilicon's Hi3881,
Huge-IC's TXW901U, iComm-semi's SSV6155P, SSV6158 and SSV6355,
MediaTek's MT7601U, Realtek's RTL8189FS, RTL8189ES, RTL8192FS,
RTL8188FU, RTL8188EUS and RTL8733BU, and WuQi's WQ9001 chips are
emulated as their Linux drivers see them, with a WPA2 access point on
the air for the camera to join.
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
| `txw901u` | USB | txw901u (its boot ROM runs the firmware with a reset, back on the bus) |
| `wq9001` | USB | wq9001 (its own access point is wlan1) |

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
from. Once the chip has an open BSS up, the client authenticates,
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
networks only.

`-trace 'wifi_ap_*'` follows the station through the handshake,
`-trace 'wifi_client_*'` the client through joining, `-trace 'aic_*'`
shows the AIC8800's firmware messages, frames, station state and its
returns to the bus, `-trace 'atbm_*'` the ATBM driver's messages and
register traffic, and `-trace 'rtl_*'` the Realtek chip's frames,
channel changes and SDIO or USB register traffic, `-trace 'bcm_*'` the
Broadcom dongle's ioctls, events and SDPCM frames, and `-trace 'ssv_*'`
the iComm-semi chip's host commands, or the SSV6355's firmware messages
and station state, its frames and USB or SDIO register traffic.

## Audio

Every T-series SoC's audio controller (AIC) and internal codec are
emulated, and the digital microphone (DMIC) of the T30, T31, T32, T33, T40
and T41, so the SDK's `audio.ko` and `libimp` record and play at the rate
the driver sets. The speaker goes to an audiodev, for instance a WAV file.
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

Zeratul SPLs power the chip off or stop when eFuse word 13 reads 0, as it
does on a T31X. The `t31zl`, `t31zx`, `t31zc`, `t31a` and `t23zn`
variants read 1 there; `-global ingenic-t31.efuse-word13=<n>` sets it on
any of these SoCs.

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
