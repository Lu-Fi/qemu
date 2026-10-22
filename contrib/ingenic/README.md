# Ingenic XBurst1 helper scripts

Companion scripts for booting and driving the Ingenic T-series machine
types (`ingenic-t10` through `ingenic-t33`) provided by this QEMU fork.
Distributed alongside the `qemu-system-mipsel` binary in release tarballs;
also usable in-tree from the build directory.

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

The variant selects EFUSE/PRid bits, PLL config, and RAM size — read by
the firmware's `soc` tool and libimp.

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
driver probes for, and sends frames while that driver has it streaming.
It sees eight colour bars over a luma ramp, crossed by a white stripe
every two seconds. Its options:

- `model`: cv8001, gc2023, gc2053, gc2063, gc3003a, gc4023, gc4653,
  gc4663, gc5613, imx307, imx327, jxf22, jxf23, jxf28, jxf37, jxh42,
  mis20c1, os04d10, os05a10, os08a10, ov2710, ov9732, ov9750, sc2135,
  sc2232, sc2232h, sc2235, sc2310, sc231hai, sc2336, sc301iot, sc4236,
  sc8238 or sc835hai
- `bus`: the I2C bus the board's sensor sits on: `i2c0` on most
  cameras, `i2c1` on some T40 boards
- `address`: another I2C address, for a board that straps the sensor
  differently (the model's usual one by default)
- `fps`: the frame rate, 30 by default
- `noise=on`: luma noise that changes every frame, for pictures an
  encoder cannot code small

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
from outside, at run time, through QOM properties of the SoC. Set them
from the QEMU monitor; with the console on stdio, put the monitor on a
socket (`-monitor unix:qemu.mon,server=on,wait=off`, or `-E` that to
`qemu-bsp.sh`) and reach it with `socat - unix:qemu.mon`:

```bash
(qemu) qom-set /machine/soc/gpio in-pb 0xefffffff   # PB28 low: press a button
(qemu) qom-set /machine/soc/sadc aux0 300           # darken channel 0's photoresistor
```

`in-pa` to `in-pf` are the levels outside drives onto each port's pads,
one bit per pin, all high (pulled up) until set; a pad set up as a GPIO
output reads what it drives. `aux0` to `aux7` are the 12-bit samples of
the SADC's channels, 2025 by default: a bright room to a photoresistor.

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

AltoBeam's ATBM60xx, Broadcom's BCM43438 and BCM43436, iComm-semi's
SSV6155P, and Realtek's RTL8189FS, RTL8189ES, RTL8192FS and RTL8188FU
chips are emulated as their Linux drivers see them, with a WPA2 access
point on the air for the camera to join. The firmware a driver uploads is accepted and ignored;
the model answers in its place. SDIO
parts go on the controller the camera wires them to, usually `msc1`: the
MSC on the T10 to T31, the SDHCI on the T32, T33, T40 and T41. USB parts
go on the OTG port, with `port=1` so no hub comes between:

| Device | Bus | Driver (thingino package) |
|---|---|---|
| `atbm6031` | SDIO | atbm6031, atbm6031x |
| `atbm6062s`, `atbm6162s` | SDIO | atbm6062s, atbm6162s (Wi-Fi 6 driver) |
| `atbm6012b`, `atbm6012bx`, `atbm6032`, `atbm6132u` | USB | same names |
| `atbm6062u`, `atbm6062cu`, `atbm6132cu`, `atbm6132cu-6162` | USB | same names (Wi-Fi 6 driver) |
| `rtl8189fs`, `rtl8189es`, `rtl8192fs` | SDIO | rtl8189fs, rtl8189es, rtl8192fs (cameras named rtl8189ftv, rtl8189etv) |
| `rtl8188fu` | USB | rtl8188fu (cameras named rtl8188ftv) |
| `bcm43438`, `bcm43436` | SDIO | syn4343 (the A1 and B0 parts; the driver picks the firmware by revision) |
| `ssv6155p` | USB | ssv6x5x with SV6155P (cameras named ssv6155) |

`atbm6132cu-6162` is the ATBM6132CU part paired with Bluetooth, which
takes USB id 0x6162 in place of 0x6055 (the atbm6132cu package with its
6162 option). The ATBM6461 has a binary driver and no model. The
RTL8192FS's efuse holds the device's `mac`; its driver turns down a
locally administered address, QEMU's default 52:54:00 one among them,
and makes up one of its own.

The Broadcom parts are FullMAC: the model plays their firmware, joining
the access point itself and leaving the 4-way handshake to the camera's
supplicant. They signal on a GPIO of their own and are powered from
another, `host-wake-pin` and `reg-on-pin` on the SoC's GPIO controller,
39 and 48 by default as thingino's syn4343 package has them.

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
`-trace 'wifi_client_*'` the client through joining, `-trace 'atbm_*'`
shows the ATBM driver's messages and register traffic, and
`-trace 'rtl_*'` the Realtek chip's frames, channel changes and SDIO or
USB register traffic, `-trace 'bcm_*'` the Broadcom dongle's ioctls,
events and SDPCM frames, and `-trace 'ssv_*'` the iComm-semi chip's host
commands, frames and USB register traffic.

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
